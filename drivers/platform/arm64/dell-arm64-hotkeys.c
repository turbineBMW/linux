// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dell hotkey events on Arm64 laptops
 *
 * Dell laptops report some hotkeys, such as mic mute, as WMI events: the
 * EC raises a notification, the firmware calls Notify(AMW0, 0xD0), and
 * _WED(0xD0) returns the event as 16-bit words (length, type, codes). On
 * x86, dell-wmi handles these through the ACPI-WMI core, which is x86
 * only. Arm64 Dell laptops such as the XPS 16 DX16263 (NVIDIA N1x) have
 * the same firmware interface, so this driver handles that one event:
 * it binds to the WMI device that declares Dell's event GUID, evaluates
 * _WED and reports the keys.
 *
 * The XPS 16 DX16263's EC never raises the notification (its FF-A
 * notifications do not arrive at all on BIOS 1.2.0), but it still queues
 * each event in EC RAM. On that machine the driver reads the queue the way
 * the firmware's _WED does. Its mic-mute LED can only be toggled, and the
 * EC keeps it lit across reboots, so the driver tracks it, exposes it as a
 * platform::micmute LED that follows the mute state rather than the key,
 * and turns it off at shutdown.
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#define DELL_EVENT_NOTIFY	0xd0
#define WDG_FLAG_EVENT		BIT(3)

#define DELL_POLL_MS		250

/* Dell WMI event GUID, 9DBB5994-A997-11DA-B012-B622A1EF5492 */
static const guid_t dell_event_guid =
	GUID_INIT(0x9dbb5994, 0xa997, 0x11da,
		  0xb0, 0x12, 0xb6, 0x22, 0xa1, 0xef, 0x54, 0x92);

/* One entry of the _WDG table, as in the ACPI-WMI core */
struct wdg_entry {
	guid_t guid;
	u8 notify_id;
	u8 reserved;
	u8 instance_count;
	u8 flags;
} __packed;

/*
 * Type 0x0010 codes, from dell-wmi. Speaker mute and brightness also come
 * from the keyboard on these machines, so their events are only
 * notifications here.
 */
static const struct key_entry dell_arm64_keymap[] = {
	{ KE_IGNORE, 0x00100000, { KEY_RESERVED } },	/* Fn-lock off */
	{ KE_IGNORE, 0x00100001, { KEY_RESERVED } },	/* Fn-lock on */
	{ KE_IGNORE, 0x0010003f, { KEY_RESERVED } },	/* Keyboard backlight changed */
	{ KE_IGNORE, 0x00100057, { KEY_BRIGHTNESSDOWN } },
	{ KE_IGNORE, 0x00100058, { KEY_BRIGHTNESSUP } },
	{ KE_IGNORE, 0x00100109, { KEY_MUTE } },
	{ KE_KEY,    0x00100150, { KEY_MICMUTE } },
	{ KE_IGNORE, 0x00100151, { KEY_RESERVED } },	/* Fn-lock */
	{ KE_IGNORE, 0x00100152, { KEY_KBDILLUMTOGGLE } },
	{ KE_END, 0 }
};

/*
 * An EC that queues WMI events in its RAM without notifying: reading the
 * length register hands out the queued event, which is then read a byte
 * at a time from the data register (the firmware's EC0A). The keys' LEDs
 * are toggled by passing an ID to MTLD, as _WED does for the speaker mute
 * (0x0109) and mic mute (0x0150) keys. The EC answers but never with the
 * LED's state, so the state cannot be read back.
 */
struct dell_arm64_quirks {
	u8 queue_len_reg;
	u8 queue_data_reg;
	u32 mute_toggle;
	u32 micmute_toggle;
};

static const struct dell_arm64_quirks dell_xps16_dx16263_quirks = {
	.queue_len_reg = 0x2b,
	.queue_data_reg = 0x2c,
	.mute_toggle = 0x0781,
	.micmute_toggle = 0x0676,
};

static const struct dmi_system_id dell_arm64_quirk_ids[] = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Dell Inc."),
			DMI_MATCH(DMI_PRODUCT_NAME, "XPS 16 DX16263"),
		},
		.driver_data = (void *)&dell_xps16_dx16263_quirks,
	},
	{ }
};

struct dell_arm64_hotkeys {
	struct device *dev;
	struct input_dev *input;
	acpi_handle handle;
	const struct dell_arm64_quirks *quirks;
	struct delayed_work poll;
	struct led_classdev micmute;
	/* Serializes the mic-mute LED's toggles with micmute_lit */
	struct mutex led_lock;
	bool micmute_lit;
};

static bool dell_arm64_has_event(acpi_handle handle)
{
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	const struct wdg_entry *entry;
	union acpi_object *obj;
	bool found = false;
	size_t i, count;

	if (ACPI_FAILURE(acpi_evaluate_object(handle, "_WDG", NULL, &out)))
		return false;

	obj = out.pointer;
	if (obj && obj->type == ACPI_TYPE_BUFFER) {
		entry = (const struct wdg_entry *)obj->buffer.pointer;
		count = obj->buffer.length / sizeof(*entry);
		for (i = 0; i < count; i++) {
			if (guid_equal(&entry[i].guid, &dell_event_guid) &&
			    entry[i].flags & WDG_FLAG_EVENT &&
			    entry[i].notify_id == DELL_EVENT_NOTIFY) {
				found = true;
				break;
			}
		}
	}

	kfree(out.pointer);
	return found;
}

static void dell_arm64_report(struct dell_arm64_hotkeys *priv, u16 type, u16 code)
{
	const struct key_entry *key;

	key = sparse_keymap_entry_from_scancode(priv->input, (type << 16) | code);
	if (!key) {
		dev_info_ratelimited(priv->dev, "Unknown event type 0x%04x code 0x%04x\n",
				     type, code);
		return;
	}

	if (key->type == KE_KEY)
		sparse_keymap_report_entry(priv->input, key, 1, true);
}

/* Report the first entry of a Dell WMI event buffer (16-bit words). */
static void dell_arm64_process(struct dell_arm64_hotkeys *priv, const u8 *buf,
			       size_t size, bool set_leds)
{
	size_t count = size / 2, len, i;
	u16 type, code;

	/*
	 * The firmware copies the event into a fixed buffer without clearing
	 * it, so anything after the first entry can be left over from an
	 * earlier event. Read only the first entry, as the firmware does.
	 */
	if (count < 2 || !get_unaligned_le16(buf))
		return;

	len = min_t(size_t, get_unaligned_le16(buf) + 1, count);
	type = get_unaligned_le16(buf + 2);
	switch (type) {
	case 0x0000:	/* One key or event */
		if (len > 2)
			dell_arm64_report(priv, type, get_unaligned_le16(buf + 4));
		break;
	case 0x0010:	/* Sequence of keys */
	case 0x0011:	/* Sequence of events */
		for (i = 2; i < len; i++) {
			code = get_unaligned_le16(buf + 2 * i);
			dell_arm64_report(priv, type, code);
			if (type != 0x0010 || !priv->quirks)
				continue;
			/* As _WED does; the mic-mute LED follows the mute state instead. */
			if (set_leds && code == 0x0109)
				acpi_execute_simple_method(NULL, "\\_SB.MTLD",
							   priv->quirks->mute_toggle);
			/* _WED toggled the mic-mute LED itself. */
			if (!set_leds && code == 0x0150) {
				guard(mutex)(&priv->led_lock);
				priv->micmute_lit = !priv->micmute_lit;
			}
		}
		break;
	default:
		dev_dbg(priv->dev, "Ignored event type 0x%04x\n", type);
		break;
	}
}

static void dell_arm64_read_event(struct dell_arm64_hotkeys *priv)
{
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object arg = {
		.integer = { .type = ACPI_TYPE_INTEGER, .value = DELL_EVENT_NOTIFY },
	};
	struct acpi_object_list args = { .count = 1, .pointer = &arg };
	union acpi_object *obj;

	if (ACPI_FAILURE(acpi_evaluate_object(priv->handle, "_WED", &args, &out))) {
		dev_warn_ratelimited(priv->dev, "Failed to read the event\n");
		return;
	}

	/* _WED toggles the LEDs itself. */
	obj = out.pointer;
	if (obj && obj->type == ACPI_TYPE_BUFFER)
		dell_arm64_process(priv, obj->buffer.pointer, obj->buffer.length, false);

	kfree(out.pointer);
}

static void dell_arm64_notify(acpi_handle handle, u32 event, void *data)
{
	if (event == DELL_EVENT_NOTIFY)
		dell_arm64_read_event(data);
}

static bool dell_arm64_ec_read(u8 reg, u8 *val)
{
	union acpi_object arg = {
		.integer = { .type = ACPI_TYPE_INTEGER, .value = reg },
	};
	struct acpi_object_list args = { .count = 1, .pointer = &arg };
	unsigned long long data;

	if (ACPI_FAILURE(acpi_evaluate_integer(NULL, "\\_SB.ECRB", &args, &data)))
		return false;

	*val = data;
	return true;
}

static void dell_arm64_poll(struct work_struct *work)
{
	struct dell_arm64_hotkeys *priv =
		container_of(to_delayed_work(work), struct dell_arm64_hotkeys, poll);
	u8 buf[128] = { }, len, i, val;

	if (dell_arm64_ec_read(priv->quirks->queue_len_reg, &len) && len) {
		dev_dbg(priv->dev, "EC queued a %u-byte event\n", len);
		/* The length is only read once: the event must be read out now. */
		for (i = 0; i < len; i++)
			if (dell_arm64_ec_read(priv->quirks->queue_data_reg, &val) &&
			    i < sizeof(buf))
				buf[i] = val;
		dell_arm64_process(priv, buf, min_t(size_t, len, sizeof(buf)), true);
	}

	queue_delayed_work(system_freezable_wq, &priv->poll,
			   msecs_to_jiffies(DELL_POLL_MS));
}

static int dell_arm64_micmute_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct dell_arm64_hotkeys *priv =
		container_of(cdev, struct dell_arm64_hotkeys, micmute);

	guard(mutex)(&priv->led_lock);
	if (!!brightness == priv->micmute_lit)
		return 0;

	if (ACPI_FAILURE(acpi_execute_simple_method(NULL, "\\_SB.MTLD",
						    priv->quirks->micmute_toggle)))
		return -EIO;

	priv->micmute_lit = !!brightness;
	return 0;
}

static void dell_arm64_stop(void *data)
{
	struct dell_arm64_hotkeys *priv = data;

	if (priv->quirks) {
		cancel_delayed_work_sync(&priv->poll);
		dell_arm64_micmute_set(&priv->micmute, LED_OFF);
	}
	acpi_execute_simple_method(priv->handle, "WED0", 0);
}

/* The EC keeps the LED lit across a reboot; leave it off for the next boot. */
static void dell_arm64_hotkeys_shutdown(struct platform_device *pdev)
{
	struct dell_arm64_hotkeys *priv = platform_get_drvdata(pdev);

	if (priv && priv->quirks)
		dell_arm64_micmute_set(&priv->micmute, LED_OFF);
}

static int dell_arm64_hotkeys_probe(struct platform_device *pdev)
{
	const struct dmi_system_id *dmi_id;
	struct dell_arm64_hotkeys *priv;
	acpi_handle handle = ACPI_HANDLE(&pdev->dev);
	int ret;

	if (!handle || !dell_arm64_has_event(handle) ||
	    !acpi_has_method(handle, "_WED"))
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;
	priv->handle = handle;
	platform_set_drvdata(pdev, priv);

	ret = devm_mutex_init(&pdev->dev, &priv->led_lock);
	if (ret)
		return ret;

	priv->input = devm_input_allocate_device(&pdev->dev);
	if (!priv->input)
		return -ENOMEM;

	priv->input->name = "Dell WMI hotkeys";
	priv->input->phys = "wmi/input0";
	priv->input->id.bustype = BUS_HOST;

	ret = sparse_keymap_setup(priv->input, dell_arm64_keymap, NULL);
	if (ret)
		return ret;

	ret = input_register_device(priv->input);
	if (ret)
		return ret;

	ret = devm_acpi_install_notify_handler(&pdev->dev, ACPI_DEVICE_NOTIFY,
					       dell_arm64_notify, priv);
	if (ret)
		return ret;

	/* Enable the event, as the ACPI-WMI core does with WExx. */
	acpi_execute_simple_method(handle, "WED0", 1);

	dmi_id = dmi_first_match(dell_arm64_quirk_ids);
	if (dmi_id && acpi_has_method(NULL, "\\_SB.ECRB") &&
	    acpi_has_method(NULL, "\\_SB.MTLD")) {
		priv->quirks = dmi_id->driver_data;
		INIT_DELAYED_WORK(&priv->poll, dell_arm64_poll);
		queue_delayed_work(system_freezable_wq, &priv->poll, 0);
	}

	ret = devm_add_action_or_reset(&pdev->dev, dell_arm64_stop, priv);
	if (ret || !priv->quirks)
		return ret;

	/* Shutdown turns the LED off, so a boot starts with it off. */
	priv->micmute.name = "platform::micmute";
	priv->micmute.max_brightness = 1;
	priv->micmute.brightness_set_blocking = dell_arm64_micmute_set;

	return devm_led_classdev_register(&pdev->dev, &priv->micmute);
}

static const struct acpi_device_id dell_arm64_hotkeys_ids[] = {
	{ "PNP0C14" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, dell_arm64_hotkeys_ids);

static struct platform_driver dell_arm64_hotkeys_driver = {
	.driver = {
		.name = "dell-arm64-hotkeys",
		.acpi_match_table = dell_arm64_hotkeys_ids,
	},
	.probe = dell_arm64_hotkeys_probe,
	.shutdown = dell_arm64_hotkeys_shutdown,
};
module_platform_driver(dell_arm64_hotkeys_driver);

MODULE_DESCRIPTION("Dell hotkey events on Arm64 laptops");
MODULE_LICENSE("GPL");
