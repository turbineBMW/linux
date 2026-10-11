// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thunderbolt driver - USB4 host interfaces described by ACPI
 *
 * Some Arm SoCs expose their USB4 host interfaces as memory-mapped platform
 * devices (ACPI0015) with a wired interrupt, rather than as PCIe NHIs. The
 * registers follow the USB4 specification, so the NHI core drives them; this
 * glue maps them, powers the host router and services the interrupt.
 */

#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk-pwrap.h>

#include "nhi.h"
#include "nhi_regs.h"
#include "tb.h"

/*
 * Three status bits per path start at REG_RING_NOTIFY_BASE, and the 64 bits
 * there end where REG_RING_INT_CLEAR begins.
 */
#define NHI_PLAT_MAX_PATHS	21

struct nhi_plat;

/**
 * struct nhi_plat_data - Platform specific host router handling
 * @power_on: Power the host router before its registers are accessed
 */
struct nhi_plat_data {
	int (*power_on)(struct nhi_plat *np);
};

/**
 * struct nhi_plat - USB4 host interface described by ACPI
 * @nhi: NHI device
 * @data: Platform specific handling
 * @irq: The interrupt every ring is signalled through
 * @irq_requested: @irq is currently requested
 * @pwrap: SSPM power control handle (NVIDIA N1x)
 */
struct nhi_plat {
	struct tb_nhi nhi;
	const struct nhi_plat_data *data;
	int irq;
	bool irq_requested;
	void *pwrap;
};

static inline struct nhi_plat *nhi_to_plat(struct tb_nhi *nhi)
{
	return container_of(nhi, struct nhi_plat, nhi);
}

static irqreturn_t nhi_plat_irq_thread(int irq, void *data)
{
	struct tb_nhi *nhi = data;

	return nhi_handle_interrupts(nhi) ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * The interrupt is level-triggered and stays asserted until the notification
 * registers are read, so keep it masked until the thread has read them.
 */
static int nhi_plat_init_interrupts(struct tb_nhi *nhi)
{
	struct nhi_plat *np = nhi_to_plat(nhi);
	int ret;

	ret = devm_request_threaded_irq(nhi->dev, np->irq, NULL,
					nhi_plat_irq_thread,
					IRQF_ONESHOT | IRQF_NO_SUSPEND,
					dev_name(nhi->dev), nhi);
	if (!ret)
		np->irq_requested = true;

	return ret;
}

/* Called when the host interface is shut down, so nothing reads it after */
static void nhi_plat_shutdown(struct tb_nhi *nhi)
{
	struct nhi_plat *np = nhi_to_plat(nhi);

	if (np->irq_requested) {
		devm_free_irq(nhi->dev, np->irq, nhi);
		np->irq_requested = false;
	}
}

static const struct tb_nhi_ops nhi_plat_ops = {
	.init_interrupts = nhi_plat_init_interrupts,
	.shutdown = nhi_plat_shutdown,
};

/*
 * NVIDIA N1x: the host routers are powered through the SSPM. The firmware
 * leaves them powered at boot, but power_wrap can release them once xHCI is
 * up, after which the SSPM stops answering for them altogether. A request
 * then hangs it and resets the SoC, so refuse to touch them in that case.
 *
 * They are never powered down again: the SSPM can turn a host router off,
 * but turning it back on does not restore what the boot firmware set up, and
 * the router no longer answers until the next boot.
 */
static int nhi_nvda8100_power_on(struct nhi_plat *np)
{
	struct device *dev = np->nhi.dev;
	struct acpi_buffer path = { ACPI_ALLOCATE_BUFFER, NULL };
	int ret;

	if (mtk_pwrap_usb4_released())
		return dev_err_probe(dev, -ENODEV,
				     "USB4 resources are released, boot with power_wrap_drv.usb4_release=0\n");

	if (!np->pwrap) {
		if (ACPI_FAILURE(acpi_get_name(ACPI_HANDLE(dev), ACPI_FULL_PATHNAME,
					       &path)))
			return -ENODEV;
		np->pwrap = mtk_pwrap_dev_probe(path.pointer);
		kfree(path.pointer);
		if (!np->pwrap)
			return dev_err_probe(dev, -ENODEV,
					     "no SSPM power control for the host router\n");
	}

	ret = mtk_pwrap_dev_resume(np->pwrap, 0);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the host router\n");

	return 0;
}

static const struct nhi_plat_data nhi_nvda8100_data = {
	.power_on = nhi_nvda8100_power_on,
};

static int nhi_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct nhi_plat *np;
	struct tb_nhi *nhi;
	u32 caps, paths;
	int ret;

	np = devm_kzalloc(dev, sizeof(*np), GFP_KERNEL);
	if (!np)
		return -ENOMEM;

	np->data = device_get_match_data(dev);
	nhi = &np->nhi;
	nhi->dev = dev;
	nhi->ops = &nhi_plat_ops;

	nhi->iobase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(nhi->iobase))
		return dev_err_probe(dev, PTR_ERR(nhi->iobase),
				     "cannot map the host interface registers\n");
	if (resource_size(res) < REG_RESET + 4)
		return dev_err_probe(dev, -ENODEV,
				     "host interface register window too small\n");

	np->irq = platform_get_irq(pdev, 0);
	if (np->irq < 0)
		return np->irq;

	if (np->data->power_on) {
		ret = np->data->power_on(np);
		if (ret)
			return ret;
	}

	/* nhi_probe() sizes everything from the path count; check it first */
	caps = ioread32(nhi->iobase + REG_CAPS);
	paths = caps & 0x3ff;
	if (!paths || paths > NHI_PLAT_MAX_PATHS)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected host interface capabilities %#x\n", caps);

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_put;

	ret = nhi_probe(nhi);
	if (ret)
		goto err_put;

	/*
	 * Nothing would notice a device being plugged in while the host router
	 * is runtime suspended, as there is no wake interrupt support yet.
	 */
	pm_runtime_forbid(dev);

	return 0;

err_put:
	pm_runtime_put_noidle(dev);
	nhi_plat_shutdown(nhi);
	return ret;
}

static void nhi_plat_remove(struct platform_device *pdev)
{
	struct tb *tb = platform_get_drvdata(pdev);

	nhi_remove(tb->nhi);
}

static const struct acpi_device_id nhi_plat_acpi_ids[] = {
	{ "NVDA8100", (kernel_ulong_t)&nhi_nvda8100_data },
	{ }
};
MODULE_DEVICE_TABLE(acpi, nhi_plat_acpi_ids);

static struct platform_driver nhi_plat_driver = {
	.driver = {
		.name = "thunderbolt-platform",
		.acpi_match_table = nhi_plat_acpi_ids,
		.pm = &nhi_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = nhi_plat_probe,
	.remove = nhi_plat_remove,
	.shutdown = nhi_plat_remove,
};
module_platform_driver(nhi_plat_driver);

MODULE_IMPORT_NS("THUNDERBOLT_NHI");
MODULE_DESCRIPTION("USB4 host interfaces described by ACPI");
MODULE_LICENSE("GPL");
