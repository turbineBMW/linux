// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-680.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v680
 *
 * Copyright (C) 2025 Linaro Ltd.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include <linux/kstrtox.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-vfe.h"

/*
 * Bring-up knobs for the pixel path, both runtime writable through
 * /sys/module/qcom_camss/parameters/:
 *
 * pix_pokes: "off=val,off=val,..." register writes applied to the VFE
 *            right before the pixel path write masters are enabled.
 * pix_drain: bitmask of bus write masters to enable in virtual frame mode
 *            alongside the FULL pair, so unused output branches of the
 *            image pipeline are consumed instead of stalling it.
 */
static char pix_pokes[8192];
module_param_string(pix_pokes, pix_pokes, sizeof(pix_pokes), 0644);
MODULE_PARM_DESC(pix_pokes, "VFE register pokes at pixel path start: off=val,...");
static unsigned int pix_drain;
module_param(pix_drain, uint, 0644);
MODULE_PARM_DESC(pix_drain, "Bus write masters to run as virtual frame drains");
static unsigned int pix_dump_window;
module_param(pix_dump_window, uint, 0644);
MODULE_PARM_DESC(pix_dump_window, "Dump this many bytes of the IFE register window at stop");

#define VFE_PIX_EXT_SIZE	0xf000
static void __iomem *vfe_pix_ext_window(struct vfe_device *vfe, u32 size);

/*
 * vfe_reg_write - Write a VFE register, logging it under dynamic debug
 * @vfe: VFE device
 * @reg: register offset from the VFE base
 * @val: value to write
 *
 * Enable with the dynamic debug control, for example:
 * echo 'func vfe_reg_write +p' > /sys/kernel/debug/dynamic_debug/control
 * Every register write then lands in the kernel log with its offset and
 * value, so a Linux stream start can be compared against another driver's
 * programming sequence for the same hardware.
 */
static inline void vfe_reg_write(struct vfe_device *vfe, u32 reg, u32 val)
{
	dev_dbg(vfe->camss->dev, "vfe%u wr 0x%05x = 0x%08x\n", vfe->id, reg, val);
	writel(val, vfe->base + reg);
}

#define VFE_TOP_IRQn_STATUS(vfe, n)		((vfe_is_lite(vfe) ? 0x1c : 0x44) + (n) * 4)
#define VFE_TOP_IRQn_MASK(vfe, n)		((vfe_is_lite(vfe) ? 0x24 : 0x34) + (n) * 4)
#define VFE_TOP_IRQn_CLEAR(vfe, n)		((vfe_is_lite(vfe) ? 0x2c : 0x3c) + (n) * 4)
#define		VFE_IRQ1_SOF(vfe, n)		((vfe_is_lite(vfe) ? BIT(2) : BIT(8)) << ((n) * 2))
#define		VFE_IRQ1_EOF(vfe, n)		((vfe_is_lite(vfe) ? BIT(3) : BIT(9)) << ((n) * 2))
#define VFE_TOP_IRQ_CMD(vfe)			(vfe_is_lite(vfe) ? 0x38 : 0x30)
#define		VFE_TOP_IRQ_CMD_GLOBAL_CLEAR	BIT(0)
#define VFE_TOP_DIAG_CONFIG			(vfe_is_lite(vfe) ? 0x40 : 0x50)
#define		VFE_TOP_DIAG_CONFIG_EN		BIT(0)
#define VFE_TOP_DIAG_SENSOR_STATUS_0		0x54
#define VFE_TOP_DIAG_SENSOR_STATUS_1		0x58
#define VFE_TOP_DIAG_FRM_CNT_STATUS_0		0x5c
#define VFE_TOP_DIAG_FRM_CNT_STATUS_1		0x60
#define VFE_TOP_IPP_VIOLATION_STATUS		0x64
#define VFE_TOP_DEBUG_CFG			0xfc
#define		VFE_TOP_DEBUG_CFG_EN		0x3

#define VFE_TOP_DEBUG_11(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xcc)
#define VFE_TOP_DEBUG_12(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xd0)
#define VFE_TOP_DEBUG_13(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xd4)

#define VFE_BUS_IRQn_MASK(vfe, n)		((vfe_is_lite(vfe) ? 0x218 : 0xc18) + (n) * 4)
#define VFE_BUS_IRQn_CLEAR(vfe, n)		((vfe_is_lite(vfe) ? 0x220 : 0xc20) + (n) * 4)
#define VFE_BUS_IRQn_STATUS(vfe, n)		((vfe_is_lite(vfe) ? 0x228 : 0xc28) + (n) * 4)
#define VFE_BUS_IRQ_GLOBAL_CLEAR(vfe)		(vfe_is_lite(vfe) ? 0x230 : 0xc30)
#define VFE_BUS_WR_VIOLATION_STATUS(vfe)	(vfe_is_lite(vfe) ? 0x264 : 0xc64)
#define VFE_BUS_WR_OVERFLOW_STATUS(vfe)		(vfe_is_lite(vfe) ? 0x268 : 0xc68)
#define VFE_BUS_WR_IMAGE_VIOLATION_STATUS(vfe)	(vfe_is_lite(vfe) ? 0x270 : 0xc70)

#define VFE_BUS_WRITE_CLIENT_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x400 : 0xe00) + (c) * 0x100)
#define		VFE_BUS_WRITE_CLIENT_CFG_EN	BIT(0)
#define		VFE_BUS_WRITE_CLIENT_CFG_VIR_FRM BIT(1)
#define VFE_BUS_IMAGE_ADDR(vfe, c)		((vfe_is_lite(vfe) ? 0x404 : 0xe04) + (c) * 0x100)
#define VFE_BUS_FRAME_INCR(vfe, c)		((vfe_is_lite(vfe) ? 0x408 : 0xe08) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG0(vfe, c)		((vfe_is_lite(vfe) ? 0x40c : 0xe0c) + (c) * 0x100)
#define		VFE_BUS_IMAGE_CFG0_DATA(h, s)	(((h) << 16) | ((s) >> 4))
#define WM_IMAGE_CFG_0_DEFAULT_WIDTH		(0xFFFF)

#define VFE_BUS_IMAGE_CFG1(vfe, c)		((vfe_is_lite(vfe) ? 0x410 : 0xe10) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG2(vfe, c)		((vfe_is_lite(vfe) ? 0x414 : 0xe14) + (c) * 0x100)
#define VFE_BUS_PACKER_CFG(vfe, c)		((vfe_is_lite(vfe) ? 0x418 : 0xe18) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x430 : 0xe30) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x434 : 0xe34) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x438 : 0xe38) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x43c : 0xe3c) + (c) * 0x100)
#define VFE_BUS_MMU_PREFETCH_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x460 : 0xe60) + (c) * 0x100)
#define		VFE_BUS_MMU_PREFETCH_CFG_EN	BIT(0)
#define VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, c)	((vfe_is_lite(vfe) ? 0x464 : 0xe64) + (c) * 0x100)
#define VFE_BUS_ADDR_STATUS0(vfe, c)		((vfe_is_lite(vfe) ? 0x470 : 0xe70) + (c) * 0x100)
#define VFE_BUS_DEBUG_STATUS_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x480 : 0xe80) + (c) * 0x100)
#define VFE_BUS_DEBUG_STATUS_0(vfe, c)		((vfe_is_lite(vfe) ? 0x484 : 0xe84) + (c) * 0x100)
#define VFE_BUS_DEBUG_STATUS_TOP(vfe)		(vfe_is_lite(vfe) ? 0x2d8 : 0xcd8)
#define VFE_BUS_BW_LIMITER(vfe, c)		((vfe_is_lite(vfe) ? 0x41c : 0xe1c) + (c) * 0x100)
#define		VFE_BUS_BW_LIMITER_DEFAULT	0xf
#define		VFE_BUS_PACKER_CFG_PLAIN8_LSB_MSB_10	0x3
#define VFE_BUS_IMAGE_CFG0_LINE(h, w)		(((h) << 16) | (w))

/* Pixel path FULL output: luma and chroma write masters of the full IFE */
#define VFE_WM_FULL_Y				0
#define VFE_WM_FULL_C				1

/*
 * TODO: differentiate the port id based on requested type of RDI, BHIST etc
 *
 * IFE write master IDs
 *
 * VIDEO_FULL_Y		0
 * VIDEO_FULL_C		1
 * VIDEO_DS_4:1		2
 * VIDEO_DS_16:1	3
 * DISPLAY_FULL_Y	4
 * DISPLAY_FULL_C	5
 * DISPLAY_DS_4:1	6
 * DISPLAY_DS_16:1	7
 * FD_Y			8
 * FD_C			9
 * PIXEL_RAW		10
 * STATS_BE0		11
 * STATS_BHIST0		12
 * STATS_TINTLESS_BG	13
 * STATS_AWB_BG		14
 * STATS_AWB_BFW	15
 * STATS_BAF		16
 * STATS_BHIST		17
 * STATS_RS		18
 * STATS_IHIST		19
 * SPARSE_PD		20
 * PDAF_V2.0_PD_DATA	21
 * PDAF_V2.0_SAD	22
 * LCR			23
 * RDI0			24
 * RDI1			25
 * RDI2			26
 * LTM_STATS		27
 *
 * IFE Lite write master IDs
 *
 * RDI0			0
 * RDI1			1
 * RDI2			2
 * RDI3			3
 * GAMMA		4
 * BE			5
 */

/* TODO: assign an ENUM in resources and use the provided master
 *       id directly for RDI, STATS, AWB_BG, BHIST.
 *       This macro only works because RDI is all we support right now.
 */
#define RDI_WM(n)			((vfe_is_lite(vfe) ? 0 : 24) + (n))

static void vfe_global_reset(struct vfe_device *vfe)
{
	/* VFE680 has no global reset, simply report a completion */
	complete(&vfe->reset_complete);
}

/*
 * vfe_isr - VFE module interrupt handler
 * @irq: Interrupt line
 * @dev: VFE device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t vfe_isr(int irq, void *dev)
{
	return IRQ_HANDLED;
}

/*
 * vfe_halt - Trigger halt on VFE module and wait to complete
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

static void vfe_disable_irq(struct vfe_device *vfe)
{
	vfe_reg_write(vfe, VFE_TOP_IRQn_MASK(vfe, 0), 0u);
	vfe_reg_write(vfe, VFE_TOP_IRQn_MASK(vfe, 1), 0u);
	vfe_reg_write(vfe, VFE_BUS_IRQn_MASK(vfe, 0), 0u);
	vfe_reg_write(vfe, VFE_BUS_IRQn_MASK(vfe, 1), 0u);
}

/*
 * vfe_pix_apply_pokes - Apply the experimental register poke list
 * @vfe: VFE device
 *
 * Called at pixel path start and again before every reg-update, since the
 * IFE's module registers may be shadowed and consumed at each apply.
 */
static void vfe_pix_apply_pokes(struct vfe_device *vfe)
{
	char *list, *tok, *orig;

	if (!*pix_pokes)
		return;

	orig = list = kstrdup(pix_pokes, GFP_ATOMIC);
	while (list && (tok = strsep(&list, ",")) != NULL) {
		char *eq = strchr(tok, '=');
		u32 off, val;

		if (!eq)
			continue;
		*eq = '\0';
		if (kstrtou32(tok, 0, &off) || kstrtou32(eq + 1, 0, &val) ||
		    off > VFE_PIX_EXT_SIZE - 4 || (off & 3))
			continue;
		if (off < 0x4000) {
			vfe_reg_write(vfe, off, val);
		} else {
			void __iomem *win = vfe_pix_ext_window(vfe, VFE_PIX_EXT_SIZE);

			if (!win)
				continue;
			dev_dbg(vfe->camss->dev, "vfe%u wr 0x%05x = 0x%08x\n",
				vfe->id, off, val);
			writel(val, win + off);
		}
	}
	kfree(orig);
}

/*
 * vfe_680_pix_frame_start - Called from the CSID at pixel path frame start
 * @camss: CAMSS device
 * @vfe_id: VFE fed by that CSID
 *
 * Re-applies the module programming and issues a reg-update so that the
 * next frame start commits it again; the IFE module registers appear to
 * be shadowed and consumed by each apply.
 */
void vfe_680_pix_frame_start(struct camss *camss, u8 vfe_id)
{
	struct vfe_device *vfe;

	if (vfe_id >= camss->res->vfe_num)
		return;
	vfe = &camss->vfe[vfe_id];
	if (vfe->wm_output_map[VFE_LINE_PIX] != VFE_LINE_PIX || !*pix_pokes)
		return;

	vfe_pix_apply_pokes(vfe);
	camss_reg_update(camss, vfe->id, VFE_LINE_PIX, false);
}

static void vfe_wm_update(struct vfe_device *vfe, u8 rdi, u32 addr,
			  struct vfe_line *line)
{
	u8 wm = RDI_WM(rdi);

	if (line->id == VFE_LINE_PIX) {
		struct v4l2_pix_format_mplane *pix =
			&line->video_out.active_fmt.fmt.pix_mp;
		u32 luma_size = pix->plane_fmt[0].bytesperline * pix->height;

		/* NV12 in one buffer: chroma plane follows the luma plane */
		vfe_reg_write(vfe, VFE_BUS_IMAGE_ADDR(vfe, VFE_WM_FULL_Y), addr);
		vfe_reg_write(vfe, VFE_BUS_IMAGE_ADDR(vfe, VFE_WM_FULL_C),
			      addr + luma_size);
		/* re-arm the module programming for the next apply */
		vfe_pix_apply_pokes(vfe);
		return;
	}

	vfe_reg_write(vfe, VFE_BUS_IMAGE_ADDR(vfe, wm), addr);
}

/*
 * vfe_pix_wm_start - Start the FULL output write masters of the pixel path
 * @vfe: VFE device
 * @line: the PIX line
 *
 * The pixel path writes NV12 through two line based write masters, luma on
 * client 0 and chroma on client 1, the chroma plane being half the height.
 * Buffer done for the pair is reported by the CSID as composite group 0.
 */
static void vfe_pix_wm_start(struct vfe_device *vfe, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u8 wm;

	for (wm = VFE_WM_FULL_Y; wm <= VFE_WM_FULL_C; wm++) {
		u32 height = wm == VFE_WM_FULL_C ? pix->height / 2 : pix->height;

		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG0(vfe, wm),
			      VFE_BUS_IMAGE_CFG0_LINE(height, pix->width));
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG1(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG2(vfe, wm), stride);
		vfe_reg_write(vfe, VFE_BUS_PACKER_CFG(vfe, wm),
			      VFE_BUS_PACKER_CFG_PLAIN8_LSB_MSB_10);
		vfe_reg_write(vfe, VFE_BUS_FRAME_INCR(vfe, wm), stride * height);
		vfe_reg_write(vfe, VFE_BUS_BW_LIMITER(vfe, wm),
			      VFE_BUS_BW_LIMITER_DEFAULT);

		vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_CFG(vfe, wm),
			      VFE_BUS_MMU_PREFETCH_CFG_EN);
		vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm), ~0u);

		/* no dropped frames, one irq per frame */
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PERIOD(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm), 0);
	}

	/*
	 * The IFE's image processing modules are left at their reset state
	 * for now. Log what the top block reports so the pixel path bring-up
	 * can be compared against a known-good programming sequence.
	 */
	dev_dbg(vfe->camss->dev,
		"PIX top: version 0x%08x cap 0x%08x lens 0x%08x stats 0x%08x color 0x%08x zoom 0x%08x core_cfg 0x%08x 0x%08x 0x%08x 0x%08x\n",
		readl(vfe->base + 0x0), readl(vfe->base + 0x4),
		readl(vfe->base + 0x8), readl(vfe->base + 0xc),
		readl(vfe->base + 0x10), readl(vfe->base + 0x14),
		readl(vfe->base + 0x24), readl(vfe->base + 0x28),
		readl(vfe->base + 0x2c), readl(vfe->base + 0x68));

	/* Buffer done comes through the CSID; VFE IRQs stay off */
	vfe_disable_irq(vfe);

	/*
	 * Drains: unused output branches get a write master in virtual frame
	 * mode, sized like the branch's output, so they consume the frame.
	 */
	for (wm = 2; wm < 24 && pix_drain; wm++) {
		u32 w = pix->width, h = pix->height;

		if (!(pix_drain & BIT(wm)))
			continue;
		if (wm == 2 || wm == 6) {
			w /= 4;
			h /= 4;
		} else if (wm == 3 || wm == 7) {
			w /= 16;
			h /= 16;
		} else if (wm == 5 || wm == 9) {
			h /= 2;
		}
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG0(vfe, wm),
			      VFE_BUS_IMAGE_CFG0_LINE(h, w));
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG1(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG2(vfe, wm), stride);
		vfe_reg_write(vfe, VFE_BUS_PACKER_CFG(vfe, wm),
			      VFE_BUS_PACKER_CFG_PLAIN8_LSB_MSB_10);
		vfe_reg_write(vfe, VFE_BUS_FRAME_INCR(vfe, wm), stride * h);
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PERIOD(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm),
			      VFE_BUS_WRITE_CLIENT_CFG_EN |
			      VFE_BUS_WRITE_CLIENT_CFG_VIR_FRM);
	}

	vfe_pix_apply_pokes(vfe);

	/* Frame counters, top debug and write master debug status, for the dump */
	vfe_reg_write(vfe, VFE_TOP_DIAG_CONFIG,
		      readl(vfe->base + VFE_TOP_DIAG_CONFIG) | VFE_TOP_DIAG_CONFIG_EN);
	vfe_reg_write(vfe, VFE_TOP_DEBUG_CFG, VFE_TOP_DEBUG_CFG_EN);
	for (wm = VFE_WM_FULL_Y; wm <= VFE_WM_FULL_C; wm++)
		vfe_reg_write(vfe, VFE_BUS_DEBUG_STATUS_CFG(vfe, wm), 1);

	for (wm = VFE_WM_FULL_Y; wm <= VFE_WM_FULL_C; wm++)
		vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm),
			      VFE_BUS_WRITE_CLIENT_CFG_EN);

	dev_dbg(vfe->camss->dev, "PIX WM:%d,%d width %d height %d stride %d\n",
		VFE_WM_FULL_Y, VFE_WM_FULL_C, pix->width, pix->height, stride);
}

static void vfe_wm_start(struct vfe_device *vfe, u8 rdi, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u32 cfg;
	u8 wm;

	if (line->id == VFE_LINE_PIX) {
		vfe_pix_wm_start(vfe, line);
		return;
	}

	cfg = VFE_BUS_IMAGE_CFG0_DATA(pix->height, stride);
	wm = RDI_WM(rdi);

	vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG0(vfe, wm), cfg);
	vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG1(vfe, wm), 0);
	vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG2(vfe, wm), stride);
	vfe_reg_write(vfe, VFE_BUS_PACKER_CFG(vfe, wm), 0);

	/* Set total frame increment value */
	vfe_reg_write(vfe, VFE_BUS_FRAME_INCR(vfe, wm),
		      pix->plane_fmt[0].bytesperline * pix->height);

	/* MMU */
	vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_CFG(vfe, wm),
		      VFE_BUS_MMU_PREFETCH_CFG_EN);
	vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm), ~0u);

	/* no dropped frames, one irq per frame */
	vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PATTERN(vfe, wm), 1);
	vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PERIOD(vfe, wm), 0);
	vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm), 1);
	vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm), 0);

	/* We don't process IRQs for VFE in RDI mode at the moment */
	vfe_disable_irq(vfe);

	/* Enable WM */
	vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm),
		      VFE_BUS_WRITE_CLIENT_CFG_EN);

	dev_dbg(vfe->camss->dev, "RDI%d WM:%d width %d height %d stride %d\n",
		rdi, wm, pix->width, pix->height, stride);
}

/*
 * vfe_pix_debug_dump - Log where the pixel path stands inside the IFE
 * @vfe: VFE device
 *
 * Called at stop while the block is powered: top status and diagnostic
 * frame counters say whether the IFE consumed frames, the bus status words
 * say whether the write masters wrote and whether anything overflowed.
 */
static void vfe_pix_debug_dump(struct vfe_device *vfe)
{
	struct device *dev = vfe->camss->dev;
	struct camss *camss = vfe->camss;
	unsigned int i;

	for (i = 0; i < camss->res->csid_num; i++) {
		struct csid_device *csid = &camss->csid[i];

		if (!csid_is_lite(csid) && csid->res->hw_ops->debug_dump &&
		    csid->subdev.entity.use_count)
			csid->res->hw_ops->debug_dump(csid);
	}

	dev_info(dev,
		 "vfe%u PIX top: irq0 0x%08x irq1 0x%08x diag 0x%08x sensor 0x%08x 0x%08x frames 0x%08x 0x%08x ipp_violation 0x%08x\n",
		 vfe->id,
		 readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 0)),
		 readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 1)),
		 readl(vfe->base + VFE_TOP_DIAG_CONFIG),
		 readl(vfe->base + VFE_TOP_DIAG_SENSOR_STATUS_0),
		 readl(vfe->base + VFE_TOP_DIAG_SENSOR_STATUS_1),
		 readl(vfe->base + VFE_TOP_DIAG_FRM_CNT_STATUS_0),
		 readl(vfe->base + VFE_TOP_DIAG_FRM_CNT_STATUS_1),
		 readl(vfe->base + VFE_TOP_IPP_VIOLATION_STATUS));
	/* 17 top debug words, four bits of state per pipeline module */
	dev_info(dev,
		 "vfe%u PIX dbg: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 vfe->id,
		 readl(vfe->base + 0xa0), readl(vfe->base + 0xa4),
		 readl(vfe->base + 0xa8), readl(vfe->base + 0xac),
		 readl(vfe->base + 0xb0), readl(vfe->base + 0xb4),
		 readl(vfe->base + 0xb8), readl(vfe->base + 0xbc),
		 readl(vfe->base + 0xc0), readl(vfe->base + 0xc4),
		 readl(vfe->base + 0xc8), readl(vfe->base + 0xcc),
		 readl(vfe->base + 0xd0), readl(vfe->base + 0xd4),
		 readl(vfe->base + 0xd8), readl(vfe->base + 0xdc),
		 readl(vfe->base + 0xe0));
	dev_info(dev,
		 "vfe%u PIX bus: irq0 0x%08x irq1 0x%08x ccif 0x%08x overflow 0x%08x imgsize 0x%08x dbgtop 0x%08x wm0 addr 0x%08x dbg 0x%08x wm1 addr 0x%08x dbg 0x%08x\n",
		 vfe->id,
		 readl(vfe->base + VFE_BUS_IRQn_STATUS(vfe, 0)),
		 readl(vfe->base + VFE_BUS_IRQn_STATUS(vfe, 1)),
		 readl(vfe->base + VFE_BUS_WR_VIOLATION_STATUS(vfe)),
		 readl(vfe->base + VFE_BUS_WR_OVERFLOW_STATUS(vfe)),
		 readl(vfe->base + VFE_BUS_WR_IMAGE_VIOLATION_STATUS(vfe)),
		 readl(vfe->base + VFE_BUS_DEBUG_STATUS_TOP(vfe)),
		 readl(vfe->base + VFE_BUS_ADDR_STATUS0(vfe, VFE_WM_FULL_Y)),
		 readl(vfe->base + VFE_BUS_DEBUG_STATUS_0(vfe, VFE_WM_FULL_Y)),
		 readl(vfe->base + VFE_BUS_ADDR_STATUS0(vfe, VFE_WM_FULL_C)),
		 readl(vfe->base + VFE_BUS_DEBUG_STATUS_0(vfe, VFE_WM_FULL_C)));
}

/*
 * vfe_pix_ext_window - Map the whole IFE block from its physical base
 * @vfe: VFE device
 * @size: bytes to map
 *
 * Bring-up aid: the DT maps 16 KiB per VFE, the image processing modules
 * sit above that. The mapping is kept for the life of the module.
 */
static void __iomem *vfe_pix_ext_window(struct vfe_device *vfe, u32 size)
{
	static void __iomem *ext[2];
	struct platform_device *pdev = to_platform_device(vfe->camss->dev);
	struct resource *r;

	if (vfe->id > 1)
		return NULL;
	if (ext[vfe->id])
		return ext[vfe->id];

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 vfe->camss->res->vfe_res[vfe->id].reg[0]);
	if (!r)
		return NULL;
	ext[vfe->id] = ioremap(r->start, size);
	return ext[vfe->id];
}

/*
 * vfe_pix_dump_window - Dump the IFE register window beyond what is mapped
 * @vfe: VFE device
 *
 * Maps the full IFE block from its physical base and logs every 32-byte
 * row that holds a nonzero word. Bring-up aid to locate the image
 * processing module registers.
 */
static void vfe_pix_dump_window(struct vfe_device *vfe)
{
	struct device *dev = vfe->camss->dev;
	void __iomem *win;
	u32 off, size = min_t(u32, pix_dump_window & ~0x1f, VFE_PIX_EXT_SIZE);

	if (!size)
		return;

	win = vfe_pix_ext_window(vfe, VFE_PIX_EXT_SIZE);
	if (!win) {
		dev_err(dev, "vfe%u: cannot map the IFE window\n", vfe->id);
		return;
	}

	dev_info(dev, "vfe%u window dump 0x%x bytes\n", vfe->id, size);
	for (off = 0; off < size; off += 0x20) {
		u32 w[8];
		int i, nz = 0;

		for (i = 0; i < 8; i++) {
			w[i] = readl(win + off + i * 4);
			nz |= w[i];
		}
		if (nz)
			dev_info(dev, "vfe%u %05x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				 vfe->id, off, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
	}
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 rdi)
{
	u8 wm = RDI_WM(rdi);

	if (rdi == VFE_LINE_PIX && !vfe_is_lite(vfe)) {
		vfe_pix_debug_dump(vfe);
		vfe_pix_dump_window(vfe);
		vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, VFE_WM_FULL_Y), 0);
		vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, VFE_WM_FULL_C), 0);
		for (wm = 2; wm < 24; wm++)
			if (pix_drain & BIT(wm))
				vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm), 0);
		return;
	}

	vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm), 0);
}

static const struct camss_video_ops vfe_video_ops_680 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_680;
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, false);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, true);
}

const struct vfe_hw_ops vfe_ops_680 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
};
