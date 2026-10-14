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
static bool pix_rearm;
module_param(pix_rearm, bool, 0644);
MODULE_PARM_DESC(pix_rearm, "Re-apply the PIX output stage at every frame start");

MODULE_PARM_DESC(pix_rearm, "Re-apply the output stage programming at every frame start (not needed)");

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
 * IFE image processing output stage, x1e80100 (Titan 680). Module bases
 * were read from the hardware at reset and matched one to one against
 * Qualcomm's GPL module list for this IFE; see ife680-module-map.md.
 * Each CROP_RND_CLAMP block: config at +0x60 (enable bits 0 and 9),
 * line window at +0x68 and pixel window at +0x6c, first << 16 | last.
 */
#define VFE_CLC_CROP_CFG			0x60
#define		VFE_CLC_CROP_CFG_EN		(BIT(0) | BIT(9))
#define VFE_CLC_CROP_LINE			0x68
#define VFE_CLC_CROP_PIX			0x6c
#define		VFE_CLC_CROP_WINDOW(first, last)	(((first) << 16) | (last))

/* luma and chroma crop pairs: FD, DISP, DISP DS4, DISP DS16, VID DSX, VID DS16 */
static const u16 vfe_pix_crops_unused[][2] = {
	{ 0x7c00, 0x7e00 }, { 0x8400, 0x8600 }, { 0x8c00, 0x8e00 },
	{ 0x9400, 0x9600 }, { 0xa400, 0xa600 }, { 0xac00, 0xae00 },
};
#define VFE_PIX_CROP_VID_Y			0x9c00
#define VFE_PIX_CROP_VID_C			0x9e00

/* drain write masters: VID DS4, VID DS16, DISP Y, DISP C, DISP DS4, DISP DS16, FD Y, FD C */
#define VFE_PIX_DRAIN_WM_FIRST			2
#define VFE_PIX_DRAIN_WM_LAST			9
#define VFE_PIX_DRAIN_WIDTH			16

/*
 * Image quality modules on the PIX path. The register layouts below were
 * measured on the X1E80100 IFE with the CSI test pattern generator (colour
 * bars and the incrementing ramp) and match the sibling CLC blocks of the
 * OPE; see ISP-PHASE-B.md for the measurements.
 */
#define VFE_CLC_MODULE_CFG			0x60
#define		VFE_CLC_MODULE_CFG_EN		BIT(0)

/* Demosaic: Bayer to RGB, 14-bit samples (10-bit raw << 4) */
#define VFE_PIX_DEMOSAIC			0x5600
#define		VFE_PIX_DEMOSAIC_CFG_DYN_G_CLAMP_EN	BIT(14)
#define		VFE_PIX_DEMOSAIC_CFG_DYN_RB_CLAMP_EN	BIT(15)
#define VFE_PIX_DEMOSAIC_INTERP_COEFF		0x68
#define		VFE_PIX_DEMOSAIC_LAMBDA_G(v)	((v) & 0xff)
#define		VFE_PIX_DEMOSAIC_LAMBDA_RB(v)	(((v) & 0xff) << 16)
#define VFE_PIX_DEMOSAIC_CLASSIFIER_0		0x6c
#define		VFE_PIX_DEMOSAIC_AK(v)		((v) << 20)
#define VFE_PIX_DEMOSAIC_CLASSIFIER_1		0x70
#define		VFE_PIX_DEMOSAIC_WK(v)		((v) & 0x3ff)

/*
 * Colour transform: three output groups (Y, Cb, Cr), each 0x18 bytes:
 * two 13-bit two's complement Q10 coefficients for the G and B inputs,
 * one for R, the output offset, the input offset, and the clamp minimum
 * and maximum. out = clamp(round((cG * G + cB * B + cR * R) >> 10)) + off.
 * The inputs are 14-bit, the result is written as 12-bit, and the 8-bit
 * planes take round(out / 16), so the maximum must stay below 0xff8.
 */
#define VFE_PIX_CST				0x6100
#define VFE_PIX_CST_GROUP(n)			(0x68 + (n) * 0x18)
#define VFE_PIX_CST_COEFF_GB			0x0
#define		VFE_PIX_CST_COEFF(g, b)		((((b) & 0x1fff) << 16) | ((g) & 0x1fff))
#define VFE_PIX_CST_COEFF_R			0x4
#define VFE_PIX_CST_OFFSET_OUT			0x8
#define		VFE_PIX_CST_OFFSET(v)		((v) << 17)
#define VFE_PIX_CST_OFFSET_IN			0xc
#define VFE_PIX_CST_CLAMP_MIN			0x10
#define VFE_PIX_CST_CLAMP_MAX			0x14
#define		VFE_PIX_CST_CLAMP(v)		((v) << 18)
#define VFE_PIX_CST_Q10(v)			(v)
#define VFE_PIX_CST_OUT_MAX			0xff7
#define VFE_PIX_CST_CHROMA_OFFSET		0x800

/*
 * Top clock gating overrides (the GPL Titan driver's core_cgc_ovd_0/1 and
 * ahb_cgc_ovd). The front-end modules gate their clocks while disabled
 * and drop register writes, so the overrides are set before programming.
 */
#define VFE_TOP_CORE_CGC_OVD_0			0x18
#define VFE_TOP_CORE_CGC_OVD_1			0x1c
#define VFE_TOP_AHB_CGC_OVD			0x20

/*
 * The front-end modules drop register writes while their clocks are
 * gated, so the overrides go on around programming and off again after:
 * the written values stay, and the gating is back for the stream.
 */
static void vfe_680_cgc_override(struct vfe_device *vfe, bool on)
{
	u32 val = on ? ~0u : 0;

	vfe_reg_write(vfe, VFE_TOP_CORE_CGC_OVD_0, val);
	vfe_reg_write(vfe, VFE_TOP_CORE_CGC_OVD_1, val);
	vfe_reg_write(vfe, VFE_TOP_AHB_CGC_OVD, val);
}

/*
 * Lens shading (the Titan LSC 4.1 block): bilinear mesh gains over the
 * per-channel (half-size) image, 14-bit Q10, two channels per DMI word
 * (the 4.0 block of the public Titan 480 code packs them in 13 bits).
 * The mesh is a grid of blocks, each split into 2^NUM_SUBBLOCKS
 * sub-blocks whose width's reciprocal (Q20) steps the interpolation; the
 * grid may hang over the image edges, so the first block, sub-block and
 * pixel offsets say where the image starts in it. CFG 5, 11 and 12 take
 * 18-bit values in their top bits that the vendor stack fills from its
 * unpacked settings (the second is the pixel offset's slot); their units
 * are not known and they stay zero, so the gains multiply the sensor
 * pedestal with the signal and the white balance subtract after the
 * block cannot take it back: corners come out brighter and pinker than
 * the mesh says, the more so the darker the scene. Flat-field data: a
 * value in CFG 11 scales the response down and crushes the darker
 * channels first, so the field is per channel pair or in other units.
 */
#define VFE_PIX_LSC				0x4300
#define VFE_PIX_LSC_DMI_CFG			0x08
#define		VFE_PIX_LSC_DMI_CFG_AUTO_LOAD_EN	BIT(20)
#define VFE_PIX_LSC_DMI_LUT_CFG			0x0c
#define		VFE_PIX_LSC_SEL_NONE		0
#define		VFE_PIX_LSC_SEL_RED		1	/* R | Gr << 14 */
#define		VFE_PIX_LSC_SEL_BLUE		2	/* B | Gb << 14 */
#define		VFE_PIX_LSC_SEL_GRID		3	/* adaptive LSC, unused */
#define VFE_PIX_LSC_DMI_DATA			0x10
#define VFE_PIX_LSC_DMI_LUT_BANK_CFG		0x58
#define VFE_PIX_LSC_MODULE_LUT_BANK_CFG		0x5c
#define VFE_PIX_LSC_MODULE_CFG			0x60
#define		VFE_PIX_LSC_MODULE_CFG_EN	BIT(0)
#define		VFE_PIX_LSC_MODULE_CFG_ALSC_EN	BIT(1)
#define		VFE_PIX_LSC_MODULE_CFG_CROP_EN	BIT(2)
#define		VFE_PIX_LSC_MODULE_CFG_NUM_SUBBLOCKS(v)	((v) << 12)
#define		VFE_PIX_LSC_MODULE_CFG_INIT_BLOCK_X(v)	((v) << 16)
#define		VFE_PIX_LSC_MODULE_CFG_INIT_BLOCK_Y(v)	((v) << 20)
#define		VFE_PIX_LSC_MODULE_CFG_INIT_SUBBLOCK_X(v) ((v) << 24)
#define		VFE_PIX_LSC_MODULE_CFG_INIT_SUBBLOCK_Y(v) ((v) << 28)
#define VFE_PIX_LSC_CFG(n)			(0x68 + 4 * (n))
/* CFG 0: blocks minus one, 1: block size minus one, 2/3: sub-block size minus one and 1/size Q20 */
#define		VFE_PIX_LSC_CFG0(bx, by)	((bx) | ((by) << 4))
#define		VFE_PIX_LSC_CFG1(w, h)		((w) | ((h) << 16))
#define		VFE_PIX_LSC_CFG23(w, inv)	((w) | ((inv) << 12))
/* CFG 4: first pixel's offset in its sub-block, 6: initial vertical phase Q20 */
#define		VFE_PIX_LSC_CFG4(x, y)		((x) | ((y) << 16))
#define		VFE_PIX_LSC_CFG9(first, last)	((first) | ((last) << 16))
#define VFE_PIX_LSC_MESH_COLS_MAX		(QCOM_CAMSS_LSC_MESH_COLS - 1)
#define VFE_PIX_LSC_MESH_ROWS_MAX		(QCOM_CAMSS_LSC_MESH_ROWS - 1)
#define VFE_PIX_LSC_GAIN_MIN			1024
#define VFE_PIX_LSC_GAIN_MAX			8191
#define VFE_PIX_LSC_GAIN_SHIFT			14

/*
 * White balance: Q10 gains for G, B and R in bits 31:17, per-channel adds
 * at +0x80..+0x88. The register at +0x94 resets to 0xa, which leaves the
 * green channel wrong; all ones restores it (meaning not yet known).
 */
#define VFE_PIX_WB				0x4500
#define VFE_PIX_WB_GAIN_G			0x68
#define VFE_PIX_WB_GAIN_B			0x6c
#define VFE_PIX_WB_GAIN_R			0x70
#define		VFE_PIX_WB_GAIN(q10)		((q10) << 17)
#define		VFE_PIX_WB_GAIN_UNITY		0x400
#define VFE_PIX_WB_SUB_G			0x74	/* subtracted before the gain, 12-bit units */
#define VFE_PIX_WB_SUB_B			0x78
#define VFE_PIX_WB_SUB_R			0x7c
#define		VFE_PIX_WB_SUB(v12)		((v12) << 20)
#define VFE_PIX_WB_CFG_94			0x94

/*
 * Gamma: three 256-entry tables of 12-bit output values indexed by the
 * top 8 bits of the input, loaded through the DMI port as in the OPE.
 */
#define VFE_PIX_GLUT				0x5f00
#define VFE_PIX_GLUT_DMI_CFG			0x08
#define		VFE_PIX_GLUT_DMI_CFG_AUTO_LOAD_EN	BIT(20)
#define VFE_PIX_GLUT_DMI_LUT_CFG		0x0c
#define		VFE_PIX_GLUT_SEL_NONE		0
#define		VFE_PIX_GLUT_SEL_G		1
#define		VFE_PIX_GLUT_SEL_B		2
#define		VFE_PIX_GLUT_SEL_R		3
#define VFE_PIX_GLUT_DMI_DATA			0x10
#define VFE_PIX_GLUT_DMI_LUT_BANK_CFG		0x58
#define VFE_PIX_GLUT_MODULE_LUT_BANK_CFG	0x5c
#define VFE_PIX_GLUT_SIZE			256

/* 12-bit output = 4095 * (i / 255) ^ (1 / 2.2) */
static const u16 vfe_pix_gamma_2_2[VFE_PIX_GLUT_SIZE] = {
	0x0000, 0x014a, 0x01c4, 0x0220, 0x026b, 0x02ae, 0x02e9, 0x031f,
	0x0351, 0x0380, 0x03ac, 0x03d5, 0x03fd, 0x0423, 0x0447, 0x046a,
	0x048b, 0x04ac, 0x04cb, 0x04ea, 0x0508, 0x0524, 0x0541, 0x055c,
	0x0577, 0x0591, 0x05ab, 0x05c4, 0x05dc, 0x05f4, 0x060c, 0x0623,
	0x063a, 0x0651, 0x0667, 0x067c, 0x0692, 0x06a7, 0x06bc, 0x06d0,
	0x06e4, 0x06f8, 0x070c, 0x071f, 0x0732, 0x0745, 0x0758, 0x076b,
	0x077d, 0x078f, 0x07a1, 0x07b2, 0x07c4, 0x07d5, 0x07e6, 0x07f7,
	0x0808, 0x0819, 0x0829, 0x0839, 0x0849, 0x0859, 0x0869, 0x0879,
	0x0889, 0x0898, 0x08a7, 0x08b7, 0x08c6, 0x08d5, 0x08e3, 0x08f2,
	0x0901, 0x090f, 0x091e, 0x092c, 0x093a, 0x0948, 0x0956, 0x0964,
	0x0972, 0x097f, 0x098d, 0x099b, 0x09a8, 0x09b5, 0x09c3, 0x09d0,
	0x09dd, 0x09ea, 0x09f7, 0x0a04, 0x0a10, 0x0a1d, 0x0a2a, 0x0a36,
	0x0a43, 0x0a4f, 0x0a5b, 0x0a68, 0x0a74, 0x0a80, 0x0a8c, 0x0a98,
	0x0aa4, 0x0ab0, 0x0abc, 0x0ac7, 0x0ad3, 0x0adf, 0x0aea, 0x0af6,
	0x0b01, 0x0b0d, 0x0b18, 0x0b23, 0x0b2f, 0x0b3a, 0x0b45, 0x0b50,
	0x0b5b, 0x0b66, 0x0b71, 0x0b7c, 0x0b87, 0x0b92, 0x0b9c, 0x0ba7,
	0x0bb2, 0x0bbc, 0x0bc7, 0x0bd1, 0x0bdc, 0x0be6, 0x0bf1, 0x0bfb,
	0x0c05, 0x0c10, 0x0c1a, 0x0c24, 0x0c2e, 0x0c38, 0x0c42, 0x0c4c,
	0x0c56, 0x0c60, 0x0c6a, 0x0c74, 0x0c7e, 0x0c88, 0x0c91, 0x0c9b,
	0x0ca5, 0x0cae, 0x0cb8, 0x0cc2, 0x0ccb, 0x0cd5, 0x0cde, 0x0ce8,
	0x0cf1, 0x0cfb, 0x0d04, 0x0d0d, 0x0d17, 0x0d20, 0x0d29, 0x0d32,
	0x0d3b, 0x0d45, 0x0d4e, 0x0d57, 0x0d60, 0x0d69, 0x0d72, 0x0d7b,
	0x0d84, 0x0d8d, 0x0d96, 0x0d9f, 0x0da7, 0x0db0, 0x0db9, 0x0dc2,
	0x0dca, 0x0dd3, 0x0ddc, 0x0de5, 0x0ded, 0x0df6, 0x0dfe, 0x0e07,
	0x0e0f, 0x0e18, 0x0e20, 0x0e29, 0x0e31, 0x0e3a, 0x0e42, 0x0e4b,
	0x0e53, 0x0e5b, 0x0e63, 0x0e6c, 0x0e74, 0x0e7c, 0x0e84, 0x0e8d,
	0x0e95, 0x0e9d, 0x0ea5, 0x0ead, 0x0eb5, 0x0ebd, 0x0ec5, 0x0ecd,
	0x0ed5, 0x0edd, 0x0ee5, 0x0eed, 0x0ef5, 0x0efd, 0x0f05, 0x0f0d,
	0x0f15, 0x0f1d, 0x0f24, 0x0f2c, 0x0f34, 0x0f3c, 0x0f43, 0x0f4b,
	0x0f53, 0x0f5a, 0x0f62, 0x0f6a, 0x0f71, 0x0f79, 0x0f81, 0x0f88,
	0x0f90, 0x0f97, 0x0f9f, 0x0fa6, 0x0fae, 0x0fb5, 0x0fbd, 0x0fc4,
	0x0fcc, 0x0fd3, 0x0fda, 0x0fe2, 0x0fe9, 0x0ff0, 0x0ff8, 0x0fff,
};

/*
 * Colour correction: out_G = a0 G + b0 B + c0 R + k0 and so on for B and R,
 * 12-bit two's complement coefficients in the OPE's layout, Q selected by
 * +0x8c (3 = Q10). Programmed as identity until a matrix is measured.
 */
#define VFE_PIX_CC				0x5800
#define VFE_PIX_CC_COEFF_A0			0x68	/* a0 bits 11:0, a1 bits 27:16 */
#define VFE_PIX_CC_COEFF_A2			0x6c
#define VFE_PIX_CC_COEFF_B0			0x70
#define VFE_PIX_CC_COEFF_B2			0x74
#define VFE_PIX_CC_COEFF_C0			0x78
#define VFE_PIX_CC_COEFF_C2			0x7c
#define VFE_PIX_CC_K(n)				(0x80 + 4 * (n))
#define		VFE_PIX_CC_K_VAL(v)		(((v) & 0x7ff) << 21)
#define VFE_PIX_CC_Q				0x8c
#define		VFE_PIX_CC_Q10			0x3
#define		VFE_PIX_CC_PAIR(lo, hi)		((((hi) & 0xfff) << 16) | ((lo) & 0xfff))
#define		VFE_PIX_CC_UNITY		0x400

/* VID output luma and chroma M/N downscalers, same layout as the OPE's */
#define VFE_PIX_DS_MN_Y_VID			0x9800
#define VFE_PIX_DS_MN_C_VID			0x9a00
#define VFE_PIX_DS_MN_CFG			0x64
#define		VFE_PIX_DS_MN_CFG_H_SCALE_EN	BIT(9)
#define		VFE_PIX_DS_MN_CFG_V_SCALE_EN	BIT(10)
#define VFE_PIX_DS_MN_IMAGE_SIZE		0x68
#define VFE_PIX_DS_MN_H_CFG			0x6c
#define VFE_PIX_DS_MN_H_PHASE			0x70
#define VFE_PIX_DS_MN_V_CFG			0x74
#define VFE_PIX_DS_MN_V_PHASE			0x78
#define		VFE_PIX_DS_MN_STEP(in, out)	(((u64)(in) << 21) / (out))
#define		VFE_PIX_DS_MN_RESOLUTION_FULL	(0x3u << 30)

static void vfe_pix_ext_write(struct vfe_device *vfe, void __iomem *win, u32 off, u32 val)
{
	dev_dbg(vfe->camss->dev, "vfe%u wr 0x%05x = 0x%08x\n", vfe->id, off, val);
	writel(val, win + off);
}

/*
 * vfe_pix_program_output_stage - Open the VID path, terminate the others
 * @vfe: VFE device
 * @line: the PIX line
 *
 * The IFE fans every frame out to the FD, DISP and VID output branches,
 * and a branch with a closed crop or without a consumer blocks the whole
 * pipeline. The VID luma and chroma crops get the full frame; every other
 * crop a one-line window; and each unused branch ends in a write master in
 * virtual frame mode that consumes its frames without writing memory.
 */
static int vfe_pix_program_output_stage(struct vfe_device *vfe, struct vfe_line *line)
{
	void __iomem *win = vfe_pix_ext_window(vfe, VFE_PIX_EXT_SIZE);
	unsigned int i;
	u8 wm;

	if (!win)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(vfe_pix_crops_unused); i++) {
		u32 y = vfe_pix_crops_unused[i][0], c = vfe_pix_crops_unused[i][1];

		vfe_pix_ext_write(vfe, win, y + VFE_CLC_CROP_CFG, VFE_CLC_CROP_CFG_EN);
		vfe_pix_ext_write(vfe, win, y + VFE_CLC_CROP_LINE, VFE_CLC_CROP_WINDOW(0, 0));
		vfe_pix_ext_write(vfe, win, y + VFE_CLC_CROP_PIX,
				  VFE_CLC_CROP_WINDOW(0, VFE_PIX_DRAIN_WIDTH - 1));
		vfe_pix_ext_write(vfe, win, c + VFE_CLC_CROP_CFG, VFE_CLC_CROP_CFG_EN);
		vfe_pix_ext_write(vfe, win, c + VFE_CLC_CROP_LINE, VFE_CLC_CROP_WINDOW(0, 0));
		vfe_pix_ext_write(vfe, win, c + VFE_CLC_CROP_PIX,
				  VFE_CLC_CROP_WINDOW(0, VFE_PIX_DRAIN_WIDTH / 2 - 1));
	}

	{
		const struct v4l2_rect *c = &line->crop;

		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_Y + VFE_CLC_CROP_CFG, VFE_CLC_CROP_CFG_EN);
		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_Y + VFE_CLC_CROP_LINE,
				  VFE_CLC_CROP_WINDOW(c->top, c->top + c->height - 1));
		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_Y + VFE_CLC_CROP_PIX,
				  VFE_CLC_CROP_WINDOW(c->left, c->left + c->width - 1));
		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_C + VFE_CLC_CROP_CFG, VFE_CLC_CROP_CFG_EN);
		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_C + VFE_CLC_CROP_LINE,
				  VFE_CLC_CROP_WINDOW(c->top / 2, c->top / 2 + c->height / 2 - 1));
		vfe_pix_ext_write(vfe, win, VFE_PIX_CROP_VID_C + VFE_CLC_CROP_PIX,
				  VFE_CLC_CROP_WINDOW(c->left / 2, c->left / 2 + c->width / 2 - 1));
	}

	for (wm = VFE_PIX_DRAIN_WM_FIRST; wm <= VFE_PIX_DRAIN_WM_LAST; wm++) {
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG0(vfe, wm),
			      VFE_BUS_IMAGE_CFG0_LINE(1, VFE_PIX_DRAIN_WIDTH));
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG1(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IMAGE_CFG2(vfe, wm), VFE_PIX_DRAIN_WIDTH);
		vfe_reg_write(vfe, VFE_BUS_PACKER_CFG(vfe, wm),
			      VFE_BUS_PACKER_CFG_PLAIN8_LSB_MSB_10);
		vfe_reg_write(vfe, VFE_BUS_FRAME_INCR(vfe, wm), VFE_PIX_DRAIN_WIDTH);
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_FRAMEDROP_PERIOD(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm), 1);
		vfe_reg_write(vfe, VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm), 0);
		vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_CFG(vfe, wm),
			      VFE_BUS_MMU_PREFETCH_CFG_EN);
		vfe_reg_write(vfe, VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm), ~0u);
		vfe_reg_write(vfe, VFE_BUS_WRITE_CLIENT_CFG(vfe, wm),
			      VFE_BUS_WRITE_CLIENT_CFG_EN |
			      VFE_BUS_WRITE_CLIENT_CFG_VIR_FRM);
	}

	return 0;
}

/*
 * vfe_680_pix_load_glut - Load the gamma table into one LUT bank
 * @vfe: VFE device
 * @win: the IFE register window
 * @table: 256 12-bit outputs
 * @bank: the DMI bank to load, then select for the module
 *
 * The module reads from the bank its bank register names, so a table can
 * be loaded into the other bank while streaming and switched in whole.
 */
static void vfe_680_pix_load_glut(struct vfe_device *vfe, void __iomem *win,
				  const u16 *table, u32 bank)
{
	u32 base = VFE_PIX_GLUT;
	unsigned int i, sel;

	vfe_pix_ext_write(vfe, win, base + VFE_PIX_GLUT_DMI_LUT_BANK_CFG, bank);
	for (sel = VFE_PIX_GLUT_SEL_G; sel <= VFE_PIX_GLUT_SEL_R; sel++) {
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_GLUT_DMI_LUT_CFG, sel);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_GLUT_DMI_CFG,
				  VFE_PIX_GLUT_DMI_CFG_AUTO_LOAD_EN);
		for (i = 0; i < VFE_PIX_GLUT_SIZE; i++)
			writel(table[i] & 0xfff, win + base + VFE_PIX_GLUT_DMI_DATA);
	}
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_GLUT_DMI_LUT_CFG, VFE_PIX_GLUT_SEL_NONE);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_GLUT_MODULE_LUT_BANK_CFG, bank);
}

/*
 * vfe_680_pix_set_cc - Program the colour correction matrix
 * @vfe: VFE device
 * @win: the IFE register window
 * @m: the control's Q10 matrix, RGB row-major, then the R, G, B offsets
 *
 * The registers group coefficients by input channel (A: G, B: B, C: R),
 * each for the G, B and R outputs in turn. Twelve significant bits; a
 * matrix with a coefficient beyond +-2.0 moves to Q9 (or lower) whole.
 */
static void vfe_680_pix_set_cc(struct vfe_device *vfe, void __iomem *win, const s32 *m)
{
	u32 base = VFE_PIX_CC;
	s32 c[9] = {
		m[4], m[7], m[1],	/* G in: to G, B, R out */
		m[5], m[8], m[2],	/* B in */
		m[3], m[6], m[0],	/* R in */
	};
	s32 k[3] = { m[10], m[11], m[9] };
	unsigned int i, q = VFE_PIX_CC_Q10;
	s32 max = 0;

	for (i = 0; i < 9; i++)
		max = max(max, abs(c[i]));
	while (max > 2047 && q > 0) {
		q--;
		max >>= 1;
		for (i = 0; i < 9; i++)
			c[i] /= 2;
	}

	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_A0, VFE_PIX_CC_PAIR(c[0], c[1]));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_A2, c[2] & 0xfff);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_B0, VFE_PIX_CC_PAIR(c[3], c[4]));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_B2, c[5] & 0xfff);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_C0, VFE_PIX_CC_PAIR(c[6], c[7]));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_COEFF_C2, c[8] & 0xfff);
	for (i = 0; i < 3; i++)
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_K(i),
				  VFE_PIX_CC_K_VAL(clamp(k[i], -1024, 1023)));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_CC_Q, q);
}

struct vfe_lsc_grid {
	unsigned int level;	/* log2 of the sub-blocks per block */
	unsigned int cols;	/* blocks across */
	unsigned int rows;	/* blocks down */
	unsigned int sub_w;	/* sub-block size, channel pixels */
	unsigned int sub_h;
	unsigned int over_x;	/* the grid's overhang on the left and top */
	unsigned int over_y;
};

/*
 * vfe_680_lsc_grid - Lay the hardware mesh over the image
 * @cw: channel (half) width
 * @ch: channel (half) height
 * @g: the grid
 *
 * The vendor's grid search: blocks of a whole number of sub-blocks, as
 * many sub-blocks as the block size allows (up to eight), the grid
 * centred on the image with at most a block of overhang, and fewer
 * blocks for small images.
 */
static void vfe_680_lsc_grid(unsigned int cw, unsigned int ch, struct vfe_lsc_grid *g)
{
	unsigned int cols = VFE_PIX_LSC_MESH_COLS_MAX, rows = VFE_PIX_LSC_MESH_ROWS_MAX;
	unsigned int level = 4, sub_w, sub_h, blk_w, blk_h, over_w, over_h;

	do {
		if (level == 0 && (cols <= 8 || rows <= 6))
			break;
		if (level > 0) {
			level--;
		} else if (cols > 8 && rows > 6) {
			cols -= 4;
			rows -= 3;
			level = 3;
		}
		sub_w = DIV_ROUND_UP(DIV_ROUND_UP(cw, cols), 1 << level);
		blk_w = sub_w << level;
		over_w = blk_w * cols - cw;
		sub_h = DIV_ROUND_UP(DIV_ROUND_UP(ch, rows), 1 << level);
		blk_h = sub_h << level;
		over_h = blk_h * rows - ch;
	} while (blk_w < 18 || blk_h < 9 || sub_w < 9 || sub_h < 9 ||
		 over_w >= blk_w || over_h >= blk_h ||
		 blk_w - (over_w + 1) / 2 < 18 ||
		 sub_w - ((over_w + 1) / 2) % sub_w < 9);

	g->level = level;
	g->cols = cols;
	g->rows = rows;
	g->sub_w = sub_w;
	g->sub_h = sub_h;
	g->over_x = (over_w + 1) / 2;
	g->over_y = (over_h + 1) / 2;
}

/*
 * vfe_680_lsc_sample - The control's mesh resampled on the hardware grid
 * @mesh: one channel's 13 x 17 Q10 gains over the whole frame
 * @cw, @ch: channel (half) width and height
 * @g: the grid
 * @col, @row: hardware mesh point
 *
 * The control's mesh spans the channel image, first and last points on
 * its first and last pixels; the hardware's points sit on block corners
 * and may fall outside the image, where the mesh is extended linearly.
 */
static u32 vfe_680_lsc_sample(const u16 *mesh, unsigned int cw, unsigned int ch,
			      const struct vfe_lsc_grid *g, unsigned int col, unsigned int row)
{
	s64 px = (s64)col * (g->sub_w << g->level) - g->over_x;
	s64 py = (s64)row * (g->sub_h << g->level) - g->over_y;
	s64 tx = (px * VFE_PIX_LSC_MESH_COLS_MAX << 16) / (s64)max(cw - 1, 1u);
	s64 ty = (py * VFE_PIX_LSC_MESH_ROWS_MAX << 16) / (s64)max(ch - 1, 1u);
	s32 ix = clamp((s32)(tx >> 16), -1, VFE_PIX_LSC_MESH_COLS_MAX);
	s32 iy = clamp((s32)(ty >> 16), -1, VFE_PIX_LSC_MESH_ROWS_MAX);
	u32 fx = (u32)(tx - ((s64)ix << 16)) & 0xffff;
	u32 fy = (u32)(ty - ((s64)iy << 16)) & 0xffff;
	s32 v[2][2];
	unsigned int i, j;

	for (j = 0; j < 2; j++) {
		for (i = 0; i < 2; i++) {
			s32 x = ix + i, y = iy + j, a, b;

			/* a step outside the mesh continues its last slope */
			if (x < 0)
				x = 0, a = 1;
			else if (x > VFE_PIX_LSC_MESH_COLS_MAX)
				x = VFE_PIX_LSC_MESH_COLS_MAX, a = -1;
			else
				a = 0;
			if (y < 0)
				y = 0, b = 1;
			else if (y > VFE_PIX_LSC_MESH_ROWS_MAX)
				y = VFE_PIX_LSC_MESH_ROWS_MAX, b = -1;
			else
				b = 0;
			v[j][i] = 2 * mesh[y * QCOM_CAMSS_LSC_MESH_COLS + x] -
				  mesh[(y + b) * QCOM_CAMSS_LSC_MESH_COLS + x + a];
			if (!a && !b)
				v[j][i] = mesh[y * QCOM_CAMSS_LSC_MESH_COLS + x];
		}
	}

	{
		s64 top = (s64)v[0][0] * (0x10000 - fx) + (s64)v[0][1] * fx;
		s64 bot = (s64)v[1][0] * (0x10000 - fx) + (s64)v[1][1] * fx;
		s64 val = (top * (0x10000 - fy) + bot * fy) >> 32;

		return clamp_t(s64, val, VFE_PIX_LSC_GAIN_MIN, VFE_PIX_LSC_GAIN_MAX);
	}
}

/*
 * vfe_680_pix_set_lsc - Program the lens shading mesh
 * @vfe: VFE device
 * @win: the IFE register window
 * @line: the PIX line
 *
 * An all-zero mesh disables the module. Otherwise the tables go into the
 * bank not in use, the grid registers follow (they are not banked), and
 * the module is switched to the new bank.
 */
static void vfe_680_pix_set_lsc(struct vfe_device *vfe, void __iomem *win, struct vfe_line *line)
{
	struct vfe_pix_params *params = &line->pix_params;
	const struct v4l2_mbus_framefmt *in = &line->fmt[MSM_VFE_PAD_SINK];
	unsigned int cw = in->width / 2, ch = in->height / 2;
	const u16 *mesh[QCOM_CAMSS_LSC_MESH_CHANNELS];
	struct vfe_lsc_grid g;
	u32 base = VFE_PIX_LSC, bank, sub_w, sub_h, blk_w, blk_h, inv_w, inv_h;
	u32 sgx, sgy, blk_x, blk_y, sub_x, sub_y, pix_x, pix_y;
	unsigned int i, row, col, sel;
	bool enabled = false;

	for (i = 0; i < QCOM_CAMSS_LSC_MESH_SIZE && !enabled; i++)
		enabled = params->lsc[i] != 0;
	if (!enabled || cw < 32 || ch < 32) {
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_MODULE_CFG, 0);
		return;
	}

	for (i = 0; i < QCOM_CAMSS_LSC_MESH_CHANNELS; i++)
		mesh[i] = params->lsc + i * QCOM_CAMSS_LSC_MESH_ROWS * QCOM_CAMSS_LSC_MESH_COLS;

	vfe_680_lsc_grid(cw, ch, &g);
	sub_w = g.sub_w - 1;
	sub_h = g.sub_h - 1;
	blk_w = (g.sub_w << g.level) - 1;
	blk_h = (g.sub_h << g.level) - 1;
	inv_w = clamp((1u << 20) / g.sub_w, 512u, 116508u);
	inv_h = clamp((1u << 20) / g.sub_h, 1024u, 116508u);
	/* where the image's first pixel falls in the grid */
	sgx = g.over_x / g.sub_w;
	sgy = g.over_y / g.sub_h;
	blk_x = min(sgx >> g.level, 15u);
	blk_y = min(sgy >> g.level, 11u);
	sub_x = sgx & ((1 << g.level) - 1);
	sub_y = sgy & ((1 << g.level) - 1);
	pix_x = g.over_x % g.sub_w;
	pix_y = g.over_y % g.sub_h;

	bank = params->lsc_bank ^ 1;
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_DMI_LUT_BANK_CFG, bank);
	for (sel = VFE_PIX_LSC_SEL_RED; sel <= VFE_PIX_LSC_SEL_GRID; sel++) {
		unsigned int lo = sel == VFE_PIX_LSC_SEL_RED ? 0 : 3;	/* R or B */
		unsigned int hi = sel == VFE_PIX_LSC_SEL_RED ? 1 : 2;	/* Gr or Gb */

		vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_DMI_LUT_CFG, sel);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_DMI_CFG,
				  VFE_PIX_LSC_DMI_CFG_AUTO_LOAD_EN);
		for (row = 0; row <= g.rows; row++) {
			for (col = 0; col <= g.cols; col++) {
				u32 word = 0;

				if (sel != VFE_PIX_LSC_SEL_GRID)
					word = vfe_680_lsc_sample(mesh[lo], cw, ch, &g, col, row) |
					       vfe_680_lsc_sample(mesh[hi], cw, ch, &g, col, row) <<
					       VFE_PIX_LSC_GAIN_SHIFT;
				writel(word, win + base + VFE_PIX_LSC_DMI_DATA);
			}
		}
	}
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_DMI_LUT_CFG, VFE_PIX_LSC_SEL_NONE);

	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(0),
			  VFE_PIX_LSC_CFG0(g.cols - 1, g.rows - 1));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(1), VFE_PIX_LSC_CFG1(blk_w, blk_h));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(2), VFE_PIX_LSC_CFG23(sub_w, inv_w));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(3), VFE_PIX_LSC_CFG23(sub_h, inv_h));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(4), VFE_PIX_LSC_CFG4(pix_x, pix_y));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(5), 0);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(6), min(pix_y * inv_h, 932064u));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(7), 0);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(8), 0);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(9),
			  VFE_PIX_LSC_CFG9(0, in->width - 1));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(10),
			  VFE_PIX_LSC_CFG9(0, in->height - 1));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(11), 0);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_CFG(12), 0);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_MODULE_LUT_BANK_CFG, bank);
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_LSC_MODULE_CFG,
			  VFE_PIX_LSC_MODULE_CFG_EN |
			  VFE_PIX_LSC_MODULE_CFG_NUM_SUBBLOCKS(g.level) |
			  VFE_PIX_LSC_MODULE_CFG_INIT_BLOCK_X(blk_x) |
			  VFE_PIX_LSC_MODULE_CFG_INIT_BLOCK_Y(blk_y) |
			  VFE_PIX_LSC_MODULE_CFG_INIT_SUBBLOCK_X(sub_x) |
			  VFE_PIX_LSC_MODULE_CFG_INIT_SUBBLOCK_Y(sub_y));
	params->lsc_bank = bank;
	dev_dbg(vfe->camss->dev,
		"vfe%u lsc %ux%u: level %u grid %ux%u sub %ux%u over %u,%u start blk %u,%u sub %u,%u pix %u,%u\n",
		vfe->id, cw, ch, g.level, g.cols, g.rows, g.sub_w, g.sub_h, g.over_x, g.over_y,
		blk_x, blk_y, sub_x, sub_y, pix_x, pix_y);
}

/*
 * vfe_680_pix_set_params - Apply the PIX line's controls
 * @line: the PIX line
 *
 * Called at start and whenever a control changes while streaming. The
 * white balance gains and the black level live in the white balance
 * module (the subtract takes the sensor pedestal off before the gains and
 * before the gamma LUT); the colour correction matrix in its own; and a
 * changed gamma table goes into the LUT bank not in use.
 */
static void vfe_680_pix_set_params(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	void __iomem *win = vfe_pix_ext_window(vfe, VFE_PIX_EXT_SIZE);
	struct vfe_pix_params *params = &line->pix_params;
	u32 sub;

	if (!win)
		return;

	vfe_680_cgc_override(vfe, true);
	vfe_680_pix_set_cc(vfe, win, params->ccm);
	if (params->gamma_dirty) {
		params->gamma_bank ^= 1;
		vfe_680_pix_load_glut(vfe, win, params->gamma, params->gamma_bank);
		params->gamma_dirty = false;
	}
	if (params->lsc_dirty) {
		vfe_680_pix_set_lsc(vfe, win, line);
		params->lsc_dirty = false;
	}
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_GAIN_G,
			  VFE_PIX_WB_GAIN(line->pix_params.wb_g));
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_GAIN_B,
			  VFE_PIX_WB_GAIN(line->pix_params.wb_b));
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_GAIN_R,
			  VFE_PIX_WB_GAIN(line->pix_params.wb_r));
	/* the black level comes off before the gains and before the gamma LUT */
	sub = VFE_PIX_WB_SUB(min(line->pix_params.black_level << 2, 0xfffu));
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_SUB_G, sub);
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_SUB_B, sub);
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_SUB_R, sub);
	if (line->output.state == VFE_OUTPUT_ON)
		vfe_680_cgc_override(vfe, false);
}

/*
 * vfe_pix_program_iq - Program the image quality modules for NV12 output
 * @vfe: VFE device
 * @line: the PIX line
 *
 * The gamma LUT and the control-set modules (white balance, black level,
 * colour correction), demosaic with the OPE's default interpolation
 * constants, a BT.601 full-range RGB to YCbCr transform scaled from the
 * 14-bit demosaic output to the 12-bit colour transform output (with the
 * sensor black level subtracted on its input), and the VID output path
 * downscalers: luma to the compose size, chroma from the full 4:4:4 frame
 * to half of it in both directions.
 */
static int vfe_pix_program_iq(struct vfe_device *vfe, struct vfe_line *line)
{
	void __iomem *win = vfe_pix_ext_window(vfe, VFE_PIX_EXT_SIZE);
	/* BT.601 full range, Q10, times 1/4 for 14-bit in to 12-bit out */
	static const struct {
		s16 g, b, r, offset;
	} cst[3] = {
		{ 150,  29,  77, 0 },			/* Y  */
		{ -85, 128, -43, VFE_PIX_CST_CHROMA_OFFSET },	/* Cb */
		{ -107, -21, 128, VFE_PIX_CST_CHROMA_OFFSET },	/* Cr */
	};
	u32 base;
	unsigned int i;

	if (!win)
		return -ENOMEM;

	vfe_680_cgc_override(vfe, true);

	/* the table goes into bank 0 here; set_params then only loads changes */
	line->pix_params.gamma_bank = 0;
	line->pix_params.gamma_dirty = false;
	vfe_680_pix_load_glut(vfe, win, line->pix_params.gamma, 0);
	line->pix_params.lsc_bank = 1;
	line->pix_params.lsc_dirty = true;
	vfe_680_pix_set_params(line);
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_PIX_WB_CFG_94, ~0u);
	vfe_pix_ext_write(vfe, win, VFE_PIX_WB + VFE_CLC_MODULE_CFG, VFE_CLC_MODULE_CFG_EN);
	vfe_pix_ext_write(vfe, win, VFE_PIX_CC + VFE_CLC_MODULE_CFG, VFE_CLC_MODULE_CFG_EN);

	base = VFE_PIX_DEMOSAIC;
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_DEMOSAIC_INTERP_COEFF,
			  VFE_PIX_DEMOSAIC_LAMBDA_G(128) | VFE_PIX_DEMOSAIC_LAMBDA_RB(0));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_DEMOSAIC_CLASSIFIER_0,
			  VFE_PIX_DEMOSAIC_AK(128));
	vfe_pix_ext_write(vfe, win, base + VFE_PIX_DEMOSAIC_CLASSIFIER_1,
			  VFE_PIX_DEMOSAIC_WK(102));
	vfe_pix_ext_write(vfe, win, base + VFE_CLC_MODULE_CFG,
			  VFE_CLC_MODULE_CFG_EN |
			  VFE_PIX_DEMOSAIC_CFG_DYN_G_CLAMP_EN |
			  VFE_PIX_DEMOSAIC_CFG_DYN_RB_CLAMP_EN);

	for (i = 0; i < ARRAY_SIZE(cst); i++) {
		base = VFE_PIX_CST + VFE_PIX_CST_GROUP(i);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_COEFF_GB,
				  VFE_PIX_CST_COEFF(cst[i].g, cst[i].b));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_COEFF_R,
				  cst[i].r & 0x1fff);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_OFFSET_OUT,
				  VFE_PIX_CST_OFFSET(cst[i].offset));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_OFFSET_IN, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_CLAMP_MIN, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_CST_CLAMP_MAX,
				  VFE_PIX_CST_CLAMP(VFE_PIX_CST_OUT_MAX));
	}
	vfe_pix_ext_write(vfe, win, VFE_PIX_CST + VFE_CLC_MODULE_CFG,
			  VFE_CLC_MODULE_CFG_EN);

	vfe_pix_ext_write(vfe, win, VFE_PIX_GLUT + VFE_CLC_MODULE_CFG, VFE_CLC_MODULE_CFG_EN);

	/*
	 * The VID output scalers: the sink pad's compose rectangle is the
	 * scaled size, the frame then cropped to the source pad's crop
	 * rectangle (the crop windows) and written at that size. The chroma
	 * reaches its scaler at the full frame size (4:4:4), so the scaler is
	 * told the full size and twice the luma ratio in both directions,
	 * giving half of the scaled size. Told half the height, it left the
	 * chroma unscaled vertically and the crop took the top of it.
	 */
	{
		const struct v4l2_mbus_framefmt *in = &line->fmt[MSM_VFE_PAD_SINK];
		const struct v4l2_rect *scaled = &line->compose;
		u32 cfg;

		base = VFE_PIX_DS_MN_Y_VID;
		cfg = 0;
		if (scaled->width != in->width)
			cfg |= VFE_PIX_DS_MN_CFG_H_SCALE_EN;
		if (scaled->height != in->height)
			cfg |= VFE_PIX_DS_MN_CFG_V_SCALE_EN;
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_CFG, cfg);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_IMAGE_SIZE,
				  ((in->width - 1) << 16) | (in->height - 1));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_H_CFG,
				  VFE_PIX_DS_MN_RESOLUTION_FULL |
				  VFE_PIX_DS_MN_STEP(in->width, scaled->width));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_H_PHASE, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_V_CFG,
				  VFE_PIX_DS_MN_RESOLUTION_FULL |
				  VFE_PIX_DS_MN_STEP(in->height, scaled->height));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_V_PHASE, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_CLC_MODULE_CFG,
				  cfg ? VFE_CLC_MODULE_CFG_EN : 0);

		base = VFE_PIX_DS_MN_C_VID;
		cfg = VFE_PIX_DS_MN_CFG_H_SCALE_EN | VFE_PIX_DS_MN_CFG_V_SCALE_EN;
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_CFG, cfg);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_IMAGE_SIZE,
				  ((in->width - 1) << 16) | (in->height - 1));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_H_CFG,
				  VFE_PIX_DS_MN_RESOLUTION_FULL |
				  VFE_PIX_DS_MN_STEP(in->width, scaled->width / 2));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_H_PHASE, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_V_CFG,
				  VFE_PIX_DS_MN_RESOLUTION_FULL |
				  VFE_PIX_DS_MN_STEP(in->height, scaled->height / 2));
		vfe_pix_ext_write(vfe, win, base + VFE_PIX_DS_MN_V_PHASE, 0);
		vfe_pix_ext_write(vfe, win, base + VFE_CLC_MODULE_CFG, VFE_CLC_MODULE_CFG_EN);
	}
	vfe_680_cgc_override(vfe, false);

	return 0;
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
	vfe_680_cgc_override(vfe, true);
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
	vfe_680_cgc_override(vfe, false);
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
	if (vfe->wm_output_map[VFE_LINE_PIX] != VFE_LINE_PIX || !pix_rearm)
		return;

	vfe_pix_program_output_stage(vfe, &vfe->line[VFE_LINE_PIX]);
	vfe_pix_program_iq(vfe, &vfe->line[VFE_LINE_PIX]);
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

	vfe_pix_program_output_stage(vfe, line);
	vfe_pix_program_iq(vfe, line);
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
	/* a DT that describes the whole block needs no second mapping */
	if (resource_size(r) >= size)
		ext[vfe->id] = vfe->base;
	else
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
			if ((pix_drain & BIT(wm)) ||
			    (wm >= VFE_PIX_DRAIN_WM_FIRST && wm <= VFE_PIX_DRAIN_WM_LAST))
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
	.pix_set_params = vfe_680_pix_set_params,
	.pix_gamma_default = vfe_pix_gamma_2_2,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
};
