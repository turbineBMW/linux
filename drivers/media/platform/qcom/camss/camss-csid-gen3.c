// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen3.h"

#define CSID_IO_PATH_CFG0(csid)		(0x4 * (csid))
#define		OUTPUT_IFE_EN			0x100
#define		INTERNAL_CSID			1

#define CSID_RST_CFG			0xC
#define		RST_MODE			BIT(0)
#define		RST_LOCATION			BIT(4)

#define CSID_RST_CMD			0x10
#define		SELECT_HW_RST			BIT(0)
#define		SELECT_IRQ_RST			BIT(2)

#define CSID_IRQ_CMD			0x14
#define		IRQ_CMD_CLEAR			BIT(0)

#define CSID_RUP_AUP_CMD		0x18
#define		CSID_RUP_AUP_RDI(rdi)		((BIT(4) | BIT(20)) << (rdi))

#define CSID_TOP_IRQ_STATUS		0x7C
#define		 TOP_IRQ_STATUS_RESET_DONE	BIT(0)

#define CSID_TOP_IRQ_MASK		0x80
#define CSID_TOP_IRQ_CLEAR		0x84
#define CSID_TOP_IRQ_SET		0x88

#define CSID_CSI2_RX_IRQ_STATUS		0x9C
#define CSID_CSI2_RX_IRQ_MASK		0xA0
#define CSID_CSI2_RX_IRQ_CLEAR		0xA4
#define CSID_CSI2_RX_IRQ_SET		0xA8

#define IS_CSID_690(csid)	((csid->camss->res->version == CAMSS_8775P) \
				 || (csid->camss->res->version == CAMSS_8300))
#define CSID_BUF_DONE_IRQ_STATUS	0x8C
#define BUF_DONE_IRQ_STATUS_RDI_OFFSET  (csid_is_lite(csid) ? \
						((IS_CSID_690(csid) ? 0 : 1)) : \
						((IS_CSID_690(csid) ? 13 : 14)))
#define CSID_BUF_DONE_IRQ_MASK		0x90
#define CSID_BUF_DONE_IRQ_CLEAR		0x94
#define CSID_BUF_DONE_IRQ_SET		0x98

#define CSID_CSI2_RDIN_IRQ_STATUS(rdi)	(0xEC + 0x10 * (rdi))
#define		RUP_DONE_IRQ_STATUS		BIT(23)

#define CSID_CSI2_RDIN_IRQ_CLEAR(rdi)	(0xF4 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_SET(rdi)	(0xF8 + 0x10 * (rdi))

#define CSID_CSI2_RX_CFG0		0x200
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES	0
#define		CSI2_RX_CFG0_VC_MODE		3
#define		CSI2_RX_CFG0_DL0_INPUT_SEL	4
#define		CSI2_RX_CFG0_PHY_NUM_SEL	20
#define		CSI2_RX_CFG0_PHY_TYPE_SEL_CPHY	BIT(24)
#define		CSI2_RX_CFG0_TPG_MUX_EN		BIT(27)
#define		CSI2_RX_CFG0_TPG_MUX_SEL	GENMASK(29, 28)

#define CSID_CSI2_RX_CFG1		0x204
#define		CSI2_RX_CFG1_ECC_CORRECTION_EN	BIT(0)
#define		CSI2_RX_CFG1_VC_MODE		BIT(2)

#define CSID_RDI_CFG0(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x300 + 0x100 * (rdi)) :\
					(0x500 + 0x100 * (rdi)))
#define		RDI_CFG0_TIMESTAMP_EN		BIT(6)
#define		RDI_CFG0_TIMESTAMP_STB_SEL	BIT(8)
#define		RDI_CFG0_DECODE_FORMAT		12
#define		RDI_CFG0_DT			16
#define		RDI_CFG0_VC			22
#define		RDI_CFG0_DT_ID			27
#define		RDI_CFG0_EN			BIT(31)

#define CSID_RDI_CTRL(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x304 + 0x100 * (rdi)) :\
					(0x504 + 0x100 * (rdi)))
#define		RDI_CTRL_START_CMD		BIT(0)

#define CSID_RDI_CFG1(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
					(0x310 + 0x100 * (rdi)) :\
					(0x510 + 0x100 * (rdi)))
#define		RDI_CFG1_DROP_H_EN		BIT(5)
#define		RDI_CFG1_DROP_V_EN		BIT(6)
#define		RDI_CFG1_CROP_H_EN		BIT(7)
#define		RDI_CFG1_CROP_V_EN		BIT(8)
#define		RDI_CFG1_PIX_STORE		BIT(10)
#define		RDI_CFG1_PACKING_FORMAT_MIPI	BIT(15)

#define CSID_RDI_IRQ_SUBSAMPLE_PATTERN(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
							(0x348 + 0x100 * (rdi)) :\
							(0x548 + 0x100 * (rdi)))
#define CSID_RDI_IRQ_SUBSAMPLE_PERIOD(rdi)	(csid_is_lite(csid) && IS_CSID_690(csid) ?\
							(0x34C + 0x100 * (rdi)) :\
							(0x54C + 0x100 * (rdi)))
#define CSI2_RX_CFG0_PHY_SEL_BASE_IDX	1

/*
 * csid_reg_write - Write a CSID register, logging it under dynamic debug
 * @csid: CSID device
 * @reg: register offset from the CSID base
 * @val: value to write
 *
 * See vfe_reg_write() in camss-vfe-gen3.c for the dynamic debug recipe.
 */
static inline void csid_reg_write(struct csid_device *csid, u32 reg, u32 val)
{
	dev_dbg(csid->camss->dev, "csid%u wr 0x%05x = 0x%08x\n", csid->id, reg, val);
	writel(val, csid->base + reg);
}

/*
 * csid_wrapper_write - Write a CSID wrapper register, logging it
 * @csid: CSID device
 * @reg: register offset from the CSID wrapper base
 * @val: value to write
 */
static inline void csid_wrapper_write(struct csid_device *csid, u32 reg,
				      u32 val)
{
	dev_dbg(csid->camss->dev, "csid-wrapper wr 0x%05x = 0x%08x\n", reg, val);
	writel(val, csid->camss->csid_wrapper_base + reg);
}

static void __csid_configure_rx(struct csid_device *csid,
				struct csid_phy_config *phy, int vc)
{
	int val;
	struct camss *camss;

	camss = csid->camss;
	val = (phy->lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= phy->lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;

	if (camss->tpg && csid->tpg_linked &&
	    camss->tpg[phy->csiphy_id].testgen.mode != TPG_PAYLOAD_MODE_DISABLED) {
		val |= FIELD_PREP(CSI2_RX_CFG0_TPG_MUX_SEL, phy->csiphy_id + 1);
		val |= CSI2_RX_CFG0_TPG_MUX_EN;
	} else {
		val |= (phy->csiphy_id + CSI2_RX_CFG0_PHY_SEL_BASE_IDX)
			<< CSI2_RX_CFG0_PHY_NUM_SEL;
		if (phy->is_cphy)
			val |= CSI2_RX_CFG0_PHY_TYPE_SEL_CPHY;
	}

	csid_reg_write(csid, CSID_CSI2_RX_CFG0, val);

	val = CSI2_RX_CFG1_ECC_CORRECTION_EN;
	if (vc > 3)
		val |= CSI2_RX_CFG1_VC_MODE;

	csid_reg_write(csid, CSID_CSI2_RX_CFG1, val);
}

static void __csid_ctrl_rdi(struct csid_device *csid, int enable, u8 rdi)
{
	int val = 0;

	if (enable)
		val = RDI_CTRL_START_CMD;

	csid_reg_write(csid, CSID_RDI_CTRL(rdi), val);
}

static void __csid_configure_wrapper(struct csid_device *csid)
{
	u32 val;

	/* csid lite doesn't need to configure top register */
	if (csid->res->is_lite)
		return;

	val = OUTPUT_IFE_EN | INTERNAL_CSID;
	csid_wrapper_write(csid, CSID_IO_PATH_CFG0(csid->id), val);
}

static void __csid_configure_rdi_stream(struct csid_device *csid, u8 enable, u8 port, u8 vc)
{
	u32 val;
	u8 lane_cnt = csid->phy.lane_cnt;
	/* Source pads matching RDI channels on hardware. Pad 1 -> RDI0, Pad 2 -> RDI1, etc. */
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + port];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);

	if (!lane_cnt)
		lane_cnt = 4;

	/*
	 * DT_ID is a two bit bitfield that is concatenated with
	 * the four least significant bits of the five bit VC
	 * bitfield to generate an internal CID value.
	 *
	 * CSID_RDI_CFG0(port)
	 * DT_ID : 28:27
	 * VC    : 26:22
	 * DT    : 21:16
	 *
	 * CID   : VC 3:0 << 2 | DT_ID 1:0
	 */
	u8 dt_id = port & 0x03;

	val = RDI_CFG0_TIMESTAMP_EN;
	val |= RDI_CFG0_TIMESTAMP_STB_SEL;
	/* note: for non-RDI path, this should be format->decode_format */
	val |= DECODE_FORMAT_PAYLOAD_ONLY << RDI_CFG0_DECODE_FORMAT;
	val |= vc << RDI_CFG0_VC;
	val |= format->data_type << RDI_CFG0_DT;
	val |= dt_id << RDI_CFG0_DT_ID;

	csid_reg_write(csid, CSID_RDI_CFG0(port), val);

	val = RDI_CFG1_PACKING_FORMAT_MIPI;
	val |= RDI_CFG1_PIX_STORE;
	val |= RDI_CFG1_DROP_H_EN;
	val |= RDI_CFG1_DROP_V_EN;
	val |= RDI_CFG1_CROP_H_EN;
	val |= RDI_CFG1_CROP_V_EN;

	csid_reg_write(csid, CSID_RDI_CFG1(port), val);

	val = 0;
	csid_reg_write(csid, CSID_RDI_IRQ_SUBSAMPLE_PERIOD(port), val);

	val = 1;
	csid_reg_write(csid, CSID_RDI_IRQ_SUBSAMPLE_PATTERN(port), val);

	val = 0;
	csid_reg_write(csid, CSID_RDI_CTRL(port), val);

	val = readl(csid->base + CSID_RDI_CFG0(port));

	if (enable)
		val |= RDI_CFG0_EN;
	csid_reg_write(csid, CSID_RDI_CFG0(port), val);
}

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	u8 i;

	__csid_configure_wrapper(csid);

	/* Loop through all enabled ports and configure a stream for each */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			__csid_configure_rdi_stream(csid, enable, i, 0);
			__csid_configure_rx(csid, &csid->phy, 0);
			__csid_ctrl_rdi(csid, enable, i);
		}
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	return 0;
}

static void csid_subdev_reg_update(struct csid_device *csid, int port_id, bool clear)
{
	if (clear) {
		csid->reg_update &= ~CSID_RUP_AUP_RDI(port_id);
	} else {
		csid->reg_update |= CSID_RUP_AUP_RDI(port_id);
		csid_reg_write(csid, CSID_RUP_AUP_CMD, csid->reg_update);
	}
}

/*
 * csid_isr - CSID module interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 val, buf_done_val;
	u8 reset_done;
	int i;

	val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	csid_reg_write(csid, CSID_TOP_IRQ_CLEAR, val);
	reset_done = val & TOP_IRQ_STATUS_RESET_DONE;

	val = readl(csid->base + CSID_CSI2_RX_IRQ_STATUS);
	csid_reg_write(csid, CSID_CSI2_RX_IRQ_CLEAR, val);

	buf_done_val = readl(csid->base + CSID_BUF_DONE_IRQ_STATUS);
	csid_reg_write(csid, CSID_BUF_DONE_IRQ_CLEAR, buf_done_val);

	/* Read and clear IRQ status for each enabled RDI channel */
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			val = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));
			csid_reg_write(csid, CSID_CSI2_RDIN_IRQ_CLEAR(i), val);

			if (val & RUP_DONE_IRQ_STATUS)
				/* clear the reg update bit */
				csid_subdev_reg_update(csid, i, true);

			if (buf_done_val & BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i)) {
				/*
				 * For Titan Gen3, bus done and RUP IRQ have been moved to
				 * CSID from VFE. Once CSID received bus done, need notify
				 * VFE of this event. Trigger VFE to handle bus done process.
				 */
				camss_buf_done(csid->camss, csid->id, i);
			}
		}

	val = IRQ_CMD_CLEAR;
	csid_reg_write(csid, CSID_IRQ_CMD, val);

	if (reset_done)
		complete(&csid->reset_complete);

	return IRQ_HANDLED;
}

/*
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	unsigned long time;
	u32 val;
	int i;

	reinit_completion(&csid->reset_complete);

	csid_reg_write(csid, CSID_TOP_IRQ_CLEAR, 1);
	csid_reg_write(csid, CSID_IRQ_CMD, 1);
	csid_reg_write(csid, CSID_TOP_IRQ_MASK, 1);

	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS; i++)
		if (csid->phy.en_vc & BIT(i)) {
			csid_reg_write(csid, CSID_BUF_DONE_IRQ_CLEAR,
				       BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i));
			csid_reg_write(csid, CSID_IRQ_CMD, IRQ_CMD_CLEAR);
			csid_reg_write(csid, CSID_BUF_DONE_IRQ_MASK,
				       BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i));
		}

	/* preserve registers */
	val = RST_LOCATION | RST_MODE;
	csid_reg_write(csid, CSID_RST_CFG, val);

	val = SELECT_HW_RST | SELECT_IRQ_RST;
	csid_reg_write(csid, CSID_RST_CMD, val);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -EIO;
	}

	return 0;
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_DISABLED;
}

const struct csid_hw_ops csid_ops_gen3 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
	.reg_update = csid_subdev_reg_update,
};
