/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ICP provides HFI interface used by IPE and BPS V4L2 m2m drivers.
 *
 * Copyright (c) 2026 Bryan O'Donoghue.
 */
#ifndef __CAMSS_ICP_H__
#define __CAMSS_ICP_H__

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/types.h>

#include "camss-icp-hfi.h"

struct icp_device;

/*
 * ICP Context
 *
 * Represents a processing session with firmware.
 * Created per-stream in IPE/BPS V4L2 driver.
 */
struct icp_context {
	u32 id;
	u32 dev_type;
	u32 fw_handle;
	struct completion done;
	int result;
	void *priv;
	void (*callback)(struct icp_context *ctx, int status);
};

/*
 * UBWC Configuration
 */
struct icp_ubwc_cfg {
	u32 fetch_cfg;
	u32 write_cfg;
};

/*
 * Frame Processing Descriptor
 */
struct icp_frame_request {
	dma_addr_t input_iova;
	u32 input_size;
	dma_addr_t output_iova;
	u32 output_size;
	dma_addr_t cmdbufs_iova;
	u32 cmdbufs_size;
	void *priv;
};

struct icp_hfi *icp_hfi_get(struct device *dev);
void icp_hfi_put(struct icp_hfi *hfi);

#endif /* __CAMSS_ICP_H__ */
