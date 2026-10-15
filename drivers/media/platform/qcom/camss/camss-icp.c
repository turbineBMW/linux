// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm ICP (Image Control Processor) driver for X1E80100
 *
 */
#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>

#include "camss-icp.h"
#include "camss-icp-hfi.h"

/* CSR register offsets */
#define ICP_CSR_HW_VERSION		0x00
#define ICP_CSR_TCM_SIZE		0x08
#define ICP_CSR_DBG_STATUS		0x44
#define ICP_CSR_DBG_CTRL		0x48

/* CIRQ register offsets (ICP v2) */
#define CIRQ_OB_MASK			0x00
#define CIRQ_OB_CLEAR			0x04
#define CIRQ_OB_SET			0x08
#define CIRQ_OB_STATUS			0x0c
#define CIRQ_OB_CMD			0x10
#define CIRQ_HOST2ICPINT		0x124

/* CIRQ bits, shared by OB_MASK, OB_CLEAR, OB_SET and OB_STATUS */
#define CIRQ_WDT_BARK_WS0		BIT(3)
#define CIRQ_WDT_BITE_WS0		BIT(4)
#define CIRQ_ICP2HOSTINT		BIT(2)
#define CIRQ_ALL			GENMASK(6, 0)

/* OB_CMD: latch the CLEAR or SET register */
#define CIRQ_CMD_SET			BIT(0)
#define CIRQ_CMD_CLEAR			BIT(1)

#define ICP_CLK_MAX			32

#define POLLING_SLEEP_US		1000
#define POLLING_TIMEOUT_US		20000

/* HFI polling - match CAMX: 100us interval, 2 second timeout */
#define HFI_POLL_DELAY_US		100
#define HFI_POLL_TIMEOUT_US		2000000

/* ICC paths */
#define HFI_MAX_ICC_PATHS		4
struct camss_icp_resources {
	int pas_id;
	const char * const *clk_names;
	int clk_num;
	const char * const *noc_names;
	int noc_num;
	struct hfi_resources hfi_res;
};

struct camss_icp {
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *cirq_base;
	int irq;

	struct dev_pm_domain_list *pd_list;

	struct clk_bulk_data clks[ICP_CLK_MAX];
	struct reset_control_bulk_data resets[3];
	struct icc_path *icc_path[HFI_MAX_ICC_PATHS];

	struct icp_hfi hfi;

	bool irq_test;
	struct completion irq_test_done;

	/* lab interface (debugfs): raw HFI packets and DMA buffers */
	struct dentry *dbg;
	struct mutex lab_lock;
	u8 *lab_resp;
	int lab_resp_len;
	struct icp_lab_buf {
		void *vaddr;
		dma_addr_t iova;
		size_t size;
	} lab_buf[16];
	/* IPE/BPS DMA goes through their own SMMU streams: mirror lab buffers there */
	struct iommu_domain *lab_mirror[2];

	phys_addr_t fw_phys;
	size_t fw_size;

	const struct camss_icp_resources *icp_res;
};

struct icp_hfi *icp_hfi_get(struct device *dev)
{
	struct camss_icp *icp = dev_get_drvdata(dev);

	if (!icp)
		return ERR_PTR(-ENODEV);

	get_device(icp->dev);
	return &icp->hfi;
}
EXPORT_SYMBOL(icp_hfi_get);

void icp_hfi_put(struct icp_hfi *phfi)
{
	struct camss_icp *icp = container_of(phfi, struct camss_icp, hfi);

	put_device(icp->dev);
}
EXPORT_SYMBOL(icp_hfi_put);

/*
 * Raise interrupt to firmware - called by HFI layer after CMD_Q write
 */
static void icp_raise_irq(void *priv)
{
	struct camss_icp *icp = priv;

	dev_dbg(icp->dev, "Raising HOST2ICPINT (CIRQ STATUS before: 0x%x)\n",
		readl(icp->cirq_base + CIRQ_OB_STATUS));

	writel(1, icp->cirq_base + CIRQ_HOST2ICPINT);

	/* Small delay then check if anything changed */
	udelay(100);
	dev_dbg(icp->dev, "CIRQ STATUS after HOST2ICPINT: 0x%x\n",
		readl(icp->cirq_base + CIRQ_OB_STATUS));
}

/* Load firmware */
static int icp_load_firmware(struct camss_icp *icp)
{
	const struct camss_icp_resources *icp_res = icp->icp_res;
	const struct firmware *fw;
	struct device_node *node;
	struct resource res;
	const char *fw_name;
	char fw_path[96];
	void *vaddr;
	ssize_t fw_size;
	int ret;

	ret = of_property_read_string(icp->dev->of_node, "firmware-name", &fw_name);
	if (ret)
		return ret;

	/* a name with an extension is used as is (a single-file .mbn) */
	if (strchr(kbasename(fw_name), '.'))
		strscpy(fw_path, fw_name, sizeof(fw_path));
	else
		snprintf(fw_path, sizeof(fw_path), "%s.mdt", fw_name);

	node = of_parse_phandle(icp->dev->of_node, "memory-region", 0);
	if (!node)
		return -ENODEV;

	ret = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (ret)
		return ret;

	dev_dbg(icp->dev, "FW memory: phys=0x%pa size=%llu\n",
		&res.start, resource_size(&res));

	ret = firmware_request_nowarn(&fw, fw_path, icp->dev);
	if (ret)
		return ret;

	fw_size = qcom_mdt_get_size(fw);
	if (fw_size < 0 || (size_t)fw_size > resource_size(&res)) {
		release_firmware(fw);
		return -EINVAL;
	}

	vaddr = ioremap_wc(res.start, resource_size(&res));
	if (!vaddr) {
		release_firmware(fw);
		return -ENOMEM;
	}

	ret = qcom_mdt_load(icp->dev, fw, fw_path, icp_res->pas_id, vaddr,
			    res.start, resource_size(&res), NULL);
	iounmap(vaddr);
	release_firmware(fw);

	if (ret == 0) {
		dev_dbg(icp->dev, "Firmware loaded: %s (%zd bytes)\n", fw_path, fw_size);
		icp->fw_phys = res.start;
		icp->fw_size = fw_size;
	}

	return ret;
}

static irqreturn_t camss_icp_isr_thread(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status;

	/* Re-read status for logging */
	status = readl(icp->cirq_base + CIRQ_OB_STATUS);

	dev_dbg(icp->dev, "IRQ thread: status=0x%x\n", status);

	if (status & CIRQ_WDT_BITE_WS0) {
		dev_err(icp->dev, "ICP watchdog bite!\n");
		icp_hfi_dump_sfr(&icp->hfi);
	}

	/* Process message queue - signals waiters if data present */
	icp_hfi_process_msg_queue(&icp->hfi);

	/* Flush debug queue - async debug messages */
	icp_hfi_flush_debug_queue(&icp->hfi);

	return IRQ_HANDLED;
}

static irqreturn_t camss_icp_isr(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status;

	dev_dbg(icp->dev, "IRQ ISR\n");

	status = readl(icp->cirq_base + CIRQ_OB_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(status, icp->cirq_base + CIRQ_OB_CLEAR);
	writel(CIRQ_CMD_CLEAR, icp->cirq_base + CIRQ_OB_CMD);

	if (icp->irq_test) {
		icp->irq_test = false;
		complete(&icp->irq_test_done);
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

/* Dump GP registers for debugging */
static void icp_dump_gp_regs(struct camss_icp *icp, const char *label)
{
	u32 regs[20];
	int i;

	dev_info(icp->dev, "=== %s ===\n", label);
	dev_info(icp->dev, "Raw CSR offsets 0x20-0x6C (GP registers):\n");

	for (i = 0; i < 20; i++)
		regs[i] = readl(icp->csr_base + 0x20 + i * 4);

	for (i = 0; i < 20; i += 4)
		dev_info(icp->dev, "  [0x%02x]: %08x %08x %08x %08x\n",
			 0x20 + i * 4, regs[i], regs[i+1], regs[i+2], regs[i+3]);
}

static void icp_hfi_latch_regs(struct camss_icp *icp)
{
	struct icp_hfi *hfi = &icp->hfi;

	/* Shared memory */
	writel(hfi->hfi_mem.shmem.dma_addr, icp->csr_base + HFI_REG_SHARED_MEM_PTR);
	writel(hfi->hfi_mem.shmem.size, icp->csr_base + HFI_REG_SHARED_MEM_SIZE);

	/* QTBL - header describing HFI queues */
	writel(hfi->hfi_mem.q_tbl.dma_addr, icp->csr_base + HFI_REG_QTBL_PTR);

	/* QDSS */
	writel(hfi->hfi_mem.qdss.dma_addr, icp->csr_base + HFI_REG_QDSS_IOVA);
	writel(HFI_QDSS_SIZE, icp->csr_base + HFI_REG_QDSS_IOVA_SIZE);

	/* FW Uncached */
	writel(hfi->hfi_mem.fwuncached.dma_addr, icp->csr_base + HFI_REG_FWUNCACHED_IOVA);
	writel(HFI_FWUNCACHED_SIZE, icp->csr_base + HFI_REG_FWUNCACHED_SIZE);

	/* Sec heap */
	writel(hfi->hfi_mem.secheap.dma_addr, icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR);
	writel(HFI_SECHEAP_SIZE, icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE);

	/* SFR buffer */
	writel(hfi->hfi_mem.sfr.dma_addr, icp->csr_base + HFI_REG_SFR_PTR);

	/* IO regions - use standard CAMX values TODO: fix this */
	writel(0x10c00000, icp->csr_base + HFI_REG_IO_REGION_1_IOVA);
	writel(0xcf400000, icp->csr_base + HFI_REG_IO_REGION_1_SIZE);
	writel(0xe0800000, icp->csr_base + HFI_REG_IO_REGION_2_IOVA);
	writel(0x1e700000, icp->csr_base + HFI_REG_IO_REGION_2_SIZE);

}

/* HFI operations */
static const struct icp_hfi_ops hfi_ops = {
	.raise_irq = icp_raise_irq,
};

static int icp_boot(struct camss_icp *icp)
{
	const struct camss_icp_resources *icp_res = icp->icp_res;
	u32 hw_version, data;
	int i, ret;

	/* Power on all domains via runtime PM */
	ret = pm_runtime_resume_and_get(icp->dev);
	if (ret) {
		dev_err(icp->dev, "Failed to power on: %d\n", ret);
		return ret;
	}

	/* Enable clocks */
	ret = clk_bulk_prepare_enable(icp_res->clk_num, icp->clks);
	if (ret) {
		dev_err(icp->dev, "Failed to enable clocks: %d\n", ret);
		return ret;
	}

	for (i = 0; i < icp_res->noc_num; i++) {
		if (icp->icc_path[i]) {
			dev_dbg(icp->dev, "Voting for BW now %s\n", icp_res->noc_names[i]);
			ret = icc_set_bw(icp->icc_path[i], 100000000, 1000000000);
			if (ret) {
				dev_err(icp->dev, "Voting for %s failed\n", icp_res->noc_names[i]);
				return ret;
			}
		}
	}

	/* Verify HW version */
	hw_version = readl(icp->csr_base + ICP_CSR_HW_VERSION);
	dev_info(icp->dev, "HW version: 0x%08x\n", hw_version);

	if (hw_version == 0 || hw_version == 0xffffffff) {
		dev_err(icp->dev, "Invalid HW version\n");
		ret = -EIO;
		goto err_clk;
	}

	/* a previous image that crashed or never came up is still held by TZ */
	ret = qcom_scm_pas_shutdown(icp_res->pas_id);
	dev_dbg(icp->dev, "PAS shutdown before load: %d\n", ret);

	/*
	 * Reset the ICP, BPS and IPE cores: after a firmware crash the ICP
	 * otherwise keeps its old state and the new image never reports ready.
	 */
	ret = reset_control_bulk_assert(ARRAY_SIZE(icp->resets), icp->resets);
	if (!ret) {
		usleep_range(10, 20);
		ret = reset_control_bulk_deassert(ARRAY_SIZE(icp->resets), icp->resets);
	}
	if (ret) {
		dev_err(icp->dev, "core reset failed: %d\n", ret);
		goto err_clk;
	}

	/* Configure interrupts */
	writel(CIRQ_ALL, icp->cirq_base + CIRQ_OB_CLEAR);
	writel(CIRQ_CMD_CLEAR, icp->cirq_base + CIRQ_OB_CMD);
	writel(CIRQ_ICP2HOSTINT | CIRQ_WDT_BITE_WS0, icp->cirq_base + CIRQ_OB_MASK);

	/* Load firmware */
	ret = icp_load_firmware(icp);
	if (ret)
		goto err_clk;

	/* Allocate HFI queues */
	ret = icp_hfi_init_queues(&icp->hfi);
	if (ret)
		goto err_clk;

	/* Start firmware via TrustZone */
	ret = qcom_scm_pas_auth_and_reset(icp_res->pas_id);
	if (ret) {
		dev_err(icp->dev, "TZ auth_and_reset failed: %d\n", ret);
		goto err_hfi;
	}

	/* Wait for firmware to boot */
	usleep_range(5000, 51000);

	/* Program HFI pointers after bootup */
	icp_hfi_latch_regs(icp);

	/*
	 * Boot handshake with ICP firmware.
	 *
	 * After TZ starts the ICP, firmware waits for host to program
	 * GP registers with memory addresses and signal readiness.
	 *
	 * Register assignments:
	 *   GP1 (0x24) = FW_VERSION        - firmware writes its version
	 *   GP2 (0x28) = HOST_ICP_INIT_REQ - host writes 1 ("addresses ready")
	 *   GP3 (0x2C) = ICP_HOST_INIT_RESP - firmware writes 1 ("ready")
	 *   GP4-GP18   = memory addresses   - host programs with IOVAs
	 *
	 * Sequence:
	 *   1. Program GP4-GP18 with memory addresses  [done above]
	 *   2. Write GP2 = 1 (HOST_ICP_INIT_REQUEST)
	 *   3. Poll GP3 until ICP_INIT_RESP_SUCCESS (1)
	 *   4. Read GP1 for firmware version
	 *
	 */

	/* Signal firmware */
	writel(ICP_INIT_REQUEST_SET, icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ);

	wmb();

	/* Wait for firmware to signal ready via GP3 */
	ret = readl_poll_timeout(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP,
				 data, data == ICP_INIT_RESP_SUCCESS,
				 HFI_POLL_DELAY_US, HFI_POLL_TIMEOUT_US);

	if (ret < 0) {
		dev_err(icp->dev, "Firmware ready timeout (GP3=0x%08x)\n",
			readl(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP));
		dev_err(icp->dev, "  GP1 (FW_VERSION)=0x%08x GP2 (INIT_REQ)=0x%08x\n",
			readl(icp->csr_base + HFI_REG_FW_VERSION),
			readl(icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ));
		icp_dump_gp_regs(icp, "FW READY TIMEOUT");
		icp_hfi_dump_sfr(&icp->hfi);
		ret = -ETIMEDOUT;
		goto err_hfi;
	}

	/* the firmware's own start-up clears the interrupt mask set above */
	writel(CIRQ_ALL, icp->cirq_base + CIRQ_OB_CLEAR);
	writel(CIRQ_CMD_CLEAR, icp->cirq_base + CIRQ_OB_CMD);
	writel(CIRQ_ICP2HOSTINT | CIRQ_WDT_BITE_WS0, icp->cirq_base + CIRQ_OB_MASK);

	/* prove the interrupt line: raise our own ICP2HOST through OB_SET */
	reinit_completion(&icp->irq_test_done);
	icp->irq_test = true;
	writel(CIRQ_ICP2HOSTINT, icp->cirq_base + CIRQ_OB_SET);
	writel(CIRQ_CMD_SET, icp->cirq_base + CIRQ_OB_CMD);
	if (!wait_for_completion_timeout(&icp->irq_test_done, msecs_to_jiffies(1000))) {
		icp->irq_test = false;
		dev_err(icp->dev, "interrupt line test failed (CIRQ status 0x%x)\n",
			readl(icp->cirq_base + CIRQ_OB_STATUS));
	} else {
		dev_info(icp->dev, "interrupt line verified\n");
	}

	/* Read firmware version from GP1 */
	icp->hfi.fw_version = readl(icp->csr_base + HFI_REG_FW_VERSION);
	dev_dbg(icp->dev, "Firmware ready! version=0x%08x\n", icp->hfi.fw_version);

	/* Dump CIRQ state after FW init */
	dev_info(icp->dev, "CIRQ after FW init: MASK=0x%x STATUS=0x%x\n",
		 readl(icp->cirq_base + CIRQ_OB_MASK),
		 readl(icp->cirq_base + CIRQ_OB_STATUS));

	/* Dump GP registers to verify what firmware sees */
	dev_info(icp->dev, "GP registers after FW init:\n");
	dev_info(icp->dev, "  GP0 (unused):        0x%08x\n", readl(icp->csr_base + 0x20));
	dev_info(icp->dev, "  GP1 (FW_VERSION):    0x%08x\n", readl(icp->csr_base + HFI_REG_FW_VERSION));
	dev_info(icp->dev, "  GP2 (INIT_REQ):      0x%08x\n", readl(icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ));
	dev_info(icp->dev, "  GP3 (INIT_RESP):     0x%08x\n", readl(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP));
	dev_info(icp->dev, "  GP4 (SHMEM_PTR):     0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SHARED_MEM_PTR), (u32)icp->hfi.hfi_mem.shmem.dma_addr);
	dev_info(icp->dev, "  GP5 (SHMEM_SIZE):    0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SHARED_MEM_SIZE), (u32)icp->hfi.hfi_mem.shmem.size);
	dev_info(icp->dev, "  GP6 (QTBL_PTR):      0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_QTBL_PTR), (u32)icp->hfi.hfi_mem.q_tbl.dma_addr);
	dev_info(icp->dev, "  GP7 (SECHEAP_PTR):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR), (u32)icp->hfi.hfi_mem.secheap.dma_addr);
	dev_info(icp->dev, "  GP8 (SECHEAP_SIZE):  0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE), HFI_SECHEAP_SIZE);
	dev_info(icp->dev, "  GP9 (STATUS):        0x%08x\n", readl(icp->csr_base + HFI_REG_RESERVED));
	dev_info(icp->dev, "  GP10 (SFR_PTR):      0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SFR_PTR), (u32)icp->hfi.hfi_mem.sfr.dma_addr);
	dev_info(icp->dev, "  GP11 (QDSS_IOVA):    0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_QDSS_IOVA), (u32)icp->hfi.hfi_mem.qdss.dma_addr);
	dev_info(icp->dev, "  GP12 (QDSS_SIZE):    0x%08x\n", readl(icp->csr_base + HFI_REG_QDSS_IOVA_SIZE));
	dev_info(icp->dev, "  GP17 (FWUNCACHED):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_FWUNCACHED_IOVA), (u32)icp->hfi.hfi_mem.fwuncached.dma_addr);
	dev_info(icp->dev, "  GP18 (FWUNC_SIZE):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_FWUNCACHED_SIZE), HFI_FWUNCACHED_SIZE);

	return 0;
err_hfi:
	qcom_scm_pas_shutdown(icp_res->pas_id);
	icp_hfi_deinit_queues(&icp->hfi);
err_clk:
	for (i = 0; i < icp_res->noc_num; i++) {
		if (icp->icc_path[i])
			icc_set_bw(icp->icc_path[i], 0, 0);
	}
	clk_bulk_disable_unprepare(icp_res->clk_num, icp->clks);
	return ret;
}

/*
 * Lab interface for bringing up the IPE/BPS sessions from userspace.
 *
 *  cmd     write: u32 want-message-type, then a complete HFI packet;
 *          the packet goes to the command queue and the write returns once
 *          a message of that type arrived. read: that message.
 *  alloc   write: a size in bytes; allocates the next DMA buffer.
 *          read: "index iova size" per buffer.
 *  bufN    the buffer's contents, read/write.
 */
#define ICP_LAB_MAX_PKT		SZ_4K

static ssize_t icp_lab_cmd_write(struct file *f, const char __user *ubuf,
				 size_t len, loff_t *ppos)
{
	struct camss_icp *icp = f->private_data;
	u32 *pkt;
	int ret;

	if (len < 12 || len > ICP_LAB_MAX_PKT + 4)
		return -EINVAL;
	pkt = memdup_user(ubuf, len);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);
	if (pkt[1] != len - 4) {	/* packet size field must match */
		kfree(pkt);
		return -EINVAL;
	}

	mutex_lock(&icp->lab_lock);
	ret = icp_hfi_send_raw(&icp->hfi, &pkt[1], pkt[0], icp->lab_resp, ICP_LAB_MAX_PKT);
	icp->lab_resp_len = ret > 0 ? ret : 0;
	mutex_unlock(&icp->lab_lock);
	kfree(pkt);
	icp_hfi_flush_debug_queue(&icp->hfi);
	return ret < 0 ? ret : len;
}

static ssize_t icp_lab_cmd_read(struct file *f, char __user *ubuf,
				size_t len, loff_t *ppos)
{
	struct camss_icp *icp = f->private_data;
	ssize_t ret;

	mutex_lock(&icp->lab_lock);
	ret = simple_read_from_buffer(ubuf, len, ppos, icp->lab_resp, icp->lab_resp_len);
	mutex_unlock(&icp->lab_lock);
	return ret;
}

static const struct file_operations icp_lab_cmd_fops = {
	.open = simple_open,
	.read = icp_lab_cmd_read,
	.write = icp_lab_cmd_write,
};

static ssize_t icp_lab_buf_rw(struct file *f, char __user *ubuf, size_t len,
			      loff_t *ppos, bool write)
{
	struct icp_lab_buf *b = f->private_data;

	if (write)
		return simple_write_to_buffer(b->vaddr, b->size, ppos,
					      (const char __user *)ubuf, len);
	return simple_read_from_buffer(ubuf, len, ppos, b->vaddr, b->size);
}

static ssize_t icp_lab_buf_read(struct file *f, char __user *u, size_t l, loff_t *p)
{
	return icp_lab_buf_rw(f, u, l, p, false);
}

static ssize_t icp_lab_buf_write(struct file *f, const char __user *u, size_t l, loff_t *p)
{
	return icp_lab_buf_rw(f, (char __user *)u, l, p, true);
}

static const struct file_operations icp_lab_buf_fops = {
	.open = simple_open,
	.read = icp_lab_buf_read,
	.write = icp_lab_buf_write,
	.llseek = default_llseek,
};

static ssize_t icp_lab_alloc_write(struct file *f, const char __user *ubuf,
				   size_t len, loff_t *ppos)
{
	struct camss_icp *icp = f->private_data;
	struct icp_lab_buf *b = NULL;
	char name[8];
	unsigned long size;
	int i, ret;

	ret = kstrtoul_from_user(ubuf, len, 0, &size);
	if (ret)
		return ret;
	if (!size || size > SZ_64M)
		return -EINVAL;

	mutex_lock(&icp->lab_lock);
	for (i = 0; i < ARRAY_SIZE(icp->lab_buf); i++) {
		if (!icp->lab_buf[i].vaddr) {
			b = &icp->lab_buf[i];
			break;
		}
	}
	if (b) {
		size = PAGE_ALIGN(size);
		b->vaddr = dma_alloc_attrs(icp->dev, size, &b->iova, GFP_KERNEL,
					   DMA_ATTR_FORCE_CONTIGUOUS);
		if (b->vaddr) {
			struct iommu_domain *own = iommu_get_domain_for_dev(icp->dev);
			phys_addr_t phys = own ? iommu_iova_to_phys(own, b->iova) : 0;
			int m;

			b->size = size;
			memset(b->vaddr, 0, size);
			for (m = 0; m < ARRAY_SIZE(icp->lab_mirror); m++) {
				if (!icp->lab_mirror[m] || !phys)
					continue;
				if (iommu_map(icp->lab_mirror[m], b->iova, phys, size,
					      IOMMU_READ | IOMMU_WRITE, GFP_KERNEL))
					dev_err(icp->dev, "lab: mirror %d of %pad failed\n", m, &b->iova);
			}
			snprintf(name, sizeof(name), "buf%d", i);
			debugfs_create_file_size(name, 0600, icp->dbg, b, &icp_lab_buf_fops, size);
		}
	}
	mutex_unlock(&icp->lab_lock);
	if (!b)
		return -ENOSPC;
	return b->vaddr ? len : -ENOMEM;
}

static ssize_t icp_lab_alloc_read(struct file *f, char __user *ubuf,
				  size_t len, loff_t *ppos)
{
	struct camss_icp *icp = f->private_data;
	char *txt;
	int i, n = 0;
	ssize_t ret;

	txt = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!txt)
		return -ENOMEM;
	mutex_lock(&icp->lab_lock);
	for (i = 0; i < ARRAY_SIZE(icp->lab_buf); i++)
		if (icp->lab_buf[i].vaddr)
			n += scnprintf(txt + n, PAGE_SIZE - n, "%d 0x%08llx %zu\n", i,
				       (u64)icp->lab_buf[i].iova, icp->lab_buf[i].size);
	mutex_unlock(&icp->lab_lock);
	ret = simple_read_from_buffer(ubuf, len, ppos, txt, n);
	kfree(txt);
	return ret;
}

static const struct file_operations icp_lab_alloc_fops = {
	.open = simple_open,
	.read = icp_lab_alloc_read,
	.write = icp_lab_alloc_write,
};

static struct iommu_domain *icp_lab_peer_domain(struct camss_icp *icp, const char *compat)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, compat);
	struct platform_device *pdev = np ? of_find_device_by_node(np) : NULL;
	struct iommu_domain *dom = pdev ? iommu_get_domain_for_dev(&pdev->dev) : NULL;

	dev_info(icp->dev, "lab: %s domain %p (type %d)\n", compat, dom, dom ? dom->type : -1);
	if (pdev)
		put_device(&pdev->dev);
	of_node_put(np);
	return dom;
}

static void icp_lab_init(struct camss_icp *icp)
{
	mutex_init(&icp->lab_lock);
	icp->lab_mirror[0] = icp_lab_peer_domain(icp, "qcom,x1e80100-camss-ipe");
	icp->lab_mirror[1] = icp_lab_peer_domain(icp, "qcom,x1e80100-camss-bps");
	if (icp->lab_mirror[1] == icp->lab_mirror[0])
		icp->lab_mirror[1] = NULL;	/* one group for both: map once */
	icp->lab_resp = devm_kzalloc(icp->dev, ICP_LAB_MAX_PKT, GFP_KERNEL);
	if (!icp->lab_resp)
		return;
	icp->dbg = debugfs_create_dir("camss-icp", NULL);
	debugfs_create_file("cmd", 0600, icp->dbg, icp, &icp_lab_cmd_fops);
	debugfs_create_file("alloc", 0600, icp->dbg, icp, &icp_lab_alloc_fops);
}

static void icp_lab_exit(struct camss_icp *icp)
{
	int i;

	debugfs_remove_recursive(icp->dbg);
	for (i = 0; i < ARRAY_SIZE(icp->lab_buf); i++) {
		struct icp_lab_buf *b = &icp->lab_buf[i];
		int m;

		if (!b->vaddr)
			continue;
		for (m = 0; m < ARRAY_SIZE(icp->lab_mirror); m++)
			if (icp->lab_mirror[m])
				iommu_unmap(icp->lab_mirror[m], b->iova, b->size);
		dma_free_attrs(icp->dev, b->size, b->vaddr, b->iova, DMA_ATTR_FORCE_CONTIGUOUS);
	}
}

static int camss_icp_probe(struct platform_device *pdev)
{
	struct dev_pm_domain_attach_data pd_data = { .pd_flags = PD_FLAG_DEV_LINK_ON };
	const struct camss_icp_resources *icp_res;
	struct camss_icp *icp;
	int ret, i;

	icp_res = of_device_get_match_data(&pdev->dev);
	if (!icp_res)
		return -EINVAL;

	icp = devm_kzalloc(&pdev->dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;

	icp->dev = &pdev->dev;
	icp->icp_res = icp_res;
	icp->hfi.res = &icp_res->hfi_res;
	icp->hfi.ops = &hfi_ops;
	icp->hfi.dev = icp->dev;
	icp->hfi.priv = icp;
	init_completion(&icp->irq_test_done);

	platform_set_drvdata(pdev, icp);

	icp->csr_base = devm_platform_ioremap_resource_byname(pdev, "csr");
	if (IS_ERR(icp->csr_base))
		return PTR_ERR(icp->csr_base);

	icp->cirq_base = devm_platform_ioremap_resource_byname(pdev, "cirq");
	if (IS_ERR(icp->cirq_base))
		return PTR_ERR(icp->cirq_base);

	icp->irq = platform_get_irq(pdev, 0);
	if (icp->irq < 0)
		return icp->irq;

	/* Reserved memory for HFI */
	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 1);
	if (ret && ret != -ENODEV) {
		dev_err(&pdev->dev, "Failed to init reserved memory: %d\n", ret);
		return ret;
	}

	ret = dev_pm_domain_attach_list(&pdev->dev, &pd_data, &icp->pd_list);
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "Failed to attach power domains: %d\n", ret);
		return ret;
	}

	for (i = 0; i < icp_res->clk_num; i++) {
		dev_info(&pdev->dev, "clk=%s\n", icp_res->clk_names[i]);
		icp->clks[i].id = icp_res->clk_names[i];
	}

	ret = devm_clk_bulk_get(&pdev->dev, icp_res->clk_num, icp->clks);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get clocks: %d\n", ret);
		goto err_pd;
	}

	icp->resets[0].id = "icp";
	icp->resets[1].id = "bps";
	icp->resets[2].id = "ipe";
	ret = devm_reset_control_bulk_get_exclusive(&pdev->dev, ARRAY_SIZE(icp->resets),
						    icp->resets);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get resets: %d\n", ret);
		goto err_pd;
	}

	for (i = 0; i < icp_res->noc_num; i++) {
		icp->icc_path[i] = devm_of_icc_get(&pdev->dev, icp_res->noc_names[i]);
		if (IS_ERR(icp->icc_path[i])) {
			if (PTR_ERR(icp->icc_path[i]) != -ENODATA) {
				ret = PTR_ERR(icp->icc_path[i]);
				goto err_pd;
			}
			icp->icc_path[i] = NULL;
		}
	};

	ret = devm_request_threaded_irq(&pdev->dev, icp->irq, camss_icp_isr,
					camss_icp_isr_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"camss-icp", icp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		goto err_pd;
	}

	pm_runtime_enable(&pdev->dev);

	ret = icp_boot(icp);
	if (ret)
		goto err_pd;

	ret = icp_hfi_core_init(&icp->hfi);
	if (ret)
		goto err_pd;

	icp_lab_init(icp);
	return 0;

err_pd:
	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);

	return ret;
}

static void camss_icp_remove(struct platform_device *pdev)
{
	struct camss_icp *icp = platform_get_drvdata(pdev);
	const struct camss_icp_resources *icp_res;
	int i;

	icp_res = icp->icp_res;
	icp_lab_exit(icp);
	qcom_scm_pas_shutdown(icp_res->pas_id);

	icp_hfi_deinit_queues(&icp->hfi);

	for (i = 0; i < icp_res->noc_num; i++) {
		if (icp->icc_path[i])
			icc_set_bw(icp->icc_path[i], 0, 0);
	}
	clk_bulk_disable_unprepare(icp_res->clk_num, icp->clks);

	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);

}

static const char * const x1e80100_clk_names [] = {
	"ahb", "core", "debug_xo",
	"gcc_hf_axi", "gcc_sf_axi",
	"cpas_ahb", "core_ahb", "cpas_fast_ahb",
	"camnoc_axi_rt", "camnoc_axi_nrt",
	"bps_ahb", "bps_fast_ahb", "bps", "cpas_bps",
	"ipe_ahb", "ipe_nps_fast_ahb", "ipe_pps_fast_ahb",
	"ipe_nps", "ipe_pps", "cpas_ipe",
};

static const char * const x1e80100_noc_names [] = {
	"ahb",
	"hf_0",
	"sf_0",
	"sf_icp"
};

static const struct camss_icp_resources x1e80100_icp_res = {
	.pas_id = 33,
	.clk_names = x1e80100_clk_names,
	.clk_num = ARRAY_SIZE(x1e80100_clk_names),
	.noc_names = x1e80100_noc_names,
	.noc_num = ARRAY_SIZE(x1e80100_noc_names),
	.hfi_res = {
		/* shared region up to the firmware-uncached one at 0x10000000 */
		.shmem_size = 0x10000000 - 0x07400000,
		.qdss_size = SZ_1M,
		.fwuncached_size = 7 * SZ_1M,
		/*
		 * Carve sub-regions from FwUncached:
		 * Over-allocate as CamX does for now.
		 *   +0x000000: SecHeap (1MB)
		 *   +0x100000: QTBL (1MB)
		 *   +0x200000: CMD_Q (1MB)
		 *   +0x300000: MSG_Q (1MB)
		 *   +0x400000: DBG_Q (1MB)
		 *   +0x500000: SFR (1MB alloc 4KB used)
		 */
		.secheap_size = SZ_1M,
		.q_tbl_size = SZ_1M,
		.qdata_size[HFI_Q_CMD_TYPE] = SZ_1M,
		.qdata_size[HFI_Q_MSG_TYPE] = SZ_1M,
		.qdata_size[HFI_Q_DBG_TYPE] = SZ_1M,
		.sfr_size = SZ_1M,
	},
};

static const struct of_device_id camss_icp_of_match[] = {
	{ .compatible = "qcom,x1e80100-camss-icp", .data = &x1e80100_icp_res},
	{ }
};
MODULE_DEVICE_TABLE(of, camss_icp_of_match);

static struct platform_driver camss_icp_driver = {
	.probe = camss_icp_probe,
	.remove = camss_icp_remove,
	.driver = {
		.name = "camss-icp",
		.of_match_table = camss_icp_of_match,
	},
};

module_platform_driver(camss_icp_driver);

MODULE_DESCRIPTION("Qualcomm ICP driver");
MODULE_LICENSE("GPL");
