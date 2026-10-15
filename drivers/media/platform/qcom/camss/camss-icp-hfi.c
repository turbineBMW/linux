// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm CAMSS ICP Host Firmware Interface: memory, queues and the
 * system commands (init, ping, debug configuration).
 *
 * The queue table, queue header layout and the interrupt-mode queue setup
 * follow the HFI the ICP firmware implements (as in Qualcomm's camera
 * kernel driver, cam_icp/hfi.c): three ring buffers of 32-bit words, the
 * host writes the command queue and reads the message and debug queues.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "camss-icp.h"
#include "camss-icp-hfi.h"

#define HFI_QHDR_ACTIVE			1
#define HFI_QHDR_SET			1
#define HFI_QHDR_SET_WM			1024
#define HFI_VAR_SIZE_PKT		0
#define HFI_NUM_ACTIVE_QUEUES		3
#define HFI_NUM_QUEUES			4

#define HFI_CMD_Q_BYTES			8192
#define HFI_MSG_Q_BYTES			8192
#define HFI_DBG_Q_BYTES			102400

#define HFI_CMD_TIMEOUT_MS		5000
#define HFI_PING_COOKIE			0xdeadbeefcafebabeULL

/* largest message we copy out of a queue */
#define HFI_MSG_MAX_BYTES		SZ_4K

static u32 hfi_q_bytes[HFI_Q_MAX] = {
	[HFI_Q_CMD_TYPE] = HFI_CMD_Q_BYTES,
	[HFI_Q_MSG_TYPE] = HFI_MSG_Q_BYTES,
	[HFI_Q_DBG_TYPE] = HFI_DBG_Q_BYTES,
};

/*
 * The firmware expects its memory at fixed addresses below the IO windows it
 * uses for image buffers (as in the camera SMMU layout of the downstream
 * driver): shared 0x07400000, firmware-uncached (secondary heap, queues)
 * 0x10000000, QDSS 0x10b00000. Allocate contiguous memory and map it there
 * in the ICP's domain as well.
 */
#define HFI_IOVA_SHMEM			0x07400000
#define HFI_IOVA_FWUNCACHED		0x10000000
#define HFI_IOVA_QDSS			0x10b00000

static int hfi_alloc(struct icp_hfi *hfi, struct icp_hfi_mem_region *r, size_t size,
		     dma_addr_t fixed)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(hfi->dev);
	phys_addr_t phys;
	int ret;

	r->vaddr = dma_alloc_attrs(hfi->dev, size, &r->alloc_addr, GFP_KERNEL,
				   DMA_ATTR_FORCE_CONTIGUOUS);
	if (!r->vaddr)
		return -ENOMEM;
	r->size = size;
	r->dma_addr = r->alloc_addr;
	memset(r->vaddr, 0, size);

	if (!fixed || !dom)
		return 0;
	phys = iommu_iova_to_phys(dom, r->alloc_addr);
	ret = phys ? iommu_map(dom, fixed, phys, size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL)
		   : -EFAULT;
	if (ret) {
		dev_err(hfi->dev, "HFI: cannot map %zu bytes at %pad: %d\n", size, &fixed, ret);
		dma_free_attrs(hfi->dev, size, r->vaddr, r->alloc_addr, DMA_ATTR_FORCE_CONTIGUOUS);
		r->vaddr = NULL;
		return ret;
	}
	r->dma_addr = fixed;
	return 0;
}

static void hfi_free(struct icp_hfi *hfi, struct icp_hfi_mem_region *r)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(hfi->dev);

	if (!r->vaddr)
		return;
	if (r->dma_addr != r->alloc_addr && dom)
		iommu_unmap(dom, r->dma_addr, r->size);
	dma_free_attrs(hfi->dev, r->size, r->vaddr, r->alloc_addr, DMA_ATTR_FORCE_CONTIGUOUS);
	r->vaddr = NULL;
}

static void hfi_carve(struct icp_hfi_mem_region *parent, size_t *off,
		      struct icp_hfi_mem_region *r, size_t size)
{
	r->vaddr = parent->vaddr + *off;
	r->dma_addr = parent->dma_addr + *off;
	r->size = size;
	*off += size;
}

int icp_hfi_init_queues(struct icp_hfi *hfi)
{
	const struct hfi_resources *res = hfi->res;
	struct icp_hfi_mem *m = &hfi->hfi_mem;
	struct hfi_q_tbl_hdr *tbl;
	struct hfi_sfr *sfr;
	size_t off = 0;
	int i, ret;

	/* the firmware takes 32-bit addresses */
	ret = dma_set_mask_and_coherent(hfi->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	mutex_init(&hfi->cmd_lock);
	mutex_init(&hfi->msg_lock);
	init_completion(&hfi->cmd_complete);

	ret = hfi_alloc(hfi, &m->shmem, res->shmem_size, HFI_IOVA_SHMEM);
	if (ret)
		goto err;
	ret = hfi_alloc(hfi, &m->qdss, res->qdss_size, HFI_IOVA_QDSS);
	if (ret)
		goto err;
	ret = hfi_alloc(hfi, &m->fwuncached, res->fwuncached_size, HFI_IOVA_FWUNCACHED);
	if (ret)
		goto err;

	hfi_carve(&m->fwuncached, &off, &m->secheap, res->secheap_size);
	hfi_carve(&m->fwuncached, &off, &m->q_tbl, res->q_tbl_size);
	for (i = 0; i < HFI_Q_MAX; i++)
		hfi_carve(&m->fwuncached, &off, &m->q_data[i], res->qdata_size[i]);
	hfi_carve(&m->fwuncached, &off, &m->sfr, res->sfr_size);
	if (off > m->fwuncached.size) {
		ret = -EINVAL;
		goto err;
	}

	tbl = m->q_tbl.vaddr;
	tbl->version = HFI_QUEUE_TABLE_VERSION;
	tbl->size = sizeof(*tbl) + HFI_NUM_QUEUES * sizeof(struct hfi_q_hdr);
	tbl->q_hdr0_offset = sizeof(*tbl);
	tbl->q_hdr_size = sizeof(struct hfi_q_hdr);
	tbl->num_queues = HFI_NUM_ACTIVE_QUEUES;
	tbl->num_active_queues = HFI_NUM_ACTIVE_QUEUES;

	for (i = 0; i < HFI_Q_MAX; i++) {
		struct hfi_q_hdr *q = &tbl->q_hdr[i];

		q->status = HFI_QHDR_ACTIVE;
		q->start_addr = m->q_data[i].dma_addr;
		q->type = i;
		q->q_size = hfi_q_bytes[i] >> HFI_BYTE_WORD_SHIFT;
		q->pkt_size = HFI_VAR_SIZE_PKT;
		q->pkt_drop_cnt = 0;
		q->read_idx = 0;
		q->write_idx = 0;
		/* interrupt mode, as the reference driver runs it */
		q->rx_wm = HFI_QHDR_SET;
		q->tx_wm = i == HFI_Q_DBG_TYPE ? HFI_QHDR_SET_WM : HFI_QHDR_SET;
		q->rx_req = i == HFI_Q_DBG_TYPE ? 0 : HFI_QHDR_SET;
		q->tx_req = 0;
		q->rx_irq_status = 0;
		q->tx_irq_status = 0;
	}
	hfi->q_tbl = tbl;

	sfr = m->sfr.vaddr;
	sfr->size = HFI_SFR_LOG_SIZE;
	hfi->sfr = sfr;

	/* tables in place before the firmware is told where they are */
	wmb();

	dev_info(hfi->dev, "HFI: qtbl %pad shmem %pad fwuncached %pad sfr %pad\n",
		 &m->q_tbl.dma_addr, &m->shmem.dma_addr,
		 &m->fwuncached.dma_addr, &m->sfr.dma_addr);
	return 0;

err:
	icp_hfi_deinit_queues(hfi);
	return ret;
}

void icp_hfi_deinit_queues(struct icp_hfi *hfi)
{
	struct icp_hfi_mem *m = &hfi->hfi_mem;

	hfi->ready = false;
	hfi->q_tbl = NULL;
	hfi->sfr = NULL;
	hfi_free(hfi, &m->fwuncached);
	hfi_free(hfi, &m->qdss);
	hfi_free(hfi, &m->shmem);
}

static int hfi_write_cmd(struct icp_hfi *hfi, const void *pkt)
{
	struct hfi_q_hdr *q = &hfi->q_tbl->q_hdr[HFI_Q_CMD_TYPE];
	u32 *ring = hfi->hfi_mem.q_data[HFI_Q_CMD_TYPE].vaddr;
	u32 words = hfi_pkt_size((void *)pkt) >> HFI_BYTE_WORD_SHIFT;
	u32 ri = READ_ONCE(q->read_idx), wi = READ_ONCE(q->write_idx);
	u32 qsize = q->q_size, space, new_wi, first;

	if (!words)
		return -EINVAL;

	space = wi >= ri ? qsize - (wi - ri) : ri - wi;
	if (space <= words)
		return -ENOSPC;

	new_wi = wi + words;
	if (new_wi < qsize) {
		memcpy(ring + wi, pkt, words << HFI_BYTE_WORD_SHIFT);
	} else {
		new_wi -= qsize;
		first = words - new_wi;
		memcpy(ring + wi, pkt, first << HFI_BYTE_WORD_SHIFT);
		memcpy(ring, (const u32 *)pkt + first, new_wi << HFI_BYTE_WORD_SHIFT);
	}

	/* packet before index, index before the doorbell */
	wmb();
	WRITE_ONCE(q->write_idx, new_wi);
	wmb();
	hfi->ops->raise_irq(hfi->priv);
	return 0;
}

/* Copy one packet out of a firmware-to-host queue; returns its size in bytes. */
static int hfi_read_pkt(struct icp_hfi *hfi, int qid, u32 *buf, u32 buf_bytes)
{
	struct hfi_q_hdr *q = &hfi->q_tbl->q_hdr[qid];
	u32 *ring = hfi->hfi_mem.q_data[qid].vaddr;
	u32 ri = READ_ONCE(q->read_idx), wi = READ_ONCE(q->write_idx);
	u32 qsize = q->q_size, words, avail, first;

	if (ri == wi)
		return 0;

	rmb();
	avail = wi > ri ? wi - ri : qsize - ri + wi;
	words = ring[ri] >> HFI_BYTE_WORD_SHIFT;	/* first word = size */
	if (!words || words > avail || (words << HFI_BYTE_WORD_SHIFT) > buf_bytes) {
		dev_err(hfi->dev, "HFI q%d: bad packet (%u words, %u queued), dropping queue\n",
			qid, words, avail);
		WRITE_ONCE(q->read_idx, wi);
		return -EIO;
	}

	if (ri + words <= qsize) {
		memcpy(buf, ring + ri, words << HFI_BYTE_WORD_SHIFT);
	} else {
		first = qsize - ri;
		memcpy(buf, ring + ri, first << HFI_BYTE_WORD_SHIFT);
		memcpy(buf + first, ring, (words - first) << HFI_BYTE_WORD_SHIFT);
	}

	mb();
	WRITE_ONCE(q->read_idx, (ri + words) % qsize);
	return words << HFI_BYTE_WORD_SHIFT;
}

void icp_hfi_flush_debug_queue(struct icp_hfi *hfi)
{
	u32 *buf;
	int len;

	if (!hfi->q_tbl)
		return;

	buf = kmalloc(HFI_MSG_MAX_BYTES, GFP_KERNEL);
	if (!buf)
		return;

	mutex_lock(&hfi->msg_lock);
	while ((len = hfi_read_pkt(hfi, HFI_Q_DBG_TYPE, buf, HFI_MSG_MAX_BYTES)) > 0) {
		struct hfi_msg_debug *dbg = (void *)buf;

		if (hfi_pkt_type(buf) == HFI_MSG_SYS_DEBUG &&
		    len > sizeof(*dbg)) {
			int n = min_t(u32, dbg->msg_size, len - sizeof(*dbg));

			dev_info(hfi->dev, "fw: %.*s\n", n, dbg->msg_data);
		} else {
			dev_dbg(hfi->dev, "fw debug packet type 0x%x size %d\n",
				hfi_pkt_type(buf), len);
		}
	}
	mutex_unlock(&hfi->msg_lock);
	kfree(buf);
}

bool icp_hfi_process_msg_queue(struct icp_hfi *hfi)
{
	bool any = false;
	u32 *buf;
	int len;

	if (!hfi->q_tbl)
		return false;

	buf = kmalloc(HFI_MSG_MAX_BYTES, GFP_KERNEL);
	if (!buf)
		return false;

	mutex_lock(&hfi->msg_lock);
	while ((len = hfi_read_pkt(hfi, HFI_Q_MSG_TYPE, buf, HFI_MSG_MAX_BYTES)) > 0) {
		u32 type = hfi_pkt_type(buf);

		any = true;
		switch (type) {
		case HFI_MSG_SYS_INIT_DONE: {
			struct hfi_msg_init_done *m = (void *)buf;
			u32 n = min_t(u32, m->prop_num, HFI_MAX_PROPS);

			hfi->api_version = m->error;
			hfi->prop_num = n;
			memcpy(hfi->properties, m->prop_data, n * sizeof(u32));
			dev_info(hfi->dev, "HFI: SYS_INIT_DONE error %u props %u\n",
				 m->error, m->prop_num);
			break;
		}
		case HFI_MSG_SYS_PING_ACK: {
			struct hfi_msg_ping_ack *m = (void *)buf;

			dev_info(hfi->dev, "HFI: ping ack 0x%016llx%s\n", m->user_data,
				 m->user_data == HFI_PING_COOKIE ? " (match)" : " (MISMATCH)");
			break;
		}
		case HFI_MSG_EVENT_NOTIFY: {
			struct hfi_msg_event *m = (void *)buf;

			dev_err(hfi->dev, "HFI: event %u session 0x%x data 0x%x 0x%x\n",
				m->event_id, m->session_id, m->data1, m->data2);
			icp_hfi_dump_sfr(hfi);
			break;
		}
		default:
			dev_info(hfi->dev, "HFI: message 0x%x size %d\n", type, len);
			break;
		}

		hfi->last_msg_len = min_t(int, len, sizeof(hfi->last_msg));
		memcpy(hfi->last_msg, buf, hfi->last_msg_len);
		smp_wmb();
		WRITE_ONCE(hfi->last_msg_type, type);
		complete(&hfi->cmd_complete);
	}
	mutex_unlock(&hfi->msg_lock);
	kfree(buf);
	return any;
}

/*
 * Send a command and wait for the message of the expected type. The ICP's
 * interrupt reaches us through the threaded handler; poll the queue as
 * well so a missing interrupt shows up as a log line, not a hang.
 */
static int hfi_cmd_wait(struct icp_hfi *hfi, const void *pkt, u32 want)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(HFI_CMD_TIMEOUT_MS);
	bool irq_seen = false;
	int ret;

	mutex_lock(&hfi->cmd_lock);
	reinit_completion(&hfi->cmd_complete);
	WRITE_ONCE(hfi->last_msg_type, 0);

	ret = hfi_write_cmd(hfi, pkt);
	if (ret)
		goto out;

	while (READ_ONCE(hfi->last_msg_type) != want) {
		if (wait_for_completion_timeout(&hfi->cmd_complete,
						msecs_to_jiffies(20))) {
			irq_seen = true;
			reinit_completion(&hfi->cmd_complete);
			continue;
		}
		if (icp_hfi_process_msg_queue(hfi) && !irq_seen)
			dev_warn_once(hfi->dev, "HFI: message arrived without an interrupt\n");
		icp_hfi_flush_debug_queue(hfi);
		if (time_after(jiffies, deadline)) {
			struct hfi_q_hdr *c = &hfi->q_tbl->q_hdr[HFI_Q_CMD_TYPE];
			struct hfi_q_hdr *m = &hfi->q_tbl->q_hdr[HFI_Q_MSG_TYPE];

			dev_err(hfi->dev, "HFI: no 0x%x for 0x%x (cmd r%u w%u, msg r%u w%u)\n",
				want, hfi_pkt_type((void *)pkt), c->read_idx, c->write_idx,
				m->read_idx, m->write_idx);
			icp_hfi_dump_sfr(hfi);
			ret = -ETIMEDOUT;
			goto out;
		}
	}
out:
	mutex_unlock(&hfi->cmd_lock);
	return ret;
}

int icp_hfi_send_raw(struct icp_hfi *hfi, const void *pkt, u32 want,
		     void *resp, u32 resp_size)
{
	int ret;

	if (!hfi->ready)
		return -ENODEV;
	ret = hfi_cmd_wait(hfi, pkt, want);
	if (ret)
		return ret;
	smp_rmb();
	ret = min(hfi->last_msg_len, resp_size);
	memcpy(resp, hfi->last_msg, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(icp_hfi_send_raw);

static int hfi_set_debug(struct icp_hfi *hfi, u32 level)
{
	struct hfi_msg_debug_level pkt = {
		.hdr.size = sizeof(pkt),
		.hdr.type = HFI_CMD_SYS_SET_PROPERTY,
		.num_prop = 1,
		.prop_id = HFI_PROP_SYS_DEBUG_CFG,
		.cfg.debug_config = level,
		.cfg.debug_mode = HFI_DEBUG_MODE_QUEUE,
	};
	int ret;

	mutex_lock(&hfi->cmd_lock);
	ret = hfi_write_cmd(hfi, &pkt);
	mutex_unlock(&hfi->cmd_lock);
	return ret;
}

int icp_hfi_core_init(struct icp_hfi *hfi)
{
	struct hfi_cmd_sys_init init = {
		.hdr.size = sizeof(init),
		.hdr.type = HFI_CMD_SYS_INIT,
	};
	struct hfi_cmd_ping ping = {
		.hdr.size = sizeof(ping),
		.hdr.type = HFI_CMD_SYS_PING,
		.user_data = HFI_PING_COOKIE,
	};
	struct hfi_msg_ping_ack *ack = (void *)hfi->last_msg;
	int ret;

	ret = hfi_cmd_wait(hfi, &init, HFI_MSG_SYS_INIT_DONE);
	if (ret)
		return ret;

	ret = hfi_set_debug(hfi, HFI_DEBUG_MSG_HIGH | HFI_DEBUG_MSG_ERROR |
			    HFI_DEBUG_MSG_FATAL);
	if (ret)
		dev_warn(hfi->dev, "HFI: debug configuration not sent: %d\n", ret);

	ret = hfi_cmd_wait(hfi, &ping, HFI_MSG_SYS_PING_ACK);
	if (ret)
		return ret;
	if (ack->user_data != HFI_PING_COOKIE)
		return -EPROTO;

	hfi->ready = true;
	dev_info(hfi->dev, "HFI ready: firmware 0x%08x, ping 0x%016llx\n",
		 hfi->fw_version, ack->user_data);
	return 0;
}

void icp_hfi_dump_sfr(struct icp_hfi *hfi)
{
	struct hfi_sfr *sfr = hfi->sfr;

	if (!sfr || !sfr->msg[0])
		return;
	dev_err(hfi->dev, "ICP failure reason: %.*s\n",
		(int)min_t(u32, sfr->size, HFI_SFR_LOG_SIZE), sfr->msg);
}
