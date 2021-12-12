/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(5,6,0)
#include <linux/mod_devicetable.h>
#else
#include "../inc/devicetable.h"
#endif
#include "../inc/mhi.h"
#include "mhi_internal.h"

bool debug = false;

module_param(debug, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
MODULE_PARM_DESC(debug,"enable/disable driver logging");

int debug_level = MHI_MSG_LVL_INFO;
module_param(debug_level, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);
MODULE_PARM_DESC(debug_level,"driver logging level");

#define MHI_WRITE_REG_64(_BASE, _OFFSET, _VAL)                                \
        writeq(_VAL, (volatile void *)((u8 *)_BASE + _OFFSET))

const char * const mhi_ee_str[MHI_EE_MAX] = {
	[MHI_EE_PBL] = "PBL",
	[MHI_EE_SBL] = "SBL",
	[MHI_EE_AMSS] = "AMSS",
	[MHI_EE_RDDM] = "RDDM",
	[MHI_EE_WFW] = "WFW",
	[MHI_EE_PTHRU] = "PASS THRU",
	[MHI_EE_EDL] = "EDL",
	[MHI_EE_DISABLE_TRANSITION] = "DISABLE",
	[MHI_EE_NOT_SUPPORTED] = "NOT SUPPORTED",
};

const char * const mhi_state_tran_str[MHI_ST_TRANSITION_MAX] = {
	[MHI_ST_TRANSITION_PBL] = "PBL",
	[MHI_ST_TRANSITION_READY] = "READY",
	[MHI_ST_TRANSITION_SBL] = "SBL",
	[MHI_ST_TRANSITION_MISSION_MODE] = "MISSION MODE",
};

const char * const mhi_state_str[MHI_STATE_MAX] = {
	[MHI_STATE_RESET] = "RESET",
	[MHI_STATE_READY] = "READY",
	[MHI_STATE_M0] = "M0",
	[MHI_STATE_M1] = "M1",
	[MHI_STATE_M2] = "M2",
	[MHI_STATE_M3] = "M3",
	[MHI_STATE_M3_FAST] = "M3_FAST",
	[MHI_STATE_BHI] = "BHI",
	[MHI_STATE_SYS_ERR] = "SYS_ERR",
};

static const char * const mhi_pm_state_str[] = {
	[MHI_PM_BIT_DISABLE] = "DISABLE",
	[MHI_PM_BIT_POR] = "POR",
	[MHI_PM_BIT_M0] = "M0",
	[MHI_PM_BIT_M2] = "M2",
	[MHI_PM_BIT_M3_ENTER] = "M?->M3",
	[MHI_PM_BIT_M3] = "M3",
	[MHI_PM_BIT_M3_EXIT] = "M3->M0",
	[MHI_PM_BIT_FW_DL_ERR] = "FW DL Error",
	[MHI_PM_BIT_SYS_ERR_DETECT] = "SYS_ERR Detect",
	[MHI_PM_BIT_SYS_ERR_PROCESS] = "SYS_ERR Process",
	[MHI_PM_BIT_SHUTDOWN_PROCESS] = "SHUTDOWN Process",
	[MHI_PM_BIT_LD_ERR_FATAL_DETECT] = "LD or Error Fatal Detect",
};

struct mhi_bus mhi_bus;

/**
 * @brief map power state to a string
 *
 * @param[in ]            state					power state
 */
const char *to_mhi_pm_state_str(enum MHI_PM_STATE state)
{
	int index = find_last_bit((unsigned long *)&state, 32);

	if (index >= ARRAY_SIZE(mhi_pm_state_str))
		return "Invalid State";

	return mhi_pm_state_str[index];
}

/**
 * @brief print bus vote to a buffer
 *
 * @param[in ]            dev 					device
 * @param[in ]            attr					device attribute
 * @param[in,out]         buf					buffer
 */
static ssize_t bus_vote_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			atomic_read(&mhi_dev->bus_vote));
}

/**
 * @brief save bus vote to a buffer
 *
 * @param[in ]            dev 					device
 * @param[in ]            attr					device attribute
 * @param[in ]            buf					buffer
 * @param[in ]            count					buffer size
 */
static ssize_t bus_vote_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	int ret = -EINVAL;

	if (sysfs_streq(buf, "get")) {
		ret = mhi_device_get_sync(mhi_dev, MHI_VOTE_BUS);
	} else if (sysfs_streq(buf, "put")) {
		mhi_device_put(mhi_dev, MHI_VOTE_BUS);
		ret = 0;
	}

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(bus_vote);

/**
 * @brief print device vote to a buffer
 *
 * @param[in ]            dev 				device
 * @param[in ]            attr 				device attribute
 * @param[in,out ]        buf 				buffer
 */
static ssize_t device_vote_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			atomic_read(&mhi_dev->dev_vote));
}

/**
 * @brief save device vote
 *
 * @param[in ]            dev 				device
 * @param[in ]            attr 				device attribute
 * @param[in ]			  buf 				buffer
 * @param[in ]			  count				buffer size
 */
static ssize_t device_vote_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	int ret = -EINVAL;

	if (sysfs_streq(buf, "get")) {
		ret = mhi_device_get_sync(mhi_dev, MHI_VOTE_DEVICE);
	} else if (sysfs_streq(buf, "put")) {
		mhi_device_put(mhi_dev, MHI_VOTE_DEVICE);
		ret = 0;
	}

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(device_vote);

static struct attribute *mhi_vote_attrs[] = {
	&dev_attr_bus_vote.attr,
	&dev_attr_device_vote.attr,
	NULL,
};

static const struct attribute_group mhi_vote_group = {
	.attrs = mhi_vote_attrs,
};

/**
 * @brief create vote sysfs
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
int mhi_create_vote_sysfs(struct mhi_controller *mhi_cntrl)
{
	return sysfs_create_group(&mhi_cntrl->mhi_dev->dev.kobj,
				  &mhi_vote_group);
}

/**
 * @brief destroy vote sysfs
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
void mhi_destroy_vote_sysfs(struct mhi_controller *mhi_cntrl)
{
	struct mhi_device *mhi_dev = mhi_cntrl->mhi_dev;

	sysfs_remove_group(&mhi_dev->dev.kobj, &mhi_vote_group);

	/* relinquish any pending votes for device */
	while (atomic_read(&mhi_dev->dev_vote))
		mhi_device_put(mhi_dev, MHI_VOTE_DEVICE);

	/* remove pending votes for the bus */
	while (atomic_read(&mhi_dev->bus_vote))
		mhi_device_put(mhi_dev, MHI_VOTE_BUS);
}

#if 0
/* MHI protocol require transfer ring to be aligned to ring length */
static int mhi_alloc_aligned_ring(struct mhi_controller *mhi_cntrl,
				  struct mhi_ring *ring,
				  u64 len)
{
    MHI_LOG("ring 0x%p len %lld", ring, len);

	ring->alloc_size = len + (len - 1);
    MHI_LOG("alloc_size %ld", ring->alloc_size);
	ring->pre_aligned = mhi_alloc_coherent(mhi_cntrl, ring->alloc_size,
					       &ring->dma_handle, GFP_KERNEL);
	if (!ring->pre_aligned)
		return -ENOMEM;

    MHI_LOG("pre_aligned 0x%p", ring->pre_aligned);

	ring->iommu_base = (ring->dma_handle + (len - 1)) & ~(len - 1);
    MHI_LOG("dma_handle 0x%llx iommu_base 0x%llx", ring->dma_handle, ring->iommu_base);

	ring->base = ring->pre_aligned + (ring->iommu_base - ring->dma_handle);
    MHI_LOG("base 0x%p", ring->base);

	return 0;
}

void mhi_deinit_free_irq(struct mhi_controller *mhi_cntrl)
{
	int i;
	struct mhi_event *mhi_event = mhi_cntrl->mhi_event;

	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++) {
		if (mhi_event->offload_ev)
			continue;

		free_irq(mhi_cntrl->irq[mhi_event->msi], mhi_event);
	}

	free_irq(mhi_cntrl->irq[0], mhi_cntrl);
}
#endif

void mhi_disable_irq(struct mhi_controller *mhi_cntrl);

/**
 * @brief uninitialize assigned irq
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
void mhi_deinit_free_irq(struct mhi_controller *mhi_cntrl)
{
	int i;
	struct mhi_event *mhi_event = mhi_cntrl->mhi_event;

	if (mhi_cntrl->msi_active) {

		/* disable the irq first */
		mhi_disable_irq(mhi_cntrl);

		/* free irq before vector */
		if (mhi_cntrl->msi_allocated > 1) {
			MHI_LOG("free_irq %d\n", mhi_cntrl->irq[0]);
			free_irq(mhi_cntrl->irq[0], mhi_cntrl);

			for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++)
			{
				if (mhi_event->offload_ev)
					continue;

				MHI_LOG("free_irq %d\n", mhi_cntrl->irq[mhi_event->msi]);
				free_irq(mhi_cntrl->irq[mhi_event->msi], mhi_event);
			}
		}
		else if (mhi_cntrl->msi_allocated == 1) {
			MHI_LOG("free_irq %d\n", mhi_cntrl->irq[0]);
			free_irq(mhi_cntrl->irq[0], mhi_cntrl);
		}

		mhi_cntrl->msi_active = false;
	}
}

/**
 * @brief initialize assigned irq
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
int mhi_init_irq_setup(struct mhi_controller *mhi_cntrl)
{
	int i = 0;
	int ret;
	struct mhi_event *mhi_event = mhi_cntrl->mhi_event;

	MHI_LOG("request_threaded_irq: allocated %d\n", mhi_cntrl->msi_allocated);

	if (mhi_cntrl->msi_allocated > 1) {

		/* for BHI INTVEC msi */
		ret = request_threaded_irq(mhi_cntrl->irq[0], mhi_intvec_handlr,
			mhi_intvec_threaded_handlr,
			IRQF_ONESHOT | IRQF_NO_SUSPEND,
			"mhi", mhi_cntrl);
		if (ret)
			return ret;

		for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++) {
			if (mhi_event->offload_ev)
				continue;

			MHI_LOG("request_irq: irq %d vector %d\n", mhi_event->msi, mhi_cntrl->irq[mhi_event->msi]);

			ret = request_irq(mhi_cntrl->irq[mhi_event->msi],
				mhi_msi_handlr, IRQF_SHARED | IRQF_NO_SUSPEND,
				"mhi", mhi_event);
			if (ret) {
				MHI_ERR("Error requesting irq:%d for ev:%d\n",
					mhi_cntrl->irq[mhi_event->msi], i);
				goto error_request;
			}
		}
	}
	else {
		MHI_LOG("shared interrupt!\n");

		ret = request_irq(mhi_cntrl->irq[0],
			mhi_msi_handlr_all, IRQF_SHARED,  //| IRQF_NO_SUSPEND,
			"mhi", mhi_cntrl);
		if (ret) {
			MHI_ERR("Error requesting irq:%d for ev:%d\n",
				mhi_cntrl->irq[0], 0);
			goto error_request;
		}
	}

	mhi_cntrl->msi_active = true;

	MHI_LOG("request_threaded_irq success");

	return 0;

error_request:
	for (--i, --mhi_event; i >= 0; i--, mhi_event--) {
		if (mhi_event->offload_ev)
			continue;

		free_irq(mhi_cntrl->irq[mhi_event->msi], mhi_event);
	}
	free_irq(mhi_cntrl->irq[0], mhi_cntrl);

	return ret;
}

/**
 * @brief uninitialize device context
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
void mhi_deinit_dev_ctxt(struct mhi_controller *mhi_cntrl)
{
    mhi_deinit_ctrl_seq(mhi_cntrl);
}

#ifdef CONFIG_DEBUG_FS

/**
 * @brief initialize debugfs for mhi states
 *
 * @param[in ]            inode 				inode
 * @param[in ]            fp 					file
 */
static int mhi_init_debugfs_mhi_states_open(struct inode *inode,
					    struct file *fp)
{
	return single_open(fp, mhi_debugfs_mhi_states_show, inode->i_private);
}

/**
 * @brief initialize debugfs for mhi event
 *
 * @param[in ]            inode 				inode
 * @param[in ]            fp 					file
 */
static int mhi_init_debugfs_mhi_event_open(struct inode *inode, struct file *fp)
{
	return single_open(fp, mhi_debugfs_mhi_event_show, inode->i_private);
}

/**
 * @brief initialize debugfs for mhi channel
 *
 * @param[in ]            inode 				inode
 * @param[in ]            fp 					file
 */
static int mhi_init_debugfs_mhi_chan_open(struct inode *inode, struct file *fp)
{
	return single_open(fp, mhi_debugfs_mhi_chan_show, inode->i_private);
}

static const struct file_operations debugfs_state_ops = {
	.open = mhi_init_debugfs_mhi_states_open,
	.release = single_release,
	.read = seq_read,
};

static const struct file_operations debugfs_ev_ops = {
	.open = mhi_init_debugfs_mhi_event_open,
	.release = single_release,
	.read = seq_read,
};

static const struct file_operations debugfs_chan_ops = {
	.open = mhi_init_debugfs_mhi_chan_open,
	.release = single_release,
	.read = seq_read,
};

DEFINE_SIMPLE_ATTRIBUTE(debugfs_trigger_reset_fops, NULL,
			mhi_debugfs_trigger_reset, "%llu\n");

/**
 * @brief initialize debugfs
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
void mhi_init_debugfs(struct mhi_controller *mhi_cntrl)
{
	struct dentry *dentry;
	char node[32];

	if (!mhi_cntrl->parent)
		return;

	snprintf(node, sizeof(node), "%04x_%02u:%02u.%02u",
		 mhi_cntrl->dev_id, mhi_cntrl->domain, mhi_cntrl->bus,
		 mhi_cntrl->slot);

	dentry = debugfs_create_dir(node, mhi_cntrl->parent);
	if (IS_ERR_OR_NULL(dentry))
		return;

	debugfs_create_file("states", 0444, dentry, mhi_cntrl,
			    &debugfs_state_ops);
	debugfs_create_file("events", 0444, dentry, mhi_cntrl,
			    &debugfs_ev_ops);
	debugfs_create_file("chan", 0444, dentry, mhi_cntrl, &debugfs_chan_ops);
	debugfs_create_file("reset", 0444, dentry, mhi_cntrl,
			    &debugfs_trigger_reset_fops);
	mhi_cntrl->dentry = dentry;
}

/**
 * @brief uninitialize debugfs
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
void mhi_deinit_debugfs(struct mhi_controller *mhi_cntrl)
{
	if (mhi_cntrl->dentry) {
		debugfs_remove_recursive(mhi_cntrl->dentry);
		mhi_cntrl->dentry = NULL;
	}
}
#else 
void mhi_init_debugfs(struct mhi_controller *mhi_cntrl)
{}

void mhi_deinit_debugfs(struct mhi_controller *mhi_cntrl)
{}
#endif /* CONFIG_DEBUG_FS */

/**
 * @brief init device context
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
int mhi_init_dev_ctxt(struct mhi_controller *mhi_cntrl)
{
	struct mhi_ctxt *mhi_ctxt;
	struct mhi_chan_ctxt *chan_ctxt;
	struct mhi_event_ctxt *er_ctxt;
	struct mhi_cmd_ctxt *cmd_ctxt;
	struct mhi_chan *mhi_chan;
	struct mhi_event *mhi_event;
	struct mhi_cmd *mhi_cmd;
	int i;

	//atomic_set(&mhi_cntrl->dev_wake, 0);
	//atomic_set(&mhi_cntrl->alloc_size, 0);
	//atomic_set(&mhi_cntrl->pending_pkts, 0);

	mhi_ctxt = kzalloc(sizeof(*mhi_ctxt), GFP_KERNEL);
	if (!mhi_ctxt)
		return -ENOMEM;

	/* setup channel ctxt */
#if 0    
	mhi_ctxt->chan_ctxt = mhi_alloc_coherent(mhi_cntrl,
			sizeof(*mhi_ctxt->chan_ctxt) * mhi_cntrl->max_chan,
			&mhi_ctxt->chan_ctxt_addr, GFP_KERNEL);
	if (!mhi_ctxt->chan_ctxt)
		goto error_alloc_chan_ctxt;
#endif
    mhi_ctxt->chan_ctxt_addr = ((void *)mhi_cntrl->ctrl_seg->xfer_ring - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;

	MHI_LOG("chan_ctxt_addr 0xx%llx", (unsigned long long)mhi_ctxt->chan_ctxt_addr);

    mhi_ctxt->chan_ctxt = mhi_cntrl->ctrl_seg->xfer_ring;

	mhi_chan = mhi_cntrl->mhi_chan;
	chan_ctxt = mhi_ctxt->chan_ctxt;
	for (i = 0; i < mhi_cntrl->max_chan; i++, chan_ctxt++, mhi_chan++) {
		/* If it's offload channel skip this step */
		if (mhi_chan->offload_ch)
			continue;

		chan_ctxt->chstate = MHI_CH_STATE_DISABLED;
		chan_ctxt->brstmode = mhi_chan->db_cfg.brstmode;
		chan_ctxt->pollcfg = mhi_chan->db_cfg.pollcfg;
		chan_ctxt->chtype = mhi_chan->type;
		chan_ctxt->erindex = mhi_chan->er_index;

        MHI_LOG("i %d erindex %d", i, mhi_chan->er_index);

		mhi_chan->ch_state = MHI_CH_STATE_DISABLED;
		mhi_chan->tre_ring.db_addr = &chan_ctxt->wp;
	}

	/* setup event context */
#if 0
	mhi_ctxt->er_ctxt = mhi_alloc_coherent(mhi_cntrl,
			sizeof(*mhi_ctxt->er_ctxt) * mhi_cntrl->total_ev_rings,
			&mhi_ctxt->er_ctxt_addr, GFP_KERNEL);
#endif
    mhi_ctxt->er_ctxt_addr = ((void *)mhi_cntrl->ctrl_seg->evt_ring - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;

	MHI_LOG("er_ctxt_addr 0xx%llx", (unsigned long long)mhi_ctxt->er_ctxt_addr);

    mhi_ctxt->er_ctxt = mhi_cntrl->ctrl_seg->evt_ring;

//	if (!mhi_ctxt->er_ctxt)
//		goto error_alloc_er_ctxt;

	er_ctxt = mhi_ctxt->er_ctxt;
	mhi_event = mhi_cntrl->mhi_event;
	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, er_ctxt++,
		     mhi_event++) {
		struct mhi_ring *ring = &mhi_event->ring;

		/* it's a satellite ev, we do not touch it */
		if (mhi_event->offload_ev)
			continue;

		er_ctxt->intmodc = 0;
		er_ctxt->intmodt = mhi_event->intmod;
		er_ctxt->ertype = MHI_ER_TYPE_VALID;

	    MHI_LOG("msi_allocated %d", mhi_cntrl->msi_allocated);
        if (mhi_cntrl->msi_allocated == 1) 
            mhi_event->msi = 0;     // all msi should set to 0

		er_ctxt->msivec = mhi_event->msi;
		mhi_event->db_cfg.db_mode = true;

		ring->el_size = sizeof(struct mhi_tre);
		ring->len = ring->el_size * ring->elements;
		MHI_LOG("ev_ring %d elements %ld", i, (long)ring->elements);

        MHI_LOG("ring 0x%p len %ld", ring, (long)ring->len);

	    ring->alloc_size = ring->len;
        MHI_LOG("alloc_size %ld", (long)ring->alloc_size);
	    
        ring->pre_aligned = &mhi_cntrl->ctrl_seg->evt_ring_element[i][0];

        MHI_LOG("pre_aligned 0x%p", ring->pre_aligned);

	    ring->iommu_base = (ring->pre_aligned - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;
        MHI_LOG("dma_handle 0x%llx iommu_base 0x%llx", (unsigned long long)ring->dma_handle, (unsigned long long)ring->iommu_base);

	    ring->base = ring->pre_aligned;
        MHI_LOG("base 0x%p", ring->base);

		MHI_LOG("ring %d base 0x%p", i, ring->base);

		ring->rp = ring->wp = ring->base;
		er_ctxt->rbase = ring->iommu_base;
		er_ctxt->rp = er_ctxt->wp = er_ctxt->rbase;
		er_ctxt->rlen = ring->len;
		ring->ctxt_wp = &er_ctxt->wp;

		MHI_LOG("ring %d ctxt_wp 0x%p", i, ring->ctxt_wp);
	}

	/* setup cmd context */
#if 0    
	mhi_ctxt->cmd_ctxt = mhi_alloc_coherent(mhi_cntrl,
				sizeof(*mhi_ctxt->cmd_ctxt) * NR_OF_CMD_RINGS,
				&mhi_ctxt->cmd_ctxt_addr, GFP_KERNEL);
	if (!mhi_ctxt->cmd_ctxt)
		goto error_alloc_er;
#endif

    mhi_ctxt->cmd_ctxt_addr = ((void *)mhi_cntrl->ctrl_seg->cmd_ring - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;

	MHI_LOG("cmd_ctxt_addr 0xx%llx", (unsigned long long)mhi_ctxt->cmd_ctxt_addr);

    mhi_ctxt->cmd_ctxt = &mhi_cntrl->ctrl_seg->cmd_ring[0];

	mhi_cmd = mhi_cntrl->mhi_cmd;
	cmd_ctxt = mhi_ctxt->cmd_ctxt;
	for (i = 0; i < NR_OF_CMD_RINGS; i++, mhi_cmd++, cmd_ctxt++) {
		struct mhi_ring *ring = &mhi_cmd->ring;

		ring->el_size = sizeof(struct mhi_tre);
		ring->elements = CMD_EL_PER_RING;
		ring->len = ring->el_size * ring->elements;

        MHI_LOG("ring 0x%p len %ld", ring, (long)ring->len);

	    ring->alloc_size = ring->len;
        MHI_LOG("alloc_size %ld", (long)ring->alloc_size);

        ring->pre_aligned = &mhi_cntrl->ctrl_seg->cmd_ring_element[0];

        MHI_LOG("pre_aligned 0x%p", ring->pre_aligned);

	    ring->iommu_base = (ring->pre_aligned - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;
        MHI_LOG("dma_handle 0x%llx iommu_base 0x%llx", (unsigned long long)ring->dma_handle, (unsigned long long)ring->iommu_base);

	    ring->base = ring->pre_aligned;
        MHI_LOG("base 0x%p", ring->base);

		ring->rp = ring->wp = ring->base;
		cmd_ctxt->rbase = ring->iommu_base;
		cmd_ctxt->rp = cmd_ctxt->wp = cmd_ctxt->rbase;
		cmd_ctxt->rlen = ring->len;
		ring->ctxt_wp = &cmd_ctxt->wp;
	}

	mhi_cntrl->mhi_ctxt = mhi_ctxt;

	return 0;
}

/**
 * @brief get time sync event configuration
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
static int mhi_get_tsync_er_cfg(struct mhi_controller *mhi_cntrl)
{
	int i;
	struct mhi_event *mhi_event = mhi_cntrl->mhi_event;

	/* find event ring with timesync support */
	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++)
		if (mhi_event->data_type == MHI_ER_TSYNC_ELEMENT_TYPE)
			return mhi_event->er_index;

	return -ENOENT;
}

/**
 * @brief initialize time sync
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
int mhi_init_timesync(struct mhi_controller *mhi_cntrl)
{
	struct mhi_timesync *mhi_tsync;
	u32 time_offset, db_offset;
	int ret;

	read_lock_bh(&mhi_cntrl->pm_lock);

	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		ret = -EIO;
		goto exit_timesync;
	}

	ret = mhi_get_capability_offset(mhi_cntrl, TIMESYNC_CAP_ID,
					&time_offset);
	if (ret) {
		MHI_LOG("No timesync capability found\n");
		goto exit_timesync;
	}

	read_unlock_bh(&mhi_cntrl->pm_lock);

	if (!mhi_cntrl->time_get || !mhi_cntrl->lpm_disable ||
	     !mhi_cntrl->lpm_enable)
		return -EINVAL;

	/* register method supported */
	mhi_tsync = kzalloc(sizeof(*mhi_tsync), GFP_KERNEL);
	if (!mhi_tsync)
		return -ENOMEM;

	spin_lock_init(&mhi_tsync->lock);
	mutex_init(&mhi_tsync->lpm_mutex);
	INIT_LIST_HEAD(&mhi_tsync->head);
	init_completion(&mhi_tsync->completion);

	/* save time_offset for obtaining time */
	MHI_LOG("TIME OFFS:0x%x\n", time_offset);
	mhi_tsync->time_reg = mhi_cntrl->regs + time_offset
			      + TIMESYNC_TIME_LOW_OFFSET;

	mhi_cntrl->mhi_tsync = mhi_tsync;

	ret = mhi_create_timesync_sysfs(mhi_cntrl);
	if (unlikely(ret)) {
		/* kernel method still work */
		MHI_ERR("Failed to create timesync sysfs nodes\n");
	}

	read_lock_bh(&mhi_cntrl->pm_lock);

	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		ret = -EIO;
		goto exit_timesync;
	}

	/* get DB offset if supported, else return */
	ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->regs,
			   time_offset + TIMESYNC_DB_OFFSET, &db_offset);
	if (ret || !db_offset) {
		ret = 0;
		goto exit_timesync;
	}

	MHI_LOG("TIMESYNC_DB OFFS:0x%x\n", db_offset);
	mhi_tsync->db = mhi_cntrl->regs + db_offset;

	read_unlock_bh(&mhi_cntrl->pm_lock);

	/* get time-sync event ring configuration */
	ret = mhi_get_tsync_er_cfg(mhi_cntrl);
	if (ret < 0) {
		MHI_LOG("Could not find timesync event ring\n");
		return ret;
	}

	mhi_tsync->er_index = ret;

	ret = mhi_send_cmd(mhi_cntrl, NULL, MHI_CMD_TIMSYNC_CFG);
	if (ret) {
		MHI_ERR("Failed to send time sync cfg cmd\n");
		return ret;
	}

	ret = wait_for_completion_timeout(&mhi_tsync->completion,
			msecs_to_jiffies(mhi_cntrl->timeout_ms));

	if (!ret || mhi_tsync->ccs != MHI_EV_CC_SUCCESS) {
		MHI_ERR("Failed to get time cfg cmd completion\n");
		return -EIO;
	}

	return 0;

exit_timesync:
	read_unlock_bh(&mhi_cntrl->pm_lock);

	return ret;
}

/**
 * @brief initialize mhi mmio
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
int mhi_init_mmio(struct mhi_controller *mhi_cntrl)
{
	u32 val;
	int i, ret;
	struct mhi_chan *mhi_chan;
	struct mhi_event *mhi_event;
	void __iomem *base = mhi_cntrl->regs;

	MHI_LOG("Initializing MMIO\n");

    MHI_LOG("base 0x%px\n", base);

	ret = mhi_read_reg(mhi_cntrl, base, MHICFG, &val);
	if (ret)
		return -EIO;

	MHI_LOG("MHICFG:0x%x\n", val);

	/* NER */
	val = (val & ~MHICFG_NER_MASK) | (mhi_cntrl->total_ev_rings << MHICFG_NER_SHIFT);

	/* NHWER */
	val = (val & ~MHICFG_NHWER_MASK) | (NUM_MHI_HW_EVT_RINGS << MHICFG_NHWER_SHIFT);

	mhi_write_reg(mhi_cntrl, base, MHICFG, val);

	MHI_LOG("MHICFG write:0x%x\n", val);

	/* set up DB register for all the chan rings */
	ret = mhi_read_reg_field(mhi_cntrl, base, CHDBOFF, CHDBOFF_CHDBOFF_MASK,
				 CHDBOFF_CHDBOFF_SHIFT, &val);
	if (ret)
		return -EIO;

	MHI_LOG("CHDBOFF:0x%x\n", val);

	/* setup wake db */
	mhi_cntrl->wake_db = base + val + (8 * MHI_DEV_WAKE_DB);
	mhi_write_reg(mhi_cntrl, mhi_cntrl->wake_db, 4, 0);
	mhi_write_reg(mhi_cntrl, mhi_cntrl->wake_db, 0, 0);
	mhi_cntrl->wake_set = false;

	/* setup channel db addresses */
	mhi_chan = mhi_cntrl->mhi_chan;
	for (i = 0; i < mhi_cntrl->max_chan; i++, val += 8, mhi_chan++) {
		mhi_chan->tre_ring.db_addr = base + val;
	    
        MHI_LOG("chan %d ChanDb 0x%px\n", i, mhi_chan->tre_ring.db_addr);
    }

	/* setup event ring db addresses */
	ret = mhi_read_reg_field(mhi_cntrl, base, ERDBOFF, ERDBOFF_ERDBOFF_MASK,
				 ERDBOFF_ERDBOFF_SHIFT, &val);
	if (ret)
		return -EIO;

	MHI_LOG("ERDBOFF:0x%x\n", val);

	mhi_event = mhi_cntrl->mhi_event;
	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, val += 8, mhi_event++) {
		if (mhi_event->offload_ev)
			continue;

		mhi_event->ring.db_addr = base + val;
        MHI_LOG("evt %d EvtDb 0x%px\n", i, mhi_event->ring.db_addr);
	}

	/* set up DB register for primary CMD rings */
	mhi_cntrl->mhi_cmd[PRIMARY_CMD_RING].ring.db_addr = base + CRDB_LOWER;

	MHI_LOG("CmdDb 0x%px\n",
           mhi_cntrl->mhi_cmd[PRIMARY_CMD_RING].ring.db_addr);

	MHI_LOG("Programming all MMIO values.\n");

	MHI_WRITE_REG_64(base, CCABAP, mhi_cntrl->mhi_ctxt->chan_ctxt_addr);
	MHI_WRITE_REG_64(base, ECABAP, mhi_cntrl->mhi_ctxt->er_ctxt_addr);
	MHI_WRITE_REG_64(base, CRCBAP, mhi_cntrl->mhi_ctxt->cmd_ctxt_addr);
	MHI_WRITE_REG_64(base, MHICTRLBASE, mhi_cntrl->mem_props.phys_aligned);
	MHI_WRITE_REG_64(base, MHICTRLLIMIT, mhi_cntrl->mem_props.phys_aligned + mhi_cntrl->mem_props.size);
	MHI_WRITE_REG_64(base, MHIDATABASE, mhi_cntrl->iova_start);
	MHI_WRITE_REG_64(base, MHIDATALIMIT, mhi_cntrl->iova_stop);

	return 0;
}

/**
 * @brief uninitialize mhi channel context
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            mhi_chan 					mhi channel
 */
void mhi_deinit_chan_ctxt(struct mhi_controller *mhi_cntrl,
			  struct mhi_chan *mhi_chan)
{
	struct mhi_ring *buf_ring;
	struct mhi_ring *tre_ring;
	struct mhi_chan_ctxt *chan_ctxt;

	buf_ring = &mhi_chan->buf_ring;
	tre_ring = &mhi_chan->tre_ring;
	chan_ctxt = &mhi_cntrl->mhi_ctxt->chan_ctxt[mhi_chan->chan];

	//MHI_LOG("mhi_free_coherent\n");
	//mhi_free_coherent(mhi_cntrl, tre_ring->alloc_size,
	//		  tre_ring->pre_aligned, tre_ring->dma_handle);
	MHI_LOG("vfree: buf_ring->base 0x%px\n", buf_ring->base);
	vfree(buf_ring->base);

	buf_ring->base = tre_ring->base = NULL;
	chan_ctxt->rbase = 0;
}

/**
 * @brief initialize mhi channel context
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            mhi_chan 					mhi channel
 */
int mhi_init_chan_ctxt(struct mhi_controller *mhi_cntrl,
		       struct mhi_chan *mhi_chan)
{
	struct mhi_ring *buf_ring;
	struct mhi_ring *tre_ring;
	struct mhi_chan_ctxt *chan_ctxt;
	//int ret;
    
    MHI_LOG("mhi_cntrl 0x%p mhi_chan 0x%p chan %d\n", mhi_cntrl, mhi_chan, mhi_chan->chan);

    buf_ring = &mhi_chan->buf_ring;
	tre_ring = &mhi_chan->tre_ring;
	tre_ring->el_size = sizeof(struct mhi_tre);
	tre_ring->len = tre_ring->el_size * tre_ring->elements;
	chan_ctxt = &mhi_cntrl->mhi_ctxt->chan_ctxt[mhi_chan->chan];

#if 0
	ret = mhi_alloc_aligned_ring(mhi_cntrl, tre_ring, tre_ring->len);
	if (ret)
		return -ENOMEM;
#endif

    MHI_LOG("tre_ring 0x%p len %ld", tre_ring, (long)tre_ring->len);

	tre_ring->alloc_size = tre_ring->len;
    MHI_LOG("alloc_size %ld", (long)tre_ring->alloc_size);
	    
    tre_ring->pre_aligned = &mhi_cntrl->ctrl_seg->xfer_ring_element[mhi_chan->tre_index][0];

    MHI_LOG("pre_aligned 0x%p", tre_ring->pre_aligned);

	tre_ring->iommu_base = (tre_ring->pre_aligned - (void *)mhi_cntrl->ctrl_seg) + mhi_cntrl->mem_props.phys_aligned;
    MHI_LOG("dma_handle 0x%llx iommu_base 0x%llx", (unsigned long long)tre_ring->dma_handle, (unsigned long long)tre_ring->iommu_base);

	tre_ring->base = tre_ring->pre_aligned;
    MHI_LOG("base 0x%p", tre_ring->base);

	buf_ring->el_size = sizeof(struct mhi_buf_info);
	buf_ring->len = buf_ring->el_size * buf_ring->elements;
	buf_ring->base = vzalloc(buf_ring->len);

	if (!buf_ring->base) {
//		mhi_free_coherent(mhi_cntrl, tre_ring->alloc_size,
//				  tre_ring->pre_aligned, tre_ring->dma_handle);
		return -ENOMEM;
	}

	chan_ctxt->chstate = MHI_CH_STATE_ENABLED;
	chan_ctxt->rbase = tre_ring->iommu_base;
	chan_ctxt->rp = chan_ctxt->wp = chan_ctxt->rbase;
	chan_ctxt->rlen = tre_ring->len;
	tre_ring->ctxt_wp = &chan_ctxt->wp;

	tre_ring->rp = tre_ring->wp = tre_ring->base;
	buf_ring->rp = buf_ring->wp = buf_ring->base;
	mhi_chan->db_cfg.db_mode = 1;

	/* update to all cores */
	smp_wmb();

	return 0;
}

int mhi_device_configure(struct mhi_device *mhi_dev,
			 enum dma_data_direction dir,
			 struct mhi_buf *cfg_tbl,
			 int elements)
{
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan;
	struct mhi_event_ctxt *er_ctxt;
	struct mhi_chan_ctxt *ch_ctxt;
	int er_index, chan;

	switch (dir) {
	case DMA_TO_DEVICE:
		mhi_chan = mhi_dev->ul_chan;
		break;
	case DMA_BIDIRECTIONAL:
	case DMA_FROM_DEVICE:
	case DMA_NONE:
		mhi_chan = mhi_dev->dl_chan;
		break;
	default:
		return -EINVAL;
	}

	er_index = mhi_chan->er_index;
	chan = mhi_chan->chan;

	for (; elements > 0; elements--, cfg_tbl++) {
		/* update event context array */
		if (!strcmp(cfg_tbl->name, "ECA")) {
			er_ctxt = &mhi_cntrl->mhi_ctxt->er_ctxt[er_index];
			if (sizeof(*er_ctxt) != cfg_tbl->len) {
				MHI_ERR(
					"Invalid ECA size, expected:%zu actual%zu\n",
					sizeof(*er_ctxt), cfg_tbl->len);
				return -EINVAL;
			}
			memcpy((void *)er_ctxt, cfg_tbl->buf, sizeof(*er_ctxt));
			continue;
		}

		/* update channel context array */
		if (!strcmp(cfg_tbl->name, "CCA")) {
			ch_ctxt = &mhi_cntrl->mhi_ctxt->chan_ctxt[chan];
			if (cfg_tbl->len != sizeof(*ch_ctxt)) {
				MHI_ERR(
					"Invalid CCA size, expected:%zu actual:%zu\n",
					sizeof(*ch_ctxt), cfg_tbl->len);
				return -EINVAL;
			}
			memcpy((void *)ch_ctxt, cfg_tbl->buf, sizeof(*ch_ctxt));
			continue;
		}

		return -EINVAL;
	}

	return 0;
}

#if 0
static int of_parse_ev_cfg(struct mhi_controller *mhi_cntrl,
			   struct device_node *of_node)
{
	int i, ret, num = 0;
	struct mhi_event *mhi_event;
	struct device_node *child;

	of_node = of_find_node_by_name(of_node, "mhi_events");
	if (!of_node)
		return -EINVAL;

	for_each_available_child_of_node(of_node, child) {
		if (!strcmp(child->name, "mhi_event"))
			num++;
	}

	if (!num)
		return -EINVAL;

	mhi_cntrl->total_ev_rings = num;
	mhi_cntrl->mhi_event = kcalloc(num, sizeof(*mhi_cntrl->mhi_event),
				       GFP_KERNEL);
	if (!mhi_cntrl->mhi_event)
		return -ENOMEM;

	/* populate ev ring */
	mhi_event = mhi_cntrl->mhi_event;
	i = 0;
	for_each_available_child_of_node(of_node, child) {
		if (strcmp(child->name, "mhi_event"))
			continue;

		mhi_event->er_index = i++;
		ret = of_property_read_u32(child, "mhi,num-elements",
					   (u32 *)&mhi_event->ring.elements);
		if (ret)
			goto error_ev_cfg;

		ret = of_property_read_u32(child, "mhi,intmod",
					   &mhi_event->intmod);
		if (ret)
			goto error_ev_cfg;

		ret = of_property_read_u32(child, "mhi,msi",
					   &mhi_event->msi);
		if (ret)
			goto error_ev_cfg;

		ret = of_property_read_u32(child, "mhi,chan",
					   &mhi_event->chan);
		if (!ret) {
			if (mhi_event->chan >= mhi_cntrl->max_chan)
				goto error_ev_cfg;
			/* this event ring has a dedicated channel */
			mhi_event->mhi_chan =
				&mhi_cntrl->mhi_chan[mhi_event->chan];
		}

		ret = of_property_read_u32(child, "mhi,priority",
					   &mhi_event->priority);
		if (ret)
			goto error_ev_cfg;

		ret = of_property_read_u32(child, "mhi,brstmode",
					   &mhi_event->db_cfg.brstmode);
		if (ret || MHI_INVALID_BRSTMODE(mhi_event->db_cfg.brstmode))
			goto error_ev_cfg;

		mhi_event->db_cfg.process_db =
			(mhi_event->db_cfg.brstmode == MHI_BRSTMODE_ENABLE) ?
			mhi_db_brstmode : mhi_db_brstmode_disable;

		ret = of_property_read_u32(child, "mhi,data-type",
					   &mhi_event->data_type);
		if (ret)
			mhi_event->data_type = MHI_ER_DATA_ELEMENT_TYPE;

		if (mhi_event->data_type > MHI_ER_DATA_TYPE_MAX)
			goto error_ev_cfg;

		switch (mhi_event->data_type) {
		case MHI_ER_DATA_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_data_event_ring;
			break;
		case MHI_ER_CTRL_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_ctrl_ev_ring;
			break;
		case MHI_ER_TSYNC_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_tsync_event_ring;
			break;
		}

		mhi_event->hw_ring = of_property_read_bool(child, "mhi,hw-ev");
		if (mhi_event->hw_ring)
			mhi_cntrl->hw_ev_rings++;
		else
			mhi_cntrl->sw_ev_rings++;
		mhi_event->cl_manage = of_property_read_bool(child,
							"mhi,client-manage");
		mhi_event->offload_ev = of_property_read_bool(child,
							      "mhi,offload");
		mhi_event++;
	}

	/* we need msi for each event ring + additional one for BHI */
	mhi_cntrl->msi_required = mhi_cntrl->total_ev_rings + 1;

	return 0;

error_ev_cfg:

	kfree(mhi_cntrl->mhi_event);
	return -EINVAL;
}
#endif

struct mhi_event_properties {
    u32 num_elements;   /* number of elements event ring support */
    u32 intmod;         /* interrupt moderation time in ms */
    u32 msi;            /* MSI associated with this event ring */
    u32 chan;           /* dedicated channel number */
    u32 priority;       /* event ring priority */
    u32 brstmode;       /* event doorbell mode configuration */
    u32 data_type;      /* type of data this event ring will process */
    bool hw_ev;         /* event ring associated with hardware channels */
    bool client_manage; /* client manages the event ring */
    bool offload;       /* event ring associated with offload channel */
};

static struct mhi_event_properties event_config[] = {
/*    num                        intmod  msi chan    pri brs type    hw_ev   c_m off  */
    { NUM_MHI_EVT_RING_ELEMENTS,  1,      1,  0,      1,  2,  1,      0,      0,  0},
    { NUM_MHI_EVT_RING_ELEMENTS,  1,      2,  100,    1,  3,  0,      1,      0,  0},
    { NUM_MHI_EVT_RING_ELEMENTS,  1,      3,  101,    1,  3,  0,      1,      0,  0},
//    { 240,  1,      3,  101,    1,  2,  0,      1,      0,  0},
//    { 240,  1,      4,  0,      1,  2,  1,      0,      0,  0},
};

/**
 * @brief get mhi event configuration
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            of_node 					device node
 */
static int of_parse_ev_cfg(struct mhi_controller *mhi_cntrl,
			   struct device_node *of_node)
{
	int i, num = 0;
	struct mhi_event *mhi_event;

    num = sizeof(event_config)/sizeof(struct mhi_event_properties);
	mhi_cntrl->total_ev_rings = num;
	mhi_cntrl->mhi_event = kcalloc(num, sizeof(*mhi_cntrl->mhi_event),
				       GFP_KERNEL);
	if (!mhi_cntrl->mhi_event)
		return -ENOMEM;

	/* populate ev ring */
	mhi_event = mhi_cntrl->mhi_event;
    for (i = 0; i < num; i++) {

		mhi_event->er_index = i;
		
        mhi_event->ring.elements = event_config[i].num_elements;
        mhi_event->intmod = event_config[i].intmod;
        mhi_event->msi = event_config[i].msi;
        
        mhi_event->chan = event_config[i].chan;

		if (mhi_event->chan >= mhi_cntrl->max_chan)
			goto error_ev_cfg;

		/* this event ring has a dedicated channel */
		mhi_event->mhi_chan =
			&mhi_cntrl->mhi_chan[mhi_event->chan];

        mhi_event->priority = event_config[i].priority;
        mhi_event->db_cfg.brstmode = event_config[i].brstmode;
		if (MHI_INVALID_BRSTMODE(mhi_event->db_cfg.brstmode))
			goto error_ev_cfg;

		mhi_event->db_cfg.process_db =
			(mhi_event->db_cfg.brstmode == MHI_BRSTMODE_ENABLE) ?
			mhi_db_brstmode : mhi_db_brstmode_disable;

        mhi_event->data_type = event_config[i].data_type;

		if (mhi_event->data_type > MHI_ER_DATA_TYPE_MAX)
			goto error_ev_cfg;

		switch (mhi_event->data_type) {
		case MHI_ER_DATA_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_data_event_ring;
			break;
		case MHI_ER_CTRL_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_ctrl_ev_ring;
			break;
		case MHI_ER_TSYNC_ELEMENT_TYPE:
			mhi_event->process_event = mhi_process_tsync_event_ring;
			break;
		}

		mhi_event->hw_ring = event_config[i].hw_ev;
		if (mhi_event->hw_ring)
			mhi_cntrl->hw_ev_rings++;
		else
			mhi_cntrl->sw_ev_rings++;

		mhi_event->cl_manage = event_config[i].client_manage;
		mhi_event->offload_ev = event_config[i].offload;
		mhi_event++;
	}

	/* we need msi for each event ring + additional one for BHI */
	mhi_cntrl->msi_required = mhi_cntrl->total_ev_rings + 1;

	return 0;

error_ev_cfg:

	kfree(mhi_cntrl->mhi_event);
	return -EINVAL;
}

#if 0
static int of_parse_ch_cfg(struct mhi_controller *mhi_cntrl,
			   struct device_node *of_node)
{
	int ret;
	struct device_node *child;
	u32 chan;

	ret = of_property_read_u32(of_node, "mhi,max-channels",
				   &mhi_cntrl->max_chan);
	if (ret)
		return ret;

	of_node = of_find_node_by_name(of_node, "mhi_channels");
	if (!of_node)
		return -EINVAL;

	mhi_cntrl->mhi_chan = vzalloc(mhi_cntrl->max_chan *
				      sizeof(*mhi_cntrl->mhi_chan));
	if (!mhi_cntrl->mhi_chan)
		return -ENOMEM;

	INIT_LIST_HEAD(&mhi_cntrl->lpm_chans);

	/* populate channel configurations */
	for_each_available_child_of_node(of_node, child) {
		struct mhi_chan *mhi_chan;

		if (strcmp(child->name, "mhi_chan"))
			continue;

		ret = of_property_read_u32(child, "reg", &chan);
		if (ret || chan >= mhi_cntrl->max_chan)
			goto error_chan_cfg;

		mhi_chan = &mhi_cntrl->mhi_chan[chan];

		ret = of_property_read_string(child, "label",
					      &mhi_chan->name);
		if (ret)
			goto error_chan_cfg;

		mhi_chan->chan = chan;

		ret = of_property_read_u32(child, "mhi,num-elements",
					   (u32 *)&mhi_chan->tre_ring.elements);
		if (!ret && !mhi_chan->tre_ring.elements)
			goto error_chan_cfg;

		/*
		 * For some channels, local ring len should be bigger than
		 * transfer ring len due to internal logical channels in device.
		 * So host can queue much more buffers than transfer ring len.
		 * Example, RSC channels should have a larger local channel
		 * than transfer ring length.
		 */
		ret = of_property_read_u32(child, "mhi,local-elements",
					   (u32 *)&mhi_chan->buf_ring.elements);
		if (ret)
			mhi_chan->buf_ring.elements =
				mhi_chan->tre_ring.elements;

		ret = of_property_read_u32(child, "mhi,event-ring",
					   &mhi_chan->er_index);
		if (ret)
			goto error_chan_cfg;

		ret = of_property_read_u32(child, "mhi,chan-dir",
					   &mhi_chan->dir);
		if (ret)
			goto error_chan_cfg;

		/*
		 * For most channels, chtype is identical to channel directions,
		 * if not defined, assign ch direction to chtype
		 */
		ret = of_property_read_u32(child, "mhi,chan-type",
					   &mhi_chan->type);
		if (ret)
			mhi_chan->type = (enum mhi_ch_type)mhi_chan->dir;

		ret = of_property_read_u32(child, "mhi,ee", &mhi_chan->ee_mask);
		if (ret)
			goto error_chan_cfg;

		of_property_read_u32(child, "mhi,pollcfg",
				     &mhi_chan->db_cfg.pollcfg);

		ret = of_property_read_u32(child, "mhi,data-type",
					   &mhi_chan->xfer_type);
		if (ret)
			goto error_chan_cfg;

		switch (mhi_chan->xfer_type) {
		case MHI_XFER_BUFFER:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_buf;
			break;
		case MHI_XFER_SKB:
			mhi_chan->queue_xfer = mhi_queue_skb;
			break;
		case MHI_XFER_SCLIST:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_sclist;
			break;
		case MHI_XFER_NOP:
			mhi_chan->queue_xfer = mhi_queue_nop;
			break;
		case MHI_XFER_DMA:
		case MHI_XFER_RSC_DMA:
			mhi_chan->queue_xfer = mhi_queue_dma;
			break;
		default:
			goto error_chan_cfg;
		}

		mhi_chan->lpm_notify = of_property_read_bool(child,
							     "mhi,lpm-notify");
		mhi_chan->offload_ch = of_property_read_bool(child,
							"mhi,offload-chan");
		mhi_chan->db_cfg.reset_req = of_property_read_bool(child,
							"mhi,db-mode-switch");
		mhi_chan->pre_alloc = of_property_read_bool(child,
							    "mhi,auto-queue");
		mhi_chan->auto_start = of_property_read_bool(child,
							     "mhi,auto-start");
		mhi_chan->wake_capable = of_property_read_bool(child,
							"mhi,wake-capable");

		if (mhi_chan->pre_alloc &&
		    (mhi_chan->dir != DMA_FROM_DEVICE ||
		     mhi_chan->xfer_type != MHI_XFER_BUFFER))
			goto error_chan_cfg;

		/* bi-dir and dirctionless channels must be a offload chan */
		if ((mhi_chan->dir == DMA_BIDIRECTIONAL ||
		     mhi_of_parse_ch_cfgchan->dir == DMA_NONE) && !mhi_chan->offload_ch)
			goto error_chan_cfg;

		/* if mhi host allocate the buffers then client cannot queue */
		if (mhi_chan->pre_alloc)
			mhi_chan->queue_xfer = mhi_queue_nop;

		if (!mhi_chan->offload_ch) {
			ret = of_property_read_u32(child, "mhi,doorbell-mode",
						   &mhi_chan->db_cfg.brstmode);
			if (ret ||
			    MHI_INVALID_BRSTMODE(mhi_chan->db_cfg.brstmode))
				goto error_chan_cfg;

			mhi_chan->db_cfg.process_db =
				(mhi_chan->db_cfg.brstmode ==
				 MHI_BRSTMODE_ENABLE) ?
				mhi_db_brstmode : mhi_db_brstmode_disable;
		}

		mhi_chan->configured = true;

		if (mhi_chan->lpm_notify)
			list_add_tail(&mhi_chan->node, &mhi_cntrl->lpm_chans);
	}

	return 0;

error_chan_cfg:
	vfree(mhi_cntrl->mhi_chan);

	return -EINVAL;
}
#endif

static struct mhi_chan_properties chan_config[] = {
/*    reg   label       num_el  ev_ring dir type db_mode ee     auto_q  auto_start */

//    { 0,    "LOOPBACK", 32,     1,      2,  0,  2,      0x14}, 
//    { 1,    "LOOPBACK", 32,     1,      1,  0,  2,      0x14}, 
    /* only add SAHARA devices for SBL */
    { 2,    "SAHARA",     32,     0,      1,  0,  2,      0x2},    // MHI_CLIENT_SAHARA_OUT = 4, MHI_EE_SBL
    { 3,    "SAHARA",     256,     0,      2,  0,  2,      0x2},    // MHI_CLIENT_SAHARA_IN = 5, MHI_EE_SBL
    { 4,    "DIAG",     32,     0,      1,  0,  2,      0x14},    // MHI_CLIENT_DIAG_OUT = 4,
    { 5,    "DIAG",     32,     0,      2,  0,  2,      0x14},    // MHI_CLIENT_DIAG_IN = 5, 
	{ 12,   "MBIM",     128,    0,      1,  0,  2,      0x14},    // MHI_CLIENT_MBIM_OUT = 12,
	{ 13,   "MBIM",     128,    0,      2,  0,  2,      0x14},    // MHI_CLIENT_MBIM_IN = 13,
	{ 14,   "QMI0",     32,     0,      1,  0,  2,      0x14},    // MHI_CLIENT_QMI_OUT = 14,
	{ 15,   "QMI0",     32,     0,      2,  0,  2,      0x14},    // MHI_CLIENT_QMI_IN = 15,
	{ 32,   "DUN",      32,     0,      1,  0,  2,      0x14},    // MHI_CLIENT_DUN_OUT = 32,
	{ 33,   "DUN",      32,     0,      2,  0,  2,      0x14},    // MHI_CLIENT_DUN_IN = 33,
#ifdef ADB_SUPPORT
	{ 36,   "ADB",      128,     0,      1,  0,  2,      0x14},    // MHI_CLIENT_ADB_OUT = 36,
	{ 37,   "ADB",      128,     0,      2,  0,  2,      0x14},    // MHI_CLIENT_ADB_IN = 37,
#endif	
//    { 20,   "IPCR",     32,     1,      1,  0,  2,      0x14,   false,  true},
//    { 20,   "IPCR",     32,     1,      2,  0,  2,      0x14,   true,   true},
    { 100,  "IP_HW0",   512,    1,      1,  6,  3,      0x14,   true,   true},      // MHI_CLIENT_IP_HW_0_OUT = 100,
    { 101,  "IP_HW0",   512,    2,      2,  4,  3,      0x14,   true,   true},      // MHI_CLIENT_IP_HW_0_IN = 101,
};

/**
 * @brief get mhi channel configuration
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            of_node 					device node
 */
static int of_parse_ch_cfg(struct mhi_controller *mhi_cntrl,
			   struct device_node *of_node)
{
	u32 chan, i;

    mhi_cntrl->max_chan = 128;

	mhi_cntrl->mhi_chan = vzalloc(mhi_cntrl->max_chan *
				      sizeof(*mhi_cntrl->mhi_chan));
	if (!mhi_cntrl->mhi_chan)
		return -ENOMEM;

	BUILD_BUG_ON(sizeof(chan_config)/sizeof(struct mhi_chan_properties) >= NUM_MHI_XFER_RINGS);

	INIT_LIST_HEAD(&mhi_cntrl->lpm_chans);

	/* populate channel configurations */
	for (i = 0; i < sizeof(chan_config)/sizeof(struct mhi_chan_properties); i++) {
        struct mhi_chan *mhi_chan;

        chan = chan_config[i].reg;

		mhi_chan = &mhi_cntrl->mhi_chan[chan];

		mhi_chan->chan = chan;

        mhi_chan->name = (const char *)chan_config[i].label;

        mhi_chan->tre_ring.elements = chan_config[i].num_elements;

		if (!mhi_chan->tre_ring.elements)
			goto error_chan_cfg;

		/*
		 * For some channels, local ring len should be bigger than
		 * transfer ring len due to internal logical channels in device.
		 * So host can queue much more buffers than transfer ring len.
		 * Example, RSC channels should have a larger local channel
		 * than transfer ring length.
		 */
		mhi_chan->buf_ring.elements = mhi_chan->tre_ring.elements;

        mhi_chan->er_index = chan_config[i].event_ring;

		mhi_chan->tre_index = i;

        mhi_chan->dir = chan_config[i].chan_dir;

		/*
		 * For most channels, chtype is identical to channel directions,
		 * if not defined, assign ch direction to chtype
		 */
		mhi_chan->type = chan_config[i].chan_dir;

		mhi_chan->ee_mask = chan_config[i].ee;
        
        mhi_chan->xfer_type = chan_config[i].data_type;

		switch (mhi_chan->xfer_type) {
		case MHI_XFER_BUFFER:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_buf;
			break;
		case MHI_XFER_SKB:
			mhi_chan->queue_xfer = mhi_queue_skb;
			break;
		case MHI_XFER_SCLIST:
			mhi_chan->gen_tre = mhi_gen_tre;
			mhi_chan->queue_xfer = mhi_queue_sclist;
			break;
		case MHI_XFER_NOP:
			mhi_chan->queue_xfer = mhi_queue_nop;
			break;
		case MHI_XFER_DMA:
		case MHI_XFER_RSC_DMA:
			mhi_chan->queue_xfer = mhi_queue_dma;
			break;
		case MHI_XFER_QMAP:
			mhi_chan->queue_xfer = mhi_queue_qmap;
			mhi_chan->queue_full = mhi_qmap_full;
			break;
		default:

			goto error_chan_cfg;
		}

		mhi_chan->lpm_notify = 0;
		mhi_chan->offload_ch = 0;
		mhi_chan->db_cfg.reset_req = 0;
		mhi_chan->pre_alloc = 0;
		//mhi_chan->auto_start = 0;
		mhi_chan->wake_capable = 0;

		if (mhi_chan->pre_alloc &&
		    (mhi_chan->dir != DMA_FROM_DEVICE ||
		     mhi_chan->xfer_type != MHI_XFER_BUFFER))
			goto error_chan_cfg;

		/* bi-dir and dirctionless channels must be a offload chan */
#if 0
		if ((mhi_chan->dir == DMA_BIDIRECTIONAL ||
		     mhi_of_parse_ch_cfgchan->dir == DMA_NONE) && !mhi_chan->offload_ch)
			goto error_chan_cfg;
#endif
		/* if mhi host allocate the buffers then client cannot queue */
		if (mhi_chan->pre_alloc)
			mhi_chan->queue_xfer = mhi_queue_nop;

		if (!mhi_chan->offload_ch) {
            mhi_chan->db_cfg.brstmode = chan_config[i].doorbell_mode;

			if (MHI_INVALID_BRSTMODE(mhi_chan->db_cfg.brstmode))
				goto error_chan_cfg;

			mhi_chan->db_cfg.process_db =
				(mhi_chan->db_cfg.brstmode ==
				 MHI_BRSTMODE_ENABLE) ?
				mhi_db_brstmode : mhi_db_brstmode_disable;
		}

		mhi_chan->configured = true;

		if (mhi_chan->lpm_notify)
			list_add_tail(&mhi_chan->node, &mhi_cntrl->lpm_chans);
	}

	return 0;

error_chan_cfg:
	vfree(mhi_cntrl->mhi_chan);

	return -EINVAL;
}

/**
 * @brief get mhi configuration
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            of_node 					device node
 */
static int of_parse_dt(struct mhi_controller *mhi_cntrl,
		       struct device_node *of_node)
{
	int ret;
	enum mhi_ee i;
	u32 *ee;
	u32 bhie_offset = 0;

	/* parse MHI channel configuration */
	ret = of_parse_ch_cfg(mhi_cntrl, of_node);
	if (ret)
		return ret;

	/* parse MHI event configuration */
	ret = of_parse_ev_cfg(mhi_cntrl, of_node);
	if (ret)
		goto error_ev_cfg;

	mhi_cntrl->timeout_ms = MHI_TIMEOUT_MS;

	mhi_cntrl->buffer_len = MHI_MAX_MTU;

	/* by default host allowed to ring DB both M0 and M2 state */
	mhi_cntrl->db_access = MHI_PM_M0 | MHI_PM_M2;

	/* parse the device ee table */
	for (i = MHI_EE_PBL, ee = mhi_cntrl->ee_table; i < MHI_EE_MAX;
	     i++, ee++) {
		/* setup the default ee before checking for override */
		*ee = i;

#if 0
		ret = of_property_match_string(of_node, "mhi,ee-names",
					       mhi_ee_str[i]);
		if (ret < 0)
			continue;

		of_property_read_u32_index(of_node, "mhi,ee", ret, ee);
#endif        
	}

	mhi_cntrl->bhie = mhi_cntrl->regs + bhie_offset;

	return 0;

error_ev_cfg:
	vfree(mhi_cntrl->mhi_chan);

	return ret;
}

/**
 * @brief register mhi controller
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
int of_register_mhi_controller(struct mhi_controller *mhi_cntrl)
{
	int ret;
	int i;
	struct mhi_event *mhi_event;
	struct mhi_chan *mhi_chan;
	struct mhi_cmd *mhi_cmd;
	struct mhi_device *mhi_dev;
	u32 soc_info;

	//if (!mhi_cntrl->of_node)
	//	return -EINVAL;

	if (!mhi_cntrl->runtime_get || !mhi_cntrl->runtime_put)
		return -EINVAL;

	if (!mhi_cntrl->status_cb || !mhi_cntrl->link_status)
		return -EINVAL;

	ret = of_parse_dt(mhi_cntrl, mhi_cntrl->of_node);
	if (ret)
		return -EINVAL;

	mhi_cntrl->mhi_cmd = kcalloc(NR_OF_CMD_RINGS,
				     sizeof(*mhi_cntrl->mhi_cmd), GFP_KERNEL);
	if (!mhi_cntrl->mhi_cmd) {
		ret = -ENOMEM;
		goto error_alloc_cmd;
	}

	INIT_LIST_HEAD(&mhi_cntrl->transition_list);
	mutex_init(&mhi_cntrl->pm_mutex);
	rwlock_init(&mhi_cntrl->pm_lock);
	spin_lock_init(&mhi_cntrl->transition_lock);
	spin_lock_init(&mhi_cntrl->wlock);
	INIT_WORK(&mhi_cntrl->st_worker, mhi_pm_st_worker);
	INIT_WORK(&mhi_cntrl->fw_worker, mhi_fw_load_worker);
	INIT_WORK(&mhi_cntrl->syserr_worker, mhi_pm_sys_err_worker);
	INIT_WORK(&mhi_cntrl->intvec_worker, mhi_intvec_worker);
	tasklet_init(&mhi_cntrl->intvec_task, mhi_intvec_task,
		(ulong)mhi_cntrl);
	init_waitqueue_head(&mhi_cntrl->state_event);
	mutex_init(&mhi_cntrl->dev_mutex);

	mhi_cmd = mhi_cntrl->mhi_cmd;
	for (i = 0; i < NR_OF_CMD_RINGS; i++, mhi_cmd++)
		spin_lock_init(&mhi_cmd->lock);

	mhi_event = mhi_cntrl->mhi_event;
	for (i = 0; i < mhi_cntrl->total_ev_rings; i++, mhi_event++) {
		if (mhi_event->offload_ev)
			continue;

		mhi_event->mhi_cntrl = mhi_cntrl;
		spin_lock_init(&mhi_event->lock);
		if (mhi_event->data_type == MHI_ER_CTRL_ELEMENT_TYPE)
			tasklet_init(&mhi_event->task, mhi_ctrl_ev_task,
				     (ulong)mhi_event);
		else
			tasklet_init(&mhi_event->task, mhi_ev_task,
				     (ulong)mhi_event);
	}

	mhi_chan = mhi_cntrl->mhi_chan;
	for (i = 0; i < mhi_cntrl->max_chan; i++, mhi_chan++) {
		mutex_init(&mhi_chan->mutex);
		init_completion(&mhi_chan->completion);
		rwlock_init(&mhi_chan->lock);
        spin_lock_init(&mhi_chan->skb_write_lock);
        spin_lock_init(&mhi_chan->dma_write_lock);
        spin_lock_init(&mhi_chan->qmap_write_lock);
		spin_lock_init(&mhi_chan->tre_lock);
	}
    
    mhi_cntrl->bounce_buf = false;

	if (mhi_cntrl->bounce_buf) {
		mhi_cntrl->map_single = mhi_map_single_use_bb;
		mhi_cntrl->unmap_single = mhi_unmap_single_use_bb;
	} else {
		mhi_cntrl->map_single = mhi_map_single_no_bb;
		mhi_cntrl->unmap_single = mhi_unmap_single_no_bb;
	}

	/* read the device info if possible */
	if (mhi_cntrl->regs) {
		ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->regs,
				   SOC_HW_VERSION_OFFS, &soc_info);
		if (ret)
			goto error_alloc_dev;

		mhi_cntrl->family_number =
			(soc_info & SOC_HW_VERSION_FAM_NUM_BMSK) >>
			SOC_HW_VERSION_FAM_NUM_SHFT;
		mhi_cntrl->device_number =
			(soc_info & SOC_HW_VERSION_DEV_NUM_BMSK) >>
			SOC_HW_VERSION_DEV_NUM_SHFT;
		mhi_cntrl->major_version =
			(soc_info & SOC_HW_VERSION_MAJOR_VER_BMSK) >>
			SOC_HW_VERSION_MAJOR_VER_SHFT;
		mhi_cntrl->minor_version =
			(soc_info & SOC_HW_VERSION_MINOR_VER_BMSK) >>
			SOC_HW_VERSION_MINOR_VER_SHFT;
	}

	/* register controller with mhi_bus */
	mhi_dev = mhi_alloc_device(mhi_cntrl);
	if (!mhi_dev) {
		ret = -ENOMEM;
		goto error_alloc_dev;
	}

	mhi_dev->dev_type = MHI_CONTROLLER_TYPE;
	mhi_dev->mhi_cntrl = mhi_cntrl;
	dev_set_name(&mhi_dev->dev, "%04x_%02u.%02u.%02u", mhi_dev->dev_id,
		     mhi_dev->domain, mhi_dev->bus, mhi_dev->slot);

	/* init wake source */
	device_init_wakeup(&mhi_dev->dev, true);

	ret = device_add(&mhi_dev->dev);
	if (ret)
		goto error_add_dev;

	mhi_cntrl->mhi_dev = mhi_dev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
#ifdef CONFIG_DEBUG_FS
	mhi_cntrl->parent = debugfs_lookup(mhi_bus_type.name, NULL);
#endif 	
#endif

	/* adding it to this list only for debug purpose */
	mutex_lock(&mhi_bus.lock);
	list_add_tail(&mhi_cntrl->node, &mhi_bus.controller_list);
	mutex_unlock(&mhi_bus.lock);

	return 0;

error_add_dev:
	mhi_dealloc_device(mhi_cntrl, mhi_dev);

error_alloc_dev:
	kfree(mhi_cntrl->mhi_cmd);

error_alloc_cmd:
	vfree(mhi_cntrl->mhi_chan);
	kfree(mhi_cntrl->mhi_event);

	return ret;
};
EXPORT_SYMBOL(of_register_mhi_controller);

/**
 * @brief unregister mhi controller
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 */
void mhi_unregister_mhi_controller(struct mhi_controller *mhi_cntrl)
{
	struct mhi_device *mhi_dev = mhi_cntrl->mhi_dev;

    if (mhi_cntrl->mhi_cmd) {
	    MHI_LOG("mhi_cmd\n");
	    kfree(mhi_cntrl->mhi_cmd);
        mhi_cntrl->mhi_cmd = NULL;
    }

    if (mhi_cntrl->mhi_event) {
	    MHI_LOG("mhi_event\n");
	    kfree(mhi_cntrl->mhi_event);
        mhi_cntrl->mhi_event = NULL;
    }

    if (mhi_cntrl->mhi_chan) {
	    MHI_LOG("mhi_chan\n");
	    vfree(mhi_cntrl->mhi_chan);
        mhi_cntrl->mhi_chan = NULL;
    }

	MHI_LOG("mhi_tsync 0x%px", mhi_cntrl->mhi_tsync);
    if (mhi_cntrl->mhi_tsync) {
	    MHI_LOG("mhi_tsync\n");
	    kfree(mhi_cntrl->mhi_tsync);
        mhi_cntrl->mhi_tsync = NULL;
    }
	
	mhi_cntrl->timesync_done = false;

	device_del(&mhi_dev->dev);
	put_device(&mhi_dev->dev);

	mutex_lock(&mhi_bus.lock);
	list_del(&mhi_cntrl->node);
	mutex_unlock(&mhi_bus.lock);
}
EXPORT_SYMBOL(mhi_unregister_mhi_controller);

/**
 * @brief set ptr to control private data
 *
 * @param[in ]            mhi_cntrl 				mhi controller
 * @param[in ]            priv 						private data
 */
static inline void mhi_controller_set_devdata(struct mhi_controller *mhi_cntrl,
					 void *priv)
{
	mhi_cntrl->priv_data = priv;
}

/**
 * @brief allocate mhi controller to register
 *
 * @param[in ]            size 				buffer size
 */
struct mhi_controller *mhi_alloc_controller(size_t size)
{
	struct mhi_controller *mhi_cntrl;

	mhi_cntrl = kzalloc(size + sizeof(*mhi_cntrl), GFP_KERNEL);

	if (mhi_cntrl && size)
		mhi_controller_set_devdata(mhi_cntrl, mhi_cntrl + 1);

	return mhi_cntrl;
}
EXPORT_SYMBOL(mhi_alloc_controller);

/**
 * @brief prepare device power
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
int mhi_prepare_for_power_up(struct mhi_controller *mhi_cntrl)
{
	int ret;
	u32 bhie_off;

	mutex_lock(&mhi_cntrl->pm_mutex);

    ret = mhi_init_ctrl_seq(mhi_cntrl);
	if (ret) {
		MHI_ERR("Error with init ctrl seq\n");
		goto error_dev_ctxt;
	}

	ret = mhi_init_dev_ctxt(mhi_cntrl);
	if (ret) {
		MHI_ERR("Error with init dev_ctxt\n");
		goto error_dev_ctxt;
	}

	/*
	 * allocate rddm table if specified, this table is for debug purpose
	 * so we'll ignore erros
	 */
	if (mhi_cntrl->rddm_size) {
		mhi_alloc_bhie_table(mhi_cntrl, &mhi_cntrl->rddm_image,
				     mhi_cntrl->rddm_size);

		/*
		 * This controller supports rddm, we need to manually clear
		 * BHIE RX registers since por values are undefined.
		 */
		if (!mhi_cntrl->bhie) {
			ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->regs, BHIEOFF,
					   &bhie_off);
			if (ret) {
				MHI_ERR("Error getting bhie offset\n");
				goto bhie_error;
			}

			mhi_cntrl->bhie = mhi_cntrl->regs + bhie_off;
		}

		memset_io(mhi_cntrl->bhie + BHIE_RXVECADDR_LOW_OFFS, 0,
			  BHIE_RXVECSTATUS_OFFS - BHIE_RXVECADDR_LOW_OFFS + 4);

		if (mhi_cntrl->rddm_image)
			mhi_rddm_prepare(mhi_cntrl, mhi_cntrl->rddm_image);
	}

	mhi_cntrl->pre_init = true;

	mutex_unlock(&mhi_cntrl->pm_mutex);

	return 0;

bhie_error:
	if (mhi_cntrl->rddm_image) {
		mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->rddm_image);
		mhi_cntrl->rddm_image = NULL;
	}

error_dev_ctxt:
	mutex_unlock(&mhi_cntrl->pm_mutex);

	return ret;
}
EXPORT_SYMBOL(mhi_prepare_for_power_up);

/**
 * @brief device power down cleanup
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
void mhi_unprepare_after_power_down(struct mhi_controller *mhi_cntrl)
{
	if (mhi_cntrl->fbc_image) {
		mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->fbc_image);
		mhi_cntrl->fbc_image = NULL;
	}

	if (mhi_cntrl->rddm_image) {
		mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->rddm_image);
		mhi_cntrl->rddm_image = NULL;
	}

	mhi_deinit_dev_ctxt(mhi_cntrl);
	mhi_cntrl->pre_init = false;
}
EXPORT_SYMBOL(mhi_unprepare_after_power_down);

/**
 * @brief match device to driver
 *
 * @param[in ]            dev 					mhi device
 * @param[in ]            drv 					driver
 */
static int mhi_match(struct device *dev, struct device_driver *drv)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	struct mhi_driver *mhi_drv = to_mhi_driver(drv);
	const struct mhi_device_id *id;

	/* if controller type there is no client driver associated with it */
	if (mhi_dev->dev_type == MHI_CONTROLLER_TYPE)
		return 0;

	for (id = mhi_drv->id_table; id->chan[0]; id++)
		if (!strcmp(mhi_dev->chan_name, id->chan)) {
			mhi_dev->id = id;
			return 1;
		}

	return 0;
};

/**
 * @brief release device
 *
 * @param[in ]            dev 					mhi device
 */
static void mhi_release_device(struct device *dev)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);

	if (mhi_dev->ul_chan)
		mhi_dev->ul_chan->mhi_dev = NULL;

	if (mhi_dev->dl_chan)
		mhi_dev->dl_chan->mhi_dev = NULL;

	kfree(mhi_dev);
}

struct bus_type mhi_bus_type = {
	.name = "mhi",
	.dev_name = "mhi",
	.match = mhi_match,
};

/**
 * @brief probe device
 *
 * @param[in ]            dev 					mhi device
 */
static int mhi_driver_probe(struct device *dev)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct device_driver *drv = dev->driver;
	struct mhi_driver *mhi_drv = to_mhi_driver(drv);
	struct mhi_event *mhi_event;
	struct mhi_chan *ul_chan = mhi_dev->ul_chan;
	struct mhi_chan *dl_chan = mhi_dev->dl_chan;
	bool auto_start = false;
	int ret;

	/* bring device out of lpm */
	ret = mhi_device_get_sync(mhi_dev, MHI_VOTE_DEVICE);
	if (ret)
		return ret;

	ret = -EINVAL;
	if (ul_chan) {
		/* lpm notification require status_cb */
		if (ul_chan->lpm_notify && !mhi_drv->status_cb)
			goto exit_probe;

		if (!ul_chan->offload_ch && !mhi_drv->ul_xfer_cb)
			goto exit_probe;

		ul_chan->xfer_cb = mhi_drv->ul_xfer_cb;
		mhi_dev->status_cb = mhi_drv->status_cb;
		auto_start = ul_chan->auto_start;
	}

	if (dl_chan) {
		if (dl_chan->lpm_notify && !mhi_drv->status_cb)
			goto exit_probe;

		if (!dl_chan->offload_ch && !mhi_drv->dl_xfer_cb)
			goto exit_probe;

		mhi_event = &mhi_cntrl->mhi_event[dl_chan->er_index];

		/*
		 * if this channal event ring manage by client, then
		 * status_cb must be defined so we can send the async
		 * cb whenever there are pending data
		 */
		if (mhi_event->cl_manage && !mhi_drv->status_cb)
			goto exit_probe;

		dl_chan->xfer_cb = mhi_drv->dl_xfer_cb;

		/* ul & dl uses same status cb */
		mhi_dev->status_cb = mhi_drv->status_cb;
		auto_start = (auto_start || dl_chan->auto_start);
	}

	ret = mhi_drv->probe(mhi_dev, mhi_dev->id);

	if (!ret && auto_start)
		mhi_prepare_for_transfer(mhi_dev);

exit_probe:
	mhi_device_put(mhi_dev, MHI_VOTE_DEVICE);

	return ret;
}

/**
 * @brief remove driver
 *
 * @param[in ]            dev 					mhi device
 */
static int mhi_driver_remove(struct device *dev)
{
	struct mhi_device *mhi_dev = to_mhi_device(dev);
	struct mhi_driver *mhi_drv = to_mhi_driver(dev->driver);
	struct mhi_controller *mhi_cntrl = mhi_dev->mhi_cntrl;
	struct mhi_chan *mhi_chan;
	enum MHI_CH_STATE ch_state[] = {
		MHI_CH_STATE_DISABLED,
		MHI_CH_STATE_DISABLED
	};
	int dir;

	/* control device has no work to do */
	if (mhi_dev->dev_type == MHI_CONTROLLER_TYPE)
		return 0;

	MHI_LOG("Removing device for chan:%s\n", mhi_dev->chan_name);

	/* reset both channels */
	for (dir = 0; dir < 2; dir++) {
		mhi_chan = dir ? mhi_dev->ul_chan : mhi_dev->dl_chan;

		if (!mhi_chan)
			continue;

		/* wake all threads waiting for completion */
		write_lock_irq(&mhi_chan->lock);
		mhi_chan->ccs = MHI_EV_CC_INVALID;
		complete_all(&mhi_chan->completion);
		write_unlock_irq(&mhi_chan->lock);

		/* move channel state to disable, no more processing */
		mutex_lock(&mhi_chan->mutex);
		write_lock_irq(&mhi_chan->lock);
		ch_state[dir] = mhi_chan->ch_state;
		mhi_chan->ch_state = MHI_CH_STATE_SUSPENDED;
		write_unlock_irq(&mhi_chan->lock);

		/* reset the channel */
		if (!mhi_chan->offload_ch)
			mhi_reset_chan(mhi_cntrl, mhi_chan);

		mutex_unlock(&mhi_chan->mutex);
	}

	/* destroy the device */
	mhi_drv->remove(mhi_dev);

	/* de_init channel if it was enabled */
	for (dir = 0; dir < 2; dir++) {
		mhi_chan = dir ? mhi_dev->ul_chan : mhi_dev->dl_chan;

		if (!mhi_chan)
			continue;

		mutex_lock(&mhi_chan->mutex);

		if (ch_state[dir] == MHI_CH_STATE_ENABLED &&
		    !mhi_chan->offload_ch)
			mhi_deinit_chan_ctxt(mhi_cntrl, mhi_chan);

		mhi_chan->ch_state = MHI_CH_STATE_DISABLED;

		mutex_unlock(&mhi_chan->mutex);
	}


	if (mhi_cntrl->tsync_dev == mhi_dev)
		mhi_cntrl->tsync_dev = NULL;

	/* relinquish any pending votes for device */
	while (atomic_read(&mhi_dev->dev_vote))
		mhi_device_put(mhi_dev, MHI_VOTE_DEVICE);

	/* remove pending votes for the bus */
	while (atomic_read(&mhi_dev->bus_vote))
		mhi_device_put(mhi_dev, MHI_VOTE_BUS);

	return 0;
}

/**
 * @brief register driver
 *
 * @param[in ]            mhi_drv 				mhi driver
 */
int mhi_driver_register(struct mhi_driver *mhi_drv)
{
	struct device_driver *driver = &mhi_drv->driver;

	if (!mhi_drv->probe || !mhi_drv->remove)
		return -EINVAL;

	driver->bus = &mhi_bus_type;

	if (!driver->bus->p) 
		return -EINVAL;

	driver->probe = mhi_driver_probe;
	driver->remove = mhi_driver_remove;
	return driver_register(driver);
}
EXPORT_SYMBOL(mhi_driver_register);

/**
 * @brief unregister driver
 *
 * @param[in ]            mhi_drv 				mhi driver
 */
void mhi_driver_unregister(struct mhi_driver *mhi_drv)
{
	driver_unregister(&mhi_drv->driver);
}
EXPORT_SYMBOL(mhi_driver_unregister);

/**
 * @brief allocate buffer for device
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
struct mhi_device *mhi_alloc_device(struct mhi_controller *mhi_cntrl)
{
	struct mhi_device *mhi_dev = kzalloc(sizeof(*mhi_dev), GFP_KERNEL);
	struct device *dev;

	if (!mhi_dev)
		return NULL;

	dev = &mhi_dev->dev;
	device_initialize(dev);
	dev->bus = &mhi_bus_type;
	dev->release = mhi_release_device;
	dev->parent = mhi_cntrl->dev;
	mhi_dev->mhi_cntrl = mhi_cntrl;
	mhi_dev->dev_id = mhi_cntrl->dev_id;
	mhi_dev->domain = mhi_cntrl->domain;
	mhi_dev->bus = mhi_cntrl->bus;
	mhi_dev->slot = mhi_cntrl->slot;
	mhi_dev->mtu = MHI_MAX_MTU;
	atomic_set(&mhi_dev->dev_vote, 0);
	atomic_set(&mhi_dev->bus_vote, 0);

	return mhi_dev;
}

/**
 * @brief allocate DMA capable buffer
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 * @param[in ]            alignment 			alignment for the buffer
 */
void
mhi_malloc_contiguous_memory(struct mhi_controller *mhi_cntrl, u64 alignment)
{
	struct mhi_mem_props *mem_props = &mhi_cntrl->mem_props;
	dma_addr_t bam_PA = 0x0LL;

	mem_props->size = sizeof(struct mhi_ctrl_seg);
	mem_props->virt_unaligned = 0;
	mem_props->virt_aligned = 0;
	mem_props->phys_unaligned = 0;
	mem_props->phys_aligned = 0;
	mem_props->alignment = alignment;
    mem_props->handle = 0;

	mem_props->virt_unaligned =
	    (size_t) mhi_alloc_coherent(mhi_cntrl,
				     (mem_props->size + (size_t) alignment),
				     &bam_PA, GFP_KERNEL);

	if (mem_props->virt_unaligned == 0) {
		MHI_ERR("ERROR: Failed allocating uncahced contiguous memory!!");
		return;
    }

    mem_props->alloc_size = mem_props->size + (size_t) alignment;

    MHI_LOG("%llu + %llu allocated. MHI Ctrl VA: 0x%llx!",
            mem_props->size, alignment, mem_props->virt_unaligned);

    mem_props->phys_unaligned = bam_PA;
    mem_props->handle = bam_PA;

    mem_props->phys_aligned =
        ((mem_props->phys_unaligned + alignment) & ~alignment);

    mem_props->virt_aligned =
        ((mem_props->virt_unaligned + alignment) & ~alignment);

    MHI_LOG("MHI Control PA-A: 0x%llx, VA-A: 0x%llx!!",
            mem_props->phys_aligned, mem_props->virt_aligned);

    return;
}

#define MHI_ALIGN_4K_OFFSET          0xFFF

/**
 * @brief initialize the control segment
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
int mhi_init_ctrl_seq(struct mhi_controller *mhi_cntrl)
{
	mhi_malloc_contiguous_memory(mhi_cntrl, MHI_ALIGN_4K_OFFSET);

	MHI_LOG(
	       "MHI ctrl: VA-A: 0x%llx, PA-A: 0x%llx, Size: %lld bytes\n",
			mhi_cntrl->mem_props.virt_aligned,
			mhi_cntrl->mem_props.phys_aligned,
			mhi_cntrl->mem_props.size);

	if (mhi_cntrl->mem_props.virt_unaligned == 0)
		return -ENOMEM;

    mhi_cntrl->ctrl_seg = (struct mhi_ctrl_seg *)(size_t)mhi_cntrl->mem_props.virt_aligned;

	MHI_LOG("ctrl_seg 0x%p", mhi_cntrl->ctrl_seg);

	return 0;
}

/**
 * @brief uninitialize the control segment
 *
 * @param[in ]            mhi_cntrl 			mhi controller
 */
int mhi_deinit_ctrl_seq(struct mhi_controller *mhi_cntrl)
{
	struct mhi_mem_props *mem_props = &mhi_cntrl->mem_props;

    if (mhi_cntrl->ctrl_seg) {
	    
        MHI_LOG("free ctrl_seg 0x%p size %llu", mhi_cntrl->ctrl_seg, mem_props->alloc_size);
        
        mhi_free_coherent(mhi_cntrl,
				     mem_props->alloc_size,
				     (void *)(size_t)mem_props->virt_unaligned,
				     mem_props->handle);

        mhi_cntrl->ctrl_seg = NULL;    
    }

    return 0;
}

/**
 * @brief uninitialize mhi core
 */
int mhi_deinit(void)
{
	bus_unregister(&mhi_bus_type);
		
	mhi_dtr_deinit();

	return 0;
}

/**
 * @brief initialize mhi core
 */
int mhi_init(void)
{
	int ret = 0;

	mutex_init(&mhi_bus.lock);
	INIT_LIST_HEAD(&mhi_bus.controller_list);

#ifdef CONFIG_DEBUG_FS
	/* parent directory */
	debugfs_create_dir(mhi_bus_type.name, NULL);
#endif

	ret = bus_register(&mhi_bus_type);

	if (!ret)
		mhi_dtr_init();

	return ret;
}
