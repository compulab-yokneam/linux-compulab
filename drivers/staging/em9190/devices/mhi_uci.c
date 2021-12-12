/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include "../inc/ipc_logging.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ctype.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include <linux/usb.h>
#include <linux/hrtimer.h>
#include <linux/atomic.h>
#include <linux/usb/usbnet.h>
#include <linux/usb/cdc.h>
#include <linux/usb/cdc_ncm.h>
#include <net/ipv6.h>
#include <net/addrconf.h>
//#include <linux/hrtimer.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(5,6,0)
#include <linux/mod_devicetable.h>
#else
#include "../inc/devicetable.h"
#endif

#include "../inc/mhi.h"
#include "../core/mhi_internal.h"
#include "../inc/msm_mhi_dev.h"

#include "mhi_uci.h"
#include "qmap.h"

#ifdef CONFIG_MHI_DEBUG
enum MHI_DEBUG_LEVEL msg_lvl = MHI_MSG_LVL_INFO;
#endif 

bool debug = false;

module_param(debug, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug,"enable/disable driver logging");

int debug_level = MHI_MSG_LVL_INFO;
module_param(debug_level, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug_level,"driver logging level");

static DECLARE_BITMAP(uci_minors, MAX_UCI_DEVICES);
static struct mhi_uci_drv mhi_uci_drv;

/**
 * @brief queue read requests for incoming messages
 *
 * @param[in ]            uci_dev				struct uci_dev	
 */
static int mhi_queue_inbound(struct uci_dev *uci_dev)
{
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	int nr_trbs = mhi_get_no_free_descriptors(mhi_dev, DMA_FROM_DEVICE);
	size_t mtu = uci_dev->mtu;
	void *buf;
	struct uci_buf *uci_buf;
	int ret = -EIO, i;

	MSG_LOG("nr_trbs %d mtu %ld\n", nr_trbs, (long)mtu);

	for (i = 0; i < nr_trbs; i++) {
		buf = kmalloc(mtu + sizeof(*uci_buf), GFP_KERNEL);
		if (!buf) {
	        MSG_ERR("failed to allocate memory %ld\n", (unsigned long)(mtu + sizeof(*uci_buf)));
			return -ENOMEM;
        }

		uci_buf = buf + mtu;
		uci_buf->data = buf;

		//MSG_VERB("Allocated buf %d of %d size %u\n", i, nr_trbs, mtu);
		MSG_LOG("Allocated buf %d of %d size %lu\n", i, nr_trbs, (unsigned long)mtu);

		ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE, buf, mtu,
					 MHI_EOT);
	    
        MSG_LOG("mhi_queue_transfer i %d ret 0x%x\n", i, ret);
		
        if (ret) {
			kfree(buf);
			MSG_ERR("Failed to queue buffer %d\n", i);
			return ret;
		}
	}

	return ret;
}

/**
 * @brief device ioctl handler
 *
 * @param[in ]            file				struct file
 * @param[in ]            cmd				command
 * @param[in ]            arg				argument
 */
static long mhi_uci_ioctl(struct file *file,
			  unsigned int cmd,
			  unsigned long arg)
{
	struct uci_dev *uci_dev = file->private_data;
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	long ret = -ERESTARTSYS;

	mutex_lock(&uci_dev->mutex);
	if (uci_dev->enabled)
		ret = mhi_ioctl(mhi_dev, cmd, arg);
	mutex_unlock(&uci_dev->mutex);

	return ret;
}

/**
 * @brief release the char device
 *
 * @param[in ]            inode				struct inode
 * @param[in ]            file				struct file
 */
static int mhi_uci_release(struct inode *inode, struct file *file)
{
	struct uci_dev *uci_dev = file->private_data;

	mutex_lock(&uci_dev->mutex);
	uci_dev->ref_count--;
	if (!uci_dev->ref_count) {
		struct uci_buf *itr, *tmp;
		struct uci_chan *uci_chan;

		MSG_LOG("Last client left, closing node\n");

		if (uci_dev->enabled)
			mhi_unprepare_from_transfer(uci_dev->mhi_dev);

		/* clean inbound channel */
		uci_chan = &uci_dev->dl_chan;
		list_for_each_entry_safe(itr, tmp, &uci_chan->pending, node) {
			list_del(&itr->node);
			kfree(itr->data);
		}
		if (uci_chan->cur_buf)
			kfree(uci_chan->cur_buf->data);

		uci_chan->cur_buf = NULL;

		if (!uci_dev->enabled) {
			MSG_LOG("Node is deleted, freeing dev node\n");
			mutex_unlock(&uci_dev->mutex);
			mutex_destroy(&uci_dev->mutex);
			clear_bit(MINOR(uci_dev->devt), uci_minors);
			kfree(uci_dev);
			return 0;
		}
	}

	MSG_LOG("exit: ref_count:%d\n", uci_dev->ref_count);

	mutex_unlock(&uci_dev->mutex);

	return 0;
}

/**
 * @brief poll the char device
 *
 * @param[in ]            file				struct file
 * @param[in ]            wait				poll_table
 */
static unsigned int mhi_uci_poll(struct file *file, poll_table *wait)
{
	struct uci_dev *uci_dev = file->private_data;
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	struct uci_chan *uci_chan;
	unsigned int mask = 0;

	poll_wait(file, &uci_dev->dl_chan.wq, wait);
	poll_wait(file, &uci_dev->ul_chan.wq, wait);

	uci_chan = &uci_dev->dl_chan;
	spin_lock_bh(&uci_chan->lock);
	if (!uci_dev->enabled) {
		mask = POLLERR;
	} else if (!list_empty(&uci_chan->pending) || uci_chan->cur_buf) {
		MSG_VERB("Client can read from node\n");
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_bh(&uci_chan->lock);

	uci_chan = &uci_dev->ul_chan;
	spin_lock_bh(&uci_chan->lock);
	if (!uci_dev->enabled) {
		mask |= POLLERR;
	} else if (mhi_get_no_free_descriptors(mhi_dev, DMA_TO_DEVICE) > 0) {
		MSG_VERB("Client can write to node\n");
		mask |= POLLOUT | POLLWRNORM;
	}
	spin_unlock_bh(&uci_chan->lock);

	MSG_LOG("Client attempted to poll, returning mask 0x%x\n", mask);

	return mask;
}

/**
 * @brief write to the char device
 *
 * @param[in ]            file				struct file
 * @param[in ]            buf				buffer
 * @param[in ]            count				buffer size
 * @param[in ]            offp				offset
 */
static ssize_t mhi_uci_write_single(struct file *file,
							 const char __user *buf,
							 size_t count,
							 loff_t *offp)
{
	struct uci_dev *uci_dev = file->private_data;
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	struct uci_chan *uci_chan = &uci_dev->ul_chan;
	size_t bytes_xfered = 0;
	int ret, nr_avail;

	if (!buf || !count)
		return -EINVAL;

	/* confirm channel is active */
	spin_lock_bh(&uci_chan->lock);
	if (!uci_dev->enabled) {
		spin_unlock_bh(&uci_chan->lock);
		return -ERESTARTSYS;
	}

	MSG_LOG("Enter: to xfer:%lu bytes\n", (unsigned long)count);
	MSG_VERB("Enter: to xfer:%lu bytes\n", count);

	while (count) {
		size_t xfer_size;
		void *kbuf;
		enum MHI_FLAGS flags;

		spin_unlock_bh(&uci_chan->lock);

		/* wait for free descriptors */
		ret = wait_event_interruptible(uci_chan->wq,
			(!uci_dev->enabled) ||
			(nr_avail = mhi_get_no_free_descriptors(mhi_dev,
							DMA_TO_DEVICE)) > 0);

		if (ret == -ERESTARTSYS || !uci_dev->enabled) {
			MSG_LOG("Exit signal caught for node or not enabled\n");
			return -ERESTARTSYS;
		}

		xfer_size = min_t(size_t, count, uci_dev->mtu);
	    MSG_LOG("xfer_size:%lu bytes\n", (unsigned long)xfer_size);
		kbuf = kmalloc(xfer_size, GFP_KERNEL);
		if (!kbuf) {
			MSG_ERR("Failed to allocate memory %lu\n", (unsigned long)xfer_size);
			return -ENOMEM;
		}

		ret = copy_from_user(kbuf, buf, xfer_size);
		if (unlikely(ret)) {
			kfree(kbuf);
			return ret;
		}

		spin_lock_bh(&uci_chan->lock);

		/* if ring is full after this force EOT */
		if (nr_avail > 1 && (count - xfer_size))
			flags = MHI_CHAIN;
		else
			flags = MHI_EOT;

		if (uci_dev->enabled) {
#if 0
	        MSG_LOG("write buffer\n");
	        print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
		        kbuf, xfer_size, true);
#endif
			ret = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, kbuf,
						 xfer_size, flags);
	        MSG_LOG("mhi_queue_transfer flags %d ret 0x%x\n", flags, ret);
        }                         
		else
			ret = -ERESTARTSYS;

		if (ret) {
			kfree(kbuf);
			goto sys_interrupt;
		}

		bytes_xfered += xfer_size;
		count -= xfer_size;
		buf += xfer_size;
	}

	spin_unlock_bh(&uci_chan->lock);
	MSG_LOG("Exit: Number of bytes xferred:%lu\n", (unsigned long)bytes_xfered);
	MSG_VERB("Exit: Number of bytes xferred:%lu\n", bytes_xfered);

	return bytes_xfered;

sys_interrupt:
	spin_unlock_bh(&uci_chan->lock);

	return ret;
}

/**
 * @brief write to the char device
 *
 * @param[in ]            file				struct file
 * @param[in ]            buf				buffer
 * @param[in ]            count				buffer size
 * @param[in ]            offp				offset
 */
static ssize_t mhi_uci_write(struct file *file,
							 const char __user *buf,
							 size_t count,
							 loff_t *offp)
{
	int ret;
	
#ifdef ADB_SUPPORT	

	struct uci_dev *uci_dev = file->private_data;
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	int ret1;

	/* for ADB UL channle, need to send ADB header and payload separately */
	if ((mhi_dev->ul_chan_id == MHI_CLIENT_ADB_OUT) && (count > ADB_HDR_LEN)) {
		ret = mhi_uci_write_single(file, buf, ADB_HDR_LEN, offp);
		if (ret < 0)
			return ret;

		ret1 = mhi_uci_write_single(file, &buf[ADB_HDR_LEN], count - ADB_HDR_LEN, offp);
		if (ret1 < 0)
			return ret1;

		return (ret + ret1);
	}		
#endif

	ret = mhi_uci_write_single(file, buf, count, offp);

	return ret;
}

/**
 * @brief read from the char device
 *
 * @param[in ]            file				struct file
 * @param[in ]            buf				buffer
 * @param[in ]            count				buffer size
 * @param[in ]            ppos				offset
 */
static ssize_t mhi_uci_read(struct file *file,
			    char __user *buf,
			    size_t count,
			    loff_t *ppos)
{
	struct uci_dev *uci_dev = file->private_data;
	struct mhi_device *mhi_dev = uci_dev->mhi_dev;
	struct uci_chan *uci_chan = &uci_dev->dl_chan;
	struct uci_buf *uci_buf;
	char *ptr;
	size_t to_copy;
	int ret = 0;

	if (!buf)
		return -EINVAL;

	MSG_VERB("Client provided buf len:%lu\n", count);

	/* confirm channel is active */
	spin_lock_bh(&uci_chan->lock);
	if (!uci_dev->enabled) {
		spin_unlock_bh(&uci_chan->lock);
		return -ERESTARTSYS;
	}

	/* No data available to read, wait */
	if (!uci_chan->cur_buf && list_empty(&uci_chan->pending)) {
		MSG_VERB("No data available to read waiting\n");

		spin_unlock_bh(&uci_chan->lock);
		ret = wait_event_interruptible(uci_chan->wq,
				(!uci_dev->enabled ||
				 !list_empty(&uci_chan->pending)));
		if (ret == -ERESTARTSYS) {
			MSG_LOG("Exit signal caught for node\n");
			return -ERESTARTSYS;
		}

		spin_lock_bh(&uci_chan->lock);
		if (!uci_dev->enabled) {
			MSG_LOG("node is disabled\n");
			ret = -ERESTARTSYS;
			goto read_error;
		}
	}

	/* new read, get the next descriptor from the list */
	if (!uci_chan->cur_buf) {
		uci_buf = list_first_entry_or_null(&uci_chan->pending,
						   struct uci_buf, node);
		if (unlikely(!uci_buf)) {
			ret = -EIO;
			goto read_error;
		}

		list_del(&uci_buf->node);
		uci_chan->cur_buf = uci_buf;
		uci_chan->rx_size = uci_buf->len;
		uci_buf->sent = false;
		MSG_VERB("Got pkt of size:%zu\n", uci_chan->rx_size);
	}

	uci_buf = uci_chan->cur_buf;
	spin_unlock_bh(&uci_chan->lock);

	mutex_lock(&uci_chan->mutex_read);

	/* Copy the buffer to user space */
	to_copy = min_t(size_t, count, uci_chan->rx_size);
	ptr = uci_buf->data + (uci_buf->len - uci_chan->rx_size);
	ret = copy_to_user(buf, ptr, to_copy);
	if (ret) {
		mutex_unlock(&uci_chan->mutex_read);
		return ret;
	}

	MSG_VERB("Copied %lu of %lu bytes\n", to_copy, uci_chan->rx_size);
	uci_chan->rx_size -= to_copy;

	mutex_unlock(&uci_chan->mutex_read);

	/* we finished with this buffer, queue it back to hardware */
	if (!uci_chan->rx_size) {
		spin_lock_bh(&uci_chan->lock);
		uci_chan->cur_buf = NULL;

		if (!uci_buf->sent) { 

			uci_buf->sent = true;
			
			if (uci_dev->enabled) {

				ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE,
					uci_buf->data, uci_dev->mtu,
					MHI_EOT);

				
				MSG_LOG("buf 0x%px ret 0x%x\n", uci_buf->data, ret);
			}
			else {
				ret = -ERESTARTSYS;
				MSG_LOG("device disabled! ret 0x%x\n", ret);
			}

			if (ret) {
				MSG_ERR("Failed to recycle element\n");
				kfree(uci_buf->data);
				goto read_error;
			}
		}

		spin_unlock_bh(&uci_chan->lock);
	}

#if 0
    MSG_LOG("read buffer\n");
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
	    ptr, to_copy, true);
#endif

	MSG_VERB("Returning %lu bytes\n", to_copy);

	return to_copy;

read_error:
	spin_unlock_bh(&uci_chan->lock);

	return ret;
}

/**
 * @brief open the char device
 *
 * @param[in ]            inode				struct inode
 * @param[in ]            file				struct file
 */
static int mhi_uci_open(struct inode *inode, struct file *filp)
{
	struct uci_dev *uci_dev = NULL, *tmp_dev;
	int ret = -EIO;
	struct uci_buf *buf_itr, *tmp;
	struct uci_chan *dl_chan;

	mutex_lock(&mhi_uci_drv.lock);
	list_for_each_entry(tmp_dev, &mhi_uci_drv.head, node) {
		if (tmp_dev->devt == inode->i_rdev) {
			uci_dev = tmp_dev;
		    MSG_LOG("found device!\n");
			break;
		}
	}

	/* could not find a minor node */
	if (!uci_dev) {
		MSG_ERR("cannot find device!\n");
		goto error_exit;
    }

	mutex_lock(&uci_dev->mutex);
	if (!uci_dev->enabled) {
		MSG_ERR("Node exist, but not in active state!\n");
		goto error_open_chan;
	}

	uci_dev->ref_count++;

	MSG_LOG("Node open, ref counts %u\n", uci_dev->ref_count);

	if (uci_dev->ref_count == 1) {
		MSG_LOG("Starting channel\n");
		ret = mhi_prepare_for_transfer(uci_dev->mhi_dev);
		if (ret) {
			MSG_ERR("Error starting transfer channels\n");
			uci_dev->ref_count--;
			goto error_open_chan;
		}

		MSG_LOG("mhi_queue_inbound\n");
		ret = mhi_queue_inbound(uci_dev);
		if (ret)
			goto error_rx_queue;
	}

	filp->private_data = uci_dev;
	mutex_unlock(&uci_dev->mutex);
	mutex_unlock(&mhi_uci_drv.lock);

	return 0;

 error_rx_queue:
	dl_chan = &uci_dev->dl_chan;
	mhi_unprepare_from_transfer(uci_dev->mhi_dev);
	list_for_each_entry_safe(buf_itr, tmp, &dl_chan->pending, node) {
		list_del(&buf_itr->node);
		kfree(buf_itr->data);
	}

 error_open_chan:
	mutex_unlock(&uci_dev->mutex);

error_exit:
	mutex_unlock(&mhi_uci_drv.lock);

	return ret;
}

static const struct file_operations mhidev_fops = {
	.owner = THIS_MODULE,
	.open = mhi_uci_open,
	.release = mhi_uci_release,
	.read = mhi_uci_read,
	.write = mhi_uci_write,
	.poll = mhi_uci_poll,
	.unlocked_ioctl = mhi_uci_ioctl,
};

/**
 * @brief remove the char device
 *
 * @param[in ]            mhi_dev			struct mhi_device
 */
static void mhi_uci_remove(struct mhi_device *mhi_dev)
{
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);

	MSG_DEFAULT("Enter\n");

	mutex_lock(&mhi_uci_drv.lock);
	mutex_lock(&uci_dev->mutex);

	/* disable the node */
	spin_lock_irq(&uci_dev->dl_chan.lock);
	spin_lock_irq(&uci_dev->ul_chan.lock);
	uci_dev->enabled = false;
	spin_unlock_irq(&uci_dev->ul_chan.lock);
	spin_unlock_irq(&uci_dev->dl_chan.lock);
	wake_up(&uci_dev->dl_chan.wq);
	wake_up(&uci_dev->ul_chan.wq);

	/* delete the node to prevent new opens */
	device_destroy(mhi_uci_drv.class, uci_dev->devt);
	uci_dev->dev = NULL;
	list_del(&uci_dev->node);

	/* safe to free memory only if all file nodes are closed */
	MSG_LOG("ref_count %d\n", uci_dev->ref_count);
	if (!uci_dev->ref_count) {
		mutex_unlock(&uci_dev->mutex);
		mutex_destroy(&uci_dev->mutex);
		clear_bit(MINOR(uci_dev->devt), uci_minors);
		kfree(uci_dev);
		mutex_unlock(&mhi_uci_drv.lock);
		return;
	}

	mutex_unlock(&uci_dev->mutex);
	mutex_unlock(&mhi_uci_drv.lock);

	MSG_DEFAULT("Exit\n");
}

/**
 * @brief probe the char device
 *
 * @param[in ]            mhi_dev			struct mhi_device
 * @param[in ]            id				struct mhi_device_id
 */
static int mhi_uci_probe(struct mhi_device *mhi_dev,
			 const struct mhi_device_id *id)
{
	struct uci_dev *uci_dev;
	int minor;
	char node_name[32];
	int dir;

    MSG_DEFAULT("enter\n");

	uci_dev = kzalloc(sizeof(*uci_dev), GFP_KERNEL);
	if (!uci_dev)
		return -ENOMEM;

	mutex_init(&uci_dev->mutex);
	uci_dev->mhi_dev = mhi_dev;

	minor = find_first_zero_bit(uci_minors, MAX_UCI_DEVICES);
	if (minor >= MAX_UCI_DEVICES) {
		kfree(uci_dev);
		return -ENOSPC;
	}

	mutex_lock(&uci_dev->mutex);
	mutex_lock(&mhi_uci_drv.lock);

	uci_dev->devt = MKDEV(mhi_uci_drv.major, minor);
	uci_dev->dev = device_create(mhi_uci_drv.class, &mhi_dev->dev,
				     uci_dev->devt, uci_dev,
				     DEVICE_NAME "_%04x_%02u.%02u.%02u%s%d",
				     mhi_dev->dev_id, mhi_dev->domain,
				     mhi_dev->bus, mhi_dev->slot, "_pipe_",
				     mhi_dev->ul_chan_id);
	set_bit(minor, uci_minors);

	/* create debugging buffer */
	snprintf(node_name, sizeof(node_name), "mhi_uci_%04x_%02u.%02u.%02u_%d",
		 mhi_dev->dev_id, mhi_dev->domain, mhi_dev->bus, mhi_dev->slot,
		 mhi_dev->ul_chan_id);
	uci_dev->ipc_log = ipc_log_context_create(MHI_UCI_IPC_LOG_PAGES,
						  node_name, 0);

	for (dir = 0; dir < 2; dir++) {
		struct uci_chan *uci_chan = (dir) ?
			&uci_dev->ul_chan : &uci_dev->dl_chan;
		spin_lock_init(&uci_chan->lock);
		init_waitqueue_head(&uci_chan->wq);
		INIT_LIST_HEAD(&uci_chan->pending);
		mutex_init(&uci_chan->mutex_read);
	};

	uci_dev->mtu = min_t(size_t, id->driver_data, mhi_dev->mtu);
	mhi_device_set_devdata(mhi_dev, uci_dev);
	uci_dev->enabled = true;

	list_add(&uci_dev->node, &mhi_uci_drv.head);
	mutex_unlock(&mhi_uci_drv.lock);
	mutex_unlock(&uci_dev->mutex);

	MSG_DEFAULT("channel:%s successfully probed\n", mhi_dev->chan_name);

	return 0;
};

/**
 * @brief host to device transfer callback
 *
 * @param[in ]            mhi_dev			struct mhi_device
 * @param[in ]            mhi_result		struct mhi_result
 */
static void mhi_ul_xfer_cb(struct mhi_device *mhi_dev,
			   struct mhi_result *mhi_result)
{
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);
	struct uci_chan *uci_chan = &uci_dev->ul_chan;

	MSG_VERB("status:%d xfer_len:%zu\n", mhi_result->transaction_status,
		 mhi_result->bytes_xferd);

	kfree(mhi_result->buf_addr);
	if (!mhi_result->transaction_status)
		wake_up(&uci_chan->wq);
}

/**
 * @brief device to host transfer callback
 *
 * @param[in ]            mhi_dev			struct mhi_device
 * @param[in ]            mhi_result		struct mhi_result
 */
static void mhi_dl_xfer_cb(struct mhi_device *mhi_dev,
			   struct mhi_result *mhi_result)
{
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);
	struct uci_chan *uci_chan = &uci_dev->dl_chan;
	unsigned long flags;
	struct uci_buf *buf;
	bool ignore;
	int ret = 0;

	MSG_VERB("status:%d receive_len:%zu\n", mhi_result->transaction_status,
		 mhi_result->bytes_xferd);

	if (mhi_result->transaction_status == -ENOTCONN) {
		kfree(mhi_result->buf_addr);
		return;
	}

	MSG_LOG("chan %d\n", mhi_dev->dl_chan->chan);

	buf = mhi_result->buf_addr + uci_dev->mtu;
	buf->data = mhi_result->buf_addr;
	buf->len = mhi_result->bytes_xferd;

	ignore = mbim_rx_cid(mhi_dev, buf->data, buf->len);

	if (ignore) {
		spin_lock_irqsave(&uci_chan->lock, flags);

		if (uci_dev->enabled) {

			ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE,
				buf->data, uci_dev->mtu,
				MHI_EOT);

				
			MSG_LOG("buf 0x%px ret 0x%x\n", buf->data, ret);

		} else {
			MSG_LOG("device disabled!\n");
			ret = -ERESTARTSYS;
		}

		if (ret) {
			MSG_ERR("Failed to recycle element\n");
			kfree(buf->data);
		}	
		
		spin_unlock_irqrestore(&uci_chan->lock, flags);

		return;
	}

	spin_lock_irqsave(&uci_chan->lock, flags);
	list_add_tail(&buf->node, &uci_chan->pending);
	spin_unlock_irqrestore(&uci_chan->lock, flags);

	if (mhi_dev->dev.power.wakeup)
		__pm_wakeup_event(mhi_dev->dev.power.wakeup, 0);

	wake_up(&uci_chan->wq);
}

/* .driver_data stores max mtu */
static const struct mhi_device_id mhi_uci_match_table[] = {
	{.chan = "LOOPBACK", .driver_data = 0x1000},
	{.chan = "EFS", .driver_data = 0x1000},
	{.chan = "MBIM", .driver_data = 0x1000},
	{.chan = "QMI0", .driver_data = 0x1000},
	{.chan = "QMI1", .driver_data = 0x1000},
#ifdef ADB_SUPPORT
	{.chan = "ADB", .driver_data = 0x1000},
#endif	
	{.chan = "TF", .driver_data = 0x1000},
	{},
};
MODULE_DEVICE_TABLE(mhi, mhi_uci_match_table);

static struct mhi_driver mhi_uci_driver = {
	.id_table = mhi_uci_match_table,
	.remove = mhi_uci_remove,
	.probe = mhi_uci_probe,
	.ul_xfer_cb = mhi_ul_xfer_cb,
	.dl_xfer_cb = mhi_dl_xfer_cb,
	.driver = {
		.name = MHI_UCI_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

/**
 * @brief module init
 */
static int mhi_uci_init(void)
{
	int ret;

    MSG_DEFAULT("enter\n");

	ret = register_chrdev(0, MHI_UCI_DRIVER_NAME, &mhidev_fops);
	if (ret < 0)
		return ret;

	mhi_uci_drv.major = ret;
	mhi_uci_drv.class = class_create(THIS_MODULE, MHI_UCI_DRIVER_NAME);
	if (IS_ERR(mhi_uci_drv.class)) {
		ret = -ENODEV;
		goto exit;
	}

	mutex_init(&mhi_uci_drv.lock);
	INIT_LIST_HEAD(&mhi_uci_drv.head);

	ret = mhi_driver_register(&mhi_uci_driver);
	if (ret)
		class_destroy(mhi_uci_drv.class);

	return ret;

exit:
	unregister_chrdev(mhi_uci_drv.major, MHI_UCI_DRIVER_NAME);

	return ret;
}

/**
 * @brief module exit
 */
static void __exit mhi_uci_exit(void)
{
    MSG_DEFAULT("enter\n");

	unregister_chrdev(mhi_uci_drv.major, MHI_UCI_DRIVER_NAME);

    mhi_driver_unregister(&mhi_uci_driver);

	class_destroy(mhi_uci_drv.class);
}

module_init(mhi_uci_init);
module_exit(mhi_uci_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_UCI");
MODULE_DESCRIPTION("MHI UCI Driver");
MODULE_VERSION("1.9.2101.1");