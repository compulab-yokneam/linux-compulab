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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>          /* struct iphdr */
#include <linux/tcp.h>         /* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/mii.h>
#include <linux/in6.h>
#include <linux/if_vlan.h>
#include "../inc/msm_rmnet.h"
#include <linux/if_arp.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include "../inc/ipc_logging.h"
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/rtnetlink.h>
#include <linux/version.h>
#include <linux/ipv6.h>
#include <net/addrconf.h>
#include <linux/hrtimer.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>
#include <linux/usb/cdc.h>
#include <linux/usb/cdc_ncm.h>
#include <linux/time64.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(5,6,0)
#include <linux/mod_devicetable.h>
#else
#include "../inc/devicetable.h"
#endif

#include "../inc/mhi.h"

#include "qmap.h"

#define MHI_NETDEV_DRIVER_NAME "mhi_netdev"
#define WATCHDOG_TIMEOUT (30 * HZ)
#define IPC_LOG_PAGES (100)
#define MAX_NETBUF_SIZE (128)

#define HEADER_RESERVE 32

enum data_dir {
	RX_DATA	= 0,
	TX_DATA	= 1
};

#define DROP_PKT_IF_RING_FULL	0

 /* alternative VLAN for IP session 0 if not untagged */
#define MBIM_IPS0_VID	4094

/* flags for the cdc_mbim_state.flags field */
enum cdc_mbim_flags {
	FLAG_IPS0_VLAN = 1 << 0,	/* IP session 0 is tagged  */
};

/* mac address */
u8 mac_addr[6] = { 0x00, 0xa0, 0xd5, 0xff, 0xf0, 0xb0 };

bool debug = false;

module_param(debug, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug,"enable/disable driver logging");

int debug_level = MHI_MSG_LVL_INFO;
module_param(debug_level, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug_level,"driver logging level");

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)

#ifndef ETH_P_MAP
#define ETH_P_MAP 0xDA1A
#endif

#ifndef ARPHRD_RAWIP
#define ARPHRD_RAWIP    519		/* Raw IP                       */
#endif

static inline void *skb_put_data(struct sk_buff *skb, const void *data,
				 unsigned int len)
{
	void *tmp = skb_put(skb, len);

	memcpy(tmp, data, len);

	return tmp;
}

static inline int page_ref_count(struct page *page)
{
	return atomic_read(&page->_count);
}

#define page_ref_tracepoint_active(t) false

static inline void __page_ref_mod(struct page *page, int v)
{
}

static inline void page_ref_inc(struct page *page)
{
	atomic_inc(&page->_count);
	if (page_ref_tracepoint_active(__tracepoint_page_ref_mod))
		__page_ref_mod(page, 1);
}

static inline void page_ref_dec(struct page *page)
{
	atomic_dec(&page->_count);
	if (page_ref_tracepoint_active(__tracepoint_page_ref_mod))
		__page_ref_mod(page, -1);
}
#endif

#ifdef CONFIG_MHI_DEBUG

#define IPC_LOG_LVL (MHI_MSG_LVL_VERBOSE)

#define MHI_ASSERT(cond, msg) do { \
	if (cond) \
		panic(msg); \
} while (0)

#define MSG_VERB(fmt, ...) do { \
	if (mhi_netdev->msg_lvl <= MHI_MSG_LVL_VERBOSE) \
		pr_err("[D][%s] " fmt, __func__, ##__VA_ARGS__);\
	if (mhi_netdev->ipc_log && (mhi_netdev->ipc_log_lvl <= \
				    MHI_MSG_LVL_VERBOSE)) \
		ipc_log_string(mhi_netdev->ipc_log, "[D][%s] " fmt, \
			       __func__, ##__VA_ARGS__); \
} while (0)

#else

#define IPC_LOG_LVL (MHI_MSG_LVL_ERROR)

#define MHI_ASSERT(cond, msg) do { \
	if (cond) { \
		MSG_ERR(msg); \
		WARN_ON(cond); \
	} \
} while (0)

#define MSG_VERB(fmt, ...)

#endif

extern bool debug;
extern int debug_level;

#define MSG_LOG(fmt, ...) do { \
	if (debug && debug_level <= MHI_MSG_LVL_INFO) \
		pr_err("[I][%s] " fmt, __func__, ##__VA_ARGS__);\
	if (mhi_netdev->ipc_log && (mhi_netdev->ipc_log_lvl <= \
				    MHI_MSG_LVL_INFO)) \
		ipc_log_string(mhi_netdev->ipc_log, "[I][%s] " fmt, \
			       __func__, ##__VA_ARGS__); \
} while (0)

#define MSG_ERR(fmt, ...) do { \
	if (debug && debug_level <= MHI_MSG_LVL_ERROR) \
		pr_err("[E][%s] " fmt, __func__, ##__VA_ARGS__); \
	if (mhi_netdev->ipc_log && (mhi_netdev->ipc_log_lvl <= \
				    MHI_MSG_LVL_ERROR)) \
		ipc_log_string(mhi_netdev->ipc_log, "[E][%s] " fmt, \
			       __func__, ##__VA_ARGS__); \
} while (0)

#define DEFAULT 1

#if DEFAULT
#define MSG_DEFAULT(fmt, ...) do { \
		pr_info("[D][%s] " fmt, __func__, ##__VA_ARGS__);\
} while (0)
#else
#define MSG_DEFAULT(fmt, ...)
#endif

struct mhi_net_chain {
	struct sk_buff *head, *tail; /* chained skb */
};

struct mhi_netdev {
	int alias;
	struct mhi_device *mhi_dev;
	struct mhi_netdev *rsc_dev; /* rsc linked node */
	bool is_rsc_dev;
	int wake;

	u32 mru;
	u32 rx_order;
	u32 mtu;
	u32 tx_order;
	const char *interface_name;
	struct napi_struct *napi;
	struct net_device *ndev;

	/* for rx */
	struct mhi_netbuf **rx_netbuf_pool;
	int rx_pool_size; /* must be power of 2 */
	int rx_current_index;
	bool rx_chain_skb;
	struct mhi_net_chain *rx_chain;

	/* for tx */
	struct mhi_netbuf **tx_netbuf_pool;
	int tx_pool_size; /* must be power of 2 */
	int tx_current_index;
	bool tx_chain_skb;
	struct mhi_net_chain *tx_chain;

	struct mhi_netbuf *pending_netbuf;
	u16 tx_offset;
	u16 tx_num_pkt;

	spinlock_t tx_lock;
	struct hrtimer tx_timer;
	struct tasklet_struct bh;

	u32 timer_interval;
	u16 tx_timer_pending;

	atomic_t stop;

	struct dentry *dentry;
	enum MHI_DEBUG_LEVEL msg_lvl;
	enum MHI_DEBUG_LEVEL ipc_log_lvl;
	void *ipc_log;

	unsigned long flags;

#ifdef XDP_SUPPORT
	struct bpf_prog *xdp_prog;
#endif	
};

struct mhi_netdev_priv {
	struct mhi_netdev *mhi_netdev;
};

/* Try not to make this structure bigger than 128 bytes, since this take space
 * in payload packet.
 * Example: If MRU = 16K, effective MRU = 16K - sizeof(mhi_netbuf)
 */
struct mhi_netbuf {
	struct mhi_buf mhi_buf; /* this must be first element */
	void (*unmap)(struct device *dev, dma_addr_t addr, size_t size,
		      enum dma_data_direction dir);
};

static struct mhi_driver mhi_netdev_driver;
static void mhi_netdev_create_debugfs(struct mhi_netdev *mhi_netdev);

#if 0
/**
 * @brief map data type to protocol
 *
 * @param[in ]            data				data type
 */
static __be16 mhi_netdev_ip_type_trans(u8 data)
{
	__be16 protocol = 0;

	/* determine L3 protocol */
	switch (data & 0xf0) {
	case 0x40:
		protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		protocol = htons(ETH_P_IPV6);
		break;
	default:
		/* default is QMAP */
		protocol = htons(ETH_P_MAP);
		break;
	}
	return protocol;
}
#endif

/**
 * @brief allocate and map pages for DMA 
 *
 * @param[in ]            dev				struct device
 * @param[in ]            gfp				flag
 * @param[in ]            order				page order
 */
static struct mhi_netbuf *mhi_netdev_alloc(struct device *dev,
					   gfp_t gfp,
					   unsigned int order,
					   enum data_dir dir)
{
	struct page *page;
	struct mhi_netbuf *netbuf;
	struct mhi_buf *mhi_buf;
	void *vaddr;

	if (dir == RX_DATA)
		page = __dev_alloc_pages(gfp, order);
	else 
		page = alloc_pages(gfp, order);

	if (!page) {
		return NULL;
	}

	vaddr = page_address(page);

	/* we going to use the end of page to store cached data */
	netbuf = vaddr + (PAGE_SIZE << order) - sizeof(*netbuf);

	mhi_buf = (struct mhi_buf *)&netbuf->mhi_buf;
	mhi_buf->page = page;
	mhi_buf->buf = vaddr;
	mhi_buf->len = (void *)netbuf - vaddr;

	if (dir == RX_DATA) {

		mhi_buf->dma_addr = dma_map_page(dev, page, 0, mhi_buf->len,
										 DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, mhi_buf->dma_addr))
		{
			__free_pages(mhi_buf->page, order);
			return NULL;
		}
	}

	return netbuf;
}

/**
 * @brief unmap pages for DMA
 *
 * @param[in ]            dev				struct device
 * @param[in ]            dma_addr			dma address
 * @param[in ]            len				dma buffer size
 * @param[in ]            dir				dma direction
 */
static void mhi_netdev_unmap_page(struct device *dev,
				  dma_addr_t dma_addr,
				  size_t len,
				  enum dma_data_direction dir)
{
	dma_unmap_page(dev, dma_addr, len, dir);
}

/**
 * @brief queue data read requests
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 * @param[in ]            nr_tre			number of requests
 */
static int mhi_netdev_tmp_alloc(struct mhi_netdev *mhi_netdev, int nr_tre)
{
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	struct device *dev = mhi_dev->dev.parent;
	const u32 order = mhi_netdev->rx_order;
	int i, ret;

	MSG_LOG("Enter free_desc:%d\n", nr_tre);

	for (i = 0; i < nr_tre; i++) {
		struct mhi_buf *mhi_buf;
		struct mhi_netbuf *netbuf = mhi_netdev_alloc(dev, GFP_ATOMIC,
							     order, RX_DATA);
		if (!netbuf)
			return -ENOMEM;

		mhi_buf = (struct mhi_buf *)netbuf;
		netbuf->unmap = mhi_netdev_unmap_page;

		ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE, mhi_buf,
					 mhi_buf->len, MHI_EOT);
		if (unlikely(ret)) {
			MSG_ERR("Failed to queue transfer, ret:%d\n", ret);
			mhi_netdev_unmap_page(dev, mhi_buf->dma_addr,
					      mhi_buf->len, DMA_FROM_DEVICE);
			__free_pages(mhi_buf->page, order);
			return ret;
		}
	}

	return 0;
}

/**
 * @brief queue data read requests
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static void mhi_netdev_queue(struct mhi_netdev *mhi_netdev)
{
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	struct device *dev = mhi_dev->dev.parent;
	struct mhi_netbuf *netbuf;
	struct mhi_buf *mhi_buf = NULL;
	struct mhi_netbuf **netbuf_pool = mhi_netdev->rx_netbuf_pool;
	int nr_tre = mhi_get_no_free_descriptors(mhi_dev, DMA_FROM_DEVICE);
	int i, peak, cur_index = 0, ret;
	const int pool_size = mhi_netdev->rx_pool_size - 1, max_peak = 4;

	MSG_VERB("Enter free_desc:%d\n", nr_tre);
	MSG_LOG("Enter free_desc:%d\n", nr_tre);

	if (!nr_tre)
		return;

	/* try going thru reclaim pool first */
	for (i = 0; i < nr_tre; i++) {
		/* peak for the next buffer, we going to peak several times,
		 * and we going to give up if buffers are not yet free
		 */
	    MSG_LOG("i %d cur_index %d\n", i, cur_index);
		cur_index = mhi_netdev->rx_current_index;
		netbuf = NULL;
		for (peak = 0; peak < max_peak; peak++) {
			struct mhi_netbuf *tmp = netbuf_pool[cur_index];

			mhi_buf = &tmp->mhi_buf;

			cur_index = (cur_index + 1) & pool_size;
	        
            MSG_LOG("peak %d cur_index %d\n", peak, cur_index);

			/* page == 1 idle, buffer is free to reclaim */
			if (page_ref_count(mhi_buf->page) == 1) {
				netbuf = tmp;
				break;
			}
		}

		/* could not find a free buffer */
		if (!netbuf)
			break;

	    MSG_LOG("mhi_buf 0x%p\n", mhi_buf);

		/* increment reference count so when network stack is done
		 * with buffer, the buffer won't be freed
		 */
		page_ref_inc(mhi_buf->page);
		dma_sync_single_for_device(dev, mhi_buf->dma_addr, mhi_buf->len,
					   DMA_FROM_DEVICE);
		ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE, mhi_buf,
					 mhi_buf->len, MHI_EOT);
		if (unlikely(ret)) {
			MSG_ERR("Failed to queue buffer, ret:%d\n", ret);
			netbuf->unmap(dev, mhi_buf->dma_addr, mhi_buf->len,
				      DMA_FROM_DEVICE);
			page_ref_dec(mhi_buf->page);
			return;
		}
		mhi_netdev->rx_current_index = cur_index;
	}

	/* recyling did not work, buffers are still busy allocate temp pkts */
	if (i < nr_tre)
		mhi_netdev_tmp_alloc(mhi_netdev, nr_tre - i);
}

/**
 * @brief allocating rx pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static int mhi_netdev_alloc_rx_pool(struct mhi_netdev *mhi_netdev)
{
	int i;
	struct mhi_netbuf *netbuf, **netbuf_pool;
	struct mhi_buf *mhi_buf;
	const u32 rx_order = mhi_netdev->rx_order;
	struct device *dev = mhi_netdev->mhi_dev->dev.parent;

	netbuf_pool = kmalloc_array(mhi_netdev->rx_pool_size, sizeof(*netbuf_pool),
				    GFP_KERNEL);
	if (!netbuf_pool)
		return -ENOMEM;

	for (i = 0; i < mhi_netdev->rx_pool_size; i++) {
		/* allocate paged data */
		netbuf = mhi_netdev_alloc(dev, GFP_DMA32 | GFP_KERNEL, rx_order, RX_DATA);
		if (!netbuf)
			goto error_alloc_page;

		netbuf->unmap = dma_sync_single_for_cpu;
		netbuf_pool[i] = netbuf;
	}

	mhi_netdev->rx_netbuf_pool = netbuf_pool;

	return 0;

error_alloc_page:
	for (--i; i >= 0; i--) {
		netbuf = netbuf_pool[i];
		mhi_buf = &netbuf->mhi_buf;
		dma_unmap_page(dev, mhi_buf->dma_addr, mhi_buf->len,
			       DMA_FROM_DEVICE);
		__free_pages(mhi_buf->page, rx_order);
	}

	kfree(netbuf_pool);

	return -ENOMEM;
}

/**
 * @brief allocating tx pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static int mhi_netdev_alloc_tx_pool(struct mhi_netdev *mhi_netdev)
{
	int i;
	struct mhi_netbuf *netbuf, **netbuf_pool;
	struct mhi_buf *mhi_buf;
	const u32 tx_order = mhi_netdev->tx_order;
	struct device *dev = mhi_netdev->mhi_dev->dev.parent;

	netbuf_pool = kmalloc_array(mhi_netdev->tx_pool_size, sizeof(*netbuf_pool),
				    GFP_KERNEL);
	if (!netbuf_pool) {
		MSG_ERR("failed to alloc buffer for tx pool!\n");
		return -ENOMEM;
	}

	for (i = 0; i < mhi_netdev->tx_pool_size; i++) {
		/* allocate paged data */
		netbuf = mhi_netdev_alloc(dev, GFP_DMA32 | GFP_KERNEL, tx_order, TX_DATA);
		if (!netbuf) {
			MSG_ERR("failed to alloc netbuf!\n");
			goto error_alloc_page;
		}

		netbuf->unmap = NULL;
		netbuf_pool[i] = netbuf;
	}

	mhi_netdev->tx_netbuf_pool = netbuf_pool;

	MSG_LOG("success!\n");

	return 0;

error_alloc_page:
	for (--i; i >= 0; i--) {
		netbuf = netbuf_pool[i];
		mhi_buf = &netbuf->mhi_buf;
		__free_pages(mhi_buf->page, tx_order);
	}

	kfree(netbuf_pool);

	return -ENOMEM;
}

/**
 * @brief free RX pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static void mhi_netdev_free_rx_pool(struct mhi_netdev *mhi_netdev)
{
	int i;
	struct mhi_netbuf *netbuf, **netbuf_pool = mhi_netdev->rx_netbuf_pool;
	struct device *dev = mhi_netdev->mhi_dev->dev.parent;
	struct mhi_buf *mhi_buf;

	for (i = 0; i < mhi_netdev->rx_pool_size; i++) {
		netbuf = netbuf_pool[i];
		mhi_buf = &netbuf->mhi_buf;
		dma_unmap_page(dev, mhi_buf->dma_addr, mhi_buf->len,
			       DMA_FROM_DEVICE);
		__free_pages(mhi_buf->page, mhi_netdev->rx_order);
	}

	kfree(mhi_netdev->rx_netbuf_pool);
	mhi_netdev->rx_netbuf_pool = NULL;
}

/**
 * @brief free TX pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static void mhi_netdev_free_tx_pool(struct mhi_netdev *mhi_netdev)
{
	int i;
	struct mhi_netbuf *netbuf, **netbuf_pool = mhi_netdev->tx_netbuf_pool;
	struct mhi_buf *mhi_buf;

	for (i = 0; i < mhi_netdev->tx_pool_size; i++) {
		netbuf = netbuf_pool[i];
		mhi_buf = &netbuf->mhi_buf;
		__free_pages(mhi_buf->page, mhi_netdev->tx_order);
	}

	kfree(mhi_netdev->tx_netbuf_pool);
	mhi_netdev->tx_netbuf_pool = NULL;
}

/**
 * @brief allocating pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static int mhi_netdev_alloc_pool(struct mhi_netdev *mhi_netdev)
{
	int ret;
	
	ret = mhi_netdev_alloc_rx_pool(mhi_netdev);
	if (ret) 
		return ret;

	ret = mhi_netdev_alloc_tx_pool(mhi_netdev);
	if (ret) 
		mhi_netdev_free_rx_pool(mhi_netdev);	

	return ret;
}

/**
 * @brief free pool of memory
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static void mhi_netdev_free_pool(struct mhi_netdev *mhi_netdev)
{
	mhi_netdev_free_rx_pool(mhi_netdev);
	mhi_netdev_free_tx_pool(mhi_netdev);
}

/**
 * @brief poll the net device
 *
 * @param[in ]            napi		struct napi_struct
 * @param[in ]            budget	poll budget
 */
static int mhi_netdev_poll(struct napi_struct *napi, int budget)
{
	struct net_device *dev = napi->dev;
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	struct mhi_netdev *rsc_dev = mhi_netdev->rsc_dev;
	struct mhi_net_chain *chain = mhi_netdev->rx_chain;
	int rx_work = 0;

	//MSG_VERB("Entered\n");
    MSG_LOG("Entered\n");

	rx_work = mhi_poll(mhi_dev, budget);

	/* chained skb, push it to stack */
	if (chain && chain->head) {
		netif_receive_skb(chain->head);
		chain->head = NULL;
	}

	if (rx_work < 0) {
		MSG_ERR("Error polling ret:%d\n", rx_work);
		napi_complete(napi);
		return 0;
	}

	/* queue new buffers */
    MSG_LOG("mhi_netdev_queue\n");
	mhi_netdev_queue(mhi_netdev);

	if (rsc_dev)
		mhi_netdev_queue(rsc_dev);

	/* complete work if # of packet processed less than allocated budget */
	if (rx_work < budget)
		napi_complete(napi);

	MSG_VERB("polled %d pkts\n", rx_work);
    MSG_LOG("polled %d pkts\n", rx_work);

	return rx_work;
}

/**
 * @brief open the net device
 *
 * @param[in ]            dev		struct net_device
 */
static int mhi_netdev_open(struct net_device *dev)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;

	MSG_LOG("Opened net dev interface\n");

	memcpy(dev->dev_addr, mac_addr, ETH_ALEN);

	/* tx queue may not necessarily be stopped already
	 * so stop the queue if tx path is not enabled
	 */
	if (!mhi_dev->ul_chan)
		netif_stop_queue(dev);
	else
		netif_start_queue(dev);

	return 0;

}

/**
 * @brief device stops
 *
 * @param[in ]		  dev		network device
 */
static int mhi_netdev_release(struct net_device *dev)
{
	netif_stop_queue(dev); 

	return 0;
}

/**
 * @brief change MTU for the net device
 *
 * @param[in ]            dev		struct net_device
 * @param[in ]            new_mtu	new MTU value
 */
static int mhi_netdev_change_mtu(struct net_device *dev, int new_mtu)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;

	if (new_mtu < 0 || mhi_dev->mtu < new_mtu)
		return -EINVAL;

	dev->mtu = new_mtu;
	return 0;
}

/**
 * @brief verify that the ethernet protocol is IPv4 or IPv6
 *
 * @param[in ]        proto		protocol
*/
static bool is_ip_proto(__be16 proto)
{
	switch (proto) {
	case htons(ETH_P_IP):
	case htons(ETH_P_IPV6):
		return true;
	}
	return false;
}

/**
 * @brief get QMAP tci
 *
 * @param[in ]        dev_info		device information
 * @param[in ]        skb			data packet
 * @param[in,out ]    tci		    
 */
int mhi_netdev_get_qmap_tci(struct net_device *dev, struct sk_buff *skb, u8 *tci_ret)
{
	u16 tci = 0;
	bool is_ip;
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;

	/* Some applications using e.g. packet sockets will
	 * bypass the VLAN acceleration and create tagged
	 * ethernet frames directly.  We primarily look for
	 * the accelerated out-of-band tag, but fall back if
	 * required
	 */
	skb_reset_mac_header(skb);
	if (vlan_get_tag(skb, &tci) < 0 && skb->len > VLAN_ETH_HLEN &&
		__vlan_get_tag(skb, &tci) == 0) {
		is_ip = is_ip_proto(vlan_eth_hdr(skb)->h_vlan_encapsulated_proto);
		skb_pull(skb, VLAN_ETH_HLEN);
	}
	else {
		is_ip = is_ip_proto(eth_hdr(skb)->h_proto);
		skb_pull(skb, ETH_HLEN);
	}

	/* Is IP session <0> tagged too? */
	if (mhi_netdev->flags & FLAG_IPS0_VLAN) {
		/* drop all untagged packets */
		if (!tci)
			goto error;
		/* map MBIM_IPS0_VID to IPS<0> */
		if (tci == MBIM_IPS0_VID)
			tci = 0;
	}

	/* mapping VLANs to QMAP mux_id id:
	 *   no tag     => mux_id <0> if !FLAG_IPS0_VLAN
	 *   1 - 127    => mux_id <vlanid>
	 *   4094       => mux_id <0> if FLAG_IPS0_VLAN
	 */

	switch (tci & 0x0f00) {
	case 0x0000: /* VLAN ID 1 - 127 */
		if (!is_ip)
			goto error;

		MSG_LOG("tci %d\n", tci);

		*tci_ret = tci; /* map VLAN ID to QMAP mux_id */
		break;

	default:
		MSG_ERR("unsupported tci 0x%04x\n", tci);
		goto error;
	}

	return 0;

error:
	return -EINVAL;
}

/**
 * @brief tx packet timer start
 *
 * @param[in ]        timer		timer
*/
void mhi_netdev_tx_timeout_start(struct mhi_netdev *mhi_netdev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
	ktime_t temp_time;
	temp_time.tv64 = mhi_netdev->timer_interval;
#endif
	/* start timer, if not already started */
	//MSG_LOG("active %d stop %d\n", hrtimer_active(&mhi_netdev->tx_timer), atomic_read(&mhi_netdev->stop));

	if (!(hrtimer_active(&mhi_netdev->tx_timer) || atomic_read(&mhi_netdev->stop))) {

		hrtimer_start(&mhi_netdev->tx_timer,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
			temp_time,
#else
			mhi_netdev->timer_interval,
#endif
			HRTIMER_MODE_REL);
	
		//MSG_LOG("tx timer started!\n");
	}
}

/**
 * @brief data tx BH
 *
 * @param[in ]        param		parameter
*/
void mhi_netdev_txpath_bh(unsigned long param)
{
	struct mhi_netdev *mhi_netdev = (struct mhi_netdev *)param;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	struct mhi_netbuf *netbuf = NULL;
	struct mhi_buf *mhi_buf = NULL;
	int res;

	spin_lock_bh(&mhi_netdev->tx_lock);

	MSG_LOG("tx_timer_pending 0x%x\n", mhi_netdev->tx_timer_pending);

	if (mhi_netdev->tx_timer_pending != 0) {
		mhi_netdev->tx_timer_pending--;
		mhi_netdev_tx_timeout_start(mhi_netdev);
	} else {
		
		netbuf = mhi_netdev->pending_netbuf;
		mhi_netdev->pending_netbuf = NULL;

		if (netbuf) {

			mhi_buf = &netbuf->mhi_buf;	

            MSG_LOG("send packets in mhi_netdev_txpath_bh 0x%p\n", netbuf);
			
			/* increment reference count so when network stack is done
		 	* with buffer, the buffer won't be freed
		 	*/
			page_ref_inc(mhi_buf->page);

			res = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, mhi_buf, mhi_buf->pkt_len,
				 MHI_EOT);

			if (res) {
				MSG_VERB("Failed to queue with reason:%d\n", res);
				page_ref_dec(mhi_buf->page);
			}
        }    
	}

	spin_unlock_bh(&mhi_netdev->tx_lock);
}

/**
 * @brief tx packet timer callback
 *
 * @param[in ]        timer		timerdev_info
*/
enum hrtimer_restart mhi_netdev_tx_timer_cb(struct hrtimer *timer)
{
	struct mhi_netdev *mhi_netdev =
		container_of(timer, struct mhi_netdev, tx_timer);

	//MSG_LOG(KERN_INFO "stop %d", atomic_read(&mhi_netdev->stop));

	if (!atomic_read(&mhi_netdev->stop))
		tasklet_schedule(&mhi_netdev->bh);
	
	return HRTIMER_NORESTART;
}

/**
 * @brief send data packet
 *
 * @param[in ]            skb		data packet
 * @param[in ]            dev		struct net_device
 */
static int mhi_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	int res = NETDEV_TX_OK;
    u32 len = 0;
	u8 tci = 0;

	struct mhi_netbuf *netbuf = NULL;
	struct mhi_buf *mhi_buf = NULL;
	struct mhi_netbuf **netbuf_pool = mhi_netdev->tx_netbuf_pool;

	int i, space;
	const int pool_size = mhi_netdev->tx_pool_size;
	u16 offset = 0, index;
    u8 header[4], *buf;

	MSG_LOG("skb 0x%p len %d\n", skb, skb->len);

#if 0
    MSG_LOG("mhi_netdev_xmit buffer\n");
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        skb->data, skb->len, true);
#endif

	/* get tci from skb */
	if (mhi_netdev_get_qmap_tci(dev, skb, &tci) < 0) {
		MSG_ERR("mhi_netdev_get_qmap_tci failed!\n");

		/* drop the packet */
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);

		return res;
	}

	if (mhi_queue_full(mhi_dev)) {
		MSG_ERR("upload channel is full!\n");

#if !defined(DROP_PKT_IF_RING_FULL) || (defined(DROP_PKT_IF_RING_FULL) && DROP_PKT_IF_RING_FULL == 0)
		netif_stop_queue(dev);
		res = NETDEV_TX_BUSY;
#else
		/* drop the packet */
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
#endif
		return res;
	}

	spin_lock_bh(&mhi_netdev->tx_lock);

	/* first packet (no pending packets)? */
	if (mhi_netdev->pending_netbuf == NULL) {

		index = mhi_netdev->tx_current_index;

		/* get the a free buffer from the pool */
		for (i = 0; i < pool_size; i++) {
			struct mhi_netbuf *tmp = netbuf_pool[index];

			index = (index+1) % pool_size;

			mhi_buf = &tmp->mhi_buf;

	    	MSG_LOG("i %d mhi_buf 0x%px page ref count %d\n", 
				i, mhi_buf, page_ref_count(mhi_buf->page));

			/* page == 1 idle, buffer is free to reclaim */
			if (page_ref_count(mhi_buf->page) == 1) {
				netbuf = tmp;
				mhi_netdev->tx_current_index = index;
				break;
			}
		}

		if (netbuf == NULL)	{
			MSG_ERR("no free tx buffer!\n");
			goto drop;
		}

		offset = 0;
		mhi_netdev->tx_num_pkt = 0;

	} else {
		netbuf = mhi_netdev->pending_netbuf;
		offset = mhi_netdev->tx_offset;
	}

	/* copy packet to mhi_buf */
	len = skb->len;

    /* compose the QMAP header */
    header[0] = (len % 4) ? (4 - (len % 4)) : 0;
    header[1] = tci;
    header[2] = ((len + header[0]) & 0xff00) >> 8;
    header[3] = ((len + header[0]) & 0x00ff);

	len += header[0];

	mhi_buf = &netbuf->mhi_buf;

	buf = (u8 *)mhi_buf->buf;

	/* copy the QMAP header */
	memcpy((u8 *)&buf[offset], &header, sizeof(header));
	offset += sizeof(header);

	/* copy the packet */
	memcpy((u8 *)&buf[offset], skb->data, skb->len);

	offset += len;

	/* save the offset */
	mhi_netdev->tx_offset = offset;

	mhi_buf->pkt_len = offset;

#if 0
    MSG_LOG("netdev_xmit buffer after adding QMAP header\n");
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        mhi_buf->buf[offset], offset, true);
#endif

	/* after copying, free the skb */
	dev_kfree_skb_any(skb);

	mhi_netdev->tx_num_pkt++;
	mhi_buf->pkt_num = mhi_netdev->tx_num_pkt;

	/* make sure the buffer is enough for another packet */
	space = mhi_buf->len - offset - dev->mtu - sizeof(header);

	MSG_LOG("pkt %d space %d\n", mhi_netdev->tx_num_pkt, space);

	if ((mhi_netdev->tx_num_pkt >= MAX_UL_PKT_AGGR) || (space < 0))
	{
		MSG_LOG("accumulate %d pkt ready to send!\n", mhi_netdev->tx_num_pkt);

		/* increment reference count so when network stack is done
		 * with buffer, the buffer won't be freed
		 */
		page_ref_inc(mhi_buf->page);

		res = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, mhi_buf, mhi_netdev->tx_offset,
				 MHI_EOT);

		if (res) {
			MSG_VERB("Failed to queue with reason:%d\n", res);

			page_ref_dec(mhi_buf->page);

			/* drop the packet */
			dev->stats.tx_dropped += mhi_netdev->tx_num_pkt;
		}

		mhi_netdev->pending_netbuf = NULL;

		res = NETDEV_TX_OK;

		goto exit;
	}

	mhi_netdev->pending_netbuf = netbuf;

	if (mhi_netdev->tx_num_pkt < CDC_NCM_RESTART_TIMER_DATAGRAM_CNT)
		mhi_netdev->tx_timer_pending = CDC_NCM_TIMER_PENDING_CNT;

	/* start timer, if there is a buffer waiting */
	if (mhi_netdev->pending_netbuf != NULL && mhi_netdev->tx_num_pkt > 0) {

		MSG_LOG("start tx timer!\n");

		mhi_netdev_tx_timeout_start(mhi_netdev);
	}

	goto exit;

drop:
	dev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);

exit:
	spin_unlock_bh(&mhi_netdev->tx_lock);

	return res;	
}	

/**
 * @brief ioctl extension handler
 *
 * @param[in ]            dev		struct net_device
 * @param[in ]            ifreq		request
 */
static int mhi_netdev_ioctl_extended(struct net_device *dev, struct ifreq *ifr)
{
	struct rmnet_ioctl_extended_s ext_cmd;
	int rc = 0;
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;

	rc = copy_from_user(&ext_cmd, ifr->ifr_ifru.ifru_data,
			    sizeof(struct rmnet_ioctl_extended_s));
	if (rc)
		return rc;

	switch (ext_cmd.extended_ioctl) {
	case RMNET_IOCTL_GET_SUPPORTED_FEATURES:
		ext_cmd.u.data = 0;
		break;
	case RMNET_IOCTL_GET_DRIVER_NAME:
		strlcpy(ext_cmd.u.if_name, mhi_netdev->interface_name,
			sizeof(ext_cmd.u.if_name));
		break;
	case RMNET_IOCTL_SET_SLEEP_STATE:
		if (ext_cmd.u.data && mhi_netdev->wake) {
			/* Request to enable LPM */
			MSG_VERB("Enable MHI LPM");
			mhi_netdev->wake--;
			mhi_device_put(mhi_dev, MHI_VOTE_DEVICE);
		} else if (!ext_cmd.u.data && !mhi_netdev->wake) {
			/* Request to disable LPM */
			MSG_VERB("Disable MHI LPM");
			mhi_netdev->wake++;
			mhi_device_get(mhi_dev, MHI_VOTE_DEVICE);
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	rc = copy_to_user(ifr->ifr_ifru.ifru_data, &ext_cmd,
			  sizeof(struct rmnet_ioctl_extended_s));
	return rc;
}

/**
 * @brief ioctl handler
 *
 * @param[in ]            dev		struct net_device
 * @param[in ]            ifreq		request
 * @param[in ]            cmd		command
 */
static int mhi_netdev_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int rc = 0;
	struct rmnet_ioctl_data_s ioctl_data;

	switch (cmd) {
	case RMNET_IOCTL_SET_LLP_IP: /* set RAWIP protocol */
		break;
	case RMNET_IOCTL_GET_LLP: /* get link protocol state */
		ioctl_data.u.operation_mode = RMNET_MODE_LLP_IP;
		if (copy_to_user(ifr->ifr_ifru.ifru_data, &ioctl_data,
		    sizeof(struct rmnet_ioctl_data_s)))
			rc = -EFAULT;
		break;
	case RMNET_IOCTL_GET_OPMODE: /* get operation mode */
		ioctl_data.u.operation_mode = RMNET_MODE_LLP_IP;
		if (copy_to_user(ifr->ifr_ifru.ifru_data, &ioctl_data,
		    sizeof(struct rmnet_ioctl_data_s)))
			rc = -EFAULT;
		break;
	case RMNET_IOCTL_SET_QOS_ENABLE:
		rc = -EINVAL;
		break;
	case RMNET_IOCTL_SET_QOS_DISABLE:
		rc = 0;
		break;
	case RMNET_IOCTL_OPEN:
	case RMNET_IOCTL_CLOSE:
		/* we just ignore them and return success */
		rc = 0;
		break;
	case RMNET_IOCTL_EXTENDED:
		rc = mhi_netdev_ioctl_extended(dev, ifr);
		break;
	default:
		/* don't fail any IOCTL right now */
		rc = 0;
		break;
	}

	return rc;
}

/**
 * @brief add VLAN id to network device
 *
 * @param[in ]        dev		network device
 * @param[in ]        proto		protocol
 * @param[in ]        vid		VLAN id
 */
static int mhi_netdev_rx_add_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;

	/* creation of this VLAN is a request to tag IP session 0 */
	if (vid == MBIM_IPS0_VID)
		mhi_netdev->flags |= FLAG_IPS0_VLAN;
	else
		if (vid > 127) {	/* we don't map these to QMAP mux_id */
			MSG_ERR("VLAN id %d is out of range!\n", vid);
			return -EINVAL;
		}
	return 0;
}

/**
 * @brief remove VLAN id from network device
 *
 * @param[in ]        dev		network device
 * @param[in ]        proto		protocol
 * @param[in ]        vid		VLAN id
 */
static int mhi_netdev_rx_kill_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;

	/* this is a request for an untagged IP session 0 */
	if (vid == MBIM_IPS0_VID)
		mhi_netdev->flags &= ~FLAG_IPS0_VLAN;

	return 0;
}

#ifdef XDP_SUPPORT
static int mhi_netdev_xdp_setup(struct net_device *dev, struct bpf_prog *prog)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;

	struct bpf_prog *old_prog;

	old_prog = xchg(&mhi_netdev->xdp_prog, prog);

	if (old_prog)
		bpf_prog_put(old_prog);

	return 0;
}

static int mhi_netdev_xdp(struct net_device *dev, struct netdev_bpf *xdp)
{
	struct mhi_netdev_priv *mhi_netdev_priv = netdev_priv(dev);
	struct mhi_netdev *mhi_netdev = mhi_netdev_priv->mhi_netdev;

	MSG_LOG("Enter: xdp command %d\n", xdp->command);

	switch (xdp->command) {
	case XDP_SETUP_PROG:
		MSG_LOG("XDP_SETUP_PROG\n");
		return mhi_netdev_xdp_setup(dev, xdp->prog);
	case XDP_QUERY_PROG:
		MSG_LOG("XDP_QUERY_PROG\n");
		xdp->prog_id = mhi_netdev->xdp_prog ?
			mhi_netdev->xdp_prog->aux->id : 0;
		return 0;

	default:
		return -EINVAL;
	}
}
#endif

static const struct net_device_ops mhi_netdev_ops_ip = {
	.ndo_open = mhi_netdev_open,
    .ndo_stop = mhi_netdev_release,
	.ndo_start_xmit = mhi_netdev_xmit,
	.ndo_do_ioctl = mhi_netdev_ioctl,
	.ndo_change_mtu = mhi_netdev_change_mtu,
	.ndo_set_mac_address = 0,
	.ndo_validate_addr = 0,
	.ndo_vlan_rx_add_vid = mhi_netdev_rx_add_vid,
	.ndo_vlan_rx_kill_vid = mhi_netdev_rx_kill_vid,
#ifdef XDP_SUPPORT
	.ndo_bpf = mhi_netdev_xdp,
#endif	
};

/**
 * @brief initialize data structure for net device
 *
 * @param[in ]            dev		struct net_device
 */
static void mhi_netdev_setup(struct net_device *dev)
{
	ether_setup(dev);

	dev->netdev_ops = &mhi_netdev_ops_ip;

	/* keep the default flags, just add NOARP */
	dev->flags |= IFF_NOARP;

	/* no need to put the VLAN tci in the packet headers */
	dev->features |= NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_FILTER;

	memcpy(dev->dev_addr, mac_addr, ETH_ALEN);

	//dev->watchdog_timeo = WATCHDOG_TIMEOUT;
}

static struct device_type wwan_type = {
	.name = "wwan",
};

/**
 * @brief enable mhi_netdev netdev, call only after grabbing mhi_netdev.mutex
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 */
static int mhi_netdev_enable_iface(struct mhi_netdev *mhi_netdev)
{
	int ret = 0;
	char ifalias[IFALIASZ];
	char ifname[IFNAMSIZ];
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	//struct device_node *of_node = mhi_dev->dev.of_node;
	struct mhi_netdev_priv *mhi_netdev_priv;

	MSG_LOG("Enter\n");

	mhi_netdev->interface_name = mhi_netdev_driver.driver.name;

	snprintf(ifalias, sizeof(ifalias), "%s_%04x_%02u.%02u.%02u_%u",
		 mhi_netdev->interface_name, mhi_dev->dev_id, mhi_dev->domain,
		 mhi_dev->bus, mhi_dev->slot, mhi_netdev->alias);

	snprintf(ifname, sizeof(ifname), "%s%%d", mhi_netdev->interface_name);

	rtnl_lock();
	mhi_netdev->ndev = alloc_netdev(sizeof(*mhi_netdev_priv),
					ifname, NET_NAME_PREDICTABLE,
					mhi_netdev_setup);
	if (!mhi_netdev->ndev) {
		rtnl_unlock();
		return -ENOMEM;
	}

	mhi_netdev->ndev->mtu = 1500; // mhi_dev->mtu;
	SET_NETDEV_DEV(mhi_netdev->ndev, &mhi_dev->dev);

	SET_NETDEV_DEVTYPE(mhi_netdev->ndev, &wwan_type);
#if 0
	dev_set_alias(mhi_netdev->ndev, ifalias, strlen(ifalias));
#endif    
	mhi_netdev_priv = netdev_priv(mhi_netdev->ndev);
	mhi_netdev_priv->mhi_netdev = mhi_netdev;
	rtnl_unlock();

	mhi_netdev->napi = devm_kzalloc(&mhi_dev->dev,
					sizeof(*mhi_netdev->napi), GFP_KERNEL);
	if (!mhi_netdev->napi) {
		ret = -ENOMEM;
		goto napi_alloc_fail;
	}

	netif_napi_add(mhi_netdev->ndev, mhi_netdev->napi,
		       mhi_netdev_poll, NAPI_POLL_WEIGHT);
	ret = register_netdev(mhi_netdev->ndev);
	if (ret) {
		MSG_ERR("Network device registration failed\n");
		goto net_dev_reg_fail;
	}

	napi_enable(mhi_netdev->napi);

	MSG_LOG("Exited.\n");

	return 0;

net_dev_reg_fail:
	netif_napi_del(mhi_netdev->napi);

napi_alloc_fail:
	free_netdev(mhi_netdev->ndev);
	mhi_netdev->ndev = NULL;

	return ret;
}

/**
 * @brief host to device transfer callback
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 * @param[in ]            mhi_result		struct mhi_result
 */
static void mhi_netdev_xfer_ul_cb(struct mhi_device *mhi_dev,
				  struct mhi_result *mhi_result)
{
	struct mhi_netdev *mhi_netdev = mhi_device_get_devdata(mhi_dev);
	struct mhi_buf *mhi_buf = mhi_result->buf_addr;
	struct net_device *ndev = mhi_netdev->ndev;

    MSG_LOG("mhi_buf 0x%p num %d len %ld\n", mhi_buf, mhi_buf->pkt_num, 
		(long int)mhi_buf->pkt_len);

	ndev->stats.tx_packets += mhi_buf->pkt_num;
	ndev->stats.tx_bytes += mhi_buf->pkt_len;

	/* drop the page ref count */
	__free_pages(mhi_buf->page, mhi_netdev->tx_order);

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

/**
 * @brief handle neighbor solicitation
 *
 * Some devices are known to send Neigbor Solicitation messages and
 * require Neigbor Advertisement replies.  The IPv6 core will not
 * respond since IFF_NOARP is set, so we must handle them ourselves.
 *
 * @param[in ]        pdevice_info		device information
 * @param[in ]        buf				buffer
 * @param[in ]        tci
*/
static void do_neigh_solicit(struct net_device *dev, u8 *buf, u16 tci)
{
	struct ipv6hdr *iph = (void *)buf;
	struct nd_msg *msg = (void *)(iph + 1);
	struct net_device *netdev;
	struct inet6_dev *in6_dev;
	bool is_router;

	/* we'll only respond to requests from unicast addresses to
	 * our solicited node addresses.
	 */
	if (!ipv6_addr_is_solict_mult(&iph->daddr) ||
		!(ipv6_addr_type(&iph->saddr) & IPV6_ADDR_UNICAST))
		return;

	/* need to send the NA on the VLAN dev, if any */
	rcu_read_lock();
	if (tci) {
		netdev = __vlan_find_dev_deep_rcu(dev, htons(ETH_P_8021Q),
			tci);
		if (!netdev) {
			rcu_read_unlock();
			return;
		}
	}
	else {
		netdev = dev;
	}
	dev_hold(netdev);
	rcu_read_unlock();

	in6_dev = in6_dev_get(netdev);
	if (!in6_dev)
		goto out;
	is_router = !!in6_dev->cnf.forwarding;
	in6_dev_put(in6_dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
	/* ipv6_stub != NULL if in6_dev_get returned an inet6_dev */
	ipv6_stub->ndisc_send_na(netdev, NULL, &iph->saddr, &msg->target,
		is_router /* router */,
		true /* solicited */,
		false /* override */,
		true /* inc_opt */);
#else
	/* ipv6_stub != NULL if in6_dev_get returned an inet6_dev */
	ipv6_stub->ndisc_send_na(netdev, &iph->saddr, &msg->target,
		is_router /* router */,
		true /* solicited */,
		false /* override */,
		true /* inc_opt */);
#endif

out:
	dev_put(netdev);
}

/**
 * @brief check if packet is a neighbor solicitation request
 *
 * @param[in ]        buf		buffer
 * @param[in ]        len		buffer length
 */
static bool is_neigh_solicit(u8 *buf, size_t len)
{
	struct ipv6hdr *iph = (void *)buf;
	struct nd_msg *msg = (void *)(iph + 1);

	return (len >= sizeof(struct ipv6hdr) + sizeof(struct nd_msg) &&
		iph->nexthdr == IPPROTO_ICMPV6 &&
		msg->icmph.icmp6_code == 0 &&
		msg->icmph.icmp6_type == NDISC_NEIGHBOUR_SOLICITATION);
}

/**
 * @brief pass data packet to stack
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 * @param[in ]            mhi_buf			struct mhi_buf
 * @param[in ]            buf				buffer
 * @param[in ]            len 				buffer size
 * @param[in ]            offset			offset
 */
static void mhi_netdev_push_skb(struct mhi_netdev *mhi_netdev,
				struct mhi_buf *mhi_buf,
				u8 *buf,
                u16 len,
                u32 offset,
				u16 tci)
{
	__be16 proto = htons(ETH_P_802_3);
	struct sk_buff *skb;

	MSG_LOG("tci %d\n", tci);

	if (tci < 128 || tci == MBIM_IPS0_VID) { /* IPS session? */
		if (len < sizeof(struct iphdr))
			goto err;

		switch (*buf & 0xf0) {
		case 0x40:
			proto = htons(ETH_P_IP);
			break;
		case 0x60:
			if (is_neigh_solicit(buf, len))
				do_neigh_solicit(mhi_netdev->ndev, buf, tci);
			proto = htons(ETH_P_IPV6);
			break;
		default:
			goto err;
		}
	}

	MSG_LOG("proto 0x%x\n", proto);

    /* reserve the head room and also the ethernet header */
	skb = alloc_skb(len + HEADER_RESERVE + ETH_HLEN, GFP_ATOMIC);
	if (!skb) {
		MSG_ERR("Failed to allocate skb!\n");
		return;
	}

	skb_reserve(skb, HEADER_RESERVE);

	MSG_LOG("skb headroom: %d\n", skb_headroom(skb));

	skb_put(skb, ETH_HLEN);
	skb_reset_mac_header(skb);
	eth_hdr(skb)->h_proto = proto;
	eth_zero_addr(eth_hdr(skb)->h_source);
	memcpy(eth_hdr(skb)->h_dest, mac_addr, ETH_ALEN);

	/* add datagram */
	skb_put_data(skb, buf, len);

	skb->dev = mhi_netdev->ndev;
	// skb->protocol = mhi_netdev_ip_type_trans(*(u8 *)buf);

	if (skb->protocol == 0)
		skb->protocol = eth_type_trans(skb, mhi_netdev->ndev);

	MSG_LOG("eth_type_trans protocol 0x%x pkt_type %d!", skb->protocol, skb->pkt_type);
	skb->pkt_type = PACKET_HOST;

	/* map MBIM session to VLAN */
	if (tci)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), tci);

#if 0
    MSG_LOG("before netif_receive_skb\n");
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        skb->data, skb->len, true);
#endif

	netif_receive_skb(skb);

err: 
	return;
}

/**
 * @brief status callback
 *
 * @param[in ]            mhi_netdev		struct mhi_netdev
 * @param[in ]            mhi_cb			enum MHI_CB
 */
static void mhi_netdev_status_cb(struct mhi_device *mhi_dev, enum MHI_CB mhi_cb)
{
	struct mhi_netdev *mhi_netdev = mhi_device_get_devdata(mhi_dev);

	if (mhi_cb != MHI_CB_PENDING_DATA)
		return;

	napi_schedule(mhi_netdev->napi);
}

#ifdef XDP_SUPPORT
/**
 * @brief send data packet for XDP 
 *
 * @param[in ]            dev		struct net_device
 * @param[in ]            xdp		struct xdp_buff
 */
static int mhi_netdev_xmit_xdp(struct mhi_netdev *mhi_netdev, 
	struct xdp_buff *xdp)
{
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;
	int res = 0;
    u32 len = 0;

	struct mhi_netbuf *netbuf = NULL;
	struct mhi_buf *mhi_buf = NULL;
	struct mhi_netbuf **netbuf_pool = mhi_netdev->tx_netbuf_pool;

	int i, space;
	const int pool_size = mhi_netdev->tx_pool_size;
	u16 offset = 0, index;
    u8 *buf;

	MSG_LOG("xdp 0x%p len %ld\n", xdp, xdp->data_end - xdp->data_hard_start);

#if 0
    MSG_LOG("mhi_netdev_xmit_xdp buffer\n");
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        xdp.data_hard_start, xdp->data_end - xdp->data + 4, true);
#endif

	spin_lock_bh(&mhi_netdev->tx_lock);

	/* first packet (no pending packets)? */
	if (mhi_netdev->pending_netbuf == NULL) {

		index = mhi_netdev->tx_current_index;

		/* get the a free buffer from the pool */
		for (i = 0; i < pool_size; i++) {
			struct mhi_netbuf *tmp = netbuf_pool[index];

			index = (index+1) % pool_size;

			mhi_buf = &tmp->mhi_buf;

	    	MSG_LOG("i %d mhi_buf 0x%px page ref count %d\n", 
				i, mhi_buf, page_ref_count(mhi_buf->page));

			/* page == 1 idle, buffer is free to reclaim */
			if (page_ref_count(mhi_buf->page) == 1) {
				netbuf = tmp;
				mhi_netdev->tx_current_index = index;
				break;
			}
		}

		if (netbuf == NULL)	{
			MSG_ERR("no free tx buffer!\n");
			goto drop;
		}

		offset = 0;
		mhi_netdev->tx_num_pkt = 0;

	} else {
		netbuf = mhi_netdev->pending_netbuf;
		offset = mhi_netdev->tx_offset;
	}

	/* copy packet (including QMAP header) to mhi_buf */
	len = xdp->data_end - xdp->data_hard_start;

	mhi_buf = &netbuf->mhi_buf;

	buf = (u8 *)mhi_buf->buf;

	/* copy the QMAP header & payload */
	memcpy((u8 *)&buf[offset], xdp->data_hard_start, len);
	offset += len;

	/* save the offset */
	mhi_netdev->tx_offset = offset;

	mhi_buf->pkt_len = offset;

#if 0
    MSG_LOG("netdev_xmit buffer after adding QMAP header\n");
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        mhi_buf->buf[offset], len, true);
#endif

	mhi_netdev->tx_num_pkt++;
	mhi_buf->pkt_num = mhi_netdev->tx_num_pkt;

	/* make sure the buffer is enough for another packet */
	space = mhi_buf->len - offset - mhi_netdev->ndev->mtu - 4;

	MSG_LOG("pkt %d space %d\n", mhi_netdev->tx_num_pkt, space);

	if ((mhi_netdev->tx_num_pkt >= MAX_UL_PKT_AGGR) || (space < 0))
	{
		MSG_LOG("accumulate %d pkt ready to send!\n", mhi_netdev->tx_num_pkt);

		/* increment reference count so when network stack is done
		 * with buffer, the buffer won't be freed
		 */
		page_ref_inc(mhi_buf->page);

		res = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, mhi_buf, mhi_netdev->tx_offset,
				 MHI_EOT);

		if (res) {
			MSG_VERB("Failed to queue with reason:%d\n", res);
			page_ref_dec(mhi_buf->page);

			/* drop the packet */
			dev->stats.tx_dropped += mhi_netdev->tx_num_pkt;
		}

		mhi_netdev->pending_netbuf = NULL;

		goto exit;
	}

	if (mhi_netdev->tx_num_pkt < CDC_NCM_RESTART_TIMER_DATAGRAM_CNT)
		mhi_netdev->tx_timer_pending = CDC_NCM_TIMER_PENDING_CNT;

	/* start timer, if there is a buffer waiting */
	if (mhi_netdev->pending_netbuf != NULL && mhi_netdev->tx_num_pkt > 0) {

		MSG_LOG("start tx timer!\n");

		mhi_netdev_tx_timeout_start(mhi_netdev);
	}

	goto exit;

drop:
	mhi_netdev->ndev->stats.tx_dropped++;

exit:
	spin_unlock_bh(&mhi_netdev->tx_lock);

	return res;	
}	

static u32 mhi_netdev_run_xdp(struct mhi_netdev *mhi_netdev,
			struct xdp_buff *xdp)
{
	int result;
	struct bpf_prog *xdp_prog;
	u32 act = XDP_PASS;		/* default action */

	xdp_prog = mhi_netdev->xdp_prog;

	if (!xdp_prog)
		goto xdp_out;

	act = bpf_prog_run_xdp(xdp_prog, xdp);
	
	MSG_LOG("action: %d\n", act);
	switch (act) {
	case XDP_PASS:
		MSG_LOG("action: XDP_PASS\n");
		break;
	case XDP_TX:
		MSG_LOG("action: XDP_TX\n");
		result = mhi_netdev_xmit_xdp(mhi_netdev, xdp);
		break;
	case XDP_REDIRECT:
		MSG_LOG("action: XDP_REDIRECT\n");
		result = xdp_do_redirect(mhi_netdev->ndev, xdp, xdp_prog);
		break;
	default:
		bpf_warn_invalid_xdp_action(act);
		/* fallthrough */
	case XDP_ABORTED:
		MSG_LOG("action: XDP_ABORTED\n");
		//trace_xdp_exception(mhi_netdev->ndev, xdp_prog, act);
		/* fallthrough -- handle aborts by dropping packet */
	case XDP_DROP:
		MSG_LOG("action: XDP_DROP\n");
		break;
	}
xdp_out:
	return act;
}
#endif 

/**
 * @brief device to host transfer callback
 *
 * @param[in ]            mhi_dev			struct mhi_device
 * @param[in ]            mhi_result		struct mhi_result
 */
static void mhi_netdev_xfer_dl_cb(struct mhi_device *mhi_dev,
				  struct mhi_result *mhi_result)
{
	struct mhi_netdev *mhi_netdev = mhi_device_get_devdata(mhi_dev);
	struct mhi_netbuf *netbuf = mhi_result->buf_addr;
	struct mhi_buf *mhi_buf = &netbuf->mhi_buf;
	//struct sk_buff *skb;
	struct net_device *ndev = mhi_netdev->ndev;
	struct device *dev = mhi_dev->dev.parent;
	//struct mhi_net_chain *chain = mhi_netdev->chain;
	u8 *header, *buf, *rx_buf = (u8 *)mhi_buf->buf;
	u32 offset = 0;
	u16 len = 0, tci;

#ifdef XDP_SUPPORT
	struct xdp_buff xdp;
	u32 act;
#endif

	netbuf->unmap(dev, mhi_buf->dma_addr, mhi_buf->len, DMA_FROM_DEVICE);

	/* modem is down, drop the buffer */
	if (mhi_result->transaction_status == -ENOTCONN) {
		__free_pages(mhi_buf->page, mhi_netdev->rx_order);
		return;
	}

	MSG_LOG("rx_bytes: %ld\n", (long)mhi_result->bytes_xferd);

#if 0
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
        mhi_buf->buf, mhi_result->bytes_xferd, true);
#endif

next_packet:
    buf = (u8 *)&rx_buf[offset];
	header = (u8 *)&rx_buf[offset];

	/* max pad is 3 and C/D bit should be 0 */
	if (header[0] > 3) {
		MSG_ERR("invalid QMAP pad or c/d bit 0x%x!", header[0]);
		goto error;
	}

	/* range of max_id is 1 to 127, 128 to 255 are reserved */
	if (header[1] > 127) {
		MSG_ERR("invalid QMAP mux_id 0x%x!", header[1]);
		goto error;
	}
	
	tci = header[1] & 0x7f;

	if (!tci && mhi_netdev->flags & FLAG_IPS0_VLAN)
		tci = MBIM_IPS0_VID;

	len = (((u16)header[2]) << 8) + header[3];

	if ((len < header[0]) || (len > mhi_result->bytes_xferd)) {
		MSG_ERR("length invalid %d!", len);
        goto error;
	}

	offset += 4;
    buf += 4;

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += (len - header[0]);

#ifdef XDP_SUPPORT
	xdp.data = buf;
	xdp.data_end = buf + len - header[0];
	xdp.data_meta = xdp.data;
	xdp.data_hard_start = buf - 4;
	xdp.handle = 0;
	xdp.rxq = NULL;

	act = mhi_netdev_run_xdp(mhi_netdev, &xdp);

	if (act == XDP_PASS) {

		MSG_LOG("pkt buf 0x%p len %d\n", buf, len - header[0]);
		mhi_netdev_push_skb(mhi_netdev, mhi_buf, buf, len - header[0], offset, tci);
	}
#else
	mhi_netdev_push_skb(mhi_netdev, mhi_buf, buf, len - header[0], offset, tci);
#endif

#if 0
	if (unlikely(!chain)) {
		mhi_netdev_push_skb(mhi_netdev, mhi_buf, buf, len, offset);
		goto next;
	}

	/* we support chaining */
	skb = alloc_skb(0, GFP_ATOMIC);
	if (likely(skb)) {
		skb_add_rx_frag(skb, 0, mhi_buf->page, offset,
				len, mhi_netdev->mru);
		/* this is first on list */
		if (!chain->head) {
			skb->dev = ndev;
			skb->protocol =
				mhi_netdev_ip_type_trans(*(u8 *)buf);
			chain->head = skb;
		} else {
			skb_shinfo(chain->tail)->frag_list = skb;
		}

		chain->tail = skb;
	} else goto error;
#endif

//next:
	offset += len;
	MSG_LOG("offset %d\n", offset);

	if ((offset + 1) < mhi_result->bytes_xferd)
		goto next_packet;

    /* send more pend data read requests */
    mhi_netdev_status_cb(mhi_dev, MHI_CB_PENDING_DATA);
    
error:
    __free_pages(mhi_buf->page, mhi_netdev->rx_order);
    return;
}

#ifdef CONFIG_DEBUG_FS

struct dentry *dentry;

static void mhi_netdev_create_debugfs(struct mhi_netdev *mhi_netdev)
{
	char node_name[32];
	struct mhi_device *mhi_dev = mhi_netdev->mhi_dev;

	/* Both tx & rx client handle contain same device info */
	snprintf(node_name, sizeof(node_name), "%s_%04x_%02u.%02u.%02u_%u",
		 mhi_netdev->interface_name, mhi_dev->dev_id, mhi_dev->domain,
		 mhi_dev->bus, mhi_dev->slot, mhi_netdev->alias);

	if (IS_ERR_OR_NULL(dentry))
		return;

	mhi_netdev->dentry = debugfs_create_dir(node_name, dentry);
	if (IS_ERR_OR_NULL(mhi_netdev->dentry))
		return;
}

static void mhi_netdev_create_debugfs_dir(void)
{
	dentry = debugfs_create_dir(MHI_NETDEV_DRIVER_NAME, 0);
}

#else

static void mhi_netdev_create_debugfs(struct mhi_netdev_private *mhi_netdev)
{
}

static void mhi_netdev_create_debugfs_dir(void)
{
}

#endif


/**
 * @brief remove net device
 *
 * @param[in ]            mhi_dev			struct mhi_device
 */
static void mhi_netdev_remove(struct mhi_device *mhi_dev)
{
	struct mhi_netdev *mhi_netdev = mhi_device_get_devdata(mhi_dev);

	MSG_DEFAULT("enter\n");

   	/* stop the channel */
	MSG_LOG("stop channel!\n");
	mhi_unprepare_from_transfer(mhi_netdev->mhi_dev);

	/* stop the tx timer */
	atomic_set(&mhi_netdev->stop, 1);

	if (hrtimer_active(&mhi_netdev->tx_timer))
		hrtimer_cancel(&mhi_netdev->tx_timer);

	tasklet_kill(&mhi_netdev->bh);

	/* rsc parent takes cares of the cleanup */
	if (mhi_netdev->is_rsc_dev) {
		mhi_netdev_free_pool(mhi_netdev);
		return;
	}

	netif_stop_queue(mhi_netdev->ndev);
	napi_disable(mhi_netdev->napi);
	unregister_netdev(mhi_netdev->ndev);
	netif_napi_del(mhi_netdev->napi);
	free_netdev(mhi_netdev->ndev);
	mhi_netdev_free_pool(mhi_netdev);

#ifdef CONFIG_DEBUG_FS
	if (!IS_ERR_OR_NULL(mhi_netdev->dentry))
		debugfs_remove_recursive(mhi_netdev->dentry);
#endif		
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,3,0)
static int mhi_netdev_match(struct device *dev, void *data)
#else
static int mhi_netdev_match(struct device *dev, const void *data)
#endif
{
	/* if phandle dt == device dt, we found a match */
	return (dev->of_node == data);
}

static void mhi_netdev_clone_dev(struct mhi_netdev *mhi_netdev,
				 struct mhi_netdev *parent)
{
	mhi_netdev->ndev = parent->ndev;
	mhi_netdev->napi = parent->napi;
	mhi_netdev->ipc_log = parent->ipc_log;
	mhi_netdev->msg_lvl = parent->msg_lvl;
	mhi_netdev->ipc_log_lvl = parent->ipc_log_lvl;
	mhi_netdev->is_rsc_dev = true;
	mhi_netdev->rx_chain = parent->rx_chain;
}

/**
 * @brief probe net device
 *
 * @param[in ]            mhi_dev			struct mhi_device
 * @param[in ]            id				struct mhi_device_id
 */
static int mhi_netdev_probe(struct mhi_device *mhi_dev,
			    const struct mhi_device_id *id)
{
	int ret;
	struct mhi_netdev *mhi_netdev, *p_netdev = NULL;
	struct device_node *of_node = mhi_dev->dev.of_node;
	int nr_tre;
	char node_name[32];
	struct device_node *phandle;
	bool no_chain;

    MSG_DEFAULT("enter\n");

	mhi_netdev = devm_kzalloc(&mhi_dev->dev, sizeof(*mhi_netdev),
				  GFP_KERNEL);
	if (!mhi_netdev)
		return -ENOMEM;

	mhi_netdev->mhi_dev = mhi_dev;
	mhi_device_set_devdata(mhi_dev, mhi_netdev);

    mhi_netdev->mru = QMAP_RX_BUFFER_SIZE;
	mhi_netdev->mtu = QMAP_TX_BUFFER_SIZE;

	/* MRU must be multiplication of page size */
	mhi_netdev->rx_order = __ilog2_u32(mhi_netdev->mru / PAGE_SIZE);
	if ((PAGE_SIZE << mhi_netdev->rx_order) < mhi_netdev->mru) {
		return -EINVAL;
    }

	/* MTU must be multiplication of page size */
	mhi_netdev->tx_order = __ilog2_u32(mhi_netdev->mtu / PAGE_SIZE);
	if ((PAGE_SIZE << mhi_netdev->tx_order) < mhi_netdev->mtu) {
		return -EINVAL;
    }

	/* check if this device shared by a parent device */
	phandle = of_parse_phandle(of_node, "mhi,rsc-parent", 0);
	if (phandle) {
		struct device *dev;
		struct mhi_device *pdev;
		/* find the parent device */
		dev = driver_find_device(mhi_dev->dev.driver, NULL, phandle,
					 mhi_netdev_match);
		if (!dev)
			return -ENODEV;

		/* this device is shared with parent device. so we won't be
		 * creating a new network interface. Clone parent
		 * information to child node
		 */
		pdev = to_mhi_device(dev);
		p_netdev = mhi_device_get_devdata(pdev);
		mhi_netdev_clone_dev(mhi_netdev, p_netdev);
		put_device(dev);
	} else {
        if (debug) {
		    mhi_netdev->msg_lvl = MHI_MSG_LVL_INFO;
        } else {     
		    mhi_netdev->msg_lvl = MHI_MSG_LVL_MASK_ALL;
        }
        
        no_chain = of_property_read_bool(of_node,
						 "mhi,disable-chain-skb");
		if (!no_chain) {
			mhi_netdev->rx_chain = devm_kzalloc(&mhi_dev->dev,
						sizeof(*mhi_netdev->rx_chain),
						GFP_KERNEL);
			if (!mhi_netdev->rx_chain)
				return -ENOMEM;
		}

		ret = mhi_netdev_enable_iface(mhi_netdev);
		if (ret)
			return ret;

		/* create ipc log buffer */
		snprintf(node_name, sizeof(node_name),
			 "%s_%04x_%02u.%02u.%02u_%u",
			 mhi_netdev->interface_name, mhi_dev->dev_id,
			 mhi_dev->domain, mhi_dev->bus, mhi_dev->slot,
			 mhi_netdev->alias);
		mhi_netdev->ipc_log = ipc_log_context_create(IPC_LOG_PAGES,
							     node_name, 0);
		mhi_netdev->ipc_log_lvl = IPC_LOG_LVL;

		mhi_netdev_create_debugfs(mhi_netdev);
	}

	/* move mhi channels to start state */
	ret = mhi_prepare_for_transfer(mhi_dev);
	if (ret) {
		MSG_ERR("Failed to start channels ret %d\n", ret);
		goto error_start;
	}

	/* setup pool size ~2x ring length*/
	nr_tre = mhi_get_no_free_descriptors(mhi_dev, DMA_FROM_DEVICE);
	mhi_netdev->rx_pool_size = 1 << __ilog2_u32(nr_tre);
	if (nr_tre > mhi_netdev->rx_pool_size)
		mhi_netdev->rx_pool_size <<= 1;
	//mhi_netdev->pool_size <<= 1;
	MSG_LOG("rx pool size %d\n", mhi_netdev->rx_pool_size);

	/* setup pool size ~2x ring length*/
	nr_tre = mhi_get_no_free_descriptors(mhi_dev, DMA_TO_DEVICE);
	mhi_netdev->tx_pool_size = 1 << __ilog2_u32(nr_tre);
	if (nr_tre > mhi_netdev->tx_pool_size)
		mhi_netdev->tx_pool_size <<= 1;
	//mhi_netdev->pool_size <<= 1;
	MSG_LOG("tx pool size %d\n", mhi_netdev->tx_pool_size);

	/* allocate memory pool */
	ret = mhi_netdev_alloc_pool(mhi_netdev);
	if (ret)
		goto error_start;

	/* link child node with parent node if it's children dev */
	if (p_netdev)
		p_netdev->rsc_dev = mhi_netdev;

	/* now we have a pool of buffers allocated, queue to hardware
	 * by triggering a napi_poll
	 */
	napi_schedule(mhi_netdev->napi);

	spin_lock_init(&mhi_netdev->tx_lock);

	mhi_netdev->timer_interval = CDC_NCM_TIMER_INTERVAL_USEC * NSEC_PER_USEC;

	atomic_set(&mhi_netdev->stop, 0);

	/* set the tx timer */
	hrtimer_init(&mhi_netdev->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mhi_netdev->tx_timer.function = &mhi_netdev_tx_timer_cb;

    tasklet_init(&mhi_netdev->bh, mhi_netdev_txpath_bh, (unsigned long)mhi_netdev);

    MSG_DEFAULT("success\n");

	return 0;

error_start:
	if (phandle)
		return ret;

	netif_stop_queue(mhi_netdev->ndev);
	napi_disable(mhi_netdev->napi);
	unregister_netdev(mhi_netdev->ndev);
	netif_napi_del(mhi_netdev->napi);
	free_netdev(mhi_netdev->ndev);

    MSG_DEFAULT("fail!\n");
	return ret;
}

static const struct mhi_device_id mhi_netdev_match_table[] = {
	{ .chan = "IP_HW0" },
	{ .chan = "IP_HW0_RSC" },
	{},
};
MODULE_DEVICE_TABLE(mhi, mhi_netdev_match_table);

static struct mhi_driver mhi_netdev_driver = {
	.id_table = mhi_netdev_match_table,
	.probe = mhi_netdev_probe,
	.remove = mhi_netdev_remove,
	.ul_xfer_cb = mhi_netdev_xfer_ul_cb,
	.dl_xfer_cb = mhi_netdev_xfer_dl_cb,
	.status_cb = mhi_netdev_status_cb,
	.driver = {
		.name = "mhi_netdev",
		.owner = THIS_MODULE,
	}
};

/**
 * @brief module init
 */
static int __init mhi_netdev_init(void)
{
    int ret;

	BUILD_BUG_ON(sizeof(struct mhi_netbuf) > MAX_NETBUF_SIZE);

    MSG_DEFAULT("enter\n");

	mhi_netdev_create_debugfs_dir();

    ret = mhi_driver_register(&mhi_netdev_driver);

	return ret;
}

/**
 * @brief module exit
 */
static void __exit mhi_netdev_exit(void)
{
    MSG_DEFAULT("enter\n");

    mhi_driver_unregister(&mhi_netdev_driver);
}

module_init(mhi_netdev_init);
module_exit(mhi_netdev_exit);

MODULE_DESCRIPTION("MHI NETDEV Network Interface");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.9.2101.1");
