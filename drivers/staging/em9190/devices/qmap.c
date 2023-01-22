#include <linux/delay.h>
#include <linux/kthread.h>

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
#include <linux/hrtimer.h>

#include "../inc/ipc_logging.h"
#include "../inc/mhi.h"
#include "../core/mhi_internal.h"
#include "mhi_uci.h"
#include "qmap.h"

/* This is in preferred order from top to bottom */
QMAP_DATA_FMT data_fmts_supported[] = 
{
    QMAPV1,
    QMAPV4,
    QMAPV5
};

const u8 mbim_extensibility_guid[16] = {
	0x2d, 0x0c, 0x12, 0xc9, 0x0e, 0x6a, 0x49, 0x5a, 0x91, 0x5c, 0x8d, 0x17, 0x4f, 0xe5, 0xd6, 0x3c
};

const u8 mbim_extensibility_guid_new[16] = {
	0xcf, 0xd4, 0x97, 0xea, 0xf0, 0xae, 0x4f, 0x71, 0x9a, 0xca, 0x14, 0x26, 0x6e, 0xae, 0xf1, 0x98
};

/**
 * @brief Utility to find the max supported QMAP Aggregation formats
 */
u32 qmap_get_max_supproted_data_fmts(void)
{
    u32 num_supported_aggr = sizeof(data_fmts_supported) / sizeof(data_fmts_supported[0]);
    return num_supported_aggr;
}

/**
 * @brief Qmap API to get the Information buffer size needed for DataFmt list
 */
u32 qmap_get_buf_size_reqd_for_data_fmt(struct mhi_device *mhi_dev)
{
    u32 num_supported_aggr = qmap_get_max_supproted_data_fmts();
    u32 buf_size = (num_supported_aggr * sizeof(QMAP_DATA_FMT_PARAMS)) +
           sizeof(QMAP_DATA_FMT_REQ_OFFSET_LEN)* num_supported_aggr +
           sizeof(QMAP_DATA_FMT_REQ);
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);

	MSG_LOG("bufSize = %d\n", buf_size);

	return buf_size;
}

/**
 * @brief Qmap API to get the list of DataFormats supported
 *
 * @param[out]  buf       Buffer address of the elements stored consecutively
 */
void qmap_get_data_format_list(u8 *buf)
{
    u8 *info_buf = buf;
    u32 num_supported_aggr = qmap_get_max_supproted_data_fmts();
    PQMAP_DATA_FMT_REQ_OFFSET_LEN offset_len_pair;
    u32 data_buf_offset;
    PQMAP_DATA_FMT_PARAMS data_fmt_params = NULL;
    u32 idx;

    /* Preference order of QMAP aggregation list */
    /*u32 buf_size = (num_supported_aggr * sizeof(QMAP_DATA_FMT_PARAMS)) +
        sizeof(QMAP_DATA_FMT_REQ_OFFSET_LEN) * num_supported_aggr +
        sizeof(QMAP_DATA_FMT_REQ); */
#define RAW_IP_MODE (1)

    /* Begin Info buffer update */
    PQMAP_DATA_FMT_REQ req = (PQMAP_DATA_FMT_REQ)info_buf;

	req->version = 1;     /* Support version 1 */
    req->element_count = num_supported_aggr;

    /* ProviderReflist */
    offset_len_pair = (PQMAP_DATA_FMT_REQ_OFFSET_LEN)(info_buf + sizeof(QMAP_DATA_FMT_REQ));
    data_buf_offset = (sizeof(QMAP_DATA_FMT_REQ_OFFSET_LEN) * num_supported_aggr) + sizeof(QMAP_DATA_FMT_REQ);

    for (idx = 0; idx < num_supported_aggr; idx++)
    {
		offset_len_pair->offset = data_buf_offset;
		offset_len_pair->length = sizeof(QMAP_DATA_FMT_PARAMS);

        /* Fill DataFmt */
		data_fmt_params = (PQMAP_DATA_FMT_PARAMS)(info_buf + data_buf_offset);
		data_fmt_params->dl_data_agg_max_datagrams = MAX_DL_PKT_AGGR;
		data_fmt_params->dl_data_agg_max_size = QMAP_RX_BUFFER_SIZE;
		data_fmt_params->ul_data_agg_max_datagrams = MAX_UL_PKT_AGGR;
		data_fmt_params->ul_data_agg_max_size = QMAP_TX_BUFFER_SIZE;
		//data_fmt_params->ul_data_agg_max_datagrams = ;
		//data_fmt_params->ul_data_agg_max_size = 0;
		data_fmt_params->link_prot = RAW_IP_MODE;
		data_fmt_params->tcp_udp_coalescing = 0;

		data_fmt_params->ul_data_agg_protocol = data_fmts_supported[idx];
		data_fmt_params->dl_data_agg_protocol = data_fmts_supported[idx];

        /* Update the data buffer offset & OL pair provider offset */
        data_buf_offset += sizeof(QMAP_DATA_FMT_PARAMS);
		offset_len_pair = (PQMAP_DATA_FMT_REQ_OFFSET_LEN)((u8 *)offset_len_pair + sizeof(QMAP_DATA_FMT_REQ_OFFSET_LEN));
    }

    return;
}

/**
 * @brief handle MBIM incoming message
 *
 * @param[in]  device_info       device information
 * @param[in]  buf               MBIM messsage buffer
 * @param[in]  len               MBIM messsage length
 */
bool
mbim_rx_cid(struct mhi_device *mhi_dev, u8 *buf, unsigned long len)
{
	struct mbim_message_header *header;
	struct mbim_open_done_message *done_msg;
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);

	if (len < sizeof(struct mbim_message_header)) {
		MSG_LOG("invalid mbim message len %ld\n", len);
		return false;
	}

	header = (struct mbim_message_header *)buf;

	switch (header->message_type) {
	case MBIM_OPEN_DONE:
		MSG_LOG("MBIM_OPEN_DONE message received!\n");
		if (len != sizeof(struct mbim_open_done_message)) {
			MSG_LOG("invalid mbim open done message len %ld\n", len);
			return false;
		}

		done_msg = (struct mbim_open_done_message *)buf;

		if (done_msg->status == MBIM_STATUS_SUCCESS) {
			MSG_LOG("set the data format as QMAP!\n");
			mbim_set_data_format(mhi_dev, header->transaction_id, false);
		}

		break;

	case MBIM_COMMAND_DONE:
	{
		struct mbim_command_done_msg *cmd_msg_done = NULL;

		MSG_LOG("MBIM_COMMAND_DONE message received!\n");
		if (NULL == buf || len < sizeof(struct mbim_command_done_msg))
		{
			MSG_ERR("Invalid argument buf %p len %lu\n", buf, len);
			return false;
		}

		MSG_LOG("dump MBIM_COMMAND_DONE msg\n");
#if 0
        print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
            buf, len, true);
#endif

		cmd_msg_done = (struct mbim_command_done_msg *)buf;

		if ((memcmp(cmd_msg_done->device_service_id, mbim_extensibility_guid, sizeof(cmd_msg_done->device_service_id)) == 0) &&
			(MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT == cmd_msg_done->cid) &&
            (uci_dev->set_data_fmt_tid == cmd_msg_done->message_header.transaction_id)) {

			MSG_LOG("Process MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT response\n");

			data_format_packet_handling(mhi_dev, buf, len, false);

            return true;
		}

		if ((memcmp(cmd_msg_done->device_service_id, mbim_extensibility_guid_new, sizeof(cmd_msg_done->device_service_id)) == 0) &&
			(MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT_NEW == cmd_msg_done->cid) &&
            (uci_dev->set_data_fmt_tid == cmd_msg_done->message_header.transaction_id)) {

			MSG_LOG("Process MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT_NEW response\n");

			data_format_packet_handling(mhi_dev, buf, len, true);

            return true;
		}
	}
	break;
	}

	return false;
}

/**
 * @brief compose and send set data format message to modem
 *
 * @param[in]  device_info       device information
 * @param[in]  transaction_id    MBIM message tid
 */
int mbim_set_data_format(struct mhi_device *mhi_dev, u32 transaction_id, bool new)
{
	u32 payload = 0;
	u32 alloc = 0;
	u8 *buf;
	unsigned long buf_len;
	u32 tid = transaction_id;
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);
    int ret = -EIO;

	MSG_LOG("MBIM_OPEN_DONE_MSG received, sending MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT to modem\n");

	payload = qmap_get_buf_size_reqd_for_data_fmt(mhi_dev);
	if (0 == payload) {
		MSG_ERR("returned 0 bytes transaction ID %u\n",	transaction_id);
		return -EINVAL;
	}
	else {
		alloc = payload + sizeof(struct mbim_command_msg);

		buf = kzalloc(alloc, GFP_KERNEL);

		if (buf == NULL) {
			MSG_ERR("mbim_set_data_format failed to allocate %u bytes\n", alloc);
			return -ENOMEM;
		}
		else {
			qmap_get_data_format_list(buf + sizeof(struct mbim_command_msg));

			if (!new)
 				tid = transaction_id ^ 0x80000000;
				
			if (tid == 0)
				tid = 1;

            uci_dev->set_data_fmt_tid = tid;

            add_data_format_header(mhi_dev, buf, alloc, tid, new);

            buf_len = alloc;

            MSG_LOG("dump qmap set data format msg\n");
#if 0
            print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
	            buf, buf_len, true);
#endif
            ret = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, buf,
                                     buf_len, MHI_EOT);

            MSG_LOG("mhi_queue_transfer flags %d ret 0x%x\n", MHI_EOT, ret);
        }
    }

	return 0;
}

/**
 * @brief add message header for set data format message
 *
 * @param[in]  buf               MBIM messsage buffer
 * @param[in]  len               MBIM messsage length
 * @param[in]  transaction_id    MBIM message tid
 */
int add_data_format_header(struct mhi_device *mhi_dev, u8 *buf, u32 len, u32 transaction_id, bool new)
{
	struct mbim_command_msg *header = NULL;
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);

	if (NULL == buf || len < sizeof(struct mbim_command_msg))
	{
		MSG_ERR("add_data_format_header invalid argument!\n");
		return -EINVAL;
	}

	header = (struct mbim_command_msg *)buf;

	header->message_header.message_type = MBIM_COMMAND;
	header->message_header.transaction_id = transaction_id;
	header->message_header.message_length = len;

    header->fragment_header.total_fragments = 1;
    header->fragment_header.current_fragment = 0;

	if (!new) 
		header->cid = MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT;
    else
		header->cid = MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT_NEW;

	header->command_type = MBIM_SET_OPERATION;

	if (!new) 
		memcpy(&header->device_service_id, mbim_extensibility_guid, sizeof(header->device_service_id));
	else 
		memcpy(&header->device_service_id, mbim_extensibility_guid_new, sizeof(header->device_service_id));

	header->information_buffer_length = len - sizeof(struct mbim_command_msg);

	return 0;
}

/**
 * @brief hanlder for set data format response
 *
 * @param[in]  buf               MBIM messsage buffer
 * @param[in]  len               MBIM messsage length
 * @param[in]  transaction_id    MBIM message tid
 */
int data_format_packet_handling(struct mhi_device *mhi_dev, u8 *buf, unsigned long len, bool new)
{
	struct mbim_command_done_msg *cmd_done = NULL;
	struct uci_dev *uci_dev = mhi_device_get_devdata(mhi_dev);

	cmd_done = (struct mbim_command_done_msg *)buf;

	if (MBIM_STATUS_SUCCESS == cmd_done->status) {

		MSG_LOG("set data format request success!\n");

		return 0;
	} else {

		MSG_ERR("set data format request failed 0x%x!\n", cmd_done->status);

        if (!new && MBIM_STATUS_NO_DEVICE_SUPPORT == cmd_done->status) {
		    MSG_LOG("device does not support, try the new one!\n");

            mbim_set_data_format(mhi_dev, cmd_done->message_header.transaction_id + 1, true);
        }
    }

	return -EINVAL;
}
