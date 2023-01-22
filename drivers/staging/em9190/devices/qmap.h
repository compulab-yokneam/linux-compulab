
#ifndef QMAP_H_
#define QMAP_H_


// ============================================================================
// Internal Use
// ============================================================================
#define QUALCOMM_DEVICE_TYPE_QMAP       (0x40008)
#define QMAP_FUNCTION_CODE_BASE         (0x800)

#define HW_BUFFER_ALIGNMENT             (0x0000000f)
#define QMAP_CKSUM_TRAILER              (8)
#define QMAP_LEN_ALIGN                  (4)
#define QMAP_CLIENT_MAX                 (17)
#define MAX_DATA_FRAGMENT_SIZE          (0x600)

#define MAX_OS_NET_PACKETS              (512)
#define MAX_OS_NET_PACKETS_UL           (MAX_OS_NET_PACKETS)
#define MAX_OS_NET_PACKETS_DL           (MAX_OS_NET_PACKETS)

#define MHI_RING_LENGTH                 (256)
#define MHI_UL_RING_LENGTH              (MHI_RING_LENGTH)
#define MHI_DL_RING_LENGTH              (MHI_RING_LENGTH)

#define MAX_MHI_DESCRIPTORS             (250)
#define MAX_MHI_UL_DESCRIPTORS          (MAX_MHI_DESCRIPTORS)
#define MAX_MHI_DL_DESCRIPTORS          (MAX_MHI_DESCRIPTORS)

#define MAX_DL_PKT_AGGR                 (10)
#define MAX_UL_PKT_AGGR                 (10)
#define QMAP_TX_BUFFER_SIZE             0x4000	// 16K must be multiplication of page size (MAX_DATA_FRAGMENT_SIZE * MAX_UL_PKT_AGGR)
#define QMAP_RX_BUFFER_SIZE             0x4000	// 16K must be multiplication of page size (MAX_DATA_FRAGMENT_SIZE * MAX_DL_PKT_AGGR)
#define MAX_CLIENT_PKT_CONTEXT          (MAX_MHI_DL_DESCRIPTORS * MAX_DL_PKT_AGGR)

#define QUALCOMM_MAC_0                 (0x00)
#define QUALCOMM_MAC_1                 (0xA0)
#define QUALCOMM_MAC_2                 (0xC6)
#define QUALCOMM_MAC_3                 (0x00)
#define QUALCOMM_MAC_4                 (0x00)
// 5 - this need either be static or global for multiple adapter support. 
#define QUALCOMM_MAC_5                 (0x01)

#define QMAPV1_TX_HDR_LEN                  (4)
#define QMAPV1_RX_HDR_LEN                  (4)
#define QMAPV5_TX_HDR_LEN                  (8)
#define QMAPV5_RX_HDR_LEN                  (8)
#define QMAPV4_TX_HDR_LEN                  (8)
#define QMAPV4_RX_HDR_LEN                  (4)
#define QMAPV4_RX_TRL_LEN                  (QMAP_CKSUM_TRAILER)
#define QMAP_TX_CKSUM_OFFSET               (4)

#define QMAP_V4_FRAG_BITS      (0x02FFF) //includes the MF bit
#define QMAP_V4_FRAG_MSB       (6)
#define QMAP_V4_FRAG_LSB       (7)
#define QMAP_V4_PROTO_BYTE     (9)

#define QMAP_V4_FRAG_PKT(pkt) (0 != (QMAP_V4_FRAG_BITS & (((pkt)[QMAP_V4_FRAG_MSB] << 8) | (pkt)[QMAP_V4_FRAG_LSB])))

#define QMAP_V6_FRAG_EXT_HDR   (44)
#define QMAP_V6_NEXT_HDR_BYTE  (6)

#define QMAP_V6_FRAG_PKT(pkt) (QMAP_V6_FRAG_EXT_HDR == (pkt)[QMAP_V6_NEXT_HDR_BYTE])

#define QMAP_V4_NON_FRAG_PKT(pkt) (0 == (QMAP_V4_FRAG_BITS & (((pkt)[QMAP_V4_FRAG_MSB] << 8) | (pkt)[QMAP_V4_FRAG_LSB])))
#define QMAP_V4_TCP_PKT(pkt) (QMAP_IPPROTO_TCP == (pkt)[QMAP_V4_PROTO_BYTE])
#define QMAP_V4_UDP_PKT(pkt) (QMAP_IPPROTO_UDP == (pkt)[QMAP_V4_PROTO_BYTE])
#define QMAP_V4_XSUM_OFFLOAD_PKT_TYPE(pkt)  ((QMAP_V4_TCP_PKT(pkt) || QMAP_V4_UDP_PKT(pkt)) && QMAP_V4_NON_FRAG_PKT(pkt))

#define QMAP_V6_TCP_PKT(pkt) (QMAP_IPPROTO_TCP == (pkt)[QMAP_V6_NEXT_HDR_BYTE])
#define QMAP_V6_UDP_PKT(pkt) (QMAP_IPPROTO_UDP == (pkt)[QMAP_V6_NEXT_HDR_BYTE])
#define QMAP_V6_NON_FRAG_PKT(pkt) (QMAP_V6_FRAG_EXT_HDR != (pkt)[QMAP_V6_NEXT_HDR_BYTE])
#define QMAP_V6_XSUM_OFFLOAD_PKT_TYPE(pkt)  ((QMAP_V6_TCP_PKT(pkt) || QMAP_V6_UDP_PKT(pkt)) && QMAP_V6_NON_FRAG_PKT(pkt))

#define MAX_RX_CLIENT_DESC             (512)
#define DEFAULT_UL_INDEX               (MHI_UL_RING_LENGTH - 1) /* To start at index 0 when it increments */
#define DEFAULT_DL_INDEX               (MHI_DL_RING_LENGTH - 1) /* To start at index 0 when it increments */

#define MAX_RSS_INDIRECTION_ENTRIES    (128)
#define MAX_QUEUES_PER_DIR             (4)
#define MAX_DIR                        (2) /* UL and DL */

#define PEND_NOTIFY_ERR_MARGIN  (0xFFFFFFFD)

#define QMAP_IPPROTO_TCP  (6)
#define QMAP_IPPROTO_UDP  (17)
#define QMAP_IPPROTO_IPV6_HDR_LEN (40)
#define QMAP_IPPROTO_IPV4 (4)
#define QMAP_IPPROTO_IPV6 (6)

#define QMAP_MIN_IPV4_HDR_LEN            (20)
#define QMAP_MIN_IPV6_HDR_LEN            (40)

#define TCP_CKSUM_OFFSET 16
#define UDP_CKSUM_OFFSET 6
#define NUMBER_OF_32BIT_WORDS 4

#define IP_VER(packet)                  ((packet[0] & 0xF0) >> 4)
#define IPV4_HDR_LEN(packet)            ((packet[0] & 0x0F) * 4);

/*! @brief QMAP header struct definition after ntohl conversion */
typedef struct _QMAP_HEADER
{
    u8       packet_info;
    u8       mux_id; // this should be same as channel
    u16      length;
    /*
    Bit 0       =   C/D bit
    1 = QMAP control command
    0 = Data packet

    Bit 1       =   Reserved (Set to 0)
    Must be ignored by the receiver


    Bit 2-7     =   PAD (6 bits; Indicates number of bytes padded to achieve a minimum of 4 byte alignment)
    Padded bytes may or may not be set to 0. The receiver must ignore these bytes regardless.

    Bit 8-15    =   MUX_ID - 8 bits; Indicates MUX channel ID
    If MUX is not negotiated, this field is set to 0 and must be ignored by the receiver
    If MUX is negotiated, MUX_ID can be in the range of 1 to 0xF0. MUX_IDs 0 and 0xFA - 0xFF are reserved.

    Bit 16-31   =   PAYLOAD_LEN_WITH_PADDING: 16 bits; Total payload length in bytes including padding
    Doesn�t include QMAP header
    The receiver must ignore a packet with this field set to 0 as hardware may use such special QMAP packet to indicate end of aggregation.
    QMAP packet will not carry any payload in this case
    */
} QMAP_HEADER, *PQMAP_HEADER;

/* Data format negotiation parameters */
#define AGGR_NOT_SUPPORTED      (0)

/* This should always be defined in order of least preferred to most preferred Aggr type */
typedef enum
{
    QMAPV1 = 1,
    QMAPV2 = AGGR_NOT_SUPPORTED,
    QMAPV3 = AGGR_NOT_SUPPORTED,
    QMAPV4 = 4,
    QMAPV5 = 5,
    QMAP_MAX_SUPP_AGGR
}QMAP_DATA_FMT;

// All structures will be 1-byte aligned.
#pragma pack(push, 1)
typedef struct 
{
    u32 offset;
    u32 length;
} QMAP_DATA_FMT_REQ_OFFSET_LEN, * PQMAP_DATA_FMT_REQ_OFFSET_LEN;

typedef struct
{
    u32 version;
    u32 element_count;
    /*! @note Followed by element_count instances of QMAP_DATA_FMT_REQ_OFFSET_LEN */
} QMAP_DATA_FMT_REQ, * PQMAP_DATA_FMT_REQ;

typedef struct 
{
    u32 link_prot;
    u32 dl_data_agg_protocol;
    u32 dl_data_agg_max_datagrams;
    u32 dl_data_agg_max_size;
	u32 ul_data_agg_protocol;
	u32 ul_data_agg_max_datagrams;
	u32 ul_data_agg_max_size;
	u32 tcp_udp_coalescing;
} QMAP_DATA_FMT_PARAMS, * PQMAP_DATA_FMT_PARAMS;
#pragma pack(pop)

typedef enum
{
	MBIM_INVALID_MSG = 0,
	MBIM_OPEN = 0x1,
	MBIM_CLOSE = 0x2,
	MBIM_COMMAND = 0x3,
	MBIM_HOST_ERROR = 0x4,
	MBIM_OPEN_DONE = 0x80000001,
	MBIM_CLOSE_DONE = 0x80000002,
	MBIM_COMMAND_DONE = 0x80000003,
	MBIM_FUNCTION_ERROR = 0x80000004,
	MBIM_INDICATE_STATUS = 0x80000007
} MBIM_MSG_ID;

typedef enum
{
	MBIM_CID_MBIM_EXTENSIBILITY_DIAG_CONFIG = 1,
	MBIM_CID_MBIM_EXTENSIBILITY_DIAG_DATA = 2,
	MBIM_CID_MBIM_EXTENSIBILITY_DEVICE_LOG = 3,
	MBIM_CID_MBIM_EXTENSIBILITY_SIMLOCK = 4,
	MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT = 5
} MBIM_SERVICE_MBIM_EXTENSIBILITY_CID_ID;

typedef enum
{
	MBIM_CID_MBIM_EXTENSIBILITY_QNET_DATA_FORMAT_NEW = 1
} MBIM_SERVICE_MBIM_EXTENSIBILITY_CID_ID_NEW;

typedef enum
{
	MBIM_QUERY_OPERATION = 0,
	MBIM_SET_OPERATION = 1
} MBIM_COMMAND_TYPE;

typedef enum
{
	MBIM_STATUS_SUCCESS = 0,
	MBIM_STATUS_BUSY = 1,
	MBIM_STATUS_FAILURE = 2,
	MBIM_STATUS_SIM_NOT_INSERTED = 3,
	MBIM_STATUS_BAD_SIM = 4,
	MBIM_STATUS_PIN_REQUIRED = 5,
	MBIM_STATUS_PIN_DISABLED = 6,
	MBIM_STATUS_NOT_REGISTERED = 7,
	MBIM_STATUS_PROVIDERS_NOT_FOUND = 8,
	MBIM_STATUS_NO_DEVICE_SUPPORT = 9,
	MBIM_STATUS_PROVIDER_NOT_VISIBLE = 10,
	MBIM_STATUS_DATA_CLASS_NOT_AVAILABLE = 11,
	MBIM_STATUS_PACKET_SERVICE_DETACHED = 12,
	MBIM_STATUS_MAX_ACTIVATED_CONTEXTS = 13,
	MBIM_STATUS_NOT_INITIALIZED = 14,
	MBIM_STATUS_VOICE_CALL_IN_PROGRESS = 15,
	MBIM_STATUS_CONTEXT_NOT_ACTIVATED = 16,
	MBIM_STATUS_SERVICE_NOT_ACTIVATED = 17,
	MBIM_STATUS_INVALID_ACCESS_STRING = 18,
	MBIM_STATUS_INVALID_USER_NAME_PWD = 19,
	MBIM_STATUS_RADIO_POWER_OFF = 20,
	MBIM_STATUS_INVALID_PARAMETERS = 21,
	MBIM_STATUS_READ_FAILURE = 22,
	MBIM_STATUS_WRITE_FAILURE = 23,
	MBIM_STATUS_NO_PHONEBOOK = 25,
	MBIM_STATUS_PARAMETER_TOO_LONG = 26,
	MBIM_STATUS_STK_BUSY = 27,
	MBIM_STATUS_OPERATION_NOT_ALLOWED = 28
} MBIM_ERROR_CODE;

struct mbim_message_header {
	u32 message_type;
	u32 message_length;
	u32 transaction_id;
} __attribute__((packed));

struct mbim_open_done_message {
	struct mbim_message_header message_header;
	u32 status;
} __attribute__((packed));

struct mbim_fragment_header {
	u32 total_fragments;
	u32 current_fragment;
} __attribute__((packed));

struct mbim_command_msg
{
	struct mbim_message_header      message_header;
	struct mbim_fragment_header     fragment_header;
	u8                      device_service_id[16];
	u32                      cid;
	u32                      command_type;
	u32                      information_buffer_length;
	// InformationBuffer follows after this;
} __attribute__((packed));

struct mbim_command_done_msg
{
	struct mbim_message_header      message_header;
	struct mbim_fragment_header     fragment_header;
	u8							device_service_id[16];
	u32							cid;
	u32							status;
	u32                      information_buffer_length;
	// InformationBuffer follows after this;
} __attribute__((packed));

bool
mbim_rx_cid(struct mhi_device *mhi_dev, u8 *buf, unsigned long len);

u32 qmap_get_max_supproted_data_fmts(void);

int mbim_set_data_format(struct mhi_device *mhi_dev, u32 transaction_id, bool new);

int add_data_format_header(struct mhi_device *mhi_dev, u8 *buf, u32 len, u32 transaction_id, bool new);

int data_format_packet_handling(struct mhi_device *mhi_dev, u8 *buf, unsigned long len, bool new);

#endif