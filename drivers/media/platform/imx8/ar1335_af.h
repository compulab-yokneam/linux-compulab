/*
 * ar1335.h - ar1335 sensor mode tables
 *
 * Copyright (c) 2018-2019, e-con Systems, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AR1335_TABLES__
#define __AR1335_TABLES__

#define AR1335_XCLK_MIN 6000000
#define AR1335_XCLK_MAX 24000000

#define ar1335_reg struct reg_16
#define AR1335_TABLE_WAIT_MS	0
#define AR1335_TABLE_END	1
#define AR1335_WAIT_MS		10
#define AR1335_DEFAULT_MODE	0

#define AR1335_DEFAULT_WIDTH    640
#define AR1335_DEFAULT_HEIGHT	480

#define AR1335_MAX_FORMAT_SUPPORTED	1

#define AR1335_DEFAULT_FPS	120

#define AR1335_DEFAULT_DATAFMT	MEDIA_BUS_FMT_UYVY8_2X8
#define AR1335_DEFAULT_DATAFMT_YUYV  MEDIA_BUS_FMT_YUYV8_2X8
#define AR1335_DEFAULT_COLORSPACE V4L2_COLORSPACE_SRGB


////////////////////////////////////////////////////////////////////////////////////////////

/* Defines related to MCU */

#define CMD_SIGNATURE		0x43
#define TX_LEN_PKT			5
#define RX_LEN_PKT			6
#define HEADER_FOOTER_SIZE	4
#define CMD_STATUS_MSG_LEN	7

#define VERSION_SIZE			32
#define VERSION_FILE_OFFSET			100

#define MCU_CMD_STATUS_SUCCESS		0x0000
#define MCU_CMD_STATUS_PENDING		0xF000
#define MCU_CMD_STATUS_ISP_PWDN		0x0FF0
#define MCU_CMD_STATUS_ISP_UNINIT	0x0FF1

#define MAX_NUM_FRATES                 10
#define MAX_CTRL_DATA_LEN 		100
#define MAX_CTRL_UI_STRING_LEN 		32

typedef enum _errno {
	ERRCODE_SUCCESS = 0x00,
	ERRCODE_BUSY = 0x01,
	ERRCODE_INVAL = 0x02,
	ERRCODE_PERM = 0x03,
	ERRCODE_NODEV = 0x04,
	ERRCODE_IO = 0x05,
	ERRCODE_HW_SPEC = 0x06,
	ERRCODE_AGAIN = 0x07,
	ERRCODE_ALREADY = 0x08,
	ERRCODE_NOTIMPL = 0x09,
	ERRCODE_RANGE = 0x0A,

	/*   Reserved 0x0B - 0xFE */

	ERRCODE_UNKNOWN = 0xFF,
} RETCODE;

typedef enum _cmd_id {
	CMD_ID_VERSION = 0x00,
	CMD_ID_GET_SENSOR_ID = 0x01,
	CMD_ID_GET_STREAM_INFO = 0x02,
	CMD_ID_GET_CTRL_INFO = 0x03,
	CMD_ID_INIT_CAM = 0x04,
	CMD_ID_GET_STATUS = 0x05,
	CMD_ID_DE_INIT_CAM = 0x06,
	CMD_ID_STREAM_ON = 0x07,
	CMD_ID_STREAM_OFF = 0x08,
	CMD_ID_STREAM_CONFIG = 0x09,
	CMD_ID_GET_CTRL_UI_INFO = 0x0A,

	/* Reserved 0x0B to 0x0F */

	CMD_ID_GET_CTRL = 0x10,
	CMD_ID_SET_CTRL = 0x11,

	/* Reserved 0x12, 0x13 */

	CMD_ID_FW_UPDT = 0x14,
	CMD_ID_ISP_PDOWN = 0x15,
	CMD_ID_ISP_PUP = 0x16,

	/* Reserved - 0x17 to 0xFE (except 0x43) */

	CMD_ID_UNKNOWN = 0xFF,

} HOST_CMD_ID;

enum {
	FRAME_RATE_DISCRETE = 0x01,
	FRAME_RATE_CONTINOUS = 0x02,
};

enum {
	CTRL_STANDARD = 0x01,
	CTRL_EXTENDED = 0x02,
};

enum {
/*  0x01 - Integer (32bit)
		0x02 - Long Int (64 bit)
		0x03 - String
		0x04 - Pointer to a 1-Byte Array
		0x05 - Pointer to a 2-Byte Array
		0x06 - Pointer to a 4-Byte Array
		0x07 - Pointer to Generic Data (custom Array)
*/

	EXT_CTRL_TYPE_INTEGER = 0x01,
	EXT_CTRL_TYPE_LONG = 0x02,
	EXT_CTRL_TYPE_STRING = 0x03,
	EXT_CTRL_TYPE_PTR8 = 0x04,
	EXT_CTRL_TYPE_PTR16 = 0x05,
	EXT_CTRL_TYPE_PTR32 = 0x06,
	EXT_CTRL_TYPE_VOID = 0x07,
};

/* Stream and Control Info Struct */
typedef struct _isp_stream_info {
	uint32_t fmt_fourcc;
	uint16_t width;
	uint16_t height;
	uint8_t frame_rate_type;
	union {
		struct {
			uint16_t frame_rate_num;
			uint16_t frame_rate_denom;
		} disc;
		struct {
			uint16_t frame_rate_min_num;
			uint16_t frame_rate_min_denom;
			uint16_t frame_rate_max_num;
			uint16_t frame_rate_max_denom;
			uint16_t frame_rate_step_num;
			uint16_t frame_rate_step_denom;
		} cont;
	} frame_rate;
} ISP_STREAM_INFO;


typedef struct _isp_ctrl_ui_info {
	struct {
		char ctrl_name[MAX_CTRL_UI_STRING_LEN];
		uint8_t ctrl_ui_type;
		uint8_t ctrl_ui_flags;
	} ctrl_ui_info;

	/* This Struct is valid only if ctrl_ui_type = 0x03 */
	struct {
		uint8_t num_menu_elem;
		char **menu;
	} ctrl_menu_info;
} ISP_CTRL_UI_INFO;

typedef struct _isp_ctrl_info_std {
	uint32_t ctrl_id;
	uint8_t ctrl_type;
	uint8_t mcu_ctrl_index;
	union {
		struct {
			int32_t ctrl_min;
			int32_t ctrl_max;
			int32_t ctrl_def;
			int32_t ctrl_step;
		} std;
		struct {
			uint8_t val_type;
			uint32_t val_length;
			// This size may vary according to ctrl types
			uint8_t val_data[MAX_CTRL_DATA_LEN];
		} ext;
	} ctrl_data;
	ISP_CTRL_UI_INFO ctrl_ui_data;
} ISP_CTRL_INFO;

struct ar1335_af {
	int numctrls;
//	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	uint16_t frate_index;
	struct media_pad pads[1];

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;

	bool soc_format_check; /*Checking SOC 1 for mini , 0 for others*/
	int csi;

	/*
	 * Format related data
	 */
	struct v4l2_pix_format pix;

	struct ar1335_datafmt {
		u32 code;
		enum v4l2_colorspace colorspace;
	} fmt;

	struct v4l2_captureparm streamcap;

	bool on;

	int num_frm_fmts;

	/*
	 * Array of Camera framesizes
	 *
	 * Moved from global
	 */
	struct mcu_frmfmt {
		struct v4l2_frmsize_discrete size;
		unsigned int * framerates;
		int num_framerates;
		int mode;
	} *mcu_cam_frmfmt;

	int power_on;

	struct v4l2_ctrl *ctrls[];
};

static ISP_STREAM_INFO *stream_info = NULL;
static ISP_CTRL_INFO *mcu_ctrl_info = NULL;

/* Total formats */
static int num_ctrls = 0;
static int *streamdb;
static uint32_t *ctrldb;

/* Mutex for I2C lock */
DEFINE_MUTEX(mcu_i2c_mutex_1335_af);

static int ar1335_read(struct i2c_client *client, u8 * val, u32 count);
static int ar1335_write(struct i2c_client *client, u8 * val, u32 count);
static int ar1335_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
static int ar1335_s_power(struct v4l2_subdev *sd, int on);

/*
 * MCU related functions
 */
static int mcu_get_fw_version(struct i2c_client *client, unsigned char * fw_version);
static int mcu_verify_fw_version(const unsigned char *const fw_version);
static int mcu_count_or_list_fmts(struct i2c_client *client, ISP_STREAM_INFO *stream_info, int *frm_fmt_size);
static int mcu_count_or_list_ctrls(struct i2c_client *client,
			  ISP_CTRL_INFO * mcu_ctrl_info, int *numctrls);
static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id);
static int mcu_get_cmd_status(struct i2c_client *client, uint8_t * cmd_id,
			      uint16_t * cmd_status, uint8_t * ret_code);
static int mcu_isp_init(struct i2c_client *client);
static int mcu_stream_config(struct i2c_client *client, uint32_t format,
			     int mode, int frate_index);
static int mcu_set_ctrl(struct i2c_client *client, uint32_t ctrl_id,
			uint8_t ctrl_type, int32_t curr_val);
static int mcu_get_ctrl(struct i2c_client *client, uint32_t ctrl_id,
			uint8_t * ctrl_type, int32_t * curr_val);
static int mcu_get_ctrl_ui(struct i2c_client *client,
			   ISP_CTRL_INFO * mcu_ui_info, int index);
static int mcu_fw_update(struct i2c_client *client, unsigned char *txt_fw_version);
static int mcu_isp_power_down(struct i2c_client *client);
static int mcu_isp_power_wakeup(struct i2c_client *client);

#endif				/* __AR1335_TABLES__ */
