/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Based on V4L2 OmniVision OV5647 Image Sensor driver
 * Copyright (C) 2016 Ramiro Oliveira <roliveir@synopsys.com>
 *
 */


#include <linux/v4l2-controls.h>
#include <linux/types.h>

static int mcu_cam_stream_on(struct i2c_client *client);

static int mcu_get_cmd_status(struct i2c_client *client,uint8_t * cmd_id, uint16_t * cmd_status,uint8_t * ret_code);


static int mcu_stream_config(struct i2c_client *client, uint32_t format,int mode, int frate_index);

//-------------------------------------------------

#ifndef __AR1335_TABLES__
#define __AR1335_TABLES__

#define MCU_SLAVE_ADDRESS                0x42            
#define IO_EXPANDER_SLAVE_ADDRESS        0x22 

#define IO_EXP_CONFIG_PORT0           0x0C
#define IO_EXP_CONFIG_PORT1           0x0D
#define IO_EXP_CONFIG_PORT2           0x0E

#define IO_EXP_SET_PORT_OUPUT    0x00
#define IO_EXP_SET_PORT_INPUT    0xFF

#define IO_EXP_OUTPUT_PORT0     0x04 
#define IO_EXP_OUTPUT_PORT1     0x05
#define IO_EXP_OUTPUT_PORT2     0x06

#define IO_EXP_PIN0_RESET_CTRL   0x01  
#define IO_EXP_PIN6_PWDN_CTRL    0x20

/* MCU */
#define MAX_CTRL_UI_STRING_LEN		 32
#define MAX_CTRL_DATA_LEN		100
#define ar1335_reg struct reg_16
#define AR1335_TABLE_WAIT_MS	0
#define AR1335_TABLE_END	1
#define AR1335_WAIT_MS		10
#define AR1335_DEFAULT_MODE	0

#define AR1335_DEFAULT_WIDTH    1920
#define AR1335_DEFAULT_HEIGHT	1080

#define AR1335_DEFAULT_DATAFMT	MEDIA_BUS_FMT_UYVY8_2X8
#define AR1335_NUM_CONTROLS 30

/* Defines related to MCU */
#define CMD_SIGNATURE		0x43
#define TX_LEN_PKT		   5
#define RX_LEN_PKT		   6
#define HEADER_FOOTER_SIZE	   4
#define CMD_STATUS_MSG_LEN	   7

#define VERSION_SIZE			 32
#define VERSION_FILE_OFFSET		100

#define MCU_CMD_STATUS_SUCCESS		0x0000
#define MCU_CMD_STATUS_PENDING		0xF000
#define MCU_CMD_STATUS_ISP_PWDN		0x0FF0
#define MCU_CMD_STATUS_ISP_UNINIT	0x0FF1

#define MAX_NUM_FRATES			 10

/* Rock960 */
#define AR1335_NUM_SUPPLIES ARRAY_SIZE(ar1335_supply_names)

/* BS: Added Minimum camera_common_data structure
*/
struct camera_common_data {
	struct v4l2_ctrl_handler		*ctrl_handler;
	struct device				*dev;
	struct v4l2_subdev			subdev;
	int	numlanes;
	int	mode;
	int	def_mode, def_width, def_height;
	int	def_clk_freq;
	int	fmt_width, fmt_height;
	void	*sensor;
};

static const char * const ar1335_supply_names[] = {
        "avdd",         /* Analog power */
        "dovdd",        /* Digital I/O power */
        "dvdd",         /* Digital core power */
};

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

struct camera_common_frmfmt {
        struct v4l2_frmsize_discrete    size;
        const int       *framerates;
        int     num_framerates;
        bool    hdr_en;
        int     mode;
};


struct ar1335_mode {
        u32 width;
        u32 height;
        u32 pixel_clock;
        u32 link_freq;

	struct v4l2_mbus_framefmt	format;
	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	u64 pixel_rate;
	/* HTS as defined in the register set (0x380C/0x380D) */
	int hts;
	/* Default VTS value for this mode */
	int vts_def;

	struct regval_list		*reg_list;
	unsigned int			num_regs;
};

/* BS: Added Minimum ar1335 structure
*/
struct ar1335 {
/* Rock960 */
        struct clk              *xvclk;
        struct gpio_desc        *reset_gpio;
        struct gpio_desc        *pwdn_gpio;
//        struct regulator_bulk_data supplies[AR1335_NUM_SUPPLIES];

        struct device *dev;
        struct pinctrl          *pinctrl;
        struct pinctrl_state    *pins_default;
        struct v4l2_mbus_framefmt fmt;
        struct v4l2_rect        crop;
        struct mutex            mutex;
        bool                    streaming;
        const struct ar1335_mode *cur_mode;
        struct v4l2_ctrl *pixel_clock;
        struct v4l2_ctrl *link_freq;


        struct v4l2_ctrl_handler ctrl_handler;
        struct i2c_client *i2c_client;
        struct v4l2_subdev subdev;
        struct media_pad pad;

        struct camera_common_data *s_data;
        int ident;
        u16 chip_id;
        u8 revision;

        uint32_t mode_index;
        uint16_t frate_index;
        uint32_t format_fourcc;
	int numfmts;
        int frmfmt_mode;
        int num_ctrls;
        ISP_STREAM_INFO *stream_info;
        ISP_CTRL_INFO *mcu_ctrl_info;
        /* Total formats */
        int *streamdb;
        uint32_t *ctrldb;
        /* Array of Camera framesizes */

//----------
	struct v4l2_subdev		sd;
	struct mutex			lock;
	const struct ar1335_mode	*mode;
	int				power_on;
	struct clk			*xclk;
	struct gpio_desc		*pwdn;
	unsigned int			flags;
	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_ctrl		*pixel_rate;
	struct v4l2_ctrl		*hblank;
	struct v4l2_ctrl		*vblank;
	struct v4l2_ctrl		*exposure;
	bool				write_mode_regs;
	uint8_t                        mipi_lane_config;
//-------------------------

        struct camera_common_frmfmt *mcu_cam_frmfmt;
        uint16_t prev_index;
        uint8_t last_sync_mode;
	struct v4l2_ctrl *ctrl[];
};

enum {
        MODE_640x480 = 0,
        MODE_1280x720,
        MODE_1280x960,
        MODE_1920x1080,
	MODE_2560x1440,
	MODE_2592x1944,
        MODE_UNKNOWN,
};

/* Mutex for I2C lock */
DEFINE_MUTEX(mcu_i2c_mutex_1335_mcu);

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

	/* Configuring MIPI Lanes */
	CMD_ID_LANE_CONFIG = 0x17,

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



#endif 

