// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX477 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
// Copyright 2018 NXP
// Copyright 2025 NXP
//
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/unaligned.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>

#define IMX477_REG_VALUE_08BIT		1
#define IMX477_REG_VALUE_16BIT		2

#define IMX477_REG_MODE_SELECT		0x0100
#define IMX477_MODE_STANDBY			0x00
#define IMX477_MODE_STREAMING		0x01

/* Chip ID */
#define IMX477_REG_CHIP_ID		0x0016
#define IMX477_CHIP_ID			0x0477

/* External clock frequency is 24M */
#define IMX477_XCLK_FREQ		24000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX477_PIXEL_RATE		840000000

#define IMX477_DEFAULT_LINK_FREQ	456000000
//#define IMX477_DEFAULT_LINK_FREQ	450000000

/* V_TIMING internal */
#define IMX477_REG_FRAME_LENGTH		0x0340
#define IMX477_FRAME_LENGTH_MAX		0xffdc

/* H_TIMING internal */
#define IMX477_REG_LINE_LENGTH		0x0342
#define IMX477_LINE_LENGTH_MAX		0xfff0

/* Long exposure multiplier */
#define IMX477_LONG_EXP_SHIFT_MAX	7
#define IMX477_LONG_EXP_SHIFT_REG	0x3100

#define IMX477_REG_VTS			0x0160
#define IMX477_VTS_15FPS		0x0dc6
#define IMX477_VTS_30FPS_1080P	0x06e3
#define IMX477_VTS_30FPS_BINNED	0x06e3
#define IMX477_VTS_30FPS_640x480 0x06e3
#define IMX477_VTS_MAX			0xfff0

#define IMX477_VBLANK_MIN		4

/*Frame Length Line*/
#define IMX477_FLL_MIN			0x08a6
#define IMX477_FLL_MAX			0xffff
#define IMX477_FLL_STEP			1
#define IMX477_FLL_DEFAULT		0x0c98

/* HBLANK control - read only */
#define IMX477_PPL_DEFAULT		3448

/* Exposure control */
#define IMX477_REG_EXPOSURE		0x0202
#define IMX477_EXPOSURE_OFFSET	22
#define IMX477_EXPOSURE_MIN		4
#define IMX477_EXPOSURE_STEP	1
#define IMX477_EXPOSURE_DEFAULT	0x640
#define IMX477_EXPOSURE_MAX		(IMX477_FRAME_LENGTH_MAX - \
					 IMX477_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX477_REG_ANALOG_GAIN	0x0204
#define IMX477_ANA_GAIN_MIN		0
#define IMX477_ANA_GAIN_MAX		976
#define IMX477_ANA_GAIN_STEP	1
#define IMX477_ANA_GAIN_DEFAULT	0x0

/* Digital gain control */
#define IMX477_REG_DIGITAL_GAIN		0x020e
#define IMX477_DGTL_GAIN_MIN		0x0100
#define IMX477_DGTL_GAIN_MAX		0x0fff
#define IMX477_DGTL_GAIN_DEFAULT	0x0100
#define IMX477_DGTL_GAIN_STEP		1

#define IMX477_REG_ORIENTATION		0x0172

/* Test Pattern Control */
#define IMX477_REG_TEST_PATTERN		0x0600
#define IMX477_TEST_PATTERN_DISABLE	0
#define IMX477_TEST_PATTERN_SOLID_COLOR	1
#define IMX477_TEST_PATTERN_COLOR_BARS	2
#define IMX477_TEST_PATTERN_GREY_COLOR	3
#define IMX477_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX477_REG_TESTP_RED		0x0602
#define IMX477_REG_TESTP_GREENR		0x0604
#define IMX477_REG_TESTP_BLUE		0x0606
#define IMX477_REG_TESTP_GREENB		0x0608
#define IMX477_TESTP_COLOUR_MIN		0
#define IMX477_TESTP_COLOUR_MAX		0x0fff
#define IMX477_TESTP_COLOUR_STEP	1
#define IMX477_TESTP_RED_DEFAULT	IMX477_TESTP_COLOUR_MAX
#define IMX477_TESTP_GREENR_DEFAULT	0
#define IMX477_TESTP_BLUE_DEFAULT	0
#define IMX477_TESTP_GREENB_DEFAULT	0

/* Trigger mode */
#define IMX477_REG_MC_MODE		0x3f0b
#define IMX477_REG_MS_SEL		0x3041
#define IMX477_REG_XVS_IO_CTRL		0x3040
#define IMX477_REG_EXTOUT_EN		0x4b81

/* Embedded metadata stream structure */
#define IMX477_EMBEDDED_LINE_WIDTH 16384
#define IMX477_NUM_EMBEDDED_LINES 1

/* IMX477 native and active pixel array size. */
#define IMX477_NATIVE_WIDTH			4072U
#define IMX477_NATIVE_HEIGHT		3072U
#define IMX477_PIXEL_ARRAY_LEFT		8U
#define IMX477_PIXEL_ARRAY_TOP		16U
#define IMX477_PIXEL_ARRAY_WIDTH	4056U
#define IMX477_PIXEL_ARRAY_HEIGHT	3040U

#define IMX477_MIN_FPS 15
#define IMX477_MAX_FPS 30
#define IMX477_DEFAULT_FPS 15

#define IMX477_XCLR_MIN_DELAY_US	6200
#define IMX477_XCLR_DELAY_RANGE_US	1000



#define IMX477_XCLK_MIN 6000000
#define IMX477_XCLK_MAX 24000000

#define IMX477_IMAGE_PAD_SOURCE		0
//#define IMX477_METADATA_PAD_SOURCE	1
#define IMX477_SENS_PADS_NUM		1

//#define IMX477_RESERVE_ID 0X2770
#define DCG_CONVERSION_GAIN 11


// gain reg 
#define ANA_GAIN_GLOBAL_A		0x157
#define DIG_GAIN_GLOBAL_A_UP   0x0158 
#define DIG_GAIN_GLOBAL_A_LOW  0x0159
// exp reg
#define COARSE_INTEGRATION_TIME_A_UP  0x15A
#define COARSE_INTEGRATION_TIME_A_LOW  0x15B

static int dpc_enable = 1;
module_param(dpc_enable, int, 0644);
MODULE_PARM_DESC(dpc_enable, "Enable on-sensor DPC");


/* regulator supplies */
static const char * const imx477_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX477_NUM_SUPPLIES ARRAY_SIZE(imx477_supply_name)


#define client_to_imx477(client)\
	container_of(i2c_get_clientdata(client), struct imx477, subdev)

struct imx477_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct imx477_reg {
	u16 address;
	u8 val;
};

struct imx477_reg_list {
	unsigned int num_of_regs;
	const struct imx477_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx477_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct imx477_reg_list reg_list;
};

struct imx477 {
	struct i2c_client *i2c_client;
	struct regulator_bulk_data supplies[IMX477_NUM_SUPPLIES];
	unsigned int pwn_gpio;
	unsigned int rst_gpio;
	unsigned int mclk;
	unsigned int mclk_source;
	struct clk *xclk;
	u32 xclk_freq;
	
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	*/
	struct mutex mutex;
	unsigned int csi_id;
	struct imx477_capture_properties ocp;

	struct v4l2_subdev subdev;
	struct media_pad pads[IMX477_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	u32 stream_status;
	u32 resume_status;
	bool common_regs_written;
	unsigned int long_exp_shift;

	/* V4L2 Controls */
	struct v4l2_ctrl_handler ctrl_handler;

	const struct imx477_mode *pmode;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;

	struct dentry *debugfs_dir;
};


static const struct imx477_reg mode_common_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0138, 0x01},
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02},
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0112, 0x0c},
	{0x0113, 0x0c},
	{0x0114, 0x01},
	{0x0350, 0x00},
	{0xbcf1, 0x02},
	{0x3ff9, 0x01},
};
/* 12-bit mode index */
#define MODE_3840x2160	0
#define MODE_4048x3040	1 
#define MODE_2028x1080	2
#define MODE_DEFAULT	MODE_3840x2160

/* 12 mpix 10fps */
static const struct imx477_reg mode_4048x3040_regs[] = {
	{0x0342, 0x5d},
	{0x0343, 0xc0},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x0f},
	{0x034d, 0xd8},
	{0x034e, 0x0b},
	{0x034f, 0xe0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x02},
	{0x3f57, 0xae},
};

/* 12 mpix 10fps */
static const struct imx477_reg mode_3840x2160_regs[] = {
	{0x0342, 0x58},
	{0x0343, 0xc8},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0e},
	{0x0349, 0xff},
	{0x034a, 0x08},
	{0x034b, 0x6f},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0x00},
	{0x040e, 0x08},
	{0x040f, 0x70},
	{0x034c, 0x0f},
	{0x034d, 0x00},
	{0x034e, 0x08},
	{0x034f, 0x70},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x02},
	{0x3f57, 0xae},
};

/* 1080p cropped mode */
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 4x4 binned. 120fps */
static const struct imx477_reg mode_1332x990_regs[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x01},
	{0x0342, 0x1a},
	{0x0343, 0x08},
	{0x0340, 0x04},
	{0x0341, 0x1a},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09},
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x3f0d, 0x00},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05},
	{0x040d, 0x34},
	{0x040e, 0x03},
	{0x040f, 0xde},
	{0x034c, 0x05},
	{0x034d, 0x34},
	{0x034e, 0x03},
	{0x034f, 0xde},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0x0309, 0x0a},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
};


/* Mode configs */
static const struct imx477_mode supported_modes_12bit[] = {
	{
		/* 12MPix 15 fps mode */
		.width = 3840,
		.height = 2160,
		.line_length_pix = 0x5dc0,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 3840,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 3000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1500
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3840x2160_regs),
			.regs = mode_3840x2160_regs,
		},
	},
	{
		/* 12MPix 10fps mode */
		.width = 4048, //4056,
		.height = 3040,
		.line_length_pix = 0x5dc0,
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP,
			.width = 4048, //4056,
			.height = 3040,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4048x3040_regs),
			.regs = mode_4048x3040_regs,
		},
	},
	{
		/* 1080p 50fps cropped mode */
		.width = 2028,
		.height = 1080,
		.line_length_pix = 0x31c4,
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP + 440,
			.width = 2028,
			.height = 1080,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 5000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1080_regs),
			.regs = mode_2028x1080_regs,
		},
	}
};

static const struct imx477_mode supported_modes_10bit[] = {
	{
		/* 120fps. 2x2 binned and cropped */
		.width = 1332,
		.height = 990,
		.line_length_pix = 6664,
		.crop = {
			/*
			 * FIXME: the analog crop rectangle is actually
			 * programmed with a horizontal displacement of 0
			 * pixels, not 4. It gets shrunk after going through
			 * the scaler. Move this information to the compose
			 * rectangle once the driver is expanded to represent
			 * its processing blocks with multiple subdevs.
			 */
			.left = IMX477_PIXEL_ARRAY_LEFT + 696,
			.top = IMX477_PIXEL_ARRAY_TOP + 528,
			.width = 1332,
			.height = 990,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 12000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 12000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1332x990_regs),
			.regs = mode_1332x990_regs,
		}
	}
};


/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 support_codes[] = {
	/*
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	*/
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};
/* Default link freq */
static const s64 link_freqs[] = {IMX477_DEFAULT_LINK_FREQ};

static const char * const imx477_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx477_test_pattern_val[] = {
	IMX477_TEST_PATTERN_DISABLE,
	IMX477_TEST_PATTERN_COLOR_BARS,
	IMX477_TEST_PATTERN_SOLID_COLOR,
	IMX477_TEST_PATTERN_GREY_COLOR,
	IMX477_TEST_PATTERN_PN9,
};



static inline void get_mode_table(unsigned int code,
				  const struct imx477_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		/* Appears like libcamera always select the hight resolution
			So, linit the array size to choose specific resolution */
		*num_modes = MODE_DEFAULT + 1 ; //ARRAY_SIZE(supported_modes_12bit);
		break;
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;
		*num_modes = ARRAY_SIZE(supported_modes_10bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

static int imx477_power_on(struct imx477 *sensor)
{
	int ret = 0;

	ret = regulator_bulk_enable(IMX477_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		return ret;
	}
	
	/* get out of powerdown and reset */
	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	clk_prepare_enable(sensor->xclk);
	fsleep(6000);

	return ret;
}

static int imx477_power_off(struct imx477 *sensor)
{
	clk_disable_unprepare(sensor->xclk);
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, sensor->supplies);
	return 0;
}

static int imx477_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);

	if (on)
		imx477_power_on(sensor);
	else
		imx477_power_off(sensor);

	return 0;
}


static int imx477_read_reg(struct imx477 *sensor, u16 reg, u32 len, u32 *val)
{
	
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;
	
	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];
	
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	
	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int imx477_write_reg(struct imx477 *sensor, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = sensor->i2c_client;
	u8 buf[6];

	if (len > 4) {
		return -EINVAL;
	}

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}


/* Write a list of registers */
static int imx477_write_regs(struct imx477 *sensor,
				const struct imx477_reg *regs, u32 len)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx477_write_reg(sensor, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static const struct imx477_reg link_456Mhz_regs[] = {
	{0x030E, 0x00},
	{0x030F, 0x98},
};


static int imx477_start_streaming(struct imx477 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	const struct imx477_reg_list *reg_list;
	int ret;
	
	if (!sensor->common_regs_written) {

		ret = imx477_write_regs(sensor, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));

		/* Must Update Link Freq */
		if (!ret)
			ret = imx477_write_regs(sensor, link_456Mhz_regs,
								ARRAY_SIZE(link_456Mhz_regs));
		
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}

		sensor->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &sensor->pmode->reg_list;
	ret = imx477_write_regs(sensor, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Set on-sensor DPC. */
	imx477_write_reg(sensor, 0x0b05, IMX477_REG_VALUE_08BIT, !!dpc_enable);
	imx477_write_reg(sensor, 0x0b06, IMX477_REG_VALUE_08BIT, !!dpc_enable);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(sensor->subdev.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return imx477_write_reg(sensor, IMX477_REG_MODE_SELECT,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

static void imx477_stop_streaming(struct imx477 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	/* set stream off register */
	ret = imx477_write_reg(sensor, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	/* Stop driving XVS out (there is still a weak pull-up) */
	//imx477_write_reg(sensor, IMX477_REG_EXTOUT_EN, IMX477_REG_VALUE_08BIT, 0);
}

static int imx477_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);
	int ret = 0;

	mutex_lock(&sensor->mutex);

	if (sensor->stream_status == enable) {
		mutex_unlock(&sensor->mutex);
		return 0;
	}

	if (enable) {

		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx477_start_streaming(sensor);
		if (ret)
			goto err_rpm_put;
		sensor->stream_status = enable;
	} else {
		imx477_stop_streaming(sensor);
		//pm_runtime_put(&client->dev);
		sensor->stream_status = 0;
		pm_runtime_mark_last_busy(&sensor->i2c_client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	mutex_unlock(&sensor->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&sensor->mutex);

	return ret;
}

/* Get bayer order based on flip setting. */
static u32 imx477_get_format_code(struct imx477 *sensor, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&sensor->mutex);

	for (i = 0; i < ARRAY_SIZE(support_codes); i++)
		if (support_codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(support_codes))
		i = 0;

	return support_codes[i];
}



#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
			         struct v4l2_subdev_pad_config *cfg,
			         struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);

	if (code->index >= (ARRAY_SIZE(support_codes) / 4))
		return -EINVAL;

	code->code = imx477_get_format_code(sensor, support_codes[code->index * 4]);

	return 0;
}

static int imx477_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);
	const struct imx477_mode *mode_list;
	unsigned int num_modes;

	if (fse->pad != IMX477_IMAGE_PAD_SOURCE) {
		return -EINVAL;
	} else {

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx477_get_format_code(sensor, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	}
	return 0;
}

static
unsigned int imx477_get_frame_length(const struct imx477_mode *mode,
				     const struct v4l2_fract *timeperframe)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * IMX477_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > IMX477_FRAME_LENGTH_MAX))
		frame_length = IMX477_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}


static void imx477_set_framing_limits(struct imx477 *sensor)
{
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct imx477_mode *pmode = sensor->pmode;

	frm_length_min = imx477_get_frame_length(pmode, &pmode->timeperframe_min);
	frm_length_default =
		     imx477_get_frame_length(pmode, &pmode->timeperframe_default);

	/* Default to no long exposure multiplier. */
	sensor->long_exp_shift = 0;

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(sensor->vblank, frm_length_min - pmode->height,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - pmode->height,
				 1, frm_length_default - pmode->height);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(sensor->vblank, frm_length_default - pmode->height);

	hblank_min = pmode->line_length_pix - pmode->width;
	__v4l2_ctrl_modify_range(sensor->hblank, hblank_min,
				 IMX477_LINE_LENGTH_MAX, 1, hblank_min);
	__v4l2_ctrl_s_ctrl(sensor->hblank, hblank_min);
}

static void imx477_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx477_update_pad_format(struct imx477 *imx477,
				     const struct imx477_mode *pmode,
				     struct v4l2_subdev_format *fmt)
{

	fmt->format.width = pmode->width;
	fmt->format.height = pmode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx477_reset_colorspace(&fmt->format);
}

static int imx477_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx477_mode *mode_list, *pmode;
	unsigned int num_modes;

	if (fmt->pad != IMX477_IMAGE_PAD_SOURCE) {
		return -EINVAL;
	}
	mutex_lock(&sensor->mutex);

	/* Bayer order varies with flips */
	fmt->format.code = imx477_get_format_code(sensor, fmt->format.code);
	get_mode_table(fmt->format.code, &mode_list, &num_modes);

	pmode = v4l2_find_nearest_size(mode_list,
				      num_modes,
				      width, height,
				      fmt->format.width,
					  fmt->format.height);
	
	if (!pmode) {
		mutex_unlock(&sensor->mutex);
		return -EINVAL;
	}
	imx477_update_pad_format(sensor, pmode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			v4l2_subdev_get_fmt(sd, state, fmt);
			*framefmt = fmt->format;

	} else if (sensor->pmode != pmode ||
				sensor->format.code != fmt->format.code) {
		sensor->format = fmt->format;
		sensor->pmode = pmode;
		imx477_set_framing_limits(sensor);
	}

	mutex_unlock(&sensor->mutex);
	return 0;
}


static int imx477_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);
	
	if (fmt->pad != IMX477_IMAGE_PAD_SOURCE)
		return -EINVAL;

	mutex_lock(&sensor->mutex);
	
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {

		v4l2_subdev_get_fmt(sd, state, fmt);
		/* update the code which could change due to vflip or hflip: */
		fmt->format.code = imx477_get_format_code(sensor, fmt->format.code);

	} else {

		fmt->format = sensor->format;
	}

	mutex_unlock(&sensor->mutex);

	return 0;
}


static u8 imx477_code2dt(const u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_DT_RAW10;

	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return MIPI_CSI2_DT_RAW12;

	default:
		return MIPI_CSI2_DT_RAW10;
	}
}

static int imx477_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	/* get sensor current code*/
	mutex_lock(&sensor->mutex);
	fd->entry[0].pixelcode = sensor->format.code;
	mutex_unlock(&sensor->mutex);

	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = imx477_code2dt(fd->entry[0].pixelcode);

	return 0;
}

static int imx477_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *sensor = client_to_imx477(client);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		mutex_lock(&sensor->mutex);
		sel->r = sensor->pmode->crop;
		mutex_unlock(&sensor->mutex);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX477_NATIVE_WIDTH;
		sel->r.height = IMX477_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left =	IMX477_PIXEL_ARRAY_LEFT;
		sel->r.top =	IMX477_PIXEL_ARRAY_TOP;
		sel->r.width =	IMX477_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX477_PIXEL_ARRAY_HEIGHT;
		return 0;
	
	}
	return -EINVAL;
}


static struct v4l2_subdev_video_ops imx477_subdev_video_ops = {
	.s_stream = imx477_s_stream,
};

static const struct v4l2_subdev_pad_ops imx477_subdev_pad_ops = {
	.set_fmt = imx477_set_fmt,
	.get_fmt = imx477_get_fmt,
	.enum_mbus_code		= imx477_enum_mbus_code,
	.get_selection		= imx477_get_selection,
	.enum_frame_size	= imx477_enum_frame_size,
	.get_frame_desc		= imx477_get_frame_desc,
};

static struct v4l2_subdev_core_ops imx477_subdev_core_ops = {
	.s_power = imx477_s_power,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};
static struct v4l2_subdev_ops imx477_subdev_ops = {
	.core  = &imx477_subdev_core_ops,
	.video = &imx477_subdev_video_ops,
	.pad   = &imx477_subdev_pad_ops,
};

static int imx477_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx477_sd_media_ops = {
	.link_setup = imx477_link_setup,
};

static void imx477_reset(struct imx477 *sensor)
{
	if (!gpio_is_valid(sensor->rst_gpio))
		return;

	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	msleep(20);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(20);

	return;
}

static int imx477_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

#if 0
	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}
#endif
	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != link_freqs[0]) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;
error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}


/* Verify chip ID */
static int imx477_identify_module(struct imx477 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	u32 val;

	ret = imx477_read_reg(sensor, IMX477_REG_CHIP_ID,
			      IMX477_REG_VALUE_16BIT, &val);

	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			IMX477_CHIP_ID);
		return ret;
	}

	if (val != IMX477_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX477_CHIP_ID, val);
		/*return -EIO;*/
	}

	return 0;
}

static void imx477_set_default_format(struct imx477 *sensor)
{
	struct v4l2_mbus_framefmt *fmt;

	sensor->pmode = &supported_modes_12bit[MODE_DEFAULT];
	fmt = &sensor->format;
	fmt->code = imx477_get_format_code(sensor,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	fmt->width =  supported_modes_12bit[MODE_DEFAULT].width;
	fmt->height = supported_modes_12bit[MODE_DEFAULT].height;
	fmt->field = V4L2_FIELD_NONE;
	
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

/* Initialize control handlers */
static void imx477_adjust_exposure_range(struct imx477 *sensor)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = sensor->format.height + sensor->vblank->val -
		       IMX477_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, sensor->exposure->val);
	__v4l2_ctrl_modify_range(sensor->exposure, sensor->exposure->minimum,
				 exposure_max, sensor->exposure->step,
				 exposure_def);
}

static int imx477_set_frame_length(struct imx477 *sensor, unsigned int val)
{
	int ret = 0;

	sensor->long_exp_shift = 0;

	while (val > IMX477_FRAME_LENGTH_MAX) {
		sensor->long_exp_shift++;
		val >>= 1;
	}

	ret = imx477_write_reg(sensor, IMX477_REG_FRAME_LENGTH,
			       IMX477_REG_VALUE_16BIT, val);
	if (ret)
		return ret;

	return imx477_write_reg(sensor, IMX477_LONG_EXP_SHIFT_REG,
				IMX477_REG_VALUE_08BIT, sensor->long_exp_shift);
}

static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *sensor =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->subdev);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		imx477_adjust_exposure_range(sensor);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx477_write_reg(sensor, IMX477_REG_ANALOG_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx477_write_reg(sensor, IMX477_REG_EXPOSURE,
				       IMX477_REG_VALUE_16BIT, ctrl->val >>
							sensor->long_exp_shift);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx477_write_reg(sensor, IMX477_REG_DIGITAL_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_write_reg(sensor, IMX477_REG_TEST_PATTERN,
				       IMX477_REG_VALUE_16BIT,
				       imx477_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx477_write_reg(sensor, IMX477_REG_TESTP_RED,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx477_write_reg(sensor, IMX477_REG_TESTP_GREENR,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx477_write_reg(sensor, IMX477_REG_TESTP_BLUE,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx477_write_reg(sensor, IMX477_REG_TESTP_GREENB,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx477_write_reg(sensor, IMX477_REG_ORIENTATION, 1,
				       sensor->hflip->val |
				       sensor->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx477_set_frame_length(sensor,
					      sensor->format.height + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx477_write_reg(sensor, IMX477_REG_LINE_LENGTH, 2,
				       sensor->format.width + ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.open = imx477_open,
};

static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
};

static int imx477_init_controls(struct imx477 *sensor)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &sensor->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &sensor->mutex;

	/* By default, PIXEL_RATE is read only */
	sensor->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX477_PIXEL_RATE,
					       IMX477_PIXEL_RATE, 1,
					       IMX477_PIXEL_RATE);
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	sensor->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx477_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       link_freqs);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx477_set_framing_limits() call below.
	 */
	sensor->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	sensor->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	sensor->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     IMX477_EXPOSURE_MAX,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANA_GAIN_MIN, IMX477_ANA_GAIN_MAX,
			  IMX477_ANA_GAIN_STEP, IMX477_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX477_DGTL_GAIN_MIN, IMX477_DGTL_GAIN_MAX,
			  IMX477_DGTL_GAIN_STEP, IMX477_DGTL_GAIN_DEFAULT);

	sensor->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (sensor->hflip)
		sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	sensor->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	else
		pr_info ("%s:  fail to get sensor->vflip\n", __func__);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				     0, 0, imx477_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX477_TESTP_COLOUR_MIN,
				  IMX477_TESTP_COLOUR_MAX,
				  IMX477_TESTP_COLOUR_STEP,
				  IMX477_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx477_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	sensor->subdev.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	mutex_lock(&sensor->mutex);
	imx477_set_framing_limits(sensor);
	mutex_unlock(&sensor->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&sensor->mutex);

	return ret;
}

static void imx477_free_controls(struct imx477 *sensor)
{
	v4l2_ctrl_handler_free(sensor->subdev.ctrl_handler);
	mutex_destroy(&sensor->mutex);
}

static int imx477_get_regulators(struct imx477 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned int i;

	for (i = 0; i < IMX477_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = imx477_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX477_NUM_SUPPLIES,
				       sensor->supplies);
}

static const struct of_device_id imx477_of_match[] = {
	{ .compatible = "sony,imx477" },
	{ /* sentinel */ }
};

static ssize_t imx477_debugfs_read_gain(struct file *file, char __user *user_buf,
                                        size_t count, loff_t *ppos)
{
    struct imx477 *imx477 = file->private_data;
    char buf[32];
    int ret;
    u32 gain;

	ret = imx477_read_reg(imx477, IMX477_REG_ANALOG_GAIN, IMX477_REG_VALUE_16BIT, &gain);

    if (ret)
        return -EIO;

    ret = snprintf(buf, sizeof(buf), "%u\n", gain);
    return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t imx477_debugfs_read_exposure(struct file *file, char __user *user_buf,
                                        size_t count, loff_t *ppos)
{
    struct imx477 *imx477 = file->private_data;
    char buf[32];
    int ret;
    u32 exposure;

	ret = imx477_read_reg(imx477, IMX477_REG_EXPOSURE, IMX477_REG_VALUE_16BIT, &exposure);
    if (ret)
        return -EIO;

    ret = snprintf(buf, sizeof(buf), "%u\n", exposure);
    return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static const struct file_operations imx477_readgain_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = imx477_debugfs_read_gain,
    .llseek = default_llseek,
};

static const struct file_operations imx477_readexposure_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = imx477_debugfs_read_exposure,
    .llseek = default_llseek,
};

MODULE_DEVICE_TABLE(of, imx477_of_match);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int imx477_probe(struct i2c_client *client)
#else
static int imx477_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
#endif
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct imx477 *sensor;
	const struct of_device_id *match;

	match = of_match_device(imx477_of_match, dev);
	if (!match) {
		return -ENODEV;
	}

	/* Check the hardware configuration in device tree */
	if (imx477_check_hwcfg(dev)) {
		return -EINVAL;
	}

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(*sensor));

	sensor->i2c_client = client;

	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
						GPIOF_OUT_INIT_LOW,
						"imx477_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->rst_gpio,
						GPIOF_OUT_INIT_HIGH,
						"imx477_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi_id));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}
	
	/* Get system clock (xclk) */
	sensor->xclk =  devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}
	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	if (sensor->xclk_freq != IMX477_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			sensor->xclk_freq);
		return -EINVAL;
	}
	
	mutex_init(&sensor->mutex);

	v4l2_i2c_subdev_init(&sensor->subdev, client, &imx477_subdev_ops);

	retval = imx477_get_regulators(sensor);
	if (retval) {
		dev_err(dev, "failed to get regulators\n");
		return retval;
	}
	
	retval = imx477_power_on(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: sensor power on fail\n", __func__);
		return retval;
	}

	imx477_reset(sensor);

	retval = imx477_identify_module(sensor);
	if (retval)
		goto probe_err_power_off;
	/* sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	retval = imx477_write_reg(sensor, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
	if (retval < 0)
		goto probe_err_power_off;
	usleep_range(100, 110);

	/* put sensor back to standby mode */
	retval= imx477_write_reg(sensor, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (retval < 0)
		goto probe_err_power_off;
	usleep_range(100, 110);

	imx477_set_default_format(sensor);

	retval = imx477_init_controls(sensor);
	if (retval) {
		dev_err(dev, "imx477_init_controls failed\n");
		return retval;
	}
	
	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	sd = &sensor->subdev;
	sd->internal_ops = &imx477_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[IMX477_IMAGE_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				IMX477_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;

	
	retval = v4l2_async_register_subdev_sensor(sd);
	if (retval < 0) {
		dev_err(&client->dev,"%s--Async register failed, ret=%d\n",
			__func__,retval);
		goto probe_err_free_entity;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	sensor->debugfs_dir = debugfs_create_dir("imx477", NULL);
	if (!sensor->debugfs_dir) {
		dev_err(&client->dev, "Failed to create debugfs directory\n");
	};
	debugfs_create_file("analog_gain", 0444, sensor->debugfs_dir, sensor, &imx477_readgain_fops);
	debugfs_create_file("expsoure", 0444, sensor->debugfs_dir, sensor, &imx477_readexposure_fops);

	return 0;

probe_err_free_entity:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	media_entity_cleanup(&sd->entity);
	imx477_free_controls(sensor);

probe_err_power_off:
	imx477_power_off(sensor);

	return retval;
}

static void imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *sensor = client_to_imx477(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx477_free_controls(sensor);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx477_power_off(sensor);
	pm_runtime_set_suspended(&client->dev);
	
	mutex_destroy(&sensor->mutex);

}

static int __maybe_unused imx477_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx477 *sensor = client_to_imx477(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status) {
		imx477_s_stream(&sensor->subdev,0);
	}

	return 0;
}

static int __maybe_unused imx477_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx477 *sensor = client_to_imx477(client);

	if (sensor->resume_status) {
		imx477_s_stream(&sensor->subdev,1);
	}

	return 0;
}

static const struct dev_pm_ops imx477_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx477_suspend, imx477_resume)
};

static const struct i2c_device_id imx477_id[] = {
	{"imx477", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx477_id);

static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imx477",
		.pm = &imx477_pm_ops,
		.of_match_table	= imx477_of_match,
	},
	.probe  = imx477_probe,
	.remove = imx477_remove,
	.id_table = imx477_id,
};


module_i2c_driver(imx477_i2c_driver);

MODULE_DESCRIPTION("Sony IMX477 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL v2");
