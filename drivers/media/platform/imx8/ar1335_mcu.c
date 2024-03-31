/*
 * A V4L2 driver for AR1335 cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>


#define SENSOR_NAME "ar1335_mcu"
#include "ar1335_mcu.h"
#include "cam_firmware.h"

#define debug_printk printk

#define FREE_SAFE(dev, ptr) \
	if(ptr) { \
		devm_kfree(dev, ptr); \
	}

#define to_ar1335(sd) container_of(sd, struct ar1335, sd)
#define AR1335_MCU_CLK	24000000

struct i2c_client *io_expander;

struct regval_list {
	u16 addr;
	u8 data;
};

static const s64 link_freq_menu_items[] = {
        500000000,
	600000000,
	800000000,
};
static const struct ar1335_mode supported_modes[] = {
	{
		.width = 640,
		.height = 480,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},  
	{
		.width = 1280,
		.height = 720,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},
	{
		.width = 1920,
		.height = 1080,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},
	{
		.width = 3840,
		.height = 2160,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},
	{
		.width = 4096,
		.height = 2160,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},
	{
		.width = 4192,
		.height = 3120,
		.pixel_clock = 500000000,
		.link_freq = 0 /* an index in link_freq[] */
	},
};



static inline struct ar1335 *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar1335, sd);
}


/*------------------I2C error checking during read and write-----*/

unsigned char errorcheck(char *data, unsigned int len) {
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
	}

	return crc;
}


/*------------------Wrapper function---------------*/

static struct ar1335 *to_ar1335_data(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ar1335, sd);
}

static int io_exp_toggle_pwdn(u8 val)
{
	int ret;
	unsigned char data[2] = { IO_EXP_OUTPUT_PORT0 , val};
	unsigned char recv_data[1] = { IO_EXP_OUTPUT_PORT0 };

	io_expander->addr = IO_EXPANDER_SLAVE_ADDRESS;
       
        ret = i2c_master_recv(io_expander, recv_data , 1);

	if(val)	
        data[1]= (recv_data[0]) | 1<<6;
	else
        data[1]= (recv_data[0]) & (~(1<<6));

	ret = i2c_master_send(io_expander, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2) {
		ret = 0;
	} else {
		debug_printk("%s: IO Expander i2c write error \n",
				__func__);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int io_exp_toggle_reset(u8 val)
{
	int ret;
	unsigned char data[2] = { IO_EXP_OUTPUT_PORT0 , val};
	unsigned char recv_data[1] = { IO_EXP_OUTPUT_PORT0 };

        io_expander->addr = IO_EXPANDER_SLAVE_ADDRESS;
     
     	ret = i2c_master_recv(io_expander, recv_data , 1);
         
	if(val)	
        data[1]= (recv_data[0]) | 1<<0;
	else
        data[1]= (recv_data[0]) & (~(1<<0));
          
	ret = i2c_master_send(io_expander, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2) {
		ret = 0;
	} else {
		debug_printk("%s: IO Expander i2c write error\n",
				__func__);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}


static int io_expander_config(u16 reg, u8 val)
{
	 int ret; 
	 unsigned char data[2] = { reg , val};

        io_expander->addr = IO_EXPANDER_SLAVE_ADDRESS;
          
	ret = i2c_master_send(io_expander, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2) {
		ret = 0;
	} else {
		debug_printk("%s: IO Expander i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}


/*----------------Wrapper Funtion for Setting Formats--------------*/

#if 0
static struct v4l2_mbus_framefmt *
__ar1335_get_pad_format(struct ar1335 *ar1335,
             struct v4l2_subdev_pad_config *cfg,
                        unsigned int pad,
                        enum v4l2_subdev_format_whence which)
{
        switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
            return v4l2_subdev_get_try_format(&ar1335->subdev, cfg, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
                return &ar1335->fmt;
        default:
                return NULL;
        }
}

static const struct v4l2_rect *
__ar1335_get_pad_crop(struct ar1335 *ar1335, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ar1335->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar1335->mode->crop;
	}
       
	return NULL;
}
#endif


static int ar1335_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ar1335 *ar1335 = to_state(sd);
	int mode = ar1335->mode_index;

	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.timeperframe.numerator = 1;
	param->parm.capture.timeperframe.denominator = 
		ar1335->mcu_cam_frmfmt[mode].framerates[ar1335->mode_index];
	debug_printk(" %s %d\n",__func__,__LINE__);
	return 0;
}

static int ar1335_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	debug_printk(" %s %d\n",__func__,__LINE__);
	return 0;
	
}
static int ar1335_s_stream(struct v4l2_subdev *sd, int on)
{

	struct ar1335 *state = to_state(sd);
        struct i2c_client *client = state->i2c_client;

	int ret = 0;

	mutex_lock(&state->lock);
	debug_printk(" %s\n",__func__);


       ret = mcu_cam_stream_on(client);

	mutex_unlock(&state->lock);

	return ret;

}

static const struct v4l2_subdev_video_ops ar1335_subdev_video_ops = {
	.g_parm = ar1335_g_parm,
	.s_parm = ar1335_s_parm,
	.s_stream = ar1335_s_stream,
};

static int ar1335_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{


        if (code->index > 0)
                return -EINVAL;

        code->code = MEDIA_BUS_FMT_UYVY8_2X8;

        return 0;


}

static int ar1335_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	
	if (fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;


	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].height;
	return 0;


}

static int ar1335_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
        struct ar1335 *ar1335 = to_ar1335(sd);
        struct i2c_client *client = ar1335->i2c_client;
        const struct ar1335_mode *new_mode;
        int ret = 0,mode,flag=0;

	debug_printk(" %s\n",__func__);

        for(mode=0;mode<ar1335->numfmts;mode++) {
           if((fmt->format.width==supported_modes[mode].width)&&
                   (fmt->format.height==supported_modes[mode].height)){
                     
                     new_mode = &supported_modes[mode];
                     flag=1;
                     break;
            }
         }

         if(flag==0){
                       return -EINVAL;
         }

	fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->format.width = new_mode->width;
	fmt->format.height = new_mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
        ar1335->cur_mode = new_mode;

	/***********************
		MCU
	*************************/

	switch (fmt->format.code) {


	case MEDIA_BUS_FMT_UYVY8_2X8:
		ar1335->format_fourcc = V4L2_PIX_FMT_UYVY;
		break;


	case MEDIA_BUS_FMT_Y12_1X12:
		ar1335->format_fourcc = V4L2_PIX_FMT_Y12;
		break;

        case MEDIA_BUS_FMT_Y8_1X8:
		ar1335->format_fourcc = V4L2_PIX_FMT_GREY;
		break;

	default:
		/* Not Implemented */
		if (fmt->which != V4L2_SUBDEV_FORMAT_TRY) {
		return -EINVAL;
		}
	}

	/* call stream config with width, height, frame rate */ 
	ar1335->frate_index = 0;
	ar1335->mode_index = mode;
	ret = mcu_stream_config(client, ar1335->format_fourcc, mode,0);
	if (ret < 0) {
		return ret;
	}
	else
	{
		debug_printk(" stream config success\n");
	}
	mdelay(10);

	return 0;
	
}

static int ar1335_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{

        struct ar1335 *ar1335 = to_ar1335(sd);
        const struct ar1335_mode *mode = ar1335->cur_mode;

        mutex_lock(&ar1335->mutex);
        if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(sd, state, fmt->pad);
#else
                mutex_unlock(&ar1335->mutex);
                return -ENOTTY;
#endif
        } else {
                fmt->format.width = mode->width;
                fmt->format.height = mode->height;
                fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		fmt->format.field = V4L2_FIELD_NONE;
        }
        mutex_unlock(&ar1335->mutex);

        return 0;


}

static const struct v4l2_subdev_pad_ops ar1335_subdev_pad_ops = {
	.enum_mbus_code = ar1335_enum_mbus_code,
	.enum_frame_size = ar1335_enum_framesizes,
	.set_fmt = ar1335_set_fmt,
	.get_fmt = ar1335_get_fmt,
};

static int ar1335_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335 *ar1335 = to_ar1335_data(client);

	if (on) {
		if (ar1335->power_on == 0) {
			io_exp_toggle_pwdn(1);
			pr_info("Sensor Hardware Power Up Sequence\n");
		}
		ar1335->power_on = 1;
	}
	else {
		if (ar1335->power_on == 1) {
			io_exp_toggle_pwdn(0);
			pr_info("Sensor Hardware Power Down Sequence\n");
		}
		ar1335->power_on = 0;
	}
	return 0;
}

static struct v4l2_subdev_core_ops ar1335_subdev_core_ops = {
	.s_power	= ar1335_s_power,
#if 0
	.queryctrl = ar1335_queryctrl,
	.g_ctrl = ar1335_g_ctrl,
	.s_ctrl = ar1335_s_ctrl,
	.g_ext_ctrls = ar1335_g_ext_ctrls,
	.s_ext_ctrls = ar1335_s_ext_ctrls,
	.try_ext_ctrls = ar1335_try_ext_ctrls,
	.querymenu = ar1335_querymenu,
	.ioctl = ar1335_priv_ioctl,
#endif
};

static const struct v4l2_subdev_ops ar1335_subdev_ops = {
	.core		= &ar1335_subdev_core_ops,
	.video		= &ar1335_subdev_video_ops,
	.pad		= &ar1335_subdev_pad_ops,
};


/*-------------------I2C write in camera mcu-------------------*/

static int ar1335_write(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = MCU_SLAVE_ADDRESS,
		.flags = 0,
		.len = count,
		.buf = val,
	};
	client->addr= MCU_SLAVE_ADDRESS;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		debug_printk("Failed writing register ret = %d!\n",
			ret);
		return ret;
	}
        return 0;
}

/*-------------------I2C read from camera mcu-------------------*/ 

static int ar1335_read(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = MCU_SLAVE_ADDRESS,
		.flags = 0,
		.buf = val,
	};
        client->addr= MCU_SLAVE_ADDRESS;

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
	{
		debug_printk("Failed reading register ret = %d!\n", ret);
		goto err;
	}

	return 0;

 err:
	return ret;
}



static int ar1335_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{

	return 0;

}

static const struct v4l2_subdev_internal_ops ar1335_subdev_internal_ops = {
	.open = ar1335_open,
};

static int ar1335_parse_dt(struct device_node *np, struct ar1335 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = { .bus_type = 0 };
	struct device_node *ep;

	int ret;

	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);

	if (!ret)
		sensor->flags = bus_cfg.bus.mipi_csi2.flags;

	of_node_put(ep);
	return ret;
}

/*---------Setting controls at Camera MCU--------*/

static int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
			uint8_t ctrl_type, int32_t curr_val)
{
	struct ar1335 *sensor = to_ar1335_data(client);

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0;
	uint32_t ctrl_id = 0;

	
	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	ctrl_id = arg_ctrl_id;

	/* call ISP Ctrl config command */

	for (loop = 0; loop < sensor->num_ctrls; loop++) {
		if (sensor->ctrldb[loop] == ctrl_id) {
			index = loop;
			break;
		}
	}


	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	payload_len = 11;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ar1335_write(client, mc_data, TX_LEN_PKT);

	/* Second Txn */
	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Control ID */
	mc_data[4] = ctrl_id >> 24;
	mc_data[5] = ctrl_id >> 16;
	mc_data[6] = ctrl_id >> 8;
	mc_data[7] = ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data[8] = ctrl_type;

	/* Ctrl Value */
	mc_data[9] = curr_val >> 24;
	mc_data[10] = curr_val >> 16;
	mc_data[11] = curr_val >> 8;
	mc_data[12] = curr_val & 0xFF;

	/* CRC */
	mc_data[13] = errorcheck(&mc_data[2], 11);

	err = ar1335_write(client, mc_data, 14);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (1) {
		cmd_id = CMD_ID_SET_CTRL;
		if (mcu_get_cmd_status
		    (client, &cmd_id, &cmd_status, &retcode) < 0) {
			debug_printk(" %s(%d) Error \n",
			       __func__, __LINE__);
			ret = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			debug_printk("(%s) %d ISP Error STATUS = 0x%04x RET = 0x%02x\n",
			     __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}
	}

 exit:

	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;
}
static int ar1335_s_ctrl(struct v4l2_ctrl *ctrl)
{
        struct ar1335 *ar1335 = container_of(ctrl->handler,
                                             struct ar1335, ctrls);

	struct i2c_client *client = ar1335->i2c_client;
        int ret = 0;
	debug_printk(" %s\n",__func__);

	/*if (pm_runtime_get(&client->dev) <= 0)
	{
		
		debug_printk(" returning without proper set ctrl  %d\n",__LINE__);
                return 0;
	} */

	if ((ctrl->id == V4L2_CID_PIXEL_RATE) || (ctrl->id == V4L2_CID_LINK_FREQ))
		ret = 0;
	else if ((mcu_set_ctrl(client, ctrl->id, CTRL_STANDARD, ctrl->val)) < 0) {
		debug_printk(" 0x%x (%d )\n", ctrl->id,ctrl->val);
		ret = 0;
	}

	//      pm_runtime_put(&client->dev);

       return ret;
	
}

static const struct v4l2_ctrl_ops ar1335_ctrl_ops = {
	.s_ctrl = ar1335_s_ctrl,
};

/*--------------------converting ASCII value to hexadecimal value-------*/ 

int mcu_bload_ascii2hex(unsigned char ascii) {
	if (ascii <= '9') {
		return (ascii - '0');
	} else if ((ascii >= 'a') && (ascii <= 'f')) {
		return (0xA + (ascii - 'a'));
	} else if ((ascii >= 'A') && (ascii <= 'F')) {
		return (0xA + (ascii - 'A'));
	}
	return -1;
}

/*----------------------Getting firmware version ID from camera MCU-----------------*/
static int mcu_get_fw_version(struct i2c_client *client, unsigned char *fw_version, unsigned char *txt_fw_version) {

	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop, i=0, retry = 5;
	unsigned long txt_fw_pos = ARRAY_SIZE(g_mcu_fw_buf)-VERSION_FILE_OFFSET;


	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	debug_printk("  %s\n",__func__);

	/* Get Text Firmware version*/
	for(loop = txt_fw_pos; loop < (txt_fw_pos+64); loop=loop+2) {
		*(txt_fw_version+i) = (mcu_bload_ascii2hex(g_mcu_fw_buf[loop]) << 4 |
				mcu_bload_ascii2hex(g_mcu_fw_buf[loop+1]));
		i++;
	}

	while (retry-- > 0) {
		/* Query firmware version from MCU */
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = ar1335_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_VERSION;
		err = ar1335_write(client, mc_data, 2);
		if (err != 0) {
			debug_printk(" %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		err = ar1335_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			debug_printk(" %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}

		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}

		/* Read the actual version from MCU*/
		payload_len =
			((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
		memset(mc_ret_data, 0x00, payload_len);
		err = ar1335_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			debug_printk(" %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
					__LINE__, ret);
			ret = -EIO;
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc = errorcheck(&mc_ret_data[2], 32);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
					__func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			continue;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
					__LINE__, errcode);
			ret = -EIO;
			continue;
		}
		if(ret == ERRCODE_SUCCESS) 
			break; 
	}


	if (retry == 0 && ret != ERRCODE_SUCCESS) {
		debug_printk(" exiting at %d\n",__LINE__);
		goto exit;
	}

	for (loop = 0 ; loop < VERSION_SIZE ; loop++ )
		*(fw_version+loop) = mc_ret_data[2+loop];

	/* Check for forced/always update field in the text firmware version*/
	if(txt_fw_version[17] == '1') {
		debug_printk("Forced Update Enabled - Firmware Version - (%.32s) \n",
				fw_version);
		ret = 2;
		goto exit;
	}			

	for(i = 0; i < VERSION_SIZE; i++) {
		if(txt_fw_version[i] != fw_version[i]) {
			ret = -1;
			goto exit;
		}
	}

	ret = ERRCODE_SUCCESS;
exit:
			debug_printk("Previous Firmware Version - (%.32s)\n", fw_version);
			debug_printk("Current Firmware Version - (%.32s)\n", txt_fw_version);

	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;
}


/*-------------------Getting bootloader version from camera MCU------------*/

int mcu_bload_get_version(struct i2c_client *client) {

	int ret = 0;
	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf[0] = BL_GET_VERSION;
	g_bload_buf[1] = ~(BL_GET_VERSION);

	ret = ar1335_write(client, g_bload_buf, 2);
	if (ret < 0) {
		debug_printk("Write Failed %d \n",__LINE__);
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed %d \n",__LINE__);
		return -1;
	}

	if (g_bload_buf[0] != 'y') {
		/*   NACK Received */
		debug_printk(" NO ACK Received... exiting..%d \n",__LINE__);
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed ...%d \n",__LINE__);
		return -1;
	}

	
	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed\n");
		return -1;
	}

	/* ---------------- GET VERSION END ------------------- */

	return 0;
}



/*----------------------Erasing firmware from camera MCU----------*/

int mcu_bload_erase_flash(struct i2c_client *client) {

	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;

	/* --------------- ERASE FLASH --------------------- */

	for (i = 0; i < NUM_ERASE_CYCLES; i++) {

		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_ERASE_MEM_NS;
		g_bload_buf[1] = ~(BL_ERASE_MEM_NS);

		ret = ar1335_write(client, g_bload_buf, 2);
		if (ret < 0) {
			debug_printk("Write Failed %d\n",__LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed %d\n",__LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received... exiting.. %d\n",__LINE__);
			return -1;
		}

		g_bload_buf[0] = (MAX_PAGES - 1) >> 8;
		g_bload_buf[1] = (MAX_PAGES - 1) & 0xFF;
		g_bload_buf[2] = g_bload_buf[0] ^ g_bload_buf[1];

		ret = ar1335_write(client, g_bload_buf, 3);
		if (ret < 0) {
			debug_printk("Write Failed ...%d\n",__LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed ...%d\n",__LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received... exiting..%d \n",__LINE__);
			return -1;
		}

		for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
			g_bload_buf[(2 * pagenum)] =
			    (pagenum + (i * MAX_PAGES)) >> 8;
			g_bload_buf[(2 * pagenum) + 1] =
			    (pagenum + (i * MAX_PAGES)) & 0xFF;
			checksum =
			    checksum ^ g_bload_buf[(2 * pagenum)] ^
			    g_bload_buf[(2 * pagenum) + 1];
		}
		g_bload_buf[2 * MAX_PAGES] = checksum;

		ret = ar1335_write(client, g_bload_buf, (2 * MAX_PAGES) + 1);
		if (ret < 0) {
			debug_printk("Write Failed ...%d\n",__LINE__);
			return -1;
		}

		msleep(2000);
 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed ...%d\n",__LINE__);
			return -1;
		}

		if (g_bload_buf[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received...  %d \n",__LINE__);
			return -1;
		}

		debug_printk("ERASE Sector %d success !! \n", i + 1);
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}


/*-------------Commands checksum in firmware flash---------*/

unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len) {

	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		checksum = (checksum + buf[i]);
	}

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

/*------------Cyclic checking of flash data-----------*/

unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len) {
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		crc ^= buf[i];
	}

	return crc;
}

/*---------------Parsing and sending commands in firmware flash--------------*/

int mcu_bload_parse_send_cmd(struct i2c_client *client,
			     unsigned char *bytearray, int rec_len) {

	IHEX_RECORD *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (IHEX_RECORD *) bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = mcu_bload_inv_checksum(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		debug_printk(" Invalid Checksum 0x%02x != 0x%02x !! \n",
		       checksum, calc_checksum);
		return -1;
	}

	if ((ihex_rec->rectype == REC_TYPE_ELA)
	    && (ihex_rec->addr == 0x0000)
	    && (ihex_rec->datasize = 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr = (ihex_rec->recdata[0] <<
				     24) | (ihex_rec->recdata[1]
					    << 16);

		debug_printk("Updated Flash Addr = 0x%08x \n",
			     g_bload_flashaddr);

	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr =
		    (g_bload_flashaddr & 0xFFFF0000) | (ihex_rec->addr);
		g_bload_crc16 ^=
		    mcu_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf[0] = BL_WRITE_MEM_NS;
		g_bload_buf[1] = ~(BL_WRITE_MEM_NS);

		ret = ar1335_write(client, g_bload_buf, 2);
		if (ret < 0) {
			debug_printk("Write Failed %d\n",__LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed %d\n",__LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received... exiting.. %d\n",__LINE__);
			return -1;
		}

		g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
		g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
		g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
		g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
		g_bload_buf[4] =
		    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^
		    g_bload_buf[3];

		ret = ar1335_write(client, g_bload_buf, 5);
		if (ret < 0) {
			debug_printk("Write Failed %d\n",__LINE__);
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed %d\n",__LINE__);
			return -1;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received... exiting.. %d\n",__LINE__);
			return -1;
		}

		g_bload_buf[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf[i + 1];
		}

		g_bload_buf[i + 1] = checksum;

		ret = ar1335_write(client, g_bload_buf, i + 2);
		if (ret < 0) {
			debug_printk("Write Failed %d\n",__LINE__);
			return -1;
		}
		msleep(10);
 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf, 1);
		if (ret < 0) {
			debug_printk("Read Failed %d\n",__LINE__);
			return -1;
		}


		if (g_bload_buf[0] == RESP_BUSY)
		{
			printk("RESP_BUSY");
			goto poll_busy;
		}

		if (g_bload_buf[0] != RESP_ACK) {
			/*   NACK Received */
			debug_printk(" No ACK Received... exiting..%d \n",__LINE__);
//			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
		return 0;
	} else {

		/*   Unhandled Type */
		debug_printk("Unhandled Command Type \n");
		return -1;
	}

	return 0;
}


/*------------------Flashing firmware in MCU---------------*/

int mcu_bload_update_fw(struct i2c_client *client) {

	/* exclude NULL character at end of string */
	unsigned long hex_file_size = ARRAY_SIZE(g_mcu_fw_buf) - 1;
	unsigned char *wbuf = devm_kzalloc(&client->dev, MAX_BUF_LEN, GFP_KERNEL);
	int i = 0, recindex = 0, ret = 0;

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (g_mcu_fw_buf[i] == ':')) {
			/*  debug_printk("Start of a Record \n"); */
		} else if (g_mcu_fw_buf[i] == CR) {
			/*   No Implementation */
		} else if (g_mcu_fw_buf[i] == LF) {
			if (recindex == 0) {
				/*   Parsing Complete */
				break;
			}

			/*   Analyze Packet and Send Commands */
			ret = mcu_bload_parse_send_cmd(client, wbuf, recindex);
			if (ret < 0) {
				debug_printk("Error in Processing Commands \n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf[i])) < 0) {
				debug_printk("Invalid Character - 0x%02x !! \n", g_mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf[i])) < 0) {
				debug_printk("Invalid Character - 0x%02x !!!! \n",g_mcu_fw_buf[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}


	/* ------------ PROGRAM FLASH END ----------------------- */
	devm_kfree(&client->dev,wbuf);
	return ret;
}



/*---------------------Reading MCU memory during Flash check------------------*/

int mcu_bload_read(struct i2c_client *client,
		   unsigned int g_bload_flashaddr, char *bytearray,
		   unsigned int len)
{
	int ret = 0;

	g_bload_buf[0] = BL_READ_MEM;
	g_bload_buf[1] = ~(BL_READ_MEM);

	ret = ar1335_write(client, g_bload_buf, 2);
	if (ret < 0) {
		debug_printk("Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		debug_printk(" No ACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = (g_bload_flashaddr & 0xFF000000) >> 24;
	g_bload_buf[1] = (g_bload_flashaddr & 0x00FF0000) >> 16;
	g_bload_buf[2] = (g_bload_flashaddr & 0x0000FF00) >> 8;
	g_bload_buf[3] = (g_bload_flashaddr & 0x000000FF);
	g_bload_buf[4] =
	    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = ar1335_write(client, g_bload_buf, 5);
	if (ret < 0) {
		debug_printk("Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		debug_printk(" No ACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf[0] = len - 1;
	g_bload_buf[1] = ~(len - 1);

	ret = ar1335_write(client, g_bload_buf, 2);
	if (ret < 0) {
		debug_printk("Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Read Failed \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		debug_printk(" No ACK Received... exiting.. \n");
		return -1;
	}

	ret = ar1335_read(client, bytearray, len);
	if (ret < 0) {
		debug_printk("Read Failed \n");
		return -1;
	}

	return 0;
}


/*------------------Flash check Firmware-----------------*/

int mcu_bload_verify_flash(struct i2c_client *client,
			   unsigned short int orig_crc) {

	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
		    (client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
			debug_printk(" i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read
		    (client, flash_addr + i, bytearray, (FLASH_SIZE - i))
		    < 0) {
			debug_printk(" i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
		debug_printk(" CRC verification fail !! 0x%04x != 0x%04x \n",
		       orig_crc, calc_crc);
		return -1;
	}

	debug_printk(" CRC Verification Success 0x%04x == 0x%04x \n",
		     orig_crc, calc_crc);

	return 0;
}


/*--------------Initialize new Firmware for I2c Commands----------*/

int mcu_bload_go(struct i2c_client *client) {

	int ret = 0;

	g_bload_buf[0] = BL_GO;
	g_bload_buf[1] = ~(BL_GO);

	ret = ar1335_write(client, g_bload_buf, 2);
	if (ret < 0) {
		debug_printk("Write Failed \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Failed Read 1 \n");
		return -1;
	}

	/*   Start Address */
	g_bload_buf[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf[4] =
	    g_bload_buf[0] ^ g_bload_buf[1] ^ g_bload_buf[2] ^ g_bload_buf[3];

	ret = ar1335_write(client, g_bload_buf, 5);
	if (ret < 0) {
		debug_printk("Write Failed \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf, 1);
	if (ret < 0) {
		debug_printk("Failed Read 1 \n");
		return -1;
	}

	if (g_bload_buf[0] != RESP_ACK) {
		/*   NACK Received */
		debug_printk(" No ACK Received... exiting.. \n");
		return -1;
	}

	return 0;
}

/*--------------------Updating firmware in MCU ---------------------*/

static int mcu_fw_update(struct i2c_client *client, unsigned char *mcu_fw_version) {

	int ret = 0;
	g_bload_crc16 = 0;

	/* Read Firmware version from bootloader MCU */
	ret = mcu_bload_get_version(client);
	if (ret < 0) {
		debug_printk(" Error in Get Version \n");
		goto exit;
	}
	else
	{
		debug_printk(" Got bload version successfully\n");
	}


	/* Erase firmware present in the MCU and flash new firmware*/
	ret = mcu_bload_erase_flash(client);
	if (ret < 0) {
		debug_printk(" Error in Erase Flash \n");
		goto exit;
	}

        printk("Erase Flash Success !! \n");

	/* Read the firmware present in the text file */
	if ((ret = mcu_bload_update_fw(client)) < 0) {
		debug_printk(" Write Flash FAIL !! \n");
		goto exit;
	}

	/* Verify the checksum for the update firmware */
	if ((ret = mcu_bload_verify_flash(client, g_bload_crc16)) < 0) {
		debug_printk(" verify_flash FAIL !! \n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	/* I2C GO Command */
	if ((ret = mcu_bload_go(client)) < 0) {
		debug_printk(" i2c_bload_go FAIL !! \n");
		goto exit;
	}
	
	if(mcu_fw_version) {
	debug_printk("(%s) - Firmware Updated - (%.32s)\n",__func__, mcu_fw_version);
	}

 exit:
	return ret;
}



/*-------------Getting current status of given Command ID----------*/

static int mcu_get_cmd_status(struct i2c_client *client,
			      uint8_t * cmd_id, uint16_t * cmd_status,
			      uint8_t * ret_code)
{
	uint32_t payload_len = 0;
	uint8_t orig_crc = 0, calc_crc = 0;
	int err = 0;


	payload_len = 1;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ar1335_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_STATUS;
	mc_data[2] = *cmd_id;
	err = ar1335_write(client, mc_data, 3);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 3);
	if (orig_crc != calc_crc) {
		debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		return -EINVAL;
	}

	*cmd_id = mc_ret_data[2];
	*cmd_status = mc_ret_data[3] << 8 | mc_ret_data[4];
	*ret_code = mc_ret_data[payload_len - 1];

	debug_printk(" mcu_get_cmd_status successfull\n");
	return 0;
}

/*----------------Configuring MIPI lanes to transfer image data------------*/

static int mcu_lane_configuration(struct i2c_client *client, struct ar1335 *sensor) {

	int ret = 0, err;
	uint16_t payload_data;
        unsigned char mc_data[10];
        uint32_t payload_len = 0;
        uint16_t cmd_status = 0; 
        uint8_t retcode = 0, cmd_id = 0;

      
        mutex_lock(&mcu_i2c_mutex_1335_mcu);

	payload_len = 2; 
		
	mc_data[0] = CMD_SIGNATURE;
        mc_data[1] = CMD_ID_LANE_CONFIG;
        mc_data[2] = payload_len >> 8;
        mc_data[3] = payload_len & 0xFF;
        mc_data[4] = errorcheck(&mc_data[2], 2);

        ar1335_write(client, mc_data, TX_LEN_PKT);

        /* Second Txn */
        mc_data[0] = CMD_SIGNATURE;
        mc_data[1] = CMD_ID_LANE_CONFIG;

        /* Lane Configuration */
	payload_data = sensor->mipi_lane_config == 4 ? NUM_LANES_4 : NUM_LANES_2; 
        mc_data[2] = payload_data >> 8;
        mc_data[3] = payload_data & 0xff;

       	/* CRC */
       	mc_data[4] = errorcheck(&mc_data[2], payload_len);
        err = ar1335_write(client, mc_data, payload_len+3);
	
        if (err != 0) {
                debug_printk(" %s(%d) MCU Set Ctrl Error - %d \n", __func__, __LINE__, err);
                ret = -1;
                goto exit;
        }

	while (1) {
		yield();
                cmd_id = CMD_ID_LANE_CONFIG;
                if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
                    0) {
                        debug_printk(" %s(%d) MCU Get CMD Status Error \n", __func__,__LINE__);
                        ret = -1;
                        goto exit;
                }

                if ((cmd_status == MCU_CMD_STATUS_ISP_UNINIT) &&
                    (retcode == ERRCODE_SUCCESS)) {
                        ret = 0;
                        goto exit;
                }

                if ((retcode != ERRCODE_BUSY) &&
                    ((cmd_status != MCU_CMD_STATUS_ISP_UNINIT))) {
                       debug_printk("(%s) %d MCU Get CMD Error STATUS = 0x%04x RET = 0x%02x\n", __func__, __LINE__, cmd_status, retcode);
                        ret = -1;
                        goto exit;
                }
        }

 exit:
       
        mutex_unlock(&mcu_i2c_mutex_1335_mcu);

        return ret;
}


/*----------------Initializing camera------------*/

static int mcu_isp_init(struct i2c_client *client) {

	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;
	
	pr_info("mcu_isp_init\n");
	/* check current status - if initialized, no need for Init */
	cmd_id = CMD_ID_INIT_CAM;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		debug_printk(" %s(%d) Error \n", __func__, __LINE__);
		return -EIO;
	}

	if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
	    (retcode == ERRCODE_SUCCESS)) {
		debug_printk(" Already Initialized !! \n");
		return 0;
	}

	/* call ISP init command */

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ar1335_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_INIT_CAM;
	err = ar1335_write(client, mc_data, 2);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		mdelay(500);

		cmd_id = CMD_ID_INIT_CAM;
		if (mcu_get_cmd_status
		    (client, &cmd_id, &cmd_status, &retcode) < 0) {
			debug_printk(" %s(%d) Error \n",
			       __func__, __LINE__);
			return -EIO;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || (retcode == ERRCODE_ALREADY))) {
			debug_printk("ISP Already Initialized !! \n");
			return 0;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			debug_printk("(%s) %d Init Error STATUS = 0x%04x RET = 0x%02x\n", __func__, __LINE__, cmd_status, retcode);
			return -EIO;
		}
	}
	debug_printk("ETIMEDOUT Error\n");
	return -ETIMEDOUT;
}



/*---------------Getting sensor ID from camera MCU----------*/

static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id) {
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;

	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	/* Read the version info. from Micro controller */

	payload_len = 0;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);


	ar1335_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_SENSOR_ID;
	err = ar1335_write(client, mc_data, 2);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}
	else
	{
		debug_printk(" I2c read success %d\n",__LINE__);
	}


	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}
	else
	{
	debug_printk(" CRC mismatch \n");
	}

	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		debug_printk(" %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}
	else
	{
	debug_printk(" ERRCODE_SUCCESS \n");
	}
	msleep(10);

	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		debug_printk(" %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		debug_printk(" %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}


	*sensor_id = mc_ret_data[2] << 8 | mc_ret_data[3];

 exit:
	
	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;
}



/*----------------------------------------------------------------------------------------------------------------*/
/*---------------------Getting camera controls from MCU--------------------*/

static int mcu_list_ctrls(struct i2c_client *client,
			  ISP_CTRL_INFO * mcu_cam_ctrl, struct ar1335 *sensor)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0,retry=30;

	mutex_lock(&mcu_i2c_mutex_1335_mcu);


	/* Array of Ctrl Info */
	while (retry-- > 0) {
		
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err = ar1335_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
                        debug_printk(" %s(%d) Error - %d \n",
                               __func__, __LINE__, err);
                        continue;
                }
		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_CTRL_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = ar1335_write(client, mc_data, 5);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			continue;
		}

		err = ar1335_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			sensor->num_ctrls = index;
			break;
		}

		payload_len =
		    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			continue;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = ar1335_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			continue;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			continue;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			continue;
		}

		if(mcu_cam_ctrl != NULL) {

			/* append ctrl info in array */
			mcu_cam_ctrl[index].ctrl_id =
				mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
				<< 8 | mc_ret_data[5];
			mcu_cam_ctrl[index].ctrl_type = mc_ret_data[6];

			switch (mcu_cam_ctrl[index].ctrl_type) {
				case CTRL_STANDARD:
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min =
						mc_ret_data[7] << 24 | mc_ret_data[8] << 16
						| mc_ret_data[9] << 8 | mc_ret_data[10];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max =
						mc_ret_data[11] << 24 | mc_ret_data[12] <<
						16 | mc_ret_data[13]
						<< 8 | mc_ret_data[14];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def =
						mc_ret_data[15] << 24 | mc_ret_data[16] <<
						16 | mc_ret_data[17]
						<< 8 | mc_ret_data[18];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step =
						mc_ret_data[19] << 24 | mc_ret_data[20] <<
						16 | mc_ret_data[21]
						<< 8 | mc_ret_data[22];
					break;

				case CTRL_EXTENDED:
					/* Not Implemented */
					break;
			}

			sensor->ctrldb[index] = mcu_cam_ctrl[index].ctrl_id;
		}
		index++;
		if(retry == 0) {
			ret = -EIO;
                        goto exit;
		}
	}

exit:

	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;

}


/*----------------Configuring stream---------------*/

static int mcu_stream_config(struct i2c_client *client, uint32_t format,
			     int mode, int frate_index)
{
	struct ar1335 *sensor = to_ar1335_data(client);
	
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;


	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	cmd_id = CMD_ID_STREAM_CONFIG;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {

		ret = -EIO;
		goto exit;
	}
	else
	{
		debug_printk(" Success in get_cmd_status %d\n",__LINE__);

	}

	if ((cmd_status != MCU_CMD_STATUS_SUCCESS) ||
	    (retcode != ERRCODE_SUCCESS)) {

		debug_printk(" ISP is Unintialized or Busy STATUS = 0x%04x Errcode = 0x%02x !! \n",
		     cmd_status, retcode);

		ret = -EBUSY;
		goto exit;
	}
	else
	{
		debug_printk(" Success ISP status  %d\n",__LINE__);
	}


	for (loop = 0;(&sensor->streamdb[loop]) != NULL; loop++) {
		if (sensor->streamdb[loop] == mode) {
			index = loop + frate_index;
			break;
		}
	}
	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if(sensor->prev_index == index) {
		debug_printk(" Skipping Previous mode set ... \n");
		ret = 0;
		goto exit;
	}

issue_cmd:
	
	payload_len = 14;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ar1335_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_STREAM_CONFIG;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;

	/* Format Fourcc - currently only UYVY */
	mc_data[4] = format >> 24;
	mc_data[5] = format >> 16;
	mc_data[6] = format >> 8;
	mc_data[7] = format & 0xFF;

	/* width */
	mc_data[8] = sensor->mcu_cam_frmfmt[mode].size.width >> 8;
	mc_data[9] = sensor->mcu_cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data[10] = sensor->mcu_cam_frmfmt[mode].size.height >> 8;
	mc_data[11] = sensor->mcu_cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data[12] = sensor->mcu_cam_frmfmt[mode].framerates[frate_index] >> 8;
	mc_data[13] = sensor->mcu_cam_frmfmt[mode].framerates[frate_index] & 0xFF;


	/* frame rate denom */
	mc_data[14] = 0x00;
	mc_data[15] = 0x01;


	mc_data[16] = errorcheck(&mc_data[2], 14);
	err = ar1335_write(client, mc_data, 17);
	if (err != 0) {

		ret = -EIO;
		goto exit;
	}
	else
	{
		debug_printk(" Success I2C write  %d\n",__LINE__);
	}

	while (--retry > 0) {
		cmd_id = CMD_ID_STREAM_CONFIG;
		if (mcu_get_cmd_status
		    (client, &cmd_id, &cmd_status, &retcode) < 0) {
		
			debug_printk(" %s(%d) MCU GET CMD Status Error : loop : %d \n",
				__func__, __LINE__, loop);
		
			ret = -EIO;
			goto exit;
		}
		else
		{
		debug_printk(" Success in get_cmd_status %d\n",__LINE__);
		}


		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}
		else
		{
                debug_printk(" cmd_status = %d  retcode = %d \n", cmd_status , retcode);
		}

		if(retcode == ERRCODE_AGAIN) {
			/* Issue Command Again if Set */
			debug_printk(" retry -> ERRCODE_AGAIN\n");
			retry = 1000;
			goto issue_cmd;
		}
		else
		{
                  debug_printk(" not trying again  %d\n",__LINE__);
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
		
			debug_printk(" (%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -EIO;

		goto exit;
		}
		else
		{
		 debug_printk(" line %d\n",__LINE__);
		}

		/* Delay after retry */
		mdelay(10);
	}

	ret = -ETIMEDOUT;

exit:

	if(!ret)
		sensor->prev_index = index;

	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;
}


/*------------------------Getting formats from camera MCU-----------------*/

static int mcu_list_fmts(struct i2c_client *client, ISP_STREAM_INFO *stream_info, int *frm_fmt_size,struct ar1335 *sensor)
{
	uint32_t payload_len = 0, err = 0,fps;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0, skip = 0;
	uint16_t index = 0, mode = 0;

	int loop = 0, num_frates = 0, ret = 0;

	/* Stream Info Variables */

	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	/* List all formats from MCU and append to mcu_ar1335_frmfmt array */

	for (index = 0;; index++) {
		
		payload_len = 2;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		ar1335_write(client, mc_data, TX_LEN_PKT);

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_GET_STREAM_INFO;
		mc_data[2] = index >> 8;
		mc_data[3] = index & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);
		err = ar1335_write(client, mc_data, 5);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = ar1335_read(client, mc_ret_data, RX_LEN_PKT);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[4];
		calc_crc = errorcheck(&mc_ret_data[2], 2);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data[2] << 8) | mc_ret_data[3]) == 0) {
			if(stream_info == NULL) {
				*frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
			}
			break;
		}

		payload_len =
		    ((mc_ret_data[2] << 8) | mc_ret_data[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data[5];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data, 0x00, payload_len);
		err = ar1335_read(client, mc_ret_data, payload_len);
		if (err != 0) {
			debug_printk(" %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			debug_printk(" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			debug_printk(" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}
		if(stream_info != NULL) {

			debug_printk(" Stream info not null ...reading from MCU\n");
		/* check if any other format than UYVY is queried - do not append in array */
		stream_info->fmt_fourcc =
		    mc_ret_data[2] << 24 | mc_ret_data[3] << 16 | mc_ret_data[4]
		    << 8 | mc_ret_data[5];
		stream_info->width = mc_ret_data[6] << 8 | mc_ret_data[7];
		stream_info->height = mc_ret_data[8] << 8 | mc_ret_data[9];
		stream_info->frame_rate_type = mc_ret_data[10];

		switch (stream_info->frame_rate_type) {
		case FRAME_RATE_DISCRETE:
			stream_info->frame_rate.disc.frame_rate_num =
			    mc_ret_data[11] << 8 | mc_ret_data[12];

			stream_info->frame_rate.disc.frame_rate_denom =
			    mc_ret_data[13] << 8 | mc_ret_data[14];

			break;

		case FRAME_RATE_CONTINOUS:
			debug_printk(" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,"
			     "which is unsupported !! \n", index);

			continue;
		}
		fps= (int)(stream_info->frame_rate.disc.frame_rate_num / stream_info->frame_rate.disc.frame_rate_denom);


		switch (stream_info->fmt_fourcc) {
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_GREY:
		case V4L2_PIX_FMT_Y12:
				
			/* ar1335_codes is already populated with V4L2_MBUS_FMT_UYVY8_1X16 */
			/* check if width and height are already in array - update frame rate only */
			for (loop = 0; loop < (mode); loop++) {
				if ((sensor->mcu_cam_frmfmt[loop].size.width ==
				     stream_info->width)
				    && (sensor->mcu_cam_frmfmt[loop].size.height ==
					stream_info->height)) {

					num_frates =
					    sensor->mcu_cam_frmfmt
					    [loop].num_framerates;
					*((int *)(sensor->mcu_cam_frmfmt[loop].framerates) + num_frates)
					    = (int)(stream_info->frame_rate.
						    disc.frame_rate_num /
						    stream_info->frame_rate.
						    disc.frame_rate_denom);

					sensor->mcu_cam_frmfmt
					    [loop].num_framerates++;

					sensor->streamdb[index] = loop;
					skip = 1;
					break;
				}
			}

			if (skip) {
				skip = 0;
				continue;
			}

			/* Add Width, Height, Frame Rate array, Mode into mcu_ar1335_frmfmt array */


			sensor->mcu_cam_frmfmt[mode].size.width = stream_info->width;
			sensor->mcu_cam_frmfmt[mode].size.height =
			    stream_info->height;
			num_frates = sensor->mcu_cam_frmfmt[mode].num_framerates;
/*
                        debug_printk("  Index = 0x%04x , format = 0x%08x, width = %hu,"
			 " height = %hu, frate num = %hu \n", index, stream_info->fmt_fourcc,stream_info->width,stream_info->height,fps);

			debug_printk("  sensor->mcu_cam_frmfmt[mode].size.width x sensor->mcu_cam_frmfmt[mode].size.height =%dx%d\n",sensor->mcu_cam_frmfmt[mode].size.width,sensor->mcu_cam_frmfmt[mode].size.height);
*/
			*((int *)(sensor->mcu_cam_frmfmt[mode].framerates) + num_frates) =
			    (int)(stream_info->frame_rate.disc.frame_rate_num /
				  stream_info->frame_rate.disc.frame_rate_denom);

			sensor->mcu_cam_frmfmt[mode].num_framerates++;

			sensor->mcu_cam_frmfmt[mode].mode = mode;
			sensor->streamdb[index] = mode;
			mode++;


			break;

		default:
			debug_printk(" The Stream format at index 0x%04x has format 0x%08x ,"
			     "which is unsupported !! \n", index,
			     stream_info->fmt_fourcc);
		}

		}



	}

 exit:
	
	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;
}


/*----------------Stream On------------*/ 

static int mcu_cam_stream_on(struct i2c_client *client) {

        unsigned char mc_data[100];
        uint32_t payload_len = 0;

        uint16_t cmd_status = 0;
        uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000,status_retry=1000, err = 0;
	debug_printk(" %s %d\n",__func__,__LINE__);

        mutex_lock(&mcu_i2c_mutex_1335_mcu);

	while(retry-- > 0) {
	
		payload_len = 0;

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		mc_data[2] = payload_len >> 8;
		mc_data[3] = payload_len & 0xFF;
		mc_data[4] = errorcheck(&mc_data[2], 2);

		err= ar1335_write(client, mc_data, TX_LEN_PKT);
		if (err != 0) {
			debug_printk(" %s(%d) MCU Stream On Write Error - %d \n", __func__,
					__LINE__, err);
			continue;
		}

		mc_data[0] = CMD_SIGNATURE;
		mc_data[1] = CMD_ID_STREAM_ON;
		err = ar1335_write(client, mc_data, 2);
		if (err != 0) {
			debug_printk(" %s(%d) MCU Stream On Write Error - %d \n", __func__,
					__LINE__, err);
			continue;
		}

		while (status_retry-- > 0) {
			/* Some Sleep for init to process */
			yield();

			cmd_id = CMD_ID_STREAM_ON;
			if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
					0) {
				debug_printk(" %s(%d) MCU Get CMD Stream On Error \n", __func__,
						__LINE__);
				err = -1;
				goto exit;
			}

			if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
					(retcode == ERRCODE_SUCCESS)) {
				debug_printk(" %s %d MCU Stream On Success !! \n", __func__, __LINE__);
				err = 0;
				goto exit;
			}

			if ((retcode != ERRCODE_BUSY) &&
					((cmd_status != MCU_CMD_STATUS_PENDING))) {
				debug_printk("(%s) %d MCU Get CMD Stream On Error STATUS = 0x%04x RET = 0x%02x\n",
						__func__, __LINE__, cmd_status, retcode);
				err = -1;
				goto exit;
			}
			mdelay(1);

                  debug_printk(" Stream ON cmd status from MCU is  cmd_status=0x%04x\n",cmd_status);

		}
		if(retry == 0)
		{	
			debug_printk(" retry expired \n");
			err = -1;
		break;
		}
	}
 exit:
	        mutex_unlock(&mcu_i2c_mutex_1335_mcu);
		return err;

}


/*-----------------Getting UI information from Camera-------------*/

static int mcu_get_ctrl_ui(struct i2c_client *client,
			   ISP_CTRL_INFO * mcu_ui_info, int index)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, i = 0, err = 0;

	
	debug_printk(" %s\n",__func__);
	mutex_lock(&mcu_i2c_mutex_1335_mcu);

	payload_len = 2;

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = payload_len >> 8;
	mc_data[3] = payload_len & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);

	ar1335_write(client, mc_data, TX_LEN_PKT);

	mc_data[0] = CMD_SIGNATURE;
	mc_data[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data[2] = index >> 8;
	mc_data[3] = index & 0xFF;
	mc_data[4] = errorcheck(&mc_data[2], 2);
	err = ar1335_write(client, mc_data, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[4];
	calc_crc = errorcheck(&mc_ret_data[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data[2] << 8) | mc_ret_data[3]) + HEADER_FOOTER_SIZE;
	errcode = mc_ret_data[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	memset(mc_ret_data, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data[payload_len - 2];
	calc_crc =
	    errorcheck(&mc_ret_data[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_name, &mc_ret_data[2],MAX_CTRL_UI_STRING_LEN);

	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type = mc_ret_data[34];
	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags = mc_ret_data[35] << 8 |
	    mc_ret_data[36];

	if (mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type == V4L2_CTRL_TYPE_MENU) {
		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem = mc_ret_data[37];

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu =
		    devm_kzalloc(&client->dev,((mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem +1) * sizeof(char *)), GFP_KERNEL);
		for (i = 0; i < mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem; i++) {
			mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] =
			    devm_kzalloc(&client->dev,MAX_CTRL_UI_STRING_LEN, GFP_KERNEL);
			strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i],
				&mc_ret_data[38 +(i *MAX_CTRL_UI_STRING_LEN)], MAX_CTRL_UI_STRING_LEN);

			debug_printk(" Menu Element %d : %s \n",
				     i, mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i]);
		}

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] = NULL;
	}

 exit:
	
	mutex_unlock(&mcu_i2c_mutex_1335_mcu);

	return ret;

}


/*----------------Adding controls to camera--------------*/

static int ar1335_try_add_ctrls(struct ar1335 *sensor, int index,
				ISP_CTRL_INFO * mcu_ctrl)
{
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_ctrl_config custom_ctrl_config;

	sensor->ctrls.error = 0;

	// Not Implemented for V4L2_CID_FRAME_SYNC
	if(mcu_ctrl->ctrl_id == 0x9a092a)
		return 0;

	// Try Enumerating in standard controls
	sensor->ctrl[index] =
	    v4l2_ctrl_new_std(&sensor->ctrls,
			      &ar1335_ctrl_ops,
			      mcu_ctrl->ctrl_id,
			      mcu_ctrl->ctrl_data.std.ctrl_min,
			      mcu_ctrl->ctrl_data.std.ctrl_max,
			      mcu_ctrl->ctrl_data.std.ctrl_step,
			      mcu_ctrl->ctrl_data.std.ctrl_def);
	if (sensor->ctrl[index] != NULL) {
		debug_printk(" %d. Initialized Control 0x%08x - %s \n",
			     index, mcu_ctrl->ctrl_id,
			     sensor->ctrl[index]->name);
		return 0;
	}

	if(mcu_ctrl->ctrl_id == V4L2_CID_EXPOSURE_AUTO)
		goto custom;

	/* Try Enumerating in standard menu */
	sensor->ctrls.error = 0;
	sensor->ctrl[index] =
	    v4l2_ctrl_new_std_menu(&sensor->ctrls,
				   &ar1335_ctrl_ops,
				   mcu_ctrl->ctrl_id,
				   mcu_ctrl->ctrl_data.std.ctrl_max,
				   0, mcu_ctrl->ctrl_data.std.ctrl_def);
	if (sensor->ctrl[index] != NULL) {
		debug_printk(" %d. Initialized Control Menu 0x%08x - %s \n",
			     index, mcu_ctrl->ctrl_id,
			     sensor->ctrl[index]->name);
		return 0;
	}

custom:
	sensor->ctrls.error = 0;
	memset(&custom_ctrl_config, 0x0, sizeof(struct v4l2_ctrl_config));

	if (mcu_get_ctrl_ui(client, mcu_ctrl, index)!= ERRCODE_SUCCESS) {
		debug_printk(" Error Enumerating Control 0x%08x !! \n",
			mcu_ctrl->ctrl_id);
		return -EIO;
	}

	/* Fill in Values for Custom Ctrls */
	custom_ctrl_config.ops = &ar1335_ctrl_ops;
	custom_ctrl_config.id = mcu_ctrl->ctrl_id;
	/* Do not change the name field for the control */
	custom_ctrl_config.name = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_name;

	/* Sample Control Type and Flags */
	custom_ctrl_config.type = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type;
	custom_ctrl_config.flags = mcu_ctrl->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags;

	custom_ctrl_config.min = mcu_ctrl->ctrl_data.std.ctrl_min;
	custom_ctrl_config.max = mcu_ctrl->ctrl_data.std.ctrl_max;
	custom_ctrl_config.step = mcu_ctrl->ctrl_data.std.ctrl_step;
	custom_ctrl_config.def = mcu_ctrl->ctrl_data.std.ctrl_def;

	if (custom_ctrl_config.type == V4L2_CTRL_TYPE_MENU) {
		custom_ctrl_config.step = 0;
		custom_ctrl_config.type_ops = NULL;

		custom_ctrl_config.qmenu =
			(const char *const *)(mcu_ctrl->ctrl_ui_data.ctrl_menu_info.menu);
	}

	sensor->ctrl[index] =
	    v4l2_ctrl_new_custom(&sensor->ctrls,
				 &custom_ctrl_config, NULL);
	if (sensor->ctrl[index] != NULL) {
		debug_printk(" %d. Initialized Custom Ctrl 0x%08x - %s \n",
			     index, mcu_ctrl->ctrl_id,
			     sensor->ctrl[index]->name);
		return 0;
	}

	debug_printk(" %d.  default: Failed to init 0x%08x ctrl Error - %d \n",
		index, mcu_ctrl->ctrl_id, sensor->ctrls.error);
	return -EINVAL;
}

static int ar1335_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations ar1335_sd_media_ops = {
	.link_setup = ar1335_link_setup,
};

static int ar1335_probe(struct i2c_client *client)
{

    struct camera_common_data *common_data;
	struct device *dev = &client->dev;
	struct ar1335 *sensor;
	int ret,i;
	struct v4l2_subdev *sd;

	struct device_node *np = client->dev.of_node;
	u32 xclk_freq;

	unsigned char fw_version[32] = {0}, bin_fw_version[32] = {0};
	int frm_fmt_size = 0, loop;   
	uint16_t sensor_id = 0;
	uint32_t mipi_lane = 0;
	int err = 0, pwdn_gpio_toggle = 0;

	debug_printk(" %s\n",__func__);

	io_expander=client;

	common_data = devm_kzalloc(&client->dev,
			sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data)
		return -ENOMEM;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->cur_mode = &supported_modes[3]; 

	mipi_lane = 2;  

	sensor->i2c_client = client;
	sensor->s_data = common_data;
	sensor->s_data->dev = &client->dev;
	sensor->subdev.dev = &client->dev;
	common_data->sensor = (void *)sensor;
	sensor->mipi_lane_config = mipi_lane;
	sensor->last_sync_mode = 1;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = ar1335_parse_dt(np, sensor);
		if (ret) {
			debug_printk("DT parsing error: %d\n", ret);
			return ret;
		}
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		debug_printk("could not get xclk");
		return PTR_ERR(sensor->xclk);
	}

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != AR1335_MCU_CLK) {
		debug_printk("Unsupported clock frequency: %u, only %d is supported\n", xclk_freq, AR1335_MCU_CLK);
		return -EINVAL;
	}

	/* Request the power down GPIO asserted */
	sensor->pwdn = devm_gpiod_get_optional(&client->dev, "pwdn",
					       GPIOD_OUT_HIGH);    //need to check optional

	gpiod_set_value_cansleep(sensor->pwdn, 1);   //IO Expander Reset
        msleep(2);
	gpiod_set_value_cansleep(sensor->pwdn, 0);   //IO Expander Reset
        msleep(10);

 	io_expander_config(IO_EXP_CONFIG_PORT0, IO_EXP_SET_PORT_OUPUT);
	msleep(10);
 	io_expander_config(IO_EXP_OUTPUT_PORT0, 0xFF);
	msleep(10);

	io_exp_toggle_pwdn(0);
	msleep(1);
	io_exp_toggle_reset(0);
	msleep(1);
	io_exp_toggle_reset(1);
	msleep(10);

        ret = mcu_get_fw_version(client, fw_version, bin_fw_version);
	if (ret != 0) {
		debug_printk("AR1335: In Boot mode...Flashing New Firmware...wait for 90 secs approx\n"); 
		io_exp_toggle_pwdn(0);
		msleep(10);
 		io_exp_toggle_reset(0);
		msleep(200);
 		io_exp_toggle_pwdn(1);
		msleep(1);
 		io_exp_toggle_reset(1);
		msleep(10);

		for(loop = 0;loop < 20; loop++) {
			err = mcu_bload_get_version(client);
			if (err < 0) {
				// Retry Method
				msleep(20);
				continue;
			} else {
				debug_printk(" Get Bload Version Success\n");
				pwdn_gpio_toggle = 1;
				break;
			}
		}
		if(loop == 20) {
			debug_printk("Error updating firmware \n");
			return -EINVAL;
		}

		if (mcu_fw_update(client, NULL) < 0) {
			debug_printk("Error updating firmware \n");
			return -EFAULT;
		}
                else
                {
                        debug_printk("Firmware updated successfully\n");
                }


		if( pwdn_gpio_toggle == 1)
                        io_exp_toggle_pwdn(0);

		msleep(100);

		for(loop = 0;loop < 100; loop++) {
			err = mcu_get_fw_version(client, fw_version, bin_fw_version);
			if (err < 0) {
				//Retry 
				msleep(1);
				continue;
			} else {
				debug_printk(" Get FW Version Success\n");
				break;
			}
		}
		if(loop == 100) {
			debug_printk("Error updating firmware \n");
			return -EINVAL;
		}						
		debug_printk("Current Firmware Version - (%.32s)",fw_version);
	} else {
		/* Same firmware version in MCU and Text File */
		debug_printk(" Skipping Flashing module ...... Current Firmware Version - (%.32s)",fw_version);
	}


	// Configure MIPI Lanes of the Sensor 
	if (mcu_lane_configuration(client, sensor) < 0) {
		debug_printk("%s, Lane Configuration failed\n",__func__);
		return -EFAULT;
	}
	else
	{
		debug_printk(" Lane configuration success\n");
	}

	//Query the number of controls from MCU
	if(mcu_list_ctrls(client, NULL, sensor) < 0) {
		debug_printk("%s, Failed in mcu_list_ctrls \n", __func__);
		return -EFAULT;
	}
	else
	{
		debug_printk("%s, Success  in mcu_list_ctrls \n", __func__);
	}

	//Query the number for Formats available from MCU
	if(mcu_list_fmts(client, NULL, &frm_fmt_size,sensor) < 0) {
		debug_printk("%s, Failed to init formats \n", __func__);
		return -EFAULT;
	}

	sensor->mcu_ctrl_info = devm_kzalloc(&client->dev, sizeof(ISP_CTRL_INFO) * sensor->num_ctrls, GFP_KERNEL);
	if(!sensor->mcu_ctrl_info) {
		debug_printk("Unable to allocate memory \n");
		return -ENOMEM;
	}

	sensor->ctrldb = devm_kzalloc(&client->dev, sizeof(uint32_t) * sensor->num_ctrls, GFP_KERNEL);
	if(!sensor->ctrldb) {
		debug_printk("Unable to allocate memory \n");
		return -ENOMEM;
	}

	sensor->stream_info = devm_kzalloc(&client->dev, sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!sensor->stream_info) {
		debug_printk("Unable to allocate memory \n");
		return -ENOMEM;
	}


	sensor->streamdb = devm_kzalloc(&client->dev, sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!sensor->streamdb) {
		debug_printk("Unable to allocate memory \n");
		return -ENOMEM;
	}

	sensor->mcu_cam_frmfmt = devm_kzalloc(&client->dev, sizeof(struct camera_common_frmfmt) * (frm_fmt_size), GFP_KERNEL);
	if(!sensor->mcu_cam_frmfmt) {
		debug_printk("Unable to allocate memory \n");
		return -ENOMEM;
	}


	if (mcu_get_sensor_id(client, &sensor_id) < 0) {
		debug_printk("Unable to get MCU Sensor ID \n");
		return -EFAULT;
	}
	else
	{
		debug_printk("Success getting  MCU Sensor ID \n");
	}


	if (mcu_isp_init(client) < 0) {
		debug_printk("Unable to INIT ISP \n");
		return -EFAULT;
	}
        else
	{
		debug_printk(" INIT ISP Success \n");
	}


	for(loop = 0; loop < frm_fmt_size; loop++) {
		sensor->mcu_cam_frmfmt[loop].framerates = devm_kzalloc(&client->dev, sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if(!sensor->mcu_cam_frmfmt[loop].framerates) {
			debug_printk("Unable to allocate memory \n");
			return -ENOMEM;
		}
	}

	// Enumerate Formats 
	if (mcu_list_fmts(client, sensor->stream_info, &frm_fmt_size,sensor) < 0) 
	{
		debug_printk("Unable to List Fmts \n");
		return -EFAULT;
	}
        else
	{
		debug_printk("Successfull in mcu list Fmts frm_fmt_size is %d \n",frm_fmt_size);
	}

	sensor->i2c_client = client;
	sensor->s_data = common_data;
	sensor->s_data->dev = &client->dev;
	sensor->subdev.dev = &client->dev;
	sensor->prev_index = 0xFFFE;
	sensor->numfmts = frm_fmt_size;

	mutex_init(&sensor->lock);

	/* Initialise controls. */
	{
		ISP_CTRL_INFO *mcu_cam_ctrls=sensor->mcu_ctrl_info;
		if (mcu_list_ctrls(client,mcu_cam_ctrls,sensor) < 0) {
		    debug_printk(" Failed to init ctrls\n");
		    goto error;
		}

		v4l2_ctrl_handler_init(&sensor->ctrls, sensor->num_ctrls+1);

		for (i = 0; i < sensor->num_ctrls; i++) {

		    if (mcu_cam_ctrls[i].ctrl_type == CTRL_STANDARD) {
			    ar1335_try_add_ctrls(sensor, i,
					 &mcu_cam_ctrls[i]);
		    } else {
			debug_printk(" Invalid format\n");
		    }
		}
	}
	sensor->ctrl_handler.error = 0;

	sensor->pixel_clock = v4l2_ctrl_new_std(&sensor->ctrls,
	                                        &ar1335_ctrl_ops,
	                                        V4L2_CID_PIXEL_RATE,
	                                        1, INT_MAX, 1, 1);
	sensor->link_freq = v4l2_ctrl_new_int_menu(&sensor->ctrls,
	                                           &ar1335_ctrl_ops,
	                                           V4L2_CID_LINK_FREQ,
	                                           ARRAY_SIZE(link_freq_menu_items) - 1,
	                                           0, link_freq_menu_items);

        if (sensor->link_freq)
                sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

        sensor->sd.ctrl_handler = &sensor->ctrls;

        if (sensor->ctrls.error) {
                debug_printk(" %s: control initialization error %d\n",
                       __func__, sensor->ctrls.error);
                err = sensor->ctrls.error;
                goto error;
        }


	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		debug_printk("%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}


	sensor->sd.ctrl_handler = &sensor->ctrls;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &ar1335_subdev_ops);

	sd->internal_ops = &ar1335_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0)
		goto mutex_remove;

	sd->entity.ops = &ar1335_sd_media_ops;
#if 0
	if (sensor->pwdn) {
		gpiod_set_value_cansleep(sensor->pwdn, 0);
		msleep(PWDN_ACTIVE_DELAY_MS);
	}

	ret = ar1335_detect(sd);

	gpiod_set_value_cansleep(sensor->pwdn, 0);

	if (ret < 0)
		goto error;
#endif

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		goto error;

	debug_printk("AR1335 camera driver probed\n");
	return 0;
error:
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);
mutex_remove:
	mutex_destroy(&sensor->lock);
	return ret;
}

static void ar1335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1335 *sensor = to_state(sd);
	int loop = 0;

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&sensor->lock);

	/* Free up memory */
	for(loop = 0; loop < sensor->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem
			; loop++) {
		FREE_SAFE(&client->dev, sensor->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu[loop]);
	}

	FREE_SAFE(&client->dev, sensor->mcu_ctrl_info->ctrl_ui_data.ctrl_menu_info.menu);

	FREE_SAFE(&client->dev, sensor->mcu_ctrl_info);

	for(loop = 0; loop < sensor->numfmts; loop++ ) {
		FREE_SAFE(&client->dev, (void *)sensor->mcu_cam_frmfmt[loop].framerates);
	}

	FREE_SAFE(&client->dev, sensor->mcu_cam_frmfmt);

	FREE_SAFE(&client->dev, sensor->ctrldb);
	FREE_SAFE(&client->dev, sensor->streamdb);

	FREE_SAFE(&client->dev, sensor->stream_info);
	FREE_SAFE(&client->dev, fw_version);
	FREE_SAFE(&client->dev, sensor->s_data);
	FREE_SAFE(&client->dev, sensor);
	return;
}

static const struct i2c_device_id ar1335_id[] = {
	{ "ar1335_mcu", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar1335_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ar1335_of_match[] = {
	{ .compatible = "aptina,ar1335_mcu" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ar1335_of_match);
#endif

static struct i2c_driver ar1335_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ar1335_of_match),
		.name	= SENSOR_NAME,
	},
	.probe		= ar1335_probe,
	.remove		= ar1335_remove,
	.id_table	= ar1335_id,
};

module_i2c_driver(ar1335_driver);

MODULE_AUTHOR("E-Con systems India Pvt Ltd");
MODULE_DESCRIPTION("A low-level driver for ar1335 sensors");
MODULE_LICENSE("GPL v2");
