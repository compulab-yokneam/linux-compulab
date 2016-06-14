/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_dvi.c
 *
 * @brief MXC DVI driver
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <video/mxc_edid.h>
#include "mxc_dispdrv.h"
#include "../edid.h"

#define MXC_EDID_LENGTH (EDID_LENGTH*4)

#define DISPDRV_DVI	"dvi"

struct mxc_dvi_data {
	struct i2c_client *client;
	struct platform_device *pdev;
	struct platform_device *core_pdev;
	struct mxc_dispdrv_handle *disp_dvi;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[MXC_EDID_LENGTH];

	u32 ipu_id;
	u32 disp_id;
	u32 hp_gpio;
};

struct i2c_client *dvi_i2c;
struct mxc_dvi_data *g_dvi;

static ssize_t mxc_dvi_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);

	if (dvi->cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, mxc_dvi_show_state, NULL);

static int dvi_update(struct mxc_dvi_data *dvi)
{
	if (gpio_is_valid(dvi->hp_gpio))
		return gpio_get_value(dvi->hp_gpio);

	/* always connected */
	return 1;
}

static ssize_t mxc_dvi_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);

	strcpy(buf, dvi->fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, mxc_dvi_show_name, NULL);

static ssize_t mxc_dvi_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_dvi_data *dvi = dev_get_drvdata(dev);
	int i, j, len = 0;

	for (j = 0; j < MXC_EDID_LENGTH/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					dvi->edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, mxc_dvi_show_edid, NULL);

static void det_worker(struct work_struct *work)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct mxc_dvi_data *dvi =
		container_of(delay_work, struct mxc_dvi_data, det_work);
	char event_string[16];
	char *envp[] = { event_string, NULL };

	/* cable connection changes */
	if (dvi_update(dvi)) {
		u8 edid_old[MXC_EDID_LENGTH];
		dvi->cable_plugin = 1;
		sprintf(event_string, "EVENT=plugin");

		memcpy(edid_old, dvi->edid, MXC_EDID_LENGTH);

		if (mxc_edid_read(dvi->client->adapter, dvi->client->addr,
				dvi->edid, &dvi->edid_cfg, dvi->fbi) < 0)
			dev_err(&dvi->client->dev,
					"MXC dvi: read edid fail\n");
		else {
			if (!memcmp(edid_old, dvi->edid, MXC_EDID_LENGTH))
				dev_info(&dvi->client->dev,
					"MXC dvi: same edid\n");
			else if (dvi->fbi->monspecs.modedb_len > 0) {
				int i;
				const struct fb_videomode *mode;
				struct fb_videomode m;

				fb_destroy_modelist(&dvi->fbi->modelist);

				for (i = 0; i < dvi->fbi->monspecs.modedb_len; i++)
					/*FIXME now we do not support interlaced mode */
					if (!(dvi->fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
						fb_add_videomode(&dvi->fbi->monspecs.modedb[i],
								&dvi->fbi->modelist);

				fb_var_to_videomode(&m, &dvi->fbi->var);
				mode = fb_find_nearest_mode(&m,
						&dvi->fbi->modelist);

				fb_videomode_to_var(&dvi->fbi->var, mode);

				dvi->fbi->var.activate |= FB_ACTIVATE_FORCE;
				console_lock();
				dvi->fbi->flags |= FBINFO_MISC_USEREVENT;
				fb_set_var(dvi->fbi, &dvi->fbi->var);
				dvi->fbi->flags &= ~FBINFO_MISC_USEREVENT;
				console_unlock();
			}
		}
	} else {
		dvi->cable_plugin = 0;
		sprintf(event_string, "EVENT=plugout");
	}

	kobject_uevent_env(&dvi->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t mxc_dvi_detect_handler(int irq, void *data)
{
	struct mxc_dvi_data *dvi = data;
	schedule_delayed_work(&(dvi->det_work), msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static int dvi_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret = 0;
	struct mxc_dvi_data *dvi = mxc_dispdrv_getdata(disp);
	bool found = false;

	ret = ipu_di_to_crtc(&dvi->pdev->dev, dvi->ipu_id,
			     dvi->disp_id, &setting->crtc);

	if (ret < 0)
		return ret;

	setting->if_fmt = IPU_PIX_FMT_RGB24;
	dvi->fbi = setting->fbi;

	/* get video mode from edid */
	INIT_LIST_HEAD(&dvi->fbi->modelist);
	if (dvi_update(dvi)) {
		dvi->cable_plugin = 1;
		/* try to read edid */
		if (mxc_edid_read(dvi->client->adapter, dvi->client->addr,
				dvi->edid, &dvi->edid_cfg, dvi->fbi) < 0)
			dev_warn(&dvi->client->dev, "Can not read edid\n");
		else if (dvi->fbi->monspecs.modedb_len > 0) {
			int i;
			const struct fb_videomode *mode;
			struct fb_videomode m;

			for (i = 0; i < dvi->fbi->monspecs.modedb_len; i++) {
				/*FIXME now we do not support interlaced mode */
				if (!(dvi->fbi->monspecs.modedb[i].vmode
					& FB_VMODE_INTERLACED))
					fb_add_videomode(
						&dvi->fbi->monspecs.modedb[i],
						&dvi->fbi->modelist);
			}

			fb_find_mode(&dvi->fbi->var, dvi->fbi, setting->dft_mode_str,
					NULL, 0, NULL, setting->default_bpp);

			fb_var_to_videomode(&m, &dvi->fbi->var);
			mode = fb_find_nearest_mode(&m,
					&dvi->fbi->modelist);
			fb_videomode_to_var(&dvi->fbi->var, mode);
			found = 1;
		}
	} else
		dvi->cable_plugin = 0;

	if (!found) {
		ret = fb_find_mode(&dvi->fbi->var, dvi->fbi, setting->dft_mode_str,
				NULL, 0, NULL, setting->default_bpp);
		if (!ret)
			return -EINVAL;
	}

	/* cable detection */
	if (dvi->client->irq) {
		ret = request_irq(dvi->client->irq, mxc_dvi_detect_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"dvi_det", dvi);
		if (ret < 0) {
			dev_warn(&dvi->client->dev,
				"MXC dvi: cound not request det irq %d\n",
				dvi->client->irq);
			goto err;
		} else {
			INIT_DELAYED_WORK(&(dvi->det_work), det_worker);
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_fb_name);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for fb name\n");
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_cable_state);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for cable state\n");
			ret = device_create_file(&dvi->pdev->dev, &dev_attr_edid);
			if (ret < 0)
				dev_warn(&dvi->client->dev,
					"MXC dvi: cound not create sys node for edid\n");

			dev_set_drvdata(&dvi->pdev->dev, dvi);
		}
	}

err:
	return ret;
}

static void dvi_deinit(struct mxc_dispdrv_handle *disp)
{
	struct mxc_dvi_data *dvi = mxc_dispdrv_getdata(disp);
	free_irq(dvi->client->irq, dvi);
}

static struct mxc_dispdrv_driver dvi_drv = {
	.name 	= DISPDRV_DVI,
	.init 	= dvi_init,
	.deinit	= dvi_deinit,
};

static int mxc_dvi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_dvi_data *dvi;
	u32 ipu_id, disp_id, hp_gpio;
	int ret = 0;

	if (!dvi_i2c)
		return -EPROBE_DEFER;

	dvi = kzalloc(sizeof(struct mxc_dvi_data), GFP_KERNEL);
	if (!dvi)
		return -ENOMEM;

	ret = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (ret) {
		dev_dbg(&pdev->dev, "get of property ipu_id fail\n");
		goto error;
	}
	ret = of_property_read_u32(np, "disp_id", &disp_id);
	if (ret) {
		dev_dbg(&pdev->dev, "get of property disp_id fail\n");
		goto error;
	}
	ret = of_property_read_u32(np, "hp_gpio", &hp_gpio);
	if (ret) {
		dev_dbg(&pdev->dev, "get of property hp_gpio fail\n");
		goto error;
	}

	dvi->ipu_id = ipu_id;
	dvi->disp_id = disp_id;
	dvi->hp_gpio = hp_gpio;

	dvi->client = dvi_i2c;
	dvi->pdev = pdev;
	dvi->disp_dvi = mxc_dispdrv_register(&dvi_drv);
	mxc_dispdrv_setdata(dvi->disp_dvi, dvi);

	dvi->client->irq = gpio_to_irq(dvi->hp_gpio);

	g_dvi = dvi;
	platform_set_drvdata(pdev, dvi);
	printk("Registered MXC DVI as a platform device\n");

	return ret;
error:
	kfree(dvi);
	return ret;
}

static const struct of_device_id imx_dvi_dt_ids[] = {
	{ .compatible = "fsl,dvi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_dvi_dt_ids);

static int mxc_dvi_remove(struct platform_device *pdev)
{
	struct mxc_dvi_data *dvi = platform_get_drvdata(pdev);
	kfree(dvi);
	g_dvi = NULL;

	return 0;
}
static struct platform_driver mxc_dvi_driver = {
	.driver = {
		.name = "mxc_dvi",
		.of_match_table	= imx_dvi_dt_ids,
	},
	.probe = mxc_dvi_probe,
	.remove = mxc_dvi_remove,
};

static int __init mxc_dvi_init(void)
{
	return platform_driver_register(&mxc_dvi_driver);
}
module_init(mxc_dvi_init);

static void __exit mxc_dvi_exit(void)
{
	platform_driver_unregister(&mxc_dvi_driver);
}
module_exit(mxc_dvi_exit);

static int mxc_dvi_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	dvi_i2c = client;
	return 0;
}

static int mxc_dvi_i2c_remove(struct i2c_client *client)
{
	dvi_i2c = NULL;
	return 0;
}

static const struct of_device_id imx_dvi_i2c_match[] = {
	{ .compatible = "fsl,imx6-dvi-i2c", },
	{ /* sentinel */ }
};

static const struct i2c_device_id mxc_dvi_i2c_id[] = {
	{ "mxc_dvi_i2c", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mxc_dvi_i2c_id);

static struct i2c_driver mxc_dvi_i2c_driver = {
	.driver = {
		   .name = "mxc_dvi_i2c",
			.of_match_table	= imx_dvi_i2c_match,
		   },
	.probe = mxc_dvi_i2c_probe,
	.remove = mxc_dvi_i2c_remove,
	.id_table = mxc_dvi_i2c_id,
};

static int __init mxc_dvi_i2c_init(void)
{
	return i2c_add_driver(&mxc_dvi_i2c_driver);
}

static void __exit mxc_dvi_i2c_exit(void)
{
	i2c_del_driver(&mxc_dvi_i2c_driver);
}

module_init(mxc_dvi_i2c_init);
module_exit(mxc_dvi_i2c_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC DVI driver");
MODULE_LICENSE("GPL");
