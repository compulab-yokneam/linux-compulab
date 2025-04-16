/*
 * SN65DSI83 DSI-to-LVDS bridge IC driver
 *
 * Copyright (C) 2017 CompuLab Ltd.
 * Author: Valentin Raevsky <valentin@compulab.co.il>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include "mipi_dsi.h"
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define DRV_NAME "sn65dsi83"

#define SN65DSI83_SOFT_RESET_REG		0x09

#define SN65DSI83_CLOCK_REG			0x0A
#define SN65DSI83_LVDS_CLK_RANGE_MASK		0xE
#define SN65DSI83_LVDS_CLK_RANGE_SHIFT		1
#define SN65DSI83_HS_CLK_SRC_MASK		0x1
#define SN65DSI83_HS_CLK_SRC_SHIFT		0

#define SN65DSI83_CLOCK_ADJUST_REG		0x0B
#define SN65DSI83_DSI_CLK_DIV_MASK		0xF8
#define SN65DSI83_DSI_CLK_DIV_SHIFT		3
#define SN65DSI83_REFCLK_MULTIPLIER_MASK	0x3
#define SN65DSI83_REFCLK_MULTIPLIER_SHIFT	0

#define SN65DSI83_PLL_EN_REG			0x0D

#define SN65DSI83_DSI_LANES_REG			0x10
#define SN65DSI83_CHA_DSI_LANES_MASK		0x18
#define SN65DSI83_CHA_DSI_LANES_SHIFT		3
#define SN65DSI83_SOT_ERR_TOL_DIS_MASK		0x1
#define SN65DSI83_SOT_ERR_TOL_DIS_SHIFT		0

#define SN65DSI83_DSI_LANES_EQ_REG		0x11

#define SN65DSI83_CHA_DSI_CLK_RANGE_REG		0x12
#define SN65DSI83_CHA_DSI_CLK_RANGE_MASK	0xFF
#define SN65DSI83_CHA_DSI_CLK_RANGE_SHIFT	0xFF

#define SN65DSI83_VIDEO_FORMAT_REG		0x18
#define SN65DSI83_DE_NEG_POLARITY_SHIFT		7
#define SN65DSI83_HS_NEG_POLARITY_SHIFT		6
#define SN65DSI83_VS_NEG_POLARITY_SHIFT		5
#define SN65DSI83_LVDS_LINK_CFG_SHIFT		4
#define SN65DSI83_CHA_24BPP_MODE_SHIFT		3
#define SN65DSI83_CHA_24BPP_FMT1_SHIFT		1

#define SN65DSI83_LVDS_VOLATGE_REG		0x19

#define SN65DSI83_LVDS_PINS_REG			0x1A
#define SN65DSI83_CHA_REVERSE_LVDS_MASK		0x20
#define SN65DSI83_CHA_REVERSE_LVDS_SHIFT	5
#define SN65DSI83_CHA_LVDS_TERM_MASK		0x1
#define SN65DSI83_CHA_LVDS_TERM_SHIFT		0

#define SN65DSI83_LVDS_CM_ADJUST_REG		0x1B
#define SN65DSI83_CHA_HACTIVE_LOW_REG		0x20
#define SN65DSI83_CHA_HACTIVE_HIGH_REG		0x21
#define SN65DSI83_CHA_VACTIVE_LOW_REG		0x24
#define SN65DSI83_CHA_VACTIVE_HIGH_REG		0x25
#define SN65DSI83_CHA_SYNC_DEL_LOW_REG		0x28
#define SN65DSI83_CHA_SYNC_DEL_HIGH_REG		0x29
#define SN65DSI83_CHA_HSYNC_LEN_LOW_REG		0x2C
#define SN65DSI83_CHA_HSYNC_LEN_HIGH_REG	0x2D
#define SN65DSI83_CHA_VSYNC_LEN_LOW_REG		0x30
#define SN65DSI83_CHA_VSYNC_LEN_HIGH_REG	0x31
#define SN65DSI83_CHA_HBACK_PORCH_REG		0x34
#define SN65DSI83_CHA_VBACK_PORCH_REG		0x36
#define SN65DSI83_CHA_HFRONT_PORCH_REG		0x38
#define SN65DSI83_CHA_VFRONT_PORCH_REG		0x3A

#define SN65DSI83_TEST_PATTERN_REG		0x3C
#define SN65DSI83_CHA_TEST_PATTERN_SHIFT	4

#define SN65DSI83_CHA_ERR_REG			0xE5

static struct fb_videomode sn65dsi_modedb[] = {
	{
		/* CHIMEI N116B6-L02 datasheet values */
		.name		= "SN65DSI_default",
		.xres		= 1366,
		.yres		= 768,
		.pixclock	= 13256,
		.left_margin	= 98,
		.right_margin 	= 31,
		.upper_margin	= 22,
		.lower_margin	= 4,
		.hsync_len	= 65,
		.vsync_len	= 12,
		.sync		= FB_SYNC_OE_LOW_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num	= 2,
	.max_phy_clk	= 800,
	.dpi_fmt	= MIPI_RGB888,
};

struct panel_drv_data {
	struct i2c_client *client;
	struct fb_videomode *fb_vm;
	int pixclk_src;
	int mode_24bpp;
	int format_24bpp;
	struct gpio_desc *enable_gpio;
} sn65dsi_panel;

static void dump_fb_videomode(struct fb_videomode *m)
{
	pr_info("fb_videomode = %d %d %d %ul %d %d %d %d %d %d %d %d %d\n",
		m->refresh, m->xres, m->yres, m->pixclock, m->left_margin,
		m->right_margin, m->upper_margin, m->lower_margin,
		m->hsync_len, m->vsync_len, m->sync, m->vmode, m->flag);
}

static void dump_videomode(struct videomode *m)
{
	pr_info("videomode = %lu %d %d %d %d %d %d %d %d %d\n",
		m->pixelclock,
		m->hactive, m->hfront_porch, m->hback_porch, m->hsync_len,
		m->vactive, m->vfront_porch, m->vback_porch, m->vsync_len,
		m->flags);
}

static int sn65dsi_write(u8 reg, u8 val)
{
	struct i2c_client *client = sn65dsi_panel.client;
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret)
		dev_err(&client->dev, "failed to write at 0x%02x", reg);

	dev_dbg(&client->dev, "%s: write reg 0x%02x data 0x%02x", __func__, reg, val);

	return ret;
}

static int sn65dsi_read(u8 reg)
{
	struct i2c_client *client = sn65dsi_panel.client;
	int ret;

	dev_notice(&client->dev, "client 0x%p", client);
	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x", reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: read reg 0x%02x data 0x%02x", __func__, reg, ret);

	return ret;
}

static int sn65dsi_probe_of(void)
{
	struct device_node *np = sn65dsi_panel.client->dev.of_node;
	struct device dev = sn65dsi_panel.client->dev;
	struct device_node *port;
	struct device_node *endpoint;
	struct device_node *panel;
	struct display_timing of_timing;
	struct videomode vm;
	struct fb_videomode fb_vm;
	struct gpio_desc *gpio;
	int ret;

	gpio = devm_gpiod_get(&dev, "enable");
	if (IS_ERR(gpio)) {
		dev_err(&dev, "failed to parse enable gpio");
		return PTR_ERR(gpio);
	} else {
		gpiod_direction_output(gpio, 0);
		sn65dsi_panel.enable_gpio = gpio;
		dev_err(&dev, "enable gpio found");
	}

	dev_notice(&dev, "node %s", np->name);

	port = of_graph_get_port_by_id(np, 0);
	if (!port)
		return -EINVAL;

	of_node_put(port);

	endpoint = of_graph_get_next_endpoint(np, NULL);
	dev_notice(&dev, "endpoint %s", endpoint->name);

	panel = of_graph_get_remote_port_parent(endpoint);
	if (!panel) {
		dev_err(&dev, "failed to find panel");
		return -ENODEV;
	}
	of_node_put(endpoint);

	dev_notice(&dev, "panel %s", panel->name);

	memset(&vm, 0 ,sizeof(vm));
	memset(&fb_vm, 0 ,sizeof(fb_vm));
	ret = of_get_display_timing(panel, "panel-timing", &of_timing);
	if (!ret) {
		videomode_from_timing(&of_timing, &vm);
		dump_videomode(&vm);
		ret = fb_videomode_from_videomode(&vm, &fb_vm);
		if (ret)
			return ret;

		dump_fb_videomode(&fb_vm);

	}
	else
		dev_info(&dev, "use default video timing settings");

	if (of_find_property(panel, "lvds-24bpp-mode", NULL))
		sn65dsi_panel.mode_24bpp = 1;

	ret = of_property_read_u32(panel, "lvds-24bpp-format",
				&sn65dsi_panel.format_24bpp);

	if (ret || (sn65dsi_panel.format_24bpp != 1 && sn65dsi_panel.format_24bpp != 2))
		sn65dsi_panel.format_24bpp = 2;

	ret = of_property_read_u32(panel, "lvds-pixclk-src",
				   &sn65dsi_panel.pixclk_src);
	if (ret)
		sn65dsi_panel.pixclk_src = 0;

	of_node_put(panel);

	return 0;
}

static int sn65dsi_power_on(void)
{

	gpiod_set_value_cansleep(sn65dsi_panel.enable_gpio, 1);
	/* Wait for 1ms for the internal voltage regulator to stabilize */
	msleep(1);

	return 0;
}

static void sn65dsi_power_off(void)
{
	gpiod_set_value_cansleep(sn65dsi_panel.enable_gpio, 0);
	/*
	 * The EN pin must be held low for at least 10 ms
	 * before being asserted high
	 */
	msleep(10);
}

static int sn65dsi_start_stream(void)
{
	struct device dev = sn65dsi_panel.client->dev;
	int regval;

	/* Set the PLL_EN bit (CSR 0x0D.0) */
	sn65dsi_write(SN65DSI83_PLL_EN_REG, 0x1);
	/* Wait for the PLL_LOCK bit to be set (CSR 0x0A.7) */
	msleep(200);

	/* Perform SW reset to apply changes */
	sn65dsi_write(SN65DSI83_SOFT_RESET_REG, 0x01);

	/* Read CHA Error register */
	regval = sn65dsi_read(SN65DSI83_CHA_ERR_REG);
	dev_info(&dev, "CHA (0x%02x) = 0x%02x",
		 SN65DSI83_CHA_ERR_REG, regval);

	return 0;
}

static void sn65dsi_stop_stream(void)
{
	/* Clear the PLL_EN bit (CSR 0x0D.0) */
	sn65dsi_write(SN65DSI83_PLL_EN_REG, 0x00);
}

static int sn65dsi_configure(void)
{
	sn65dsi_write(0x09,0x00);
	sn65dsi_write(0x0A,0x05);
	sn65dsi_write(0x0B,0x28);
	sn65dsi_write(0x0D,0x00);
	sn65dsi_write(0x10,0x36);
	sn65dsi_write(0x11,0x00);
	sn65dsi_write(0x12,0x5c);
	sn65dsi_write(0x13,0x00);
	sn65dsi_write(0x18,0x72);
	sn65dsi_write(0x19,0x00);
	sn65dsi_write(0x1A,0x03);
	sn65dsi_write(0x1B,0x00);
	sn65dsi_write(0x20,0x56);
	sn65dsi_write(0x21,0x05);
	sn65dsi_write(0x22,0x00);
	sn65dsi_write(0x23,0x00);
	sn65dsi_write(0x24,0x00);
	sn65dsi_write(0x25,0x00);
	sn65dsi_write(0x26,0x00);
	sn65dsi_write(0x27,0x00);
	sn65dsi_write(0x28,0x21);
	sn65dsi_write(0x29,0x00);
	sn65dsi_write(0x2A,0x00);
	sn65dsi_write(0x2B,0x00);
	sn65dsi_write(0x2C,0x41);
	sn65dsi_write(0x2D,0x00);
	sn65dsi_write(0x2E,0x00);
	sn65dsi_write(0x2F,0x00);
	sn65dsi_write(0x30,0x0c);
	sn65dsi_write(0x31,0x00);
	sn65dsi_write(0x32,0x00);
	sn65dsi_write(0x33,0x00);
	sn65dsi_write(0x34,0x62);
	sn65dsi_write(0x35,0x00);
	sn65dsi_write(0x36,0x00);
	sn65dsi_write(0x37,0x00);
	sn65dsi_write(0x38,0x00);
	sn65dsi_write(0x39,0x00);
	sn65dsi_write(0x3A,0x00);
	sn65dsi_write(0x3B,0x00);
	sn65dsi_write(0x3C,0x00);
	sn65dsi_write(0x3D,0x00);
	sn65dsi_write(0x3E,0x00);
	return 0;
}

int sn65dsi83_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;
	dev_notice(&dev, "MIPI DSI LCD setup.\n");

	sn65dsi_power_on();
	sn65dsi_configure();

	return 0;
}

int sn65dsi83_lcd_start(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;
	dev_notice(&dev, "MIPI DSI LCD start.\n");

	sn65dsi_start_stream();

	return 0;
}

int sn65dsi83_lcd_stop(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;
	dev_notice(&dev, "MIPI DSI LCD stop.\n");

	sn65dsi_stop_stream();
	sn65dsi_power_off();

	return 0;
}

void sn65dsi83_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &sn65dsi_modedb[0];
	*size = ARRAY_SIZE(sn65dsi_modedb);
	*data = &lcd_config;
}

static int sn65dsi_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;

	dev_notice(&client->dev, "probe, client 0x%p\n", client);
	memset(&sn65dsi_panel, 0, sizeof(sn65dsi_panel));

	sn65dsi_panel.client = client;
	sn65dsi_panel.fb_vm = &sn65dsi_modedb[0];

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	ret = sn65dsi_probe_of();
	if (ret) {
		dev_err(&client->dev, "failed to parse DT data");
		return -1;
	}

	sn65dsi_power_off();
	sn65dsi_power_on();

	/* Soft Reset reg value at power on should be 0x00 */
	ret = sn65dsi_read(SN65DSI83_SOFT_RESET_REG);
	if (ret != 0x00)
		return -ENODEV;

	sn65dsi_power_off();

	return ret;
}

static int __exit sn65dsi_remove(struct i2c_client *client)
{
	sn65dsi_stop_stream();
	sn65dsi_power_off();
	return 0;
}

static const struct i2c_device_id sn65dsi_id[] = {
	{ DRV_NAME, 0},
	{ },
};

MODULE_DEVICE_TABLE(i2c, sn65dsi_id);

static const struct of_device_id sn65dsi_of_match[] = {
	{ .compatible = "ti,sn65dsi83", },
	{ }
};

MODULE_DEVICE_TABLE(of, sn65dsi_of_match);

static struct i2c_driver sn65dsi_driver = {
	.probe	= sn65dsi_probe,
	.remove	= __exit_p(sn65dsi_remove),
	.id_table = sn65dsi_id,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sn65dsi_of_match,
	},
};

module_i2c_driver(sn65dsi_driver);

MODULE_AUTHOR("Valentin Raevsky <valentin@compulab.co.il>");
MODULE_DESCRIPTION("SN65DSI83 DSI-to-LVDS bridge IC driver");
MODULE_LICENSE("GPL");
