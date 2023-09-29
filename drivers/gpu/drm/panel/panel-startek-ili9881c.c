// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019, CompuLab LTD.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <video/of_display_timing.h>

#include <video/mipi_display.h>
#include <video/display_timing.h>
#include <video/videomode.h>
#include <linux/media-bus-format.h>

struct ili9881c {
    struct drm_panel	panel;
    struct mipi_dsi_device	*dsi;

    struct backlight_device *backlight;
    struct regulator	*power;
    int rst_gpio;

    bool prepared;
    bool enabled;
};

enum ili9881c_op {
    ILI9881C_SWITCH_PAGE,
    ILI9881C_COMMAND,
};

struct ili9881c_instr {
    enum ili9881c_op	op;

    union arg {
        struct cmd {
            u8	cmd;
            u8	data;
        } cmd;
        u8	page;
    } arg;
};

#define ILI9881C_SWITCH_PAGE_INSTR(_page)	\
    {					\
        .op = ILI9881C_SWITCH_PAGE,	\
        .arg = {			\
            .page = (_page),	\
        },				\
    }

#define ILI9881C_COMMAND_INSTR(_cmd, _data)		\
    {						\
        .op = ILI9881C_COMMAND,		\
        .arg = {				\
            .cmd = {			\
                .cmd = (_cmd),		\
                .data = (_data),	\
            },				\
        },					\
    }

static const struct ili9881c_instr ili9881c_init[] = {
    ILI9881C_SWITCH_PAGE_INSTR(3),
    ILI9881C_COMMAND_INSTR(0x01, 0x00),
    ILI9881C_COMMAND_INSTR(0x02, 0x00),
    ILI9881C_COMMAND_INSTR(0x03, 0x72),
    ILI9881C_COMMAND_INSTR(0x04, 0x00),
    ILI9881C_COMMAND_INSTR(0x05, 0x00),
    ILI9881C_COMMAND_INSTR(0x06, 0x09),
    ILI9881C_COMMAND_INSTR(0x07, 0x00),
    ILI9881C_COMMAND_INSTR(0x08, 0x00),
    ILI9881C_COMMAND_INSTR(0x09, 0x01),
    ILI9881C_COMMAND_INSTR(0x0a, 0x00),
    ILI9881C_COMMAND_INSTR(0x0b, 0x00),
    ILI9881C_COMMAND_INSTR(0x0c, 0x01),
    ILI9881C_COMMAND_INSTR(0x0d, 0x00),
    ILI9881C_COMMAND_INSTR(0x0e, 0x00),
    ILI9881C_COMMAND_INSTR(0x0f, 0x00),
    ILI9881C_COMMAND_INSTR(0x10, 0x00),
    ILI9881C_COMMAND_INSTR(0x11, 0x00),
    ILI9881C_COMMAND_INSTR(0x12, 0x00),
    ILI9881C_COMMAND_INSTR(0x13, 0x00),
    ILI9881C_COMMAND_INSTR(0x14, 0x00),
    ILI9881C_COMMAND_INSTR(0x15, 0x00),
    ILI9881C_COMMAND_INSTR(0x16, 0x00),
    ILI9881C_COMMAND_INSTR(0x17, 0x00),
    ILI9881C_COMMAND_INSTR(0x18, 0x00),
    ILI9881C_COMMAND_INSTR(0x19, 0x00),
    ILI9881C_COMMAND_INSTR(0x1a, 0x00),
    ILI9881C_COMMAND_INSTR(0x1b, 0x00),
    ILI9881C_COMMAND_INSTR(0x1c, 0x00),
    ILI9881C_COMMAND_INSTR(0x1d, 0x00),
    ILI9881C_COMMAND_INSTR(0x1e, 0x40),
    ILI9881C_COMMAND_INSTR(0x1f, 0x80),
    ILI9881C_COMMAND_INSTR(0x20, 0x05),
    ILI9881C_COMMAND_INSTR(0x20, 0x05),
    ILI9881C_COMMAND_INSTR(0x21, 0x02),
    ILI9881C_COMMAND_INSTR(0x22, 0x00),
    ILI9881C_COMMAND_INSTR(0x23, 0x00),
    ILI9881C_COMMAND_INSTR(0x24, 0x00),
    ILI9881C_COMMAND_INSTR(0x25, 0x00),
    ILI9881C_COMMAND_INSTR(0x26, 0x00),
    ILI9881C_COMMAND_INSTR(0x27, 0x00),
    ILI9881C_COMMAND_INSTR(0x28, 0x33),
    ILI9881C_COMMAND_INSTR(0x29, 0x02),
    ILI9881C_COMMAND_INSTR(0x2a, 0x00),
    ILI9881C_COMMAND_INSTR(0x2b, 0x00),
    ILI9881C_COMMAND_INSTR(0x2c, 0x00),
    ILI9881C_COMMAND_INSTR(0x2d, 0x00),
    ILI9881C_COMMAND_INSTR(0x2e, 0x00),
    ILI9881C_COMMAND_INSTR(0x2f, 0x00),
    ILI9881C_COMMAND_INSTR(0x30, 0x00),
    ILI9881C_COMMAND_INSTR(0x31, 0x00),
    ILI9881C_COMMAND_INSTR(0x32, 0x00),
    ILI9881C_COMMAND_INSTR(0x32, 0x00),
    ILI9881C_COMMAND_INSTR(0x33, 0x00),
    ILI9881C_COMMAND_INSTR(0x34, 0x04),
    ILI9881C_COMMAND_INSTR(0x35, 0x00),
    ILI9881C_COMMAND_INSTR(0x36, 0x00),
    ILI9881C_COMMAND_INSTR(0x37, 0x00),
    ILI9881C_COMMAND_INSTR(0x38, 0x3C),
    ILI9881C_COMMAND_INSTR(0x39, 0x00),
    ILI9881C_COMMAND_INSTR(0x3a, 0x40),
    ILI9881C_COMMAND_INSTR(0x3b, 0x40),
    ILI9881C_COMMAND_INSTR(0x3c, 0x00),
    ILI9881C_COMMAND_INSTR(0x3d, 0x00),
    ILI9881C_COMMAND_INSTR(0x3e, 0x00),
    ILI9881C_COMMAND_INSTR(0x3f, 0x00),
    ILI9881C_COMMAND_INSTR(0x40, 0x00),
    ILI9881C_COMMAND_INSTR(0x41, 0x00),
    ILI9881C_COMMAND_INSTR(0x42, 0x00),
    ILI9881C_COMMAND_INSTR(0x43, 0x00),
    ILI9881C_COMMAND_INSTR(0x44, 0x00),
    ILI9881C_COMMAND_INSTR(0x50, 0x01),
    ILI9881C_COMMAND_INSTR(0x51, 0x23),
    ILI9881C_COMMAND_INSTR(0x52, 0x45),
    ILI9881C_COMMAND_INSTR(0x53, 0x67),
    ILI9881C_COMMAND_INSTR(0x54, 0x89),
    ILI9881C_COMMAND_INSTR(0x55, 0xab),
    ILI9881C_COMMAND_INSTR(0x56, 0x01),
    ILI9881C_COMMAND_INSTR(0x57, 0x23),
    ILI9881C_COMMAND_INSTR(0x58, 0x45),
    ILI9881C_COMMAND_INSTR(0x59, 0x67),
    ILI9881C_COMMAND_INSTR(0x5a, 0x89),
    ILI9881C_COMMAND_INSTR(0x5b, 0xab),
    ILI9881C_COMMAND_INSTR(0x5c, 0xcd),
    ILI9881C_COMMAND_INSTR(0x5d, 0xef),
    ILI9881C_COMMAND_INSTR(0x5e, 0x11),
    ILI9881C_COMMAND_INSTR(0x5f, 0x01),
    ILI9881C_COMMAND_INSTR(0x60, 0x00),
    ILI9881C_COMMAND_INSTR(0x61, 0x15),
    ILI9881C_COMMAND_INSTR(0x62, 0x14),
    ILI9881C_COMMAND_INSTR(0x63, 0x0E),
    ILI9881C_COMMAND_INSTR(0x64, 0x0F),
    ILI9881C_COMMAND_INSTR(0x65, 0x0C),
    ILI9881C_COMMAND_INSTR(0x66, 0x0D),
    ILI9881C_COMMAND_INSTR(0x67, 0x06),
    ILI9881C_COMMAND_INSTR(0x68, 0x02),
    ILI9881C_COMMAND_INSTR(0x69, 0x07),
    ILI9881C_COMMAND_INSTR(0x6a, 0x02),
    ILI9881C_COMMAND_INSTR(0x6b, 0x02),
    ILI9881C_COMMAND_INSTR(0x6c, 0x02),
    ILI9881C_COMMAND_INSTR(0x6d, 0x02),
    ILI9881C_COMMAND_INSTR(0x6e, 0x02),
    ILI9881C_COMMAND_INSTR(0x6f, 0x02),
    ILI9881C_COMMAND_INSTR(0x70, 0x02),
    ILI9881C_COMMAND_INSTR(0x71, 0x02),
    ILI9881C_COMMAND_INSTR(0x72, 0x02),
    ILI9881C_COMMAND_INSTR(0x73, 0x02),
    ILI9881C_COMMAND_INSTR(0x74, 0x02),
    ILI9881C_COMMAND_INSTR(0x75, 0x01),
    ILI9881C_COMMAND_INSTR(0x76, 0x00),
    ILI9881C_COMMAND_INSTR(0x77, 0x14),
    ILI9881C_COMMAND_INSTR(0x78, 0x15),
    ILI9881C_COMMAND_INSTR(0x79, 0x0E),
    ILI9881C_COMMAND_INSTR(0x7a, 0x0F),
    ILI9881C_COMMAND_INSTR(0x7b, 0x0C),
    ILI9881C_COMMAND_INSTR(0x7c, 0x0D),
    ILI9881C_COMMAND_INSTR(0x7d, 0x06),
    ILI9881C_COMMAND_INSTR(0x7e, 0x02),
    ILI9881C_COMMAND_INSTR(0x7f, 0x07),
    ILI9881C_COMMAND_INSTR(0x80, 0x02),
    ILI9881C_COMMAND_INSTR(0x81, 0x02),
    ILI9881C_COMMAND_INSTR(0x83, 0x02),
    ILI9881C_COMMAND_INSTR(0x84, 0x02),
    ILI9881C_COMMAND_INSTR(0x85, 0x02),
    ILI9881C_COMMAND_INSTR(0x86, 0x02),
    ILI9881C_COMMAND_INSTR(0x87, 0x02),
    ILI9881C_COMMAND_INSTR(0x88, 0x02),
    ILI9881C_COMMAND_INSTR(0x89, 0x02),
    ILI9881C_COMMAND_INSTR(0x8A, 0x02),
    ILI9881C_SWITCH_PAGE_INSTR(0x4),
    ILI9881C_COMMAND_INSTR(0x6C, 0x15),
    ILI9881C_COMMAND_INSTR(0x6E, 0x2A),
    ILI9881C_COMMAND_INSTR(0x6F, 0x33),
    ILI9881C_COMMAND_INSTR(0x3A, 0x94),
    ILI9881C_COMMAND_INSTR(0x8D, 0x15),
    ILI9881C_COMMAND_INSTR(0x87, 0xBA),
    ILI9881C_COMMAND_INSTR(0x26, 0x76),
    ILI9881C_COMMAND_INSTR(0xB2, 0xD1),
    ILI9881C_COMMAND_INSTR(0xB5, 0x06),
    ILI9881C_SWITCH_PAGE_INSTR(0x1),
    ILI9881C_COMMAND_INSTR(0x22, 0x0A),
    ILI9881C_COMMAND_INSTR(0x31, 0x00),
    ILI9881C_COMMAND_INSTR(0x53, 0x90),
    ILI9881C_COMMAND_INSTR(0x55, 0xA2),
    ILI9881C_COMMAND_INSTR(0x50, 0xB7),
    ILI9881C_COMMAND_INSTR(0x51, 0xB7),
    ILI9881C_COMMAND_INSTR(0x60, 0x22),
    ILI9881C_COMMAND_INSTR(0x61, 0x00),
    ILI9881C_COMMAND_INSTR(0x62, 0x19),
    ILI9881C_COMMAND_INSTR(0x63, 0x10),
    ILI9881C_COMMAND_INSTR(0xA0, 0x08),
    ILI9881C_COMMAND_INSTR(0xA1, 0x1A),
    ILI9881C_COMMAND_INSTR(0xA2, 0x27),
    ILI9881C_COMMAND_INSTR(0xA3, 0x15),
    ILI9881C_COMMAND_INSTR(0xA4, 0x17),
    ILI9881C_COMMAND_INSTR(0xA5, 0x2A),
    ILI9881C_COMMAND_INSTR(0xA6, 0x1E),
    ILI9881C_COMMAND_INSTR(0xA7, 0x1F),
    ILI9881C_COMMAND_INSTR(0xA8, 0x8B),
    ILI9881C_COMMAND_INSTR(0xA9, 0x1B),
    ILI9881C_COMMAND_INSTR(0xAA, 0x27),
    ILI9881C_COMMAND_INSTR(0xAB, 0x78),
    ILI9881C_COMMAND_INSTR(0xAC, 0x18),
    ILI9881C_COMMAND_INSTR(0xAD, 0x18),
    ILI9881C_COMMAND_INSTR(0xAE, 0x4C),
    ILI9881C_COMMAND_INSTR(0xAF, 0x21),
    ILI9881C_COMMAND_INSTR(0xB0, 0x27),
    ILI9881C_COMMAND_INSTR(0xB1, 0x54),
    ILI9881C_COMMAND_INSTR(0xB2, 0x67),
    ILI9881C_COMMAND_INSTR(0xB3, 0x39),
    ILI9881C_COMMAND_INSTR(0xC0, 0x08),
    ILI9881C_COMMAND_INSTR(0xC1, 0x1A),
    ILI9881C_COMMAND_INSTR(0xC2, 0x27),
    ILI9881C_COMMAND_INSTR(0xC3, 0x15),
    ILI9881C_COMMAND_INSTR(0xC4, 0x17),
    ILI9881C_COMMAND_INSTR(0xC5, 0x2A),
    ILI9881C_COMMAND_INSTR(0xC6, 0x1E),
    ILI9881C_COMMAND_INSTR(0xC7, 0x1F),
    ILI9881C_COMMAND_INSTR(0xC8, 0x8B),
    ILI9881C_COMMAND_INSTR(0xC9, 0x1B),
    ILI9881C_COMMAND_INSTR(0xCA, 0x27),
    ILI9881C_COMMAND_INSTR(0xCB, 0x78),
    ILI9881C_COMMAND_INSTR(0xCC, 0x18),
    ILI9881C_COMMAND_INSTR(0xCD, 0x18),
    ILI9881C_COMMAND_INSTR(0xCE, 0x4C),
    ILI9881C_COMMAND_INSTR(0xCF, 0x21),
    ILI9881C_COMMAND_INSTR(0xD0, 0x27),
    ILI9881C_COMMAND_INSTR(0xD1, 0x54),
    ILI9881C_COMMAND_INSTR(0xD2, 0x67),
    ILI9881C_COMMAND_INSTR(0xD3, 0x39),
    ILI9881C_SWITCH_PAGE_INSTR(0),
    ILI9881C_COMMAND_INSTR(0x35, 0x00),
    ILI9881C_COMMAND_INSTR(0x3A, 0x7),
};

static const u32 ili_bus_formats[] = {
        MEDIA_BUS_FMT_RGB888_1X24,
        MEDIA_BUS_FMT_RGB666_1X18,
        MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 ili_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

static inline struct ili9881c *panel_to_ili9881c(struct drm_panel *panel)
{
    return container_of(panel, struct ili9881c, panel);
}

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
    switch (format) {
    case MIPI_DSI_FMT_RGB565:
        return 0x55;
    case MIPI_DSI_FMT_RGB666:
    case MIPI_DSI_FMT_RGB666_PACKED:
        return 0x66;
    case MIPI_DSI_FMT_RGB888:
        return 0x77;
    default:
        return 0x77;
    }
};

static int ili9881c_switch_page(struct ili9881c *ctx, u8 page)
{
    u8 buf[4] = { 0xff, 0x98, 0x81, page };
    int ret;

    ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
    if (ret < 0) {
        dev_err(&ctx->dsi->dev,"%s:%d failed %d\n", __func__, page, ret);
        return ret;
    }

    return 0;
}

static int ili9881c_send_cmd_data(struct ili9881c *ctx, u8 cmd, u8 data)
{
	u8 buf[2] = { cmd, data };
    int ret;

    ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, sizeof(buf));
    if (ret < 0) {
        dev_err(&ctx->dsi->dev,"%s:%d failed %d\n", __func__,cmd, ret);
        return ret;
    }
    return 0;
}

static int ili9881c_enable(struct drm_panel *panel)
{
    unsigned int i;
    int ret;
    struct ili9881c *ctx = panel_to_ili9881c(panel);
    struct mipi_dsi_device *dsi = ctx->dsi;
    int color_format = color_format_from_dsi_format(dsi->format);

    if (ctx->enabled)
        return 0;

    if (!ctx->prepared) {
        dev_err(&ctx->dsi->dev, "Panel not prepared!\n");
        return -EPERM;
    }

    dsi->mode_flags |= MIPI_DSI_MODE_LPM;

    for (i = 0; i < ARRAY_SIZE(ili9881c_init); i++) {
        const struct ili9881c_instr *instr = &ili9881c_init[i];

        if (instr->op == ILI9881C_SWITCH_PAGE)
            ret = ili9881c_switch_page(ctx, instr->arg.page);
        else if (instr->op == ILI9881C_COMMAND)
            ret = ili9881c_send_cmd_data(ctx, instr->arg.cmd.cmd,
                              instr->arg.cmd.data);

        if (ret) {
            dev_err(&ctx->dsi->dev,"%s %d Failed command  # = [ %d ]; ret = [ %d ]\n",__func__,__LINE__,i,ret);
            return ret;
        }
    }

   /* The default value is 4-lane mode
    * Issue if 2-lane required
    */
    if (ctx->dsi->lanes == 2) {
        ret = ili9881c_switch_page(ctx, 1);
        if (ret)
            return ret;

        ret = ili9881c_send_cmd_data(ctx, 0xB7, 0x03);
        if (ret)
            return ret;

        ret = ili9881c_switch_page(ctx, 0);
        if (ret)
            return ret;
    }

    ret = mipi_dsi_dcs_set_tear_on(ctx->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
    if (ret)
        return ret;

    /* Set tear scanline */
    ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
    if (ret < 0) {
        dev_err(&ctx->dsi->dev, "Failed to set tear scanline (%d)\n", ret);
        return ret;
    }

    /* Set pixel format */
    ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
    if (ret < 0) {
        dev_err(&ctx->dsi->dev, "Failed to set pixel format (%d)\n", ret);
        return ret;
    }

    ret = mipi_dsi_dcs_set_display_brightness(dsi, 0xffff);
    if (ret < 0) {
        dev_err(&ctx->dsi->dev, "Failed to set display brightness (%d)\n", ret);
        return ret;
    }

    ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
    if (ret) {
        dev_err(&ctx->dsi->dev, "Failed to exit sleep mode (%d)\n", ret);
        return ret;
    }
    msleep(120);

    ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
    if (ret) {
        dev_err(&ctx->dsi->dev, "Failed to set display on (%d)\n", ret);
        return ret;
    }
    msleep(20);

    ctx->enabled = true;

    dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	dev_notice(&ctx->dsi->dev,
            "%s: Set %d-lane mode ; Color %d\n",__func__,
            ctx->dsi->lanes,color_format);

    return 0;
}

static int ili9881c_prepare(struct drm_panel *panel)
{
    struct ili9881c *ctx = panel_to_ili9881c(panel);
    int ret;

    if (ctx->prepared)
        return 0;

    /* Power the panel */
    ret = regulator_enable(ctx->power);
    if (ret)
        return ret;

    /* And reset it */
    gpio_set_value_cansleep(ctx->rst_gpio, 0);
    msleep(20);

    gpio_set_value_cansleep(ctx->rst_gpio, 1);
    msleep(20);

    ctx->prepared = true;

    return 0;
}

static int ili9881c_disable(struct drm_panel *panel)
{
    struct ili9881c *ctx = panel_to_ili9881c(panel);
    int ret;

    if (!ctx->enabled)
        return 0;

    /* backlight_disable(ctx->backlight); */
    ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
    if (ret < 0) {
        dev_err(&ctx->dsi->dev, "Failed to enter sleep mode (%d)\n", ret);
        return ret;
    }

    ctx->enabled = false;

    return 0;
}

static int ili9881c_unprepare(struct drm_panel *panel)
{
    struct ili9881c *ctx = panel_to_ili9881c(panel);

    if (!ctx->prepared)
        return 0;

    if (ctx->enabled) {
        dev_err(&ctx->dsi->dev, "Panel still enabled!\n");
        return -EPERM;
    }

    mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
    regulator_disable(ctx->power);
    gpio_set_value_cansleep(ctx->rst_gpio, 0);

    ctx->prepared = false;

    return 0;
}

static struct display_timing banana_timing = {
    .pixelclock = { 62000000, 62000000, 62000000 },
    .hactive = { 720, 720, 720 },
    .hfront_porch = { 20, 20, 20 },
    .hback_porch = { 10, 10, 10 },
    .hsync_len = { 10, 10, 10 },
    .vactive = { 1280, 1280, 1280 },
    .vfront_porch = { 10, 10, 10 },
    .vback_porch = { 30, 30, 30 },
    .vsync_len = { 20, 20, 20 },
    .flags = DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_DE_LOW,
};

static struct drm_display_mode default_mode = {
	.width_mm = 63,
	.height_mm = 115,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static int ili9881c_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = ili_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 ili_bus_formats,
					 ARRAY_SIZE(ili_bus_formats));
	return 1;
}

static const struct drm_panel_funcs ili9881c_funcs = {
    .prepare	= ili9881c_prepare,
    .unprepare	= ili9881c_unprepare,
    .enable	= ili9881c_enable,
    .disable	= ili9881c_disable,
    .get_modes	= ili9881c_get_modes,
};

static void ili9881c_dt_to_drm(struct display_timing *dt, struct drm_display_mode *mode)
{
    struct videomode vm;
    videomode_from_timing(dt, &vm);
    drm_display_mode_from_videomode(&vm, mode);
}

static int ili9881c_dsi_probe(struct mipi_dsi_device *dsi)
{
    struct device_node *np;
    struct ili9881c *ctx;
	int lanes = 4;

    ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;
    mipi_dsi_set_drvdata(dsi, ctx);
    ctx->dsi = dsi;

    drm_panel_init(&ctx->panel, &dsi->dev, &ili9881c_funcs,
		       DRM_MODE_CONNECTOR_DSI);

    ctx->power = devm_regulator_get(&dsi->dev, "power");
    if (IS_ERR(ctx->power)) {
        dev_err(&dsi->dev, "Couldn't get our power regulator\n");
        return PTR_ERR(ctx->power);
    }

    np = of_parse_phandle(dsi->dev.of_node, "reset-gpio", 0);
    if (np) {
        ctx->rst_gpio = of_get_named_gpio(dsi->dev.of_node, "reset-gpio", 0);
        if (!gpio_is_valid(ctx->rst_gpio)) {
            dev_notice(&dsi->dev, "Couldn't get panel reset pin available\n");
            return -EPROBE_DEFER;
        }
    }

    np = of_parse_phandle(dsi->dev.of_node, "backlight", 0);
    if (np) {
        ctx->backlight = of_find_backlight_by_node(np);
        of_node_put(np);

        if (!ctx->backlight)
            return -EPROBE_DEFER;
    }

    if (of_get_display_timing(dsi->dev.of_node, "panel-timing", &banana_timing))
        dev_notice(&dsi->dev, "Couldn't get panel timing from dtb, use driver settings\n");
    else
        dev_notice(&dsi->dev, "Use panel timing from dtb\n");

    of_property_read_u32(dsi->dev.of_node, "dsi-lanes", &lanes);
    drm_panel_add(&ctx->panel);

    dsi->mode_flags = MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
    dsi->mode_flags |= MIPI_DSI_MODE_VIDEO;
    dsi->format = MIPI_DSI_FMT_RGB888;
    dsi->lanes = lanes;

    ili9881c_dt_to_drm(&banana_timing,&default_mode);

    return mipi_dsi_attach(dsi);
}

static void ili9881c_dsi_remove(struct mipi_dsi_device *dsi)
{
    struct ili9881c *ctx = mipi_dsi_get_drvdata(dsi);

    mipi_dsi_detach(dsi);
    drm_panel_remove(&ctx->panel);

    if (ctx->backlight)
        put_device(&ctx->backlight->dev);

    return;
}

static const struct of_device_id ili9881c_of_match[] = {
    { .compatible = "startek,kd050hdfia020" },
    { }
};
MODULE_DEVICE_TABLE(of, ili9881c_of_match);

static struct mipi_dsi_driver ili9881c_dsi_driver = {
    .probe		= ili9881c_dsi_probe,
    .remove		= ili9881c_dsi_remove,
    .driver = {
        .name		= "ili9881c-dsi",
        .of_match_table	= ili9881c_of_match,
    },
};
module_mipi_dsi_driver(ili9881c_dsi_driver);

MODULE_AUTHOR("<CompuLab LTD> compulab@compulab.com>");
MODULE_DESCRIPTION("Startek Panel Driver with Ilitek ILI9881C Controller");
MODULE_LICENSE("GPL v2");
