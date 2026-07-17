// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for watchdog device controlled through GPIO-line
 *
 * Author: 2013, Alexander Shiyan <shc_work@mail.ru>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/watchdog.h>
#include <linux/pm.h>

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define SOFT_TIMEOUT_MIN	1
#define SOFT_TIMEOUT_DEF	60

#define WDD_EXT_DEBUG 1
#ifdef WDD_EXT_DEBUG
#define STAMPX(X) printk("wdt-gpio: %s [ %d ] ( 0x%lx ) \n",__func__,__LINE__,(X));
#else
#define STAMPX(X)
#endif

enum {
	HW_ALGO_TOGGLE,
	HW_ALGO_LEVEL,
};

struct gpio_wdt_priv {
	struct gpio_desc	*gpiod;
	bool			state;
	bool			always_running;
	unsigned int		hw_algo;
	struct watchdog_device	wdd;
	/* extra stuff */
	struct gpio_desc	*gpiod_enable;
	bool			e_state;
};

static void gpio_wdt_enable(struct gpio_wdt_priv *priv)
{
	STAMPX(priv->wdd.status);
	gpiod_set_value_cansleep(priv->gpiod_enable, 1);
	priv->e_state = 1;
	gpiod_direction_output(priv->gpiod_enable, priv->e_state);
}

static void gpio_wdt_disable(struct gpio_wdt_priv *priv)
{
	STAMPX(priv->wdd.status);
	/* Eternal ping */
	gpiod_set_value_cansleep(priv->gpiod, 1);

	/* Put GPIO back to tristate */
	if (priv->hw_algo == HW_ALGO_TOGGLE)
		gpiod_direction_input(priv->gpiod);

	gpiod_set_value_cansleep(priv->gpiod_enable, 0);
	priv->e_state = 0;
	gpiod_direction_output(priv->gpiod_enable, priv->e_state);
}

static int gpio_wdt_ping(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	switch (priv->hw_algo) {
	case HW_ALGO_TOGGLE:
		/* Toggle output pin */
		priv->state = !priv->state;
		gpiod_set_value_cansleep(priv->gpiod, priv->state);
		break;
	case HW_ALGO_LEVEL:
		/* Pulse */
		gpiod_set_value_cansleep(priv->gpiod, 1);
		udelay(1);
		gpiod_set_value_cansleep(priv->gpiod, 0);
		break;
	}
	return 0;
}

static int gpio_wdt_start(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	gpio_wdt_enable(priv);

	priv->state = 0;
	gpiod_direction_output(priv->gpiod, priv->state);

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return gpio_wdt_ping(wdd);
}

static int gpio_wdt_stop(struct watchdog_device *wdd)
{
	struct gpio_wdt_priv *priv = watchdog_get_drvdata(wdd);

	if (!priv->always_running) {
		gpio_wdt_disable(priv);
	} else {
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	}

	return 0;
}

static const struct watchdog_info gpio_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
			  WDIOF_SETTIMEOUT,
	.identity	= "GPIO Watchdog",
};

static const struct watchdog_ops gpio_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= gpio_wdt_start,
	.stop		= gpio_wdt_stop,
	.ping		= gpio_wdt_ping,
};

static int gpio_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_wdt_priv *priv;
	enum gpiod_flags gflags;
	unsigned int hw_margin;
	const char *algo;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	ret = device_property_read_string(dev, "hw_algo", &algo);
	if (ret)
		return ret;
	if (!strcmp(algo, "toggle")) {
		priv->hw_algo = HW_ALGO_TOGGLE;
		gflags = GPIOD_IN;
	} else if (!strcmp(algo, "level")) {
		priv->hw_algo = HW_ALGO_LEVEL;
		gflags = GPIOD_OUT_LOW;
	} else {
		return -EINVAL;
	}

	priv->gpiod = devm_gpiod_get(dev, "ping", gflags);
	if (IS_ERR(priv->gpiod))
		return PTR_ERR(priv->gpiod);

	priv->gpiod_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_enable))
		return PTR_ERR(priv->gpiod_enable);

	ret = device_property_read_u32(dev, "hw_margin_ms", &hw_margin);
	if (ret)
		return ret;
	/* Disallow values lower than 2 and higher than 65535 ms */
	if (hw_margin < 2 || hw_margin > 65535)
		return -EINVAL;

	priv->always_running = device_property_read_bool(dev, "always-running");

	watchdog_set_drvdata(&priv->wdd, priv);

	priv->wdd.info		= &gpio_wdt_ident;
	priv->wdd.ops		= &gpio_wdt_ops;
	priv->wdd.min_timeout	= SOFT_TIMEOUT_MIN;
	priv->wdd.max_hw_heartbeat_ms = hw_margin;
	priv->wdd.parent	= dev;
	priv->wdd.timeout	= SOFT_TIMEOUT_DEF;

	watchdog_init_timeout(&priv->wdd, 0, dev);
	watchdog_set_nowayout(&priv->wdd, nowayout);

	watchdog_stop_on_reboot(&priv->wdd);

	if (priv->always_running)
		gpio_wdt_start(&priv->wdd);

	ret = devm_watchdog_register_device(dev, &priv->wdd);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Initial timeout %d sec%s\n",
		 priv->wdd.timeout, nowayout ? ", nowayout" : "");

	return 0;
}

static const struct of_device_id gpio_wdt_dt_ids[] = {
	{ .compatible = "linux,wdt-gpio-ext", },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_wdt_dt_ids);

static int gpio_wdt_suspend(struct device *dev)
{
    struct  gpio_wdt_priv *priv = dev_get_drvdata(dev);

    STAMPX(priv->wdd.status);

    if (watchdog_active(&priv->wdd))
        gpio_wdt_stop(&priv->wdd);

    return 0;
}

static int gpio_wdt_resume(struct device *dev)
{
    struct  gpio_wdt_priv *priv = dev_get_drvdata(dev);

    STAMPX(priv->wdd.status);

    if (watchdog_active(&priv->wdd))
        gpio_wdt_start(&priv->wdd);

    return 0;
}

/* Bind callbacks to the PM subsystem */
static const struct dev_pm_ops gpio_wdt_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(gpio_wdt_suspend,
			    gpio_wdt_resume)
};

static struct platform_driver gpio_wdt_driver = {
	.driver	= {
		.name		= "gpio-wdt-ext",
		.of_match_table	= gpio_wdt_dt_ids,
		.pm = &gpio_wdt_pm_ops,
	},
	.probe	= gpio_wdt_probe,
};

#ifdef CONFIG_GPIO_WATCHDOG_ARCH_INITCALL
static int __init gpio_wdt_init(void)
{
	return platform_driver_register(&gpio_wdt_driver);
}
arch_initcall(gpio_wdt_init);
#else
module_platform_driver(gpio_wdt_driver);
#endif

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("GPIO Watchdog Ext");
MODULE_LICENSE("GPL");
