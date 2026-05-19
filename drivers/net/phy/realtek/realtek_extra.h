#include <dt-bindings/net/realtek-rtl8211x.h>

#define RTL8211F_LCR				0x10
#define RTL8211F_SPEED_10			BIT(0)
#define RTL8211F_SPEED_100			BIT(1)
#define RTL8211F_SPEED_1000			BIT(3)
#define RTL8211F_ACTITITY			BIT(4)
#define RTL8211F_LED_CTRL			5
#define RTL8211F_LEDS_NUM			3

static int rtl8211x_config_init(struct phy_device *phydev)
{
#define RTL821E_PHYCR				0x10
#define RTL821E_PHYCR_CLK125			BIT(4)
	struct device *dev = &phydev->mdio.dev;
	u16 val;

	if (of_property_read_bool(dev->of_node, "realtek,clkout-disable")) {
		 __phy_modify(phydev, RTL821E_PHYCR, RTL821E_PHYCR_CLK125, RTL821E_PHYCR_CLK125);
	}

	val = __phy_read(phydev, RTL821E_PHYCR) & RTL821E_PHYCR_CLK125;
	dev_dbg(dev, "CLK125 clock is %s\n",  val ? "disabled" : "enabled");

	return 0;
};

static inline int _rtl8211f_config_leds(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	u32 led_modes[RTL8211F_LEDS_NUM];
	int len;
	int ret=0;

	len = of_property_read_variable_u32_array(dev->of_node, "realtek,led-modes",
						  led_modes, 0, ARRAY_SIZE(led_modes));
	if (len > 0) {
		u16 reg = 0;
		int i;

		for (i = 0; i < len; i++) {
			led_modes[i] &= PHY_LED_MASK;
			/* RTL8211F: Active makes no sense if no LINK Bit set */
			if (led_modes[i] == PHY_LED_ACTIVITY)
				led_modes[i] = PHY_LED_LINK_ACTIVITY;

			/* Parse the requested LED mode and set the corresponding bits in the control register */
			reg |= (((led_modes[i] & PHY_LED_ACTIVITY)   ? (RTL8211F_ACTITITY)   : (0)) |
				((led_modes[i] & PHY_LED_SPEED_10)   ? (RTL8211F_SPEED_10)   : (0)) |
				((led_modes[i] & PHY_LED_SPEED_100)  ? (RTL8211F_SPEED_100)  : (0)) |
				((led_modes[i] & PHY_LED_SPEED_1000) ? (RTL8211F_SPEED_1000) : (0))) << (i * RTL8211F_LED_CTRL);
		}
		dev_info(dev, "Applying LEDs configuration found in Device Tree\n");
		dev_dbg(dev, "Set LCR (LED Control Register) value: 0x%04x\n", reg);
		ret = phy_modify_paged_changed(phydev, 0xd04, RTL8211F_LCR, GENMASK(15,0), reg);
		if (ret < 0) {
			dev_err(dev, "Failed to update LED Control Register\n");
			return ret;
		}
	} else {
		dev_info(dev, "No valid LEDs configuration found, use defaults\n");
		return -ENODATA;
	}

	return 0;
}

static int rtl8211f_config_leds(struct phy_device *phydev) {
	struct device *dev = &phydev->mdio.dev;
	if ( _rtl8211f_config_leds(phydev) < 0 ) {
		dev_warn(dev, "Unable to apply LEDs settings from Device Tree, use defaults\n");
		phy_write_paged(phydev, 0xd04, RTL8211F_LCR, 0x2d7b);
	}
	dev_dbg(dev, "Read LCR (LED Control Register): 0x%04x\n",
		(unsigned int)phy_read_paged(phydev, 0xd04, RTL8211F_LCR));

	return 0;
}
