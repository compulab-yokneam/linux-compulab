/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/regmap.h>
#include "common.h"
#include "hardware.h"
#include <linux/string.h>

static void __init ucm_imx7_enet_clk_sel(void)
{
	struct regmap *gpr;
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET1_CLK_DIR_MASK, IMX7D_GPR1_ENET1_CLK_DIR_MASK);
	} else {
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
	}
}

static inline void ucm_imx7_enet_init(void)
{
	ucm_imx7_enet_clk_sel();
}

static int ucm_imx7_init(void)
{
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "compulab,ucm-imx7");
	if (!np)
		return -EINVAL;

	ucm_imx7_enet_init();

	return 0;
}

static void __exit ucm_imx7_exit(void)
{
	return;
}
module_init(ucm_imx7_init);
module_exit(ucm_imx7_exit);

MODULE_AUTHOR("CompuLab, Ltd.");
MODULE_DESCRIPTION("CompuLab UCM-iMX7 machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ucm-imx7");

