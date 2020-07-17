#include <linux/reboot.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>

static void gpr_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx8mq-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		pr_info("IOMUXC_GPR5[0] = 1\n");
		regmap_update_bits(gpr, IOMUXC_GPR5, 1 << 0, 1 << 0);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomuxc-gpr regmap\n",
		       __func__);
	}
}

static int cl_som_imx8_init(void)
{
	struct device_node *np =
		of_find_compatible_node(NULL, NULL, "compulab,cl-som-imx8,rev1.0");

	if (!np)
		return 0;

	gpr_init();

	return 0;
}

static void __exit cl_som_imx8_exit(void)
{

}
module_init(cl_som_imx8_init);
module_exit(cl_som_imx8_exit);

MODULE_AUTHOR("CompuLab, Ltd.");
MODULE_DESCRIPTION("CompuLab CL-SOM-iMX8 machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cl-som-imx8");
