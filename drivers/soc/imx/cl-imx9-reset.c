#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/reboot.h>    // For reboot notifier
#include <linux/notifier.h>  // For notifier block

#define RESET_REGISTER_VALUE   0x1000

static u64 base;

static void clab_imx9_reset(void)
{
    void __iomem *reg_base;
    reg_base = ioremap(base, sizeof(u64));
    if (!reg_base) {
        pr_err("Failed to map reset register\n");
        return;
    }

    writel(RESET_REGISTER_VALUE, reg_base);  
    iounmap(reg_base);
}

static int clab_imx9_reboot_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
    pr_info("Custom board reset triggered on reboot\n");
    clab_imx9_reset();
    return NOTIFY_DONE;
}

static struct notifier_block clab_imx9_reboot_nb = {
    .notifier_call = clab_imx9_reboot_notifier,
};

static int clab_imx9_probe(struct platform_device *pdev)
{
    int ret;
	of_property_read_u64(pdev->dev.of_node, "reg", &base);
    pr_info("CLab imx9 reset driver initialized\n");
    ret = register_reboot_notifier(&clab_imx9_reboot_nb);
    if (ret) {
        pr_err("Failed to register reboot notifier\n");
        return ret;
    }
    return 0;
}

static const struct of_device_id clab_imx9_of_match[] = {
    { .compatible = "clab,imx9-reset", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, clab_imx9_of_match);

static struct platform_driver clab_imx9_driver = {
    .probe  = clab_imx9_probe,
    .driver = {
        .name           = "clab-imx9-reset",
        .of_match_table = clab_imx9_of_match,
    },
};
module_platform_driver(clab_imx9_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yaakov Entin");
MODULE_DESCRIPTION("Custom reset driver for clab imx9");

