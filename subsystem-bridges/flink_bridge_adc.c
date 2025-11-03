// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

MODULE_AUTHOR("Patrick Good");
MODULE_DESCRIPTION("Flink GPIO Subsystem Bridge");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:flink-bridge-gpio");

static int flink_bridge_gpio_probe(struct platform_device *pdev)
{
    dev_info(&pdev->dev, "Hello from flink_bridge_gpio — autoload OK.\n");
    return 0;
}

static int flink_bridge_gpio_remove(struct platform_device *pdev)
{
    dev_info(&pdev->dev, "flink_bridge_gpio removed.\n");
    return 0;
}

static const struct of_device_id flink_bridge_gpio_of_match[] = {
    { .compatible = "ost,flink-bridge-gpio-1.0" },
    {},
};
MODULE_DEVICE_TABLE(of, flink_bridge_gpio_of_match);

static struct platform_driver flink_bridge_gpio_driver = {
    .probe  = flink_bridge_gpio_probe,
    .remove = flink_bridge_gpio_remove,
    .driver = {
        .name           = "flink-bridge-gpio",
        .of_match_table = flink_bridge_gpio_of_match,
    },
};
module_platform_driver(flink_bridge_gpio_driver);