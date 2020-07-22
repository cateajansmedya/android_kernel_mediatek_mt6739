#include <linux/platform_device.h>
#include <linux/module.h>
#include "gpio_driver.h"

struct pinctrl *gpio_ctrl;
struct pinctrl_state *gpio_state[32];
static DEFINE_MUTEX(gpio_mutex);

#if defined(CONFIG_C7S_GPIO)
const char* gpio_list[] = {
	"pin8_gpio_out0",
	"pin8_gpio_out1",
	"pin9_ldo_out0",
	"pin9_ldo_out1",
	"pin10_gpio_out0",
	"pin10_gpio_out1",
	"pin11_gpio_in_eint",
};
#else
const char* gpio_list[] = {
	"pin4_gpio_out_u1tx",
	"pin5_gpio_in_u1rx",
	"pin6_gpio_out0",
	"pin6_gpio_out1",
	"pin7_gpio_out0",
	"pin7_gpio_out1",
	"pin8_gpio_in_eint",
	"pin10_gpio_in_eint",
	"pin11_gpio_out0",
	"pin11_gpio_out1",
	"pin18_gpio_out0",
	"pin18_gpio_out1",
	"pin28_gpio_out0",
	"pin28_gpio_out1",
};
#endif

void gpio_set(int value) {
	printk("%s: %s\n", __func__, gpio_list[value]);
	mutex_lock(&gpio_mutex);
	pinctrl_select_state(gpio_ctrl, gpio_state[value]);
	mutex_unlock(&gpio_mutex);
}

static int gpio_probe(struct platform_device *pdev) {
	int i = 0;
	gpio_ctrl = devm_pinctrl_get(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(gpio_list); i++) {
		gpio_state[i] = pinctrl_lookup_state(gpio_ctrl, gpio_list[i]);
	}

	return 0;
}

static int gpio_remove(struct platform_device *pdev) {
	return 0;
}

static struct of_device_id gpio_table[] = {
	{ .compatible = "mediatek,wiite_ctrl", },
	{},
};

static struct platform_driver gpio_driver = {
	.probe = gpio_probe,
	.remove = gpio_remove,
	.driver = {
		.name = "gpio_driver",
		.owner = THIS_MODULE,
		.of_match_table = gpio_table,
	}
};

static int __init gpio_init(void) {
	if (platform_driver_register(&gpio_driver) != 0) {
		printk("%s: platform_driver_register failed\n", __func__);
	}

	return 0;
}

static void __exit gpio_exit(void) {
	platform_driver_unregister(&gpio_driver);
}

module_init(gpio_init);
module_exit(gpio_exit);
