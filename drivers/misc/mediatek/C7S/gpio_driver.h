void gpio_set(int value);

#if defined(CONFIG_C7S_GPIO)
enum {
	pin8_gpio_out0,
	pin8_gpio_out1,
	pin9_ldo_out0,
	pin9_ldo_out1,
	pin10_gpio_out0,
	pin10_gpio_out1,
	pin11_gpio_in_eint,
};
#else
enum {
	pin4_gpio_out_u1tx,
	pin5_gpio_in_u1rx,
	pin6_gpio_out0,
	pin6_gpio_out1,
	pin7_gpio_out0,
	pin7_gpio_out1,
	pin8_gpio_in_eint,
	pin10_gpio_in_eint,
	pin11_gpio_out0,
	pin11_gpio_out1,
	pin18_gpio_out0,
	pin18_gpio_out1,
	pin28_gpio_out0,
	pin28_gpio_out1,
};
#endif
