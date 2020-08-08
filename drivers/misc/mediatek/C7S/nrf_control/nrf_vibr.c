#include "nrf.h"

static struct vibrator_hw *vibr;

struct vibrator_hw {
	int vib_timer;
};

struct vibrator_hw *get_cust_vibrator_dtsi(void) {
	if (!vibr) {
		printk("%s: failed!\n", __func__);
	}

	return vibr;
}

struct vibrator_hw *mt_get_cust_vibrator_hw(void) {
	struct vibrator_hw *hw = get_cust_vibrator_dtsi();
	return hw;
}

static void nrf_notify_vibr(int enable) {
	printk("%s: %d\n", __func__, enable);

	if (nrf_client->init_status) {
		nrf_i2c_feature(nrf_i2c_client, VIBR_OFFSET_1, VIBR_OFFSET_2, VIBR_OFFSET_3, enable ? 1 : 0);
	}
}

void vibr_Enable_HW(void) {
	nrf_notify_vibr(1);
}

void vibr_Disable_HW(void) {
	nrf_notify_vibr(0);
}

void vibr_power_set(void) {
	printk("%s: not supported!\n", __func__);
}

void init_cust_vibrator_dtsi(struct platform_device *pdev) {
	int ret = 0;

	if (!vibr) {
		vibr = kmalloc(sizeof(struct vibrator_hw), GFP_KERNEL);
		if (!vibr) {
			printk("%s: kmalloc failed!\n", __func__);
			return;
		}

		ret = of_property_read_u32(pdev->dev.of_node, "vib_timer", &vibr->vib_timer);
		if (ret) {
			vibr->vib_timer = 25;
		}
	}
}

int nrf_vibr_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);

	return 0;
}
