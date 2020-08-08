#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include "nrf.h"

static int backlight = -1;

int nrf_i2c_read(struct i2c_client *client, uint8_t addr, uint8_t *data, uint16_t len) {
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		printk("%s: i2c_transfer error: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int nrf_i2c_write(struct i2c_client *client, uint8_t addr, uint8_t *data, uint16_t len) {
	int ret = 0;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = 0,
	};

	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		printk("%s: kmalloc failed!\n", __func__);
		return -1;
	}

	msg.buf[0] = addr;
	memcpy(&msg.buf[1], data, len);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		printk("%s: i2c_transfer error: %d\n", __func__, ret);
		return ret;
	}

	kfree(msg.buf);

	return 0;
}

int nrf_i2c_feature(struct i2c_client *client, uint8_t pos, uint8_t mask, uint8_t offset, uint8_t value) {
	int ret = 0;
	nrf_client->config[pos] &= ~(mask << offset);
	nrf_client->config[pos] |= (value << offset);

	ret = nrf_i2c_write(client, REG_CONFIG, nrf_client->config, 4);
	if (ret) {
		printk("%s: nrf_i2c_write failed!\n", __func__);
	}

	printk("%s: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __func__,
		nrf_client->config[0], nrf_client->config[1], nrf_client->config[2], nrf_client->config[3]);

	return ret;
}

void nrf_set_backlight(int value) {
	if (backlight != value) {
		printk("%s: %d\n", __func__, value);
		backlight = value;
		nrf_i2c_feature(nrf_i2c_client, PWR_OFFSET_1, PWR_OFFSET_2, PWR_OFFSET_3, value ? 1 : 0);
	}
}

static void nrf_notify_key(void) {
	char buffer[2];
	int key = 0;

	nrf_i2c_read(nrf_i2c_client, REG_STATUS, buffer, 1);
	printk("%s: %d\n", __func__, buffer[0]);

	key = (buffer[0] & 0x01 || buffer[0] & 0x02) ? 1 : 0;
	kpd_key_input(KEY_BACK, key ? 1 : 0);
}

static int nrf_interrupt(unsigned char *buf) {
	unsigned char buffer[2];
	printk("%s: 0x%02X, 0x%02X, 0x%02X\n", __func__, buf[0], buf[1], buf[2]);

	if (buf[0] & 0x01) {
		nrf_notify_ebus(0x03);
	}

	if (buf[0] & 0x04) {
		nrf_notify_key();
	}

	if (buf[0] & 0x10) {
		nrf_notify_ebus(0x02);
	}

	if (buf[0] & 0x20) {
		nrf_notify_ebus(0x04);
	}

	if (buf[1] & 0x01) {
		nrf_notify_tilt();
	}

	if (buf[1] & 0x02) {
		buffer[0] = 0x01;
		nrf_i2c_write(nrf_i2c_client, REG_CONFIG, buffer, 1);
		nrf_i2c_write(nrf_i2c_client, REG_CONFIG, nrf_client->config, 4);
		nrf_notify_ebus(0x01);
	}

	if (buf[2] & 0x01) {
		nrf_notify_step();
	}

	return 0;
}

static irqreturn_t nrf_wake_irq_enable(int irq, void *handle) {
	unsigned char buffer[4];
	printk("%s\n", __func__);

	if (!nrf_i2c_read(nrf_i2c_client, REG_INPUT, buffer, 4)) {
		nrf_interrupt(buffer);
	}

	enable_irq(nrf_client->irq_wake);
	return IRQ_HANDLED;
}

static irqreturn_t nrf_wake_irq_disable(int irq, void *handle) {
	printk("%s\n", __func__);
	disable_irq_nosync(nrf_client->irq_wake);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t nrf_no_wake_irq_enable(int irq, void *handle) {
	printk("%s\n", __func__);
	enable_irq(nrf_client->irq_no_wake);
	return IRQ_HANDLED;
}

static irqreturn_t nrf_no_wake_irq_disable(int irq, void *handle) {
	printk("%s\n", __func__);
	disable_irq_nosync(nrf_client->irq_no_wake);
	return IRQ_WAKE_THREAD;
}

static struct of_device_id nrf_ctrl_table[] = {
	{.compatible = "mediatek,nrf_ctrl",},
	{},
};

static struct of_device_id nrf_ctrl_wk_table[] = {
	{.compatible = "mediatek,nrf_ctrl_wk",},
	{},
};

static struct of_device_id nrf_ctrl_nwk_table[] = {
	{.compatible = "mediatek,nrf_ctrl_nwk",},
	{},
};

static int nrf_dts_info(void) {
	struct device_node *node;
	printk("%s\n", __func__);

	node = of_find_matching_node(0, nrf_ctrl_wk_table);
	if (node) {
		nrf_client->irq_wake = irq_of_parse_and_map(node, 0);
	} else {
		printk("%s: irq_of_parse_and_map for irq_wake failed!\n", __func__);
		return -1;
	}

	node = of_find_matching_node(0, nrf_ctrl_nwk_table);
	if (node) {
		nrf_client->irq_no_wake = irq_of_parse_and_map(node, 0);
	} else {
		printk("%s: irq_of_parse_and_map for irq_no_wake failed!\n", __func__);
		return -1;
	}

	return 0;
}

static int nrf_request_irq(void) {
	int ret = 0;
	printk("%s\n", __func__);

	gpio_set(pin8_gpio_in_eint);
	gpio_set(pin10_gpio_in_eint);

	ret = nrf_dts_info();
	if (ret < 0) {
		printk("%s: nrf_dts_info failed!\n", __func__);
		return ret;
	}

	ret = request_threaded_irq(nrf_client->irq_wake, nrf_wake_irq_disable,
		nrf_wake_irq_enable, IRQF_TRIGGER_RISING, "nrf_ctrl_wake_irq", 0);

	ret = request_threaded_irq(nrf_client->irq_no_wake, nrf_no_wake_irq_disable,
		nrf_no_wake_irq_enable, IRQF_TRIGGER_RISING, "nrf_ctrl_no_wake_irq", 0);

	if (nrf_client->irq_wake) {
		enable_irq_wake(nrf_client->irq_wake);
	}

	return ret;
}

static int nrf_free_irq(void) {
	printk("%s\n", __func__);

	if (nrf_client->irq_wake) {
		free_irq(nrf_client->irq_wake, 0);
	}

	if (nrf_client->irq_no_wake) {
		free_irq(nrf_client->irq_no_wake, 0);
	}

	return 0;
}

static void nrf_chip_reset(void) {
	printk("%s\n", __func__);

	gpio_set(pin18_gpio_out1);
	msleep(100);
	gpio_set(pin18_gpio_out0);
	msleep(4000);
}

static int nrf_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	unsigned char buffer[2];

	nrf_client = kmalloc(sizeof(struct nrf_info), GFP_KERNEL);
	nrf_i2c_client = client;

	if (!nrf_client) {
		printk("%s: kmalloc failed!\n", __func__);
		return -1;
	}

	gpio_set(pin28_gpio_out1);
	gpio_set(pin7_gpio_out1);
	gpio_set(pin18_gpio_out0);
	gpio_set(pin4_gpio_out_u1tx);
	gpio_set(pin5_gpio_in_u1rx);

	if (nrf_i2c_read(nrf_i2c_client, REG_VERSION, buffer, 1)) {
		nrf_chip_reset();
	}

	if (nrf_i2c_read(nrf_i2c_client, REG_CHIP, buffer, 1)) {
		goto error;
	}

	printk("%s: version = 0x%02X\n", __func__, buffer[0]);

	if (buffer[0] == 0x58 || buffer[0] == 0x69) {
		nrf_client->version = buffer[0];
	} else {
		goto error;
	}

	buffer[0] = (0x01 << 7);
	nrf_i2c_read(nrf_i2c_client, REG_CONFIG, nrf_client->config, 4);
	nrf_i2c_write(nrf_i2c_client, REG_INPUT, buffer, 1);
	nrf_i2c_feature(nrf_i2c_client, SYS_OFFSET_1, SYS_OFFSET_2, SYS_OFFSET_3, 1);

	if (nrf_request_irq() < 0) {
		printk("%s: nrf_request_irq failed!\n", __func__);
	}

	nrf_client->init_status = 1;

	nrf_acc_register(nrf_i2c_client, nrf_client);
	nrf_ebus_register(nrf_i2c_client, nrf_client);
	nrf_hrs_register(nrf_i2c_client, nrf_client);
	nrf_misc_register(nrf_i2c_client, nrf_client);
	nrf_step_register(nrf_i2c_client, nrf_client);
	nrf_tilt_register(nrf_i2c_client, nrf_client);
	nrf_vibr_register(nrf_i2c_client, nrf_client);

	return 0;

error:
	printk("%s: failed!\n", __func__);
	nrf_client->init_status = 0;

	return -1;
}

static int nrf_i2c_remove(struct i2c_client *client) {
	nrf_free_irq();
	return 0;
}

static int nrf_i2c_suspend(struct device *dev) {
	printk("%s\n", __func__);

	if (nrf_client->irq_no_wake) {
		disable_irq_nosync(nrf_client->irq_no_wake);
	}

	return 0;
}

static int nrf_i2c_resume(struct device *dev) {
	printk("%s\n", __func__);

	if (nrf_client->irq_no_wake) {
		enable_irq(nrf_client->irq_no_wake);
	}

	return 0;
}

static const struct dev_pm_ops nrf_i2c_pm_ops = {
	.suspend = nrf_i2c_suspend,
	.resume = nrf_i2c_resume
};

static const struct i2c_device_id nrf_i2c_device_id[] = {
	{NRF_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nrf_i2c_device_id);

static struct i2c_driver nrf_i2c_driver = {
	.driver = {
		.name = NRF_NAME,
		.owner = THIS_MODULE,
		.of_match_table = nrf_ctrl_table,
		.pm = &nrf_i2c_pm_ops,
	},
	.probe = nrf_i2c_probe,
	.remove = nrf_i2c_remove,
	.id_table = nrf_i2c_device_id,
};

static int __init nrf_init(void) {
	if (i2c_add_driver(&nrf_i2c_driver)) {
		printk("%s: i2c_add_driver failed!\n", __func__);
		return -1;
	}

	return 0;
}

static void __exit nrf_exit(void) {
}

module_init(nrf_init);
module_exit(nrf_exit);
