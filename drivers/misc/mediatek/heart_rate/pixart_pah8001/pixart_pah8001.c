#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "hwmsensor.h"
#include "sensor_attr.h"
#include "sensor_event.h"
#include "pixart_pah8001.h"
#include "gpio_driver.h"

static int pah8001_i2c_read(char addr, char *data, char len) {
	int ret = 0;
	char buffer[16];
	buffer[0] = addr;

	ret = i2c_master_send(pah8001data.client, buffer, 1);
	if (ret < 0) {
		printk("%s: writing to 0x%X failed!\n", __func__, addr);
	}

	ret = i2c_master_recv(pah8001data.client, data, len);
	if (ret < 0) {
		printk("%s: receiving data from 0x%X failed\n", __func__, addr);
	}

	return 0;
}

static int pah8001_i2c_write(char addr, char data) {
	int ret = 0;
	char buffer[2];
	buffer[0] = addr;
	buffer[1] = data;

	ret = i2c_master_send(pah8001data.client, buffer, 2);
	if (ret < 0) {
		printk("%s: writing to 0x%X failed!\n", __func__, addr);
	}

	return 0;
}

static void pah8001_led_ctrl(int touch) {
	int ep_l = 0, ep_h = 0, exposure_line = 0;

	if (touch != 0x80) {
		led_change_flag = 0;
		return;
	}

	if (state_count <= STATE_COUNT_TH) {
		state_count++;
		led_change_flag = 0;
		return;
	}

	state_count = 0;
	exposure_line = (ep_h << 8) + ep_l;

	if (!state) {
		if (exposure_line >= LED_CTRL_EXPO_TIME_HI_BOUND ||
				exposure_line <= LED_CTRL_EXPO_TIME_LOW_BOUND) {
			led_step &= 0x1F;

			if (exposure_line >= LED_CTRL_EXPO_TIME_HI_BOUND &&
					led_step < LED_CURRENT_HI) {
				state = 1;
				led_step = led_step + LED_INC_DEC_STEP;

				if (led_step > LED_CURRENT_HI) {
					led_step = LED_CURRENT_HI;
				}

				led_change_flag = 1;
			} else if (exposure_line <= LED_CTRL_EXPO_TIME_LOW_BOUND &&
					led_step > LED_CURRENT_LOW) {
				state = 2;

				if (led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
					led_step = LED_CURRENT_LOW;
				} else {
					led_step = led_step - LED_INC_DEC_STEP;
				}

				led_change_flag = 1;
			} else {
				state = 0;
				led_change_flag = 0;
			}
		} else {
			led_change_flag = 0;
		}
	} else if (state) {
		if (exposure_line > LED_CTRL_EXPO_TIME_HI) {
			state = 1;
			led_step = led_step + LED_INC_DEC_STEP;

			if (led_step >= LED_CURRENT_HI) {
				state = 0;
				led_step = LED_CURRENT_HI;
			}

			led_change_flag = 1;
		} else {
			state = 0;
			led_change_flag = 0;
		}
	} else {
		if (exposure_line < LED_CTRL_EXPO_TIME_LOW) {
			state = 2;

			if (led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
				state = 0;
				led_step = LED_CURRENT_LOW;
			} else {
				led_step = led_step - LED_INC_DEC_STEP;
			}

			led_change_flag = 1;
		} else {
			state = 0;
			led_change_flag = 0;
		}
	}
}

static void pah8001_power_down(int value) {
	if (value) {
		pah8001_i2c_write(0x7F, 0x00);
		pah8001_i2c_write(0x06, 0x0A);
	} else {
		pah8001_i2c_write(0x7F, 0x00);
		pah8001_i2c_write(0x06, 0x02);
	}
}

static int pah8001_chip_id(void) {
	char data[2];

	pah8001_i2c_write(0x7F, 0x00);
	pah8001_i2c_read(0x00, data, 2);
	printk("%s: 0x%X, 0x%X\n", __func__, data[0], data[1]);

	if (data[0] != 0x30 || (data[1] & 0xF0) != 0xD0) {
		return -1;
	}

	return 0;
}

static int pah8001_init_reg(void) {
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(init_ppg_array); i++) {
		pah8001_i2c_write(init_ppg_array[i][0], init_ppg_array[i][1]);
	}

	return 0;
}

static void pah8001_send_event(void) {
	struct sensor_event event;

	event.flush_action = DATA_ACTION;
	event.handle = ID_HEART_RATE;
	event.word[0] = *(int *)(pah8001data.ppg_mems_data.HRD_Data);
	event.word[1] = *(int *)(pah8001data.ppg_mems_data.HRD_Data + 4);
	event.word[2] = *(int *)(pah8001data.ppg_mems_data.HRD_Data + 8);
	sensor_input_event(pixart_device.minor, &event);
}

static void pah8001_ppg(void) {
	char data = 0, touch = 0;

	pah8001_i2c_write(0x7F, 0x00);
	pah8001_i2c_read(0x59, &touch, 1);
	pah8001_led_ctrl(touch & 0x80);

	pah8001_i2c_write(0x7F, 0x01);
	pah8001_i2c_read(0x68, &data, 1);
	pah8001data.ppg_mems_data.HRD_Data[0] = data & 0x0F;

	if (!pah8001data.ppg_mems_data.HRD_Data[0]) {
		pah8001_i2c_write(0x7F, 0x00);
		msleep(10);
	} else {
		pah8001_i2c_read(0x64, &pah8001data.ppg_mems_data.HRD_Data[1], 4);
		pah8001_i2c_read(0x1A, &pah8001data.ppg_mems_data.HRD_Data[5], 3);
		pah8001data.ppg_mems_data.HRD_Data[8] = frame_count++;

		stop = jiffies;
		pah8001data.ppg_mems_data.HRD_Data[9] = jiffies_to_msecs(stop - start);
		start = stop;

		pah8001data.ppg_mems_data.HRD_Data[10] = led_change_flag;
		pah8001data.ppg_mems_data.HRD_Data[11] = touch & 0x80;
		pah8001data.ppg_mems_data.HRD_Data[12] = pah8001data.ppg_mems_data.HRD_Data[6];

		pah8001_send_event();
	}
}

static void pah8001_x_work_func(struct work_struct *work) {
	pah8001_power_down(0);

	while (pah8001data.run_ppg) {
		pah8001_ppg();
	}

	pah8001_power_down(1);
}

static int pah8001_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t pah8001_read(struct file *file, char *buf, size_t count, loff_t *pos) {
	return sensor_event_read(pixart_device.minor, file, buf, count, pos);
}

static unsigned int pah8001_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(pixart_device.minor, file, wait);
}

static struct file_operations pah8001_fops = {
	.owner = THIS_MODULE,
	.open = pah8001_open,
	.read = pah8001_read,
	.poll = pah8001_poll,
};

static struct sensor_attr_t pixart_device = {
	.minor = ID_HEART_RATE,
	.name = DEV_HEART_RATE,
	.fops = &pah8001_fops,
};

static ssize_t pah8001_active_store(struct device* dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0, handle = 0;
	sscanf(buf, "%d, %d", &handle, &enable);

	if (enable) {
		printk("%s: enable\n", __func__);
		pah8001data.run_ppg = true;
		schedule_delayed_work(&pah8001data.x_work, msecs_to_jiffies(100));
	} else {
		printk("%s: disable\n", __func__);
		pah8001data.run_ppg = false;
	}

	return count;
}

static ssize_t pah8001_active_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", pah8001data.run_ppg);
}

static ssize_t pah8001_no_support_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported");
}

static DEVICE_ATTR(hrs_active, 0644, pah8001_active_show, pah8001_active_store);
static DEVICE_ATTR(hrs_batch, 0444, pah8001_no_support_show, NULL);
static DEVICE_ATTR(hrs_flush, 0444, pah8001_no_support_show, NULL);

static struct attribute *pah8001_attr_list[] = {
	&dev_attr_hrs_active.attr,
	&dev_attr_hrs_batch.attr,
	&dev_attr_hrs_flush.attr,
	NULL,
};

static struct attribute_group pah8001_attribute_group = {
	.attrs = pah8001_attr_list
};

static int pah8001_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -1;
	}

	client->addr = PAH8001_ADDR;
	pah8001data.client = client;

	gpio_set(pin11_gpio_in_eint);
	gpio_set(pin9_ldo_out1);
	mdelay(10);
	gpio_set(pin8_gpio_out0);
	mdelay(10);
	gpio_set(pin8_gpio_out1);
	mdelay(10);

	if (sensor_attr_register(&pixart_device)) {
		printk("%s: sensor_attr_register failed!\n", __func__);
		goto error;
	}

	if (sysfs_create_group(&pixart_device.this_device->kobj, &pah8001_attribute_group)) {
		printk("%s: sysfs_create_group failed!\n", __func__);
		goto error;
	}

	if (pah8001_chip_id() < 0) {
		printk("%s: pah8001_chip_id error!\n", __func__);
		goto error;
	}

	pah8001_init_reg();
	pah8001_power_down(1);

	INIT_DELAYED_WORK(&pah8001data.x_work, pah8001_x_work_func);
	pah8001data.run_ppg = false;

	printk("%s: done!\n", __func__);

	return 0;

error:
	gpio_set(pin9_ldo_out0);
	gpio_set(pin8_gpio_out0);
	printk("%s: failed!\n", __func__);

	return -1;
}

static int pah8001_i2c_remove(struct i2c_client *client) {
	return 0;
}

static const struct i2c_device_id pah8001_device_id[] = {
	{"pixart_pah8001", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pah8001_device_id);

static struct of_device_id pah8001_i2c_match_table[] = {
	{.compatible = "mediatek,heartrate",},
	{},
};

static struct i2c_driver pah8001_i2c_driver = {
	.driver = {
		.name = PAH8001_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pah8001_i2c_match_table,
	},
	.probe = pah8001_i2c_probe,
	.remove = pah8001_i2c_remove,
	.id_table = pah8001_device_id,
};

static int __init pah8001_init(void) {
	if (i2c_add_driver(&pah8001_i2c_driver) != 0) {
		printk("%s: i2c_add_driver failed!\n", __func__);
	}

	return 0;
}

static void __exit pah8001_exit(void) {
	sensor_attr_deregister(&pixart_device);
	sysfs_remove_group(&pixart_device.this_device->kobj, &pah8001_attribute_group);
	i2c_del_driver(&pah8001_i2c_driver);
}

module_init(pah8001_init);
module_exit(pah8001_exit);
