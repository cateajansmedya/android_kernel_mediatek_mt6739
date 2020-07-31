#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "hwmsensor.h"
#include "sensor_attr.h"
#include "sensor_event.h"
#include "tyhx_hrs3300.h"
#include "gpio_driver.h"

#define DEV_HEART_RATE "m_hrs_misc"
#define HRS3300_NAME "tyhx_hrs3300"
#define HRS3300_ADDR 0x44

static struct sensor_attr_t heartrate_device;
static struct hrtimer hrs3300_hr_ctimer;
static struct hrtimer hrs3300_bp_ctimer;
static struct work_struct hrs3300_hr_work;
static struct work_struct hrs3300_bp_work;
static hrs3300_linux_data_t hrs3300data;

static int hrs3300_i2c_read(unsigned char reg, unsigned char *data) {
	unsigned char buf[2];
	int rc = 0;
	buf[0] = reg;

	rc = i2c_master_send(hrs3300data.client, buf, 1);
	if (rc != 1) {
		printk("[HRS3300] writing to address 0x%X failed!\n", reg);
		return -1;
	}

	rc = i2c_master_recv(hrs3300data.client, data, 1);
	if (rc != 1) {
		printk("[HRS3300] reading data %d failed!\n", rc);
		return -1;
	}

	return 0;

}

static int hrs3300_i2c_write(unsigned char reg, unsigned char *data, int len) {
	unsigned char buf[20];
	int rc = 0;

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	rc = i2c_master_send(hrs3300data.client, buf, len + 1);
	if (rc != len + 1) {
		printk("[HRS3300] writing to reg 0x%X failed!\n", reg);
		return -1;
	}

	return 0;
}

unsigned char Hrs3300_write_reg(unsigned char addr, unsigned char data) {
	return hrs3300_i2c_write(addr, &data, 1);
}

unsigned char Hrs3300_read_reg(unsigned char addr) {
	unsigned char data_buf = 0;
	hrs3300_i2c_read(addr, &data_buf);
	return data_buf;
}

void Hrs3300_chip_enable(void) {
	Hrs3300_write_reg(0x16, 0x78);
	Hrs3300_write_reg(0x01, 0xD0);
	Hrs3300_write_reg(0x0C, 0x2E);

	return;
}

void Hrs3300_chip_disable(void) {
	Hrs3300_write_reg(0x01, 0x08);
	Hrs3300_write_reg(0x02, 0x80);
	Hrs3300_write_reg(0x0C, 0x4E);
	Hrs3300_write_reg(0x16, 0x88);

	Hrs3300_write_reg(0x0C, 0x22);
	Hrs3300_write_reg(0x01, 0xF0);
	Hrs3300_write_reg(0x0C, 0x02);
	Hrs3300_write_reg(0x0c, 0x22);

	Hrs3300_write_reg(0x01, 0xF0);
	Hrs3300_write_reg(0x0C, 0x02);
	Hrs3300_write_reg(0x0C, 0x22);
	Hrs3300_write_reg(0x01, 0xF0);

	Hrs3300_write_reg(0x0C, 0x02);
	Hrs3300_write_reg(0x0C, 0x22);
	Hrs3300_write_reg(0x01, 0xF0);
	Hrs3300_write_reg(0x0C, 0x02);

	return;
}

bool Hrs3300_chip_init(void) {
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(init_register_array); i++) {
		if (Hrs3300_write_reg(init_register_array[i][0], init_register_array[i][1])) {
			return false;
		}
	}

	if (!hrs3300_power_up_flg) {
		reg_0x7f = Hrs3300_read_reg(0x7F);
		reg_0x80 = Hrs3300_read_reg(0x80);
		reg_0x81 = Hrs3300_read_reg(0x81);
		reg_0x82 = Hrs3300_read_reg(0x82);
		hrs3300_power_up_flg = 1;
	}

	return true;
}

static void hrs3300_init_hr(int enable) {
	if (!enable && hrs3300data.hrs_timer_enable) {
		printk("[HRS3300] disable heartrate function!\n");
		hrtimer_cancel(&hrs3300_hr_ctimer);
		hrs3300data.hrs_timer_enable = 0;
		Hrs3300_chip_disable();
		gpio_set(pin9_ldo_out0);
		mdelay(2);
	} else if (enable && !hrs3300data.hrs_timer_enable) {
		printk("[HRS3300] enable heartrate function!\n");
		gpio_set(pin9_ldo_out1);
		mdelay(5);
		Hrs3300_chip_init();
		Hrs3300_chip_enable();
		Hrs3300_alg_open();
		hrs3300data.hrs_timer_enable = 1;
		hrtimer_start(&hrs3300_hr_ctimer, ktime_set(0, 40 * 1000000), HRTIMER_MODE_REL);
	}
}

static void hrs3300_init_bp(int enable) {
	if (!enable && hrs3300data.hrs_timer_enable) {
		printk("[HRS3300] disable blood pressure function!\n");
		hrtimer_cancel(&hrs3300_bp_ctimer);
		hrs3300data.hrs_timer_enable = 0;
		Hrs3300_chip_disable();
		gpio_set(pin9_ldo_out0);
		mdelay(2);
	} else if (enable && !hrs3300data.hrs_timer_enable) {
		printk("[HRS3300] enable blood pressure function!\n");
		gpio_set(pin9_ldo_out1);
		mdelay(5);
		Hrs3300_chip_init();
		Hrs3300_chip_enable();
		Hrs3300_bp_alg_open();
		hrs3300data.hrs_timer_enable = 1;
		hrtimer_start(&hrs3300_bp_ctimer, ktime_set(0, 20 * 1000000), HRTIMER_MODE_REL);
	}
}

uint16_t Hrs3300_read_hrs(void) {
	uint8_t databuf[3];
	uint16_t data = 0;

	databuf[0] = Hrs3300_read_reg(0x09);
	databuf[1] = Hrs3300_read_reg(0x0A);
	databuf[2] = Hrs3300_read_reg(0x0F);
	data = ((databuf[0] << 8) | ((databuf[1] & 0x0F) << 4) | (databuf[2] & 0x0F));

	return data;
}

uint16_t Hrs3300_read_als(void) {
	uint8_t databuf[3];
	uint16_t data = 0;

	databuf[0] = Hrs3300_read_reg(0x08);
	databuf[1] = Hrs3300_read_reg(0x0D);
	databuf[2] = Hrs3300_read_reg(0x0E);
	data = ((databuf[0] << 3) | ((databuf[1] & 0x3F) << 11) | (databuf[2] & 0x07));

	if (data > 32767)
		data = 32767;

	return data;
}

static void hrs3300_send_event(void) {
	struct sensor_event event;

	event.flush_action = DATA_ACTION;
	event.handle = ID_HEART_RATE;
	event.word[0] = hr_date;
	event.word[1] = up_date;
	event.word[2] = down_date;
	sensor_input_event(heartrate_device.minor, &event);
}

void hrs3300_hr_timeout_handler(void) {
	uint16_t hrm_raw_data = 0;
	uint16_t als_raw_data = 0;
	static uint16_t timer_index = 0;
	hrs3300_results_t alg_results;

	hrm_raw_data = Hrs3300_read_hrs();
	als_raw_data = Hrs3300_read_als();
	Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, 0, 0, 0);

	timer_index++;
	if (timer_index >= 25) {
		timer_index = 0;
		alg_results = Hrs3300_alg_get_results();

		if (alg_results.alg_status == MSG_NO_TOUCH) {
			hr_date = 0;
		} else {
			hr_date = alg_results.hr_result;
		}

		hrs3300_send_event();
	}
}

void hrs3300_bp_timeout_handler(void) {
	uint16_t hrm_raw_data = 0;
	static uint16_t timer_index = 0;
	hrs3300_bp_results_t bp_alg_results;

	hrm_raw_data = Hrs3300_read_hrs();
	Hrs3300_bp_alg_send_data(hrm_raw_data);

	timer_index++;
	if (timer_index >= 50) {
		timer_index = 0;
		bp_alg_results = Hrs3300_alg_get_bp_results();

		if (bp_alg_results.bp_alg_status == MSG_BP_NO_TOUCH) {
			up_date = 0;
			down_date = 0;
		} else {
			up_date = bp_alg_results.sbp;
			down_date = bp_alg_results.dbp;
		}

		hrs3300_send_event();
	}
}

static void hrs3300_hr_work_func(struct work_struct *work) {
	hrs3300_hr_timeout_handler();

	if (hrs3300data.hrs_timer_enable) {
		hrtimer_start(&hrs3300_hr_ctimer, ktime_set(0, 35 * 1000000), HRTIMER_MODE_REL);
	}
}

static void hrs3300_bp_work_func(struct work_struct *work) {
	hrs3300_bp_timeout_handler();

	if (hrs3300data.hrs_timer_enable) {
		hrtimer_start(&hrs3300_bp_ctimer, ktime_set(0, 18 * 1000000), HRTIMER_MODE_REL);
	}
}

static enum hrtimer_restart hrs3300_hr_timer_func(struct hrtimer *timer) {
	schedule_work(&hrs3300_hr_work);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart hrs3300_bp_timer_func(struct hrtimer *timer) {
	schedule_work(&hrs3300_bp_work);
	return HRTIMER_NORESTART;
}

static ssize_t hrs3300_hr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0, handle = 0;
	sscanf(buf, "%d, %d", &handle, &enable);

	if (enable) {
		if (!hr_enable) {
			printk("[HRS3300] hr_store enable\n");
			hrs3300_init_hr(1);
			hr_enable = 1;
		}
	} else {
		if (hr_enable) {
			printk("[HRS3300] hr_store disable\n");
			hrs3300_init_hr(0);
			hr_enable = 0;
		}
	}

	return count;
}

static ssize_t hrs3300_hr_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", hr_enable);
}

static ssize_t hrs3300_support_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported");
}

static ssize_t hrs3300_bp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0;
	sscanf(buf, "%d", &enable);

	if (enable) {
		if (!bp_enable) {
			printk("[HRS3300] bp_store enable\n");
			hrs3300_init_bp(1);
			bp_enable = 1;
		}
	} else {
		if (bp_enable) {
			printk("[HRS3300] bp_store disable\n");
			hrs3300_init_bp(0);
			bp_enable = 0;
		}
	}

	return count;
}

static ssize_t hrs3300_bp_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "SYS = %d, DIA = %d\n", up_date, down_date);
}

static DEVICE_ATTR(hrs_active, 0644, hrs3300_hr_show, hrs3300_hr_store);
static DEVICE_ATTR(hrs_batch, 0444, hrs3300_support_show, NULL);
static DEVICE_ATTR(hrs_flush, 0444, hrs3300_support_show, NULL);
static DEVICE_ATTR(blood_pressure, 0644, hrs3300_bp_show, hrs3300_bp_store);

static struct attribute *hrs3300_sensor_attr_list[] = {
	&dev_attr_hrs_active.attr,
	&dev_attr_hrs_batch.attr,
	&dev_attr_hrs_flush.attr,
	&dev_attr_blood_pressure.attr,
	NULL,
};

static struct attribute_group hrs3300_attribute_group = {
	.attrs = hrs3300_sensor_attr_list
};

static int hrs3300_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t hrs3300_read(struct file *file, char *buf, size_t count, loff_t *pos) {
	return sensor_event_read(heartrate_device.minor, file, buf, count, pos);
}

static unsigned int hrs3300_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(heartrate_device.minor, file, wait);
}

static struct file_operations hrs3300_fops = {
	.owner = THIS_MODULE,
	.open = hrs3300_open,
	.read = hrs3300_read,
	.poll = hrs3300_poll,
};

static struct sensor_attr_t heartrate_device = {
	.minor = ID_HEART_RATE,
	.name = DEV_HEART_RATE,
	.fops = &hrs3300_fops,
};

static int hrs3300_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	hrs3300data.client = client;
	hrs3300data.hrs_timer_enable = 0;

	client->addr = HRS3300_ADDR;
	gpio_set(pin9_ldo_out1);
	mdelay(10);

	if (Hrs3300_read_reg(0x00) != 0x21) {
		printk("[HRS3300] chip id check failed!\n");
		goto error;
	}

	gpio_set(pin9_ldo_out0);

	if (sensor_attr_register(&heartrate_device)) {
		printk("[HRS3300] sensor_attr_register failed!\n");
		goto error;
	}

	if (sysfs_create_group(&heartrate_device.this_device->kobj, &hrs3300_attribute_group)) {
		printk("[HRS3300] sysfs_create_group failed!\n");
		goto error;
	}

	hrtimer_init(&hrs3300_hr_ctimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrs3300_hr_ctimer.function = hrs3300_hr_timer_func;
	INIT_WORK(&hrs3300_hr_work, hrs3300_hr_work_func);

	hrtimer_init(&hrs3300_bp_ctimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrs3300_bp_ctimer.function = hrs3300_bp_timer_func;
	INIT_WORK(&hrs3300_bp_work, hrs3300_bp_work_func);

	printk("[HRS3300] i2c_probe done!\n");
	hrs3300data.init_flag = true;

	return 0;

error:
	gpio_set(pin9_ldo_out0);
	hrs3300data.init_flag = false;

	return -1;
}

static int hrs3300_i2c_remove(struct i2c_client *client) {
	hrtimer_cancel(&hrs3300_hr_ctimer);
	hrtimer_cancel(&hrs3300_bp_ctimer);
	sysfs_remove_group(&heartrate_device.this_device->kobj, &hrs3300_attribute_group);
	sensor_attr_deregister(&heartrate_device);
	return 0;
}

static const struct i2c_device_id hrs3300_device_id[] = {
	{HRS3300_NAME, 0},
	{}
};

static struct of_device_id hrs3300_match_table[] = {
	{.compatible = "mediatek,heartrate",},
	{},
};
MODULE_DEVICE_TABLE(of, hrs3300_match_table);

struct i2c_driver hrs3300_i2c_driver={
	.driver = {
		.name = HRS3300_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hrs3300_match_table,
	},
	.probe = hrs3300_i2c_probe,
	.remove = hrs3300_i2c_remove,
	.id_table = hrs3300_device_id,
};

static int __init hrs3300_heartrate_init(void) {
	if (i2c_add_driver(&hrs3300_i2c_driver)) {
		printk("[HRS3300] unable to add i2c driver!\n");
		return -1;
	}

	return 0;
}

static void hrs3300_heartrate_exit(void) {
	i2c_del_driver(&hrs3300_i2c_driver);
	return;
}

module_init(hrs3300_heartrate_init);
module_exit(hrs3300_heartrate_exit);
