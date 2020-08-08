#include "nrf.h"

#define DEV_HEART_RATE "m_hrs_misc"

static struct sensor_attr_t nrf_hrs_device;
static struct delayed_work nrf_hrs_work;

static void nrf_notify_hrs(void) {
	struct sensor_event event;
	unsigned char buffer[2];

	nrf_i2c_read(nrf_i2c_client, REG_HRS, buffer, 1);
	printk("%s: %d\n", __func__, buffer[0]);

	event.flush_action = DATA_ACTION;
	event.handle = ID_HEART_RATE;
	event.word[0] = buffer[0];
	sensor_input_event(nrf_hrs_device.minor, &event);
}

static void nrf_hrs_work_func(struct work_struct *work) {
	nrf_i2c_feature(nrf_i2c_client, HRS_OFFSET_1, HRS_OFFSET_2, HRS_OFFSET_3, 1);

	while (nrf_client->hrs_status) {
		nrf_notify_hrs();
		msleep(1000);
	}

	nrf_i2c_feature(nrf_i2c_client, HRS_OFFSET_1, HRS_OFFSET_2, HRS_OFFSET_3, 0);
}

static ssize_t nrf_hrs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0, handle = 0;

	sscanf(buf, "%d, %d", &handle, &enable);
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->hrs_status) {
			nrf_client->hrs_status = 1;
			schedule_delayed_work(&nrf_hrs_work, msecs_to_jiffies(100));
		}
	} else {
		if (nrf_client->hrs_status) {
			nrf_client->hrs_status = 0;
		}
	}

	return count;
}

static ssize_t nrf_hrs_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", nrf_client->hrs_status);
}

static ssize_t nrf_info_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static DEVICE_ATTR(hrs_active, 0644, nrf_hrs_show, nrf_hrs_store);
static DEVICE_ATTR(hrs_batch, 0444, nrf_info_show, NULL);
static DEVICE_ATTR(hrs_flush, 0444, nrf_info_show, NULL);

static struct attribute *nrf_hrs_attributes[] = {
	&dev_attr_hrs_active.attr,
	&dev_attr_hrs_batch.attr,
	&dev_attr_hrs_flush.attr,
	NULL,
};

static struct attribute_group nrf_hrs_attribute_group = {
	.attrs = nrf_hrs_attributes
};

static int nrf_hrs_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t nrf_hrs_read(struct file *file, char *buf, size_t count, loff_t *pos) {
	return sensor_event_read(nrf_hrs_device.minor, file, buf, count, pos);
}

static unsigned int nrf_hrs_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(nrf_hrs_device.minor, file, wait);
}

static struct file_operations nrf_hrs_fops = {
	.owner = THIS_MODULE,
	.open = nrf_hrs_open,
	.read = nrf_hrs_read,
	.poll = nrf_hrs_poll,
};

static struct sensor_attr_t nrf_hrs_device = {
	.minor = ID_HEART_RATE,
	.name = DEV_HEART_RATE,
	.fops = &nrf_hrs_fops,
};

int nrf_hrs_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);

	if (nrf_client->init_status) {
		if (sensor_attr_register(&nrf_hrs_device)) {
			printk("%s: sensor_attr_register failed!\n", __func__);
			return -1;
		}

		if (sysfs_create_group(&nrf_hrs_device.this_device->kobj, &nrf_hrs_attribute_group)) {
			printk("%s: sysfs_create_group failed!\n", __func__);
			return -1;
		}

		INIT_DELAYED_WORK(&nrf_hrs_work, nrf_hrs_work_func);
	}

	return 0;
}

