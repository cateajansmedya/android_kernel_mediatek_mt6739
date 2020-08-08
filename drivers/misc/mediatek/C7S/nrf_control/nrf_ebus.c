#include "nrf.h"

#define DEV_EVENT_BUS "m_ebus_misc"

static struct sensor_attr_t nrf_ebus_device;

void nrf_notify_ebus(int value) {
	struct sensor_event event;
	printk("%s: %d\n", __func__, value);

	event.flush_action = DATA_ACTION;
	event.handle = ID_LIGHT;
	event.word[0] = value + 1000;
	sensor_input_event(nrf_ebus_device.minor, &event);
}

static ssize_t nrf_ebus_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0;

	sscanf(buf, "%d", &enable);
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->ebus_status) {
			nrf_client->ebus_status = 1;
		}
	} else {
		if (nrf_client->ebus_status) {
			nrf_client->ebus_status = 0;
		}
	}

	return count;
}

static ssize_t nrf_ebus_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", nrf_client->ebus_status);
}

static ssize_t nrf_info_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static DEVICE_ATTR(ebus_active, 0644, nrf_ebus_show, nrf_ebus_store);
static DEVICE_ATTR(ebus_batch, 0444, nrf_info_show, NULL);
static DEVICE_ATTR(ebus_delay, 0444, nrf_info_show, NULL);
static DEVICE_ATTR(ebus_flush, 0444, nrf_info_show, NULL);

static struct attribute *nrf_ebus_attributes[] = {
	&dev_attr_ebus_active.attr,
	&dev_attr_ebus_batch.attr,
	&dev_attr_ebus_delay.attr,
	&dev_attr_ebus_flush.attr,
	NULL,
};

static struct attribute_group nrf_ebus_attribute_group = {
	.attrs = nrf_ebus_attributes
};

static int nrf_ebus_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t nrf_ebus_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos) {
	return sensor_event_read(nrf_ebus_device.minor, file, buffer, count, ppos);
}

static unsigned int nrf_ebus_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(nrf_ebus_device.minor, file, wait);
}

static const struct file_operations nrf_ebus_fops = {
	.owner = THIS_MODULE,
	.open = nrf_ebus_open,
	.read = nrf_ebus_read,
	.poll = nrf_ebus_poll,
};

static struct sensor_attr_t nrf_ebus_device = {
	.minor = ID_LIGHT,
	.name = DEV_EVENT_BUS,
	.fops = &nrf_ebus_fops,
};

int nrf_ebus_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);

	if (nrf_client->init_status) {
		if (sensor_attr_register(&nrf_ebus_device)) {
			printk("%s: sensor_attr_register failed!\n", __func__);
			return -1;
		}

		if (sysfs_create_group(&nrf_ebus_device.this_device->kobj, &nrf_ebus_attribute_group)) {
			printk("%s: sysfs_create_group failed!\n", __func__);
			return -1;
		}
	}

	return 0;
}
