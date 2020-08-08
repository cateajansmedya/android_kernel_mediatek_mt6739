#include "nrf.h"

static struct sensor_attr_t nrf_tilt_device;

void nrf_notify_tilt(void) {
	struct sensor_event event;
	printk("%s\n", __func__);

	event.flush_action = DATA_ACTION;
	event.handle = ID_TILT_DETECTOR;
	event.word[0] = 1;
	sensor_input_event(nrf_tilt_device.minor, &event);
}

static ssize_t nrf_tilt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0, handle = 0;

	sscanf(buf, "%d, %d", &handle, &enable);
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->tilt_status) {
			nrf_i2c_feature(nrf_i2c_client, TILT_OFFSET_1, TILT_OFFSET_2, TILT_OFFSET_3, 1);
			nrf_client->tilt_status = 1;
		}
	} else {
		if (nrf_client->tilt_status) {
			nrf_i2c_feature(nrf_i2c_client, TILT_OFFSET_1, TILT_OFFSET_2, TILT_OFFSET_3, 0);
			nrf_client->tilt_status = 0;
		}
	}

	return count;
}

static ssize_t nrf_tilt_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", nrf_client->tilt_status);
}

static ssize_t nrf_info_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static DEVICE_ATTR(tilt_active, 0644, nrf_tilt_show, nrf_tilt_store);
static DEVICE_ATTR(tilt_batch, 0444, nrf_info_show, NULL);
static DEVICE_ATTR(tilt_flush, 0444, nrf_info_show, NULL);

static struct attribute *nrf_tilt_attributes[] = {
	&dev_attr_tilt_active.attr,
	&dev_attr_tilt_batch.attr,
	&dev_attr_tilt_flush.attr,
	NULL,
};

static struct attribute_group nrf_tilt_attribute_group = {
	.attrs = nrf_tilt_attributes
};

static int nrf_tilt_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t nrf_tilt_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos) {
	return sensor_event_read(nrf_tilt_device.minor, file, buffer, count, ppos);
}

static unsigned int nrf_tilt_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(nrf_tilt_device.minor, file, wait);
}

static const struct file_operations nrf_tilt_fops = {
	.owner = THIS_MODULE,
	.open = nrf_tilt_open,
	.read = nrf_tilt_read,
	.poll = nrf_tilt_poll,
};

static struct sensor_attr_t nrf_tilt_device = {
	.minor = ID_TILT_DETECTOR,
	.name = TILT_MISC_DEV_NAME,
	.fops = &nrf_tilt_fops,
};

int nrf_tilt_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);

	if (nrf_client->init_status) {
		if (sensor_attr_register(&nrf_tilt_device)) {
			printk("%s: sensor_attr_register failed!\n", __func__);
			return -1;
		}

		if (sysfs_create_group(&nrf_tilt_device.this_device->kobj, &nrf_tilt_attribute_group)) {
			printk("%s: sysfs_create_group failed!\n", __func__);
			return -1;
		}
	}

	return 0;
}
