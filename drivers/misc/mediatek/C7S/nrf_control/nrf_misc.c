#include <linux/miscdevice.h>
#include "nrf.h"

#define NRF_RX 0x05
#define NRF_BT 0x06

static DEFINE_MUTEX(nrf_gpio_mutex);
static int nrf_command = 0;

static int nrf_misc_health(const char *buf, int len) {
	int ret = 0, cmd = 0, health1 = 0, health2 = 0, health3 = 0, health4 = 0, value = 0;
	unsigned char buffer[8];

	ret = sscanf(buf, "%d, %d, %d, %d, %d, %d", &cmd, &health1, &health2, &health3, &health4, &value);
	if (ret == 6) {
		buffer[0] = health1;
		buffer[1] = health2;
		buffer[2] = health3;
		buffer[3] = health4;
		memcpy(&buffer[4], &value, 4);
		nrf_i2c_write(nrf_i2c_client, REG_HEALTH, buffer, 8);

		printk("%s: %d, %d, %d, %d, [%d, %d, %d, %d]\n", __func__,
			buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	return 0;
}

static int nrf_misc_notify(const char *buf, int len) {
	int handle = 0, enable = 0;

	sscanf(buf, "%d, %d", &handle, &enable);
	printk("%s: %d\n", __func__, enable);
	nrf_i2c_feature(nrf_i2c_client, MISC_OFFSET_1, MISC_OFFSET_2, MISC_OFFSET_3, enable ? 1 : 0);

	return 0;
}

static ssize_t nrf_mac_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	unsigned int buffer1[6];
	unsigned char buffer2[6];
	int ret = 0;

	ret = sscanf(buf, "%x:%x:%x:%x:%x:%x", &buffer1[0], &buffer1[1],
		&buffer1[2], &buffer1[3], &buffer1[4], &buffer1[5]);
	if (ret == 6) {
		buffer2[5] = buffer1[0];
		buffer2[4] = buffer1[1];
		buffer2[3] = buffer1[2];
		buffer2[2] = buffer1[3];
		buffer2[1] = buffer1[4];
		buffer2[0] = buffer1[5];
		nrf_i2c_write(nrf_i2c_client, REG_MAC, buffer2, 6);

		printk("%s: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
			buffer2[0], buffer2[1], buffer2[2], buffer2[3], buffer2[4], buffer2[5]);
	}

	return count;
}

static ssize_t nrf_mac_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "FF:FF:FF:FF:FF:FF\n");
}

static ssize_t nrf_command_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	unsigned char buffer[2];
	int cmd = 0;

	sscanf(buf, "%d", &cmd);
	printk("%s: 0x%02X, [%d]\n", __func__, cmd, count);

	switch (cmd) {
		case 0x01:
			nrf_misc_health(buf, count);
			break;

		case 0x02:
			buffer[0] = 0x01;
			nrf_i2c_write(nrf_i2c_client, REG_RESET, buffer, 1);
			break;

		case 0x03:
			nrf_i2c_feature(nrf_i2c_client, LCM_OFFSET_1, LCM_OFFSET_2, LCM_OFFSET_3, 1);
			break;

		case 0x04:
			nrf_i2c_feature(nrf_i2c_client, LCM_OFFSET_1, LCM_OFFSET_2, LCM_OFFSET_3, 0);
			break;

		case NRF_RX:
			nrf_command = NRF_RX;
			break;

		case NRF_BT:
			nrf_command = NRF_BT;
			break;

		case 0x07:
			nrf_misc_notify(buf, count);
			break;

		case 0x20:
			buffer[0] = 0x01;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x21:
			buffer[0] = 0x02;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x22:
			buffer[0] = 0x03;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x23:
			buffer[0] = 0x04;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x24:
			buffer[0] = 0x05;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x25:
			buffer[0] = 0x06;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x26:
			buffer[0] = 0x07;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x27:
			buffer[0] = 0x08;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x28:
			buffer[0] = 0x09;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x29:
			buffer[0] = 0x0A;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		case 0x2A:
			buffer[0] = 0x0B;
			nrf_i2c_write(nrf_i2c_client, REG_REMOTE, buffer, 1);
			break;

		default:
			break;
	}

	return count;
}

static ssize_t nrf_command_show(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned char buffer[2];
	int cmd1 = 0, cmd2 = 0, value = 0;

	nrf_i2c_read(nrf_i2c_client, REG_STATUS, buffer, 1);
	cmd1 = (buffer[0] & 0x10) ? 1 : 0;
	cmd2 = (buffer[0] & 0x20) ? 1 : 0;
	printk("%s: rx = %d, bt = %d, [0x%02X]\n", __func__, cmd1, cmd2, buffer[0]);

	if (nrf_command == NRF_RX) {
		value = cmd1 ? 1 : 0;
	}

	if (nrf_command == NRF_BT) {
		value = cmd2 ? 1 : 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t nrf_longterm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int cmd = 0;

	sscanf(buf, "%d", &cmd);
	printk("%s: 0x%02X\n", __func__, cmd);

	switch (cmd) {
		case 0x00:
			nrf_i2c_feature(nrf_i2c_client, SYS_OFFSET_1, SYS_OFFSET_2, SYS_OFFSET_3, 1);
			nrf_i2c_feature(nrf_i2c_client, TERM_OFFSET_1, TERM_OFFSET_2, TERM_OFFSET_3, 0);
			break;

		case 0x01:
			nrf_i2c_feature(nrf_i2c_client, TERM_OFFSET_1, TERM_OFFSET_2, TERM_OFFSET_3, 1);
			break;

		case 0x09:
			nrf_i2c_feature(nrf_i2c_client, SYS_OFFSET_1, SYS_OFFSET_2, SYS_OFFSET_3, 0);
			break;
	}

	return count;
}

static ssize_t nrf_config_show(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned char buffer[4];
	nrf_i2c_read(nrf_i2c_client, REG_CONFIG, buffer, 4);

	return snprintf(buf, PAGE_SIZE, "0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
		buffer[0], buffer[1], buffer[2], buffer[3]);
}

static ssize_t nrf_bt_show(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned char buffer[6];
	printk("%s\n", __func__);

	nrf_i2c_read(nrf_i2c_client, REG_MAC, buffer, 6);
	return snprintf(buf, PAGE_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X\n",
		buffer[5], buffer[4], buffer[3], buffer[2], buffer[1], buffer[0]);
}

static ssize_t nrf_rtc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	unsigned char buffer[6];
	int ret = 0, rtc1 = 0, rtc2 = 0, rtc3 = 0, rtc4 = 0, rtc5 = 0, rtc6 = 0, format = 0;

	ret = sscanf(buf, "%d-%d-%d %d:%d:%d %d", &rtc1, &rtc2, &rtc3, &rtc4, &rtc5, &rtc6, &format);
	if (ret == 7) {
		buffer[0] = rtc1 - 2000;
		buffer[1] = rtc2;
		buffer[2] = rtc3;
		buffer[3] = rtc4;
		buffer[4] = rtc5;
		buffer[5] = rtc6;

		nrf_i2c_feature(nrf_i2c_client, RTC_OFFSET_1, RTC_OFFSET_2, RTC_OFFSET_3, format ? 1 : 0);
		nrf_i2c_write(nrf_i2c_client, REG_RTC, buffer, 6);
		printk("%s: %04d-%02d-%02d %02d:%02d:%02d\n", __func__, rtc1, rtc2, rtc3, rtc4, rtc5, rtc6);
	}

	return size;
}

static ssize_t nrf_rtc_show(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned char buffer[6];
	nrf_i2c_read(nrf_i2c_client, REG_RTC, buffer, 6);

	return snprintf(buf, PAGE_SIZE, "%04d-%02d-%02d %02d:%02d:%02d\n",
		buffer[0] + 2000, buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
}

static ssize_t nrf_rst_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int cmd = 0;

	sscanf(buf, "%d", &cmd);
	printk("%s: %d\n", __func__, cmd);

	if (cmd) {
		mutex_lock(&nrf_gpio_mutex);
		gpio_set(pin18_gpio_out1);
		msleep(100);
		gpio_set(pin18_gpio_out0);
		mutex_unlock(&nrf_gpio_mutex);
	}

	return count;
}

static ssize_t nrf_uart_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0;

	sscanf(buf, "%d", &enable);
	printk("%s: %d\n", __func__, enable);
	nrf_i2c_feature(nrf_i2c_client, UART_OFFSET_1, UART_OFFSET_2, UART_OFFSET_3, enable ? 1 : 0);

	return count;
}

static ssize_t nrf_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int cmd = 0;

	sscanf(buf, "%d", &cmd);
	printk("%s: %d\n", __func__, cmd);

	if (cmd) {
		mutex_lock(&nrf_gpio_mutex);
		gpio_set(pin11_gpio_out1);
		gpio_set(pin18_gpio_out1);
		msleep(100);
		gpio_set(pin18_gpio_out0);
		mutex_unlock(&nrf_gpio_mutex);
	} else {
		mutex_lock(&nrf_gpio_mutex);
		gpio_set(pin11_gpio_out0);
		mutex_unlock(&nrf_gpio_mutex);
	}

	return count;
}

static ssize_t nrf_info_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t nrf_version_show(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned char buffer[2];
	int version = 0;

	if (nrf_client->version == 0x58) {
		nrf_i2c_read(nrf_i2c_client, REG_VERSION, &buffer[0], 1);
		buffer[1] = 0xFE;
	} else if (nrf_client->version == 0x69) {
		nrf_i2c_read(nrf_i2c_client, REG_VERSION, &buffer[0], 1);
		nrf_i2c_read(nrf_i2c_client, REG_VENDOR, &buffer[1], 1);
	} else {
		buffer[0] = 0xFF;
		buffer[1] = 0xFF;
	}

	version = buffer[1];
	version = (version << 8) | buffer[0];

	return snprintf(buf, PAGE_SIZE, "%d\n", version);
}

static DEVICE_ATTR(btMAC, 0644, nrf_mac_show, nrf_mac_store);
static DEVICE_ATTR(command, 0644, nrf_command_show, nrf_command_store);
static DEVICE_ATTR(config, 0444, nrf_config_show, NULL);
static DEVICE_ATTR(longterm, 0644, nrf_info_show, nrf_longterm_store);
static DEVICE_ATTR(nrf_bt, 0444, nrf_bt_show, NULL);
static DEVICE_ATTR(nrf_rtc, 0644, nrf_rtc_show, nrf_rtc_store);
static DEVICE_ATTR(rst_op, 0644, nrf_info_show, nrf_rst_store);
static DEVICE_ATTR(uart_op, 0644, nrf_info_show, nrf_uart_store);
static DEVICE_ATTR(upgrade_op, 0644, nrf_info_show, nrf_upgrade_store);
static DEVICE_ATTR(ver_no, 0444, nrf_version_show, NULL);

static struct attribute *nrf_misc_attributes[] = {
	&dev_attr_btMAC.attr,
	&dev_attr_command.attr,
	&dev_attr_config.attr,
	&dev_attr_longterm.attr,
	&dev_attr_nrf_bt.attr,
	&dev_attr_nrf_rtc.attr,
	&dev_attr_rst_op.attr,
	&dev_attr_uart_op.attr,
	&dev_attr_upgrade_op.attr,
	&dev_attr_ver_no.attr,
	NULL
};

static struct attribute_group nrf_misc_attribute_group = {
	.attrs = nrf_misc_attributes
};

static struct miscdevice nrf_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NRF_MISC,
};

int nrf_misc_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);

	if (nrf_client->init_status) {
		if (misc_register(&nrf_misc_device)) {
			printk("%s: misc_register failed!\n", __func__);
			return -1;
		}

		if (sysfs_create_group(&nrf_misc_device.this_device->kobj, &nrf_misc_attribute_group)) {
			printk("%s: sysfs_create_group failed!\n", __func__);
			return -1;
		}
	}

	return 0;
}
