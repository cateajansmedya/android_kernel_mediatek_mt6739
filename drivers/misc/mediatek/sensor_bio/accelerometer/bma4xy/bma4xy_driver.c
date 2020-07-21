/* BOSCH BMA4XY ACC Sensor Driver
 * (C) Copyright 2011~2017 Bosch Sensortec GmbH All Rights Reserved
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 * VERSION: V2.5
 * Date: 2017/05/08
 */

#define DRIVER_VERSION "0.0.2.5"
#include "bma4xy_driver.h"

#define BMA4XY_DEV_NAME "bma4xy_acc"
struct i2c_client *bma4xy_i2c_client;

static const struct i2c_device_id bma4xy_i2c_id[] = {
	{BMA4XY_DEV_NAME, 0},
	{}
};

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bma4xy_i2c_remove(struct i2c_client *client);
static int bma4xy_local_init(void);
static int bma4xy_remove(void);
static int bma4xy_tilt_local_init(void);

static struct data_resolution bma4xy_acc_data_resolution[1] = {
	{{1, 95}, 512},	/* +/-4g in 12-bit resolution: 1.95 mg/LSB */
};
static struct data_resolution bma4xy_acc_offset_resolution = {{1, 95}, 512};

#define GSE_TAG "[bma4xy] "
#define GSE_FUN(f) printk(GSE_TAG "%s\n", __func__)
#define GSE_ERR(fmt, args...) printk(GSE_TAG "%s %d: "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...) printk(GSE_TAG fmt, ##args)

#define ERRNUM1 -1
#define ERRNUM2 -2
#define ERRNUM3 -3
#define CHIP_ID_ERR -4

static DEFINE_MUTEX(gsensor_mutex);
struct acc_hw bma4_accel_cust;
struct bma4xy_client_data *bma4_obj_i2c_data;
struct sensor_attr_t tilt_device;
static struct acc_hw *hw = &bma4_accel_cust;
static struct GSENSOR_VECTOR3D gsensor_gain;
static bool enable_status;
static bool sensor_power = true;
static int sensor_suspend;
static int gsensor_init_flag = -1;
static int remap_dir = 0;
static int tilt_flag = 0;
static uint8_t dev_addr = 0;

enum BMA4XY_SENSOR_INT_MAP {
	BMA4XY_FFULL_INT = 8,
	BMA4XY_FWM_INT = 9,
	BMA4XY_DRDY_INT = 10,
};

enum BMA4XY_CONFIG_FUN {
	BMA4XY_SIG_MOTION_SENSOR = 0,
	BMA4XY_STEP_DETECTOR_SENSOR = 1,
	BMA4XY_STEP_COUNTER_SENSOR = 2,
	BMA4XY_TILT_SENSOR = 3,
	BMA4XY_PICKUP_SENSOR = 4,
	BMA4XY_GLANCE_DETECTOR_SENSOR = 5,
	BMA4XY_WAKEUP_SENSOR = 6,
	BMA4XY_ANY_MOTION_SENSOR = 7,
	BMA4XY_ORIENTATION_SENSOR = 8,
	BMA4XY_FLAT_SENSOR = 9,
	BMA4XY_TAP_SENSOR = 10,
	BMA4XY_HIGH_G_SENSOR = 11,
	BMA4XY_LOW_G_SENSOR = 12,
	BMA4XY_ACTIVITY_SENSOR = 13,
	BMA4XY_NO_MOTION_SENSOR = 14,
};

enum BMA4XY_INT_STATUS0 {
	SIG_MOTION_OUT = 0x01,
	STEP_DET_OUT = 0x02,
	TILT_OUT = 0x08,
	PICKUP_OUT = 0x08,
	GLANCE_OUT = 0x10,
	WAKEUP_OUT = 0x20,
	ANY_NO_MOTION_OUT = 0x40,
	ERROR_INT_OUT = 0x80,
};

enum BMA4XY_INT_STATUS1 {
	FIFOFULL_OUT = 0x01,
	FIFOWATERMARK_OUT = 0x02,
	MAG_DRDY_OUT = 0x20,
	ACC_DRDY_OUT = 0x80,
};

enum BMA4_FIFO_ANALYSE_RETURN_T {
	FIFO_OVER_READ_RETURN = -10,
	FIFO_SENSORTIME_RETURN = -9,
	FIFO_SKIP_OVER_LEN = -8,
	FIFO_M_A_OVER_LEN = -5,
	FIFO_M_OVER_LEN = -3,
	FIFO_A_OVER_LEN = -1
};

static void bma4xy_i2c_delay(uint32_t usec, void *ptr) {
	usleep_range(usec, usec + 1000);
}

static int bma4xy_i2c_read(struct i2c_client *client, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int32_t retry;
	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};

	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000, BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}

	if (BMA4XY_MAX_RETRY_I2C_XFER <= retry) {
		GSE_ERR("i2c transfer error!\n");
		return -EIO;
	}

	return 0;
}

static int bma4xy_i2c_write(struct i2c_client *client, uint8_t reg_addr,
		const uint8_t *data, uint8_t len) {
	int32_t retry;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};

	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		GSE_ERR("allocate memory failed!\n");
		return -ENOMEM;
	}

	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);

	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, &msg, 1) > 0)
			break;
		else
			usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000, BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}

	kfree(msg.buf);

	if (BMA4XY_MAX_RETRY_I2C_XFER <= retry) {
		GSE_ERR("i2c transfer error!\n");
		return -EIO;
	}

	return 0;
}

static int8_t bma4xy_i2c_read_wrapper(uint8_t addr, uint8_t *data, uint32_t len, void *ptr) {
	int err;
	err = bma4xy_i2c_read(bma4xy_i2c_client, addr, data, len);

	return err;
}

static int8_t bma4xy_i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint32_t len, void *ptr) {
	int err;
	err = bma4xy_i2c_write(bma4xy_i2c_client, addr, data, len);

	return err;
}

static void BMA4XY_power(struct acc_hw *hw, unsigned int on) {
}

static int BMA4XY_SetDataResolution(struct bma4xy_client_data *obj) {
	obj->reso = &bma4xy_acc_data_resolution[0];

	return 0;
}

static int BMA4XY_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize) {
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return ERRNUM1;

	if (NULL == client) {
		*buf = 0;
		return ERRNUM2;
	}

	snprintf(buf, 96, "BMA4XY Chip");

	return 0;
}

static int BMA4XY_ReadData(struct i2c_client *client, s16 data[BMA4XY_ACC_AXIS_NUM]) {
	int err = 0;
	struct bma4_accel raw_data;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (NULL == client)
		return -EINVAL;

	err = bma4_read_accel_xyz(&raw_data, &client_data->device);
	if (err < 0)
		return err;

	data[BMA4XY_ACC_AXIS_X] = raw_data.x;
	data[BMA4XY_ACC_AXIS_Y] = raw_data.y;
	data[BMA4XY_ACC_AXIS_Z] = raw_data.z;

	return err;
}

static int BMA4XY_ReadSensorData(struct i2c_client *client, char *buf, int bufsize) {
	struct bma4xy_client_data *obj = (struct bma4xy_client_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[BMA4XY_ACC_AXIS_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -ERRNUM1;

	if (NULL == client) {
		*buf = 0;
		return -ERRNUM2;
	}

	if (sensor_suspend == 1)
		return 0;

	res = BMA4XY_ReadData(client, obj->data);
	if (res != 0) {
		GSE_ERR("I2C error: value = %d\n", res);
		return -ERRNUM3;
	}

	obj->data[BMA4XY_ACC_AXIS_X] += obj->cali_sw[BMA4XY_ACC_AXIS_X];
	obj->data[BMA4XY_ACC_AXIS_Y] += obj->cali_sw[BMA4XY_ACC_AXIS_Y];
	obj->data[BMA4XY_ACC_AXIS_Z] += obj->cali_sw[BMA4XY_ACC_AXIS_Z];

	acc[obj->cvt.map[BMA4XY_ACC_AXIS_X]] = obj->cvt.sign[BMA4XY_ACC_AXIS_X] * obj->data[BMA4XY_ACC_AXIS_X];
	acc[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * obj->data[BMA4XY_ACC_AXIS_Y];
	acc[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * obj->data[BMA4XY_ACC_AXIS_Z];

	acc[BMA4XY_ACC_AXIS_X] = acc[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Y] = acc[BMA4XY_ACC_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[BMA4XY_ACC_AXIS_Z] = acc[BMA4XY_ACC_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	snprintf(buf, 96, "%04x %04x %04x", acc[BMA4XY_ACC_AXIS_X], acc[BMA4XY_ACC_AXIS_Y], acc[BMA4XY_ACC_AXIS_Z]);

	if (atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		GSE_LOG("gsensor data: %s\n", buf);

	return 0;
}

static int BMA4XY_ReadRawData(struct i2c_client *client, char *buf) {
	int res = 0;
	struct bma4xy_client_data *obj = (struct bma4xy_client_data *)i2c_get_clientdata(client);

	if (!buf || !client)
		return -EINVAL;

	res = BMA4XY_ReadData(client, obj->data);
	if (0 != res) {
		GSE_ERR("I2C error: value=%d\n", res);
		return -EIO;
	}

	snprintf(buf, PAGE_SIZE, "BMA4XY_ReadRawData %04X %04X %04X", obj->data[BMA4XY_ACC_AXIS_X],
		obj->data[BMA4XY_ACC_AXIS_Y], obj->data[BMA4XY_ACC_AXIS_Z]);

	return 0;
}

static int BMA4XY_ReadOffset(struct i2c_client *client, s8 ofs[BMA4XY_ACC_AXIS_NUM]) {
	int err = 0;

	ofs[0] = ofs[1] = ofs[2] = 0x0;
	GSE_LOG("offset: x = %X, y = %X, z = %X\n", ofs[0], ofs[1], ofs[2]);

	return err;
}

static int BMA4XY_ResetCalibration(struct i2c_client *client) {
	struct bma4xy_client_data *priv = i2c_get_clientdata(client);
	int err = 0;

	memset(priv->cali_sw, 0x00, sizeof(priv->cali_sw));
	memset(priv->offset, 0x00, sizeof(priv->offset));

	return err;
}

static int BMA4XY_ReadCalibration(struct i2c_client *client, int dat[BMA4XY_ACC_AXIS_NUM]) {
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int mul = 0;
	GSE_FUN();
	
	dat[obj->cvt.map[BMA4XY_ACC_AXIS_X]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_X]*(obj->offset[BMA4XY_ACC_AXIS_X]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_X]);
	dat[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Y]*(obj->offset[BMA4XY_ACC_AXIS_Y]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y]);
	dat[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] =
	obj->cvt.sign[BMA4XY_ACC_AXIS_Z]*(obj->offset[BMA4XY_ACC_AXIS_Z]*mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z]);

	return err;
}

static int BMA4XY_ReadCalibrationEx(struct i2c_client *client,
		int act[BMA4XY_ACC_AXIS_NUM], int raw[BMA4XY_ACC_AXIS_NUM]) {
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int mul = 0;

	raw[BMA4XY_ACC_AXIS_X] = obj->offset[BMA4XY_ACC_AXIS_X] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_X];
	raw[BMA4XY_ACC_AXIS_Y] = obj->offset[BMA4XY_ACC_AXIS_Y] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y];
	raw[BMA4XY_ACC_AXIS_Z] = obj->offset[BMA4XY_ACC_AXIS_Z] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_X]] = obj->cvt.sign[BMA4XY_ACC_AXIS_X] * raw[BMA4XY_ACC_AXIS_X];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_Y]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Y] * raw[BMA4XY_ACC_AXIS_Y];
	act[obj->cvt.map[BMA4XY_ACC_AXIS_Z]] = obj->cvt.sign[BMA4XY_ACC_AXIS_Z] * raw[BMA4XY_ACC_AXIS_Z];

	return err;
}

static int BMA4XY_WriteCalibration(struct i2c_client *client, int dat[BMA4XY_ACC_AXIS_NUM]) {
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	int err = 0;
	int cali[BMA4XY_ACC_AXIS_NUM], raw[BMA4XY_ACC_AXIS_NUM];

	err = BMA4XY_ReadCalibrationEx(client, cali, raw);
	if (0 != err) {
		GSE_ERR("read offset failed: %d\n", err);
		return err;
	}

	cali[BMA4XY_ACC_AXIS_X] += dat[BMA4XY_ACC_AXIS_X];
	cali[BMA4XY_ACC_AXIS_Y] += dat[BMA4XY_ACC_AXIS_Y];
	cali[BMA4XY_ACC_AXIS_Z] += dat[BMA4XY_ACC_AXIS_Z];

	obj->cali_sw[BMA4XY_ACC_AXIS_X] = obj->cvt.sign[BMA4XY_ACC_AXIS_X] *
	(cali[obj->cvt.map[BMA4XY_ACC_AXIS_X]]);
	obj->cali_sw[BMA4XY_ACC_AXIS_Y] = obj->cvt.sign[BMA4XY_ACC_AXIS_Y] *
	(cali[obj->cvt.map[BMA4XY_ACC_AXIS_Y]]);
	obj->cali_sw[BMA4XY_ACC_AXIS_Z] = obj->cvt.sign[BMA4XY_ACC_AXIS_Z] *
	(cali[obj->cvt.map[BMA4XY_ACC_AXIS_Z]]);

	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	return err;
}

static int BMA4XY_CheckDeviceID(struct i2c_client *client) {
	u8 databuf[2] = { 0 };
	int ret = 0;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	ret = bma4xy_i2c_read(client, BMA4XY_CHIP_ID, databuf, 1);
	if (ret < 0)
		goto exit_BMA4XY_CheckDeviceID;

	GSE_LOG("CheckDeviceID: %d\n", databuf[0]);
	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	return 0;

	exit_BMA4XY_CheckDeviceID:
		if (ret < 0) {
			GSE_ERR("CheckDeviceID: %d failed!\n ", databuf[0]);
			return ret;
		}

	return ret;
}

static int BMA4XY_SetPowerMode(struct i2c_client *client, bool enable) {
	int err = 0;
	struct bma4xy_client_data *client_data = (struct bma4xy_client_data *)i2c_get_clientdata(client);

	if (enable == 0 &&
			(client_data->sigmotion_enable == 0) && (client_data->stepdet_enable == 0) &&
			(client_data->stepcounter_enable == 0) && (client_data->tilt_enable == 0) &&
			(client_data->wrist_wear == 0) && (client_data->pickup_enable == 0) &&
			(client_data->glance_enable == 0) && (client_data->wakeup_enable == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		GSE_LOG("acc_op_mode = %d\n", enable);
	} else if (enable == 1) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		GSE_LOG("acc_op_mode = %d\n", enable);
	}

	if (err) {
		GSE_ERR("failed!\n");
		return err;
	}

	sensor_power = enable;
	client_data->acc_pm = enable;
	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);
	GSE_LOG("sensor_power = %d\n", sensor_power);

	return err;
}

static int BMA4XY_SetDataFormat(struct i2c_client *client, u8 dataformat) {
	int err = 0;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	struct bma4_accel_config acc_config;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	acc_config.range = (uint8_t)(BMA4_ACCEL_RANGE_4G);

	err += bma4_set_accel_config(&acc_config, &client_data->device);
	if (err) {
		GSE_ERR("failed!\n");
		return -EIO;
	}

	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);

	return BMA4XY_SetDataResolution(obj);
}

static int BMA4XY_SetBWRate(struct i2c_client *client, u8 bwrate) {
	uint8_t data = 0;
	int ret = 0;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	data = (uint8_t)bwrate;
	if (bwrate == 4)
		data = 0x74;
	else
		data |= 0xA0;

	ret = bma4xy_i2c_write_wrapper(0x40, &data, 1, client_data->device.intf_ptr);
	if (ret) {
		GSE_ERR("failed!\n");
		return ret;
	}

	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);
	GSE_LOG("acc_odr = %d\n", data);

	return ret;
}

static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data) {
	int err = 0;
	uint8_t chip_id = 0;
	uint8_t check_chip_id = 0;
	err = client_data->device.bus_read(BMA4_CHIP_ID_ADDR, &chip_id, 1, client_data->device.intf_ptr);
	if (err) {
		GSE_ERR("error\n");
		return err;
	}

	GSE_LOG("chip_id read back: 0x%X \n", chip_id);
	check_chip_id = BMA423_CHIP_ID;

	if (chip_id == check_chip_id)
		return err;
	else {
		err = CHIP_ID_ERR;
		return err;
	}
}

static ssize_t bma4xy_show_chip_id(struct device_driver *ddri, char *buf) {
	uint8_t chip_id = 0;
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4xy_i2c_read_wrapper(BMA4_CHIP_ID_ADDR, &chip_id, 1, client_data->device.intf_ptr);
	if (err) {
		GSE_ERR("failed!\n");
		return err;
	}

	return snprintf(buf, 48, "chip_id: %X\n", chip_id);
}

static ssize_t bma4xy_show_acc_op_mode(struct device_driver *ddri, char *buf) {
	int err;
	unsigned char acc_op_mode;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4_get_accel_enable(&acc_op_mode, &client_data->device);
	if (err) {
		GSE_ERR("read failed!\n");
		return err;
	}

	return snprintf(buf, 96, "status: %d\n", acc_op_mode);
}

static ssize_t bma4xy_store_acc_op_mode(struct device_driver *ddri, const char *buf, size_t count) {
	int err = 0;
	unsigned long op_mode;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	if (op_mode == 2 &&
		(client_data->sigmotion_enable == 0) && (client_data->stepdet_enable == 0) &&
		(client_data->stepcounter_enable == 0) && (client_data->tilt_enable == 0) &&
		(client_data->pickup_enable == 0) && (client_data->glance_enable == 0) &&
		(client_data->wakeup_enable == 0) && (client_data->anymotion_enable == 0) &&
		(client_data->nomotion_enable == 0) && (client_data->orientation_enable== 0)) {
			err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
			GSE_LOG("acc_op_mode %ld\n", op_mode);
		}
	else if (op_mode == 0) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		GSE_LOG("acc_op_mode %ld\n", op_mode);
	}

	if (err) {
		GSE_ERR("failed!\n");
		return err;
	}

	client_data->acc_pm = op_mode;
	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);

	return count;
}

static ssize_t bma4xy_show_acc_value(struct device_driver *ddri, char *buf) {
	struct bma4_accel data;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	int err;

	err = bma4_read_accel_xyz(&data, &client_data->device);
	if (err < 0)
		return err;

	return snprintf(buf, 48, "%hd %hd %hd\n", data.x, data.y, data.z);
}

static ssize_t bma4xy_show_acc_range(struct device_driver *ddri, char *buf) {
	int err;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		GSE_ERR("failed!\n");
		return err;
	}

	return snprintf(buf, 16, "%d\n", acc_config.range);
}

static ssize_t bma4xy_store_acc_range(struct device_driver *ddri, const char *buf, size_t count) {
	int err;
	unsigned long acc_range;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	struct bma4_accel_config acc_config;

	err = kstrtoul(buf, 10, &acc_range);
	if (err)
		return err;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	acc_config.range = (uint8_t)(acc_range);
	err += bma4_set_accel_config(&acc_config, &client_data->device);
	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);

	if (err) {
		GSE_ERR("failed!\n");
		return -EIO;
	}

	return count;
}

static ssize_t bma4xy_show_acc_odr(struct device_driver *ddri, char *buf) {
	int err;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		GSE_ERR("read failed!\n");
		return err;
	}

	client_data->acc_odr = acc_config.odr;

	return snprintf(buf, 16, "%d\n", client_data->acc_odr);

}

static ssize_t bma4xy_store_acc_odr(struct device_driver *ddri, const char *buf, size_t count) {
	int err;
	unsigned long acc_odr;
	uint8_t data = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &acc_odr);
	if (err)
		return err;

	data = (uint8_t)acc_odr;
	if (acc_odr == 4)
		data = 0x74;
	else
		data |= 0xA0;

	err = client_data->device.bus_write(0x40, &data, 1, client_data->device.intf_ptr);
	if (err) {
		GSE_ERR("failed!\n");
		return -EIO;
	}

	GSE_ERR("acc_odr = %ld\n", acc_odr);
	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);
	client_data->acc_odr = acc_odr;

	return count;
}

static ssize_t bma4xy_show_selftest(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	return snprintf(buf, 16, "%d\n", client_data->selftest);
}

static ssize_t bma4xy_store_selftest(struct device_driver *ddri, const char *buf, size_t count) {
	int err;
	uint8_t result = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4_perform_accel_selftest(&result, &client_data->device);
	if (err) {
		GSE_ERR("write failed!\n");
		return err;
	}

	if (result == 0) {
		GSE_LOG("selftest successsful\n");
		client_data->selftest = 1;
	} else
		client_data->selftest = 0;

	return count;
}

static ssize_t bma4xy_show_foc(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	return snprintf(buf, 64, "Use echo g_sign aixs > foc to begin foc\n");
}

static ssize_t bma4xy_store_foc(struct device_driver *ddri, const char *buf, size_t count) {
	int g_value[3] = {0};
	struct bma4_accel_foc_g_value data = {0};
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	int err = 0;

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	err = sscanf(buf, "%11d %11d %11d", &g_value[0], &g_value[1], &g_value[2]);
	GSE_LOG("g_value0 = %d, g_value1 = %d, g_value2 = %d\n", g_value[0], g_value[1], g_value[2]);

	if (err != 3) {
		GSE_ERR("invalid argument!\n");
		return -EINVAL;
	}

	data.x = g_value[0] * BMA4XY_MULTIPLIER;
	data.y = g_value[1] * BMA4XY_MULTIPLIER;
	data.z = g_value[2] * BMA4XY_MULTIPLIER;

	err = bma4_perform_accel_foc(&data, &client_data->device);
	if (err) {
		GSE_ERR("write failed!\n");
		return err;
	}

	GSE_FUN("FOC successfull");

	return count;
}

static ssize_t bma4xy_show_config_function(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	return snprintf(buf, PAGE_SIZE,
		"sig_motion0 = %d, step_detector1 = %d, step_counter2 = %d\n"
		"tilt3 = %d, pickup4 = %d, glance_detector5 = %d, wakeup6 =% d\n"
		"any_motion7 = %d, nomotion8 = %d, orientation9 = %d flat10 = %d\n"
		"high_g11 = %d, low_g12 = %d, activity13 = %d, nomotion14 = %d\n",
		client_data->sigmotion_enable, client_data->stepdet_enable,
		client_data->stepcounter_enable, client_data->tilt_enable,
		client_data->pickup_enable, client_data->glance_enable,
		client_data->wakeup_enable, client_data->anymotion_enable,
		client_data->nomotion_enable, client_data->orientation_enable,
		client_data->flat_enable, client_data->highg_enable,
		client_data->lowg_enable, client_data->activity_enable,
		client_data->nomotion_enable);
}

static ssize_t bma4xy_store_config_function(struct device_driver *ddri, const char *buf, size_t count) {
	int ret;
	int config_func = 0;
	int enable = 0;
	uint8_t feature = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	GSE_LOG("config_func = %d, enable = %d\n", config_func, enable);

	if (ret != 2) {
		GSE_ERR("invalid argument!\n");
		return -EINVAL;
	}

	if (config_func < 0 || config_func > 14)
		return -EINVAL;

	switch (config_func) {
		case BMA4XY_SIG_MOTION_SENSOR:
			client_data->sigmotion_enable = enable;
			break;
		case BMA4XY_STEP_DETECTOR_SENSOR:
			if (bma423_step_detector_enable(enable, &client_data->device) < 0) {
				GSE_ERR("BMA4XY_STEP_DETECTOR_SENSOR error!\n");
				return -EINVAL;
			}
			client_data->stepdet_enable = enable;
			break;
		case BMA4XY_STEP_COUNTER_SENSOR:
			feature = BMA423_STEP_CNTR;
			client_data->stepcounter_enable = enable;
			break;
		case BMA4XY_TILT_SENSOR:
			client_data->tilt_enable = enable;
			break;
		case BMA4XY_PICKUP_SENSOR:
			client_data->pickup_enable = enable;
			break;
		case BMA4XY_GLANCE_DETECTOR_SENSOR:
			client_data->glance_enable = enable;
			break;
		case BMA4XY_WAKEUP_SENSOR:
			client_data->wakeup_enable = enable;
			break;
		case BMA4XY_ANY_MOTION_SENSOR:
			client_data->anymotion_enable = enable;
			break;
		case BMA4XY_ORIENTATION_SENSOR:
			client_data->orientation_enable = enable;
			break;
		case BMA4XY_FLAT_SENSOR:
			client_data->flat_enable = enable;
			break;
		case BMA4XY_TAP_SENSOR:
			client_data->tap_enable = enable;
			break;
		case BMA4XY_HIGH_G_SENSOR:
			client_data->highg_enable = enable;
			break;
		case BMA4XY_LOW_G_SENSOR:
			client_data->lowg_enable = enable;
			break;
		case BMA4XY_ACTIVITY_SENSOR:
			client_data->activity_enable = enable;
			break;
		case BMA4XY_NO_MOTION_SENSOR:
			client_data->nomotion_enable = enable;
			break;
		default:
			GSE_ERR("invalid sensor handle: %d\n", config_func);
			return -EINVAL;
	}

	if (bma423_feature_enable(feature, enable, &client_data->device) < 0) {
		GSE_ERR("set bma423 virtual error!");
		return -EINVAL;
	}

	return count;
}

static ssize_t bma4xy_dump_regs_function(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!");
		return -ENODEV;
	}

	return 0;

}

static ssize_t bma4xy_show_load_config_stream(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	return snprintf(buf, 48, "config stream %s\n", client_data->config_stream_name);
}

static void bma4xy_set_tilt_remap(struct bma4xy_client_data *client_data,
		struct bma423_axes_remap * axis_remap_data) {
	if (client_data && client_data->hw) {
		switch (remap_dir) {
			case 1:
				axis_remap_data->x_axis = 0;
				axis_remap_data->x_axis_sign = 0;
				axis_remap_data->y_axis = 1;
				axis_remap_data->y_axis_sign = 0;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 0;
				break;
			case 2:
				axis_remap_data->x_axis = 0;
				axis_remap_data->x_axis_sign = 1;
				axis_remap_data->y_axis = 1;
				axis_remap_data->y_axis_sign = 1;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 0;
				break;
			case 3:
				axis_remap_data->x_axis = 1;
				axis_remap_data->x_axis_sign = 1;
				axis_remap_data->y_axis = 0;
				axis_remap_data->y_axis_sign = 0;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 0;
				break;
			case 4:
				axis_remap_data->x_axis = 1;
				axis_remap_data->x_axis_sign = 0;
				axis_remap_data->y_axis = 0;
				axis_remap_data->y_axis_sign = 1;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 0;
				break;
			case 5:
				axis_remap_data->x_axis = 1;
				axis_remap_data->x_axis_sign = 0;
				axis_remap_data->y_axis = 0;
				axis_remap_data->y_axis_sign = 0;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 1;
				break;
			case 6:
				axis_remap_data->x_axis = 1;
				axis_remap_data->x_axis_sign = 1;
				axis_remap_data->y_axis = 0;
				axis_remap_data->y_axis_sign = 1;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 1;
				break;
			case 7:
				axis_remap_data->x_axis = 0;
				axis_remap_data->x_axis_sign = 1;
				axis_remap_data->y_axis = 1;
				axis_remap_data->y_axis_sign = 0;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 1;
				break;
			case 8:
				axis_remap_data->x_axis = 0;
				axis_remap_data->x_axis_sign = 0;
				axis_remap_data->y_axis = 1;
				axis_remap_data->y_axis_sign = 1;
				axis_remap_data->z_axis = 2;
				axis_remap_data->z_axis_sign = 1;
				break;
			default:
				break;
		}
	}
}

int bma4xy_init_after_config_stream_load(struct bma4xy_client_data *client_data) {
	int err = 0;
	uint8_t int_enable = 0x0B;
	uint8_t latch_enable = 0x00;
	uint8_t int1_map = 0x08;
	struct bma423_axes_remap axis_remap_data;

	GSE_FUN();
	err = bma4_write_regs(BMA4_INT_MAP_1_ADDR, &int1_map, 1, &client_data->device);
	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);

	err += bma4_write_regs(BMA4_INT1_IO_CTRL_ADDR, &int_enable, 1, &client_data->device);
	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	err += bma4_write_regs(BMA4_INTR_LATCH_ADDR, &latch_enable, 1, &client_data->device);
	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	if (err)
		GSE_ERR("map and enable interrupt failed err = %d", err);

	memset(&axis_remap_data, 0, sizeof(axis_remap_data));
	bma4xy_set_tilt_remap(client_data, &axis_remap_data);

	err = bma423_set_remap_axes(&axis_remap_data, &client_data->device);
	if (err)
		GSE_ERR("set bma423 step_count select platform error!");

	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);
	if (err) {
		GSE_ERR("write axis_remap failed!\n");
		return err;
	}

	return err;
}

int bma4xy_init_fifo_config(struct bma4xy_client_data *client_data) {
	int err = 0;

	err = bma4_set_fifo_config(BMA4_FIFO_HEADER, BMA4_ENABLE, &client_data->device);
	if (err)
		GSE_ERR("enable fifo header failed err = %d", err);

	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	err = bma4_set_fifo_config(BMA4_FIFO_TIME, BMA4_ENABLE, &client_data->device);
	if (err)
		GSE_ERR("enable fifo timer failed err = %d", err);

	bma4xy_i2c_delay(BMA4_MS_TO_US(1), client_data->device.intf_ptr);

	return err;
}

int bma4xy_update_config_stream(struct bma4xy_client_data *client_data, int choose) {
	char *name;
	int err = 0;
	uint8_t crc_check = 0;

	switch (choose) {
		case 1:
		name = "android.tbin";
		break;
	case 2:
		name = "legacy.tbin";
		break;
	default:
		GSE_LOG("using default firmware\n");
		name = "bma4xy_config_stream";
		break;
	}

	GSE_LOG("config_stream %s", name);
	client_data->config_stream_name = name;

	if ((choose == 1) || (choose == 2)) {
		GSE_ERR("using default firmware\n");
		GSE_LOG("config_stream: %s", name);
	} else if (choose == 3) {
		err = bma423_write_config_file(&client_data->device);
		if (err)
			GSE_ERR("download config stream failed!");

		bma4xy_i2c_delay(BMA4_MS_TO_US(200), client_data->device.intf_ptr);

		err = bma4_read_regs(BMA4_INTERNAL_STAT, &crc_check, 1, &client_data->device);
		if (err)
			GSE_ERR("reading crc failed!\n");

		if (crc_check != BMA4_ASIC_INITIALIZED)
			GSE_ERR("crc check error: %X", crc_check);
	}

	return err;
}

static ssize_t bma4xy_store_load_config_stream(struct device_driver *ddri,
		const char *buf, size_t count) {
	unsigned long choose = 0;
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = kstrtoul(buf, 10, &choose);
	if (err)
		return err;

	GSE_LOG("config_stream_choose %ld\n", choose);

	err = bma4xy_update_config_stream(client_data, choose);
	if (err) {
		GSE_ERR("config_stream load error!\n");
		return count;
	}

	err = bma4xy_init_after_config_stream_load(client_data);
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error!\n");
		return count;
	}

	return count;
}

int bma4xy_load_config_stream(struct bma4xy_client_data *client_data) {
	int err = 0;

	err = bma4xy_update_config_stream(client_data, 3);
	if (err) {
		GSE_ERR("config_stream load error!\n");
		return err;
	}

	err = bma4xy_init_after_config_stream_load(client_data);
	if (err) {
		GSE_ERR("bma4xy_init_after_config_stream_load error!\n");
		return err;
	}

	return err;
}

static ssize_t bma4xy_show_reg_sel(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	return snprintf(buf, 64, "reg = 0X%02X, len = %d\n", client_data->reg_sel, client_data->reg_len);
}

static ssize_t bma4xy_store_reg_sel(struct device_driver *ddri, const char *buf, size_t count) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	ssize_t ret;

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11X %11d", &client_data->reg_sel, &client_data->reg_len);
	if (ret != 2) {
		GSE_ERR("invalid argument!\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t bma4xy_show_reg_val(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	ssize_t ret;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	if (client_data->reg_sel == BMA4_FEATURE_CONFIG_ADDR)
		ret = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR,reg_data, client_data->reg_len, &client_data->device);
	else
		ret = client_data->device.bus_read(client_data->reg_sel, reg_data,
			client_data->reg_len, client_data->device.intf_ptr);

	if (ret < 0) {
		GSE_ERR("reg op failed!\n");
		return ret;
	}

	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}

	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';

	return pos;
}

static ssize_t bma4xy_store_reg_val(struct device_driver *ddri, const char *buf, size_t count) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);
	ssize_t ret;
	u8 reg_data[128];
	int i, j, status, digit;

	if (client_data == NULL) {
		GSE_ERR("invalid client_data pointer!\n");
		return -ENODEV;
	}

	status = 0;
	for (i = j = 0; i < count && j < client_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' || buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}

		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		GSE_LOG("digit is %d\n", digit);
		switch (status) {
			case 0:
				reg_data[j] = digit;
				status = 1;
				break;
			case 1:
				reg_data[j] = reg_data[j] * 16 + digit;
				status = 2;
				break;
			case 2:
				++j;
		}
	}

	if (status > 0)
		++j;

	if (j > client_data->reg_len)
		j = client_data->reg_len;
	else if (j < client_data->reg_len) {
		GSE_ERR("invalid argument!\n");
		return -EINVAL;
	}

	GSE_LOG("reg data read as");

	for (i = 0; i < j; ++i)
		GSE_LOG("%d", reg_data[i]);

	if (client_data->reg_sel == BMA4_FEATURE_CONFIG_ADDR)
		ret = bma4_write_regs(BMA4_FEATURE_CONFIG_ADDR, reg_data, client_data->reg_len, &client_data->device);
	else
		ret = client_data->device.bus_write(client_data->reg_sel,
			reg_data, client_data->reg_len, client_data->device.intf_ptr);

	if (ret < 0) {
		GSE_ERR("reg op failed!\n");
		return ret;
	}

	return count;
}

static ssize_t bma4xy_show_driver_version(struct device_driver *ddri, char *buf) {
	return snprintf(buf, 128, "driver version: %s\n", DRIVER_VERSION);
}

static ssize_t bma4xy_show_config_file_version(struct device_driver *ddri, char *buf) {
	int err = 0;
	uint16_t version = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma423_get_config_id(&version, &client_data->device);

	if (err) {
		GSE_ERR("read failed!\n");
		return err;
	}

	return snprintf(buf, 128, "driver version: %s, config version: 0x%X\n",
		DRIVER_VERSION, version);
}

static ssize_t bma4xy_show_avail_sensor(struct device_driver *ddri, char *buf) {
	uint16_t avail_sensor = 423;

	return snprintf(buf, 32, "%d\n", avail_sensor);
}

int bma423_config_feature(struct bma4xy_client_data *client_data) {
	int err = 0;
	uint8_t feature = 0;

	if (client_data->stepdet_enable == BMA4_ENABLE) {
		if (bma423_step_detector_enable(BMA4_ENABLE, &client_data->device) < 0)
			GSE_ERR("set BMA421_STEP_DETECTOR error!");
	}

	bma4xy_i2c_delay(BMA4_MS_TO_US(2), client_data->device.intf_ptr);

	if (client_data->wrist_wear == BMA4_ENABLE)
		feature = feature | BMA423_WRIST_WEAR;
	if (client_data->single_tap == BMA4_ENABLE)
		feature = feature | BMA423_SINGLE_TAP;
	if (client_data->double_tap == BMA4_ENABLE)
		feature = feature | BMA423_DOUBLE_TAP;
	if (client_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA423_STEP_CNTR;
	if (client_data->stepdet_enable == BMA4_ENABLE)
		feature = feature | BMA423_STEP_ACT;

	err = bma423_feature_enable(feature, BMA4_ENABLE, &client_data->device);
	if (err)
		GSE_ERR("set feature error!");

	return err;
}

int bma4xy_reinit_after_error_interrupt(struct bma4xy_client_data *client_data) {
	int err = 0;
	uint8_t data = 0;
	uint8_t crc_check = 0;

	client_data->err_int_trigger_num += 1;
	client_data->step_counter_val = client_data->step_counter_temp;

	err = bma4_set_command_register(0xB6, &client_data->device);
	if (!err)
		GSE_LOG("reset chip!\n");

	err = bma4xy_init_fifo_config(client_data);
	if (err)
		GSE_ERR("fifo init failed!\n");

	err = bma4_write_config_file(&client_data->device);
	if (err)
		GSE_ERR("download config stream failed!");

	bma4xy_i2c_delay(BMA4_MS_TO_US(200), client_data->device.intf_ptr);

	err = bma4_read_regs(BMA4_INTERNAL_STAT, &crc_check, 1, &client_data->device);
	if (err)
		GSE_ERR("reading crc failed!\n");

	if (crc_check != BMA4_ASIC_INITIALIZED)
		GSE_ERR("crc check error: %X\n", crc_check);

	err = bma4xy_init_after_config_stream_load(client_data);
	if (err)
		GSE_ERR("reconfig interrupt and remap error!\n");

	err = bma423_config_feature(client_data);
	if (err)
		GSE_ERR("re-init virtual sensor error!\n");

	if (client_data->acc_odr != 0) {
		data = client_data->acc_odr;
		if (data == 4)
			data = 0x74;
		else
			data |= 0xA0;

		err = client_data->device.bus_write(0x40, &data, 1, client_data->device.intf_ptr);
		if (err)
			GSE_ERR("set acc_odr failed!\n");

		bma4xy_i2c_delay(BMA4_MS_TO_US(2), client_data->device.intf_ptr);
	}

	if (client_data->acc_pm == 0)
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);

	if (err)
		GSE_ERR("set acc_op_mode failed!\n");

	bma4xy_i2c_delay(BMA4_MS_TO_US(2), client_data->device.intf_ptr);

	err = bma4_set_fifo_config(BMA4_FIFO_ACCEL, client_data->fifo_acc_enable, &client_data->device);
	if (err)
		GSE_ERR("set acc_fifo_enable failed!\n");

	bma4xy_i2c_delay(BMA4_MS_TO_US(5), client_data->device.intf_ptr);

	return 0;
}

static void bma4xy_irq_work_func(struct work_struct *work) {
	struct bma4xy_client_data *client_data = container_of(work, struct bma4xy_client_data, irq_work);
	struct sensor_event event;
	unsigned char int_status[2] = {0, 0};
	int err = 0;

	if (tilt_flag == true) {
		GSE_LOG("tilt notify!\n");
		event.flush_action = DATA_ACTION;
		event.handle = ID_TILT_DETECTOR;
		event.word[0] = 1;
		sensor_input_event(tilt_device.minor, &event);
	}

	err = client_data->device.bus_read(BMA4_INT_STAT_0_ADDR, int_status, 2, client_data->device.intf_ptr);
	if (err) {
		GSE_ERR("bus_read stat reg failed!\n");
		return;
	}

	GSE_LOG("%s: = 0x%X, 0x%X\n", __func__, int_status[0], int_status[1]);

	return;
}

static void bma4xy_delay_sigmo_work_func(struct work_struct *work) {
	struct bma4xy_client_data *client_data =
	container_of(work, struct bma4xy_client_data,
	delay_work_sig.work);
	unsigned char int_status[2] = {0, 0};
	int err = 0;

	err = client_data->device.bus_read(BMA4_INT_STAT_0_ADDR, int_status, 2, client_data->device.intf_ptr);
	if (err)
		return;

	GSE_LOG("int_status0 = %X int_status1 = %X", int_status[0], int_status[1]);

	if ((int_status[0] & SIG_MOTION_OUT) == 0x01)
		step_notify(TYPE_SIGNIFICANT);
}

static irqreturn_t bma4xy_irq_handle(int irq, void *handle) {
	struct bma4xy_client_data *client_data = handle;
	schedule_work(&client_data->irq_work);

	return IRQ_HANDLED;
}

static int bma4xy_request_irq(struct bma4xy_client_data *client_data) {
	int err = 0;
	struct device_node *node = NULL;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,gse_1-eint");
	if (node) {
		client_data->IRQ = irq_of_parse_and_map(node, 0);
		ret = request_irq(client_data->IRQ, bma4xy_irq_handle, IRQF_TRIGGER_RISING, SENSOR_NAME, client_data);
			if (ret > 0)
				GSE_LOG("request_irq failed!\n");
			else
				GSE_LOG("request_irq success!\n");
	} else {
		GSE_LOG("can not find irq!\n");
	}

	GSE_LOG("gpio_pin = %d, irq_num = %d\n", client_data->gpio_pin, client_data->IRQ);
	INIT_WORK(&client_data->irq_work, bma4xy_irq_work_func);
	INIT_DELAYED_WORK(&client_data->delay_work_sig, bma4xy_delay_sigmo_work_func);

	return err;
}

static int bma4xy_init_client(struct i2c_client *client, int reset_cali) {
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();
	res = BMA4XY_CheckDeviceID(client);
	if (res) {
		GSE_ERR("CheckDeviceID failed!\n");
		return res;
	}

	res = BMA4XY_SetDataFormat(client, BMA4_ACCEL_RANGE_4G);
	if (res) {
		GSE_ERR("SetDataFormat failed!");
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = BMA4XY_SetPowerMode(client, 0);
	if (res) {
		GSE_ERR("SetPowerMode failed!\n");
		return res;
	}

	if (0 != reset_cali) {
		res = BMA4XY_ResetCalibration(client);
		if (res) {
			GSE_ERR("ResetCalibration failed!\n");
			return res;
		}
	}

	res = bma4xy_load_config_stream(obj);
	if (res)
		GSE_ERR("init failed after loading config_stream!\n");

	GSE_LOG("bma4xy_init_client OK!\n");

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	char strbuf[BMA4XY_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA4XY_ReadChipInfo(client, strbuf, BMA4XY_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	char strbuf[BMA4XY_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	BMA4XY_ReadSensorData(client, strbuf, BMA4XY_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int err, len = 0, mul;
	int tmp[BMA4XY_ACC_AXIS_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	err = BMA4XY_ReadOffset(client, obj->offset);
	if (0 != err)
		return -EINVAL;

	err = BMA4XY_ReadCalibration(client, tmp);
	if (0 != err)
		return -EINVAL;

	mul = obj->reso->sensitivity / bma4xy_acc_offset_resolution.sensitivity;

	len += snprintf(buf + len, PAGE_SIZE - len, "[HW ][%d] (%+3d, %+3d, %+3d): (0x%02X, 0x%02X, 0x%02X)\n", mul,
		obj->offset[BMA4XY_ACC_AXIS_X], obj->offset[BMA4XY_ACC_AXIS_Y], obj->offset[BMA4XY_ACC_AXIS_Z],
		obj->offset[BMA4XY_ACC_AXIS_X], obj->offset[BMA4XY_ACC_AXIS_Y], obj->offset[BMA4XY_ACC_AXIS_Z]);

	len += snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		obj->cali_sw[BMA4XY_ACC_AXIS_X], obj->cali_sw[BMA4XY_ACC_AXIS_Y], obj->cali_sw[BMA4XY_ACC_AXIS_Z]);

	len += snprintf(buf + len, PAGE_SIZE - len, "[ALL] (%+3d, %+3d, %+3d): (%+3d, %+3d, %+3d)\n",
		obj->offset[BMA4XY_ACC_AXIS_X] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_X],
		obj->offset[BMA4XY_ACC_AXIS_Y] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Y],
		obj->offset[BMA4XY_ACC_AXIS_Z] * mul + obj->cali_sw[BMA4XY_ACC_AXIS_Z],
		tmp[BMA4XY_ACC_AXIS_X], tmp[BMA4XY_ACC_AXIS_Y], tmp[BMA4XY_ACC_AXIS_Z]);

	return len;
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count) {
	struct i2c_client *client = bma4xy_i2c_client;
	int err, x, y, z;
	int dat[BMA4XY_ACC_AXIS_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = BMA4XY_ResetCalibration(client);
		if (0 != err)
			GSE_ERR("reset offset error = %d\n", err);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[BMA4XY_ACC_AXIS_X] = x;
		dat[BMA4XY_ACC_AXIS_Y] = y;
		dat[BMA4XY_ACC_AXIS_Z] = z;

		err = BMA4XY_WriteCalibration(client, dat);
		if (0 != err)
			GSE_ERR("write calibration error = %d\n", err);
	} else {
		GSE_ERR("invalid format!\n");
	}

	return count;
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count) {
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf) {
	ssize_t res;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count) {
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!\n");
		return 0;
	}

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: %s, length = %d\n", buf, (int)count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf) {
	ssize_t len = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!\n");
		return 0;
	}

	if (obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d, %d (%d, %d)\n",
		obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id,
		obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf) {
	unsigned char acc_op_mode;
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma4_get_accel_enable(&acc_op_mode, &client_data->device);
	if (err) {
		GSE_ERR("read failed!\n");
		return err;
	}

	if (sensor_power)
		GSE_LOG("sensor is in work mode, sensor_power = %d\n",
		sensor_power);
	else
		GSE_LOG("sensor is in standby mode, sensor_power = %d\n",
		sensor_power);

	return snprintf(buf, PAGE_SIZE, "%X\n", acc_op_mode);
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *pbBuf) {
	ssize_t _tLength = 0;

	GSE_LOG("default direction: %d\n", hw->direction);
	_tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", hw->direction);

	return _tLength;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *pbBuf, size_t tCount) {
	int _nDirection = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *_pt_i2c_obj = i2c_get_clientdata(client);

	if (NULL == _pt_i2c_obj)
		return 0;

	if (!kstrtoint(pbBuf, 10, &_nDirection)) {
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			GSE_ERR("error: failed to set direction!\n");
	}

	GSE_LOG("set direction: %d\n", _nDirection);

	return tCount;
}

static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(chip_id, S_IRUGO, bma4xy_show_chip_id, NULL);
static DRIVER_ATTR(acc_value, S_IRUGO, bma4xy_show_acc_value, NULL);
static DRIVER_ATTR(acc_range, S_IWUSR | S_IRUGO, bma4xy_show_acc_range, bma4xy_store_acc_range);
static DRIVER_ATTR(acc_odr, S_IWUSR | S_IRUGO, bma4xy_show_acc_odr, bma4xy_store_acc_odr);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, bma4xy_show_selftest, bma4xy_store_selftest);
static DRIVER_ATTR(avail_sensor, S_IRUGO, bma4xy_show_avail_sensor, NULL);
static DRIVER_ATTR(load_fw, S_IWUSR | S_IRUGO, bma4xy_show_load_config_stream, bma4xy_store_load_config_stream);
static DRIVER_ATTR(reg_val, S_IWUSR | S_IRUGO, bma4xy_show_reg_val, bma4xy_store_reg_val);
static DRIVER_ATTR(foc, S_IWUSR | S_IRUGO, bma4xy_show_foc, bma4xy_store_foc);
static DRIVER_ATTR(acc_op_mode, S_IWUSR | S_IRUGO, bma4xy_show_acc_op_mode, bma4xy_store_acc_op_mode);
static DRIVER_ATTR(reg_sel, S_IWUSR | S_IRUGO, bma4xy_show_reg_sel, bma4xy_store_reg_sel);
static DRIVER_ATTR(driver_version, S_IRUGO, bma4xy_show_driver_version, NULL);
static DRIVER_ATTR(config_file_version, S_IRUGO, bma4xy_show_config_file_version, NULL);
static DRIVER_ATTR(config_function, S_IWUSR | S_IRUGO, bma4xy_show_config_function, bma4xy_store_config_function);
static DRIVER_ATTR(dump_regs, S_IRUGO, bma4xy_dump_regs_function, NULL);

static struct driver_attribute *bma4xy_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_cali,
	&driver_attr_firlen,
	&driver_attr_trace,
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_orientation,
	&driver_attr_chip_id,
	&driver_attr_acc_op_mode,
	&driver_attr_acc_value,
	&driver_attr_acc_range,
	&driver_attr_acc_odr,
	&driver_attr_selftest,
	&driver_attr_avail_sensor,
	&driver_attr_foc,
	&driver_attr_driver_version,
	&driver_attr_load_fw,
	&driver_attr_reg_sel,
	&driver_attr_reg_val,
	&driver_attr_config_function,
	&driver_attr_dump_regs,
	&driver_attr_config_file_version,
};

static int bma4xy_create_attr(struct device_driver *driver) {
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, bma4xy_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n",
			bma4xy_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int bma4xy_delete_attr(struct device_driver *driver) {
	int idx, err = 0;
	int num = (int)(sizeof(bma4xy_attr_list) / sizeof(bma4xy_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, bma4xy_attr_list[idx]);

	return err;
}

static int gsensor_open_report_data(int open) {
	return 0;
}

static int gsensor_enable_nodata(int en) {
	int err = 0;

	if (((en == 0) && (sensor_power == false)) ||
		((en == 1) && (sensor_power == true))) {
		enable_status = sensor_power;
		GSE_LOG("sensor device has been updated!\n");
	} else {
		enable_status = !sensor_power;
		if (atomic_read(&bma4_obj_i2c_data->suspend) == 0) {
			err = BMA4XY_SetPowerMode(bma4_obj_i2c_data->client, enable_status);
			GSE_LOG("sensor not in suspend mode, status: = %d\n", enable_status);
		} else {
			GSE_LOG("sensor in suspend mode, status: = %d\n", enable_status);
		}
	}

	if (err) {
		GSE_ERR("gsensor_enable_nodata failed!\n");
		return ERRNUM1;
	}

	GSE_LOG("gsensor_enable_nodata OK!\n");

	return 0;
}

static int gsensor_set_delay(u64 ns) {
	int err = 0;
	int value;
	int sample_delay;

	value = (int)ns / 1000 / 1000;
	if (value <= 5)
		sample_delay = BMA4_OUTPUT_DATA_RATE_200HZ;
	else if (value <= 10)
		sample_delay = BMA4_OUTPUT_DATA_RATE_100HZ;
	else
		sample_delay = BMA4_OUTPUT_DATA_RATE_100HZ;

	err = BMA4XY_SetBWRate(bma4_obj_i2c_data->client, sample_delay);
	if (err) {
		GSE_ERR("set delay parameter error!\n");
		return ERRNUM1;
	}

	if (value >= 50)
		atomic_set(&bma4_obj_i2c_data->filter, 0);

	GSE_LOG("gsensor_set_delay (%d)\n", value);

	return 0;
}

static int gsensor_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs) {
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;
	GSE_LOG("bma acc set delay = (%d)\n", value);

	return gsensor_set_delay(samplingPeriodNs);
}

static int gsensor_acc_flush(void) {
	return acc_flush_report();
}

static int gsensor_get_data(int *x, int *y, int *z, int *status) {
	char buff[BMA4XY_BUFSIZE];
	int ret;

	mutex_lock(&gsensor_mutex);
	BMA4XY_ReadSensorData(bma4_obj_i2c_data->client, buff, BMA4XY_BUFSIZE);
	mutex_unlock(&gsensor_mutex);

	ret = sscanf(buff, "%x %x %x", x, y, z);
	if (ret != 3) {
		GSE_ERR("invalid argument");
		return -EINVAL;
	}

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static struct acc_init_info bma4xy_init_info = {
	.name = BMA4XY_DEV_NAME,
	.init = bma4xy_local_init,
	.uninit = bma4xy_remove,
};

static int bma4xy_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms) {
	int err;

	err = BMA4XY_SetPowerMode(bma4_obj_i2c_data->client, 1);
	if (err) {
		GSE_ERR("enable sensor failed!\n");
		return -1;
	}

	err = gsensor_acc_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("enable set batch failed!\n");
		return -1;
	}

	return 0;
}

static int bma4xy_factory_get_data(int32_t data[3], int *status) {
	return gsensor_get_data(&data[0], &data[1], &data[2], status);
}

static int bma4xy_factory_get_raw_data(int32_t data[3]) {
	char strbuf[BMA4XY_BUFSIZE] = {0};

	BMA4XY_ReadRawData(bma4_obj_i2c_data->client, strbuf);
	data[0] = strbuf[0];
	data[1] = strbuf[1];
	data[2] = strbuf[2];

	return 0;
}

static int bma4xy_factory_enable_calibration(void) {
	return 0;
}

static int bma4xy_factory_clear_cali(void) {
	int err = 0;
	err = BMA4XY_ResetCalibration(bma4_obj_i2c_data->client);
	return 0;
}

static int bma4xy_factory_set_cali(int32_t data[3]) {
	int err = 0;
	int cali[3] = {0};

	cali[BMA4XY_ACC_AXIS_X] = data[0] * bma4_obj_i2c_data->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA4XY_ACC_AXIS_Y] = data[1] * bma4_obj_i2c_data->reso->sensitivity / GRAVITY_EARTH_1000;
	cali[BMA4XY_ACC_AXIS_Z] = data[2] * bma4_obj_i2c_data->reso->sensitivity / GRAVITY_EARTH_1000;
	err = BMA4XY_WriteCalibration(bma4_obj_i2c_data->client, cali);

	if (err) {
		GSE_ERR("bma_WriteCalibration failed!\n");
		return -1;
	}

	return 0;
}

static int bma4xy_factory_get_cali(int32_t data[3]) {
	int err = 0;
	int cali[3] = {0};

	err = BMA4XY_ReadCalibration(bma4_obj_i2c_data->client, cali);
	if (err) {
		GSE_ERR("bmi160_ReadCalibration failed!\n");
		return -1;
	}

	data[0] = cali[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / bma4_obj_i2c_data->reso->sensitivity;
	data[1] = cali[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / bma4_obj_i2c_data->reso->sensitivity;
	data[2] = cali[BMA4XY_ACC_AXIS_X] * GRAVITY_EARTH_1000 / bma4_obj_i2c_data->reso->sensitivity;

	return 0;
}

static int bma4xy_factory_do_self_test(void) {
	return 0;
}

static struct accel_factory_fops bma4xy_factory_fops = {
	.enable_sensor = bma4xy_factory_enable_sensor,
	.get_data = bma4xy_factory_get_data,
	.get_raw_data = bma4xy_factory_get_raw_data,
	.enable_calibration = bma4xy_factory_enable_calibration,
	.clear_cali = bma4xy_factory_clear_cali,
	.set_cali = bma4xy_factory_set_cali,
	.get_cali = bma4xy_factory_get_cali,
	.do_self_test = bma4xy_factory_do_self_test,
};

static struct accel_factory_public bma4xy_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &bma4xy_factory_fops,
};

static int bma4xy_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct i2c_client *new_client;
	struct device_node *node = NULL;
	struct bma4xy_client_data *client_data = NULL;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	int err = 0;
	u32 remap_direction[] = {0};

	GSE_FUN();
	dev_addr = BMA4_I2C_ADDR_PRIMARY;
	node = of_find_compatible_node(NULL, NULL, "mediatek,bma4xy");

	hw = get_accel_dts_func(node, hw);
	if (hw == NULL) {
		GSE_ERR("get dts info failed!\n");
		err = -EFAULT;
		goto exit_err_clean;
	}

	err = of_property_read_u32_array(node, "remap_direction", remap_direction, ARRAY_SIZE(remap_direction));
	if (err == 0)
		remap_dir = remap_direction[0];

	client_data = kzalloc(sizeof(struct bma4xy_client_data), GFP_KERNEL);
	if (client_data == NULL) {
		GSE_ERR("no memory available!\n");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	memset(client_data, 0, sizeof(struct bma4xy_client_data));
	client_data->device.intf = BMA4_I2C_INTF;
	client_data->device.bus_read = bma4xy_i2c_read_wrapper;
	client_data->device.bus_write = bma4xy_i2c_write_wrapper;
	client_data->device.delay_us = bma4xy_i2c_delay;
	client_data->device.intf_ptr = &dev_addr;
	client_data->device.variant = BMA42X_VARIANT;
	client_data->device.read_write_len = 32;
	client_data->hw = hw;

	err = hwmsen_get_convert(client_data->hw->direction, &client_data->cvt);
	if (0 != err) {
		GSE_ERR("invalid direction: %d\n", client_data->hw->direction);
		goto exit_err_clean;
	}

	client->addr = *hw->i2c_addr;
	bma4_obj_i2c_data = client_data;
	client_data->client = client;
	client_data->client->addr = BMA4_I2C_ADDR_PRIMARY;
	new_client = client_data->client;
	i2c_set_clientdata(new_client, client_data);
	atomic_set(&client_data->trace, 0);
	atomic_set(&client_data->suspend, 0);
	bma4xy_i2c_client = new_client;

	err = bma4xy_check_chip_id(client_data);
	if (!err) {
		GSE_LOG("Bosch Sensortec device %s detected\n", SENSOR_NAME);
	} else {
		GSE_ERR("Bosch Sensortec device not found, chip id mismatch!\n");
		goto exit_err_clean;
	}

	err = bma423_init(&client_data->device);
	if (err)
		GSE_ERR("init failed!\n");

	err = bma4_set_command_register(0xB6, &client_data->device);
	if (!err)
		GSE_LOG("reset chip\n");

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);

	err = bma423_map_interrupt(0, (0x0001 << 3), 1, &client_data->device);
	if (err < 0)
		GSE_ERR("request irq failed!\n");

	wake_lock_init(&client_data->wakelock, WAKE_LOCK_SUSPEND, "bma4xy");

	err = bma4xy_init_client(new_client, 1);
	if (err)
		GSE_ERR("bma4xy_device init client failed!\n");

	err = accel_factory_device_register(&bma4xy_factory_device);
	if (err) {
		GSE_ERR("bma4xy_device register failed!\n");
		goto exit_err_clean;
	}

	err = bma4xy_create_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute error = %d\n", err);
		goto exit_err_clean;
	}

	ctl.open_report_data = gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay = gsensor_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;
	ctl.batch = gsensor_acc_batch;
	ctl.flush = gsensor_acc_flush;

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path error!\n");
		goto exit_err_clean;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;

	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path error!\n");
		goto exit_err_clean;
	}

	err = bma4xy_request_irq(client_data);
	if (err < 0)
		GSE_ERR("request irq failed!\n");

	gsensor_init_flag = 0;
	GSE_LOG("%s: OK\n", __func__);

	bma4xy_tilt_local_init();

	return 0;

exit_err_clean:
	if (err) {
		bma4xy_i2c_client = NULL;
		if (client_data != NULL)
			kfree(client_data);

		bma4_obj_i2c_data = NULL;
		return err;
	}

	return err;
}

static int bma4xy_i2c_remove(struct i2c_client *client) {
	int err = 0;

	err = bma4xy_delete_attr(&bma4xy_init_info.platform_diver_addr->driver);
	if (err != 0)
		GSE_ERR("bma4xy_delete_attr failed: %d\n", err);

	accel_factory_device_deregister(&bma4xy_factory_device);
	if (0 != err)
		GSE_ERR("acc_deregister failed: %d\n", err);

	bma4xy_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;

}

static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor",},
	{ }
};
MODULE_DEVICE_TABLE(i2c, accel_of_match);

static int bma4xy_i2c_suspend(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	GSE_FUN();
	enable_irq_wake(client_data->IRQ);
	atomic_set(&client_data->in_suspend, 1);

	return 0;
}

static int bma4xy_i2c_resume(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	GSE_FUN();
	disable_irq_wake(client_data->IRQ);
	atomic_set(&client_data->in_suspend, 0);

	return 0;
}

static const struct dev_pm_ops bma4xy_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bma4xy_i2c_suspend, bma4xy_i2c_resume)
};

static struct i2c_driver bma4xy_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.of_match_table = accel_of_match,
		.pm = &bma4xy_pm_ops,
	},
	.probe = bma4xy_i2c_probe,
	.remove = bma4xy_i2c_remove,
	.id_table = bma4xy_i2c_id,
};

static int bma4xy_local_init(void) {
	GSE_FUN();
	BMA4XY_power(hw, 1);

	if (i2c_add_driver(&bma4xy_i2c_driver)) {
		GSE_ERR("add driver error!\n");
		return ERRNUM1;
	}

	if (gsensor_init_flag == -1)
		return ERRNUM1;

	return 0;
}

static int bma4xy_remove(void) {
	GSE_FUN();
	BMA4XY_power(hw, 0);
	i2c_del_driver(&bma4xy_i2c_driver);

	return 0;
}

static int bma4xy_step_c_open_report_data(int open) {
	return 0;
}

static int bma4xy_step_c_set_delay(u64 delay) {
	return 0;
}

static int bma4xy_setp_d_set_selay(u64 delay) {
	return 0;
}

static int bma4xy_step_c_enable_nodata(int en) {
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	STEP_C_LOG("bma4xy_step_c_enable_nodata = %d\n", en);

	if (en == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	if (bma423_feature_enable(BMA423_STEP_CNTR, en, &client_data->device) < 0) {
		STEP_C_PR_ERR("set bma423 virtual error!\n");
		return -EINVAL;
	}

	if ((en == 0) && (bma4_obj_i2c_data->sigmotion_enable == 0) &&
		(bma4_obj_i2c_data->stepdet_enable == 0) && (bma4_obj_i2c_data->acc_pm == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	bma4_obj_i2c_data->stepcounter_enable = en;

	return err;
}

static int bma4xy_step_c_enable_significant(int en) {
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	STEP_C_LOG("bma4xy_step_c_enable_significant = %d", en);
	if (en == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	if ((en == 0) && (bma4_obj_i2c_data->stepcounter_enable == 0) &&
		(bma4_obj_i2c_data->stepdet_enable == 0) &&
		(bma4_obj_i2c_data->acc_pm == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	bma4_obj_i2c_data->sigmotion_enable = en;

	return err;

}

static int bma4xy_step_c_enable_step_detect(int enable) {
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	STEP_C_LOG("bma4xy_step_c_enable_step_detect = %d", enable);

	if (enable == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);

	if (bma423_step_detector_enable(enable, &client_data->device) < 0) {
		STEP_C_PR_ERR("set bma423 virtual error!\n");
		return -EINVAL;
	}

	if ((enable == 0) && (bma4_obj_i2c_data->sigmotion_enable == 0) &&
		(bma4_obj_i2c_data->stepcounter_enable == 0) && (bma4_obj_i2c_data->wakeup_enable == 0) &&
		(bma4_obj_i2c_data->tilt_enable == 0) && (bma4_obj_i2c_data->wrist_wear == 0) &&
		(bma4_obj_i2c_data->single_tap == 0) && (bma4_obj_i2c_data->double_tap == 0) &&
		(bma4_obj_i2c_data->acc_pm == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		STEP_C_PR_ERR("set acc_op_mode failed!\n");

	bma4_obj_i2c_data->stepdet_enable = enable;

	return err;
}

static int bma4xy_step_c_get_data(u32 *value, int *status) {
	int err = 0;
	uint32_t step_counter_val = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	err = bma423_step_counter_output(
	&step_counter_val, &client_data->device);

	if (err) {
		GSE_ERR("read failed!\n");
		return err;
	}

	*value = step_counter_val;
	*status = 1;
	STEP_C_LOG("step_c_get_data = %d\n", (int)(*value));

	return err;
}

static int bma4xy_stc_get_data_significant(u32 *value, int *status) {
	return 0;
}

static int bma4xy_stc_get_data_step_d(u32 *value, int *status) {
	return 0;
}

static int bma4xy_floor_set_delay(u64 ns) {
	return 0;
}

static int bma4xy_floor_enable(int en) {
	return 0;
}

static int bma4xy_get_data_floor(uint32_t *value, int *status) {
	return 0;
}

static int bma4xy_step_c_probe(void) {
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};
	int err = 0;

	GSE_FUN();
	ctl.open_report_data = bma4xy_step_c_open_report_data;
	ctl.enable_nodata = bma4xy_step_c_enable_nodata;
	ctl.enable_step_detect = bma4xy_step_c_enable_step_detect;
	ctl.enable_significant = bma4xy_step_c_enable_significant;
	ctl.step_c_set_delay = bma4xy_step_c_set_delay;
	ctl.step_d_set_delay = bma4xy_setp_d_set_selay;
	ctl.is_report_input_direct = false;
	ctl.is_counter_support_batch = false;
	ctl.is_detector_support_batch = false;
	ctl.is_smd_support_batch = false;
	ctl.is_report_input_direct = false;
	ctl.enable_floor_c = bma4xy_floor_enable;
	ctl.floor_c_set_delay = bma4xy_floor_set_delay;
	ctl.floor_c_batch = false;
	ctl.floor_c_flush = false;

	err = step_c_register_control_path(&ctl);
	if (err) {
		STEP_C_PR_ERR("step_c_register_control_path failed = %d\n", err);
		goto exit;
	}

	data.get_data = bma4xy_step_c_get_data;
	data.vender_div = 1000;
	data.get_data_significant = bma4xy_stc_get_data_significant;
	data.get_data_step_d = bma4xy_stc_get_data_step_d;
	data.get_data_floor_c = bma4xy_get_data_floor;

	err = step_c_register_data_path(&data);
	if (err) {
		STEP_C_PR_ERR("step_c_register_data_path failed = %d\n", err);
		goto exit;
	}

	STEP_C_LOG("%s: OK\n", __func__);

	return 0;

exit:
	STEP_C_PR_ERR("err = %d\n", err);
	return err;
}

static int bma4xy_stc_local_init(void) {
	STEP_C_FUN("bma4xy_stc_local_init\n");

	if (bma4_obj_i2c_data == NULL) {
		STEP_C_FUN("bma4xy_acc_init_error!\n");
		return -ENODEV;
	}

	if (bma4xy_step_c_probe()) {
		STEP_C_PR_ERR("failed to register bma4xy step_c driver!\n");
		return -ENODEV;
	}

	return 0;
}

static int bma4xy_stc_remove(void) {
	STEP_C_FUN();
	return 0;
}

static struct step_c_init_info bma4xy_stc_init_info = {
	.name = "bma4xy_step_counter",
	.init = bma4xy_stc_local_init,
	.uninit = bma4xy_stc_remove,
};

static int bma4xy_tilt_enable(int en) {
	int err = 0;
	struct i2c_client *client = bma4xy_i2c_client;
	struct bma4xy_client_data *client_data = i2c_get_clientdata(client);

	GSE_LOG("%s = %d\n", __func__, en);

	if (en == 1) {
		err = bma4_set_advance_power_save(0, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
		err += bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	}

	if (err)
		GSE_ERR("set acc_op_mode failed!");

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);

	if (bma423_feature_enable(BMA423_WRIST_WEAR, en, &client_data->device) < 0) {
		GSE_ERR("set bma423 virtual error!");
		return -EINVAL;
	}

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);

	if ((en == 0) && (bma4_obj_i2c_data->sigmotion_enable == 0) &&
		(bma4_obj_i2c_data->stepdet_enable == 0) && (bma4_obj_i2c_data->stepcounter_enable == 0) &&
		(bma4_obj_i2c_data->wakeup_enable == 0) && (bma4_obj_i2c_data->single_tap == 0) &&
		(bma4_obj_i2c_data->double_tap == 0) && (bma4_obj_i2c_data->acc_pm == 0))
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);

	if (err)
		GSE_ERR("set acc_op_mode failed!");

	bma4xy_i2c_delay(BMA4_MS_TO_US(10), client_data->device.intf_ptr);
	bma4_obj_i2c_data->wrist_wear = en;
	bma4_obj_i2c_data->tilt_enable = en;

	return err;
}

static ssize_t tilt_store_active(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count) {
	int enable = 0, handle = 0;

	sscanf(buf, "%d, %d", &handle, &enable);
	if (enable) {
		bma4xy_tilt_enable(1);
		tilt_flag = true;
	} else {
		bma4xy_tilt_enable(0);
		tilt_flag = false;
	}

	return count;
}

static ssize_t tilt_show_active(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", tilt_flag);
}

static ssize_t tilt_no_support_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported");
}

static DEVICE_ATTR(tilt_active, 0644, tilt_show_active, tilt_store_active);
static DEVICE_ATTR(tilt_batch, 0444, tilt_no_support_show, NULL);
static DEVICE_ATTR(tilt_flush, 0444, tilt_no_support_show, NULL);

static struct attribute *tilt_attributes[] = {
	&dev_attr_tilt_active.attr,
	&dev_attr_tilt_batch.attr,
	&dev_attr_tilt_flush.attr,
	NULL,
};

static struct attribute_group tilt_attribute_group = {
	.attrs = tilt_attributes
};

static int tilt_open(struct inode *inode, struct file *file) {
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t tilt_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos) {
	return sensor_event_read(tilt_device.minor, file, buffer, count, ppos);
}

static unsigned int tilt_poll(struct file *file, poll_table *wait) {
	return sensor_event_poll(tilt_device.minor, file, wait);
}

static const struct file_operations tilt_fops = {
	.owner = THIS_MODULE,
	.open = tilt_open,
	.read = tilt_read,
	.poll = tilt_poll,
};

struct sensor_attr_t tilt_device = {
	.minor = ID_TILT_DETECTOR,
	.name = TILT_MISC_DEV_NAME,
	.fops = &tilt_fops,
};

static int bma4xy_tilt_local_init(void) {
	GSE_FUN();

	if (bma4_obj_i2c_data == NULL) {
		GSE_ERR("bma4_obj_i2c_data failed!\n");
		return -ENODEV;
	}

	if (sensor_attr_register(&tilt_device)) {
		GSE_ERR("sensor_attr_register failed!\n");
		return -ENODEV;
	}

	if (sysfs_create_group(&tilt_device.this_device->kobj, &tilt_attribute_group)) {
		GSE_ERR("sysfs_create_group failed!\n");
		return -ENODEV;
	}

	return 0;
}

static int __init BMA4xy_init(void) {
	GSE_FUN();
	acc_driver_add(&bma4xy_init_info);
	step_c_driver_add(&bma4xy_stc_init_info);

	return 0;
}

static void __exit BMA4xy_exit(void) {
	GSE_FUN();
}

module_init(BMA4xy_init);
module_exit(BMA4xy_exit);
