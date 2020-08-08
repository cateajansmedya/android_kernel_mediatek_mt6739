#include <accel.h>
#include "nrf.h"

static int nrf_acc_data(int *x, int *y, int *z, int *value) {
	unsigned char buffer[6];
	short x_new = 0, y_new = 0, z_new = 0;

	if (!nrf_i2c_read(nrf_i2c_client, REG_ACC, buffer, 6)) {
		x_new = (buffer[1] << 8) | buffer[0];
		y_new = (buffer[3] << 8) | buffer[2];
		z_new = (buffer[5] << 8) | buffer[4];

		*x = x_new;
		*y = y_new;
		*z = z_new;
	}

	*value = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int nrf_acc_enable(int enable) {
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->acc_status) {
			nrf_i2c_feature(nrf_i2c_client, ACC_OFFSET_1, ACC_OFFSET_2, ACC_OFFSET_3, 1);
			nrf_client->acc_status = 1;
		}
	} else {
		if (nrf_client->acc_status) {
			nrf_i2c_feature(nrf_i2c_client, ACC_OFFSET_1, ACC_OFFSET_2, ACC_OFFSET_3, 0);
			nrf_client->acc_status = 0;
		}
	}

	return 0;
}

static int nrf_acc_delay(u64 ns) {
	return 0;
}

static int nrf_acc_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs) {
	return nrf_acc_delay(samplingPeriodNs);
}

static int nrf_acc_flush(void) {
	return acc_flush_report();
}

static int nrf_acc_report(int open) {
	return 0;
}

static int nrf_acc_probe(void) {
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};
	int err = 0;

	ctl.open_report_data = nrf_acc_report;
	ctl.enable_nodata = nrf_acc_enable;
	ctl.set_delay = nrf_acc_delay;
	ctl.is_report_input_direct = 0;
	ctl.is_support_batch = 0;
	ctl.batch = nrf_acc_batch;
	ctl.flush = nrf_acc_flush;

	err = acc_register_control_path(&ctl);
	if (err) {
		printk("%s acc_register_control_path failed!\n", __func__);
		return -1;
	}

	data.get_data = nrf_acc_data;
	data.vender_div = 1000;

	err = acc_register_data_path(&data);
	if (err) {
		printk("%s: acc_register_data_path failed!\n", __func__);
		return -1;
	}

	return 0;
}

static int nrf_acc_init(void) {
	if (nrf_client->init_status) {
		nrf_acc_probe();
		return 0;
	}

	return -1;
}

static int nrf_acc_remove(void) {
	return 0;
}

static struct acc_init_info nrf_acc_info = {
	.name = "nrf_acc_driver",
	.init = nrf_acc_init,
	.uninit = nrf_acc_remove,
};

int nrf_acc_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);
	acc_driver_add(&nrf_acc_info);

	return 0;
}
