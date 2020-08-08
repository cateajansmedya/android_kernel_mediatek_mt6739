#include <step_counter.h>
#include "nrf.h"

void nrf_notify_step(void) {
	printk("%s\n", __func__);

	if (nrf_client->step_counter) {
		step_notify(ID_STEP_COUNTER);
	}

	if (nrf_client->step_detect) {
		step_notify(ID_STEP_DETECTOR);
	}
}

static void nrf_step_set(int enable) {
	printk("%s: %d\n", __func__, enable);
	nrf_i2c_feature(nrf_i2c_client, STEP_OFFSET_1, STEP_OFFSET_2, STEP_OFFSET_3, enable ? 1 : 0);
}

static int nrf_step_enable(int enable) {
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->step_counter) {
			if (!nrf_client->step_status) {
				nrf_step_set(1);
				nrf_client->step_status = 1;
			}

			nrf_client->step_counter = 1;
		}
	} else {
		if (nrf_client->step_counter) {
			if (nrf_client->step_status && !nrf_client->step_detect) {
				nrf_step_set(0);
				nrf_client->step_status = 0;
			}

			nrf_client->step_counter = 0;
		}
	}

	return 0;
}

static int nrf_step_detect(int enable) {
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->step_detect) {
			if (!nrf_client->step_status) {
				nrf_step_set(1);
				nrf_client->step_status = 1;
			}

			nrf_client->step_detect = 1;
		}
	} else {
		if (nrf_client->step_status) {
			if (nrf_client->step_status && !nrf_client->step_counter) {
				nrf_step_set(0);
				nrf_client->step_status = 0;
			}

			nrf_client->step_detect = 0;
		}
	}

	return 0;
}

static int nrf_step_significant(int enable) {
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		if (!nrf_client->step_counter) {
			if (!nrf_client->step_status) {
				nrf_step_set(1);
				nrf_client->step_status = 1;
			}

			nrf_client->step_counter = 1;
		}
	} else {
		if (nrf_client->step_counter) {
			if (nrf_client->step_status && !nrf_client->step_detect) {
				nrf_step_set(0);
				nrf_client->step_status = 0;
			}

			nrf_client->step_counter = 0;
		}
	}

	return 0;
}

static int nrf_step_get_data(uint32_t *step, uint32_t *dist, uint32_t *cal, int *status) {
	char buffer[12];
	int step_tmp = 0, dist_tmp = 0, cal_tmp = 0;

	if (!nrf_i2c_read(nrf_i2c_client, REG_STEP, buffer, 12)) {
		step_tmp = buffer[3];
		step_tmp = (step_tmp << 8 ) | buffer[2];
		step_tmp = (step_tmp << 8 ) | buffer[1];
		step_tmp = (step_tmp << 8 ) | buffer[0];
		*step = step_tmp;

		dist_tmp = buffer[7];
		dist_tmp = (dist_tmp << 8 ) | buffer[6];
		dist_tmp = (dist_tmp << 8 ) | buffer[5];
		dist_tmp = (dist_tmp << 8 ) | buffer[4];
		*dist = dist_tmp;

		cal_tmp = buffer[11];
		cal_tmp = (cal_tmp << 8 ) | buffer[10];
		cal_tmp = (cal_tmp << 8 ) | buffer[9];
		cal_tmp = (cal_tmp << 8 ) | buffer[8];
		*cal = cal_tmp;
	}

	*status = 1;

	return 0;
}

static int nrf_generic_get_data(uint32_t *value, int *status) {
	return 0;
}

static int nrf_generic_enable(int enable) {
	return 0;
}

static int nrf_generic_set_delay(u64 delay) {
	return 0;
}

static int nrf_step_probe(void) {
	int ret = 0;
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};

	ctl.open_report_data = nrf_generic_enable;
	ctl.enable_nodata = nrf_step_enable;
	ctl.enable_step_detect = nrf_step_detect;
	ctl.enable_significant = nrf_step_significant;
	ctl.step_c_set_delay = nrf_generic_set_delay;
	ctl.step_d_set_delay = nrf_generic_set_delay;
	ctl.is_report_input_direct = 1;
	ctl.is_counter_support_batch = 1;
	ctl.is_detector_support_batch = 0;
	ctl.is_smd_support_batch = 0;
	ctl.is_report_input_direct = 0;
	ctl.enable_floor_c = nrf_generic_enable;
	ctl.floor_c_set_delay = nrf_generic_set_delay;
	ctl.floor_c_batch = 0;
	ctl.floor_c_flush = 0;

	ret = step_c_register_control_path(&ctl);
	if (ret) {
		printk("%s: step_c_register_control_path failed!\n", __func__);
		return -1;
	}

	data.get_data = nrf_step_get_data;
	data.vender_div = 1000;
	data.get_data_significant = nrf_generic_get_data;
	data.get_data_step_d = nrf_generic_get_data;
	data.get_data_floor_c = nrf_generic_get_data;

	ret = step_c_register_data_path(&data);
	if (ret) {
		printk("%s: step_c_register_data_path failed!\n", __func__);
		return -1;
	}

	return 0;
}

static int nrf_step_init(void) {
	if (nrf_client->init_status) {
		nrf_step_probe();
		return 0;
	}

	return -1;
}

static int nrf_step_remove(void) {
	return 0;
}

static struct step_c_init_info nrf_step_info = {
	.name = "nrf_step_driver",
	.init = nrf_step_init,
	.uninit = nrf_step_remove,
};

int nrf_step_register(struct i2c_client *client, struct nrf_info *data) {
	nrf_i2c_client = client;
	nrf_client = data;
	printk("%s\n", __func__);
	step_c_driver_add(&nrf_step_info);

	return 0;
}
