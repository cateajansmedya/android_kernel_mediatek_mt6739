#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include "tpd.h"

#define CTP_NAME "IT7250"
#define POWER_ON 1
#define POWER_OFF 0

static struct i2c_client *i2c_client;
static struct task_struct *thread;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int power_flag = 0, tpd_flag = 0, touch_irq = 0, flag = 0;
static int x[2] = {-1, -1}, y[2] = {-1, -1};
static bool finger[2];

static inline int it7259_i2c_read(int writelen, char *writebuf, int readlen, char *readbuf) {
	int ret = 0;
	struct i2c_msg msgs[2];

	msgs[0].addr = i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = writelen;
	msgs[0].buf = writebuf;

	msgs[1].addr = i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = readlen;
	msgs[1].buf = readbuf;

	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (ret < 0) {
		printk("[IT7259] i2c_transfer error: %d\n", ret);
		return ret;
	}

	return 0;
}

static void tpd_up(int x, int y) {
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
}

static void tpd_down(int x, int y) {
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
}

static int touch_event_handler(void *unused) {
	int ret = 0;
	unsigned char pucPoint[14], buffer[1];

	struct sched_param param = {.sched_priority = 4};
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

		buffer[0] = 0x80;
		ret = it7259_i2c_read(1, buffer, 1, &pucPoint[0]);
		if (!(pucPoint[0] & 0x80 || pucPoint[0] & 0x01)) {
			msleep(10);
			goto exit_work_func;
		}

		buffer[0] = 0xC0;
		ret = it7259_i2c_read(1, buffer, 8, &pucPoint[6]);
		buffer[0] = 0xE0;
		ret += it7259_i2c_read(1, buffer, 6, &pucPoint[0]);

		if ((ret == 0xE02) || (ret == 0x00)) {
			if (pucPoint[0] & 0xF0) {
				printk("[IT7259] gesture detected\n");
				goto exit_work_func;

			} else if (pucPoint[1] & 0x01) {
				printk("[IT7259] palm detected\n");
				goto exit_work_func;

			} else if (!(pucPoint[0] & 0x08)) {
				if (finger[0]) {
					finger[0] = 0;
					tpd_up(x[0], y[0]);
					flag = 1;
				}

				if (finger[1]) {
					finger[1] = 0;
					tpd_up(x[1], y[1]);
					flag = 1;
				}

				if (flag) {
					flag = 0;
					input_sync(tpd->dev);
				}

				printk("[IT7259] no more data available\n");
				goto exit_work_func;

			} else if (pucPoint[0] & 0x04) {
				printk("[IT7259] three fingers not supported\n");
				goto exit_work_func;

			} else {
				if (pucPoint[0] & 0x01) {
					x[0] = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
					y[0] = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
					finger[0] = 1;
					tpd_down(x[0], y[0]);
				} else if (finger[0]) {
					tpd_up(x[0], y[0]);
					finger[0] = 0;
				}

				if (pucPoint[0] & 0x02) {
					x[1] = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
					y[1] = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];
					finger[1] = 1;
					tpd_down(x[1], y[1]);
				} else if (finger[1]) {
					tpd_up(x[1], y[1]);
					finger[1] = 0;
				}

				input_sync(tpd->dev);
			}
		} else {
			printk("[IT7259] touch_event_handler error = 0x%X\n", ret);
		}

	exit_work_func:
		enable_irq(touch_irq);

	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id) {
	disable_irq_nosync(touch_irq);
	tpd_flag = 1;
	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}

static int tpd_irq_registration(void) {
	struct device_node *node;
	int ret = 0;

	node = of_find_matching_node(0, touch_of_match);
	if (node) {
		touch_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(touch_irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, TPD_DEVICE, 0);
		if (ret > 0) {
			printk("[IT7259] tpd_irq_registration failed!");
		}
	} else {
		printk("[IT7259] device node not found!");
	}

	return 0;
}

static void it7259_power_switch(int state) {
	int ret = 0;

	switch (state) {
		case POWER_ON:
			if (power_flag == POWER_OFF) {
				if (!tpd->reg) {
					tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
					if (IS_ERR(tpd->reg)) {
						printk("[IT7259] regulator_get failed!\n");
						return;
					}
				}

				ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);
				if (ret) {
					printk("[IT7259] regulator_set_voltage failed!\n");
				}

				ret = regulator_enable(tpd->reg);
				if (ret) {
					printk("[IT7259] regulator_enable failed!\n");
				}

				power_flag = POWER_ON;
			}
			break;

		case POWER_OFF:
			if (power_flag == POWER_ON) {
				if (!IS_ERR_OR_NULL(tpd->reg)) {
					ret = regulator_disable(tpd->reg);
					if (ret) {
						printk("[IT7259] regulator_disable failed!\n");
					}

					regulator_put(tpd->reg);
					tpd->reg = 0;
					power_flag = POWER_OFF;
				}
			}
			break;

		default:
			printk("[IT7259] invalid power switch command!\n");
			break;
		}
}

static int get_version(void) {
	unsigned char data[2], buffer[2];

	buffer[0] = 0x70;
	buffer[1] = 0x32;
	it7259_i2c_read(2, buffer, 1, &data[0]);

	buffer[0] = 0x70;
	buffer[1] = 0x33;
	it7259_i2c_read(2, buffer, 1, &data[1]);

	if (data[1] == 0x72 && data[0] == 0x59) {
		printk("[IT7259] found!\n");
		return 1;
	}

	printk("[IT7259] not found!\n");

	return 0;
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int retval = 0;

	i2c_client = client;
	client->addr = (0x8C >> 1);

	tpd_gpio_output(GTP_RST_PORT, 0);
	it7259_power_switch(POWER_ON);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(200);

	if (get_version()) {
		tpd_gpio_output(GTP_RST_PORT, 0);
		msleep(2);
		tpd_gpio_output(GTP_RST_PORT, 1);
		msleep(200);
	} else {
		goto error;
	}

	tpd_gpio_as_int(GTP_INT_PORT);
	msleep(50);
	tpd_irq_registration();

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		retval = PTR_ERR(thread);
		printk("[IT7259] failed to create kernel thread: %d\n", retval);
		goto error;
	}

	tpd_load_status = 1;

	return 0;

error:
	disable_irq(touch_irq);
	it7259_power_switch(POWER_OFF);
	printk("[IT7259] tpd_probe failed!\n");

	return -1;
}

static int tpd_remove(struct i2c_client *client) {
	return 0;
}

static const struct i2c_device_id it7259_tpd_id[] = {
	{CTP_NAME, 0},
	{}
};

static const struct of_device_id it7259_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
MODULE_DEVICE_TABLE(of, it7259_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = it7259_dt_match,
		.name = CTP_NAME,
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = it7259_tpd_id,
};

static int tpd_local_init(void) {
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		printk("[IT7259] unable to add i2c driver!\n");
		return -1;
	}

	return 0;
}

static void tpd_suspend(struct device *h) {
	it7259_power_switch(POWER_OFF);
	tpd_gpio_output(GTP_RST_PORT, 0);
	msleep(100);
	disable_irq(touch_irq);
}

static void tpd_resume(struct device *h) {
	it7259_power_switch(POWER_ON);
	msleep(100);
	tpd_gpio_output(GTP_RST_PORT, 1);
	msleep(100);

	enable_irq(touch_irq);
	tpd_up(0, 0);
	input_sync(tpd->dev);
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = CTP_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int __init tpd_driver_init(void) {
	printk("[IT7259] tpd_driver_init\n");
	tpd_get_dts_info();

	if (tpd_driver_add(&tpd_device_driver) < 0) {
		printk("[IT7259] tpd_driver_init failed!\n");
	}

	return 0;
}

static void __exit tpd_driver_exit(void) {
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
