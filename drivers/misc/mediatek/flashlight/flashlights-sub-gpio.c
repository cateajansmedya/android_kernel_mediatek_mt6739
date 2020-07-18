/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include "flashlight.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef SUB_GPIO_DTNAME
#define SUB_GPIO_DTNAME "mediatek,flashlights_sub_gpio"
#endif

/* TODO: define driver name */
#define SUB_GPIO_NAME "flashlights-sub-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(sub_gpio_mutex);
static struct work_struct sub_gpio_work;

/* define pinctrl */
/* TODO: define pinctrl */
#define SUB_GPIO_PINCTRL_PIN_EN 0
#define SUB_GPIO_PINCTRL_PINSTATE_LOW 0
#define SUB_GPIO_PINCTRL_PINSTATE_HIGH 1
#define SUB_GPIO_PINCTRL_STATE_EN_HIGH "sub_gpio_en_high"
#define SUB_GPIO_PINCTRL_STATE_EN_LOW  "sub_gpio_en_low"
static struct pinctrl *sub_gpio_pinctrl;
static struct pinctrl_state *sub_gpio_en_high;
static struct pinctrl_state *sub_gpio_en_low;

/* define usage count */
static int use_count;


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int sub_gpio_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	sub_gpio_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sub_gpio_pinctrl)) {
		fl_pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(sub_gpio_pinctrl);
	}

	/* Flashlight en pin initialization */
	sub_gpio_en_high = pinctrl_lookup_state(sub_gpio_pinctrl, SUB_GPIO_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(sub_gpio_en_high)) {
		fl_pr_err("Failed to init (%s)\n", SUB_GPIO_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(sub_gpio_en_high);
	}
	sub_gpio_en_low = pinctrl_lookup_state(sub_gpio_pinctrl, SUB_GPIO_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(sub_gpio_en_low)) {
		fl_pr_err("Failed to init (%s)\n", SUB_GPIO_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(sub_gpio_en_low);
	}

	return ret;
}

static int sub_gpio_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(sub_gpio_pinctrl)) {
		fl_pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case SUB_GPIO_PINCTRL_PIN_EN:
		if (state == SUB_GPIO_PINCTRL_PINSTATE_LOW && !IS_ERR(sub_gpio_en_low))
			pinctrl_select_state(sub_gpio_pinctrl, sub_gpio_en_low);
		else if (state == SUB_GPIO_PINCTRL_PINSTATE_HIGH && !IS_ERR(sub_gpio_en_high))
			pinctrl_select_state(sub_gpio_pinctrl, sub_gpio_en_high);
		else
			fl_pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		fl_pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	fl_pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * sub_gpio operations
 *****************************************************************************/
/* flashlight enable function */
static int sub_gpio_enable(void)
{
	int pin = 0, state = 1;
	fl_pr_err("%s: pin(%d) state(%d)\n", __func__, pin, state);
	return sub_gpio_pinctrl_set(pin, state);
}

/* flashlight disable function */
static int sub_gpio_disable(void)
{
	int pin = 0, state = 0;
	fl_pr_err("%s: pin(%d) state(%d)\n", __func__, pin, state);
	return sub_gpio_pinctrl_set(pin, state);
}

/* set flashlight level */
static int sub_gpio_set_level(int level)
{
	int pin = 0, state = 0;
	fl_pr_err("%s: pin(%d) state(%d)\n", __func__, pin, state);
	return 0;
}

/* flashlight init */
static int sub_gpio_init(void)
{
	int pin = 0, state = 0;
	fl_pr_err("%s: pin(%d) state(%d)\n", __func__, pin, state);
	return sub_gpio_pinctrl_set(pin, state);
}

/* flashlight uninit */
static int sub_gpio_uninit(void)
{
	int pin = 0, state = 0;
	fl_pr_err("%s: pin(%d) state(%d)\n", __func__, pin, state);
	return sub_gpio_pinctrl_set(pin, state);
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer sub_gpio_timer;
static unsigned int sub_gpio_timeout_ms;

static void sub_gpio_work_disable(struct work_struct *data)
{
	fl_pr_debug("work queue callback\n");
	sub_gpio_disable();
}

static enum hrtimer_restart sub_gpio_timer_func(struct hrtimer *timer)
{
	schedule_work(&sub_gpio_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int sub_gpio_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		fl_pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		sub_gpio_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		fl_pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		sub_gpio_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		fl_pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (sub_gpio_timeout_ms) {
				ktime = ktime_set(sub_gpio_timeout_ms / 1000,
						(sub_gpio_timeout_ms % 1000) * 1000000);
				hrtimer_start(&sub_gpio_timer, ktime, HRTIMER_MODE_REL);
			}
			sub_gpio_enable();
		} else {
			sub_gpio_disable();
			hrtimer_cancel(&sub_gpio_timer);
		}
		break;
	default:
		fl_pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int sub_gpio_open(void *pArg)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int sub_gpio_release(void *pArg)
{
	/* uninit chip and clear usage count */
	mutex_lock(&sub_gpio_mutex);
	use_count--;
	if (!use_count)
		sub_gpio_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&sub_gpio_mutex);

	fl_pr_debug("Release: %d\n", use_count);

	return 0;
}

static int sub_gpio_set_driver(void)
{
	/* init chip and set usage count */
	mutex_lock(&sub_gpio_mutex);
	if (!use_count)
		sub_gpio_init();
	use_count++;
	mutex_unlock(&sub_gpio_mutex);

	fl_pr_debug("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t sub_gpio_strobe_store(struct flashlight_arg arg)
{
	sub_gpio_set_driver();
	sub_gpio_set_level(arg.level);
	sub_gpio_enable();
	msleep(arg.dur);
	sub_gpio_disable();
	sub_gpio_release(NULL);

	return 0;
}

static struct flashlight_operations sub_gpio_ops = {
	sub_gpio_open,
	sub_gpio_release,
	sub_gpio_ioctl,
	sub_gpio_strobe_store,
	sub_gpio_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int sub_gpio_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * sub_gpio_init();
	 */

	return 0;
}

static int sub_gpio_probe(struct platform_device *dev)
{
	int err;

	fl_pr_debug("Probe start.\n");

	/* init pinctrl */
	if (sub_gpio_pinctrl_init(dev)) {
		fl_pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init work queue */
	INIT_WORK(&sub_gpio_work, sub_gpio_work_disable);

	/* init timer */
	hrtimer_init(&sub_gpio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sub_gpio_timer.function = sub_gpio_timer_func;
	sub_gpio_timeout_ms = 100;

	/* init chip hw */
	sub_gpio_chip_init();

	/* register flashlight operations */
	if (flashlight_dev_register(SUB_GPIO_NAME, &sub_gpio_ops)) {
		err = -EFAULT;
		goto err;
	}

	/* clear usage count */
	use_count = 0;

	fl_pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int sub_gpio_remove(struct platform_device *dev)
{
	fl_pr_debug("Remove start.\n");

	/* flush work queue */
	flush_work(&sub_gpio_work);

	/* unregister flashlight operations */
	flashlight_dev_unregister(SUB_GPIO_NAME);

	fl_pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sub_gpio_gpio_of_match[] = {
	{.compatible = SUB_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, sub_gpio_gpio_of_match);
#else
static struct platform_device sub_gpio_gpio_platform_device[] = {
	{
		.name = SUB_GPIO_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, sub_gpio_gpio_platform_device);
#endif

static struct platform_driver sub_gpio_platform_driver = {
	.probe = sub_gpio_probe,
	.remove = sub_gpio_remove,
	.driver = {
		.name = SUB_GPIO_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sub_gpio_gpio_of_match,
#endif
	},
};

static int __init flashlight_sub_gpio_init(void)
{
	int ret;

	fl_pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&sub_gpio_gpio_platform_device);
	if (ret) {
		fl_pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&sub_gpio_platform_driver);
	if (ret) {
		fl_pr_err("Failed to register platform driver\n");
		return ret;
	}

	fl_pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_sub_gpio_exit(void)
{
	fl_pr_debug("Exit start.\n");

	platform_driver_unregister(&sub_gpio_platform_driver);

	fl_pr_debug("Exit done.\n");
}

module_init(flashlight_sub_gpio_init);
module_exit(flashlight_sub_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight SUB GPIO Driver");

