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

#include <linux/of_gpio.h>

#include "flashlight.h"
#include "flashlight-dt.h"

#include <linux/hct_include/hct_project_all_config.h>

/* define device tree */
/* modify temp device tree name */
#ifndef AW3641_DTNAME
#define AW3641_DTNAME "mediatek,flashlights_aw3641"
#endif

/* define driver name */
#define AW3641_NAME "flashlights-aw3641"

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(aw3641_mutex);
static struct work_struct aw3641_work;

/* define pinctrl */
#define AW3641_PINCTRL_PINSTATE_LOW 0
#define AW3641_PINCTRL_PINSTATE_HIGH 1

#define AW3641_PINCTRL_PIN_EN 0
#define AW3641_PINCTRL_PIN_MODE 1
#define AW3641_PINCTRL_STATE_EN_HIGH "aw3641_en_high"
#define AW3641_PINCTRL_STATE_EN_LOW  "aw3641_en_low"
#define AW3641_PINCTRL_STATE_MODE_HIGH "aw3641_mode_high"
#define AW3641_PINCTRL_STATE_MODE_LOW  "aw3641_mode_low"
#define AW3641_PIN_EN_GPIO		"aw3641_gpio"

#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
#define MAIN_GPIO_PINCTRL_STATE_EN_HIGH "main_gpio_en_high"
#define MAIN_GPIO_PINCTRL_STATE_EN_LOW  "main_gpio_en_low"
#endif
#endif

static struct pinctrl *aw3641_pinctrl;
static struct pinctrl_state *aw3641_en_high;
static struct pinctrl_state *aw3641_en_low;
static struct pinctrl_state *aw3641_mode_high;
static struct pinctrl_state *aw3641_mode_low;

#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
static struct pinctrl_state *main_gpio_en_high;
static struct pinctrl_state *main_gpio_en_low;
#endif
#endif

/* define usage count */
static int use_count;

static int g_duty = -1;

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
int g_gpio_aw3641_en = 0;

int hct_gpio_aw3641_en(struct device *dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	pr_info("[%s] start..\n", __func__);
	if (np) {
		ret = of_get_named_gpio(np, "aw3641_en_gpio", 0);
		if (ret < 0)
			pr_err("%s: get aw3641_en GPIO failed (%d)", __func__, ret);
		else
			g_gpio_aw3641_en = ret;
	}
	else
		return -1;

	if (gpio_is_valid(g_gpio_aw3641_en))
			pr_info("gpio number %d is valid\n", g_gpio_aw3641_en);

	if (g_gpio_aw3641_en != 0) {
		ret = gpio_request(g_gpio_aw3641_en, "aw3641_en_gpio");
		if (ret) {
			pr_err("%s : aw3641_en gpio_request failed\n", __func__);
			return -ENODEV;
		}
		pr_info("%s : aw3641_en GPIO = %d\n", __func__, g_gpio_aw3641_en);
		ret = gpio_direction_output(g_gpio_aw3641_en, 0);
		if (ret) {
			pr_err("%s : aw3641_en gpio_direction_output failed\n",  __func__);
			return -ENODEV;
		}
		gpio_set_value(g_gpio_aw3641_en, 0);
		pr_info("%s : aw3641_en gpio_get_value = %d\n", __func__, gpio_get_value(g_gpio_aw3641_en));
	}
	pr_info("[%s] end..\n", __func__);
	return 0;
}
static int aw3641_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	aw3641_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3641_pinctrl)) {
		fl_pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3641_pinctrl);
	}

	/* Flashlight en pin initialization */
	aw3641_en_high = pinctrl_lookup_state(aw3641_pinctrl, AW3641_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(aw3641_en_high)) {
		fl_pr_err("Failed to init (%s)\n", AW3641_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(aw3641_en_high);
	}
	aw3641_en_low = pinctrl_lookup_state(aw3641_pinctrl, AW3641_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(aw3641_en_low)) {
		fl_pr_err("Failed to init (%s)\n", AW3641_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(aw3641_en_low);
	}

	/* Flashlight mode pin initialization */
	aw3641_mode_high = pinctrl_lookup_state(aw3641_pinctrl, AW3641_PINCTRL_STATE_MODE_HIGH);
	if (IS_ERR(aw3641_mode_high)) {
		fl_pr_err("Failed to init (%s)\n", AW3641_PINCTRL_STATE_MODE_HIGH);
		ret = PTR_ERR(aw3641_mode_high);
	}
	aw3641_mode_low = pinctrl_lookup_state(aw3641_pinctrl, AW3641_PINCTRL_STATE_MODE_LOW);
	if (IS_ERR(aw3641_mode_low)) {
		fl_pr_err("Failed to init (%s)\n", AW3641_PINCTRL_STATE_MODE_LOW);
		ret = PTR_ERR(aw3641_mode_low);
	}

#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
	main_gpio_en_high = pinctrl_lookup_state(aw3641_pinctrl, MAIN_GPIO_PINCTRL_STATE_EN_HIGH);
	if (IS_ERR(main_gpio_en_high)) {
		fl_pr_err("Failed to init (%s)\n", MAIN_GPIO_PINCTRL_STATE_EN_HIGH);
		ret = PTR_ERR(main_gpio_en_high);
	}
	main_gpio_en_low = pinctrl_lookup_state(aw3641_pinctrl, MAIN_GPIO_PINCTRL_STATE_EN_LOW);
	if (IS_ERR(main_gpio_en_low)) {
		fl_pr_err("Failed to init (%s)\n", MAIN_GPIO_PINCTRL_STATE_EN_LOW);
		ret = PTR_ERR(main_gpio_en_low);
	}
#endif
#endif

	return ret;
}

static int aw3641_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(aw3641_pinctrl)) {
		fl_pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW3641_PINCTRL_PIN_EN:
		if (state == AW3641_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3641_en_low))
		{
			pinctrl_select_state(aw3641_pinctrl, aw3641_en_low);
			#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
			#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
			pinctrl_select_state(aw3641_pinctrl, main_gpio_en_low);
			#endif
			#endif
		}	
		else if (state == AW3641_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3641_en_high))
		{
			pinctrl_select_state(aw3641_pinctrl, aw3641_en_high);
			#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
			#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
			pinctrl_select_state(aw3641_pinctrl, main_gpio_en_high);
			#endif
			#endif
		}
		else
			fl_pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	case AW3641_PINCTRL_PIN_MODE:
		if (state == AW3641_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3641_mode_low))
		{
			pinctrl_select_state(aw3641_pinctrl, aw3641_mode_low);
			#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
			#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
			pinctrl_select_state(aw3641_pinctrl, main_gpio_en_low);
			#endif
			#endif
		}
		else if (state == AW3641_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3641_mode_high))
		{
			pinctrl_select_state(aw3641_pinctrl, aw3641_mode_high);
				#if defined(__HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__)
			#if __HCT_TRUE_FALSE_FLASHLIGHT_SUPPORT__
			pinctrl_select_state(aw3641_pinctrl, main_gpio_en_high);
			#endif
			#endif
		}	
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
 * aw3641 operations
 *****************************************************************************/
/* flashlight enable function */
static inline void aw3641e_set_flash_mode(int en){
	int i;
	aw3641_pinctrl_set(AW3641_PINCTRL_PIN_MODE, AW3641_PINCTRL_PINSTATE_HIGH);
	for(i=0; i<en; i++)
	{
		gpio_set_value(g_gpio_aw3641_en, 0);
		udelay(2);
		gpio_set_value(g_gpio_aw3641_en, 1);
		udelay(2);
	}
}

static inline void aw3641e_set_torch_mode(void){
	aw3641_pinctrl_set(AW3641_PINCTRL_PIN_MODE, AW3641_PINCTRL_PINSTATE_LOW);
	aw3641_pinctrl_set(AW3641_PINCTRL_PIN_EN, AW3641_PINCTRL_PINSTATE_HIGH);
}

static int aw3641_enable(void)
{
	fl_pr_err("%s: line(%d) g_duty(%d)\n", __func__, __LINE__, g_duty);
	if(g_duty > 0){
		aw3641e_set_flash_mode(9);
	}else{
		aw3641e_set_torch_mode();
	}
	return 0;
}

/* flashlight disable function */
static int aw3641_disable(void)
{
	fl_pr_err("%s: line(%d) g_duty(%d)\n", __func__, __LINE__, g_duty);
	return aw3641_pinctrl_set(AW3641_PINCTRL_PIN_EN, AW3641_PINCTRL_PINSTATE_LOW);
}

/* set flashlight level */
static int aw3641_set_level(int level)
{
	fl_pr_err("%s: line(%d) level(%d)\n", __func__, __LINE__, level);
	g_duty = level;
	return 0;
}

/* flashlight init */
static int aw3641_init(void)
{
	aw3641_pinctrl_set(AW3641_PINCTRL_PIN_EN, AW3641_PINCTRL_PINSTATE_LOW);;
	aw3641_pinctrl_set(AW3641_PINCTRL_PIN_MODE, AW3641_PINCTRL_PINSTATE_LOW);
	fl_pr_err("%s: line(%d) g_duty(%d)\n", __func__, __LINE__, g_duty);
	return 0;
}

/* flashlight uninit */
static int aw3641_uninit(void)
{
	fl_pr_err("%s: line(%d) g_duty(%d)\n", __func__, __LINE__, g_duty);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3641_timer;
static unsigned int aw3641_timeout_ms;

static void aw3641_work_disable(struct work_struct *data)
{
	fl_pr_debug("work queue callback\n");
	aw3641_disable();
}

static enum hrtimer_restart aw3641_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw3641_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3641_ioctl(unsigned int cmd, unsigned long arg)
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
		aw3641_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		fl_pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3641_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		fl_pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw3641_timeout_ms) {
				ktime = ktime_set(aw3641_timeout_ms / 1000,
						(aw3641_timeout_ms % 1000) * 1000000);
				hrtimer_start(&aw3641_timer, ktime, HRTIMER_MODE_REL);
			}
			aw3641_enable();
		} else {
			aw3641_disable();
			hrtimer_cancel(&aw3641_timer);
		}
		break;
	default:
		fl_pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3641_open(void *pArg)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int aw3641_release(void *pArg)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw3641_mutex);
	use_count--;
	if (!use_count)
		aw3641_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw3641_mutex);

	fl_pr_debug("Release: %d\n", use_count);

	return 0;
}

static int aw3641_set_driver(void)
{
	/* init chip and set usage count */
	mutex_lock(&aw3641_mutex);
	if (!use_count)
		aw3641_init();
	use_count++;
	mutex_unlock(&aw3641_mutex);

	fl_pr_debug("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw3641_strobe_store(struct flashlight_arg arg)
{
	aw3641_set_driver();
	aw3641_set_level(arg.level);
	aw3641_enable();
	msleep(arg.dur);
	aw3641_disable();
	aw3641_release(NULL);

	return 0;
}

static struct flashlight_operations aw3641_ops = {
	aw3641_open,
	aw3641_release,
	aw3641_ioctl,
	aw3641_strobe_store,
	aw3641_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw3641_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw3641_init();
	 */

	return 0;
}

static int aw3641_probe(struct platform_device *dev)
{
	int err;

	fl_pr_debug("Probe start.\n");

	/* init pinctrl */
	if (aw3641_pinctrl_init(dev)) {
		fl_pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	hct_gpio_aw3641_en(&dev->dev);

	/* init work queue */
	INIT_WORK(&aw3641_work, aw3641_work_disable);

	/* init timer */
	hrtimer_init(&aw3641_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3641_timer.function = aw3641_timer_func;
	aw3641_timeout_ms = 100;

	/* init chip hw */
	aw3641_chip_init();

	/* register flashlight operations */
	if (flashlight_dev_register(AW3641_NAME, &aw3641_ops)) {
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

static int aw3641_remove(struct platform_device *dev)
{
	fl_pr_debug("Remove start.\n");

	/* flush work queue */
	flush_work(&aw3641_work);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW3641_NAME);

	fl_pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3641_gpio_of_match[] = {
	{.compatible = AW3641_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641_gpio_of_match);
#else
static struct platform_device aw3641_gpio_platform_device[] = {
	{
		.name = AW3641_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3641_gpio_platform_device);
#endif

static struct platform_driver aw3641_platform_driver = {
	.probe = aw3641_probe,
	.remove = aw3641_remove,
	.driver = {
		.name = AW3641_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641_gpio_of_match,
#endif
	},
};

static int __init flashlight_aw3641_init(void)
{
	int ret;

	fl_pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641_gpio_platform_device);
	if (ret) {
		fl_pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3641_platform_driver);
	if (ret) {
		fl_pr_err("Failed to register platform driver\n");
		return ret;
	}

	fl_pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_aw3641_exit(void)
{
	fl_pr_debug("Exit start.\n");

	platform_driver_unregister(&aw3641_platform_driver);

	fl_pr_debug("Exit done.\n");
}

module_init(flashlight_aw3641_init);
module_exit(flashlight_aw3641_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight AW3641 Driver");

