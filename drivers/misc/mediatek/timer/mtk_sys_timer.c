/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <mt-plat/sync_write.h>

#if defined(CONFIG_MTK_SYS_TIMER_TIMESYNC) && !defined(CONFIG_FPGA_EARLY_PORTING)

#define SYS_TIMER_CLK_RATE           (13000000)
#define SYS_TIMER_MAX_SEC            (5000)
#define SYS_TIMER_REGULAR_SYNC_SEC   (60 * 60 * HZ)

static const char               sys_timer_node_name[] = "mediatek,sys_timer";
static struct workqueue_struct *sys_timer_workqueue;

struct sys_timer_timesync_context_t {
	void __iomem *sysram_base;
	struct work_struct work;
	struct timer_list timer;
	spinlock_t lock;
	u32 mult;
	u32 shift;
	u64 base_tick;
	u64 base_ts;
	u8 enabled;
	u8 base_fz;
};

static struct sys_timer_timesync_context_t timesync_cxt;

#define TIMESYNC_BASE_TICK      (0)
#define TIMESYNC_BASE_TS        (8)
#define TIMESYNC_BASE_FREEZE    (16)

#define sys_timer_sysram_write(val, addr)     mt_reg_sync_writel(val, addr)

void sys_timer_timesync_update_base(int suspended)
{
	u64 tick, ts;
	unsigned long flags = 0;

	if (!timesync_cxt.enabled)
		return;

	spin_lock_irqsave(&timesync_cxt.lock, flags);

	ts = sched_clock_get_cyc(&tick);

	sys_timer_sysram_write((tick >> 32) & 0xFFFFFFFF,
		timesync_cxt.sysram_base + TIMESYNC_BASE_TICK);
	sys_timer_sysram_write(tick & 0xFFFFFFFF,
		timesync_cxt.sysram_base + TIMESYNC_BASE_TICK + 4);

	sys_timer_sysram_write((ts >> 32) & 0xFFFFFFFF,
		timesync_cxt.sysram_base + TIMESYNC_BASE_TS);
	sys_timer_sysram_write(ts & 0xFFFFFFFF,
		timesync_cxt.sysram_base + TIMESYNC_BASE_TS + 4);

	sys_timer_sysram_write(suspended,
		timesync_cxt.sysram_base + TIMESYNC_BASE_FREEZE);

	timesync_cxt.base_tick = tick;
	timesync_cxt.base_ts = ts;

	spin_unlock_irqrestore(&timesync_cxt.lock, flags);

	pr_info("update base: ts=%llu, tick=0x%llx, fz=%d\n",
		ts, tick, suspended);
}

static inline u64 notrace cyc_to_ns(u64 cyc, u32 mult, u32 shift)
{
	return (cyc * mult) >> shift;
}

u64 sys_timer_tick_to_sched_clock(u64 tick)
{
	u64 ret;
	unsigned long flags = 0;

	if (tick < timesync_cxt.base_tick)
		return 0;

	spin_lock_irqsave(&timesync_cxt.lock, flags);

	if (timesync_cxt.base_fz)
		ret = timesync_cxt.base_ts;
	else {
		ret = cyc_to_ns(tick - timesync_cxt.base_tick,
			timesync_cxt.mult, timesync_cxt.shift) +
			timesync_cxt.base_ts;
	}

	spin_unlock_irqrestore(&timesync_cxt.lock, flags);

	return ret;
}

static void sys_timer_timesync_ws(struct work_struct *ws)
{
	sys_timer_timesync_update_base(0);
}

static void sys_timer_timesync_timeout(unsigned long data)
{
	queue_work(sys_timer_workqueue, &(timesync_cxt.work));

	timesync_cxt.timer.expires = jiffies + SYS_TIMER_REGULAR_SYNC_SEC;
	add_timer(&(timesync_cxt.timer));
}

static void __init sys_timer_of_init(void)
{
	struct device_node *node = NULL;
	int sysram_size = -1;

	/* get sys timer node */

	node = of_find_compatible_node(NULL, NULL, sys_timer_node_name);

	if (!node) {
		pr_info("node '%s' not found\n", sys_timer_node_name);
		return;
	}

	/* get sysram base */

	timesync_cxt.sysram_base = of_iomap(node, 1);

	if (!timesync_cxt.sysram_base)
		return;

	/* get sysram size */

	if (of_property_read_u32(node, "mediatek,sysram-size", &sysram_size))
		return;

	/* check if we have enough sysram size */

	if (sysram_size < 20) {
		pr_info("not enough sram size %d\n", sysram_size);
		return;
	}

	timesync_cxt.enabled = 1;
}

static void __init sys_timer_timesync_init(void)
{
	clocks_calc_mult_shift(&timesync_cxt.mult, &timesync_cxt.shift,
		SYS_TIMER_CLK_RATE, NSEC_PER_SEC, SYS_TIMER_MAX_SEC);

	pr_info("mult=%u, shift=%u, maxsec=%u\n",
		timesync_cxt.mult, timesync_cxt.shift, SYS_TIMER_MAX_SEC);
}

static int __init sys_timer_work_init(void)
{
	if (!timesync_cxt.enabled)
		return -1;

	sys_timer_workqueue = create_workqueue("sys_timer_wq");

	if (!sys_timer_workqueue) {
		pr_info("workqueue create failed\n");
		return -1;
	}

	INIT_WORK(&(timesync_cxt.work), sys_timer_timesync_ws);
	setup_timer(&(timesync_cxt.timer), &sys_timer_timesync_timeout, 0);
	timesync_cxt.timer.expires = jiffies + SYS_TIMER_REGULAR_SYNC_SEC;
	add_timer(&(timesync_cxt.timer));

	return 0;
}

static int __init sys_timer_device_init(void)
{
	spin_lock_init(&timesync_cxt.lock);

	sys_timer_of_init();

	sys_timer_timesync_init();

	/* add the 1st base */
	sys_timer_timesync_update_base(0);

	sys_timer_work_init();

	pr_info("enabled: %d\n", timesync_cxt.enabled);

	return 0;
}

#else

static int __init sys_timer_device_init(void)
{
	return 0;
}

#endif

device_initcall(sys_timer_device_init);

