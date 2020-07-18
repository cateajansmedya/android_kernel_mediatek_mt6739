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

#if defined(CONFIG_MTK_SYS_TIMER_TIMESYNC) && !defined(CONFIG_FPGA_EARLY_PORTING)
extern u64  sys_timer_tick_to_sched_clock(u64 tick);
extern void sys_timer_timesync_update_base(int suspended);
#else
u64  sys_timer_tick_to_sched_clock(u64 tick) { return 0; };
void sys_timer_timesync_update_base(int suspended) {};
#endif

