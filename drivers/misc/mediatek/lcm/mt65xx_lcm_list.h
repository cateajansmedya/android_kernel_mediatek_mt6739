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

#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

extern LCM_DRIVER hct_ili9881p_dsi_vdo_hdp_panda_55_hz;


LCM_DRIVER *lcm_driver_list[] = {

#if defined(HCT_ILI9881P_DSI_VDO_HDP_PANDA_55_HZ)
	&hct_ili9881p_dsi_vdo_hdp_panda_55_hz,
#endif


};

#endif
