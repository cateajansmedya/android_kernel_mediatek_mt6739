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
extern LCM_DRIVER ST7797_400x400_dsi_vdo_lcm_drv;
extern LCM_DRIVER RM67162_400x400_dsi_cmd_oled_drv;

LCM_DRIVER *lcm_driver_list[] = {

#if defined(HCT_ILI9881P_DSI_VDO_HDP_PANDA_55_HZ)
	&hct_ili9881p_dsi_vdo_hdp_panda_55_hz,
#endif

#if defined(ST7797_400X400_DSI_VDO)
	&ST7797_400x400_dsi_vdo_lcm_drv,
#endif

#if defined(RM67162_400X400_DSI_CMD_OLED)
	&RM67162_400x400_dsi_cmd_oled_drv,
#endif
};

#endif
