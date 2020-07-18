/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


#ifndef BUILD_LK

#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#include "lcm_i2c.h"
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
	#include "cust_gpio_usage.h"
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	//#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                 (720)
#define FRAME_HEIGHT                (1440)
#define LCM_ID                      (0x9881)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)		lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0
 struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

extern int mt_dsi_pinctrl_set(unsigned int pin , unsigned int level);

static void lcm_power_5v_en(unsigned char enabled)
{
	if (enabled)
	{
	mt_dsi_pinctrl_set(LCM_POWER_ENN, 1);
	}	
	else
	{
	mt_dsi_pinctrl_set(LCM_POWER_ENN, 0);
	}
	
}


static void lcm_power_n5v_en(unsigned char enabled)
{
	if (enabled)
	{
	mt_dsi_pinctrl_set(LCM_POWER_ENP, 1);
	}	
	else
	{
	mt_dsi_pinctrl_set(LCM_POWER_ENP, 0);
	}
}

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,0x03,{0x98,0x81,0x05}},
	{0xB2,0x01,{0x70}},
	{0x03,0x01,{0x00}},
	{0x04,0x01,{0x24}},
	{0x30,0x01,{0xF7}},
	{0x29,0x01,{0x00}},
	{0x2A,0x01,{0x12}},
	{0x38,0x01,{0xA8}},
	{0x1A,0x01,{0x50}},
	{0x52,0x01,{0x5F}},
	{0x54,0x01,{0x28}},
	{0x55,0x01,{0x25}},
	{0x26,0x01,{0x02}},
	{0x3D,0x01,{0xA1}},
	{0x1B,0x01,{0x01}},

	{0xFF,0x03,{0x98,0x81,0x02}},
	{0x42,0x01,{0x2F}},
	{0x01,0x01,{0x50}},
	{0x15,0x01,{0x10}},
	{0x57,0x01,{0x00}},
	{0x58,0x01,{0x17}},
	{0x59,0x01,{0x26}},
	{0x5A,0x01,{0x14}},
	{0x5B,0x01,{0x17}},
	{0x5C,0x01,{0x29}},
	{0x5D,0x01,{0x1D}},
	{0x5E,0x01,{0x1F}},
	{0x5F,0x01,{0x8B}},
	{0x60,0x01,{0x1E}},
	{0x61,0x01,{0x2A}},
	{0x62,0x01,{0x78}},
	{0x63,0x01,{0x19}},
	{0x64,0x01,{0x17}},
	{0x65,0x01,{0x4B}},
	{0x66,0x01,{0x20}},
	{0x67,0x01,{0x27}},
	{0x68,0x01,{0x4A}},
	{0x69,0x01,{0x5A}},
	{0x6A,0x01,{0x25}},
	{0x6B,0x01,{0x00}},
	{0x6C,0x01,{0x17}},
	{0x6D,0x01,{0x26}},
	{0x6E,0x01,{0x14}},
	{0x6F,0x01,{0x17}},
	{0x70,0x01,{0x29}},
	{0x71,0x01,{0x1D}},
	{0x72,0x01,{0x1F}},
	{0x73,0x01,{0x8B}},
	{0x74,0x01,{0x1E}},
	{0x75,0x01,{0x2A}},
	{0x76,0x01,{0x78}},
	{0x77,0x01,{0x19}},
	{0x78,0x01,{0x17}},
	{0x79,0x01,{0x4B}},
	{0x7A,0x01,{0x20}},
	{0x7B,0x01,{0x27}},
	{0x7C,0x01,{0x4A}},
	{0x7D,0x01,{0x5A}},
	{0x7E,0x01,{0x25}},

	{0xFF,0x03,{0x98,0x81,0x01}},
	{0x01,0x01,{0x00}},
	{0x02,0x01,{0x00}},
	{0x03,0x01,{0x56}},
	{0x04,0x01,{0x13}},
	{0x05,0x01,{0x13}},
	{0x06,0x01,{0x0a}},
	{0x07,0x01,{0x05}},
	{0x08,0x01,{0x05}},
	{0x09,0x01,{0x1D}},
	{0x0a,0x01,{0x01}},
	{0x0b,0x01,{0x00}},
	{0x0c,0x01,{0x3F}},
	{0x0d,0x01,{0x29}},
	{0x0e,0x01,{0x29}},
	{0x0f,0x01,{0x1D}},
	{0x10,0x01,{0x1D}},
	{0x11,0x01,{0x00}},
	{0x12,0x01,{0x00}},
	{0x13,0x01,{0x08}},
	{0x14,0x01,{0x08}},
	{0x15,0x01,{0x00}},
	{0x16,0x01,{0x00}},
	{0x17,0x01,{0x00}},
	{0x18,0x01,{0x00}},
	{0x19,0x01,{0x00}},
	{0x1a,0x01,{0x00}},
	{0x1b,0x01,{0x00}},
	{0x1c,0x01,{0x00}},
	{0x1d,0x01,{0x00}},
	{0x1e,0x01,{0x40}},
	{0x1f,0x01,{0x88}},
	{0x20,0x01,{0x08}},
	{0x21,0x01,{0x01}},
	{0x22,0x01,{0x00}},
	{0x23,0x01,{0x00}},
	{0x24,0x01,{0x00}},
	{0x25,0x01,{0x00}},
	{0x26,0x01,{0x00}},
	{0x27,0x01,{0x00}},
	{0x28,0x01,{0x33}},
	{0x29,0x01,{0x03}},
	{0x2a,0x01,{0x00}},
	{0x2b,0x01,{0x00}},
	{0x2c,0x01,{0x00}},
	{0x2d,0x01,{0x00}},
	{0x2e,0x01,{0x00}},
	{0x2f,0x01,{0x00}},
	{0x30,0x01,{0x00}},
	{0x31,0x01,{0x00}},
	{0x32,0x01,{0x00}},
	{0x33,0x01,{0x00}},
	{0x34,0x01,{0x00}},
	{0x35,0x01,{0x00}},
	{0x36,0x01,{0x00}},
	{0x37,0x01,{0x00}},
	{0x38,0x01,{0x00}},
	{0x39,0x01,{0x0f}},
	{0x3a,0x01,{0x2a}},
	{0x3b,0x01,{0xc0}},
	{0x3c,0x01,{0x00}},
	{0x3d,0x01,{0x00}},
	{0x3e,0x01,{0x00}},
	{0x3f,0x01,{0x00}},
	{0x40,0x01,{0x00}},
	{0x41,0x01,{0xe0}},
	{0x42,0x01,{0x40}},
	{0x43,0x01,{0x0f}},
	{0x44,0x01,{0x31}},
	{0x45,0x01,{0xa8}},
	{0x46,0x01,{0x00}},
	{0x47,0x01,{0x08}},
	{0x48,0x01,{0x00}},
	{0x49,0x01,{0x00}},
	{0x4a,0x01,{0x00}},
	{0x4b,0x01,{0x00}},
	{0x4c,0x01,{0xb2}},
	{0x4d,0x01,{0x22}},
	{0x4e,0x01,{0x01}},
	{0x4f,0x01,{0xf7}},
	{0x50,0x01,{0x29}},
	{0x51,0x01,{0x72}},
	{0x52,0x01,{0x25}},
	{0x53,0x01,{0xb2}},
	{0x54,0x01,{0x22}},
	{0x55,0x01,{0x22}},
	{0x56,0x01,{0x22}},
	{0x57,0x01,{0xa2}},
	{0x58,0x01,{0x22}},
	{0x59,0x01,{0x01}},
	{0x5a,0x01,{0xe6}},
	{0x5b,0x01,{0x28}},
	{0x5c,0x01,{0x62}},
	{0x5d,0x01,{0x24}},
	{0x5e,0x01,{0xa2}},
	{0x5f,0x01,{0x22}},
	{0x60,0x01,{0x22}},
	{0x61,0x01,{0x22}},
	{0x62,0x01,{0xee}},
	{0x63,0x01,{0x02}},
	{0x64,0x01,{0x0b}},
	{0x65,0x01,{0x02}},
	{0x66,0x01,{0x02}},
	{0x67,0x01,{0x01}},
	{0x68,0x01,{0x00}},
	{0x69,0x01,{0x0f}},
	{0x6a,0x01,{0x07}},
	{0x6b,0x01,{0x55}},
	{0x6c,0x01,{0x02}},
	{0x6d,0x01,{0x02}},
	{0x6e,0x01,{0x5b}},
	{0x6f,0x01,{0x59}},
	{0x70,0x01,{0x02}},
	{0x71,0x01,{0x02}},
	{0x72,0x01,{0x57}},
	{0x73,0x01,{0x02}},
	{0x74,0x01,{0x02}},
	{0x75,0x01,{0x02}},
	{0x76,0x01,{0x02}},
	{0x77,0x01,{0x02}},
	{0x78,0x01,{0x02}},
	{0x79,0x01,{0x02}},
	{0x7a,0x01,{0x0a}},
	{0x7b,0x01,{0x02}},
	{0x7c,0x01,{0x02}},
	{0x7d,0x01,{0x01}},
	{0x7e,0x01,{0x00}},
	{0x7f,0x01,{0x0e}},
	{0x80,0x01,{0x06}},
	{0x81,0x01,{0x54}},
	{0x82,0x01,{0x02}},
	{0x83,0x01,{0x02}},
	{0x84,0x01,{0x5a}},
	{0x85,0x01,{0x58}},
	{0x86,0x01,{0x02}},
	{0x87,0x01,{0x02}},
	{0x88,0x01,{0x56}},
	{0x89,0x01,{0x02}},
	{0x8a,0x01,{0x02}},
	{0x8b,0x01,{0x02}},
	{0x8c,0x01,{0x02}},
	{0x8d,0x01,{0x02}},
	{0x8e,0x01,{0x02}},
	{0x8f,0x01,{0x44}},
	{0x90,0x01,{0x44}},

	{0xFF,0x03,{0x98,0x81,0x06}},
	{0x01,0x01,{0x03}},
	{0x04,0x01,{0x70}},
	{0x2B,0x01,{0x0A}},
	{0xC0,0x01,{0xCF}},
	{0xC1,0x01,{0x2A}},

	{0xFF,0x03,{0x98,0x81,0x00}},
	{0x35,0x01,{0x00}},
	{0x36,0x01,{0x00}},
	{0x11,0x01,{0x00}},
	{REGFLAG_DELAY,120, {}},
	{0x29,0x01,{0x00}},
	{REGFLAG_DELAY,20, {}},

};


//static int vcom=0x40;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
			/*case 0xd9:
			table[i].para_list[0]=vcom;
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            vcom+=2;
			break;
			*/
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE :
                break;
                
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                //MDELAY(1);
        }
    }
    
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode             = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity    = LCM_POLARITY_RISING;

	params->dsi.mode   = SYNC_EVENT_VDO_MODE;


	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine. 
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	// Highly depends on LCD driver capability.

	params->dsi.packet_size=256;
	// Video mode setting       
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->physical_width = 69;
	params->physical_height = 122;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 20;
	params->dsi.vertical_frontporch					= 20; 
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 80;
	params->dsi.horizontal_frontporch				= 80;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.HS_TRAIL = 120;
	params->dsi.PLL_CLOCK = 286;//260;	
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	params->dsi.ssc_disable=1;
}

static void lcm_init(void)
{
	lcm_power_5v_en(1);

	MDELAY(5);
	lcm_power_n5v_en(1);
	MDELAY(20);


	SET_RESET_PIN(1);
	MDELAY(5);   //10
	SET_RESET_PIN(0);
	MDELAY(20);    //20
	SET_RESET_PIN(1);
	MDELAY(120);      //120
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	unsigned int data_array[16];


	data_array[0]=0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	data_array[0]=0x00100500;

	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	SET_RESET_PIN(0);
	MDELAY(20);

}

static void lcm_resume(void)
{
	//lcm_power_5v_en(1);

	//MDELAY(5);
	//lcm_power_n5v_en(1);
//	MDELAY(20);


	SET_RESET_PIN(1);
	MDELAY(1);   //10
	SET_RESET_PIN(0);
	MDELAY(10);    //20
	SET_RESET_PIN(1);
	MDELAY(10);      //120
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//lcm_init();
}


static unsigned int lcm_compare_id(void)
{
	int   array[4];
	char  buffer[5];
	unsigned int id_high;
	unsigned int id_low;
	unsigned int id=0;

	//Do reset here
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(30);

	array[0]=0x00043902;
	array[1]=0x018198ff;
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);
	array[0]=0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x00, buffer,1);
	id_high = buffer[0]; ///////////////////////0x98
	read_reg_v2(0x01, buffer,1);
	id_low = buffer[0]; ///////////////////////0x06
	// id = (id_midd &lt;&lt; 8) | id_low;
	id = (id_high << 8) | id_low;

	#if defined(BUILD_LK)
	printf("ILI9881 %s id_high = 0x%04x, id_low = 0x%04x\n,id=0x%x\n", __func__, id_high, id_low,id);
	#else
	printk("ILI9881 %s id_high = 0x%04x, id_low = 0x%04x\n,id=0x%x\n", __func__, id_high, id_low,id);
	#endif

	return (LCM_ID == id)?1:0;

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_ili9881p_dsi_vdo_hdp_panda_55_hz = 
{
    .name           = "hct_ili9881p_dsi_vdo_hdp_panda_55_hz",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = lcm_compare_id,    
#if 0//defined(LCM_DSI_CMD_MODE)
    //.set_backlight    = lcm_setbacklight,
    //.esd_check   = lcm_esd_check, 
    //.esd_recover   = lcm_esd_recover, 
    .update         = lcm_update,
#endif  //wqtao
};

