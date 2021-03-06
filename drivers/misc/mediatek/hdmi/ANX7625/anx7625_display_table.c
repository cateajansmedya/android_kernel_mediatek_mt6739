/******************************************************************************
*
Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V0.1

Filename :

Project  : ANX7625

Created  : 20 Sept. 2016

Devices  : ANX7625

Toolchain: Keil

Description:

Revision History:

******************************************************************************/


#ifndef NULL
#define NULL ((void *) 0L)
#endif

#include  "MI2_REG.h"
#include  "anx7625_display.h"

const unsigned char PPS_4K[] = { /*VC707 (DPI+DSC)*/
	0x11, 0x00, 0x00, 0x89, 0x10, 0x80, 0x08, 0x70,
	0x0f, 0x00, 0x00, 0x08, 0x07, 0x80, 0x07, 0x80,
	0x02, 0x00, 0x04, 0xc0, 0x00, 0x20, 0x01, 0x1e,
	0x00, 0x1a, 0x00, 0x0c, 0x0d, 0xb7, 0x03, 0x94,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00,
	0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
	0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8,
	0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char PPS_1080P[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x04, 0x38,
	0x07, 0x80, 0x00, 0x08, 0x03, 0xc0, 0x03, 0xc0,
	0x02, 0x00, 0x02, 0xe0, 0x00, 0x20, 0x00, 0xed,
	0x00, 0x0d, 0x00, 0x0c, 0x0d, 0xb7, 0x07, 0x27,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00,
	0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
	0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8,
	0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const struct RegisterValueConfig Bit_Matrix[] = {
	{TX_P2, AUDIO_CONTROL_REGISTER, 0x80},
	{TX_P2, VIDEO_BIT_MATRIX_12, 0x18},
	{TX_P2, VIDEO_BIT_MATRIX_13, 0x19},
	{TX_P2, VIDEO_BIT_MATRIX_14, 0x1a},
	{TX_P2, VIDEO_BIT_MATRIX_15, 0x1b},
	{TX_P2, VIDEO_BIT_MATRIX_16, 0x1c},
	{TX_P2, VIDEO_BIT_MATRIX_17, 0x1d},
	{TX_P2, VIDEO_BIT_MATRIX_18, 0x1e},
	{TX_P2, VIDEO_BIT_MATRIX_19, 0x1f},
	{TX_P2, VIDEO_BIT_MATRIX_20, 0x20},
	{TX_P2, VIDEO_BIT_MATRIX_21, 0x21},
	{TX_P2, VIDEO_BIT_MATRIX_22, 0x22},
	{TX_P2, VIDEO_BIT_MATRIX_23, 0x23},
	{0x00, 0x00, 0x00}
};

const struct RegisterValueConfig Regvalue_Cust[] = {
	{RX_P2, 0xa1, 0x01},
	{RX_P2, 0xa2, 0x02},
	{RX_P2, 0xa3, 0x03},
	{RX_P2, 0xa4, 0x04},
	{RX_P2, 0xa6, 0x05},
	{RX_P2, 0xa6, 0x06},
	{RX_P2, 0xa7, 0x07},
	{RX_P2, 0xa8, 0x08},
	{0x00, 0x00, 0x00}
};
const struct MIPI_Video_Format mipi_video_timing_table[] = {
	/*	lane_count--pixel_clk-----M---N--div- */
	/*	-diff--compr--3d--table--custom0--custom1*/
	/*	original timing */
	/*	total-H active-Vtotal-V active-HFP-HSW-HBP-VFP-VSW-VBP*/
	/*	decompressed timing */
	/*	tota-H active-Vtotal-V active-HFP-HSW-HBP-VFP-VSW-VBP*/
	{
		0, "720x480@60", 3, 27000000, 0xC00000, 0x100000, 0x0B,
			0x3B, 0, VIDEO_3D_NONE, NULL, Bit_Matrix, NULL,
		{ { 858, 720, 525, 480, 16, 60, 62, 10, 6, 29 } }
	},
	{
		1, "1280X720P@60", 3, 74250000, 0xB00000, 0x080000, 0x07,
			0x3A, 0, VIDEO_3D_NONE, NULL, Bit_Matrix, NULL,
		{ { 1650, 1280,  750,  720,	110,	 40,  220, 5, 5, 20 } }
	},
	{
		2, "1920x1080p@30", 3, 74000000, 0x940000, 0x06C000, 0x07,
			0x3B, 0, VIDEO_3D_NONE, NULL, Bit_Matrix, NULL,
		{ { 2200,  1920, 1125, 1080, 88, 44, 148, 4, 5, 36  } }
	},
	{
		3, "1920x1080p@60", 3, 148500000, 0xB00000, 0x080000, 0x03,
			0x37, 0, VIDEO_3D_NONE, NULL,  Bit_Matrix, NULL,
		{ { 2200, 1920, 1125, 1080, 88, 44, 148, 4, 5, 36 } }
	},
	/*MTK 4K24 DPI*/
	{
		4, "3840x2160@24",  3, 297000000, 0xB00000, 0x080000, 0x01,
			0x37, 3, VIDEO_3D_NONE, PPS_4K, Bit_Matrix, NULL,
		{	{ 1650, 1280,  2250, 2160, 242, 30, 98, 8, 10, 72 },
			{ 4950, 3840,  2250, 2160, 726, 90, 294, 8, 10, 72 }
		}
	},
	/*MTK 4K30 DPI*/
	{
		5, "3840x2160@30", 3, 297000000, 0xB00000, 0x080000, 0x01,
			0x37, 3, VIDEO_3D_NONE, PPS_4K, Bit_Matrix, NULL,
		{	{ 1474, 1280, 2250, 2160, 66,  30, 98,  8, 10, 72 },
			{ 4422, 3840, 2250, 2160, 198, 90, 294,  8, 10, 72 }
		}
	},

	{/*DSI*/
		6, "720x480@60", 3, 27000000, 0xC00000, 0x100000, 0x0B,
			0x3B,  0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 858, 720, 525, 480, 16, 60, 62, 10, 6, 29 } }
	},
	{/*DSI*/
		7, "1280X720P@60", 3, 74250000,	0xB00000, 0x080000, 0x07,
			0x3A, 0,  VIDEO_3D_NONE, NULL,   NULL, NULL,
		{ { 1650,	1280, 750, 720, 110, 40, 220, 5, 5, 20 } }
	},
	{/*DSI*/
		8, "1920x1080p@30", 3, 74250000, 0xB00000, 0x080000, 0x07,
			0x3B, 0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 2200, 1920, 1125, 1080, 88, 44, 148, 4, 5, 36 } }
	},
	{/*DSI*/
		9, "1920x1080p@60", 3, 148500000, 0xB00000, 0x080000, 0x03,
			0x37,   0,   VIDEO_3D_NONE, NULL,  NULL, NULL,
		{ { 2200, 1920, 1125, 1080, 88, 44, 148, 4, 5, 36 } }
	},

	/* 3840x2160p24  - MTK X30 -DSI*/
	{/*DSI*/
		10, "3840x2160p24", 3, 268176696, 0xAA808D, 0x089544, 0x01,
			0x37, 3, VIDEO_3D_NONE, PPS_4K, NULL, NULL,
		{	{ 1650, 1280, 2250, 2160, 242, 30, 98, 8, 10, 72 },
			{ 4950, 3840, 2250, 2160, 726, 90, 294, 8, 10, 72 }
		}
	},
	/* 3840x2160p30 3:1 DSC - MTK X30 -DSI*/
	{/*DSI*/
		11, "1280x2160p30", 3, 297000000, 0xA7B3AB, 0x07A120, 0x01,
			0x37, 3, VIDEO_3D_NONE, PPS_4K, NULL, NULL,
		{	{ 1467, 1280, 2250, 2160,  66, 30,  91, 8, 10, 72 },
			{ 4400, 3840, 2250, 2160, 198, 90, 272, 8, 10, 72 }
		}
	},

	{
		12, "********@60",	 0,		 0,	   0, 0,    0,
			0,  0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 0,   0,  0,	 0, 0,  0, 0,  0,  0,  0 } }
	},
	{
		13, "********@60",	 0,		 0,	   0, 0,    0,
			0,  0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 0,   0,  0,	 0, 0,  0, 0,  0,  0,  0 } }
	},
	{
		14, "********@60",	 0,		 0,	   0, 0,    0,
			0,  0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 0,   0,  0,	 0, 0,  0, 0,  0,  0,  0 } }
	},
	{
		15, "********@60",	 0,		 0,	   0, 0,    0,
			0,  0, VIDEO_3D_NONE, NULL, NULL, NULL,
		{ { 0,   0,  0,	 0, 0,  0, 0,  0,  0,  0 } }
	},


};

const struct AudioFormat audio_format_table[] = {
	{0,		AUDIO_I2S,      I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_48K,	 AUDIO_W_LEN_24_24MAX},
	{1,		AUDIO_I2S,     I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_441K,	 AUDIO_W_LEN_24_24MAX},
	{2,		AUDIO_I2S,     I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_32K,	 AUDIO_W_LEN_24_24MAX},
	{3,		AUDIO_I2S,     I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_882K,	 AUDIO_W_LEN_24_24MAX},
	{4,		AUDIO_I2S,	I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_96K,  AUDIO_W_LEN_24_24MAX},
	{5,		AUDIO_I2S,	I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_1764K,  AUDIO_W_LEN_24_24MAX},
	{6,		AUDIO_I2S,	I2S_CH_2,	I2S_LAYOUT_0,
		AUDIO_FS_192K,  AUDIO_W_LEN_24_24MAX},
	{7,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_48K,	 AUDIO_W_LEN_24_24MAX},
	{8,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_441K,  AUDIO_W_LEN_24_24MAX},
	{9,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_32K,	 AUDIO_W_LEN_24_24MAX},
	{10,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_882K,  AUDIO_W_LEN_24_24MAX},
	{11,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_96K,	AUDIO_W_LEN_24_24MAX},
	{12,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_1764K,  AUDIO_W_LEN_24_24MAX},
	{13,		AUDIO_TDM,	TDM_CH_8,	I2S_LAYOUT_0,
		AUDIO_FS_192K,  AUDIO_W_LEN_24_24MAX},
};

