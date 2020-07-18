#include "lcm_drv.h"

#define LCM_NAME "ST7797_400x400_dsi_vdo_lcm"

#define FRAME_WIDTH (400)
#define FRAME_HEIGHT (400)

#define REGFLAG_DELAY 0xFE
#define REGFLAG_END_OF_TABLE 0xFF

static LCM_UTIL_FUNCS lcm_util = {0};

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_init_setting[] = {
	{0xF0, 0x01, {0xC3}},
	{0xF0, 0x01, {0x96}},
	{0xED, 0x01, {0xC3}},
	{0xE4, 0x02, {0x40, 0x0F}},
	{0xE7, 0x01, {0x83}},
	{0xC3, 0x04, {0x33, 0x02, 0x25, 0x04}},
	{0xE5, 0x0E, {0xB2, 0xF5, 0xBD, 0x24, 0x22, 0x25, 0x10, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22}},
	{0xEC, 0x07, {0x00, 0x55, 0x00, 0x00, 0x00, 0x49, 0x22}},
	{0x36, 0x01, {0x0C}},
	{0xB2, 0x01, {0x00}},
	{0x3A, 0x01, {0x07}},
	{0xC5, 0x01, {0x6B}},
	{0xE0, 0x0E, {0x88, 0x0B, 0x10, 0x08, 0x07, 0x03, 0x2C, 0x33, 0x43, 0x08, 0x16, 0x16, 0x2A, 0x2E}},
	{0xE1, 0x0E, {0x88, 0x0B, 0x10, 0x08, 0x06, 0x02, 0x2B, 0x32, 0x42, 0x09, 0x16, 0x15, 0x2A, 0x2E}},
	{0xA4, 0x02, {0xC0, 0x7B}},
	{0xD9, 0x01, {0x02}},
	{0xB6, 0x02, {0xC7, 0x31}},
	{0xB3, 0x01, {0x01}},
	{0xC1, 0x04, {0x77, 0x07, 0xC2, 0x07}},
	{0xA5, 0x09, {0x00, 0x00, 0x00, 0x00, 0x20, 0x16, 0x2A, 0x8A, 0x02}},
	{0xBA, 0x07, {0x0A, 0x5A, 0x23, 0x10, 0x25, 0x02, 0x00}},
	{0xBB, 0x08, {0x00, 0x27, 0x00, 0x29, 0x82, 0x87, 0x18, 0x00}},
	{0xA6, 0x09, {0x00, 0x00, 0x00, 0x00, 0x20, 0x16, 0x2A, 0x8A, 0x02}},
	{0xBC, 0x08, {0x00, 0x27, 0x00, 0x29, 0x82, 0x87, 0x18, 0x00}},
	{0xBD, 0x0B, {0xA1, 0xB2, 0x2B, 0x1A, 0x56, 0x43, 0x34, 0x65, 0xFF, 0xFF, 0x0F}},
	{0xF0, 0x01, {0x3C}},
	{0xF0, 0x01, {0x69}},
	{0x21, 0x00, {0x00}},
	{0x11, 0x00, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0x00, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0x00, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{0x10, 0x00, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update) {
	unsigned int i;

	for (i = 0; i < count; i++) {
		switch (table[i].cmd) {
			case REGFLAG_DELAY:
				lcm_util.mdelay(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				lcm_util.dsi_set_cmdq_V2(table[i].cmd, table[i].count, table[i].para_list, force_update);
				break;
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util) {
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params) {
	memset(params, 0, sizeof(LCM_PARAMS));
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.packet_size = 256;
	params->dsi.vertical_sync_active = 3;
	params->dsi.vertical_backporch = 4;
	params->dsi.vertical_frontporch = 4;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 40;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.PLL_CLOCK = 150;
	params->dsi.ssc_disable = 1;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.word_count = 960;
}

static void lcm_init(void) {
	lcm_util.set_reset_pin(0);
	lcm_util.mdelay(200);
	lcm_util.set_reset_pin(1);
	lcm_util.mdelay(200);
	push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void) {
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void) {
	lcm_init();
}

LCM_DRIVER ST7797_400x400_dsi_vdo_lcm_drv =  {
	.name = LCM_NAME,
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.resume = lcm_resume,
	.suspend = lcm_suspend,
};
