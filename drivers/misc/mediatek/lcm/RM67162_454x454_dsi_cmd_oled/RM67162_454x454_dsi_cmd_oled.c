#include "lcm_drv.h"

#define LCM_NAME "RM67162_454x454_dsi_cmd_oled"

#define FRAME_WIDTH (454)
#define FRAME_HEIGHT (454)

#define REGFLAG_DELAY 0xD1
#define REGFLAG_END_OF_TABLE 0xD2

static LCM_UTIL_FUNCS lcm_util = {0};
extern int ambient_mode;
static int change_backlight = 1;

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_init_setting[] = {
	{0xFE, 0x01, {0x00}},
	{0x35, 0x01, {0x00}},
	{0x11, 0x01, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0x01, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_setting[] = {
	{0x51, 0x01, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28, 0x01, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{0x10, 0x01, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	{0x11, 0x01, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0x01, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_ambient_in_setting[] = {
	{0x39, 0x01, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_ambient_out_setting[] = {
	{0x38, 0x01, {0x00}},
	{REGFLAG_DELAY, 10, {}},
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
	params->dsi.mode = CMD_MODE;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.compatibility_for_nvk = 0;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.packet_size = 256;
	params->dsi.vertical_sync_active = 45;
	params->dsi.vertical_backporch = 12;
	params->dsi.vertical_frontporch = 20;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.PLL_CLOCK = 209;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
}

static void lcm_set_backlight(void* handle, unsigned int level) {
	int default_level = 45, mapped_level = 0;

	if (level > 255) {
		level = 255;
	}

	if (level) {
		if (change_backlight) {
			mapped_level = default_level + level * (255 - default_level) / (255);
			lcm_backlight_setting[0].para_list[0] = mapped_level;
			push_table(lcm_backlight_setting, sizeof(lcm_backlight_setting) / sizeof(struct LCM_setting_table), 1);
		}
	} else {
		if (ambient_mode) {
			mapped_level = 70;
		} else {
			mapped_level = 0;
		}

		lcm_backlight_setting[0].para_list[0] = mapped_level;
		push_table(lcm_backlight_setting, sizeof(lcm_backlight_setting) / sizeof(struct LCM_setting_table), 1);
	}

	printk("%s: mapped_level = %d\n", __func__, mapped_level);
}

static void lcm_init(void) {
	lcm_util.set_reset_pin(1);
	lcm_util.mdelay(10);
	lcm_util.set_reset_pin(0);
	lcm_util.mdelay(20);
	lcm_util.set_reset_pin(1);
	lcm_util.mdelay(120);
	push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void) {
	if (ambient_mode) {
		push_table(lcm_ambient_in_setting, sizeof(lcm_ambient_in_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	}

	change_backlight = 0;
}

static void lcm_resume(void) {
	if (ambient_mode) {
		push_table(lcm_ambient_out_setting, sizeof(lcm_ambient_out_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	}

	change_backlight = 1;
}

LCM_DRIVER RM67162_454x454_dsi_cmd_oled_drv = {
	.name = LCM_NAME,
	.set_backlight_cmdq = lcm_set_backlight,
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.resume = lcm_resume,
	.suspend = lcm_suspend,
};
