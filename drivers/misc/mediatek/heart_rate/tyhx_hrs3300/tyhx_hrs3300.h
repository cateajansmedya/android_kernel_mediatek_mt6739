const uint16_t hrs3300_AMP_LTH = 120;
const uint8_t hrs3300_agc_init_stage = 0x04;
const uint8_t hrs3300_accurate_first_shot = 0;
const uint8_t hrs3300_bp_power_grade = 0;
const uint8_t hrs3300_bp_timeout_grade = 0;
const uint8_t hrs3300_up_factor = 3;
const uint8_t hrs3300_up_shift = 2;

static int hr_date = 0, up_date = 0, down_date = 0;
static int bp_enable = 0, hr_enable = 0;

static int hrs3300_power_up_flg = 0;
uint8_t reg_0x7f, reg_0x80, reg_0x81, reg_0x82;

const uint8_t init_register_array[][2] = {
	{0x0C, 0x4E},
	{0x16, 0x78},
	{0x17, 0x0D},
	{0x02, 0x80},
	{0x03, 0x00},
	{0x04, 0x00},
	{0x05, 0x00},
	{0x06, 0x00},
	{0x07, 0x00},
	{0x08, 0x74},
	{0x09, 0x00},
	{0x0A, 0x08},
	{0x0B, 0x00},
	{0x0C, 0x6E},
	{0x0D, 0x02},
	{0x0E, 0x07},
	{0x0F, 0x0F},
};

typedef enum {
	MSG_ALG_NOT_OPEN = 0x01,
	MSG_NO_TOUCH = 0x02,
	MSG_PPG_LEN_TOO_SHORT = 0x03,
	MSG_HR_READY = 0x04,
	MSG_ALG_TIMEOUT = 0x05,
	MSG_SETTLE = 0x06
} hrs3300_msg_code_t;

typedef enum {
	MSG_BP_ALG_NOT_OPEN = 0x01,
	MSG_BP_NO_TOUCH = 0x02,
	MSG_BP_PPG_LEN_TOO_SHORT = 0x03,
	MSG_BP_READY = 0x04,
	MSG_BP_ALG_TIMEOUT = 0x05,
	MSG_BP_SETTLE = 0x06
} hrs3300_bp_msg_code_t;

typedef struct hrs3300_linux_data {
	struct i2c_client *client;
	unsigned int hrs_timer_enable;
	unsigned int init_flag;
} hrs3300_linux_data_t;

typedef struct hrs3300_result_t_struct {
	hrs3300_msg_code_t alg_status;
	uint32_t data_cnt;
	uint8_t hr_result;
	uint8_t hr_result_qual;
	bool object_flg;
} hrs3300_results_t;

typedef struct hrs3300_bp_results_t_struct{
	hrs3300_bp_msg_code_t bp_alg_status;
	uint8_t sbp;
	uint8_t dbp;
	uint32_t data_cnt;
	uint8_t hr_result;
	bool object_flg;
} hrs3300_bp_results_t;

hrs3300_results_t Hrs3300_alg_get_results(void);
hrs3300_bp_results_t Hrs3300_alg_get_bp_results(void);

void Hrs3300_bp_alg_open(void);
bool Hrs3300_bp_alg_send_data(int16_t new_raw_data);
bool Hrs3300_alg_open(void);
bool Hrs3300_alg_send_data(int16_t new_raw_data, int16_t als_raw_data, int16_t gsen_data_x, int16_t gsen_data_y, int16_t gsen_data_z);
