#define DEFAULT_LED_STEP 18
#define LED_CTRL_EXPO_TIME_HI_BOUND 496
#define LED_CTRL_EXPO_TIME_LOW_BOUND 32
#define LED_CTRL_EXPO_TIME_HI 420
#define LED_CTRL_EXPO_TIME_LOW 64
#define LED_CURRENT_HI 31
#define LED_CURRENT_LOW 1
#define LED_INC_DEC_STEP 2
#define STATE_COUNT_TH 3

#define DEV_HEART_RATE "m_hrs_misc"
#define PAH8001_NAME "pixart_pah8001"
#define PAH8001_ADDR 0x33

static struct sensor_attr_t pixart_device;
static int led_step = DEFAULT_LED_STEP, led_change_flag = 0;
static int state = 0, state_count = 0, frame_count = 0, start = 0, stop = 0;

typedef struct {
	uint8_t HRD_Data[13];
	float MEMS_Data[3];
} ppg_mems_data_t;

typedef struct {
	struct i2c_client *client;
	struct delayed_work x_work;
	ppg_mems_data_t ppg_mems_data;
	bool run_ppg;
} pah8001_data_t;

static pah8001_data_t pah8001data;

static const unsigned char init_ppg_array[][2] = {
	{0x7F, 0x00},
	{0x09, 0x5A},
	{0x05, 0x99},
	{0x17, 0xA2},
	{0x27, 0xFF},
	{0x28, 0xFA},
	{0x29, 0x0A},
	{0x2A, 0xC8},
	{0x2B, 0xA0},
	{0x2C, 0x8C},
	{0x2D, 0x64},
	{0x42, 0x20},
	{0x48, 0x00},
	{0x4D, 0x1A},
	{0x7A, 0xB5},
	{0x7F, 0x01},
	{0x07, 0x48},
	{0x23, 0x3C},
	{0x26, 0x0F},
	{0x2E, 0x48},
	{0x38, 0xF2},
	{0x42, 0xA4},
	{0x43, 0x41},
	{0x44, 0x41},
	{0x45, 0x24},
	{0x46, 0xC0},
	{0x52, 0x32},
	{0x53, 0x28},
	{0x56, 0x60},
	{0x57, 0x28},
	{0x6D, 0x02},
	{0x0F, 0xC8},
	{0x7F, 0x00},
	{0x5D, 0x81},
};
