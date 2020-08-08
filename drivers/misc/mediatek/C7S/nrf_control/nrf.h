#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "gpio_driver.h"
#include "hwmsensor.h"
#include "sensor_attr.h"
#include "sensor_event.h"

#define NRF_NAME "nrf_ctrl_driver"
#define NRF_MISC "nrf_ctrl"

#define REG_VERSION 0x01
#define REG_CHIP 0x02
#define REG_INPUT 0x03
#define REG_STATUS 0x06
#define REG_CONFIG 0x07
#define REG_HRS 0x0B
#define REG_RTC 0x0F
#define REG_STEP 0x18
#define REG_ACC 0x24
#define REG_RESET 0x3C
#define REG_MAC 0x40
#define REG_VENDOR 0x47
#define REG_REMOTE 0x49
#define REG_HEALTH 0xA0

#define HRS_OFFSET_1 0x00
#define HRS_OFFSET_2 0x01
#define HRS_OFFSET_3 0x00

#define TERM_OFFSET_1 0x00
#define TERM_OFFSET_2 0x01
#define TERM_OFFSET_3 0x06

#define SYS_OFFSET_1 0x00
#define SYS_OFFSET_2 0x01
#define SYS_OFFSET_3 0x07

#define STEP_OFFSET_1 0x01
#define STEP_OFFSET_2 0x01
#define STEP_OFFSET_3 0x00

#define TILT_OFFSET_1 0x01
#define TILT_OFFSET_2 0x01
#define TILT_OFFSET_3 0x01

#define MISC_OFFSET_1 0x01
#define MISC_OFFSET_2 0x01
#define MISC_OFFSET_3 0x03

#define LCM_OFFSET_1 0x01
#define LCM_OFFSET_2 0x01
#define LCM_OFFSET_3 0x04

#define PWR_OFFSET_1 0x01
#define PWR_OFFSET_2 0x01
#define PWR_OFFSET_3 0x07

#define ACC_OFFSET_1 0x02
#define ACC_OFFSET_2 0x01
#define ACC_OFFSET_3 0x00

#define UART_OFFSET_1 0x02
#define UART_OFFSET_2 0x01
#define UART_OFFSET_3 0x03

#define VIBR_OFFSET_1 0x02
#define VIBR_OFFSET_2 0x01
#define VIBR_OFFSET_3 0x04

#define RTC_OFFSET_1 0x03
#define RTC_OFFSET_2 0x01
#define RTC_OFFSET_3 0x00

static struct i2c_client *nrf_i2c_client;
static struct nrf_info *nrf_client;

struct nrf_info {
	int init_status;
	int acc_status;
	int ebus_status;
	int hrs_status;
	int step_status;
	int step_counter;
	int step_detect;
	int tilt_status;
	int version;
	int irq_wake;
	int irq_no_wake;
	char config[4];
};

void nrf_notify_ebus(int value);
void nrf_notify_step(void);
void nrf_notify_tilt(void);
void kpd_key_input(int key, int value);

int nrf_i2c_read(struct i2c_client *client, uint8_t addr, uint8_t *data, uint16_t len);
int nrf_i2c_write(struct i2c_client *client, uint8_t addr, uint8_t *data, uint16_t len);
int nrf_i2c_feature(struct i2c_client *client, uint8_t pos, uint8_t mask, uint8_t offset, uint8_t value);

int nrf_acc_register(struct i2c_client *client, struct nrf_info *data);
int nrf_ebus_register(struct i2c_client *client, struct nrf_info *data);
int nrf_hrs_register(struct i2c_client *client, struct nrf_info *data);
int nrf_misc_register(struct i2c_client *client, struct nrf_info *data);
int nrf_step_register(struct i2c_client *client, struct nrf_info *data);
int nrf_tilt_register(struct i2c_client *client, struct nrf_info *data);
int nrf_vibr_register(struct i2c_client *client, struct nrf_info *data);
