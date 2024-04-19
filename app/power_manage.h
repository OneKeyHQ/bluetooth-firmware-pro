#ifndef _POWER_MANAGE_H_
#define _POWER_MANAGE_H_

#include "custom_board.h"
#include "pmu.h"
#include "pmu_common.h"

// defines
#define TWI_INSTANCE_ID 1
// #define TWI_SDA_M 14
// #define TWI_SCL_M 15
#define POWER_IC_IRQ_IO  6
#define POWER_IC_OK_IO   7
#define POWER_IC_CHAG_IO 8

// Charge type
#define AXP_CHARGE_TYPE_USB      0x01
#define AXP_CHARGE_TYPE_WIRELESS 0x02


#define BLE_CMD_KEY_STA     0x0A
#define BLE_KEY_LONG_PRESS  0x01
#define BLE_KEY_SHORT_PRESS 0x02

#define BLE_CMD_PWR_STA  0x0B
#define BLE_CLOSE_SYSTEM 0x01
#define BLE_CLOSE_EMMC   0x02
#define BLE_OPEN_EMMC    0x03
#define BLE_PWR_PERCENT  0X04


//
#define BLE_CMD_POWER_STA 0x08
#define BLE_INSERT_POWER  0x01
#define BLE_REMOVE_POWER  0x02
#define BLE_CHARGING_PWR  0x03
#define BLE_CHAGE_OVER    0x04

// AXP216 global status
static uint8_t g_charge_status = 0;
static uint8_t g_bas_update_flag = 0;

// pmu handle
extern PMU_t* pmu;

static uint8_t bak_buff[128];

bool power_manage_init();
bool power_manage_deinit();

#endif //_POWER_MANAGE_H_