#ifndef _POWER_MANAGE_H_
#define _POWER_MANAGE_H_

#include "pmu_common.h"
#include "pmu.h"

// defines
#define PMIC_IRQ_IO                          6
#define PMIC_PWROK_IO                        7
#define POWER_IC_CHAG_IO                     8

// stm protocol
#define BLE_CMD_KEY_STA                      0x0A
#define BLE_KEY_SHORT_PRESS                  0x01
#define BLE_KEY_LONG_PRESS                   0x02
#define BLE_KEY_PRESS                        0x20
#define BLE_KEY_RELEASE                      0x40

#define BLE_CMD_POWER_STA                    0x08
#define BLE_INSERT_POWER                     0x01
#define BLE_REMOVE_POWER                     0x02
#define BLE_CHARGING_PWR                     0x03
#define BLE_CHARGE_OVER                      0x04
#define CHARGE_TYPE_USB                      0x01
#define CHARGE_TYPE_WIRELESS                 0x02

#define BLE_CMD_POWER_ERR                    0x0B
#define BLE_CMD_POWER_ERR__NONE              0x00
#define BLE_CMD_POWER_ERR__PMU_OVER_TEMP     0x01
#define BLE_CMD_POWER_ERR__BATT_OVER_TEMP    0x02
#define BLE_CMD_POWER_ERR__BATT_UNDER_TEMP   0x03
#define BLE_CMD_POWER_ERR__BATT_OVER_VOLTAGE 0x04
#define BLE_CMD_POWER_ERR__CHARGE_TIMEOUT    0x05

// pmu handle
extern PMU_t* pmu_p;

void set_send_stm_data_p(void (*send_stm_data_p_)(uint8_t* pdata, uint8_t lenth));

bool power_manage_init();
bool power_manage_deinit();
void axp_reg_dump(uint8_t pmu_addr);
// void axp2101_brom_dump();

#endif //_POWER_MANAGE_H_