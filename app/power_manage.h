#ifndef __POWER_MANAGE_H_
#define __POWER_MANAGE_H_
#include "axp_config.h"
#include "axp_mfd_216.h"
#include "axp_supply.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#define AXP_DCDC1  0x02
#define AXP_DCDC2  0x04
#define AXP_DCDC3  0x08
#define AXP_DCDC4  0x10
#define AXP_DCDC5  0x20

//Charge type 
#define AXP_CHARGE_TYPE_USB       0x01
#define AXP_CHARGE_TYPE_WIRELESS  0x02

//IRQ status
#define IRQ_VBUS_INSERT 0x48
#define IRQ_VBUS_REMOVE 0x24

#define IRQ_CHARGING_BAT 0x01
#define IRQ_CHARGE_OVER  0x02

#define IRQ_LOW_BAT_1    0x01
#define IRQ_LOW_BAT_2    0x02

#define IRQ_SHORT_PRESS  0x10
#define IRQ_LONG_PRESS   0x08
#define IRQ_OFF_LEVEL    0x04
#define IRQ_RISING_EDGE  0x40
#define IRQ_FALLING_EDGE 0x20



#define AXP_CLOSE_EMMC   0x00
#define AXP_OPEN_EMMC    0x20

#define AXP_CLOSE_BL     0x00
#define AXP_OPEN_BL      0x80

extern ret_code_t usr_power_init(void);

extern ret_code_t open_all_power(void);

extern void close_all_power(void);

extern void ctl_emmc_power(uint8_t value);

extern uint8_t get_battery_percent(void);

extern uint8_t get_charge_status(void);  

extern uint8_t get_charge_type(void);  //get the charging type when charging

extern void get_battery_cv_msg(uint8_t bat_reg_addr,uint8_t bat_value[2]);

extern uint8_t get_irq_vbus_status(void);

extern uint8_t get_irq_charge_status(void);

extern uint8_t get_irq_battery_status(void);


extern uint8_t get_irq_status(void);

extern void test_dcdc(void);

extern void set_wakeup_irq(uint8_t set_value);

extern void clear_irq_reg(void);

#endif
