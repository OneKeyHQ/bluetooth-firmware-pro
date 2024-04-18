#ifndef __AXP216_H_
#define __AXP216_H_

#include "pmu_common.h"

// defines
#define AXP_I2C_ADDR       (0x34)

#define AXP_STATUS         (0x00)
#define AXP_MODE_CHGSTATUS (0x01)

#define AXP_IC_TYPE        (0x03)
#define AXP_LDO_DC_EN1     (0X10)
#define AXP_LDO_DC_EN2     (0X12)
#define AXP_LDO_DC_EN3     (0X13)
#define AXP_DC1OUT_VOL     (0x21)
#define AXP_ALDO1OUT_VOL   (0x28)
#define AXP_ALDO2OUT_VOL   (0x29)
#define AXP_ALDO3OUT_VOL   (0x2A)
#define AXP_VOFF_SET       (0x31)
#define AXP_OFF_CTL        (0x32)
#define AXP_CHARGE1        (0x33)
#define AXP_CHARGE2        (0x34)
#define AXP_CHARGE3        (0x35)

#define AXP_INTEN1         (0x40)
#define AXP_INTEN2         (0x41)
#define AXP_INTEN3         (0x42)
#define AXP_INTEN4         (0x43)
#define AXP_INTEN5         (0x44)
#define AXP_INTSTS1        (0x48)
#define AXP_INTSTS2        (0x49)
#define AXP_INTSTS3        (0x4A)
#define AXP_INTSTS4        (0x4B)
#define AXP_INTSTS5        (0x4C)

#define AXP_ADC_CONTROL3   (0x84)
#define AXP_INTTEMPH       (0x56)
#define AXP_INTTEMPL       (0x57)
#define AXP_VTSH_RES       (0x58)
#define AXP_VTSL_RES       (0x59)
#define AXP_OCVBATH_RES    (0xBC)
#define AXP_OCVBATL_RES    (0xBD)
#define AXP_VBATH_RES      (0x78) // 电池电压高位寄存器
#define AXP_VBATL_RES      (0x79) // 电池电压低位寄存器
#define AXP_CCBATH_RES     (0X7A) // 电池充电电流高位寄存器
#define AXP_CCBATL_RES     (0X7B) // 电池充电电流低位寄存器
#define AXP_DCBATH_RES     (0X7C) // 电池放电电流高位寄存器
#define AXP_DCBATL_RES     (0X7D) // 电池放电电流低位寄存器

#define AXP_GPIO0_CTL      (0x90)
#define AXP_GPIO1_CTL      (0x92)
#define AXP_GPIO01_SIGNAL  (0x94)

#define AXP_CAP            (0xB9)
#define AXP_BATCAP0        (0xe0)
#define AXP_BATCAP1        (0xe1)

Power_Error_t axp216_init(void);
Power_Error_t axp216_deinit(void);
Power_Error_t axp216_reset(void);
Power_Error_t axp216_config(void);
Power_Error_t axp216_irq(void);
Power_Error_t axp216_set_state(const Power_State_t state);
Power_Error_t axp216_get_state(Power_State_t* state);
Power_Error_t axp216_get_status(Power_Status_t* status);

void axp216_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p);

#endif //__AXP216_H_