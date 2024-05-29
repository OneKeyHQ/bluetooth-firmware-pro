#ifndef _AXP216_H_
#define _AXP216_H_

#include "pmu_common.h"
// #include "axp_supply.h"

// defines
#define AXP216_I2C_ADDR           (0x34)

#define AXP216_STATUS             (0x00)
#define AXP216_MODE_CHGSTATUS     (0x01)
#define AXP216_IC_TYPE            (0x03)
#define AXP216_BUFFER1            (0x04)
#define AXP216_BUFFER2            (0x05)
#define AXP216_BUFFER3            (0x06)
#define AXP216_BUFFER4            (0x07)
#define AXP216_BUFFER5            (0x08)
#define AXP216_BUFFER6            (0x09)
#define AXP216_BUFFER7            (0x0A)
#define AXP216_BUFFER8            (0x0B)
#define AXP216_BUFFER9            (0x0C)
#define AXP216_BUFFERA            (0x0D)
#define AXP216_BUFFERB            (0x0E)
#define AXP216_BUFFERC            (0x0F)
#define AXP216_IPS_SET            (0x30)
#define AXP216_VOFF_SET           (0x31)
#define AXP216_OFF_CTL            (0x32)
#define AXP216_CHARGE1            (0x33)
#define AXP216_CHARGE2            (0x34)
#define AXP216_CHARGE3            (0x35)
#define AXP216_POK_SET            (0x36)
#define AXP216_VLTF_CHG           (0x38)
#define AXP216_VHTF_CHG           (0x39)
#define AXP216_VLTF_DISCHG        (0x3C)
#define AXP216_VHTF_DISCHG        (0x3D)
#define AXP216_INTEN1             (0x40)
#define AXP216_INTEN2             (0x41)
#define AXP216_INTEN3             (0x42)
#define AXP216_INTEN4             (0x43)
#define AXP216_INTEN5             (0x44)
#define AXP216_INTSTS1            (0x48)
#define AXP216_INTSTS2            (0x49)
#define AXP216_INTSTS3            (0x4A)
#define AXP216_INTSTS4            (0x4B)
#define AXP216_INTSTS5            (0x4C)
#define AXP216_LDO_DC_EN1         (0X10)
#define AXP216_LDO_DC_EN2         (0X12)
#define AXP216_LDO_DC_EN3         (0X13)
#define AXP216_DLDO1OUT_VOL       (0x15)
#define AXP216_DLDO2OUT_VOL       (0x16)
#define AXP216_DLDO3OUT_VOL       (0x17)
#define AXP216_DLDO4OUT_VOL       (0x18)
#define AXP216_ELDO1OUT_VOL       (0x19)
#define AXP216_ELDO2OUT_VOL       (0x1A)
#define AXP216_ELDO3OUT_VOL       (0x1B)
#define AXP216_DC5LDOOUT_VOL      (0x1C)
#define AXP216_DC1OUT_VOL         (0x21)
#define AXP216_DC2OUT_VOL         (0x22)
#define AXP216_DC3OUT_VOL         (0x23)
#define AXP216_DC4OUT_VOL         (0x24)
#define AXP216_DC5OUT_VOL         (0x25)
#define AXP216_GPIO0LDOOUT_VOL    (0x91)
#define AXP216_GPIO1LDOOUT_VOL    (0x93)
#define AXP216_ALDO1OUT_VOL       (0x28)
#define AXP216_ALDO2OUT_VOL       (0x29)
#define AXP216_ALDO3OUT_VOL       (0x2A)
#define AXP216_OFFLEVEL_DELAY     (0x37)
#define AXP216_DCDC_FREQSET       (0x3B)
#define AXP216_DCDC_MODESET       (0x80)
#define AXP216_ADC_EN             (0x82)
#define AXP216_ADC_CONTROL3       (0x84)
#define AXP216_INTTEMPH           (0x56)
#define AXP216_INTTEMPL           (0x57)
#define AXP216_VTSH_RES           (0x58)
#define AXP216_VTSL_RES           (0x59)
#define AXP216_OCVBATH_RES        (0xBC)
#define AXP216_OCVBATL_RES        (0xBD)
#define AXP216_VBATH_RES          (0x78) // 电池电压高位寄存器
#define AXP216_VBATL_RES          (0x79) // 电池电压低位寄存器
#define AXP216_CCBATH_RES         (0X7A) // 电池充电电流高位寄存器
#define AXP216_CCBATL_RES         (0X7B) // 电池充电电流低位寄存器
#define AXP216_DCBATH_RES         (0X7C) // 电池放电电流高位寄存器
#define AXP216_DCBATL_RES         (0X7D) // 电池放电电流低位寄存器
#define AXP216_HOTOVER_CTL        (0x8F)
#define AXP216_GPIO0_CTL          (0x90)
#define AXP216_GPIO1_CTL          (0x92)
#define AXP216_GPIO01_SIGNAL      (0x94)
#define AXP216_BAT_CHGCOULOMB3    (0xB0)
#define AXP216_BAT_CHGCOULOMB2    (0xB1)
#define AXP216_BAT_CHGCOULOMB1    (0xB2)
#define AXP216_BAT_CHGCOULOMB0    (0xB3)
#define AXP216_BAT_DISCHGCOULOMB3 (0xB4)
#define AXP216_BAT_DISCHGCOULOMB2 (0xB5)
#define AXP216_BAT_DISCHGCOULOMB1 (0xB6)
#define AXP216_BAT_DISCHGCOULOMB0 (0xB7)
#define AXP216_COULOMB_CTL        (0xB8)

#define AXP216_BAT_LEVEL          (0xB9)
#define AXP216_BAT_CAP0           (0xe0)
#define AXP216_BAT_CAP1           (0xe1)
#define AXP216_BAT_WARN           (0xe6)

// irqs

#define IRQ_SHORT_PRESS  0x10
#define IRQ_LONG_PRESS   0x08
#define IRQ_OFF_LEVEL    0x04
#define IRQ_RISING_EDGE  0x40
#define IRQ_FALLING_EDGE 0x20

Power_Error_t axp216_init(void);
Power_Error_t axp216_deinit(void);
Power_Error_t axp216_reset(void);
Power_Error_t axp216_config(void);
Power_Error_t axp216_irq(void);
Power_Error_t axp216_set_state(const Power_State_t state);
Power_Error_t axp216_get_state(Power_State_t* state);
Power_Error_t axp216_get_status(Power_Status_t* status);
Power_Error_t axp216_set_feature(Power_Featrue_t feature, bool enable);
Power_Error_t axp216_get_feature(Power_Featrue_t feature, bool* enable);

void axp216_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p);

#endif //_AXP216_H_