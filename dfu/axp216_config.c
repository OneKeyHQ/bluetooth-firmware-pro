#include "util_micros.h"
#include "nrf_i2c.h"
#include "nrf_delay.h"

// axp216 defines

#define AXP216_I2C_ADDR     (0x34)

#define AXP216_IC_TYPE      (0x03)
#define AXP216_BUFFER1      (0x04)
#define AXP216_LDO_DC_EN1   (0X10)
#define AXP216_LDO_DC_EN2   (0X12)
#define AXP216_DC1OUT_VOL   (0x21)
#define AXP216_ALDO1OUT_VOL (0x28)
#define AXP216_ALDO2OUT_VOL (0x29)
#define AXP216_ALDO3OUT_VOL (0x2A)

// macro utils
#define EC_E_BOOL_R_BOOL(expr)     ExecuteCheck_ADV(expr, true, { return false; })

#define axp216_reg_read(reg, val)  nrf_i2c_handle->Reg.Read(AXP216_I2C_ADDR, reg, val)
#define axp216_reg_write(reg, val) nrf_i2c_handle->Reg.Write(AXP216_I2C_ADDR, reg, val)
#define axp216_set_bits(reg, mask) nrf_i2c_handle->Reg.SetBits(AXP216_I2C_ADDR, reg, mask)
#define axp216_clr_bits(reg, mask) nrf_i2c_handle->Reg.ClrBits(AXP216_I2C_ADDR, reg, mask)

I2C_t* nrf_i2c_handle = NULL;

bool axp216_minimum_config()
{
    nrf_i2c_handle = nrf_i2c_get_instance();
    uint8_t val = 0;

    // init
    EC_E_BOOL_R_BOOL(nrf_i2c_handle->Init());

    // looking for AXP216
    for ( uint8_t retry = 0; retry < 10; retry++ )
    {
        if ( !axp216_reg_read(AXP216_IC_TYPE, &val) )
        {
            nrf_delay_ms(5); // needed? i2c has it's own timeout
            continue;
        }

        // check id, as axp different ic may share same i2c addrs
        if ( val == 0x62 )
        {
            // check buffer marker?

            // voltages
            // ALDO1 -> LDO_1V8 1.8V
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
            // ALDO2 -> LDO_FB 3.3V
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
            // ALDO3 -> LDO_NF 3.3V
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
            // DCDC1 -> MAIN 3.3V
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));

            // set buffer marker?

            // power cycle
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN2, 0x20));
            EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));

            break;
        }
    }

    // deinit
    EC_E_BOOL_R_BOOL(nrf_i2c_handle->Deinit());

    return (val == 0x62);
}