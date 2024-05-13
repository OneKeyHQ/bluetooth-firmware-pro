#include "axp216_config.h"

#include "util_micros.h"
#include "nrf_i2c.h"
#include "nrf_delay.h"

#include "nrf_log.h"

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
#define AXP216_VOFF_SET     (0x31)
#define AXP216_OFF_CTL      (0x32)

// macro utils

#define EC_E_BOOL_BREAK(expr)      ExecuteCheck_ADV(expr, true, break;)

#define axp216_reg_read(reg, val)  nrf_i2c_handle->Reg.Read(AXP216_I2C_ADDR, reg, val)
#define axp216_reg_write(reg, val) nrf_i2c_handle->Reg.Write(AXP216_I2C_ADDR, reg, val)
#define axp216_set_bits(reg, mask) nrf_i2c_handle->Reg.SetBits(AXP216_I2C_ADDR, reg, mask)
#define axp216_clr_bits(reg, mask) nrf_i2c_handle->Reg.ClrBits(AXP216_I2C_ADDR, reg, mask)

static I2C_t* nrf_i2c_handle = NULL;

AXP216_CONF_R_t axp216_minimum_config()
{
    nrf_i2c_handle = nrf_i2c_get_instance();
    uint8_t val = 0;

    AXP216_CONF_R_t result = AXP216_CONF_INVALID;

    // force reset bus
    nrf_i2c_handle->Reset();

    // init
    if ( !nrf_i2c_handle->Init() )
        result = AXP216_CONF_BUS_ERR;

    if ( nrf_i2c_handle->isInitialized )
    {
        // looking for AXP216
        for ( uint8_t retry = 0; retry < 10; retry++ )
        {
            if ( !axp216_reg_read(AXP216_IC_TYPE, &val) )
            {
                result = AXP216_CONF_NO_ACK;
                nrf_delay_ms(100); // needed? i2c has it's own timeout
                continue;
            }

            // check id, as axp different ic may share same i2c addrs
            if ( val == 0x62 )
            {
                result = AXP216_CONF_FAILED;

                // enable wakeup
                // EC_E_BOOL_BREAK(axp216_set_bits(AXP216_VOFF_SET, 0b00001000));

                // check buffer marker?

                // voltages
                // ALDO1 -> LDO_1V8 1.8V
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
                // ALDO2 -> LDO_FB 3.3V
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
                // ALDO3 -> LDO_NF 3.3V
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
                // DCDC1 -> MAIN 3.3V
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));

                // set buffer marker?

                // misc
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_VOFF_SET, 0x03));
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_OFF_CTL, 0x43));

                // power cycle
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0x80)); // keep aldo2 to not go into sleep mode
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));

                // // try wakeup
                // EC_E_BOOL_BREAK(axp216_set_bits(AXP216_VOFF_SET, 0b00100000));
                // shutdown
                // EC_E_BOOL_BREAK(axp216_set_bits(AXP216_OFF_CTL, 0b10000000));

                nrf_delay_ms(100);

                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x3C));
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));

                result = AXP216_CONF_SUCCESS;
                break;
            }
        }
    }

    // deinit
    if ( !nrf_i2c_handle->Deinit() )
        result = AXP216_CONF_BUS_ERR;

    return result;
}

// static uint32_t wait_axp()
// {
//     uint32_t wait_ms = 0;
// #include "nrf_i2c.h"
//     static I2C_t* nrf_i2c_handle = NULL;
//     nrf_i2c_handle = nrf_i2c_get_instance();
//     nrf_i2c_handle->Reset();
//     nrf_i2c_handle->Init();
//     // reboot
//     while(!nrf_i2c_handle->Reg.SetBits(0x34,0x31, 0b01000000));
//     while ( !nrf_i2c_handle->Reg.SetBits(0x34,0x31, 0b01000000) )
//     {
//         nrf_delay_ms(1);
//         wait_ms++;
//     }
//     // uint8_t val = 0;
//     // while ( !nrf_i2c_handle->Reg.Read(0x34,0x03, &val) )
//     // {
//     //     nrf_delay_ms(1);
//     //     wait_ms++;
//     // }
//     nrf_i2c_handle->Deinit();
//     return wait_ms;
// }
