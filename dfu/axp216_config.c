#include "axp216_config.h"

#include "util_macros.h"
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
#define AXP216_VOFF_SET     (0x31)
#define AXP216_OFF_CTL      (0x32)

// axp2101 defines

#define AXP2101_I2C_ADDR (0x35)

// macro utils

#define EC_E_BOOL_BREAK(expr)       ExecuteCheck_ADV(expr, true, break;)

#define axp216_reg_read(reg, val)   nrf_i2c_handle->Reg.Read(AXP216_I2C_ADDR, reg, val)
#define axp216_reg_write(reg, val)  nrf_i2c_handle->Reg.Write(AXP216_I2C_ADDR, reg, val)
#define axp216_set_bits(reg, mask)  nrf_i2c_handle->Reg.SetBits(AXP216_I2C_ADDR, reg, mask)
#define axp216_clr_bits(reg, mask)  nrf_i2c_handle->Reg.ClrBits(AXP216_I2C_ADDR, reg, mask)

#define axp2101_reg_read(reg, val)  nrf_i2c_handle->Reg.Read(AXP2101_I2C_ADDR, reg, val)
#define axp2101_reg_write(reg, val) nrf_i2c_handle->Reg.Write(AXP2101_I2C_ADDR, reg, val)
#define axp2101_set_bits(reg, mask) nrf_i2c_handle->Reg.SetBits(AXP2101_I2C_ADDR, reg, mask)
#define axp2101_clr_bits(reg, mask) nrf_i2c_handle->Reg.ClrBits(AXP2101_I2C_ADDR, reg, mask)

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
        for ( uint8_t retry = 0; retry < 10; retry++ )
        {
            // looking for AXP2101
            if ( axp2101_reg_read(0x00, &val) )
            {
                result = AXP216_CONF_NOT_NEEDED;
                break;
            }

            // looking for AXP216
            nrf_i2c_handle->HighDriveStrengthCtrl(true);
            if ( !axp216_reg_read(AXP216_IC_TYPE, &val) )
            {
                result = AXP216_CONF_NO_ACK;
                nrf_delay_ms(100);
                continue;
            }

            // check id, as axp different ic may share same i2c addrs
            if ( val == 0x62 )
            {
                result = AXP216_CONF_FAILED;

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

                // configure correct power output
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x34));
                EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));

                // shutdown
                // EC_E_BOOL_BREAK(axp216_set_bits(AXP216_OFF_CTL, 0b10000000));

                if ( false )
                {
                    // power cycle by sleep and wakeup
                    // enable wakeup
                    EC_E_BOOL_BREAK(axp216_reg_read(AXP216_VOFF_SET, &val)); // backup reg
                    EC_E_BOOL_BREAK(axp216_set_bits(AXP216_VOFF_SET, (1 << 3)));
                    // turn off output to sleep
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
                    // delay
                    nrf_delay_ms(50);
                    // try wakeup
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_VOFF_SET, (val | (1 << 5))));
                }
                else
                {
                    // power cycle by soft off (left an output on)
                    // soft off
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x15)); // keep eldo1 on
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
                    // delay
                    nrf_delay_ms(50);
                    // back on
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN2, 0x34));
                    EC_E_BOOL_BREAK(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));
                }

                result = AXP216_CONF_SUCCESS;
                break;
            }
            nrf_i2c_handle->HighDriveStrengthCtrl(false);
        }
    }

    // deinit
    if ( !nrf_i2c_handle->Deinit() )
        result = AXP216_CONF_BUS_ERR;

    return result;
}

#ifdef NOT_USED_ASDF

static uint32_t wait_axp()
{
    uint32_t wait_ms = 0;
  #include "nrf_i2c.h"
    static I2C_t* nrf_i2c_handle = NULL;
    nrf_i2c_handle = nrf_i2c_get_instance();
    nrf_i2c_handle->Reset();
    nrf_i2c_handle->Init();
    // reboot
    while ( !nrf_i2c_handle->Reg.SetBits(0x34, 0x31, 0b01000000) )
        ;
    while ( !nrf_i2c_handle->Reg.SetBits(0x34, 0x31, 0b01000000) )
    {
        nrf_delay_ms(1);
        wait_ms++;
    }
    // uint8_t val = 0;
    // while ( !nrf_i2c_handle->Reg.Read(0x34,0x03, &val) )
    // {
    //     nrf_delay_ms(1);
    //     wait_ms++;
    // }
    nrf_i2c_handle->Deinit();
    return wait_ms;
}

static ret_code_t power_pre_config()
{
    NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);
    NRF_LOG_FLUSH();

    ENSURE_I2C_AVAILABLE()

    preg(AXP216_ALDO1OUT_VOL);
    preg(AXP216_ALDO2OUT_VOL);
    preg(AXP216_ALDO3OUT_VOL);
    preg(AXP216_DC1OUT_VOL);

    // ALDO1 -> LDO_1V8 1.8V
    NRF_LOG_INFO("ALDO1 -> LDO_1V8 1.8V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
    // ret = axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B);
    // if ( ret != NRF_SUCCESS )
    //     return ret;

    // ALDO2 -> LDO_FB 3.3V
    NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
    // ret = axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a);
    // ret = axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1C);
    // if ( ret != NRF_SUCCESS )
    //     return ret;static void pmu_sleep_ctrl(bool sleep_enable)
    {
        NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);

        if ( !i2c_configured )
            return;

        if ( sleep_enable )
        {
            // prepare sleep
            axp_set_bits(AXP216_VOFF_SET, (1 << 3));
            // power off (sleep)
            power_off();
        }
        else
        {
            // wakeup
            // axp_set_bits(AXP216_VOFF_SET, (1 << 5));

            uint8_t reg_val;
            NRF_EXEC_RETRY(axp216_read(AXP216_VOFF_SET, 1, &reg_val));

            if ( (reg_val & (1 << 5)) != (1 << 5) )
            {
                reg_val |= (1 << 5);
                NRF_EXEC_RETRY(axp216_reg_write(AXP216_VOFF_SET, reg_val));
            }
        }
    }

    // ALDO3 -> LDO_NF 3.3V
    NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
    // ret = axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a);
    // ret = axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1E);
    // if ( ret != NRF_SUCCESS )
    //     return ret;

    // DCDC1 -> MAIN 3.3V
    NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));
    // ret = axp216_reg_write(AXP216_DC1OUT_VOL, 0x11);
    // if ( ret != NRF_SUCCESS )
    //     return ret;

    preg(AXP216_ALDO1OUT_VOL);
    preg(AXP216_ALDO2OUT_VOL);
    preg(AXP216_ALDO3OUT_VOL);
    preg(AXP216_DC1OUT_VOL);

    return NRF_SUCCESS;
}

static void print_axp_reg(char* reg_name, uint8_t reg_addr)
{
    ret_code_t ret = 0;
    uint8_t val = 0x5a;
    axp216_read(reg_addr, 1, &val);
    NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
    NRF_LOG_INFO("%s(0x%02x)=0x%02x", reg_name, reg_addr, val);
}
  #define preg(x) print_axp_reg(#x, x)

  #define NRF_EXEC_RETRY_ADV(EXPR, RETRY, DELAY_MS)
{
    uint32_t retry = 1;
    ret_code_t ret = NRF_SUCCESS;
    while ( retry <= RETRY )
    {
        NRF_LOG_INFO("EXPR=%s, TRY=%lu/%lu", #EXPR, retry, RETRY);
        ret = (EXPR);
        if ( ret == NRF_SUCCESS )
            break;
        else
            nrf_delay_ms(DELAY_MS);
        retry++;
    }
    if ( ret != NRF_SUCCESS )
        return ret;
}

  #define NRF_EXEC_RETRY(EXPR) NRF_EXEC_RETRY_ADV(EXPR, 10, 10)

static ret_code_t power_config_aio(bool left_on)
{
    NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);
    NRF_LOG_FLUSH();

    ENSURE_I2C_AVAILABLE();

    // configure voltages
    // ALDO1 -> LDO_1V8 1.8V
    NRF_LOG_INFO("ALDO1 -> LDO_1V8 1.8V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
    // ALDO2 -> LDO_FB 3.3V
    NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
    // ALDO3 -> LDO_NF 3.3V
    NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
    // DCDC1 -> MAIN 3.3V
    NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
    NRF_LOG_FLUSH();
    NRF_EXEC_RETRY(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));

    // configure controls
    if ( left_on )
    {
        NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN2, 0x20));
        NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));
    }
    else
    {
        NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
        NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14)); // bit 2:4 default 101, should not be
        changed
    }

    // preg(AXP216_LDO_DC_EN1);
    // preg(AXP216_LDO_DC_EN2);
    // preg(AXP216_ALDO1OUT_VOL);
    // preg(AXP216_ALDO2OUT_VOL);
    // preg(AXP216_ALDO3OUT_VOL);
    // preg(AXP216_DC1OUT_VOL);
}

#endif