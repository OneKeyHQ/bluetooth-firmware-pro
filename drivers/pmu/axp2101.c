#include "axp2101.h"

#include "ntc_util.h"

// macros
#define axp2101_reg_read(reg, val)  pmu_interface_p->Reg.Read(AXP2101_I2C_ADDR, reg, val)
#define axp2101_reg_write(reg, val) pmu_interface_p->Reg.Write(AXP2101_I2C_ADDR, reg, val)
#define axp2101_set_bits(reg, mask) pmu_interface_p->Reg.SetBits(AXP2101_I2C_ADDR, reg, mask)
#define axp2101_clr_bits(reg, mask) pmu_interface_p->Reg.ClrBits(AXP2101_I2C_ADDR, reg, mask)

// vars private
static bool initialized = false;
static PMU_Interface_t* pmu_interface_p = NULL;
static Power_State_t state_current = PWR_STATE_INVALID;
static Power_Status_t status_current = {0};

// functions private

static bool axp2101_config_voltage(void)
{
    //  voltages
    // ALDO1 -> RAIL_1V8 1.8V
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ALDO1_CFG, 0x0D));
    // DCDC1 -> RAIL_3V3 3.3V
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC1_CFG, 0x14));

    return true;
}

static bool axp2101_config_battery_param(void)
{
    uint8_t val_temp = 0xff;

    // check if BROM already programed
    // according to axp support, check reg A2 bit 4
    EC_E_BOOL_R_BOOL(axp2101_reg_read(AXP2101_CONFIG, &val_temp));
    if ( (val_temp & (1 << 4)) == (1 << 4) )
    {
        pmu_interface_p->Log(PWR_LOG_LEVEL_INFO, "BROM already configured, skip");
        return true;
    }

    // battery param
    static const uint8_t batt_cal_data[128] = {
        0x01, 0xf5, 0x40, 0x00, 0x1b, 0x1e, 0x28, 0x0f, 0x0c, 0x1e, 0x32, 0x02, 0x14, 0x05, 0x0a, 0x04, //
        0x74, 0x00, 0x78, 0x0c, 0xdf, 0x10, 0xcc, 0xfc, 0xd0, 0x01, 0xea, 0x0c, 0x28, 0x06, 0xa4, 0x06, //
        0x6b, 0x0b, 0x37, 0x0f, 0xe0, 0x0f, 0x81, 0x0a, 0x32, 0x0e, 0xed, 0x0e, 0xe8, 0x04, 0xdc, 0x04, //
        0xcf, 0x09, 0xc5, 0x0e, 0xb5, 0x0e, 0xb0, 0x09, 0xa5, 0x0e, 0x92, 0x0e, 0x8c, 0x04, 0x7d, 0x04, //
        0x6d, 0x09, 0x69, 0x0e, 0x57, 0x0d, 0xdf, 0x07, 0x85, 0x59, 0x2c, 0x28, 0x1e, 0x0d, 0x12, 0x08, //
        0xc5, 0x98, 0x7e, 0x66, 0x4e, 0x44, 0x38, 0x1a, 0x12, 0x0a, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6, //
        0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6, //
        0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6  //
    };

    // reset fuel gauge
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_RESET_CFG, (1 << 2)));
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_RESET_CFG, (1 << 2)));
    // enable BROM access
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_CONFIG, (1 << 0)));
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_CONFIG, (1 << 0)));
    // program BROM
    for ( uint8_t i = 0; i < sizeof(batt_cal_data); i++ )
    {
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_BROM, batt_cal_data[i]));
        pmu_interface_p->Log(PWR_LOG_LEVEL_INFO, "BROM prog! i=%u, buff=0x%02x", i, batt_cal_data[i]);
        pmu_interface_p->Delay_ms(10);
    }
    // enable BROM access
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_CONFIG, (1 << 0)));
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_CONFIG, (1 << 0)));
    // verify BROM
    for ( uint8_t i = 0; i < sizeof(batt_cal_data); i++ )
    {
        val_temp = 0;
        EC_E_BOOL_R_BOOL(axp2101_reg_read(AXP2101_BROM, &val_temp));
        if ( batt_cal_data[i] != val_temp )
        {
            pmu_interface_p->Log(PWR_LOG_LEVEL_ERR, "BROM verify failed!");
            pmu_interface_p->Log(PWR_LOG_LEVEL_ERR, "i=%u, buff=0x%02x, val=0x%02x", i, batt_cal_data[i], val_temp);
            return false;
        }
        pmu_interface_p->Delay_ms(10);
    }
    // disable BROM access
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_CONFIG, (1 << 0)));
    // set fuel gauge use BROM
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_CONFIG, (1 << 4)));
    // reset fuel gauge
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_RESET_CFG, (1 << 2)));
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_RESET_CFG, (1 << 2)));

    return true;
}

#define AXP2101_VTS_TO_VHTF(mv) (mv / 32)
#define AXP2101_VTS_TO_VLTF(mv) (mv / 2)
static bool axp2101_config_battery(void)
{
    // warn level
    // WARN -> 10%
    // CRITICAL -> 5%
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_GAUGE_THLD, 0x55));

    // charge setting
    // normarl charge current 500ma
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ICC_CFG, 0b00001011));

    // voltage
    // bit 7:3 zero
    // bit 2:0 0b100 -> 4.35v
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_CHG_V_CFG, (uint8_t)0b00000100));

    // temp formula
    // V = reg_val * 0x10 * 0.0008V
    // Vh = reg_val * 32mv
    // Vl = reg_val * 2mv

    // charge temp max
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_VHTF_CHG, (uint8_t)(AXP2101_VTS_TO_VHTF(196.96)))); // 196.96mv

    // charge temp min
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_VLTF_CHG, (uint8_t)(AXP2101_VTS_TO_VLTF(711.2)))); // 711.2mv

    // discharge temp max
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_VHTF_DISCHG, (uint8_t)(AXP2101_VTS_TO_VHTF(121.28)))); // 121.28mv

    // discharge temp min
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_VLTF_DISCHG, (uint8_t)(AXP2101_VTS_TO_VLTF(2520)))); // 2520mv

    return true;
}

static bool axp2101_config_adc(void)
{
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DIE_TEMP_CFG, 0b00000001)); // 115C max, detect enable
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ADC_CH_EN0, 0b00011111));

    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_TS_CFG, 0b00000001)); // bit 1:0 = 01
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_TS_CFG, 0b00000010)); // TS current 40ua

    return true;
}

static bool axp2101_config_irq(void)
{
    // disable irq
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN1, 0x00));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN2, 0x00));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN3, 0x00));

    // clear irq
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTSTS1, 0xFF));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTSTS2, 0xFF));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTSTS3, 0xFF));

    // enable irq (only needed)
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN1, 0xC0));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN2, 0xCF));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_INTEN3, 0x18));

    return true;
}

static bool axp2101_config_common(void)
{
    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_COMM_CFG, (1 << 2))); // pwr key hold 16s to resetart

    // wdt
    // TODO: how to reset timer? any other way than write reg?
    // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_WATCHDOG_CFG, 0b00010011)); // on trigger send irq and reset, 8s timer
    // EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_MODULE_EN, (1 << 0)));       //enable

    return true;
}

static bool axp2101_charge_current_sel(bool low_current_mode)
{
    uint8_t reg_val = 0;
    EC_E_BOOL_R_BOOL(axp2101_reg_read(AXP2101_ICC_CFG, &reg_val));
    reg_val &= 0b11100000;                                   // clear current bits [4:0]
    reg_val |= (low_current_mode ? 0b00001001 : 0b00001011); // set current 300ma or 500ma
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ICC_CFG, reg_val));
    return true;
}

// function public

Power_Error_t axp2101_init(void)
{
    if ( initialized )
    {
        // interface init
        if ( !pmu_interface_p->Init() )
            return PWR_ERROR_FAIL;

        // get id
        uint8_t val = 0;
        if ( !axp2101_reg_read(AXP2101_CHIP_ID, &val) )
            return PWR_ERROR_FAIL;

        // compare id
        if ( val != 0x4a )
            return PWR_ERROR_FAIL;

        initialized = true;
    }

    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_deinit(void)
{
    if ( !initialized )
    {
        if ( !pmu_interface_p->Deinit() )
            return PWR_ERROR_FAIL;

        initialized = false;
    }
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_reset(void)
{
    EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_COMM_CFG, (1 << 1)));
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_irq(void)
{
    uint8_t irqs[3];
    uint64_t irq_bits = 0;

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS3, &irqs[2]));

    irq_bits |= ((((irqs[1] & (1 << 7))) != 0) << PWR_IRQ_PWR_CONNECTED);    // vbus
    irq_bits |= ((((irqs[1] & (1 << 6))) != 0) << PWR_IRQ_PWR_DISCONNECTED); // vbus
    irq_bits |= ((((irqs[2] & (1 << 3))) != 0) << PWR_IRQ_CHARGING);
    irq_bits |= ((((irqs[2] & (1 << 4))) != 0) << PWR_IRQ_CHARGED);
    irq_bits |= ((((irqs[0] & (1 << 7))) != 0) << PWR_IRQ_BATT_LOW);
    irq_bits |= ((((irqs[0] & (1 << 6))) != 0) << PWR_IRQ_BATT_CRITICAL);
    irq_bits |= ((((irqs[1] & (1 << 1))) != 0) << PWR_IRQ_PB_PRESS);
    irq_bits |= ((((irqs[1] & (1 << 0))) != 0) << PWR_IRQ_PB_RELEASE);
    irq_bits |= ((((irqs[1] & (1 << 3))) != 0) << PWR_IRQ_PB_SHORT);
    irq_bits |= ((((irqs[1] & (1 << 2))) != 0) << PWR_IRQ_PB_LONG);
    // PWR_IRQ_PB_FORCEOFF not supported

    // process irq
    pmu_interface_p->Irq(irq_bits);

    // clear irq
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_INTSTS1, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_INTSTS2, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_INTSTS3, 0xFF));

    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_config(void)
{
    EC_E_BOOL_R_PWR_ERR(axp2101_config_voltage());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_common());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_battery_param());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_battery());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_adc());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_irq());
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_set_state(const Power_State_t state)
{
    switch ( state )
    {
    case PWR_STATE_SOFT_OFF:
        // close output
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x40)); // keep cpuldo on
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0x00));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x00));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00));
        // cpuldo is not used and not connected, keep it on is just for prevent axp "sleep"
        // "close all output means sleep" is a really stupid design, as axp turns off I2C when sleeping
        // there is no way to wake it up if you do't enable wakeup source before close all output
        // make sure config the "left on" reg first
        break;
    case PWR_STATE_HARD_OFF:
        // close output (not needed as the pmu off will kill all output)
        // EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x00));
        // EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00));
        // EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x00));
        // EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0x00));
        // pmu off
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_COMM_CFG, (1 << 0)));
        break;
    case PWR_STATE_ON:
        // strong drive
        pmu_interface_p->HighDriveStrengthCtrl(true);
        // open output
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x01)); // aldo1 on, other off
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0X00)); // all off
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x01));   // dcdc1 on, other off
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00));   // all off
        // normal drive
        pmu_interface_p->HighDriveStrengthCtrl(false);
        break;
    case PWR_STATE_SLEEP:
        // allow irq wakeup
        // EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 4)));
        // enable wakeup
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 3)));
        // close output ("sleep")
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x00));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x00));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0x00));
        break;
    case PWR_STATE_WAKEUP:
        // strong drive
        pmu_interface_p->HighDriveStrengthCtrl(true);
        // wakeup via i2c
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 1)));
        // normal drive
        pmu_interface_p->HighDriveStrengthCtrl(false);
        break;

    case PWR_STATE_INVALID:
    default:
        return PWR_ERROR_USAGE;
        break;
    }

    // normal exit
    state_current = state;
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_get_state(Power_State_t* state)
{
    *state = state_current;
    return PWR_ERROR_USAGE;
}

Power_Error_t axp2101_pull_status(void)
{
    HL_Buff hlbuff;

    Power_Status_t status_temp = {0};

    // battery present
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT0, &(hlbuff.u8_low)));
    status_temp.batteryPresent = ((hlbuff.u8_low & (1 << 3)) == (1 << 3));

    if ( status_temp.batteryPresent )
    {
        // battery percent
        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_SOC, &(hlbuff.u8_low)));
        status_temp.batteryPercent = hlbuff.u8_low;

        // battery voltage
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_H, &(hlbuff.u8_high)));
        hlbuff.u8_high &= 0b00111111; // drop bit 7:6
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_L, &(hlbuff.u8_low)));
        status_temp.batteryVoltage = hlbuff.u16;

        // battery temp
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_H, &(hlbuff.u8_high)));
        hlbuff.u8_high &= 0b00111111; // drop bit 7:6
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_L, &(hlbuff.u8_low)));
        status_temp.batteryTemp =
            ntc_temp_cal_cv(NTC_Char_NCP15XH103F03RC_2585, 40, ((hlbuff.u16 * 0.5) * 0.8 + 0) * 1000); // temp_c
    }
    else
    {
        status_temp.batteryPercent = 0;
        status_temp.batteryVoltage = 0;
        status_temp.batteryTemp = -999;
    }

    // pmu temp
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_H, &(hlbuff.u8_high)));
    hlbuff.u8_high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_L, &(hlbuff.u8_low)));
    status_temp.pmuTemp = (7274 - hlbuff.u16) / 20 + 22;

    // charging
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_MODULE_EN, &(hlbuff.u8_low)));
    status_temp.chargeAllowed = ((hlbuff.u8_low & (1 << 1)) == (1 << 1));

    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT0, &(hlbuff.u8_low)));
    status_temp.chargerAvailable = ((hlbuff.u8_low & (1 << 5)) == (1 << 5)); // vbus good

    if ( status_temp.chargeAllowed && status_temp.chargerAvailable )
    {
        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT1, &(hlbuff.u8_low)));
        status_temp.chargeFinished = ((hlbuff.u8_low & 0b00000111) == 0b00000100); // bit 2:0 = 100 charge done

        if ( (hlbuff.u8_low & 0b01100000) == 0b00100000 || // bit 6:5 = 01 battery current charge
             status_temp.chargeFinished                    // still try get power source
        )
        {
            // TODO: find a batter way to check GPIO?
            // read gpio
            bool gpio_high_low = true;
            EC_E_BOOL_R_PWR_ERR(pmu_interface_p->GPIO.Config(8, PWR_GPIO_Config_READ_PH));
            pmu_interface_p->Delay_ms(5);
            EC_E_BOOL_R_PWR_ERR(pmu_interface_p->GPIO.Read(8, &gpio_high_low));
            EC_E_BOOL_R_PWR_ERR(pmu_interface_p->GPIO.Config(8, PWR_GPIO_Config_UNUSED));

            status_temp.wirelessCharge = !gpio_high_low; // low is wireless

            // if not wireless charging then it's wired
            status_temp.wiredCharge = !status_temp.wirelessCharge;

            // wireless charge current limit to 300ma
            EC_E_BOOL_R_PWR_ERR(axp2101_charge_current_sel(status_temp.wirelessCharge));
        }
        else
        {
            status_temp.wiredCharge = false;
            status_temp.wirelessCharge = false;
        }

        // not supported by AXP2101
        status_temp.chargeCurrent = 0;
        status_temp.dischargeCurrent = 0;
    }
    else
    {
        status_temp.chargeFinished = false;
        status_temp.wiredCharge = false;
        status_temp.wirelessCharge = false;
        status_temp.chargeCurrent = 0;
        status_temp.dischargeCurrent = 0;
    }

    memcpy(&status_current, &status_temp, sizeof(Power_Status_t));
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_set_feature(Power_Featrue_t feature, bool enable)
{
    switch ( feature )
    {
    case PWR_FEAT_CHARGE:
        if ( enable )
        {
            EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_MODULE_EN, (1 << 1)));
        }
        else
        {
            EC_E_BOOL_R_PWR_ERR(axp2101_clr_bits(AXP2101_MODULE_EN, (1 << 1)));
        }
        break;

    case PWR_FEAT_INVALID:
    default:
        return PWR_ERROR_USAGE;
        break;
    }

    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_get_feature(Power_Featrue_t feature, bool* enable)
{
    uint8_t reg_val;
    switch ( feature )
    {
    case PWR_FEAT_CHARGE:
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_MODULE_EN, &reg_val));
        *enable = ((reg_val & (1 << 1)) == (1 << 1));
        break;

    case PWR_FEAT_INVALID:
    default:
        return PWR_ERROR_USAGE;
        break;
    }

    return PWR_ERROR_NONE;
}

void axp2101_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p)
{
    pmu_interface_p = pmu_if_p;

    pmu_p->isInitialized = &initialized;
    strncpy(pmu_p->InstanceName, "AXP2101", PMU_INSTANCE_NAME_MAX_LEN);
    pmu_p->PowerStatus = &status_current;

    pmu_p->Init = axp2101_init;
    pmu_p->Deinit = axp2101_deinit;
    pmu_p->Reset = axp2101_reset;
    pmu_p->Config = axp2101_config;
    pmu_p->Irq = axp2101_irq;
    pmu_p->SetState = axp2101_set_state;
    pmu_p->GetState = axp2101_get_state;
    pmu_p->PullStatus = axp2101_pull_status;
    pmu_p->SetFeature = axp2101_set_feature;
    pmu_p->GetFeature = axp2101_get_feature;
}
