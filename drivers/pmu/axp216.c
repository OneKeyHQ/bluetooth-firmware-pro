#include "axp216.h"

#include "ntc_util.h"

// macros
#define axp216_reg_read(reg, val)  pmu_interface_p->Reg.Read(AXP216_I2C_ADDR, reg, val)
#define axp216_reg_write(reg, val) pmu_interface_p->Reg.Write(AXP216_I2C_ADDR, reg, val)
#define axp216_set_bits(reg, mask) pmu_interface_p->Reg.SetBits(AXP216_I2C_ADDR, reg, mask)
#define axp216_clr_bits(reg, mask) pmu_interface_p->Reg.ClrBits(AXP216_I2C_ADDR, reg, mask)

// vars private
static bool initialized = false;
static PMU_Interface_t* pmu_interface_p = NULL;
static Power_State_t state_current = PWR_STATE_INVALID;
static Power_Status_t status_current = {0};

// functions private

static bool axp216_config_voltage(void)
{
    //  voltages
    // ALDO1 -> LDO_1V8 1.8V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
    // ALDO2 -> LDO_FB 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
    // ALDO3 -> LDO_NF 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
    // DCDC1 -> MAIN 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));

    return true;
}

#define AXP216_VTS_TO_VXTF(mv) (mv / 0.8 / 0x10 / 1000)
static bool axp216_config_battery(void)
{
    // cap
    uint16_t value = (float)(530 / 1.456);
    // cap set flag
    value |= 0x80;
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP0, (uint8_t)(value >> 8)));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP1, (uint8_t)value));

    // warn level
    // WARN -> 10%
    // CRITICAL -> 5%
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_WARN, 0x55));

    // charge settings
    // allow chg, 4.35v, 450ma, 45ma cutoff
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_CHARGE1, 0xF1));
    // pre chg 40min timeout, keep chg output, type a, follow, 8hr max
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_CHARGE2, 0x25));

    // temp formula
    // V = reg_val * 0x10 * 0.0008V
    // charge temp max
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VHTF_CHG, (uint8_t)(AXP216_VTS_TO_VXTF(196.96)))); // 196.96mv
    // charge temp min
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VLTF_CHG, (uint8_t)(AXP216_VTS_TO_VXTF(711.2)))); // 711.2mv
    // discharge temp max
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VHTF_DISCHG, (uint8_t)(AXP216_VTS_TO_VXTF(121.28)))); // 121.28mv
    // discharge temp min
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VLTF_DISCHG, (uint8_t)(AXP216_VTS_TO_VXTF(2520)))); // 2520mv

    return true;
}

static bool axp216_config_adc(void)
{
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ADC_CONTROL3, 0x12)); // ts enable, 40ua
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ADC_EN, 0xE1));

    return true;
}

static bool axp216_config_irq(void)
{
    // disable irq
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN1, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN2, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN3, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN4, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN5, 0x00));

    // clear irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS1, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS2, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS3, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS4, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS5, 0xFF));

    // enable irq (only needed)
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN1, 0xFC));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN2, 0xCC));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN3, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN4, 0x10));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN5, 0x78));

    return true;
}

static bool axp216_config_common(void)
{
    // vbus hold at 4.4v, no input current limit
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_IPS_SET, 0x60));
    // pwr on 1s, long press 2s, force off 4s, no auto reboot on force off
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_POK_SET, 0x68));
    // battery detect enable, pwrok delay 8ms
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_OFF_CTL, 0x48));
    // 16s key press force reboot, die ovtmp off, no irq turn on
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_HOTOVER_CTL, 0x0f));
    // voff(ipsout/vsys) 3.3v, no irq wakeup
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VOFF_SET, 0x07));

    return true;
}

// function public

Power_Error_t axp216_init(void)
{
    if ( initialized )
    {
        return PWR_ERROR_NONE;
    }

    do
    {
        // interface init
        if ( !*(pmu_interface_p->isInitialized) )
        {
            if ( !pmu_interface_p->Init() )
            {
                break;
            }
        }

        // get id
        uint8_t val = 0;
        if ( !axp216_reg_read(AXP216_IC_TYPE, &val) )
        {
            break;
        }

        // compare id
        if ( val != 0x62 )
            break;

        initialized = true;
        return PWR_ERROR_NONE;
    }
    while ( false );

    return PWR_ERROR_FAIL;
}

Power_Error_t axp216_deinit(void)
{

    if ( !initialized )
        return PWR_ERROR_NONE;

    do
    {
        // interface deinit()
        if ( *(pmu_interface_p->isInitialized) )
            if ( !pmu_interface_p->Deinit() )
                break;

        // nothing left

        initialized = false;
        return PWR_ERROR_NONE;
    }
    while ( false );

    return PWR_ERROR_FAIL;
}

Power_Error_t axp216_reset(void)
{
    EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 6)));
    return PWR_ERROR_NONE;
}

Power_Error_t axp216_irq(void)
{
    uint8_t irqs[5] = {0};
    uint64_t irq_bits = 0;

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS3, &irqs[2]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS4, &irqs[3]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS5, &irqs[4]));

    // irq_bits |= ((irqs[0] & (1 << 6)) >> 6 << PWR_IRQ_PWR_CONNECTED);    // acin only, as vbus connected to acin
    // irq_bits |= ((irqs[0] & (1 << 5)) >> 5 << PWR_IRQ_PWR_DISCONNECTED); // acin only, as vbus connected to acin
    // irq_bits |= ((irqs[1] & (1 << 3)) >> 3 << PWR_IRQ_CHARGING);
    // irq_bits |= ((irqs[1] & (1 << 2)) >> 2 << PWR_IRQ_CHARGED);
    // irq_bits |= ((irqs[3] & (1 << 1)) >> 1 << PWR_IRQ_BATT_LOW);
    // irq_bits |= ((irqs[3] & (1 << 0)) >> 0 << PWR_IRQ_BATT_CRITICAL);
    // irq_bits |= ((irqs[4] & (1 << 5)) >> 6 << PWR_IRQ_PB_RELEASE);
    // irq_bits |= ((irqs[4] & (1 << 6)) >> 5 << PWR_IRQ_PB_PRESS);
    // irq_bits |= ((irqs[4] & (1 << 4)) >> 4 << PWR_IRQ_PB_SHORT);
    // irq_bits |= ((irqs[4] & (1 << 3)) >> 3 << PWR_IRQ_PB_LONG);
    // irq_bits |= ((irqs[4] & (1 << 2)) >> 2 << PWR_IRQ_PB_FORCEOFF);

    irq_bits |= ((((irqs[0] & (1 << 6))) != 0) << PWR_IRQ_PWR_CONNECTED);    // acin only, as vbus connected to acin
    irq_bits |= ((((irqs[0] & (1 << 5))) != 0) << PWR_IRQ_PWR_DISCONNECTED); // acin only, as vbus connected to acin
    irq_bits |= ((((irqs[1] & (1 << 3))) != 0) << PWR_IRQ_CHARGING);
    irq_bits |= ((((irqs[1] & (1 << 2))) != 0) << PWR_IRQ_CHARGED);
    irq_bits |= ((((irqs[3] & (1 << 1))) != 0) << PWR_IRQ_BATT_LOW);
    irq_bits |= ((((irqs[3] & (1 << 0))) != 0) << PWR_IRQ_BATT_CRITICAL);
    irq_bits |= ((((irqs[4] & (1 << 6))) != 0) << PWR_IRQ_PB_RELEASE); // low to high
    irq_bits |= ((((irqs[4] & (1 << 5))) != 0) << PWR_IRQ_PB_PRESS);   // high to low
    irq_bits |= ((((irqs[4] & (1 << 4))) != 0) << PWR_IRQ_PB_SHORT);
    irq_bits |= ((((irqs[4] & (1 << 3))) != 0) << PWR_IRQ_PB_LONG);
    irq_bits |= ((((irqs[4] & (1 << 2))) != 0) << PWR_IRQ_PB_FORCEOFF);

    // process irq
    pmu_interface_p->Irq(irq_bits);

    // clear irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS1, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS2, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS3, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS4, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_INTSTS5, 0xFF));

    return PWR_ERROR_NONE;
}

Power_Error_t axp216_config(void)
{
    EC_E_BOOL_R_PWR_ERR(axp216_config_voltage());
    EC_E_BOOL_R_PWR_ERR(axp216_config_common());
    EC_E_BOOL_R_PWR_ERR(axp216_config_battery());
    EC_E_BOOL_R_PWR_ERR(axp216_config_adc());
    EC_E_BOOL_R_PWR_ERR(axp216_config_irq());

    pmu_interface_p->Delay_ms(200);

    return PWR_ERROR_NONE;
}

Power_Error_t axp216_set_state(const Power_State_t state)
{
    static uint8_t reg31_bak = 0x00;
    switch ( state )
    {
    case PWR_STATE_SOFT_OFF:
        // close output
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN2, 0x15)); // keep eldo1 on
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
        // eldo1 is not used and not connected, keep it on is just for prevent axp "sleep"
        // "close all output means sleep" is a really stupid design, as axp turns off I2C when sleeping
        // there is no way to wake it up if you do't enable wakeup source before close all output
        // make sure config the "left on" reg first
        break;
    case PWR_STATE_HARD_OFF:
        // close output (not needed as the pmu off will kill all output)
        // EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
        // EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));
        // pmu off
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_OFF_CTL, (1 << 7)));
        break;
    case PWR_STATE_ON:
        // strong drive
        pmu_interface_p->HighDriveStrengthCtrl(true);
        // open output
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN2, 0x34));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));
        // normal drive
        pmu_interface_p->HighDriveStrengthCtrl(false);
        break;
    case PWR_STATE_SLEEP:
        // backup reg
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VOFF_SET, &reg31_bak));
        // allow irq wakeup
        // EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 4)));
        // enable wakeup
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 3)));
        // close output to sleep (that's how axp "sleep" works)
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
        break;
    case PWR_STATE_WAKEUP:
        // strong drive
        pmu_interface_p->HighDriveStrengthCtrl(true);
        // wakeup via i2c
        // EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 5))); // may not work, cant read while sleep
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_VOFF_SET, (reg31_bak | (1 << 5))));
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

Power_Error_t axp216_get_state(Power_State_t* state)
{
    *state = state_current;
    return PWR_ERROR_USAGE;
}

Power_Error_t axp216_pull_status(void)
{
    HL_Buff hlbuff;

    Power_Status_t status_temp = {0};

    // battery present
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_MODE_CHGSTATUS, &(hlbuff.u8_low)));
    status_temp.batteryPresent = ((hlbuff.u8_low & (1 << 5)) == (1 << 5)); // bit 5, not documented

    if ( status_temp.batteryPresent )
    {
        // battery percent
        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_BAT_LEVEL, &(hlbuff.u8_low)));
        if ( (hlbuff.u8_low & 0x80) == 0x80 ) // is data valid
            status_temp.batteryPercent = hlbuff.u8_low & 0x7f;
        else
            status_temp.batteryPercent = 100;

        // battery voltage
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VBATH_RES, &(hlbuff.u8_high)));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VBATL_RES, &(hlbuff.u8_low)));
        status_temp.batteryVoltage = (hlbuff.u16 >> 4) * 1.1 + 0; // val * step - base

        // battery temp
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VTSH_RES, &(hlbuff.u8_high)));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VTSL_RES, &(hlbuff.u8_low)));
        status_temp.batteryTemp =
            ntc_temp_cal_cv(NTC_Char_NCP15XH103F03RC_2585, 40, ((hlbuff.u16 >> 4) * 0.8 + 0) * 1000); // temp_c
    }
    else
    {
        status_temp.batteryPercent = 0;
        status_temp.batteryVoltage = 0;
        status_temp.batteryTemp = -999;
    }

    // pmu temp
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTTEMPH, &(hlbuff.u8_high)));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTTEMPL, &(hlbuff.u8_low)));
    status_temp.pmuTemp = (uint16_t)((hlbuff.u16 >> 4) * 0.1 - 267.7); // val * step - base

    // charging
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CHARGE1, &(hlbuff.u8_low)));
    status_temp.chargeAllowed = ((hlbuff.u8_low & (1 << 7)) == (1 << 7));

    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_STATUS, &(hlbuff.u8_low)));
    status_temp.chargerAvailable =
        (((hlbuff.u8_low & ((1 << 7) | (1 << 6))) == ((1 << 7) | (1 << 6))) && // acin
         ((hlbuff.u8_low & ((1 << 5) | (1 << 4))) == ((1 << 5) | (1 << 4)))    // vbus
        );

    if ( status_temp.chargeAllowed && status_temp.chargerAvailable )
    {
        if ( ((hlbuff.u8_low & (1 << 2)) == (1 << 2)) ) // check if charging
        {
            // read gpio
            EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_GPIO1_CTL, 0b00000010)); // gpio1 input
            hlbuff.u8_high = 0;
            EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_GPIO01_SIGNAL, &(hlbuff.u8_low))); // gpio1 read
            EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_GPIO1_CTL, 0b00000111));          // gpio1 float
            status_temp.wirelessCharge = ((hlbuff.u8_low & (1 << 1)) != (1 << 1));            // low when wireless charging

            // if not wireless charging then it's wired
            status_temp.wiredCharge = !status_temp.wirelessCharge;
        }
        else
        {
            status_temp.wiredCharge = false;
            status_temp.wirelessCharge = false;
        }

        if ( status_temp.wiredCharge || status_temp.wirelessCharge )
        {
            EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CCBATH_RES, &(hlbuff.u8_high)));
            EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CCBATL_RES, &(hlbuff.u8_low)));
            status_temp.chargeCurrent = (hlbuff.u16 >> 4);
            status_temp.dischargeCurrent = 0;
        }
        else
        {
            EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_DCBATH_RES, &(hlbuff.u8_high)));
            EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_DCBATL_RES, &(hlbuff.u8_low)));
            status_temp.dischargeCurrent = (hlbuff.u16 >> 4);
            status_temp.chargeCurrent = 0;
        }

        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_MODE_CHGSTATUS, &(hlbuff.u8_low)));
        status_temp.chargeFinished = ((hlbuff.u8_low & (1 << 6)) != (1 << 6));
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

Power_Error_t axp216_set_feature(Power_Featrue_t feature, bool enable)
{
    switch ( feature )
    {
    case PWR_FEAT_CHARGE:
        if ( enable )
        {
            EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_CHARGE1, (1 << 7)));
        }
        else
        {
            EC_E_BOOL_R_PWR_ERR(axp216_clr_bits(AXP216_CHARGE1, (1 << 7)));
        }
        break;

    case PWR_FEAT_INVALID:
    default:
        return PWR_ERROR_USAGE;
        break;
    }

    return PWR_ERROR_NONE;
}

Power_Error_t axp216_get_feature(Power_Featrue_t feature, bool* enable)
{
    uint8_t reg_val;
    switch ( feature )
    {
    case PWR_FEAT_CHARGE:
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CHARGE1, &reg_val));
        *enable = ((reg_val & (1 << 7)) == (1 << 7));
        break;

    case PWR_FEAT_INVALID:
    default:
        return PWR_ERROR_USAGE;
        break;
    }

    return PWR_ERROR_NONE;
}

void axp216_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p)
{
    pmu_interface_p = pmu_if_p;

    pmu_p->isInitialized = &initialized;
    strncpy(pmu_p->InstanceName, "AXP216", PMU_INSTANCE_NAME_MAX_LEN);
    pmu_p->PowerStatus = &status_current;

    pmu_p->Init = axp216_init;
    pmu_p->Deinit = axp216_deinit;
    pmu_p->Reset = axp216_reset;
    pmu_p->Config = axp216_config;
    pmu_p->Irq = axp216_irq;
    pmu_p->SetState = axp216_set_state;
    pmu_p->GetState = axp216_get_state;
    pmu_p->PullStatus = axp216_pull_status;
    pmu_p->SetFeature = axp216_set_feature;
    pmu_p->GetFeature = axp216_get_feature;
}
