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

static bool axp2101_config_battery(void)
{
    // battery param
    // TODO: DATA REQUIRED

    // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_BROM, 0x00));

    // warn level
    // WARN -> 10%
    // CRITICAL -> 5%
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_GAUGE_THLD, 0x55));

    // voltage
    // bit 7:3 zero
    // bit 2:0 0b100 -> 4.35v
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_CHG_V_CFG, (uint8_t)0b00000100));

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

static bool axp2101_config_adc(void)
{
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DIE_TEMP_CFG, 0b00000001)); // 115C max
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ADC_CH_EN0, 0b00011111));

    EC_E_BOOL_R_BOOL(axp2101_set_bits(AXP2101_TS_CFG, 0b00000001)); // bit 1:0 = 01
    EC_E_BOOL_R_BOOL(axp2101_clr_bits(AXP2101_TS_CFG, 0b00000010)); // TS current 40ua

    return true;
}

static bool axp2101_output_ctl(bool on_off)
{
    if ( on_off )
    {
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x01));
        // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0X00)); // no need
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x01)
        ); // dcdc1 on, other off
           // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00)); // no need
    }
    else
    {
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x00));
        // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC_CFG1, 0x00)); // no need
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x00));
        // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0x00)); // no need
    }
    return true;
}

// function public

Power_Error_t axp2101_init(void)
{
    if ( initialized )
    {
        return PWR_ERROR_NONE;
    }

    do
    {
        // interface init
        if ( !pmu_interface_p->Init() )
            break;

        // get id
        uint8_t val = 0;
        if ( !axp2101_reg_read(AXP2101_CHIP_ID, &val) )
            break;

        // compare id
        if ( val != 0x4a )
            break;

        initialized = true;
        return PWR_ERROR_NONE;
    }
    while ( false );

    return PWR_ERROR_FAIL;
}

Power_Error_t axp2101_deinit(void)
{
    do
    {
        // interface init()
        if ( !pmu_interface_p->Deinit() )
            break;

        // nothing left

        initialized = false;
        return PWR_ERROR_NONE;
    }
    while ( false );

    return PWR_ERROR_FAIL;
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
    EC_E_BOOL_R_PWR_ERR(axp2101_config_battery());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_irq());
    EC_E_BOOL_R_PWR_ERR(axp2101_config_adc());
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_set_state(const Power_State_t state)
{
    switch ( state )
    {
    case PWR_STATE_OFF:
        // close output
        EC_E_BOOL_R_PWR_ERR(axp2101_output_ctl(false));
        // pmu off
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_COMM_CFG, (1 << 0)));
        break;
    case PWR_STATE_ON:
        // try wakeup anyways
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 1)));
        // config eveything
        EC_E_BOOL_R_PWR_ERR(axp2101_config());
        // open output
        EC_E_BOOL_R_PWR_ERR(axp2101_output_ctl(true));
        break;
    case PWR_STATE_SLEEP:
        // allow irq wakeup
        // EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 4)));
        // enable wakeup
        EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP2101_SLEEP_CFG, (1 << 3)));
        // close output ("sleep")
        EC_E_BOOL_R_PWR_ERR(axp2101_output_ctl(false));
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

Power_Error_t axp2101_get_status(Power_Status_t* status)
{
    HL_Buff hlbuff;

    memset(status, 0x00, sizeof(Power_Status_t));
    status->isValid = false;

    // battery present
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT0, &(hlbuff.u8_low)));
    status->batteryPresent = ((hlbuff.u8_low & (1 << 3)) == (1 << 3));

    if ( status->batteryPresent )
    {
        // battery percent
        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_SOC, &(hlbuff.u8_low)));
        status->batteryPercent = hlbuff.u8_low;

        // battery voltage
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_H, &(hlbuff.u8_high)));
        hlbuff.u8_high &= 0b00111111; // drop bit 7:6
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_L, &(hlbuff.u8_low)));
        status->batteryVoltage = hlbuff.u16;

        // battery temp
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_H, &(hlbuff.u8_high)));
        hlbuff.u8_high &= 0b00111111; // drop bit 7:6
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_L, &(hlbuff.u8_low)));
        status->batteryTemp =
            ntc_temp_cal_cv(NTC_Char_NCP15XH103F03RC_2585, 40, ((hlbuff.u16 * 0.5) * 0.8 + 0) * 1000); // temp_c
    }
    else
    {
        status->batteryPercent = 0;
        status->batteryVoltage = 0;
        status->batteryTemp = 0;
    }

    // pmu temp
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_H, &(hlbuff.u8_high)));
    hlbuff.u8_high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_L, &(hlbuff.u8_low)));
    status->pmuTemp = (7274 - hlbuff.u16) / 20 + 22;

    // charging
    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_MODULE_EN, &(hlbuff.u8_low)));
    status->chargeAllowed = ((hlbuff.u8_low & (1 << 1)) == (1 << 1));

    hlbuff.u8_high = 0;
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT0, &(hlbuff.u8_low)));
    status->chargerAvailable = ((hlbuff.u8_low & (1 << 5)) == (1 << 5)); // vbus good

    if ( status->chargeAllowed && status->chargerAvailable )
    {
        hlbuff.u8_high = 0;
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT1, &(hlbuff.u8_low)));
        status->chargeFinished = ((hlbuff.u8_low & 0b00000111) == 0b00000100); // bit 2:0 = 100 charge done

        if ( (hlbuff.u8_low & 0b01100000) == 0b00100000 || // bit 6:5 = 01 battery current charge
             status->chargeFinished                        // still try get power source
        )
        {
            // TODO: find a batter way to check GPIO?
            // read gpio
            bool gpio_high_low;
            pmu_interface_p->GPIO.Config(8, PWR_GPIO_Config_READ_NP);
            pmu_interface_p->GPIO.Read(8, &gpio_high_low);
            pmu_interface_p->GPIO.Config(8, PWR_GPIO_Config_UNUSED);

            status->wirelessCharge = gpio_high_low; // high is wireless
            // if not wireless charging then it's wired
            status->wiredCharge = !status->wirelessCharge;
        }
        else
        {
            status->wiredCharge = false;
            status->wirelessCharge = false;
        }

        // not supported by AXP2101
        status->chargeCurrent = 0;
        status->dischargeCurrent = 0;
    }
    else
    {
        status->chargeFinished = false;
        status->wiredCharge = false;
        status->wirelessCharge = false;
        status->chargeCurrent = 0;
        status->dischargeCurrent = 0;
    }

    status->isValid = true;
    return PWR_ERROR_NONE;
}

void axp2101_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p)
{
    pmu_interface_p = pmu_if_p;

    pmu_p->isInitialized = &initialized;
    strncpy(pmu_p->InstanceName, "AXP2101", PMU_INSTANCE_NAME_MAX_LEN);

    pmu_p->Init = axp2101_init;
    pmu_p->Deinit = axp2101_deinit;
    pmu_p->Reset = axp2101_reset;
    pmu_p->Config = axp2101_config;
    pmu_p->Irq = axp2101_irq;
    pmu_p->SetState = axp2101_set_state;
    pmu_p->GetState = axp2101_get_state;
    pmu_p->GetStatus = axp2101_get_status;
}
