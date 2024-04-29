#include "axp2101.h"

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
    // cap
    uint16_t value = (float)(530 / 1.456);
    // cap set flag
    value |= 0x80;
    // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP216_BAT_CAP0, (uint8_t)(value >> 8)));
    // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP216_BAT_CAP1, (uint8_t)value));

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
    uint8_t irqs[5];

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_INTSTS3, &irqs[2]));

    // process irq
    // pmu_interface_p->Irq();
    // TODO: impl.

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
    H6L8_Buff h6l8_conv;
    uint8_t tmp;

    memset(status, 0x00, sizeof(Power_Status_t));
    status->isValid = false;

    // battery percent
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_SOC, &tmp));
    status->batteryPercent = tmp & 0x7f; // drop bit 7

    // battery voltage
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_H, &(h6l8_conv.u8_high)));
    h6l8_conv.u8_high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_L, &(h6l8_conv.u8_low)));
    status->batteryVoltage = h6l8_conv.u16; // TODO: CALCULATE ACTUAL VALUE

    // battery temp
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_H, &(h6l8_conv.u8_high)));
    h6l8_conv.u8_high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_L, &(h6l8_conv.u8_low)));
    status->batteryVoltage = h6l8_conv.u16; // TODO: CALCULATE ACTUAL VALUE

    // pmu temp
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_H, &(h6l8_conv.u8_high)));
    h6l8_conv.u8_high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_L, &(h6l8_conv.u8_low)));
    status->pmuTemp = h6l8_conv.u16; // TODO: CALCULATE ACTUAL VALUE

    // charging

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_MODULE_EN, &(h6l8_conv.u8_low)));
    status->chargeAllowed = ((h6l8_conv.u8_low & (1 << 1)) == (1 << 1));

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT0, &(h6l8_conv.u8_low)));
    status->chargerAvailable = ((h6l8_conv.u8_low & (1 << 5)) == (1 << 5)); // vbus good

    // check if charging
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT1, &(h6l8_conv.u8_low)));
    if ( (h6l8_conv.u8_low & 0b01100000) == 0b00100000 ) // bit 6:5 = 01
    {
        // read gpio
        // TODO: impl.
        status->wirelessCharge = ((h6l8_conv.u8_low & (1 << 1)) == (1 << 1));

        // if not wireless charging then it's wired
        status->wiredCharge = !status->wirelessCharge;
    }
    else
    {
        status->wiredCharge = false;
        status->wirelessCharge = false;
    }

    // current measure not supported
    status->chargeCurrent = 0;
    status->dischargeCurrent = 0;

    // check if battery standby
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_COMM_STAT1, &(h6l8_conv.u8_low)));
    status->chargeFinished = ((h6l8_conv.u8_low & 0b01100000) == 0b00000000); // bit 6:5 = 00

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
