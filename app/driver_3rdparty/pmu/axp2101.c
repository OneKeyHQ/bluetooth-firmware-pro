#include "axp2101.h"

// vars private
static bool initialized = false;
static PMU_Interface_t* pmu_interface_p = NULL;
static Power_State_t state_current = PWR_STATE_INVALID;

// functions private

static bool axp2101_reg_read(const uint8_t reg, uint8_t* val)
{
    uint8_t tmp = reg;
    if ( !pmu_interface_p->Send(AXP_I2C_ADDR, sizeof(tmp), &tmp) )
        return false;
    if ( !pmu_interface_p->Receive(AXP_I2C_ADDR, sizeof(*val), val) )
        return false;

    return true;
}

static bool axp2101_reg_write(const uint8_t reg, const uint8_t val)
{
    uint8_t tx_buff[sizeof(reg) + sizeof(val)];

    tx_buff[0] = reg;
    tx_buff[1] = val;

    if ( !pmu_interface_p->Send(AXP_I2C_ADDR, sizeof(tx_buff), tx_buff) )
        return false;

    return true;
}

static bool axp2101_set_bits(int reg, uint8_t bit_mask)
{
    uint8_t reg_val;

    do
    {
        if ( !axp2101_reg_read(reg, &reg_val) )
            break;

        if ( (reg_val & bit_mask) != bit_mask )
        {
            reg_val |= bit_mask;
            if ( !axp2101_reg_write(reg, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool axp2101_clr_bits(int reg, uint8_t bit_mask)
{
    uint8_t reg_val;
    do
    {
        if ( !axp2101_reg_read(reg, &reg_val) )
            break;

        if ( reg_val & bit_mask )
        {
            reg_val &= ~bit_mask;
            if ( !axp2101_reg_write(reg, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool axp2101_config_voltage(void)
{
    //  voltages
    // ALDO1 -> RAIL_1V8 1.8V
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_ALDO1_CFG, 0x0D));
    // DCDC1 -> RAIL_3V3 3.3V
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC1_CFG, 0x14));
}

static bool axp2101_config_battery(void)
{
    // cap
    uint16_t value = (float)(530 / 1.456);
    // cap set flag
    value |= 0x80;
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP_BATCAP0, (uint8_t)(value >> 8)));
    EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP_BATCAP1, (uint8_t)value));
}

static bool axp2101_output_ctl(bool on_off)
{
    if ( on_off )
    {
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG0, 0x01));
        // EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_LDO_EN_CFG1, 0X00)); // no need
        EC_E_BOOL_R_BOOL(axp2101_reg_write(AXP2101_DCDC_CFG0, 0x01)); // dcdc1 on, other off
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
    do
    {
        // interface init()
        if ( !pmu_interface_p->Init() )
            break;

        // get id
        uint8_t val = 0;
        if ( !axp2101_reg_read(AXP2101_CHIP_ID, &val) )
            break;

        // compare id
        if ( val != 0x62 )
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
    EC_E_BOOL_R_PWR_ERR(axp2101_set_bits(AXP_VOFF_SET, (1 << 6)));
    return PWR_ERROR_NONE;
}

Power_Error_t axp2101_irq(void)
{
    uint8_t irqs[5];

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_INTSTS3, &irqs[2]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_INTSTS4, &irqs[3]));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_INTSTS5, &irqs[4]));

    // process irq
    // pmu_interface_p->Irq();
    // TODO: impl.

    // clear irq
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_INTSTS1, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_INTSTS2, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_INTSTS3, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_INTSTS4, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_INTSTS5, 0xFF));

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
    HL_Buff buffer;

    memset(status, 0x00, sizeof(Power_Status_t));
    status->isValid = false;

    // battery
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_SOC, &buffer.u8.low));
    status->batteryPercent = buffer.u8.low;

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_H, &buffer.u8.high));
    buffer.u8.high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_VBAT_L, &buffer.u8.low));
    status->batteryVoltage = buffer.u16;

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_H, &buffer.u8.high));
    buffer.u8.high &= 0b00111111; // drop bit 7:6
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TS_L, &buffer.u8.low));
    status->batteryTemp = buffer.u16;

    // pmu
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_H, &buffer.u8.high));
    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_TDIE_L, &buffer.u8.low));
    status->pmuTemp = buffer.u16;

    // charging

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP2101_MODULE_EN, &buffer.u8.low));
    status->chargeAllowed = ((buffer.u8.low & (1 << 1)) == (1 << 1));

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_STATUS, &buffer.u8.low));
    status->chargerAvailable =
        (((buffer.u8.low & (1 << 7) & (1 << 6)) == ((1 << 7) & (1 << 6))) || // acin
         ((buffer.u8.low & (1 << 5) & (1 << 4)) == ((1 << 5) & (1 << 4)))    // vbus
        );
    status->wiredCharge = ((buffer.u8.low & (1 << 2)) == (1 << 2));

    if ( !status->wiredCharge )
    {
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_GPIO1_CTL, 0b00000010));        // gpio1 input
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_GPIO01_SIGNAL, &buffer.u8.low)); // gpio1 read
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_write(AXP_GPIO1_CTL, 0b00000111));        // gpio1 float
        status->wirelessCharge = ((buffer.u8.low & (1 << 1)) == (1 << 1));
    }
    else
    {
        status->wirelessCharge = false;
    }

    if ( status->wiredCharge || status->wirelessCharge )
    {
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_CCBATH_RES, &buffer.u8.high));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_CCBATL_RES, &buffer.u8.low));
        status->chargeCurrent = buffer.u16;
        status->dischargeCurrent = 0;
    }
    else
    {
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_DCBATH_RES, &buffer.u8.high));
        EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_DCBATL_RES, &buffer.u8.low));
        status->dischargeCurrent = buffer.u16;
        status->chargeCurrent = 0;
    }

    EC_E_BOOL_R_PWR_ERR(axp2101_reg_read(AXP_MODE_CHGSTATUS, &buffer.u8.low));
    status->chargeFinished = ((buffer.u8.low & (1 << 6)) == (1 << 6));

    status->isValid = false;
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
