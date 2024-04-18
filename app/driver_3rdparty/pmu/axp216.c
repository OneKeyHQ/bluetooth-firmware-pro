#include "axp216.h"

// vars private
static bool initialized = false;
static PMU_Interface_t* pmu_interface_p = NULL;
static Power_State_t state_current = PWR_STATE_INVALID;

// functions private

static bool axp216_reg_read(const uint8_t reg, uint8_t* val)
{
    uint8_t tmp = reg;
    if ( !pmu_interface_p->Send(AXP_I2C_ADDR, sizeof(tmp), &tmp) )
        return false;
    if ( !pmu_interface_p->Receive(AXP_I2C_ADDR, sizeof(*val), val) )
        return false;

    return true;
}

static bool axp216_reg_write(const uint8_t reg, const uint8_t val)
{
    uint8_t tx_buff[sizeof(reg) + sizeof(val)];

    tx_buff[0] = reg;
    tx_buff[1] = val;

    if ( !pmu_interface_p->Send(AXP_I2C_ADDR, sizeof(tx_buff), tx_buff) )
        return false;

    return true;
}

static bool axp216_set_bits(int reg, uint8_t bit_mask)
{
    uint8_t reg_val;

    do
    {
        if ( !axp216_reg_read(reg, &reg_val) )
            break;

        if ( (reg_val & bit_mask) != bit_mask )
        {
            reg_val |= bit_mask;
            if ( !axp216_reg_write(reg, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool axp216_clr_bits(int reg, uint8_t bit_mask)
{
    uint8_t reg_val;
    do
    {
        if ( !axp216_reg_read(reg, &reg_val) )
            break;

        if ( reg_val & bit_mask )
        {
            reg_val &= ~bit_mask;
            if ( !axp216_reg_write(reg, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool axp216_config_voltage(void)
{
    //  voltages
    // ALDO1 -> LDO_1V8 1.8V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_ALDO1OUT_VOL, 0x0B));
    // ALDO2 -> LDO_FB 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_ALDO2OUT_VOL, 0x1a));
    // ALDO3 -> LDO_NF 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_ALDO3OUT_VOL, 0x1a));
    // DCDC1 -> MAIN 3.3V
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_DC1OUT_VOL, 0x11));
}

static bool axp216_config_battery(void)
{
    // cap
    uint16_t value = (float)(530 / 1.456);
    // cap set flag
    value |= 0x80;
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_BATCAP0, (uint8_t)(value >> 8)));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_BATCAP1, (uint8_t)value));
}

static bool axp216_output_ctl(bool on_off)
{
    if ( on_off )
    {
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_LDO_DC_EN2, 0x20));
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_LDO_DC_EN1, 0xC2));
    }
    else
    {
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_LDO_DC_EN1, 0x00));
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP_LDO_DC_EN2, 0x14));
        // bit 2:4 default 101, should not be changed
    }
    return true;
}

// function public

Power_Error_t axp216_init(void)
{
    do
    {
        // interface init()
        if ( !pmu_interface_p->Init() )
            break;

        // get id
        uint8_t val = 0;
        if ( !axp216_reg_read(AXP_IC_TYPE, &val) )
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

Power_Error_t axp216_deinit(void)
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

Power_Error_t axp216_reset(void)
{
    EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP_VOFF_SET, (1 << 6)));
    return PWR_ERROR_NONE;
}

Power_Error_t axp216_irq(void)
{
    uint8_t irqs[5];

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTSTS3, &irqs[2]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTSTS4, &irqs[3]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTSTS5, &irqs[4]));

    // process irq
    // pmu_interface_p->Irq();
    // TODO: impl.

    // clear irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_INTSTS1, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_INTSTS2, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_INTSTS3, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_INTSTS4, 0xFF));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_INTSTS5, 0xFF));

    return PWR_ERROR_NONE;
}

Power_Error_t axp216_config(void)
{
    EC_E_BOOL_R_PWR_ERR(axp216_config_voltage());
    EC_E_BOOL_R_PWR_ERR(axp216_config_battery());

    return PWR_ERROR_NONE;
}

Power_Error_t axp216_set_state(const Power_State_t state)
{
    switch ( state )
    {
    case PWR_STATE_OFF:
        // close output
        EC_E_BOOL_R_PWR_ERR(axp216_output_ctl(false));
        // pmu off
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP_OFF_CTL, (1 << 7)));
        break;
    case PWR_STATE_ON:
        // try wakeup anyways
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP_VOFF_SET, (1 << 5)));
        // config eveything
        EC_E_BOOL_R_PWR_ERR(axp216_config());
        // open output
        EC_E_BOOL_R_PWR_ERR(axp216_output_ctl(true));
        break;
    case PWR_STATE_SLEEP:
        // allow irq wakeup
        // EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP_VOFF_SET, (1 << 4)));
        // enable wakeup
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP_VOFF_SET, (1 << 3)));
        // close output ("sleep")
        EC_E_BOOL_R_PWR_ERR(axp216_output_ctl(false));
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

Power_Error_t axp216_get_status(Power_Status_t* status)
{
    HL_Buff buffer;

    memset(status, 0x00, sizeof(Power_Status_t));
    status->isValid = false;

    // battery
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_CAP, &buffer.u8.low));
    if ( (buffer.u8.low & 0x80) == 0x80 ) // is data valid
        status->batteryPercent = buffer.u8.low & 0x7f;
    else
        status->batteryPercent = 100;

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_VBATH_RES, &buffer.u8.high));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_VBATL_RES, &buffer.u8.low));
    status->batteryVoltage = buffer.u16;

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_VTSH_RES, &buffer.u8.high));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_VTSL_RES, &buffer.u8.low));
    status->batteryTemp = buffer.u16;

    // pmu
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTTEMPH, &buffer.u8.high));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_INTTEMPL, &buffer.u8.low));
    status->pmuTemp = buffer.u16;

    // charging

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_CHARGE1, &buffer.u8.low));
    status->chargeAllowed = ((buffer.u8.low & (1 << 7)) == (1 << 7));

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_STATUS, &buffer.u8.low));
    status->chargerAvailable =
        (((buffer.u8.low & (1 << 7) & (1 << 6)) == ((1 << 7) & (1 << 6))) || // acin
         ((buffer.u8.low & (1 << 5) & (1 << 4)) == ((1 << 5) & (1 << 4)))    // vbus
        );

    // TODO: change the logic, detect wireless charge first
    status->wiredCharge = ((buffer.u8.low & (1 << 2)) == (1 << 2));

    if ( !status->wiredCharge )
    {
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_GPIO1_CTL, 0b00000010));        // gpio1 input
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_GPIO01_SIGNAL, &buffer.u8.low)); // gpio1 read
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP_GPIO1_CTL, 0b00000111));        // gpio1 float
        status->wirelessCharge = ((buffer.u8.low & (1 << 1)) == (1 << 1));
    }
    else
    {
        status->wirelessCharge = false;
    }

    if ( status->wiredCharge || status->wirelessCharge )
    {
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_CCBATH_RES, &buffer.u8.high));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_CCBATL_RES, &buffer.u8.low));
        status->chargeCurrent = buffer.u16;
        status->dischargeCurrent = 0;
    }
    else
    {
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_DCBATH_RES, &buffer.u8.high));
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_DCBATL_RES, &buffer.u8.low));
        status->dischargeCurrent = buffer.u16;
        status->chargeCurrent = 0;
    }

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP_MODE_CHGSTATUS, &buffer.u8.low));
    status->chargeFinished = ((buffer.u8.low & (1 << 6)) == (1 << 6));

    status->isValid = false;
    return PWR_ERROR_NONE;
}

void axp216_setup_interface(PMU_Interface_t* pmu_if_p, PMU_t* pmu_p)
{
    pmu_interface_p = pmu_if_p;

    pmu_p->isInitialized = &initialized;
    strncpy(pmu_p->InstanceName, "AXP216", PMU_INSTANCE_NAME_MAX_LEN);

    pmu_p->Init = axp216_init;
    pmu_p->Deinit = axp216_deinit;
    pmu_p->Reset = axp216_reset;
    pmu_p->Config = axp216_config;
    pmu_p->Irq = axp216_irq;
    pmu_p->SetState = axp216_set_state;
    pmu_p->GetState = axp216_get_state;
    pmu_p->GetStatus = axp216_get_status;
}

// static ret_code_t power_pre_config()
// {
//     NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);
//     NRF_LOG_FLUSH();

//     ENSURE_I2C_AVAILABLE()

//     preg(AXP_ALDO1OUT_VOL);
//     preg(AXP_ALDO2OUT_VOL);
//     preg(AXP_ALDO3OUT_VOL);
//     preg(AXP_DC1OUT_VOL);

//     // ALDO1 -> LDO_1V8 1.8V
//     NRF_LOG_INFO("ALDO1 -> LDO_1V8 1.8V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO1OUT_VOL, 0x0B));
//     // ret = axp216_reg_write(AXP_ALDO1OUT_VOL, 0x0B);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     // ALDO2 -> LDO_FB 3.3V
//     NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO2OUT_VOL, 0x1a));
//     // ret = axp216_reg_write(AXP_ALDO2OUT_VOL, 0x1a);
//     // ret = axp216_reg_write(AXP_ALDO2OUT_VOL, 0x1C);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;static void pmu_sleep_ctrl(bool sleep_enable)
// {
//     NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);

//     if ( !i2c_configured )
//         return;

//     if ( sleep_enable )
//     {
//         // prepare sleep
//         axp_set_bits(AXP_VOFF_SET, (1 << 3));
//         // power off (sleep)
//         power_off();
//     }
//     else
//     {
//         // wakeup
//         // axp_set_bits(AXP_VOFF_SET, (1 << 5));

//         uint8_t reg_val;
//         NRF_EXEC_RETRY(axp216_read(AXP_VOFF_SET, 1, &reg_val));

//         if ( (reg_val & (1 << 5)) != (1 << 5) )
//         {
//             reg_val |= (1 << 5);
//             NRF_EXEC_RETRY(axp216_reg_write(AXP_VOFF_SET, reg_val));
//         }
//     }
// }

//     // ALDO3 -> LDO_NF 3.3V
//     NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO3OUT_VOL, 0x1a));
//     // ret = axp216_reg_write(AXP_ALDO3OUT_VOL, 0x1a);
//     // ret = axp216_reg_write(AXP_ALDO3OUT_VOL, 0x1E);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     // DCDC1 -> MAIN 3.3V
//     NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_DC1OUT_VOL, 0x11));
//     // ret = axp216_reg_write(AXP_DC1OUT_VOL, 0x11);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     preg(AXP_ALDO1OUT_VOL);
//     preg(AXP_ALDO2OUT_VOL);
//     preg(AXP_ALDO3OUT_VOL);
//     preg(AXP_DC1OUT_VOL);

//     return NRF_SUCCESS;
// }

// static void print_axp_reg(char* reg_name, uint8_t reg_addr)
// {
//     ret_code_t ret = 0;
//     uint8_t val = 0x5a;
//     axp216_read(reg_addr, 1, &val);
//     NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
//     NRF_LOG_INFO("%s(0x%02x)=0x%02x", reg_name, reg_addr, val);
// }
// #define preg(x) print_axp_reg(#x, x)

// #define NRF_EXEC_RETRY_ADV(EXPR, RETRY, DELAY_MS)                      \
//     {                                                                  \
//         uint32_t retry = 1;                                            \
//         ret_code_t ret = NRF_SUCCESS;                                  \
//         while ( retry <= RETRY )                                       \
//         {                                                              \
//             NRF_LOG_INFO("EXPR=%s, TRY=%lu/%lu", #EXPR, retry, RETRY); \
//             ret = (EXPR);                                              \
//             if ( ret == NRF_SUCCESS )                                  \
//                 break;                                                 \
//             else                                                       \
//                 nrf_delay_ms(DELAY_MS);                                \
//             retry++;                                                   \
//         }                                                              \
//         if ( ret != NRF_SUCCESS )                                      \
//             return ret;                                                \
//     }

// #define NRF_EXEC_RETRY(EXPR) NRF_EXEC_RETRY_ADV(EXPR, 10, 10)

// static ret_code_t power_config_aio(bool left_on)
// {
//     NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);
//     NRF_LOG_FLUSH();

//     ENSURE_I2C_AVAILABLE();

//     // configure voltages
//     // ALDO1 -> LDO_1V8 1.8V
//     NRF_LOG_INFO("ALDO1 -> LDO_1V8 1.8V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO1OUT_VOL, 0x0B));
//     // ALDO2 -> LDO_FB 3.3V
//     NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO2OUT_VOL, 0x1a));
//     // ALDO3 -> LDO_NF 3.3V
//     NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_ALDO3OUT_VOL, 0x1a));
//     // DCDC1 -> MAIN 3.3V
//     NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP_DC1OUT_VOL, 0x11));

//     // configure controls
//     if ( left_on )
//     {
//         NRF_EXEC_RETRY(axp216_reg_write(AXP_LDO_DC_EN2, 0x20));
//         NRF_EXEC_RETRY(axp216_reg_write(AXP_LDO_DC_EN1, 0xC2));
//     }
//     else
//     {
//         NRF_EXEC_RETRY(axp216_reg_write(AXP_LDO_DC_EN1, 0x00));
//         NRF_EXEC_RETRY(axp216_reg_write(AXP_LDO_DC_EN2, 0x14)); // bit 2:4 default 101, should not be
//         changed
//     }

//     // preg(AXP_LDO_DC_EN1);
//     // preg(AXP_LDO_DC_EN2);
//     // preg(AXP_ALDO1OUT_VOL);
//     // preg(AXP_ALDO2OUT_VOL);
//     // preg(AXP_ALDO3OUT_VOL);
//     // preg(AXP_DC1OUT_VOL);
// }
