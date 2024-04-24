#include "axp216.h"

// macros

#define axp216_reg_read(reg, val)  pmu_interface_p->Reg.Read(AXP216_I2C_ADDR, reg, val)
#define axp216_reg_write(reg, val) pmu_interface_p->Reg.Write(AXP216_I2C_ADDR, reg, val)
#define axp216_set_bits(reg, mask) pmu_interface_p->Reg.SetBits(AXP216_I2C_ADDR, reg, mask)
#define axp216_clr_bits(reg, mask) pmu_interface_p->Reg.ClrBits(AXP216_I2C_ADDR, reg, mask)

// vars private
static bool initialized = false;
static PMU_Interface_t* pmu_interface_p = NULL;
static Power_State_t state_current = PWR_STATE_INVALID;

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

static bool axp216_config_control_parameter(void)
{
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_CHARGE1, 0x71));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_CHARGE2, 0x25));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN1, 0xFC));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN2, 0xCC));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN3, 0x00));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN4, 0x10));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_INTEN5, 0x78));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_ADC_CONTROL3, 0x36));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_IPS_SET, 0x60));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_POK_SET, 0x68));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_OFF_CTL, 0x4B));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_VOFF_SET, 0x13));

    return true;
}

static bool axp216_config_battery(void)
{
    // cap
    uint16_t value = (float)(530 / 1.456);
    // cap set flag
    value |= 0x80;
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP0, (uint8_t)(value >> 8)));
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP1, (uint8_t)value));

    // warn level
    EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_WARN, 0x55));

    // EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP0, 0x81));
    // EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_BAT_CAP1, 0x6C));

    return true;
}

static bool axp216_output_ctl(bool on_off)
{
    if ( on_off )
    {
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN2, 0x20));
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));
    }
    else
    {
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
        EC_E_BOOL_R_BOOL(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14));
        // bit 2:4 default 101, should not be changed
    }
    return true;
}

// function public

Power_Error_t axp216_init(void)
{

    if ( initialized )
        return PWR_ERROR_NONE;

    do
    {
        // interface init()
        if ( !pmu_interface_p->isInitialized )
            if ( !pmu_interface_p->Init() )
                break;

        // get id
        uint8_t val = 0;
        if ( !axp216_reg_read(AXP216_IC_TYPE, &val) )
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

    if ( !initialized )
        return PWR_ERROR_NONE;

    do
    {
        // interface deinit()
        if ( pmu_interface_p->isInitialized )
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
    uint8_t irqs[5];
    uint64_t irq_bits = 0;

    // read irq
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS1, &irqs[0]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS2, &irqs[1]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS3, &irqs[2]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS4, &irqs[3]));
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTSTS5, &irqs[4]));

    // translate irq
    irq_bits |= ((irqs[0] & (1 << 6)) << PWR_IRQ_PWR_CONNECTED);    // acin only, as vbus connected to acin
    irq_bits |= ((irqs[0] & (1 << 5)) << PWR_IRQ_PWR_DISCONNECTED); // acin only, as vbus connected to acin

    irq_bits |= ((irqs[1] & (1 << 3)) << PWR_IRQ_CHARGING);
    irq_bits |= ((irqs[1] & (1 << 2)) << PWR_IRQ_CHARGED);

    irq_bits |= ((irqs[3] & (1 << 1)) << PWR_IRQ_BATT_LOW);
    irq_bits |= ((irqs[3] & (1 << 0)) << PWR_IRQ_BATT_CRITICAL);

    irq_bits |= ((irqs[4] & (1 << 6)) << PWR_IRQ_PB_PRESS);
    irq_bits |= ((irqs[4] & (1 << 5)) << PWR_IRQ_PB_RELEASE);
    irq_bits |= ((irqs[4] & (1 << 4)) << PWR_IRQ_PB_SHORT);
    irq_bits |= ((irqs[4] & (1 << 3)) << PWR_IRQ_PB_LONG);
    irq_bits |= ((irqs[4] & (1 << 2)) << PWR_IRQ_PB_FORCEOFF);

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
    EC_E_BOOL_R_PWR_ERR(axp216_config_control_parameter());
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
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_OFF_CTL, (1 << 7)));
        break;
    case PWR_STATE_ON:
        // try wakeup anyways
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 5)));
        // config eveything
        EC_E_BOOL_R_PWR_ERR(axp216_config());
        // open output
        EC_E_BOOL_R_PWR_ERR(axp216_output_ctl(true));
        break;
    case PWR_STATE_SLEEP:
        // allow irq wakeup
        // EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 4)));
        // enable wakeup
        EC_E_BOOL_R_PWR_ERR(axp216_set_bits(AXP216_VOFF_SET, (1 << 3)));
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
    H8L4_Buff h8l4_conv;
    uint8_t tmp;

    memset(status, 0x00, sizeof(Power_Status_t));
    status->isValid = false;

    // battery percent
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_BAT_LEVEL, &tmp));
    if ( (tmp & 0x80) == 0x80 ) // is data valid
        status->batteryPercent = tmp & 0x7f;
    else
        status->batteryPercent = 100;

    // battery voltage
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VBATH_RES, &tmp));
    h8l4_conv.u8_high = tmp;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VBATL_RES, &tmp));
    h8l4_conv.u8_high = tmp & 0xf0;
    status->batteryVoltage = h8l4_conv.u16;

    // battery temp
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VTSH_RES, &tmp));
    h8l4_conv.u8_high = tmp;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_VTSL_RES, &tmp));
    h8l4_conv.u8_high = tmp & 0xf0;
    status->batteryTemp = h8l4_conv.u16;

    // pmu temp
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTTEMPH, &tmp));
    h8l4_conv.u8_high = tmp;
    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_INTTEMPL, &tmp));
    h8l4_conv.u8_high = tmp & 0xf0;
    status->pmuTemp = h8l4_conv.u16;

    // charging

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CHARGE1, &tmp));
    status->chargeAllowed = ((tmp & (1 << 7)) == (1 << 7));

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_STATUS, &tmp));
    status->chargerAvailable =
        (((tmp & (1 << 7) & (1 << 6)) == ((1 << 7) & (1 << 6))) || // acin
         ((tmp & (1 << 5) & (1 << 4)) == ((1 << 5) & (1 << 4)))    // vbus
        );

    if ( ((tmp & (1 << 2)) == (1 << 2)) ) // check if charging
    {
        // read gpio
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_GPIO1_CTL, 0b00000010)); // gpio1 input
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_GPIO01_SIGNAL, &tmp));    // gpio1 read
        EC_E_BOOL_R_PWR_ERR(axp216_reg_write(AXP216_GPIO1_CTL, 0b00000111)); // gpio1 float
        status->wirelessCharge = ((tmp & (1 << 1)) == (1 << 1));

        // if not wireless charging then it's wired
        status->wiredCharge = !status->wirelessCharge;
    }
    else
    {
        status->wiredCharge = false;
        status->wirelessCharge = false;
    }

    if ( status->wiredCharge || status->wirelessCharge )
    {
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CCBATH_RES, &tmp));
        h8l4_conv.u8_high = tmp;
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_CCBATL_RES, &tmp));
        h8l4_conv.u8_high = tmp & 0xf0;
        status->chargeCurrent = h8l4_conv.u16;
        status->dischargeCurrent = 0;
    }
    else
    {
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_DCBATH_RES, &tmp));
        h8l4_conv.u8_high = tmp;
        EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_DCBATL_RES, &tmp));
        h8l4_conv.u8_high = tmp & 0xf0;
        status->dischargeCurrent = h8l4_conv.u16;
        status->chargeCurrent = 0;
    }

    EC_E_BOOL_R_PWR_ERR(axp216_reg_read(AXP216_MODE_CHGSTATUS, &tmp));
    status->chargeFinished = ((tmp & (1 << 6)) == (1 << 6));

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

//     preg(AXP216_ALDO1OUT_VOL);
//     preg(AXP216_ALDO2OUT_VOL);
//     preg(AXP216_ALDO3OUT_VOL);
//     preg(AXP216_DC1OUT_VOL);

//     // ALDO1 -> LDO_1V8 1.8V
//     NRF_LOG_INFO("ALDO1 -> LDO_1V8 1.8V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
//     // ret = axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     // ALDO2 -> LDO_FB 3.3V
//     NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
//     // ret = axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a);
//     // ret = axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1C);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;static void pmu_sleep_ctrl(bool sleep_enable)
// {
//     NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);

//     if ( !i2c_configured )
//         return;

//     if ( sleep_enable )
//     {
//         // prepare sleep
//         axp_set_bits(AXP216_VOFF_SET, (1 << 3));
//         // power off (sleep)
//         power_off();
//     }
//     else
//     {
//         // wakeup
//         // axp_set_bits(AXP216_VOFF_SET, (1 << 5));

//         uint8_t reg_val;
//         NRF_EXEC_RETRY(axp216_read(AXP216_VOFF_SET, 1, &reg_val));

//         if ( (reg_val & (1 << 5)) != (1 << 5) )
//         {
//             reg_val |= (1 << 5);
//             NRF_EXEC_RETRY(axp216_reg_write(AXP216_VOFF_SET, reg_val));
//         }
//     }
// }

//     // ALDO3 -> LDO_NF 3.3V
//     NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
//     // ret = axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a);
//     // ret = axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1E);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     // DCDC1 -> MAIN 3.3V
//     NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));
//     // ret = axp216_reg_write(AXP216_DC1OUT_VOL, 0x11);
//     // if ( ret != NRF_SUCCESS )
//     //     return ret;

//     preg(AXP216_ALDO1OUT_VOL);
//     preg(AXP216_ALDO2OUT_VOL);
//     preg(AXP216_ALDO3OUT_VOL);
//     preg(AXP216_DC1OUT_VOL);

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

// #define NRF_EXEC_RETRY_ADV(EXPR, RETRY, DELAY_MS)
//     {
//         uint32_t retry = 1;
//         ret_code_t ret = NRF_SUCCESS;
//         while ( retry <= RETRY )
//         {
//             NRF_LOG_INFO("EXPR=%s, TRY=%lu/%lu", #EXPR, retry, RETRY);
//             ret = (EXPR);
//             if ( ret == NRF_SUCCESS )
//                 break;
//             else
//                 nrf_delay_ms(DELAY_MS);
//             retry++;
//         }
//         if ( ret != NRF_SUCCESS )
//             return ret;
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
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO1OUT_VOL, 0x0B));
//     // ALDO2 -> LDO_FB 3.3V
//     NRF_LOG_INFO("ALDO2 -> LDO_FB 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO2OUT_VOL, 0x1a));
//     // ALDO3 -> LDO_NF 3.3V
//     NRF_LOG_INFO("ALDO3 -> LDO_NF 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_ALDO3OUT_VOL, 0x1a));
//     // DCDC1 -> MAIN 3.3V
//     NRF_LOG_INFO("DCDC1 -> MAIN 3.3V");
//     NRF_LOG_FLUSH();
//     NRF_EXEC_RETRY(axp216_reg_write(AXP216_DC1OUT_VOL, 0x11));

//     // configure controls
//     if ( left_on )
//     {
//         NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN2, 0x20));
//         NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN1, 0xC2));
//     }
//     else
//     {
//         NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN1, 0x00));
//         NRF_EXEC_RETRY(axp216_reg_write(AXP216_LDO_DC_EN2, 0x14)); // bit 2:4 default 101, should not be
//         changed
//     }

//     // preg(AXP216_LDO_DC_EN1);
//     // preg(AXP216_LDO_DC_EN2);
//     // preg(AXP216_ALDO1OUT_VOL);
//     // preg(AXP216_ALDO2OUT_VOL);
//     // preg(AXP216_ALDO3OUT_VOL);
//     // preg(AXP216_DC1OUT_VOL);
// }
