#include <memory.h>

#include "power_manage.h"

#include "nrf_i2c.h"

// #include "nrf_delay.h"
// #include "nrf_gpio.h"
// #include "nrf_drv_gpiote.h"

// workarounds to keep this file clean
static void (*send_stm_data_p)(uint8_t* pdata, uint8_t lenth);
void set_send_stm_data_p(void (*send_stm_data_p_)(uint8_t* pdata, uint8_t lenth))
{
    send_stm_data_p = send_stm_data_p_;
}

// ================================
// vars
static PMU_Interface_t pmu_if;
PMU_t* pmu_p = NULL;

// ================================
// functions private

static bool pmu_if_reset()
{
    PRINT_CURRENT_LOCATION();

    // no impl.

    return true;
}

static void pmu_if_irq(const uint64_t irq)
{

    if ( irq == 0 )
        return;
    Power_Status_t status;
    pmu_p->GetStatus(&status);
    if ( 0 != (irq & (1 << PWR_IRQ_PWR_CONNECTED)) ) {}
    if ( 0 != (irq & (1 << PWR_IRQ_PWR_DISCONNECTED)) ) {}
    if ( 0 != (irq & (1 << PWR_IRQ_CHARGING)) )
    {
        bak_buff[0] = BLE_CMD_POWER_STA;
        bak_buff[1] = status.chargerAvailable;
        if ( status.wiredCharge )
        {
            bak_buff[2] = AXP_CHARGE_TYPE_USB;
        }
        else
        {
            bak_buff[2] = AXP_CHARGE_TYPE_WIRELESS;
        }
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq & (1 << PWR_IRQ_CHARGED)) ) {}
    if ( 0 != (irq & (1 << PWR_IRQ_BATT_LOW)) ) {}
    if ( 0 != (irq & (1 << PWR_IRQ_BATT_CRITICAL)) ) {}
    if ( 0 != (irq & (1 << PWR_IRQ_PB_PRESS)) )
    {
        NRF_LOG_INFO("short  irq PWR_IRQ_PB_PRESS ");
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x20;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq & (1 << PWR_IRQ_PB_RELEASE)) ) // PWR_IRQ_PB_RELEASE
    {
        NRF_LOG_INFO("short  irq PWR_IRQ_PB_RELEASE ");
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x40;
        send_stm_data_p(bak_buff, 2);
    }
    // NRF_LOG_INFO("short  irq PWR_IRQ_PB_SHORT %d",PWR_IRQ_PB_SHORT);
    if ( 0 != (irq & (1 << PWR_IRQ_PB_SHORT)) )
    {

        NRF_LOG_INFO("short  irq PWR_IRQ_PB_SHORT ");
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x01;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq & (1 << PWR_IRQ_PB_LONG)) )
    {

        NRF_LOG_INFO("short  irq PWR_IRQ_PB_LONG ");
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x02;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq & (1 << PWR_IRQ_PB_FORCEOFF)) ) {}
}

// ================================
// functions public
bool power_manage_init()
{
    PRINT_CURRENT_LOCATION();

    I2C_t* i2c_handle = nrf_i2c_get_instance();

    // interface
    pmu_if.isInitialized = i2c_handle->isInitialized;
    pmu_if.Init = i2c_handle->Init;
    pmu_if.Deinit = i2c_handle->Deinit;
    pmu_if.Reset = pmu_if_reset;
    pmu_if.Send = i2c_handle->Send;
    pmu_if.Receive = i2c_handle->Receive;
    pmu_if.Irq = pmu_if_irq;
    pmu_if.Reg.Write = i2c_handle->Reg.Write;
    pmu_if.Reg.Read = i2c_handle->Reg.Read;
    pmu_if.Reg.SetBits = i2c_handle->Reg.SetBits;
    pmu_if.Reg.ClrBits = i2c_handle->Reg.ClrBits;

    // pmu handle
    pmu_p = pmu_probe(&pmu_if);
    if ( pmu_p == NULL )
        return false;

    // init
    if ( pmu_p->Init() != PWR_ERROR_NONE )
        return false;

    return true;
}

bool power_manage_deinit()
{
    PRINT_CURRENT_LOCATION();

    // deinit
    if ( pmu_p->Deinit() != PWR_ERROR_NONE )
        return false;

    // pmu handle
    pmu_p = NULL;

    // interface
    memset(&pmu_if, 0x00, sizeof(PMU_Interface_t));

    return true;
}

#if notused

void in_gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch ( pin )
    {
    case POWER_IC_OK_IO:
        NRF_LOG_INFO("POWER_IC_OK_IO");
        NRF_LOG_FLUSH();

        if ( action == NRF_GPIOTE_POLARITY_TOGGLE )
        {
            NRF_LOG_INFO("NRF_GPIOTE_POLARITY_TOGGLE");
            NRF_LOG_FLUSH();

            // debounce
            nrf_delay_ms(1);

            if ( nrf_gpio_pin_read(pin) )
            {
                NRF_LOG_INFO("PWR_OK HIGH!");
                NRF_LOG_FLUSH();

                power_config_aio(true);

                NRF_LOG_INFO("PWR_OK HIGH! exit");
                NRF_LOG_FLUSH();
            }
            else
            {
                NRF_LOG_INFO("PWR_OK LOW!");
                NRF_LOG_FLUSH();

                if ( i2c_configured )
                    i2c_control(false);
            }
        }
        else if ( action == NRF_GPIOTE_POLARITY_LOTOHI )
        {
            NRF_LOG_INFO("NRF_GPIOTE_POLARITY_LOTOHI");
            NRF_LOG_FLUSH();
        }
        else if ( action == NRF_GPIOTE_POLARITY_HITOLO )
        {
            NRF_LOG_INFO("NRF_GPIOTE_POLARITY_HITOLO");
            NRF_LOG_FLUSH();
        }

        break;

    case POWER_IC_IRQ_IO:
        NRF_LOG_INFO("POWER_IC_IRQ_IO");
        NRF_LOG_FLUSH();
        break;

    default:
        break;
    }
}

static void gpiote_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // power ok
    nrf_drv_gpiote_in_config_t in_config_toggle = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config_toggle.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(POWER_IC_OK_IO, &in_config_toggle, in_gpiote_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(POWER_IC_OK_IO, true);

    // power irq
    nrf_drv_gpiote_in_config_t in_config_hl = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config_hl.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(POWER_IC_IRQ_IO, &in_config_hl, in_gpiote_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(POWER_IC_IRQ_IO, true);
}

// main
NRF_LOG_INFO("Configuring GPIO");
NRF_LOG_FLUSH();
gpiote_init();

NRF_LOG_INFO("Check Power Status");
if ( nrf_gpio_pin_read(POWER_IC_OK_IO) )
{
    i2c_control(true);

    NRF_LOG_INFO("Power seems on...");

    NRF_LOG_INFO("Configuring Power Initial");
    NRF_LOG_FLUSH();

    power_config_aio(true);

    NRF_LOG_INFO("Configuring Power Initial Done");
    NRF_LOG_FLUSH();

    i2c_control(false);
}

#endif