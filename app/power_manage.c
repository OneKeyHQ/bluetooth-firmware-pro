#include <memory.h>

#include "power_manage.h"

#include "nrf_delay.h"
#include "nrf_twi.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// workarounds to keep this file clean
static void (*send_stm_data_p)(uint8_t* pdata, uint8_t lenth);
void set_send_stm_data_p(void (*send_stm_data_p_)(uint8_t* pdata, uint8_t lenth))
{
    send_stm_data_p = send_stm_data_p_;
}

// ================================
// helpers
#define PRINT_CURRENT_LOCATION()                                    \
    {                                                               \
        NRF_LOG_INFO("%s:%d:%s", __FILE__, __LINE__, __FUNCTION__); \
        NRF_LOG_FLUSH();                                            \
    }

// ================================
// vars
static const nrf_drv_twi_t pwr_i2c_handler = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static bool pwr_i2c_configured = false;
static PMU_Interface_t pmu_if;
PMU_t* pmu_p = NULL;

// ================================
// functions private interface
static bool pmu_if_init()
{
    PRINT_CURRENT_LOCATION();

    if ( !pwr_i2c_configured )
    {
        const nrf_drv_twi_config_t twi_config = {
            .scl = TWI_SCL_M,
            .sda = TWI_SDA_M,
            .frequency = NRF_DRV_TWI_FREQ_400K,
            .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
            .clear_bus_init = true
        };
        if ( NRF_SUCCESS != nrf_drv_twi_init(&pwr_i2c_handler, &twi_config, NULL, NULL) )
        {
            return false;
        }
        nrf_drv_twi_enable(&pwr_i2c_handler);

        pwr_i2c_configured = true;
    }

    return true;
}

static bool pmu_if_deinit()
{
    PRINT_CURRENT_LOCATION();

    if ( pwr_i2c_configured )
    {

        nrf_drv_twi_disable(&pwr_i2c_handler);
        nrf_drv_twi_uninit(&pwr_i2c_handler);

        pwr_i2c_configured = false;
    }

    return true;
}

static bool pmu_if_reset()
{
    PRINT_CURRENT_LOCATION();

    // no impl.

    return true;
}

static bool pmu_if_send(const uint8_t device_addr, const uint32_t len, const uint8_t* const data)
{
    PRINT_CURRENT_LOCATION();

    // check i2c bus
    if ( !pwr_i2c_configured )
        return false;

    // wait busy
    while ( nrf_drv_twi_is_busy(&pwr_i2c_handler) )
        ;

    // send
    return (NRF_SUCCESS == nrf_drv_twi_tx(&pwr_i2c_handler, device_addr, data, len, false));
}

static bool pmu_if_receive(const uint8_t device_addr, const uint32_t len, uint8_t* const data)
{
    PRINT_CURRENT_LOCATION();

    // check i2c bus
    if ( !pwr_i2c_configured )
        return false;

    // wait busy
    while ( nrf_drv_twi_is_busy(&pwr_i2c_handler) )
        ;

    // read
    return (NRF_SUCCESS == nrf_drv_twi_rx(&pwr_i2c_handler, device_addr, data, len));
}

static void pmu_if_irq(const uint64_t irq)
{
    PRINT_CURRENT_LOCATION();

    if ( irq == 0 )
        return;

    // TODO: fix me! DECODE irq

    Power_Status_t status;
    pmu_p->GetStatus(&status);

    if ( 0 != (irq && (1 << PWR_IRQ_PWR_CONNECTED)) )
    {
        PRINT_CURRENT_LOCATION();
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PWR_DISCONNECTED)) )
    {
        PRINT_CURRENT_LOCATION();
    }
    if ( 0 != (irq && (1 << PWR_IRQ_CHARGING)) )
    {
        PRINT_CURRENT_LOCATION();
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
        send_stm_data_p(bak_buff, 3);
    }
    if ( 0 != (irq && (1 << PWR_IRQ_CHARGED)) )
    {
        PRINT_CURRENT_LOCATION();
    }
    if ( 0 != (irq && (1 << PWR_IRQ_BATT_LOW)) )
    {
        PRINT_CURRENT_LOCATION();
    }
    if ( 0 != (irq && (1 << PWR_IRQ_BATT_CRITICAL)) )
    {
        PRINT_CURRENT_LOCATION();
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PB_PRESS)) )
    {
        PRINT_CURRENT_LOCATION();
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x20;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PB_RELEASE)) )
    {
        PRINT_CURRENT_LOCATION();
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x40;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PB_SHORT)) )
    {
        PRINT_CURRENT_LOCATION();
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x01;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PB_LONG)) )
    {
        PRINT_CURRENT_LOCATION();
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x02;
        send_stm_data_p(bak_buff, 2);
    }
    if ( 0 != (irq && (1 << PWR_IRQ_PB_FORCEOFF)) )
    {
        PRINT_CURRENT_LOCATION();
    }
}

// reg
static bool pmu_if_reg_write(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t val)
{
    PRINT_CURRENT_LOCATION();

    uint8_t tx_buff[sizeof(reg_addr) + sizeof(val)];

    tx_buff[0] = reg_addr;
    tx_buff[1] = val;

    if ( !pmu_if_send(device_addr, sizeof(tx_buff), tx_buff) )
        return false;

    return true;
}

static bool pmu_if_reg_read(const uint8_t device_addr, const uint8_t reg_addr, uint8_t* const val)
{
    PRINT_CURRENT_LOCATION();

    uint8_t tmp = reg_addr;
    if ( !pmu_if_send(device_addr, sizeof(tmp), &tmp) )
        return false;
    if ( !pmu_if_receive(device_addr, sizeof(*val), val) )
        return false;

    return true;
}

static bool pmu_if_reg_set_bits(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask)
{
    PRINT_CURRENT_LOCATION();

    uint8_t reg_val;

    do
    {
        if ( !pmu_if_reg_read(device_addr, reg_addr, &reg_val) )
            break;

        if ( (reg_val & bit_mask) != bit_mask )
        {
            reg_val |= bit_mask;
            if ( !pmu_if_reg_write(device_addr, reg_addr, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool pmu_if_reg_clr_bits(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask)
{
    PRINT_CURRENT_LOCATION();

    uint8_t reg_val;
    do
    {
        if ( !pmu_if_reg_read(device_addr, reg_addr, &reg_val) )
            break;

        if ( reg_val & bit_mask )
        {
            reg_val &= ~bit_mask;
            if ( !pmu_if_reg_write(device_addr, reg_addr, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

// ================================
// functions public
bool power_manage_init()
{
    PRINT_CURRENT_LOCATION();

    // interface
    pmu_if.isInitialized = &pwr_i2c_configured;
    pmu_if.Init = pmu_if_init;
    pmu_if.Deinit = pmu_if_deinit;
    pmu_if.Reset = pmu_if_reset;
    pmu_if.Send = pmu_if_send;
    pmu_if.Receive = pmu_if_receive;
    pmu_if.Irq = pmu_if_irq;
    pmu_if.Reg.Write = pmu_if_reg_write;
    pmu_if.Reg.Read = pmu_if_reg_read;
    pmu_if.Reg.SetBits = pmu_if_reg_set_bits;
    pmu_if.Reg.ClrBits = pmu_if_reg_clr_bits;

    // pmu handle
    pmu_p = pmu_probe(&pmu_if);
    if ( pmu_p == NULL )
        return false;

    // init
    if ( !pmu_p->isInitialized )
        pmu_p->Init();

    return true;
}

bool power_manage_deinit()
{
    PRINT_CURRENT_LOCATION();

    // deinit
    if ( pmu_p->isInitialized )
        pmu_p->Deinit();

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