#include "nrf_i2c.h"

#include "util_micros.h"

#include "nrf_delay.h"
#include "nrf_twi.h"
#include "nrf_drv_twi.h"

static const nrf_drv_twi_t nrf_i2c_handle = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static bool i2c_configured = false;
static I2C_t i2c_handle = {NULL};

// ================================
// functions private

static bool nrf_i2c_init()
{
    PRINT_CURRENT_LOCATION();

    if ( !i2c_configured )
    {
        const nrf_drv_twi_config_t twi_config = {
            .scl = TWI_SCL_M,
            .sda = TWI_SDA_M,
            .frequency = NRF_DRV_TWI_FREQ_400K,
            .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
            .clear_bus_init = true
        };
        if ( NRF_SUCCESS != nrf_drv_twi_init(&nrf_i2c_handle, &twi_config, NULL, NULL) )
        {
            return false;
        }
        nrf_drv_twi_enable(&nrf_i2c_handle);

        i2c_configured = true;
    }

    return true;
}

static bool nrf_i2c_deinit()
{
    PRINT_CURRENT_LOCATION();

    if ( i2c_configured )
    {
        nrf_drv_twi_disable(&nrf_i2c_handle);
        nrf_drv_twi_uninit(&nrf_i2c_handle);

        i2c_configured = false;
    }

    return true;
}

static bool nrf_i2c_send(const uint8_t device_addr, const uint32_t len, const uint8_t* const data)
{
    PRINT_CURRENT_LOCATION();

    // check i2c bus
    if ( !i2c_configured )
        return false;

    // wait busy
    while ( nrf_drv_twi_is_busy(&nrf_i2c_handle) )
        ;

    // send
    return (NRF_SUCCESS == nrf_drv_twi_tx(&nrf_i2c_handle, device_addr, data, len, false));
}

static bool nrf_i2c_receive(const uint8_t device_addr, const uint32_t len, uint8_t* const data)
{
    PRINT_CURRENT_LOCATION();

    // check i2c bus
    if ( !i2c_configured )
        return false;

    // wait busy
    while ( nrf_drv_twi_is_busy(&nrf_i2c_handle) )
        ;

    // read
    return (NRF_SUCCESS == nrf_drv_twi_rx(&nrf_i2c_handle, device_addr, data, len));
}

// reg
static bool nrf_i2c_reg_write(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t val)
{
    PRINT_CURRENT_LOCATION();

    uint8_t tx_buff[sizeof(reg_addr) + sizeof(val)];

    tx_buff[0] = reg_addr;
    tx_buff[1] = val;

    if ( !nrf_i2c_send(device_addr, sizeof(tx_buff), tx_buff) )
        return false;

    return true;
}

static bool nrf_i2c_reg_read(const uint8_t device_addr, const uint8_t reg_addr, uint8_t* const val)
{
    PRINT_CURRENT_LOCATION();

    uint8_t tmp = reg_addr;
    if ( !nrf_i2c_send(device_addr, sizeof(tmp), &tmp) )
        return false;
    if ( !nrf_i2c_receive(device_addr, sizeof(*val), val) )
        return false;

    return true;
}

static bool nrf_i2c_reg_set_bits(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask)
{
    PRINT_CURRENT_LOCATION();

    uint8_t reg_val;

    do
    {
        if ( !nrf_i2c_reg_read(device_addr, reg_addr, &reg_val) )
            break;

        if ( (reg_val & bit_mask) != bit_mask )
        {
            reg_val |= bit_mask;
            if ( !nrf_i2c_reg_write(device_addr, reg_addr, reg_val) )
                break;
            ;
        }

        return true;
    }
    while ( false );
    return false;
}

static bool nrf_i2c_reg_clr_bits(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask)
{
    PRINT_CURRENT_LOCATION();

    uint8_t reg_val;
    do
    {
        if ( !nrf_i2c_reg_read(device_addr, reg_addr, &reg_val) )
            break;

        if ( reg_val & bit_mask )
        {
            reg_val &= ~bit_mask;
            if ( !nrf_i2c_reg_write(device_addr, reg_addr, reg_val) )
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
I2C_t* nrf_i2c_get_instance()
{
    PRINT_CURRENT_LOCATION();

    // interface
    i2c_handle.isInitialized = &i2c_configured;
    i2c_handle.Init = nrf_i2c_init;
    i2c_handle.Deinit = nrf_i2c_deinit;
    i2c_handle.Send = nrf_i2c_send;
    i2c_handle.Receive = nrf_i2c_receive;
    i2c_handle.Reg.Write = nrf_i2c_reg_write;
    i2c_handle.Reg.Read = nrf_i2c_reg_read;
    i2c_handle.Reg.SetBits = nrf_i2c_reg_set_bits;
    i2c_handle.Reg.ClrBits = nrf_i2c_reg_clr_bits;

    return &i2c_handle;
}