#include "lm36011.h"

#include "util_macros.h"
#include "nrf_i2c.h"

// vars private
static I2C_t* i2c_handle = NULL;

// functions private

static bool flashled_if_ensure_ready()
{
    if ( i2c_handle == NULL )
        i2c_handle = nrf_i2c_get_instance();

    if ( !i2c_handle->isInitialized )
        if ( !i2c_handle->Init() )
            return false;

    return true;
}

// function public

ret_code_t lm36011_write(const uint8_t writeAddr, const uint8_t writeData)
{
    if ( !flashled_if_ensure_ready() )
        return NRF_ERROR_INTERNAL;

    return ((i2c_handle->Reg.Write(LM36011_DEVICES_ADDR, writeAddr, writeData) ? NRF_SUCCESS : NRF_ERROR_INTERNAL));
}

ret_code_t lm36011_read(uint8_t readAddr, uint8_t byteNum, uint8_t* readData)
{
    if ( !flashled_if_ensure_ready() )
        return NRF_ERROR_INTERNAL;

    // ignored as this function only used for read reg, which always 1 byte
    UNUSED_VAR(byteNum);

    return ((i2c_handle->Reg.Read(LM36011_DEVICES_ADDR, readAddr, readData) ? NRF_SUCCESS : NRF_ERROR_INTERNAL));
}
