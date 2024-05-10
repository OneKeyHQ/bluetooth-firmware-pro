#include "nrf_uicr.h"

#include <memory.h>

#include "util_micros.h"
#include "nrfx_nvmc.h"
// #include "nrf_nvmc.h"

#define EC_E_NRFX_SUCCESS_R_BOOL(expr)         ExecuteCheck_ADV(expr, NRFX_SUCCESS, { return false; })
#define EC_E_BOOL_R_BOOL(expr)         ExecuteCheck_ADV(expr, true, { return false; })


static inline bool uicr_validate_addr_range(uint32_t addr_start, uint32_t addr_end)
{
    return ((UICR_START <= addr_start) && (addr_end <= UICR_END));
}

bool uicr_check_blank(uint32_t addr, uint32_t len)
{
    if ( !uicr_validate_addr_range(addr, addr + len) )
        return false;

    for ( uint32_t i = 0; i < len; i++ )
    {
        if ( *((uint8_t*)addr) != 0xff )
            return false;
    }
    return true;
}

bool uicr_write(uint32_t addr, void* data, uint32_t len)
{
    if ( !uicr_validate_addr_range(addr, addr + len) )
        return false;

    // if same then skip
    if ( memcmp((uint32_t*)addr, data, len) == 0 )
        return true;

    // check blank
    if ( !uicr_check_blank(addr, len) )
        return false;

    // write
    nrfx_nvmc_bytes_write(addr, data, len);

    return true;

    // uicr programs effecte only after reboot
    // but it should be up to caller, not here
    // NVIC_SystemReset();
}

bool uicr_read(uint32_t addr, void* data, uint32_t len)
{
    if ( !uicr_validate_addr_range(addr, addr + len) )
        return false;

    memcpy((uint8_t*)data, (uint8_t*)addr, len);
    return true;
}

bool uicr_update_customer(void* data, uint8_t len)
{
    if ( len > sizeof(NRF_UICR->CUSTOMER) )
        return false;

    // backup
    NRF_UICR_Type uicr_backup;
    EC_E_BOOL_R_BOOL(uicr_read(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type)) )

    // wipe
    EC_E_NRFX_SUCCESS_R_BOOL(nrfx_nvmc_uicr_erase());

    // check blank
    if ( !uicr_check_blank(UICR_START, UICR_SIZE) )
        return false;

    // copy over new customer region
    memcpy((uint8_t*)(&(uicr_backup.CUSTOMER)), data, len);

    // write back
    if ( !uicr_write(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type)) )
        return false;

    return true; // remember to reboot !
}