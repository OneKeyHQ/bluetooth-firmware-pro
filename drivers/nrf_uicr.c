#include "nrf_uicr.h"

#include <memory.h>

#include "util_macros.h"

#include "nrfx_nvmc.h"

#define EC_E_NRFX_SUCCESS_R_BOOL(expr) ExecuteCheck_ADV(expr, NRFX_SUCCESS, { return false; })
#define EC_E_BOOL_R_BOOL(expr)         ExecuteCheck_ADV(expr, true, { return false; })

static bool uicr_validate_addr_range(uint32_t addr, uint32_t len_word)
{
    if ( addr % sizeof(uint32_t) != 0 )
        return false;

    if ( len_word == 0 )
        return false;

    if ( !(UICR_START <= addr) )
        return false;

    if ( !((addr + len_word * sizeof(uint32_t)) <= UICR_END) )
        return false;

    return true;
}

bool uicr_check_blank(uint32_t addr, uint32_t len_word)
{
    if ( !uicr_validate_addr_range(addr, len_word) )
        return false;

    for ( uint32_t i = 0; i < len_word; i++ )
    {
        if ( *((uint32_t*)(addr + i * sizeof(uint32_t))) != 0xffffffff )
            return false;
    }

    return true;
}

bool uicr_write(uint32_t addr, void* data, uint32_t len_word)
{
    if ( !uicr_validate_addr_range(addr, len_word) )
        return false;

    // if same then skip
    if ( memcmp((uint8_t*)addr, (uint8_t*)data, len_word * sizeof(uint32_t)) == 0 )
        return true;

    // check blank
    if ( !uicr_check_blank(addr, len_word) )
        return false;

    // write
    nrfx_nvmc_words_write(addr, (uint32_t*)data, len_word);

    // wait done
    while ( !nrfx_nvmc_write_done_check() ) {}

    return true;

    // uicr programs only effect after reboot
    // but it should be up to caller, not here
    // NVIC_SystemReset();
}

bool uicr_read(uint32_t addr, void* data, uint32_t len_word)
{
    if ( !uicr_validate_addr_range(addr, len_word) )
        return false;

    memcpy((uint8_t*)data, (uint8_t*)addr, len_word * sizeof(uint32_t));
    return true;
}

bool uicr_update_bootloader_addr(uint32_t bootloader_addr)
{
    // backup
    NRF_UICR_Type uicr_backup;
    EC_E_BOOL_R_BOOL(uicr_read(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type) / sizeof(uint32_t)));

    // copy over new customer region
    uicr_backup.NRFFW[0] = bootloader_addr;

    // wipe
    EC_E_NRFX_SUCCESS_R_BOOL(nrfx_nvmc_uicr_erase());

    // write back
    EC_E_BOOL_R_BOOL(uicr_write(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type) / sizeof(uint32_t)));

    return true; // remember to reboot !
}

bool uicr_update_customer(uint8_t offset, void* data, uint8_t len)
{
    if ( (offset % sizeof(uint32_t)) != 0 || (len % sizeof(uint32_t)) != 0 )
        return false;
    if ( (offset + len) > sizeof(NRF_UICR->CUSTOMER) )
        return false;

    // backup
    NRF_UICR_Type uicr_backup;
    EC_E_BOOL_R_BOOL(uicr_read(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type) / sizeof(uint32_t)));

    // copy over new customer region at offset
    memcpy((uint8_t*)(&(uicr_backup.CUSTOMER)) + offset, data, len);

    // wipe
    EC_E_NRFX_SUCCESS_R_BOOL(nrfx_nvmc_uicr_erase());

    // write back
    EC_E_BOOL_R_BOOL(uicr_write(UICR_START, &uicr_backup, sizeof(NRF_UICR_Type) / sizeof(uint32_t)));

    return true; // remember to reboot !
}

bool uicr_get_customer(uint8_t offset, void* data, uint8_t len)
{
    if ( (offset % sizeof(uint32_t)) != 0 || (len % sizeof(uint32_t)) != 0 )
        return false;
    if ( (offset + len) > sizeof(NRF_UICR->CUSTOMER) )
        return false;

    // check blank (only check the requested region)
    uint32_t start_addr = (uint32_t)(&(NRF_UICR->CUSTOMER)) + offset;
    if ( uicr_check_blank(start_addr, len / sizeof(uint32_t)) )
        return false;

    // read
    memcpy((uint8_t*)data, (uint8_t*)(&(NRF_UICR->CUSTOMER)) + offset, len);

    return true;
}