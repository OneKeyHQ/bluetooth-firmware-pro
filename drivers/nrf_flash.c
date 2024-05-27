#include "nrf_flash.h"

#include "util_macros.h"

#include "nrf_delay.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

// is this really needed?
static void flash_evt_handler(nrf_fstorage_evt_t* p_evt)
{
    if ( p_evt->result != NRF_SUCCESS )
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch ( p_evt->id )
    {
    case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.", p_evt->len, p_evt->addr);
        }
        break;

    case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.", p_evt->len, p_evt->addr);
        }
        break;

    default:
        break;
    }
}

static NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage_config) = {
    .evt_handler = flash_evt_handler,
    .start_addr = 0x6D000,
    .end_addr = 0x6E000,
};

static nrf_fstorage_api_t* fstor_api = NULL;

bool flash_init(void)
{
    fstor_api = &nrf_fstorage_sd;
    APP_ERROR_CHECK(nrf_fstorage_init(&fstorage_config, fstor_api, NULL));
    return true;
}

bool flash_deinit(void)
{
    APP_ERROR_CHECK(nrf_fstorage_uninit(&fstorage_config, NULL));
    fstor_api = NULL;
    return true;
}

void flash_wait_busy(void)
{
    while ( nrf_fstorage_is_busy(&fstorage_config) )
    {
        nrf_delay_ms(10);
    }
}

bool flash_check_blank(uint32_t addr, uint32_t len)
{
    for ( uint32_t i = 0; i < len; i++ )
    {
        if ( *((uint8_t*)addr) != 0xff )
            return false;
    }
    return true;
}

bool flash_erase(uint32_t address, uint32_t len)
{
    uint32_t erase_page_count = len / fstorage_config.p_flash_info->erase_unit;
    if ( erase_page_count < 0 )
        return false;
    return (nrf_fstorage_erase(&fstorage_config, address, erase_page_count, NULL) == NRF_SUCCESS);
}

bool flash_read(uint32_t address, void* data, uint32_t len)
{
    return (nrf_fstorage_read(&fstorage_config, address, data, len) == NRF_SUCCESS);
}

bool flash_write(uint32_t address, void* data, uint32_t len)
{
    return (nrf_fstorage_write(&fstorage_config, address, data, len, NULL) == NRF_SUCCESS);
}
