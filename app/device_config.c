#include "device_config.h"

#include <memory.h>

#include "util_micros.h"

#include "nrf_delay.h"
#include "nrf_uicr.h"
#include "nrf_flash.h"
#include "crc32.h"
#include "ecdsa.h"

// ======================
// keystore

static device_keystore_t deviceKeyStore;
device_keystore_t* deviceKeyStore_p = NULL;

#if DEVICE_CONFIG_KEYSTORE_HANDLE_LEGACY
  #define DEVICE_KEY_LEGACY_ADDR 0x71000

typedef struct
{
    uint32_t key_lock_flag;
    uint32_t key_flag;
    uint8_t private_key[32];
    uint8_t public_key[64];
} ecdsa_key_info_legacy_t;

static bool device_keystore_convert_legacy(device_keystore_t* converted)
{
    ecdsa_key_info_legacy_t key_store_legacy = {0};
    if ( flash_read(DEVICE_KEY_LEGACY_ADDR, &key_store_legacy, sizeof(ecdsa_key_info_legacy_t)) )
        return false;

    if ( key_store_legacy.key_flag != DEVICE_CONFIG_FLAG_MAGIC )
        return false;

    memcpy(&(converted->private_key), &(key_store_legacy.private_key), sizeof(converted->private_key));
    memcpy(&(converted->public_key), &(key_store_legacy.public_key), sizeof(converted->public_key));

    converted->crc32 = crc32_compute((uint8_t*)(&(converted->private_key)), sizeof(converted->private_key), NULL);
    converted->crc32 =
        crc32_compute((uint8_t*)(&(converted->public_key)), sizeof(converted->public_key), &(converted->crc32));

    // do not wipe the legacy store, just in case
    // rc = nrf_fstorage_erase(&fstorage, DEVICE_KEY_INFO_ADDR, FDS_PHY_PAGES_IN_VPAGE, NULL);
    // APP_ERROR_CHECK(rc);

    return true;
}
#endif

bool device_keystore_write(device_keystore_t* key_store)
{
    // no need to wait busy as UICR programming is a blocking operation
    return uicr_update_customer((uint8_t*)(&key_store), sizeof(device_keystore_t));
}

bool device_keystore_read(device_keystore_t* key_store)
{
    return uicr_read((uint32_t)(&(NRF_UICR->CUSTOMER)), (uint8_t*)(&key_store), sizeof(device_keystore_t));
}

uint32_t device_keystore_compute_crc32(device_keystore_t* key_store)
{
    uint32_t crc32 = 0;
    crc32 = crc32_compute((uint8_t*)(&(key_store->private_key)), sizeof(key_store->private_key), NULL);
    crc32 = crc32_compute((uint8_t*)(&(key_store->public_key)), sizeof(key_store->public_key), &crc32);

    return crc32;
}

bool device_keystore_validate(void)
{
    if ( !device_keystore_read(&deviceKeyStore) )
        return false;
    return (deviceKeyStore.crc32 == device_keystore_compute_crc32(&deviceKeyStore));
}

bool device_keystore_init(void)
{
    // deviceKeyStore_p = NULL;

    // check key store
    if ( device_keystore_validate() )
    {
        deviceKeyStore_p = &deviceKeyStore;
        return true;
    }

    // check if storage actually empty or is corrupted
    if ( !uicr_check_blank((uint32_t)(&(NRF_UICR->CUSTOMER)), sizeof(NRF_UICR->CUSTOMER)) )
        return false; // not empty, must be corrupted

    // current key store invalid
    if ( device_keystore_convert_legacy(&deviceKeyStore) )
    {
        // legacy store found, converted
        // nothing left to do
    }
    else
    {
        // no legacy store, setup new
        generate_ecdsa_keypair(deviceKeyStore.private_key, deviceKeyStore.public_key);
        deviceKeyStore.crc32 = device_keystore_compute_crc32(&deviceKeyStore);
    }

    // write key store
    if ( !device_keystore_write(&deviceKeyStore) )
        return false;

    // reboot to take effect
    NVIC_SystemReset();

    return true;
}

bool device_keystore_lock(void)
{
    // key store must be valid
    if ( !device_keystore_validate() )
        return false;

    // error out if already locked
    if ( deviceKeyStore.flag_locked == DEVICE_CONFIG_FLAG_MAGIC )
        return false;

    // lock
    deviceKeyStore.flag_locked = DEVICE_CONFIG_FLAG_MAGIC;

    // rewrite key store
    if ( !device_keystore_write(&deviceKeyStore) )
        return false;

    // reboot to take effect
    NVIC_SystemReset();

    return true;
}

// ======================
// general settings

// static uint8_t* device_config_flash_p = (uint8_t*)(DEVICE_CONFIG_ADDR);

static device_settings_t deviceSettings __attribute__((aligned(8)));
device_settings_t* deviceSettings_p = NULL;

#if DEVICE_CONFIG_SETTING_HANDLE_LEGACY
  #define BLE_CTL_ADDR 0x6f000
  #define BLE_CTL_OFF  0xABABABAB
  #define BLE_CTL_ON   0xBADC0FFE // don't care

static bool device_settings_convert_legacy(device_settings_t* converted)
{
    uint32_t bt_ctrl_legacy = 0;
    
    memcpy((uint8_t*)(&bt_ctrl_legacy), (uint32_t*)BLE_CTL_ADDR, sizeof(bt_ctrl_legacy));

    if ( bt_ctrl_legacy == 0xffffffff )
        // was never set
        return false;

    converted->flag_initialized = DEVICE_CONFIG_FLAG_MAGIC;

    if ( bt_ctrl_legacy == BLE_CTL_OFF )
        // was turned off
        converted->bt_ctrl = DEVICE_CONFIG_FLAG_MAGIC;
    else
        // was on or invalid value
        converted->bt_ctrl = 0;

    return true;
}
#endif

static bool device_config_update(uint8_t* data, uint32_t len)
{
    if ( len > DEVICE_CONFIG_SIZE )
        return false;

    // wipe
    if ( !flash_erase(DEVICE_CONFIG_ADDR, DEVICE_CONFIG_SIZE) )
        return false;

    // write
    if ( !flash_write(DEVICE_CONFIG_ADDR, data, len) )
        return false;

    // wait done
    flash_wait_busy();

    return true;
}

bool device_settings_validate(void)
{
    // read current
    if ( !flash_read(DEVICE_CONFIG_ADDR, (uint8_t*)(&deviceSettings), sizeof(device_settings_t)) )
        return false;

    return (deviceSettings.flag_initialized == DEVICE_CONFIG_FLAG_MAGIC);
}

bool device_settings_init(void)
{
    PRINT_CURRENT_LOCATION();

    deviceSettings_p = NULL;

    flash_init();

    // check settings
    if ( !device_settings_validate() )
    {
        // no valid settings found
        if ( !device_settings_convert_legacy(&deviceSettings) )
        {
            // no legacy store, setup new
            memset(&deviceSettings, 0xff, sizeof(device_settings_t));
            deviceSettings.flag_initialized = DEVICE_CONFIG_FLAG_MAGIC;
        }

        // commit
        if ( device_settings_commit() )
            return false;
    }

    deviceSettings_p = &deviceSettings;
    return true;
}

bool device_settings_commit(void)
{
    return device_config_update((uint8_t*)(&deviceSettings), sizeof(device_settings_t));
}