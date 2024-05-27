#include "device_config.h"

#include <memory.h>

#include "crc32.h"

#include "util_macros.h"
#include "nrf_flash.h"
#include "ecdsa.h"

#define EC_E_BOOL_R_BOOL(expr) ExecuteCheck_ADV(expr, true, { return false; })

static deviceCfg_t deviceConfig;
deviceCfg_t* deviceConfig_p = NULL;

// ======================
// Device Configs Items

// *** key store ***

#if DEVICE_CONFIG_KEYSTORE_HANDLE_LEGACY
  #define DEVICE_KEY_LEGACY_ADDR 0x71000

typedef struct
{
    uint32_t key_lock_flag;
    uint32_t key_flag;
    uint8_t private_key[32];
    uint8_t public_key[64];
} ecdsa_key_info_legacy_t;

static bool deviceCfg_keystore_convert_legacy(deviceCfg_keystore_t* converted)
{
    ecdsa_key_info_legacy_t* keystore_legacy = (ecdsa_key_info_legacy_t*)DEVICE_KEY_LEGACY_ADDR;

    if ( keystore_legacy->key_flag != DEVICE_CONFIG_FLAG_MAGIC )
        return false;

    converted->flag_locked = keystore_legacy->key_lock_flag;

    memcpy(&(converted->private_key), &(keystore_legacy->private_key), sizeof(converted->private_key));
    memcpy(&(converted->public_key), &(keystore_legacy->public_key), sizeof(converted->public_key));

    // do not wipe the legacy store, just in case

    return true;
}
#endif

static uint32_t deviceCfg_keystore_crc32(deviceCfg_keystore_t* keystore)
{
    uint32_t crc32 = 0;
    crc32 = crc32_compute((uint8_t*)(&(keystore->private_key)), sizeof(keystore->private_key), NULL);
    crc32 = crc32_compute((uint8_t*)(&(keystore->public_key)), sizeof(keystore->public_key), &crc32);

    return crc32;
}

bool deviceCfg_keystore_validate(deviceCfg_keystore_t* keystore)
{
    return (keystore->crc32 == deviceCfg_keystore_crc32(keystore));
}

void deviceCfg_keystore_setup(deviceCfg_keystore_t* keystore)
{
    // try convert legacy
    if ( !deviceCfg_keystore_convert_legacy(keystore) )
    {
        // no legacy found, setup new
        generate_ecdsa_keypair(keystore->private_key, keystore->public_key);
    }
    // calculate crc32
    keystore->crc32 = deviceCfg_keystore_crc32(keystore);
}

bool deviceCfg_keystore_lock(deviceCfg_keystore_t* keystore)
{
    // key store must be valid
    if ( !deviceCfg_keystore_validate(keystore) )
        return false;

    // error out if already locked
    if ( keystore->flag_locked == DEVICE_CONFIG_FLAG_MAGIC )
        return false;

    // lock
    keystore->flag_locked = DEVICE_CONFIG_FLAG_MAGIC;

    return true;
}

// *** general settings ***

#if DEVICE_CONFIG_SETTING_HANDLE_LEGACY
  #define BLE_CTL_ADDR 0x6f000
  #define BLE_CTL_OFF  0xABABABAB
  #define BLE_CTL_ON   0xBADC0FFE // don't care

static bool deviceCfg_settings_convert_legacy(deviceCfg_settings_t* converted)
{
    uint32_t bt_ctrl_legacy = *((uint32_t*)BLE_CTL_ADDR);

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

bool deviceCfg_settings_validate(deviceCfg_settings_t* settings)
{
    EC_E_BOOL_R_BOOL(settings->flag_initialized == DEVICE_CONFIG_FLAG_MAGIC);

    return true;
}

void deviceCfg_settings_setup(deviceCfg_settings_t* settings)
{
    // try convert legacy
    if ( !deviceCfg_settings_convert_legacy(settings) )
    {
        // no legacy found, setup new
        settings->flag_initialized = DEVICE_CONFIG_FLAG_MAGIC;
        settings->bt_ctrl = 0;
    }
}

// ======================
// Device Configs

bool device_config_commit(void)
{
    EC_E_BOOL_R_BOOL(sizeof(deviceCfg_t) < DEVICE_CONFIG_SIZE);

    EC_E_BOOL_R_BOOL(flash_erase(DEVICE_CONFIG_ADDR, DEVICE_CONFIG_SIZE));
    EC_E_BOOL_R_BOOL(flash_write(DEVICE_CONFIG_ADDR, &deviceConfig, sizeof(deviceCfg_t)));
    flash_wait_busy();

    return true;
}

bool device_config_init(void)
{
    bool commit_pending = false;

    // flash init
    EC_E_BOOL_R_BOOL(flash_init());

    // read
    EC_E_BOOL_R_BOOL(flash_read(DEVICE_CONFIG_ADDR, (uint8_t*)(&deviceConfig), sizeof(deviceCfg_t)))

    // check header
    // currently only one version of config layout, so not upgrade provided
    if ( deviceConfig.header != DEVICE_CONFIG_HEADER_MAGIC || deviceConfig.version != DEVICE_CONFIG_VERSION )
    {
        deviceConfig.header = DEVICE_CONFIG_HEADER_MAGIC;
        deviceConfig.version = DEVICE_CONFIG_VERSION;
        commit_pending = true;
    }

    // check keystore
    if ( !deviceCfg_keystore_validate(&(deviceConfig.keystore)) )
    {
        deviceCfg_keystore_setup(&(deviceConfig.keystore));
        commit_pending = true;
    }

    // check settings
    if ( !deviceCfg_settings_validate(&(deviceConfig.settings)) )
    {
        deviceCfg_settings_setup(&(deviceConfig.settings));
        commit_pending = true;
    }

    // commit (if any pending)
    if ( commit_pending )
        EC_E_BOOL_R_BOOL(device_config_commit());

    // set interface pointer
    deviceConfig_p = &deviceConfig;

    return true;
}
