// own headers
#include "device_config.h"
#include "ecdsa.h"

// std library
#include <memory.h>

// sdk
#include "crc32.h"
#define NRF_LOG_MODULE_NAME DeviceConfig
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

// project library
#include "nrf_flash.h"
#include "nrf_uicr.h"
#include "util_macros.h"

#define EC_E_BOOL_R_BOOL(expr) ExecuteCheck_ADV(expr, true, { return false; })

static deviceCfg_t deviceConfig;
deviceCfg_t* deviceConfig_p = NULL;

// ======================
// Device Presistence Configs

// UICR 31 is used for battery profile flag
uint32_t devicePresistence_get_battery_profile_flag(void)
{
    return NRF_UICR->CUSTOMER[31];
}
bool devicePresistence_set_battery_profile_flag(uint32_t flag)
{
    return uicr_write((uint32_t)(&(NRF_UICR->CUSTOMER[31])), &flag, 1);
}

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

    // calculate crc32
    converted->crc32 = deviceCfg_keystore_crc32(converted);

    // backup to uicr
    if ( !deviceCfg_keystore_backup_to_uicr(converted) )
        return false;

    // do not wipe the legacy store, just in case

    return true;
}
#endif

static bool deviceCfg_keystore_write_to_uicr(deviceCfg_keystore_t* keystore)
{
    // write to uicr
    if ( !uicr_update_customer(0, (uint8_t*)(keystore), sizeof(deviceCfg_keystore_t)) )
        return false;

    // no need to wait busy as UICR programming is a blocking operation
    // we are not going to reboot, since we don't need it to be available ASAP
    return true;
}

static bool deviceCfg_keystore_read_from_uicr(deviceCfg_keystore_t* keystore)
{
    return uicr_get_customer(0, (uint8_t*)keystore, sizeof(deviceCfg_keystore_t));
}

uint32_t deviceCfg_keystore_crc32(deviceCfg_keystore_t* keystore)
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

bool deviceCfg_keystore_restore_from_uicr(deviceCfg_keystore_t* keystore)
{
    deviceCfg_keystore_t keystore_uicr;

    if ( !deviceCfg_keystore_read_from_uicr(&keystore_uicr) )
        return false;

    // check if update needed
    if ( memcmp(keystore, &keystore_uicr, sizeof(deviceCfg_keystore_t)) == 0 )
        return deviceCfg_keystore_validate(keystore);

    // check if both valid
    bool is_flash_keystore_valid = deviceCfg_keystore_validate(keystore);
    bool is_uicr_keystore_valid = deviceCfg_keystore_validate(&keystore_uicr);

    // if uicr copy in valid, no restore action
    if ( !is_uicr_keystore_valid )
        return false;

    // if uicr copy invalid, no flag check, restore only
    if ( is_uicr_keystore_valid && !is_flash_keystore_valid )
        memcpy(keystore, &keystore_uicr, sizeof(deviceCfg_keystore_t));

    // if both copy valid, flash copy check flag, no restore action if flash copy flag locked
    if ( is_uicr_keystore_valid && is_flash_keystore_valid )
        if ( keystore->flag_locked != DEVICE_CONFIG_FLAG_MAGIC )
            memcpy(keystore, &keystore_uicr, sizeof(deviceCfg_keystore_t));

    return true;
}

bool deviceCfg_keystore_backup_to_uicr(deviceCfg_keystore_t* keystore)
{
    deviceCfg_keystore_t keystore_uicr;

    // don't care the result, either empty or corrupted we still proceed.
    deviceCfg_keystore_read_from_uicr(&keystore_uicr);

    // check if both valid
    bool is_flash_keystore_valid = deviceCfg_keystore_validate(keystore);
    bool is_uicr_keystore_valid = deviceCfg_keystore_validate(&keystore_uicr);

    // if flash copy in valid, no backup action
    if ( !is_flash_keystore_valid )
        return false;

    // if uicr copy invalid, no flag check, backup only
    if ( is_flash_keystore_valid && !is_uicr_keystore_valid )
        if ( !deviceCfg_keystore_write_to_uicr(keystore) )
            return false;

    // if both copy valid, uicr copy check flag, no backup action if uicr copy flag locked
    if ( is_flash_keystore_valid && is_uicr_keystore_valid )
        if ( keystore_uicr.flag_locked != DEVICE_CONFIG_FLAG_MAGIC )
            if ( !deviceCfg_keystore_write_to_uicr(keystore) )
                return false;

    return true;
}

bool deviceCfg_keystore_backup_compare(deviceCfg_keystore_t* keystore)
{
    deviceCfg_keystore_t keystore_uicr;

    // don't care the result, either empty or corrupted we still proceed.
    deviceCfg_keystore_read_from_uicr(&keystore_uicr);

    // check if update needed
    return (memcmp(keystore, &keystore_uicr, sizeof(deviceCfg_keystore_t)) == 0);
}

bool deviceCfg_keystore_setup_new(deviceCfg_keystore_t* keystore)
{
    // set up new keypair
    generate_ecdsa_keypair(keystore->private_key, keystore->public_key);

    // calculate crc32
    keystore->crc32 = deviceCfg_keystore_crc32(keystore);

    // backup to uicr
    if ( !deviceCfg_keystore_backup_to_uicr(keystore) )
        return false;

    return true;
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

    // backup to uicr
    // not here, will be triggered next reboot

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

#if DEVICE_CONFIG_HANDLE_LEGACY
  #define DEVICE_CONFIG_LEGACY_ADDR 0x6D000U

static bool device_config_convert_legacy()
{
    deviceCfg_t devcfg_legacy;

    EC_E_BOOL_R_BOOL(flash_read(DEVICE_CONFIG_LEGACY_ADDR, (uint8_t*)(&devcfg_legacy), sizeof(deviceCfg_t)));

    if ( devcfg_legacy.header != DEVICE_CONFIG_HEADER_MAGIC || devcfg_legacy.version != DEVICE_CONFIG_VERSION )
        return false;

    memcpy(&deviceConfig, &devcfg_legacy, sizeof(deviceCfg_t));

    EC_E_BOOL_R_BOOL(flash_erase(DEVICE_CONFIG_LEGACY_ADDR, DEVICE_CONFIG_SIZE));

    return true;
}
#endif

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

    // try convert legacy
    if ( device_config_convert_legacy() )
    {
        NRF_LOG_INFO("Converted from legacy format!");
        // legacy found, commit
        EC_E_BOOL_R_BOOL(device_config_commit());
        NRF_LOG_INFO("Commited!");
    }

    // read
    EC_E_BOOL_R_BOOL(flash_read(DEVICE_CONFIG_ADDR, (uint8_t*)(&deviceConfig), sizeof(deviceCfg_t)))

    // check header
    // currently only one version of config layout, no upgrade method provided
    if ( deviceConfig.header != DEVICE_CONFIG_HEADER_MAGIC || deviceConfig.version != DEVICE_CONFIG_VERSION )
    {
        deviceConfig.header = DEVICE_CONFIG_HEADER_MAGIC;
        deviceConfig.version = DEVICE_CONFIG_VERSION;
        commit_pending = true;
    }

    // check keystore
    if ( !deviceCfg_keystore_validate(&(deviceConfig.keystore)) )
    {
        // current flash keystore invalid
        NRF_LOG_INFO("Keystore invalid!");

        do
        {
            // try restore from uicr
            if ( deviceCfg_keystore_restore_from_uicr(&(deviceConfig.keystore)) )
            {
                NRF_LOG_INFO("Keystore restored from uicr");
                commit_pending = true;
                break;
            }

            // try convert from legacy store
            if ( deviceCfg_keystore_convert_legacy(&(deviceConfig.keystore)) )
            {
                NRF_LOG_INFO("Keystore converted from legacy store");
                commit_pending = true;
                break;
            }

            // setup new
            if ( deviceCfg_keystore_setup_new(&(deviceConfig.keystore)) )
            {
                NRF_LOG_INFO("Keystore generated new keypair");
                commit_pending = true;
                break;
            }
        }
        while ( false );
    }
    else
        NRF_LOG_INFO("Keystore valid!");

    // check keystore backup
    if ( !deviceCfg_keystore_backup_compare(&(deviceConfig.keystore)) )
    {
        NRF_LOG_INFO("Keystore uicr compare missmatch, backup required");
        if ( deviceCfg_keystore_backup_to_uicr(&(deviceConfig.keystore)) )
        {
            NRF_LOG_INFO("Keystore backup to uicr succeed");
        }
        else
        {
            NRF_LOG_INFO("Keystore backup to uicr failed");
        }
    }
    else
    {
        NRF_LOG_INFO("Keystore uicr compare match, backup skiped");
    }

    // check settings
    if ( !deviceCfg_settings_validate(&(deviceConfig.settings)) )
    {
        NRF_LOG_INFO("Settings invalid!");
        deviceCfg_settings_setup(&(deviceConfig.settings));
        NRF_LOG_INFO("Settings created new!");
        commit_pending = true;
    }
    else
        NRF_LOG_INFO("Settings valid!");

    // commit (if any pending)
    if ( commit_pending )
    {
        EC_E_BOOL_R_BOOL(device_config_commit());
        NRF_LOG_INFO("Commited!");
    }

    // set interface pointer
    deviceConfig_p = &deviceConfig;

    return true;
}
