#ifndef _DEVICE_CONFIG_
#define _DEVICE_CONFIG_

#include <stdint.h>
#include <stdbool.h>

// ======================
// Device Configs Items

// *** key store ***
#define DEVICE_CONFIG_KEYSTORE_HANDLE_LEGACY 1
typedef struct
{
    uint32_t crc32; // PRIVATE AND PUBLIC
    uint32_t flag_locked;
    uint8_t private_key[32]; // 8*UINT32
    uint8_t public_key[64];  // 16*UINT32
} deviceCfg_keystore_t;
bool deviceCfg_keystore_validate(deviceCfg_keystore_t* keystore);
void deviceCfg_keystore_setup(deviceCfg_keystore_t* keystore);
bool deviceCfg_keystore_lock(deviceCfg_keystore_t* keystore);

// *** general settings ***
#define DEVICE_CONFIG_SETTING_HANDLE_LEGACY 1

typedef struct
{
    uint32_t flag_initialized;
    // DEVICE_CONFIG_FLAG_MAGIC to turn off any other value turns on
    uint32_t bt_ctrl;

} deviceCfg_settings_t;
bool deviceCfg_settings_validate(deviceCfg_settings_t* settings);
void deviceCfg_settings_setup(deviceCfg_settings_t* settings);

// ======================
// Device Configs

#define DEVICE_CONFIG_HEADER_MAGIC 0xAAAAAAAAU
#define DEVICE_CONFIG_FLAG_MAGIC   0xa55aa55aU
#define DEVICE_CONFIG_ADDR         0x6D000U
#define DEVICE_CONFIG_SIZE         0x1000U
#define DEVICE_CONFIG_VERSION      1U

typedef struct
{
    uint32_t header;
    uint32_t version;

    deviceCfg_keystore_t keystore;
    deviceCfg_settings_t settings;

} deviceCfg_t; // version 1 layout

extern deviceCfg_t* deviceConfig_p;

// bool device_config_validate(void);
bool device_config_commit(void);
bool device_config_init(void);

#endif //_DEVICE_CONFIG_