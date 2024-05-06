#ifndef _DEVICE_CONFIG_
#define _DEVICE_CONFIG_

#include <stdint.h>
#include <stdbool.h>

#define DEVICE_CONFIG_FLAG_MAGIC 0xa55aa55a

// ======================
// keystore

#define DEVICE_CONFIG_KEYSTORE_HANDLE_LEGACY 1

typedef struct
{
    uint32_t flag_locked;
    uint8_t private_key[32]; // 8*UINT32
    uint8_t public_key[64];  // 16*UINT32
    uint32_t crc32;          // PRIVATE AND PUBLIC
} device_keystore_t;

extern device_keystore_t* deviceKeyStore_p;

// bool device_keystore_write(device_keystore_t* key_store);
// bool device_keystore_read(device_keystore_t* key_store);
bool device_keystore_validate(void);
bool device_keystore_init(void);
bool device_keystore_lock(void);

// ======================
// general settings

#define DEVICE_CONFIG_ADDR                  0x6D000
#define DEVICE_CONFIG_SIZE                  0x1000

#define DEVICE_CONFIG_SETTING_HANDLE_LEGACY 1

typedef struct
{
    uint32_t flag_initialized;

    // DEVICE_CONFIG_FLAG_MAGIC to turn off
    // any other value turns on
    uint32_t bt_ctrl;

} device_settings_t;

extern device_settings_t* deviceSettings_p;

bool device_settings_validate(void);
bool device_settings_init(void);
bool device_settings_commit(void);

#endif //_DEVICE_CONFIG_