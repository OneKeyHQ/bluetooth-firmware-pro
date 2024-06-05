#ifndef _I2C_COMMON_
#define _I2C_COMMON_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool* isInitialized;
    bool (*Init)(void);
    bool (*Deinit)(void);
    void (*Reset)(void);
    void (*HighDriveStrengthCtrl)(bool enable);
    bool (*Send)(const uint8_t device_addr, const uint32_t len, const uint8_t* const data); // iic host send
    bool (*Receive)(const uint8_t device_addr, const uint32_t len, uint8_t* const data);    // iic host receive

    struct
    {
        bool (*Write)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t data);
        bool (*Read)(const uint8_t device_addr, const uint8_t reg_addr, uint8_t* const data);
        bool (*SetBits)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask);
        bool (*ClrBits)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask);

    } Reg;

} I2C_t;

#endif // _I2C_COMMON_