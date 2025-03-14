#ifndef __PMU_COMMON_H_
#define __PMU_COMMON_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "util_macros.h"

// ================================
// defines
#define PMU_INSTANCE_NAME_MAX_LEN 16

// ================================
// macros
#define EC_PWR_ERR_ADV(expr, on_false) ExecuteCheck_ADV(expr, PWR_ERROR_NONE, on_false)
#define EC_BOOL_ADV(expr, on_false)    ExecuteCheck_ADV(expr, true, on_false)

#define EC_E_BOOL_R_PWR_ERR(expr)      ExecuteCheck_ADV(expr, true, { return PWR_ERROR_FAIL; })
#define EC_E_BOOL_R_BOOL(expr)         ExecuteCheck_ADV(expr, true, { return false; })

#define PWR_ENUM_ITEM(b, c)            JOIN_EXPR(PWR, b, c)
// regex -> (PWR_ENUM_ITEM\((.*), (.*)\).*,).*
// replace -> $1 // PWR_$2_$3

// ================================
// types

typedef union
{
    uint16_t u16;

    struct
    {
        uint8_t u8_low;
        uint8_t u8_high;
    } __attribute__((packed, aligned(1)));

} HL_Buff;

typedef enum
{
    PWR_ENUM_ITEM(ERROR, NONE) = 0,   // PWR_ERROR_NONE
    PWR_ENUM_ITEM(ERROR, FAIL),       // PWR_ERROR_FAIL
    PWR_ENUM_ITEM(ERROR, USAGE),      // PWR_ERROR_USAGE
    PWR_ENUM_ITEM(ERROR, NO_SUPPORT), // PWR_ERROR_NO_SUPPORT
} Power_Error_t;

typedef enum
{
    PWR_ENUM_ITEM(STATE, INVALID) = -1, // PWR_STATE_INVALID
    PWR_ENUM_ITEM(STATE, SOFT_OFF),     // PWR_STATE_SOFT_OFF
    PWR_ENUM_ITEM(STATE, HARD_OFF),     // PWR_STATE_HARD_OFF
    PWR_ENUM_ITEM(STATE, ON),           // PWR_STATE_ON
    PWR_ENUM_ITEM(STATE, SLEEP),        // PWR_STATE_SLEEP
    PWR_ENUM_ITEM(STATE, WAKEUP),       // PWR_STATE_WAKEUP
} Power_State_t;

typedef enum
{
    PWR_ENUM_ITEM(FEAT, INVALID) = -1, // PWR_FEAT_INVALID
    PWR_ENUM_ITEM(FEAT, CHARGE),       // PWR_FEAT_CHARGE
} Power_Featrue_t;

typedef enum
{
    PWR_ENUM_ITEM(IRQ, INVALID) = -1,     // PWR_IRQ_INVALID
    PWR_ENUM_ITEM(IRQ, PWR_CONNECTED),    // PWR_IRQ_PWR_CONNECTED
    PWR_ENUM_ITEM(IRQ, PWR_DISCONNECTED), // PWR_IRQ_PWR_DISCONNECTED
    PWR_ENUM_ITEM(IRQ, CHARGING),         // PWR_IRQ_CHARGING
    PWR_ENUM_ITEM(IRQ, CHARGED),          // PWR_IRQ_CHARGED
    PWR_ENUM_ITEM(IRQ, BATT_LOW),         // PWR_IRQ_BATT_LOW
    PWR_ENUM_ITEM(IRQ, BATT_CRITICAL),    // PWR_IRQ_BATT_CRITICAL
    PWR_ENUM_ITEM(IRQ, PB_PRESS),         // PWR_IRQ_PB_PRESS
    PWR_ENUM_ITEM(IRQ, PB_RELEASE),       // PWR_IRQ_PB_RELEASE
    PWR_ENUM_ITEM(IRQ, PB_SHORT),         // PWR_IRQ_PB_SHORT
    PWR_ENUM_ITEM(IRQ, PB_LONG),          // PWR_IRQ_PB_LONG
    PWR_ENUM_ITEM(IRQ, PB_FORCEOFF),      // PWR_IRQ_PB_FORCEOFF

    PWR_ENUM_ITEM(IRQ, PMU_OVER_TEMP),     // PWR_IRQ_PMU_OVER_TEMP
    PWR_ENUM_ITEM(IRQ, BATT_OVER_TEMP),    // PWR_IRQ_BATT_OVER_TEMP
    PWR_ENUM_ITEM(IRQ, BATT_UNDER_TEMP),   // PWR_IRQ_BATT_UNDER_TEMP
    PWR_ENUM_ITEM(IRQ, BATT_OVER_VOLTAGE), // PWR_IRQ_BATT_OVER_VOLTAGE
    PWR_ENUM_ITEM(IRQ, CHARGE_TIMEOUT),    // PWR_IRQ_CHARGE_TIMEOUT
} Power_Irq_t;

typedef enum
{
    PWR_ENUM_ITEM(GPIO_Config, DEFAULT) = 0, // PWR_GPIO_Config_DEFAULT
    PWR_ENUM_ITEM(GPIO_Config, READ_NP),     // PWR_GPIO_Config_READ_NP
    PWR_ENUM_ITEM(GPIO_Config, READ_PH),     // PWR_GPIO_Config_READ_PH
    PWR_ENUM_ITEM(GPIO_Config, READ_PL),     // PWR_GPIO_Config_READ_PL
    PWR_ENUM_ITEM(GPIO_Config, WRITE_NP),    // PWR_GPIO_Config_WRITE_NP
    PWR_ENUM_ITEM(GPIO_Config, WRITE_PH),    // PWR_GPIO_Config_WRITE_PH
    PWR_ENUM_ITEM(GPIO_Config, WRITE_PL),    // PWR_GPIO_Config_WRITE_PL
    PWR_ENUM_ITEM(GPIO_Config, UNUSED),      // PWR_GPIO_Config_UNUSED
} Power_GPIO_Config_t;

typedef enum
{
    PWR_ENUM_ITEM(LOG_LEVEL, OFF) = 0, // PWR_LOG_LEVEL_OFF
    PWR_ENUM_ITEM(LOG_LEVEL, ERR),     // PWR_LOG_LEVEL_ERR
    PWR_ENUM_ITEM(LOG_LEVEL, WARN),    // PWR_LOG_LEVEL_WARN
    PWR_ENUM_ITEM(LOG_LEVEL, INFO),    // PWR_LOG_LEVEL_INFO
    PWR_ENUM_ITEM(LOG_LEVEL, DBG),     // PWR_LOG_LEVEL_DBG
    PWR_ENUM_ITEM(LOG_LEVEL, TRACE),   // PWR_LOG_LEVEL_TRACE
} Power_LogLevel_t;

typedef struct
{
    uint16_t sysVoltage;
    bool batteryPresent;
    uint8_t batteryPercent;
    uint16_t batteryVoltage;
    int16_t batteryTemp;

    uint16_t pmuTemp;

    bool chargeAllowed;
    bool chargerAvailable;
    bool chargeFinished;
    bool wiredCharge;
    bool wirelessCharge;
    uint16_t chargeCurrent;
    uint16_t dischargeCurrent;

    uint64_t irqSnapshot; // snapshot, not pulled every loop, for debug only

} Power_Status_t;

typedef struct
{
    bool* isInitialized;
    bool (*Init)(void);
    bool (*Deinit)(void);
    void (*Reset)(void);
    void (*HighDriveStrengthCtrl)(bool);
    bool (*Send)(const uint8_t device_addr, const uint32_t len, const uint8_t* const data); // iic host send
    bool (*Receive)(const uint8_t device_addr, const uint32_t len, uint8_t* const data);    // iic host receive
    void (*Irq)(const uint64_t irq);                                                        // passed irq out

    struct
    {
        bool (*Write)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t data);
        bool (*Read)(const uint8_t device_addr, const uint8_t reg_addr, uint8_t* const data);
        bool (*SetBits)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask);
        bool (*ClrBits)(const uint8_t device_addr, const uint8_t reg_addr, const uint8_t bit_mask);

    } Reg;

    struct
    {
        bool (*Config)(uint32_t pin_num, const Power_GPIO_Config_t config);
        bool (*Write)(uint32_t pin_num, const bool high_low);
        bool (*Read)(uint32_t pin_num, bool* const high_low);
    } GPIO;

    void (*Delay_ms)(const uint32_t ms);
    void (*Log)(const Power_LogLevel_t level, const char* fmt, ...);

} PMU_Interface_t;

typedef struct
{
    bool* isInitialized;
    char InstanceName[PMU_INSTANCE_NAME_MAX_LEN];
    Power_Status_t* PowerStatus;
    Power_Error_t (*Init)(void);
    Power_Error_t (*Deinit)(void);
    Power_Error_t (*Reset)(bool hard_reset);
    Power_Error_t (*Config)(void);
    Power_Error_t (*Irq)(void); // irq call in
    Power_Error_t (*SetState)(const Power_State_t state);
    Power_Error_t (*GetState)(Power_State_t* state);
    Power_Error_t (*PullStatus)(void);
    Power_Error_t (*SetFeature)(Power_Featrue_t feature, bool enable);
    Power_Error_t (*GetFeature)(Power_Featrue_t feature, bool* enable);
} PMU_t;

#if PMU_IF_FUNCTION_TEMPLATES
static void pmu_if_log(Power_LogLevel_t level, const char* fmt, ...)
{
    char log_buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);

    switch ( level )
    {
    case PWR_LOG_LEVEL_ERR:
        LOG_ERROR(log_buffer);
        break;
    case PWR_LOG_LEVEL_WARN:
        LOG_WARNING(log_buffer);
        break;
    case PWR_LOG_LEVEL_INFO:
        LOG_INFO(log_buffer);
        break;
    case PWR_LOG_LEVEL_DBG:
        LOG_DEBUG(log_buffer);
        break;
    case PWR_LOG_LEVEL_TRACE:
        LOG_TRACE(log_buffer);
        break;

    case PWR_LOG_LEVEL_OFF:
        break;
    default:
        break;
    }
}
#endif

#endif //__PMU_COMMON_H_