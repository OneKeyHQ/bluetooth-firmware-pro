#ifndef __PMU_COMMON_H_
#define __PMU_COMMON_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PMU_INSTANCE_NAME_MAX_LEN 16

#define ExecuteCheck_ADV(expr, expected_result, on_false) \
    {                                                     \
        typeof(expected_result) ret = (expr);             \
        if ( ret != (expected_result) )                   \
        {                                                 \
            on_false                                      \
        }                                                 \
    }
#define EC_PWR_ERR_ADV(expr, on_false) ExecuteCheck_ADV(expr, PWR_ERROR_NONE, on_false)
#define EC_BOOL_ADV(expr, on_false)    ExecuteCheck_ADV(expr, true, on_false)

#define EC_E_BOOL_R_PWR_ERR(expr)      ExecuteCheck_ADV(expr, true, { return PWR_ERROR_FAIL; })
#define EC_E_BOOL_R_BOOL(expr)         ExecuteCheck_ADV(expr, true, { return false; })

#define JOIN_EXPR(a, b, c)             a##_##b##_##c
// regex ->(JOIN_EXPR\((.*), (.*), (.*)\).*,)
// replace -> $1 // $2_$3_$4
#define PWR_ENUM_ITEM(b, c) JOIN_EXPR(PWR, b, c)
// regex ->(PWR_ENUM_ITEM\((.*), (.*)\).*,)
// replace -> $1 // PWR_$2_$3

typedef union
{
    uint16_t u16;
    struct
    {
        uint8_t high;
        uint8_t low;
    } u8;
} HL_Buff;

typedef enum
{
    PWR_ENUM_ITEM(ERROR, NONE) = 0, // PWR_ERROR_NONE
    PWR_ENUM_ITEM(ERROR, FAIL),     // PWR_ERROR_FAIL
    PWR_ENUM_ITEM(ERROR, USAGE),    // PWR_ERROR_USAGE
} Power_Error_t;

typedef enum
{
    PWR_ENUM_ITEM(STATE, INVALID) = -1, // PWR_STATE_INVALID
    PWR_ENUM_ITEM(STATE, OFF),          // PWR_STATE_OFF
    PWR_ENUM_ITEM(STATE, ON),           // PWR_STATE_ON
    PWR_ENUM_ITEM(STATE, SLEEP),        // PWR_STATE_SLEEP
} Power_State_t;

typedef enum
{
    PWR_ENUM_ITEM(IRQ, INVALID) = -1,  // PWR_IRQ_INVALID
    PWR_ENUM_ITEM(IRQ, PB_SHORT),      // PWR_IRQ_PB_SHORT
    PWR_ENUM_ITEM(IRQ, PB_LONG),       // PWR_IRQ_PB_LONG
    PWR_ENUM_ITEM(IRQ, PB_RELEASE),    // PWR_IRQ_PB_RELEASE
    PWR_ENUM_ITEM(IRQ, PB_PRESS),      // PWR_IRQ_PB_PRESS
    PWR_ENUM_ITEM(IRQ, CHARGING),      // PWR_IRQ_CHARGING
    PWR_ENUM_ITEM(IRQ, DISCHARGING),   // PWR_IRQ_DISCHARGING
    PWR_ENUM_ITEM(IRQ, BATT_LOW),      // PWR_IRQ_BATT_LOW
    PWR_ENUM_ITEM(IRQ, BATT_CRITICAL), // PWR_IRQ_BATT_CRITICAL
} Power_Irq_t;

typedef struct
{
    bool isValid;

    uint8_t batteryPercent;
    uint16_t batteryVoltage;
    uint16_t batteryTemp;

    uint16_t pmuTemp;

    bool chargeAllowed;
    bool chargerAvailable;
    bool chargeFinished;
    bool wiredCharge;
    bool wirelessCharge;
    uint16_t chargeCurrent;
    uint16_t dischargeCurrent;

} Power_Status_t;

typedef struct
{
    bool* isInitialized;
    bool (*Init)(void);
    bool (*Deinit)(void);
    bool (*Reset)(void);
    bool (*Send)(const uint8_t device_addr, const uint32_t len, uint8_t* data);
    bool (*Receive)(const uint8_t device_addr, const uint32_t len, uint8_t* data);
    void (*Irq)(const Power_Irq_t irq); // passed irq out
} PMU_Interface_t;

typedef struct
{
    bool* isInitialized;
    char InstanceName[PMU_INSTANCE_NAME_MAX_LEN];
    Power_Error_t (*Init)(void);
    Power_Error_t (*Deinit)(void);
    Power_Error_t (*Reset)(void);
    Power_Error_t (*Config)(void);
    Power_Error_t (*Irq)(void); // irq call in
    Power_Error_t (*SetState)(const Power_State_t state);
    Power_Error_t (*GetState)(Power_State_t* state);
    Power_Error_t (*GetStatus)(Power_Status_t* status);
} PMU_t;

#endif //__PMU_COMMON_H_