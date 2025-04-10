#ifndef _UTIL_MACROS_H_
#define _UTIL_MACROS_H_

#define UNUSED_VAR(X) ((void)(X))

#define BYTE_TO_BINARY_PATTERN "0b%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                   \
    ((byte) & 0x80 ? '1' : '0'), ((byte) & 0x40 ? '1' : '0'), ((byte) & 0x20 ? '1' : '0'),     \
        ((byte) & 0x10 ? '1' : '0'), ((byte) & 0x08 ? '1' : '0'), ((byte) & 0x04 ? '1' : '0'), \
        ((byte) & 0x02 ? '1' : '0'), ((byte) & 0x01 ? '1' : '0')

#define ExecuteCheck_ADV(expr, expected_result, on_false) \
    {                                                     \
        typeof(expected_result) ret = (expr);             \
        if ( ret != (expected_result) )                   \
        {                                                 \
            on_false                                      \
        }                                                 \
    }

#define JOIN_EXPR(a, b, c) a##_##b##_##c
// regex ->(JOIN_EXPR\((.*), (.*), (.*)\).*,).*
// replace -> $1 // $2_$3_$4

// from exisiting enum use following (change "xx")
// #define xx_ENUM_ITEM(CLASS, TYPE) JOIN_EXPR(xx, CLASS, TYPE)
// regex -> ^(\s*)(\S*)(.*),
// replace -> $1xx_ENUM_ITEM(CLASS, $2)$3,

#include <stdint.h>
#include <stdbool.h>
#define EXEC_RETRY(MAX_RETRY, ON_INIT, ON_LOOP, ON_SUCCESS, ON_FALSE) \
    {                                                                 \
        bool loop_exec()                                              \
        {                                                             \
            (ON_LOOP);                                                \
        }                                                             \
        bool loop_result = false;                                     \
        (ON_INIT);                                                    \
        for ( uint32_t retry = 0; retry < MAX_RETRY; retry++ )        \
        {                                                             \
            loop_result = loop_exec();                                \
            if ( loop_result )                                        \
                break;                                                \
        }                                                             \
        if ( loop_result )                                            \
            (ON_SUCCESS);                                             \
        else                                                          \
            (ON_FALSE);                                               \
    }

#if DEBUG
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define PRINT_CURRENT_LOCATION()                                    \
    {                                                               \
        NRF_LOG_INFO("%s:%d:%s", __FILE__, __LINE__, __FUNCTION__); \
        NRF_LOG_FLUSH();                                            \
    }
#else
#define PRINT_CURRENT_LOCATION()
#endif

#endif //_UTIL_MACROS_H_