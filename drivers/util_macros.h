#ifndef _util_macros_
#define _util_macros_

#define UNUSED_VAR(X) ((void)(X))

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

#endif //_util_macros_