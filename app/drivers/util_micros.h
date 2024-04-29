#ifndef _UTIL_MICROS_
#define _UTIL_MICROS_

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
// regex ->(JOIN_EXPR\((.*), (.*), (.*)\).*,)
// replace -> $1 // $2_$3_$4

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define PRINT_CURRENT_LOCATION()                                    \
    {                                                               \
        NRF_LOG_INFO("%s:%d:%s", __FILE__, __LINE__, __FUNCTION__); \
        NRF_LOG_FLUSH();                                            \
    }

#endif //_UTIL_MICROS_