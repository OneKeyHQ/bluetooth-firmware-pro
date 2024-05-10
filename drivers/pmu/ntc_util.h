#ifndef _NTC_UTIL_
#define _NTC_UTIL_

#include <stdint.h>

typedef struct
{
    float tStd_k;
    float rStd_ohm;
    float b;
} NTC_Char_t;

#define NTC_Char(name, _tStd_k, _rStd_ohm, _b) \
    NTC_Char_t NTC_Char_##name = {.tStd_k = _tStd_k, .rStd_ohm = _rStd_ohm, .b = _b}

#define EXT_NTC_Char(name) extern NTC_Char_t NTC_Char_##name

EXT_NTC_Char(NCP15XH103F03RC_2585); // NTC_Char_NCP15XH103F03RC_2585

float ntc_temp_cal_cv(NTC_Char_t ntc_char, uint32_t current_ua, uint32_t voltage_uv);
float ntc_temp_cal_r(NTC_Char_t ntc_char, float r_ohm);

#endif //_NTC_UTIL_