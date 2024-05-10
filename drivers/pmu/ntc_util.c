#include "ntc_util.h"

#include <stdint.h>
#include <math.h>

NTC_Char(NCP15XH103F03RC_2585, 298.15, 10 * 1000, 3434);

inline float ntc_temp_cal_cv(NTC_Char_t ntc_char, uint32_t current_ua, uint32_t voltage_uv)
{
    float r_ohm = voltage_uv / current_ua;

    return ntc_temp_cal_r(ntc_char, r_ohm);
}

inline float ntc_temp_cal_r(NTC_Char_t ntc_char, float r_ohm)
{
    return (1 / ((log10f(r_ohm / ntc_char.rStd_ohm) / log10f(M_E) / ntc_char.b) + (1 / ntc_char.tStd_k)) - 273.15);
}