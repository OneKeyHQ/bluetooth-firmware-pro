#ifndef __RTC_CALENDAR_H_
#define __RTC_CALENDAR_H_
#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_rtc.h"
#include "time.h"

#define RTC_FREQUENCY 8

typedef struct
{
    int RTC_Year;
    int RTC_Month;
    int RTC_Day;
    int RTC_Hours;
    int RTC_Minutes;
    int RTC_Seconds;
    int RTC_Weekday;
} rtc_date_t;

extern volatile uint32_t g_timestamp;

extern void usr_rtc_init(void);
extern void usr_rtc_tick_disable(void);
extern void rtc_set_time(uint32_t timestampNow);
extern uint32_t rtc_get_time(void);

#endif
