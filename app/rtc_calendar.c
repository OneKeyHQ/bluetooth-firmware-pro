#include "rtc_calendar.h"
#include "nrf_log.h"

static void rtc_evt_handler(nrf_drv_rtc_int_type_t interruptType);
struct tm m_time;
rtc_date_t rtc_date;

volatile uint32_t m_timestamp = 1642758421;

static const nrf_drv_rtc_t s_rtc_handle = NRF_DRV_RTC_INSTANCE(2); // Declaring an instance of nrf_drv_rtc for rtc2.
static uint8_t s_timeCount1second = 0;

static void utc_to_calendar(uint32_t timestamp)
{
    struct tm* tm_p;

    tm_p = localtime((time_t*)&timestamp);
    rtc_date.RTC_Year = tm_p->tm_year - 100;
    rtc_date.RTC_Month = tm_p->tm_mon + 1;
    rtc_date.RTC_Day = tm_p->tm_mday;
    rtc_date.RTC_Weekday = tm_p->tm_wday;
    rtc_date.RTC_Hours = tm_p->tm_hour + 8;
    rtc_date.RTC_Minutes = tm_p->tm_min;
    rtc_date.RTC_Seconds = tm_p->tm_sec;

    // NRF_LOG_INFO("year-%d/month-%d/day-%d,weeday %d",rtc_date.RTC_Year,
    // rtc_date.RTC_Month,rtc_date.RTC_Day,rtc_date.RTC_Weekday); NRF_LOG_INFO("%d:%d:%d",
    // rtc_date.RTC_Hours,rtc_date.RTC_Minutes, rtc_date.RTC_Seconds);
}

static void rtc_evt_handler(nrf_drv_rtc_int_type_t interruptType)
{
    if ( interruptType == NRF_DRV_RTC_INT_COMPARE0 ) {}
    else if ( interruptType == NRF_DRV_RTC_INT_TICK )
    {
        // 125ms * 8 = 1s
        if ( s_timeCount1second >= (RTC_FREQUENCY - 1) )
        {
            s_timeCount1second = 0;
            m_timestamp++;
            utc_to_calendar(m_timestamp);
        }
        else
        {
            s_timeCount1second++;
        }
    }
}

void usr_rtc_init(void)
{
    ret_code_t errCode;

    // Initialize rtc instance
    nrf_drv_rtc_config_t RTCConfig = NRF_DRV_RTC_DEFAULT_CONFIG;
    // set 8Hz -> 32768/8-1 = 4095
    RTCConfig.prescaler = 4095;

    errCode = nrf_drv_rtc_init(&s_rtc_handle, &RTCConfig, rtc_evt_handler);
    APP_ERROR_CHECK(errCode);

    // Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&s_rtc_handle, true);
    // Power on rtc instance
    nrf_drv_rtc_enable(&s_rtc_handle);
}

void usr_rtc_tick_disable(void)
{
    nrf_drv_rtc_tick_disable(&s_rtc_handle);
}

void rtc_set_time(uint32_t timestampNow)
{
    m_timestamp = timestampNow;
}

uint32_t rtc_get_time(void)
{
    return m_timestamp;
}
