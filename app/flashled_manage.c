#include "flashled_manage.h"

ret_code_t set_led_brightness(uint8_t brightness)
{
    uint8_t brightness_regist_value = 0;
    ret_code_t ret;

    if ( brightness == 0 )
    {
        ret = lm36011_write(LM36011_LED_STATUS, LED_FLASHLIGHT_OFF);
        return ret;
    }

    if ( brightness >= LM36011_LED_MAX_PRECENT )
    {
        brightness = LM36011_LED_MAX_PRECENT;
    }
    // calculate the value corresponding to the corresponding brightness ratio
    brightness_regist_value = ((uint32_t)brightness * LM36011_LED_MAX_SSC) / LM36011_LED_MAX_PRECENT;

    if ( brightness_regist_value >= LM36011_LED_MAX_SSC )
    {
        brightness_regist_value = LM36011_LED_MAX_SSC;
    }
    // set brightness value
    ret = lm36011_write(LM36011_LED_BRIGHTNESS, brightness_regist_value);
    if ( ret != LED_CONTROL_SUCCESS )
    {
        return ret;
    }
    nrf_delay_ms(100);

    ret = lm36011_write(LM36011_LED_STATUS, LED_FLASHLIGHT_MODE);
    return ret;
}
uint8_t get_led_brightness(void)
{
    uint8_t brightness = 0;
    ret_code_t ret;
    ret = lm36011_read(LM36011_LED_STATUS, 1, &brightness);

    if ( ret != LED_CONTROL_SUCCESS )
    {
        return ret;
    }

    if ( brightness == LED_FLASHLIGHT_OFF )
    {
        return brightness;
    }

    ret = lm36011_read(LM36011_LED_BRIGHTNESS, 1, &brightness);
    if ( ret != LED_CONTROL_SUCCESS )
    {

        return ret;
    }
    return brightness;
}

ret_code_t reset_led_statu(void)
{
    ret_code_t ret = lm36011_write(LM36011_LED_RESET, 0x80);
    return ret;
}
