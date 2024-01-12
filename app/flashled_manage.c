#include "flashled_manage.h"
#define LED_BRIGHTNESS_VALUE 0x15  // LED brightness value


//开启灯及设置亮度值
ret_code_t set_led_brightness(uint8_t brightness)
{      
    ret_code_t ret = 1;        
        if (brightness == 0)
        {
          ret = lm36011_write(LM36011_LED_STATUS, 0);   //关闭闪光灯    
        } else {
          ret = lm36011_write(LM36011_LED_STATUS, 2);   //开启闪光灯手电筒模式
          nrf_delay_ms(100);   
          ret  = lm36011_write(LM36011_LED_BRIGHTNESS, LED_BRIGHTNESS_VALUE);
        }
    return ret;
}


//获取灯亮度值
uint8_t get_led_brightness(void)
{
        uint8_t brightness = 0;
        lm36011_read(LM36011_LED_STATUS,1, &brightness);
        if(brightness == 0)
        {
           return  brightness;   //灯未开，直接返回0
        }
        lm36011_read(LM36011_LED_BRIGHTNESS, 1,&brightness);  
     return   brightness ;
}


//重置灯的状态
ret_code_t reset_led_statu(void)
{              
     ret_code_t ret = lm36011_write(LM36011_LED_RESET, 0x80);              
     return   ret ;
}

