#include "flashled_manage.h"
#include "nrf_log.h"

//开启灯及设置亮度值
ret_code_t set_led_brightness(uint8_t brightness) {
  uint8_t brightness_regist_value = 0;
  ret_code_t ret;
  //关闭闪光灯
  if (brightness == 0) {
    ret = lm36011_write(LM36011_LED_STATUS, LED_FLASHLIGHT_OFF);
    return ret;
  }
  // 判断是否超过最大设置比例
  if (brightness >= LM36011_LED_MAX_PRECENT) {
    brightness = LM36011_LED_MAX_PRECENT;
  }
  // 计算对应亮度比例对应的值
  brightness_regist_value = ((uint32_t)brightness * LM36011_LED_MAX_SSC) / LM36011_LED_MAX_PRECENT;
  // 判断是否超过最大允许电流，如果是，设置为最大电流（防止后期亮度比例允许超过100）
  if (brightness_regist_value >= LM36011_LED_MAX_SSC) {
    brightness_regist_value = LM36011_LED_MAX_SSC;
  }
  //设置闪光灯亮度值
  ret = lm36011_write(LM36011_LED_BRIGHTNESS, brightness_regist_value);
  if (ret != LED_CONTROL_SUCCESS) {
    return ret;
  }
  nrf_delay_ms(100);
  //开启闪光灯手电筒模式
  ret = lm36011_write(LM36011_LED_STATUS, LED_FLASHLIGHT_MODE);
  // NRF_LOG_INFO("setting status = %d", ret);
  return ret;
}

//获取灯亮度值
uint8_t get_led_brightness(void) {
  uint8_t brightness = 0;
  ret_code_t ret;
  ret = lm36011_read(LM36011_LED_STATUS, 1, &brightness);

  if (ret != LED_CONTROL_SUCCESS) {
    // 读取失败
    return ret;
  }
  // 如果LED关闭，亮度为0，无需进一步读取
  if (brightness == LED_FLASHLIGHT_OFF) {
    // 灯未开，直接返回0
    return brightness;
  }

  ret = lm36011_read(LM36011_LED_BRIGHTNESS, 1, &brightness);
  if (ret != LED_CONTROL_SUCCESS) {
    // 读取失败
    return ret;
  }
  return brightness;
}

//重置灯的状态
ret_code_t reset_led_statu(void) {
  ret_code_t ret = lm36011_write(LM36011_LED_RESET, 0x80);
  return ret;
}
