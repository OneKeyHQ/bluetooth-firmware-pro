#ifndef _LM36011_H_
#define _LM36011_H_

#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"

// ================================
// defines
#define LM36011_DEVICES_ADDR    (0x64)

#define LM36011_LED_STATUS      (0x01) // control light switch
#define LM36011_LED_BRIGHTNESS  (0x04) // control light brightness
#define LM36011_LED_RESET       (0x06) // control light reset
#define LM36011_LED_MAX_SSC     (0x20) // max ssc  79ma
#define LM36011_LED_MAX_PRECENT 100
#define LED_FLASHLIGHT_MODE     2
#define LED_FLASHLIGHT_OFF      0
#define LED_BRIGHTNESS_VALUE    0x15 // LED brightness value

ret_code_t lm36011_write(const uint8_t writeAddr, const uint8_t writeData);
ret_code_t lm36011_read(uint8_t readAddr, uint8_t byteNum, uint8_t* readData);

#endif // _LM36011_H_