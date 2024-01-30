#ifndef __LINUX_LM_CFG_H_
#define __LINUX_LM_CFG_H_

#include <stdint.h>  // 对于 uint8_t
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"

#define LM36011_TWI_SDA_M    14
#define LM36011_TWI_SCL_M    15
#define LM36011_DEVICES_ADDR (0x64)
#define LM36011_ADDRESS_LEN  1

#define LM36011_LED_STATUS      (0x01)  // control light switch
#define LM36011_LED_BRIGHTNESS  (0x04)  // control light brightness
#define LM36011_LED_RESET       (0x06)  // control light reset
#define LM36011_LED_MAX_SSC     (0x20)  // max ssc  79ma
#define LM36011_LED_MAX_PRECENT 100
#define LED_FLASHLIGHT_MODE     2
#define LED_FLASHLIGHT_OFF      0
#define LED_BRIGHTNESS_VALUE    0x15  // LED brightness value
#define LED_CONTROL_SUCCESS     0

#define TWI_INSTANCE_ID 1

ret_code_t lm36011_twi_master_init(void);

ret_code_t lm36011_write(const uint8_t writeAddr, const uint8_t writeData);

ret_code_t lm36011_read(uint8_t readAddr, uint8_t byteNum, uint8_t *readData);

#endif
