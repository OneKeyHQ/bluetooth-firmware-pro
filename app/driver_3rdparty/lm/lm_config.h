#ifndef __LINUX_LM_CFG_H_
#define __LINUX_LM_CFG_H_


#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include <stdint.h>   // 对于 uint8_t


#define LM36011_TWI_SDA_M		14   
#define LM36011_TWI_SCL_M		15   
#define	LM36011_DEVICES_ADDR	    (0x64)
#define LM36011_ADDRESS_LEN   	1

#define LM36011_LED_STATUS          (0x01)   //control light switch
#define LM36011_LED_BRIGHTNESS      (0x04)   //control light brightness
#define LM36011_LED_RESET           (0x06)   //control light reset

#define TWI_INSTANCE_ID    		1

ret_code_t lm36011_twi_master_init(void);

ret_code_t lm36011_write(const uint8_t writeAddr, const uint8_t writeData);

ret_code_t lm36011_read( uint8_t readAddr, uint8_t byteNum , uint8_t *readData);

#endif

