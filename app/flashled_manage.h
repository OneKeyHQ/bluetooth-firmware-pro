#ifndef __FLSHLED_MANAGE_H_
#define __FLSHLED_MANAGE_H_

#include "lm_config.h"

extern ret_code_t set_led_brightness(uint8_t brightness);
extern uint8_t get_led_brightness(void);
extern ret_code_t reset_led_statu(void);

#endif