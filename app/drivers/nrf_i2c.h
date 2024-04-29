#ifndef _NRF_I2C_
#define _NRF_I2C_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "i2c_common.h"

// defines
#define TWI_INSTANCE_ID 1
#define TWI_SDA_M       14
#define TWI_SCL_M       15

I2C_t* nrf_i2c_get_instance(void);

#endif //_NRF_I2C_