#ifndef _POWER_MANAGE_H_
#define _POWER_MANAGE_H_

#include "custom_board.h"
#include "pmu.h"

// defines
#define TWI_INSTANCE_ID 1
// #define TWI_SDA_M 14
// #define TWI_SCL_M 15
#define POWER_IC_IRQ_IO  6
#define POWER_IC_OK_IO   7
#define POWER_IC_CHAG_IO 8

// pmu handle
extern PMU_t* pmu;

bool power_manage_init();
bool power_manage_deinit();

#endif //_POWER_MANAGE_H_