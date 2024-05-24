#ifndef _DFU_UPGRADE_H_
#define _DFU_UPGRADE_H_

#include <stdbool.h>

#define DFU_ADDR         0x6E000
#define DFU_SETTING_ADDR 0x7F000
#define MBR_PARAM_ADDR   0x7E000

bool try_dfu_upgrade(bool force);

#endif // _DFU_UPGRADE_H_
