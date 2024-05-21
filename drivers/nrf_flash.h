#ifndef _NRF_FLASH_
#define _NRF_FLASH_

#include <stdint.h>
#include <stdbool.h>

bool flash_init(void);
bool flash_deinit(void);
void flash_wait_busy(void);
bool flash_check_blank(uint32_t addr, uint32_t len);
bool flash_erase(uint32_t address, uint32_t len);
bool flash_read(uint32_t address, void* data, uint32_t len);
bool flash_write(uint32_t address, void* data, uint32_t len);

#endif //_NRF_FLASH_