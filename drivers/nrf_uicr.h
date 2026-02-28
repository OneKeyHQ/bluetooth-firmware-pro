#ifndef _NRF_UICR_
#define _NRF_UICR_

#include "stdint.h"
#include "stdbool.h"

#include "nrf_nvmc.h"

#define UICR_SIZE  (sizeof(NRF_UICR_Type))
#define UICR_START (NRF_UICR_BASE)
#define UICR_END   (NRF_UICR_BASE + UICR_SIZE)

bool uicr_check_blank(uint32_t addr, uint32_t len_word);
bool uicr_write(uint32_t addr, void* data, uint32_t len_word);
bool uicr_read(uint32_t addr, void* data, uint32_t len_word);
bool uicr_update_bootloader_addr(uint32_t bootloader_addr);
bool uicr_update_customer(uint8_t offset, void* data, uint8_t len);
bool uicr_get_customer(uint8_t offset, void* data, uint8_t len);

#endif //_NRF_UICR_