#include "dfu_upgrade.h"

#include <stdbool.h>
#include <stdint.h>

#include "util_macros.h"

#include "dfu_upgrade_payload.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_nvmc.h"
#include "nrf_mbr.h"
#include "app_util_platform.h"

#define EC_E_BOOL_R_BOOL(expr)   ExecuteCheck_ADV(expr, true, { return false; })
#define EC_E_BOOL_CONTINUE(expr) ExecuteCheck_ADV(expr, true, { continue; })

static bool sd_compare_bl()
{
    NRF_LOG_INFO("%s, line -> %d", __func__, __LINE__);
    NRF_LOG_FLUSH();

    sd_mbr_command_t sd_mbr_cmd;
    sd_mbr_cmd.command = SD_MBR_COMMAND_COMPARE;
    sd_mbr_cmd.params.compare.ptr1 = (uint32_t*)DFU_ADDR;
    sd_mbr_cmd.params.compare.ptr2 = (uint32_t*)&payload;
    sd_mbr_cmd.params.compare.len = payload_size / 4;

    return (NRF_SUCCESS == sd_mbr_command(&sd_mbr_cmd));
}

static uint32_t sd_copy_bl()
{
    NRF_LOG_INFO("%s, line -> %d", __func__, __LINE__);
    NRF_LOG_FLUSH();

    // According to document (for SD_MBR_COMMAND_COPY_BL):
    // xxx_BOOTLOADER_ADDR and xxx_PARAM_PAGE_ADDR must already set
    // if both mbr and uicr address set, mbr value will be used
    // command will protect all flash area other than bootloader area
    // if succeed, will auto reboot

    sd_mbr_command_t sd_mbr_cmd;
    sd_mbr_cmd.command = SD_MBR_COMMAND_COPY_BL;
    sd_mbr_cmd.params.copy_bl.bl_src = (uint32_t*)&payload;
    sd_mbr_cmd.params.copy_bl.bl_len = payload_size / 4;

    return sd_mbr_command(&sd_mbr_cmd);
}

bool try_dfu_upgrade(bool force)
{
    NRF_LOG_INFO("try_dfu_upgrade, line -> %d", __LINE__);
    NRF_LOG_INFO("dfu payload at 0x%x", (uint32_t)&payload);
    NRF_LOG_INFO("dfu payload size %lu (0x%x)", payload_size, payload_size);
    NRF_LOG_FLUSH();

    if ( //
         // NRF_UICR->NRFFW[0] != DFU_ADDR ||          // uicr location // uicr not used
        *(uint32_t*)MBR_BOOTLOADER_ADDR != DFU_ADDR || // mbr location
        !sd_compare_bl() ||                            // dfu binary missmatch
        force                                          // do it anyways
    )
    {
        NRF_LOG_INFO("%s, line -> %d", __func__, __LINE__);
        uint8_t retry_max = 1;
        uint8_t retry = true;
        uint8_t backup_buf[4096];
        do
        {
            retry++;

            // ===========================================================
            // *** UICR DFU ADDRESS PATCH
            CRITICAL_REGION_ENTER();
            // uicr backup
            uint32_t uicr_address = NRF_UICR_BASE;
            for ( uint32_t i = 0; i < sizeof(NRF_UICR_Type); i++ )
            {
                if ( i == 64 )
                {
                    i += 64;
                    continue;
                    // skip the RESERVED1[64] area;
                }
                ((uint32_t*)&backup_buf)[i] = *(uint32_t*)uicr_address;
                while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {}
                uicr_address += 0x04;
            }
            // modify dfu address
            ((NRF_UICR_Type*)&backup_buf)->NRFFW[0] = 0xffffffff; // do not set addr in uicr, use mbr instead
            // uicr erase
            if ( nrfx_nvmc_uicr_erase() != NRF_SUCCESS ) // erase uicr
                continue;                                // retry
            // uicr write
            nrfx_nvmc_words_write(NRF_UICR_BASE, (uint32_t*)&backup_buf, sizeof(NRF_UICR_Type) / 4);
            CRITICAL_REGION_EXIT();

            // ===========================================================
            // *** MBR DFU ADDRESS PATCH
            CRITICAL_REGION_ENTER();

            // mbr backup
            uint32_t flash_address = 0x0;
            for ( uint32_t i = 0; i < sizeof(backup_buf); i += 4 )
            {
                // NRF_LOG_INFO(
                //     "backup mbr, i=%lu (0x%08x), flash_address=0x%08x, val=0x%08x", i, i, (flash_address + i),
                //     *((uint32_t*)(flash_address + i))
                // );
                // NRF_LOG_FLUSH();

                *(uint32_t*)(&backup_buf[i]) = *((uint32_t*)(flash_address + i));
                while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {}
            }
            // mbr dfu address
            *(uint32_t*)(&backup_buf[MBR_BOOTLOADER_ADDR]) = DFU_ADDR;
            // mbr erase
            if ( nrfx_nvmc_page_erase(0x0) != NRF_SUCCESS ) // erase page 0
                continue;                                   // retry
            // mbr write
            nrfx_nvmc_words_write(0x00, (uint32_t*)(backup_buf), sizeof(backup_buf) / 4);
            CRITICAL_REGION_EXIT();

            // ===========================================================
            // *** DFU UPDATE

            uint32_t error_if_any = sd_copy_bl();

            NRF_LOG_ERROR("sd_copy_bl FAIL, code -> %lu", error_if_any)
            NRF_LOG_FLUSH();
        }
        while ( retry <= retry_max );
        return false;
    }
    else
    {
        return true;
    }
}
