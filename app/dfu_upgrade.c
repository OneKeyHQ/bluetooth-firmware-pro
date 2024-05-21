#include "dfu_upgrade.h"

#include <stdbool.h>
#include <stdint.h>

#include "util_micros.h"

// #include "nrf_uicr.h"
// #include "nrf_flash.h"

#include "dfu_upgrade_payload.h"

#include "nrf_log.h"
#include "nrfx_nvmc.h"

// #include "sdk_common.h"
// #include "nrf.h"
// #include "nrf_mbr.h"
// #include "nrf_sdm.h"
// #include "nrf_dfu_types.h"
// #include "nrf_dfu_req_handler.h"
// #include "app_util_platform.h"

#define EC_E_BOOL_R_BOOL(expr)   ExecuteCheck_ADV(expr, true, { return false; })
#define EC_E_BOOL_CONTINUE(expr) ExecuteCheck_ADV(expr, true, { continue; })

void try_dfu_upgrade(bool force)
{
    if (                                               //
        NRF_UICR->NRFFW[0] != DFU_ADDR ||              // uicr location
        *(uint32_t*)MBR_PARAM_PAGE_ADDR != DFU_ADDR || // mbr location
        force                                          // do it anyways
    )
    {
        uint8_t retry_max = 10;
        uint8_t retry = true;
        do
        {
            retry++;

            sd_mbr_command_t sd_mbr_cmd;

            // ===========================================================
            // *** UICR DFU ADDRESS PATCH
            CRITICAL_REGION_ENTER();
            // uicr backup
            NRF_UICR_Type uicr_backup = {0};
            uint32_t uicr_address = NRF_UICR_BASE;
            for ( uint32_t i = 0; i < sizeof(uicr_backup); i++ )
            {
                if ( i == 64 )
                {
                    i += 64;
                    continue;
                    // skip the RESERVED1[64] area;
                }
                ((uint32_t*)&uicr_backup)[i] = *(uint32_t*)uicr_address;
                while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {}
                uicr_address += 0x04;
            }
            // modify dfu address
            uicr_backup.NRFFW[0] = DFU_ADDR;
            // uicr erase
            nrfx_nvmc_uicr_erase();
            // uicr write
            nrfx_nvmc_words_write(NRF_UICR_BASE, (uint32_t*)&uicr_backup, sizeof(uicr_backup) / 4);
            CRITICAL_REGION_EXIT();

            // ===========================================================
            // *** MBR DFU ADDRESS PATCH
            CRITICAL_REGION_ENTER();

            // mbr backup
            uint32_t mbr_backup[1024] = {0};
            uint32_t flash_address = 0x0;
            for ( uint32_t i = 0; i < sizeof(mbr_backup); i++ )
            {
                if ( i == 64 )
                {
                    i += 64;
                    continue;
                    // skip the RESERVED1[64] area;
                }
                ((uint32_t*)&mbr_backup)[i] = *(uint32_t*)flash_address;
                while ( NRF_NVMC->READY == NVMC_READY_READY_Busy ) {}
                flash_address += 0x04;
            }
            // mbr dfu address
            mbr_backup[MBR_PARAM_PAGE_ADDR / 4] = DFU_ADDR;
            // mbr erase
            nrfx_nvmc_page_erase(0x0); // erase page 0 // TODO: check nrfx_err_t result
            // mbr write
            nrfx_nvmc_words_write(0x00, (uint32_t*)&mbr_backup, sizeof(mbr_backup) / 4);
            CRITICAL_REGION_EXIT();

            // ===========================================================
            // *** DFU UPDATE

            sd_mbr_cmd.command = SD_MBR_COMMAND_COPY_BL;
            sd_mbr_cmd.params.copy_bl.bl_src = (uint32_t*)&payload;
            sd_mbr_cmd.params.copy_bl.bl_len = payload_size;

            // According to document (for SD_MBR_COMMAND_COPY_BL):
            // This command requires that MBR_PARAM_PAGE_ADDR or MBR_UICR_PARAM_PAGE_ADDR is set, see sd_mbr_command.
            // This command will use the flash protect peripheral (BPROT or ACL) to protect the flash that is not
            // intended to be written.
            uint32_t error_if_any = sd_mbr_command(&sd_mbr_cmd);

            NRF_LOG_ERROR("sd_mbr_command FAIL, code -> %lu", error_if_any)
        }
        while ( retry <= retry_max );
    }
}