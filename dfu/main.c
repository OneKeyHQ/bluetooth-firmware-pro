/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_info.h"
#include "nrf_mbr.h"
#include "nrf_gpio.h"
#include "nrf_sdh.h"
#include "nrf_power.h"

#include "util_macros.h"
#include "axp216_config.h"

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t* p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}

void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer1(nrf_dfu_evt_type_t evt_type)
{
    switch ( evt_type )
    {
    case NRF_DFU_EVT_DFU_FAILED:
    case NRF_DFU_EVT_DFU_ABORTED:
    case NRF_DFU_EVT_DFU_INITIALIZED:
    case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
    case NRF_DFU_EVT_DFU_STARTED:
        break;
    default:
        break;
    }
}

// static char spinner()
// {
//     static uint8_t count = 0;
//     count++;
//     char spin_symb[] = {'-', '/', '|', '\\'};
//     return spin_symb[count % 4];
// }

/**@brief Function for application main entry. */
#define PMIC_IRQ_IO   6
#define PMIC_PWROK_IO 7
static void enter_low_power_mode(void)
{
    PRINT_CURRENT_LOCATION();
    NRF_LOG_FINAL_FLUSH();

    // enable wakeup
    nrf_gpio_cfg_sense_input(PMIC_PWROK_IO, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

    // enter sysoff
#ifdef SOFTDEVICE_PRESENT
    if ( nrf_sdh_is_enabled() )
    {
        ret_code_t ret_code = sd_power_system_off();
        ASSERT((ret_code == NRF_SUCCESS) || (ret_code == NRF_ERROR_SOFTDEVICE_NOT_ENABLED));
        UNUSED_VARIABLE(ret_code);
  #ifdef DEBUG
        while ( true )
        {
            __WFE();
        }
  #endif
    }
#endif // SOFTDEVICE_PRESENT
    nrf_power_system_off();
}
int main(void)
{
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("DFU Enter");
    NRF_LOG_FLUSH();

    uint32_t u32Reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear reset reason by writting 1.
    NRF_LOG_INFO("Reset Status -> %x", u32Reset_reason);
    NRF_LOG_FLUSH();

    // // try config axp216
    // EXEC_RETRY(
    //     3,
    //     {
    //         NRF_LOG_INFO("AXP216 Config");
    //         NRF_LOG_FLUSH();
    //         nrf_gpio_cfg_default(PMIC_IRQ_IO);
    //         nrf_gpio_cfg_default(PMIC_PWROK_IO);
    //     },
    //     {
    //         switch ( axp216_minimum_config() )
    //         {
    //         case AXP216_CONF_BUS_ERR:
    //             NRF_LOG_INFO("AXP216_CONF_BUS_ERR");
    //             NRF_LOG_FLUSH();
    //             return false;
    //             break;
    //         case AXP216_CONF_NO_ACK:
    //             NRF_LOG_INFO("AXP216_CONF_NO_ACK");
    //             NRF_LOG_FLUSH();
    //             return false;
    //             break;
    //         case AXP216_CONF_NOT_NEEDED:
    //             NRF_LOG_INFO("AXP216_CONF_NOT_NEEDED");
    //             NRF_LOG_FLUSH();
    //             return true;
    //             break;
    //         case AXP216_CONF_SUCCESS:
    //             NRF_LOG_INFO("AXP216_CONF_SUCCESS");
    //             NRF_LOG_FLUSH();
    //             return true;
    //             break;
    //         case AXP216_CONF_FAILED:
    //             NRF_LOG_INFO("AXP216_CONF_FAILED");
    //             NRF_LOG_FLUSH();
    //             return false;
    //         case AXP216_CONF_INVALID:
    //         default:
    //             NRF_LOG_INFO("AXP216_CONF_INVALID");
    //             NRF_LOG_FLUSH();
    //             return false;
    //         }
    //     },
    //     {
    //         // do nothing on success
    //     },
    //     {
    //         // sleep on fail
    //         enter_low_power_mode();
    //     }
    // );

    // nrf_bootloader_mbr_addrs_populate(); // we dont use uicr address anymore

    // Protect MBR and bootloader flash area
    APP_ERROR_CHECK(nrf_bootloader_flash_protect(0, MBR_SIZE, false));
    APP_ERROR_CHECK(nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false));

    // app_read_protect(); // NRF_BL_DEBUG_PORT_DISABLE = 1 already did it

    // flush log befor enter app
    NRF_LOG_FINAL_FLUSH();

    APP_ERROR_CHECK(nrf_bootloader_init(dfu_observer1));
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK_BOOL(false);
}

/**
 * @}
 */
