/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_fido.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sdk_macros.h"
#include "app_scheduler.h"
// #include "ble_dfu.h"
#include "nrf_bootloader_info.h"
#include "nrf_crypto.h"
#include "nrf_crypto_init.h"
#include "nrf_delay.h"
#include "nrf_drv_wdt.h"
#include "nrf_power.h"
#include "nrf_uarte.h"
#include "nrfx_gpiote.h"

#include "util_macros.h"
#include "ecdsa.h"
#include "power_manage.h"
#include "flashled_manage.h"
#include "data_transmission.h"
#include "device_config.h"
#include "firmware_config.h"
#include "dfu_upgrade.h"

#define RX_PIN_NUMBER           11
#define TX_PIN_NUMBER           12
#define CTS_PIN_NUMBER          UART_PIN_DISCONNECTED
#define RTS_PIN_NUMBER          UART_PIN_DISCONNECTED

#define APDU_TAG_BLE            0x44

#define BLE_DEFAULT             0
#define BLE_CONNECT             1
#define BLE_DISCONNECT          2
#define BLE_DIS_PIN             3
#define BLE_PIN_ERR             4
#define BLE_PIN_TIMEOUT         5
#define BLE_PIN_CANCEL          7
#define BLE_RCV_DATA            8

#define DEFAULT_FLAG            0
#define SEND_SPI_DATA           1
#define READ_SPI_HEAD           2
#define READ_SPI_DATA           3

#define BLE_DEF                 0
#define BLE_ON_ALWAYS           1
#define BLE_OFF_ALWAYS          2
#define BLE_DISCON              3
#define BLE_CON                 4
#define BLE_PAIR                5

#define PWR_DEF                 0
#define PWR_SHUTDOWN_SYS        1
#define PWR_CLOSE_EMMC          2
#define PWR_OPEN_EMMC           3
#define PWR_BAT_PERCENT         5
#define PWR_USB_STATUS          6

#define LED_DEF                 0
#define LED_SET_BRIHTNESS       1
#define LED_GET_BRIHTNESS       2

#define BAT_DEF                 0
#define SEND_BAT_VOL            1
#define SEND_BAT_CHARGE_CUR     2
#define SEND_BAT_DISCHARGE_CUR  3
#define SEND_BAT_INNER_TEMP     4

#define INIT_VALUE              0
#define AUTH_VALUE              1

#define APP_BLE_OBSERVER_PRIO   3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG    1 /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL        40 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION        0 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define ADV_ADDL_MANUF_DATA_LEN 6
#define COMPANY_IDENTIFIER      0xFE

// SCHEDULER CONFIGS
#define SCHED_MAX_EVENT_DATA_SIZE   256 //!< Maximum size of the scheduler event data.
#define SCHED_QUEUE_SIZE            4   //!< Size of the scheduler queue.

#define RCV_DATA_TIMEOUT_INTERVAL   APP_TIMER_TICKS(1000)
#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(1000) /**< Battery level measurement interval (ticks). */
#define BATTERY_MEAS_LONG_INTERVAL  APP_TIMER_TICKS(5000)

#define MIN_CONN_INTERVAL           MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL           MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY               0                               /**< Slave latency. */
#define CONN_SUP_TIMEOUT            MSEC_TO_UNITS(300, UNIT_10_MS) /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                                                                    \
    APP_TIMER_TICKS(100) /**< Time from initiating event (connect or start of notification) to first time \
                            sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY \
    APP_TIMER_TICKS(30000             \
    ) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define ONE_SECOND_INTERVAL      APP_TIMER_TICKS(1000)

#define RST_ONE_SECNOD_COUNTER() one_second_counter = 0;
#define TWI_TIMEOUT_COUNTER      10

#define MAX_CONN_PARAM_UPDATE_COUNT                                                  \
    3 /**< Number of attempts before giving up the connection parameter negotiation. \
       */

#define LESC_DEBUG_MODE           0 /**< Set to 1 to use LESC debug keys, allows you to  use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND            1 /**< Perform bonding. */
#define SEC_PARAM_MITM            1 /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC            1                            /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS        0                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_DISPLAY_ONLY /**< Display I/O capabilities. */
#define SEC_PARAM_OOB             0                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                           /**< Maximum encryption key size. */

#define PASSKEY_LENGTH            6 /**< Length of pass-key received by the stack for display. */
#define HEAD_NAME_LENGTH          4
#define ADV_NAME_LENGTH           8
#define MAC_ADDRESS_LENGTH        6

#define DEAD_BEEF \
    0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 600 //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS \
    90 // 270=0.3v  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series
       // with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of
       // 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT 1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION \
    6 //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be
      //!< multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

// UART define
#define MAX_TEST_DATA_BYTES (15U) /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE    256   /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE    256   /**< UART RX buffer size. */

// BLE send CMD
#define BLE_CMD_ADV_NAME 0x01
//
#define BLE_CMD_CON_STA    0x02
#define BLE_CON_STATUS     0x01
#define BLE_DISCON_STATUS  0x02
#define BLE_ADV_ON_STATUS  0x03
#define BLE_ADV_OFF_STATUS 0x04
//
#define BLE_CMD_PAIR_CODE 0x03
//
#define BLE_CMD_PAIR_STA 0x04
#define BLE_PAIR_SUCCESS 0x01
#define BLE_PAIR_FAIL    0x02
//
#define BLE_FIRMWARE_VER   0x05
#define BLE_SOFTDEVICE_VER 0x06
#define BLE_BOOTLOADER_VER 0x07

//
#define BLE_SYSTEM_POWER_PERCENT 0x09

#define BLE_CMD_FLASH_LED_STA    0x0C
#define BLE_CMD_BAT_CV_MSG       0x0D

#define BLE_CMD_KEY_RESP         0x0E
#define BLE_KEY_RESP_SUCCESS     0x00
#define BLE_KEY_RESP_FAILED      0x01
#define BLE_KEY_RESP_PUBKEY      0x02
#define BLE_KEY_RESP_SIGN        0x03

#define BLE_CMD_BUILD_ID         0x10
#define BLE_CMD_HASH             0x11
#define BLE_CMD_BT_MAC           0x12

// end BLE send CMD
//
#define UART_CMD_BLE_CON_STA  0x01
#define UART_CMD_BLE_PAIR_STA 0x02
#define UART_CMD_PAIR_CODE    0x03
#define UART_CMD_ADV_NAME     0x04
#define UART_CMD_BAT_PERCENT  0x05
#define UART_CMD_BLE_VERSION  0x06
#define UART_CMD_CTL_BLE      0x07

#define UART_CMD_DFU_STA      0x0a

// Receive ST CMD
#define ST_CMD_BLE               0x81
#define ST_SEND_OPEN_BLE         0x01
#define ST_SEND_CLOSE_BLE        0x02
#define ST_SEND_DISCON_BLE       0x03
#define ST_GET_BLE_SWITCH_STATUS 0x04
//
#define ST_CMD_POWER           0x82
#define ST_SEND_CLOSE_SYS_PWR  0x01
#define ST_SEND_CLOSE_EMMC_PWR 0x02
#define ST_SEND_OPEN_EMMC_PWR  0x03
#define ST_REQ_POWER_PERCENT   0x04
#define ST_REQ_USB_STATUS      0x05
#define ST_REQ_ENABLE_CHARGE   0x06
#define ST_REQ_DISABLE_CHARGE  0x07
//
#define ST_CMD_BLE_INFO       0x83
#define ST_REQ_ADV_NAME       0x01
#define ST_REQ_FIRMWARE_VER   0x02
#define ST_REQ_SOFTDEVICE_VER 0x03
#define ST_REQ_BOOTLOADER_VER 0x04
#define ST_REQ_BUILD_ID       0x05
#define ST_REQ_HASH           0x06
#define ST_REQ_BT_MAC         0x07

//
#define ST_CMD_RESET_BLE   0x84
#define ST_VALUE_RESET_BLE 0x01
//
#define ST_CMD_LED                 0X85
#define ST_SEND_SET_LED_BRIGHTNESS 0X01
#define ST_SEND_GET_LED_BRIGHTNESS 0X02
//
#define STM_CMD_BAT                0X86
#define STM_SEND_BAT_VOL           0X01 // Send battery voltage
#define STM_SEND_BAT_CHARGE_CUR    0X02 // Send battery charge current
#define STM_SEND_BAT_DISCHARGE_CUR 0X03 // Send battery discharge current
#define STM_SEND_BAT_INNER_TEMP    0X04 // Send battery inner temperature

#define STM_CMD_KEY                0x87
#define STM_GET_PUBKEY             0x01
#define STM_LOCK_PUBKEY            0x02
#define STM_REQUEST_SIGN           0x03

// end Receive ST CMD

// VALUE
#define VALUE_CONNECT    0x01
#define VALUE_DISCONNECT 0x02
#define VALUE_SECCESS    0x01
#define VALUE_FAILED     0x02

// DFU STATUS
#define VALUE_PREPARE_DFU  0x01
#define VALUE_ENTER_DFU    0x02
#define VALUE_ENTER_FAILED 0x03
#define VALUE_RSP_FAILED   0x04
#define VALUE_UNKNOWN_ERR  0x05

// DATA FLAG
#define DATA_INIT 0x00
#define DATA_HEAD 0x01

// BLE RSP STATUS
#define CTL_SUCCESSS 0x01
#define CTL_FAILED   0x02

// CHANNEL
#define BLE_CHANNEL 0x01
// #define NFC_CHANNEL                     0x02
#define UART_CHANNEL                0x03

#define UART_DEF                    0x00
#define ACTIVE_SEND_UART            0x01
#define RESPONESE_NAME              0x02
#define RESPONESE_BAT               0x03
#define RESPONESE_VER               0x04
#define RESPONESE_SD_VER            0x05
#define RESPONESE_BOOT_VER          0x06
#define RESPONESE_LED_VER           0x07
#define RESPONESE_BLE_PUBKEY        0x08
#define RESPONESE_BLE_PUBKEY_LOCK   0x09
#define RESPONESE_BLE_SIGN          0x0A
#define RESPONESE_BUILD_ID          0x0B
#define RESPONESE_HASH              0x0C
#define RESPONESE_BT_MAC            0x0D
#define DEF_RESP                    0xFF

#define TIMER_INIT_FLAG             0
#define TIMER_RESET_FLAG            1
#define TIMER_START_FLAG            2
#define TIMER_STOP_FLAG             3

#define BLE_GAP_DATA_LENGTH_DEFAULT 27  //!< The stack's default data length.
#define BLE_GAP_DATA_LENGTH_MAX     251 //!< Maximum data length.

#define ST_WAKE_IO                  22

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);
BLE_FIDO_DEF(m_fido, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);  /**< Battery timer. */
APP_TIMER_DEF(data_wait_timer_id);  /**< data wait timeout timer. */
APP_TIMER_DEF(m_1s_timer_id);
nrf_drv_wdt_channel_id m_channel_id;

static volatile uint8_t one_second_counter = 0;
volatile uint8_t ble_adv_switch_flag = BLE_DEF;
static volatile uint8_t ble_conn_flag = BLE_DEF;
static volatile uint8_t ble_conn_nopair_flag = BLE_DEF;
static volatile uint8_t pwr_status_flag = PWR_DEF;
static volatile uint8_t trans_info_flag = UART_DEF;
static volatile uint8_t led_brightness_flag = LED_DEF;
static volatile uint8_t bat_msg_flag = BAT_DEF;
static volatile uint8_t ble_trans_timer_flag = TIMER_INIT_FLAG;
static uint8_t mac_ascii[12];
static uint8_t mac[BLE_GAP_ADDR_LEN] = {0x42, 0x13, 0xc7, 0x98, 0x95, 0x1a}; // Device MAC address
static char ble_adv_name[ADV_NAME_LENGTH];

// void BusFault_Handler(void)
// {
//     NRF_LOG_ERROR("BusFault -> 0x08%x", SCB->BFAR);
//     NRF_LOG_FINAL_FLUSH();
// }

// add tmp
unsigned char cfg;
unsigned char write;

#ifdef BOND_ENABLE
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
#endif
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[] =                        /**< Universally unique service identifiers. */
    {
#if BLE_DIS_ENABLED
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
#endif
        {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_FIDO_SERVICE, BLE_UUID_TYPE_BLE}};

static volatile uint8_t flag_uart_trans = 1;
static uint8_t uart_trans_buff[128];
static uint8_t bak_buff[128];
static uint8_t uart_data_array[64];
static void uart_put_data(uint8_t* pdata, uint8_t lenth);
static void send_stm_data(uint8_t* pdata, uint8_t lenth);
static uint8_t calcXor(uint8_t* buf, uint8_t len);

static bool bt_advertising_ctrl(bool enable, bool commit);
static void idle_state_handle(void);
void start_data_wait_timer(void);

static uint8_t bond_check_key_flag = INIT_VALUE;
static uint8_t rcv_head_flag = 0;
static uint8_t ble_status_flag = 0;

static volatile uint16_t ble_nus_send_len = 0, ble_nus_send_offset = 0;
static uint8_t* ble_nus_send_buf;

// global vars
static uint8_t g_bas_update_flag = 0;

// PMU global status
static volatile bool pmu_status_synced = false;
static volatile bool pmu_feat_synced = false;
static volatile bool pmu_feat_charge_enable = false;

// FLASH LED global status
static volatile bool led_brightness_synced = false;
static volatile uint8_t led_brightness_value = 0;

// misc status flags
// mainly use for workarounds when needs shutdown before app fullly boots to main even loop
// NRF5 SDK is poorly designed which do not track if a feature is initialized when calling apis
// Thus uninit function may stuck if feature init never called
static volatile bool app_uart_is_initialized = false;

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    NRF_LOG_DEBUG("%s , nrf_pwr_mgmt_evt_t = %d", __func__, event);
    NRF_LOG_FLUSH();

    switch ( event )
    {
    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
        // enable wakeup
        nrf_gpio_cfg_sense_input(PMIC_PWROK_IO, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
        // nrf_gpio_cfg_sense_input(PMIC_IRQ_IO, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
        return true;

    case NRF_PWR_MGMT_EVT_PREPARE_RESET:
        return true;

    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
    case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
    default:
        return false;
    }
}
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
} /**< Structure used to identify the battery service. */

static inline void gpio_uninit(void)
{
    nrfx_gpiote_uninit();
}

static void enter_low_power_mode(void)
{
    // stop uart
    if ( app_uart_is_initialized )
    {
        app_uart_close();
        app_uart_is_initialized = false;
    }

    // stop bt adv
    if ( nrf_sdh_is_enabled() )
    {
        while ( !bt_advertising_ctrl(false, false) )
        {
            nrf_delay_ms(100);
        }
    }

    // release pmu interface
    if ( (pmu_p != NULL) && (pmu_p->isInitialized) )
        pmu_p->Deinit();

    // release gpio
    gpio_uninit();
    nrf_gpio_cfg_default(ST_WAKE_IO);

    // shutdown
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}

// sdh_soc_handler, mainly for power warning, no other event processed yet
void sdh_soc_handler(uint32_t sys_evt, void* p_context)
{
    UNUSED_VAR(p_context);

    if ( sys_evt == NRF_EVT_POWER_FAILURE_WARNING )
    {
        // nrf_gpio_cfg_output(PMIC_PWROK_IO);
        // nrf_gpio_pin_clear(PMIC_PWROK_IO);
        NRF_LOG_INFO("NRF Power POF triggered!");
        NRF_LOG_FLUSH();

        pmu_p->SetState(PWR_STATE_HARD_OFF);
        enter_low_power_mode();
    }
}
NRF_SDH_SOC_OBSERVER(m_soc_observer, 0, sdh_soc_handler, NULL);

void battery_level_meas_timeout_handler(void* p_context)
{
    ret_code_t err_code;
    static uint8_t battery_percent = 0;

    UNUSED_PARAMETER(p_context);
    if ( battery_percent != pmu_p->PowerStatus->batteryPercent )
    {
        battery_percent = pmu_p->PowerStatus->batteryPercent;
        if ( g_bas_update_flag == 1 )
        {
            err_code = ble_bas_battery_level_update(&m_bas, battery_percent, BLE_CONN_HANDLE_ALL);
            if ( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) &&
                 (err_code != NRF_ERROR_RESOURCES) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) )
            {
                APP_ERROR_HANDLER(err_code);
            }
        }
    }
}

void data_wait_timeout_hander(void* p_context)
{
    UNUSED_PARAMETER(p_context);

    rcv_head_flag = DATA_INIT;
    spi_state_reset();
}

void m_1s_timeout_hander(void* p_context)
{
    UNUSED_PARAMETER(p_context);

    nrf_drv_wdt_channel_feed(m_channel_id);

    one_second_counter++;

    if ( (one_second_counter % 5) == 0 )
    {
        pmu_status_synced = false;
    }

    if ( one_second_counter > 59 )
    {
        one_second_counter = 0;
    }
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
void on_bas_evt(ble_bas_t* p_bas, ble_bas_evt_t* p_evt)
{
    switch ( p_evt->evt_type )
    {
    case BLE_BAS_EVT_NOTIFICATION_ENABLED:
        g_bas_update_flag = 1;
        break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

    case BLE_BAS_EVT_NOTIFICATION_DISABLED:
        g_bas_update_flag = 0;
        break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

    default:
        // No implementation needed.
        break;
    }
}

// battery service init
void sys_bas_init(void)
{
    uint32_t err_code;
    ble_bas_init_t bas_init;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}
#ifdef BOND_ENABLE
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(const pm_evt_t* p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch ( p_evt->evt_id )
    {

    case PM_EVT_CONN_SEC_CONFIG_REQ:
        NRF_LOG_INFO("%s ---> PM_EVT_CONN_SEC_CONFIG_REQ", __func__);
        {
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }
        break;

    case PM_EVT_CONN_SEC_SUCCEEDED:
        NRF_LOG_INFO("%s ---> PM_EVT_CONN_SEC_SUCCEEDED", __func__);
        {
            pm_conn_sec_status_t conn_sec_status;

            // Check if the link is authenticated (meaning at least MITM).
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);

            if ( conn_sec_status.mitm_protected )
            {
                if ( ble_conn_nopair_flag == BLE_PAIR )
                {
                    ble_conn_nopair_flag = BLE_DEF;
                    bak_buff[0] = BLE_CMD_PAIR_STA;
                    bak_buff[1] = BLE_PAIR_SUCCESS;
                    send_stm_data(bak_buff, 2);
                }

                nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_MAX);
                NRF_LOG_INFO(
                    "Link secured. Role: %d. conn_handle: %d, Procedure: %d", ble_conn_state_role(p_evt->conn_handle),
                    p_evt->conn_handle, p_evt->params.conn_sec_succeeded.procedure
                );
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

    case PM_EVT_CONN_SEC_FAILED:
        NRF_LOG_INFO("%s ---> PM_EVT_CONN_SEC_FAILED", __func__);
        NRF_LOG_INFO(
            "conn_sec_failed: procedure=0x%x, error=0x%x, error_src=0x%x", p_evt->params.conn_sec_failed.procedure,
            p_evt->params.conn_sec_failed.error, p_evt->params.conn_sec_failed.error_src
        );
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        bak_buff[0] = BLE_CMD_PAIR_STA;
        bak_buff[1] = BLE_PAIR_FAIL;
        send_stm_data(bak_buff, 2);
        break;

    case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        NRF_LOG_INFO("%s ---> PM_EVT_LOCAL_DB_CACHE_APPLIED", __func__);
        break;
    case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        NRF_LOG_INFO("%s ---> PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED", __func__);
        break;

    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEER_DATA_UPDATE_SUCCEEDED", __func__);
        break;
    case PM_EVT_PEER_DATA_UPDATE_FAILED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEER_DATA_UPDATE_FAILED", __func__);
        break;
    case PM_EVT_PEER_DELETE_SUCCEEDED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEER_DELETE_SUCCEEDED", __func__);
        break;
    case PM_EVT_PEER_DELETE_FAILED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEER_DELETE_FAILED", __func__);
        break;
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEERS_DELETE_SUCCEEDED", __func__);
        bt_advertising_ctrl(true, false);
        break;
    case PM_EVT_PEERS_DELETE_FAILED:
        NRF_LOG_INFO("%s ---> PM_EVT_PEERS_DELETE_FAILED", __func__);
        break;

    default:
        break;
    }
}
#endif

void mac_address_get(void)
{
    ble_gap_addr_t Mac_address;
    unsigned char i, j = 0;

    uint32_t err_code = sd_ble_gap_addr_get(&Mac_address);
    APP_ERROR_CHECK(err_code);

    memcpy(mac, Mac_address.addr, BLE_GAP_ADDR_LEN);

    for ( i = 0; i < BLE_GAP_ADDR_LEN; i++ )
    {
        if ( (mac[i] >> 4) < 0x0a )
        {
            mac_ascii[j] = 0x30 + (mac[i] >> 4);
            j++;
        }
        else
        {
            mac_ascii[j] = 0x41 + (((mac[i] >> 4) & 0x0f) - 0x0A); //(mac[i]>>4)&0x0f-0x0A
            j++;
        }

        if ( (mac[i] & 0x0f) < 0x0a )
        {
            mac_ascii[j] = 0x30 + (mac[i] & 0x0f);
            j++;
        }
        else
        {
            mac_ascii[j] = 0x41 + ((mac[i] & 0x0f) - 0x0A); // mac[i]&0x0f-0x0A
            j++;
        }
    }
    memcpy(&ble_adv_name[0], ADV_HEAD_NAME, HEAD_NAME_LENGTH);
    memcpy(&ble_adv_name[HEAD_NAME_LENGTH], mac_ascii, ADV_NAME_LENGTH - HEAD_NAME_LENGTH);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t*)ble_adv_name, sizeof(ble_adv_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static uint16_t m_ble_gatt_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer
                                     by the Nordic UART service module. */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t* p_gatt, const nrf_ble_gatt_evt_t* p_evt)
{
    if ( (m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) )
    {
        m_ble_gatt_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_gatt_max_data_len, m_ble_gatt_max_data_len);
    }
    NRF_LOG_DEBUG(
        "ATT MTU exchange completed. central 0x%x peripheral 0x%x", p_gatt->att_mtu_desired_central,
        p_gatt->att_mtu_desired_periph
    );
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void ble_nus_send_packet(uint8_t* data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t length = 0;

    do
    {
        length = data_len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : data_len;
        err_code = ble_nus_data_send(&m_nus, data, &length, m_conn_handle);
        if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
             (err_code != NRF_ERROR_NOT_FOUND) )
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    while ( err_code == NRF_ERROR_RESOURCES );
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t* p_evt)
{
    NRF_LOG_INFO("----> nus_data_handler CALLED");
    static uint32_t msg_len;
    uint32_t pad;
    uint8_t nus_data_buf[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH] = {0};
    uint32_t nus_data_len = 0;

    if ( p_evt->type == BLE_NUS_EVT_RX_DATA )
    {
        NRF_LOG_INFO("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        nus_data_len = p_evt->params.rx_data.length;
        memcpy(nus_data_buf, (uint8_t*)p_evt->params.rx_data.p_data, nus_data_len);

        if ( rcv_head_flag == DATA_INIT )
        {
            if ( nus_data_buf[0] == '?' && nus_data_buf[1] == '#' && nus_data_buf[2] == '#' )
            {
                if ( nus_data_len < 9 )
                {
                    return;
                }
                else
                {
                    msg_len = (uint32_t)((nus_data_buf[5] << 24) + (nus_data_buf[6] << 16) + (nus_data_buf[7] << 8) +
                                         (nus_data_buf[8]));
                    pad = ((nus_data_len + 63) / 64) + 8;
                    if ( msg_len > nus_data_len - pad )
                    {
                        msg_len -= nus_data_len - pad;
                        rcv_head_flag = DATA_HEAD;
                    }
                    start_data_wait_timer();
                }
            }
            else if ( nus_data_buf[0] == 0x5A && nus_data_buf[1] == 0xA5 && nus_data_buf[2] == 0x07 &&
                      nus_data_buf[3] == 0x1 && nus_data_buf[4] == 0x03 )
            {
                ble_adv_switch_flag = BLE_OFF_ALWAYS;
                return;
            }
        }
        else
        {
            if ( nus_data_buf[0] == '?' )
            {
                pad = (nus_data_len + 63) / 64;
                if ( nus_data_len - pad > msg_len )
                {
                    rcv_head_flag = DATA_INIT;
                    nus_data_len = msg_len + (msg_len + 62) / 63;
                    msg_len = 0;
                }
                else
                {
                    msg_len -= nus_data_len - pad;
                }
            }
            else
            {
                rcv_head_flag = DATA_INIT;
            }
        }
        // spi_write_st_data(nus_data_buf, nus_data_len);
        app_sched_event_put(nus_data_buf, nus_data_len, spi_write_st_data);
    }
    else if ( p_evt->type == BLE_NUS_EVT_TX_RDY )
    {
        uint32_t length = 0;

        length = ble_nus_send_len - ble_nus_send_offset;
        if ( length > 0 )
        {
            length = length > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : length;
            ble_nus_send_packet(ble_nus_send_buf + ble_nus_send_offset, length);
            ble_nus_send_offset += length;
        }
        else
        {
            ble_nus_send_len = 0;
            ble_nus_send_offset = 0;
        }
    }
}

#include "fido.h"

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_init_t dis_init;
    ble_nus_init_t nus_init;
    ble_fido_init_t fido_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    sys_bas_init();

#if BLE_DIS_ENABLED
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, ble_adv_name);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FW_REVISION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SW_REVISION);

    ble_dis_sys_id_t system_id;
    system_id.manufacturer_id = MANUFACTURER_ID;
    system_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id = &system_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif

    // Initialize FIDO.
    memset(&fido_init, 0, sizeof(fido_init));
    fido_init.data_handler = fido_data_handler;
    err_code = ble_fido_init(&m_fido, &fido_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_1s_timer_id, APP_TIMER_MODE_REPEATED, m_1s_timeout_hander);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&data_wait_timer_id, APP_TIMER_MODE_SINGLE_SHOT, data_wait_timeout_hander);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start battery timer
    err_code = app_timer_start(m_battery_timer_id, BATTERY_MEAS_LONG_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    // Start application timers.
    err_code = app_timer_start(m_1s_timer_id, ONE_SECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    // Start data wait timer
    // err_code = app_timer_start(data_wait_timer_id, RCV_DATA_TIMEOUT_INTERVAL, NULL);
    // APP_ERROR_CHECK(err_code);
}

void start_data_wait_timer(void)
{
    ret_code_t err_code = app_timer_start(data_wait_timer_id, RCV_DATA_TIMEOUT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void stop_data_wait_timer(void)
{
    ret_code_t err_code = app_timer_stop(data_wait_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t* p_evt)
{
    ret_code_t err_code;

    if ( p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED )
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch ( ble_adv_evt )
    {
    case BLE_ADV_EVT_IDLE:
        NRF_LOG_INFO("ble_adv_evt_t -> BLE_ADV_EVT_IDLE");
        break;
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("ble_adv_evt_t -> BLE_ADV_EVT_FAST");
        break;

    default:
        break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(const ble_evt_t* p_ble_evt, void* p_context)
{
#ifdef BOND_ENABLE
    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

    switch ( p_ble_evt->header.evt_id )
    {

        // GAP

    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_CONNECTED", __func__);
        {
            bak_buff[0] = BLE_CMD_CON_STA;
            bak_buff[1] = BLE_CON_STATUS;
            send_stm_data(bak_buff, 2);

            m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_DEFAULT);
            // Start Security Request timer.
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_DISCONNECTED", __func__);
        {
            bond_check_key_flag = INIT_VALUE;
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            bak_buff[0] = BLE_CMD_CON_STA;
            bak_buff[1] = BLE_DISCON_STATUS;
            send_stm_data(bak_buff, 2);

            // Check if the last connected peer had not used MITM, if so, delete its bond information.
            if ( m_peer_to_be_deleted != PM_PEER_ID_INVALID )
            {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
        }
        break;

        // BLE_GAP_EVT_CONN_PARAM_UPDATE

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_SEC_PARAMS_REQUEST", __func__);
        break;

        // BLE_GAP_EVT_SEC_INFO_REQUEST

    case BLE_GAP_EVT_PASSKEY_DISPLAY:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_PASSKEY_DISPLAY", __func__);
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;

            ble_conn_nopair_flag = BLE_PAIR;
            bak_buff[0] = BLE_CMD_PAIR_CODE;
            memcpy(&bak_buff[1], passkey, PASSKEY_LENGTH);
            send_stm_data(bak_buff, 1 + PASSKEY_LENGTH);

            NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
        }
        break;

        // BLE_GAP_EVT_KEY_PRESSED

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_AUTH_KEY_REQUEST", __func__);
        break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_LESC_DHKEY_REQUEST", __func__);
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_AUTH_STATUS", __func__);
        NRF_LOG_INFO(
            "BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
            p_ble_evt->evt.gap_evt.params.auth_status.auth_status, p_ble_evt->evt.gap_evt.params.auth_status.bonded,
            p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
            *((uint8_t*)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
            *((uint8_t*)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer)
        );
        bond_check_key_flag = AUTH_VALUE;
        break;

        // case BLE_GAP_EVT_CONN_SEC_UPDATE:
        //     NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_CONN_SEC_UPDATE", __func__);
        //     break;

        // case BLE_GAP_EVT_TIMEOUT:
        //     NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_TIMEOUT", __func__);
        //     err_code =
        //         sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        //         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //     APP_ERROR_CHECK(err_code);
        //     break;

        // BLE_GAP_EVT_RSSI_CHANGED
        // BLE_GAP_EVT_ADV_REPORT
        // BLE_GAP_EVT_SEC_REQUEST
        // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST
        // BLE_GAP_EVT_SCAN_REQ_REPORT

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        NRF_LOG_DEBUG("%s ---> BLE_GAP_EVT_PHY_UPDATE_REQUEST", __func__);
        {
            const ble_gap_phys_t phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
        break;

        // BLE_GAP_EVT_PHY_UPDATE
        // BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST
        // BLE_GAP_EVT_DATA_LENGTH_UPDATE
        // BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT
        // BLE_GAP_EVT_ADV_SET_TERMINATED

        // GATTC

    case BLE_GATTC_EVT_TIMEOUT:
        NRF_LOG_DEBUG("%s ---> BLE_GATTC_EVT_TIMEOUT", __func__);
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

        // GATTS

    case BLE_GATTS_EVT_TIMEOUT:
        NRF_LOG_DEBUG("%s ---> BLE_GATTS_EVT_TIMEOUT", __func__);
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

        // case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        //     // No system attributes have been stored.
        //     err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
        //     APP_ERROR_CHECK(err_code);
        //     break;

    default:
        // No implementation needed.
        break;
    }
#else
    uint32_t err_code;

    switch ( p_ble_evt->header.evt_id )
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        // LED indication will be changed when advertising starts.
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            const ble_gap_phys_t phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
#endif
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
#ifdef BOND_ENABLE
/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = 1;
    sec_param.kdist_own.id = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t* p_event)
{
    static uint8_t index = 0;
    static uint32_t lenth = 0;
    uint8_t uart_xor_byte;

    switch ( p_event->evt_type )
    {
    case APP_UART_DATA_READY:
        UNUSED_VARIABLE(app_uart_get(&uart_data_array[index]));
        index++;
        // NRF_LOG_INFO("receive uart data.")
        if ( 1 == index )
        {
            if ( UART_TX_TAG != uart_data_array[0] )
            {
                index = 0;
                return;
            }
        }
        else if ( 2 == index )
        {
            if ( UART_TX_TAG2 != uart_data_array[1] )
            {
                index = 0;
                return;
            }
        }
        else if ( 3 == index )
        {
            if ( (UART_TX_TAG2 == uart_data_array[0]) && (UART_TX_TAG == uart_data_array[1]) )
            {
                index = 0;
                return;
            }
        }
        else if ( 4 == index )
        {
            lenth = ((uint32_t)uart_data_array[2] << 8) + uart_data_array[3];
        }
        else if ( index >= lenth + 4 )
        {
            uart_xor_byte = calcXor(uart_data_array, index - 1);
            if ( uart_xor_byte != uart_data_array[index - 1] )
            {
                index = 0;
                return;
            }

            switch ( uart_data_array[4] )
            {
            case ST_CMD_BLE:
                switch ( uart_data_array[5] )
                {
                case ST_SEND_OPEN_BLE:
                    ble_adv_switch_flag = BLE_ON_ALWAYS;
                    NRF_LOG_INFO("RCV ble always ON.");
                    break;
                case ST_SEND_CLOSE_BLE:
                    ble_adv_switch_flag = BLE_OFF_ALWAYS;
                    NRF_LOG_INFO("RCV ble always OFF.");
                    break;
                case ST_SEND_DISCON_BLE:
                    ble_conn_flag = BLE_DISCON;
                    NRF_LOG_INFO("RCV ble flag disconnect.");
                    break;
                case ST_GET_BLE_SWITCH_STATUS:
                    ble_conn_flag = BLE_CON;
                    break;
                default:
                    break;
                }
                break;
            case ST_CMD_POWER:
                switch ( uart_data_array[5] )
                {
                case ST_SEND_CLOSE_SYS_PWR:
                    pwr_status_flag = PWR_SHUTDOWN_SYS;
                    break;
                case ST_SEND_CLOSE_EMMC_PWR:
                    pwr_status_flag = PWR_CLOSE_EMMC;
                    break;
                case ST_SEND_OPEN_EMMC_PWR:
                    pwr_status_flag = PWR_OPEN_EMMC;
                    break;
                case ST_REQ_POWER_PERCENT:
                    pwr_status_flag = PWR_BAT_PERCENT;
                    break;
                case ST_REQ_USB_STATUS:
                    pwr_status_flag = PWR_USB_STATUS;
                    break;
                case ST_REQ_ENABLE_CHARGE:
                    pmu_feat_charge_enable = true;
                    pmu_feat_synced = false;
                    NRF_LOG_INFO("RCV pwr charge enable");
                    break;
                case ST_REQ_DISABLE_CHARGE:
                    pmu_feat_charge_enable = false;
                    pmu_feat_synced = false;
                    NRF_LOG_INFO("RCV pwr charge disable");
                    break;
                default:
                    pwr_status_flag = PWR_DEF;
                    break;
                }
                break;
            case ST_CMD_BLE_INFO:
                switch ( uart_data_array[5] )
                {
                case ST_REQ_ADV_NAME:
                    trans_info_flag = RESPONESE_NAME;
                    break;
                case ST_REQ_FIRMWARE_VER:
                    trans_info_flag = RESPONESE_VER;
                    break;
                case ST_REQ_SOFTDEVICE_VER:
                    trans_info_flag = RESPONESE_SD_VER;
                    break;
                case ST_REQ_BOOTLOADER_VER:
                    trans_info_flag = RESPONESE_BOOT_VER;
                    break;
                case ST_REQ_BUILD_ID:
                    trans_info_flag = RESPONESE_BUILD_ID;
                    break;
                case ST_REQ_HASH:
                    trans_info_flag = RESPONESE_HASH;
                    break;
                case ST_REQ_BT_MAC:
                    trans_info_flag = RESPONESE_BT_MAC;
                    break;
                default:
                    trans_info_flag = UART_DEF;
                    break;
                }
                break;
            case ST_CMD_RESET_BLE:
                if ( ST_VALUE_RESET_BLE == uart_data_array[5] )
                {
                    NVIC_SystemReset();
                }
                break;
            case ST_CMD_LED:
                if ( ST_SEND_GET_LED_BRIGHTNESS == uart_data_array[5] )
                {
                    led_brightness_flag = LED_GET_BRIHTNESS;
                }
                else if ( ST_SEND_SET_LED_BRIGHTNESS == uart_data_array[5] )
                {
                    led_brightness_flag = LED_SET_BRIHTNESS;
                    led_brightness_value = uart_data_array[6];
                    led_brightness_synced = false;
                }
                break;

            case STM_CMD_BAT:
                switch ( uart_data_array[5] )
                {
                case STM_SEND_BAT_VOL:
                    bat_msg_flag = SEND_BAT_VOL;
                    break;
                case STM_SEND_BAT_CHARGE_CUR:
                    bat_msg_flag = SEND_BAT_CHARGE_CUR;
                    break;
                case STM_SEND_BAT_DISCHARGE_CUR:
                    bat_msg_flag = SEND_BAT_DISCHARGE_CUR;
                    break;
                case STM_SEND_BAT_INNER_TEMP:
                    bat_msg_flag = SEND_BAT_INNER_TEMP;
                    break;
                default:
                    bat_msg_flag = BAT_DEF;
                    break;
                }
                break;
            case STM_CMD_KEY:
                switch ( (uart_data_array[5]) )
                {
                case STM_GET_PUBKEY:
                    trans_info_flag = RESPONESE_BLE_PUBKEY;
                    break;
                case STM_LOCK_PUBKEY:
                    trans_info_flag = RESPONESE_BLE_PUBKEY_LOCK;
                    break;
                case STM_REQUEST_SIGN:
                    trans_info_flag = RESPONESE_BLE_SIGN;
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
            index = 0;
        }
        break;
    // ignore uart error
    case APP_UART_COMMUNICATION_ERROR:
        // APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        // APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
    NRF_LOG_FLUSH();
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void usr_uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params = {
        .rx_pin_no = RX_PIN_NUMBER,
        .tx_pin_no = TX_PIN_NUMBER,
        .rts_pin_no = RTS_PIN_NUMBER,
        .cts_pin_no = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = false,
        .baud_rate = NRF_UARTE_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(
        &comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code
    );
    APP_ERROR_CHECK(err_code);
    app_uart_is_initialized = true;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    ble_advertising_init_t init;
    // ble_advdata_manuf_data_t   manuf_data;
    // uint8_t m_addl_adv_manuf_data[MAC_ADDRESS_LENGTH];

    memset(&init, 0, sizeof(init));

    // manuf_data.company_identifier = COMPANY_IDENTIFIER;
    // manuf_data.data.size          = ADV_ADDL_MANUF_DATA_LEN;
    // memcpy(m_addl_adv_manuf_data,mac,MAC_ADDRESS_LENGTH);
    // manuf_data.data.p_data        = m_addl_adv_manuf_data;

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;
    // init.advdata.p_manuf_specific_data = &manuf_data;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void ble_nus_send(uint8_t* data, uint16_t len)
{
    uint16_t length = 0;

    NRF_LOG_INFO("ble_nus_send_len: %d", len);

    if ( m_conn_handle == BLE_CONN_HANDLE_INVALID )
    {
        return;
    }

    ble_nus_send_buf = data;
    ble_nus_send_len = len;

    length = len > m_ble_gatt_max_data_len ? m_ble_gatt_max_data_len : len;
    ble_nus_send_offset = length;
    ble_nus_send_packet(data, length);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    if ( (BLE_DEF == ble_adv_switch_flag) || (BLE_ON_ALWAYS == ble_adv_switch_flag) )
    {
        if ( bond_check_key_flag != AUTH_VALUE )
        {
            err_code = nrf_ble_lesc_request_handler();
            APP_ERROR_CHECK(err_code);
        }
    }
    if ( NRF_LOG_PROCESS() == false )
    {
        nrf_pwr_mgmt_run();
    }
}

void in_gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch ( pin )
    {
    case SLAVE_SPI_RSP_IO:
        NRF_LOG_INFO("GPIO IRQ -> SLAVE_SPI_RSP_IO");
        if ( spi_dir_out )
        {
            spi_dir_out = false;
        }
        else if ( nrf_gpio_pin_read(SLAVE_SPI_RSP_IO) == 0 && !spi_dir_out )
        {
            // spi_read_st_data(NULL, 0);
            app_sched_event_put(NULL, 0, spi_read_st_data);
        }
        break;
    default:
        break;
    }
    NRF_LOG_FLUSH();
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrfx_gpiote_in_init(SLAVE_SPI_RSP_IO, &in_config, in_gpiote_handler);
    APP_ERROR_CHECK(err_code);
    nrfx_gpiote_in_event_enable(SLAVE_SPI_RSP_IO, true);

    nrf_gpio_cfg_input(PMIC_PWROK_IO, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PMIC_IRQ_IO, NRF_GPIO_PIN_PULLUP);
}

static uint8_t calcXor(uint8_t* buf, uint8_t len)
{
    uint8_t tmp = 0;
    uint8_t i;

    for ( i = 0; i < len; i++ )
    {
        tmp ^= buf[i];
    }
    return tmp;
}
static void uart_put_data(uint8_t* pdata, uint8_t lenth)
{
    uint32_t err_code;

    while ( lenth-- )
    {
        do
        {
            err_code = app_uart_put(*pdata++);
            if ( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) )
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        while ( err_code == NRF_ERROR_BUSY );
    }
}

static void send_stm_data(uint8_t* pdata, uint8_t lenth)
{
    uart_trans_buff[0] = UART_TX_TAG2;
    uart_trans_buff[1] = UART_TX_TAG;
    uart_trans_buff[2] = 0x00;
    uart_trans_buff[3] = lenth + 1;
    memcpy(&uart_trans_buff[4], pdata, lenth);
    uart_trans_buff[uart_trans_buff[3] + 3] = calcXor(uart_trans_buff, (uart_trans_buff[3] + 3));

    uart_put_data(uart_trans_buff, uart_trans_buff[3] + 4);
}

static bool bt_disconnect()
{
    if ( m_conn_handle != BLE_CONN_HANDLE_INVALID )
    {
        if ( NRF_SUCCESS != sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION) )
            return false;
        NRF_LOG_INFO("bt_disconnect");
    }
    int i = 0;
    while ( m_conn_handle != BLE_CONN_HANDLE_INVALID )
    {
        nrf_delay_ms(10);
        if ( i++ > 100 )
            return false;
    }

    return true;
}

static bool bt_advertising_ctrl(bool enable, bool commit)
{
    if ( enable )
    {
        ble_status_flag = BLE_ON_ALWAYS;
        deviceConfig_p->settings.bt_ctrl = 0; // turn on
        if ( NRF_SUCCESS != ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST) )
            return false;
    }
    else
    {
        // disconnect (if connected)
        if ( !bt_disconnect() )
            return false;

        // turn off
        ble_status_flag = BLE_OFF_ALWAYS;
        deviceConfig_p->settings.bt_ctrl = DEVICE_CONFIG_FLAG_MAGIC;
        if ( NRF_SUCCESS != ble_advertising_stop(&m_advertising) )
            return false;
    }

    if ( commit )
        if ( !device_config_commit() )
            return false;

    return true;
}

static void rsp_st_uart_cmd(void* p_event_data, uint16_t event_size)
{
    if ( trans_info_flag == DEF_RESP )
    {
        return;
    }

    memset(bak_buff, 0x00, sizeof(bak_buff));
    switch ( trans_info_flag )
    {
    case RESPONESE_NAME:
        bak_buff[0] = BLE_CMD_ADV_NAME;
        memcpy(&bak_buff[1], (uint8_t*)ble_adv_name, ADV_NAME_LENGTH);
        send_stm_data(bak_buff, 1 + ADV_NAME_LENGTH);
        break;

    case RESPONESE_BT_MAC:
        bak_buff[0] = BLE_CMD_BT_MAC;
        memcpy(&bak_buff[1], (uint8_t*)mac, BLE_GAP_ADDR_LEN);
        send_stm_data(bak_buff, 1 + BLE_GAP_ADDR_LEN);
        trans_info_flag = DEF_RESP;
        break;

    case RESPONESE_VER:
        bak_buff[0] = BLE_FIRMWARE_VER;
        memcpy(&bak_buff[1], FW_REVISION, sizeof(FW_REVISION) - 1);
        send_stm_data(bak_buff, sizeof(FW_REVISION));
        break;

    case RESPONESE_SD_VER:
        bak_buff[0] = BLE_SOFTDEVICE_VER;
        memcpy(&bak_buff[1], SW_REVISION, sizeof(SW_REVISION) - 1);
        send_stm_data(bak_buff, sizeof(SW_REVISION));
        break;

    case RESPONESE_BOOT_VER:
        bak_buff[0] = BLE_BOOTLOADER_VER;
        memcpy(&bak_buff[1], BT_REVISION, sizeof(BT_REVISION) - 1);
        send_stm_data(bak_buff, sizeof(BT_REVISION));
        break;

    case RESPONESE_BLE_PUBKEY:
        bak_buff[0] = BLE_CMD_KEY_RESP;
        if ( deviceConfig_p->keystore.flag_locked == DEVICE_CONFIG_FLAG_MAGIC )
        {
            bak_buff[1] = BLE_KEY_RESP_FAILED;
            send_stm_data(bak_buff, 2);
        }
        else if ( !deviceCfg_keystore_validate(&(deviceConfig_p->keystore)) )
        {
            bak_buff[1] = BLE_KEY_RESP_FAILED;
            send_stm_data(bak_buff, 2);
        }
        else
        {
            bak_buff[1] = BLE_KEY_RESP_PUBKEY;
            memcpy(&bak_buff[2], deviceConfig_p->keystore.public_key, sizeof(deviceConfig_p->keystore.public_key));
            send_stm_data(bak_buff, sizeof(deviceConfig_p->keystore.public_key) + 2);
        }
        break;

    case RESPONESE_BLE_PUBKEY_LOCK:
        bak_buff[0] = BLE_CMD_KEY_RESP;
        bak_buff[1] =
            ((deviceCfg_keystore_lock(&(deviceConfig_p->keystore)) && device_config_commit()) ? BLE_KEY_RESP_SUCCESS
                                                                                              : BLE_KEY_RESP_FAILED);
        send_stm_data(bak_buff, 2);
        break;

    case RESPONESE_BLE_SIGN:
        uint32_t msg_len = (uart_data_array[2] << 8 | uart_data_array[3]) - 3;
        bak_buff[0] = BLE_CMD_KEY_RESP;
        if ( !deviceCfg_keystore_validate(&(deviceConfig_p->keystore)) )
        {
            bak_buff[1] = BLE_KEY_RESP_FAILED;
            send_stm_data(bak_buff, 2);
        }
        else
        {
            bak_buff[1] = BLE_KEY_RESP_SIGN;
            if ( deviceConfig_p->keystore.flag_locked != DEVICE_CONFIG_FLAG_MAGIC )
            {
                deviceCfg_keystore_lock(&(deviceConfig_p->keystore));
                device_config_commit();
            }
            sign_ecdsa_msg(deviceConfig_p->keystore.private_key, uart_data_array + 6, msg_len, bak_buff + 2);
            send_stm_data(bak_buff, 64 + 2);
        }
        break;

    case RESPONESE_BUILD_ID:
        bak_buff[0] = BLE_CMD_BUILD_ID;
        memcpy(&bak_buff[1], (uint8_t*)BUILD_ID, 7);
        send_stm_data(bak_buff, 8);
        break;

    case RESPONESE_HASH:
        ret_code_t err_code = NRF_SUCCESS;
        nrf_crypto_backend_hash_context_t hash_context = {0};
        uint8_t hash[32] = {0};
        size_t hash_len = 32;
        int chunks = 0;
        int app_size = 0;
        uint8_t* code_addr = (uint8_t*)0x26000;
        uint8_t* code_len = (uint8_t*)0x7F018;
        app_size = code_len[0] + code_len[1] * 256 + code_len[2] * 256 * 256;
        chunks = app_size / 512;

        err_code = nrf_crypto_hash_init(&hash_context, &g_nrf_crypto_hash_sha256_info);
        APP_ERROR_CHECK(err_code);
        for ( int i = 0; i < chunks; i++ )
        {
            err_code = nrf_crypto_hash_update(&hash_context, code_addr + i * 512, 512);
            APP_ERROR_CHECK(err_code);
        }
        if ( app_size % 512 )
        {
            err_code = nrf_crypto_hash_update(&hash_context, code_addr + chunks * 512, app_size % 512);
            APP_ERROR_CHECK(err_code);
        }
        err_code = nrf_crypto_hash_finalize(&hash_context, hash, &hash_len);
        APP_ERROR_CHECK(err_code);

        bak_buff[0] = BLE_CMD_HASH;
        memcpy(&bak_buff[1], hash, 32);
        send_stm_data(bak_buff, 33);
        break;
    default:
        break;
    }
    trans_info_flag = DEF_RESP;
}
static void manage_bat_level(void* p_event_data, uint16_t event_size)
{
    static uint8_t bak_bat_persent = 0x00;

    if ( bak_bat_persent != pmu_p->PowerStatus->batteryPercent )
    {
        bak_bat_persent = pmu_p->PowerStatus->batteryPercent;
        bak_buff[0] = BLE_SYSTEM_POWER_PERCENT;
        bak_buff[1] = pmu_p->PowerStatus->batteryPercent;
        send_stm_data(bak_buff, 2);
    }
}
static void ble_ctl_process(void* p_event_data, uint16_t event_size)
{
    uint8_t respons_flag = 0;

    if ( BLE_OFF_ALWAYS == ble_adv_switch_flag )
    {
        ble_adv_switch_flag = BLE_DEF;
        if ( BLE_ON_ALWAYS == ble_status_flag )
        {
            // disconnect, stop adv, commit
            bt_advertising_ctrl(false, true);
        }
        bak_buff[0] = BLE_CMD_CON_STA;
        bak_buff[1] = BLE_ADV_OFF_STATUS;
        send_stm_data(bak_buff, 2);
    }
    else if ( BLE_ON_ALWAYS == ble_adv_switch_flag )
    {
        ble_adv_switch_flag = BLE_DEF;
        if ( BLE_OFF_ALWAYS == ble_status_flag )
        {
            bt_advertising_ctrl(true, true);
            NRF_LOG_INFO("2-Start advertisement.\n");
        }

        bak_buff[0] = BLE_CMD_CON_STA;
        bak_buff[1] = BLE_ADV_ON_STATUS;
        send_stm_data(bak_buff, 2);
    }

    if ( BLE_DISCON == ble_conn_flag )
    {
        ble_conn_flag = BLE_DEF;
        bak_buff[0] = BLE_CMD_CON_STA;
        bak_buff[1] = BLE_DISCON_STATUS;
        send_stm_data(bak_buff, 2);

        bt_disconnect();
    }
    if ( BLE_CON == ble_conn_flag )
    {
        ble_conn_flag = BLE_DEF;
        bak_buff[0] = BLE_CMD_CON_STA;
        bak_buff[1] = ble_status_flag + 2;
        send_stm_data(bak_buff, 2);
    }

    switch ( pwr_status_flag )
    {
    case PWR_SHUTDOWN_SYS:
        pwr_status_flag = PWR_DEF;

        bak_buff[0] = BLE_CMD_PWR_STA;
        bak_buff[1] = BLE_CLOSE_SYSTEM;
        send_stm_data(bak_buff, 2);

        if ( ble_status_flag != BLE_OFF_ALWAYS )
        {
            bt_disconnect();
        }
        pmu_p->SetState(PWR_STATE_HARD_OFF);
        break;
    case PWR_CLOSE_EMMC:
        respons_flag = BLE_CLOSE_EMMC;
        // ctl_emmc_power(AXP_CLOSE_EMMC);
        break;
    case PWR_OPEN_EMMC:
        respons_flag = BLE_OPEN_EMMC;
        // ctl_emmc_power(AXP_OPEN_EMMC);
        break;
    case PWR_BAT_PERCENT:
        pwr_status_flag = PWR_DEF;
        bak_buff[0] = BLE_SYSTEM_POWER_PERCENT;
        bak_buff[1] = pmu_p->PowerStatus->batteryPercent;
        send_stm_data(bak_buff, 2);
        break;
    case PWR_USB_STATUS:
        pwr_status_flag = PWR_DEF;
        bak_buff[0] = BLE_CMD_POWER_STA;

        if ( pmu_p->PowerStatus->chargerAvailable )
        {
            // bak_buff[1] =
            //     ((pmu_p->PowerStatus->chargeFinished && pmu_p->PowerStatus->chargeAllowed) ? BLE_CHAGE_OVER
            //                                                                                : BLE_CHARGING_PWR);
            bak_buff[1] = BLE_CHARGING_PWR;
            bak_buff[2] = (pmu_p->PowerStatus->wiredCharge ? AXP_CHARGE_TYPE_USB : AXP_CHARGE_TYPE_WIRELESS);
        }
        else
        {
            bak_buff[1] = BLE_REMOVE_POWER;
            bak_buff[2] = 0;
        }
        send_stm_data(bak_buff, 3);
        break;
    default:
        break;
    }
    if ( (PWR_DEF != pwr_status_flag) && (respons_flag != 0x00) )
    {
        bak_buff[0] = BLE_CMD_PWR_STA;
        bak_buff[1] = respons_flag;
        send_stm_data(bak_buff, 2);

        pwr_status_flag = PWR_DEF;
    }
}

static inline void pmu_status_print()
{
    NRF_LOG_INFO("=== PowerStatus ===");
    NRF_LOG_INFO("PMIC_IRQ_IO -> %s", (nrf_gpio_pin_read(PMIC_IRQ_IO) ? "HIGH" : "LOW"));
    NRF_LOG_INFO("sysVoltage=%lu", pmu_p->PowerStatus->sysVoltage);
    NRF_LOG_INFO("batteryPresent=%u", pmu_p->PowerStatus->batteryPresent);
    NRF_LOG_INFO("batteryPercent=%u", pmu_p->PowerStatus->batteryPercent);
    NRF_LOG_INFO("batteryVoltage=%lu", pmu_p->PowerStatus->batteryVoltage);
    NRF_LOG_INFO("batteryTemp=%ld", pmu_p->PowerStatus->batteryTemp);
    NRF_LOG_INFO("pmuTemp=%lu", pmu_p->PowerStatus->pmuTemp);
    NRF_LOG_INFO("chargeAllowed=%u", pmu_p->PowerStatus->chargeAllowed);
    NRF_LOG_INFO("chargerAvailable=%u", pmu_p->PowerStatus->chargerAvailable);
    NRF_LOG_INFO("chargeFinished=%u", pmu_p->PowerStatus->chargeFinished);
    NRF_LOG_INFO("wiredCharge=%u", pmu_p->PowerStatus->wiredCharge);
    NRF_LOG_INFO("wirelessCharge=%u", pmu_p->PowerStatus->wirelessCharge);
    NRF_LOG_INFO("chargeCurrent=%lu", pmu_p->PowerStatus->chargeCurrent);
    NRF_LOG_INFO("dischargeCurrent=%lu", pmu_p->PowerStatus->dischargeCurrent);
    NRF_LOG_INFO("=== ============== ===");
    NRF_LOG_FLUSH();
}

static void pmu_req_process(void* p_event_data, uint16_t event_size)
{
    // features control
    if ( !pmu_feat_synced )
    {
        PRINT_CURRENT_LOCATION();
        if ( pmu_feat_charge_enable )
            pmu_p->SetFeature(PWR_FEAT_CHARGE, true); // enable charge
        else
            pmu_p->SetFeature(PWR_FEAT_CHARGE, false); // disable charge

        pmu_feat_synced = true;
    }
}

static void pmu_status_refresh(void* p_event_data, uint16_t event_size)
{
    if ( pmu_status_synced )
        return;

    PRINT_CURRENT_LOCATION();
    pmu_p->PullStatus();
    pmu_status_synced = true;
    pmu_status_print();
}

static void pmu_pwrok_pull(void* p_event_data, uint16_t event_size)
{
    static uint8_t match_count = 0;
    const uint8_t match_required = 10;

    if ( !nrf_gpio_pin_read(PMIC_PWROK_IO) )
    {
        match_count++;
        NRF_LOG_INFO("PowerOK debounce, match %u/%u", match_count, match_required);
    }
    else
    {
        if ( match_count > 0 )
        {
            match_count = 0;
            NRF_LOG_INFO("PowerOK debounce, match reset");
            NRF_LOG_FLUSH();
        }
    }

    if ( (match_count >= match_required) )
    {
        NRF_LOG_INFO("PowerOK debounce, match fulfilled, entering low power mode");
        NRF_LOG_FLUSH();
        enter_low_power_mode();
    }
}

static void pmu_irq_pull(void* p_event_data, uint16_t event_size)
{
    if ( !nrf_gpio_pin_read(PMIC_IRQ_IO) )
    {
        PRINT_CURRENT_LOCATION();

        // wait charger status stabilize before process irq
        // chargerAvailable may take few ms to be set in some case

        Power_Status_t pwr_status_temp = {0};
        uint8_t match_count = 0;
        const uint8_t match_required = 3;
        while ( match_count < match_required )
        {
            pmu_p->PullStatus();

            if ( (pwr_status_temp.chargerAvailable == pmu_p->PowerStatus->chargerAvailable) &&
                 (pwr_status_temp.wiredCharge == pmu_p->PowerStatus->wiredCharge) &&
                 (pwr_status_temp.wirelessCharge == pmu_p->PowerStatus->wirelessCharge) )
            {
                match_count++;
                NRF_LOG_INFO("PowerStatus debounce, match %u/%u", match_count, match_required);

                continue;
            }
            else
            {
                match_count = 0;
                NRF_LOG_INFO("PowerStatus debounce, match reset");

                pwr_status_temp.chargerAvailable = pmu_p->PowerStatus->chargerAvailable;
                pwr_status_temp.wiredCharge = pmu_p->PowerStatus->wiredCharge;
                pwr_status_temp.wirelessCharge = pmu_p->PowerStatus->wirelessCharge;
                nrf_delay_ms(10);
            }
        }
        pmu_status_synced = true;
        pmu_status_print();
        pmu_p->Irq();
    }
}

static void pmu_sys_voltage_monitor(void* p_event_data, uint16_t event_size)
{
    static uint8_t match_count = 0;
    const uint8_t match_required = 30;
    const uint16_t minimum_mv = 3300;

    if ( (match_count < match_required) )
    {
        // not triggered
        if ( pmu_p->PowerStatus->batteryVoltage < minimum_mv )
        {
            // voltage low
            if ( pmu_p->PowerStatus->chargerAvailable )
            {
                // has charger, ignore
                // NRF_LOG_INFO("Low batteryVoltage detect skiped since charger available");
                // reset counter
                match_count = 0;
            }
            else
            {
                // increase counter
                match_count++;
                NRF_LOG_INFO(
                    "Low batteryVoltage debounce, match %u/%u (batteryVoltage=%lu)", match_count, match_required,
                    pmu_p->PowerStatus->batteryVoltage
                );
            }
        }
        else
        {
            // voltage normal
            if ( match_count > 0 )
            {
                // reset counter if not zero
                match_count = 0;
                NRF_LOG_INFO("Low batteryVoltage debounce, match reset");
                NRF_LOG_FLUSH();
            }
        }
    }
    else
    {
        // already triggered
        NRF_LOG_INFO("Low batteryVoltage debounce, match fulfilled, force shutdown pmu");
        NRF_LOG_FLUSH();
        pmu_p->SetState(PWR_STATE_HARD_OFF);
    }
}

static void led_ctl_process(void* p_event_data, uint16_t event_size)
{
    // handle commands
    if ( ST_SEND_SET_LED_BRIGHTNESS == led_brightness_flag )
    {
        bak_buff[0] = BLE_CMD_FLASH_LED_STA;
        bak_buff[1] = led_brightness_flag;
        bak_buff[2] = led_brightness_value;
        send_stm_data(bak_buff, 3);
        led_brightness_flag = LED_DEF;
    }
    else if ( ST_SEND_GET_LED_BRIGHTNESS == led_brightness_flag )
    {
        bak_buff[0] = BLE_CMD_FLASH_LED_STA;
        bak_buff[1] = led_brightness_flag;
        bak_buff[2] = led_brightness_value;
        send_stm_data(bak_buff, 3);
        led_brightness_flag = LED_DEF;
    }

    // keep controller in sync
    if ( !led_brightness_synced )
    {
        if ( set_led_brightness(led_brightness_value) == NRF_SUCCESS )
            led_brightness_synced = true;
    }
}

static void bat_msg_report_process(void* p_event_data, uint16_t event_size)
{

    uint16_t val = 0;

    switch ( bat_msg_flag )
    {
    case SEND_BAT_VOL:
        val = pmu_p->PowerStatus->batteryVoltage;
        break;
    case SEND_BAT_CHARGE_CUR:
        val = pmu_p->PowerStatus->chargeCurrent;
        break;
    case SEND_BAT_DISCHARGE_CUR:
        val = pmu_p->PowerStatus->dischargeCurrent;
        break;
    case SEND_BAT_INNER_TEMP:
        val = (uint16_t)(pmu_p->PowerStatus->batteryTemp);
        break;
    default:
        return;
    }

    bak_buff[0] = BLE_CMD_BAT_CV_MSG;
    bak_buff[1] = bat_msg_flag;

    bak_buff[2] = (val & 0xFF00) >> 8;
    bak_buff[3] = (val & 0x00FF);

    send_stm_data(bak_buff, 4);

    bat_msg_flag = BAT_DEF;
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void m_wdt_event_handler(void)
{
    NRF_LOG_INFO("WDT Triggered!");
    NRF_LOG_FLUSH();
}

static void watch_dog_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    // Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, m_wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

int main(void)
{
    // ###############################
    // Critical Init Items
    // ==> Log
    log_init();
    NRF_LOG_INFO("Critical Init Seq.");
    NRF_LOG_FLUSH();
    // ==> Bus Fault
    // SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    // ==> lowlevel minimal
    gpio_init();
    power_management_init();
    // ==> NRF Crypto API
    nrf_crypto_init();
    // ==> Power Manage IC, LED Driver, and Device Configs
    CRITICAL_REGION_ENTER();
    // pmu init
    EXEC_RETRY(
        10, { set_send_stm_data_p(send_stm_data); },
        {
            nrf_delay_ms(10);
            NRF_LOG_INFO("Trying...");
            NRF_LOG_FLUSH();
            return (power_manage_init() && (pmu_p != NULL && pmu_p->isInitialized));
        },
        {
            NRF_LOG_INFO("PMU Init Success");
            NRF_LOG_FLUSH();
        },
        {
            NRF_LOG_INFO("PMU Init Fail");
            NRF_LOG_FLUSH();
            enter_low_power_mode(); // something wrong, shutdown to prevent battery drain
        }
    );
    // soft power off ST until self init done
    // pmu_p->SetState(PWR_STATE_SOFT_OFF);
    // device config init
    EXEC_RETRY(
        3, {}, { return device_config_init(); },
        {
            NRF_LOG_INFO("Config Init Success");
            NRF_LOG_FLUSH();
        },
        {
            NRF_LOG_INFO("Config Init Fail");
            NRF_LOG_FLUSH();
            enter_low_power_mode(); // something wrong, shutdown to prevent battery drain
        }
    );
    CRITICAL_REGION_EXIT();

    // ###############################
    // DFU Update
    // TODO: check battery?
    // TODO: change to on demand
    // NRF_LOG_INFO("DFU Update Seq.");
    // NRF_LOG_FLUSH();
    // NRF_LOG_INFO(try_dfu_upgrade(false) ? "DFU update not needed." : "DFU update failed!");
    // NRF_LOG_FLUSH();

    // ###############################
    // General Init Items
    NRF_LOG_INFO("General Init Seq.");
    NRF_LOG_FLUSH();
    usr_uart_init();
    usr_spim_init();
    timers_init();
    scheduler_init();
    watch_dog_init();

    // ###############################
    // Power Manage Init Items
    NRF_LOG_INFO("Power Config Seq.");
    NRF_LOG_FLUSH();
    // ST power on
    pmu_p->SetState(PWR_STATE_ON);
    // make sure light is off
    set_led_brightness(0);

    // ###############################
    // Bluetooth Init Items
    NRF_LOG_INFO("Bluetooth Init Seq.");
    NRF_LOG_FLUSH();
    ble_stack_init();
    mac_address_get();
#ifdef BOND_ENABLE
    peer_manager_init();
#endif
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    bt_advertising_ctrl(
        ((deviceConfig_p->settings.flag_initialized == DEVICE_CONFIG_FLAG_MAGIC) && // valid
         (deviceConfig_p->settings.bt_ctrl != DEVICE_CONFIG_FLAG_MAGIC))            // set to flag means turn off
        ,
        false
    ); // TODO: check return!

    // ###############################
    // Power mode and warning (depends on SD, has to be here)
    ExecuteCheck_ADV(sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE), NRF_SUCCESS, {
        NRF_LOG_INFO("NRF Power enable DCDC failed");
        NRF_LOG_FLUSH();
        enter_low_power_mode();
    });
    ExecuteCheck_ADV(sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V28), NRF_SUCCESS, {
        NRF_LOG_INFO("NRF Power set POF threadhold failed");
        NRF_LOG_FLUSH();
        enter_low_power_mode();
    });
    ExecuteCheck_ADV(sd_power_pof_enable(true), NRF_SUCCESS, {
        NRF_LOG_INFO("NRF Power enable POF failed");
        NRF_LOG_FLUSH();
        enter_low_power_mode();
    });
    NRF_LOG_INFO("NRF Power configured");

    // ###############################
    // Main Loop
    NRF_LOG_INFO("Main Loop Enter");
    NRF_LOG_FLUSH();
    application_timers_start();
    for ( ;; )
    {
        pmu_sys_voltage_monitor(NULL, 0);
        pmu_pwrok_pull(NULL, 0);
        pmu_irq_pull(NULL, 0);
        pmu_status_refresh(NULL, 0);
        pmu_req_process(NULL, 0);
        ble_ctl_process(NULL, 0);
        rsp_st_uart_cmd(NULL, 0);
        manage_bat_level(NULL, 0);
        led_ctl_process(NULL, 0);
        bat_msg_report_process(NULL, 0);
        // event exec
        app_sched_execute();
        // idle
        idle_state_handle();
    }
}

/**
 * @}
 */
