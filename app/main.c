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
#include "bsp_btn_ble.h"
#include "fds.h"
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
// #include "nfc.h"
#include "app_scheduler.h"
#include "ble_dfu.h"
#include "custom_board.h"
#include "data_transmission.h"
#include "ecdsa.h"
#include "fds_internal_defs.h"
#include "flashled_manage.h"
#include "nrf_bootloader_info.h"
#include "nrf_crypto_init.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_wdt.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_power.h"
#include "power_manage.h"
#include "rtc_calendar.h"
#include "nrf_crypto.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "firmware_config.h"

#define APDU_TAG_BLE 0x44

#define BLE_DEFAULT     0
#define BLE_CONNECT     1
#define BLE_DISCONNECT  2
#define BLE_DIS_PIN     3
#define BLE_PIN_ERR     4
#define BLE_PIN_TIMEOUT 5
#define BLE_PIN_CANCEL  7
#define BLE_RCV_DATA    8

#define DEFAULT_FLAG  0
#define SEND_SPI_DATA 1
#define READ_SPI_HEAD 2
#define READ_SPI_DATA 3

#define BLE_DEF        0
#define BLE_ON_ALWAYS  1
#define BLE_OFF_ALWAYS 2
#define BLE_DISCON     3
#define BLE_CON        4
#define BLE_PAIR       5

#define PWR_DEF          0
#define PWR_SHUTDOWN_SYS 1
#define PWR_CLOSE_EMMC   2
#define PWR_OPEN_EMMC    3
#define PWR_BAT_PERCENT  5
#define PWR_USB_STATUS   6

#define LED_DEF           0
#define LED_SET_BRIHTNESS 1
#define LED_GET_BRIHTNESS 2

#define BAT_DEF                0
#define SEND_BAT_VOL           1
#define SEND_BAT_CHARGE_CUR    2
#define SEND_BAT_DISCHARGE_CUR 3
#define SEND_BAT_INNER_TEMP    4

#define NO_CHARGE  0
#define USB_CHARGE 1
#define ERROR_STA  2

#define INIT_VALUE 0
#define AUTH_VALUE 1

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG  1 /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL 40 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION 0  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define ADV_ADDL_MANUF_DATA_LEN 6
#define COMPANY_IDENTIFIER      0xFE

// SCHEDULER CONFIGS
#define SCHED_MAX_EVENT_DATA_SIZE 64  //!< Maximum size of the scheduler event data.
#define SCHED_QUEUE_SIZE          20  //!< Size of the scheduler queue.

#define RCV_DATA_TIMEOUT_INTERVAL   APP_TIMER_TICKS(100)
#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(1000) /**< Battery level measurement interval (ticks). */
#define BATTERY_MEAS_LONG_INTERVAL  APP_TIMER_TICKS(5000)
#define MIN_BATTERY_LEVEL           81  /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL           100 /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT     1   /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Minimum acceptable connection interval (10 ms). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Maximum acceptable connection interval (100 ms) */
#define SLAVE_LATENCY                  0                               /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(300, UNIT_10_MS)  /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(100)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define ONE_SECOND_INTERVAL            APP_TIMER_TICKS(1000)

#define RST_ONE_SECNOD_COUNTER() one_second_counter = 0;
#define TWI_TIMEOUT_COUNTER      10

#define MAX_CONN_PARAM_UPDATE_COUNT 3 /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to  use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND            1                            /**< Perform bonding. */
#define SEC_PARAM_MITM            1                            /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_LESC            1                            /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS        0                            /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_DISPLAY_ONLY /**< Display I/O capabilities. */
#define SEC_PARAM_OOB             0                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                           /**< Maximum encryption key size. */

#define PASSKEY_LENGTH     6 /**< Length of pass-key received by the stack for display. */
#define HEAD_NAME_LENGTH   4
#define ADV_NAME_LENGTH    8
#define MAC_ADDRESS_LENGTH 6

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS \
  90  // 270=0.3v  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA.
      // Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT 1024  //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION \
  6  //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) ((((ADC_VALUE)*ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

#ifdef UART_TRANS
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
#define BLE_CMD_POWER_STA 0x08
#define BLE_INSERT_POWER  0x01
#define BLE_REMOVE_POWER  0x02
#define BLE_CHARGING_PWR  0x03
#define BLE_CHAGE_OVER    0x04
//
#define BLE_SYSTEM_POWER_PERCENT 0x09
//
#define BLE_CMD_KEY_STA     0x0A
#define BLE_KEY_LONG_PRESS  0x01
#define BLE_KEY_SHORT_PRESS 0x02

#define BLE_CMD_PWR_STA  0x0B
#define BLE_CLOSE_SYSTEM 0x01
#define BLE_CLOSE_EMMC   0x02
#define BLE_OPEN_EMMC    0x03
#define BLE_PWR_PERCENT  0X04

#define BLE_CMD_FLASH_LED_STA 0x0C
#define BLE_CMD_BAT_CV_MSG    0x0D

#define BLE_CMD_KEY_RESP     0x0E
#define BLE_KEY_RESP_SUCCESS 0x00
#define BLE_KEY_RESP_FAILED  0x01
#define BLE_KEY_RESP_PUBKEY  0x02
#define BLE_KEY_RESP_SIGN    0x03

#define BLE_CMD_BUILD_ID     0x10
#define BLE_CMD_HASH         0x11

// end BLE send CMD
//
#define UART_CMD_BLE_CON_STA  0x01
#define UART_CMD_BLE_PAIR_STA 0x02
#define UART_CMD_PAIR_CODE    0x03
#define UART_CMD_ADV_NAME     0x04
#define UART_CMD_BAT_PERCENT  0x05
#define UART_CMD_BLE_VERSION  0x06
#define UART_CMD_CTL_BLE      0x07

#define UART_CMD_DFU_STA 0x0a

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
//
#define ST_CMD_BLE_INFO       0x83
#define ST_REQ_ADV_NAME       0x01
#define ST_REQ_FIRMWARE_VER   0x02
#define ST_REQ_SOFTDEVICE_VER 0x03
#define ST_REQ_BOOTLOADER_VER 0x04
#define ST_REQ_BUILD_ID       0x05
#define ST_REQ_HASH           0x06

//
#define ST_CMD_RESET_BLE   0x84
#define ST_VALUE_RESET_BLE 0x01
//
#define ST_CMD_LED                 0X85
#define ST_SEND_SET_LED_BRIGHTNESS 0X01
#define ST_SEND_GET_LED_BRIGHTNESS 0X02
//
#define STM_CMD_BAT                0X86
#define STM_SEND_BAT_VOL           0X01  // Send battery voltage
#define STM_SEND_BAT_CHARGE_CUR    0X02  // Send battery charge current
#define STM_SEND_BAT_DISCHARGE_CUR 0X03  // Send battery discharge current
#define STM_SEND_BAT_INNER_TEMP    0X04  // Send battery inner temperature

#define STM_CMD_KEY      0x87
#define STM_GET_PUBKEY   0x01
#define STM_LOCK_PUBKEY  0x02
#define STM_REQUEST_SIGN 0x03

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
#define UART_CHANNEL 0x03

#define UART_DEF                  0x00
#define ACTIVE_SEND_UART          0x01
#define RESPONESE_NAME_UART       0x02
#define RESPONESE_BAT_UART        0x03
#define RESPONESE_VER_UART        0x04
#define RESPONESE_SD_VER_UART     0x05
#define RESPONESE_BOOT_VER_UART   0x06
#define RESPONESE_LED_VER_UART    0x07
#define RESPONESE_BLE_PUBKEY      0x08
#define RESPONESE_BLE_PUBKEY_LOCK 0x09
#define RESPONESE_BLE_SIGN        0x0A
#define RESPONESE_BUILD_ID        0x0B
#define RESPONESE_HASH            0x0C
#define DEF_RESP                  0xFF

#define BLE_CTL_ADDR         0x6f000
#define BAT_LVL_ADDR         0x70000
#define DEVICE_KEY_INFO_ADDR 0x71000
#endif

#define TIMER_INIT_FLAG  0
#define TIMER_RESET_FLAG 1
#define TIMER_START_FLAG 2
#define TIMER_STOP_FLAG  3

#define BLE_GAP_DATA_LENGTH_DEFAULT 27   //!< The stack's default data length.
#define BLE_GAP_DATA_LENGTH_MAX     251  //!< Maximum data length.

#define ST_WAKE_IO 22

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);  /**< Battery timer. */
APP_TIMER_DEF(m_100ms_timer_id);    /**< 100ms timer. */
APP_TIMER_DEF(m_1s_timer_id);
nrf_drv_wdt_channel_id m_channel_id;

static volatile uint8_t one_second_counter = 0;
static volatile uint8_t ble_evt_flag = BLE_DEFAULT;
uint8_t spi_evt_flag = DEFAULT_FLAG;
volatile uint8_t ble_adv_switch_flag = BLE_DEF;
static volatile uint8_t ble_conn_flag = BLE_DEF;
static volatile uint8_t ble_conn_nopair_flag = BLE_DEF;
static volatile uint8_t pwr_status_flag = PWR_DEF;
static volatile uint8_t trans_info_flag = UART_DEF;
static volatile uint8_t led_brightness_flag = LED_DEF;
static volatile uint8_t bat_msg_flag = BAT_DEF;
static volatile uint8_t ble_trans_timer_flag = TIMER_INIT_FLAG;
static uint8_t mac_ascii[12];
static uint8_t mac[6] = {0x42, 0x13, 0xc7, 0x98, 0x95, 0x1a};  // Device MAC address
static char ble_adv_name[ADV_NAME_LENGTH];

extern rtc_date_t rtc_date;

static volatile uint8_t bat_level_to_st = 0x00;

// add  tmp
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
        {BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}};

static void advertising_start(void);
#ifdef SCHED_ENABLE
static void twi_write_data(void *p_event_data, uint16_t event_size);
#endif
void forwarding_to_st_data(void);
#ifdef UART_TRANS
static volatile uint8_t flag_uart_trans = 1;
static uint8_t uart_trans_buff[128];
static uint8_t bak_buff[128];
static uint8_t uart_data_array[64];
static void uart_put_data(uint8_t *pdata, uint8_t lenth);
static void send_stm_data(uint8_t *pdata, uint8_t lenth);
static uint8_t calcXor(uint8_t *buf, uint8_t len);
#endif

static void advertising_start(void);
static void advertising_stop(void);
static void idle_state_handle(void);

/* Dummy data to write to flash. */
static uint32_t m_data2 = 0xBADC0FFE;
static uint32_t m_data = 0xABABABAB;
static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt);
static uint8_t bond_check_key_flag = INIT_VALUE;
static uint8_t rcv_head_flag = 0;
static uint8_t ble_status_flag = 0;
static bool ble_send_ready = false;

// AXP216 global status
static uint8_t g_charge_status = 0;
static uint8_t g_bas_update_flag = 0;
// static uint8_t g_offlevel_flag = 0;

// LM    global status
static uint8_t led_brightness_value = 0;

#ifdef SCHED_ENABLE
static ringbuffer_t m_ble_fifo;
#endif

#define STORAGE_TRUE_FLAG 0xa55aa55a

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) = {
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x6f000,
    .end_addr = 0x72000,
};

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get() {
  uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
  uint32_t const page_sz = NRF_FICR->CODEPAGESIZE;
  uint32_t const code_sz = NRF_FICR->CODESIZE;

  return (bootloader_addr != 0xFFFFFFFF ? bootloader_addr : (code_sz * page_sz));
}
void wait_for_flash_ready(nrf_fstorage_t const *p_fstorage) {
  /* While fstorage is busy, sleep and wait for an event. */
  while (nrf_fstorage_is_busy(p_fstorage)) {
    idle_state_handle();
  }
}

static void flash_data_write(uint32_t adress, uint32_t data) {
  ret_code_t rc;

  nrf_fstorage_erase(&fstorage, adress, FDS_PHY_PAGES_IN_VPAGE, NULL);

  rc = nrf_fstorage_write(&fstorage, adress, &data, sizeof(m_data), NULL);
  APP_ERROR_CHECK(rc);

  wait_for_flash_ready(&fstorage);
}

static void device_key_info_init(void) {
  ret_code_t rc;
  ecdsa_key_info_t key_info = {0};
  nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
  if (key_info.key_flag != STORAGE_TRUE_FLAG) {
    key_info.key_flag = STORAGE_TRUE_FLAG;
    generate_ecdsa_keypair(key_info.private_key, key_info.public_key);

    rc = nrf_fstorage_erase(&fstorage, DEVICE_KEY_INFO_ADDR, FDS_PHY_PAGES_IN_VPAGE, NULL);
    APP_ERROR_CHECK(rc);

    rc = nrf_fstorage_write(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
  }
}

#ifdef SCHED_ENABLE
static void create_ringBuffer(ringbuffer_t *ringBuf, uint8_t *buf, uint32_t buf_len) {
  ringBuf->br = 0;
  ringBuf->bw = 0;
  ringBuf->btoRead = 0;
  ringBuf->source = buf;
  ringBuf->length = buf_len;
}

static void clear_ringBuffer(ringbuffer_t *ringBuf) {
  ringBuf->br = 0;
  ringBuf->bw = 0;
  ringBuf->btoRead = 0;
  memset((uint8_t *)ringBuf->source, 0, ringBuf->length);
}

static uint32_t write_ringBuffer(uint8_t *buffer, uint32_t size, ringbuffer_t *ringBuf) {
  uint32_t len = 0;
  uint32_t ringBuf_bw = ringBuf->bw;
  uint32_t ringBuf_len = ringBuf->length;
  uint8_t *ringBuf_source = ringBuf->source;

  if ((ringBuf_bw + size) <= ringBuf_len) {
    memcpy(ringBuf_source + ringBuf_bw, buffer, size);
  } else {
    len = ringBuf_len - ringBuf_bw;
    memcpy(ringBuf_source + ringBuf_bw, buffer, len);
    memcpy(ringBuf_source, buffer + ringBuf_bw, size - len);
  }

  ringBuf->bw = (ringBuf->bw + size) % ringBuf_len;
  ringBuf->btoRead += size;

  return size;
}

static uint32_t read_ringBuffer(uint8_t *buffer, uint32_t size, ringbuffer_t *ringBuf) {
  uint32_t len = 0;
  uint32_t ringBuf_br = ringBuf->br;
  uint32_t ringBuf_len = ringBuf->length;
  uint8_t *ringBuf_source = ringBuf->source;

  if ((ringBuf_br + size) <= ringBuf_len) {
    memcpy(buffer, ringBuf_source + ringBuf_br, size);
  } else {
    len = ringBuf_len - ringBuf_br;
    memcpy(buffer, ringBuf_source + ringBuf_br, len);
    memcpy(buffer + len, ringBuf_source, size - len);
  }

  ringBuf->br = (ringBuf->br + size) % ringBuf_len;
  ringBuf->btoRead -= size;

  return size;
}

static uint32_t get_ringBuffer_btoRead(ringbuffer_t *ringBuf) { return ringBuf->btoRead; }

static uint32_t get_ringBuffer_length(ringbuffer_t *ringBuf) { return ringBuf->length; }
#endif

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
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  nrf_gpio_pin_sense_t sense;
  switch (event) {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
      NRF_LOG_INFO("Power management wants to reset to DFU mode.");
      // YOUR_JOB: Get ready to reset into DFU mode
      //
      // If you aren't finished with any ongoing tasks, return "false" to
      // signal to the system that reset is impossible at this stage.
      //
      // Here is an example using a variable to delay resetting the device.
      //
      // if (!m_ready_for_reset)
      // {
      //      return false;
      // }
      // else
      //{
      //
      //    // Device ready to enter
      //    uint32_t err_code;
      //    err_code = sd_softdevice_disable();
      //    APP_ERROR_CHECK(err_code);
      //    err_code = app_timer_stop_all();
      //    APP_ERROR_CHECK(err_code);
      //}
      break;
    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
      sense = NRF_GPIO_PIN_SENSE_HIGH;
      nrf_gpio_cfg_sense_set(POWER_IC_OK_IO, sense);
      break;

    default:
      // YOUR_JOB: Implement any of the other events available from the power management module:
      //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
      //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
      //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
      return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
  return true;
}

// lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context) {
  if (state == NRF_SDH_EVT_STATE_DISABLED) {
    // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

    // Go to system off.
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
  }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
    .handler = buttonless_dfu_sdh_state_observer,
};

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
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) { app_error_handler(DEAD_BEEF, line_num, p_file_name); } /**< Structure used to identify the battery service. */

static void gpio_uninit(void){
    nrf_drv_gpiote_uninit();
}

static void enter_low_power_mode(void) {
  axp_disable();
  gpio_uninit();
  nrf_gpio_cfg_input(AXP216_TWI_SDA_M, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(AXP216_TWI_SCL_M, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(ST_WAKE_IO, NRF_GPIO_PIN_NOPULL);
  app_uart_close();
  nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}
void battery_level_meas_timeout_handler(void *p_context) {
  ret_code_t err_code;
  static uint8_t battery_percent = 0;

  UNUSED_PARAMETER(p_context);
  if (battery_percent != bat_level_to_st) {
    battery_percent = bat_level_to_st;
    if (g_bas_update_flag == 1) {
      err_code = ble_bas_battery_level_update(&m_bas, battery_percent, BLE_CONN_HANDLE_ALL);
      if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
      }
    }
  }
}

static volatile uint8_t timeout_count = 0;
static volatile uint16_t timeout_longcnt = 0;

void m_100ms_timeout_hander(void *p_context) {
  UNUSED_PARAMETER(p_context);

  nrf_drv_wdt_channel_feed(m_channel_id);

  if (TIMER_RESET_FLAG == ble_trans_timer_flag) {
    timeout_longcnt = 0;
    timeout_count = 1;
    ble_trans_timer_flag = TIMER_INIT_FLAG;
    NRF_LOG_INFO("1-timer reset.");
  } else if (TIMER_START_FLAG == ble_trans_timer_flag) {
    timeout_count = 0;
    timeout_longcnt = 1;
    ble_trans_timer_flag = TIMER_INIT_FLAG;
    NRF_LOG_INFO("2-timer start.");
  }

  if (timeout_count >= 1) {
    timeout_count++;
    if (timeout_count >= 20) {
      NRF_LOG_INFO("1-timer timeout.");
      timeout_count = 0;
      ble_trans_timer_flag = TIMER_INIT_FLAG;
      rcv_head_flag = DATA_INIT;
    }
  }
  if (timeout_longcnt >= 1) {
    timeout_longcnt++;
    if (timeout_longcnt >= 580) {
      NRF_LOG_INFO("2-timer timeout.");
      timeout_longcnt = 0;
      ble_trans_timer_flag = TIMER_INIT_FLAG;
      rcv_head_flag = DATA_INIT;
    }
  }
}

void m_1s_timeout_hander(void *p_context) {
  UNUSED_PARAMETER(p_context);

  one_second_counter++;

  if ((one_second_counter % 5) == 0) {
    bat_level_to_st = get_battery_percent();
  }

  if (one_second_counter > 59) {
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
void on_bas_evt(ble_bas_t *p_bas, ble_bas_evt_t *p_evt) {
  switch (p_evt->evt_type) {
    case BLE_BAS_EVT_NOTIFICATION_ENABLED:
      g_bas_update_flag = 1;
      break;  // BLE_BAS_EVT_NOTIFICATION_ENABLED

    case BLE_BAS_EVT_NOTIFICATION_DISABLED:
      g_bas_update_flag = 0;
      break;  // BLE_BAS_EVT_NOTIFICATION_DISABLED

    default:
      // No implementation needed.
      break;
  }
}

// battery service init
void sys_bas_init(void) {
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
static void pm_evt_handler(pm_evt_t const *p_evt) {
  ret_code_t err_code;

  pm_handler_on_pm_evt(p_evt);
  pm_handler_disconnect_on_sec_failure(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
    case PM_EVT_CONN_SEC_SUCCEEDED: {
      pm_conn_sec_status_t conn_sec_status;

      // Check if the link is authenticated (meaning at least MITM).
      err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
      APP_ERROR_CHECK(err_code);

      if (conn_sec_status.mitm_protected) {
#ifdef UART_TRANS
        if (ble_conn_nopair_flag == BLE_PAIR) {
          ble_conn_nopair_flag = BLE_DEF;
          bak_buff[0] = BLE_CMD_PAIR_STA;
          bak_buff[1] = BLE_PAIR_SUCCESS;
          send_stm_data(bak_buff, 2);
        }
#endif
        nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_MAX);
        NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d", ble_conn_state_role(p_evt->conn_handle), p_evt->conn_handle, p_evt->params.conn_sec_succeeded.procedure);
      } else {
        // The peer did not use MITM, disconnect.
        NRF_LOG_INFO("Collector did not use MITM, disconnecting");
        err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
        APP_ERROR_CHECK(err_code);
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
      }
    } break;

    case PM_EVT_CONN_SEC_CONFIG_REQ: {
      pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
      pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    } break;  // PM_EVT_CONN_SEC_CONFIG_REQ

    case PM_EVT_CONN_SEC_FAILED:
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_PAIR_STA;
      bak_buff[1] = BLE_PAIR_FAIL;
      send_stm_data(bak_buff, 2);
#endif
      break;

    case PM_EVT_PEERS_DELETE_SUCCEEDED:
      advertising_start();
      break;

    default:
      break;
  }
}
#endif

void mac_address_get(void) {
  ble_gap_addr_t Mac_address;
  unsigned char i, j = 0;

  uint32_t err_code = sd_ble_gap_addr_get(&Mac_address);
  APP_ERROR_CHECK(err_code);

  memcpy(mac, Mac_address.addr, 6);
  for (i = 0; i < 6; i++) {
    if ((mac[i] >> 4) < 0x0a) {
      mac_ascii[j] = 0x30 + (mac[i] >> 4);
      j++;
    } else {
      mac_ascii[j] = 0x41 + (((mac[i] >> 4) & 0x0f) - 0x0A);  //(mac[i]>>4)&0x0f-0x0A
      j++;
    }

    if ((mac[i] & 0x0f) < 0x0a) {
      mac_ascii[j] = 0x30 + (mac[i] & 0x0f);
      j++;
    } else {
      mac_ascii[j] = 0x41 + ((mac[i] & 0x0f) - 0x0A);  // mac[i]&0x0f-0x0A
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
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;
#ifdef FIXED_PIN
  // set fixed Passkey
  ble_opt_t ble_opt;
  uint8_t g_ucBleTK[6] = "123456";
  ble_opt.gap_opt.passkey.p_passkey = g_ucBleTK;
#endif
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)ble_adv_name, strlen(ble_adv_name));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
#ifdef FIXED_PIN
  // set fixed Passkey
  err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
  APP_ERROR_CHECK(err_code);
#endif
}

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
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
static void nrf_qwr_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

static void disconnect(uint16_t conn_handle, void *p_context) {
  UNUSED_PARAMETER(p_context);

  ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
  } else {
    NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
  }
}

#ifdef BUTTONLESS_ENABLED
static void advertising_config_get(ble_adv_modes_config_t *p_config) {
  memset(p_config, 0, sizeof(ble_adv_modes_config_t));

  p_config->ble_adv_fast_enabled = true;
  p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
  p_config->ble_adv_fast_timeout = APP_ADV_DURATION;
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
#ifdef UART_TRANS
  bak_buff[0] = UART_CMD_DFU_STA;
  bak_buff[1] = 0x01;
#endif
  switch (event) {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE: {
      NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
#ifdef UART_TRANS
      bak_buff[2] = VALUE_PREPARE_DFU;
#endif
      // Prevent device from advertising on disconnect.
      ble_adv_modes_config_t config;
      advertising_config_get(&config);
      config.ble_adv_on_disconnect_disabled = true;
      ble_advertising_modes_config_set(&m_advertising, &config);

      // Disconnect all other bonded devices that currently are connected.
      // This is required to receive a service changed indication
      // on bootup after a successful (or aborted) Device Firmware Update.
      uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
      NRF_LOG_INFO("Disconnected %d links.", conn_count);
      break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
#ifdef UART_TRANS
      bak_buff[2] = VALUE_ENTER_DFU;
#endif
      // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
      //           by delaying reset by reporting false in app_shutdown_handler
      NRF_LOG_INFO("Device will enter bootloader mode.");
      break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
#ifdef UART_TRANS
      bak_buff[2] = VALUE_ENTER_FAILED;
#endif
      NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
      // YOUR_JOB: Take corrective measures to resolve the issue
      //           like calling APP_ERROR_CHECK to reset the device.
      break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
#ifdef UART_TRANS
      bak_buff[2] = VALUE_RSP_FAILED;
#endif
      NRF_LOG_ERROR("Request to send a response to client failed.");
      // YOUR_JOB: Take corrective measures to resolve the issue
      //           like calling APP_ERROR_CHECK to reset the device.
      APP_ERROR_CHECK(false);
      break;

    default:
#ifdef UART_TRANS
      bak_buff[2] = VALUE_UNKNOWN_ERR;
#endif
      NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
      break;
  }
}
#endif
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t *p_evt)
#ifdef SCHED_ENABLE
{
  static uint32_t msg_len;
  static bool reading = false;
  uint8_t *rcv_data = (uint8_t *)p_evt->params.rx_data.p_data;
  uint32_t rcv_len = p_evt->params.rx_data.length;

  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
    NRF_LOG_INFO("Received data from BLE NUS.");
    NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

    if (reading == false) {
      if (rcv_data[0] == '?' && rcv_data[1] == '#' && rcv_data[2] == '#') {
        data_recived_flag = false;
        if (rcv_len < 9) {
          return;
        } else {
          msg_len = (uint32_t)((rcv_data[5] << 24) + (rcv_data[6] << 16) + (rcv_data[7] << 8) + (rcv_data[8]));
          clear_ringBuffer(&m_ble_fifo);
          write_ringBuffer(rcv_data, rcv_len, &m_ble_fifo);
          if (msg_len > rcv_len) {
            reading = true;
          }
          ble_evt_flag = BLE_RCV_DATA;
        }
      }
    } else {
      if (rcv_len < msg_len) {
        reading = true;
        msg_len -= rcv_len;
        write_ringBuffer(rcv_data, rcv_len, &m_ble_fifo);
      } else {
        reading = false;
      }
      ble_evt_flag = BLE_RCV_DATA;
    }
  }
}
#else
{
  static uint32_t msg_len;
  uint32_t pad;
  // uint8_t *rcv_data=(uint8_t *)p_evt->params.rx_data.p_data;
  // uint32_t rcv_len=p_evt->params.rx_data.length;

  if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
    // NRF_LOG_INFO("Received data from BLE NUS.");
    // NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    data_recived_len = p_evt->params.rx_data.length;
    memcpy(data_recived_buf, (uint8_t *)p_evt->params.rx_data.p_data, data_recived_len);

    if (rcv_head_flag == DATA_INIT) {
      if (data_recived_buf[0] == '?' && data_recived_buf[1] == '#' && data_recived_buf[2] == '#') {
        data_recived_flag = false;
        if (data_recived_len < 9) {
          return;
        } else {
          // erase ST flash about 4 second
          if (data_recived_buf[3] == 0x00 && data_recived_buf[4] == 0x06) {
            ble_trans_timer_flag = TIMER_START_FLAG;
          } else {
            ble_trans_timer_flag = TIMER_RESET_FLAG;
          }
          msg_len = (uint32_t)((data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) + (data_recived_buf[7] << 8) + (data_recived_buf[8]));
          pad = ((data_recived_len + 63) / 64) + 8;
          if (msg_len > data_recived_len - pad) {
            msg_len -= data_recived_len - pad;
            rcv_head_flag = DATA_HEAD;
          }
          ble_evt_flag = BLE_RCV_DATA;
        }
      } else if (data_recived_buf[0] == 0x5A && data_recived_buf[1] == 0xA5 && data_recived_buf[2] == 0x07 && data_recived_buf[3] == 0x1 && data_recived_buf[4] == 0x03) {
        ble_adv_switch_flag = BLE_OFF_ALWAYS;
        return;
      }
    } else {
      if (data_recived_buf[0] == '?') {
        pad = (data_recived_len + 63) / 64;
        if (data_recived_len - pad > msg_len) {
          rcv_head_flag = DATA_INIT;
          data_recived_len = msg_len + (msg_len + 62) / 63;
          msg_len = 0;
        } else {
          msg_len -= data_recived_len - pad;
        }
        ble_evt_flag = BLE_RCV_DATA;
        ble_trans_timer_flag = TIMER_RESET_FLAG;
      } else {
        rcv_head_flag = DATA_INIT;
      }
    }
    forwarding_to_st_data();
  }
}
#endif

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
static void services_init(void) {
  ret_code_t err_code;
  ble_dis_init_t dis_init;
  ble_nus_init_t nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};
#ifdef BUTTONLESS_ENABLED
  ble_dfu_buttonless_init_t dfus_init = {0};
#endif

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);
#ifdef BUTTONLESS_ENABLED
  dfus_init.evt_handler = ble_dfu_evt_handler;

  err_code = ble_dfu_buttonless_init(&dfus_init);
  APP_ERROR_CHECK(err_code);
#endif
  // Initialize Battery Service.
  sys_bas_init();

#if BLE_DIS_ENABLED
  // Initialize Device Information Service.
  memset(&dis_init, 0, sizeof(dis_init));

  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
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
static void timers_init(void) {
  ret_code_t err_code;

  // Initialize timer module.
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // Create timers.
  err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_1s_timer_id, APP_TIMER_MODE_REPEATED, m_1s_timeout_hander);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_100ms_timer_id, APP_TIMER_MODE_REPEATED, m_100ms_timeout_hander);
  APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting application timers.
 */
static void application_timers_start(void) {
  ret_code_t err_code;

  // Start application timers.
  err_code = app_timer_start(m_1s_timer_id, ONE_SECOND_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

  // Start battery timer
  err_code = app_timer_start(m_battery_timer_id, BATTERY_MEAS_LONG_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

  // Start 100ms timer
  err_code = app_timer_start(m_100ms_timer_id, RCV_DATA_TIMEOUT_INTERVAL, NULL);
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
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
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
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
#ifdef DEV_BSP
  ret_code_t err_code;
#endif
  switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
      NRF_LOG_INFO("Fast advertising");
#ifdef DEV_BSP
      err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      APP_ERROR_CHECK(err_code);
#endif
      break;  // BLE_ADV_EVT_FAST

    case BLE_ADV_EVT_IDLE:  //协议栈上抛这个事件就回进入睡眠模式
      break;                // BLE_ADV_EVT_IDLE

    default:
      break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
#ifdef BOND_ENABLE
  ret_code_t err_code;

  pm_handler_secure_on_connection(p_ble_evt);

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_DISCONNECTED: {
      NRF_LOG_INFO("Disconnected");
      ble_evt_flag = BLE_DISCONNECT;
      bond_check_key_flag = INIT_VALUE;
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_DISCON_STATUS;
      send_stm_data(bak_buff, 2);
#endif
      // Check if the last connected peer had not used MITM, if so, delete its bond information.
      if (m_peer_to_be_deleted != PM_PEER_ID_INVALID) {
        err_code = pm_peer_delete(m_peer_to_be_deleted);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("Collector's bond deleted");
        m_peer_to_be_deleted = PM_PEER_ID_INVALID;
      }
    } break;

    case BLE_GAP_EVT_CONNECTED: {
      NRF_LOG_INFO("Connected");
      ble_evt_flag = BLE_CONNECT;
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_CON_STATUS;
      send_stm_data(bak_buff, 2);
#endif
      m_peer_to_be_deleted = PM_PEER_ID_INVALID;
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);
      nrf_ble_gatt_data_length_set(&m_gatt, m_conn_handle, BLE_GAP_DATA_LENGTH_DEFAULT);
      // Start Security Request timer.
    } break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys = {
          .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
      break;

    case BLE_GAP_EVT_PASSKEY_DISPLAY: {
      char passkey[PASSKEY_LENGTH + 1];
      memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
      passkey[PASSKEY_LENGTH] = 0;
#ifdef UART_TRANS
      ble_conn_nopair_flag = BLE_PAIR;
      bak_buff[0] = BLE_CMD_PAIR_CODE;
      memcpy(&bak_buff[1], passkey, PASSKEY_LENGTH);
      send_stm_data(bak_buff, 1 + PASSKEY_LENGTH);
#endif
      NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
    } break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
      NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
      break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
      NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
      break;

    case BLE_GAP_EVT_AUTH_STATUS:
      NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x", p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                   p_ble_evt->evt.gap_evt.params.auth_status.bonded, p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4, *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                   *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
      bond_check_key_flag = AUTH_VALUE;
      break;

    default:
      // No implementation needed.
      break;
  }
#else
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id) {
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

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys = {
          .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

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
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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
static void ble_stack_init(void) {
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
static void peer_manager_init(void) {
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

#ifdef UART_TRANS
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t *p_event) {
  static uint8_t index = 0;
  static uint32_t lenth = 0;
  uint8_t uart_xor_byte;

  switch (p_event->evt_type) {
    case APP_UART_DATA_READY:
      UNUSED_VARIABLE(app_uart_get(&uart_data_array[index]));
      index++;
      // NRF_LOG_INFO("receive uart data.")
      if (1 == index) {
        if (UART_TX_TAG != uart_data_array[0]) {
          index = 0;
          return;
        }
      } else if (2 == index) {
        if (UART_TX_TAG2 != uart_data_array[1]) {
          index = 0;
          return;
        }
      } else if (3 == index) {
        if ((UART_TX_TAG2 == uart_data_array[0]) && (UART_TX_TAG == uart_data_array[1])) {
          index = 0;
          return;
        }
      } else if (4 == index) {
        lenth = ((uint32_t)uart_data_array[2] << 8) + uart_data_array[3];
      } else if (index >= lenth + 4) {
        uart_xor_byte = calcXor(uart_data_array, index - 1);
        if (uart_xor_byte != uart_data_array[index - 1]) {
          index = 0;
          return;
        }

        switch (uart_data_array[4]) {
          case ST_CMD_BLE:
            switch (uart_data_array[5]) {
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
            switch (uart_data_array[5]) {
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
              default:
                pwr_status_flag = PWR_DEF;
                break;
            }
            break;
          case ST_CMD_BLE_INFO:
            switch (uart_data_array[5]) {
              case ST_REQ_ADV_NAME:
                trans_info_flag = RESPONESE_NAME_UART;
                break;
              case ST_REQ_FIRMWARE_VER:
                trans_info_flag = RESPONESE_VER_UART;
                break;
              case ST_REQ_SOFTDEVICE_VER:
                trans_info_flag = RESPONESE_SD_VER_UART;
                break;
              case ST_REQ_BOOTLOADER_VER:
                trans_info_flag = RESPONESE_BOOT_VER_UART;
                break;
              case ST_REQ_BUILD_ID:
                trans_info_flag = RESPONESE_BUILD_ID;
                break;
              case ST_REQ_HASH:
                trans_info_flag = RESPONESE_HASH;
                break;
              default:
                trans_info_flag = UART_DEF;
                break;
            }
            break;
          case ST_CMD_RESET_BLE:
            if (ST_VALUE_RESET_BLE == uart_data_array[5]) {
              NVIC_SystemReset();
            }
            break;
          case ST_CMD_LED:
            if (ST_SEND_GET_LED_BRIGHTNESS == uart_data_array[5]) {
              led_brightness_flag = LED_GET_BRIHTNESS;
            } else if (ST_SEND_SET_LED_BRIGHTNESS == uart_data_array[5]) {
              led_brightness_flag = LED_SET_BRIHTNESS;
              led_brightness_value = uart_data_array[6];
            }
            break;

          case STM_CMD_BAT:
            switch (uart_data_array[5]) {
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
            switch ((uart_data_array[5])) {
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
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void usr_uart_init(void) {
  uint32_t err_code;
  app_uart_comm_params_t const comm_params = {
    .rx_pin_no = RX_PIN_NUMBER,
    .tx_pin_no = TX_PIN_NUMBER,
    .rts_pin_no = RTS_PIN_NUMBER,
    .cts_pin_no = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .use_parity = false,
#if defined(UART_PRESENT)
    .baud_rate = NRF_UART_BAUDRATE_115200
#else
    .baud_rate = NRF_UARTE_BAUDRATE_115200
#endif
  };

  APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code);
  APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void) {
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

uint32_t get_rtc_counter(void) { return NRF_RTC1->COUNTER; }
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}
#ifdef SCHED_ENABLE

#else
void forwarding_to_st_data(void) {
  uint8_t send_spi_offset = 0;

  if (BLE_RCV_DATA == ble_evt_flag) {
    if (data_recived_len != 0) {
      while (data_recived_len >= 64) {
        usr_spi_write(data_recived_buf + send_spi_offset, 64);
        send_spi_offset += 64;
        data_recived_len -= 64;
      }
      if (data_recived_len) {
        usr_spi_write(data_recived_buf + send_spi_offset, 64);
        data_recived_len = 0;
        send_spi_offset = 0;
      }
    } else {
      usr_spi_write(data_recived_buf, 64);
    }

    spi_evt_flag = SEND_SPI_DATA;
    RST_ONE_SECNOD_COUNTER();
    NRF_LOG_HEXDUMP_INST_INFO("recv data", data_recived_buf, data_recived_len);
  }
}
#endif
static void ble_resp_data(void *p_event_data, uint16_t event_size) {
  ret_code_t err_code;
  uint16_t length = 0;
  uint16_t offset = 0;

  if (!ble_send_ready) return;
  ble_send_ready = false;

  while (data_recived_len > m_ble_nus_max_data_len) {
    length = m_ble_nus_max_data_len;
    do {
      err_code = ble_nus_data_send(&m_nus, data_recived_buf + offset, &length, m_conn_handle);
      if (err_code == NRF_ERROR_INVALID_STATE) {
        data_recived_len = 0;
        return;
      }
      if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND)) {
        APP_ERROR_CHECK(err_code);
      }
      if (err_code == NRF_ERROR_INVALID_STATE) {
        return;
      }
      if (err_code == NRF_SUCCESS) {
        data_recived_len -= m_ble_nus_max_data_len;
        offset += m_ble_nus_max_data_len;
      }
    } while (err_code == NRF_ERROR_RESOURCES);
  }

  if (data_recived_len) {
    length = data_recived_len;
    do {
      err_code = ble_nus_data_send(&m_nus, data_recived_buf + offset, &length, m_conn_handle);
      if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND)) {
        APP_ERROR_CHECK(err_code);
      }
    } while (err_code == NRF_ERROR_RESOURCES);
    data_recived_len = 0;
    length = 0;
  }
}
static void phone_resp_data(void) {
  read_st_resp_data();
  spi_evt_flag = READ_SPI_HEAD;

  if (false == data_recived_flag) return;

  data_recived_flag = false;
  spi_evt_flag = READ_SPI_DATA;

  // response data
  // ble_resp_data();
  spi_evt_flag = DEFAULT_FLAG;
  ble_send_ready = true;
  RST_ONE_SECNOD_COUNTER();
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  ret_code_t err_code;

  if ((BLE_DEF == ble_adv_switch_flag) || (BLE_ON_ALWAYS == ble_adv_switch_flag)) {
    if (bond_check_key_flag != AUTH_VALUE) {
      err_code = nrf_ble_lesc_request_handler();
      APP_ERROR_CHECK(err_code);
    }
  }
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

void in_gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  static uint8_t last_charge_status;
  switch (pin) {
    case SLAVE_SPI_RSP_IO:
      if (spi_dir_out) {
        spi_send_done = true;
      } else {
        phone_resp_data();
      }
      break;
    case POWER_IC_OK_IO:
      if (action == NRF_GPIOTE_POLARITY_LOTOHI) {
        // open_all_power();
      } else if (action == NRF_GPIOTE_POLARITY_HITOLO) {
        NRF_LOG_INFO("SET OFF LEVEL");
        enter_low_power_mode();
      }
      break;
    case POWER_IC_IRQ_IO:

      g_charge_status = get_charge_status();

      if (last_charge_status != g_charge_status) {
        last_charge_status = g_charge_status;
        bak_buff[0] = BLE_CMD_POWER_STA;
        bak_buff[1] = g_charge_status;
        bak_buff[2] = get_charge_type();
        if (get_charge_type() == AXP_CHARGE_TYPE_WIRELESS) {
           axp_update(AXP_CHARGE_CONTROL1, 0, 0x0F);//wireless charging
        } else {
           axp_update(AXP_CHARGE_CONTROL1, 0x01, 0x0F);//usb charging  
        }
        send_stm_data(bak_buff, 3);
      }
      // AXP interrupt processing logic
      uint8_t irq_req_value = get_irq_status();
      if ((irq_req_value & IRQ_SHORT_PRESS) == IRQ_SHORT_PRESS) {  // SHORT (Short press trigger)
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x01;
        send_stm_data(bak_buff, 2);
      }
      if ((irq_req_value & IRQ_LONG_PRESS) == IRQ_LONG_PRESS) {  // LONG (Long  press trigger)
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x02;
        send_stm_data(bak_buff, 2);
      }
      if ((irq_req_value & IRQ_FALLING_EDGE) == IRQ_FALLING_EDGE) {  // FALLING (Press trigger)
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x20;
        send_stm_data(bak_buff, 2);
      }
      if ((irq_req_value & IRQ_RISING_EDGE) == IRQ_RISING_EDGE) {  // RISING (Release trigger)
        bak_buff[0] = BLE_CMD_KEY_STA;
        bak_buff[1] = 0x40;
        send_stm_data(bak_buff, 2);
      }
      clear_irq_reg();  // clear irq
      break;

    default:
      break;
  }
}

static void gpiote_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  // nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  // in_config.pull = NRF_GPIO_PIN_PULLUP;

  nrf_drv_gpiote_in_config_t in_config1 = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config1.pull = NRF_GPIO_PIN_PULLUP;
  err_code = nrf_drv_gpiote_in_init(SLAVE_SPI_RSP_IO, &in_config1, in_gpiote_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(SLAVE_SPI_RSP_IO, true);

  err_code = nrf_drv_gpiote_in_init(POWER_IC_OK_IO, &in_config1, in_gpiote_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(POWER_IC_OK_IO, true);

  err_code = nrf_drv_gpiote_in_init(POWER_IC_IRQ_IO, &in_config1, in_gpiote_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(POWER_IC_IRQ_IO, true);
}

static void gpio_init(void) { gpiote_init(); }
#ifdef UART_TRANS
static uint8_t calcXor(uint8_t *buf, uint8_t len) {
  uint8_t tmp = 0;
  uint8_t i;

  for (i = 0; i < len; i++) {
    tmp ^= buf[i];
  }
  return tmp;
}
static void uart_put_data(uint8_t *pdata, uint8_t lenth) {
  uint32_t err_code;

  while (lenth--) {
    do {
      err_code = app_uart_put(*pdata++);
      if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY)) {
        APP_ERROR_CHECK(err_code);
      }
    } while (err_code == NRF_ERROR_BUSY);
  }
}

static void send_stm_data(uint8_t *pdata, uint8_t lenth) {
  uart_trans_buff[0] = UART_TX_TAG2;
  uart_trans_buff[1] = UART_TX_TAG;
  uart_trans_buff[2] = 0x00;
  uart_trans_buff[3] = lenth + 1;
  memcpy(&uart_trans_buff[4], pdata, lenth);
  uart_trans_buff[uart_trans_buff[3] + 3] = calcXor(uart_trans_buff, (uart_trans_buff[3] + 3));

  uart_put_data(uart_trans_buff, uart_trans_buff[3] + 4);
}
#endif

static void system_init() {
#ifdef SCHED_ENABLE
  create_ringBuffer(&m_ble_fifo, data_recived_buf, sizeof(data_recived_buf));
#endif
  usr_rtc_init();
  usr_spim_init();
  ret_code_t err_code = usr_power_init();
  APP_ERROR_CHECK(err_code);
#ifdef UART_TRANS
  usr_uart_init();
  NRF_LOG_INFO("uart init ok ......");
#endif
  gpio_init();
  set_led_brightness(0);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void) {
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

  APP_ERROR_CHECK(err_code);
}
static void advertising_stop(void) {
  ret_code_t err_code = ble_advertising_stop(&m_advertising);

  APP_ERROR_CHECK(err_code);
}
#ifdef DEV_BSP
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
  //    uint32_t err_code;
  switch (event) {
    case BSP_EVENT_SLEEP:
      break;

    case BSP_EVENT_DISCONNECT:

      break;

    case BSP_EVENT_WHITELIST_OFF:

      break;

    default:
      break;
  }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void) {
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);
}
#endif
static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt) {
  if (p_evt->result != NRF_SUCCESS) {
    NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
    return;
  }

  switch (p_evt->id) {
    case NRF_FSTORAGE_EVT_WRITE_RESULT: {
      NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.", p_evt->len, p_evt->addr);
    } break;

    case NRF_FSTORAGE_EVT_ERASE_RESULT: {
      NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.", p_evt->len, p_evt->addr);
    } break;

    default:
      break;
  }
}

static void fs_init(void) {
  ret_code_t rc;

  nrf_fstorage_api_t *p_fs_api;

  p_fs_api = &nrf_fstorage_sd;
  rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  APP_ERROR_CHECK(rc);

  /* It is possible to set the start and end addresses of an fstorage instance at runtime.
   * They can be set multiple times, should it be needed. The helper function below can
   * be used to determine the last address on the last page of flash memory available to
   * store data. */
  (void)nrf5_flash_end_addr_get();

  // flash_data_write(BLE_CTL_FLAG,m_data);
}
static void ctl_advertising(void) {
  uint32_t len = 4;
  uint8_t data[4];

  nrf_fstorage_read(&fstorage, BLE_CTL_ADDR, data, len);
  if (((0xFF == data[0]) && (0xFF == data[1]) && (0xFF == data[2]) && (0xFF == data[3])) || ((0xFE == data[0]) && (0x0F == data[1]) && (0xDC == data[2]) && (0xBA == data[3]))) {
    ble_status_flag = BLE_ON_ALWAYS;
    advertising_start();
    NRF_LOG_INFO("1-Start adv.\n");
  } else if ((0xAB == data[0]) && (0xAB == data[1]) && (0xAB == data[2]) && (0xAB == data[3])) {
    ble_status_flag = BLE_OFF_ALWAYS;
    NRF_LOG_INFO("2-No Adv.\n");
  } else {
    ble_status_flag = BLE_ON_ALWAYS;
    advertising_start();
    NRF_LOG_INFO("3-Start adv.\n");
  }
}

static void rsp_st_uart_cmd(void *p_event_data, uint16_t event_size) {
  memset(bak_buff, 0x00, sizeof(bak_buff));
  if (RESPONESE_NAME_UART == trans_info_flag) {
    bak_buff[0] = BLE_CMD_ADV_NAME;
    memcpy(&bak_buff[1], (uint8_t *)ble_adv_name, ADV_NAME_LENGTH);
    send_stm_data(bak_buff, 1 + ADV_NAME_LENGTH);
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_VER_UART) {
    bak_buff[0] = BLE_FIRMWARE_VER;
    memcpy(&bak_buff[1], FW_REVISION, sizeof(FW_REVISION) - 1);
    send_stm_data(bak_buff, sizeof(FW_REVISION));
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_SD_VER_UART) {
    bak_buff[0] = BLE_SOFTDEVICE_VER;
    memcpy(&bak_buff[1], SW_REVISION, sizeof(SW_REVISION) - 1);
    send_stm_data(bak_buff, sizeof(SW_REVISION));
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_BOOT_VER_UART) {
    bak_buff[0] = BLE_BOOTLOADER_VER;
    memcpy(&bak_buff[1], BT_REVISION, sizeof(BT_REVISION) - 1);
    send_stm_data(bak_buff, sizeof(BT_REVISION));
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_BLE_PUBKEY) {
    bak_buff[0] = BLE_CMD_KEY_RESP;

    ecdsa_key_info_t key_info = {0};
    nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
    if (key_info.key_lock_flag == STORAGE_TRUE_FLAG) {
      bak_buff[1] = BLE_KEY_RESP_FAILED;
      send_stm_data(bak_buff, 2);
    } else if (key_info.key_flag != STORAGE_TRUE_FLAG) {
      bak_buff[1] = BLE_KEY_RESP_FAILED;
      send_stm_data(bak_buff, 2);
    } else {
      bak_buff[1] = BLE_KEY_RESP_PUBKEY;
      memcpy(&bak_buff[2], key_info.public_key, sizeof(key_info.public_key));
      send_stm_data(bak_buff, sizeof(key_info.public_key) + 2);
    }

    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_BLE_PUBKEY_LOCK) {
    ecdsa_key_info_t key_info = {0};
    nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
    if (key_info.key_lock_flag != STORAGE_TRUE_FLAG) {
      ret_code_t rc;
      key_info.key_lock_flag = STORAGE_TRUE_FLAG;
      rc = nrf_fstorage_erase(&fstorage, DEVICE_KEY_INFO_ADDR, FDS_PHY_PAGES_IN_VPAGE, NULL);
      APP_ERROR_CHECK(rc);

      rc = nrf_fstorage_write(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t), NULL);
      APP_ERROR_CHECK(rc);

      wait_for_flash_ready(&fstorage);
    }
    bak_buff[0] = BLE_CMD_KEY_RESP;
    bak_buff[1] = BLE_KEY_RESP_SUCCESS;
    send_stm_data(bak_buff, 2);
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_BLE_SIGN) {
    uint32_t msg_len = (uart_data_array[2] << 8 | uart_data_array[3]) - 3;
    bak_buff[0] = BLE_CMD_KEY_RESP;

    ecdsa_key_info_t key_info = {0};
    nrf_fstorage_read(&fstorage, DEVICE_KEY_INFO_ADDR, &key_info, sizeof(ecdsa_key_info_t));
    if (key_info.key_flag != STORAGE_TRUE_FLAG) {
      bak_buff[1] = BLE_KEY_RESP_FAILED;
      send_stm_data(bak_buff, 2);
    } else {
      bak_buff[1] = BLE_KEY_RESP_SIGN;
      sign_ecdsa_msg(key_info.private_key, uart_data_array + 6, msg_len, bak_buff + 2);
      send_stm_data(bak_buff, 64 + 2);
    }
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_BUILD_ID) {
    bak_buff[0] = BLE_CMD_BUILD_ID;
    memcpy(&bak_buff[1], (uint8_t *)BUILD_ID, 7);
    send_stm_data(bak_buff,8);
    trans_info_flag = DEF_RESP;
  } else if (trans_info_flag == RESPONESE_HASH) {
    ret_code_t  err_code = NRF_SUCCESS;
    nrf_crypto_backend_hash_context_t hash_context = {0};
    uint8_t hash[32]={0};
    size_t hash_len = 32;
    int chunks = 0;
    int app_size = 0;
    uint8_t *code_addr = (uint8_t *)0x26000;
    uint8_t *code_len = (uint8_t *)0x7F018;
    app_size = code_len[0] + code_len[1]*256 + code_len[2]*256*256;
    chunks = app_size/512;

    err_code = nrf_crypto_hash_init(&hash_context, &g_nrf_crypto_hash_sha256_info);
    APP_ERROR_CHECK(err_code);
    for(int i = 0; i<chunks; i++) {
      err_code = nrf_crypto_hash_update(&hash_context, code_addr + i*512, 512);
      APP_ERROR_CHECK(err_code);
    }
    if(app_size%512) {
      err_code = nrf_crypto_hash_update(&hash_context, code_addr + chunks*512, app_size%512);
      APP_ERROR_CHECK(err_code);
    }
    err_code = nrf_crypto_hash_finalize(&hash_context, hash, &hash_len);
    APP_ERROR_CHECK(err_code);

    bak_buff[0] = BLE_CMD_HASH;
    memcpy(&bak_buff[1], hash, 32);
    send_stm_data(bak_buff,33);
    trans_info_flag = DEF_RESP;
  }
}
static void manage_bat_level(void *p_event_data, uint16_t event_size) {
  static uint8_t bak_bat_persent = 0x00;

  if (bak_bat_persent != bat_level_to_st) {
    bak_bat_persent = bat_level_to_st;
    bak_buff[0] = BLE_SYSTEM_POWER_PERCENT;
    bak_buff[1] = bat_level_to_st;
    send_stm_data(bak_buff, 2);
  }
}
static void check_advertising_stop(void) {
  if (ble_evt_flag != BLE_DISCONNECT || ble_evt_flag != BLE_DEFAULT) {
    sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    ble_evt_flag = BLE_DISCONNECT;
    NRF_LOG_INFO("Ctl disconnect.");
  }
  nrf_delay_ms(1000);
  advertising_stop();
}
static void ble_ctl_process(void *p_event_data, uint16_t event_size) {
  uint8_t respons_flag = 0;

  if (BLE_OFF_ALWAYS == ble_adv_switch_flag) {
    ble_adv_switch_flag = BLE_DEF;
    if (BLE_ON_ALWAYS == ble_status_flag) {
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_ADV_OFF_STATUS;
      send_stm_data(bak_buff, 2);
#endif
      flash_data_write(BLE_CTL_ADDR, m_data);
      ble_status_flag = BLE_OFF_ALWAYS;
      NRF_LOG_INFO("1-Ble disconnect.\n");
      NRF_LOG_INFO("BLE status is %d", ble_evt_flag);

      check_advertising_stop();
    } else {
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_ADV_OFF_STATUS;
      send_stm_data(bak_buff, 2);
#endif
    }
  } else if (BLE_ON_ALWAYS == ble_adv_switch_flag) {
    ble_adv_switch_flag = BLE_DEF;
    if (BLE_OFF_ALWAYS == ble_status_flag) {
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_ADV_ON_STATUS;
      send_stm_data(bak_buff, 2);
#endif
      advertising_start();
      flash_data_write(BLE_CTL_ADDR, m_data2);
      ble_status_flag = BLE_ON_ALWAYS;
      NRF_LOG_INFO("2-Start advertisement.\n");
    } else {
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_CON_STA;
      bak_buff[1] = BLE_ADV_ON_STATUS;
      send_stm_data(bak_buff, 2);
#endif
    }
  }
  if (BLE_DISCON == ble_conn_flag) {
    ble_conn_flag = BLE_DEF;
    if (ble_evt_flag != BLE_DISCONNECT || ble_evt_flag != BLE_DEFAULT) {
      sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      ble_evt_flag = BLE_DISCONNECT;

      NRF_LOG_INFO("Ctl disconnect.");
    }
#ifdef UART_TRANS
    bak_buff[0] = BLE_CMD_CON_STA;
    bak_buff[1] = BLE_DISCON_STATUS;
    send_stm_data(bak_buff, 2);
#endif
  }
  if (BLE_CON == ble_conn_flag) {
    ble_conn_flag = BLE_DEF;
    bak_buff[0] = BLE_CMD_CON_STA;
    bak_buff[1] = ble_status_flag + 2;
    send_stm_data(bak_buff, 2);
  }
  //
  switch (pwr_status_flag) {
    case PWR_SHUTDOWN_SYS:
      pwr_status_flag = PWR_DEF;
#ifdef UART_TRANS
      bak_buff[0] = BLE_CMD_PWR_STA;
      bak_buff[1] = BLE_CLOSE_SYSTEM;
      send_stm_data(bak_buff, 2);
#endif
      if (ble_status_flag != BLE_OFF_ALWAYS) {
        check_advertising_stop();
      }
      close_all_power();
      break;
    case PWR_CLOSE_EMMC:
      respons_flag = BLE_CLOSE_EMMC;
      ctl_emmc_power(AXP_CLOSE_EMMC);
      break;
    case PWR_OPEN_EMMC:
      respons_flag = BLE_OPEN_EMMC;
      ctl_emmc_power(AXP_OPEN_EMMC);
      break;
    case PWR_BAT_PERCENT:
      pwr_status_flag = PWR_DEF;
      bak_buff[0] = BLE_SYSTEM_POWER_PERCENT;
      bak_buff[1] = get_battery_percent();
      send_stm_data(bak_buff, 2);
      break;
    case PWR_USB_STATUS:
      bak_buff[0] = BLE_CMD_POWER_STA;
      bak_buff[1] = get_charge_status();
      bak_buff[2] = get_charge_type();
      if (get_charge_type() == AXP_CHARGE_TYPE_WIRELESS) {
          axp_update(AXP_CHARGE_CONTROL1, 0, 0x0F);      //wireless charging
      } else {
          axp_update(AXP_CHARGE_CONTROL1, 0x01, 0x0F);   //usb charging        
      }
       send_stm_data(bak_buff, 3);
      pwr_status_flag = PWR_DEF;
      break;
    default:
      break;
  }
  if ((PWR_DEF != pwr_status_flag) && (respons_flag != 0x00)) {
#ifdef UART_TRANS
    bak_buff[0] = BLE_CMD_PWR_STA;
    bak_buff[1] = respons_flag;
    send_stm_data(bak_buff, 2);
#endif
    pwr_status_flag = PWR_DEF;
  }
}

static void led_ctl_process(void *p_event_data, uint16_t event_size) {
  if (ST_SEND_SET_LED_BRIGHTNESS == led_brightness_flag) {
    set_led_brightness(led_brightness_value);
#ifdef UART_TRANS
    bak_buff[0] = BLE_CMD_FLASH_LED_STA;
    bak_buff[1] = led_brightness_flag;
    bak_buff[2] = led_brightness_value;
    send_stm_data(bak_buff, 3);
#endif

    led_brightness_flag = LED_DEF;

  } else if (ST_SEND_GET_LED_BRIGHTNESS == led_brightness_flag) {
    uint8_t current_led_brihtness = 0;
    current_led_brihtness = get_led_brightness();
#ifdef UART_TRANS
    bak_buff[0] = BLE_CMD_FLASH_LED_STA;
    bak_buff[1] = led_brightness_flag;
    bak_buff[2] = current_led_brihtness;
    send_stm_data(bak_buff, 3);
#endif

    led_brightness_flag = LED_DEF;
  }
}

static void bat_msg_report_process(void *p_event_data, uint16_t event_size) {
  uint8_t axp_reg = 0;
  switch (bat_msg_flag) {
    case SEND_BAT_VOL:
      axp_reg = AXP_VBATH_RES;
      break;
    case SEND_BAT_CHARGE_CUR:
      axp_reg = AXP_CCBATH_RES;
      break;
    case SEND_BAT_DISCHARGE_CUR:
      axp_reg = AXP_DCBATH_RES;
      break;
    case SEND_BAT_INNER_TEMP:
      axp_reg = AXP_INTTEMP;
      break;
    default:
      return;
  }
  uint8_t bat_values[2] = {0};
  //获取电池对应信息
  get_battery_cv_msg(axp_reg, bat_values);

#ifdef UART_TRANS
  bak_buff[0] = BLE_CMD_BAT_CV_MSG;
  bak_buff[1] = bat_msg_flag;
  bak_buff[2] = (bat_values[0]&0xF0) >> 4;
  bak_buff[3] = (bat_values[0]&0x0F) << 4 | (bat_values[1]&0x0F);
  send_stm_data(bak_buff, 4);
#endif
  bat_msg_flag = BAT_DEF;
}

static void scheduler_init(void) { APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE); }
static void main_loop(void) {
  app_sched_event_put(NULL, 0, ble_ctl_process);
  app_sched_event_put(NULL, 0, rsp_st_uart_cmd);
  app_sched_event_put(NULL, 0, manage_bat_level);
  // app_sched_event_put(NULL,NULL,nfc_poll);
  app_sched_event_put(NULL, 0, ble_resp_data);
  app_sched_event_put(NULL, 0, led_ctl_process);
  app_sched_event_put(NULL, 0, bat_msg_report_process);
}

static void watch_dog_init(void) {
  uint32_t err_code = NRF_SUCCESS;
  // Configure WDT.
  nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
  err_code = nrf_drv_wdt_init(&config, NULL);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  nrf_drv_wdt_enable();
}

int main(void) {
#ifdef BUTTONLESS_ENABLED
  // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
  ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
  APP_ERROR_CHECK(err_code);
#endif
  // Initialize.
  log_init();
  system_init();
  scheduler_init();

  fs_init();
  nrf_crypto_init();
  device_key_info_init();

  timers_init();
#ifdef DEV_BSP
  buttons_leds_init();
#endif
  power_management_init();
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
  application_timers_start();
  // Start execution.
  NRF_LOG_INFO("Debug logging for UART over RTT started.");

  ctl_advertising();

  // nfc_init();
  watch_dog_init();

  // Enter main loop.
  for (;;) {
    main_loop();
    app_sched_execute();
    idle_state_handle();
  }
}

/**
 * @}
 */
