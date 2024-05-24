/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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
 * @defgroup nfc_uart_tag_example_main main.c
 * @{
 * @ingroup nfc_uart_tag_example
 * @brief The application main file of NFC UART Tag example.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "boards.h"
#include "data_transmission.h"
#include "nfc_t4t_lib.h"
#include "nrf_drv_twi.h"
#include "sdk_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#include "nfc.h"

#define MAX_APDU_LEN 1024 /**< Maximal APDU length, Adafruit limitation. */
// #define HEADER_FIELD_SIZE 1      /**< Header field size. */
#define HEADER_FIELD_SIZE 0 /**< no header */
#define PACKAGE_LENTH     64
#define HEAD_LENTH        9

static bool multi_package = false;

// static uint8_t i2c_buf_dma[255];

// NFC buffer
uint8_t nfc_apdu[253];
uint32_t nfc_apdu_len = 0;
uint8_t nfc_data_out_buf[APDU_BUFF_SIZE];
uint32_t nfc_data_out_len = 0;
bool nfc_multi_packet = false;

bool data_recived_flag = false;
uint8_t data_recived_buf[APDU_BUFF_SIZE];
uint16_t data_recived_len = 0;
uint16_t data_remaining_len = 0;
uint16_t data_recived_offset = 0;

extern uint8_t spi_evt_flag;

static void apdu_command(const uint8_t* p_buf, uint32_t data_len);
bool apdu_cmd = false;

// TWI driver
// static volatile bool twi_xfer_done = false ;
// static uint8_t twi_xfer_dir = 0; //0-write 1-read
extern uint8_t ble_adv_switch_flag;
uint8_t ctl_channel_flag;

#if 0
/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with simulated
 * EEPROM memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

static void twi_handler(nrf_drv_twi_evt_t const * p_event, void *p_context )
{
    static uint8_t read_state = READSTATE_IDLE;
    static uint32_t data_len =0;
    uint32_t len;
    
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            twi_xfer_done=true;
            if(twi_xfer_dir==1)
            {
                if(read_state == READSTATE_IDLE)
                {
                    if(data_recived_buf[0] == '?' && data_recived_buf[1] == '#' && data_recived_buf[2] == '#')
                    {
                        read_state = READSTATE_READ_INFO;
                        data_recived_len= 3;
                        //nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+data_recived_len,6);//read id+len bytes len
                    }
                }
                else if(read_state == READSTATE_READ_INFO)
                {
                    data_recived_len += 6;
                    data_len = ((uint32_t)data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) + (data_recived_buf[7] << 8) + data_recived_buf[8];
                    len=data_len>255?255:data_len;
                    if(len > 0)
                    {
                        read_state = READSTATE_READ_DATA;
                        //nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+data_recived_len,len);//read id+len bytes len
                        data_len-=len;
                        data_recived_len+=len;
                    }
                    else
                    {
                        data_recived_flag = true;
                        read_state = READSTATE_IDLE;
                    }
                }
                else if(read_state == READSTATE_READ_DATA)
                {
                    len=data_len>255?255:data_len;
                    if(len>0)
                    {
                        //nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+data_recived_len,len);//read id+len bytes len
                        data_len-=len;
                        data_recived_len+=len;
                    }
                    else
                    {
                        data_recived_flag = true;
                        read_state = READSTATE_IDLE; 
                    }
                }
            }
            else
            {
                 
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}
#endif
#if 0
/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
int twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, twi_handler, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

bool i2c_master_write(uint8_t *buf,uint32_t len)
{
    ret_code_t err_code;
    uint32_t offset = 0;
    
    twi_xfer_dir = 0;
    twi_xfer_done = false;
    
    NRF_LOG_INFO("twi send data len =%d",len);    
    while(len > 255)
    {
        while(nrf_drv_twi_is_busy(&m_twi_master));
        memcpy(i2c_buf_dma,buf+offset,255);
        err_code = nrf_drv_twi_tx(&m_twi_master,SLAVE_ADDR,i2c_buf_dma,255,true);
        offset += 255;
        len -= 255;
    }
    if(len)
    {    
        while(nrf_drv_twi_is_busy(&m_twi_master));
        memcpy(i2c_buf_dma,buf+offset,len);
        err_code = nrf_drv_twi_tx(&m_twi_master,SLAVE_ADDR,i2c_buf_dma,len,false); 
    }            
    NRF_LOG_INFO("twi send data finish");
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }    
    return true;

}

bool i2c_master_write_ex(uint8_t *buf,uint8_t len,bool no_stop)
{
    ret_code_t err_code;
    
    twi_xfer_dir = 0;
    twi_xfer_done = false;    
    NRF_LOG_INFO("twi send data len =%d ,%d",len,no_stop); 
    while(nrf_drv_twi_is_busy(&m_twi_master));
    err_code = nrf_drv_twi_tx(&m_twi_master,SLAVE_ADDR,buf,len,no_stop);            
    NRF_LOG_INFO("twi send data finish");
    if(NRF_SUCCESS != err_code)
    {
        NRF_LOG_INFO("twi send error");
        return false;
    }    
    return true;

}

bool i2c_master_read(void)
{    
    uint32_t offset = 0;
    ret_code_t err_code;

    twi_xfer_dir = 1;
    data_recived_len= 0;
    
    err_code=nrf_drv_twi_rx(&m_twi_master,SLAVE_ADDR,data_recived_buf+offset,3);
    if(NRF_SUCCESS != err_code)
    {
        return false;
    }
    return true;
}
#endif
/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void* context, nfc_t4t_event_t event, const uint8_t* data, size_t dataLength, uint32_t flags)
{
    ret_code_t err_code;

    (void)context;

    switch ( event )
    {
    case NFC_T4T_EVENT_FIELD_ON:
        multi_package = false;
        NRF_LOG_INFO("NFC Tag has been selected. UART transmission can start...");
#ifdef DEV_BSP
        bsp_board_led_on(BSP_BOARD_LED_1);
#endif
        break;

    case NFC_T4T_EVENT_FIELD_OFF:
        multi_package = false;
        NRF_LOG_INFO("NFC field lost. Data from UART will be discarded...");
#ifdef DEV_BSP
        bsp_board_led_off(BSP_BOARD_LED_1);
#endif
        break;
    case NFC_T4T_EVENT_DATA_IND:
        spi_evt_flag = 0;
        if ( dataLength > APDU_BUFF_SIZE )
        {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }

        if ( flags != NFC_T4T_DI_FLAG_MORE )
        {
            memcpy(nfc_apdu, data, dataLength);
            nfc_apdu_len = dataLength;
            nfc_multi_packet = false;
            NRF_LOG_INFO("NFC RX data length: %d", dataLength);
            // NRF_LOG_HEXDUMP_INFO(data,dataLength);
            apdu_command(data, dataLength);
            if ( nfc_data_out_len > 0 )
            {
                // Send the response PDU over NFC.
                err_code = nfc_t4t_response_pdu_send(nfc_data_out_buf, nfc_data_out_len + HEADER_FIELD_SIZE);
                APP_ERROR_CHECK(err_code);
                nfc_data_out_len = 0;
            }
#ifdef DEV_BSP
            bsp_board_led_off(BSP_BOARD_LED_2);
#endif
        }
        else
        {
            multi_package = true;
            memcpy(nfc_apdu, data, dataLength);
            nfc_apdu_len = dataLength;
            apdu_cmd = true;
            nfc_multi_packet = true;
            usr_spi_write((uint8_t*)data, dataLength);
#ifdef DEV_BSP
            bsp_board_led_on(BSP_BOARD_LED_2);
#endif
        }
        break;

    default:
        break;
    }
}

/**
 * @brief Function for application main entry.
 */
int nfc_init(void)
{
    ret_code_t err_code;

    /* Set up NFC */
    err_code = nfc_t4t_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);

    /* Start sensing NFC field */
    err_code = nfc_t4t_emulation_start();
    APP_ERROR_CHECK(err_code);
    return 0;
}

void nfc_disable(void)
{
    nfc_t4t_emulation_stop();
}
void poll_st_resp_data()
{
    if ( data_remaining_len > 63 )
    {
        usr_spi_read(data_recived_buf + data_recived_offset, 64);
        memmove(data_recived_buf + data_recived_offset, data_recived_buf + data_recived_offset + 1, 63);
        data_remaining_len -= 63;
        data_recived_offset += 63;
        data_recived_len += 63;
        return;
    }

    if ( data_remaining_len )
    {
        usr_spi_read(data_recived_buf + data_recived_offset, 64);
        memmove(data_recived_buf + data_recived_offset, data_recived_buf + data_recived_offset + 1, 63);
        data_recived_len += data_remaining_len;
        data_remaining_len = 0;
        data_recived_offset = 0;
        data_recived_flag = true;
    }
}

void read_st_resp_data(void)
{
    uint32_t receive_offset = 0;
    uint32_t data_len = 0;

    if ( nrf_gpio_pin_read(TWI_STATUS_GPIO) == 0 )
    {
        if ( data_remaining_len )
        {
            poll_st_resp_data();
            return;
        }

        usr_spi_read(data_recived_buf, 64);
        receive_offset = PACKAGE_LENTH;
        if ( data_recived_buf[0] == '?' && data_recived_buf[1] == '#' && data_recived_buf[2] == '#' )
        {
            data_len = ((uint32_t)data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) +
                       (data_recived_buf[7] << 8) + data_recived_buf[8];
            if ( data_len <= (PACKAGE_LENTH - HEAD_LENTH) )
            {
                data_recived_len = data_len + HEAD_LENTH;
                data_recived_flag = true;
                data_len = 0;
                receive_offset = 0;
            }
            else
            {
                data_recived_len += PACKAGE_LENTH;
                data_remaining_len = data_len - (PACKAGE_LENTH - HEAD_LENTH);
                data_recived_offset = receive_offset;
            }
        }
    }
}

static void apdu_command(const uint8_t* p_buf, uint32_t data_len)
{
    static bool reading = false;

    if ( multi_package ) // multi_package end
    {
        multi_package = false;
        apdu_cmd = true;
        usr_spi_write((uint8_t*)p_buf, data_len);
        nfc_data_out_len = 2;
        memcpy(nfc_data_out_buf, "\x90\x00", nfc_data_out_len);
    }
    else
    {
        if ( p_buf[0] == '?' )
        {
            data_recived_flag = false;
            apdu_cmd = true;
            reading = false;
            usr_spi_write((uint8_t*)p_buf, data_len);
            nfc_data_out_len = 2;
            memcpy(nfc_data_out_buf, "\x90\x00", nfc_data_out_len);
        }
        else if ( p_buf[0] == '#' && p_buf[1] == '*' && p_buf[2] == '*' )
        {
            // usart
            if ( reading == false )
            {
                read_st_resp_data();
                reading = true;
                nfc_data_out_len = 3;
                memcpy(nfc_data_out_buf, "#**", nfc_data_out_len);
            }
            else
            {
                if ( data_recived_flag == false )
                {
                    nfc_data_out_len = 3;
                    memcpy(nfc_data_out_buf, "#**", nfc_data_out_len);
                }
                else
                {
                    data_recived_flag = false;
                    reading = false;
                    nfc_data_out_len = data_recived_len;
                    memcpy(nfc_data_out_buf, data_recived_buf, nfc_data_out_len);
                }
            }
        }
        else
        {
            if ( p_buf[0] == 0x5A && p_buf[1] == 0xA5 && p_buf[2] == 0x07 && p_buf[3] == 0x1 )
            {
                if ( p_buf[4] == 0x03 )
                {
                    ble_adv_switch_flag = 3;
                }
                else if ( p_buf[4] == 0x02 )
                {
                    ble_adv_switch_flag = 2;
                }
                ctl_channel_flag = 2;
                nfc_data_out_len = 3;
                memcpy(nfc_data_out_buf, "\xA5\x5\01", nfc_data_out_len);
            }
            else
            {
                nfc_data_out_len = 2;
                memcpy(nfc_data_out_buf, "\x6D\x00", nfc_data_out_len);
            }
        }
    }
}

void nfc_poll(void* p_event_data, uint16_t event_size)
{
    if ( apdu_cmd == true )
    {
        apdu_cmd = false;
        usr_spi_write(nfc_apdu, nfc_apdu_len);
    }
}
/** @} */
