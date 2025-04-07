#include "data_transmission.h"
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "sdk_config.h"

static volatile bool spi_xfer_done = true;
static const nrfx_spim_t m_spim_master = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
static nrfx_spim_xfer_desc_t driver_spim_xfer;

void start_data_wait_timer(void);
void ble_nus_send(uint8_t* data, uint16_t len);

enum
{
    READSTATE_IDLE,
    READSTATE_READ_INFO,
    READSTATE_READ_DATA,
    READSTATE_READ_DATA_WAIT,
    READSTATE_READ_FIDO_STATUS,
    READSTATE_READ_FIDO_LEN,
    READSTATE_READ_FIDO_DATA,
};

#define DATA_TYPE_NUS  1
#define DATA_TYPE_FIDO 2

bool spi_dir_out = false;

void spi_event_handler(const nrfx_spim_evt_t* p_event, void* p_context)
{
    spi_xfer_done = true;
}

void usr_spim_init(void)
{
    ret_code_t err_code;

    nrfx_spim_config_t driver_spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    driver_spi_config.miso_pin = STM32_SPI2_MISO_IO;
    driver_spi_config.mosi_pin = STM32_SPI2_MOSI_IO;
    driver_spi_config.sck_pin = STM32_SPI2_CLK_IO;
    driver_spi_config.frequency = NRF_SPIM_FREQ_4M;
    err_code = nrfx_spim_init(&m_spim_master, &driver_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(STM32_SPI2_CSN_IO);
    nrf_gpio_pin_set(STM32_SPI2_CSN_IO);
}

void usr_spi_write(uint8_t* p_buffer, uint32_t size)
{
    spi_dir_out = true;

    uint8_t buffer[256] = {0};

    uint32_t padding_len = 0;

    if ( size % 16 != 0 )
    {
        padding_len = 16 - (size % 16);
        size += padding_len;
    }

    nrf_gpio_pin_clear(STM32_SPI2_CSN_IO);

    while ( size > 0 )
    {
        uint32_t send_len = size > 255 ? 255 : size;
        memcpy(buffer, p_buffer, send_len);

        if ( size == send_len )
        {
            memset(buffer + send_len, 0xFF, padding_len);
        }

        spi_xfer_done = false;

        driver_spim_xfer.tx_length = send_len;
        driver_spim_xfer.p_tx_buffer = buffer;
        driver_spim_xfer.rx_length = 0;
        driver_spim_xfer.p_rx_buffer = NULL;
        ret_code_t err_code = nrfx_spim_xfer(&m_spim_master, &driver_spim_xfer, 0);
        APP_ERROR_CHECK(err_code);
        p_buffer += send_len;
        size -= send_len;
        while ( !spi_xfer_done )
        {
            nrf_delay_us(1);
        }
    }
    nrf_gpio_pin_set(STM32_SPI2_CSN_IO);

    // uint32_t timeout = 5000;
    // while ( spi_dir_out )
    // {
    //     timeout--;
    //     if ( timeout == 0 )
    //     {
    //         break;
    //     }
    //     nrf_delay_us(1);
    // }
    // spi_dir_out = false;
}

bool usr_spi_read(uint8_t* p_buffer, uint32_t size)
{

    nrf_gpio_pin_clear(STM32_SPI2_CSN_IO);

    while ( size > 0 )
    {
        spi_xfer_done = false;
        uint32_t read_len = size > 255 ? 255 : size;
        driver_spim_xfer.tx_length = 0;
        driver_spim_xfer.p_tx_buffer = NULL;
        driver_spim_xfer.rx_length = read_len;
        driver_spim_xfer.p_rx_buffer = p_buffer;
        APP_ERROR_CHECK(nrfx_spim_xfer(&m_spim_master, &driver_spim_xfer, 0));
        p_buffer += read_len;
        size -= read_len;
        while ( !spi_xfer_done )
        {
            nrf_delay_us(1);
        }
    }
    nrf_gpio_pin_set(STM32_SPI2_CSN_IO);

    return true;
}

// Disable spi mode to enter low power mode
void usr_spi_disable(void)
{
    nrfx_spim_uninit(&m_spim_master);
}

uint8_t data_recived_buf[DATA_RECV_BUF_SIZE];
uint16_t data_recived_len = 0;
uint16_t data_recived_offset = 0;
uint8_t spi_data_type = 0;

#define PACKAGE_LENTH      64
#define HEAD_LENTH         9
#define HEAD2_LENTH        1
#define PACKAGE_DATA_LENTH 63

static uint8_t read_state = READSTATE_IDLE;

static bool spi_read_data(void)
{
    static uint32_t data_len = 0;
    if ( read_state == READSTATE_IDLE )
    {
        if ( !usr_spi_read(data_recived_buf, 3) )
        {
            return false;
        }
        if ( data_recived_buf[0] == '?' && data_recived_buf[1] == '#' && data_recived_buf[2] == '#' )
        {
            spi_data_type = DATA_TYPE_NUS;
            if ( !usr_spi_read(data_recived_buf + 3, PACKAGE_LENTH - 3) )
            {
                goto failed;
            }
            data_len = (data_recived_buf[5] << 24) + (data_recived_buf[6] << 16) + (data_recived_buf[7] << 8) +
                       data_recived_buf[8];
            if ( data_len <= (PACKAGE_LENTH - HEAD_LENTH) )
            {
                data_recived_len = data_len + HEAD_LENTH;
                data_len = 0;
                data_recived_offset = 0;
                return true;
            }
            else
            {
                if ( data_len > sizeof(data_recived_buf) - 3 )
                {
                    return false;
                }
                data_recived_len = PACKAGE_LENTH;
                data_len -= (PACKAGE_LENTH - HEAD_LENTH);
                read_state = READSTATE_READ_DATA;
                return false;
            }
        }
        else if ( data_recived_buf[0] == 'f' && data_recived_buf[1] == 'i' && data_recived_buf[2] == 'd' )
        {
            spi_data_type = DATA_TYPE_FIDO;

            // read len
            if ( !usr_spi_read(data_recived_buf, 2) )
            {
                return false;
            }
            data_len = (data_recived_buf[0] << 8) + data_recived_buf[1];
            if ( data_len > sizeof(data_recived_buf) )
            {
                return false;
            }
            if ( data_len > 0 )
            {
                if ( !usr_spi_read(data_recived_buf, data_len) )
                {
                    return false;
                }
            }
            data_recived_len = data_len;
            return true;
        }
        else
        {
            return false;
        }
    }
    else if ( read_state == READSTATE_READ_DATA )
    {
        uint8_t header = 0;
        if ( data_len > PACKAGE_DATA_LENTH )
        {
            if ( !usr_spi_read(&header, 1) )
            {
                goto failed;
            }
            if ( header != '?' )
            {
                goto failed;
            }
            if ( !usr_spi_read(data_recived_buf + data_recived_len, PACKAGE_DATA_LENTH) )
            {
                goto failed;
            }
            data_recived_len += PACKAGE_DATA_LENTH;
            data_len -= PACKAGE_DATA_LENTH;
            return false;
        }
        else
        {
            if ( !usr_spi_read(&header, 1) )
            {
                goto failed;
            }
            if ( header != '?' )
            {
                goto failed;
            }
            if ( !usr_spi_read(data_recived_buf + data_recived_len, PACKAGE_DATA_LENTH) )
            {
                goto failed;
            }
            data_recived_len += data_len;
            data_len = 0;
            read_state = READSTATE_IDLE;
            return true;
        }
    }
failed:
    read_state = READSTATE_IDLE;
    return false;
}

void spi_write_st_data(void* data, uint16_t len)
{
    uint16_t send_spi_offset = 0;

    while ( len > 0 )
    {
        uint32_t send_len = len > 64 ? 64 : len;
        if ( send_len < 64 )
        {
            usr_spi_write((uint8_t*)(data + send_spi_offset), 64);
        }
        else
        {
            usr_spi_write((uint8_t*)(data + send_spi_offset), send_len);
        }
        send_spi_offset += send_len;
        len -= send_len;
    }
}

extern void ble_fido_send(uint8_t* data, uint16_t data_len);

void spi_read_st_data(void* data, uint16_t len)
{
    if ( spi_read_data() )
    {
        if ( spi_data_type == DATA_TYPE_NUS )
        {
            ble_nus_send(data_recived_buf, data_recived_len);
        }
        else if ( spi_data_type == DATA_TYPE_FIDO )
        {
            ble_fido_send(data_recived_buf, data_recived_len);
        }
    }
}

void spi_state_reset(void)
{
    read_state = READSTATE_IDLE;
}

void spi_state_update(void)
{
    if (read_state == READSTATE_READ_DATA)
    {
        read_state = READSTATE_READ_DATA_WAIT;
    }
    else if (read_state == READSTATE_READ_DATA_WAIT)
    {
        read_state = READSTATE_IDLE;
    }
}
