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
static uint8_t driver_spi_rx_buf[256];
static uint8_t driver_spi_tx_buf[256];

bool spi_dir_out = false;
bool spi_send_done = false;

void usr_spim_init(void)
{
    ret_code_t err_code;

    nrfx_spim_config_t driver_spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    driver_spi_config.ss_pin = STM32_SPI2_CSN_IO;
    driver_spi_config.miso_pin = STM32_SPI2_MISO_IO;
    driver_spi_config.mosi_pin = STM32_SPI2_MOSI_IO;
    driver_spi_config.sck_pin = STM32_SPI2_CLK_IO;
    driver_spi_config.frequency = NRF_SPIM_FREQ_4M;
    err_code = nrfx_spim_init(&m_spim_master, &driver_spi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
}

void usr_spi_write(uint8_t* p_buffer, uint32_t size)
{
    spi_dir_out = true;

    while ( true )
    {
        if ( size <= 255 )
        {
            driver_spim_xfer.tx_length = size;
            driver_spim_xfer.p_tx_buffer = p_buffer;
            driver_spim_xfer.rx_length = 0;
            driver_spim_xfer.p_rx_buffer = driver_spi_rx_buf;
            APP_ERROR_CHECK(nrfx_spim_xfer(&m_spim_master, &driver_spim_xfer, 0));
            break;
        }
        else
        {
            driver_spim_xfer.tx_length = 255;
            driver_spim_xfer.p_tx_buffer = p_buffer;
            driver_spim_xfer.rx_length = 0;
            driver_spim_xfer.p_rx_buffer = driver_spi_rx_buf;
            APP_ERROR_CHECK(nrfx_spim_xfer(&m_spim_master, &driver_spim_xfer, 0));
            p_buffer += 255;
            size -= 255;
        }
    }

    uint32_t timeout = 100000;
    while ( !spi_send_done )
    {
        timeout--;
        if ( timeout == 0 )
        {
            break;
        }
        nrf_delay_us(1);
    }
    spi_send_done = false;
    spi_dir_out = false;
}
void usr_spi_read(uint8_t* p_buffer, uint32_t size)
{
    driver_spim_xfer.tx_length = 0;
    driver_spim_xfer.p_tx_buffer = driver_spi_tx_buf;
    driver_spim_xfer.rx_length = size;
    driver_spim_xfer.p_rx_buffer = p_buffer;
    APP_ERROR_CHECK(nrfx_spim_xfer(&m_spim_master, &driver_spim_xfer, 0));
    // NRF_LOG_HEXDUMP_INFO(p_buffer, size);
}

// Disable spi mode to enter low power mode
void usr_spi_disable(void)
{
    nrfx_spim_uninit(&m_spim_master);
}

bool data_recived_flag = false;
uint8_t data_recived_buf[DATA_RECV_BUF_SIZE];
uint16_t data_recived_len = 0;
uint16_t data_remaining_len = 0;
uint16_t data_recived_offset = 0;

#define PACKAGE_LENTH 64
#define HEAD_LENTH    9

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