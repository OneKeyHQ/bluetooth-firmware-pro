#include "lm_config.h"

/*********************************************************************
 * LOCAL VARIABLES
 */

static const nrf_drv_twi_t lm36011_m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

ret_code_t lm36011_twi_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t lm36011_twi_config = {
        .scl = LM36011_TWI_SCL_M, // 15
        .sda = LM36011_TWI_SDA_M, // 14
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false
    };

    ret = nrf_drv_twi_init(&lm36011_m_twi, &lm36011_twi_config, NULL, NULL);

    if ( NRF_SUCCESS == ret )
    {
        nrf_drv_twi_enable(&lm36011_m_twi);
    }

    return ret;
}

ret_code_t lm36011_write(const uint8_t writeAddr, const uint8_t writeData)
{
    ret_code_t ret;
    uint8_t tx_buff[LM36011_ADDRESS_LEN + 1];

    tx_buff[0] = writeAddr;
    tx_buff[1] = writeData;

    while ( nrf_drv_twi_is_busy(&lm36011_m_twi) )
        ;

    ret = nrf_drv_twi_tx(&lm36011_m_twi, LM36011_DEVICES_ADDR, tx_buff, LM36011_ADDRESS_LEN + 1, false);

    return ret;
}

ret_code_t lm36011_read(uint8_t readAddr, uint8_t byteNum, uint8_t* readData)
{
    ret_code_t ret;

    do
    {
        while ( nrf_drv_twi_is_busy(&lm36011_m_twi) )
            ;
        ret = nrf_drv_twi_tx(&lm36011_m_twi, LM36011_DEVICES_ADDR, &readAddr, LM36011_ADDRESS_LEN, false);
        if ( NRF_SUCCESS != ret )
        {
            break;
        }
        ret = nrf_drv_twi_rx(&lm36011_m_twi, LM36011_DEVICES_ADDR, readData, byteNum);
        if ( NRF_SUCCESS != ret )
        {
            break;
        }
    }
    while ( 0 );

    return ret;
}
