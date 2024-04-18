#include "power_manage.h"

#define DC1SW 0x80

static void print_axp_reg(char* reg_name, uint8_t reg_addr)
{
    ret_code_t ret = 0;
    uint8_t val = 0x5a;
    axp216_read(reg_addr, 1, &val);
    ret = axp216_write(AXP_LDO_DC_EN2, 0xFF);
    NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
    NRF_LOG_INFO("%s(0x%02x)=0x%02x", reg_name, reg_addr, val);
    nrf_delay_ms(10);
}

#define preg(x) print_axp_reg(#x, x)

static bool axp_probe()
{
    NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);
    ret_code_t ret = 0;
    uint8_t val = 0x5a;
    for ( uint8_t addr = 0; addr <= 127; addr++ )
    {
        NRF_LOG_INFO("Probing addr 0x%02x...", addr);
        axp216_set_i2c_addr(addr);
        ret = axp216_read(AXP_STATUS, 1, &val);

        nrf_delay_ms(100);

        if ( ret != 0 )
        {
            NRF_LOG_INFO("fail\n", addr);
            NRF_LOG_FLUSH();
            continue;
        }
        else
        {
            NRF_LOG_INFO("success\n", addr);
            NRF_LOG_FLUSH();
            return true;
        }
    }
    return false;
}

static bool axp_reg_dump()
{
    NRF_LOG_INFO("**************** %s ****************", __FUNCTION__);

    ret_code_t ret = 0;
    uint8_t val = 0x5a;
    for ( uint8_t reg = 0x00; reg < 0xff; reg++ )
    {
        ret = axp216_read(reg, 1, &val);
        nrf_delay_ms(10);

        if ( ret == 0 )
        {
            NRF_LOG_INFO("reg 0x%02x = 0x%02x", reg, val);
            NRF_LOG_FLUSH();
        }
        else
        {
            NRF_LOG_INFO("reg 0x%02x FAIL ret = %lu", reg, ret);
            NRF_LOG_FLUSH();
            return false;
        }
    }
    return false;
}

ret_code_t usr_power_init(void)
{
    ret_code_t ret;

    axp216_set_i2c_addr(AXP_DEVICES_ADDR);

    ret = axp216_twi_master_init();

    // axp_probe();

    // axp_reg_dump();

    NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
    nrf_delay_ms(100); // here must delay 800ms at least
    NRF_LOG_INFO("Init twi master.");
    axp216_init();
    NRF_LOG_INFO("Init axp216 chip.");
    nrf_delay_ms(100);
    // open_all_power();
    nrf_delay_ms(500);
    clear_irq_reg();

    preg(AXP_OFF_CTL);
    return ret;
}

// ret_code_t self_power_config()
// {
//     ret_code_t ret = NRF_SUCCESS;
//     nrf_drv_power_pofwarn_config_t pof_config = {
//         .handler = NULL,
//         .thr = NRF_POWER_POFTHR_V17,
//     };
//     ret = nrf_drv_power_pof_init(&pof_config);
//     if ( ret != NRF_SUCCESS )
//         return ret;

//     uint8_t reg_val;
//     ret = axp216_read(AXP_LDO_DC_EN2, 1, &reg_val);
//     if ( ret != NRF_SUCCESS )
//         return ret;

//     if ( reg_val != 0xxx )
//     {
//         ret = axp216_write(AXP_LDO_DC_EN2, 0xFF);
//         if ( ret != NRF_SUCCESS )
//             return ret;
//     }

//     return ret;
// }

ret_code_t open_all_power(void)
{
    preg(AXP_ALDO1OUT_VOL);
    preg(AXP_ALDO2OUT_VOL);
    preg(AXP_ALDO3OUT_VOL);
    preg(AXP_DC1OUT_VOL);
    preg(AXP_LDO_DC_EN1);
    preg(AXP_LDO_DC_EN2);

    ret_code_t ret = 0;
    uint8_t val = 0;

    ret = axp216_write(AXP_LDO_DC_EN2, 0xFF);
    NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
    nrf_delay_ms(200);
    val = 0;
    ret = axp216_read(AXP_LDO_DC_EN2, 1, &val);
    NRF_LOG_INFO("%d ret=%ld", __LINE__, ret);
    nrf_delay_ms(100);

    // ALDO1 -> LDO_1V8 1.8V
    axp216_write(AXP_ALDO1OUT_VOL, 0x0B);
    // ALDO2 -> LDO_FB 3.0V
    axp216_write(AXP_ALDO2OUT_VOL, 0x1C);
    // ALDO3 -> LDO_NF 3.2V
    axp216_write(AXP_ALDO3OUT_VOL, 0x1E);

    // DCDC1 Value 3.2V
    axp216_write(AXP_DC1OUT_VOL, 0x10);

    // enable output
    axp216_read(AXP_LDO_DC_EN1, 1, &val);
    val |= 0x80;
    axp216_write(AXP_LDO_DC_EN1, val);

    nrf_delay_ms(200);

    preg(AXP_ALDO1OUT_VOL);
    preg(AXP_ALDO2OUT_VOL);
    preg(AXP_ALDO3OUT_VOL);
    preg(AXP_DC1OUT_VOL);
    preg(AXP_LDO_DC_EN1);
    preg(AXP_LDO_DC_EN2);

    return ret;
}

void close_all_power(void)
{
    uint8_t val;

    /* set  32H bit7 to 1 close all LDO&DCDC except RTC&Charger.*/
    axp216_read(AXP_OFF_CTL, 1, &val);
    val &= 0x7F;
    val |= 0x80;
    axp216_write(AXP_OFF_CTL, val);
}

// EMMC --- ALDO3(0.7~3.3V) 0x20
void ctl_emmc_power(uint8_t value)
{
    axp_update(AXP_LDO_DC_EN2, value, 0x20);
}

uint8_t get_battery_percent(void)
{
    uint8_t percent, mm;

    axp216_read(AXP_CAP, 1, &mm);
    percent = mm & 0x7F;
    // NRF_LOG_INFO("nnow_rest_CAP = %d",(percent & 0x7F));

    // axp216_read(0x10,1,&mm);//34h   52
    // NRF_LOG_INFO("switch_control_mm = %d",(mm & 0x7F) );
    axp_charging_monitor();

    return percent;
}

uint8_t get_charge_status(void)
{
    uint8_t charge_state = 0;
    uint8_t val[2];
    axp216_read(AXP_CHARGE_STATUS, 2, val);
    if ( (val[0] & AXP_STATUS_USBVA) || (val[1] & AXP_IN_CHARGE) )
    {
        charge_state = 0x03;
    }
    else
    {
        charge_state = 0x02;
    }
    return charge_state;
}

// get the charging type when charging (usb or wireless)
uint8_t get_charge_type(void)
{
    uint8_t charge_type = 0;
    uint8_t val;
    axp216_read(AXP_GPIO1_CTL, 1, &val);
    axp216_write(AXP_GPIO1_CTL, ((val & 0xF8) | 0x02)); // 设置gpio1为通用输入功能
    axp216_read(AXP_GPIO01_SIGNAL, 2, &val);
    if ( (val & AXP_IN_CHARGE_TYPE) )
    {
        charge_type = AXP_CHARGE_TYPE_USB; // usb
    }
    else
    {
        charge_type = AXP_CHARGE_TYPE_WIRELESS; // wireless
    }
    axp216_write(AXP_GPIO1_CTL, (val | 0x07)); // 为降低功耗，将gpio还原为浮空状态，只在检测时开启
    return charge_type;
}

// REG48H
uint8_t get_irq_vbus_status(void)
{
    static uint8_t last_vbus_status = 0;
    uint8_t vbus_status = 0, reg = 0;

    axp216_read(AXP_INTSTS1, 1, &reg);
    NRF_LOG_INFO("vbus status %d ", reg);
    if ( reg == IRQ_VBUS_INSERT )
    {
        vbus_status = 0x01;
    }
    else if ( reg == IRQ_VBUS_REMOVE )
    {
        vbus_status = 0x02;
    }
    // compare
    if ( last_vbus_status != vbus_status )
    {
        last_vbus_status = vbus_status;
        return last_vbus_status;
    }
    else
    {
        return 0;
    }
}
// REG49H
uint8_t get_irq_charge_status(void)
{
    static uint8_t last_charge_stasus = 0;
    uint8_t charge_status = 0, reg = 0;

    axp216_read(AXP_INTSTS2, 1, &reg);
    NRF_LOG_INFO("charge status %d ", reg);
    if ( (reg & 0x08) == 0x08 )
    {
        charge_status = IRQ_CHARGING_BAT;
    }
    else if ( (reg & 0x04) == 0x04 )
    {
        charge_status = IRQ_CHARGE_OVER;
    }
    // compare
    if ( last_charge_stasus != charge_status )
    {
        last_charge_stasus = charge_status;
        return last_charge_stasus;
    }
    else
    {
        return 0;
    }
}
// REG49H
uint8_t get_bat_con_status(void)
{
    static uint8_t last_bat_con_stasus = 0;
    uint8_t bat_con_status = 0, reg = 0;

    axp216_read(AXP_INTSTS2, 1, &reg);
    NRF_LOG_INFO("bat connect status %d ", reg);
    if ( reg == 0x80 )
    {
        last_bat_con_stasus = IRQ_CHARGING_BAT;
    }
    else if ( reg == 0x40 )
    {
        last_bat_con_stasus = IRQ_CHARGE_OVER;
    }
    // compare
    if ( last_bat_con_stasus != bat_con_status )
    {
        last_bat_con_stasus = bat_con_status;
        return last_bat_con_stasus;
    }
    else
    {
        return 0;
    }
}
// REG4BH
uint8_t get_irq_battery_status(void)
{
    static uint8_t last_bat_status = 0;
    uint8_t bat_status = 0, reg = 0;

    axp216_read(AXP_INTSTS4, 1, &reg);
    NRF_LOG_INFO("battery status %d ", reg);
    if ( reg == 0x02 )
    {
        bat_status = IRQ_LOW_BAT_1;
    }
    else if ( reg == 0x01 )
    {
        bat_status = IRQ_LOW_BAT_2;
    }
    if ( last_bat_status != bat_status )
    {
        last_bat_status = bat_status;
        return last_bat_status;
    }
    else
    {
        return 0;
    }
}

// REG 4CH
uint8_t get_irq_status(void)
{
    uint8_t reg = 0;
    axp216_read(AXP_INTSTS5, 1, &reg);
    return reg;
}

// 获取电池相关信息
void get_battery_cv_msg(uint8_t bat_reg_addr, uint8_t bat_value[2])
{
    uint8_t val[2] = {0};
    axp216_read(bat_reg_addr, 2, val);
    val[1] &= 0x0F; // 不用的位置零
    bat_value[0] = val[0];
    bat_value[1] = val[1];
}

void set_wakeup_irq(uint8_t set_value)
{
    uint8_t reg_val;

    axp216_read(AXP_VOFF_SET, 1, &reg_val);

    reg_val = (reg_val & ~0x10) | set_value;
    axp216_write(AXP_VOFF_SET, reg_val);
}

void clear_irq_reg(void)
{
    axp216_write(AXP_INTSTS1, 0xFF);
    axp216_write(AXP_INTSTS2, 0xFF);
    axp216_write(AXP_INTSTS3, 0xFF);
    axp216_write(AXP_INTSTS4, 0xFF);
    axp216_write(AXP_INTSTS5, 0xFF);
}