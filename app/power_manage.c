#include "power_manage.h"

#define DC1SW 0x80

ret_code_t usr_power_init(void)
{
    ret_code_t ret;
    
    ret = axp216_twi_master_init();
    nrf_delay_ms(800);  // here must delay 800ms at least
    NRF_LOG_INFO("Init twi master.");
    axp216_init();
    NRF_LOG_INFO("Init axp216 chip.");
    nrf_delay_ms(2000);
    open_all_power();

    clear_irq_reg();
    return ret;
}

ret_code_t open_all_power(void)
{
    ret_code_t ret;
	uint8_t val=0;
    
    ret = axp216_write(AXP_LDO_DC_EN2,0xFF );
    nrf_delay_ms(100);  
    val = 0;
    ret = axp216_read(AXP_LDO_DC_EN2,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(100);

    //set ALDO3 EMMC Value 3.2V
    axp216_write(AXP_ALDO3OUT_VOL,0x1E);
    //set DCDC1 Value 3.2V
    axp216_write(AXP_DC1OUT_VOL,0x10);
    //set LDO2 Value 3.2V
    axp216_write(AXP_ALDO2OUT_VOL,0x1F);
    //set ALDO1 Value 1.8V
    axp216_write(AXP_ALDO1OUT_VOL,0xB);
    nrf_delay_ms(100);
    return ret;
}

void close_all_power(void)
{
    uint8_t val;

	/* set  32H bit7 to 1 close all LDO&DCDC except RTC&Charger.*/
	axp216_read(AXP_OFF_CTL,1,&val);
	val &= 0x7F;
	val |= 0x80;
	axp216_write(AXP_OFF_CTL,val);
}

//EMMC --- ALDO3(0.7~3.3V) 0x20
void ctl_emmc_power(uint8_t value)
{
    axp_update(AXP_LDO_DC_EN2,value,0x20);
}

uint8_t get_battery_percent(void)
{
    uint8_t percent,mm;

    axp216_read(AXP_CAP,1,&mm);
    percent = mm & 0x7F;
    // NRF_LOG_INFO("nnow_rest_CAP = %d",(percent & 0x7F));

    // axp216_read(0x10,1,&mm);//34h   52
    // NRF_LOG_INFO("switch_control_mm = %d",(mm & 0x7F) );
    axp_charging_monitor(); 

    return percent;
}

uint8_t get_charge_status(void){
    uint8_t charge_state = 0;
    uint8_t val[2];
    axp216_read(AXP_CHARGE_STATUS,2,val);
    if((val[0] & AXP_STATUS_USBVA) || (val[1] & AXP_IN_CHARGE)){
        charge_state = 0x03;
    }else{
        charge_state = 0x02;
    }
    return charge_state;
}


// get the charging type when charging (usb or wireless)
uint8_t get_charge_type(void){
    uint8_t charge_type = 0;
    uint8_t val;
    axp216_read(AXP_GPIO1_CTL,1,&val);
    axp216_write(AXP_GPIO1_CTL,((val & 0xF8) | 0x02));   //设置gpio1为通用输入功能
    axp216_read(AXP_GPIO01_SIGNAL,2,&val);
    if((val & AXP_IN_CHARGE_TYPE)){
        charge_type = AXP_CHARGE_TYPE_USB;   //usb
    }else{
        charge_type = AXP_CHARGE_TYPE_WIRELESS;  //wireless
    }
    axp216_write(AXP_GPIO1_CTL,(val|0x07));   //为降低功耗，将gpio还原为浮空状态，只在检测时开启
    return charge_type;
}



//REG48H
uint8_t get_irq_vbus_status(void)
{
    static uint8_t last_vbus_status = 0;
    uint8_t vbus_status = 0,reg = 0;
    
    axp216_read(AXP_INTSTS1,1,&reg);
    NRF_LOG_INFO("vbus status %d ",reg);
    if(reg == IRQ_VBUS_INSERT){
        vbus_status = 0x01;
    }else if(reg == IRQ_VBUS_REMOVE){
        vbus_status = 0x02;
    }
    //compare
    if(last_vbus_status != vbus_status){
        last_vbus_status = vbus_status;
        return last_vbus_status;
    }else{
        return 0;
    }
}
//REG49H 
uint8_t get_irq_charge_status(void)
{
    static uint8_t last_charge_stasus = 0;
    uint8_t charge_status = 0,reg = 0;

    axp216_read(AXP_INTSTS2,1,&reg);
    NRF_LOG_INFO("charge status %d ",reg);
    if((reg & 0x08) == 0x08){
        charge_status = IRQ_CHARGING_BAT;
    }else if((reg & 0x04) == 0x04){
        charge_status = IRQ_CHARGE_OVER;
    }
    //compare
    if(last_charge_stasus != charge_status){
        last_charge_stasus = charge_status;
        return last_charge_stasus;
    }else{
        return 0;
    }
}
//REG49H 
uint8_t get_bat_con_status(void)
{
    static uint8_t last_bat_con_stasus = 0;
    uint8_t bat_con_status = 0,reg = 0;

    axp216_read(AXP_INTSTS2,1,&reg);
    NRF_LOG_INFO("bat connect status %d ",reg);
    if(reg == 0x80){
        last_bat_con_stasus = IRQ_CHARGING_BAT;
    }else if(reg == 0x40){
        last_bat_con_stasus = IRQ_CHARGE_OVER;
    }
    //compare
    if(last_bat_con_stasus != bat_con_status){
        last_bat_con_stasus = bat_con_status;
        return last_bat_con_stasus;
    }else{
        return 0;
    }
}
//REG4BH
uint8_t get_irq_battery_status(void)
{
    static uint8_t last_bat_status = 0;
    uint8_t bat_status = 0,reg = 0;

    axp216_read(AXP_INTSTS4,1,&reg);
    NRF_LOG_INFO("battery status %d ",reg);
    if(reg == 0x02){
        bat_status = IRQ_LOW_BAT_1;
    }else if(reg == 0x01){
        bat_status = IRQ_LOW_BAT_2;
    }
    if(last_bat_status != bat_status){
        last_bat_status = bat_status;
        return last_bat_status;
    }else{
        return 0;
    }
}
//REG 4CH
uint8_t get_irq_status(void)
{
    uint8_t key_status = 0,reg = 0;

    axp216_read(AXP_INTSTS5,1,&reg);
    NRF_LOG_INFO("key status %d ",reg);
    if((reg & IRQ_SHORT_PRESS) == IRQ_SHORT_PRESS){
        key_status = 0x01;
    }else if((reg & IRQ_LONG_PRESS) == IRQ_LONG_PRESS){
        key_status = 0x02;
    }else if((reg & IRQ_OFF_LEVEL) == IRQ_OFF_LEVEL){
        return IRQ_OFF_LEVEL;
    }

    if(key_status != 0)
    {
        return key_status;
    }else
    {
        return 0;
    }
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
    axp216_write(AXP_INTSTS1,0xFF);
    axp216_write(AXP_INTSTS2,0xFF);
    axp216_write(AXP_INTSTS3,0xFF);
    axp216_write(AXP_INTSTS4,0xFF);
    axp216_write(AXP_INTSTS5,0xFF);
}
