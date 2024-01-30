#include "axp_supply.h"
#include "axp_config.h"
#include "axp_mfd_216.h"
#include "stdio.h"

struct axp_adc_res adc;
struct axp_charger charger;

void axp_enable_irqs(uint64_t irqs) {
  uint8_t byte_data[5];

  byte_data[0] = (irqs)&0xff;
  byte_data[1] = ((irqs) >> 8) & 0xff;
  byte_data[2] = ((irqs) >> 16) & 0xff;
  byte_data[3] = ((irqs) >> 24) & 0xff;
  byte_data[4] = ((irqs) >> 32) & 0xff;

  axp216_write(AXP_INTEN1, byte_data[0]);
  // NRF_LOG_INFO("AXP_INTEN1(reg_0x40)=0x%2x",byte_data[0]);

  axp216_write(AXP_INTEN2, byte_data[1]);
  // NRF_LOG_INFO("AXP_INTEN2(reg_0x41)=0x%2x",byte_data[1]);

  axp216_write(AXP_INTEN3, byte_data[2]);
  // NRF_LOG_INFO("AXP_INTEN3(reg_0x42)=0x%2x",byte_data[2]);

  axp216_write(AXP_INTEN4, byte_data[3]);
  // NRF_LOG_INFO("AXP_INTEN4(reg_0x43)=0x%2x",byte_data[3]);

  axp216_write(AXP_INTEN5, byte_data[4]);
  // NRF_LOG_INFO("AXP_INTEN5(reg_0x44)=0x%2x",byte_data[4]);
}

// Set low power protection voltage point
void set_voff_vol(void) {
  uint8_t val;

  if (SHUTDOWNVOL >= 2600 && SHUTDOWNVOL <= 3300) {
    if (SHUTDOWNVOL > 3200) {
      val = 0x07;
    } else if (SHUTDOWNVOL > 3100) {
      val = 0x06;
    } else if (SHUTDOWNVOL > 3000) {
      val = 0x05;
    } else if (SHUTDOWNVOL > 2900) {
      val = 0x04;
    } else if (SHUTDOWNVOL > 2800) {
      val = 0x03;
    } else if (SHUTDOWNVOL > 2700) {
      val = 0x02;
    } else if (SHUTDOWNVOL > 2600) {
      val = 0x01;
    } else {
      val = 0x00;
    }

    axp_update(AXP_VOFF_SET, val, 0x07);
  }
}

void axp216_init(void) {
  int var, tmp;
  uint8_t val, val2, val3;
  uint8_t ocv_cap[32], i;
  int Cur_CoulombCounter, rdc;

  axp_set_charge();  // Charging parameter setting
  set_voff_vol();    // Voff voltage setting
  axp_enable_irqs(AXP_NOTIFIER_ON);

  /*ADC Set*/
  axp216_read(AXP_ADC_CONTROL3, 1, &val3);
  switch (ADCFREQ / 100) {
    case 1:
      val3 &= ~(3 << 6);
      break;
    case 2:
      val3 &= ~(3 << 6);
      val3 |= 1 << 6;
      break;
    case 4:
      val3 &= ~(3 << 6);
      val3 |= 2 << 6;
      break;
    case 8:
      val3 |= 3 << 6;
      break;
    default:
      break;
  }
  axp216_write(AXP_ADC_CONTROL3, val3);

  /* usb voltage limit */
  if ((USBVOLLIM) && (USBVOLLIMEN)) {
    axp_set_bits(AXP_CHARGE_VBUS, 0x40);
    var = USBVOLLIM * 1000;
    if (var >= 4000000 && var <= 4700000) {
      tmp = (var - 4000000) / 100000;
      axp216_read(AXP_CHARGE_VBUS, 1, &val);
      val &= 0xC7;
      val |= tmp << 3;
      axp216_write(AXP_CHARGE_VBUS, val);
    }
  } else {
    axp_clr_bits(AXP_CHARGE_VBUS, 0x40);
  }

  /*usb current limit*/
  if ((USBCURLIM) && (USBCURLIMEN)) {
    axp_clr_bits(AXP_CHARGE_VBUS, 0x02);
    var = USBCURLIM * 1000;
    if (var == 900000)
      axp_clr_bits(AXP_CHARGE_VBUS, 0x03);
    else if (var == 500000) {
      axp_set_bits(AXP_CHARGE_VBUS, 0x01);
    }
  } else {
    axp_set_bits(AXP_CHARGE_VBUS, 0x03);
  }

  // set low power warning/shutdown level
  axp216_write(AXP_WARNING_LEVEL, ((BATLOWLV1 - 5) << 4) + BATLOWLV2);

  ocv_cap[0] = OCVREG0;
  ocv_cap[1] = OCVREG1;
  ocv_cap[2] = OCVREG2;
  ocv_cap[3] = OCVREG3;
  ocv_cap[4] = OCVREG4;
  ocv_cap[5] = OCVREG5;
  ocv_cap[6] = OCVREG6;
  ocv_cap[7] = OCVREG7;
  ocv_cap[8] = OCVREG8;
  ocv_cap[9] = OCVREG9;
  ocv_cap[10] = OCVREGA;
  ocv_cap[11] = OCVREGB;
  ocv_cap[12] = OCVREGC;
  ocv_cap[13] = OCVREGD;
  ocv_cap[14] = OCVREGE;
  ocv_cap[15] = OCVREGF;
  ocv_cap[16] = OCVREG10;
  ocv_cap[17] = OCVREG11;
  ocv_cap[18] = OCVREG12;
  ocv_cap[19] = OCVREG13;
  ocv_cap[20] = OCVREG14;
  ocv_cap[21] = OCVREG15;
  ocv_cap[22] = OCVREG16;
  ocv_cap[23] = OCVREG17;
  ocv_cap[24] = OCVREG18;
  ocv_cap[25] = OCVREG19;
  ocv_cap[26] = OCVREG1A;
  ocv_cap[27] = OCVREG1B;
  ocv_cap[28] = OCVREG1C;
  ocv_cap[29] = OCVREG1D;
  ocv_cap[30] = OCVREG1E;
  ocv_cap[31] = OCVREG1F;

  for (i = 0; i < 32; i++) {
    axp216_write(0xC0 + i, ocv_cap[i]);
  }

  /* pok open time set */
  axp216_read(AXP_POK_SET, 1, &val);
  if (PEKOPEN < 1000)
    val &= 0x3f;
  else if (PEKOPEN < 2000) {
    val &= 0x3f;
    val |= 0x40;
  } else if (PEKOPEN < 3000) {
    val &= 0x3f;
    val |= 0x80;
  } else {
    val &= 0x3f;
    val |= 0xc0;
  }
  axp216_write(AXP_POK_SET, val);

  /* pok long time set*/
  if (PEKLONG < 1000)
    tmp = 1000;
  else if (PEKLONG > 2500)
    tmp = 2500;
  else
    tmp = PEKLONG;
  axp216_read(AXP_POK_SET, 1, &val);
  val &= 0xcf;
  val |= (((tmp - 1000) / 500) << 4);
  axp216_write(AXP_POK_SET, val);

  /* pek offlevel poweroff en set*/
  if (PEKOFFEN) {
    tmp = 1;
  } else {
    tmp = 0;
  }
  axp216_read(AXP_POK_SET, 1, &val);
  val &= 0xf7;
  val |= (tmp << 3);
  axp216_write(AXP_POK_SET, val);

  /*Init offlevel restart or not */
  if (PEKOFFRESTART) {
    axp_set_bits(AXP_POK_SET, 0x04);  // restart
  } else {
    axp_clr_bits(AXP_POK_SET, 0x04);  // not restart
  }

  /* pek delay set */
  axp216_read(AXP_OFF_CTL, 1, &val);
  val &= 0xfc;
  val |= ((PEKDELAY / 8) - 1);
  axp216_write(AXP_OFF_CTL, val);

  /* pek offlevel time set */
  if (PEKOFF < 4000)
    tmp = 4000;
  else if (PEKOFF > 10000)
    tmp = 10000;
  else
    tmp = PEKOFF;
  tmp = (tmp - 4000) / 2000;
  axp216_read(AXP_POK_SET, 1, &val);
  val &= 0xfc;
  val |= tmp;
  axp216_write(AXP_POK_SET, val);
  /*Init 16's Reset PMU en */
  if (PMURESET) {
    axp_set_bits(0x8F, 0x08);  // enable
  } else {
    axp_clr_bits(0x8F, 0x08);  // disable
  }

  /*Init IRQ wakeup en*/
  if (IRQWAKEUP) {
    axp_set_bits(0x8F, 0x80);  // enable
  } else {
    axp_clr_bits(0x8F, 0x80);  // disable
  }

  /*Init N_VBUSEN status*/
  if (VBUSEN) {
    axp_set_bits(0x8F, 0x10);  // output
  } else {
    axp_clr_bits(0x8F, 0x10);  // input
  }

  /*Init InShort status*/
  if (VBUSACINSHORT) {
    axp_set_bits(0x8F, 0x60);  // InShort
  } else {
    axp_clr_bits(0x8F, 0x60);  // auto detect
  }

  /*Init CHGLED function*/
  if (CHGLEDFUN) {
    axp_set_bits(0x32, 0x08);  // control by charger
  } else {
    axp_clr_bits(0x32, 0x08);  // drive MOTO
  }

  /*set CHGLED Indication Type*/
  if (CHGLEDTYPE) {
    axp_set_bits(0x45, 0x10);  // Type A
  } else {
    axp_clr_bits(0x45, 0x10);  // Type B
  }

  /*Init PMU Over Temperature protection*/
  if (OTPOFFEN) {
    axp_set_bits(0x8f, 0x04);  // enable
  } else {
    axp_clr_bits(0x8f, 0x04);  // disable
  }

  /*Init battery capacity correct function*/
  if (BATCAPCORRENT) {
    axp_set_bits(0xb8, 0x20);  // enable
  } else {
    axp_clr_bits(0xb8, 0x20);  // disable
  }
  /* Init battery regulator enable or not when charge finish*/
  if (BATREGUEN) {
    axp_set_bits(0x34, 0x20);  // enable
  } else {
    axp_clr_bits(0x34, 0x20);  // disable
  }

  if (!BATDET)
    axp_clr_bits(AXP_PDBC, 0x40);
  else
    axp_set_bits(AXP_PDBC, 0x40);

  /* RDC initial */
  axp216_read(AXP_RDC0, 1, &val2);
  // If configuring the internal resistance
  // of the battery, manually configure
  if ((BATRDC) && (!(val2 & 0x40))) {
    rdc = (BATRDC * 10000 + 5371) / 10742;
    axp216_write(AXP_RDC0, ((rdc >> 8) & 0x1F) | 0x80);
    axp216_write(AXP_RDC1, rdc & 0x00FF);
  }

  // Initialize the RDC so that it calculates
  // the correct OCV in advance, and then start
  // the metering system here
  axp216_read(AXP_BATCAP0, 1, &val2);
  if ((BATCAP) && (!(val2 & 0x80))) {
    Cur_CoulombCounter = BATCAP * 1000 / 1456;
    axp216_write(AXP_BATCAP0, ((Cur_CoulombCounter >> 8) | 0x80));
    axp216_write(AXP_BATCAP1, Cur_CoulombCounter & 0x00FF);
  } else if (!BATCAP) {
    axp216_write(AXP_BATCAP0, 0x00);
    axp216_write(AXP_BATCAP1, 0x00);
  }
  // Power reading
  axp216_read(AXP_CAP, 1, &val2);
  // charger->rest_vol = (int) (val2 & 0x7F);

  NRF_LOG_INFO("now_rest_CAP = %d", (val2 & 0x7F));

  // Enable AXP IRQ pin
  axp_update(AXP_VOFF_SET, 0x10, 0x10);
}

static void axp_read_adc(struct axp_adc_res *adc) {
  uint8_t tmp[8];

  adc->vac_res = 0;
  adc->iac_res = 0;
  adc->vusb_res = 0;
  adc->iusb_res = 0;
  axp216_read(AXP_VBATH_RES, 6, tmp);
  adc->vbat_res = ((uint16_t)tmp[0] << 8) | tmp[1];
  adc->ichar_res = ((uint16_t)tmp[2] << 8) | tmp[3];
  adc->idischar_res = ((uint16_t)tmp[4] << 8) | tmp[5];
  axp216_read(AXP_OCVBATH_RES, 2, tmp);
  adc->ocvbat_res = ((uint16_t)tmp[0] << 8) | tmp[1];
}

static int axp_vbat_to_mV(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))) * 1100 / 1000; }

static int axp_ocvbat_to_mV(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))) * 1100 / 1000; }

static int axp_vdc_to_mV(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))) * 1700 / 1000; }

static int axp_ibat_to_mA(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))); }

static int axp_icharge_to_mA(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))); }

static int axp_iac_to_mA(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))) * 625 / 1000; }

static int axp_iusb_to_mA(uint16_t reg) { return ((int)(((reg >> 8) << 4) | (reg & 0x000F))) * 375 / 1000; }

static void axp_charger_update_state(struct axp_charger *charger) {
  uint8_t val[2];
  uint16_t tmp;

  axp216_read(AXP_CHARGE_STATUS, 2, val);
  tmp = (val[1] << 8) + val[0];
  charger->is_on = (val[1] & AXP_IN_CHARGE) ? 1 : 0;
  charger->fault = val[1];
  charger->bat_det = (tmp & AXP_STATUS_BATEN) ? 1 : 0;
  charger->ac_det = (tmp & AXP_STATUS_ACEN) ? 1 : 0;
  charger->usb_det = (tmp & AXP_STATUS_USBEN) ? 1 : 0;
  charger->usb_valid = (tmp & AXP_STATUS_USBVA) ? 1 : 0;
  charger->ac_valid = (tmp & AXP_STATUS_ACVA) ? 1 : 0;
  charger->ext_valid = charger->ac_valid | charger->usb_valid;
  charger->bat_current_direction = (tmp & AXP_STATUS_BATCURDIR) ? 1 : 0;
  charger->in_short = (tmp & AXP_STATUS_ACUSBSH) ? 1 : 0;
  charger->batery_active = (tmp & AXP_STATUS_BATINACT) ? 1 : 0;
  charger->int_over_temp = (tmp & AXP_STATUS_ICTEMOV) ? 1 : 0;
  axp216_read(AXP_CHARGE_CONTROL1, 1, val);
  charger->charge_on = ((val[0] >> 7) & 0x01);
}

static void axp_charger_update(struct axp_charger *charger) {
  uint16_t tmp;
  uint8_t val[2];

  charger->adc = &adc;
  axp_read_adc(&adc);
  tmp = charger->adc->vbat_res;
  charger->vbat = axp_vbat_to_mV(tmp);
  tmp = charger->adc->ocvbat_res;
  charger->ocv = axp_ocvbat_to_mV(tmp);
  // tmp = charger->adc->ichar_res + charger->adc->idischar_res;
  charger->ibat = ABS(axp_icharge_to_mA(charger->adc->ichar_res) - axp_ibat_to_mA(charger->adc->idischar_res));
  tmp = 00;
  charger->vac = axp_vdc_to_mV(tmp);
  tmp = 00;
  charger->iac = axp_iac_to_mA(tmp);
  tmp = 00;
  charger->vusb = axp_vdc_to_mV(tmp);
  tmp = 00;
  charger->iusb = axp_iusb_to_mA(tmp);
  axp216_read(AXP_INTTEMP, 2, val);

  tmp = (val[0] << 4) + (val[1] & 0x0F);
  charger->ic_temp = (int)tmp * 1063 / 10000 - 2667 / 10;
  charger->disvbat = charger->vbat;
  charger->disibat = axp_ibat_to_mA(charger->adc->idischar_res);
}

void axp_charging_monitor(void) {
  uint8_t val, temp_val[4];
  uint8_t batt_max_cap_val[3];
  int batt_max_cap, coulumb_counter;

  axp_charger_update_state(&charger);
  axp_charger_update(&charger);

  axp216_read(AXP_CAP, 1, &val);
  charger.rest_vol = (int)(val & 0x7F);

  axp216_read(0xe2, 2, temp_val);
  coulumb_counter = (((temp_val[0] & 0x7f) << 8) + temp_val[1]) * 1456 / 1000;
  axp216_read(0xe0, 2, temp_val);
  batt_max_cap = (((temp_val[0] & 0x7f) << 8) + temp_val[1]) * 1456 / 1000;

  /* Avoid the power stay in 100% for a long time. */
  if (coulumb_counter > batt_max_cap) {
    batt_max_cap_val[0] = temp_val[0] | (0x1 << 7);
    batt_max_cap_val[1] = temp_val[1];
    axp216_write(0xe2, batt_max_cap_val[0]);
    axp216_write(0xe3, batt_max_cap_val[1]);
    // NRF_LOG_INFO("Axp22 coulumb_counter = %d\n",batt_max_cap);
  }

  if (axp_debug) {
    // NRF_LOG_INFO("*************** AXP216 Debug********************* \n");
    // NRF_LOG_INFO("charger->ic_temp = %d\n",charger.ic_temp);
    // NRF_LOG_INFO("charger->vbat = %d\n",charger.vbat);
    // NRF_LOG_INFO("charger->ibat = %d\n",charger.ibat);
    // NRF_LOG_INFO("charger->ocv = %d\n",charger.ocv);
    // NRF_LOG_INFO("charger->disvbat = %d\n",charger.disvbat);
    // NRF_LOG_INFO("charger->disibat = %d\n",charger.disibat);
    // NRF_LOG_INFO("charger->rest_vol = %d\n",charger.rest_vol); //电量百分比
    axp216_read(0xba, 2, temp_val);
    // NRF_LOG_INFO("AXP Rdc = %d\n",(((temp_val[0] & 0x1f) <<8) + temp_val[1])*10742/10000);
    axp216_read(0xe0, 2, temp_val);
    // NRF_LOG_INFO("AXP batt_max_cap = %d\n",(((temp_val[0] & 0x7f) <<8) + temp_val[1])*1456/1000);
    axp216_read(0xe2, 2, temp_val);
    // NRF_LOG_INFO("AXP coulumb_counter = %d\n",(((temp_val[0] & 0x7f) <<8) + temp_val[1])*1456/1000);
    axp216_read(0xb8, 1, temp_val);
    // NRF_LOG_INFO("AXP REG_B8 = %x\n",temp_val[0]);
    axp216_read(0xe4, 2, temp_val);
    // NRF_LOG_INFO("AXP OCV_percentage = %d\n",(temp_val[0] & 0x7f));
    // NRF_LOG_INFO("AXP Coulumb_percentage = %d\n",(temp_val[1] & 0x7f));
    // NRF_LOG_INFO("charger->is_on = %d\n",charger.is_on);	 //充电指示
    // NRF_LOG_INFO("charger->bat_current_direction = %d\n",charger.bat_current_direction);
    // NRF_LOG_INFO("charger->charge_on = %d\n",charger.charge_on);//充电使能
    // NRF_LOG_INFO("charger->ext_valid = %d\n",charger.ext_valid);

    axp216_read(0x33, 1, temp_val);
    // NRF_LOG_INFO("AXP REG_33 = %x\n",temp_val[0]);
    temp_val[0] = temp_val[0] & 0x0f;
    // NRF_LOG_INFO("pmu_chgcur = %d\n",temp_val[0]*150+300);  //充电电流配置

    // NRF_LOG_INFO("********************END********************* \n");
  }
}

void axp_set_charge(void) {
  uint8_t val = 0x00;
  uint8_t tmp = 0x00;
  uint8_t charge_en;
  int chgpretime, chgcsttime, chgcur;

  charge_en = EN_CHARGE;
  chgpretime = CHGPRETIME;
  chgcsttime = CHGCSTTIME;
  chgcur = STACHGCUR;

  if (CHGVOL < 4200000) {
    val &= ~(3 << 5);
  } else if (CHGVOL < 4240000) {
    val &= ~(3 << 5);
    val |= 1 << 5;
  } else if (CHGVOL < 4350000) {
    val &= ~(3 << 5);
    val |= 1 << 6;
  } else
    val |= 3 << 5;

  if (STACHGCUR == 0) charge_en = 0;

  if (chgcur < 300000)
    chgcur = 300000;
  else if (chgcur > 2550000)
    chgcur = 2550000;

  val |= (chgcur - 300000) / 150000;

  if (ENDCHGRATE == 10) {
    val &= ~(1 << 4);
  } else {
    val |= 1 << 4;
  }

  val &= 0x7F;

  val |= charge_en << 7;

  if (chgpretime < 30) chgpretime = 30;
  if (chgcsttime < 360) chgcsttime = 360;

  tmp = ((((chgpretime - 40) / 10) << 6) | ((chgcsttime - 360) / 120));

  axp216_write(AXP_CHARGE_CONTROL1, val);
  axp_update(AXP_CHARGE_CONTROL2, tmp, 0xC2);
}
