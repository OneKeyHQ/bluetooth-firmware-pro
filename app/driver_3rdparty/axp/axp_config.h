#ifndef __LINUX_AXP_CFG_H_
#define __LINUX_AXP_CFG_H_

#include "axp_mfd_216.h"
#include "axp_supply.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define AXP216_TWI_SDA_M 14
#define AXP216_TWI_SCL_M 15

#define POWER_IC_IRQ_IO  6
#define POWER_IC_OK_IO   7
#define POWER_IC_CHAG_IO 8

#define AXP_DEVICES_ADDR 0x34
#define AXP_ADDRESS_LEN  1

#define TWI_INSTANCE_ID 1

#define axp_debug 0

/*电池容量，mAh：根据实际电池容量来定义，对库仑计方法来说
这个参数很重要，必须配置*/
#define BATCAP 900

/*初始化电池内阻，mΩ：一般在100~200之间，不过最好根据实际
测试出来的确定，方法是打开打印信息，不接电池烧好固件后，
上电池，不接充电器，开机，开机1分钟后，接上充电器，充
1~2分钟，看打印信息中的rdc值，填入这里*/
#define BATRDC 150

/*开路电压方法中的电池电压的缓存*/
//#define AXP_VOL_MAX			1
/*
        充电功能使能：
        AXP:0-关闭，1-打开
*/
#define EN_CHARGE 1

/*
        充电电流设置，uA，0为关闭：
                AXP:300~2550,100/step
*/
/*充电电流，uA*/
#define STACHGCUR 450 * 1000

/*目标充电电压，mV*/
/*
        AXP:4100000/4200000/4240000/4350000
*/
#define CHGVOL 4350000

/*充电电流小于设置电流的ENDCHGRATE%时，停止充电，%*/
/*
        AXP:10\15
*/
#define ENDCHGRATE 15
/*关机电压，mV*/
/*
        系统设计的关机过后的电池端电压，需要与关机百分比、
        开路电压对应百分比表及低电警告电压相互配合才会有作用
*/
#define SHUTDOWNVOL 3200

/*adc采样率设置，Hz*/
/*
        AXP:100\200\400\800
*/
#define ADCFREQ 100

/*预充电超时时间，min*/
/*
        AXP:40\50\60\70
*/
#define CHGPRETIME 40
/*恒流充电超时时间，min*/
/*
        AXP:360\480\600\720
*/
#define CHGCSTTIME 360

/*pek开机时间，ms*/
/*
        按power键硬件开机时间：
                AXP:128/1000/2000/3000
*/
#define PEKOPEN 1000
/*pek长按时间，ms*/
/*
        按power键发长按中断的时间，短于此时间是短按，发短按键irq，
        长于此时间是长按，发长按键irq：
                AXP:1000/1500/2000/2500
*/
#define PEKLONG 2000
/*pek长按关机使能*/
/*
        按power键超过关机时长硬件关机功能使能：
                AXP:0-不关，1-关机
*/
#define PEKOFFEN 1
/*pek长按关机使能后开机选择*/
/*
        按power键超过关机时长硬件关机还是重启选择:
                AXP:0-只关机不重启，1-关机后重启
*/
#define PEKOFFRESTART 0
/*pekpwr延迟时间，ms*/
/*
        开机后powerok信号的延迟时间：
                AXP20:8/16/32/64
*/
#define PEKDELAY 32
/*pek长按关机时间，ms*/
/*
        按power键的关机时长：
                AXP:4000/6000/8000/10000
*/
#define PEKOFF 4000
/*过温关机使能*/
/*
        AXP内部温度过高硬件关机功能使能：
                AXP:0-不关，1-关机
*/
#define OTPOFFEN 1
/* 充电电压限制使能*/
/*
        AXP:0-关闭，1-打开
*/
#define USBVOLLIMEN 0
/*  充电限压，mV，0为不限制*/
/*
        AXP:4000~4700，100/step
*/
#define USBVOLLIM 4200
/*  USB充电限压，mV，0为不限制*/
/*
        AXP:4000~4700，100/step
*/
//#define USBVOLLIMPC			4200

/* 充电电流限制使能*/
/*
        AXP:0-关闭，1-打开
*/
#define USBCURLIMEN 1
/* 充电限流，mA，0为不限制*/
/*
        AXP:500/900
*/
#define USBCURLIM 900
/* usb 充电限流，mA，0为不限制*/
/*
        AXP:500/900
*/
#define USBCURLIMPC 900
/* PMU 中断触发唤醒使能*/
/*
        AXP:0-不唤醒，1-唤醒
*/
#define IRQWAKEUP 1
/* N_VBUSEN PIN 功能控制*/
/*
        AXP:0-输出，驱动OTG升压模块，1-输入，控制VBUS通路
*/
#define VBUSEN 1
/* ACIN/VBUS In-short 功能设置*/
/*
        AXP:0-AC VBUS分开，1-使用VBUS当AC,无单独AC
*/
#define VBUSACINSHORT 1  // 0   **
/* CHGLED 管脚控制设置*/
/*
        AXP:0-驱动马达，1-由充电功能控制
*/
#define CHGLEDFUN 1
/* CHGLED LED 类型设置*/
/*
        AXP:0-充电时led长亮，1-充电时led闪烁
*/
#define CHGLEDTYPE 0
/* 电池总容量校正使能*/
/*
        AXP:0-关闭，1-打开
*/
#define BATCAPCORRENT 1
/* 充电完成后，充电输出使能*/
/*
        AXP:0-关闭，1-打开
*/
#define BATREGUEN 1  // 0 **
/* 电池检测功能使能*/
/*
        AXP:0-关闭，1-打开
*/
#define BATDET 1
/* PMU重置使能*/
/*
        AXP:0-关闭，1-打开 按电源键16秒重置PMU功能
*/
#define PMURESET 0
/*低电警告电压1，%*/
/*
        根据系统设计来定：
        AXP:5%~20%
*/
#define BATLOWLV1 10
/*低电警告电压2，%*/
/*
        根据系统设计来定：
        AXP:0%~15%
*/
#define BATLOWLV2 5

#define ABS(x) ((x) > 0 ? (x) : -(x))

/*初始化开路电压对应百分比表*/
/*
        可以使用默认值，但是最好根据实际测试的电池来确定每级
        对应的剩余百分比，特别需要注意，关机电压SHUTDOWNVOL和电池
        容量开始校准剩余容量百分比BATCAPCORRATE这两级的准确性
        AXP适用
*/
#define OCVREG0  0    // 3.13V
#define OCVREG1  0    // 3.27V
#define OCVREG2  0    // 3.34V
#define OCVREG3  1    // 3.41V
#define OCVREG4  4    // 3.48V
#define OCVREG5  8    // 3.52V
#define OCVREG6  12   // 3.55V
#define OCVREG7  16   // 3.57V
#define OCVREG8  20   // 3.59V
#define OCVREG9  22   // 3.61V
#define OCVREGA  26   // 3.63V
#define OCVREGB  30   // 3.64V
#define OCVREGC  35   // 3.66V
#define OCVREGD  40   // 3.7V
#define OCVREGE  44   // 3.73V
#define OCVREGF  48   // 3.77V
#define OCVREG10 52   // 3.78V
#define OCVREG11 56   // 3.8V
#define OCVREG12 60   // 3.82V
#define OCVREG13 64   // 3.84V
#define OCVREG14 68   // 3.85V
#define OCVREG15 72   // 3.87V
#define OCVREG16 76   // 3.91V
#define OCVREG17 80   // 3.94V
#define OCVREG18 83   // 3.98V
#define OCVREG19 86   // 4.01V
#define OCVREG1A 90   // 4.05V
#define OCVREG1B 92   // 4.08V
#define OCVREG1C 94   // 4.1V
#define OCVREG1D 96   // 4.12V
#define OCVREG1E 98   // 4.14V
#define OCVREG1F 100  // 4.15V

/*选择需要打开的中断使能*/
static const uint64_t AXP_NOTIFIER_ON = (AXP_IRQ_USBIN | AXP_IRQ_USBRE | AXP_IRQ_USBOV | AXP_IRQ_ACIN | AXP_IRQ_ACRE | AXP_IRQ_ACOV | AXP_IRQ_BATIN | AXP_IRQ_BATRE | AXP_IRQ_CHAST | AXP_IRQ_CHAOV |
                                         AXP_IRQ_POKSH | AXP_IRQ_POKLO | AXP_IRQ_PEKFE | AXP_IRQ_PEKRE);  //|
                                                                                                          // (uint64_t)AXP_IRQ_PEKFE |(uint64_t)AXP_IRQ_PEKRE);

/* 需要做插入火牛、usb关机重启进boot时power_start设置为1，否则为0*/
//#define POWER_START 0

ret_code_t axp216_twi_master_init(void);

ret_code_t axp216_write(const uint8_t writeAddr, const uint8_t writeData);

ret_code_t axp216_read(uint8_t readAddr, uint8_t byteNum, uint8_t *readData);

void delay_ms(uint16_t nms);

void axp_set_bits(int reg, uint8_t bit_mask);

void axp_clr_bits(int reg, uint8_t bit_mask);

void axp_update(int reg, uint8_t val, uint8_t mask);

void axp_disable(void);

#endif
