#ifndef __PMU_H_
#define __PMU_H_

#include <stdio.h>

#include "pmu_common.h"

PMU_t* pmu_probe(PMU_Interface_t* pmu_if);

// static void pmu_status_refresh()
// {
//     pmu_p->PullStatus();

//     NRF_LOG_INFO("=== PowerStatus ===");
//     NRF_LOG_INFO("sysVoltage=%lu", pmu_p->PowerStatus->sysVoltage);
//     NRF_LOG_INFO("batteryPresent=%u", pmu_p->PowerStatus->batteryPresent);
//     NRF_LOG_INFO("batteryPercent=%u", pmu_p->PowerStatus->batteryPercent);
//     NRF_LOG_INFO("batteryVoltage=%lu", pmu_p->PowerStatus->batteryVoltage);
//     NRF_LOG_INFO("batteryTemp=%ld", pmu_p->PowerStatus->batteryTemp);
//     NRF_LOG_INFO("pmuTemp=%lu", pmu_p->PowerStatus->pmuTemp);
//     NRF_LOG_INFO("chargeAllowed=%u", pmu_p->PowerStatus->chargeAllowed);
//     NRF_LOG_INFO("chargerAvailable=%u", pmu_p->PowerStatus->chargerAvailable);
//     NRF_LOG_INFO("chargeFinished=%u", pmu_p->PowerStatus->chargeFinished);
//     NRF_LOG_INFO("wiredCharge=%u", pmu_p->PowerStatus->wiredCharge);
//     NRF_LOG_INFO("wirelessCharge=%u", pmu_p->PowerStatus->wirelessCharge);
//     NRF_LOG_INFO("chargeCurrent=%lu", pmu_p->PowerStatus->chargeCurrent);
//     NRF_LOG_INFO("dischargeCurrent=%lu", pmu_p->PowerStatus->dischargeCurrent);
//     NRF_LOG_INFO("=== ============== ===");
//     NRF_LOG_FLUSH();
// }

// static void print_buffers(uint32_t line, HL_Buff hlbuff)
// {
//     pmu_interface_p->Log(
//         PWR_LOG_LEVEL_INFO,
//         "line=%u\n"
//         "hlbuff.high=%02x\n"
//         "hlbuff.low=%02x\n"
//         "hlbuff.u16=%lu\n",
//         line, hlbuff.u8_high, hlbuff.u8_low, (hlbuff.u16 >> 4)
//     );
// }
// #define PRINT_BUFFERS() print_buffers(__LINE__, hlbuff)

#endif //__PMU_H_