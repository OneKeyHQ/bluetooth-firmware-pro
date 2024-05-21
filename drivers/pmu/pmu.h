#ifndef __PMU_H_
#define __PMU_H_

#include <stdio.h>

#include "pmu_common.h"

PMU_t* pmu_probe(PMU_Interface_t* pmu_if);

// static void pmu_print_status(Power_Status_t status)
// {
//     NRF_LOG_INFO("=== Power_Status_t ===\n");
//     NRF_LOG_INFO("isValid=%u\n",status.isValid);
//     NRF_LOG_INFO("batteryPresent=%u\n",status.batteryPresent);
//     NRF_LOG_INFO("batteryPercent=%u\n",status.batteryPercent);
//     NRF_LOG_INFO("batteryVoltage=%lu\n",status.batteryVoltage);
//     NRF_LOG_INFO("batteryTemp=%ld\n",status.batteryTemp);
//     NRF_LOG_INFO("pmuTemp=%lu\n",status.pmuTemp);
//     NRF_LOG_INFO("chargeAllowed=%u\n",status.chargeAllowed);
//     NRF_LOG_INFO("chargerAvailable=%u\n",status.chargerAvailable);
//     NRF_LOG_INFO("chargeFinished=%u\n",status.chargeFinished);
//     NRF_LOG_INFO("wiredCharge=%u\n",status.wiredCharge);
//     NRF_LOG_INFO("wirelessCharge=%u\n",status.wirelessCharge);
//     NRF_LOG_INFO("chargeCurrent=%lu\n",status.chargeCurrent);
//     NRF_LOG_INFO("dischargeCurrent=%lu\n",status.dischargeCurrent);
//     NRF_LOG_INFO("=== ============== ===\n");
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