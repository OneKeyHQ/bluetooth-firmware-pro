#ifndef __PMU_H_
#define __PMU_H_

#include <stdio.h>

#include "pmu_common.h"

PMU_t* pmu_probe(PMU_Interface_t* pmu_if);


// static void pmu_print_status(Power_Status_t status)
// {
//     NRF_LOG_INFO("=== Power_Status_t ===\n");
//     NRF_LOG_INFO("isValid=%u\n",status.isValid);
//     NRF_LOG_INFO("batteryPercent=%u\n",status.batteryPercent);
//     NRF_LOG_INFO("batteryVoltage=%lu\n",status.batteryVoltage);
//     NRF_LOG_INFO("batteryTemp=%lu\n",status.batteryTemp);
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

#endif //__PMU_H_