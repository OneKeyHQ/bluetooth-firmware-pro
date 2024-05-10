#include "pmu.h"

#include "axp2101.h"
#include "axp216.h"

static PMU_t pmu = {0};

PMU_t* pmu_probe(PMU_Interface_t* pmu_if)
{

    // axp216 probe
    axp216_setup_interface(pmu_if, &pmu);
    if ( pmu.Init() == PWR_ERROR_NONE )
        return &pmu;

    // axp2101 probe
    axp2101_setup_interface(pmu_if, &pmu);
    if ( pmu.Init() == PWR_ERROR_NONE )
        return &pmu;

    return NULL;
}
