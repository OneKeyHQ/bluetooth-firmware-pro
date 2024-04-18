#include "pmu.h"

#include "axp216.h"
#include "axp2101.h"

static PMU_Interface_t pmu_interface = {0};
static PMU_t pmu = {0};

PMU_t* pmu_probe(PMU_Interface_t* pmu_if)
{
    // copy instead of reference as the outside obj may be destroyed
    memcpy(&pmu_interface, pmu_if, sizeof(PMU_Interface_t));
    
    PMU_t* ret = NULL;

    // axp216 probe
    axp216_setup_interface(&pmu_interface, &pmu);
    if ( pmu.Init() == PWR_ERROR_NONE )
        ret = &pmu;

    // axp2101 probe
    // axp2101_setup_interface(&pmu_interface, &pmu);
    // if ( pmu.Init() == PWR_ERROR_NONE )
    // {
    //     ret = &pmu;
    // }

    return ret;
}
