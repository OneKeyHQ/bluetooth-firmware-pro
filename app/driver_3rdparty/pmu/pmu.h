#ifndef __PMU_H_
#define __PMU_H_

#include <stdio.h>

#include "pmu_common.h"

PMU_t* pmu_probe(PMU_Interface_t* pmu_if);

#endif //__PMU_H_