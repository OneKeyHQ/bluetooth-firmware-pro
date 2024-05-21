#ifndef _AXP216_CONF_H_
#define _AXP216_CONF_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AXP216_CONF_INVALID = -1,
    AXP216_CONF_BUS_ERR,
    AXP216_CONF_NO_ACK,
    AXP216_CONF_NOT_NEEDED,
    AXP216_CONF_SUCCESS,
    AXP216_CONF_FAILED,
} AXP216_CONF_R_t;

AXP216_CONF_R_t axp216_minimum_config();

#endif // _AXP216_CONF_H_
