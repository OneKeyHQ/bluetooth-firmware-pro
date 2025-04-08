#ifndef FIRMWARE_CONFIG
#define FIRMWARE_CONFIG

#define MANUFACTURER_NAME "OneKey" /**< Manufacturer. Will be passed to Device Information Service. */
#define ADV_HEAD_NAME     "Pro "
#define MODEL_NUMBER      "Pro" /**< Model Number string. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                                                                                             \
    0x55AA55AA55 /**< DUMMY Manufacturer ID. Will be passed to Device Information Service. You shall use the ID for \
                    your Company*/
#define ORG_UNIQUE_ID                                                                                           \
    0xEEBBEE /**< DUMMY Organisation Unique ID. Will be passed to Device Information Service. You shall use the \
                Organisation Unique ID relevant for your Company */
#define HW_REVISION "1.0.0"
#define FW_REVISION "2.3.4"
#define SW_REVISION "s132_nrf52_7.0.1"
#define BT_REVISION "1.0.1"

#endif // FIRMWARE_CONFIG
