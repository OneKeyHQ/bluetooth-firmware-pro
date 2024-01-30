# cmake options
cmake_minimum_required(VERSION 3.22.1)

###############################
# Toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY") 
set(TOOL_CHAIN_PREFIX "arm-none-eabi")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
find_program(CMAKE_C_COMPILER NAMES ${TOOL_CHAIN_PREFIX}-gcc)
find_program(CMAKE_ASM_COMPILER NAMES ${TOOL_CHAIN_PREFIX}-gcc)
find_program(CMAKE_OBJCOPY NAMES ${TOOL_CHAIN_PREFIX}-objcopy)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

###############################
# Project
project(OnekeyProBTFW_APP LANGUAGES C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###############################
# NRF SDK
set(NRF_SDK_ROOT "${PROJECT_SOURCE_DIR}/../ble-firmware")

set(
  NRF_SDK_INC
  ${NRF_SDK_ROOT}/components
  ${NRF_SDK_ROOT}/components/ble/ble_advertising
  ${NRF_SDK_ROOT}/components/ble/ble_dtm
  ${NRF_SDK_ROOT}/components/ble/ble_link_ctx_manager
  ${NRF_SDK_ROOT}/components/ble/ble_racp
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ancs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ans_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_cscs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_cts_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dis
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_gls
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hids
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hrs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hrs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_hts
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ias
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_ias_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lls
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_rscs
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_rscs_c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_tps
  ${NRF_SDK_ROOT}/components/ble/common
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gatt
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gq
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_qwr
  ${NRF_SDK_ROOT}/components/ble/peer_manager
  # ${NRF_SDK_ROOT}/components/nfc/ndef/conn_hand_parser
  # ${NRF_SDK_ROOT}/components/nfc/ndef/conn_hand_parser/ac_rec_parser
  # ${NRF_SDK_ROOT}/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser
  # ${NRF_SDK_ROOT}/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/ac_rec
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/ble_oob_advdata
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_lib
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/ble_pair_msg
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/common
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/ep_oob_rec
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/hs_rec
  # ${NRF_SDK_ROOT}/components/nfc/ndef/connection_handover/le_oob_rec
  # ${NRF_SDK_ROOT}/components/nfc/ndef/generic/message
  # ${NRF_SDK_ROOT}/components/nfc/ndef/generic/record
  # ${NRF_SDK_ROOT}/components/nfc/ndef/launchapp
  # ${NRF_SDK_ROOT}/components/nfc/ndef/parser/message
  # ${NRF_SDK_ROOT}/components/nfc/ndef/parser/record
  # ${NRF_SDK_ROOT}/components/nfc/ndef/text
  # ${NRF_SDK_ROOT}/components/nfc/ndef/uri
  # ${NRF_SDK_ROOT}/components/nfc/platform
  # ${NRF_SDK_ROOT}/components/nfc/t2t_lib
  # ${NRF_SDK_ROOT}/components/nfc/t2t_parser
  # ${NRF_SDK_ROOT}/components/nfc/t4t_lib
  # ${NRF_SDK_ROOT}/components/nfc/t4t_parser/apdu
  # ${NRF_SDK_ROOT}/components/nfc/t4t_parser/cc_file
  # ${NRF_SDK_ROOT}/components/nfc/t4t_parser/hl_detection_procedure
  # ${NRF_SDK_ROOT}/components/nfc/t4t_parser/tlv
  ${NRF_SDK_ROOT}/components/boards
  ${NRF_SDK_ROOT}/components/libraries/atomic
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo
  ${NRF_SDK_ROOT}/components/libraries/atomic_flags
  ${NRF_SDK_ROOT}/components/libraries/balloc
  ${NRF_SDK_ROOT}/components/libraries/bootloader
  ${NRF_SDK_ROOT}/components/libraries/bootloader/ble_dfu
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu
  ${NRF_SDK_ROOT}/components/libraries/bsp
  ${NRF_SDK_ROOT}/components/libraries/button
  ${NRF_SDK_ROOT}/components/libraries/cli
  ${NRF_SDK_ROOT}/components/libraries/crc16
  ${NRF_SDK_ROOT}/components/libraries/crc32
  ${NRF_SDK_ROOT}/components/libraries/crypto
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cc310
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cc310_bl
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/cifra
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/mbedtls
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_sw
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/optiga
  ${NRF_SDK_ROOT}/components/libraries/csense
  ${NRF_SDK_ROOT}/components/libraries/csense_drv
  ${NRF_SDK_ROOT}/components/libraries/delay
  ${NRF_SDK_ROOT}/components/libraries/ecc
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars
  ${NRF_SDK_ROOT}/components/libraries/experimental_task_manager
  ${NRF_SDK_ROOT}/components/libraries/fds
  ${NRF_SDK_ROOT}/components/libraries/fifo
  ${NRF_SDK_ROOT}/components/libraries/fstorage
  ${NRF_SDK_ROOT}/components/libraries/gfx
  ${NRF_SDK_ROOT}/components/libraries/gpiote
  ${NRF_SDK_ROOT}/components/libraries/hardfault
  ${NRF_SDK_ROOT}/components/libraries/hci
  ${NRF_SDK_ROOT}/components/libraries/led_softblink
  ${NRF_SDK_ROOT}/components/libraries/log
  ${NRF_SDK_ROOT}/components/libraries/log/src
  ${NRF_SDK_ROOT}/components/libraries/low_power_pwm
  ${NRF_SDK_ROOT}/components/libraries/mem_manager
  ${NRF_SDK_ROOT}/components/libraries/memobj
  ${NRF_SDK_ROOT}/components/libraries/mpu
  ${NRF_SDK_ROOT}/components/libraries/mutex
  ${NRF_SDK_ROOT}/components/libraries/pwm
  ${NRF_SDK_ROOT}/components/libraries/pwr_mgmt
  ${NRF_SDK_ROOT}/components/libraries/queue
  ${NRF_SDK_ROOT}/components/libraries/ringbuf
  ${NRF_SDK_ROOT}/components/libraries/scheduler
  ${NRF_SDK_ROOT}/components/libraries/sdcard
  ${NRF_SDK_ROOT}/components/libraries/sensorsim
  ${NRF_SDK_ROOT}/components/libraries/slip
  ${NRF_SDK_ROOT}/components/libraries/sortlist
  ${NRF_SDK_ROOT}/components/libraries/spi_mngr
  ${NRF_SDK_ROOT}/components/libraries/stack_guard
  ${NRF_SDK_ROOT}/components/libraries/stack_info
  ${NRF_SDK_ROOT}/components/libraries/strerror
  ${NRF_SDK_ROOT}/components/libraries/svc
  ${NRF_SDK_ROOT}/components/libraries/timer
  ${NRF_SDK_ROOT}/components/libraries/twi_mngr
  ${NRF_SDK_ROOT}/components/libraries/twi_sensor
  ${NRF_SDK_ROOT}/components/libraries/uart
  ${NRF_SDK_ROOT}/components/libraries/usbd
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/audio
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/cdc
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/cdc/acm
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/generic
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/kbd
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/hid/mouse
  ${NRF_SDK_ROOT}/components/libraries/usbd/class/msc
  ${NRF_SDK_ROOT}/components/libraries/util
  ${NRF_SDK_ROOT}/components/softdevice/common
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers/nrf52
  ${NRF_SDK_ROOT}/components/toolchain/cmsis/include
  ${NRF_SDK_ROOT}/external/fprintf
  ${NRF_SDK_ROOT}/external/mbedtls/include
  ${NRF_SDK_ROOT}/external/micro-ecc/micro-ecc
  ${NRF_SDK_ROOT}/external/nrf_cc310/include
  ${NRF_SDK_ROOT}/external/nrf_oberon
  ${NRF_SDK_ROOT}/external/nrf_oberon/include
  ${NRF_SDK_ROOT}/external/nrf_tls/mbedtls/nrf_crypto/config
  ${NRF_SDK_ROOT}/external/segger_rtt
  ${NRF_SDK_ROOT}/external/utf_converter
  ${NRF_SDK_ROOT}/integration/nrfx
  ${NRF_SDK_ROOT}/integration/nrfx/legacy
  ${NRF_SDK_ROOT}/modules/nrfx
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/include
  ${NRF_SDK_ROOT}/modules/nrfx/hal
  ${NRF_SDK_ROOT}/modules/nrfx/mdk
)

set(
  NRF_SDK_SRC
  ${NRF_SDK_ROOT}/components/ble/ble_advertising/ble_advertising.c
  ${NRF_SDK_ROOT}/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c
  ${NRF_SDK_ROOT}/components/ble/ble_racp/ble_racp.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_bas/ble_bas.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_bonded.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_unbonded.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_dis/ble_dis.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_lbs/ble_lbs.c
  ${NRF_SDK_ROOT}/components/ble/ble_services/ble_nus/ble_nus.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_advdata.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_conn_params.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_conn_state.c
  ${NRF_SDK_ROOT}/components/ble/common/ble_srv_common.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gatt/nrf_ble_gatt.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_gq/nrf_ble_gq.c
  ${NRF_SDK_ROOT}/components/ble/nrf_ble_qwr/nrf_ble_qwr.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/auth_status_tracker.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/gatt_cache_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/gatts_cache_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/id_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/nrf_ble_lesc.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_data_storage.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_database.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_id.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_manager.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/peer_manager_handler.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/pm_buffer.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/security_dispatcher.c
  ${NRF_SDK_ROOT}/components/ble/peer_manager/security_manager.c
  ${NRF_SDK_ROOT}/components/boards/boards.c
  ${NRF_SDK_ROOT}/components/libraries/atomic/nrf_atomic.c
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c
  ${NRF_SDK_ROOT}/components/libraries/atomic_flags/nrf_atflags.c
  ${NRF_SDK_ROOT}/components/libraries/balloc/nrf_balloc.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_svci.c
  ${NRF_SDK_ROOT}/components/libraries/bsp/bsp.c
  ${NRF_SDK_ROOT}/components/libraries/bsp/bsp_btn_ble.c
  ${NRF_SDK_ROOT}/components/libraries/button/app_button.c
  ${NRF_SDK_ROOT}/components/libraries/crc16/crc16.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_init.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_hw/nrf_hw_backend_rng_mbedtls.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_chacha_poly_aead.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_eddsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hmac.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aead.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_aes_shared.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_eddsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_error.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hkdf.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hmac.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_init.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_rng.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_shared.c
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c
  ${NRF_SDK_ROOT}/components/libraries/fds/fds.c
  ${NRF_SDK_ROOT}/components/libraries/fifo/app_fifo.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_sd.c
  ${NRF_SDK_ROOT}/components/libraries/hardfault/hardfault_implementation.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_backend_rtt.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_backend_serial.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_backend_uart.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_default_backends.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c
  ${NRF_SDK_ROOT}/components/libraries/mem_manager/mem_manager.c
  ${NRF_SDK_ROOT}/components/libraries/memobj/nrf_memobj.c
  ${NRF_SDK_ROOT}/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c
  ${NRF_SDK_ROOT}/components/libraries/queue/nrf_queue.c
  ${NRF_SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c
  ${NRF_SDK_ROOT}/components/libraries/scheduler/app_scheduler.c
  ${NRF_SDK_ROOT}/components/libraries/sortlist/nrf_sortlist.c
  ${NRF_SDK_ROOT}/components/libraries/strerror/nrf_strerror.c
  ${NRF_SDK_ROOT}/components/libraries/timer/app_timer2.c
  ${NRF_SDK_ROOT}/components/libraries/timer/drv_rtc.c
  ${NRF_SDK_ROOT}/components/libraries/uart/app_uart_fifo.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error_weak.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_util_platform.c
  ${NRF_SDK_ROOT}/components/libraries/util/nrf_assert.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf_format.c
  ${NRF_SDK_ROOT}/external/mbedtls/library/aes.c
  ${NRF_SDK_ROOT}/external/mbedtls/library/ctr_drbg.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c
  ${NRF_SDK_ROOT}/external/segger_rtt/SEGGER_RTT_printf.c
  ${NRF_SDK_ROOT}/external/utf_converter/utf.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_clock.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_rng.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_spi.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_twi.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_uart.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_clock.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rng.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_rtc.c
  # ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_nfct.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_saadc.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_spim.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_timer.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_twi.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_twim.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uart.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_wdt.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52.S
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/system_nrf52.c
  ${NRF_SDK_ROOT}/modules/nrfx/soc/nrfx_atomic.c
)

set(NRF_SDK_LINK
  ${NRF_SDK_ROOT}/external/nrf_cc310/lib/cortex-m4/hard-float/libnrf_cc310_0.9.12.a
  ${NRF_SDK_ROOT}/external/nrf_oberon/lib/cortex-m4/hard-float/liboberon_3.0.1.a
  # ${NRF_SDK_ROOT}/external/micro-ecc/nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
  # ${NRF_SDK_ROOT}/components/nfc/t4t_lib/nfc_t4t_lib_gcc.a
)

###############################
# micro-ecc

set(MicroECC_DIR ${NRF_SDK_ROOT}/external/micro-ecc)

add_library(
  MicroECC STATIC
  ${MicroECC_DIR}/micro-ecc/uECC.c
)

target_include_directories(
  MicroECC PUBLIC
  ${MicroECC_DIR}/micro-ecc
  ${NRF_SDK_ROOT}/components/toolchain/cmsis/include
)

target_compile_options(
  MicroECC PRIVATE
  # Optimization flags
  -Os -g3
  # Uncomment the line below to enable link time optimization
  # -flto
  # C flags common to all targets
  -DFLOAT_ABI_HARD
  -DuECC_ENABLE_VLI_API=0
  -DuECC_OPTIMIZATION_LEVEL=3
  -DuECC_SQUARE_FUNC=0
  -DuECC_SUPPORT_COMPRESSED_POINT=0
  -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  # -Wall -Werror
  -Wall -Werror
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # keep every function in a separate section, this allows linker to discard unused ones
  -ffunction-sections -fdata-sections -fno-strict-aliasing
  -fno-builtin -fshort-enums -Wno-unused-function
  # heap and stack
  -D__HEAP_SIZE=4096
  -D__STACK_SIZE=4096
)

target_link_options(
  MicroECC PRIVATE
  # Linker flags
  -Os -g3
  -mthumb -mabi=aapcs -L${NRF_SDK_ROOT}/modules/nrfx/mdk -T${MicroECC_DIR}/nrf52hf_armgcc/armgcc/ext_micro_ecc_gcc_nRF5x.ld
  -mcpu=cortex-m4
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # let linker dump unused sections
  -Wl,--gc-sections
  # use newlib in nano version
  -lc -lnosys -lm
)

###############################
# Target

add_executable(
  ${CMAKE_PROJECT_NAME} 
  main.c
)

target_sources(
  ${CMAKE_PROJECT_NAME} PRIVATE 
  data_transmission.c
  driver_3rdparty/axp/axp_config.c
  driver_3rdparty/axp/axp_supply.c
  driver_3rdparty/lm/lm_config.c
  power_manage.c
  flashled_manage.c
  rtc_calendar.c
  ecdsa.c
  # nfc.c
  ${NRF_SDK_SRC}
)

target_include_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE
  driver_3rdparty/axp
  driver_3rdparty/lm
  ${NRF_SDK_INC}
)

target_compile_options(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # Optimization flags
  -O3 -g3
  # Uncomment the line below to enable link time optimization
  -flto
  # C flags common to all targets
  -DAPP_TIMER_V2
  -DAPP_TIMER_V2_RTC1_ENABLED
  -DBOARD_CUSTOM
  -DCONFIG_GPIO_AS_PINRESET
  -DFLOAT_ABI_HARD
  -DNRF52
  -DNRF52832_XXAA
  -DNRF52_PAN_74
  -DNRF_SD_BLE_API_VERSION=7
  -DS132
  -DSOFTDEVICE_PRESENT
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  # project flags
  # -DBOARD_PCA10040
  -DBUTTONLESS_ENABLED
  -DUART_TRANS
  -DBOND_ENABLE
  -DNRF_DFU_TRANSPORT_BLE
  -DMBEDTLS_CONFIG_FILE="nrf_crypto_mbedtls_config.h"
  -DNRF_APP_VERSION=0x00000001
  -DNRF_APP_VERSION_ADDR=0x1D000
  -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
  #  -Wall -Werror
  -Wall
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # keep every function in a separate section, this allows linker to discard unused ones
  -ffunction-sections -fdata-sections -fno-strict-aliasing
  -fno-builtin -fshort-enums
  # heap and stack
  -D__HEAP_SIZE=8192
  -D__STACK_SIZE=8192
)

target_link_options(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # Linker flags
  -O3 -g3
  -mthumb -mabi=aapcs -L${NRF_SDK_ROOT}/modules/nrfx/mdk -T${PROJECT_SOURCE_DIR}/ble_app_gcc_nrf52.ld
  -mcpu=cortex-m4
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # let linker dump unused sections
  -Wl,--gc-sections
  # use newlib in nano version
  --specs=nano.specs -lc -lnosys -lm
)

target_link_libraries(
  ${CMAKE_PROJECT_NAME} PRIVATE
  ${NRF_SDK_LINK}
  MicroECC
)

###############################
# bin and hex
add_custom_target(
  ${CMAKE_PROJECT_NAME}_bin ALL
  COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin
  DEPENDS ${CMAKE_PROJECT_NAME}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)

add_custom_target(
  ${CMAKE_PROJECT_NAME}_hex ALL
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.hex
  DEPENDS ${CMAKE_PROJECT_NAME}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)

###############################
# development related

# sdk config
add_custom_target(
  "sdk_config"
  COMMAND java -jar ${NRF_SDK_ROOT}/external_tools/CMSIS_Configuration_Wizard.jar sdk_config.h
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

###############################
# Install
install(
  FILES 
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
  # DESTINATION ${PROJECT_SOURCE_DIR}/../artifacts
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)