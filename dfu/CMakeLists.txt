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
project(OnekeyProBTFW_BL LANGUAGES C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###############################
# NRF SDK
set(NRF_SDK_ROOT "${PROJECT_SOURCE_DIR}/../ble-firmware")

set(
  NRF_SDK_INC
  ${NRF_SDK_ROOT}/components/ble/common
  ${NRF_SDK_ROOT}/components/boards
  ${NRF_SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd
  ${NRF_SDK_ROOT}/components/libraries/atomic
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo
  ${NRF_SDK_ROOT}/components/libraries/balloc
  ${NRF_SDK_ROOT}/components/libraries/bootloader
  ${NRF_SDK_ROOT}/components/libraries/bootloader/ble_dfu
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu
  ${NRF_SDK_ROOT}/components/libraries/bootloader/serial_dfu
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
  ${NRF_SDK_ROOT}/components/libraries/delay
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars
  ${NRF_SDK_ROOT}/components/libraries/fstorage
  ${NRF_SDK_ROOT}/components/libraries/log
  ${NRF_SDK_ROOT}/components/libraries/log/src
  ${NRF_SDK_ROOT}/components/libraries/mem_manager
  ${NRF_SDK_ROOT}/components/libraries/memobj
  ${NRF_SDK_ROOT}/components/libraries/queue
  ${NRF_SDK_ROOT}/components/libraries/ringbuf
  ${NRF_SDK_ROOT}/components/libraries/scheduler
  ${NRF_SDK_ROOT}/components/libraries/sha256
  ${NRF_SDK_ROOT}/components/libraries/slip
  ${NRF_SDK_ROOT}/components/libraries/stack_info
  ${NRF_SDK_ROOT}/components/libraries/strerror
  ${NRF_SDK_ROOT}/components/libraries/svc
  ${NRF_SDK_ROOT}/components/libraries/util
  ${NRF_SDK_ROOT}/components/softdevice/common
  ${NRF_SDK_ROOT}/components/softdevice/mbr/headers
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers
  ${NRF_SDK_ROOT}/components/softdevice/s132/headers/nrf52
  ${NRF_SDK_ROOT}/components/toolchain/cmsis/include
  ${NRF_SDK_ROOT}/external/fprintf
  # ${NRF_SDK_ROOT}/external/micro-ecc/micro-ecc
  ${NRF_SDK_ROOT}/external/nano-pb
  ${NRF_SDK_ROOT}/external/nrf_oberon
  ${NRF_SDK_ROOT}/external/nrf_oberon/include
  ${NRF_SDK_ROOT}/integration/nrfx
  ${NRF_SDK_ROOT}/integration/nrfx/legacy
  ${NRF_SDK_ROOT}/modules/nrfx
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/include
  ${NRF_SDK_ROOT}/modules/nrfx/hal
  ${NRF_SDK_ROOT}/modules/nrfx/mdk
)

set(
  NRF_SDK_SRC
  ${NRF_SDK_ROOT}/components/ble/common/ble_srv_common.c
  ${NRF_SDK_ROOT}/components/boards/boards.c
  ${NRF_SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c
  ${NRF_SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c
  ${NRF_SDK_ROOT}/components/libraries/atomic/nrf_atomic.c
  ${NRF_SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c
  ${NRF_SDK_ROOT}/components/libraries/balloc/nrf_balloc.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/ble_dfu/nrf_dfu_ble.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/dfu-cc.pb.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_flash.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_handling_error.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_mbr.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_req_handler.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_settings.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_settings_svci.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_svci.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_svci_handler.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_transport.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_utils.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_validation.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_ver_validation.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_app_start.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_app_start_final.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_dfu_timers.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_fw_activation.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_info.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/nrf_bootloader_wdt.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/serial_dfu/nrf_dfu_serial.c
  ${NRF_SDK_ROOT}/components/libraries/bootloader/serial_dfu/nrf_dfu_serial_uart.c
  ${NRF_SDK_ROOT}/components/libraries/crc32/crc32.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/micro_ecc/micro_ecc_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/nrf_sw/nrf_sw_backend_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_chacha_poly_aead.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdh.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_eddsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/backend/oberon/oberon_backend_hmac.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecc.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_ecdsa.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_hash.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_init.c
  ${NRF_SDK_ROOT}/components/libraries/crypto/nrf_crypto_shared.c
  ${NRF_SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_nvmc.c
  ${NRF_SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_sd.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c
  ${NRF_SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c
  ${NRF_SDK_ROOT}/components/libraries/mem_manager/mem_manager.c
  ${NRF_SDK_ROOT}/components/libraries/memobj/nrf_memobj.c
  ${NRF_SDK_ROOT}/components/libraries/queue/nrf_queue.c
  ${NRF_SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c
  ${NRF_SDK_ROOT}/components/libraries/scheduler/app_scheduler.c
  ${NRF_SDK_ROOT}/components/libraries/sha256/sha256.c
  ${NRF_SDK_ROOT}/components/libraries/slip/slip.c
  ${NRF_SDK_ROOT}/components/libraries/strerror/nrf_strerror.c
  ${NRF_SDK_ROOT}/components/libraries/svc/nrf_svc_handler.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_error_weak.c
  ${NRF_SDK_ROOT}/components/libraries/util/app_util_platform.c
  ${NRF_SDK_ROOT}/components/libraries/util/nrf_assert.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c
  ${NRF_SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf.c
  ${NRF_SDK_ROOT}/external/fprintf/nrf_fprintf_format.c
  ${NRF_SDK_ROOT}/external/nano-pb/pb_common.c
  ${NRF_SDK_ROOT}/external/nano-pb/pb_decode.c
  ${NRF_SDK_ROOT}/integration/nrfx/legacy/nrf_drv_uart.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uart.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c
  ${NRF_SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c
  ${NRF_SDK_ROOT}/modules/nrfx/hal/nrf_nvmc.c
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52.S
  ${NRF_SDK_ROOT}/modules/nrfx/mdk/system_nrf52.c
  ${NRF_SDK_ROOT}/modules/nrfx/soc/nrfx_atomic.c
  # ${NRF_SDK_ROOT}/external/micro-ecc/micro-ecc/uECC.c
)

set(NRF_SDK_LINK
  ${NRF_SDK_ROOT}/external/nrf_oberon/lib/cortex-m4/hard-float/liboberon_3.0.1.a
  # ${NRF_SDK_ROOT}/external/micro-ecc/nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
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
  dfu_public_key.c
)

target_sources(
  ${CMAKE_PROJECT_NAME} PRIVATE 
  ${NRF_SDK_SRC}
)

target_include_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${NRF_SDK_INC}
)

target_compile_options(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # Optimization flags
  -Os -g3 -s
  # Uncomment the line below to enable link time optimization
  -flto
  # C flags common to all targets
  -DBLE_STACK_SUPPORT_REQD
  -DCONFIG_GPIO_AS_PINRESET
  -DFLOAT_ABI_HARD
  -DMBR_PRESENT
  -DNRF52
  -DNRF52832_XXAA
  -DNRF52_PAN_74
  -DNRF_DFU_DEBUG_VERSION
  -DNRF_DFU_SETTINGS_VERSION=2
  -DNRF_DFU_SVCI_ENABLED
  -DNRF_SD_BLE_API_VERSION=7
  -DS132
  -DSOFTDEVICE_PRESENT
  -mcpu=cortex-m4
  -mthumb -mabi=aapcs
  # project flags
  -DBOARD_PCA10040
  -DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION
  -DuECC_ENABLE_VLI_API=0
  -DuECC_OPTIMIZATION_LEVEL=3
  -DuECC_SQUARE_FUNC=0
  -DuECC_SUPPORT_COMPRESSED_POINT=0
  -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1
  # -Wall -Werror
  -Wall
  -mfloat-abi=hard -mfpu=fpv4-sp-d16
  # keep every function in a separate section, this allows linker to discard unused ones
  -ffunction-sections -fdata-sections -fno-strict-aliasing
  -fno-builtin -fshort-enums
  # heap and stack
  -D__HEAP_SIZE=0
)

target_link_options(
  ${CMAKE_PROJECT_NAME} PRIVATE
  # Linker flags
  -O3 -g3 -s
  -mthumb -mabi=aapcs -L${NRF_SDK_ROOT}/modules/nrfx/mdk -T${PROJECT_SOURCE_DIR}/secure_bootloader_gcc_nrf52.ld
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
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin 
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
  ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}
  # DESTINATION ${PROJECT_SOURCE_DIR}/../artifacts
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)