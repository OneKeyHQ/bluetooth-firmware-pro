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
project(OnekeyProBTFW LANGUAGES C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###############################
# Sub Projects

include(ExternalProject)

ExternalProject_Add(
  dfu
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/dfu
  INSTALL_DIR ${PROJECT_BINARY_DIR}/artifacts
  CMAKE_GENERATOR "Ninja"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX:STRING=${PROJECT_SOURCE_DIR}/artifacts"
)
ExternalProject_Add(
  app
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/app
  INSTALL_DIR ${PROJECT_BINARY_DIR}/artifacts
  CMAKE_GENERATOR "Ninja"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX:STRING=${PROJECT_SOURCE_DIR}/artifacts"
)

###############################
# Pack and Sigh

# populate vars
set(DIR_UTILS "${PROJECT_SOURCE_DIR}/utils")
set(FILE_KEY "${PROJECT_BINARY_DIR}/temp.pk")
set(FILE_INPUT_BL_CONFIG "bl_config.hex")
set(FILE_INPUT_BOOTLOADER "${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_BL.hex")
set(FILE_INPUT_APP "${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_APP.hex")
set(FILE_INPUT_SD "${PROJECT_SOURCE_DIR}/ble-firmware/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex")
set(DIR_OUTPUT "${PROJECT_SOURCE_DIR}/artifacts_signed")
set(FILE_OUTPUT_FACTORY_HEX "${DIR_OUTPUT}/factory.hex")
set(FILE_OUTPUT_FACTORY_BIN "${DIR_OUTPUT}/factory.bin")
set(FILE_OUTPUT_ZIP "${DIR_OUTPUT}/ota.zip")
set(FILE_OUTPUT_OTA_BIN "${DIR_OUTPUT}/ota.bin")

# prepare output dir
# as nrfutils won't cerate folders, this is required
add_custom_target(
  ${CMAKE_PROJECT_NAME}_OUT_DIR ALL
  COMMAND mkdir -p ${DIR_OUTPUT}
)

# prepare nrf tools
add_custom_target(
  ${CMAKE_PROJECT_NAME}_UTIL
  COMMAND ${DIR_UTILS}/nrfutil install nrf5sdk-tools
)

# prepare temp key
add_custom_target(
  ${CMAKE_PROJECT_NAME}_KEY
  COMMAND cp "$ENV{BT_SIG_PK_FILE}" ${FILE_KEY}
)

# gen bl config
add_custom_target(
  ${CMAKE_PROJECT_NAME}_BL_CONF ALL
  COMMAND ${DIR_UTILS}/nrfutil 
          settings generate --family NRF52 --bootloader-version 2 --bl-settings-version 2 
          --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --application ${FILE_INPUT_APP} --application-version 3 
          --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ${FILE_INPUT_SD} 
          --key-file ${FILE_KEY}  ${FILE_INPUT_BL_CONFIG}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_KEY app
)

# megre factory hex
add_custom_target(
  ${CMAKE_PROJECT_NAME}_FACTORY_HEX ALL
  COMMAND ${DIR_UTILS}/mergehex 
                      --merge ${FILE_INPUT_BOOTLOADER} ${FILE_INPUT_BL_CONFIG} ${FILE_INPUT_SD} ${FILE_INPUT_APP} 
                      --output ${FILE_OUTPUT_FACTORY_HEX}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_BL_CONF dfu app
)

# gen factory bin
add_custom_target(
  ${CMAKE_PROJECT_NAME}_FACTORY_BIN ALL
  COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary --gap-fill=0xFF ${FILE_OUTPUT_FACTORY_HEX} ${FILE_OUTPUT_FACTORY_BIN}.tmp
  COMMAND dd if=${FILE_OUTPUT_FACTORY_BIN}.tmp of=${FILE_OUTPUT_FACTORY_BIN} bs=512K count=1
  COMMAND rm ${FILE_OUTPUT_FACTORY_BIN}.tmp 
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_FACTORY_HEX
)

# gen ota bin
add_custom_target(
  ${CMAKE_PROJECT_NAME}_OTA_BIN ALL
  COMMAND ${DIR_UTILS}/nrfutil pkg generate 
          --hw-version 52 --sd-req 0xCB 
          --application ${FILE_INPUT_APP} --application-version 3 --app-boot-validation VALIDATE_ECDSA_P256_SHA256
          --key-file ${FILE_KEY} ${FILE_OUTPUT_ZIP}
  COMMAND ${DIR_UTILS}/ota_to_onekey_bin.py ${FILE_OUTPUT_ZIP} ${FILE_OUTPUT_OTA_BIN}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_KEY dfu app
)

###############################
# development related

# flash
add_custom_target(
  ${CMAKE_PROJECT_NAME}_FLASH_FACTORY
  COMMAND JLINK_SN=$ENV{JLINK_SN} ${PROJECT_SOURCE_DIR}/flash_factory.sh
  DEPENDS ${CMAKE_PROJECT_NAME}_FACTORY_HEX
)

###############################
# Install
# install(
#   FILES 
#   xxx.bin
#   DESTINATION ${CMAKE_INSTALL_PREFIX}
# )