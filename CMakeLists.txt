cmake_minimum_required(VERSION 3.22.1)

###############################
# Toolchain
include("./toolchain.cmake")


###############################
# Project
project(OnekeyProBTFW LANGUAGES C)
set(CMAKE_INCLUDE_CURRENT_DIR OFF)

# populate vars
set(DIR_UTILS "${PROJECT_SOURCE_DIR}/utils")
set(FILE_KEY "${PROJECT_BINARY_DIR}/temp.pk")
set(FILE_INPUT_BL_CONFIG "bl_config.hex")
set(FILE_INPUT_MBR_UICR "${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_MBR_UICR.hex")
set(FILE_INPUT_BOOTLOADER "${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_BL.hex")
set(FILE_INPUT_APP "${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_APP.hex")
set(FILE_INPUT_SD "${PROJECT_SOURCE_DIR}/ble-firmware/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex")
set(DIR_OUTPUT "${PROJECT_SOURCE_DIR}/artifacts_signed")
set(FILE_OUTPUT_FACTORY_HEX "${DIR_OUTPUT}/factory.hex")
set(FILE_OUTPUT_FACTORY_BIN "${DIR_OUTPUT}/factory.bin")
set(FILE_OUTPUT_ZIP "${DIR_OUTPUT}/ota.zip")
set(FILE_OUTPUT_OTA_BIN "${DIR_OUTPUT}/ota.bin")
set(PYTHON_RUNNER exit -1 && ) # placeholder, will be set later 

###############################
# Prerequests and Environments

# output dir
# as nrfutils won't cerate folders, this is required
add_custom_target(
  ${CMAKE_PROJECT_NAME}_OUT_DIR ALL
  COMMAND mkdir -p ${DIR_OUTPUT}
)

# temp key
add_custom_target(
  ${CMAKE_PROJECT_NAME}_KEY
  COMMAND cp "$ENV{BT_SIG_PK_FILE}" ${FILE_KEY}
)

# nrf tools
add_custom_target(
  ${CMAKE_PROJECT_NAME}_UTIL
  COMMAND ${DIR_UTILS}/nrfutil install nrf5sdk-tools
)

# python env (poetry)
# set(PYTHON_RUNNER poetry -C ${DIR_UTILS} run)
# add_custom_target(
#   ${CMAKE_PROJECT_NAME}_PYENV
#   COMMAND ${PYTHON_RUNNER} install
#   COMMAND ${PYTHON_RUNNER} env info
# )

# python env (pip)
set(PYTHON_RUNNER ${PROJECT_BINARY_DIR}/venv/bin/python)
add_custom_target(
  ${CMAKE_PROJECT_NAME}_PYENV
  COMMAND python -m venv ${PROJECT_BINARY_DIR}/venv
  COMMAND ${PROJECT_BINARY_DIR}/venv/bin/pip install -r ${DIR_UTILS}/requirements.txt
)

###############################
# Targets

include(ExternalProject)

# dfu
ExternalProject_Add(
  dfu
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/dfu
  INSTALL_DIR ${PROJECT_SOURCE_DIR}/artifacts
  CMAKE_GENERATOR "Ninja"
  CMAKE_ARGS "-DTOOL_CHAIN_PREFIX=${TOOL_CHAIN_PREFIX} "
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX:STRING=${PROJECT_SOURCE_DIR}/artifacts " 
)

# dfu payload
add_custom_target(
  ${CMAKE_PROJECT_NAME}_DFU_UPGRADE_PAYLOAD ALL
  COMMAND ${PYTHON_RUNNER} ${DIR_UTILS}/bin2array.py -l 16 -i ${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_BL.bin -o ${PROJECT_SOURCE_DIR}/artifacts/dfu_upgrade_payload.h
  DEPENDS dfu ${CMAKE_PROJECT_NAME}_PYENV
)

# mbr and uicr config
add_custom_target(
  ${CMAKE_PROJECT_NAME}_MBR_UICR ALL
  COMMAND ${PYTHON_RUNNER} ${DIR_UTILS}/uicr_mbr_hex_gen.py --no-use-uicr -i ${PROJECT_SOURCE_DIR}/artifacts/OnekeyProBTFW_BL -o ${FILE_INPUT_MBR_UICR}
  DEPENDS dfu ${CMAKE_PROJECT_NAME}_PYENV
)

# app
ExternalProject_Add(
  app
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/app
  INSTALL_DIR ${PROJECT_SOURCE_DIR}/artifacts
  CMAKE_GENERATOR "Ninja"
  CMAKE_ARGS "-DTOOL_CHAIN_PREFIX=${TOOL_CHAIN_PREFIX} "
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX:STRING=${PROJECT_SOURCE_DIR}/artifacts "
  DEPENDS ${CMAKE_PROJECT_NAME}_DFU_UPGRADE_PAYLOAD
)

###############################
# Pack and Sigh

# gen bl config
add_custom_target(
  ${CMAKE_PROJECT_NAME}_BL_CONF ALL
  COMMAND ${DIR_UTILS}/nrfutil 
          settings generate --family NRF52 --bootloader-version 2 --bl-settings-version 2 
          --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --application ${FILE_INPUT_APP} --application-version 3 
          --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ${FILE_INPUT_SD} 
          --key-file ${FILE_KEY}  ${FILE_INPUT_BL_CONFIG}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_KEY app
)

# megre factory hex
add_custom_target(
  ${CMAKE_PROJECT_NAME}_FACTORY_HEX ALL
  COMMAND ${DIR_UTILS}/mergehex 
                      --merge ${FILE_INPUT_BOOTLOADER} ${FILE_INPUT_BL_CONFIG} ${FILE_INPUT_SD} ${FILE_INPUT_APP} ${FILE_INPUT_MBR_UICR}
                      --output ${FILE_OUTPUT_FACTORY_HEX}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_BL_CONF dfu app ${CMAKE_PROJECT_NAME}_MBR_UICR
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
  COMMAND ${PYTHON_RUNNER} ${DIR_UTILS}/ota_to_onekey_bin.py ${FILE_OUTPUT_ZIP} ${FILE_OUTPUT_OTA_BIN}
  DEPENDS ${CMAKE_PROJECT_NAME}_OUT_DIR ${CMAKE_PROJECT_NAME}_UTIL ${CMAKE_PROJECT_NAME}_KEY dfu app
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