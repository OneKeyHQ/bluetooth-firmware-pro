#!/bin/sh

PATH_UTILS=./utils
PATH_KEY=temp.pk
PATH_OUTPUT_DIR=artifacts_signed
PATH_OUTPUT_FACTORY_HEX=$PATH_OUTPUT_DIR/factory.hex
PATH_OUTPUT_FACTORY_BIN=$PATH_OUTPUT_DIR/factory.bin
PATH_OUTPUT_ZIP=$PATH_OUTPUT_DIR/ota.zip
PATH_OUTPUT_OTA_BIN=$PATH_OUTPUT_DIR/ota.bin
PATH_BL_CONFIG=bl_config.hex
PATH_BOOTLOADER=artifacts/dfu.hex
PATH_APP=artifacts/app.hex
PATH_SD=ble-firmware/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex

rm -rf $PATH_OUTPUT_DIR
mkdir $PATH_OUTPUT_DIR

chmod +x $PATH_UTILS/*

##### SETUP #####

$PATH_UTILS/nrfutil install nrf5sdk-tools

echo "$BT_SIG_PK" > $PATH_KEY
unset BT_SIG_PK

##### FACTORY #####

# gen hex
$PATH_UTILS/nrfutil settings generate --family NRF52 --application $PATH_APP --application-version 3 --bootloader-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --key-file $PATH_KEY --bl-settings-version 2 $PATH_BL_CONFIG
$PATH_UTILS/mergehex --merge $PATH_BOOTLOADER $PATH_BL_CONFIG $PATH_SD $PATH_APP --output $PATH_OUTPUT_FACTORY_HEX
rm $PATH_BL_CONFIG

# gen bin
objcopy -I ihex -O binary --gap-fill=0xFF $PATH_OUTPUT_FACTORY_HEX $PATH_OUTPUT_FACTORY_BIN.tmp
dd if=$PATH_OUTPUT_FACTORY_BIN.tmp of=$PATH_OUTPUT_FACTORY_BIN bs=512K count=1
rm $PATH_OUTPUT_FACTORY_BIN.tmp 

##### OTA #####

# sign app
$PATH_UTILS/nrfutil pkg generate --application $PATH_APP --application-version 3 --hw-version 52 --sd-req 0xCB --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --key-file $PATH_KEY $PATH_OUTPUT_ZIP
rm $PATH_KEY
$PATH_UTILS/ota_to_onekey_bin.py $PATH_OUTPUT_ZIP $PATH_OUTPUT_OTA_BIN