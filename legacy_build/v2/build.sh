#!/bin/sh

##### BUILD #####

# remove build folder if exists
rm -r _build 2> /dev/null

# build app
mkdir -p _build/app
cmake  -G 'Ninja'  -S ./app  -B ./_build/app  -DCMAKE_INSTALL_PREFIX:STRING=$(pwd)/artifacts 
cmake --build ./_build/app --config Release -- -j$(nproc)
cmake --build ./_build/app --target install

# build bl
mkdir -p _build/bl
cmake  -G 'Ninja'  -S ./dfu  -B ./_build/bl  -DCMAKE_INSTALL_PREFIX:STRING=$(pwd)/artifacts 
cmake --build ./_build/bl --config Release -- -j$(nproc)
cmake --build ./_build/bl --target install

# remove build folder
rm -r _build

##### SIGN and PACK #####

# remove target folder if exists
rm -r $DIR_OUTPUT 2> /dev/null

# prepare nrf tools
DIR_UTILS=./utils
chmod +x $DIR_UTILS/*
$DIR_UTILS/nrfutil install nrf5sdk-tools

# populate vars
FILE_INPUT_BL_CONFIG=bl_config.hex
FILE_INPUT_BOOTLOADER=artifacts/OnekeyProBTFW_BL.hex
FILE_INPUT_APP=artifacts/OnekeyProBTFW_APP.hex
FILE_INPUT_SD=ble-firmware/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex
DIR_OUTPUT=artifacts_signed
FILE_OUTPUT_FACTORY_HEX=$DIR_OUTPUT/factory.hex
FILE_OUTPUT_FACTORY_BIN=$DIR_OUTPUT/factory.bin
FILE_OUTPUT_ZIP=$DIR_OUTPUT/ota.zip
FILE_OUTPUT_OTA_BIN=$DIR_OUTPUT/ota.bin

# target dir
mkdir -p $DIR_OUTPUT

# prepare temp key
FILE_KEY=temp.pk
echo "$BT_SIG_PK" > $FILE_KEY
unset BT_SIG_PK

# gen bl config
$DIR_UTILS/nrfutil \
    settings generate --family NRF52 --bootloader-version 2 --bl-settings-version 2 \
    --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --application $FILE_INPUT_APP --application-version 3   \
    --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice $FILE_INPUT_SD \
    --key-file $FILE_KEY  $FILE_INPUT_BL_CONFIG

# megre factory hex
$DIR_UTILS/mergehex --merge $FILE_INPUT_BOOTLOADER $FILE_INPUT_BL_CONFIG $FILE_INPUT_SD $FILE_INPUT_APP --output $FILE_OUTPUT_FACTORY_HEX

# gen factory bin
objcopy -I ihex -O binary --gap-fill=0xFF $FILE_OUTPUT_FACTORY_HEX $FILE_OUTPUT_FACTORY_BIN.tmp
dd if=$FILE_OUTPUT_FACTORY_BIN.tmp of=$FILE_OUTPUT_FACTORY_BIN bs=512K count=1
rm $FILE_OUTPUT_FACTORY_BIN.tmp 

# gen ota bin
$DIR_UTILS/nrfutil pkg generate --application $FILE_INPUT_APP --application-version 3 --hw-version 52 --sd-req 0xCB --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --key-file $FILE_KEY $FILE_OUTPUT_ZIP
$DIR_UTILS/ota_to_onekey_bin.py $FILE_OUTPUT_ZIP $FILE_OUTPUT_OTA_BIN

rm $FILE_INPUT_BL_CONFIG
rm $FILE_KEY