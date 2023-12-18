#!/bin/sh

export TOP_DIR=$(pwd)
export GNU_INSTALL_ROOT=/usr/bin/
export GNU_PREFIX=arm-none-eabi
export GNU_VERSION=$($GNU_PREFIX-gcc -dumpfullversion)
PATH_OUTPUT_DIR=artifacts

echo Build micro-ecc
cd ble-firmware/external/micro-ecc/
make -C nrf52hf_armgcc/armgcc
cd $TOP_DIR

echo Build dfu
cd dfu
make -j8
cd $TOP_DIR

echo Build app
cd app
make -j8
cd $TOP_DIR

echo Move artifacts
rm -rf $PATH_OUTPUT_DIR
mkdir $PATH_OUTPUT_DIR
mv dfu/_build/nrf52832_xxaa_s132.bin $PATH_OUTPUT_DIR/dfu.bin
mv dfu/_build/nrf52832_xxaa_s132.hex $PATH_OUTPUT_DIR/dfu.hex
mv app/_build/nrf52832_xxaa.bin $PATH_OUTPUT_DIR/app.bin
mv app/_build/nrf52832_xxaa.hex $PATH_OUTPUT_DIR/app.hex
