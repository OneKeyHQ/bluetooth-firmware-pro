#!/bin/bash
# $1 -> hex path

tee TempFlashScript.jlink > /dev/null << EOT
usb $JLINK_SN
device NRF52832_XXAA
SelectInterface swd
speed 8000
RSetType 0
loadbin $1 0x00000000
g
exit
EOT

JLinkExe -nogui 1 -commanderscript TempFlashScript.jlink

rm TempFlashScript.jlink