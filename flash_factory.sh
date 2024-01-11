#!/bin/sh

tee flashFactory.jlink > /dev/null << EOT
usb $JLINK_SN
device NRF52832_XXAA
SelectInterface swd
speed 8000
loadbin artifacts_signed/factory.hex 0x00000000
rx 100
g
exit
EOT

JLinkExe -nogui 1 -commanderscript flashFactory.jlink

rm flashFactory.jlink