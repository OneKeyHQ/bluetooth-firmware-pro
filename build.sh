#!/bin/sh

##### KEY #####
export BT_SIG_PK_FILE=$(pwd)/temp.pk
echo "$BT_SIG_PK" > $BT_SIG_PK_FILE

##### CLEANUP #####
rm -r artifacts 2> /dev/null
rm -r artifacts_signed 2> /dev/null

##### BUILD #####
# remove build folder if exists
rm -r _build 2> /dev/null
# build
mkdir -p _build
cmake  -G 'Ninja'  -S ./  -B ./_build
cmake --build ./_build --config Release -- -j$(nproc)
# remove build folder
rm $BT_SIG_PK_FILE
rm -r _build
