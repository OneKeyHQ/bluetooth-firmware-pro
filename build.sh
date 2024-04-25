#!/bin/sh

set -e

##### KEY #####
export BT_SIG_PK_FILE=$(pwd)/temp.pk
echo "$BT_SIG_PK" > $BT_SIG_PK_FILE

##### CLEANUP #####
rm -rf artifacts
rm -rf artifacts_signed 

##### BUILD #####
# remove build folder if exists
rm -rf _build
# build
mkdir -p _build
cmake  -G 'Ninja'  -S ./  -B ./_build -DTOOL_CHAIN_PREFIX=$TOOL_CHAIN_PREFIX
cmake --build ./_build --config Release -- -j$(nproc)
utils/hash.py -t bluetooth -f artifacts/OnekeyProBTFW_APP.bin > artifacts/sha256.txt
# remove build folder
rm -f $BT_SIG_PK_FILE
rm -rf _build
