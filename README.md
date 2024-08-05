# OneKey Pro Bluetooth Firmware

This repo contains bluetooth firmware for OneKey Pro

The firmware is based on NRF5 SDK 16.0.0, and build with CMake

## How to build

```shell
# make sure you have cmake, Python 3, amd aarm-none-eabi toolchain available in PATH

# export your OWN key for firmware signing
export BT_SIG_PK=$(cat <<EOF
-----BEGIN EC PRIVATE KEY-----
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
-----END EC PRIVATE KEY-----
EOF
)

# export toolchain prefix
export TOOL_CHAIN_PREFIX="arm-none-eabi"

# call build script
./build.sh
```

## CMake Build Targets

```shell
# ota image
OnekeyProBTFW_OTA_BIN

# full HEX image
OnekeyProBTFW_FACTORY_HEX

# flash full HEX image with jlink
OnekeyProBTFW_FLASH_FACTORY
```

## License

Please check License.md for details