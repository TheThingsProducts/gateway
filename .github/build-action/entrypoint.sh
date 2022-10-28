#!/bin/bash

pushd firmware

./patch_network.sh
./generate_version_header.sh
./fix_mplab.sh
./compile.sh
./generate_hex_with_checksum.sh
./merge_bootloader.sh

popd
