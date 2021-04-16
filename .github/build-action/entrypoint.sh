#!/bin/bash

pushd webui

java -jar ~/microchip/harmony/v1_08_01/utilities/mpfs_generator/mpfs2.jar /c ./static ../firmware/src mpfs_img2

popd

pushd firmware

./generate_version_header.sh
./fix_mplab.sh
./compile.sh
./generate_hex_with_checksum.sh
./merge_bootloader.sh

popd
