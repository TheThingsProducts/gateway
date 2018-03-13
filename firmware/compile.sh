#!/bin/bash

set -e
set -o pipefail

if [ "$(uname -s)" = "Darwin" ]; then
  export PATH="$PATH:/Applications/microchip/xc32/v1.42/bin"
  prjMakefilesGenerator='/Applications/microchip/mplabx/v3.45/mplab_ide.app/Contents/Resources/mplab_ide/bin/prjMakefilesGenerator.sh'
elif [ "$(uname -s)" = "Linux" ]; then
  export PATH="$PATH:/opt/microchip/xc32/v1.42/bin"
  prjMakefilesGenerator='/opt/microchip/mplabx/v3.45/mplab_ide/bin/prjMakefilesGenerator.sh'
else
  echo "We're not sure how to handle your platform or OS. Please add your platform or OS to compile.sh and submit a pull request."
  exit 1
fi

$prjMakefilesGenerator -v TTN_Gateway.X@TTN_Gateway_v1
make -j -C TTN_Gateway.X/ -f nbproject/Makefile-TTN_Gateway_v1.mk SUBPROJECTS= .build-conf 2>&1 | tee compile_log.txt

