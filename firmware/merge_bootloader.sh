#!/bin/bash

set -e

if [ "$(uname -s)" = "Darwin" ]; then
  export PATH="$PATH:/Applications/microchip/xc32/v1.42/bin:/Applications/microchip/mplabx/v3.45/mplab_ide.app/Contents/Resources/mplab_ide/bin"
elif [ "$(uname -s)" = "Linux" ]; then
  export PATH="$PATH:/opt/microchip/xc32/v1.42/bin:/opt/microchip/mplabx/v3.45/mplab_ide/bin"
else
  echo "We're not sure how to handle your platform or OS. Please add your platform or OS to compile.sh and submit a pull request."
  exit 1
fi

hexmate TTN_Gateway.X/dist/TTN_Gateway_v1/production/TTN_Gateway.X.production.hex bootloader.hex -ofirmware-with-bootloader.hex
