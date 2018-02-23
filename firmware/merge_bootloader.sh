#!/bin/bash

set -e

/opt/microchip/mplabx/v3.45/mplab_ide/platform/../mplab_ide/modules/../../bin/hexmate --edf=/opt/microchip/mplabx/v3.45/mplab_ide/platform/../mplab_ide/modules/../../dat/en_msgs.txt TTN_Gateway.X/dist/TTN_Gateway_v1/production/TTN_Gateway.X.production.hex bootloader.hex -ofirmware-with-bootloader.hex
