#!/bin/bash

set -e -o pipefail

dpkg --add-architecture i386
apt-get update
apt-get install -y libc6:i386 libx11-6:i386 libxext6:i386 libstdc++6:i386 libexpat1:i386

if [[ ! -d /tmp/microchip ]]
then
  mkdir -p /tmp/microchip
fi

## Download and Install MPLAB if needed
if [[ ! -d /opt/microchip/mplabx/v3.45 ]]
then
  if [[ ! -f /tmp/microchip/MPLABX-v3.45-linux-installer.sh ]]
  then
    curl -sSL http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-v3.45-linux-installer.tar | tar -xv -C /tmp/microchip
  fi
  sudo /tmp/microchip/MPLABX-v3.45-linux-installer.sh -- --mode unattended
fi

## Link MPLAB if needed
if [[ ! -e /opt/microchip/mplab_ide ]]
then
  ln -s /opt/microchip/mplabx/v3.45/mplab_ide /opt/microchip/mplab_ide
fi

## Download and install XC32 if needed
if [[ ! -d /opt/microchip/xc32/v1.42 ]]
then
  if [[ ! -f /tmp/microchip/xc32-v1.42-full-install-linux-installer.run ]]
  then
    curl -sSL -o /tmp/microchip/xc32-v1.42-full-install-linux-installer.run http://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v1.42-full-install-linux-installer.run
    chmod +x /tmp/microchip/xc32-v1.42-full-install-linux-installer.run
  fi
  sudo /tmp/microchip/xc32-v1.42-full-install-linux-installer.run -- --mode unattended --netservername ""
fi
