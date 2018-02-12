FROM ubuntu:xenial

RUN dpkg --add-architecture i386 && \
  apt-get update && \
  apt-get install -y build-essential curl sudo git && \
  apt-get install -y libc6:i386 libx11-6:i386 libxext6:i386 libstdc++6:i386 libexpat1:i386 && \
  rm -rf /var/lib/apt/lists/*

## Set versions

ARG mplab_version=3.45
ARG xc32_version=1.42
ARG harmony_version=1_08_01

## Download installers

RUN curl -sSL -o /tmp/MPLABX-v${mplab_version}-linux-installer.tar http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-v${mplab_version}-linux-installer.tar && \
  tar -xvf /tmp/MPLABX-v${mplab_version}-linux-installer.tar -C /tmp && \
  chmod +x /tmp/MPLABX-v${mplab_version}-linux-installer.sh && \
  rm /tmp/MPLABX-v${mplab_version}-linux-installer.tar
RUN curl -sSL -o /tmp/xc32-v${xc32_version}-full-install-linux-installer.run http://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v${xc32_version}-full-install-linux-installer.run && \
  chmod +x /tmp/xc32-v${xc32_version}-full-install-linux-installer.run
RUN curl -sSL -o /tmp/harmony_v${harmony_version}_linux_installer.run http://ww1.microchip.com/downloads/en/DeviceDoc/harmony_v${harmony_version}_linux_installer.run && \
  chmod +x /tmp/harmony_v${harmony_version}_linux_installer.run

## Install MPLAB

RUN sudo /tmp/MPLABX-v${mplab_version}-linux-installer.sh -- --mode unattended
RUN ln -s /opt/microchip/mplabx/v${mplab_version}/mplab_ide /opt/microchip/mplab_ide

## Install XC32

RUN /tmp/xc32-v${xc32_version}-full-install-linux-installer.run -- --mode unattended --netservername ""

## Install Harmony

RUN /tmp/harmony_v${harmony_version}_linux_installer.run -- --mode unattended
