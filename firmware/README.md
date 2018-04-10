# The Things Gateway Firmware

## Installation

Installing the firmware is usually done using over-the-air updates from our official releases. Here are the steps to install custom firmware:

- Prepare a FAT32-formatted SD card with a folder called `update`.
- Copy the `firmware.hex` and `checksums` files to the `update` folder.
- Place the SD card in the gateway and power cycle the gateway.

```
SDCARD
└── update
    ├── checksums
    └── firmware.hex
```

## Firmware Releases

| **Branch** | **Downloads** | **Notes** |
| ---------- | ------------- | --------- |
| `stable`   | [`firmware.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/stable/firmware.hex) [`checksums`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/stable/checksums) | Default over-the-air update channel |
| `beta`     | [`firmware.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/beta/firmware.hex) [`checksums`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/beta/checksums) | Over-the-air update channel when "Beta Updates" enabled |
| `master`   | [`firmware.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/master/firmware.hex) [`checksums`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/master/checksums) | Release candidate for `stable` |
| `develop`  | [`firmware.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/develop/firmware.hex) [`checksums`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/develop/checksums) | Release candidate for `beta` |

## Firmware with bootloader

⚠️ Only for experts ⚠️

If you have a Microchip Debugger/Programmer, you can use the following files to flash the bootloader and firmware simultaneously. See [the docs](https://www.thethingsnetwork.org/docs/gateways/gateway/programhexfile.html) for more details on the programming procedure.

| **Branch** | **Downloads** | **Notes** |
| ---------- | ------------- | --------- |
| `stable`   | [`firmware-with-bootloader.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/stable/firmware-with-bootloader.hex) | |
| `beta`     | [`firmware-with-bootloader.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/beta/firmware-with-bootloader.hex) | |
| `master`   | [`firmware-with-bootloader.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/master/firmware-with-bootloader.hex) | |
| `develop`  | [`firmware-with-bootloader.hex`](https://thethingsproducts.blob.core.windows.net/the-things-gateway/v1/develop/firmware-with-bootloader.hex) | |

## Development Setup

You probably already have the following "essential" development tools, but just to be sure:

- `build-essential` or XCode (includes `git` and `make`)
- `sha256sum` (from GNU coreutils)

Get the source code and initialize the submodules to get the dependencies.

```sh
git clone https://github.com/TheThingsProducts/gateway.git
git submodule update --init
```

Download the following tools from the Microchip download pages:

| Tool | Windows | Mac | Linux | 
|--------|---------|---------------------------|-----------------|
| MPLAB IDE X v3.45|  [link](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-v3.45-windows-installer.exe)|  [link](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-v3.45-osx-installer.dmg)|[link](http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-v3.45-linux-installer.tar)|
| MPLAB XC32 v1.42 | [link](http://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v1.42-full-install-windows-installer.exe)| [link](http://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v1.42-full-install-osx-installer.dmg)| [link](http://ww1.microchip.com/downloads/en/DeviceDoc/xc32-v1.42-full-install-linux-installer.run)|
| Harmony v1.08.01| [link](http://ww1.microchip.com/downloads/en/DeviceDoc/harmony_v1_08_01_windows_installer.exe)| [link](http://ww1.microchip.com/downloads/en/DeviceDoc/harmony_v1_08_01_osx_installer.dmg)| [link](http://ww1.microchip.com/downloads/en/DeviceDoc/harmony_v1_08_01_linux_installer.run)|

## Fix MPLab script

After building the project using the MPLabX GUI, the paths in the `configurations.xml` are made platform dependent. In order to fix this, run the `./fix_mplab.sh` script.

## Local Build

- Make sure you have downloaded and installed the IDE and compilers stated at **Development Setup** above.
- Run `./generate_version_header.sh`.
- Run `./compile.sh`.
- After successful build, create the `firmware.hex` and `checksums` files with `./generate_hex_with_checksum.sh`.
- Use those two files in the **Installation** procedure above.

## Build server

- Make sure you have downloaded and installed the IDE and compilers stated at **Development Setup** above.
- Install **Jenkins Server**
- Create Multi branch Pipline project
- Link this repository
- Hit *Scan Multibranch Pipeline Now*

## Build in Docker

- Install [Docker](https://www.docker.com/community-edition#/download)
- To build the Docker image, run `docker build -t firmware-builder -f Dockerfile.build .` (uou only need to do this once).
- To compile the firmware, run `docker run --rm -it -v $(pwd):/build -w /build firmware-builder ./compile.sh && ./generate_hex_with_checksum.sh`

## TASKS

| Task        | Priority | Size |
|-------------|----------|------|
| LORA_READ   | 7        | 4096 |
| TCPIP_TASKS | 6        | 4096 |
| WIFI_DRV    | 5        | 2048 |
| SYS_TASKS   | 4        | 4096 |
| LORA_TASKS  | 3        | 4096 |
| APP_TASKS   | 2        | 4096 |
| MQTTTASK    | 1        | 4096 |

## SERIALFLASH LAYOUT

| Sector | Address | Content                   | Length (Bytes)  |
|--------|---------|---------------------------|-----------------|
| 0      | 0       | SHA256 Hash of current FW | 32              |
|        |         |                           |                 |
| 1      | 4096    | Magic Bytes               | 4               |
|        | 4100    | Network Type              | 1               |
|        | 4101    | Security Mode             | 1               |
|        | 4102    | WiFi SSID                 | 32 + 1          |
|        | 4135    | WiFi Security Key         | 64 + 1          |
|        |         |                           |                 |
| 2      | 8192    | Magic Bytes               | 4               |
|        | 8196    | Gateway ID                | 100             |
|        | 8296    | Gateway Key               | 200             |
|        | 8496    | Account Server URL        | 255             |
|        | 8751    | Locked flag               | 1               |
|        |         |                           |                 |
| 3      | 12288   | Magic Bytes               | 4               |
|        | 12292   | FW Image Length           | 4               |
|        | 12296   | SHA256 Hash of new FW     | 32              |
|        |         |                           |                 |
| 4      | 16384   | FW Image                  | FW Image Length |
