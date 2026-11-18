# The Things Gateway

## Software Modernization Effort

This repository represents an ongoing effort to modify and update the original gateway software with the assistance of Large Language Models (LLMs). The complete toolchain is currently running on a dedicated Ubuntu deployment, with development facilitated by Claude-code. This approach allows for systematic analysis, refactoring, and enhancement of the legacy codebase while maintaining compatibility with the existing hardware platform.

Currently, direct flashing via PICkit4 is not possible due to a defective programmer. However, firmware deployment using the SD-card method works reliably and is being used for testing and updates.

### Key Modifications

**Version 1.1.3 - WiFi Access Point Management:** The original firmware exhibited problematic behavior where the WiFi Access Point remained active even when an Ethernet connection was established and no WiFi credentials were configured. This resulted in unnecessary pollution of the 2.4 GHz WiFi band and user confusion. The modification implements intelligent AP management that automatically disables the Access Point when Ethernet connectivity is available, while ensuring the AP remains accessible for configuration when needed. Modified files include [app_wifi.c](firmware/src/app_wifi.c), [app_wifi.h](firmware/src/app_wifi.h), and [ssm_wait_for_internet.c](firmware/src/ssm_wait_for_internet.c). **Status: Implemented and tested.**

**Version 1.1.6 - Certificate Updates:** Critical security certificates embedded in the firmware have been updated to prevent TLS handshake failures with The Things Industries servers. The original certificates (Baltimore CyberTrust Root expired May 12, 2025, and Amazon Root CA 1 expired October 19, 2025) have been replaced with renewed certificates valid until 2037-2038. The Baltimore CyberTrust Root was removed entirely as DigiCert Global Root G2 (valid to 2028) provides equivalent coverage. Modified files include [AMAZON_ROOT_CA_1.c](firmware/AMAZON_ROOT_CA_1.c), [AMAZON_ROOT_CA_1.h](firmware/AMAZON_ROOT_CA_1.h), [STARFIELD_SERVICES_ROOT_CA_G2.c](firmware/STARFIELD_SERVICES_ROOT_CA_G2.c), and [STARFIELD_SERVICES_ROOT_CA_G2.h](firmware/STARFIELD_SERVICES_ROOT_CA_G2.h). **Status: Implemented and verified with TLS handshake testing.**

**Version 1.2.28 - Semtech UDP Packet Forwarder:** Added a Semtech-protocol UDP packet forwarder alongside the existing MQTT forwarder. The UDP forwarder provides compatibility with standard LoRaWAN network servers that use the Semtech UDP protocol (port 1700 by default). Configuration is persisted to serial flash and exposed via a new web UI settings page at `/udp`. The implementation supports concurrent operation with MQTT, configurable uplink-only mode, and graceful handling of downlink conflicts between protocols. Modified files include [app_udp.c](firmware/src/app_udp.c), [app_udp.h](firmware/src/app_udp.h), [app_serialflash.c](firmware/src/app_serialflash.c), [custom_http_app.c](firmware/src/custom_http_app.c), and web UI templates. **Status: Implemented and integrated.**

**Version 1.2.30 - UDP Forwarder ChirpStack Compatibility:** Uplinks forwarded over the Semtech UDP protocol were accepted by The Things Network but silently dropped by ChirpStack. The forwarder formatted the `freq` field as `(double)hz / 1e6` with `%.6f`; XC32 compiles `double` as 32-bit, so 868300000 Hz was printed as 868.300048 MHz. ChirpStack matches `freq` exactly against the region's channel list and rejects the frame with "Unknown channel for frequency", while TTN tolerates the deviation. The frequency is now formatted with integer arithmetic (rounded to 500 Hz as a safety net) and the downlink `txpk.freq` is parsed without floating point as well. The forwarder also discards queued uplinks while it is disabled so that stale packets are no longer flushed when it is re-enabled. Modified file: [app_udp.c](firmware/src/app_udp.c). **Status: Implemented, verified with a UDP capture (issue #1).**

**Version 1.2.28 - FOTA URL Override:** Added the ability to configure a custom firmware update server URL through the gateway web UI, enabling firmware updates from alternative sources without requiring a connection to The Things Network. The custom FOTA URL can be configured at Settings → FOTA in the web UI. Supports both HTTP and HTTPS protocols. The setting persists independently of other configuration parameters, allowing use of custom FOTA servers such as [https://github.com/pe1mew/ttksgfotasrv](https://github.com/pe1mew/ttksgfotasrv). **Status: Implemented and integrated.**

**FOTA Server:** A custom Firmware-Over-The-Air (FOTA) server implementation has been created to support firmware updates for The Things Gateway. The server implements the HTTP/HTTPS interface required by the gateway's FOTA client, serving firmware checksums and binary files. The FOTA server specification and implementation can be found at [https://github.com/pe1mew/ttksgfotasrv](https://github.com/pe1mew/ttksgfotasrv). This enables independent firmware distribution without reliance on The Things Industries' infrastructure.

![The Things Gateway](doc/header.png)

## Where to buy

A list of resellers can be found [here](https://www.thethingsnetwork.org/the-things-products).

## Enclosure

Designs of the enclosure can be found in the [enclosure](./enclosure) directory. These designs are licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).

## Hardware

Designs of the hardware can be found in the [hardware](./hardware) directory. These designs are licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).

## Firmware

Firmware source code is located in the [firmware](./firmware) directory. Firmware source code is released under a [MIT License](https://opensource.org/licenses/MIT).

## Feature Requests, Bug Reports and Support

- For feature requests and bug reports, create an [issue on Github](https://github.com/TheThingsProducts/gateway/issues)
- Community support on our [forum](https://www.thethingsnetwork.org/forum/c/gateways/the-things-gateway)
- Commercial support by [e-mail](mailto:support@thethingsproducts.com)
