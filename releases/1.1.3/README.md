# Firmware Release 1.1.3 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.1.3                        |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-04-18                   |
| Commit      | f9cea314                     |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.1.3](https://github.com/pe1mew/gateway/releases/tag/v1.1.3):

| File          | Description                                      |
|---------------|--------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card or PICkit 4 flashing |
| `checksums`    | SHA-256 checksum for integrity verification      |

### SHA-256

```
adb8815183138eec5d4768a9f2db1d6ac0b6391ccae20f9071e38323d0b17be5  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes in this release

### WiFi AP behaviour fix
**Problem:** The gateway access point (AP) remained visible and connectable
even when a wired Ethernet connection was active, and the AP was not
restored when a WiFi connection attempt failed.

**Fix:** Reworked the WiFi / internet state machine:
- AP is disabled as soon as Ethernet or WiFi infrastructure connects.
- AP is restored within 10 seconds when WiFi fails to connect, so the
  user can always reach the web UI to update credentials.
- `APP_WIFI_DISABLE()` now reconfigures the MRF24WN radio to INFRA mode
  before taking the interface down, stopping the AP SSID broadcast at the
  hardware level (previously only the TCP/IP layer was brought down,
  leaving the radio still advertising the AP SSID).
- `WIFI_CONNECT_TIMEOUT` (10 s) is now used for both the infra connect
  window and the AP retry interval, replacing the unused 120 s
  `WIFI_RETRY_TIMEOUT`.

**Note:** If the AP is still on, configuration information may still be in
non-volatile memory. To erase WiFi client settings, press the button in the
gateway for 2 seconds until the second LED lights, then release. The AP will
now not be active.

**Files changed:**
- `firmware/src/app_wifi.h` — added `APP_WIFI_DISABLE()` declaration
- `firmware/src/app_wifi.c` — added `EVENT_DISABLE_WIFI`, handler, and `APP_WIFI_DISABLE()`
- `firmware/src/ssm_wait_for_internet.c` — reworked `STATE_WAIT_FOR_NETWORK` on-enter and task loop

### CA certificate update
**Problem:** Two of the four CA root certificates compiled into firmware
had expired, causing all TLS connections (MQTT, OTA, activation, config
download, frequency plan) to fail silently.

| Certificate                  | Status   | Expiry     |
|------------------------------|----------|------------|
| Baltimore CyberTrust Root    | Removed  | 2025-05-12 (expired) |
| Amazon Root CA 1 (old)       | Replaced | 2025-10-19 (expired) |
| Amazon Root CA 1 (renewed)   | Added    | 2038-01-17 |
| Starfield Services Root CA G2| Added    | 2037-12-31 |
| ISRG Root X1                 | Unchanged| 2035-09-30 |
| DigiCert Global Root G2      | Unchanged| 2028-11-10 |

**Fix:** Baltimore CyberTrust Root removed (DigiCert Global Root G2
already present covers the same chains). Amazon Root CA 1 replaced with
the renewed certificate. Starfield Services Root CA G2 added as secondary
Amazon trust anchor.

**File changed:**
- `firmware/src/system_config/TTN_Gateway_v1/framework/net/pres/net_pres_cert_store.c`

---

## Flashing

### SD card (no programmer required)

1. Format SD card as FAT32 and copy files:
   ```bash
   sudo /path/to/prepare-sdcard.sh
   ```
2. Insert SD card into gateway and power cycle.
3. Bootloader verifies SHA-256, flashes firmware, then removes update files and reboots.

### PICkit 4

```bash
IPECMD=/opt/microchip/mplabx/v6.25/mplab_platform/mplab_ipe/ipecmd.sh
$IPECMD -TPPK4 -P32MZ2048EFM144 -F firmware.hex -M
```

---

## Known limitations

- TLS peer certificate CN/SAN verification (SNI) is not enabled —
  `mbedtls_ssl_set_hostname()` is not called. This means the hostname is
  not verified against the server certificate. Fix is planned.
