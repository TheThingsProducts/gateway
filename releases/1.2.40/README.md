# Firmware Release 1.2.40 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.40                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-25                   |
| Commit      | df79ab9                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.40](https://github.com/pe1mew/gateway/releases/tag/v1.2.40):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
ecdda016d71ecbdd43095c7696d764f21fcca27e57d5e63285d5bc931d1ec638  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

Web UI wording only. No change in behaviour, in `status.cgi` or in the settings.

### "TTN" instead of "Broker" on the Info page (issue #16)

The MQTT broker connection of this firmware is the connection to The Things Network; the Info
page now says so. The UDP rows are unchanged.

| Row on the Info page, 1.2.39 | 1.2.40 | `status.cgi` (unchanged) |
|---|---|---|
| Broker connection | TTN connection | `connbroker` |
| Broker packets up | TTN packets up | `pup` |
| Broker packets down | TTN packets down | `pdown` |

Scripts that read `status.cgi` keep working: the field names are the same.

### Validation on the test gateway (2026-11-25)

On this image: updated by FOTA in a single boot; web UI acceptance test 322 passed, 0 failed;
HTTP stress without failures (56 KB heap free, largest free block 44064 B, lowest free 49 KB);
the three rows checked in a browser.

Short soak on this image (2026-11-25 13:37 to 19:40, 6 hours; the firmware's own 24-hour reboot
was not part of it) passed: no reset, no exception, assert or failed allocation; heap flat
(278–281 KB used, 53–56 KB free, largest free block 44064 bytes in every sample, lowest free
49 KB). One MQTT error, handled by an MQTT-only reconnect, bridging again within 1 second; no
network re-initialisation, no TLS timeout. 1265 uplinks delivered to The Things Network, 1 failed
send (at the MQTT error), 181 downlinks received; 1265 uplinks sent to the UDP server, all
acknowledged; the `status.cgi` counters matched the console. A 24-hour soak was not repeated
because only the text of three rows on the Info page differs from 1.2.39, which passed one.

### Known issue

A gateway that uses WiFi as its uplink can be unreachable from the local network (web interface,
ping) on some WiFi networks while it keeps forwarding packets: see
[issue #18](https://github.com/pe1mew/gateway/issues/18). Not new in this version.

---

## Flashing

### FOTA (preferred)

Serve `firmware.hex` and `checksums` from a FOTA server (see
[pe1mew/ttksgfotasrv](https://github.com/pe1mew/ttksgfotasrv)) and set the gateway's
Override FOTA URL to that server; the gateway updates on its next boot.

### SD card (no programmer required)

Copy `firmware.hex` and `checksums` to a directory `update/` on a FAT32 formatted SD card,
insert the card into the gateway and power cycle. Remove the card after the update and
power cycle again.

### PICkit 4

```bash
IPECMD=/opt/microchip/mplabx/v6.25/mplab_platform/mplab_ipe/ipecmd.sh
$IPECMD -TPPK4 -P32MZ2048EFM144 -F firmware.hex -M
```
