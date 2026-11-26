# Firmware Release 1.2.37 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.37                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-20                   |
| Commit      | 606259d                      |

Version 1.2.36 was a diagnostic build and was never released.

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.37](https://github.com/pe1mew/gateway/releases/tag/v1.2.37):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
16fc5d0fdd11b3c9c5bf6ffaefc54f98bf50b656412266613c87f59780e84fc1  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

### Random TCP source ports and sequence numbers (issue #6)

The firmware initialised its pseudo-random generator with the same constant at every boot. The
TCP/IP stack takes the source port and the initial sequence number of every outgoing connection
from that generator (and the DHCP and DNS transaction ids), so after each reboot the gateway
connected to the same servers from the same ports as before. On a local test server 134
connections from one gateway came from about a dozen source ports.

When the gateway rebooted several times in a row, about 5 in 97 TLS handshakes to the account
server stalled until the 10 s timeout introduced in 1.2.35. A connection that reuses the port of
one that was open seconds earlier can be mishandled by the server, a load balancer or a NAT
router; which of them is responsible has not been established.

- The generator is seeded at boot with 64 bits from the microcontroller's hardware random number
  generator (`system_init.c`). The boot banner reports it:

```
PRNG seed: 64 hardware random bits
```

Sequence numbers and DNS transaction ids are no longer predictable either.

### Known double free in the WiFi driver library (issue #4)

The console line `Head magic match void: trying to free an already freed block, ignore` on every
WiFi shutdown comes from Microchip's closed MRF24WN library, which frees its context twice. It
cannot be fixed in source; the heap tracker ignores the second free, and the check now carries a
comment that it must stay. No functional change.

### Validation on the test gateway (2026-11-20)

30 reboots in a row per build, serial console logged:

| | 1.2.35 | 1.2.37 |
|---|---|---|
| TLS connects started | 97 | 90 |
| stalled TLS connects (10 s timeout, retried) | 5 | 0 |
| source ports of the connections to the local FOTA server | a few, repeated | 32 connections, 32 ports |

HTTP stress without failures and a boot with an HTTPS FOTA URL passed on this image.

24-hour soak on this image (2026-11-20 22:57 to 2026-11-21 22:56) passed: no unexpected reboot, no
exception or assert, heap flat (290–294 KB used, 40–44 KB free); six MQTT errors recovered
without a reset; the firmware's own 24-hour reboot came back in a single boot with all TLS
connects at the first attempt.

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
