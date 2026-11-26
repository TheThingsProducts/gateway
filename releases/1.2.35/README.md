# Firmware Release 1.2.35 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.35                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-19                   |
| Commit      | 8cb3cfd                      |

Versions 1.2.33 and 1.2.34 were test builds and were never released.

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.35](https://github.com/pe1mew/gateway/releases/tag/v1.2.35):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
8263bfe483808982552fa56111aff56761b6f831c8fa0e73d708ed3910264bf5  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

Both changes remove watchdog resets during boot. Together they explain the two or three reboots
in a row that gateways showed after a reboot.

### Watchdog reset during LoRa module configuration (issue #3)

The LoRa module UART was opened once, and that single driver handle was used for the blocking
read of the receive thread and the blocking writes of the LoRa task. A Harmony USART client has
one buffer object for both directions, so every write overwrote the buffer object of the pending
read. When the module's answer arrived before the LoRa task had finished its write call, that
task spun forever in the driver and the watchdog reset the gateway after 8 s. The same race
existed for downlinks during normal operation.

- Separate read and write handles for the LoRa UART (`app_lora.c`), one more USART driver client.

### Watchdog reset during TLS connect (issue #5)

The TLS handshake ran as a busy loop inside the system task. While it waited for data from the
server the watchdog was not cleared and the application and LoRa tasks got no CPU, so a handshake
that stalled for 8 s reset the gateway.

- The handshake does one step per call and returns to the task loop while it waits
  (`net_pres_enc_glue.c`).
- The HTTP and MQTT clients give up a TLS connect after 10 s and retry, instead of waiting
  forever.

Why a handshake to the account server occasionally stalls is not known yet. The gateway now
retries it (15–25 s longer boot) instead of resetting.

### Memory layout

RAM was completely in use: the startup stack had only about 700 bytes left, and a build with
80 bytes more variables crashed at boot. `configTOTAL_HEAP_SIZE` is reduced from 350000 to
346000 bytes and the linker now reserves at least 4096 bytes for the startup stack. Free heap
while bridging is about 44 KB (was 48 KB).

### Watchdog diagnostics

When the watchdog has not been cleared for 6 s the console prints, every 0.5 s until the reset,
which tasks have checked in and which task is running:

```
*** WDT pending 60 ds, armed sys/app/lora=011 sysstage=8 tlsloops=1304998 tlsret=-26880 cur=Sys Tasks
```

The planned 24 h reboot for the firmware update check is done through the watchdog, so these
lines also appear there, after `MAIN: Rebooting gateway for firmware update check`.

### Validation on the test gateway (2026-11-19)

30 reboots in a row per build, serial console logged:

| | 1.2.32 | 1.2.35 |
|---|---|---|
| watchdog reset during LoRa module configuration | 5 | 0 |
| watchdog reset during TLS connect | 5 | 0 |
| stalled TLS connects ended by the timeout and retried | – | 5 |

HTTP stress without failures and a boot with an HTTPS FOTA URL passed on this image.

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
