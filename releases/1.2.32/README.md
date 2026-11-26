# Firmware Release 1.2.32 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.32                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-18                   |
| Commit      | 7787847                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.32](https://github.com/pe1mew/gateway/releases/tag/v1.2.32):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
3f2b53181b771452ce84608e672d48d2c9f5647403a41557d5f7d5bae6a8b5c1  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

Note: the console banner of this image reports `commit: 230b4bdc` because the image was built
from the 1.2.32 working tree before it was committed as 7787847. The sources are identical; this
is the exact image that passed the validation below.

---

## Changes

### Reboot on web UI access after long uptime (issue #2)

Gateways rebooted when the web UI was used after a few days of uptime, and a gateway configured
with an HTTPS FOTA URL could boot-loop. Root cause: the firmware used FreeRTOS `heap_2`, which
never merges freed blocks. Web UI requests and TLS sessions fragment the heap; after any MQTT
publish timeout the firmware tore down the whole network stack and opened a fresh TLS session
that needs about 75 KB in four large blocks. On the fragmented heap that allocation failed and
`vApplicationMallocFailedHook` reset the gateway.

- FreeRTOS `heap_4` (coalescing allocator) replaces `heap_2`. Same 48 KB free as before, but in
  11–20 blocks instead of 220–243, with a largest free block of 30–40 KB instead of 11 KB.
- An MQTT error no longer tears down the network. The gateway reconnects MQTT only and stays
  operational; after three consecutive failures it falls back to the full network re-init.
- The `MON:` console line reports the largest free block, the number of free blocks and the
  minimum free heap ever, so future memory problems are visible without a debugger.

Validation on the test gateway (2026-11-18): HTTP stress without failures, boot with an HTTPS
FOTA URL (the boot-loop condition) stable, cable pull recovered in 1 s, and a forced MQTT
publish timeout (TCP 8881 blocked at the router) handled by the MQTT-only reconnect without a
reset. A production gateway on 1.2.30 reset on the same forced reconnect.

---

## Flashing

### FOTA (preferred)

Serve `firmware.hex` and `checksums` from a FOTA server (see
[pe1mew/ttksgfotasrv](https://github.com/pe1mew/ttksgfotasrv)) and set the gateway's
Override FOTA URL to that server; the gateway updates on its next boot.

### SD card (no programmer required)

```bash
lsblk   # find device node, e.g. /dev/sdb
sudo /home/remko/ksig/prepare-sdcard.sh /dev/sdX
```

Insert SD card into gateway and power cycle.

### PICkit 4

```bash
IPECMD=/opt/microchip/mplabx/v6.25/mplab_platform/mplab_ipe/ipecmd.sh
$IPECMD -TPPK4 -P32MZ2048EFM144 -F firmware.hex -M
```
