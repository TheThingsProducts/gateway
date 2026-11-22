# Firmware Release 1.2.29 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.29                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-04-21                   |
| Commit      | d854bb2                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.29](https://github.com/pe1mew/gateway/releases/tag/v1.2.29):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card or PICkit 4 flashing |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
08cf98624b2d273ef96ae7d4a13c3ab9c1e7878429cf09e6501496f4a86984b3  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

### UDP gateway stat acknowledgement rate fix

The `ackr` field in the periodic gateway stat report could exceed 100% (e.g. 150%). The stat message itself is a PUSH_DATA and receives a PUSH_ACK from the LNS, but the sent counter (`s_push_sent`) was only incremented for uplink PUSH_DATA frames. The received ACK counter (`s_push_acked`) counted all PUSH_ACKs including those for stat messages, causing the rate to be overstated.

Fixed by incrementing `s_push_sent` when the stat PUSH_DATA is transmitted.

---

## Flashing

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
