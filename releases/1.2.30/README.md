# Firmware Release 1.2.30 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.30                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-18                   |
| Commit      | da57961 + 1.2.30 changes     |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.30](https://github.com/pe1mew/gateway/releases/tag/v1.2.30):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
9a3ed773ad6e33f52e22b56a0b56be6ecd820a4a4ac11df159d099d0239245ce  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

### UDP forwarder: uplinks rejected by ChirpStack (issue #1)

Uplinks forwarded with the Semtech UDP protocol were accepted by The Things Network but
silently dropped by ChirpStack. The `freq` field of `rxpk` was formatted as
`(double)hz / 1e6` with `%.6f`. XC32 compiles `double` as 32-bit, so 868300000 Hz was
printed as `868.300048` (and 867.500061, 868.099975, 868.499938 for the other channels).
ChirpStack matches `freq` exactly against the region's channel list and rejects the frame
with "Unknown channel for frequency"; TTN tolerates the deviation.

- `freq` is now formatted with integer arithmetic (`868.300000`), rounded to 500 Hz as a
  safety net. Verified with a UDP capture: all channels now report exact values.
- The downlink `txpk.freq` is parsed without floating point as well.
- Uplinks queued while the forwarder is disabled are discarded, so stale packets are no
  longer flushed when it is enabled.

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
