# Firmware Release 1.2.28 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.28                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-04-20                   |
| Commit      | d854bb2                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.28](https://github.com/pe1mew/gateway/releases/tag/v1.2.28):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card or PICkit 4 flashing |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
ebfb4292e7f24f2150a8f49214d05ddc28ea90bd3b9ee4cf21fb09e2f642c96f  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Features

### FOTA URL override

A custom firmware update server URL can be configured through the gateway web UI without requiring a connection to TTN. When enabled, the gateway fetches firmware updates from the configured URL instead of the TTN update server.

- Configure via **Settings → FOTA** in the web UI
- Supports plain HTTP and HTTPS
- URL format: `http://<host>[:<port>][/<path>]`
- Files served must follow the standard layout: `<path>/checksums` and `<path>/firmware.hex`
- Setting persists over power cycles independently of other settings

---

### Semtech UDP packet forwarder

The gateway can forward received LoRa packets to any LNS (LoRaWAN Network Server) that supports the Semtech UDP packet forwarder protocol, such as ChirpStack. This enables use of the gateway with third-party network servers independently of TTN.

Configure via **Settings → UDP Forwarder** in the web UI.

| Setting | Description |
|---|---|
| Server address | Hostname or IP of the LNS gateway bridge |
| Port up | UDP port for upstream frames (PUSH_DATA) |
| Port down | UDP port for downstream frames (PULL_DATA / PULL_RESP) |
| Uplink only | Disable downlink scheduling |

**Protocol features:**

- **PUSH_DATA** — uplink frames forwarded with correct spreading factor, bandwidth, RSSI, SNR, frequency, and CRC status
- **PULL_DATA** — keepalive sent every 5 seconds to maintain the downlink path
- **PULL_RESP / TX_ACK** — downlink scheduling from the LNS (unless uplink-only is enabled)
- **Gateway stat** — periodic status report sent every 30 seconds containing packet counters (`rxnb`, `rxok`, `rxfw`, `dwnb`, `txnb`) and acknowledgement rate
- Separate upstream and downstream sockets when `port_up ≠ port_down`
- Gateway EUI-64 derived from WiFi MAC address

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
