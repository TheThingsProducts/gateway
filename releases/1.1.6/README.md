# Firmware Release 1.1.6 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.1.6                        |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-04-19                   |
| Commit      | e5b6fb64                     |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.1.6](https://github.com/pe1mew/gateway/releases/tag/v1.1.6):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card or PICkit 4 flashing |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
ab919e95a426ba1531c50f8c17fcc4176d324fc2554b469f376ba4099299a022  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes in this release

### FOTA TLS handshake failure — investigation and fix (1.1.4 – 1.1.6)

After deploying the certificate update in 1.1.3, FOTA (over-the-air firmware
update) continued to fail with `TLS Connection Negotiation Failed` on every boot.
This release concludes a three-step investigation that diagnosed the root cause
and fixed two TLS bugs uncovered in the process.

---

#### Step 1 — Diagnose: log the FOTA server URL (1.1.4)

**Problem:** The FOTA connection failed but the gateway never printed which server
it was connecting to. The firmware URL is fetched from TTN's config server at
runtime, and the relevant log line was gated behind `SYS_ERROR_DEBUG` (suppressed
in normal operation).

**Fix:** Changed the firmware URL log line in `app_ota.c` from `SYS_ERROR_DEBUG`
to `SYS_PRINT` and added a second line printing the parsed host, port, and TLS
flag. On the next boot the FOTA endpoint was revealed:
`https://ttkg-fw.thethingsindustries.com/v1/beta`

**Files changed:**
- `firmware/src/app_ota.c` — log `firmware_url` and parsed host/port/tls at INFO level before the TLS connection attempt

---

#### Step 2 — Fix: add SNI to HTTPS connections (1.1.5)

**Problem:** With the FOTA server identified, testing with
`openssl s_client -tls1_2 -noservername` confirmed that
`ttkg-fw.thethingsindustries.com` requires the SNI (Server Name Indication) TLS
extension and returns a fatal handshake alert (alert 40) without it. The
gateway's mbedTLS glue layer never called `mbedtls_ssl_set_hostname()`, so SNI
was never included in the TLS ClientHello for any HTTPS connection. The HTTPS
connections to `eu1.cloud.thethings.network` (config, frequency plan) happened
to work without SNI; the FOTA server did not.

**Fix:** Added `NET_PRES_EncGlue_SetSNIHostname()` to the mbedTLS glue layer.
`app_http_request.c` calls it with `request.host` before encrypting the socket.
The glue layer stores the hostname and calls `mbedtls_ssl_set_hostname()` with it
just before the TLS handshake.

After deploying 1.1.5, FOTA TLS negotiated successfully and the firmware check
completed. However, MQTT immediately failed — see step 3.

**Files changed:**
- `firmware/src/system_config/TTN_Gateway_v1/framework/net/pres/net_pres_enc_glue.h` — declared `NET_PRES_EncGlue_SetSNIHostname()`
- `firmware/src/system_config/TTN_Gateway_v1/framework/net/pres/net_pres_enc_glue.c` — added `_sni_hostname` global, `NET_PRES_EncGlue_SetSNIHostname()`, and `mbedtls_ssl_set_hostname()` call before handshake
- `firmware/src/app_http_request.c` — call `NET_PRES_EncGlue_SetSNIHostname(request.host)` before `NET_PRES_SocketEncryptSocket()`

---

#### Step 3 — Fix: prevent SNI hostname from leaking across connections (1.1.6)

**Problem:** After deploying 1.1.5, MQTT failed on every boot with
`TLS Connection Negotiation Failed`. The MQTT broker connection follows FOTA in
the boot sequence (state 4 → state 5). The `_sni_hostname` global was set to
`ttkg-fw.thethingsindustries.com` during the FOTA HTTPS request and never
cleared. When the MQTT TLS handshake ran, mbedTLS sent SNI for the wrong hostname
to the MQTT broker (`eu1.cloud.thethings.network`), which rejected the handshake.

**Fix:** Clear `_sni_hostname` immediately after passing it to
`mbedtls_ssl_set_hostname()`, so the hostname is consumed exactly once per
connection and cannot leak into subsequent connections that do not explicitly set
a hostname (such as MQTT).

**Files changed:**
- `firmware/src/system_config/TTN_Gateway_v1/framework/net/pres/net_pres_enc_glue.c` — reset `_sni_hostname[0]` to `'\0'` after calling `mbedtls_ssl_set_hostname()`

---

## Verified behaviour after 1.1.6

All TLS connections succeed on every boot:

| Connection | Host | Port | Result |
|------------|------|------|--------|
| HTTPS config | `eu1.cloud.thethings.network` | 443 | OK |
| HTTPS frequency plan | `eu1.cloud.thethings.network` | 443 | OK |
| HTTPS FOTA key + firmware check | `ttkg-fw.thethingsindustries.com` | 443 | OK |
| MQTTS | `eu1.cloud.thethings.network` | 8881 | OK |

Gateway bridges LoRa traffic and sends uplinks normally.

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
