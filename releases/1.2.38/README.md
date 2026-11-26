# Firmware Release 1.2.38 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.38                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-22                   |
| Commit      | 18ca7ab                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.38](https://github.com/pe1mew/gateway/releases/tag/v1.2.38):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
72620cfc2f0db1b6f1daae2440766328a5073fbd937eb89f542f401445655a8c  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

One web UI change and two bug fixes. No setting change.

### Footer with the firmware version on every web UI page (issue #7)

Every page of the web UI ends with a footer that shows the firmware version and a link to this
repository:

```
Firmware 1.2.38 · github.com/pe1mew/gateway
```

The text comes from one place in the firmware (the dynamic variable `~gwfooter~`), so the version
always matches the running image.

- New acceptance test for the web UI, `webui/test_webui.py`: it checks the page sources, and with
  `--gateway <ip>` the pages and JSON interfaces of a running gateway. See
  [`webui/README.md`](../../webui/README.md).
- The UDP forwarder page shows a "Save failed" alert when the gateway does not accept the
  settings. The alert was in the page source since the page was added, but the web image compiled
  into the firmware had not been regenerated.

### Saving the UDP forwarder settings during start-up (issue #10)

Saving the UDP forwarder settings moved the gateway to its operational state from any state. When
the save arrived while the gateway was still loading its configuration from The Things Network,
the rest of the start-up was skipped: the LoRa module was not started and the gateway reconnected
to the network every 8 seconds until it was rebooted.

- The settings are stored only while the gateway is operational or waiting for a network
  connection. In any other state the save waits until start-up has passed that point; the console
  prints

```
MAIN: UDP config changed in state 3, storing it later
```

- After a save in a wait state the gateway continues where it was (`MAIN: Resuming state 2`)
  instead of entering the operational state.

Known limitation: a save that is still waiting is lost when the gateway reboots before it is
stored. This happens when the settings are saved while the gateway downloads a firmware update:
the web UI accepts the save, the update completes, and after the reboot the previous settings are
still in place. Save the settings again after the update.

### Public sync word with the built-in frequency plan (issue #13)

The gateway has a built-in frequency plan for when no plan is available or the downloaded plan
does not fit the band of the LoRa card. The sync word was then still taken from the downloaded
plan; without a plan that is the private sync word, with which the gateway receives only a
fraction of the public LoRaWAN traffic. In the current start-up sequence the LoRa module is only
started after a plan was downloaded, so gateways in normal use were not affected.

- With the built-in plan the sync word is always the public one. With a valid downloaded plan
  nothing changes.
- The console prints the sync word in use:

```
LORA: sync word: public (0x34)
```

### Validation on the test gateway (2026-11-22)

Saving the UDP forwarder settings (issue #10):

| Save issued while | Before | With the fix |
|---|---|---|
| operational | stored, bridging again after 1 s | the same |
| loading the configuration (state 3) | endless reconnect loop, reboot needed | stored after start-up, bridging |
| waiting for the network (state 2) | not tested | stored, start-up continues, bridging |
| waiting for the network in access point mode, no Ethernet (state 2) | not tested | stored, access point client stays connected; bridging after the cable is plugged in |
| downloading a firmware update (state 4) | not tested | update completes, single boot; the save is not stored (known limitation) |
| network lost during operation (state 6) | not tested | stored, bridging 5 s later, no reboot |

Built-in EU frequency plan forced by a diagnostic build (issue #13), uplinks received in
10 minutes at the same location:

| | Sync word | Uplinks |
|---|---|---|
| downloaded plan (reference) | public | 35–36 |
| built-in plan, before | private | 8 |
| built-in plan, with the fix | public | 37 |

On this image: web UI acceptance test 313 passed, 0 failed; HTTP stress without failures
(41–44 KB heap free, lowest free 37 KB).

Not tested: a gateway that uses WiFi as its uplink, and the built-in US915 plan.

Soak on this image (2026-11-22 12:13 to 22:00, 9 hours 45 minutes; shorter than the 24 hours of
earlier releases, so the firmware's own 24-hour reboot was not part of it) passed: no reboot, no
exception or assert, heap flat (289–293 KB used, 41–45 KB free, largest free block 30160 bytes,
lowest free 38 KB); no MQTT error or reconnect and no TLS timeout; 2074 uplinks delivered to The
Things Network without a failed send, 505 downlinks received, UDP forwarder acknowledged 99–100 %
after the first minutes. The first four minutes of this boot were the access point test from the
table above.

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
