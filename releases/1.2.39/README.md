# Firmware Release 1.2.39 — RemkoWelling

| Field       | Value                        |
|-------------|------------------------------|
| Version     | 1.2.39                       |
| Name        | Remko Welling                |
| Type        | 0 (production)               |
| Built       | 2026-11-24                   |
| Commit      | 487de87                      |

## Files

The firmware image is not stored in the repository. Download both files from the GitHub release
[v1.2.39](https://github.com/pe1mew/gateway/releases/tag/v1.2.39):

| File           | Description                                                |
|----------------|------------------------------------------------------------|
| `firmware.hex` | Intel HEX firmware image for SD card, PICkit 4 or FOTA     |
| `checksums`    | SHA-256 checksum for integrity verification                |

### SHA-256

```
d8a5707b34a95e6af1a30ccdeea83b73d9487888356286363e836970a745f574  firmware.hex
```

Verify before flashing:
```bash
sha256sum -c checksums
```

---

## Changes

The Info page of the web UI and `status.cgi` tell more, and what they already told is now
correct. One bug fix: no reset any more when the MQTT connection is rebuilt after a network
re-initialisation. No setting change.

### Broker connection and packet counters (issue #12)

"Broker connection" on the Info page (`connbroker` in `status.cgi`) was true whenever the gateway
was in its operational state, also while the MQTT connection to The Things Network was being
rebuilt. The packet counters counted only the traffic of The Things Network, and the uplink
counter also counted uplinks whose MQTT send had failed.

- `connbroker` is true only while the MQTT connection is up.
- `pup` counts only the uplinks that were delivered to the broker. The same counter is reported
  in the status message to The Things Network.
- New fields `udpup` (uplinks sent to the UDP server) and `udpdown` (downlinks accepted from it).
- The Info page shows both pairs:

| Row on the Info page | `status.cgi` |
|---|---|
| Broker packets up / down (was: Packets up / down) | `pup`, `pdown` |
| UDP packets up / down (new) | `udpup`, `udpdown` |

- The console prints a line for every downlink from the UDP server; until now a UDP downlink left
  no line of its own:

```
UDP: PULL_RESP accepted
UDP: PULL_RESP rejected (LATE)
```

### Frequency plan in use (issue #14)

When the gateway runs on its built-in frequency plan, the web UI showed only
`Config correct: false`, the same value as during start-up.

- New field `freqplan` in `status.cgi` and a row "Frequency plan" on the Info page:

| `freqplan` | Info page | Meaning |
|---|---|---|
| `none` | LoRa module not configured yet | start-up |
| `downloaded` | Downloaded | the plan from The Things Network is in use |
| `builtin` | Built-in (no plan downloaded, or it does not fit the gateway card) | the built-in fallback plan is in use |

- `configured` is unchanged: it is true exactly when `freqplan` is `downloaded`.

### Uptime as hh:mm:ss (issue #15)

- The Info page shows the uptime as `hh:mm:ss` instead of a number of seconds, with the days in
  front when there are any (`1 d 00:00:05`).
- A line under it says that the gateway restarts every 24 hours to check for a firmware update.
- `status.cgi` still gives `uptime` in seconds, so scripts that read it keep working.

### No reset after a network re-initialisation (issue #17)

- Every MQTT connect allocated a 16 KB stack for the MQTT read task. After the network had been
  re-initialised (an outage longer than a few seconds) the free heap could be laid out so that
  no free block of that size was left, although 58 KB was free. The failed allocation reset the
  gateway (`vApplicationMallocFailedHook`); it was bridging again 35 s later.
- This happened once, in the 24 h test of the first candidate of this version, after 7.5 hours
  and during a real internet outage. The same code is in the earlier releases.
- The task uses 2.3 KB of that stack (564 words, measured over 18 connections with uplinks and
  downlinks). It now gets 4 KB (`MQTT_TASK_STACK_WORDS` in `firmware/src/MQTTHarmony.c`).
- `firmware/src/MQTTHarmony.c` is a copy of the file in the `paho.mqtt.embedded-c` submodule and
  is built in its place; the stack size is its only change.
- While connected the gateway has 12 KB more heap free (56 KB instead of 44 KB).

### Validation on the test gateway (2026-11-23 and 2026-11-24)

`connbroker`, read every 0.4 s while the UDP forwarder settings were saved twice (each save
rebuilds the MQTT connection):

| Time | Console | `connbroker` |
|---|---|---|
| 19:03:34 | bridging | true |
| 19:03:40.2 | settings stored, MQTT connection rebuilt | false |
| 19:03:41.3 | `MAIN: Gateway bridging` | true |
| 19:03:44.0 | second save | false |
| 19:03:45.4 | `MAIN: Gateway bridging` | true |

During start-up it stays false until `MAIN: Gateway bridging`. On 1.2.38 it was true for the
whole time (about 5 s after a save).

Counters against the console lines of the same boot:

| `status.cgi` | Value | Console |
|---|---|---|
| `udpup` | 52 | 52 × `UDP: PUSH_DATA sent` |
| `pup` | 52 | 52 × `MQTT: Sending UPLINK OK` |
| `udpdown` | 3 | 3 × `UDP: PULL_RESP accepted` |
| `pdown` | 4 | 4 × `MQTT: Received DOWNLINK` |

In an earlier boot one uplink arrived before the MQTT connection was up: `udpup` 16, `pup` 15.
A save of the UDP forwarder settings does not reset the counters.

`freqplan`: `none` during start-up and `downloaded` in normal operation; `builtin` (with
`configured` false) on a diagnostic build that forces the built-in plan.

Issue #17, on diagnostic builds that force a network re-initialisation every 3 minutes and print
the heap layout:

| | 16 KB stack | 4 KB stack |
|---|---|---|
| Forced re-initialisations | 53 | 5 |
| Failed allocations, resets | 0 | 0 |
| Largest free block while bridging | 29600 B, or 15104 B after 24 of 54 connects | 45696 B, or 28944 B after 1 of 6 connects |
| Lowest free heap | 37 KB | 50 KB |
| Unused stack of the MQTT task | 3532 of 4096 words | 460 of 1024 words |

The failed allocation itself was not reproduced with forced outages, so the fix is shown by the
larger free blocks, not by a failure that disappeared.

On this image: web UI acceptance test 322 passed, 0 failed; HTTP stress without failures (56 KB
heap free, largest free block 39824 B or more, lowest free 49 KB).

Not tested: the uplink counter when an MQTT send fails, a downlink that the UDP forwarder rejects
as too late, and a real frequency plan for the wrong band (it takes the same path as the forced
case).

Soak on this image (2026-11-24 12:52 to 2026-11-25 12:52, 24 hours, ended by the firmware's own
24-hour reboot) passed: no reset, no exception, assert or failed allocation; heap flat (278–281 KB
used, 53–56 KB free, largest free block 46032 bytes in every sample, lowest free 49 KB). Seven
MQTT errors, each handled by an MQTT-only reconnect, bridging again within 1 to 6 seconds; no TLS
timeout. 5418 uplinks delivered to The Things Network, 5 failed sends (one at each of five of the
MQTT errors), 954 downlinks received; the `status.cgi` counters matched the console. The planned
reboot came back in a single boot, all TLS connections at the first attempt, bridging after
40 seconds.

No network re-initialisation occurred in these 24 hours, so the situation of issue #17 was not
met by a real outage in this soak; for that fix the forced re-initialisations in the table above
are the evidence.

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
