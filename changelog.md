# Changelog

All notable changes to the firmware in this repository are documented in this file, newest first.

The format is based on [Keep a Changelog](https://keepachangelog.com/). Versions are numbered `major.minor.patch`. Version numbers that are missing from this list were test or diagnostic builds and were never released. Each release has its full notes, with the measurements, in `releases/<version>/README.md`; the firmware images are attached to the [GitHub releases](https://github.com/pe1mew/gateway/releases) and are not stored in the repository.

Versions up to 1.0.10 are the original firmware of The Things Products; their history is in the commit log of [TheThingsProducts/gateway](https://github.com/TheThingsProducts/gateway).

---

## [1.2.38] — Unreleased  (web UI footer with the firmware version, UDP settings can be saved during start-up)

Web UI change and two bug fixes. No setting change.

**Added:**

- **Footer on every web UI page** (#7) with the firmware version (`x.y.z`) and a link to this repository. The text comes from one place in the firmware (`~gwfooter~`), so the version always matches the running image.
- **Acceptance test for the web UI**, `webui/test_webui.py`: checks the page sources, and with `--gateway <ip>` the pages and JSON interfaces of a running gateway; see [`webui/README.md`](webui/README.md).

**Fixed:**

- **UDP forwarder page: "Save failed" alert** when the gateway does not accept the settings. The alert was in the page source since the page was added, but the web image compiled into the firmware had not been regenerated.
- **Saving the UDP forwarder settings during start-up no longer breaks the connection to The Things Network** (#10). The save moved the gateway to the operational state from any state; when that happened while it was still loading its configuration, the rest of the start-up was skipped and the gateway reconnected endlessly until it was rebooted. The settings are now stored only while the gateway is operational or waiting for a network connection; in any other state the save waits until start-up has passed that point (console line `MAIN: UDP config changed in state N, storing it later`). A save that is still waiting is lost if the gateway reboots first.
- **The built-in frequency plan always uses the public LoRaWAN sync word** (#13). The built-in plan is used when no frequency plan is available or when the downloaded plan does not fit the band of the LoRa card. The sync word was then still taken from the downloaded plan: without a plan that is the private sync word, and the gateway received only a fraction of the public LoRaWAN traffic (8 uplinks in 10 minutes on the test gateway, against 37 with the fix). With a valid downloaded plan nothing changes. The console now prints the sync word in use (`LORA: sync word: public (0x34)`).

---

## [1.2.37] — 2026-11-22  (random TCP source ports, no more stalled TLS handshakes after a reboot)

Bug fix. No setting or web UI change. Details: [`releases/1.2.37/README.md`](releases/1.2.37/README.md). This is the first release without a firmware image in the repository; the image is attached to the GitHub release.

**Fixed:**

- **The pseudo-random generator is seeded from the hardware random number generator at boot** (#6). It was seeded with the same constant at every boot, and the TCP/IP stack takes the source port and the initial sequence number of every connection from it, so after each reboot the gateway connected to the same servers from the same ports. In a row of reboots about 5 in 97 TLS handshakes stalled until the 10 second timeout; with the fix none in 90. The boot banner reports `PRNG seed: 64 hardware random bits`. Sequence numbers and DNS transaction ids are no longer predictable either.

**Documented:**

- **Double free on every WiFi shutdown** (#4): it is in Microchip's closed MRF24WN library and cannot be fixed in source. The heap tracker ignores the second free; the check now carries a comment that it must stay. No functional change.

## [1.2.35] — 2026-11-20  (no more watchdog resets during start-up)

Bug fixes. No setting or web UI change. Details: [`releases/1.2.35/README.md`](releases/1.2.35/README.md). Versions 1.2.33 and 1.2.34 were test builds.

**Fixed:**

- **The LoRa module UART has separate read and write handles** (#3). One driver handle served the blocking read of the receive thread and the blocking writes of the LoRa task; the driver has one buffer object per handle, so the LoRa task could spin forever after a command and the watchdog reset the gateway.
- **The TLS handshake no longer blocks the system task** (#5). It ran as a busy loop, so the watchdog was not cleared when the server's data was late. It now does one step per call, and the HTTP and MQTT clients give up a TLS connect after 10 seconds and retry.
- **The startup stack is at least 4096 bytes.** RAM was completely in use and about 700 bytes were left. The FreeRTOS heap is reduced from 350000 to 346000 bytes.

**Added:** console lines `*** WDT pending …` when the watchdog has not been cleared for 6 seconds, naming the task that is running.

In a test of 30 reboots in a row 1.2.32 had 10 watchdog resets and 1.2.35 none.

## [1.2.32] — 2026-11-19  (no more reboots when the web UI is used after days of uptime)

Bug fix. No setting or web UI change. Details: [`releases/1.2.32/README.md`](releases/1.2.32/README.md). Version 1.2.31 was a test build.

**Fixed:**

- **The heap no longer fragments** (#2). The firmware used the FreeRTOS allocator `heap_2`, which never merges freed blocks. Web UI requests and TLS sessions fragmented the heap until the TLS session opened after an MQTT publish timeout (about 75 KB in four large blocks) could not be allocated, and the gateway reset. An HTTPS FOTA URL could make a gateway boot-loop for the same reason. The allocator is now `heap_4`, which merges free blocks.

**Changed:**

- **An MQTT error reconnects MQTT only.** The whole network stack is re-initialised only after three failures in a row.
- The console `MON:` line reports the largest free block and the minimum free heap.

## [1.2.30] — 2026-11-18  (UDP forwarder: uplinks accepted by ChirpStack)

Bug fix. Details: [`releases/1.2.30/README.md`](releases/1.2.30/README.md).

**Fixed:**

- **The `freq` field of forwarded uplinks is exact** (#1). It was formatted through a `double`, which is 32 bits wide with this compiler: 868300000 Hz was sent as 868.300048 MHz. ChirpStack rejected such frames ("Unknown channel for frequency"); The Things Network tolerated them. Frequencies are now formatted and parsed with integer arithmetic.
- The forwarder discards queued uplinks while it is disabled, so old packets are not sent when it is enabled again.

## [1.2.29] — 2026-04-21  (UDP forwarder: acknowledgement rate)

Bug fix. Details: [`releases/1.2.29/README.md`](releases/1.2.29/README.md).

**Fixed:** the `ackr` field of the gateway status report could exceed 100 %: status messages were acknowledged by the server but not counted as sent.

## [1.2.28] — 2026-04-20  (Semtech UDP packet forwarder, FOTA URL override)

New features. Details: [`releases/1.2.28/README.md`](releases/1.2.28/README.md).

**Added:**

- **Semtech UDP packet forwarder** next to the MQTT connection to The Things Network, for network servers that use the Semtech UDP protocol (port 1700 by default), such as ChirpStack. It runs at the same time as MQTT, can be set to uplink only, and is configured on the new UDP page of the web UI. The settings are stored in the serial flash.
- **FOTA URL override:** a firmware update server of your own can be set on the Settings page of the web UI, over HTTP or HTTPS. With the override enabled the gateway takes its updates from that server and not from The Things Network. A server for it: [pe1mew/ttksgfotasrv](https://github.com/pe1mew/ttksgfotasrv).

## [1.1.6] — 2026-04-19  (firmware updates over TLS work again)

Bug fixes. Details: [`releases/1.1.6/README.md`](releases/1.1.6/README.md). Versions 1.1.4 and 1.1.5 were the intermediate steps.

**Fixed:**

- **HTTPS requests send the server name (SNI).** The firmware update server of The Things Industries refuses a TLS handshake without it, so every update check failed.
- **The server name of one connection is no longer used for the next.** After the SNI fix the MQTT connection failed because it inherited the name of the update server.

**Added:** the firmware update URL is printed on the console.

## [1.1.3] — 2026-04-18  (WiFi access point off when not needed, renewed CA certificates)

Bug fixes. Details: [`releases/1.1.3/README.md`](releases/1.1.3/README.md).

**Fixed:**

- **The WiFi access point is switched off when the gateway is connected** over Ethernet or WiFi. It stayed on and occupied the 2.4 GHz band. It comes back within 10 seconds when a WiFi connection fails, so the web UI stays reachable for new credentials.
- **Expired CA certificates replaced.** Two of the four root certificates in the firmware had expired, so all TLS connections failed. Baltimore CyberTrust Root is removed, Amazon Root CA 1 is renewed (valid to 2038) and Starfield Services Root CA G2 is added (valid to 2037). ISRG Root X1 and DigiCert Global Root G2 are unchanged.
