# Gateway Web Interface

The pages in `static/` are compiled into the firmware as an MPFS2 image
(`../firmware/src/mpfs_img2.c`).

## Generating the firmware package

```sh
python3 gen_mpfs.py
```

This must be run after every change in `static/`; the firmware build does not do it. The
original Microchip tool does the same:

```sh
java -jar ~/microchip/harmony/v1_08_01/utilities/mpfs_generator/mpfs2.jar /c ./static ../firmware/src mpfs_img2
```

## Testing

`test_webui.py` is the acceptance test of the web interface. It needs only Python 3.

```sh
./test_webui.py                                  # source checks, no gateway needed
./test_webui.py --gateway <ip>                   # + read-only checks on a running gateway
./test_webui.py --gateway <ip> --write           # + UDP settings round trip (restores them)
./test_webui.py --gateway <ip> --serial-log <console capture>
./test_webui.py --selftest                       # test the test against a built-in mock
```

| Part | What it checks |
|---|---|
| Sources | every page has the header include and one footer token, inside the form; every `~token~` is in `gen_mpfs.py`, in the dispatch table of `http_print.c` and has a callback; the footer callback (repository link, version from `version.h`, no separator line, one copy); local links and element ids; `mpfs_img2.c` equals `static/` file by file and its dynamic variable index matches |
| Live, read only | every page: 200, header, one footer with the running firmware version, identical on all pages, served text equals header + source + footer; `status.cgi`, `settings.cgi`, `udp.cgi`, `ssids.cgi`, `id.cgi`: JSON fields and types; static files byte for byte; 404 for an unknown file; repeated and parallel requests; no reboot |
| Live, writing | `udp.cgi`: a valid POST redirects to `udp.html` and is read back; port 0, port 70000, a non-numeric port, enabled without server and an over-long server name redirect to the error page and change nothing; the original settings are restored |
| Console (`--serial-log`) | no exception, assert, watchdog line, allocation failure or reboot on the debug console while the test runs |

The test never posts to `settings.cgi`: that stores the WiFi and activation configuration and
restarts networking. Saving that form, and how the pages look, are listed as MANUAL at the end
of a run and have to be checked in a browser.

Acceptance of a web interface change: source checks pass, then on a gateway running the build
`./test_webui.py --gateway <ip> --write --serial-log <capture>` passes and the MANUAL points
are looked at.
