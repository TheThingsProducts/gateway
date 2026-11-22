#!/usr/bin/env python3
"""
Acceptance test for the gateway web UI.

Three parts, each can be run on its own:

  1. Source checks (always run, no gateway needed)
     The page sources in ./static, the token table in gen_mpfs.py, the dispatch table in
     ../firmware/src/http_print.c, the callbacks in ../firmware/src/custom_http_app.c and the
     generated image ../firmware/src/mpfs_img2.c must agree with each other.

  2. Live checks, read only (--gateway <ip>)
     Fetches every page, CGI endpoint and static file from a running gateway and compares
     them with the sources: header and footer on every page, footer version equal to the
     running firmware, JSON fields and types, static files byte for byte, 404 for unknown
     files, repeated and parallel requests.

  3. Live checks that write (--gateway <ip> --write)
     Changes the UDP forwarder settings through udp.cgi, checks the redirects and the
     rejection of invalid input, and restores the original settings. It never posts to
     settings.cgi (that stores the WiFi/activation configuration and restarts networking).

Usage:
  ./test_webui.py                                   source checks only
  ./test_webui.py --gateway 192.168.1.10            + live read-only checks
  ./test_webui.py --gateway 192.168.1.10 --write    + UDP settings round trip
  ./test_webui.py --gateway <ip> --expect-version 1.2.38 --serial-log <console capture>
  ./test_webui.py --selftest                        test the test against a built-in mock

Options:
  --expect-version X.Y.Z   the footer and status.cgi must report this version
                           (default: ../firmware/version/{major,minor,patch})
  --any-version            do not compare with the version files, only footer == status.cgi
  --rounds N               repetitions of the request loop (default 5)
  --serial-log FILE        console capture that is being written during the test; the test
                           fails when an exception, assert, watchdog line or a reboot
                           appears in it while the test runs

Exit status 0 when no check failed. Lines marked MANUAL at the end list what a person has
to look at in a browser; they are not part of the exit status.
"""

import argparse
import http.client
import http.server
import json
import os
import re
import struct
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, 'static')
SRC = os.path.join(HERE, '..', 'firmware', 'src')
VERSION_DIR = os.path.join(HERE, '..', 'firmware', 'version')

PAGES = ['index.html', 'info.html', 'udp.html', 'activate.html', 'error.html', 'reconnect.html']
REPO_URL = 'https://github.com/pe1mew/gateway/'
FOOTER_RE = re.compile(
    r'<center style="[^"]*">Firmware (\d+\.\d+\.\d+) &middot; '
    r'<a href="' + re.escape(REPO_URL) + r'"[^>]*>github\.com/pe1mew/gateway</a></center>')

STATUS_FIELDS = {
    'gateway': bool, 'hwversion': str, 'blversion': str, 'fwversion': str, 'uptime': int,
    'connected': bool, 'interface': str, 'ssid': str, 'activation_locked': bool,
    'configured': bool, 'freqplan': str, 'region': str, 'gwcard': str, 'connbroker': bool, 'connudp': bool,
    'pup': int, 'pdown': int, 'udpup': int, 'udpdown': int, 'estor': bool,
}
SETTINGS_FIELDS = {'ssid': str, 'gwid': str, 'gwkey': str, 'asrv': str, 'locked': bool,
                   'fotaen': bool, 'fotaul': str}
# Element ids that only unused functions of mchp.js refer to (left over from the Microchip demo
# and from an earlier UDP form): removeCheckmark/addScanRow/userSelectNetwork use "scanTable",
# switchNetwork/printButtonName use "rescan", prepareUDPForm uses the two "_val" ids. No page
# calls these functions.
DEAD_CODE_IDS = {'scanTable', 'rescan', 'udpen_val', 'uponly_val'}

UDP_FIELDS = {'enabled': bool, 'server': str, 'portup': int, 'portdn': int, 'uponly': bool,
              'gateway_eui': str}


class Report:
    def __init__(self, quiet=False):
        self.passed = 0
        self.failed = 0
        self.failures = []
        self.quiet = quiet

    def section(self, title):
        if not self.quiet:
            print('\n== ' + title)

    def check(self, ok, name, detail=''):
        if ok:
            self.passed += 1
            if not self.quiet:
                print('  PASS  ' + name)
        else:
            self.failed += 1
            self.failures.append(name)
            if not self.quiet:
                print('  FAIL  ' + name + ((' -- ' + detail) if detail else ''))
        return ok

    def info(self, text):
        if not self.quiet:
            print('  INFO  ' + text)


def read(path, mode='rb'):
    with open(path, mode) as f:
        return f.read()


def tokens_in(data):
    """Same tokenizer as gen_mpfs.py: list of (offset, name) for ~name~."""
    out = []
    i = 0
    while i < len(data):
        if data[i:i + 1] == b'~':
            j = data.find(b'~', i + 1)
            if j != -1:
                try:
                    out.append((i, data[i + 1:j].decode('ascii')))
                except UnicodeDecodeError:
                    i += 1
                    continue
                i = j + 1
                continue
        i += 1
    return out


def static_files():
    out = {}
    for root, dirs, names in os.walk(STATIC):
        for n in names:
            p = os.path.join(root, n)
            out[os.path.relpath(p, STATIC).replace(os.sep, '/')] = read(p)
    return out


def expected_version():
    return '.'.join(read(os.path.join(VERSION_DIR, k), 'r').strip() for k in ('major', 'minor', 'patch'))


# ---------------------------------------------------------------------------------------
# Part 1: source checks
# ---------------------------------------------------------------------------------------

def load_token_table():
    text = read(os.path.join(HERE, 'gen_mpfs.py'), 'r')
    block = text[text.index('TOKEN_TO_ID = {'):]
    block = block[:block.index('}')]
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"'([^']+)':\s*(\d+)", block)}


def load_dispatch():
    """callback id -> ('include', file) or ('print', function name) from http_print.c."""
    text = read(os.path.join(SRC, 'http_print.c'), 'r')
    out = {}
    for m in re.finditer(r'case (0x[0-9A-Fa-f]+):\s*(.*?)break;', text, re.S):
        body = m.group(2)
        inc = re.search(r'TCPIP_HTTP_FileInclude\(connHandle, \(const uint8_t\*\)"([^"]+)"\)', body)
        prn = re.search(r'TCPIP_HTTP_Print_(\w+)\(connHandle\)', body)
        if inc:
            out[int(m.group(1), 16)] = ('include', inc.group(1))
        elif prn:
            out[int(m.group(1), 16)] = ('print', prn.group(1))
    return out


def decode_image():
    text = read(os.path.join(SRC, 'mpfs_img2.c'), 'r')
    body = text[text.index('NVM_MEDIA_DATA[DRV_NVM_MEDIA_SIZE*1024] = {'):]
    body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
    img = bytes(int(x, 16) for x in re.findall(r'0x([0-9a-fA-F]{2})', body))
    n = struct.unpack_from('<H', img, 6)[0]
    fat = 8 + n * 2
    files = {}
    for i in range(n):
        s, d, length, ts, mt, fl = struct.unpack_from('<IIIIIH', img, fat + i * 22)
        files[img[s:img.index(b'\0', s)].decode('ascii')] = (img[d:d + length], fl)
    return img, files


def name_hash(name):
    h = 0
    for c in name:
        h = ((h + ord(c)) << 1) & 0xFFFF
    return h


def source_checks(r):
    files = static_files()
    table = load_token_table()
    dispatch = load_dispatch()
    app_c = read(os.path.join(SRC, 'custom_http_app.c'), 'r')

    r.section('Sources: pages')
    for page in PAGES:
        r.check(page in files, f'{page} exists')
    header = files.get('header.inc', b'')
    r.check(b'<body' in header and b'<form' in header, 'header.inc opens body and form')
    for page in PAGES:
        data = files.get(page, b'')
        toks = [n for _, n in tokens_in(data)]
        r.check(data.startswith(b'~inc:header.inc~'), f'{page} starts with the header include')
        r.check(toks.count('gwfooter') == 1, f'{page} has exactly one footer token',
                f'found {toks.count("gwfooter")}')
        pos = data.find(b'~gwfooter~')
        tail = data[pos + len(b'~gwfooter~'):] if pos != -1 else b''
        r.check(pos != -1 and not re.sub(rb'\s|</form>|</body>|</html>', b'', tail),
                f'{page} footer is the last content of the page')
        end_form = data.find(b'</form>')
        r.check(pos != -1 and (end_form == -1 or pos < end_form),
                f'{page} footer is inside the form (the white card)')
        r.check(data.rstrip().endswith(b'</html>'), f'{page} ends with </html>')

    r.section('Sources: tokens, dispatch table and callbacks')
    used = set()
    for name, data in sorted(files.items()):
        for _, tok in tokens_in(data):
            used.add(tok)
            r.check(tok in table, f'token ~{tok}~ in {name} is in gen_mpfs.py TOKEN_TO_ID')
    ids = list(table.values())
    r.check(len(ids) == len(set(ids)), 'callback IDs in TOKEN_TO_ID are unique')
    for tok, cid in sorted(table.items(), key=lambda kv: kv[1]):
        d = dispatch.get(cid)
        if tok.startswith('inc:'):
            ok = d == ('include', tok[4:])
            r.check(ok, f'ID {cid} ~{tok}~ includes {tok[4:]} in http_print.c', str(d))
            r.check(tok[4:] in files, f'included file {tok[4:]} exists')
        else:
            ok = d == ('print', tok)
            r.check(ok, f'ID {cid} ~{tok}~ calls TCPIP_HTTP_Print_{tok} in http_print.c', str(d))
            r.check(re.search(r'^void TCPIP_HTTP_Print_' + re.escape(tok) + r'\(HTTP_CONN_HANDLE', app_c, re.M)
                    is not None, f'TCPIP_HTTP_Print_{tok}() is defined in custom_http_app.c')
        r.check(tok in used, f'token ~{tok}~ is used by a page or CGI file')

    r.section('Sources: footer callback')
    m = re.search(r'void TCPIP_HTTP_Print_gwfooter\(.*?\n}', app_c, re.S)
    body = m.group(0) if m else ''
    r.check(bool(m), 'footer callback found')
    r.check(REPO_URL in body, 'footer links to ' + REPO_URL)
    r.check(all(k in body for k in ('VERSION_MAJOR', 'VERSION_MINOR', 'VERSION_PATCH')),
            'footer version comes from version.h (major, minor, patch)')
    r.check(not re.search(r'<hr|border', body, re.I), 'footer has no separator line')
    r.check(app_c.count(REPO_URL) == 1, 'footer text exists once in custom_http_app.c')
    r.check(not any(REPO_URL.encode() in d for d in files.values()),
            'no page carries its own copy of the footer link')

    r.section('Sources: links and element ids')
    html_all = b''.join(d for n, d in files.items() if n.endswith(('.html', '.inc')))
    for name in PAGES + ['header.inc']:
        for ref in re.findall(rb'(?:href|src|action)="([^"#:?]+)"', files.get(name, b'')):
            ref = ref.decode().lstrip('./')
            r.check(ref in files, f'{name} refers to {ref}, which exists')
    ids_html = set(m.decode() for m in re.findall(rb'id="([^"]+)"', html_all))
    ids_js = set()
    for name in ['mchp.js'] + PAGES:
        ids_js |= set(m.decode() for m in re.findall(rb"getElementById\(['\"]([A-Za-z_0-9]+)['\"]\)", files.get(name, b'')))
    # ids the scripts create themselves at run time
    ids_made = set(m.decode() for m in re.findall(rb"setAttribute\('id',\s*'([A-Za-z_0-9]+)'\)", files.get('mchp.js', b'')))
    for i in sorted(ids_js):
        if i in DEAD_CODE_IDS and i not in ids_html:
            r.info(f'element id "{i}" is only used by unused script functions (no page has it)')
            continue
        r.check(i in ids_html or i in ids_made, f'element id "{i}" used by the scripts exists in a page')

    r.section('Sources: generated image mpfs_img2.c is up to date')
    img, in_image = decode_image()
    r.check(img[:6] == b'MPFS\x02\x01', 'image has the MPFS2 signature')
    for name, data in sorted(files.items()):
        ok = name in in_image and in_image[name][0] == data
        r.check(ok, f'{name} in the image equals the source file',
                'missing from the image' if name not in in_image else
                f'image {len(in_image[name][0])} bytes, source {len(data)} bytes: run gen_mpfs.py')
    for name in sorted(set(in_image) - set(files) - {'FileRcrd.bin', 'DynRcrd.bin'}):
        r.check(False, f'{name} is in the image but not in the sources')
    filercrd = in_image.get('FileRcrd.bin', (b'', 0))[0]
    dynrcrd = in_image.get('DynRcrd.bin', (b'', 0))[0]
    index = {}
    if len(filercrd) >= 4:
        for i in range(struct.unpack_from('<I', filercrd, 0)[0]):
            h, off, cnt = struct.unpack_from('<HII', filercrd, 4 + i * 10)
            index[h] = [struct.unpack_from('<II', dynrcrd, off + 6 + k * 8) for k in range(cnt)]
    for name, data in sorted(files.items()):
        want = [(off, table[tok]) for off, tok in tokens_in(data) if tok in table]
        if want or name_hash(name) in index:
            r.check(index.get(name_hash(name)) == want, f'dynamic variable index of {name} matches its tokens')
    cfg = read(os.path.join(SRC, 'system_config', 'TTN_Gateway_v1', 'system_config.h'), 'r')
    m = re.search(r'#define\s+DRV_NVM_MEDIA_SIZE\s+(\d+)', cfg)
    if m:
        limit = int(m.group(1)) * 1024
        r.check(len(img) <= limit, f'image ({len(img)} bytes) fits the reserved flash ({limit} bytes)')


# ---------------------------------------------------------------------------------------
# Part 2 and 3: live checks
# ---------------------------------------------------------------------------------------

class Gateway:
    def __init__(self, host, timeout=15):
        self.host = host
        self.timeout = timeout

    def request(self, method, path, body=None):
        """Returns (status, headers dict lower-case, body bytes); does not follow redirects."""
        host, _, port = self.host.partition(':')
        conn = http.client.HTTPConnection(host, int(port) if port else 80, timeout=self.timeout)
        try:
            headers = {'Connection': 'close'}
            if body is not None:
                headers['Content-Type'] = 'application/x-www-form-urlencoded'
            conn.request(method, path, body=body, headers=headers)
            resp = conn.getresponse()
            data = resp.read()
            return resp.status, {k.lower(): v for k, v in resp.getheaders()}, data
        finally:
            conn.close()

    def get(self, path):
        return self.request('GET', path)

    def post(self, path, fields):
        return self.request('POST', path, urllib.parse.urlencode(fields))


def page_pattern(source, header, version):
    """Regex the served page must match: literal text of the source, tokens substituted."""
    parts = []
    pos = 0
    for off, tok in tokens_in(source):
        parts.append(re.escape(source[pos:off]))
        if tok == 'inc:header.inc':
            parts.append(re.escape(header))
        elif tok == 'gwfooter':
            parts.append(FOOTER_RE.pattern.replace(r'(\d+\.\d+\.\d+)', re.escape(version)).encode())
        else:
            parts.append(rb'.*?')
        pos = off + len(tok) + 2
    parts.append(re.escape(source[pos:]))
    return re.compile(b''.join(parts), re.S)


def check_json(r, name, raw, fields):
    try:
        obj = json.loads(raw.decode('utf-8'))
    except (ValueError, UnicodeDecodeError) as e:
        r.check(False, f'{name} returns valid JSON', f'{e}: {raw[:80]!r}')
        return None
    r.check(isinstance(obj, dict), f'{name} returns a JSON object')
    if not isinstance(obj, dict):
        return None
    for key, typ in fields.items():
        val = obj.get(key)
        ok = isinstance(val, typ) and not (typ is int and isinstance(val, bool))
        r.check(ok, f'{name} field "{key}" is {typ.__name__}', f'got {val!r}')
    return obj


def serial_mark(path):
    return os.path.getsize(path) if path and os.path.exists(path) else None


def serial_check(r, path, mark):
    if mark is None:
        return
    with open(path, 'rb') as f:
        f.seek(mark)
        new = f.read().decode('latin-1')
    r.section('Console capture during the test')
    r.check('*** ' not in new, 'no exception, assert or watchdog line on the console',
            next((l.strip() for l in new.splitlines() if '*** ' in l), ''))
    r.check('Reboot reason' not in new, 'the gateway did not reboot during the test')
    r.check('Malloc' not in new and 'malloc failed' not in new.lower(), 'no allocation failure on the console')
    mon = [l.strip() for l in new.splitlines() if 'MON:' in l]
    if mon:
        r.info('first ' + mon[0][-110:])
        r.info('last  ' + mon[-1][-110:])
    else:
        r.info('no MON line in the capture during the test (is the logger running?)')


def live_readonly(r, gw, version, any_version, rounds):
    files = static_files()
    header = files['header.inc']

    r.section('Live: status.cgi and version')
    st, hd, raw = gw.get('/status.cgi')
    r.check(st == 200, 'status.cgi answers 200', str(st))
    status = check_json(r, 'status.cgi', raw, STATUS_FIELDS) or {}
    m = re.match(r'v(\d+\.\d+\.\d+)-[0-9a-f]{8} \(', status.get('fwversion', ''))
    r.check(bool(m), 'status.cgi fwversion looks like vX.Y.Z-<commit> (<date>)', status.get('fwversion', ''))
    running = m.group(1) if m else None
    r.info(f'gateway reports {status.get("fwversion")}, uptime {status.get("uptime")} s')
    if not any_version:
        r.check(running == version, f'running firmware is {version}', f'gateway runs {running}')
    footer_version = running or version
    r.check(status.get('gateway') is True, 'status.cgi "gateway" is true')
    r.check(status.get('interface') in ('WIFI', 'Ethernet', 'WIFI & Ethernet', 'No'),
            'status.cgi "interface" has a known value', str(status.get('interface')))

    r.section('Live: pages')
    footers = {}
    for page in PAGES:
        st, hd, body = gw.get('/' + page)
        r.check(st == 200, f'{page} answers 200', str(st))
        r.check(hd.get('content-type', '').startswith('text/html'), f'{page} is served as text/html',
                hd.get('content-type', '(none)'))
        found = FOOTER_RE.findall(body.decode('utf-8', 'replace'))
        r.check(len(found) == 1, f'{page} shows the footer once', f'found {len(found)}')
        if found:
            footers[page] = FOOTER_RE.search(body.decode('utf-8', 'replace')).group(0)
            r.check(found[0] == footer_version, f'{page} footer shows version {footer_version}', found[0])
        r.check(b'!DEF' not in body, f'{page} has no undefined dynamic variable')
        r.check(not [t for _, t in tokens_in(body) if t in load_token_table()],
                f'{page} has no unsubstituted token')
        r.check(page_pattern(files[page], header, footer_version).fullmatch(body) is not None,
                f'{page} as served equals header + source + footer')
        tail = body[body.rfind(b'<center style="margin-top'):]
        r.check(not re.search(rb'<hr|border', tail, re.I), f'{page} has no separator line at the footer')
    r.check(len(set(footers.values())) == 1 and len(footers) == len(PAGES), 'the footer is identical on all pages')
    st, hd, body = gw.get('/')
    st2, hd2, body2 = gw.get('/index.html')
    r.check(st == 200 and body == body2, 'the root URL serves index.html')

    r.section('Live: CGI endpoints')
    st, hd, raw = gw.get('/settings.cgi')
    r.check(st == 200, 'settings.cgi answers 200', str(st))
    settings = check_json(r, 'settings.cgi', raw, SETTINGS_FIELDS) or {}
    if settings.get('locked'):
        r.check(len(settings.get('gwkey', '')) <= 25, 'settings.cgi shows only the tail of the key when locked')
    st, hd, raw = gw.get('/udp.cgi')
    r.check(st == 200, 'udp.cgi answers 200', str(st))
    udp = check_json(r, 'udp.cgi', raw, UDP_FIELDS) or {}
    r.check(re.fullmatch(r'[0-9A-Fa-f]{16}', udp.get('gateway_eui', '')) is not None,
            'udp.cgi gateway_eui is 16 hex digits', udp.get('gateway_eui', ''))
    for k in ('portup', 'portdn'):
        r.check(isinstance(udp.get(k), int) and 1 <= udp.get(k) <= 65535, f'udp.cgi {k} is a valid port')
    r.check(status.get('freqplan') in ('none', 'downloaded', 'builtin'),
            'status.cgi freqplan is none, downloaded or builtin', str(status.get('freqplan')))
    r.check(status.get('configured') is (status.get('freqplan') == 'downloaded'),
            'status.cgi configured is true exactly when freqplan is downloaded')
    r.check(status.get('connudp') is False or udp.get('enabled') is True,
            'status.cgi connudp is only true when the forwarder is enabled')
    st, hd, raw = gw.get('/ssids.cgi')
    r.check(st == 200, 'ssids.cgi answers 200', str(st))
    ssids = check_json(r, 'ssids.cgi', raw, {'APS': list, 'scanning': bool}) or {}
    for ap in ssids.get('APS', [])[:50]:
        if not (isinstance(ap, dict) and isinstance(ap.get('name'), str)):
            r.check(False, 'every ssids.cgi entry has a name', repr(ap)[:80])
            break
    st, hd, raw = gw.get('/id.cgi')
    r.check(st == 200 and raw == files['id.cgi'], 'id.cgi returns the gateway marker', repr(raw[:40]))

    r.section('Live: static files equal the sources')
    for name in sorted(files):
        if name in PAGES or name.endswith(('.cgi', '.inc')):
            continue
        st, hd, body = gw.get('/' + name)
        r.check(st == 200 and body == files[name], f'{name} is served byte for byte',
                f'status {st}, {len(body)} bytes, source {len(files[name])} bytes')

    r.section('Live: error handling')
    st, hd, body = gw.get('/does-not-exist.html')
    r.check(st == 404, 'an unknown file answers 404', str(st))
    st, hd, body = gw.get('/index.html?x=' + 'a' * 40)
    r.check(st == 200, 'a page with a query string still answers 200', str(st))

    r.section(f'Live: repeated requests ({rounds} rounds) and parallel requests')
    targets = ['/' + p for p in PAGES] + ['/status.cgi', '/settings.cgi', '/udp.cgi', '/mchp.js']
    bad = []
    t0 = time.time()
    for i in range(rounds):
        for t in targets:
            try:
                st, hd, body = gw.get(t)
                if st != 200 or not body or (t.endswith('.html') and not body.rstrip().endswith(b'</html>')):
                    bad.append(f'{t} round {i + 1}: status {st}, {len(body)} bytes')
            except Exception as e:
                bad.append(f'{t} round {i + 1}: {e}')
    r.check(not bad, f'{rounds * len(targets)} sequential requests all complete', '; '.join(bad[:3]))
    r.info(f'{rounds * len(targets)} requests in {time.time() - t0:.1f} s')

    bad = []

    def worker(t):
        try:
            st, hd, body = gw.get(t)
            if st != 200 or not body:
                bad.append(f'{t}: status {st}')
        except Exception as e:
            bad.append(f'{t}: {e}')

    for i in range(rounds):
        threads = [threading.Thread(target=worker, args=(t,)) for t in targets[:4]]
        for th in threads:
            th.start()
        for th in threads:
            th.join()
    r.check(not bad, f'{rounds} bursts of 4 parallel requests all complete', '; '.join(bad[:3]))

    st, hd, raw = gw.get('/status.cgi')
    after = check_json(Report(quiet=True), 'status.cgi', raw, {}) or {}
    r.check(st == 200 and isinstance(after.get('uptime'), int) and after['uptime'] >= status.get('uptime', 0),
            'uptime did not restart during the test (no reboot)',
            f'before {status.get("uptime")}, after {after.get("uptime")}')
    return udp


def udp_fields(conf):
    return {'udpen': '1' if conf['enabled'] else '0', 'server': conf['server'],
            'portup': str(conf['portup']), 'portdn': str(conf['portdn']),
            'uponly': '1' if conf['uponly'] else '0'}


def udp_get(gw):
    st, hd, raw = gw.get('/udp.cgi')
    conf = json.loads(raw.decode('utf-8'))
    return {k: conf[k] for k in ('enabled', 'server', 'portup', 'portdn', 'uponly')}


def live_write(r, gw):
    r.section('Live, writing: UDP forwarder settings through udp.cgi')
    original = udp_get(gw)
    r.info(f'original settings: {original}')
    try:
        changed = dict(original)
        changed['portup'] = 1701 if original['portup'] != 1701 else 1702
        changed['portdn'] = 1703 if original['portdn'] != 1703 else 1704
        changed['uponly'] = not original['uponly']
        if not changed['server']:
            changed['server'] = 'test.invalid'
        st, hd, body = gw.post('/udp.cgi', udp_fields(changed))
        r.check(st == 302 and hd.get('location', '').endswith('/udp.html'),
                'a valid POST redirects to udp.html', f'status {st}, location {hd.get("location")}')
        time.sleep(1)
        r.check(udp_get(gw) == changed, 'the new settings are read back', str(udp_get(gw)))

        for label, fields in (
            ('port 0', dict(udp_fields(changed), portup='0')),
            ('port 70000', dict(udp_fields(changed), portdn='70000')),
            ('a port that is not a number', dict(udp_fields(changed), portup='abc')),
            ('enabled without a server', dict(udp_fields(changed), udpen='1', server='')),
            ('a server name of 95 characters', dict(udp_fields(changed), server='a' * 95)),
        ):
            st, hd, body = gw.post('/udp.cgi', fields)
            r.check(st == 302 and hd.get('location', '').endswith('/error'),
                    f'a POST with {label} redirects to the error page',
                    f'status {st}, location {hd.get("location")}')
            time.sleep(0.5)
            r.check(udp_get(gw) == changed, f'the settings are unchanged after {label}', str(udp_get(gw)))
        st, hd, body = gw.get('/error')
        r.check(st in (200, 404), 'the error redirect target answers', str(st))
    finally:
        st, hd, body = gw.post('/udp.cgi', udp_fields(original))
        time.sleep(1)
        restored = udp_get(gw)
        r.check(restored == original, 'the original settings are restored', f'now {restored}')
        if restored != original:
            print(f'\n  !!! UDP settings NOT restored. Original: {original}\n')


# ---------------------------------------------------------------------------------------
# Self-test: a mock gateway built from the sources, plus deliberately broken variants
# ---------------------------------------------------------------------------------------

def make_mock(version, fault=None):
    files = static_files()
    state = {'udp': {'enabled': True, 'server': 'example.org', 'portup': 1700, 'portdn': 1700, 'uponly': False},
             'boot': time.time()}
    footer = ('<center style="margin-top:2em;font-size:.8em;color:#999">Firmware %s &middot; '
              '<a href="%s" target="_blank" rel="noopener">github.com/pe1mew/gateway</a></center>')

    def dyn(tok, path):
        if tok == 'inc:header.inc':
            return files['header.inc']
        if tok == 'gwfooter':
            if fault == 'no-footer' and path == 'info.html':
                return b''
            v = '0.0.1' if fault == 'wrong-version' else version
            f = footer % (v, REPO_URL)
            if fault == 'separator':
                f = '<hr>' + f
            return f.encode()
        if tok == 'gwstatus':
            return json.dumps({
                'gateway': True, 'hwversion': 'v1', 'blversion': 'r1-00000000 (x)',
                'fwversion': 'v%s-0123abcd (2026-01-01T00:00:00Z)' % version,
                'uptime': int(time.time() - state['boot']) + 100, 'connected': True, 'interface': 'Ethernet',
                'ssid': '', 'activation_locked': True, 'configured': True, 'freqplan': 'downloaded', 'region': 'EU868',
                'gwcard': '868Mhz', 'connbroker': True, 'connudp': False, 'pup': 1, 'pdown': 0, 'udpup': 0, 'udpdown': 0,
                'estor': False}).encode()
        if tok == 'gwsettings':
            return json.dumps({'ssid': '', 'gwid': 'gw', 'gwkey': '[...]tail', 'asrv': 'https://a',
                               'locked': True, 'fotaen': False, 'fotaul': ''}).encode()
        if tok == 'gwudp':
            return json.dumps(dict(state['udp'], gateway_eui='0011223344556677')).encode()
        if tok == 'scanresult':
            return json.dumps({'APS': [{'name': 'net', 'rssi': -60, 'sec': 'wpa2', 'type': 'infra'}],
                               'scanning': False}).encode()
        return b''

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *a):
            pass

        def send(self, status, body=b'', ctype=None, location=None):
            self.send_response(status)
            self.send_header('Connection', 'close')
            if ctype:
                self.send_header('Content-Type', ctype)
            if location:
                self.send_header('Location', location)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0].lstrip('/') or 'index.html'
            if path not in files or path.endswith('.inc'):
                return self.send(404, b'404: File not found\r\n')
            data = files[path]
            if fault == 'stale-asset' and path == 'mchp.js':
                data = data + b'// old'
            out, pos = b'', 0
            for off, tok in tokens_in(data):
                out += data[pos:off] + dyn(tok, path)
                pos = off + len(tok) + 2
            out += data[pos:]
            ctype = 'text/html' if path.endswith(('.html', '.cgi')) else 'application/octet-stream'
            self.send(200, out, ctype)

        def do_POST(self):
            length = int(self.headers.get('Content-Length', 0))
            f = dict(urllib.parse.parse_qsl(self.rfile.read(length).decode(), keep_blank_values=True))
            if self.path != '/udp.cgi':
                return self.send(302, location='/error')
            new = dict(state['udp'])
            try:
                for k in ('portup', 'portdn'):
                    if k in f:
                        if not 1 <= int(f[k]) <= 65535:
                            raise ValueError
                        new[k] = int(f[k])
                if 'server' in f:
                    if len(f['server']) >= 93:
                        raise ValueError
                    new['server'] = f['server']
                if 'uponly' in f:
                    new['uponly'] = f['uponly'] == '1'
                if 'udpen' in f:
                    new['enabled'] = f['udpen'] == '1'
                if f.get('udpen') == '1' and not f.get('server'):
                    raise ValueError
            except ValueError:
                if fault != 'accepts-bad-input':
                    return self.send(302, location='/error')
            state['udp'] = new
            self.send(302, location='/udp.html')

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def selftest():
    version = expected_version()
    print('Self-test: the live checks against a mock gateway built from the sources')
    results = []
    for fault in (None, 'no-footer', 'wrong-version', 'separator', 'stale-asset', 'accepts-bad-input'):
        server = make_mock(version, fault)
        r = Report(quiet=True)
        gw = Gateway('127.0.0.1:%d' % server.server_address[1])
        try:
            live_readonly(r, gw, version, False, 2)
            live_write(r, gw)
        finally:
            server.shutdown()
        if fault is None:
            ok = r.failed == 0
            print(f'  {"PASS" if ok else "FAIL"}  correct mock: {r.passed} checks pass, {r.failed} fail')
            for f in r.failures:
                print('          ' + f)
        else:
            ok = r.failed > 0
            print(f'  {"PASS" if ok else "FAIL"}  fault "{fault}" is detected ({r.failed} checks fail'
                  + (f', first: {r.failures[0]}' if r.failures else '') + ')')
        results.append(ok)
    return all(results)


def main():
    ap = argparse.ArgumentParser(description='Acceptance test for the gateway web UI')
    ap.add_argument('--gateway', help='address of a running gateway (ip or ip:port)')
    ap.add_argument('--write', action='store_true', help='also run the UDP settings round trip')
    ap.add_argument('--expect-version')
    ap.add_argument('--any-version', action='store_true')
    ap.add_argument('--rounds', type=int, default=5)
    ap.add_argument('--serial-log')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)
    if args.write and not args.gateway:
        ap.error('--write needs --gateway')

    r = Report()
    source_checks(r)
    if args.gateway:
        version = args.expect_version or expected_version()
        gw = Gateway(args.gateway)
        mark = serial_mark(args.serial_log)
        if args.serial_log and mark is None:
            r.check(False, f'console capture {args.serial_log} exists')
        try:
            live_readonly(r, gw, version, args.any_version, args.rounds)
            if args.write:
                live_write(r, gw)
        except (OSError, http.client.HTTPException) as e:
            r.check(False, 'the gateway answers', f'{type(e).__name__}: {e}')
        if mark is not None:
            time.sleep(12)  # one MON interval, so the capture holds a line from after the test
            serial_check(r, args.serial_log, mark)

    print(f'\n{r.passed} passed, {r.failed} failed')
    if r.failures:
        print('Failed checks:')
        for f in r.failures:
            print('  - ' + f)
    if args.gateway:
        print('\nMANUAL (look at it in a browser, not covered by this test):')
        print('  - Settings, Info and UDP pages: the footer reads as part of the page, no line above it')
        print('  - Info page: all fields are filled in; UDP page: saving shows the stored values')
        print('  - Settings page: saving the WiFi/activation form (restarts networking)')
        if not args.write:
            print('  - UDP settings round trip was skipped (run with --write)')
    sys.exit(1 if r.failed else 0)


if __name__ == '__main__':
    main()
