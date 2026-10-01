#!/usr/bin/env python3
"""alertterm - reference client for the ALERT receiver's serial interface (V2).

The interface is specified in docs/ALERT_SERIAL.md. This file is the worked
example of that document and the tool the bench uses; test_console.py checks
its parsers against the document's own example lines.

    python tools/alert/alertterm.py live                    # decode table, raw lines to logs/
    python tools/alert/alertterm.py info
    python tools/alert/alertterm.py get [NAME]
    python tools/alert/alertterm.py set CSV_OUT ON
    python tools/alert/alertterm.py time-sync
    python tools/alert/alertterm.py log-download --csv out.csv [--last N]
    python tools/alert/alertterm.py log-clear --yes
    python tools/alert/alertterm.py stations-upload [--filter F | --all | --blob B]
    python tools/alert/alertterm.py stations-clear --yes
    python tools/alert/alertterm.py screenshot out.png [--scale 4]
    python tools/alert/alertterm.py console                 # interactive

The radio is found by USB VID 36B7 and DTR is asserted: the firmware sends
nothing without it. Every subcommand first sets the radio's clock (TIME
<epoch>) unless --no-time is given, because that clock lives in RAM and every
reboot loses it. Windows lets one process hold a COM port, so stop
`radio.py log` or any terminal on the port first.

Standard library only, plus pyserial, which is imported only when a port is
actually opened (the parsers and the PNG writer work without it).
"""
import argparse
import collections
import csv
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zlib
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
LOG_DIR = os.path.join(HERE, 'logs')                 # its .gitignore keeps *.log out of git
DEFAULT_FILTER = os.path.join(REPO, 'filters', 'stations.filter')

VID = 0x36B7            # App/usb/usbd_cdc_if.c; the stock bootloader enumerates with it too
SCHEMA = 2              # the HDR schema this client was written for
MAX_CMD = 96            # console line limit, V2_SPEC section 7
STN_REGION = 0x10000    # the 64 KB station region, V2_SPEC section 8
STN_BLOB_MAX = STN_REGION - 32   # its last 32 bytes are the radio's ownership mark
STN_MAX_CHUNK = 64      # STN W carries at most 64 bytes
SILENCE_S = 25.0        # STA arrives every 10 s: this long without a byte, re-assert DTR

# Record fields, schema 2 (V2_SPEC section 6). An HDR block from the radio
# replaces these, so a build that appends fields is still read by name.
FIELDS = {
    'DEC': ('seq', 'epoch', 'uptime_ms', 'boot', 'id', 'name', 'kind', 'value', 'eng',
            'unit', 'fmt', 'pol', 'inv', 'frame', 'rssi', 'nf', 'sens', 'fade',
            'burst_ms', 'payload_hex', 'payload_bin'),
    'BST': ('seq', 'epoch', 'uptime_ms', 'peak', 'nf', 'burst_ms', 'nframes', 'nbits',
            'bits_hex'),
    'STA': ('epoch', 'uptime_ms', 'nf', 'rssi', 'sq', 'batt_mv', 'batt_pct', 'bursts',
            'decodes', 'min_ok', 'log_state', 'log_count', 'log_cap', 'stn_src'),
    'EVT': ('epoch', 'uptime_ms', 'code', 'detail'),
}

# Console reply lines (section 7). LOG and STN have two shapes each, told apart
# by field count / a numeric first field; LOG DUMP lines carry the DEC fields.
LOG_STAT = ('count', 'capacity', 'oldest_seq', 'newest_seq', 'state')
STN_GET = ('id', 'name', 'kind')
STN_INFO = ('source', 'count', 'crc', 'state')
CONSOLE_FIELDS = {
    'GET': ('name', 'value'),
    'INFO': ('key', 'value'),
    'TIME': ('epoch',),
    'SCR': ('row', 'hex'),
    'SPI': ('addr', 'hex'),
    'HELP': ('text',),
}
REST_FIELDS = ('detail', 'value', 'text')   # a last field by these names takes any commas left

# The stream the radio sends on its own; everything else with a comma is a reply.
RECORD_TYPES = frozenset(('HDR', 'DEC', 'BST', 'STA', 'EVT', 'ALERT'))

# stock 0xABCD framing (tools/serialtool/msg.py, App/app/uart.c)
OBFUS = bytes.fromhex('166C14E62E910D402135D5401303E980')
BOOTLOADER_IDS = (0x0518, 0x0530)   # beacons the stock bootloader sends while in DFU


class ConsoleError(Exception):
    """The radio answered ERR,<reason>."""
    def __init__(self, cmd, reason):
        Exception.__init__(self, 'radio refused %r: ERR%s' % (cmd, ',' + reason if reason else ''))
        self.cmd, self.reason = cmd, reason


class ConsoleTimeout(Exception):
    """No terminal OK/ERR line in time."""


class LinkError(Exception):
    """No radio, or its port could not be opened."""


# ---------------------------------------------------------------------------
# Lines

def classify(line):
    """What a received line is:
    'final'  OK / OK,<detail> / ERR / ERR,<reason>: the end of a command
    'record' the stream the radio sends on its own (HDR DEC BST STA EVT, old ALERT)
    'debug'  space-separated diagnostics without a comma: D, A, B, K, X ...
    'data'   a console reply line: LOG, STN, SCR, GET, INFO, TIME, SPI, HELP ..."""
    typ = line.split(',', 1)[0]
    if typ in ('OK', 'ERR'):
        return 'final'
    if ',' in line:
        return 'record' if typ in RECORD_TYPES else 'data'
    return 'debug' if ' ' in line else 'data'


def parse_final(line):
    """'OK,x' -> (True, 'x'); 'ERR,RANGE' -> (False, 'RANGE')."""
    typ, _, detail = line.partition(',')
    return typ == 'OK', detail


class Record(dict):
    """One parsed line: field name -> text exactly as sent ('' = unknown).
    .type is the record type, .extra any fields beyond the known names."""
    __slots__ = ('type', 'extra')

    def __init__(self, typ):
        dict.__init__(self)
        self.type = typ
        self.extra = []

    def num(self, name):
        """The field as an int, or None when it is empty, missing or not a number."""
        s = self.get(name)
        if not s:
            return None
        try:
            return int(s, 10)
        except ValueError:
            return None


def fields_for(typ, vals, fields=None):
    table = fields or FIELDS
    if typ == 'LOG':
        return LOG_STAT if len(vals) == len(LOG_STAT) else table.get('DEC', FIELDS['DEC'])
    if typ == 'STN':
        return STN_GET if vals and vals[0].isdigit() else STN_INFO
    return table.get(typ) or CONSOLE_FIELDS.get(typ, ())


def parse_record(line, fields=None):
    """Split a comma line into a Record, naming fields from `fields` (a
    Schema's field lists) or the schema-2 defaults."""
    parts = line.split(',')
    typ, vals = parts[0], parts[1:]
    names = fields_for(typ, vals, fields)
    rec = Record(typ)
    for i, name in enumerate(names):
        if i >= len(vals):
            break
        if i == len(names) - 1 and name in REST_FIELDS:
            rec[name] = ','.join(vals[i:])
            return rec
        rec[name] = vals[i]
    rec.extra = vals[len(names):]
    return rec


class Schema(object):
    """What the HDR block said: firmware hash, schema number, field lists.
    Starts from the schema-2 defaults so lines are readable before any HDR."""

    def __init__(self):
        self.fw = None
        self.schema = None
        self.fields = dict(FIELDS)

    def feed(self, line):
        """Take an HDR line; False for anything else."""
        parts = line.split(',')
        if len(parts) < 2 or parts[0] != 'HDR':
            return False
        if parts[1] == 'fw':
            kv = parts[1:]
            for k, v in zip(kv[0::2], kv[1::2]):
                if k == 'fw':
                    self.fw = v
                elif k == 'schema':
                    self.schema = int(v) if v.isdigit() else v
        else:
            self.fields[parts[1]] = tuple(parts[2:])
        return True


def format_dec(rec, when=None):
    """One DEC (or LOG DUMP) record as a table row for people."""
    eng, unit = rec.get('eng', ''), rec.get('unit', '')
    reading = '%s %s' % (eng, unit) if unit else eng
    fade = rec.num('fade')
    flags = '%s %s%s' % (rec.get('fmt', ''), rec.get('pol', ''),
                         ' INV' if rec.get('inv') == '1' else '')
    return '%-8s %5s  %-22.22s %-4s %10s %5s %5s  %s' % (
        when or '', rec.get('id', ''), rec.get('name') or '?', rec.get('kind') or '-',
        reading, rec.get('rssi', ''), '' if fade is None else '%+d' % fade, flags)


DEC_HEADER = '%-8s %5s  %-22s %-4s %10s %5s %5s  %s' % (
    'TIME', 'ID', 'STATION', 'KIND', 'VALUE', 'RSSI', 'FADE', 'FRAME')


def format_sta(rec):
    """One STA record as a status line."""
    mv = rec.num('batt_mv')
    return ('-- NF %s  RSSI %s  SQ %s  BATT %s%%%s  BURSTS %s  DEC %s  MIN_OK %s  LOG %s %s/%s  STN %s'
            % (rec.get('nf', ''), rec.get('rssi', ''), rec.get('sq', ''), rec.get('batt_pct', ''),
               ' %.2fV' % (mv / 1000.0) if mv else '', rec.get('bursts', ''),
               rec.get('decodes', ''), rec.get('min_ok') or '-', rec.get('log_state', ''),
               rec.get('log_count', ''), rec.get('log_cap', ''), rec.get('stn_src', '')))


# ---------------------------------------------------------------------------
# Screen (SCREEN -> SCR lines -> PNG)

def parse_scr(line):
    """'SCR,<row>,<256 hex>' -> (row, 128 bytes)."""
    parts = line.split(',')
    if len(parts) != 3 or parts[0] != 'SCR':
        raise ValueError('not an SCR line: %r' % line[:40])
    row, data = int(parts[1]), bytes.fromhex(parts[2])
    if not 0 <= row <= 7 or len(data) != 128:
        raise ValueError('SCR row %d carries %d bytes, want row 0-7 and 128' % (row, len(data)))
    return row, data


def screen_pixels(rows):
    """rows {0..7: 128 bytes} -> 64 lists of 128 ints, 1 = pixel on.
    SCR row 0 is the status line, rows 1-7 the frame buffer rows 0-6. The
    bytes are the ST7565's pages: byte x of a row is column x, and bit b of it
    is y = 8 * row + b (bit 0 at the top)."""
    return [[(rows[y >> 3][x] >> (y & 7)) & 1 for x in range(128)] for y in range(64)]


def png_1bit(pixels, scale=1, invert=False):
    """A 1-bit grayscale PNG. A lit pixel is black, like the LCD, unless invert.
    Written by hand (zlib + CRC) so no imaging library is needed."""
    height, width = len(pixels) * scale, len(pixels[0]) * scale
    raw = bytearray()
    for row in pixels:
        line = bytearray((width + 7) // 8)
        for x in range(width):
            white = (not row[x // scale]) != bool(invert)
            if white:
                line[x >> 3] |= 0x80 >> (x & 7)
        for _ in range(scale):
            raw += b'\x00' + line           # filter type 0 (None) on every scanline

    def chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data +
                struct.pack('>I', zlib.crc32(kind + data) & 0xFFFFFFFF))

    ihdr = struct.pack('>IIBBBBB', width, height, 1, 0, 0, 0, 0)   # 1 bit, grayscale
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
            chunk(b'IDAT', zlib.compress(bytes(raw), 9)) + chunk(b'IEND', b''))


# ---------------------------------------------------------------------------
# Station blob (V2_SPEC section 8) and its upload

ASTB = struct.Struct('<4sHHII16s')     # magic, version, count, names_len, crc32, source: 32 B


def parse_blob_header(data):
    """The 32-byte ASTB header -> dict; ValueError if it is not one."""
    if len(data) < ASTB.size:
        raise ValueError('%d bytes is shorter than the 32-byte ASTB header' % len(data))
    magic, ver, count, names_len, crc, src = ASTB.unpack_from(data)
    if magic != b'ASTB':
        raise ValueError('not a station blob: magic %r, want ASTB' % magic)
    return {'version': ver, 'count': count, 'names_len': names_len, 'crc': crc,
            'source': src.split(b'\0', 1)[0].decode('ascii', 'replace')}


def check_blob(blob):
    """Header plus the checks the radio will make -> (header, [problems])."""
    hdr = parse_blob_header(blob)
    problems = []
    if len(blob) > STN_BLOB_MAX:
        problems.append('%d bytes is over the %d-byte station blob limit' % (len(blob), STN_BLOB_MAX))
    need = ASTB.size + 8 * hdr['count'] + hdr['names_len']
    if len(blob) < need:
        problems.append('%d bytes, but the header describes %d' % (len(blob), need))
    if hdr['version'] != 1:
        problems.append('version %d, this client knows 1' % hdr['version'])
    if zlib.crc32(blob[ASTB.size:]) & 0xFFFFFFFF != hdr['crc']:
        problems.append('header crc32 %08X does not match the body' % hdr['crc'])
    return hdr, problems


def stn_commands(blob, chunk=STN_MAX_CHUNK):
    """The console lines that upload `blob`: BEGIN, one W per chunk, END.
    BEGIN carries the CRC-32 (zlib) of all len bytes; offsets are decimal."""
    if not 1 <= chunk <= STN_MAX_CHUNK:
        raise ValueError('chunk must be 1-%d bytes' % STN_MAX_CHUNK)
    yield 'STN BEGIN %d %08X' % (len(blob), zlib.crc32(blob) & 0xFFFFFFFF)
    for off in range(0, len(blob), chunk):
        yield 'STN W %d %s' % (off, blob[off:off + chunk].hex().upper())
    yield 'STN END'


def upload_stations(link, blob, chunk=STN_MAX_CHUNK, progress=None, note=print):
    """Upload with flow control (each line waits for its OK). Returns the
    chunk size that worked.

    A 64-byte chunk is a 140-character line, over the console's 96-character
    limit. A build that holds to that limit refuses the line, or cuts it and
    then fails a later chunk or the CRC at END. So any refusal after BEGIN
    while the chunks are over 32 bytes restarts the whole upload once with
    32-byte chunks, which fit (STN BEGIN erases again). A real fault, such as
    a flash error, fails the second attempt too and is raised from there."""
    sizes = [chunk] + ([32] if chunk > 32 else [])
    for attempt, size in enumerate(sizes):
        retry_left = attempt < len(sizes) - 1
        cmds = list(stn_commands(blob, size))
        for i, cmd in enumerate(cmds):
            slow = cmd.startswith('STN BEGIN') or cmd == 'STN END'   # erase / verify
            try:
                link.command(cmd, timeout=30.0 if slow else 5.0)
            except ConsoleError as e:
                if retry_left and i > 0:
                    note('%d-byte chunks refused (ERR,%s at %s): retrying with 32'
                         % (size, e.reason, cmd[:16]))
                    break
                raise
            if progress:
                progress(i + 1, len(cmds))
        else:
            return size
    raise AssertionError('unreachable')


def build_blob(args):
    """Run gen_stations.py --blob into a temporary directory and return the bytes."""
    gen = os.path.join(HERE, 'gen_stations.py')
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'stations.bin')
        if args.all:
            filt = os.path.join(td, 'all.filter')
            with open(filt, 'w') as fh:
                fh.write('all\n')          # the filter directive for every MegaNet address
        else:
            filt = args.filter or DEFAULT_FILTER
        # --out points into the temp dir so a blob run can never touch the
        # generated header in the tree; --max-bytes is the blob limit, not
        # the 8 KB built-in table limit
        cmd = [sys.executable, gen, '--blob', out, '--filter', filt,
               '--out', os.path.join(td, 'unused.h'), '--max-bytes', str(STN_BLOB_MAX)]
        if args.meganet_dir:
            cmd += ['--meganet-dir', args.meganet_dir]
        print('building the station blob: %s' % ' '.join(cmd[1:]), flush=True)
        rc = subprocess.call(cmd)
        if rc != 0 or not os.path.exists(out):
            raise SystemExit('gen_stations.py failed (exit %d)' % rc)
        with open(out, 'rb') as fh:
            return fh.read()


# ---------------------------------------------------------------------------
# Binary 0xABCD frames (only what the document describes; hotflash.py and
# tools/serialtool are the real users)

def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def make_frame(msg_id, body=b''):
    """AB CD | len | obfuscated(ID, data_len, data, CRC-16/XMODEM) | DC BA,
    byte for byte what tools/serialtool/msg.py make_packet builds."""
    m = struct.pack('<HH', msg_id, len(body)) + bytes(body)
    if len(m) & 1:
        m += b'\0'
    payload = bytearray(m + struct.pack('<H', crc16_xmodem(m)))
    for i in range(len(payload)):
        payload[i] ^= OBFUS[i % 16]
    return b'\xab\xcd' + struct.pack('<H', len(m)) + bytes(payload) + b'\xdc\xba'


class LineReader(object):
    """Bytes in, text lines out. Binary 0xABCD frames that share the stream
    are cut out whole (their IDs kept in .frame_ids); the firmware sends each
    line and each frame as one write, so a frame never lands inside a line."""
    MAX_LINE = 1024

    def __init__(self):
        self.buf = bytearray()
        self.frame_ids = []

    @property
    def bootloader(self):
        """True once a stock-bootloader beacon has been seen: the radio is in DFU."""
        return any(i in BOOTLOADER_IDS for i in self.frame_ids)

    def feed(self, data):
        self.buf += data
        out = []
        while self.buf:
            i = self.buf.find(0xAB)
            if i < 0:
                cut = max(self.buf.rfind(b'\n'), self.buf.rfind(b'\r')) + 1
                self._text(self.buf[:cut], out)
                del self.buf[:cut]
                if len(self.buf) > self.MAX_LINE:
                    del self.buf[:]             # no line end in 1 KB: noise, not a line
                break
            if i:
                self._text(self.buf[:i] + b'\n', out)
                del self.buf[:i]
                continue
            if len(self.buf) < 4:
                break
            if self.buf[1] != 0xCD:
                del self.buf[:1]                # a stray 0xAB byte
                continue
            size = self.buf[2] | (self.buf[3] << 8)
            if size > 512:
                del self.buf[:2]
                continue
            if len(self.buf) < size + 8:
                break
            if self.buf[size + 6] != 0xDC or self.buf[size + 7] != 0xBA:
                del self.buf[:2]
                continue
            if size >= 2:
                self.frame_ids.append((self.buf[4] ^ OBFUS[0]) | ((self.buf[5] ^ OBFUS[1]) << 8))
            del self.buf[:size + 8]
        return out

    # Control bytes and non-ASCII are never part of a line: dropping them keeps
    # connect-time garbage (a NUL, half a frame) from gluing onto the next line.
    _JUNK = bytes(range(0x00, 0x20)) + bytes(range(0x7F, 0x100))

    @classmethod
    def _text(cls, data, out):
        for raw in bytes(data).replace(b'\r', b'\n').split(b'\n'):
            txt = raw.translate(None, cls._JUNK).decode('ascii').strip()
            if txt:
                out.append(txt)


# ---------------------------------------------------------------------------
# The link: one open port, the console protocol on top

Reply = collections.namedtuple('Reply', 'lines detail')


class Link(object):
    """A radio connection. `ser` is a pyserial port (or anything with read,
    write, in_waiting). on_async(line) gets every stream line (records and
    debug lines), including those that arrive in the middle of a command."""

    def __init__(self, ser, on_async=None, raw=None, dev=None):
        self.ser = ser
        self.dev = dev
        self.on_async = on_async
        self.raw = raw                  # file: every received line, as received
        self.reader = LineReader()
        self.queue = collections.deque()
        self.schema = Schema()
        self.last_rx = time.monotonic()
        self.write_lock = threading.Lock()

    def close(self):
        try:
            self.ser.close()
        finally:
            if self.raw:
                self.raw.close()

    def reassert_dtr(self):
        # A USB send that times out in the firmware clears its DTR flag, and
        # it stays silent until a fresh SET_CONTROL_LINE_STATE: toggling
        # guarantees one (setting True while True may not reach the device).
        self.ser.dtr = False
        time.sleep(0.05)
        self.ser.dtr = True

    def _feed(self, data):
        if not data:
            return
        self.last_rx = time.monotonic()
        for line in self.reader.feed(data):
            if self.raw:
                self.raw.write(line + '\n')
                self.raw.flush()
            self.queue.append(line)

    def readline(self, timeout):
        """The next line, or None if none arrives within timeout seconds."""
        end = time.monotonic() + timeout
        while True:
            if self.queue:
                line = self.queue.popleft()
                self.schema.feed(line)
                return line
            n = self.ser.in_waiting
            if n:
                self._feed(self.ser.read(n))
                continue
            if time.monotonic() >= end:
                return None
            self._feed(self.ser.read(1))   # waits up to the port timeout (50 ms)

    def stream(self, line):
        if self.on_async:
            self.on_async(line)

    def send(self, text):
        if '\r' in text or '\n' in text:
            raise ValueError('one command per line')
        with self.write_lock:
            self.ser.write(text.encode('ascii') + b'\n')

    def drain(self):
        """Hand everything already received to the stream. A stray OK/ERR here
        belongs to no command still waiting (a late second terminal line) and
        is dropped, so it cannot end the next command early."""
        while True:
            line = self.readline(0)
            if line is None:
                return
            if classify(line) in ('record', 'debug'):
                self.stream(line)

    def command(self, text, timeout=3.0, on_data=None):
        """Send one console command and wait for its terminal line. Returns
        Reply(data lines, detail of the final OK); raises ConsoleError on ERR
        and ConsoleTimeout when nothing ends it. `timeout` is an idle limit:
        every reply line restarts it, so a long LOG DUMP never times out while
        lines keep coming; stream lines (STA every 10 s) do not. One command
        in flight at a time: the radio's receive ring is 256 bytes."""
        self.drain()
        self.send(text)
        lines = []
        end = time.monotonic() + timeout
        while True:
            line = self.readline(max(0.0, end - time.monotonic()))
            if line is None:
                hint = (' - the radio is in its bootloader (DFU)' if self.reader.bootloader
                        else ' - is ALERT v2 firmware running, and DTR asserted?')
                raise ConsoleTimeout('no OK/ERR for %r within %g s%s' % (text, timeout, hint))
            kind = classify(line)
            if kind == 'final':
                ok, detail = parse_final(line)
                if not ok:
                    raise ConsoleError(text, detail)
                return Reply(lines, detail)
            if kind in ('record', 'debug'):
                self.stream(line)
                continue
            end = time.monotonic() + timeout
            if on_data:
                on_data(line)
            else:
                lines.append(line)


def find_ports():
    from serial.tools import list_ports
    return sorted((p for p in list_ports.comports() if p.vid == VID), key=lambda p: p.device)


def open_radio(port=None):
    """-> (pyserial port, device name), DTR asserted, input flushed."""
    try:
        import serial
    except ImportError:
        raise LinkError('pyserial is needed to talk to the radio: pip install pyserial')
    dev = port
    if not dev:
        found = find_ports()
        if not found:
            raise LinkError('no radio: no serial port with USB VID %04X (plugged in and on? '
                            'or give --port)' % VID)
        dev = found[0].device
    try:
        # the radio's CDC ACM port ignores the baud rate; any value works
        ser = serial.Serial(dev, 115200, timeout=0.05, write_timeout=2.0)
    except (serial.SerialException, OSError) as e:
        raise LinkError('cannot open %s: %s\n(only one program may hold the port: '
                        'stop radio.py log, a terminal, CHIRP...)' % (dev, e))
    ser.rts = True
    ser.dtr = False             # a fresh SET_CONTROL_LINE_STATE, see Link.reassert_dtr
    time.sleep(0.05)
    ser.dtr = True
    time.sleep(0.1)             # anything sent before DTR is up is partial: drop it
    ser.reset_input_buffer()
    return ser, dev


def sync_clock(link):
    """TIME <now>, retried once after a DTR toggle (a firmware send that timed
    out earlier leaves it silent until DTR is asserted again)."""
    for attempt in range(2):
        try:
            link.command('TIME %d' % int(time.time()), timeout=3.0)
            return
        except ConsoleTimeout:
            if attempt:
                raise
            link.reassert_dtr()


def connect(args, on_async=None, raw=None, sync=True, required=True):
    ser, dev = open_radio(args.port)
    fh = None
    if raw:
        os.makedirs(os.path.dirname(os.path.abspath(raw)), exist_ok=True)
        fh = open(raw, 'a', encoding='ascii', errors='replace', newline='')
    link = Link(ser, on_async=on_async, raw=fh, dev=dev)
    if sync and not args.no_time:
        try:
            sync_clock(link)
        except (ConsoleTimeout, ConsoleError) as e:
            if required:
                link.close()
                raise
            print('# clock not set: %s' % e, file=sys.stderr)
    return link


def iso(epoch):
    return datetime.fromtimestamp(epoch, timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


# ---------------------------------------------------------------------------
# Subcommands

class LiveView(object):
    def __init__(self, args):
        self.args = args
        self.last_sta = 0.0
        self.said_fw = None
        self.schema = None

    def __call__(self, line):
        typ = line.split(',', 1)[0]
        fields = self.schema.fields if self.schema else None
        if typ == 'DEC':
            print(format_dec(parse_record(line, fields), datetime.now().strftime('%H:%M:%S')),
                  flush=True)
        elif typ == 'STA':
            now = time.monotonic()
            if self.args.sta == 0 or now - self.last_sta >= self.args.sta:
                self.last_sta = now
                print(format_sta(parse_record(line, fields)), flush=True)
        elif typ == 'EVT':
            r = parse_record(line, fields)
            print('** %s %s' % (r.get('code', ''), r.get('detail', '')), flush=True)
        elif typ == 'HDR' and self.schema and self.schema.fw and self.schema.fw != self.said_fw:
            self.said_fw = self.schema.fw
            warn = '' if self.schema.schema in (None, SCHEMA) else \
                '  (this client knows schema %d: fields are read by name)' % SCHEMA
            print('# firmware %s, schema %s%s' % (self.schema.fw, self.schema.schema, warn),
                  flush=True)
        elif typ == 'BST' and self.args.bursts:
            print('   ' + line, flush=True)
        elif classify(line) == 'debug' and self.args.debug_lines:
            print('   ' + line, flush=True)


def cmd_live(args):
    raw = args.raw or os.path.join(LOG_DIR, 'live-%s.log' % datetime.now().strftime('%Y%m%d-%H%M%S'))
    view = LiveView(args)
    print('raw lines -> %s (Ctrl-C to stop)' % raw, flush=True)
    print(DEC_HEADER, flush=True)
    said_waiting = False
    while True:
        try:
            link = connect(args, on_async=view, raw=raw, required=False)
        except KeyboardInterrupt:
            return 0
        except Exception as e:      # LinkError, or the port died while connecting
            if not said_waiting:
                print('# waiting for the radio: %s' % str(e).split('\n')[0], flush=True)
                said_waiting = True
            time.sleep(1.0)
            continue
        said_waiting = False
        view.schema = link.schema
        print('# connected on %s' % link.dev, flush=True)
        try:
            for probe in ('CSV HDR', 'GET CSV_OUT'):
                try:
                    rep = link.command(probe)
                except (ConsoleError, ConsoleTimeout):
                    continue
                if any(l.upper() == 'GET,CSV_OUT,OFF' for l in rep.lines):
                    print('# CSV OUT is OFF on the radio: nothing will arrive '
                          '(alertterm.py set CSV_OUT ON)', flush=True)
            while True:
                line = link.readline(1.0)
                if line is not None:
                    view(line)
                elif time.monotonic() - link.last_rx >= SILENCE_S:
                    link.reassert_dtr()
                    link.last_rx = time.monotonic()
        except KeyboardInterrupt:
            link.close()
            return 0
        except Exception as e:      # the port vanished: a reboot, DFU, the cable
            print('# port lost (%s); waiting for the radio' % e, flush=True)
            try:
                link.close()
            except Exception:
                pass
            time.sleep(1.0)


def cmd_info(args):
    link = connect(args)
    try:
        rep = link.command('INFO')
    finally:
        link.close()
    for line in rep.lines:
        r = parse_record(line)
        if r.type == 'INFO':
            print('%-12s %s' % (r.get('key', ''), r.get('value', '')))
        elif r.type == 'GET':
            print('  %-10s %s' % (r.get('name', ''), r.get('value', '')))
        else:
            print(line)
    return 0


def cmd_get(args):
    link = connect(args)
    try:
        rep = link.command('GET' + (' ' + setting_name(args.name) if args.name else ''))
    finally:
        link.close()
    for line in rep.lines:
        r = parse_record(line)
        print('%-10s %s' % (r.get('name', ''), r.get('value', '')) if r.type == 'GET' else line)
    return 0


def setting_name(name):
    return name.strip().upper().replace(' ', '_')


def cmd_set(args):
    name = setting_name(args.name)
    value = '_'.join(args.value)        # "2 COPIES" travels as 2_COPIES
    link = connect(args)
    try:
        link.command('SET %s %s' % (name, value))
        rep = link.command('GET %s' % name)
    finally:
        link.close()
    for line in rep.lines:
        r = parse_record(line)
        print('%-10s %s' % (r.get('name', ''), r.get('value', '')) if r.type == 'GET' else line)
    return 0


def cmd_time_sync(args):
    link = connect(args, sync=False)
    try:
        before = None
        for line in link.command('TIME').lines:
            r = parse_record(line)
            if r.type == 'TIME':
                before = r.num('epoch')
        now = int(time.time())
        link.command('TIME %d' % now)
    finally:
        link.close()
    if before:
        print('radio clock was %s (%+d s)' % (iso(before), before - now))
    else:
        print('radio clock was not set')
    print('set to %s' % iso(now))
    return 0


def download_log(link, out, last=None, progress=None):
    """LOG DUMP -> CSV on the text file `out`: a header row of the DEC field
    names, then one row per record, oldest first. Returns the row count."""
    try:
        link.command('CSV HDR')         # current field lists, in case a build appended some
    except (ConsoleError, ConsoleTimeout):
        pass
    w = csv.writer(out, lineterminator='\n')
    w.writerow(link.schema.fields.get('DEC', FIELDS['DEC']))
    count = [0]

    def row(line):
        if line.startswith('LOG,'):
            w.writerow(line.split(',')[1:])
            count[0] += 1
            if progress:
                progress(count[0])

    link.command('LOG DUMP' + (' %d' % last if last else ''), timeout=15.0, on_data=row)
    return count[0]


def cmd_log_download(args):
    link = connect(args)
    try:
        stat = None
        for line in link.command('LOG STAT').lines:
            r = parse_record(line)
            if r.type == 'LOG':
                stat = r
        if stat is not None:
            print('log %s: %s of %s records' % (stat.get('state'), stat.get('count'),
                                                 stat.get('capacity')), flush=True)
        if link.schema.schema not in (None, SCHEMA):
            print('# firmware schema %s, this client knows %d: columns follow the HDR'
                  % (link.schema.schema, SCHEMA), file=sys.stderr)

        def progress(n):
            if n % 500 == 0:
                print('  %d records' % n, file=sys.stderr, flush=True)

        with open(args.csv, 'w', encoding='ascii', errors='replace', newline='') as fh:
            n = download_log(link, fh, args.last, progress)
    finally:
        link.close()
    print('wrote %d records to %s' % (n, args.csv))
    return 0


def confirm(args, what):
    if args.yes:
        return True
    try:
        ans = input('%s - type YES to go ahead: ' % what)
    except EOFError:
        ans = ''
    return ans.strip() == 'YES'


def cmd_log_clear(args):
    if not confirm(args, 'erase every record in the radio\'s decode log'):
        print('not cleared')
        return 1
    link = connect(args)
    try:
        link.command('LOG CLEAR YES', timeout=60.0)     # erases 48 sectors
    finally:
        link.close()
    print('log cleared')
    return 0


def cmd_stations_upload(args):
    if args.blob:
        with open(args.blob, 'rb') as fh:
            blob = fh.read()
    else:
        blob = build_blob(args)
    try:
        hdr, problems = check_blob(blob)
    except ValueError as e:
        raise SystemExit('bad station blob: %s' % e)
    print('blob: %d bytes, %d sites, source %s, crc32 %08X'
          % (len(blob), hdr['count'], hdr['source'], zlib.crc32(blob) & 0xFFFFFFFF))
    for p in problems:
        print('# warning: %s (the radio will judge)' % p)
    if args.save_blob:
        with open(args.save_blob, 'wb') as fh:
            fh.write(blob)
    if args.dry_run:
        print('dry run: %d console lines at %d-byte chunks'
              % (len(list(stn_commands(blob, args.chunk))), args.chunk))
        return 0
    link = connect(args)
    try:
        t0 = time.time()
        shown = [-1]

        def progress(i, n):
            pct = 100 * i // n
            if pct // 10 != shown[0]:
                shown[0] = pct // 10
                print('  %3d%%' % pct, flush=True)

        size = upload_stations(link, blob, args.chunk, progress)
        info = link.command('STN INFO').lines
    finally:
        link.close()
    print('uploaded in %.0f s (%d-byte chunks)' % (time.time() - t0, size))
    for line in info:
        print(line)
    return 0


def cmd_stations_clear(args):
    if not confirm(args, 'erase the uploaded station table (the built-in one comes back)'):
        print('not cleared')
        return 1
    link = connect(args)
    try:
        link.command('STN CLEAR YES', timeout=30.0)
        info = link.command('STN INFO').lines
    finally:
        link.close()
    for line in info:
        print(line)
    return 0


def cmd_screenshot(args):
    link = connect(args)
    try:
        rep = link.command('SCREEN', timeout=5.0)
    finally:
        link.close()
    rows = dict(parse_scr(l) for l in rep.lines if l.startswith('SCR,'))
    missing = [r for r in range(8) if r not in rows]
    if missing:
        raise SystemExit('SCREEN answered without rows %s' % missing)
    with open(args.out, 'wb') as fh:
        fh.write(png_1bit(screen_pixels(rows), args.scale, args.invert))
    print('wrote %s (%dx%d)' % (args.out, 128 * args.scale, 64 * args.scale))
    return 0


def cmd_console(args):
    link = connect(args)
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                line = link.readline(0.2)
            except Exception as e:
                print('# port lost: %s' % e, flush=True)
                stop.set()
                return
            if line is not None and not (args.quiet and classify(line) in ('record', 'debug')):
                print(line, flush=True)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    print('# %s: type console commands (HELP); "quit" or Ctrl-C leaves' % link.dev, flush=True)
    try:
        while not stop.is_set():
            try:
                text = input()
            except EOFError:
                break
            text = text.strip()
            if text.lower() in ('quit', 'exit'):
                break
            if not text:
                continue
            if len(text) > MAX_CMD:
                print('# %d characters: the console takes %d' % (len(text), MAX_CMD), flush=True)
            link.send(text)
    except KeyboardInterrupt:
        pass
    stop.set()
    th.join(1.0)
    link.close()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', help='serial port (default: the first with USB VID 36B7)')
    ap.add_argument('--no-time', action='store_true',
                    help='do not set the radio clock on connect')
    ap.add_argument('--raw', help='append every received line to this file')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('live', help='decode table from DEC/STA/EVT; raw lines to a file')
    p.add_argument('--sta', type=float, default=60.0,
                   help='show an STA line at most every N s (0: all; default 60)')
    p.add_argument('--bursts', action='store_true', help='also show BST lines')
    p.add_argument('--debug-lines', action='store_true', help='also show D/A/B/K lines')
    p.set_defaults(fn=cmd_live)

    p = sub.add_parser('info', help='INFO: firmware, clock, battery, log, stations, settings')
    p.set_defaults(fn=cmd_info)

    p = sub.add_parser('get', help='GET [NAME]: settings')
    p.add_argument('name', nargs='?')
    p.set_defaults(fn=cmd_get)

    p = sub.add_parser('set', help='SET NAME VALUE (e.g. set CSV_OUT ON, set CONFIRM 2 COPIES)')
    p.add_argument('name')
    p.add_argument('value', nargs='+')
    p.set_defaults(fn=cmd_set)

    p = sub.add_parser('time-sync', help='report the radio clock, then set it')
    p.set_defaults(fn=cmd_time_sync)

    p = sub.add_parser('log-download', help='LOG DUMP to a CSV file')
    p.add_argument('--csv', required=True, help='output file')
    p.add_argument('--last', type=int, help='only the newest N records')
    p.set_defaults(fn=cmd_log_download)

    p = sub.add_parser('log-clear', help='LOG CLEAR YES: erase the decode log')
    p.add_argument('--yes', action='store_true', help='do not ask')
    p.set_defaults(fn=cmd_log_clear)

    p = sub.add_parser('stations-upload', help='build a station blob and upload it (STN)')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--filter', help='gen_stations filter (default filters/stations.filter)')
    g.add_argument('--all', action='store_true', help='the whole of MegaNet')
    g.add_argument('--blob', help='upload this prebuilt blob instead of running gen_stations')
    p.add_argument('--meganet-dir', help='local MegaNet checkout for gen_stations')
    p.add_argument('--save-blob', help='also keep the blob in this file')
    p.add_argument('--chunk', type=int, default=STN_MAX_CHUNK,
                   help='bytes per STN W line, 1-64 (default 64, falls back to 32)')
    p.add_argument('--dry-run', action='store_true', help='build and check, do not upload')
    p.set_defaults(fn=cmd_stations_upload)

    p = sub.add_parser('stations-clear', help='STN CLEAR YES: back to the built-in table')
    p.add_argument('--yes', action='store_true', help='do not ask')
    p.set_defaults(fn=cmd_stations_clear)

    p = sub.add_parser('screenshot', help='SCREEN to a 128x64 1-bit PNG')
    p.add_argument('out')
    p.add_argument('--scale', type=int, default=1, help='pixel scale (default 1)')
    p.add_argument('--invert', action='store_true', help='lit pixels white')
    p.set_defaults(fn=cmd_screenshot)

    p = sub.add_parser('console', help='interactive: type commands, see every line')
    p.add_argument('--quiet', action='store_true', help='hide the record/debug stream')
    p.set_defaults(fn=cmd_console)

    args = ap.parse_args(argv)
    if getattr(args, 'chunk', 1) not in range(1, STN_MAX_CHUNK + 1):
        ap.error('--chunk must be 1-%d' % STN_MAX_CHUNK)
    if getattr(args, 'scale', 1) < 1:
        ap.error('--scale must be at least 1')
    try:
        return args.fn(args)
    except (LinkError, ConsoleError, ConsoleTimeout) as e:
        print('alertterm: %s' % e, file=sys.stderr)
        if isinstance(e, ConsoleError) and e.reason == 'FOREIGN':
            print('alertterm: the station region holds data this firmware did not write. '
                  'If you are sure nothing else uses 0x1A0000-0x1AFFFF, take it over with\n'
                  '  alertterm.py console   then   STN FORMAT FORCE', file=sys.stderr)
        return 1
    except OSError as e:            # pyserial's SerialException is one: the port went away
        print('alertterm: serial port: %s' % e, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
