#!/usr/bin/env python3
"""Tests for alertterm.py, the reference client of docs/ALERT_SERIAL.md.

    python tools/alert/test_console.py [-v]

These tests do three things:

- Parse the exact example lines of the interface document with alertterm's
  parsers. Every example line in the document is checked: field counts, the
  200-character limit, payloads that re-decode to the same id/value, bursts
  whose bits decode to the frames reported for them, and sessions that end
  each command with exactly one final line. So the document and the client
  cannot drift apart unnoticed.
- Check the encoders: the PNG writer (decoded back with zlib), the STN
  chunking and the station blob, and the 0xABCD frames (against
  tools/serialtool/msg.py).
- Run the console protocol against a simulated radio: stream lines in the
  middle of a command, errors, timeouts, the 64 -> 32 byte upload fallback,
  and log download to CSV.

No test opens a serial port: alertterm.open_radio is replaced with a guard,
because a real radio may be plugged in and live.
"""
import contextlib
import io
import math
import os
import random
import re
import struct
import sys
import tempfile
import time
import unittest
import unittest.mock
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'tools', 'serialtool'))

import alertterm as at  # noqa: E402
import alertmon as am   # noqa: E402  the Python port of the firmware decoder
import msg              # noqa: E402  tools/serialtool/msg.py

DOC = os.path.join(REPO, 'docs', 'ALERT_SERIAL.md')


def _no_real_port(*a, **k):
    raise AssertionError('a test tried to open a real serial port')


at.open_radio = _no_real_port
at.find_ports = _no_real_port


# ---------------------------------------------------------------------------
# the document

def doc_text():
    with open(DOC, encoding='utf-8') as fh:
        return fh.read()


def doc_blocks():
    """Fenced code blocks -> [(info string, [lines])]."""
    blocks, cur, info = [], None, ''
    for line in doc_text().splitlines():
        if line.startswith('```'):
            if cur is None:
                cur, info = [], line[3:].strip()
            else:
                blocks.append((info, cur))
                cur = None
        elif cur is not None:
            cur.append(line)
    return blocks


KNOWN_TYPES = set(at.FIELDS) | set(at.CONSOLE_FIELDS) | {'HDR', 'LOG', 'STN', 'OK', 'ERR'}


def doc_radio_lines():
    """Every line in a plain block that the radio would send."""
    out = []
    for info, lines in doc_blocks():
        if info:
            continue
        for line in lines:
            if line.startswith('> '):
                continue
            if line.split(',', 1)[0] in KNOWN_TYPES:
                out.append(line)
    return out


def doc_sessions():
    """[(command, [reply lines])] from every block that has '> ' lines."""
    out = []
    for info, lines in doc_blocks():
        if info or not any(l.startswith('> ') for l in lines):
            continue
        cur = None
        for line in lines:
            if line.startswith('> '):
                cur = (line[2:], [])
                out.append(cur)
            elif cur is not None and line.strip():
                cur[1].append(line)
    return out


# Exact lines from the document; test_examples_are_in_the_doc keeps them honest.
EX = {
    'dec': 'DEC,1041,1790843886,187340,12,2088,MARBURG,BATT,143,14.3,V,ABF,STD,1,0,-20,-121,-109,89,412,16067B23,00010110000001100111101100100011',
    'dec2': 'DEC,1043,1790843919,220510,12,2442,KINGSHOLME MO,RAIN,23,23,tips,ABF,STD,1,1,-47,-121,-109,62,530,52667703,01010010011001100111011100000011',
    'dec_lvl': 'DEC,1045,1790843967,268870,12,4110,ROTHWELL,LVL,12,12,,ABF,STD,1,0,-104,-121,-109,5,390,72029B03,01110010000000101001101100000011',
    'dec_unknown': 'DEC,7,,95230,,3001,,,57,57,,EIF,NEG,0,0,-61,-119,-107,46,380,9F753812,10011111011101010011100000010010',
    'bst': 'BST,14,1790843886,187340,-20,-121,412,1,100,0000003D2FCB08EE0109001060',
    'sta': 'STA,1790843970,270000,-121,-124,0,7890,78,16,18,-104,OK,1045,12065,BUILTIN MegaNet:95f6f8d',
    'sta_noclock': 'STA,,31000,-119,-122,0,7650,61,0,0,,FOREIGN,0,0,BUILTIN MegaNet:95f6f8d',
    'evt': 'EVT,1790843790,91240,SET,SNR_REQ=14',
    'evt_boot': 'EVT,,5230,BOOT,POR 12',
    'hdr_fw': 'HDR,fw,4d06107f,schema,2',
    'hdr_dec': 'HDR,DEC,seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,frame,rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin',
    'log_stat': 'LOG,1045,12065,1,1045,OK',
    'log_empty': 'LOG,0,12065,,,OK',
    'log_dump': 'LOG,1044,1790843962,263880,12,4109,ROTHWELL,RAIN,1290,1290,tips,ABF,STD,1,0,-88,-121,-109,21,455,B202AB17,10110010000000101010101100010111',
    'stn_get': 'STN,2088,MARBURG,BATT',
    'stn_miss': 'STN,3001,,',
    'stn_info': 'STN,SPI MegaNet:95f6f8d,2604,3E8F0A61',
    'stn_builtin': 'STN,BUILTIN MegaNet:95f6f8d,443,',
    'get': 'GET,CONFIRM,2 COPIES',
    'info_log': 'INFO,log,OK,1040,12065',
    'time': 'TIME,1790843762',
    'spi': 'SPI,1C0000,4153544201002C0A24C40000610A8F3E4D6567614E65743A3935663666386400',
    'scr1': 'SCR,1,FF' + '00' * 126 + 'FF',
    'ok': 'OK',
    'err': 'ERR,RANGE',
}


class TestDocExamples(unittest.TestCase):
    """The exact example lines, parsed and checked field by field."""

    def test_examples_are_in_the_doc(self):
        text = doc_text()
        for name, line in EX.items():
            self.assertIn(line, text, name)

    def test_dec(self):
        r = at.parse_record(EX['dec'])
        self.assertEqual(r.type, 'DEC')
        self.assertEqual(r.extra, [])
        self.assertEqual(len(r), 21)
        self.assertEqual((r.num('seq'), r.num('epoch'), r.num('uptime_ms'), r.num('boot')),
                         (1041, 1790843886, 187340, 12))
        self.assertEqual((r.num('id'), r['name'], r['kind'], r.num('value')),
                         (2088, 'MARBURG', 'BATT', 143))
        self.assertEqual((r['eng'], r['unit'], r['fmt'], r['pol'], r.num('inv'), r.num('frame')),
                         ('14.3', 'V', 'ABF', 'STD', 1, 0))
        self.assertEqual((r.num('rssi'), r.num('nf'), r.num('sens'), r.num('fade'), r.num('burst_ms')),
                         (-20, -121, -109, 89, 412))
        # sens = nf + SNR_REQ (12), fade = rssi - sens (V2_SPEC section 4)
        self.assertEqual(r.num('sens'), r.num('nf') + 12)
        self.assertEqual(r.num('fade'), r.num('rssi') - r.num('sens'))

    def test_dec_unknowns_are_empty(self):
        r = at.parse_record(EX['dec_unknown'])
        self.assertIsNone(r.num('epoch'))
        self.assertEqual((r['name'], r['kind'], r['unit']), ('', '', ''))
        self.assertEqual((r.num('boot'), r.num('seq')), (None, 7))
        r = at.parse_record(EX['dec_lvl'])
        self.assertEqual((r['kind'], r['eng'], r['unit']), ('LVL', '12', ''))

    def test_dec_payload_redecodes(self):
        for key in ('dec', 'dec2', 'dec_lvl', 'dec_unknown', 'log_dump'):
            check_payload(self, at.parse_record(EX[key]))

    def test_bst(self):
        r = at.parse_record(EX['bst'])
        self.assertEqual(r.extra, [])
        self.assertEqual((r.num('seq'), r.num('uptime_ms'), r.num('peak'), r.num('nf'),
                          r.num('burst_ms'), r.num('nframes'), r.num('nbits')),
                         (14, 187340, -20, -121, 412, 1, 100))
        self.assertEqual(len(r['bits_hex']), 2 * math.ceil(100 / 8))
        found = decode_bits(r)
        self.assertIn(('ABF', 2088, 143), found)

    def test_sta(self):
        r = at.parse_record(EX['sta'])
        self.assertEqual(r.extra, [])
        self.assertEqual((r.num('nf'), r.num('rssi'), r.num('sq'), r.num('batt_mv'),
                          r.num('batt_pct'), r.num('bursts'), r.num('decodes'), r.num('min_ok')),
                         (-121, -124, 0, 7890, 78, 16, 18, -104))
        self.assertEqual((r['log_state'], r.num('log_count'), r.num('log_cap'), r['stn_src']),
                         ('OK', 1045, 12065, 'BUILTIN MegaNet:95f6f8d'))
        r = at.parse_record(EX['sta_noclock'])
        self.assertIsNone(r.num('epoch'))
        self.assertIsNone(r.num('min_ok'))
        self.assertEqual(r['log_state'], 'FOREIGN')

    def test_evt_detail_takes_the_rest(self):
        r = at.parse_record(EX['evt'])
        self.assertEqual((r.num('epoch'), r.num('uptime_ms'), r['code'], r['detail']),
                         (1790843790, 91240, 'SET', 'SNR_REQ=14'))
        r = at.parse_record('EVT,,100,LOG,ERR erase, sector 3')
        self.assertEqual(r['detail'], 'ERR erase, sector 3')
        self.assertEqual(r.extra, [])
        self.assertEqual(at.parse_record(EX['evt_boot'])['code'], 'BOOT')

    def test_hdr(self):
        s = at.Schema()
        self.assertTrue(s.feed(EX['hdr_fw']))
        self.assertTrue(s.feed(EX['hdr_dec']))
        self.assertFalse(s.feed(EX['dec']))
        self.assertEqual((s.fw, s.schema), ('4d06107f', 2))
        self.assertEqual(s.fields['DEC'], at.FIELDS['DEC'])

    def test_hdr_appended_field_is_read_by_name(self):
        s = at.Schema()
        s.feed(EX['hdr_dec'] + ',snr')
        r = at.parse_record(EX['dec'] + ',33', s.fields)
        self.assertEqual((r['snr'], r['name'], r.extra), ('33', 'MARBURG', []))
        r = at.parse_record(EX['dec'] + ',33')          # no HDR seen: kept, unnamed
        self.assertEqual(r.extra, ['33'])

    def test_log_shapes(self):
        r = at.parse_record(EX['log_stat'])
        self.assertEqual((r.num('count'), r.num('capacity'), r.num('oldest_seq'),
                          r.num('newest_seq'), r['state']), (1045, 12065, 1, 1045, 'OK'))
        r = at.parse_record(EX['log_empty'])
        self.assertEqual((r.num('count'), r.num('oldest_seq'), r['state']), (0, None, 'OK'))
        r = at.parse_record(EX['log_dump'])
        self.assertEqual((r.num('seq'), r.num('id'), r['name'], r['eng'], r['unit']),
                         (1044, 4109, 'ROTHWELL', '1290', 'tips'))

    def test_stn_shapes(self):
        r = at.parse_record(EX['stn_get'])
        self.assertEqual((r.num('id'), r['name'], r['kind']), (2088, 'MARBURG', 'BATT'))
        r = at.parse_record(EX['stn_miss'])
        self.assertEqual((r.num('id'), r['name'], r['kind']), (3001, '', ''))
        r = at.parse_record(EX['stn_info'])
        self.assertEqual((r['source'], r.num('count'), r['crc']),
                         ('SPI MegaNet:95f6f8d', 2604, '3E8F0A61'))
        self.assertEqual(at.parse_record(EX['stn_builtin'])['crc'], '')

    def test_console_data_lines(self):
        self.assertEqual(at.parse_record(EX['get'])['value'], '2 COPIES')
        r = at.parse_record(EX['info_log'])
        self.assertEqual((r['key'], r['value']), ('log', 'OK,1040,12065'))
        self.assertEqual(at.parse_record(EX['time']).num('epoch'), 1790843762)
        self.assertIsNone(at.parse_record('TIME,').num('epoch'))
        r = at.parse_record(EX['spi'])
        self.assertEqual(r['addr'], '1C0000')
        self.assertEqual(len(bytes.fromhex(r['hex'])), 32)

    def test_final_lines(self):
        self.assertEqual(at.parse_final(EX['ok']), (True, ''))
        self.assertEqual(at.parse_final('OK,3 written'), (True, '3 written'))
        self.assertEqual(at.parse_final(EX['err']), (False, 'RANGE'))
        self.assertEqual(at.parse_final('ERR'), (False, ''))

    def test_classify(self):
        cases = {
            'OK': 'final', 'OK,x': 'final', 'ERR': 'final', 'ERR,RANGE': 'final',
            EX['dec']: 'record', EX['bst']: 'record', EX['sta']: 'record', EX['evt']: 'record',
            EX['hdr_fw']: 'record', 'ALERT,2088,143,ABF,-20,MARBURG': 'record',
            'D I0 F0 G0 B0 R-73 Q0 N0 f-142 p-127 V0 C0': 'debug',
            'K dfu ok=1': 'debug', 'B ver=4d06107f rst=sw n=0 bl=22CDCECB': 'debug',
            EX['log_stat']: 'data', EX['log_dump']: 'data', EX['stn_get']: 'data',
            EX['scr1']: 'data', EX['get']: 'data', EX['info_log']: 'data',
            EX['time']: 'data', EX['spi']: 'data', 'HELP,SCREEN': 'data',
        }
        for line, want in cases.items():
            self.assertEqual(at.classify(line), want, line[:30])

    def test_scr(self):
        row, data = at.parse_scr(EX['scr1'])
        self.assertEqual((row, len(data), data[0], data[1], data[127]), (1, 128, 0xFF, 0, 0xFF))
        with self.assertRaises(ValueError):
            at.parse_scr('SCR,8,' + '00' * 128)
        with self.assertRaises(ValueError):
            at.parse_scr('SCR,0,' + '00' * 127)

    def test_format_dec(self):
        row = at.format_dec(at.parse_record(EX['dec']), '08:38:06')
        for part in ('08:38:06', '2088', 'MARBURG', 'BATT', '14.3 V', '-20', '+89', 'ABF STD INV'):
            self.assertIn(part, row)
        row = at.format_dec(at.parse_record(EX['dec_unknown']))
        self.assertIn('?', row)
        self.assertIn('EIF NEG', row)
        self.assertNotIn('INV', row)
        self.assertIn('BATT 78% 7.89V', at.format_sta(at.parse_record(EX['sta'])))


def check_payload(tc, r):
    """payload_hex and payload_bin agree, and re-decode to fmt/id/value."""
    p = int(r['payload_hex'], 16)
    tc.assertEqual(len(r['payload_hex']), 8)
    tc.assertEqual(r['payload_bin'], format(p, '032b'))
    tc.assertEqual(am.decode_payload32(p), (r['fmt'], r.num('id'), r.num('value')),
                   'payload of %s %s' % (r.type, r.get('seq')))


def decode_bits(r):
    """(fmt, id, value) found in a BST line's bits, in both senses."""
    data = bytes.fromhex(r['bits_hex'])
    n = min(r.num('nbits'), 8 * len(data))
    found = set()
    for buf in (data, bytes(b ^ 0xFF for b in data)):
        for hit in am.scan_all(buf, n):
            found.add(hit[0])
    return found


class TestWholeDoc(unittest.TestCase):
    """Every example in the document, not just the ones copied above."""

    def test_every_radio_line_parses(self):
        lines = doc_radio_lines()
        self.assertGreater(len(lines), 60)
        schema = at.Schema()
        for line in lines:
            typ = line.split(',', 1)[0]
            r = at.parse_record(line)
            if typ in ('DEC', 'BST', 'STA', 'EVT', 'HDR'):
                self.assertLessEqual(len(line), 200, line[:40])
            if typ in ('DEC', 'BST', 'STA'):
                self.assertEqual(len(line.split(',')) - 1, len(at.FIELDS[typ]), line[:40])
                self.assertEqual(r.extra, [], line[:40])
            if typ == 'EVT':
                self.assertIn(r['code'], ('BOOT', 'CENSUS', 'SET', 'LOG', 'STN', 'CLOCK'))
            if typ == 'DEC' or (typ == 'LOG' and 'payload_hex' in r):
                self.assertEqual(len(line.split(',')) - 1, 21, line[:40])
                check_payload(self, r)
                self.assertIn(r['pol'], ('STD', 'NEG'))
                self.assertIn(r['fmt'], ('ABF', 'EIF', 'A2C'))
                self.assertIn(r['kind'], ('RAIN', 'LVL', 'BATT', 'REP', 'SNSR', 'CHK', ''))
            if typ == 'LOG' and 'state' in r:
                self.assertIn(r['state'], ('OK', 'OFF', 'FOREIGN', 'ERR'))
            if typ == 'SCR':
                self.assertEqual(len(line), 262)
                at.parse_scr(line)
            if typ == 'HDR':
                schema.feed(line)
        self.assertEqual(schema.schema, at.SCHEMA)
        for typ, fields in at.FIELDS.items():
            self.assertEqual(schema.fields[typ], fields, 'HDR,%s in the doc' % typ)

    def test_bursts_match_their_frames(self):
        lines = sorted(set(doc_radio_lines()))          # the quick start repeats a DEC line
        decs = [at.parse_record(l) for l in lines if l.startswith('DEC,')]
        bsts = [at.parse_record(l) for l in lines if l.startswith('BST,')]
        self.assertGreaterEqual(len(bsts), 3)
        for b in bsts:
            mine = [d for d in decs if d['uptime_ms'] == b['uptime_ms']]
            self.assertEqual(len(mine), b.num('nframes'), b['uptime_ms'])
            found = decode_bits(b)
            for d in mine:
                self.assertIn((d['fmt'], d.num('id'), d.num('value')), found)
                self.assertEqual((d['epoch'], d['nf'], d['rssi'], d['burst_ms']),
                                 (b['epoch'], b['nf'], b['peak'], b['burst_ms']))
            self.assertEqual(sorted(d.num('frame') for d in mine), list(range(len(mine))))

    def test_sessions(self):
        sessions = doc_sessions()
        self.assertGreater(len(sessions), 30)
        for cmd, reply in sessions:
            self.assertLessEqual(len(cmd), at.MAX_CMD, cmd[:30])
            self.assertEqual(cmd, cmd.strip())
            kinds = [at.classify(l) for l in reply]
            self.assertEqual(kinds.count('final'), 1, '%s -> %s' % (cmd, reply))
            # the final line ends the reply: only stream lines may follow it
            after = kinds[kinds.index('final') + 1:]
            self.assertTrue(all(k == 'record' for k in after), cmd)
            self.assertNotIn('debug', kinds)
            self.assertTrue(all(l.split(',', 1)[0] in KNOWN_TYPES for l in reply), cmd)

    def test_python_blocks_compile(self):
        blocks = [lines for info, lines in doc_blocks() if info == 'python']
        self.assertTrue(blocks)
        for lines in blocks:
            compile('\n'.join(lines), 'ALERT_SERIAL.md', 'exec')

    def test_frames_in_the_doc(self):
        pat = re.compile(r'^(0x[0-9A-F]{4})\S*(?:\s+"[^"]*")?(?:\s+id=([0-9A-F]{8}))?\s+'
                         r'((?:[0-9A-F]{2} )+[0-9A-F]{2})$')
        seen = {}
        for info, lines in doc_blocks():
            for line in lines:
                m = pat.match(line.strip())
                if m:
                    seen[int(m.group(1), 16)] = (m.group(2), bytes.fromhex(m.group(3)))
        self.assertEqual(sorted(seen), [0x0514, 0x05DD, 0x05E0, 0x05E1])
        bodies = {0x05E1: b'', 0x05DD: b'', 0x05E0: struct.pack('<I', 0x44465521),
                  0x0514: struct.pack('<I', int(seen[0x0514][0], 16))}
        for mid, (_, frame) in seen.items():
            self.assertEqual(frame, at.make_frame(mid, bodies[mid]), hex(mid))
            self.assertEqual(frame, reference_frame(mid, bodies[mid]), hex(mid))

    def test_screen_example_is_a_border(self):
        rows = dict(at.parse_scr(l) for l in doc_radio_lines() if l.startswith('SCR,'))
        self.assertEqual(sorted(rows), list(range(8)))
        px = at.screen_pixels(rows)
        self.assertEqual(sum(map(sum, px)), 2 * 128 + 2 * 62)
        for x in range(128):
            self.assertEqual((px[0][x], px[63][x]), (1, 1))
        for y in range(64):
            self.assertEqual((px[y][0], px[y][127]), (1, 1))
            if 0 < y < 63:
                self.assertEqual(sum(px[y][1:127]), 0)

    def test_upload_example_adds_up(self):
        text = doc_text()
        spi = at.parse_record(EX['spi'])
        hdr = at.parse_blob_header(bytes.fromhex(spi['hex']))
        self.assertEqual((hdr['version'], hdr['count'], hdr['names_len'], hdr['source']),
                         (1, 2604, 50212, 'MegaNet:95f6f8d'))
        self.assertEqual('%08X' % hdr['crc'], at.parse_record(EX['stn_info'])['crc'])
        size = at.ASTB.size + 8 * hdr['count'] + hdr['names_len']
        self.assertIn('> STN BEGIN %d ' % size, text)
        self.assertIn('> STN W 0 %s' % spi['hex'], text)             # chunk 0 is the header
        last = 32 * ((size - 1) // 32)
        m = re.search(r'> STN W %d ([0-9A-F]+)' % last, text)
        self.assertIsNotNone(m)
        self.assertEqual(len(m.group(1)) // 2, size - last)
        more = math.ceil(size / 32) - 3
        self.assertIn('%s more `STN W` lines' % format(more, ','), text)
        for off, hexs in re.findall(r'> STN W (\d+) ([0-9A-F]+)', text):
            self.assertEqual(int(off) % 32, 0)
            self.assertLessEqual(len(hexs), 64)


def reference_frame(mid, body):
    m = msg.Msg.make(mid, len(body))
    m.buf[4:] = body
    return bytes(msg.make_packet(bytes(m.buf)))


# ---------------------------------------------------------------------------
# encoders

def decode_png(data):
    """-> (width, height, bit depth, color type, rows of 0/1). Checks every CRC."""
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    pos, chunks = 8, []
    while pos < len(data):
        n, kind = struct.unpack('>I4s', data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        crc, = struct.unpack('>I', data[pos + 8 + n:pos + 12 + n])
        assert zlib.crc32(kind + body) & 0xFFFFFFFF == crc, kind
        chunks.append((kind, body))
        pos += 12 + n
    assert chunks[0][0] == b'IHDR' and chunks[-1] == (b'IEND', b'')
    w, h, depth, ctype, comp, filt, lace = struct.unpack('>IIBBBBB', chunks[0][1])
    assert (comp, filt, lace) == (0, 0, 0)
    raw = zlib.decompress(b''.join(b for k, b in chunks if k == b'IDAT'))
    stride = (w * depth + 7) // 8
    assert len(raw) == h * (stride + 1)
    rows = []
    for y in range(h):
        line = raw[y * (stride + 1):(y + 1) * (stride + 1)]
        assert line[0] == 0, 'filter type'
        rows.append([(line[1 + (x >> 3)] >> (7 - (x & 7))) & 1 for x in range(w)])
    return w, h, depth, ctype, rows


class TestPng(unittest.TestCase):
    def setUp(self):
        rnd = random.Random(5)
        self.px = [[rnd.getrandbits(1) for _ in range(128)] for _ in range(64)]

    def test_round_trip(self):
        w, h, depth, ctype, rows = decode_png(at.png_1bit(self.px))
        self.assertEqual((w, h, depth, ctype), (128, 64, 1, 0))
        # lit = black = 0 in a grayscale PNG
        self.assertEqual(rows, [[1 - p for p in row] for row in self.px])

    def test_invert(self):
        _, _, _, _, rows = decode_png(at.png_1bit(self.px, invert=True))
        self.assertEqual(rows, self.px)

    def test_scale(self):
        w, h, _, _, rows = decode_png(at.png_1bit(self.px, scale=3))
        self.assertEqual((w, h), (384, 192))
        for y in (0, 5, 100, 191):
            for x in (0, 7, 200, 383):
                self.assertEqual(rows[y][x], 1 - self.px[y // 3][x // 3])

    def test_from_scr_bytes(self):
        rows = {r: bytes(128) for r in range(8)}
        rows[2] = bytes([0x00] * 10 + [0x81] + [0x00] * 117)   # x=10: y=16 and y=23
        px = at.screen_pixels(rows)
        self.assertEqual([(x, y) for y in range(64) for x in range(128) if px[y][x]],
                         [(10, 16), (10, 23)])
        _, _, _, _, png_rows = decode_png(at.png_1bit(px))
        self.assertEqual((png_rows[16][10], png_rows[23][10], png_rows[17][10]), (0, 0, 1))


def build_test_blob(sites, source=b'MegaNet:test123'):
    """An ASTB blob laid out as V2_SPEC section 8 says."""
    entries, names = b'', b''
    for base, kinds, name in sorted(sites):
        entries += struct.pack('<HHI', base, kinds, len(names))
        names += name.encode('ascii') + b'\0'
    body = entries + names
    return struct.pack('<4sHHII16s', b'ASTB', 1, len(sites), len(names),
                       zlib.crc32(body) & 0xFFFFFFFF, source) + body


SITES = [(2086, 0x00D1, 'MARBURG'), (4109, 0x00D1, 'ROTHWELL'),
         (2442, 0x0019, 'KINGSHOLME MOUNTAIN RD'), (4132, 0x00D1, 'BEACHMERE STATE SCHOOL')]


class TestStationBlob(unittest.TestCase):
    def test_check_blob(self):
        blob = build_test_blob(SITES)
        hdr, problems = at.check_blob(blob)
        self.assertEqual(problems, [])
        self.assertEqual((hdr['count'], hdr['source'], hdr['version']), (4, 'MegaNet:test123', 1))
        bad = bytearray(blob)
        bad[-2] ^= 1
        self.assertTrue(any('crc32' in p for p in at.check_blob(bytes(bad))[1]))
        self.assertTrue(any('describes' in p for p in at.check_blob(blob[:-3])[1]))
        with self.assertRaises(ValueError):
            at.check_blob(b'ASTX' + blob[4:])
        with self.assertRaises(ValueError):
            at.check_blob(blob[:20])

    def test_chunking(self):
        blob = bytes(range(256)) * 2 + b'\x01\x02\x03'           # 515 bytes
        for chunk in (1, 7, 32, 64):
            cmds = list(at.stn_commands(blob, chunk))
            self.assertEqual(cmds[0], 'STN BEGIN 515 %08X' % (zlib.crc32(blob) & 0xFFFFFFFF))
            self.assertEqual(cmds[-1], 'STN END')
            self.assertEqual(len(cmds), 2 + math.ceil(len(blob) / chunk))
            got, expect_off = bytearray(), 0
            for c in cmds[1:-1]:
                _, _, off, hexs = c.split(' ')
                self.assertEqual(int(off), expect_off)
                piece = bytes.fromhex(hexs)
                self.assertLessEqual(len(piece), chunk)
                self.assertEqual(hexs, hexs.upper())
                got += piece
                expect_off += len(piece)
            self.assertEqual(bytes(got), blob)

    def test_chunk_limits(self):
        for bad in (0, 65):
            with self.assertRaises(ValueError):
                list(at.stn_commands(b'x', bad))
        self.assertEqual(list(at.stn_commands(b'')), ['STN BEGIN 0 00000000', 'STN END'])

    def test_line_lengths(self):
        big = bytes(at.STN_BLOB_MAX)
        longest32 = max(len(c) for c in at.stn_commands(big, 32))
        longest64 = max(len(c) for c in at.stn_commands(big, 64))
        self.assertLessEqual(longest32, at.MAX_CMD)          # 32-byte chunks always fit
        self.assertEqual((longest32, longest64), (77, 141))  # the numbers the doc quotes
        self.assertIn('is 77 characters', doc_text())
        self.assertIn('141-character line', doc_text())


class TestLineReader(unittest.TestCase):
    def test_split_across_reads(self):
        r = at.LineReader()
        self.assertEqual(r.feed(b'DEC,1,2'), [])
        self.assertEqual(r.feed(b',3\r'), ['DEC,1,2,3'])
        self.assertEqual(r.feed(b'\nOK\r\n\r\nERR,RANGE\n'), ['OK', 'ERR,RANGE'])
        self.assertEqual(r.feed(b'STA,,1\rOK\r'), ['STA,,1', 'OK'])

    def test_frames_are_cut_out(self):
        r = at.LineReader()
        frame = at.make_frame(0x0515, bytes(36))
        data = b'OK\r\n' + frame + b'DEC,1\r\n' + at.make_frame(0x0518)
        out = []
        for i in range(0, len(data), 5):                        # dribbled in 5-byte reads
            out += r.feed(data[i:i + 5])
        self.assertEqual(out, ['OK', 'DEC,1'])
        self.assertEqual(r.frame_ids, [0x0515, 0x0518])
        self.assertTrue(r.bootloader)

    def test_noise(self):
        r = at.LineReader()
        self.assertEqual(r.feed(b'\xab\x00OK\r\n'), ['OK'])      # a stray 0xAB is dropped
        self.assertFalse(r.bootloader)
        self.assertEqual(r.feed(b'x' * 2000), [])
        self.assertEqual(r.feed(b'\r\nOK\r\n'), ['OK'])       # the runaway line was dropped


class TestFrames(unittest.TestCase):
    def test_matches_serialtool(self):
        for mid, body in ((0x0514, b'\x2d\x1c\x0b\x6a'), (0x05DD, b''), (0x05E1, b''),
                          (0x05E0, b'!UFD'), (0x0601, b'\x58'), (0x051B, bytes(range(9)))):
            self.assertEqual(at.make_frame(mid, body), reference_frame(mid, body), hex(mid))

    def test_crc(self):
        self.assertEqual(at.crc16_xmodem(b'123456789'), 0x31C3)   # the XMODEM check value


# ---------------------------------------------------------------------------
# the console protocol, against a simulated radio

HDR_BLOCK = [EX['hdr_fw'], EX['hdr_dec'],
             'HDR,BST,' + ','.join(at.FIELDS['BST']),
             'HDR,STA,' + ','.join(at.FIELDS['STA']),
             'HDR,EVT,' + ','.join(at.FIELDS['EVT'])]


class FakeRadio(object):
    """Enough of the console to exercise the client, over pyserial's API."""

    def __init__(self, max_line=at.MAX_CMD, truncate=False):
        self.rxbuf = bytearray()
        self.txbuf = bytearray()
        self.dtr = self.rts = True
        self.max_line, self.truncate = max_line, truncate
        self.commands = []
        self.epoch = None
        self.settings = {'CSV_OUT': 'ON', 'SNR_REQ': '12', 'CONFIRM': 'OFF'}
        self.log = [EX['log_dump'].split(',')[1:], EX['dec_lvl'].split(',')[1:]]
        self.hdr = list(HDR_BLOCK)
        self.upload = None
        self.stn = None
        self.inject = None           # a stream line sent in the middle of every reply
        self.mute = False            # never answer
        self.double_final = False    # a second bare ERR after ERR,<reason>
        self.fail_end = False
        self.foreign = False         # the station region holds someone else's data

    # pyserial's side
    @property
    def in_waiting(self):
        return len(self.txbuf)

    def read(self, n=1):
        d = bytes(self.txbuf[:n])
        del self.txbuf[:n]
        return d

    def write(self, data):
        self.rxbuf += data
        while b'\n' in self.rxbuf:
            raw, _, rest = bytes(self.rxbuf).partition(b'\n')
            self.rxbuf = bytearray(rest)
            self.line(raw.decode('ascii').strip())
        return len(data)

    def reset_input_buffer(self):
        del self.txbuf[:]

    def close(self):
        pass

    # the radio's side
    def say(self, *lines):
        for l in lines:
            self.txbuf += (l + '\r\n').encode('ascii')

    def err(self, reason):
        self.say('ERR,' + reason)
        if self.double_final:
            self.say('ERR')

    def line(self, text):
        if not text or self.mute:
            return
        if len(text) > self.max_line:
            if not self.truncate:
                return self.err('TOOLONG')
            text = text[:self.max_line]
        self.commands.append(text)
        w = text.split()
        up = [x.upper() for x in w]
        if self.inject:
            self.say(self.inject)
        try:
            handler = getattr(self, 'c_' + up[0])
        except AttributeError:
            return self.err('UNKNOWN')
        handler(w, up)

    def c_TIME(self, w, up):
        if len(w) == 1:
            return self.say('TIME,%s' % (self.epoch or ''), 'OK')
        if not w[1].isdigit():
            return self.err('ARGS')
        self.epoch = int(w[1])
        self.say('OK', 'EVT,%d,61020,CLOCK,SET' % self.epoch)

    def c_CSV(self, w, up):
        self.say(*self.hdr)
        self.say('OK')

    def c_GET(self, w, up):
        names = [up[1]] if len(up) > 1 else sorted(self.settings)
        if any(n not in self.settings for n in names):
            return self.err('NAME')
        self.say(*['GET,%s,%s' % (n, self.settings[n]) for n in names])
        self.say('OK')

    def c_SET(self, w, up):
        if len(up) < 3:
            return self.err('ARGS')
        if up[1] not in self.settings:
            return self.err('NAME')
        value = ' '.join(up[2:]).replace('_', ' ')
        if up[1] == 'SNR_REQ' and not 6 <= int(value) <= 20:
            return self.err('RANGE')
        self.settings[up[1]] = value
        self.say('OK')

    def c_LOG(self, w, up):
        if up[1] == 'STAT':
            self.say('LOG,%d,12065,%s,%s,OK' % (len(self.log), self.log[0][0], self.log[-1][0]))
        elif up[1] == 'DUMP':
            n = int(up[2]) if len(up) > 2 else len(self.log)
            for i, rec in enumerate(self.log[-n:]):
                self.say('LOG,' + ','.join(rec))
                if i == 0:
                    self.say(EX['sta'])          # the stream goes on during a dump
        elif up[1:] == ['CLEAR', 'YES']:
            self.log = []
        else:
            return self.err('CONFIRM' if up[1] == 'CLEAR' else 'ARGS')
        self.say('OK')

    def c_STN(self, w, up):
        sub = up[1]
        if self.foreign and sub in ('BEGIN', 'CLEAR'):
            return self.err('FOREIGN')
        if sub == 'BEGIN':
            n, crc = int(w[2]), int(w[3], 16)
            if n > at.STN_BLOB_MAX:
                return self.err('RANGE')
            self.upload, self.crc = bytearray(b'\xff' * n), crc
            self.reached = 4096           # the firmware erases as it goes, in order
        elif sub == 'W':
            if self.upload is None:
                return self.err('STATE')
            try:
                off, data = int(w[2]), bytes.fromhex(w[3])
            except (ValueError, IndexError):
                return self.err('ARGS')
            if not 1 <= len(data) <= 64 or off + len(data) > len(self.upload) or off > self.reached:
                return self.err('STATE')
            while self.reached < off + len(data):
                self.reached += 4096
            self.upload[off:off + len(data)] = data
        elif sub == 'END':
            if self.upload is None:
                return self.err('STATE')
            if self.fail_end or zlib.crc32(bytes(self.upload)) & 0xFFFFFFFF != self.crc:
                self.upload = None
                return self.err('CRC')
            self.stn, self.upload = bytes(self.upload), None
        elif sub == 'INFO':
            self.say('STN,SPI MegaNet:test123,4,00000000' if self.stn else
                     'STN,BUILTIN MegaNet:95f6f8d,443,')
        elif up[1:] == ['CLEAR', 'YES']:
            self.stn = None
        else:
            return self.err('ARGS')
        self.say('OK')

    def c_SCREEN(self, w, up):
        for r in range(8):
            fill = 0x01 if r == 0 else (0x80 if r == 7 else 0x00)
            self.say('SCR,%d,%s' % (r, bytes([0xFF] + [fill] * 126 + [0xFF]).hex().upper()))
        self.say('OK')


class TestConsole(unittest.TestCase):
    def link(self, radio=None, stream=None):
        self.radio = radio or FakeRadio()
        self.stream = [] if stream is None else stream
        return at.Link(self.radio, on_async=self.stream.append, dev='FAKE')

    def test_reply_and_stream_are_separated(self):
        link = self.link()
        self.radio.inject = EX['sta']
        rep = link.command('GET CSV_OUT')
        self.assertEqual(rep.lines, ['GET,CSV_OUT,ON'])
        self.assertEqual(self.stream, [EX['sta']])

    def test_err_raises_with_reason(self):
        link = self.link()
        with self.assertRaises(at.ConsoleError) as cm:
            link.command('SET SNR_REQ 30')
        self.assertEqual(cm.exception.reason, 'RANGE')
        with self.assertRaises(at.ConsoleError) as cm:
            link.command('FOO')
        self.assertEqual(cm.exception.reason, 'UNKNOWN')

    def test_a_late_second_final_line_does_not_end_the_next_command(self):
        link = self.link()
        self.radio.double_final = True
        with self.assertRaises(at.ConsoleError):
            link.command('SET SNR_REQ 30')
        self.assertEqual(link.command('GET SNR_REQ').lines, ['GET,SNR_REQ,12'])

    def test_timeout(self):
        link = self.link()
        self.radio.mute = True
        t0 = time.monotonic()
        with self.assertRaises(at.ConsoleTimeout):
            link.command('INFO', timeout=0.2)
        self.assertLess(time.monotonic() - t0, 2.0)

    def test_timeout_names_the_bootloader(self):
        link = self.link()
        self.radio.mute = True
        self.radio.txbuf += at.make_frame(0x0518, bytes(8))
        with self.assertRaises(at.ConsoleTimeout) as cm:
            link.command('INFO', timeout=0.1)
        self.assertIn('bootloader', str(cm.exception))

    def test_hdr_updates_the_schema_even_mid_command(self):
        link = self.link()
        self.radio.hdr[1] += ',snr'
        link.command('CSV HDR')
        self.assertEqual(link.schema.fields['DEC'][-1], 'snr')
        self.assertEqual((link.schema.fw, link.schema.schema), ('4d06107f', 2))

    def test_sync_clock(self):
        link = self.link()
        at.sync_clock(link)
        self.assertLessEqual(abs(self.radio.epoch - time.time()), 2)
        # the EVT came after the OK: it waits for the next read, then goes to the stream
        self.assertEqual(self.stream, [])
        link.drain()
        self.assertEqual(self.stream[-1].split(',')[3], 'CLOCK')

    def test_upload_at_64(self):
        link = self.link(FakeRadio(max_line=160))
        blob = build_test_blob(SITES)
        self.assertEqual(at.upload_stations(link, blob, note=lambda s: None), 64)
        self.assertEqual(self.radio.stn, blob)
        self.assertEqual(sum(c.startswith('STN BEGIN') for c in self.radio.commands), 1)

    def test_upload_falls_back_when_long_lines_are_refused(self):
        link = self.link()
        blob = build_test_blob(SITES * 3)
        notes = []
        self.assertEqual(at.upload_stations(link, blob, note=notes.append), 32)
        self.assertEqual(self.radio.stn, blob)
        self.assertEqual(len(notes), 1)
        self.assertIn('TOOLONG', notes[0])

    def test_upload_falls_back_when_long_lines_are_cut(self):
        link = self.link(FakeRadio(truncate=True))
        blob = build_test_blob(SITES * 3)
        self.assertEqual(at.upload_stations(link, blob, note=lambda s: None), 32)
        self.assertEqual(self.radio.stn, blob)

    def test_upload_writes_in_order(self):
        # the radio refuses a write past the sectors it has reached (one erase
        # per line); the client's chunks never skip ahead
        link = self.link(FakeRadio(max_line=160))
        blob = build_test_blob(SITES * 100)
        self.assertGreater(len(blob), 2 * 4096)
        at.upload_stations(link, blob, note=lambda s: None)
        self.assertEqual(self.radio.stn, blob)
        link.command('STN BEGIN %d 00000000' % len(blob))
        with self.assertRaises(at.ConsoleError) as cm:
            link.command('STN W 8192 00')
        self.assertEqual(cm.exception.reason, 'STATE')

    def test_upload_real_failure_is_raised(self):
        link = self.link(FakeRadio(max_line=160))
        self.radio.fail_end = True
        with self.assertRaises(at.ConsoleError) as cm:
            at.upload_stations(link, build_test_blob(SITES), note=lambda s: None)
        self.assertEqual(cm.exception.reason, 'CRC')
        self.assertEqual(sum(c.startswith('STN BEGIN') for c in self.radio.commands), 2)

    def test_log_download(self):
        link = self.link()
        out = io.StringIO()
        self.assertEqual(at.download_log(link, out), 2)
        rows = out.getvalue().splitlines()
        self.assertEqual(rows[0], ','.join(at.FIELDS['DEC']))
        self.assertEqual(rows[1:], [EX['log_dump'][4:], EX['dec_lvl'][4:]])
        self.assertIn(EX['sta'], self.stream)           # routed to the stream, not the CSV
        self.assertEqual(self.radio.commands[-1], 'LOG DUMP')
        at.download_log(link, io.StringIO(), last=1)
        self.assertEqual(self.radio.commands[-1], 'LOG DUMP 1')

    def test_log_download_follows_the_hdr(self):
        link = self.link()
        self.radio.hdr[1] += ',snr'
        self.radio.log = [r + ['33'] for r in self.radio.log]
        out = io.StringIO()
        at.download_log(link, out)
        rows = out.getvalue().splitlines()
        self.assertTrue(rows[0].endswith(',payload_bin,snr'))
        self.assertTrue(rows[1].endswith(',33'))


class TestCommandLine(unittest.TestCase):
    """main() end to end, with open_radio handing out a FakeRadio."""

    def run_main(self, argv, radio=None):
        self.radio = radio or FakeRadio()
        saved = at.open_radio
        at.open_radio = lambda port=None: (self.radio, 'FAKE')
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
                rc = at.main(argv)
        finally:
            at.open_radio = saved
        return rc, out.getvalue()

    def test_screenshot(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 's.png')
            rc, _ = self.run_main(['screenshot', path, '--scale', '2'])
            self.assertEqual(rc, 0)
            with open(path, 'rb') as fh:
                w, h, _, _, rows = decode_png(fh.read())
        self.assertEqual((w, h), (256, 128))
        self.assertEqual((rows[0][100], rows[2][100], rows[127][255]), (0, 1, 0))
        self.assertTrue(self.radio.commands[0].startswith('TIME '))   # clock set on connect

    def test_set_joins_value_words(self):
        rc, out = self.run_main(['set', 'confirm', '2', 'COPIES'])
        self.assertEqual(rc, 0)
        self.assertEqual(self.radio.settings['CONFIRM'], '2 COPIES')
        self.assertIn('SET CONFIRM 2_COPIES', self.radio.commands)
        self.assertIn('2 COPIES', out)

    def test_no_time(self):
        rc, _ = self.run_main(['--no-time', 'get'])
        self.assertEqual(rc, 0)
        self.assertEqual(self.radio.commands, ['GET'])

    def test_err_exits_1(self):
        rc, out = self.run_main(['set', 'SNR_REQ', '30'])
        self.assertEqual(rc, 1)
        self.assertIn('ERR,RANGE', out)

    def test_log_download_file(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'log.csv')
            rc, out = self.run_main(['log-download', '--csv', path])
            with open(path) as fh:
                text = fh.read()
        self.assertEqual(rc, 0)
        self.assertIn('wrote 2 records', out)
        self.assertEqual(text.splitlines()[1], EX['log_dump'][4:])

    def test_stations_upload_blob_and_dry_run(self):
        blob = build_test_blob(SITES)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 's.bin')
            with open(path, 'wb') as fh:
                fh.write(blob)
            rc, out = self.run_main(['stations-upload', '--blob', path, '--dry-run'])
            self.assertEqual(rc, 0)
            self.assertIn('dry run: %d console lines' % (2 + math.ceil(len(blob) / 64)), out)
            self.assertEqual(self.radio.commands, [])          # never connected
            rc, out = self.run_main(['stations-upload', '--blob', path])
        self.assertEqual(rc, 0)
        self.assertEqual(self.radio.stn, blob)
        self.assertIn('STN,SPI MegaNet:test123', out)

    def test_foreign_region_names_the_way_out(self):
        radio = FakeRadio()
        radio.foreign = True
        rc, out = self.run_main(['stations-clear', '--yes'], radio)
        self.assertEqual(rc, 1)
        self.assertIn('ERR,FOREIGN', out)
        self.assertIn('STN FORMAT FORCE', out)

    def test_clear_needs_yes(self):
        with unittest.mock.patch('builtins.input', return_value='no'):
            rc, out = self.run_main(['log-clear'])
        self.assertEqual(rc, 1)
        self.assertEqual(self.radio.commands, [])
        rc, _ = self.run_main(['log-clear', '--yes'])
        self.assertEqual(rc, 0)
        self.assertIn('LOG CLEAR YES', self.radio.commands)


if __name__ == '__main__':
    unittest.main()
