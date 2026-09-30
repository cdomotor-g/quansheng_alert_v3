#!/usr/bin/env python3
"""Fabricate X1 logs with known answers and check sweep_judge.py's verdicts.

The judge decides what the next flash is, so it has to be right on logs whose
answer is known before the radio produces one. Each scenario writes a log the
way radio.py would (ISO time, then the line as the firmware sends it), made
from a model of the slicer: ALERT bits at 300 baud read at k = TONE2/300
samples per bit, with a random phase, the sync pattern the firmware prepends,
and the discriminator noise the window carries after squelch-lost.

  works      A1 decodes station 702 in 4 bursts -> WORKS, A2 slices noise ->
             NOISE-BITS, D1 delivers nothing -> NO-BITS; exit 0
  structure  A8 (k = 4.33, bytes stored LSB-first) has clean runs that never
             frame -> BITS-NO-DECODE in lsb order; alertmon-style stamps and
             junk lines on the way; exit 1
  dead       every Phase 1 and Phase 3 arrangement twice, all noise -> exit 2
  short      the same with one row a burst short -> exit 3, naming the row
  edge       A5 (FFFFFFF0 edge sync, k = 4) decodes 702 in 4 bursts stored
             LSB-first: found only at the firmware's 7-bit gate and with the
             prepended sync left as programmed -> WORKS in lsb order; exit 0
  adc        AUD pass, confirmed bursts, 702 three times on ADC -> exit 0,
             with a non-qualifying opening (P BURST, no C line) before each
             that the confirmation must not count
  stream     the dead scenario, but S2 never streams and neither it nor B2
             ever syncs on noise over 10 expected intervals -> still exit 2
  framing    radio.py's frames parsed the way uart.c parses them, and its
             demultiplexer on a reply packet mixed into the line stream

    python tools/alert/test_judge.py
"""
import contextlib
import io
import json
import os
import random
import sys
import tempfile
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sweep_judge as sj  # noqa: E402
import radio  # noqa: E402

TABLE = sj.load_table()
STATION = 702               # MURTS HILL MO rain gauge
assert STATION in TABLE

L_ROWS = {  # tag: (idx, r58, t2, r70, t1, sync, 5c, policy)
    'A1': (0, '3FC3', 1200, '80E0', 2100, '0000FFFF', 'AA30', 'S'),
    'A2': (3, '3FC3', 1200, '00E0', 0, '0000FFFF', '5625', 'S'),
    'A8': (8, '3FC3', 1300, '80E0', 2100, '0000FFFF', '5625', 'S'),
    'D1': (21, '03C1', 300, '00E0', 0, '0000FFFF', '5625', 'S'),
}
PHASE1 = dict(zip(sj.PHASE1_TAGS, range(len(sj.PHASE1_TAGS))))
TONE2 = {'A1': 1200, 'A3': 1800, 'B2': 1042, 'A2': 1200, 'A4': 1500, 'A5': 1200, 'B1': 521,
         'A7': 1400, 'A8': 1300, 'C1': 1200, 'A6': 1800, 'A9': 1200, 'B4': 1200, 'A10': 1200,
         'A11': 1200, 'B3': 525, 'C2': 2400, 'C3': 1050, 'B5': 1042, 'S1': 1200, 'S2': 1042,
         'D1': 300, 'D2': 1200}


# ---------------------------------------------------------------------------
# signal model

def eif_words(a, d):
    r = sj.am.crc6((a << 11) | d, 24)
    w3 = (d >> 9) & 0x03
    for j in range(6):
        w3 |= ((r >> (5 - j)) & 1) << (2 + j)
    return [(a & 0x3F) | 0xC0, ((a >> 6) & 0x7F) | ((d & 1) << 7), (d >> 1) & 0xFF, w3]


def neg_bits(words, pre_bits=60, tail_bits=8):
    """Negative logic as the field sends it: idle 0, start 1, data LSB-first, stop 0."""
    bits = [0] * pre_bits
    for w in words:
        bits += [1] + [(w >> i) & 1 for i in range(8)] + [0]
    return bits + [0] * tail_bits


def slicer(bits, t2, rnd, rate_err=0.0):
    """The engine's samples: TONE2 per second against 300 baud (+rate_err)."""
    step = 300.0 * (1 + rate_err) / t2          # bits per sample
    ph = rnd.random()
    out = []
    i = 0
    while True:
        j = int(ph + i * step)
        if j >= len(bits):
            return out
        out.append(bits[j])
        i += 1


def window(core, idle, rnd, skip, noise_after=60):
    """What BurstFinish linearizes: the prepended sync (16 samples at the
    idle level), the stream from the sync on, then noise past squelch-lost."""
    return [idle] * 16 + core[skip:] + [rnd.randint(0, 1) for _ in range(noise_after)]


def store(samples, lsb=False):
    buf = sj.pack(samples)
    return bytes(sj.REV[b] for b in buf) if lsb else buf


# ---------------------------------------------------------------------------
# log writer

class LogWriter:
    def __init__(self, stamps='iso'):
        self.t = datetime(2026, 10, 1, 9, 0, 0)
        self.lines = []
        self.stamps = stamps
        self.burst = 0

    def put(self, line, dt=0.0):
        self.t += timedelta(seconds=dt)
        if self.stamps == 'iso':
            self.lines.append('%s %s' % (self.t.isoformat(timespec='milliseconds'), line))
        else:
            self.lines.append('[%s] %s' % (self.t.strftime('%H:%M:%S.%f')[:-3], line))

    def heartbeats(self, secs):
        for _ in range(int(secs / 0.5)):
            self.put('D I0 S0 F0 G0 X0 B0 R-75 Q0 N0 m0 y0 v0 l0 f-76 p-20 V0 C0', 0.5)

    def arr(self, tag, idx=None, t2=None, ph=1, v=0):
        row = L_ROWS.get(tag)
        if row is None:
            row = (PHASE1.get(tag, 0), '3FC3', TONE2.get(tag, 1200), '00E0', 0, '0000FFFF',
                   '5625', 'T' if tag in sj.STREAM_TAGS else 'S')
        i, r58, tt2, r70, t1, sy, r5c, pol = row
        self.put('L %s %s r58=%s t2=%d r70=%s t1=%d sy=%s s4=0 5c=%s pol=%s ph=%d v=%02X'
                 % (i if idx is None else idx, tag, r58, t2 or tt2, r70, t1, sy, r5c, pol, ph, v))
        return i if idx is None else idx

    def burst_lines(self, idx, t2, samples, stored, src='open', dt=40, pn='P', fw=()):
        self.burst += 1
        b = self.burst
        n = len(samples)
        k = t2 / 300.0
        st = sj.window_stats(samples, k) if n > 1 else {'tr': 0, 'rk': 0, 'mx': 0, 'lead': 0,
                                                        'base': 2.0 ** (1 - sj.m_for(k))}
        win = int(round(n * 1000.0 / t2)) if n else 450
        self.put('C %s b=%d pk=-18 fl=-76 win=%d src=%s dt=%s pn=%s w=%d n=%d k=%d tr=%d rk=%d '
                 'base=%d mx=%d lead=%d dec=%d tab=%d'
                 % (idx, b, win, src, dt, pn, (n + 15) // 16, n, round(100 * k),
                    1000 * st['tr'], 1000 * st['rk'], 1000 * st['base'], st['mx'], st['lead'],
                    len(fw), len(fw)), 0.6)
        for off in range(0, len(stored), 64):
            self.put('H %s %d %d %s' % (idx, b, off, stored[off:off + 64].hex().upper()))
        for rid, val, pol, inv in fw:
            self.put('X %s %d id=%d v=%d fmt=EIF pol=%s inv=%d pos=100 known=%d'
                     % (idx, b, rid, val, pol, inv, 1 if rid in TABLE else 0))
        return b

    def write(self, path):
        with open(path, 'w', newline='\r\n') as f:
            f.write('\n'.join(self.lines) + '\n')


def judge(paths, extra=()):
    """Run the judge as the command line would; returns (exit code, results, text)."""
    out = io.StringIO()
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'r.json')
        with contextlib.redirect_stdout(out):
            code = sj.main(list(paths) + ['--jobs', '1', '--json', js] + list(extra))
        with open(js) as f:
            res = json.load(f)
    return code, res, out.getvalue()


def by_tag(res):
    return {r['tag']: r for r in res['arrangements']}


# ---------------------------------------------------------------------------
# scenarios

def scenario_works(td, rnd):
    w = LogWriter()
    w.put('B ver=0123abcd rst=por n=0 bl=22CDCECB')
    w.put('AUD pin=none pa=0 floor_on=-76 floor_off=-76')
    # A1: EIF frames from 702, slicer idle level 1 (engine synced P on FFFF)
    idx = w.arr('A1')
    for i in range(4):
        w.heartbeats(10)
        core = [x ^ 1 for x in slicer(neg_bits(eif_words(STATION, 100 + i)), 1200, rnd,
                                        rate_err=rnd.uniform(-0.015, 0.015))]
        s = window(core, 1, rnd, skip=rnd.randint(20, 50))
        fw = [(STATION, 100 + i, 'N', 1)] if i < 2 else ()
        w.burst_lines(idx, 1200, s, store(s), dt=rnd.randint(20, 90), pn='P', fw=fw)
        w.put('Y %s ms=%d pn=P sq=1 rssi=-20' % (idx, rnd.randint(100, 900)))
    # A2: synced, but slices noise; four noise syncs in ~110 s is the chance rate
    idx = w.arr('A2')
    for i in range(3):
        w.heartbeats(18)
        w.put('Y %s ms=1000 pn=N sq=0 rssi=-75' % idx)
        s = [1] * 16 + [rnd.randint(0, 1) for _ in range(560)]
        w.burst_lines(idx, 1200, s, store(s), dt=rnd.randint(0, 140), pn='P')
    w.put('Y %s ms=1000 pn=N sq=0 rssi=-75' % idx, 10)
    w.heartbeats(10)
    # D1: the old configuration, no words during a burst
    idx = w.arr('D1')
    for i in range(3):
        w.heartbeats(12)
        w.burst_lines(idx, 300, [], b'', src='none', dt='NA', pn='-')
    w.put('Z 0 bq=4 bb=4 bs=4 bt=4 dx=4 dt=4 rep=4 ns=0')
    w.put('ALERT,702,103,MURTS HILL MO,-18,RAIN')
    w.put('K ok')
    p = os.path.join(td, 'works.log')
    w.write(p)
    return p


def scenario_structure(td, rnd):
    w = LogWriter(stamps='tod')
    w.put('B ver=0123abcd rst=sw n=0 bl=22CDCECB')
    w.put('# radio.py comment line')
    idx = w.arr('A8')
    # runs of 1, 2, 3, 2 bits: the transition density of a frame, but no stop bit
    # ever lands where four consecutive words need one, under any framing or sense
    pattern = [0] * 60 + [0, 1, 1, 0, 0, 0, 1, 1] * 10 + [0] * 6
    for i in range(3):
        w.heartbeats(8)
        core = slicer(pattern, 1300, rnd)
        s = window(core, 0, rnd, skip=rnd.randint(20, 50))
        w.burst_lines(idx, 1300, s, store(s, lsb=True), dt=60, pn='N')
    w.put('C 8 b=')                                           # torn line
    w.put('\x7f\x10garbage without a tag')
    w.put('H 8 999 0 ZZ')                                     # not hex: ignored
    idx = w.arr('A2')
    for i in range(2):
        w.heartbeats(8)
        s = [rnd.randint(0, 1) for _ in range(500)]
        w.burst_lines(idx, 1200, s, store(s), dt=30, pn='P')
    p = os.path.join(td, 'structure.log')
    w.write(p)
    return p


def scenario_dead(td, rnd, drop=None, quiet_stream=None):
    """quiet_stream: an S row whose engine never syncs on noise, so it never
    streams: every burst finds it searching (src=open), and it and its base
    row sit armed for longer than 10 expected noise-sync intervals."""
    w = LogWriter()
    w.put('B ver=0123abcd rst=por n=0 bl=22CDCECB')
    # alert.c's index space: Phase 3 is 40 generated REG_58 codes at 64..103
    # and 8 variants of A1 (v 0x60, 0x71..0x77), which must not merge with A1
    rows = [(tag, PHASE1[tag], TONE2[tag], 1, 0) for tag in sj.PHASE1_TAGS]
    rows += [('G%d' % i, 64 + i, 1200, 3, 0) for i in range(40)]
    rows += [('A1', 0, 1200, 3, v) for v in [0x60] + list(range(0x71, 0x78))]
    assert len(rows) - len(sj.PHASE1_TAGS) == sj.PHASE3_ROWS
    for npass in range(2):
        for tag, idx, t2, ph, v in rows:
            if drop == tag and npass == 1:
                continue
            w.arr(tag, idx=idx, t2=t2, ph=ph, v=v)
            quiet = quiet_stream is not None and v == 0 and \
                tag in (quiet_stream, sj.STREAM_BASE[quiet_stream])
            w.heartbeats(170 if quiet else 2)
            stream = tag in sj.STREAM_TAGS and tag != quiet_stream
            s = [rnd.randint(0, 1) for _ in range(160)]
            w.burst_lines(idx, t2, s, store(s), src='pre' if stream else 'open',
                          dt=-2000 if stream else 50, pn='P')
    p = os.path.join(td, 'dead%s%s.log' % ('-' + drop if drop else '',
                                          '-' + quiet_stream if quiet_stream else ''))
    w.write(p)
    return p


def scenario_edge(td, rnd):
    """A5: the engine synced on FFFFFFF0, the idle run and the start bit, so
    the window holds the 28 idle samples the sync consumed (7 bits at k = 4,
    under the usual 12-bit gate) and no preamble. The bytes after the sync are
    stored LSB-first; the sync itself goes in as programmed."""
    w = LogWriter()
    w.put('B ver=0123abcd rst=por n=0 bl=22CDCECB')
    w.put('L 5 A5 r58=3FC3 t2=1200 r70=80E0 t1=2100 sy=FFFFFFF0 s4=1 5c=5625 pol=S ph=1 v=00')
    prefix = [1] * 28 + [0] * 4
    for i in range(4):
        w.heartbeats(10)
        core = [x ^ 1 for x in slicer(neg_bits(eif_words(STATION, 200 + i)), 1200, rnd,
                                        rate_err=rnd.uniform(-0.01, 0.01))]
        e = core.index(0)                           # the first start edge after the preamble
        while e + 4 < len(core) and core[e + 4] == 0:
            e += 1                                  # a 5-sample start bit: the sync ends on its last
        body = core[e + 4:] + [rnd.randint(0, 1) for _ in range(60)]
        body += [0] * (-len(body) % 8)
        s = prefix + body
        w.burst_lines(5, 1200, s, sj.pack(prefix) + store(body, lsb=True),
                      dt=rnd.randint(20, 90), pn='P')
    p = os.path.join(td, 'edge.log')
    w.write(p)
    return p


def scenario_adc(td, rnd):
    w = LogWriter()
    w.put('B ver=0123abcd rst=por n=0 bl=22CDCECB')
    w.put('PT PA4 u1 d0 PB1 u1 d1')
    w.put('P ch=4 b=1 pa=0 src=M mean=2048 pp=12 rms=30 g13=80 g17=82 g21=79')
    # the idle level is the louder tone bin: 10 dB over 121, not over 120
    w.put('P ch=4 b=1 pa=0 src=F mean=2048 pp=40 rms=60 g13=120 g17=118 g21=121')
    w.put('AUD pin=PA4B pa=0 floor_on=-75 floor_off=-76')
    idx = w.arr('A1')
    for i in range(3):
        w.heartbeats(15)
        # a noise flicker: BurstStop reports it, BurstFinish gates it (no C line)
        w.put('P ch=4 b=1 pa=0 src=BURST mean=2048 pp=60 rms=70 g13=125 g17=120 g21=126')
        w.put('P ch=9 b=1 pa=0 src=BURST mean=780 pp=10 rms=5 g13=40 g17=41 g21=40')
        w.put('Y %s ms=900 pn=P sq=1 rssi=-60' % idx)
        w.heartbeats(5)
        w.put('P ch=4 b=1 pa=0 src=BURST mean=2048 pp=900 rms=400 g13=%d g17=150 g21=%d'
              % (260 + i, 300 + i))
        w.put('P ch=9 b=1 pa=0 src=BURST mean=780 pp=10 rms=5 g13=40 g17=41 g21=40')
        s = [1] * 16 + [rnd.randint(0, 1) for _ in range(560)]
        b = w.burst_lines(idx, 1200, s, store(s), dt=40, pn='P')
        w.put('X ADC %d id=%d v=%d fmt=EIF pol=N inv=0 pos=40 known=1' % (b, STATION, i))
    w.put('G ADOPT ADC v=0 id=702 rep=3')
    p = os.path.join(td, 'adc.log')
    w.write(p)
    return p


# ---------------------------------------------------------------------------
# radio.py framing, checked against a re-implementation of uart.c's parser

OBF = bytes([0x16, 0x6C, 0x14, 0xE6, 0x2E, 0x91, 0x0D, 0x40,
             0x21, 0x35, 0xD5, 0x40, 0x13, 0x03, 0xE9, 0x80])   # uart.c Obfuscation[]


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def uart_parse(pkt):
    """UART_IsCommandAvailable + UART_HandleCommand's view: (ID, body) or an error."""
    assert pkt[0] == 0xAB and pkt[1] == 0xCD, 'no 0xABCD'
    size = pkt[2] | (pkt[3] << 8)
    assert len(pkt) == size + 8, 'length'
    assert pkt[size + 6] == 0xDC and pkt[size + 7] == 0xBA, 'no 0xDCBA'
    buf = bytearray(pkt[4:size + 6])
    for i in range(size + 2):
        buf[i] ^= OBF[i % 16]
    assert crc16_xmodem(buf[:size]) == buf[size] | (buf[size + 1] << 8), 'CRC'
    msg_id = buf[0] | (buf[1] << 8)
    body_len = buf[2] | (buf[3] << 8)
    assert 4 + body_len <= size, 'body length'
    return msg_id, bytes(buf[4:4 + body_len])


def fw_reply(msg_id, data):
    """SendReply_VCP: obfuscated message, footer padding as the firmware writes it."""
    m = bytearray([msg_id & 0xFF, msg_id >> 8, len(data) & 0xFF, len(data) >> 8]) + data
    size = len(m)
    for i in range(size):
        m[i] ^= OBF[i % 16]
    return (bytes([0xAB, 0xCD, size & 0xFF, size >> 8]) + bytes(m) +
            bytes([OBF[size % 16] ^ 0xFF, OBF[(size + 1) % 16] ^ 0xFF, 0xDC, 0xBA]))


def check_framing(check):
    cases = [
        (radio.CMD_SWEEP_CTL, radio.body_sweep_ctl(radio.SWEEP_OPS['enter'])),
        (radio.CMD_SWEEP_CTL, radio.body_sweep_ctl(radio.SWEEP_OPS['goto'], 19)),
        (radio.CMD_ARR_SET, radio.body_arr_set(32, {'r58': 0x3FC3, 't2': 1300})),
        (radio.CMD_ARR_SET, radio.body_arr_set(39, {'tag': 'Q7', 'r59': 0x0408, 'flags': 1})),
        (radio.CMD_POKE_LIST, radio.body_poke_list([(0x59, 0xFFF0, 0x0006), (0x5C, 0, 0xAA30)])),
        (radio.CMD_POKE_LIST, radio.body_poke_list([])),
        (radio.CMD_BK_READ, bytes([0x58])),
        (radio.CMD_BK_WRITE, bytes([0x72, 0x65, 0x30])),
        (radio.CMD_REBOOT, b''),
    ]
    for msg_id, body in cases:
        try:
            got = uart_parse(radio.frame(msg_id, body))
            ok = got == (msg_id, body)
        except AssertionError as e:
            ok, got = False, str(e)
        check('frame %04X body %s parses as uart.c would' % (msg_id, body.hex() or '-'), ok, got)
    check('SWEEP_CTL enter is op 7', radio.body_sweep_ctl(radio.SWEEP_OPS['enter'])[0] == 7)

    # ARR_SET read back at the offsets ALERT_HostArrSet uses (alert.h)
    b = radio.body_arr_set(33, {'tag': 'X9', 'r58': 0x03C5, 'r70': 0x80E0, 't1': 2100,
                                't2': 1042, 'r5c': 0xAA30, 'r59': 0x0408, 'r5a': 0xFFFF,
                                'r5b': 0xFFF0, 'flags': 1})

    def le(o):
        return b[o] | (b[o + 1] << 8)
    check('ARR_SET is 21 bytes at the offsets alert.h gives',
          len(b) == 21 and b[0] == 33 and b[1:4] == b'X9' + bytes(1) and le(4) == 0x03C5 and
          le(6) == 0x80E0 and le(8) == 2100 and le(10) == 1042 and le(12) == 0xAA30 and
          le(14) == 0x0408 and le(16) == 0xFFFF and le(18) == 0xFFF0 and b[20] == 1, b.hex())

    stream = (b'D I1 S0 F0\r\nL 0 A1 r58=3FC3\r\n' + fw_reply(0x0601, bytes([0x58, 0xC3, 0x3F])) +
              b'K arr ok\r\nD I2')
    rnd = random.Random(5)
    for trial in range(20):
        dm = radio.Demux()
        items = []
        i = 0
        while i < len(stream):
            n = rnd.randint(1, 9)
            items += dm.feed(stream[i:i + n])
            i += n
        want = [('line', 'D I1 S0 F0'), ('line', 'L 0 A1 r58=3FC3'),
                ('pkt', 0x0601, bytes([0x58, 0xC3, 0x3F])), ('line', 'K arr ok')]
        if items != want:
            check('demux trial %d' % trial, False, items)
            break
    else:
        check('demux splits lines and a 0x0601 reply fed in random chunks', True)
    check('relay text round-trips a packet',
          radio.item_from_text(radio.item_text(('pkt', 0x0601, b'\x58\xc3\x3f'))) ==
          ('pkt', 0x0601, b'\x58\xc3\x3f'))


class FakeSerial:
    """Stands in for the radio: acks every command the way dfu.c does."""

    def __init__(self, ok=1):
        self.rx = bytearray(b'D I1 S0\r\n')
        self.tx = bytearray()
        self.ok = ok
        self.dtr = True

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        d = bytes(self.rx[:n])
        del self.rx[:n]
        return d

    def write(self, b):
        self.tx += b
        msg_id, _ = uart_parse(bytes(b))
        self.rx += b'D I2 S0\r\nK %04X ok=%d\r\n' % (msg_id, self.ok)

    def close(self):
        pass


def no_real_port(explicit=None):
    # a radio may be plugged in and live: if the relay path ever broke, the
    # client would fall back to opening it, so the test makes that impossible
    raise AssertionError('test tried to open a real serial port')


def check_relay(check, td):
    """A command from a second radio.py goes through a running logger."""
    import argparse
    import socket
    import threading
    radio.find_port = radio.open_port = no_real_port
    for ok in (1, 0):
        log_args = argparse.Namespace(out=os.path.join(td, 'relay%d.log' % ok), port=None,
                                      no_d=True, minutes=0)
        with contextlib.redirect_stdout(io.StringIO()):
            lg = radio.Logger(log_args)
            lg.ser, lg.dev = FakeSerial(ok), 'FAKE'
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.bind(('127.0.0.1', 0))
            srv.listen(2)
            srv.setblocking(False)
            saved = radio.RELAY_ADDR
            radio.RELAY_ADDR = srv.getsockname()
            rc = []
            cli = argparse.Namespace(direct=False, port=None, verbose=False, op='goto', a=19, b=0)
            th = threading.Thread(target=lambda: rc.append(radio.cmd_sweep_ctl(cli)))
            th.start()
            import time
            end = time.time() + 5
            while th.is_alive() and time.time() < end:
                lg.poll_serial()
                lg.poll_relay(srv)
                lg.expire_clients()
                time.sleep(0.01)
            th.join(1)
            radio.RELAY_ADDR = saved
            srv.close()
            sent = bytes(lg.ser.tx)
            lg.fh.close()
        with open(log_args.out) as f:
            logged = f.read()
        check('relay (K ok=%d): exit %d, frame reached the port, both ends logged'
              % (ok, 1 - ok),
              rc == [1 - ok] and uart_parse(sent) == (radio.CMD_SWEEP_CTL, bytes([2, 19, 0])) and
              '#tx ' in logged and 'K 0A02 ok=%d' % ok in logged, (rc, sent.hex(), logged))


def check_chance_logic(check):
    """The early stops, with a stand-in for the shuffling workers."""
    class Stub:
        def __init__(self, hits_per_20):
            self.h = hits_per_20

        def map(self, tasks):
            return [self.h * t[4] // 20 for t in tasks]

    win = [(bytes(20), 160)] * 3
    r = sj.chance_control(win, 3, 3, 1200, TABLE, Stub(0), 200, 1, True)
    check('chance: no shuffled hits passes after the first 20 shuffles',
          r['pass'] and r['shuffles'] == 20, r)
    r = sj.chance_control(win, 3, 3, 1200, TABLE, Stub(5), 200, 1, True)
    check('chance: a 25% shuffled hit rate fails (E = 0.75 > 3/20)',
          not r['pass'] and abs(r['E'] - 0.75) < 1e-9 and r['shuffles'] < 200, r)
    r = sj.chance_control(win, 6, 3, 1200, TABLE, Stub(0), 200, 1, True)
    check('chance: bursts without H scale E up, early pass needs more shuffles',
          r['pass'] and r['shuffles'] == 40, r)


# ---------------------------------------------------------------------------

def main():
    fails = []

    def check(label, cond, detail=None):
        print('%-66s %s' % (label, 'PASS' if cond else 'FAIL'))
        if not cond:
            fails.append(label)
            if detail is not None:
                print('    got: %r' % (detail,))
        return cond

    rnd = random.Random(20260930)
    print('edge slicer: %s' % ('scan_samples.py' if sj.edge_slicer() else 'missing (decimation only)'))
    with tempfile.TemporaryDirectory() as td:
        # works
        code, res, text = judge([scenario_works(td, rnd)])
        a = by_tag(res)
        print(text)
        check('works: exit 0', code == 0, code)
        check('works: A1 WORKS', a['A1']['verdict'] == 'WORKS', a['A1'])
        check('works: A1 REPEAT 4 on 702, NEG framing, inverted sense',
              a['A1']['repeat'] == 4 and a['A1']['repeat_path'] == '702 N inv1',
              (a['A1']['repeat'], a['A1']['repeat_path']))
        # at integer k a byte holds whole bits, so STRUCT alone may not tell the
        # orders apart; the decode does
        check('works: A1 STRUCT 1.00 (msb among the orders), SYNC 1.00 P',
              a['A1']['struct'] == 1.0 and 'msb' in a['A1']['struct_order'] and
              a['A1']['sync'] == 1.0 and a['A1']['sync_pn'] == 'P',
              (a['A1']['struct'], a['A1']['struct_order'], a['A1']['sync']))
        check('works: A1 chance control passes', a['A1']['chance']['pass'], a['A1']['chance'])
        check('works: A1 decoded in msb order only',
              set(v.split('/')[1] for v in a['A1']['via']) <= {'msb', '-'}, a['A1']['via'])
        check('works: A2 NOISE-BITS', a['A2']['verdict'] == 'NOISE-BITS', a['A2'])
        check('works: A2 noise-sync ratio near 1 (chance rate)',
              a['A2']['nsr'] is not None and 0.6 <= a['A2']['nsr'] <= 1.6, a['A2']['nsr'])
        check('works: D1 NO-BITS', a['D1']['verdict'] == 'NO-BITS', a['D1'])

        # the same log with the edge slicer off still finds it by decimation (k = 4)
        code2, res2, _ = judge([scenario_works(td, random.Random(20260930))], ['--no-edge'])
        check('works, decimation only: A1 WORKS', by_tag(res2)['A1']['verdict'] == 'WORKS',
              by_tag(res2)['A1'])

        # structure without a decode
        code, res, text = judge([scenario_structure(td, rnd)])
        a = by_tag(res)
        check('structure: exit 1', code == 1, code)
        check('structure: A8 BITS-NO-DECODE, found in lsb order',
              a['A8']['verdict'] == 'BITS-NO-DECODE' and a['A8']['struct_order'] == 'lsb',
              (a['A8']['verdict'], a['A8']['struct'], a['A8']['struct_order']))
        check('structure: A2 still NOISE-BITS', a['A2']['verdict'] == 'NOISE-BITS',
              a['A2']['verdict'])

        # full coverage, nothing structured
        code, res, text = judge([scenario_dead(td, rnd)])
        check('dead: exit 2 (FSK route dead)', code == 2, (code, res['overall'],
                                                            res['still_needed']))
        code, res, text = judge([scenario_dead(td, rnd, drop='B3')])
        check('short: exit 3 and names B3', code == 3 and any('B3' in n for n in
                                                               res['still_needed']),
              (code, res['still_needed']))

        # S2 cannot stream: covered all the same, but only once it has had the time
        code, res, text = judge([scenario_dead(td, rnd, quiet_stream='S2')])
        check('stream: S2 never syncs on noise -> inapplicable, exit 2',
              code == 2 and 'stream policy inapplicable' in text and 'S2' in text,
              (code, res['still_needed']))

        # edge sync, LSB-first bytes
        code, res, text = judge([scenario_edge(td, rnd)])
        a = by_tag(res)
        check('edge: exit 0, A5 WORKS at the 7-bit edge-sync gate',
              code == 0 and a['A5']['verdict'] == 'WORKS' and a['A5']['gate'] == 7,
              (code, a['A5']['verdict'], a['A5']['gate'], a['A5']['repeat']))
        check('edge: A5 decoded in lsb order, STRUCT in lsb order',
              set(v.split('/')[1] for v in a['A5']['via']) == {'lsb'} and
              a['A5']['struct'] == 1.0 and 'lsb' in a['A5']['struct_order'],
              (a['A5']['via'], a['A5']['struct'], a['A5']['struct_order']))

        # ADC
        code, res, text = judge([scenario_adc(td, rnd)])
        check('adc: exit 0, ADC route works', code == 0 and res['adc']['works'], res['adc'])
        check('adc: confirmation 3/3, the gated flickers not counted',
              res['adc']['confirm'] == '3/3' and res['adc']['confirmed'], res['adc'])

        check_relay(check, td)

    check_framing(check)
    check_chance_logic(check)

    if fails:
        print('\nFAILED: %d check(s)' % len(fails))
        return 1
    print('\nall judge checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
