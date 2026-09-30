#!/usr/bin/env python3
"""Judge an ALERT-X1 sweep: which arrangement, if any, recovers ALERT frames.

Reads one or more logs of the X1 USB stream (radio.py log, alertmon.py output,
or a bare capture) and applies the criteria of X1_PLAN.md s9 to every
arrangement, from its qualifying bursts:

  BITS    bursts delivering n >= 0.5 x win x TONE2/1000 samples
  SYNC    of the bursts with the engine searching at the open (src=open or
          none), plus those streaming from a sync at most 300 ms before it (on
          the carrier), the fraction synced 0..150 ms after the open or in those
          300 ms before it, all with one pn
  STRUCT  (k >= 3 only) the H window, read under both in-byte bit orders:
          runs of >= m = max(2, round(0.75k)) samples are >= 80% of runs,
          transitions/sample <= 0.6/k, and the first run is >= 20k samples
  DECODE  bursts with a table-station decode: the firmware's X lines, and the
          host's own decode of the H window with the edge slicer (the
          scan_samples.py port of ALERT_ScanSamples) and phase decimation for
          integer k, each over NEG/STD framing x both data senses x both bit
          orders, under the same 12-idle-bit preamble gate as the firmware
  REPEAT  most separate bursts in which one table station decoded under the
          same (framing, sense) path
  chance  each capture's run lengths shuffled up to 200 times and decoded by
          the same procedure: the expected number of chance table-hit bursts
          E. DECODE must be >= 20 x E. Run lengths carry all the oversampling
          structure, so this is the control that structure alone cannot pass.
  NSR     noise-sync ratio: sq=0 syncs per armed second over 2 x TONE2 / 2^16
          (2^32 for a 4-byte sync); above 3 the detector tolerates errors or the
          demodulated noise is correlated

Verdicts, and the exit code of the whole run:

  WORKS           REPEAT >= 3 and the chance control passes         exit 0
  LIKELY          REPEAT = 2, or >= 2 table stations with STRUCT
                  in >= 2/3 of bursts                                exit 0
  BITS-NO-DECODE  STRUCT in >= 2/3 of bursts, DECODE = 0             exit 1
  NOISE-BITS      BITS >= 0.5, rk within 0.1 of the noise baseline
  NO-BITS         BITS < 0.2
  FSK route dead  every Phase 1 and Phase 3 arrangement has >= 2
                  bursts, S1/S2 >= 2 stream-coincident bursts each,
                  and STRUCT never passed                            exit 2
  not enough data anything short of that                            exit 3
  ADC route works AUD passed, bursts confirmed, REPEAT >= 3 on ADC  exit 0

Two per-arrangement verdicts the plan does not name: CHANCE (REPEAT >= 3 but
shuffled run lengths decode as often) and MIXED (bits, but none of the above).

STRUCT deviates from the letter of s9 in one way: it is measured from the
preamble run (the longest run starting in the window's first 250 ms) through
one frame, not over the whole window. The window runs 40 ms past squelch-lost
and may start 100 ms before the open, and the discriminator noise there adds
more short runs than a whole burst has runs, so a perfect capture would fail.
Without H lines the firmware's own C-line figures are used, as they are.

A burst that only the firmware decoded still counts: the H lines may have
been lost. BITS-NO-DECODE means the bits are there and the bit order, the
rate or the decoder is wrong; take its bursts to capture_stats.py.

    python tools/alert/sweep_judge.py tools/alert/logs/x1-*.log
    python tools/alert/sweep_judge.py --json result.json --jobs 8 a.log b.log
"""
import argparse
import json
import math
import os
import random
import re
import sys
import zlib
from collections import Counter, OrderedDict, defaultdict
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import alertmon as am  # noqa: E402  (the 1x scanner, with the preamble gate)

STATIONS_H = os.path.join(HERE, '..', '..', 'App', 'app', 'alert_stations_gen.h')

# X1_PLAN.md s9, exactly
BITS_FILL = 0.5             # n >= 0.5 x win x TONE2/1000
SYNC_LATE_MS = 150          # 0 <= dt <= 150 after the open ...
SYNC_EARLY_MS = -300        # ... or -300 <= dt < 0, a sync on the carrier
STRUCT_RK = 0.80
STRUCT_TR_K = 0.6           # transitions/sample <= 0.6 / k
STRUCT_LEAD_K = 20          # lead >= 20 k samples
STRUCT_MIN_K = 3.0
LEAD_SEARCH_MS = 250        # the preamble run starts this early in the window at the latest
FRAME_BITS = 44             # one 40-bit frame and a little slack after the preamble
MIN_IDLE_BITS = 12          # the firmware's preamble gate
MAX_GAP_BITS = 20
CHANCE_SHUFFLES = 200
CHANCE_FACTOR = 20
NSR_FLAG = 3.0
NOISE_RK_MARGIN = 0.1
FEW_BITS = 0.2
STRUCT_MAJORITY = 2.0 / 3.0
REPEAT_WORKS = 3
MIN_BURSTS_DEAD = 2         # per arrangement, for "FSK route dead"

# Phase 1 rows (X1_PLAN.md s3.2), by tag; S1/S2 are judged on stream-coincident bursts
PHASE1_TAGS = ('A1', 'A3', 'B2', 'A2', 'A4', 'A5', 'B1', 'A7', 'A8', 'C1', 'A6', 'A9',
               'B4', 'A10', 'A11', 'B3', 'C2', 'C3', 'B5', 'S1', 'S2', 'D1', 'D2')
STREAM_TAGS = ('S1', 'S2')
PHASE3_ROWS = 48            # 40 undocumented REG_58 codes + 8 REG_59 variants on A1

ORDERS = ('msb', 'lsb')     # sample order inside each stored byte
REV = bytes(int('{:08b}'.format(i)[::-1], 2) for i in range(256))
FMT_NAMES = {1: 'ABF', 2: 'EIF', 3: 'A2C'}


# ---------------------------------------------------------------------------
# station table: the same generated header the firmware compiles

def load_table(path=STATIONS_H):
    ids = set()
    with open(path, encoding='utf-8') as f:
        for base, kinds in re.findall(r'\{\s*(\d+),\s*0x([0-9a-fA-F]+),\s*\d+\s*\}', f.read()):
            base, kinds = int(base), int(kinds, 16)
            for off in range(5):
                if (kinds >> (3 * off)) & 7:
                    ids.add(base + off)
    return frozenset(ids)


# ---------------------------------------------------------------------------
# the edge slicer is stream A's port; imported lazily so the judge still runs
# (on phase decimation alone) if that module is missing or has changed shape

_SS = None


def edge_slicer():
    global _SS
    if _SS is None:
        try:
            import scan_samples as ss
            ok = (callable(getattr(ss, 'scan_samples', None)) and
                  hasattr(ss, 'POL_NEGATIVE') and hasattr(ss, 'POL_STANDARD'))
            _SS = ss if ok else False
        except Exception:
            _SS = False
    return _SS or None


def _edge_scan(ss, buf, nsamp, spb_q8, pc, inv):
    """ss.scan_samples, or [] and the edge slicer switched off if its API has
    moved under us: a judge that stops is worse than one that loses a path."""
    global _SS
    try:
        return ss.scan_samples(buf, nsamp, spb_q8, pc, bool(inv), MIN_IDLE_BITS)
    except (TypeError, AttributeError, ValueError, IndexError) as e:
        print('scan_samples.py failed (%r): continuing on phase decimation alone' % e,
              file=sys.stderr)
        _SS = False
        return []


def _reading(r):
    """(fmt, id, value, pos) from a scan_samples reading, whatever its shape."""
    if hasattr(r, 'id'):
        fmt, rid, val, pos = r.format, r.id, r.value, getattr(r, 'bit_pos', 0)
    elif isinstance(r, dict):
        fmt, rid, val, pos = r.get('format'), r['id'], r['value'], r.get('bit_pos', 0)
    else:                                   # alertmon shape: ((fmt, id, value), pos, pol)
        (fmt, rid, val), pos = r[0], r[1]
    return FMT_NAMES.get(fmt, fmt), rid, val, pos


# ---------------------------------------------------------------------------
# sample windows

def unpack(buf, n):
    return [(buf[i >> 3] >> (7 - (i & 7))) & 1 for i in range(n)]


def pack(samples):
    buf = bytearray((len(samples) + 7) // 8)
    for i, b in enumerate(samples):
        if b:
            buf[i >> 3] |= 0x80 >> (i & 7)
    return bytes(buf)


def in_order(data, order):
    """Repack the stored bytes so the sample sequence reads MSB-first."""
    return data if order == 'msb' else bytes(REV[b] for b in data)


def run_lengths(s):
    out = []
    cur = 1
    for i in range(1, len(s)):
        if s[i] == s[i - 1]:
            cur += 1
        else:
            out.append(cur)
            cur = 1
    if s:
        out.append(cur)
    return out


def m_for(k):
    return max(2, int(0.75 * k + 0.5))


def window_stats(s, k):
    """The firmware's C-line statistics over the whole window."""
    m = m_for(k)
    runs = run_lengths(s)
    if len(s) < 2:
        return {'rk': 0.0, 'tr': 0.0, 'lead': len(s), 'mx': len(s), 'm': m,
                'base': 2.0 ** (1 - m)}
    return {'rk': sum(1 for r in runs if r >= m) / len(runs), 'tr': (len(runs) - 1) / (len(s) - 1),
            'lead': runs[0], 'mx': max(runs), 'm': m, 'base': 2.0 ** (1 - m)}


def struct_test(s, k, t2):
    """STRUCT on the burst itself: the preamble run and the frame after it.

    Over the whole window a real burst cannot pass. The window runs 40 ms past
    squelch-lost and, when the engine was already streaming, starts 100 ms
    before the open; the discriminator's noise there adds many short runs,
    while a whole burst (preamble plus one 40-bit frame) is only ~20 runs. So
    the lead is the longest run starting in the first 250 ms (the preamble,
    wherever the window began), and rk and transitions are measured from it
    through one frame. Noise still fails: its longest early run is ~8 samples
    against the 20k required."""
    if k < STRUCT_MIN_K or len(s) < 2:
        return {'ok': False, 'rk': 0.0, 'tr': 0.0, 'lead': 0}
    runs = run_lengths(s)
    horizon = LEAD_SEARCH_MS * t2 // 1000
    best, best_at, best_i, pos = 0, 0, 0, 0
    for i, r in enumerate(runs):
        if pos >= horizon:
            break
        if r > best:
            best, best_at, best_i = r, pos, i
        pos += r
    core = s[best_at:best_at + best + int(FRAME_BITS * k)]
    cruns = run_lengths(core)
    m = m_for(k)
    rk = sum(1 for r in cruns if r >= m) / len(cruns)
    tr = (len(cruns) - 1) / max(1, len(core) - 1)
    ok = rk >= STRUCT_RK and tr <= STRUCT_TR_K / k and best >= STRUCT_LEAD_K * k
    return {'ok': ok, 'rk': rk, 'tr': tr, 'lead': best}


def host_decode(data, nsamp, t2, use_edge=True, stop_on=None):
    """Every reading the host can get from one stored window.

    Edge slicer (when present) over both bit orders x NEG/STD x both senses;
    phase decimation over every phase: for integer k beside the edge slicer,
    and for any k when the edge slicer is missing (fractional steps then).
    stop_on: a set of ids; return at the first reading in it (chance control).
    """
    out = []
    if nsamp < 10 or not t2:
        return out
    k = t2 / 300.0
    ss = edge_slicer() if use_edge else None
    spb_q8 = (256 * t2 + 150) // 300
    integer_k = t2 % 300 == 0
    for order in ORDERS:
        buf = in_order(data, order)
        if ss is not None:
            for pol, pc in (('N', ss.POL_NEGATIVE), ('S', ss.POL_STANDARD)):
                for inv in (0, 1):
                    for r in _edge_scan(ss, buf, nsamp, spb_q8, pc, inv):
                        fmt, rid, val, pos = _reading(r)
                        out.append({'method': 'edge', 'order': order, 'pol': pol, 'inv': inv,
                                    'id': rid, 'value': val, 'fmt': fmt, 'pos': pos})
                        if stop_on is not None and rid in stop_on:
                            return out
        if ss is not None and not integer_k:
            continue
        s = unpack(buf, nsamp)
        for ph in range(int(math.ceil(k - 1e-9))):
            if integer_k:
                d = s[ph::t2 // 300]
            else:
                d = [s[int(ph + j * k)] for j in range(int((nsamp - 1 - ph) / k) + 1)]
            if len(d) < 40:
                continue
            for inv in (0, 1):
                db = pack([x ^ inv for x in d]) if inv else pack(d)
                for pol, pc in (('N', am.POL_NEGATIVE), ('S', am.POL_STANDARD)):
                    for (fmt, rid, val), pos, _ in am.scan_polarity(
                            db, len(d), pc, MAX_GAP_BITS, 8, MIN_IDLE_BITS):
                        out.append({'method': 'dec%d' % ph, 'order': order, 'pol': pol,
                                    'inv': inv, 'id': rid, 'value': val, 'fmt': fmt,
                                    'pos': int(ph + pos * k)})
                        if stop_on is not None and rid in stop_on:
                            return out
    return out


def shuffle_runs(s, rnd):
    """Same run lengths in a random order, same first level."""
    runs = run_lengths(s)
    rnd.shuffle(runs)
    out = []
    v = s[0]
    for r in runs:
        out.extend([v] * r)
        v ^= 1
    return out


def _chance_task(task):
    """Worker: table-hit count over `count` run-length shuffles of one window."""
    data, nsamp, t2, seed, count, table, use_edge = task
    s = unpack(data, nsamp)
    rnd = random.Random(seed)
    hits = 0
    for _ in range(count):
        sh = pack(shuffle_runs(s, rnd))
        if any(r['id'] in table for r in host_decode(sh, nsamp, t2, use_edge, table)):
            hits += 1
    return hits


class ChanceRunner:
    def __init__(self, jobs):
        self.jobs = max(1, jobs)
        self.pool = None

    def map(self, tasks):
        if self.jobs > 1 and len(tasks) > 1:
            if self.pool is None:
                from concurrent.futures import ProcessPoolExecutor
                self.pool = ProcessPoolExecutor(self.jobs)
            return list(self.pool.map(_chance_task, tasks))
        return [_chance_task(t) for t in tasks]

    def close(self):
        if self.pool is not None:
            self.pool.shutdown()


def chance_control(windows, n_bursts, decode, t2, table, runner, shuffles, seed, use_edge):
    """Expected chance table-hit bursts E over n_bursts, and whether
    decode >= 20 E. Stops early once the outcome is settled: a fail as soon as
    the point estimate at `shuffles` can no longer pass, a pass as soon as a
    ~95% upper bound on E already does (so it is never looser than running
    all of them). windows: [(bytes, nsamp)] of the bursts with H data."""
    if not windows or decode <= 0:
        return {'E': None, 'pass': False, 'shuffles': 0, 'hits': 0}
    scale = n_bursts / len(windows)       # bursts without H data: assume the same rate
    done = hits = 0
    step = 20
    while done < shuffles:
        c = min(step, shuffles - done)
        tasks = [(d, n, t2, seed + 7919 * i + done, c, table, use_edge)
                 for i, (d, n) in enumerate(windows)]
        hits += sum(runner.map(tasks))
        done += c
        if scale * hits / shuffles * CHANCE_FACTOR > decode:
            break                          # cannot pass any more
        upper = scale * (hits + 3.0 + 2.0 * math.sqrt(hits)) / done
        if decode >= CHANCE_FACTOR * upper:
            break
    E = scale * hits / done
    return {'E': E, 'pass': decode >= CHANCE_FACTOR * E, 'shuffles': done, 'hits': hits}


# ---------------------------------------------------------------------------
# log parsing

TS_ISO = re.compile(r'^(\d{4}-\d\d-\d\d[T ]\d\d:\d\d:\d\d(?:\.\d+)?)\s+(.*)$')
TS_TOD = re.compile(r'^\[(\d\d):(\d\d):(\d\d(?:\.\d+)?)\]\s+(.*)$')
H_LINE = re.compile(r'^H\s+(\S+)\s+(\d+)\s+(\d+)\s+([0-9A-Fa-f]*)\s*$')


def kv(tokens):
    return dict(t.split('=', 1) for t in tokens if '=' in t)


def ival(s, base=10):
    try:
        return int(s, base)
    except (TypeError, ValueError):
        return None


class Arr:
    def __init__(self, key, idx, tag='?', v=0, params=None, t2=None):
        self.key = key
        self.idx = idx
        self.tag = tag
        self.v = v
        self.params = params or {}
        self.t2 = t2
        self.phases = set()
        self.bursts = []
        self.seconds = 0.0
        self.y_noise = 0
        self.y_burst = 0
        self.fin = []

    @property
    def s4(self):
        return self.params.get('s4') == '1'

    @property
    def stream(self):
        return self.tag in STREAM_TAGS or self.params.get('pol') == 'T'

    def label(self):
        return '%s %s v%s' % (self.idx, self.tag, self.v)


class Burst:
    def __init__(self, sess, idx, num, arr):
        self.sess, self.idx, self.num, self.arr = sess, idx, num, arr
        self.c = None
        self.chunks = {}
        self.fw = []

    def data(self):
        """Stored window bytes, contiguous from offset 0, and whether any were missing."""
        out = bytearray()
        for off in sorted(self.chunks):
            if off > len(out):
                return bytes(out), False
            out[off:off + len(self.chunks[off])] = self.chunks[off]
        want = ival((self.c or {}).get('n'))
        return bytes(out), want is None or 8 * len(out) >= want

    def nsamp(self, data):
        n = ival((self.c or {}).get('n'))
        return min(n, 8 * len(data)) if n is not None else 8 * len(data)


class Log:
    def __init__(self):
        self.arrs = OrderedDict()
        self.bursts = OrderedDict()
        self.sess = 0
        self.cur = {}
        self.seg = None             # [arr, t_start, heartbeats]
        self.t = None
        self.tod_base = None
        self.census = []
        self.adopt = []
        self.boots = []
        self.files = 0
        self.timed = False

    # time: ISO stamps from radio.py; [HH:MM:SS.mmm] from alertmon (wraps at midnight)
    def _stamp(self, line):
        m = TS_ISO.match(line)
        if m:
            try:
                self.t = datetime.fromisoformat(m.group(1)).timestamp()
                self.timed = True
            except ValueError:
                pass
            return m.group(2)
        m = TS_TOD.match(line)
        if m:
            tod = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
            if self.tod_base is None:
                self.tod_base = 0.0
            elif self.t is not None and tod + self.tod_base < self.t - 43200:
                self.tod_base += 86400.0
            self.t = tod + self.tod_base
            self.timed = True
            return m.group(4)
        return line

    def _close(self):
        if self.seg:
            arr, t0, beats = self.seg
            if t0 is not None and self.t is not None and self.t >= t0:
                arr.seconds += self.t - t0
            else:
                arr.seconds += 0.5 * beats      # D heartbeats are 500 ms apart
        self.seg = None

    def _placeholder(self, idx, t2=None):
        key = ('?', idx)
        arr = self.arrs.get(key)
        if arr is None:
            arr = self.arrs[key] = Arr(key, idx, 'ADC' if idx == 'ADC' else '?', 0, {}, t2)
        if arr.t2 is None and t2:
            arr.t2 = t2
        return arr

    def _arr(self, idx, t2=None):
        return self.cur.get(idx) or self._placeholder(idx, t2)

    def _burst(self, idx, num, t2=None):
        key = (self.sess, idx, num)
        b = self.bursts.get(key)
        if b is None:
            arr = self._arr(idx, t2)
            b = self.bursts[key] = Burst(self.sess, idx, num, arr)
            arr.bursts.append(b)
        return b

    def feed_file(self, path):
        self.files += 1
        self.sess += 1
        self.cur = {}
        self.t = None
        self.tod_base = None
        with open(path, 'r', encoding='ascii', errors='replace') as f:
            for raw in f:
                self.feed(raw)
        self._close()

    def feed(self, raw):
        line = self._stamp(raw.strip())
        if not line or line.startswith('#'):
            return
        tok = line.split()
        tag = tok[0]
        try:
            self._dispatch(tag, tok, line)
        except (IndexError, KeyError, ValueError):
            pass                            # a torn line: skip it, never stop

    def _dispatch(self, tag, tok, line):
        if tag == 'D':
            if self.seg:
                self.seg[2] += 1
        elif tag == 'L':
            f = kv(tok[3:])
            idx, atag = tok[1], tok[2]
            params = {k: f[k] for k in ('r58', 't2', 'r70', 't1', 'sy', 's4', '5c', 'pol')
                      if k in f}
            v = ival(f.get('v', '0'), 16) or 0      # the variant byte, printed %02X
            key = (idx, atag, v) + tuple(sorted(params.items()))
            arr = self.arrs.get(key)
            if arr is None:
                arr = self.arrs[key] = Arr(key, idx, atag, v, params, ival(f.get('t2')))
            ph = ival(f.get('ph'))
            if ph is not None:
                arr.phases.add(ph)
            self._close()
            self.cur[idx] = arr
            self.seg = [arr, self.t, 0]
        elif tag == 'B':
            self._close()
            self.sess += 1
            self.cur = {}
            self.boots.append(line)
        elif tag == 'C':
            f = kv(tok[2:])
            k100, num = ival(f.get('k')), ival(f.get('b'))
            if num is None:
                raise ValueError('C line without a burst number')
            b = self._burst(tok[1], num, 3 * k100 if k100 else None)
            b.c = f
        elif tag == 'H':
            m = H_LINE.match(line)
            if m:
                b = self._burst(m.group(1), int(m.group(2)))
                b.chunks[int(m.group(3))] = bytes.fromhex(m.group(4))
        elif tag == 'X':
            f = kv(tok[3:])
            self._burst(tok[1], int(tok[2])).fw.append(f)
        elif tag == 'Y':
            f = kv(tok[2:])
            arr = self._arr(tok[1])
            if f.get('sq') == '0':
                arr.y_noise += 1
            else:
                arr.y_burst += 1
        elif tag == 'F':
            f = kv(tok[2:])
            self._arr(tok[1]).fin.append((ival(f.get('words')), ival(f.get('ms'))))
        elif tag in ('P', 'PT', 'AUD'):
            self.census.append((tag, kv(tok[1:]), line))
        elif tag == 'G':
            self.adopt.append(line)
        # A, ALERT, Z, K, W, T, U and anything newer: not needed for a verdict


# ---------------------------------------------------------------------------
# judging

def judge_arr(arr, table, runner, opts):
    t2 = arr.t2
    k = t2 / 300.0 if t2 else 0.0
    # ADC decodes arrive as X lines only; every FSK burst has its C line
    bursts = [b for b in arr.bursts if b.c is not None or arr.idx == 'ADC']
    for b in bursts:
        b.c = b.c or {}
    r = OrderedDict(idx=arr.idx, tag=arr.tag, v=arr.v, t2=t2, k=round(k, 3),
                    stream=arr.stream, phases=sorted(arr.phases), bq=len(bursts))

    # BITS
    fills = []
    for b in bursts:
        n, win = ival(b.c.get('n')), ival(b.c.get('win'))
        if n is not None and win and t2:
            fills.append(n >= BITS_FILL * win * t2 / 1000.0)
    r['bits'] = sum(fills) / len(fills) if fills else None

    # SYNC, over the bursts where the engine was searching at the open. In the
    # C line that is src=open (it synced during the burst, dt >= 0) or src=none
    # (it never did); src=pre means it was already streaming, which counts only
    # for a sync on the carrier, at most 300 ms before the open.
    elig, good = 0, Counter()
    pns = set()
    for b in bursts:
        src, dt, pn = b.c.get('src'), ival(b.c.get('dt')), b.c.get('pn', '-')
        carrier = dt is not None and SYNC_EARLY_MS <= dt < 0
        if src in ('open', 'none') or (src == 'pre' and carrier):
            elig += 1
            if dt is not None and (0 <= dt <= SYNC_LATE_MS or carrier) and pn in ('P', 'N'):
                good[pn] += 1
                pns.add(pn)
    if elig:
        pn, cnt = good.most_common(1)[0] if good else ('-', 0)
        r['sync'] = cnt / elig
        r['sync_pn'] = pn if len(pns) <= 1 else 'P+N'
    else:
        r['sync'], r['sync_pn'] = None, '-'
    r['stream_coincident'] = sum(1 for b in bursts if b.c.get('src') == 'pre')

    # STRUCT and host decode, burst by burst
    passes = Counter()
    struct_n = 0
    rks, bases = [], []
    per_burst = []
    windows = []
    missing_h = 0
    for b in bursts:
        data, complete = b.data()
        nsamp = b.nsamp(data)
        expected = ival(b.c.get('n'))
        row = {'burst': '%d.%d' % (b.sess, b.num), 'hits': [], 'struct': []}
        crk, cbase = ival(b.c.get('rk')), ival(b.c.get('base'))
        if nsamp >= 16:
            windows.append((data, nsamp))
            for order in ORDERS:
                s = unpack(in_order(data, order), nsamp)
                if order == 'msb' and crk is None:
                    st = window_stats(s, k)
                    crk, cbase = st['rk'] * 1000, st['base'] * 1000
                if t2 and struct_test(s, k, t2)['ok']:
                    row['struct'].append(order)
            if not complete:
                missing_h += 1          # some H lines lost: judged on what arrived
            row['hits'] = host_decode(data, nsamp, t2, not opts.no_edge) if t2 else []
        else:
            if expected:
                missing_h += 1
            # no window on the host: fall back to the firmware's own stats (stored order)
            ctr, clead = ival(b.c.get('tr')), ival(b.c.get('lead'))
            if (k >= STRUCT_MIN_K and crk is not None and ctr is not None and clead is not None
                    and crk >= 1000 * STRUCT_RK and ctr <= 1000 * STRUCT_TR_K / k
                    and clead >= STRUCT_LEAD_K * k):
                row['struct'].append('msb')
        if k >= STRUCT_MIN_K:
            struct_n += 1
            for order in row['struct']:
                passes[order] += 1
        if crk is not None:
            rks.append(crk / 1000.0)
            bases.append((cbase if cbase is not None else 1000 * 2.0 ** (1 - m_for(k))) / 1000.0)
        for f in b.fw:
            rid = ival(f.get('id'))
            if rid is None:
                continue
            row['hits'].append({'method': 'fw', 'order': '-', 'pol': f.get('pol', '?'),
                                'inv': ival(f.get('inv')) or 0, 'id': rid,
                                'value': ival(f.get('v')), 'fmt': f.get('fmt', '?'),
                                'pos': ival(f.get('pos')), 'known': f.get('known') == '1'})
        per_burst.append(row)

    if struct_n:
        cnt = max(passes[o] for o in ORDERS)
        r['struct'] = cnt / struct_n
        # both, when runs are long enough that the order inside a byte barely matters
        r['struct_order'] = '+'.join(o for o in ORDERS if passes[o] == cnt) if cnt else '-'
    else:
        r['struct'], r['struct_order'] = None, '-'
    r['struct_any'] = any(row['struct'] for row in per_burst) and k >= STRUCT_MIN_K
    r['rk'] = sum(rks) / len(rks) if rks else None
    r['base'] = sum(bases) / len(bases) if bases else None
    r['missing_h'] = missing_h

    # DECODE / REPEAT over table stations
    decoded = 0
    keys = defaultdict(set)
    ids = set()
    via = Counter()
    for row in per_burst:
        tab = [h for h in row['hits'] if h['id'] in table or h.get('known')]
        if tab:
            decoded += 1
        seen = set()
        for h in tab:
            ids.add(h['id'])
            path = (h['id'], h['pol'], h['inv'])
            if path not in seen:
                keys[path].add(row['burst'])
                seen.add(path)
                via['%s/%s' % (h['method'], h['order'])] += 1
    r['decode'] = decoded
    r['stations'] = sorted(ids)
    if keys:
        best = max(keys, key=lambda p: (len(keys[p]), -p[0]))
        r['repeat'] = len(keys[best])
        r['repeat_path'] = '%d %s inv%d' % best
    else:
        r['repeat'], r['repeat_path'] = 0, '-'
    r['via'] = dict(via)

    # chance control, only where it can matter
    if decoded and t2:
        r['chance'] = chance_control(windows, len(bursts), decoded, t2, table, runner,
                                     opts.shuffles,
                                     opts.seed + zlib.crc32(arr.label().encode()) % 100000,
                                     not opts.no_edge)
    else:
        r['chance'] = {'E': None, 'pass': False, 'shuffles': 0, 'hits': 0}

    # noise-sync ratio
    if t2 and arr.seconds > 0 and not arr.stream:
        expect = 2.0 * t2 / (2 ** (32 if arr.s4 else 16))
        r['nsr'] = (arr.y_noise / arr.seconds) / expect
    else:
        r['nsr'] = None
    r['armed_s'] = round(arr.seconds, 1)
    r['noise_syncs'] = arr.y_noise
    fin = [w for w, _ in arr.fin if w is not None]
    r['finished_words'] = sorted(fin)[len(fin) // 2] if fin else None

    r['verdict'] = verdict(r)
    return r


def verdict(r):
    if r['bq'] == 0:
        return 'NO-DATA'
    struct = r['struct'] or 0.0
    if r['repeat'] >= REPEAT_WORKS and r['chance']['pass']:
        return 'WORKS'
    if r['repeat'] == 2 or (len(r['stations']) >= 2 and struct >= STRUCT_MAJORITY):
        return 'LIKELY'
    if r['repeat'] >= REPEAT_WORKS:
        return 'CHANCE'                 # repeats, but no better than shuffled runs do
    if r['struct'] is not None and struct >= STRUCT_MAJORITY and r['decode'] == 0:
        return 'BITS-NO-DECODE'
    bits = r['bits']
    if (bits is not None and bits >= 0.5 and r['rk'] is not None and r['base'] is not None
            and abs(r['rk'] - r['base']) <= NOISE_RK_MARGIN):
        return 'NOISE-BITS'
    if bits is not None and bits < FEW_BITS:
        return 'NO-BITS'
    return 'MIXED'


def coverage(results):
    """What "FSK route dead" still needs; an empty list means nothing."""
    need = []
    by_tag = defaultdict(lambda: [0, 0])
    p3 = 0
    for r in results:
        if r['v'] == 0 and r['tag'] in PHASE1_TAGS:
            by_tag[r['tag']][0] += r['bq']
            by_tag[r['tag']][1] += r['stream_coincident']
        if 3 in r['phases'] and r['bq'] >= MIN_BURSTS_DEAD:
            p3 += 1
    short = [t for t in PHASE1_TAGS if t not in STREAM_TAGS and by_tag[t][0] < MIN_BURSTS_DEAD]
    if short:
        need.append('Phase 1 rows with < %d bursts: %s' % (MIN_BURSTS_DEAD, ' '.join(short)))
    for t in STREAM_TAGS:
        if by_tag[t][1] < MIN_BURSTS_DEAD:
            need.append('%s: %d of %d stream-coincident bursts' % (t, by_tag[t][1],
                                                                   MIN_BURSTS_DEAD))
    if p3 < PHASE3_ROWS:
        need.append('Phase 3: %d of %d arrangements with >= %d bursts'
                    % (p3, PHASE3_ROWS, MIN_BURSTS_DEAD))
    return need


def judge_adc(log, results):
    """ADC route: AUD pass, burst confirmation, REPEAT >= 3 on idx ADC.

    Confirmation is s5's: on the first 3 bursts, 1300 and 2100 Hz energy on the
    chosen pin both >= 10 dB (100 in dB x10) above that pin's F-idle level."""
    aud = [f for t, f, _ in log.census if t == 'AUD']
    pins = [f.get('pin', 'none') for f in aud]
    aud_pass = any(p != 'none' for p in pins)
    chosen = {'PA4': '4', 'PA4B': '4', 'PB1': '9'}.get(pins[-1] if pins else 'none')
    idle = {}
    confirm = []
    for t, f, _ in log.census:
        if t != 'P':
            continue
        where = (f.get('ch'), f.get('b'), f.get('pa'))
        if f.get('src') == 'F':
            idle[where] = f
            idle[where[0]] = f
        elif (f.get('src') == 'BURST' and len(confirm) < 3 and
              (chosen is None or where[0] == chosen)):
            base = idle.get(where) or idle.get(where[0])
            if base is None:
                continue
            g13, g21 = ival(f.get('g13')), ival(f.get('g21'))
            b13, b21 = ival(base.get('g13')), ival(base.get('g21'))
            if None not in (g13, g21, b13, b21):
                confirm.append(g13 >= b13 + 100 and g21 >= b21 + 100)
    adc = [r for r in results if r['idx'] == 'ADC']
    repeat = max((r['repeat'] for r in adc), default=0)
    # the firmware only runs the ADC decoder once it has confirmed the pin itself
    confirmed = sum(confirm) >= 2 or (not confirm and repeat > 0)
    return {'aud': [ln for t, _, ln in log.census if t == 'AUD'], 'aud_pass': aud_pass,
            'confirm': '%d/%d' % (sum(confirm), len(confirm)), 'confirmed': confirmed,
            'repeat': repeat, 'works': aud_pass and confirmed and repeat >= REPEAT_WORKS}


def overall(results, adc):
    fsk = [r for r in results if r['idx'] != 'ADC']
    wins = [r for r in fsk if r['verdict'] in ('WORKS', 'LIKELY')]
    if wins or adc['works']:
        best = sorted(wins, key=lambda r: (r['verdict'] != 'WORKS', -r['repeat']))
        what = (['%s %s (idx %s v%s)' % (r['verdict'], r['tag'], r['idx'], r['v'])
                 for r in best] + (['ADC route works'] if adc['works'] else []))
        return 0, '; '.join(what), []
    nodec = [r for r in fsk if r['verdict'] == 'BITS-NO-DECODE']
    if nodec:
        return 1, 'BITS-NO-DECODE on %s: analyse offline with capture_stats.py' % ', '.join(
            '%s (idx %s, %s order)' % (r['tag'], r['idx'], r['struct_order']) for r in nodec), []
    need = coverage(fsk)
    if not need and not any(r['struct_any'] for r in fsk):
        return 2, 'FSK route dead: full coverage and STRUCT never passed', []
    if not need:
        return 3, 'not enough data: STRUCT passed somewhere but on under 2/3 of its bursts', []
    return 3, 'not enough data', need


def fmt_frac(x):
    return '  - ' if x is None else '%4.2f' % x


def report(log, results, adc, code, why, need, opts):
    ss = edge_slicer()
    nb = sum(r['bq'] for r in results)
    print('ALERT-X1 sweep judge: %d log(s), %d boot(s), %d arrangement(s), %d qualifying bursts'
          % (log.files, len(log.boots), len(results), nb))
    print('host decode: %s + phase decimation, %d-bit preamble gate, %d table ids'
          % ('edge slicer (scan_samples.py)' if ss and not opts.no_edge
             else 'NO edge slicer (scan_samples.py missing)', MIN_IDLE_BITS, len(opts.table)))
    if not log.timed:
        print('no PC timestamps: armed time estimated from D heartbeats (0.5 s each)')
    print()
    print('%-4s %-5s %2s %5s %1s %4s %4s %6s %-12s %3s %3s %13s %5s  %s'
          % ('idx', 'tag', 'v', 'TONE2', 'p', 'bq', 'BITS', 'SYNC', 'STRUCT', 'DEC', 'REP',
             'chance', 'NSR', 'verdict'))
    for r in results:
        sync = '  -   ' if r['sync'] is None else '%4.2f %s' % (r['sync'], r['sync_pn'][:1])
        if r['struct'] is None:
            struct = ' n/a'
        else:
            struct = '%4.2f %s' % (r['struct'], r['struct_order'])
        ch = r['chance']
        if ch['E'] is None:
            chance = '-'
        else:
            chance = 'E=%.3f %s' % (ch['E'], 'ok' if ch['pass'] else 'FAIL')
        nsr = '  -  ' if r['nsr'] is None else '%5.1f' % r['nsr']
        if r['nsr'] is not None and r['nsr'] > NSR_FLAG:
            nsr += '!'
        extra = []
        if r['repeat']:
            extra.append('id %s' % r['repeat_path'])
        if r['stream']:
            extra.append('%d stream-coincident' % r['stream_coincident'])
        if r['missing_h']:
            extra.append('%d without full H' % r['missing_h'])
        print('%-4s %-5s %2s %5s %1s %4d %s %6s %-12s %3d %3d %13s %5s  %s%s'
              % (r['idx'], r['tag'][:5], r['v'], r['t2'] or '?', 'T' if r['stream'] else 'S',
                 r['bq'], fmt_frac(r['bits']), sync, struct, r['decode'], r['repeat'], chance,
                 nsr, r['verdict'], ('  (' + ', '.join(extra) + ')') if extra else ''))
    print()
    flagged = [r for r in results if r['nsr'] is not None and r['nsr'] > NSR_FLAG]
    if flagged:
        print('NSR > %g (the sync detector tolerates errors, or the noise is correlated): %s'
              % (NSR_FLAG, ', '.join(r['tag'] for r in flagged)))
    fin = [r['finished_words'] for r in results if r['finished_words'] is not None]
    if fin:
        med = sorted(fin)[len(fin) // 2]
        print('RX_FINISHED: median %d words after sync (~1024: the 11-bit REG_5D length works;'
              ' ~128: it does not)' % med)
    for ln in log.adopt:
        print(ln)
    if adc['aud']:
        print('census: %s' % ' | '.join(adc['aud']))
        print('ADC route: AUD %s, bursts confirmed %s, REPEAT %d -> %s'
              % ('pass' if adc['aud_pass'] else 'fail', adc['confirm'], adc['repeat'],
                 'WORKS' if adc['works'] else 'no'))
    print()
    print('Overall: %s  (exit %d)' % (why, code))
    for n in need:
        print('  still needed: %s' % n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--shuffles', type=int, default=CHANCE_SHUFFLES,
                    help='run-length shuffles per capture (default %d)' % CHANCE_SHUFFLES)
    ap.add_argument('--jobs', type=int, default=os.cpu_count() or 1,
                    help='processes for the chance control')
    ap.add_argument('--seed', type=int, default=4242)
    ap.add_argument('--no-edge', action='store_true',
                    help='phase decimation only, as if scan_samples.py were missing')
    ap.add_argument('--table', default=STATIONS_H, help='alert_stations_gen.h')
    ap.add_argument('--json', help='also write the per-arrangement results here')
    opts = ap.parse_args(argv)

    opts.table = load_table(opts.table)
    log = Log()
    for path in opts.logs:
        log.feed_file(path)

    def order(a):
        i = ival(a.idx)
        return (i is None, i if i is not None else 0, str(a.idx), a.v)

    runner = ChanceRunner(opts.jobs)
    try:
        results = [judge_arr(a, opts.table, runner, opts)
                   for a in sorted(log.arrs.values(), key=order)]
    finally:
        runner.close()
    adc = judge_adc(log, results)
    code, why, need = overall(results, adc)
    report(log, results, adc, code, why, need, opts)
    if opts.json:
        with open(opts.json, 'w') as f:
            json.dump({'exit': code, 'overall': why, 'still_needed': need, 'adc': adc,
                       'arrangements': results}, f, indent=1, default=str)
    return code


if __name__ == '__main__':
    sys.exit(main())
