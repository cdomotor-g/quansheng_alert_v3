#!/usr/bin/env python3
"""Python port of ALERT_ScanSamples (App/app/alert_decode.c), kept in lockstep.

The FSK engine hands over an oversampled slicer stream: k = TONE2 / 300
samples per ALERT bit, often fractional (1300 Hz gives 4.33). This is a
software UART over that stream. It re-times every word from that word's own
transitions, so a transmitter a few percent off 300 baud still decodes, and it
only believes a frame that follows a real idle preamble, because ABF and EIF
each constrain only about 8 bits and noise otherwise decodes all the time.

Same algorithm and the same integer arithmetic as the C, statement for
statement, so a host judge that runs this on a logged capture sees exactly
what the radio saw. Change one, change the other, and rerun both self-tests:
the LOCKSTEP table below is asserted by tools/alert/test_decode.c too.

    python tools/alert/scan_samples.py              # self-test, stdlib only
    python tools/alert/scan_samples.py --lockstep   # print the LOCKSTEP table

API (mirrors the C):
    scan_samples(buf, nsamp, spb_q8, polarity, invert, min_idle_bits, max_out=8)
    scan_bits_gated(buf, nbits, polarity, max_gap, invert, min_idle_bits, max_out=8)
    spb_q8_for(tone2_hz)
Both return a list of Reading(id, value, format, polarity, bit_pos).

Polarity numbering follows the C enum (STANDARD 0, NEGATIVE 1, ANY 2).
alertmon.py numbers them the other way round; do not mix the two.
"""
import os
import random
import re
import sys
from collections import namedtuple

POL_STANDARD, POL_NEGATIVE, POL_ANY = 0, 1, 2
FMT_NONE, FMT_ABF, FMT_EIF = 0, 1, 2

SCAN_MAX_GAP = 20      # idle bits allowed between the words of one frame
SCAN_EXEMPT_BITS = 20  # a frame this close after an accepted one skips the idle gate
SCAN_MAX_TRANS = 20    # more transitions than this inside one word span is noise
SCAN_MED_MIN_Q8 = 512  # median-of-3 only when every bit spans at least 2 samples
SCAN_SPB_MIN_Q8 = 256
SCAN_SPB_MAX_Q8 = 4096
SCAN_NSAMP_MAX = 0xFFFF  # bit_pos is 16 bits

# word timing candidates: phase in eighths of a bit, rate in 1/40ths (2.5 %)
PHASE_STEPS = (0, -1, 1, -2, 2)
RATE_STEPS = (0, -1, 1)

Reading = namedtuple('Reading', 'id value format polarity bit_pos')


# --------------------------------------------------------------------------
# frame decoding, port of ALERT_Crc6 / ALERT_DecodePayload32

def crc6(bits, nbits):
    reg = 0
    for i in range(nbits - 1, -1, -1):
        b = (bits >> i) & 1
        fb = ((reg >> 5) & 1) ^ b
        reg = (reg << 1) & 0x3F
        if fb:
            reg ^= 0x19
    return reg


def decode_payload32(payload):
    """(format, id, value) or None; payload bit 31 = first transmitted data bit."""
    def P(p):
        return (payload >> (31 - p)) & 1

    def lsb_first(p, n):
        return sum(P(p + i) << i for i in range(n))

    def msb_first(p, n):
        v = 0
        for i in range(n):
            v = (v << 1) | P(p + i)
        return v

    k1 = (P(6) << 1) | P(7)
    k2 = (P(14) << 1) | P(15)
    if k1 == 2 and k2 == 2:
        if P(22) and P(23) and P(30) and P(31):
            a = lsb_first(0, 6) | (lsb_first(8, 6) << 6) | (lsb_first(16, 1) << 12)
            d = lsb_first(17, 5) | (lsb_first(24, 6) << 5)
            return (FMT_ABF, a, d)
        return None
    if k1 == 3:
        a = lsb_first(0, 6) | (lsb_first(8, 7) << 6)
        d = lsb_first(15, 1) | (lsb_first(16, 8) << 1) | (lsb_first(24, 2) << 9)
        if crc6((a << 11) | d, 24) == msb_first(26, 6):
            return (FMT_EIF, a, d)
    return None


def assemble(words):
    """C: assemble(). Payload position 8k+i = data bit i of word k."""
    p = 0
    for k in range(4):
        for i in range(8):
            p |= ((words[k] >> i) & 1) << (31 - (8 * k + i))
    return p


def getbit(buf, n):
    return (buf[n >> 3] >> (7 - (n & 7))) & 1


def spb_q8_for(tone2_hz):
    """C: ALERT_SPB_Q8(). Samples per ALERT bit in Q8: round(256 * f / 300)."""
    return (256 * tone2_hz + 150) // 300


# --------------------------------------------------------------------------
# ALERT_ScanSamples

class Samples:
    """C: Samples_t + samp(): bit i of the stream, complemented when inverted,
    through a median-of-3 when a bit is long enough to survive one."""

    def __init__(self, buf, n, inv, med):
        self.buf, self.n, self.inv, self.med = buf, n, inv, med

    def __call__(self, i):
        b = getbit(self.buf, i) ^ self.inv
        if not self.med or i == 0 or i + 1 >= self.n:
            return b
        s = b + (getbit(self.buf, i - 1) ^ self.inv) + (getbit(self.buf, i + 1) ^ self.inv)
        return 1 if s >= 2 else 0


def read_word(s, e, spb_q8):
    """C: read_word(). Returns (rc, bits, stop_q8): rc 1 = a word was read
    (bit j of bits = ALERT bit j, 0 start .. 9 stop; stop_q8 = stop-bit centre),
    0 = it would run past the end of the buffer, -1 = too busy to be a word."""
    spb = spb_q8
    q = spb >> 3
    dr = spb // 40
    h = spb >> 10
    b0 = (e << 8) - 128           # the start edge lies between samples e-1 and e
    cmax = (b0 + 2 * q + ((spb + dr) >> 1) + 9 * (spb + dr) + 128) >> 8
    if cmax + h >= s.n:
        return 0, 0, 0

    ts = []
    xend = (b0 + 9 * spb + (spb >> 1)) >> 8
    p = s(e - 1)
    for x in range(e, xend + 1):
        v = s(x)
        if v != p:
            if len(ts) == SCAN_MAX_TRANS:
                return -1, 0, 0
            ts.append(x - e)
        p = v

    best_cost = None
    best_base = 0
    best_r = spb
    for rs in RATE_STEPS:
        r = spb + rs * dr
        for ps in PHASE_STEPS:
            bnd = b0 + ps * q
            mid = bnd + (r >> 1)
            cost = 0
            for off in ts:
                t = b0 + (off << 8)
                while t > mid:
                    bnd += r
                    mid += r
                d = t - bnd
                if d < 0:
                    d = -d
                cost += d
                if d > 128:
                    cost += 4 * (d - 128)
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_base = b0 + ps * q + (r >> 1)
                best_r = r

    bits = 0
    for j in range(10):
        c = (best_base + j * best_r + 128) >> 8
        ones = 0
        for x in range(c - h, c + h + 1):
            ones += s(x)
        if 2 * ones > 2 * h + 1:
            bits |= 1 << j
    return 1, bits, best_base + 9 * best_r


def _scan_samples_pol(s, spb_q8, pol, min_idle_bits, out, max_out):
    idle = 0 if pol == POL_NEGATIVE else 1
    start = idle ^ 1
    need = (min_idle_bits * spb_q8) >> 8

    words = [0] * 4
    edge = [0] * 4
    nwords = 0
    found = 0
    last_stop_q8 = -1

    if s.n < 2:
        return 0
    i = 1
    prev = s(0)
    while i < s.n and found < max_out:
        cur = s(i)
        if prev != idle or cur != start:
            prev = cur
            i += 1
            continue
        e = i
        rc, bits, stop_q8 = read_word(s, e, spb_q8)
        if rc == 0:
            break
        if rc < 0 or (bits & 1) != start or ((bits >> 9) & 1) != idle:
            nwords = 0
            prev = cur
            i += 1
            continue

        if nwords and ((e - edge[nwords - 1]) << 8) > (10 + SCAN_MAX_GAP) * spb_q8:
            nwords = 0
        if nwords == 4:
            words[0:3] = words[1:4]
            edge[0:3] = edge[1:4]
            nwords = 3
        words[nwords] = (bits >> 1) & 0xFF
        edge[nwords] = e
        nwords += 1

        c = (stop_q8 + 128) >> 8
        prev = s(c)
        i = c + 1

        if nwords == 4:
            e0 = edge[0]
            gate = last_stop_q8 >= 0 and (e0 << 8) <= last_stop_q8 + SCAN_EXEMPT_BITS * spb_q8
            if not gate and e0 >= need:
                j = e0 - need
                while j < e0 and s(j) == idle:
                    j += 1
                gate = j == e0
            if gate:
                d = decode_payload32(assemble(words))
                if d is not None:
                    out.append(Reading(d[1], d[2], d[0], pol, e0))
                    found += 1
                    last_stop_q8 = stop_q8 + (spb_q8 >> 1)
                    nwords = 0
    return found


def scan_samples(buf, nsamp, spb_q8, polarity, invert, min_idle_bits, max_out=8):
    """Port of ALERT_ScanSamples. buf: bytes, sample n = bit n MSB-first
    (ALERT_GetBit order). Returns a list of Reading, at most max_out."""
    out = []
    if spb_q8 < SCAN_SPB_MIN_Q8 or spb_q8 > SCAN_SPB_MAX_Q8 or max_out <= 0:
        return out
    if nsamp > SCAN_NSAMP_MAX:
        nsamp = SCAN_NSAMP_MAX
    s = Samples(buf, nsamp, 1 if invert else 0, spb_q8 >= SCAN_MED_MIN_Q8)
    if polarity != POL_ANY:
        _scan_samples_pol(s, spb_q8, polarity, min_idle_bits, out, max_out)
        return out
    n = _scan_samples_pol(s, spb_q8, POL_NEGATIVE, min_idle_bits, out, max_out)
    if n < max_out:
        _scan_samples_pol(s, spb_q8, POL_STANDARD, min_idle_bits, out, max_out - n)
    return out


# --------------------------------------------------------------------------
# ALERT_ScanBitsGated: the 1x scanner (scan_polarity) with the same idle gate,
# for streams already reduced to one sample per bit (phase decimation)

def _scan_bits_pol(buf, nbits, pol, max_gap, inv, min_idle_bits, out, max_out):
    idle = 0 if pol == POL_NEGATIVE else 1
    start = idle ^ 1

    def GB(n):
        return getbit(buf, n) ^ inv

    words = [0] * 4
    word_pos = [0] * 4
    nwords = 0
    found = 0
    last_stop = -1
    pos = 0
    while pos + 10 <= nbits and found < max_out:
        if GB(pos) != start:
            pos += 1
            continue
        if pos > 0 and GB(pos - 1) != idle:
            pos += 1
            continue
        if GB(pos + 9) != idle:
            pos += 1
            nwords = 0
            continue
        w = 0
        for i in range(8):
            w |= GB(pos + 1 + i) << i
        if nwords and (pos - (word_pos[nwords - 1] + 10)) > max_gap:
            nwords = 0
        if nwords == 4:
            words[0:3] = words[1:4]
            word_pos[0:3] = word_pos[1:4]
            nwords = 3
        words[nwords] = w
        word_pos[nwords] = pos
        nwords += 1
        pos += 10
        if nwords == 4:
            p0 = word_pos[0]
            gate = min_idle_bits == 0 or (last_stop >= 0 and p0 <= last_stop + SCAN_EXEMPT_BITS)
            if not gate and p0 >= min_idle_bits:
                j = p0 - min_idle_bits
                while j < p0 and GB(j) == idle:
                    j += 1
                gate = j == p0
            if gate:
                d = decode_payload32(assemble(words))
                if d is not None:
                    out.append(Reading(d[1], d[2], d[0], pol, p0 & 0xFFFF))
                    found += 1
                    last_stop = pos
                    nwords = 0
    return found


def scan_bits_gated(buf, nbits, polarity, max_gap, invert, min_idle_bits, max_out=8):
    out = []
    inv = 1 if invert else 0
    if polarity != POL_ANY:
        _scan_bits_pol(buf, nbits, polarity, max_gap, inv, min_idle_bits, out, max_out)
        return out
    n = _scan_bits_pol(buf, nbits, POL_NEGATIVE, max_gap, inv, min_idle_bits, out, max_out)
    if n < max_out:
        _scan_bits_pol(buf, nbits, POL_STANDARD, max_gap, inv, min_idle_bits, out, max_out - n)
    return out


# ==========================================================================
# self-test
#
# The PRNG and the frame synthesiser are integer-only and mirrored in
# tools/alert/test_decode.c, so both build byte-identical sample buffers.

class Rng:
    """xorshift32, as rng() in test_decode.c"""

    def __init__(self, seed):
        self.s = seed

    def next(self):
        x = self.s
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.s = x
        return x


def abf_words(a, d):
    return [(a & 0x3F) | 0x40, ((a >> 6) & 0x3F) | 0x40,
            ((a >> 12) & 1) | ((d & 0x1F) << 1) | 0xC0, ((d >> 5) & 0x3F) | 0xC0]


def eif_words(a, d):
    r = crc6((a << 11) | d, 24)
    w3 = (d >> 9) & 0x03
    for j in range(6):
        w3 |= ((r >> (5 - j)) & 1) << (2 + j)
    return [(a & 0x3F) | 0xC0, ((a >> 6) & 0x7F) | ((d & 1) << 7), (d >> 1) & 0xFF, w3]


LOGIC_NEG, LOGIC_STD = 0, 1


def symbols(frames, logic, pre_bits, gap_bits=0, tail_bits=12, lead_mark_bits=0):
    """Tone symbols, 0 = idle tone, 1 = start tone. Negative logic (field
    hardware) sends data 1 on the start tone, standard logic on the idle tone.
    lead_mark_bits of start tone ahead of the preamble bound the idle run."""
    s = [1] * lead_mark_bits + [0] * pre_bits
    for f, words in enumerate(frames):
        if f:
            s += [0] * gap_bits
        for w in words:
            s.append(1)
            for i in range(8):
                b = (w >> i) & 1
                s.append(b if logic == LOGIC_NEG else b ^ 1)
            s.append(0)
    return s + [0] * tail_bits


def sample(syms, idle_level, tone2_hz, ppm, phase_q16):
    """Slicer at TONE2 samples/s against a transmitter at 300 * (1 + ppm/1e6)
    baud: sample i reads symbol (phase + i * step) >> 16, step in Q16."""
    step_q16 = (65536 * 300 * (1000000 + ppm)) // (tone2_hz * 1000000)
    out = []
    i = 0
    while True:
        k = (phase_q16 + i * step_q16) >> 16
        if k >= len(syms):
            break
        out.append(syms[k] ^ idle_level)
        i += 1
    return out


def pack(samples):
    buf = bytearray((len(samples) + 7) // 8)
    for i, b in enumerate(samples):
        if b:
            buf[i >> 3] |= 0x80 >> (i & 7)
    return bytes(buf), len(samples)


def noise_buf(seed, nsamp):
    r = Rng(seed)
    return bytes(r.next() >> 24 for _ in range((nsamp + 7) // 8)), nsamp


# the four (logic, slicer sense) cases and the (framing, invert) that must read them
COMBOS = (
    (LOGIC_NEG, 0, POL_NEGATIVE, 0),
    (LOGIC_NEG, 1, POL_NEGATIVE, 1),
    (LOGIC_STD, 1, POL_STANDARD, 0),
    (LOGIC_STD, 0, POL_STANDARD, 1),
)
TONES = (521, 1042, 1200, 1300, 1400, 1500, 1800, 2400)   # k 1.74 .. 8
PPMS = (-20000, 0, 20000)
PHASES = (0, 0x5555, 0xAAAA, 0xE000)


def load_stations():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', '..', 'App', 'app', 'alert_stations_gen.h')
    ids = set()
    with open(path, encoding='utf-8') as f:
        for base, kinds in re.findall(r'\{\s*(\d+),\s*0x([0-9a-fA-F]+),\s*\d+\s*\}', f.read()):
            base, kinds = int(base), int(kinds, 16)
            for off in range(5):
                if (kinds >> (3 * off)) & 7:
                    ids.add(base + off)
    return ids


def digest(readings):
    """order-sensitive checksum, as digest() in test_decode.c"""
    h = 0
    for r in readings:
        for v in (r.id, r.value, r.format, r.polarity, r.bit_pos):
            h = (h * 31 + v) & 0xFFFFFFFF
    return h


def clean_frames(tone):
    """test 1's sweep: every rate error, combination and phase, noise-free.
    Returns (frames that decoded exactly, all readings from the framing-specific
    scans in order)."""
    spb = spb_q8_for(tone)
    ok = 0
    readings = []
    for ppm in PPMS:
        for ci, (logic, lvl, pol, inv) in enumerate(COMBOS):
            for pi, ph in enumerate(PHASES):
                a, d = 6129, 1599
                eif = (ci + pi) & 1
                words = eif_words(a, d) if eif else abf_words(a, d)
                buf, n = pack(sample(symbols([words], logic, 60), lvl, tone, ppm, ph))
                r = scan_samples(buf, n, spb, pol, inv, 12)
                ra = scan_samples(buf, n, spb, POL_ANY, inv, 12)
                readings += r
                ok += (len(r) == 1 and r[0].id == a and r[0].value == d and
                       r[0].format == (FMT_EIF if eif else FMT_ABF) and
                       r[0].polarity == pol and
                       any(x.id == a and x.value == d for x in ra))
    return ok, readings


def noisy_frames(tone):
    """16 random frames, random rate error within 2.5 %, phase and
    sense/framing, 2 % of samples flipped; all drawn from the shared PRNG."""
    rng = Rng(LOCKSTEP_SEED ^ tone)
    spb = spb_q8_for(tone)
    readings = []
    for _ in range(16):
        a = rng.next() % 8192
        d = rng.next() % 2048
        eif = rng.next() & 1
        logic, lvl, pol, inv = COMBOS[rng.next() & 3]
        ppm = (rng.next() % 50001) - 25000
        ph = rng.next() & 0xFFFF
        words = eif_words(a, d) if eif else abf_words(a, d)
        smp = sample(symbols([words], logic, 60), lvl, tone, ppm, ph)
        for i in range(len(smp)):
            if rng.next() % 100 < 2:
                smp[i] ^= 1
        buf, n = pack(smp)
        readings += scan_samples(buf, n, spb, pol, inv, 12)
    return readings


# Scanner outputs that test_decode.c must reproduce exactly: equal counts and
# digests here and there mean the two ports agree, on clean frames, on noisy
# ones, and on pure noise scanned ungated (which walks every reject path).
# After an intentional algorithm change: --lockstep, and paste into both files.
LOCKSTEP_SEED = 0x1234ABCD
LOCKSTEP_NSAMP = 100000
LOCKSTEP_TONES = (521, 1042, 1200, 1300, 1400, 2400)
LOCKSTEP = {
    # (case, tone2 Hz): (readings, digest)
    ('clean', 521): (42, 0x312CA2E8),
    ('clean', 1042): (48, 0x9E5807E8),
    ('clean', 1200): (48, 0x67C9CF64),
    ('clean', 1300): (48, 0x003B09E0),
    ('clean', 1400): (48, 0x38EC60E0),
    ('clean', 2400): (48, 0x73FBE050),
    ('noise', 521): (7, 0x0FDDD241),
    ('noise', 1042): (2, 0x8A64074D),
    ('noise', 1200): (4, 0xA130FB26),
    ('noise', 1300): (8, 0x8E2ADEDF),
    ('noise', 1400): (0, 0x00000000),
    ('noise', 2400): (0, 0x00000000),
    ('noisy', 521): (8, 0x7BD3C742),
    ('noisy', 1042): (15, 0xC7FB834C),
    ('noisy', 1200): (16, 0x40CE0BDC),
    ('noisy', 1300): (14, 0xD3B8FCA8),
    ('noisy', 1400): (16, 0x4FEE737D),
    ('noisy', 2400): (16, 0x42E23BC5),
}


def lockstep_cases():
    res = {}
    buf, n = noise_buf(LOCKSTEP_SEED, LOCKSTEP_NSAMP)
    for tone in LOCKSTEP_TONES:
        _, r = clean_frames(tone)
        res[('clean', tone)] = (len(r), digest(r))
        r = noisy_frames(tone)
        res[('noisy', tone)] = (len(r), digest(r))
        r = (scan_samples(buf, n, spb_q8_for(tone), POL_ANY, 0, 0, 64) +
             scan_samples(buf, n, spb_q8_for(tone), POL_ANY, 1, 0, 64))
        res[('noise', tone)] = (len(r), digest(r))
    return res


def selftest(robust_n):
    fails = []

    def check(label, cond):
        if not cond:
            fails.append(label)

    # 1. every k, rate error, sense/framing combination and phase, noise-free.
    #    Below 2 samples per bit a word is timed from one or two samples per
    #    bit and some phases are ambiguous; see the robustness table.
    for tone in TONES:
        spb = spb_q8_for(tone)
        ok, _ = clean_frames(tone)
        total = len(PPMS) * len(COMBOS) * len(PHASES)
        need = total if spb >= SCAN_MED_MIN_Q8 else total * 3 // 4
        check('clean frames at %d Hz: %d/%d' % (tone, ok, total), ok >= need)
        print('clean frames      k=%.2f  %2d/%2d decode' % (tone / 300, ok, total))

    # 2. noise gives no table hits under the gate
    stations = load_stations()
    buf, n = noise_buf(0xC0FFEE11, 100000)
    for tone in TONES:
        spb = spb_q8_for(tone)
        hits = gated = raw = 0
        for inv in (0, 1):
            g = scan_samples(buf, n, spb, POL_ANY, inv, 12, 64)
            gated += len(g)
            hits += sum(1 for x in g if x.id in stations)
            raw += len(scan_samples(buf, n, spb, POL_ANY, inv, 0, 64))
        check('noise table hits at %d Hz' % tone, hits == 0)
        print('noise 1e5 samples k=%.2f  gated %d (table %d), ungated %d'
              % (tone / 300, gated, hits, raw))

    # 3. only 8 idle bits before word 0: rejected at gate 12, accepted at gate 6
    for tone in TONES:
        spb = spb_q8_for(tone)
        rej = acc = 0
        for logic, lvl, pol, inv in COMBOS:
            for ph in PHASES:
                s = symbols([abf_words(705, 123)], logic, 8, lead_mark_bits=3)
                buf, n = pack(sample(s, lvl, tone, 0, ph))
                rej += len(scan_samples(buf, n, spb, pol, inv, 12)) == 0
                r = scan_samples(buf, n, spb, pol, inv, 6)
                acc += len(r) == 1 and r[0].id == 705
        check('8-bit preamble rejected at %d Hz' % tone, rej == 16)
        check('8-bit preamble accepted by gate 6 at %d Hz' % tone, acc == 16)

    # 4. a second frame 10 idle bits after an accepted one is exempt from the
    #    gate; after a frame that did not decode it is not. The failed frame's
    #    words carry all start tone, so only stop + gap = 9 idle bits precede.
    for tone in TONES:
        spb = spb_q8_for(tone)
        both = orphan = 0
        for logic, lvl, pol, inv in COMBOS:
            busy = [0xFF] * 4 if logic == LOGIC_NEG else [0x00] * 4
            for ph in PHASES:
                s = symbols([abf_words(705, 123), eif_words(706, 456)], logic, 60, gap_bits=10)
                buf, n = pack(sample(s, lvl, tone, 0, ph))
                r = scan_samples(buf, n, spb, pol, inv, 12)
                both += [(x.id, x.value) for x in r] == [(705, 123), (706, 456)]
                s = symbols([busy, eif_words(706, 456)], logic, 60, gap_bits=8)
                buf, n = pack(sample(s, lvl, tone, 0, ph))
                orphan += len(scan_samples(buf, n, spb, pol, inv, 12)) == 0
        check('back-to-back frames at %d Hz' % tone, both == 16)
        check('no exemption after a failed frame at %d Hz' % tone, orphan == 16)
    check('busy words do not decode', decode_payload32(assemble([0xFF] * 4)) is None)

    # 5. the gated 1x scanner: gate 0 is the plain scanner; the gate rejects a
    #    short lead; a frame right behind an accepted one is exempt. Under
    #    standard logic frame A's last data bits may themselves be idle, so
    #    B's own idle run is counted from the symbols rather than assumed.
    rnd = random.Random(7)
    for _ in range(100):
        a, d = rnd.randrange(8192), rnd.randrange(2048)
        for logic, lvl, pol, inv in COMBOS:
            busy = [0xFF] * 4 if logic == LOGIC_NEG else [0x00] * 4
            lead = rnd.choice((3, 8, 11, 12, 20))
            s = symbols([abf_words(a, d), eif_words(a ^ 1, d)], logic, lead, gap_bits=4)
            b_at = lead + 40 + 4
            run_b = 0
            while s[b_at - 1 - run_b] == 0:
                run_b += 1
            want = [a] if lead >= 12 else []
            if want or run_b >= 12:
                want.append(a ^ 1)
            buf, n = pack([x ^ lvl for x in s])
            g0 = scan_bits_gated(buf, n, pol, 20, inv, 0)
            g12 = scan_bits_gated(buf, n, pol, 20, inv, 12)
            check('gated 1x, gate 0 finds both', [x.id for x in g0] == [a, a ^ 1])
            check('gated 1x, gate 12, lead %d' % lead, [x.id for x in g12] == want)
            s = symbols([busy, eif_words(a ^ 1, d)], logic, 20, gap_bits=4)
            buf, n = pack([x ^ lvl for x in s])
            check('gated 1x, no exemption after a failed frame',
                  scan_bits_gated(buf, n, pol, 20, inv, 12) == [])
    ref = [(r.id, r.value, r.format, r.polarity, r.bit_pos)
           for r in scan_bits_gated(bytes([0, 0, 0, 0xC7, 0xBF, 0x6F, 0xC3, 0xC0, 0, 0]), 75,
                                    POL_NEGATIVE, 20, 0, 12)]
    check('gated 1x on the test_decode.c vector', ref == [(6129, 1599, FMT_EIF, POL_NEGATIVE, 24)])

    # 6. robustness with a float model: random phase, Gaussian edge jitter
    #    (fraction of a bit), random sample flips. Reported; floors asserted.
    print('\nfraction decoded, %d random ABF frames per cell (NEG, inv 0, gate 12)' % robust_n)
    print('  k      rate  clean  jit.10  jit.10+1%flip')
    for tone in TONES:
        spb = spb_q8_for(tone)
        for e in (-0.02, 0.0, 0.02):
            cells = [_robust(tone, spb, e, jit, flip, robust_n, rnd)
                     for jit, flip in ((0.0, 0.0), (0.10, 0.0), (0.10, 0.01))]
            print('  %4.2f  %+3.0f%%   %4.2f   %4.2f    %4.2f' % (tone / 300, e * 100, *cells))
            if spb >= 3 * 256:
                check('robust %d Hz %+.0f%% clean' % (tone, e * 100), cells[0] >= 0.99)
                check('robust %d Hz %+.0f%% jitter' % (tone, e * 100), cells[1] >= 0.85)
            else:
                check('robust %d Hz %+.0f%% clean' % (tone, e * 100), cells[0] >= 0.80)

    # 7. the constants test_decode.c checks
    got = lockstep_cases()
    check('lockstep table matches (run --lockstep)', got == LOCKSTEP)

    print()
    for f in fails:
        print('FAIL', f)
    print('%s (%d failure%s)' % ('FAILED' if fails else 'all tests passed',
                                 len(fails), '' if len(fails) == 1 else 's'))
    return not fails


def _robust(tone, spb, e, jit, flip, n, rnd):
    ok = 0
    fs = float(tone)
    baud = 300.0 * (1 + e)
    for _ in range(n):
        a, d = rnd.randrange(8192), rnd.randrange(2048)
        s = symbols([abf_words(a, d)], LOGIC_NEG, 60)
        edges = [0.0] + [j / baud + rnd.gauss(0, jit / baud) for j in range(1, len(s))]
        t = rnd.random() / fs
        end = len(s) / baud
        out = []
        j = 0
        while t < end:
            while j + 1 < len(s) and t >= edges[j + 1]:
                j += 1
            b = s[j]
            if flip and rnd.random() < flip:
                b ^= 1
            out.append(b)
            t += 1 / fs
        buf, ns = pack(out)
        r = scan_samples(buf, ns, spb, POL_NEGATIVE, 0, 12)
        ok += any(x.id == a and x.value == d for x in r)
    return ok / n


if __name__ == '__main__':
    if '--lockstep' in sys.argv:
        for k, v in sorted(lockstep_cases().items()):
            print('    %r: (%d, 0x%08X),' % (k, v[0], v[1]))
        sys.exit(0)
    sys.exit(0 if selftest(int(os.environ.get('ROBUST_N', '100'))) else 1)
