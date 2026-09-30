#!/usr/bin/env python3
"""Bit-exact simulation of the firmware's integer AFSK correlator.

Answers, without any hardware, the question that has never been tested:
given the REAL ALERT tones (V.23: mark 2100 Hz, space 1300 Hz at 300 baud),
does ALERT_AdcTick + ALERT_ScanBits recover the reading?

Every integer operation below mirrors App/app/alert.c so the answer applies to
the code that will actually run, not to a floating-point idealisation of it.
"""
import math
import random

ADC_FS = 9600

# the firmware's 32-entry int8 sine table, verbatim
SINTAB = [0, 25, 49, 71, 90, 106, 117, 125, 127, 125, 117, 106, 90, 71, 49, 25,
          0, -25, -49, -71, -90, -106, -117, -125, -127, -125, -117, -106, -90, -71, -49, -25]


def s16(v):
    """C (int16_t) truncation."""
    return ((v + 32768) & 0xFFFF) - 32768


def sin256(ph):
    return SINTAB[ph >> 3]


def cos256(ph):
    return SINTAB[((ph + 64) & 0xFF) >> 3]


# ---------------------------------------------------------------------------
# signal generation: how an ALERT station actually transmits

def crc6(bits, nbits):
    """x^6+x^4+x^3+1, MSB first - matches ALERT_Crc6 and MegaNet crc6()."""
    reg = 0
    for i in range(nbits - 1, -1, -1):
        b = (bits >> i) & 1
        fb = ((reg >> 5) & 1) ^ b
        reg = (reg << 1) & 0x3F
        if fb:
            reg ^= 0x19
    return reg


def eif_words(a, d):
    """EIF: A0..A5[1][1] | A6..A12 D0 | D1..D8 | D9 D10 R5..R0"""
    r = crc6(((a << 11) | d), 24)
    w0 = (a & 0x3F) | 0xC0
    w1 = ((a >> 6) & 0x7F) | ((d & 1) << 7)
    w2 = (d >> 1) & 0xFF
    w3 = (d >> 9) & 0x03
    for j in range(6):
        w3 |= ((r >> (5 - j)) & 1) << (2 + j)
    return [w0, w1, w2, w3]


def frame_bits(words, lead=24, trail=12, repeats=1):
    """Negative logic (ALERT_POL_NEGATIVE): idle 0, start 1, 8 data LSB-first, stop 0."""
    bits = []
    for _ in range(repeats):
        bits += [0] * lead
        for w in words:
            bits.append(1)
            bits.extend((w >> i) & 1 for i in range(8))
            bits.append(0)
    bits += [0] * trail
    return bits


def afsk_adc(bits, baud, mark, space, fs=ADC_FS, amp=900, noise=0.0, seed=1):
    """Continuous-phase AFSK rendered as 12-bit ADC counts around mid-rail.

    amp=900 counts keeps x = (raw-dc)>>3 near +-112, well inside the int16
    headroom of the x*sin product (112*127 = 14224).
    """
    rnd = random.Random(seed)
    spb = fs / baud
    out = []
    phase = 0.0
    carry = 0.0
    for b in bits:
        f = mark if b else space
        dph = 2.0 * math.pi * f / fs
        n = int(spb + carry)
        carry = spb + carry - n
        for _ in range(n):
            v = amp * math.sin(phase)
            if noise:
                v += rnd.gauss(0.0, amp * noise)
            s = int(round(2048 + v))
            out.append(0 if s < 0 else (4095 if s > 4095 else s))
            phase += dph
            if phase > 2.0 * math.pi:
                phase -= 2.0 * math.pi
    return out


# ---------------------------------------------------------------------------
# the demodulator, operation for operation as ALERT_AdcTick runs it

def demod(samples, inc1, inc2, corr_w, spb):
    """inc1 is the tone that decides bit 1. Returns the recovered bit list."""
    dc_x16 = 2048 * 16
    ph1 = ph2 = 0
    h1c = [0] * corr_w
    h1s = [0] * corr_w
    h2c = [0] * corr_w
    h2s = [0] * corr_w
    hi = 0
    s1c = s1s = s2c = s2s = 0
    phase = 0
    last_bit = 0
    bits = []

    for raw in samples:
        dc_x16 += ((raw << 4) - dc_x16) >> 7
        x = (raw - (dc_x16 >> 4)) >> 3
        x = max(-256, min(255, x))      # alert_adc.c DemodSample: keeps the int16 products whole

        p1c = s16(x * cos256(ph1))
        p1s = s16(x * sin256(ph1))
        p2c = s16(x * cos256(ph2))
        p2s = s16(x * sin256(ph2))
        ph1 = (ph1 + inc1) & 0xFF
        ph2 = (ph2 + inc2) & 0xFF

        s1c += p1c - h1c[hi]; h1c[hi] = p1c
        s1s += p1s - h1s[hi]; h1s[hi] = p1s
        s2c += p2c - h2c[hi]; h2c[hi] = p2c
        s2s += p2s - h2s[hi]; h2s[hi] = p2s
        hi += 1
        if hi >= corr_w:
            hi = 0

        a1 = s1c >> 6; b1 = s1s >> 6
        a2 = s2c >> 6; b2 = s2s >> 6
        bit = 1 if (a1 * a1 + b1 * b1) > (a2 * a2 + b2 * b2) else 0

        if bit != last_bit:
            half = spb // 2
            if phase < half:
                phase -= phase // 4
            else:
                phase += (spb - phase) // 4
            last_bit = bit

        phase += 1
        if phase >= spb:
            phase = 0
        # alert_adc.c: the bit is taken half a bit after the locked transitions,
        # not at phase 0 (which is the bit boundary)
        if phase == spb // 2:
            bits.append(bit)
    return bits


# ---------------------------------------------------------------------------
# ALERT_ScanBits, ported

def decode_payload32(payload):
    def P(p):
        return (payload >> (31 - p)) & 1

    def lsb_first(p, n):
        v = 0
        for i in range(n):
            v |= P(p + i) << i
        return v

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
            return ('ABF', a, d)
        return None
    if k1 == 3:
        a = lsb_first(0, 6) | (lsb_first(8, 7) << 6)
        d = lsb_first(15, 1) | (lsb_first(16, 8) << 1) | (lsb_first(24, 2) << 9)
        r = msb_first(26, 6)
        if crc6(((a << 11) | d), 24) == r:
            return ('EIF', a, d)
    return None


def scan_bits(bits, polarity=1, max_gap=24):
    """polarity 1 = ALERT_POL_NEGATIVE (idle 0, start 1)."""
    idle = 0 if polarity == 1 else 1
    start = idle ^ 1
    out = []
    words = []
    wpos = []
    pos = 0
    n = len(bits)
    while pos + 10 <= n:
        if bits[pos] != start:
            pos += 1
            continue
        if pos > 0 and bits[pos - 1] != idle:
            pos += 1
            continue
        if bits[pos + 9] != idle:
            pos += 1
            words = []; wpos = []
            continue
        w = 0
        for i in range(8):
            w |= bits[pos + 1 + i] << i
        if words and (pos - (wpos[-1] + 10)) > max_gap:
            words = []; wpos = []
        if len(words) == 4:
            words = words[1:]; wpos = wpos[1:]
        words.append(w); wpos.append(pos)
        pos += 10
        if len(words) == 4:
            payload = 0
            for k in range(4):
                for i in range(8):
                    payload |= ((words[k] >> i) & 1) << (31 - (8 * k + i))
            got = decode_payload32(payload)
            if got:
                out.append(got)
                words = []; wpos = []
    return out


# ---------------------------------------------------------------------------

def inc_for(freq):
    """The phase increment the firmware would compute, rounded."""
    return int((256 * freq + ADC_FS // 2) // ADC_FS) & 0xFF


def actual(inc):
    return inc * ADC_FS / 256.0


def ber(sent, got):
    """Best bit error rate over small alignments AND both senses.

    Reporting only the direct sense is actively misleading here, and cost this
    session an hour: a demodulator tuned to the wrong tone pair can slice these
    two tones perfectly and simply label them the other way round, which scores
    ~0.66 direct and 0.00 inverted. "Garbage" and "clean but inverted" call for
    completely different fixes, so the sense is returned alongside the rate.

    Returns (rate, sense) where sense is 'direct' or 'inverted'.
    """
    if not sent or not got:
        return 1.0, 'direct'
    best = (1.0, 'direct')
    for off in range(0, 8):
        m = min(len(sent), len(got) - off)
        if m <= 0:
            continue
        e = sum(1 for i in range(m) if sent[i] != got[off + i])
        for lbl, err in (('direct', e), ('inverted', m - e)):
            if err / m < best[0]:
                best = (err / m, lbl)
    return best


def run(mark, space, corr_w, baud=300, noise=0.0, sid=6129, val=1599, seed=1):
    bits = frame_bits(eif_words(sid, val), repeats=1)
    samples = afsk_adc(bits, baud, mark, space, noise=noise, seed=seed)
    spb = ADC_FS // baud
    got = demod(samples, inc_for(mark), inc_for(space), corr_w, spb)
    found = scan_bits(got, polarity=1)
    return bits, got, found, ber(bits, got)[0]


def part_windows_and_noise():
    SID, VAL = 6129, 1599
    print("ALERT V.23: mark 2100 Hz = bit 1, space 1300 Hz = bit 0, 300 baud")
    print("phase increments at %d Hz: 2100 -> %d (%.1f Hz), 1300 -> %d (%.1f Hz)"
          % (ADC_FS, inc_for(2100), actual(inc_for(2100)),
             inc_for(1300), actual(inc_for(1300))))
    print()

    print("=== correlator window sweep, clean signal ===")
    print("%6s  %8s  %s" % ("CORR_W", "BER", "decoded"))
    for w in (8, 12, 16, 20, 24, 28, 32):
        _, _, found, b = run(2100.0, 1300.0, w, sid=SID, val=VAL)
        ok = ','.join('%s id=%d value=%d' % f for f in found) or '-'
        print("%6d  %8.4f  %s" % (w, b, ok))
    print()

    print("=== the constants the firmware ships today (Bell 202, inverted) ===")
    for w in (16,):
        _, _, found, b = run(1200.0, 2200.0, w, sid=SID, val=VAL)
        ok = ','.join('%s id=%d value=%d' % f for f in found) or '-'
        print("mark=1200 space=2200 CORR_W=%d  BER=%.4f  decoded: %s" % (w, b, ok))
    print()

    print("=== noise tolerance of the best window ===")
    best_w = 16
    print("%8s  %8s  %s" % ("noise", "BER", "decoded"))
    for nz in (0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0):
        hits = 0
        bsum = 0.0
        trials = 20
        for s in range(trials):
            _, _, found, b = run(2100.0, 1300.0, best_w, noise=nz, sid=SID, val=VAL, seed=s + 1)
            bsum += b
            if any(f[1] == SID and f[2] == VAL for f in found):
                hits += 1
        print("%8.2f  %8.4f  %d/%d" % (nz, bsum / trials, hits, trials))


# ---------------------------------------------------------------------------
# the tests that matter: transmit tones and receive tones set independently

def run2(tx_one, tx_zero, rx_one, rx_zero, corr_w=16, baud=300,
         noise=0.0, sid=6129, val=1599, seed=1, pol=1):
    """tx_one/tx_zero: tones the station puts on the air for decoder bits 1/0.
       rx_one/rx_zero: tones the demodulator assigns to bits 1/0."""
    bits = frame_bits(eif_words(sid, val), repeats=1)
    samples = afsk_adc(bits, baud, tx_one, tx_zero, noise=noise, seed=seed)
    spb = ADC_FS // baud
    got = demod(samples, inc_for(rx_one), inc_for(rx_zero), corr_w, spb)
    found = scan_bits(got, polarity=pol)
    return bits, got, found, ber(bits, got)


def show(label, **kw):
    sent, got, found, b = run2(**kw)
    rate, sense = b if isinstance(b, tuple) else (b, 'direct')
    ok = ', '.join('%s id=%d value=%d' % f for f in found) or 'NOTHING'
    print("%-46s BER=%.4f %-8s %s" % (label, rate, sense, ok))
    return found


def part_tone_mismatch():
    print()
    print("=== THE REAL TEST: air is 2100/1300, demodulator is what ships ===")
    show("air 2100/1300, rx 1200/2200 (shipped constants)",
         tx_one=2100.0, tx_zero=1300.0, rx_one=1200.0, rx_zero=2200.0)
    show("air 2100/1300, rx 2100/1300 (corrected)",
         tx_one=2100.0, tx_zero=1300.0, rx_one=2100.0, rx_zero=1300.0)
    print()

    print("=== does the mark/space sense matter, given the polarity search? ===")
    show("tones swapped at rx, decoder polarity NEGATIVE",
         tx_one=2100.0, tx_zero=1300.0, rx_one=1300.0, rx_zero=2100.0, pol=1)
    show("tones swapped at rx, decoder polarity STANDARD",
         tx_one=2100.0, tx_zero=1300.0, rx_one=1300.0, rx_zero=2100.0, pol=0)
    print()

    print("=== if the air is the other sense (1300 on bit 1) ===")
    show("air 1300/2100, rx 2100/1300, polarity NEGATIVE",
         tx_one=1300.0, tx_zero=2100.0, rx_one=2100.0, rx_zero=1300.0, pol=1)
    show("air 1300/2100, rx 2100/1300, polarity STANDARD",
         tx_one=1300.0, tx_zero=2100.0, rx_one=2100.0, rx_zero=1300.0, pol=0)
    print()

    print("=== tolerance to the station being off frequency ===")
    for err in (0, 25, 50, 75, 100, 150, 200):
        show("air mark %d / space %d, rx 2100/1300" % (2100 + err, 1300 + err),
             tx_one=2100.0 + err, tx_zero=1300.0 + err,
             rx_one=2100.0, rx_zero=1300.0)


if __name__ == '__main__':
    part_windows_and_noise()
    part_tone_mismatch()
