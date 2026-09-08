#!/usr/bin/env python3
"""Round-trip alertmon's decoder port against synthetic frames.

alertmon.py re-decodes every capture the radio reports, and its verdict is
used to tell "the bits are not there" from "the firmware decoder is wrong".
That verdict is only worth anything if the port itself is right, so build
frames from known id/value pairs, serialise them the way the transmitter
would, and check they come back.

    python tools/alert/test_alertmon.py
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alertmon as m


def build_eif(a, d):
    r = m.crc6(((a << 11) | d), 24)
    w0 = (a & 0x3F) | 0xC0                       # k1 = 11 marks EIF
    w1 = ((a >> 6) & 0x7F) | ((d & 1) << 7)
    w2 = (d >> 1) & 0xFF
    w3 = (d >> 9) & 0x03
    for j in range(6):
        w3 |= ((r >> (5 - j)) & 1) << (2 + j)
    return [w0, w1, w2, w3]


def build_abf(a, d):
    w0 = (a & 0x3F) | 0x40                       # k1 = 10
    w1 = ((a >> 6) & 0x3F) | 0x40                # k2 = 10
    w2 = ((a >> 12) & 1) | ((d & 0x1F) << 1) | 0xC0
    w3 = ((d >> 5) & 0x3F) | 0xC0
    return [w0, w1, w2, w3]


def frame(words, polarity, lead=7, trail=6):
    """start bit + 8 data bits LSB first + stop bit, per word, idle padded."""
    idle = 0 if polarity == m.POL_NEGATIVE else 1
    bits = [idle] * lead
    for w in words:
        bits.append(idle ^ 1)
        bits.extend((w >> i) & 1 for i in range(8))
        bits.append(idle)
    bits += [idle] * trail
    buf = bytearray((len(bits) + 7) // 8)
    for i, b in enumerate(bits):
        if b:
            buf[i >> 3] |= 0x80 >> (i & 7)
    return bytes(buf), len(bits)


def main():
    failures = []

    def check(label, words, want, pol):
        buf, n = frame(words, pol)
        hits = m.scan_all(buf, n)
        ok = any(h[0] == want for h in hits)
        pols = 'NEG' if pol == m.POL_NEGATIVE else 'STD'
        print('%-26s %-4s %s' % (label, pols, 'PASS' if ok else 'FAIL %s' % (hits,)))
        if not ok:
            failures.append(label + ' ' + pols)

    for pol in (m.POL_NEGATIVE, m.POL_STANDARD):
        check('EIF id=1234 value=567', build_eif(1234, 567), ('EIF', 1234, 567), pol)
        check('EIF id=0 value=0', build_eif(0, 0), ('EIF', 0, 0), pol)
        check('EIF id=8191 value=2047', build_eif(8191, 2047), ('EIF', 8191, 2047), pol)
        check('ABF id=4095 value=1000', build_abf(4095, 1000), ('ABF', 4095, 1000), pol)

    # the EIF CRC6 has to actually bite
    w = build_eif(1234, 567)
    w[2] ^= 0x10
    buf, n = frame(w, m.POL_NEGATIVE)
    bad = m.scan_all(buf, n)
    print('%-26s %-4s %s' % ('corrupted EIF rejected', 'NEG',
                             'PASS' if not bad else 'FAIL %s' % (bad,)))
    if bad:
        failures.append('corrupted EIF rejected')

    # Noise should rarely frame. ABF carries no CRC - only four check bits - so
    # it is not zero, which is what the CONFIRM setting (two copies in one
    # burst) is for. Flag it if it gets much worse than that.
    random.seed(7)
    hits = sum(len(m.scan_all(bytes(random.getrandbits(8) for _ in range(32)), 256))
               for _ in range(200))
    print('%-26s %-4s %s (%d in 200 x 256 random bits)' %
          ('false-positive rate', '', 'PASS' if hits <= 10 else 'HIGH', hits))
    if hits > 10:
        failures.append('false-positive rate')

    if failures:
        print('\nFAILED: %s' % ', '.join(failures))
        return 1
    print('\nall alertmon decoder checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
