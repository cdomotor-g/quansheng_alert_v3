#!/usr/bin/env python3
"""Compare two sets of captures and say whether the difference is real.

Built for one specific question: with the AF selector unmuted, do the bits the
FSK engine produces stop looking like noise? Eyeballing transition densities
cannot answer that on three or four captures, which is exactly how this project
has talked itself into findings before.

The test. In a window of n bits there are n-1 adjacent pairs. If the engine is
emitting independent coin flips, each pair differs with probability 1/2, so the
transition count is Binomial(n-1, 1/2). A real 300-baud burst read at its own
rate runs at about 0.22, far below that, because bits come in runs. So the
one-sided question "is the transition count lower than chance" has an exact
binomial p-value, and counts pool across captures.

Windowing matters. A burst lasts at most ~500 ms, or 150 bits at 300 baud, and
a capture ended by the 1.5 s watchdog is mostly the engine free-running on
noise afterwards. The opening window is where the signal is, so that is what
gets tested; the tail is reported beside it as an internal control, since it
should look like noise under either condition.

    python tools/alert/compare_captures.py before.log after.log
    python tools/alert/compare_captures.py --hex-a AABB,CCDD after.log

Names are only labels. Nothing here assumes which set is which.
"""
import math
import os
import re
import sys

A_LINE = re.compile(r'^A\s+(\d+)\s+(\d+)\s+(-?\d+)\s+([0-9A-Fa-f]+)\s*$')

WINDOW = 150      # bits: the longest a burst can be at 300 baud
NOISE_P = 0.5     # transition probability for independent bits


def bits_of(hexstr, nbits=None):
    b = bytes.fromhex(hexstr)
    n = len(b) * 8 if nbits is None else min(nbits, len(b) * 8)
    return [(b[i >> 3] >> (7 - (i & 7))) & 1 for i in range(n)]


def load(arg):
    """a log path, or comma-separated hex strings"""
    caps = []
    if os.path.exists(arg):
        with open(arg, 'r', errors='replace') as f:
            for ln in f:
                m = A_LINE.match(ln.strip())
                if m:
                    caps.append((int(m.group(1)), int(m.group(3)), m.group(4)))
    else:
        for h in arg.split(','):
            h = h.strip()
            if h:
                caps.append((len(h) * 4, None, h))
    return caps


def transitions(bits):
    return sum(1 for i in range(1, len(bits)) if bits[i] != bits[i - 1])


def binom_sf_le(k, n, p):
    """P(X <= k) for X ~ Binomial(n, p). Exact; n here is at most a few thousand."""
    if n <= 0:
        return 1.0
    tot = 0.0
    for i in range(0, k + 1):
        tot += math.comb(n, i) * (p ** i) * ((1 - p) ** (n - i))
    return min(1.0, tot)


def describe(caps, label, window=WINDOW):
    print("=== %s: %d capture(s) ===" % (label, len(caps)))
    if not caps:
        print("   (none)")
        print()
        return None
    print("%-5s %6s %7s %9s %9s" % ("#", "rssi", "bits", "open", "tail"))
    open_t = open_n = tail_t = tail_n = 0
    for i, (nbits, rssi, hx) in enumerate(caps, 1):
        b = bits_of(hx, nbits)
        o = b[:window]
        t = b[window:]
        od = transitions(o) / max(1, len(o) - 1) if len(o) > 1 else float('nan')
        td = transitions(t) / max(1, len(t) - 1) if len(t) > 1 else float('nan')
        open_t += transitions(o); open_n += max(0, len(o) - 1)
        tail_t += transitions(t); tail_n += max(0, len(t) - 1)
        print("%-5d %6s %7d %9.3f %9s"
              % (i, rssi if rssi is not None else '-', len(b), od,
                 "%.3f" % td if len(t) > 1 else '-'))
    print()
    print("pooled opening window: %d transitions in %d pairs -> %.4f"
          % (open_t, open_n, open_t / open_n if open_n else float('nan')))
    if tail_n:
        print("pooled tail (control): %d transitions in %d pairs -> %.4f"
              % (tail_t, tail_n, tail_t / tail_n))
    p = binom_sf_le(open_t, open_n, NOISE_P)
    print("one-sided p(transitions this low or lower | independent bits) = %.4g" % p)
    print()
    return (open_t, open_n, p)


def main(argv):
    hexa = None
    if argv and argv[0] == '--hex-a':
        hexa = argv[1]
        argv = argv[2:]
    if hexa is None and len(argv) < 2:
        print(__doc__)
        return 2
    a = load(hexa) if hexa else load(argv[0])
    b = load(argv[-1])
    la = "A (%s)" % ('hex' if hexa else os.path.basename(argv[0]))
    lb = "B (%s)" % os.path.basename(argv[-1])

    ra = describe(a, la)
    rb = describe(b, lb)

    print("=== verdict ===")
    print("Reference points: independent bits give 0.500, a real 300-baud burst")
    print("read at its own rate gives about 0.22.")
    print()
    if not ra or not rb:
        print("Need captures in both sets to compare. One set is empty, which is")
        print("itself the result: that configuration produced no bits.")
        return 0
    for lbl, r in ((la, ra), (lb, rb)):
        v = "consistent with noise" if r[2] > 0.01 else "BELOW chance, p=%.3g" % r[2]
        print("  %-22s opening %.4f   %s" % (lbl, r[0] / r[1], v))
    print()
    # difference between the two pooled proportions, two-proportion z test
    p1, n1 = ra[0] / ra[1], ra[1]
    p2, n2 = rb[0] / rb[1], rb[1]
    pool = (ra[0] + rb[0]) / (n1 + n2)
    se = math.sqrt(pool * (1 - pool) * (1 / n1 + 1 / n2))
    if se > 0:
        z = (p1 - p2) / se
        print("  difference %.4f, z = %.2f" % (p1 - p2, z))
        if abs(z) < 2:
            print("  Not significant. The two sets are not distinguishable, so")
            print("  unmuting the AF changed nothing measurable here.")
        else:
            print("  Significant at |z|>2. The two configurations differ.")
    print()
    print("A p-value near 1 with a pooled figure near 0.500 means the engine is")
    print("emitting noise, whatever the setting. That points away from the AF")
    print("path and towards the tone detector simply not matching this signal.")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
