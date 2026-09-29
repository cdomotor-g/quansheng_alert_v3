#!/usr/bin/env python3
"""Decide whether the radio's captures are signal or noise, and say why.

The app streams every capture to the USB port as

    A <nbits> <gated> <rssi> <hex...>

Point this at a log of that stream. It answers three questions, in the order
that changes what you do next:

 1. Do these bits carry any structure, or is the FSK engine emitting noise?
    Measured against a random baseline on the same number of bits, using ones
    density, transition density, longest run, the run-length histogram and
    autocorrelation. Noise and signal look nothing alike on these, but "looks
    structured" is easy to talk yourself into by eye, which is why the baseline
    is printed next to the capture every time.

 2. If there is structure, is it at the air rate or a multiple of it? The
    BK4819's FFSK mode is natively 1200 baud. If TONE2 does not really set the
    receive bit clock, a 300-baud signal would arrive 4x oversampled and every
    run length would be a multiple of 4.

 3. Does anything decode, under any of the four framing x data-sense
    combinations, at any decimation?

On question 3 read the control line before you believe a hit. The search tries
~84 paths per capture, and ABF and EIF each constrain only about 8 bits, so
roughly a quarter of pure-noise captures yield at least one apparent decode.
A real one repeats: the same address at the same settings across several
bursts. A single hit at an odd decimation is chance, and has misled this
project before.

    python tools/alert/capture_stats.py com5.log
    python tools/alert/capture_stats.py --hex 04A52F16...
"""
import os
import random
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alertmon as m

A_LINE = re.compile(r'^A\s+(\d+)\s+(\d+)\s+(-?\d+)\s+([0-9A-Fa-f]+)\s*$')


def bits_of(hexstr, nbits=None):
    b = bytes.fromhex(hexstr)
    n = len(b) * 8 if nbits is None else min(nbits, len(b) * 8)
    return [(b[i >> 3] >> (7 - (i & 7))) & 1 for i in range(n)]


def runs(bits):
    out, cur = [], 1
    for i in range(1, len(bits)):
        if bits[i] == bits[i - 1]:
            cur += 1
        else:
            out.append(cur)
            cur = 1
    out.append(cur)
    return out


def stats_line(bits):
    t = sum(1 for i in range(1, len(bits)) if bits[i] != bits[i - 1])
    return (len(bits), sum(bits) / len(bits), t / max(1, len(bits) - 1), max(runs(bits)))


def autocorr_max(bits, maxlag=24):
    x = [1 if b else -1 for b in bits]
    best = (0, 0.0)
    for lag in range(1, maxlag + 1):
        n = len(x) - lag
        if n < 8:
            break
        r = sum(x[i] * x[i + lag] for i in range(n)) / n
        if abs(r) > abs(best[1]):
            best = (lag, r)
    return best


def scan_all(bits):
    """every combination the decoder can reach: 2 framings x 2 data senses"""
    hits = []
    for pol in ('NEG', 'STD'):
        for inv in (0, 1):
            b = bytearray((len(bits) + 7) // 8)
            for i, v in enumerate(bits):
                if v ^ inv:
                    b[i >> 3] |= 0x80 >> (i & 7)
            pv = m.POL_NEGATIVE if pol == 'NEG' else m.POL_STANDARD
            for r, at, _ in m.scan_polarity(bytes(b), len(bits), pv):
                hits.append((pol, inv, r))
    return hits


def search(bits, label, maxdec=6):
    found = []
    for k in range(1, maxdec + 1):
        for ph in range(k):
            d = bits[ph::k]
            if len(d) < 40:
                continue
            for pol, inv, r in scan_all(d):
                found.append((k, ph, pol, inv, r))
                print("    HIT  %s dec=%d ph=%d %s inv=%d -> %s"
                      % (label, k, ph, pol, inv, r))
    return found


def noise_control(nbits, trials=120, maxdec=6, seed=4242):
    """what fraction of pure-noise captures of this length appear to decode"""
    rnd = random.Random(seed)
    hit = 0
    for _ in range(trials):
        bits = [rnd.randint(0, 1) for _ in range(nbits)]
        got = False
        for k in range(1, maxdec + 1):
            for ph in range(k):
                d = bits[ph::k]
                if len(d) >= 40 and scan_all(d):
                    got = True
                    break
            if got:
                break
        if got:
            hit += 1
    return hit / trials


def main(argv):
    caps = []
    if len(argv) >= 2 and argv[0] == '--hex':
        caps = [('hex', None, None, argv[1])]
    elif argv:
        with open(argv[0], 'r', errors='replace') as f:
            for ln in f:
                mm = A_LINE.match(ln.strip())
                if mm:
                    caps.append(('cap%d' % (len(caps) + 1),
                                 int(mm.group(1)), int(mm.group(3)), mm.group(4)))
    else:
        print(__doc__)
        return 2

    if not caps:
        print("no 'A <nbits> <gated> <rssi> <hex>' lines found")
        return 1

    print("=== %d capture(s) ===" % len(caps))
    print("%-7s %6s %6s %9s %10s %7s %s"
          % ("", "bits", "rssi", "ones/bit", "trans/bit", "maxrun", "autocorr"))
    lengths = set()
    for name, nbits, rssi, hx in caps:
        b = bits_of(hx, nbits)
        lengths.add(len(b))
        n, ones, tr, mx = stats_line(b)
        lag, r = autocorr_max(b)
        print("%-7s %6d %6s %9.3f %10.3f %7d  max|r|=%.3f at lag %d"
              % (name, n, rssi if rssi is not None else '-', ones, tr, mx, abs(r), lag))

    nb = max(lengths)
    rnd = random.Random(7)
    print()
    print("--- baselines at %d bits, for comparison ---" % nb)
    for k in range(3):
        n, ones, tr, mx = stats_line([rnd.randint(0, 1) for _ in range(nb)])
        print("%-7s %6d %6s %9.3f %10.3f %7d" % ("random", n, '-', ones, tr, mx))
    src = [0] * 24
    for w in (0xC0, 0x40, 0x80, 0x01):
        src += [1] + [(w >> i) & 1 for i in range(8)] + [0]
    for mult, lbl in ((1, "300@300"), (2, "300@600"), (4, "300@1200")):
        b = [x for x in src for _ in range(mult)]
        n, ones, tr, mx = stats_line(b)
        print("%-7s %6d %6s %9.3f %10.3f %7d" % (lbl, n, '-', ones, tr, mx))
    print()
    print("A real burst read at its own rate has a transition density well")
    print("below 0.4 and long runs; read 4x oversampled, below 0.1. Noise sits")
    print("at 0.5 with a geometric run histogram. Judge by that, not by eye.")
    print()

    print("=== windowed: only the start of a capture holds the burst ===")
    print("A burst lasts at most ~500 ms. The chip waits for cfg.pktlen bytes")
    print("before raising RX_FINISHED - 32 bytes is 853 ms at 300 baud - so the")
    print("watchdog ends the capture at 1.5 s and the tail is the engine")
    print("free-running on noise after the signal stopped. Judging the whole")
    print("capture dilutes whatever signal the opening holds. Set CAPTURE to 16")
    print("bytes or less so a capture is mostly burst.")
    print()
    print("at 300 baud: 133 ms = 40 bits, 350 ms = 105, 500 ms = 150")
    print()
    print("%-7s %-11s %6s %10s %7s" % ("", "window", "bits", "trans/bit", "maxrun"))
    for name, nbits, rssi, hx in caps:
        b = bits_of(hx, nbits)
        for lo, hi, lbl in ((0, 40, "first 40"), (0, 64, "first 64"),
                            (0, 105, "first 105"), (0, 150, "first 150"),
                            (150, None, "after 150"), (0, None, "all")):
            w = b[lo:hi] if hi else b[lo:]
            if len(w) < 8:
                continue
            _, _, tr, mx = stats_line(w)
            print("%-7s %-11s %6d %10.3f %7d" % (name, lbl, len(w), tr, mx))
        print()
    print("Signal reads LOW here (~0.22), noise ~0.50. On a 40-bit window the")
    print("standard error is about 0.08, so anything inside 0.42-0.58 is chance.")
    print("Several captures agreeing in the same direction is the evidence; one")
    print("window on one capture is not.")
    print()

    print("=== run-length histogram ===")
    for name, nbits, rssi, hx in caps:
        r = runs(bits_of(hx, nbits))
        h = {}
        for v in r:
            h[v] = h.get(v, 0) + 1
        print("%-7s n=%3d mean=%.2f  %s"
              % (name, len(r), statistics.mean(r),
                 ' '.join('%d:%d' % (k, h[k]) for k in sorted(h))))
    print("(a 4x oversampled signal makes every run a multiple of 4)")
    print()

    print("=== decode search: 4 combinations x decimation 1..6 x every phase ===")
    total = []
    for name, nbits, rssi, hx in caps:
        total += search(bits_of(hx, nbits), name)
    if not total:
        print("    nothing decoded under any combination")
    print()
    fp = noise_control(nb)
    print("=== control ===")
    print("pure noise of the same length through the same search:")
    print("  %.0f%% of captures yield at least one apparent decode" % (100 * fp))
    if total:
        print("  this run found %d hit(s) across %d capture(s) - treat as chance"
              % (len(total), len(caps)))
        print("  unless the same address recurs at the same settings.")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
