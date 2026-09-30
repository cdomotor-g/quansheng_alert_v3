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

ALERT-X1 logs carry each burst's window as H lines instead,

    H <idx> <burst#> <byte offset> <hex>

oversampled at k = TONE2/300 samples per bit. k is taken from the L line
that set up arrangement <idx> (or the burst's C line), windows scale by k, and
the decode search decimates by k alone at every phase, fractional k included,
rather than trying 1..6. sweep_judge.py says which in-byte bit order passes
STRUCT; --lsb reads the stored bytes LSB-first to match. --arr keeps one
arrangement's bursts. Timestamps from radio.py or alertmon.py are skipped.

    python tools/alert/capture_stats.py com5.log
    python tools/alert/capture_stats.py --hex 04A52F16...
    python tools/alert/capture_stats.py [--lsb] [--arr 8] tools/alert/logs/x1-*.log
"""
import math
import os
import random
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alertmon as m

A_LINE = re.compile(r'^A\s+(\d+)\s+(\d+)\s+(-?\d+)\s+([0-9A-Fa-f]+)\s*$')
H_LINE = re.compile(r'^H\s+(\S+)\s+(\d+)\s+(\d+)\s+([0-9A-Fa-f]*)\s*$')
STAMP = re.compile(r'^(?:\d{4}-\d\d-\d\d[T ]\d\d:\d\d:\d\d(?:\.\d+)?|\[[\d:.]+\])\s+')
REV = bytes(int('{:08b}'.format(i)[::-1], 2) for i in range(256))


def kv(tokens):
    return dict(t.split('=', 1) for t in tokens if '=' in t)


def read_log(path, lsb=False, only_arr=None):
    """A lines as they were; H lines stitched per burst, with k from L (or C).
    Each capture: (name, nbits, rssi, hex MSB-first, k or None)."""
    caps = []
    t2_of = {}          # idx -> TONE2 from the latest L line
    bursts = {}         # (session, idx, burst#) -> [chunks, n, rssi, k]
    order = []
    sess = 0
    with open(path, 'r', errors='replace') as f:
        for ln in f:
            ln = STAMP.sub('', ln.strip())
            mm = A_LINE.match(ln)
            if mm:
                caps.append(('cap%d' % (len(caps) + 1), int(mm.group(1)), int(mm.group(3)),
                             mm.group(4), None))
                continue
            tok = ln.split()
            if not tok:
                continue
            try:
                if tok[0] == 'B':
                    sess += 1
                elif tok[0] == 'L':
                    t2_of[tok[1]] = int(kv(tok[3:])['t2'])
                elif tok[0] in ('C', 'H'):
                    if tok[0] == 'C':
                        f_ = kv(tok[2:])
                        key = (sess, tok[1], int(f_['b']))
                    else:
                        mh = H_LINE.match(ln)
                        if not mh:
                            continue
                        key = (sess, tok[1], int(mh.group(2)))
                    if key not in bursts:
                        t2 = t2_of.get(tok[1])
                        bursts[key] = [{}, None, None, t2 / 300.0 if t2 else None]
                        order.append(key)
                    b = bursts[key]
                    if tok[0] == 'C':
                        b[1] = int(f_['n']) if 'n' in f_ else None
                        b[2] = int(f_['pk']) if 'pk' in f_ else None
                        if b[3] is None and 'k' in f_:
                            b[3] = int(f_['k']) / 100.0
                    else:
                        b[0][int(mh.group(3))] = bytes.fromhex(mh.group(4))
            except (KeyError, ValueError, IndexError):
                continue        # torn line
    for key in order:
        chunks, n, rssi, k = bursts[key]
        if only_arr is not None and key[1] != only_arr:
            continue
        data = bytearray()
        for off in sorted(chunks):
            if off > len(data):
                break           # a lost H line: stop at the gap
            data[off:off + len(chunks[off])] = chunks[off]
        if not data:
            continue
        if lsb:
            data = bytearray(REV[x] for x in data)
        nbits = min(n, 8 * len(data)) if n else 8 * len(data)
        caps.append(('a%sb%d' % (key[1], key[2]), nbits, rssi, data.hex(), k))
    return caps


def decimate(bits, step, ph):
    """Every step-th sample from ph; step may be fractional (k = TONE2/300)."""
    if step == int(step):
        return bits[ph::int(step)]
    return [bits[int(ph + j * step)] for j in range(int((len(bits) - 1 - ph) / step) + 1)]


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


def steps_for(k, maxdec=6):
    """Decimation steps to try: 1..maxdec for A lines, k itself for H lines."""
    return (k,) if k else tuple(range(1, maxdec + 1))


def search(bits, label, steps):
    found = []
    for k in steps:
        for ph in range(int(math.ceil(k - 1e-9))):
            d = decimate(bits, k, ph)
            if len(d) < 40:
                continue
            for pol, inv, r in scan_all(d):
                found.append((k, ph, pol, inv, r))
                print("    HIT  %s dec=%g ph=%d %s inv=%d -> %s"
                      % (label, k, ph, pol, inv, r))
    return found


def noise_control(nbits, steps, trials=120, seed=4242):
    """what fraction of pure-noise captures of this length appear to decode"""
    rnd = random.Random(seed)
    hit = 0
    for _ in range(trials):
        bits = [rnd.randint(0, 1) for _ in range(nbits)]
        got = False
        for k in steps:
            for ph in range(int(math.ceil(k - 1e-9))):
                d = decimate(bits, k, ph)
                if len(d) >= 40 and scan_all(d):
                    got = True
                    break
            if got:
                break
        if got:
            hit += 1
    return hit / trials


def main(argv):
    lsb = '--lsb' in argv
    argv = [a for a in argv if a != '--lsb']
    only_arr = None
    if '--arr' in argv:
        i = argv.index('--arr')
        only_arr = argv[i + 1] if i + 1 < len(argv) else None
        del argv[i:i + 2]
    caps = []
    if len(argv) >= 2 and argv[0] == '--hex':
        caps = [('hex', None, None, argv[1], None)]
    elif argv:
        for path in argv:
            caps += read_log(path, lsb, only_arr)
    else:
        print(__doc__)
        return 2

    if not caps:
        print("no 'A <nbits> <gated> <rssi> <hex>' or 'H' lines found")
        return 1

    print("=== %d capture(s)%s ===" % (len(caps), ', bytes read LSB-first' if lsb else ''))
    print("%-9s %5s %6s %6s %9s %10s %7s %s"
          % ("", "k", "bits", "rssi", "ones/bit", "trans/bit", "maxrun", "autocorr"))
    lengths = set()
    for name, nbits, rssi, hx, k in caps:
        b = bits_of(hx, nbits)
        lengths.add(len(b))
        n, ones, tr, mx = stats_line(b)
        lag, r = autocorr_max(b, 24 if not k else max(24, int(6 * k)))
        print("%-9s %5s %6d %6s %9.3f %10.3f %7d  max|r|=%.3f at lag %d"
              % (name, '%.2f' % k if k else '1', n, rssi if rssi is not None else '-',
                 ones, tr, mx, abs(r), lag))

    nb = max(lengths)
    rnd = random.Random(7)
    print()
    print("--- baselines at %d bits, for comparison ---" % nb)
    for k in range(3):
        n, ones, tr, mx = stats_line([rnd.randint(0, 1) for _ in range(nb)])
        print("%-9s %5s %6d %6s %9.3f %10.3f %7d" % ("random", '', n, '-', ones, tr, mx))
    src = [0] * 24
    for w in (0xC0, 0x40, 0x80, 0x01):
        src += [1] + [(w >> i) & 1 for i in range(8)] + [0]
    for mult, lbl in ((1, "300@300"), (2, "300@600"), (4, "300@1200")):
        b = [x for x in src for _ in range(mult)]
        n, ones, tr, mx = stats_line(b)
        print("%-9s %5s %6d %6s %9.3f %10.3f %7d" % (lbl, '', n, '-', ones, tr, mx))
    print()
    print("A real burst read at its own rate has a transition density well")
    print("below 0.4 and long runs; read k times oversampled, below 0.4/k. Noise")
    print("sits at 0.5 with a geometric run histogram. Judge by that, not by eye.")
    print()

    print("=== windowed: only the start of a capture holds the burst ===")
    print("An A capture runs to the 1.5 s watchdog, so its tail is the engine")
    print("free-running on noise after the signal stopped; an X1 H window ends")
    print("40 ms after squelch-lost, and may start 100 ms before the open. Either")
    print("way the opening is where a burst would be. Windows are in ALERT bits:")
    print("40 bits = 133 ms at 300 baud, which is 40k samples of an H window.")
    print()
    print("%-9s %-11s %6s %10s %7s" % ("", "window", "bits", "trans/bit", "maxrun"))
    for name, nbits, rssi, hx, k in caps:
        b = bits_of(hx, nbits)
        s = k or 1
        for lo, hi, lbl in ((0, 40, "first 40"), (0, 64, "first 64"),
                            (0, 105, "first 105"), (0, 150, "first 150"),
                            (150, None, "after 150"), (0, None, "all")):
            w = b[int(lo * s):int(hi * s)] if hi else b[int(lo * s):]
            if len(w) < 8:
                continue
            _, _, tr, mx = stats_line(w)
            print("%-9s %-11s %6d %10.3f %7d" % (name, lbl, len(w), tr, mx))
        print()
    print("Signal reads LOW here (~0.22 per bit, ~0.22/k per sample), noise ~0.50.")
    print("On a 40-bit window the standard error is about 0.08, so anything inside")
    print("0.42-0.58 is chance. Several captures agreeing in the same direction is")
    print("the evidence; one window on one capture is not.")
    print()

    print("=== run-length histogram ===")
    for name, nbits, rssi, hx, k in caps:
        r = runs(bits_of(hx, nbits))
        h = {}
        for v in r:
            h[v] = h.get(v, 0) + 1
        print("%-9s n=%3d mean=%.2f  %s"
              % (name, len(r), statistics.mean(r),
                 ' '.join('%d:%d' % (x, h[x]) for x in sorted(h))))
    ks = sorted(set('%.2f' % k for *_, k in caps if k))
    print("(a signal read k times oversampled puts its runs near multiples of k%s)"
          % (': k = ' + ', '.join(ks) if ks else ''))
    print()

    print("=== decode search: 4 combinations x decimation (1..6, or k) x every phase ===")
    total = []
    groups = {}
    for name, nbits, rssi, hx, k in caps:
        b = bits_of(hx, nbits)
        steps = steps_for(k)
        total += search(b, name, steps)
        groups[steps] = max(groups.get(steps, 0), len(b))
    if not total:
        print("    nothing decoded under any combination")
    print()
    print("=== control ===")
    print("pure noise of the same length through the same search (ungated):")
    for steps, n in sorted(groups.items()):
        fp = noise_control(n, steps)
        print("  decimation %s, %d bits: %.0f%% of captures yield at least one apparent"
              " decode" % ('/'.join('%g' % s for s in steps), n, 100 * fp))
    if total:
        print("  this run found %d hit(s) across %d capture(s) - treat as chance"
              % (len(total), len(caps)))
        print("  unless the same address recurs at the same settings.")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
