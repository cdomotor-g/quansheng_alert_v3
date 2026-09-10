#!/usr/bin/env python3
"""Work out what an ALERT burst actually is, from a recording of the audio.

Everything tried so far has assumed the signal: 300 baud, 1200/2200 Hz, a
particular framing. None of that has been measured, and the radio is a poor
instrument for measuring it - the BK4819's FSK engine produces bits on this
waveform that are statistically indistinguishable from noise, so it cannot tell
us whether our assumptions are wrong or merely unlucky.

A recording of the receiver's audio settles it directly. Feed this a WAV of one
burst - earphone socket into a sound card, or even a phone held to the speaker,
since the tones survive lossy recording - and it reports:

  * the tones actually present, by frequency, rather than the pair we assumed
  * the symbol rate, measured from the shortest repeated symbol
  * a demodulation at those measured parameters, run through the same decoder
    the firmware uses (tools/alert/alertmon.py), in both polarities

    python tools/alert/wavdecode.py burst.wav
    python tools/alert/wavdecode.py burst.wav --mark 1200 --space 2200 --baud 300

Check it against a known answer first - a tool that has only ever been run on
unknown data cannot be trusted when it says "no ALERT frame":

    python tools/alert/make_test_wav.py t.wav --id 1234 --value 567
    python tools/alert/wavdecode.py t.wav

Stereo is mixed to mono; 8- and 16-bit PCM are handled. No numpy needed.
"""
import math
import os
import sys
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alertmon as m


def load_wav(path):
    with wave.open(path, 'rb') as w:
        ch, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width == 1:
        vals = [b - 128 for b in raw]
    elif width == 2:
        vals = [int.from_bytes(raw[i:i + 2], 'little', signed=True)
                for i in range(0, len(raw), 2)]
    else:
        raise SystemExit('need 8- or 16-bit PCM, got %d-byte samples' % width)
    if ch > 1:
        vals = [sum(vals[i:i + ch]) / ch for i in range(0, len(vals) - ch + 1, ch)]
    mean = sum(vals) / len(vals) if vals else 0.0
    return [v - mean for v in vals], rate


def goertzel(block, rate, freq):
    """Magnitude at an arbitrary frequency - no bin alignment required."""
    w = 2.0 * math.pi * freq / rate
    coeff = 2.0 * math.cos(w)
    s1 = s2 = 0.0
    for x in block:
        s0 = x + coeff * s1 - s2
        s2 = s1
        s1 = s0
    p = s1 * s1 + s2 * s2 - coeff * s1 * s2
    return math.sqrt(p) if p > 0 else 0.0


def tone_survey(sig, rate, lo=300, hi=3000, step=25, windows=14):
    """Strongest response at each frequency across the whole burst.

    One window is not enough: an FSK signal spends each symbol on one tone or
    the other, so a window that happens to land on idle reports one tone and
    hides the other. Taking the maximum over windows spread across the
    recording finds both.
    """
    n = min(2048, max(256, len(sig) // windows))
    starts = [int(i * (len(sig) - n) / max(1, windows - 1)) for i in range(windows)]
    best = {f: 0.0 for f in range(lo, hi + 1, step)}
    for a in starts:
        block = sig[a:a + n]
        for f in best:
            v = goertzel(block, rate, f)
            if v > best[f]:
                best[f] = v
    peak = max(best.values()) or 1.0
    return [(f, best[f] / peak) for f in sorted(best)]


def find_peaks(survey, floor=0.35):
    peaks = []
    for i in range(1, len(survey) - 1):
        f, v = survey[i]
        if v >= floor and v >= survey[i - 1][1] and v >= survey[i + 1][1]:
            peaks.append((v, f))
    peaks.sort(reverse=True)
    kept = []
    for v, f in peaks:
        if all(abs(f - g) > 150 for _, g in kept):
            kept.append((v, f))
    return kept


def demod(sig, rate, mark, space, spb):
    """One decision per symbol: is mark or space stronger?

    `spb` is samples per bit and is deliberately a float. Rounding it to a whole
    number costs up to half a sample per bit, which at 22050 Hz and 300 baud
    walks the sampling point clean off the symbol part way through a burst. That
    alone made a synthetic file with a known answer fail at 300 baud and pass at
    302, which is exactly the kind of near miss that reads as "will not decode".
    """
    out = []
    win = max(int(spb), int(rate / max(mark, space) * 3))
    k = 0
    while True:
        i = int(k * spb)
        if i + win >= len(sig):
            break
        out.append(1 if goertzel(sig[i:i + win], rate, mark)
                   > goertzel(sig[i:i + win], rate, space) else 0)
        k += 1
    return out


def to_bytes(bits):
    buf = bytearray((len(bits) + 7) // 8)
    for i, v in enumerate(bits):
        if v:
            buf[i >> 3] |= 0x80 >> (i & 7)
    return bytes(buf)


def try_decode(sig, rate, mark, space, baud):
    b = demod(sig, rate, mark, space, rate / baud)
    buf = to_bytes(b)
    return buf, len(b), m.scan_all(buf, len(b))


def run_lengths(bits):
    runs, r = [], 1
    for i in range(1, len(bits)):
        if bits[i] == bits[i - 1]:
            r += 1
        else:
            runs.append(r)
            r = 1
    runs.append(r)
    return runs


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    path = args[0]
    opt = {}
    for i in range(1, len(args) - 1, 2):
        if args[i].startswith('--'):
            opt[args[i][2:]] = float(args[i + 1])

    sig, rate = load_wav(path)
    dur = len(sig) / rate
    rms = math.sqrt(sum(x * x for x in sig) / len(sig)) if sig else 0.0
    print('%s: %.2f s at %d Hz, RMS %.0f' % (os.path.basename(path), dur, rate, rms))
    if dur < 0.05:
        print('too short to say anything')
        return 1

    print('')
    print('tones present (relative to the strongest):')
    peaks = find_peaks(tone_survey(sig, rate))
    if not peaks:
        print('  nothing stands out - is there actually a burst in this file?')
        return 1
    for v, f in peaks[:6]:
        print('  %4d Hz  %.2f  %s' % (f, v, '#' * int(v * 40)))

    if 'mark' in opt and 'space' in opt:
        mark, space = opt['mark'], opt['space']
        print('')
        print('using the pair given: %g and %g Hz' % (mark, space))
    elif len(peaks) >= 2:
        mark, space = peaks[0][1], peaks[1][1]
        print('')
        print('two strongest tones: %g and %g Hz' % (mark, space))
        print('  (1200/2200 is what the firmware decoder assumes)')
    else:
        print('')
        print('only one tone stands out - this may not be two-tone FSK at all')
        return 1

    fine = max(1.0, rate / 12000.0)
    runs = sorted(run_lengths(demod(sig, rate, mark, space, fine)))
    if len(runs) < 20:
        print('')
        print('too few transitions to measure a symbol rate')
        return 1
    unit = runs[len(runs) // 20]           # 5th percentile: one symbol
    measured = rate / (unit * fine)
    print('')
    print('symbol rate: shortest repeated symbol is %d fine steps -> %.0f baud'
          % (unit, measured))
    print('  %d transitions over %.2f s' % (len(runs), dur))

    # Which tone is mark and which is space is not something a spectrum can
    # tell us - the louder one is simply whichever the transmitter idles on -
    # so try it both ways. And search either side of the measured rate, because
    # a rate slightly wrong looks identical to a signal that will not decode.
    bauds = {int(round(measured)), 300, 600, 1200, 200}
    if opt.get('baud'):
        bauds.add(int(round(opt['baud'])))
    for frac in (0.99, 0.995, 1.005, 1.01):
        bauds.add(int(round(measured * frac)))
    bauds = sorted(b for b in bauds if 100 <= b <= 3000)

    print('')
    print('trying %d rates x 2 tone assignments...' % len(bauds))
    found = []
    for mk, sp in ((mark, space), (space, mark)):
        for baud in bauds:
            _, _, hits = try_decode(sig, rate, mk, sp, baud)
            for (fmt, a, d), pos, pol in hits:
                found.append((baud, '%g/%g' % (mk, sp), fmt, a, d, pos,
                              'NEG' if pol == m.POL_NEGATIVE else 'STD'))

    if not found:
        buf, _, _ = try_decode(sig, rate, mark, space, int(round(measured)))
        print('  nothing decoded at any of them.')
        print('  bits at %d baud, mark %g: %s'
              % (round(measured), mark, buf[:24].hex(' ')))
        print('')
        print('That is a real answer rather than a failure: whatever is on the air')
        print('is not an ALERT frame at these tones and this rate. The tone list')
        print('above is the thing to look at next.')
        return 1

    print('  decoded %d frame(s):' % len(found))
    seen = set()
    for baud, label, fmt, a, d, pos, pol in found:
        key = (fmt, a, d)
        note = '' if key in seen else '   <-- new reading'
        seen.add(key)
        print('  %4d baud  mark/space %-11s  %s id=%d value=%d  bit %d %s%s'
              % (baud, label, fmt, a, d, pos, pol, note))
    print('')
    print('Those are the parameters the firmware should be using.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
