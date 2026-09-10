#!/usr/bin/env python3
"""Synthesise an ALERT burst as a WAV, to prove wavdecode.py works.

wavdecode is about to be pointed at a recording nobody has analysed before, and
a tool that has only ever been run on unknown data cannot be trusted when it
says "no ALERT frame". This builds a burst with a known id and value at known
tones and a known rate, so the tool can be checked against an answer.

    python tools/alert/make_test_wav.py out.wav [--id 1234 --value 567]
    python tools/alert/wavdecode.py out.wav        # should report id=1234 value=567
"""
import array
import math
import os
import sys
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alertmon as m

RATE = 22050
MARK, SPACE = 1200.0, 2200.0
BAUD = 300.0


def eif_words(a, d):
    r = m.crc6(((a << 11) | d), 24)
    w0 = (a & 0x3F) | 0xC0
    w1 = ((a >> 6) & 0x7F) | ((d & 1) << 7)
    w2 = (d >> 1) & 0xFF
    w3 = (d >> 9) & 0x03
    for j in range(6):
        w3 |= ((r >> (5 - j)) & 1) << (2 + j)
    return [w0, w1, w2, w3]


def frame_bits(words, lead=24, trail=12, repeats=3):
    """Idle, then each word as start + 8 data LSB-first + stop. Idle is 0,
    matching ALERT_POL_NEGATIVE, which is the decoder's default."""
    bits = []
    for _ in range(repeats):
        bits += [0] * lead
        for w in words:
            bits.append(1)
            bits.extend((w >> i) & 1 for i in range(8))
            bits.append(0)
    bits += [0] * trail
    return bits


def afsk(bits, rate=RATE, baud=BAUD, mark=MARK, space=SPACE, amp=12000):
    """Continuous-phase AFSK, the way a real modem sends it."""
    spb = rate / baud
    out = array.array('h')
    phase = 0.0
    for b in bits:
        f = mark if b else space
        dph = 2.0 * math.pi * f / rate
        for _ in range(int(spb)):
            out.append(int(amp * math.sin(phase)))
            phase += dph
            if phase > 2.0 * math.pi:
                phase -= 2.0 * math.pi
    return out


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'alert_test.wav'
    opt = {}
    for i in range(2, len(sys.argv) - 1, 2):
        if sys.argv[i].startswith('--'):
            opt[sys.argv[i][2:]] = int(sys.argv[i + 1])
    sid = opt.get('id', 1234)
    val = opt.get('value', 567)

    bits = frame_bits(eif_words(sid, val))
    samples = afsk(bits)
    with wave.open(out, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(samples.tobytes())
    print('%s: EIF id=%d value=%d, %d bits at %g baud, %g/%g Hz, %.2f s'
          % (out, sid, val, len(bits), BAUD, MARK, SPACE, len(samples) / RATE))


if __name__ == '__main__':
    sys.exit(main())
