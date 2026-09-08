#!/usr/bin/env python3
"""Read the ALERT app's debug telemetry off the radio's USB serial port.

The app blocks APP_Update while it runs, so nothing that goes through the
normal K5 command path can be read while it is open. It does call
UART_ServiceCommands() every pass, though, and the USB CDC send is a no-op
unless a host has the port open with DTR asserted - so the app just pushes
lines out and this listens for them.

Two line types:

    D I<irq> S<sync> F<frames> G<gated> X<stuck> B<bits> R<rssi> Q<squelch>
        every 500 ms, the counters as shown on the radio's bottom line

    A <nbits> <gated> <rssi> <hex...>
        one per capture, emitted *before* the squelch gate and *before* the
        decoder, so it shows what the demodulator actually produced

Every A line is re-decoded here with a Python port of App/app/alert_decode.c.
If the firmware missed a frame the bits do contain, this says so - which
separates "the decoder is wrong" from "the bits are not there".

    python tools/alert/alertmon.py COM5 [seconds]

Passive: sends nothing to the radio.
"""
import sys
import time
from datetime import datetime

import serial

# --------------------------------------------------------------------------
# port of App/app/alert_decode.c

CRC6_POLY = 0x19  # x^6 + x^4 + x^3 + 1

POL_NEGATIVE, POL_STANDARD = 0, 1


def crc6(bits, nbits):
    reg = 0
    for i in range(nbits - 1, -1, -1):
        b = (bits >> i) & 1
        fb = ((reg >> 5) & 1) ^ b
        reg = (reg << 1) & 0x3F
        if fb:
            reg ^= CRC6_POLY
    return reg


def getbit(buf, pos):
    return (buf[pos >> 3] >> (7 - (pos & 7))) & 1


def decode_payload32(payload):
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
            return ('ABF', a, d)
        return None

    if k1 == 3:
        a = lsb_first(0, 6) | (lsb_first(8, 7) << 6)
        d = lsb_first(15, 1) | (lsb_first(16, 8) << 1) | (lsb_first(24, 2) << 9)
        r = msb_first(26, 6)
        if crc6(((a << 11) | d), 24) == r:
            return ('EIF', a, d)
    return None


def scan_polarity(buf, nbits, polarity, max_gap=20, max_out=8):
    idle = 0 if polarity == POL_NEGATIVE else 1
    start = idle ^ 1
    words, word_pos, out = [], [], []
    pos = 0
    while pos + 10 <= nbits and len(out) < max_out:
        if getbit(buf, pos) != start:
            pos += 1
            continue
        if pos > 0 and getbit(buf, pos - 1) != idle:
            pos += 1
            continue
        if getbit(buf, pos + 9) != idle:
            pos += 1
            words, word_pos = [], []
            continue
        w = sum(getbit(buf, pos + 1 + i) << i for i in range(8))
        if words and (pos - (word_pos[-1] + 10)) > max_gap:
            words, word_pos = [], []
        if len(words) == 4:
            words, word_pos = words[1:], word_pos[1:]
        words.append(w)
        word_pos.append(pos)
        pos += 10
        if len(words) == 4:
            payload = 0
            for k in range(4):
                for i in range(8):
                    payload |= ((words[k] >> i) & 1) << (31 - (8 * k + i))
            r = decode_payload32(payload)
            if r:
                out.append((r, word_pos[0], polarity))
                words, word_pos = [], []
    return out


def scan_all(buf, nbits):
    return scan_polarity(buf, nbits, POL_NEGATIVE) + scan_polarity(buf, nbits, POL_STANDARD)


# --------------------------------------------------------------------------

def bit_stats(data, nbits):
    """Cheap sanity check on the raw bits: a demodulator producing nothing
    useful usually gives all-zeros, all-ones, or no transitions at all."""
    ones = sum(getbit(data, i) for i in range(nbits))
    trans = sum(1 for i in range(1, nbits) if getbit(data, i) != getbit(data, i - 1))
    runs = []
    run = 1
    for i in range(1, nbits):
        if getbit(data, i) == getbit(data, i - 1):
            run += 1
        else:
            runs.append(run)
            run = 1
    runs.append(run)
    return ones, trans, (max(runs) if runs else 0)


def handle(txt):
    stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    if txt.startswith('A '):
        parts = txt.split()
        try:
            nbits, gated, rssi = int(parts[1]), int(parts[2]), int(parts[3])
            data = bytes.fromhex(parts[4]) if len(parts) > 4 else b''
        except (ValueError, IndexError):
            print('[%s] unparsed: %r' % (stamp, txt))
            return
        have = min(nbits, len(data) * 8)
        print('[%s] CAPTURE %d bits (%d B carried), gated=%d rssi=%d' %
              (stamp, nbits, len(data), gated, rssi))
        print('             %s' % (data.hex(' ') if data else '(none)'))
        if have:
            ones, trans, longest = bit_stats(data, have)
            print('             ones %d/%d, transitions %d, longest run %d' %
                  (ones, have, trans, longest))
            hits = scan_all(data, have)
            if hits:
                for (fmt, a, d), pos, pol in hits:
                    print('             >>> %s id=%d value=%d  bit %d  pol=%s' %
                          (fmt, a, d, pos, 'NEG' if pol == POL_NEGATIVE else 'STD'))
            else:
                print('             no framed word run decodes in either polarity')
    elif txt.startswith('D '):
        print('[%s] %s' % (stamp, txt))
    else:
        print('[%s] %s' % (stamp, txt))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 180

    ser = serial.Serial(port, 38400, timeout=0.2)
    ser.dtr = True          # the radio only sends while DTR is asserted
    ser.rts = True

    print('listening on %s for %ds (passive, DTR asserted)' % (port, secs), flush=True)
    t0 = time.time()
    pending = b''
    total = 0
    last_beat = 0.0
    while time.time() - t0 < secs:
        n = ser.in_waiting
        if n:
            pending += ser.read(n)
            total += n
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                txt = line.decode('ascii', 'replace').strip()
                if txt:
                    handle(txt)
                    sys.stdout.flush()
            last_beat = time.time() - t0
        else:
            el = time.time() - t0
            if el - last_beat >= 15:
                last_beat = el
                print('  ... %.0fs, %d bytes' % (el, total), flush=True)
            time.sleep(0.02)
    print('done: %d bytes in %ds' % (total, secs), flush=True)
    ser.close()


if __name__ == '__main__':
    main()
