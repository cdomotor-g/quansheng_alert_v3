# ALERT receiver — what is known, and what to try next

Written 2026-09-29, from a session with the radio on the bench and both serial
ports live. It exists because the earlier chat logs were lost and the same
ground was being covered twice.

## The signal, settled

Measured off air by the operator, not inferred from a datasheet:

| | |
|---|---|
| Rate | **300 baud** |
| Mark | **2100 Hz** |
| Space | **1300 Hz** |
| Scheme | V.23 mode 2 tones, ordinary non-coherent AFSK |
| Framing | four 10-bit async words, start + 8 data LSB-first + stop |

Everything in this repo used to assume Bell-202 (1200/2200). That was never
checked, and it made every test of the decode chain a test against the wrong
signal. `make_test_wav.py` now generates the real pair.

## The radio is hearing it

Over ten minutes on 151.500 MHz, read from the USB-C port:

| | |
|---|---|
| Bursts seen | 86 |
| Peak RSSI | **−15 dBm** |
| Noise floor | −75 dBm |
| Sync words found | 14 |
| **Frames decoded** | **0** |
| Captures ended by the watchdog | 14 of 14 |

So reception is not the problem. Every capture is exactly 448 bits and every
one runs until the 1.5 s watchdog stops it — 448 bits in 1.5 s is ~300 bit/s,
which says TONE2 *is* setting the bit clock correctly. The engine is running at
the right rate and slicing the wrong thing.

## The captures are noise, and there is a control for saying so

```
                bits   ones/bit  trans/bit  maxrun   autocorr
capture 1        320      0.438      0.511       9   max|r|=0.095
capture 2        320      0.406      0.511       8   max|r|=0.144
random bits      320      0.47–0.51  0.48–0.52   7–8
real burst        64      0.141      0.222      24
same, 4x over-   256      0.141      0.055      96
```

Geometric run histogram, autocorrelation flat to lag 24, and no trace of the
run-length-multiple-of-4 structure a 300-baud signal read at 1200 baud would
show. These are noise by every measure.

**Do not trust an apparent decode without the control.** `capture_stats.py`
tries ~84 paths per capture, and ABF and EIF each constrain only about 8 bits,
so **roughly a quarter of pure-noise captures yield at least one "decode"**.
Both hits in the captures above are chance. A real one repeats: the same
address, at the same settings, across several bursts. Reading structure into
noise has already cost this project one wrong turn.

## Three bugs fixed, none of which was the blocker

1. **An inverted bitstream could not be decoded at all.** `ALERT_POL_STANDARD`
   swaps which level is idle; it does not complement the data bits, so an
   inverted signal frames up and then fails every check bit and FCS. Four
   combinations exist (two framings × two data senses) and the polarity
   argument reached two. `ProcessCapture` now retries the complemented sense;
   `V` in the heartbeat counts when that was the one that worked. This is the
   exact shape of "syncs greater than zero, frames exactly zero", and it would
   have bitten even with perfect bits.

   The sweep's comment argued `invert` could stay at 0 because the sync values
   are closed under complement. True of *acquiring* a burst, false of decoding
   one.

2. **The software correlator was tuned to 1200/2200.** It did still separate
   the real tones — they fall either side of its decision boundary — but with
   the sense reversed and much less margin. Through the exact integer code,
   40 bursts per point: at 0.3 noise 19/40 decoded before, 38/40 after.

3. **The sweep never varied mode.** It spent transmissions on baud, which is
   settled, and pinned mode 0. Now sync × mode at a fixed 300 baud.

## The one thing most worth testing next

`ModemArm` set the AF selector from `cfg.monitor`, which defaults **off**, so
`REG_47` sat at `BK4819_AF_MUTE` the whole time the modem was armed — and the
FSK engine slices that same demodulated audio. Meanwhile
`BK4819_PrepareFSKReceive`, the one FSK path in this fork that demonstrably
works, never writes `REG_47` at all, so aircopy runs with AF at FM. This app
was the odd one out and it is the one that has never decoded.

A demodulator fed silence produces exactly the noise measured above, and it
would do so at *every* sync word and bit rate — which is why sweeping the
settings downstream could never have shown a difference.

Fixed here: AF stays at FM, and `AUDIO_AudioPathOff()` alone decides whether
the speaker is live.

**Testable on the radio as it stands, no reflash:** MONITOR on (`F` then `#`
inside the app) is the old code's only route to AF = FM. Capture the USB stream
and compare.

```bash
python tools/alert/capture_stats.py com5.log
```

If the transition density drops away from 0.5, that was it.

Second, also no reflash: **MDM MODE → SAME**. Of the four modes, SAME
(1562/2083 Hz) is the only one that has ever been near this signal — its mark
sits 17 Hz from the real 2100 Hz, and it is a non-coherent AFSK detector, where
FFSK 1200/1800 locks tone to bit rate and matches neither. Mode has never been
set to anything but 0.

## If the engine still will not do it

The BK4819's FSK modes are fixed tone pairs and none of them is 1300/2100 at
300 baud. The route with a sound basis is to get the receiver's audio to an ADC
pin and run the software correlator, which handles arbitrary tones and needs no
sync word at all. That needs three things:

1. the one-wire mod (BK4819 EARO → 100 nF → an ADC-capable pin),
2. `AdcStart`/`ALERT_AdcTick` ported from DP32G030 to PY32F071 — the current
   code is V1-only, which is why `ENABLE_ALERT_ADC` is defined nowhere,
3. `ENABLE_ALERT_ADC` in the V3 build.

PB0/PB1 were probed and carry no audio: PB1 is a steady DC level, 18 counts of
spread, the same whether the squelch is open or shut.

**Much cheaper alternative.** A 3.5 mm lead from the earphone socket to a PC
sound card skips all of the above — the PC-side chain already demodulates and
decodes correctly end to end:

```bash
python tools/alert/wavdecode.py burst.wav
```

## The tools

| | |
|---|---|
| `capture_stats.py` | signal or noise, with a random baseline and the false-positive control. Feed it a log of the USB stream. |
| `simdemod.py` | bit-exact model of `ALERT_AdcTick`, so a demodulator change can be measured before it is flashed. Its BER helper reports the *sense* as well as the rate, because "garbage" and "clean but inverted" need opposite fixes. |
| `test_decode.c` | 13 host assertions on the decoder, including the inversion regression. Builds with `-Wall -Wextra -Werror`. |
| `wavdecode.py` | measures tones and rate from a recording, then decodes. |
| `make_test_wav.py` | synthesises a burst with a known answer, at the real tones. |
| `alertmon.py` | reads the live serial stream. |

```bash
gcc -std=c11 -Wall -Wextra -Werror -I App \
    tools/alert/test_decode.c App/app/alert_decode.c -o test_decode && ./test_decode
```

## Serial ports

Both reach the radio and both answer the read-only `0x0514` hello with
`F4HWN v5.9.0`.

| Port | What it is |
|---|---|
| USB-C on the radio (CDC, VID_36B7) | the debug stream — `D` heartbeats, `A` captures, `ALERT,` decodes. Use this one. |
| CH340 on the mic/speaker jacks | the UART. Silent until the app decodes something or it is spoken to. |

The app calls `UART_ServiceCommands()` every loop pass, so the radio answers
protocol commands while the ALERT screen is up. Changing a setting this way is
not worth it: the app loads its config on entry and writes it back on exit, so
an EEPROM poke would be overwritten. Use the menu.

## Heartbeat fields

```
D I<irq> S<syncs> F<frames> G<gated> X<stuck> B<bits> R<rssi> Q<squelch>
  N<bursts> m<mode> y<sync> v<invert> l<sync4> f<floor> p<peak> V<inverted-sense>
```
