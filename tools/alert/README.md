# ALERT receiver — what is known, and what to try next

Written 2026-09-29, from a session with the radio on the bench and both serial
ports live. It exists because the earlier chat logs were lost and the same
ground was being covered twice.

## Working, 2026-09-30: 22 bursts of 22 decoded, no hardware change

The receiver's demodulated audio is already on **PA4**, readable by the MCU's
ADC with the DAC holding a mid-rail bias (`AUD pin=PA4B pa=0`). The PB1 probe
that said "no audio" ran with the AF muted. The X1 build finds this by itself
at app entry and runs the software AFSK demodulator (`App/app/alert_adc.c`) on
every burst.

| Build | Bursts decoded | Named stations |
|---|---|---|
| everything before X1 (BK4819 FSK engine) | 0 | 0 |
| X1 `cee0699`, PA4 route | 3 of 3, then switched itself off | 0 (table was Mt Kanigan) |
| `340608b` (confirmation, gate and table fixes) | 11 of 18 (61%) | 9 |
| `8a9b2ec` (mid-bit sampling) | **22 of 22** | **21** |

Heard on 151.500 MHz: Tallai, Beachmere, Loamside, Kilmoylar Rd, Ipswich, Vennor
Drive, Alice Gap, Sheep Station, Logan Village, John Bray Park, Dayboro, Waller
Rd, Brassall, Bellbird Park, Wongawallan, Willow Vale, Rothwell.

What had to change after the first on-air run, in order:
1. The burst confirmation wanted the tones 10 dB over the idle hiss; they sit
   ~6 dB over it, so it turned the working route off. A decode now confirms it.
2. The idle gate dropped frames that follow 40 bits of something that does not
   frame (Bundamba 2044). Strictly back-to-back frames may now start up to 48
   bits behind an idle run; the chance rate on noise is unchanged.
3. The station table covered Central Queensland; the radio hears South East
   Queensland. `filters/stations.filter` is now Constitution Hill + Mt Glorious.
4. **The demodulator took every bit on its boundary.** The bit clock locks
   phase 0 to the transitions and the bit was taken at phase 0. Taking it at
   phase 16 is the whole difference between 61% and 100%.

The BK4819 FSK engine never once synced during a burst in any of the 23 X1
arrangements (`bs=0` across the board). It is not needed.

### Flashing is hands-off now

After the one PTT-at-power-on flash of X1, every later build goes on over USB-C:

```bash
python tools/hotflash.py --run <CI run id> --expect-hash <short sha>
```

It checks the on-radio bootloader (CRC 22CDCECB, 7.00.07), jumps the running
app into the stock bootloader's own DFU, flashes with serialtool, and confirms
the new build's hash. Verified three times on this radio.

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

**Testable on the radio as it stands, no reflash:** MONITOR on is the old
code's only route to AF = FM. There is no `#` key on this radio's keypad, so use
the app's MENU and the MONITOR row, not the keyboard shortcut the notes above
once gave. Capture the USB stream and compare.

```bash
python tools/alert/capture_stats.py com5.log
```

If the transition density drops away from 0.5, that was it.

Second, also no reflash: **MDM MODE → SAME**. Of the four modes, SAME
(1562/2083 Hz) is the only one that has ever been near this signal — its mark
sits 17 Hz from the real 2100 Hz, and it is a non-coherent AFSK detector, where
FFSK 1200/1800 locks tone to bit rate and matches neither. Mode has never been
set to anything but 0.

## Result: SAME mode does not acquire (tested 2026-09-29)

MDM MODE set to SAME, MONITOR on, over 2.2 minutes:

| | |
|---|---|
| Bursts counted | 7 |
| RSSI range | −87 to **−19 dBm** |
| Strongest peak | −15 dBm |
| Sync words found | **0** |
| Captures | **0** |

Signal is unambiguously arriving and the squelch opens for it. The SAME-mode
FSK engine never raises `FSK_RX_SYNC`, not once. Mode 0 at least tripped sync
occasionally, on noise; SAME trips it never.

That is worse than a negative result, because **a capture only begins on a sync
interrupt**, so SAME yields no bits at all to examine. The tone question cannot
be answered from behind a sync detector that will not fire.

## A flaw in the earlier capture analysis

A burst lasts at most ~500 ms. `cfg.pktlen` defaults to 32 bytes, and at 300
baud the chip will not raise `RX_FINISHED` until 256 bits have arrived, which is
853 ms. So the watchdog ends every capture at 1.5 s, and roughly the last 350
bits of each 448-bit capture are the engine free-running on noise *after* the
signal stopped. Judging the whole capture dilutes whatever the opening holds.

Re-examined windowed, the two captures disagree: cap1 opens at 0.590
transitions per bit and cap2 at 0.385, one above chance and one below, both
inside about 1.4 standard errors of 0.5. **Two captures cannot settle it.**

Fix the setting before gathering more: **CAPTURE to 16 bytes** (427 ms at 300
baud) so a capture is mostly burst. `capture_stats.py` now prints the windowed
view and the standard error, so a single suggestive window cannot be mistaken
for evidence.

## The decisive test still outstanding

**MDM MODE back to FFSK1218, MONITOR left on, CAPTURE 16.**

Mode 0 is the only mode that demonstrably trips sync, so it is the only one that
yields bits. With MONITOR on, the AF selector is at FM instead of MUTE, which is
the one variable that differs from the captures that came out as noise. Same
mode, same sync word, unmuted AF, undiluted window. If the transition density
drops away from 0.5 across several captures, the mute was the problem.

If it does not, try **SYNC = FFFF** on SAME. If the engine's output sits stuck
high, that will trip immediately and finally hand over some bits to look at.

## Correction: every capture so far was recorded during silence

This invalidates the "the engine emits noise" conclusion above, and it is the
most useful thing found so far. The `A` capture line's second field is `gated`,
meaning the squelch was open during the capture. Across every capture ever
taken, in both configurations:

| Capture | gated | RSSI | floor at the time |
|---|---|---|---|
| earlier run, AF muted | 0 | −70 dBm | ~−74 dBm |
| earlier run, AF muted | 0 | −72 dBm | ~−74 dBm |
| mode 0, AF at FM | 0 | −84 dBm | −98 dBm |
| mode 0, AF at FM | 0 | −83 dBm | −98 dBm |

Bursts peak at **−14 dBm**. Every capture sat at the noise floor with the
squelch shut. **No capture has ever coincided with a burst.** The statistics
were measuring the receiver's own silence, which is of course noise, and they
were never a test of what the engine does with a signal.

`SQ GATE` defaults **off**, which is why these were accepted rather than
discarded.

### What that means

The sync detector is not failing to slice the burst. It is not firing during
the burst at all. It fires occasionally on noise between bursts, because random
bits throw up sixteen zeros now and then, and `SYNC` is set to `0000`.

Everything fits: syncs happen but rarely, never during a burst, and every
resulting capture is noise. The default `SYNC_0000` was chosen on the reasoning
that an idle preamble under negative logic reads as zeros. That assumed the
sense, and the sense is exactly what has never been established.

### Also settled: unmuting the AF changed nothing

Mode 0 with MONITOR on, opening window pooled over 298 adjacent pairs, gives
0.5034 with a one-sided p of 0.57 against the AF-muted set's 0.5201. Difference
z = 0.41, not significant. The AF-mute theory is dead. Both sets are silence, so
this was never going to show anything, but it is ruled out either way.

## The test that follows from it

Two menu changes, and the radio does the rest:

1. **SQ GATE = ON.** Captures recorded with the squelch shut are then rejected
   and counted in `G`, so `S` rising with a capture at a strong RSSI means a
   real burst was acquired. Pure instrumentation, and it stops the analysis
   being poisoned by silence.
2. **SWEEP = ON.** The sweep in the shipped build walks sync × baud and reports
   each arrangement over USB as `W <idx> s<sync> bd<baud> S<syncs> B<bursts>`,
   which is exactly "which sync word acquires, judged on this many real
   bursts". It covers all four sync values at 300 baud among its sixteen
   arrangements and paces itself to the signal, so it needs no further
   keypresses.

The candidates it will cover, and what each would mean:

| `SYNC` | Fires if the sliced preamble is |
|---|---|
| `0000` | a steady tone read as zeros (tried, never fires on a burst) |
| `FFFF` | a steady tone read as ones, i.e. the opposite sense |
| `AAAA` / `5555` | an alternating pattern, as modems send for clock recovery |

If one of those acquires during bursts, there will be real bits to decode, and
the inversion fix already committed means the sense no longer has to be guessed.

## The sweep confirmed it, and SQ GATE is what proved it

Eight arrangements, one qualifying burst each, with SQ GATE on:

| idx | SYNC | baud | syncs | bursts |
|---|---|---|---|---|
| 2 | 0xAAAA | 200 | 0 | 1 |
| 3 | 0x5555 | 200 | 0 | 1 |
| 4 | 0x0000 | 300 | 0 | 1 |
| 5 | **0xFFFF** | **300** | **1** | 1 |
| 6 | 0xAAAA | 300 | 0 | 1 |
| 7 | 0x5555 | 300 | 0 | 1 |
| 8 | 0x0000 | 600 | 0 | 1 |
| 9 | 0xFFFF | 600 | 0 | 1 |

Arrangement 5 looks like an acquisition until you read the capture it produced:

```
A 448 0 -87    <- gated 0, RSSI -87 dBm
```

Squelch shut, at the noise floor, while bursts peak at −14 dBm. Another noise
sync that happened to land inside that arrangement's window. `G` incremented to
1, so SQ GATE correctly rejected it, which is exactly the instrumentation that
was missing before.

**No sync word acquires during a burst.** Not `0000`, not `FFFF`, not `AAAA`,
not `5555`. Syncs arrive at roughly one per eight bursts' worth of elapsed time
and are uncorrelated with the signal, because they are noise.

The `F1` that appeared earlier is not evidence either. The firmware's own
single-path scan decodes something from **2.70%** of random 448-bit captures,
measured over 4000 of them, so across the dozen captures this session one
spurious frame has a 28% chance of turning up. A decode worth believing names a
real station: only 379 of the 8192 possible addresses are in the table, so a
chance decode lands on a named station just 0.12% of the time.

## The fix, committed and needing a flash

Capture on the **squelch edges** instead of on a sync word. `AdcPoll` has worked
this way since the port — open squelch starts collecting, closed squelch ends
it, no sync word anywhere — and the modem path should have been written the same
way from the start.

The engine never needed the sync word to make bits. Once triggered it free-runs
and fills 56 bytes in the 1.5 s before the watchdog stops it. The only thing
missing was draining the FIFO while the signal was actually present, and the
squelch already knows when that is.

Why the sync word never matched is inference rather than measurement: `REG_59`
asks for a six-byte preamble and the BK4819's preamble detector wants something
alternating, which a burst opening on a steady tone does not provide.

`C` in the heartbeat counts squelch-started captures, kept separate from `S` so
the two triggers stay distinguishable.

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
| `radio.py` | ALERT-X1: logs the USB-C stream with PC timestamps and sends the X1 host commands. |
| `sweep_judge.py` | ALERT-X1: gives a verdict on every arrangement from one or more logs, with the plan's exit codes. |
| `scan_samples.py` | Python port of `ALERT_ScanSamples`, the oversampled edge slicer; the judge decodes with it. |
| `test_judge.py` | fabricates X1 logs whose answer is known and checks the judge's verdicts and radio.py's framing. |

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
D I<squelch irqs> F<frames> G<gated> B<bits> R<rssi> Q<squelch> N<bursts>
  f<floor> p<peak> V<inverted-sense> C<squelch openings>
```

Every 500 ms with the squelch shut. B is the bits the ADC demodulator gave for
the last qualifying burst. The FSK fields (S syncs, X stuck streams, a
arrangement, z pass) went with the sweep in V2.

## ALERT-X1: the sweep (removed in V2) and judging its logs

V2 Phase A (`V2_SPEC.md` section 1) removed the FSK sweep from the firmware:
the arrangement table, scoring and adoption, the `L C H Y F Z G` lines and the
0x0A01-0x0A03 host commands. The receiver is plain RX with the AF at FM and
the ADC route decodes every burst; a census choice that has decoded is kept,
so the census runs only on a fresh radio or when the CENSUS setting asks.
What follows describes the X1 build, and `sweep_judge.py` still reads its logs.

The plan is `X1_PLAN.md`. After the one PTT-held flash, the build needs no
keypresses. The ALERT app starts 5 s after boot, runs the audio-pin census, and
paces the sweep to real bursts. A Phase 1 pass takes about 8 minutes; coverage
good enough for a "dead" verdict takes 1-2 hours of traffic.

```bash
python tools/alert/radio.py log                  # Ctrl-C to stop; or --minutes 120
python tools/alert/sweep_judge.py tools/alert/logs/x1-*.log
```

`radio.py log` finds the port by VID 36B7 and asserts DTR. It re-asserts DTR
after 10 s of silence, because a timed-out USB send in the firmware clears the
DTR flag and nothing more arrives until the host sets it again. It writes every
line with a PC timestamp to `tools/alert/logs/`, which git ignores. It survives
resets, and it releases the port for 90 s when it sees bootloader beacons so
that `hotflash.py` can take it.

While the logger runs, the other subcommands pass their frame to it over
localhost, because Windows lets only one process open a COM port:

```bash
python tools/alert/radio.py bk-read 0x58 0x59 0x5C
python tools/alert/radio.py dfu-check            # the K bl line: may hotflash.py proceed?
python tools/alert/radio.py reboot --wait        # prints the B line
```

The judge prints one row per arrangement: BITS, SYNC, STRUCT with the in-byte
bit order that passed, DECODE, REPEAT, the chance control and the noise-sync
ratio. It then gives an overall verdict:

| Exit | Verdict | Next |
|---|---|---|
| 0 | WORKS, LIKELY, or ADC route works | The firmware adopts at REPEAT 3 by itself (`G ADOPT`). For LIKELY, keep logging. |
| 1 | BITS-NO-DECODE | The bits are there. Check the bit order, rate or decoder with `capture_stats.py [--lsb] --arr <idx> <log>`. |
| 2 | FSK route dead | See the plan's section 11: one capacitor to PA4, or the earphone lead and `wavdecode.py`. |
| 3 | Not enough data | Keep logging. The "still needed" lines say which arrangements are short. |

The judge decodes the `H` windows on the host with the `scan_samples.py` edge
slicer. For integer k it also uses phase decimation, and it falls back to
decimation alone if `scan_samples.py` is missing. Every path uses the
firmware's 12-idle-bit gate, and the chance control shuffles each window's run
lengths through the same procedure. A decode counts only when it names a table
station, so a real result repeats: the same ID, on the same path, across
separate bursts.

STRUCT is measured from the preamble run through one frame, not over the whole
window. The window carries up to 140 ms of discriminator noise on either side
of the burst, which would sink a perfect capture. `python tools/alert/test_judge.py`
checks all of this on fabricated logs.

`radio.py send <id> <hex>` sends any body as-is.
