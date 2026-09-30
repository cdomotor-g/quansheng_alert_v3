# ALERT receiver: engineering plan for the next firmware build (one flash)

## 0. Bottom line

Goal of the build: one flash that tests every firmware-only route on its own. It paces itself to real bursts, reports per-arrangement evidence over USB-C in lines a script can judge, adopts and saves whatever decodes, and makes every later flash hands-off.

**The one physical step that cannot be avoided.** This build must go on once through today's path: power on with PTT held. The running firmware has no command that can write flash or reach DFU:
- `0x05DD` only calls `NVIC_SystemReset` (App/app/uart.c:847-852).
- `ENABLE_UART_RW_BK_REGS` is off (CMakePresets.json:85).

After that one flash, every later flash can be scripted from the PC.

**Routes, ranked.** Probabilities are my estimates, not measurements.

| Rank | Route | P(decodes) | Basis |
|---|---|---|---|
| 1 | BK48xx FSK engine in a real sub-carrier receive mode (RX 111 FFSK1200/1800), oversampled at TONE2 1200/1500/1800 or tone-scaled 1300/1400, software edge slicer | ~35% | ta1js decodes off-air Bell 202 in RX 111 (a single-author self-report, on BK4819 not BK4829). Main doubts: the 1300 Hz margin, sync on a steady tone, BK4829 differences. |
| 2 | Same engine, SAME receive (RX 000, BW 010) or FFSK1200/2400 | ~15% combined | Documented, never demonstrated on any UV-K5. |
| – | Any FSK arrangement (union, including stream-through) | ~45% | |
| 3 | Receiver audio already on an MCU ADC pin (PB1 or PA4), found by an automatic census, decoded by the existing correlator ported to PY32 | ~25-30% | The PB1 "no audio" probe ran with AF muted (see §1). PA4 has never been sampled. fagci's K1 firmware used PB1 as "APRS audio" (suggestive only). |
| 4 | Polling BK registers (REG_6F level, DTMF/SelCall, GPIO) | ≤5% | Every documented detector is 20 ms or slower against a 3.33 ms bit. Not in this build. |
| – | At least one firmware-only route works | **~55-60%** | Routes 1-2 and 3 are roughly independent. |
| – | Hands-off flash trampoline works first time | ~70% | A failure costs a power cycle, never a brick (§6). |

## 1. Corrections to the input reports (checked this session)

- **Wrong TONE2 register values.** `tone_reg()` (alert.c:266-269) gives:

  | Hz | Register |
  |---|---|
  | 1500 | 0x3C7F |
  | 1800 | 0x4898 |
  | 521 | 0x1503 |
  | 525 | 0x152C |
  | 600 | 0x1833 |
  | 1042 | 0x2A06 |
  | 1050 | 0x2A59 |
  | 1200 | 0x3065 |
  | 1300 | 0x346E |
  | 1400 | 0x3876 |
  | 2100 | 0x54B1 |
  | 2400 | 0x60CB |

  The critique's 0x3A94 and 0x4697 are 1452 Hz and 1750 Hz. The table below stores Hz and calls `tone_reg()`, so this cannot recur.
- **Erasing the app does not force DFU.** Disassembly of the stock 7.00.07 bootloader (`bl70007.bin`, 0x2700 bytes, zlib CRC32 0x22CDCECB) with the scratch `thumb.py`:
  - The app-jump routine at 0x080006F0 checks `(SP & 0x2FFC0000) == 0x20000000` at 0x0800071E.
  - If the check fails it returns (0x0800073A), `main` returns, and the MCU hangs. It does not enter DFU.
  - So "erase the vector page and reset" is a brick until PTT recovery. The DFU trampoline in §6 is the only safe software entry.
- **Bootloader facts the trampoline relies on (verified):**
  - The clock init at 0x08000EC8 clears PLLON, then spins at 0x08000EFC until PLLRDY is 0. The PLL must therefore be off, with SYSCLK on HSI, before calling it.
  - The delay at 0x08000418 is a NOP loop. There is no SysTick dependency.
  - The handshake at 0x08000240 accepts only "7.00" or '*'.
  - serialtool defaults `--bl-ver` to "?" (tools/serialtool/cli.py:216-221), so it must be run with `--bl-ver 7.00`.
- **The PB1 probe ran with AF muted.** It was taken at e6f2e823, where ModemArm set `cfg.monitor ? AF_FM : AF_MUTE` with monitor off. It sampled only 192 conversions (about 0.4 ms). PA4 was never sampled (alert.c:743-744).
- **REG_5E is written by the compiled driver.** bk4829.c:1145 (aircopy) writes 0x3204 = `0x3000 | (64<<3) | 4`. The comment at alert.c:345 saying nothing touches it is wrong.
- **The radio is a BK4829 in software terms.** App/CMakeLists.txt:10 compiles `driver/bk4829.c`. The BK4819 documentation is assumed to carry over.
- **Budget** (CI run 36664302193): RAM 11,328 of 16,384 B, flash 92,128 of 120,832 B. About 5 KB RAM and 28 KB flash are free.
- **Burst rate** (com5.log): median interval 19.75 s (range 4.5-27.5 s). A 23-arrangement pass therefore takes about 8 minutes.
- **USB send blocks.** `cdc_acm_data_send_with_dtr` (App/usb/usbd_cdc_if.c:187-201) blocks, and on timeout it sets `dtr_enable = 0`, after which output stops until the host re-asserts DTR. The host tool must re-assert DTR.

## 2. What the build does with no input (build name "ALERT-X1")

1. **Boot.** Print `B` with the git hash and reset reason. Open a 5 s window in normal mode in which USB commands (including DFU) are serviced. Then start the ALERT app automatically, unless the last three resets were abnormal (§7).
2. **App entry.** Settle the RSSI floor for 10 s. Run the audio-pin census (about 3 s, §5). Then start the FSK sweep at Phase 1 (§3), unless an arrangement was adopted earlier.
3. **Every qualifying burst** produces `C`, `H` and `X` lines and scores the current arrangement. The ADC decoder runs in parallel on the same burst if the census passed.
4. **Adoption.** When an arrangement decodes the same table station in 3 separate bursts, it is adopted and saved, and the app switches to production decoding. It keeps logging, and resumes the sweep if 20 qualifying bursts in a row fail to decode.

## 3. FSK-engine sweep

### 3.1 Arm recipe (replaces ModemArm, alert.c:295-384, as `ArrArm(const Arr_t *a)`)

Prologue, kept as today: `BK4819_ResetFSK(); REG_02=0; REG_3F=0; BK4819_RX_TurnOn();`

Then, in order:

| Register | Value |
|---|---|
| REG_70 | `a->r70`: 0x00E0 (TONE2 on, gain 96), or 0x80E0 (TONE1 on at gain 0, as ta1js does) |
| REG_71 | `tone_reg(a->t1hz)`, only when TONE1 is on |
| REG_72 | `tone_reg(a->t2hz)` |
| REG_58 | `a->r58` |
| REG_5A | sync bytes 0,1 |
| REG_5B | sync bytes 2,3, only when 4-byte sync |
| REG_5C | `a->r5c`: 0x5625 (current, CRC off) or 0xAA30 (ta1js) |
| REG_5D | 0xFFE0 = 2048 bytes. The field is N-1; <15:8> low byte, <7:5> high bits (Registers V1.1). If the 11-bit length is not honoured on BK4829, the effective length is 256 B; the `F` line measures which. |
| REG_5E | 0x3202: keeps the upper bits of the proven 0x3204, almost-full threshold 2 |
| REG_59 | `base = (inv<<10) \| (pre<<4) \| (s4<<3)`, preamble nibble **0** (every RX precedent uses 0). Write `base\|0xC000`, then `base`, then `base\|0x1000`. |
| REG_3F | FSK_RX_SYNC \| FSK_FIFO_ALMOST_FULL \| FSK_RX_FINISHED \| SQUELCH_FOUND \| SQUELCH_LOST |
| REG_02 | 0 |

Then `BK4819_SetAF(FM)` and the audio path from `cfg.monitor`, as now. Finally apply the host poke list (§8, 0x0A03).

**Re-arm** (light; replaces ModemReArm, alert.c:398-404): write `base`, then `base|0x4000`, then `base|0x1000`.

### 3.2 Phase 1 table

A `const Arr_t` array in alert.c, replacing `modeReg58[]` and SweepApply's sync×mode mapping (alert.c:1106-1127).

Defaults unless a row says otherwise: gain 3, 2-byte sync, REG_5C 0x5625, r70 0x00E0.

| # | Tag | r58 | TONE2 Hz | k = TONE2/300 | r70 / TONE1 | Sync | Why |
|---|---|---|---|---|---|---|---|
| 0 | A1 | 0x3FC3 | 1200 | 4 | 0x80E0 / 2100 | FFFF, 5C=AA30 | ta1js RX clone (aprs_minimal.c:375-399) |
| 1 | A3 | 0x3FC3 | 1800 | 6 | 0x80E0 / 2100 | FFFF | avoids the 2100 Hz null under a 1/1200 differential detector |
| 2 | B2 | 0x03C5 | 1042 | 3.47 | – | FFFF | SAME RX (RX 000, BW 010) at 2x |
| 3 | A2 | 0x3FC3 | 1200 | 4 | – | FFFF | messenger style, TONE1 off |
| 4 | A4 | 0x3FC3 | 1500 | 5 | 0x80E0 / 2100 | FFFF | |
| 5 | A5 | 0x3FC3 | 1200 | 4 | 0x80E0 / 2100 | FFFFFFF0 (4-byte) | edge sync: 28 idle samples + start |
| 6 | B1 | 0x03C5 | 521 | 1.74 | – | FFFF | SAME native rate |
| 7 | A7 | 0x3FC3 | 1400 | 4.67 | 0x80E0 / 2100 | FFFF | if tones scale with TONE2: 1400/2100 |
| 8 | A8 | 0x3FC3 | 1300 | 4.33 | 0x80E0 / 2100 | FFFF | if tones scale: 1300/1950 |
| 9 | C1 | 0x73C9 | 1200 | 4 | – | FFFF | FFSK1200/2400 RX (RX 100, BW 100) |
| 10 | A6 | 0x3FC3 | 1800 | 6 | 0x80E0 / 2100 | FFFFFFC0 (4-byte) | edge sync at k=6 |
| 11 | A9 | 0x3FC3 | 1200 | 4 | 0x80E0 / 2100 | AAAA | MSK hedge |
| 12 | B4 | 0x03C5 | 1200 | 4 | – | FFFF | |
| 13 | A10 | 0x3F03 | 1200 | 4 | – | FFFF | <7:6>=00, as OneOfEleven/losehu |
| 14 | A11 | 0x3FC3 | 1200 | 4 | 0x80E0 / 1300 | FFFF | TONE1 at the space tone |
| 15 | B3 | 0x03C5 | 525 | 1.75 | – | FFFF | SAME tones become 1575/2100 if they are 3x/4x the rate |
| 16 | C2 | 0x73C9 | 2400 | 8 | – | FFFF | |
| 17 | C3 | 0x73C9 | 1050 | 3.5 | – | FFFF | if tones scale: 1050/2100 |
| 18 | B5 | 0xA3C5 | 1042 | 3.47 | – | FFFF | SAME with TX mode 101 |
| 19 | S1 | as A1 | 1200 | 4 | as A1 | FFFF | **stream policy** |
| 20 | S2 | as B2 | 1042 | 3.47 | – | FFFF | **stream policy** |
| 21 | D1 | 0x03C1 | 300 | 1 | – | FFFF | negative control: today's configuration; expect no burst bits |
| 22 | D2 | 0x03C1 | 1200 | 4 | – | FFFF | direct-FM slicer at 4x (control) |

**Dwell.** One qualifying burst per arrangement per pass. S1 and S2 dwell until 2 bursts arrive while the engine is already streaming, or 6 bursts pass.

**Qualifying burst.** Squelch open, then closed; peak ≥ floor + 15 dB (existing `BURST_MARGIN_DB`); duration 100-1500 ms.

**Phase 2 (automatic).** Runs after pass 2 if any arrangement had a structured capture or a decode but has not been adopted. Take the top 2 by score and try these variants, 2 bursts each:
- RX gain 0-3 (`r58` bits 9:8)
- for FFSK1218, BW 100 (0x3FC9)
- 4-byte sync
- REG_59 invert
- preamble nibble 6

A variant is saved as base index + variant byte.

**Phase 3 (automatic).** Runs only if passes 1-2 produced no structured capture anywhere:
- 40 undocumented REG_58 codes: `(1<<13)|(rx<<10)|(3<<8)|(3<<6)|(bw<<1)|1` for rx in {1,2,3,5,6} and bw in {0..7}, at 1200 / FFFF. Generated from the index, so nothing is stored.
- 8 variants on A1: REG_59 bit 8, REG_59<2:0> = 1..7.
- Any FIFO_ALMOST_FULL during a burst without an RX_SYNC is logged. That would mean sync-free output.

After Phase 3 the sweep returns to Phase 1 passes and runs indefinitely; scores accumulate.

### 3.3 Capture state machine (rewrite of ModemPoll, alert.c:427-515)

- **Ring buffer.** `capBuf[256]` becomes a ring with a head index and a running `wordsTotal`. Each word from REG_5F is stored low byte first, then high byte (ta1js aprs_minimal.c:409-413), XORed with 0xFFFF when the sync was negative (REG_0B bit 7; OneOfEleven mdc1200.c does the same).
- **On FSK_RX_SYNC.** Record the time, REG_0B<7:6>, sqOpen and RSSI. Emit `Y` after the burst. Write the matched sync pattern into the ring first (2 or 4 bytes, complemented on SyncN), so the edge-sync cases keep their start bit and the preamble gate has idle samples.
- **Search policy** (all rows except S1/S2):
  - Squelch rising edge: re-arm, unless the engine synced within the last 300 ms or is mid-capture. That keeps syncs that fired on the carrier or the tone onset.
  - A sync while the squelch is shut: stream at most 1.0 s (counted as noise), then re-arm, so the engine is usually searching when a burst starts.
- **Stream policy** (S1/S2): never re-arm on the squelch. On RX_FINISHED, re-arm at once. A burst's bits then arrive whenever it lands inside a noise-started stream, which tests the demodulator with no dependence on the sync detector.
- **On FIFO_ALMOST_FULL:** read 2 words.
- **On squelch falling edge:** keep draining for 40 ms (so no words are stranded), then call BurstFinish. Never wait for RX_FINISHED; ta1js notes it often never fires once the carrier drops.
- **On RX_FINISHED:** emit `F` with words since sync and ms since sync. Near 1024 words means the 11-bit length works; near 128 means it does not.
- **While the squelch is open:** suppress `Draw()` so the 8-word FIFO (107 ms at 1200, 53 ms at 2400) cannot overflow.

### 3.4 BurstFinish (replaces FinishCapture, alert.c:412-425)

1. **Window.** Linearize the ring from max(capture start, squelch-open − 100 ms) to squelch-lost + 40 ms. Positions come from `wordsTotal` recorded at each edge.
2. **Stats.** Compute n, transitions per sample, rk, maxrun and lead (length of the first run):
   - m = max(2, round(0.75k))
   - rk = fraction of runs of length ≥ m
   - noise baseline = 2^(1−m)
3. **Decode** with `ALERT_ScanSamples` (§4): NEG/STD × invert 0/1, preamble gate. If k is below 3, the structure test does not apply.
4. **Report.** Send the ALERT line and history through the existing ProcessCapture path, split so it takes already-decoded readings. Emit `C`, `H` and `X`.
5. **Score.** Per-arrangement uint8 counters (saturating), 8 bytes each:
   - bq: qualifying bursts
   - bb: bursts delivering ≥ 50% of the expected samples (window_ms × TONE2/1000)
   - bs: in-burst syncs, counted over bursts where the engine was searching at open
   - bt: structure passes (rk ≥ 0.80, transitions ≤ 0.6/k, lead ≥ 20k; only for k ≥ 3)
   - dx: bursts with any decode
   - dt: bursts with a table-station decode
   - rep: the largest number of separate bursts in which one station ID decoded
   - ns: noise syncs
6. **Adopt** at rep ≥ 3. Emit `G`, persist, switch to production. Re-arm.

## 4. Decoder change (App/app/alert_decode.c/.h)

New function:

```c
int ALERT_ScanSamples(const uint8_t *buf, uint32_t nsamp, uint32_t spb_q8, uint8_t polarity,
                      bool invert, uint8_t min_idle_bits, AlertReading_t *out, int max_out);
```

Parameters: `spb_q8` = round(256 × TONE2 / 300), which handles fractional k; `min_idle_bits` = 12; `max_gap` = 20. Bits are read MSB-first within each byte, low byte first (ALERT_GetBit, as verified in ta1js).

Algorithm:
1. Apply a median-of-3 filter to the samples on the fly.
2. Find an idle→start edge e.
3. For the first word of a frame, require ≥ min_idle_bits × spb idle samples before e. For edge-sync captures use min(12, sync idle samples / k).
4. Try offsets {−1, 0, +1} samples. For each, sample bit j at the centre e + (j + 0.5)·spb by majority over ±floor(spb/4) samples. Keep the offset with the fewest transitions inside the bit centres.
5. Check start and stop bits. Take data bits LSB-first.
6. Search for the next start from the stop-bit centre. This re-aligns every word, which tolerates ±2-4% transmitter rate error (the AL200X spec allows ±2%).
7. Four words within max_gap feed `ALERT_DecodePayload32`. A frame that begins within 20 bits of an accepted frame's stop is exempt from the gate.

Also add a gated variant of `scan_polarity` (min idle bits before word 0) for the host decimation path.

Tests: extend tools/alert/test_decode.c (already run in CI):
- frames at k ∈ {1.736, 3.47, 4, 4.33, 4.67, 5, 6, 8}, rate error ±2%, a 60-bit preamble, all four sense/framing combinations → must decode;
- 10^5 samples of noise → 0 table hits;
- only 8 idle bits before word 0 → rejected.

## 5. Audio-pin census and ADC decoder

This replaces AdcProbe/ProbeOnce (alert.c:708-749) and the DP32G030 `ENABLE_ALERT_ADC` block (alert.c:39-46, 520-688). PA4 is free because ENABLE_VOICE is false and BOARD_GPIO_Init never touches it.

**Sampler.**
- TIM3: PSC 0, ARR 4999 (9600 Hz), TRGO = update.
- ADC1: sequencer 2 ranks [CH4 = PA4, CH9 = PB1], 41.5-cycle sampling, trigger `LL_ADC_REG_TRIG_EXT_TIM3_TRGO`, external trigger enabled, DMA unlimited.
- DMA1 channel 1 (free; channels 2/3/4/5/7 are used): `LL_SYSCFG_SetDMARemap(DMA1, LL_DMA_CHANNEL_1, LL_SYSCFG_DMA_MAP_ADC1)`, circular, 2 × 64 scans × 2 channels (512 B), HT/TC interrupts.
- `DMA1_Channel1_IRQHandler` (currently the weak default) updates Goertzel accumulators (1300, 1700, 2100) and mean/min/max/sum of squares per channel. When the decoder is on, it feeds the chosen channel into the ALERT_AdcTick body, refactored to take the sample as an argument (tones 2100/1312.5 Hz, spb 32).
- On stop: TIM3 off, DMA off, `BOARD_ADC_Init()`.

**Census at entry.** Each condition takes 1024 scans; drop the first 8 after any change. Conditions:
- PA4 mode: DAC off (analog, high-Z), or DAC1 unbuffered at 0x800 as a mid-rail bias;
- PA8 audio amp off / on;
- source:
  - M: `SetAF(MUTE)`
  - F: AF = FM, idle
  - T13 / T21: `BK4819_PlayTone(f, true); BK4819_ExitTxMute();` wait 20 ms, sample, `EnterTxMute()`. This is the key-beep path: TX DSP on, no PA gain, no RF. Never use `PlaySingleTone` or `EnableTXLink`, which enable PA gain.

Afterwards restore: `BK4819_TurnsOffTones_TurnsOnRX()`, the saved REG_71, `SetAF(FM)`, `ArrArm(current)`, and PA8 from `cfg.monitor`.

Also run a wiring test: each pin as a digital input with pull-up, then pull-down, 5 ms each, reading IDR. Report `PT`.

**Pin passes** if, in some mode / PA8 combination:
- Goertzel(f) under tone f is ≥ 10 dB above the MUTE baseline, for both tones, and
- the matching tone is ≥ 10 dB above the other tone.

Then confirm on the first 3 qualifying bursts: grab during squelch-open and require 1300 and 2100 energy ≥ 10 dB above F-idle. If confirmed, the ADC decoder runs in parallel on every burst. TIM3 runs only while the squelch is open, to avoid the RF self-interference that made fagci remove the same pipeline. Its decodes go through ProcessCapture and are scored as arrangement "ADC" with the same adoption rule. The census also logs the RSSI floor with the sampler on and off.

If audio only appears with PA8 on, the speaker plays the bursts. The owner may want the volume low, but note the level may depend on the knob.

## 6. Hands-off flashing (in this build)

**Firmware side:**

1. **Noinit region.** In Core/py32f071xb.ld, shrink RAM to end at 0x20003FF0, set `_estack = 0x20003FF0`, and add a `NOINIT (NOLOAD)` region at 0x20003FF0-0x20003FFF. It holds a DFU magic (value + complement), the reset reason, and an abnormal-reset counter. The bootloader zero-fills only 0x200001A0-0x2000257F and its stack top is 0x20002580, so it never touches this region.
2. **uart.c, new commands:**
   - `0x05E1` DFU_CHECK: reply `K bl crc=%08X ver=%s ok=%u` over 0x08000000-0x080026FF.
   - `0x05E0` ENTER_DFU (payload magic 0x44465521): refuse unless the CRC32 is 0x22CDCECB and "7.00.07\0" is at 0x0800209A. Otherwise write the noinit magic and call `NVIC_SystemReset()`.
3. **Trampoline.** The first statement of `main()` in Core/Src/main.c, before any peripheral setup:
   1. If the magic is set: clear it; `__disable_irq`; SysTick->CTRL = 0; NVIC ICER/ICPR = all ones.
   2. Pulse the RCC AHB/APB1/APB2/IOP peripheral resets. This detaches USB.
   3. Switch SYSCLK to HSI and wait for it; turn the PLL off and wait for !PLLRDY.
   4. SCB->VTOR = 0x08000000.
   5. Call scatter-load entry 0x080001C9(0x080020C4, 0x20000000, 0x1A0), then 0x08001161(0x080020D8, 0x200001A0, 0x23E0). After this, no app globals may be touched.
   6. Replay main's prologue: RCC->IOPENR |= 7; RCC->APBENR2 |= 0x4001; RCC->APBENR1 |= 0x10800000; RCC->AHBENR |= 1.
   7. Call 0x08000EC9, 0x08000689, 0x08000749, then 0x08000549 with r0 = 0x20001DE0.
   8. In inline asm: MSP = 0x20002568 with a fake {r3-r7, lr} frame whose lr slot points to 0x080000DD (`b .`); r4 = 0x20000020, r5 = 0x2000, r6 = 0x50000800, r7 = 0x50000400; `cpsie i`; `bx 0x080013B3`.

   The bootloader then sets DFU state 1, starts USB, lights the torch, beacons `0x0518`, and after the last page (state 3) boots the new app by itself.
4. **CI step.** Download Ichi's `archive/stock-bl_7.00.07.bin` at a pinned commit, check its sha256, run `arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08000000`, and assert the instruction words at every address above (0x0800137C-0x080013BA, 0x080013B2, 0x08000EC8/EFC, 0x080020A4 table, 0x08000240, 0x0800071E). The build fails if any differ. Also build with `-DENABLE_UART_RW_BK_REGS=ON`, add a `GIT_HASH` define, and grep the binary for the new command strings.

**Safety.** Nothing is erased until page 0 of a new image arrives.
- Trampoline hangs: power cycle; the old app is intact.
- Host dies mid-flash: the bootloader is in state 2 and never times out; resend from page 0.
- Power lost mid-flash: PTT recovery, exactly as today.
- The host never sends `0x0516`, so the bootloader is never written.

**Host side:** tools/hotflash.py.
1. `gh run download` the artifact.
2. Find the port by VID 36B7 (the app and the bootloader both use 0x36b7; App/usb/usbd_cdc_if.c:9).
3. Send `0x05E1`; stop unless ok=1.
4. Send `0x05E0`; wait for re-enumeration and `0x0518` beacons carrying "7.00.07".
5. Run `python tools/serialtool/cli.py flash --port COMx --bl-ver 7.00 f4hwn.rescueops.bin`.
6. Wait for the port to return and compare the hash in the `B` line.

**Bootstrap, once:** radio off, hold PTT, switch on, then either the serialtool command above or UV Studio.

## 7. Robustness (needed for hands-off)

- **HardFault** (Core/Src/py32f071_it.c:57): record reason "fault", increment the counter, `NVIC_SystemReset()`. Today it spins forever.
- **Soft watchdog** in `SysTick_Handler` (App/scheduler.c:48): armed only inside APP_RunAlert. If the loop has not kicked it for 5 s, record "wd" and reset.
- **Reset loops.** Clear the counter after 60 s of healthy running. At 3 consecutive abnormal resets, skip autostart. At 5, enter the DFU trampoline automatically, so a broken build heals into DFU.
- **Autostart.** A 5 s delay after boot, then `gRequestAlertApp = true`, using the existing service point at App/app/app.c:1537. The sweep starts automatically unless an arrangement is adopted.

## 8. USB lines (ASCII, CRLF) and commands

Existing lines are kept: `D`, `A`, `ALERT,`. `D` gains `a<idx>` and `z<pass>`.

| Line | When | Content |
|---|---|---|
| `B` | boot | `B ver=<hash> rst=<por\|sw\|wd\|fault> n=<abnormal count> bl=<crc>` |
| `L` | every arrangement change | `L <idx> <tag> r58=%04X t2=<Hz> r70=%04X t1=<Hz> sy=%08X s4=<0\|1> 5c=%04X pol=<S\|T> ph=<phase> v=<variant>` |
| `C` | each qualifying burst | `C <idx> b=<burst#> pk=<dBm> fl=<dBm> win=<ms> src=<open\|pre\|none> dt=<ms sync−open or NA> pn=<P\|N\|-> w=<words in window> n=<samples> k=<k×100> tr=<‰> rk=<‰> base=<‰> mx=<maxrun> lead=<samples> dec=<n> tab=<n>` |
| `H` | each qualifying burst | `H <idx> <burst#> <byte offset> <up to 64 bytes hex>`, window bytes in stored order (including the prepended sync) |
| `X` | each decode | `X <idx\|ADC> <burst#> id=<id> v=<value> fmt=<ABF\|EIF> pol=<N\|S> inv=<0\|1> pos=<sample> known=<0\|1>` |
| `Y` | each sync, sent after the burst | `Y <idx> ms=<since arm> pn=<P\|N> sq=<0\|1> rssi=<dBm>` |
| `F` | RX_FINISHED | `F <idx> words=<since sync> ms=<since sync>` |
| `P` | census | `P ch=<4\|9> b=<0\|1> pa=<0\|1> src=<M\|F\|T13\|T21\|BURST> mean= pp= rms= g13= g17= g21=` (dB×10) |
| `PT` | wiring test | `PT PA4 u<0\|1> d<0\|1> PB1 u<0\|1> d<0\|1>` |
| `AUD` | census result | `AUD pin=<none\|PA4\|PA4B\|PB1> pa=<0\|1> floor_on=<dBm> floor_off=<dBm>` |
| `Z` | end of each pass | per-arrangement `bq bb bs bt dx dt rep ns` |
| `G` | adoption | `G ADOPT <idx\|ADC> v=<variant> id=<station> rep=<n>` |
| `K` | command ack | |

**New commands.** They use the existing 0xABCD framing, CRC and XOR obfuscation (uart.c:163-166, 641-783). Check at implementation time that the IDs do not clash with anything in the uart.c switch.

| ID | Name | Payload / effect |
|---|---|---|
| 0x0A01 | ARR_SET | `{u8 slot, Arr_t}` into RAM slots 32-39, so any register recipe can be tried without reflashing |
| 0x0A02 | SWEEP_CTL | `{op, a, b}`: stop / start phase / go to arrangement / adopt and persist / clear scores / set dwell / run census / enter ALERT app (op 7, from normal mode) / reboot |
| 0x0A03 | POKE_LIST | up to 8 `{reg, and, or}` entries applied after every arm |
| 0x0601 / 0x0602 | BK register read/write | via `ENABLE_UART_RW_BK_REGS` |

## 9. Host tools and exact judging criteria

**New: tools/alert/radio.py**
- Finds the port by VID 36B7, asserts DTR, and re-asserts it after 10 s without a line.
- Logs every line with a PC timestamp to tools/alert/logs/.
- Sends the commands above, building frames with tools/serialtool/msg.py.

**New: tools/alert/sweep_judge.py** (reads one or more logs). For each arrangement, over its qualifying bursts:
- **BITS** = fraction of bursts with n ≥ 0.5 × win × TONE2/1000.
- **SYNC** = over bursts with src=open: fraction with 0 ≤ dt ≤ 150 ms (or −300 ≤ dt < 0, a sync on the carrier) and the same pn every time.
- **STRUCT** (k ≥ 3), recomputed from `H` under both in-byte bit orders: rk ≥ 0.80, transitions/sample ≤ 0.6/k, lead ≥ 20k. Report which bit order passes.
- **DECODE** = bursts with a table-station decode. Run on the host with both the edge slicer (a port of ALERT_ScanSamples) and phase decimation for integer k.
- **REPEAT** = the largest number of separate bursts in which one station decodes under the same (pol, inv) path.
- **Chance control.** Shuffle each capture's run lengths 200 times and decode with the same gate to get an expected false table-hit rate. Require observed DECODE ≥ 20× that expectation.
- **Noise-sync ratio** = observed `sq=0` sync rate / (2·TONE2/65536 per second, for a 2-byte sync). A ratio above 3 means the sync detector tolerates errors or the demodulated noise is correlated.

Verdicts and exit codes:

| Verdict | Condition | Exit code |
|---|---|---|
| WORKS | REPEAT ≥ 3 and the chance control passes | 0 |
| LIKELY | REPEAT = 2, or ≥ 2 table stations decoded with STRUCT ≥ 2/3 of bursts | 0 |
| BITS-NO-DECODE | STRUCT ≥ 2/3 of bursts, DECODE = 0. The fault is bit order, rate or the decoder; analyse offline with capture_stats.py. | 1 |
| NOISE-BITS | BITS ≥ 0.5 but rk within 0.1 of the noise baseline. That mode does not slice the tones. | – |
| NO-BITS | BITS < 0.2 | – |
| FSK route dead | Every Phase 1 and Phase 3 arrangement has ≥ 2 bursts, S1/S2 each have ≥ 2 stream-coincident bursts, and STRUCT never passes | 2 |
| Not enough data | fewer bursts than the above require | 3 |
| ADC route works | AUD pass, burst confirmation, REPEAT ≥ 3 on idx ADC | 0 |

**Updates to existing tools:**
- capture_stats.py: accept `H` lines and take k from `L`.
- alertmon.py: tolerate the new lines.
- check_menu_wiring.py and check_layout.py: follow the settings rows. Keep the literal "MDM MODE" row (now showing the arrangement tag), because the CI binary check greps for it.

## 10. Change list by file

**App/app/alert.c:**
- 51-125: cfg becomes CFG_MAGIC 0xC0. b[1] = adopted arrangement index, b[2] = route (sweep / FSK / ADC) + ADC pin + PA8, b[3] = variant byte. The 1200-baud cap goes away.
- 173-235: the sweep statics become the Arr_t table plus scores.
- 282-285: FskBase gets preamble 0 and per-arrangement s4/inv.
- 295-384: ArrArm (§3.1).
- 386-404: ring read and SyncN XOR; light re-arm.
- 412-515: BurstFinish (§3.4) and the ModemPoll state machine (§3.3).
- 520-688, 708-749: PY32 sampler, census and ADC decoder (§5).
- 800-888: ProcessCapture split into decode, score and report.
- 964-1053: DrawMain shows the arrangement or adoption and the AUD result.
- 1106-1163: SweepApply/SweepSetRunning replaced.
- 1384-1617: entry sequence (floor settle, census, sweep autostart); remove the timed sweep block (1501-1539); soft-watchdog kick; defer redraw while the squelch is open.

**Other files:**
- App/app/alert_decode.c/.h: ALERT_ScanSamples and the gated scan.
- App/app/uart.c: 0x05E0, 0x05E1, 0x0A01-0x0A03.
- Core/Src/main.c: DFU check first in main().
- Core/Src/py32f071_it.c: HardFault resets.
- App/scheduler.c: soft watchdog.
- App/app/app.c: autostart delay and reset-loop check.
- Core/py32f071xb.ld: NOINIT region.
- .github/workflows/main.yml: `-DENABLE_UART_RW_BK_REGS=ON`, GIT_HASH, bootloader objdump assertions, new tests.
- tools/: §9 and hotflash.py.

## 11. Fallback if every firmware route fails

- **Smallest hardware option:** one 100 nF capacitor from BK4829 pin 8 (EARO; DS-BK4829-E01) to the PA4 net. X1 already contains the matching firmware: on the next boot the census would test PA4 (with the DAC bias) and switch to the ADC decoder by itself if it passes. No reflash. Do not use PB1: it sits at a driven ~0.63 V (about 780 counts) and its purpose is unknown.
- **No-solder alternative:** a 3.5 mm lead from the earphone socket to a PC sound card, running tools/alert/wavdecode.py. That chain already decodes end to end.

## 12. What stays unknown until X1 has run

- Whether RX 111 slices 1300/2100 Hz at all on a BK4829.
- Whether the sync detector matches on a long steady tone (the stream policy works around this).
- Whether TONE2 also scales the FFSK/SAME tones.
- Whether REG_5D's 11-bit length works on BK4829.
- The bit order within bytes. It is assumed from ta1js on BK4819; the judge checks both orders.
- Whether PB1 or PA4 carries AF.
- Whether the trampoline's replayed init is complete. The addresses are checked in CI, but the behaviour is not.

X1 records evidence on each of these automatically within roughly 1-2 hours of traffic at about 3 bursts per minute.
---

## 13. Implementation contract (binding on every implementer)

Branch `alert-x1`. No local ARM compiler exists; CI (`gh workflow run main.yml --ref alert-x1`) is the compiler.
Implementers do NOT run CI themselves and do NOT commit or push; the integrator does. Implementers must not
touch files owned by another stream. Code style: match the surrounding code (tabs in App/app/alert*.c, 4 spaces in
most F4HWN files), comment density like alert.c (explain *why*), C11, no dynamic allocation, no printf of floats.
Budget: at most ~4 KB extra RAM and ~20 KB extra flash across all streams.

### Ownership

| Stream | Owns (only these files) |
|---|---|
| A decoder | App/app/alert_decode.c, App/app/alert_decode.h, tools/alert/test_decode.c, tools/alert/scan_samples.py (new, Python port of ALERT_ScanSamples) |
| B sweep | App/app/alert.c, App/app/alert.h, tools/alert/check_layout.py, tools/alert/check_menu_wiring.py |
| C adc | App/app/alert_adc.c, App/app/alert_adc.h (both new; already listed in App/CMakeLists.txt under ENABLE_ALERT) |
| D dfu | App/app/dfu.c, App/app/dfu.h (new; already in App/CMakeLists.txt main list), Core/Src/main.c, Core/py32f071xb.ld (or whichever linker script the RescueOps build uses), Core/Src/py32f071_it.c, App/scheduler.c, App/app/app.c, App/app/uart.c, CMakePresets.json, .github/workflows/main.yml, tools/hotflash.py (new), tools/serialtool/* |
| E host | tools/alert/radio.py (new), tools/alert/sweep_judge.py (new), tools/alert/capture_stats.py, tools/alert/alertmon.py, tools/alert/README.md |

### A → B: decoder API (alert_decode.h)

```c
// Decode async ALERT frames from an oversampled sample stream (bit n = ALERT_GetBit order).
// spb_q8 = samples per ALERT bit in Q8 (round(256 * TONE2_Hz / 300)); must handle 1.5 <= spb <= 9.
// polarity: ALERT_POL_STANDARD / ALERT_POL_NEGATIVE / ALERT_POL_ANY. invert complements samples.
// min_idle_bits: idle bits required before the first word of a frame (12 normally).
// Frames starting within 20 bits of a previously accepted frame's stop are exempt from the idle gate.
// bit_pos in the output = sample index of the frame's first start edge.
int ALERT_ScanSamples(const uint8_t *buf, uint32_t nsamp, uint32_t spb_q8, uint8_t polarity,
                      bool invert, uint8_t min_idle_bits, AlertReading_t *out, int max_out);
```
Stack use must stay under 256 bytes; no static buffers larger than 64 bytes (RAM is tight).

### C → B: ADC API (alert_adc.h, compiled only with ENABLE_ALERT, PY32F071 only)

```c
typedef void (*AlertAdcEmit_t)(const char *line);   // B passes its DbgSend

// Run the audio-pin census (§5). Blocking, <= 4 s. Emits P / PT / AUD lines via emit.
// Leaves the BK4819 in RX with AF=FM, tones off, REG_71 restored; the caller re-arms the modem afterwards.
// Returns true if a pin passed; the chosen pin/PA8 setting is remembered internally.
bool ALERTADC_Census(AlertAdcEmit_t emit);
void ALERTADC_SetChoice(uint8_t pin, bool pa8);     // restore a persisted choice (pin: 0 none, 1 PA4, 2 PA4 with DAC bias, 3 PB1)
uint8_t ALERTADC_ChoicePin(void);
bool    ALERTADC_ChoicePa8(void);

// Squelch open: start TIM3/ADC/DMA sampling of the chosen pin and the software AFSK demodulator
// (tones 2100/1300 Hz, 300 baud; port of the old ALERT_AdcTick, now fed from the DMA IRQ).
void ALERTADC_BurstStart(void);
// Squelch closed: stop sampling. Returns number of demodulated bits (one per ALERT bit, 300 baud)
// copied to buf (MSB-first per byte, ALERT_GetBit order), at most maxbits.
uint16_t ALERTADC_BurstStop(uint8_t *buf, uint16_t maxbits);
// Burst confirmation helper (§5): energy at 1300/2100 during the last burst vs idle, dB x10.
void ALERTADC_LastBurstEnergy(int16_t *g13, int16_t *g21, int16_t *idle);
// Leave everything as BOARD_ADC_Init() configured it (battery measurement keeps working).
void ALERTADC_Shutdown(void);
```

### D → B: robustness hooks (dfu.h)

```c
void DFU_WatchdogArm(bool on);   // B: on at APP_RunAlert entry, off at exit
void DFU_WatchdogKick(void);     // B: once per main-loop pass inside APP_RunAlert
```
D wires uart.c so the ALERT host commands below reach B, and prints the `B` boot line.

### B → D: host command hooks (alert.h, called from uart.c)

```c
// 0x0A01 ARR_SET, 0x0A02 SWEEP_CTL, 0x0A03 POKE_LIST. payload/len are the command body after the
// 4-byte message header (ID + length) and any timestamp D's framing requires; B defines the byte layouts
// and documents them in a comment. Return false on malformed input; D replies with a K line either way.
bool ALERT_HostArrSet(const uint8_t *payload, uint16_t len);
bool ALERT_HostSweepCtl(const uint8_t *payload, uint16_t len);
bool ALERT_HostPokeList(const uint8_t *payload, uint16_t len);
```
SWEEP_CTL op 7 "enter the ALERT app from normal mode" is implemented by B setting `gRequestAlertApp = true`.
Autostart (D, in app.c): 5 s after boot, unless the abnormal-reset counter is >= 3, set `gRequestAlertApp = true` once.

### Line formats
Exactly as §8. Every line ends CRLF, is ASCII, is at most 200 characters, and is sent through the existing DbgSend-style
USB/UART path (B: DbgSend in alert.c; C: the emit callback; D: its own helper in dfu.c using the same path).
