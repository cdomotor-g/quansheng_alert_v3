/* ALERT telemetry receiver app for the Quansheng UV-K5 V3 (PY32F071 + BK4829).
 *
 * Receives legacy ALERT (ERTS) flood-warning telemetry - 300 baud AFSK, mark
 * 2100 Hz and space 1300 Hz (V.23 mode 2 tones, measured off air; this file
 * used to say Bell-202 1200/2200, which was never checked), four 10-bit async
 * words carrying a 13-bit station address and an 11-bit value - decodes it,
 * looks the address up in the MegaNet-derived station table, shows it on the
 * LCD and optionally reads it out with the radio's voice prompts.
 *
 * Two routes to the bits, both tried automatically (tools/alert/X1_PLAN.md):
 *
 *   FSK    The BK4829's own FSK engine, put into a sub-carrier receive mode
 *          with its TONE2 bit clock oversampling the 300 baud signal. The
 *          engine's words are kept in a ring buffer between the squelch edges
 *          and framed in software (ALERT_ScanSamples). Which register recipe
 *          works is not known, so the app sweeps a table of them, paced to
 *          real bursts, and adopts the first that decodes one table station
 *          in three separate bursts.
 *
 *   ADC    Receiver audio on an MCU ADC pin (PA4 or PB1), if the census at
 *          app entry finds any, demodulated in software (app/alert_adc.c).
 *          It runs alongside the FSK engine on every burst and is scored and
 *          adopted by the same rule.
 *
 * Everything the app learns goes out over USB-C as one-letter lines (§8 of the
 * plan) so a script can judge each route without anyone watching the screen.
 */
#ifndef APP_ALERT_H
#define APP_ALERT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef ENABLE_ALERT

// Modal app: takes over the radio and the screen until EXIT is pressed.
void APP_RunAlert(void);

// Load / store the app's configuration bytes (gEeprom.ALERT_CFG).
void ALERT_LoadConfig(void);
void ALERT_StoreConfig(void);

// Host commands, called from uart.c. payload/len are the command body after
// the message header (and any timestamp the framing carries). Multi-byte
// fields are little-endian. Each returns false on malformed input. Commands
// that touch the radio are queued (four deep; false when full) and run at the
// next moment the squelch is shut, so a burst in progress is never disturbed.
//
// 0x0A01 ARR_SET, 21 bytes - a register recipe into RAM slot 32..39:
//    [0]      slot, 32..39
//    [1..3]   tag, up to 3 printable ASCII characters, NUL padded ("H<n>" if empty)
//    [4..5]   REG_58
//    [6..7]   REG_70 (<15> set: TONE1 is written too)
//    [8..9]   TONE1 Hz, 0..3000
//    [10..11] TONE2 Hz, 1..3000; 0 empties the slot
//    [12..13] REG_5C
//    [14..15] REG_59 base: <10> invert, <8>, <7:4> preamble, <3> 4-byte sync,
//             <2:0>; bits 15:11 are ignored (the app drives FIFO/enable)
//    [16..17] REG_5A (sync bytes 0, 1)
//    [18..19] REG_5B (sync bytes 2, 3; written only with REG_59<3>)
//    [20]     flags: <0> stream policy (never re-arm on the squelch)
//    Loaded slots are swept at the end of every Phase 1 pass. A slot is RAM
//    only: an adoption of one does not survive a reboot.
//
// 0x0A02 SWEEP_CTL, 3 bytes {op, a, b}:
//    0 stop sweeping, hold the current arrangement
//    1 start phase a (1, 2 or 3) from its first entry
//    2 go to arrangement a with variant byte b (the sweep, if running, carries
//      on from its own position afterwards)
//    3 adopt arrangement a / variant b and persist; a = 0xFE adopts the ADC
//      route, a = 0xFF clears the adoption and resumes the sweep
//    4 clear all scores and repeat counts
//    5 set the dwell to a qualifying bursts per arrangement per pass (0 -> 1)
//    6 run the audio-pin census again
//    7 enter the ALERT app (from normal mode; ignored inside it)
//    8 reboot (outside the app at once, acked first)
//    Outside the app only 7 and 8 are accepted; the rest act on a running
//    sweep and are refused rather than queued for some later entry.
//    Arrangement indices: 0..22 the Phase 1 table, 32..39 host slots, 64..103
//    the Phase 3 REG_58 codes. Variant byte: high nibble operation, low nibble
//    argument - 0x1g RX gain g, 0x20 RX BW 100 (FFSK1218 only), 0x30 4-byte
//    sync, 0x40 invert, 0x5p preamble nibble p, 0x60 REG_59<8>, 0x7n
//    REG_59<2:0> = n; 0x00 none.
//
// 0x0A03 POKE_LIST, 1 + 5n bytes - applied after every full arm:
//    [0]      n, 0..8 (0 clears the list)
//    then n x {u8 reg (0x00..0x7F), u16 and-mask, u16 or-mask}:
//    reg = (reg & and-mask) | or-mask
//    The transmitter's registers are refused: a list naming REG_30, REG_33 or
//    REG_36 is rejected whole, and REG_59<11> (FSK TX enable) is forced clear.
bool ALERT_HostArrSet(const uint8_t *payload, uint16_t len);
bool ALERT_HostSweepCtl(const uint8_t *payload, uint16_t len);
bool ALERT_HostPokeList(const uint8_t *payload, uint16_t len);

#endif

#endif
