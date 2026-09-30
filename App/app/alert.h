/* ALERT telemetry receiver app for the Quansheng UV-K5 V3 (PY32F071 + BK4829).
 *
 * Receives legacy ALERT (ERTS) flood-warning telemetry - 300 baud AFSK, mark
 * 2100 Hz and space 1300 Hz (V.23 mode 2 tones, measured off air), four
 * 10-bit async words carrying a 13-bit station address and an 11-bit value -
 * decodes it, looks the address up in the MegaNet-derived station table,
 * shows it on the LCD and optionally reads it out with the voice prompts.
 *
 * The bits come from the receiver's demodulated audio on an MCU ADC pin
 * (PA4, found by the audio-pin census at first entry and remembered once it
 * has decoded), demodulated in software while the squelch is open
 * (app/alert_adc.c) and framed by app/alert_decode.c. The BK4829's own FSK
 * engine never synced on an ALERT burst in 23 register arrangements and is
 * not used (tools/alert/README.md).
 *
 * The product this is growing into is tools/alert/V2_SPEC.md; the modules and
 * what they share are described in app/alert_int.h.
 */
#ifndef APP_ALERT_H
#define APP_ALERT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef ENABLE_ALERT

// Modal app: takes over the radio and the screen until EXIT is pressed.
void APP_RunAlert(void);

#endif

#endif
