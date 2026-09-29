/* ALERT telemetry receiver app for the Quansheng UV-K5 (DP32G030 + BK4819).
 *
 * Receives legacy ALERT (ERTS) flood-warning telemetry - 300 baud AFSK, mark
 * 2100 Hz and space 1300 Hz (V.23 mode 2 tones, measured off air; this file
 * used to say Bell-202 1200/2200, which was never checked), four 10-bit async
 * words carrying a 13-bit station address and an 11-bit value - decodes it,
 * looks the address up in the MegaNet-derived
 * station table, shows it on the LCD and optionally reads it out with the
 * radio's voice prompts.
 *
 * Two signal inputs are supported, selectable at run time:
 *
 *   MODEM  The BK4819's own FSK engine is used as a bit slicer: it is put in
 *          FFSK 1200/1800 receive mode with its bit clock set to the ALERT
 *          baud rate and a sync word equal to the idle preamble pattern, so
 *          every burst is captured into the FSK FIFO as raw bits, which are
 *          then framed in software (app/alert_decode.c). No hardware change.
 *
 *   ADC    (ENABLE_ALERT_ADC) The discriminator audio is sampled by the MCU's
 *          SAR ADC at 9600 Hz and demodulated in software (2100/1300 Hz tone
 *          correlators + bit clock recovery). Needs a one-wire hardware mod:
 *          BK4819 pin 8 (EARO) -> 100 nF -> DP32G030 pin 9 (PA8 / ADC CH3).
 *          See README.md.
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

#ifdef ENABLE_ALERT_ADC
// Called from the SysTick handler at 9600 Hz while gAlertAdcRun is set.
extern volatile bool gAlertAdcRun;
void ALERT_AdcTick(void);
#endif

#endif

#endif
