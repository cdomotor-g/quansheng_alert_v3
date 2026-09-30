/* ALERT receiver: what the app's own modules share.
 *
 *   alert.c          entry/loop, keys, burst -> decode -> record, cfg
 *   alert_ui.c       every screen
 *   alert_log.c      SPI-flash ring log            (alert_log.h)
 *   alert_stn.c      station lookup                (alert_stn.h)
 *   alert_console.c  text console over USB CDC     (alert_console.h)
 *
 * Nothing outside those files includes this; the rest of the firmware sees
 * app/alert.h only. The types and the functions marked [spec] are the
 * contract of tools/alert/V2_SPEC.md section 2 and are frozen: change the
 * spec first. The rest are what the modules needed from each other when
 * alert.c was split, declared here rather than reached for through globals.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifndef APP_ALERT_INT_H
#define APP_ALERT_INT_H

#ifdef ENABLE_ALERT

#include <stdbool.h>
#include <stdint.h>

#include "driver/keyboard.h"

// ---------------------------------------------------------------------------
// one decoded frame [spec]: 24 bytes, also the log payload

typedef struct {
	uint32_t epoch;          // UTC seconds if the clock was set, else 0
	uint32_t uptime_ms;      // ms since boot at squelch close
	uint16_t boot;           // boot counter (persisted by the log), 0 if no log
	uint16_t id;             // 13-bit ALERT address
	uint16_t value;          // 11-bit raw value
	uint8_t  fmt;            // ALERT_FMT_*
	uint8_t  flags;          // ALERTREC_*
	int8_t   rssi;           // dBm, peak during the burst
	int8_t   nf;             // dBm, averaged noise floor at the time
	uint16_t burst_ms;       // squelch-open duration
	uint32_t payload;        // the 32 decoded data bits (ALERT_DecodePayload32 input)
} AlertRecord_t;

// The log stores these raw, so the layout is part of the on-flash format.
_Static_assert(sizeof(AlertRecord_t) == 24, "AlertRecord_t is the 24-byte log payload");

// AlertRecord_t.flags
#define ALERTREC_POL_STD      0x01u    // framed idle-high (ALERT_POL_STANDARD)
#define ALERTREC_INV          0x02u    // decoded only with the bits complemented
#define ALERTREC_TABLE        0x04u    // the id is in the station table
#define ALERTREC_FRAME_SHIFT  4u       // <7:4> frame index within its burst, 0-15
#define ALERTREC_FRAME(f)     ((uint8_t)((f) >> ALERTREC_FRAME_SHIFT))

// ---------------------------------------------------------------------------
// settings (V2_SPEC section 5), persisted in gEeprom.ALERT_CFG by alert.c

enum {
	ALERT_SPK_OFF = 0,      // speaker path always off
	ALERT_SPK_SQL,          // on only while the squelch is open
	ALERT_SPK_ON,           // always on (listen to the channel)
};

typedef struct {
	bool    voice;          // VOICE: read new readings out loud
	uint8_t speaker;        // SPEAKER: ALERT_SPK_*
	bool    csv;            // CSV OUT: CSV lines on USB / UART
	bool    log;            // LOG: append to the SPI-flash log when it is usable
	bool    show_unknown;   // UNKNOWN: show addresses not in the station table
	bool    confirm;        // CONFIRM: require the same reading twice in one burst
	bool    gate;           // SQ GATE: a burst must peak well over the floor to count
	uint8_t snr_req;        // SNR REQ: dB over the noise floor a burst needs to decode
	bool    debug;          // DEBUG: D heartbeat and raw A lines
	uint8_t polarity;       // ALERT_POL_*: framing tried by the decoder (ANY normally)
	uint8_t adc_pin;        // census choice, ALERTADC_PIN_*
	bool    adc_pa8;        // ... and whether it needs PA8 on
	bool    adc_ok;         // that choice has decoded: no census at entry
} AlertCfg_t;

extern AlertCfg_t gAlertCfg;             // [spec] defined in alert.c

#define ALERT_SNR_MIN      6u
#define ALERT_SNR_MAX      20u
#define ALERT_SNR_DEFAULT  12u

// ---------------------------------------------------------------------------
// alert.c exports

int8_t   ALERT_NoiseFloor(void);         // [spec] averaged NF, dBm
int8_t   ALERT_Rssi(void);               // [spec] instantaneous, dBm
bool     ALERT_SquelchOpen(void);        // [spec]
uint32_t ALERT_Epoch(void);              // [spec] UTC seconds, 0 if unset
void     ALERT_SetEpoch(uint32_t epoch); // [spec] RAM only: lost on reboot
uint8_t  ALERT_HistoryCount(void);       // [spec] RAM ring, newest first
const AlertRecord_t *ALERT_History(uint8_t back);   // [spec] NULL past the end
void     ALERT_Emit(const char *line);   // [spec] one line out (USB CDC; see alert.c)
void     ALERT_SettingsChanged(void);    // [spec] persist gAlertCfg at the next quiet moment

uint32_t ALERT_UptimeMs(void);           // ms since boot, 10 ms steps
uint16_t ALERT_Bursts(void);             // qualifying bursts since boot
uint32_t ALERT_Decodes(void);            // frames delivered since boot (never reset)
int16_t  ALERT_LastPeak(void);           // peak RSSI of the last qualifying burst
void     ALERT_RequestRedraw(void);      // redraw the screen at the next quiet moment

// Where the app is: settling the RSSI floor, running the census, receiving.
enum { ALERT_ENTRY_SETTLE = 0, ALERT_ENTRY_CENSUS, ALERT_ENTRY_RUN };
uint8_t  ALERT_EntryState(uint8_t *secsLeft);   // *secsLeft set while SETTLE (may be NULL)

// The ADC route: census result and burst confirmation.
enum { ALERT_ADC_UNTESTED = 0, ALERT_ADC_OFF, ALERT_ADC_CONFIRM, ALERT_ADC_ON, ALERT_ADC_REJECT };
uint8_t  ALERT_AdcState(void);
bool     ALERT_CensusPending(void);      // requested, or running now
const char *ALERT_AudName(void);         // census pin: "NONE", "PA4", "PA4B", "PB1"

// Settings rows, shared by the settings view and the console. Names are at
// most 9 characters, values at most 8 (the settings row is 1 + 9 + 8 = 18).
uint8_t     ALERT_SetCount(void);
const char *ALERT_SetName(uint8_t row);
void        ALERT_SetValue(uint8_t row, char *out);   // out: >= 9 bytes
void        ALERT_SetStep(uint8_t row, int dir);      // +1 UP, -1 DOWN; takes effect at once

// ---------------------------------------------------------------------------
// alert_ui.c

enum { ALERT_VIEW_MAIN = 0, ALERT_VIEW_SETTINGS, ALERT_VIEW_DETAIL };

void    ALERTUI_Draw(void);                      // [spec] current view, then blit
void    ALERTUI_Key(KEY_Code_t key, bool held);  // [spec] UI-level keys
void    ALERTUI_Tick10ms(void);                  // [spec] marquee, idle snap-back
void    ALERTUI_DrawStatus(void);                // status line only (every 100 ms)
uint8_t ALERTUI_View(void);                      // ALERT_VIEW_*
void    ALERTUI_Reset(void);                     // app entry: main view, top of the list

#endif // ENABLE_ALERT

#endif
