/* ALERT telemetry receiver app - see alert.h for the overview, alert_int.h
 * for how the modules divide the work.
 *
 * This file owns the radio: entry and the main loop, the keys that act on the
 * receiver, squelch edges -> ADC burst -> decode -> record, the configuration
 * and the lines that go out over USB.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <string.h>

#include "app/alert.h"
#include "app/alert_adc.h"
#include "app/alert_console.h"
#include "app/alert_decode.h"
#include "app/alert_int.h"
#include "app/alert_log.h"
#include "app/alert_stn.h"
#include "app/dfu.h"
#if defined(ENABLE_UART) || defined(ENABLE_USB)
#include "app/uart.h"
#endif
#include "audio.h"
#include "driver/backlight.h"
#include "driver/bk4819.h"
#include "driver/keyboard.h"
#include "driver/gpio.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "board.h"
#include "external/printf/printf.h"
#include "functions.h"
#include "misc.h"
#include "py32f0xx.h"
#include "radio.h"
#include "scheduler.h"
#include "settings.h"
#ifdef ENABLE_UART
	#include "driver/uart.h"
#endif
#ifdef ENABLE_USB
	#include "driver/vcp.h"
#endif

// ---------------------------------------------------------------------------
// configuration (persisted in gEeprom.ALERT_CFG[5], see settings.c)
//
//   b[0]  <0> CSV OUT, <2:1> polarity, <3> voice, <4> SQ GATE,
//         <5> show unknown, <6> LOG, <7> DEBUG
//   b[1]  <1:0> SPEAKER (ALERT_SPK_*), <7:3> SNR REQ in dB
//   b[2]  <1:0> 2 = the census choice below has decoded, <3:2> ADC pin
//         (ALERTADC_SetChoice numbering), <4> PA8 needed by that pin
//   b[3]  free
//   b[4]  <7:4> CFG_MAGIC, <3:1> layout version, <0> confirm
//
// Layout 0 is the X1 sweep build's: b[0]<6> bit reverse, <7> monitor, b[1]
// and b[3] the adopted arrangement and variant, and b[2]<1:0> the adopted
// route, where 2 was the ADC. b[2] was kept bit for bit, so an ADC adoption
// made by the sweep reads as a confirmed census choice and this radio skips
// the census on its first entry after the update.

#define CFG_MAGIC   0xC0u
#define CFG_VERSION 1u
#define CFG_ADC_OK  2u     // b[2]<1:0>

AlertCfg_t gAlertCfg;

static void DefaultConfig(void)
{
	// ANY, not NEGATIVE: nothing has established the sense on this radio, and
	// it is the table-station and repeat rules that keep chance hits out, not
	// a guess at the polarity. The rest are V2_SPEC section 5's defaults.
	gAlertCfg.polarity     = ALERT_POL_ANY;
	gAlertCfg.voice        = false;
	gAlertCfg.speaker      = ALERT_SPK_SQL;
	gAlertCfg.csv          = true;
	gAlertCfg.log          = true;
	gAlertCfg.show_unknown = true;
	gAlertCfg.confirm      = false;
	gAlertCfg.gate         = true;
	gAlertCfg.snr_req      = ALERT_SNR_DEFAULT;
	gAlertCfg.debug        = false;
	gAlertCfg.adc_pin      = 0;
	gAlertCfg.adc_pa8      = false;
	gAlertCfg.adc_ok       = false;
}

static void LoadConfig(void)
{
	const uint8_t *b = gEeprom.ALERT_CFG;

	DefaultConfig();
	if ((b[4] & 0xF0u) != CFG_MAGIC)
		return;

	const uint8_t ver = (b[4] >> 1) & 7u;
	gAlertCfg.polarity     = (b[0] >> 1) & 3u;
	gAlertCfg.voice        = b[0] & (1u << 3);
	gAlertCfg.gate         = b[0] & (1u << 4);
	gAlertCfg.show_unknown = b[0] & (1u << 5);
	gAlertCfg.confirm      = b[4] & 1u;
	gAlertCfg.adc_ok       = (b[2] & 3u) == CFG_ADC_OK;
	gAlertCfg.adc_pin      = (b[2] >> 2) & 3u;
	gAlertCfg.adc_pa8      = b[2] & (1u << 4);
	if (ver == 0) {
		// X1 layout: the new settings take their defaults, and MONITOR ON
		// carries over as SPEAKER ON
		if (b[0] & (1u << 7))
			gAlertCfg.speaker = ALERT_SPK_ON;
	} else {
		gAlertCfg.csv     = b[0] & 1u;
		gAlertCfg.log     = b[0] & (1u << 6);
		gAlertCfg.debug   = b[0] & (1u << 7);
		gAlertCfg.speaker = b[1] & 3u;
		gAlertCfg.snr_req = b[1] >> 3;
	}
	if (gAlertCfg.polarity > ALERT_POL_ANY)  gAlertCfg.polarity = ALERT_POL_ANY;
	if (gAlertCfg.speaker > ALERT_SPK_ON)    gAlertCfg.speaker  = ALERT_SPK_SQL;
	if (gAlertCfg.snr_req < ALERT_SNR_MIN || gAlertCfg.snr_req > ALERT_SNR_MAX)
		gAlertCfg.snr_req = ALERT_SNR_DEFAULT;
	if (!gAlertCfg.adc_pin)
		gAlertCfg.adc_ok = false;   // nothing to skip the census for
}

static void StoreConfig(void)
{
	uint8_t *b = gEeprom.ALERT_CFG;
	b[0] = (uint8_t)((gAlertCfg.csv ? 1u : 0u) | ((gAlertCfg.polarity & 3u) << 1) |
	                 (gAlertCfg.voice << 3) | (gAlertCfg.gate << 4) | (gAlertCfg.show_unknown << 5) |
	                 (gAlertCfg.log << 6) | (gAlertCfg.debug << 7));
	b[1] = (uint8_t)((gAlertCfg.speaker & 3u) | ((gAlertCfg.snr_req & 31u) << 3));
	b[2] = (uint8_t)((gAlertCfg.adc_ok ? CFG_ADC_OK : 0u) | ((gAlertCfg.adc_pin & 3u) << 2) |
	                 (gAlertCfg.adc_pa8 << 4));
	b[3] = 0;
	b[4] = (uint8_t)(CFG_MAGIC | (CFG_VERSION << 1) | (gAlertCfg.confirm ? 1u : 0u));
}

// ---------------------------------------------------------------------------
// state

// The RAM history: a ring, newest at histHead. The log (when there is one)
// holds the long history; this is what the screens can always rely on.
#define HISTORY_N 16u
static AlertRecord_t hist[HISTORY_N];
static uint8_t       histHead;       // index of the newest record
static uint8_t       histCount;

static struct {
	uint16_t frames;     // readings delivered
	uint16_t gated;      // squelch openings that did not qualify as a burst
	uint16_t unknown;    // frames whose address is not in the table
	uint16_t inv;        // frames that only decoded with the bits complemented
	uint16_t sqcap;      // squelch openings
} stats;

#define ALERT_DEFAULT_FREQ 15150000u   // 151.500 MHz, in units of 10 Hz

static uint16_t dbgIrqCount;     // squelch interrupts serviced
static int16_t  dbgRawKey;       // what KEYBOARD_GetKey returned THIS pass
static uint8_t  dbgRawPtt;       // PTT GPIO read directly, this pass

static bool     sqOpen;
static bool     sqPrev;         // for edge detection: bursts, not samples
static uint16_t burstCount;     // qualifying bursts since boot
static uint32_t decodesTotal;   // frames delivered since boot
// A squelch opening is not the same thing as a transmission. At SQL 1.0 the
// squelch flickers on noise; a burst only counts if it rose well clear of the
// noise, and the floor is measured rather than assumed so it works wherever
// the radio happens to be.
#define BURST_MARGIN_DB 15
static int16_t  rssiFloor;      // slow minimum: the noise floor, in dBm
static int16_t  burstPeak;      // strongest RSSI during the burst in progress
static int16_t  lastPeak;       // ... of the last qualifying burst, for display
static int16_t  rssiDbm;
static bool     running;
static bool     redraw;
static bool     persistReq;     // gAlertCfg to flash at the next quiet moment
static uint16_t tick;
#ifdef ENABLE_VOICE
static bool     voiceBusy;
#endif

// UTC, once a host has set it: the epoch at a SysTick count, advanced from it.
static uint32_t epochBase;
static uint32_t epochTicks;

// A millisecond clock at 10 ms resolution, advanced by the main loop's
// timeslice. The scheduler's own counter keeps running while the loop blocks
// and this does not, which is what the burst timing below was tuned against:
// nothing that blocks is allowed to run while a burst is in progress, the one
// time the clock has to be right to a tick or two.
static uint32_t nowMs;

// App entry: settle the RSSI floor, run the audio-pin census, then receive.
// With a confirmed census choice from an earlier entry there is no census,
// and the settle only has to seed the floor.
enum { ENTRY_SETTLE = ALERT_ENTRY_SETTLE, ENTRY_CENSUS = ALERT_ENTRY_CENSUS, ENTRY_RUN = ALERT_ENTRY_RUN };
#define SETTLE_MS       10000u
#define SETTLE_KNOWN_MS 1000u
static uint8_t  entry;
static uint32_t entryMs;
static uint32_t settleMs;
static bool     censusReq;       // run the census again at the next quiet moment
static bool     censusBusy;      // for the screen: the census blocks for seconds

// ---------------------------------------------------------------------------
// USB lines

// Debug telemetry over USB. UART_ServiceCommands() runs on every pass of the
// main loop below, and cdc_acm_data_send_with_dtr() is a no-op unless a host
// has the port open with DTR asserted, so this costs nothing when nobody is
// listening. When someone is, it blocks until the bytes are gone, which is why
// no line is ever sent while the squelch is open.
static void DbgSend(const char *s)
{
#ifdef ENABLE_USB
	VCP_SendStr(s);
#else
	(void)s;
#endif
}

// USB only, like every line before it: the UART at 38400 would hold the loop
// for ~25 ms a line, and only the ALERT, line has ever gone out there.
void ALERT_Emit(const char *line)
{
	DbgSend(line);
}

// One buffer for every line this file builds. DbgSend returns only once the
// bytes are gone, so no two lines are ever alive at once, and RAM is too short
// to give each line type its own. Every format was sized to stay under 200.
static char lb[208];

static const char hexDigits[] = "0123456789ABCDEF";

static char *PutHex(char *d, const uint8_t *s, uint16_t n)
{
	while (n--) {
		*d++ = hexDigits[*s >> 4];
		*d++ = hexDigits[*s++ & 15u];
	}
	*d = 0;
	return d;
}

// ---------------------------------------------------------------------------
// the receiver

// ADC route state (the sampler and demodulator are in alert_adc.c)
enum {
	ADCST_UNTESTED = ALERT_ADC_UNTESTED, ADCST_OFF = ALERT_ADC_OFF, ADCST_CONFIRM = ALERT_ADC_CONFIRM,
	ADCST_ON = ALERT_ADC_ON, ADCST_REJECT = ALERT_ADC_REJECT
};
static uint8_t  adcState;
static bool     adcRunning;      // BurstStart called, BurstStop not yet
static uint8_t  confN, confOk;   // burst confirmation, first three qualifying bursts
static uint8_t  adcBits[64];     // one burst at 300 baud: 512 bits is 1.7 s
static uint16_t lastBits;        // bits the demodulator gave for the last burst

// Only SPEAKER ON opens the path for now. SQL (on while the squelch is open)
// arrives with V2 Phase B, after the decode rate with it is checked on air;
// until then it behaves as OFF, exactly as MONITOR OFF did. When the census
// found audio only with PA8 on, ALERTADC_BurstStart switches it on for the
// burst and BurstStop puts it back, so the speaker is not left hissing between
// bursts on the ADC's account.
static void AudioPath(void)
{
	if (gAlertCfg.speaker == ALERT_SPK_ON) AUDIO_AudioPathOn(); else AUDIO_AudioPathOff();
}

// burst state
static uint32_t openMs, lostMs, lostUp;
static bool     draining;        // squelch shut, the burst not finished for DRAIN_MS
static bool     burstTainted;    // the receiver was re-armed under it
// The loop was just blocked - an arm (~30 ms), the census (~3 s), a flash
// write. A squelch edge first seen right after one may be long past: the
// opening was never sampled, so that burst is tainted rather than decoded.
static bool     blocked;

// DRAIN_MS was the FSK FIFO's: no word left stranded when the carrier drops.
// It stays because the demodulator has been running across it all along, so
// the ADC's bits and a squelch flicker's being one burst are unchanged.
#define DRAIN_MS       40u       // squelch lost -> BurstFinish
#define BURST_MAX_MS   1500u     // longer than this is not an ALERT burst

// Plain receive with the AF at FM: the ADC route takes the demodulated audio
// from PA4, and nothing else is needed from the chip. This is what the FSK
// arm's prologue did, and it is kept whole - the DSP restart takes REG_30
// through zero and back (BK4819_ResetFSK idles it and switches the FSK engine
// off, BK4819_RX_TurnOn brings it back up) - because the census and
// BK4819_SetupSquelch leave the chip in states this app has only ever
// received from after one. The squelch interrupts stay enabled as they always
// were: REG_0C<1> is what the loop reads, and the bounded service in RxPoll
// keeps REG_02 clear.
static void RxArm(void)
{
	BK4819_ResetFSK();
	BK4819_WriteRegister(BK4819_REG_02, 0);
	BK4819_WriteRegister(BK4819_REG_3F, 0);
	BK4819_RX_TurnOn();
	// Tones off: the state the census found the audio on PA4 in. The FSK arm
	// used to leave TONE1/TONE2 running here as the engine's bit clock.
	BK4819_WriteRegister(BK4819_REG_70, 0);
	BK4819_WriteRegister(BK4819_REG_3F, BK4819_REG_3F_SQUELCH_FOUND | BK4819_REG_3F_SQUELCH_LOST);
	BK4819_WriteRegister(BK4819_REG_02, 0);

	// AF stays at FM whatever the speaker is doing: the ADC samples the
	// demodulated audio, and the speaker has its own gate.
	BK4819_SetAF(BK4819_AF_FM);
	AudioPath();

	blocked = true;
	if (sqOpen || draining)
		burstTainted = true;
}

// Persist gAlertCfg. Blocking (a flash write), so only at a quiet moment.
static void Persist(void)
{
	StoreConfig();
	DFU_WatchdogKick();
	SETTINGS_SaveSettings();
	DFU_WatchdogKick();
	blocked = true;
}

void ALERT_SettingsChanged(void)
{
	StoreConfig();
	persistReq = true;
}

// ---------------------------------------------------------------------------
// decoded readings

#ifdef ENABLE_VOICE
static void SpeakDigits(uint16_t id, uint16_t value)
{
	// station id then value, digit by digit (the voice ROM has digits only)
	uint8_t n = 0;
	uint16_t v[2] = { id, value };
	gVoiceWriteIndex = 0;
	gVoiceReadIndex  = 0;
	for (uint8_t k = 0; k < 2; k++) {
		uint8_t d[5], nd = 0;
		do { d[nd++] = v[k] % 10u; v[k] /= 10u; } while (v[k]);
		while (nd && n < 8)
			gVoiceID[n++] = (VOICE_ID_t)d[--nd];
	}
	gVoiceWriteIndex = n;
	AUDIO_PlaySingleVoice(false);
	voiceBusy = true;
}
#endif

static void Announce(const AlertRecord_t *h)
{
	BACKLIGHT_TurnOn();
#ifdef ENABLE_VOICE
	if (gAlertCfg.voice && gEeprom.VOICE_PROMPT != VOICE_PROMPT_OFF) {
		SpeakDigits(h->id, h->value);
		return;
	}
#endif
	(void)h;
}

static void PushHistory(const AlertRecord_t *r)
{
	histHead = (uint8_t)((histHead + 1u) % HISTORY_N);
	hist[histHead] = *r;
	if (histCount < HISTORY_N)
		histCount++;
}

static int8_t ClampS8(int16_t v)
{
	return (int8_t)(v < -128 ? -128 : (v > 127 ? 127 : v));
}

// De-duplicate one burst's readings, apply CONFIRM and UNKNOWN, and turn what
// is left into records: history, the ALERT, line, the voice.
static void Deliver(const AlertReading_t *r, int n, int ninv0, uint16_t burstMs)
{
	if (n <= 0)
		return;

	uint8_t frame = 0;
	uint8_t announced = 0;
	for (int i = 0; i < n; i++) {
		// collapse repeats of the same reading inside one burst
		bool dup = false;
		for (int j = 0; j < i; j++)
			if (r[j].id == r[i].id && r[j].value == r[i].value) { dup = true; break; }
		if (dup)
			continue;
		if (gAlertCfg.confirm) {
			bool again = false;
			for (int j = i + 1; j < n; j++)
				if (r[j].id == r[i].id && r[j].value == r[i].value) { again = true; break; }
			if (!again)
				continue;
		}

		char name[ALERT_NAME_MAX + 1];
		const bool known = ALERTSTN_Lookup(r[i].id, name, sizeof(name), NULL);
		if (!known) {
			stats.unknown++;
			if (!gAlertCfg.show_unknown)
				continue;
		}
		stats.frames++;
		decodesTotal++;

		AlertRecord_t rec;
		rec.epoch     = ALERT_Epoch();
		rec.uptime_ms = lostUp;
		rec.boot      = ALERTLOG_Boot();
		rec.id        = r[i].id;
		rec.value     = r[i].value;
		rec.fmt       = r[i].format;
		rec.flags     = (uint8_t)((r[i].polarity == ALERT_POL_STANDARD ? ALERTREC_POL_STD : 0u) |
		                          (i >= ninv0 ? ALERTREC_INV : 0u) | (known ? ALERTREC_TABLE : 0u) |
		                          ((frame < 15u ? frame : 15u) << ALERTREC_FRAME_SHIFT));
		rec.rssi      = ClampS8(burstPeak);
		rec.nf        = ALERT_NoiseFloor();
		rec.burst_ms  = burstMs;
		rec.payload   = r[i].payload;
		frame++;
		PushHistory(&rec);

#ifdef ENABLE_UART
		{
			// both ports, as it always went: the UART's one line
			snprintf(lb, sizeof(lb), "ALERT,%u,%u,%s,%d,%s\r\n", r[i].id, r[i].value,
			         r[i].format == ALERT_FMT_EIF ? "EIF" : "ABF", burstPeak, name);
			UART_Send(lb, strlen(lb));
#ifdef ENABLE_USB
			VCP_SendStr(lb);
#endif
		}
#endif
		if (!announced++)
			Announce(&hist[histHead]);
	}
	redraw = true;
}

// The X line: one decoded frame, before de-duplication and the table rules.
static void EmitX(const AlertReading_t *r, bool inv)
{
	sprintf(lb, "X ADC %u id=%u v=%u fmt=%s pol=%c inv=%u pos=%u known=%u\r\n", burstCount,
	        r->id, r->value, r->format == ALERT_FMT_EIF ? "EIF" : "ABF",
	        r->polarity == ALERT_POL_STANDARD ? 'S' : 'N', inv ? 1u : 0u, r->bit_pos,
	        ALERTSTN_Lookup(r->id, NULL, 0, NULL) ? 1u : 0u);
	DbgSend(lb);
}

// ---------------------------------------------------------------------------
// burst end

static void BurstFinish(void)
{
	AlertReading_t r[12];
	int      na = 0, na0 = 0;
	uint16_t adcN = 0;
	const bool     adcRan = adcRunning;
	const uint32_t dur    = lostMs - openMs;

	if (adcRunning) {
		adcN = ALERTADC_BurstStop(adcBits, (uint16_t)(sizeof(adcBits) * 8u));
		adcRunning = false;
		AudioPath();             // SPEAKER may have changed during the burst; it wins
	}
	redraw = true;

	// Qualifying: 100-1500 ms of open squelch, and with SQ GATE on a peak well
	// clear of the floor - the one thing that separates a burst from a flicker.
	if (entry != ENTRY_RUN || burstTainted || dur < 100u || dur > BURST_MAX_MS ||
	    (gAlertCfg.gate && burstPeak < (int16_t)(rssiFloor + BURST_MARGIN_DB))) {
		stats.gated++;
		return;
	}
	burstCount++;
	lastPeak = burstPeak;
	lastBits = adcN;

	// Decode, both data senses; the polarity argument covers both framings.
	// Gated: the demodulator starts at the squelch open, well inside the
	// preamble, so a real frame always has its 12 idle bits.
	if (adcN >= 40u) {
		na  = ALERT_ScanBitsGated(adcBits, adcN, gAlertCfg.polarity, 20, false, 12u, r, 12);
		na0 = na;
		if (na < 12)
			na += ALERT_ScanBitsGated(adcBits, adcN, gAlertCfg.polarity, 20, true, 12u, r + na, 12 - na);
	}
	stats.inv += (uint16_t)(na - na0);

	for (int i = 0; i < na; i++)
		EmitX(&r[i], i >= na0);

	if (adcRan && adcN) {
		// The ADC's bits, one per ALERT bit, in the old A-line shape: that is
		// what the A line always carried, 300 baud and one sample per bit.
		const uint16_t nb = (uint16_t)((adcN + 7u) / 8u);
		char *d = lb + sprintf(lb, "A %u 1 %d ", adcN, burstPeak);
		d = PutHex(d, adcBits, nb < 40u ? nb : 40u);
		strcpy(d, "\r\n");
		DbgSend(lb);
	}
	if (adcRan && adcState == ADCST_CONFIRM) {
		// Burst confirmation: both tones 10 dB over the census's idle level on
		// two of the first three qualifying bursts. The energies themselves went
		// out in the P src=BURST lines ALERTADC_BurstStop sent.
		int16_t g13 = ALERTADC_LEVEL_NONE, g21 = ALERTADC_LEVEL_NONE, idle = ALERTADC_LEVEL_NONE;
		ALERTADC_LastBurstEnergy(&g13, &g21, &idle);
		const bool ok = g13 != ALERTADC_LEVEL_NONE && g21 != ALERTADC_LEVEL_NONE &&
		                idle != ALERTADC_LEVEL_NONE && (g13 - idle) >= 100 && (g21 - idle) >= 100;
		if (ok) confOk++;
		// A decode settles it outright. The energy test alone rejected PA4 on
		// air after it had decoded three bursts out of three: the FM
		// discriminator's idle hiss fills the same band, so the tones sat only
		// ~6 dB over it. Check bits passing is far stronger evidence than that.
		if (na)
			adcState = ADCST_ON;
		else if (++confN >= 3u)
			adcState = (confOk >= 2u) ? ADCST_ON : ADCST_REJECT;
		if (adcState == ADCST_ON) {
			// remembered, so the next entry goes straight to receiving
			gAlertCfg.adc_pin = ALERTADC_ChoicePin();
			gAlertCfg.adc_pa8 = ALERTADC_ChoicePa8();
			gAlertCfg.adc_ok  = true;
			ALERT_SettingsChanged();
		}
	}

	Deliver(r, na, na0, (uint16_t)dur);
}

// ---------------------------------------------------------------------------
// squelch edges

static void BurstOpen(void)
{
	openMs       = nowMs;
	burstPeak    = rssiDbm;
	burstTainted = false;
	stats.sqcap++;
	if (adcState == ADCST_CONFIRM || adcState == ADCST_ON) {
		// TIM3 runs only while the squelch is open: the sampler's own RF
		// interference is what made fagci drop the same pipeline
		ALERTADC_BurstStart();
		adcRunning = true;
	}
}

// Nothing that blocks - a redraw, a USB line, a flash write - may run while a
// burst is being sampled. Once the squelch has been open well past the
// longest burst there is nothing left to protect (a voice conversation on the
// channel would otherwise freeze the screen and the heartbeat for its length).
static bool Quiet(void)
{
	if (draining)
		return false;
	return !sqOpen || (uint32_t)(nowMs - openMs) > BURST_MAX_MS + 500u;
}

static void RxPoll(void)
{
	// squelch state straight from the chip: REG_0C<1> 1 = open
	sqOpen = (BK4819_ReadRegister(BK4819_REG_0C) & 2u) != 0;
	if (sqOpen && !sqPrev) {
		if (draining) {
			draining = false;        // a flicker inside one burst, not a new one
		} else {
			BurstOpen();
			if (blocked)
				burstTainted = true; // opened while the loop was stuck elsewhere
		}
	} else if (!sqOpen && sqPrev) {
		draining = true;
		lostMs   = nowMs;
		lostUp   = ALERT_UptimeMs();
	}
	sqPrev  = sqOpen;
	blocked = false;                 // anything blocking from here on marks it again
	if (sqOpen && rssiDbm > burstPeak)
		burstPeak = rssiDbm;

	// Bounded. An unbounded "while (REG_0C & 1)" once froze the whole radio when
	// writing REG_02 did not clear the pending flag.
	for (uint8_t guard = 0; guard < 16 && (BK4819_ReadRegister(BK4819_REG_0C) & 1u); guard++) {
		BK4819_WriteRegister(BK4819_REG_02, 0);
		dbgIrqCount++;
	}

	if (draining && (uint32_t)(nowMs - lostMs) >= DRAIN_MS) {
		draining = false;
		BurstFinish();
	}
}

// ---------------------------------------------------------------------------
// the census

static void RunCensus(void)
{
	bool ok;
	censusBusy = true;
	ALERTUI_Draw();                  // it blocks for seconds; say why the screen froze
	DFU_WatchdogKick();
	ok = ALERTADC_Census(DbgSend);
	DFU_WatchdogKick();
	censusBusy = false;
	censusReq  = false;              // after, so the menu row reads PENDING throughout
	if (ok) {
		// The same pin again keeps its confirmation; a different one earns its own.
		const bool same = gAlertCfg.adc_ok && gAlertCfg.adc_pin == ALERTADC_ChoicePin() &&
		                  gAlertCfg.adc_pa8 == ALERTADC_ChoicePa8();
		gAlertCfg.adc_pin = ALERTADC_ChoicePin();
		gAlertCfg.adc_pa8 = ALERTADC_ChoicePa8();
		gAlertCfg.adc_ok  = same;
		adcState = same ? ADCST_ON : ADCST_CONFIRM;
		if (!same)
			ALERT_SettingsChanged();
	} else if (gAlertCfg.adc_ok && gAlertCfg.adc_pin) {
		// Confirmed before, missed now - the census depends on the volume knob
		// among other things. Trust the bursts that confirmed it.
		ALERTADC_SetChoice(gAlertCfg.adc_pin, gAlertCfg.adc_pa8);
		adcState = ADCST_ON;
	} else {
		adcState = ADCST_OFF;
	}
	confN = confOk = 0;
	RxArm();                         // the census reprogrammed the BK4819
	redraw = true;
}

// ---------------------------------------------------------------------------
// exports for the other modules (alert_int.h)

int8_t ALERT_NoiseFloor(void)
{
	// the slow minimum for now; V2 section 4's average replaces it
	return ClampS8(rssiFloor);
}

int8_t   ALERT_Rssi(void)        { return ClampS8(rssiDbm); }
bool     ALERT_SquelchOpen(void) { return sqOpen; }
uint32_t ALERT_UptimeMs(void)    { return SCHEDULER_Ticks10ms() * 10u; }
uint16_t ALERT_Bursts(void)      { return burstCount; }
uint32_t ALERT_Decodes(void)     { return decodesTotal; }
int16_t  ALERT_LastPeak(void)    { return lastPeak; }
void     ALERT_RequestRedraw(void) { redraw = true; }
uint8_t  ALERT_AdcState(void)    { return adcState; }
bool     ALERT_CensusPending(void) { return censusReq || censusBusy; }

uint32_t ALERT_Epoch(void)
{
	if (!epochBase)
		return 0;
	return epochBase + (SCHEDULER_Ticks10ms() - epochTicks) / 100u;
}

void ALERT_SetEpoch(uint32_t epoch)
{
	epochTicks = SCHEDULER_Ticks10ms();
	epochBase  = epoch;
}

uint8_t ALERT_HistoryCount(void)
{
	return histCount;
}

const AlertRecord_t *ALERT_History(uint8_t back)
{
	if (back >= histCount)
		return NULL;
	return &hist[(uint8_t)((histHead + HISTORY_N - back) % HISTORY_N)];
}

uint8_t ALERT_EntryState(uint8_t *secsLeft)
{
	if (censusBusy)
		return ENTRY_CENSUS;
	if (secsLeft) {
		const uint32_t el = nowMs - entryMs;
		*secsLeft = (uint8_t)(el >= settleMs ? 0u : (settleMs - el + 999u) / 1000u);
	}
	return entry;
}

const char *ALERT_AudName(void)
{
	static const char *const names[4] = { "NONE", "PA4", "PA4B", "PB1" };
	return names[ALERTADC_ChoicePin() & 3u];
}

// ---------------------------------------------------------------------------
// settings rows

enum {
	SET_POLARITY, SET_VOICE, SET_GATE, SET_UNKNOWN, SET_CONFIRM, SET_MONITOR,
	SET_MODE, SET_CENSUS, SET_FREQ, SET_SQL, SET_N
};

// "MDM MODE" and "SQ GATE" are grepped for by CI as proof the app is in the
// image, so they keep their names whatever the rows now do.
static const char *const setNames[SET_N] = {
	"POLARITY", "VOICE", "SQ GATE", "UNKNOWN", "CONFIRM", "MONITOR",
	"MDM MODE", "CENSUS",
	// in enum order: these two were once swapped, so FREQ stepped the squelch
	"FREQ MHz", "SQL LEVEL"
};

uint8_t ALERT_SetCount(void)
{
	return SET_N;
}

const char *ALERT_SetName(uint8_t row)
{
	return row < SET_N ? setNames[row] : "";
}

// values are at most eight characters, see the settings view
void ALERT_SetValue(uint8_t row, char *s)
{
	static const char *const onoff[2] = { "OFF", "ON" };
	static const char *const pol[3]   = { "NEG", "STD", "ANY" };
	switch (row) {
		case SET_POLARITY: strcpy(s, pol[gAlertCfg.polarity]); break;
		case SET_VOICE:    strcpy(s, onoff[gAlertCfg.voice]); break;
		case SET_GATE:     strcpy(s, onoff[gAlertCfg.gate]); break;
		case SET_UNKNOWN:  strcpy(s, gAlertCfg.show_unknown ? "SHOW" : "HIDE"); break;
		case SET_CONFIRM:  strcpy(s, gAlertCfg.confirm ? "2 COPIES" : "OFF"); break;
		case SET_MONITOR:  strcpy(s, onoff[gAlertCfg.speaker == ALERT_SPK_ON]); break;
		case SET_MODE:     strcpy(s, "ADC"); break;    // the only demodulator left
		case SET_CENSUS:
			if (ALERT_CensusPending())           strcpy(s, "PENDING");
			else if (adcState == ADCST_UNTESTED) strcpy(s, "NOT RUN");
			else sprintf(s, "%s%s", ALERT_AudName(), ALERTADC_ChoicePa8() ? "+" : "");
			break;
		case SET_SQL:      sprintf(s, "%u.%u", gEeprom.SQUELCH_LEVEL, gEeprom.SQUELCH_TENTHS); break;
		case SET_FREQ: {
			const uint32_t f = gRxVfo->pRX->Frequency;
			sprintf(s, "%u.%03u", (unsigned)(f / 100000u), (unsigned)((f % 100000u) / 100u));
			break;
		}
		default: s[0] = 0; break;
	}
}

static void ApplySquelch(void)
{
	RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
	BK4819_SetupSquelch(
		gRxVfo->SquelchOpenRSSIThresh,    gRxVfo->SquelchCloseRSSIThresh,
		gRxVfo->SquelchOpenNoiseThresh,   gRxVfo->SquelchCloseNoiseThresh,
		gRxVfo->SquelchCloseGlitchThresh, gRxVfo->SquelchOpenGlitchThresh);
	// BK4819_SetupSquelch zeroes REG_70 and mutes AF
	RxArm();
}

static void StepSquelch(int dir)
{
	int lvl = gEeprom.SQUELCH_LEVEL * 10 + gEeprom.SQUELCH_TENTHS + dir;
	if (lvl < 0) lvl = 0;
	if (lvl > 90) lvl = 90;
	gEeprom.SQUELCH_LEVEL  = (uint8_t)(lvl / 10);
	gEeprom.SQUELCH_TENTHS = (uint8_t)(lvl % 10);
	ApplySquelch();
}

void ALERT_SetStep(uint8_t row, int dir)
{
	switch (row) {
		case SET_POLARITY: gAlertCfg.polarity = (uint8_t)((gAlertCfg.polarity + 3 + dir) % 3); break;
		case SET_VOICE:    gAlertCfg.voice = !gAlertCfg.voice; break;
		case SET_GATE:     gAlertCfg.gate = !gAlertCfg.gate; break;
		case SET_UNKNOWN:  gAlertCfg.show_unknown = !gAlertCfg.show_unknown; break;
		case SET_CONFIRM:  gAlertCfg.confirm = !gAlertCfg.confirm; break;
		case SET_MONITOR:
			// ON and not-ON (SQL, the default, which is off until Phase B wires
			// it) are the two states this row has always had. Only the audio
			// path moves: the AF stays at FM for the ADC.
			gAlertCfg.speaker = (gAlertCfg.speaker == ALERT_SPK_ON) ? ALERT_SPK_SQL : ALERT_SPK_ON;
			AudioPath();
			break;
		case SET_CENSUS:   censusReq = true; break;
		case SET_FREQ: {
			// Frequency is in units of 10 Hz. Step 12.5 kHz, the ALERT channel
			// spacing, and keep it inside the 2 m / VHF range the receiver can
			// actually tune.
			int32_t f = (int32_t)gRxVfo->pRX->Frequency + dir * 1250;
			if (f < 13000000) f = 13000000;      // 130 MHz
			if (f > 17400000) f = 17400000;      // 174 MHz
			gRxVfo->pRX->Frequency = (uint32_t)f;
			gRequestSaveVFO = true;              // so it survives leaving the app
			RADIO_SetupRegisters(true);
			RxArm();
			break;
		}
		case SET_SQL:      StepSquelch(dir); break;
		default: break;                          // SET_MODE is read-only
	}
	redraw = true;
}

// ---------------------------------------------------------------------------
// keys

// The keys that act on the receiver, from the main view. Everything else -
// view changes, the settings view's own keys - is the UI's.
static void OnKey(KEY_Code_t key)
{
	BACKLIGHT_TurnOn();
	redraw = true;

	if (ALERTUI_View() == ALERT_VIEW_MAIN) {
		switch (key) {
			case KEY_EXIT: running = false; return;
			case KEY_STAR: gAlertCfg.voice = !gAlertCfg.voice; StoreConfig(); return;
			case KEY_F:    ALERT_SetStep(SET_MONITOR, 0); return;
			case KEY_UP:   StepSquelch(+1); return;
			case KEY_DOWN: StepSquelch(-1); return;
			case KEY_0:
				histCount = 0;
				memset(&stats, 0, sizeof(stats));
				return;
			case KEY_5:
				if (histCount) Announce(&hist[histHead]);
				return;
			default: break;
		}
	}
	ALERTUI_Key(key, false);
}

// ---------------------------------------------------------------------------
// main loop

void APP_RunAlert(void)
{
	KEY_Code_t lastKey     = KEY_INVALID;
	uint16_t   keyHeld10ms = 0;
	uint16_t   pttHeld10ms = 0;

	LoadConfig();

	// These are statics, so without this they carry over from the last run.
	running = true;
	redraw  = true;
	persistReq = false;
	memset(&stats, 0, sizeof(stats));
	histCount = 0;
	dbgIrqCount = 0;
	// burstCount is not reset: with the boot line it keys every X and A line,
	// and a second visit to the app in one power-up must not reuse numbers.
	sqOpen = false; sqPrev = false; draining = false;
	rssiFloor = 0; burstPeak = -127; lastPeak = -127; lastBits = 0;
	nowMs = 0; tick = 0;
	entry = ENTRY_SETTLE; entryMs = 0; censusBusy = false; censusReq = false;
	adcState = ADCST_UNTESTED; adcRunning = false; confN = 0; confOk = 0;
	blocked = false; burstTainted = false;
	ALERTUI_Reset();
	ALERTSTN_Init();
	(void)ALERTLOG_Init();

	DFU_WatchdogArm(true);

	// The app listens on whatever the VFO is tuned to. If the radio is parked
	// somewhere it could never hear an ALERT burst - 400 MHz was what turned up
	// on hardware - snap to the network frequency instead of silently listening
	// to nothing. Anything already in band is left alone.
	if (gRxVfo->pRX->Frequency < 13000000u || gRxVfo->pRX->Frequency > 17400000u) {
		gRxVfo->pRX->Frequency = ALERT_DEFAULT_FREQ;
		gRequestSaveVFO = true;
	}
	RADIO_SetupRegisters(true);
	FUNCTION_Select(FUNCTION_FOREGROUND);
	AUDIO_AudioPathOff();
	rssiDbm = BK4819_GetRSSI_dBm();

	// Receiving from the start so the floor settles; bursts during the settle
	// are seen but not decoded.
	RxArm();

	// A census choice that has decoded before is used as it stands: the census
	// takes seconds, and its verdict depends on the volume knob among other
	// things. Setting CENSUS runs it again.
	settleMs = SETTLE_MS;
	if (gAlertCfg.adc_ok && gAlertCfg.adc_pin) {
		ALERTADC_SetChoice(gAlertCfg.adc_pin, gAlertCfg.adc_pa8);
		adcState = ADCST_ON;
		settleMs = SETTLE_KNOWN_MS;
	}

	while (running) {
		DFU_WatchdogKick();

		// keys (edge triggered)
		const KEY_Code_t key = KEYBOARD_GetKey();
#if defined(ENABLE_UART) || defined(ENABLE_USB)
		// Spectrum does this every pass. Without it the radio stops answering
		// on USB for as long as this app is open.
		UART_ServiceCommands();
#endif
		ALERTCON_Poll();
		dbgRawKey = (int16_t)key;
		dbgRawPtt = GPIO_IsPttPressed() ? 1u : 0u;
		if (key != lastKey) {
			if (key != KEY_INVALID && key != KEY_PTT)
				OnKey(key);
			lastKey     = key;
			keyHeld10ms = 0;   // a new key starts its own hold timer
		}

		RxPoll();

		// Anything that blocks - the census, a flash write - waits for the
		// squelch to shut, so a burst on the air is never left unsampled.
		if (entry == ENTRY_SETTLE && (uint32_t)(nowMs - entryMs) >= settleMs) {
			entry  = (adcState == ADCST_ON) ? ENTRY_RUN : ENTRY_CENSUS;
			redraw = true;
		}
		if (Quiet()) {
			if (persistReq) {
				persistReq = false;
				Persist();
			} else if (entry == ENTRY_CENSUS) {
				RunCensus();
				entry = ENTRY_RUN;
			} else if (entry == ENTRY_RUN && censusReq) {
				RunCensus();
			}
		}

#ifdef ENABLE_VOICE
		if (voiceBusy) {
			if (gFlagPlayQueuedVoice) {
				gFlagPlayQueuedVoice = false;
				AUDIO_PlayQueuedVoice();
			}
			if (gVoiceReadIndex == 0 && gVoiceWriteIndex == 0) {
				// finished: voice playback muted the AF and may have touched the AF path
				voiceBusy = false;
				BK4819_SetAF(BK4819_AF_FM);   // the ADC needs it back regardless
				AudioPath();
			}
		}
#endif

		// housekeeping every 10 ms: clock, RSSI, hold timers; screen and USB
		// only while the squelch is shut
		if (gNextTimeslice) {
			const bool quiet = Quiet();
			gNextTimeslice = false;
			nowMs += 10u;
			BACKLIGHT_Update();
			ALERTUI_Tick10ms();

			// Hold-to-leave timers, in real 10 ms ticks. Counting loop passes
			// made "800 ms" of PTT about 160 ms, and ordinary presses dropped the
			// user out of the app.
			if (dbgRawPtt) {
				if (++pttHeld10ms > 50)     // 500 ms
					running = false;
			} else {
				pttHeld10ms = 0;
			}
			if (dbgRawKey != (int16_t)KEY_INVALID) {
				if (++keyHeld10ms > 250)    // 2.5 s
					running = false;
			} else {
				keyHeld10ms = 0;
			}

			// Every 10 ms, not every 100: a burst can be shorter than the old
			// sampling interval.
			rssiDbm = BK4819_GetRSSI_dBm();
			if (rssiFloor == 0 || rssiDbm < rssiFloor)
				rssiFloor = rssiDbm;
			if ((++tick % 500) == 0 && rssiFloor < 0)
				rssiFloor++;          // let the floor drift back up over 5 s steps

			if ((tick % 10) == 0 && quiet)
				ALERTUI_DrawStatus();
			if ((tick % 50) == 0 && quiet) {
				// The ST7565 loses its register state when the BK4819 changes RF
				// state; this fork re-sends the init list after TX and after
				// sleep-wake for exactly that reason.
				ST7565_FixInterfGlitch();
				redraw = true;
				sprintf(lb, "D I%u F%u G%u B%u R%d Q%u N%u f%d p%d V%u C%u\r\n",
				        dbgIrqCount, stats.frames, stats.gated, lastBits, rssiDbm,
				        sqOpen ? 1u : 0u, burstCount, rssiFloor, lastPeak, stats.inv, stats.sqcap);
				DbgSend(lb);
			}
		}

		// Never during a burst: a full-screen blit blocks the loop, and the
		// squelch edges are read here.
		if (redraw && Quiet()) {
			ALERTUI_Draw();
			redraw = false;
		}
	}

	// leave: ADC off, radio back to normal
	if (adcRunning) {
		(void)ALERTADC_BurstStop(adcBits, 0);
		adcRunning = false;
	}
	ALERTADC_Shutdown();
	BK4819_WriteRegister(BK4819_REG_3F, 0);
	BK4819_WriteRegister(BK4819_REG_02, 0);
	Persist();
	RADIO_SetupRegisters(true);
	ST7565_FixInterfGlitch();   // leave the controller in a state the main UI can draw on
	gRequestDisplayScreen = DISPLAY_MAIN;
	gUpdateStatus  = true;
	gUpdateDisplay = true;
	DFU_WatchdogArm(false);
}

#endif // ENABLE_ALERT
