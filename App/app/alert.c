/* ALERT telemetry receiver app - see alert.h for the overview, alert_int.h
 * for how the modules divide the work.
 *
 * This file owns the radio: entry and the main loop, the keys that act on the
 * receiver, squelch edges -> ADC burst -> decode -> record, the noise floor,
 * the speaker, the configuration and the CSV lines that go out over USB and
 * the UART (tools/alert/V2_SPEC.md sections 4, 5 and 6).
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
#include "helper/battery.h"
#include "misc.h"
#include "py32f0xx.h"
#include "radio.h"
#include "scheduler.h"
#include "settings.h"
#include "ui/ui.h"
#ifdef ENABLE_UART
	#include "py32f071_ll_usart.h"
#endif
#ifdef ENABLE_USB
	#include "driver/vcp.h"
#endif

// App/CMakeLists.txt defines it for every App source; dfu.c falls back the same way
#ifndef BUILD_COMMIT
	#define BUILD_COMMIT "unknown"
#endif

// ---------------------------------------------------------------------------
// configuration (persisted in gEeprom.ALERT_CFG[5], see settings.c)
//
//   b[0]  <0> CSV OUT, <2:1> polarity (always ANY now), <3> voice, <4> SQ GATE,
//         <5> show unknown, <6> LOG, <7> DEBUG
//   b[1]  <1:0> SPEAKER (ALERT_SPK_*), <7:3> SNR REQ in dB
//   b[2]  <1:0> 2 = the census choice below has decoded, <3:2> ADC pin
//         (ALERTADC_SetChoice numbering), <4> PA8 needed by that pin
//   b[3]  free
//   b[4]  <7:4> CFG_MAGIC, <3:1> layout version, <0> confirm
//
// POLARITY is no longer a setting (V2_SPEC 5 does not list it): the decoder
// always tries both framings. A NEG or STD saved by an earlier build is not
// read back, since with the row gone nothing could ever put it right.
//
// Only a decode sets b[2]<1:0> (BurstFinish), and a census that picks a
// different pin leaves it and the pin alone until that pin decodes, so what
// is in flash is always a pin that has decoded on this radio.
//
// Layout 0 is the X1 sweep build's: b[0]<6> bit reverse, <7> monitor, b[1]
// and b[3] the adopted arrangement and variant, and b[2]<1:0> the adopted
// route, where 2 was the ADC. That route was earned by decodes, but not the
// pin bits beside it: X1 re-ran the census on every entry and saved whatever
// it picked last. So an X1 radio's pin is not taken as confirmed, and its
// first entry after the update runs the census, as every X1 entry did.

#define CFG_MAGIC   0xC0u
#define CFG_VERSION 1u
#define CFG_ADC_OK  2u     // b[2]<1:0>

AlertCfg_t gAlertCfg;
// The console reads and changes settings from app.c's slice too, possibly
// before the app has ever been entered: until then gAlertCfg is all zeros.
static bool cfgLoaded;

static void DefaultConfig(void)
{
	// ANY: nothing has established the sense on this radio, and it is the
	// table-station and repeat rules that keep chance hits out, not a guess at
	// the polarity. The rest are V2_SPEC section 5's defaults.
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
	cfgLoaded = true;
	if ((b[4] & 0xF0u) != CFG_MAGIC)
		return;

	const uint8_t ver = (b[4] >> 1) & 7u;
	gAlertCfg.voice        = b[0] & (1u << 3);
	gAlertCfg.gate         = b[0] & (1u << 4);
	gAlertCfg.show_unknown = b[0] & (1u << 5);
	gAlertCfg.confirm      = b[4] & 1u;
	gAlertCfg.adc_ok       = (b[2] & 3u) == CFG_ADC_OK;
	gAlertCfg.adc_pin      = (b[2] >> 2) & 3u;
	gAlertCfg.adc_pa8      = b[2] & (1u << 4);
	if (ver == 0) {
		// X1 layout: the new settings take their defaults, and MONITOR ON
		// carries over as SPEAKER ON. The pin was not necessarily the one
		// that decoded (see the layout notes above).
		if (b[0] & (1u << 7))
			gAlertCfg.speaker = ALERT_SPK_ON;
		gAlertCfg.adc_ok = false;
	} else {
		gAlertCfg.csv     = b[0] & 1u;
		gAlertCfg.log     = b[0] & (1u << 6);
		gAlertCfg.debug   = b[0] & (1u << 7);
		gAlertCfg.speaker = b[1] & 3u;
		gAlertCfg.snr_req = b[1] >> 3;
	}
	if (gAlertCfg.speaker > ALERT_SPK_ON)    gAlertCfg.speaker  = ALERT_SPK_SQL;
	if (gAlertCfg.snr_req < ALERT_SNR_MIN || gAlertCfg.snr_req > ALERT_SNR_MAX)
		gAlertCfg.snr_req = ALERT_SNR_DEFAULT;
	if (!gAlertCfg.adc_pin)
		gAlertCfg.adc_ok = false;   // nothing to skip the census for
}

static void CfgReady(void)
{
	if (!cfgLoaded)
		LoadConfig();
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
// noise. This floor - a slow minimum - is the one SQ GATE has been tuned and
// proven on air against, so it stays the gate's; the averaged noise floor
// below is the reported one (V2_SPEC 4).
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

// Noise floor (V2_SPEC 4): an exponential average of the RSSI taken every
// 10 ms while the squelch is shut, alpha 1/3000 - a ~30 s time constant -
// seeded with the mean of the first second. Q16 rather than the spec's
// suggested Q8: at alpha 1/3000 a Q8 step is (diff * 256) / 3000, zero for
// any difference under 12 dB, so a Q8 average would never move at all for
// the few-dB changes it is there to follow. Q16 leaves a 0.05 dB dead band.
#define NF_ALPHA_DIV 3000
#define NF_SEED_N    100         // signed: it divides a negative sum
#define NF_HOLD_MS   500u        // after each squelch close: the tail is not noise
static int32_t  nfQ16;           // dBm * 65536, valid once seeded
static int32_t  nfSeedSum;
static uint8_t  nfSeedN;

// The weakest peak RSSI that has decoded a table station this boot (V2_SPEC
// 4). Table stations only: an unknown address may be a chance hit on noise,
// and one of those would set a sensitivity nothing can actually reach.
#define MINOK_NONE 127
static int16_t  minOk = MINOK_NONE;

// SPEAKER=SQL: the squelch has opened for a transmission not yet finished
// (BurstOpen -> BurstFinish), so the speaker path is wanted on.
static bool     txOpen;

// Records and lines
static uint32_t bootSeq;         // DEC seq when not logging: records since boot
static uint8_t  csvLines;        // CSV lines since the last HDR block
static uint32_t staMs;           // nowMs of the last STA line
static uint16_t setDirty;        // settings rows changed, EVT SET owed (1 << row)
static bool     logErr;          // an append failed this entry: said once
static bool     bootSaid;        // EVT BOOT: once per power-up

// ---------------------------------------------------------------------------
// USB and UART lines

// Debug telemetry over USB. UART_ServiceCommands() runs on every pass of the
// main loop below, and cdc_acm_data_send_with_dtr() is a no-op unless a host
// has the port open with DTR asserted, so this costs nothing when nobody is
// listening. When someone is, it blocks until the bytes are gone (~1 ms a
// line), which is why nothing but a key press ever sends while a burst is
// being sampled.
static void DbgSend(const char *s)
{
#ifdef ENABLE_USB
	VCP_SendStr(s);
#else
	(void)s;
#endif
}

// The DEBUG lines: D heartbeat, A bits, X frames, and the census's P/PT/AUD.
static void DbgLine(const char *s)
{
	if (gAlertCfg.debug)
		DbgSend(s);
}

// USB only. The console answers through this, and an answer belongs on the
// port the question came in on; the CSV lines use Out2 below.
void ALERT_Emit(const char *line)
{
	DbgSend(line);
}

#ifdef ENABLE_UART
// The UART copy of the CSV lines, queued and drained a byte or two per loop
// pass. At 38400 a 190-character DEC line holds the wire for 50 ms, and a
// blocking send after a burst would still be going when a repeater's copy of
// it starts. A line is queued whole or not at all, so the UART never carries
// half a line; USB has every line regardless.
static uint8_t uq[256];
static uint8_t uqHead, uqTail;   // uint8_t: the ring wraps by itself

static void UartDrain(void)
{
	while (uqTail != uqHead && LL_USART_IsActiveFlag_TXE(USART1))
		LL_USART_TransmitData8(USART1, uq[uqTail++]);
}

// Wait for room for n more bytes - but never while the squelch is open (the
// burst matters more than the UART's copy of a line) and never for more than
// ~100 ms (a UART that has stopped).
static bool UartRoom(uint16_t n)
{
	const uint32_t t0 = SCHEDULER_Ticks10ms();
	while ((uint16_t)(uint8_t)(uqHead - uqTail) + n > 255u) {
		if ((BK4819_ReadRegister(BK4819_REG_0C) & 2u) || SCHEDULER_Ticks10ms() - t0 > 10u)
			return false;
		UartDrain();
	}
	return true;
}

static void UartLine(const char *s)
{
	const uint16_t n = (uint16_t)strlen(s);
	// Outside the app nothing drains the queue, and the UART is the binary
	// protocol's then.
	if (!running || n > 255u || !UartRoom(n))
		return;
	while (*s)
		uq[uqHead++] = (uint8_t)*s++;
}
#endif

// A line on both ports (V2_SPEC 6).
static void Out2(const char *s)
{
	DbgSend(s);
#ifdef ENABLE_UART
	UartLine(s);
#endif
}

static void CsvLine(const char *s)
{
	if (!gAlertCfg.csv)
		return;
	Out2(s);
	if (csvLines < 255u)
		csvLines++;
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

// A number that may be unknown: nothing at all when it is 0 (V2_SPEC 6: an
// empty field is unknown). The epoch and the boot counter are 0 when unset.
static char *PutOpt(char *d, uint32_t v)
{
	*d = 0;
	if (v)
		d += sprintf(d, "%lu", (unsigned long)v);
	return d;
}

static const char *const hdrLines[] = {
	"HDR,DEC,seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,frame,"
		"rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin\r\n",
	"HDR,BST,seq,epoch,uptime_ms,peak,nf,burst_ms,nframes,nbits,bits_hex\r\n",
	"HDR,STA,epoch,uptime_ms,nf,rssi,sq,batt_mv,batt_pct,bursts,decodes,min_ok,"
		"log_state,log_count,log_cap,stn_src\r\n",
	"HDR,EVT,epoch,uptime_ms,code,detail\r\n",
};

// Always, whatever CSV OUT says: the console's CSV HDR asks for it by name.
void ALERT_CsvHeader(void)
{
	sprintf(lb, "HDR,fw,%s,schema,2\r\n", BUILD_COMMIT);
	Out2(lb);
	for (uint8_t i = 0; i < sizeof(hdrLines) / sizeof(hdrLines[0]); i++)
		Out2(hdrLines[i]);
	csvLines = 0;
}

static void EmitEvt(const char *code, const char *detail)
{
	if (!gAlertCfg.csv)
		return;
	char *d = PutOpt(lb + sprintf(lb, "EVT,"), ALERT_Epoch());
	sprintf(d, ",%lu,%s,%s\r\n", (unsigned long)ALERT_UptimeMs(), code, detail);
	CsvLine(lb);
}

static const char *const fmtNames[4] = { "", "ABF", "EIF", "A2C" };

uint8_t ALERT_FormatRecord(char *out, const char *type, uint32_t seq, const AlertRecord_t *r)
{
	char        name[ALERT_NAME_MAX + 1], eng[8];
	uint8_t     kind = ALERT_KIND_NONE;
	const char *unit = "";
	char       *d;

	CfgReady();                      // SNR REQ, when the console dumps the log outside the app
	ALERTSTN_Lookup(r->id, name, sizeof(name), &kind);
	for (d = name; *d; d++)
		if (*d == ',')
			*d = ' ';                // one field, whatever the table holds

	// eng/unit as FormatValue shows them; a full-scale value is a dead or
	// over-range sensor, i.e. no reading: left empty
	eng[0] = 0;
	if (r->value != ALERT_VALUE_FULL_SCALE) {
		if (kind == ALERT_KIND_BATT) {
			sprintf(eng, "%u.%u", r->value / 10u, r->value % 10u);
			unit = "V";
		} else {
			sprintf(eng, "%u", r->value);
			if (kind == ALERT_KIND_RAIN)
				unit = "tips";
		}
	}

	// sens = NF + SNR REQ, an estimate (V2_SPEC 4); fade = how far over it
	const int sens = r->nf + (int)gAlertCfg.snr_req;

	d = PutOpt(out + sprintf(out, "%s,%lu,", type, (unsigned long)seq), r->epoch);
	d = PutOpt(d + sprintf(d, ",%lu,", (unsigned long)r->uptime_ms), r->boot);
	d += sprintf(d, ",%u,%s,%s,%u,%s,%s,%s,%s,%u,%u,%d,%d,%d,%d,%u,%08lX,",
	             r->id, name, ALERT_KindLabel(kind), r->value, eng, unit,
	             fmtNames[r->fmt & 3u], (r->flags & ALERTREC_POL_STD) ? "STD" : "NEG",
	             (r->flags & ALERTREC_INV) ? 1u : 0u, ALERTREC_FRAME(r->flags),
	             r->rssi, r->nf, sens, r->rssi - sens, r->burst_ms, (unsigned long)r->payload);
	for (uint8_t i = 0; i < 32u; i++)
		*d++ = ((r->payload >> (31u - i)) & 1u) ? '1' : '0';
	strcpy(d, "\r\n");
	return (uint8_t)(d + 2 - out);
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
static bool     adcDelayed;      // BurstStart owed once the amplifier has settled
static uint8_t  confN, confOk;   // burst confirmation, first three qualifying bursts
static uint8_t  adcBits[64];     // one burst at 300 baud: 512 bits is 1.7 s
static uint16_t lastBits;        // bits the demodulator gave for the last burst

static const char *const adcStNames[5] = { "UNTESTED", "OFF", "CONFIRM", "ON", "REJECT" };

// The speaker path (PA8, the audio amplifier) per SPEAKER: OFF never, ON
// always, SQL while a transmission is open. Never off under a burst whose
// census choice needs the amplifier for its audio: ALERTADC_BurstStart
// switched it on for that burst and BurstStop puts it back, so the ADC owns
// it until then, whatever SPEAKER says (V2_SPEC 5, "as today").
static void AudioPath(void)
{
	const bool want = gAlertCfg.speaker == ALERT_SPK_ON ||
	                  (gAlertCfg.speaker == ALERT_SPK_SQL && txOpen) ||
	                  (adcRunning && ALERTADC_ChoicePa8());
	if (want) AUDIO_AudioPathOn(); else AUDIO_AudioPathOff();
}

// Apply a census choice with the amplifier off. The PA4B bias steps PA4 from
// wherever it floats to mid-rail, and PA4 is in the speaker path: switched
// with PA8 on, the step clicks the speaker. The census applies its own choice
// with PA8 off for the same reason (alert_adc.c).
static void SetChoiceQuiet(uint8_t pin, bool pa8)
{
	AUDIO_AudioPathOff();
	ALERTADC_SetChoice(pin, pa8);
	AudioPath();
}

static void CensusEvt(const char *state)
{
	char d[28];
	sprintf(d, "%s pa=%u %s", ALERT_AudName(), ALERTADC_ChoicePa8() ? 1u : 0u, state);
	EmitEvt("CENSUS", d);
}

// burst state
static uint32_t openMs, lostMs, lostUp;
static bool     draining;        // squelch shut, the burst not finished for DRAIN_MS
static bool     burstTainted;    // the receiver was re-armed under it
// The loop was just blocked - an arm (~30 ms), the census (~3 s), a settings
// save, or a stall past STALL_TICKS. A squelch edge first seen right after one
// may be long past: the opening was never sampled, so that burst is tainted
// rather than decoded.
static bool     blocked;

// DRAIN_MS was the FSK FIFO's: no word left stranded when the carrier drops.
// It stays because the demodulator has been running across it all along, so
// the ADC's bits and a squelch flicker's being one burst are unchanged.
#define DRAIN_MS       40u       // squelch lost -> BurstFinish
#define BURST_MAX_MS   1500u     // longer than this is not an ALERT burst
// SPEAKER=SQL switching the amplifier on at the squelch open puts its turn-on
// step and pop on PA4. When BurstStart switches it on itself it skips 20 ms of
// samples for exactly that (PA8_SETTLE_SAMPLES); here the sampling starts that
// much later instead. nowMs moves in 10 ms steps, so 30 is 20-30 ms real.
#define SPK_SETTLE_MS  30u
// A stall a burst absorbs. Its sampling starts SPK_SETTLE_MS after the edge
// anyway, the decoder needs 12 idle bits (40 ms) ahead of a frame and no
// more, and the check bits reject whatever is not a frame. A console line or
// a log record is a few ms, a sector erase usually tens: tainting the next
// burst after each of those threw away bursts that decode. Past ~100 ms the
// opening, and the burst_ms, are anyone's guess.
#define STALL_TICKS    10u       // 10 ms ticks: more than this taints

static void Stalled(uint32_t t0)
{
	if (SCHEDULER_Ticks10ms() - t0 > STALL_TICKS)
		blocked = true;
}

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
	// A flash write: in the app, at the next quiet moment. Outside it (the
	// console from app.c's slice) at once - nothing else there would save it,
	// and a setting lost at the next power-off was never really set.
	if (running)
		persistReq = true;
	else
		SETTINGS_SaveSettings();
}

// ---------------------------------------------------------------------------
// noise floor

static void NfReseed(void)
{
	nfSeedN   = 0;
	nfSeedSum = 0;
}

static void NfSample(void)
{
	if (nfSeedN < NF_SEED_N) {
		nfSeedSum += rssiDbm;
		if (++nfSeedN == NF_SEED_N)
			nfQ16 = nfSeedSum * 65536 / NF_SEED_N;
		return;
	}
	// truncating division: a symmetric dead band of 3000/65536 dB
	nfQ16 += ((int32_t)rssiDbm * 65536 - nfQ16) / NF_ALPHA_DIV;
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

// Into the log when LOG is on and the log usable, then out as a DEC line
// carrying the log's sequence number (V2_SPEC 6), else the per-boot count.
static void Record(const AlertRecord_t *rec)
{
	uint32_t seq = 0;

	bootSeq++;
	if (gAlertCfg.log) {
		// A page write is ~1 ms, but crossing into a new sector erases it
		// first, and LOG turned on since entry makes this boot's claim on the
		// log here (ALERTLOG_Use): either can outlast what a burst absorbs.
		const uint32_t t0 = SCHEDULER_Ticks10ms();
		if (ALERTLOG_Use() && (seq = ALERTLOG_NextSeq()) != 0 && !ALERTLOG_Append(rec)) {
			seq = 0;
			if (!logErr) {
				logErr = true;
				EmitEvt("LOG", "APPEND FAIL");
			}
		}
		Stalled(t0);
	}
	if (gAlertCfg.csv) {
		ALERT_FormatRecord(lb, "DEC", seq ? seq : bootSeq, rec);
		CsvLine(lb);
	}
}

// De-duplicate one burst's readings, apply CONFIRM, and turn what is left into
// records: history, log, DEC line, voice. Every frame is recorded whether its
// address is in the table or not; UNKNOWN only decides what the screen shows
// (V2_SPEC 3) and what is announced. Returns the records made.
static uint8_t Deliver(const AlertReading_t *r, int n, int ninv0, uint16_t burstMs)
{
	uint8_t frame = 0;
	bool    announced = false;

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

		const bool known = ALERTSTN_Lookup(r[i].id, NULL, 0, NULL);
		if (!known)
			stats.unknown++;
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
		Record(&rec);
		if (known && burstPeak < minOk)
			minOk = burstPeak;

		if (gAlertCfg.debug) {
			// the X1 line, both ports as it always went
			char name[ALERT_NAME_MAX + 1];
			ALERTSTN_Lookup(r[i].id, name, sizeof(name), NULL);
			sprintf(lb, "ALERT,%u,%u,%s,%d,%s\r\n", r[i].id, r[i].value,
			        fmtNames[r[i].format & 3u], burstPeak, name);
			Out2(lb);
		}
		if (!announced && (known || gAlertCfg.show_unknown)) {
			announced = true;
			Announce(&rec);
		}
	}
	if (n > 0)
		redraw = true;
	return frame;
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

// One qualifying burst, decoded or not. seq is the burst's number this boot:
// bursts are not logged, so there is no log sequence to give it, and its
// uptime_ms (the squelch close) is the DEC lines' own, which ties them to it.
static void EmitBst(uint8_t nframes, uint16_t nbits, uint16_t dur)
{
	if (!gAlertCfg.csv)
		return;
	const uint16_t nb = (uint16_t)((nbits + 7u) / 8u);
	char *d = PutOpt(lb + sprintf(lb, "BST,%u,", burstCount), ALERT_Epoch());
	d += sprintf(d, ",%lu,%d,%d,%u,%u,%u,", (unsigned long)lostUp, burstPeak, ALERT_NoiseFloor(),
	             dur, nframes, nbits);
	d = PutHex(d, adcBits, nb < 60u ? nb : 60u);
	strcpy(d, "\r\n");
	CsvLine(lb);
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
	const uint8_t  st0    = adcState;

	adcDelayed = false;              // closed before the amplifier settled: never sampled
	if (adcRunning) {
		adcN = ALERTADC_BurstStop(adcBits, (uint16_t)(sizeof(adcBits) * 8u));
		adcRunning = false;
	}
	// SPEAKER=SQL closes here rather than at the squelch edge: after
	// BurstStop, so the amplifier's switch-off step never lands in samples.
	// SPEAKER may also have changed during the burst; it wins.
	txOpen = false;
	AudioPath();
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

	if (gAlertCfg.debug) {
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
		if (na) {
			// Remembered, so the next entry goes straight to receiving. Only a
			// decode is: the energy test has been wrong on air (above), and a
			// pin saved on its word alone would be sampled on every power-up
			// from then on without ever decoding. (CONFIRM means this choice
			// is not the saved one: RunCensus goes straight to ON when it is.)
			adcState = ADCST_ON;
			gAlertCfg.adc_pin = ALERTADC_ChoicePin();
			gAlertCfg.adc_pa8 = ALERTADC_ChoicePa8();
			gAlertCfg.adc_ok  = true;
			ALERT_SettingsChanged();
		} else if (++confN >= 3u) {
			if (gAlertCfg.adc_ok && gAlertCfg.adc_pin) {
				// A re-run census picked a pin that has not decoded in three
				// bursts. The saved one has decoded on this radio, so it
				// comes back, as it does when the census finds nothing.
				SetChoiceQuiet(gAlertCfg.adc_pin, gAlertCfg.adc_pa8);
				adcState = ADCST_ON;
			} else {
				// Nothing proven to fall back to: the energy test decides,
				// for this session only.
				adcState = (confOk >= 2u) ? ADCST_ON : ADCST_REJECT;
			}
		}
	}
	if (adcState != st0)
		CensusEvt(adcStNames[adcState]);

	const uint8_t nrec = Deliver(r, na, na0, (uint16_t)dur);
	EmitBst(nrec, adcN, (uint16_t)dur);
}

// ---------------------------------------------------------------------------
// squelch edges

static void BurstOpen(void)
{
	// The header's RX (V2_SPEC 3), first: a status-line blit is ~1.4 ms, and
	// the sampling below has not started yet. The full redraw that clears it
	// waits for the squelch to shut, which no ALERT burst outlasts.
	ALERTUI_MarkRx();
	openMs       = nowMs;
	burstPeak    = rssiDbm;
	burstTainted = false;
	txOpen       = true;
	stats.sqcap++;
	if (adcState == ADCST_CONFIRM || adcState == ADCST_ON) {
		if (gAlertCfg.speaker == ALERT_SPK_SQL && !ALERTADC_ChoicePa8()) {
			// The speaker opens now, the sampling once the amplifier has
			// settled (SPK_SETTLE_MS, RxPoll). A choice that needs PA8 has it
			// switched by BurstStart, which settles it itself.
			AUDIO_AudioPathOn();
			adcDelayed = true;
			return;
		}
		// TIM3 runs only while the squelch is open: the sampler's own RF
		// interference is what made fagci drop the same pipeline
		ALERTADC_BurstStart();
		adcRunning = true;
	}
	AudioPath();
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

	if (adcDelayed && sqOpen && (uint32_t)(nowMs - openMs) >= SPK_SETTLE_MS) {
		adcDelayed = false;
		ALERTADC_BurstStart();
		adcRunning = true;
	}

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
	// The census drops any bias first and applies its choice's last, and puts
	// PA8 back as it found it: with the amplifier off here, neither step
	// reaches the speaker (see SetChoiceQuiet). RxArm below restores SPEAKER.
	AUDIO_AudioPathOff();
	DFU_WatchdogKick();
	// Its P/PT/AUD lines, and the P src=BURST lines BurstStop sends through the
	// same callback afterwards, are DEBUG lines; EVT CENSUS carries the result.
	ok = ALERTADC_Census(DbgLine);
	DFU_WatchdogKick();
	censusBusy = false;
	censusReq  = false;              // after, so the menu row reads PENDING throughout
	if (ok) {
		// The same pin again keeps its confirmation. A different one has to
		// decode before it replaces the saved choice (BurstFinish); until then
		// gAlertCfg, and flash, keep the old one to fall back to.
		const bool same = gAlertCfg.adc_ok && gAlertCfg.adc_pin == ALERTADC_ChoicePin() &&
		                  gAlertCfg.adc_pa8 == ALERTADC_ChoicePa8();
		adcState = same ? ADCST_ON : ADCST_CONFIRM;
	} else if (gAlertCfg.adc_ok && gAlertCfg.adc_pin) {
		// Confirmed before, missed now - the census depends on the volume knob
		// among other things. Trust the bursts that confirmed it. The
		// amplifier is still off.
		ALERTADC_SetChoice(gAlertCfg.adc_pin, gAlertCfg.adc_pa8);
		adcState = ADCST_ON;
	} else {
		adcState = ADCST_OFF;
	}
	confN = confOk = 0;
	RxArm();                         // the census reprogrammed the BK4819
	CensusEvt(adcStNames[adcState]);
	redraw = true;
}

// ---------------------------------------------------------------------------
// exports for the other modules (alert_int.h)

int8_t ALERT_NoiseFloor(void)
{
	int32_t q;
	if (nfSeedN >= NF_SEED_N)
		q = nfQ16;
	else if (nfSeedN)
		q = nfSeedSum * 65536 / nfSeedN;     // still seeding: the mean so far
	else
		return ClampS8(rssiDbm);             // nothing measured yet
	// to the nearest dB; >> on a negative value is arithmetic in GCC
	return ClampS8((int16_t)((q + 32768) >> 16));
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
	const uint32_t was = ALERT_Epoch();
	char d[20];

	epochTicks = SCHEDULER_Ticks10ms();
	epochBase  = epoch;
	// the step says how far the clock had drifted since it was last set
	if (was)
		sprintf(d, "STEP %ld", (long)(epoch - was));
	else
		strcpy(d, "SET");
	CfgReady();
	EmitEvt("CLOCK", d);
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
// settings rows (V2_SPEC 5, in its order)

enum {
	SET_VOICE, SET_SPEAKER, SET_CSV, SET_LOG, SET_UNKNOWN, SET_CONFIRM, SET_GATE,
	SET_SNR, SET_DEBUG, SET_MODE, SET_CENSUS, SET_FREQ, SET_SQL, SET_N
};

// "MDM MODE" and "SQ GATE" are grepped for by CI as proof the app is in the
// image, so they keep their names whatever the rows now do.
static const char *const setNames[SET_N] = {
	"VOICE", "SPEAKER", "CSV OUT", "LOG", "UNKNOWN", "CONFIRM", "SQ GATE",
	"SNR REQ", "DEBUG", "MDM MODE", "CENSUS",
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

// Values are at most eight characters (see the settings view), and a
// two-state row reads exactly OFF/ON or its own two words, so the console can
// match a SET against them.
static const char *const onOff[2]    = { "OFF", "ON" };
static const char *const spkNames[3] = { "OFF", "SQL", "ON" };   // ALERT_SPK_*

// The two-valued rows: where each keeps its value. NULL for the others.
static bool *Flag(uint8_t row)
{
	switch (row) {
		case SET_VOICE:   return &gAlertCfg.voice;
		case SET_CSV:     return &gAlertCfg.csv;
		case SET_LOG:     return &gAlertCfg.log;
		case SET_UNKNOWN: return &gAlertCfg.show_unknown;
		case SET_CONFIRM: return &gAlertCfg.confirm;
		case SET_GATE:    return &gAlertCfg.gate;
		case SET_DEBUG:   return &gAlertCfg.debug;
		default:          return NULL;
	}
}

static const char *Word(uint8_t row, bool on)
{
	if (row == SET_UNKNOWN)
		return on ? "SHOW" : "HIDE";
	if (row == SET_CONFIRM)
		return on ? "2 COPIES" : "OFF";
	return onOff[on];
}

void ALERT_SetValue(uint8_t row, char *s)
{
	const bool *f;

	CfgReady();
	if ((f = Flag(row)) != NULL) {
		strcpy(s, Word(row, *f));
		return;
	}
	switch (row) {
		case SET_SPEAKER:  strcpy(s, spkNames[gAlertCfg.speaker % 3u]); break;
		case SET_SNR:      sprintf(s, "%u", gAlertCfg.snr_req); break;
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

// EVT SET: "NAME=VALUE", the name as the console spells it (spaces -> _).
static void SetEvt(uint8_t row)
{
	char d[20], *p;
	strcpy(d, setNames[row]);
	for (p = d; *p; p++)
		if (*p == ' ')
			*p = '_';
	*p++ = '=';
	ALERT_SetValue(row, p);
	EmitEvt("SET", d);
}

// A row has changed: the screen, and its EVT SET. In the app the EVT waits
// for a quiet moment (and a held key's steps coalesce into one line with the
// final value); outside it there is no burst to protect.
static void Changed(uint8_t row)
{
	redraw = true;
	if (running)
		setDirty |= (uint16_t)(1u << row);
	else
		SetEvt(row);
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

// SQL LEVEL in tenths, 0.0-9.0
static void SetSquelch(int lvl)
{
	if (lvl < 0) lvl = 0;
	if (lvl > 90) lvl = 90;
	gEeprom.SQUELCH_LEVEL  = (uint8_t)(lvl / 10);
	gEeprom.SQUELCH_TENTHS = (uint8_t)(lvl % 10);
	ApplySquelch();
}

// FREQ, in units of 10 Hz, inside the 2 m / VHF range the receiver can
// actually tune. Saved with the VFO so it survives leaving the app; another
// channel has another floor.
#define FREQ_MIN 13000000        // 130 MHz
#define FREQ_MAX 17400000        // 174 MHz

static void Tune(int32_t f)
{
	gRxVfo->pRX->Frequency = (uint32_t)(f < FREQ_MIN ? FREQ_MIN : (f > FREQ_MAX ? FREQ_MAX : f));
	gRequestSaveVFO = true;
	RADIO_SetupRegisters(true);
	RxArm();
	NfReseed();
}

// Outside the app - the console also runs from app.c's slice, in normal radio
// operation - the BK4819 and the speaker belong to the rest of the firmware:
// RxArm there would drop the CTCSS/DCS/DTMF interrupts RADIO_SetupRegisters
// enabled and force the AF to FM, possibly in the middle of a transmission.
// So without `running` only the app's own settings change, and the rows that
// act on the receiver refuse.
bool ALERT_SetStep(uint8_t row, int dir)
{
	bool *f;

	CfgReady();
	if ((f = Flag(row)) != NULL) {
		*f = !*f;
		if (row == SET_CSV)
			csvLines = 200;              // a host that just switched it on gets the HDR block first
		Changed(row);
		return true;
	}
	switch (row) {
		case SET_SPEAKER:
			// OFF -> SQL -> ON going up. Only the audio path moves: the AF
			// stays at FM for the ADC. Not under a burst being sampled, where
			// the amplifier's switching step would land in the samples:
			// BurstFinish applies SPEAKER once the burst is over.
			gAlertCfg.speaker = (uint8_t)((gAlertCfg.speaker + 3 + dir) % 3);
			if (running && !adcRunning && !adcDelayed)
				AudioPath();
			break;
		case SET_SNR: {
			const int v = (int)gAlertCfg.snr_req + dir;
			if (v < (int)ALERT_SNR_MIN || v > (int)ALERT_SNR_MAX)
				return false;
			gAlertCfg.snr_req = (uint8_t)v;
			break;
		}
		case SET_CENSUS:
			// UP re-runs it (V2_SPEC 5); DOWN does nothing
			if (!running || dir <= 0)
				return false;
			censusReq = true;
			break;
		case SET_FREQ:
			// 12.5 kHz, the ALERT channel spacing
			if (!running)
				return false;
			Tune((int32_t)gRxVfo->pRX->Frequency + dir * 1250);
			break;
		case SET_SQL:
			if (!running)
				return false;
			SetSquelch(gEeprom.SQUELCH_LEVEL * 10 + gEeprom.SQUELCH_TENTHS + dir);
			break;
		default: return false;                   // SET_MODE is read-only
	}
	Changed(row);
	return true;
}

// A console value against a row's word: '_' is a space, either case.
static bool ValIs(const char *want, const char *v)
{
	for (;; want++, v++) {
		char c = *v;
		if (c == '_')
			c = ' ';
		else if (c >= 'a' && c <= 'z')
			c = (char)(c - 32);
		if (c != *want)
			return false;
		if (!c)
			return true;
	}
}

// A settings number, all of it: digits with at most one '.' and at most dec
// digits after it, as an integer in units of 10^-dec ("162.55" with dec 5 is
// 16255000). Anything else - a sign, a comma, a trailing letter - is false.
// Saturates far past every row's range rather than overflowing.
static bool Fixed(const char *s, uint8_t dec, int32_t *v)
{
	int32_t x = 0;
	int8_t  frac = -1;               // digits after the '.'; -1 before one
	bool    digit = false;

	for (; *s; s++) {
		if (*s == '.' && frac < 0) {
			frac = 0;
			continue;
		}
		if (*s < '0' || *s > '9' || (frac >= 0 && ++frac > (int8_t)dec))
			return false;
		digit = true;
		x = x < 100000000 ? x * 10 + (*s - '0') : 999999999;
	}
	for (frac = frac < 0 ? 0 : frac; frac < (int8_t)dec; frac++)
		x = x < 100000000 ? x * 10 : 999999999;
	*v = x;
	return digit;
}

// The console's SET used to walk ALERT_SetStep towards the value: a retune
// (30 ms in BK4819_ResetFSK) per 12.5 kHz of FREQ, the side effects of every
// value passed on the way, and all of it undone after a refusal. This goes
// straight to the value, or nowhere.
const char *ALERT_SetTo(uint8_t row, const char *v)
{
	char        cur[12];
	int32_t     x = 0;
	const bool *f;

	ALERT_SetValue(row, cur);            // CfgReady too
	if (ValIs(cur, v))
		return NULL;                     // already so: nothing to change or say
	if ((f = Flag(row)) != NULL) {
		if (!ValIs(Word(row, !*f), v))
			return "ARGS";
		ALERT_SetStep(row, +1);
	} else switch (row) {
		case SET_SPEAKER:
			while (x < 3 && !ValIs(spkNames[x], v))
				x++;
			if (x == 3)
				return "ARGS";
			// of three values, whichever it is is one step away, up or down
			ALERT_SetStep(row, x == (gAlertCfg.speaker + 1) % 3 ? +1 : -1);
			break;
		case SET_SNR:
			if (!Fixed(v, 0, &x))
				return "ARGS";
			if (x < (int32_t)ALERT_SNR_MIN || x > (int32_t)ALERT_SNR_MAX)
				return "RANGE";
			if (x == gAlertCfg.snr_req)
				return NULL;
			gAlertCfg.snr_req = (uint8_t)x;
			Changed(row);
			break;
		case SET_FREQ:
			if (!Fixed(v, 5, &x))
				return "ARGS";
			x = (x + 625) / 1250 * 1250;         // the nearest 12.5 kHz channel
			if (x < FREQ_MIN || x > FREQ_MAX)
				return "RANGE";
			if (!running)
				return "NOTINAPP";
			if ((uint32_t)x == gRxVfo->pRX->Frequency)
				return NULL;
			Tune(x);
			Changed(row);
			break;
		case SET_SQL:
			if (!Fixed(v, 1, &x))
				return "ARGS";
			if (x > 90)
				return "RANGE";
			if (!running)
				return "NOTINAPP";
			if (x == gEeprom.SQUELCH_LEVEL * 10 + gEeprom.SQUELCH_TENTHS)
				return NULL;
			SetSquelch(x);
			Changed(row);
			break;
		case SET_MODE:
			return "READONLY";
		default:
			return "ARGS";                       // CENSUS: only "+" re-runs it
	}
	ALERT_SettingsChanged();
	return NULL;
}

// ---------------------------------------------------------------------------
// keys

#ifdef ENABLE_VOICE
// KEY_0: the selected entry again, from where the list takes it (alert_ui.c's
// UseLog) - the log while logging and not empty, else the RAM ring.
// ALERTUI_Selected is 0 at the top of the list even when UNKNOWN=HIDE skips
// the newest records there, so this takes the first shown record from it on,
// as the list does, and gives up after as many as the list's own search.
static void Replay(void)
{
	AlertRecord_t  rec;
	const bool     useLog = gAlertCfg.log && !strcmp(ALERTLOG_State(), "OK") && ALERTLOG_Count();
	uint32_t       back   = ALERTUI_Selected();

	for (uint8_t n = 0;; n++, back++) {
		if (useLog) {
			if (!ALERTLOG_ReadBack(back, &rec, NULL))
				return;
		} else {
			const AlertRecord_t *h = back < 255u ? ALERT_History((uint8_t)back) : NULL;
			if (!h)
				return;
			rec = *h;
		}
		if (gAlertCfg.show_unknown || (rec.flags & ALERTREC_TABLE))
			break;
		if (n >= 127u)
			return;
	}
	Announce(&rec);
}
#endif

// V2_SPEC 3. alert.c keeps the keys that act on the receiver or the app -
// EXIT, STAR, F, 0 - on the list and the detail view; everything else (MENU,
// UP/DOWN, 1, and every key in the settings view) is the UI's. `held` is an
// auto-repeat of UP/DOWN (see the loop).
static void OnKey(KEY_Code_t key, bool held)
{
	const uint8_t view = ALERTUI_View();

	BACKLIGHT_TurnOn();
	redraw = true;

	if (!held && view != ALERT_VIEW_SETTINGS) {
		switch (key) {
			case KEY_EXIT:
				if (view != ALERT_VIEW_MAIN)
					break;                   // detail -> list is the UI's
				if (ALERTUI_Selected()) {
					// scrolled: back to the top first, and only if the UI did
					// not do that, out
					ALERTUI_Key(KEY_EXIT, false);
					if (!ALERTUI_Selected())
						return;
				}
				running = false;
				return;
			case KEY_STAR:
				ALERT_SetStep(SET_VOICE, +1);
				ALERT_SettingsChanged();
				return;
			case KEY_F:
				ALERT_SetStep(SET_SPEAKER, +1);
				ALERT_SettingsChanged();
				return;
			case KEY_0:
#ifdef ENABLE_VOICE
				Replay();
#endif
				return;
			default:
				break;
		}
	}
	ALERTUI_Key(key, held);
}

// UP/DOWN held: a first repeat after 500 ms, then every 100 ms - a log holds
// thousands of entries, and FREQ steps 12.5 kHz at a time.
#define KEY_REPEAT_START 50u     // 10 ms ticks
#define KEY_REPEAT_EVERY 10u

// ---------------------------------------------------------------------------
// periodic lines

static void EmitSta(void)
{
	char *d = PutOpt(lb + sprintf(lb, "STA,"), ALERT_Epoch());
	d += sprintf(d, ",%lu,%d,%d,%u,%u,%u,%u,%lu,", (unsigned long)ALERT_UptimeMs(),
	             ALERT_NoiseFloor(), ALERT_Rssi(), sqOpen ? 1u : 0u,
	             gBatteryVoltageAverage * 10u, BATTERY_VoltsToPercent(gBatteryVoltageAverage),
	             burstCount, (unsigned long)decodesTotal);
	if (minOk != MINOK_NONE)
		d += sprintf(d, "%d", minOk);
	sprintf(d, ",%s,%lu,%lu,%s\r\n", ALERTLOG_State(), (unsigned long)ALERTLOG_Count(),
	        (unsigned long)ALERTLOG_Capacity(), ALERTSTN_Source());
	CsvLine(lb);
}

// The modal loop bypasses app.c's battery sampling, which would leave the
// status-line icon and the STA line frozen at the entry reading (foxhunt.c
// does the same). Never while TIM3 is triggering the ADC (alert_adc.h).
static void BatteryRefresh(void)
{
	if (ALERTADC_IsSampling())
		return;
	BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryVoltageIndex++], &gBatteryCurrent);
	if (gBatteryVoltageIndex > 3)
		gBatteryVoltageIndex = 0;
	BATTERY_GetReadings(false);
}

// What goes out at app entry: the HDR block and the state of things.
static void EntryLines(void)
{
	static const char *const rst[4] = { "POR", "SW", "WD", "FAULT" };
	char d[48];

	if (gAlertCfg.csv)
		ALERT_CsvHeader();
	if (!bootSaid) {
		bootSaid = true;
		sprintf(d, "%s %u", rst[DFU_ResetReason() & 3u], ALERTLOG_Boot());
		EmitEvt("BOOT", d);
	}
	sprintf(d, "%s %u", ALERTSTN_Source(), ALERTSTN_Count());
	EmitEvt("STN", d);
	sprintf(d, "%s %lu/%lu", ALERTLOG_State(), (unsigned long)ALERTLOG_Count(),
	        (unsigned long)ALERTLOG_Capacity());
	EmitEvt("LOG", d);
	if (adcState == ADCST_ON)
		CensusEvt("SAVED");          // no census this entry: the choice that decoded before
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
	// Nor are bootSeq and minOk: both are "this boot" (V2_SPEC 4, 6).
	sqOpen = false; sqPrev = false; draining = false;
	rssiFloor = 0; burstPeak = -127; lastPeak = -127; lastBits = 0;
	nowMs = 0; tick = 0;
	entry = ENTRY_SETTLE; entryMs = 0; censusBusy = false; censusReq = false;
	adcState = ADCST_UNTESTED; adcRunning = false; adcDelayed = false; confN = 0; confOk = 0;
	blocked = false; burstTainted = false; txOpen = false;
	openMs = 0; lostMs = 0;          // lostMs = 0: the floor skips the first 500 ms too
	NfReseed();
	staMs = 0; setDirty = 0; logErr = false;
	ALERTUI_Reset();
	ALERTSTN_Init();
	// LOG OFF writes nothing to the log region, not even the boot counter
	(void)(gAlertCfg.log ? ALERTLOG_Use() : ALERTLOG_Init());

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

	// A census choice that has decoded before is used as it stands: the census
	// takes seconds, and its verdict depends on the volume knob among other
	// things. Setting CENSUS runs it again. Applied here, before RxArm opens
	// the speaker path, so the bias goes on with the amplifier off (see
	// SetChoiceQuiet), and after a Shutdown, so the ADC starts from
	// BOARD_ADC_Init's state exactly as it does after a census.
	settleMs = SETTLE_MS;
	if (gAlertCfg.adc_ok && gAlertCfg.adc_pin) {
		ALERTADC_Shutdown();
		ALERTADC_SetChoice(gAlertCfg.adc_pin, gAlertCfg.adc_pa8);
		adcState = ADCST_ON;
		settleMs = SETTLE_KNOWN_MS;
	}

	// Receiving from the start so the floor settles; bursts during the settle
	// are seen but not decoded.
	RxArm();
	// After RxArm: the UART queue checks the squelch, which needs the chip in RX.
	EntryLines();

	while (running) {
		DFU_WatchdogKick();

		// keys (edge triggered)
		const KEY_Code_t key = KEYBOARD_GetKey();
#if defined(ENABLE_UART) || defined(ENABLE_USB)
		// Spectrum does this every pass. Without it the radio stops answering
		// on USB for as long as this app is open.
		UART_ServiceCommands();
#endif
#ifdef ENABLE_UART
		UartDrain();
#endif
		{
			// A console command, or the log pre-erase the console runs, can
			// hold the loop for up to ~300 ms (a sector erase): a squelch edge
			// first seen after that long is marked like one after a settings
			// save. A LOG DUMP's few lines a poll are not.
			const uint32_t t0 = SCHEDULER_Ticks10ms();
			ALERTCON_Poll();
			Stalled(t0);
		}
		dbgRawKey = (int16_t)key;
		dbgRawPtt = GPIO_IsPttPressed() ? 1u : 0u;
		if (key != lastKey) {
			if (key != KEY_INVALID && key != KEY_PTT)
				OnKey(key, false);
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

		// housekeeping every 10 ms: clock, RSSI, hold timers; screen and lines
		// only while the squelch is shut
		if (gNextTimeslice) {
			const bool quiet = Quiet();
			gNextTimeslice = false;
			nowMs += 10u;
			BACKLIGHT_Update();
			ALERTUI_Tick10ms();

			// Hold timers, in real 10 ms ticks. Counting loop passes made
			// "800 ms" of PTT about 160 ms, and ordinary presses dropped the
			// user out of the app. UP/DOWN repeat instead of leaving.
			if (dbgRawPtt) {
				if (++pttHeld10ms > 50)     // 500 ms
					running = false;
			} else {
				pttHeld10ms = 0;
			}
			if (dbgRawKey != (int16_t)KEY_INVALID) {
				++keyHeld10ms;
				if (dbgRawKey == (int16_t)KEY_UP || dbgRawKey == (int16_t)KEY_DOWN) {
					if (keyHeld10ms >= KEY_REPEAT_START && (keyHeld10ms % KEY_REPEAT_EVERY) == 0)
						OnKey((KEY_Code_t)dbgRawKey, true);
				} else if (keyHeld10ms > 250) {   // 2.5 s
					running = false;
				}
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
			if (!sqOpen && !draining && (uint32_t)(nowMs - lostMs) >= NF_HOLD_MS)
				NfSample();

			if ((tick % 10) == 0 && quiet)
				ALERTUI_DrawStatus();
			if (quiet) {
				if ((tick % 100) == 0)
					BatteryRefresh();
				if (setDirty) {
					for (uint8_t row = 0; row < SET_N; row++)
						if (setDirty & (1u << row))
							SetEvt(row);
					setDirty = 0;
				}
				if ((uint32_t)(nowMs - staMs) >= 10000u) {
					staMs = nowMs;
					EmitSta();
				}
				if (csvLines >= 200u && gAlertCfg.csv)
					ALERT_CsvHeader();   // for a host that connected since the last one
			}
			if ((tick % 50) == 0 && quiet) {
				// The ST7565 loses its register state when the BK4819 changes RF
				// state; this fork re-sends the init list after TX and after
				// sleep-wake for exactly that reason.
				ST7565_FixInterfGlitch();
				redraw = true;
				if (gAlertCfg.debug) {
					sprintf(lb, "D I%u F%u G%u B%u R%d Q%u N%u f%d p%d V%u C%u\r\n",
					        dbgIrqCount, stats.frames, stats.gated, lastBits, rssiDbm,
					        sqOpen ? 1u : 0u, burstCount, rssiFloor, lastPeak, stats.inv, stats.sqcap);
					DbgSend(lb);
				}
			}
		}

		// Never during a burst: a full-screen blit blocks the loop, and the
		// squelch edges are read here.
		if (redraw && Quiet()) {
			ALERTUI_Draw();
			redraw = false;
		}
	}

	// leave: ADC off, radio back to normal. The amplifier goes off first:
	// Shutdown drops the PA4B bias, and RADIO_SetupRegisters would only switch
	// PA8 off after that step had reached the speaker. RADIO_SetupRegisters
	// then leaves the audio as the radio keeps it between receptions (path
	// off, gEnableSpeaker false, AF per the VFO's modulation), whatever
	// SPEAKER was doing here.
	AUDIO_AudioPathOff();
	adcDelayed = false;
	txOpen     = false;
	if (adcRunning) {
		(void)ALERTADC_BurstStop(adcBits, 0);
		adcRunning = false;
	}
	ALERTADC_Shutdown();
	BK4819_WriteRegister(BK4819_REG_3F, 0);
	BK4819_WriteRegister(BK4819_REG_02, 0);
	Persist();
	RADIO_SetupRegisters(true);
#ifdef ENABLE_UART
	// what is still queued goes out now (<= 66 ms), or not at all: outside the
	// app the UART is the binary protocol's
	(void)UartRoom(255u);
	uqTail = uqHead;
#endif
	ST7565_FixInterfGlitch();   // leave the controller in a state the main UI can draw on
	gRequestDisplayScreen = DISPLAY_MAIN;
	gUpdateStatus  = true;
	gUpdateDisplay = true;
	DFU_WatchdogArm(false);
}

#endif // ENABLE_ALERT
