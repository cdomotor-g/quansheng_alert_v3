/* ALERT telemetry receiver app - see alert.h for the overview.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <string.h>

#include "app/alert.h"
#include "app/alert_adc.h"
#include "app/alert_decode.h"
#include "app/alert_stations.h"
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
#include "settings.h"
#include "ui/helper.h"
#include "ui/ui.h"
#ifdef ENABLE_UART
	#include "driver/uart.h"
#endif
#ifdef ENABLE_USB
	#include "driver/vcp.h"
#endif

// ---------------------------------------------------------------------------
// configuration (persisted in gEeprom.ALERT_CFG[5], see settings.c)
//
//   b[0]  <2:1> polarity, <3> voice, <4> SQ GATE, <5> show unknown,
//         <6> bit reverse, <7> monitor. <0> was the MODEM/ADC input switch;
//         the census now decides whether there is an ADC route at all.
//   b[1]  adopted arrangement index
//   b[2]  <1:0> route (ROUTE_*), <3:2> ADC pin (ALERTADC_SetChoice numbering),
//         <4> PA8 needed by that pin
//   b[3]  adopted variant byte (ApplyVariant)
//   b[4]  <7:4> CFG_MAGIC, <0> confirm
//
// The mode / sync / invert / rx gain / baud / capture fields went with the
// menu rows that set them: an arrangement is now a row of a register table, so
// there is nothing left to set by hand one field at a time.

enum { ROUTE_SWEEP = 0, ROUTE_FSK, ROUTE_ADC };

static struct {
	uint8_t  polarity;     // ALERT_POL_NEGATIVE / STANDARD / ANY
	bool     voice;        // read new readings out loud
	bool     gate;         // a burst must peak BURST_MARGIN_DB over the floor to count
	bool     show_unknown; // show addresses that are not in the station table
	bool     bitrev;       // reverse the bit order inside each captured byte
	bool     monitor;      // leave the audio path open (listen to the bursts)
	bool     confirm;      // require the same reading twice in one burst
	uint8_t  route;        // ROUTE_*: what has been adopted, if anything
	uint8_t  adopted;      // arrangement index (ROUTE_FSK)
	uint8_t  variant;      // variant byte applied to it
	uint8_t  adcPin;       // census choice (ROUTE_ADC)
	bool     adcPa8;
} cfg;

// Bumped from 0xB0: every byte after the first changed meaning, and a B0 sync
// word read back as an arrangement index would adopt something nobody chose.
#define CFG_MAGIC 0xC0u

void ALERT_LoadConfig(void)
{
	const uint8_t *b = gEeprom.ALERT_CFG;
	if ((b[4] & 0xF0u) != CFG_MAGIC) {
		// ANY, not NEGATIVE: nothing has established the sense on this radio,
		// and it is the table-station and repeat rules that keep chance hits
		// out, not a guess at the polarity.
		cfg.polarity = ALERT_POL_ANY;
		cfg.voice = true;  cfg.gate = true;  cfg.show_unknown = true;
		cfg.bitrev = false; cfg.monitor = false; cfg.confirm = false;
		cfg.route = ROUTE_SWEEP; cfg.adopted = 0; cfg.variant = 0;
		cfg.adcPin = 0; cfg.adcPa8 = false;
		return;
	}
	cfg.polarity     = (b[0] >> 1) & 3u;
	cfg.voice        = b[0] & (1u << 3);
	cfg.gate         = b[0] & (1u << 4);
	cfg.show_unknown = b[0] & (1u << 5);
	cfg.bitrev       = b[0] & (1u << 6);
	cfg.monitor      = b[0] & (1u << 7);
	cfg.adopted      = b[1];
	cfg.route        = b[2] & 3u;
	cfg.adcPin       = (b[2] >> 2) & 3u;
	cfg.adcPa8       = b[2] & (1u << 4);
	cfg.variant      = b[3];
	cfg.confirm      = b[4] & 1u;
	if (cfg.polarity > ALERT_POL_ANY) cfg.polarity = ALERT_POL_ANY;
	if (cfg.route > ROUTE_ADC)        cfg.route = ROUTE_SWEEP;
}

void ALERT_StoreConfig(void)
{
	uint8_t *b = gEeprom.ALERT_CFG;
	b[0] = (uint8_t)(((cfg.polarity & 3u) << 1) | (cfg.voice << 3) | (cfg.gate << 4)
	     | (cfg.show_unknown << 5) | (cfg.bitrev << 6) | (cfg.monitor << 7));
	b[1] = cfg.adopted;
	b[2] = (uint8_t)((cfg.route & 3u) | ((cfg.adcPin & 3u) << 2) | (cfg.adcPa8 << 4));
	b[3] = cfg.variant;
	b[4] = CFG_MAGIC | (cfg.confirm ? 1u : 0u);
}

// ---------------------------------------------------------------------------
// state

typedef struct {
	uint16_t id;
	uint16_t value;
	uint8_t  kind;
	uint8_t  format;
	int8_t   rssi_dbm;
	uint8_t  seq;        // burst sequence number (readings from one burst share it)
} History_t;

#define HISTORY_N 6
static History_t history[HISTORY_N];
static uint8_t   historyCount;
static uint8_t   burstSeq;

static struct {
	uint16_t syncs;      // FSK sync words seen
	uint16_t frames;     // readings delivered
	uint16_t gated;      // squelch openings that did not qualify as a burst
	uint16_t unknown;    // frames whose address is not in the table
	uint16_t stuck;      // noise-started streams cut off after a second
	uint16_t inv;        // frames that only decoded with the bits complemented
	uint16_t sqcap;      // squelch openings
} stats;

#define ALERT_DEFAULT_FREQ 15150000u   // 151.500 MHz, in units of 10 Hz

static uint16_t dbgIrqCount;     // times REG_0C reported an interrupt pending
static int16_t  dbgRawKey;       // what KEYBOARD_Poll returned THIS pass
static uint8_t  dbgRawPtt;       // PTT GPIO read directly, this pass

static bool     sqOpen;
static bool     sqPrev;         // for edge detection: bursts, not samples
static uint16_t burstCount;     // qualifying bursts since entering the app
// A squelch opening is not the same thing as a transmission. At SQL 1.0 the
// squelch flickers on noise, and a sweep that counted those advanced happily
// through eight arrangements with nothing on the air, scoring every one of
// them zero - indistinguishable from "tried it, does not work". A burst only
// counts if it rose well clear of the noise, and the floor is measured rather
// than assumed so it works wherever the radio happens to be.
#define BURST_MARGIN_DB 15
static int16_t  rssiFloor;      // slow minimum: the noise floor, in dBm
static int16_t  burstPeak;      // strongest RSSI during the burst in progress
static int16_t  lastPeak;       // ... of the last qualifying burst, for display
static int16_t  rssiDbm;
static bool     running;
static bool     redraw;
static uint8_t  view;            // VIEW_*
static uint8_t  setIndex;        // selected settings row
static uint8_t  lastCap[8];      // first bytes of the last window, for the raw view
static uint16_t lastCapLen;      // samples in it
static uint16_t lastRk, lastTr, lastLead;   // its statistics, for the main view
static uint16_t tick;
#ifdef ENABLE_VOICE
static bool     voiceBusy;
#endif

enum { VIEW_MAIN = 0, VIEW_SETTINGS, VIEW_RAW };

// A millisecond clock at 10 ms resolution, advanced by the main loop's
// timeslice. The scheduler's own counter is private to scheduler.c. Ticks are
// only lost while the loop is blocked, and nothing that blocks is allowed to
// run while a burst is in progress, which is the only time the clock has to be
// right to better than a tick or two.
static uint32_t nowMs;

// App entry: settle the RSSI floor, run the audio-pin census, then start.
enum { ENTRY_SETTLE = 0, ENTRY_CENSUS, ENTRY_RUN };
#define SETTLE_MS 10000u
static uint8_t  entry;
static uint32_t entryMs;
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

// same scaling as scale_freq() in driver/bk4819.c: register units for TONE1/2.
// The product overflows 32 bits above ~3170 Hz, which is why host recipes are
// capped at 3000.
static uint16_t tone_reg(uint16_t hz)
{
	return (uint16_t)((((uint32_t)hz * 1353245u) + (1u << 16)) >> 17);
}

// ---------------------------------------------------------------------------
// arrangements: one row of registers for the FSK engine
//
// Every mode this app tried before X1 was either a baseband slicer or a code
// Beken does not document (REG_58 0x04C3, 0x14C5), so the "FFSK1218" and
// "SAME" menu entries never selected FFSK or SAME receive at all. The table
// below puts the engine into the documented sub-carrier receive modes, at a
// TONE2 bit clock that oversamples the 300 baud signal, and lets software find
// the ALERT bits in what comes out. Tones are stored in Hz and converted by
// tone_reg(), so a hand-computed register value cannot be wrong again.

#define ARR_STREAM  0x01u     // stream policy: never re-arm on the squelch
#define ARR_59_S4   0x0008u   // REG_59<3>: 4-byte sync word

typedef struct {
	char     tag[4];
	uint16_t r58;     // <15:13> TX mode, <12:10> RX mode, <9:8> RX gain, <7:6> 11, <3:1> RX BW, <0> enable
	uint16_t r70;     // <15> TONE1 on, <14:8> TONE1 gain, <7> TONE2 on, <6:0> TONE2 gain
	uint16_t t1hz;    // TONE1, only written when r70<15> is set
	uint16_t t2hz;    // TONE2 = the FSK bit clock
	uint16_t r5c;
	uint16_t r59;     // REG_59 without FIFO/enable bits: <10> invert, <8> ?, <7:4> preamble, <3> 4-byte, <2:0> ?
	uint16_t s01;     // REG_5A: sync bytes 0 (high) and 1
	uint16_t s23;     // REG_5B: sync bytes 2 and 3, only with ARR_59_S4
	uint8_t  flags;   // ARR_STREAM
} Arr_t;

// tag, REG_58, TONE2, REG_70, TONE1, REG_5A, REG_5B, REG_59, REG_5C, flags
#define ARR_ROW(tg, r58, t2, r70, t1, s01, s23, r59, r5c, fl) \
	{ tg, (r58), (r70), (t1), (t2), (r5c), (r59), (s01), (s23), (fl) }
#define T12  0x80E0u   // TONE1 on at gain 0, TONE2 on at gain 96: what ta1js writes
#define T2   0x00E0u   // TONE2 only, gain 96
#define C56  0x5625u   // CRC off, what this app has always written
#define CAA  0xAA30u   // ta1js

// Phase 1, in the order the plan ranks them (tools/alert/X1_PLAN.md §3.2).
// k = TONE2 / 300 samples per ALERT bit.
static const Arr_t arrTab[] = {
	ARR_ROW("A1",  0x3FC3, 1200, T12, 2100, 0xFFFF, 0x0000, 0x0000, CAA, 0),          // ta1js RX clone
	ARR_ROW("A3",  0x3FC3, 1800, T12, 2100, 0xFFFF, 0x0000, 0x0000, C56, 0),          // off the 2100 Hz null
	ARR_ROW("B2",  0x03C5, 1042, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // SAME RX at 2x
	ARR_ROW("A2",  0x3FC3, 1200, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // TONE1 off
	ARR_ROW("A4",  0x3FC3, 1500, T12, 2100, 0xFFFF, 0x0000, 0x0000, C56, 0),
	ARR_ROW("A5",  0x3FC3, 1200, T12, 2100, 0xFFFF, 0xFFF0, ARR_59_S4, C56, 0),       // edge sync
	ARR_ROW("B1",  0x03C5, 521,  T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // SAME native rate
	ARR_ROW("A7",  0x3FC3, 1400, T12, 2100, 0xFFFF, 0x0000, 0x0000, C56, 0),          // tones scale: 1400/2100
	ARR_ROW("A8",  0x3FC3, 1300, T12, 2100, 0xFFFF, 0x0000, 0x0000, C56, 0),          // tones scale: 1300/1950
	ARR_ROW("C1",  0x73C9, 1200, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // FFSK1200/2400 RX
	ARR_ROW("A6",  0x3FC3, 1800, T12, 2100, 0xFFFF, 0xFFC0, ARR_59_S4, C56, 0),       // edge sync at k=6
	ARR_ROW("A9",  0x3FC3, 1200, T12, 2100, 0xAAAA, 0x0000, 0x0000, C56, 0),          // MSK hedge
	ARR_ROW("B4",  0x03C5, 1200, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),
	ARR_ROW("A10", 0x3F03, 1200, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // <7:6>=00
	ARR_ROW("A11", 0x3FC3, 1200, T12, 1300, 0xFFFF, 0x0000, 0x0000, C56, 0),          // TONE1 at space
	ARR_ROW("B3",  0x03C5, 525,  T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // SAME 1575/2100?
	ARR_ROW("C2",  0x73C9, 2400, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),
	ARR_ROW("C3",  0x73C9, 1050, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // tones scale: 1050/2100
	ARR_ROW("B5",  0xA3C5, 1042, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // SAME, TX mode 101
	ARR_ROW("S1",  0x3FC3, 1200, T12, 2100, 0xFFFF, 0x0000, 0x0000, CAA, ARR_STREAM), // A1, stream policy
	ARR_ROW("S2",  0x03C5, 1042, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, ARR_STREAM), // B2, stream policy
	ARR_ROW("D1",  0x03C1, 300,  T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // negative control
	ARR_ROW("D2",  0x03C1, 1200, T2,  0,    0xFFFF, 0x0000, 0x0000, C56, 0),          // direct FM at 4x
};
#undef ARR_ROW
#undef T12
#undef T2
#undef C56
#undef CAA

// Index space. 0..22 the table above, 32..39 recipes loaded by the host
// (0x0A01), 64..103 the Phase 3 REG_58 codes, generated rather than stored.
#define ARR_P1_N    ((uint8_t)(sizeof(arrTab) / sizeof(arrTab[0])))
#define ARR_A2      3u
#define ARR_HOST0   32u
#define ARR_HOST_N  8u
#define ARR_GEN0    64u
#define ARR_GEN_N   40u
#define ARR_ADC     0xFEu     // the ADC route, for scoring and adoption
#define ARR_NONE    0xFFu

static Arr_t hostArr[ARR_HOST_N];   // t2hz == 0: empty

// Register pokes applied after every full arm (0x0A03), so a recipe can be
// tried from the PC without a flash.
#define POKE_N 8
static struct { uint8_t reg; uint16_t andMask, orMask; } pokes[POKE_N];
static uint8_t pokeN;

// A variant byte modifies an arrangement: high nibble the operation, low
// nibble its argument. One modification per variant, so a score says exactly
// what changed.
//   0x1g RX gain g            0x20 RX BW 100 (FFSK1218 only)   0x30 4-byte sync
//   0x40 REG_59 invert        0x5p preamble nibble p           0x60 REG_59<8>
//   0x7n REG_59<2:0> = n
static void ApplyVariant(Arr_t *a, uint8_t v)
{
	switch (v >> 4) {
		case 1: a->r58 = (uint16_t)((a->r58 & ~0x0300u) | ((v & 3u) << 8)); break;
		case 2:
			if (((a->r58 >> 10) & 7u) == 7u)
				a->r58 = (uint16_t)((a->r58 & ~0x000Eu) | (4u << 1));
			break;
		case 3:
			// the same sixteen bits twice, as the old SYNC LEN setting did
			if (!(a->r59 & ARR_59_S4)) { a->r59 |= ARR_59_S4; a->s23 = a->s01; }
			break;
		case 4: a->r59 ^= 0x0400u; break;
		case 5: a->r59 = (uint16_t)((a->r59 & ~0x00F0u) | ((v & 15u) << 4)); break;
		case 6: a->r59 |= 0x0100u; break;
		case 7: a->r59 = (uint16_t)((a->r59 & ~0x0007u) | (v & 7u)); break;
		default: break;
	}
}

static bool ArrGet(uint8_t idx, uint8_t var, Arr_t *a)
{
	if (idx < ARR_P1_N) {
		*a = arrTab[idx];
	} else if (idx >= ARR_HOST0 && idx < ARR_HOST0 + ARR_HOST_N) {
		*a = hostArr[idx - ARR_HOST0];
		if (!a->t2hz)
			return false;
	} else if (idx >= ARR_GEN0 && idx < ARR_GEN0 + ARR_GEN_N) {
		// Undocumented RX modes 1,2,3,5,6 by every bandwidth code, at 1200 /
		// FFFF: (1<<13)|(rx<<10)|(3<<8)|(3<<6)|(bw<<1)|1.
		static const uint8_t genRx[5] = { 1, 2, 3, 5, 6 };
		const uint8_t i = (uint8_t)(idx - ARR_GEN0);
		*a = arrTab[ARR_A2];
		a->r58 = (uint16_t)((1u << 13) | ((uint16_t)genRx[i >> 3] << 10) | (3u << 8) | (3u << 6)
		                  | ((i & 7u) << 1) | 1u);
		a->tag[0] = 'U';
		a->tag[1] = (char)('0' + i / 10u);
		a->tag[2] = (char)('0' + i % 10u);
		a->tag[3] = 0;
	} else {
		return false;
	}
	ApplyVariant(a, var);
	return true;
}

static bool ArrKnown(uint8_t idx, uint8_t var)
{
	Arr_t a;
	return ArrGet(idx, var, &a);
}

static bool ArrSame(const Arr_t *a, const Arr_t *b)
{
	return a->r58 == b->r58 && a->r70 == b->r70 && a->t1hz == b->t1hz && a->t2hz == b->t2hz &&
	       a->r5c == b->r5c && a->r59 == b->r59 && a->s01 == b->s01 && a->s23 == b->s23 &&
	       a->flags == b->flags;
}

static Arr_t   cur;            // the arrangement armed now, variant applied
static uint8_t curIdx = ARR_NONE;
static uint8_t curVar;
static uint8_t curSlot;

// ---------------------------------------------------------------------------
// scores

// Per arrangement, saturating. A zero with bq = 0 means "never tested", not
// "does not work"; the Z line carries both so nobody has to guess which.
typedef struct {
	uint8_t bq;    // qualifying bursts
	uint8_t bb;    // ... delivering at least half the expected samples
	uint8_t bs;    // in-burst syncs, over bursts where the engine was searching at open
	uint8_t bt;    // structure passes
	uint8_t dx;    // bursts with any decode
	uint8_t dt;    // bursts with a table-station decode
	uint8_t rep;   // most separate bursts one table station decoded in
	uint8_t ns;    // syncs with the squelch shut
} Score_t;

// Phase 1 rows and host slots keep scores across passes. Everything else - a
// Phase 2 or Phase 3 variant, which is visited once - shares SLOT_TMP, which is
// reported and cleared when the sweep moves on.
#define SLOT_HOST0 ARR_P1_N
#define SLOT_ADC   ((uint8_t)(SLOT_HOST0 + ARR_HOST_N))
#define SLOT_TMP   ((uint8_t)(SLOT_ADC + 1u))
#define SLOT_N     ((uint8_t)(SLOT_TMP + 1u))
static Score_t scores[SLOT_N];
static bool    anyStruct;       // a structure pass anywhere, SLOT_TMP included

// Repeats are what separate a decode from chance: a random capture names a
// table station about 0.12% of the time, and the same one three bursts running
// essentially never. Only table stations are tracked.
#define REP_N 16
static struct { uint16_t id; uint8_t slot; uint8_t n; } reps[REP_N];

static void Inc(uint8_t *c)
{
	if (*c < 255u)
		(*c)++;
}

static uint8_t SlotOf(uint8_t idx, uint8_t var)
{
	if (var == 0 && idx < ARR_P1_N)
		return idx;
	if (var == 0 && idx >= ARR_HOST0 && idx < ARR_HOST0 + ARR_HOST_N)
		return (uint8_t)(SLOT_HOST0 + (idx - ARR_HOST0));
	return SLOT_TMP;
}

static uint8_t SlotIdx(uint8_t slot)
{
	if (slot < SLOT_HOST0) return slot;
	if (slot < SLOT_ADC)   return (uint8_t)(ARR_HOST0 + (slot - SLOT_HOST0));
	if (slot == SLOT_ADC)  return ARR_ADC;
	return curIdx;
}

static uint8_t RepBump(uint8_t slot, uint16_t id)
{
	uint8_t i, lo = 0;
	for (i = 0; i < REP_N; i++) {
		if (reps[i].n && reps[i].slot == slot && reps[i].id == id)
			break;
		if (reps[i].n < reps[lo].n)
			lo = i;
	}
	if (i == REP_N) {           // new: an empty entry, else the weakest
		i = lo;
		reps[i].id   = id;
		reps[i].slot = slot;
		reps[i].n    = 0;
	}
	Inc(&reps[i].n);
	return reps[i].n;
}

static bool Known(uint16_t id)
{
	const char *name; uint8_t kind;
	return ALERT_LookupStation(id, &name, &kind);
}

// Bump the repeat count of every table station in one burst's readings, once
// per station however many frames it sent. Returns the best count reached.
static uint8_t ScoreRep(uint8_t slot, const AlertReading_t *r, int n, uint16_t *id)
{
	uint8_t best = 0;
	for (int i = 0; i < n; i++) {
		bool dup = false;
		if (!Known(r[i].id))
			continue;
		for (int j = 0; j < i; j++)
			if (r[j].id == r[i].id) { dup = true; break; }
		if (dup)
			continue;
		const uint8_t c = RepBump(slot, r[i].id);
		if (c > best) { best = c; *id = r[i].id; }
	}
	if (best > scores[slot].rep)
		scores[slot].rep = best;
	return best;
}

// ---------------------------------------------------------------------------
// ADC route state (the sampler and demodulator are in alert_adc.c)

enum { ADCST_UNTESTED = 0, ADCST_OFF, ADCST_CONFIRM, ADCST_ON, ADCST_REJECT };
static uint8_t  adcState;
static bool     adcRunning;      // BurstStart called, BurstStop not yet
static uint8_t  confN, confOk;   // burst confirmation, first three qualifying bursts
static uint8_t  adcBits[64];     // one burst at 300 baud: 512 bits is 1.7 s

// PA8 follows MONITOR. When the census found audio only with PA8 on,
// ALERTADC_BurstStart switches it on for the burst and BurstStop puts it back,
// so the speaker is not left hissing between bursts on the ADC's account.
static void AudioPath(void)
{
	if (cfg.monitor) AUDIO_AudioPathOn(); else AUDIO_AudioPathOff();
}

// ---------------------------------------------------------------------------
// capture state

// A ring: position p of the stream lives at capBuf[p & 255], and the uint8_t
// head wraps by itself. Lengths are counted back from the head rather than as
// absolute positions, and saturate, so a stream that never finishes cannot wrap
// a counter into a small, plausible, wrong number.
static uint8_t  capBuf[256];
static uint8_t  ringHead;
static uint16_t capLen;          // bytes since the current stream began (sync prefix included)
static uint8_t  capPre;          // bytes of sync prefix at the start of that stream (0, 2, 4)
static uint16_t openLen;         // bytes since the squelch opened
static uint16_t validLen;        // bytes since the ring was last linearized in place
static uint16_t syncWords;       // words since the sync, for the F line
static uint16_t wordXor;         // 0xFFFF after a negative sync
static bool     streaming;       // the engine is emitting words and they are being kept
static bool     syncSeen;
static uint32_t syncMs, armMs, openMs, lostMs;
static char     syncPN;          // 'P' / 'N' from REG_0B, '?' neither, '-' FIFO without a sync
static bool     draining;        // squelch shut, still reading the FIFO for DRAIN_MS

enum { SRC_NONE = 0, SRC_OPEN, SRC_PRE };
static const char *const srcNames[3] = { "none", "open", "pre" };
static uint8_t  burstSrc;
static int16_t  burstDt;         // ms, sync - squelch open
static char     burstPN;
static bool     searchAtOpen;
static bool     syncInBurst;
static bool     burstTainted;    // re-armed mid-burst: the window is not one arrangement's
// The loop was just blocked - a full arm (~30 ms), the census (~3 s), a flash
// write. A squelch edge first seen right after one may be long past: the
// opening was never captured and the engine may have been reprogrammed under
// it, so that burst is tainted rather than scored as clean.
static bool     blocked;

#define DRAIN_MS       40u       // squelch lost -> BurstFinish; no word left stranded
#define BURST_MAX_MS   1500u     // longer than this is not an ALERT burst
#define NOISE_STREAM_MS 1000u    // search policy: a stream started on noise lasts this long
#define CARRIER_SYNC_MS 300u     // a sync this recent at squelch open is kept

// Y and F lines wait until the squelch shuts: sending blocks, and the FIFO
// holds eight words (107 ms at TONE2 1200, 53 ms at 2400).
#define YQ_N 4
static struct { uint32_t ms; int16_t rssi; char pn; uint8_t sq; } yq[YQ_N];
static uint8_t  yqN;
static bool     fPend;
static uint16_t fWords;
static uint32_t fMs;

static void EmitPending(void)
{
	for (uint8_t i = 0; i < yqN; i++) {
		sprintf(lb, "Y %u ms=%u pn=%c sq=%u rssi=%d\r\n", curIdx, (unsigned)yq[i].ms, yq[i].pn,
		        yq[i].sq, yq[i].rssi);
		DbgSend(lb);
	}
	yqN = 0;
	if (fPend) {
		// ~1024 words means REG_5D's 11-bit length works on this chip, ~128
		// that only its low byte does
		sprintf(lb, "F %u words=%u ms=%u\r\n", curIdx, fWords, (unsigned)fMs);
		DbgSend(lb);
		fPend = false;
	}
}

static void RingPut(uint8_t b)
{
	capBuf[ringHead++] = b;
	if (capLen   < 0xFFFFu) capLen++;
	if (openLen  < 0xFFFFu) openLen++;
	if (validLen < 0xFFFFu) validLen++;
}

static void Reverse(uint8_t *p, uint16_t n)
{
	if (n < 2u)
		return;
	uint8_t *q = p + n - 1u;
	while (p < q) {
		const uint8_t t = *p;
		*p++ = *q;
		*q-- = t;
	}
}

// Rotate the ring in place so the byte at index r lands at capBuf[0]. Three
// reversals, no second buffer: there is no RAM for one.
static void RingLinearize(uint8_t r)
{
	if (!r)
		return;
	Reverse(capBuf, r);
	Reverse(capBuf + r, (uint16_t)(sizeof(capBuf) - r));
	Reverse(capBuf, sizeof(capBuf));
}

// Each word low byte first, bits MSB-first within a byte (ta1js
// aprs_minimal.c:409-413), complemented after a negative sync as OneOfEleven's
// mdc1200.c does, so every stream arrives in the sense the sync was written in.
static void ReadWords(unsigned words)
{
	while (words--) {
		const uint16_t w = BK4819_ReadRegister(BK4819_REG_5F) ^ wordXor;
		if (!streaming)
			continue;
		RingPut((uint8_t)(w & 0xFFu));
		RingPut((uint8_t)(w >> 8));
		if (syncWords < 0xFFFFu)
			syncWords++;
	}
}

// ---------------------------------------------------------------------------
// BK4819 FSK engine

// REG_59: <15> clr TX FIFO, <14> clr RX FIFO, <13> scramble, <12> RX enable,
// <10> invert RX data, <7:4> preamble length, <3> 4-byte sync word.
//
// Preamble nibble 0 now, per arrangement: every receive precedent (OneOfEleven
// MDC, ta1js APRS) uses 0, and 6 was carried over from a TX path.
static uint16_t FskBase(void)
{
	return (uint16_t)(cur.r59 & 0x07FFu);
}

static void ModemStop(void)
{
	BK4819_WriteRegister(BK4819_REG_59, (1u << 14) | (1u << 15));   // clear FIFOs, RX/TX off
	BK4819_WriteRegister(BK4819_REG_58, 0);                          // FSK disable
	BK4819_WriteRegister(BK4819_REG_70, 0);
	BK4819_WriteRegister(BK4819_REG_5E, 0x3204);   // what aircopy, the one other reader, expects
	streaming = false;
}

// Light re-arm: drop the FIFO and search for the sync word again, without the
// 30 ms DSP restart of a full arm. REG_02 is cleared as well: an almost-full
// latched just before the re-arm would otherwise arrive with nothing streaming
// and be logged as sync-free output, which is the one thing that must not be
// faked.
static void ArrReArm(void)
{
	const uint16_t base = FskBase();
	BK4819_WriteRegister(BK4819_REG_59, base);
	BK4819_WriteRegister(BK4819_REG_59, base | (1u << 14));
	BK4819_WriteRegister(BK4819_REG_59, base | (1u << 12));
	BK4819_WriteRegister(BK4819_REG_02, 0);
	streaming = false;
	armMs     = nowMs;
}

static void ArrArm(void)
{
	// Start the way BK4819_PrepareFSKReceive() does. The piece that was once
	// missing here was the DSP restart: BK4819_ResetFSK() takes REG_30 to zero
	// through BK4819_Idle(), and BK4819_RX_TurnOn() brings it back up. Writing
	// REG_58/59 into an already-running receiver produced no sync at all.
	BK4819_ResetFSK();
	BK4819_WriteRegister(BK4819_REG_02, 0);
	BK4819_WriteRegister(BK4819_REG_3F, 0);
	BK4819_RX_TurnOn();

	BK4819_WriteRegister(BK4819_REG_70, cur.r70);
	if (cur.r70 & 0x8000u)
		BK4819_WriteRegister(BK4819_REG_71, tone_reg(cur.t1hz));
	BK4819_WriteRegister(BK4819_REG_72, tone_reg(cur.t2hz));
	BK4819_WriteRegister(BK4819_REG_58, cur.r58);
	BK4819_WriteRegister(BK4819_REG_5A, cur.s01);
	if (cur.r59 & ARR_59_S4)
		BK4819_WriteRegister(BK4819_REG_5B, cur.s23);
	BK4819_WriteRegister(BK4819_REG_5C, cur.r5c);
	// 2048 bytes: the field is N-1, <15:8> its low byte and <7:5> the top three
	// bits. The length only decides when RX_FINISHED fires; captures end on the
	// squelch, so long is right - a burst must never outlast the packet.
	BK4819_WriteRegister(BK4819_REG_5D, 0xFFE0);
	// The upper bits of aircopy's 0x3204 (which this app used to say nobody
	// wrote - bk4829.c:1145 does), almost-full at 2 words so at most one word
	// sits unread when the carrier drops.
	BK4819_WriteRegister(BK4819_REG_5E, 0x3202);

	{
		const uint16_t base = FskBase();
		BK4819_WriteRegister(BK4819_REG_59, base | (1u << 14) | (1u << 15));
		BK4819_WriteRegister(BK4819_REG_59, base);
		BK4819_WriteRegister(BK4819_REG_59, base | (1u << 12));
	}

	BK4819_WriteRegister(BK4819_REG_3F,
		BK4819_REG_3F_FSK_RX_SYNC | BK4819_REG_3F_FSK_FIFO_ALMOST_FULL | BK4819_REG_3F_FSK_RX_FINISHED |
		BK4819_REG_3F_SQUELCH_FOUND | BK4819_REG_3F_SQUELCH_LOST);
	BK4819_WriteRegister(BK4819_REG_02, 0);

	// AF stays at FM whatever the speaker is doing: the one FSK path in this
	// fork known to work (aircopy) never mutes REG_47, and the ADC census
	// wants the demodulated audio present. The speaker has its own gate.
	BK4819_SetAF(BK4819_AF_FM);
	AudioPath();

	for (uint8_t i = 0; i < pokeN; i++) {
		const BK4819_REGISTER_t reg = (BK4819_REGISTER_t)pokes[i].reg;
		BK4819_WriteRegister(reg, (uint16_t)((BK4819_ReadRegister(reg) & pokes[i].andMask) | pokes[i].orMask));
	}

	streaming = false;
	syncSeen  = false;
	wordXor   = 0;
	armMs     = nowMs;
	blocked   = true;
	if (sqOpen || draining)
		burstTainted = true;
}

// ---------------------------------------------------------------------------
// line emitters (formats: tools/alert/X1_PLAN.md §8)

static const char *IdxStr(uint8_t idx)
{
	static char s[4];
	if (idx == ARR_ADC)
		return "ADC";
	sprintf(s, "%u", idx);
	return s;
}

static uint8_t ShownPhase(void);

static void EmitL(void)
{
	sprintf(lb, "L %u %s r58=%04X t2=%u r70=%04X t1=%u sy=%04X%04X s4=%u 5c=%04X pol=%c ph=%u v=%02X\r\n",
	        curIdx, cur.tag, (unsigned)cur.r58, cur.t2hz, (unsigned)cur.r70, cur.t1hz,
	        (unsigned)cur.s01, (unsigned)cur.s23, (cur.r59 & ARR_59_S4) ? 1u : 0u, (unsigned)cur.r5c,
	        (cur.flags & ARR_STREAM) ? 'T' : 'S', ShownPhase(), curVar);
	DbgSend(lb);
}

static uint8_t pass;             // passes of any phase list completed

static void EmitZ(uint8_t slot)
{
	const Score_t *s = &scores[slot];
	const char *tag = "-";
	uint8_t idx = ARR_ADC, var = 0;
	Arr_t a;

	if (!s->bq && !s->ns)
		return;                  // never tested: say nothing rather than a row of zeroes
	if (slot != SLOT_ADC) {
		idx = SlotIdx(slot);
		var = (slot == SLOT_TMP) ? curVar : 0;
		if (ArrGet(idx, var, &a))
			tag = a.tag;
	}
	sprintf(lb, "Z %s %s v=%02X z=%u bq=%u bb=%u bs=%u bt=%u dx=%u dt=%u rep=%u ns=%u\r\n",
	        IdxStr(idx), tag, var, pass, s->bq, s->bb, s->bs, s->bt, s->dx, s->dt, s->rep, s->ns);
	DbgSend(lb);
}

static void EmitX(uint8_t idx, const AlertReading_t *r, bool inv)
{
	sprintf(lb, "X %s %u id=%u v=%u fmt=%s pol=%c inv=%u pos=%u known=%u\r\n", IdxStr(idx), burstCount,
	        r->id, r->value, r->format == ALERT_FMT_EIF ? "EIF" : "ABF",
	        r->polarity == ALERT_POL_STANDARD ? 'S' : 'N', inv ? 1u : 0u, r->bit_pos,
	        Known(r->id) ? 1u : 0u);
	DbgSend(lb);
}

static void EmitH(uint16_t off, uint16_t n)
{
	char *d = lb + sprintf(lb, "H %u %u %u ", curIdx, burstCount, off);
	d = PutHex(d, capBuf + off, n);
	strcpy(d, "\r\n");
	DbgSend(lb);
}

// ---------------------------------------------------------------------------
// sweep control (§3.2)

enum { MODE_HOLD = 0, MODE_SWEEP, MODE_PROD };
enum { PH_HOLD = 0, PH_1, PH_2, PH_3 };

static uint8_t mode;             // MODE_*
static uint8_t phase;            // PH_*, the phase being swept (kept while held)
static uint8_t step;             // position in that phase's list
static uint8_t p1Passes;
static bool    p2Done, p3Done;
static uint8_t p2Base[2];        // Phase 2 bases, arrangement indices
static uint8_t dwellSet = 1;     // bursts per arrangement per pass (0x0A02 op 5)
static uint8_t dwellQ, dwellPre; // bursts on this arrangement, and those that found it streaming
static uint8_t prodFail;         // production: qualifying bursts in a row with no decode

// Phase 2 tries each of these on its two best bases; Phase 3 tries REG_59<8>
// and REG_59<2:0> on A1.
static const uint8_t p2Vars[8] = { 0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x40, 0x56 };
#define P2_BURSTS 2u
#define PROD_FAILS 20u

static uint8_t ShownPhase(void)
{
	return mode == MODE_SWEEP ? phase : (uint8_t)PH_HOLD;
}

static uint8_t PhaseSteps(uint8_t ph)
{
	switch (ph) {
		case PH_1: return (uint8_t)(ARR_P1_N + ARR_HOST_N);
		case PH_2: return 16u;
		case PH_3: return (uint8_t)(ARR_GEN_N + 8u);
		default:   return 0u;
	}
}

// The arrangement at step st of phase ph, or false if that step is skipped.
static bool PhaseEntry(uint8_t ph, uint8_t st, uint8_t *idx, uint8_t *var)
{
	switch (ph) {
		case PH_1:
			// the table, then any recipes the host has loaded
			*var = 0;
			if (st < ARR_P1_N) {
				*idx = st;
				return true;
			}
			*idx = (uint8_t)(ARR_HOST0 + (st - ARR_P1_N));
			return ArrKnown(*idx, 0);
		case PH_2: {
			Arr_t a, b;
			*idx = p2Base[st >> 3];
			*var = p2Vars[st & 7u];
			// a variant that changes nothing for this base is not worth a burst
			return *idx != ARR_NONE && ArrGet(*idx, 0, &a) && ArrGet(*idx, *var, &b) && !ArrSame(&a, &b);
		}
		case PH_3:
			if (st < ARR_GEN_N) {
				*idx = (uint8_t)(ARR_GEN0 + st);
				*var = 0;
			} else {
				*idx = 0;
				*var = (st == ARR_GEN_N) ? 0x60u : (uint8_t)(0x70u | (st - ARR_GEN_N));
			}
			return true;
		default:
			return false;
	}
}

// The two best arrangements that showed structure or decoded anything.
static bool Phase2Pick(void)
{
	uint32_t best[2] = { 0, 0 };
	p2Base[0] = p2Base[1] = ARR_NONE;
	for (uint8_t sl = 0; sl < SLOT_ADC; sl++) {
		const Score_t *s = &scores[sl];
		if (!s->bt && !s->dx)
			continue;
		const uint32_t key = ((uint32_t)s->dt << 24) | ((uint32_t)s->dx << 16) | ((uint32_t)s->bt << 8) | s->bb;
		if (key > best[0]) {
			best[1] = best[0]; p2Base[1] = p2Base[0];
			best[0] = key;     p2Base[0] = SlotIdx(sl);
		} else if (key > best[1]) {
			best[1] = key;     p2Base[1] = SlotIdx(sl);
		}
	}
	return p2Base[0] != ARR_NONE;
}

static void SetArr(uint8_t idx, uint8_t var)
{
	EmitPending();                           // lines still owed to the old arrangement
	if (curSlot == SLOT_TMP && curIdx != ARR_NONE && (idx != curIdx || var != curVar)) {
		// a variant is visited once: report it now, before its slot is reused
		EmitZ(SLOT_TMP);
		memset(&scores[SLOT_TMP], 0, sizeof(scores[SLOT_TMP]));
		for (uint8_t i = 0; i < REP_N; i++)
			if (reps[i].slot == SLOT_TMP)
				reps[i].n = 0;
	}
	if (!ArrGet(idx, var, &cur)) {           // a host slot emptied under us
		idx = 0;
		var = 0;
		ArrGet(0, 0, &cur);
	}
	curIdx   = idx;
	curVar   = var;
	curSlot  = SlotOf(idx, var);
	dwellQ   = 0;
	dwellPre = 0;
	ArrArm();
	EmitL();
	redraw = true;
}

// A phase list ran out: report, then pick what comes next.
static uint8_t PassEnd(uint8_t ph)
{
	for (uint8_t sl = 0; sl < SLOT_TMP; sl++)
		EmitZ(sl);
	Inc(&pass);
	if (ph == PH_1) Inc(&p1Passes);
	if (ph == PH_2) p2Done = true;
	if (ph == PH_3) p3Done = true;
	// Phase 2 refines anything that showed signs of life; Phase 3 goes looking
	// in undocumented corners only when nothing documented showed any.
	if (p1Passes >= 2u && !p2Done && Phase2Pick())
		return PH_2;
	if (p1Passes >= 2u && !p3Done && !anyStruct)
		return PH_3;
	return PH_1;
}

// First usable entry at or after st, moving on through the phases as their
// lists run out. Phase 1 always has entries, so this always lands somewhere.
static void SweepGo(uint8_t ph, uint8_t st)
{
	uint8_t idx = 0, var = 0;
	for (uint8_t guard = 0; guard < 200u; guard++) {
		if (st >= PhaseSteps(ph)) {
			ph = PassEnd(ph);
			st = 0;
			continue;
		}
		if (PhaseEntry(ph, st, &idx, &var))
			break;
		st++;
	}
	phase = ph;
	step  = st;
	SetArr(idx, var);
}

static void SweepStart(uint8_t ph)
{
	mode = MODE_SWEEP;
	if (ph == PH_2)
		Phase2Pick();
	SweepGo(ph, 0);
}

static void Persist(void)
{
	ALERT_StoreConfig();
	DFU_WatchdogKick();
	SETTINGS_SaveSettings();
	DFU_WatchdogKick();
	blocked = true;
}

static void Adopt(uint8_t idx, uint8_t var, uint16_t id, uint8_t rep)
{
	if (idx == ARR_ADC) {
		if (!ALERTADC_ChoicePin())
			return;              // nothing to adopt: the census found no pin
		cfg.route  = ROUTE_ADC;
		cfg.adcPin = ALERTADC_ChoicePin();
		cfg.adcPa8 = ALERTADC_ChoicePa8();
		adcState   = ADCST_ON;
	} else {
		cfg.route   = ROUTE_FSK;
		cfg.adopted = idx;
		cfg.variant = var;
	}
	sprintf(lb, "G ADOPT %s v=%02X id=%u rep=%u\r\n", IdxStr(idx), var, id, rep);
	DbgSend(lb);
	mode     = MODE_PROD;
	prodFail = 0;
	// A host slot is RAM only, so an adoption of one lasts until the next boot;
	// ALERT_LoadConfig's caller drops it then rather than arm an empty recipe.
	Persist();
	if (idx != ARR_ADC && (idx != curIdx || var != curVar))
		SetArr(idx, var);
	redraw = true;
}

// Production gave up: twenty qualifying bursts in a row with nothing decoded.
// The adoption stays persisted - the next entry tries it again first - but the
// repeat counts start over so it has to earn its way back.
static void Resume(void)
{
	sprintf(lb, "G RESUME %s fails=%u\r\n", IdxStr(cfg.route == ROUTE_ADC ? ARR_ADC : curIdx), prodFail);
	DbgSend(lb);
	memset(reps, 0, sizeof(reps));
	prodFail = 0;
	SweepStart(PH_1);
}

// ---------------------------------------------------------------------------
// burst end (§3.4)

static void Deliver(const AlertReading_t *r, int n, int16_t rssi);

// For an edge sync (a run of idle then the start bit, like FFFFFFF0) the
// capture holds only that run of idle before the first start edge. Its length
// in samples, or 0 when the sync is a steady preamble or alternating.
static uint8_t EdgeSyncRun(void)
{
	const uint32_t pat = ((uint32_t)cur.s01 << 16) | cur.s23;
	const uint8_t  nb  = (cur.r59 & ARR_59_S4) ? 32u : 16u;
	const uint32_t top = pat >> 31;
	uint8_t run = 1;
	while (run < nb && ((pat >> (31u - run)) & 1u) == top)
		run++;
	return (run < 8u || run >= nb) ? 0u : run;
}

// ... so the decoder's idle gate has to come down to what the sync provides.
// tools/alert/sweep_judge.py (min_idle_bits) computes the same from the L line.
static uint8_t MinIdleBits(void)
{
	const uint8_t run = EdgeSyncRun();
	if (!run)
		return 12u;
	const uint32_t bits = (uint32_t)run * 300u / cur.t2hz;
	return (uint8_t)(bits < 12u ? bits : 12u);
}

// STRUCT (plan section 3.4, k >= 3 only), measured as sweep_judge.struct_test
// measures it, so the firmware's Phase 2/3 decisions and the judge agree. Over
// the whole window a real burst cannot pass: the window may start 100 ms before
// the open inside a noise-started stream and always runs DRAIN_MS plus the
// squelch's own close delay past the burst, and that noise adds more short runs
// than a whole burst has runs. So the lead is the longest run that starts in
// the window's first 250 ms (the preamble, wherever the window began), and rk
// and transitions are taken from it through one frame (44 bits). Noise still
// fails: its longest early run is a few samples against the 20k required.
//
// An edge sync leaves no preamble in the window, only the sync's own idle run.
// When the window starts with it, the core starts there (a gap between frames,
// or the idle after the last, can be longer) and the lead needed comes down to
// that run.
static bool StructPass(uint32_t n, uint16_t t2, uint8_t m)
{
	if (t2 < 900u || n < 2u)
		return false;
	const uint8_t  edge    = EdgeSyncRun();
	const uint32_t horizon = (uint32_t)t2 / 4u;               // 250 ms of samples
	uint32_t best = 0, bestAt = 0, pos = 0, run = 0, first = 0;
	for (uint32_t i = 0; i < n && pos < horizon; i++) {
		run++;
		if (i + 1u == n || ALERT_GetBit(capBuf, i + 1u) != ALERT_GetBit(capBuf, i)) {
			if (!pos)         first = run;
			if (run > best) { best = run; bestAt = pos; }
			pos += run;
			run = 0;
		}
	}
	if (edge && first >= edge) {
		best   = first;
		bestAt = 0;
	}
	uint32_t end = bestAt + best + 44u * t2 / 300u;          // int(FRAME_BITS * k)
	if (end > n)
		end = n;
	uint32_t trans = 0, runs = 0, runsM = 0;
	run = 0;
	for (uint32_t i = bestAt; i < end; i++) {
		run++;
		if (i + 1u == end || ALERT_GetBit(capBuf, i + 1u) != ALERT_GetBit(capBuf, i)) {
			if (i + 1u < end) trans++;
			runs++;
			if (run >= m) runsM++;
			run = 0;
		}
	}
	const uint32_t coreLen = end - bestAt;
	uint32_t leadMin = (t2 + 14u) / 15u;                      // 20k samples, rounded up
	if (edge && edge < leadMin)
		leadMin = edge;
	// rk >= 0.80, transitions / sample <= 0.6 / k, exactly (no per-mille rounding)
	return runs && runsM * 5u >= runs * 4u && trans * t2 <= 180u * (coreLen > 1u ? coreLen - 1u : 1u) &&
	       best >= leadMin;
}

// The prefix StreamBegin wrote is in programmed order already, whatever the
// engine does inside a byte, so it is skipped (sweep_judge's lsb order does the
// same); reversing it would put a 0F where an edge sync's F0 ends.
static void BitReverse(uint16_t from, uint16_t len)
{
	for (uint16_t i = from; i < len; i++) {
		uint8_t b = capBuf[i];
		b = (uint8_t)(((b & 0x55u) << 1) | ((b & 0xAAu) >> 1));
		b = (uint8_t)(((b & 0x33u) << 2) | ((b & 0xCCu) >> 2));
		capBuf[i] = (uint8_t)((b << 4) | (b >> 4));
	}
}

static void BurstFinish(void)
{
	AlertReading_t r[12];            // FSK first, then ADC
	int      nf = 0, nf0 = 0, na = 0, na0 = 0;
	uint16_t adcN = 0;
	const bool     adcRan = adcRunning;
	const bool     stream = (cur.flags & ARR_STREAM) != 0;
	const uint32_t dur    = lostMs - openMs;

	if (adcRunning) {
		adcN = ALERTADC_BurstStop(adcBits, (uint16_t)(sizeof(adcBits) * 8u));
		adcRunning = false;
		AudioPath();             // MONITOR may have changed during the burst; it wins
	}
	EmitPending();
	redraw = true;

	// Qualifying: 100-1500 ms of open squelch, and with SQ GATE on a peak well
	// clear of the floor. SQ GATE used to reject captures made with the squelch
	// shut; every capture is squelch-bounded now, so the gate moved to the one
	// thing that still separates a burst from a flicker.
	if (entry != ENTRY_RUN || burstTainted || dur < 100u || dur > BURST_MAX_MS ||
	    (cfg.gate && burstPeak < (int16_t)(rssiFloor + BURST_MARGIN_DB))) {
		stats.gated++;
		if (!stream)
			ArrReArm();
		return;
	}
	burstCount++;
	lastPeak = burstPeak;

	// 1. window: from max(stream start, squelch open - 100 ms) to squelch lost
	//    + DRAIN_MS, bounded by what the ring still holds
	const uint16_t t2  = cur.t2hz;
	const uint16_t win = (uint16_t)(dur + 100u + DRAIN_MS);
	uint16_t len = 0;
	if (burstSrc != SRC_NONE) {
		const uint32_t lim = (uint32_t)openLen + t2 / 80u;   // 100 ms of samples, in bytes
		len = capLen;
		if (len > lim)             len = (uint16_t)lim;
		if (len > validLen)        len = validLen;
		if (len > sizeof(capBuf))  len = sizeof(capBuf);
	}
	if (len) {
		RingLinearize((uint8_t)(ringHead - len));
		validLen = 0;            // everything behind the head is now out of order
	}
	// the window starts at the stream's start exactly when it holds all of it
	const uint16_t pre = (len && len == capLen) ? capPre : 0u;
	if (cfg.bitrev)
		BitReverse(pre, len);

	// 2. statistics, in the order the decoder reads the bits
	const uint32_t n = (uint32_t)len * 8u;
	uint8_t m = (uint8_t)((t2 + 200u) / 400u);          // round(0.75 k)
	if (m < 2u) m = 2u;
	uint16_t tr = 0, rk = 0, maxrun = 0, lead = 0;
	{
		uint16_t trans = 0, runs = 0, runsM = 0, run = 0;
		for (uint32_t i = 0; i < n; i++) {
			run++;
			if (i + 1u == n || ALERT_GetBit(capBuf, i + 1u) != ALERT_GetBit(capBuf, i)) {
				if (i + 1u < n) trans++;
				if (!runs)        lead = run;
				runs++;
				if (run >= m)     runsM++;
				if (run > maxrun) maxrun = run;
				run = 0;
			}
		}
		if (n)    tr = (uint16_t)((uint32_t)trans * 1000u / n);
		if (runs) rk = (uint16_t)((uint32_t)runsM * 1000u / runs);
	}
	const uint16_t base = (uint16_t)(1000u >> (m - 1u));  // rk of random bits: 2^(1-m)
	// The C line keeps the whole-window figures (plan section 8); the structure
	// verdict is taken on the burst itself, see StructPass.
	const bool structured = StructPass(n, t2, m);
	const bool gotBits = n && (uint32_t)n * 2000u >= (uint32_t)win * t2;
	lastRk = rk; lastTr = tr; lastLead = lead;

	// 3. decode, both data senses; the polarity argument covers both framings
	if (n >= 40u) {
		const uint32_t spb = ((uint32_t)t2 * 256u + 150u) / 300u;
		if (spb < 384u) {
			// under 1.5 samples a bit (D1): one sample per bit, the old framer,
			// behind the same 12-bit preamble gate as everything else. Ungated,
			// 2048 random bits name a table station in ~5% of windows, and D1 is
			// the negative control: a chance hit there would read as a decode.
			nf  = ALERT_ScanBitsGated(capBuf, n, cfg.polarity, 20, false, 12u, r, 8);
			nf0 = nf;
			if (nf < 8)
				nf += ALERT_ScanBitsGated(capBuf, n, cfg.polarity, 20, true, 12u, r + nf, 8 - nf);
		} else if (spb <= ALERT_SPB_Q8_MAX) {
			const uint8_t idle = MinIdleBits();
			nf  = ALERT_ScanSamples(capBuf, n, spb, cfg.polarity, false, idle, r, 8);
			nf0 = nf;
			if (nf < 8)
				nf += ALERT_ScanSamples(capBuf, n, spb, cfg.polarity, true, idle, r + nf, 8 - nf);
		}
	}
	if (adcN >= 40u) {
		// Gated as well: the demodulator starts at the squelch open, well inside
		// the preamble, so a real frame always has its 12 idle bits.
		na  = ALERT_ScanBitsGated(adcBits, adcN, cfg.polarity, 20, false, 12u, r + nf, 12 - nf);
		na0 = na;
		if (nf + na < 12)
			na += ALERT_ScanBitsGated(adcBits, adcN, cfg.polarity, 20, true, 12u, r + nf + na, 12 - nf - na);
	}
	stats.inv += (uint16_t)((nf - nf0) + (na - na0));

	uint8_t tabF = 0, tabA = 0;
	for (int i = 0; i < nf; i++)      if (Known(r[i].id)) tabF++;
	for (int i = nf; i < nf + na; i++) if (Known(r[i].id)) tabA++;

	// 4. report: C, then H (in stored order, so the bit reversal comes off
	//    again), then X
	{
		char dts[8];
		if (burstSrc == SRC_NONE) strcpy(dts, "NA"); else sprintf(dts, "%d", burstDt);
		sprintf(lb, "C %u b=%u pk=%d fl=%d win=%u src=%s dt=%s pn=%c w=%u n=%u k=%u tr=%u rk=%u base=%u"
		            " mx=%u lead=%u dec=%u tab=%u\r\n",
		        curIdx, burstCount, burstPeak, rssiFloor, win, srcNames[burstSrc], dts,
		        burstSrc == SRC_NONE ? '-' : burstPN, len / 2u, (unsigned)n, t2 / 3u, tr, rk, base,
		        maxrun, lead, (unsigned)nf, tabF);
		DbgSend(lb);
	}
	if (cfg.bitrev)
		BitReverse(pre, len);
	for (uint16_t off = 0; off < len; off += 64u)
		EmitH(off, (uint16_t)((len - off) < 64u ? (len - off) : 64u));
	lastCapLen = (uint16_t)n;
	memset(lastCap, 0, sizeof(lastCap));
	memcpy(lastCap, capBuf, len < sizeof(lastCap) ? len : sizeof(lastCap));
	for (int i = 0; i < nf; i++)
		EmitX(curIdx, &r[i], i >= nf0);
	for (int i = nf; i < nf + na; i++)
		EmitX(ARR_ADC, &r[i], i >= nf + na0);

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
		// §5 burst confirmation: both tones 10 dB over the census's idle level
		// on two of the first three qualifying bursts. The energies themselves
		// went out in the P src=BURST lines ALERTADC_BurstStop sent.
		int16_t g13 = ALERTADC_LEVEL_NONE, g21 = ALERTADC_LEVEL_NONE, idle = ALERTADC_LEVEL_NONE;
		ALERTADC_LastBurstEnergy(&g13, &g21, &idle);
		const bool ok = g13 != ALERTADC_LEVEL_NONE && g21 != ALERTADC_LEVEL_NONE &&
		                idle != ALERTADC_LEVEL_NONE && (g13 - idle) >= 100 && (g21 - idle) >= 100;
		if (ok) confOk++;
		if (++confN >= 3u)
			adcState = (confOk >= 2u) ? ADCST_ON : ADCST_REJECT;
	}

	Deliver(r, nf + na, burstPeak);

	// 5. score
	uint16_t idF = 0, idA = 0;
	uint8_t  repF, repA = 0;
	{
		Score_t *s = &scores[curSlot];
		Inc(&s->bq);
		if (gotBits)                     Inc(&s->bb);
		if (searchAtOpen && syncInBurst) Inc(&s->bs);
		if (structured)                { Inc(&s->bt); anyStruct = true; }
		if (nf)                          Inc(&s->dx);
		if (tabF)                        Inc(&s->dt);
		repF = ScoreRep(curSlot, r, nf, &idF);
	}
	if (adcRan) {
		Score_t *s = &scores[SLOT_ADC];
		Inc(&s->bq);
		if ((uint32_t)adcN * 2000u >= (uint32_t)win * 300u) Inc(&s->bb);
		if (na)   Inc(&s->dx);
		if (tabA) Inc(&s->dt);
		repA = ScoreRep(SLOT_ADC, r + nf, na, &idA);
	}

	// 6. adopt, or in production watch for the route going quiet
	if (mode != MODE_PROD) {
		if (repF >= 3u)      Adopt(curIdx, curVar, idF, repF);
		else if (repA >= 3u) Adopt(ARR_ADC, 0, idA, repA);
	} else {
		const bool ok = (cfg.route == ROUTE_ADC) ? (na > 0) : (nf > 0);
		if (ok) {
			prodFail = 0;
		} else if (++prodFail >= PROD_FAILS) {
			Resume();
			return;              // SetArr re-armed
		}
	}

	// 7. dwell. The stream rows wait for bursts that land in a stream already
	//    running, which is the whole of what they test.
	if (mode == MODE_SWEEP) {
		Inc(&dwellQ);
		if (burstSrc == SRC_PRE) Inc(&dwellPre);
		bool done;
		if (stream)              done = dwellPre >= 2u || dwellQ >= 6u;
		// Phase 3 runs once, so it takes its two bursts in one visit: the
		// "FSK route dead" verdict (plan section 9) needs two on every row.
		else if (phase == PH_2 || phase == PH_3)
		                         done = dwellQ >= P2_BURSTS;
		else                     done = dwellQ >= dwellSet;
		// A table decode buys more bursts: adoption needs three, and at one
		// burst per pass collecting them would take three whole passes.
		if (done && tabF && dwellQ < 8u)
			done = false;
		if (done) {
			SweepGo(phase, (uint8_t)(step + 1u));
			return;              // SetArr re-armed
		}
	}
	if (!stream)
		ArrReArm();
}

// ---------------------------------------------------------------------------
// capture state machine (§3.3)

static int16_t ClampMs(uint32_t later, uint32_t earlier)
{
	const int32_t d = (int32_t)(later - earlier);
	return (int16_t)(d > 32767 ? 32767 : (d < -32768 ? -32768 : d));
}

// The engine started emitting words: a sync (prefix = true) or, if it ever
// happens, FIFO data with no sync in front of it - output no documented mode
// produces, which is why it is logged (pn '-') rather than ignored.
static void StreamBegin(uint16_t xorMask, bool prefix, char pn)
{
	syncSeen = true;
	syncMs   = nowMs;
	syncPN   = pn;
	if (yqN < YQ_N) {
		yq[yqN].ms   = nowMs - armMs;
		yq[yqN].rssi = rssiDbm;
		yq[yqN].pn   = pn;
		yq[yqN].sq   = sqOpen ? 1u : 0u;
		yqN++;
	}
	if (sqOpen || draining) {
		syncInBurst = true;
		if (burstSrc == SRC_NONE) {
			burstSrc = SRC_OPEN;
			burstDt  = ClampMs(syncMs, openMs);
			burstPN  = pn;
		}
	} else if (prefix) {
		Inc(&scores[curSlot].ns);
	}

	wordXor   = xorMask;
	streaming = true;
	capLen    = 0;
	capPre    = prefix ? ((cur.r59 & ARR_59_S4) ? 4u : 2u) : 0u;
	syncWords = 0;
	if (prefix) {
		// The sync pattern goes in first, so an edge sync keeps its start bit
		// and the idle gate has the idle samples the sync consumed. It is
		// written as programmed, not complemented: after a negative sync the
		// words are complemented back (ReadWords), and in that sense the
		// pattern that matched is the one programmed.
		RingPut((uint8_t)(cur.s01 >> 8));
		RingPut((uint8_t)(cur.s01 & 0xFFu));
		if (cur.r59 & ARR_59_S4) {
			RingPut((uint8_t)(cur.s23 >> 8));
			RingPut((uint8_t)(cur.s23 & 0xFFu));
		}
	}
}

static void BurstOpen(void)
{
	openMs       = nowMs;
	openLen      = 0;
	burstPeak    = rssiDbm;
	burstTainted = false;
	syncInBurst  = false;
	searchAtOpen = !streaming;
	stats.sqcap++;
	if (streaming) {
		burstSrc = SRC_PRE;
		burstDt  = ClampMs(syncMs, openMs);   // negative: the sync came first
		burstPN  = syncPN;
	} else {
		burstSrc = SRC_NONE;
		burstPN  = '-';
	}
	// Search policy: start the burst with a clean search, unless the engine is
	// already streaming or synced very recently - a sync on the carrier or the
	// tone onset is exactly what we hope for, and re-arming would throw it away.
	if (!(cur.flags & ARR_STREAM) && !streaming &&
	    !(syncSeen && (uint32_t)(nowMs - syncMs) < CARRIER_SYNC_MS))
		ArrReArm();
	if (adcState == ADCST_CONFIRM || adcState == ADCST_ON) {
		// TIM3 runs only while the squelch is open: the sampler's own RF
		// interference is what made fagci drop the same pipeline
		ALERTADC_BurstStart();
		adcRunning = true;
	}
}

// Nothing that blocks - a redraw, a USB line, a re-arm - may run while a burst
// could be filling the FIFO. Once the squelch has been open well past the
// longest burst there is nothing left to protect (a voice conversation on the
// channel would otherwise freeze the screen and the heartbeat for its length).
static bool Quiet(void)
{
	if (draining)
		return false;
	return !sqOpen || (uint32_t)(nowMs - openMs) > BURST_MAX_MS + 500u;
}

static void ArrPoll(void)
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
		// Keep reading the FIFO for DRAIN_MS, then finish. Never wait for
		// RX_FINISHED: ta1js found it often never fires once the carrier drops.
		draining = true;
		lostMs   = nowMs;
	}
	sqPrev  = sqOpen;
	blocked = false;                 // anything blocking from here on marks it again
	if (sqOpen && rssiDbm > burstPeak)
		burstPeak = rssiDbm;

	// Bounded. An unbounded "while (REG_0C & 1)" once froze the whole radio when
	// writing REG_02 did not clear the pending flag.
	for (uint8_t guard = 0; guard < 16 && (BK4819_ReadRegister(BK4819_REG_0C) & 1u); guard++) {
		BK4819_WriteRegister(BK4819_REG_02, 0);
		const uint16_t irq = BK4819_ReadRegister(BK4819_REG_02);
		dbgIrqCount++;

		if (irq & BK4819_REG_02_FSK_RX_SYNC) {
			// REG_0B<7> SyncN, <6> SyncP: the chip matches the complement too
			const uint16_t r0b = BK4819_ReadRegister(BK4819_REG_0B);
			stats.syncs++;
			StreamBegin((r0b & 0x80u) ? 0xFFFFu : 0u, true,
			            (r0b & 0x80u) ? 'N' : ((r0b & 0x40u) ? 'P' : '?'));
		}
		if (irq & BK4819_REG_02_FSK_FIFO_ALMOST_FULL) {
			if (!streaming)
				StreamBegin(0, false, '-');
			ReadWords(2);            // the REG_5E threshold
		}
		if (irq & BK4819_REG_02_FSK_RX_FINISHED) {
			if (streaming) {
				fPend  = true;
				fWords = syncWords;
				fMs    = nowMs - syncMs;
			}
			ArrReArm();              // the packet is over; both policies search again
		}
	}

	if (draining && (uint32_t)(nowMs - lostMs) >= DRAIN_MS) {
		draining = false;
		BurstFinish();
	}

	// Search policy: a stream a noise sync started is cut after a second, so
	// the engine is usually searching when a burst begins.
	if (streaming && !(cur.flags & ARR_STREAM) && !sqOpen && !draining &&
	    (uint32_t)(nowMs - syncMs) >= NOISE_STREAM_MS) {
		stats.stuck++;
		ArrReArm();
	}
}

// ---------------------------------------------------------------------------
// entry sequence and the census (§2, §5)

static void Draw(void);

static void RunCensus(void)
{
	bool ok;
	censusBusy = true;
	Draw();                          // it blocks for seconds; say why the screen froze
	DFU_WatchdogKick();
	ok = ALERTADC_Census(DbgSend);
	DFU_WatchdogKick();
	censusBusy = false;
	censusReq  = false;              // after, so the menu row reads PENDING throughout
	if (ok) {
		cfg.adcPin = ALERTADC_ChoicePin();
		cfg.adcPa8 = ALERTADC_ChoicePa8();
		adcState   = (cfg.route == ROUTE_ADC) ? ADCST_ON : ADCST_CONFIRM;
	} else if (cfg.route == ROUTE_ADC && cfg.adcPin) {
		// Adopted before, missed now - the census depends on the volume knob
		// among other things. Trust the bursts that earned the adoption.
		ALERTADC_SetChoice(cfg.adcPin, cfg.adcPa8);
		adcState = ADCST_ON;
	} else {
		adcState = ADCST_OFF;
	}
	confN = confOk = 0;
	ArrArm();                        // the census reprogrammed the BK4819
	blocked = true;                  // (ArrArm says so too; the census is the long part)
	redraw = true;
}

static void StartRoute(void)
{
	if (cfg.route == ROUTE_FSK) {
		mode = MODE_PROD;
		SetArr(cfg.adopted, cfg.variant);
	} else if (cfg.route == ROUTE_ADC) {
		mode = MODE_PROD;            // the FSK engine stays where it is and keeps logging
	} else {
		SweepStart(PH_1);
	}
	prodFail = 0;
}

// ---------------------------------------------------------------------------
// host commands (0x0A01-0x0A03, byte layouts in alert.h)

enum { OP_STOP = 0, OP_START, OP_GOTO, OP_ADOPT, OP_CLEAR, OP_DWELL, OP_CENSUS, OP_ENTER, OP_REBOOT,
       OP_RELOAD = 0x80 };

// Commands arrive from UART_ServiceCommands at the top of the loop, which can
// be mid-burst. Anything that touches the radio waits here for a quiet moment.
#define HQ_N 4
static uint8_t hq[HQ_N][3];
static uint8_t hqN;

static bool HostQueue(uint8_t op, uint8_t a, uint8_t b)
{
	if (hqN >= HQ_N)
		return false;
	hq[hqN][0] = op;
	hq[hqN][1] = a;
	hq[hqN][2] = b;
	hqN++;
	return true;
}

static uint16_t Le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

bool ALERT_HostArrSet(const uint8_t *payload, uint16_t len)
{
	Arr_t a;
	if (!payload || len < 21u || payload[0] < ARR_HOST0 || payload[0] >= ARR_HOST0 + ARR_HOST_N)
		return false;
	memset(&a, 0, sizeof(a));
	for (uint8_t i = 0; i < 3u && payload[1 + i]; i++) {
		const char c = (char)payload[1 + i];
		a.tag[i] = (c > ' ' && c < 0x7F) ? c : '?';   // a space would split the L line
	}
	if (!a.tag[0]) {
		a.tag[0] = 'H';
		a.tag[1] = (char)('0' + (payload[0] - ARR_HOST0));
	}
	a.r58   = Le16(payload + 4);
	a.r70   = Le16(payload + 6);
	a.t1hz  = Le16(payload + 8);
	a.t2hz  = Le16(payload + 10);
	a.r5c   = Le16(payload + 12);
	a.r59   = (uint16_t)(Le16(payload + 14) & 0x07FFu);
	a.s01   = Le16(payload + 16);
	a.s23   = Le16(payload + 18);
	a.flags = (uint8_t)(payload[20] & ARR_STREAM);
	if (a.t1hz > 3000u || a.t2hz > 3000u)
		return false;            // tone_reg overflows above ~3170 Hz
	hostArr[payload[0] - ARR_HOST0] = a;
	if (running && curIdx == payload[0])
		HostQueue(OP_RELOAD, 0, 0);
	return true;
}

bool ALERT_HostPokeList(const uint8_t *payload, uint16_t len)
{
	if (!payload || len < 1u)
		return false;
	const uint8_t n = payload[0];
	if (n > POKE_N || len < 1u + 5u * n)
		return false;
	// Nothing else in this app can key the transmitter, and these are re-applied
	// after every arm for the rest of the power-up, silently, with PTT ignored:
	// REG_30 (TX enable, PA gain), REG_33 (BK GPIO1 is the PA enable) and REG_36
	// (PA bias) are refused outright. A raw write (0x0602) is still there for
	// anyone who means it.
	for (uint8_t i = 0; i < n; i++) {
		const uint8_t reg = payload[1u + 5u * i];
		if (reg > 0x7Fu || reg == 0x30u || reg == 0x33u || reg == 0x36u)
			return false;
	}
	for (uint8_t i = 0; i < n; i++) {
		const uint8_t *e = payload + 1u + 5u * i;
		pokes[i].reg     = e[0];
		pokes[i].andMask = Le16(e + 1);
		pokes[i].orMask  = Le16(e + 3);
		if (e[0] == 0x59u) {
			// REG_59<11> is FSK TX enable: forced clear, as ARR_SET masks it
			pokes[i].andMask &= (uint16_t)~0x0800u;
			pokes[i].orMask  &= (uint16_t)~0x0800u;
		}
	}
	pokeN = n;
	if (running)
		HostQueue(OP_RELOAD, 0, 0);   // apply now, not at the next arrangement change
	return true;
}

bool ALERT_HostSweepCtl(const uint8_t *payload, uint16_t len)
{
	if (!payload || len < 3u)
		return false;
	const uint8_t op = payload[0], a = payload[1], b = payload[2];
	// Everything but ENTER and REBOOT acts on a running sweep. Outside the app
	// it would sit in the queue, be acked as done, and then run after the
	// census of some later entry, long after anyone expected it.
	if (!running && op != OP_ENTER && op != OP_REBOOT)
		return false;
	switch (op) {
		case OP_STOP: case OP_CLEAR: case OP_DWELL: case OP_CENSUS:
			break;
		case OP_START:
			if (a < PH_1 || a > PH_3)
				return false;
			break;
		case OP_GOTO:
			if (!ArrKnown(a, b))
				return false;
			break;
		case OP_ADOPT:
			if (a != ARR_NONE && a != ARR_ADC && !ArrKnown(a, b))
				return false;
			break;
		case OP_ENTER:
			// the service point in app.c starts it from the main screen
			if (!running)
				gRequestAlertApp = true;
			return true;
		case OP_REBOOT:
			if (!running) {
				// No burst to protect outside the app, so at once - but acked
				// first, since uart.c's ack would come after the reset. The tag
				// is the one uart.c uses for 0x0A02.
				DFU_EmitAck("a02", true);
				DFU_SafeReset();     // TX off first: PTT may be held right now
			}
			break;
		default:
			return false;
	}
	return HostQueue(op, a, b);
}

static void HostService(void)
{
	if (!hqN)
		return;
	const uint8_t op = hq[0][0], a = hq[0][1], b = hq[0][2];
	hqN--;
	memmove(hq[0], hq[1], (size_t)hqN * 3u);
	switch (op) {
		case OP_STOP:
			if (mode == MODE_SWEEP) mode = MODE_HOLD;
			break;
		case OP_START:  SweepStart(a); break;
		case OP_GOTO:   SetArr(a, b); break;
		case OP_ADOPT:
			if (a == ARR_NONE) {
				cfg.route = ROUTE_SWEEP;
				Persist();
				if (mode == MODE_PROD)
					SweepStart(PH_1);
			} else {
				Adopt(a, b, 0, 0);
			}
			break;
		case OP_CLEAR:
			memset(scores, 0, sizeof(scores));
			memset(reps, 0, sizeof(reps));
			anyStruct = false;
			break;
		case OP_DWELL:  dwellSet = a ? a : 1u; break;
		case OP_CENSUS: censusReq = true; break;
		case OP_REBOOT: DFU_SafeReset(); break;
		case OP_RELOAD: SetArr(curIdx, curVar); break;   // new recipe or pokes: arm again
		default: break;
	}
	redraw = true;
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

static void Announce(const History_t *h)
{
	BACKLIGHT_TurnOn();
#ifdef ENABLE_VOICE
	if (cfg.voice && gEeprom.VOICE_PROMPT != VOICE_PROMPT_OFF) {
		SpeakDigits(h->id, h->value);
		return;
	}
#endif
	(void)h;
}

static void PushHistory(const AlertReading_t *r, int16_t rssi, uint8_t kind, uint8_t seq)
{
	if (historyCount < HISTORY_N)
		historyCount++;
	for (uint8_t i = historyCount - 1; i > 0; i--)
		history[i] = history[i - 1];
	history[0].id       = r->id;
	history[0].value    = r->value;
	history[0].kind     = kind;
	history[0].format   = r->format;
	history[0].rssi_dbm = (int8_t)((rssi < -127) ? -127 : (rssi > 0 ? 0 : rssi));
	history[0].seq      = seq;
}

// What ProcessCapture did after its decoder ran: the decoding now happens in
// BurstFinish, once per route, and both routes' readings arrive here together
// so a station heard on both is shown and spoken once.
static void Deliver(const AlertReading_t *r, int n, int16_t rssi)
{
	if (n <= 0)
		return;

	burstSeq++;
	uint8_t announced = 0;
	for (int i = 0; i < n; i++) {
		// collapse repeats of the same reading inside one burst
		bool dup = false;
		for (int j = 0; j < i; j++)
			if (r[j].id == r[i].id && r[j].value == r[i].value) { dup = true; break; }
		if (dup)
			continue;
		if (cfg.confirm) {
			bool again = false;
			for (int j = i + 1; j < n; j++)
				if (r[j].id == r[i].id && r[j].value == r[i].value) { again = true; break; }
			if (!again)
				continue;
		}

		const char *name; uint8_t kind;
		const bool known = ALERT_LookupStation(r[i].id, &name, &kind);
		if (!known) {
			stats.unknown++;
			if (!cfg.show_unknown)
				continue;
		}
		stats.frames++;
		PushHistory(&r[i], rssi, kind, burstSeq);

#ifdef ENABLE_UART
		{
			char line[48];
			sprintf(line, "ALERT,%u,%u,%s,%d,%s\r\n", r[i].id, r[i].value,
			        r[i].format == ALERT_FMT_EIF ? "EIF" : "ABF", rssi, name);
			UART_Send(line, strlen(line));
#ifdef ENABLE_USB
			VCP_SendStr(line);
#endif
		}
#endif
		if (!announced++)
			Announce(&history[0]);
	}
	redraw = true;
}

// ---------------------------------------------------------------------------
// display

static void FormatValue(char *out, uint16_t value, uint8_t kind)
{
	if (value == ALERT_VALUE_FULL_SCALE) {
		strcpy(out, "FULL");
		return;
	}
	switch (kind) {
		case ALERT_KIND_BATT:   // tenths of a volt (MegaNet convention)
			sprintf(out, "%u.%uV", value / 10u, value % 10u);
			break;
		case ALERT_KIND_RAIN:   // tip count; 0.2 mm/tip assumed unless recorded
			sprintf(out, "%u", value);
			break;
		default:
			sprintf(out, "%u", value);
			break;
	}
}

static unsigned Cap(unsigned v, unsigned max)
{
	return v > max ? max : v;
}

static const char *AudName(void)
{
	static const char *const names[4] = { "NONE", "PA4", "PA4B", "PB1" };
	return names[ALERTADC_ChoicePin() & 3u];
}

// The small font is 6 px wide with 1 px of spacing, so a character costs 7 px
// and the 128 px line holds eighteen of them. Both halves of this line once
// ignored that and were drawn over each other. The columns below are picked
// from the widths, with the reading right-aligned into the space that is left.
#define ST_CHAR   7u
#define ST_COL_IN (6u * ST_CHAR)     // 42: clear of "ALERT"
#define ST_COL_SQ (16u * ST_CHAR)    // 112: last two cells
static void DrawStatus(void)
{
	char s[16];
	memset(gStatusLine, 0, sizeof(gStatusLine));

	UI_PrintStringSmallBufferNormal("ALERT", gStatusLine + 0);
	// the route in use: the arrangement tag, or ADC once that is adopted
	UI_PrintStringSmallBufferNormal((mode == MODE_PROD && cfg.route == ROUTE_ADC) ? "ADC" : cur.tag,
	                                gStatusLine + ST_COL_IN);

	sprintf(s, "%ddBm", rssiDbm);
	{	// right-align, but never start before the tag ends
		unsigned int col = (unsigned int)(ST_COL_SQ - strlen(s) * ST_CHAR);
		if (col < ST_COL_IN + 4u * ST_CHAR)
			col = ST_COL_IN + 4u * ST_CHAR;
		UI_PrintStringSmallBufferNormal(s, gStatusLine + col);
	}

	if (streaming)
		UI_PrintStringSmallBufferNormal("*", gStatusLine + ST_COL_SQ + ST_CHAR);
	else if (sqOpen)
		UI_PrintStringSmallBufferNormal("+", gStatusLine + ST_COL_SQ + ST_CHAR);
	ST7565_BlitStatusLine();
}

// The slot with the most table decodes, then the most decodes; SLOT_N if none.
static uint8_t BestSlot(void)
{
	uint8_t b = SLOT_N;
	for (uint8_t sl = 0; sl < SLOT_N; sl++) {
		const Score_t *s = &scores[sl];
		if (!s->dx)
			continue;
		if (b == SLOT_N || s->dt > scores[b].dt || (s->dt == scores[b].dt && s->dx > scores[b].dx))
			b = sl;
	}
	return b;
}

static void DrawMain(void)
{
	char s[32], st[16];

	UI_DisplayClear();

	if (historyCount == 0) {
		// Row 6 is deliberately left empty. On this radio the bottom row of the
		// glass sits partly under the bezel, so anything printed there cannot be
		// read. Everything that has to be legible lives in rows 0..5, and no line
		// here exceeds the eighteen characters the small font fits across 128 px.
		const uint32_t f = gRxVfo->pRX->Frequency;
		sprintf(s, "%u.%05u MHz", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
		UI_PrintStringSmallNormal(s, 0, 127, 0);

		// what the app is doing: at most 12 + 1 + 5 = 18
		if (censusBusy || entry == ENTRY_CENSUS) {
			strcpy(st, "CENSUS");
		} else if (entry == ENTRY_SETTLE) {
			const uint32_t el = nowMs - entryMs;
			sprintf(st, "SETTLING %u", (unsigned)(el >= SETTLE_MS ? 0u : (SETTLE_MS - el + 999u) / 1000u));
		} else if (mode == MODE_SWEEP) {
			sprintf(st, "SWEEP P%u Z%u", phase, Cap(pass, 99u));
		} else if (mode == MODE_PROD) {
			sprintf(st, "ADOPTED %s", cfg.route == ROUTE_ADC ? "ADC" : cur.tag);
		} else {
			sprintf(st, "HOLD %s", cur.tag);
		}
		sprintf(s, "%s %s", st, cfg.voice ? "VOICE" : "QUIET");
		UI_PrintStringSmallNormal(s, 0, 127, 1);

		// N qualifying bursts, F frames, P peak of the last burst, so it is
		// obvious whether a transmission is getting through at all: 17 at most
		sprintf(s, "N%u F%u P%d", Cap(burstCount, 9999u), Cap(stats.frames, 9999u), lastPeak);
		UI_PrintStringSmallNormal(s, 0, 0, 2);

		// the arrangement armed now: "#255 U39 3000 v77" is 17
		sprintf(s, "#%u %s %u v%02X", curIdx, cur.tag, cur.t2hz, curVar);
		UI_PrintStringSmallNormal(s, 0, 0, 3);

		{	// Whatever has decoded best, left on the glass so it can be read back
			// hours later without anyone having captured the serial stream; until
			// then the last window's structure: runs >= 0.75k, transitions per
			// sample (both %), and the leading run.
			const uint8_t b = BestSlot();
			if (b < SLOT_N) {
				Arr_t a;
				const char *tag = "ADC";
				if (b == SLOT_TMP)
					tag = cur.tag;
				else if (b != SLOT_ADC && ArrGet(SlotIdx(b), 0, &a))
					tag = a.tag;
				sprintf(s, "BEST %s %u/%u", tag, scores[b].dt, scores[b].bq);   // 16
			} else {
				sprintf(s, "RK%u TR%u L%u", lastRk / 10u, lastTr / 10u, Cap(lastLead, 9999u));   // 17
			}
			UI_PrintStringSmallNormal(s, 0, 0, 4);
		}

		{	// the census result and what became of it: "AUD PA4B+ REJ" is 13
			static const char *const states[5] = { "?", "NO", "CHK", "ON", "REJ" };
			sprintf(s, "AUD %s%s %s", AudName(), ALERTADC_ChoicePa8() ? "+" : "", states[adcState]);
			UI_PrintStringSmallNormal(s, 0, 0, 5);
		}
	} else {
		const History_t *h = &history[0];
		const char *name; uint8_t kind;
		ALERT_LookupStation(h->id, &name, &kind);

		// line 0: station name (or unknown), line 1-2: value big, right: kind/id
		if (name[0]) {
			UI_PrintStringSmallNormal(name, 0, 0, 0);
		} else {
			sprintf(s, "ID %u UNKNOWN", h->id);   // "NOT IN TABLE" ran to 20
			UI_PrintStringSmallNormal(s, 0, 0, 0);
		}
		FormatValue(s, h->value, h->kind);
		UI_PrintString(s, 0, 0, 1, 8);
		sprintf(s, "%s", ALERT_KindLabel(h->kind));
		UI_PrintStringSmallNormal(s, 88, 0, 1);
		sprintf(s, "#%u", h->id);
		UI_PrintStringSmallNormal(s, 88, 0, 2);

		// rows 3..5: history. Row 6 is under the bezel, see above.
		for (uint8_t i = 0; i < 3 && i < historyCount; i++) {
			const History_t *e = &history[i];
			const char *n2; uint8_t k2; char v[16];
			ALERT_LookupStation(e->id, &n2, &k2);
			FormatValue(v, e->value, e->kind);
			// 4 + 1 + 7 + 1 + 5 = 18, exactly what fits. It was 21.
			sprintf(s, "%4u %-7.7s %5s", e->id, n2[0] ? n2 : "?", v);
			UI_PrintStringSmallNormal(s, 0, 0, 3 + i);
		}
	}
	ST7565_BlitFullScreen();
}

// settings rows
enum {
	SET_POLARITY, SET_VOICE, SET_GATE, SET_UNKNOWN, SET_CONFIRM, SET_MONITOR,
	SET_MODE, SET_SWEEP, SET_ADOPT, SET_BITREV, SET_CENSUS, SET_FREQ, SET_SQL, SET_N
};

// "MDM MODE" and "SQ GATE" are grepped for by CI as proof the app is in the
// image, so they keep their names whatever the rows now do.
static const char *const setNames[SET_N] = {
	"POLARITY", "VOICE", "SQ GATE", "UNKNOWN", "CONFIRM", "MONITOR",
	"MDM MODE", "SWEEP", "ADOPT", "BIT REV", "CENSUS",
	// in enum order: these two were once swapped, so FREQ stepped the squelch
	"FREQ MHz", "SQL LEVEL"
};

// values are at most eight characters, see DrawSettings
static void SetValueString(uint8_t idx, char *s)
{
	static const char *const onoff[2] = { "OFF", "ON" };
	static const char *const pol[3]   = { "NEG", "STD", "ANY" };
	switch (idx) {
		case SET_POLARITY: strcpy(s, pol[cfg.polarity]); break;
		case SET_VOICE:    strcpy(s, onoff[cfg.voice]); break;
		case SET_GATE:     strcpy(s, onoff[cfg.gate]); break;
		case SET_UNKNOWN:  strcpy(s, cfg.show_unknown ? "SHOW" : "HIDE"); break;
		case SET_CONFIRM:  strcpy(s, cfg.confirm ? "2 COPIES" : "OFF"); break;
		case SET_MONITOR:  strcpy(s, onoff[cfg.monitor]); break;
		case SET_MODE:     sprintf(s, "%s v%02X", cur.tag, curVar); break;
		case SET_SWEEP:
			if (mode == MODE_SWEEP) sprintf(s, "ON P%u", phase); else strcpy(s, "OFF");
			break;
		case SET_ADOPT: {
			Arr_t a;
			if (cfg.route == ROUTE_ADC)
				strcpy(s, "ADC");
			else if (cfg.route == ROUTE_FSK && ArrGet(cfg.adopted, cfg.variant, &a))
				sprintf(s, "%s v%02X", a.tag, cfg.variant);
			else
				strcpy(s, "NONE");
			break;
		}
		case SET_BITREV:   strcpy(s, onoff[cfg.bitrev]); break;
		case SET_CENSUS:
			if (censusReq)                      strcpy(s, "PENDING");
			else if (adcState == ADCST_UNTESTED) strcpy(s, "NOT RUN");
			else sprintf(s, "%s%s", AudName(), ALERTADC_ChoicePa8() ? "+" : "");
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
	// BK4819_SetupSquelch zeroes REG_70 (our bit clock) and mutes AF
	ArrArm();
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

static void ChangeSetting(uint8_t idx, int dir)
{
	switch (idx) {
		case SET_POLARITY: cfg.polarity = (uint8_t)((cfg.polarity + 3 + dir) % 3); break;
		case SET_VOICE:    cfg.voice = !cfg.voice; break;
		case SET_GATE:     cfg.gate = !cfg.gate; break;
		case SET_UNKNOWN:  cfg.show_unknown = !cfg.show_unknown; break;
		case SET_CONFIRM:  cfg.confirm = !cfg.confirm; break;
		case SET_MONITOR:
			// only the audio path moves: the AF selector stays at FM so the FSK
			// engine keeps its signal whether or not the speaker is live
			cfg.monitor = !cfg.monitor;
			AudioPath();
			break;
		case SET_MODE: {
			// by hand through the Phase 1 table, holding wherever it lands
			int i = (curIdx < ARR_P1_N) ? curIdx : 0;
			i = (i + ARR_P1_N + dir) % ARR_P1_N;
			mode = MODE_HOLD;
			SetArr((uint8_t)i, 0);
			break;
		}
		case SET_SWEEP:
			if (mode == MODE_SWEEP) mode = MODE_HOLD;
			else                    SweepStart(PH_1);
			break;
		case SET_ADOPT:
			if (cfg.route != ROUTE_SWEEP) {
				// forget it; persisted on the way out of the menu
				cfg.route = ROUTE_SWEEP;
				if (mode == MODE_PROD)
					SweepStart(PH_1);
			} else {
				Adopt(curIdx, curVar, 0, 0);
			}
			break;
		case SET_BITREV:   cfg.bitrev = !cfg.bitrev; break;
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
			ArrArm();
			break;
		}
		case SET_SQL:      StepSquelch(dir); break;
		default: break;
	}
}

static void DrawSettings(void)
{
	char s[24], v[16];
	UI_DisplayClear();
	UI_PrintStringSmallNormal("ALERT SETTINGS", 0, 127, 0);
	// A window of five rows, not six: row 6 is under the bezel on this radio
	// and the selected row could land there and be invisible.
	uint8_t first = (setIndex >= 4) ? (uint8_t)(setIndex - 4) : 0;
	if (first > SET_N - 5) first = SET_N - 5;
	for (uint8_t i = 0; i < 5; i++) {
		const uint8_t idx = (uint8_t)(first + i);
		SetValueString(idx, v);
		// 1 + 9 + 8 = 18: the longest name ("SQL LEVEL") and the longest value
		// ("2 COPIES", "PENDING", "A10 v77") together fill the row.
		sprintf(s, "%c%-9s%s", idx == setIndex ? '>' : ' ', setNames[idx], v);
		UI_PrintStringSmallNormal(s, 0, 0, 1 + i);
	}
	ST7565_BlitFullScreen();
}

// Six usable rows, eighteen characters each.
static void DrawRaw(void)
{
	char s[28];
	UI_DisplayClear();

	sprintf(s, "RAW #%u %s v%02X", curIdx, cur.tag, curVar);   // 16 at most
	UI_PrintStringSmallNormal(s, 0, 0, 0);

	sprintf(s, "S%u F%u G%u", Cap(stats.syncs, 9999u), Cap(stats.frames, 9999u), Cap(stats.gated, 9999u));
	UI_PrintStringSmallNormal(s, 0, 0, 1);

	sprintf(s, "LAST %u BITS", lastCapLen);
	UI_PrintStringSmallNormal(s, 0, 0, 2);

	sprintf(s, "%02X%02X%02X%02X %02X%02X%02X%02X", lastCap[0], lastCap[1], lastCap[2],
	        lastCap[3], lastCap[4], lastCap[5], lastCap[6], lastCap[7]);
	UI_PrintStringSmallNormal(s, 0, 0, 3);

	{	// the first eighteen samples as 0/1, so the preamble and start bits can
		// be eyeballed. Eighteen because that is what fits across 128 px.
		uint8_t i;
		for (i = 0; i < 18; i++)
			s[i] = ALERT_GetBit(lastCap, i) ? '1' : '0';
		s[i] = 0;
		UI_PrintStringSmallNormal(s, 0, 0, 4);
	}

	sprintf(s, "%s %uHZ SQL%u.%u", cfg.route == ROUTE_ADC ? "ADC" : "MDM", cur.t2hz,
	        gEeprom.SQUELCH_LEVEL, gEeprom.SQUELCH_TENTHS);
	UI_PrintStringSmallNormal(s, 0, 0, 5);
	ST7565_BlitFullScreen();
}

static void Draw(void)
{
	// Spectrum renders at a fixed rate and blits a single line per pass rather
	// than pushing the whole 1 KB framebuffer in one burst. Ours hammered
	// BlitFullScreen from a 1 ms loop, which is the one thing this app does to
	// the LCD that no working code in this fork does.
	DrawStatus();
	switch (view) {
		case VIEW_SETTINGS: DrawSettings(); break;
		case VIEW_RAW:      DrawRaw(); break;
		default:            DrawMain(); break;
	}
	redraw = false;
}

// ---------------------------------------------------------------------------
// keys

static void OnKey(KEY_Code_t key)
{
	BACKLIGHT_TurnOn();
	redraw = true;

	if (view == VIEW_SETTINGS) {
		switch (key) {
			case KEY_UP:   ChangeSetting(setIndex, +1); break;
			case KEY_DOWN: ChangeSetting(setIndex, -1); break;
			case KEY_MENU: setIndex = (uint8_t)((setIndex + 1) % SET_N); break;
			case KEY_STAR: setIndex = (uint8_t)((setIndex + SET_N - 1) % SET_N); break;
			case KEY_EXIT:
				Persist();
				view = VIEW_MAIN;
				break;
			default: break;
		}
		return;
	}

	switch (key) {
		case KEY_EXIT:
			if (view == VIEW_RAW) view = VIEW_MAIN;
			else running = false;
			break;
		case KEY_MENU: view = VIEW_SETTINGS; setIndex = 0; break;
		case KEY_1:    view = (view == VIEW_RAW) ? VIEW_MAIN : VIEW_RAW; break;
		case KEY_STAR: cfg.voice = !cfg.voice; ALERT_StoreConfig(); break;
		case KEY_F:    ChangeSetting(SET_MONITOR, 0); break;
		case KEY_UP:   StepSquelch(+1); break;
		case KEY_DOWN: StepSquelch(-1); break;
		case KEY_0:
			historyCount = 0;
			memset(&stats, 0, sizeof(stats));
			break;
		case KEY_5:
			if (historyCount) Announce(&history[0]);
			break;
		default: break;
	}
}

// ---------------------------------------------------------------------------
// main loop

void APP_RunAlert(void)
{
	KEY_Code_t lastKey     = KEY_INVALID;
	uint16_t   keyHeld10ms = 0;
	uint16_t   pttHeld10ms = 0;

	ALERT_LoadConfig();
	// An adopted recipe that is not there any more - a host slot lives in RAM
	// only - or an ADC adoption with no pin goes back to sweeping.
	if (cfg.route == ROUTE_FSK && !ArrKnown(cfg.adopted, cfg.variant))
		cfg.route = ROUTE_SWEEP;
	if (cfg.route == ROUTE_ADC && !cfg.adcPin)
		cfg.route = ROUTE_SWEEP;

	// These are statics, so without this they carry over from the last run.
	running = true;
	view    = VIEW_MAIN;
	redraw  = true;
	memset(&stats, 0, sizeof(stats));
	memset(scores, 0, sizeof(scores));
	memset(reps, 0, sizeof(reps));
	historyCount = 0;
	dbgIrqCount  = 0;
	// burstCount is not reset: with the boot line it keys every C/H/X line, and
	// a second visit to the app in one power-up must not reuse burst numbers.
	sqOpen = false; sqPrev = false; draining = false;
	rssiFloor = 0; burstPeak = -127; lastPeak = -127;
	lastCapLen = 0; lastRk = 0; lastTr = 0; lastLead = 0;
	memset(lastCap, 0, sizeof(lastCap));
	streaming = false; syncSeen = false; capLen = 0; openLen = 0; validLen = 0;
	yqN = 0; fPend = false;
	nowMs = 0; tick = 0;
	entry = ENTRY_SETTLE; entryMs = 0; censusBusy = false;
	mode = MODE_HOLD; phase = PH_1; step = 0; pass = 0; p1Passes = 0;
	p2Done = false; p3Done = false; anyStruct = false;
	p2Base[0] = ARR_NONE; p2Base[1] = ARR_NONE;
	dwellQ = 0; dwellPre = 0; prodFail = 0;
	adcState = ADCST_UNTESTED; adcRunning = false; confN = 0; confOk = 0;
	curIdx = ARR_NONE; curVar = 0; curSlot = 0;
	hqN = 0; blocked = false;

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

	// Armed from the start so the radio is in receive while the floor settles;
	// bursts during the settle are seen but not judged.
	if (cfg.route == ROUTE_FSK) SetArr(cfg.adopted, cfg.variant);
	else                        SetArr(0, 0);

	while (running) {
		DFU_WatchdogKick();

		// keys (edge triggered)
		const KEY_Code_t key = KEYBOARD_GetKey();
#if defined(ENABLE_UART) || defined(ENABLE_USB)
		// Spectrum does this every pass. Without it the radio stops answering
		// on USB for as long as this app is open.
		UART_ServiceCommands();
#endif
		dbgRawKey = (int16_t)key;
		dbgRawPtt = GPIO_IsPttPressed() ? 1u : 0u;
		if (key != lastKey) {
			if (key != KEY_INVALID && key != KEY_PTT)
				OnKey(key);
			lastKey     = key;
			keyHeld10ms = 0;   // a new key starts its own hold timer
		}

		ArrPoll();

		// Anything that blocks - the census, host commands that re-arm - waits
		// for the squelch to shut, so the eight-word FIFO is never left unread
		// while a burst is on the air.
		if (entry == ENTRY_SETTLE && (uint32_t)(nowMs - entryMs) >= SETTLE_MS) {
			entry  = ENTRY_CENSUS;
			redraw = true;
		}
		if (Quiet()) {
			if (entry == ENTRY_CENSUS) {
				RunCensus();
				StartRoute();
				entry = ENTRY_RUN;
			} else if (entry == ENTRY_RUN) {
				if (censusReq) RunCensus();
				else           HostService();
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
				BK4819_SetAF(BK4819_AF_FM);   // the engine needs it back regardless
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

			if (quiet)
				EmitPending();        // syncs on noise, between bursts
			if ((tick % 10) == 0 && quiet)
				DrawStatus();
			if ((tick % 50) == 0 && quiet) {
				// The ST7565 loses its register state when the BK4819 changes RF
				// state; this fork re-sends the init list after TX and after
				// sleep-wake for exactly that reason.
				ST7565_FixInterfGlitch();
				redraw = true;
				// The arrangement goes out with every heartbeat, so any line can
				// be attributed to the settings that produced it.
				sprintf(lb, "D I%u S%u F%u G%u X%u B%u R%d Q%u N%u f%d p%d V%u C%u a%u z%u\r\n",
				        dbgIrqCount, stats.syncs, stats.frames, stats.gated, stats.stuck,
				        lastCapLen, rssiDbm, sqOpen ? 1u : 0u, burstCount, rssiFloor, lastPeak,
				        stats.inv, stats.sqcap, curIdx, pass);
				DbgSend(lb);
			}
		}

		// Never during a burst: a full-screen blit is long enough to let the
		// FIFO overflow at the higher TONE2 rates.
		if (redraw && Quiet())
			Draw();
	}

	// leave: modem/ADC off, radio back to normal
	if (adcRunning) {
		(void)ALERTADC_BurstStop(adcBits, 0);
		adcRunning = false;
	}
	ALERTADC_Shutdown();
	ModemStop();
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
