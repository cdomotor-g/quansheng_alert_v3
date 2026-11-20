/* ALERT receiver: every screen - the status line, the readings in four
 * layouts, the detail view, the settings view and the one-line notes that
 * confirm a key (tools/alert/V2_SPEC.md section 3). It reads the receiver
 * through alert_int.h and changes it only through ALERT_SetStep, ALERT_SetTo
 * and ALERT_SettingsChanged.
 *
 * The glass: a 128 px status line plus main rows 0..5. Row 6 sits partly
 * under the bezel on this radio and is never used. The small font is a 6 px
 * glyph plus 1 px of spacing, so a character costs 7 px and a row holds 18;
 * eighteen characters end at px 125, which leaves the last column for the
 * scroll bar. Numbers can also be drawn in the radio's big frequency digits,
 * 16 px tall, or those at twice the size, 32 px.
 *
 * Every main-area writer is one of Put(), Digits(), Digits2(), Big() and
 * Invert(), which refuse a row that would reach row 6 at run time, and all
 * status text goes through Status(), which clips each field to the width its
 * column was planned for. tools/alert/check_layout.py checks those rules and
 * the status columns exactly, and tools/alert/screens.py draws every screen
 * on a computer with this very file and checks row 6 stays dark.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <string.h>

#include "app/alert_adc.h"
#include "app/alert_decode.h"
#include "app/alert_int.h"
#include "app/alert_log.h"
#include "app/alert_stn.h"
#include "bitmaps.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "font.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/battery.h"
#include "ui/helper.h"

#define CHAR_PX     7u               // 6 px glyph + 1 px spacing
#define UI_COLS     18u               // characters across a row
#define UI_ROWS     6u                // main rows 0..5: row 6 is under the bezel
#define TEXT_PX     (UI_COLS * CHAR_PX)   // 126: the text columns, px 0..125
#define BAR_END     (TEXT_PX + 1u)    // a dark bar runs to px 126: a margin after the last letter
#define BAR_X       (LCD_WIDTH - 1u)  // scroll bar column, clear of every row's text and bar
#define BAR_PX      (UI_ROWS * 8u)    // the bar spans rows 0..5: 48 px
#define PAGE_MAX    6u                // the most entries a layout shows (LIST)
#define NAME_W      13u               // CLASSIC's name field; the age takes cols 14..17
#define LIST_NAME_W 9u                // LIST's: then value cols 10..13, age cols 14..17
#define SCAN_MAX    128u              // records read per search past hidden ones (UNKNOWN=HIDE)
#define IDLE_TICKS  3000u             // 30 s of 10 ms ticks without a key: back to the newest
#define MQ_STEP     30u               // marquee: one character every 300 ms
#define MQ_PAUSE    100u              // ... and a 1 s rest at each end
#define NOTE_TICKS  150u              // a note stays 1.5 s
#define SET_ROWS    5u                // settings rows on the glass, under the title
#define DIGIT_PX    13u               // gFontBigDigits as UI_DisplayFrequency spaces them
#define DOT_PX      3u
#define DIGIT2_PX   24u               // twice the size: 20 px a digit, 4 px between
#define DOT2_PX     8u

static uint8_t  view;             // ALERT_VIEW_*
static uint8_t  setIndex;         // selected settings row (an ALERT_Set* row number)
static bool     editing;          // ... and its value is being changed
static char     editWas[12];      // its value when MENU started the change, for EXIT
static char     typed[12];        // a value being typed: the digits so far, '_' for the rest
static uint8_t  typedN;           // digits typed so far; 0 = not typing
static bool     srcLog;           // the list reads the SPI log, else the RAM ring
static uint32_t top;              // back index the list starts at, 0 = newest
static uint16_t newN;             // decodes that arrived while the list was held
static uint32_t lastDecodes;      // ALERT_Decodes() when last looked at
static uint16_t idle;             // 10 ms ticks since the last UI key
static char     note[17];         // what a key just did: 16 big letters at most
static uint8_t  noteTicks;

// marquee state, for the selected entry's name only
static bool     mqLive;           // the frame just drawn has a name that scrolls
static uint16_t mqId;
static uint8_t  mqLen, mqField, mqPos, mqTick;

// Names of what is on the glass. The marquee redraws every 300 ms and the
// status tick every 500, and an SPI-flash station table costs a binary search
// of flash reads per lookup: each name is looked up once while it is visible.
typedef struct {
	uint16_t id;
	uint8_t  kind;                    // ALERT_KIND_*, NONE when not in the table
	bool     known;
	bool     valid;
	char     name[ALERT_NAME_MAX + 1];
} NameSlot_t;
static NameSlot_t names[PAGE_MAX];
static uint16_t   namesCount;         // ALERTSTN_Count() they were looked up against

// Kinds: four characters where room is short, spelt out where it is not.
// Index 0 (and 7, which no table uses) is an address the table does not have.
static const char kindShort[8][5] = { "-", "RAIN", "LVL", "BATT", "REP", "OTH", "CHK", "-" };
static const char kindLong[8][9]  = { "UNKNOWN", "RAIN", "LEVEL", "BATTERY", "REPEATER",
                                      "OTHER", "CHECK", "UNKNOWN" };
static const char fmtName[4][4]   = { "?", "ABF", "EIF", "A2C" };   // ALERT_FMT_*

// ---------------------------------------------------------------------------
// drawing primitives

// Every line of small text, at a pixel column. The row check is here, at run
// time, because most rows are computed; the right edge is UI_PrintStringSmall's,
// which clips to what fits, so no string the data makes can pass the glass.
static void Put(const char *s, uint8_t x, uint8_t row, bool bold)
{
	if (row < UI_ROWS) {
		if (bold)
			UI_PrintStringSmallBold(s, x, 0, row);
		else
			UI_PrintStringSmallNormal(s, x, 0, row);
	}
}

static void Text(const char *s, uint8_t col, uint8_t row)
{
	Put(s, (uint8_t)(col * CHAR_PX), row, false);
}

static void TextRight(const char *s, uint8_t row)
{
	const size_t n = strlen(s);
	Text(s, (uint8_t)(n < UI_COLS ? UI_COLS - n : 0u), row);
}

static void TextCentre(const char *s, uint8_t row)
{
	const size_t n = strlen(s);
	Text(s, (uint8_t)(n < UI_COLS ? (UI_COLS - n) / 2u : 0u), row);
}

// Dark where it was light and the other way round, from x0 up to x1 on one
// row: what was written there becomes light letters in a dark bar. A letter
// lights px 1..6 of its 7, so a bar from px 0 to BAR_END leaves one dark
// pixel either side of the text. Never further: the scroll bar's column stays
// its own.
static void Invert(uint8_t row, uint8_t x0, uint8_t x1)
{
	if (row >= UI_ROWS)
		return;
	if (x1 > BAR_END)
		x1 = BAR_END;
	for (; x0 < x1; x0++)
		gFrameBuffer[row][x0] ^= 0xFFu;
}

// A one-pixel line along the bottom of a row, across the text columns: the
// small font and the big digits both leave that pixel row empty.
static void Rule(uint8_t row)
{
	if (row < UI_ROWS)
		for (uint8_t x = 0; x < BAR_END; x++)
			gFrameBuffer[row][x] |= 0x80u;
}

// The width in px of a number in the big digits: per digit (or '-'), per '.'.
static uint8_t DigitsWidth(const char *s, uint8_t digit, uint8_t dot)
{
	uint8_t w = 0;
	for (; *s; s++)
		w = (uint8_t)(w + (*s == '.' ? dot : digit));
	return w;
}

// A number in the radio's frequency digits, 16 px: this row and the next.
// UI_DisplayFrequency checks no edge at all, so this does: the whole number
// must fit, or nothing is drawn.
static void Digits(const char *s, uint8_t x, uint8_t row)
{
	if (row + 1u < UI_ROWS && x + DigitsWidth(s, DIGIT_PX, DOT_PX) <= LCD_WIDTH)
		UI_DisplayFrequency(s, x, row, false);
}

// The same digits at twice the size, 32 px: rows row..row+3. Each column of
// the glyph becomes two and each pixel two, so 0-9, '-' and '.' look exactly
// like the frequency on the radio's main screen, only bigger.
static void Digits2(const char *s, uint8_t x, uint8_t row)
{
	if (row + 3u >= UI_ROWS || x + DigitsWidth(s, DIGIT2_PX, DOT2_PX) > LCD_WIDTH + 4u)
		return;                       // the last digit's gap may hang off the edge
	for (; *s; s++) {
		if (*s == '.') {
			for (uint8_t i = 0; i < 6u; i++)
				gFrameBuffer[row + 3u][x + i] |= 0x3Cu;   // the 1x dot's 3x2, doubled, on the baseline
			x = (uint8_t)(x + DOT2_PX);
			continue;
		}
		if ((*s >= '0' && *s <= '9') || *s == '-') {
			const uint8_t *g = gFontBigDigits[*s == '-' ? 10 : *s - '0'];
			for (uint8_t c = 0; c < 10u; c++) {
				const uint16_t col  = (uint16_t)(g[c] | (g[10u + c] << 8));
				uint32_t       wide = 0;
				for (uint8_t b = 0; b < 16u; b++)
					if (col & (1u << b))
						wide |= 3uL << (2u * b);
				for (uint8_t r = 0; r < 4u; r++) {
					gFrameBuffer[row + r][x + 2u * c]      |= (uint8_t)(wide >> (8u * r));
					gFrameBuffer[row + r][x + 2u * c + 1u] |= (uint8_t)(wide >> (8u * r));
				}
			}
		}
		x = (uint8_t)(x + DIGIT2_PX);
	}
}

// Letters in the big font, 16 px: this row and the next. Clipped to the row.
static void Big(const char *s, uint8_t x, uint8_t row)
{
	if (row + 1u < UI_ROWS)
		UI_PrintString(s, x, 0, row, 8);
}

// One pixel wide down the last column of rows 0..5. The thumb is the share
// of the whole that one entry is, but never under 3 px so it stays visible
// with thousands of log records behind it.
static void DrawScrollBar(uint32_t pos, uint32_t count)
{
	uint32_t len, y;

	if (count < 2u)
		return;
	if (pos >= count)
		pos = count - 1u;
	len = BAR_PX / count;
	if (len < 3u)
		len = 3u;
	y = (BAR_PX - len) * pos / (count - 1u);
	for (len += y; y < len; y++)
		gFrameBuffer[y >> 3][BAR_X] |= (uint8_t)(1u << (y & 7u));
}

// ---------------------------------------------------------------------------
// formatting

// A reading as the screens show it. BATT is in tenths of a volt (MegaNet
// convention), everything else the raw count. Where there are four characters
// for it the unit goes (the BATT label says it) and a three-digit voltage
// loses its tenths; the detail view has room for both.
static void FormatValue(char *out, uint16_t value, uint8_t kind, bool full)
{
	if (value == ALERT_VALUE_FULL_SCALE)
		strcpy(out, "FULL");
	else if (kind != ALERT_KIND_BATT)
		sprintf(out, "%u", value);
	else if (full)
		sprintf(out, "%u.%uV", value / 10u, value % 10u);
	else if (value < 1000u)
		sprintf(out, "%u.%u", value / 10u, value % 10u);
	else
		sprintf(out, "%u", value / 10u);
}

// 12s, 4m, 3h, 2d: at most four characters. From the UTC clock when the
// record and the radio both have it, else from uptime - which only compares
// within one boot, so an earlier boot's record without a timestamp has no
// age ("-", and false). The RAM ring holds this boot's records only (it is
// emptied at app entry), whatever boot number they carry: the log can go to
// ERR, or be formatted, and renumber the boot under them.
static bool Age(char *out, const AlertRecord_t *r)
{
	static const uint32_t unit[4] = { 1u, 60u, 3600u, 86400u };
	const uint32_t now = ALERT_Epoch();
	uint32_t s;
	uint8_t  i = 0;

	if (r->epoch && now >= r->epoch) {
		s = now - r->epoch;
	} else if (!srcLog || r->boot == ALERTLOG_Boot()) {
		s = (ALERT_UptimeMs() - r->uptime_ms) / 1000u;
	} else {
		strcpy(out, "-");
		return false;
	}
	while (i < 3u && s >= unit[i + 1u])
		i++;
	s /= unit[i];
	sprintf(out, "%u%c", (unsigned)(s > 999u ? 999u : s), "smhd"[i]);
	return true;
}

// "12s AGO", or just "12s" when it would pass max characters. Returns the length.
static uint8_t AgeAgo(char *out, const AlertRecord_t *r, uint8_t max)
{
	uint8_t n;

	if (!Age(out, r))
		return 1u;
	n = (uint8_t)strlen(out);
	if (n + 4u <= max) {
		strcpy(out + n, " AGO");
		n = (uint8_t)(n + 4u);
	}
	return n;
}

// ---------------------------------------------------------------------------
// station names

// Keep only the slots the coming frame asks for, so a miss always finds a
// free slot and never evicts a name it is about to draw. A table uploaded or
// cleared since changes the count, and then nothing cached is kept.
static void NamesKeep(const AlertRecord_t *e, uint8_t count)
{
	const uint16_t stations = ALERTSTN_Count();

	for (uint8_t i = 0; i < PAGE_MAX; i++) {
		bool keep = false;
		for (uint8_t j = 0; j < count; j++)
			keep |= names[i].id == e[j].id;
		if (!keep || stations != namesCount)
			names[i].valid = false;
	}
	namesCount = stations;
}

static const NameSlot_t *Name(uint16_t id)
{
	NameSlot_t *n = &names[0];

	for (uint8_t i = 0; i < PAGE_MAX; i++) {
		if (names[i].valid && names[i].id == id)
			return &names[i];
		if (!names[i].valid)
			n = &names[i];
	}
	n->known = ALERTSTN_Lookup(id, n->name, sizeof(n->name), &n->kind);
	n->id    = id;
	n->valid = true;
	return n;
}

// The name field: its first w characters, or for the selected entry the
// window of a longer name the marquee has reached (see ALERTUI_Tick10ms).
// An address the table does not have is "ID 4142" and the ending given.
// Bold for a name that goes in a bar: light one-pixel strokes in a dark bar
// read thinner than dark ones on the light glass.
static void DrawName(const NameSlot_t *n, uint8_t w, uint8_t row, bool sel, bool bold,
                     const char *unknown)
{
	char    s[UI_COLS + 1];
	uint8_t off = 0;

	if (!n->known) {
		snprintf(s, (size_t)w + 1u, "ID %u%s", n->id, unknown);
		Put(s, 0, row, bold);
		return;
	}
	if (sel) {
		const uint8_t len = (uint8_t)strlen(n->name);
		if (n->id != mqId || w != mqField || len != mqLen) {
			mqId    = n->id;
			mqField = w;
			mqLen   = len;
			mqPos   = 0;
			mqTick  = 0;
		}
		mqLive = true;
		if (len > w)
			off = mqPos;
	}
	snprintf(s, (size_t)w + 1u, "%s", n->name + off);
	Put(s, 0, row, bold);
}

// ---------------------------------------------------------------------------
// the history the list shows

// Logging: LOG on and the region usable. That lights the header's L.
static bool Logging(void)
{
	return gAlertCfg.log && strcmp(ALERTLOG_State(), "OK") == 0;
}

// The list reads the log while logging (thousands back), else the RAM ring
// (16) - also while the log is still empty, which is what alert.c's KEY_0
// replay takes as "not usable" too. With LOG off nothing new is appended,
// so an old log would miss every decode since.
static bool UseLog(void)
{
	return Logging() && ALERTLOG_Count();
}

static void SnapTop(void)
{
	top  = 0;
	newN = 0;
}

// A switch between log and ring renumbers every entry: start from the top.
static void Source(void)
{
	const bool log = UseLog();
	if (log != srcLog) {
		srcLog = log;
		SnapTop();
	}
}

static bool Shown(const AlertRecord_t *r)
{
	return gAlertCfg.show_unknown || (r->flags & ALERTREC_TABLE);
}

// Decodes since the last look. While the list is held - scrolled, or an entry
// open in the detail view - what is on the glass keeps its place: every back
// index moves on by the number that arrived. The header's +N counts those the
// list will show (UNKNOWN=HIDE only filters the display; alert.c records every
// frame), judged from the RAM ring: it holds the same newest records as the
// log and costs nothing to read here, squelch open or not.
static void Sync(void)
{
	const uint32_t d = ALERT_Decodes() - lastDecodes;
	uint32_t shown = 0;

	if (!d)
		return;
	lastDecodes += d;
	if (!top && view != ALERT_VIEW_DETAIL)
		return;
	top += d;
	for (uint32_t i = 0; i < d; i++) {
		const AlertRecord_t *h = i < 256u ? ALERT_History((uint8_t)i) : NULL;
		if (!h || Shown(h))
			shown++;                  // gone from the ring already: count it
	}
	newN = (uint16_t)((newN + shown > 99u) ? 99u : newN + shown);   // "+99" fills the field
}

static uint32_t Total(void)
{
	return srcLog ? ALERTLOG_Count() : ALERT_HistoryCount();
}

static bool Fetch(uint32_t back, AlertRecord_t *r)
{
	const AlertRecord_t *h;

	if (srcLog) {
		uint32_t seq;
		return ALERTLOG_ReadBack(back, r, &seq);
	}
	h = back < 256u ? ALERT_History((uint8_t)back) : NULL;
	if (!h)
		return false;
	*r = *h;
	return true;
}

// From *back in direction dir (+1 older, -1 newer), the first record the list
// shows: every one, or with UNKNOWN=HIDE those that were in the station table
// when decoded (the flag, not a lookup: a lookup per skipped record would be
// a flash search each). Bounded, so a long hidden run in the log costs at
// most SCAN_MAX record reads rather than thousands; past that the list ends
// there.
static bool FindVisible(uint32_t *back, int8_t dir, AlertRecord_t *r)
{
	uint32_t b = *back;

	for (uint8_t n = 0; n < SCAN_MAX; n++) {
		if (!Fetch(b, r))
			return false;
		if (Shown(r)) {
			*back = b;
			return true;
		}
		if (dir < 0 && !b)
			return false;
		b = dir < 0 ? b - 1u : b + 1u;
	}
	return false;
}

// The selected entry is the list's top one: the first shown record at or
// after `top`. False when there is nothing to show.
static bool Select(uint32_t *back, AlertRecord_t *r)
{
	const uint32_t total = Total();

	if (!total)
		return false;
	if (top >= total)
		top = total - 1u;             // the RAM ring dropped what the list was on
	*back = top;
	if (FindVisible(back, +1, r))
		return true;
	if (!top)
		return false;
	// only hidden records from there on: start again from the newest
	SnapTop();
	*back = 0;
	return FindVisible(back, +1, r);
}

// UP (-1) towards the newest, DOWN (+1) towards the oldest, one entry.
static void Step(int8_t dir)
{
	AlertRecord_t r;
	uint32_t      b;

	if (!Select(&b, &r))
		return;
	if (dir < 0) {
		if (!b)
			return;                   // already the newest
		b--;
	} else {
		b++;
	}
	if (FindVisible(&b, dir, &r))
		top = b;
	else if (dir < 0)
		top = 0;                      // only hidden records above: that is the top
	if (!top)
		newN = 0;
}

// The selected entry and up to want-1 after it, in e[]: how many there are.
static uint8_t Page(AlertRecord_t *e, uint8_t want, uint32_t *first)
{
	uint32_t b;
	uint8_t  n = 1;

	if (!Select(first, &e[0]))
		return 0;
	for (b = *first + 1u; n < want && FindVisible(&b, +1, &e[n]); b++)
		n++;
	NamesKeep(e, n);
	return n;
}

// Where the selected entry is: NEWEST, or how many entries are newer - the
// UP presses back to the top, when UNKNOWN=HIDE hides none of them.
static void Where(char *out, uint32_t back)
{
	if (!top)
		strcpy(out, "NEWEST");
	else
		sprintf(out, "%lu NEWER", (unsigned long)(back > 9999u ? 9999u : back));
}

// ---------------------------------------------------------------------------
// status line

// Fields in px. Text in a field starting at x lights x+1 .. x+7w-1, so these
// leave 2-3 px between neighbours and stop short of the battery icon, which
// is 17 px here (the fork's bitmap, not the stock 13) at the right edge.
#define ST_ALERT_X 0u                 // "ALERT"
#define ST_ALERT_W 5u
#define ST_MARK_X  37u                // "RX" while the squelch is open, or "+N"
#define ST_MARK_W  3u
#define ST_NF_X    59u                // averaged noise floor, "NF-128" at worst
#define ST_NF_W    6u
#define ST_LOG_X   103u               // "L" while logging
#define ST_LOG_W   1u
#define ST_BATT_X  (LCD_WIDTH - sizeof(BITMAP_BatteryLevel1))

// One field, clipped to its width: the value decides the length, the plan
// above decides the room, and UI_PrintStringSmallBufferNormal checks nothing.
static void Status(const char *s, uint8_t x, uint8_t w)
{
	char t[8];
	snprintf(t, (size_t)w + 1u, "%s", s);
	UI_PrintStringSmallBufferNormal(t, gStatusLine + x);
}

void ALERTUI_DrawStatus(void)
{
	char s[8];

	Sync();
	memset(gStatusLine, 0, sizeof(gStatusLine));
	Status("ALERT", ST_ALERT_X, ST_ALERT_W);
	if (newN) {
		sprintf(s, "+%u", newN);
		Status(s, ST_MARK_X, ST_MARK_W);
	} else if (ALERT_SquelchOpen()) {
		Status("RX", ST_MARK_X, ST_MARK_W);
	}
	sprintf(s, "NF%d", ALERT_NoiseFloor());
	Status(s, ST_NF_X, ST_NF_W);
	if (Logging())
		Status("L", ST_LOG_X, ST_LOG_W);
	UI_DrawBattery(gStatusLine + ST_BATT_X, gBatteryDisplayLevel, gLowBatteryBlink);
	ST7565_BlitStatusLine();
}

void ALERTUI_MarkRx(void)
{
	if (newN)
		return;                       // +N keeps the field until the list is back at the top
	memset(gStatusLine + ST_MARK_X, 0, ST_MARK_W * CHAR_PX);
	Status("RX", ST_MARK_X, ST_MARK_W);
	ST7565_BlitStatusLine();
}

// ---------------------------------------------------------------------------
// nothing to show yet

// Nothing to list yet, or the app is still settling / running the census
// (which blocks for seconds: this says why the screen froze). The frequency
// is drawn as the radio's main screen draws it; 12.5 kHz channels need the
// fourth decimal.
static void DrawEmpty(void)
{
	static const char *const adc[5] = { "?", "OFF", "CHK", "OK", "REJ" };   // ALERT_ADC_*
	char     s[32];
	uint8_t  secs = 0;
	const uint32_t f = gRxVfo->pRX->Frequency;

	sprintf(s, "%u.%04u", (unsigned)(f / 100000u), (unsigned)((f % 100000u) / 10u));
	Digits(s, 0, 0);
	Put("MHz", (uint8_t)(DigitsWidth(s, DIGIT_PX, DOT_PX) + 4u), 1, false);
	switch (ALERT_EntryState(&secs)) {
		case ALERT_ENTRY_SETTLE: sprintf(s, "SETTLING %u", secs); break;
		case ALERT_ENTRY_CENSUS: strcpy(s, "CENSUS"); break;
		default:                 strcpy(s, "LISTENING"); break;
	}
	Put(s, 0, 2, true);
	// the squelch, and whether readings are being kept
	sprintf(s, "SQL %u.%u", gEeprom.SQUELCH_LEVEL, gEeprom.SQUELCH_TENTHS);
	Text(s, 0, 3);
	TextRight(!gAlertCfg.log ? "LOG OFF" : Logging() ? "LOG ON" : "NO LOG", 3);
	{	// counters too long together for the spelt-out form get the short
		// one: 2 + 5 + 5 + 5 = 17 at most, bursts being 16-bit
		const unsigned bursts = ALERT_Bursts();
		const unsigned dec    = (unsigned)(ALERT_Decodes() > 99999u ? 99999u : ALERT_Decodes());
		if (sprintf(s, "BURSTS %u  DEC %u", bursts, dec) > (int)UI_COLS)
			sprintf(s, "B %u DEC %u", bursts, dec);
		Text(s, 0, 4);
	}
	// the census result and what became of it: "AUD PA4B+ REJ" is 13
	sprintf(s, "AUD %s%s %s", ALERT_AudName(), ALERTADC_ChoicePa8() ? "+" : "",
	        adc[ALERT_AdcState() % 5u]);
	Text(s, 0, 5);
}

// ---------------------------------------------------------------------------
// the readings, in the layout LAYOUT asks for. The selected entry - the one
// the detail view opens and KEY_0 says again - is always the first, and in
// every layout but CLASSIC its name is in a dark bar.

// BIG: the selected reading alone.
//   ▓STRATHPINE GYMPIE▓   the name in the bar (the marquee's when longer)
//      18  LEVEL          the value in digits twice the frequency's size,
//                         and its kind as its unit; rows 1-4
//   34s AGO     NEWEST    how old, and where in the history
static bool DrawBig(void)
{
	AlertRecord_t r;
	uint32_t      b;
	char          s[UI_COLS + 1], v[8];
	const NameSlot_t *n;

	if (!Page(&r, 1, &b))
		return false;
	n = Name(r.id);
	DrawName(n, UI_COLS, 0, true, true, " (UNKNOWN)");
	Invert(0, 0, BAR_END);

	FormatValue(v, r.value, n->kind, false);
	if (r.value == ALERT_VALUE_FULL_SCALE) {
		Big("FULL", (TEXT_PX - 4u * 8u) / 2u, 2);   // 2047: the sensor's maximum, centred
	} else {
		// value, a gap, then the kind - spelt out when it fits beside the
		// number - the whole centred, the kind on the digits' baseline. An
		// address the table does not have has no kind: the value alone.
		const uint8_t vw = (uint8_t)(DigitsWidth(v, DIGIT2_PX, DOT2_PX) - 4u);
		const char   *k  = n->known ? kindLong[n->kind & 7u] : "";
		uint8_t       w;
		if (vw + 5u + strlen(k) * CHAR_PX > TEXT_PX)
			k = kindShort[n->kind & 7u];
		w = (uint8_t)(*k ? vw + 5u + strlen(k) * CHAR_PX : vw);
		w = (uint8_t)((TEXT_PX - w) / 2u);   // now the left edge
		Digits2(v, w, 1);
		Put(k, (uint8_t)(w + vw + 4u), 4, false);
	}

	AgeAgo(s, &r, 9);
	Text(s, 0, 5);
	Where(s, b);
	TextRight(s, 5);
	DrawScrollBar(b, Total());
	return true;
}

// TWO: two readings, three rows each.
//   ▓STRATHPINE GYMPIE▓   name, the selected one in the bar
//      18  LEVEL          value in the frequency digits, right-aligned in a
//          34s AGO        four-digit field; kind and age beside it
static bool DrawTwo(void)
{
	AlertRecord_t e[2];
	uint32_t      b;
	char          s[UI_COLS + 1], v[8];
	const uint8_t cnt = Page(e, 2, &b);

	if (!cnt)
		return false;
	for (uint8_t i = 0; i < cnt; i++) {
		const uint8_t     row = (uint8_t)(3u * i);
		const NameSlot_t *n   = Name(e[i].id);

		DrawName(n, UI_COLS, row, i == 0u, true, " ?");
		if (i == 0u)
			Invert(row, 0, BAR_END);
		else
			Rule((uint8_t)(row - 1u));      // between the two
		FormatValue(v, e[i].value, n->kind, false);
		if (e[i].value == ALERT_VALUE_FULL_SCALE) {
			Big(v, 20, (uint8_t)(row + 1u));
		} else {
			const uint8_t w = DigitsWidth(v, DIGIT_PX, DOT_PX);
			Digits(v, (uint8_t)(w < 4u * DIGIT_PX ? 4u * DIGIT_PX - w : 0u), (uint8_t)(row + 1u));
		}
		Text(kindLong[n->kind & 7u], 8, (uint8_t)(row + 1u));
		AgeAgo(s, &e[i], UI_COLS - 8u);
		Text(s, 8, (uint8_t)(row + 2u));
	}
	DrawScrollBar(b, Total());
	return true;
}

// CLASSIC: three readings, two rows each - the receiver's first layout.
//   BEACHMERE ST   4m    name (13, the marquee's when selected), age right
//   4133 LVL  22   -87   id, kind, value, peak RSSI
// The second row is 4 + 1 + 4 + 1 + 4 + 4 = 18 whatever the data, every field
// its own width. A four-character value and a three-digit RSSI touch: the
// minus sign separates them then, as there is no nineteenth column.
static void DrawEntry(const AlertRecord_t *r, uint8_t row, bool sel)
{
	char s[UI_COLS + 1], v[8];
	const NameSlot_t *n = Name(r->id);

	DrawName(n, NAME_W, row, sel, false, " ?");
	Age(v, r);
	TextRight(v, row);

	FormatValue(v, r->value, n->kind, false);
	snprintf(s, sizeof(s), "%4u %-4s %-4.4s%4d", r->id, kindShort[n->kind & 7u], v, r->rssi);
	Text(s, 0, (uint8_t)(row + 1u));
}

static bool DrawClassic(void)
{
	AlertRecord_t e[3];
	uint32_t      b;
	const uint8_t cnt = Page(e, 3, &b);

	for (uint8_t i = 0; i < cnt; i++)
		DrawEntry(&e[i], (uint8_t)(2u * i), i == 0u);
	if (cnt)
		DrawScrollBar(b, Total());
	return cnt != 0u;
}

// LIST: six readings, one row each.
//   ▓STRATHPIN   18 34s▓   name (9), value, age; the selected one in the bar
static bool DrawList(void)
{
	AlertRecord_t e[PAGE_MAX];
	uint32_t      b;
	char          s[UI_COLS + 1], v[8];
	const uint8_t cnt = Page(e, PAGE_MAX, &b);

	for (uint8_t i = 0; i < cnt; i++) {
		const NameSlot_t *n = Name(e[i].id);
		DrawName(n, LIST_NAME_W, i, i == 0u, i == 0u, "");
		FormatValue(v, e[i].value, n->kind, false);
		sprintf(s, "%4.4s", v);
		Put(s, (LIST_NAME_W + 1u) * CHAR_PX, i, i == 0u);
		Age(v, &e[i]);
		Put(v, (uint8_t)(TEXT_PX - strlen(v) * CHAR_PX), i, i == 0u);
		if (i == 0u)
			Invert(0, 0, BAR_END);
	}
	if (cnt)
		DrawScrollBar(b, Total());
	return cnt != 0u;
}

// ---------------------------------------------------------------------------
// detail view: the selected entry in full

static bool DrawDetail(void)
{
	AlertRecord_t r;
	uint32_t      b;
	char          s[24], v[8];
	const NameSlot_t *n;

	if (!Page(&r, 1, &b))
		return false;
	n = Name(r.id);
	if (n->known)
		DrawName(n, UI_COLS, 0, true, true, "");
	else
		Put("NOT IN TABLE", 0, 0, true);
	Invert(0, 0, BAR_END);

	sprintf(s, "ID %u %s", r.id, kindLong[n->kind & 7u]);
	Text(s, 0, 1);

	FormatValue(v, r.value, n->kind, true);
	if (sprintf(s, "VAL %s RAW 0x%03X", v, r.value) > (int)UI_COLS)
		sprintf(s, "VAL %s 0x%03X", v, r.value);    // a five-character voltage
	Text(s, 0, 2);

	sprintf(s, "RSSI %d NF %d", r.rssi, r.nf);
	Text(s, 0, 3);

	// V2_SPEC section 4: sensitivity is the floor at the time plus SNR REQ,
	// an estimate; the fade margin is how far over it the burst peaked.
	sprintf(s, "FADE %+ddB", r.rssi - (r.nf + gAlertCfg.snr_req));
	Text(s, 0, 4);

	{	// "ABF STD INV 12s AGO" is 19: AGO goes when INV and the age need the room
		const bool dated = Age(v, &r);
		const int  len   = sprintf(s, "%s %s%s %s", fmtName[r.fmt & 3u],
		                           (r.flags & ALERTREC_POL_STD) ? "STD" : "NEG",
		                           (r.flags & ALERTREC_INV) ? " INV" : "", v);
		if (dated && len + 4 <= (int)UI_COLS)
			strcpy(s + len, " AGO");
		Text(s, 0, 5);
	}
	DrawScrollBar(b, Total());
	return true;
}

// ---------------------------------------------------------------------------
// settings view
//
// As the radio's own menu: UP/DOWN pick a row, MENU starts changing it, UP/
// DOWN change it - at once, so FREQ and SQL LEVEL can be heard - MENU keeps
// the change and EXIT puts the value back. EXIT from the list saves and
// returns. Rows the screen leaves out (ALERT_SetOnScreen) are skipped.
//
// A row whose value is a number with a point in it - FREQ MHz "151.5125",
// SQL LEVEL "3.5" - can also be typed with the number keys, as a frequency
// is on the radio's main screen: each digit fills the next place, the point
// stays where it is, and the value is set the moment the last place is
// filled. MENU sets it sooner, the places left as zeros; EXIT takes the last
// digit back. A digit on such a row in the list starts the change too.

// The next row the screen shows from row, in dir, wrapping round.
static uint8_t NextRow(uint8_t row, int8_t dir)
{
	const uint8_t nset = ALERT_SetCount();

	for (uint8_t i = 0; i < nset; i++) {
		row = (uint8_t)((row + nset + dir) % nset);
		if (ALERT_SetOnScreen(row))
			break;
	}
	return row;
}

static void DrawSettings(void)
{
	char s[UI_COLS + 1], v[12];
	const uint8_t nset = ALERT_SetCount();
	uint8_t order[24], shown = 0, at = 0, first;

	if (setIndex >= nset || !ALERT_SetOnScreen(setIndex))
		setIndex = NextRow(setIndex, +1);
	for (uint8_t row = 0; row < nset && shown < sizeof(order); row++) {
		if (!ALERT_SetOnScreen(row))
			continue;
		if (row == setIndex)
			at = shown;
		order[shown++] = row;
	}
	// a window of five under the title, the selected row kept off its edges
	// while there are rows beyond them
	first = at > 2u ? (uint8_t)(at - 2u) : 0u;
	if (shown > SET_ROWS && first > shown - SET_ROWS)
		first = (uint8_t)(shown - SET_ROWS);
	if (shown <= SET_ROWS)
		first = 0;

	// the title says what the keys do while a value is changing
	if (editing)
		TextCentre("MENU=OK EXIT=UNDO", 0);
	else
		TextCentre("ALERT SETTINGS", 0);
	for (uint8_t i = 0; i < SET_ROWS && first + i < shown; i++) {
		const uint8_t row = order[first + i];
		const uint8_t y   = (uint8_t)(1u + i);
		if (row == setIndex && typedN)
			strcpy(v, typed);           // what has been typed so far: "151.5___"
		else
			ALERT_SetValue(row, v);
		// 9 + 9 = 18: the longest name ("SQL LEVEL") and the longest value
		// ("2 COPIES", "151.5125") with a space between at the least
		sprintf(s, "%-9s%9s", ALERT_SetName(row), v);
		Text(s, 0, y);
		if (row == setIndex)
			Invert(y, editing ? (uint8_t)(TEXT_PX - strlen(v) * CHAR_PX) : 0u, BAR_END);
	}
	DrawScrollBar(at, shown);
}

static bool Numeric(const char *v)
{
	for (; *v; v++)
		if ((*v < '0' || *v > '9') && *v != '.')
			return false;
	return true;
}

// Digits with exactly one point: a value the number keys can type.
static bool Typable(uint8_t row)
{
	char    v[12];
	uint8_t dots = 0;

	ALERT_SetValue(row, v);
	for (const char *c = v; *c; c++) {
		if (*c == '.')
			dots++;
		else if (*c < '0' || *c > '9')
			return false;
	}
	return dots == 1u;
}

// The typed value, the places not reached as zeros, set as the console would
// set it: whole, checked, the frequency to the nearest 12.5 kHz channel.
// False (and a note) when the row refuses it - FREQ outside 130-174 MHz.
static bool TypedSet(void)
{
	char s[12];

	for (uint8_t i = 0; i < sizeof(s); i++)
		if ((s[i] = typed[i] == '_' ? '0' : typed[i]) == 0)
			break;
	typedN = 0;
	if (ALERT_SetTo(setIndex, s) == NULL)
		return true;
	ALERTUI_Note("OUT OF RANGE");
	return false;
}

// One digit into the next place. The first takes the shape of the value as
// it is now: every digit a place, the point kept.
static void TypeDigit(char c)
{
	char *p;

	if (!typedN) {
		ALERT_SetValue(setIndex, typed);
		for (p = typed; *p; p++)
			if (*p != '.')
				*p = '_';
	}
	if ((p = strchr(typed, '_')) == NULL)
		return;
	*p = c;
	typedN++;
	if (!strchr(typed, '_'))
		(void)TypedSet();             // every place filled: set it now
}

// EXIT while typing: the last digit typed is a place again.
static void TypeBack(void)
{
	for (char *p = typed + strlen(typed); p-- > typed;) {
		if (*p >= '0' && *p <= '9') {
			*p = '_';
			typedN--;
			return;
		}
	}
}

static void SettingsKey(KEY_Code_t key, bool held)
{
	char v[12];
	const bool digit = key <= KEY_9;

	if (!editing) {
		if (digit && Typable(setIndex)) {
			// typing on the list starts the change with that digit
			ALERT_SetValue(setIndex, editWas);
			editing = true;
			typedN  = 0;
			TypeDigit((char)('0' + key));
			return;
		}
		switch (key) {
			case KEY_UP:   setIndex = NextRow(setIndex, -1); break;
			case KEY_DOWN: setIndex = NextRow(setIndex, +1); break;
			case KEY_MENU:
				if (held)
					break;
				ALERT_SetValue(setIndex, editWas);
				editing = true;
				typedN  = 0;
				break;
			case KEY_EXIT:
				if (held)
					break;
				ALERT_SettingsChanged();
				view = ALERT_VIEW_MAIN;
				break;
			default: break;
		}
		return;
	}

	if (digit) {
		if (Typable(setIndex))
			TypeDigit((char)('0' + key));
		return;
	}
	if (typedN) {
		// part of a value typed: MENU sets it, EXIT takes a digit back,
		// UP/DOWN forget it and step from the value as it is
		if (key == KEY_MENU) {
			if (TypedSet()) {
				editing = false;
				ALERT_SettingsChanged();
			}
			return;
		}
		if (key == KEY_EXIT) {
			TypeBack();
			return;
		}
		typedN = 0;
	}

	if (held) {
		// A held UP/DOWN repeats every 100 ms: worth it for a number (FREQ,
		// SQL LEVEL, SNR REQ), but it would flip any other row back and forth
		// under the finger - "2 COPIES" starts with a digit too, so the whole
		// value has to be one.
		ALERT_SetValue(setIndex, v);
		if (!Numeric(v))
			return;
	}
	switch (key) {
		case KEY_UP:   ALERT_SetStep(setIndex, +1); break;
		case KEY_DOWN: ALERT_SetStep(setIndex, -1); break;
		case KEY_MENU:
			editing = false;
			ALERT_SettingsChanged();
			break;
		case KEY_EXIT:
			// back to the value MENU started from; CENSUS, an action, has none
			editing = false;
			ALERT_SetValue(setIndex, v);
			if (strcmp(v, editWas))
				(void)ALERT_SetTo(setIndex, editWas);
			break;
		default: break;
	}
}

// ---------------------------------------------------------------------------

void ALERTUI_Reset(void)
{
	view        = ALERT_VIEW_MAIN;
	setIndex    = 0;
	editing     = false;
	typedN      = 0;
	srcLog      = UseLog();
	lastDecodes = ALERT_Decodes();
	idle        = 0;
	mqLive      = false;
	noteTicks   = 0;
	SnapTop();
	for (uint8_t i = 0; i < PAGE_MAX; i++)
		names[i].valid = false;
}

uint8_t ALERTUI_View(void)
{
	return view;
}

// The selected entry (the list's top one, or the detail view's), as entries
// back from the newest in the list's source; 0 while the list is at its top.
// alert.c reads it for EXIT (non-zero: snap back rather than leave) and for
// KEY_0's replay. At the top it is 0 even when UNKNOWN=HIDE is skipping the
// newest records: EXIT must see "at the top" there, or a snap back would
// look like a failed one and leave the app.
uint32_t ALERTUI_Selected(void)
{
	AlertRecord_t r;
	uint32_t      b;

	Sync();
	Source();
	if (!top || !Select(&b, &r))
		return 0;
	return b;
}

void ALERTUI_Note(const char *s)
{
	snprintf(note, sizeof(note), "%s", s);
	noteTicks = NOTE_TICKS;
	ALERT_RequestRedraw();
}

void ALERTUI_NoteRow(uint8_t row)
{
	char s[UI_COLS + 1], v[12];

	ALERT_SetValue(row, v);
	snprintf(s, sizeof(s), "%s: %s", ALERT_SetName(row), v);
	ALERTUI_Note(s);
}

void ALERTUI_Draw(void)
{
	static bool (*const layouts[ALERT_LAYOUT_N])(void) = {
		[ALERT_LAYOUT_BIG] = DrawBig, [ALERT_LAYOUT_TWO] = DrawTwo,
		[ALERT_LAYOUT_CLASSIC] = DrawClassic, [ALERT_LAYOUT_LIST] = DrawList,
	};

	// Spectrum renders at a fixed rate and blits a single line per pass rather
	// than pushing the whole 1 KB framebuffer in one burst. The caller only
	// asks for a redraw with the squelch shut.
	Sync();
	Source();
	ALERTUI_DrawStatus();
	UI_DisplayClear();
	mqLive = false;
	if (view == ALERT_VIEW_SETTINGS) {
		DrawSettings();
	} else if (ALERT_EntryState(NULL) != ALERT_ENTRY_RUN ||
	           !(view == ALERT_VIEW_DETAIL ? DrawDetail()
	                                       : layouts[gAlertCfg.layout % ALERT_LAYOUT_N]())) {
		view = ALERT_VIEW_MAIN;       // nothing to open: the detail view closes
		DrawEmpty();
	}
	if (noteTicks) {
		// over the bottom two rows: what the key just did, in big letters
		// in a bar - covering whole rows, so nothing is left cut in half
		const size_t n = strlen(note);
		memset(gFrameBuffer[UI_ROWS - 2u], 0, BAR_END);
		memset(gFrameBuffer[UI_ROWS - 1u], 0, BAR_END);
		Big(note, (uint8_t)(n < 16u ? (TEXT_PX - n * 8u) / 2u : 0u), UI_ROWS - 2u);
		Invert(UI_ROWS - 2u, 0, BAR_END);
		Invert(UI_ROWS - 1u, 0, BAR_END);
	}
	ST7565_BlitFullScreen();
}

// Keys alert.c passes on: on the main and detail views UP/DOWN scroll, 1 opens
// and closes the detail view, 2 (alert.c has changed LAYOUT) shows the list,
// MENU opens settings, EXIT closes the detail view or snaps the list back to
// the top. Settings: see SettingsKey.
void ALERTUI_Key(KEY_Code_t key, bool held)
{
	idle = 0;
	Sync();
	Source();

	if (view == ALERT_VIEW_SETTINGS) {
		SettingsKey(key, held);
		ALERT_RequestRedraw();
		return;
	}

	switch (key) {
		case KEY_UP:   Step(-1); break;
		case KEY_DOWN: Step(+1); break;
		case KEY_1:
			view = (view == ALERT_VIEW_DETAIL) ? ALERT_VIEW_MAIN : ALERT_VIEW_DETAIL;
			break;
		case KEY_2:    view = ALERT_VIEW_MAIN; break;
		case KEY_MENU:
			view      = ALERT_VIEW_SETTINGS;
			setIndex  = 0;
			editing   = false;
			typedN    = 0;
			noteTicks = 0;            // a note from the list would cover two rows
			break;
		case KEY_EXIT:
			if (view == ALERT_VIEW_DETAIL)
				view = ALERT_VIEW_MAIN;
			else
				SnapTop();
			break;
		default: break;
	}
	ALERT_RequestRedraw();
}

// Runs every 10 ms, squelch open or not, so it only counts: anything that
// draws is left to the redraw it requests, which waits for a quiet moment.
void ALERTUI_Tick10ms(void)
{
	Sync();

	// 30 s without a key: back to the newest, as EXIT would. Not from the
	// settings view, which someone may be reading.
	if (view != ALERT_VIEW_SETTINGS) {
		if (idle < IDLE_TICKS) {
			idle++;
		} else if (top || view == ALERT_VIEW_DETAIL) {
			view = ALERT_VIEW_MAIN;
			SnapTop();
			ALERT_RequestRedraw();
		}
	}

	if (noteTicks && !--noteTicks)
		ALERT_RequestRedraw();        // the note's time is up: the row under it again

	// The marquee: a rest at each end, one character per step in between.
	if (mqLive && mqLen > mqField) {
		const uint8_t end = (uint8_t)(mqLen - mqField);
		if (++mqTick >= ((mqPos == 0u || mqPos >= end) ? MQ_PAUSE : MQ_STEP)) {
			mqTick = 0;
			mqPos  = (mqPos >= end) ? 0u : (uint8_t)(mqPos + 1u);
			ALERT_RequestRedraw();
		}
	}
}

#endif // ENABLE_ALERT
