/* ALERT receiver: every screen - the status line, the decode list, the
 * detail view and the settings view (tools/alert/V2_SPEC.md section 3). It
 * reads the receiver through alert_int.h and changes it only through
 * ALERT_SetStep / ALERT_SettingsChanged.
 *
 * The glass: a 128 px status line plus main rows 0..5. Row 6 sits partly
 * under the bezel on this radio and is never used. The small font is a 6 px
 * glyph plus 1 px of spacing, so a character costs 7 px and a row holds 18;
 * eighteen characters end at px 125, which leaves the last column for the
 * scroll bar. All main-area text goes through Text(), which refuses row 6,
 * and all status text through Status(), which clips each field to the width
 * its column was planned for. tools/alert/check_layout.py checks both, and
 * the status columns, exactly.
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
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "ui/battery.h"
#include "ui/helper.h"

#define CHAR_PX    7u                 // 6 px glyph + 1 px spacing
#define UI_COLS    18u                // characters across a row
#define UI_ROWS    6u                 // main rows 0..5: row 6 is under the bezel
#define BAR_X      (LCD_WIDTH - 1u)   // scroll bar column, clear of every row's text
#define BAR_PX     (UI_ROWS * 8u)     // the bar spans rows 0..5: 48 px
#define PAGE       3u                 // list entries on the glass, two rows each
#define NAME_W     13u                // list name field; the age takes cols 14..17
#define SCAN_MAX   128u               // records read per search past hidden ones (UNKNOWN=HIDE)
#define IDLE_TICKS 3000u              // 30 s of 10 ms ticks without a key: back to the top
#define MQ_STEP    30u                // marquee: one character every 300 ms
#define MQ_PAUSE   100u               // ... and a 1 s rest at each end

static uint8_t  view;             // ALERT_VIEW_*
static uint8_t  setIndex;         // selected settings row
static bool     srcLog;           // the list reads the SPI log, else the RAM ring
static uint32_t top;              // back index the list starts at, 0 = newest
static uint16_t newN;             // decodes that arrived while the list was held
static uint32_t lastDecodes;      // ALERT_Decodes() when last looked at
static uint16_t idle;             // 10 ms ticks since the last UI key

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
static NameSlot_t names[PAGE];
static uint16_t   namesCount;         // ALERTSTN_Count() they were looked up against

// Kinds: four characters on the list, spelt out in the detail view. Index 0
// (and 7, which no table uses) is an address the table does not have.
static const char kindShort[8][5] = { "-", "RAIN", "LVL", "BATT", "REP", "OTH", "CHK", "-" };
static const char kindLong[8][9]  = { "UNKNOWN", "RAIN", "LEVEL", "BATTERY", "REPEATER",
                                      "OTHER", "CHECK", "UNKNOWN" };
static const char fmtName[4][4]   = { "?", "ABF", "EIF", "A2C" };   // ALERT_FMT_*

// ---------------------------------------------------------------------------
// drawing primitives

// Every line of main-area text. The row check is here, at run time, because
// most rows are computed; the right edge is UI_PrintStringSmall's, which
// clips to what fits, so no string the data makes can pass column 17.
static void Text(const char *s, uint8_t col, uint8_t row)
{
	if (row < UI_ROWS)
		UI_PrintStringSmallNormal(s, (uint8_t)(col * CHAR_PX), 0, row);
}

static void TextRight(const char *s, uint8_t row)
{
	const size_t n = strlen(s);
	Text(s, (uint8_t)(n < UI_COLS ? UI_COLS - n : 0u), row);
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
// convention), everything else the raw count. The list has four characters
// for it, so there the unit goes (the BATT label says it) and a three-digit
// voltage loses its tenths; the detail view has room for both.
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
// age ("-", and false).
static bool Age(char *out, const AlertRecord_t *r)
{
	static const uint32_t unit[4] = { 1u, 60u, 3600u, 86400u };
	const uint32_t now = ALERT_Epoch();
	uint32_t s;
	uint8_t  i = 0;

	if (r->epoch && now >= r->epoch) {
		s = now - r->epoch;
	} else if (r->boot == ALERTLOG_Boot()) {
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

// ---------------------------------------------------------------------------
// station names

// Keep only the slots the coming frame asks for, so a miss always finds a
// free slot and never evicts a name it is about to draw. A table uploaded or
// cleared since changes the count, and then nothing cached is kept.
static void NamesKeep(const AlertRecord_t *e, uint8_t count)
{
	const uint16_t stations = ALERTSTN_Count();

	for (uint8_t i = 0; i < PAGE; i++) {
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

	for (uint8_t i = 0; i < PAGE; i++) {
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
static void DrawName(const NameSlot_t *n, uint8_t w, uint8_t row, bool sel)
{
	char    s[UI_COLS + 1];
	uint8_t off = 0;

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
	Text(s, 0, row);
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

// ---------------------------------------------------------------------------
// main view

// Nothing to list yet, or the app is still settling / running the census
// (which blocks for seconds: this says why the screen froze).
static void DrawEmpty(void)
{
	static const char *const adc[5] = { "?", "OFF", "CHK", "OK", "REJ" };   // ALERT_ADC_*
	char     s[32];
	uint8_t  secs = 0;
	const uint32_t f = gRxVfo->pRX->Frequency;

	sprintf(s, "%u.%05u MHz", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
	Text(s, 0, 0);
	switch (ALERT_EntryState(&secs)) {
		case ALERT_ENTRY_SETTLE: sprintf(s, "SETTLING %u", secs); break;
		case ALERT_ENTRY_CENSUS: strcpy(s, "CENSUS"); break;
		default:                 strcpy(s, "WAITING"); break;
	}
	Text(s, 0, 1);
	sprintf(s, "NF %d dBm", ALERT_NoiseFloor());
	Text(s, 0, 2);
	{	// counters too long together for the spelt-out form get the short one
		const unsigned bursts = ALERT_Bursts();
		const unsigned dec    = (unsigned)(ALERT_Decodes() > 99999u ? 99999u : ALERT_Decodes());
		if (sprintf(s, "BURSTS %u  DEC %u", bursts, dec) > (int)UI_COLS)
			sprintf(s, "BST %u DEC %u", bursts, dec);
		Text(s, 0, 3);
	}
	// the census result and what became of it: "AUD PA4B+ REJ" is 13
	sprintf(s, "AUD %s%s %s", ALERT_AudName(), ALERTADC_ChoicePa8() ? "+" : "",
	        adc[ALERT_AdcState() % 5u]);
	Text(s, 0, 4);
	if (!gAlertCfg.log)
		strcpy(s, "LOG OFF");
	else if (Logging())
		sprintf(s, "LOG OK %u", (unsigned)ALERTLOG_Count());
	else
		sprintf(s, "LOG %s", ALERTLOG_State());
	Text(s, 0, 5);
}

// One list entry, two rows:
//   BEACHMERE ST   4m    name (13, the marquee's when selected), age right
//   4133 LVL  22   -87   id, kind, value, peak RSSI
// The second row is 4 + 1 + 4 + 1 + 4 + 4 = 18 whatever the data, every field
// its own width. A four-character value and a three-digit RSSI touch: the
// minus sign separates them then, as there is no nineteenth column.
static void DrawEntry(const AlertRecord_t *r, uint8_t row, bool sel)
{
	char s[UI_COLS + 1], v[8];
	const NameSlot_t *n = Name(r->id);

	if (n->known) {
		DrawName(n, NAME_W, row, sel);
	} else {
		sprintf(s, "ID %u ?", r->id);
		Text(s, 0, row);
	}
	Age(v, r);
	TextRight(v, row);

	FormatValue(v, r->value, n->kind, false);
	snprintf(s, sizeof(s), "%4u %-4s %-4.4s%4d", r->id, kindShort[n->kind & 7u], v, r->rssi);
	Text(s, 0, (uint8_t)(row + 1u));
}

static bool DrawList(void)
{
	AlertRecord_t e[PAGE];
	uint32_t      first, b;
	uint8_t       n = 1;

	if (!Select(&first, &e[0]))
		return false;
	for (b = first + 1u; n < PAGE && FindVisible(&b, +1, &e[n]); b++)
		n++;
	NamesKeep(e, n);
	for (uint8_t i = 0; i < n; i++)
		DrawEntry(&e[i], (uint8_t)(2u * i), i == 0u);
	DrawScrollBar(first, Total());
	return true;
}

// ---------------------------------------------------------------------------
// detail view: the selected entry in full

static bool DrawDetail(void)
{
	AlertRecord_t r;
	uint32_t      b;
	char          s[24], v[8];
	const NameSlot_t *n;

	if (!Select(&b, &r))
		return false;
	NamesKeep(&r, 1);
	n = Name(r.id);
	if (n->known)
		DrawName(n, UI_COLS, 0, true);
	else
		Text("NOT IN TABLE", 0, 0);

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

static void DrawSettings(void)
{
	char s[24], v[16];
	const uint8_t nset = ALERT_SetCount();
	// A window of five rows, not six: row 6 is under the bezel on this radio
	// and the selected row could land there and be invisible.
	const uint8_t rows = nset < 5u ? nset : 5u;
	uint8_t first;

	if (setIndex >= nset)
		setIndex = 0;
	first = (setIndex >= 4u) ? (uint8_t)(setIndex - 4u) : 0u;
	if (first > nset - rows)
		first = (uint8_t)(nset - rows);
	UI_PrintStringSmallNormal("ALERT SETTINGS", 0, 127, 0);
	for (uint8_t i = 0; i < rows; i++) {
		const uint8_t idx = (uint8_t)(first + i);
		ALERT_SetValue(idx, v);
		// 1 + 9 + 8 = 18: the longest name ("SQL LEVEL") and the longest value
		// ("2 COPIES", "PENDING") together fill the row.
		sprintf(s, "%c%-9s%s", idx == setIndex ? '>' : ' ', ALERT_SetName(idx), v);
		Text(s, 0, (uint8_t)(1u + i));
	}
	DrawScrollBar(setIndex, nset);
}

// ---------------------------------------------------------------------------

void ALERTUI_Reset(void)
{
	view        = ALERT_VIEW_MAIN;
	setIndex    = 0;
	srcLog      = UseLog();
	lastDecodes = ALERT_Decodes();
	idle        = 0;
	mqLive      = false;
	SnapTop();
	for (uint8_t i = 0; i < PAGE; i++)
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

void ALERTUI_Draw(void)
{
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
	           !(view == ALERT_VIEW_DETAIL ? DrawDetail() : DrawList())) {
		view = ALERT_VIEW_MAIN;       // nothing to open: the detail view closes
		DrawEmpty();
	}
	ST7565_BlitFullScreen();
}

// Keys alert.c passes on: on the main and detail views UP/DOWN scroll, 1 opens
// and closes the detail view, MENU opens settings, EXIT closes the detail view
// or snaps the list back to the top; in settings UP/DOWN change the value,
// MENU/STAR move the cursor and EXIT saves.
void ALERTUI_Key(KEY_Code_t key, bool held)
{
	idle = 0;
	Sync();
	Source();

	if (view == ALERT_VIEW_SETTINGS) {
		const uint8_t nset = ALERT_SetCount();
		if (held) {
			// A held UP/DOWN repeats every 100 ms: worth it for a number
			// (FREQ, SQL LEVEL, SNR REQ), but it would flip an OFF/ON row
			// back and forth under the finger.
			char v[12];
			ALERT_SetValue(setIndex, v);
			if (v[0] < '0' || v[0] > '9')
				return;
		}
		switch (key) {
			case KEY_UP:   ALERT_SetStep(setIndex, +1); break;
			case KEY_DOWN: ALERT_SetStep(setIndex, -1); break;
			case KEY_MENU: setIndex = (uint8_t)((setIndex + 1) % nset); break;
			case KEY_STAR: setIndex = (uint8_t)((setIndex + nset - 1) % nset); break;
			case KEY_EXIT:
				ALERT_SettingsChanged();
				view = ALERT_VIEW_MAIN;
				break;
			default: break;
		}
		ALERT_RequestRedraw();
		return;
	}

	switch (key) {
		case KEY_UP:   Step(-1); break;
		case KEY_DOWN: Step(+1); break;
		case KEY_1:
			view = (view == ALERT_VIEW_DETAIL) ? ALERT_VIEW_MAIN : ALERT_VIEW_DETAIL;
			break;
		case KEY_MENU: view = ALERT_VIEW_SETTINGS; setIndex = 0; break;
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
