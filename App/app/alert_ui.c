/* ALERT receiver: every screen - the status line, the main view and the
 * settings view. It reads the receiver through alert_int.h and changes it
 * only through ALERT_SetStep / ALERT_SettingsChanged.
 *
 * The glass: a 128 px status line plus main rows 0..5. Row 6 sits partly
 * under the bezel on this radio and is never used. The small font is a 6 px
 * glyph plus 1 px of spacing, so a character costs 7 px and a row holds 18.
 * tools/alert/check_layout.py enforces what of this can be checked exactly.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <string.h>

#include "app/alert_adc.h"
#include "app/alert_decode.h"
#include "app/alert_int.h"
#include "app/alert_stn.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/ui.h"

static uint8_t view;             // ALERT_VIEW_*
static uint8_t setIndex;         // selected settings row

void ALERTUI_Reset(void)
{
	view     = ALERT_VIEW_MAIN;
	setIndex = 0;
}

uint8_t ALERTUI_View(void)
{
	return view;
}

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

// ---------------------------------------------------------------------------
// status line

// Both halves of this line once ignored the character width and were drawn
// over each other. The columns below are picked from the widths, with the
// reading right-aligned into the space that is left.
#define ST_CHAR   7u
#define ST_COL_IN (6u * ST_CHAR)     // 42: clear of "ALERT"
#define ST_COL_SQ (16u * ST_CHAR)    // 112: last two cells
void ALERTUI_DrawStatus(void)
{
	char s[16];
	memset(gStatusLine, 0, sizeof(gStatusLine));

	UI_PrintStringSmallBufferNormal("ALERT", gStatusLine + 0);
	// the route in use: the ADC is the only one left
	UI_PrintStringSmallBufferNormal("ADC", gStatusLine + ST_COL_IN);

	sprintf(s, "%ddBm", ALERT_Rssi());
	{	// right-align, but never start before the tag ends
		unsigned int col = (unsigned int)(ST_COL_SQ - strlen(s) * ST_CHAR);
		if (col < ST_COL_IN + 4u * ST_CHAR)
			col = ST_COL_IN + 4u * ST_CHAR;
		UI_PrintStringSmallBufferNormal(s, gStatusLine + col);
	}

	if (ALERT_SquelchOpen())
		UI_PrintStringSmallBufferNormal("+", gStatusLine + ST_COL_SQ + ST_CHAR);
	ST7565_BlitStatusLine();
}

// ---------------------------------------------------------------------------
// main view

static void DrawMain(void)
{
	char s[32], st[16];
	const uint8_t n = ALERT_HistoryCount();

	UI_DisplayClear();

	if (n == 0) {
		// Row 6 is deliberately left empty, and no line here exceeds the
		// eighteen characters the small font fits across 128 px.
		const uint32_t f = gRxVfo->pRX->Frequency;
		sprintf(s, "%u.%05u MHz", (unsigned)(f / 100000u), (unsigned)(f % 100000u));
		UI_PrintStringSmallNormal(s, 0, 127, 0);

		// what the app is doing: at most 12 + 1 + 5 = 18
		{
			uint8_t secs = 0;
			switch (ALERT_EntryState(&secs)) {
				case ALERT_ENTRY_SETTLE: sprintf(st, "SETTLING %u", secs); break;
				case ALERT_ENTRY_CENSUS: strcpy(st, "CENSUS"); break;
				default:                 strcpy(st, "LISTENING"); break;
			}
		}
		sprintf(s, "%s %s", st, gAlertCfg.voice ? "VOICE" : "QUIET");
		UI_PrintStringSmallNormal(s, 0, 127, 1);

		// N qualifying bursts, F frames, P peak of the last burst, so it is
		// obvious whether a transmission is getting through at all: 17 at most
		sprintf(s, "N%u F%u P%d", Cap(ALERT_Bursts(), 9999u), Cap(ALERT_Decodes(), 9999u),
		        ALERT_LastPeak());
		UI_PrintStringSmallNormal(s, 0, 0, 2);

		// "FLOOR -128 SQL 9.9" is 18
		sprintf(s, "FLOOR %d SQL %u.%u", ALERT_NoiseFloor(), gEeprom.SQUELCH_LEVEL,
		        gEeprom.SQUELCH_TENTHS);
		UI_PrintStringSmallNormal(s, 0, 0, 3);

		{	// the census result and what became of it: "AUD PA4B+ REJ" is 13
			static const char *const states[5] = { "?", "NO", "CHK", "ON", "REJ" };
			sprintf(s, "AUD %s%s %s", ALERT_AudName(), ALERTADC_ChoicePa8() ? "+" : "",
			        states[ALERT_AdcState() % 5u]);
			UI_PrintStringSmallNormal(s, 0, 0, 5);
		}
	} else {
		const AlertRecord_t *h = ALERT_History(0);
		char name[ALERT_NAME_MAX + 1];
		uint8_t kind;
		ALERTSTN_Lookup(h->id, name, sizeof(name), &kind);

		// line 0: station name (or unknown), line 1-2: value big, right: kind/id
		if (name[0]) {
			name[18] = 0;                  // the row, not the table, sets the width
			UI_PrintStringSmallNormal(name, 0, 0, 0);
		} else {
			sprintf(s, "ID %u UNKNOWN", h->id);   // "NOT IN TABLE" ran to 20
			UI_PrintStringSmallNormal(s, 0, 0, 0);
		}
		FormatValue(s, h->value, kind);
		UI_PrintString(s, 0, 0, 1, 8);
		sprintf(s, "%s", ALERT_KindLabel(kind));
		UI_PrintStringSmallNormal(s, 88, 0, 1);
		sprintf(s, "#%u", h->id);
		UI_PrintStringSmallNormal(s, 88, 0, 2);

		// rows 3..5: history. Row 6 is under the bezel, see above.
		for (uint8_t i = 0; i < 3 && i < n; i++) {
			const AlertRecord_t *e = ALERT_History(i);
			char v[16];
			uint8_t k2;
			ALERTSTN_Lookup(e->id, name, sizeof(name), &k2);
			FormatValue(v, e->value, k2);
			// 4 + 1 + 7 + 1 + 5 = 18, exactly what fits. It was 21.
			sprintf(s, "%4u %-7.7s %5s", e->id, name[0] ? name : "?", v);
			UI_PrintStringSmallNormal(s, 0, 0, 3 + i);
		}
	}
	ST7565_BlitFullScreen();
}

// ---------------------------------------------------------------------------
// settings view

static void DrawSettings(void)
{
	char s[24], v[16];
	const uint8_t nset = ALERT_SetCount();
	UI_DisplayClear();
	UI_PrintStringSmallNormal("ALERT SETTINGS", 0, 127, 0);
	// A window of five rows, not six: row 6 is under the bezel on this radio
	// and the selected row could land there and be invisible.
	uint8_t first = (setIndex >= 4) ? (uint8_t)(setIndex - 4) : 0;
	if (first > nset - 5) first = (uint8_t)(nset - 5);
	for (uint8_t i = 0; i < 5; i++) {
		const uint8_t idx = (uint8_t)(first + i);
		ALERT_SetValue(idx, v);
		// 1 + 9 + 8 = 18: the longest name ("SQL LEVEL") and the longest value
		// ("2 COPIES", "PENDING") together fill the row.
		sprintf(s, "%c%-9s%s", idx == setIndex ? '>' : ' ', ALERT_SetName(idx), v);
		UI_PrintStringSmallNormal(s, 0, 0, 1 + i);
	}
	ST7565_BlitFullScreen();
}

// ---------------------------------------------------------------------------

void ALERTUI_Draw(void)
{
	// Spectrum renders at a fixed rate and blits a single line per pass rather
	// than pushing the whole 1 KB framebuffer in one burst. The caller only
	// asks for a redraw with the squelch shut.
	ALERTUI_DrawStatus();
	switch (view) {
		case ALERT_VIEW_SETTINGS: DrawSettings(); break;
		default:                  DrawMain(); break;
	}
}

void ALERTUI_Key(KEY_Code_t key, bool held)
{
	const uint8_t nset = ALERT_SetCount();
	(void)held;

	if (view == ALERT_VIEW_SETTINGS) {
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
		case KEY_MENU: view = ALERT_VIEW_SETTINGS; setIndex = 0; break;
		case KEY_EXIT: view = ALERT_VIEW_MAIN; break;   // from any sub-view
		default: break;
	}
	ALERT_RequestRedraw();
}

void ALERTUI_Tick10ms(void)
{
	// nothing moves on the screen yet (the marquee and the snap-back to the
	// top of the list come with the V2 screens)
}

#endif // ENABLE_ALERT
