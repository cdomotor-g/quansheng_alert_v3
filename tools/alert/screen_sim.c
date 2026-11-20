/* The ALERT receiver's screens, drawn on a PC by the firmware's own code.
 *
 * App/app/alert_ui.c is compiled for the host together with the parts of the
 * firmware it draws with (ui/helper.c, font.c, bitmaps.c, ui/battery.c and the
 * embedded printf), so what this prints is, pixel for pixel, what the radio
 * would show. Only the radio is pretended: alert.c's receiver, the log, the
 * station table and the display controller are the stand-ins below.
 *
 * tools/alert/screens.py builds it, feeds it a script on stdin and turns the
 * screens it prints into PNGs. The script is one command a line:
 *
 *   setrow NAME=VALUE       a settings row, in alert.c's order (screens.py reads setNames[])
 *   stn ID KIND NAME...     a station in the table (KIND: ALERT_KIND_* number)
 *   rec ID VALUE RSSI AGE [NF]   a decode AGE seconds ago, newest last
 *   logtotal N              the log holds N records (older ones repeat the newest)
 *   cfg NAME N              gAlertCfg field: layout, show_unknown, log, speaker, voice, snr_req
 *   entry STATE [SECS]      0 settling, 1 census, 2 receiving
 *   sq 0|1                  squelch shut / open
 *   nf DBM                  averaged noise floor
 *   freq HZ10               receive frequency in 10 Hz units (15150000 = 151.500 MHz)
 *   batt LEVEL              battery level as the status line takes it (6 = full)
 *   key NAME [held]         a key, routed as alert.c's OnKey routes it
 *   tick N                  N passes of the 10 ms tick
 *   draw NAME               draw the current view and print it as SCREEN NAME + 8 SCR lines
 *
 * Every draw also checks that nothing was drawn on row 6, which is under the
 * bezel on this radio: a pixel there prints BEZEL and the run fails.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/alert_adc.h"
#include "app/alert_int.h"
#include "app/alert_log.h"
#include "app/alert_stn.h"
#include "driver/st7565.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/inputbox.h"

// ---------------------------------------------------------------------------
// the display and the globals the drawing code reads

uint8_t gStatusLine[LCD_WIDTH];
uint8_t gFrameBuffer[FRAME_LINES][LCD_WIDTH];

static unsigned blits;
void ST7565_BlitFullScreen(void) { blits++; }
void ST7565_BlitStatusLine(void) {}
void ST7565_DrawLine(const unsigned int Column, const unsigned int Line, const uint8_t *pBitmap,
                     const unsigned int Size)
{
	(void)Column; (void)Line; (void)pBitmap; (void)Size;   // ui/battery.c's other writer: unused
}

static FREQ_Config_t rxFreq = { .Frequency = 15150000u };
static VFO_Info_t    rxVfo  = { .pRX = &rxFreq };
VFO_Info_t *gRxVfo = &rxVfo;

uint8_t         gBatteryDisplayLevel = 6;
bool            gLowBatteryBlink;
EEPROM_Config_t gEeprom;
uint8_t         gKeypadLocked;
char            gInputBox[8];
uint8_t         gInputBoxIndex;
void _putchar(char c) { (void)c; }          // the embedded printf's sink: unused

// ---------------------------------------------------------------------------
// the receiver (alert.c)

AlertCfg_t gAlertCfg;

#define NREC 64
static AlertRecord_t recs[NREC];             // newest first
static uint8_t  nrec;
static uint32_t logTotal;
static uint32_t uptimeMs = 3600000u;
static uint32_t decodes;
static int8_t   noiseFloor = -112;
static bool     sqOpen;
static uint8_t  entryState = ALERT_ENTRY_RUN, entrySecs;
static bool     redrawReq;

int8_t   ALERT_NoiseFloor(void)   { return noiseFloor; }
int8_t   ALERT_Rssi(void)         { return noiseFloor; }
bool     ALERT_SquelchOpen(void)  { return sqOpen; }
uint32_t ALERT_Epoch(void)        { return 0; }
uint32_t ALERT_UptimeMs(void)     { return uptimeMs; }
uint16_t ALERT_Bursts(void)       { return (uint16_t)(decodes + decodes / 4u); }
uint32_t ALERT_Decodes(void)      { return decodes; }
int16_t  ALERT_LastPeak(void)     { return -90; }
void     ALERT_RequestRedraw(void) { redrawReq = true; }
void     ALERT_SettingsChanged(void) {}
uint8_t  ALERT_HistoryCount(void) { return nrec < 16u ? nrec : 16u; }
uint8_t  ALERT_AdcState(void)     { return ALERT_ADC_ON; }
bool     ALERT_CensusPending(void) { return false; }
const char *ALERT_AudName(void)   { return "PA4B"; }
bool     ALERTADC_ChoicePa8(void) { return false; }

const AlertRecord_t *ALERT_History(uint8_t back)
{
	return back < ALERT_HistoryCount() ? &recs[back] : NULL;
}

uint8_t ALERT_EntryState(uint8_t *secsLeft)
{
	if (secsLeft)
		*secsLeft = entrySecs;
	return entryState;
}

// the log: the same records, and as many older ones as logtotal asks for
uint32_t ALERTLOG_Count(void)     { return nrec ? (logTotal > nrec ? logTotal : nrec) : 0; }
uint32_t ALERTLOG_Capacity(void)  { return 12288u; }
uint16_t ALERTLOG_Boot(void)      { return 7; }
const char *ALERTLOG_State(void)  { return gAlertCfg.log ? "OK" : "OFF"; }

bool ALERTLOG_ReadBack(uint32_t back, AlertRecord_t *r, uint32_t *seq)
{
	if (back >= ALERTLOG_Count())
		return false;
	*r = recs[back % nrec];
	if (back >= nrec)                   // an older copy: a minute further back per lap
		r->uptime_ms -= (uint32_t)(back / nrec) * 60000u;
	if (seq)
		*seq = 1000u - back;
	return true;
}

// ---------------------------------------------------------------------------
// the station table

typedef struct { uint16_t id; uint8_t kind; char name[ALERT_NAME_MAX + 1]; } SimStn_t;
static SimStn_t stns[64];
static uint8_t  nstn;

bool ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind)
{
	for (uint8_t i = 0; i < nstn; i++) {
		if (stns[i].id == id) {
			snprintf(name, name_max, "%s", stns[i].name);
			*kind = stns[i].kind;
			return true;
		}
	}
	name[0] = 0;
	*kind = ALERT_KIND_NONE;
	return false;
}

uint16_t    ALERTSTN_Count(void)  { return 443; }
const char *ALERTSTN_Source(void) { return "BUILTIN MegaNet:95f6f8d"; }

// ---------------------------------------------------------------------------
// settings rows: the names come from alert.c (screens.py passes them in),
// the values are stand-ins that change the way the real ones do

#define NROW 20
static char rowName[NROW][12], rowVal[NROW][12];
static uint8_t nrow;

static const char *const layoutNames[ALERT_LAYOUT_N] = { "BIG", "TWO", "CLASSIC", "LIST" };
static const char *const spkNames[3] = { "OFF", "SQL", "ON" };

uint8_t     ALERT_SetCount(void)            { return nrow; }
const char *ALERT_SetName(uint8_t row)      { return row < nrow ? rowName[row] : ""; }

// as the RescueOps build: no voice prompts, and MDM MODE only on the console
bool ALERT_SetOnScreen(uint8_t row)
{
	return row < nrow && strcmp(rowName[row], "MDM MODE") && strcmp(rowName[row], "VOICE");
}

void ALERT_SetValue(uint8_t row, char *out)
{
	if (row >= nrow)
		strcpy(out, "");
	else if (!strcmp(rowName[row], "LAYOUT"))
		strcpy(out, layoutNames[gAlertCfg.layout % ALERT_LAYOUT_N]);
	else if (!strcmp(rowName[row], "SPEAKER"))
		strcpy(out, spkNames[gAlertCfg.speaker % 3u]);
	else
		strcpy(out, rowVal[row]);
}

static int Row(const char *name)
{
	for (uint8_t i = 0; i < nrow; i++)
		if (!strcmp(rowName[i], name))
			return i;
	return -1;
}

bool ALERT_SetStep(uint8_t row, int dir)
{
	char *v = rowVal[row];

	if (row >= nrow)
		return false;
	if (!strcmp(rowName[row], "LAYOUT")) {
		gAlertCfg.layout = (uint8_t)((gAlertCfg.layout + ALERT_LAYOUT_N + dir) % ALERT_LAYOUT_N);
	} else if (!strcmp(rowName[row], "SPEAKER")) {
		gAlertCfg.speaker = (uint8_t)((gAlertCfg.speaker + 3 + dir) % 3);
	} else if (!strcmp(rowName[row], "FREQ MHz")) {
		const long f = (long)(atof(v) * 10000.0 + 0.5) + dir * 125;
		sprintf(v, "%ld.%04ld", f / 10000, f % 10000);
	} else if (!strcmp(rowName[row], "SQL LEVEL")) {
		int t = (int)(atof(v) * 10.0 + 0.5) + dir;
		t = t < 0 ? 0 : (t > 90 ? 90 : t);
		sprintf(v, "%d.%d", t / 10, t % 10);
	} else if (!strcmp(v, "ON") || !strcmp(v, "OFF")) {
		strcpy(v, strcmp(v, "ON") ? "ON" : "OFF");
	} else if (!strcmp(v, "SHOW") || !strcmp(v, "HIDE")) {
		strcpy(v, strcmp(v, "SHOW") ? "SHOW" : "HIDE");
	} else {
		return false;
	}
	return true;
}

const char *ALERT_SetTo(uint8_t row, const char *value)
{
	if (row >= nrow)
		return "ARGS";
	if (!strcmp(rowName[row], "LAYOUT") || !strcmp(rowName[row], "SPEAKER")) {
		const bool lay = !strcmp(rowName[row], "LAYOUT");
		for (uint8_t i = 0; i < (lay ? ALERT_LAYOUT_N : 3u); i++)
			if (!strcmp(value, lay ? layoutNames[i] : spkNames[i])) {
				*(lay ? &gAlertCfg.layout : &gAlertCfg.speaker) = i;
				return NULL;
			}
		return "ARGS";
	}
	snprintf(rowVal[row], sizeof(rowVal[row]), "%s", value);
	return NULL;
}

// ---------------------------------------------------------------------------
// keys, as alert.c's OnKey routes them: EXIT, *, F, 0 and 2 on the list and
// the detail view are the app's; everything else is the UI's

static void QuickStep(const char *name)
{
	const int row = Row(name);
	if (row >= 0) {
		ALERT_SetStep((uint8_t)row, +1);
		ALERTUI_NoteRow((uint8_t)row);
	}
}

static void Key(KEY_Code_t key, bool held)
{
	const uint8_t view = ALERTUI_View();

	if (!held && view != ALERT_VIEW_SETTINGS) {
		switch (key) {
			case KEY_EXIT:
				if (view != ALERT_VIEW_MAIN)
					break;
				if (ALERTUI_Selected()) {
					ALERTUI_Key(KEY_EXIT, false);
					if (!ALERTUI_Selected())
						return;
				}
				printf("LEAVE\n");
				return;
			case KEY_STAR:
			case KEY_0:
				ALERTUI_Note("NO VOICE");
				return;
			case KEY_F:
				QuickStep("SPEAKER");
				return;
			case KEY_2:
				QuickStep("LAYOUT");
				break;
			default:
				break;
		}
	}
	ALERTUI_Key(key, held);
}

static KEY_Code_t KeyCode(const char *s)
{
	static const char *const names[] = {
		"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
		"MENU", "UP", "DOWN", "EXIT", "STAR", "F", "PTT", "SIDE2", "SIDE1",
	};
	for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++)
		if (!strcmp(s, names[i]))
			return (KEY_Code_t)i;
	fprintf(stderr, "unknown key %s\n", s);
	exit(2);
}

// ---------------------------------------------------------------------------

static bool Dump(const char *name)
{
	bool clean = true;

	printf("SCREEN %s\n", name);
	for (int row = 0; row < 8; row++) {
		const uint8_t *p = row ? gFrameBuffer[row - 1] : gStatusLine;
		printf("SCR,%d,", row);
		for (int x = 0; x < LCD_WIDTH; x++)
			printf("%02X", p[x]);
		printf("\n");
	}
	for (int x = 0; x < LCD_WIDTH; x++)
		clean &= gFrameBuffer[6][x] == 0;
	if (!clean)
		printf("BEZEL %s\n", name);
	return clean;
}

static int SetCfg(const char *f, int v)
{
	if      (!strcmp(f, "layout"))       gAlertCfg.layout = (uint8_t)v;
	else if (!strcmp(f, "show_unknown")) gAlertCfg.show_unknown = v;
	else if (!strcmp(f, "log"))          gAlertCfg.log = v;
	else if (!strcmp(f, "speaker"))      gAlertCfg.speaker = (uint8_t)v;
	else if (!strcmp(f, "voice"))        gAlertCfg.voice = v;
	else if (!strcmp(f, "snr_req"))      gAlertCfg.snr_req = (uint8_t)v;
	else return -1;
	return 0;
}

int main(void)
{
	char line[256];
	int  ok = 1, n = 0;

	gAlertCfg.log = true;
	gAlertCfg.show_unknown = true;
	gAlertCfg.speaker = ALERT_SPK_SQL;
	gAlertCfg.snr_req = ALERT_SNR_DEFAULT;
	gAlertCfg.layout = ALERT_LAYOUT_DEFAULT;
	gEeprom.SQUELCH_LEVEL = 3;
	ALERTUI_Reset();

	while (fgets(line, sizeof(line), stdin)) {
		char cmd[16] = "", a[64] = "", b[64] = "";
		int  i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0;

		line[strcspn(line, "\r\n")] = 0;
		n++;
		if (!line[0] || line[0] == '#' || sscanf(line, "%15s", cmd) != 1)
			continue;

		if (!strcmp(cmd, "setrow")) {
			const char *eq = strchr(line, '=');
			if (!eq || nrow >= NROW)
				goto bad;
			snprintf(rowName[nrow], sizeof(rowName[nrow]), "%.*s", (int)(eq - line - 7), line + 7);
			snprintf(rowVal[nrow], sizeof(rowVal[nrow]), "%s", eq + 1);
			nrow++;
		} else if (!strcmp(cmd, "stn")) {
			int off = 0;
			if (sscanf(line, "stn %d %d %n", &i1, &i2, &off) < 2 || !off || nstn >= 64)
				goto bad;
			stns[nstn].id = (uint16_t)i1;
			stns[nstn].kind = (uint8_t)i2;
			snprintf(stns[nstn].name, sizeof(stns[nstn].name), "%s", line + off);
			nstn++;
		} else if (!strcmp(cmd, "rec")) {
			AlertRecord_t r = {0};
			i5 = noiseFloor;
			if (sscanf(line, "rec %d %d %d %d %d", &i1, &i2, &i3, &i4, &i5) < 4)
				goto bad;
			r.id = (uint16_t)i1;
			r.value = (uint16_t)i2;
			r.rssi = (int8_t)i3;
			r.uptime_ms = uptimeMs - (uint32_t)i4 * 1000u;
			r.nf = (int8_t)i5;
			r.boot = ALERTLOG_Boot();
			r.fmt = 1;
			r.flags = ALERTREC_POL_STD;
			for (uint8_t k = 0; k < nstn; k++)
				if (stns[k].id == r.id)
					r.flags |= ALERTREC_TABLE;
			memmove(&recs[1], &recs[0], sizeof(recs[0]) * (NREC - 1));
			recs[0] = r;
			if (nrec < NREC)
				nrec++;
			decodes++;
		} else if (!strcmp(cmd, "logtotal")) {
			if (sscanf(line, "logtotal %d", &i1) != 1)
				goto bad;
			logTotal = (uint32_t)i1;
		} else if (!strcmp(cmd, "cfg")) {
			if (sscanf(line, "cfg %63s %d", a, &i1) != 2 || SetCfg(a, i1))
				goto bad;
		} else if (!strcmp(cmd, "entry")) {
			if (sscanf(line, "entry %d %d", &i1, &i2) < 1)
				goto bad;
			entryState = (uint8_t)i1;
			entrySecs = (uint8_t)i2;
		} else if (!strcmp(cmd, "sq")) {
			if (sscanf(line, "sq %d", &i1) != 1)
				goto bad;
			sqOpen = i1;
		} else if (!strcmp(cmd, "nf")) {
			if (sscanf(line, "nf %d", &i1) != 1)
				goto bad;
			noiseFloor = (int8_t)i1;
		} else if (!strcmp(cmd, "freq")) {
			if (sscanf(line, "freq %d", &i1) != 1)
				goto bad;
			rxFreq.Frequency = (uint32_t)i1;
		} else if (!strcmp(cmd, "batt")) {
			if (sscanf(line, "batt %d", &i1) != 1)
				goto bad;
			gBatteryDisplayLevel = (uint8_t)i1;
		} else if (!strcmp(cmd, "key")) {
			if (sscanf(line, "key %63s %63s", a, b) < 1)
				goto bad;
			Key(KeyCode(a), !strcmp(b, "held"));
		} else if (!strcmp(cmd, "tick")) {
			if (sscanf(line, "tick %d", &i1) != 1)
				goto bad;
			while (i1-- > 0) {
				uptimeMs += 10u;
				ALERTUI_Tick10ms();
			}
		} else if (!strcmp(cmd, "draw")) {
			if (sscanf(line, "draw %63s", a) != 1)
				goto bad;
			ALERTUI_Draw();
			ok &= Dump(a);
		} else {
			goto bad;
		}
		(void)Row;
		continue;
bad:
		fprintf(stderr, "line %d: cannot read \"%s\"\n", n, line);
		return 2;
	}
	return ok ? 0 : 1;
}
