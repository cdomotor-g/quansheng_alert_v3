/* ALERT receiver: USB CDC text console - see alert_console.h.
 *
 * Commands: V2_SPEC section 7. The reply shapes and error codes the spec
 * leaves open are the ones docs/ALERT_SERIAL.md section 7 gives (alertterm.py
 * parses those), with two trailing fields added: the table's state after
 * STN INFO and INFO,stn.
 *
 * Sharing the port with the binary protocol. The USB driver writes every
 * byte the host sends into one 256-byte ring, VCP_RxBuf, and moves one write
 * index. Each reader keeps its own read index over it and never writes the
 * ring or anyone else's index: the binary parser (UART_IsCommandAvailable),
 * the K5Viewer ping when that is built, and this console (sRd). So:
 *
 * - The console cannot eat binary bytes: it only reads. The binary parser
 *   hunts for 0xAB 0xCD and skips everything else, and console text is
 *   printable ASCII, which never contains 0xAB.
 * - Binary bytes cannot become console commands. 0xAB starts a frame: the
 *   console reads 0xCD and the 16-bit size and skips size + 4 more bytes
 *   (payload, CRC, 0xDC 0xBA) without looking at them. The binary parser
 *   zeroes a frame once it has consumed it, so a frame the console reaches
 *   late reads as zeros, which are ignored. Any other byte outside printable
 *   ASCII (an arrow key's escape sequence, a K5Viewer keepalive, line noise)
 *   is dropped and spoils its line: a line with text in it is answered
 *   ERR,ARGS at its CR/LF, as every command gets its one final line, and one
 *   with none is silent. A client should start with a bare CR: empty lines
 *   are silent.
 *
 * The host sends one line and waits for its final OK/ERR: the ring has no
 * overflow check, and 256 bytes is under two STN W lines.
 *
 * Lines are at most 96 characters, upper-cased and with runs of spaces
 * collapsed as they arrive. STN W is the exception: once "STN W <off> " is
 * in, its hex digits are packed two to a byte into the same buffer, so a
 * 64-byte write (141 characters of text) fits.
 *
 * Nothing runs while a burst is on the air (see Busy()) or while the radio
 * transmits: outside the app this runs from app.c's 10 ms slice, which is
 * serviced during TX too, and an erase or a CRC pass here would hold PTT
 * release and the TX timeout up for as long. A LOG DUMP goes out a few
 * records per poll, so the receiver keeps running under it. The log's sector
 * pre-erase (ALERTLOG_Idle) is run from here too, once the squelch has been
 * shut for a second. CSV HDR and the LOG DUMP lines are alert.c's own
 * (ALERT_CsvHeader, ALERT_FormatRecord), so they match what the app sends.
 *
 * Copyright 2026 cdomotor-g. Apache-2.0, like the egzumer base it lives in.
 */
#ifdef ENABLE_ALERT

#include <stdarg.h>
#include <string.h>

#include "app/alert_console.h"
#include "app/alert_int.h"
#include "app/alert_log.h"
#include "app/alert_stn.h"
#include "app/dfu.h"
#include "driver/py25q16.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "helper/battery.h"
#ifdef ENABLE_USB
	#include "driver/vcp.h"
#endif

#ifndef BUILD_COMMIT
	#define BUILD_COMMIT "unknown"
#endif

#ifdef ENABLE_USB

// Text lines are at most 96 characters (V2_SPEC section 7). The buffer is a
// CSV line long because a LOG DUMP formats its lines in it: no input is read
// while a dump runs, and 104 bytes of static beat a 200-byte stack frame.
#define CON_TEXT       96u
#define CON_LINE       (ALERT_LINE_MAX + 1u)
#define DUMP_PER_POLL  8u

enum { M_TEXT = 0, M_JUNK, M_AB, M_LEN0, M_LEN1, M_SKIP };
enum { E_LONG = 1, E_HEX = 2, E_BYTE = 4 };

static char     sLine[CON_LINE];
static uint8_t  sLen;
static uint8_t  sData;          // STN W: where the packed bytes start; 0 = a text line
static uint8_t  sNib;           // STN W: 0x10 | a high nibble waiting for its low one
static uint8_t  sErr;           // E_*: what is wrong with the line so far
static uint8_t  sMode;          // M_*
static uint16_t sSkip;          // binary frame bytes still to skip
static uint16_t sRd;            // read index into VCP_RxBuf
static bool     sSqOpen;        // squelch as last seen, and since when
static uint32_t sSqEdge;
static uint32_t sDumpSeq;       // LOG DUMP in progress: next seq, and how many left
static uint32_t sDumpLeft;

static void Out(const char *s)
{
	ALERT_Emit(s);
}

// One formatted piece of output; the caller supplies any CRLF.
static void Say(const char *fmt, ...)
{
	char    b[104];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(b, sizeof(b), fmt, ap);
	va_end(ap);
	Out(b);
}

static void Ok(void)
{
	Out("OK\r\n");
}

// Reasons as docs/ALERT_SERIAL.md 7.2 lists them, plus NOTINAPP and FOREIGN.
static void Err(const char *why)
{
	Say("ERR,%s\r\n", why);
}

static char *Hex(char *d, const uint8_t *s, uint8_t n)
{
	static const char digits[] = "0123456789ABCDEF";
	while (n--) {
		*d++ = digits[*s >> 4];
		*d++ = digits[*s++ & 15u];
	}
	*d = 0;
	return d;
}

static uint8_t HexVal(char c)
{
	if (c >= '0' && c <= '9')
		return (uint8_t)(c - '0');
	if (c >= 'A' && c <= 'F')
		return (uint8_t)(c - 'A' + 10);
	return 0xFFu;
}

// A whole token as a u32: decimal, or hex after 0x (or always, base 16).
static bool Num(const char *s, uint32_t *v, uint8_t base)
{
	uint32_t x = 0;

	if (s[0] == '0' && s[1] == 'X') {
		base = 16;
		s += 2;
	}
	if (!*s)
		return false;
	for (; *s; s++) {
		const uint8_t d = HexVal(*s);
		if (d >= base || x > (0xFFFFFFFFu - d) / base)
			return false;
		x = x * base + d;
	}
	*v = x;
	return true;
}

// The next space-separated token ("" at the end); *p moves past it.
static char *Tok(char **p)
{
	char *t = *p, *e;

	while (*t == ' ')
		t++;
	for (e = t; *e && *e != ' '; e++)
		;
	if (*e)
		*e++ = 0;
	*p = e;
	return t;
}

static bool Is(const char *a, const char *b)
{
	return strcmp(a, b) == 0;
}

// ---------------------------------------------------------------------------
// settings: rows by name, spaces spelt '_' (SET CSV_OUT OFF)

static char Norm(char c)
{
	if (c == ' ')
		return '_';
	return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
}

static bool Same(const char *a, const char *b)
{
	while (*a && Norm(*a) == Norm(*b)) {
		a++;
		b++;
	}
	return Norm(*a) == Norm(*b);
}

static int8_t FindRow(const char *name)
{
	for (uint8_t r = 0; r < ALERT_SetCount(); r++)
		if (Same(ALERT_SetName(r), name))
			return (int8_t)r;
	return -1;
}

// GET,<NAME>,<value>
static void SayRow(uint8_t row)
{
	char        n[12], v[12];
	const char *s = ALERT_SetName(row);
	uint8_t     i;

	for (i = 0; s[i] && i < sizeof(n) - 1u; i++)
		n[i] = Norm(s[i]);
	n[i] = 0;
	ALERT_SetValue(row, v);
	Say("GET,%s,%s\r\n", n, v);
}

// Why ALERT_SetStep said no, from what alert_int.h says it refuses: the rows
// that act on the receiver, outside the app (CENSUS only ever goes up); the
// read-only row; a value at the end of its range.
static const char *Refusal(uint8_t row, int dir)
{
	const char *n = ALERT_SetName(row);

	if (Same(n, "MDM MODE"))
		return "READONLY";
	if (Same(n, "FREQ MHz") || Same(n, "SQL LEVEL") || (Same(n, "CENSUS") && dir > 0))
		return "NOTINAPP";
	return "RANGE";
}

// SET <name> <value|+|->. + and - are one step, as UP/DOWN in the settings
// view (SET CENSUS + re-runs the census); a value is set in one go by
// ALERT_SetTo, which checks all of it first and otherwise changes nothing.
static void CmdSet(char *p)
{
	char        *name = Tok(&p);
	const int8_t row  = FindRow(name);
	const char  *why;

	if (row < 0 || !*p) {
		Err(row < 0 ? "NAME" : "ARGS");
		return;
	}
	if ((p[0] == '+' || p[0] == '-') && !p[1]) {
		const int dir = (p[0] == '+') ? 1 : -1;
		if (!ALERT_SetStep((uint8_t)row, dir)) {
			Err(Refusal((uint8_t)row, dir));
			return;
		}
		ALERT_SettingsChanged();
	} else if ((why = ALERT_SetTo((uint8_t)row, p)) != NULL) {
		Err(why);
		return;
	}
	Ok();
}

static void CmdGet(char *p)
{
	const char  *name = Tok(&p);
	const int8_t row  = FindRow(name);

	if (*name && row < 0) {
		Err("NAME");
		return;
	}
	for (uint8_t r = 0; r < ALERT_SetCount(); r++)
		if (!*name || r == (uint8_t)row)
			SayRow(r);
	Ok();
}

// ---------------------------------------------------------------------------
// the other commands

static const char kHelp[] =
	"HELP,HELP\r\nHELP,INFO\r\nHELP,TIME [epoch]\r\nHELP,CSV HDR\r\nHELP,GET [name]\r\n"
	"HELP,SET name value|+|-\r\n"
	"HELP,LOG STAT|DUMP [n]|CLEAR YES|FORMAT FORCE\r\n"
	"HELP,STN INFO|GET id|BEGIN len crc32|W off hex|END|CLEAR YES|FORMAT FORCE\r\n"
	"HELP,SCREEN\r\nHELP,SPI READ addr len\r\nHELP,REBOOT\r\n";

static void CmdInfo(void)
{
	const uint32_t e = ALERT_Epoch();

	(void)ALERTLOG_Init();
	ALERTSTN_Init();
	Say("INFO,fw,%s\r\n", BUILD_COMMIT);
	Say("INFO,uptime_ms,%lu\r\n", (unsigned long)ALERT_UptimeMs());
	if (e)
		Say("INFO,epoch,%lu\r\n", (unsigned long)e);
	else
		Out("INFO,epoch,\r\n");
	Say("INFO,batt_mv,%u\r\nINFO,batt_pct,%u\r\n", gBatteryVoltageAverage * 10u,
	    BATTERY_VoltsToPercent(gBatteryVoltageAverage));
	Say("INFO,nf,%d\r\n", ALERT_NoiseFloor());
	Say("INFO,log,%s,%lu,%lu\r\n", ALERTLOG_State(), (unsigned long)ALERTLOG_Count(),
	    (unsigned long)ALERTLOG_Capacity());
	Say("INFO,stn,%s,%u,%s\r\n", ALERTSTN_Source(), ALERTSTN_Count(), ALERTSTN_State());
	for (uint8_t r = 0; r < ALERT_SetCount(); r++)
		SayRow(r);
	Ok();
}

static void CmdTime(char *p)
{
	const char *t = Tok(&p);
	uint32_t    e;

	if (*t) {
		if (!Num(t, &e, 10)) {
			Err("ARGS");
			return;
		}
		ALERT_SetEpoch(e);
	} else if ((e = ALERT_Epoch()) != 0) {
		Say("TIME,%lu\r\n", (unsigned long)e);
	} else {
		Out("TIME,\r\n");
	}
	Ok();
}

static void CmdLog(char *p)
{
	const char *t = Tok(&p), *a = Tok(&p);
	uint32_t    n;

	(void)ALERTLOG_Init();
	const uint32_t count = ALERTLOG_Count(), next = ALERTLOG_NextSeq();

	if (Is(t, "STAT")) {
		if (count)
			Say("LOG,%lu,%lu,%lu,%lu,%s\r\n", (unsigned long)count, (unsigned long)ALERTLOG_Capacity(),
			    (unsigned long)(next - count), (unsigned long)(next - 1u), ALERTLOG_State());
		else
			Say("LOG,0,%lu,,,%s\r\n", (unsigned long)ALERTLOG_Capacity(), ALERTLOG_State());
	} else if (Is(t, "DUMP")) {
		n = count;
		if (*a && !Num(a, &n, 10)) {
			Err("ARGS");
			return;
		}
		if (!next) {
			Err("NOLOG");
			return;
		}
		sDumpLeft = n < count ? n : count;
		sDumpSeq  = next - sDumpLeft;
		if (sDumpLeft)
			return;                      // ALERTCON_Poll sends the lines and the OK
	} else if (Is(t, "CLEAR") || Is(t, "FORMAT")) {
		const bool clear = (t[0] == 'C');
		if (!Is(a, clear ? "YES" : "FORCE")) {
			Err("CONFIRM");
			return;
		}
		if (clear && !next) {
			Err("NOLOG");
			return;
		}
		if (!(clear ? ALERTLOG_Clear() : ALERTLOG_Format(true))) {
			Err("FLASH");
			return;
		}
	} else {
		Err("ARGS");
		return;
	}
	Ok();
}

static void DumpStep(void)
{
	AlertRecord_t r;
	uint32_t      seq;

	for (uint8_t i = 0; i < DUMP_PER_POLL && sDumpLeft; i++, sDumpSeq++, sDumpLeft--) {
		// by seq, not by position: records logged since the dump began move
		// `back`, and one the ring has since overwritten is left out
		const uint32_t next = ALERTLOG_NextSeq();
		if (next > sDumpSeq && ALERTLOG_ReadBack(next - 1u - sDumpSeq, &r, &seq)) {
			ALERT_FormatRecord(sLine, "LOG", seq, &r);      // the DEC layout, alert.c's own
			Out(sLine);
		}
	}
	if (!sDumpLeft)
		Ok();
}

static void CmdStn(char *p)
{
	const char *t = Tok(&p), *a = Tok(&p), *b = Tok(&p);
	uint32_t    x, y;

	ALERTSTN_Init();
	if (Is(t, "INFO")) {
		const uint32_t c = ALERTSTN_Crc();
		if (c)
			Say("STN,%s,%u,%08lX,%s\r\n", ALERTSTN_Source(), ALERTSTN_Count(), (unsigned long)c, ALERTSTN_State());
		else
			Say("STN,%s,%u,,%s\r\n", ALERTSTN_Source(), ALERTSTN_Count(), ALERTSTN_State());
	} else if (Is(t, "GET")) {
		char    name[ALERT_NAME_MAX + 1];
		uint8_t k;
		if (!Num(a, &x, 10) || x > 0x1FFFu) {
			Err("ARGS");
			return;
		}
		ALERTSTN_Lookup((uint16_t)x, name, sizeof(name), &k);
		Say("STN,%lu,%s,%s\r\n", (unsigned long)x, name, ALERT_KindLabel(k));
	} else if (Is(t, "BEGIN")) {
		if (!Num(a, &x, 10) || !Num(b, &y, 16)) {
			Err("ARGS");
			return;
		}
		if (!ALERTSTN_UploadBegin(x, y)) {
			Err(Is(ALERTSTN_State(), "FOREIGN") ? "FOREIGN" : "RANGE");
			return;
		}
	} else if (Is(t, "END")) {
		if (!ALERTSTN_UploadLen()) {
			Err("STATE");
			return;
		}
		if (!ALERTSTN_UploadEnd()) {
			Err("CRC");
			return;
		}
	} else if (Is(t, "CLEAR") || Is(t, "FORMAT")) {
		const bool force = (t[0] == 'F');
		if (!Is(a, force ? "FORCE" : "YES")) {
			Err("CONFIRM");
			return;
		}
		if (!ALERTSTN_Format(force)) {
			Err("FOREIGN");
			return;
		}
	} else {
		Err("ARGS");
		return;
	}
	Ok();
}

// STN W <off> <hex>: the bytes arrive packed (see Feed).
static void CmdStnW(const char *off, const uint8_t *data, uint8_t n)
{
	const uint32_t len = ALERTSTN_UploadLen();
	uint32_t       x;

	if (sNib || !Num(off, &x, 10) || !n) {
		Err("ARGS");                     // no offset, no bytes, or an odd digit
		return;
	}
	if (!len || x > len || n > len - x) {
		Err("STATE");
		return;
	}
	if (!ALERTSTN_UploadWrite(x, data, n)) {
		// the first six bytes: a header that is not ASTB version 1; past them,
		// a write that skips ahead of the sectors reached so far
		Err(x < 6u ? "ARGS" : "STATE");
		return;
	}
	Ok();
}

// SCR,<row>,<256 hex>: the status line, then gFrameBuffer rows 0-6, 128
// column-major bytes each exactly as in RAM.
static void CmdScreen(void)
{
	char b[65];

	for (uint8_t row = 0; row < 8u; row++) {
		const uint8_t *src = row ? gFrameBuffer[row - 1u] : gStatusLine;
		Say("SCR,%u,", row);
		for (uint8_t i = 0; i < LCD_WIDTH; i += 32u) {
			Hex(b, src + i, 32);
			Out(b);
		}
		Out("\r\n");
	}
	Ok();
}

// SPI READ <addr> <len>: SPI,<addr hex>,<up to 32 bytes hex> lines. Reads only.
static void CmdSpi(char *p)
{
	const char *t = Tok(&p), *a = Tok(&p), *b = Tok(&p);
	uint32_t    addr, len;
	uint8_t     buf[32];
	char        hex[65];

	if (!Is(t, "READ") || !Num(a, &addr, 10) || !Num(b, &len, 10) || !len || len > 256u ||
	    addr >= 0x200000u || len > 0x200000u - addr) {
		Err("ARGS");
		return;
	}
	while (len) {
		const uint8_t n = len < sizeof(buf) ? (uint8_t)len : (uint8_t)sizeof(buf);
		PY25Q16_ReadBuffer(addr, buf, n);
		Hex(hex, buf, n);
		Say("SPI,%06lX,%s\r\n", (unsigned long)addr, hex);
		addr += n;
		len  -= n;
	}
	Ok();
}

static void Execute(void)
{
	char *p = sLine, *cmd;

	if (sErr) {
		Err((sErr & E_LONG) ? "TOOLONG" : "ARGS");
		return;
	}
	if (sData) {
		sLine[sData - 1u] = 0;           // "STN W <off>", then the packed bytes
		CmdStnW(sLine + 6, (const uint8_t *)sLine + sData, (uint8_t)(sLen - sData));
		return;
	}
	while (sLen && sLine[sLen - 1u] == ' ')
		sLen--;                          // runs were collapsed on arrival; the last one goes
	sLine[sLen] = 0;
	cmd = Tok(&p);

	if (Is(cmd, "HELP")) {
		Out(kHelp);
		Ok();
	} else if (Is(cmd, "INFO")) {
		CmdInfo();
	} else if (Is(cmd, "TIME")) {
		CmdTime(p);
	} else if (Is(cmd, "CSV")) {
		if (!Is(Tok(&p), "HDR")) {
			Err("ARGS");
			return;
		}
		ALERT_CsvHeader();
		Ok();
	} else if (Is(cmd, "SET")) {
		CmdSet(p);
	} else if (Is(cmd, "GET")) {
		CmdGet(p);
	} else if (Is(cmd, "LOG")) {
		CmdLog(p);
	} else if (Is(cmd, "STN")) {
		CmdStn(p);
	} else if (Is(cmd, "SCREEN")) {
		CmdScreen();
	} else if (Is(cmd, "SPI")) {
		CmdSpi(p);
	} else if (Is(cmd, "REBOOT")) {
		Ok();
		DFU_SafeReset();
	} else {
		Err("UNKNOWN");
	}
}

static void ResetLine(void)
{
	sLen  = 0;
	sData = 0;
	sNib  = 0;
	sErr  = 0;
}

// One byte off the port. True when it ends a line to execute.
static bool Feed(uint8_t b)
{
	switch (sMode) {
		case M_AB:
			sMode = (b == 0xCDu) ? M_LEN0 : M_JUNK;
			return false;
		case M_LEN0:
			sSkip = b;
			sMode = M_LEN1;
			return false;
		case M_LEN1:
			// The payload, its CRC and 0xDC 0xBA, whatever the size: only a
			// binary client sends 0xAB 0xCD, and its payload is never text,
			// even in a frame too big for the binary parser to take.
			sSkip = (uint16_t)(sSkip | (b << 8));
			sSkip = (uint16_t)(sSkip < 0xFFFBu ? sSkip + 4u : 0xFFFFu);
			sMode = M_SKIP;
			return false;
		case M_SKIP:
			if (--sSkip == 0)
				sMode = M_TEXT;
			return false;
		default:
			break;
	}

	if (b == '\r' || b == '\n') {
		// every error comes with text, so a line with none is silent: a bare
		// CR, or nothing but stray bytes
		const bool go = sMode == M_TEXT && sLen;
		sMode = M_TEXT;
		if (!go)
			ResetLine();
		return go;
	}
	if (b == 0)
		return false;                    // a frame the binary parser has consumed
	if (b == 0xABu) {
		ResetLine();
		sMode = M_AB;
		return false;
	}
	if (sMode == M_JUNK)
		return false;
	if (b == '\t')
		b = ' ';
	if (b == 8u || b == 0x7Fu) {         // backspace, for people at a terminal
		if (sLen && !sData)
			sLen--;
		return false;
	}
	if (b < ' ' || b > '~') {
		sErr |= E_BYTE;
		return false;
	}
	if (b >= 'a' && b <= 'z')
		b = (uint8_t)(b - 32u);

	if (sData) {
		const uint8_t v = HexVal((char)b);
		if (b == ' ')
			return false;
		if (v > 15u) {
			sErr |= E_HEX;
		} else if (!sNib) {
			sNib = (uint8_t)(0x10u | v);
		} else {
			if (sLen < CON_LINE)
				sLine[sLen++] = (char)(((sNib & 15u) << 4) | v);
			else
				sErr |= E_LONG;
			sNib = 0;
		}
		return false;
	}
	if (b == ' ') {
		if (!sLen || sLine[sLen - 1u] == ' ')
			return false;               // leading, or a run
		if (sLen > 6u && memcmp(sLine, "STN W ", 6) == 0 && !memchr(sLine + 6, ' ', sLen - 6u)) {
			sLine[sLen++] = ' ';
			sData = sLen;
			return false;
		}
	}
	if (sLen >= CON_TEXT) {
		sErr |= E_LONG;
		return false;
	}
	sLine[sLen++] = (char)b;
	return false;
}

// Inside the app a burst must never wait on a USB line or a flash write, so
// nothing runs while the squelch is open or for 100 ms after it shuts (the
// sampler drains, BurstFinish decodes). After 2 s open there is nothing left
// to protect: a voice conversation, or - outside the app, where
// ALERT_SquelchOpen keeps reporting the state the app left behind - nothing.
static bool Busy(uint32_t now)
{
	const bool open = ALERT_SquelchOpen();

	if (open != sSqOpen) {
		sSqOpen = open;
		sSqEdge = now;
	}
	return now - sSqEdge < (open ? 2000u : 100u);
}

void ALERTCON_Poll(void)
{
	const uint32_t now = ALERT_UptimeMs();

	// Transmitting (outside the app: PTT leaves it): input waits in the ring
	if (DFU_Transmitting() || Busy(now))
		return;
	if (sDumpLeft) {
		DumpStep();
		return;
	}
	// the log's pre-erase, a second into a quiet spell
	if (!sSqOpen && now - sSqEdge >= 1000u)
		ALERTLOG_Idle();

	// the USB driver can leave its write index at 256, the end of the ring
	const uint16_t w = (uint16_t)(VCP_RxBufPointer % VCP_RX_BUF_SIZE);
	while (sRd != w) {
		const uint8_t b = VCP_RxBuf[sRd];
		sRd = (uint16_t)((sRd + 1u) % VCP_RX_BUF_SIZE);
		if (Feed(b)) {
			Execute();
			ResetLine();
			break;                       // one command per poll
		}
	}
}

#else

void ALERTCON_Poll(void)
{
	// no USB CDC in this build: nothing to listen to
}

#endif // ENABLE_USB

#endif // ENABLE_ALERT
