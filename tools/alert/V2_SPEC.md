# ALERT receiver v2 — product spec and implementation contract

Written 2026-10-01 from the owner's requests after X1 proved the PA4 audio route
(22 of 22 bursts decoded). Binding on every implementer. Where this file and the
code disagree, this file wins; where this file is silent, match the surrounding
code and say what you chose.

## 0. Ground truth to preserve

- Decoding runs on the PA4 ADC route (`App/app/alert_adc.c`), census choice
  `PA4B pa=0`. Mid-bit sampling (phase 16) must stay. 100% decode on air must
  survive every change; the integrator verifies this on the radio.
- Hands-off DFU (`App/app/dfu.c`, uart.c 0x05E0/0x05E1, the linker NOINIT cell,
  the trampoline) must keep working in every build. Never remove or alter it
  unless a finding proves it wrong.
- CI greps the binary for the literal strings `ALERT SETTINGS`, `SQ GATE`,
  `MDM MODE`. `.github/workflows/main.yml` cannot be pushed from here (token
  lacks `workflow` scope), so those strings must remain in the image.
- Display: 128 px status line (`gStatusLine`) + main rows 0..5 of `gFrameBuffer`
  (row 6 is under the bezel: never use it). Small font = 7 px per char,
  18 chars per row. Text on the main screens uses the small font; numbers and
  the notes may use the bigger ones §3 names (revised 2026-11-20).
- RAM is 16 KB with ~4.8 KB free for stack before this work. New statics:
  at most ~1.5 KB in total. No malloc. Flash: must fit in 118 KB.

## 1. Phase A — strip and restructure (one implementer, sequential)

Remove the BK4819 FSK sweep entirely: the Arr_t tables, Phase 1/2/3, scoring,
adoption, ring-buffer capture, `L C H Y F Z G` lines, host hooks 0x0A01-0x0A03
(and their uart.c routing, radio.py subcommands), MODE/SWEEP/ADOPT/BIT REV
settings, VIEW_RAW. The FSK engine is not armed at all any more: the receiver is
set up as normal RX with AF = FM (what ArrArm's prologue did minus the FSK
registers). Keep: census (but skip it at entry when a persisted choice exists
and was confirmed; setting CENSUS re-runs it), ADC burst start/stop, decode
(`ALERT_ScanBitsGated`, both senses, gate 12), history, voice, settings, DFU
watchdog kicks, autostart.

Split into modules (all under App/app/, all in App/CMakeLists.txt ENABLE_ALERT):

| File | Owns |
|---|---|
| `alert.c` | app entry/loop, keys dispatch, burst capture → decode → record, NF averaging, speaker mode, CSV emission, cfg load/store |
| `alert_int.h` | shared internal types and APIs below (no public use) |
| `alert_ui.c` | every Draw* function, header, list, detail view, settings view, marquee |
| `alert_log.c/.h` | SPI-flash ring log (stub in Phase A: `ALERTLOG_Init` returns false) |
| `alert_stn.c/.h` | station lookup: SPI table if valid, else built-in (Phase A: built-in only, new API) |
| `alert_console.c/.h` | text console over USB CDC (Phase A: stub) |

Phase A output must build green and behave on air exactly as today minus the
sweep (ALERT and D lines kept for now).

## 2. Shared types (alert_int.h) — written in Phase A, frozen for Phase B

```c
typedef struct {            // one decoded frame; 24 bytes, also the log payload
	uint32_t epoch;          // UTC seconds if the clock was set, else 0
	uint32_t uptime_ms;      // ms since boot at squelch close
	uint16_t boot;           // boot counter (persisted by the log), 0 if no log
	uint16_t id;             // 13-bit ALERT address
	uint16_t value;          // 11-bit raw value
	uint8_t  fmt;            // ALERT_FMT_*
	uint8_t  flags;          // <0> pol STD, <1> inverted sense, <2> table station, <7:4> frame index in burst
	int8_t   rssi;           // dBm, peak during the burst
	int8_t   nf;             // dBm, averaged noise floor at the time (see §4)
	uint16_t burst_ms;       // squelch-open duration
	uint32_t payload;        // the 32 decoded data bits (ALERT_DecodePayload32 input)
} AlertRecord_t;
```

APIs (signatures fixed; implementers may add static helpers only):

```c
// alert_stn.h
bool        ALERTSTN_Lookup(uint16_t id, char *name, uint8_t name_max, uint8_t *kind); // name "" if unknown
const char *ALERTSTN_Source(void);          // "BUILTIN MegaNet:95f6f8d" or "SPI MegaNet:xxxxxxx"
void        ALERTSTN_Init(void);            // validates the SPI table once at app entry
// console-facing upload API
bool ALERTSTN_UploadBegin(uint32_t len, uint32_t crc32);
bool ALERTSTN_UploadWrite(uint32_t off, const uint8_t *p, uint16_t n);
bool ALERTSTN_UploadEnd(void);              // verify CRC, activate
bool ALERTSTN_Clear(void);                  // erase, revert to built-in
uint16_t ALERTSTN_Count(void);

// alert_log.h
bool     ALERTLOG_Init(void);               // false: log unusable (region not blank and no magic)
bool     ALERTLOG_Append(const AlertRecord_t *r);   // assigns nothing; seq is internal
uint32_t ALERTLOG_Count(void);              // records currently held
uint32_t ALERTLOG_Capacity(void);
bool     ALERTLOG_ReadBack(uint32_t back, AlertRecord_t *r, uint32_t *seq); // back 0 = newest
uint16_t ALERTLOG_Boot(void);               // this boot's counter
bool     ALERTLOG_Clear(void);
bool     ALERTLOG_Format(bool force);       // force: also over foreign data (console only)
const char *ALERTLOG_State(void);           // "OK", "OFF", "FOREIGN", "ERR"

// alert_console.h
void ALERTCON_Poll(void);                   // called every loop pass inside the app AND from app.c's slice outside it

// alert_ui.c (declared in alert_int.h)
void ALERTUI_Draw(void);                    // draws the current view into the buffers and blits
void ALERTUI_Key(KEY_Code_t key, bool held); // UI-level keys (scroll, view switching)
void ALERTUI_Tick10ms(void);                // marquee, idle snap-back

// alert.c exports for the others (declared in alert_int.h)
extern AlertCfg_t gAlertCfg;                // §5 settings, defined in alert.c
int8_t   ALERT_NoiseFloor(void);            // averaged NF, dBm
int8_t   ALERT_Rssi(void);                  // instantaneous
bool     ALERT_SquelchOpen(void);
uint32_t ALERT_Epoch(void);                 // 0 if unset
void     ALERT_SetEpoch(uint32_t epoch);
uint8_t  ALERT_HistoryCount(void);          // RAM ring, newest first
const AlertRecord_t *ALERT_History(uint8_t back);
void     ALERT_Emit(const char *line);      // USB/UART line out (the DbgSend path)
void     ALERT_SettingsChanged(void);       // persist cfg
```

## 3. Screens (Phase B, alert_ui.c)

Revised 2026-11-20 after the owner found the three-entry list crowded and
could not tell which end of it was the newest: the readings now come in four
layouts (LAYOUT, §5), the default showing one reading big, and the settings
work like the radio's own menu. `tools/alert/screens.py` draws every screen
below on a computer with alert_ui.c itself; `tools/alert/screens/` holds the
pictures.

Small font = 7 px a character, 18 to a row; numbers may also use the radio's
frequency digits (13 px wide, 16 tall: two rows) or the same at twice the
size (24 px, 32 tall: four rows), and the notes the big letters (8 px, 16
tall). Nothing is ever drawn on row 6. The selected reading - the one the
detail view opens and KEY_0 says - is always the first one shown, and in
every layout but CLASSIC its name is in a dark bar (light bold letters).

**Header (status line)**: `ALERT` · RX marker (`RX` while squelch open, blank
otherwise) · averaged noise floor `NF-112` · log indicator (`L` when logging,
nothing otherwise) · battery icon at the right edge (reuse `UI_DrawBattery` into
`gStatusLine`, same bitmap as the main screen uses, 13 px). Nothing overlaps.

**Main view, LAYOUT = BIG (default)**: the selected reading alone.

```
row 0:    ▓STRATHPINE GYMPIE▓   name in the bar (marquee if longer than 18)
rows 1-4:       18  LEVEL       value in the double-size digits, its kind as
                                its unit on the baseline (spelt out when it
                                fits, else RAIN/LVL/BATT/REP), centred
row 5:    34s AGO      NEWEST   age; NEWEST at the top, else "N NEWER"
```

**LAYOUT = TWO**: two readings, three rows each, a 1 px line between them.

```
row a:    ▓STRATHPINE GYMPIE▓   name (bold; the selected one in the bar)
rows b-c:   18  LEVEL           value in the frequency digits, right-aligned
                34s AGO         in a four-digit field; kind and age beside it
```

**LAYOUT = CLASSIC**: the first layout, three entries, two rows each, newest at the top.

```
row a:  BEACHMERE ST    4m        name left (13 max, or marquee if longer), age right
row b:  4133 LVL 22    -13        id, kind label (RAIN/LVL/BATT/REP/-), value, rssi right
```

**LAYOUT = LIST**: six readings, one row each: name (9), value, age; the
selected row in the bar.

In every layout:

- Age: `12s`, `4m`, `3h`, `2d` (from uptime; epoch not needed), with ` AGO`
  where there is room.
- Unknown station: the name shows as `ID 4142 ?` (BIG: `ID 4142 (UNKNOWN)`,
  and no kind) unless UNKNOWN=HIDE (then skipped).
- UP/DOWN scroll one entry; a 1-px scroll bar on the rightmost column of rows
  0..5 shows position within history. History comes from the log when it is
  usable (thousands back), else from the RAM ring (16).
- When scrolled away from the top and a new decode arrives, keep the position
  and show `+N` in place of the RX marker; EXIT or 30 s idle snaps back to top.
- Empty history: rows 0-1 the frequency in the frequency digits with `MHz`
  (four decimals: 12.5 kHz channels need them), row 2 `LISTENING` (bold;
  `SETTLING 7` or `CENSUS` while starting), row 3 `SQL 3.0` and `LOG ON` /
  `LOG OFF` / `NO LOG`, row 4 `BURSTS 0  DEC 0`, row 5 audio route
  (`AUD PA4B OK`).

**Notes**: a key that changes something on the main or detail view says what
it did for 1.5 s, in big light letters in a dark bar over rows 4-5:
`SPEAKER: SQL`, `LAYOUT: TWO`, `VOICE: ON`, or `NO VOICE` in a build without
the voice prompts.

**Detail view** (KEY_1; EXIT back): the selected entry:
full name in the bar (marquee if > 18), `ID 4133 LEVEL`, `VAL 22 RAW 0x016`,
`RSSI -13 NF -112`, `FADE +87dB` (see §4), `ABF STD INV 12s AGO`.

**Marquee**: a name longer than its field scrolls one character every 300 ms
with a 1 s pause at each end; only the top/selected entry scrolls.

**Settings** (MENU): the title `ALERT SETTINGS`, then five rows of
`NAME      VALUE`, in §5's order. As the radio's own menu: UP/DOWN move the
bar from row to row (wrapping round); MENU starts changing the row, which
lights only its value and turns the title into `MENU=OK EXIT=UNDO`; UP/DOWN
change it, taking effect at once (FREQ retunes, SQL LEVEL applies); MENU
keeps the change, EXIT puts back the value it had when MENU was pressed.
EXIT from the rows saves and returns. Held UP/DOWN repeat (while changing,
only for numbers). The screen leaves out MDM MODE (always ADC) and, in a
build without voice prompts, VOICE; the console has every row.

Keys on the main view: UP/DOWN scroll (UP newer, DOWN older) · MENU settings ·
1 detail · 2 next LAYOUT · STAR voice toggle · F cycles SPEAKER · 0 replays
the selected entry by voice (as today) · EXIT back to the newest, then
leaves the app. Squelch level moves to settings only.

## 4. Measurements (alert.c)

- **Noise floor (NF)**: exponential average of RSSI sampled every 10 ms while
  the squelch is closed, skipping 500 ms after each squelch close; time
  constant ~30 s (alpha 1/3000 in fixed point, e.g. Q8 accumulator). Seed from
  the first 1 s of samples. Report integer dBm.
- **RSSI of a transmission**: peak RSSI while the squelch was open.
- **Estimated sensitivity**: `sens = NF + SNR_REQ` where SNR_REQ is a setting
  (default 12 dB): the signal needs roughly that much above the local noise
  to decode. This is an estimate and is documented as such.
  Also track `min_ok`: the weakest peak RSSI that has decoded this boot.
- **Fade margin**: `fade = rssi - sens` (dB).

## 5. Settings (persisted in gEeprom.ALERT_CFG[5]; bytes 1 and 3 are free now)

In this order on the screen and in the console's GET (revised 2026-11-20:
what is changed in the field first, the diagnostics last; it used to start
with VOICE and end with FREQ MHz, SQL LEVEL).

| Row name | Values | Default |
|---|---|---|
| FREQ MHz | 12.5 kHz steps, four decimals (`151.5125`) | as today |
| SQL LEVEL | as today | as today |
| SPEAKER | OFF / SQL / ON | **SQL** — speaker path on only while the squelch is open |
| LAYOUT | BIG / TWO / CLASSIC / LIST (§3), in ALERT_CFG[3] | **BIG** |
| VOICE | OFF/ON (on the screen only in builds with voice prompts) | OFF |
| UNKNOWN | SHOW/HIDE | SHOW |
| LOG | OFF/ON | **ON** (only effective if the region is usable) |
| CSV OUT | OFF/ON | **ON** |
| CONFIRM | OFF / 2 COPIES | OFF |
| SQ GATE | OFF/ON | ON (keep the name) |
| SNR REQ | 6..20 dB | 12 |
| CENSUS | shows result; UP re-runs | — |
| DEBUG | OFF/ON | OFF (D heartbeat and raw A lines) |
| MDM MODE | ADC (read-only; keep the name; console only) | ADC |

SPEAKER=SQL: `AUDIO_AudioPathOn()` on squelch open, `Off()` on close, unless
the census choice needs PA8 (then the ADC module owns PA8 during the burst, as
today). Leaving the app restores the normal radio audio state. The integrator
must confirm on air that SQL mode does not reduce the decode rate.

## 6. Serial output (CSV over USB CDC and the UART; CSV OUT = ON)

Every line: ASCII, CRLF, ≤ 200 chars, comma-separated, first field = record
type. No spaces around commas. Empty field = unknown. Station names have commas
replaced by spaces.

| Type | When | Fields after the type |
|---|---|---|
| `HDR` | boot, `CSV HDR`, every 200 records | `fw,<hash>,schema,2` then one `HDR,<type>,<field list>` line per type below |
| `DEC` | each decoded frame | `seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,frame,rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin` |
| `BST` | each qualifying burst, decoded or not | `seq,epoch,uptime_ms,peak,nf,burst_ms,nframes,nbits,bits_hex` (bits_hex: first 60 bytes of the demodulated 300-baud bits, MSB-first) |
| `STA` | every 10 s | `epoch,uptime_ms,nf,rssi,sq,batt_mv,batt_pct,bursts,decodes,min_ok,log_state,log_count,log_cap,stn_src` |
| `EVT` | boot, census, setting change, errors | `epoch,uptime_ms,<code>,<detail>` (codes: BOOT, CENSUS, SET, LOG, STN, CLOCK) |

`seq` is the log sequence number when logging, else a per-boot counter.
`eng`/`unit` follow FormatValue (BATT → `13.6`,`V`; RAIN → count,`tips`;
LEVEL → raw,``). `payload_bin` is 32 chars of 0/1.

The old `ALERT,` line and the X1 debug lines are dropped from the default
output; DEBUG=ON restores `D` and `A`.

## 7. Text console (alert_console.c)

A line-based console on the same USB CDC port, usable from any terminal
(`python -m serial.tools.miniterm COM5 115200`). It must coexist with the binary
0xABCD protocol used by hotflash.py and serialtool (read uart.c/vcp.c; the
binary parser only acts on 0xAB 0xCD framing — give the console its own read
index over the same RX ring, or an equivalent that cannot steal binary frames).
Lines end CR or LF; case-insensitive; max 96 chars. Responses: `OK,...`,
`ERR,<reason>`, or typed data lines, then always a final `OK` or `ERR`.

| Command | Effect |
|---|---|
| `HELP` | list commands |
| `INFO` | fw hash, uptime, epoch, battery, NF, log state/count/capacity, station source/count, settings |
| `TIME` / `TIME <epoch>` | get / set UTC clock (RAM; lost on reboot) |
| `CSV HDR` | re-emit the HDR block |
| `SET <name> <value>` / `GET [name]` | settings by row name (spaces → `_`, e.g. `SET CSV_OUT OFF`) |
| `LOG STAT` | `LOG,count,capacity,oldest_seq,newest_seq,state` |
| `LOG DUMP [n]` | newest n (default all) as `LOG,<same fields as DEC>` lines, oldest first |
| `LOG CLEAR YES` | erase the log |
| `LOG FORMAT FORCE` | take over a non-blank region (explicit) |
| `STN INFO` | source, count, crc |
| `STN BEGIN <len> <crc32hex>` / `STN W <off> <hex≤64 bytes>` / `STN END` | upload a station blob (§8) |
| `STN CLEAR YES` | revert to built-in table |
| `STN GET <id>` | `STN,<id>,<name>,<kind>` |
| `SCREEN` | 8 lines `SCR,<row 0..7>,<256 hex chars>`: status line then gFrameBuffer rows 0..6, 128 bytes each, column-major bytes exactly as in RAM |
| `SPI READ <addr> <len≤256>` | hex dump, read-only, any address |
| `REBOOT` | DFU_SafeReset |

The console must be serviced both inside the ALERT app loop and in normal
radio operation (app.c's 10 ms slice).

## 8. SPI flash regions (alert_log.c / alert_stn.c)

Known users of the 2 MB PY25Q16: 0x000000-0x011FFF settings/calibration/logo
(eeprom_compat), 0x00A028/0x00A148 (fm/spectrum), voice clips below 0x0C9000
and their index at 0x14C000-0x14CFFF, RX/TX log 0x1E0000-0x1E7FFF, foxhunt cfg
(find its address). The implementer must grep every PY25Q16 address in the tree
and confirm the regions below are unused by any code, then:

- **Log**: 0x1B0000-0x1DFFFF (192 KB = 48 sectors; moved 2026-10-01: this radio's
  factory voice clips occupy 0x14D000-0x196FFF, found with SPI READ). Records 32 bytes
  (AlertRecord_t 24 B + seq u32 + crc16 + marker): 128 per sector, 12,288
  total. Ring: find head at init by scanning each sector's first/last seq
  (binary search acceptable); erase the next sector before crossing into it.
  A sector header (first 16 B) may hold magic `ALOG`, version, boot counter.
  At init: if the region has no `ALOG` magic and is not all 0xFF, state
  FOREIGN and do nothing until `LOG FORMAT FORCE`. Commit order must make a
  torn write detectable (write the record, then its marker byte).
- **Stations**: 0x1A0000-0x1AFFFF (64 KB; the whole of MegaNet is ~58 KB). Blob:
  `magic "ASTB", u16 version=1, u16 count, u32 names_len, u32 crc32(of everything after this header),
  char source[16]`, then `count` × `{u16 base_id, u16 kinds, u32 name_off}` sorted by base_id, then
  NUL-terminated names (≤ 40 chars, uppercase, commas → spaces). Kinds packing
  as in alert_stations_gen.h. Lookup = binary search over base_id with
  base..base+4 coverage, exactly like the built-in. Same FOREIGN rule.
- `tools/alert/gen_stations.py --blob out.bin` writes this blob (filter as
  usual, but full names, no 13-char cut); with `all` the whole of MegaNet
  (~2,600 sites) must fit.

## 9. Host tools and docs (Phase B)

- `docs/ALERT_SERIAL.md`: the authoritative interface document for humans and
  agents — ports (VID 36B7, CDC, any baud), line format, every record type and
  field with units and examples, console commands with example sessions, error
  codes, the binary 0xABCD commands that remain (0x0514 hello, 0x05DD reboot,
  0x05E0/0x05E1 DFU), timing, and a "writing a client" section.
- `tools/alert/alertterm.py`: reference client. Subcommands: `live` (pretty
  table of DEC/STA, also appends raw lines to a file), `log-download
  --csv out.csv`, `log-clear`, `stations-upload [--filter F | --all]` (builds
  the blob with gen_stations and uploads with STN), `stations-clear`,
  `screenshot out.png` (from SCREEN; pure-Python PNG writer, no PIL
  dependency), `time-sync`, `set NAME VALUE`, `info`, `console` (interactive).
  Finds the port by VID 36B7. Sets the clock automatically on connect.
- Update `radio.py`, `alertmon.py`, `sweep_judge.py`/`capture_stats.py` only as
  needed to not break; the sweep tools may be retired with a note.
- Tests: `tools/alert/test_console.py` (parser/formatter tests against recorded
  lines), keep `check_layout.py` meaningful for the new screens (18 chars, no
  row 6, no overlap), `test_decode.c` still passes.
