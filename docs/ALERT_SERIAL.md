# ALERT receiver serial interface

The authoritative description of what the ALERT receiver firmware (Quansheng
UV-K5 V3, F4HWN fork, `ENABLE_ALERT`) sends and accepts on its USB serial
port. It is written for people and for software agents. A client built from
this page alone should work.

- **Schema 2.** The firmware reports the schema it speaks in its `HDR` block
  (section 4).
- **Sources.** `tools/alert/V2_SPEC.md` sections 6-8 are the binding product
  spec. This page restates them with types, units, ranges and examples, and
  adds what the code fixes: the USB descriptor, DTR, the binary frames.
- **The dagger mark (†).** Where the spec leaves a detail open, this page
  defines it and marks it †. Section 13 lists every † item in one place, so
  the firmware can be checked against it.
- **Reference client.** `tools/alert/alertterm.py` implements everything
  here. `tools/alert/test_console.py` parses every example line on this page
  with that client, so the examples are kept correct.

Contents: 1 Quick start · 2 Connecting · 3 Line format · 4 Versioning ·
5 Records · 6 Timing · 7 Console · 8 Station upload · 9 Screen capture ·
10 Binary 0xABCD commands · 11 Writing a client · 12 alertterm.py ·
13 Gaps filled here.

## 1. Quick start

```bash
pip install pyserial
python tools/alert/alertterm.py live          # table of decodes as they arrive
python tools/alert/alertterm.py info          # firmware, clock, battery, log, settings
python -m serial.tools.miniterm COM5 115200   # or any terminal: type HELP
```

A session in a terminal, with the radio on and the ALERT app running:

```
> TIME 1790843760
OK
EVT,1790843760,61020,CLOCK,SET
DEC,1041,1790843886,187340,12,2088,MARBURG,BATT,143,14.3,V,ABF,STD,1,0,-20,-121,-109,89,412,16067B23,00010110000001100111101100100011
```

In every example on this page, `> ` marks a line the host sends. Every other
line is sent by the radio.

## 2. Connecting

### Finding the port

| Property | Value |
|---|---|
| USB class | CDC ACM (a virtual COM port) |
| VID | `0x36B7` (`App/usb/usbd_cdc_if.c`) |
| PID | `0xFFFF` |
| Windows | `COMn` |
| Linux | `/dev/ttyACMn`; the user needs the `dialout` group or equivalent |
| macOS | `/dev/cu.usbmodem*` |

Find the port by VID, not by name. In Python:
`[p.device for p in serial.tools.list_ports.comports() if p.vid == 0x36B7]`.

The stock bootloader (DFU mode) enumerates with the same VID. To tell the two
apart:

- The firmware sends text lines.
- The bootloader sends only binary frames, the beacons `0x0518` and `0x0530`
  (section 10). If you see those, the radio is in DFU. The port belongs to the
  flasher (`tools/hotflash.py`).

### DTR is required

The firmware sends only while the host holds DTR asserted.
`cdc_acm_data_send_with_dtr()` checks a flag that the host's
SET_CONTROL_LINE_STATE sets and clears.

A send the host does not collect times out after a 100,000-iteration spin,
which is a few milliseconds. The firmware then clears that flag and treats the
host as gone. From that point the radio is silent, console replies included,
until the host asserts DTR again. So a client must:

1. Assert DTR when it opens the port. To be sure a fresh
   SET_CONTROL_LINE_STATE reaches the device, set DTR false, wait 50 ms, then
   set it true. Setting it true while it is already true may send nothing.
2. Keep reading all the time, even when it has nothing to say.
3. Toggle DTR again after about 25 s without a byte. With `CSV_OUT` on, an
   `STA` line arrives every 10 s, so 25 s of silence means the flag was
   cleared.

Binary replies to 0xABCD frames (the `0x0515` hello reply) go out through an
asynchronous send that does not check DTR. The text lines, including the `K`
lines, do check it.

### Baud rate, the UART, sharing the port

- **Baud rate.** The radio does not use the baud rate you set. Any value
  works, and 115200 is conventional.
- **UART.** The record lines of section 5 also go out on the radio's UART:
  the K1 speaker/mic jack at 38400 8N1, through a USB-serial cable, no DTR.
  The console (section 7) is USB only.
- **One program per port.** Windows lets one process open a COM port. Stop
  `tools/alert/radio.py log`, CHIRP or any terminal before running another
  client. On Linux, ModemManager may probe a new ttyACM port with `AT`
  commands; the console answers them with `ERR,UNKNOWN`, which does no harm.
- **Text and binary share the port.** The console and the binary 0xABCD
  protocol (section 10) use the same port. The console never consumes a
  binary frame, and the binary parser ignores text. Do not have a text command
  and a binary frame in flight at the same time.

## 3. Line format

Every line the radio sends:

- is ASCII, ends in CR LF, and is split into fields by commas;
- has the record type as its first field;
- has no spaces around the commas;
- uses an empty field to mean unknown or not applicable (`STA` with no clock
  set starts `STA,,`);
- replaces any comma in a station name with a space;
- is at most 200 characters for the section 5 records. Console reply lines
  may be longer: an `SCR` line is 262.

### Four classes of line

Classify each received line, in this order:

| Class | Test | Lines |
|---|---|---|
| final | first field is `OK` or `ERR` | `OK`, `OK,<detail>`, `ERR,<reason>`: the end of one console command |
| record | has a comma and the first field is `HDR` `DEC` `BST` `STA` or `EVT` | the stream (section 5), sent by the radio on its own |
| debug | no comma, has a space | diagnostics: `D` heartbeat and `A` raw bits (DEBUG=ON), `B` boot line, `K` DFU acks, others. Log them; do not parse them. |
| data | anything else | console reply lines: `LOG`, `STN`, `SCR`, `GET`, `INFO`, `TIME`, `SPI`, `HELP` |

Records and debug lines can arrive at any moment, including between the reply
lines of a console command. A client routes them to its stream handler and
goes on waiting for the command's final line.

The pre-V2 `ALERT,<id>,<value>,<fmt>,<rssi>,<name>` line is gone from the
default output. A client may treat it as a record and ignore it.

## 4. Versioning

The `HDR` block (section 5.1) starts with `HDR,fw,<hash>,schema,2`. Then comes
one `HDR,<type>,<field list>` line per record type. The rules:

- Within schema 2, fields are only ever **appended** at the end of a record.
  New record types, new `EVT` codes, new console commands and new reply types
  may appear.
- Removing, renaming, reordering or changing the unit or meaning of a field
  changes the schema number.
- A client maps fields **by name** from the latest `HDR,<type>` line. It
  ignores fields and record types it does not know. It warns, or stops, on a
  schema number other than the one it was written for.
- Until an `HDR` block arrives, use the field lists on this page. Send
  `CSV HDR` after connecting to get the block at once: the one sent when the
  app started is lost if no host had DTR up then.
- `fw` is the firmware's git hash (`git rev-parse --short HEAD` at build
  time). It identifies the build, not the interface.
- A console command the build does not have answers `ERR,UNKNOWN`.

## 5. Records

These lines are sent while `CSV_OUT` is ON (the default). DEC, BST and STA
come from the ALERT app, so they flow only while the app runs. The console
(section 7) answers both inside the app and in normal radio operation.

### 5.1 HDR: the schema

Sent when the ALERT app starts, on `CSV HDR`, and again after every 200
record lines, so a client that joins mid-stream learns the schema within 200
lines. The automatic ones follow `CSV_OUT`; `CSV HDR` answers either way.

```
HDR,fw,4d06107f,schema,2
HDR,DEC,seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,frame,rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin
HDR,BST,seq,epoch,uptime_ms,peak,nf,burst_ms,nframes,nbits,bits_hex
HDR,STA,epoch,uptime_ms,nf,rssi,sq,batt_mv,batt_pct,bursts,decodes,min_ok,log_state,log_count,log_cap,stn_src
HDR,EVT,epoch,uptime_ms,code,detail
```

### 5.2 DEC: one decoded frame

One line per frame delivered: after de-duplication and the CONFIRM rule, and
excluding unknown addresses while UNKNOWN=HIDE. A burst can carry several
frames. They share `uptime_ms` and are numbered by `frame`.

| # | Field | Type | Unit | Range | Meaning |
|---|---|---|---|---|---|
| 1 | seq | u32 | | ≥ 1 | Log sequence number while LOG=ON and the log is usable; otherwise a per-boot counter starting at 1 † |
| 2 | epoch | u32 | s | empty, or Unix time | UTC at squelch close. Empty when the clock was not set (TIME, section 7). Treat 0 as empty. |
| 3 | uptime_ms | u32 | ms | | Time since boot at squelch close, in 10 ms steps. The burst's BST line has the same value. |
| 4 | boot | u16 | | empty, or 1-65535 | Boot counter kept by the log; empty when there is no usable log |
| 5 | id | int | | 0-8191 | ALERT address (13 bits) |
| 6 | name | text | | ≤ 40 chars | Station name: uppercase, commas replaced by spaces, empty when the id is not in the table. Built-in table names are cut to 13 characters; uploaded names are full. |
| 7 | kind | text | | `RAIN` `LVL` `BATT` `REP` `SNSR` `CHK` or empty | What the station table says this address reports; empty when unknown † |
| 8 | value | int | raw | 0-2047 | The 11-bit value as sent. 2047 is full scale (over range, or a dead sensor). |
| 9 | eng | text | | | The value in engineering units, as the radio shows it: BATT = value / 10 to one decimal (143 → `14.3`); RAIN = tip count; everything else = raw. Empty for 2047 (full scale: no reading) |
| 10 | unit | text | | `V`, `tips` or empty | Unit of `eng`; empty when `eng` is |
| 11 | fmt | text | | `ABF` `EIF` `A2C` | Frame format |
| 12 | pol | text | | `STD` `NEG` | Framing polarity: STD = idle high (start 0, stop 1); NEG = idle low (start 1, stop 0) † |
| 13 | inv | int | | 0 or 1 | 1 = the frame decoded only with the demodulated bits complemented |
| 14 | frame | int | | 0-15 | Index of this frame within its burst |
| 15 | rssi | int | dBm | | Peak RSSI while the squelch was open |
| 16 | nf | int | dBm | | Averaged noise floor at the time: a ~30 s exponential average of the RSSI while the squelch is closed |
| 17 | sens | int | dBm | | Estimated sensitivity = nf + SNR_REQ (a setting, default 12 dB). An estimate. |
| 18 | fade | int | dB | | Fade margin = rssi - sens |
| 19 | burst_ms | int | ms | 100-1500 | How long the squelch was open |
| 20 | payload_hex | hex | | 8 digits | The 32 decoded data bits, uppercase |
| 21 | payload_bin | text | | 32 chars `0`/`1` | The same bits, bit 31 first |

The payload is the input of `ALERT_DecodePayload32()` (`App/app/alert_decode.c`;
the Python port is `decode_payload32()` in `tools/alert/alertmon.py`). Bit
31 − (8k + i) is bit i of the frame's k-th 8-bit word, where bits go LSB first
on air. Re-decoding it gives `fmt`, `id` and `value` back.

Examples:

```
DEC,1041,1790843886,187340,12,2088,MARBURG,BATT,143,14.3,V,ABF,STD,1,0,-20,-121,-109,89,412,16067B23,00010110000001100111101100100011
DEC,1042,1790843919,220510,12,2443,KINGSHOLME MO,BATT,142,14.2,V,ABF,STD,1,0,-47,-121,-109,62,530,D2663B23,11010010011001100011101100100011
DEC,1043,1790843919,220510,12,2442,KINGSHOLME MO,RAIN,23,23,tips,ABF,STD,1,1,-47,-121,-109,62,530,52667703,01010010011001100111011100000011
DEC,1044,1790843962,263880,12,4109,ROTHWELL,RAIN,1290,1290,tips,ABF,STD,1,0,-88,-121,-109,21,455,B202AB17,10110010000000101010101100010111
DEC,1045,1790843967,268870,12,4110,ROTHWELL,LVL,12,12,,ABF,STD,1,0,-104,-121,-109,5,390,72029B03,01110010000000101001101100000011
DEC,7,,95230,,3001,,,57,57,,EIF,NEG,0,0,-61,-119,-107,46,380,9F753812,10011111011101010011100000010010
```

What the examples show:

- **Two frames, one burst.** 1042 and 1043 came from the same burst: same
  `uptime_ms`, `frame` 0 and 1.
- **No clock, no log.** The last line is from a radio with no clock set (empty
  `epoch`) and no usable log (empty `boot`, a per-boot `seq`).
- **Unknown address.** Its id 3001 is not in the station table: empty `name`
  and `kind`.

### 5.3 BST: one burst

One line per qualifying burst, decoded or not. A burst qualifies when the
squelch was open for 100-1500 ms while receiving. With SQ GATE on, its peak
must also be at least 15 dB over the noise floor. BST is how a client tells
"nothing on air" from "heard it, could not decode it".

| # | Field | Type | Unit | Range | Meaning |
|---|---|---|---|---|---|
| 1 | seq | u32 | | ≥ 1 | Qualifying bursts since boot, counting this one: the STA `bursts` count † |
| 2 | epoch | u32 | s | empty, or Unix time | As DEC |
| 3 | uptime_ms | u32 | ms | | Squelch close. The DEC lines from this burst have the same value. |
| 4 | peak | int | dBm | | Peak RSSI during the burst |
| 5 | nf | int | dBm | | Averaged noise floor |
| 6 | burst_ms | int | ms | 100-1500 | Squelch-open duration |
| 7 | nframes | int | | 0-15 | Number of DEC lines this burst produced (0 = not decoded) † |
| 8 | nbits | int | | 0-512 | Bits the demodulator produced, one per 300-baud bit |
| 9 | bits_hex | hex | | ≤ 120 digits | The first min(60, ⌈nbits/8⌉) bytes of those bits. MSB first: the first bit of the burst is the top bit of the first byte. Raw sense (not complemented). Bits past `nbits` in the last byte mean nothing: use `nbits`. |

```
BST,14,1790843886,187340,-20,-121,412,1,100,0000003D2FCB08EE0109001060
BST,15,1790843919,220510,-47,-121,530,2,138,0000004B599711DC6B599621FC0009C30440
BST,16,1790843962,263880,-88,-121,455,1,115,000000000029AFEAA8F4012901C040
```

These are real bursts, and each decodes (complemented, polarity STD) to the
DEC lines above with the same `uptime_ms`.

### 5.4 STA: status, every 10 s

| # | Field | Type | Unit | Range | Meaning |
|---|---|---|---|---|---|
| 1 | epoch | u32 | s | empty, or Unix time | Now |
| 2 | uptime_ms | u32 | ms | | Now |
| 3 | nf | int | dBm | | Averaged noise floor |
| 4 | rssi | int | dBm | | Instantaneous RSSI |
| 5 | sq | int | | 0 or 1 | 1 = squelch open |
| 6 | batt_mv | int | mV | | Battery voltage |
| 7 | batt_pct | int | % | 0-100 | Battery charge, as the radio's icon shows it |
| 8 | bursts | int | | | Qualifying bursts since boot |
| 9 | decodes | int | | | Frames delivered (DEC lines) since boot |
| 10 | min_ok | int | dBm | | Weakest peak RSSI that decoded this boot; empty before the first decode |
| 11 | log_state | text | | `OK` `OFF` `FOREIGN` `ERR` | The log: usable / not in use / region holds someone else's data / failed |
| 12 | log_count | int | | 0-log_cap | Records in the log |
| 13 | log_cap | int | | | Records the log can hold: 12065 in this build (95 of the 96 sectors × 127 records; one sector is always the next to be erased), 0 when the log is not usable |
| 14 | stn_src | text | | | Station table in use: `BUILTIN MegaNet:<hash>` or `SPI MegaNet:<hash>` |

```
STA,1790843970,270000,-121,-124,0,7890,78,16,18,-104,OK,1045,12065,BUILTIN MegaNet:95f6f8d
STA,,31000,-119,-122,0,7650,61,0,0,,FOREIGN,0,0,BUILTIN MegaNet:95f6f8d
```

### 5.5 EVT: events

`EVT,<epoch>,<uptime_ms>,<code>,<detail>`. `detail` is free text for people:
log it, but do not parse it. It may contain spaces and commas, so it runs to
the end of the line.

| Code | When |
|---|---|
| `BOOT` | The app started after a reset; detail is the reset reason (`POR` `SW` `WD` `FAULT`) and the boot counter |
| `CENSUS` | The audio route: detail is the pin, `pa=` 0/1 and the route's state (`ON`, `CONFIRM`, `OFF`, `REJECT`, or `SAVED` for a choice kept from before) |
| `SET` | A setting changed (keypad or console); detail is `NAME=value`, the name as the console spells it |
| `LOG` | The log's state and fill (`OK 1045/12065`), at app entry and when it changes; `APPEND FAIL` when a record could not be written |
| `STN` | The station table in use and its site count, at app entry and when it changes |
| `CLOCK` | The clock was set: detail `SET` the first time, then `STEP <seconds>` (how far it moved) |

```
EVT,,5230,BOOT,POR 12
EVT,,12050,CENSUS,PA4B pa=0 ON
EVT,1790843760,61020,CLOCK,SET
EVT,1790843790,91240,SET,SNR_REQ=14
EVT,,5240,LOG,FOREIGN 0/0
EVT,1790844100,402000,STN,SPI MegaNet:95f6f8d 2604
```

## 6. Timing

| What | When |
|---|---|
| DEC, BST | Shortly after the squelch closes at the end of a burst. Nothing is sent while the squelch is open: a USB send blocks, and the burst sampler must not be held up. |
| STA | Every 10 s while the ALERT app runs |
| HDR | When the app starts, on `CSV HDR`, and after every 200 record lines |
| EVT | As it happens |
| Console reply | Polled on every pass of the ALERT app's loop, and every 10 ms in normal radio operation. The console does nothing while the squelch is open and for 100 ms after it closes, so a reply can be held back by up to about 2 s. |

Recommended client timeouts. Each is an idle timeout: restart it on every
reply line, so a long `LOG DUMP` never times out while lines keep coming.

| Command | Timeout |
|---|---|
| Most commands | 3 s |
| `LOG DUMP` | 15 s idle |
| `LOG CLEAR YES`, `LOG FORMAT FORCE` | 60 s (the flash erase covers 96 sectors) |
| `STN BEGIN`, `STN CLEAR YES`, `STN FORMAT FORCE` | 30 s |
| `STN W` | 5 s (a line that enters a new 4 KB sector erases it first: 40-300 ms) |
| `STN END` | 30 s (reads back and checks the CRC) |

The clock (`TIME`) counts from the radio's 10 ms tick. It is kept in RAM and
lost on every reboot, so set it on every connect.

## 7. Console

A line-based command interface on the same USB port. It is usable from any
terminal: `python -m serial.tools.miniterm COM5 115200`.

### 7.1 Framing

- A command is one line of at most 96 characters (an `STN W` line's hex
  excepted, section 8), ended by CR or LF.
  Commands are case-insensitive. An empty line is ignored and gets no
  reply †.
- The console does not echo what you type †. Use your terminal's local echo
  (miniterm `--echo`).
- **One command at a time.** Wait for the final `OK` or `ERR` line before
  sending the next command. The radio's receive ring is 256 bytes, shared
  with the binary protocol. A pipelined second command can be lost.
- Every command gets exactly one final line: `OK`, `OK,<detail>` or
  `ERR,<reason>` †. Any reply data lines come before it. Records and debug
  lines may appear anywhere in between.

### 7.2 Errors

| Reply | Meaning † |
|---|---|
| `ERR,UNKNOWN` | No such command in this build |
| `ERR,ARGS` | An argument is missing or malformed |
| `ERR,TOOLONG` | The line was over the limit and was discarded |
| `ERR,NAME` | `SET`/`GET`: no setting by that name |
| `ERR,RANGE` | A value is outside what the setting or command allows |
| `ERR,READONLY` | That setting cannot be changed (`MDM_MODE`) |
| `ERR,NOTINAPP` | That setting acts on the receiver: only while the ALERT app runs (see 7.4) |
| `ERR,FOREIGN` | `STN BEGIN`/`STN CLEAR`: the station region holds data that is not ours (see `STN FORMAT FORCE`) |
| `ERR,CONFIRM` | A destructive command without its `YES` / `FORCE` |
| `ERR,NOLOG` | The log is not usable (state OFF, FOREIGN or ERR) |
| `ERR,STATE` | `STN W`/`STN END` without a `STN BEGIN`, or a write outside the announced length |
| `ERR,CRC` | `STN END`: the uploaded bytes do not match the CRC; nothing was activated |
| `ERR,FLASH` | An SPI flash erase, program or verify failed |

A client should show the reason to the user. It should not depend on reasons
beyond `OK` versus `ERR`, except to treat `UNKNOWN` as "this build lacks the
command".

### 7.3 Commands

| Command | Reply data lines | Effect |
|---|---|---|
| `HELP` | `HELP,<synopsis>` per command | Lists the commands (wording is informational) |
| `INFO` | `INFO,<key>,<value>[,...]`, then `GET` lines | Firmware, uptime, clock, battery, NF, log, station source, then every setting |
| `TIME` | `TIME,<epoch>` (empty if unset) | Reads the UTC clock |
| `TIME <epoch>` | none | Sets the UTC clock: decimal Unix seconds, RAM only. Emits `EVT ... CLOCK`. |
| `CSV HDR` | the HDR block (record lines) | Sends the section 5.1 block again |
| `GET [name]` | `GET,<NAME>,<value>` per setting | One setting, or all of them |
| `SET <name> <value>` | none | Changes a setting and saves it. Emits `EVT ... SET`. |
| `LOG STAT` | `LOG,<count>,<capacity>,<oldest_seq>,<newest_seq>,<state>` | Log status; seq fields are empty when the log is empty |
| `LOG DUMP [n]` | `LOG,<the 21 DEC fields>` per record | The newest n records (default all), oldest first |
| `LOG CLEAR YES` | none | Erases the log |
| `LOG FORMAT FORCE` | none | Takes over a region that holds foreign data (state FOREIGN) and formats it |
| `STN INFO` | `STN,<source>,<count>,<crc>,<state>` | Table in use. crc is the uploaded blob's header CRC (8 hex digits), empty for the built-in table. state: `SPI` (uploaded table in use), `BUILTIN` (region blank or cleared), `BAD` (an upload of ours that failed its checks), `FOREIGN` (not ours; uploads refused until `STN FORMAT FORCE`) |
| `STN GET <id>` | `STN,<id>,<name>,<kind>` | One lookup; name and kind are empty when the id is not in the table |
| `STN BEGIN <len> <crc32>` | none | Starts an upload (section 8): erases, expects len bytes |
| `STN W <off> <hex>` | none | Writes bytes at offset off of the upload |
| `STN END` | none | Checks the CRC and activates the new table. Emits `EVT ... STN`. |
| `STN CLEAR YES` | none | Erases the uploaded table and reverts to the built-in one |
| `STN FORMAT FORCE` | none | Takes over a station region that holds foreign data (uploads are refused until then), like `LOG FORMAT FORCE` † |
| `SCREEN` | 8 × `SCR,<row>,<256 hex>` | The display, section 9 |
| `SPI READ <addr> <len>` | `SPI,<addr>,<hex>` per 32 bytes | Reads SPI flash at any address: addr is `0x` hex or decimal, len 1-256; the reply addr is 6 hex digits |
| `REBOOT` | none | `OK`, then a reset with the transmitter safely off (`DFU_SafeReset`). The port disappears and comes back once the radio has restarted. The OK itself can be lost to the reset. |

Numbers in commands are decimal unless the table says otherwise. The line
shapes of `HELP`, `INFO`, `TIME`, `GET`, `STN INFO` and `SPI` are †. The
`LOG`, `STN GET` and `SCR` shapes are from the spec.

### 7.4 Settings

`SET` and `GET` take the settings screen's row names, with spaces written as
`_`. Values are written as `GET` shows them, also with `_` for a space
(`SET CONFIRM 2_COPIES`) †.

| Name | Values | Default | Notes |
|---|---|---|---|
| `VOICE` | `OFF` `ON` | OFF | Read new readings out loud |
| `SPEAKER` | `OFF` `SQL` `ON` | SQL | SQL = speaker on only while the squelch is open |
| `CSV_OUT` | `OFF` `ON` | ON | The section 5 records. The console answers either way. |
| `LOG` | `OFF` `ON` | ON | Append to the flash log (only when the log is usable) |
| `UNKNOWN` | `SHOW` `HIDE` | SHOW | HIDE drops addresses that are not in the table, from the screen and from DEC |
| `CONFIRM` | `OFF` `2 COPIES` | OFF | Require the same reading twice in one burst |
| `SQ_GATE` | `OFF` `ON` | ON | A burst must peak 15 dB over the floor to count |
| `SNR_REQ` | `6`-`20` | 12 | dB over the noise floor assumed necessary to decode; used for `sens` |
| `DEBUG` | `OFF` `ON` | OFF | Adds the `D` heartbeat and raw `A` lines |
| `MDM_MODE` | `ADC` | ADC | Read-only |
| `CENSUS` | the chosen audio pin, e.g. `PA4B`; `NOT RUN`, `PENDING` | | Re-running it is a keypad action (UP on the row) |
| `FREQ_MHZ` | e.g. `151.500` | as stored | 12.5 kHz steps |
| `SQL_LEVEL` | e.g. `3.0` | as stored | |

`FREQ_MHZ`, `SQL_LEVEL` and `CENSUS` retune the receiver, so the console can
change them only while the ALERT app runs. Outside the app they answer
`ERR,NOTINAPP` (`MDM_MODE` always answers `ERR,READONLY`). Outside the app, a changed
`SPEAKER` takes effect at the next app entry.

### 7.5 Example sessions

Identify and read the settings:

```
> TIME 1790843760
OK
EVT,1790843760,61020,CLOCK,SET
> TIME
TIME,1790843762
OK
> INFO
INFO,fw,4d06107f
INFO,uptime_ms,61240
INFO,epoch,1790843762
INFO,batt_mv,7890
INFO,batt_pct,78
INFO,nf,-121
INFO,log,OK,1040,12065
INFO,stn,BUILTIN MegaNet:95f6f8d,443,BUILTIN
GET,VOICE,OFF
GET,SPEAKER,SQL
GET,CSV_OUT,ON
GET,LOG,ON
GET,UNKNOWN,SHOW
GET,CONFIRM,OFF
GET,SQ_GATE,ON
GET,SNR_REQ,12
GET,DEBUG,OFF
GET,MDM_MODE,ADC
GET,CENSUS,PA4B
GET,FREQ_MHZ,151.500
GET,SQL_LEVEL,3.0
OK
> CSV HDR
HDR,fw,4d06107f,schema,2
HDR,DEC,seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,frame,rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin
HDR,BST,seq,epoch,uptime_ms,peak,nf,burst_ms,nframes,nbits,bits_hex
HDR,STA,epoch,uptime_ms,nf,rssi,sq,batt_mv,batt_pct,bursts,decodes,min_ok,log_state,log_count,log_cap,stn_src
HDR,EVT,epoch,uptime_ms,code,detail
OK
```

Change settings:

```
> GET SNR_REQ
GET,SNR_REQ,12
OK
> SET SNR_REQ 14
OK
EVT,1790843790,91240,SET,SNR_REQ=14
> SET SNR_REQ 30
ERR,RANGE
> SET CONFIRM 2_COPIES
OK
EVT,1790843795,96010,SET,CONFIRM=2 COPIES
> GET CONFIRM
GET,CONFIRM,2 COPIES
OK
> SET MDM_MODE FSK
ERR,READONLY
> SET VOLUME 3
ERR,NAME
```

The log:

```
> LOG STAT
LOG,1045,12065,1,1045,OK
OK
> LOG DUMP 2
LOG,1044,1790843962,263880,12,4109,ROTHWELL,RAIN,1290,1290,tips,ABF,STD,1,0,-88,-121,-109,21,455,B202AB17,10110010000000101010101100010111
LOG,1045,1790843967,268870,12,4110,ROTHWELL,LVL,12,12,,ABF,STD,1,0,-104,-121,-109,5,390,72029B03,01110010000000101001101100000011
OK
> LOG CLEAR
ERR,CONFIRM
> LOG CLEAR YES
OK
EVT,1790844020,321500,LOG,OK 0/12065
> LOG STAT
LOG,0,12065,,,OK
OK
```

A region that holds someone else's data:

```
> LOG STAT
LOG,0,0,,,FOREIGN
OK
> LOG DUMP
ERR,NOLOG
> LOG FORMAT FORCE
OK
EVT,,5890,LOG,OK 0/12065
```

Station lookups, reads and errors:

```
> STN INFO
STN,BUILTIN MegaNet:95f6f8d,443,,BUILTIN
OK
> STN GET 2088
STN,2088,MARBURG,BATT
OK
> STN GET 3001
STN,3001,,
OK
> SPI READ 0x1C0000 32
SPI,1C0000,4153544201002C0A24C40000610A8F3E4D6567614E65743A3935663666386400
OK
> STN END
ERR,STATE
> FOO
ERR,UNKNOWN
> TIME yesterday
ERR,ARGS
> HELP
HELP,HELP
HELP,INFO
HELP,TIME [epoch]
HELP,CSV HDR
HELP,GET [name]
HELP,SET name value
HELP,LOG STAT|DUMP [n]|CLEAR YES|FORMAT FORCE
HELP,STN INFO|GET id|BEGIN len crc32|W off hex|END|CLEAR YES
HELP,SCREEN
HELP,SPI READ addr len
HELP,REBOOT
OK
```

## 8. Station upload

The radio names stations from a table. The built-in one (`BUILTIN
MegaNet:<hash>`) is compiled into the firmware and cut to 13-character names
to fit. A larger table with full names can be uploaded into SPI flash
(0x1C0000-0x1DFFFF, 128 KB). `tools/alert/gen_stations.py --blob out.bin`
builds one. With the filter `all`, it covers the whole of MegaNet (~2,600
sites).

### Blob format

All little-endian (V2_SPEC section 8):

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `ASTB` |
| 4 | 2 | version = 1 |
| 6 | 2 | count: number of sites |
| 8 | 4 | names_len: bytes of name text |
| 12 | 4 | crc32 of everything after this 32-byte header (zlib CRC-32) |
| 16 | 16 | source, e.g. `MegaNet:95f6f8d`, NUL-padded |
| 32 | 8 × count | `{u16 base_id, u16 kinds, u32 name_off}`, sorted by base_id |
| 32 + 8 × count | names_len | NUL-terminated names, ≤ 40 characters, uppercase, commas replaced by spaces |

One entry covers `base_id` to `base_id + 4`. `kinds` packs a 3-bit kind code
per offset: bits 0-2 for base+0, up to bits 12-14 for base+4. The codes are
0 none, 1 RAIN, 2 LEVEL, 3 BATT, 4 REP, 5 OTHER, 6 CHECK, the same as
`App/app/alert_stations_gen.h`.

### Procedure

1. **`STN BEGIN <len> <crc32>`.** `len` is the blob size in decimal bytes, at
   most 131072. `crc32` is the zlib CRC-32 of all len bytes, as 8 hex digits
   †. The radio erases the region: allow 30 s.
2. **`STN W <off> <hex>`, once per chunk, in increasing offset order.**
   `off` is the decimal byte offset. `hex` is the chunk's bytes, 2 hex digits
   each, at most 64 bytes. Wait for each `OK` before sending the next line.
   Write each byte once: flash cannot be rewritten without an erase.
3. **`STN END`.** The radio checks the CRC and activates the table.
   `STN INFO` then shows `SPI ...` (the next app entry's `EVT ... STN` too).

**Chunk size.** The 96-character line limit (section 7.1) holds a 32-byte
chunk (`STN W 131040 ` plus 64 hex digits is 77 characters). A 64-byte chunk
makes a 141-character line; this firmware takes it anyway, because it packs
an `STN W` line's hex digits two to a byte as they arrive, so the limit only
applies to the `STN W <off> ` part. alertterm starts at 64, and if any
`STN W` or the `STN END` is refused, starts over at 32 †, which also covers
a build without the packing.

If an upload is interrupted, the erased region holds no valid table and the
built-in table stays in use. Run the upload again. `STN CLEAR YES` erases an
uploaded table on purpose.

```
> STN BEGIN 71076 9A3C51E0
OK
> STN W 0 4153544201002C0A24C40000610A8F3E4D6567614E65743A3935663666386400
OK
> STN W 32 6C071C00000000006E07D100130000007307190029000000750719003E000000
OK
```

(2,219 more `STN W` lines of 32 bytes)

```
> STN W 71072 54455200
OK
> STN END
OK
> STN INFO
STN,SPI MegaNet:95f6f8d,2604,3E8F0A61,SPI
OK
```

The first chunk is the header itself: `ASTB`, version 1, 2604 sites, 50212
bytes of names, header crc `3E8F0A61`, source `MegaNet:95f6f8d`.

## 9. Screen capture

`SCREEN` returns 8 lines, `SCR,<row>,<256 hex digits>`, then `OK`:

- **Row 0** is the status line (`gStatusLine`).
- **Rows 1-7** are the frame buffer rows 0-6 (`gFrameBuffer`). Row 7 is under
  the bezel on the real radio.

Each row is 128 bytes, exactly as in RAM. These are the ST7565's pages:

- Byte x is column x.
- Bit b of that byte is the pixel at y = 8 × row + b. Bit 0 is the top; a set
  bit is a lit (dark) pixel.

The result is a 128 × 64 image:
`pixel(x, y) = (row[y // 8][x] >> (y % 8)) & 1`.

This example is a 1-pixel frame around the whole display:

```
> SCREEN
SCR,0,FF010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101FF
SCR,1,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,2,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,3,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,4,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,5,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,6,FF000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000FF
SCR,7,FF808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080808080FF
OK
```

`alertterm.py screenshot out.png` turns this into a PNG. The writer is about
20 lines of Python (`png_1bit`), with no imaging library.

## 10. Binary 0xABCD commands

The stock Quansheng protocol shares the port. The ALERT build keeps four of
its commands; the rest of the stock set is out of scope here. Framing, as in
`tools/serialtool/msg.py` and `App/app/uart.c`:

```
AB CD | len (u16 LE) | obfuscated( msg , crc ) | DC BA
msg = ID (u16 LE) | data_len (u16 LE) | data     (padded with one 0 to an even length)
crc = CRC-16/XMODEM of msg (poly 0x1021, init 0), u16 LE
obfuscation: XOR byte i of (msg + crc) with key[i % 16],
key = 16 6C 14 E6 2E 91 0D 40 21 35 D5 40 13 03 E9 80
```

`len` counts msg only. Frames from the radio use the same framing, but their
two CRC bytes are `FF FF` before obfuscation, not a CRC. Do not check them.

| ID | Name | Data | Answer |
|---|---|---|---|
| `0x0514` | hello | u32 session id (any value) | Binary `0x0515`: `char version[16]`, `u8 has_custom_aes_key`, `u8 in_lock_screen`, 2 pad bytes, `u32 challenge[4]`. As on the stock firmware, it may also switch the backlight off. |
| `0x05DD` | reboot | none | None. The radio resets with the transmitter safely off (`DFU_SafeReset`), and the port disappears. After re-enumeration the boot line `B ver=<hash> rst=sw n=<abnormal resets> bl=<bootloader crc>` is sent, if a host already has DTR up. |
| `0x05E1` | DFU check | none | Text line `K bl crc=<8 hex> ver=<7 chars> ok=<0/1> fw=<hash>`. ok=1 means the on-device stock bootloader passed its CRC/version guard, so hands-off DFU is safe. |
| `0x05E0` | enter DFU | u32 LE `0x44465521` ("DFU!") | `K dfu ok=1`, then a reset into the stock bootloader's DFU (its `0x0518`/`0x0530` beacons follow). Refused with `K dfu ok=0 err=magic`, `err=guard` (bootloader check failed) or `err=tx` (transmitting). |

Complete frames:

```
0x05E1           AB CD 04 00 F7 69 14 E6 80 88 DC BA
0x05DD           AB CD 04 00 CB 69 14 E6 5B EB DC BA
0x05E0 "DFU!"    AB CD 08 00 F6 69 10 E6 0F C4 4B 04 F0 61 DC BA
0x0514 id=6A0B1C2D AB CD 08 00 02 69 10 E6 03 8D 06 2A 22 51 DC BA
```

`tools/hotflash.py` drives `0x05E1` and `0x05E0` to flash a radio without
keypresses, and `tools/alert/radio.py` sends any of them (`dfu-check`,
`reboot`, `send`). When reading the port, cut binary frames out of the text
stream using their length field. The firmware sends each line and each frame
as one write, so a frame never lands inside a line. `0xAB` never appears in
the ASCII text.

## 11. Writing a client

Checklist:

1. **Find the port** by VID `0x36B7`. Open it at any baud, assert DTR
   (toggle it), then flush the input.
2. **Read continuously.** Split on LF, strip CR, and cut out binary frames.
   Toggle DTR after 25 s of silence.
3. **Classify every line** (section 3): final, record, debug or data.
4. **Map record fields by name** from the `HDR` block. Send `CSV HDR` once
   connected; until it answers, use this page's lists. Ignore unknown types
   and fields; check `schema`.
5. **Set the clock:** `TIME <unix seconds>` on every connect.
6. **One command at a time.** Send, collect data lines, and stop at the
   final line. Hand record and debug lines that arrive meanwhile to the
   stream handler. Use an idle timeout (section 6).
7. **Use `DEC` for readings and `BST` for bursts.** Pair them by
   `uptime_ms`. Treat an empty field as unknown.

A minimal client in Python: it sets the clock, then prints every reading.

```python
import time

import serial
from serial.tools import list_ports

port = next(p.device for p in list_ports.comports() if p.vid == 0x36B7)
ser = serial.Serial(port, 115200, timeout=0.1)
ser.dtr = False
time.sleep(0.05)
ser.dtr = True                  # the radio sends nothing without DTR
time.sleep(0.1)
ser.reset_input_buffer()

pending = b''
fields = {'DEC': 'seq,epoch,uptime_ms,boot,id,name,kind,value,eng,unit,fmt,pol,inv,'
                 'frame,rssi,nf,sens,fade,burst_ms,payload_hex,payload_bin'.split(',')}


def read_line(timeout):
    """One text line, or None. (A full client also cuts out 0xABCD frames.)"""
    global pending
    end = time.time() + timeout
    while True:
        while b'\n' in pending:
            raw, pending = pending.split(b'\n', 1)
            text = raw.decode('ascii', 'replace').strip()
            if text:
                return text
        if time.time() >= end:
            return None
        pending += ser.read(ser.in_waiting or 1)


def on_stream(line):
    parts = line.split(',')
    if parts[0] == 'HDR' and len(parts) > 2 and parts[1] != 'fw':
        fields[parts[1]] = parts[2:]            # field names come from the radio
    elif parts[0] == 'DEC':
        rec = dict(zip(fields['DEC'], parts[1:]))
        print(rec.get('id'), rec.get('name') or '?', rec.get('eng'), rec.get('unit'),
              rec.get('rssi'), 'dBm')


def command(cmd, timeout=3.0):
    """Send one console command; return its data lines, raise on ERR."""
    ser.write(cmd.encode('ascii') + b'\n')
    data = []
    while True:
        line = read_line(timeout)
        if line is None:
            raise TimeoutError(cmd)
        typ = line.split(',', 1)[0]
        if typ in ('OK', 'ERR'):
            if typ == 'ERR':
                raise RuntimeError('%s: %s' % (cmd, line))
            return data
        if ',' not in line or typ in ('HDR', 'DEC', 'BST', 'STA', 'EVT'):
            on_stream(line)                     # the stream goes on during a command
        else:
            data.append(line)


command('TIME %d' % time.time())
command('CSV HDR')
while True:
    line = read_line(1.0)
    if line:
        on_stream(line)
```

## 12. The reference client: alertterm.py

`tools/alert/alertterm.py` covers everything on this page. Its global
options:

- `--port` overrides discovery by VID.
- `--raw FILE` appends every received line to FILE.
- `--no-time` skips setting the clock. Otherwise, every subcommand sets it on
  connect.

| Subcommand | Does |
|---|---|
| `live [--sta N] [--bursts] [--debug-lines]` | Table of DEC lines, STA summaries and EVT lines. Every raw line goes to `tools/alert/logs/live-<date>.log`. Reconnects after a reboot. |
| `info` | `INFO` |
| `get [NAME]` / `set NAME VALUE` | Settings; `set` reads the value back |
| `time-sync` | Reports the radio's clock error, then sets it |
| `log-download --csv out.csv [--last N]` | `LOG DUMP` to CSV. The header row is the DEC field names from `CSV HDR`. |
| `log-clear [--yes]` | `LOG CLEAR YES` |
| `stations-upload [--filter F \| --all \| --blob B] [--chunk N] [--dry-run]` | Runs `gen_stations.py --blob` and uploads with `STN` |
| `stations-clear [--yes]` | `STN CLEAR YES` |
| `screenshot out.png [--scale N] [--invert]` | `SCREEN` to a 1-bit PNG |
| `console [--quiet]` | Interactive passthrough |

`python tools/alert/test_console.py` checks alertterm's parsers against every
example line on this page. It also checks the PNG writer, the chunking and
the binary frames, and runs the console protocol against a simulated radio.

## 13. Gaps filled here

V2_SPEC sections 6-8 fix the record fields, the command names and the
formats of the LOG, STN GET, SCR and blob data. This page defines the
following as well. Each is marked † above. The firmware should match them;
change this page if it cannot.

- **Final line.** Every console command ends with exactly one final line,
  `OK`, `OK,<detail>` or `ERR,<reason>`. The console does not echo, and an
  empty line gets no reply.
- **Error reasons.** The set in section 7.2.
- **Reply line shapes.** `HELP,<text>`; `INFO,<key>,<value>` (followed by the
  `GET` lines); `TIME,<epoch>` (empty when unset); `GET,<NAME>,<value>`;
  `STN,<source>,<count>,<crc>,<state>` for `STN INFO`, where crc is the
  header crc32 and is empty for the built-in table (`INFO,stn` carries the
  same state after the count); `SPI,<addr 6 hex>,<≤ 32 bytes hex>`.
- **Setting values.** `_` stands for a space in values as well as names.
  `FREQ_MHZ`, `SQL_LEVEL` and `CENSUS` answer `ERR,NOTINAPP` outside the app
  (they return false from `ALERT_SetStep` there); `MDM_MODE` `ERR,READONLY`.
- **`STN BEGIN` crc.** The zlib CRC-32 of all len bytes, not the header's
  crc32 field. Offsets and lengths are decimal.
- **`STN W` chunk size.** The spec allows 64 bytes, which is past the
  96-character line limit; the firmware packs `STN W` hex as it arrives, so
  64 works (section 8).
- **DEC fields.** `kind` uses the labels of `ALERT_KindLabel()` (`LVL`, not
  `LEVEL`), and is empty when unknown. `pol` is `STD`/`NEG`. The non-log
  `seq` starts at 1 each boot.
- **BST fields.** `seq` is the `bursts` count and `nframes` the number of
  DEC lines produced.
- **`STN FORMAT FORCE`.** The station region's version of `LOG FORMAT
  FORCE` (V2_SPEC: "same FOREIGN rule"; `ALERTSTN_Format` in
  `alert_stn.h`).
