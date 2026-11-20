#!/usr/bin/env python3
"""Draw the ALERT receiver's screens on this computer, exactly as the radio would.

tools/alert/screen_sim.c compiles the firmware's own screen code
(App/app/alert_ui.c, with the font, the text writers and the battery icon it
draws with) for this computer, so a screen here is the radio's screen pixel for
pixel. What it cannot show is the glass itself: contrast, viewing angle, and the
bottom row hidden under the bezel (which it checks is never drawn on).

    python3 tools/alert/screens.py                 # PNGs of every screen into tools/alert/screens/
    python3 tools/alert/screens.py --out /tmp/s    # somewhere else
    python3 tools/alert/screens.py --check         # draw them all, write nothing; non-zero on a fault
    python3 tools/alert/screens.py --ascii big     # one screen as text, for a quick look

It needs gcc. The program is built with the address and undefined-behaviour
checkers, so a write past a buffer in the screen code fails the run here
rather than freezing a radio.

The scenarios at the bottom are made-up readings at real South-East
Queensland stations from the built-in table; names longer than 13 letters are
the full names an uploaded station table gives.
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
APP = os.path.join(ROOT, 'App')
BUILD = os.path.join(ROOT, 'build', 'screen_sim')

SOURCES = [os.path.join(HERE, 'screen_sim.c')] + [os.path.join(APP, p) for p in (
    'app/alert_ui.c', 'ui/helper.c', 'ui/battery.c', 'font.c', 'bitmaps.c',
    'external/printf/printf.c')]

# What the RescueOps build defines that these files look at.
DEFINES = ['ENABLE_ALERT', 'ENABLE_SMALL_BOLD', 'ENABLE_FEAT_F4HWN', 'ENABLE_UART',
           'ENABLE_USB', 'PRINTF_INCLUDE_CONFIG_H', 'BUILD_COMMIT="sim"']

LCD_BG = (0xBE, 0xD7, 0xAA)       # the colours of the captures in screens/
LCD_FG = (0x28, 0x28, 0x28)

# Values the settings rows show in the pictures. The rows themselves, and
# their order, are read from alert.c.
ROW_VALUES = {
    'FREQ MHz': '151.500', 'SQL LEVEL': '3.0', 'SPEAKER': 'SQL', 'LAYOUT': 'BIG',
    'VOICE': 'OFF', 'UNKNOWN': 'SHOW', 'LOG': 'ON', 'CSV OUT': 'ON', 'CONFIRM': 'OFF',
    'SQ GATE': 'ON', 'SNR REQ': '12', 'CENSUS': 'PA4B', 'DEBUG': 'OFF', 'MDM MODE': 'ADC',
}


def setting_rows():
    """alert.c's settings rows in order: ['FREQ MHz', ...]."""
    text = open(os.path.join(APP, 'app', 'alert.c'), encoding='utf-8').read()
    m = re.search(r'setNames\s*\[\s*SET_N\s*\]\s*=\s*\{(.*?)\};', text, re.S)
    if not m:
        raise SystemExit('setNames[] not found in alert.c')
    return re.findall(r'"([^"]*)"', re.sub(r'//[^\n]*', '', m.group(1)))


def build():
    os.makedirs(BUILD, exist_ok=True)
    exe = os.path.join(BUILD, 'screen_sim')
    if os.path.exists(exe) and all(os.path.getmtime(exe) > os.path.getmtime(s) for s in
                                   SOURCES + [os.path.join(APP, 'app', 'alert_int.h')]):
        return exe
    cmd = ['gcc', '-std=gnu11', '-O1', '-g', '-Wall', '-Wno-unused-function',
           '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
           '-I', APP] + ['-D' + d for d in DEFINES] + SOURCES + ['-o', exe]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit('screen_sim did not build')
    return exe


def run(exe, script):
    """Run one script -> {name: 64 rows of 128 pixels}, and any BEZEL faults."""
    pre = ''.join('setrow %s=%s\n' % (n, ROW_VALUES.get(n, '?')) for n in setting_rows())
    r = subprocess.run([exe], input=pre + script, capture_output=True, text=True)
    if r.returncode not in (0, 1):
        sys.stderr.write(r.stdout[-2000:] + r.stderr)
        raise SystemExit('screen_sim failed (exit %d)' % r.returncode)
    screens, faults, name, rows = {}, [], None, {}
    for line in r.stdout.splitlines():
        if line.startswith('SCREEN '):
            name, rows = line[7:], {}
        elif line.startswith('SCR,'):
            _, row, data = line.split(',')
            rows[int(row)] = bytes.fromhex(data)
            if len(rows) == 8:
                screens[name] = [[(rows[y >> 3][x] >> (y & 7)) & 1 for x in range(128)]
                                 for y in range(64)]
        elif line.startswith('BEZEL '):
            faults.append('%s: drawn on row 6, which is under the bezel' % line[6:])
    return screens, faults


def png(pixels, scale=4):
    """An RGB PNG in the LCD's colours, written by hand (no imaging library)."""
    raw = bytearray()
    for row in pixels:
        line = bytearray()
        for p in row:
            line += bytes(LCD_FG if p else LCD_BG) * scale
        for _ in range(scale):
            raw += b'\x00' + line

    def chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data +
                struct.pack('>I', zlib.crc32(kind + data) & 0xFFFFFFFF))

    w, h = len(pixels[0]) * scale, len(pixels) * scale
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(bytes(raw), 9)) + chunk(b'IEND', b''))


def ascii_art(pixels):
    return '\n'.join(''.join('#' if p else '.' for p in row) for row in pixels)


# ---------------------------------------------------------------------------
# scenarios

STATIONS = """\
stn 4102 2 STRATHPINE GYMPIE RD
stn 4824 2 AVON RIVER AT GAUGE
stn 4126 1 NORMANBY WAY
stn 1994 2 BEACHMERE
stn 1903 2 LESLIE HARRIS DAM
stn 2001 3 CONSTITUTION HILL
stn 1972 1 DAYBORO
"""

# a quiet evening's readings, oldest first
READINGS = """\
nf -74
logtotal 1000
rec 1972 412 -96 290
rec 2001 136 -61 240
rec 1903 1187 -88 170
rec 1994 22 -79 90
rec 4126 34 -13 52
rec 4824 20 -12 41
rec 4102 18 -12 34
"""

LAYOUT = {'big': 0, 'two': 1, 'classic': 2, 'list': 3}


def main_view(layout, extra=''):
    return STATIONS + READINGS + 'cfg layout %d\n' % LAYOUT[layout] + extra


SCENARIOS = {
    # the four layouts, at the newest reading
    'main-big': main_view('big', 'draw main-big\n'),
    'main-two': main_view('two', 'draw main-two\n'),
    'main-classic': main_view('classic', 'draw main-classic\n'),
    'main-list': main_view('list', 'draw main-list\n'),
    # scrolled two back: the bar says how far from the newest
    'big-older': main_view('big', 'key DOWN\nkey DOWN\ndraw big-older\n'),
    # scrolled, and two readings arrive meanwhile: +2 in the header
    'two-held': main_view('two', 'key DOWN\nrec 1994 23 -79 0\nrec 4126 35 -14 0\ndraw two-held\n'),
    # other kinds of value in the big layout
    'big-battery': main_view('big', 'rec 2001 136 -61 3\ndraw big-battery\n'),
    'big-4digit': main_view('big', 'rec 1903 1187 -88 5\ndraw big-4digit\n'),
    'big-full': main_view('big', 'rec 1903 2047 -88 5\ndraw big-full\n'),
    'big-unknown': main_view('big', 'rec 4142 517 -95 8\ndraw big-unknown\n'),
    # the notes that confirm a key
    'note-layout': main_view('big', 'key 2\ndraw note-layout\n'),
    'note-speaker': main_view('two', 'key F\nkey F\ndraw note-speaker\n'),
    # the detail view (1)
    'detail': main_view('two', 'key 1\ndraw detail\n'),
    # settings: the list, then a value being changed
    'settings': main_view('two', 'key MENU\ndraw settings\n'),
    'settings-edit': main_view('two', 'key MENU\nkey MENU\nkey UP\ndraw settings-edit\n'),
    'settings-lower': main_view('two', 'key MENU\n' + 'key DOWN\n' * 7 + 'draw settings-lower\n'),
    # nothing heard yet, and the first seconds
    'waiting': 'nf -112\nentry 2\ndraw waiting\n',
    'settling': 'nf -112\nentry 0 7\ndraw settling\n',
    # The capture in screens/radio-capture-classic.png, redrawn: the same
    # stations, values and ages, the marquee two letters in. --check compares
    # them pixel for pixel, which is what says these pictures are the radio's.
    'classic-check': main_view('classic', 'draw first\ntick 130\ndraw classic-check\n'),
}

CAPTURE = os.path.join(HERE, 'screens', 'radio-capture-classic.png')


def read_png(path):
    """64 rows of 128 pixels from a capture like ours (RGB, 4x, no filters)."""
    data, pos, idat, w = open(path, 'rb').read(), 8, b'', 0
    while pos < len(data):
        n = struct.unpack('>I', data[pos:pos + 4])[0]
        kind = data[pos + 4:pos + 8]
        if kind == b'IHDR':
            w = struct.unpack('>I', data[pos + 8:pos + 12])[0]
        elif kind == b'IDAT':
            idat += data[pos + 8:pos + 8 + n]
        pos += 12 + n
    raw, stride, scale = zlib.decompress(idat), 1 + w * 3, w // 128
    return [[int(raw[y * scale * stride + 1 + x * scale * 3:][:3] == bytes(LCD_FG))
             for x in range(128)] for y in range(64)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(HERE, 'screens'), help='folder for the PNGs')
    ap.add_argument('--check', action='store_true', help='draw everything, write nothing')
    ap.add_argument('--ascii', metavar='SCREEN', help='print one screen as text')
    ap.add_argument('--scale', type=int, default=4)
    args = ap.parse_args()

    exe = build()
    every, faults = {}, []
    for script in SCENARIOS.values():
        s, f = run(exe, script)
        every.update(s)
        faults += f
    if os.path.exists(CAPTURE):
        diff = sum(a != b for ra, rb in zip(read_png(CAPTURE), every['classic-check'])
                   for a, b in zip(ra, rb))
        if diff:
            faults.append('classic-check: %d pixels differ from the radio capture %s'
                          % (diff, os.path.basename(CAPTURE)))
    for f in faults:
        print('FAIL  ' + f)
    if args.ascii:
        if args.ascii not in every:
            raise SystemExit('no screen %r; there are: %s' % (args.ascii, ', '.join(sorted(every))))
        print(ascii_art(every[args.ascii]))
    elif not args.check:
        os.makedirs(args.out, exist_ok=True)
        # the final screen of each scenario; classic-check is a check, and
        # radio-capture-classic.png is the real thing it is checked against
        pictures = [n for n in every if n in SCENARIOS and not n.endswith('-check')]
        for name in pictures:
            with open(os.path.join(args.out, name + '.png'), 'wb') as fh:
                fh.write(png(every[name], args.scale))
        print('wrote %d screens to %s' % (len(pictures), args.out))
    if not faults:
        print('screens OK: %d drawn, nothing on row 6' % len(every))
    return 1 if faults else 0


if __name__ == '__main__':
    sys.exit(main())
