#!/usr/bin/env python3
"""Check the ALERT app's screen text against the geometry of the glass.

Two faults reached hardware that a script can rule out without a reflash:

  * text drawn past the right-hand edge. The small font is a 6 px glyph plus
    1 px of spacing, so a character costs 7 px and the 128 px line holds
    eighteen. "ALERT MDM" placed at column 0 is 63 px wide and the signal
    reading was placed at column 60, so the header was drawn over itself.

  * anything on row 6. On this radio the bottom row of the glass sits partly
    under the bezel and cannot be read - which is where the counters were.

Only checks that are exact are enforced. Widths of formatted strings depend on
the range of each argument, which is not visible here: guessing five digits for
every %u flagged every line in the file and would have taught everyone to
ignore the output. Those worst cases are worked out at the call site instead,
and written down in the comment next to it.

    python tools/alert/check_layout.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'App', 'app', 'alert.c')

CHAR_PX = 7                       # 6 px glyph + 1 px spacing
LCD_PX = 128
MAX_CHARS = LCD_PX // CHAR_PX     # 18
BEZEL_ROW = 6                     # unreadable on this hardware

STR = r'"((?:[^"\\]|\\.)*)"'


def main():
    text = open(SRC, 'r', encoding='utf-8', errors='replace').read()
    name = os.path.basename(SRC)
    problems = []

    def lineno(pos):
        return text.count('\n', 0, pos) + 1

    # 1. rows. Every row argument, literal or formatted, must clear the bezel.
    for m in re.finditer(r'UI_PrintString(?:Small)?\w*\(\s*(?:%s|\w+)\s*,'
                         r'\s*\w+\s*,\s*\w+\s*,\s*(\d+)' % STR, text):
        row = int(m.group(2))
        if row >= BEZEL_ROW:
            problems.append('%s:%d  row %d is under the bezel and cannot be read'
                            % (name, lineno(m.start()), row))

    # 2. literal strings on a row: exact, so enforce them.
    for m in re.finditer(r'UI_PrintStringSmallNormal\(\s*%s' % STR, text):
        s = m.group(1)
        if len(s) > MAX_CHARS:
            problems.append('%s:%d  literal is %d chars, %d fit: "%s"'
                            % (name, lineno(m.start()), len(s), MAX_CHARS, s))

    # 3. status line: absolute pixel columns, also exact for literals.
    placed = []
    for m in re.finditer(r'UI_PrintStringSmallBufferNormal\(\s*(?:%s|[^,]+)\s*,'
                         r'\s*gStatusLine\s*\+\s*([A-Z_0-9]+(?:\s*\+\s*[A-Z_0-9]+)?)' % STR, text):
        lit, col = m.group(1), m.group(2)
        if lit is None or not col.strip().isdigit():
            continue
        start = int(col)
        end = start + len(lit) * CHAR_PX
        if end > LCD_PX:
            problems.append('%s:%d  status "%s" at %d px ends at %d, panel is %d'
                            % (name, lineno(m.start()), lit, start, end, LCD_PX))
        placed.append((start, end, lit, lineno(m.start())))

    placed.sort()
    for (s1, e1, t1, l1), (s2, _e2, t2, _l2) in zip(placed, placed[1:]):
        if s2 < e1:
            problems.append('%s:%d  status "%s" ends at %d px but "%s" starts at %d'
                            % (name, l1, t1, e1, t2, s2))

    for p in problems:
        print('FAIL  %s' % p)
    if problems:
        print('\n%d layout problem(s)' % len(problems))
        return 1
    print('layout OK: no row %d use, no literal over %d chars, no status overlap'
          % (BEZEL_ROW, MAX_CHARS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
