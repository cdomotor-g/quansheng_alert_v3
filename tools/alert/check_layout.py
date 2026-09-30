#!/usr/bin/env python3
"""Check the ALERT app's screen text against the geometry of the glass.

Two faults reached hardware that a script can rule out without a reflash:

  * text drawn past the right-hand edge. The small font is a 6 px glyph plus
    1 px of spacing, so a character costs 7 px and the 128 px line holds
    eighteen. "ALERT MDM" placed at column 0 is 63 px wide and the signal
    reading was placed at column 60, so the header was drawn over itself.

  * anything on row 6. On this radio the bottom row of the glass sits partly
    under the bezel and cannot be read - which is where the counters were.

Also exact, so also enforced: every settings name fits its nine-character
field and there is one per SET_* row, and the three strings the CI binary check
greps for ("ALERT SETTINGS", "SQ GATE", "MDM MODE") are still in the source.

The screens are in alert_ui.c and the settings rows in alert.c: every file in
SRCS is checked, and the settings and CI-string checks look across all of them.

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
SRCS = [os.path.join(ROOT, 'App', 'app', f) for f in ('alert.c', 'alert_ui.c')]

CHAR_PX = 7                       # 6 px glyph + 1 px spacing
LCD_PX = 128
MAX_CHARS = LCD_PX // CHAR_PX     # 18
BEZEL_ROW = 6                     # unreadable on this hardware

STR = r'"((?:[^"\\]|\\.)*)"'


def check_screen(path, problems):
    """Checks 1-3 (rows, literal widths, status-line columns) on one file."""
    text = open(path, 'r', encoding='utf-8', errors='replace').read()
    name = os.path.basename(path)

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
    return text


def main():
    problems = []
    texts = {path: check_screen(path, problems) for path in SRCS}

    # the settings table lives in whichever file defines setNames[]
    path = next((p for p, t in texts.items() if 'setNames' in t), SRCS[0])
    text = texts[path]
    name = os.path.basename(path)

    def lineno(pos):
        return text.count('\n', 0, pos) + 1

    # 4. settings rows. Each is "%c%-9s%s": a name over nine characters pushes
    #    its value off the glass, and a name list that is longer or shorter than
    #    the SET_* enum shifts every label onto the wrong row - FREQ once
    #    stepped the squelch that way. Both are exact.
    m = re.search(r'enum\s*\{([^}]*\bSET_N\b[^}]*)\}', text)
    names = re.search(r'setNames\s*\[\s*SET_N\s*\]\s*=\s*\{(.*?)\};', text, re.S)
    if not m or not names:
        problems.append('%s  settings enum or setNames[] not found' % name)
    else:
        rows = [r.strip() for r in m.group(1).split(',')]
        rows = rows[:rows.index('SET_N')] if 'SET_N' in rows else rows
        body = re.sub(r'//[^\n]*', '', names.group(1))
        labels = re.findall(STR, body)
        if len(labels) != len(rows):
            problems.append('%s:%d  %d settings rows but %d names'
                            % (name, lineno(names.start()), len(rows), len(labels)))
        for s in labels:
            if len(s) > 9:
                problems.append('%s:%d  settings name "%s" is %d chars, 9 fit'
                                % (name, lineno(names.start()), s, len(s)))

    # 5. the strings CI looks for in the binary as proof the app was built in
    #    (.github/workflows/main.yml). Renaming one fails the build an hour later;
    #    this says so in a second.
    everything = ''.join(texts.values())
    for s in ('ALERT SETTINGS', 'SQ GATE', 'MDM MODE'):
        if '"%s"' % s not in everything:
            problems.append('%s  "%s" is gone, and the CI binary check greps for it'
                            % ('/'.join(os.path.basename(p) for p in SRCS), s))

    for p in problems:
        print('FAIL  %s' % p)
    if problems:
        print('\n%d layout problem(s)' % len(problems))
        return 1
    print('layout OK: no row %d use, no literal over %d chars, no status overlap, '
          'settings rows match their names' % (BEZEL_ROW, MAX_CHARS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
