#!/usr/bin/env python3
"""Check the ALERT app's screen text against the geometry of the glass.

Two faults reached hardware that a script can rule out without a reflash:

  * text drawn past the right-hand edge. The small font is a 6 px glyph plus
    1 px of spacing, so a character costs 7 px and the 128 px line holds
    eighteen. "ALERT MDM" placed at column 0 is 63 px wide and the signal
    reading was placed at column 60, so the header was drawn over itself.

  * anything on row 6. On this radio the bottom row of the glass sits partly
    under the bezel and cannot be read - which is where the counters were.

The V2 screens (alert_ui.c) compute most rows and all of the data's widths,
so the file is built to be checkable instead: every main-area string goes
through Text()/TextRight(), which refuse row 6 at run time and are clipped
to the row by UI_PrintStringSmall, and every status-line string goes through
Status(), which clips it to the width planned for its column. What is
enforced here, all exactly:

  1. rows: no literal row >= 6 anywhere; no UI_PrintString* call with a
     computed row outside Text(); Text() holds the `row < UI_ROWS` guard,
     UI_ROWS is 6 and UI_COLS 18; no gFrameBuffer[6].
  2. literals: a literal on a row fits from its column (18 characters).
  3. status line: the ST_<F>_X / ST_<F>_W fields do not overlap each other or
     the battery icon (its width read from bitmaps.h) and stay on the panel;
     each Status() call names one field's X and W, and a literal fits its W.
     A string written straight into gStatusLine needs a literal column.
  4. the scroll bar column is clear of eighteen characters of text.
  5. every settings name fits its nine-character field and there is one per
     SET_* row (alert.c's table).
  6. the strings the CI binary check greps for ("ALERT SETTINGS", "SQ GATE",
     "MDM MODE") are still in the source.

Widths of formatted strings depend on the range of each argument, which is
not visible here: guessing five digits for every %u flagged every line and
would have taught everyone to ignore the output. Those can no longer reach
past the glass (the clipping above); whether they read well at their worst
is worked out at the call site, in the comment next to it.

    python tools/alert/check_layout.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
APP = os.path.join(ROOT, 'App')
SRCS = [os.path.join(APP, 'app', f) for f in ('alert.c', 'alert_ui.c')]
BITMAPS_H = os.path.join(APP, 'bitmaps.h')

CHAR_PX = 7                       # 6 px glyph + 1 px spacing
LCD_PX = 128
MAX_CHARS = LCD_PX // CHAR_PX     # 18
BEZEL_ROW = 6                     # unreadable on this hardware

STR = r'"((?:[^"\\]|\\.)*)"'
PRINTS = ('UI_PrintString', 'UI_PrintStringSmallNormal', 'UI_PrintStringSmallBold',
          'UI_PrintStringSmallNormalInverse')
STATUS_PRINTS = ('UI_PrintStringSmallBufferNormal', 'UI_PrintStringSmallBufferBold')


def literal(arg):
    """The text of a single C string literal, else None."""
    m = re.fullmatch(r'\s*%s\s*' % STR, arg)
    return m.group(1) if m else None


def number(arg):
    """The value of an integer literal (123, 123u), else None."""
    m = re.fullmatch(r'\s*(\d+)[uU]?\s*', arg)
    return int(m.group(1)) if m else None


def split_args(text, pos):
    """Arguments of the call whose '(' is just before pos: ([args], end)."""
    args, depth, cur, i = [], 0, [], pos
    while i < len(text):
        c = text[i]
        if c == '"' or c == "'":
            j = i + 1
            while j < len(text) and text[j] != c:
                j += 2 if text[j] == '\\' else 1
            cur.append(text[i:j + 1])
            i = j + 1
            continue
        if c == '(':
            depth += 1
        elif c == ')':
            if depth == 0:
                args.append(''.join(cur).strip())
                return args, i + 1
            depth -= 1
        elif c == ',' and depth == 0:
            args.append(''.join(cur).strip())
            cur = []
            i += 1
            continue
        cur.append(c)
        i += 1
    return args, i


def calls(text, fname):
    """Every call of fname: (pos, args). Definitions and prototypes are skipped."""
    for m in re.finditer(r'\b%s\s*\(' % re.escape(fname), text):
        line_start = text.rfind('\n', 0, m.start()) + 1
        before = text[line_start:m.start()]
        if re.search(r'\b(void|bool|int|char|uint\d+_t|size_t)\s*\**\s*$', before):
            continue
        args, _ = split_args(text, m.end())
        yield m.start(), args


def body(text, fname):
    """(start, end) of fname's definition body, or None."""
    m = re.search(r'\b(?:void|bool|int|uint\d+_t)\s+%s\s*\([^)]*\)\s*\{' % re.escape(fname), text)
    if not m:
        return None
    depth, i = 1, m.end()
    while i < len(text) and depth:
        depth += {'{': 1, '}': -1}.get(text[i], 0)
        i += 1
    return m.end(), i


def defines(text):
    return {m.group(1): m.group(2).split('//')[0].strip()
            for m in re.finditer(r'^#define\s+(\w+)\s+(.+)$', text, re.M)}


def evaluate(expr, defs, sizes, depth=0):
    """Integer value of a #define expression built from literals, other
    defines, LCD_WIDTH and sizeof(<bitmap>); None if it is anything else."""
    if depth > 8:
        return None
    e = expr
    for name, n in sizes.items():
        e = re.sub(r'sizeof\s*\(\s*%s\s*\)' % name, str(n), e)
    e = e.replace('LCD_WIDTH', str(LCD_PX))

    def sub(m):
        if m.group(0) in defs:
            v = evaluate(defs[m.group(0)], defs, sizes, depth + 1)
            return '(%d)' % v if v is not None else 'X'
        return m.group(0)
    e = re.sub(r'\b[A-Za-z_]\w*\b', sub, e)
    e = re.sub(r'\b(\d+)[uU]\b', r'\1', e)
    if not re.fullmatch(r'[\d\s+\-*/()]*', e) or not e.strip():
        return None
    return int(eval(e.replace('/', '//')))    # digits and operators only


def bitmap_sizes():
    try:
        text = open(BITMAPS_H, 'r', encoding='utf-8', errors='replace').read()
    except OSError:
        return {}
    return {m.group(1): int(m.group(2))
            for m in re.finditer(r'\b(BITMAP_\w+)\s*\[\s*(\d+)\s*\]', text)}


def check_screen(path, problems, sizes):
    """Checks 1-4 on one file."""
    text = open(path, 'r', encoding='utf-8', errors='replace').read()
    name = os.path.basename(path)
    defs = defines(text)

    def lineno(pos):
        return text.count('\n', 0, pos) + 1

    def fail(pos, msg):
        problems.append('%s:%d  %s' % (name, lineno(pos), msg))

    def inside(pos, span):
        return span is not None and span[0] <= pos < span[1]

    text_body = body(text, 'Text')
    status_body = body(text, 'Status')

    # 1 + 2. the helpers' own guards, when this file has them
    if text_body is not None:
        if not re.search(r'\brow\s*<\s*UI_ROWS\b', text[text_body[0]:text_body[1]]):
            fail(text_body[0], 'Text() no longer refuses rows >= UI_ROWS')
        if evaluate(defs.get('UI_ROWS', ''), defs, sizes) != BEZEL_ROW:
            fail(text_body[0], 'UI_ROWS must be %d: row %d is under the bezel'
                 % (BEZEL_ROW, BEZEL_ROW))
        if evaluate(defs.get('UI_COLS', ''), defs, sizes) != MAX_CHARS:
            fail(text_body[0], 'UI_COLS must be %d' % MAX_CHARS)

    # the firmware's writers: row literal and clear of the bezel, or inside Text()
    for fn in PRINTS:
        for pos, args in calls(text, fn):
            if len(args) < 4:
                continue
            s, start, row = literal(args[0]), number(args[1]), number(args[3])
            if row is None:
                if not inside(pos, text_body):
                    fail(pos, '%s with a computed row: route it through Text(), '
                         'which refuses row %d' % (fn, BEZEL_ROW))
            elif row >= BEZEL_ROW:
                fail(pos, 'row %d is under the bezel and cannot be read' % row)
            if s is not None:
                if len(s) > MAX_CHARS:
                    fail(pos, 'literal is %d chars, %d fit: "%s"' % (len(s), MAX_CHARS, s))
                elif start is not None and number(args[2]) == 0 \
                        and start + len(s) * CHAR_PX > LCD_PX - 1:
                    fail(pos, '"%s" at %d px runs into the last column' % (s, start))

    for pos, args in calls(text, 'Text'):
        if len(args) != 3:
            continue
        s, col, row = literal(args[0]), number(args[1]), number(args[2])
        if row is not None and row >= BEZEL_ROW:
            fail(pos, 'Text on row %d, which is under the bezel' % row)
        if s is not None and (col or 0) + len(s) > MAX_CHARS:
            fail(pos, '"%s" from column %d is %d chars, %d fit'
                 % (s, col or 0, len(s), MAX_CHARS - (col or 0)))
    for pos, args in calls(text, 'TextRight'):
        if len(args) != 2:
            continue
        s, row = literal(args[0]), number(args[1])
        if row is not None and row >= BEZEL_ROW:
            fail(pos, 'TextRight on row %d, which is under the bezel' % row)
        if s is not None and len(s) > MAX_CHARS:
            fail(pos, 'literal is %d chars, %d fit: "%s"' % (len(s), MAX_CHARS, s))

    for m in re.finditer(r'gFrameBuffer\s*\[\s*(\d+)[uU]?\s*\]', text):
        if int(m.group(1)) >= BEZEL_ROW:
            fail(m.start(), 'gFrameBuffer[%s] is under the bezel' % m.group(1))

    # 3. the status line, planned as fields
    fields = {}
    for key in defs:
        m = re.fullmatch(r'ST_(\w+)_X', key)
        if m and m.group(1) != 'BATT':
            x = evaluate(defs[key], defs, sizes)
            w = evaluate(defs.get('ST_%s_W' % m.group(1), ''), defs, sizes)
            if x is None or w is None:
                problems.append('%s  status field %s: X or W is not a plain number'
                                % (name, m.group(1)))
                continue
            fields[m.group(1)] = (x, w)
    spans = []
    for f, (x, w) in fields.items():
        if w > 7:
            problems.append('%s  status field %s is %d wide; Status() clips to 7'
                            % (name, f, w))
        spans.append((x + 1, x + w * CHAR_PX - 1, f))       # lit pixels
    if 'ST_BATT_X' in defs:
        bx = evaluate(defs['ST_BATT_X'], defs, sizes)
        bw = sizes.get('BITMAP_BatteryLevel1')
        if bx is None or bw is None:
            problems.append('%s  battery column or bitmaps.h width unreadable' % name)
        else:
            if bx + bw > LCD_PX:
                problems.append('%s  battery icon at %d px (%d wide) runs off the panel'
                                % (name, bx, bw))
            spans.append((bx, bx + bw - 1, 'battery'))
    spans.sort()
    for s1, e1, f1 in spans:
        if e1 > LCD_PX - 1:
            problems.append('%s  status %s ends at %d px, panel is %d'
                            % (name, f1, e1, LCD_PX))
    for (s1, e1, f1), (s2, _e2, f2) in zip(spans, spans[1:]):
        if s2 <= e1:
            problems.append('%s  status %s ends at %d px but %s starts at %d'
                            % (name, f1, e1, f2, s2))

    for pos, args in calls(text, 'Status'):
        if len(args) != 3:
            continue
        mx = re.fullmatch(r'ST_(\w+)_X', args[1])
        mw = re.fullmatch(r'ST_(\w+)_W', args[2])
        if not mx or not mw or mx.group(1) != mw.group(1) or mx.group(1) not in fields:
            fail(pos, 'Status() must name one field: ST_<F>_X, ST_<F>_W (got %s, %s)'
                 % (args[1], args[2]))
            continue
        s = literal(args[0])
        if s is not None and len(s) > fields[mx.group(1)][1]:
            fail(pos, 'status "%s" is %d chars, field %s holds %d'
                 % (s, len(s), mx.group(1), fields[mx.group(1)][1]))

    # anything written into gStatusLine other than through Status(): literal
    # columns only, checked as the old header was
    placed = []
    for fn in STATUS_PRINTS:
        for pos, args in calls(text, fn):
            if len(args) < 2 or 'gStatusLine' not in args[1] or inside(pos, status_body):
                continue
            m = re.fullmatch(r'\s*gStatusLine\s*\+\s*(\d+)[uU]?\s*', args[1])
            s = literal(args[0])
            if not m or s is None:
                fail(pos, 'status text at a computed column: use Status() and a ST_ field')
                continue
            start = int(m.group(1))
            end = start + len(s) * CHAR_PX
            if end > LCD_PX:
                fail(pos, 'status "%s" at %d px ends at %d, panel is %d'
                     % (s, start, end, LCD_PX))
            placed.append((start, end, s, lineno(pos)))
    placed.sort()
    for (s1, e1, t1, l1), (s2, _e2, t2, _l2) in zip(placed, placed[1:]):
        if s2 < e1:
            problems.append('%s:%d  status "%s" ends at %d px but "%s" starts at %d'
                            % (name, l1, t1, e1, t2, s2))

    # 4. the scroll bar's column: character k lights 7k+1 .. 7k+6, so eighteen
    #    from column 0 light up to px 125
    if 'BAR_X' in defs:
        last = MAX_CHARS * CHAR_PX - 1
        bar = evaluate(defs['BAR_X'], defs, sizes)
        if bar is None or bar <= last or bar > LCD_PX - 1:
            problems.append('%s  BAR_X (%s) is not clear of the text, which ends at %d px'
                            % (name, defs['BAR_X'], last))
    return text


def main():
    problems = []
    sizes = bitmap_sizes()
    texts = {path: check_screen(path, problems, sizes) for path in SRCS}

    # the settings table lives in whichever file defines setNames[]
    path = next((p for p, t in texts.items() if 'setNames' in t), SRCS[0])
    text = texts[path]
    name = os.path.basename(path)

    def lineno(pos):
        return text.count('\n', 0, pos) + 1

    # 5. settings rows. Each is "%c%-9s%s": a name over nine characters pushes
    #    its value off the glass, and a name list that is longer or shorter than
    #    the SET_* enum shifts every label onto the wrong row - FREQ once
    #    stepped the squelch that way. Both are exact.
    m = re.search(r'enum\s*\{([^}]*\bSET_N\b[^}]*)\}', text)
    names = re.search(r'setNames\s*\[\s*SET_N\s*\]\s*=\s*\{(.*?)\};', text, re.S)
    if not m or not names:
        problems.append('%s  settings enum or setNames[] not found' % name)
    else:
        body_ = re.sub(r'//[^\n]*', '', m.group(1))
        rows = [r.strip() for r in body_.split(',') if r.strip()]
        rows = rows[:rows.index('SET_N')] if 'SET_N' in rows else rows
        labels = re.findall(STR, re.sub(r'//[^\n]*', '', names.group(1)))
        if len(labels) != len(rows):
            problems.append('%s:%d  %d settings rows but %d names'
                            % (name, lineno(names.start()), len(rows), len(labels)))
        for s in labels:
            if len(s) > 9:
                problems.append('%s:%d  settings name "%s" is %d chars, 9 fit'
                                % (name, lineno(names.start()), s, len(s)))

    # 6. the strings CI looks for in the binary as proof the app was built in
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
