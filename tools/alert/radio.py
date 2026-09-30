#!/usr/bin/env python3
"""Log the ALERT-X1 build's USB-C stream, and send it the X1 host commands.

The radio's USB-C port is a CDC ACM device, VID 0x36B7 (App/usb/usbd_cdc_if.c;
the stock bootloader uses the same VID). The firmware only sends while the host
holds DTR, and a send that times out clears its DTR flag: from then on it is
silent until the host asserts DTR again. So the logger re-asserts DTR after 10 s
without a byte, which is what turned several past sessions into empty logs.

    python tools/alert/radio.py log [--minutes N]        # tools/alert/logs/x1-*.log
    python tools/alert/radio.py sweep-ctl goto 5         # 0x0A02 SWEEP_CTL
    python tools/alert/radio.py sweep-ctl goto 0 0x40    # A1, variant: REG_59 invert
    python tools/alert/radio.py sweep-ctl start 1
    python tools/alert/radio.py arr-set 32 --r58 0x3FC3 --t2 1300 --r70 0x80E0 --t1 2100
    python tools/alert/radio.py sweep-ctl goto 32        # arm what arr-set loaded
    python tools/alert/radio.py poke 0x59:0xFFF0:0x0006    # 0x0A03 POKE_LIST
    python tools/alert/radio.py poke                       # empty list: clear
    python tools/alert/radio.py bk-read 0x58 0x5C 0x0B     # 0x0601
    python tools/alert/radio.py bk-write 0x72 0x3065       # 0x0602
    python tools/alert/radio.py reboot [--wait]            # 0x05DD
    python tools/alert/radio.py send 0x0A02 070000         # any ID, raw body

Windows lets one process hold a COM port. While `log` runs it owns the port,
so the other subcommands hand their frame to it over 127.0.0.1 and it sends
the frame and relays the reply; the exchange lands in the log with the rest.
With no logger running they open the port themselves.

Every log line is "<ISO time> <line as received>". Lines the host adds start
with '#': commands sent (#tx), binary replies (#pkt <id> <hex>), DTR
re-assertions and port loss. sweep_judge.py skips them.

Frames use the stock 0xABCD framing, CRC-16/XMODEM and XOR obfuscation, built
by tools/serialtool/msg.py exactly as the firmware's UART_IsCommandAvailable
unpacks them (App/app/uart.c).
"""
import argparse
import json
import os
import socket
import struct
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'serialtool'))
import msg  # noqa: E402  tools/serialtool/msg.py

VID = 0x36B7
LOG_DIR = os.path.join(HERE, 'logs')
RELAY_ADDR = ('127.0.0.1', 47301)   # the logger's command relay; loopback only
SILENCE_S = 10.0                    # re-assert DTR after this long without a byte
BOOTLOADER_BACKOFF_S = 90.0         # leave the port to the flasher (hotflash.py)

CMD_REBOOT = 0x05DD
CMD_BK_READ = 0x0601
CMD_BK_WRITE = 0x0602
CMD_ARR_SET = 0x0A01
CMD_SWEEP_CTL = 0x0A02
CMD_POKE_LIST = 0x0A03

# bootloader beacons: seeing one means the radio is in DFU, and the port
# belongs to whoever is flashing it
BOOTLOADER_IDS = (msg.MSG_NOTIFY_DEV_INFO, msg.MSG_NOTIFY_BL_VER)

# ---------------------------------------------------------------------------
# Payload layouts: the comment above ALERT_HostArrSet in App/app/alert.h is
# the authority, and these follow it. Little-endian. A body the firmware
# cannot use gets "K a0N ok=0"; `send` covers any later change on the bench
# without editing this file.

# SWEEP_CTL {u8 op, u8 a, u8 b}
SWEEP_OPS = {
    'stop': 0,      # stop sweeping, hold the current arrangement
    'start': 1,     # a = phase 1, 2 or 3, from its first entry
    'goto': 2,      # a = arrangement, b = variant byte
    'adopt': 3,     # a = arrangement, b = variant: adopt and persist;
                    # a = 0xFE adopts the ADC route, 0xFF clears the adoption
    'clear': 4,     # clear all scores and repeat counts
    'dwell': 5,     # a = qualifying bursts per arrangement per pass
    'census': 6,    # run the audio-pin census again
    'enter': 7,     # from normal mode: start the ALERT app
    'reboot': 8,
}
# Arrangement indices: 0..22 Phase 1, 32..39 host slots, 64..103 Phase 3.
# Variant byte: 0x1g RX gain g, 0x20 RX BW 100, 0x30 4-byte sync, 0x40 invert,
# 0x5p preamble p, 0x60 REG_59<8>, 0x7n REG_59<2:0> = n, 0x00 none.

# ARR_SET, 21 bytes: slot 32..39, then the recipe
ARR_LAYOUT = (
    ('tag', '3s'),      # up to 3 printable characters, NUL padded ("H<n>" if empty)
    ('r58', 'H'),       # REG_58, RX gain in <9:8>
    ('r70', 'H'),       # <15> set: TONE1 is written too (0x80E0), else 0x00E0
    ('t1', 'H'),        # TONE1 Hz, 0..3000
    ('t2', 'H'),        # TONE2 Hz, 1..3000: the sample clock; 0 empties the slot
    ('r5c', 'H'),
    ('r59', 'H'),       # <10> invert, <8>, <7:4> preamble, <3> 4-byte sync, <2:0>
    ('r5a', 'H'),       # sync bytes 0, 1
    ('r5b', 'H'),       # sync bytes 2, 3 (used with REG_59<3>)
    ('flags', 'B'),     # <0> stream policy (as S1/S2)
)
ARR_DEFAULTS = {'tag': b'', 'r58': 0x3FC3, 'r70': 0x00E0, 't1': 0, 't2': 1200,
                'r5c': 0x5625, 'r59': 0x0000, 'r5a': 0xFFFF, 'r5b': 0x0000, 'flags': 0}
ARR_SLOTS = range(32, 40)

# POKE_LIST {u8 n, n x {u8 reg, u16 and, u16 or}}; n 0 clears; reg = reg & and | or
POKE_MAX = 8


def body_sweep_ctl(op, a=0, b=0):
    return struct.pack('<BBB', op, a, b)


def body_arr_set(slot, fields):
    vals = dict(ARR_DEFAULTS)
    vals.update({k: v for k, v in fields.items() if v is not None})
    if isinstance(vals['tag'], str):
        vals['tag'] = vals['tag'].encode('ascii')
    body = struct.pack('<B', slot)
    for name, fmt in ARR_LAYOUT:
        body += struct.pack('<' + fmt, vals[name])
    return body


def body_poke_list(entries):
    if len(entries) > POKE_MAX:
        raise ValueError('at most %d pokes' % POKE_MAX)
    body = struct.pack('<B', len(entries))
    for reg, and_mask, or_mask in entries:
        body += struct.pack('<BHH', reg, and_mask, or_mask)
    return body


def frame(msg_id, body=b''):
    """One host-to-radio packet: AB CD | len | ID len body | CRC | DC BA."""
    m = msg.Msg.make(msg_id, len(body))
    m.buf[4:] = body
    return bytes(msg.make_packet(bytes(m.buf)))


# ---------------------------------------------------------------------------
# Receive side. Text lines and framed replies share the stream. Lines are
# ASCII, so any 0xAB byte starts a packet; each whole send is one USB write,
# so a packet never lands inside a line.

class Demux:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        """-> list of ('line', str) and ('pkt', id, body)"""
        self.buf += data
        out = []
        while self.buf:
            i = self.buf.find(0xAB)
            text = self.buf if i < 0 else self.buf[:i]
            if i < 0:
                # keep a partial last line until its newline arrives
                cut = text.rfind(b'\n') + 1
                self._lines(bytes(text[:cut]), out)
                del self.buf[:cut]
                break
            if i:
                self._lines(bytes(text) + b'\n', out)
                del self.buf[:i]
                continue
            if len(self.buf) < 4:
                break
            if self.buf[1] != 0xCD:
                del self.buf[:1]            # stray 0xAB: noise, not a packet
                continue
            size = self.buf[2] | (self.buf[3] << 8)
            if size > 512:
                del self.buf[:2]
                continue
            if len(self.buf) < size + 8:
                break
            if self.buf[size + 6] != 0xDC or self.buf[size + 7] != 0xBA:
                del self.buf[:2]
                continue
            pkt = bytearray(self.buf[4:size + 6])
            msg._obfus(pkt, 0, size + 2)
            del self.buf[:size + 8]
            # replies carry no valid CRC (msg.fetch says the same), so none is checked
            if size >= 4:
                m = msg.Msg(pkt[:size])
                n = min(m.get_data_len(), size - 4)
                out.append(('pkt', m.get_msg_type(), bytes(pkt[4:4 + n])))
        return out

    @staticmethod
    def _lines(data, out):
        for raw in data.split(b'\n'):
            txt = raw.decode('ascii', 'replace').strip()
            if txt:
                out.append(('line', txt))


def item_text(item):
    """How an item is logged and relayed."""
    if item[0] == 'line':
        return item[1]
    return '#pkt %04X %s' % (item[1], item[2].hex())


def item_from_text(txt):
    if txt.startswith('#pkt '):
        parts = txt.split()
        try:
            return ('pkt', int(parts[1], 16), bytes.fromhex(parts[2] if len(parts) > 2 else ''))
        except ValueError:
            pass
    return ('line', txt)


def stamp():
    return datetime.now().isoformat(timespec='milliseconds')


# ---------------------------------------------------------------------------
# Serial port

def find_port(explicit=None):
    if explicit:
        return explicit
    from serial.tools import list_ports
    ports = sorted(p.device for p in list_ports.comports() if p.vid == VID)
    return ports[0] if ports else None


def open_port(dev):
    import serial
    ser = serial.Serial(dev, 38400, timeout=0)
    ser.dtr = True      # the radio sends nothing without it
    ser.rts = True
    return ser


def reassert_dtr(ser):
    # toggling guarantees a fresh SET_CONTROL_LINE_STATE; setting True while
    # it is already True may not reach the device at all
    ser.dtr = False
    time.sleep(0.05)
    ser.dtr = True


# ---------------------------------------------------------------------------
# log

class Logger:
    def __init__(self, args):
        self.args = args
        self.ser = None
        self.dev = None
        self.demux = Demux()
        self.last_rx = time.time()
        self.retry_at = 0.0
        self.said_waiting = False
        self.clients = []           # [socket, expiry]
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = args.out or os.path.join(
            LOG_DIR, 'x1-%s.log' % datetime.now().strftime('%Y%m%d-%H%M%S'))
        self.fh = open(self.path, 'a', encoding='ascii', errors='replace', newline='\r\n')

    def emit(self, txt, echo=True):
        self.fh.write('%s %s\n' % (stamp(), txt))
        self.fh.flush()
        if echo and not (self.args.no_d and txt.startswith('D ')):
            if txt.startswith('H ') and len(txt) > 80:
                txt = txt[:60] + '...'
            print(txt, flush=True)
        if txt.startswith('#') and not txt.startswith('#pkt'):
            return                  # the host's own notes are for the log, not a reply
        for c in list(self.clients):
            try:
                c[0].sendall((txt + '\n').encode('ascii', 'replace'))
            except OSError:
                self.drop(c)

    def drop(self, c):
        try:
            c[0].close()
        except OSError:
            pass
        if c in self.clients:
            self.clients.remove(c)

    def ensure_port(self):
        if self.ser is not None or time.time() < self.retry_at:
            return
        try:
            dev = find_port(self.args.port)
            if dev is None:
                raise OSError('no VID %04X port' % VID)
            self.ser = open_port(dev)
        except Exception as e:  # serial.SerialException is an OSError subclass, but be sure
            self.ser = None
            if not self.said_waiting:
                self.emit('# waiting for the radio: %s' % e)
                self.said_waiting = True
            self.retry_at = time.time() + 1.0
            return
        self.dev = dev
        self.said_waiting = False
        self.last_rx = time.time()
        self.demux = Demux()
        self.emit('# opened %s, DTR asserted' % dev)

    def lose_port(self, why, backoff=1.0):
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.retry_at = time.time() + backoff
        self.emit('# port %s released: %s' % (self.dev, why))

    def poll_serial(self):
        if self.ser is None:
            return False
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b''
        except Exception as e:
            self.lose_port('lost (%s)' % e)
            return False
        if not data:
            if time.time() - self.last_rx >= SILENCE_S:
                try:
                    reassert_dtr(self.ser)
                except Exception as e:
                    self.lose_port('lost (%s)' % e)
                    return False
                self.emit('# %.0f s silent: DTR re-asserted' % SILENCE_S, echo=False)
                self.last_rx = time.time()
            return False
        self.last_rx = time.time()
        for item in self.demux.feed(data):
            self.emit(item_text(item))
            if item[0] == 'pkt' and item[1] in BOOTLOADER_IDS:
                self.lose_port('bootloader beacon, leaving the port to the flasher for %.0f s'
                               % BOOTLOADER_BACKOFF_S, BOOTLOADER_BACKOFF_S)
                break
        return True

    def poll_relay(self, srv):
        try:
            conn, _ = srv.accept()
        except (BlockingIOError, socket.timeout):
            return
        except OSError:
            return
        conn.settimeout(2.0)
        try:
            req = b''
            while not req.endswith(b'\n') and len(req) < 4096:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                req += chunk
            req = json.loads(req.decode('ascii'))
            pkt = bytes.fromhex(req['frame'])
            wait = float(req.get('wait', 3.0))
        except (OSError, ValueError, KeyError) as e:
            conn.close()
            self.emit('# relay: bad request (%s)' % e)
            return
        if self.ser is None:
            try:
                conn.sendall(b'#error radio port not open\n')
            except OSError:
                pass
            conn.close()
            return
        c = [conn, time.time() + wait]
        self.clients.append(c)
        self.emit('#tx %s' % pkt.hex(), echo=False)
        try:
            self.ser.write(pkt)
        except Exception as e:
            self.lose_port('lost on write (%s)' % e)

    def expire_clients(self):
        now = time.time()
        for c in list(self.clients):
            if now >= c[1]:
                self.drop(c)

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.bind(RELAY_ADDR)
        except OSError:
            print('another logger already holds the relay on %s:%d' % RELAY_ADDR)
            return 1
        srv.listen(4)
        srv.setblocking(False)
        end = time.time() + 60.0 * self.args.minutes if self.args.minutes else None
        print('logging to %s (Ctrl-C to stop)' % self.path, flush=True)
        self.emit('# radio.py log start%s' % (', %g min' % self.args.minutes
                                              if self.args.minutes else ''))
        try:
            while end is None or time.time() < end:
                self.ensure_port()
                busy = self.poll_serial()
                self.poll_relay(srv)
                self.expire_clients()
                if not busy:
                    time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.emit('# radio.py log stop', echo=False)
            for c in list(self.clients):
                self.drop(c)
            srv.close()
            if self.ser is not None:
                self.ser.close()
            self.fh.close()
        print('log: %s' % self.path)
        return 0


def cmd_log(args):
    return Logger(args).run()


# ---------------------------------------------------------------------------
# one command, one reply

def transact(args, pkt, wait, done):
    """Send pkt and collect items until done(item) or `wait` seconds pass.
    Goes through a running logger if there is one, else opens the port."""
    if not args.direct:
        try:
            s = socket.create_connection(RELAY_ADDR, timeout=0.5)
        except OSError:
            s = None
        if s is not None:
            args.relayed = True
            return _via_relay(s, pkt, wait, done)
    args.relayed = False
    dev = find_port(args.port)
    if dev is None:
        print('no radio found (VID %04X); use --port' % VID)
        return None
    ser = open_port(dev)
    try:
        time.sleep(0.1)     # let DTR reach the device before it has anything to say
        ser.reset_input_buffer()
        ser.write(pkt)
        return listen(ser, wait, done)
    finally:
        ser.close()


def listen(ser, wait, done):
    dm = Demux()
    got = []
    end = time.time() + wait
    while time.time() < end:
        try:
            n = ser.in_waiting
            data = ser.read(n) if n else b''
        except Exception:
            break           # the port went away, e.g. after a reset
        if not data:
            time.sleep(0.02)
            continue
        for item in dm.feed(data):
            got.append(item)
            if done(item):
                return got
    return got


def _via_relay(s, pkt, wait, done):
    got = []
    s.sendall((json.dumps({'frame': pkt.hex(), 'wait': wait}) + '\n').encode('ascii'))
    s.settimeout(wait + 2.0)
    pending = b''
    try:
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            pending += chunk
            while b'\n' in pending:
                raw, pending = pending.split(b'\n', 1)
                txt = raw.decode('ascii', 'replace').strip()
                if txt.startswith('#error'):
                    print(txt[1:])
                    return None
                item = item_from_text(txt)
                got.append(item)
                if done(item):
                    return got
    except socket.timeout:
        pass
    finally:
        s.close()
    return got


def is_ack(item):
    return item[0] == 'line' and (item[1] == 'K' or item[1].startswith('K '))


def show(items, all_lines):
    """Print the reply, not the heartbeat stream it arrived in."""
    for item in items or ():
        txt = item_text(item)
        if all_lines or item[0] == 'pkt' or not txt.startswith(('D ', 'H ')):
            print(txt)


def ack_command(args, msg_id, body, wait=3.0):
    items = transact(args, frame(msg_id, body), wait, is_ack)
    if items is None:
        return 1
    show(items, args.verbose)
    acks = [i[1] for i in items if is_ack(i)]
    if not acks:
        print('no K line within %.0f s: is this the X1 build, with the ALERT app running?'
              % wait)
        return 1
    if 'ok=0' in acks[-1].split():
        # the firmware answers "K <tag> ok=0" to a body it cannot parse
        print('refused: check the payload layout against ALERT_Host* in alert.c')
        return 1
    return 0


# ---------------------------------------------------------------------------
# subcommands

def num(s):
    return int(s, 0)


def cmd_sweep_ctl(args):
    op = SWEEP_OPS.get(args.op)
    if op is None:
        op = num(args.op)
    return ack_command(args, CMD_SWEEP_CTL, body_sweep_ctl(op, args.a, args.b))


def cmd_arr_set(args):
    if args.slot not in ARR_SLOTS:
        print('slot must be 32..39 (the RAM slots)')
        return 2
    if args.raw:
        body = struct.pack('<B', args.slot) + bytes.fromhex(args.raw)
    else:
        r59 = args.r59
        if r59 is None:
            # the same REG_59 fields the Phase 2 variants vary
            r59 = (int(args.inv) << 10) | ((args.pre or 0) << 4) | (int(args.s4) << 3)
        body = body_arr_set(args.slot, {'tag': args.tag, 'r58': args.r58, 'r70': args.r70,
                                        't1': args.t1, 't2': args.t2, 'r5c': args.r5c,
                                        'r59': r59, 'r5a': args.r5a, 'r5b': args.r5b,
                                        'flags': int(args.stream)})
    return ack_command(args, CMD_ARR_SET, body)


def cmd_poke(args):
    entries = []
    for spec in args.pokes:
        parts = spec.split(':')
        if len(parts) != 3:
            print('poke is REG:AND:OR, e.g. 0x59:0xFFF0:0x0006')
            return 2
        entries.append(tuple(num(p) for p in parts))
    return ack_command(args, CMD_POKE_LIST, body_poke_list(entries))


def cmd_bk_read(args):
    rc = 0
    for reg in args.regs:
        r = num(reg)

        def done(item, r=r):
            return item[0] == 'pkt' and item[1] == CMD_BK_READ and item[2][:1] == bytes([r])
        items = transact(args, frame(CMD_BK_READ, struct.pack('<B', r)), 2.0, done)
        hit = [i for i in items or () if done(i)]
        if hit and len(hit[0][2]) >= 3:
            print('REG_%02X = 0x%04X' % (r, hit[0][2][1] | (hit[0][2][2] << 8)))
        else:
            print('REG_%02X: no reply (is ENABLE_UART_RW_BK_REGS on in this build?)' % r)
            rc = 1
    return rc


def cmd_bk_write(args):
    reg, val = num(args.reg), num(args.value)
    # 0x0602 has no reply in uart.c; a short listen shows a K line if the build adds one
    items = transact(args, frame(CMD_BK_WRITE, struct.pack('<BH', reg, val)), 1.0, is_ack)
    if items is None:
        return 1
    show(items, args.verbose)
    print('REG_%02X <- 0x%04X sent (read it back with bk-read)' % (reg, val))
    return 0


def is_boot(item):
    return item[0] == 'line' and item[1].startswith('B ')


def cmd_reboot(args, wait_s=25.0):
    # through a logger, the logger reopens the port itself and relays the B line
    items = transact(args, frame(CMD_REBOOT), wait_s if args.wait else 0.5, is_boot)
    if items is None:
        return 1
    if not args.wait:
        print('reset sent')
        return 0
    boot = [i for i in items if is_boot(i)]
    end = time.time() + wait_s
    time.sleep(1.0)
    while not boot and not args.relayed and time.time() < end:
        # direct: the port vanished with the reset; wait for it to come back
        dev = find_port(args.port)
        if dev:
            try:
                ser = open_port(dev)
            except Exception:
                ser = None
            if ser is not None:
                try:
                    boot = [i for i in listen(ser, end - time.time(), is_boot) if is_boot(i)]
                finally:
                    ser.close()
        time.sleep(0.5)
    if boot:
        print(boot[0][1])
        return 0
    print('no B line within %.0f s (sent before the host reopened the port?)' % wait_s)
    return 1


def cmd_send(args):
    body = bytes.fromhex(args.hex) if args.hex else b''
    items = transact(args, frame(num(args.id), body), args.wait, lambda i: False)
    if items is None:
        return 1
    show(items, args.verbose)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', help='COM port (default: the first with VID 36B7)')
    ap.add_argument('--direct', action='store_true',
                    help='open the port even if a logger is running (it will fail on Windows)')
    ap.add_argument('-v', '--verbose', action='store_true', help='also print D and H lines')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('log', help='log the stream to tools/alert/logs/')
    p.add_argument('--minutes', type=float, default=0, help='stop after N minutes (default: Ctrl-C)')
    p.add_argument('--out', help='log file (default: logs/x1-<date>-<time>.log)')
    p.add_argument('--no-d', action='store_true', help='log D heartbeats but do not echo them')
    p.set_defaults(fn=cmd_log)

    p = sub.add_parser('sweep-ctl', help='0x0A02: ' + ' '.join(SWEEP_OPS))
    p.add_argument('op', help='|'.join(SWEEP_OPS) + ' or a number')
    p.add_argument('a', nargs='?', type=num, default=0)
    p.add_argument('b', nargs='?', type=num, default=0)
    p.set_defaults(fn=cmd_sweep_ctl)

    p = sub.add_parser('arr-set', help='0x0A01: write a RAM arrangement slot (32-39)')
    p.add_argument('slot', type=num)
    p.add_argument('--tag', help='up to 3 characters for the L line (default H<n>)')
    for name in ('r58', 'r70', 't1', 't2', 'r5c', 'r5a', 'r5b'):
        p.add_argument('--' + name, type=num, help='default %s' % (
            ARR_DEFAULTS[name] if name in ('t1', 't2') else '0x%04X' % ARR_DEFAULTS[name]))
    p.add_argument('--r59', type=num, help='REG_59 base, raw (overrides --inv/--pre/--s4)')
    p.add_argument('--inv', action='store_true', help='REG_59<10> invert')
    p.add_argument('--pre', type=num, help='REG_59<7:4> preamble nibble (default 0)')
    p.add_argument('--s4', action='store_true', help='REG_59<3>: 4-byte sync, REG_5A then REG_5B')
    p.add_argument('--stream', action='store_true', help='stream policy (as S1/S2)')
    p.add_argument('--raw', help='20 bytes of hex sent after the slot byte instead of the fields')
    p.set_defaults(fn=cmd_arr_set)

    p = sub.add_parser('poke', help='0x0A03: REG:AND:OR entries applied after every arm')
    p.add_argument('pokes', nargs='*')
    p.set_defaults(fn=cmd_poke)

    p = sub.add_parser('bk-read', help='0x0601: read BK4829 registers')
    p.add_argument('regs', nargs='+')
    p.set_defaults(fn=cmd_bk_read)

    p = sub.add_parser('bk-write', help='0x0602: write one BK4829 register')
    p.add_argument('reg')
    p.add_argument('value')
    p.set_defaults(fn=cmd_bk_write)

    p = sub.add_parser('reboot', help='0x05DD: NVIC_SystemReset')
    p.add_argument('--wait', action='store_true', help='wait for the B boot line')
    p.set_defaults(fn=cmd_reboot)

    p = sub.add_parser('send', help='any message ID with a raw hex body')
    p.add_argument('id')
    p.add_argument('hex', nargs='?', default='')
    p.add_argument('--wait', type=float, default=3.0)
    p.set_defaults(fn=cmd_send)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
