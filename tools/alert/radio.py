#!/usr/bin/env python3
"""Log the ALERT build's USB-C stream, and send it binary protocol commands.

The radio's USB-C port is a CDC ACM device, VID 0x36B7 (App/usb/usbd_cdc_if.c;
the stock bootloader uses the same VID). The firmware only sends while the host
holds DTR, and a send that times out clears its DTR flag: from then on it is
silent until the host asserts DTR again. So the logger re-asserts DTR after 10 s
without a byte, which is what turned several past sessions into empty logs.

    python tools/alert/radio.py log [--minutes N]        # tools/alert/logs/alert-*.log
    python tools/alert/radio.py bk-read 0x58 0x5C 0x0B   # 0x0601
    python tools/alert/radio.py bk-write 0x72 0x3065     # 0x0602
    python tools/alert/radio.py dfu-check                # 0x05E1: the bootloader guard
    python tools/alert/radio.py reboot [--wait]          # 0x05DD
    python tools/alert/radio.py send 0x0514 00000000     # any ID, raw body

The X1 sweep commands (0x0A01 ARR_SET, 0x0A02 SWEEP_CTL, 0x0A03 POKE_LIST)
went with the FSK sweep in V2; the firmware ignores those IDs now.

V2 streams CSV records (HDR, DEC, BST, STA, EVT; docs/ALERT_SERIAL.md). They
are logged like every other line. A command's reply here leaves them out
unless -v, as it does the D heartbeats. The V2 text console (TIME, LOG,
STN, SCREEN...) is alertterm.py's job, not this tool's.

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
CMD_DFU_CHECK = 0x05E1
CMD_BK_READ = 0x0601
CMD_BK_WRITE = 0x0602

# bootloader beacons: seeing one means the radio is in DFU, and the port
# belongs to whoever is flashing it
BOOTLOADER_IDS = (msg.MSG_NOTIFY_DEV_INFO, msg.MSG_NOTIFY_BL_VER)

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
            LOG_DIR, 'alert-%s.log' % datetime.now().strftime('%Y%m%d-%H%M%S'))
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


# V2's CSV records: the stream, never a reply to a command (docs/ALERT_SERIAL.md)
STREAM_PREFIXES = ('D ', 'H ', 'HDR,', 'DEC,', 'BST,', 'STA,', 'EVT,')


def show(items, all_lines):
    """Print the reply, not the heartbeat/record stream it arrived in."""
    for item in items or ():
        txt = item_text(item)
        if all_lines or item[0] == 'pkt' or not txt.startswith(STREAM_PREFIXES):
            print(txt)


def ack_command(args, msg_id, body, wait=3.0):
    items = transact(args, frame(msg_id, body), wait, is_ack)
    if items is None:
        return 1
    show(items, args.verbose)
    acks = [i[1] for i in items if is_ack(i)]
    if not acks:
        print('no K line within %.0f s: is this an ALERT build?' % wait)
        return 1
    if 'ok=1' not in acks[-1].split():
        # "K <tag> ... ok=0": refused, or (dfu-check) the guard failed
        print('the radio answered ok=0')
        return 1
    return 0


# ---------------------------------------------------------------------------
# subcommands

def num(s):
    return int(s, 0)


def cmd_dfu_check(args):
    # dfu.c answers "K bl crc=<crc32> ver=<bootloader> ok=<0|1> fw=<hash>":
    # ok=1 means hotflash.py may take this radio into the stock DFU
    return ack_command(args, CMD_DFU_CHECK, b'')


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
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='also print the stream: D and H lines, V2 CSV records')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('log', help='log the stream to tools/alert/logs/')
    p.add_argument('--minutes', type=float, default=0, help='stop after N minutes (default: Ctrl-C)')
    p.add_argument('--out', help='log file (default: logs/alert-<date>-<time>.log)')
    p.add_argument('--no-d', action='store_true', help='log D heartbeats but do not echo them')
    p.set_defaults(fn=cmd_log)

    p = sub.add_parser('bk-read', help='0x0601: read BK4829 registers')
    p.add_argument('regs', nargs='+')
    p.set_defaults(fn=cmd_bk_read)

    p = sub.add_parser('bk-write', help='0x0602: write one BK4829 register')
    p.add_argument('reg')
    p.add_argument('value')
    p.set_defaults(fn=cmd_bk_write)

    p = sub.add_parser('dfu-check', help='0x05E1: report the bootloader guard (K line)')
    p.set_defaults(fn=cmd_dfu_check)

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
