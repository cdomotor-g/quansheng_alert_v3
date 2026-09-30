#!/usr/bin/env python3
#
# Copyright (c) 2026 ALERT-X1
# Licensed under the MIT License (the "License"); see tools/serialtool for the
# text. Distributed WITHOUT WARRANTY OF ANY KIND.
#
"""Hands-off reflash for the ALERT-X1 build (plan section 6).

After the one unavoidable manual flash, every later flash runs from the PC with
no button held:

  1. (optional) download the CI artifact with `gh run download`;
  2. find the radio by USB VID 0x36B7;
  3. ask the running firmware for the on-device bootloader guard (0x05E1) and
     stop unless it passes;
  4. ask it to enter DFU (0x05E0, payload = magic 0x44465521) - the firmware
     re-checks the same guard, arms a no-init cell and resets into the stock
     bootloader's DFU;
  5. wait for the bootloader's 0x0518 beacons carrying "7.00.07";
  6. drive tools/serialtool to write the 256-byte pages (--bl-ver 7.00);
  7. wait for the app to come back and confirm the git hash changed.

It never holds PTT and never sends 0x0516, so the bootloader is never written;
the worst outcome of any failure is a power cycle. If the bootloader was left
mid-flash by an earlier interrupted run it no longer beacons, so this retries the
page write with serialtool's --resume (resend from page 0) recovery.
"""

import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SERIALTOOL_DIR = os.path.join(HERE, "serialtool")
CLI_PY = os.path.join(SERIALTOOL_DIR, "cli.py")
sys.path.insert(0, SERIALTOOL_DIR)

VID = 0x36B7               # both the app and the bootloader (usbd_cdc_if.c:9)
BL_VER = "7.00"            # the only 0x0530 token the stock bootloader accepts
ENTER_DFU_MAGIC = 0x44465521   # "DFU!" - must match DFU_HOST_MAGIC in dfu.h
CMD_DFU_CHECK = 0x05E1
CMD_ENTER_DFU = 0x05E0
MSG_DEV_INFO = 0x0518      # bootloader beacon


# ---------------------------------------------------------------------------
# Port discovery

def find_ports():
    from serial.tools import list_ports
    return sorted(p.device for p in list_ports.comports() if p.vid == VID)


def wait_for_port(timeout, exclude=None, quiet=False):
    """Return the first VID-matching port, or None on timeout."""
    exclude = set(exclude or [])
    end = time.time() + timeout
    if not quiet:
        print("Waiting for a radio on USB VID {:04X}..".format(VID))
    while time.time() < end:
        ports = [p for p in find_ports() if p not in exclude]
        if ports:
            return ports[0]
        time.sleep(0.2)
    return None


def open_port(dev):
    import serial
    ser = serial.Serial(dev, baudrate=38400, timeout=0.05, write_timeout=2)
    ser.dtr = True     # the firmware only sends its K/B lines while DTR is asserted
    ser.rts = True
    return ser


# ---------------------------------------------------------------------------
# Command framing (reuse serialtool's obfuscation/CRC framing)

def build_cmd(cmd_id, payload=b""):
    import msg as mm
    m = mm.Msg.make(cmd_id, len(payload))
    if payload:
        m.buf[4:4 + len(payload)] = payload
    return mm.make_packet(m.buf)


def send_cmd(ser, cmd_id, payload=b""):
    ser.write(build_cmd(cmd_id, payload))
    ser.flush()


# ---------------------------------------------------------------------------
# ASCII line reader (B / K telemetry lines)

def read_lines(ser, seconds, match=None):
    """Collect CRLF-terminated ASCII lines for up to `seconds`. If `match` is a
    compiled regex, return early as soon as a line matches it."""
    end = time.time() + seconds
    buf = bytearray()
    lines = []
    while time.time() < end:
        data = ser.read(256)
        if not data:
            time.sleep(0.01)
            continue
        buf.extend(data)
        while b"\n" in buf:
            raw, _, rest = buf.partition(b"\n")
            del buf[:]
            buf.extend(rest)
            s = raw.decode("ascii", "replace").strip()
            if not s:
                continue
            lines.append(s)
            if match is not None and match.search(s):
                return lines
    return lines


_K_OK = re.compile(r"\bok=(\d+)")
_K_FW = re.compile(r"\bfw=(\S+)")
_K_BL = re.compile(r"^K bl ")


def dfu_check(ser, seconds=3.0):
    """Send 0x05E1 and parse the 'K bl ... ok=<n> fw=<hash>' reply.
    Returns (ok: bool|None, fw: str|None). ok is None if no reply arrived."""
    send_cmd(ser, CMD_DFU_CHECK)
    for line in read_lines(ser, seconds, match=_K_BL):
        if _K_BL.match(line):
            print("  <", line)
            mo = _K_OK.search(line)
            mf = _K_FW.search(line)
            return (bool(mo and mo.group(1) == "1"),
                    mf.group(1) if mf else None)
    return (None, None)


# ---------------------------------------------------------------------------
# Bootloader beacon wait (framed 0x0518 messages)

def wait_for_beacons(ser, seconds, need=3):
    """Return True once `need` 0x0518 beacons carrying '7.00.07' are seen."""
    import msg as mm
    end = time.time() + seconds
    acc = bytearray()
    seen = 0
    while time.time() < end:
        data = ser.read(256)
        if data:
            acc.extend(data)
            while True:
                m = mm.fetch(acc)
                if m is None:
                    break
                if m.get_msg_type() == MSG_DEV_INFO:
                    ver = _beacon_ver(m)
                    if "7.00" in ver:
                        seen += 1
                        if seen == 1:
                            print("  beacon: bootloader {}".format(ver))
                        if seen >= need:
                            return True
        else:
            time.sleep(0.01)
    return False


def _beacon_ver(m):
    b = m.buf
    end = b.find(b"\x00", 20, 36)
    if end < 0:
        end = min(36, len(b))
    try:
        return b[20:end].decode("ascii", "replace")
    except Exception:
        return ""


# ---------------------------------------------------------------------------
# serialtool page programming

def run_serialtool(port, image, resume=False):
    cmd = [sys.executable, CLI_PY, "flash", "--port", port, "--bl-ver", BL_VER]
    if resume:
        cmd.append("--resume")
    cmd.append(image)
    print("  $", " ".join(cmd))
    return subprocess.call(cmd) == 0


# ---------------------------------------------------------------------------
# Optional artifact download

def gh_download(run_id, dest):
    os.makedirs(dest, exist_ok=True)
    cmd = ["gh", "run", "download"]
    if run_id:
        cmd.append(str(run_id))
    cmd += ["-D", dest]
    print("  $", " ".join(cmd))
    if subprocess.call(cmd) != 0:
        return None
    for root, _dirs, files in os.walk(dest):
        for f in files:
            if f.endswith(".rescueops.bin") or f == "f4hwn.rescueops.bin":
                return os.path.join(root, f)
    # fall back to any .bin
    for root, _dirs, files in os.walk(dest):
        for f in files:
            if f.endswith(".bin"):
                return os.path.join(root, f)
    return None


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Hands-off ALERT-X1 reflasher")
    ap.add_argument("--file", "-f", help="firmware image (.bin) to flash")
    ap.add_argument("--run", help="GitHub Actions run id to `gh run download` (else latest)")
    ap.add_argument("--download-dir", default=os.path.join(HERE, "_hotflash_dl"),
                    help="where to place a downloaded artifact")
    ap.add_argument("--port", help="serial port (else auto-detect by VID 36B7)")
    ap.add_argument("--expect-hash", help="require this git hash in the post-flash reply")
    ap.add_argument("--force", action="store_true",
                    help="flash even if the bootloader guard (0x05E1) does not pass")
    ap.add_argument("--beacon-timeout", type=float, default=15.0)
    args = ap.parse_args()

    # 1. Resolve the image.
    image = args.file
    if not image:
        print("Downloading artifact..")
        image = gh_download(args.run, args.download_dir)
    if not image or not os.path.isfile(image):
        print("No firmware image (pass --file or --run).")
        return 2
    print("Image: {} ({} bytes)".format(image, os.path.getsize(image)))

    # 2. Find the radio.
    port = args.port or wait_for_port(10)
    if not port:
        print("No radio found on USB VID {:04X}.".format(VID))
        return 2
    print("Radio on {}".format(port))

    before_hash = None
    already_in_bootloader = False

    # 3-4. Ask the app for the guard, then to enter DFU.
    try:
        ser = open_port(port)
    except Exception as e:
        print("Cannot open {}: {}".format(port, e))
        return 2

    try:
        print("Checking bootloader guard (0x05E1)..")
        ok, before_hash = dfu_check(ser)
        if ok is None:
            print("  no reply - the radio may already be in the bootloader.")
            already_in_bootloader = True
        else:
            print("  guard ok={} running fw={}".format(int(ok), before_hash))
            if not ok and not args.force:
                print("Bootloader guard failed; refusing to flash. Use --force to override.")
                return 1
            print("Requesting DFU entry (0x05E0)..")
            magic = ENTER_DFU_MAGIC.to_bytes(4, "little")
            send_cmd(ser, CMD_ENTER_DFU, magic)
            read_lines(ser, 1.0, match=re.compile(r"^K dfu "))
    finally:
        try:
            ser.close()
        except Exception:
            pass

    # 5. The radio resets; its port re-enumerates. Wait for the bootloader beacons.
    time.sleep(1.0)
    port = args.port or wait_for_port(args.beacon_timeout) or port
    print("Bootloader port: {}".format(port))

    beacons = False
    try:
        ser = open_port(port)
        print("Waiting for 0x0518 beacons..")
        beacons = wait_for_beacons(ser, args.beacon_timeout)
        ser.close()
    except Exception as e:
        print("  beacon wait failed: {}".format(e))

    # 6. Program the pages. Normal path when beacons appeared; otherwise resend
    #    from page 0 without the beacon wait (a bootloader stuck in state 2).
    ok = run_serialtool(port, image, resume=not beacons)
    if not ok and beacons:
        print("Flash failed; retrying with --resume (resend from page 0)..")
        ok = run_serialtool(port, image, resume=True)
    if not ok:
        print("Flashing failed. The old app is intact; power-cycle to recover, "
              "or rerun (a bootloader left mid-flash is retried from page 0).")
        return 1

    # 7. Wait for the app to come back and confirm the new git hash.
    time.sleep(1.5)
    port = args.port or wait_for_port(15) or port
    try:
        ser = open_port(port)
    except Exception as e:
        print("Flashed, but cannot reopen {} to confirm: {}".format(port, e))
        return 0

    after_hash = None
    try:
        # Catch the boot 'B' line if it is still coming, then query 0x05E1.
        for line in read_lines(ser, 2.0, match=re.compile(r"^B ver=")):
            if line.startswith("B ver="):
                print("  <", line)
        _ok, after_hash = dfu_check(ser)
    finally:
        ser.close()

    print("Flashed. before fw={} after fw={}".format(before_hash, after_hash))
    if args.expect_hash:
        if after_hash == args.expect_hash:
            print("Confirmed: running the expected build ({}).".format(after_hash))
            return 0
        print("::warning:: expected fw={} but radio reports fw={}"
              .format(args.expect_hash, after_hash))
        return 1
    if after_hash and before_hash and after_hash == before_hash and not already_in_bootloader:
        print("::warning:: git hash unchanged ({}); did the new image differ?"
              .format(after_hash))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
