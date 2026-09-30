#!/usr/bin/env python3
#
# Copyright (c) 2026 ALERT-X1
# Licensed under the MIT License (the "License"); see tools/serialtool for the
# text. Distributed WITHOUT WARRANTY OF ANY KIND.
#
"""Hands-off reflash for the ALERT-X1 build (plan section 6).

After the one unavoidable manual flash, every later flash runs from the PC with
no button held:

  1. (optional) download the CI artifact with `gh run download` from this
     fork's own repository (never the upstream remote), and check that the
     image is an X1 build that can itself be reflashed this way;
  2. find the radio by USB VID 0x36B7;
  3. ask the running firmware for the on-device bootloader guard (0x05E1) and
     stop unless it passes;
  4. ask it to enter DFU (0x05E0, payload = magic 0x44465521) - the firmware
     re-checks the same guard (and refuses while transmitting), arms a no-init
     cell and resets into the stock bootloader's DFU;
  5. wait for the bootloader's 0x0518 beacons carrying "7.00.07", on whatever
     port it enumerates as (its USB PID differs from the app's, so Windows
     gives it another COM number);
  6. drive tools/serialtool to write the 256-byte pages (--bl-ver 7.00);
  7. wait for the app to come back and confirm the git hash changed.

It never holds PTT and never sends 0x0516, so the bootloader is never written;
the worst outcome of any failure is a power cycle. If the bootloader was left
mid-flash by an earlier interrupted run it no longer beacons; rerun with
--resume to resend from page 0. That is the only case --resume works in (a
freshly entered bootloader rejects every page until the handshake), so it is
never tried unless asked for or after a flash that got past the handshake.
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
DEFAULT_REPO = "cdomotor-g/quansheng_alert_v3"
ARTIFACT_PATTERN = "alert-v3-rescueops-*"
# Strings an image must hold to be worth flashing hands-off: dfu.c's boot line
# and ENTER_DFU reply. An image without them (upstream F4HWN, an old build)
# would flash fine and then need PTT for every flash after it.
IMAGE_MARKERS = (b"B ver=", b"K dfu ok=")
SERIALTOOL_STALL_S = 30     # serialtool --timeout: no progress in any state


# ---------------------------------------------------------------------------
# Port discovery

def find_ports():
    from serial.tools import list_ports
    return sorted(p.device for p in list_ports.comports() if p.vid == VID)


def wait_for_gone(dev, timeout):
    """Wait for `dev` to drop off the bus (the reset), at most `timeout` s."""
    end = time.time() + timeout
    while time.time() < end:
        if dev not in find_ports():
            return True
        time.sleep(0.1)
    return False


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
    import serial
    end = time.time() + seconds
    buf = bytearray()
    lines = []
    while time.time() < end:
        try:
            data = ser.read(256)
        except serial.SerialException:
            # The radio reset and its CDC port vanished (0x05E0 does this, often
            # before the K line is out); callers treat "no reply" as "maybe reset".
            break
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


def dfu_check(ser, seconds=6.0, tries=2):
    """Send 0x05E1 and parse the 'K bl ... ok=<n> fw=<hash>' reply.
    Returns (ok: bool|None, fw: str|None). ok is None if no reply arrived.

    The ALERT app does not service USB while its audio-pin census runs (~3-4 s
    at entry and on request), so one short wait is not proof of silence: the
    command is queued and answered afterwards."""
    for _ in range(tries):
        send_cmd(ser, CMD_DFU_CHECK)
        for line in read_lines(ser, seconds, match=_K_BL):
            if _K_BL.match(line):
                print("  <", line)
                mo = _K_OK.search(line)
                mf = _K_FW.search(line)
                return (bool(mo and mo.group(1) == "1"),
                        mf.group(1) if mf else None)
    return (None, None)


_K_DFU = re.compile(r"^K dfu ")
_K_ERR = re.compile(r"\berr=(\S+)")


def enter_dfu(ser):
    """Send 0x05E0. Returns (True, None) on 'K dfu ok=1', (False, err) on ok=0,
    (None, None) when no reply arrived (the reset can beat the line out)."""
    send_cmd(ser, CMD_ENTER_DFU, ENTER_DFU_MAGIC.to_bytes(4, "little"))
    for line in read_lines(ser, 2.0, match=_K_DFU):
        if _K_DFU.match(line):
            print("  <", line)
            mo = _K_OK.search(line)
            if mo and mo.group(1) == "1":
                return True, None
            me = _K_ERR.search(line)
            return False, me.group(1) if me else "?"
    return None, None


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
    """True only if serialtool saw the last page acknowledged. Bounded: it gives
    up after SERIALTOOL_STALL_S without progress, so this never hangs."""
    cmd = [sys.executable, CLI_PY, "flash", "--port", port, "--bl-ver", BL_VER,
           "--timeout", str(SERIALTOOL_STALL_S)]
    if resume:
        cmd.append("--resume")
    cmd.append(image)
    print("  $", " ".join(cmd))
    return subprocess.call(cmd) == 0


# ---------------------------------------------------------------------------
# Optional artifact download

def default_repo():
    """owner/name of this clone's origin. Always passed to gh with -R: with an
    upstream remote as well and no gh default set, gh resolves to upstream and
    would find no run, or download upstream firmware."""
    try:
        url = subprocess.check_output(["git", "remote", "get-url", "origin"], cwd=HERE,
                                      text=True, stderr=subprocess.DEVNULL).strip()
        m = re.search(r"github\.com[:/]([^/]+/[^/]+?)(?:\.git)?/?$", url)
        if m:
            return m.group(1)
    except Exception:
        pass
    return DEFAULT_REPO


def current_branch():
    try:
        return subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=HERE,
                                       text=True, stderr=subprocess.DEVNULL).strip() or None
    except Exception:
        return None


def latest_run(repo, branch):
    """Database id of the newest successful main.yml run on `branch`, or None."""
    cmd = ["gh", "run", "list", "-R", repo, "-w", "main.yml", "-s", "success", "-L", "1",
           "--json", "databaseId", "-q", ".[0].databaseId"]
    if branch:
        cmd += ["-b", branch]
    print("  $", " ".join(cmd))
    try:
        out = subprocess.check_output(cmd, text=True).strip()
    except Exception as e:
        print("  gh run list failed: {}".format(e))
        return None
    return out or None


def gh_download(run_id, dest, repo, branch):
    if not run_id:
        run_id = latest_run(repo, branch)
        if not run_id:
            print("No successful main.yml run found on {} {}; pass --run."
                  .format(repo, branch or "(any branch)"))
            return None
        print("  latest successful run: {}".format(run_id))
    dest = os.path.join(dest, str(run_id))
    os.makedirs(dest, exist_ok=True)
    cmd = ["gh", "run", "download", str(run_id), "-R", repo, "-p", ARTIFACT_PATTERN, "-D", dest]
    print("  $", " ".join(cmd))
    if subprocess.call(cmd) != 0:
        return None
    for root, _dirs, files in os.walk(dest):
        for f in files:
            if f.endswith(".rescueops.bin"):
                return os.path.join(root, f)
    return None


def check_image(path):
    """Missing markers (empty = fine) for an image that must keep the hands-off route."""
    with open(path, "rb") as f:
        data = f.read()
    return [m.decode() for m in IMAGE_MARKERS if m not in data]


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Hands-off ALERT-X1 reflasher")
    ap.add_argument("--file", "-f", help="firmware image (.bin) to flash")
    ap.add_argument("--run", help="GitHub Actions run id to `gh run download` (else the latest "
                    "successful main.yml run on --branch)")
    ap.add_argument("--repo", default=None,
                    help="GitHub repo for gh (default: this clone's origin, else {})"
                    .format(DEFAULT_REPO))
    ap.add_argument("--branch", default=None,
                    help="branch whose latest run to download (default: the checked-out one)")
    ap.add_argument("--download-dir", default=os.path.join(HERE, "_hotflash_dl"),
                    help="where to place a downloaded artifact")
    ap.add_argument("--port", help="serial port (else auto-detect by VID 36B7)")
    ap.add_argument("--expect-hash", help="require this git hash in the post-flash reply")
    ap.add_argument("--force", action="store_true",
                    help="flash even if the bootloader guard (0x05E1) does not pass")
    ap.add_argument("--skip-image-check", action="store_true",
                    help="flash an image without the X1 DFU strings (the next flash will "
                    "need PTT held at power-on)")
    ap.add_argument("--resume", action="store_true",
                    help="the radio is in a bootloader left mid-flash by an interrupted "
                    "run (no beacons): resend from page 0")
    ap.add_argument("--beacon-timeout", type=float, default=15.0)
    args = ap.parse_args()

    # 1. Resolve the image.
    image = args.file
    if not image:
        repo = args.repo or default_repo()
        branch = args.branch or current_branch()
        print("Downloading artifact from {}..".format(repo))
        image = gh_download(args.run, args.download_dir, repo, branch)
    if not image or not os.path.isfile(image):
        print("No firmware image (pass --file or --run).")
        return 2
    print("Image: {} ({} bytes)".format(image, os.path.getsize(image)))
    missing = check_image(image)
    if missing and not args.skip_image_check:
        print("Refusing: the image lacks {} - not an ALERT-X1 build, so the radio could "
              "not be reflashed hands-off after it. Use --skip-image-check to override."
              .format(", ".join(repr(m) for m in missing)))
        return 2

    # 2. Find the radio.
    port = args.port or wait_for_port(10)
    if not port:
        print("No radio found on USB VID {:04X}.".format(VID))
        return 2
    print("Radio on {}".format(port))

    before_hash = None
    already_in_bootloader = False
    app_port = port

    # 3-4. Ask the app for the guard, then to enter DFU.
    try:
        ser = open_port(port)
    except Exception as e:
        print("Cannot open {}: {}".format(port, e))
        return 2

    try:
        if args.resume:
            print("--resume: treating {} as a bootloader left mid-flash.".format(port))
        else:
            print("Checking bootloader guard (0x05E1)..")
            ok, before_hash = dfu_check(ser)
            if ok is None:
                # Silence is not proof of a bootloader (the app may be busy, or
                # run older firmware with no 0x05E1): look for its beacons.
                print("  no reply; listening for bootloader beacons..")
                if not wait_for_beacons(ser, 3.0, need=2):
                    print("Neither the app nor a bootloader answers on {}. If an earlier "
                          "flash was interrupted mid-way, rerun with --resume; otherwise "
                          "check the port, or power-cycle the radio.".format(port))
                    return 2
                already_in_bootloader = True
            else:
                print("  guard ok={} running fw={}".format(int(ok), before_hash))
                if not ok and not args.force:
                    print("Bootloader guard failed; refusing to flash. Use --force to override.")
                    return 1
                print("Requesting DFU entry (0x05E0)..")
                entered, err = enter_dfu(ser)
                if entered is False:
                    print("The radio refused DFU entry (err={}){}.".format(
                        err, " - it is transmitting; try again when it is not"
                        if err == "tx" else ""))
                    return 1
                if entered is None:
                    print("  no K dfu reply; watching for the reset anyway..")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    beacons = already_in_bootloader
    if not already_in_bootloader and not args.resume:
        # 5. The radio resets and re-enumerates, as the bootloader's own USB
        #    device (another PID, so possibly another COM number, even when
        #    --port named the app's): wait for the app's port to go, then take
        #    whichever VID-matching port appears.
        wait_for_gone(port, 5.0)
        time.sleep(0.5)
        port = wait_for_port(args.beacon_timeout) or port
        print("Bootloader port: {}".format(port))
        try:
            ser = open_port(port)
            print("Waiting for 0x0518 beacons..")
            beacons = wait_for_beacons(ser, args.beacon_timeout)
            ser.close()
        except Exception as e:
            print("  beacon wait failed: {}".format(e))
        if not beacons:
            print("No bootloader beacons on {}. Nothing was written: power-cycle the "
                  "radio to get the old app back.".format(port))
            return 1

    # 6. Program the pages: the beacon wait and handshake when the bootloader is
    #    fresh; --resume only when asked for, or after a first attempt that may
    #    have got past the handshake (serialtool gives up on a stall either way).
    if args.resume and not beacons:
        ok = run_serialtool(port, image, resume=True)
    else:
        ok = run_serialtool(port, image)
        if not ok:
            print("Flash failed; retrying with --resume (resend from page 0)..")
            ok = run_serialtool(port, image, resume=True)
    if not ok:
        print("Flashing failed. If no page was written the old app is intact; power-cycle "
              "to recover. If it stopped mid-way, rerun with --resume.")
        return 1

    # 7. Wait for the app to come back and confirm the new git hash.
    time.sleep(1.5)
    port = wait_for_port(15) or args.port or app_port
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
