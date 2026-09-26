#!/usr/bin/env python3
"""
iquery.py — safe serial probe for the iobox3 bench/car box.

WHY THIS EXISTS (probing issue, 2026-09-22):
  * The bench CP2102 adapter ties RTS to EN: opening the port with a normal
    serial open asserts RTS/DTR and PULSES a reset — and DTR/RTS assertion
    can wedge the boot into ROM "try 0x400805e4" spam.
  * Queries sent during the 3-4 s boot window are lost (empty reply).
  * 2026-09-22 (2nd): RAW OPEN FIX — open the fd with os.open() and clear
    RTS/DTR via TIOCMBIC ioctl BEFORE the tty layer asserts them. Verified:
    the box answers in <1 s with NO reset (no boot banner, counters keep
    climbing). This replaces the old "wait for boot banner" workaround.

RULES the script enforces:
  1. os.open() + TIOCMBIC(DTR|RTS) immediately — never let the port
     handshake assert the modem lines (that is what resets the box).
  2. Default Linux tty open may assert RTS/DTR too; cleared via ioctl.
  3. Keep the port open: query many times, close once at the end.
  4. Never assert RTS/DTR, even to "reset" the box.

Usage:
  python3 tools/iquery.py [port] [-n queries] [-i seconds] [-r raw]

  port   serial device            (default /dev/ttyUSB0)
  -n N   number of `?` queries    (default 1)
  -i S   seconds between queries  (default 2)
  -r     dump raw bytes instead of decoded text

Exit: 0 on at least one non-empty reply, 1 otherwise.
"""
import argparse, fcntl, os, re, struct, sys, termios, time


def open_no_reset(port: str):
    """Open a CP2102 port WITHOUT pulsing DTR/RTS (no board reset)."""
    TIOCMBIC = 0x5427          # clear modem bits
    TIOCM_RTS = 0x40
    TIOCM_DTR = 0x02
    TIOCM_CLEAR = TIOCM_RTS | TIOCM_DTR
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.ioctl(fd, TIOCMBIC, struct.pack("I", TIOCM_CLEAR))
    a = termios.tcgetattr(fd)
    a[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    a[1] &= ~(termios.OPOST)
    a[2] &= ~(termios.CRTSCTS)
    a[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)
    a[3] |= termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    os.set_blocking(fd, False)
    return fd


def read_avail(fd, secs: float) -> bytes:
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        try:
            d = os.read(fd, 4096)
            if d:
                buf += d
        except BlockingIOError:
            time.sleep(0.02)
    return buf


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("-n", type=int, default=1)
    ap.add_argument("-i", type=float, default=2.0)
    ap.add_argument("-r", "--raw", action="store_true")
    a = ap.parse_args()

    fd = open_no_reset(a.port)

    buf = b""
    ok = False
    for i in range(a.n):
        os.write(fd, b"?\n")                  # single query; box replies on '\n'
        time.sleep(0.3)
        resp = read_avail(fd, 1.2)
        if not resp.strip():
            resp = read_avail(fd, 1.5)
        if resp.strip():
            buf += resp
            ok = True
        if i + 1 < a.n:
            time.sleep(a.i)

    os.close(fd)
    txt = buf.decode("utf-8", "replace")
    if a.raw:
        sys.stdout.buffer.write(buf)
    else:
        # Strip ROM bootloader progress lines so status is readable.
        txt = re.sub(
            r"(?m)^(esp_loader|load:0x[0-9a-f]+|entry 0x[0-9a-f]+|rst:0x.*|try 0x[0-9a-f]+|configsip:.*|clk_drv:.*|mode:.*|boot:.*)$",
            "", txt)
        print(txt.strip())
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())