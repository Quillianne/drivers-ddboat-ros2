#!/usr/bin/env python3
"""Interactive emulator for DDBoat serial devices.

This version keeps pseudo‑terminal pairs open and tries to mimic the
behaviour of the real hardware a little more closely.  It still does **not**
create actual I²C devices but it is sufficient for exercising the ROS2
drivers in this repository.

Run as root to create convenient ``/dev/tty*`` symlinks.
"""

import os
import pty
import random
import select
import signal
import threading
import time


class Device:
    """Represents one pseudo serial device."""

    def __init__(self, path: str):
        master, slave = pty.openpty()
        self.master = master
        self.slave_name = os.ttyname(slave)
        self.path = path
        if os.geteuid() == 0:
            try:
                if os.path.exists(path) or os.path.islink(path):
                    os.remove(path)
                os.symlink(self.slave_name, path)
                print(f"Created {path} -> {self.slave_name}")
            except PermissionError as exc:
                print(f"Cannot create symlink for {path}: {exc}")
        else:
            print(f"Use {self.slave_name} for {path} (run as root to symlink)")

    def write(self, data: bytes | str) -> None:
        if isinstance(data, str):
            data = data.encode()
        os.write(self.master, data)


def gps_line() -> str:
    lat = random.uniform(-90, 90)
    lon = random.uniform(-180, 180)
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    lat = abs(lat) * 100
    lon = abs(lon) * 100
    return f"$GPGLL,{lat:.4f},{ns},{lon:.4f},{ew},000000,A*00\r\n"


def enc_frame(counter: int) -> bytes:
    now = int(time.time())
    pos = counter & 0xFFFF
    frame = bytearray(17)
    frame[0] = 0xFF
    frame[1] = 0x0D
    frame[2] = (now >> 24) & 0xFF
    frame[3] = (now >> 16) & 0xFF
    frame[4] = (now >> 8) & 0xFF
    frame[5] = now & 0xFF
    frame[6] = 0  # dir_right
    frame[7] = 0  # dir_left
    frame[8] = (pos >> 8) & 0xFF
    frame[9] = pos & 0xFF
    frame[10] = (pos >> 8) & 0xFF
    frame[11] = pos & 0xFF
    frame[12] = 0x04  # volt_right (arbitrary)
    frame[13] = 0xD0
    frame[14] = 0x04  # volt_left
    frame[15] = 0xD0
    frame[16] = 0xAA
    return bytes(frame)


def start_gps(dev: Device) -> None:
    def writer():
        while True:
            dev.write(gps_line())
            time.sleep(1)

    def reader():
        buf = b""
        while True:
            r, _, _ = select.select([dev.master], [], [], 0.1)
            if dev.master in r:
                buf += os.read(dev.master, 1024)
                if b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line.strip().startswith(b"$PMTK"):
                        dev.write("$PMTK001,0,3*30\r\n")

    threading.Thread(target=writer, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()


def start_arduino(dev: Device) -> None:
    def reader():
        buf = b""

        def send_later(msg: str, delay: float = 0.0) -> None:
            def _():
                if delay:
                    time.sleep(delay)
                dev.write(msg + "\n")

            threading.Thread(target=_, daemon=True).start()

        while True:
            r, _, _ = select.select([dev.master], [], [], 0.1)
            if dev.master in r:
                buf += os.read(dev.master, 1024)
                while b";" in buf:
                    idx = buf.index(b";")
                    cmd = buf[: idx + 1].decode(errors="ignore")
                    buf = buf[idx + 1 :]
                    if not cmd:
                        continue
                    c = cmd[0]
                    if c == "I":
                        send_later("CALIB_OK", 1.0)
                    elif c in "CZRXSE":
                        send_later("OK")

    threading.Thread(target=reader, daemon=True).start()


def start_encoders(dev: Device) -> None:
    delay_ms = 100
    counter = 0

    def writer():
        nonlocal counter
        while True:
            dev.write(enc_frame(counter))
            counter += 1
            time.sleep(delay_ms / 1000.0)

    def reader():
        nonlocal delay_ms
        buf = b""
        while True:
            r, _, _ = select.select([dev.master], [], [], 0.1)
            if dev.master in r:
                buf += os.read(dev.master, 1024)
                while b";" in buf:
                    idx = buf.index(b";")
                    cmd = buf[: idx + 1].decode(errors="ignore")
                    buf = buf[idx + 1 :]
                    if cmd.startswith("D"):
                        try:
                            delay_ms = int(cmd[1:-1])
                        except ValueError:
                            pass

    threading.Thread(target=writer, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()


def start_radio(dev: Device) -> None:
    """Echo incoming frames with a canned reply and periodically send a ping."""

    def writer() -> None:
        while True:
            dev.write("2:1:4:pong\n")
            time.sleep(1.0)

    def reader() -> None:
        buf = b""
        while True:
            r, _, _ = select.select([dev.master], [], [], 0.1)
            if dev.master in r:
                buf += os.read(dev.master, 1024)
                while b"\n" in buf:
                    _line, buf = buf.split(b"\n", 1)
                    dev.write("2:1:4:pong\n")

    threading.Thread(target=writer, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()


def main() -> None:
    gps = Device("/dev/ttyGPS0")
    arduino = Device("/dev/ttyV0")
    enc = Device("/dev/ttyENC1")
    radio = Device("/dev/ttyLORA1")

    start_gps(gps)
    start_arduino(arduino)
    start_encoders(enc)
    start_radio(radio)

    print("Devices emulation running. Press Ctrl+C to stop.")
    signal.pause()


if __name__ == "__main__":
    main()
