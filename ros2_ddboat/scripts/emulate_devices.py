#!/usr/bin/env python3
"""Simple emulator for DDBoat serial devices.

This script creates pseudo serial ports and sends dummy data so the
ROS2 drivers can run without real hardware.
Run as root if you want symbolic links to /dev/tty* to be created.
"""
import os
import pty
import threading
import time
import random
import signal

# Map device path to a line generator function
DEVICE_GENS = {
    '/dev/ttyGPS0': lambda: gps_line(),
    '/dev/ttyGPS1': lambda: gps_line(),
    '/dev/ttyGPS2': lambda: gps_line(),
    '/dev/ttyV0': lambda: 'OK\n',
    '/dev/ttyENC0': lambda: enc_line(),
    '/dev/ttyENC1': lambda: enc_line(),
    '/dev/ttyENC2': lambda: enc_line(),
    '/dev/ttyLORA1': lambda: '2:1:4:pong\n',
    '/dev/ttyLORA2': lambda: '2:1:4:pong\n',
}


def gps_line():
    lat = random.uniform(-90, 90)
    lon = random.uniform(-180, 180)
    ns = 'N' if lat >= 0 else 'S'
    ew = 'E' if lon >= 0 else 'W'
    lat = abs(lat) * 100
    lon = abs(lon) * 100
    return f"$GPGLL,{lat:.4f},{ns},{lon:.4f},{ew},000000,A*00\r\n"


def enc_line():
    t = int(time.time())
    return f"{t} 0 0 0 0 0 0\n"


def create_device(path, gen_func):
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    if os.geteuid() == 0:
        try:
            if os.path.exists(path) or os.path.islink(path):
                os.remove(path)
            os.symlink(slave_name, path)
            print(f"Created {path} -> {slave_name}")
        except PermissionError as e:
            print(f"Cannot create symlink for {path}: {e}")
    else:
        print(f"Use {slave_name} for {path} (run as root to symlink)")

    def writer():
        while True:
            os.write(master, gen_func().encode())
            time.sleep(1)

    threading.Thread(target=writer, daemon=True).start()


def main():
    for path, gen in DEVICE_GENS.items():
        create_device(path, gen)
    print("Devices emulation running. Press Ctrl+C to stop.")
    signal.pause()


if __name__ == '__main__':
    main()
