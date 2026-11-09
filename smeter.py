#!/usr/bin/env python3
"""Log the IC-9700 S-meter over CI-V (/dev/ttyUSB0, 19200, address 0xA2), timestamped, until killed.
usage: smeter.py OUT.tsv [PERIOD_S]   lines: unix_time  raw(0-255; ~120 = S9, 241 = S9+60)"""
import sys, time, serial
out, period = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
CMD = bytes([0xFE, 0xFE, 0xA2, 0xE0, 0x15, 0x02, 0xFD])
with serial.Serial("/dev/ttyUSB0", 19200, timeout=0.3) as s, open(out, "a", buffering=1) as f:
    while True:
        t = time.time(); s.reset_input_buffer(); s.write(CMD)
        buf = s.read_until(b"\xfd", 64)
        if buf.endswith(CMD): buf += s.read_until(b"\xfd", 64)          # the bus echoes our command first
        i = buf.find(bytes([0xFE, 0xFE, 0xE0, 0xA2, 0x15, 0x02]))
        if i >= 0 and len(buf) >= i + 9:
            b = buf[i + 6:i + 8]; v = int(f"{b[0]:02x}{b[1]:02x}")         # BCD 0000-0255
            f.write(f"{t:.2f}\t{v}\n")
        time.sleep(max(0, period - (time.time() - t)))
