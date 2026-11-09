#!/usr/bin/env python3
"""Record the radio's mic (dvtx R command: DC removed, x16, before the EQ) to a wav, no TX.
usage: mic_cap.py SECS OUT.wav"""
import sys, wave, serial
secs, out = int(sys.argv[1]), sys.argv[2]
with serial.Serial("/dev/ttyUSB2", 230400, timeout=2) as s:
    s.reset_input_buffer(); s.write(f"R{secs}\r".encode())
    n = int(s.readline().split()[1]); print("recording", n / 8000, "s", flush=True)
    s.timeout = secs + 5                      # pyserial's timeout bounds the whole read
    d = s.read(2 * n); s.timeout = 2
    print(s.read_until(b"DONE").decode(errors="replace").strip())
    if len(d) < 2 * n: sys.exit(f"short read: {len(d) // 2} of {n} samples")
w = wave.open(out, "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(8000); w.writeframes(d); w.close()
