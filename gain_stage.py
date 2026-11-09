#!/usr/bin/env python3
"""Mic gain staging: for each chip-0 register setting (dvtx W writes), capture 6 s (R command)
while the PC plays 1.5 s silence / 3 s pink noise / 1.5 s silence near the radio's mic,
then 3 s with nothing playing for the floor.
Prints per-band SNR (pink vs silence), the level during noise, and the peak. Restores the
firmware's settings at the end. usage: gain_stage.py"""
import os, subprocess, sys, threading, time
import numpy as np, serial
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mic_eq import make_stimulus, STIM

def radio_port():
    """the RT-950's USB-serial adapter re-enumerates (ttyUSB2 -> ttyUSB3 ...) when RF from the
    antenna resets it during TX: take whichever ttyUSB isn't the IC-9700's (USB0 CI-V, USB1)"""
    import glob, time
    for _ in range(40):
        p = sorted(x for x in glob.glob("/dev/ttyUSB*") if x not in ("/dev/ttyUSB0", "/dev/ttyUSB1"))
        if p: return p[0]
        time.sleep(0.25)
    raise SystemExit("radio serial port not found")

BANDS = [(100, 300), (300, 800), (800, 1600), (1600, 2400), (2400, 3200)]
BASE = {0x2C: 0x003F, 0x7D: 0xE91F, 0x48: 0xB092}
SETTINGS = [{}, {0x2C: 0x30}, {0x2C: 0x20}, {0x2C: 0x10}, {0x2C: 0x00},
            {0x7D: 0xE910}, {0x7D: 0xE908}, {0x7D: 0xE900},
            {0x2C: 0x20, 0x7D: 0xE93F}, {0x2C: 0x10, 0x7D: 0xE93F},
            {0x48: 0xB091}, {0x48: 0xB090}]

def cap(s, secs):
    time.sleep(0.05); s.reset_input_buffer(); s.write(f"R{secs}\r".encode())
    while not (ln := s.readline()).startswith(b"RAW"): pass       # skip stray "ok" / blank lines
    n = int(ln.split()[1])
    s.timeout = secs + 5; d = s.read(2 * n); s.timeout = 2; s.read_until(b"DONE")
    return np.frombuffer(d, np.int16).astype(float)

def bands(x):
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2 / len(x); f = np.fft.rfftfreq(len(x), 1 / 8000)
    return np.array([10 * np.log10(X[(f >= a) & (f < b)].mean() + 1e-9) for a, b in BANDS])

def main():
    if not os.path.exists(STIM): make_stimulus()
    with serial.Serial(radio_port(), 230400, timeout=2) as s:
        print("setting                      " + " ".join(f"{a}-{b}".rjust(9) for a, b in BANDS) + "   noise dBFS  peak")
        for st in SETTINGS:
            regs = {**BASE, **st}
            for r, v in regs.items(): s.write(f"W0,{r:02x},{v:04x}\r".encode()); s.read_until(b"ok")
            res = {}; th = threading.Thread(target=lambda: res.__setitem__("x", cap(s, 6))); th.start()
            time.sleep(0.3); subprocess.run(["pw-play", "--volume=1.0", STIM], check=True); th.join()
            x = res["x"]; x = x - x.mean()
            # stimulus starts ~0.3 s + 1.5 s in: noise 2.2..4.4 s, silence 0.2..1.6 and 5.0..5.9
            # floor: a separate capture with nothing playing (the speaker adds hiss of its own)
            q = cap(s, 3)[4000:]; q = q - q.mean(); sig = x[int(2.2 * 8000):int(4.4 * 8000)]
            snr = bands(sig) - bands(q)
            lv = 20 * np.log10(np.sqrt(np.mean(sig ** 2)) / 32768); nf = 20 * np.log10(np.sqrt(np.mean(q ** 2)) / 32768)
            pk = 20 * np.log10(np.abs(x).max() / 32768)
            name = " ".join(f"{r:02X}={v:04X}" for r, v in regs.items())
            print(f"{name:28s} " + " ".join(f"{v:9.1f}" for v in snr) + f"   {lv:5.1f} {nf:6.1f} {pk:5.1f}", flush=True)
        for r, v in BASE.items(): s.write(f"W0,{r:02x},{v:04x}\r".encode()); s.read_until(b"ok")

if __name__ == "__main__":
    main()
