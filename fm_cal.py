#!/usr/bin/env python3
"""FM deviation calibration: the PC speaker plays a 1 kHz tone into the RT-950's mic while the radio
sends a timed FM over (F command) with chip-1 register overrides (X command); the IC-9700's audio
level of the tone is compared with the DV tones' level (T command) and the no-carrier static.
usage: fm_cal.py"""
import glob, subprocess, threading, time, wave
import numpy as np, serial
SRC = "alsa_input.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00.analog-stereo"
FS = 48000

def radio_port():
    for _ in range(40):
        p = sorted(x for x in glob.glob("/dev/ttyUSB*") if x not in ("/dev/ttyUSB0", "/dev/ttyUSB1"))
        if p: return p[0]
        time.sleep(0.25)
    raise SystemExit("radio serial port not found")

def send(cmds):
    with serial.Serial(radio_port(), 230400, timeout=0.3) as r:
        for c in cmds: r.write(f"{c}\r".encode()); time.sleep(0.08)
        r.read(200)

def record(secs):
    out = subprocess.run(["pw-record", f"--target={SRC}", f"--rate={FS}", "--channels=1", "--format=s16", "--raw", "-"],
                         capture_output=True, timeout=secs + 3) if False else None
    p = subprocess.Popen(["pw-record", f"--target={SRC}", f"--rate={FS}", "--channels=1", "--format=s16", "--raw", "-"],
                         stdout=subprocess.PIPE)
    d = p.stdout.read(int(secs * FS) * 2); p.terminate(); p.wait()
    return np.frombuffer(d, np.int16).astype(float)

def tone(secs, f=1000, amp=0.1):
    t = np.arange(int(secs * FS)) / FS
    y = (amp * 32767 * np.sin(2 * np.pi * f * t)).astype(np.int16)
    return subprocess.Popen(["pw-play", f"--rate={FS}", "--channels=1", "--format=s16", "--raw", "-"],
                            stdin=subprocess.PIPE), y.tobytes()

def band_db(x, lo, hi):
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2; f = np.fft.rfftfreq(len(x), 1 / FS)
    return 10 * np.log10(X[(f >= lo) & (f < hi)].sum() / len(x) ** 2 * 8 + 1e-12) - 20 * np.log10(32768)

def over(cmd, secs, with_tone):
    res = {}
    th = threading.Thread(target=lambda: res.__setitem__("x", record(secs + 3))); th.start()
    time.sleep(0.5)
    pl = None
    if with_tone:
        pl, data = tone(secs + 1.5)
        threading.Thread(target=lambda: (pl.stdin.write(data), pl.stdin.close()), daemon=True).start()
    time.sleep(0.3); send([f"{cmd}{secs}"]); th.join()
    if pl: pl.wait()
    x = res["x"]; x = x - x.mean()
    return x[int(1.8 * FS):int((1.8 + secs - 1.2) * FS)]          # the middle of the over

if __name__ == "__main__":
    send(["X"])
    x = over("T", 3, False); print(f"DV tones (reference): rms {20 * np.log10(x.std() / 32768):6.1f} dBFS", flush=True)
    time.sleep(3)
    x = record(1.5); print(f"no carrier (static):  rms {20 * np.log10(x.std() / 32768):6.1f} dBFS", flush=True)
    for name, ov in [("current (2C 22, 40 350A)", []), ("2C 3F", ["X1,2C,003F"]),
                     ("40 360A", ["X1,40,360A"]), ("40 370A", ["X1,40,370A"]),
                     ("2C 3F + 40 360A", ["X1,2C,003F", "X1,40,360A"])]:
        time.sleep(3)
        send(["X"] + ov)
        x = over("F", 3, True)
        print(f"FM {name:26s}: 1 kHz {band_db(x, 950, 1050):6.1f} dBFS, total rms {20 * np.log10(x.std() / 32768):6.1f}, "
              f"2-3 kHz (harmonics/noise) {band_db(x, 1950, 3050):6.1f}", flush=True)
    send(["X"])
