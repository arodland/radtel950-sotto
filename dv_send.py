#!/usr/bin/env python3
"""Send a DV over from the IC-9700: speech (8 kHz wav) -> the radio's sotto encoder (sotto_enc_fx_lo)
-> V2 frames -> continuous-phase 4-FSK audio -> the 9700's USB input with CI-V PTT (FM-D mode,
DATA MOD = USB). For testing the radio's own DV receiver. usage: dv_send.py SPEECH.wav [LEVEL]"""
import os, subprocess, sys, tempfile, time, wave
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import dvframe as d, dv_live as L
FS = 48000

def frames_audio(frames, level=0.3):
    """48-bit superframes (bit arrays) -> the over as continuous-phase 4-FSK audio at 48 kHz"""
    syms = np.concatenate([d.preamble()] + [d.frame_symbols(f) for f in frames] + [np.array([0, 3] * 16)])
    ph, out, n = 0.0, [], np.arange(40)
    for k in syms:
        out.append(np.sin(ph + 2 * np.pi * 600 * (k + 1) * n / FS)); ph += 2 * np.pi * 600 * (k + 1) * 40 / FS
    return (level * 32767 * np.concatenate(out)).astype(np.int16)

def dv_audio(path, level=0.3):
    w = wave.open(path); assert w.getframerate() == 8000 and w.getnchannels() == 1
    pcm = w.readframes(w.getnframes())
    with tempfile.TemporaryDirectory() as td:
        open(f"{td}/in.raw", "wb").write(pcm)
        subprocess.run([f"{HERE}/sotto_enc_fx_lo", "-1200", f"{td}/in.raw", f"{td}/out.bit"], check=True)
        bits = open(f"{td}/out.bit", "rb").read()
    frames = [np.unpackbits(np.frombuffer(bits[i:i + 6], np.uint8)) for i in range(0, len(bits) - 5, 6)]
    return frames_audio(frames, level), len(frames)

if __name__ == "__main__":
    y, nf = dv_audio(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.3)
    civ = L.Civ()
    try:
        civ.ptt(True); time.sleep(0.4)
        subprocess.run(["pw-play", f"--target={L.Parrot.SINK}", f"--rate={FS}", "--channels=1", "--format=s16", "--raw", "-"],
                       input=y.tobytes(), check=False)
        time.sleep(0.2)
    finally:
        civ.ptt(False)
    print(f"sent {nf} frames ({nf * 0.04:.1f} s of speech)")
