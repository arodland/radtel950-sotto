#!/usr/bin/env python3
"""Pre-encoder chain A/B: mic_cap.py recording -> EQ [-> high-cut] [-> gate/compressor] -> sotto 1200
(the firmware's encoder) -> decoder. Writes ab_<name>.wav (codec out) and ab_<name>_pre.wav (encoder in)
to the scratch dir. usage: voice_ab.py MIC.wav"""
import os, re, subprocess, sys, wave
import numpy as np
from scipy.signal import firwin, lfilter
HERE = os.path.dirname(os.path.abspath(__file__))
from dv_rx import SCRATCH
FS = 8000

def eq_fw():
    """eq_coef.h exactly as the firmware runs it (Q format, >> shift, clip)"""
    t = open(os.path.join(HERE, "eq_coef.h")).read()
    sh = int(re.search(r"EQ_SHIFT (\d+)", t).group(1))
    h = np.array([int(v) for v in re.findall(r"-?\d+", t.split("{", 1)[1].split("}")[0])])
    return lambda x: np.clip(np.floor(lfilter(h, 1, x) / 2 ** sh), -32767, 32767)

def highcut(fc, taps=63):
    h = firwin(taps, fc, fs=FS); return lambda x: lfilter(h, 1, x)

def hp300(x):
    a = 1 - np.exp(-2 * np.pi * 300 / FS); return x - lfilter([a], [1, a - 1], x)   # x - one-pole low-pass

def dynamics(gate_db=-40, gate_floor=-18, thr_db=-24, ratio=4, makeup_db=10, att_ms=3, rel_ms=120, hold_ms=80, close_ms=None):
    """1 ms blocks (8 samples, cheap on the MCU). Compressor: peak envelope (fast attack, slow
    release) in dB re full scale, gain reduced above thr. Gate: rms envelope (20 ms, of a 300 Hz high-pass) below gate_db
    for hold_ms -> gate_floor dB (hiss peaks sit only ~10 dB under quiet speech peaks, so a peak
    detector keeps re-opening). Gain smoothed per block, interpolated per sample"""
    def f(x):
        B = 8; n = len(x) // B; X = x[:n * B].reshape(n, B) / 32768
        blk = np.abs(X).max(1); ms = (hp300(X.ravel()).reshape(n, B) ** 2).mean(1); a20 = 1 - np.exp(-1 / 20); pw = 0
        aa, ar = 1 - np.exp(-1 / att_ms), 1 - np.exp(-1 / rel_ms)
        env = 0; g = 1.0; gg = 1.0; held = 0; gs = np.empty(n)
        for i, (p, m) in enumerate(zip(blk, ms)):
            pw += a20 * (m - pw)
            env += (aa if p > env else ar) * (p - env)
            L = 20 * np.log10(env + 1e-9)
            gdb = makeup_db - max(L - thr_db, 0) * (1 - 1 / ratio)
            held = hold_ms if 10 * np.log10(pw + 1e-12) > gate_db else max(held - 1, 0)
            if close_ms is None:                                  # gate folded into the gain
                if not held: gdb += gate_floor
                tgt = 10 ** (gdb / 20)
                g += (0.5 if tgt < g else 0.05) * (tgt - g)      # fast down, ~20 ms up
                gs[i] = g
            else:                                                 # separate gate gain: opens in
                tgt = 10 ** (gdb / 20)                            # ~2 ms, closes over close_ms
                g += (0.5 if tgt < g else 0.05) * (tgt - g)
                gt = 1.0 if held else 10 ** (gate_floor / 20)
                gg += (0.4 if gt > gg else 1 / close_ms) * (gt - gg)
                gs[i] = g * gg
        G = np.interp(np.arange(len(x)), np.arange(n) * B + B / 2, gs)
        return np.clip(x * G, -32767, 32767)
    return f

def denoise(floor_db=-12, over=1.5):
    """STFT noise suppression (256-pt, 50% overlap: ~1 MHz on the M4 with CMSIS FFTs).
    Noise PSD per bin = 20th percentile over time (a stand-in for a running minimum tracker);
    Wiener-style gain with over-subtraction, floored at floor_db"""
    from scipy.signal import stft, istft
    def f(x):
        _, _, Z = stft(x, FS, nperseg=256); P = np.abs(Z) ** 2
        N = np.percentile(P, 20, axis=1, keepdims=True)
        G = np.maximum(1 - over * N / (P + 1e-9), 10 ** (floor_db / 20))
        return istft(Z * G, FS, nperseg=256)[1][:len(x)]
    return f

def match_eq(ref, mic, cap_db=30, taps=255):
    """FIR (on the raw mic) giving the radio's long-term speech spectrum the shape of ref
    (a decent PC mic, same speaker), 0 dB at 800-1200 Hz, boost capped, cut above 3 kHz"""
    from scipy.signal import welch, firwin2
    rd = lambda p: (lambda w: np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float))(wave.open(p))
    f, Pr = welch(rd(ref), FS, nperseg=256); _, Pm = welch(rd(mic), FS, nperseg=256)
    g = 10 * np.log10(Pr / Pm); g = np.convolve(g, np.ones(5) / 5, "same")
    g -= g[(f >= 800) & (f < 1200)].mean(); g = np.minimum(g, cap_db); g[f > 3000] = -40; g[f < 60] = -20
    h = firwin2(taps, f, 10 ** (g / 20), fs=FS)
    return lambda x: np.clip(lfilter(h, 1, x), -32767, 32767)

VARIANTS = {
    "base":      [eq_fw()],
    "hc3000":    [eq_fw(), highcut(3000)],
    "hc2600":    [eq_fw(), highcut(2600)],
    "dyn":       [eq_fw(), dynamics()],
    "hc3000dyn": [eq_fw(), highcut(3000), dynamics()],
    "nr":        [eq_fw(), highcut(3000), denoise(), dynamics()],
}

if os.path.exists(f"{SCRATCH}/pcmic.wav") and os.path.exists(f"{SCRATCH}/ab_mic_raw.wav"):
    m = match_eq(f"{SCRATCH}/pcmic.wav", f"{SCRATCH}/ab_mic_raw.wav")
    VARIANTS["match"] = [m, dynamics()]
    VARIANTS["match_nogate"] = [m, dynamics(gate_floor=0)]
    VARIANTS["match_gate6"] = [m, dynamics(gate_floor=-6, hold_ms=200, close_ms=150)]
    VARIANTS["match_gate10"] = [m, dynamics(gate_floor=-10, hold_ms=200, close_ms=150)]

def codec(y, name):
    raw, bit, out = (f"{SCRATCH}/ab_{name}.{e}" for e in ("raw", "bit", "out"))
    y.astype(np.int16).tofile(raw)
    subprocess.run([f"{HERE}/sotto_enc_fx_lo", "-1200", raw, bit], check=True)
    subprocess.run([f"{HERE}/sotto_dec_fx_d1", "-1200", bit, out], check=True)
    return np.fromfile(out, np.int16)

def wav(path, y):
    w = wave.open(path, "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS)
    w.writeframes(np.asarray(y, np.int16).tobytes()); w.close()

if __name__ == "__main__":
    w = wave.open(sys.argv[1]); x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float)
    print(f"mic in: rms {20*np.log10(np.sqrt(np.mean(x**2))/32768):.1f} dBFS, peak {20*np.log10(np.abs(x).max()/32768):.1f}")
    for name, chain in VARIANTS.items():
        y = x
        for f in chain: y = f(y)
        rms = 20 * np.log10(np.sqrt(np.mean(y ** 2)) / 32768); clip = np.mean(np.abs(y) >= 32767)
        wav(f"{SCRATCH}/ab_{name}_pre.wav", y); wav(f"{SCRATCH}/ab_{name}.wav", codec(y, name))
        print(f"{name:10s} encoder in: rms {rms:6.1f} dBFS, clipped {clip:.4%}")
    print("wrote", f"{SCRATCH}/ab_*.wav")
