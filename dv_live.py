#!/usr/bin/env python3
"""Live dvtx receiver: IC-9700 USB audio in, decoded speech out on the default audio device.
usage: dv_live.py [--src PIPEWIRE_SOURCE] [--wav OUT.wav] [--hold N] [--replay AIR.wav]
Locks on 3 consecutive syncs a frame apart, decodes each 40 ms frame (tone-pair trellis
metrics as dv_rx.py, templates adapted as frames decode), plays it, and drops lock after N bad
frames in a row (--hold, default 12: a fade is bridged on the radio's steady symbol clock). Bad frames go to the decoder as lost (it conceals them)."""
import argparse, ctypes, os, subprocess, sys, time, wave
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dvframe as d
from dv_rx import FS, SPS, TONES, FRAME, isi_metrics

SRC = "alsa_input.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00.analog-stereo"
CHUNK = 960                            # 20 ms reads
KEEP = 3 * FS                          # samples of history kept
LOCK_SCORE, BAD_SER, BAD_RUN = 0.4, 0.15, 12   # BAD_SER: noise frames score 0.2-0.35, ~7% under 0.2; BAD_RUN: frames held through a fade (0.48 s)

class Decoder:
    def __init__(self):
        self.lib = ctypes.CDLL(os.path.join(HERE, "libsottodec.so"))
        self.lib.sotto_lr_dec_create.restype = ctypes.c_void_p
        for f in ("sotto_lr_dec_destroy", "sotto_lr_decode", "sotto_lr_dec_flush"):
            getattr(self.lib, f).argtypes = [ctypes.c_void_p] + ([ctypes.c_char_p, ctypes.c_int] if f == "sotto_lr_decode" else []) + \
                ([ctypes.c_void_p] if f != "sotto_lr_dec_destroy" else [])
        self.d = self.lib.sotto_lr_dec_create(); self.out = np.zeros(320, np.int16)
    def decode(self, sf, good):
        self.lib.sotto_lr_decode(self.d, sf, int(good), self.out.ctypes.data); return self.out.copy()
    def flush(self):
        res = []
        while self.lib.sotto_lr_dec_flush(self.d, self.out.ctypes.data): res.append(self.out.copy())
        self.lib.sotto_lr_dec_destroy(self.d); self.d = self.lib.sotto_lr_dec_create()
        return res

class Rx:
    def __init__(self, play, hold=BAD_RUN, clock=time.time):
        n = np.arange(SPS)
        self.kern = np.exp(2j * np.pi * np.outer(TONES, n) / FS)[:, ::-1]
        self.x = np.zeros(0); self.P = np.zeros((0, 4)); self.off = 0     # absolute index of x[0]
        self.mu = np.load(os.path.join(HERE, "mu_init.npy"))
        self.dec = Decoder(); self.play = play; self.hold = hold; self.clock = clock
        self.locked = False; self.t = 0; self.bad = 0; self.searched = 0
        self.n = self.nbad = 0; self.sers = []

    def feed(self, chunk):
        self.x = np.concatenate([self.x, chunk - 0.0])
        new = len(self.x) - SPS + 1 - len(self.P)          # tone powers for every complete window
        if new > 0:
            seg = self.x[len(self.P): len(self.P) + new + SPS - 1]
            p = np.stack([np.abs(np.convolve(seg, k, "valid")) for k in self.kern], 1)
            self.P = np.concatenate([self.P, p])
        if len(self.x) > KEEP + FS:                            # trim, keeping whole history needed
            cut = len(self.x) - KEEP
            if self.locked: cut = min(cut, self.t - self.off - 64)
            if cut > 0:
                self.x, self.P, self.off = self.x[cut:], self.P[cut:], self.off + cut
                self.searched = max(self.searched - cut, 0)
        self.locked and self.track() or self.search()

    def at(self, t):                                       # (48, 4) tone powers of the frame at t
        M = self.P[t - self.off + np.arange(d.FRAME_SYMS) * SPS]
        return M / (np.median(M.max(1)) + 1e-9)

    def sync(self, lo, hi):                                # sync score for starts lo..hi-1 (relative)
        P = self.P[lo: hi + (d.SYNC_LEN - 1) * SPS]; diff = (P[:, 3] - P[:, 0]) / (P.max(1) + 1e-9)
        s = np.zeros(hi - lo)
        for i, c in enumerate(d.SYNC * 2 - 1): s += c * diff[i * SPS: hi - lo + i * SPS]
        return s / d.SYNC_LEN

    def search(self):
        hi = len(self.P) - 3 * FRAME                       # need three frames after a candidate
        if hi <= self.searched: return False
        lo = self.searched; self.searched = hi
        s0 = self.sync(lo, hi)
        for r in np.where(s0 > LOCK_SCORE)[0]:
            t = lo + r
            ok = all(self.sync(t + k * FRAME - 2, t + k * FRAME + 3).max() > 0.2 for k in (1, 2))
            for k in (0, 1):                               # and the first two frames decode cleanly
                if not ok: break
                met = isi_metrics(self.at(self.off + t + k * FRAME), self.mu)
                bits = d.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
                ok = np.mean(met.argmax(1) != d.frame_symbols(bits)) < 0.1
            if ok:
                self.locked, self.t, self.bad = True, self.off + t, 0
                self.n = self.nbad = 0; self.sers = []; self.t0 = time.time()
                print(f"{self.stamp()} over: locked", flush=True)
                return True
        return False

    def frame_at(self, r):                                 # decode the frame at relative start r
        F = self.P[r + np.arange(d.FRAME_SYMS) * SPS]; F = F / (np.median(F.max(1)) + 1e-9)
        met = isi_metrics(F, self.mu)
        bits = d.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
        syms = d.frame_symbols(bits)
        return np.mean(met.argmax(1) != syms), F, bits, syms

    def track(self):
        while self.t - self.off + FRAME + 4 + SPS <= len(self.P):
            r0 = self.t - self.off
            def q(u):
                M = self.P[u + np.arange(d.FRAME_SYMS) * SPS]; srt = np.sort(M, 1)
                return np.mean(srt[:, -1] / (srt[:, -2] + 1e-9)) + 2 * self.sync(u, u + 1)[0]
            r = max(range(max(r0 - 2, 0), r0 + 3), key=q)
            ser, F, bits, syms = self.frame_at(r)
            if ser >= BAD_SER and r != r0:                     # flywheel: in a fade the retiming
                alt = self.frame_at(r0)                         # chases noise; the radio's symbol
                if alt[0] < ser: r, (ser, F, bits, syms) = r0, alt   # clock doesn't drift
            good = ser < BAD_SER
            if good:                                       # adapt the tone-pair templates
                prev = np.r_[syms[0], syms[:-1]]
                for k in range(1, len(syms)): self.mu[prev[k], syms[k]] += 0.02 * (F[k] - self.mu[prev[k], syms[k]])
            self.play(self.dec.decode(np.packbits(bits).tobytes(), good))
            self.n += 1; self.nbad += not good; self.sers.append(ser)
            self.bad = 0 if good else self.bad + 1
            self.t = self.off + r + FRAME
            if self.bad >= self.hold:
                for y in self.dec.flush(): self.play(y)
                n = self.n - self.hold
                print(f"{self.stamp()} over: end, {n} frames ({n * 0.04:.1f} s), "
                      f"{self.nbad - self.hold} bad, raw SER {np.mean(self.sers[:-self.hold] or [0]):.4f}", flush=True)
                self.locked = False; self.searched = self.t - self.off
                return True
        return True

    def stamp(self): return time.strftime("%H:%M:%S", time.localtime(self.clock()))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--src", default=SRC); ap.add_argument("--wav")
    ap.add_argument("--hold", type=int, default=BAD_RUN, help="bad frames in a row before dropping lock")
    ap.add_argument("--replay", metavar="AIR.wav", help="decode a recording (48 kHz) instead, no playback; "
                    "AIR_start.txt beside it (unix time of sample 0) gives the timestamps")
    a = ap.parse_args()
    if a.replay:
        w = wave.open(a.replay); st = os.path.splitext(a.replay)[0] + "_start.txt"
        t0 = float(open(st).read()) if os.path.exists(st) else 0.0
        wf = None
        if a.wav:
            wf = wave.open(a.wav, "wb"); wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(8000)
        rx = Rx(lambda y: wf and wf.writeframes(y.tobytes()), a.hold, lambda: t0 + (rx.off + len(rx.x)) / FS)
        while b := w.readframes(CHUNK): rx.feed(np.frombuffer(b, np.int16).astype(float))
        if wf: wf.close()
        return
    rec = subprocess.Popen(["pw-record", f"--target={a.src}", "--rate=48000", "--channels=1", "--format=s16",
                            "--raw", "-"], stdout=subprocess.PIPE)
    out = subprocess.Popen(["pw-play", "--rate=8000", "--channels=1", "--format=s16", "--raw",
                            "--latency=40ms", "-"], stdin=subprocess.PIPE)
    wf = None
    if a.wav:
        wf = wave.open(a.wav, "wb"); wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(8000)
    def play(y):
        out.stdin.write(y.tobytes()); out.stdin.flush()
        if wf: wf.writeframes(y.tobytes())
    rx = Rx(play)
    print("listening", flush=True)
    try:
        while True:
            b = rec.stdout.read(CHUNK * 2)
            if not b: break
            rx.feed(np.frombuffer(b, np.int16).astype(float))
    except KeyboardInterrupt:
        pass
    finally:
        rec.terminate(); out.stdin.close(); out.wait()
        if wf: wf.close()

if __name__ == "__main__":
    main()
