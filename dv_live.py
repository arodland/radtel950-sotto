#!/usr/bin/env python3
"""Live dvtx receiver: IC-9700 USB audio in, decoded speech out on the default audio device.
usage: [DVFMT=v2] dv_live.py [--src PIPEWIRE_SOURCE] [--wav OUT.wav] [--hold N] [--noncoh] [--replay AIR.wav]
Locks on 3 consecutive syncs a frame apart, decodes each frame coherently (coh.py: the air signal
is continuous-phase 4-FSK; carrier phase carried from frame to frame, templates adapted as frames
decode; --noncoh: Goertzel + tone-pair trellis as dv_rx.py), plays it, and drops lock after N bad
frames in a row (--hold, default 12: a fade is bridged on the radio's steady symbol clock). Bad frames go to the decoder as lost (it conceals them)."""
import argparse, ctypes, os, subprocess, sys, time, wave
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dvframe as d
from dv_rx import FS, SPS, TONES, FMT, FRAME, isi_metrics
import coh

SRC = "alsa_input.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00.analog-stereo"
CHUNK = 960                            # 20 ms reads
KEEP = 3 * FS                          # samples of history kept
LOCK_SCORE, BAD_SER = 0.4, 0.15                  # BAD_SER: noise frames score 0.2-0.35, ~7% under 0.2
BAD_RUN = max(3, round(0.5 * 1200 / FMT.FRAME_SYMS))   # frames held through a fade (~0.5 s)

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
    def __init__(self, play, hold=BAD_RUN, clock=time.time, coherent=False):
        n = np.arange(SPS)
        self.kern = np.exp(2j * np.pi * np.outer(TONES, n) / FS)[:, ::-1]
        self.x = np.zeros(0); self.P = np.zeros((0, 4)); self.off = 0     # absolute index of x[0]
        self.mu = np.load(os.path.join(HERE, "mu_init.npy"))
        self.coh = coherent; self.an = coh.Analytic(); self.z = np.zeros(0, complex); self.T = None; self.ngood = 0
        self.last_sym = None; self.psi = None; self.psi_amb = True; self.tf = 0.0; self.drift = 0.0
        self.dec = Decoder(); self.play = play; self.hold = hold; self.clock = clock
        self.locked = False; self.t = 0; self.bad = 0; self.searched = 0
        self.n = self.nbad = 0; self.sers = []; self.gmasks = []

    def feed(self, chunk):
        self.x = np.concatenate([self.x, chunk - 0.0])
        if self.coh: self.z = np.concatenate([self.z, self.an(chunk)])
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
                if self.coh: self.z = self.z[cut:]
                self.searched = max(self.searched - cut, 0)
        self.locked and self.track() or self.search()

    def at(self, t):                                       # (48, 4) tone powers of the frame at t
        M = self.P[t - self.off + np.arange(FMT.FRAME_SYMS) * SPS]
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
                bits = FMT.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
                ok = np.mean(met.argmax(1) != FMT.frame_symbols(bits)) < 0.1
            if ok:
                self.locked, self.t, self.bad = True, self.off + t, 0
                self.n = self.nbad = 0; self.sers = []; self.gmasks = []; self.t0 = time.time()
                self.tf = float(self.t); self.drift = 0.0; self.last_sym = None; self.psi = None; self.psi_amb = True
                print(f"{self.stamp()} over: locked", flush=True)
                return True
        return False

    def frame_at(self, r):                                 # decode the frame at relative start r
        F = self.P[r + np.arange(FMT.FRAME_SYMS) * SPS]; F = F / (np.median(F.max(1)) + 1e-9)
        met = isi_metrics(F, self.mu)
        bits = FMT.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
        syms = FMT.frame_symbols(bits)
        ser = np.mean(met.argmax(1) != syms); self.met_data = met[d.SYNC_LEN:]
        if not self.coh: return ser, F, bits, syms
        # coherent: carrier phase = the phase carried from the last good frame (held through a
        # fade; 0/pi ambiguous after bad frames) combined with the sync symbols' estimate, then
        # re-estimated from the whole re-encoded frame (two passes, as the noncoherent receiver)
        S = coh.segs(self.z, r + self.an.d, FMT.FRAME_SYMS)
        if self.T is None:                                 # first frame: ideal templates at the signal level
            C = coh.corr(S, coh.ideal_templates()); self.T = coh.ideal_templates(np.median(np.abs(C.max((1, 2)))) / SPS)
        C = coh.corr(S, self.T)
        sync = d.SYNC * 3
        th = (coh.ADV[sync[0]] + np.concatenate([[0], np.cumsum(coh.ADV[sync[1:-1]])])) % 2
        vs = (C[1:d.SYNC_LEN, sync[:-1], sync[1:]] * np.exp(-1j * np.pi * th)).sum()
        # phase hypotheses: the carried phase (and its pi twin when ambiguous) with +-0.5 rad,
        # the sync symbols' estimate, and the noncoherent decode's when that decode looks good;
        # the trellis picks the best path. Then re-estimate from the re-encoded frame, re-detect.
        cands = [np.angle(vs)]
        if self.psi is not None:
            for p in ([self.psi, self.psi + np.pi] if self.psi_amb else [self.psi]): cands += [p, p - 0.5, p + 0.5]
        if ser < BAD_SER:
            cands.append(coh.phase_est(C, np.r_[self.last_sym if self.last_sym is not None else syms[0], syms[:-1]], syms))
        psi = np.array(cands)
        for _ in range(2):
            met, h = coh.metrics(C, self.T, psi, theta0=0, prev0=self.last_sym)
            bits = FMT.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
            syms = FMT.frame_symbols(bits); prev = np.r_[self.last_sym if self.last_sym is not None else syms[0], syms[:-1]]
            psi = coh.phase_est(C, prev, syms)
        ser = np.mean(met.argmax(1) != syms); self.met_data = met[d.SYNC_LEN:]
        self._coh = (S, prev, syms, psi)
        return ser, F, bits, syms

    def timing_err(self, r0, syms):
        """symbol timing error (samples, -2..2) of the frame decoded at r0, from the decoded
        symbols: coherent sum of the matched filters (sharp in timing), or the noncoherent
        tone concentration"""
        if self.coh:
            S, prev, syms, psi = self._coh
            th = np.concatenate([[0], np.cumsum(coh.ADV[syms[:-1]])]) % 2
            rot = np.exp(-1j * np.pi * th)
            J = [abs((np.einsum("is,is->i", coh.segs(self.z, r0 + dd + self.an.d, FMT.FRAME_SYMS), self.T[prev, syms].conj()) * rot).sum())
                 for dd in range(-2, 3)]
        else:
            idx = np.arange(FMT.FRAME_SYMS) * SPS
            J = [self.P[r0 + dd + idx, syms].sum() for dd in range(-2, 3)]
        k = int(np.argmax(J))
        if 0 < k < 4:                                       # parabolic peak
            y0, y1, y2 = J[k - 1], J[k], J[k + 1]; den = y0 - 2 * y1 + y2
            return k - 2 + (0.5 * (y0 - y2) / den if den else 0)
        return k - 2

    def track(self):
        while int(round(self.tf)) - self.off + FRAME + 4 + SPS <= len(self.P):
            r0 = int(round(self.tf)) - self.off
            ser, F, bits, syms = self.frame_at(r0)
            good = ser < BAD_SER
            e = 0.0
            if good:
                e = self.timing_err(r0, syms)
                prev = np.r_[syms[0], syms[:-1]]           # adapt the tone-pair templates
                for k in range(1, len(syms)): self.mu[prev[k], syms[k]] += 0.02 * (F[k] - self.mu[prev[k], syms[k]])
                if self.coh:
                    S, prev, syms, psi = self._coh; self.ngood += 1
                    coh.update_templates(self.T, S, prev, syms, psi, alpha=max(0.02, 1 / (self.ngood + 1)))
                    self.psi = psi + np.pi * (coh.ADV[syms].sum() % 2); self.psi_amb = False
                    self.last_sym = int(syms[-1])
            else:
                self.last_sym = None; self.psi_amb = True
            # timing loop: the radio's symbol clock is steady, so follow the error slowly
            self.tf += FRAME + self.drift + 0.3 * e; self.drift = float(np.clip(self.drift + 0.02 * e, -0.3, 0.3))
            self.t = int(round(self.tf))
            if good and FMT.classes is not None:            # unequal protection: shape may be lost alone
                sa, sb = FMT.class_ser(self.met_data, bits); gmask = 1 if sb < BAD_SER else 2
            else: gmask = int(good)
            self.gmasks.append(gmask)
            pb = np.packbits(bits).tobytes()
            for k in range(FMT.SF_PER_FRAME): self.play(self.dec.decode(pb[6 * k: 6 * k + 6], gmask))
            self.n += 1; self.nbad += not good; self.sers.append(ser)
            self.bad = 0 if good else self.bad + 1
            if self.bad >= self.hold:
                for y in self.dec.flush(): self.play(y)
                n = self.n - self.hold
                part = sum(g == 2 for g in self.gmasks[:-self.hold])
                print(f"{self.stamp()} over: end, {n} frames ({n * FMT.FRAME_SYMS / 1200:.1f} s), "
                      f"{self.nbad - self.hold} bad" + (f", {part} shape-only" if part else "") +
                      f", raw SER {np.mean(self.sers[:-self.hold] or [0]):.4f}", flush=True)
                self.locked = False; self.searched = self.t - self.off
                return True
        return True

    def stamp(self): return time.strftime("%H:%M:%S", time.localtime(self.clock()))

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--src", default=SRC); ap.add_argument("--wav")
    ap.add_argument("--hold", type=int, default=BAD_RUN, help="bad frames in a row before dropping lock")
    ap.add_argument("--noncoh", action="store_true", help="noncoherent (Goertzel + tone-pair trellis) detection instead of coh.py")
    ap.add_argument("--replay", metavar="AIR.wav", help="decode a recording (48 kHz) instead, no playback; "
                    "AIR_start.txt beside it (unix time of sample 0) gives the timestamps")
    a = ap.parse_args()
    if a.replay:
        w = wave.open(a.replay); st = os.path.splitext(a.replay)[0] + "_start.txt"
        t0 = float(open(st).read()) if os.path.exists(st) else 0.0
        wf = None
        if a.wav:
            wf = wave.open(a.wav, "wb"); wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(8000)
        rx = Rx(lambda y: wf and wf.writeframes(y.tobytes()), a.hold, lambda: t0 + (rx.off + len(rx.x)) / FS, not a.noncoh)
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
    rx = Rx(play, a.hold, coherent=not a.noncoh)
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
