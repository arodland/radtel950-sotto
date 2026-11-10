#!/usr/bin/env python3
"""Live dvtx receiver: IC-9700 USB audio in, decoded speech out on the default audio device.
usage: dv_live.py [--if] [--raw IN.wav] [--disc] [--src PIPEWIRE_SOURCE] [--wav OUT.wav] [--hold N] [--noncoh] [--parrot] [--smeter OUT.tsv] [--replay AIR.wav]
Locks on 3 consecutive syncs a frame apart, decodes each frame coherently (coh.py: the air signal
is continuous-phase 4-FSK; carrier phase carried from frame to frame, templates adapted as frames
decode; --noncoh: Goertzel + tone-pair trellis as dv_rx.py), plays it, and drops lock after N bad
frames in a row (--hold, default 12: a fade is bridged on the radio's steady symbol clock). Bad frames go to the decoder as lost (it conceals them)."""
import argparse, ctypes, os, subprocess, sys, threading, time, wave
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dvframe as d
from dv_rx import FS, SPS, TONES, FMT, FRAME, isi_metrics
import coh
from scipy.signal import firwin, lfilter, fftconvolve

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
    LOCK_SCORE, BAD_SER, LOCK_SER = LOCK_SCORE, BAD_SER, 0.1
    TGAIN, DGAIN = 0.3, 0.02                               # timing loop: phase and drift gains per frame
    QNAME = "raw SER"
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
        self.front(chunk)
        if len(self.x) > KEEP + FS:                            # trim, keeping whole history needed
            cut = len(self.x) - KEEP
            if self.locked: cut = min(cut, self.t - self.off - 64)
            if cut > 0:
                self.x, self.P, self.off = self.x[cut:], self.P[cut:], self.off + cut
                if self.coh: self.z = self.z[cut:]
                self.searched = max(self.searched - cut, 0)
        self.locked and self.track() or self.search()

    def front(self, chunk):                                # append input; tone powers for every complete window
        self.x = np.concatenate([self.x, chunk - 0.0])
        if self.coh: self.z = np.concatenate([self.z, self.an(chunk)])
        new = len(self.x) - SPS + 1 - len(self.P)
        if new > 0:
            seg = self.x[len(self.P): len(self.P) + new + SPS - 1]
            p = np.stack([np.abs(np.convolve(seg, k, "valid")) for k in self.kern], 1)
            self.P = np.concatenate([self.P, p])

    def met(self, F): return isi_metrics(F, self.mu)       # (48, 4) normalized tone powers -> tone metrics
    def quality(self, met, syms): return np.mean(met.argmax(1) != syms)   # frame score, low = good (raw SER)

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
        for r in np.where(s0 > self.LOCK_SCORE)[0]:
            t = lo + r
            ok = all(self.sync(t + k * FRAME - 2, t + k * FRAME + 3).max() > 0.2 for k in (1, 2))
            for k in (0, 1):                               # and the first two frames decode cleanly
                if not ok: break
                met = self.met(self.at(self.off + t + k * FRAME))
                bits = FMT.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
                ok = self.quality(met, FMT.frame_symbols(bits)) < self.LOCK_SER
            if ok:
                self.locked, self.t, self.bad = True, self.off + t, 0
                self.n = self.nbad = 0; self.sers = []; self.gmasks = []; self.t0 = time.time()
                self.tf = float(self.t); self.drift = 0.0; self.last_sym = None; self.psi = None; self.psi_amb = True
                print(f"{self.stamp()} over: locked", flush=True)
                return True
        return False

    def frame_at(self, r):                                 # decode the frame at relative start r
        F = self.P[r + np.arange(FMT.FRAME_SYMS) * SPS]; F = F / (np.median(F.max(1)) + 1e-9)
        met = self.met(F)
        bits = FMT.frame_decode(met[d.SYNC_LEN:] / (np.std(met) + 1e-9))
        syms = FMT.frame_symbols(bits)
        ser = self.quality(met, syms); self.met_data = met[d.SYNC_LEN:]
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
            good = ser < self.BAD_SER
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
            self.tf += FRAME + self.drift + self.TGAIN * e; self.drift = float(np.clip(self.drift + self.DGAIN * e, -0.3, 0.3))
            self.t = int(round(self.tf))
            if good and FMT.classes is not None:            # unequal protection: shape may be lost alone
                sa, sb = FMT.class_ser(self.met_data, bits); gmask = 1 if sb < BAD_SER else 2
            else: gmask = int(good)
            self.gmasks.append(gmask)
            pb = np.packbits(bits).tobytes()
            if getattr(self, 'on_frame', None): self.on_frame(bits, gmask)
            for k in range(FMT.SF_PER_FRAME): self.play(self.dec.decode(pb[6 * k: 6 * k + 6], gmask))
            self.n += 1; self.nbad += not good; self.sers.append(ser)
            self.bad = 0 if good else self.bad + 1
            if self.bad >= self.hold:
                for y in self.dec.flush(): self.play(y)
                n = self.n - self.hold
                part = sum(g == 2 for g in self.gmasks[:-self.hold])
                print(f"{self.stamp()} over: end, {n} frames ({n * FMT.FRAME_SYMS / 1200:.1f} s), "
                      f"{self.nbad - self.hold} bad" + (f", {part} shape-only" if part else "") +
                      f", {self.QNAME} {np.mean(self.sers[:-self.hold] or [0]):.4f}", flush=True)
                self.locked = False; self.searched = self.t - self.off
                if getattr(self, 'on_over_end', None): self.on_over_end()
                return True
        return True

    def stamp(self): return time.strftime("%H:%M:%S", time.localtime(self.clock()))

class IfRx(Rx):
    """V2 from the IC-9700's IF output (USB AF/IF = IF, centre 12 kHz, 48 kHz) with no FM demodulator: below the
    discriminator's threshold (ifbench.py: ~5 dB). The IF is mixed to baseband (+-8 kHz channel filter) and every
    sample position gets per-tone metrics |<s, T>|, T the 40-sample IF waveforms of (previous tone, tone, audio phase
    in 8 bins) (ifT_init.npy, learned by ifbench.py from a clean capture) maxed over previous tone and phase. They
    stand in for the noncoherent receiver's tone powers (sync, lock, timing loop, hold) and go to the Viterbi as
    they are. The mixer follows the signal's mean frequency on good frames (captures: within 30 Hz of 12 kHz)."""
    # frame score: the re-encoded path's mean soft distance from the best tone (in metric sd), not raw SER:
    # at 40-43 dB-Hz 0.11 keeps 95% of correct frames and passes 5% of wrong ones (SER: 23%)
    LOCK_SCORE, BAD_SER, LOCK_SER, QNAME = LOCK_SCORE, 0.11, 0.1, "soft loss"
    TGAIN, DGAIN = 0.1, 0.005                              # slower than the AF receiver: steadier at low C/N0
    def __init__(self, play, hold=BAD_RUN, clock=time.time, fc=12000.0):
        super().__init__(play, hold, clock, coherent=False)
        self.K = np.load(os.path.join(HERE, "ifT_init.npy")).reshape(-1, SPS)[:, ::-1].conj()
        self.ch = firwin(255, 8000, fs=FS); self.zi = np.zeros(254, complex); self.fc = fc; self.ph = 0.0
        self.x = np.zeros(0, complex)
    def front(self, chunk):
        w = 2 * np.pi * self.fc / FS
        mixed = chunk * np.exp(-1j * (self.ph + w * np.arange(len(chunk)))); self.ph = (self.ph + w * len(chunk)) % (2 * np.pi)
        zb, self.zi = lfilter(self.ch, 1, mixed, zi=self.zi); self.x = np.concatenate([self.x, zb])
        new = len(self.x) - SPS + 1 - len(self.P)
        if new > 0:
            seg = self.x[len(self.P): len(self.P) + new + SPS - 1]
            C = np.abs(fftconvolve(seg[None], self.K, mode="valid", axes=1))
            self.P = np.concatenate([self.P, C.reshape(4, 4, 8, -1).max((0, 2)).T])
    def met(self, F): return F - F.mean(1, keepdims=True)
    def quality(self, met, syms):
        z = met / (met.std() + 1e-9); return (z.max(1) - z[np.arange(len(syms)), syms]).mean()
    def timing_err(self, r0, syms):                        # called on good frames: follow the IF centre too
        z = self.x[r0: r0 + FRAME]
        self.fc = float(np.clip(self.fc + 0.1 * np.angle((z[1:] * z[:-1].conj()).sum()) * FS / (2 * np.pi), 11500, 12500))
        return super().timing_err(r0, syms)

def if_disc(x, fc=12000.0):
    """IF recording (12 kHz centre) -> what an FM receiver's AF would be: +-8 kHz channel, discriminator, 3.4 kHz"""
    z = lfilter(firwin(255, 8000, fs=FS), 1, x * np.exp(-2j * np.pi * fc * np.arange(len(x)) / FS))
    f = np.r_[0, np.angle(z[1:] * z[:-1].conj())] * FS / (2 * np.pi)
    f = lfilter(firwin(127, 3400, fs=FS), 1, f); f -= np.convolve(f, np.ones(1920) / 1920, "same")   # AC-coupled, as the AF is
    k = 1 - np.exp(-1 / (60e-6 * FS)); f = lfilter([k], [1, k - 1], f)    # the AF's gentle roll-off (~60 us)
    return f * 2                                                            # ~ the 9700's AF level (4 kHz dev -> 8000)

class Civ:
    """IC-9700 over CI-V (address 0xA2): S-meter reads and PTT. One owner per port."""
    def __init__(self, port="/dev/ttyUSB0"):
        import serial
        self.s = serial.Serial(port, 19200, timeout=0.3); self.lock = threading.Lock()
    def cmd(self, data):
        pkt = bytes([0xFE, 0xFE, 0xA2, 0xE0]) + data + b"\xfd"
        with self.lock:
            self.s.reset_input_buffer(); self.s.write(pkt); b = self.s.read_until(b"\xfd", 64)
            if b.endswith(pkt): b += self.s.read_until(b"\xfd", 64)
        return b
    def smeter(self):
        b = self.cmd(bytes([0x15, 0x02])); i = b.find(bytes([0xFE, 0xFE, 0xE0, 0xA2, 0x15, 0x02]))
        return int(f"{b[i + 6]:02x}{b[i + 7]:02x}") if i >= 0 and len(b) >= i + 9 else -1
    def ptt(self, on): self.cmd(bytes([0x1C, 0x00, 1 if on else 0]))

class Parrot:
    """after each over, key the IC-9700 (CI-V PTT) and play the decoded speech into its USB audio
    input (set the 9700's MOD input to USB). Overs shorter than MIN_S are ignored."""
    SINK = "alsa_output.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00.analog-stereo"
    MIN_S, MAX_S = 1.0, 30.0
    def __init__(self, civ, level=0.4, dv=False):
        self.civ = civ; self.level = level; self.dv = dv; self.buf = []; self.frames = []; self.last = None; self.busy = False
    def audio(self, y): self.buf.append(y)
    def frame(self, bits, good):                       # DV parrot: relay the received superframes
        if good: self.last = bits.copy()
        if self.last is not None: self.frames.append(self.last if not good else bits.copy())   # bad: hold the last good one
    def over_end(self):
        y = np.concatenate(self.buf) if self.buf else np.zeros(0, np.int16); self.buf = []
        frames, self.frames, self.last = self.frames[:-BAD_RUN], [], None   # the hold frames are the over's tail
        if len(y) < self.MIN_S * 8000 or self.busy: return
        if self.dv:
            if len(frames) < self.MIN_S * 25: return
            from dv_send import frames_audio
            y48 = frames_audio(frames[:int(self.MAX_S * 25)], 0.3); what = f"DV {len(frames) * 0.04:.1f} s"
        else:
            from scipy.signal import resample_poly                    # PipeWire's own 8 -> 48 kHz path stuttered
            y = y[:int(self.MAX_S * 8000)]
            y48 = np.clip(resample_poly(y.astype(float), 6, 1) * self.level, -32767, 32767).astype(np.int16)
            what = f"FM {len(y) / 8000:.1f} s"
        threading.Thread(target=self.send, args=(y48, what), daemon=True).start()
    def send(self, y48, what):
        self.busy = True
        try:
            self.civ.ptt(True); time.sleep(0.4)
            subprocess.run(["pw-play", f"--target={self.SINK}", "--rate=48000", "--channels=1", "--format=s16",
                            "--raw", "-"], input=y48.tobytes(), check=False)
            time.sleep(0.3); self.civ.ptt(False)
            print(f"{time.strftime('%H:%M:%S')} parrot: sent {what}", flush=True)
        finally:
            self.civ.ptt(False); self.busy = False

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--src", default=SRC); ap.add_argument("--wav")
    ap.add_argument("--hold", type=int, default=BAD_RUN, help="bad frames in a row before dropping lock")
    ap.add_argument("--if", dest="ifmode", action="store_true", help="decode from the 9700's IF output (USB AF/IF output = IF) without FM demodulation: IfRx")
    ap.add_argument("--raw", metavar="IN.wav", help="also record the raw 48 kHz input (and IN_start.txt) for --replay")
    ap.add_argument("--disc", action="store_true", help="with --replay of an IF recording: FM-demodulate it in software and run the AF receiver (A/B against --if)")
    ap.add_argument("--noncoh", action="store_true", help="noncoherent (Goertzel + tone-pair trellis) detection instead of coh.py")
    ap.add_argument("--parrot", nargs="?", const="dv", choices=["dv", "fm"], help="retransmit each over on the IC-9700 (CI-V PTT, USB audio): dv relays the received superframes as a DV over (default), fm plays the decoded speech as FM")
    ap.add_argument("--smeter", metavar="OUT.tsv", help="log the IC-9700 S-meter every 0.5 s (unix time, raw)")
    ap.add_argument("--parrot-level", type=float, default=0.3, help="parrot playback level into the 9700's USB input (0-1; 0.3 is clean)")
    ap.add_argument("--replay", metavar="AIR.wav", help="decode a recording (48 kHz) instead, no playback; "
                    "AIR_start.txt beside it (unix time of sample 0) gives the timestamps")
    a = ap.parse_args()
    if a.replay:
        w = wave.open(a.replay); st = os.path.splitext(a.replay)[0] + "_start.txt"
        t0 = float(open(st).read()) if os.path.exists(st) else 0.0
        wf = None
        if a.wav:
            wf = wave.open(a.wav, "wb"); wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(8000)
        pl, ck = (lambda y: wf and wf.writeframes(y.tobytes())), (lambda: t0 + (rx.off + len(rx.x)) / FS)
        rx = IfRx(pl, a.hold, ck) if a.ifmode else Rx(pl, a.hold, ck, not a.noncoh)
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float)
        if a.disc: x = if_disc(x); rx.mu = np.broadcast_to(np.eye(4), (4, 4, 4)).copy()   # mu_init.npy is the 9700's AF
        for i in range(0, len(x), CHUNK): rx.feed(x[i:i + CHUNK])
        if wf: wf.close()
        return
    rec = subprocess.Popen(["pw-record", f"--target={a.src}", "--rate=48000", "--channels=1", "--format=s16",
                            "--raw", "-"], stdout=subprocess.PIPE)
    out = subprocess.Popen(["pw-play", "--rate=8000", "--channels=1", "--format=s16", "--raw",
                            "--latency=40ms", "-"], stdin=subprocess.PIPE)
    wf = None
    if a.wav:
        wf = wave.open(a.wav, "wb"); wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(8000)
    raw = None
    if a.raw:
        raw = wave.open(a.raw, "wb"); raw.setnchannels(1); raw.setsampwidth(2); raw.setframerate(FS)
        open(os.path.splitext(a.raw)[0] + "_start.txt", "w").write(f"{time.time():.3f}\n")
    civ = Civ() if (a.parrot or a.smeter) else None
    parrot = Parrot(civ, a.parrot_level, a.parrot == 'dv') if a.parrot else None
    def play(y):
        out.stdin.write(y.tobytes()); out.stdin.flush()
        if wf: wf.writeframes(y.tobytes())
        if parrot: parrot.audio(y)
    rx = IfRx(play, a.hold) if a.ifmode else Rx(play, a.hold, coherent=not a.noncoh)
    if parrot: rx.on_over_end = parrot.over_end; rx.on_frame = parrot.frame
    if a.smeter:
        def smeter_log():
            with open(a.smeter, "a", buffering=1) as f:
                while True:
                    t = time.time(); v = civ.smeter()
                    if v >= 0: f.write(f"{t:.2f}\t{v}\n")
                    time.sleep(max(0, 0.5 - (time.time() - t)))
        threading.Thread(target=smeter_log, daemon=True).start()
    print("listening", flush=True)
    try:
        while True:
            b = rec.stdout.read(CHUNK * 2)
            if not b: break
            if raw: raw.writeframes(b)
            rx.feed(np.frombuffer(b, np.int16).astype(float))
    except KeyboardInterrupt:
        pass
    finally:
        rec.terminate(); out.stdin.close(); out.wait()
        if wf: wf.close()
        if raw: raw.close()

if __name__ == "__main__":
    main()
