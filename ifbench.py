"""Bench: does an IF-domain detector beat the FM discriminator on our 4-FSK?
Takes a clean (S9) IC-9700 IF capture of a DV over plus the radio's TX log (ifrun-style dir:
if.wav, log.txt), adds complex white noise at a swept C/N0, and runs two detectors with the same
genie help (frame timing and payload-aligned templates learned from the clean capture):
  disc  the 9700-style path: FM discriminator -> 3.4 kHz audio -> coh.py coherent trellis
  if    no demodulator: per-symbol noncoherent correlation of the complex IF against learned
        (previous tone, tone, audio phase in NB bins) templates, |<s, T>| maxed over previous tone and phase
Both feed dvframe's Viterbi; reports raw SER and frame error rate vs C/N0.
Writes ifT_init.npy (the IF templates, for dv_live --if).
Usage: python3 ifbench.py [capture dir [C/N0 from to]] ; SPDX-License-Identifier: MIT"""
import os, sys, wave, numpy as np
from scipy.signal import hilbert, firwin, lfilter
import dvframe as d, coh
from dv_rx import parse_log, decode

FS, SPS = 48000, 40
NB = 8                                            # audio phase bins for the IF templates

def load(W):
    w = wave.open(f"{W}/if.wav"); x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float); x -= x.mean()
    z = hilbert(x)
    B = 4800; n = len(x) // B
    X = np.abs(np.fft.rfft(x[:n * B].reshape(n, B) * np.hanning(B), axis=1)) ** 2; f = np.fft.rfftfreq(B, 1 / FS)
    r = 10 * np.log10(X[:, (f > 4000) & (f < 20000)].sum(1) / X[:, (f > 22000) | (f < 2000)].sum(1))
    idx = np.flatnonzero(r > 45); a, b = idx[0] * B, (idx[-1] + 1) * B
    fc = np.median(np.diff(np.unwrap(np.angle(z[a + 4800:b - 4800])))) * FS / (2 * np.pi)
    zb = (z * np.exp(-2j * np.pi * fc * np.arange(len(z)) / FS))[a - 4800:b + 4800]
    return zb / np.sqrt(np.mean(np.abs(zb[4800:-4800]) ** 2))      # carrier power 1

CH = firwin(255, 8000, fs=FS)                     # channel filter, +-8 kHz (16 kHz IF, like FM-D)
AF = firwin(127, 3400, fs=FS)                     # discriminator audio low-pass

def chan(z): return lfilter(CH, 1, np.r_[z, np.zeros(127)])[127:]
def disc(z):
    f = np.r_[0, np.angle(z[1:] * z[:-1].conj())] * FS / (2 * np.pi)
    return lfilter(AF, 1, np.r_[f, np.zeros(63)])[63:]

def align(zb, ref):
    """frame starts in zb (samples) and their TX log index, from the clean discriminator decode"""
    got, starts, _, _ = decode(disc(chan(zb)))
    best = max(range(-30, 31), key=lambda o: sum(g == ref[i + o] for i, g in enumerate(got) if 0 <= i + o < len(ref)))
    return [(starts[i], i + best) for i in range(len(got)) if 0 <= i + best < len(ref) and got[i] == ref[i + best]]

def frame_info(pairs, ref):
    """the HT's symbol clock is steady and an over's frames are back to back: positions on a fitted line
    (the decoder's own frame starts jitter by ~2 samples, which smears learned templates)"""
    out = []; tj = np.array(pairs, float); fit = np.polyfit(tj[:, 1], tj[:, 0], 1)
    for _, j in pairs:
        t = int(round(np.polyval(fit, j)))
        bits = np.unpackbits(np.frombuffer(ref[j], np.uint8)); sy = d.frame_symbols(bits).astype(int)
        out.append((t, bits, sy, np.r_[sy[0], sy[:-1]]))
    return out

def theta(sy): return np.concatenate([[0], np.cumsum(coh.ADV[sy[:-1]])]) % 2

def learn(zc, ac, frames):
    """disc templates (4,4,SPS) + per-frame audio phase psi; IF templates (4,4,NB,SPS)"""
    T = coh.ideal_templates(np.std(ac) * np.sqrt(2)); psis = []
    for t, _, sy, pv in frames:
        psi = coh.phase_est(coh.corr(coh.segs(ac, t), T), pv, sy); psis.append(psi)
    psis = np.array(psis)
    Td = np.zeros((4, 4, SPS), complex); Ti = np.zeros((4, 4, NB, SPS), complex)
    nd = np.zeros((4, 4)); ni = np.zeros((4, 4, NB))
    for (t, _, sy, pv), psi in zip(frames, psis):
        th = theta(sy); Sa = coh.segs(ac, t); Sz = coh.segs(zc, t)
        rot = Sa * np.exp(-1j * (psi + np.pi * th))[:, None]
        tb = np.round((psi + np.pi * th) / (2 * np.pi / NB)).astype(int) % NB  # audio phase at symbol start
        for i in range(len(sy)):
            Td[pv[i], sy[i]] += rot[i]; nd[pv[i], sy[i]] += 1
            s = Sz[i] * np.exp(-1j * np.angle(Sz[i][SPS // 2]))
            Ti[pv[i], sy[i], tb[i]] += s; ni[pv[i], sy[i], tb[i]] += 1
    return Td / np.maximum(nd, 1)[..., None], Ti / np.maximum(ni, 1)[..., None], psis, (ni > 0).mean()

def run(zc, frames, Td, Ti, psis):
    ac = coh.analytic(disc(zc)); res = {"disc": [0, 0, 0], "if": [0, 0, 0]}     # sym errs, frame errs, n
    TiN = Ti / np.maximum(np.linalg.norm(Ti, axis=-1, keepdims=True), 1e-9)
    for (t, bits, sy, pv), psi in zip(frames, psis):
        md, _ = coh.metrics(coh.corr(coh.segs(ac, t), Td), Td, psi)
        mi = np.abs(np.einsum("is,pcts->ipct", coh.segs(zc, t), TiN.conj())).max((1, 3))
        for k, m in (("disc", md), ("if", mi)):
            m = m[d.SYNC_LEN:]; m = m - m.mean(1, keepdims=True)
            res[k][0] += int((m.argmax(1) != sy[d.SYNC_LEN:]).sum())
            res[k][1] += int((d.frame_decode(m / (m.std() + 1e-9)) != bits).any()); res[k][2] += 1
    return res

if __name__ == "__main__":
    W = sys.argv[1] if len(sys.argv) > 1 else "/home/andrew/code/radtel950/captures/if_eq-2026-11-09"
    zb = load(W); ref = parse_log(open(f"{W}/log.txt").read().splitlines())
    zc = chan(zb); frames = frame_info(align(zb, ref), ref)
    Td, Ti, psis, spread = learn(zc, coh.analytic(disc(zc)), frames)
    print(f"{len(frames)} clean frames aligned to the TX log; IF template cells filled {spread:.0%}")
    empty = np.linalg.norm(Ti, axis=-1) == 0                             # fill from the same tone / phase bin
    Ti[empty] = np.broadcast_to(Ti.sum(0, keepdims=True), Ti.shape)[empty]
    np.save(f"{os.path.dirname(os.path.abspath(__file__))}/ifT_init.npy", Ti / np.linalg.norm(Ti, axis=-1, keepdims=True))
    rng = np.random.default_rng(1)
    print("C/N0 dB-Hz  CNR(16k) dB   disc SER  FER     if SER  FER")
    for cn0 in np.arange(*(int(a) for a in sys.argv[2:4]), -1) if len(sys.argv) > 3 else np.arange(62, 35, -2):
        tot = {"disc": np.zeros(3), "if": np.zeros(3)}
        for _ in range(3):
            n0 = 10 ** (-cn0 / 10)                                           # carrier power 1
            w = (rng.normal(size=len(zb)) + 1j * rng.normal(size=len(zb))) * np.sqrt(n0 * FS / 2)
            r = run(chan(zb + w), frames, Td, Ti, psis)
            for k in tot: tot[k] += r[k]
        f = lambda k: f"{tot[k][0] / (tot[k][2] * d.DATA_SYMS):8.3f} {tot[k][1] / tot[k][2]:6.2f}"
        print(f"{cn0:8.0f}     {cn0 - 10 * np.log10(16000):6.1f}    {f('disc')}   {f('if')}", flush=True)
