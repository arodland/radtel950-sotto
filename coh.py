"""Coherent detection of the dvtx 4-level CPFSK (h = 0.5) on the FM receiver's audio.
The BK4829 tone generator is phase-continuous across symbol changes (measured on air), so the
phase at every symbol boundary is 0 or pi relative to the first one: tone k (600 (k + 1) Hz)
advances it by pi (k + 1). A symbol's waveform is a (previous tone, tone) template times
exp(j (psi + theta)), psi the frame's carrier phase, theta the trellis state. Templates (4, 4, SPS)
complex are learned decision-directed; branch metric Re(e^-j(psi+theta) <s, T>) - |T|^2 / 2;
forward-backward max-sum over the 8 states (previous tone, theta) gives per-symbol tone metrics
for the FEC decoder, as dv_rx.isi_metrics does for the noncoherent receiver.
SPDX-License-Identifier: MIT"""
import numpy as np
from scipy.signal import firwin, lfilter, lfilter_zi

FS, SPS = 48000, 40
TONES = np.array([600, 1200, 1800, 2400])
ADV = np.array([1, 0, 1, 0])                   # phase advance of tone k in units of pi, mod 2
HTAPS = 127

def hilbert_fir(n=HTAPS):
    """type-III Hilbert transformer (odd length), band 200..3800 Hz, delay (n - 1) / 2"""
    m = np.arange(n) - (n - 1) / 2
    h = np.where(m == 0, 0, (1 - np.cos(np.pi * m)) / (np.pi * np.where(m == 0, 1, m)))
    return h * np.hamming(n)

class Analytic:
    """streaming analytic signal: real part delayed to match the Hilbert FIR"""
    def __init__(self):
        self.h = hilbert_fir(); self.zi = lfilter_zi(self.h, 1) * 0; self.d = (HTAPS - 1) // 2
        self.buf = np.zeros(self.d)
    def __call__(self, x):
        q, self.zi = lfilter(self.h, 1, x, zi=self.zi)
        r = np.concatenate([self.buf, x]); self.buf = r[len(r) - self.d:]
        return r[:len(x)] + 1j * q

def analytic(x):
    a = Analytic(); z = a(np.concatenate([x, np.zeros(a.d)])); return z[a.d:]

def ideal_templates(amp=1.0):
    n = np.arange(SPS); T = np.zeros((4, 4, SPS), complex)
    for c in range(4): T[:, c] = amp * np.exp(2j * np.pi * TONES[c] * n / FS)
    return T

def segs(z, t, n=48):
    """(n, SPS) complex segments of the n symbols starting at sample t"""
    return z[t + np.arange(n)[:, None] * SPS + np.arange(SPS)]

def corr(S, T):
    """<s_i, T[p][c]> for every symbol: (n, 4, 4) complex"""
    return np.einsum("is,pcs->ipc", S, T.conj())

def phase_est(C, prev, syms, theta0=0):
    """carrier phase from known symbols: C (n,4,4) correlations, prev/syms tone indices"""
    th = (theta0 + np.concatenate([[0], np.cumsum(ADV[syms[:-1]])])) % 2
    v = C[np.arange(len(syms)), prev, syms] * np.exp(-1j * np.pi * th)
    return np.angle(v.sum())

def metrics(C, T, psis, theta0=None, prev0=None):
    """per-symbol tone metrics (n, 4) by forward-backward max-sum over (prev tone, theta), for
    the best of several carrier-phase hypotheses psis (chosen by the best path's metric).
    theta0 / prev0 fix the start state when known (None: free). Returns (metrics, index)."""
    psis = np.atleast_1d(psis); H = len(psis)
    n = len(C); e = 0.5 * (np.abs(T) ** 2).sum(-1)                       # (p, c)
    b = (C[None] * np.exp(-1j * psis)[:, None, None, None]).real           # (h, i, p, c) for theta = 0
    br = np.stack([b - e, -b - e], 3)                                     # (h, i, p, th, c)
    al = np.full((n + 1, H, 4, 2), -1e18)
    for p in (range(4) if prev0 is None else [prev0]):
        for th in (range(2) if theta0 is None else [theta0]): al[0, :, p, th] = 0
    nxt = [(c, th, (th + ADV[c]) % 2) for c in range(4) for th in range(2)]
    for i in range(n):                                                     # forward, all hypotheses
        m = al[i][:, :, :, None] + br[:, i]                                # (h, p, th, c)
        for c, th, th2 in nxt: al[i + 1, :, c, th2] = m[:, :, th, c].max(1)
    h = int(al[n].reshape(H, -1).max(1).argmax())
    br = br[h]; al = al[:, h]
    be = np.zeros((n + 1, 4, 2)); out = np.empty((n, 4))
    for i in range(n - 1, -1, -1):                                         # backward + marginals
        nx = np.empty((2, 4))                                              # be of next state (th, c)
        for c, th, th2 in nxt: nx[th, c] = be[i + 1, c, th2]
        tot = al[i][:, :, None] + br[i] + nx[None]                         # (p, th, c)
        out[i] = tot.max((0, 1))
        for p in range(4):
            for th in range(2): be[i, p, th] = (br[i, p, th] + nx[th]).max()
    return out - out.max(1, keepdims=True), h

def update_templates(T, S, prev, syms, psi, theta0=0, alpha=0.02):
    th = (theta0 + np.concatenate([[0], np.cumsum(ADV[syms[:-1]])])) % 2
    rot = S * np.exp(-1j * (psi + np.pi * th))[:, None]
    for i in range(len(syms)):
        T[prev[i], syms[i]] += alpha * (rot[i] - T[prev[i], syms[i]])
    return T

if __name__ == "__main__":                       # self-check: ideal CPFSK + noise
    import dvframe as d
    rng = np.random.default_rng(1); T = ideal_templates()
    for sigma in (0.0, 0.5, 0.8):
        bits = rng.integers(0, 2, d.INFO); sy = d.frame_symbols(bits)
        ph = 0.7; x = []
        for k in sy:                             # continuous phase
            n = np.arange(SPS); x.append(np.cos(ph + 2 * np.pi * TONES[k] * n / FS)); ph += 2 * np.pi * TONES[k] * SPS / FS
        x = np.concatenate([np.zeros(200), np.concatenate(x), np.zeros(200)]) + rng.normal(0, sigma, 48 * SPS + 400)
        z = analytic(x); S = segs(z, 200); C = corr(S, T)
        prev = np.r_[sy[0], sy[:-1]]
        psi = phase_est(C[:4], prev[:4], sy[:4])
        met, _ = metrics(C, T, psi, theta0=0)
        ser = np.mean(met.argmax(1) != sy)
        nerr = int((d.frame_decode(met[d.SYNC_LEN:] / (met.std() + 1e-9)) != bits).sum())
        print(f"noise {sigma}: psi err {abs((psi - 0.7 + np.pi) % (2 * np.pi) - np.pi):.2f}, raw SER {ser:.3f}, bit errors {nerr}")
