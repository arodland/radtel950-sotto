"""RT-950 DV framing: 4-tone AFSK (continuous phase, h = 0.5) at 1200 Bd carrying sotto 1200.
Reference model for two formats; the receivers and the firmware follow it.

Common
  preamble  symbols alternating tone 0 / tone 3, sent until the first frame is ready (the
            transmitter's filler, also used if it ever runs dry): timing lock
  tones     index 0..3 = 600, 1200, 1800, 2400 Hz; a dibit (b0 b1) maps by Gray: 00->0 01->1 11->2 10->3
  frames    SYNC_LEN symbols of tones 0 / 3 ("1001", 1 = tone 3; 0 correlation with the preamble)
            then DATA_SYMS data symbols
  FEC       K=7 convolutional code, polynomials 133, 171 (octal), rate 1/2, 6 zero tail bits,
            g1 then g2 per input bit; punctured, interleaved, dibits -> symbols
  Payload   SF_PER_FRAME sotto 1200 superframes of 48 bits, MSB first

V2 (40 ms frames, 1 superframe, 2026-11-09 walk test): 48 symbols = 4 sync + 44 data.
  108 coded bits; coded bit i dropped when (i * 20) % 108 < 20 (20 bits, evenly spread) -> 88;
  interleave: bit k goes to position (k % 8) * 11 + k // 8.

V3 (80 ms frames, 2 superframes, unequal error protection): 96 symbols = 4 sync + 92 data.
  The 96 payload bits are reordered so the two superframes' critical fields come first
  (voicing A, voicing B, energy A, energy B, pitch A, pitch B of superframe 0 then 1: 44 bits,
  the least damaging steps last) then the 52 shape bits (shape A stages 1, 2 and shape B).
  204 coded bits; the critical half (coded bits 0..87) is not punctured; of the shape half's 104
  coded bits (88..191) bit j is dropped when (j * 20) % 104 < 20 -> 84; the 12 tail bits stay.
  Class A = critical + tail = 100 bits = 50 symbols in data slots j with (j * 50) % 92 < 50
  (evenly spread); class B = shape = 84 bits = 42 symbols in the other slots. Within a class,
  bit k goes to position (k % 10) * 10 + k // 10 (A) or (k % 12) * 7 + k // 12 (B).
  A receiver judges each class by its own re-encoded symbol error rate, so a frame can carry
  trusted pitch / energy / voicing with lost shape (sotto decoder: good = 2 holds the last shape).
Latency: a frame is sent once its superframes exist and decoded once all of it is in.
SPDX-License-Identifier: MIT"""
import numpy as np

K, G1, G2 = 7, 0o133, 0o171
PRE_SYMS, SYNC_LEN = 240, 4
SYNC = np.array([int(c) for c in "1001"])
GRAY = {(0, 0): 0, (0, 1): 1, (1, 1): 2, (1, 0): 3}
TONE_BITS = {v: k for k, v in GRAY.items()}
SF_BITS = 48
# sotto 1200 superframe fields: (name, first bit, width); critical ones first in V3's order
SF_FIELDS = [("vA", 0, 2), ("pA", 2, 7), ("eA", 9, 5), ("sA1", 14, 9), ("sA2", 23, 9),
             ("vB", 32, 2), ("pB", 34, 3), ("eB", 37, 3), ("sB", 40, 8)]
CRIT_ORDER = ["vA", "vB", "eA", "eB", "pA", "pB"]
SHAPE_ORDER = ["sA1", "sA2", "sB"]

def parity(x): return bin(x).count("1") & 1

def conv_encode(bits):
    s, out = 0, []
    for b in list(bits) + [0] * (K - 1):
        s = ((s << 1) | int(b)) & 0x7F
        out += [parity(s & G1), parity(s & G2)]
    return np.array(out, np.int8)

class Format:
    def __init__(self, name, sf_per_frame, data_syms, keep, ilv, order=None, classes=None):
        self.name = name; self.SF_PER_FRAME = sf_per_frame
        self.INFO = SF_BITS * sf_per_frame; self.CODED = 2 * (self.INFO + K - 1)
        self.DATA_SYMS = data_syms; self.FRAME_SYMS = SYNC_LEN + data_syms
        self.KEEP = keep; self.ILV = ilv                           # kept coded bit k -> position
        self.ORDER = order if order is not None else np.arange(self.INFO)   # payload bit -> info bit
        self.INV = np.argsort(self.ORDER)
        self.classes = classes                                      # symbol index -> class (A=0, B=1) or None
        assert keep.sum() == 2 * data_syms == len(ilv) and sorted(ilv) == list(range(2 * data_syms))

    def frame_symbols(self, payload_bits):
        """INFO payload bits (superframes in order) -> FRAME_SYMS tone indices"""
        info = np.asarray(payload_bits, np.int8)[self.ORDER]
        c = conv_encode(info)[self.KEEP]
        il = np.empty_like(c); il[self.ILV] = c
        data = [GRAY[(int(il[2 * i]), int(il[2 * i + 1]))] for i in range(self.DATA_SYMS)]
        return np.concatenate([SYNC * 3, data]).astype(np.int8)

    def frame_decode(self, met):
        """(DATA_SYMS, 4) tone metrics -> INFO payload bits"""
        sb = soft_bits(met)
        coded = np.zeros(self.CODED); coded[self.KEEP] = sb[self.ILV]
        return viterbi(coded, self.INFO)[self.INV]

    def class_ser(self, met, bits):
        """re-encoded symbol error rate per class (A, B), or (ser, ser) for an unclassed format"""
        sy = self.frame_symbols(bits)[SYNC_LEN:]; hard = met.argmax(1); err = hard != sy
        if self.classes is None: return err.mean(), err.mean()
        return tuple(err[self.classes == c].mean() for c in (0, 1))

def _v2():
    coded = 2 * (SF_BITS + K - 1); data = 44; npunc = coded - 2 * data
    keep = np.array([(i * npunc) % coded >= npunc for i in range(coded)])
    ilv = np.array([(k % 8) * 11 + k // 8 for k in range(2 * data)])
    return Format("v2", 1, data, keep, ilv)

def _v3():
    data = 92; info = 2 * SF_BITS; coded = 2 * (info + K - 1)                 # 204
    fields = {n: (a, w) for n, a, w in SF_FIELDS}
    order = []
    for names in (CRIT_ORDER, SHAPE_ORDER):
        for n in names:
            for sf in range(2):
                a, w = fields[n]; order += [sf * SF_BITS + a + j for j in range(w)]
    order = np.array(order); ncrit = sum(fields[n][1] for n in CRIT_ORDER) * 2  # 44
    keep = np.ones(coded, bool)
    shape_lo, shape_hi = 2 * ncrit, 2 * info                                   # 88..192
    nsh = shape_hi - shape_lo; npunc = coded - 2 * data                        # 104, 20
    for j in range(nsh): keep[shape_lo + j] = (j * npunc) % nsh >= npunc
    # kept bits in order: A = critical (88) ... wait, kept order is coded order: critical, shape, tail
    na, nb = 2 * ncrit + 2 * (K - 1), nsh - npunc                              # 100, 84
    slots_a = [j for j in range(data) if (j * (na // 2)) % data < na // 2]
    slots_b = [j for j in range(data) if j not in slots_a]
    assert len(slots_a) == na // 2 and len(slots_b) == nb // 2
    kept_idx = np.flatnonzero(keep)                                            # coded index of kept bit k
    ilv = np.empty(2 * data, int); classes = np.empty(data, int)
    classes[slots_a] = 0; classes[slots_b] = 1
    for k, ci in enumerate(kept_idx):
        if ci < shape_lo or ci >= shape_hi:                                    # class A: critical + tail
            ka = k if ci < shape_lo else k - nb                                # tail bits follow the critical ones
            pos = (ka % 10) * 10 + ka // 10; ilv[k] = 2 * slots_a[pos // 2] + pos % 2
        else:
            kb = k - 2 * ncrit; pos = (kb % 12) * 7 + kb // 12; ilv[k] = 2 * slots_b[pos // 2] + pos % 2
    return Format("v3", 2, data, keep, ilv, order, classes)

V2, V3 = _v2(), _v3()
FORMATS = {"v2": V2, "v3": V3}

def preamble():
    return np.array([0, 3] * (PRE_SYMS // 2), np.int8)

# ---------------------------------------------------------------- receive side
def soft_bits(met):
    """per-symbol tone metrics (n, 4) -> LLR-ish soft bits (n*2), + = bit 1 (max-log over tones)"""
    out = np.empty((len(met), 2))
    for j in range(2):
        one = [t for t in range(4) if TONE_BITS[t][j] == 1]; zero = [t for t in range(4) if TONE_BITS[t][j] == 0]
        out[:, j] = met[:, one].max(1) - met[:, zero].max(1)
    return out.ravel()

_VT = {}
def viterbi(soft, info):
    """soft (CODED,) with 0 for punctured bits, + = 1 -> info bits (zero-tailed).
    State = last 6 input bits; next state n has predecessors n >> 1 and (n >> 1) | 32 with input n & 1."""
    ns = 1 << (K - 1)
    if not _VT:
        n = np.arange(ns); b = n & 1; p0 = n >> 1; p1 = p0 | (ns >> 1)
        reg0, reg1 = (p0 << 1) | b, (p1 << 1) | b                    # 7-bit registers into each state
        sgn = lambda r, g: np.array([parity(int(x) & g) for x in r]) * 2 - 1
        _VT.update(p0=p0, p1=p1, a0=sgn(reg0, G1), c0=sgn(reg0, G2), a1=sgn(reg1, G1), c1=sgn(reg1, G2))
    p0, p1, a0, c0, a1, c1 = (_VT[k] for k in ("p0", "p1", "a0", "c0", "a1", "c1"))
    pm = np.full(ns, -1e18); pm[0] = 0
    T = info + K - 1
    dec = np.zeros((T, ns), np.int64)
    for t in range(T):
        x, y = soft[2 * t], soft[2 * t + 1]
        m0 = pm[p0] + a0 * x + c0 * y
        m1 = pm[p1] + a1 * x + c1 * y
        take1 = m1 > m0
        pm = np.where(take1, m1, m0); dec[t] = np.where(take1, p1, p0)
    s, bits = 0, []                                               # zero tail: end in state 0
    for t in range(T - 1, -1, -1):
        bits.append(s & 1); s = dec[t][s]
    return np.array(bits[::-1][:info], np.int8)

# V2 names kept for the existing receivers
FRAME_SYMS, DATA_SYMS, SF_PER_FRAME, INFO = V2.FRAME_SYMS, V2.DATA_SYMS, V2.SF_PER_FRAME, V2.INFO
frame_symbols, frame_decode = V2.frame_symbols, V2.frame_decode

if __name__ == "__main__":
    rng = np.random.default_rng(5)
    for F in (V2, V3):
        for trial, sigma in enumerate((0.0, 0.3, 0.3, 0.45)):
            p = rng.integers(0, 2, F.INFO)
            sy = F.frame_symbols(p)
            assert len(sy) == F.FRAME_SYMS and (sy[:SYNC_LEN] == SYNC * 3).all()
            met = np.eye(4)[sy[SYNC_LEN:]] * 1.0 + rng.normal(0, sigma, (F.DATA_SYMS, 4))
            raw_ser = np.mean(met.argmax(1) != sy[SYNC_LEN:])
            bits = F.frame_decode(met); nerr = int((bits != p).sum())
            print(f"{F.name} trial {trial}: noise {sigma}, raw SER {raw_ser:.3f} -> payload bit errors {nerr}, class SER {np.round(F.class_ser(met, bits), 3)}")
            if raw_ser < 0.06: assert nerr == 0
    # V3 bookkeeping
    assert sorted(V3.ORDER) == list(range(96)) and V3.KEEP.sum() == 184 and (V3.classes == 0).sum() == 50
    print("dvframe self-check ok")
