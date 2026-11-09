"""RT-950 DV framing (v2, 40 ms frames): 4-tone AFSK at 1200 Bd carrying sotto 1200, reference model.

Air format
  preamble  symbols alternating tone 0 / tone 3, sent until the first frame is ready (the
            transmitter's filler, also used if it ever runs dry): timing lock
  frames    each FRAME_SYMS = 48 symbols (40 ms):
              SYNC (4 symbols of tones 0 / 3, 1 = tone 3) then DATA_SYMS = 44 data symbols
  tones     index 0..3 = 600, 1200, 1800, 2400 Hz; a dibit (b0 b1) maps by Gray: 00->0 01->1 11->2 10->3
Payload per frame: SF_PER_FRAME = 1 sotto 1200 superframe = 48 bits, MSB first, in order.
FEC: K=7 convolutional code, polynomials 133, 171 (octal), rate 1/2, 6 zero tail bits
  -> 108 coded bits (pairs: g1 then g2 output per input bit);
  puncture: coded bit i is dropped when (i * 20) % 108 < 20 (20 bits, evenly spread) -> 88;
  interleave: bit k of the 88 goes to position (k % 8) * 11 + k // 8;
  then consecutive pairs of the interleaved bits are dibits -> 44 symbols.
Latency: a frame can be sent once its 2 superframes exist and decoded once all of it is in,
so framing adds 2 x 40 ms; the frame is one superframe, so nothing waits for a second one.
The sync is short: receivers find the frame phase by folding it over many frames.
SPDX-License-Identifier: MIT"""
import numpy as np

K, G1, G2 = 7, 0o133, 0o171
PRE_SYMS, SYNC_LEN, FRAME_SYMS = 240, 4, 48
DATA_SYMS = FRAME_SYMS - SYNC_LEN                   # 44
SF_PER_FRAME, INFO = 1, 48
CODED = 2 * (INFO + K - 1)                          # 108
NPUNC = CODED - 2 * DATA_SYMS                       # 20
SYNC = np.array([int(c) for c in "1001"])         # 0 against the preamble at both phases
GRAY = {(0, 0): 0, (0, 1): 1, (1, 1): 2, (1, 0): 3}
TONE_BITS = {v: k for k, v in GRAY.items()}
KEEP = np.array([(i * NPUNC) % CODED >= NPUNC for i in range(CODED)])
ILV = np.array([(k % 8) * 11 + k // 8 for k in range(2 * DATA_SYMS)])   # k -> position
assert KEEP.sum() == 2 * DATA_SYMS and sorted(ILV) == list(range(2 * DATA_SYMS))

def parity(x): return bin(x).count("1") & 1

def conv_encode(bits):
    s, out = 0, []
    for b in list(bits) + [0] * (K - 1):
        s = ((s << 1) | int(b)) & 0x7F
        out += [parity(s & G1), parity(s & G2)]
    return np.array(out, np.int8)

def frame_symbols(payload_bits):
    """48 payload bits -> 48 tone indices (sync + data)"""
    c = conv_encode(payload_bits)[KEEP]
    il = np.empty_like(c); il[ILV] = c
    data = [GRAY[(int(il[2 * i]), int(il[2 * i + 1]))] for i in range(DATA_SYMS)]
    return np.concatenate([SYNC * 3, data]).astype(np.int8)

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

def viterbi(soft):
    """soft (CODED,) with 0 for punctured bits, + = 1 -> INFO bits (zero-tailed).
    State = last 6 input bits; next state n has predecessors n >> 1 and (n >> 1) | 32 with input n & 1."""
    ns = 1 << (K - 1)
    n = np.arange(ns); b = n & 1; p0 = n >> 1; p1 = p0 | (ns >> 1)
    reg0, reg1 = (p0 << 1) | b, (p1 << 1) | b                    # 7-bit registers into each state
    sgn = lambda r, g: np.array([parity(int(x) & g) for x in r]) * 2 - 1
    a0, c0, a1, c1 = sgn(reg0, G1), sgn(reg0, G2), sgn(reg1, G1), sgn(reg1, G2)
    pm = np.full(ns, -1e18); pm[0] = 0
    T = INFO + K - 1
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
    return np.array(bits[::-1][:INFO], np.int8)

def frame_decode(met):
    """(DATA_SYMS, 4) tone metrics of one frame's data -> 48 payload bits"""
    sb = soft_bits(met)
    coded = np.zeros(CODED); coded[KEEP] = sb[ILV]
    return viterbi(coded)

if __name__ == "__main__":
    rng = np.random.default_rng(5)
    for trial, sigma in enumerate((0.0, 0.3, 0.3, 0.45)):
        p = rng.integers(0, 2, INFO)
        sy = frame_symbols(p)
        assert len(sy) == FRAME_SYMS and (sy[:SYNC_LEN] == SYNC * 3).all()
        met = np.eye(4)[sy[SYNC_LEN:]] * 1.0 + rng.normal(0, sigma, (DATA_SYMS, 4))
        raw_ser = np.mean(met.argmax(1) != sy[SYNC_LEN:])
        nerr = int((frame_decode(met) != p).sum())
        print(f"trial {trial}: noise {sigma}, raw SER {raw_ser:.3f} -> payload bit errors {nerr}")
        if raw_ser < 0.06: assert nerr == 0
    print("dvframe self-check ok")
