#!/usr/bin/env python3
"""Receive one dvtx over from the IC-9700 (USB audio) and decode it.
usage: dv_rx.py SECS            key the radio (dvtx firmware on /dev/ttyUSB2) for SECS and receive
       dv_rx.py --listen SECS   record for SECS while someone keys the radio with PTT
       dv_rx.py --file WAV LOG  decode a saved recording against a saved radio log
Writes dv_rx.wav (decoded off air) and dv_ref.wav (the radio's own superframes, no channel)."""
import os, subprocess, sys, threading, time, wave
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dvframe as d

def radio_port():
    """the RT-950's USB-serial adapter: it re-enumerates (ttyUSB2 -> ttyUSB3 ...) when RF from the
    antenna resets it during TX, so take whichever ttyUSB isn't the IC-9700's (USB0 CI-V, USB1)"""
    import glob, time
    for _ in range(40):
        p = sorted(x for x in glob.glob("/dev/ttyUSB*") if x not in ("/dev/ttyUSB0", "/dev/ttyUSB1"))
        if p: return p[0]
        time.sleep(0.25)
    raise SystemExit("radio serial port not found")

FS, SPS = 48000, 40                                   # 1200 Bd
TONES = np.array([600, 1200, 1800, 2400])
FMT = d.FORMATS[os.environ.get("DVFMT", "v3")]      # air format (dvframe.py); DVFMT=v2 for old recordings
FRAME = FMT.FRAME_SYMS * SPS
DEC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sotto_dec_fx_d1")   # LR_INTERP=0: 1 superframe of delay
import tempfile
SCRATCH = os.path.join(tempfile.gettempdir(), "dvtx")   # recordings, logs, intermediate bitstreams
os.makedirs(SCRATCH, exist_ok=True)

def tone_power(x):
    """|Goertzel| of each tone over the SPS-sample window starting at every sample -> (N, 4)"""
    n = np.arange(SPS)
    out = np.empty((len(x) - SPS + 1, 4))
    for j, f in enumerate(TONES):
        out[:, j] = np.abs(np.convolve(x, np.exp(2j * np.pi * f * n / FS)[::-1], "valid"))
    return out

def sync_score(P):
    """correlation of tone 3 - tone 0 against the sync word, normalised, at every start sample"""
    diff = P[:, 3] - P[:, 0]; tot = P.max(1) + 1e-9
    s = np.zeros(len(P) - d.SYNC_LEN * SPS); w = d.SYNC * 2 - 1
    for i, c in enumerate(w):
        s += c * (diff / tot)[i * SPS: i * SPS + len(s)]
    return s / d.SYNC_LEN

def frames(P, thresh=0.35, win=12):
    """frame start samples: the frame phase that collects the most sync energy over the whole
    recording (one strong false peak can't win), then every frame on it whose sync is present"""
    s = sync_score(P)
    n = len(s) // FRAME
    fold = np.clip(s[:n * FRAME], 0, None).reshape(n, FRAME).sum(0)
    fold = np.maximum.reduce([np.roll(fold, k) for k in range(-win, win + 1)])   # tolerate drift
    ph = int(fold.argmax())
    starts = []
    for c in range(ph, len(s), FRAME):
        lo, hi = max(c - win, 0), min(c + win + 1, len(s))
        j = lo + int(s[lo:hi].argmax())
        if j + FRAME <= len(P): starts.append((j, s[j] >= thresh))
    ok = [i for i, (_, g) in enumerate(starts) if g]
    return ([j for j, _ in starts[ok[0]:ok[-1] + 1]] if ok else []), s

def grid(P, starts):
    """the radio clocks symbols from a timer, so frames sit on a straight line: robust fit of the
    sync positions, then each frame re-timed by +-6 samples on tone-energy concentration"""
    starts = np.array(starts); i = np.round((starts - starts[0]) / FRAME)
    keep = np.ones(len(starts), bool)
    for _ in range(4):
        b, a = np.polyfit(i[keep], starts[keep], 1)
        res = starts - (a + b * i); keep = np.abs(res) < max(8, 3 * np.median(np.abs(res[keep])))
    def q(t):
        M = P[t + np.arange(FMT.FRAME_SYMS) * SPS]; srt = np.sort(M, 1)
        return np.mean(srt[:, -1] / (srt[:, -2] + 1e-9))
    out = []
    for k in i:
        t = int(round(a + b * k))
        out.append(max(range(t - 6, t + 7), key=q))
    return out

def isi_metrics(F, mu):
    """F (n, 4) tone powers of consecutive symbols, mu (4, 4, 4) mean powers for (previous, current)
    tone. Forward-backward min-sum over the (previous -> current) trellis: per-symbol cost of each
    tone with the neighbours marginalised (max-log). Returns metrics, higher = more likely."""
    dk = ((F[:, None, None, :] - mu[None]) ** 2).sum(-1)          # (n, prev, cur)
    n = len(F); al = np.zeros((n, 4)); be = np.zeros((n, 4))
    a = np.zeros(4)
    for k in range(n):
        a = (a[:, None] + dk[k]).min(0); a -= a.min(); al[k] = a
    b = np.zeros(4)
    for k in range(n - 1, -1, -1):
        be[k] = b; b = (dk[k] + b[None, :]).min(1); b -= b.min()
    return -(al + be)

def frame_feats(P, t):
    M = P[t + np.arange(FMT.FRAME_SYMS) * SPS]
    return M / np.median(M.max(1))

def decode(x):
    """pass 1: per-symbol decisions + FEC; then learn tone-pair templates from the re-encoded
    frames (decision-directed) and decode again with the ISI-aware metrics"""
    x = x - x.mean()
    P = tone_power(x)
    starts, s = frames(P)
    if len(starts) < 3: return [], starts, s, [1.0]
    starts = [t for t in grid(P, starts) if t + FRAME <= len(P)]
    feats = [frame_feats(P, t) for t in starts]
    bits1 = [FMT.frame_decode(F[d.SYNC_LEN:]) for F in feats]
    syms = [FMT.frame_symbols(b) for b in bits1]
    F_all = np.concatenate(feats); T = np.concatenate(syms); prev = np.r_[T[0], T[:-1]]
    mu = np.zeros((4, 4, 4))
    for a in range(4):
        for c in range(4):
            m = (prev == a) & (T == c)
            mu[a, c] = F_all[m].mean(0) if m.sum() >= 4 else np.eye(4)[c]
    out, sers = [], []
    for F in feats:
        met = isi_metrics(F, mu)[d.SYNC_LEN:]
        bits = FMT.frame_decode(met / (np.std(met) + 1e-9))
        out.append(np.packbits(bits).tobytes())
        sers.append(np.mean(met.argmax(1) != FMT.frame_symbols(bits)[d.SYNC_LEN:]))
    return out, starts, s, sers

def parse_log(lines):
    sf = {}
    for ln in lines:
        p = ln.split()
        if len(p) >= 4 and p[0] == "S": sf[(int(p[1]), int(p[2]))] = bytes.fromhex(p[3])
    n = max(f for f, _ in sf) + 1 if sf else 0
    return [b"".join(sf.get((f, k), bytes(6)) for k in range(FMT.SF_PER_FRAME)) for f in range(n)]

def sotto_wav(payload, name):
    bit = f"{SCRATCH}/{name}.bit"; raw = f"{SCRATCH}/{name}.raw"
    open(bit, "wb").write(payload)
    subprocess.run([DEC, "-1200", bit, raw], check=True)
    y = np.fromfile(raw, np.int16)
    w = wave.open(f"{name}.wav", "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(8000)
    w.writeframes(y.tobytes()); w.close()

def over(secs, key=True):
    import serial, rig
    wavp = f"{SCRATCH}/dv_air.wav"; logp = f"{SCRATCH}/dv_log.txt"
    res = {}
    th = threading.Thread(target=lambda: res.__setitem__("x", rig.record(secs + 4, wavp))); th.start()
    time.sleep(0.8)
    lines = []
    with serial.Serial(radio_port(), 230400, timeout=0.5) as ser:
        ser.reset_input_buffer()
        for c in [c for c in os.environ.get("DVCMD", "").split(";") if c]:   # e.g. DVCMD="X;X1,40,3800"
            ser.write(f"{c}\r".encode()); time.sleep(0.05)
        if key: ser.write(f"T{secs}\r".encode())
        t_end = time.time() + secs + 8
        while time.time() < t_end:
            ln = ser.readline().decode(errors="replace").strip()
            if ln: lines.append(ln)
            if ln == "DONE": break
    th.join()
    open(logp, "w").write("\n".join(lines) + "\n")
    print(*[l for l in lines if not l.startswith("S ")], sep="\n")
    return res["x"], lines

if __name__ == "__main__":
    if sys.argv[1] == "--listen":                 # PTT overs: record SECS, the radio keys itself
        x, lines = over(int(sys.argv[2]), key=False)
    elif sys.argv[1] == "--file":
        w = wave.open(sys.argv[2]); x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(float)
        lines = open(sys.argv[3]).read().splitlines()
    else:
        x, lines = over(int(sys.argv[1]))
    ref = parse_log(lines)
    got, starts, s, sers = decode(x)
    print(f"radio sent {len(ref)} frames; found {len(got)} syncs "
          f"(scores {np.min(s[starts]):.2f}..{np.max(s[starts]):.2f}), raw SER mean {np.mean(sers):.4f}")
    # align: the received frame list may start late or miss the first frame
    best = max(range(-30, 31), key=lambda o: sum(g == ref[i + o] for i, g in enumerate(got) if 0 <= i + o < len(ref)))
    pairs = [(g, ref[i + best]) for i, g in enumerate(got) if 0 <= i + best < len(ref)]
    nbits = sum(8 * len(r) for _, r in pairs)
    err = sum(bin(int.from_bytes(g, "big") ^ int.from_bytes(r, "big")).count("1") for g, r in pairs)
    bad = sum(g != r for g, r in pairs)
    print(f"offset {best}: {len(pairs)} frames compared, bit errors {err}/{nbits}, bad frames {bad}, "
          f"missing frames {len(ref) - len(pairs)}")
    rx = [None] * len(ref)
    for i, g in enumerate(got):
        if 0 <= i + best < len(ref): rx[i + best] = g
    sotto_wav(b"".join(r if r is not None else bytes(len(ref[0])) for r in rx), "dv_rx")
    sotto_wav(b"".join(ref), "dv_ref")
    print("wrote dv_rx.wav, dv_ref.wav")
