#!/usr/bin/env python3
"""Walk-test analysis: split a continuous IC-9700 air recording into overs using the S-meter log,
decode each over offline (dv_rx.decode: whole-over two-pass receiver) and report per over
S-meter, carrier time, frames decoded and raw SER; per 0.5 s S-meter sample the frame SERs there.
usage: walk_eval.py DIR   (DIR has air.wav, air_start.txt, smeter.tsv from the walk setup)"""
import os, sys, time, wave
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dv_rx import decode, FS, FRAME

def main(dr):
    t0 = float(open(f"{dr}/air_start.txt").read())
    w = wave.open(f"{dr}/air.wav"); assert w.getframerate() == FS
    d = np.loadtxt(f"{dr}/smeter.tsv"); ts, sm = d[:, 0], d[:, 1]
    on = sm > 8
    edges = np.flatnonzero(np.diff(on.astype(int))) + 1
    rows = []
    for s in np.split(np.arange(len(sm)), edges):
        if not on[s[0]]: continue
        a, b = ts[s[0]] - 1.0, ts[s[-1]] + 1.5            # pad: the meter samples every 0.5 s
        i0 = max(int((a - t0) * FS), 0); w.setpos(i0)
        x = np.frombuffer(w.readframes(int((b - a) * FS)), np.int16).astype(float)
        got, starts, sc, sers = decode(x)
        sers = np.array(sers if got else [])
        good = np.flatnonzero(sers < 0.1)                  # noise frames score 0.15-0.45: count
        lo, hi = (good[0], good[-1] + 1) if len(good) else (0, 0)   # only first..last good frame
        keep = []
        for k, st in list(enumerate(starts[:len(sers)]))[lo:hi]:            # per frame: S-meter at its time;
            tf = t0 + (i0 + st + FRAME / 2) / FS                 # frames in the padding (no carrier) dropped
            v = sm[np.argmin(np.abs(ts - tf))]
            if v > 8 and ts[s[0]] - 0.5 <= tf <= ts[s[-1]] + 0.5: rows.append((v, sers[k])); keep.append(sers[k])
        keep = np.array(keep)
        print(f"{time.strftime('%H:%M:%S', time.localtime(ts[s[0]]))}  carrier {len(s) * 0.5:4.1f} s  "
              f"S-meter med {int(np.median(sm[s])):3d} min {int(sm[s].min()):3d}  frames {len(keep):3d}  "
              f"raw SER {np.mean(keep) if len(keep) else float('nan'):.3f}  bad (SER>=0.2) {int((keep >= 0.2).sum())}")
    if rows:
        r = np.array(rows); print("\nS-meter band   frames  mean raw SER  frames SER>=0.2")
        for lo, hi in ((0, 20), (20, 40), (40, 60), (60, 90), (90, 120), (120, 256)):
            m = (r[:, 0] >= lo) & (r[:, 0] < hi)
            if m.any(): print(f"{lo:3d}-{hi:3d}       {m.sum():5d}   {r[m, 1].mean():.4f}        {int((r[m, 1] >= 0.2).sum())}")

if __name__ == "__main__":
    main(sys.argv[1])
