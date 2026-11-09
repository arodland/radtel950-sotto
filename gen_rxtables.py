#!/usr/bin/env python3
"""dvrx_tables.h: constants for the C receiver (no libm on the radio): Hilbert FIR, tone phasors
at 19.2 kHz, +-0.5 rad rotation."""
import numpy as np
FS, SPS, HT = 19200, 16, 31
m = np.arange(HT) - (HT - 1) // 2
h = np.where(m == 0, 0, (1 - np.cos(np.pi * m)) / (np.pi * np.where(m == 0, 1, m))) * np.hamming(HT)
n = np.arange(SPS); tones = [600, 1200, 1800, 2400]
with open("dvrx_tables.h", "w") as f:
    f.write("/* gen_rxtables.py */\n#define DVRX_FS 19200\n#define DVRX_SPS 16\n#define DVRX_HT 31\n")
    f.write("static const float dvrx_hilb[DVRX_HT] = {" + ",".join(f"{v:.8f}f" for v in h) + "};\n")
    f.write("static const float dvrx_tone_re[4][DVRX_SPS] = {" + ",".join("{" + ",".join(f"{np.cos(2*np.pi*t*k/FS):.8f}f" for k in n) + "}" for t in tones) + "};\n")
    f.write("static const float dvrx_tone_im[4][DVRX_SPS] = {" + ",".join("{" + ",".join(f"{np.sin(2*np.pi*t*k/FS):.8f}f" for k in n) + "}" for t in tones) + "};\n")
    f.write(f"#define DVRX_ROT_RE {np.cos(0.5):.8f}f\n#define DVRX_ROT_IM {np.sin(0.5):.8f}f\n")
print("wrote dvrx_tables.h")
