#!/usr/bin/env python3
"""A26 mic EQ (chosen by ear, 2026-11-09) as a 255-tap fixed-point FIR for the 8 kHz capture: eq_coef.h.
Gains are relative to 1.2 kHz (0 dB): +20 dB at 100 Hz, +26 at 150, +13.2 at 300, +4.9 at 600,
-8 at 2 kHz, -9.6 at 2.9 kHz and up."""
import numpy as np
from scipy.signal import firwin2, freqz
F = [0, 50, 100, 150, 300, 600, 1200, 2000, 2900, 4000]
DB = [0, 0, 20, 26, 13.2, 4.9, 0, -8, -9.6, -9.6]
h = firwin2(255, F, 10 ** (np.array(DB) / 20), fs=8000)
SHIFT = 15
while np.abs(h).max() * (1 << SHIFT) >= 32767: SHIFT -= 1  # largest Q format that fits int16
q = np.round(h * (1 << SHIFT)).astype(int)
w, H = freqz(q / (1 << SHIFT), worN=[150, 300, 1200, 2900], fs=8000)
print("peak coefficient", round(np.abs(h).max(), 3), "-> Q", SHIFT)
print("check (dB at 150/300/1200/2900 Hz):", np.round(20 * np.log10(np.abs(H)), 1))
with open("eq_coef.h", "w") as f:
    f.write(f"/* A26 mic EQ, 255-tap FIR, Q{SHIFT} (gen_eq.py). 0 dB at 1.2 kHz. */\n#define EQ_TAPS 255\n#define EQ_SHIFT {SHIFT}\n")
    f.write("static const int16_t eq_coef[EQ_TAPS] = {\n")
    for i in range(0, 255, 15): f.write("  " + ", ".join(str(v) for v in q[i:i + 15]) + ",\n")
    f.write("};\n")
