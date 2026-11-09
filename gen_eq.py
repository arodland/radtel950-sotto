#!/usr/bin/env python3
"""Mic EQ "match" (chosen by ear, 2026-11-09) as a 255-tap fixed-point FIR for the 8 kHz capture: eq_coef.h.
The curve gives the radio's raw mic capture (mic_cap.py) the long-term speech spectrum of a PC mic,
same speaker: voice_ab.py match_eq(), Welch 256-pt bins (31.25 Hz), 5-bin smoothing, 0 dB at
800-1200 Hz, boost capped at +30 dB, -40 dB above 3 kHz, -20 dB below 60 Hz.
(Previous: "A26", +20 dB@100, +26@150, +13.2@300, +4.9@600, 0@1.2k, -8@2k, -9.6@2.9k+.)"""
import numpy as np
from scipy.signal import firwin2, freqz
DB = [-20.0, -20.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 24.1, 18.4, 14.0, 12.4, 11.3, 10.1, 8.9, 7.3, 6.4,
      6.0, 5.7, 4.6, 3.8, 2.6, 1.3, 0.0, -0.6, -1.0, -1.3, -1.5, -1.1, -0.1, 0.8, 1.6, 2.0, 1.9, 1.3, 0.0, -1.0,
      -1.7, -2.0, -1.8, -1.0, -0.3, 0.2, 0.2, -0.1, -0.4, -0.7, -1.1, -1.9, -2.5, -3.5, -4.8, -5.9, -6.8, -7.5,
      -7.9, -8.3, -8.3, -8.3, -8.0, -7.4, -6.6, -5.9, -5.3, -4.9, -4.5, -4.3, -4.4, -4.4, -4.5, -4.8, -4.9, -5.0,
      -5.4, -6.1, -7.1, -8.4, -9.4, -9.8, -9.4, -8.3, -7.4, -6.5, -5.7, -5.6, -5.8, -6.2, -6.8, -7.7, -8.3, -8.8,
      -8.4, -7.3, -6.1, -4.6, -3.3] + [-40.0] * 32
F = np.arange(129) * 31.25
GAIN_DB = -6                     # headroom before the compressor (its thresholds sit 6 dB lower)
h = firwin2(255, F, 10 ** ((np.array(DB) + GAIN_DB) / 20), fs=8000)
SHIFT = 15
while np.abs(h).max() * (1 << SHIFT) >= 32767: SHIFT -= 1  # largest Q format that fits int16
q = np.round(h * (1 << SHIFT)).astype(int)
w, H = freqz(q / (1 << SHIFT), worN=[100, 150, 300, 1000, 2000, 3500], fs=8000)
print("peak coefficient", round(np.abs(h).max(), 3), "-> Q", SHIFT)
print("check (dB at 100/150/300/1000/2000/3500 Hz):", np.round(20 * np.log10(np.abs(H)), 1))
with open("eq_coef.h", "w") as f:
    f.write(f"/* mic EQ \"match\", 255-tap FIR, Q{SHIFT} (gen_eq.py). -6 dB at 800-1200 Hz. */\n#define EQ_TAPS 255\n#define EQ_SHIFT {SHIFT}\n")
    f.write("static const int16_t eq_coef[EQ_TAPS] = {\n")
    for i in range(0, 255, 15): f.write("  " + ", ".join(str(v) for v in q[i:i + 15]) + ",\n")
    f.write("};\n")
