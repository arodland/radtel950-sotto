# dvtx: digital voice on the Radtel RT-950 Pro

Sotto 1200 speech as 4-tone AFSK (600/1200/1800/2400 Hz, 1200 Bd, continuous phase) from the
RT-950 Pro's own mic, encoded on the radio's AT32F403A at 240 MHz.
About 240 ms mouth to ear (80 ms frames; the 40 ms format is kept as V2 for old recordings).

- `main.c`: test firmware.
  - PTT (either button) transmits. The power knob works.
  - Serial (UART4, 230400): `T<secs>` sends a timed over, `!` resets to the bootloader,
    `X`/`W` override BK4829 registers, `Pxn=0|1|z` drives a pin.
  - Fixed at 433.000 MHz, minimum APC.
- `dvframe.py`: the air formats, reference encoder and decoder. `gen_tables.py` writes the
  firmware's tables (`dvtables.h`) from it.
  - V3 (current): 80 ms frames, 4 sync + 92 data symbols, 2 superframes. K=7 rate-1/2 code:
    the 44 voicing / pitch / energy bits unpunctured, the 52 shape bits punctured to 84, each
    class in its own symbols so the receiver can trust one without the other (sotto decoder
    `good = 2` holds the last shape).
  - V2: 40 ms frames, 1 superframe, uniformly punctured. `DVFMT=v2` selects it.
- `coh.py`: coherent detection. The BK4829 tone generator is phase-continuous, so the signal
  is 4-level CPFSK with h = 0.5: a 2-state phase trellis with (previous tone, tone) templates,
  carrier phase carried across frames and held through fades. Raw SER 0.000 at S9+ against
  0.5-1% noncoherent; a little fewer lost frames at S2-S4 (FM threshold noise limits it there).
- `dv_live.py`: live receiver.
  - IC-9700 USB audio in, speech out on the default audio device. `--replay AIR.wav` runs the
    same receiver on a recording; `--noncoh` for the Goertzel + tone-pair trellis detector.
  - Symbol timing follows a slow loop (the radio's clock is steady); lock is held through
    about 0.5 s of bad frames, which the decoder conceals.
- `dv_rx.py`: test receiver. It keys the radio over serial, then counts bit errors against the
  superframes the radio reports. Uses `../rig.py` for recording.
- `gen_eq.py` -> `eq_coef.h`: the mic EQ (FIR, applied on the radio).
- `smeter.py`, `walk_eval.py`: walk-test logging (IC-9700 S-meter over CI-V) and offline
  analysis by signal level. Walk 2026-11-09 (V2, dummy-load leakage): clean at S5 and up,
  errors from S3-4, breaks up near S2.

Build (needs the sotto `fx-1200-encoder` branch, https://github.com/arodland/sotto, and the
open RT-950 firmware tree for startup and drivers):

    make SOTTO=~/code/radtel950/sotto RT=~/software/Radtel-RT950-Pro-Firmware   # dvtx.BTF
    make receiver                                                               # libsottodec.so, sotto_dec_fx_d1
    python3 ../flash.py dvtx.BTF                                                # radio already running a test image

## Mic audio

Chain on the radio: 255-tap "match" EQ in the 8 kHz interrupt (`gen_eq.py`), then a compressor and gentle gate per 1 ms block before each encode (`dyn.h`). Encoder mean is about 1.04 M cycles per 10 ms (about 104 of 240 MHz), worst call 3.0 M.

- The EQ fits the radio's mic to a PC-mic recording of the same speaker: up to +24 dB below 300 Hz (the BK4829 mic path high-passes hard), cut above 3 kHz, 6 dB headroom.
- The gate detects on a 300 Hz high-pass of the EQ output. Without that, the boosted bass noise kept the gate open and sotto turned it into a groan.
- Hiss comes from the mic front end: `gain_stage.py` shows the current gains (`2C=3F 7D=E91F 48=B092`) give the best SNR. Offline noise suppression and the low-delay encoder knobs made little audible difference.

Tools: `R<secs>` streams the raw mic (no TX); `mic_cap.py SECS OUT.wav` saves it; `voice_ab.py MIC.wav` runs chain variants through the radio's encoder (`make sotto_enc_fx_lo`) and decoder to `/tmp/dvtx/ab_*.wav`; `gain_stage.py` sweeps chip-0 gains with pink noise from the PC speaker.
