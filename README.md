# dvtx: digital voice on the Radtel RT-950 Pro

Sotto 1200 speech as 4-tone AFSK (600/1200/1800/2400 Hz, 1200 Bd) from the RT-950 Pro's own mic,
encoded on the radio's AT32F403A at 240 MHz.
About 200 ms mouth to ear.

- `main.c`: test firmware.
  - PTT (either button) transmits. The power knob works.
  - Serial (UART4, 230400): `T<secs>` sends a timed over, `!` resets to the bootloader,
    `X`/`W` override BK4829 registers, `Pxn=0|1|z` drives a pin.
  - Fixed at 433.000 MHz, minimum APC.
- `dvframe.py`: the air format, reference encoder and decoder.
  - 40 ms frames: 4 sync + 44 data symbols.
  - K=7 rate-1/2 code, punctured, interleaved.
- `dv_live.py`: live receiver.
  - IC-9700 USB audio in, speech out on the default audio device.
  - Tone-pair (ISI) trellis metrics, which are needed on this link.
- `dv_rx.py`: test receiver. It keys the radio over serial, then counts bit errors against the
  superframes the radio reports. Uses `../rig.py` for recording.
- `gen_eq.py` -> `eq_coef.h`: the mic EQ (FIR, applied on the radio).

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
