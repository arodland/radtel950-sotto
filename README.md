# dvtx: digital voice on the Radtel RT-950 Pro

Sotto 1200 speech as 4-tone AFSK (600/1200/1800/2400 Hz, 1200 Bd, continuous phase) from the
RT-950 Pro's own mic, encoded on the radio's AT32F403A at 240 MHz, and received on the radio
itself. About 235 ms mouth to ear (40 ms frames, decoder rebuilds a lone lost superframe).

- `main.c`: test firmware. 432.950 MHz, about 2 W (BK PA low + APC 2200; factory "low" is 1.2 W).
  - PTT1 sends DV (red LED), PTT2 sends plain FM voice at the same power (green LED).
  - Idle: FM receive on the speaker (RSSI squelch). DV overs are detected and decoded on the
    radio (`dvrx.c`) and played instead (red and green LEDs). The power knob is the analog volume
    for both; the knob's power switch works.
  - Serial (UART4, 230400): `T<secs>` / `F<secs>` timed DV / FM over, `A<v>` APC (0 = a few uW),
    `V`/`Q` FM level / squelch, `I` status, `C` factory power table, `!` resets to the bootloader,
    `X`/`W` BK4829 registers, `Pxn=0|1|z` pins, `R<secs>` raw mic or PA2, `D`/`B`/`S` DAC tests.
- `dvrx.c`: the receiver in C (the coherent path of `dv_live.py`), 19.2 kHz input; `dvrx_host`
  runs it on the PC. On the radio the decoded speech plays through the DAC while an NLMS filter
  cancels the DAC's leak into the receive audio (PA2 and the speaker share one node).
- `dvframe.py`: the air formats, reference encoder and decoder. `gen_tables.py` writes the
  firmware's tables (`dvtables.h`) from it.
  - V2 (current): 40 ms frames, 1 superframe, uniformly punctured.
  - V3 (walk 2 only): 80 ms frames, 4 sync + 92 data symbols, 2 superframes. K=7 rate-1/2 code:
    the 44 voicing / pitch / energy bits unpunctured, the 52 shape bits punctured to 84, each
    class in its own symbols so the receiver can trust one without the other (sotto decoder
    `good = 2` holds the last shape). `DVFMT=v3` selects it. It showed no benefit over V2.
- `coh.py`: coherent detection. The BK4829 tone generator is phase-continuous, so the signal
  is 4-level CPFSK with h = 0.5: a 2-state phase trellis with (previous tone, tone) templates,
  carrier phase carried across frames and held through fades. Raw SER 0.000 at S9+ against
  0.5-1% noncoherent; a little fewer lost frames at S2-S4 (FM threshold noise limits it there).
- `dv_live.py`: live receiver.
  - IC-9700 USB audio in, speech out on the default audio device. `--replay AIR.wav` runs the
    same receiver on a recording; `--noncoh` for the Goertzel + tone-pair trellis detector.
  - `--parrot` answers each DV over from the IC-9700 (FM-D, DATA MOD = USB, CI-V PTT) with the
    received superframes as a new DV over, for the HT to decode; `--parrot fm` replies with the
    decoded speech as FM. `--smeter` logs the S-meter. `dv_send.py` sends a DV over of a wav.
  - Symbol timing follows a slow loop (the radio's clock is steady); lock is held through
    about 0.5 s of bad frames, which the decoder conceals.
  - `--if`: decode from the IC-9700's IF output (USB AF/IF output set to IF, 12 kHz centre) with
    no FM demodulator: per-symbol correlation against IF templates (`ifT_init.npy`). About 5 dB
    below the discriminator's threshold on `ifbench.py` (10% FER at 41 vs 46 dB-Hz).
    `--raw IN.wav` records the input; `--replay IN.wav --disc` runs the AF receiver on a software
    discriminator of that IF recording, for an A/B on the same signal.
- `ifbench.py`: adds noise to a clean IF capture and compares the discriminator and IF detectors;
  writes `ifT_init.npy`.
- `dv_rx.py`: test receiver. It keys the radio over serial, then counts bit errors against the
  superframes the radio reports. Uses `../rig.py` for recording.
- `gen_eq.py` -> `eq_coef.h`: the mic EQ (FIR, applied on the radio).
- `smeter.py`, `walk_eval.py`: walk-test logging (IC-9700 S-meter over CI-V) and offline
  analysis by signal level. Walks 2026-11-09, rubber duck: at APC 0 (a few uW) clean at S5 and up,
  errors from S3-4, breaks up near S2; at ~2 W clean to 1.1 km (walk 3, DV vs FM pairs).

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
