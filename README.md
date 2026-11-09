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

## Todo

- Audio quality is still not satisfactory. Next:
  - more EQ work, possibly a high-cut; it can go into the existing FIR at no extra cost
  - a noise gate and a compressor ahead of the encoder
- CPU on the radio leaves room:
  - encoder about 1.0 M cycles per 10 ms on average (about 100 of 240 MHz), worst call 2.9 M
  - the 255-tap EQ runs in the 8 kHz sample interrupt
