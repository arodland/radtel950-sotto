# RT-950 Pro DV transmitter test image (240 MHz). make [SOTTO=..] [RT=..]
SOTTO ?= /home/andrew/code/radtel950/sotto
RT ?= /home/andrew/software/Radtel-RT950-Pro-Firmware
CC = arm-none-eabi-gcc
ARCH = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
SOTTO_DEFS ?= -DFX_LAG=4 -DLR_AHEAD=0       # low-delay encoder (sotto docs/sotto-1200.md)
CFLAGS = $(ARCH) -O2 $(SOTTO_DEFS) -ffunction-sections -fdata-sections -std=c11 -DHCLK_240 -DSOTTO_V36 -DSOTTO_PITCH_THIRD \
         -Wno-error=unterminated-string-initialization -I$(RT)/include -I$(SOTTO)/c -isystem $(SOTTO)/c/fixed/cm4/inc
SRCS = main.c $(addprefix $(RT)/src/,startup.c system.c debug_uart.c drivers/gpio.c drivers/adc.c drivers/bk4829.c)
SOTTO_SRCS = $(addprefix $(SOTTO)/c/fixed/,fx_math.c fx_tables.c sotto_fx_enc.c sotto_fx_dec.c sotto_fx_lr_enc.c sotto_fx_lr.c)
SOTTO_OBJS = $(notdir $(SOTTO_SRCS:.c=.o))

dvtx.BTF: dvtx.bin
	python3 -I $(RT)/tools/encrypt_btf.py $< $@ --key 00000000000000000000000000000000
dvtx.bin: dvtx.elf
	arm-none-eabi-objcopy -O binary $< $@
%.o: $(SOTTO)/c/fixed/%.c         # clang -O2 was fastest for the encoder (88 vs 92 MHz)
	clang --target=thumbv7em-none-eabihf -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(filter-out -O2 -Wno-error%,$(CFLAGS)) -O2 -c -o $@ $<
shim.o: shim.c                    # must not turn back into calls to itself (memset -> memset)
	$(CC) $(CFLAGS) -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns -c -o $@ $<
dvtx.elf: $(SRCS) eq_coef.h shim.o $(SOTTO_OBJS)
	$(CC) $(CFLAGS) -T$(RT)/linker.ld -Wl,--gc-sections -nostdlib -o $@ $(SRCS) shim.o $(SOTTO_OBJS) -lgcc
	arm-none-eabi-size $@
clean:
	rm -f dvtx.elf dvtx.bin dvtx.BTF *.o libsottodec.so sotto_dec_fx_d1

# the receiver's decoder (x86 shared library for dv_live.py): fixed sotto 1200, LR_INTERP=0 so it
# plays superframe s as soon as s + 1 is in (1 superframe of delay; frames are lost whole anyway)
FX_DEC = $(addprefix $(SOTTO)/c/fixed/,sotto_fx_lr.c sotto_fx_dec.c fx_math.c fx_tables.c)
DEC_DEFS = -DSOTTO_V36 -DSOTTO_PITCH_THIRD -DLR_INTERP=0 -DSOTTO_LR_DEC_DELAY=1
libsottodec.so: $(FX_DEC)
	cc -O2 -shared -fPIC -std=c99 $(DEC_DEFS) -I$(SOTTO)/c -o $@ $(FX_DEC)
# the same decoder as a file-to-file tool, for dv_rx.py
sotto_dec_fx_d1: $(FX_DEC) $(SOTTO)/c/sotto_dec.c
	cc -O2 -std=c99 $(DEC_DEFS) -I$(SOTTO)/c -o $@ $^
receiver: libsottodec.so sotto_dec_fx_d1
