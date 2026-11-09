/* RT-950 Pro digital-voice transmitter, test image (v0).
 *
 * Mic (chip 0, no-RF capture, mic boost max) -> PA2 -> ADC at 8 kHz (TIM6) -> "match" EQ (FIR) -> compressor/gate (dyn.h)
 * -> sotto 1200 (fixed-point encoder) -> 1 superframe a frame (40 ms) -> K=7 r1/2 + puncture +
 * interleave + Gray (dvframe.py is the reference) -> 4-tone AFSK on chip 1's tone generator,
 * one REG_71 write per symbol at exactly 1200 Bd (TIM7) -> UHF path at minimum APC.
 *
 * UART4 (programming cable) at 230400:
 *   T<secs>   transmit for secs (preamble, frames, then the encoder's flush), then unkey
 * PTT (either side button, PE3 / PE2) does the same for as long as it is held (180 s limit).
 *   !         reset to the bootloader (from the interrupt, always)
 * The radio reports "S <frame> <superframe> <12 hex digits>" for every superframe it sends
 * and "STAT ..." / "DONE" lines, so the receiver can count bit errors exactly.
 * Turning the power knob off (PE0 high for 50 ms) unkeys and powers down (PB9 and PA11 low). SPDX-License-Identifier: MIT */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "at32f403a.h"
#include "cortex_m4.h"
#include "rt950_pinmap.h"
#include "drivers/gpio.h"
#include "drivers/adc.h"
#include "drivers/bk4829.h"
#include "sotto.h"
#include "eq_coef.h"
#include "dyn.h"
#include "drivers/spi.h"

extern void delay_ms(uint32_t ms);
extern volatile uint32_t bk4829_bb_loops;
extern size_t heap_used;
int sotto_lr_enc_pop(struct sotto_lr_enc *e, unsigned char out[SOTTO_BYTES_PER_FRAME]);   /* sotto_internal.h */                  /* shim's bump allocator: one encoder per over */
#define CYC (*(volatile uint32_t *)0xE0001004UL)

/* ------------------------------------------------------------------ frame format */
#define MIN_PRE_SYMS 240                   /* 200 ms of 0/3 filler at least before frame 0 */
#define FIRST_READY 198                    /* V2 measured 147 (40 ms audio + 70 ms encoder + start-up); + 48 for the 2nd superframe */
#define JITTER_SYMS 12                     /* 10 ms of slack: 40 ms was never touched */
#include "dvtables.h"                       /* V3: 80 ms frames, 2 superframes, unequal protection */
static const uint8_t gray[4] = {0, 1, 3, 2};          /* dibit (b0 b1) as 2 b0 + b1 -> tone */

/* ------------------------------------------------------------------ UART */
static volatile uint8_t rx_buf[64];
static volatile uint8_t rx_head, rx_tail;

static void uart_putc(uint8_t c) { while (!(UART4->SR & USART_SR_TXE)) ; UART4->DR = c; }
static void out_s(const char *s) { while (*s) uart_putc((uint8_t)*s++); }
static void out_u(uint32_t v)
{
    char b[11]; int i = 10; b[i] = 0;
    do { b[--i] = (char)('0' + v % 10); v /= 10; } while (v);
    out_s(b + i);
}
static void out_hex8(uint8_t v) { uart_putc((uint8_t)"0123456789abcdef"[v >> 4]); uart_putc((uint8_t)"0123456789abcdef"[v & 15]); }

void UART4_IRQHandler(void)
{
    uint8_t c = (uint8_t)UART4->DR;
    if (c == '!') {
        SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
        for (;;) ;
    }
    rx_buf[rx_head] = c;
    rx_head = (uint8_t)((rx_head + 1) & 63);
}

static int read_line(char *line, int max)
{
    static int n;
    while (rx_tail != rx_head) {
        char c = (char)rx_buf[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1) & 63);
        if (c == '\n' || c == '\r') {
            if (!n) continue;
            line[n] = 0; n = 0; return 1;
        }
        if (n < max - 1) line[n++] = c;
    }
    return 0;
}

static void pa_route(int on);
static void power_off(void);

/* Power knob: PE0 is grounded while the knob is on and pulled up when it is turned off (as
   src/app/power.c). 50 ms off: unkey, then release the PB9 latch, which cuts the supply. */
extern uint32_t get_tick(void);
static void housekeeping(void)
{
    static uint32_t off_since;
    *(volatile uint32_t *)0x40003000UL = 0xAAAA;                    /* IWDG */
#ifdef KNOB_PROBE
    return;
#endif
    if (!(GPIOE->IDR & GPIO_PIN_0)) { off_since = 0; return; }
    if (!off_since) { off_since = get_tick() | 1; return; }
    if (get_tick() - off_since < 50) return;
    power_off();
}

/* Unkey, release PB9 and pull PA11 low: on this unit PB9 alone doesn't cut the supply, PA11 low
   does (measured with the knob off). Should power stay up anyway, idle with RF off until the knob
   comes back on, then reset into the firmware. */
static void power_off(void)
{
    __asm volatile ("cpsid i");
    bk4829_bb_loops = 155;
    bk4829_write_reg(1, 0x70, 0); bk4829_write_reg(1, 0x30, 0); bk4829_write_reg(0, 0x30, 0);
    pa_route(0);
    gpio_clear_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);
    gpio_clear_pin(GPIOA, GPIO_PIN_11);         /* PA11 low is what actually cuts the supply (measured) */
    uint32_t on = 0;
    for (;;) {
        *(volatile uint32_t *)0x40003000UL = 0xAAAA;                /* IWDG */
        for (volatile uint32_t i = 0; i < 24000; i++) ;             /* ~0.5 ms, SysTick is masked */
        on = (GPIOE->IDR & GPIO_PIN_0) ? 0 : on + 1;
        if (on >= 100) {                                            /* knob on for ~50 ms */
            SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
            for (;;) ;
        }
    }
}

/* ------------------------------------------------------------------ BK4829 setup */
typedef struct { uint8_t chip, reg; uint16_t val; } bkw_t;
#define TX_START(c) {c,0x70,0}, {c,0x58,0}, {c,0x3F,0}, {c,0x59,0x0028}, {c,0x24,0x87DF}, {c,0x31,0}, \
                    {c,0x40,0x350A}, {c,0x37,0x9D1F}, {c,0x43,0x3028}, {c,0x51,0}, {c,0x52,0x0292}
#define BASE_TONE 15487                     /* 1500 Hz in REG_71 units (x 10.32444) */
#define STEP_TONE 3097                      /* 300 Hz */
static const bkw_t mic_setup[] = {          /* chip 0: mic capture "A" (mic boost max), no RF */
    TX_START(0), {0,0x40,0}, {0,0x47,0x6142}, {0,0x48,0xB092}, {0,0x7D,0xE91F}, {0,0x2C,0x003F},
    {0,0x2B,0}, {0,0x30,0}, {0,0x30,0x0206},
};
static const bkw_t tx_setup[] = {           /* chip 1: 432.950 MHz (same crystal correction as the 433.000 value), tones */
    TX_START(1), {1,0x38,0xA108}, {1,0x39,0x0294}, {1,0x30,0x0200}, {1,0x30,0xC3FE}, {1,0x47,0x6042},
    {1,0x7D,0xE900}, {1,0x2C,0}, {1,0x36,0x8E79}, {1,0x40,0x3720}, {1,0x71,BASE_TONE}, {1,0x70,0xE000},
};

static const bkw_t fm_setup[] = {           /* chip 1: FM voice at the same frequency and power (PTT2) */
    TX_START(1), {1,0x38,0xA108}, {1,0x39,0x0294}, {1,0x2C,0x0022}, {1,0x7D,0xE91A}, {1,0x48,0xB3FF},   /* mic gain 0x1A (chip default 0x12 was a bit low; 0x1F max) */
    {1,0x36,0x8E79}, {1,0x47,0x6656}, {1,0x30,0x0200}, {1,0x30,0xC1FE},
};

static void bk_apply(const bkw_t *w, int n) { for (int i = 0; i < n; i++) bk4829_write_reg(w[i].chip, w[i].reg, w[i].val); }

static void pa_route(int on)                /* nicFW's UHF TX route; APC (DAC2) stays at 0 */
{
    gpio_clear_pin(GPIOC, GPIO_PIN_4);  gpio_config_pin(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOB, GPIO_PIN_1);  gpio_config_pin(GPIOB, GPIO_PIN_1, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOE, GPIO_PIN_14); gpio_config_pin(GPIOE, GPIO_PIN_14, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    if (on) gpio_set_pin(GPIOE, GPIO_PIN_7); else gpio_clear_pin(GPIOE, GPIO_PIN_7);
    gpio_config_pin(GPIOE, GPIO_PIN_7, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
}

static bkw_t ovr[8]; static int n_ovr;
static uint16_t apc = 2200;                 /* A<v>: DAC2 (PA5, PA power control) while keyed. Factory table at
                                               433 MHz: low 1968 (1.2 W with REG_36 = 0x8E79), mid 2496 (4.1 W);
                                               2200 is about 2 W. 0 = a few uW */      /* X c,rr,vvvv: extra writes after tx_setup (test knob) */

/* ------------------------------------------------------------------ capture (TIM6, 8 kHz) */
#define SRING 2048
static int16_t sring[SRING];
static volatile uint32_t s_head, s_tail;
static volatile uint32_t s_overrun;
static int16_t eq_hist[2 * EQ_TAPS];
static uint32_t eq_pos;
static int32_t dc_q8 = 1220 << 8;
static volatile int raw_cap;             /* R command: ring gets the pre-EQ samples */

static uint16_t adc_fast(void)
{
    ADC2->SQR1 = 0; ADC2->SQR3 = 2; ADC2->SR = 0;
    ADC2->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC2->SR & ADC_SR_EOC)) ;
    return (uint16_t)(ADC2->DR & 0x0FFF);
}

void TIM6_IRQHandler(void)
{
    TIM6->SR = 0;
    int32_t a = (int32_t)adc_fast() << 8;
    dc_q8 += (a - dc_q8) >> 12;
    int32_t x = (a - dc_q8) >> 4;                               /* 12-bit counts x 16 */
    if (x > 32767) x = 32767; else if (x < -32767) x = -32767;
    /* FIR on a doubled history buffer: the window is always contiguous */
    eq_hist[eq_pos] = eq_hist[eq_pos + EQ_TAPS] = (int16_t)x;
    const int16_t *h = &eq_hist[eq_pos + 1];                    /* oldest .. newest */
    int64_t acc = 0;
    for (int k = 0; k < EQ_TAPS; k++) acc += (int32_t)h[k] * eq_coef[EQ_TAPS - 1 - k];
    eq_pos = eq_pos + 1 == EQ_TAPS ? 0 : eq_pos + 1;
    int32_t y = raw_cap ? x : (int32_t)(acc >> EQ_SHIFT);
    if (y > 32767) y = 32767; else if (y < -32767) y = -32767;
    if (s_head - s_tail >= SRING) { s_overrun++; return; }
    sring[s_head % SRING] = (int16_t)y;
    s_head++;
}

static void capture_start(int on)
{
    TIM6->CR1 = 0; NVIC_DisableIRQ(TIM6_IRQn);
    if (!on) return;
    CRM->APB1EN |= CRM_APB1EN_TIM6EN;
    TIM6->PSC = 29;                                             /* 240 MHz -> 8 MHz */
    TIM6->ARR = 999;                                            /* -> 8 kHz */
    TIM6->EGR = 1; TIM6->SR = 0; TIM6->DIER = 1;
    NVIC_SetPriority(TIM6_IRQn, 1); NVIC_EnableIRQ(TIM6_IRQn);
    TIM6->CR1 = 1;
}

/* ------------------------------------------------------------------ symbols (TIM7, 1200 Bd) */
#define YRING 4096
static uint8_t yring[YRING];
static volatile uint32_t y_head, y_tail, y_underrun;
static volatile uint8_t y_last;

void TIM7_IRQHandler(void)
{
    static const int8_t lv[4] = {-3, -1, 1, 3};
    TIM7->SR = 0;
    uint8_t t;
    if (y_tail != y_head) { t = yring[y_tail % YRING]; y_tail++; }
    else { y_underrun++; t = (uint8_t)(y_last == 0 ? 3 : 0); }  /* keep alternating outer tones */
    y_last = t;
    bk4829_write_reg(1, 0x71, (uint16_t)(BASE_TONE + lv[t] * STEP_TONE));
}

static void symbols_start(int on)
{
    TIM7->CR1 = 0; NVIC_DisableIRQ(TIM7_IRQn);
    if (!on) return;
    CRM->APB1EN |= CRM_APB1EN_TIM7EN;
    TIM7->PSC = 199; TIM7->ARR = 999;                           /* 240 MHz / 200 / 1000 = 1200 Hz */
    TIM7->EGR = 1; TIM7->SR = 0; TIM7->DIER = 1;
    NVIC_SetPriority(TIM7_IRQn, 0); NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 = 1;
}

static void push_sym(uint8_t t) { yring[y_head % YRING] = t; y_head++; }

/* ------------------------------------------------------------------ FEC + framing */
static int parity7(uint32_t x) { x ^= x >> 4; x ^= x >> 2; x ^= x >> 1; return (int)(x & 1); }
static void fec_init(void) { }
static void push_frame(const uint8_t payload[INFO / 8])
{
    static uint8_t coded[CODED], il[KEPT];
    uint32_t s = 0; int n = 0;
    for (int i = 0; i < INFO + 6; i++) {
        int b = 0;
        if (i < INFO) { int p = fmt_order[i]; b = (payload[p >> 3] >> (7 - (p & 7))) & 1; }
        s = ((s << 1) | (uint32_t)b) & 0x7F;
        coded[n++] = (uint8_t)parity7(s & 0133);
        coded[n++] = (uint8_t)parity7(s & 0171);
    }
    for (int k = 0; k < KEPT; k++) il[fmt_ilv[k]] = coded[fmt_kept[k]];
    for (int i = 0; i < SYNC_LEN; i++) push_sym((uint8_t)(sync_bits[i] * 3));
    for (int i = 0; i < DATA_SYMS; i++) push_sym(gray[2 * il[2 * i] + il[2 * i + 1]]);
}

/* ------------------------------------------------------------------ session */
static void report_sf(uint32_t frame, int k, const uint8_t *b)
{
    out_s("S "); out_u(frame); out_s(" "); out_u((uint32_t)k); out_s(" ");
    for (int i = 0; i < SOTTO_BYTES_PER_FRAME; i++) out_hex8(b[i]);
    out_s(" "); out_u(s_tail);                 /* samples fed to the encoder when it came out */
    out_s("\r\n");
}

static int ptt_down(void) { return (GPIOE->IDR & (GPIO_PIN_3 | GPIO_PIN_2)) != (GPIO_PIN_3 | GPIO_PIN_2); }
static int ptt1_down(void) { return !(GPIOE->IDR & GPIO_PIN_3); }
static int ptt2_down(void) { return !(GPIOE->IDR & GPIO_PIN_2); }
#define LED_ON 1                            /* PC13 red, PC14 green (rt950_pinmap.h) */
static void led(int red, int green)
{
    if (red == LED_ON) gpio_set_pin(GPIOC, GPIO_PIN_13); else gpio_clear_pin(GPIOC, GPIO_PIN_13);
    if (green == LED_ON) gpio_set_pin(GPIOC, GPIO_PIN_14); else gpio_clear_pin(GPIOC, GPIO_PIN_14);
}

#define PTT_RELEASE 160                     /* samples (20 ms) up before the over ends: debounce */
#define PTT_TOT (180 * 8000)                /* time-out timer */

/* secs > 0: a timed over (serial T command); secs = 0: until both PTT buttons are released */
static void transmit(uint32_t secs)
{
    static int16_t pcm[SOTTO_LR_SAMPLES_PER_FRAME];
    static uint8_t payload[INFO / 8];
    static struct dyn dy;
    uint8_t sf[SOTTO_BYTES_PER_FRAME];
    heap_used = 0; dyn_reset(&dy);
    struct sotto_lr_enc *e = sotto_lr_enc_create();
    if (!e) { out_s("STAT alloc failed\r\nDONE\r\n"); return; }
    uint32_t frames = 0, nsf = 0, enc_max = 0, nenc = 0, fill_max = 0, pre = 0, fill_min = ~0u;
    uint64_t enc_sum = 0;

    /* key up: route + chip 1 TX + tone generator; chip 0 already capturing */
    s_head = s_tail = s_overrun = 0; dc_q8 = 1220 << 8; y_head = y_tail = y_underrun = 0; y_last = 3;
    bk4829_bb_loops = 155;                  /* setup at the proven read speed; 4 lost writes */
    led(1, 0); pa_route(1); DAC->DHR12R2 = apc;
    bk_apply(tx_setup, (int)(sizeof tx_setup / sizeof tx_setup[0]));
    bk_apply(ovr, n_ovr);
    bk4829_bb_loops = 30;                   /* symbol writes: ~20 us each */
    symbols_start(1);
    /* the preamble's minimum length must not become queueing delay: send filler first and
       start the mic only when the first frame (ready ~FIRST_READY symbols after capture
       starts) will land at the end of the minimum preamble */
    while (y_underrun + FIRST_READY < MIN_PRE_SYMS) housekeeping();
    capture_start(1);

    uint32_t t_end = secs * 8000, taken = 0, last_down = 0;
    int flushing = 0;
    for (;;) {
        housekeeping();
        int got = 0;
        if (!flushing && sotto_lr_enc_pop(e, sf)) got = 1;      /* a burst: one per pass */
        else if (!flushing && s_head - s_tail >= 80) {            /* 10 ms at a time: no 40 ms wait */
            for (int i = 0; i < 80; i++) pcm[i] = sring[(s_tail + i) % SRING];
            s_tail += 80; taken += 80;
            uint32_t t0 = CYC;
            for (int i = 0; i < 80; i += DY_B) dyn_block(&dy, pcm + i);   /* compressor + gate */
            sotto_lr_encode_part(e, pcm, 1);
            got = sotto_lr_enc_pop(e, sf);
            uint32_t dt = CYC - t0; enc_sum += dt; nenc++; if (dt > enc_max) enc_max = dt;
            if (ptt_down()) last_down = taken;
            if (secs ? taken >= t_end : taken - last_down >= PTT_RELEASE || taken >= PTT_TOT) { flushing = 1; capture_start(0); }
        } else if (flushing) {
            got = sotto_lr_enc_flush(e, sf);
            if (!got) {
                if (nsf % SF_PER_FRAME == 0) break;
                memset(sf, 0, sizeof sf); got = 1;                 /* pad the last frame with silence */
            }
        }
        if (got) {
            memcpy(payload + (nsf % SF_PER_FRAME) * SOTTO_BYTES_PER_FRAME, sf, SOTTO_BYTES_PER_FRAME);
            report_sf(frames, (int)(nsf % SF_PER_FRAME), sf);
            if (++nsf % SF_PER_FRAME == 0) {
                if (!frames) {                          /* preamble = filler so far; add the slack */
                    uint32_t sent = y_underrun, pad = JITTER_SYMS;   /* filler symbols so far */
                    pre = sent;
                    y_underrun = 0;                                 /* count only real dry spells */
                    if (sent + pad < MIN_PRE_SYMS) pad = MIN_PRE_SYMS - sent;
                    for (uint32_t i = 0; i < pad; i++) push_sym((uint8_t)((i & 1) ? y_last : 3 - y_last));
                }
                if (frames) { uint32_t f = y_head - y_tail; if (f < fill_min) fill_min = f; }
                push_frame(payload); frames++;
            }
        }
        uint32_t fill = y_head - y_tail; if (fill > fill_max) fill_max = fill;
    }
    for (int i = 0; i < 32; i++) push_sym((uint8_t)(i & 1 ? 3 : 0));   /* tail */
    while (y_tail != y_head) housekeeping();
    symbols_start(0);
    /* unkey */
    bk4829_bb_loops = 155;
    bk4829_write_reg(1, 0x70, 0); bk4829_write_reg(1, 0x30, 0);
    pa_route(0); DAC->DHR12R2 = 0; led(0, 0);
    sotto_lr_enc_destroy(e);
    out_s("STAT frames="); out_u(frames); out_s(" superframes="); out_u(nsf);
    out_s(" enc10ms_mean_cyc="); out_u(nenc ? (uint32_t)(enc_sum / nenc) : 0); out_s(" enc_max_cyc="); out_u(enc_max);
    out_s(" sym_underruns="); out_u(y_underrun); out_s(" sym_fill_max="); out_u(fill_max);
    out_s(" pcm_overruns="); out_u(s_overrun); out_s(" pre_filler="); out_u(pre); out_s(" fill_min_before_push="); out_u(fill_min); out_s("\r\nDONE\r\n");
}

/* PTT2: plain FM voice (stock TX path, mic at the chip's defaults) at the same power, until released */
static void transmit_fm(void)
{
    led(0, 1); bk4829_bb_loops = 155;
    pa_route(1); DAC->DHR12R2 = apc;
    bk_apply(fm_setup, (int)(sizeof fm_setup / sizeof fm_setup[0]));
    uint32_t up = 0, t0 = get_tick();
    while (up < 20 && get_tick() - t0 < 180000) {     /* 20 ms release debounce, 180 s timeout */
        housekeeping(); delay_ms(1);
        up = ptt2_down() ? 0 : up + 1;
    }
    bk4829_write_reg(1, 0x30, 0); bk4829_write_reg(1, 0x36, 0);
    pa_route(0); DAC->DHR12R2 = 0; led(0, 0);
    out_s("FM over\r\n");
}

int main(void)
{
    gpio_enable_clock(GPIOA); gpio_enable_clock(GPIOB); gpio_enable_clock(GPIOC);
    gpio_enable_clock(GPIOD); gpio_enable_clock(GPIOE);
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);
    gpio_set_pin(GPIOA, GPIO_PIN_11); gpio_config_pin(GPIOA, GPIO_PIN_11, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);  /* power hold */
    gpio_config_pin(GPIOE, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_CNF_PULL); gpio_set_pin(GPIOE, GPIO_PIN_0);  /* knob */
    for (volatile uint32_t i = 0; i < 2400; i++) ;                 /* let the pull-up settle */
#ifndef KNOB_PROBE
    if (GPIOE->IDR & GPIO_PIN_0) { bk4829_bb_loops = 155; power_off(); }   /* booted with the knob off */
#endif

    CRM->APB1EN |= CRM_APB1EN_UART4EN;
    gpio_config_pin(CPS_TX_PORT, CPS_TX_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_AF_PP);
    gpio_config_pin(CPS_RX_PORT, CPS_RX_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);
    UART4->BRR = 521;                                           /* 120 MHz / 230400 */
    UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_SetPriority(UART4_IRQn, 2); NVIC_EnableIRQ(UART4_IRQn);

    *(volatile uint32_t *)0xE000EDFCUL |= (1UL << 24);         /* DWT */
    *(volatile uint32_t *)0xE0001000UL |= 1UL;

    /* speaker off, internal mic; BK4829 bus idle, both chips initialised */
    gpio_set_pin(GPIOE, GPIO_PIN_1); gpio_config_pin(GPIOE, GPIO_PIN_1, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOE, GPIO_PIN_4); gpio_config_pin(GPIOE, GPIO_PIN_4, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOB, GPIO_PIN_8); gpio_config_pin(GPIOB, GPIO_PIN_8, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_config_pin(BK4829_SEN1_PORT, BK4829_SEN1_PIN, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_set_pin(BK4829_SEN1_PORT, BK4829_SEN1_PIN);
    gpio_config_pin(BK4829_SEN2_PORT, BK4829_SEN2_PIN, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_set_pin(BK4829_SEN2_PORT, BK4829_SEN2_PIN);
    gpio_config_pin(BK4829_SCK_PORT, BK4829_SCK_PIN, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_clear_pin(BK4829_SCK_PORT, BK4829_SCK_PIN);
    gpio_config_pin(BK4829_SDA_PORT, BK4829_SDA_PIN, GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    bk4829_bb_loops = 155;
    bk4829_init(0); bk4829_init(1);
    bk4829_write_reg(0, 0x30, 0); bk4829_write_reg(1, 0x30, 0);
    pa_route(0);

    adc_init(); spi2_init();
    CRM->APB1EN |= CRM_APB1EN_DACEN;                            /* as the test firmware: */
    gpio_config_pin(GPIOA, GPIO_PIN_5, GPIO_MODE_INPUT, GPIO_CNF_ANALOG);
    DAC->CR |= (1UL << 0) | (1UL << 1) | (1UL << 16);           /* DAC1 (beep, on the mic node) */
    DAC->DHR12R1 = 2048; DAC->DHR12R2 = 0;                      /* mid-scale; DAC2 = APC = 0 */
    ADC2->SMPR2 = (ADC2->SMPR2 & ~(7UL << 6)) | (7UL << 6);    /* channel 2: 239.5 cycles */
    fec_init();
    bk_apply(mic_setup, (int)(sizeof mic_setup / sizeof mic_setup[0]));

    out_s("\r\ndvtx ready\r\n");
    char line[20];
    gpio_config_pin(GPIOE, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_CNF_PULL); gpio_set_pin(GPIOE, GPIO_PIN_3);  /* PTT1 */
    gpio_config_pin(GPIOE, GPIO_PIN_2, GPIO_MODE_INPUT, GPIO_CNF_PULL); gpio_set_pin(GPIOE, GPIO_PIN_2);  /* PTT2 */
    led(0, 0); gpio_config_pin(GPIOC, GPIO_PIN_13, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP); gpio_config_pin(GPIOC, GPIO_PIN_14, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    uint32_t down_ms = 0;
    for (;;) {
        housekeeping();
#ifdef KNOB_PROBE
        {   /* PE0 read three ways, 5 times a second */
            static uint32_t last;
            if (get_tick() - last >= 200) {
                last = get_tick();
                int r[3];
                for (int m = 0; m < 3; m++) {
                    if (m == 0) gpio_config_pin(GPIOE, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);
                    else { gpio_config_pin(GPIOE, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_CNF_PULL);
                           if (m == 1) gpio_set_pin(GPIOE, GPIO_PIN_0); else gpio_clear_pin(GPIOE, GPIO_PIN_0); }
                    delay_ms(2);
                    r[m] = (GPIOE->IDR & GPIO_PIN_0) ? 1 : 0;
                }
                out_s("PE0 float="); out_u((uint32_t)r[0]); out_s(" up="); out_u((uint32_t)r[1]);
                out_s(" down="); out_u((uint32_t)r[2]); out_s("\r\n");
            }
        }
#endif
        if (ptt_down()) {                   /* 20 ms held: PTT1 = DV, PTT2 = FM voice */
            delay_ms(1);
            if (++down_ms >= 20) { if (ptt1_down()) transmit(0); else if (ptt2_down()) transmit_fm(); down_ms = 0; }
            continue;
        }
        down_ms = 0;
        if (!read_line(line, sizeof line)) continue;
        if ((line[0] == 'X' || line[0] == 'W') && line[1] && line[2] == ',' && line[5] == ',') {
            /* X1,40,3800 adds an override applied at every key-up (X alone clears); W writes now */
            uint32_t v[3] = {0, 0, 0}; int k = 0;
            for (const char *q = line + 1; *q && k < 3; q++) {
                if (*q == ',') { k++; continue; }
                v[k] = v[k] * 16 + (uint32_t)(*q <= '9' ? *q - '0' : (*q | 32) - 'a' + 10);
            }
            bkw_t w = {(uint8_t)v[0], (uint8_t)v[1], (uint16_t)v[2]};
            bk4829_bb_loops = 155;
            if (line[0] == 'W') bk4829_write_reg(w.chip, w.reg, w.val);
            else if (n_ovr < 8) ovr[n_ovr++] = w;
            out_s("ok\r\n");
        } else if (line[0] == 'X') { n_ovr = 0; out_s("ok\r\n"); }
        else if (line[0] == 'P' && line[1] >= 'A' && line[1] <= 'E' && line[2]) {
            /* Pxn=0|1|z: drive pin n of port x, or float it (test knob) */
            GPIO_TypeDef *pt[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};
            GPIO_TypeDef *p = pt[line[1] - 'A'];
            int n = 0; const char *q = line + 2;
            while (*q >= '0' && *q <= '9') n = n * 10 + (*q++ - '0');
            if (*q == '=' && n < 16) {
                uint16_t pin = (uint16_t)(1u << n);
                if (q[1] == 'z') gpio_config_pin(p, pin, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);
                else {
                    if (q[1] == '1') gpio_set_pin(p, pin); else gpio_clear_pin(p, pin);
                    gpio_config_pin(p, pin, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
                }
                out_s("ok\r\n");
            }
        }
        else if (line[0] == 'R') {
            /* R<secs>: no TX; stream the mic (DC removed, x16, before the EQ) as raw s16le
               8 kHz: "RAW n" line, n samples, then DONE. 16 kB/s fits the 23 kB/s UART. */
            uint32_t secs = 0;
            for (const char *q = line + 1; *q >= '0' && *q <= '9'; q++) secs = secs * 10 + (uint32_t)(*q - '0');
            if (secs < 1) secs = 1;
            if (secs > 60) secs = 60;
            uint32_t n = secs * 8000;
            out_s("RAW "); out_u(n); out_s("\r\n");
            s_head = s_tail = s_overrun = 0; dc_q8 = 1220 << 8; raw_cap = 1;
            capture_start(1);
            for (uint32_t i = 0; i < n; ) {
                housekeeping();
                if (s_head == s_tail) continue;
                uint16_t v = (uint16_t)sring[s_tail % SRING]; s_tail++; i++;
                uart_putc((uint8_t)v); uart_putc((uint8_t)(v >> 8));
            }
            capture_start(0); raw_cap = 0;
            out_s("STAT pcm_overruns="); out_u(s_overrun); out_s("\r\nDONE\r\n");
        }
        else if (line[0] == 'A') {                 /* A<v>: APC DAC value for the next overs */
            uint32_t v = 0;
            for (const char *q = line + 1; *q >= '0' && *q <= '9'; q++) v = v * 10 + (uint32_t)(*q - '0');
            apc = (uint16_t)(v > 4095 ? 4095 : v); out_s("ok\r\n");
        }
        else if (line[0] == 'C') {                 /* C: factory TX power calibration, 12 bands x (low, mid, high) */
            uint8_t cal[192]; spi_flash_read(0x00F000, cal, 192);
            for (int b = 0; b < 12; b++) {
                out_s("CAL "); out_u((uint32_t)b);
                for (int k = 0; k < 8; k++) { out_s(" "); out_u((uint32_t)cal[b * 16 + 2 * k] | ((uint32_t)cal[b * 16 + 2 * k + 1] << 8)); }
                out_s("\r\n");
            }
            uint8_t e[32]; spi_flash_read(0x00F230, e, 32);
            out_s("EDGES"); for (int i = 0; i < 32; i++) { out_s(" "); out_u(e[i]); } out_s("\r\n");
        }
        else if (line[0] == 'T') {
            uint32_t secs = 0;
            for (const char *q = line + 1; *q >= '0' && *q <= '9'; q++) secs = secs * 10 + (uint32_t)(*q - '0');
            if (secs < 1) secs = 1;
            if (secs > 60) secs = 60;
            transmit(secs);
        }
    }
}
