/* Pre-encoder compressor + gentle gate on 1 ms (8-sample) blocks: the C port of voice_ab.py
   dynamics(gate_db=-46, gate_floor=-10, thr_db=-30, ratio=4, makeup_db=16, att_ms=3, rel_ms=120,
   hold_ms=120, close_ms=80) ("match_gate10" (hold 200, close 150) sped up, chosen by ear 2026-11-09; thresholds 6 dB under
   the voice_ab ones because the EQ runs 6 dB lower for headroom). Gain ramps linearly across
   each block. Floats: the M4F has an FPU; no libm needed. */
#define DY_B 8
#define DY_ATT 0.28347f            /* 1 - exp(-1/3) */
#define DY_REL 0.0083F             /* 1 - exp(-1/120) */
#define DY_A20 0.048771f           /* 1 - exp(-1/20): rms envelope for the gate */
#define DY_THR 0.031623f           /* -30 dBFS peak */
#define DY_MAKEUP 6.3096f          /* +16 dB */
#define DY_GATE_PW 2.5119e-5f      /* -46 dBFS rms, as power */
#define DY_FLOOR 0.31623f          /* -10 dB */
#define DY_HP 0.21009f             /* 1 - exp(-2 pi 300 / 8000) */
#define DY_HOLD 120                /* blocks (ms) */
#define DY_CLOSE (1.0f / 80)

struct dyn { float env, pw, g, gg, lp; int held; };
static void dyn_reset(struct dyn *d) { d->env = d->pw = d->lp = 0; d->g = d->gg = 1; d->held = 0; }

#if defined(__ARM_FP)                 /* vsqrt.f32: no libm / errno on the radio */
static inline float dy_sqrt(float x) { float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x)); return r; }
#else
static inline float dy_sqrt(float x) { return __builtin_sqrtf(x); }
#endif

static void dyn_block(struct dyn *d, int16_t *x)        /* one 8-sample block, in place */
{
    float p = 0, m = 0;
    for (int i = 0; i < DY_B; i++) {
        float v = x[i] * (1.0f / 32768); float a = v < 0 ? -v : v;
        if (a > p) p = a;
        d->lp += DY_HP * (v - d->lp); float h = v - d->lp;      /* gate detector: 300 Hz high-pass, */
        m += h * h;                                              /* the EQ's +24 dB bass is mostly noise */
    }
    d->pw += DY_A20 * (m * (1.0f / DY_B) - d->pw);
    d->env += (p > d->env ? DY_ATT : DY_REL) * (p - d->env);
    float tgt = DY_MAKEUP;                                   /* x (thr/env)^(3/4) above thr */
    if (d->env > DY_THR) { float r = DY_THR / d->env, s = dy_sqrt(r); tgt *= s * dy_sqrt(s); }
    d->held = d->pw > DY_GATE_PW ? DY_HOLD : (d->held ? d->held - 1 : 0);
    float g0 = d->g * d->gg;
    d->g += (tgt < d->g ? 0.5f : 0.05f) * (tgt - d->g);
    float gt = d->held ? 1.0f : DY_FLOOR;
    d->gg += (gt > d->gg ? 0.4f : DY_CLOSE) * (gt - d->gg);
    float g1 = d->g * d->gg;
    for (int i = 0; i < DY_B; i++) {
        float y = x[i] * (g0 + (g1 - g0) * (i + 1) * (1.0f / DY_B));
        x[i] = (int16_t)(y > 32767 ? 32767 : y < -32767 ? -32767 : y);
    }
}
