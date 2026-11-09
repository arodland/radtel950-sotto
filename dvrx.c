/* dvrx: dvtx V2 receiver (see dvrx.h). Port of dv_live.py / coh.py:
   - search: sync score from sliding tone magnitudes (tone 3 - tone 0 over the 4 sync symbols),
     candidate at > 0.4, the next two syncs > 0.2, then the first two frames must decode coherently
     with re-encoded symbol error rate < 0.1
   - per frame: analytic signal (31-tap Hilbert), correlations with (previous tone, tone) templates,
     forward-backward max-sum over the 8-state (previous tone, phase 0/pi) CPFSK trellis for the best
     of several carrier-phase hypotheses, soft bits, K=7 Viterbi, re-encode, re-estimate the phase,
     detect again; good when the re-encoded symbol error rate < 0.15
   - templates and carrier phase carried from frame to frame; slow timing loop from the coherent
     matched-filter peak; the over ends after DVRX_HOLD bad frames in a row
   SPDX-License-Identifier: MIT */
#include <string.h>
#include "dvrx.h"
#include "dvrx_tables.h"
#include "dvtables.h"

#define SPS DVRX_SPS
#define FRAME (FRAME_SYMS * SPS)
#define MASK (DVRX_RING - 1)
#define HD ((DVRX_HT - 1) / 2)
#define NEG (-1e30f)
#define BAD_SER 0.15f
#define LOCK_SCORE 0.4f
#define ZN (FRAME + 2)                       /* analytic samples per frame, +-1 for timing */

static const uint8_t ADV[4] = {1, 0, 1, 0};  /* phase advance of tone k, units of pi, mod 2 */
static const uint8_t SYNC_T[SYNC_LEN] = {3, 0, 0, 3};

typedef struct { float re, im; } cpx;

struct dvrx {
    int16_t x[DVRX_RING];
    float diff[DVRX_RING];
    uint32_t n, nd, searched;
    int locked, pending_end, relevel; uint32_t relevel_at;
    uint32_t t; float frac, drift;           /* frame start = t + frac (samples) */
    cpx T[4][4][SPS]; int tset, ngood;
    cpx psi; int psi_set, psi_amb, last_sym;
    int nfr, nbad, bad_run; float ser_sum, ser_tail[DVRX_HOLD];
    int st_frames, st_bad, st_ser;
    cpx z[ZN];                               /* z[k] = analytic sample at frame start - 1 + k */
    cpx C[FRAME_SYMS][4][4];
    float al[FRAME_SYMS + 1][4][2], met[FRAME_SYMS][4];
};

unsigned dvrx_size(void) { return sizeof(struct dvrx); }

static float fsqrt(float v)
{
#if defined(__ARM_FP)
    float r; __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(v)); return r;
#else
    return __builtin_sqrtf(v);
#endif
}

static void state_reset(struct dvrx *r)
{
    r->tset = 0; r->ngood = 0; r->psi_set = 0; r->psi_amb = 1; r->last_sym = -1;
}

void dvrx_init(struct dvrx *r)
{
    memset(r, 0, sizeof *r);
    state_reset(r);
}

void dvrx_push(struct dvrx *r, int16_t x) { r->x[r->n & MASK] = x; r->n++; }
int dvrx_locked(const struct dvrx *r) { return r->locked; }
uint32_t dvrx_count(const struct dvrx *r) { return r->n; }
void dvrx_relevel_at(struct dvrx *r, uint32_t n) { r->relevel = 1; r->relevel_at = n; }
void dvrx_stats(const struct dvrx *r, int *f, int *b, int *s) { *f = r->st_frames; *b = r->st_bad; *s = r->st_ser; }

/* ------------------------------------------------------------------ search */
static void diff_at(struct dvrx *r, uint32_t t)          /* window x[t .. t + SPS - 1] */
{
    float P[4], mx = 0;
    for (int c = 0; c < 4; c++) {
        float re = 0, im = 0;
        for (int m = 0; m < SPS; m++) {
            float v = r->x[(t + m) & MASK];
            re += v * dvrx_tone_re[c][m]; im -= v * dvrx_tone_im[c][m];
        }
        P[c] = fsqrt(re * re + im * im); if (P[c] > mx) mx = P[c];
    }
    r->diff[t & MASK] = (P[3] - P[0]) / (mx + 1e-9f);
}

static float score(const struct dvrx *r, uint32_t u)
{
    float s = 0;
    for (int k = 0; k < SYNC_LEN; k++) {
        float d = r->diff[(u + k * SPS) & MASK];
        s += SYNC_T[k] ? d : -d;
    }
    return s / SYNC_LEN;
}

/* ------------------------------------------------------------------ FEC */
static int parity(uint32_t v) { v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return (int)(v & 1); }

static void frame_symbols(const uint8_t payload[INFO], uint8_t syms[FRAME_SYMS])
{
    uint8_t coded[CODED], il[KEPT]; uint32_t s = 0; int n = 0;
    static const uint8_t gray[4] = {0, 1, 3, 2};             /* dibit b0 b1 -> tone */
    for (int i = 0; i < INFO + 6; i++) {
        int b = i < INFO ? payload[fmt_order[i]] : 0;
        s = ((s << 1) | (uint32_t)b) & 0x7F;
        coded[n++] = (uint8_t)parity(s & 0133); coded[n++] = (uint8_t)parity(s & 0171);
    }
    for (int k = 0; k < KEPT; k++) il[fmt_ilv[k]] = coded[fmt_kept[k]];
    for (int i = 0; i < SYNC_LEN; i++) syms[i] = SYNC_T[i];
    for (int i = 0; i < DATA_SYMS; i++) syms[SYNC_LEN + i] = gray[2 * il[2 * i] + il[2 * i + 1]];
}

static void viterbi(const float soft[CODED], uint8_t payload[INFO])
{
    static uint8_t dec[INFO + 6][64];
    float pm[64], nm[64];
    for (int s = 0; s < 64; s++) pm[s] = NEG;
    pm[0] = 0;
    for (int t = 0; t < INFO + 6; t++) {
        float x = soft[2 * t], y = soft[2 * t + 1];
        for (int s = 0; s < 64; s++) {
            int b = s & 1, p0 = s >> 1, p1 = p0 | 32;
            uint32_t r0 = (uint32_t)((p0 << 1) | b), r1 = (uint32_t)((p1 << 1) | b);
            float m0 = pm[p0] + (parity(r0 & 0133) ? x : -x) + (parity(r0 & 0171) ? y : -y);
            float m1 = pm[p1] + (parity(r1 & 0133) ? x : -x) + (parity(r1 & 0171) ? y : -y);
            if (m1 > m0) { nm[s] = m1; dec[t][s] = (uint8_t)p1; } else { nm[s] = m0; dec[t][s] = (uint8_t)p0; }
        }
        memcpy(pm, nm, sizeof pm);
    }
    uint8_t info[INFO + 6]; int s = 0;
    for (int t = INFO + 5; t >= 0; t--) { info[t] = (uint8_t)(s & 1); s = dec[t][s]; }
    for (int i = 0; i < INFO; i++) payload[fmt_order[i]] = info[i];
}

static void decode_bits(const float met[FRAME_SYMS][4], uint8_t payload[INFO])
{
    float mean = 0, var = 0, soft[CODED], sb[2 * DATA_SYMS];
    for (int i = 0; i < FRAME_SYMS; i++) for (int c = 0; c < 4; c++) mean += met[i][c];
    mean /= FRAME_SYMS * 4;
    for (int i = 0; i < FRAME_SYMS; i++) for (int c = 0; c < 4; c++) { float d = met[i][c] - mean; var += d * d; }
    float k = 1.0f / (fsqrt(var / (FRAME_SYMS * 4)) + 1e-9f);
    for (int i = 0; i < DATA_SYMS; i++) {
        const float *m = met[SYNC_LEN + i];
        float a = m[2] > m[3] ? m[2] : m[3], b = m[0] > m[1] ? m[0] : m[1];       /* bit 0: tones 2, 3 */
        float c = m[1] > m[2] ? m[1] : m[2], d = m[0] > m[3] ? m[0] : m[3];       /* bit 1: tones 1, 2 */
        sb[2 * i] = (a - b) * k; sb[2 * i + 1] = (c - d) * k;
    }
    memset(soft, 0, sizeof soft);
    for (int j = 0; j < KEPT; j++) soft[fmt_kept[j]] = sb[fmt_ilv[j]];
    viterbi(soft, payload);
}

/* ------------------------------------------------------------------ coherent detection */
static void analytic(struct dvrx *r, uint32_t t0)      /* z for samples t0 - 1 .. t0 + FRAME */
{
    for (int k = 0; k < ZN; k++) {
        uint32_t s = t0 - 1 + (uint32_t)k; float im = 0;
        for (int j = 0; j < DVRX_HT; j++) im += dvrx_hilb[j] * r->x[(s + HD - (uint32_t)j) & MASK];
        r->z[k].re = r->x[s & MASK]; r->z[k].im = im;
    }
}

static void correlate(struct dvrx *r, int off)         /* off: 0 / 1 / 2 = shift -1 / 0 / +1 */
{
    for (int i = 0; i < FRAME_SYMS; i++) {
        const cpx *z = &r->z[off + i * SPS];
        for (int p = 0; p < 4; p++) for (int c = 0; c < 4; c++) {
            const cpx *T = r->T[p][c]; float re = 0, im = 0;
            for (int j = 0; j < SPS; j++) { re += z[j].re * T[j].re + z[j].im * T[j].im; im += z[j].im * T[j].re - z[j].re * T[j].im; }
            r->C[i][p][c].re = re; r->C[i][p][c].im = im;
        }
    }
}

static void init_templates(struct dvrx *r)             /* ideal tones at the signal's level */
{
    float mx[FRAME_SYMS];
    for (int i = 0; i < FRAME_SYMS; i++) {
        const cpx *z = &r->z[1 + i * SPS]; mx[i] = 0;
        for (int c = 0; c < 4; c++) {
            float re = 0, im = 0;
            for (int j = 0; j < SPS; j++) { re += z[j].re * dvrx_tone_re[c][j] + z[j].im * dvrx_tone_im[c][j]; im += z[j].im * dvrx_tone_re[c][j] - z[j].re * dvrx_tone_im[c][j]; }
            float m = fsqrt(re * re + im * im); if (m > mx[i]) mx[i] = m;
        }
    }
    for (int i = 1; i < FRAME_SYMS; i++) { float v = mx[i]; int j = i - 1; while (j >= 0 && mx[j] > v) { mx[j + 1] = mx[j]; j--; } mx[j + 1] = v; }
    float amp = 0.5f * (mx[FRAME_SYMS / 2 - 1] + mx[FRAME_SYMS / 2]) / SPS;
    for (int p = 0; p < 4; p++) for (int c = 0; c < 4; c++) for (int j = 0; j < SPS; j++) {
        r->T[p][c][j].re = amp * dvrx_tone_re[c][j]; r->T[p][c][j].im = amp * dvrx_tone_im[c][j];
    }
    r->tset = 1;
}

/* forward pass for one phase hypothesis; keep: store alphas. Returns the best final metric */
static float forward(struct dvrx *r, cpx ph, const float e[4][4], int prev0, int keep)
{
    float al[4][2], nx[4][2];
    for (int p = 0; p < 4; p++) al[p][0] = al[p][1] = NEG;
    if (prev0 >= 0) al[prev0][0] = 0; else for (int p = 0; p < 4; p++) al[p][0] = 0;
    if (keep) memcpy(r->al[0], al, sizeof al);
    for (int i = 0; i < FRAME_SYMS; i++) {
        for (int c = 0; c < 4; c++) nx[c][0] = nx[c][1] = NEG;
        for (int p = 0; p < 4; p++) for (int c = 0; c < 4; c++) {
            float b = r->C[i][p][c].re * ph.re + r->C[i][p][c].im * ph.im;     /* Re(C conj(ph)) */
            for (int th = 0; th < 2; th++) {
                float v = al[p][th] + (th ? -b : b) - e[p][c];
                int th2 = th ^ ADV[c];
                if (v > nx[c][th2]) nx[c][th2] = v;
            }
        }
        memcpy(al, nx, sizeof al);
        if (keep) memcpy(r->al[i + 1], al, sizeof al);
    }
    float best = NEG;
    for (int c = 0; c < 4; c++) for (int th = 0; th < 2; th++) if (al[c][th] > best) best = al[c][th];
    return best;
}

static void metrics(struct dvrx *r, const cpx *hyp, int nh, int prev0)
{
    float e[4][4];
    for (int p = 0; p < 4; p++) for (int c = 0; c < 4; c++) {
        float s = 0; for (int j = 0; j < SPS; j++) s += r->T[p][c][j].re * r->T[p][c][j].re + r->T[p][c][j].im * r->T[p][c][j].im;
        e[p][c] = 0.5f * s;
    }
    int h = 0; float best = NEG;
    if (nh > 1) for (int k = 0; k < nh; k++) { float v = forward(r, hyp[k], e, prev0, 0); if (v > best) { best = v; h = k; } }
    cpx ph = hyp[h];
    forward(r, ph, e, prev0, 1);
    float be[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    for (int i = FRAME_SYMS - 1; i >= 0; i--) {
        float nb[4][2], out[4];
        for (int c = 0; c < 4; c++) out[c] = NEG;
        for (int p = 0; p < 4; p++) nb[p][0] = nb[p][1] = NEG;
        for (int p = 0; p < 4; p++) for (int c = 0; c < 4; c++) {
            float b = r->C[i][p][c].re * ph.re + r->C[i][p][c].im * ph.im;
            for (int th = 0; th < 2; th++) {
                float br = (th ? -b : b) - e[p][c], nxt = be[c][th ^ ADV[c]];
                float tot = r->al[i][p][th] + br + nxt;
                if (tot > out[c]) out[c] = tot;
                if (br + nxt > nb[p][th]) nb[p][th] = br + nxt;
            }
        }
        float mn = NEG;
        for (int c = 0; c < 4; c++) if (out[c] > mn) mn = out[c];
        for (int c = 0; c < 4; c++) r->met[i][c] = out[c] - mn;
        float bm = NEG;
        for (int p = 0; p < 4; p++) for (int th = 0; th < 2; th++) if (nb[p][th] > bm) bm = nb[p][th];
        for (int p = 0; p < 4; p++) for (int th = 0; th < 2; th++) be[p][th] = nb[p][th] - bm;
    }
}

static cpx unit(cpx v) { float m = fsqrt(v.re * v.re + v.im * v.im) + 1e-20f; v.re /= m; v.im /= m; return v; }
static cpx rot(cpx v, float c, float s) { cpx o = {v.re * c - v.im * s, v.re * s + v.im * c}; return o; }

/* carrier phase from the decided symbols (phase 0 at symbol 0) */
static cpx phase_est(const struct dvrx *r, const uint8_t prev[FRAME_SYMS], const uint8_t syms[FRAME_SYMS])
{
    cpx v = {0, 0}; int th = 0;
    for (int i = 0; i < FRAME_SYMS; i++) {
        cpx c = r->C[i][prev[i]][syms[i]];
        if (th) { v.re -= c.re; v.im -= c.im; } else { v.re += c.re; v.im += c.im; }
        th ^= ADV[syms[i]];
    }
    return unit(v);
}

/* decode the frame at t with the current state; updates the state when good. Returns the
   re-encoded symbol error rate and the timing error estimate (samples, good frames only) */
static float decode_frame(struct dvrx *r, uint32_t t, uint8_t payload[INFO], float *terr)
{
    uint8_t syms[FRAME_SYMS], prev[FRAME_SYMS];
    analytic(r, t);
    if (r->relevel && (int32_t)(t - r->relevel_at) >= 0) { r->tset = 0; r->ngood = 0; r->relevel = 0; }
    if (!r->tset) init_templates(r);
    correlate(r, 1);
    /* phase hypotheses: the sync symbols' estimate; the carried phase (and its pi twin after a
       bad frame) with +-0.5 rad */
    cpx hyp[8]; int nh = 0;
    {
        cpx v = {0, 0}; int th = ADV[SYNC_T[0]];
        for (int k = 1; k < SYNC_LEN; k++) {
            cpx c = r->C[k][SYNC_T[k - 1]][SYNC_T[k]];
            if (th) { v.re -= c.re; v.im -= c.im; } else { v.re += c.re; v.im += c.im; }
            th ^= ADV[SYNC_T[k]];
        }
        hyp[nh++] = unit(v);
    }
    if (r->psi_set) {
        for (int tw = 0; tw < (r->psi_amb ? 2 : 1); tw++) {
            cpx p = r->psi; if (tw) { p.re = -p.re; p.im = -p.im; }
            hyp[nh++] = p; hyp[nh++] = rot(p, DVRX_ROT_RE, DVRX_ROT_IM); hyp[nh++] = rot(p, DVRX_ROT_RE, -DVRX_ROT_IM);
        }
    }
    cpx psi = hyp[0];
    for (int pass = 0; pass < 2; pass++) {
        if (pass == 0) metrics(r, hyp, nh, r->last_sym); else metrics(r, &psi, 1, r->last_sym);
        decode_bits(r->met, payload);
        frame_symbols(payload, syms);
        prev[0] = (uint8_t)(r->last_sym >= 0 ? r->last_sym : syms[0]);
        for (int i = 1; i < FRAME_SYMS; i++) prev[i] = syms[i - 1];
        psi = phase_est(r, prev, syms);
    }
    int err = 0;
    for (int i = 0; i < FRAME_SYMS; i++) {
        int a = 0; for (int c = 1; c < 4; c++) if (r->met[i][c] > r->met[i][a]) a = c;
        err += a != syms[i];
    }
    float ser = (float)err / FRAME_SYMS;
    *terr = 0;
    if (ser < BAD_SER) {
        /* timing: coherent matched-filter sum at -1 / 0 / +1 samples, parabolic peak */
        float J[3];
        for (int off = 0; off < 3; off++) {
            cpx v = {0, 0}; int th = 0;
            for (int i = 0; i < FRAME_SYMS; i++) {
                const cpx *z = &r->z[off + i * SPS], *T = r->T[prev[i]][syms[i]]; float re = 0, im = 0;
                for (int j = 0; j < SPS; j++) { re += z[j].re * T[j].re + z[j].im * T[j].im; im += z[j].im * T[j].re - z[j].re * T[j].im; }
                if (th) { v.re -= re; v.im -= im; } else { v.re += re; v.im += im; }
                th ^= ADV[syms[i]];
            }
            J[off] = v.re * v.re + v.im * v.im;
        }
        for (int k = 0; k < 3; k++) J[k] = fsqrt(J[k]);
        if (J[1] >= J[0] && J[1] >= J[2]) { float den = J[0] - 2 * J[1] + J[2]; *terr = den != 0 ? 0.5f * (J[0] - J[2]) / den : 0; }
        else *terr = J[0] > J[2] ? -1.0f : 1.0f;
        /* templates: rotate each symbol back by the carrier and its trellis phase, average in */
        r->ngood++;
        float alpha = 1.0f / (float)(r->ngood + 1); if (alpha < 0.02f) alpha = 0.02f;
        int th = 0;
        for (int i = 0; i < FRAME_SYMS; i++) {
            const cpx *z = &r->z[1 + i * SPS]; cpx *T = r->T[prev[i]][syms[i]];
            float sr = th ? -psi.re : psi.re, si = th ? -psi.im : psi.im;      /* conj below */
            for (int j = 0; j < SPS; j++) {
                float re = z[j].re * sr + z[j].im * si, im = z[j].im * sr - z[j].re * si;
                T[j].re += alpha * (re - T[j].re); T[j].im += alpha * (im - T[j].im);
            }
            th ^= ADV[syms[i]];
        }
        int adv = 0; for (int i = 0; i < FRAME_SYMS; i++) adv ^= ADV[syms[i]];
        r->psi = adv ? (cpx){-psi.re, -psi.im} : psi; r->psi_set = 1; r->psi_amb = 0;
        r->last_sym = syms[FRAME_SYMS - 1];
    } else {
        r->last_sym = -1; r->psi_amb = 1;
    }
    return ser;
}

/* ------------------------------------------------------------------ driver */
static void pack(const uint8_t payload[INFO], uint8_t sf[6])
{
    memset(sf, 0, 6);
    for (int i = 0; i < INFO; i++) if (payload[i]) sf[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
}

int dvrx_step(struct dvrx *r, uint8_t sf[6], int *good)
{
    while (r->nd + SPS <= r->n) { diff_at(r, r->nd); r->nd++; }
    if (r->pending_end) { r->pending_end = 0; return DVRX_END; }
    uint8_t payload[INFO]; float terr;
    if (r->locked) {
        if (r->n < r->t + FRAME + HD + 3) return DVRX_NONE;
        float ser = decode_frame(r, r->t, payload, &terr);
        *good = ser < BAD_SER; pack(payload, sf);
        r->frac += r->drift + 0.3f * terr;
        r->drift += 0.02f * terr; if (r->drift > 0.12f) r->drift = 0.12f; if (r->drift < -0.12f) r->drift = -0.12f;
        r->t += FRAME;
        while (r->frac >= 0.5f) { r->frac -= 1; r->t++; }
        while (r->frac < -0.5f) { r->frac += 1; r->t--; }
        r->nfr++; r->nbad += !*good; r->ser_sum += ser; r->ser_tail[r->nfr % DVRX_HOLD] = ser;
        r->bad_run = *good ? 0 : r->bad_run + 1;
        if (r->bad_run >= DVRX_HOLD) {
            float tail = 0; for (int k = 0; k < DVRX_HOLD; k++) tail += r->ser_tail[k];
            r->st_frames = r->nfr - DVRX_HOLD; r->st_bad = r->nbad - DVRX_HOLD;
            r->st_ser = r->st_frames > 0 ? (int)(1000 * (r->ser_sum - tail) / r->st_frames) : 0;
            r->locked = 0; r->searched = r->t; r->pending_end = 1;
        }
        return DVRX_FRAME;
    }
    if ((int32_t)(r->n - r->searched) > DVRX_RING - 256) r->searched = r->n - (DVRX_RING - 256);   /* fell behind */
    while (r->searched + 2 * FRAME + 2 + SYNC_LEN * SPS <= r->nd && r->searched + 2 * FRAME + HD + 3 <= r->n) {
        uint32_t u = r->searched++;
        if (score(r, u) <= LOCK_SCORE) continue;
        int ok = 1;
        for (int k = 1; k <= 2 && ok; k++) {
            float m = -1;
            for (int d = -1; d <= 1; d++) { float s = score(r, u + (uint32_t)(k * FRAME + d)); if (s > m) m = s; }
            ok = m > 0.2f;
        }
        state_reset(r);
        for (int k = 0; k < 2 && ok; k++) ok = decode_frame(r, u + (uint32_t)(k * FRAME), payload, &terr) < 0.1f;
        state_reset(r);
        if (ok) {
            r->locked = 1; r->t = u; r->frac = 0; r->drift = 0; r->relevel = 0;
            r->nfr = r->nbad = r->bad_run = 0; r->ser_sum = 0; memset(r->ser_tail, 0, sizeof r->ser_tail);
            return DVRX_LOCK;
        }
    }
    return DVRX_NONE;
}
