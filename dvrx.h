/* dvrx: the dvtx V2 receiver in C (port of dv_live.py's coherent path), for the radio and for a
   host test harness. Input: the FM discriminator audio at 19.2 kHz (16 samples per symbol), any
   level, DC removed. Output: one 6-byte sotto superframe per 40 ms frame with a good flag, and
   over start / end events. Floats only, no libm. SPDX-License-Identifier: MIT */
#ifndef DVRX_H
#define DVRX_H
#include <stdint.h>

#define DVRX_RING 4096                       /* input samples kept (213 ms) */
#define DVRX_HOLD 12                         /* bad frames in a row before the over ends */

enum { DVRX_NONE, DVRX_LOCK, DVRX_FRAME, DVRX_END };

struct dvrx;
unsigned dvrx_size(void);                    /* bytes for dvrx_init's buffer */
void dvrx_init(struct dvrx *r);
void dvrx_push(struct dvrx *r, int16_t x);   /* one input sample */
/* call until it returns DVRX_NONE: DVRX_FRAME fills sf (6 bytes) and *good (0/1);
   DVRX_END comes after the last frame of an over (the HOLD bad frames are already out) */
int dvrx_step(struct dvrx *r, uint8_t sf[6], int *good);
/* last over's stats: frames, bad frames, raw symbol errors x 1000 per frame (mean) */
void dvrx_stats(const struct dvrx *r, int *frames, int *bad, int *ser_milli);
int dvrx_locked(const struct dvrx *r);
uint32_t dvrx_count(const struct dvrx *r);  /* samples pushed so far */
/* the input level changes at sample n (absolute, as dvrx_count): the first frame starting at or
   after n re-learns the templates from the ideal tones at the new level */
void dvrx_relevel_at(struct dvrx *r, uint32_t n);
#endif
