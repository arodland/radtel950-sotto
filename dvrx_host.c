/* host harness for dvrx: raw int16 at 19.2 kHz on stdin (or a file), over lines on stdout,
   decoded superframes + good flags to OUT.bit / OUT.good if given.
   usage: dvrx_host IN.raw [T0_UNIX] [OUT_PREFIX] */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "dvrx.h"
int main(int argc, char **argv)
{
    FILE *in = fopen(argv[1], "rb"); double t0 = argc > 2 ? atof(argv[2]) : 0;
    FILE *ob = NULL, *og = NULL;
    if (argc > 3) { char p[512]; snprintf(p, sizeof p, "%s.bit", argv[3]); ob = fopen(p, "wb"); snprintf(p, sizeof p, "%s.good", argv[3]); og = fopen(p, "wb"); }
    struct dvrx *r = malloc(dvrx_size()); dvrx_init(r);
    int16_t buf[384]; size_t got; unsigned long long n = 0;
    while ((got = fread(buf, 2, 384, in)) > 0) {
        for (size_t i = 0; i < got; i++) dvrx_push(r, buf[i]);
        n += got;
        uint8_t sf[6]; int good, ev;
        while ((ev = dvrx_step(r, sf, &good)) != DVRX_NONE) {
            time_t tt = (time_t)(t0 + n / 19200.0); char ts[16]; strftime(ts, sizeof ts, "%H:%M:%S", localtime(&tt));
            if (ev == DVRX_LOCK) printf("%s over: locked\n", ts);
            else if (ev == DVRX_FRAME) { if (ob) { fwrite(sf, 1, 6, ob); fputc(good, og); } }
            else { int f, b, s; dvrx_stats(r, &f, &b, &s); printf("%s over: end, %d frames (%.1f s), %d bad, raw SER %.4f\n", ts, f, f * 0.04, b, s / 1000.0); }
        }
    }
    fprintf(stderr, "struct dvrx: %u bytes\n", dvrx_size());
    return 0;
}
