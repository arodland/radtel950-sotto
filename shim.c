/* The few C library functions the fixed-point sotto calls, for the bare-metal Cortex-M4 bench
 * (cm4/run.py): a bump allocator (heap_used is the high-water mark), byte loops for the
 * memory functions, and no output. Not for a real firmware, which has its own C library.
 * SPDX-License-Identifier: MIT */
#include <stddef.h>
#include <stdint.h>
typedef struct FILE FILE;
FILE *stderr;
int fprintf(FILE *f, const char *fmt, ...) { (void)f; (void)fmt; return 0; }
static unsigned char heap[48 * 1024] __attribute__((aligned(8)));
size_t heap_used;
/* Word-at-a-time: startup clears .bss through this before the PLL is up, and the
 * bootloader's IWDG is already running, so a byte loop over ~50 KB can trip it. */
void *memset(void *d, int c, size_t n) {
  unsigned char *p = d;
  uint32_t w = (unsigned char)c * 0x01010101u;
  while (n && ((uintptr_t)p & 3)) { *p++ = (unsigned char)c; n--; }
  for (; n >= 16; n -= 16, p += 16) {
    ((uint32_t *)p)[0] = w; ((uint32_t *)p)[1] = w; ((uint32_t *)p)[2] = w; ((uint32_t *)p)[3] = w;
  }
  while (n--) *p++ = (unsigned char)c;
  return d;
}
void *malloc(size_t n) {
  void *p;
  n = (n + 7) & ~(size_t)7;
  if (heap_used + n > sizeof heap) return 0;
  p = heap + heap_used;
  heap_used += n;
  return p;
}
void *calloc(size_t n, size_t s) { void *p = malloc(n * s); if (p) memset(p, 0, n * s); return p; }
void free(void *p) { (void)p; }
void abort(void) { for (;;) __asm__ volatile("bkpt #1"); }
void *memcpy(void *d, const void *s, size_t n) {
  unsigned char *a = d;
  const unsigned char *b = s;
  if ((((uintptr_t)a | (uintptr_t)b) & 3) == 0)
    for (; n >= 4; n -= 4, a += 4, b += 4) *(uint32_t *)a = *(const uint32_t *)b;
  while (n--) *a++ = *b++;
  return d;
}
void *memmove(void *d, const void *s, size_t n) {
  unsigned char *a = d;
  const unsigned char *b = s;
  if (a < b) while (n--) *a++ = *b++;
  else { a += n; b += n; while (n--) *--a = *--b; }
  return d;
}
int memcmp(const void *x, const void *y, size_t n) {
  const unsigned char *a = x, *b = y;
  for (; n; n--, a++, b++) if (*a != *b) return *a - *b;
  return 0;
}
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
/* the run-time ABI's memory helpers, which the compiler calls for struct copies */
void __aeabi_memclr(void *d, size_t n) { memset(d, 0, n); }
void __aeabi_memclr4(void *d, size_t n) { memset(d, 0, n); }
void __aeabi_memclr8(void *d, size_t n) { memset(d, 0, n); }
void __aeabi_memset(void *d, size_t n, int c) { memset(d, c, n); }
void __aeabi_memset4(void *d, size_t n, int c) { memset(d, c, n); }
void __aeabi_memcpy(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memcpy4(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memcpy8(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memmove(void *d, const void *s, size_t n) { memmove(d, s, n); }
void __aeabi_memmove4(void *d, const void *s, size_t n) { memmove(d, s, n); }
/* 64-bit division (__aeabi_uldivmod / __aeabi_ldivmod in divmod.s call these): one
   hardware division when both fit 32 bits, else shift and subtract. libgcc's version is
   faster on wide operands, so the bench slightly overstates the few 64-bit divisions. */
uint64_t udivmod64(uint64_t n, uint64_t d, uint64_t *rem) {
  uint64_t q = 0;
  int s, i;
  if ((n >> 32) == 0 && (d >> 32) == 0) { *rem = (uint32_t)n % (uint32_t)d; return (uint32_t)n / (uint32_t)d; }
  if (n < d) { *rem = n; return 0; }
  s = __builtin_clzll(d) - __builtin_clzll(n);
  d <<= s;
  for (i = 0; i <= s; i++, d >>= 1) {
    q <<= 1;
    if (n >= d) n -= d, q |= 1;
  }
  *rem = n;
  return q;
}
int64_t sdivmod64(int64_t n, int64_t d, int64_t *rem) {
  int nn = n < 0, nd = d < 0;
  uint64_t r, q = udivmod64(nn ? -(uint64_t)n : (uint64_t)n, nd ? -(uint64_t)d : (uint64_t)d, &r);
  *rem = nn ? -(int64_t)r : (int64_t)r;
  return (nn ^ nd) ? -(int64_t)q : (int64_t)q;
}
