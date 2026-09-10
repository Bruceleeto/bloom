/* WHICH CODE THE FRAME ACTUALLY RUNS, BY SAMPLING.
 *
 * The performance counters give totals: 14.2 ms of icache stall a frame, 78.6%
 * of it in emitted code.  What they cannot say is WHICH functions are paying
 * it, and that is the question a layout change has to answer first.  Two
 * functions collide in the icache when their addresses agree mod 8192 -- the
 * cache is 8 KiB and direct-mapped -- so fixing a collision means knowing
 * which pairs are both hot.  A pair where one side runs twice a frame is not
 * worth moving.
 *
 * THE METHOD IS A TIMER INTERRUPT READING SPC.  TMU2 underflows at
 * HOTSET_HZ, the handler takes the interrupted PC out of the saved context
 * and bumps a bucket.  No instrumentation in the measured code, so nothing
 * about `.text` changes when this is on except this file's own presence --
 * which is the usual caveat and the reason the report prints addresses rather
 * than trying to be its own conclusion.
 *
 * WHY IT SEES EMITTED CODE TOO.  An interrupt is taken wherever the machine
 * happens to be, and generated blocks are just addresses; samples landing in
 * the code buffer are counted separately and reported as one lump.  That makes
 * the JIT/native split a by-product rather than a second measurement.
 *
 * BUCKETS ARE 128 BYTES, not functions: the handler has no symbol table.
 * `tools/hotset_resolve.py` folds the printed addresses back into function
 * names with `sh-elf-nm -S`, and `tools/icache_alias.py` takes that ranked
 * list and prints the colliding pairs.  Neither needs the Dreamcast.
 *
 * COST.  One interrupt every 500 us at the default rate: a few hundred cycles
 * against 200 million a second.  Under a tenth of a percent, and it does not
 * touch the doors the way emitmiss.c does. */

#ifndef BLOOM_HOTSET_H
#define BLOOM_HOTSET_H

#if defined(BLOOM_HOTSET) && defined(__sh__)

/* Samples a second.  A frame at 15 fps is 66 ms, so 2000 Hz puts ~133 samples
 * in a frame and ~16000 in the window below -- enough that a function worth
 * moving is tens of samples clear of the noise. */
#ifndef HOTSET_HZ
#define HOTSET_HZ 2000
#endif

/* Presented frames to let go by before sampling starts.  Level load runs a
 * different program than the steady state: the compiler thread is working and
 * the hot set it shows is the compiler's, not the game's. */
#ifndef HOTSET_SKIP
#define HOTSET_SKIP 120
#endif

/* Presented frames sampled.  At 15 fps this is eight seconds. */
#ifndef HOTSET_FRAMES
#define HOTSET_FRAMES 120
#endif

/* Buckets printed, highest first.  The tail is a long thin thing and nothing
 * in it is worth moving. */
#ifndef HOTSET_REPORT
#define HOTSET_REPORT 64
#endif

void bloom_hotset_vsync(void);

#define HOTSET_VSYNC()  bloom_hotset_vsync()

#else

#define HOTSET_VSYNC()  do { } while (0)

#endif

#endif /* BLOOM_HOTSET_H */
