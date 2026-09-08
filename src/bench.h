/* THE WORK BENCH.
 *
 * The perf report cannot settle "deadline vs counter" because a frame is not
 * a fixed amount of work in both builds.  In a counter build a vsync arrives
 * every 564480 guest cycles and guest cycles ARE guest instructions, so the
 * work per frame is fixed by construction.  In a deadline build a vsync
 * arrives every 564480 cycles of K times PRFC1, and how much guest work fits
 * in that depends entirely on whether K is right.
 *
 * So the comparison is not frames per second.  It is: run a FIXED NUMBER OF
 * VSYNCS from the SAME savestate, and report how much guest work went through
 * and how long the wall clock took.  A build that does the same work in less
 * wall time is faster.  A build that reports a different amount of work for
 * the same vsyncs has the wrong K, and its frame times mean nothing until
 * that is fixed.
 *
 * This build is the reference: guest cycles here come off the emitted per-
 * block charge, so `cyc/vsync` is what the deadline build has to match.
 */
#ifndef FGL_BENCH_H
#define FGL_BENCH_H

#ifndef FGL_BENCH
#define FGL_BENCH 0
#endif

/* Guest vsyncs in the measured window.  1800 is thirty guest seconds. */
#ifndef FGL_BENCH_VSYNCS
#define FGL_BENCH_VSYNCS 1800
#endif

/* Called once per guest vsync with the guest clock.  Does nothing unless
 * FGL_BENCH. */
/* Vsyncs discarded before the window opens: savestate load, the first compile
 * burst, and whatever backlog they leave in the owed pool.  300 is five guest
 * seconds, comfortably past the 2.18-billion-cycle lump that was landing at
 * about the 180th. */
#ifndef FGL_BENCH_WARMUP
#define FGL_BENCH_WARMUP 300
#endif

void fgl_bench_vsync(unsigned int guest_cycle);

/* WHAT THE BENCH SAW, so the dl: line can print it next to what the report
 * saw.  Both are called once per delivered vblank, eighteen lines apart in the
 * same branch of psxRcntUpdate, yet they disagreed about the guest clock by
 * 3.1x.  Printing both settles which one is lying without another theory. */
extern unsigned int fgl_bench_calls, fgl_bench_cycle;

/* GUEST BLOCKS EXECUTED, and the reason the rest of this header is no longer
 * the whole story.  The text above assumes the deadline build's rate depends
 * on getting K right; under the stretch it does not -- vsync/s is 60 divided
 * by the delivered stretch whatever the recompiler does, so "same vsyncs, less
 * wall time" cannot happen and the comparison ties by construction.
 *
 * Blocks per wall second is immune: not derived from the guest clock, not
 * affected by which perf bracket the work is billed to.  Zero unless the build
 * counts (-DWITH_FGL_BLOCK_COUNT=ON); turn it on for both sides of a pair. */
extern unsigned int fgl_blocks_run;
extern unsigned int fgl_blocks_ret, fgl_blocks_ind, fgl_blocks_nolink;

#endif
