/* WHERE A FRAME GOES, IN FOUR NUMBERS AND A PRINTF.
 *
 * Not a profiler.  A profiler says which function; this says which STAGE, and
 * that is the question a recompiler rewrite actually needs answered -- whether
 * a change moved time out of emulated code and into the renderer, or moved it
 * nowhere and only moved it around inside the same bucket.
 *
 * Wall clock, microseconds, accumulated per bucket and reported over the same
 * one-second window `dc_vout_flip` already keeps for the fps counter, so the
 * buckets and the frame rate are measured over exactly the same interval and
 * can be put in the same sentence.
 *
 * IT COSTS SOMETHING AND THE COST IS NOT ZERO.  Each bracket is two TMU reads
 * and a 64-bit subtract.  CPU is bracketed per dispatch (tens per frame), GPU
 * per command list (hundreds), so the overhead is tens of microseconds a frame
 * -- fine for reading a spread, not fine for quoting a frame time from.  A
 * number to compare against a build without this is a number from a build
 * without this.
 *
 * WHAT `other` IS.  The window's wall time minus the four buckets: audio, the
 * CD drive, MDEC, input, KOS, and the parts of pcsx_rearmed that are none of
 * the above.  If it grows, the thing to do is bracket another stage, not to
 * guess.
 *
 * JIT IS ON ANOTHER THREAD.  The recompiler runs in its own worker, so its
 * microseconds are real work but they do not come out of the frame's budget
 * the way the other three do -- they overlap.  It is reported because a
 * compile storm is what a recompile dip IS, and seeing it spike next to a
 * dropped frame is the whole point.
 */

#ifndef BLOOM_PERF_H
#define BLOOM_PERF_H

#include <stdint.h>

enum {
	PERF_CPU,       /* inside generated code: lightrec_execute        */
	PERF_JIT,       /* compiling a block (recompiler thread)          */
	PERF_EVENT,     /* gen_interupt: root counters, IRQs, DMA         */
	PERF_GPU,       /* do_cmd_list: GPU command list -> PVR polygons  */
	PERF_FLIP,      /* dc_vout_flip: scene submit and present         */
	PERF_N
};

extern uint64_t bloom_perf_us[PERF_N];
extern uint32_t bloom_perf_cnt[PERF_N];

/* Microseconds.  Defined in platform.c so that lightrec and pcsx_rearmed do
 * not have to see a KOS header to be bracketed. */
uint64_t bloom_perf_now(void);

#define PERF_BEGIN(b)   uint64_t perf_t0_##b = bloom_perf_now()
#define PERF_END(b)     do {                                            \
		bloom_perf_us[b] += bloom_perf_now() - perf_t0_##b;      \
		bloom_perf_cnt[b]++;                                     \
	} while (0)

#endif /* BLOOM_PERF_H */
