/* WHERE THE WALL TIME GOES, in buckets that sum to the run.
 *
 * bloom's bench line reports one split: "% in guest code", taken from a timer
 * around lightrec_execute() in the lightrec plugin.  That number cannot say
 * what the other 14% is, and — more importantly — it cannot say what is inside
 * the 86%, which is where the gap against bloop lives.  This divides a run's
 * real seconds between running the guest, the GTE, classifying its memory
 * accesses, compiling, servicing hardware, and drawing.
 *
 * THE CLOCK IS PRFC1.  KOS uses PRFC0 for its own nanosecond timer
 * (kernel/arch/dreamcast/kernel/perfctr.c), so the second performance counter
 * is free.  In elapsed-time mode it counts one per CPU cycle, 5 ns a count,
 * which is finer than any TMU and is not affected by anything the emulator
 * does to the timers.
 *
 * SELF TIME, VIA A STACK.  These regions nest — psxHwWrite runs inside the
 * lightrec memory wrapper, the display-list walk inside a hardware write — so
 * an enter/leave pair charges elapsed time to whatever is on top of the stack
 * at that moment and no higher.  Every bucket is exclusive of the ones it
 * contains and the columns add up to the run rather than to some multiple of
 * it.  That is the property that makes the numbers arguable, and it is why
 * this is not a set of independent timers.
 *
 * ONE THREAD.  The stack belongs to the thread that called prof_claim(), which
 * must be the emulator thread — the benchmark arms the profiler from a
 * separate watcher thread, so claiming the caller of prof_start() claims the
 * wrong one and every bucket reads zero.
 * lightrec runs a compiler worker (recompiler.c:272, "Threaded recompiler
 * started with 1 workers") and a bracket taken on it would interleave with the
 * emulator's and corrupt both.  Calls from any other thread are ignored, and
 * the report prints per-thread CPU time separately so the worker's share is
 * visible rather than silently folded into whatever bucket was on top.
 *
 * OFF BY DEFAULT — configure with -DPROF=ON.  The counter read is an on-chip
 * IO access and bloom crosses into C far more often than bloop does, so this
 * instrument is not free and must not be in the build that produces a
 * throughput figure.  Take the window both ways; the difference prices the
 * instrument, and that difference goes in the report next to the numbers.
 *
 * WHAT IT CANNOT SEE.  Only C choke points are bracketed.  Time inside
 * generated code, inside the dispatcher, and in lightrec's own assembly falls
 * into GUEST, which is the residual bucket by construction — it is not
 * measured, it is what is left after everything else is subtracted.  Splitting
 * GUEST further is a job for the PC sampler (jitprof.c) and the SH-4 issue and
 * cache-stall counters, not for this file.
 */

#ifndef BLOOM_PROF_H
#define BLOOM_PROF_H

#include <stdint.h>

typedef enum {
	PROF_GUEST,	/* residual: generated code, the dispatcher, lightrec asm */
	PROF_GTE,	/* cop2_op() and the cp2_ops[] table it dispatches to */
	PROF_MEMCLASS,	/* deciding in software what a guest address is:
			 * lightrec_rw_generic_cb and the plugin's rw wrappers.
			 * bloop has no equivalent — its MMU classifies. */
	PROF_IO,	/* psxHwRead/psxHwWrite bodies, inside MEMCLASS */
	PROF_GPUFRONT,	/* gpulib: GP0 decode and the DMA display-list walk */
	PROF_RENDER,	/* src/pvr.c: the PVR renderer proper */
	PROF_FLIP,	/* pvr_wait_ready + pvr_scene_finish — waiting, not work */
	PROF_COMPILE,	/* block compile and emit, when it happens inline */
	PROF_INVALIDATE,/* lightrec_invalidate* from the DMA clear path */
	PROF_CD,	/* the host image read */
	PROF_EVENTS,	/* gen_interupt, psxBranchTest, event scheduling */
	PROF_SPU,	/* src/aica.c */
	PROF_N
} prof_bucket;

const char *prof_name(prof_bucket b);

#ifdef BLOOM_PROF

void prof_claim(void);		/* call on the emulator thread — see prof.c */
void prof_start(void);		/* configure PRFC1 and begin charging */
void prof_reset(void);		/* zero the buckets, keep the counter running */

/* Time N empty enter/leave pairs so the report can subtract the instrument
 * from the buckets it measures.  Call once after prof_start(); it zeroes the
 * buckets when it is done. */
void prof_calibrate(void);
uint32_t prof_pair_ns(void);
void prof_enter(prof_bucket b);
void prof_leave(void);

uint64_t prof_ns(prof_bucket b);
uint32_t prof_calls(prof_bucket b);

/* Everything the buckets account for.  Printed against the run's wall clock as
 * a check on the instrument itself: two different clocks reading the same
 * seconds, so a large disagreement means one of them stopped and neither
 * number should be believed. */
uint64_t prof_total_ns(void);

/* Non-zero if a bracket was left unbalanced or nested past the stack.  The
 * numbers are wrong if this fires; it is printed rather than asserted because
 * a run that got this far is still worth its other counters. */
unsigned int prof_broken(void);

/* ISSUE SLOTS AND STALLS, inside generated code specifically.
 *
 * GUEST is a residual — 64% of the run with no bracket of its own — so the
 * bucket table cannot say whether it is slow because it executes more host
 * instructions or because it stalls on the same number.  The SH-4's event
 * counter answers that directly, and it is the only instrument that can.
 *
 * PRFC1 stays on elapsed time for the buckets; PRFC0 takes the event.  bloop
 * has to choose between its buckets and its counters because both of its
 * modes wanted PRFC1 (`bloop/Makefile` refuses PROF=1 STALL=1); here they
 * coexist, so one run gives buckets and counters over the same window.
 *
 * GATED TO GENERATED CODE.  A whole-run figure mixes generated code with the
 * renderer, the GTE and the IO handlers — and the two emulators have very
 * different non-CPU fractions (bloom 19%, bloop 35%), so a raw whole-run IPC
 * comparison flatters bloom by exactly the amount that differs.  The counters
 * here accumulate only while the bracket stack is empty, which is the
 * definition of GUEST, and the report prints both figures so the contamination
 * is visible rather than assumed.
 *
 * -DSTALL=n picks the PRFC0 event:
 *    1  instruction cache pipeline freeze (cycles)
 *    2  operand cache pipeline freeze (cycles)
 *    3  instructions issued
 *    4  instructions issued in parallel (the pairing rate, against 3)
 */

/* The table.  frames and guest_insns are denominators — a bucket in percent
 * cannot be compared against another emulator drawing a different scene, so
 * every row also prints microseconds per frame. */
void prof_report(uint32_t wall_ms, uint32_t frames, uint64_t guest_insns);

#else

#define prof_claim()			((void)0)
#define prof_start()			((void)0)
#define prof_reset()			((void)0)
#define prof_calibrate()		((void)0)
#define prof_pair_ns()			0u
#define prof_enter(b)			((void)0)
#define prof_leave()			((void)0)
#define prof_ns(b)			0ull
#define prof_calls(b)			0u
#define prof_total_ns()			0ull
#define prof_broken()			0u
#define prof_report(ms, f, i)		((void)0)

#endif /* BLOOM_PROF */

#endif /* BLOOM_PROF_H */
