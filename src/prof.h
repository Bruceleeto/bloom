/* WHERE A FRAME GOES, AND WHY THE PIPELINE WAS STOPPED WHILE IT WENT THERE.
 *
 * Ported from bloop's `src/prof.c`, which is the only instrument in this
 * project whose known defects are already fixed. Two questions, one build:
 *
 *   1. WHICH STAGE. Self time per bucket, exclusive, via a bracket stack, so
 *      the columns add to the run instead of to some multiple of it.
 *   2. WHY. PRFC0 rotates once a second through issue slots, instruction-cache
 *      freeze and operand-cache freeze, each keeping its own share of the
 *      guest region, so ONE run answers all three over the same scene.
 *
 * THE CLOCK IS PRFC1 IN ELAPSED-TIME MODE, not a TMU. One count is one CPU
 * cycle, 5 ns at 200 MHz. TMU rates are changed by other knobs in this tree,
 * so a duration read from one is not a duration.
 *
 * ------------------------------------------------------------------------
 * THE FIVE THINGS THAT WENT WRONG LAST TIME. Every one of these cost this
 * project a wrong conclusion. They are handled here; do not undo them.
 * ------------------------------------------------------------------------
 *
 * 1. `PMCR_INSTRUCTION_ISSUED` (0x13) COUNTS A DUAL-ISSUED PAIR ONCE, not
 *    twice. An earlier project carried a cache theory for a month because its
 *    calibration workload had nothing pairable in it. Treat this counter as
 *    "issue slots used", never as "instructions retired", and if you need the
 *    real instruction count add `PMCR_PARALLEL_INSTRUCTION_ISSUED_MODE` on the
 *    other counter and sum them.
 *
 * 2. THE BUCKETS NEST, so `prof_enter` charges elapsed time to whatever is on
 *    top of the stack and no higher. A flat version of this design once summed
 *    to 104%, and a `rest` bucket elsewhere in this project turned out to be
 *    three times larger than its author believed because it silently absorbed
 *    the renderer. `prof_broken()` is non-zero if a bracket was left unbalanced
 *    or the stack overflowed -- read it before believing any row.
 *
 * 3. THE INSTRUMENT IS NOT FREE, so `prof_calibrate()` times 65536 empty
 *    bracket pairs and the report subtracts `prof_pair_ns() * calls` from every
 *    bucket. bloom's earlier build of this design measured 18% overhead across
 *    17M pairs, 46% of it landing on its most-bracketed bucket. A frame time
 *    to be quoted still comes from a build WITHOUT this.
 *
 * 4. NEVER QUOTE A STALL PERCENTAGE FROM A SUB-SECOND RUN. The same workload
 *    read 33.6% over a quarter second and 12.9% over several, because the
 *    denominator does not cover the same span as the counter baseline on a
 *    window that short. The report prints accumulated totals for exactly this
 *    reason; the per-slice rows are for watching, not for quoting.
 *
 * 5. FRAME TIME IS A BAD RULER ON THIS MACHINE. Output is vblank-quantised
 *    (there is no step between 66.8 ms and 50 ms) and `.text` layout alone
 *    moves it +-6.5 ms. That is why this measures cycles and events rather
 *    than milliseconds: a 9% instruction cut is 1.65 ms and invisible against
 *    that band, which is how three real changes were each recorded as "no
 *    gain".
 *
 * OFF BY DEFAULT -- `-DWITH_PROF=ON`. The macros vanish when it is off.
 */

#ifndef BLOOM_PROF_H
#define BLOOM_PROF_H

#include <stdint.h>

#include "bloom-config.h"

/* THE BUCKETS. Exclusive, and every one names a real choke point in this tree.
 * Order matters only for the report; the nesting is a property of where the
 * brackets sit, not of this list. */
typedef enum {
	PROF_GUEST,     /* everything NOT bracketed: emitted code, the GTE's
			 * asm if it is ever used, the dispatcher. Attributed
			 * by exclusion on purpose -- a bucket that names
			 * itself is a bucket that can quietly absorb the
			 * renderer. */
	PROF_COMPILE,   /* lightrec: decode, optimise, emit, icache sync */
	PROF_GTE,       /* cop2_op and the gte* handlers it dispatches to */
	PROF_IO,        /* the C half of a guest MMIO access: lightrec_rw and
			 * the hw_* shims, plus whatever handler they reach */
	PROF_GPUFRONT,  /* INSIDE io: do_cmd_list -- GP0 decode and the DMA
			 * list walk, reached from a DMA register write, so it
			 * is inside io and never beside it */
	PROF_RENDER,    /* INSIDE gpufront: building PVR lists */
	PROF_FLIP,      /* INSIDE render: pvr_scene_finish and the waiting.
			 * Waiting is not work -- see the note on prof_ns. */
	PROF_EVT,       /* psxEvents / gen_interupt: the tick between block
			 * dispatches, and the vblank work hanging off it */
	PROF_DISC,      /* the GD-ROM / image read */
	PROF_N
} prof_bucket;

#ifndef WITH_PROF
#define WITH_PROF 0
#endif

#if defined(__sh__) && WITH_PROF

void prof_start(void);          /* configure and start PRFC0/PRFC1 */
void prof_calibrate(void);      /* price the instrument, then zero */
uint32_t prof_pair_ns(void);
void prof_reset(void);
void prof_enter(prof_bucket b);
void prof_leave(void);

/* WALL TIME IN A BUCKET IS NOT THE SAME AS COST, and `flip` is the row where
 * the difference decides what to do next: time spent waiting on the PVR is
 * either the SH-4 standing still with work left, or it is slack on a vblank
 * that was going to be missed anyway and cannot be reclaimed by making
 * anything faster. A timer cannot tell those apart. Read `flip` with that in
 * mind and do not add it to a total of "things to optimise". */
uint64_t prof_ns(prof_bucket b);
uint32_t prof_calls(prof_bucket b);

/* PRFC0 through the same brackets as the time. Paired with the cycles the
 * bucket ran for, so the two make a CPI for that region and nothing else. */
uint64_t prof_events(prof_bucket b);
uint64_t prof_bucket_cycles(prof_bucket b);

/* The three-event rotation. Call prof_pmu_rotate() once a second. */
void prof_pmu_rotate(void);
uint64_t prof_pmu_events(unsigned i);
uint64_t prof_pmu_cycles(unsigned i);
const char *prof_pmu_label(unsigned i);
unsigned prof_pmu_count(void);
unsigned prof_pmu_current(void);

uint64_t prof_total_ns(void);
unsigned prof_broken(void);
const char *prof_name(prof_bucket b);
uint32_t prof_cycles(void);

/* One line per bucket plus the three event rows, to stdout. */
void prof_report(void);

/* Call once per frame. Reports after WITH_PROF_FRAMES frames and stops. A fixed
 * frame count, not a wall-clock window: same amount of emulated machine every
 * run, so ms/frame is comparable between builds. The three events rotate at the
 * thirds of the run. */
void prof_frame(void);
int prof_done(void);

#else

#define prof_start()            ((void)0)
#define prof_calibrate()        ((void)0)
#define prof_pair_ns()          0u
#define prof_reset()            ((void)0)
#define prof_enter(b)           ((void)0)
#define prof_leave()            ((void)0)
#define prof_ns(b)              0u
#define prof_calls(b)           0u
#define prof_events(b)          0u
#define prof_bucket_cycles(b)   0u
#define prof_pmu_rotate()       ((void)0)
#define prof_pmu_events(i)      0u
#define prof_pmu_cycles(i)      0u
#define prof_pmu_label(i)       "off"
#define prof_pmu_count()        0u
#define prof_pmu_current()      0u
#define prof_total_ns()         0u
#define prof_broken()           0u
#define prof_name(b)            "off"
#define prof_cycles()           0u
#define prof_report()           ((void)0)
#define prof_frame()            ((void)0)
#define prof_done()             0

#endif

#endif /* BLOOM_PROF_H */
