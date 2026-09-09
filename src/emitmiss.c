#include "emitmiss.h"

#if defined(BLOOM_EMITMISS) && defined(__sh__)

#if defined(BLOOM_PERFCMP)
#error "BLOOM_EMITMISS and BLOOM_PERFCMP both drive PRFC0/PRFC1; build one."
#endif

#include <stdio.h>
#include <dc/perfctr.h>
#include <kos/timer.h>

/* SIX WINDOWS: THREE PAIRS, EACH RUN GATED AND UNGATED.
 *
 * There are two counters and the question needs twelve numbers, so the window
 * is run six times.  Three event pairs:
 *
 *      pair 0   what missed          imiss, omiss     (counts)
 *      pair 1   what that cost       istall, dstall   (cycles frozen)
 *      pair 2   the denominator      cycles, issued   (cycles, count)
 *
 * and each pair twice -- once GATED, paused at every door out of emitted code
 * so it prices the recompiler alone, and once UNGATED, free-running so it
 * prices the whole frame.  That pairing is the entire point.  A gated number
 * on its own says nothing about share, and subtracting it from a number taken
 * in some other build and some other run is not subtraction, it is guessing:
 * the gating itself costs a pause/resume at ~1300 doors a frame, which moves
 * the frame time and pollutes the very icache being measured.  Gated and
 * ungated taken in the SAME build, the SAME boot and adjacent windows are the
 * only two numbers here that may legitimately be divided into each other.
 *
 * The digest at the end does that division and prints the answer, so nothing
 * downstream has to do arithmetic across runs. */
static const struct {
	perf_cntr_event_t ev0, ev1;
	const char *n0, *n1;
	char cyc0, cyc1;
} pairs[] = {
	{ PMCR_INSTRUCTION_CACHE_MISS_MODE,
	  PMCR_OPERAND_CACHE_MISS_MODE,
	  "imiss",  "omiss",  0, 0 },
	{ PMCR_PIPELINE_FREEZE_BY_ICACHE_MISS_MODE,
	  PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE,
	  "istall", "dstall", 1, 1 },
	{ PMCR_ELAPSED_TIME_MODE,
	  PMCR_INSTRUCTION_ISSUED_MODE,
	  "cycles", "issued", 1, 0 },
};
#define NB_PAIRS (sizeof(pairs) / sizeof(pairs[0]))

/* The link table, so the dispatcher question can be answered from a build
 * that does not also carry the wall-clock brackets.  Declared rather than
 * included for the same reason platform.c declares them. */
extern unsigned fgl_link_patched, fgl_link_range, fgl_link_full;

/* Laps run pair 0 gated, pair 0 whole-frame, pair 1 gated, ... so a pair's
 * two halves are adjacent in time and see the same part of the level. */
#define NB_LAPS         (NB_PAIRS * 2u)
#define LAP_PAIR(l)     ((l) >> 1)
#define LAP_GATED(l)    (((l) & 1u) == 0u)

static int      armed;                  /* 0 before the window, -1 after */
static unsigned skipped;
static unsigned vsyncs;
static unsigned lap;
static int      gated;                  /* is THIS lap paused at the doors */
static uint64_t win_ns;
static uint32_t win_blocks;

/* Every lap's numbers, per frame, kept so the digest can divide them. */
static double   res0[NB_LAPS], res1[NB_LAPS], res_ms[NB_LAPS];
static double   res_disp[NB_LAPS];

/* HOW DEEP OUT OF EMITTED CODE WE ARE.
 *
 * `depth == 0` means the counters are running.  Every door increments on the
 * way out and decrements on the way back, and only the outermost pair touches
 * the hardware -- a hardware write reaches do_cmd_list with the hw bracket
 * already open, so these nest.
 *
 * It starts at 1 because nothing is running emitted code yet when the window
 * opens; `lightrec_execute` takes it to 0 on entry. */
static unsigned depth = 1;

static void emitmiss_run(int on)
{
	if (on) {
		perf_cntr_resume(PRFC0);
		perf_cntr_resume(PRFC1);
	} else {
		perf_cntr_stop(PRFC0);
		perf_cntr_stop(PRFC1);
	}
}

void bloom_emitmiss_resume(void)
{
	if (armed <= 0 || !gated || !depth)
		return;

	if (--depth == 0)
		emitmiss_run(1);
}

void bloom_emitmiss_pause(void)
{
	if (armed <= 0 || !gated)
		return;

	if (depth++ == 0)
		emitmiss_run(0);
}

/* Load `lap`'s pair and open a window on it.  Clear once, then
 * perf_cntr_start (which writes the whole config) -- never clear and then
 * resume; see the header in emitmiss.h.  A GATED lap then stops the counters,
 * so the window begins paused and only emitted code turns it on; an UNGATED
 * lap leaves them running and the doors do not touch them. */
static void emitmiss_arm(void)
{
	const unsigned p = LAP_PAIR(lap);

	gated = LAP_GATED(lap);

	perf_cntr_stop(PRFC0);
	perf_cntr_stop(PRFC1);
	perf_cntr_clear(PRFC0);
	perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC0, pairs[p].ev0, PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, pairs[p].ev1, PMCR_COUNT_CPU_CYCLES);
	if (gated) {
		perf_cntr_stop(PRFC0);
		perf_cntr_stop(PRFC1);
	}

	depth = 1;
	vsyncs = 0;
	win_blocks = fgl_blocks_run;
	win_ns = timer_ns_gettime64();
	armed = 1;
}

/* THE DIGEST: the twelve numbers turned into the four answers.
 *
 * Printed once, after the last lap, because every line of it divides a gated
 * lap by its ungated twin and neither is worth anything alone.  Nothing here
 * is measured -- it is all arithmetic on what the laps already printed, so the
 * raw lines above remain the record and this is only the reading of them. */
static void emitmiss_digest(void)
{
	const double e_imiss = res0[0], e_omiss = res1[0];
	const double w_imiss = res0[1], w_omiss = res1[1];
	const double e_ist = res0[2], e_dst = res1[2];
	const double w_ist = res0[3], w_dst = res1[3];
	const double e_cyc = res0[4], e_iss = res1[4];
	const double w_cyc = res0[5], w_iss = res1[5];
	const double e_stall = e_ist + e_dst, w_stall = w_ist + w_dst;
	const double frame_ms = (res_ms[1] + res_ms[3] + res_ms[5]) / 3.0;
	const double ms = 1.0 / 200e3;          /* cycles -> ms at 200 MHz */

	printf("EMIT ----- digest, per frame, %.2f ms/frame -----\n", frame_ms);

	/* WHERE THE FRAME IS.  `emitted` is elapsed cycles with the counters
	 * gated, so it is time genuinely inside generated code; `rest` is the
	 * renderer, the C shims, the event tick and the compiler thread, and
	 * it carries the gating overhead too, so read it as an upper bound. */
	printf("EMIT  frame  | whole %6.2f ms | emitted %6.2f ms (%4.1f%%)"
	       " | rest %6.2f ms\n",
	       w_cyc * ms, e_cyc * ms,
	       w_cyc > 0.0 ? 100.0 * e_cyc / w_cyc : 0.0,
	       (w_cyc - e_cyc) * ms);

	/* CACHE HELL, SPLIT.  The line the whole build exists for: how many of
	 * the frame's frozen cycles are the recompiler's, and how many belong
	 * to everything else. */
	printf("EMIT  stall  | whole %6.2f ms (%4.1f%% of frame)"
	       " | emitted %6.2f ms (%4.1f%% of emitted)"
	       " | rest %6.2f ms\n",
	       w_stall * ms, w_cyc > 0.0 ? 100.0 * w_stall / w_cyc : 0.0,
	       e_stall * ms, e_cyc > 0.0 ? 100.0 * e_stall / e_cyc : 0.0,
	       (w_stall - e_stall) * ms);

	/* AND SPLIT AGAIN BY WHICH CACHE.  Instruction fetch and data are
	 * different bugs with different fixes -- icache says the code is too
	 * big, dcache says the data is laid out wrong -- so they are never
	 * added together anywhere a decision gets made. */
	printf("EMIT  icache | whole %6.2f ms | emitted %6.2f ms (%4.1f%%)"
	       " | miss %6.1fk of %6.1fk | %5.1f cyc/miss\n",
	       w_ist * ms, e_ist * ms,
	       w_ist > 0.0 ? 100.0 * e_ist / w_ist : 0.0,
	       e_imiss / 1e3, w_imiss / 1e3,
	       e_imiss > 0.0 ? e_ist / e_imiss : 0.0);
	printf("EMIT  dcache | whole %6.2f ms | emitted %6.2f ms (%4.1f%%)"
	       " | miss %6.1fk of %6.1fk | %5.1f cyc/miss\n",
	       w_dst * ms, e_dst * ms,
	       w_dst > 0.0 ? 100.0 * e_dst / w_dst : 0.0,
	       e_omiss / 1e3, w_omiss / 1e3,
	       e_omiss > 0.0 ? e_dst / e_omiss : 0.0);

	/* ISSUE.  CPI for emitted code alone against CPI for the frame: if the
	 * emitted figure is the worse of the two then the recompiler is the
	 * stalled part and not merely the large part, which is a different
	 * claim and the one that decides whether layout work is worth it. */
	printf("EMIT  issue  | whole %7.0fk CPI %4.2f"
	       " | emitted %7.0fk CPI %4.2f (%4.1f%% of insns)\n",
	       w_iss / 1e3, w_iss > 0.0 ? w_cyc / w_iss : 0.0,
	       e_iss / 1e3, e_iss > 0.0 ? e_cyc / e_iss : 0.0,
	       w_iss > 0.0 ? 100.0 * e_iss / w_iss : 0.0);

	/* UNACCOUNTED.  Elapsed minus issued minus the two cache freezes is
	 * every other reason the pipe stopped -- taken branches, FPU latency,
	 * the store queue.  It is printed rather than dropped because a big
	 * one means the two caches are NOT the story and the next counter to
	 * load is a different one. */
	printf("EMIT  other  | emitted %6.2f ms | whole %6.2f ms"
	       "  (elapsed - issued - both stalls)\n",
	       (e_cyc - e_iss - e_stall) * ms,
	       (w_cyc - w_iss - w_stall) * ms);

	/* THE DISPATCHER.  `disp` is arrivals at `.Lrun`: exits that could NOT
	 * be linked, since a patched link branches straight into its successor
	 * and never passes that label.  `full` is edges refused for want of
	 * table -- a zero there means FGL_MAX_LINKS is not the constraint and
	 * raising it buys nothing, which is worth knowing before anyone
	 * spends a day on it. */
	printf("EMIT  disp   | %.0f entries/frame | links near %u far %u"
	       " (%u patched) full %u\n",
	       res_disp[5], fgl_link_patched, fgl_link_range,
	       fgl_link_patched + fgl_link_range, fgl_link_full);
}

void bloom_emitmiss_vsync(void)
{
	uint64_t c0, c1, ns;
	unsigned p, f;

	if (armed < 0)
		return;

	if (!armed) {
		if (skipped++ < EMITMISS_SKIP)
			return;         /* still warming up */

		emitmiss_arm();
		return;
	}

	if (++vsyncs < EMITMISS_VSYNCS)
		return;

	perf_cntr_stop(PRFC0);
	perf_cntr_stop(PRFC1);

	c0 = perf_cntr_count(PRFC0);
	c1 = perf_cntr_count(PRFC1);
	ns = timer_ns_gettime64() - win_ns;
	f  = vsyncs ? vsyncs : 1;
	p  = LAP_PAIR(lap);

	res0[lap]    = (double)c0 / f;
	res1[lap]    = (double)c1 / f;
	res_ms[lap]  = (double)ns / f / 1e6;
	res_disp[lap] = (double)(fgl_blocks_run - win_blocks) / f;

	/* PER FRAME, because that is the unit every other number in this
	 * emulator is quoted in.  A cycle event also gets milliseconds at the
	 * SH-4's 200 MHz, so it can be laid beside a wall-clock bracket. */
	printf("EMIT lap %u %-8s fr %u | %s/fr %8.1fk", lap,
	       gated ? "emitted" : "whole", f, pairs[p].n0, res0[lap] / 1e3);
	if (pairs[p].cyc0)
		printf(" (%5.2f ms)", res0[lap] / 200e3);

	printf(" | %s/fr %8.1fk", pairs[p].n1, res1[lap] / 1e3);
	if (pairs[p].cyc1)
		printf(" (%5.2f ms)", res1[lap] / 200e3);

	printf(" | disp/fr %.0f | frame %5.2f ms | fps %u.%02u\n",
	       res_disp[lap], res_ms[lap],
	       (unsigned)(vsyncs * 1000000000ull / (ns ? ns : 1)),
	       (unsigned)((vsyncs * 100000000000ull / (ns ? ns : 1)) % 100u));

	if (++lap == NB_LAPS) {
		emitmiss_digest();
		armed = -1;             /* every lap priced; stand down */
		return;
	}

	emitmiss_arm();
}

#endif
