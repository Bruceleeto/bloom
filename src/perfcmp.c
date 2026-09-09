/* THE HEAD-TO-HEAD AGAINST GNU LIGHTNING.
 *
 * The same five SH7750 events the lightning tree's src/perf.c rotates
 * through, printed in the same units, so the two logs can be read side by
 * side.  PRFC0 counts elapsed cycles for the whole window; PRFC1 takes one
 * event per window and moves on, so five windows price a frame completely:
 *
 *      stalls (freeze_icache + freeze_dcache) + issue = elapsed
 *
 * WHAT THIS IS FOR.  build/docs/issue_not_cache.md claims bloom is LESS
 * stalled than the lightning build it replaced, not more -- 40% of cycles
 * against 62% -- and that the deficit is issue.  That was assembled from two
 * runs taken in different places.  Both trees load the same savestate
 * (romdisk/spyro.sav.gz == broesph romdisk/bench.sav.gz, identical md5), so
 * running both and comparing the same lap settles it.
 *
 * NO AREA ATTRIBUTION.  The lightning tree splits every window by area with
 * perf_area_switch() calls threaded through its core.  Nothing here needs
 * that: the question is what the whole frame does, and the totals are what
 * the two builds have in common.
 *
 * THE KOS perf_cntr_clear BUG.  KOS asserts PMCR_CLR and never drops it, and
 * perf_cntr_resume only ORs in PMCR_RUN -- so clear-then-resume leaves the
 * counter running with CLR still asserted and reads near zero.  Clear once,
 * then perf_cntr_start, which writes the full config.  Never resume a counter
 * that was cleared.
 */
#include <stdio.h>
#include <stdint.h>
#include <dc/perfctr.h>

#include "perfcmp.h"

#if BLOOM_PERFCMP

static const struct { perf_cntr_event_t ev; const char *name; char cycles; } events[] = {
	{ PMCR_INSTRUCTION_ISSUED_MODE,              "instr_issued",     0 },
	{ PMCR_PARALLEL_INSTRUCTION_ISSUED_MODE,     "dual_issued",      0 },
	{ PMCR_PIPELINE_FREEZE_BY_ICACHE_MISS_MODE,  "freeze_icache",    1 },
	{ PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE,  "freeze_dcache",    1 },
	{ PMCR_INSTRUCTION_CACHE_MISS_MODE,          "icache_miss",      0 },
};
#define NB_EVENTS (sizeof(events) / sizeof(events[0]))

static unsigned cur_ev, lap;
static int inited;

static void perfcmp_init(void)
{
	perf_cntr_timer_disable();
	perf_cntr_clear(PRFC0);
	perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC0, PMCR_ELAPSED_TIME_MODE, PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, events[0].ev, PMCR_COUNT_CPU_CYCLES);
	inited = 1;
}

void perfcmp_report(unsigned frames)
{
	uint64_t tc, te;

	if (!inited) {
		perfcmp_init();
		return;                 /* first window is partial; discard */
	}
	if (!frames)
		frames = 1;

	tc = perf_cntr_count(PRFC0);
	te = perf_cntr_count(PRFC1);

	printf("PERF %-14s lap %u fr %u | cyc/fr %7.3fM (%5.2f ms) | %s/fr %8.1fk%s\n",
	       events[cur_ev].name, lap, frames,
	       (double)tc / frames / 1e6, (double)tc / frames / 200e3,
	       events[cur_ev].cycles ? "cyc" : "cnt",
	       (double)te / frames / 1e3,
	       events[cur_ev].cycles ? " (cycles)" : "");

	if (++cur_ev == NB_EVENTS) {
		cur_ev = 0;
		lap++;
	}

	/* Both counters restart together so the window they price is the
	 * same window.  Clear then start -- never resume; see the header. */
	perf_cntr_stop(PRFC0);
	perf_cntr_stop(PRFC1);
	perf_cntr_clear(PRFC0);
	perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC0, PMCR_ELAPSED_TIME_MODE, PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, events[cur_ev].ev, PMCR_COUNT_CPU_CYCLES);
}

#endif /* BLOOM_PERFCMP */
