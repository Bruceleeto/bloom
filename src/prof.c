/* See prof.h. Ported from bloop's src/prof.c; the bucket set is bloom's. */

#include "prof.h"

#if defined(__sh__) && WITH_PROF

#include <stdio.h>
#include <arch/arch.h>
#include <dc/perfctr.h>

/* PRFC1's low half, read directly rather than through perf_cntr_count(). The
 * API's reader takes the high word twice to catch a carry out of the low one,
 * which is three on-chip accesses where one will do: every use here is a
 * difference between two reads, and unsigned 32-bit arithmetic is already
 * correct across the wrap so long as no single bracketed region runs for the
 * 21 seconds it takes to get there. None does.
 *
 * KOS's PMCTR_LOW(PRFCn) worked out: 0xff100008 stepping by eight. */
#define PMCTR0_LOW  (*(volatile uint32_t *)0xff100008u)
#define PMCTR1_LOW  (*(volatile uint32_t *)0xff100010u)

/* THREE EVENTS, ONE RUN. Note the mode numbers: issued is 0x13 and the
 * pipeline-freeze modes are 0x24/0x25. 0x18 and 0x19 are TRAPA and UBC match,
 * not caches -- that mistake has been made in this project already. */
#define PMU_N 3

static const unsigned char pmu_mode[PMU_N] = {
	PMCR_INSTRUCTION_ISSUED_MODE,
	PMCR_PIPELINE_FREEZE_BY_ICACHE_MISS_MODE,
	PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE
};

static const char *const pmu_label[PMU_N] = {
	"issue slots", "icache freeze", "dcache freeze"
};

static uint64_t pmu_ev[PMU_N];
static uint64_t pmu_cyc[PMU_N];
static unsigned pmu_cur;
static uint64_t pmu_ev_base, pmu_cyc_base;

/* 200 MHz, one count per cycle in elapsed-time mode. */
#define NS_PER_COUNT 5u

#define PROF_DEPTH 8

static uint64_t total[PROF_N];
static uint64_t event[PROF_N];
static uint32_t calls[PROF_N];

static uint32_t last, ev_last;
static uint8_t  stack[PROF_DEPTH];
static uint8_t  sp;
static unsigned over;           /* stack overflowed this many times */
static unsigned broken;

void prof_reset(void)
{
	unsigned i;

	for (i = 0; i < PROF_N; i++) {
		total[i] = 0;
		event[i] = 0;
		calls[i] = 0;
	}
	for (i = 0; i < PMU_N; i++) {
		pmu_ev[i] = 0;
		pmu_cyc[i] = 0;
	}
	pmu_ev_base = 0;
	pmu_cyc_base = 0;
	over = 0;
	broken = 0;
	last = PMCTR1_LOW;
	ev_last = PMCTR0_LOW;

	/* The bracket stack is deliberately NOT touched. This is called from the
	 * flip, which runs inside a GPU command, so io/gpufront are on the stack
	 * and their leaves are still coming -- zeroing sp here would make every
	 * one of them a leave-with-no-enter and set broken. */
}

void prof_start(void)
{
	sp = 0;
	stack[0] = PROF_GUEST;
	perf_cntr_start(PRFC0, pmu_mode[0], PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, PMCR_ELAPSED_TIME_MODE, PMCR_COUNT_CPU_CYCLES);
	pmu_cur = 0;
	prof_reset();
}

/* WHAT THE INSTRUMENT ITSELF COSTS, MEASURED RATHER THAN ASSUMED. Each bracket
 * is two on-chip counter reads, and an on-chip access goes off the CPU's
 * internal bus -- it is not a register move. Time N empty pairs, report ns
 * each, and let the report subtract pair_ns * calls from every bucket. The
 * buckets are zeroed afterwards so the calibration does not appear in the run
 * it prices. The loop and call overhead are inside the figure, which is
 * correct: that is what a bracket costs at a call site. */
#define PROF_CAL_PAIRS 65536u

static uint32_t pair_ns;

uint32_t prof_pair_ns(void)
{
	return pair_ns;
}

void prof_calibrate(void)
{
	uint32_t t0, t1, i;

	t0 = PMCTR1_LOW;

	for (i = 0; i < PROF_CAL_PAIRS; i++) {
		prof_enter(PROF_GUEST);
		prof_leave();
	}

	t1 = PMCTR1_LOW;

	pair_ns = (uint32_t)(((uint64_t)(t1 - t0) * NS_PER_COUNT)
			     / PROF_CAL_PAIRS);

	prof_reset();
}

void prof_enter(prof_bucket b)
{
	uint32_t now = PMCTR1_LOW;
	uint32_t evn = PMCTR0_LOW;

	total[stack[sp]] += now - last;
	event[stack[sp]] += evn - ev_last;
	last = now;
	ev_last = evn;

	/* Overflowing the stack is a bug in the bracketing, not a condition to
	 * recover from -- but the run keeps its other counters, so the region
	 * is charged to whatever is already on top and the matching leave is
	 * told to pop nothing. Balance survives; only the attribution is
	 * wrong, and prof_broken() says so. */
	if (sp + 1 >= PROF_DEPTH) {
		over++;
		broken++;
		return;
	}
	stack[++sp] = (uint8_t)b;
	calls[b]++;
}

void prof_leave(void)
{
	uint32_t now = PMCTR1_LOW;
	uint32_t evn = PMCTR0_LOW;

	total[stack[sp]] += now - last;
	event[stack[sp]] += evn - ev_last;
	last = now;
	ev_last = evn;

	if (over)
		over--;
	else if (sp)
		sp--;
	else
		broken++;               /* a leave with no enter */
}

uint64_t prof_ns(prof_bucket b)         { return total[b] * NS_PER_COUNT; }
uint32_t prof_calls(prof_bucket b)      { return calls[b]; }
uint64_t prof_events(prof_bucket b)     { return event[b]; }
uint64_t prof_bucket_cycles(prof_bucket b) { return total[b]; }
uint32_t prof_cycles(void)              { return PMCTR1_LOW; }

/* CALLED ONCE A SECOND. Closes the current mode's share of the guest region,
 * then hands PRFC0 to the next event. Starting a counter zeroes it, so the
 * baseline is re-read after the switch. */
void prof_pmu_rotate(void)
{
	uint64_t ev = event[PROF_GUEST];
	uint64_t cyc = total[PROF_GUEST];

	pmu_ev[pmu_cur] += ev - pmu_ev_base;
	pmu_cyc[pmu_cur] += cyc - pmu_cyc_base;
	pmu_ev_base = ev;
	pmu_cyc_base = cyc;

	pmu_cur = (pmu_cur + 1u) % PMU_N;
	perf_cntr_stop(PRFC0);
	perf_cntr_start(PRFC0, pmu_mode[pmu_cur], PMCR_COUNT_CPU_CYCLES);
	ev_last = PMCTR0_LOW;
}

/* The current slice never sees a rotation, so it is closed here instead. */
uint64_t prof_pmu_events(unsigned i)
{
	if (i == pmu_cur)
		return pmu_ev[i] + (event[PROF_GUEST] - pmu_ev_base);
	return i < PMU_N ? pmu_ev[i] : 0;
}

uint64_t prof_pmu_cycles(unsigned i)
{
	if (i == pmu_cur)
		return pmu_cyc[i] + (total[PROF_GUEST] - pmu_cyc_base);
	return i < PMU_N ? pmu_cyc[i] : 0;
}

const char *prof_pmu_label(unsigned i) { return i < PMU_N ? pmu_label[i] : "?"; }
unsigned prof_pmu_count(void)          { return PMU_N; }
unsigned prof_pmu_current(void)        { return pmu_cur; }

uint64_t prof_total_ns(void)
{
	uint64_t sum = 0;
	unsigned i;

	for (i = 0; i < PROF_N; i++)
		sum += total[i];
	return sum * NS_PER_COUNT;
}

unsigned prof_broken(void)
{
	/* sp != 0 is NOT broken here: the report is printed from the flip, which
	 * runs inside a GPU command, so io/gpufront are legitimately still on
	 * the stack. Only a leave with no enter, or a stack overflow, is a bug. */
	return broken;
}

const char *prof_name(prof_bucket b)
{
	static const char *const names[PROF_N] = {
		"guest", "compile", "gte", "io", "gpufront", "render",
		"flip", "evt", "disc"
	};

	return b < PROF_N ? names[b] : "?";
}

/* The report. Buckets first, with the instrument's own cost subtracted, then
 * the three event rows as ACCUMULATED totals -- never a single slice, because a
 * sub-second stall percentage is not a stall percentage. */
void prof_report(void)
{
	uint64_t sum = prof_total_ns();
	unsigned i;

	if (!sum)
		return;

	printf("prof: %llu ms accounted, bracket %u ns%s\n",
	       (unsigned long long)(sum / 1000000u), (unsigned)pair_ns,
	       prof_broken() ? "  *** BRACKETS BROKEN, rows are wrong ***" : "");

	for (i = 0; i < PROF_N; i++) {
		uint64_t ns = prof_ns((prof_bucket)i);
		uint64_t cost = (uint64_t)pair_ns * calls[i];
		uint64_t net = ns > cost ? ns - cost : 0;
		uint64_t cyc = total[i];

		if (!calls[i] && i != PROF_GUEST)
			continue;

		printf("  %-9s %7llu ms  %5.1f%%  %9u calls  ev/cyc %.3f\n",
		       prof_name((prof_bucket)i),
		       (unsigned long long)(net / 1000000u),
		       100.0 * (double)net / (double)sum,
		       (unsigned)calls[i],
		       cyc ? (double)event[i] / (double)cyc : 0.0);
	}

	for (i = 0; i < PMU_N; i++) {
		uint64_t ev = prof_pmu_events(i);
		uint64_t cyc = prof_pmu_cycles(i);

		if (!cyc)
			continue;

		/* Issue slots are a rate against cycles; the freeze modes are a
		 * share of cycles, which is the number that means something. */
		printf("  guest %-14s %12llu over %12llu cyc = %5.2f%s\n",
		       pmu_label[i], (unsigned long long)ev,
		       (unsigned long long)cyc,
		       (double)ev / (double)cyc,
		       i ? " (fraction of cycles frozen)" : " per cycle");
	}
}

/* Frames discarded before counting starts, so the compile burst that follows
 * the savestate load stays out of the run. */
#define PROF_WARMUP 60u

static uint32_t ruler_frames;
static uint32_t warm_frames;
static int ruler_done;

int prof_done(void)
{
	return ruler_done;
}

void prof_frame(void)
{
	if (ruler_done)
		return;

	/* Warm up first, then zero everything. Without this the run carries the
	 * compile burst that follows the savestate load, which is both large and
	 * different every time. */
	if (warm_frames < PROF_WARMUP) {
		if (++warm_frames == PROF_WARMUP)
			prof_reset();
		return;
	}

	ruler_frames++;

	/* Rotate EVERY frame, so each event samples every third frame across the
	 * whole scene. Rotating at the thirds gave each event one contiguous
	 * third instead, and the three thirds are not the same workload -- that
	 * showed up as a 10% swing in the freeze fractions between identical
	 * runs. */
	prof_pmu_rotate();

	if (ruler_frames < WITH_PROF_FRAMES)
		return;

	printf("prof: %u frames, %llu ms, %.3f ms/frame\n",
	       (unsigned)ruler_frames,
	       (unsigned long long)(prof_total_ns() / 1000000u),
	       (double)prof_total_ns() / 1000000.0 / (double)ruler_frames);

	prof_report();

	ruler_done = 1;
	arch_exit();
}

#endif /* __sh__ && WITH_PROF */
