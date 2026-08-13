/* See prof.h. */

#include "prof.h"

static const char * const prof_names[PROF_N] = {
	[PROF_GUEST]		= "GUEST",
	[PROF_GTE]		= "GTE",
	[PROF_MEMCLASS]		= "MEMCLASS",
	[PROF_IO]		= "IO",
	[PROF_GPUFRONT]		= "GPUFRONT",
	[PROF_RENDER]		= "RENDER",
	[PROF_FLIP]		= "FLIP",
	[PROF_COMPILE]		= "COMPILE",
	[PROF_INVALIDATE]	= "INVALIDATE",
	[PROF_CD]		= "CD",
	[PROF_EVENTS]		= "EVENTS",
	[PROF_SPU]		= "SPU",
};

const char *prof_name(prof_bucket b)
{
	if ((unsigned int)b >= PROF_N)
		return "?";

	return prof_names[b];
}

#ifdef BLOOM_PROF

#include <stdio.h>

#include <dc/perfctr.h>
#include <kos/thread.h>

/* PRFC1's low half, read directly rather than through perf_cntr_count().  The
 * API's reader takes the high word twice to catch a carry out of the low one,
 * which is three on-chip accesses where one will do: every use here is a
 * difference between two reads, and unsigned 32-bit arithmetic is already
 * correct across the wrap so long as no single bracketed region runs for the
 * 21 seconds it takes to get there.  None does.
 *
 * KOS's own PMCTR_LOW(o) is *((volatile uint32_t *)0xff100008 + (o << 1)),
 * i.e. 0xff100008 stepping by eight per counter, so PRFC1 is 0xff100010
 * (perfctr.c:15). */
#define PMCTR1_LOW	(*(volatile uint32_t *)0xff100010u)

/* PRFC0's low half, same stepping (perfctr.c:15).  KOS uses this counter for
 * perf_cntr_timer_ns(); a STALL build takes it over, so nothing here may call
 * that function. */
#define PMCTR0_LOW	(*(volatile uint32_t *)0xff100008u)

/* 200 MHz, one count per cycle in elapsed-time mode (perfctr.c:29). */
#define NS_PER_COUNT	5u

#define PROF_DEPTH	8

static uint64_t total[PROF_N];
static uint32_t calls[PROF_N];

static uint32_t last;			/* when the current region started */
static uint8_t stack[PROF_DEPTH];
static unsigned int sp;			/* stack[sp] is what is being charged */
static unsigned int over;		/* enters the stack had no room for */
static unsigned int broken;
static int running;

/* The thread that owns the stack — see prof.h.  Read as a plain pointer
 * compare, which is what makes the guard affordable on every bracket. */
static kthread_t *owner;

/* OFF-THREAD TIME — the compiler worker.
 *
 * PRFC1 measures elapsed time, not per-thread time, so when lightrec's worker
 * preempts the emulator its cost lands in whatever bucket happened to be on
 * top of the stack — almost always GUEST.  A bracket taken on that thread
 * cannot go on the shared stack (it would interleave), so it goes here
 * instead: a flat start/stop with its own depth counter, one bucket at a time.
 *
 * These milliseconds are ALSO counted in the table above.  That is not a bug —
 * it is the only way to price a thread the wall clock cannot separate.  The
 * report says so, and the subtraction is left to the reader. */
static uint64_t other_ns[PROF_N];
static uint32_t other_calls[PROF_N];
static uint32_t other_last;
static unsigned int other_depth;
static uint8_t other_bucket;

static uint64_t cpu0_all;

#ifdef BLOOM_STALL
/* See the note in prof.h.  `ev` is whatever PRFC0 was configured to count;
 * `cyc` is elapsed cycles from PRFC1.  The `_guest` pair accumulates only
 * across stretches where the bracket stack was empty. */
static uint64_t ev_guest, cyc_guest;
static uint32_t ev_mark, cyc_mark;	/* when the current guest stretch began */
static uint32_t ev_run0, cyc_run0;	/* whole-run baselines */

static const char *stall_event_name(void)
{
	switch (BLOOM_STALL) {
	case 1:  return "icache freeze (cycles)";
	case 2:  return "dcache freeze (cycles)";
	case 3:  return "instructions issued";
	case 4:  return "instructions issued in parallel";
	default: return "?";
	}
}

static perf_cntr_event_t stall_event(void)
{
	switch (BLOOM_STALL) {
	case 1:  return PMCR_PIPELINE_FREEZE_BY_ICACHE_MISS_MODE;
	case 2:  return PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE;
	case 3:  return PMCR_INSTRUCTION_ISSUED_MODE;
	default: return PMCR_PARALLEL_INSTRUCTION_ISSUED_MODE;
	}
}

/* Close the open guest stretch, if the stack was empty. */
static inline void stall_guest_close(void)
{
	ev_guest  += (uint32_t)(PMCTR0_LOW - ev_mark);
	cyc_guest += (uint32_t)(PMCTR1_LOW - cyc_mark);
}

static inline void stall_guest_open(void)
{
	ev_mark  = PMCTR0_LOW;
	cyc_mark = PMCTR1_LOW;
}
#else
#define stall_guest_close()	((void)0)
#define stall_guest_open()	((void)0)
#endif

void prof_reset(void)
{
	unsigned int i;

	for (i = 0; i < PROF_N; i++) {
		total[i] = 0;
		calls[i] = 0;
	}

	sp = 0;
	stack[0] = PROF_GUEST;
	over = 0;
	broken = 0;

	for (i = 0; i < PROF_N; i++) {
		other_ns[i] = 0;
		other_calls[i] = 0;
	}
	other_depth = 0;

#ifdef BLOOM_STALL
	ev_guest = cyc_guest = 0;
	ev_run0  = PMCTR0_LOW;
	cyc_run0 = PMCTR1_LOW;
	stall_guest_open();
#else
	cpu0_all = perf_cntr_timer_ns();
#endif

	last = PMCTR1_LOW;
}

/* Called ON THE EMULATOR THREAD, before it starts running the guest.
 *
 * The benchmark arms the profiler from its own watcher thread, so prof_start()
 * cannot claim the owner — it would claim the watcher, and every bracket on
 * the emulator thread would then be ignored as if it came from the compiler
 * worker.  That is exactly what happened the first time this ran: a full
 * 125-second window with every bucket reading zero. */
void prof_claim(void)
{
	owner = thd_get_current();
}

void prof_start(void)
{
	if (running)
		return;

	perf_cntr_start(PRFC1, PMCR_ELAPSED_TIME_MODE, PMCR_COUNT_CPU_CYCLES);

#ifdef BLOOM_STALL
	/* PRFC0 is KOS's nanosecond timer by default; a STALL build takes it,
	 * so prof_report must not call perf_cntr_timer_ns() below. */
	perf_cntr_start(PRFC0, stall_event(), PMCR_COUNT_CPU_CYCLES);
#endif

	running = 1;
	prof_reset();

	printf("PROF: armed, owner thread %p, PRFC1 elapsed-time, %u ns/count%s\n",
	       (void *)owner, NS_PER_COUNT,
	       owner ? "" : "  ** NO OWNER — prof_claim() was never called **");
	fflush(stdout);
}

/* WHAT THE INSTRUMENT ITSELF COSTS, MEASURED RATHER THAN ASSUMED.
 *
 * Each bracket is two on-chip counter reads plus a thread compare, and an
 * on-chip access goes off the CPU's internal bus rather than being a register
 * move.  Multiplied by the bracket count that is not small: the first
 * measured window here was 18% slower than bare, and the GTE carried 46% of
 * that because it is the most frequently bracketed thing in the run.  A
 * bucket billed for its own measurement is not a finding.
 *
 * So: time N empty pairs, and let the report subtract pair_ns * calls from
 * every bucket.  The buckets are zeroed afterwards so the calibration does not
 * appear in the run it prices.  Loop and call overhead are inside the figure,
 * which is right — that is what a bracket costs at a call site. */
#define PROF_CAL_PAIRS 65536u

static uint32_t pair_ns;

uint32_t prof_pair_ns(void)
{
	return pair_ns;
}

void prof_calibrate(void)
{
	uint32_t t0, t1, i;

	if (!running)
		return;

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
	uint32_t now;

	if (!running)
		return;

	if (thd_get_current() != owner) {
		if (!other_depth++) {
			other_bucket = (uint8_t)b;
			other_calls[b]++;
			other_last = PMCTR1_LOW;
		}
		return;
	}

	now = PMCTR1_LOW;
	total[stack[sp]] += (uint64_t)(now - last) * NS_PER_COUNT;
	last = now;

	calls[b]++;

	/* Leaving generated code: close the open stretch. */
	if (!sp)
		stall_guest_close();

	if (sp + 1 >= PROF_DEPTH) {
		over++;
		broken = 1;
		return;
	}

	stack[++sp] = (uint8_t)b;
}

void prof_leave(void)
{
	uint32_t now;

	if (!running)
		return;

	if (thd_get_current() != owner) {
		if (other_depth && !--other_depth)
			other_ns[other_bucket] +=
				(uint64_t)(PMCTR1_LOW - other_last)
				* NS_PER_COUNT;
		return;
	}

	now = PMCTR1_LOW;
	total[stack[sp]] += (uint64_t)(now - last) * NS_PER_COUNT;
	last = now;

	/* Unwind the enters the stack had no room for before popping a real
	 * one, or the depths go out of step and every bucket after is wrong. */
	if (over) {
		over--;
		return;
	}

	if (!sp) {
		broken = 1;
		return;
	}

	sp--;

	/* Back in generated code: open a new stretch. */
	if (!sp)
		stall_guest_open();
}

uint64_t prof_ns(prof_bucket b)
{
	return (unsigned int)b < PROF_N ? total[b] : 0;
}

uint32_t prof_calls(prof_bucket b)
{
	return (unsigned int)b < PROF_N ? calls[b] : 0;
}

uint64_t prof_total_ns(void)
{
	uint64_t sum = 0;
	unsigned int i;

	for (i = 0; i < PROF_N; i++)
		sum += total[i];

	return sum;
}

unsigned int prof_broken(void)
{
	return broken;
}

/* Tenths of a percent — printed as pct/10 "." pct%10. */
static uint32_t pct(uint64_t n, uint64_t of)
{
	if (!of)
		return 0;

	return (uint32_t)((n * 1000 + of / 2) / of);
}

void prof_report(uint32_t wall_ms, uint32_t frames, uint64_t guest_insns)
{
	uint64_t acct, ns, all_ms, other_sum = 0;
	unsigned int i;

	if (!running)
		return;

	acct = prof_total_ns();

#ifdef BLOOM_STALL
	all_ms = 0;
	(void)cpu0_all;
#else
	all_ms = (perf_cntr_timer_ns() - cpu0_all) / 1000000u;
#endif

	for (i = 0; i < PROF_N; i++)
		other_sum += other_ns[i];

	printf("PROF: %lu ms accounted of %lu ms wall (%lu.%lu%%)%s\n",
	       (unsigned long)(acct / 1000000u), (unsigned long)wall_ms,
	       (unsigned long)(pct(acct / 1000000u, wall_ms) / 10),
	       (unsigned long)(pct(acct / 1000000u, wall_ms) % 10),
	       prof_broken() ? "  ** BROKEN, see below **" : "");

	{
		uint64_t inst = 0;

		for (i = 0; i < PROF_N; i++)
			inst += (uint64_t)pair_ns * calls[i];

		printf("PROF: instrument %lu ns/bracket — %lu ms of the %lu ms"
		       " above is this profiler\n",
		       (unsigned long)pair_ns,
		       (unsigned long)(inst / 1000000u),
		       (unsigned long)(acct / 1000000u));
	}

	printf("PROF: %-10s %10s %6s %10s %10s %10s\n",
	       "bucket", "ms", "%", "calls", "net ms", "us/frame");

	for (i = 0; i < PROF_N; i++) {
		uint64_t own, net;

		ns = total[i];
		own = (uint64_t)pair_ns * calls[i];
		net = ns > own ? ns - own : 0;

		printf("PROF: %-10s %10lu %3lu.%lu %10lu %10lu %10lu\n",
		       prof_name(i),
		       (unsigned long)(ns / 1000000u),
		       (unsigned long)(pct(ns, acct) / 10),
		       (unsigned long)(pct(ns, acct) % 10),
		       (unsigned long)calls[i],
		       (unsigned long)(net / 1000000u),
		       (unsigned long)(frames ? (net / 1000u) / frames : 0));
	}

	if (other_sum) {
		printf("PROF: off-thread (compiler worker) %lu ms —"
		       " ALSO counted in the buckets above, subtract from"
		       " whichever was on top:\n",
		       (unsigned long)(other_sum / 1000000u));

		for (i = 0; i < PROF_N; i++) {
			if (!other_ns[i])
				continue;

			printf("PROF:   %-10s %10lu ms %10lu calls\n",
			       prof_name(i),
			       (unsigned long)(other_ns[i] / 1000000u),
			       (unsigned long)other_calls[i]);
		}
	} else {
		printf("PROF: off-thread 0 ms — nothing bracketed ran off the"
		       " owner thread\n");
	}

	(void)all_ms;

#ifdef BLOOM_STALL
	{
		/* THE RUN'S CYCLES COME FROM THE WALL CLOCK, NOT THE COUNTER.
		 *
		 * PMCTR is 48 bits and only its low word is read here.  That is
		 * correct for the short deltas the accumulators are built from,
		 * and wrong for a whole run: 136 seconds at 199.5 MHz is 2.7e10
		 * cycles, which wraps 32 bits six times.  The first STALL run
		 * printed "1111.8% of the run's cycles" for exactly that
		 * reason.  Wall milliseconds cannot wrap. */
		uint64_t cyc_all = (uint64_t)wall_ms * 199500u;

		/* Close the stretch that is open right now, or the last one
		 * before the report is silently dropped. */
		stall_guest_close();

		printf("STALL: PRFC0 = %s\n", stall_event_name());
		printf("STALL: generated   %llu events / %llu cycles"
		       "  (%lu.%lu%% of the run's %llu)\n",
		       (unsigned long long)ev_guest,
		       (unsigned long long)cyc_guest,
		       (unsigned long)(pct(cyc_guest, cyc_all) / 10),
		       (unsigned long)(pct(cyc_guest, cyc_all) % 10),
		       (unsigned long long)cyc_all);

		(void)ev_run0;
		(void)cyc_run0;

		if (guest_insns && ev_guest)
			printf("STALL: generated   %lu.%02lu events per guest"
			       " instruction\n",
			       (unsigned long)(ev_guest / guest_insns),
			       (unsigned long)((ev_guest * 100 / guest_insns)
					       % 100));

		if (cyc_guest)
			printf("STALL: generated   %lu.%02lu events per cycle"
			       "  (IPC if PRFC0 counts issue)\n",
			       (unsigned long)(ev_guest / cyc_guest),
			       (unsigned long)((ev_guest * 100 / cyc_guest)
					       % 100));

		if (guest_insns)
			printf("STALL: generated   %lu.%02lu host cycles per"
			       " guest instruction (parity is 5.89)\n",
			       (unsigned long)(cyc_guest / guest_insns),
			       (unsigned long)((cyc_guest * 100 / guest_insns)
					       % 100));
	}
#endif

	if (guest_insns)
		printf("PROF: %llu guest instructions, %lu ns each accounted\n",
		       (unsigned long long)guest_insns,
		       (unsigned long)(acct / guest_insns));

	if (prof_broken())
		printf("PROF: BROKEN — unbalanced bracket or nesting past"
		       " depth %u; the numbers above are wrong\n", PROF_DEPTH);

	fflush(stdout);
}

#endif /* BLOOM_PROF */
