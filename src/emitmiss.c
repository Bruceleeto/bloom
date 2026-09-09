#include "emitmiss.h"

#if defined(BLOOM_EMITMISS) && defined(__sh__)

#include <stdio.h>
#include <dc/perfctr.h>
#include <kos/timer.h>

static int      armed;                  /* 0 before the window, -1 after */
static unsigned skipped;
static unsigned vsyncs;
static uint64_t win_ns;
static uint32_t win_blocks;

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
	if (armed <= 0 || !depth)
		return;

	if (--depth == 0)
		emitmiss_run(1);
}

void bloom_emitmiss_pause(void)
{
	if (armed <= 0)
		return;

	if (depth++ == 0)
		emitmiss_run(0);
}

void bloom_emitmiss_vsync(void)
{
	uint64_t c0, c1, ns;

	if (armed < 0)
		return;

	if (!armed) {
		if (skipped++ < EMITMISS_SKIP)
			return;         /* still warming up */

		/* Arm once the JIT is warm.  Clear once, start once --
		 * see the header on why clear must not be paired with resume
		 * later -- then stop, so the window begins paused and only
		 * emitted code turns it on. */
		perf_cntr_stop(PRFC0);
		perf_cntr_stop(PRFC1);
		perf_cntr_clear(PRFC0);
		perf_cntr_clear(PRFC1);
		perf_cntr_start(PRFC0, PMCR_INSTRUCTION_CACHE_MISS_MODE,
				PMCR_COUNT_CPU_CYCLES);
		perf_cntr_start(PRFC1, PMCR_OPERAND_CACHE_MISS_MODE,
				PMCR_COUNT_CPU_CYCLES);
		perf_cntr_stop(PRFC0);
		perf_cntr_stop(PRFC1);

		depth = 1;
		vsyncs = 0;
		win_blocks = fgl_blocks_run;
		win_ns = timer_ns_gettime64();
		armed = 1;
		return;
	}

	if (++vsyncs < EMITMISS_VSYNCS)
		return;

	perf_cntr_stop(PRFC0);
	perf_cntr_stop(PRFC1);

	c0 = perf_cntr_count(PRFC0);
	c1 = perf_cntr_count(PRFC1);
	ns = timer_ns_gettime64() - win_ns;

	printf("PERF emitted imiss=%llu omiss=%llu blocks=%u"
	       " (dispatcher entries only) vsyncs=%u..%u fps=%u.%02u"
	       " elapsed_ns=%llu\n",
	       (unsigned long long)c0, (unsigned long long)c1,
	       fgl_blocks_run - win_blocks,
	       (unsigned)EMITMISS_SKIP, (unsigned)EMITMISS_SKIP + vsyncs,
	       (unsigned)(vsyncs * 1000000000ull / (ns ? ns : 1)),
	       (unsigned)((vsyncs * 100000000000ull / (ns ? ns : 1)) % 100u),
	       (unsigned long long)ns);

	armed = -1;
}

#endif
