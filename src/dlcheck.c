/* DOES PRFC1 COUNT WHAT IT CLAIMS TO?
 *
 * The performance counter is the only meter on this chip that measures work
 * rather than time, which is exactly what the deadline needs -- and the
 * standing objection to it is that nothing else in the system reads it, so
 * nothing ever contradicts it if it is wrong.  This removes that objection or
 * confirms it.
 *
 * What the deadline needs is not a counter that reports some particular
 * number per instruction; it needs one whose reading is PROPORTIONAL to work
 * done, because the charge multiplies it by a fitted K anyway.  So this does
 * not assert a count.  It runs the same assembly loop over three iteration
 * counts spaced equally apart and requires the two counter differences to
 * match: equal work must produce equal counts, wherever in the run it falls.
 * Everything constant -- the call, the counter reads, the loop setup -- cancels
 * inside each difference, so there is no fudge term to fit.
 *
 * The measured slope is reported rather than checked.  It is not nine per
 * iteration even though the loop body is nine instructions, because the SH-4
 * issues two instructions per cycle where it can and PMCR_INSTRUCTION_ISSUED
 * counts issue slots.  That is fine, and it is why asserting a count here was
 * wrong: the slot rate is a property of the code being measured, and K absorbs
 * it.  A fail is worth more than a pass -- it says the one hardware work meter
 * on this chip is not usable, and the deadline needs a different answer. */

#include <stdio.h>
#include <dc/perfctr.h>

#include "dlcheck.h"

static void dl_spin(unsigned int iters)
{
	unsigned int n = iters;

	/* `bf` has no delay slot, so an iteration is exactly these nine. */
	__asm__ __volatile__(
		"1:\n\t"
		"nop\n\tnop\n\tnop\n\tnop\n\t"
		"nop\n\tnop\n\tnop\n\t"
		"dt %0\n\t"
		"bf 1b\n\t"
		: "+r" (n) : : "t");
}

static unsigned long long dl_span(unsigned int iters)
{
	unsigned long long t0, t1;

	t0 = perf_cntr_count(PRFC1);
	dl_spin(iters);
	t1 = perf_cntr_count(PRFC1);

	return t1 - t0;
}

int fgl_dl_check_prfc1(void)
{
	unsigned long long c0, c1, c2, d1, d2;
	unsigned int step = 200000u;
	long long err;
	int ok;

	perf_cntr_start(PRFC1, PMCR_INSTRUCTION_ISSUED_MODE, PMCR_COUNT_CPU_CYCLES);

	c0 = dl_span(step);
	c1 = dl_span(step * 2u);
	c2 = dl_span(step * 3u);

	/* Two differences over identical extra work.  Constants cancel twice. */
	d1 = c1 - c0;
	d2 = c2 - c1;

	err = (long long)d2 - (long long)d1;
	ok  = d1 && err > -(long long)(d1 / 100) && err < (long long)(d1 / 100);

	printf("prfc1: spans %llu %llu %llu | equal-work deltas %llu %llu"
	       " err %lld | slope %llu/1024 per iter (%s)\n",
	       c0, c1, c2, d1, d2, err,
	       (unsigned long long)((d1 << 10) / step),
	       ok ? "LINEAR, trusted" : "NOT LINEAR, do not use");

	return ok;
}
