/* See bench.h for what this is measuring and why the perf report cannot. */
#include <stdio.h>
#include <stdint.h>

#include <arch/timer.h>

#include "bench.h"

unsigned int fgl_bench_calls, fgl_bench_cycle;

void fgl_bench_vsync(unsigned int guest_cycle)
{
	static uint64_t t0;
	static uint32_t c0, cprev, jump;
	static uint32_t b0, bprev;
	static uint64_t msprev;
	static unsigned int vsyncs, beat, done;
	uint64_t now, ms;

	if (!FGL_BENCH || done)
		return;

	fgl_bench_calls++;
	fgl_bench_cycle = guest_cycle;

	now = timer_ms_gettime64();

	/* THE START LINE IS NOT THE FIRST VSYNC.
	 *
	 * It used to be, and one run cost 2.18 BILLION guest cycles in a single
	 * vsync around the 180th -- the savestate load and the first compile
	 * burst draining out of the owed pool in one settle.  That lump sits
	 * inside `guest_cycle - c0` for the rest of the run and made cyc/vsync
	 * read 1756160 against a true 564480.  So the window opens well after
	 * the game is actually running. */
	if (++vsyncs < FGL_BENCH_WARMUP)
		return;

	if (vsyncs == FGL_BENCH_WARMUP) {
		t0 = now;
		c0 = guest_cycle;
		cprev = guest_cycle;
		b0 = bprev = fgl_blocks_run;
		msprev = 0;
		printf("bench: start at vsync %u, %u to go\n",
		       vsyncs, FGL_BENCH_VSYNCS);
		return;
	}

	/* AND THE WINDOW SAYS WHEN IT HAS BEEN LIED TO.  A warm-up cannot rule
	 * out a lump, it only makes one unlikely; `jump` is the largest single
	 * vsync inside the window, so a total that is off can be spotted for
	 * what it is instead of being theorised about. */
	{
		uint32_t d = guest_cycle - cprev;

		cprev = guest_cycle;
		if (d > jump)
			jump = d;
	}

	ms = now - t0;

	/* The heartbeat, so there is something to watch and something to exit
	 * on if the run wanders somewhere it should not. */
	if (ms >= (uint64_t)(beat + 10) * 1000u) {
		/* THE SPEEDOMETER.  vsync/s is a setting -- 60 over the
		 * delivered stretch -- so it says nothing about which build is
		 * faster.  Guest blocks per wall second does: it is not derived
		 * from the guest clock and it does not move when a perf bracket
		 * moves.  Both interval and cumulative, because the interval
		 * rate shows a scene change for what it is instead of letting
		 * it quietly bias the total. */
		uint32_t b = fgl_blocks_run;
		uint64_t dms = ms - msprev;

		beat += 10;
		printf("bench: %3u s | %u/%u vsync | %u blk/s (avg %u)\n",
		       beat, vsyncs - FGL_BENCH_WARMUP, FGL_BENCH_VSYNCS,
		       dms ? (unsigned)((uint64_t)(b - bprev) * 1000u / dms) : 0,
		       ms ? (unsigned)((uint64_t)(b - b0) * 1000u / ms) : 0);
		bprev = b;
		msprev = ms;
	}

	if (vsyncs - FGL_BENCH_WARMUP < FGL_BENCH_VSYNCS)
		return;

	done = 1;
	printf("bench: DONE %u vsync | wall %u ms | guest cycles %u"
	       " | %u cyc/vsync | %u vsync/wall-s | biggest vsync %u"
	       " | blocks %u | %u blk/wall-s\n",
	       FGL_BENCH_VSYNCS, (unsigned)ms, guest_cycle - c0,
	       (guest_cycle - c0) / FGL_BENCH_VSYNCS,
	       ms ? (unsigned)((uint64_t)FGL_BENCH_VSYNCS * 1000u / ms) : 0,
	       jump, fgl_blocks_run - b0,
	       ms ? (unsigned)((uint64_t)(fgl_blocks_run - b0) * 1000u / ms) : 0);
}
