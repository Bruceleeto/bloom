// SPDX-License-Identifier: GPL-2.0-only
/*
 * Timing for the GTE.
 *
 * The question this exists to answer is what a GTE command costs and what
 * share of the machine that is, in a form that survives being compared
 * between two builds.  Frame rate cannot answer it: the pacer quantises to
 * divisors of 60 Hz, so at 12 fps the next value up is 15 and a tenth of the
 * machine disappears into the rounding.
 *
 * Two numbers are reported per command and one for the whole window:
 *
 *   cycles per call   what the command costs, probe cost removed
 *   share             GTE cycles over elapsed cycles in the window
 *
 * The window is wall time on the same counter, so the share is a share of
 * everything the machine did - drawing, CD, the recompiler - and not of some
 * subset that moves when the implementation does.
 *
 * Copyright (C) 2026 bloom contributors
 */

#include <stdio.h>

#include "gteprof.h"

/* Report roughly every five seconds of a 200 MHz clock. */
#define GTEPROF_WINDOW	(5ull * 200000000ull)

static const char *const gteprof_names[64] = {
	[0x01] = "RTPS",  [0x06] = "NCLIP", [0x0c] = "OP",    [0x10] = "DPCS",
	[0x11] = "INTPL", [0x12] = "MVMVA", [0x13] = "NCDS",  [0x14] = "CDP",
	[0x16] = "NCDT",  [0x1b] = "NCCS",  [0x1c] = "CC",    [0x1e] = "NCS",
	[0x20] = "NCT",   [0x28] = "SQR",   [0x29] = "DCPL",  [0x2a] = "DPCT",
	[0x2d] = "AVSZ3", [0x2e] = "AVSZ4", [0x30] = "RTPT",  [0x3d] = "GPF",
	[0x3e] = "GPL",   [0x3f] = "NCCT",
};

static uint64_t cmd_cyc[64];
static uint32_t cmd_calls[64];
static uint64_t total_cyc;
static uint64_t total_calls;
static uint64_t window_base;

/*
 * What one bracket costs, so it can be taken back off.
 *
 * Two counter reads per call against a command in the hundreds of cycles is a
 * few percent, and it is the same few percent in both builds - but a
 * cycles-per-call figure that silently includes it is wrong in a way nobody
 * reading the number would suspect.
 */
static uint64_t probe_cyc;

static void gteprof_calibrate(void)
{
	const unsigned n = 1024;
	uint64_t t0, t1;
	unsigned i;

	t0 = gteprof_now();
	for (i = 0; i < n; i++) {
		uint64_t inner = gteprof_now();

		(void)inner;
	}
	t1 = gteprof_now();

	/* One read for the enter, one for the leave. */
	probe_cyc = 2 * (t1 - t0) / n;
}

void gteprof_init(void)
{
	perf_cntr_stop(GTEPROF_CNTR);
	perf_cntr_clear(GTEPROF_CNTR);
	perf_cntr_start(GTEPROF_CNTR, PMCR_ELAPSED_TIME_MODE,
			PMCR_COUNT_CPU_CYCLES);

	gteprof_calibrate();

	window_base = gteprof_now();

	printf("gteprof: counting on PRFC1, bracket costs %u cycles\n",
	       (unsigned)probe_cyc);
}

static void gteprof_report(uint64_t now)
{
	uint64_t window = now - window_base;
	unsigned i;

	printf("gteprof: %u Mcyc window, GTE %u.%02u%% of it,"
	       " %u calls, %u cyc/call\n",
	       (unsigned)(window / 1000000ull),
	       (unsigned)(total_cyc * 100ull / window),
	       (unsigned)(total_cyc * 10000ull / window % 100ull),
	       (unsigned)total_calls,
	       total_calls ? (unsigned)(total_cyc / total_calls) : 0);

	for (i = 0; i < 64; i++) {
		if (!cmd_calls[i])
			continue;

		printf("gteprof:   %-5s %6u calls, %5u cyc/call, %2u.%02u%%\n",
		       gteprof_names[i] ? gteprof_names[i] : "?",
		       (unsigned)cmd_calls[i],
		       (unsigned)(cmd_cyc[i] / cmd_calls[i]),
		       (unsigned)(cmd_cyc[i] * 100ull / window),
		       (unsigned)(cmd_cyc[i] * 10000ull / window % 100ull));

		cmd_cyc[i] = 0;
		cmd_calls[i] = 0;
	}

	total_cyc = 0;
	total_calls = 0;
	window_base = now;
}

void gteprof_account(uint64_t cycles, uint32_t fn)
{
	uint64_t now;

	cycles = cycles > probe_cyc ? cycles - probe_cyc : 0;

	fn &= 0x3f;
	cmd_cyc[fn] += cycles;
	cmd_calls[fn]++;
	total_cyc += cycles;
	total_calls++;

	/*
	 * The window is checked here rather than on a timer because this is
	 * the only place that already has the counter in a register.  A title
	 * that stops issuing GTE commands stops reporting, which is correct:
	 * there is nothing to report about.
	 */
	now = gteprof_now();

	if (now - window_base >= GTEPROF_WINDOW)
		gteprof_report(now);
}
