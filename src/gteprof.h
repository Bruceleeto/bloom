/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Timing for the GTE, on the SH-4 performance counter.
 *
 * It brackets the call the CPU core makes, not a particular implementation,
 * so a WITH_FPU_GTE build and a stock one report the same two numbers about
 * the same boundary and can be compared directly.
 *
 * Copyright (C) 2026 bloom contributors
 */

#ifndef BLOOM_GTEPROF_H
#define BLOOM_GTEPROF_H

#include <stdint.h>

#include <dc/perfctr.h>

/*
 * PRFC1: KOS claims PRFC0 at init for its nanosecond timer.  Elapsed time in
 * CPU cycles, free-running from gteprof_init, which is both the per-call clock
 * and the denominator a share is taken against.
 */
#define GTEPROF_CNTR	PRFC1

void gteprof_init(void);

/* Called with the cycles one GTE command took, and its function field. */
void gteprof_account(uint64_t cycles, uint32_t fn);

static inline uint64_t gteprof_now(void)
{
	return perf_cntr_count(GTEPROF_CNTR);
}

#define GTEPROF_ENTER	uint64_t gteprof_t0 = gteprof_now()
#define GTEPROF_LEAVE(fn) \
	gteprof_account(gteprof_now() - gteprof_t0, (fn))

#endif /* BLOOM_GTEPROF_H */
