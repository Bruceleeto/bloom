/*
 * emitmiss_calib.c -- what does PRFC0 mode 0x08 actually count?
 *
 * Drop into bloom (src/), call emitmiss_calib() once after KOS is up (before
 * the emulator starts is fine).  It generates K stubs of `bra next; nop`, one
 * per 32-byte icache line, ends the chain with `rts; nop`, and calls the chain
 * PASSES times with the icache-miss counter running.  It prints, per K:
 *
 *   CALIB K=<lines> miss/pass=<x> fill_cyc/pass=<y> cyc/miss=<z>
 *
 * Expected if one counted miss == one 32-byte line fill and the icache is
 * 8 KB direct-mapped as modelled:
 *   K <= 256 : miss/pass ~ 0 after the first pass (everything fits)
 *   K =  512 : miss/pass ~ 512 (every line evicted by its alias before reuse)
 *   K = 1024 : miss/pass ~ 1024
 * If K<=256 shows ~K misses a pass, a taken branch to another line misses
 * regardless of residency and the model is wrong about entries hitting.
 * If K=1024 shows ~2K or more, one fill is counted as several misses.
 * cyc/miss for K=1024 is the true cost of a line fill from SDRAM.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <malloc.h>
#include <arch/cache.h>
#include <arch/irq.h>
#include <dc/perfctr.h>

#define PASSES 2000

static void run_k(unsigned K)
{
	uint16_t *code = memalign(32, K * 32 + 32);
	unsigned i;
	uint64_t miss, fill;
	void (*fn)(void);
	int old;

	if (!code) { printf("CALIB K=%u alloc failed\n", K); return; }
	for (i = 0; i < K; i++) {
		uint16_t *s = code + i * 16;
		unsigned j;
		for (j = 0; j < 16; j++) s[j] = 0x0009;   /* nop */
		if (i + 1 < K) {
			s[0] = 0xA000 | 14;                    /* bra +32 : disp=(32-4)/2 */
		} else {
			s[0] = 0x000B;                         /* rts */
		}
		s[1] = 0x0009;                             /* delay slot nop */
	}
	dcache_wback_range((uintptr_t)code, K * 32 + 32);
	icache_sync_range((uintptr_t)code, K * 32 + 32);
	fn = (void (*)(void))code;

	fn();                                          /* warm */

	old = irq_disable();
	perf_cntr_stop(PRFC0); perf_cntr_stop(PRFC1);
	perf_cntr_clear(PRFC0); perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC0, PMCR_INSTRUCTION_CACHE_MISS_MODE, PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, PMCR_INSTRUCTION_CACHE_FILL_MODE, PMCR_COUNT_CPU_CYCLES);
	for (i = 0; i < PASSES; i++)
		fn();
	perf_cntr_stop(PRFC0); perf_cntr_stop(PRFC1);
	miss = perf_cntr_count(PRFC0);
	fill = perf_cntr_count(PRFC1);
	irq_restore(old);

	printf("CALIB K=%u miss/pass=%.2f fill_cyc/pass=%.1f cyc/miss=%.1f\n",
	       K, (double)miss / PASSES, (double)fill / PASSES,
	       miss ? (double)fill / miss : 0.0);
	free(code);
}

void emitmiss_calib(void)
{
	static const unsigned ks[] = { 4, 64, 200, 256, 300, 512, 1024, 4096 };
	unsigned i;
	for (i = 0; i < sizeof ks / sizeof ks[0]; i++)
		run_k(ks[i]);
}
