#include <stdio.h>
#include <stdint.h>
#include <dc/perfctr.h>
#include "perf.h"

static const struct { perf_cntr_event_t ev; const char *name; char cycles; } events[] = {
	{ PMCR_INSTRUCTION_ISSUED_MODE,              "instr_issued",     0 },
	{ PMCR_INSTRUCTION_CACHE_MISS_MODE,          "icache_miss",      0 },
	{ PMCR_PIPELINE_FREEZE_BY_ICACHE_MISS_MODE,  "freeze_icache",    1 },
	{ PMCR_OPERAND_CACHE_MISS_MODE,              "dcache_miss",      0 },
	{ PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE,  "freeze_dcache",    1 },
	{ PMCR_INSTRUCTION_CACHE_FILL_MODE,          "icache_fill",      1 },
	{ PMCR_OPERAND_CACHE_FILL_MODE,              "dcache_fill",      1 },
	{ PMCR_PIPELINE_FREEZE_BY_BRANCH_MODE,       "freeze_branch",    1 },
	{ PMCR_PIPELINE_FREEZE_BY_CPU_REGISTER_MODE, "freeze_reg",       1 },
	{ PMCR_PIPELINE_FREEZE_BY_FPU_MODE,          "freeze_fpu",       1 },
	{ PMCR_BRANCH_TAKEN_MODE,                    "branch_taken",     0 },
	{ PMCR_BRANCH_ISSUED_MODE,                   "branch_issued",    0 },
	{ PMCR_PARALLEL_INSTRUCTION_ISSUED_MODE,     "dual_issued",      0 },
	{ PMCR_FPU_INSTRUCTION_ISSUED_MODE,          "fpu_issued",       0 },
	{ PMCR_OPERAND_ACCESS_MODE,                  "operand_access",   0 },
	{ PMCR_OPERAND_CACHE_READ_MISS_MODE,         "dcache_rd_miss",   0 },
	{ PMCR_OPERAND_CACHE_WRITE_MISS_MODE,        "dcache_wr_miss",   0 },
	{ PMCR_INSTRUCTION_FETCH_MODE,               "instr_fetch",      0 },
	{ PMCR_ON_CHIP_IO_ACCESS_MODE,               "onchip_io",        0 },
	{ PMCR_UTLB_MISS_MODE,                       "utlb_miss",        0 },
	{ PMCR_INTERRUPT_COUNTER_MODE,               "interrupts",       0 },
};
#define NB_EVENTS (sizeof(events) / sizeof(events[0]))

static const char *area_names[PERF_NB_AREAS] = { "rest", "gpu", "flip", "cd", "gp1", "jit", "hw", "spu" };

/* lightrec.c: blocks compiled so far, to see whether steady state is steady. */
extern unsigned int lightrec_perf_nb_compile;
extern unsigned int lightrec_perf_nb_execute;
extern unsigned int lightning_perf_pairs;
extern unsigned int lightning_perf_singles;
static unsigned int last_nb_compile;
static unsigned int last_nb_execute;

static uint64_t cyc[PERF_NB_AREAS], ev[PERF_NB_AREAS];
static uint64_t last_cyc, last_ev;
static enum perf_area cur;
static unsigned int cur_ev, lap;
static int inited;

static void perf_init(void)
{
	perf_cntr_timer_disable();
	perf_cntr_clear(PRFC0);
	perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC0, PMCR_ELAPSED_TIME_MODE, PMCR_COUNT_CPU_CYCLES);
	perf_cntr_start(PRFC1, events[0].ev, PMCR_COUNT_CPU_CYCLES);
	last_cyc = perf_cntr_count(PRFC0);
	last_ev = perf_cntr_count(PRFC1);
	inited = 1;
}

static inline void perf_accumulate(void)
{
	uint64_t c = perf_cntr_count(PRFC0), e = perf_cntr_count(PRFC1);

	cyc[cur] += c - last_cyc;
	ev[cur] += e - last_ev;
	last_cyc = c;
	last_ev = e;
}

enum perf_area perf_area_switch(enum perf_area area)
{
	enum perf_area prev = cur;

	if (!inited)
		perf_init();

	perf_accumulate();
	cur = area;

	return prev;
}

void perf_report(unsigned int frames, unsigned int ms)
{
	uint64_t tc = 0, te = 0;
	unsigned int i;

	if (!inited)
		perf_init();

	perf_accumulate();

	for (i = 0; i < PERF_NB_AREAS; i++) {
		tc += cyc[i];
		te += ev[i];
	}
	if (!frames)
		frames = 1;

	/* PERF <event> lap N frames F comp C | cyc/fr total (ms) <area>... | <cnt|cyc>/fr total <area>... */
	printf("PERF %-14s lap %u fr %u comp %u exec/fr %u pair %u%% | cyc/fr %7.3fM (%5.2f ms)",
	       events[cur_ev].name, lap, frames,
	       lightrec_perf_nb_compile - last_nb_compile,
	       (lightrec_perf_nb_execute - last_nb_execute) / frames,
	       lightning_perf_pairs + lightning_perf_singles
	       ? 200 * lightning_perf_pairs
		 / (2 * lightning_perf_pairs + lightning_perf_singles) : 0,
	       (double)tc / frames / 1e6, (double)tc / frames / 200e3);
	for (i = 0; i < PERF_NB_AREAS; i++)
		printf(" %s %6.3fM", area_names[i], (double)cyc[i] / frames / 1e6);
	printf(" | %s/fr %8.1fk", events[cur_ev].cycles ? "cyc" : "cnt",
	       (double)te / frames / 1e3);
	for (i = 0; i < PERF_NB_AREAS; i++)
		printf(" %s %8.1fk", area_names[i], (double)ev[i] / frames / 1e3);
	printf("%s\n", events[cur_ev].cycles ? " (cycles)" : "");
	last_nb_compile = lightrec_perf_nb_compile;
	last_nb_execute = lightrec_perf_nb_execute;
	(void)ms;

	for (i = 0; i < PERF_NB_AREAS; i++)
		cyc[i] = ev[i] = 0;

	/* Next event */
	cur_ev++;
	if (cur_ev == NB_EVENTS) {
		cur_ev = 0;
		lap++;
	}
	perf_cntr_stop(PRFC1);
	perf_cntr_clear(PRFC1);
	perf_cntr_start(PRFC1, events[cur_ev].ev, PMCR_COUNT_CPU_CYCLES);
	last_ev = perf_cntr_count(PRFC1);
	last_cyc = perf_cntr_count(PRFC0);
}
