/* See jitprof.h. */

#ifdef JITPROF

#include <stdio.h>
#include <string.h>

#include <arch/irq.h>
#include <arch/timer.h>
#include <kos/irq.h>

#include "jitprof.h"

/* 2 kHz over a 60 s window is ~120k samples: a bucket holding 0.1% of the
 * time is still ~120 samples, and the handler is a masked compare plus two
 * increments. */
#define JITPROF_HZ		2000

/* 256-byte granularity on both sides.  Fine enough that a reported address
 * resolves to one function under addr2line, coarse enough to keep the
 * counter arrays in BSS. */
#define JITPROF_GRAIN_SHIFT	8

#define JITPROF_JIT_BUCKETS	4096	/* 1 MiB of emitted code */
#define JITPROF_HOST_BUCKETS	12288	/* 3 MiB from the load address */

#define JITPROF_TOP		24

/* KOS loads at a fixed address and bloom's text is ~1.2 MiB, so 3 MiB from
 * the load base covers .text and then some.  Deriving this from
 * __executable_start / _etext looked tidier but those are PROVIDE'd symbols
 * that do not survive this build's LTO link.  The start line prints the
 * range so it can be checked against dcload's section list. */
#define JITPROF_TEXT_BASE	0x8c000000u
#define JITPROF_TEXT_LEN \
	((uint32_t)JITPROF_HOST_BUCKETS << JITPROF_GRAIN_SHIFT)

extern void *code_buffer;

/* Set by lightrec_execute() around the call into the dispatcher, so a sample
 * landing in host C can be attributed to "called from generated code" rather
 * than to the frontend, the plugins or the CD.  This is the split the bench
 * line could never make: its "% in guest code" counts the whole dispatcher
 * call, C wrappers included. */
volatile int jitprof_in_jit;

static uint32_t jp_jit_b[JITPROF_JIT_BUCKETS];
static uint32_t jp_hostjit_b[JITPROF_HOST_BUCKETS];
static uint32_t jp_hostother_b[JITPROF_HOST_BUCKETS];

static uint32_t jp_total;
static uint32_t jp_jit, jp_tramp, jp_hostjit, jp_hostother, jp_elsewhere;

static uintptr_t jp_code_base, jp_text_base;
static uint32_t jp_text_len;
static int jp_running;

static void jitprof_irq(irq_t code, irq_context_t *ctx, void *data)
{
	uintptr_t pc = ctx->pc & 0x1fffffff;
	uintptr_t off;
	int in_jit = jitprof_in_jit;

	(void)code;
	(void)data;

	timer_clear(TMU2);

	jp_total++;

	off = pc - jp_code_base;
	if (off < (uintptr_t)JITPROF_JIT_BUCKETS << JITPROF_GRAIN_SHIFT) {
		jp_jit++;
		/* The dispatcher and the RW wrapper are emitted before any
		 * game block, so they own the bottom of the buffer. */
		if (off < 4096)
			jp_tramp++;
		jp_jit_b[off >> JITPROF_GRAIN_SHIFT]++;
		return;
	}

	off = pc - jp_text_base;
	if (off < jp_text_len) {
		off >>= JITPROF_GRAIN_SHIFT;
		if (in_jit) {
			jp_hostjit++;
			jp_hostjit_b[off]++;
		} else {
			jp_hostother++;
			jp_hostother_b[off]++;
		}
		return;
	}

	jp_elsewhere++;
}

void jitprof_start(void)
{
	if (jp_running || !code_buffer)
		return;

	jp_code_base = (uintptr_t)code_buffer & 0x1fffffff;
	jp_text_base = JITPROF_TEXT_BASE & 0x1fffffff;
	jp_text_len = JITPROF_TEXT_LEN;

	memset(jp_jit_b, 0, sizeof(jp_jit_b));
	memset(jp_hostjit_b, 0, sizeof(jp_hostjit_b));
	memset(jp_hostother_b, 0, sizeof(jp_hostother_b));
	jp_total = jp_jit = jp_tramp = 0;
	jp_hostjit = jp_hostother = jp_elsewhere = 0;

	irq_set_handler(EXC_TMU2_TUNI2, jitprof_irq, NULL);
	timer_prime(TMU2, JITPROF_HZ, 1);
	timer_start(TMU2);

	jp_running = 1;

	printf("JITPROF: %d Hz, code 0x%08lx, host 0x%08lx +%lu KiB,"
	       " %u-byte buckets\n",
	       JITPROF_HZ,
	       (unsigned long)(uintptr_t)code_buffer,
	       (unsigned long)JITPROF_TEXT_BASE,
	       (unsigned long)(jp_text_len / 1024),
	       1u << JITPROF_GRAIN_SHIFT);
	fflush(stdout);
}

/* Returns tenths of a percent — the callers print it as pct/10 "." pct%10. */
static uint32_t pct(uint32_t n, uint32_t of)
{
	if (!of)
		return 0;
	return (uint32_t)(((uint64_t)n * 1000 + of / 2) / of);
}

/* Print the heaviest buckets of one histogram, plus a paste-ready addr2line
 * argument list for the same addresses. */
static void jitprof_top(const char *title, const uint32_t *b,
			unsigned int nb, uintptr_t base, uint32_t of,
			int want_cmd)
{
	static uint8_t taken[JITPROF_HOST_BUCKETS];
	uintptr_t addr[JITPROF_TOP];
	unsigned int i, rank, n = 0;
	uint32_t best, cum = 0;
	int best_i;

	if (!of)
		return;

	memset(taken, 0, nb);

	printf("JITPROF: --- %s ---\n", title);

	for (rank = 0; rank < JITPROF_TOP; rank++) {
		best = 0;
		best_i = -1;

		for (i = 0; i < nb; i++) {
			if (!taken[i] && b[i] > best) {
				best = b[i];
				best_i = i;
			}
		}

		if (best_i < 0 || !best)
			break;

		taken[best_i] = 1;
		cum += best;
		/* Counting is done on masked addresses so a sample taken
		 * through any region window lands in the right bucket;
		 * reporting puts the P1 bit back, because that is the address
		 * addr2line and the disassembler expect. */
		addr[n++] = 0x80000000u | (base
			    + ((uintptr_t)best_i << JITPROF_GRAIN_SHIFT));

		printf("JITPROF:   0x%08lx  %7lu  %3lu.%lu%%  cum %3lu.%lu%%\n",
		       (unsigned long)addr[n - 1], (unsigned long)best,
		       (unsigned long)(pct(best, of) / 10),
		       (unsigned long)(pct(best, of) % 10),
		       (unsigned long)(pct(cum, of) / 10),
		       (unsigned long)(pct(cum, of) % 10));
	}

	if (want_cmd && n) {
		printf("JITPROF: addr2line:");
		for (i = 0; i < n; i++)
			printf(" %08lx", (unsigned long)addr[i]);
		printf("\n");
	}
}

void jitprof_report(void)
{
	unsigned int i, live;

	if (!jp_running)
		return;

	timer_stop(TMU2);
	irq_set_handler(EXC_TMU2_TUNI2, NULL, NULL);
	jp_running = 0;

	if (!jp_total) {
		printf("JITPROF: no samples\n");
		return;
	}

	printf("JITPROF: %lu samples\n", (unsigned long)jp_total);
	printf("JITPROF:   emitted code        %3lu.%lu%%"
	       "  (trampolines %lu.%lu%%, blocks %lu.%lu%%)\n",
	       (unsigned long)(pct(jp_jit, jp_total) / 10),
	       (unsigned long)(pct(jp_jit, jp_total) % 10),
	       (unsigned long)(pct(jp_tramp, jp_total) / 10),
	       (unsigned long)(pct(jp_tramp, jp_total) % 10),
	       (unsigned long)(pct(jp_jit - jp_tramp, jp_total) / 10),
	       (unsigned long)(pct(jp_jit - jp_tramp, jp_total) % 10));
	printf("JITPROF:   C called from JIT   %3lu.%lu%%\n",
	       (unsigned long)(pct(jp_hostjit, jp_total) / 10),
	       (unsigned long)(pct(jp_hostjit, jp_total) % 10));
	printf("JITPROF:   other host code     %3lu.%lu%%\n",
	       (unsigned long)(pct(jp_hostother, jp_total) / 10),
	       (unsigned long)(pct(jp_hostother, jp_total) % 10));
	printf("JITPROF:   outside both        %3lu.%lu%%\n",
	       (unsigned long)(pct(jp_elsewhere, jp_total) / 10),
	       (unsigned long)(pct(jp_elsewhere, jp_total) % 10));

	for (i = 0, live = 0; i < JITPROF_JIT_BUCKETS; i++)
		if (jp_jit_b[i])
			live++;
	printf("JITPROF:   %u live emitted buckets of %u bytes\n",
	       live, 1u << JITPROF_GRAIN_SHIFT);

	jitprof_top("C called from JIT (% of that class)",
		    jp_hostjit_b, JITPROF_HOST_BUCKETS, jp_text_base,
		    jp_hostjit, 1);
	jitprof_top("other host code (% of that class)",
		    jp_hostother_b, JITPROF_HOST_BUCKETS, jp_text_base,
		    jp_hostother, 1);
	jitprof_top("emitted code (% of that class)",
		    jp_jit_b, JITPROF_JIT_BUCKETS, jp_code_base,
		    jp_jit, 0);

	fflush(stdout);
}

#endif /* JITPROF */
