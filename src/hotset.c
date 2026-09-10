/* The sampler behind src/hotset.h.  See there for what this is for. */

#include "hotset.h"

#if defined(BLOOM_HOTSET) && defined(__sh__)

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <arch/irq.h>
#include <arch/timer.h>
#include <kos/irq.h>

/* The recompiler's output lives here (src/mmap.c), outside every section the
 * linker knows about, so a sample in it resolves to no symbol.  Counting the
 * range separately is what turns that from a hole in the report into the
 * emitted-vs-native split. */
extern void *code_buffer;

/* ---------------------------------------------------------------- */
/* The table                                                         */
/* ---------------------------------------------------------------- */

/* OPEN ADDRESSING, AND A DROP RATHER THAN A GROW.  The handler runs in
 * interrupt context: it cannot allocate, and it must not walk a chain of
 * unknown length.  So the table is fixed, a probe is eight slots at most, and
 * a sample that finds no room is counted in `lost` and thrown away.  A `lost`
 * that is not near zero means HOTSET_BITS is too small for the program's
 * footprint and every ranking below it is understated. */
#define HOTSET_BITS     12
#define HOTSET_SLOTS    (1u << HOTSET_BITS)
#define HOTSET_PROBE    8

/* 128-byte buckets: four icache lines, small enough to separate two functions
 * and large enough that a 4096-slot table covers 512 KB of distinct code. */
#define HOTSET_GRAIN    128

static uint32_t slot_key[HOTSET_SLOTS];         /* bucket address, 0 = empty */
static uint32_t slot_hits[HOTSET_SLOTS];

static uint32_t total, jit_hits, lost;
static uint32_t cb_lo, cb_hi;

static int state;                               /* 0 idle, 1 running, -1 done */
static unsigned frames;

static inline uint32_t hotset_hash(uint32_t k)
{
	/* Knuth's multiplicative, taking the high bits: consecutive buckets
	 * land far apart, which is what keeps a hot run of code from piling
	 * into one probe chain. */
	return (k * 2654435761u) >> (32 - HOTSET_BITS);
}

static void hotset_handler(irq_t code, irq_context_t *ctx, void *data)
{
	uint32_t pc, key, h, i;

	(void)code;
	(void)data;

	/* The underflow flag has to go before the next tick or the interrupt
	 * re-enters immediately. */
	timer_clear(TMU2);

	if (state != 1)
		return;

	/* P1/P2 alias to the same physical code, so everything below works in
	 * physical addresses: a sample taken through the uncached window then
	 * lands in the cached window's bucket instead of a second one. */
	pc = (uint32_t)CONTEXT_PC(*ctx) & 0x1fffffffu;
	total++;

	/* Emitted code carries no symbols, so it is one number rather than a
	 * thousand anonymous buckets competing with the C for table room. */
	if (pc >= cb_lo && pc < cb_hi) {
		jit_hits++;
		return;
	}

	key = pc & ~(uint32_t)(HOTSET_GRAIN - 1);
	if (!key)
		key = HOTSET_GRAIN;             /* 0 means empty, so never 0 */

	h = hotset_hash(key);
	for (i = 0; i < HOTSET_PROBE; i++, h = (h + 1) & (HOTSET_SLOTS - 1)) {
		if (slot_key[h] == key) {
			slot_hits[h]++;
			return;
		}
		if (!slot_key[h]) {
			slot_key[h] = key;
			slot_hits[h] = 1;
			return;
		}
	}

	lost++;
}

/* ---------------------------------------------------------------- */
/* The report                                                        */
/* ---------------------------------------------------------------- */

/* SELECTION, NOT A SORT.  HOTSET_REPORT passes over 4096 slots is cheaper to
 * write than a sort and runs once, off the frame.  Each pass takes the largest
 * count still below the previous one; ties are broken by address so a pair of
 * equal buckets does not print twice. */
static void hotset_report(void)
{
	uint32_t prev_hits = 0xffffffffu, prev_key = 0;
	unsigned n, i;

	printf("HOT  window %u frames | %u samples at %u Hz"
	       " | emitted %u (%u.%u%%) | native %u | lost %u\n",
	       frames, total, HOTSET_HZ, jit_hits,
	       total ? 100u * jit_hits / total : 0u,
	       total ? (1000u * jit_hits / total) % 10u : 0u,
	       total - jit_hits - lost, lost);
	printf("HOT  code buffer %08x..%08x | bucket %u bytes"
	       " | resolve with tools/hotset_resolve.py\n",
	       cb_lo, cb_hi, HOTSET_GRAIN);

	for (n = 0; n < HOTSET_REPORT; n++) {
		uint32_t best_hits = 0, best_key = 0;

		for (i = 0; i < HOTSET_SLOTS; i++) {
			uint32_t k = slot_key[i], c = slot_hits[i];

			if (!k)
				continue;
			if (c > prev_hits || (c == prev_hits && k >= prev_key))
				continue;       /* already printed */
			if (c > best_hits || (c == best_hits && k > best_key)) {
				best_hits = c;
				best_key = k;
			}
		}

		if (!best_key)
			break;

		/* Back into P1 so the address matches the map file. */
		printf("HOT  %08x %6u %3u.%02u%%\n",
		       best_key | 0x80000000u, best_hits,
		       total ? 100u * best_hits / total : 0u,
		       total ? (10000u * best_hits / total) % 100u : 0u);

		prev_hits = best_hits;
		prev_key = best_key;
	}

	printf("HOT  end\n");
}

/* ---------------------------------------------------------------- */
/* The window                                                        */
/* ---------------------------------------------------------------- */

void bloom_hotset_vsync(void)
{
	if (state < 0)
		return;

	if (!state) {
		if (++frames < HOTSET_SKIP)
			return;

		/* TMU2 IS THE ONE THAT IS FREE.  TMU0 runs KOS's scheduler and
		 * TMU1 is the free-running counter timer_us_gettime reads; both
		 * would be taken out from under the kernel.  TMU2 is bloom's
		 * only if the deadline is using it, and that is parked. */
		memset(slot_key, 0, sizeof(slot_key));
		memset(slot_hits, 0, sizeof(slot_hits));
		total = jit_hits = lost = 0;
		frames = 0;

		cb_lo = (uint32_t)(uintptr_t)code_buffer & 0x1fffffffu;
		cb_hi = cb_lo + CODE_BUFFER_SIZE;

		irq_set_handler(EXC_TMU2_TUNI2, hotset_handler, NULL);
		timer_prime(TMU2, HOTSET_HZ, 1);
		timer_start(TMU2);

		state = 1;
		return;
	}

	if (++frames < HOTSET_FRAMES)
		return;

	timer_stop(TMU2);
	irq_set_handler(EXC_TMU2_TUNI2, NULL, NULL);
	state = -1;

	hotset_report();
}

#endif
