/*
 * PC sampling profiler.
 *
 * A timer interrupt records the PC it interrupted, bucketed by icache line.
 * The result says where wall-clock time goes, which is the question that
 * matters and the one instruction counts cannot answer: a renderer doing more
 * instructions can cost less time than a JIT stalling on an 8 KiB
 * direct-mapped icache.
 *
 * Buckets are 32 bytes - one SH-4 icache line - so the output can be read as
 * cache behaviour as well as attribution. JIT code is covered along with
 * everything else; blocks move between runs, so JIT addresses are only
 * meaningful in aggregate, which is what the code-buffer summary is for.
 *
 * Output goes to /pc/ (dcload's host filesystem) in the format
 * build/docs/prof_agg.py already reads.
 */

#include "bloom-config.h"

#include "guestdump.h"
#include "prof.h"

#if WITH_PROF

#include <stdio.h>
#include <string.h>

#include <arch/irq.h>
#include <arch/timer.h>
#include <kos/irq.h>

/* Two histograms rather than one flat table over all of RAM.
 *
 * Only two regions of the 16 MiB are ever executed - bloom's own image, and
 * lightrec's JIT code buffer - and covering the gap between them costs real
 * memory on a machine that has none spare. A flat 32-byte table over all of
 * RAM is 1 MiB of mostly-zero bins, which is enough to push the heap up and
 * fail an allocation the emulator needs.
 *
 * 64-byte buckets: two icache lines. Fine for attributing time to functions,
 * and half the memory of line-accurate bins.
 */
#ifndef WITH_PROF_SHIFT
#define WITH_PROF_SHIFT		6
#endif

#define PROF_IMAGE_BASE	0x8c000000u
#define PROF_IMAGE_SPAN	0x00400000u			/* 4 MiB: text + data */
#define PROF_IMAGE_BINS	(PROF_IMAGE_SPAN >> WITH_PROF_SHIFT)

/* The code buffer sits just below the stack; cover generously from there to
 * the top of RAM so this needs no knowledge of where it lands. */
#define PROF_JIT_BASE	0x8ca00000u
#define PROF_JIT_SPAN	0x00600000u			/* 6 MiB */
#define PROF_JIT_BINS	(PROF_JIT_SPAN >> WITH_PROF_SHIFT)

#ifndef WITH_PROF_HZ
#define WITH_PROF_HZ	2000
#endif

/* BENCH windows to sample before dumping. Waiting for a clean exit is not
 * good enough: a profiling run is usually ended by pulling the plug or by a
 * harness timeout, and a profile that only survives a tidy shutdown is the
 * one you never get. */
#ifndef WITH_PROF_WINDOWS
#define WITH_PROF_WINDOWS	20
#endif

/* Windows to throw away before the histogram starts counting. */
#ifndef WITH_PROF_SKIP
#define WITH_PROF_SKIP		15
#endif

/* u16 and saturating: a bin that reaches 65535 is already the hottest thing
 * in the run by a wide margin, and halving the table's memory matters more on
 * a 16 MiB machine than distinguishing "very hot" from "very hot indeed". */
static uint16_t prof_image[PROF_IMAGE_BINS];
static uint16_t prof_jit[PROF_JIT_BINS];

static uint32_t prof_total;
static uint32_t prof_outside;		/* PC outside RAM: BIOS, P4, garbage */
static uint32_t prof_saturated;
static int prof_running;
static unsigned int prof_windows;
static int prof_dumped;

static void prof_tick(irq_t code, irq_context_t *ctx, void *data)
{
	uint32_t pc = ctx->pc;
	uint16_t *bin;

	(void)code;
	(void)data;

	timer_clear(TMU2);

	prof_total++;

	/* Fold the cached and uncached views of RAM together: the same code
	 * seen through 0xac... is the same line in the cache. */
	pc &= 0x9fffffffu;

	if (pc >= PROF_IMAGE_BASE && pc < PROF_IMAGE_BASE + PROF_IMAGE_SPAN) {
		bin = &prof_image[(pc - PROF_IMAGE_BASE) >> WITH_PROF_SHIFT];
	} else if (pc >= PROF_JIT_BASE && pc < PROF_JIT_BASE + PROF_JIT_SPAN) {
		bin = &prof_jit[(pc - PROF_JIT_BASE) >> WITH_PROF_SHIFT];
	} else {
		prof_outside++;
		return;
	}

	if (*bin == 0xffff)
		prof_saturated++;
	else
		(*bin)++;
}

void prof_init(void)
{
	if (prof_running)
		return;

	memset(prof_image, 0, sizeof(prof_image));
	memset(prof_jit, 0, sizeof(prof_jit));
	prof_total = prof_outside = prof_saturated = 0;

	/* TMU0 is KOS's scheduler tick and TMU1 its free-running counter, so
	 * TMU2 is the only channel free to take. */
	irq_set_handler(EXC_TMU2_TUNI2, prof_tick, NULL);

	if (timer_prime(TMU2, WITH_PROF_HZ, 1) < 0) {
		printf("prof: timer_prime failed, profiler disabled\n");
		return;
	}
	if (timer_start(TMU2) < 0) {
		printf("prof: timer_start failed, profiler disabled\n");
		return;
	}

	prof_running = 1;
	prof_windows = 0;
	printf("prof: sampling at %d Hz for %d windows, %u bins of %d bytes "
	       "(%u KiB)\n",
	       WITH_PROF_HZ, WITH_PROF_WINDOWS,
	       (unsigned)(PROF_IMAGE_BINS + PROF_JIT_BINS), 1 << WITH_PROF_SHIFT,
	       (unsigned)((sizeof(prof_image) + sizeof(prof_jit)) >> 10));
}

void prof_window(void)
{
	if (!prof_running || prof_dumped)
		return;

	prof_windows++;

	/* Discard the first few windows. The scene has just been restored from
	 * a savestate, so the early frames are dominated by compiling code
	 * that has never run - which is real work, but not the steady state a
	 * profile is usually meant to describe. */
	if (prof_windows == WITH_PROF_SKIP) {
		memset(prof_image, 0, sizeof(prof_image));
		memset(prof_jit, 0, sizeof(prof_jit));
		prof_total = prof_outside = prof_saturated = 0;
		printf("prof: warm-up discarded, sampling steady state\n");
		return;
	}

	if (prof_windows < WITH_PROF_SKIP + WITH_PROF_WINDOWS)
		return;

	prof_shutdown();
	prof_dump("run");

	/* Same instant as the profile, so tools/lrtest replays the scene these
	 * numbers describe rather than the BIOS boot. */
	guest_dump("run");

	prof_dumped = 1;
}

void prof_shutdown(void)
{
	if (!prof_running)
		return;

	timer_stop(TMU2);
	irq_set_handler(EXC_TMU2_TUNI2, NULL, NULL);
	prof_running = 0;
}

void prof_dump(const char *tag)
{
	char path[64];
	unsigned int i, written = 0;
	FILE *f;

	/* Silent when there is nothing to say: the exit path calls this too,
	 * and by then the window trigger has usually already dumped. */
	if (!prof_total)
		return;

	snprintf(path, sizeof(path), "/pc/bloom_prof_%s.txt", tag ? tag : "run");

	f = fopen(path, "w");
	if (!f) {
		/* No host filesystem - fall back to the console, which the
		 * same aggregator can read out of a dc-tool log. */
		printf("prof: cannot open %s, printing instead\n", path);
		f = NULL;
	}

	/* Every line carries the PROFTXT tag so the console fallback can be
	 * sieved straight out of a dc-tool log by the same aggregator. */
#define OUT(...) do {							\
		if (f)							\
			fprintf(f, __VA_ARGS__);			\
		else							\
			printf(__VA_ARGS__);				\
	} while (0)

	OUT("PROFTXT total %u outside %u\n", prof_total, prof_outside);
	OUT("PROFTXT saturated-bins %u\n", prof_saturated);
	OUT("PROFTXT hz %d bucket %d\n", WITH_PROF_HZ, 1 << WITH_PROF_SHIFT);

	for (i = 0; i < PROF_IMAGE_BINS; i++) {
		if (!prof_image[i])
			continue;
		OUT("PROFTXT %08x:%u\n",
		    (unsigned)(PROF_IMAGE_BASE + (i << WITH_PROF_SHIFT)),
		    (unsigned)prof_image[i]);
		written++;
	}
	for (i = 0; i < PROF_JIT_BINS; i++) {
		if (!prof_jit[i])
			continue;
		OUT("PROFTXT %08x:%u\n",
		    (unsigned)(PROF_JIT_BASE + (i << WITH_PROF_SHIFT)),
		    (unsigned)prof_jit[i]);
		written++;
	}
#undef OUT

	if (f)
		fclose(f);

	printf("prof: %u samples, %u outside RAM, %u live bins -> %s\n",
	       prof_total, prof_outside, written, f ? path : "console");

	memset(prof_image, 0, sizeof(prof_image));
	memset(prof_jit, 0, sizeof(prof_jit));
	prof_total = prof_outside = prof_saturated = 0;
}

#endif /* WITH_PROF */
