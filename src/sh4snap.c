// SPDX-License-Identifier: GPL-2.0-only
/*
 * SH4-timing snapshot hook (build with -DWITH_SH4_SNAPSHOT=ON; off by default).
 *
 * At PS1 frame WITH_SH4_SNAPSHOT_FRAME (counted at each vout flip) Bloom saves its own CPU
 * state (sh4snap_capture.S, a copy of SH4-timing's sim/snapshot/capture.S) and writes
 *   /pc/bloom_state.bin  (816 bytes)   /pc/bloom_ram.bin  (16 MiB)
 * Rendering then stops (no PowerVR access): the PS1 GPU commands are still parsed and timed by
 * the emulator, only the PowerVR conversion is skipped. Bloom runs SH4SNAP_RUN_FRAMES more
 * frames, hashes the PS1 state, writes /pc/bloom_result.txt (hash + console time per frame)
 * and exits. The SH4-timing lab resumes the same snapshot, runs the same frames with the same
 * code, and must reach the same hash.
 */

#include <arch/arch.h>
#include <arch/cache.h>
#include <arch/timer.h>

#include <libpcsxcore/psxmem.h>
#include <libpcsxcore/r3000a.h>
#include <gpulib/gpu.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bloom-config.h"

#define SH4SNAP_RUN_FRAMES 3

extern int sh4_snapshot_capture(uint32_t *state) __attribute__((returns_twice));

bool sh4snap_no_pvr;		/* read by the renderer and the vout flip */

static uint32_t snap_state[204] __attribute__((aligned(32)));
static unsigned int snap_frames;	/* flips seen so far */
static int snap_resumed;
static uint64_t snap_t0;
static uint64_t snap_ns[SH4SNAP_RUN_FRAMES];

static int snap_write(const char *path, const void *data, size_t n)
{
	FILE *f = fopen(path, "wb");
	size_t done = 0, chunk;

	if (!f) {
		printf("SNAPSHOT error: cannot open %s\n", path);
		return -1;
	}

	while (done < n) {
		chunk = n - done > 65536 ? 65536 : n - done;
		if (fwrite((const uint8_t *)data + done, 1, chunk, f) != chunk)
			break;
		done += chunk;
	}

	fclose(f);

	if (done != n) {
		printf("SNAPSHOT error: short write %s\n", path);
		return -1;
	}

	return 0;
}

static uint32_t fnv_words(uint32_t h, const void *data, size_t bytes)
{
	const uint32_t *p = data, *end = p + bytes / 4;

	for (; p < end; p++)
		h = (h ^ *p) * 16777619u;

	return h;
}

/* The end of the measured run (the lab's stop PC). */
uint32_t __attribute__((noinline)) sh4snap_hash(void)
{
	uint32_t h = 2166136261u;

	h = fnv_words(h, psxM, 0x200000);		/* PS1 RAM */
	h = fnv_words(h, gpu.vram, 1024 * 1024);	/* PS1 VRAM */
	h = fnv_words(h, psxRegs.GPR.r, sizeof(psxRegs.GPR.r));
	h = fnv_words(h, &psxRegs.cycle, sizeof(psxRegs.cycle));

	return h;
}

/* Called once per PS1 frame, at the vout flip (the lab's frame marker). */
void __attribute__((noinline)) sh4snap_frame(void)
{
	char line[200];
	uint64_t now;
	uint32_t h;

	snap_frames++;

	if (snap_frames < WITH_SH4_SNAPSHOT_FRAME)
		return;

	if (snap_frames == WITH_SH4_SNAPSHOT_FRAME) {
		snap_resumed = sh4_snapshot_capture(snap_state);
		sh4snap_no_pvr = true;

		if (!snap_resumed) {
			dcache_wback_range(0x8c000000, 16 * 1024 * 1024);
			printf("SNAPSHOT capture frame=%u pc=%08lx sr=%08lx sp=%08lx\n",
			       snap_frames, (unsigned long)snap_state[18],
			       (unsigned long)snap_state[19], (unsigned long)snap_state[15]);

			if (snap_write("/pc/bloom_state.bin", snap_state, sizeof(snap_state)) ||
			    snap_write("/pc/bloom_ram.bin", (const void *)0x8c000000, 16 * 1024 * 1024))
				arch_exit();

			printf("SNAPSHOT files written\n");
			snap_t0 = timer_ns_gettime64();
		}

		return;
	}

	/* frames 1 .. SH4SNAP_RUN_FRAMES after the snapshot */
	if (!snap_resumed) {
		now = timer_ns_gettime64();
		snap_ns[snap_frames - WITH_SH4_SNAPSHOT_FRAME - 1] = now - snap_t0;
		snap_t0 = now;
	}

	if (snap_frames < WITH_SH4_SNAPSHOT_FRAME + SH4SNAP_RUN_FRAMES)
		return;

	h = sh4snap_hash();

	if (snap_resumed) {
		*(volatile uint32_t *)0xacfffff0 = h;	/* lab only: end of run */
		for (;;)
			;
	}

	snprintf(line, sizeof(line), "hash=%08lx frames=%u frame_ns=%llu,%llu,%llu\n",
		 (unsigned long)h, snap_frames,
		 (unsigned long long)snap_ns[0], (unsigned long long)snap_ns[1],
		 (unsigned long long)snap_ns[2]);
	printf("SNAPSHOT result %s", line);
	snap_write("/pc/bloom_result.txt", line, strlen(line));
	printf("done\n");
	arch_exit();
}
