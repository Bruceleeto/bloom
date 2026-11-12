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
#include <arch/irq.h>
#include <dc/pvr.h>

#include <libpcsxcore/psxmem.h>
#include <libpcsxcore/r3000a.h>
#include <gpulib/gpu.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bloom-config.h"

#ifdef SH4SNAP_RENDER
#include "pvr_internal.h"	/* KOS internal: pvr_state */
#endif

#define SH4SNAP_RUN_FRAMES 3

extern int sh4_snapshot_capture(uint32_t *state) __attribute__((returns_twice));

bool sh4snap_no_pvr;		/* read by the renderer and the vout flip */
bool sh4snap_lab;		/* running resumed in the SH4-timing lab */

/* Lab only (render mode): the lab has no PowerVR and no interrupts, so nothing would ever
 * clear the TA/render busy flags. Clear them where the console would wait. */
void sh4snap_pvr_idle(void)
{
#ifdef SH4SNAP_RENDER
	pvr_state.ta_busy = 0;
	pvr_state.render_busy = 0;
#endif
}

static uint32_t snap_state[204] __attribute__((aligned(32)));
static unsigned int snap_frames;	/* flips seen so far */
static unsigned int snap_at;		/* flip of the current capture, 0 = none yet */
static unsigned int snap_tries;
static unsigned int snap_reads0;	/* cdimg_reads at the capture */
static int snap_resumed;
static uint64_t snap_t0;
static uint64_t snap_ns[SH4SNAP_RUN_FRAMES];
/* The same frames' cost WITH rendering: the last frames before the capture (PowerVR still on). */
static uint64_t pre_t, pre_ns[SH4SNAP_RUN_FRAMES];
/* Of those, time blocked on the PowerVR (pvr.c), and KOS's own render/registration times. */
extern uint64_t sh4snap_wait_ns;
extern uint32_t sh4snap_cnt[16];
static uint64_t pre_w, pre_wait[SH4SNAP_RUN_FRAMES], pre_rnd[SH4SNAP_RUN_FRAMES], pre_reg[SH4SNAP_RUN_FRAMES];

/* Disc image reads by any thread (cdriso.c). The lab has no IDE/GD-ROM, so a measured run
 * must not read the disc: if it did, capture again at the current frame and retry. */
extern volatile unsigned int cdimg_reads;
int cdra_snap_preload(int before, int count);	/* cdrom-async.c: disc sectors into RAM */

static int snap_write(const char *path, const void *data, size_t n);

/* PC SAMPLER: TMU1 at SAMPLE_HZ over the SAMPLE_FRAMES rendered frames before the capture.
 * Each sample is {pc, pr} of the interrupted code; /pc/bloom_samples.bin. Code running with
 * interrupts masked is not seen. Stopped before the capture, so the lab never sees TMU1. */
#define SAMPLE_HZ	6000
#define SAMPLE_FRAMES	150
#define SAMPLE_N	32768
static uint32_t samp[SAMPLE_N][2];
static volatile unsigned int samp_n;
static int samp_state;		/* 0 idle, 1 running, 2 done */
static uint64_t samp_t0, samp_ns;
static unsigned int samp_f0;

static void samp_irq(irq_t src, irq_context_t *c, void *d)
{
	(void)src; (void)d;
	timer_clear(TMU1);
	if (samp_n < SAMPLE_N) {
		samp[samp_n][0] = c->pc;
		samp[samp_n][1] = c->pr;
		samp_n++;
	}
}

static void samp_start(void)
{
	samp_state = 1;
	samp_f0 = snap_frames;
	irq_set_handler(EXC_TMU1_TUNI1, samp_irq, NULL);
	timer_prime(TMU1, SAMPLE_HZ, 1);
	memset(sh4snap_cnt, 0, sizeof(sh4snap_cnt));
	samp_t0 = timer_ns_gettime64();
	timer_start(TMU1);
}

static void samp_stop(void)
{
	uint32_t hdr[4];

	timer_stop(TMU1);
	timer_disable_ints(TMU1);
	samp_ns = timer_ns_gettime64() - samp_t0;
	samp_state = 2;
	hdr[0] = 0x504d4153;	/* "SAMP" */
	hdr[1] = samp_n;
	hdr[2] = snap_frames - samp_f0;
	hdr[3] = (uint32_t)(samp_ns / 1000);	/* microseconds */
	printf("SAMPLES %u over %u frames, %lu us\n", samp_n, hdr[2], (unsigned long)hdr[3]);
	{
		static uint32_t out[4 + 2 * SAMPLE_N];
		memcpy(out, hdr, sizeof(hdr));
		memcpy(out + 4, samp, samp_n * 8);
		{
		static const char *const names[16] = {
			"gp0_packets", "process_poly", "poly_draw_now", "draw_prim",
			"draw_prim_hdr", "vertices", "vertices_64B", "load_palette",
			"update_texture", "codebook_slow", "process_gpu_commands", "add_clip",
			"enqueue_pt", "enqueue_tr", "fast_poly", "",
		};
		char txt[1024];
		int len = 0, k;

		for (k = 0; k < 15; k++)
			len += snprintf(txt + len, sizeof(txt) - len, "%s %lu\n", names[k],
					(unsigned long)sh4snap_cnt[k]);
		snap_write("/pc/bloom_counts.txt", txt, len);
	}
	snap_write("/pc/bloom_samples.bin", out, 16 + samp_n * 8);
	}
}

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

static int __attribute__((noinline)) snap_capture(void)
{
	int pre;

	if (samp_state == 1)
		samp_stop();
	snap_at = snap_frames;
	/* The measured frames read the disc (XA streaming): serve the next sectors from RAM. */
	pre = cdra_snap_preload(32, 192);
	snap_resumed = sh4_snapshot_capture(snap_state);
#ifdef SH4SNAP_RENDER
	sh4snap_no_pvr = false;
#else
	sh4snap_no_pvr = true;
#endif

	if (snap_resumed) {
		sh4snap_lab = true;
		return 0;
	}

	snap_reads0 = cdimg_reads;
	dcache_wback_range(0x8c000000, 16 * 1024 * 1024);
	printf("SNAPSHOT capture frame=%u preload=%d pc=%08lx sr=%08lx sp=%08lx\n",
	       snap_frames, pre, (unsigned long)snap_state[18],
	       (unsigned long)snap_state[19], (unsigned long)snap_state[15]);

	if (snap_write("/pc/bloom_state.bin", snap_state, sizeof(snap_state)) ||
	    snap_write("/pc/bloom_ram.bin", (const void *)0x8c000000, 16 * 1024 * 1024))
		return -1;
#ifdef SH4SNAP_RENDER
	/* KOS keeps its texture allocator's chunk headers in VRAM: the lab needs them (--pvr-vram).
	 * 32-bit view, as the allocator itself reads it. */
	if (snap_write("/pc/bloom_vram.bin", (const void *)0xa5000000, 8 * 1024 * 1024))
		return -1;
#endif

	printf("SNAPSHOT files written\n");
	snap_reads0 = cdimg_reads;	/* not the file writes themselves */
	snap_t0 = timer_ns_gettime64();
	return 0;
}

/* Called once per PS1 frame, at the vout flip (the lab's frame marker). */
void __attribute__((noinline)) sh4snap_frame(void)
{
	char line[512];
	uint64_t now;
	uint32_t h;

	snap_frames++;

	if (!snap_at) {
		now = timer_ns_gettime64();
		if (pre_t) {
			pvr_stats_t st;

			memmove(pre_ns, pre_ns + 1, sizeof(pre_ns) - sizeof(pre_ns[0]));
			memmove(pre_wait, pre_wait + 1, sizeof(pre_wait) - sizeof(pre_wait[0]));
			memmove(pre_rnd, pre_rnd + 1, sizeof(pre_rnd) - sizeof(pre_rnd[0]));
			memmove(pre_reg, pre_reg + 1, sizeof(pre_reg) - sizeof(pre_reg[0]));
			pre_ns[SH4SNAP_RUN_FRAMES - 1] = now - pre_t;
			pre_wait[SH4SNAP_RUN_FRAMES - 1] = sh4snap_wait_ns - pre_w;
			if (!pvr_get_stats(&st)) {
				pre_rnd[SH4SNAP_RUN_FRAMES - 1] = st.rnd_last_time;
				pre_reg[SH4SNAP_RUN_FRAMES - 1] = st.reg_last_time;
			}
		}
		pre_t = now;
		pre_w = sh4snap_wait_ns;
	}

	if (!samp_state && snap_frames + SAMPLE_FRAMES == WITH_SH4_SNAPSHOT_FRAME)
		samp_start();

	if (snap_frames < WITH_SH4_SNAPSHOT_FRAME)
		return;

	if (!snap_at) {
		if (snap_capture())
			arch_exit();
		return;
	}

	/* frames 1 .. SH4SNAP_RUN_FRAMES after the capture */
	if (!snap_resumed) {
		now = timer_ns_gettime64();
		snap_ns[snap_frames - snap_at - 1] = now - snap_t0;
		snap_t0 = now;
	}

	if (snap_frames < snap_at + SH4SNAP_RUN_FRAMES)
		return;

	h = sh4snap_hash();

	if (snap_resumed) {
		*(volatile uint32_t *)0xacfffff0 = h;	/* lab only: end of run */
		for (;;)
			;
	}

	if (cdimg_reads != snap_reads0) {
		printf("SNAPSHOT retry: %u disc reads during the run\n", cdimg_reads - snap_reads0);
		if (++snap_tries >= 30) {
			printf("SNAPSHOT error: no 3-frame window without disc reads\n");
			arch_exit();
		}
		if (snap_capture())
			arch_exit();
		return;
	}

	snprintf(line, sizeof(line), "hash=%08lx frames=%u frame_ns=%llu,%llu,%llu capture=%u render_ns=%llu,%llu,%llu wait_ns=%llu,%llu,%llu pvr_rnd_ns=%llu,%llu,%llu pvr_reg_ns=%llu,%llu,%llu\n",
		 (unsigned long)h, snap_frames,
		 (unsigned long long)snap_ns[0], (unsigned long long)snap_ns[1],
		 (unsigned long long)snap_ns[2], snap_at,
		 (unsigned long long)pre_ns[0], (unsigned long long)pre_ns[1],
		 (unsigned long long)pre_ns[2],
		 (unsigned long long)pre_wait[0], (unsigned long long)pre_wait[1],
		 (unsigned long long)pre_wait[2],
		 (unsigned long long)pre_rnd[0], (unsigned long long)pre_rnd[1],
		 (unsigned long long)pre_rnd[2],
		 (unsigned long long)pre_reg[0], (unsigned long long)pre_reg[1],
		 (unsigned long long)pre_reg[2]);
	printf("SNAPSHOT result %s", line);
	snap_write("/pc/bloom_result.txt", line, strlen(line));
	printf("done\n");
	arch_exit();
}
