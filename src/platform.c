// SPDX-License-Identifier: GPL-2.0-only
/*
 * Misc. glue code for the PCSX port
 *
 * Copyright (C) 2024 Paul Cercueil <paul@crapouillou.net>
 */

#include <frontend/plugin_lib.h>
#include <libpcsxcore/psxcounters.h>
#include <libpcsxcore/gpu.h>

#include <kos/thread.h>
#include <arch/timer.h>
#include <dc/matrix.h>
#include <dc/pvr.h>
#include <dc/sq.h>
#include <dc/video.h>
#include <dc/vmu_fb.h>

#include <stdint.h>
#include <sys/time.h>

#include "bloom-config.h"
#include "emu.h"
#include "overlay.h"
#include "perf.h"
#include "emitmiss.h"
#include "perfcmp.h"
#include "pvr.h"

#define MAX_LAG_FRAMES 3

#define tvdiff(tv, tv_old) \
	((tv.tv_sec - tv_old.tv_sec) * 1000000 + tv.tv_usec - tv_old.tv_usec)

/* PVR texture size in pixels */
#define TEX_WIDTH  1024
#define TEX_HEIGHT 512

static unsigned int frames;
static uint64_t timer_ms;

uint64_t bloom_perf_us[PERF_N];
uint32_t bloom_perf_cnt[PERF_N];
uint64_t bloom_perf_idle_us[PERF_N];
uint64_t bloom_perf_evt_us[PERF_EVT_N];
uint32_t bloom_perf_evt_cnt[PERF_EVT_N];

uint64_t bloom_perf_now(void)
{
	return timer_us_gettime64();
}

/* KOS accounts the idle thread in nanoseconds. */
uint64_t bloom_perf_idle_now(void)
{
	return thd_get_cpu_time(thd_get_idle()) / 1000u;
}
/* Whether the PVR, and not the emulator, is what the frame is waiting on.
 * Read by the renderer: cutting PVR work is only worth VRAM when this is
 * true (src/pvr.c, rtt_update_alloc). */
bool bloom_pvr_bound;

static pvr_ptr_t pvram;
static uint32_t *pvram_sq;

static bool frame_was_24bpp;

float screen_fw, screen_fh;
static unsigned int screen_w, screen_h;
unsigned int screen_bpp;

static uint64_t last_cputime;
static uint64_t last_idletime;

static void dc_alloc_pvram(void)
{
	pvram = pvr_mem_malloc(TEX_WIDTH * TEX_HEIGHT * 2);

	assert(!!pvram);
	assert(!((unsigned int)pvram & 0x1f));

	pvram_sq = (uint32_t *)(((uintptr_t)pvram & 0xffffff) | PVR_TA_TEX_MEM);
}

static int dc_vout_open(void)
{
	if (!started)
		return 0;

	frame_was_24bpp = false;

	if (HARDWARE_ACCELERATED)
		hw_render_start();
	else
		dc_alloc_pvram();

	return 0;
}

static void dc_vout_close(void)
{
	if (!started)
		return;

	if (HARDWARE_ACCELERATED)
		hw_render_stop();

	if (!HARDWARE_ACCELERATED || frame_was_24bpp)
		pvr_mem_free(pvram);
}

static void dc_vout_set_mode(int w, int h, int raw_w, int raw_h, int bpp)
{
	if (!started)
		return;

	screen_w = raw_w;
	screen_h = raw_h;
	screen_bpp = bpp;

	/* Use 1280x480 when using FSAA */
	screen_fw = (float)SCREEN_WIDTH / (float)raw_w;
	screen_fh = (float)SCREEN_HEIGHT / (float)raw_h;

	/*
	 * The GTE owns XMTRX and draw_prim applies this transform as four
	 * multiplies, so loading it here would overwrite a matrix the GTE
	 * believes is still resident - and a mode change can happen between
	 * any two GTE commands.
	 */
	if (0) {
		matrix_t matrix = {
			{ screen_fw, 0.0f, 0.0f, 0.0f },
			{ 0.0f, screen_fh, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f / 256.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f / 1024.0f },
		};

		mat_load(&matrix);
	}
}

static inline void copy15(const uint16_t *vram, int w, int h)
{
	const uint32_t *vram32 = (const uint32_t *)vram;
	uint32_t pixels, r, g, b;
	uint32_t *line, *dest = (uint32_t *)pvram_sq;
	unsigned int x, y, i;

	for (y = 0; y < h; y++) {
		line = sq_lock(dest);

		for (x = 0; x < w; x += 16) {
			for (i = 0; i < 8; i++) {
				pixels = *vram32++;

				b = (pixels >> 10) & 0x001f001f;
				g = pixels & 0x03e003e0;
				r = (pixels & 0x001f001f) << 10;

				line[i] = r | g | b;
			}

			sq_flush(line);
			line += 8;
		}

		vram32 += (TEX_WIDTH - w) / 2;
		dest += TEX_WIDTH / 2;

		sq_unlock();
	}
}

static inline uint16_t rgb_24_to_16(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint16_t)r & 0xf8) << 8
		| ((uint16_t)g & 0xfc) << 3
		| (uint16_t)b >> 3;
}

static inline void copy24(const uint16_t *vram, int w, int h)
{
	const uint32_t *vram32 = (const uint32_t *)vram;
	uint32_t *line, *dest = (uint32_t *)pvram_sq;
	uint32_t w0, w1, w2;
	unsigned int x, y, i;
	uint16_t px0, px1;

	for (y = 0; y < h; y++) {
		line = sq_lock(dest);

		for (x = 0; x < w; x += 16) {
			for (i = 0; i < 8; i += 2) {
				w0 = *vram32++; /* BGRB */
				w1 = *vram32++; /* GRBG */
				w2 = *vram32++; /* RBGR */

				px0 = rgb_24_to_16(w0, w0 >> 8, w0 >> 16);
				px1 = rgb_24_to_16(w0 >> 24, w1, w1 >> 8);
				line[i] = (uint32_t)px1 << 16 | px0;

				px0 = rgb_24_to_16(w1 >> 16, w1 >> 24, w2);
				px1 = rgb_24_to_16(w2 >> 8, w2 >> 16, w2 >> 24);
				line[i + 1] = (uint32_t)px1 << 16 | px0;
			}

			sq_flush(line);
			line += 8;
		}

		sq_unlock();

		vram32 += (TEX_WIDTH * 2 - w * 3) / 4;
		dest += TEX_WIDTH / 2;
	}
}

/* The names of the event sources, in `enum psxint_ev` order (psxevents.h).
 * Short, because fifteen of them share a line. */
static const char *const perf_evt_name[PERF_EVT_N] = {
	"sio", "cdr", "cdread", "gpudma", "mdecout", "spudma", "spuirq",
	"mdecin", "gpuotc", "cdrdma", "drc", "rcnt", "cdrlid", "irq10",
	"spuupd", "?"
};

/* THE BUCKET TABLE.  Name and parent, so that adding a bucket is one line
 * here and one line in perf.h and the report picks it up on its own.
 *
 * The parent column is not decoration.  `gpu` is inside `hw` because the GPU
 * command list runs off a DMA register write, and an early report that made
 * them siblings had the frame summing to more than the frame.  Nothing here
 * is ever added into a total except the three PERF_IN_FRAME rows. */
static const struct {
	const char *name;
	unsigned char in;
} perf_row[PERF_N] = {
	[PERF_CPU]       = { "cpu",    PERF_IN_FRAME },
	[PERF_EVT]       = { "evt",    PERF_IN_FRAME },
	[PERF_FLIP]      = { "flip",   PERF_IN_FRAME },
	[PERF_HW]        = { "hw",     PERF_IN_CPU   },
	[PERF_RW]        = { "rw",     PERF_IN_CPU   },
	[PERF_COP]       = { "cop",    PERF_IN_CPU   },
	[PERF_SVC]       = { "svc",    PERF_IN_CPU   },
	[PERF_GPU]       = { "gpu",    PERF_IN_HW    },
	[PERF_EMUUPDATE] = { "emuupd", PERF_IN_EVT   },
	[PERF_LACE]      = { "lace",   PERF_IN_EVT   },
	[PERF_SPU]       = { "spu",    PERF_IN_EVT   },
	[PERF_VBLANK]    = { "vbl",    PERF_IN_EVT   },
	[PERF_JIT]       = { "jit",    PERF_IN_NONE  },
};

/* A bucket, in the only two columns worth having: milliseconds per frame and
 * calls per frame.  Calls are why this is not just a timer -- "hw 2.1 ms over
 * 6000 accesses" and "hw 2.1 ms over 12 accesses" are different bugs. */
/* The link counters live in the recompiler glue; declaring them here keeps
 * platform.c out of lightrec's private header. */
extern unsigned fgl_link_patched, fgl_link_uncompiled;
extern unsigned fgl_link_undone, fgl_link_bad_site;
extern unsigned fgl_link_range, fgl_link_calls, fgl_link_full;

/* Mirrors the enum in lightrec-private.h; see the unlink line below. */
extern unsigned fgl_unlink_calls[6], fgl_unlink_links[6];

static void perf_col(int b, unsigned int nframes)
{
	printf(" %s %5.2f (%u/f)", perf_row[b].name,
	       bloom_perf_us[b] / 1000.0f / nframes,
	       bloom_perf_cnt[b] / nframes);

	/* Only the buckets that asked for it carry an idle figure, and it is
	 * printed as "of which idle" rather than netted off, because the
	 * subtraction is the reader's judgement: a row that is nearly all
	 * idle is slack, a row that is nearly none is work. */
	if (bloom_perf_idle_us[b])
		printf("[idle %4.1f]",
		       bloom_perf_idle_us[b] / 1000.0f / nframes);
}

/* Three lines a second, next to the fps counter, over the same window.
 *
 * Line one is the frame: three siblings and the remainder, in MILLISECONDS
 * PER FRAME.  The percentages are there to be glanced at, but ms/f is the
 * column that means anything -- the split barely moves across a run while the
 * total swings four to one, so a percentage says nothing about whether
 * anything got faster.
 *
 * Line two opens up `cpu`, which was 80% of the frame and a single number.
 * Everything on it is C that generated code called without leaving the
 * dispatch, and `rest` is what is left: emitted instructions, the GTE, and
 * the divide shims.  If `rest` is the whole of `cpu` then the frame is
 * genuinely in the emitted code and the next lever is the code itself; if it
 * is not, the named row says where to go instead.
 *
 * Line three opens up `evt`, which turned out to be almost entirely `rcnt`,
 * which is not counter work: pcsx hangs the frame boundary off the counter
 * that crosses vblank, so `lace` and `emuupd` and `spu` are in there.  A
 * large `lace` is the render thread being waited on, and waiting is not cost.
 *
 * Silent rows are dropped -- fifteen event sources of which three are ever
 * warm is a line nobody reads.
 *
 * THIS BUILD IS FOR READING A SPREAD, NOT FOR QUOTING A FRAME TIME.  The MMIO
 * shims fire thousands of times a frame and each now pays two timer reads, so
 * the total is inflated by the measurement.  Compare rows against each other,
 * not this run against a run without the brackets. */
static void bloom_perf_report(uint64_t window_ms, unsigned int nframes)
{
	uint64_t wall_us = window_ms * 1000u;
	uint64_t parents, other, kids;
	unsigned int i;

	if (!nframes || !wall_us)
		return;

#if !BLOOM_PERF
	/* The brackets compiled out, so every bucket is zero and the report is
	 * four lines a second of zeroes down the serial port.  See perf.h. */
	(void)wall_us; (void)parents; (void)other; (void)kids; (void)i;
#else
	parents = bloom_perf_us[PERF_CPU] + bloom_perf_us[PERF_EVT]
		+ bloom_perf_us[PERF_FLIP];
	other = wall_us > parents ? wall_us - parents : 0;

	printf("perf %5.1f fps %6.2f ms/f | cpu %6.2f (%4.1f%%, %u/f) "
	       "evt %6.2f (%4.1f%%, %u/f) flip %5.2f  other %5.2f (%4.1f%%)\n",
	       (float)nframes * 1000.0f / (float)window_ms,
	       (float)window_ms / (float)nframes,
	       bloom_perf_us[PERF_CPU] / 1000.0f / nframes,
	       100.0f * bloom_perf_us[PERF_CPU] / wall_us,
	       bloom_perf_cnt[PERF_CPU] / nframes,
	       bloom_perf_us[PERF_EVT] / 1000.0f / nframes,
	       100.0f * bloom_perf_us[PERF_EVT] / wall_us,
	       bloom_perf_cnt[PERF_EVT] / nframes,
	       bloom_perf_us[PERF_FLIP] / 1000.0f / nframes,
	       other / 1000.0f / nframes,
	       100.0f * other / wall_us);

	/* in cpu.  Only the direct children come out of `cpu`; `gpu` is
	 * inside `hw` and is printed after them, in brackets, so that it is
	 * never read as another slice of the same pie. */
	kids = 0;
	printf("  in cpu |");
	for (i = 0; i < PERF_N; i++) {
		if (perf_row[i].in != PERF_IN_CPU)
			continue;
		kids += bloom_perf_us[i];
		perf_col(i, nframes);
	}
	printf("  (gpu %5.2f, %u/f)  rest %6.2f\n",
	       bloom_perf_us[PERF_GPU] / 1000.0f / nframes,
	       bloom_perf_cnt[PERF_GPU] / nframes,
	       (bloom_perf_us[PERF_CPU] > kids
		? bloom_perf_us[PERF_CPU] - kids : 0) / 1000.0f / nframes);

	/* in evt: the warm event sources, then what the vblank tick does. */
	printf("  in evt |");
	for (i = 0; i < PERF_EVT_N; i++) {
		/* A tenth of a millisecond a frame is the floor: below that a
		 * source is noise and its column is in the way. */
		if (bloom_perf_evt_us[i] < 100u * nframes)
			continue;
		printf(" %s %5.2f (%u/f)", perf_evt_name[i],
		       bloom_perf_evt_us[i] / 1000.0f / nframes,
		       bloom_perf_evt_cnt[i] / nframes);
	}
	printf("  |");
	for (i = 0; i < PERF_N; i++)
		if (perf_row[i].in == PERF_IN_EVT)
			perf_col(i, nframes);
	printf("  | jit %5.2f (%u blk)\n",
	       bloom_perf_us[PERF_JIT] / 1000.0f / nframes,
	       bloom_perf_cnt[PERF_JIT]);

	/* Links are cumulative, not per-window: an edge is patched once and
	 * then stops costing anything, so a rate would read as zero forever
	 * after the first second.  `near` is how many of the patched edges were
	 * inside a `bra`'s reach -- the population the FAR-only experiment is
	 * about -- and it is counted even when the far path is forced. */
	printf("  links  | near %u far %u uncompiled %u undone %u"
	       " badsite %u calls %u full %u\n",
	       fgl_link_patched, fgl_link_range, fgl_link_uncompiled,
	       fgl_link_undone, fgl_link_bad_site, fgl_link_calls, fgl_link_full);

	/* WHO KEEPS KILLING THE LINKS.  A teardown is global -- freeing or
	 * invalidating anything puts every patched site in the program back --
	 * so `undone` above says how much work was thrown away and this says
	 * which caller threw it.  Two numbers each: how often it fired, and
	 * how many live links it took with it. */
	{
		static const char *const why[] = {
			"invmap", "smc", "free", "inv", "invall", "lut"
		};
		unsigned k;

		printf("  unlink |");
		for (k = 0; k < 6; k++)
			printf(" %s %u/%u", why[k], fgl_unlink_calls[k],
			       fgl_unlink_links[k]);
		printf("\n");
	}

	for (i = 0; i < PERF_N; i++) {
		bloom_perf_us[i] = 0;
		bloom_perf_cnt[i] = 0;
		bloom_perf_idle_us[i] = 0;
	}
	for (i = 0; i < PERF_EVT_N; i++) {
		bloom_perf_evt_us[i] = 0;
		bloom_perf_evt_cnt[i] = 0;
	}
#endif /* BLOOM_PERF */
}

static void dc_vout_flip(const void *vram, int offset, int bgr24,
			 int x, int y, int w, int h, int dims_changed)
{
	float ymin, ymax, xmin, xmax, idle_diff, cpu_diff;
	uint64_t new_timer, cputime, idletime;
	pvr_stats_t pvr_stats;
	pvr_poly_cxt_t cxt;
	pvr_poly_hdr_t hdr;
	pvr_vertex_t vert;
	int copy_w;

	if (!started || !vram)
		return;

	PERF_BEGIN(PERF_FLIP);

	if (HARDWARE_ACCELERATED && !frame_was_24bpp) {
		/* Render the old frame */
		hw_render_stop();

		if (bgr24) {
			invalidate_all_textures();
			dc_alloc_pvram();
		}
	}

	if (HARDWARE_ACCELERATED && !bgr24) {
		if (frame_was_24bpp)
			pvr_mem_free(pvram);

		/* Prepare the next frame */
		hw_render_start();
	} else {
		vram = (void *)((uintptr_t)vram + offset);
		assert(!((unsigned int)vram & 0x3));

		/* We transfer 16 pixels at a time, so align width to 32 bytes.
		 * We are just transferring the texture so it does not matter if
		 * we're reading too far. */
		copy_w = (w + 31) & ~31;

		if (bgr24)
			copy24(vram, copy_w, h);
		else
			copy15(vram, copy_w, h);

		ymin = (float)y * (float)screen_fh;
		ymax = (float)(y + h) * (float)screen_fh;
		xmin = (float)x * (float)screen_fw;
		xmax = (float)(x + w) * (float)screen_fw;

		pvr_wait_ready();
		pvr_scene_begin();
		pvr_list_begin(PVR_LIST_OP_POLY);

		pvr_poly_cxt_txr(&cxt, PVR_LIST_OP_POLY,
				 PVR_TXRFMT_NONTWIDDLED | (bgr24 ? PVR_TXRFMT_RGB565 : PVR_TXRFMT_ARGB1555),
				 TEX_WIDTH, TEX_HEIGHT, pvram, PVR_FILTER_NONE);

		pvr_poly_compile(&hdr, &cxt);
		pvr_prim(&hdr, sizeof(hdr));

		vert.argb = PVR_PACK_COLOR(1.0f, 1.0f, 1.0f, 1.0f);
		vert.oargb = 0;
		vert.flags = PVR_CMD_VERTEX;

		vert.x = xmin;
		vert.y = ymin;
		vert.z = 1.0f;
		vert.u = 0.0f;
		vert.v = 0.0f;
		pvr_prim(&vert, sizeof(vert));

		vert.x = xmax;
		vert.y = ymin;
		vert.z = 1.0f;
		vert.u = (float)w / (float)TEX_WIDTH;
		vert.v = 0.0f;
		pvr_prim(&vert, sizeof(vert));

		vert.x = xmin;
		vert.y = ymax;
		vert.z = 1.0f;
		vert.u = 0.0f;
		vert.v = (float)h / (float)TEX_HEIGHT;
		pvr_prim(&vert, sizeof(vert));

		vert.x = xmax;
		vert.y = ymax;
		vert.z = 1.0f;
		vert.u = (float)w / (float)TEX_WIDTH;
		vert.v = (float)h / (float)TEX_HEIGHT;
		vert.flags = PVR_CMD_VERTEX_EOL;
		pvr_prim(&vert, sizeof(vert));

		pvr_list_finish();
		pvr_scene_finish();
	}

	frame_was_24bpp = bgr24;

	PERF_END(PERF_FLIP);

	new_timer = timer_ms_gettime64();

	frames++;
	EMITMISS_VSYNC();

	if (timer_ms == 0) {
		timer_ms = new_timer;
		return;
	}

	if (new_timer > (timer_ms + 1000)) {
		perfcmp_report(frames);
		bloom_perf_report(new_timer - timer_ms, frames);
		pvr_get_stats(&pvr_stats);

		cputime = timer_ms_gettime64();
		idletime = thd_get_cpu_time(thd_get_idle());

		idle_diff = idletime - last_idletime;
		cpu_diff = cputime - last_cputime;

		vmu_printf(" FPS: %5.1f\n\n %ux%u-%u\n PVR %02.02f%%\n SH4 %02.02f%%",
			   (float)frames, screen_w, screen_h, screen_bpp,
			   (float)pvr_stats.rnd_last_time * 100.0f / 16666666.7f,
			   100.0f - 100.0f * idle_diff / cpu_diff);

		if (BLOOM_PERFCMP)
			printf("BENCH fps %5.1f frame %6.2f ms pvr %5.2f%% sh4 %5.2f%%\n",
			       (float)frames * 1000.0f / (float)(new_timer - timer_ms),
			       (float)(new_timer - timer_ms) / (float)frames,
			       (float)pvr_stats.rnd_last_time * 100.0f / 16666666.7f,
			       100.0f - 100.0f * idle_diff / cpu_diff);

		overlay_set("%.1f fps  %.2f ms",
			    (float)frames * 1000.0f / (float)(new_timer - timer_ms),
			    (float)(new_timer - timer_ms) / (float)frames);

#if FGL_FAULT_IO
		/* Faults per frame, and the last address one came from.  A
		 * device-heavy frame is a few thousand; anything in the
		 * millions is a poll loop that is not getting the answer it
		 * wants, and the address says which one. */
		{
			extern unsigned int fgl_fault_io_count;
			extern unsigned int fgl_fault_io_last;
			static unsigned int last_count;
			unsigned int now = fgl_fault_io_count;

			fprintf(stderr, "fgl: io %u faults/frame last %08x\n",
				frames ? (now - last_count) / frames : 0,
				fgl_fault_io_last);
			last_count = now;
		}
#endif

		/* Idle on the SH-4 that the PVR is being waited on for. */
		bloom_pvr_bound = idle_diff * 100 > cpu_diff * 10
			&& (float)pvr_stats.rnd_last_time / 1000000.0f
			   > (float)(new_timer - timer_ms) / (float)frames * 0.5f;

		timer_ms = new_timer;
		frames = 0;

		last_cputime = cputime;
		last_idletime = idletime;
	}
}

static struct rearmed_cbs dc_rearmed_cbs = {
	.pl_vout_open		= dc_vout_open,
	.pl_vout_close		= dc_vout_close,
	.pl_vout_set_mode	= dc_vout_set_mode,
	.pl_vout_flip		= dc_vout_flip,

	.gpu_hcnt		= (unsigned int *)&hSyncCount,
	.gpu_frame_count	= (unsigned int *)&frame_counter,
	.gpu_state_change	= gpu_state_change,

	.gpu_unai = {
		.lighting = 1,
		.blending = 1,
	},
};

void plugin_call_rearmed_cbs(void)
{
	extern void *hGPUDriver;
	void (*rearmed_set_cbs)(const struct rearmed_cbs *cbs);

	rearmed_set_cbs = SysLoadSym(hGPUDriver, "GPUrearmedCallbacks");
	if (rearmed_set_cbs != NULL)
		rearmed_set_cbs(&dc_rearmed_cbs);
}

void pl_frame_limit(void)
{
}
