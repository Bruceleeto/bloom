// SPDX-License-Identifier: GPL-2.0-only
/*
 * MDEC movies through the PVR's YUV420 converter.
 *
 * Every 24-bit MDEC macroblock is also kept as a 384-byte YUV420 block (U, V,
 * then the four 8x8 Y blocks) in a ring.  When the game uploads the decoded
 * RGB to VRAM as 16-pixel wide columns, straight from the MDEC output buffer,
 * the VRAM cells it lands on remember which ring entry they came from.  If a
 * 24-bit frame is entirely made of such cells, it is displayed by feeding the
 * ring entries to the YUV converter instead of converting VRAM to RGB565, and
 * the next MDEC decodes skip the YCbCr -> RGB conversion altogether.
 */

#include <dc/pvr.h>
#include <dc/sq.h>
#include <kos/cache.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "mdec_yuv.h"

#define MB_SIZE		384
#define RING_SIZE	1024	/* macroblocks, a power of two */

/* VRAM cells: 24 words (16 pixels at 24 bpp) by 16 lines */
#define CELL_W		24
#define CELL_H		16
#define GRID_COLS	(1024 / CELL_W)
#define GRID_ROWS	(512 / CELL_H)

/* Recent runs of macroblocks decoded to contiguous guest memory */
#define NB_RUNS		4

struct mb_run {
	const uint8_t *addr;
	uint32_t first;		/* ring sequence number of the first macroblock */
	uint32_t count;
};

static uint8_t *ring;
static uint32_t ring_seq;
static struct mb_run runs[NB_RUNS];
static unsigned int run_last;

/* Ring sequence number + 1 of the macroblock in each cell, 0 when unknown */
static uint32_t grid[GRID_ROWS][GRID_COLS];

bool mdec_yuv_skip_rgb;
unsigned int mdec_yuv_frames, mdec_yuv_misses;

uint8_t *mdec_yuv_mb(const uint8_t *out)
{
	struct mb_run *run = &runs[run_last];
	unsigned int i;
	uint8_t *mb;

	if (__builtin_expect(!ring, 0)) {
		ring = memalign(32, RING_SIZE * MB_SIZE);
		if (!ring)
			return NULL;
	}

	if (run->count && out == run->addr + run->count * 768) {
		run->count++;
	} else {
		run_last = (run_last + 1) % NB_RUNS;
		run = &runs[run_last];
		run->addr = out;
		run->first = ring_seq;
		run->count = 1;
	}

	mb = ring + (ring_seq & (RING_SIZE - 1)) * MB_SIZE;
	ring_seq++;

	/* The slot is overwritten whole: skip the read of each store miss */
	for (i = 0; i < MB_SIZE; i += 32)
		dcache_alloc_line(mb + i);

	return mb;
}

static inline bool seq_alive(uint32_t seq)
{
	return ring_seq - seq <= RING_SIZE;
}

/* Ring sequence number + 1 of the macroblock decoded at this address, or 0 */
static uint32_t lookup_mb(const uint8_t *addr)
{
	const struct mb_run *run;
	unsigned int i;
	uintptr_t off;

	for (i = 0; i < NB_RUNS; i++) {
		run = &runs[(run_last + NB_RUNS - i) % NB_RUNS];

		if (!run->count || addr < run->addr)
			continue;

		off = addr - run->addr;
		if (off % 768 || off / 768 >= run->count)
			continue;

		if (!seq_alive(run->first + off / 768))
			return 0;

		return run->first + off / 768 + 1;
	}

	return 0;
}

static void invalidate(int x, int y, int w, int h)
{
	int c0 = x / CELL_W, c1 = (x + w - 1) / CELL_W;
	int r0 = y / CELL_H, r1 = (y + h - 1) / CELL_H;
	int r, c;

	if (c1 >= GRID_COLS)
		c1 = GRID_COLS - 1;
	if (r1 >= GRID_ROWS)
		r1 = GRID_ROWS - 1;

	for (r = r0; r <= r1; r++)
		for (c = c0; c <= c1; c++)
			grid[r][c] = 0;
}

void mdec_yuv_vram_write(const uint16_t *data, int words16, int x, int y,
			 int w, int h, int first_line)
{
	const uint8_t *addr = (const uint8_t *)data;
	int rows, i, line;
	uint32_t seq;

	if (!ring)
		return;

	rows = words16 / w;
	if (rows > h)
		rows = h;
	if (!rows)
		return;

	if (w != CELL_W || x % CELL_W || (y - first_line) % CELL_H
	    || y % CELL_H || x + w > 1024) {
		invalidate(x, y, w, rows);
		return;
	}

	for (i = 0, line = y; i + CELL_H <= rows && line + CELL_H <= 512;
	     i += CELL_H, line += CELL_H, addr += 768) {
		seq = lookup_mb(addr);
		grid[line / CELL_H][x / CELL_W] = seq;
	}

	if (i < rows)
		invalidate(x, line, w, rows - i);
}

/* Plain 32-bit stores, the way the rest of bloom feeds the SQs; KOS's
 * sq_fast_cpy() faulted here */
static inline void sq_put_mb(uint32_t *d, const uint8_t *mb)
{
	const uint32_t *s = (const uint32_t *)mb;
	unsigned int n;

	for (n = MB_SIZE / 32; n; n--, s += 8) {
		d[0] = s[0];
		d[1] = s[1];
		d[2] = s[2];
		d[3] = s[3];
		d[4] = s[4];
		d[5] = s[5];
		d[6] = s[6];
		d[7] = s[7];
		sq_flush(d);
	}
}

bool mdec_yuv_upload(pvr_ptr_t tex, int offset, int w, int h,
		     unsigned int *tex_w, unsigned int *tex_h)
{
	unsigned int cols, rows, tw, th, stride, r, c, i;
	int src_x, src_y;
	uint32_t *d, seq;
	const uint8_t *mb;

	if (!ring || offset & 1)
		goto miss;

	src_x = (offset / 2) % 1024;
	src_y = (offset / 2) / 1024;
	cols = (w + 15) / 16;
	rows = (h + 15) / 16;

	if (src_x % CELL_W || src_y % CELL_H
	    || src_x / CELL_W + cols > GRID_COLS
	    || src_y / CELL_H + rows > GRID_ROWS)
		goto miss;

	src_x /= CELL_W;
	src_y /= CELL_H;

	for (r = 0; r < rows; r++)
		for (c = 0; c < cols; c++) {
			seq = grid[src_y + r][src_x + c];
			if (!seq || !seq_alive(seq - 1))
				goto miss;
		}

	tw = cols * 16 <= 512 ? 512 : 1024;
	th = rows * 16 <= 256 ? 256 : 512;
	stride = tw / 16;

	PVR_SET(PVR_YUV_ADDR, (uint32_t)tex & 0xffffff);
	PVR_SET(PVR_YUV_CFG, ((rows - 1) << 8) | (stride - 1));
	PVR_GET(PVR_YUV_CFG);

	d = sq_lock((void *)PVR_TA_YUV_CONV);

	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			seq = grid[src_y + r][src_x + c] - 1;
			mb = ring + (seq & (RING_SIZE - 1)) * MB_SIZE;
			sq_put_mb(d, mb);
		}

		for (i = cols * (MB_SIZE / 32); i < stride * (MB_SIZE / 32); i++)
			sq_flush(d);
	}

	sq_unlock();

	*tex_w = tw;
	*tex_h = th;
	mdec_yuv_skip_rgb = true;
	mdec_yuv_frames++;
	return true;

miss:
	mdec_yuv_skip_rgb = false;
	mdec_yuv_misses++;
	return false;
}

void mdec_yuv_stop(void)
{
	mdec_yuv_skip_rgb = false;
}
