// SPDX-License-Identifier: GPL-2.0-only
/*
 * On-screen text on the PVR: a 128x128 bitmap font (DMS-Engine's, embedded
 * as a .dt texture) and one textured quad per glyph, submitted into the
 * translucent list at the end of the frame.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <dc/pvr.h>
#include <pvrtex/file_dctex.h>

#include "overlay.h"
#include "font_data.c"

#define FONT_SIZE	10		/* glyph height in the texture */
#define TEXT_SCALE	3.0f	/* integer: keeps the bitmap crisp */
#define TEXT_X		12.0f
#define TEXT_Y		12.0f

static const unsigned char char_widths[224] = {
	3, 1, 4, 6, 5, 7, 6, 2, 3, 3, 5, 5, 2, 4, 1, 7, 5, 2, 5, 5, 5, 5, 5, 5, 5, 5, 1, 1, 3, 4, 3, 6,
	7, 6, 6, 6, 6, 6, 6, 6, 6, 3, 5, 6, 5, 7, 6, 6, 6, 6, 6, 6, 7, 6, 7, 7, 6, 6, 6, 2, 7, 2, 3, 5,
	2, 5, 5, 5, 5, 5, 4, 5, 5, 1, 2, 5, 2, 5, 5, 5, 5, 5, 5, 5, 4, 5, 5, 5, 5, 5, 5, 3, 1, 3, 4, 4,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 5, 5, 5, 7, 1, 5, 3, 7, 3, 5, 4, 1, 7, 4, 3, 5, 3, 3, 2, 5, 6, 1, 2, 2, 3, 5, 6, 6, 6, 6,
	6, 6, 6, 6, 6, 6, 7, 6, 6, 6, 6, 6, 3, 3, 3, 3, 7, 6, 6, 6, 6, 6, 6, 5, 6, 6, 6, 6, 6, 6, 4, 6,
	5, 5, 5, 5, 5, 5, 9, 5, 5, 5, 5, 5, 2, 2, 3, 3, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 3, 5,
};

static struct { float u1, v1, u2, v2; } glyph[224];
static pvr_poly_hdr_t hdr;
static pvr_ptr_t tex;
static char text[128];
static int ready;

void overlay_init(void)
{
	const fDtHeader *h = (const fDtHeader *)font_dt;
	size_t hs = fDtGetHeaderSize(h), ts = fDtGetTotalSize(h) - hs;
	pvr_poly_cxt_t cxt;
	unsigned int i, x = 1, y = 1;

	tex = pvr_mem_malloc(ts);
	if (!tex) {
		printf("overlay: no VRAM for font\n");
		return;
	}
	pvr_txr_load(font_dt + hs, tex, ts);

	pvr_poly_cxt_txr(&cxt, PVR_LIST_TR_POLY, h->pvr_type & 0xffc00000,
			 fDtGetPvrWidth(h), fDtGetPvrHeight(h), tex, PVR_FILTER_NONE);
	cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;
	cxt.depth.write = PVR_DEPTHWRITE_DISABLE;
	cxt.gen.culling = PVR_CULLING_NONE;
	pvr_poly_compile(&hdr, &cxt);

	for (i = 0; i < 224; i++) {
		unsigned int w = char_widths[i];

		if (x + w + 1 > 128) {
			x = 1;
			y += FONT_SIZE + 1;
		}
		glyph[i].u1 = x / 128.0f;
		glyph[i].v1 = y / 128.0f;
		glyph[i].u2 = (x + w) / 128.0f;
		glyph[i].v2 = (y + FONT_SIZE) / 128.0f;
		x += w + 1;
	}
	ready = 1;
}

void overlay_set(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(text, sizeof(text), fmt, ap);
	va_end(ap);
}

static void quad(float x0, float y0, float x1, float y1,
		 float u0, float v0, float u1, float v1, float z, uint32_t argb)
{
	static const float xs[4] = { 0, 1, 0, 1 }, ys[4] = { 0, 0, 1, 1 };
	pvr_vertex_t *v;
	unsigned int i;

	for (i = 0; i < 4; i++) {
		v = pvr_dr_target();
		*v = (pvr_vertex_t){
			.flags = i == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX,
			.x = xs[i] ? x1 : x0, .y = ys[i] ? y1 : y0, .z = z,
			.u = xs[i] ? u1 : u0, .v = ys[i] ? v1 : v0,
			.argb = argb,
		};
		pvr_dr_commit(v);
	}
}

void overlay_draw(float z)
{
	const float h = FONT_SIZE * TEXT_SCALE;
	const float lh = (FONT_SIZE + 2) * TEXT_SCALE;
	float x, y;
	pvr_poly_hdr_t *sq;
	const char *p;
	int pass;

	if (!ready || !text[0])
		return;

	sq = pvr_dr_target();
	*sq = hdr;
	pvr_dr_commit(sq);

	/* shadow first, then the text */
	for (pass = 0; pass < 2; pass++) {
		float ox = pass ? 0.0f : 3.0f, oy = pass ? 0.0f : 3.0f;
		uint32_t argb = pass ? 0xff009e2f : 0xff000000; /* raylib LIME, as DrawFPS() */

		x = TEXT_X;
		y = TEXT_Y;
		for (p = text; *p; p++) {
			int c = *p - 32;
			float w;

			if (*p == '\n') {
				x = TEXT_X;
				y += lh;
				continue;
			}
			if (c < 0 || c >= 224)
				continue;
			w = char_widths[c] * TEXT_SCALE;
			quad(x + ox, y + oy, x + ox + w, y + oy + h,
			     glyph[c].u1, glyph[c].v1, glyph[c].u2, glyph[c].v2,
			     z, argb);
			x += (char_widths[c] + 2) * TEXT_SCALE;
		}
	}
}
