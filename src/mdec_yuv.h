/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __BLOOM_MDEC_YUV_H__
#define __BLOOM_MDEC_YUV_H__

#include <stdbool.h>
#include <stdint.h>

#include <dc/pvr.h>

extern bool mdec_yuv_skip_rgb;
extern unsigned int mdec_yuv_frames, mdec_yuv_misses;

/* Slot for the YUV420 copy of a macroblock whose RGB goes to 'out' */
uint8_t *mdec_yuv_mb(const uint8_t *out);

/* A VRAM write of whole lines from 'data', starting at line y of a transfer
 * that started at first_line */
void mdec_yuv_vram_write(const uint16_t *data, int words16, int x, int y,
			 int w, int h, int first_line);

bool mdec_yuv_upload(pvr_ptr_t tex, int offset, int w, int h,
		     unsigned int *tex_w, unsigned int *tex_h);

void mdec_yuv_stop(void);

#endif /* __BLOOM_MDEC_YUV_H__ */
