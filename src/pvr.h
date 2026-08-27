/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PowerVR powered hardware renderer - gpulib interface
 *
 * Copyright (C) 2024 Paul Cercueil <paul@crapouillou.net>
 */

#ifndef __BLOOM_PVR_H
#define __BLOOM_PVR_H

#include <stdint.h>

extern float screen_fw, screen_fh;

void pvr_renderer_init(void);
void pvr_renderer_shutdown(void);

void hw_render_start(void);
void hw_render_stop(void);

void invalidate_all_textures(void);

/* Presented-frame counters, reset by whoever prints them. */
extern unsigned int pvr_commits, pvr_drops;

/* W ruler: host time the guest spends producing a frame, from the first
 * vblank after its previous GP1(05) (when it leaves its VSync wait) to the
 * next GP1(05). Under the wall clock fps is quantised to whole vblanks and
 * cannot show progress; this can. Sum/max in microseconds over the window,
 * reset by whoever prints them. */
extern uint64_t pvr_work_us_sum, pvr_work_us_max;
extern unsigned int pvr_work_frames;
void pvr_vblank_tick(void);

#endif /* __BLOOM_PVR_H */
