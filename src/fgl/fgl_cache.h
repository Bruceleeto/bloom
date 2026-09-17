/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
/* The SH-4 cache model hooks.  On a workstation the emitted SH-4 is
 * interpreted and the caches simulated (proving ground fgl_cache.c); on a
 * Dreamcast they are real, so these compile to nothing.  A header rather
 * than an #ifdef at the call site so the two trees share fgl_lightrec.c. */

#ifndef FGL_CACHE_H
#define FGL_CACHE_H

#include <stdint.h>

static inline void fgl_cache_note_block(const void *code, unsigned size,
					uint32_t guest_pc)
{
	(void)code; (void)size; (void)guest_pc;
}

static inline void fgl_cache_forget(const void *code, unsigned size)
{
	(void)code; (void)size;
}

static inline void fgl_cache_call(const char *name) { (void)name; }

#endif /* FGL_CACHE_H */
