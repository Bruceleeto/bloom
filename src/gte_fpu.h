/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The GTE, on the SH-4 FPU.
 *
 * Nothing here knows which CPU core is calling it: a command takes the COP2
 * register file and the instruction word, so the interpreter and the
 * recompiler share one implementation.
 *
 * Copyright (C) 2026 bloom contributors
 */

#ifndef BLOOM_GTE_FPU_H
#define BLOOM_GTE_FPU_H

#include <libpcsxcore/gte.h>

/* One GTE command.  `op` is the whole instruction word. */
void gte_fpu_cmd(psxCP2Regs *r, u32 op);

/*
 * Compile-time dispatch for the recompiler: the function to call for `op`,
 * or NULL for a command the hardware treats as a no-op.  Also declared by
 * hand in deps/lightrec/emitter.c, which cannot see this header.
 */
void *gte_fpu_resolve(u32 op);

/* LEAF COMMANDS -- bleem's shape.  A command with a leaf is called by the
 * generated code directly (`jsr`), no shim, no C ABI: the routine finds the
 * file at GBR + FGL_AT_CP2D * 4 and preserves everything but r0, r1, PR, MAC
 * and T (gte_rtp.S).  Nonzero is the leaf's index; the recompiler maps it to
 * the routine's address (bloom) or to a token the interpreter runs (Linux). */
#define GTE_LEAF_NCLIP 1
#define GTE_LEAF_RTPS  2
#define GTE_LEAF_RTPT  3
#define GTE_LEAF_MVMVA 4
#define GTE_LEAF_DPCS  5
#define GTE_LEAF_INTPL 6
#define GTE_LEAF_SQR   7
#define GTE_LEAF_OP    8
#define GTE_LEAF_GPF   9
#define GTE_LEAF_GPL   10
#define GTE_LEAF_DCPL  11
#define GTE_LEAF_DPCT  12
#define GTE_LEAF_AVSZ3 13
#define GTE_LEAF_AVSZ4 14
#define GTE_LEAF_NCS   15
#define GTE_LEAF_NCT   16
#define GTE_LEAF_NCCS  17
#define GTE_LEAF_NCCT  18
#define GTE_LEAF_NCDS  19
#define GTE_LEAF_NCDT  20
#define GTE_LEAF_CC    21
#define GTE_LEAF_CDP   22
#define GTE_LEAF_N     23
/* Nonzero: the leaf index, | 1 when the leaf wants the op word in r1. */
int gte_fpu_leaf_cmd(u32 op);
/* The leaf's symbol name without the gte_/_leaf wrapping ("nclip"). */
const char *gte_fpu_leaf_name(int leaf);
/* The C-callable forms (self-test). */
void gte_leaf_call(int leaf, psxCP2Regs *r, u32 op);
void gte_nclip_leaf_call(psxCP2Regs *r);
void gte_rtps_leaf_call(psxCP2Regs *r);
void gte_rtpt_leaf_call(psxCP2Regs *r);

/* Whether the recompiler may emit NCLIP inline (refused when PROF/CENSUS
 * need every command to pass through the counted entry point). */
int gte_fpu_nclip_inline(void);

/* The two coordinate transforms, for the tests in docs/tests. */
void gte_fpu_rtps(psxCP2Regs *r);
void gte_fpu_rtpt(psxCP2Regs *r);

/*
 * Drop everything cached from the control file.  Required after anything that
 * writes CP2C without going through CTC2 - a reset, or a savestate load.
 */
void gte_fpu_reset(void);

/* Point the core's psxCP2[] table at this implementation. */
void gte_fpu_install(void);

#endif /* BLOOM_GTE_FPU_H */
