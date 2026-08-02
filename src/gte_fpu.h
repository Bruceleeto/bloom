/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The GTE, on the SH-4 FPU.  Built only when WITH_FPU_GTE is set.
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
