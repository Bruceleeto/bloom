/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * A direct SH-4 emitter for lightrec.
 *
 * Replaces GNU Lightning's node graph, register allocator and two-pass
 * emitter with single-pass emission. lightrec already owns register
 * allocation; what it could not own with lightning in the way was r0 and r3,
 * which lightning reserves for itself. r0 is unavoidable on SH-4 - the
 * GBR-relative, indexed and 8-bit-immediate forms accept no other register -
 * but it is only needed *within* one emitted operation, not reserved across
 * a whole block. r3 comes back entirely. That is the register the sixth pin
 * needs.
 *
 * Correctness is checked at two levels: sh4-encode.h is diffed against GNU as
 * by tools/lrtest/check-encoders.sh, and this layer is diffed against C
 * semantics by tools/lrtest/optest.c - the same suite GNU Lightning passes.
 */

#ifndef __SH4_JIT_H__
#define __SH4_JIT_H__

#include <stddef.h>
#include <stdint.h>

#include "sh4-encode.h"

/* Registers, in the ABI's terms rather than the emitter's. */
#define SH4_ARG0	4	/* r4-r7: incoming arguments, caller-saved */
#define SH4_NARGS	4
#define SH4_SAVED0	8	/* r8-r13: callee-saved */
#define SH4_NSAVED	6
#define SH4_FP		14
#define SH4_SP		15

/* Scratch. Architecturally forced for a large part of the ISA, so it is never
 * handed out to a caller and is clobbered by most operations here. */
#define SH4_SCRATCH	SH4_R0

#define SH4_MAX_LABELS	256
#define SH4_MAX_PATCHES	512

enum sh4_patch_kind {
	SH4_PATCH_BRA,		/* 12-bit displacement, bra/bsr */
	SH4_PATCH_COND,		/* 8-bit displacement, bt/bf */
	SH4_PATCH_LITERAL,	/* mov.l @(disp,PC) into the literal pool */
};

struct sh4_patch {
	uint16_t *at;		/* instruction to fix up */
	int label;		/* label it refers to, or -1 for a literal */
	uint32_t literal;	/* value, when kind is LITERAL */
	enum sh4_patch_kind kind;
};

struct sh4_jit {
	struct sh4_emit e;

	uint16_t *labels[SH4_MAX_LABELS];
	int nlabels;

	struct sh4_patch patches[SH4_MAX_PATCHES];
	int npatches;

	/* Set when a limit is hit - label, patch or code space. Checked once
	 * when the block is finished rather than at every call site. */
	int failed;

	/* Set when a conditional branch's 8-bit displacement did not reach.
	 * Emission is single-pass, so the fix is for the caller to re-emit
	 * the block with long_branches set; see sh4_jit_finish(). */
	int branch_overflow;
	int long_branches;
};

void sh4_jit_init(struct sh4_jit *j, void *buf, size_t len);

/* Finish emission: flush the literal pool and apply every patch. Returns the
 * number of bytes emitted, or 0 if anything overflowed. */
size_t sh4_jit_finish(struct sh4_jit *j);

int sh4_label(struct sh4_jit *j);		/* label here */
int sh4_label_forward(struct sh4_jit *j);	/* label to be placed later */
void sh4_label_here(struct sh4_jit *j, int label);

/* --- moves and constants --- */
void sh4_movr(struct sh4_jit *j, int rd, int rs);
void sh4_movi(struct sh4_jit *j, int rd, uint32_t imm);

/* --- arithmetic and logic --- */
void sh4_addr(struct sh4_jit *j, int rd, int a, int b);
void sh4_addi(struct sh4_jit *j, int rd, int a, int32_t imm);
void sh4_subr(struct sh4_jit *j, int rd, int a, int b);
void sh4_mulr(struct sh4_jit *j, int rd, int a, int b);
void sh4_andr(struct sh4_jit *j, int rd, int a, int b);
void sh4_orr(struct sh4_jit *j, int rd, int a, int b);
void sh4_xorr(struct sh4_jit *j, int rd, int a, int b);
void sh4_comr(struct sh4_jit *j, int rd, int a);
void sh4_negr(struct sh4_jit *j, int rd, int a);

/* --- extends --- */
void sh4_extr_c(struct sh4_jit *j, int rd, int a);
void sh4_extr_uc(struct sh4_jit *j, int rd, int a);
void sh4_extr_s(struct sh4_jit *j, int rd, int a);
void sh4_extr_us(struct sh4_jit *j, int rd, int a);

/* --- shifts --- */
void sh4_lshi(struct sh4_jit *j, int rd, int a, int32_t n);
void sh4_rshi(struct sh4_jit *j, int rd, int a, int32_t n);
void sh4_rshi_u(struct sh4_jit *j, int rd, int a, int32_t n);
void sh4_lshr(struct sh4_jit *j, int rd, int a, int b);
void sh4_rshr(struct sh4_jit *j, int rd, int a, int b);
void sh4_rshr_u(struct sh4_jit *j, int rd, int a, int b);

/* --- memory ---
 * Displacements outside the narrow encodable range fall back to the indexed
 * form, which costs the scratch register but has full range. */
void sh4_ldxi_i(struct sh4_jit *j, int rd, int rb, int32_t disp);
void sh4_ldxi_c(struct sh4_jit *j, int rd, int rb, int32_t disp);
void sh4_ldxi_uc(struct sh4_jit *j, int rd, int rb, int32_t disp);
void sh4_ldxi_s(struct sh4_jit *j, int rd, int rb, int32_t disp);
void sh4_ldxi_us(struct sh4_jit *j, int rd, int rb, int32_t disp);
void sh4_stxi_i(struct sh4_jit *j, int32_t disp, int rb, int rs);
void sh4_stxi_c(struct sh4_jit *j, int32_t disp, int rb, int rs);
void sh4_stxi_s(struct sh4_jit *j, int32_t disp, int rb, int rs);

/* GBR-relative, where lightrec keeps the emulator state. The displacement is
 * 8 bits scaled by the access width, and the architecture allows only r0 as
 * the other operand - which is why the state pointer lives in GBR at all. */
void sh4_ldxi_gbr(struct sh4_jit *j, int rd, int32_t disp);
void sh4_stxi_gbr(struct sh4_jit *j, int32_t disp, int rs);

/* --- comparisons, producing 0 or 1 --- */
void sh4_eqr(struct sh4_jit *j, int rd, int a, int b);
void sh4_ltr(struct sh4_jit *j, int rd, int a, int b);
void sh4_ltr_u(struct sh4_jit *j, int rd, int a, int b);

/* --- branches ---
 * Each returns the label it branches to, already allocated and unplaced;
 * call sh4_label_here() at the destination. */
int sh4_beqr(struct sh4_jit *j, int a, int b);
int sh4_bner(struct sh4_jit *j, int a, int b);
int sh4_bltr(struct sh4_jit *j, int a, int b);
int sh4_bler(struct sh4_jit *j, int a, int b);
int sh4_bgtr(struct sh4_jit *j, int a, int b);
int sh4_bger(struct sh4_jit *j, int a, int b);
int sh4_bltr_u(struct sh4_jit *j, int a, int b);
int sh4_bgtr_u(struct sh4_jit *j, int a, int b);
int sh4_b(struct sh4_jit *j);
void sh4_jmpr(struct sh4_jit *j, int r);

#endif /* __SH4_JIT_H__ */
