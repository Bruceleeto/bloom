/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Direct SH-4 emission for lightrec. See sh4-jit.h for why this exists.
 */

#include <string.h>

#include "sh4-jit.h"

void sh4_jit_init(struct sh4_jit *j, void *buf, size_t len)
{
	memset(j, 0, sizeof(*j));
	sh4_emit_init(&j->e, buf, len);
}

static int fail(struct sh4_jit *j)
{
	j->failed = 1;
	return -1;
}

int sh4_label_forward(struct sh4_jit *j)
{
	if (j->nlabels >= SH4_MAX_LABELS)
		return fail(j);

	j->labels[j->nlabels] = NULL;
	return j->nlabels++;
}

void sh4_label_here(struct sh4_jit *j, int label)
{
	if (label < 0 || label >= j->nlabels) {
		fail(j);
		return;
	}
	j->labels[label] = j->e.pc;
}

int sh4_label(struct sh4_jit *j)
{
	int l = sh4_label_forward(j);

	if (l >= 0)
		sh4_label_here(j, l);
	return l;
}

static void add_patch(struct sh4_jit *j, uint16_t *at, int label,
		      uint32_t literal, enum sh4_patch_kind kind)
{
	if (j->npatches >= SH4_MAX_PATCHES) {
		fail(j);
		return;
	}

	j->patches[j->npatches].at = at;
	j->patches[j->npatches].label = label;
	j->patches[j->npatches].literal = literal;
	j->patches[j->npatches].kind = kind;
	j->npatches++;
}

/* --- constants ---
 *
 * `mov #imm,Rn` only carries a sign-extended byte. Anything wider is loaded
 * from a literal placed after the code, reached by `mov.l @(disp,PC),Rn`.
 *
 * The PC-relative displacement is unsigned, 8 bits, scaled by 4, and measured
 * from (PC & ~3) + 4, so a literal has to sit within about 1 KB *after* the
 * instruction. The pool is therefore flushed at the end of the block, and a
 * block longer than that reach fails the emit rather than silently encoding a
 * wrong displacement - sh4_jit_finish() reports it.
 */
void sh4_movi(struct sh4_jit *j, int rd, uint32_t imm)
{
	int32_t s = (int32_t)imm;

	/* Byte-sized constants go inline. */
	if (s >= -128 && s <= 127) {
		SH4_MOVI(&j->e, rd, (unsigned)(s & 0xff));
		return;
	}

	/* A few common shapes are cheaper to build than to load. */
	if (imm == 0xffff) {
		SH4_MOVI(&j->e, rd, 0xff);	/* -1 */
		SH4_EXTUW(&j->e, rd, rd);
		return;
	}
	if (imm == 0xff) {
		SH4_MOVI(&j->e, rd, 0xff);
		SH4_EXTUB(&j->e, rd, rd);
		return;
	}

	add_patch(j, j->e.pc, -1, imm, SH4_PATCH_LITERAL);
	SH4_LDPL(&j->e, rd, 0);		/* displacement filled in later */
}

void sh4_movr(struct sh4_jit *j, int rd, int rs)
{
	if (rd != rs)
		SH4_MOV(&j->e, rd, rs);
}

/* --- arithmetic ---
 *
 * SH-4's ALU is two-operand: the destination is also the first source. Every
 * three-operand form below therefore moves first, and has to be careful when
 * the destination aliases the *second* source, where a naive move would
 * destroy it before it is read.
 */
void sh4_addr(struct sh4_jit *j, int rd, int a, int b)
{
	if (rd == b) {
		SH4_ADD(&j->e, rd, a);	/* add is commutative */
		return;
	}
	sh4_movr(j, rd, a);
	SH4_ADD(&j->e, rd, b);
}

void sh4_addi(struct sh4_jit *j, int rd, int a, int32_t imm)
{
	if (imm >= -128 && imm <= 127) {
		sh4_movr(j, rd, a);
		if (imm)
			SH4_ADDI(&j->e, rd, (unsigned)(imm & 0xff));
		return;
	}

	/* Wider constants need a register. r0 is the only one guaranteed free,
	 * and add is commutative, so it can hold the immediate. */
	sh4_movi(j, SH4_SCRATCH, (uint32_t)imm);
	sh4_movr(j, rd, a);
	SH4_ADD(&j->e, rd, SH4_SCRATCH);
}

void sh4_subr(struct sh4_jit *j, int rd, int a, int b)
{
	if (rd == b && rd != a) {
		/* sub is not commutative: rd = a - rd. Stage through r0. */
		sh4_movr(j, SH4_SCRATCH, a);
		SH4_SUB(&j->e, SH4_SCRATCH, b);
		sh4_movr(j, rd, SH4_SCRATCH);
		return;
	}
	sh4_movr(j, rd, a);
	SH4_SUB(&j->e, rd, b);
}

void sh4_mulr(struct sh4_jit *j, int rd, int a, int b)
{
	/* mul.l puts the low 32 bits of the product in MACL. */
	SH4_MULL(&j->e, a, b);
	SH4_STSMACL(&j->e, rd);
}

#define BITWISE(name, insn)						\
void sh4_##name(struct sh4_jit *j, int rd, int a, int b)		\
{									\
	if (rd == b) {							\
		insn(&j->e, rd, a);					\
		return;							\
	}								\
	sh4_movr(j, rd, a);						\
	insn(&j->e, rd, b);						\
}
BITWISE(andr, SH4_AND)
BITWISE(orr,  SH4_OR)
BITWISE(xorr, SH4_XOR)

void sh4_comr(struct sh4_jit *j, int rd, int a)
{
	SH4_NOT(&j->e, rd, a);
}

void sh4_negr(struct sh4_jit *j, int rd, int a)
{
	SH4_NEG(&j->e, rd, a);
}

void sh4_extr_c(struct sh4_jit *j, int rd, int a)  { SH4_EXTSB(&j->e, rd, a); }
void sh4_extr_uc(struct sh4_jit *j, int rd, int a) { SH4_EXTUB(&j->e, rd, a); }
void sh4_extr_s(struct sh4_jit *j, int rd, int a)  { SH4_EXTSW(&j->e, rd, a); }
void sh4_extr_us(struct sh4_jit *j, int rd, int a) { SH4_EXTUW(&j->e, rd, a); }

/* --- shifts ---
 *
 * Constant shifts have single-instruction forms only for 1, 2, 8 and 16.
 * Anything else is built from those, which beats materialising a count and
 * using the dynamic form for the small shifts that dominate.
 */
static void shift_const(struct sh4_jit *j, int rd, int32_t n,
			void (*by1)(struct sh4_emit *, unsigned),
			int has_wide,
			void (*by2)(struct sh4_emit *, unsigned),
			void (*by8)(struct sh4_emit *, unsigned),
			void (*by16)(struct sh4_emit *, unsigned))
{
	while (n > 0) {
		if (has_wide && n >= 16) {
			by16(&j->e, rd);
			n -= 16;
		} else if (has_wide && n >= 8) {
			by8(&j->e, rd);
			n -= 8;
		} else if (has_wide && n >= 2) {
			by2(&j->e, rd);
			n -= 2;
		} else {
			by1(&j->e, rd);
			n -= 1;
		}
	}
}

static void w_shll(struct sh4_emit *e, unsigned r)   { SH4_SHLL(e, r); }
static void w_shll2(struct sh4_emit *e, unsigned r)  { SH4_SHLL2(e, r); }
static void w_shll8(struct sh4_emit *e, unsigned r)  { SH4_SHLL8(e, r); }
static void w_shll16(struct sh4_emit *e, unsigned r) { SH4_SHLL16(e, r); }
static void w_shlr(struct sh4_emit *e, unsigned r)   { SH4_SHLR(e, r); }
static void w_shlr2(struct sh4_emit *e, unsigned r)  { SH4_SHLR2(e, r); }
static void w_shlr8(struct sh4_emit *e, unsigned r)  { SH4_SHLR8(e, r); }
static void w_shlr16(struct sh4_emit *e, unsigned r) { SH4_SHLR16(e, r); }
static void w_shar(struct sh4_emit *e, unsigned r)   { SH4_SHAR(e, r); }

void sh4_lshi(struct sh4_jit *j, int rd, int a, int32_t n)
{
	sh4_movr(j, rd, a);
	shift_const(j, rd, n, w_shll, 1, w_shll2, w_shll8, w_shll16);
}

void sh4_rshi_u(struct sh4_jit *j, int rd, int a, int32_t n)
{
	sh4_movr(j, rd, a);
	shift_const(j, rd, n, w_shlr, 1, w_shlr2, w_shlr8, w_shlr16);
}

void sh4_rshi(struct sh4_jit *j, int rd, int a, int32_t n)
{
	/* Arithmetic right shift has only a by-1 form, so a large constant
	 * shift is cheaper through the dynamic instruction. */
	if (n > 3) {
		sh4_movi(j, SH4_SCRATCH, (uint32_t)(-n));
		sh4_movr(j, rd, a);
		SH4_SHAD(&j->e, rd, SH4_SCRATCH);
		return;
	}
	sh4_movr(j, rd, a);
	shift_const(j, rd, n, w_shar, 0, NULL, NULL, NULL);
}

/* Dynamic shifts: shld/shad shift left for a positive count and right for a
 * negative one, so a right shift has to negate the count first. */
void sh4_lshr(struct sh4_jit *j, int rd, int a, int b)
{
	if (rd == b) {
		sh4_movr(j, SH4_SCRATCH, b);
		sh4_movr(j, rd, a);
		SH4_SHLD(&j->e, rd, SH4_SCRATCH);
		return;
	}
	sh4_movr(j, rd, a);
	SH4_SHLD(&j->e, rd, b);
}

static void shift_right_dyn(struct sh4_jit *j, int rd, int a, int b, int arith)
{
	SH4_NEG(&j->e, SH4_SCRATCH, b);
	sh4_movr(j, rd, a);
	if (arith)
		SH4_SHAD(&j->e, rd, SH4_SCRATCH);
	else
		SH4_SHLD(&j->e, rd, SH4_SCRATCH);
}

void sh4_rshr(struct sh4_jit *j, int rd, int a, int b)
{
	shift_right_dyn(j, rd, a, b, 1);
}

void sh4_rshr_u(struct sh4_jit *j, int rd, int a, int b)
{
	shift_right_dyn(j, rd, a, b, 0);
}

/* --- memory ---
 *
 * The displacement forms are narrow: 4 bits for the word form (scaled by 4,
 * so 0-60) and 4 bits for byte and halfword, which additionally accept only
 * r0. Anything else uses the indexed form @(r0,Rm), which reaches anywhere at
 * the cost of the scratch register.
 */
static int disp_fits_l(int32_t d) { return d >= 0 && d <= 60 && !(d & 3); }
static int disp_fits_w(int32_t d) { return d >= 0 && d <= 30 && !(d & 1); }
static int disp_fits_b(int32_t d) { return d >= 0 && d <= 15; }

void sh4_ldxi_i(struct sh4_jit *j, int rd, int rb, int32_t disp)
{
	if (disp_fits_l(disp)) {
		SH4_LDDL(&j->e, rd, rb, disp / 4);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_LDRL(&j->e, rd, rb);
}

void sh4_stxi_i(struct sh4_jit *j, int32_t disp, int rb, int rs)
{
	if (disp_fits_l(disp)) {
		SH4_STDL(&j->e, rb, rs, disp / 4);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_STRL(&j->e, rb, rs);
}

/* The byte and halfword displacement forms only reach r0, so they are worth
 * taking only when the destination is not r0 itself and the base is not r0
 * either - otherwise the move needed to get the value out costs what the
 * indexed form costs anyway. */
void sh4_ldxi_c(struct sh4_jit *j, int rd, int rb, int32_t disp)
{
	if (disp_fits_b(disp) && rb != SH4_SCRATCH) {
		SH4_LDDB(&j->e, rb, disp);	/* sign-extends into r0 */
		sh4_movr(j, rd, SH4_SCRATCH);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_LDRB(&j->e, rd, rb);		/* sign-extends into rd */
}

void sh4_ldxi_uc(struct sh4_jit *j, int rd, int rb, int32_t disp)
{
	sh4_ldxi_c(j, rd, rb, disp);
	SH4_EXTUB(&j->e, rd, rd);
}

void sh4_ldxi_s(struct sh4_jit *j, int rd, int rb, int32_t disp)
{
	if (disp_fits_w(disp) && rb != SH4_SCRATCH) {
		SH4_LDDW(&j->e, rb, disp / 2);
		sh4_movr(j, rd, SH4_SCRATCH);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_LDRW(&j->e, rd, rb);
}

void sh4_ldxi_us(struct sh4_jit *j, int rd, int rb, int32_t disp)
{
	sh4_ldxi_s(j, rd, rb, disp);
	SH4_EXTUW(&j->e, rd, rd);
}

void sh4_stxi_c(struct sh4_jit *j, int32_t disp, int rb, int rs)
{
	if (disp_fits_b(disp) && rs != SH4_SCRATCH && rb != SH4_SCRATCH) {
		sh4_movr(j, SH4_SCRATCH, rs);
		SH4_STDB(&j->e, rb, disp);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_STRB(&j->e, rb, rs);
}

void sh4_stxi_s(struct sh4_jit *j, int32_t disp, int rb, int rs)
{
	if (disp_fits_w(disp) && rs != SH4_SCRATCH && rb != SH4_SCRATCH) {
		sh4_movr(j, SH4_SCRATCH, rs);
		SH4_STDW(&j->e, rb, disp / 2);
		return;
	}
	sh4_movi(j, SH4_SCRATCH, (uint32_t)disp);
	SH4_STRW(&j->e, rb, rs);
}

/* GBR-relative: 8-bit displacement scaled by 4, so 0-1020, r0 only. This is
 * why lightrec_state's hot fields are kept inside 1020 bytes. */
void sh4_ldxi_gbr(struct sh4_jit *j, int rd, int32_t disp)
{
	if (disp < 0 || disp > 1020 || (disp & 3)) {
		fail(j);
		return;
	}
	SH4_GBRLDL(&j->e, disp / 4);
	sh4_movr(j, rd, SH4_SCRATCH);
}

void sh4_stxi_gbr(struct sh4_jit *j, int32_t disp, int rs)
{
	if (disp < 0 || disp > 1020 || (disp & 3)) {
		fail(j);
		return;
	}
	sh4_movr(j, SH4_SCRATCH, rs);
	SH4_GBRSTL(&j->e, disp / 4);
}

/* --- comparisons ---
 *
 * SH-4 comparisons set the T bit; movt materialises it. The operand order is
 * chosen so no case needs T inverted: `a < b` is emitted as `b > a`.
 */
void sh4_eqr(struct sh4_jit *j, int rd, int a, int b)
{
	SH4_CMPEQ(&j->e, a, b);
	SH4_MOVT(&j->e, rd);
}

void sh4_ltr(struct sh4_jit *j, int rd, int a, int b)
{
	SH4_CMPGT(&j->e, b, a);		/* T = (b > a) = (a < b) */
	SH4_MOVT(&j->e, rd);
}

void sh4_ltr_u(struct sh4_jit *j, int rd, int a, int b)
{
	SH4_CMPHI(&j->e, b, a);
	SH4_MOVT(&j->e, rd);
}

/* --- branches ---
 *
 * bt/bf carry a signed 8-bit displacement, which reaches about 256
 * instructions. Emission is single pass, so a branch that turns out not to
 * reach cannot be widened in place; instead sh4_jit_finish() reports it and
 * the caller re-emits with long_branches set, which puts the condition on a
 * short branch over a bra. Blocks needing that are rare, so the common case
 * stays two instructions.
 */
static int emit_cond_branch(struct sh4_jit *j, int on_true)
{
	int label = sh4_label_forward(j);

	if (label < 0)
		return label;

	if (j->long_branches) {
		/* Skip over the bra when the condition is *not* taken. The
		 * inner branch has a fixed, always-reachable distance. */
		if (on_true)
			SH4_BF(&j->e, 1);
		else
			SH4_BT(&j->e, 1);

		add_patch(j, j->e.pc, label, 0, SH4_PATCH_BRA);
		SH4_BRA(&j->e, 0);
		SH4_NOP(&j->e);			/* delay slot */
		return label;
	}

	add_patch(j, j->e.pc, label, 0, SH4_PATCH_COND);
	if (on_true)
		SH4_BT(&j->e, 0);
	else
		SH4_BF(&j->e, 0);
	return label;
}

int sh4_beqr(struct sh4_jit *j, int a, int b)
{
	SH4_CMPEQ(&j->e, a, b);
	return emit_cond_branch(j, 1);
}

int sh4_bner(struct sh4_jit *j, int a, int b)
{
	SH4_CMPEQ(&j->e, a, b);
	return emit_cond_branch(j, 0);
}

int sh4_bgtr(struct sh4_jit *j, int a, int b)
{
	SH4_CMPGT(&j->e, a, b);		/* T = a > b */
	return emit_cond_branch(j, 1);
}

int sh4_bler(struct sh4_jit *j, int a, int b)
{
	SH4_CMPGT(&j->e, a, b);
	return emit_cond_branch(j, 0);
}

int sh4_bger(struct sh4_jit *j, int a, int b)
{
	SH4_CMPGE(&j->e, a, b);		/* T = a >= b */
	return emit_cond_branch(j, 1);
}

int sh4_bltr(struct sh4_jit *j, int a, int b)
{
	SH4_CMPGE(&j->e, a, b);
	return emit_cond_branch(j, 0);
}

int sh4_bgtr_u(struct sh4_jit *j, int a, int b)
{
	SH4_CMPHI(&j->e, a, b);
	return emit_cond_branch(j, 1);
}

int sh4_bltr_u(struct sh4_jit *j, int a, int b)
{
	SH4_CMPHS(&j->e, a, b);		/* T = a >= b, unsigned */
	return emit_cond_branch(j, 0);
}

int sh4_b(struct sh4_jit *j)
{
	int label = sh4_label_forward(j);

	if (label < 0)
		return label;

	add_patch(j, j->e.pc, label, 0, SH4_PATCH_BRA);
	SH4_BRA(&j->e, 0);
	SH4_NOP(&j->e);			/* delay slot */
	return label;
}

void sh4_jmpr(struct sh4_jit *j, int r)
{
	SH4_JMP(&j->e, r);
	SH4_NOP(&j->e);			/* delay slot */
}

/* --- finishing ---
 *
 * Literals go after all the code, so every PC-relative load reaches forward.
 * Identical values share a slot: constants repeat heavily in emitted code and
 * the pool competes with the block itself for icache.
 */
static uint16_t *emit_literals(struct sh4_jit *j)
{
	uint32_t *pool;
	int i, k, n = 0;
	uint32_t vals[SH4_MAX_PATCHES];
	int slot[SH4_MAX_PATCHES];

	for (i = 0; i < j->npatches; i++) {
		if (j->patches[i].kind != SH4_PATCH_LITERAL)
			continue;

		for (k = 0; k < n; k++)
			if (vals[k] == j->patches[i].literal)
				break;
		if (k == n)
			vals[n++] = j->patches[i].literal;
		slot[i] = k;
	}

	if (!n)
		return NULL;

	/* mov.l @(disp,PC) reads from (PC & ~3) + 4 + disp*4, so the pool has
	 * to start 4-byte aligned. */
	if (((uintptr_t)j->e.pc & 3) != 0)
		SH4_NOP(&j->e);

	pool = (uint32_t *)j->e.pc;
	for (k = 0; k < n; k++) {
		if ((uint16_t *)(pool + k + 1) > j->e.end) {
			fail(j);
			return NULL;
		}
		pool[k] = vals[k];
	}
	j->e.pc = (uint16_t *)(pool + n);

	for (i = 0; i < j->npatches; i++) {
		uintptr_t base, disp;

		if (j->patches[i].kind != SH4_PATCH_LITERAL)
			continue;

		base = ((uintptr_t)j->patches[i].at & ~(uintptr_t)3) + 4;
		disp = ((uintptr_t)(pool + slot[i]) - base) / 4;

		if (disp > 0xff) {
			/* Out of reach - the block is too long for a
			 * PC-relative pool. Reported, never mis-encoded. */
			fail(j);
			return NULL;
		}
		*j->patches[i].at |= (uint16_t)disp;
	}

	return (uint16_t *)pool;
}

size_t sh4_jit_finish(struct sh4_jit *j)
{
	int i;

	emit_literals(j);

	for (i = 0; i < j->npatches; i++) {
		struct sh4_patch *p = &j->patches[i];
		intptr_t disp;
		uint16_t *target;

		if (p->kind == SH4_PATCH_LITERAL)
			continue;

		if (p->label < 0 || p->label >= j->nlabels ||
		    !j->labels[p->label]) {
			fail(j);
			continue;
		}

		target = j->labels[p->label];
		disp = (target - (p->at + 2)) ;

		if (p->kind == SH4_PATCH_BRA) {
			if (disp < -2048 || disp > 2047) {
				fail(j);
				continue;
			}
			*p->at |= (uint16_t)(disp & 0xfff);
		} else {
			if (disp < -128 || disp > 127) {
				/* Distinguished from a hard failure: the
				 * caller can re-emit with long_branches. */
				j->branch_overflow = 1;
				fail(j);
				continue;
			}
			*p->at |= (uint16_t)(disp & 0xff);
		}
	}

	if (j->failed || j->e.overflow)
		return 0;

	return sh4_emit_offset(&j->e);
}
