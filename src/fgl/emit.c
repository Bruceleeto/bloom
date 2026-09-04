/* The emit pass: IR nodes to SH-4.
 *
 * One linear walk. The allocation pass has already said where every operand
 * lives -- a pool register, or -1 for "still in the state block" -- so there
 * is one code path per node and the out-of-registers case is not a special
 * one, it just reads through r0 like any other state access.
 *
 * The contract this emits under is in fgl.h and is not negotiable here.
 */

#include "fgl.h"
#include "fgl_state.h"

#include <string.h>

/* Word index of the next instruction to be emitted. Displacement fixups are
 * recorded in these units, not bytes, because that is what patching wants. */
static int here(const fgl_emitter *e)
{
	return (int)((e->cg.ptr - e->start) / 2);
}

uint32_t fgl_size(const fgl_emitter *e)
{
	return (uint32_t)(e->cg.ptr - e->start);
}

void fgl_init(fgl_emitter *e, void *buf, uint32_t size, uint32_t base)
{
	memset(e, 0, sizeof(*e));
	e->cg.ptr = (uint8_t *)buf;
	e->cg.end = (uint8_t *)buf + size;
	e->start  = (uint8_t *)buf;
	e->base   = base;
}

/* OR bits into an already-emitted word: how every displacement that was not
 * known at emission time gets filled in. */
static void or_word_at(fgl_emitter *e, int at, uint16_t bits)
{
	uint8_t *p = e->start + 2 * at;
	uint16_t w = (uint16_t)(p[0] | (p[1] << 8));

	w |= bits;
	p[0] = (uint8_t)(w & 0xff);
	p[1] = (uint8_t)(w >> 8);
}

static int bt_fwd(fgl_emitter *e) { int s = here(e); sh4_emit_bt(&e->cg, 0); return s; }
static int bf_fwd(fgl_emitter *e) { int s = here(e); sh4_emit_bf(&e->cg, 0); return s; }
static int bra_fwd(fgl_emitter *e) { int s = here(e); sh4_emit_bra(&e->cg, 0); sh4_emit_nop(&e->cg); return s; }

static void patch_fwd8(fgl_emitter *e, int site)
{
	int disp = here(e) - site - 2;

	if (!sh4_disp8_fits(disp))
		e->overflow = 1;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xff));
}

static void patch_fwd12(fgl_emitter *e, int site)
{
	int disp = here(e) - site - 2;

	if (!sh4_disp12_fits(disp))
		e->overflow = 1;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xfff));
}

/* ---------------------------------------------------------------- */
/* Constants                                                         */
/* ---------------------------------------------------------------- */

/* Two tiers, and the cheap one is not an optimisation so much as the common
 * case: MIPS immediates are small far more often than not, and `mov #imm,Rn`
 * is one word with no pool entry and no load. */
static void emit_const(fgl_emitter *e, uint32_t v, int rn)
{
	int32_t s = (int32_t)v;

	if (s >= -128 && s <= 127) {
		sh4_emit_mov_imm(&e->cg, (int)s, rn);
		return;
	}

	if (e->n_fix >= FGL_MAX_LITERALS) {
		e->overflow = 1;
		return;
	}
	e->fix[e->n_fix].value = v;
	e->fix[e->n_fix].at = here(e);
	e->n_fix++;

	/* Displacement 0 for now; the pool pass ORs the real one in. */
	sh4_emit_mov_l_load_pc(&e->cg, 0, rn);
}

/* Place the literal pool and resolve every site that wants one.
 *
 * `mov.l @(disp,PC)` reads from (PC + 4) rounded down to a longword boundary,
 * so the pool must be longword aligned and every displacement computed from
 * the rounded address rather than from the site itself. */
static void emit_pool(fgl_emitter *e)
{
	uint32_t values[FGL_MAX_LITERALS];
	int n_values = 0;
	uint32_t pool_at;
	int i, k;

	if (!e->n_fix)
		return;

	if ((e->base + fgl_size(e)) & 2)
		sh4_emit_nop(&e->cg);           /* align the pool */
	pool_at = e->base + fgl_size(e);

	for (i = 0; i < e->n_fix; i++) {
		for (k = 0; k < n_values; k++)
			if (values[k] == e->fix[i].value)
				break;
		if (k == n_values)
			values[n_values++] = e->fix[i].value;
	}

	for (k = 0; k < n_values; k++) {
		sh4_word(&e->cg, (uint16_t)(values[k] & 0xffff));
		sh4_word(&e->cg, (uint16_t)(values[k] >> 16));
	}
	e->pool_bytes += 4u * (uint32_t)n_values;

	for (i = 0; i < e->n_fix; i++) {
		uint32_t site = e->base + 2u * (uint32_t)e->fix[i].at;
		uint32_t ref = (site + 4) & ~3u;
		uint32_t disp;

		for (k = 0; k < n_values; k++)
			if (values[k] == e->fix[i].value)
				break;

		disp = (pool_at + 4u * (uint32_t)k - ref) / 4u;
		if (disp > 255) {
			e->overflow = 1;        /* pool out of reach */
			return;
		}
		or_word_at(e, e->fix[i].at, (uint16_t)disp);
	}

	e->n_fix = 0;
}

/* A LITERAL POOL THAT DOES NOT HAVE TO SIT AT THE END.
 *
 * `mov.l @(disp,PC)` reaches 1020 bytes forward. A block of 32 guest
 * instructions that lower to twenty SH-4 each -- which the unaligned four do
 * -- is well past that, so a pool parked after the code is unreachable from
 * the top of the block and every site in it overflows.
 *
 * Rejecting the block is not an option: THERE IS NO FALLBACK PATH, so a block
 * fgl declines is a game that does not run. Instead the pool is flushed mid
 * block, jumped over, and started again. Cost is two instructions per flush,
 * paid only by blocks long enough to need one.
 *
 * The trigger is the OLDEST outstanding site, not the newest: that is the one
 * whose reach runs out first. */
static void emit_pool(fgl_emitter *e);

static void maybe_flush_pool(fgl_emitter *e)
{
	int over;

	if (!e->n_fix)
		return;

	/* 700 rather than 1020: the pool itself, and whatever the next node
	 * emits before the flush actually happens, both sit inside the gap. */
	if (2 * (here(e) - e->fix[0].at) < 700)
		return;

	over = bra_fwd(e);
	emit_pool(e);
	patch_fwd12(e, over);
	e->pool_flushes++;
}

/* ---------------------------------------------------------------- */
/* State-block traffic                                               */
/* ---------------------------------------------------------------- */

/* Guest register `g` into host register `rn`, and back. Two instructions,
 * not one, whenever `rn` is not r0 -- the GBR displacement form has no other
 * destination. This is the whole reason r0 is reserved. */
static void ld_guest(fgl_emitter *e, unsigned g, int rn)
{
	sh4_emit_mov_l_load_gbr(&e->cg, (int)GUEST_AT(g));
	if (rn != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, rn);
}

static void st_guest(fgl_emitter *e, unsigned g, int rn)
{
	if (rn != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, rn, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(g));
}

/* An operand, wherever the allocator left it. `-1` means the state block, and
 * `spare` is where to put it -- which the caller has to have kept free. */
static int operand(fgl_emitter *e, int host, unsigned g, int spare)
{
	if (host >= 0)
		return host;
	ld_guest(e, g, spare);
	return spare;
}

/* ---------------------------------------------------------------- */
/* Addresses                                                         */
/* ---------------------------------------------------------------- */

/* A guest address, `rs + imm`, masked and left in `dst`.
 *
 * THE MASK IS A REGISTER, NOT A CONSTANT. r13 holds it for the life of the
 * block, so every access costs one `and Rm,Rn` instead of materialising
 * 0x1fffffff and reloading it -- which, at one load and store per handful of
 * guest instructions, is the difference the pinning is there to buy.
 *
 * `other` is the emitter's other working register, used only when the
 * displacement is too wide for `add #imm`. The caller picks the pair, because
 * which of r0/r1 is free depends on whether a value is already in flight: a
 * load builds its address in r0, a store builds it in r1 and keeps r0 for the
 * value it is about to write.
 */
static void emit_addr(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	int32_t s = (int32_t)p->imm;

	if (p->hs >= 0) {
		if (p->hs != dst)
			sh4_emit_mov_reg(&e->cg, p->hs, dst);
	} else {
		ld_guest(e, p->rs, dst);
	}

	if (s) {
		if (s >= -128 && s <= 127) {
			sh4_emit_add_imm(&e->cg, (int)s, dst);
		} else {
			emit_const(e, p->imm, other);
			sh4_emit_add_reg(&e->cg, other, dst);
		}
	}

	sh4_emit_and(&e->cg, FGL_R_MASK, dst);
}

/* ---------------------------------------------------------------- */
/* Shifts                                                            */
/* ---------------------------------------------------------------- */

/* How many instructions the constant-shift decomposition would cost. SH-4
 * has 16/8/2/1-bit shifts but only in one direction each, so a shift by 23
 * is five instructions and `mov #imm` + `shld` is two. Pick per amount. */
static int shift_steps(unsigned k)
{
	return (int)(k / 16) + (int)((k % 16) / 8) + (int)((k % 8) / 2) + (int)(k % 2);
}

static void shift_const(fgl_emitter *e, int rn, unsigned k, int dir)
{
	unsigned n;

	if (!k)
		return;

	/* SH_RA has no multi-bit form at all, so it is always the dynamic
	 * one past a single bit. */
	if (dir == SH_RA) {
		if (k == 1) {
			sh4_emit_shar(&e->cg, rn);
			return;
		}
		sh4_emit_mov_imm(&e->cg, -(int)k, FGL_R_T1);
		sh4_emit_shad(&e->cg, FGL_R_T1, rn);
		return;
	}

	if (shift_steps(k) > 2) {
		sh4_emit_mov_imm(&e->cg, dir == SH_LL ? (int)k : -(int)k, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, rn);
		return;
	}

	for (n = k / 16; n--;) dir == SH_LL ? sh4_emit_shll16(&e->cg, rn) : sh4_emit_shlr16(&e->cg, rn);
	for (n = (k % 16) / 8; n--;) dir == SH_LL ? sh4_emit_shll8(&e->cg, rn) : sh4_emit_shlr8(&e->cg, rn);
	for (n = (k % 8) / 2; n--;) dir == SH_LL ? sh4_emit_shll2(&e->cg, rn) : sh4_emit_shlr2(&e->cg, rn);
	for (n = k % 2; n--;) dir == SH_LL ? sh4_emit_shll(&e->cg, rn) : sh4_emit_shlr(&e->cg, rn);
}

/* ---------------------------------------------------------------- */
/* ALU                                                               */
/* ---------------------------------------------------------------- */

/* rd = rs <op> rt on a two-operand machine.
 *
 * The destructive form means the destination has to start out holding rs, so
 * the only awkward case is rd already being rt -- there the move would
 * destroy the other operand, and a working register is used instead. */
static void alu_rr(fgl_emitter *e, int sub, int rd, int rs, int rt)
{
	int w = (rd == rt && rd != rs) ? FGL_R_T1 : rd;

	switch (sub) {
	case ALU_SLT:
	case ALU_SLTU:
		/* No move at all: the comparison reads both operands and the
		 * result is one bit out of T. */
		if (sub == ALU_SLT)
			sh4_emit_cmpgt(&e->cg, rs, rt);   /* T = rt > rs */
		else
			sh4_emit_cmphi(&e->cg, rs, rt);
		sh4_emit_movt(&e->cg, rd);
		return;
	default:
		break;
	}

	if (w != rs)
		sh4_emit_mov_reg(&e->cg, rs, w);

	switch (sub) {
	case ALU_ADD: sh4_emit_add_reg(&e->cg, rt, w); break;
	case ALU_SUB: sh4_emit_sub(&e->cg, rt, w); break;
	case ALU_AND: sh4_emit_and(&e->cg, rt, w); break;
	case ALU_OR:  sh4_emit_or(&e->cg, rt, w); break;
	case ALU_XOR: sh4_emit_xor(&e->cg, rt, w); break;
	case ALU_NOR: sh4_emit_or(&e->cg, rt, w); sh4_emit_not(&e->cg, w, w); break;
	default:      e->unsupported = 1; e->unsupported_op = sub; return;
	}

	if (w != rd)
		sh4_emit_mov_reg(&e->cg, w, rd);
}

/* rd = rs <op> imm. ADD has a one-word immediate form; the rest materialise
 * the constant, which is what the R0-only immediate ALU forms cost us. */
static void alu_ri(fgl_emitter *e, int sub, int rd, int rs, uint32_t imm)
{
	int32_t s = (int32_t)imm;

	if (sub == ALU_ADD && s >= -128 && s <= 127) {
		if (rd != rs)
			sh4_emit_mov_reg(&e->cg, rs, rd);
		if (s)
			sh4_emit_add_imm(&e->cg, (int)s, rd);
		return;
	}

	if (sub == ALU_SLT || sub == ALU_SLTU) {
		emit_const(e, imm, FGL_R_T1);
		if (sub == ALU_SLT)
			sh4_emit_cmpgt(&e->cg, rs, FGL_R_T1);
		else
			sh4_emit_cmphi(&e->cg, rs, FGL_R_T1);
		sh4_emit_movt(&e->cg, rd);
		return;
	}

	emit_const(e, imm, FGL_R_T1);
	alu_rr(e, sub, rd, rs, FGL_R_T1);
}

/* ---------------------------------------------------------------- */
/* The unaligned four                                                */
/* ---------------------------------------------------------------- */

/* LWL/LWR/SWL/SWR, the only guest accesses DEFINED on an unaligned address.
 *
 * All four work the same way: take the address, split it into the aligned
 * word below it and a byte offset, and use the offset to build a shift and a
 * mask. The shift distances are `8 * offset` and its complement `24 - 8n`,
 * which is why both are computed up front.
 *
 * These need a scratch register from the allocator -- the address survives
 * across the whole sequence, so r0 and r1 alone are not enough. It gives one
 * to LOAD_UN and two to STORE_UN (alloc.c:479-495).
 */

/* r0 = 8 * (address & 3), s0 = address & ~3. */
static void emit_unaligned_setup(fgl_emitter *e, const ir_node *p, int s0)
{
	emit_addr(e, p, FGL_R_XFER, FGL_R_T1);          /* r0 = address     */
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, s0);
	sh4_emit_and_imm(&e->cg, 3);                    /* r0 = offset      */
	sh4_emit_shll2(&e->cg, FGL_R_XFER);
	sh4_emit_shll(&e->cg, FGL_R_XFER);              /* r0 = 8 * offset  */
	sh4_emit_mov_imm(&e->cg, -4, FGL_R_T1);
	sh4_emit_and(&e->cg, FGL_R_T1, s0);             /* s0 = address & ~3 */
}

/* r1 = 24 - r0, the complementary shift distance. */
static void emit_comp_shift(fgl_emitter *e)
{
	sh4_emit_mov_imm(&e->cg, 24, FGL_R_T1);
	sh4_emit_sub(&e->cg, FGL_R_XFER, FGL_R_T1);
}

static void emit_load_un(fgl_emitter *e, const ir_node *p)
{
	int s0 = p->sc[0];
	int d;

	if (!s0) {
		e->overflow = 1;
		return;
	}

	emit_unaligned_setup(e, p, s0);
	sh4_emit_mov_l_load(&e->cg, s0, s0);            /* s0 = the word */

	if (p->sub == UN_LWL) {
		/* value = word << (24 - 8n), mask = 0x00ffffff >> 8n */
		emit_comp_shift(e);
		sh4_emit_shld(&e->cg, FGL_R_T1, s0);
		emit_const(e, 0x00ffffffu, FGL_R_T1);
		sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	} else {
		/* value = word >> 8n, mask = 0xffffff00 << (24 - 8n) */
		sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, s0);
		emit_comp_shift(e);
		sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
		emit_const(e, 0xffffff00u, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	}

	/* The destination keeps the bytes the load does not cover, so it is
	 * read as well as written -- the one node where that is true.
	 *
	 * Deferred, it is read and NOT written: the merge is built in r0 and
	 * parked, so the shadow instruction still sees the old register. */
	if (p->defer) {
		if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, p->hd, FGL_R_XFER);
		else
			ld_guest(e, p->rd, FGL_R_XFER);
		sh4_emit_and(&e->cg, FGL_R_T1, FGL_R_XFER);
		sh4_emit_or(&e->cg, s0, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		return;
	}

	d = p->hd;
	if (d < 0) {
		ld_guest(e, p->rd, FGL_R_XFER);
		d = FGL_R_XFER;
	}
	sh4_emit_and(&e->cg, FGL_R_T1, d);
	sh4_emit_or(&e->cg, s0, d);
	if (p->hd < 0)
		st_guest(e, p->rd, d);
}

static void emit_store_un(fgl_emitter *e, const ir_node *p)
{
	int s0 = p->sc[0], s1 = p->sc[1];
	int v;

	if (!s0 || !s1) {
		e->overflow = 1;
		return;
	}

	/* The value goes to a register before anything else, because from the
	 * address onwards r0 is spoken for. */
	v = operand(e, p->ht, p->rt, FGL_R_XFER);
	sh4_emit_mov_reg(&e->cg, v, s1);

	emit_unaligned_setup(e, p, s0);

	if (p->sub == UN_SWL) {
		/* keep = word & (0xffffff00 << 8n), value >>= 24 - 8n */
		emit_comp_shift(e);
		sh4_emit_neg(&e->cg, FGL_R_T1, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, s1);
		emit_const(e, 0xffffff00u, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	} else {
		/* keep = word & (0x00ffffff >> (24 - 8n)), value <<= 8n */
		sh4_emit_shld(&e->cg, FGL_R_XFER, s1);
		emit_comp_shift(e);
		sh4_emit_neg(&e->cg, FGL_R_T1, FGL_R_T1);
		sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
		emit_const(e, 0x00ffffffu, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	}

	sh4_emit_mov_l_load(&e->cg, s0, FGL_R_XFER);    /* the word */
	sh4_emit_and(&e->cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_or(&e->cg, s1, FGL_R_XFER);
	sh4_emit_mov_l_store(&e->cg, FGL_R_XFER, s0);
}

/* ---------------------------------------------------------------- */
/* The block                                                         */
/* ---------------------------------------------------------------- */

static void emit_fixup(fgl_emitter *e, const ir_fixup *f)
{
	if (f->store)
		st_guest(e, f->guest, f->host);
	else
		ld_guest(e, f->guest, f->host);
}

static void emit_node(fgl_emitter *e, const ir_node *p)
{
	int rd, rs, rt;

	switch (p->op) {
	case IR_MOVE:
		rs = operand(e, p->hs, p->rs, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
		if (rd != rs)
			sh4_emit_mov_reg(&e->cg, rs, rd);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SET:
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		emit_const(e, p->imm, rd);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_ALU:
		rs = operand(e, p->hs, p->rs, FGL_R_T1);
		/* The second operand cannot also land in r1: `alu_rr` uses it
		 * as the awkward-case working register. A node the allocator
		 * left with two operands in memory is given scratch for it. */
		rt = operand(e, p->ht, p->rt, p->sc[0] ? p->sc[0] : FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : (rs == FGL_R_T1 ? FGL_R_XFER : FGL_R_T1);
		alu_rr(e, p->sub, rd, rs, rt);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_ALU_IMM:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
		alu_ri(e, p->sub, rd, rs, p->imm);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SHIFT_IMM:
		rt = operand(e, p->ht, p->rt, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		if (rd != rt)
			sh4_emit_mov_reg(&e->cg, rt, rd);
		shift_const(e, rd, p->imm & 31u, p->sub);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SHIFT_REG:
		/* THE AMOUNT IS TAKEN BEFORE THE VALUE IS MOVED. `sllv rd,rt,rs`
		 * is allowed to name the same guest register for rd and rs, and
		 * moving the value into the destination first would destroy the
		 * count before it was read. Reading the count into r0 up front
		 * makes the aliasing case ordinary.
		 *
		 * The value is fetched first only in the sense of choosing its
		 * register: `operand` may go through r0 to load from the state
		 * block, so the count cannot already be sitting there. */
		rt = operand(e, p->ht, p->rt, FGL_R_T1);

		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		sh4_emit_and_imm(&e->cg, 31);           /* r0 &= 31 */
		/* shad/shld take a signed count, so a right shift is a
		 * negative left one. */
		if (p->sub != SH_LL)
			sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		if (rd != rt)
			sh4_emit_mov_reg(&e->cg, rt, rd);

		if (p->sub == SH_RA)
			sh4_emit_shad(&e->cg, FGL_R_XFER, rd);
		else
			sh4_emit_shld(&e->cg, FGL_R_XFER, rd);

		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_LOAD:
		/* The address goes in r0 and r1 is free for a wide
		 * displacement; the value lands wherever the allocator put it,
		 * or in r1 on its way to the state block.
		 *
		 * A DEFERRED load reads memory here and parks the value in the
		 * state block's temp word, writing no guest register -- the
		 * IR_TEMP_GET after the shadow instruction does that. See
		 * `ir.h`. */
		emit_addr(e, p, FGL_R_XFER, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		switch (p->sub) {
		case MEM_B:
			sh4_emit_mov_b_load(&e->cg, FGL_R_XFER, rd);
			break;
		case MEM_BU:
			sh4_emit_mov_b_load(&e->cg, FGL_R_XFER, rd);
			sh4_emit_extu_b(&e->cg, rd, rd);
			break;
		case MEM_H:
			sh4_emit_mov_w_load(&e->cg, FGL_R_XFER, rd);
			break;
		case MEM_HU:
			sh4_emit_mov_w_load(&e->cg, FGL_R_XFER, rd);
			sh4_emit_extu_w(&e->cg, rd, rd);
			break;
		case MEM_W:
			sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, rd);
			break;
		default:
			e->unsupported = 1;
			e->unsupported_op = p->op;
			return;
		}
		if (p->defer) {
			if (rd != FGL_R_XFER)
				sh4_emit_mov_reg(&e->cg, rd, FGL_R_XFER);
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		} else if (p->hd < 0) {
			st_guest(e, p->rd, rd);
		}
		break;

	case IR_TEMP_GET:
		sh4_emit_mov_l_load_gbr(&e->cg, FGL_AT_TEMP_REG);
		if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(p->rd));
		break;

	case IR_STORE:
		/* The other way round: the address is built in r1 first, so
		 * that r0 is still free to carry a value out of the state
		 * block. Doing it in the other order costs a scratch register
		 * on every store whose value is not in a register. */
		emit_addr(e, p, FGL_R_T1, FGL_R_XFER);
		rt = operand(e, p->ht, p->rt, FGL_R_XFER);
		switch (p->sub) {
		case MEM_B:
		case MEM_BU:
			sh4_emit_mov_b_store(&e->cg, rt, FGL_R_T1);
			break;
		case MEM_H:
		case MEM_HU:
			sh4_emit_mov_w_store(&e->cg, rt, FGL_R_T1);
			break;
		case MEM_W:
			sh4_emit_mov_l_store(&e->cg, rt, FGL_R_T1);
			break;
		default:
			e->unsupported = 1;
			e->unsupported_op = p->op;
			return;
		}
		break;

	case IR_LOAD_UN:
		emit_load_un(e, p);
		break;

	case IR_STORE_UN:
		emit_store_un(e, p);
		break;

	case IR_JUMP:
		emit_const(e, p->imm, FGL_R_EXIT);
		break;

	case IR_CAPTURE:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_EXIT)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_EXIT);
		break;

	case IR_COND: {
		int taken;

		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		rt = (p->sub == CC_EQ || p->sub == CC_NE)
			   ? operand(e, p->ht, p->rt, p->sc[0] ? p->sc[0] : FGL_R_T1)
			   : 0;

		switch (p->sub) {
		case CC_EQ:  sh4_emit_cmpeq(&e->cg, rt, rs); break;
		case CC_NE:  sh4_emit_cmpeq(&e->cg, rt, rs); break;
		case CC_LEZ: sh4_emit_cmppl(&e->cg, rs); break;  /* T = rs > 0 */
		case CC_GTZ: sh4_emit_cmppl(&e->cg, rs); break;
		case CC_LTZ: sh4_emit_cmppz(&e->cg, rs); break;  /* T = rs >= 0 */
		case CC_GEZ: sh4_emit_cmppz(&e->cg, rs); break;
		default:     e->unsupported = 1; e->unsupported_op = p->op; return;
		}

		/* T now says "the easy sense"; whether that is taken or not
		 * depends on the condition, so the branch is chosen rather
		 * than the comparison inverted. */
		taken = (p->sub == CC_EQ || p->sub == CC_GTZ || p->sub == CC_GEZ)
			      ? bf_fwd(e)       /* T set = taken: skip on clear */
			      : bt_fwd(e);      /* T set = not taken */

		emit_const(e, p->imm, FGL_R_EXIT);              /* taken */
		{
			int over = bra_fwd(e);
			patch_fwd8(e, taken);
			emit_const(e, p->imm2, FGL_R_EXIT);     /* fallthrough */
			patch_fwd12(e, over);
		}
		break;
	}

	default:
		e->unsupported = 1;
		e->unsupported_op = p->op;
		break;
	}
}

uint32_t fgl_emit(fgl_emitter *e, const ir_node *ir, int n, const ir_alloc *a)
{
	uint32_t entry;
	int i, f = 0;

	entry = e->base + fgl_size(e);

	for (i = 0; i < a->n_preload; i++)
		emit_fixup(e, &a->preload[i]);

	for (i = 0; i < n; i++) {
		while (f < a->n_fix && a->fix[f].at == i)
			emit_fixup(e, &a->fix[f++]);
		maybe_flush_pool(e);
		emit_node(e, &ir[i]);
	}

	while (f < a->n_fix)
		emit_fixup(e, &a->fix[f++]);

	/* The block publishes where it goes and returns. */
	sh4_emit_mov_reg(&e->cg, FGL_R_EXIT, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_NEXT_PC);
	sh4_emit_rts(&e->cg);
	sh4_emit_nop(&e->cg);

	emit_pool(e);

	if (e->cg.overflow)
		e->overflow = 1;

	return (e->overflow || e->unsupported) ? 0 : entry;
}

uint32_t fgl_emit_block(fgl_emitter *e, const uint32_t *words, uint32_t pc)
{
	ir_node ir[IR_MAX_NODES];
	ir_alloc a;
	int n = ir_decode(words, pc, ir, IR_MAX_NODES);

	if (n <= 0)
		return 0;
	ir_allocate(ir, n, &a);
	return fgl_emit(e, ir, n, &a);
}
