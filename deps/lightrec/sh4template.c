/* SPDX-License-Identifier: LGPL-2.1-or-later */

#include "sh4template.h"

#include <string.h>

/*
 * Keep bloom's measured hot-register order, but move it off lightning's
 * JIT_V0..JIT_V5 (r8..r13): r13 belongs to the address mask in the target
 * ABI.  r7..r12 are the initial six-pin suffix of the r1..r12 pool; future
 * pin-density sweeps can grow downward without moving an existing pin.
 */
static const uint8_t pinned_guest[LIGHTREC_SH4T_NR_PINS] = {
	2, 3, 4, 5, 1, 6, /* v0, v1, a0, a1, at, a2 */
};

#define LIGHTREC_SH4T_MAX_FIRST_BUCKET_OPS 56

static unsigned int gpr_disp(struct lightrec_sh4t *t,
			     unsigned int guest_reg)
{
	unsigned int offset = t->gpr_offset + guest_reg * sizeof(uint32_t);

	if ((offset & 3) || offset > 1020) {
		t->as.failed = true;
		return 0;
	}

	return offset >> 2;
}

void lightrec_sh4t_init(struct lightrec_sh4t *t, void *buffer, size_t size,
			uintptr_t base, unsigned int gpr_offset)
{
	memset(t, 0, sizeof(*t));
	sh4asm_init(&t->as, buffer, size, base);
	t->gpr_offset = gpr_offset;
	if ((gpr_offset & 3) || gpr_offset + 31 * sizeof(uint32_t) > 1020)
		t->as.failed = true;
}

int lightrec_sh4t_guest_host(unsigned int guest_reg)
{
	unsigned int i;

	for (i = 0; i < LIGHTREC_SH4T_NR_PINS; i++)
		if (pinned_guest[i] == guest_reg)
			return SH4ASM_R7 + (int)i;

	return -1;
}

static void emit_load_guest(struct lightrec_sh4t *t, unsigned int guest_reg,
			    unsigned int dest)
{
	int host = lightrec_sh4t_guest_host(guest_reg);

	if (!guest_reg) {
		sh4asm_emit_mov_imm(&t->as, 0, dest);
	} else if (host >= 0) {
		if ((unsigned int)host != dest)
			sh4asm_emit_mov(&t->as, (unsigned int)host, dest);
	} else {
		sh4asm_emit_ld_l_gbr(&t->as, gpr_disp(t, guest_reg));
		if (dest != LIGHTREC_SH4T_XFER)
			sh4asm_emit_mov(&t->as, LIGHTREC_SH4T_XFER, dest);
	}
}

static void emit_store_guest(struct lightrec_sh4t *t, unsigned int guest_reg,
			     unsigned int source)
{
	int host;

	if (!guest_reg)
		return;

	host = lightrec_sh4t_guest_host(guest_reg);
	if (host >= 0) {
		if ((unsigned int)host != source)
			sh4asm_emit_mov(&t->as, source, (unsigned int)host);
	} else {
		if (source != LIGHTREC_SH4T_XFER)
			sh4asm_emit_mov(&t->as, source, LIGHTREC_SH4T_XFER);
		sh4asm_emit_st_l_gbr(&t->as, gpr_disp(t, guest_reg));
	}
}

void lightrec_sh4t_emit_pin_loads(struct lightrec_sh4t *t)
{
	unsigned int i;

	for (i = 0; i < LIGHTREC_SH4T_NR_PINS; i++) {
		sh4asm_emit_ld_l_gbr(&t->as, gpr_disp(t, pinned_guest[i]));
		sh4asm_emit_mov(&t->as, LIGHTREC_SH4T_XFER,
				 SH4ASM_R7 + i);
	}
}

void lightrec_sh4t_emit_pin_stores(struct lightrec_sh4t *t)
{
	unsigned int i;

	for (i = 0; i < LIGHTREC_SH4T_NR_PINS; i++) {
		sh4asm_emit_mov(&t->as, SH4ASM_R7 + i,
				 LIGHTREC_SH4T_XFER);
		sh4asm_emit_st_l_gbr(&t->as, gpr_disp(t, pinned_guest[i]));
	}
}

static unsigned int output_reg(unsigned int guest_reg)
{
	int host = lightrec_sh4t_guest_host(guest_reg);

	return host >= 0 ? (unsigned int)host : LIGHTREC_SH4T_SCRATCH;
}

static void emit_load_constant(struct lightrec_sh4t *t, unsigned int rt,
			       uint32_t value)
{
	unsigned int dest = output_reg(rt);

	sh4asm_emit_load_imm32(&t->as, dest, value);
	emit_store_guest(t, rt, dest);
}

static void emit_addiu(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int dest = output_reg(op->i.rt);
	int32_t imm = (int16_t)op->i.imm;

	if (!op->i.rs || (op->flags & LIGHTREC_MOVI)) {
		uint32_t value = (uint32_t)imm;

		if (op->flags & LIGHTREC_MOVI)
			value += (uint32_t)t->movi_high[op->i.rt] << 16;
		emit_load_constant(t, op->i.rt, value);
		return;
	}

	emit_load_guest(t, op->i.rs, dest);
	if (imm >= -128 && imm <= 127) {
		if (imm)
			sh4asm_emit_add_imm(&t->as, imm, dest);
	} else {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER,
				       (uint32_t)imm);
		sh4asm_emit_add(&t->as, LIGHTREC_SH4T_XFER, dest);
	}
	emit_store_guest(t, op->i.rt, dest);
}

static void emit_logic_imm(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int dest = output_reg(op->i.rt);
	int source_host = lightrec_sh4t_guest_host(op->i.rs);

	if (op->i.op == OP_ANDI && op->i.imm == 0xff) {
		if (source_host >= 0)
			sh4asm_emit_extu_b(&t->as, (unsigned int)source_host,
					    dest);
		else {
			emit_load_guest(t, op->i.rs, dest);
			sh4asm_emit_extu_b(&t->as, dest, dest);
		}
	} else if (op->i.op == OP_ANDI && op->i.imm == 0xffff) {
		if (source_host >= 0)
			sh4asm_emit_extu_w(&t->as, (unsigned int)source_host,
					    dest);
		else {
			emit_load_guest(t, op->i.rs, dest);
			sh4asm_emit_extu_w(&t->as, dest, dest);
		}
	} else {
		emit_load_guest(t, op->i.rs, dest);
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, op->i.imm);
		switch (op->i.op) {
		case OP_ANDI:
			sh4asm_emit_and(&t->as, LIGHTREC_SH4T_XFER, dest);
			break;
		case OP_ORI:
			sh4asm_emit_or(&t->as, LIGHTREC_SH4T_XFER, dest);
			break;
		case OP_XORI:
			sh4asm_emit_xor(&t->as, LIGHTREC_SH4T_XFER, dest);
			break;
		}
	}

	emit_store_guest(t, op->i.rt, dest);
}

static void emit_slti(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int source;
	unsigned int dest = output_reg(op->i.rt);
	int source_host = lightrec_sh4t_guest_host(op->i.rs);

	/* Preserve a memory-backed source while r0 receives the constant. */
	if (source_host >= 0)
		source = (unsigned int)source_host;
	else {
		source = LIGHTREC_SH4T_SCRATCH;
		emit_load_guest(t, op->i.rs, source);
	}

	sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER,
			       (uint32_t)(int32_t)(int16_t)op->i.imm);
	if (op->i.op == OP_SLTI)
		/* cmp/gt Rm,Rn sets T when signed Rn > Rm. */
		sh4asm_emit_cmpgt(&t->as, source, LIGHTREC_SH4T_XFER);
	else
		sh4asm_emit_cmphi(&t->as, source, LIGHTREC_SH4T_XFER);
	sh4asm_emit_movt(&t->as, dest);
	emit_store_guest(t, op->i.rt, dest);
}

bool lightrec_sh4t_emit_alu_imm(struct lightrec_sh4t *t,
				const struct opcode *op)
{
	if (op->i.op < OP_ADDI || op->i.op > OP_LUI)
		return false;

	/* Writes to the architectural zero register have no side effects. */
	if (!op->i.rt)
		return true;

	switch (op->i.op) {
	case OP_ADDI:
	case OP_ADDIU:
		emit_addiu(t, op);
		break;
	case OP_SLTI:
	case OP_SLTIU:
		emit_slti(t, op);
		break;
	case OP_ANDI:
	case OP_XORI:
		emit_logic_imm(t, op);
		break;
	case OP_ORI:
		if (op->flags & LIGHTREC_MOVI) {
			emit_load_constant(t, op->i.rt,
				((uint32_t)t->movi_high[op->i.rt] << 16) |
				op->i.imm);
		} else {
			emit_logic_imm(t, op);
		}
		break;
	case OP_LUI:
		if (op->flags & LIGHTREC_MOVI)
			t->movi_high[op->i.rt] = op->i.imm;
		else
			emit_load_constant(t, op->i.rt,
					   (uint32_t)op->i.imm << 16);
		break;
	}

	return true;
}

static unsigned int source_reg(struct lightrec_sh4t *t, unsigned int guest,
			       unsigned int scratch)
{
	int host = lightrec_sh4t_guest_host(guest);

	if (host >= 0)
		return (unsigned int)host;

	emit_load_guest(t, guest, scratch);
	return scratch;
}

static void emit_commutative(struct lightrec_sh4t *t,
			     const struct opcode *op)
{
	unsigned int work = output_reg(op->r.rd);
	unsigned int source;
	int dest_host = lightrec_sh4t_guest_host(op->r.rd);
	int rt_host = lightrec_sh4t_guest_host(op->r.rt);

	/* If rd aliases rt, keep rt in place and apply rs.  This is the one
	 * alias that loading rs into the two-operand destination would destroy. */
	if (dest_host >= 0 && dest_host == rt_host && op->r.rs != op->r.rt) {
		source = source_reg(t, op->r.rs, LIGHTREC_SH4T_XFER);
	} else {
		emit_load_guest(t, op->r.rs, work);
		source = source_reg(t, op->r.rt, LIGHTREC_SH4T_XFER);
	}

	switch (op->r.op) {
	case OP_SPECIAL_ADD:
	case OP_SPECIAL_ADDU:
		sh4asm_emit_add(&t->as, source, work);
		break;
	case OP_SPECIAL_AND:
		sh4asm_emit_and(&t->as, source, work);
		break;
	case OP_SPECIAL_OR:
		sh4asm_emit_or(&t->as, source, work);
		break;
	case OP_SPECIAL_XOR:
		sh4asm_emit_xor(&t->as, source, work);
		break;
	case OP_SPECIAL_NOR:
		sh4asm_emit_or(&t->as, source, work);
		sh4asm_emit_not(&t->as, work, work);
		break;
	}

	emit_store_guest(t, op->r.rd, work);
}

static void emit_subtract(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int work = output_reg(op->r.rd);
	unsigned int source;
	int dest_host = lightrec_sh4t_guest_host(op->r.rd);
	int rt_host = lightrec_sh4t_guest_host(op->r.rt);

	/* SUB is not commutative.  Compute through scratch when overwriting rd
	 * would otherwise destroy the right operand. */
	if (dest_host >= 0 && dest_host == rt_host && op->r.rs != op->r.rt)
		work = LIGHTREC_SH4T_SCRATCH;

	emit_load_guest(t, op->r.rs, work);
	source = source_reg(t, op->r.rt, LIGHTREC_SH4T_XFER);
	sh4asm_emit_sub(&t->as, source, work);
	emit_store_guest(t, op->r.rd, work);
}

static void emit_compare(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int dest = output_reg(op->r.rd);
	unsigned int rs, rt;

	rs = source_reg(t, op->r.rs, LIGHTREC_SH4T_SCRATCH);
	rt = source_reg(t, op->r.rt,
			rs == LIGHTREC_SH4T_SCRATCH ?
			LIGHTREC_SH4T_XFER : LIGHTREC_SH4T_SCRATCH);

	/* cmp/gt Rm,Rn and cmp/hi Rm,Rn both test Rn > Rm. */
	if (op->r.op == OP_SPECIAL_SLT)
		sh4asm_emit_cmpgt(&t->as, rs, rt);
	else
		sh4asm_emit_cmphi(&t->as, rs, rt);
	sh4asm_emit_movt(&t->as, dest);
	emit_store_guest(t, op->r.rd, dest);
}

/*
 * Both constant and variable MIPS shifts go through SH-4's shld/shad, whose
 * amount register is signed: a non-negative count shifts left, a negative one
 * shifts right (shld logical, shad arithmetic).  This is uniform and correct;
 * the shll2/8/16 decomposition is a later footprint optimisation.
 */
static bool is_arith_right(unsigned int sh_op)
{
	return sh_op == OP_SPECIAL_SRA || sh_op == OP_SPECIAL_SRAV;
}

static bool is_right_shift(unsigned int sh_op)
{
	return sh_op == OP_SPECIAL_SRL || sh_op == OP_SPECIAL_SRA ||
		sh_op == OP_SPECIAL_SRLV || sh_op == OP_SPECIAL_SRAV;
}

/* Apply the count in `amount` to `dest` (dest <<= or >>= amount). */
static void emit_do_shift(struct lightrec_sh4t *t, unsigned int sh_op,
			  unsigned int amount, unsigned int dest)
{
	if (is_arith_right(sh_op))
		sh4asm_emit_shad(&t->as, amount, dest);
	else
		sh4asm_emit_shld(&t->as, amount, dest);
}

static void emit_shift_const(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int dest = output_reg(op->r.rd);
	unsigned int sa = op->r.imm & 0x1f;

	emit_load_guest(t, op->r.rt, dest);
	if (sa) {
		int amt = is_right_shift(op->r.op) ? -(int)sa : (int)sa;

		sh4asm_emit_mov_imm(&t->as, amt, LIGHTREC_SH4T_XFER);
		emit_do_shift(t, op->r.op, LIGHTREC_SH4T_XFER, dest);
	}
	emit_store_guest(t, op->r.rd, dest);
}

static void emit_shift_var(struct lightrec_sh4t *t, const struct opcode *op)
{
	unsigned int dest = output_reg(op->r.rd);
	int dest_host = lightrec_sh4t_guest_host(op->r.rd);
	int rs_host = lightrec_sh4t_guest_host(op->r.rs);

	/* The MIPS variable shift masks the count to five bits.  and #imm,r0 is
	 * r0-only and mov.l @(disp,gbr) targets r0, so the amount is built in
	 * r0 and, only when loading the value would overwrite the count source
	 * (rd and rs share a pin), stashed in the free scratch first. */
	if (dest_host >= 0 && dest_host == rs_host) {
		emit_load_guest(t, op->r.rs, LIGHTREC_SH4T_XFER);
		sh4asm_emit_and_imm_r0(&t->as, 0x1f);
		if (is_right_shift(op->r.op))
			sh4asm_emit_neg(&t->as, LIGHTREC_SH4T_XFER,
					LIGHTREC_SH4T_XFER);
		sh4asm_emit_mov(&t->as, LIGHTREC_SH4T_XFER,
				LIGHTREC_SH4T_SCRATCH);
		emit_load_guest(t, op->r.rt, dest);
		emit_do_shift(t, op->r.op, LIGHTREC_SH4T_SCRATCH, dest);
	} else {
		emit_load_guest(t, op->r.rt, dest);
		emit_load_guest(t, op->r.rs, LIGHTREC_SH4T_XFER);
		sh4asm_emit_and_imm_r0(&t->as, 0x1f);
		if (is_right_shift(op->r.op))
			sh4asm_emit_neg(&t->as, LIGHTREC_SH4T_XFER,
					LIGHTREC_SH4T_XFER);
		emit_do_shift(t, op->r.op, LIGHTREC_SH4T_XFER, dest);
	}

	emit_store_guest(t, op->r.rd, dest);
}

bool lightrec_sh4t_emit_alu_reg(struct lightrec_sh4t *t,
				const struct opcode *op)
{
	if (op->i.op != OP_SPECIAL)
		return false;

	/* ADD/SUB overflow exceptions are intentionally the same policy as the
	 * existing lightrec backend: compute the wrapping result. */
	if (!op->r.rd)
		return op->r.op == OP_SPECIAL_ADD ||
			op->r.op == OP_SPECIAL_ADDU ||
			op->r.op == OP_SPECIAL_SUB ||
			op->r.op == OP_SPECIAL_SUBU ||
			op->r.op == OP_SPECIAL_AND ||
			op->r.op == OP_SPECIAL_OR ||
			op->r.op == OP_SPECIAL_XOR ||
			op->r.op == OP_SPECIAL_NOR ||
			op->r.op == OP_SPECIAL_SLT ||
			op->r.op == OP_SPECIAL_SLTU ||
			op->r.op == OP_SPECIAL_SLL ||
			op->r.op == OP_SPECIAL_SRL ||
			op->r.op == OP_SPECIAL_SRA ||
			op->r.op == OP_SPECIAL_SLLV ||
			op->r.op == OP_SPECIAL_SRLV ||
			op->r.op == OP_SPECIAL_SRAV;

	switch (op->r.op) {
	case OP_SPECIAL_ADD:
	case OP_SPECIAL_ADDU:
	case OP_SPECIAL_AND:
	case OP_SPECIAL_OR:
	case OP_SPECIAL_XOR:
	case OP_SPECIAL_NOR:
		emit_commutative(t, op);
		break;
	case OP_SPECIAL_SUB:
	case OP_SPECIAL_SUBU:
		emit_subtract(t, op);
		break;
	case OP_SPECIAL_SLT:
	case OP_SPECIAL_SLTU:
		emit_compare(t, op);
		break;
	case OP_SPECIAL_SLL:
	case OP_SPECIAL_SRL:
	case OP_SPECIAL_SRA:
		emit_shift_const(t, op);
		break;
	case OP_SPECIAL_SLLV:
	case OP_SPECIAL_SRLV:
	case OP_SPECIAL_SRAV:
		emit_shift_var(t, op);
		break;
	default:
		return false;
	}

	return true;
}

/*
 * Build a direct-map host address in `dest`: (guest addr) [& mask] + host base.
 * For a MOVI-fused access the base register is a folded lui constant that was
 * never materialised, so the whole guest address is known at compile time and
 * the host address collapses to a single immediate load.
 */
static void emit_mem_addr(struct lightrec_sh4t *t, const struct opcode *op,
			  unsigned int dest, uintptr_t host_offset,
			  bool use_mask, uint32_t mask)
{
	int32_t imm = (int16_t)op->i.imm;

	if (op->flags & LIGHTREC_MOVI) {
		uint32_t addr = ((uint32_t)t->movi_high[op->i.rs] << 16)
				+ (uint32_t)imm;

		if (use_mask)
			addr &= mask;
		addr += (uint32_t)host_offset;
		sh4asm_emit_load_imm32(&t->as, dest, addr);
		return;
	}

	/* guest address = rs + imm, then (optionally) mask to the map. */
	emit_load_guest(t, op->i.rs, dest);
	if (imm >= -128 && imm <= 127) {
		if (imm)
			sh4asm_emit_add_imm(&t->as, imm, dest);
	} else {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, (uint32_t)imm);
		sh4asm_emit_add(&t->as, LIGHTREC_SH4T_XFER, dest);
	}
	if (use_mask) {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, mask);
		sh4asm_emit_and(&t->as, LIGHTREC_SH4T_XFER, dest);
	}

	/* host address = guest address + host base (base is nonzero on real
	 * maps; guard keeps a zero-based test map from emitting a dead add). */
	if (host_offset) {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER,
				       (uint32_t)host_offset);
		sh4asm_emit_add(&t->as, LIGHTREC_SH4T_XFER, dest);
	}
}

/*
 * Fast-path guest load for the RAM/BIOS/scratchpad direct maps.  The caller has
 * already resolved the host base and mask from the opcode's IO mode and proven
 * (via can_emit_block) that there is no load-delay slot.  The full host address
 * is built in the destination register, then the load reads back into it:
 * mov.{b,w,l} @Rn,Rn is legal and frees the scratch for the pin store.
 */
bool lightrec_sh4t_emit_load(struct lightrec_sh4t *t, const struct opcode *op,
			     uintptr_t host_offset, bool use_mask, uint32_t mask)
{
	int32_t imm = (int16_t)op->i.imm;
	unsigned int dest;

	/* A load into the zero register has no architectural effect on the
	 * direct maps (no read side effects), matching rec_load_memory. */
	if (!op->i.rt)
		return true;

	dest = output_reg(op->i.rt);

	emit_mem_addr(t, op, dest, host_offset, use_mask, mask);

	switch (op->i.op) {
	case OP_LB:	/* mov.b sign-extends, matching MIPS LB. */
		sh4asm_emit_ld_b(&t->as, dest, dest);
		break;
	case OP_LBU:
		sh4asm_emit_ld_b(&t->as, dest, dest);
		sh4asm_emit_extu_b(&t->as, dest, dest);
		break;
	case OP_LH:	/* mov.w sign-extends, matching MIPS LH. */
		sh4asm_emit_ld_w(&t->as, dest, dest);
		break;
	case OP_LHU:
		sh4asm_emit_ld_w(&t->as, dest, dest);
		sh4asm_emit_extu_w(&t->as, dest, dest);
		break;
	case OP_LW:
		sh4asm_emit_ld_l(&t->as, dest, dest);
		break;
	default:
		return false;
	}

	emit_store_guest(t, op->i.rt, dest);
	return true;
}

/*
 * Fast-path guest store for the RAM/scratchpad direct maps.  Template stores
 * never touch the code LUT, so the caller only routes a store here when
 * lightrec itself would skip invalidation (scratchpad, INV_DMA_ONLY, or a
 * per-op no-invalidate).  The host address is built in the scratch register and
 * the value comes from its pin or, failing that, r0.
 */
bool lightrec_sh4t_emit_store(struct lightrec_sh4t *t, const struct opcode *op,
			      uintptr_t host_offset, bool use_mask, uint32_t mask)
{
	unsigned int addr = LIGHTREC_SH4T_SCRATCH;
	unsigned int value;
	int rt_host;

	/* host address built in r3 (rs + imm, [& mask], + host base). */
	emit_mem_addr(t, op, addr, host_offset, use_mask, mask);

	/* Source value: a pin is used in place; anything else (including the
	 * zero register) is materialised in r0 after the address is built. */
	rt_host = lightrec_sh4t_guest_host(op->i.rt);
	if (op->i.rt && rt_host >= 0) {
		value = (unsigned int)rt_host;
	} else {
		value = LIGHTREC_SH4T_XFER;
		emit_load_guest(t, op->i.rt, value);
	}

	switch (op->i.op) {
	case OP_SB:	/* mov.b stores the low byte, matching MIPS SB. */
		sh4asm_emit_st_b(&t->as, value, addr);
		break;
	case OP_SH:
		sh4asm_emit_st_w(&t->as, value, addr);
		break;
	case OP_SW:
		sh4asm_emit_st_l(&t->as, value, addr);
		break;
	default:
		return false;
	}

	return true;
}

/*
 * Emit the SH-4 comparison that sets the T bit from a MIPS branch condition and
 * return whether "taken" means T==1 (use bt) or T==0 (use bf).  rs/rt must still
 * hold the branch's inputs; the caller places the bt/bf and its two edges.  This
 * is used for both mid-block (internal-target) and terminating branches.
 */
bool lightrec_sh4t_emit_condition(struct lightrec_sh4t *t,
				  const struct opcode *op)
{
	unsigned int rs, rt;

	if (op->i.op == OP_BEQ || op->i.op == OP_BNE) {
		rs = source_reg(t, op->i.rs, LIGHTREC_SH4T_SCRATCH);
		rt = source_reg(t, op->i.rt,
				rs == LIGHTREC_SH4T_SCRATCH ?
				LIGHTREC_SH4T_XFER : LIGHTREC_SH4T_SCRATCH);
		sh4asm_emit_cmpeq(&t->as, rs, rt);
		/* BEQ taken when equal (T==1); BNE taken when not (T==0). */
		return op->i.op == OP_BEQ;
	}

	rs = source_reg(t, op->i.rs, LIGHTREC_SH4T_SCRATCH);
	if (op->i.op == OP_BGTZ || op->i.op == OP_BLEZ) {
		sh4asm_emit_cmppl(&t->as, rs);		/* T = (rs > 0) */
		return op->i.op == OP_BGTZ;
	}

	sh4asm_emit_cmppz(&t->as, rs);			/* T = (rs >= 0) */
	return op->i.rt == OP_REGIMM_BGEZ;		/* OP_REGIMM: BLTZ / BGEZ */
}

static void emit_charge_cycles(struct lightrec_sh4t *t, unsigned int cycles)
{
	if (cycles <= 128) {
		if (cycles)
			sh4asm_emit_add_imm(&t->as, -(int)cycles, SH4ASM_R1);
	} else {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, cycles);
		sh4asm_emit_sub(&t->as, LIGHTREC_SH4T_XFER, SH4ASM_R1);
	}
}

/* JAL / static call: write the return address to the link register, then take
 * the ordinary static exit to the (known) target. link_reg 0 means no link. */
void lightrec_sh4t_emit_call(struct lightrec_sh4t *t, unsigned int link_reg,
			     uint32_t return_pc, unsigned int cycles,
			     uint32_t target_pc, uint32_t target_lut,
			     unsigned int fast_eob_offset)
{
	if (link_reg)
		emit_load_constant(t, link_reg, return_pc);

	lightrec_sh4t_emit_dispatch_exit(t, cycles, target_pc, target_lut,
					 fast_eob_offset);
}

/* JR / JALR: put the guest target (rs) in the PC register, optionally link, and
 * jump to eob_wrapper_func, which resolves the code LUT from the PC at runtime.
 * rs is read before the link is written, so JALR with rd == rs is safe. */
void lightrec_sh4t_emit_jump_reg(struct lightrec_sh4t *t, unsigned int rs_guest,
				 unsigned int link_reg, uint32_t return_pc,
				 unsigned int cycles, unsigned int eob_offset)
{
	if ((eob_offset & 3) || eob_offset > 1020) {
		t->as.failed = true;
		return;
	}

	emit_load_guest(t, rs_guest, SH4ASM_R4);	/* r4 = target PC */
	if (link_reg)
		emit_load_constant(t, link_reg, return_pc);

	emit_charge_cycles(t, cycles);
	sh4asm_emit_ld_l_gbr(&t->as, eob_offset >> 2);	/* r0 = eob_wrapper */
	sh4asm_emit_jmp(&t->as, LIGHTREC_SH4T_XFER);
	sh4asm_emit_nop(&t->as);
}

/*
 * Dumb-but-complete memory access: hand the opcode to the C helper (state in r4,
 * the raw opcode in r5), which reads rs/rt from state and writes the result
 * back.  Pins are synced to memory first and reloaded after, since the helper
 * touches guest state through the state pointer, not our registers.  c_func is
 * the GBR-shimmed entry, which swaps GBR for the C runtime and back.
 */
void lightrec_sh4t_emit_mem_call(struct lightrec_sh4t *t, uint32_t opcode,
				 uintptr_t c_func)
{
	lightrec_sh4t_emit_pin_stores(t);

	/* r1 (cycle counter) is caller-saved; park it on the host stack, which
	 * the callee preserves.  The pins are already in memory. */
	sh4asm_emit_add_imm(&t->as, -8, SH4ASM_R15);
	sh4asm_emit_st_l(&t->as, SH4ASM_R1, SH4ASM_R15);

	sh4asm_emit_stc_gbr(&t->as, SH4ASM_R4);			/* r4 = state */
	sh4asm_emit_load_imm32(&t->as, SH4ASM_R5, opcode);	/* r5 = opcode */
	sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, (uint32_t)c_func);
	sh4asm_emit_jsr(&t->as, LIGHTREC_SH4T_XFER);
	sh4asm_emit_nop(&t->as);

	sh4asm_emit_ld_l(&t->as, SH4ASM_R15, SH4ASM_R1);
	sh4asm_emit_add_imm(&t->as, 8, SH4ASM_R15);

	lightrec_sh4t_emit_pin_loads(t);
}

/* mult/div/HI-LO via the C helper: like emit_mem_call but also passes the op's
 * flags in r6 (the helper needs LIGHTREC_NO_HI/NO_LO). */
void lightrec_sh4t_emit_muldiv_call(struct lightrec_sh4t *t, uint32_t opcode,
				    uint32_t flags, uintptr_t c_func)
{
	lightrec_sh4t_emit_pin_stores(t);

	sh4asm_emit_add_imm(&t->as, -8, SH4ASM_R15);
	sh4asm_emit_st_l(&t->as, SH4ASM_R1, SH4ASM_R15);

	sh4asm_emit_stc_gbr(&t->as, SH4ASM_R4);			/* r4 = state */
	sh4asm_emit_load_imm32(&t->as, SH4ASM_R5, opcode);	/* r5 = opcode */
	sh4asm_emit_load_imm32(&t->as, SH4ASM_R6, flags);	/* r6 = flags */
	sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, (uint32_t)c_func);
	sh4asm_emit_jsr(&t->as, LIGHTREC_SH4T_XFER);
	sh4asm_emit_nop(&t->as);

	sh4asm_emit_ld_l(&t->as, SH4ASM_R15, SH4ASM_R1);
	sh4asm_emit_add_imm(&t->as, 8, SH4ASM_R15);

	lightrec_sh4t_emit_pin_loads(t);
}

bool lightrec_sh4t_emit_meta_alu(struct lightrec_sh4t *t,
				 const struct opcode *op)
{
	unsigned int dest;

	if (op->i.op != OP_META ||
	    (op->m.op != OP_META_MOV && op->m.op != OP_META_COM))
		return false;
	if (!op->m.rd)
		return true;

	dest = output_reg(op->m.rd);
	emit_load_guest(t, op->m.rs, dest);
	if (op->m.op == OP_META_COM)
		sh4asm_emit_not(&t->as, dest, dest);
	emit_store_guest(t, op->m.rd, dest);
	return true;
}

/* Loads the flat emitter handles: the fixed-width integer loads.  RAM/BIOS/
 * scratchpad go inline; every other mode goes through the C helper.  A load with
 * a delay slot to defer still falls back (the C helper's deferral needs the
 * dispatcher's check_load_delay, which the template does not run).  LWL/LWR/
 * LWC2/META are separate ops and still fall back. */
static bool is_supported_load(const struct opcode *op)
{
	switch (op->i.op) {
	case OP_LB:
	case OP_LBU:
	case OP_LH:
	case OP_LHU:
	case OP_LW:
		break;
	default:
		return false;
	}

	/* A MOVI-fused load folds a lui into a constant base address.  The direct
	 * maps materialise that constant inline; the C helper (other modes) reads
	 * the stale base register, so those stay on lightning. */
	if (op->flags & LIGHTREC_MOVI) {
		unsigned int mode = LIGHTREC_FLAGS_GET_IO_MODE(op->flags);

		if (mode != LIGHTREC_IO_RAM && mode != LIGHTREC_IO_BIOS &&
		    mode != LIGHTREC_IO_SCRATCH)
			return false;
	}

	/* An SMC / no-invalidate access takes lightrec's code-coherence path,
	 * which the flat emitter does not reproduce; leave it on lightning. */
	if (op->flags & (LIGHTREC_SMC | LIGHTREC_NO_INVALIDATE))
		return false;

	return !op_flag_load_delay(op->flags);
}

/* A load that goes through the C helper (any non-direct map) writes its result
 * to gpr[rt] immediately, whereas MIPS defers it one instruction.  lightrec
 * relies on a runtime deferral for these, not the LIGHTREC_LOAD_DELAY flag, so
 * such a load is only safe here when the very next op does not read rt. */
static bool is_c_path_load(const struct opcode *op)
{
	unsigned int mode = LIGHTREC_FLAGS_GET_IO_MODE(op->flags);

	switch (op->i.op) {
	case OP_LB:
	case OP_LBU:
	case OP_LH:
	case OP_LHU:
	case OP_LW:
		return mode != LIGHTREC_IO_RAM && mode != LIGHTREC_IO_BIOS &&
			mode != LIGHTREC_IO_SCRATCH;
	default:
		return false;
	}
}

/* Conservative "does op read guest register r": rs and rt are the source fields
 * for the ops the template handles (rt is a source for stores, R-type and
 * branches; for I-type ALU/loads it is a destination, so this may over-reject,
 * which only costs coverage, never correctness). */
static bool op_reads_reg(const struct opcode *op, unsigned int r)
{
	return r != 0 && (op->i.rs == r || op->i.rt == r);
}

/* Stores the flat emitter handles: fixed-width integer stores.  RAM/scratchpad
 * go inline; other modes go through the C helper.  A RAM/direct store is only
 * accepted where lightrec itself skips code-LUT invalidation - under INV_DMA_ONLY
 * or a per-op no-invalidate.  (Scratchpad and true I/O never touch code.) */
static bool is_supported_store(const struct opcode *op, bool inv_dma_only)
{
	unsigned int mode;

	switch (op->i.op) {
	case OP_SB:
	case OP_SH:
	case OP_SW:
		break;
	default:
		return false;
	}

	/* An SMC / no-invalidate store takes lightrec's code-coherence path
	 * (LUT invalidation), which the flat emitter does not reproduce; leave
	 * it on lightning. */
	if (op->flags & (LIGHTREC_SMC | LIGHTREC_NO_INVALIDATE))
		return false;

	mode = LIGHTREC_FLAGS_GET_IO_MODE(op->flags);

	/* A MOVI-fused store folds a lui into a constant base address; only the
	 * inline direct maps (RAM/scratchpad) can use it - the C helper would
	 * read the stale base register. */
	if ((op->flags & LIGHTREC_MOVI) &&
	    mode != LIGHTREC_IO_RAM && mode != LIGHTREC_IO_SCRATCH)
		return false;

	if (mode == LIGHTREC_IO_RAM || mode == LIGHTREC_IO_DIRECT)
		return inv_dma_only;

	return true;
}

static bool is_supported_body(const struct opcode *op, bool inv_dma_only)
{
	if (!op->opcode ||
	    (op->i.op >= OP_ADDI && op->i.op <= OP_LUI))
		return true;

	if (is_supported_load(op) || is_supported_store(op, inv_dma_only))
		return true;

	if (op->i.op == OP_SPECIAL) {
		switch (op->r.op) {
		case OP_SPECIAL_ADD:
		case OP_SPECIAL_ADDU:
		case OP_SPECIAL_SUB:
		case OP_SPECIAL_SUBU:
		case OP_SPECIAL_AND:
		case OP_SPECIAL_OR:
		case OP_SPECIAL_XOR:
		case OP_SPECIAL_NOR:
		case OP_SPECIAL_SLT:
		case OP_SPECIAL_SLTU:
		case OP_SPECIAL_SLL:
		case OP_SPECIAL_SRL:
		case OP_SPECIAL_SRA:
		case OP_SPECIAL_SLLV:
		case OP_SPECIAL_SRLV:
		case OP_SPECIAL_SRAV:
		/* mult/div and HI/LO moves go through the C helper. */
		case OP_SPECIAL_MULT:
		case OP_SPECIAL_MULTU:
		case OP_SPECIAL_DIV:
		case OP_SPECIAL_DIVU:
		case OP_SPECIAL_MFHI:
		case OP_SPECIAL_MFLO:
		case OP_SPECIAL_MTHI:
		case OP_SPECIAL_MTLO:
			return true;
		default:
			return false;
		}
	}

	return op->i.op == OP_META &&
		(op->m.op == OP_META_MOV || op->m.op == OP_META_COM);
}

bool lightrec_sh4t_is_uncond(const struct opcode *op)
{
	return op->i.op == OP_J || op->i.op == OP_JAL ||
		(op->i.op == OP_BEQ && op->i.rs == op->i.rt);
}

/* Register-indirect jumps: JR (return/dispatch) and JALR (call via register). */
bool lightrec_sh4t_is_jump_reg(const struct opcode *op)
{
	return op->i.op == OP_SPECIAL &&
		(op->r.op == OP_SPECIAL_JR || op->r.op == OP_SPECIAL_JALR);
}

/* Conditional branches with two external targets that the branch bucket emits.
 * BEQ with rs==rt is the unconditional (static) form, handled above; the AL
 * link variants and JAL are excluded (they write $ra). */
bool lightrec_sh4t_is_cond_branch(const struct opcode *op)
{
	switch (op->i.op) {
	case OP_BNE:
	case OP_BLEZ:
	case OP_BGTZ:
		return true;
	case OP_BEQ:
		return op->i.rs != op->i.rt;
	case OP_REGIMM:
		return op->i.rt == OP_REGIMM_BLTZ ||
			op->i.rt == OP_REGIMM_BGEZ;
	default:
		return false;
	}
}

/* Guest register a body op writes, or 0 (the zero register, i.e. no effect).
 * Used to prove a delay slot does not clobber its branch's compared inputs. */
static unsigned int body_output_reg(const struct opcode *op)
{
	if (op->i.op == OP_SPECIAL)
		return op->r.rd;
	if (op->i.op == OP_META)
		return op->m.rd;
	switch (op->i.op) {
	case OP_SB:
	case OP_SH:
	case OP_SW:
		return 0;	/* stores write no register */
	default:
		return op->i.rt;
	}
}

uint32_t lightrec_sh4t_branch_target(const struct opcode *op, uint32_t pc,
				     unsigned int i, bool no_ds)
{
	if (op->i.op == OP_J || op->i.op == OP_JAL)
		return (pc & 0xf0000000u) | (op->j.imm << 2);

	return pc + (uint32_t)(((int32_t)i - (int32_t)no_ds + 1 +
			       (int16_t)op->i.imm) * 4);
}

/* Delay-slot safety: a slot emitted before its branch's condition must be a
 * supported body op that does not clobber the registers the branch compares. */
static bool ds_ok_for_branch(const struct opcode *ds, const struct opcode *br,
			     bool inv_dma_only)
{
	unsigned int out;
	bool cmp_rt;

	if (!ds->opcode)
		return true;
	if (!is_supported_body(ds, inv_dma_only))
		return false;

	out = body_output_reg(ds);
	cmp_rt = br->i.op == OP_BEQ || br->i.op == OP_BNE;
	return !out || (out != br->i.rs && !(cmp_rt && out == br->i.rt));
}

/*
 * Accept a block of implemented ops with arbitrary FORWARD control flow: any
 * number of conditional branches (mid-block or terminating) whose taken target
 * is either forward within the block or external, plus optional unconditional
 * terminators to external targets, and a fall-through off the end.  The template
 * keeps all guest state canonical after every op (pins or memory), so branch
 * joins need no reconciliation.  Backward branches (loops), internal
 * unconditional jumps, and local-flagged branches still fall back to lightning.
 */
bool lightrec_sh4t_can_emit_block(const struct opcode *ops,
				 unsigned int nr_ops, uint32_t pc,
				 bool inv_dma_only)
{
	uint32_t block_end = pc + nr_ops * sizeof(uint32_t);
	bool consumed[LIGHTREC_SH4T_MAX_FIRST_BUCKET_OPS] = { false };
	unsigned int i;

	/* The cap bounds both the literal table and every PC-relative load's
	 * reach.  It can be raised once mid-block literal islands exist. */
	if (!ops || !nr_ops || nr_ops > LIGHTREC_SH4T_MAX_FIRST_BUCKET_OPS)
		return false;

	for (i = 0; i < nr_ops; i++) {
		const struct opcode *op = &ops[i];
		bool no_ds = op_flag_no_ds(op->flags);
		bool conditional, uncond, jr;
		uint32_t target;

		if (consumed[i])
			continue;
		/* SYNC (BIT1) applies to any op and does not collide with the
		 * mult/div flag bits, so keep rejecting it up front. */
		if (op_flag_sync(op->flags))
			return false;
		if (!op->opcode)
			continue;
		if (is_supported_body(op, inv_dma_only)) {
			/* C-path load with a load-delay hazard on the next op. */
			if (is_c_path_load(op) && i + 1 < nr_ops &&
			    op_reads_reg(&ops[i + 1], op->i.rt))
				return false;
			continue;
		}

		/* Only branches reach here; the EMULATE_BRANCH/LOCAL_BRANCH bits
		 * (BIT3/BIT4) alias NO_LO/NO_HI on mult/div, which are body ops
		 * already accepted above, so test them only for real branches. */
		if (op_flag_emulate_branch(op->flags) ||
		    op_flag_local_branch(op->flags))
			return false;

		conditional = lightrec_sh4t_is_cond_branch(op);
		uncond = lightrec_sh4t_is_uncond(op);
		jr = lightrec_sh4t_is_jump_reg(op);
		if (!conditional && !uncond && !jr)
			return false;

		/* Consume and validate a real (non-hoisted) delay slot: it must be
		 * a supported body op that does not clobber the register the branch
		 * compares (conditional) or jumps through (jr). */
		if (!no_ds) {
			const struct opcode *ds = &ops[i + 1];

			if (i + 1 >= nr_ops)
				return false;
			if (ds->opcode && !is_supported_body(ds, inv_dma_only))
				return false;
			if (conditional && !ds_ok_for_branch(ds, op, inv_dma_only))
				return false;
			if (jr && ds->opcode && body_output_reg(ds) &&
			    body_output_reg(ds) == op->r.rs)
				return false;
			consumed[i + 1] = true;
		}

		/* Register-indirect jumps resolve their target at runtime. */
		if (jr)
			continue;

		target = lightrec_sh4t_branch_target(op, pc, i, no_ds);

		/* Internal targets must be forward and op-aligned; backward
		 * targets (loops) and internal unconditional jumps fall back. */
		if (target >= pc && target < block_end) {
			unsigned int tgt_op = (target - pc) / sizeof(uint32_t);

			if ((target & 3) || tgt_op <= i || uncond)
				return false;
		}
	}

	return true;
}

void lightrec_sh4t_emit_dispatch_exit(struct lightrec_sh4t *t,
				      unsigned int cycles, uint32_t next_pc,
				      uint32_t lut_entry,
				      unsigned int fast_eob_offset)
{
	if ((fast_eob_offset & 3) || fast_eob_offset > 1020) {
		t->as.failed = true;
		return;
	}

	if (cycles <= 128) {
		if (cycles)
			sh4asm_emit_add_imm(&t->as, -(int)cycles, SH4ASM_R1);
	} else {
		sh4asm_emit_load_imm32(&t->as, LIGHTREC_SH4T_XFER, cycles);
		sh4asm_emit_sub(&t->as, LIGHTREC_SH4T_XFER, SH4ASM_R1);
	}

	sh4asm_emit_load_imm32(&t->as, SH4ASM_R4, next_pc);
	sh4asm_emit_load_imm32(&t->as, SH4ASM_R5, lut_entry);
	sh4asm_emit_ld_l_gbr(&t->as, fast_eob_offset >> 2);
	sh4asm_emit_jmp(&t->as, LIGHTREC_SH4T_XFER);
	sh4asm_emit_nop(&t->as);
}
