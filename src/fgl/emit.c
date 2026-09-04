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

void fgl_set_targets(fgl_emitter *e, const fgl_targets *t)
{
	e->tgt = t;
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
		e->overflow = 3;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xff));
}

static void patch_fwd12(fgl_emitter *e, int site)
{
	int disp = here(e) - site - 2;

	if (!sh4_disp12_fits(disp))
		e->overflow = 4;
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
		e->overflow = 5;
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
			e->overflow = 2;        /* pool out of reach */
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

	/* TWO TRIGGERS, AND THE SECOND IS NOT A SAFETY MARGIN.
	 *
	 * Distance first: 700 rather than 1020, because the pool itself, and
	 * whatever the next node emits before the flush actually happens, both
	 * sit inside the gap.
	 *
	 * But reach is not the only thing that runs out.  `fix[]` holds one
	 * entry per SITE, not per distinct value, and a service call costs
	 * three sites in five instructions -- so a run of I/O or GTE nodes
	 * fills the table long before it has travelled 700 bytes.  That is
	 * what refused the BIOS block at 0xbfc05460: 36 nodes, no reach
	 * problem, and `emit_const` out of fixup slots.  A full table is a
	 * reason to flush, exactly like a full reach; it was only ever an
	 * overflow because nothing here looked at it.
	 *
	 * The headroom is for the widest single node, since a flush can only
	 * happen BETWEEN nodes -- an unaligned store setting up a shim call is
	 * the worst of them and is nowhere near sixteen. */
	if (2 * (here(e) - e->fix[0].at) < 700 &&
	    e->n_fix < FGL_MAX_LITERALS - 16)
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
static void emit_addr_ex(fgl_emitter *e, const ir_node *p, int dst, int other,
			 int mask)
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

	/* THE MASK, AND THE ONE PROOF THAT LETS IT GO.
	 *
	 * Every guest address is masked into the window, and it costs one
	 * instruction on every load and every store -- which, at six and
	 * five-and-a-half SH-4 instructions per guest access, is real money on
	 * the two hottest classes there are.
	 *
	 * It comes off only where the optimiser established that the WHOLE
	 * possible range of this address already lands inside one region:
	 * `lightrec_get_constprop_map` requires the minimum and maximum
	 * computed address to agree in their top three bits before it will
	 * name a region at all (constprop.c:745), and NO_MASK is set on top of
	 * that. The raw-word front end never sets it, so the oracle's default
	 * path still masks and this is a strictly-added fast path rather than
	 * a changed one.
	 *
	 * If that proof is ever wrong the failure is a store through a wild
	 * address, which is the worst kind of bug this project can produce --
	 * so it is driven by the flag alone and never by anything inferred
	 * here. */
	if (mask && !(p->hint & FGL_H_NO_MASK))
		sh4_emit_and(&e->cg, FGL_R_MASK, dst);
}

/* A DIRECT access: the address is dereferenced right here, so it is masked
 * into the window the host map covers. */
static void emit_addr(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	emit_addr_ex(e, p, dst, other, 1);
}

/* A DEVICE access, handed to C -- AND THE MASK MUST NOT BE APPLIED.
 *
 * `lightrec_hw_lb` and its family take the RAW guest address and run their own
 * `kunseg` on it (lightrec.c:1615 onwards), and `kunseg` is not a mask: it
 * tells KSEG1 from everything else by testing `addr >= 0xa0000000`, and that
 * test is exactly what `and 0x1fffffff` destroys.
 *
 * For most addresses masking first is invisible, because the two agree.  It is
 * not invisible for the segment lightrec keys a map on rather than a physical
 * address: the cache control register is reached at 0xfffe0130, `kunseg` makes
 * that 0x5ffe0130, and PSX_MAP_CACHE_CONTROL sits at 0x5ffe0130 (plugin.c:358).
 * Pre-masking gives 0x1ffe0130 instead, no map matches, and the store comes
 * back as a segfault from code that was otherwise perfectly correct.
 *
 * lightrec's own `rec_store_hw_call` does `jit_addi(tmp, rs, imm)` and passes
 * that -- an add and nothing else, which is what this reproduces. */
static void emit_addr_raw(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	emit_addr_ex(e, p, dst, other, 0);
}

/* SELF-MODIFYING CODE: A STORE INTO RAM MUST UNCOMPILE WHAT IT WROTE OVER.
 *
 * The guest writes an instruction, jumps to it, and expects the NEW one to
 * run.  Nothing about the store tells the dispatcher that the block it
 * compiled from those bytes is now a lie, so the store has to say so: it
 * writes NULL into the block table entry covering the word, and the next
 * dispatch through that PC misses, calls C, and compiles the new code.
 *
 * lightrec emitted exactly this inline on every invalidating store
 * (emitter.c:1600), and fgl not emitting it is what failed S7-SMC on the CPU
 * test -- the guest patched a function, called it, and got the code from
 * before the patch.  There is no cheaper place to put it: an emitted store is
 * a `mov.l` with no call and no exit, so C never sees it happen.
 *
 * THE TABLE INDEX IS THE ADDRESS. One entry per aligned guest word, four
 * bytes each, so the byte offset is `addr & 0x1ffffc` -- the mirrors fold
 * into it for free, exactly as `lut_offset` does for the dispatcher, and
 * masking the low bits is right whether or not the address was masked
 * already (FGL_H_NO_MASK leaves a KSEG address in the register and the bits
 * that survive are the same ones).
 *
 * THE RANGE TEST, and why only the unproven class pays for it. FGL_IO_RAM is
 * a proof that the address lands in RAM, so the entry is always the right one
 * to clear. FGL_IO_DIRECT is only a proof that it lands in SOME directly
 * backed region, which may be the scratchpad or the BIOS -- and clearing a
 * table entry for one of those addresses would uncompile an unrelated block.
 * On bloom's map every non-RAM region sits above 0x1f000000 once masked, and
 * RAM with its mirrors is the low 8 MB, so `(addr & mask) >> 24 == 0` decides
 * it in three instructions and no literal.
 *
 * REGISTERS. This runs AFTER the value has been stored, so r0 and the address
 * register are both dead and may be used freely; the address register is left
 * holding zero, which is what gets written to the table. */
static void emit_invalidate(fgl_emitter *e, const ir_node *p, int addr, int tmp)
{
	int skip = -1;

	if (!ir_store_invalidates(p))
		return;

	if (tmp < 0) {
		e->overflow = 9;        /* the allocator and ir.h disagree */
		return;
	}

	if (p->io == FGL_IO_DIRECT) {
		sh4_emit_mov_reg(&e->cg, addr, tmp);
		sh4_emit_and(&e->cg, FGL_R_MASK, tmp);
		sh4_emit_shlr16(&e->cg, tmp);
		sh4_emit_shlr8(&e->cg, tmp);
		sh4_emit_tst(&e->cg, tmp, tmp);   /* T = (it is RAM) */
		skip = bf_fwd(e);
	}

	emit_const(e, 0x001ffffcu, FGL_R_XFER);
	sh4_emit_and(&e->cg, addr, FGL_R_XFER);         /* r0 = byte offset */
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, tmp);
	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_LUT);
	sh4_emit_mov_imm(&e->cg, 0, addr);              /* the NULL to write */
	sh4_emit_mov_l_store_r0(&e->cg, addr, tmp);     /* mov.l Rn,@(r0,tmp) */

	if (skip >= 0)
		patch_fwd8(e, skip);
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
/* COP2 register traffic                                             */
/* ---------------------------------------------------------------- */

/* MFC2/CFC2/MTC2/CTC2/LWC2/SWC2 -- the moves only.  The geometry
 * commands themselves are IR_GTE and are not lowered here.
 *
 * `imm` IS ALREADY THE STATE-BLOCK WORD INDEX of the coprocessor
 * register, resolved by the decoder (`cop2_disp`, decode.c:59).  Data
 * and control are one contiguous 64-entry file at cp2d = word 66 and
 * cp2c = word 98 (fgl_state.h), so the highest index any of these can
 * name is 129 -- byte 516, inside `mov.l @(disp,GBR)`'s 1020 -- and no
 * address is ever computed for a coprocessor register.  Nothing below
 * needs to know which half of the file it is touching.
 *
 * LWC2 AND SWC2 KEEP THE MEMORY DISPLACEMENT IN `imm2`, NOT `imm`
 * (decode.c:419, 428), which is the reverse of every other memory
 * node.  `emit_addr` reads `imm`, so it is handed a node with the
 * fields the way round it expects; getting this wrong would address
 * off a coprocessor register number and the fault would look like a
 * bad base register rather than a swapped field. */
static void emit_cop2_addr(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	ir_node q = *p;

	q.imm = p->imm2;
	emit_addr(e, &q, dst, other);
}

/* ---------------------------------------------------------------- */
/* Exceptions                                                        */
/* ---------------------------------------------------------------- */

#define CP0_AT(r) ((int)(FGL_AT_COP0 + (unsigned)(r)))

/* THE GUEST'S GENERAL EXCEPTION VECTOR, BEV=0.
 *
 * `tools/docs/cpuspecifications.md`, "Exception Vectors": General is
 * 80000080h with BEV clear and BFC00180h with it set, and the same section
 * states the PSX uses only the BEV=0 vectors -- the BIOS ROM does not contain
 * the BEV=1 ones at all.  So the constant is emitted unconditionally and the
 * SR bit 22 test is not.  THAT IS THE CONDITION on this sequence: a guest that
 * sets BEV and then traps goes to the wrong place.
 *
 * The 80000040h "COP0 Break" vector in that same table is the COP0 debug
 * breakpoint unit (BPC/BDA), not the BREAK opcode; BREAK raises excode 09h
 * through the general vector like any other exception.  `psxException`
 * (deps/pcsx_rearmed/libpcsxcore/r3000a.c:113) is the runtime we have to agree
 * with and it makes no distinction either. */
#define GUEST_EXC_VECTOR     0x80000080u
#define GUEST_EXC_VECTOR_BEV 0xbfc00180u

/* Replace a field of a COP0 word with three instructions and no third
 * register.
 *
 * `and #imm` and `or #imm` exist only for r0, so the obvious "load, mask with
 * a materialised constant, or in the new bits" wants a register we do not
 * have and a literal we do not want.  Instead: with `a` in r0 and a copy in
 * r1, and `b` a value whose bits outside the field are don't-care,
 *
 *      x = a ^ b ; y = x & field ; z = a ^ y
 *
 * gives z = (a & ~field) | (b & field).  Bits outside the field xor with zero
 * twice; bits inside come out as b.  Two xors and an `and #imm` -- and it is
 * the `and #imm` that lets the caller be sloppy about b's high bits. */
static void merge_low_field(fgl_emitter *e, int field)
{
	sh4_emit_xor(&e->cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_and_imm(&e->cg, field);
	sh4_emit_xor(&e->cg, FGL_R_T1, FGL_R_XFER);
}

/* The mode stack push an exception performs: SR bits 3:0 move up to 5:2, and
 * 1:0 come in zero -- kernel mode, interrupts off.  Bits 5:4 (the OLD pair)
 * are pushed off the end and lost.
 *
 * `cpuspecifications.md`, cop0r12: bit0 IEc / bit1 KUc are current, bit2 IEp /
 * bit3 KUp previous, bit4 IEo / bit5 KUo old.  The runtime writes exactly
 * `SR = (SR & ~0x3f) | ((SR & 0x0f) << 2)` (r3000a.c:139), and
 * `(SR << 2) & 0x3f` is that same value -- the field mask does the `& 0x0f`,
 * so the shift stands alone. */
static void emit_sr_push(fgl_emitter *e)
{
	sh4_emit_mov_l_load_gbr(&e->cg, CP0_AT(COP0_SR));
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, FGL_R_T1);
	sh4_emit_shll2(&e->cg, FGL_R_XFER);
	merge_low_field(e, 0x3f);
	sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_SR));
}

/* CAUSE for a synchronous exception taken outside a delay slot.
 *
 * `CAUSE = (CAUSE & 0x700) | (code << 2)`, which is `psxException` verbatim
 * (r3000a.c:136, with bdt = 0).  Bits 10:8 are the two software interrupt
 * bits and the hardware one and MUST survive -- a trap that cleared them
 * would lose an interrupt the guest has not looked at yet.  Everything else,
 * BD and BT at 31:30 included, is written zero, which is correct here because
 * SYSCALL and BREAK reach this emitter only outside a delay slot: `is_transfer`
 * counts them as terminators (decode.c:654) and the decoder does not call
 * `decode_transfer` for a delay slot.
 *
 * Three instructions rather than a masked literal: shifting the field down to
 * the bottom brings it inside `and #imm`'s reach, and `or #imm` then supplies
 * the ExcCode -- 0x20 for Sys, 0x24 for Bp, both 8-bit. */
static void emit_cause(fgl_emitter *e, unsigned excode)
{
	sh4_emit_mov_l_load_gbr(&e->cg, CP0_AT(COP0_CAUSE));
	sh4_emit_shlr8(&e->cg, FGL_R_XFER);
	sh4_emit_and_imm(&e->cg, 0x07);
	sh4_emit_shll8(&e->cg, FGL_R_XFER);
	sh4_emit_or_imm(&e->cg, (int)(excode << 2));
	sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_CAUSE));
}

/* ---------------------------------------------------------------- */
/* Multiply and divide                                               */
/* ---------------------------------------------------------------- */

/* Where one half of the pair ends up.  A host register means a move and no
 * host register means a store, like any other destination -- but there are
 * two of them and neither is `p->rd`, so this is not the
 * `st_guest(e, p->rd, rd)` tail every other node ends with. */
static void st_pair(fgl_emitter *e, int host, unsigned g, int src)
{
	if (host < 0) {
		st_guest(e, g, src);
		return;
	}
	if (host != src)
		sh4_emit_mov_reg(&e->cg, src, host);
}

/* THE 32-STEP DIVIDE, UNSIGNED, AND THE ONLY PART OF THIS THAT IS HARDWARE.
 *
 * `div0u` clears M, Q and T; then each `rotcl`/`div1` pair shifts one bit of
 * the dividend into the partial remainder and one quotient bit back in at the
 * bottom, the partial remainder's 33rd bit living in Q.  After 32 pairs the
 * dividend register holds the quotient bar its last bit, which the trailing
 * `rotcl` supplies.
 *
 * The divisor must be non-zero.  A divide by zero here does not fault, it
 * produces a number, and MIPS wants a specific different one -- the caller
 * tests for it.
 *
 * `dividend` is destroyed and becomes the quotient, `rem` is written from
 * scratch and becomes the remainder, `divisor` is only read.
 *
 * THE SEQUENCE IS NON-RESTORING, so a step that subtracts too much is not
 * undone -- it is compensated by the next step adding.  After the last step
 * there is no next one, so the remainder is left one divisor low exactly when
 * that last subtraction failed.  T already IS that answer, because it is also
 * the quotient's last bit.  The quotient comes out right either way, which is
 * why the correction is easy to leave out and hard to notice: only code that
 * reads HI ever sees it.  Neither `bt` nor `add` writes T, so the trailing
 * `rotcl` still gets the bit it came for.
 *
 * 69 words, and there is no shorter form: a loop would have to carry T across
 * the back edge, and every loop-control instruction on this machine writes
 * it. */
static void emit_div_core(fgl_emitter *e, int dividend, int divisor, int rem)
{
	int i, exact;

	sh4_emit_mov_imm(&e->cg, 0, rem);
	sh4_emit_div0u(&e->cg);
	for (i = 0; i < 32; i++) {
		sh4_emit_rotcl(&e->cg, dividend);
		sh4_emit_div1(&e->cg, divisor, rem);
	}

	exact = bt_fwd(e);
	sh4_emit_add_reg(&e->cg, divisor, rem);
	patch_fwd8(e, exact);

	sh4_emit_rotcl(&e->cg, dividend);
}

/* |Rn|, in place, with no second register and no constant. */
static void emit_abs(fgl_emitter *e, int rn)
{
	int pos;

	sh4_emit_cmppz(&e->cg, rn);
	pos = bt_fwd(e);
	sh4_emit_neg(&e->cg, rn, rn);
	patch_fwd8(e, pos);
}

/* Negate `rn` when `sign` is negative: the fixup that turns an unsigned
 * result back into a signed one. */
static void emit_neg_if(fgl_emitter *e, int sign, int rn)
{
	int pos;

	sh4_emit_cmppz(&e->cg, sign);
	pos = bt_fwd(e);
	sh4_emit_neg(&e->cg, rn, rn);
	patch_fwd8(e, pos);
}

/* MULT, MULTU, DIV, DIVU.
 *
 * The multiplies are three instructions.  The divides are a hundred, because
 * SH-4 has no divide and the architecture's own sequence is 32 unrolled
 * steps; see NOTES for the service routine that lifts them out of the call
 * site once fgl has somewhere to put one.
 *
 * The dividend has to be somewhere the core may destroy that is not the
 * allocator's, so it is copied to r1.  The remainder needs a third register,
 * which is the scratch alloc.c already grants this node.  The divisor stays
 * where the allocator left it for DIVU -- the core only reads it -- and is
 * copied to r0 for DIV, which negates it.
 *
 * DIVIDE BY ZERO IS NOT A TRAP ON THIS GUEST.  It has defined results, and
 * they are produced here rather than left to whatever the sequence happens to
 * do:
 *
 *      DIVU, d == 0:   LO = 0xffffffff             r3000a.h:363-364
 *                      HI = n
 *      DIV,  d == 0:   LO = n >= 0 ? 0xffffffff : 1
 *                      HI = n                      r3000a.h:348-350
 *
 * The other special case needs no code at all.  `0x80000000 / -1` is defined
 * as LO = 0x80000000, HI = 0 (r3000a.h:351-353), and that is what falls out:
 * |n| is 0x80000000 and |d| is 1, the core divides them to 0x80000000
 * remainder 0, and the quotient's sign `n ^ d` has its top bit clear so
 * nothing is negated.
 *
 * THE TWO SIGN WORDS GO ON THE STACK, NOT IN THE STATE BLOCK'S TEMP WORD.
 * That word is the parking slot for a deferred load (ir.h, THE LOAD SHADOW),
 * and a divide is allowed to BE the shadow instruction sitting between an
 * IR_LOAD and its IR_TEMP_GET -- writing it here would eat the parked value.
 * Both pushes are past the divide-by-zero branch, so every path through this
 * node pops exactly what it pushed.
 */
static void emit_muldiv(fgl_emitter *e, const ir_node *p)
{
	int dividend = FGL_R_T1;
	int rem = p->sc[0];
	int divisor;
	int to_zero, done;
	int a, b;
	/* A half nothing reads is a half nothing computes. The allocator is
	 * told the same thing (alloc.c) and gives the dead half no register,
	 * so there is nothing holding a stale value for the block's writeback
	 * to publish. */
	int no_lo = (p->hint & FGL_H_NO_LO) != 0;
	int no_hi = (p->hint & FGL_H_NO_HI) != 0;
	int check_zero = !(p->hint & FGL_H_NO_DIV_CHK);

	if (p->sub == MD_MULT || p->sub == MD_MULTU) {
		/* `dmuls.l`/`dmulu.l` leave the whole 64-bit product in
		 * MACH:MACL, MACL the low half -- which is LO, MACH being HI.
		 * `mul.l` would save nothing here and leaves MACH undefined,
		 * so it is only usable where HI is dead; the IR does not say
		 * that, so it is not used. */
		a = operand(e, p->hs, p->rs, FGL_R_T1);
		b = operand(e, p->ht, p->rt, FGL_R_XFER);

		/* `mul.l` computes only the low 32 bits and leaves MACH
		 * undefined, so it is usable exactly when the optimiser has
		 * shown nothing reads HI. That is the whole of what NO_HI buys
		 * here, and it is why the comment above says the IR does not
		 * say that -- now it does. */
		if (no_hi)
			sh4_emit_mul_l(&e->cg, a, b);
		else if (p->sub == MD_MULT)
			sh4_emit_dmuls_l(&e->cg, a, b);
		else
			sh4_emit_dmulu_l(&e->cg, a, b);

		/* Both halves sit in MAC until they are read, so the order is
		 * free and LO goes first to match the reference's. */
		if (!no_lo) {
			a = p->hd >= 0 ? p->hd : FGL_R_XFER;
			sh4_emit_sts_macl(&e->cg, a);
			st_pair(e, p->hd, GUEST_LO, a);
		}

		if (!no_hi) {
			a = p->hx >= 0 ? p->hx : FGL_R_XFER;
			sh4_emit_sts_mach(&e->cg, a);
			st_pair(e, p->hx, GUEST_HI, a);
		}
		return;
	}

	if (p->sub != MD_DIV && p->sub != MD_DIVU) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	if (!rem) {
		e->overflow = 6;
		return;
	}

	/* The dividend is taken first: loading it from the state block goes
	 * through r0, which is where the divisor is about to live. */
	a = operand(e, p->hs, p->rs, FGL_R_T1);
	if (a != dividend)
		sh4_emit_mov_reg(&e->cg, a, dividend);

	if (p->sub == MD_DIVU && p->ht >= 0) {
		divisor = p->ht;                /* read-only: no copy needed */
	} else {
		divisor = FGL_R_XFER;
		b = operand(e, p->ht, p->rt, FGL_R_XFER);
		if (b != divisor)
			sh4_emit_mov_reg(&e->cg, b, divisor);
	}

	/* THE DIVIDE-BY-ZERO PATH, AND WHEN IT IS NOT THERE.
	 *
	 * A zero divisor is defined on the R3000A rather than trapping -- the
	 * quotient is -1 or 1 by the sign of the dividend and the remainder is
	 * the dividend -- so the check is part of the instruction, not a
	 * safety net, and it is emitted unless the optimiser proved the
	 * divisor cannot be zero. `LIGHTREC_NO_DIV_CHECK` is that proof, and
	 * nothing here infers it: an inferred one would remove a defined
	 * result and the wrong answer would be a plausible number.
	 *
	 * Note what is NOT skipped with it: the signed path's sign handling
	 * stays exactly as it was. Only the zero test and its branch go. */
	if (check_zero) {
		sh4_emit_tst(&e->cg, divisor, divisor);
		to_zero = bt_fwd(e);
	} else {
		to_zero = 0;
	}

	if (p->sub == MD_DIVU) {
		emit_div_core(e, dividend, divisor, rem);

		if (check_zero) {
			done = bra_fwd(e);
			patch_fwd8(e, to_zero);
			sh4_emit_mov_reg(&e->cg, dividend, rem);   /* HI = n  */
			sh4_emit_mov_imm(&e->cg, -1, dividend);    /* LO = -1 */
			patch_fwd12(e, done);
		}
	} else {
		int nonneg;

		/* The quotient takes the sign of the operands' exclusive or
		 * and the remainder the sign of the dividend -- C's
		 * truncating division, which is what `n / d` and `n % d` mean
		 * at r3000a.h:354-355, not a floor.  Neither sign survives
		 * the core: both operands are made positive, and `rem` is
		 * built by the core itself. */
		sh4_emit_mov_reg(&e->cg, dividend, rem);
		sh4_emit_xor(&e->cg, divisor, rem);             /* n ^ d */
		sh4_emit_mov_l_store_dec(&e->cg, rem, 15);
		sh4_emit_mov_l_store_dec(&e->cg, dividend, 15);

		emit_abs(e, dividend);
		emit_abs(e, divisor);

		emit_div_core(e, dividend, divisor, rem);

		/* r0 holds the divisor, which the core's own correction was
		 * the last reader of, so the sign words come back into it. */
		sh4_emit_mov_l_load_inc(&e->cg, 15, FGL_R_XFER);   /* n     */
		emit_neg_if(e, FGL_R_XFER, rem);
		sh4_emit_mov_l_load_inc(&e->cg, 15, FGL_R_XFER);   /* n ^ d */
		emit_neg_if(e, FGL_R_XFER, dividend);

		if (check_zero) {
			done = bra_fwd(e);

			patch_fwd8(e, to_zero);
			sh4_emit_mov_reg(&e->cg, dividend, rem);   /* HI = n */
			sh4_emit_mov_imm(&e->cg, -1, dividend);
			sh4_emit_cmppz(&e->cg, rem);
			nonneg = bt_fwd(e);
			sh4_emit_mov_imm(&e->cg, 1, dividend);
			patch_fwd8(e, nonneg);
			patch_fwd12(e, done);
		}
	}

	/* The core computes both halves whatever happens -- a 32-step divide
	 * produces its remainder on the way to its quotient -- so a dead half
	 * saves the store, not the work. */
	if (!no_lo)
		st_pair(e, p->hd, GUEST_LO, dividend);
	if (!no_hi)
		st_pair(e, p->hx, GUEST_HI, rem);
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
		e->overflow = 7;
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
		e->overflow = 8;
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

	/* SWL and SWR write inside ONE aligned word, which is exactly what one
	 * table entry covers, so the aligned address this path already built
	 * is the right thing to invalidate and no third scratch is needed --
	 * both of these are dead the instant the merge is stored. */
	emit_invalidate(e, p, s0, s1);
}

/* ---------------------------------------------------------------- */
/* The block                                                         */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Hardware registers                                                */
/* ---------------------------------------------------------------- */

/* WHEN A LOAD OR A STORE IS NOT MEMORY.
 *
 * lightrec's optimiser tags every access with the region it proved, and fgl
 * lowers most of them to the two-instruction masked access that is the whole
 * point of having a region analysis.  `FGL_IO_HW` is the tag that says the
 * proof went the other way: this address is a device register, and reading it
 * has side effects that a `mov.l` does not produce.
 *
 * WHY THIS IS A CALL AND NOT A FAULT.  bloop reaches its device model by
 * leaving the address unmapped and catching the exception, which costs no
 * instructions at the site at all.  Measured on hardware that path is 170-230
 * cycles against 27 for a call, because the cost is in the exception entry and
 * not in anything that strides or caches; see build/docs/CLAUDE.md 3c, which
 * also records what would flip the decision back.
 *
 * WHAT THE CALLEE ALREADY DOES, so that the call site does not do it twice:
 * `lightrec_hw_lb` and its family `kunseg()` the address themselves, and they
 * return a value ALREADY EXTENDED to 32 bits -- sign for lb/lh, zero for
 * lbu/lhu (lightrec.c, the `(u32)(s32)(s8)` and `(u8)` casts).  So there is no
 * extension here, and a call site that added one would sign-extend an already
 * sign-extended byte, which is invisible until the byte has its top bit set.
 *
 * WHAT NEEDS NO FLUSH, which is the reason this path is cheap.  These take
 * their address and their value as ARGUMENTS and hand the result back as a
 * return value; they do not read or write the guest register file.  So the
 * allocator may keep every guest register in a host register across the call
 * and nothing has to be spilled.  That is exactly not true of the generic
 * `lightrec_rw` path, which works on the register file in the state block --
 * see build/docs/fgl_step4_generic_io.md.
 */
static void emit_hw_load(fgl_emitter *e, const ir_node *p)
{
	uint32_t fn;
	int rs = p->sc[0];
	int rd;

	if (!e->tgt || !e->tgt->shim_call || p->sub >= 5 ||
	    !(fn = e->tgt->hw_load[p->sub]) || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	/* The address is the shim's first argument, so it is built in r1 and
	 * r0 is the spare -- the opposite of the direct path, which keeps r0
	 * for the address it is about to dereference. */
	emit_addr_raw(e, p, FGL_R_T1, FGL_R_XFER);
	emit_const(e, fn, FGL_R_XFER);
	emit_const(e, e->tgt->shim_call, rs);
	sh4_emit_jsr(&e->cg, rs);
	sh4_emit_nop(&e->cg);           /* delay slot */

	/* The value comes back in r0, and from here this is the tail of an
	 * ordinary load: parked for the shadow if deferred, otherwise written
	 * wherever the allocator put the destination. */
	rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
	if (p->defer) {
		sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
	} else if (p->hd >= 0) {
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, rd);
	} else {
		sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(p->rd));
	}
}

/* The store, which is the one service with three values to pass and therefore
 * the one that parks a value in the state block on the way.  See shim.h. */
static void emit_hw_store(fgl_emitter *e, const ir_node *p)
{
	uint32_t fn;
	int rs = p->sc[0];
	int rt;

	if (!e->tgt || !e->tgt->shim_call_st || p->sub >= 5 ||
	    !(fn = e->tgt->hw_store[p->sub]) || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	/* The value first and through r0, because `mov.l Rm,@(disp,GBR)` has
	 * no other source register -- and it goes to memory before the address
	 * is built, so the address computation is free to use r0 as its spare
	 * afterwards. */
	rt = operand(e, p->ht, p->rt, FGL_R_XFER);
	if (rt != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, rt, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_SHIM_ARG);

	emit_addr_raw(e, p, FGL_R_T1, FGL_R_XFER);
	emit_const(e, fn, FGL_R_XFER);
	emit_const(e, e->tgt->shim_call_st, rs);
	sh4_emit_jsr(&e->cg, rs);
	sh4_emit_nop(&e->cg);           /* delay slot */
}

/* The whole access, done by C.
 *
 * Everything about this node is in the state block: C reads the base register
 * from there and writes the destination back to there, which is why the
 * allocation pass flushes around it and why there are no operands to set up.
 * All that is left at the call site is the guest instruction word.
 *
 * The cycle counters are reconciled by the shim, which matters more here than
 * anywhere else: an unproven address can reach a device, a device can raise
 * an interrupt, and an interrupt is delivered by moving `target_cycle` in
 * memory where a register cannot see it. */
static void emit_rw(fgl_emitter *e, const ir_node *p)
{
	int rs = p->sc[0];

	if (!e->tgt || !e->tgt->shim_call || !e->tgt->rw || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	emit_const(e, e->tgt->rw, FGL_R_XFER);
	emit_const(e, p->imm, FGL_R_T1);        /* the guest instruction word */
	emit_const(e, e->tgt->shim_call, rs);
	sh4_emit_jsr(&e->cg, rs);
	sh4_emit_nop(&e->cg);                   /* delay slot */
}

/* Does this access reach a device rather than memory?
 *
 * Everything the optimiser could name a plain region for is memory, including
 * FGL_IO_DIRECT_HW -- that tag means the FRONT END certified this particular
 * hardware address as ordinary storage (optimizer.c, `ops.hw_direct`), so the
 * masked direct access is right for it under the same flat-map assumption RAM
 * and BIOS already rely on.
 *
 * FGL_IO_UNKNOWN is deliberately NOT here.  It is not memory either, but it
 * needs the generic `lightrec_rw` protocol rather than this one, and routing
 * it here would silently give it a device access it may not want.  Until that
 * path exists an unknown region is a refusal, which is what having no fallback
 * means. */
static int is_hw(const ir_node *p)
{
	return p->io == FGL_IO_HW;
}

/* ---------------------------------------------------------------- */
/* COP2 commands                                                     */
/* ---------------------------------------------------------------- */

/* THE ONE PLACE A BLOCK CALLS OUT, AND WHAT IT COSTS TO GET THERE.
 *
 * A GTE command is thousands of instructions of fixed-point geometry; there
 * was never a version of this that got inlined.  What there is instead is the
 * cheapest possible way to leave and come back:
 *
 *      mov.l   @(disp,pc), r0          ; the body, resolved right now
 *      mov.l   @(disp,pc), r1          ; the guest command word
 *      mov.l   @(disp,pc), rS          ; fgl_shim_gte
 *      jsr     @rS
 *       nop
 *
 * The body is chosen HERE, at compile time, because the command word is a
 * constant in the block -- so nothing decodes it at run time and nothing
 * compares it against a table on every execution.  This is the same decision
 * lightrec's own SH-4 path made (emitter.c, rec_CP2_gte) and the reason it
 * skipped the generic C wrapper: a COP2 command reads and writes the COP2
 * file in the state block and touches neither the cycle counter nor the
 * guest's general registers.
 *
 * WHY THERE IS NO FLUSH HERE.  The callee reads the COP2 file out of the
 * state block, and the COP2 file is never held in a host register -- IR_MTC2
 * stores straight through to the state block and IR_MFC2 loads straight out
 * of it.  So the file the callee sees is already the current one, and the
 * guest GPRs the allocator is holding in r3-r12 are none of its business.
 * If COP2 registers ever start living in registers, this comment is the thing
 * that stops being true.
 *
 * A command the hardware ignores resolves to nothing, and the right amount of
 * code for it is none -- not a call to an empty function. */
static void emit_gte(fgl_emitter *e, const ir_node *p)
{
	uint32_t body;
	int rs = p->sc[0];

	if (!e->tgt || !e->tgt->gte_body || !e->tgt->shim_gte) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	body = e->tgt->gte_body(e->tgt->user, p->imm);
	if (!body)
		return;

	/* The allocator owes this node a scratch register, because `jsr` wants
	 * its target in a general register and r0 and r1 are both carrying
	 * arguments. Without one there is nowhere to put the shim's address. */
	if (!rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	emit_const(e, body, FGL_R_XFER);
	emit_const(e, p->imm, FGL_R_T1);
	emit_const(e, e->tgt->shim_gte, rs);
	sh4_emit_jsr(&e->cg, rs);
	sh4_emit_nop(&e->cg);           /* delay slot */
}

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
		if (is_hw(p)) {
			emit_hw_load(e, p);
			break;
		}
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
		if (is_hw(p)) {
			emit_hw_store(e, p);
			break;
		}
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
		emit_invalidate(e, p, FGL_R_T1, p->sc[0]);
		break;

	case IR_LOAD_UN:
		emit_load_un(e, p);
		break;

	case IR_STORE_UN:
		emit_store_un(e, p);
		break;

	case IR_MFC0:
		/* COP0 sits inside GBR's reach like everything else, so a
		 * coprocessor read is the same one instruction an ordinary
		 * state access is -- no address computation, no second file. */
		sh4_emit_mov_l_load_gbr(&e->cg, (int)(FGL_AT_COP0 + (p->imm & 31)));
		if (p->defer)
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		else if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(p->rd));
		break;

	case IR_MTC_C:
		/* The same call shape as IR_RW, and flushed the same way --
		 * C reads the source register out of the state block. */
		rs = p->sc[0];
		if (!e->tgt || !e->tgt->shim_call || !e->tgt->mtc || !rs) {
			e->unsupported = 1;
			e->unsupported_op = p->op;
			break;
		}
		emit_const(e, e->tgt->mtc, FGL_R_XFER);
		emit_const(e, p->imm, FGL_R_T1);   /* the guest instruction */
		emit_const(e, e->tgt->shim_call, rs);
		sh4_emit_jsr(&e->cg, rs);
		sh4_emit_nop(&e->cg);              /* delay slot */
		break;

	case IR_MTC0:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)(FGL_AT_COP0 + (p->imm & 31)));
		break;

	case IR_STOP:
		/* AN EXCEPTION IS AN ORDINARY BLOCK EXIT.
		 *
		 * Taking one is four state writes and a change of PC, and the
		 * block can do all five itself: EPC, CAUSE and SR into the
		 * state block, and the vector into the exit register, which
		 * the epilogue publishes to `next_pc` like any other
		 * destination.  So no service routine, no call out of
		 * generated code, and nothing in the runtime that has to know
		 * this block ended differently from a `j` -- the dispatcher
		 * compiles the handler at the vector the way it compiles any
		 * other address, and a guest that installed its own handler
		 * there gets it.
		 *
		 * The allocation pass has already flushed every live guest
		 * register (alloc.c, IR_STOP), so the handler's block sees
		 * coherent state.
		 *
		 * EPC first, SR last: the SR read has to see the value from
		 * BEFORE the push, and anything added here later that wants
		 * the old SR (a BEV test, say) finds it still in r1. */
		emit_const(e, p->imm, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_EPC));

		emit_cause(e, p->sub);
		emit_sr_push(e);

		/* WHICH VECTOR, AND WHY IT IS TESTED RATHER THAN ASSUMED.
		 *
		 * SR bit 22 (BEV) picks the bootstrap vector at bfc00180h
		 * over the normal one at 80000080h. It is tempting to skip
		 * the test on the grounds that the PSX never sets BEV -- but
		 * the runtime we are keeping does test it (`psxException`,
		 * deps/pcsx_rearmed/libpcsxcore/r3000a.c:135), and a guest
		 * that sets BEV and then traps would otherwise be sent
		 * somewhere the interpreter would not send it. Ten
		 * instructions on a path that runs once per BIOS call is not
		 * where this project's time goes.
		 *
		 * `emit_sr_push` leaves the OLD SR in r1, which is what the
		 * test needs -- the push clears BEV's neighbours but not BEV,
		 * so either copy would do; taking the old one means nothing
		 * here depends on that staying true.
		 *
		 * Bit 22 is brought to bit 31 by two shifts and read with
		 * `cmp/pz`, rather than masked: `tst #imm` reaches only the
		 * low byte. */
		{
			int bev, over;

			sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
			sh4_emit_shll8(&e->cg, FGL_R_XFER);
			sh4_emit_shll(&e->cg, FGL_R_XFER);
			sh4_emit_cmppz(&e->cg, FGL_R_XFER);
			bev = bf_fwd(e);                /* T clear = BEV set */
			emit_const(e, GUEST_EXC_VECTOR, FGL_R_EXIT);
			over = bra_fwd(e);
			patch_fwd8(e, bev);
			emit_const(e, GUEST_EXC_VECTOR_BEV, FGL_R_EXIT);
			patch_fwd12(e, over);
		}
		break;

	case IR_RFE:
		/* Pop the same stack: SR bits 5:2 copied down to 3:0, bits 5:4
		 * left alone -- so the top pair ends up duplicated.
		 * `cpuspecifications.md`, "cop0cmd=10h - RFE opcode": "bit2-3
		 * are copied to bit0-1, and bit4-5 are copied to bit2-3, all
		 * other bits (including bit4-5) are left unchanged".  Both
		 * moves at once are `(SR >> 2) & 0x0f` into the low nibble,
		 * which is lightrec's `rec_cp0_RFE` (deps/lightrec/emitter.c:
		 * 3226) as well.
		 *
		 * RFE does NOT jump to EPC -- the handler does that with its
		 * own `jr`, and this instruction is what sits in that jump's
		 * delay slot.  So there is nothing to write to the exit
		 * register here. */
		sh4_emit_mov_l_load_gbr(&e->cg, CP0_AT(COP0_SR));
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, FGL_R_T1);
		sh4_emit_shlr2(&e->cg, FGL_R_XFER);
		merge_low_field(e, 0x0f);
		sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_SR));
		break;

	case IR_MFC2:
		/* One load and one move.  No sign extension on the way out:
		 * the sixteen-bit registers are narrowed on the way IN
		 * (see IR_MTC2), so what is in the state block is already
		 * the value a read must hand back -- r3000a.h:483-484 reads
		 * `cp2[]` raw for exactly that reason. */
		sh4_emit_mov_l_load_gbr(&e->cg, (int)p->imm);
		if (p->defer)
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		else if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(p->rd));
		break;

	case IR_MTC2:
		/* `imm2` IS THE HALFWORD FLAG HERE, not a displacement
		 * (decode.c:406) -- the field is used for one thing on this
		 * node and something else entirely on IR_LWC2/IR_SWC2.
		 *
		 * Nine of the sixty-four registers keep only the low
		 * halfword of a write and hand back its sign extension
		 * (decode.c:64-104; nocash GTE section, "Writing 32bit
		 * values to 16bit GTE registers by software does not
		 * trigger saturation").  The decoder has already worked out
		 * WHICH register this is; narrowing it is still the
		 * emitter's job, and it is one `exts.w` because the
		 * register number was known at compile time.  The reference
		 * does the same narrowing on the write side,
		 * `r3k_cop2_in` (r3000a.h:279-286). */
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		if (p->imm2)
			sh4_emit_exts_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)p->imm);
		break;

	case IR_LWC2:
		/* Address in r0 with r1 free for a wide displacement, the
		 * same pairing IR_LOAD uses, and the loaded word goes
		 * straight back out through r0 -- so the whole node holds
		 * nothing live across the state access.
		 *
		 * No narrowing, deliberately: the reference stores the full
		 * word (r3000a.h:518), and matching it is what the oracle
		 * compares against.  See NOTES. */
		emit_cop2_addr(e, p, FGL_R_XFER, FGL_R_T1);
		sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)p->imm);
		break;

	case IR_SWC2:
		/* Address in r1 first, as in IR_STORE: r0 has to stay free
		 * to carry the coprocessor register out of the state block,
		 * and there is no other register that can. */
		emit_cop2_addr(e, p, FGL_R_T1, FGL_R_XFER);
		sh4_emit_mov_l_load_gbr(&e->cg, (int)p->imm);
		sh4_emit_mov_l_store(&e->cg, FGL_R_XFER, FGL_R_T1);
		break;

	case IR_MULDIV:
		emit_muldiv(e, p);
		break;

	case IR_GTE:
		emit_gte(e, p);
		break;

	case IR_RW:
		emit_rw(e, p);
		break;

	case IR_JUMP:
		emit_const(e, p->imm, FGL_R_EXIT);
		break;

	/* LEAVING FOR C, WHICH IS DONE BY ARITHMETIC AND NOT BY A BRANCH.
	 *
	 * lightrec has no "return to the caller" instruction in a block. What
	 * it has is one budget test in the dispatcher, so a block that wants
	 * out reconciles the absolute counters and sets the delta to zero;
	 * the dispatcher regains control by the ordinary path, finds the
	 * budget spent and returns to C, which reads `exit_flags` to find out
	 * that this was a request rather than a timeout.
	 *
	 * The reconciliation is `current = target = target - delta`, which is
	 * what `rec_exit_early` does (emitter.c) -- the delta being what is
	 * left unspent, so subtracting it from the target is what the machine
	 * actually reached.
	 *
	 * The block still runs its own epilogue after this, which charges for
	 * the instructions and publishes the exit PC. That charge lands on a
	 * delta that is now zero and makes it negative, which is harmless:
	 * the test is `<= 0` and the counters C reads were already written.
	 *
	 * Seven instructions, and every exit code is a power of two below 64,
	 * so the flag is an immediate and costs no literal. */
	case IR_EXIT:
		sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_TARGET_CYCLE);
		sh4_emit_sub(&e->cg, FGL_R_CYCLE, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_TARGET_CYCLE);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_CURRENT_CYCLE);
		sh4_emit_mov_imm(&e->cg, 0, FGL_R_CYCLE);

		sh4_emit_mov_imm(&e->cg, (int)p->imm, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_EXIT_FLAGS);

		/* Where C resumes. The epilogue publishes it like any other
		 * exit, so there is nothing further to do here. */
		emit_const(e, p->imm2, FGL_R_EXIT);
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

uint32_t fgl_emit(fgl_emitter *e, const ir_node *ir, int n, const ir_alloc *a,
		  unsigned n_ops)
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

	/* THE EPILOGUE: CHARGE FOR THE BLOCK, AND LEAVE THROUGH THE DISPATCHER.
	 *
	 * Five instructions.  The block does not return -- it jumps to the
	 * address in the state block, carrying the guest PC it is leaving for
	 * in r2, which is where the exit register has held it all along.
	 *
	 * IT USED TO BE `rts`, AND THAT WAS ONE INSTRUCTION CHEAPER-LOOKING
	 * AND STRICTLY WORSE.  Returning means PR holds the block's return
	 * address for the block's entire life, so PR silently joins the
	 * register contract and every service routine called from inside a
	 * block has to save and restore it -- while fgl.h says a service may
	 * clobber "r0 and r1 and nothing else" and does not mention PR.  The
	 * `rts` form also filled its delay slot usefully, with the store of
	 * the exit PC to `next_pc`, so it looked like it was getting that
	 * publication for free.  It was not free; it was unnecessary.  The
	 * dispatcher reads r2.
	 *
	 * So: same five instructions, PR is nobody's, and `next_pc` is written
	 * only on the paths where C actually reads it.  The `nop` is forced --
	 * `mov.l @(disp,GBR),Rn` exists only for R0, so the dispatch address
	 * has to land in r0, and an instruction in a `jmp`'s delay slot may
	 * not modify the jump's target register.
	 *
	 * The last thing this buys is the one that matters later: a jump
	 * through a slot can be PATCHED into a direct branch to the block that
	 * follows it.  An `rts` can never be linked to anything.
	 *
	 * A block that consumed no guest instructions charges nothing, which
	 * is entry 0 of the table and still a correct load.
	 *
	 * A BLOCK CAN BE LONGER THAN THE TABLE.  `IR_MAX_NODES` bounds the IR,
	 * not the guest instructions behind it: a constant fold (front.c's
	 * `movi_step`) turns a LUI/ORI pair into no nodes at all, and BIOS
	 * init code is largely made of those.  Blocks of fifty guest ops
	 * against a forty-node budget are ordinary.  This used to CLAMP, which
	 * silently charged 33 ops for all of them -- the block at `bfc001f0`
	 * is 63 ops, and fgl billed it 57 cycles against the interpreter's
	 * 110.  A guest clock that runs slow is invisible to a register
	 * comparison until an interrupt lands on a different instruction.
	 *
	 * So charge in whole table-loads and then the remainder.  Two extra
	 * instructions per 33 ops, on the epilogue of a long block only. */
	while (n_ops > FGL_CYCLE_ENTRIES - 1) {
		sh4_emit_mov_l_load_gbr(&e->cg,
					(int)(FGL_AT_CYCLES + FGL_CYCLE_ENTRIES - 1));
		sh4_emit_sub(&e->cg, FGL_R_XFER, FGL_R_CYCLE);
		n_ops -= FGL_CYCLE_ENTRIES - 1;
	}
	sh4_emit_mov_l_load_gbr(&e->cg, (int)(FGL_AT_CYCLES + n_ops));
	sh4_emit_sub(&e->cg, FGL_R_XFER, FGL_R_CYCLE);

	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_DISPATCH);
	sh4_emit_jmp(&e->cg, FGL_R_XFER);
	sh4_emit_nop(&e->cg);                                  /* delay slot */

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
	return fgl_emit(e, ir, n, &a, (unsigned)ir_block_length(words));
}
