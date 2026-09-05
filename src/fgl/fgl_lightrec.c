/* Where fgl and lightrec have to agree, and the proof that they do.
 *
 * fgl's emitter bakes state-block displacements into instruction words at
 * compile time.  It gets them from `fgl_state.h`, which is a list of integer
 * literals -- it has to be, because the host test rig compiles fgl without
 * lightrec's headers in scope, and that is the whole reason the oracle can
 * run on a workstation.  So the two descriptions of the same memory live in
 * two files and nothing but this translation unit makes them one.
 *
 * THAT IS THE FAILURE THIS FILE EXISTS TO PREVENT, AND IT IS A QUIET ONE.
 * A field inserted into `struct lightrec_state` above `current_cycle` does
 * not break a build and does not fault.  Generated code goes on issuing
 * `mov.l @(167*4,GBR),r0`, that displacement now names `target_cycle`
 * instead, and the emulator runs -- wrongly, in a way that surfaces as a game
 * hanging some minutes later on hardware, with nothing pointing here.
 *
 * So every displacement fgl can emit is checked against `offsetof` below.  A
 * mismatch is a compile error naming the field.
 */

#include <stddef.h>

#include "lightrec-private.h"

#include "fgl_state.h"
#include "fgl.h"
#include "decode_int.h"

#define FGL_ASSERT(cond, name) \
	typedef char fgl_layout_##name[(cond) ? 1 : -1]

/* The register file.  A guest register number IS its displacement: this is
 * what lets the writeback flush emit a store from the register number with no
 * translation table, and what makes LO and HI ordinary registers rather than
 * a special case (lightrec.h:120 gives gpr[] 34 entries for exactly that). */
FGL_ASSERT(offsetof(struct lightrec_state, regs.gpr) == 0, gpr_base);
FGL_ASSERT(GUEST_AT(0) * 4 == offsetof(struct lightrec_state, regs.gpr[0]),
	   gpr_scale);
FGL_ASSERT(GUEST_LO * 4 == offsetof(struct lightrec_state, regs.gpr[32]),
	   guest_lo);
FGL_ASSERT(GUEST_HI * 4 == offsetof(struct lightrec_state, regs.gpr[33]),
	   guest_hi);
FGL_ASSERT(GUEST_LO == REG_LO && GUEST_HI == REG_HI, lo_hi_agree);

/* The coprocessor files. */
FGL_ASSERT(FGL_AT_COP0 * 4 == offsetof(struct lightrec_state, regs.cp0),
	   cop0_base);
FGL_ASSERT(FGL_AT_CP2D * 4 == offsetof(struct lightrec_state, regs.cp2d),
	   cp2d_base);
FGL_ASSERT(FGL_AT_CP2C * 4 == offsetof(struct lightrec_state, regs.cp2c),
	   cp2c_base);

/* The scratch word and the two PCs. */
FGL_ASSERT(FGL_AT_TEMP_REG * 4 == offsetof(struct lightrec_state, temp_reg),
	   temp_reg);
FGL_ASSERT(FGL_AT_TEMP_REG == REG_TEMP, temp_reg_agrees);
FGL_ASSERT(FGL_AT_CURR_PC * 4 == offsetof(struct lightrec_state, curr_pc),
	   curr_pc);
FGL_ASSERT(FGL_AT_NEXT_PC * 4 == offsetof(struct lightrec_state, next_pc),
	   next_pc);

/* The cycle table, whose displacement is an instruction count. */
FGL_ASSERT(FGL_AT_CYCLES * 4 == offsetof(struct lightrec_state, cycle_table),
	   cycle_table);
FGL_ASSERT(FGL_CYCLE_ENTRIES == LIGHTREC_CYCLE_ENTRIES, cycle_entries);

/* The counters and the flag, which only the exit paths touch. */
FGL_ASSERT(FGL_AT_CURRENT_CYCLE * 4
	   == offsetof(struct lightrec_state, current_cycle), current_cycle);
FGL_ASSERT(FGL_AT_TARGET_CYCLE * 4
	   == offsetof(struct lightrec_state, target_cycle), target_cycle);
FGL_ASSERT(FGL_AT_EXIT_FLAGS * 4
	   == offsetof(struct lightrec_state, exit_flags), exit_flags);

/* Where a finished block jumps.  Read by every block that ever runs, so a
 * wrong displacement here is not a subtle desynchronisation -- it is a jump to
 * whatever `exit_flags` happened to contain. */
FGL_ASSERT(FGL_AT_DISPATCH * 4
	   == offsetof(struct lightrec_state, dispatch), dispatch);
FGL_ASSERT(FGL_AT_LUT * 4
	   == offsetof(struct lightrec_state, lut_base), lut_base);
FGL_ASSERT(FGL_AT_ADDR_MASK * 4
	   == offsetof(struct lightrec_state, addr_mask), addr_mask);
FGL_ASSERT(FGL_AT_SHIM_ARG * 4
	   == offsetof(struct lightrec_state, shim_arg), shim_arg);

/* THE REACH ITSELF.  `mov.l @(disp,GBR),r0` has an eight-bit displacement
 * scaled by four, so the last word generated code can name is at +1020.
 * Everything asserted above is inside that; this says so once, rather than
 * leaving it to be rediscovered when a field is appended. */
FGL_ASSERT(FGL_STATE_WORDS <= 256, gbr_reach);

/* And the COP0 registers fgl actually keeps. */
FGL_ASSERT(COP0_SR == 12 && COP0_CAUSE == 13 && COP0_EPC == 14, cop0_regs);

/* ------------------------------------------------------------------ */
/* The seam itself                                                     */
/* ------------------------------------------------------------------ */

/* Everything above this line is a static assertion and generates no code.
 * What follows is what lightrec calls into fgl: one function to compile a
 * block, and the table of service addresses the emitter bakes into it.
 *
 * The other half of the seam -- the C that fgl's ASSEMBLY calls -- is in
 * lightrec.c instead, because two of the three functions it needs
 * (`lightrec_memset`, `lightrec_check_load_delay`) are static there.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "fgl.h"
#include "front.h"
#include "shim.h"

#include <arch/irq.h>
#include <arch/arch.h>

#include "blockcache.h"
/* Declared rather than included: `gte_fpu.h` pulls in libpcsxcore's headers,
 * which this translation unit has no other need of and which the host layout
 * check cannot see. lightrec's own emitter declared it the same way and for
 * the same reason. */
extern void *gte_fpu_resolve(u32 op);
extern uint32_t psxCP2CtrlGen;

/* HOW BIG A BLOCK CAN GET BEFORE IT IS MEASURED.
 *
 * The first emission pass has nowhere to put the code yet, so it goes into a
 * fixed buffer whose only job is to be big enough.  32 guest instructions is
 * the ceiling (IR_MAX_INSNS), and the most expensive lowering fgl has is an
 * inline signed division at 98 SH-4 words, so 32 * 98 * 2 is the bound with
 * the literal pool still to come.  Overshooting costs nothing but BSS;
 * undershooting is caught -- `fgl_emit` sets `overflow` and returns 0, and the
 * block simply does not compile. */
#define FGL_SCRATCH_BYTES 16384

/* The COP2 body for one command, resolved at compile time.  `gte_fpu_resolve`
 * returns NULL for a command the hardware ignores, and zero means the same
 * thing to the emitter: emit nothing at all. */
static u32 fgl_gte_body(void *user, u32 op)
{
	(void)user;
	return (u32)(uintptr_t)gte_fpu_resolve(op);
}

/* An access whose region the optimiser could not prove.  C performs the whole
 * thing against the state block, which is why the allocation pass flushes
 * around the call -- see ir.h on IR_RW.
 *
 * This is `lightrec_rw_helper`'s job, written out here rather than called,
 * because that function is static and because the wrapper protocol it was
 * written for -- a packed argument and a selector index into `c_wrappers[]` --
 * does not exist any more.
 *
 * THE LOAD-DELAY BRANCH OF IT IS DELIBERATELY ABSENT.  lightrec parks the
 * value in `temp_reg` when `in_delay_slot_n` says the access is in a branch's
 * delay slot; fgl declines that case in `front.c` rather than lowering it, so
 * reaching here with one would mean the front end let something through.
 * Implementing it here as well would be a second mechanism for one thing. */
void fgl_rw(u32 opcode, struct lightrec_state *state)
{
	union code op = { .opcode = opcode };
	u32 ret;

	ret = lightrec_rw(state, op, state->regs.gpr[op.i.rs],
			  state->regs.gpr[op.i.rt], NULL, NULL, 0);

	switch (op.i.op) {
	case OP_LB:
	case OP_LBU:
	case OP_LH:
	case OP_LHU:
	case OP_LWL:
	case OP_LWR:
	case OP_LW:
	case OP_META_LWU:
		if (op.i.rt)
			state->regs.gpr[op.i.rt] = ret;
		break;
	default:
		break;
	}
}

/* A write to COP0 Status or Cause.  `lightrec_mtc_cb`'s job, minus its LWC2
 * branch, which is a COP2 path and cannot arrive here: only `ir_mtc_needs_c`
 * builds the node and it tests for MTC0.
 *
 * The whole reason this is a call and not two instructions is bit 16 of
 * Status.  See ir.h on IR_MTC_C. */
void fgl_mtc(u32 opcode, struct lightrec_state *state)
{
	union code op = { .opcode = opcode };

	lightrec_mtc(state, op, op.r.rd, state->regs.gpr[op.r.rt]);
}

/* A COP2 read that is not a load: `IRGB` and `ORGB`.  `lightrec_mfc2`'s job,
 * and it is exported, so this is only the argument order and the destination
 * write -- the same shape `fgl_rw` has for the same reason. */
void fgl_mfc(u32 opcode, struct lightrec_state *state)
{
	union code op = { .opcode = opcode };

	if (op.r.rt)
		state->regs.gpr[op.r.rt] = lightrec_mfc(state, op);
}

/* Returning from an exception.  `lightrec_rfe` pops the interrupt-enable
 * stack and writes Status back through `lightrec_mtc0`, which is where the
 * pending-interrupt check lives.  See ir.h on IR_RFE. */
void fgl_rfe(u32 unused, struct lightrec_state *state)
{
	(void) unused;

	lightrec_rfe(state);
}

/* ---------------------------------------------------------------- */
/* Faults inside emitted code                                        */
/* ---------------------------------------------------------------- */

/* WHAT A CRASH IN GENERATED CODE LOOKS LIKE WITHOUT THIS.
 *
 * KOS prints the SH-4 register file and a stack trace, and the trace stops at
 * `lightrec_execute` because the frame that faulted was written by fgl and is
 * in no symbol table.  The PC is a bare address in the code buffer.  That is
 * enough to know the fault was in emitted code and nothing else -- not which
 * guest instruction, not which lowering.
 *
 * So the reporter resolves the address back through the block cache and
 * prints the guest PC it was compiled from, together with the SH-4 words
 * around the fault.  The instruction form alone usually names the emit path:
 * a `mov.l Rm,@Rn` is an ordinary store, a `mov.l Rm,@(R0,Rn)` is the code-LUT
 * invalidation, and so on.
 *
 * It handles the four data faults a bad address can raise and nothing else;
 * everything KOS already explains is left to KOS. */
/* DEBUG: the dispatcher's ring of guest block PCs.  Slot 0 is the index,
 * then 64 two-word slots -- the guest PC and guest $v1 as the block was
 * entered -- youngest at index.  Written by eight
 * instructions in `dispatch.S` at every block entry; read only here. */
u32 fgl_pc_ring[1 + 64 * 4];

static struct lightrec_state *fgl_crash_state;

/* A window of guest RAM, for reading against a register that should have
 * been a pointer into it. */
void fgl_dump_mem_pub(u32 addr, unsigned words);
static void fgl_dump_mem(u32 addr, unsigned words)
{
	const struct lightrec_mem_map *ram =
		&fgl_crash_state->maps[PSX_MAP_KERNEL_USER_RAM];
	u32 base = (kunseg(addr) & ~15u) - ram->pc;
	const u32 *w;
	unsigned k;

	if (base >= ram->length)
		return;

	w = (const u32 *)((u8 *)ram->address + base);

	for (k = 0; k < words; k++)
		fprintf(stderr, "fgl: mem %08x  %08x\n",
			(unsigned)((addr & ~15u) + 4u * k), w[k]);
}

/* The last 64 block entries, oldest first. */
void fgl_dump_ring(void)
{
	unsigned idx = fgl_pc_ring[0] & 63u;
	unsigned k;

	for (k = 0; k < 64; k++) {
		unsigned e = (idx + 1u + k) & 63u;

		if (fgl_pc_ring[1 + 4 * e])
			fprintf(stderr, "fgl: came from %08x  $v1=%08x "
				"sp0=%08x $s8=%08x\n",
				fgl_pc_ring[1 + 4 * e],
				fgl_pc_ring[2 + 4 * e],
				fgl_pc_ring[3 + 4 * e],
				fgl_pc_ring[4 + 4 * e]);
	}
}

/* One block's guest instructions, straight out of guest RAM. */
static void fgl_dump_guest(const struct block *b, unsigned max)
{
	const struct lightrec_mem_map *ram =
		&fgl_crash_state->maps[PSX_MAP_KERNEL_USER_RAM];
	u32 off = kunseg(b->pc) - ram->pc;
	const u32 *w;
	unsigned k;

	if (off >= ram->length)
		return;

	w = (const u32 *)((u8 *)ram->address + off);

	for (k = 0; k < b->nb_ops && k < max; k++)
		fprintf(stderr, "fgl: guest %08x  %08x\n",
			(unsigned)(b->pc + 4u * k), w[k]);

	/* AND WHAT FGL WAS ACTUALLY GIVEN, which is not the same thing.
	 *
	 * fgl compiles lightrec's optimised opcode list, not guest RAM: the
	 * optimiser folds constants, rewrites base registers, substitutes meta
	 * opcodes and swaps delay slots, and it hangs the region and hazard
	 * flags off each entry.  Reproducing a miscompile from the RAM words
	 * reproduces a different block. */
	if (block_has_flag(b, BLOCK_NO_OPCODE_LIST) || !b->opcode_list)
		return;

	for (k = 0; k < b->nb_ops && k < max; k++)
		fprintf(stderr, "fgl: op %2u  %08x flags=%08x\n", k,
			b->opcode_list[k].c.opcode, b->opcode_list[k].flags);
}



/* DEBUG: an emitted store computed an address outside guest RAM and outside
 * the scratchpad.  Whatever base register it used is wrong, and the ring
 * still holds the blocks that produced it. */
void fgl_wild_store(void)
{
	static int said;

	if (said)
		return;
	said = 1;

	fprintf(stderr, "\nfgl: WILD STORE addr=%08x\n",
		fgl_crash_state ?
		((const u32 *)fgl_crash_state)[FGL_AT_TEMP_REG] : 0);
	fgl_dump_ring();
	arch_abort();
}

/* DEBUG: the dispatcher calls this the first time the watched table word
 * holds something that is neither null nor a pointer.  The ring still has
 * the block that wrote it. */
/* The watch is only meaningful once the game is drawing: during boot that
 * word is ordinary data and the test fires on every block, which turns the
 * dispatcher into a C call per block and the guest into a crawl.  pvr.c
 * arms it after the renderer has been busy for a while. */
u32 fgl_ot_armed;

void fgl_bad_ot(void)
{
	static int said;

	if (said)
		return;
	said = 1;

	fprintf(stderr, "\nfgl: table word 80070754 became %08x\n",
		*(volatile u32 *)0x00070754u);
	fgl_dump_ring();
	arch_abort();
}

/* DEBUG: the dispatcher calls this the first time guest $v1 reaches the
 * display-list block as neither null nor a pointer. */
void fgl_bad_v1(void)
{
	static int said;

	if (said)
		return;
	said = 1;

	fprintf(stderr, "\nfgl: FIRST bad $v1 = %08x at 80024ff8\n",
		fgl_crash_state ? fgl_crash_state->regs.gpr[3] : 0);
	fgl_dump_ring();
	arch_abort();
}

static void fgl_crash_report(irq_t code, irq_context_t *ctx, void *data)
{
	uintptr_t pc = (uintptr_t) ctx->pc;
	const uint16_t *insn = (const uint16_t *) (pc & ~1u);
	struct block *block = NULL;
	int i;

	(void) data;

	if (fgl_crash_state)
		block = lightrec_find_block_from_code(fgl_crash_state->block_cache, pc);

	fprintf(stderr, "\nfgl: fault %04x at %08x\n", (unsigned) code,
		(unsigned) pc);

	if (block) {
		fprintf(stderr, "fgl: in the block compiled from guest pc "
			"%08x (%u ops), %u bytes into %u\n",
			(unsigned) block->pc, (unsigned) block->nb_ops,
			(unsigned) (pc - (uintptr_t) block->function),
			(unsigned) block->code_size);
	} else {
		fprintf(stderr, "fgl: no block owns that address -- the fault "
			"is not in emitted code\n");
	}

	for (i = -4; i <= 2; i++)
		fprintf(stderr, "fgl: %s %08x  %04x\n", i ? "  " : "->",
			(unsigned) (pc + 2 * i), insn[i]);

	fgl_dump_ring();

	{
		unsigned idx = fgl_pc_ring[0] & 63u;
		unsigned k;

		/* And the code of the last few, because when the path turns
		 * out to be the normal one the answer is in what those blocks
		 * computed, not in the order they ran. */
		for (k = 4; k >= 1; k--) {
			unsigned e = (idx + 1u - k) & 63u;
			u32 bpc = fgl_pc_ring[1 + 4 * e];
			struct block *b;

			if (!bpc)
				continue;
			b = lightrec_find_block(fgl_crash_state->block_cache,
						bpc);
			if (!b)
				continue;
			fgl_dump_guest(b, 20);
		}
	}

	/* THE GUEST SIDE, WHICH IS THE SIDE THAT EXPLAINS ANYTHING.
	 *
	 * The SH-4 words above say what fgl emitted; they cannot say what the
	 * guest asked for.  A wild address in emitted code is almost always a
	 * guest register that was already wrong when the block started, so
	 * the instructions the block was compiled from and the register file
	 * it ran against are what a fault has to be read against. */
	if (block && fgl_crash_state)
		fgl_dump_guest(block, 24);

	if (fgl_crash_state) {

		/* The registers that are supposed to be pointers, and what
		 * guest memory actually holds there. */
		/* A wide window of the table the load reads from.  Whether the
		 * bad entries are scattered or periodic, and whether they look
		 * like truncated pointers, says who wrote them. */
		fgl_dump_mem(fgl_crash_state->regs.gpr[19], 32);
		fgl_dump_mem(fgl_crash_state->regs.gpr[2] - 64u, 48);

		for (i = 0; i < 32; i += 8)
			fprintf(stderr, "fgl: $%-2d %08x %08x %08x %08x "
				"%08x %08x %08x %08x\n", i,
				fgl_crash_state->regs.gpr[i + 0],
				fgl_crash_state->regs.gpr[i + 1],
				fgl_crash_state->regs.gpr[i + 2],
				fgl_crash_state->regs.gpr[i + 3],
				fgl_crash_state->regs.gpr[i + 4],
				fgl_crash_state->regs.gpr[i + 5],
				fgl_crash_state->regs.gpr[i + 6],
				fgl_crash_state->regs.gpr[i + 7]);
	}

	fprintf(stderr, "fgl: r0-r7  %08x %08x %08x %08x %08x %08x %08x %08x\n",
		ctx->r[0], ctx->r[1], ctx->r[2], ctx->r[3],
		ctx->r[4], ctx->r[5], ctx->r[6], ctx->r[7]);
	fprintf(stderr, "fgl: r8-r15 %08x %08x %08x %08x %08x %08x %08x %08x\n",
		ctx->r[8], ctx->r[9], ctx->r[10], ctx->r[11],
		ctx->r[12], ctx->r[13], ctx->r[14], ctx->r[15]);

	arch_abort();
}

static void fgl_crash_handler_once(struct lightrec_state *state)
{
	static int installed;

	fgl_crash_state = state;

	if (installed)
		return;
	installed = 1;

	irq_set_handler(EXC_DATA_ADDRESS_READ, fgl_crash_report, NULL);
	irq_set_handler(EXC_DATA_ADDRESS_WRITE, fgl_crash_report, NULL);
	irq_set_handler(EXC_DTLB_PV_READ, fgl_crash_report, NULL);
	irq_set_handler(EXC_DTLB_PV_WRITE, fgl_crash_report, NULL);
}

/* WHERE THE SERVICES LIVE, as data rather than as link-time references.
 *
 * `fgl.h` explains why the emitter is told these instead of naming them: the
 * same emitter is compiled by the host oracle, where none of these symbols
 * exist and the SH-4 it emits is run by an interpreter.  This is the one
 * place that fills the table in for real. */
static fgl_targets fgl_dc_targets;

/* FILLED AT RUN TIME, because C will not have it any other way: the address of
 * a function is not a constant expression, so none of these can be a static
 * initialiser however obviously fixed they are after linking.
 *
 * Filling it twice is harmless -- every value is the same on every call -- so
 * the threaded compiler racing here costs nothing and needs no lock. */
static void fgl_targets_once(void)
{
	if (fgl_dc_targets.shim_call)
		return;

	fgl_dc_targets.shim_call_st = (u32)(uintptr_t)fgl_shim_call_st;
	fgl_dc_targets.shim_gte     = (u32)(uintptr_t)fgl_shim_gte;
	fgl_dc_targets.shim_divu    = (u32)(uintptr_t)fgl_shim_divu;
	fgl_dc_targets.shim_div     = (u32)(uintptr_t)fgl_shim_div;

	fgl_dc_targets.hw_load[MEM_B]  = (u32)(uintptr_t)lightrec_hw_lb;
	fgl_dc_targets.hw_load[MEM_BU] = (u32)(uintptr_t)lightrec_hw_lbu;
	fgl_dc_targets.hw_load[MEM_H]  = (u32)(uintptr_t)lightrec_hw_lh;
	fgl_dc_targets.hw_load[MEM_HU] = (u32)(uintptr_t)lightrec_hw_lhu;
	fgl_dc_targets.hw_load[MEM_W]  = (u32)(uintptr_t)lightrec_hw_lw;

	fgl_dc_targets.hw_store[MEM_B] = (u32)(uintptr_t)lightrec_hw_sb;
	fgl_dc_targets.hw_store[MEM_H] = (u32)(uintptr_t)lightrec_hw_sh;
	fgl_dc_targets.hw_store[MEM_W] = (u32)(uintptr_t)lightrec_hw_sw;

	fgl_dc_targets.rw       = (u32)(uintptr_t)fgl_rw;
	fgl_dc_targets.mtc      = (u32)(uintptr_t)fgl_mtc;
	fgl_dc_targets.mfc      = (u32)(uintptr_t)fgl_mfc;
	fgl_dc_targets.rfe      = (u32)(uintptr_t)fgl_rfe;
	fgl_dc_targets.cp2_ctrl_gen = (u32)(uintptr_t)&psxCP2CtrlGen;
	fgl_dc_targets.wild     = (u32)(uintptr_t)fgl_wild_store;
	fgl_dc_targets.gte_body = fgl_gte_body;

	/* Last, and it is what the guard above tests: nothing may observe a
	 * half-filled table. */
	fgl_dc_targets.shim_call = (u32)(uintptr_t)fgl_shim_call;
}

/* ------------------------------------------------------------------ */
/* Compiling one block                                                 */
/* ------------------------------------------------------------------ */

/* fgl emits position-dependent code -- a literal pool reached by PC-relative
 * loads -- so it has to know where the block will live before it emits it,
 * and the arena will not say how big a block is until it has been emitted.
 * The way out is to emit it twice: once into scratch to learn the size, then
 * again, for real, at the address the arena hands back.  `test_reloc` is the
 * proof that two emissions differ in nothing but their base, over 18553
 * blocks at four different bases.
 *
 * ONLY THE FIRST BASIC BLOCK IS COMPILED.  A lightrec block can hold several
 * and `fgl_front` stops at the first control transfer.  The rest are reached
 * through the code table like any other address: the branch leaves with its
 * target in the exit register, the dispatcher finds no entry there, and C
 * compiles a block starting at it.  That is correct, and it costs a dispatch
 * per internal branch -- which is exactly what lightrec's `cstate->targets[]`
 * existed to avoid, so it is the first thing to revisit once local branches
 * are worth optimising.
 */
#if FGL_DUMB_BLOCKS
/* DOES A BLOCK ENDING HERE ORPHAN A LOAD'S SHADOW?
 *
 * Only dumb mode has to ask.  A MIPS load's register write lands one
 * instruction late, and fgl carries that shadow inside a block -- so a block
 * that ends immediately after a load leaves a write owed that the next block
 * knows nothing about, and the value appears an instruction too early.  The
 * normal build never hits it: lightrec's blocks end at transfers, and the
 * transfer standing in a load's shadow is inside the block with it.  Dumb
 * mode cuts every two instructions, so it must check. */
static int dumb_is_load(uint32_t insn)
{
	switch (insn >> 26) {
	case OP_LB: case OP_LH: case OP_LWL: case OP_LW:
	case OP_LBU: case OP_LHU: case OP_LWR:
	case OP_LWC2:
	case OP_META_LWU:
		return 1;
	default:
		return 0;
	}
}
#endif

/* Lower and emit every basic block of `block`'s opcode list, in order, into
 * one emitter -- so the emitted function covers the whole list, exactly as
 * lightrec's own compiler did and as lightrec's bookkeeping assumes.
 *
 * `fgl_front` stops at the first control transfer and reports how far it got
 * in `info.n_ops`; this is the loop that calls it again from there, which is
 * what front.h has always asked its caller to do.  Each basic block is
 * allocated and emitted independently: nothing is pinned across one, so every
 * guest register is in the state block at each boundary and concatenation is
 * sound with no fixups between them.
 *
 * Returns 1 on success, 0 if a basic block could not be lowered -- and then
 * `*info` describes the one that failed, for the caller's diagnostic.  There
 * is no partial success: a function that stops halfway through the list is
 * the bug this exists to remove. */
static int fgl_emit_all(fgl_emitter *e, struct block *block, unsigned nb,
			ir_node *ir, ir_alloc *alloc, fgl_front_info *info,
			struct fgl_entry *ent, unsigned int *n_ent, int *n_out)
{
	unsigned at = 0, k = 0;

	while (at < nb) {
		uint32_t entry;
		int n;

		memset(info, 0, sizeof *info);
		n = fgl_front(block->opcode_list + at, nb - at,
			      block->pc + 4u * at, ir, IR_MAX_NODES, info);
		if (n <= 0 || info->unsupported) {
			*n_out = n;
			return 0;
		}

		/* A basic block that consumed nothing would loop forever. */
		if (!info->n_ops) {
			*n_out = n;
			info->stop_reason = FGL_STOP_UNSUPPORTED;
			return 0;
		}

		if (k >= FGL_MAX_ENTRIES) {
			*n_out = n;
			info->stop_reason = FGL_STOP_FULL;
			return 0;
		}

		ir_allocate(ir, n, alloc);
		entry = fgl_emit(e, ir, n, alloc, info->n_ops);
		if (!entry) {
			*n_out = n;
			return 0;
		}

		ent[k].pc = block->pc + 4u * at;
		ent[k].code = (void *)(uintptr_t)entry;
		k++;

		at += info->n_ops;
	}

	*n_ent = k;
	*n_out = 0;
	return 1;
}

void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why,
			struct fgl_entry *entries, unsigned int *nb_entries)
{
	struct lightrec_state *state = cstate->state;
	static uint8_t scratch[FGL_SCRATCH_BYTES];
	/* Two failures that look the same from the outside and must not be
	 * confused: the arena being full is transient and the caller may flush
	 * and retry, while fgl declining to lower a block is permanent and a
	 * retry is an infinite loop. */
	int dummy_why;
	fgl_front_info info;
	ir_node ir[IR_MAX_NODES];
	ir_alloc alloc;
	fgl_emitter e;
	unsigned size;
	void *code;
	int n;
	unsigned nb;

	if (!why)
		why = &dummy_why;
	*why = -EINVAL;                 /* a hole in fgl until proven otherwise */


	fgl_targets_once();
	fgl_crash_handler_once(cstate->state);

	memset(&info, 0, sizeof info);
	nb = block->nb_ops;
#if FGL_DUMB_BLOCKS
	if (nb > 1) {
		/* ONE INSTRUCTION, OR TWO WHEN THE FIRST IS A TRANSFER.
		 *
		 * A branch and its delay slot are one unit of guest
		 * behaviour, not two -- cutting between them would change
		 * what the machine does rather than only how it is compiled,
		 * and the point of dumb mode is to remove fgl's cleverness
		 * without removing the guest's semantics.  Everything else
		 * gets a block to itself. */
		/* TWO, NOT ONE, AND THE REASON IS THE LOAD SHADOW.
		 *
		 * A MIPS load's result is not visible to the instruction
		 * after it, and fgl tracks that shadow inside a block.  Ending
		 * a block immediately after a load orphans it: the next block
		 * has no idea a write is still owed, so the value lands one
		 * instruction too early.  Measured -- one instruction per
		 * block reported divergences in four BIOS loops that the
		 * normal build gets right, every one of them a load followed
		 * by its consumer.
		 *
		 * Two instructions is the smallest block that still holds a
		 * load and the instruction standing in its shadow, and it
		 * removes just as much of the block formation as one did.  A
		 * transfer landing on the second slot takes a third, because
		 * a branch without its delay slot is not the same program. */
		nb = 2u;
		/* Never end on a load (the shadow), never end on a transfer
		 * (the delay slot).  Either takes another instruction, and a
		 * load in a delay slot wants both. */
		while (nb < block->nb_ops &&
		       (dumb_is_load(block->opcode_list[nb - 1u].opcode) ||
			ir_is_transfer(block->opcode_list[nb - 1u].opcode)))
			nb++;
		if (nb > block->nb_ops)
			nb = block->nb_ops;
	}
#endif
	/* First pass: into scratch, to measure.
	 *
	 * THE BASE MUST NOT BE ZERO, and it is not arbitrary that it is the
	 * scratch buffer's own address. `fgl_emit` reports failure by returning
	 * 0 and success by returning the block's ENTRY ADDRESS -- so a block
	 * emitted at base 0 succeeds and says 0, which is indistinguishable
	 * from having refused. Every block then looks unlowerable, with
	 * `overflow` and `unsupported` both clear to prove nothing was actually
	 * wrong. test_reloc had this same bug and was fixed for it; this is the
	 * same mistake one layer down. */
	fgl_init(&e, scratch, sizeof scratch, (u32)(uintptr_t)scratch);
	fgl_set_targets(&e, &fgl_dc_targets);

	if (!fgl_emit_all(&e, block, nb, ir, &alloc, &info,
			  entries, nb_entries, &n)) {
		static unsigned f, by_reason[8], by_op[64];
		unsigned k;
		f++;
		if (info.stop_reason < 8) by_reason[info.stop_reason]++;
		if ((info.unsupported_op >> 26) < 64) by_op[info.unsupported_op >> 26]++;
		if (f <= 10 || (f % 2000) == 0) {
			fprintf(stderr, "fgl FAIL %u: pc=%08x nb_ops=%u n=%d unsup=%u "
			       "reason=%u op=%08x at=%08x | reasons:",
			       f, (unsigned)block->pc, block->nb_ops, n,
			       info.unsupported, info.stop_reason,
			       info.unsupported_op, info.unsupported_pc);
			for (k = 0; k < 8; k++) fprintf(stderr, " %u", by_reason[k]);
			fprintf(stderr, " | top majors:");
			for (k = 0; k < 64; k++)
				if (by_op[k]) fprintf(stderr, " %02x=%u", k, by_op[k]);
			fprintf(stderr, "\n");
		}
		return NULL;
	}

	size = fgl_size(&e);

	code = lightrec_alloc_code(state, size);
	if (!code) {
		*why = -ENOMEM;
		return NULL;
	}

	/* Second pass, at the real address. The allocation pass is not run
	 * again: its result depends on the IR, not on where the code lands. */
	fgl_init(&e, code, size, (u32)(uintptr_t)code);
	fgl_set_targets(&e, &fgl_dc_targets);
	if (!fgl_emit_all(&e, block, nb, ir, &alloc, &info,
			  entries, nb_entries, &n) || fgl_size(&e) != size) {
		fprintf(stderr, "fgl: SECOND PASS DIFFERS pc=%08x %u vs %u "
			"ovf=%d unsup=%d\n", (unsigned)block->pc,
			fgl_size(&e), size, e.overflow, e.unsupported);
		lightrec_free_code(state, code);
		return NULL;
	}

	if (state->ops.code_inv)
		state->ops.code_inv(code, size);




	*code_size = size;
	return code;
}

/* DEBUG: reachable from lightrec.c's segfault reporter. */
void fgl_dump_mem_pub(u32 addr, unsigned words)
{
	fgl_dump_mem(addr, words);
}
