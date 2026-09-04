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
static struct lightrec_state *fgl_crash_state;

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
void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why)
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
	int n, n2;

	if (!why)
		why = &dummy_why;
	*why = -EINVAL;                 /* a hole in fgl until proven otherwise */


	fgl_targets_once();
	fgl_crash_handler_once(cstate->state);

	memset(&info, 0, sizeof info);
	n = fgl_front(block->opcode_list, block->nb_ops, block->pc,
		      ir, IR_MAX_NODES, &info);
	if (n <= 0 || info.unsupported) {
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
	ir_allocate(ir, n, &alloc);
	if (!fgl_emit(&e, ir, n, &alloc, info.n_ops)) {
		fprintf(stderr, "fgl: FIRST PASS REFUSED pc=%08x n=%d ops=%u "
			"ovf=%d unsup=%d op=%u\n", (unsigned)block->pc, n,
			info.n_ops, e.overflow, e.unsupported, e.unsupported_op);
		return NULL;
	}
	size = fgl_size(&e);
	n2 = n;

	code = lightrec_alloc_code(state, size);
	if (!code) {
		*why = -ENOMEM;
		return NULL;
	}

	/* Second pass, at the real address. The allocation pass is not run
	 * again: its result depends on the IR, not on where the code lands. */
	fgl_init(&e, code, size, (u32)(uintptr_t)code);
	fgl_set_targets(&e, &fgl_dc_targets);
	if (!fgl_emit(&e, ir, n, &alloc, info.n_ops) || fgl_size(&e) != size) {
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
