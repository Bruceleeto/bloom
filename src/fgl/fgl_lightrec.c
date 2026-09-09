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
#include "perf.h"
#include "emitmiss.h"
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
FGL_ASSERT(FGL_AT_LINK * 4
	   == offsetof(struct lightrec_state, link), link);
FGL_ASSERT(FGL_AT_TSAVE * 4
	   == offsetof(struct lightrec_state, tsave), tsave);

/* THE REACH ITSELF.  `mov.l @(disp,GBR),r0` has an eight-bit displacement
 * scaled by four, so the last word generated code can name is at +1020.
 * Everything asserted above is inside that; this says so once, rather than
 * leaving it to be rediscovered when a field is appended. */
FGL_ASSERT(FGL_STATE_WORDS <= 256, gbr_reach);

/* And the COP0 registers fgl actually keeps. */
FGL_ASSERT(COP0_SR == 12 && COP0_CAUSE == 13 && COP0_EPC == 14, cop0_regs);

/* THE INTERPRETER RUNS GUEST CODE TOO, AND IT DOES NOT MODEL THE LOAD DELAY.
 *
 * `ENABLE_FIRST_PASS` makes `fgl_get_next_block` interpret a block once before
 * it is compiled, so lightrec's interpreter executes every instruction fgl
 * ever sees at least once.  fgl pays the MIPS load delay itself -- the shadow
 * rotation in `decode.c` -- but that only covers code fgl EMITTED.  The
 * interpreter's only source of load-delay handling is
 * `lightrec_handle_load_delays` tagging the opcodes, which `OPT_HANDLE_LOAD_
 * DELAYS` gates.
 *
 * With the pass off and the first pass on, nothing anywhere pays the delay
 * during interpretation: the instruction standing in a load's shadow reads the
 * loaded value instead of the previous one.  That is not a subtle drift.  It
 * cost a day here on Spyro's memcard load, where `lw $k0` / `addu $at,$k0` at
 * 0x35a4 handed the store four instructions later a pointer for a base and
 * faulted at 0x5ffffc54 -- with fgl's own emitted code for the block correct
 * and never executed, which is exactly what made it hard to find.
 *
 * So the two are tied together here rather than left to whoever next flips an
 * option.  If you want `OPT_HANDLE_LOAD_DELAYS` off, turn `ENABLE_FIRST_PASS`
 * off in the same breath and fgl becomes the only thing running guest code. */
FGL_ASSERT(!ENABLE_FIRST_PASS || OPT_HANDLE_LOAD_DELAYS, first_pass_load_delay);

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
/* THE BLOCK AND OP INDEX OF A GENERIC ACCESS, from the pc the node published
 * (`emit_publish_pc`, FGL_AT_CURR_PC).  lightrec's own generic wrapper gets
 * both handed to it at compile time; here the pc is any address inside the
 * block, so the block is found by walking back from it.  The index is the
 * list slot at that address, checked against the word: a swapped delay-slot
 * pair puts the branch and its slot in each other's slots, so the
 * neighbours are tried. */
static struct opcode *fgl_rw_op(struct lightrec_state *state, u32 opcode,
				struct block **out, u16 *offset)
{
	u32 pc = state->curr_pc, k;

	/* A lightrec block can be far longer than what fgl lowers of it:
	 * `nb_ops` runs to the first unconditional transfer.  Blocks also
	 * start at every address something once jumped to, so a block found
	 * at a small distance may end before the access: keep walking. */
	for (k = 0; k < 4096; k++) {
		struct block *block =
			lightrec_find_block(state->block_cache, pc - 4u * k);
		int i, d[3] = { 0, 1, -1 };

		if (!block || !block->opcode_list ||
		    block_has_flag(block, BLOCK_NO_OPCODE_LIST) ||
		    k >= block->nb_ops)
			continue;
		for (i = 0; i < 3; i++) {
			int idx = (int)k + d[i];

			if (idx < 0 || idx >= (int)block->nb_ops)
				continue;
			if (block->opcode_list[idx].c.opcode == opcode) {
				*out = block;
				*offset = (u16)idx;
				return &block->opcode_list[idx];
			}
		}
	}
	return NULL;
}

void fgl_rw(u32 opcode, struct lightrec_state *state)
{
	PERF_BEGIN(PERF_RW); EMITMISS_OUT_BEGIN();
	union code op = { .opcode = opcode };
	struct block *block = NULL;
	struct opcode *lop;
	u16 offset = 0;
	u32 ret;

	/* TAG THE ACCESS, AS LIGHTREC'S GENERIC WRAPPER DOES.  An access the
	 * optimiser could not place is performed by C once, `lightrec_rw`
	 * records where it went in the op's flags and marks the block for
	 * recompilation, and the recompiled block lowers it for real.  With
	 * NULL flags none of that happened: every untagged access stayed on
	 * this path for ever, and a region -- compiled from a list nothing has
	 * run yet -- is made of untagged accesses.  Spyro's pad routine made
	 * 68k generic calls per 40 vsyncs against 4.5k after the fix. */
	lop = fgl_rw_op(state, opcode, &block, &offset);
	ret = lightrec_rw(state, op, state->regs.gpr[op.i.rs],
			  state->regs.gpr[op.i.rt], lop ? &lop->flags : NULL,
			  block, offset);

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

	EMITMISS_OUT_END(); PERF_END(PERF_RW);
}

/* A write to COP0 Status or Cause.  `lightrec_mtc_cb`'s job, minus its LWC2
 * branch, which is a COP2 path and cannot arrive here: only `ir_mtc_needs_c`
 * builds the node and it tests for MTC0.
 *
 * The whole reason this is a call and not two instructions is bit 16 of
 * Status.  See ir.h on IR_MTC_C. */
void fgl_mtc(u32 opcode, struct lightrec_state *state)
{
	PERF_BEGIN(PERF_COP); EMITMISS_OUT_BEGIN();
	union code op = { .opcode = opcode };

	lightrec_mtc(state, op, op.r.rd, state->regs.gpr[op.r.rt]);
	EMITMISS_OUT_END(); PERF_END(PERF_COP);
}

/* A COP2 read that is not a load: `IRGB` and `ORGB`.  `lightrec_mfc2`'s job,
 * and it is exported, so this is only the argument order and the destination
 * write -- the same shape `fgl_rw` has for the same reason. */
void fgl_mfc(u32 opcode, struct lightrec_state *state)
{
	PERF_BEGIN(PERF_COP); EMITMISS_OUT_BEGIN();
	union code op = { .opcode = opcode };

	if (op.r.rt)
		state->regs.gpr[op.r.rt] = lightrec_mfc(state, op);
	EMITMISS_OUT_END(); PERF_END(PERF_COP);
}

/* Returning from an exception.  `lightrec_rfe` pops the interrupt-enable
 * stack and writes Status back through `lightrec_mtc0`, which is where the
 * pending-interrupt check lives.  See ir.h on IR_RFE. */
void fgl_rfe(u32 unused, struct lightrec_state *state)
{
	PERF_BEGIN(PERF_COP); EMITMISS_OUT_BEGIN();

	(void) unused;

	lightrec_rfe(state);
	EMITMISS_OUT_END(); PERF_END(PERF_COP);
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

/* Dispatcher entries, bumped by four instructions at `.Lrun` when
 * FGL_BLOCK_COUNT.  Always defined so the reporter links either way; it stays
 * zero in a build that does not count.  See src/emitmiss.h for why this is
 * dispatcher entries and NOT blocks executed. */
u32 fgl_blocks_run;

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

/* FGL_STATS: how blocks were cut. */
unsigned long fgl_region_blocks, fgl_region_ranges, fgl_region_fallbacks;

/* ONE lightrec BLOCK, AS SEVERAL BASIC BLOCKS IN ONE CODE BLOCK.
 *
 * `fgl_front` lowers up to the first control transfer.  Called once, that
 * compiled the first basic block of a lightrec block and left every internal
 * branch to exit.  Measured against GNU lightning on the same savestate:
 * 4135 fgl blocks against 1722, and the hot loop at 800269cc is one 780-byte
 * lightning block against 22 fgl blocks of 2240 bytes, every branch target
 * in the loop restarting a block with its own entry, publish, exit and pool.
 *
 * So the whole opcode list is lowered here, one `fgl_front` per basic block,
 * and emitted into one code block as a REGION (fgl.h): every basic block
 * start is a label, and a branch to a label is a host `bra`.  The cuts are
 * (a) the target of every LIGHTREC_LOCAL_BRANCH, so each is a label, and (b)
 * wherever `fgl_front` stops on its own -- a transfer, a full IR array, an
 * opcode it hands to C.  A range that fails to lower ends the region there;
 * branches to anything past it are ordinary link sites, and the tail is
 * compiled as its own block when it is reached, as before.
 *
 * Registers: each basic block preloads and flushes on its own, exactly as
 * separate blocks did, so a label sees the same machine a block entry does
 * -- pins in their registers, the pool written back.  Cycles: each basic
 * block charges its own guest instructions.  Budget: backward edges test
 * r14 (a loop), forward edges only after a crossing into C; see
 * emit_local_edge. */
#define FGL_MAX_RANGES 24

/* THE BISECTION SWITCH.  0 is the pre-region shape: one basic block per
 * compiled block, every internal branch leaving through the dispatcher.
 * The other three changes of this port (the alloc.c hoist fix, fgl_rw
 * tagging, the delay-slot filler) are unaffected by it, so flipping this
 * says which half a fault belongs to.
 *
 * 1 IS THE WHOLE BLOCK, AND IT COMPILES CODE THAT NEVER RUNS.  A range that
 * ends on an unconditional transfer is followed by whatever the opcode list
 * holds next, reachable or not, and the region keeps going to the end of the
 * block.  Measured against no regions on the same savestate: twice the guest
 * instructions compiled, 751 KiB of host code against 470, and the DC pays
 * the difference in data stalls -- 73.4 ms a frame against 66.6.
 *
 * 3 IS LEADERS ONLY: the same region, stopped at the first transfer whose
 * fall-through is not a branch target of this block.  Loops stay inside the
 * region, which is the whole point of regions, and the cold tail is left to
 * the dispatcher to compile if it is ever reached. */
#ifndef FGL_REGIONS
#define FGL_REGIONS 3
#endif

/* The per-512-block progress line.  Off by default; it is the only way
 * to see the fallback rate, which must stay 0. */
#ifndef FGL_REGION_STATS
#define FGL_REGION_STATS 0
#endif

struct fgl_range {
	ir_node   ir[IR_MAX_NODES];
	ir_alloc  alloc;
	int       n;
	unsigned  n_ops;
	uint32_t  pc;
	unsigned  stop;
};

/* Cut the opcode list into ranges.  Returns the count, or 0 with `info`
 * describing the first range's refusal.  `whole` 0 is the pre-region shape:
 * the first basic block only. */
static int fgl_cut_ranges(const struct block *block, struct fgl_range *r,
			  int max, int whole, unsigned nb,
			  fgl_front_info *info)
{
	static uint8_t lead[4096];
	const struct opcode *ops = block->opcode_list;
	unsigned i, k;
	int nr = 0;

	if (nb > sizeof lead)
		whole = 0;

	if (whole) {
		memset(lead, 0, nb);
		for (i = 0; i < nb; i++) {
			uint32_t at, tgt;

			/* BIT(3) IS LOCAL_BRANCH ON A BRANCH ONLY; on a store
			 * it is something else.  The same opcode test as
			 * lightrec's `is_local_branch`. */
			switch (ops[i].c.i.op) {
			case OP_BEQ: case OP_BNE: case OP_BLEZ: case OP_BGTZ:
			case OP_REGIMM:
				break;
			default:
				continue;
			}
			if (!op_flag_local_branch(ops[i].flags))
				continue;
			/* A swapped delay-slot pair moves the branch's index,
			 * not its address. */
			at = block->pc + 4u * i -
			     4u * (uint32_t)!!op_flag_no_ds(ops[i].flags);
			tgt = at + 4u +
			      ((uint32_t)(int32_t)(int16_t)ops[i].c.i.imm << 2);
			if (tgt >= block->pc && (tgt - block->pc) / 4u < nb)
				lead[(tgt - block->pc) / 4u] = 1;
		}
	}

	for (k = 0; k < nb && nr < max; nr++) {
		unsigned lim = nb;
		struct fgl_range *q = &r[nr];

		if (whole)
			for (i = k + 1; i < nb; i++)
				if (lead[i]) { lim = i; break; }

		memset(info, 0, sizeof *info);
		q->pc = (uint32_t)block->pc + 4u * k;
		q->n = fgl_front(&ops[k], lim - k, q->pc, q->ir, IR_MAX_NODES,
				 info);
		if (q->n <= 0 || info->unsupported)
			break;
		q->n_ops = info->n_ops;
		q->stop = info->stop_reason;
		k += info->n_ops;

		/* LEADERS ONLY: STOP WHERE THE FALL-THROUGH IS UNREACHABLE.
		 *
		 * A range that ended on an unconditional transfer hands off to
		 * the next opcode in the list, and nothing branches there
		 * unless it is one of this block's own branch targets.  Taking
		 * it anyway is how the whole-block shape came to compile twice
		 * the guest code that no regions does. */
		if (whole == 3 && q->stop == FGL_STOP_TRANSFER &&
		    !(k < nb && lead[k])) {
			nr++;
			break;
		}

		if (!whole || !info->n_ops) {   /* one block, or C resumes here */
			nr++;
			break;
		}
	}
	return nr;
}

/* One emission pass of `nr` ranges at `buf`.  Returns the size, 0 on
 * refusal. */
static unsigned fgl_emit_ranges(fgl_emitter *e, void *buf, unsigned bufsize,
				struct fgl_range *r, int nr,
				const uint32_t *label_pcs, int region)
{
	int i;

	fgl_init(e, buf, bufsize, (u32)(uintptr_t)buf);
	fgl_set_targets(e, &fgl_dc_targets);
	if (region)
		fgl_region_begin(e, label_pcs, nr);
	for (i = 0; i < nr; i++) {
		/* Where the range's cycle charge is measured from: the guest
		 * pc it is entered at, not the first node that survived
		 * folding. */
		e->charge_pc = r[i].pc;
		if (!fgl_emit(e, r[i].ir, r[i].n, &r[i].alloc, r[i].n_ops))
			return 0;
	}
	if (e->region && fgl_region_end(e))
		return 0;
	return fgl_size(e);
}

void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why,
			struct fgl_entry *entries, unsigned int *nb_entries)
{
	struct lightrec_state *state = cstate->state;
	/* ALIGNED LIKE THE ARENA, AND THE TWO-PASS PROTOCOL DEPENDS ON IT.
	 * The link site is padded to a four-byte boundary, so whether the pad
	 * exists is a function of the base address -- and the measuring pass
	 * runs at this buffer's address while the real pass runs in the arena.
	 * Two differently aligned bases give two different sizes and the block
	 * is refused for "SECOND PASS DIFFERS".
	 *
	 * 32, to match the cache-line alignment `lightrec_alloc_code` gives
	 * every block entry.  Four would still satisfy the pad rule -- 32 is a
	 * superset -- but keeping the two bases congruent is the property this
	 * comment exists to protect. */
	static uint8_t scratch[FGL_SCRATCH_BYTES] __attribute__((aligned(32)));
	/* Two failures that look the same from the outside and must not be
	 * confused: the arena being full is transient and the caller may flush
	 * and retry, while fgl declining to lower a block is permanent and a
	 * retry is an infinite loop. */
	int dummy_why;
	fgl_front_info info;
	static struct fgl_range ranges[FGL_MAX_RANGES];
	static uint32_t label_pcs[FGL_MAX_RANGES];
	fgl_emitter e;
	unsigned size;
	void *code;
	int nr, i, whole = FGL_REGIONS;
	unsigned nb;

	if (!why)
		why = &dummy_why;
	*why = -EINVAL;                 /* a hole in fgl until proven otherwise */


	fgl_targets_once();
	fgl_crash_handler_once(cstate->state);

	nb = block->nb_ops;
#if FGL_DUMB_BLOCKS
	whole = 0;
	if (nb > 1) {
		/* ONE INSTRUCTION, OR TWO WHEN THE FIRST IS A TRANSFER.  A
		 * branch and its delay slot are one unit of guest behaviour;
		 * and never end on a load, because a MIPS load's write lands
		 * one instruction late and fgl carries that shadow inside a
		 * block. */
		nb = 2u;
		while (nb < block->nb_ops &&
		       (dumb_is_load(block->opcode_list[nb - 1u].opcode) ||
			ir_is_transfer(block->opcode_list[nb - 1u].opcode)))
			nb++;
		if (nb > block->nb_ops)
			nb = block->nb_ops;
	}
#endif

	nr = fgl_cut_ranges(block, ranges, FGL_MAX_RANGES, whole, nb, &info);
	if (nr <= 0) {
		static unsigned f, by_reason[8], by_op[64];
		unsigned k;
		f++;
		if (info.stop_reason < 8) by_reason[info.stop_reason]++;
		if ((info.unsupported_op >> 26) < 64) by_op[info.unsupported_op >> 26]++;
		if (f <= 10 || (f % 2000) == 0) {
			fprintf(stderr, "fgl FAIL %u: pc=%08x nb_ops=%u n=%d unsup=%u "
			       "reason=%u op=%08x at=%08x | reasons:",
			       f, (unsigned)block->pc, block->nb_ops, ranges[0].n,
			       info.unsupported, (unsigned)info.stop_reason,
			       (unsigned)info.unsupported_op,
			       (unsigned)info.unsupported_pc);
			for (k = 0; k < 8; k++) fprintf(stderr, " %u", by_reason[k]);
			fprintf(stderr, " | top majors:");
			for (k = 0; k < 64; k++)
				if (by_op[k]) fprintf(stderr, " %02x=%u", k, by_op[k]);
			fprintf(stderr, "\n");
		}
		return NULL;
	}

	for (i = 0; i < nr; i++) {
		ir_allocate(ranges[i].ir, ranges[i].n, &ranges[i].alloc);
		label_pcs[i] = ranges[i].pc;
	}

	/* First pass: into scratch, to measure.
	 *
	 * THE BASE MUST NOT BE ZERO, and it is not arbitrary that it is the
	 * scratch buffer's own address. `fgl_emit` reports failure by
	 * returning 0 and success by returning the block's ENTRY ADDRESS --
	 * so a block emitted at base 0 succeeds and says 0, which is
	 * indistinguishable from having refused. */
	size = fgl_emit_ranges(&e, scratch, sizeof scratch, ranges, nr,
			       label_pcs, whole);
	if (!size && nr > 1) {
		/* THE REGION DID NOT FIT -- a `bra` out of 12-bit reach, the
		 * scratch buffer, a label never reached, overflow 5/6/7.  The
		 * old shape still works: the first basic block alone, the
		 * rest compiled as blocks when they are reached. */
		fgl_region_fallbacks++;
		nr = fgl_cut_ranges(block, ranges, 1, 0, nb, &info);
		if (nr != 1)
			return NULL;
		ir_allocate(ranges[0].ir, ranges[0].n, &ranges[0].alloc);
		label_pcs[0] = ranges[0].pc;
		whole = 0;
		size = fgl_emit_ranges(&e, scratch, sizeof scratch, ranges, 1,
				       label_pcs, 0);
	}
	if (!size) {
		fprintf(stderr, "fgl: FIRST PASS REFUSED pc=%08x ranges=%d "
			"ovf=%d unsup=%d op=%u\n", (unsigned)block->pc, nr,
			e.overflow, e.unsupported, e.unsupported_op);
		return NULL;
	}
	fgl_region_blocks++;
	fgl_region_ranges += nr;
	/* HOW THE BLOCKS WERE CUT, and the one number worth watching: a
	 * nonzero fallback rate on the DC means `bra` reach or the scratch
	 * size differs from what the Linux tree measured (0 in 1889). */
#if FGL_REGION_STATS
	if ((fgl_region_blocks & 511u) == 0)
		printf("fgl: regions %lu blocks / %lu basic blocks, "
		       "%lu fallbacks\n", fgl_region_blocks,
		       fgl_region_ranges, fgl_region_fallbacks);
#endif

	code = lightrec_alloc_code(state, size);
	if (!code) {
		*why = -ENOMEM;
		return NULL;
	}

	/* Second pass, at the real address.  The allocation pass is not run
	 * again: its result depends on the IR, not on where the code lands. */
	if (fgl_emit_ranges(&e, code, size, ranges, nr, label_pcs, whole)
	    != size) {
		fprintf(stderr, "fgl: SECOND PASS DIFFERS pc=%08x %u vs %u "
			"ovf=%d unsup=%d\n", (unsigned)block->pc,
			(unsigned)fgl_size(&e), size, e.overflow,
			e.unsupported);
		lightrec_free_code(state, code);
		return NULL;
	}

	/* ONE ENTRY, AT THE BLOCK'S OWN PC.  The internal basic blocks are
	 * reached by `bra` from inside the region now, not through the LUT. */
	entries[0].pc = block->pc;
	entries[0].code = code;
	*nb_entries = 1;

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

/* ---------------------------------------------------------------- */
/* CROSS-BLOCK LINKING: THE DISPATCH THAT DELETES ITSELF             */
/* ---------------------------------------------------------------- */

/* A BLOCK EXIT IS EIGHTEEN INSTRUCTIONS AND ONLY TWO OF THEM ARE THE JUMP.
 *
 * Leaving a block costs the epilogue -- load the cycle constant, `sub` it off
 * r14, load the dispatch address, `jmp`, `nop` -- and then the dispatcher:
 * publish `curr_pc`, test the budget, ten instructions of branchless block
 * table arithmetic, load the slot, test it, and an INDIRECT jump.  All of that
 * to reach an address that, for a branch with a constant target, was known
 * when the block was compiled.
 *
 * The reason it is not simply branched to is that at compile time the
 * successor usually does not exist yet.  So the exit is emitted as a call to
 * `fgl_link_stub`, and the stub asks this function where the successor went;
 * once there is an answer, the CALL SITE IS REWRITTEN INTO A `bra` and the
 * question is never asked again.  The steady-state edge is `bra` + delay slot,
 * with no memory reference and no indirect branch for the pipeline to stall on
 * -- and the epilogue that is no longer emitted is footprint the icache does
 * not have to carry, which is the measured lever on this machine.
 *
 * WHAT IT COSTS, AND IT IS NOT FREE.  A patched link is a branch baked into
 * another block's code with no indirection left in front of it, so the block
 * table can no longer redirect it.  Selective invalidation therefore stops
 * working, and the only invalidation left is to put every link back.  bloop
 * pays exactly the same price and says so (`bloop/src/blocks.h`): it is a
 * consequence of making the common case free, not an oversight.
 *
 * Hence `fgl_unlink_all`, and hence the rule its callers obey: it runs BEFORE
 * anything frees code or clears a table slot, while the sites are still ours.
 */

/* THE CALL SITE, WHICH IS THREE INSTRUCTIONS AND ONE LONGWORD.
 *
 *      site+0  mov.l @(FGL_AT_LINK,gbr), r0
 *      site+2  jsr   @r0
 *      site+4  nop                     <- delay slot; PR is site+6
 *
 * The patch replaces site+0..3 -- the load and the `jsr` -- with `bra target`
 * and its delay slot, in ONE ALIGNED LONGWORD STORE.  Two halfword stores
 * would be visible to the instruction fetcher half-done, and on this machine
 * that is not a theoretical race: the compiler runs on another thread.  A
 * longword store to a two-mod-four address is an address error, so the
 * emitter pads the site to four bytes and this asserts it rather than
 * trusting that it did.
 *
 * The `nop` at site+4 is dead once patched: the `bra` leaves from site+0 and
 * its own delay slot is at site+2.  Two dead bytes per link is the price of
 * being able to store the patch atomically. */
#define FGL_LINK_SITE_W0  ((uint16_t)(0xc600u | FGL_AT_LINK))   /* mov.l @(d,gbr),r0 */
#define FGL_LINK_SITE_W1  ((uint16_t)0x400bu)                   /* jsr @r0           */

/* Little-endian: the halfword at the lower address is the low half. */
#define FGL_PACK(lo, hi)  (((uint32_t)(uint16_t)(hi) << 16) | (uint16_t)(lo))

#define FGL_LINK_SITE_WORD FGL_PACK(FGL_LINK_SITE_W0, FGL_LINK_SITE_W1)

/* Every site this run has patched, so that they can be put back.
 *
 * A fixed array and not a list: it is walked in full or not at all, it is
 * never searched, and an allocation on the compile path is a failure mode
 * this does not need.
 *
 * RUNNING OUT IS WORSE THAN IT LOOKS.  A site that cannot be recorded cannot
 * be patched, and an unpatched site pays the stub AND a C call on every single
 * execution -- strictly worse than never having linked it.  It is not a
 * graceful degradation, it is a cliff.  So this wants to be comfortably above
 * the live edge count, and `fgl_link_patched + fgl_link_range` is the number
 * to check it against: the last steady-state census was ~1160.
 *
 * It was briefly 16384, raised in the same change that added conditional
 * linking, on the guess that the edge count would triple.  Two variables at
 * once and the build faulted; this went back first because it was the guess.
 *
 * AND THE GUESS WAS RIGHT, JUST UNMEASURED AT THE TIME.  4096 is what the
 * census counted before conditional linking and before the backward scan; now
 * every direct branch gets a site and the live set runs well past it.  What
 * kept the old table on the cliff was not only its size but the global sweep
 * on every free, which emptied and refilled it: 253,424 links torn down in a
 * 600-vsync run against 1,156 once teardown became block-scoped.  With the
 * sweep gone the table holds the live set instead of a churn of dead ones.
 *
 * NOT 65536.  Two uint32_t arrays of that length is 512 KiB of bss, and with
 * it Spyro died at `sbrk` during boot -- "Unable to allocate memory", then
 * "Unable to recompile block".  The link table competes with the code heap for
 * the same 16 MiB, so it is bounded by what is left over.  16384 is 128 KiB.
 *
 * `full` in the links report is the refusal count and is the number to watch:
 * if it is not zero, this is too small and the emulator is on the cliff. */
#define FGL_MAX_LINKS 16384
static uint32_t fgl_link_site[FGL_MAX_LINKS];

/* WHICH TABLE SLOT EACH SITE BRANCHES AT, so that a teardown can be about the
 * code that actually died.
 *
 * A site is stored with `lut_offset()` of its successor rather than the guest
 * PC, because that is the unit the invalidation paths work in: they clear
 * `((len + 3) / 4)` slots starting at `lut_offset(addr)`, so a site is stale
 * exactly when its index falls in that window.  Comparing indices makes the
 * two agree by construction instead of by both getting the segment masking
 * right separately. */
static uint32_t fgl_link_slot[FGL_MAX_LINKS];
static unsigned fgl_n_links;

/* HOW MANY LINKS POINT INTO EACH PAGE OF THE TABLE.
 *
 * An invalidation clears two slots and then asked "is any of my 1400 links
 * one of these two?" by walking all 1400.  Spyro does that 213 times a frame
 * and the answer is no every single time: 358k entries visited a frame, 118 ms
 * of it, which was the whole of the `rw` bucket and most of the frame.
 *
 * A count per 1024 slots turns the common answer into one load.  The walk is
 * still there for when a page really does hold links -- this only skips the
 * sweeps that were always going to find nothing.  Exact, not a filter: the
 * count is incremented where a site enters the table and decremented where one
 * leaves, so a zero means zero. */
#define FGL_LINK_PAGE_SHIFT 10
#define FGL_LINK_PAGES ((CODE_LUT_SIZE >> FGL_LINK_PAGE_SHIFT) + 1u)
static uint16_t fgl_link_pgcnt[FGL_LINK_PAGES];

static int fgl_links_in_range(u32 first, u32 last)
{
	u32 p = first >> FGL_LINK_PAGE_SHIFT;
	u32 e = (last - 1u) >> FGL_LINK_PAGE_SHIFT;

	if (e >= FGL_LINK_PAGES)
		e = FGL_LINK_PAGES - 1u;
	for (; p <= e; p++)
		if (fgl_link_pgcnt[p])
			return 1;
	return 0;
}

/* Edges refused because the table was full.  Nonzero means FGL_MAX_LINKS is
 * too small and the emulator is on the cliff described above. */
unsigned fgl_link_full;

/* DIAGNOSTIC: how the edges came out, printed by nothing yet but readable
 * from a debugger and cheap to keep. */
unsigned fgl_link_patched, fgl_link_uncompiled, fgl_link_range, fgl_link_undone;
unsigned fgl_link_calls, fgl_link_bad_site;

/* `calls` IS THE ONE THAT SAYS WHETHER THIS IS WORKING.  A patched edge never
 * comes back here, so in a machine that is linking well the call count goes
 * quiet while the frame counter keeps moving; a call count that tracks the
 * frame rate means the links are being torn down as fast as they are made.
 *
 * Nothing prints them -- they are read from a debugger, or by a printf added
 * for one run.  A per-second report was here and was noise once the mechanism
 * was known to work. */

unsigned fgl_unlink_calls[FGL_UNLINK_N];
unsigned fgl_unlink_links[FGL_UNLINK_N];

static void fgl_link_restore(struct lightrec_state *state, uint32_t site)
{
	*(volatile uint32_t *)(uintptr_t)site = FGL_LINK_SITE_WORD;
	if (state->ops.code_inv)
		state->ops.code_inv((void *)(uintptr_t)site, 4);
}

void fgl_unlink_all(struct lightrec_state *state, unsigned why)
{
	unsigned i;

	fgl_unlink_calls[why]++;
	fgl_unlink_links[why] += fgl_n_links;

	if (!fgl_n_links)
		return;

	for (i = 0; i < fgl_n_links; i++)
		fgl_link_restore(state, fgl_link_site[i]);

	fgl_link_undone += fgl_n_links;
	fgl_n_links = 0;
	memset(fgl_link_pgcnt, 0, sizeof fgl_link_pgcnt);
}

/* THE SAME, FOR ONE WINDOW OF THE TABLE -- WHICH IS ALMOST ALWAYS WHAT WAS
 * MEANT.
 *
 * `lightrec_invalidate` clears `n` slots and nothing else, and then called
 * `fgl_unlink_all`, which put back every patched site in the program.  A four
 * byte guest store threw away the whole link graph.  Measured on Spyro: 7992
 * invalidations destroyed 251516 links, 99.7% of all the teardown in the run,
 * while the other five callers together managed 688.
 *
 * A site is stale when the slot it branches at is one of the cleared ones, and
 * no other site is affected, so only that window is torn down and the rest of
 * the table is compacted down over the holes.  Order within the table means
 * nothing -- it is walked whole or not at all -- so compacting is just a
 * second index.
 *
 * FREEING CODE IS THE OTHER CASE, and it is `fgl_unlink_block` below.  This
 * one restores sites, which means writing to them, so it is only sound while
 * every site in the table is still live memory. */
void fgl_unlink_range(struct lightrec_state *state, u32 first, u32 n,
		      unsigned why)
{
	unsigned i, keep = 0, hit = 0;
	u32 last = first + n;

	fgl_unlink_calls[why]++;

	if (!n || !fgl_links_in_range(first, last))
		return;

	for (i = 0; i < fgl_n_links; i++) {
		if (fgl_link_slot[i] >= first && fgl_link_slot[i] < last) {
			fgl_link_restore(state, fgl_link_site[i]);
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		fgl_link_site[keep] = fgl_link_site[i];
		fgl_link_slot[keep] = fgl_link_slot[i];
		keep++;
	}

	fgl_n_links = keep;
	fgl_unlink_links[why] += hit;
	fgl_link_undone += hit;
}

/* ONE BLOCK'S CODE IS ABOUT TO BE HANDED BACK TO THE ARENA.
 *
 * Two different populations die with it, and they need opposite treatment:
 *
 *   - Sites POINTING AT it.  Their target slot is inside [first, first + n),
 *     they live in some other block that is still perfectly good memory, and
 *     they must be rewritten back into a call to the stub.  Same as a range
 *     unlink.
 *
 *   - Sites LIVING IN it, anywhere in [code, code + code_size).  These must be
 *     forgotten WITHOUT being touched.  Restoring one means storing into
 *     memory that is being freed this instant, and a later teardown that still
 *     had it in the table would store into whatever the arena handed out next.
 *     That is the hazard the old global sweep avoided by running before every
 *     free and taking the entire table with it.
 *
 * A site can be in both sets -- a block that branches to itself -- and the
 * living-in test wins, because not writing is always safe and writing into
 * freed memory never is.
 *
 * MUST RUN BEFORE THE FREE.  The first population is rewritten here and the
 * addresses have to still be ours. */
void fgl_unlink_block(struct lightrec_state *state, u32 first, u32 n,
		      uintptr_t code, u32 code_size, unsigned why)
{
	unsigned i, keep = 0, hit = 0;
	u32 last = first + n;

	fgl_unlink_calls[why]++;

	for (i = 0; i < fgl_n_links; i++) {
		uintptr_t site = (uintptr_t)fgl_link_site[i];

		/* Living in the dying code: drop it, do not write it. */
		if (code_size && site >= code && site < code + code_size) {
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		/* Pointing at the dying code: put it back. */
		if (fgl_link_slot[i] >= first && fgl_link_slot[i] < last) {
			fgl_link_restore(state, fgl_link_site[i]);
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		fgl_link_site[keep] = fgl_link_site[i];
		fgl_link_slot[keep] = fgl_link_slot[i];
		keep++;
	}

	fgl_n_links = keep;
	fgl_unlink_links[why] += hit;
	fgl_link_undone += hit;
}

/* Called by `fgl_link_stub` with the guest PC the block is leaving for and the
 * address of the call site that asked.  Returns the host address to enter, or
 * 0 to mean "go round the dispatcher this time" -- which is what happens while
 * the successor is not compiled yet, and is why the stub is a call and not a
 * one-shot resolver.
 *
 * The answer is only PATCHED IN when it is one a `bra` can reach.  SH-4's
 * unconditional branch has a twelve-bit word displacement, so +/-4 KB; the
 * code arena is megabytes, so out-of-range successors are ordinary and they
 * keep the call.  They are not a bug and not worth a second mechanism: the
 * call is what the edge cost before linking existed.
 *
 * IT REFUSES TO LINK TO THE COMPILE SENTINEL.  Under the threaded compiler a
 * table slot can hold `get_next_block` to mean "seen, not compiled yet"
 * (lightrec.c, `lightrec_precompile_block`).  That is a legal thing for the
 * DISPATCHER to jump to, because the dispatcher asks again next time -- but
 * baking it into a `bra` would freeze the edge on the sentinel for ever and
 * the real block would never be reached. */
static u32 fgl_link_resolve_inner(struct lightrec_state *state, u32 target,
				  u32 site);

/* Reached from emitted code through `fgl_link_stub`, so it is another door
 * out.  Wrapped rather than bracketed inline because it has several returns. */
u32 fgl_link_resolve(struct lightrec_state *state, u32 target, u32 site)
{
	u32 r;

	EMITMISS_OUT_BEGIN();
	r = fgl_link_resolve_inner(state, target, site);
	EMITMISS_OUT_END();

	return r;
}

static u32 fgl_link_resolve_inner(struct lightrec_state *state, u32 target,
				  u32 site)
{
	void *slot = lut_read(state, lut_offset(target));
	int32_t d;

	fgl_link_calls++;

	if (!slot) {
		fgl_link_uncompiled++;
		return 0;
	}

	if (slot == (void *)(uintptr_t)state->get_next_block) {
		/* THE SENTINEL IS AN ENTRY POINT, AND IT READS `curr_pc`.
		 *
		 * The stub `jmp`s straight to whatever comes back, so returning
		 * this sends the edge into `_fgl_dispatch_compile` -- which
		 * takes the guest PC out of the state block, because the
		 * dispatcher's own route there has already destroyed r2 working
		 * out the table index.  The loop publishes on its way in; a
		 * linked edge never passes through the loop, so without this
		 * the compile would run on whichever block last went the long
		 * way round and enter the wrong one, silently.
		 *
		 * Here rather than in the stub because this is the only branch
		 * that needs it: every other return is either 0, which goes
		 * round the loop, or a real block entry, which is entered with
		 * r2 and needs nothing. */
		state->curr_pc = target;
		fgl_link_uncompiled++;
		return (u32)(uintptr_t)slot;
	}

	if (fgl_n_links >= FGL_MAX_LINKS) {
		fgl_link_full++;
		return (u32)(uintptr_t)slot;
	}

	if (site & 3)                   /* the emitter's pad failed */
		return (u32)(uintptr_t)slot;

	/* THE SITE HAS TO STILL BE A SITE BEFORE ANYTHING IS WRITTEN TO IT.
	 *
	 * `site` is not passed in from anywhere trustworthy -- the stub derives
	 * it from PR, as `PR - 6`, which is only the call site if the emitter
	 * laid the site out exactly as the stub assumes.  If those two ever
	 * disagree, the two stores below write a `bra` and a `nop` into
	 * WHATEVER `site` happens to name.  The code arena and bloom's own
	 * .text are both writable, so the likely outcome is not a fault here
	 * but silently corrupted C, faulting somewhere unrelated much later --
	 * which is exactly the shape of the boot fault that prompted this.
	 *
	 * An unpatched site is two known halfwords and nothing else can be
	 * mistaken for them, so checking is one load and one compare.  Refusing
	 * costs the edge and nothing more; the dispatcher still runs.
	 *
	 * An ALREADY-patched site reaching here would also fail this test, and
	 * that is equally a bug: a site is recorded when it is patched and only
	 * an unpatched one can call the stub. */
	if (*(volatile uint32_t *)(uintptr_t)site != FGL_LINK_SITE_WORD) {
		/* Capped: if this fires at all it is likely to fire on every
		 * edge, and a flood over dc-load is slower than the fault. */
		if (fgl_link_bad_site++ < 20)
			fprintf(stderr,
				"fgl: link site %08x is not a link site: %08x\n",
				(unsigned)site,
				(unsigned)*(volatile uint32_t *)
					(uintptr_t)site);
		return (u32)(uintptr_t)slot;
	}

	/* `sh4_branch_disp12` counts from the branch's own address, which is
	 * site+0 -- the load is what the `bra` replaces, not what it follows. */
	d = sh4_branch_disp12(site, (uintptr_t)slot);
	if (sh4_disp12_fits(d)) {
		*(volatile uint32_t *)(uintptr_t)site =
			FGL_PACK(SH4_D12(0xa000, d), 0x0009u /* nop */);
		fgl_link_patched++;
	} else {
		/* FAR: the address goes in the six dead bytes the emitter left
		 * behind the site, and the site becomes a PC-relative load and
		 * an indirect jump.  `mov.l @(disp,PC),R0` reads
		 * `(PC & ~3) + disp * 4` with PC = site+4, and the site is
		 * four-aligned, so disp 1 names site+8.
		 *
		 * THE LITERAL IS WRITTEN FIRST.  Until the instruction pair
		 * below lands, site+8 is data nothing reads; after it lands it
		 * is the jump's target.  The other order gives the fetcher a
		 * jump through whatever the emitter left there. */
		*(volatile uint32_t *)(uintptr_t)(site + 8) =
			(uint32_t)(uintptr_t)slot;
		if (state->ops.code_inv)
			state->ops.code_inv((void *)(uintptr_t)(site + 8), 4);

		*(volatile uint32_t *)(uintptr_t)site =
			FGL_PACK(0xd001u /* mov.l @(1,pc),r0 */,
				 0x402bu /* jmp @r0 */);
		fgl_link_range++;
	}

	if (state->ops.code_inv)
		state->ops.code_inv((void *)(uintptr_t)site, 4);

	fgl_link_slot[fgl_n_links] = lut_offset(target);
	fgl_link_site[fgl_n_links++] = site;
	fgl_link_pgcnt[lut_offset(target) >> FGL_LINK_PAGE_SHIFT]++;

	return (u32)(uintptr_t)slot;
}
