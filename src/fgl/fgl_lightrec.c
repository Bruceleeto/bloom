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
#include "fgl_backend.h"

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

/* THE BACKEND BLOCK.  lightrec gives a backend an opaque `backend[]` array at
 * a fixed offset and keeps its own fields behind it, so every displacement
 * below is `offsetof(backend) + offsetof(within fgl_words)` -- which is the
 * whole reason the port is a backend and not a fork.  BE_AT() says that once.
 */
#define BE_AT(field) (offsetof(struct lightrec_state, backend) \
		      + offsetof(struct fgl_words, field))

/* The cycle table, whose displacement is an instruction count. */
FGL_ASSERT(FGL_AT_CYCLES * 4 == BE_AT(cycle_table), cycle_table);
FGL_ASSERT(sizeof(((struct fgl_words *)0)->cycle_table)
	   == FGL_CYCLE_ENTRIES * 4, cycle_entries);

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
	   == BE_AT(dispatch), dispatch);
FGL_ASSERT(FGL_AT_LUT * 4
	   == BE_AT(lut_base), lut_base);
FGL_ASSERT(FGL_AT_METER_BASE * 4 == BE_AT(meter_base), meter_base);
FGL_ASSERT(FGL_AT_METER_IN * 4 == BE_AT(meter_in), meter_in);
FGL_ASSERT(FGL_AT_INV_MASK * 4 == BE_AT(inv_mask), inv_mask);
FGL_ASSERT(FGL_AT_INV_RAM * 4 == BE_AT(inv_ram), inv_ram);
FGL_ASSERT(FGL_AT_INV_DIRECT * 4 == BE_AT(inv_direct), inv_direct);
FGL_ASSERT(FGL_AT_PIN_PUB * 4 == BE_AT(pin_pub), pin_pub);
FGL_ASSERT(FGL_AT_PIN_REL * 4 == BE_AT(pin_rel), pin_rel);
FGL_ASSERT(FGL_AT_RW_TRAMP * 4 == BE_AT(rw_tramp), rw_tramp);
FGL_ASSERT(FGL_AT_RW_BLOCK * 4 == BE_AT(rw_block), rw_block);
FGL_ASSERT(FGL_AT_SVC * 4 == BE_AT(svc), svc);
FGL_ASSERT(FGL_AT_INV_MASK * 4 == BE_AT(inv_mask), inv_mask);
FGL_ASSERT(FGL_AT_INV_RAM * 4 == BE_AT(inv_ram), inv_ram);
FGL_ASSERT(FGL_AT_INV_DIRECT * 4 == BE_AT(inv_direct), inv_direct);
FGL_ASSERT(FGL_AT_PIN_PUB * 4 == BE_AT(pin_pub), pin_pub);
FGL_ASSERT(FGL_AT_PIN_REL * 4 == BE_AT(pin_rel), pin_rel);
FGL_ASSERT(FGL_AT_RW_TRAMP * 4 == BE_AT(rw_tramp), rw_tramp);
FGL_ASSERT(FGL_AT_RW_BLOCK * 4 == BE_AT(rw_block), rw_block);
FGL_ASSERT(FGL_AT_SVC * 4 == BE_AT(svc), svc);
FGL_ASSERT(FGL_AT_EXIT_METER * 4 == BE_AT(exit_meter),
	   exit_meter);
FGL_ASSERT(FGL_AT_ADDR_MASK * 4
	   == BE_AT(addr_mask), addr_mask);
FGL_ASSERT(FGL_AT_SHIM_ARG * 4
	   == BE_AT(shim_arg), shim_arg);
FGL_ASSERT(FGL_AT_LINK * 4
	   == BE_AT(link), link);
FGL_ASSERT(FGL_AT_TSAVE * 4
	   == BE_AT(tsave), tsave);

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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "fgl.h"
#include "fgl_cache.h"
#include "front.h"
#include "shim.h"

#include "blockcache.h"
/* WHAT THE HOST BUILD SUPPLIES INSTEAD OF THE DREAMCAST'S SYMBOLS.
 *
 * On the target the service table is filled with link-time addresses: the
 * shims out of shim.S, the accessors out of lightrec.c, one float body per
 * COP2 command out of gte_fpu.c.  None of that survives here -- emitted code
 * is executed by an interpreter and a C function is not SH-4 -- so fgl_run.c
 * fills the table with tokens it recognises at the call instead. */
uint32_t fgl_run_gte_body(void *user, uint32_t op);
void fgl_run_fill_targets(fgl_targets *t);

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
 * block, so the block is found by walking back from it -- a region can be
 * up to FGL_MAX_BLOCK_OPS words long.  The index is the list slot at that
 * address, checked against the word: a swapped delay-slot pair puts the
 * branch and its slot in each other's slots, so the neighbours are tried. */
static unsigned long rw_walk_k;

/* THE BLOCK-START CACHE, AND WHY THE WALK BELOW NEEDS ONE.
 *
 * The walk is O(the access's index within its block), because that is what it
 * is measuring: it is handed the access's exact guest pc and finds the block
 * by trying every address behind it until one names a block start.  Measured
 * on the x86 tree over 120M guest cycles of the Spyro savestate
 * (FGL_RW_STATS=1), `avg k` is 31.2 for RAM and 61.6 for the scratchpad
 * across 58,596 crossings -- about 2.4 MILLION `lightrec_find_block` probes,
 * each a hash bucket plus a walk down a `block->next` chain.
 *
 * On x86 that is 182ns and invisible.  On the Dreamcast it was 10.2% of the
 * whole machine -- the hottest single function in the profile, more than the
 * entire GTE -- because it is pointer-chasing through structures spread over
 * megabytes against a 16 KiB D-cache.
 *
 * So remember the answer.  A site that crosses generically once crosses
 * thousands of times: the same `curr_pc` recurs, and with it the same block
 * start and the same index.
 *
 * WHAT IS CACHED IS THE BLOCK'S START PC, NOT THE BLOCK POINTER, and that is
 * the whole safety argument.  Blocks are freed and rebuilt under us --
 * `lightrec_early_unload` is 1.6% of the profile by itself -- so a cached
 * `struct block *` would dangle.  A start pc is just a number; it is re-looked
 * up every time, one probe instead of thirty, and the entry is then CHECKED
 * against the opcode it claims to name before being believed.  A miss, a
 * stale entry and a collision all fail that check and fall through to the
 * walk, which is still correct on its own.
 *
 * Direct-mapped and unsynchronised on purpose.  A torn or stale entry cannot
 * produce a wrong answer, only a wasted probe, so there is nothing here worth
 * a lock. */
#define RW_CACHE_BITS 9
#define RW_CACHE_N    (1u << RW_CACHE_BITS)

static struct {
	u32 pc;                 /* the access's guest pc, the key           */
	u32 block_pc;           /* the block start it was found to be in    */
	u16 offset;             /* its index in that block's opcode list    */
	u8  valid;
} rw_cache[RW_CACHE_N];

/* Is this block usable, and does slot `idx` in it really hold `opcode`? */
static struct opcode *fgl_rw_at(struct block *block, u32 opcode, int idx)
{
	if (!block || !block->opcode_list ||
	    block_has_flag(block, BLOCK_NO_OPCODE_LIST))
		return NULL;
	if (idx < 0 || idx >= (int)block->nb_ops)
		return NULL;
	if (block->opcode_list[idx].c.opcode != opcode)
		return NULL;

	return &block->opcode_list[idx];
}

static struct opcode *fgl_rw_op(struct lightrec_state *state, u32 opcode,
				struct block **out, u16 *offset)
{
	u32 pc = state->curr_pc, k;
	unsigned slot = (pc >> 2) & (RW_CACHE_N - 1u);
	struct opcode *lop;
	u32 owner = fgl_state_words(state)->rw_block;

	/* THE SITE NAMES ITS OWNER (fgl_state.h, FGL_AT_RW_BLOCK), so the op
	 * is `(pc - block->pc) / 4` into that list and nothing else.  A walk
	 * back from the pc is wrong whenever blocks overlap: it tags the
	 * nearest list, that block recompiles to a direct access, and the
	 * block whose code is actually running -- compiled from another list
	 * covering the same pc -- keeps its generic site for ever.  Spyro:
	 * 212 such crossings a frame into plain RAM and scratchpad.
	 *
	 * An owner that is gone, or whose list was freed, gets no tag: the
	 * code that runs was compiled from that list and no other. */
	if (owner) {
		struct block *block = lightrec_find_block(state->block_cache,
							  owner);

		if (!block || pc < block->pc)
			return NULL;
		k = (pc - block->pc) / 4u;
		lop = fgl_rw_at(block, opcode, (int)k);
		if (!lop)
			return NULL;
		*out = block;
		*offset = (u16)k;
		rw_walk_k = 0;
		return lop;
	}

	if (rw_cache[slot].valid && rw_cache[slot].pc == pc) {
		struct block *block = lightrec_find_block(state->block_cache,
							  rw_cache[slot].block_pc);

		lop = fgl_rw_at(block, opcode, rw_cache[slot].offset);
		if (lop) {
			*out = block;
			*offset = rw_cache[slot].offset;
			rw_walk_k = 0;
			return lop;
		}

		/* The block was unloaded, or rebuilt to a different shape, or
		 * another pc owns this slot now.  Drop it and walk. */
		rw_cache[slot].valid = 0;
	}

	/* A lightrec block can be far longer than what fgl lowers of it:
	 * `nb_ops` runs to the first unconditional transfer.  Blocks also
	 * start at every address something once jumped to, so a block found
	 * at a small distance may end before the access: keep walking. */
	for (k = 0; k < 4096; k++) {
		struct block *block =
			lightrec_find_block(state->block_cache, pc - 4u * k);
		int i, d[3] = { 0, 1, -1 };

		if (!block || k >= block->nb_ops)
			continue;
		for (i = 0; i < 3; i++) {
			int idx = (int)k + d[i];

			lop = fgl_rw_at(block, opcode, idx);
			if (!lop)
				continue;

			*out = block;
			*offset = (u16)idx;
			rw_walk_k = k;

			rw_cache[slot].pc = pc;
			rw_cache[slot].block_pc = block->pc;
			rw_cache[slot].offset = (u16)idx;
			rw_cache[slot].valid = 1;
			return lop;
		}
	}
	return NULL;
}

/* FGL_RW_STATS=1: the generic crossings by device, with the block walk and
 * the access timed separately (Linux ns; ratios port, absolutes do not). */
#include <time.h>
static struct { unsigned long n, k, t_walk, t_rw; } rw_st[16];
static int rw_st_on;
int fgl_hw_stats_on;
static const char *const rw_cls[16] = {
	"ram", "scratch", "bios", "memctl", "sio", "irq", "dma", "rcnt",
	"cdrom", "gpustat", "gp0", "mdec", "spu", "exp", "other", "nomap" };
static int rw_class(u32 a)
{
	a &= 0x1fffffff;
	if (a < 0x800000) return 0;
	if ((a & 0xfffffc00) == 0x1f800000) return 1;
	if (a >= 0x1fc00000 && a < 0x1fc80000) return 2;
	if (a < 0x1f801000 || a >= 0x1f802000) return a < 0x1f800000 ? 13 : 14;
	if (a < 0x1f801040) return 3;
	if (a < 0x1f801070) return 4;
	if (a < 0x1f801080) return 5;
	if (a < 0x1f801100) return 6;
	if (a < 0x1f801130) return 7;
	if (a >= 0x1f801800 && a < 0x1f801804) return 8;
	if (a == 0x1f801814) return 9;
	if (a == 0x1f801810) return 10;
	if (a >= 0x1f801820 && a < 0x1f801828) return 11;
	if (a >= 0x1f801c00 && a < 0x1f802000) return 12;
	return 14;
}
static unsigned long rw_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (unsigned long)ts.tv_sec * 1000000000ul + (unsigned long)ts.tv_nsec;
}
/* The direct hardware path (lightrec_hw_lw and friends), same table, loads and
 * stores separately. */
static struct { unsigned long n, t; } hw_st[2][16];
void fgl_hw_stat(u32 addr, int store, uint64_t ns)
{
	int c = rw_class(addr);
	hw_st[store][c].n++;
	hw_st[store][c].t += ns;
}
void fgl_hw_stats_print(void)
{
	int i, d;
	unsigned long tn = 0, tt = 0;

	if (!fgl_hw_stats_on)
		return;
	for (d = 0; d < 2; d++)
		for (i = 0; i < 16; i++) { tn += hw_st[d][i].n; tt += hw_st[d][i].t; }
	fprintf(stderr, "\nfgl_hw direct: %lu, %.1f ms\n%-8s %-5s %10s %8s %8s\n",
		tn, tt / 1e6, "device", "dir", "n", "avg ns", "ms");
	for (d = 0; d < 2; d++)
		for (i = 0; i < 16; i++)
			if (hw_st[d][i].n)
				fprintf(stderr, "%-8s %-5s %10lu %8lu %8.2f\n", rw_cls[i],
					d ? "store" : "load", hw_st[d][i].n,
					hw_st[d][i].t / hw_st[d][i].n, hw_st[d][i].t / 1e6);
}
void fgl_rw_stats_print(void)
{
	int i;

	if (!rw_st_on)
		return;
	unsigned long tn = 0, tw = 0, tr = 0;

	for (i = 0; i < 16; i++) { tn += rw_st[i].n; tw += rw_st[i].t_walk; tr += rw_st[i].t_rw; }
	fprintf(stderr, "\nfgl_rw crossings: %lu, walk %.1f ms, access %.1f ms\n"
		"%-8s %10s %6s %8s %8s %8s\n", tn, tw / 1e6, tr / 1e6,
		"device", "n", "avg k", "walk ns", "rw ns", "ms total");
	for (i = 0; i < 16; i++)
		if (rw_st[i].n)
			fprintf(stderr, "%-8s %10lu %6.1f %8lu %8lu %8.2f\n", rw_cls[i],
				rw_st[i].n, (double)rw_st[i].k / rw_st[i].n,
				rw_st[i].t_walk / rw_st[i].n, rw_st[i].t_rw / rw_st[i].n,
				(rw_st[i].t_walk + rw_st[i].t_rw) / 1e6);
}

void fgl_rw(u32 opcode, struct lightrec_state *state)
{
	union code op = { .opcode = opcode };
	struct block *block = NULL;
	struct opcode *lop;
	u16 offset = 0;
	u32 ret;
	static int st = -1;
	unsigned long t0 = 0, t1 = 0;
	int cls = 0;

	if (st < 0) {
		st = rw_st_on = getenv("FGL_RW_STATS") != NULL;
		fgl_hw_stats_on = st;
	}
	if (st) {
		cls = rw_class(state->regs.gpr[op.i.rs] + (s16)op.i.imm);
		t0 = rw_ns();
	}

	/* TAG THE ACCESS, AS LIGHTREC'S GENERIC WRAPPER DOES.  An access the
	 * optimiser could not place is performed by C once, `lightrec_rw`
	 * records where it went in the op's flags and marks the block for
	 * recompilation, and the recompiled block lowers it for real.  With
	 * NULL flags none of that happened: every untagged access stayed on
	 * this path for ever, and a region -- compiled from a list nothing has
	 * run yet -- is made of untagged accesses.  Spyro's pad routine made
	 * 68k generic calls per 40 vsyncs against the old build's zero. */
	lop = fgl_rw_op(state, opcode, &block, &offset);
	if (st)
		t1 = rw_ns();
	ret = lightrec_rw(state, op, state->regs.gpr[op.i.rs],
			  state->regs.gpr[op.i.rt], lop ? &lop->flags : NULL,
			  block, offset);
	if (st) {
		rw_st[cls].n++;
		rw_st[cls].k += rw_walk_k;
		rw_st[cls].t_walk += t1 - t0;
		rw_st[cls].t_rw += rw_ns() - t1;
	}

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

/* The last 64 block entries, oldest first. */
void fgl_dump_ring(void)
{
	unsigned idx = fgl_pc_ring[0] & 63u;
	unsigned k;

	for (k = 0; k < 64; k++) {
		unsigned e = (idx + 1u + k) & 63u;

		if (fgl_pc_ring[1 + 4 * e])
			fprintf(stderr, "fgl: came from %08x  $v1=%08x "
				"$v0=%08x $t8=%08x\n",
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
		fprintf(stderr, "fgl: guest %08x  %08x  list %08x flags %08x\n",
			(unsigned)(b->pc + 4u * k), w[k],
			(unsigned)b->opcode_list[k].opcode,
			(unsigned)b->opcode_list[k].flags);

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
	/* KOS's `arch_abort` on the target; here the process is worth keeping
	 * alive -- the SH-4 interpreter has already stopped the run and named
	 * the block, and everything after this point is readable under gdb. */
	abort();
}

/* The KOS fault reporter is not ported: under the SH-4 interpreter a wild
 * address is `sh4_fault_at`, which stops the run, names the address and the
 * guest block, and leaves the process alive under gdb.  `fgl_dump_ring` and
 * `fgl_dump_guest` stay because lightrec's lockstep calls them. */

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

	fgl_run_fill_targets(&fgl_dc_targets);
}

/* The same addresses again, where generated code can reach them in one
 * instruction and no literal pool word.  See FGL_AT_SVC. */
static void fgl_svc_once(struct lightrec_state *state)
{
	struct fgl_words *w = fgl_state_words(state);
	const fgl_targets *t = &fgl_dc_targets;
	unsigned i;

	if (w->svc[FGL_SVC_SHIM_CALL])
		return;

	w->svc[FGL_SVC_SHIM_CALL]    = t->shim_call;
	w->svc[FGL_SVC_SHIM_CALL_ST] = t->shim_call_st;
	w->svc[FGL_SVC_SHIM_GTE]     = t->shim_gte;
	w->svc[FGL_SVC_RW]           = t->rw;
	w->svc[FGL_SVC_MTC]          = t->mtc;
	w->svc[FGL_SVC_MFC]          = t->mfc;
	w->svc[FGL_SVC_RFE]          = t->rfe;
	for (i = 0; i < 5; i++) {
		w->svc[FGL_SVC_HW_LOAD + i]  = t->hw_load[i];
		w->svc[FGL_SVC_HW_STORE + i] = t->hw_store[i];
	}
	for (i = 0; i < FGL_GTE_LEAF_N; i++)
		w->svc[FGL_SVC_GTE_LEAF + i] = t->gte_leaf_tab[i];
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
void fgl_static_census(const void *code, unsigned size, unsigned n_ops);

/* FGL_STATS: how blocks were cut. */
unsigned long fgl_region_blocks, fgl_region_ranges, fgl_region_fallbacks,
	      fgl_rpin_regions;

/* ONE lightrec BLOCK, AS SEVERAL BASIC BLOCKS IN ONE CODE BLOCK.
 *
 * `fgl_front` lowers up to the first control transfer.  Called once, that
 * compiled the first basic block of a lightrec block and left every internal
 * branch to exit -- measured against GNU lightning on the same guest loop at
 * 800269cc (FGL.md 2026-09-09): 22 fgl blocks and ~14 dispatcher or link
 * transitions per trip where lightning has one block and one.
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
 * emit_local_edge.  FGL_NO_REGION=1 restores one basic block per block. */
#define FGL_MAX_RANGES 24

struct fgl_range {
	ir_node   ir[IR_MAX_NODES];
	ir_alloc  alloc;
	int       n;
	unsigned  n_ops;
	uint32_t  pc;
	unsigned  stop;         /* FGL_STOP_*, for FGL_DUMP_PC */
};

static int fgl_region_off(void)
{
	static int off = -1;

	if (off < 0)
		off = FGL_ENTRY_HOOK || getenv("FGL_NO_REGION") != NULL;
	return off;
}

/* Cut the opcode list into ranges.  Returns the count, or 0 with `info`
 * describing the first range's refusal. */
static int fgl_cut_ranges(const struct block *block, struct fgl_range *r,
			  int max, int whole, fgl_front_info *info)
{
	static uint8_t lead[4096];
	const struct opcode *ops = block->opcode_list;
	unsigned nb = block->nb_ops, i, k;
	int nr = 0;

	if (nb > sizeof lead)
		whole = 0;

	/* whole: 1 = leaders + fall-through, 2 = fall-through only (no
	 * leaders), 3 = leaders only (stop at the first non-local transfer). */
	if (whole == 1 || whole == 3) {
		memset(lead, 0, nb);
		for (i = 0; i < nb; i++) {
			uint32_t at, tgt;

			/* BIT(3) IS LOCAL_BRANCH ON A BRANCH ONLY; on a store it
			 * is something else (front.c on the flags union).  The
			 * same opcode test as lightrec's `is_local_branch`. */
			switch (ops[i].c.i.op) {
			case OP_BEQ: case OP_BNE: case OP_BLEZ: case OP_BGTZ:
			case OP_REGIMM:
				break;
			default:
				continue;
			}
			if (!op_flag_local_branch(ops[i].flags))
				continue;
			/* The swapped pair moves the branch's index, not its
			 * address (front.c, `branch_at`). */
			at = block->pc + 4u * i -
			     4u * (uint32_t)!!op_flag_no_ds(ops[i].flags);
			tgt = at + 4u + ((uint32_t)(int32_t)(int16_t)ops[i].c.i.imm << 2);
			if (tgt >= block->pc && (tgt - block->pc) / 4u < nb)
				lead[(tgt - block->pc) / 4u] = 1;
		}
	}

	for (k = 0; k < nb && nr < max; nr++) {
		unsigned lim = nb;
		struct fgl_range *q = &r[nr];

		if (whole == 1 || whole == 3)
			for (i = k + 1; i < nb; i++)
				if (lead[i]) { lim = i; break; }
		else if (whole == 2)
			memset(lead, 0, nb);

		memset(info, 0, sizeof *info);
		q->pc = block->pc + 4u * k;
		q->n = fgl_front(&ops[k], lim - k, q->pc, q->ir, IR_MAX_NODES,
				 info);
		if (q->n <= 0 || info->unsupported)
			break;
		q->n_ops = info->n_ops;
		q->stop = info->stop_reason;
		k += info->n_ops;
		if (!whole || !info->n_ops) {   /* one block, or C resumes here */
			nr++;
			break;
		}
		if (whole == 3 && q->stop == FGL_STOP_TRANSFER &&
		    !(k < nb && lead[k])) {
			nr++;
			break;
		}
	}
	return nr;
}

/* One emission pass of `nr` ranges at `base`.  Returns the size, 0 on
 * refusal. */
static unsigned fgl_emit_ranges(fgl_emitter *e, void *buf, unsigned bufsize,
				struct fgl_range *r, int nr, uint32_t *label_pcs,
				int region, uint8_t *tags,
				uint8_t *hint_out, const uint8_t *hint_in,
				u32 block_pc)
{
	int i;

	fgl_init(e, buf, bufsize, (u32)(uintptr_t)buf);
	e->cg.tagp = tags;
	e->block_pc = block_pc;
	e->hint_out = hint_out;
	e->hint_in = hint_in;
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
	fgl_region_hints(e);
	return fgl_size(e);
}

/* THE INVALIDATION STUBS GO INTO THE ARENA ONCE, BEFORE ANY BLOCK.
 *
 * They are ordinary emitted code -- the SH-4 interpreter executes the arena
 * here and the hardware executes it on the target -- so this needs nothing
 * assembled or linked, and the same two routines serve every block for the
 * life of the process.  They are allocated and never freed; the arena's
 * allocator does not move what it has handed out, and code invalidation frees
 * blocks rather than arbitrary allocations.
 *
 * A failure here is not fatal.  `fgl_inv_stubs_ready` stays clear, and
 * emit_invalidate goes on inlining the sequence exactly as it did before.
 */
static void fgl_inv_stubs_init(struct lightrec_state *state)
{
	struct fgl_words *w = fgl_state_words(state);
	unsigned size, ram = 0, direct = 0, pub = 0, rel = 0, rwt = 0;
	uint8_t tmp[512];
	void *code;

	if (w->inv_ram)
		return;

	size = fgl_emit_inv_stubs(tmp, sizeof tmp, &ram, &direct, &pub, &rel,
				  &rwt);
	if (!size)
		return;

	code = lightrec_alloc_code(state, size);
	if (!code)
		return;
	memcpy(code, tmp, size);
	if (state->ops.code_inv)
		state->ops.code_inv(code, size);

	{
#ifdef __sh__
		u32 base = (u32)(uintptr_t)code;
#else
		/* Zero means the SH-4 runtime cannot name this address yet;
		 * leave the slots clear and try again on the next block. */
		u32 base = fgl_run_addr_of(code);

		if (!base)
			return;
#endif
		w->inv_ram    = base + ram;
		w->inv_direct = base + direct;
		w->pin_pub    = base + pub;
		w->pin_rel    = base + rel;
		w->rw_tramp   = base + rwt;
	}
	fgl_inv_stubs_ready = 1;
	fgl_pin_stubs_ready = 1;
	fgl_rw_tramp_ready = 1;
}

void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why)
{
	struct lightrec_state *state = cstate->state;
	static uint8_t scratch[FGL_SCRATCH_BYTES];
	static struct fgl_range ranges[FGL_MAX_RANGES];
	static uint32_t label_pcs[FGL_MAX_RANGES];
	/* Two failures that look the same from the outside and must not be
	 * confused: the arena being full is transient and the caller may flush
	 * and retry, while fgl declining to lower a block is permanent and a
	 * retry is an infinite loop. */
	int dummy_why;
	fgl_front_info info;
	fgl_emitter e;
	static uint8_t hints[FGL_MAX_LABELS];
	unsigned size, size2, total_ops;
	void *code;
	int nr, i, whole = !fgl_region_off();

	if (!why)
		why = &dummy_why;
	*why = -EINVAL;                 /* a hole in fgl until proven otherwise */


	fgl_targets_once();
	fgl_svc_once(state);
	fgl_inv_stubs_init(state);
	fgl_crash_state = state;

	/* FGL_REGION_LO/HI=<hex>: regions only for blocks in [lo, hi).  A
	 * bisection knob. */
	{
		static u32 lo, hi = 0xffffffffu;
		static int tried;

		if (!tried) {
			const char *v;

			tried = 1;
			if ((v = getenv("FGL_REGION_LO")))
				lo = (u32)strtoul(v, NULL, 16);
			if ((v = getenv("FGL_REGION_HI")))
				hi = (u32)strtoul(v, NULL, 16);
		}
		if ((u32)block->pc < lo || (u32)block->pc >= hi)
			whole = 0;
		/* FGL_REGION_DBG=8: no regions for blocks that touch a device. */
		{
			const char *d = getenv("FGL_REGION_DBG");
			extern bool fgl_block_touches_device(const struct block *);

			if (d && (strtoul(d, NULL, 0) & 8) &&
			    fgl_block_touches_device(block))
				whole = 0;
			/* 64/128/256: no region for blocks with HW io /
			 * unknown io / any store. */
			{
				extern unsigned fgl_block_access_classes(const struct block *);
				unsigned long m = d ? strtoul(d, NULL, 0) : 0;
				unsigned c = (m & (64 | 128 | 256)) ?
					fgl_block_access_classes(block) : 0;

				if (((m & 64) && (c & 1)) || ((m & 128) && (c & 2)) ||
				    ((m & 256) && (c & 4)))
					whole = 0;
			}
			/* 16: fall-through only, 32: leaders only */
			if (d && whole && (strtoul(d, NULL, 0) & 16))
				whole = 2;
			if (d && whole && (strtoul(d, NULL, 0) & 32))
				whole = 3;
		}
	}

	{
		static int maxr = -1;
		if (maxr < 0) {
			const char *v = getenv("FGL_REGION_MAXR");
			maxr = v ? atoi(v) : FGL_MAX_RANGES;
			if (maxr < 1 || maxr > FGL_MAX_RANGES) maxr = FGL_MAX_RANGES;
		}
		nr = fgl_cut_ranges(block, ranges, maxr, whole, &info);
	}

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

	/* FGL_DUMP_PC=<hex>: the guest instructions and lightrec flags of one
	 * block, once, so a miscompile can be read off the ops rather than
	 * reconstructed from the registers it wrecked. */
	{
		static u32 want = 1;

		if (want == 1) {
			const char *v = getenv("FGL_DUMP_PC");

			want = v ? (u32)strtoul(v, NULL, 16) : 0;
		}
		if (want && (u32)block->pc == want) {
			fprintf(stderr, "fgl: block %08x nb_ops=%u ranges=%d:",
				(unsigned)block->pc, block->nb_ops, nr);
			for (i = 0; i < nr; i++)
				fprintf(stderr, " %08x/%u/%d/s%u", ranges[i].pc,
					ranges[i].n_ops, ranges[i].n, ranges[i].stop);
			fprintf(stderr, "\n");
			fgl_dump_guest(block, 128);
		}
	}

	/* REGION PINS (POC): the region's most-mentioned guest registers,
	 * not already pinned, take the free slots for the region's length.
	 * OFF BY DEFAULT: measured a loss at every count (PLAN.md 2026-09-10),
	 * because every slot it takes is one fewer rotating register and the
	 * fixed pins get evicted instead. FGL_RPIN_MAX=n turns it on. */
	{
		static int rpin_max = -1;
		int8_t extra[ALLOC_N];
		int use = 0;

		if (rpin_max < 0) {
			const char *v = getenv("FGL_RPIN_MAX");
			rpin_max = v ? atoi(v) : 0;
		}
		for (i = 0; i < ALLOC_N; i++)
			extra[i] = -1;
		if (nr > 1 && rpin_max > 0) {
			unsigned cnt[32];
			int k, h, taken = 0;

			memset(cnt, 0, sizeof cnt);
			for (k = 0; k < nr; k++)
				for (i = 0; i < ranges[k].n; i++) {
					const ir_node *p = &ranges[k].ir[i];
					if (p->rs < 32) cnt[p->rs]++;
					if (p->rt < 32) cnt[p->rt]++;
					if (p->rd < 32) cnt[p->rd]++;
				}
			cnt[0] = 0;
			for (h = 0; h < ALLOC_N; h++)
				if (ir_pin[h] > 0 && ir_pin[h] < 32)
					cnt[ir_pin[h]] = 0;
			for (h = 0; h < ALLOC_N && taken < rpin_max; h++) {
				int best = 0;

				if (ir_pin[h] >= 0)
					continue;
				for (k = 1; k < 32; k++)
					if (cnt[k] > cnt[best])
						best = k;
				if (cnt[best] < 2)
					break;
				extra[h] = (int8_t)best;
				cnt[best] = 0;
				taken++;
				use = 1;
			}
		}
		ir_pin_set_region(use ? extra : NULL);
		fgl_rpin_regions += use;
		if (use && getenv("FGL_RPIN_DBG"))
			fprintf(stderr, "fgl: rpin block %08x ranges %d pins r%d=$%d r%d=$%d r%d=$%d\n",
				(unsigned)block->pc, nr,
				ALLOC_FIRST + 0, extra[0], ALLOC_FIRST + 1, extra[1],
				ALLOC_FIRST + 2, extra[2]);
	}
	for (i = 0; i < nr; i++) {
		ir_allocate_region(ranges[i].ir, ranges[i].n, &ranges[i].alloc,
				   i == 0);
		label_pcs[i] = ranges[i].pc;
		if (getenv("FGL_RPIN_DBG") && atoi(getenv("FGL_RPIN_DBG")) > 1 &&
		    ir_pin_region[0] + ir_pin_region[1] + ir_pin_region[2]) {
			int k;
			fprintf(stderr, "  range %d pc %08x n=%d pre:", i,
				ranges[i].pc, ranges[i].n);
			for (k = 0; k < ranges[i].alloc.n_preload; k++)
				fprintf(stderr, " r%d<-$%d",
					ranges[i].alloc.preload[k].host,
					ranges[i].alloc.preload[k].guest);
			fprintf(stderr, " fix:");
			for (k = 0; k < ranges[i].alloc.n_fix; k++)
				fprintf(stderr, " @%d:r%d%s$%d",
					ranges[i].alloc.fix[k].at,
					ranges[i].alloc.fix[k].host,
					ranges[i].alloc.fix[k].store ? "->" : "<-",
					ranges[i].alloc.fix[k].guest);
			fprintf(stderr, "\n");
		}
	}
	ir_pin_set_region(NULL);

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
	memset(hints, 0, sizeof hints);
	size = fgl_emit_ranges(&e, scratch, sizeof scratch, ranges, nr,
			       label_pcs, whole, NULL, hints, NULL, (u32)block->pc);
	if (!size && nr > 1) {
		/* The region did not fit (a `bra` out of reach, the buffer, a
		 * label never reached).  The old shape still works: the first
		 * basic block alone, the rest compiled as they are reached. */
		fgl_region_fallbacks++;
		nr = fgl_cut_ranges(block, ranges, 1, 0, &info);
		if (nr != 1)
			return NULL;
		ir_pin_set_region(NULL);
		ir_allocate(ranges[0].ir, ranges[0].n, &ranges[0].alloc);
		label_pcs[0] = ranges[0].pc;
		whole = 0;
		memset(hints, 0, sizeof hints);
		size = fgl_emit_ranges(&e, scratch, sizeof scratch, ranges, 1,
				       label_pcs, 0, NULL, hints, NULL, (u32)block->pc);
	}
	if (!size) {
		fprintf(stderr, "fgl: FIRST PASS REFUSED pc=%08x ranges=%d "
			"ovf=%d unsup=%d op=%u\n", (unsigned)block->pc, nr,
			e.overflow, e.unsupported, e.unsupported_op);
		return NULL;
	}
	fgl_region_blocks++;
	fgl_region_ranges += nr;

	code = lightrec_alloc_code(state, size);
	if (!code) {
		*why = -ENOMEM;
		return NULL;
	}

	total_ops = 0;
	for (i = 0; i < nr; i++)
		total_ops += ranges[i].n_ops;

	/* Second pass, at the real address. The allocation pass is not run
	 * again: its result depends on the IR, not on where the code lands.
	 * Only the real pass is tagged: the measuring pass writes to a scratch
	 * buffer that is not part of the arena.  With the first pass's hints
	 * it may come out SMALLER (emit.c, "TWO SHAPES THE FIRST PASS
	 * DECIDES"); the slack stays allocated, the block is the real size. */
	size2 = fgl_emit_ranges(&e, code, size, ranges, nr, label_pcs, whole,
				fgl_tags_at(code), NULL, hints, (u32)block->pc);
	if (!size2 || size2 > size) {
		fprintf(stderr, "fgl: SECOND PASS DIFFERS pc=%08x %u vs %u "
			"ovf=%d unsup=%d\n", (unsigned)block->pc,
			fgl_size(&e), size, e.overflow, e.unsupported);
		lightrec_free_code(state, code);
		return NULL;
	}
	size = size2;

	/* The profiler's ops map: guest instructions per basic block, at
	 * the word each one is entered at. */
	for (i = 0; i < nr; i++) {
		int at = e.region ? fgl_region_label_at(&e, i) : 0;

		if (at >= 0)
			fgl_ops_at((uint8_t *)code + 2 * at, ranges[i].n_ops);
	}
	fgl_cache_note_block(code, size, (u32)block->pc);
	/* FGL_BLOCKMAP=<path>: guest pc -> guest instruction count, one line
	 * per compiled block.  The denominator for anything measured on a
	 * DIFFERENT emulator running the same game: bleem's block header holds
	 * the MIPS pc it was compiled from, so a bleem block can be weighted by
	 * the guest instruction count of the block fgl compiled from the same
	 * address. */
	{
		static FILE *bm;
		static int bm_tried;
		const char *bmp;

		if (!bm_tried) {
			bm_tried = 1;
			bmp = getenv("FGL_BLOCKMAP");
			if (bmp)
				bm = fopen(bmp, "w");
		}
		if (bm)
			fprintf(bm, "%08x %u %u %d\n", (unsigned)block->pc,
				total_ops, size, nr);
	}

	fgl_static_census(code, size, total_ops);

	/* FGL_DUMP_CODE=<hex>[,<path>]: the emitted bytes of one block, raw,
	 * for `sh-elf-objdump -D -b binary -m sh4 -EL`. */
	{
		static const char *dc;
		static int dc_tried;

		if (!dc_tried) {
			dc_tried = 1;
			dc = getenv("FGL_DUMP_CODE");
		}
		if (dc && (u32)block->pc == (u32)strtoul(dc, NULL, 16)) {
			const char *path = strchr(dc, ',');
			FILE *f = fopen(path ? path + 1 : "/tmp/fgl_block.bin", "wb");

			if (f) {
				fwrite(code, 1, size, f);
				fclose(f);
				fprintf(stderr, "fgl: dumped block %08x %u bytes %u ops\n",
					(unsigned)block->pc, size, total_ops);
			}
		}
	}

	if (state->ops.code_inv)
		state->ops.code_inv(code, size);

	*code_size = size;
	return code;
}
