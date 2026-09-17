/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * fgl as a lightrec backend: the vtable in lightrec-backend.h and the words
 * fgl keeps in the state block.  The block cache, optimiser, arena, LUT,
 * reaper and first-pass interpreter are lightrec's, shared unchanged.
 */

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "lightrec-private.h"
#include "blockcache.h"
#include "interpreter.h"
#include "memmanager.h"
#include "fgl_backend.h"

/* ----------------------------------------------------------------------
 * The backend's words in struct lightrec_state
 * ---------------------------------------------------------------------- */

_Static_assert(LIGHTREC_BACKEND_WORDS == FGL_BACKEND_WORDS,
	       "lightrec was built with the wrong LIGHTREC_BACKEND_WORDS");
_Static_assert(sizeof(struct fgl_words) == FGL_BACKEND_WORDS * 4,
	       "fgl_words does not fill the backend block");

/* ----------------------------------------------------------------------
 * The core services fgl's assembly calls back into (dispatch.S, fgl_run.c):
 * lightrec's own functions, aliased rather than renamed.
 * ---------------------------------------------------------------------- */

void * fgl_get_next_block(struct lightrec_state *state, u32 pc)
{
	return lightrec_get_next_block_func(state, pc);
}

u32 fgl_memset(struct lightrec_state *state)
{
	return lightrec_memset(state);
}

u32 fgl_emulate_block(struct lightrec_state *state, struct block *block, u32 pc)
{
	return lightrec_emulate_block(state, block, pc);
}

u32 fgl_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg)
{
	return lightrec_check_load_delay(state, pc, reg);
}

/* ----------------------------------------------------------------------
 * Direct device access
 * ----------------------------------------------------------------------
 *
 * An IO_HW tag records where the op's FIRST access landed; later executions
 * may point anywhere, so only the I/O window takes the fast path and the
 * rest goes through the full map dispatch.  Called bare from generated code
 * with the cycle contract emitted inline.  Loads return the value already
 * sign/zero-extended.
 */
static inline const struct lightrec_mem_map_ops *
hw_shim_ops(struct lightrec_state *state, u32 kaddr)
{
	const struct lightrec_mem_map *map = &state->maps[PSX_MAP_HW_REGISTERS];

	if (likely(kaddr - map->pc < map->length))
		return map->ops;

	return NULL;
}

static u32 hw_shim_slow(struct lightrec_state *state, u32 op, u32 addr,
			u32 data)
{
	return lightrec_rw(state, (union code){ .i.op = op }, addr, data,
			   NULL, NULL, 0);
}

u32 lightrec_hw_lb(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u32)(s32)(s8)ops->lb(state, 0, NULL, kaddr);

	return hw_shim_slow(state, OP_LB, addr, 0);
}

u32 lightrec_hw_lbu(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u8)ops->lb(state, 0, NULL, kaddr);

	return hw_shim_slow(state, OP_LBU, addr, 0);
}

u32 lightrec_hw_lh(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u32)(s32)(s16)ops->lh(state, 0, NULL, kaddr);

	return hw_shim_slow(state, OP_LH, addr, 0);
}

u32 lightrec_hw_lhu(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u16)ops->lh(state, 0, NULL, kaddr);

	return hw_shim_slow(state, OP_LHU, addr, 0);
}

u32 lightrec_hw_lw(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return ops->lw(state, 0, NULL, kaddr);

	return hw_shim_slow(state, OP_LW, addr, 0);
}

void lightrec_hw_sb(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sb(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SB, addr, val);
}

void lightrec_hw_sh(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sh(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SH, addr, val);
}

void lightrec_hw_sw(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sw(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SW, addr, val);
}

/* ----------------------------------------------------------------------
 * The cycle meter
 * ----------------------------------------------------------------------
 *
 * The budget emitted code holds is a count of guest instructions, not
 * cycles: code charges `add #-n,r14` only, so the cycles-per-op multiply
 * lives at `in` (hand out) and `out` (settle).  On the target the shims do
 * the same arithmetic inline (shim.S), so the anchor and the parked budget
 * are state-block words, not statics.  Under the SH-4 interpreter fgl_run.c
 * keeps the anchor itself; the words exist in the layout either way.
 */
#ifdef __sh__
static u32 fgl_cycles_per_op(const struct lightrec_state *state)
{
	/* cycle_table[1] is cycles_per_op; read every time, it can move. */
	u32 cpo = fgl_state_words((struct lightrec_state *)state)->cycle_table[1];

	return cpo ? cpo : 1u;
}

/* Settling re-anchors, so the second settle `lightrec_execute` does with the
 * same delta charges zero instead of the whole slice twice. */
void fgl_cycles_out(struct lightrec_state *state, s32 delta)
{
	struct fgl_words *w = fgl_state_words(state);

	state->current_cycle += (u32)(w->meter_base - delta) *
			        fgl_cycles_per_op(state);
	w->meter_base = delta;
}

void fgl_cycles_settle(struct lightrec_state *state, s32 delta)
{
	fgl_cycles_out(state, delta);
}

/* `ceil`, not `floor`: the gate is `cmp/pl r14` (run while positive), and
 * `m - sum > 0` matches `sum < left/cpo` exactly when m = ceil(left/cpo). */
static s32 fgl_meter_of(const struct lightrec_state *state)
{
	u32 cpo  = fgl_cycles_per_op(state);
	s32 left = (s32)(state->target_cycle - state->current_cycle);

	return left > 0 ? (s32)(((u32)left + cpo - 1u) / cpo) : 0;
}

void fgl_meter_refresh(struct lightrec_state *state)
{
	fgl_state_words(state)->meter_in = fgl_meter_of(state);
}

s32 fgl_cycles_in(struct lightrec_state *state)
{
	struct fgl_words *w = fgl_state_words(state);
	/* No budget is zero budget, not a negative cycle count: the settle
	 * multiplies, so a cycle count leaking in comes back scaled. */
	s32 m = fgl_meter_of(state);

	/* Nothing has parked a budget for this slice yet. */
	w->exit_meter = INT32_MIN;
	w->meter_base = m;
	w->meter_in   = m;
	return m;
}

/* A budget in guest instructions, in cycles. */
u32 fgl_meter_cycles(const struct lightrec_state *state, s32 delta)
{
	return (u32)delta * fgl_cycles_per_op(state);
}
#else
/* The interpreter's runtime rebuilds the budget on every way back in. */
void fgl_meter_refresh(struct lightrec_state *state)
{
	(void)state;
}
#endif /* __sh__ */

/* ----------------------------------------------------------------------
 * The vtable
 * ---------------------------------------------------------------------- */

/* cycle_table[k] = k * cycles_per_op: a block's charge is one load whose
 * displacement is its instruction count. */
static void fgl_fill_cycle_table(struct lightrec_state *state)
{
	struct fgl_words *w = fgl_state_words(state);
	unsigned int i;

	for (i = 0; i < FGL_CYCLE_ENTRIES; i++)
		w->cycle_table[i] = i * state->cycles_per_op;
}

/* The dispatcher and the service trampoline are hand-written assembly, not
 * generated: fgl's register contract is fixed (fgl.h).  `c_wrappers[]` is
 * not filled in; fgl calls C through a fixed shim with the callee as a
 * literal. */
static int fgl_backend_init(struct lightrec_state *state)
{
	struct fgl_words *w = fgl_state_words(state);

	fgl_fill_cycle_table(state);

	state->memset_func      = fgl_dispatch_memset;
	state->get_next_block   = fgl_dispatch_compile;
	state->interpreter_func = fgl_dispatch_interpreter;
	state->ds_check_func    = fgl_dispatch_ds_check;

	/* What generated code reads out of the state block, all within GBR's
	 * 1020-byte reach (asserted in fgl_lightrec.c). */
#ifdef __sh__
	/* On the target these are host addresses. */
	w->dispatch  = (u32)(uintptr_t)fgl_dispatch_loop;
	w->link      = (u32)(uintptr_t)fgl_link_stub;
	w->lut_base  = (u32)(uintptr_t)state->code_lut;
#else
	/* On a workstation emitted code names them in 32 bits, so the SH-4
	 * runtime hands out addresses it will answer for itself. */
	w->dispatch  = fgl_run_dispatch_addr();
	w->link      = fgl_run_link_addr();
	w->lut_base  = fgl_run_lut_addr();
#endif
	w->addr_mask = 0x1fffffff;
	/* The code-table index mask, in the state block so emit_invalidate()
	 * and the stubs reach it through GBR.  Zero here would not fail
	 * loudly: every store would clear entry 0 and invalidation would
	 * silently stop working. */
	w->inv_mask  = 0x001ffffcu;

	return 0;
}

static void fgl_backend_destroy(struct lightrec_state *state)
{
}

/* fgl owns the lowering, register allocation, emission and placement, and
 * takes lightrec's optimised opcode list as input.  `cstate->targets[]` and
 * `local_branches[]` stay empty: fgl ends a block at the first control
 * transfer, so every branch leaves through the dispatcher. */
/* `fgl_blocks_refused`: a refused block is BLOCK_NEVER_COMPILE and
 * interpreted for the rest of the run, so a large count means the game is
 * running in C. */
unsigned long fgl_blocks_compiled, fgl_blocks_refused, fgl_blocks_nomem;
unsigned long fgl_bytes_emitted, fgl_ops_compiled;

static int fgl_backend_compile(struct lightrec_cstate *cstate,
			       struct block *block)
{
	unsigned int code_size = 0;
	void *new_fn;
	int err = 0;

	cstate->nb_local_branches = 0;
	cstate->nb_targets = 0;

	new_fn = fgl_compile_block(cstate, block, &code_size, &err);
	if (!new_fn) {
		if (err == -ENOMEM) {
			fgl_blocks_nomem++;
			if (!ENABLE_THREADED_COMPILER)
				pr_err("Code arena full compiling block at "
				       PC_FMT"\n", block->pc);
			return -ENOMEM;
		}

		/* fgl cannot lower this block.  A HOLE TO FILL, and the flag
		 * is not a fallback path -- it is what stops the emulator
		 * from spending the rest of its life recompiling the same
		 * refusal.  -ENOMEM here means "arena full" to the caller,
		 * which flushes the whole block cache and asks again, gets
		 * the same refusal, and flushes again.  Interpreting the
		 * block instead keeps the machine alive long enough for the
		 * message below to be read and the hole to be filled. */
		{	/* Capped: a refusal is worth seeing, a thousand of the
			 * same refusal is worth nothing and drowns the log. */
			static unsigned int said;

			if (said < 20) {
				said++;
				pr_err("fgl cannot lower the block at "PC_FMT
				       " -- interpreting it; this is a hole in "
				       "fgl, not a design\n", block->pc);
			}
		}

		fgl_blocks_refused++;
		block_set_flags(block, BLOCK_NEVER_COMPILE);
		return -EINVAL;
	}

	block->function = new_fn;
	block->code_size = code_size;

	fgl_blocks_compiled++;
	fgl_bytes_emitted += code_size;
	fgl_ops_compiled += block->nb_ops;

	/* THE ACCOUNTING GNU LIGHTNING USED TO DO IN lightrec_emit_code.
	 * lightrec_print_info reports MEM_FOR_CODE and divides it by
	 * MEM_FOR_MIPS_CODE to get the average instructions-per-instruction --
	 * the expansion factor of the compiler, and the one number that says
	 * whether a change to the emitter made the code bigger or smaller. */
	lightrec_register(MEM_FOR_CODE, block->code_size);

	return 0;
}

/* There is no compilation context to free: fgl's emitter lives on the stack
 * for the length of one call and owns nothing that outlives it.  The code
 * itself is the core's, allocated out of the shared arena. */
static void fgl_backend_free_block(struct lightrec_state *state,
				   struct block *block)
{
}

static void fgl_backend_unlink_range(struct lightrec_state *state, u32 offset,
				     unsigned int nb_ops, unsigned int why)
{
	fgl_unlink_range(state, offset, nb_ops, why);
}

static void fgl_backend_unlink_block(struct lightrec_state *state, u32 offset,
				     unsigned int nb_ops, void *fn,
				     unsigned int size, unsigned int why)
{
	fgl_unlink_block(state, offset, nb_ops, fn, size, why);
}

static void fgl_backend_unlink_all(struct lightrec_state *state,
				   unsigned int why)
{
	fgl_unlink_all(state, why);
}

static u32 fgl_backend_execute(struct lightrec_state *state, u32 pc,
			       u32 target_cycle)
{
	void *block_trace;
	s32 cycles_delta;

	/* Handle the cycle counter overflowing */
	if (unlikely(target_cycle < state->current_cycle))
		target_cycle = UINT_MAX;

	state->target_cycle = target_cycle;
	state->curr_pc = pc;

	block_trace = lightrec_get_next_block_func(state, pc);
	if (block_trace) {
		cycles_delta = state->target_cycle - state->current_cycle;

		/* Straight into the hand-written dispatcher.  Its signature is
		 * the one lightrec's generated dispatcher had, third argument
		 * included -- which fgl ignores, looking the first block up
		 * through the table like any other. */
		cycles_delta = fgl_dispatch(state, state->curr_pc,
					    block_trace, cycles_delta);

		fgl_cycles_settle(state, cycles_delta);
	}

	return state->curr_pc;
}

const struct lightrec_backend lightrec_backend_fgl = {
	.name		= "fgl",

	/* The LUI/ORI constant fold parks the LUI's immediate and leaves the
	 * consumer to materialise the whole constant.  fgl's front end honours
	 * that for ORI/ADDI/ADDIU only (movi_step, front.c): a tagged load is
	 * not consumed, so the parked half is flushed as an ordinary register
	 * write while the load, retagged IO_RAM and expecting the address
	 * folded in, adds its immediate a second time.  On Spyro that reaches
	 * the memory shim as an untranslated guest address and faults.
	 * Teaching movi_step to consume loads would let these back in. */
	.flags		= LIGHTREC_BACKEND_NO_LUI_LOAD_FOLD,

	/* 32-byte aligned, so every block starts an icache line and the Linux
	 * arena has bloom's line layout (fgl_cache.c). */
	.code_align	= 32,

	.init		= fgl_backend_init,
	.destroy	= fgl_backend_destroy,
	.compile	= fgl_backend_compile,
	.free_block	= fgl_backend_free_block,
	.cycles_changed	= fgl_fill_cycle_table,
	.cycles_moved	= fgl_meter_refresh,
	.execute	= fgl_backend_execute,
	.unlink_range	= fgl_backend_unlink_range,
	.unlink_block	= fgl_backend_unlink_block,
	.unlink_all	= fgl_backend_unlink_all,
};
