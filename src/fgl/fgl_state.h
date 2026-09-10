/* The guest state block, as generated code sees it.
 *
 * GBR points here.  Every access from generated code is one
 * `mov.l @(disp,GBR),r0` -- an 8-bit displacement scaled by 4, so 1020 bytes
 * of reach, and everything hot has to live inside it.  That reach is why the
 * state pointer is GBR and not a GPR: `@(disp,Rn)` has a 4-bit field, 60
 * bytes, so a GPR-based state pointer costs three instructions per access
 * instead of one.
 *
 * WE DID NOT HAVE TO DESIGN THIS LAYOUT.  lightrec's `struct lightrec_state`
 * already opens with `struct lightrec_registers` (lightrec.h:119), and the
 * whole of it lands inside GBR's reach:
 *
 *      gpr[34]     0 .. 135      guest register n is at displacement 4n
 *      cp0[32]   136 .. 263
 *      cp2d[32]  264 .. 391      COP2 data
 *      cp2c[32]  392 .. 519      COP2 control
 *      then curr_pc, next_pc, cycle counters, ~520 .. 560
 *
 * Two things fall out of that and both are worth stating, because they are
 * the reason we keep lightrec's runtime rather than porting a state block:
 *
 *   - A guest register number IS its displacement, scaled.  The writeback
 *     flush emits a store whose displacement is the raw register number and
 *     the instruction does the scaling.  No translation table.
 *   - LO and HI are `gpr[32]` and `gpr[33]` (lightrec-private.h:93), which is
 *     already how the IR numbers them.  So the mapping is the identity, and
 *     the one conditional other designs need here does not exist.
 *
 * The static assertions live in the emitter's translation unit, where the
 * real struct is in scope; this header is also compiled by host tools that
 * do not have lightrec's headers, which is why the numbers are literals.
 */

#ifndef FGL_STATE_H
#define FGL_STATE_H

#ifndef __ASSEMBLER__
#include <stdint.h>
#endif

/* NOTE FOR ANYTHING WRITTEN IN ASSEMBLY.  Every FGL_AT_* below is a WORD
 * index, because that is what the emitter wants: `mov.l @(disp,GBR),R0`
 * carries an 8-bit field that the hardware scales by four, and the emitter
 * puts the index straight into the instruction word.  The assembler does not
 * work that way -- `mov.l @(disp,gbr),r0` in a .S file takes the displacement
 * in BYTES and scales it itself.  So hand-written assembly must write
 * `FGL_AT_CURR_PC * 4`, and forgetting the * 4 is caught by the assembler as
 * "misaligned offset" for three quarters of these constants and SILENTLY
 * ACCEPTED for the quarter that happen to be multiples of four. */

/* What the IR calls LO and HI.  Also where they are: see above. */
#define GUEST_LO 32
#define GUEST_HI 33

/* A guest register's word index in the state block.  The identity, kept as a
 * macro so the one place that would have to change is one place. */
#define GUEST_AT(g) ((unsigned)(g))

/* Word indices of the coprocessor files. */
#define FGL_AT_COP0 34u         /* +136 */
#define FGL_AT_CP2D 66u         /* +264 */
#define FGL_AT_CP2C 98u         /* +392 */

/* Where a block publishes the guest PC it leaves for. lightrec keeps
 * `curr_pc` and `next_pc` immediately behind the register files, and both are
 * still inside GBR's reach -- see the layout above. */
#define FGL_AT_TEMP_REG 130u    /* +520 */
#define FGL_AT_CURR_PC  131u    /* +524 */
#define FGL_AT_NEXT_PC  132u    /* +528 */

/* THE CYCLE TABLE: A MULTIPLY THE EMITTER DOES NOT HAVE TO DO.
 *
 * A block charges `n_ops * cycles_per_op`. `cycles_per_op` is not a compile
 * time constant -- lightrec's frontend derives it from the cycle multiplier
 * and it lands somewhere around 1800 -- so the charge is a runtime value too
 * big for `mov #imm`, and the obvious lowering is a literal in the pool.
 *
 * Instead the products are precomputed, one per possible block length, and
 * THE INSTRUCTION COUNT IS THE DISPLACEMENT. The charge is then a single
 * GBR-relative load of a value that was never computed at run time and never
 * took a word of literal pool:
 *
 *      mov.l   @(FGL_AT_CYCLES + n_ops, gbr), r0
 *      sub     r0, r14
 *
 * Two instructions either way, so this is not about instruction count -- it
 * is four bytes of pool per block against 136 bytes of table for the whole
 * program, and footprint is the thing this project is chasing.
 *
 * The table sits immediately after `next_pc` so its displacement does not
 * depend on anything below it in lightrec's state struct. Whoever wires fgl
 * into the emulator must put it there and fill it; see build/docs/CLAUDE.md. */
#define FGL_AT_CYCLES 133u      /* +532, 34 entries: k * cycles_per_op */
#define FGL_CYCLE_ENTRIES 34u   /* 0 .. IR_MAX_INSNS + 1 inclusive     */

/* THE COUNTERS AND THE FLAG, WHICH ONLY THE EXIT PATHS TOUCH.
 *
 * `current_cycle` and `target_cycle` are lightrec's absolute pair; the live
 * quantity while blocks are chaining is the signed delta in r14, and these two
 * are only reconciled when a block leaves for C. `exit_flags` is how it says
 * why.
 *
 * THEIR DISPLACEMENTS ARE DECLARED HERE AND lightrec's STRUCT MUST MATCH.
 * In lightrec's own layout these sit after `wrapper_regs[NUM_TEMPS]`, and
 * NUM_TEMPS came from GNU Lightning -- which is deleted, so the offsets are
 * not merely unknown, they no longer have a definition. fgl replaces the
 * register cache that gave them meaning, so the order is ours to state: these
 * follow the cycle table, and every one of them stays inside GBR's 1020-byte
 * reach, which is the only property that actually matters.
 *
 * Whoever wires fgl into the emulator reorders `struct lightrec_state` to
 * match and checks it with a static assertion, rather than trusting this
 * comment. Nothing here is discoverable at run time: a wrong displacement
 * reads a neighbouring field and the machine desynchronises quietly. */
#define FGL_AT_CURRENT_CYCLE (FGL_AT_CYCLES + FGL_CYCLE_ENTRIES)      /* +668 */
#define FGL_AT_TARGET_CYCLE  (FGL_AT_CURRENT_CYCLE + 1u)              /* +672 */
#define FGL_AT_EXIT_FLAGS    (FGL_AT_TARGET_CYCLE + 1u)               /* +676 */

/* WHERE A BLOCK GOES WHEN IT IS DONE, AND WHY IT IS NOT `rts`.
 *
 * The obvious epilogue returns: the dispatcher calls a block with `jsr` and
 * the block ends with `rts`.  That works, and it was what fgl did, and it
 * quietly makes PR part of the register contract -- PR holds the block's
 * return address for the block's entire life, so every service routine called
 * from inside a block has to save and restore it, and the contract in fgl.h
 * says a service may clobber "r0 and r1 and nothing else" without mentioning
 * PR at all.  That silence is a bug waiting on the first service.
 *
 * Ending in an indirect jump through this slot costs exactly the same five
 * instructions and removes PR from the contract entirely.  A block does not
 * use PR, so a service may do what it likes with it.
 *
 * It also buys the thing that actually matters later: a block that ends in a
 * jump through a table slot can have that jump PATCHED into a direct branch
 * to the next block, which is where the frames are (bloop links blocks
 * directly and reaches the indirect dispatcher on 0.3% of entries).  A block
 * that ends in `rts` can never be linked to anything. */
#define FGL_AT_DISPATCH      (FGL_AT_EXIT_FLAGS + 1u)                 /* +680 */

/* THE BLOCK TABLE, WHICH CANNOT BE REACHED ANY OTHER WAY.
 *
 * lightrec's `code_lut[]` is a flexible array member at the END of
 * `struct lightrec_state`, hundreds of bytes past GBR's 1020-byte reach, so
 * the dispatcher cannot name it with a displacement. The alternative is
 * `stc gbr,r0` plus an add of `sizeof(struct lightrec_state)` from a literal,
 * which is three instructions on the hottest path in the emulator instead of
 * one load. So the base is cached here.
 *
 * `addr_mask` is what the dispatcher loads into r13 on the way in. It lives
 * here rather than as a literal in the dispatcher so that there is exactly one
 * definition of it: the emitter's `and r13,r0` and whatever decides which
 * pages are mapped have to agree, and a second copy is how they stop
 * agreeing. */
#define FGL_AT_LUT           (FGL_AT_DISPATCH + 1u)                   /* +684 */
#define FGL_AT_ADDR_MASK     (FGL_AT_LUT + 1u)                        /* +688 */

/* THE THIRD ARGUMENT, FOR THE ONE SERVICE THAT NEEDS ONE.
 *
 * A block hands a service its arguments in r0 and r1, and that is all it has:
 * r2 is carrying the exit PC and r3-r12 are carrying guest values.  Two is
 * enough for every service but one.  `lightrec_hw_sb(addr, val, state)` takes
 * three, and the state block is the third of them, so the value has nowhere
 * left to travel.
 *
 * It travels here.  The call site stores it before the call and the shim
 * loads it into the third argument register, which costs one instruction on
 * each side and no registers at all.
 *
 * NOT `temp_reg`, WHICH IS THE OBVIOUS PLACE AND IS WRONG.  That word is the
 * parking slot for a deferred load's value (see ir.h) and the park outlives
 * the node that made it -- the load parks, an unrelated node runs, and
 * IR_TEMP_GET collects.  If that unrelated node is a hardware store, reusing
 * the slot destroys a guest register's value with nothing to show for it.
 * Two purposes that overlap in time need two words. */
#define FGL_AT_SHIM_ARG      (FGL_AT_ADDR_MASK + 1u)                  /* +692 */

/* THE LINK STUB, WHICH IS A DISPATCH THAT DELETES ITSELF.
 *
 * A block whose successor is a compile-time constant does not have to go round
 * the dispatcher to reach it -- it can branch straight there.  It cannot do
 * that when it is compiled, because the successor usually is not compiled yet,
 * so the branch is emitted as a call to the stub whose address lives here and
 * the stub REWRITES ITS OWN CALL SITE into a `bra` the first time it runs.
 * After that the edge costs two instructions and no memory reference at all.
 *
 * It is a state-block word rather than a literal in each block's pool for the
 * ordinary reason: one instruction to reach instead of one instruction plus
 * four bytes of pool, on an edge that exists in nearly every block. */
#define FGL_AT_LINK          (FGL_AT_SHIM_ARG + 1u)                   /* +696 */

/* How many words of state block generated code can touch, so a harness knows
 * how much to allocate. */
/* WHERE THE T BIT WAITS OUT A DELAY SLOT.
 *
 * A conditional block's link arm is chosen by the T bit that IR_COND's
 * comparison left standing.  That works only while IR_COND is the last node:
 * the decoder puts a transfer AHEAD of its delay slot, so a delay slot that
 * decoded to anything real emits nodes after IR_COND, and any of them may
 * write T.  Blocks in that shape got no link site at all and went round the
 * dispatcher on every execution for ever -- measured at 95% of all dispatcher
 * entries, 11.86M of 12.49M in a 41-second run.
 *
 * So T is parked here across the delay slot and restored at the link point.
 * Four instructions in conditional blocks, against a dispatcher round trip on
 * every single execution.
 *
 * NOT `shim_arg` AND NOT `temp_reg`: both are live across exactly the window
 * this one needs (a hardware store's third argument, and a deferred load's
 * parked value).  Two purposes that overlap in time need two words. */
#define FGL_AT_TSAVE         (FGL_AT_LINK + 1u)                       /* +700 */

/* WHAT r14 STILL HELD WHEN A BLOCK ASKED TO LEAVE.
 *
 * The budget is a count of guest instructions now, so `IR_EXIT` can no longer
 * reconcile the absolute counters itself -- turning a meter into cycles needs
 * a multiply, and the anchor it would multiply against lives in C.  So the
 * exit parks the unspent meter here and zeroes r14 to close the gate, and the
 * dispatcher charges from this instead of from r14 (which the epilogue has
 * since taken the block's own length out of).
 *
 * C stamps it back to INT32_MIN once read, so a stale value can never be
 * mistaken for a fresh one: an exit flag raised from C, and there are
 * several, leaves this alone. */
#define FGL_AT_EXIT_METER    (FGL_AT_TSAVE + 1u)                      /* +704 */

/* THE METER READING `current_cycle` ALREADY ACCOUNTS FOR.
 *
 * Settling charges `(base - r14) * cycles_per_op` and then moves the anchor
 * here, which is what makes a second settle with the same budget cost nothing
 * -- the dispatcher settles on its way out and `lightrec_execute` settles
 * again.  It lives in the state block rather than in a C static because the
 * shims settle too, and a shim cannot afford a call to do it. */
#define FGL_AT_METER_BASE    (FGL_AT_EXIT_METER + 1u)                 /* +708 */

/* THE METER A SHIM PICKS BACK UP.
 *
 * A shim cannot rebuild the budget itself: that is `ceil((target - current) /
 * cycles_per_op)` and the SH-4 has no divide.  It does not have to, because
 * whatever moved the pair while it was out was C, and C can leave the answer
 * here -- `lightrec_set_target_cycle_count`, `lightrec_reset_cycle_count` and
 * `lightrec_set_exit_flags` all refresh it, which is every way the pair moves
 * under a running block.  SHIM_CYCLES_OUT seeds it with the meter it already
 * has, so a call that moves nothing reads its own value back.
 *
 * The forced exit falls out for free: `set_exit_flags` pulls the target down
 * to the current cycle, so what lands here is zero and the gate closes. */
#define FGL_AT_METER_IN      (FGL_AT_METER_BASE + 1u)                 /* +712 */

#define FGL_STATE_WORDS (FGL_AT_METER_IN + 1u)

/* The COP0 registers that are actually live.  Everything else reads zero and
 * discards writes. */
#define COP0_SR    12
#define COP0_CAUSE 13
#define COP0_EPC   14

#endif /* FGL_STATE_H */
