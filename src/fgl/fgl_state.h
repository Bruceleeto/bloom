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
/* THESE THREE SIT AFTER THE BACKEND BLOCK, NOT INSIDE IT.
 *
 * `struct lightrec_state` gives a backend an opaque `backend[]` array at a
 * fixed offset and keeps its own fields behind it, so the cycle table and the
 * seven scratch words below are fgl's and land first; the core's three
 * counters follow.  All of it is still inside GBR's 1020-byte reach, which is
 * the only property that actually matters. */
#define FGL_AT_CURRENT_CYCLE (FGL_AT_SVC + FGL_SVC_N)                   /* +712 */
#define FGL_AT_TARGET_CYCLE  (FGL_AT_CURRENT_CYCLE + 1u)              /* +716 */
#define FGL_AT_EXIT_FLAGS    (FGL_AT_TARGET_CYCLE + 1u)               /* +720 */

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
#define FGL_AT_DISPATCH      (FGL_AT_CYCLES + FGL_CYCLE_ENTRIES)      /* +668 */

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
#define FGL_AT_LUT           (FGL_AT_DISPATCH + 1u)                   /* +672 */
#define FGL_AT_ADDR_MASK     (FGL_AT_LUT + 1u)                        /* +676 */

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
#define FGL_AT_SHIM_ARG      (FGL_AT_ADDR_MASK + 1u)                  /* +680 */

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
#define FGL_AT_LINK          (FGL_AT_SHIM_ARG + 1u)                   /* +684 */

/* How many words of state block generated code can touch, so a harness knows
 * how much to allocate. */
/* WHERE T IS PARKED ACROSS A DELAY SLOT (bloom, 2026-09).  The epilogue's
 * link arm is chosen by the T bit IR_COND's comparison left standing, and the
 * decoder puts the transfer AHEAD of its delay slot, so whenever the slot
 * decoded to anything real there are nodes after the IR_COND and any of them
 * may write T.  Blocks in that shape got no link site at all and went round
 * the dispatcher on every execution for ever -- on Linux 4.24M dispatcher
 * entries per 300 vsyncs against the DC's 168k, nearly all of them
 * two-instruction local-branch blocks.
 *
 * So T is parked here across the delay slot and restored at the link point.
 * Four instructions in conditional blocks, against a dispatcher round trip on
 * every single execution.
 *
 * NOT `shim_arg` AND NOT `temp_reg`: both are live across exactly the window
 * this one needs (a hardware store's third argument, and a deferred load's
 * parked value).  Two purposes that overlap in time need two words. */
#define FGL_AT_TSAVE         (FGL_AT_LINK + 1u)                       /* +688 */

/* WHAT r14 STILL HELD WHEN A BLOCK ASKED TO LEAVE (2026-09-10).
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
#define FGL_AT_EXIT_METER    (FGL_AT_TSAVE + 1u)                      /* +692 */

/* THE METER READING `current_cycle` ALREADY ACCOUNTS FOR.
 *
 * Settling charges `(base - r14) * cycles_per_op` and then moves the anchor
 * here, which is what makes a second settle with the same budget cost nothing
 * -- the dispatcher settles on its way out and `lightrec_execute` settles
 * again.  It is a state-block word rather than a C static because on the
 * target the SHIMS settle too, inline, and a shim cannot afford a call to do
 * it (shim.S, SHIM_CYCLES_OUT). */
#define FGL_AT_METER_BASE    (FGL_AT_EXIT_METER + 1u)                 /* +700 */

/* THE METER A SHIM PICKS BACK UP.
 *
 * A shim cannot rebuild the budget itself: that is `ceil((target - current) /
 * cycles_per_op)` and the SH-4 has no divide.  It does not have to, because
 * whatever moved the pair while it was out was C, and C can leave the answer
 * here -- the backend's `cycles_moved` hook is called from every core setter
 * that touches the pair, which is every way it moves under a running block.
 * SHIM_CYCLES_OUT seeds it with the meter it already has, so a call that
 * moves nothing reads its own value back. */
#define FGL_AT_METER_IN      (FGL_AT_METER_BASE + 1u)                 /* +704 */

/* A CONSTANT IN THE STATE BLOCK RATHER THAN IN THE LITERAL POOL.
 *
 * `mov.l @(disp,GBR),r0` and `mov.l @(disp,PC),r0` are both one instruction,
 * but the PC-relative one drags a four-byte word into the instruction stream
 * with it, and that word is deduplicated WITHIN a block only -- so a constant
 * every store in the game wants costs four bytes once per block that stores.
 * Measured on Spyro's bridge: 0x001ffffc appears 1205 times in the pool.
 *
 * bleem reaches its own such values the same way (`@(536,gbr)`, `@(548,gbr)`,
 * `@(552,gbr)` in recompiler_backend.md), and for the same reason.
 *
 * The cost is a state word and the r0-only restriction of the GBR form, which
 * is free here because the invalidation sequence builds its offset in r0
 * anyway. */
#define FGL_AT_INV_MASK      (FGL_AT_METER_IN + 1u)                   /* +708 */

/* THE INVALIDATION SEQUENCE, OUT OF LINE.
 *
 * Every store into guest RAM has to uncompile whatever was compiled from the
 * bytes it overwrote, and that was eleven instructions and a pool word
 * INLINED AT EVERY STORE -- 7.1% of all emitted bytes on Spyro's bridge, the
 * largest single thing in the emitter that is not a guest instruction.
 *
 * It is also identical at every site, so it collapses to a call:
 *
 *      mov.l   @(INV_RAM,gbr), r0
 *      jsr     @r0
 *       <slot>                         ; guest address already in r1
 *
 * Six bytes against sixteen (a store the optimiser proved is RAM) or
 * twenty-eight (one that has to test the region first).  It costs a `jsr`
 * and an `rts` EXECUTED at every store, which is the trade this whole
 * exercise is: instructions are at parity with bleem and bytes are twice its
 * number, so spending the first to buy the second is the right direction.
 *
 * PR is free to clobber -- a block does not use it; see FGL_AT_DISPATCH.
 * The RAM stub preserves T, the DIRECT one does not, which is exactly what
 * the two inline sequences did.
 *
 * The stubs are emitted once into the code arena and their addresses land
 * here, because that works unchanged on both the workstation (where the SH-4
 * interpreter executes the arena) and the Dreamcast (where the hardware
 * does).  Nothing needs to be assembled or linked. */
#define FGL_AT_INV_RAM       (FGL_AT_INV_MASK + 1u)                   /* +712 */
#define FGL_AT_INV_DIRECT    (FGL_AT_INV_RAM + 1u)                    /* +716 */

/* THE PINNED REGISTERS, PUBLISHED AND RELOADED OUT OF LINE.
 *
 * Every crossing into C has to put the pinned guest registers back into the
 * state block so the callee sees them, and pick them up again afterwards
 * because the callee may have changed them.  That is two instructions per pin
 * in each direction -- with six pins, TWENTY-FOUR INSTRUCTIONS, forty-eight
 * bytes, at every site.
 *
 * And it is the SAME forty-eight bytes every time: the pin table in pins.h is
 * compile-time constant, so host register and state displacement are both
 * fixed.  Nothing about a call site varies it.  Two shared routines and a
 * `jsr` each cost twelve.
 *
 * This is where the bytes were.  The generic unproven-address fallback
 * (`emit_rw`) is 20.3% of everything fgl emits on Spyro's bridge -- more than
 * the literal pool, more than block linking -- and most of a site is these
 * two sequences.  bleem pays none of it, because it classifies no address in
 * generated code at all (opcode_templates.md, 0x374D2). */
#define FGL_AT_PIN_PUB       (FGL_AT_INV_DIRECT + 1u)                 /* +720 */
#define FGL_AT_PIN_REL       (FGL_AT_PIN_PUB + 1u)                    /* +724 */

/* THE SERVICE ADDRESSES, WHICH ARE THE SAME AT EVERY SITE THAT USES THEM.
 *
 * `fgl_targets` is filled once and never changes, so every one of these is a
 * compile-time-known 32-bit value -- too big for `mov #imm`, so each site
 * spent a literal-pool word on it.  The pool is deduplicated WITHIN a block
 * and not across blocks, so the shim address wanted by a thousand blocks cost
 * four bytes a thousand times: 0x7d000000 alone was 4.8% of Spyro's pool.
 *
 * In a GBR slot it is `mov.l @(disp,gbr),r0` -- the same one instruction, no
 * pool word, and nothing to relocate.  This is bleem's "GBR helper slots"
 * table verbatim (opcode_templates.md lists six of them, @(552,gbr) for
 * JAL/JALR through @(596,gbr) for SWR).
 *
 * The r0-only restriction of the GBR form is why `emit_svc` takes a
 * destination: a site that wants the address somewhere else pays one `mov`
 * and still comes out two bytes ahead. */
/* THE GENERIC-ACCESS TRAMPOLINE.  Everything the unproven-address path did
 * inline -- publish the pins, publish curr_pc, load the two service
 * addresses, call, reload the pins -- written once out of line.  The site
 * keeps only the call and the two words that vary: the guest pc and the
 * guest instruction word, which the trampoline reads through PR. */
#define FGL_AT_RW_TRAMP      (FGL_AT_PIN_REL + 1u)                   /* +728 */

/* THE BLOCK A GENERIC SITE WAS COMPILED FROM: the third word at the site,
 * parked here by the trampoline for `fgl_rw`.  The tag an access earns has to
 * land on the opcode list the running code came from -- lightrec blocks
 * overlap, and a walk back from the pc found the NEAREST list, which was
 * tagged and recompiled while the block actually running kept crossing into
 * C on every execution (fgl_lightrec.c, fgl_rw_op). */
#define FGL_AT_RW_BLOCK      (FGL_AT_RW_TRAMP + 1u)                   /* +732 */

#define FGL_AT_SVC           (FGL_AT_RW_BLOCK + 1u)                    /* +736 */

#define FGL_SVC_SHIM_CALL     0u
#define FGL_SVC_SHIM_CALL_ST  1u
#define FGL_SVC_SHIM_GTE      2u
#define FGL_SVC_RW            3u
#define FGL_SVC_MTC           4u
#define FGL_SVC_MFC           5u
#define FGL_SVC_RFE           6u
#define FGL_SVC_HW_LOAD       7u      /* five, indexed by MEM_* */
#define FGL_SVC_HW_STORE     12u      /* five, indexed by MEM_* */

/* THE GTE LEAVES, for the same reason as everything above them.
 *
 * A COP2 command with a leaf is emitted as bleem emits it -- the routine's
 * address in r0 and a `jsr`, no shim and no cycle reconcile -- but the
 * address itself was still a literal, so every (block, command) pair paid a
 * pool word for it.  Spyro's bridge runs RTPT, NCDS and NCLIP in nearly
 * every geometry block; the pool is deduplicated within a block and not
 * across blocks, so that is four bytes per block per distinct command.
 *
 * The addresses are fixed after link and there are only 23 of them, so they
 * go in slots indexed by gte_fpu.h's GTE_LEAF_* directly.  Index 0 is not a
 * command and stays zero; paying one dead word buys the index arithmetic
 * being `k >> 1` and nothing else. */
#define FGL_SVC_GTE_LEAF     17u
#define FGL_GTE_LEAF_N       23u      /* must equal gte_fpu.h GTE_LEAF_N */

#define FGL_SVC_N            (FGL_SVC_GTE_LEAF + FGL_GTE_LEAF_N)

#define FGL_STATE_WORDS (FGL_AT_EXIT_FLAGS + 1u)

/* What the core has to reserve in struct lightrec_state::backend[] for this
 * backend: the cycle table plus the seven words after it.  Passed to
 * lightrec as LIGHTREC_BACKEND_WORDS and asserted in fgl_lightrec.c. */
#define FGL_BACKEND_WORDS (FGL_CYCLE_ENTRIES + 16u + FGL_SVC_N)

/* The COP0 registers that are actually live.  Everything else reads zero and
 * discards writes. */
#define COP0_SR    12
#define COP0_CAUSE 13
#define COP0_EPC   14

#endif /* FGL_STATE_H */
