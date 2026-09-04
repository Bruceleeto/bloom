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

/* How many words of state block generated code can touch, so a harness knows
 * how much to allocate. */
#define FGL_STATE_WORDS (FGL_AT_ADDR_MASK + 1u)

/* The COP0 registers that are actually live.  Everything else reads zero and
 * discards writes. */
#define COP0_SR    12
#define COP0_CAUSE 13
#define COP0_EPC   14

#endif /* FGL_STATE_H */
