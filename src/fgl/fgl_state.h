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

#include <stdint.h>

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

/* How many words of state block generated code can touch, so a harness knows
 * how much to allocate. */
#define FGL_STATE_WORDS 133u

/* The COP0 registers that are actually live.  Everything else reads zero and
 * discards writes. */
#define COP0_SR    12
#define COP0_CAUSE 13
#define COP0_EPC   14

#endif /* FGL_STATE_H */
