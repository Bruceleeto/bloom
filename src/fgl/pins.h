/* The pinned register assignment, in one place because two languages need it.
 *
 * A PINNED GUEST REGISTER IS ONE A BLOCK ASSUMES IT WAS HANDED.  It arrives in
 * a host register, put there by whoever branched here, and it leaves the same
 * way -- no load at entry, no store at exit.  That is worth real instructions
 * on every block, and it is also a contract between code compiled separately,
 * so every route into a block has to honour the same assignment or the block
 * reads someone else's value.
 *
 * THE ROUTES ARE THE WHOLE PROBLEM, AND THERE ARE ONLY TWO KINDS.
 *
 *   - Generated code reaching generated code.  Free: the dispatcher touches
 *     only r0, r1, r2, and r3-r12 are nobody's but the allocator's, so a
 *     pinned register survives a trip through `fgl_dispatch_loop` untouched.
 *     That is what makes this pay -- the cost is at the C boundary, not per
 *     block.
 *
 *   - Anything that reaches C.  C reads and writes guest registers through the
 *     state block and knows nothing about r9-r12, so a pinned value has to be
 *     PUBLISHED into the state block before the crossing and RELOADED after it
 *     if C could have written it.  `dispatch.S` does that at its crossings,
 *     `emit.c` at the nodes that call a service.  A missed one does not crash:
 *     it hands C a stale register and the guest drifts.
 *
 * WHY THE TOP OF THE POOL.  Slot k is host register ALLOC_FIRST + k, so the
 * pins below are r12 downwards, and the allocator spends its lowest-ranked
 * registers first (alloc.c) -- so pinning the top means a short block never
 * disturbs a pinned register at all and the flush has nothing to put back.
 *
 * r7 IS PINNED EVEN THOUGH IT IS CALLER-SAVED, and that is not an oversight.
 * r8-r15 are callee-saved in the SH-4 ABI, so a pin above r8 survives a
 * compiled C call on its own; r7 does not.  It does not have to: `shim.S`
 * saves r2-r7 across every crossing it owns, which is the same set, so the
 * register is preserved by the shim rather than by the callee.  The publish
 * below exists for a different reason entirely -- C reads the STATE BLOCK, not
 * the register -- and that reason applies equally to all six.
 *
 * WHICH GUEST REGISTERS: BLOOP'S SIX, EXACTLY.  $a0 $a1 $at $sp $v0 $v1, in
 * bloop's own host registers (`bloop/src/alloc.c`, `ir_pin[]`).  Matching it
 * is the point of the exercise, so this table is not a place to be creative:
 * bloop is the measured target and a different set makes every number
 * incomparable with it.
 *
 * DO NOT CITE THE OLD "FOUR BEAT FIVE" MEASUREMENT AGAINST THIS.  It came off
 * the GNU Lightning regcache with a candidate set of `v0 v1 a0 a1 at a2` --
 * $sp was never in it, and $sp is the register bloop values most, being in
 * nearly every load and store compiled MIPS emits.  A number that could not
 * have chosen $sp says nothing about a set that contains it.
 *
 * If it is re-benched, use INSTRUCTION COUNTS.  Changing FGL_NUM_PINS changes
 * .text, so matched-pair frame timing will not hold.
 *
 * SETTING FGL_NUM_PINS TO 0 TURNS ALL OF THIS OFF and puts every guest
 * register back through the state block.  Keep that working: it is the only
 * A/B that isolates a pinning bug from an emitter bug.
 */

#ifndef FGL_PINS_H
#define FGL_PINS_H

/* How many of the four below are live.  0 disables pinning entirely. */
#ifndef FGL_NUM_PINS
#define FGL_NUM_PINS 6
#endif

#if FGL_NUM_PINS > 6
#error "FGL_NUM_PINS is larger than the table below"
#endif

/* host register, guest register -- bloop's assignment, register for register.
 * Host must be in ALLOC_FIRST..ALLOC_FIRST+9.  Dropping FGL_NUM_PINS below six
 * drops from the END of this list, so the order is bloop's priority: the ones
 * most worth keeping are first. */
#define FGL_PIN0_HOST   10
#define FGL_PIN0_GUEST  29      /* $sp */
#define FGL_PIN1_HOST   11
#define FGL_PIN1_GUEST   2      /* $v0 */
#define FGL_PIN2_HOST   12
#define FGL_PIN2_GUEST   3      /* $v1 */
#define FGL_PIN3_HOST    7
#define FGL_PIN3_GUEST   4      /* $a0 */
#define FGL_PIN4_HOST    8
#define FGL_PIN4_GUEST   5      /* $a1 */
#define FGL_PIN5_HOST    9
#define FGL_PIN5_GUEST   1      /* $at */

/* The allocator's per-slot view of the same table, and the switch that makes
 * the flush treat a pinned register as live in its host register rather than
 * as something to store.  The two must move together: pinning without
 * ALLOC_PINNED_LIVE stores every pinned register at every flush, which is
 * correct but is the cost the pinning exists to avoid; ALLOC_PINNED_LIVE
 * without pinning is a no-op.  See alloc.c on both. */
#if FGL_NUM_PINS > 0
#define ALLOC_PINNED_LIVE 1
#endif

#endif /* FGL_PINS_H */
