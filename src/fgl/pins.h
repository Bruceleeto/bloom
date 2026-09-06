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

/* THE GUEST-FILE BASE REGISTER, AND THE 2:1 IT DELETES.
 *
 * `mov.l @(disp,GBR),Rn` does not exist: the GBR displacement form has R0 as
 * its only destination.  So every guest register access is TWO instructions,
 * a GBR load and a `mov` off R0 -- exactly 2:1, and the harness measured it:
 * 83.4M guest-register accesses costing 167M of 598M executed instructions,
 * 27.9% of the program.  It is the largest single line item there is, and it
 * is also why R0 is a serialisation point that shows up as load-use stalls.
 *
 * `mov.l @(disp,Rm),Rn` has no such restriction, but its displacement is four
 * bits scaled by four -- 0..60 bytes, SIXTEEN words.  Guest registers live at
 * word `g` of the state block (`GUEST_AT`), so one base register pointing at
 * the state block reaches $0-$15 in ONE instruction to any destination, and
 * $16-$33 keep the GBR pair.  $0-$15 is $zero, $at, $v0/$v1, $a0-$a3 and
 * $t0-$t7 -- the half MIPS actually computes in.
 *
 * WHERE THE REGISTER COMES FROM.  All sixteen were spoken for, so this takes
 * the BOTTOM of the allocator's pool: `ALLOC_FIRST` moves 3 -> 4 and the pool
 * is r4-r12, nine deep.  It deliberately does not take r13 (the address mask,
 * one `and` on every guest memory access, ~27.6M of them) and it deliberately
 * does not drop a pin -- the pinned six are bloop's set, register for
 * register, and changing them makes every number incomparable with bloop.
 * What it costs is one of the four ROTATING registers, so the rotating set
 * goes 4 -> 3 and spilling rises.  Spilling is 8.1% of executed instructions
 * against the 14% this saves, so the trade only fails if losing a quarter of
 * the rotating set nearly doubles it.  That is what the A/B is for.
 *
 * IT IS A CONSTANT, WHICH IS WHY IT IS NEVER SAVED.  Its value is the state
 * block pointer, the same thing GBR holds, so anything that may have clobbered
 * it rebuilds it with `stc gbr, r3` rather than touching the stack.  That
 * matters because r3 is CALLER-saved in the SH-4 ABI: a compiled C callee may
 * destroy it and is entitled to.  `shim.S` happens to save r2-r7 already, so
 * the shims are covered; `dispatch.S` rebuilds it at the two places that come
 * back from C into generated code.
 *
 * Set FGL_GBASE to 0 to put every guest access back on the GBR pair and give
 * r3 back to the allocator.  That is the A/B, and it has to keep working. */
#ifndef FGL_GBASE
#define FGL_GBASE 1
#endif

#if FGL_GBASE
#define FGL_R_GBASE     3
#define FGL_GBASE_MAX   16      /* guest registers reachable: $0..$15 */
#endif

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
