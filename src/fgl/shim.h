/* The service shims, as the emitter has to see them.
 *
 * shim.S is hand-written SH-4 and knows nothing about this header; this
 * header is the contract the emitter codes against.  Read shim.S for why any
 * of it is shaped the way it is -- the reasoning lives there, next to the
 * instructions.
 *
 * HOW A BLOCK REACHES ONE.  `jsr @Rn`, with the shim's address materialised
 * into a register first.  PR is free inside a block (a block does not return;
 * it jumps through FGL_AT_DISPATCH), so the `jsr` costs nothing to set up and
 * nothing to unwind.  But `jsr` needs its target in a general register and
 * neither r0 nor r1 can hold it -- both are carrying arguments -- so a node
 * that calls a shim MUST BE GIVEN A SCRATCH REGISTER BY THE ALLOCATION PASS,
 * the same way emit_muldiv is already given `sc[0]`.  It is dead the instant
 * the `jsr` issues, so it may be any register in the pool.
 *
 * A shim's address is a link-time constant, so it is an ordinary literal:
 * `mov.l @(disp,PC),Rs`, one pool word, and `bsr` is not an option because
 * blocks live in a code buffer nowhere near .text.
 *
 * WHAT SURVIVES A SHIM.  Everything the contract in fgl.h names except r0 and
 * r1 -- r2's exit PC, the whole of r3-r12, the mask in r13, GBR -- and the
 * stack comes back where it started.  The one exception is r14: a shim that
 * calls C rebuilds the cycle delta from the state block, because the C side
 * raises an interrupt by moving `target_cycle` in memory where a register
 * cannot see it.  That is why FGL_SHIM_WRITES_CYCLE is spelt out below rather
 * than left to be noticed.
 *
 * NOT PRESERVED, and safe today by construction rather than by contract: the
 * T bit (fgl never has a T value live across a node -- IR_COND emits its
 * compare and its branch adjacently), and MACH/MACL (emit_muldiv reads them
 * inside the node that wrote them).  Either of those changing makes a shim
 * change with it.
 */

#ifndef FGL_SHIM_H
#define FGL_SHIM_H

#include "fgl_state.h"

/* ---------------------------------------------------------------- */
/* The symbols                                                       */
/* ---------------------------------------------------------------- */

/* A general C-ABI service.
 *
 *      in    r0 = the callee, r1 = its first argument
 *      out   r0 = the callee's return value
 *      the callee sees  f(r4 = the argument, r5 = the state block)
 *
 * The second argument is GBR and is supplied whether or not the callee names
 * one -- that covers both `f(x)` and the `f(value, state)` shape of the
 * lightrec_hw_* family, and an argument register a function does not name is
 * invisible to it.
 *
 * Call site, five instructions and three pool words:
 *
 *      mov.l   @(disp,pc), r0          ; the callee
 *      mov.l   @(disp,pc), r1          ; the argument, if it is a constant
 *      mov.l   @(disp,pc), rS          ; fgl_shim_call
 *      jsr     @rS
 *       nop
 */
void fgl_shim_call(void);

/* A hardware store, which is the one service with three values to pass.
 *
 *      in    r0 = the callee, r1 = the address,
 *            state->shim_arg = the value to store
 *      out   nothing; lightrec_hw_sb/_sh/_sw all return void
 *      the callee sees  f(r4 = the address, r5 = the value,
 *                        r6 = the state block)
 *
 * `f(addr, val, state)` needs two values carried in from the block, and the
 * block has exactly two registers to carry anything in -- so there is no room
 * left for the callee itself.  The value takes the state block instead: the
 * call site parks it in FGL_AT_SHIM_ARG and the shim loads it into the second
 * argument register.  One store here, one load there, and no register.  That
 * word is not `temp_reg`; fgl_state.h says why.
 *
 * Call site, seven instructions and three pool words -- the five of
 * fgl_shim_call plus the two that park the value, which must be through r0
 * because @(disp,GBR) has no other destination:
 *
 *      mov     rV, r0                  ; the value to store
 *      mov.l   r0, @(FGL_AT_SHIM_ARG*4, gbr)
 *      mov.l   @(disp,pc), r0          ; the callee
 *      mov     rA, r1                  ; the address
 *      mov.l   @(disp,pc), rS          ; fgl_shim_call_st
 *      jsr     @rS
 *       nop
 */
void fgl_shim_call_st(void);

/* The GTE.
 *
 *      in    r0 = the callee, r1 = the guest COP2 command word
 *      out   nothing; every GTE body returns void
 *      the callee sees  f(r4 = &regs.cp2d, r5 = the command word)
 *
 * The callee comes from `gte_fpu_resolve(op)` AT COMPILE TIME, so both values
 * are literals the emitter already knows; there is no runtime dispatch and
 * nothing here decodes the command.  The first argument is not passed because
 * it is GBR + FGL_AT_CP2D*4 and the shim computes it -- which is exactly what
 * leaves the callee and the op word room in the only two registers a block
 * may hand over.
 *
 * Call site, five instructions and three pool words, as above with the op
 * word in r1 and `fgl_shim_gte` in the scratch.
 */
void fgl_shim_gte(void);

/* The 32-step divides, out of line.
 *
 *      in    r1 = the dividend, r0 = the divisor
 *      out   r0 = the quotient (guest LO), r1 = the remainder (guest HI)
 *
 * Which is where emit_muldiv already puts its operands, so the call site is
 * three instructions and one pool word on top of the operand setup that path
 * emits today:
 *
 *      mov.l   @(disp,pc), rS          ; fgl_shim_divu
 *      jsr     @rS
 *       nop
 *
 * against 77 emitted words for DIVU and 98 for DIV inline.
 *
 * NEITHER CALLS C, so neither reconciles the cycle counters and neither may
 * write r14 -- they are services in the plain sense of fgl.h, r0 and r1 and
 * nothing else.  Both produce the guest's defined results for a zero divisor
 * and both handle 0x80000000 / -1; the zero test is unconditional, so
 * FGL_H_NO_DIV_CHK buys nothing at a call site and is simply not consulted
 * there.  Both destroy T.
 */
void fgl_shim_divu(void);
void fgl_shim_div(void);

/* ---------------------------------------------------------------- */
/* What the emitter needs to size a call site                        */
/* ---------------------------------------------------------------- */

/* Instructions at the call site, and pool words it costs, so an IPI census
 * counts a service call rather than guessing at it.  The `+ operands` for the
 * divides is whatever emit_muldiv already emits to get the dividend into r1
 * and the divisor into r0. */
#define FGL_SHIM_CALL_INSNS  5
#define FGL_SHIM_CALL_POOL   3
#define FGL_SHIM_CALL_ST_INSNS 7
#define FGL_SHIM_CALL_ST_POOL  3
#define FGL_SHIM_GTE_INSNS   5
#define FGL_SHIM_GTE_POOL    3
#define FGL_SHIM_DIV_INSNS   3
#define FGL_SHIM_DIV_POOL    1

/* Instructions actually executed inside each body, for the same reason.  The
 * divides are ranges because the sign fixups and the non-restoring correction
 * are branched over when they are not needed. */
#define FGL_SHIM_CALL_BODY   29
#define FGL_SHIM_CALL_ST_BODY 29
#define FGL_SHIM_GTE_BODY    28
#define FGL_SHIM_DIVU_BODY   76         /* 75 when the last step was exact */
#define FGL_SHIM_DIV_BODY    95         /* 90 at best, by the operands' signs */

/* A node that calls a shim needs one scratch register from the allocation
 * pass to hold the shim's address for the `jsr`. */
#define FGL_SHIM_SCRATCH     1

/* The shims that rebuild the cycle delta, which is the one write to r14 that
 * anything other than a block epilogue is allowed to make. */
#define FGL_SHIM_WRITES_CYCLE(sym) \
	((sym) == fgl_shim_call || (sym) == fgl_shim_call_st || \
	 (sym) == fgl_shim_gte)

/* ---------------------------------------------------------------- */
/* Where shim.S and fgl_state.h agree, and why nothing here checks it */
/* ---------------------------------------------------------------- */

/* shim.S includes `fgl_state.h` directly and writes its GBR displacements as
 * `FGL_AT_* * 4`, exactly as dispatch.S does -- the header carries an
 * `__ASSEMBLER__` guard over its one C include for precisely this.  So there
 * is no second copy of the numbers to drift, and nothing to assert here.
 *
 * THE `* 4` IS NOT DECORATION.  The emitter puts a WORD INDEX straight into
 * the instruction word and lets the hardware scale it; a `.S` file does not,
 * and GAS wants the byte displacement.  Three quarters of these constants
 * would be caught as a misaligned offset, and the quarter that happen to be
 * multiples of four are accepted SILENTLY at the wrong field -- which is the
 * bug the dispatcher already shipped and found.  A shim reading `exit_flags`
 * where it meant `target_cycle` does not fault; it makes the machine drift.
 *
 * What is left to check is the reach, and that IS worth an assertion, because
 * @(disp,GBR) is an 8-bit displacement scaled by 4 and a layout change that
 * pushed the cycle pair past 1020 bytes would fail inside shim.S with a
 * message about an operand rather than about a layout. */
_Static_assert(FGL_AT_TARGET_CYCLE * 4u <= 1020u,
	       "shim.S: the cycle counters left GBR's reach");
_Static_assert(FGL_AT_CP2D * 4u <= 1020u,
	       "shim.S: the COP2 file left GBR's reach");
_Static_assert(FGL_AT_SHIM_ARG * 4u <= 1020u,
	       "shim.S: the third argument's word left GBR's reach");

#endif /* FGL_SHIM_H */
