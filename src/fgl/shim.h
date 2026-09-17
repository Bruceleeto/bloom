/* The service shims, as the emitter codes against them.  shim.S has the bodies.
 *
 * A block reaches one by `jsr @Rn`: r0 and r1 carry arguments, so the node
 * needs a scratch register from the allocation pass for the address.
 *
 * Survives a shim: everything but r0, r1, T and MACH/MACL.  r14 is rebuilt
 * by the shims that call C (FGL_SHIM_WRITES_CYCLE), since C moves target_cycle.
 */

#ifndef FGL_SHIM_H
#define FGL_SHIM_H

#include "fgl_state.h"

/* ---------------------------------------------------------------- */
/* The symbols                                                       */
/* ---------------------------------------------------------------- */

/* A general C-ABI service.
 *      in    r0 = the callee, r1 = its first argument
 *      out   r0 = the callee's return value
 *      the callee sees  f(r4 = the argument, r5 = the state block)
 * Call site: five instructions, three pool words.
 */
void fgl_shim_call(void);

/* A hardware store: the one service with three values.
 *      in    r0 = the callee, r1 = the address, state->shim_arg = the value
 *      the callee sees  f(r4 = the address, r5 = the value, r6 = the state block)
 * The value goes through FGL_AT_SHIM_ARG because the block has only r0/r1 to hand over.
 */
void fgl_shim_call_st(void);

/* The GTE.
 *      in    r0 = the callee (from gte_fpu_resolve at compile time), r1 = the COP2 command word
 *      the callee sees  f(r4 = &regs.cp2d, r5 = the command word)
 */
void fgl_shim_gte(void);

/* The 32-step divides, out of line.
 *      in    r1 = the dividend, r0 = the divisor
 *      out   r0 = the quotient (LO), r1 = the remainder (HI)
 * Neither calls C nor writes r14.  Both handle a zero divisor and 0x80000000 / -1.  Both destroy T.
 */
void fgl_shim_divu(void);
void fgl_shim_div(void);

/* ---------------------------------------------------------------- */
/* What the emitter needs to size a call site                        */
/* ---------------------------------------------------------------- */

/* Instructions and pool words at a call site, for the IPI census. */
#define FGL_SHIM_CALL_INSNS  5
#define FGL_SHIM_CALL_POOL   3
#define FGL_SHIM_CALL_ST_INSNS 7
#define FGL_SHIM_CALL_ST_POOL  3
#define FGL_SHIM_GTE_INSNS   5
#define FGL_SHIM_GTE_POOL    3
#define FGL_SHIM_DIV_INSNS   3
#define FGL_SHIM_DIV_POOL    1

/* Instructions executed inside each body; the divides are ranges. */
#define FGL_SHIM_CALL_BODY   29
#define FGL_SHIM_CALL_ST_BODY 29
#define FGL_SHIM_GTE_BODY    28
#define FGL_SHIM_DIVU_BODY   76         /* 75 when the last step was exact */
#define FGL_SHIM_DIV_BODY    95         /* 90 at best, by the operands' signs */

/* A node that calls a shim needs one scratch register for the `jsr`. */
#define FGL_SHIM_SCRATCH     1

/* The shims that rebuild the cycle delta: the one write to r14 outside a block epilogue. */
#define FGL_SHIM_WRITES_CYCLE(sym) \
	((sym) == fgl_shim_call || (sym) == fgl_shim_call_st || \
	 (sym) == fgl_shim_gte)

/* ---------------------------------------------------------------- */
/* Where shim.S and fgl_state.h agree, and why nothing here checks it */
/* ---------------------------------------------------------------- */

/* shim.S includes fgl_state.h and writes displacements as `FGL_AT_* * 4`
 * (GAS wants bytes; the emitter uses word indexes).  Only the reach needs
 * checking: @(disp,GBR) is 8 bits scaled by 4.
 */
_Static_assert(FGL_AT_TARGET_CYCLE * 4u <= 1020u,
	       "shim.S: the cycle counters left GBR's reach");
_Static_assert(FGL_AT_CP2D * 4u <= 1020u,
	       "shim.S: the COP2 file left GBR's reach");
_Static_assert(FGL_AT_SHIM_ARG * 4u <= 1020u,
	       "shim.S: the third argument's word left GBR's reach");

#endif /* FGL_SHIM_H */
