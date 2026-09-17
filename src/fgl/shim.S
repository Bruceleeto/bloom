/* fgl's service shims: the layer between generated code (fgl's register
 * contract, see fgl.h) and the SH-4 C ABI.
 *
 * r8-r15 are callee-saved, so a C callee returns r8-r12 of the guest pool,
 * r13 (mask), r14 (cycle meter) and the stack on its own.  Only r2-r7 and PR
 * need saving here.  PR is free: a block never returns, it jumps through
 * FGL_AT_DISPATCH.
 *
 * Never save FPU state and never `frchg`: gte_fpu.c keeps the rotation
 * matrix resident in the back bank.  GBR is not callee-saved and survives
 * only because nothing in the image uses TLS.  T and MACH/MACL are never
 * live across a service. */

#include "fgl_state.h"

	.text
	.align	2

/* Save r2-r7 and PR; r2 goes deepest so its pop fills the `rts` slot. */
.macro	SHIM_ENTER
	mov.l	r2, @-r15
	mov.l	r3, @-r15
	mov.l	r4, @-r15
	mov.l	r5, @-r15
	mov.l	r6, @-r15
	mov.l	r7, @-r15
	sts.l	pr, @-r15
.endm

.macro	SHIM_LEAVE
	lds.l	@r15+, pr
	mov.l	@r15+, r7
	mov.l	@r15+, r6
	mov.l	@r15+, r5
	mov.l	@r15+, r4
	mov.l	@r15+, r3
	rts
	 mov.l	@r15+, r2
.endm

/* Make the absolute cycle pair current before calling C: C raises interrupts
 * by moving `target_cycle` in memory, which a delta in r14 cannot see.
 * r14 counts guest instructions, so `current += (base - r14) * cycles_per_op`
 * and the anchor moves to r14.  `cycle_table[1]` is cycles_per_op.  Only
 * these shims may write r14. */
.macro	SHIM_CYCLES_OUT
	mov.l	@(FGL_AT_METER_BASE * 4, gbr), r0
	sub	r14, r0			/* the meter spent since the anchor */
	mov	r0, r2
	mov.l	@((FGL_AT_CYCLES + 1) * 4, gbr), r0	/* cycles_per_op */
	mul.l	r0, r2
	mov.l	@(FGL_AT_CURRENT_CYCLE * 4, gbr), r0
	sts	macl, r2
	add	r2, r0
	mov.l	r0, @(FGL_AT_CURRENT_CYCLE * 4, gbr)
	mov	r14, r0
	mov.l	r0, @(FGL_AT_METER_BASE * 4, gbr)
	/* Seed FGL_AT_METER_IN so a callee that moves nothing reads it back. */
	mov.l	r0, @(FGL_AT_METER_IN * 4, gbr)
.endm

/* Back in: C leaves the rebuilt meter in FGL_AT_METER_IN every time it moves
 * the pair (the SH-4 has no divide).  The anchor must move with it, or the
 * next settle spans two readings on different scales.  r1 is left alone for
 * a parked return value. */
.macro	SHIM_CYCLES_IN
	mov.l	@(FGL_AT_METER_IN * 4, gbr), r0
	mov	r0, r14
	mov.l	r0, @(FGL_AT_METER_BASE * 4, gbr)
.endm

/* (a) General C call.
 *      entry   r0 = callee, r1 = first argument
 *      exit    r0 = return value
 * Callee gets r4 = argument, r5 = state pointer; a one-argument callee
 * ignores r5.  The callee is parked in r7 because the settle needs r0. */
	.align	2
	.globl	_fgl_shim_call
	.type	_fgl_shim_call, @function
_fgl_shim_call:
	SHIM_ENTER
	mov	r0, r7			/* the callee, out of the way */
	mov	r1, r4			/* argument one */
	SHIM_CYCLES_OUT
	stc	gbr, r5			/* argument two: the state block */
	jsr	@r7
	 nop
	mov	r0, r1			/* park the return value */
	SHIM_CYCLES_IN
	mov	r1, r0
	SHIM_LEAVE
	.size	_fgl_shim_call, . - _fgl_shim_call

/* (b) Hardware store, three arguments.
 *      entry   r0 = callee, r1 = address, FGL_AT_SHIM_ARG = value
 *      exit    nothing
 * The value travels through the state block: the block has only r0 and r1
 * to give.  It is loaded AFTER the settle, which needs r0. */
	.align	2
	.globl	_fgl_shim_call_st
	.type	_fgl_shim_call_st, @function
_fgl_shim_call_st:
	SHIM_ENTER
	mov	r0, r7			/* the callee, out of the way */
	mov	r1, r4			/* argument one: the address */
	SHIM_CYCLES_OUT
	mov.l	@(FGL_AT_SHIM_ARG * 4, gbr), r0
	mov	r0, r5			/* argument two: the value */
	stc	gbr, r6			/* argument three: the state block */
	jsr	@r7
	 nop
	SHIM_CYCLES_IN
	SHIM_LEAVE
	.size	_fgl_shim_call_st, . - _fgl_shim_call_st

/* (c) GTE body, `f(psxCP2Regs *, u32 op)`.
 *      entry   r0 = callee, r1 = COP2 command word
 *      exit    nothing
 * regs = GBR + FGL_AT_CP2D * 4 (cp2d and cp2c are contiguous). */
	.align	2
	.globl	_fgl_shim_gte
	.type	_fgl_shim_gte, @function
_fgl_shim_gte:
	/* No cycle settle: a COP2 body cannot move the clock or take an
	 * exception.  Unused when every command is a leaf (gte_rtp.S). */
	SHIM_ENTER
	mov	r0, r7			/* the callee, out of the way */
	mov	r1, r5			/* argument two: the command word */
	mov.w	1f, r0
	stc	gbr, r4
	jsr	@r7
	 add	r0, r4			/* argument one: &regs.cp2d */
	SHIM_LEAVE
	.align	2
1:	.word	FGL_AT_CP2D * 4
	.size	_fgl_shim_gte, . - _fgl_shim_gte

/* (d) Divides.  No C call, so no settle, no PR use, r14 untouched.
 *      entry   r1 = dividend, r0 = divisor
 *      exit    r0 = quotient (LO), r1 = remainder (HI)
 * Out of line for footprint only: the 32-step `div1` sequence is the only
 * lowering the SH7091 has.  The zero divisor is tested here (defined guest
 * results, r3000a.h); the cold path sits first so its branch is a short `bf`.
 *
 * DIVU: d == 0 gives LO = 0xffffffff, HI = n.  The sequence is
 * non-restoring, so after the last step the remainder is one divisor low
 * exactly when T is clear; the quotient is right either way. */
	.align	2
	.globl	_fgl_shim_divu
	.type	_fgl_shim_divu, @function
_fgl_shim_divu:
	mov.l	r3, @-r15

	tst	r0, r0
	bf	1f
	bra	9f
	 mov	#-1, r0			/* LO = -1, HI = n, already in r1 */

1:	mov	#0, r3
	div0u
	.rept	32
	rotcl	r1
	div1	r0, r3
	.endr
	bt	2f
	add	r0, r3			/* the non-restoring correction */
2:	rotcl	r1

	mov	r1, r0			/* quotient  -> LO */
	mov	r3, r1			/* remainder -> HI */

9:	rts
	 mov.l	@r15+, r3
	.size	_fgl_shim_divu, . - _fgl_shim_divu

/* DIV: truncating.  Quotient takes the sign of n ^ d, remainder the sign of
 * n; both are kept in r5/r4 since the core needs positive operands.
 * d == 0 gives LO = (n >= 0 ? -1 : 1), HI = n.  0x80000000 / -1 falls out
 * right without a special case. */
	.align	2
	.globl	_fgl_shim_div
	.type	_fgl_shim_div, @function
_fgl_shim_div:
	mov.l	r3, @-r15
	mov.l	r4, @-r15
	mov.l	r5, @-r15

	tst	r0, r0
	bf	1f
	mov	#-1, r0
	cmp/pz	r1
	bt	9f			/* n >= 0: LO = -1, HI = n */
	bra	9f
	 mov	#1, r0			/* n <  0: LO =  1, HI = n */

1:	mov	r1, r4			/* n,     the remainder's sign */
	mov	r1, r5
	xor	r0, r5			/* n ^ d, the quotient's sign  */

	cmp/pz	r1
	bt	2f
	neg	r1, r1
2:	cmp/pz	r0
	bt	3f
	neg	r0, r0

3:	mov	#0, r3
	div0u
	.rept	32
	rotcl	r1
	div1	r0, r3
	.endr
	bt	4f
	add	r0, r3			/* the non-restoring correction */
4:	rotcl	r1

	cmp/pz	r5
	bt	5f
	neg	r1, r1
5:	cmp/pz	r4
	bt	6f
	neg	r3, r3
6:	mov	r1, r0			/* quotient  -> LO */
	mov	r3, r1			/* remainder -> HI */

9:	mov.l	@r15+, r5
	mov.l	@r15+, r4
	rts
	 mov.l	@r15+, r3
	.size	_fgl_shim_div, . - _fgl_shim_div
