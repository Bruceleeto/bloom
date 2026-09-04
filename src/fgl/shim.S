/*
 * fgl's service shims -- the only way generated code reaches anything it did
 * not emit itself.
 *
 * A block runs with no ABI at all (see fgl.h): r0 is the transfer register,
 * r1 the second scratch, r2 the exit PC, r3-r12 the pinned guest pool, r13
 * the guest address mask, r14 the cycle delta, GBR the state block.  A
 * compiled C function knows none of that and would destroy most of it, so
 * nothing compiled is callable from a block directly.  Everything here is the
 * layer in between: hand-written SH-4 that speaks fgl's contract on one side
 * and the SH-4 C ABI on the other.
 *
 * WHAT THE ABI ALREADY GIVES US, WHICH IS MOST OF IT.  r8-r15 are
 * callee-saved on this machine.  That covers r8-r12 of the guest pool, the
 * mask in r13, the cycle delta in r14 and the stack -- a compiled callee
 * gives all of them back on its own.  The exposure is r2-r7 and PR, six
 * general registers and one control register, and that is the whole of what
 * the prologue below saves.  This was checked against the tree rather than
 * taken from a document: gte_rtp.S's own prologues push r8-r14 and never
 * touch r2-r7.
 *
 * THE SAVE GOES ON THE STACK, NOT INTO THE STATE BLOCK.  GBR addressing
 * exists only for r0, so spilling any other register through it costs a `mov`
 * either side -- two instructions each way against one.  And a fixed shim
 * could not spill to the state block correctly even if it were free: the
 * allocator tracks per-node which guest registers are dirty (alloc.h), this
 * code has no access to that model, and an unconditional writeback would
 * publish values the allocator still believes are unwritten.
 *
 * PR IS FREE.  A block does not return -- it ends by jumping through
 * FGL_AT_DISPATCH (fgl_state.h) -- so nothing in a block owns PR and a shim
 * may `jsr` with it.  The block reaches a shim with `jsr` for the same
 * reason, and a shim that itself calls C saves PR across that inner call.
 *
 * NO FPU STATE IS SAVED, AND THAT IS DELIBERATE RATHER THAN AN OVERSIGHT.
 * Generated code touches no floating-point register at all -- fgl lowers
 * nothing to the FPU -- so there is nothing of the block's to preserve.  More
 * importantly gte_fpu.c owns the back float bank (xf0-xf15) as a residency
 * cache for the rotation matrix, so `frchg` here would silently invalidate
 * that cache and the GTE would start reading someone else's matrix.  NOTHING
 * IN THIS FILE MAY EXECUTE `frchg`, now or later.
 *
 * GBR IS NOT RELOADED, AND THAT IS A PROPERTY OF THE LINK.  GBR is the SH-4
 * TLS thread pointer and is NOT callee-saved; fgl's use of it survives a C
 * call only because nothing in the linked image uses TLS -- bloom.elf has no
 * .tbss and no .tdata, and newlib's errno goes through a function reading
 * __impure_ptr rather than through GBR.  That is checked at link time, not
 * assumed; see build/docs/fgl_shim.md.  If it ever stops being true, every
 * shim here that calls C needs GBR restored from a literal after the call,
 * at two instructions.
 *
 * THE T BIT IS NOT SAVED.  It is caller-saved in the C ABI and appears in no
 * contract here.  fgl is safe today by construction -- IR_COND emits its
 * compare and its branch adjacently, so no T value is ever live across a
 * node, let alone across a service.  bloop shipped a real bug from exactly
 * this shape, so if fgl ever separates a compare from its branch, the shims
 * that call C grow a `movt`/`shal` pair.
 *
 * MACH/MACL ARE NOT SAVED for the same kind of reason: emit_muldiv issues its
 * `dmuls.l` and its `sts` inside one node, so the MAC registers are never
 * live across anything.
 */

#include "fgl_state.h"

	.text
	.align	2

/*
 * The prologue and epilogue every C-calling shim shares: r2-r7 and PR, seven
 * words, the exact set the ABI does not cover.  r2 goes deepest because it is
 * the one that has to come back in the `rts` delay slot -- the epilogue is
 * eight instructions with the last pop free rather than nine.
 */
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

/*
 * THE CYCLE RECONCILIATION, WHICH IS NOT OPTIONAL.
 *
 * While blocks are chaining, the live quantity is the signed delta
 * `target_cycle - current_cycle` in r14; the absolute pair in the state block
 * is stale and only the exit paths care.  That is fine until C runs, because
 * the C side's entire way of raising an interrupt from inside a block is
 * `lightrec_set_exit_flags`, which ORs a flag and then sets
 * `target_cycle = current_cycle` -- IN MEMORY, AND NOWHERE ELSE.  A delta
 * sitting in a register cannot see that move, so the dispatcher's `cmp/pl
 * r14` would stay true and the machine would keep running blocks: the
 * interrupt arrives late rather than never, quietly, and only under load.
 *
 * So the pair is made current before the call and the delta is rebuilt from
 * it afterwards, which is exactly what lightrec's own wrapper does either
 * side of a C call (update_cycle_counter_before_c / _after_c).  Three
 * instructions out, four back, paid only on a service call.
 *
 * THIS IS THE ONE EXCEPTION TO "A SERVICE MAY CLOBBER r0 AND r1 AND NOTHING
 * ELSE": the shims in this file, and nothing else anywhere, may write r14.  A
 * service that makes no C call -- the divides below -- neither needs this nor
 * is allowed it.
 */
.macro	SHIM_CYCLES_OUT
	mov.l	@(FGL_AT_TARGET_CYCLE * 4, gbr), r0
	sub	r14, r0			/* current = target - delta */
	mov.l	r0, @(FGL_AT_CURRENT_CYCLE * 4, gbr)
.endm

/*
 * And back.  Built with `neg`/`add` rather than the obvious pair of loads and
 * a `sub` because both loads must land in r0 -- @(disp,GBR) has no other
 * destination -- so the second one would otherwise need a third register to
 * survive.  Four instructions, and r1 is left alone so a caller that has a
 * return value to protect can park it there.
 */
.macro	SHIM_CYCLES_IN
	mov.l	@(FGL_AT_CURRENT_CYCLE * 4, gbr), r0
	neg	r0, r14
	mov.l	@(FGL_AT_TARGET_CYCLE * 4, gbr), r0
	add	r0, r14			/* delta = target - current */
.endm

/*
 * ----------------------------------------------------------------
 * (a) The general C-ABI service.
 * ----------------------------------------------------------------
 *
 *      entry   r0 = the callee, r1 = its first argument
 *      exit    r0 = whatever the callee returned
 *
 * Two values arrive because two are all there is room for: r0 and r1 are the
 * only registers a block may hand over, everything from r2 up being spoken
 * for.  The callee is given `r4 = the argument` and `r5 = the state pointer`,
 * which covers both the one-argument shape and the `f(value, state)` shape
 * the lightrec_hw_* family uses -- an extra argument register is invisible to
 * a function that does not name it, the same ABI fact psxCP2[]'s
 * single-argument entries already rely on.  A service wanting a different
 * shape gets its own shim; it is eight lines.
 *
 * The callee lands in r7 rather than being called out of r0 because r0 is
 * needed by the reconciliation above, and r7 is already saved by the time we
 * want somewhere to put it.
 */
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

/*
 * ----------------------------------------------------------------
 * (b) The hardware stores, which need a third value.
 * ----------------------------------------------------------------
 *
 *      entry   r0 = the callee, r1 = the address,
 *              FGL_AT_SHIM_ARG = the value to store
 *      exit    nothing -- lightrec_hw_sb/_sh/_sw all return void
 *
 * `lightrec_hw_sb(addr, val, state)` takes three arguments and the block has
 * two registers to give: r0 and r1, everything from r2 up being spoken for.
 * The state block is not one of the three that has to travel -- it is GBR and
 * this shim reads it -- but the address and the value both are, and that is
 * already one more than there is room for.  So the value goes through the
 * state block instead of through a register: the call site parks it in
 * FGL_AT_SHIM_ARG, one store, and the load below is one instruction against
 * the register there is no way to free.  fgl_state.h says why that word is
 * its own and not `temp_reg`.
 *
 * THE LOAD MUST FOLLOW THE RECONCILIATION, not precede it.  @(disp,GBR) has
 * no destination but r0, and SHIM_CYCLES_OUT wants r0 for the counters --
 * loading the value first would hand the callee the target cycle.
 *
 * The callee lands in r7 for the same reason it does above: r0 is needed by
 * the reconciliation, and r7 is saved by the time we want somewhere to put
 * it.  This calls C, so it reconciles in both directions like any other shim
 * that does; there is no return value to park across the way back.
 */
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

/*
 * ----------------------------------------------------------------
 * (c) The GTE.
 * ----------------------------------------------------------------
 *
 *      entry   r0 = the callee, r1 = the guest COP2 command word
 *      exit    nothing -- every GTE body returns void
 *
 * `gte_fpu_resolve(op)` picks the body at COMPILE time, so there is no
 * runtime dispatch: the emitter has already turned the command word into a
 * function pointer and both are literals at the call site.  Every body it can
 * return takes the same shape, `f(psxCP2Regs *, u32 op)` -- the five hand
 * written fast paths (gte_rtps_fast, gte_rtpt_fast, gte_mvmva_fast,
 * gte_dpcs_fast, gte_intpl_fast) and the twenty-odd C ones alike -- so one
 * shim serves all of them.
 *
 * The first argument is free to compute here rather than to pass.
 * `psxCP2Regs` is the contiguous `{ CP2D[32]; CP2C[32] }` that FGL_AT_CP2D
 * and FGL_AT_CP2C already describe, so a pointer to it is just GBR + 264 --
 * which is why only the callee and the op word have to arrive in registers,
 * and why they fit in the two the contract leaves free.
 *
 * 264 does not fit `add #imm`, so it comes from a halfword literal.  That
 * literal load is free: it fills the `jsr` delay slot, where the pointer
 * arithmetic then goes.
 */
	.align	2
	.globl	_fgl_shim_gte
	.type	_fgl_shim_gte, @function
_fgl_shim_gte:
	SHIM_ENTER
	mov	r0, r7			/* the callee, out of the way */
	mov	r1, r5			/* argument two: the command word */
	SHIM_CYCLES_OUT
	mov.w	1f, r0
	stc	gbr, r4
	jsr	@r7
	 add	r0, r4			/* argument one: &regs.cp2d */
	SHIM_CYCLES_IN
	SHIM_LEAVE
	.align	2
1:	.word	FGL_AT_CP2D * 4
	.size	_fgl_shim_gte, . - _fgl_shim_gte

/*
 * ----------------------------------------------------------------
 * (d) The divides.
 * ----------------------------------------------------------------
 *
 *      entry   r1 = the dividend, r0 = the divisor
 *      exit    r0 = the quotient (guest LO), r1 = the remainder (guest HI)
 *
 * NOTHING HERE CALLS C, so nothing here reconciles the cycle counters, does
 * not touch PR beyond the `rts` it was entered for, and may not write r14.
 * It is a service in the ordinary sense of fgl.h -- r0 and r1 and nothing
 * else -- and the three registers its body wants beyond those are saved and
 * given back.
 *
 * WHY THIS EXISTS AT ALL IS FOOTPRINT, NOT SPEED.  The SH7091 has no divide
 * unit -- searching the tree and the whole KOS install for dvdnt, DVSR, DVCR
 * and 0xff000000 returns nothing -- so the 32-step `div1` sequence is the
 * only lowering there is, and out of line it runs exactly the same
 * instructions it ran inline.  What changes is that it stops being 77 emitted
 * words for DIVU and 98 for DIV at every division in the program and becomes
 * a three-instruction call.  Pipeline freeze by icache miss is 12.5 ms of a
 * 66.8 ms frame on hardware; that is the whole argument.
 *
 * THE ZERO DIVISOR IS TESTED HERE AND NOT AT THE CALL SITE.  A divide by zero
 * on this guest is not a trap, it has defined results (r3000a.h) and they
 * have to be produced.  fgl's inline path skips the test when the optimiser
 * has proved the divisor non-zero (FGL_H_NO_DIV_CHK); out of line that proof
 * is worth two instructions on a body of a hundred, so there is no
 * check-free variant and the hint is simply ignored for the call.  The cold
 * path is placed FIRST rather than after the core so its branch is a short
 * `bf` over a `bra` -- a `bt` forward over the whole core is within `bt`'s
 * 254-byte reach today and would silently stop being so if the core ever
 * grew.
 */

/*
 * DIVU.  d == 0 gives LO = 0xffffffff and HI = the dividend, which is already
 * where it needs to be.
 *
 * The core itself: `div0u` clears M, Q and T, then each `rotcl`/`div1` pair
 * shifts one bit of the dividend into the partial remainder and one quotient
 * bit back in at the bottom, the remainder's 33rd bit living in Q.  After 32
 * pairs the dividend register holds the quotient bar its last bit, which the
 * trailing `rotcl` supplies.
 *
 * THE SEQUENCE IS NON-RESTORING, so a step that subtracts too much is not
 * undone -- the next step compensates by adding.  After the last step there
 * is no next one, so the remainder is left one divisor low exactly when that
 * last subtraction failed; T already IS that answer, because it is also the
 * quotient's last bit.  The quotient comes out right either way, which is why
 * this correction is easy to leave out and hard to notice: only code that
 * reads HI ever sees it.  Neither `bt` nor `add` writes T, so the trailing
 * `rotcl` still gets the bit it came for.
 *
 * There is no shorter form.  A loop would have to carry T across the back
 * edge and every loop-control instruction on this machine writes it.
 */
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

/*
 * DIV.  The guest's rule is C's truncating division and not a floor: the
 * quotient takes the sign of the operands' exclusive or and the remainder the
 * sign of the dividend.  Neither sign survives the core -- both operands are
 * made positive going in -- so both are computed first and kept, which is
 * what the two registers beyond the remainder are for.
 *
 * d == 0 gives LO = (n >= 0 ? 0xffffffff : 1) and HI = n.
 *
 * 0x80000000 / -1 needs no code and gets none.  It is defined as LO =
 * 0x80000000, HI = 0, and that is what falls out: |n| is 0x80000000, |d| is
 * 1, the core divides them to 0x80000000 remainder 0, and the quotient's sign
 * word n ^ d has its top bit clear so nothing is negated.
 */
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
