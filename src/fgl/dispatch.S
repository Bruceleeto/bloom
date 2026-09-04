/* fgl's dispatcher: the loop that runs blocks, and the only door to C.
 *
 * This is hand-written and not generated, because it is the one piece of the
 * system that has to know both worlds.  Above it is C, which expects the SH-4
 * ABI and knows nothing of GBR or r13; below it is generated code, which obeys
 * fgl's contract and would be ruined by a compiler's idea of a free register.
 * A C compiler cannot be told to hold guest state in r3-r12 across a call it
 * does not know is happening, so the boundary is written out by hand and the
 * contract is stated once, here, in instructions.
 *
 * WHAT A BLOCK HANDS BACK.  A block does not return.  It ends with
 *
 *      mov.l   @(FGL_AT_CYCLES + n_ops, gbr), r0
 *      sub     r0, r14
 *      mov.l   @(FGL_AT_DISPATCH, gbr), r0
 *      jmp     @r0
 *      nop
 *
 * so it arrives at `fgl_dispatch_loop` with the guest PC it is leaving for in
 * r2 and its cost already subtracted from r14.  Nothing is on the stack and PR
 * is nobody's.  That is why a service routine called from inside a block may
 * use PR freely, and it is why a block can later be linked directly to its
 * successor: there is a jump through a slot to patch, rather than a return.
 *
 * WHAT THE DISPATCHER MAY TOUCH.  r0, r1, r2, T, PR, MACH/MACL.  Nothing else.
 * r3-r12 are the allocator's pool, r13 is the guest address mask, r14 is the
 * cycle delta and r15 is the stack; all of them belong to generated code for
 * the whole of a run.
 *
 * THE ONE TEST.  Every reason a block can stop -- the budget running out, a
 * SYSCALL, a BREAK, an unknown opcode, an interrupt raised from C -- arrives
 * here as the same condition, because both `lightrec_set_exit_flags` and the
 * emitter's own exit path force the cycle delta to zero on their way out.  So
 * the loop tests one register against zero and never decodes `exit_flags`.
 *
 * ON PUBLISHING GUEST REGISTERS, WHICH TODAY IS NOTHING.  `ir_pin[]` in
 * alloc.c is currently all -1: no guest register is held in a host register
 * across a block boundary, so every one of them is already in the state block
 * whenever this code runs, and the crossings to C below have nothing to
 * publish or reload.  When pinning is switched on that stops being true, and
 * the three places it changes are marked PIN below -- deliberately marked
 * rather than left to be found, because a missed one does not crash, it hands
 * C a stale register.
 */

#include "fgl_state.h"

	.text
	.align	2

/* ---------------------------------------------------------------------------
 * s32 fgl_dispatch(struct lightrec_state *state, u32 pc, void *unused,
 *                  s32 cycle_delta);
 *
 * Runs blocks until the budget is spent or something asks for C, and returns
 * what is left of the delta.  The guest PC it stopped at is in `curr_pc`.
 *
 * The third argument is lightrec's "entry point of the first block" and is
 * ignored: looking it up costs one pass round the loop and removes a second
 * way in, which is worth more than the lookup.
 * ------------------------------------------------------------------------- */
	.globl	_fgl_dispatch
	.type	_fgl_dispatch, @function
_fgl_dispatch:
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov.l	r14, @-r15
	sts.l	pr, @-r15
	/* GBR is the SH-4 TLS pointer and is NOT callee-saved.  Nothing in
	 * bloom's image uses TLS -- there is no .tbss and no .tdata -- so this
	 * save is not what makes the scheme work; it is what keeps the
	 * dispatcher honest towards whatever called it. */
	stc	gbr, r0
	mov.l	r0, @-r15

	ldc	r4, gbr				/* the state block */
	mov	r7, r14				/* the cycle delta */
	mov.l	@(FGL_AT_ADDR_MASK * 4, gbr), r0
	mov	r0, r13				/* the guest address mask */
	mov	r5, r2				/* the PC to run */

	/* PIN: preload the pinned guest registers here. */

/* ---------------------------------------------------------------------------
 * The loop.  `state->dispatch` points here, so this label is where every
 * block in the program arrives.
 * ------------------------------------------------------------------------- */
	.globl	_fgl_dispatch_loop
_fgl_dispatch_loop:
	/* Published before the test rather than after it, so that the value C
	 * reads on the way out is the PC we actually stopped at and not the
	 * one before it. */
	mov	r2, r0
	mov.l	r0, @(FGL_AT_CURR_PC * 4, gbr)

	cmp/pl	r14				/* T = (delta > 0) */
	bf	.Lexit

	/* The guest PC into a byte offset in the block table, branchlessly.
	 *
	 * lightrec's `lut_offset()` is a two-case function of bit 28: RAM is
	 * `pc & 0x1fffff`, and the BIOS window is `(pc & 0x7ffff) + 0x200000`.
	 * Both cases agree with
	 *
	 *      (kpc & 0x1fffff) | ((kpc >> 7) & 0x200000)
	 *
	 * because for a BIOS address `kpc & 0x1fffff` already equals
	 * `kpc & 0x7ffff`, and bit 28 shifted down by seven lands exactly on
	 * bit 21, which is 0x200000.  The shift is `shlr8` then `shll`: the
	 * bit that falls off the bottom is one we are about to mask away. */
	mov	r2, r1
	mov.l	.Lkuseg, r0
	and	r0, r1				/* kpc */

	mov	r1, r2
	shlr8	r2
	shll	r2
	mov.l	.Lbiosbit, r0
	and	r0, r2				/* 0, or 0x200000 for BIOS */

	mov.l	.Lrammask, r0
	and	r0, r1
	or	r2, r1				/* byte offset into the table */

	mov.l	@(FGL_AT_LUT * 4, gbr), r0
	mov	r0, r2
	mov	r1, r0
	mov.l	@(r0, r2), r1			/* the slot */

	/* A null slot means the block is not compiled.  bloop removes this
	 * test by seeding every slot with a compile trampoline, which is worth
	 * doing -- but it is two instructions on an edge that bloop's own
	 * census puts at 0.3% of block entries, so it is not what makes this
	 * fast and it is not worth reaching for first. */
	tst	r1, r1
	bt	.Lcompile

	jmp	@r1
	 nop

/* ---------------------------------------------------------------------------
 * Nothing at this PC yet: ask C for it.
 * ------------------------------------------------------------------------- */
.Lcompile:
	bsr	.Lsync_out
	 nop

	stc	gbr, r4				/* arg0: the state */
	mov.l	@(FGL_AT_CURR_PC * 4, gbr), r0
	mov	r0, r5				/* arg1: the guest PC */
	mov.l	.Lgetnext, r0
	jsr	@r0
	 nop
	mov.l	r0, @-r15			/* park the entry point */

	bsr	.Lsync_in
	 nop

	/* C may have run the interpreter, moved the PC, or raised something,
	 * so take the PC back from the state block rather than trusting r2. */
	mov.l	@(FGL_AT_CURR_PC * 4, gbr), r0
	mov	r0, r2

	mov.l	@r15+, r1
	tst	r1, r1
	bt	.Lexit				/* C could not produce a block */

	jmp	@r1
	 nop

/* ---------------------------------------------------------------------------
 * Leaving for good.
 * ------------------------------------------------------------------------- */
.Lexit:
	bsr	.Lsync_out
	 nop

	/* PIN: publish the pinned guest registers here. */

	mov	r14, r1				/* the return value */

	mov.l	@r15+, r0
	ldc	r0, gbr
	lds.l	@r15+, pr
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	rts
	 mov	r1, r0

/* ---------------------------------------------------------------------------
 * The two halves of the cycle contract, which is the thing most likely to be
 * got wrong quietly.
 *
 * While blocks are chaining, the live quantity is the SIGNED DELTA in r14 and
 * the absolute pair in the state block is stale.  C only understands the
 * absolute pair -- and the way C raises an interrupt from underneath a running
 * block is to move `target_cycle` down to meet `current_cycle`, which a
 * register cannot see.  So the delta is spent into the pair before every
 * crossing and rebuilt from the pair after every one.
 *
 * Both are called with `bsr`, which costs PR -- and PR is free here precisely
 * because a block does not own it.
 * ------------------------------------------------------------------------- */
.Lsync_out:
	mov.l	@(FGL_AT_TARGET_CYCLE * 4, gbr), r0
	sub	r14, r0
	rts
	 mov.l	r0, @(FGL_AT_CURRENT_CYCLE * 4, gbr)

.Lsync_in:
	mov.l	@(FGL_AT_CURRENT_CYCLE * 4, gbr), r0
	mov	r0, r1
	mov.l	@(FGL_AT_TARGET_CYCLE * 4, gbr), r0
	sub	r1, r0
	rts
	 mov	r0, r14

	.align	2
.Lkuseg:	.long	0x1fffffff
.Lrammask:	.long	0x001fffff
.Lbiosbit:	.long	0x00200000
/* Labelled so a host harness can point it somewhere else.  The linker fills
 * this in for the real build; the off-hardware test rig has no C to call, so
 * it patches this word to a stub it wrote itself and exercises the same
 * instructions. */
	.globl	_fgl_getnext_slot
_fgl_getnext_slot:
.Lgetnext:	.long	_fgl_get_next_block
