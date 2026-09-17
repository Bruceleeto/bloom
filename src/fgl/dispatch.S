/* fgl's dispatcher: the loop that runs blocks, and the doors to C.
 *
 * A block never returns.  It subtracts its cost from r14 and jumps through
 * FGL_AT_DISPATCH with the guest PC it is leaving for in r2; nothing is on
 * the stack and PR is nobody's.
 *
 * The dispatcher may touch r0, r1, r2, T, PR, MACH/MACL and nothing else:
 * r3-r12 are the allocator's pool, r13 the address mask, r14 the cycle
 * meter, r15 the stack.
 *
 * Every stop reason (budget spent, SYSCALL, BREAK, unknown opcode, interrupt
 * from C) forces r14 to zero, so the loop tests one register and never
 * decodes exit_flags.
 *
 * memset_func, interpreter_func and ds_check_func are entered by address
 * with no return address and rejoin the loop, so they live here.  memset is
 * reached by the loop's ordinary indirect jump: C writes it straight into
 * the block table.  Places that change when pinning changes are marked PIN.
 */

#include "fgl_state.h"
#include "pins.h"

/* Pinned guest registers (pins.h) live in host registers.  PIN_PUBLISH puts
 * them where C looks before a crossing; PIN_LOAD takes them back after one
 * that may have written a guest register (.Lcompile runs the first-pass
 * interpreter, the interpreter door runs guest code, ds_check assigns
 * regs.gpr[reg]).  @(disp,GBR) targets r0 only, which is free here. */
.macro	PIN_PUBLISH
#if FGL_NUM_PINS > 0
	mov	r\()FGL_PIN0_HOST, r0
	mov.l	r0, @(FGL_PIN0_GUEST * 4, gbr)
#endif
#if FGL_NUM_PINS > 1
	mov	r\()FGL_PIN1_HOST, r0
	mov.l	r0, @(FGL_PIN1_GUEST * 4, gbr)
#endif
#if FGL_NUM_PINS > 2
	mov	r\()FGL_PIN2_HOST, r0
	mov.l	r0, @(FGL_PIN2_GUEST * 4, gbr)
#endif
#if FGL_NUM_PINS > 3
	mov	r\()FGL_PIN3_HOST, r0
	mov.l	r0, @(FGL_PIN3_GUEST * 4, gbr)
#endif
#if FGL_NUM_PINS > 4
	mov	r\()FGL_PIN4_HOST, r0
	mov.l	r0, @(FGL_PIN4_GUEST * 4, gbr)
#endif
#if FGL_NUM_PINS > 5
	mov	r\()FGL_PIN5_HOST, r0
	mov.l	r0, @(FGL_PIN5_GUEST * 4, gbr)
#endif
.endm

.macro	PIN_LOAD
#if FGL_NUM_PINS > 0
	mov.l	@(FGL_PIN0_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN0_HOST
#endif
#if FGL_NUM_PINS > 1
	mov.l	@(FGL_PIN1_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN1_HOST
#endif
#if FGL_NUM_PINS > 2
	mov.l	@(FGL_PIN2_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN2_HOST
#endif
#if FGL_NUM_PINS > 3
	mov.l	@(FGL_PIN3_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN3_HOST
#endif
#if FGL_NUM_PINS > 4
	mov.l	@(FGL_PIN4_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN4_HOST
#endif
#if FGL_NUM_PINS > 5
	mov.l	@(FGL_PIN5_GUEST * 4, gbr), r0
	mov	r0, r\()FGL_PIN5_HOST
#endif
.endm

/* FGL_PIN3 is the one pin below r8 and is caller-saved; bracket any C call
 * that does not go through shim.S with these. */
.macro	PIN_CALLER_SAVE
#if FGL_NUM_PINS > 3
	mov.l	r\()FGL_PIN3_HOST, @-r15
#endif
.endm

.macro	PIN_CALLER_RESTORE
#if FGL_NUM_PINS > 3
	mov.l	@r15+, r\()FGL_PIN3_HOST
#endif
.endm

	.text
	.align	2

/* s32 fgl_dispatch(struct lightrec_state *state, u32 pc, void *unused,
 *                  s32 cycle_delta);
 * Runs blocks until the budget is spent or C is needed; returns what is left
 * of the delta.  The stop PC is in curr_pc.  The third argument is ignored. */
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
	/* GBR is the TLS pointer and not callee-saved. */
	stc	gbr, r0
	mov.l	r0, @-r15

	ldc	r4, gbr				/* the state block */
	mov.l	@(FGL_AT_ADDR_MASK * 4, gbr), r0
	mov	r0, r13				/* the guest address mask */

#if FGL_GBASE2
	/* Second guest-file window.  r9 is callee-saved, so one setup covers
	 * the run; the allocator never hands it out (ALLOC_SKIP). */
	stc	gbr, r9
	add	#(FGL_GBASE2_LO * 4), r9	/* $16..$31 live here */
#endif

	/* The fourth argument is cycles; r14 counts guest instructions, so
	 * the budget is built from the pair by .Lsync_in.  The call destroys
	 * r5, hence the park. */
	mov.l	r5, @-r15
	bsr	.Lsync_in
	 nop
	mov.l	@r15+, r2			/* the PC to run */

	PIN_LOAD

	/* Enter below the cycle test: a zero delta on entry means "run exactly
	 * one block" (plugin.c's fastboot stepping), not "do nothing". */
	bra	.Lrun
	 nop

/* The loop.  state->dispatch points here; every block arrives here. */
	.globl	_fgl_dispatch_loop
_fgl_dispatch_loop:
	/* Published before the test so C sees the PC we stopped at. */
	mov	r2, r0
	mov.l	r0, @(FGL_AT_CURR_PC * 4, gbr)

	cmp/pl	r14				/* T = (delta > 0) */
	bf	.Lservice

.Lrun:

#if FGL_BLOCK_COUNT
	/* Counts arrivals here; a patched link never reaches this label. */
	mov.l	.Lblkctr, r1
	mov.l	@r1, r0
	add	#1, r0
	mov.l	r0, @r1
#endif

#if FGL_DEBUG_RING
	/* DEBUG: ring of the last 64 block PCs (-DWITH_FGL_DEBUG_RING=ON).
	 * r2 must survive; r0 and r1 are reloaded below. */
	mov.l	.Lringp, r1
	mov.l	@r1, r0				/* the index */
	add	#1, r0
	and	#63, r0				/* AND #imm is R0-only */
	mov.l	r0, @r1
	shll2	r0
	shll2	r0				/* sixteen bytes per slot */
	add	r0, r1
	mov.l	r2, @(4, r1)			/* slot.pc = pc */

	mov.l	@(3 * 4, gbr), r0		/* $v1 */
	mov.l	r0, @(8, r1)
	/* scratchpad word 0 */
	mov.l	.Lsp0, r0
	mov.l	@r0, r0
	mov.l	r0, @(12, r1)
	mov.l	@(30 * 4, gbr), r0		/* $s8 -- the list pointer */
	mov.l	r0, @(16, r1)
#endif

#if FGL_DEBUG_CP
	/* DEBUG: C checkpoint on every block entry for the cross-engine
	 * comparator.  r8-r15 and GBR survive the call; r3-r7 hold nothing at
	 * a block boundary while nothing is pinned (PIN: publish here). */
	sts.l	pr, @-r15
	mov.l	r2, @-r15
	stc	gbr, r4				/* the state block IS the state */
	mov	r2, r5
	mov.l	.Lcheckpoint, r0
	jsr	@r0
	 nop
	mov.l	@r15+, r2
	lds.l	@r15+, pr
#if FGL_GBASE
	stc	gbr, r3
#endif
#endif

	/* Guest PC to block-table byte offset, branchless:
	 *   (kpc & 0x1fffff) | ((kpc >> 7) & 0x200000)
	 * agrees with lightrec's lut_offset() for both RAM and BIOS. */
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

	/* A null slot means not compiled. */
	tst	r1, r1
	bt	.Lcompile

	jmp	@r1
	 nop

/* Not compiled: ask C.  Also an address: the threaded compiler seeds slots
 * with state->get_next_block to mean "seen, not compiled", and the loop
 * jumps to whatever it finds. */
	.globl	_fgl_dispatch_compile
_fgl_dispatch_compile:
.Lcompile:
	/* get_next_block interprets the block first (ENABLE_FIRST_PASS).
	 * Published before the settle: .Lsync_out calls C and r7 is
	 * caller-saved. */
	PIN_PUBLISH

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

	/* C may have moved the PC; take it from the state block. */
	mov.l	@(FGL_AT_CURR_PC * 4, gbr), r0
	mov	r0, r2

	PIN_LOAD

	mov.l	@r15+, r1
	tst	r1, r1
	bt	.Lexit				/* C could not produce a block */

	jmp	@r1
	 nop

/* The slice is spent: ask C for another one without leaving.  r8-r15 and
 * GBR survive the call.  The pair is settled and the pins published first
 * because the scheduler can raise an exception; the budget comes back from
 * the pair since C sized the next slice. */
.Lservice:
	/* IR_EXIT zeroes r14 and parks the unspent meter in FGL_AT_EXIT_METER;
	 * take that instead, and stamp it out once read. */
	mov.l	@(FGL_AT_EXIT_METER * 4, gbr), r0
	mov	#1, r1
	rotr	r1				/* 0x80000000, no literal */
	cmp/eq	r1, r0
	bt	1f
	mov	r0, r14				/* the parked, unspent meter */
	mov	r1, r0
	mov.l	r0, @(FGL_AT_EXIT_METER * 4, gbr)
1:
	PIN_PUBLISH

	bsr	.Lsync_out
	 nop

	stc	gbr, r4				/* arg0: the state */
	mov	r14, r5				/* arg1: the meter we spent */
	mov.l	.Lservicefn, r0
	jsr	@r0
	 nop

	tst	r0, r0
	bt	.Lexit_published		/* no more slices: leave */

	bsr	.Lsync_in
	 nop
	mov	r0, r14				/* the fresh budget */

	PIN_LOAD

	/* C may have moved the PC (interrupt vector); never trust r2. */
	mov.l	@(FGL_AT_CURR_PC * 4, gbr), r0
	bra	.Lrun
	 mov	r0, r2

/* Leaving for good. */
.Lexit:
	PIN_PUBLISH

	bsr	.Lsync_out
	 nop

.Lexit_published:
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

/* memset_func: lightrec puts this address in the block table for a block it
 * recognises as a memset loop, so it is entered as a block, not a service.
 * lightrec_memset(state) reads $a0/$a1 and returns the cost in guest cycles.
 * r14 counts instructions, so the charge goes onto current_cycle directly;
 * it is additive with the settle.  No cycle sync: the call cannot move the
 * pair or raise an interrupt, and r14 is callee-saved.  The replaced block
 * ended in jr $ra. */
	.globl	_fgl_dispatch_memset
_fgl_dispatch_memset:
	/* lightrec_memset reads $a0 and $a1. */
	PIN_PUBLISH

	stc	gbr, r4				/* arg0: the state */
	mov.l	.Lmemset, r0
	jsr	@r0
	 nop
	mov	r0, r1				/* the cost, in guest cycles */
	mov.l	@(FGL_AT_CURRENT_CYCLE * 4, gbr), r0
	add	r1, r0
	mov.l	r0, @(FGL_AT_CURRENT_CYCLE * 4, gbr)

#if FGL_GBASE
	/* r3 is caller-saved; this door skips .Lsync_in. */
	stc	gbr, r3
#endif

	/* Not for stale values: FGL_PIN3_HOST is r7, caller-saved, and this
	 * door did not save it. */
	PIN_LOAD

	/* PIN: if $ra is ever pinned, read the pinned copy. */
	mov.l	@(31 * 4, gbr), r0		/* guest $ra */
	bra	_fgl_dispatch_loop
	 mov	r0, r2

/* interpreter_func: a branch this compiler will not touch.
 * Entry: r2 = the branch's guest PC, r1 = struct block *.
 * lightrec_emulate_block(state, block, pc) returns the PC it stopped at and
 * spends cycles, so both halves of the cycle contract apply.  The PC is the
 * return value, not curr_pc. */
	.globl	_fgl_dispatch_interpreter
_fgl_dispatch_interpreter:
	PIN_PUBLISH

	bsr	.Lsync_out
	 nop

	mov	r2, r6				/* arg2: the branch's PC */
	mov	r1, r5				/* arg1: the block */
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Lemulate, r0
	jsr	@r0
	 nop
	mov	r0, r2				/* where the interpreter stopped */

	bsr	.Lsync_in
	 nop

	PIN_LOAD

	bra	_fgl_dispatch_loop
	 nop

/* ds_check_func: a load in a branch's delay slot whose write-back depends on
 * whether the first opcode at the target reads the register.
 * Entry: r2 = branch target, r1 = guest register number.
 * lightrec_check_load_delay(state, pc, reg) returns the PC to run; its slow
 * path can compile, interpret or fail (pulling target_cycle down), so both
 * halves of the cycle contract apply. */
	.globl	_fgl_dispatch_ds_check
_fgl_dispatch_ds_check:
	PIN_PUBLISH

	bsr	.Lsync_out
	 nop

	mov	r1, r6				/* arg2: the guest register */
	mov	r2, r5				/* arg1: the branch target */
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Ldscheck, r0
	jsr	@r0
	 nop
	mov	r0, r2				/* the PC C decided on */

	bsr	.Lsync_in
	 nop

	/* C may have written a pinned register. */
	PIN_LOAD

	bra	_fgl_dispatch_loop
	 nop

/* fgl_link_stub: the dispatch a linked edge uses once.  A block with a
 * constant successor ends in
 *      cmp/pl  r14
 *      bf      .Lout
 *      [nop]                           pad so the site is four-aligned
 *  site+0  mov.l @(FGL_AT_LINK, gbr), r0
 *  site+2  jsr   @r0
 *  site+4  nop                         PR is site+6
 * and arrives with the target PC in r2.  fgl_link_resolve rewrites
 * site+0..3 into `bra target; nop` when it can.  The budget test lives in
 * the tail because a linked edge never passes the loop.  C clobbers r0-r7;
 * only r7 (a pin) holds anything at a block tail, so it is saved by hand.
 * Nothing is published: the callee touches no guest state. */
	.globl	_fgl_link_stub
_fgl_link_stub:
	sts	pr, r0
	add	#-6, r0				/* the call site */
	mov.l	r0, @-r15
	mov.l	r2, @-r15			/* the target guest PC */
#if FGL_NUM_PINS > 3
	PIN_CALLER_SAVE				/* pinned and caller-saved */
	mov.l	@(4, r15), r5			/* arg1: the target */
	mov.l	@(8, r15), r6			/* arg2: the site */
#else
	mov.l	@r15, r5			/* arg1: the target */
	mov.l	@(4, r15), r6			/* arg2: the site */
#endif
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Llinkres, r0
	jsr	@r0
	 nop
	mov	r0, r1				/* where to go, or 0 */
#if FGL_GBASE
	stc	gbr, r3				/* clobbered by C; see pins.h */
#endif
	PIN_CALLER_RESTORE
	mov.l	@r15+, r2			/* the exit PC, untouched */
	add	#4, r15				/* drop the site */

	tst	r1, r1
	bt	.Llink_slow
	jmp	@r1
	 nop
.Llink_slow:
	bra	_fgl_dispatch_loop
	 nop

/* The two halves of the cycle contract.  While blocks chain, the live
 * quantity is the meter in r14 and the absolute pair is stale; C raises an
 * interrupt by moving target_cycle, which a register cannot see.  So the
 * meter is spent into the pair before every crossing and rebuilt after.
 * Both call C (fgl_lightrec.c), so callers publish pins first; PR and r2
 * are saved here rather than at each call site. */
.Lsync_out:
	sts.l	pr, @-r15
	mov.l	r2, @-r15
	stc	gbr, r4				/* arg0: the state */
	mov	r14, r5				/* arg1: the meter unspent */
	mov.l	.Lcycout, r0
	jsr	@r0
	 nop
	mov.l	@r15+, r2
	lds.l	@r15+, pr
#if FGL_GBASE
	stc	gbr, r3
#endif
	rts
	 nop

.Lsync_in:
	sts.l	pr, @-r15
	mov.l	r2, @-r15
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Lcycin, r0
	jsr	@r0
	 nop
	mov.l	@r15+, r2
	lds.l	@r15+, pr
#if FGL_GBASE
	/* r3 is caller-saved; the value is gbr. */
	stc	gbr, r3
#endif
	rts
	 mov	r0, r14

	.align	2
#if FGL_BLOCK_COUNT
.Lblkctr:	.long	_fgl_blocks_run
#endif
#if FGL_DEBUG_RING
.Lringp:	.long	_fgl_pc_ring
.Lsp0:		.long	0x1f800000
#endif
#if FGL_DEBUG_CP
.Lcheckpoint:	.long	_fgl_checkpoint
#endif
.Lkuseg:	.long	0x1fffffff
.Lrammask:	.long	0x001fffff
.Lbiosbit:	.long	0x00200000
/* Labelled so a host harness can patch it. */
	.globl	_fgl_getnext_slot
_fgl_getnext_slot:
.Lgetnext:	.long	_fgl_get_next_block

	.align	2
.Lservicefn:	.long	_fgl_service_events
.Lcycout:	.long	_fgl_cycles_out
.Lcycin:	.long	_fgl_cycles_in

/* fgl-side names: lightrec_memset and lightrec_check_load_delay are static. */
	.globl	_fgl_memset_slot
_fgl_memset_slot:
.Lmemset:	.long	_fgl_memset

	.globl	_fgl_emulate_slot
_fgl_emulate_slot:
.Lemulate:	.long	_fgl_emulate_block

	.globl	_fgl_dscheck_slot
_fgl_dscheck_slot:
.Ldscheck:	.long	_fgl_check_load_delay

	.globl	_fgl_linkres_slot
_fgl_linkres_slot:
.Llinkres:	.long	_fgl_link_resolve
