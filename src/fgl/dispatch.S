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
 * THE OTHER THREE DOORS.  lightrec's dispatcher is not one entry point but
 * four, and the other three are addresses stored in state fields that
 * generated code and plain C both jump to with no return address:
 *
 *      memset_func       a block recognised as a pure memset loop.  This one
 *                        is not reached by an emitted jump at all -- C writes
 *                        the address straight into the block table
 *                        (lightrec.c:759, :1473), so the loop's ordinary
 *                        indirect jump enters it thinking it is compiled code.
 *      interpreter_func  a branch that cannot be compiled and has to be run by
 *                        lightrec's own interpreter.
 *      ds_check_func     a branch with a load in its delay slot, where whether
 *                        the target reads the loaded register decides what
 *                        happens next.
 *
 * All three end by rejoining the loop rather than returning, which is why they
 * live here and not in shim.S: they are the loop, entered sideways.
 *
 * ON PUBLISHING GUEST REGISTERS, WHICH TODAY IS NOTHING.  `ir_pin[]` in
 * alloc.c is currently all -1: no guest register is held in a host register
 * across a block boundary, so every one of them is already in the state block
 * whenever this code runs, and the crossings to C below have nothing to
 * publish or reload.  When pinning is switched on that stops being true, and
 * the places it changes are marked PIN below -- deliberately marked rather
 * than left to be found, because a missed one does not crash, it hands C a
 * stale register.  Two of the marks below are not about C's view in general
 * but about named registers: `lightrec_memset` reads the guest's $a0, $a1 and
 * $ra, so those three must be in the state block before it runs and $ra must
 * be re-read after.
 */

#include "fgl_state.h"
#include "pins.h"

/* THE PINNED REGISTERS, PUBLISHED AND RELOADED.
 *
 * Generated code holds the guest registers named in pins.h in host registers
 * and does not write them back per block -- that is what pinning is.  C reads
 * and writes them through the state block, so every crossing here has to put
 * them where C looks and take back what C left.
 *
 * Two instructions each way per pin: `@(disp,GBR)` has no destination but r0,
 * and r0 is free at all of these sites because the dispatcher owns it.
 *
 * PIN_PUBLISH before a crossing; PIN_LOAD after one that could have WRITTEN a
 * guest register, and at the entry from C where the registers are filled for
 * the first time.  Which crossings write is not a guess: `.Lcompile` runs the
 * first-pass interpreter, `_fgl_dispatch_interpreter` runs guest code, and
 * `_fgl_dispatch_ds_check` assigns `regs.gpr[reg]` (lightrec.c:971).  The
 * memset service reads $a0/$a1 and writes neither. */
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

/* The one pin below r8, saved and restored by hand around a C call.
 *
 * r8-r15 are callee-saved in the SH-4 ABI, so five of the six pins survive a
 * compiled call on their own.  FGL_PIN3 does not -- see pins.h on why it is
 * there anyway -- so anything here that calls C without going through shim.S
 * has to bracket the call with these. */
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

	/* Filled for the first time; from here they live in registers. */
	PIN_LOAD

	/* Into the loop BELOW the cycle test.
	 *
	 * A zero delta on the way in is not "do nothing", it is the stepping
	 * contract: plugin.c's early-boot path (`lightrec_plugin_execute_block`
	 * -> `execute_internal(block_only = true)`) sets `cycles_pcsx = 0`
	 * deliberately, meaning RUN EXACTLY ONE BLOCK AND COME BACK, so that
	 * the 0x80030000 fastboot hack can step the BIOS a block at a time.
	 *
	 * Lightning honoured that by construction rather than by intent: its
	 * test sat at `loop2`, the label a block RETURNS to, and the entry path
	 * fell straight into `jmpr` -- so one block always ran before anything
	 * was tested.  This loop's head is both the entry and the return point,
	 * so testing there refuses to run the block, the PC never moves, and
	 * the caller steps the same address forever.
	 *
	 * Hence one branch: come in past the test, leave it where blocks land. */
	bra	.Lrun
	 nop

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

.Lrun:

#if FGL_DEBUG_RING
	/* DEBUG: a ring of the last 64 guest block PCs.
	 *
	 * A fault inside emitted code names the block it happened in and
	 * nothing about how the machine got there, which is useless when the
	 * bug is that the block should never have run.  Eight instructions
	 * here buy the predecessor chain.
	 *
	 * Off by default -- this is the hottest path in the emulator, and the
	 * ring costs four loads and four stores on every block entry.  Build
	 * with -DWITH_FGL_DEBUG_RING=ON to get it back.
	 *
	 * r2 is the PC and must survive; r0 and r1 are both reloaded from
	 * scratch immediately below, so they are free to clobber. */
	mov.l	.Lringp, r1
	mov.l	@r1, r0				/* the index */
	add	#1, r0
	and	#63, r0				/* AND #imm is R0-only */
	mov.l	r0, @r1
	shll2	r0
	shll2	r0				/* sixteen bytes per slot */
	add	r0, r1
	mov.l	r2, @(4, r1)			/* slot.pc = pc */

	/* And the three guest registers this hunt needs with it.  Nothing is
	 * pinned yet, so every register is in the state block at a block
	 * boundary and these are the true values the next block starts from.
	 * `mov.l @(disp,GBR)` can only target r0, which is free by now. */
	mov.l	@(3 * 4, gbr), r0		/* $v1 */
	mov.l	r0, @(8, r1)
	/* DEBUG: guest scratchpad word 0, the list head this hunt is about.
	 * Recording it per block entry names the block boundary it changed
	 * at, and the ring's predecessors then name the writer. */
	mov.l	.Lsp0, r0
	mov.l	@r0, r0
	mov.l	r0, @(12, r1)
	mov.l	@(30 * 4, gbr), r0		/* $s8 -- the list pointer */
	mov.l	r0, @(16, r1)
#endif

#if FGL_DEBUG_CP
	/* DEBUG: a C call on EVERY block entry, for the cross-engine
	 * comparator.
	 *
	 * The comparator needs the two engines sampled at the same guest
	 * moments.  lightrec's interpreter hashes state once per block, from
	 * inside `lightrec_run_interpreter`; the fgl side had no equivalent
	 * hook and was sampled from `fgl_lockstep` instead, which C enters
	 * once per DISPATCH, not once per block.  So one log was every block
	 * and the other a sparse sample of outer-loop entries, and matching
	 * them as a subsequence reported a divergence that was only the two
	 * granularities disagreeing.  This is the missing hook: `.Lrun` is
	 * where every block in the program arrives, with r2 the guest PC.
	 *
	 * The dispatcher's budget is r0/r1/r2/T/PR, and a C call takes more
	 * than that -- but r8-r15 are callee-saved by the SH-4 ABI, so r13
	 * and r14 come back untouched, GBR is never written by code that has
	 * no TLS, and r3-r7 hold nothing at a block boundary while `ir_pin[]`
	 * is all -1.  PIN: when pinning is switched on, publish here.
	 * Only r2 has to be saved by hand.
	 *
	 * Ruinously slow, and meant to be: it is a debug build. */
	sts.l	pr, @-r15
	mov.l	r2, @-r15
	stc	gbr, r4				/* the state block IS the state */
	mov	r2, r5
	mov.l	.Lcheckpoint, r0
	jsr	@r0
	 nop
	mov.l	@r15+, r2
	lds.l	@r15+, pr
#endif

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
/* Also reachable as an ADDRESS, not only as a branch.
 *
 * Under the threaded compiler `lightrec_precompile_block` seeds a table slot
 * with `state->get_next_block` to mean "seen, not compiled yet" -- a value the
 * C side recognises and steps over.  The loop above cannot: it tests the slot
 * for zero and jumps to whatever else it finds.  So the sentinel has to be a
 * real entry point that does the right thing when jumped to, and this is it. */
	.globl	_fgl_dispatch_compile
_fgl_dispatch_compile:
.Lcompile:
	bsr	.Lsync_out
	 nop

	/* ENABLE_FIRST_PASS makes `get_next_block` INTERPRET the block before
	 * compiling it, so this crossing reads and writes guest registers like
	 * any other run of guest code -- it is not merely the compiler. */
	PIN_PUBLISH

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

	PIN_LOAD

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

	PIN_PUBLISH

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
 * memset_func: a block that is a memset loop, done in one call.
 *
 * lightrec spots `for (i = 0; i < a1; i++) a0[i] = 0` and refuses to compile
 * it, putting THIS ADDRESS in the block table where the block's code would
 * have gone.  So there is no emitted jump to find: the loop above reaches here
 * through `jmp @r1` believing it has found compiled code, which is why this
 * has to obey a block's entry contract exactly and not a service routine's.
 *
 * `lightrec_memset(state)` reads $a0 and $a1 out of the state block, zeroes
 * the guest memory, and RETURNS THE COST IN GUEST CYCLES -- a length-dependent
 * charge that no cycle-table entry could have predicted, which is the whole
 * reason this is a call and not emitted code.
 *
 * NO CYCLE SYNC, WHICH IS DELIBERATE AND IS NOT AN OVERSIGHT.  The other two
 * doors below cross into C that reads and moves the absolute pair; this one
 * does not.  `lightrec_memset` touches neither counter and cannot raise an
 * interrupt -- it hands its cost back as a return value instead -- so the
 * delta in r14 never stops being the live quantity and there is nothing to
 * reconcile.  r14 survives the call on its own: r8-r15 are callee-saved in the
 * SH-4 ABI, so the charge is one `sub` against the register that is still
 * holding the budget.
 *
 * The block being replaced was a leaf loop ending in `jr $ra`, so where it
 * goes next is the guest's $ra and nowhere else.
 * ------------------------------------------------------------------------- */
	.globl	_fgl_dispatch_memset
_fgl_dispatch_memset:
	/* $a0 and $a1 are what `lightrec_memset` reads (lightrec.c:941).  It
	 * writes no guest register, so there is no reload. */
	PIN_PUBLISH

	stc	gbr, r4				/* arg0: the state */
	mov.l	.Lmemset, r0
	jsr	@r0
	 nop
	sub	r0, r14				/* charge what the memset cost */

	/* PIN: $ra is a guest register like any other; if it is ever pinned
	 * this must read the pinned copy rather than the state block. */
	mov.l	@(31 * 4, gbr), r0		/* guest $ra */
	bra	_fgl_dispatch_loop
	 mov	r0, r2

/* ---------------------------------------------------------------------------
 * interpreter_func: a branch this compiler will not touch.
 *
 * The block jumps here having put the guest PC of the offending branch in r2
 * -- where a block always puts the PC it is leaving for -- and the `struct
 * block *` it belongs to in r1.  r1 rather than r0 because r0 is the transfer
 * register and the block had to load the address of this routine through it;
 * r2 rather than a fresh register because the exit PC is already the thing r2
 * means, so nothing new joins the contract.
 *
 * `lightrec_emulate_block(state, block, pc)` runs guest instructions in C and
 * returns the PC it stopped at.  It spends guest cycles while it does, so both
 * halves of the contract are needed: the delta must be spent into the pair
 * before it runs because that is the only place C can see it, and rebuilt
 * afterwards because the interpreter moved `current_cycle` and may have moved
 * `target_cycle` down underneath us to raise an interrupt.
 *
 * Unlike `.Lcompile`, the PC is NOT re-read from the state block: C returned
 * it, and that return value is the answer even when the state block's
 * `curr_pc` is still the branch we came in with.
 * ------------------------------------------------------------------------- */
	.globl	_fgl_dispatch_interpreter
_fgl_dispatch_interpreter:
	bsr	.Lsync_out
	 nop

	/* The interpreter reads and writes every one of them. */
	PIN_PUBLISH

	mov	r2, r6				/* arg2: the branch's PC */
	mov	r1, r5				/* arg1: the block */
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Lemulate, r0
	jsr	@r0
	 nop
	/* r2 is safe across `.Lsync_in`, which owns r0 and r1 only. */
	mov	r0, r2				/* where the interpreter stopped */

	bsr	.Lsync_in
	 nop

	PIN_LOAD

	bra	_fgl_dispatch_loop
	 nop

/* ---------------------------------------------------------------------------
 * ds_check_func: a load in a branch's delay slot, unresolved.
 *
 * The delay slot has already run and its value is parked in `temp_reg`; what
 * is not yet decided is whether it may be written to the guest register, which
 * depends on whether the FIRST OPCODE AT THE TARGET reads that register.  That
 * is one instruction of the guest, at an address that may not be compiled yet,
 * so it is a question only C can answer.
 *
 * The block jumps here with the branch target in r2 and the guest register
 * number in r1, by the same reasoning as above.
 * `lightrec_check_load_delay(state, pc, reg)` either completes the load and
 * hands the PC back unchanged, or takes the slow path -- which can compile a
 * block, interpret an instruction, or fail and set the exit flags -- and hands
 * back a different PC.  All three of those can move the counters, so this gets
 * both halves of the contract too; the failure case in particular works ONLY
 * because of `.Lsync_in`, since C signals it by pulling `target_cycle` down to
 * `current_cycle` and the loop's one test has to see that.
 * ------------------------------------------------------------------------- */
	.globl	_fgl_dispatch_ds_check
_fgl_dispatch_ds_check:
	bsr	.Lsync_out
	 nop

	PIN_PUBLISH

	mov	r1, r6				/* arg2: the guest register */
	mov	r2, r5				/* arg1: the branch target */
	stc	gbr, r4				/* arg0: the state */
	mov.l	.Ldscheck, r0
	jsr	@r0
	 nop
	mov	r0, r2				/* the PC C decided on */

	bsr	.Lsync_in
	 nop

	/* C assigned `regs.gpr[reg]` (lightrec.c:971), so a pinned register
	 * may be the one it wrote. */
	PIN_LOAD

	bra	_fgl_dispatch_loop
	 nop

/* ---------------------------------------------------------------------------
 * fgl_link_stub: the dispatch a linked edge uses ONCE.
 *
 * A block whose successor is a compile-time constant ends in
 *
 *      cmp/pl  r14                     <- the budget, moved out of the loop
 *      bf      .Lout
 *      [nop]                           <- pad, so the site is four-aligned
 *  site+0  mov.l @(FGL_AT_LINK, gbr), r0
 *  site+2  jsr   @r0
 *  site+4  nop                         <- delay slot; PR is site+6
 *  .Lout   the ordinary five-instruction epilogue
 *
 * and arrives here with the target guest PC in r2, exactly as it would have
 * arrived at the loop.  `fgl_link_resolve` looks the target up and, when it is
 * compiled and within `bra` reach, REWRITES site+0..3 into `bra target; nop`
 * -- so the second execution of this edge never comes here again.
 *
 * WHY THE BUDGET TEST MOVED.  It used to sit in the loop, which every block
 * exit passed through; a linked edge does not pass through the loop, so a
 * guest loop of two linked blocks would spin for ever and never give up its
 * timeslice.  Two instructions in the tail buy that back, and the loop keeps
 * its own copy for every edge that still goes round it.
 *
 * WHAT THIS MAY TOUCH is a block's contract, r0/r1/r2/T/PR -- but it makes a
 * C call, and C clobbers r0-r7.  r3-r6 hold nothing at a block tail, the
 * allocator having flushed; r7 does, because it is a pinned guest register and
 * the only pin below r8 (see pins.h).  So r7 is saved here by hand, the way
 * `shim.S` saves it for every other crossing.  No pin needs PUBLISHING: this
 * calls one pure function of the block table and it touches no guest state.
 * ------------------------------------------------------------------------- */
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
/* Labelled so a host harness can point it somewhere else.  The linker fills
 * this in for the real build; the off-hardware test rig has no C to call, so
 * it patches this word to a stub it wrote itself and exercises the same
 * instructions. */
	.globl	_fgl_getnext_slot
_fgl_getnext_slot:
.Lgetnext:	.long	_fgl_get_next_block

/* The same arrangement for the other three doors.  Each is the fgl-side name
 * of one of lightrec's C functions -- `lightrec_memset` and
 * `lightrec_check_load_delay` are static in lightrec.c and cannot be named
 * from here, so the wiring supplies these and they are not merely a stylistic
 * echo of the slot above. */
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
