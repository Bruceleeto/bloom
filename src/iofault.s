! SPDX-License-Identifier: GPL-2.0-only
!
! Fault-driven guest I/O - the DTLB-miss half.
!
! Generated code compiles every unproven guest load/store to a bare masked
! access.  The PSX I/O window (0x1f801000+) is not mapped, so a real I/O
! touch raises a DTLB miss and lands here, via a thunk the installer
! (src/iofault.c) writes over KOS's VBR+0x400 slot.
!
! Contract on entry (SH-4 exception): MD=1, RB=1, BL=1.  r0-r7 name BANK1
! and are ours to clobber; the interrupted code's r0-r7 sit in bank0 and
! are reached with stc/ldc Rn_BANK.  r8-r15 are shared and live.  SPC/SSR
! hold the return state; rte re-executes at SPC, so a serviced access
! advances SPC by 2 to skip the faulting instruction.  The emitter must
! never put a guest memory access in a branch delay slot (jit_sh-cpu.c,
! sh4_slot_legal) - SPC would point at the branch, not the access.
!
! Anything that is not a DTLB miss on a generated-code guest access is
! chained to KOS's original handler (address recovered by the installer).

	.text
	.align	2
	.globl	_bloom_iofault_entry
	.globl	_bloom_iofault_chain_target

_bloom_iofault_entry:
	! Only EXPEVT 0x040 (read miss - also ITLB, filtered by SPC check
	! in C) and 0x060 (write miss) are ours.
	mov.l	Lexpevt, r0
	mov.l	@r0, r1
	mov	#0x40, r2
	cmp/eq	r1, r2
	bt	1f
	mov	#0x60, r2
	cmp/eq	r1, r2
	bf	Lchain_now
1:
	! Frame: pad + pr/mach/macl + 16-word register array; r15 ends as
	! &array[0].  80 bytes, keeps 8-byte alignment.
	mov	r15, r3			! r3 = interrupted r15 (array[15])
	add	#-4, r15		! pad
	sts.l	pr, @-r15
	sts	mach, r0
	mov.l	r0, @-r15
	sts	macl, r0
	mov.l	r0, @-r15
	mov.l	r3, @-r15		! array[15]
	mov.l	r14, @-r15		! array[14]
	mov.l	r13, @-r15
	mov.l	r12, @-r15
	mov.l	r11, @-r15
	mov.l	r10, @-r15
	mov.l	r9, @-r15
	mov.l	r8, @-r15		! array[8]
	stc	r7_bank, r0
	mov.l	r0, @-r15		! array[7]
	stc	r6_bank, r0
	mov.l	r0, @-r15
	stc	r5_bank, r0
	mov.l	r0, @-r15
	stc	r4_bank, r0
	mov.l	r0, @-r15
	stc	r3_bank, r0
	mov.l	r0, @-r15
	stc	r2_bank, r0
	mov.l	r0, @-r15
	stc	r1_bank, r0
	mov.l	r0, @-r15
	stc	r0_bank, r0
	mov.l	r0, @-r15		! array[0]; r15 = &array[0]

	mov	r15, r4			! arg0: regs array
	stc	spc, r5			! arg1: faulting pc
	mov.l	Ltea, r0
	mov.l	@r0, r6			! arg2: fault address
	mov.l	Lhandler, r0
	jsr	@r0
	mov	r1, r7			! arg3: expevt (delay slot)

	tst	r0, r0			! 0 = serviced
	bf	Lchain_restore

	! Write the (possibly updated) register file back and skip the
	! faulting instruction.
	mov.l	@r15+, r0
	ldc	r0, r0_bank
	mov.l	@r15+, r0
	ldc	r0, r1_bank
	mov.l	@r15+, r0
	ldc	r0, r2_bank
	mov.l	@r15+, r0
	ldc	r0, r3_bank
	mov.l	@r15+, r0
	ldc	r0, r4_bank
	mov.l	@r15+, r0
	ldc	r0, r5_bank
	mov.l	@r15+, r0
	ldc	r0, r6_bank
	mov.l	@r15+, r0
	ldc	r0, r7_bank
	mov.l	@r15+, r8
	mov.l	@r15+, r9
	mov.l	@r15+, r10
	mov.l	@r15+, r11
	mov.l	@r15+, r12
	mov.l	@r15+, r13
	mov.l	@r15+, r14
	add	#4, r15			! array[15]: r15 itself, never written
	mov.l	@r15+, r0
	lds	r0, macl
	mov.l	@r15+, r0
	lds	r0, mach
	lds.l	@r15+, pr
	add	#4, r15			! pad

	stc	spc, r0
	add	#2, r0
	ldc	r0, spc
	rte
	nop

Lchain_restore:
	! C declined (SPC not in the code buffer): put everything back
	! exactly as it was and hand the exception to KOS.
	mov.l	@r15+, r0
	ldc	r0, r0_bank
	mov.l	@r15+, r0
	ldc	r0, r1_bank
	mov.l	@r15+, r0
	ldc	r0, r2_bank
	mov.l	@r15+, r0
	ldc	r0, r3_bank
	mov.l	@r15+, r0
	ldc	r0, r4_bank
	mov.l	@r15+, r0
	ldc	r0, r5_bank
	mov.l	@r15+, r0
	ldc	r0, r6_bank
	mov.l	@r15+, r0
	ldc	r0, r7_bank
	mov.l	@r15+, r8
	mov.l	@r15+, r9
	mov.l	@r15+, r10
	mov.l	@r15+, r11
	mov.l	@r15+, r12
	mov.l	@r15+, r13
	mov.l	@r15+, r14
	add	#4, r15
	mov.l	@r15+, r0
	lds	r0, macl
	mov.l	@r15+, r0
	lds	r0, mach
	lds.l	@r15+, pr
	add	#4, r15

Lchain_now:
	mov.l	Lchain, r0
	mov.l	@r0, r0
	jmp	@r0
	mov	#2, r4			! KOS exception class (delay slot)

	.align	2
Lexpevt:
	.long	0xff000024
Ltea:
	.long	0xff00000c
Lhandler:
	.long	_bloom_io_fault
Lchain:
	.long	_bloom_iofault_chain_target

	.data
	.align	2
_bloom_iofault_chain_target:
	.long	0
