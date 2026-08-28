! SPDX-License-Identifier: GPL-2.0-only
!
!  PS1 BGR555 -> PVR ARGB1555, one 16x16 texture block.
!
!  void cvt16_block(uint32_t *dst, const uint32_t *src);       normal pages
!  void cvt16_block_mask(uint32_t *dst, const uint32_t *src);  mask pages
!
!  src: VRAM, 2048-byte row stride. dst: staging, 128-byte row stride,
!  32-byte aligned rows (each row of the block is exactly one cache line,
!  allocated with movca.l so it is not read in first).
!
!  Per 32-bit word (two pixels): swap the 5-bit R and B fields, then set
!  bit 15 of each non-zero halfword (mask pages toggle it):
!  (h & 0x7fff) + 0x7fff carries into bit 15 iff h != 0.
!  GCC's version of this loop was 27 instructions a word with two literal
!  reloads inside it; this is 17, unrolled per row.

.text
.align 2

! r1 = word in, r0 = converted out (r0: movca.l source). Clobbers r2,r3.
! r8 = 7c007c00  r9 = 001f001f  r10 = 83e083e0  r11 = 7fff7fff  r12 = 80008000
! r1 = word in, r0 = converted out (r0: movca.l source). Clobbers r2,r3.
! r8 = 7c007c00  r9 = 001f001f  r10 = 83e083e0  r11 = 7fff7fff  r12 = 80008000
! r6 = 10, r7 = -10 (shld counts)
.macro SWAP
	mov	r1,r2
	and	r8,r2
	shld	r7,r2
	mov	r1,r3
	and	r9,r3
	shld	r6,r3
	and	r10,r1
	or	r2,r1
	or	r3,r1
.endm

.macro CVT op
	SWAP
	mov	r1,r0
	and	r11,r0
	add	r11,r0
	and	r12,r0
	\op	r1,r0
.endm

! No alpha: for blocks only the FB quad reads (drawn over black, so a
! zero pixel opaque-black equals a hole). 10 instructions a word.
.macro CVT_NA op
	SWAP
	mov	r1,r0
.endm

.macro ROW cvt, op
	mov.l	@r5+,r1
	\cvt	\op
	movca.l	r0,@r4
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(4,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(8,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(12,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(16,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(20,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(24,r4)
	mov.l	@r5+,r1
	\cvt	\op
	mov.l	r0,@(28,r4)
	ocbwb	@r4			! write the row back now: the flush is a DMA
.endm

.macro BLOCK name, cvt, op
.globl _\name
.type _\name,%function
_\name:
	mov.l	r8,@-r15
	mov.l	r9,@-r15
	mov.l	r10,@-r15
	mov.l	r11,@-r15
	mov.l	r12,@-r15
	mov.l	r13,@-r15
	mov.l	.Lc_a_\name,r8
	mov.l	.Lc_b_\name,r9
	mov.l	.Lc_c_\name,r10
	mov.l	.Lc_d_\name,r11
	mov.l	.Lc_e_\name,r12
	mov	#10,r6
	mov	#-10,r7
	mov	#16,r13
.Lloop_\name:
	mov.l	.Lc_p_\name,r0		! prefetch one row ahead
	add	r5,r0
	pref	@r0
	ROW	\cvt, \op
	mov.l	.Lc_s_\name,r0		! source row stride minus the row's 32 bytes
	add	r0,r5
	add	#64,r4			! +128: the immediate is 8-bit signed
	dt	r13
	bf/s	.Lloop_\name
	add	#64,r4
	mov.l	@r15+,r13
	mov.l	@r15+,r12
	mov.l	@r15+,r11
	mov.l	@r15+,r10
	mov.l	@r15+,r9
	rts
	mov.l	@r15+,r8
.align 2
.Lc_a_\name:	.long	0x7c007c00
.Lc_b_\name:	.long	0x001f001f
.Lc_c_\name:	.long	0x83e083e0
.Lc_d_\name:	.long	0x7fff7fff
.Lc_e_\name:	.long	0x80008000
.Lc_s_\name:	.long	2048-32
.Lc_p_\name:	.long	2048
.endm

	BLOCK	cvt16_block, CVT, or
	BLOCK	cvt16_block_mask, CVT, xor
	BLOCK	cvt16_block_noalpha, CVT_NA, or

