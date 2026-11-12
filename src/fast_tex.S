! Fast textured polygon (GP0 24-3f, opaque, 4bpp/8bpp) in SH-4 assembly.
! bool fast_tex_asm(const uint32_t *buf /*r4*/, uint32_t cmd /*r5*/)
! Same checks, state changes and TA bytes as fast_tex_c() in pvr.c. Anything it
! does not handle (bright colours, texture upload needed, codebook cache miss,
! clip toggle) jumps to fast_tex_c() with r4/r5 untouched and no state changed.
! Checked in the SH4-timing lab: every TA record it writes matches the C path.
! Layout it relies on is asserted next to struct ft_ctx in pvr.c.

	.set	P_ZOFF, 8
	.set	P_DX1, 12
	.set	P_DY1, 14
	.set	P_DX2, 16
	.set	P_DY2, 18
	.set	P_OX, 24
	.set	P_OY, 26
	.set	P_BITS, 36
	! struct ft_ctx
	.set	C_PVR, 0
	.set	C_OLD, 4
	.set	C_CBC, 8
	.set	C_CBE, 12
	.set	C_LUT, 16
	.set	C_TMPL, 20
	.set	C_INVAL, 24
	.set	C_PAGE4, 28
	.set	C_PAGE8, 156

	.macro	BBOX
	add	r8, r1
	mov.l	@r1, r0
	exts.w	r0, r2
	cmp/gt	r2, r10
	bf	1f
	mov	r2, r10
1:	cmp/gt	r11, r2
	bf	1f
	mov	r2, r11
1:	swap.w	r0, r0
	exts.w	r0, r2
	cmp/gt	r2, r12
	bf	1f
	mov	r2, r12
1:	cmp/gt	r13, r2
	bf	1f
	mov	r2, r13
1:	mov.l	@(4, r1), r0
	extu.b	r0, r2
	cmp/hi	r2, r14
	bf	1f
	mov	r2, r14
1:	cmp/hi	r3, r2
	bf	1f
	mov	r2, r3
1:	shlr8	r0
	extu.b	r0, r2
	cmp/hi	r2, r6
	bf	1f
	mov	r2, r6
1:	cmp/hi	r7, r2
	bf	1f
	mov	r2, r7
1:
	.endm

	! colour word in r2 -> 0x00rrggbb through the LUT (r3) into r7; uses r0, r1
	.macro	COLOUR
	extu.b	r2, r0
	mov.b	@(r0, r3), r7
	shlr8	r2
	extu.b	r7, r7
	extu.b	r2, r0
	mov.b	@(r0, r3), r1
	shlr8	r2
	shll16	r7
	extu.b	r1, r1
	extu.b	r2, r0
	shll8	r1
	mov.b	@(r0, r3), r0
	or	r1, r7
	extu.b	r0, r0
	or	r0, r7
	.endm

	.text
	.align	2
.Lbail0_a:
	mov.l	.Lcfn_a, r0
	jmp	@r0
	nop
.Lbail_a:
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	mov.l	.Lcfn_a, r0
	jmp	@r0
	nop
	.align	2
.Lcfn_a: .long	_fast_tex_c
	.align	5
	.globl	_fast_tex_asm
_fast_tex_asm:
	mov.l	.Lctx, r1
	mov	r5, r0
	mov.l	@(C_PVR, r1), r1
	mov.l	@(P_BITS, r1), r2
	tst	#2, r0
	bf	.Lbail0_a			! semi-transparent
	mov	r2, r0
	tst	#0x0d, r0		! new_frame | set_mask | check_mask
	bf	.Lbail0_a

	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov.l	r14, @-r15

	mov	r5, r0
	tst	#0x10, r0
	mov	#8, r8			! stride in bytes
	bt	1f
	mov	#12, r8
1:	tst	#0x08, r0
	mov	#3, r9			! vertices
	bt	1f
	mov	#4, r9
1:
	! bright colours go to C
	tst	#1, r0
	bf	.Lnobright
	mov.l	.Lm7, r10
	mov.l	.Lm8, r11
	tst	#0x10, r0
	mov	#1, r3
	bt	2f
	mov	r9, r3
2:	mov	r4, r1
3:	mov.l	@r1, r2
	mov	r2, r0
	and	r10, r0
	add	r10, r0
	and	r2, r0
	tst	r11, r0
	bf	.Lbail_a
	dt	r3
	add	r8, r1
	bf	3b
.Lnobright:

	! bounding boxes
	mov	r4, r1
	add	#4, r1
	mov.l	@r1, r0
	exts.w	r0, r10
	mov	r10, r11
	swap.w	r0, r0
	exts.w	r0, r12
	mov	r12, r13
	mov.l	@(4, r1), r0
	extu.b	r0, r14
	mov	r14, r3
	shlr8	r0
	extu.b	r0, r6
	mov	r6, r7
	BBOX
	BBOX
	mov	r9, r0
	cmp/eq	#4, r0
	bf	.Lbbdone
	BBOX
.Lbbdone:
	mov	r11, r0
	sub	r10, r0
	mov.w	.L1024, r2
	cmp/ge	r2, r0
	bt	.Ldrop_b
	mov	r13, r0
	sub	r12, r0
	mov.w	.L512, r2
	cmp/ge	r2, r0
	bt	.Ldrop_b

	bra	.Lclip
	nop
.Ldrop_b:
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	rts
	mov	#1, r0
.Lbail_b:
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	mov.l	.Lcfn_b, r0
	jmp	@r0
	nop
	.align	1
.L1024:	.word	1024
.L512:	.word	512
	.align	2
.Lctx:	.long	_ft_ctx
.Lm7:	.long	0x007f7f7f
.Lm8:	.long	0x00808080
.Lcfn_b: .long	_fast_tex_c
.Lclip:
	! draw-area test
	mov.l	.Lctx_c, r1
	mov.l	@(C_PVR, r1), r1
	mov.l	@(P_BITS, r1), r0
	tst	#0x10, r0
	bt	.Lnoclip
	mov.w	@(P_OX, r1), r0
	add	r0, r10
	add	r0, r11
	mov.w	@(P_OY, r1), r0
	add	r0, r12
	add	r0, r13
	exts.w	r10, r0
	cmp/eq	r0, r10
	bf	.Lbail_b
	exts.w	r11, r0
	cmp/eq	r0, r11
	bf	.Lbail_b
	exts.w	r12, r0
	cmp/eq	r0, r12
	bf	.Lbail_b
	exts.w	r13, r0
	cmp/eq	r0, r13
	bf	.Lbail_b
	mov.w	@(P_DX1, r1), r0
	cmp/gt	r10, r0
	bt	.Lbail_b
	mov.w	@(P_DX2, r1), r0
	cmp/gt	r0, r11
	bt	.Lbail_b
	mov.w	@(P_DY1, r1), r0
	cmp/gt	r12, r0
	bt	.Lbail_b
	mov.w	@(P_DY2, r1), r0
	cmp/gt	r0, r13
	bt	.Lbail_b
.Lnoclip:

	! texture page
	mov	r4, r0
	add	r8, r0
	mov.w	@(10, r0), r0
	extu.w	r0, r0
	mov	r0, r1
	shll	r0
	shlr8	r0
	and	#3, r0			! bpp
	mov	#31, r2
	and	r2, r1			! page id
	cmp/eq	#0, r0
	bt	.L4bpp
	cmp/eq	#1, r0
	bf	.Lbail_c
	! 8bpp: 128 texels per page
	mov	r14, r0
	shll	r0
	shlr8	r0			! offt = umin >> 7
	mov	r3, r2
	add	#-1, r2
	extu.w	r2, r2
	shll	r2
	shlr8	r2
	cmp/eq	r0, r2
	bf	.Lbail_c
	add	r0, r1
	mov	#31, r2
	and	r2, r1
	mov	r0, r11
	shll8	r11
	shlr	r11			! usub = offt << 7
	sub	r11, r14
	shll	r14
	sub	r11, r3
	shll	r3
	mov	#1, r10			! ushift
	mov.w	.Lpg8, r0
	bra	.Lpage
	shll2	r1
.L4bpp:
	mov	#0, r10
	mov	#0, r11
	mov	#C_PAGE4, r0
	shll2	r1
.Lpage:
	mov.l	.Lctx_c, r2
	add	r2, r0
	mov.l	@(r0, r1), r12		! page
	mov.l	@(8, r12), r13		! vq
	tst	r13, r13
	bt	.Lbail_c

	! block mask (r1 low, r2 high)
	mov	#-64, r0
	and	r14, r0
	cmp/hi	r0, r3
	bf	.Lbm0
	mov	#-16, r0
	and	r6, r0
	cmp/hi	r0, r7
	bf	.Lbm0
	mov	r14, r0
	shlr2	r0
	shlr2	r0
	shlr2	r0
	mov	#1, r2
	shld	r0, r2
	add	#-1, r2
	not	r2, r2
	mov	r3, r0
	add	#-1, r0
	shlr2	r0
	shlr2	r0
	shlr2	r0
	mov	#2, r1
	shld	r0, r1
	add	#-1, r1
	and	r2, r1			! horiz
	mov.l	.L1111, r0
	dmulu.l	r0, r1
	mov.l	.Lltp, r14
	sts	macl, r1
	sts	mach, r2
	add	r1, r2
	mov	r6, r0
	shlr2	r0
	shlr2	r0
	shll2	r0
	shll	r0
	mov.l	@(r0, r14), r3
	add	#4, r0
	mov.l	@(r0, r14), r6
	not	r3, r3
	not	r6, r6
	mov	r7, r0
	add	#-1, r0
	shlr2	r0
	shlr2	r0
	add	#1, r0
	shll2	r0
	shll	r0
	mov.l	@(r0, r14), r7
	add	#4, r0
	mov.l	@(r0, r14), r0
	and	r3, r7
	and	r6, r0
	and	r7, r1
	bra	.Lbmdone
	and	r0, r2
.Lbail_c:
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	mov.l	.Lcfn_c, r0
	jmp	@r0
	nop
	.align	1
.Lpg8:	.word	C_PAGE8
	.align	2
.Lctx_c: .long	_ft_ctx
.L1111:	.long	0x11111111
.Lltp:	.long	.Llt
.Lcfn_c: .long	_fast_tex_c
.Lbm0:
	mov	#0, r1
	mov	#0, r2
.Lbmdone:
	mov.l	@(12, r12), r0
	not	r0, r0
	tst	r1, r0
	bf	.Lbail_c			! blocks to load
	mov.l	@(16, r12), r0
	not	r0, r0
	tst	r2, r0
	bf	.Lbail_c

	! codebook cache
	mov.w	@(10, r4), r0
	mov.w	.L7fff, r3
	and	r3, r0
	mov	r0, r3			! clut
	mov	r0, r6
	shlr2	r6
	shlr2	r6
	shlr2	r6
	xor	r6, r0
	mov	r12, r6
	shlr2	r6
	shlr2	r6
	shlr	r6
	xor	r6, r0
	extu.b	r0, r0			! 256 entries
	mov	r0, r6
	add	r0, r6
	add	r0, r6
	shll2	r6
	mov.l	.Lctx_d, r0
	mov.l	@(C_CBC, r0), r0
	add	r0, r6
	mov.l	@r6, r0
	cmp/eq	r0, r12
	bf	.Lbail_c
	mov.w	@(4, r6), r0
	extu.w	r0, r0
	cmp/eq	r0, r3
	bf	.Lbail_c
	mov.w	@(6, r6), r0
	extu.w	r0, r7
	mov.l	.Lctx_d, r0
	mov.l	@(C_INVAL, r0), r0
	mov.w	@r0, r0
	extu.w	r0, r0
	cmp/eq	r0, r7
	bf	.Lbail_c
	mov.w	@(8, r6), r0
	extu.w	r0, r7
	mov.l	.Lctx_d, r0
	mov.l	@(C_CBE, r0), r0
	mov.w	@r0, r0
	extu.w	r0, r0
	cmp/eq	r0, r7
	bf	.Lbail_c
	mov.w	@(10, r6), r0
	extu.w	r0, r0			! codebook

	tst	r10, r10
	bf	1f
	mov.w	.L248, r14
	sub	r0, r14			! voffset
	shll8	r0
	bra	2f
	add	r0, r13			! tex
.Lbail_d:
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	mov.l	.Lcfn_d, r0
	jmp	@r0
	nop
	.align	1
.L7fff:	.word	0x7fff
.L248:	.word	248
	.align	2
.Lctx_d: .long	_ft_ctx
.Lcfn_d: .long	_fast_tex_c
1:	mov	#31, r14
	sub	r0, r14
	shll2	r14
	shll2	r14
	shll8	r0
	shll2	r0
	shll	r0
	add	r0, r13
2:
	mov	r5, r0
	and	#8, r0
	shll2	r0
	mov.w	.L90, r3
	or	r0, r3			! flags
	mov.l	.Lctx_e, r6
	mov.l	@(C_OLD, r6), r6
	mov.w	@(2, r6), r0
	extu.w	r0, r7			! old flags
	xor	r3, r0
	tst	#0x80, r0
	bf	.Lbail_d			! clip state toggles: C handles it

	! ---- committed ----
	mov.l	@(20, r12), r0
	or	r1, r0
	mov.l	r0, @(20, r12)
	mov.l	@(24, r12), r0
	or	r2, r0
	mov.l	r0, @(24, r12)
	mov.l	.Lctx_e, r1
	mov.l	@(C_PVR, r1), r1
	mov.l	@(P_ZOFF, r1), r0
	mov	r0, r2
	add	#1, r2
	mov.l	r2, @(P_ZOFF, r1)
	extu.w	r0, r0
	shll8	r0
	mov.l	.Lzbase, r12
	add	r0, r12			! z
	mov.l	.Ldr_e, r1
	mov.l	@r1, r2			! sq

	mov.b	@r6, r0
	tst	r0, r0
	bt	.Lhdr
	cmp/eq	r7, r3
	bf	.Lhdr
	mov.l	@(4, r6), r0
	cmp/eq	r0, r13
	bt	.Lnohdr
.Lhdr:
	mov	#1, r0
	mov.b	r0, @r6
	mov	r3, r0
	mov.w	r0, @(2, r6)
	mov.l	r13, @(4, r6)
	mov	#32, r0
	xor	r0, r2
	mov.l	.Lctx_e, r1
	mov.l	@(C_TMPL, r1), r1
	mov.l	@r1, r0
	mov.l	r0, @r2
	mov.l	@(4, r1), r0
	mov.l	r0, @(4, r2)
	mov.l	@(8, r1), r0
	tst	r10, r10
	bt	1f
	mov	#-9, r7
	and	r7, r0			! 8bpp: u size 512
1:	mov.l	r0, @(8, r2)
	mov	r13, r0
	shll8	r0
	shll	r0
	shlr8	r0
	shlr2	r0
	shlr2	r0
	mov.l	.L44, r7
	or	r7, r0
	mov.l	r0, @(12, r2)
	mov.l	@(16, r1), r0
	mov.l	r0, @(16, r2)
	mov.l	@(20, r1), r0
	mov.l	r0, @(20, r2)
	mov.l	@(24, r1), r0
	mov.l	r0, @(24, r2)
	mov.l	@(28, r1), r0
	mov.l	r0, @(28, r2)
	pref	@r2
.Lnohdr:
	! the fields every vertex shares, in both queues
	mov.l	.Lvtx, r0
	mov	#0, r1
	mov	#32, r7
	mov.l	r0, @r2
	mov.l	r12, @(12, r2)
	mov.l	r1, @(28, r2)
	xor	r7, r2
	mov.l	r0, @r2
	mov.l	r12, @(12, r2)
	mov.l	r1, @(28, r2)
	xor	r7, r2
	mov	r2, r6			! sq

	mov.l	.Lctx_e, r3
	mov.l	@(C_PVR, r3), r1
	mov.w	@(P_OX, r1), r0
	mov	r0, r12
	mov.w	@(P_OY, r1), r0
	mov	r0, r13
	mov.l	.Lfoffp, r1
	fmov.s	@r1+, fr4
	fmov.s	@r1, fr5
	mov.l	@(C_LUT, r3), r3
	mov.l	.Lwhite, r7
	mov	r5, r0
	tst	#1, r0
	bf	.Lraw
	tst	#0x10, r0
	bf	.Lmulti
	mov.l	@r4, r2
	COLOUR
.Lraw:
	bra	.Lgo
	mov	#0, r5
	.align	1
.L90:	.word	0x0090
	.align	2
.Lctx_e: .long	_ft_ctx
.Lzbase: .long	0x3e800000
.Ldr_e:	.long	_pvr_dr_addr
.L44:	.long	0x44000000
.Lvtx:	.long	0xe0000000
.Lfoffp: .long	.Lfoff
.Lwhite: .long	0x00ffffff
.Lmulti:
	mov	#1, r5
.Lgo:
	add	#4, r4
	.align	2
.Lvloop:
	tst	r5, r5
	bt	1f
	mov	r4, r1
	add	#-4, r1
	mov.l	@r1, r2
	COLOUR
1:	mov.l	@r4, r0
	exts.w	r0, r1
	add	r12, r1
	exts.w	r1, r1
	lds	r1, fpul
	swap.w	r0, r0
	float	fpul, fr0
	exts.w	r0, r1
	add	r13, r1
	exts.w	r1, r1
	lds	r1, fpul
	mov.l	@(4, r4), r0
	float	fpul, fr1
	extu.b	r0, r1
	sub	r11, r1
	shld	r10, r1
	lds	r1, fpul
	shlr8	r0
	float	fpul, fr2
	extu.b	r0, r1
	add	r14, r1
	lds	r1, fpul
	mov	#32, r0
	float	fpul, fr3
	xor	r0, r6
	ftrv	xmtrx, fv0
	dt	r9
	fadd	fr4, fr2
	bf/s	2f
	fadd	fr5, fr3
	mov.l	.Leol, r0
	mov.l	r0, @r6
2:	mov	r6, r1
	add	#28, r1
	mov.l	r7, @-r1
	fmov.s	fr3, @-r1
	fmov.s	fr2, @-r1
	add	#-4, r1
	fmov.s	fr1, @-r1
	fmov.s	fr0, @-r1
	pref	@r1
	tst	r9, r9
	bf/s	.Lvloop
	add	r8, r4

	mov.l	.Ldr, r1
	mov.l	r6, @r1
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	rts
	mov	#1, r0

	.align	2
.Ldr:	.long	_pvr_dr_addr
.Leol:	.long	0xf0000000
.Lfoff:	.float	0.00048828125		! 1/2048
	.float	0.000030517578125	! 1/32768
	! (1 << 4k) - 1 as 64 bits, k = 0..16
.Llt:	.long	0x00000000, 0
	.long	0x0000000f, 0
	.long	0x000000ff, 0
	.long	0x00000fff, 0
	.long	0x0000ffff, 0
	.long	0x000fffff, 0
	.long	0x00ffffff, 0
	.long	0x0fffffff, 0
	.long	0xffffffff, 0
	.long	0xffffffff, 0x0000000f
	.long	0xffffffff, 0x000000ff
	.long	0xffffffff, 0x00000fff
	.long	0xffffffff, 0x0000ffff
	.long	0xffffffff, 0x000fffff
	.long	0xffffffff, 0x00ffffff
	.long	0xffffffff, 0x0fffffff
	.long	0xffffffff, 0xffffffff
