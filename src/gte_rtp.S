/*
 * RTPS / RTPT in SH-4 assembly.
 *
 * The same arithmetic as the C in gte_fpu.c, in the same order and the same
 * precision - one `ftrv`, one `fdiv`, `fmac` projections, `ftrc` truncation -
 * so results are bit-identical to the C path this replaces (see
 * gte_rtp_selftest() there).  What it removes is the compiler: gcc converted
 * the unsigned divide operands through double precision, switching FPSCR.PR
 * twice per vertex, spilled every pointer to the stack for the RTPT loop and
 * saved four FP registers it then barely used.  Measured on a Chocobo Racing
 * profile: ~600 host instructions per RTPT, ~160 per vertex.  This is ~320.
 *
 * Calling convention: r4 = the COP2 file (psxCP2Regs *), r5 = &gte_hot.  The
 * caller has already made sure XMTRX holds the rotation matrix scaled by
 * 1/4096 with TR in the fourth column, and gte_hot.ofx/ofy are current.
 *
 * Register plan:
 *   r2  file + 32   IR0..IR3 @0/4/8/12, SXY0..2 @16/20/24, SZ0..3 @32..44
 *   r3  file + 232  H @0, DQA @4, DQB @8, FLAG @20
 *   r4  file        VX/VY/VZ for vertex n at 8n/8n+2/8n+4
 *   r6  file + 64   SZ0..3 @0/4/8/12 (halfwords), MAC0 @32, MAC1..3 @36/40/44
 *   r5  gte_hot     flag @0, ofx @4, ofy @8
 *   r7  the flag word being accumulated
 *   r1  IR1   r8  IR2   r9  MAC3, then SZ, then SX   r10  quotient
 *   fr5  -32768.0   fr6 ofx   fr7 ofy   fr8 65536.0   fr9 1/65536
 *   fr10 0.5        fr11 32768.0        fv0 the vertex, then scratch
 *
 * Every limit is the reference's: the in-range path falls through and each
 * bound branches to a stub that ORs its flag word and substitutes the bound.
 * Constant pools sit after each vertex's stubs because `mov.w @(disp,PC)`
 * reaches only 510 bytes forward.
 */

	.text
	.little

/* One vertex.  voff: its register offset from r4.  szoff: the SZ slot as an
 * offset from r6 (SZ0..3 @0/4/8/12), sxyoff: the SXY slot as an offset from
 * r2.  SZ is a halfword store, as the reference's: the top half of the word
 * is left alone.  Leaves the quotient in r10. */
.macro RTP_VERTEX voff, szoff, sxyoff
	/* V -> fv0, w = 1 */
	mov.w	@(\voff, r4), r0
	lds	r0, fpul
	float	fpul, fr0
	mov.w	@(\voff + 2, r4), r0
	lds	r0, fpul
	float	fpul, fr1
	mov.w	@(\voff + 4, r4), r0
	lds	r0, fpul
	float	fpul, fr2
	fldi1	fr3
	ftrv	xmtrx, fv0

	/* MAC1, IR1 = limB1(MAC1) -> r1 */
	ftrc	fr0, fpul
	sts	fpul, r1
	mov.l	r1, @(36, r6)
	exts.w	r1, r0
	cmp/eq	r0, r1
	bf	11f
12:	mov.w	r0, @(4, r2)

	/* MAC2, IR2 = limB2(MAC2) -> r8 */
	ftrc	fr1, fpul
	sts	fpul, r8
	mov.l	r8, @(40, r6)
	exts.w	r8, r0
	cmp/eq	r0, r8
	bf	21f
22:	mov.w	r0, @(8, r2)

	/* MAC3 -> r9 (unclamped, limD wants it), IR3 = limB3(MAC3) */
	ftrc	fr2, fpul
	sts	fpul, r9
	mov.l	r9, @(44, r6)
	exts.w	r9, r0
	cmp/eq	r0, r9
	bf	31f
32:	mov.w	r0, @(12, r2)

	/* SZ = limD(MAC3) -> r9.  Unsigned compare catches negatives too. */
	mov.l	81f, r0
	cmp/hi	r0, r9
	bt	41f
42:	mov	r9, r0
	mov.w	r0, @(\szoff, r6)

	/* quotient = limE(H * 65536 / SZ + 0.5), all-ones when H >= 2 SZ */
	mov.w	@(0, r3), r0
	extu.w	r0, r0
	mov	r9, r10
	shll	r10
	cmp/hs	r10, r0
	bt	52f
	lds	r0, fpul
	float	fpul, fr0
	fmul	fr8, fr0
	lds	r9, fpul
	float	fpul, fr1
	fdiv	fr1, fr0
	fadd	fr10, fr0
	ftrc	fr0, fpul
	sts	fpul, r0
	mov.l	82f, r10
	cmp/hi	r10, r0
	bt	52f
53:	mov	r0, r10

	/* qs = quotient / 65536 -> fr0 */
	lds	r0, fpul
	float	fpul, fr0
	fmul	fr9, fr0

	/* SX = limG1(project(ofx + IR1 * qs)) -> r9 */
	lds	r1, fpul
	float	fpul, fr1
	fmov	fr6, fr2
	fmac	fr0, fr1, fr2
	fcmp/gt	fr2, fr11
	bf	61f
	fcmp/gt	fr2, fr5
	bt	62f
	ftrc	fr2, fpul
	sts	fpul, r0
63:	mov.w	85f, r1
	cmp/gt	r1, r0
	bt	64f
	mov.w	86f, r1
	cmp/ge	r1, r0
	bf	64f
66:	extu.w	r0, r9

	/* SY = limG2(project(ofy + IR2 * qs)), packed with SX */
	lds	r8, fpul
	float	fpul, fr1
	fmov	fr7, fr2
	fmac	fr0, fr1, fr2
	fcmp/gt	fr2, fr11
	bf	71f
	fcmp/gt	fr2, fr5
	bt	72f
	ftrc	fr2, fpul
	sts	fpul, r0
73:	mov.w	85f, r1
	cmp/gt	r1, r0
	bt	74f
	mov.w	86f, r1
	cmp/ge	r1, r0
	bf	74f
76:	shll16	r0
	or	r9, r0
	bra	99f
	 mov.l	r0, @(\sxyoff, r2)

	/* --- the stubs --- */

	/* IR1 out of range: flag, bound into r0 and r1 */
11:	mov.l	91f, r0
	or	r0, r7
	cmp/pz	r1
	bt	13f
	mov.w	84f, r0
	bra	12b
	 mov	r0, r1
13:	mov.w	83f, r0
	bra	12b
	 mov	r0, r1

	/* IR2 */
21:	mov.l	92f, r0
	or	r0, r7
	cmp/pz	r8
	bt	23f
	mov.w	84f, r0
	bra	22b
	 mov	r0, r8
23:	mov.w	83f, r0
	bra	22b
	 mov	r0, r8

	/* IR3: only the stored value, r9 keeps MAC3 */
31:	mov.l	93f, r0
	or	r0, r7
	cmp/pz	r9
	bt	33f
	mov.w	84f, r0
	bra	32b
	 nop
33:	mov.w	83f, r0
	bra	32b
	 nop

	/* limD */
41:	mov.l	94f, r0
	or	r0, r7
	cmp/pz	r9
	bt	43f
	bra	42b
	 mov	#0, r9
43:	mov.l	81f, r9
	bra	42b
	 nop

	/* limE, also the H >= 2 SZ path (all-ones limits the same way) */
52:	mov.l	95f, r10
	or	r10, r7
	mov.l	82f, r0
	bra	53b
	 nop

	/* SX projection saturated high / low, then through limG1 */
61:	mov.l	96f, r1
	or	r1, r7
	mov.w	83f, r0
	bra	63b
	 nop
62:	mov.l	97f, r1
	or	r1, r7
	mov.w	84f, r0
	bra	63b
	 nop
	/* limG1: r1 holds the bound that failed */
64:	mov.l	98f, r0
	or	r0, r7
	bra	66b
	 mov	r1, r0

	/* SY */
71:	mov.l	96f, r1
	or	r1, r7
	mov.w	83f, r0
	bra	73b
	 nop
72:	mov.l	97f, r1
	or	r1, r7
	mov.w	84f, r0
	bra	73b
	 nop
74:	mov.l	100f, r0
	or	r0, r7
	bra	76b
	 mov	r1, r0

	.align	2
81:	.long	0xffff
82:	.long	0x1ffff
83:	.word	0x7fff
84:	.word	0x8000		/* sign-extends to -32768 */
85:	.word	0x3ff
86:	.word	0xfc00		/* -1024 */
	.align	2
91:	.long	0x81000000	/* limB1: master | bit 24 */
92:	.long	0x80800000	/* limB2: master | bit 23 */
93:	.long	0x00400000	/* limB3: bit 22, no master */
94:	.long	0x80040000	/* limD */
95:	.long	0x80020000	/* limE */
96:	.long	0x80010000	/* project: >= 2^15 */
97:	.long	0x80008000	/* project: < -2^15 */
98:	.long	0x80004000	/* limG1 */
100:	.long	0x80002000	/* limG2 */
99:
.endm

/* Prologue: save r8-r10, set up the bases and the float constants. */
.macro RTP_ENTER
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov	r4, r2
	add	#32, r2
	mov	r4, r6
	add	#64, r6
	mov.w	2f, r3
	add	r4, r3
	mov	#0, r7
	mova	3f, r0
	fmov.s	@r0+, fr8
	fmov.s	@r0+, fr9
	fmov.s	@r0+, fr10
	fmov.s	@r0+, fr11
	mov	r5, r1
	add	#4, r1
	fmov.s	@r1+, fr6
	fmov.s	@r1, fr7
	fmov	fr11, fr5
	bra	1f
	 fneg	fr5
2:	.word	232
	.align	2
3:	.long	0x47800000	/* 65536.0 */
	.long	0x37800000	/* 1/65536 */
	.long	0x3f000000	/* 0.5 */
	.long	0x47000000	/* 32768.0 */
1:
.endm

/* The depth cue from the last quotient (r10), the flag word out, return. */
.macro RTP_LEAVE
	mov.w	@(4, r3), r0		/* DQA, sign-extended */
	dmuls.l	r0, r10
	mov.l	@(8, r3), r1		/* DQB */
	mov	r1, r0
	shll	r0
	subc	r0, r0			/* sign of DQB, 64-bit */
	sts	macl, r9
	sts	mach, r8
	clrt
	addc	r1, r9
	addc	r0, r8
	mov.l	r9, @(32, r6)		/* MAC0 = low word */
	mov	#20, r0
	shld	r0, r8
	mov	#-12, r0
	shld	r0, r9
	or	r8, r9			/* (tmp >> 12) as s32 */
	mov.w	4f, r0
	cmp/gt	r0, r9
	bt	1f
	cmp/pz	r9
	bf	2f
	mov	r9, r0
3:	mov.w	r0, @(0, r2)		/* IR0 */
	mov.l	r7, @(20, r3)		/* FLAG */
	mov.l	r7, @r5			/* gte_hot.flag */
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	rts
	 mov.l	@r15+, r8
1:	mov.w	4f, r1
	or	r1, r7
	bra	3b
	 nop
2:	mov.w	4f, r1
	or	r1, r7
	bra	3b
	 mov	#0, r0
4:	.word	0x1000			/* limH bound, and its flag bit */
.endm

	.balign	4
	.global	_gte_rtpt_asm
	.type	_gte_rtpt_asm, @function
_gte_rtpt_asm:
	RTP_ENTER
	/* SZ0 takes the old SZ3 before anything is written */
	mov.w	@(12, r6), r0
	mov.w	r0, @(0, r6)
	RTP_VERTEX 0, 4, 16
	RTP_VERTEX 8, 8, 20
	RTP_VERTEX 16, 12, 24
	RTP_LEAVE
	.size	_gte_rtpt_asm, . - _gte_rtpt_asm

	.balign	4
	.global	_gte_rtps_asm
	.type	_gte_rtps_asm, @function
_gte_rtps_asm:
	RTP_ENTER
	/* Both stacks shift before the new vertex lands */
	mov.w	@(4, r6), r0
	mov.w	r0, @(0, r6)
	mov.w	@(8, r6), r0
	mov.w	r0, @(4, r6)
	mov.w	@(12, r6), r0
	mov.w	r0, @(8, r6)
	mov.l	@(20, r2), r0
	mov.l	r0, @(16, r2)
	mov.l	@(24, r2), r0
	mov.l	r0, @(20, r2)
	RTP_VERTEX 0, 12, 24
	RTP_LEAVE
	.size	_gte_rtps_asm, . - _gte_rtps_asm
