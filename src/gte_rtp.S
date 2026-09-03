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

/*
 * MVMVA, the fast shape: sf=1, a real matrix, lm=0.
 *
 *   r4  the COP2 file      r5  the vector (three s16)      r6  their stride
 *   r7  &gte_hot
 *
 * XMTRX already holds the selected matrix and translation column.  MAC1..3
 * are the truncated transform; IR1..3 clamp them to s16.  The reference's
 * A1..A3 flags say MAC left the 32-bit range, which here is `ftrc` having
 * saturated to INT_MAX / INT_MIN - never a valid IR, so those tests live in
 * the clamp stubs and the in-range path pays nothing for them.
 */
	.balign	4
	.global	_gte_mvmva_asm
	.type	_gte_mvmva_asm, @function
_gte_mvmva_asm:
	mov.w	@r5, r0
	lds	r0, fpul
	add	r6, r5
	float	fpul, fr0
	mov.w	@r5, r0
	lds	r0, fpul
	add	r6, r5
	float	fpul, fr1
	mov.w	@r5, r0
	lds	r0, fpul
	mov	r4, r6
	float	fpul, fr2
	add	#64, r6			/* MAC1..3 @36/40/44 */
	fldi1	fr3
	mov	r4, r2
	ftrv	xmtrx, fv0
	add	#32, r2			/* IR1..3 @4/8/12 */
	mov	#0, r3

	/* MAC1 / IR1 */
	ftrc	fr0, fpul
	sts	fpul, r1
	mov.l	r1, @(36, r6)
	exts.w	r1, r0
	cmp/eq	r0, r1
	bf	11f
12:	mov.w	r0, @(4, r2)

	/* MAC2 / IR2 */
	ftrc	fr1, fpul
	sts	fpul, r1
	mov.l	r1, @(40, r6)
	exts.w	r1, r0
	cmp/eq	r0, r1
	bf	21f
22:	mov.w	r0, @(8, r2)

	/* MAC3 / IR3 */
	ftrc	fr2, fpul
	sts	fpul, r1
	mov.l	r1, @(44, r6)
	exts.w	r1, r0
	cmp/eq	r0, r1
	bf	31f
32:	mov.w	r0, @(12, r2)

	mov.l	r3, @r7			/* gte_hot.flag */
	mov.w	80f, r0
	rts
	 mov.l	r3, @(r0, r4)		/* FLAG */

	/* Out of range.  r1 = MAC: raise the saturation flag if ftrc
	 * saturated, then the clamp's, and substitute the bound. */
11:	mov.l	91f, r0
	or	r0, r3
	cmp/pz	r1
	bt	13f
	mov.l	81f, r0
	cmp/eq	r0, r1
	bf	14f
	mov.l	94f, r0
	or	r0, r3
14:	mov.w	84f, r0
	bra	12b
	 nop
13:	mov.l	82f, r0
	cmp/eq	r0, r1
	bf	15f
	mov.l	97f, r0
	or	r0, r3
15:	mov.w	83f, r0
	bra	12b
	 nop

21:	mov.l	92f, r0
	or	r0, r3
	cmp/pz	r1
	bt	23f
	mov.l	81f, r0
	cmp/eq	r0, r1
	bf	24f
	mov.l	95f, r0
	or	r0, r3
24:	mov.w	84f, r0
	bra	22b
	 nop
23:	mov.l	82f, r0
	cmp/eq	r0, r1
	bf	25f
	mov.l	98f, r0
	or	r0, r3
25:	mov.w	83f, r0
	bra	22b
	 nop

31:	mov.l	93f, r0
	or	r0, r3
	cmp/pz	r1
	bt	33f
	mov.l	81f, r0
	cmp/eq	r0, r1
	bf	34f
	mov.l	96f, r0
	or	r0, r3
34:	mov.w	84f, r0
	bra	32b
	 nop
33:	mov.l	82f, r0
	cmp/eq	r0, r1
	bf	35f
	mov.l	99f, r0
	or	r0, r3
35:	mov.w	83f, r0
	bra	32b
	 nop

80:	.word	252			/* FLAG's offset in the file */
83:	.word	0x7fff
84:	.word	0x8000
	.align	2
81:	.long	0x80000000		/* ftrc saturated low */
82:	.long	0x7fffffff		/* ftrc saturated high */
91:	.long	0x81000000		/* limB1 */
92:	.long	0x80800000		/* limB2 */
93:	.long	0x00400000		/* limB3, no master bit */
94:	.long	0x88000000		/* A1 negative: master | bit 27 */
95:	.long	0x84000000		/* A2 negative: master | bit 26 */
96:	.long	0x82000000		/* A3 negative: master | bit 25 */
97:	.long	0x40000000		/* A1 positive: bit 30 */
98:	.long	0x20000000		/* A2 positive: bit 29 */
99:	.long	0x10000000		/* A3 positive: bit 28 */
	.size	_gte_mvmva_asm, . - _gte_mvmva_asm

/*
 * The 3x3 of a control-file matrix as XMTRX columns 0-2, scaled by 1/4096.
 *
 *   r4  the five packed words (R11R12, R13R21, R22R23, R31R32, R33)
 *   r5  twelve floats out, column-major: m[i + 4j] = R(j+1)(i+1)
 *
 * Each halfword converts straight into the register that is its slot, so the
 * stores are six paired moves.  gcc produced ~150 instructions for this from
 * the C; this is ~60.
 */
	.balign	4
	.global	_gte_rot_convert
	.type	_gte_rot_convert, @function
_gte_rot_convert:
	fmov.s	fr12, @-r15
	mova	1f, r0
	fmov.s	@r0, fr12		/* 1/4096 */

	mov.l	@(0, r4), r1		/* R11 | R12 << 16 */
	exts.w	r1, r0
	lds	r0, fpul
	float	fpul, fr0		/* m[0] = R11 */
	swap.w	r1, r0
	exts.w	r0, r0
	lds	r0, fpul
	float	fpul, fr4		/* m[4] = R12 */

	mov.l	@(4, r4), r1		/* R13 | R21 << 16 */
	exts.w	r1, r0
	lds	r0, fpul
	float	fpul, fr8		/* m[8] = R13 */
	swap.w	r1, r0
	exts.w	r0, r0
	lds	r0, fpul
	float	fpul, fr1		/* m[1] = R21 */

	mov.l	@(8, r4), r1		/* R22 | R23 << 16 */
	exts.w	r1, r0
	lds	r0, fpul
	float	fpul, fr5		/* m[5] = R22 */
	swap.w	r1, r0
	exts.w	r0, r0
	lds	r0, fpul
	float	fpul, fr9		/* m[9] = R23 */

	mov.l	@(12, r4), r1		/* R31 | R32 << 16 */
	exts.w	r1, r0
	lds	r0, fpul
	float	fpul, fr2		/* m[2] = R31 */
	swap.w	r1, r0
	exts.w	r0, r0
	lds	r0, fpul
	float	fpul, fr6		/* m[6] = R32 */

	mov.w	@(16, r4), r0		/* R33 */
	lds	r0, fpul
	float	fpul, fr10		/* m[10] = R33 */

	fmul	fr12, fr0
	fmul	fr12, fr1
	fmul	fr12, fr2
	fldi0	fr3
	fmul	fr12, fr4
	fmul	fr12, fr5
	fmul	fr12, fr6
	fldi0	fr7
	fmul	fr12, fr8
	fmul	fr12, fr9
	fmul	fr12, fr10
	fldi0	fr11

	fschg
	fmov.d	dr0, @r5
	add	#8, r5
	fmov.d	dr2, @r5
	add	#8, r5
	fmov.d	dr4, @r5
	add	#8, r5
	fmov.d	dr6, @r5
	add	#8, r5
	fmov.d	dr8, @r5
	add	#8, r5
	fmov.d	dr10, @r5
	fschg
	rts
	 fmov.s	@r15+, fr12

	.align	2
1:	.long	0x39800000
	.size	_gte_rot_convert, . - _gte_rot_convert

/* ------------------------------------------------------------------ */
/* Entry points the generated code calls directly                      */
/* ------------------------------------------------------------------ */

/*
 * The C wrappers cost 30-60 instructions of decode, cache check and call
 * frame per command; on Spyro that was 1.5% of the frame.  These do the
 * two-compare hot check themselves (gte_mtx_use's contract: the column entry
 * was checked at this generation and XMTRX holds this pair) and fall straight
 * into the body, calling C only when the check fails.
 *
 *   gte_hot:  flag @0, ofx @4, ofy @8, ofs_gen @12, xmtrx_key @16
 *   gte_col:  32 bytes per (matrix, column) entry, gen @16
 */

/* RTPS/RTPT: matrix 0, column 0 (key 0), and the offsets must be current.
 * Leaves r4 = file, r5 = &gte_hot for RTP_ENTER. */
.macro RTP_CHECK
	mov.l	10f, r1
	mov.l	@r1, r1			/* psxCP2CtrlGen */
	mov.l	11f, r0
	mov.l	@(16, r0), r2		/* gte_col[0][0].gen */
	cmp/eq	r1, r2
	bf	8f
	mov.l	12f, r5
	mov.l	@(16, r5), r2		/* xmtrx_key */
	tst	r2, r2
	bf	8f
	mov.l	@(12, r5), r2		/* ofs_gen */
	cmp/eq	r1, r2
	bt	9f
8:	sts.l	pr, @-r15
	mov.l	r4, @-r15
	mov.l	13f, r1
	jsr	@r1			/* gte_rtp_slow(file) */
	 nop
	mov.l	@r15+, r4
	lds.l	@r15+, pr
	mov.l	12f, r5
	bra	9f
	 nop
	.align	2
10:	.long	_psxCP2CtrlGen
11:	.long	_gte_col
12:	.long	_gte_hot
13:	.long	_gte_rtp_slow
9:
.endm

	.balign	4
	.global	_gte_rtps_fast
	.type	_gte_rtps_fast, @function
_gte_rtps_fast:
	RTP_CHECK
	mov.l	1f, r0
	jmp	@r0
	 nop
	.align	2
1:	.long	_gte_rtps_asm
	.size	_gte_rtps_fast, . - _gte_rtps_fast

	.balign	4
	.global	_gte_rtpt_fast
	.type	_gte_rtpt_fast, @function
_gte_rtpt_fast:
	RTP_CHECK
	mov.l	1f, r0
	jmp	@r0
	 nop
	.align	2
1:	.long	_gte_rtpt_asm
	.size	_gte_rtpt_fast, . - _gte_rtpt_fast

/*
 * MVMVA: r4 = file, r5 = the op word.  sf=1, a real matrix and lm=0 go to
 * the body above (r5 = vector, r6 = stride, r7 = &gte_hot); anything else is
 * the reference's C.
 */
	.balign	4
	.global	_gte_mvmva_fast
	.type	_gte_mvmva_fast, @function
_gte_mvmva_fast:
	mov.l	20f, r0			/* sf */
	tst	r0, r5
	bt	8f
	mov.w	21f, r0			/* lm */
	tst	r0, r5
	bf	8f
	mov	r5, r0
	shlr16	r0
	shlr	r0
	and	#3, r0			/* mx */
	cmp/eq	#3, r0
	bt	8f
	mov	r0, r6
	mov	r5, r0
	shlr8	r0
	shlr2	r0
	shlr2	r0
	shlr	r0
	and	#3, r0			/* cv */
	mov	r0, r7
	mov	r6, r1
	shll2	r1
	add	r0, r1			/* key */
	mov.l	10f, r2
	mov.l	@r2, r2			/* gen */
	mov	r1, r0
	shll2	r0
	shll2	r0
	shll	r0
	mov.l	11f, r3
	add	r3, r0
	mov.l	@(16, r0), r3		/* gte_col[mx][cv].gen */
	cmp/eq	r2, r3
	bf	7f
	mov.l	12f, r3
	mov.l	@(16, r3), r2		/* xmtrx_key */
	cmp/eq	r1, r2
	bf	7f
6:	mov	r5, r0
	shlr8	r0
	shlr2	r0
	shlr2	r0
	shlr2	r0
	shlr	r0
	and	#3, r0			/* v */
	cmp/eq	#3, r0
	bt	5f
	shll2	r0
	shll	r0
	add	r4, r0
	mov	r0, r5			/* V0..2: three halfwords in a row */
	mov	#2, r6
	mov.l	12f, r7
	mov.l	13f, r0
	jmp	@r0
	 nop
5:	mov	r4, r5
	add	#36, r5			/* IR1..3: every four bytes */
	mov	#4, r6
	mov.l	12f, r7
	mov.l	13f, r0
	jmp	@r0
	 nop
7:	sts.l	pr, @-r15
	mov.l	r5, @-r15
	mov.l	r4, @-r15
	mov	r6, r5
	mov	r7, r6
	mov	r1, r7
	mov.l	14f, r1
	jsr	@r1			/* gte_mtx_use_slow(file, mx, cv, key) */
	 nop
	mov.l	@r15+, r4
	mov.l	@r15+, r5
	lds.l	@r15+, pr
	bra	6b
	 nop
8:	mov.l	15f, r1
	jmp	@r1			/* gte_mvmva_slow(file, op) */
	 nop
	.align	2
10:	.long	_psxCP2CtrlGen
11:	.long	_gte_col
12:	.long	_gte_hot
13:	.long	_gte_mvmva_asm
14:	.long	_gte_mtx_use_slow
15:	.long	_gte_mvmva_slow
20:	.long	0x80000
21:	.word	0x400
	.size	_gte_mvmva_fast, . - _gte_mvmva_fast

/* ------------------------------------------------------------------ */
/* DPCS / INTPL - the depth cue, which Spyro runs on every vertex       */
/* ------------------------------------------------------------------ */

/*
 * The reference's arithmetic, with its 64-bit bounds checks done the SH-4
 * way: `subv` reports the subtraction overflowing 32 bits, and a shift that
 * left 32 bits is one that does not shift back.  gcc emulated the s64 shifts
 * with subc/shld chains and branched on the shift amount - ~210 instructions
 * per DPCS.  The values are the same either way; only the flags need the
 * checks, so they live in the stubs.
 *
 * Register plan for both:
 *   r2  file + 32   IR0 @0, IR1..3 @4/8/12
 *   r6  file + 64   RGB0..2 @16/20/24, MAC1..3 @36/40/44
 *   r9  file + 212  RFC/GFC/BFC @0/4/8
 *   r3  flags   r7  IR0   r12  the colour word being built
 *   r5  DPCS: the extra shift (12 when sf=0, else 0), r14 = -r5
 *       INTPL: the final shift (-12 when sf=1, else 0), r13 = lm's low bound
 *   r8  DPCS: the RGB word
 */

/* limC(mac >> 4) into byte \n of r12; r10 = mac */
.macro RGB_BYTE n, fC
	mov	r10, r0
	mov	#-4, r11
	shad	r11, r0
	mov.w	80f, r11
	cmp/hi	r11, r0
	bt	49f
50:
.if \n == 0
	mov	r0, r12
.elseif \n == 1
	shll8	r0
	or	r0, r12
.else
	shll16	r0
	or	r0, r12
.endif
	bra	99f
	 nop
49:	mov.l	\fC, r11
	or	r11, r3
	cmp/pz	r0
	bt	54f
	bra	50b
	 mov	#0, r0
54:	mov.w	80f, r0
	bra	50b
	 nop
.endm

/* One DPCS component: X = colour byte \n */
.macro DPCS_COMP n, fcoff, macoff, iroff, fAneg, fApos, fB, fC
	mov	r8, r1
.if \n == 1
	shlr8	r1
.elseif \n == 2
	shlr16	r1
.endif
	extu.b	r1, r1			/* X */
	mov	r1, r10
	shll2	r10
	shll2	r10			/* X << 4 */
	mov.l	@(\fcoff, r9), r0
	subv	r10, r0			/* t = FC - (X << 4), T on overflow */
	bt	41f
42:	mov	r0, r10
	shld	r5, r0			/* t << (12 - shift) */
	mov	r0, r11
	shad	r14, r11
	cmp/eq	r10, r11		/* did it survive the shift? */
	bf	43f
44:	exts.w	r0, r11
	cmp/eq	r0, r11
	bf	45f			/* limB */
46:	mul.l	r11, r7			/* IR0 * lim */
	mov	r1, r0
	shll16	r0			/* X << 16 */
	sts	macl, r11
	add	r11, r0
	mov	#-12, r11
	shad	r11, r0			/* MAC */
	mov.l	r0, @(\macoff, r6)
	mov	r0, r10
	exts.w	r0, r11
	cmp/eq	r0, r11
	bf	47f			/* IR = limB(MAC) */
48:	mov.w	r0, @(\iroff, r2)
	RGB_BYTE \n, \fC

	/* the subtraction overflowed: X<<4 >= 0, so it went negative */
41:	mov.l	\fAneg, r11
	or	r11, r3
	bra	42b
	 nop
	/* the shift overflowed: the sign of t says which way */
43:	cmp/pz	r10
	bt	51f
	mov.l	\fAneg, r11
	or	r11, r3
	bra	44b
	 nop
51:	mov.l	\fApos, r11
	or	r11, r3
	bra	44b
	 nop
	/* limB on the shifted t: bound into r11 */
45:	mov.l	\fB, r11
	or	r11, r3
	cmp/pz	r0
	bt	52f
	mov.w	82f, r11
	bra	46b
	 nop
52:	mov.w	81f, r11
	bra	46b
	 nop
	/* limB on MAC for IR: bound into r0 */
47:	mov.l	\fB, r11
	or	r11, r3
	cmp/pz	r0
	bt	53f
	mov.w	82f, r0
	bra	48b
	 nop
53:	mov.w	81f, r0
	bra	48b
	 nop
80:	.word	255
81:	.word	0x7fff
82:	.word	0x8000
	.align	2
99:
.endm

/* One INTPL component: X = IR\n+1 */
.macro INTPL_COMP n, fcoff, macoff, iroff, fAneg, fApos, fB, fC
	mov.w	@(\iroff, r2), r0
	mov	r0, r1			/* X = IRn */
	mov.l	@(\fcoff, r9), r0
	subv	r1, r0			/* t = FC - IRn */
	bt	41f
42:	exts.w	r0, r11
	cmp/eq	r0, r11
	bf	45f			/* limB */
46:	mul.l	r11, r7			/* IR0 * lim */
	mov	r1, r0
	mov	#12, r11
	shld	r11, r0			/* IRn << 12 */
	sts	macl, r11
	add	r11, r0
	shad	r5, r0			/* >> 12 when sf */
	mov.l	r0, @(\macoff, r6)
	mov	r0, r10
	mov.w	81f, r11
	cmp/gt	r11, r0
	bt	47f
	cmp/ge	r13, r0
	bf	55f			/* IR = lim(MAC), lm's low bound */
48:	mov.w	r0, @(\iroff, r2)
	RGB_BYTE \n, \fC

	/* FC - IRn overflowed: IRn > 0 means it went negative, and IRn < 0
	 * would need FC near INT_MAX, which the positive check covers */
41:	cmp/pz	r1
	bf	56f
	mov.l	\fAneg, r11
	or	r11, r3
	bra	42b
	 nop
56:	mov.l	\fApos, r11
	or	r11, r3
	bra	42b
	 nop
45:	mov.l	\fB, r11
	or	r11, r3
	cmp/pz	r0
	bt	52f
	mov.w	82f, r11
	bra	46b
	 nop
52:	mov.w	81f, r11
	bra	46b
	 nop
47:	mov.l	\fB, r11
	or	r11, r3
	mov.w	81f, r0
	bra	48b
	 nop
55:	mov.l	\fB, r11
	or	r11, r3
	bra	48b
	 mov	r13, r0
80:	.word	255
81:	.word	0x7fff
82:	.word	0x8000
	.align	2
99:
.endm

/* The colour FIFO shift, RGB2 from r12 + the old CODE, FLAG out, return.
 * Pops r8..r14. */
.macro RGB_LEAVE
	mov.l	@(20, r6), r0
	mov.l	r0, @(16, r6)		/* RGB0 = RGB1 */
	mov.l	@(24, r6), r0
	mov.l	r0, @(20, r6)		/* RGB1 = RGB2 */
	mov.l	@(24, r4), r0
	shlr16	r0
	shlr8	r0
	shll16	r0
	shll8	r0			/* CODE << 24 */
	or	r12, r0
	mov.l	r0, @(24, r6)		/* RGB2 */
	mov.l	90f, r0
	mov.l	r3, @r0			/* gte_hot.flag */
	mov.w	91f, r0
	mov.l	r3, @(r0, r4)		/* FLAG */
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	rts
	 mov.l	@r15+, r8
91:	.word	252
	.align	2
90:	.long	_gte_hot
.endm

.macro CUE_ENTER
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov.l	r14, @-r15
	mov	r4, r2
	add	#32, r2
	mov	r4, r6
	add	#64, r6
	mov.w	92f, r9
	add	r4, r9
	mov	#0, r3
	mov.w	@r2, r7			/* IR0 */
.endm

	.balign	4
	.global	_gte_dpcs_fast
	.type	_gte_dpcs_fast, @function
_gte_dpcs_fast:
	CUE_ENTER
	mov.l	93f, r0
	tst	r0, r5			/* T = (sf == 0) */
	movt	r5
	shll2	r5
	mov	r5, r0
	add	r0, r5
	add	r0, r5			/* 12 when sf == 0, else 0 */
	neg	r5, r14
	mov.l	@(24, r4), r8		/* RGB */
	bra	1f
	 nop
92:	.word	212
	.align	2
93:	.long	0x80000
1:
	DPCS_COMP 0, 0, 36, 4,  60f, 61f, 62f, 63f
	DPCS_COMP 1, 4, 40, 8,  64f, 65f, 66f, 67f
	DPCS_COMP 2, 8, 44, 12, 68f, 69f, 70f, 71f
	RGB_LEAVE
	.align	2
60:	.long	0x88000000		/* A1 negative */
61:	.long	0x40000000		/* A1 positive */
62:	.long	0x81000000		/* limB1 */
63:	.long	0x00200000		/* limC1 */
64:	.long	0x84000000
65:	.long	0x20000000
66:	.long	0x80800000
67:	.long	0x00100000
68:	.long	0x82000000
69:	.long	0x10000000
70:	.long	0x00400000
71:	.long	0x00080000
	.size	_gte_dpcs_fast, . - _gte_dpcs_fast

	.balign	4
	.global	_gte_intpl_fast
	.type	_gte_intpl_fast, @function
_gte_intpl_fast:
	CUE_ENTER
	mov.w	94f, r0
	tst	r0, r5			/* T = (lm == 0) */
	movt	r13
	neg	r13, r13		/* -1 when lm == 0 */
	mov.w	95f, r0
	and	r0, r13			/* -0x8000 when lm == 0, else 0 */
	mov.l	93f, r0
	tst	r0, r5			/* T = (sf == 0) */
	movt	r5
	add	#-1, r5			/* -1 when sf, else 0 */
	shll2	r5
	mov	r5, r0
	add	r0, r5
	add	r0, r5			/* -12 when sf, else 0 */
	bra	1f
	 nop
92:	.word	212
94:	.word	0x400
95:	.word	0x8000
	.align	2
93:	.long	0x80000
1:
	INTPL_COMP 0, 0, 36, 4,  60f, 61f, 62f, 63f
	INTPL_COMP 1, 4, 40, 8,  64f, 65f, 66f, 67f
	INTPL_COMP 2, 8, 44, 12, 68f, 69f, 70f, 71f
	RGB_LEAVE
	.align	2
60:	.long	0x88000000
61:	.long	0x40000000
62:	.long	0x81000000
63:	.long	0x00200000
64:	.long	0x84000000
65:	.long	0x20000000
66:	.long	0x80800000
67:	.long	0x00100000
68:	.long	0x82000000
69:	.long	0x10000000
70:	.long	0x00400000
71:	.long	0x00080000
	.size	_gte_intpl_fast, . - _gte_intpl_fast
