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

	/* POC: q = H / SZ in float -> fr4 (bleem's shape, no integer quotient,
	 * no rounding), limE flagged when H >= 2 SZ */
	mov.w	@(0, r3), r0
	extu.w	r0, r0
	mov	r9, r10
	shll	r10
	cmp/hs	r10, r0
	bt	52f
	lds	r0, fpul
	float	fpul, fr4
	lds	r9, fpul
	float	fpul, fr1
	fdiv	fr1, fr4
53:
	/* SX = limG1(ofx + IR1 * q) -> r9, no project saturation */
	lds	r1, fpul
	float	fpul, fr1
	fmov	fr6, fr2
	fmov	fr4, fr0
	fmac	fr0, fr1, fr2
	ftrc	fr2, fpul
	sts	fpul, r0
63:	mov.w	85f, r1
	cmp/gt	r1, r0
	bt	64f
	mov.w	86f, r1
	cmp/ge	r1, r0
	bf	64f
66:	extu.w	r0, r9

	/* SY = limG2(ofy + IR2 * q), packed with SX */
	lds	r8, fpul
	float	fpul, fr1
	fmov	fr7, fr2
	fmov	fr4, fr0
	fmac	fr0, fr1, fr2
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

	/* limE, also the H >= 2 SZ path: q = 0x1ffff / 65536 */
52:	mov.l	95f, r10
	or	r10, r7
	mov.l	82f, r0
	lds	r0, fpul
	float	fpul, fr4
	bra	53b
	 fmul	fr9, fr4

	/* limG1: r1 holds the bound that failed */
64:	mov.l	98f, r0
	or	r0, r7
	bra	66b
	 mov	r1, r0

	/* SY */
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
	/* POC: MAC0 = ftrc(DQB + DQA * 65536 * q) in float */
	mov.w	@(4, r3), r0		/* DQA */
	lds	r0, fpul
	float	fpul, fr0
	fmul	fr8, fr0
	fmul	fr4, fr0
	mov.l	@(8, r3), r0		/* DQB */
	lds	r0, fpul
	float	fpul, fr1
	fadd	fr1, fr0
	ftrc	fr0, fpul
	sts	fpul, r9
	mov.l	r9, @(32, r6)		/* MAC0 */
	mov	#-12, r0
	shad	r0, r9
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

/*
 * MVMVA with sf == 0, in integers.
 *
 * The FPU path above cannot serve this.  gte_rot_convert folds the >>12 into
 * XMTRX as 1/4096, so with sf set the MAC lands near 2^19 and float32's
 * 24-bit mantissa holds it exactly.  With sf clear the results are 4096x
 * larger - up to 2^31 - which float32 cannot represent exactly, and GTE
 * results are guest-visible.
 *
 * Measured on Gran Turismo: every MVMVA bail-out is sf == 0, about 9,700 a
 * second, and not one is mx >= 3 or lm.  This covers sf == 0, mx < 3, lm == 0;
 * anything else still goes to the C.
 *
 *   r4 = psxCP2Regs *r
 *   r5 = const s16 *v   three contiguous halfwords
 *   r6 = const s16 *m   nine contiguous halfwords, m11 m12 m13 m21 ... m33
 *   r7 = const s32 *t   three words; the caller passes zeros for cv == 3
 *
 * Bit-exact with gte_mvmva_c: 64-bit accumulate, the A1/A2/A3 bounds check,
 * then limB1/2/3.  MAC1..3 are at r4+100/104/108 and IR1..3 at r4+36/40/44,
 * both out of displacement range, so the two bases are kept in registers and
 * the stores are indexed.  No calls, so PR is untouched.
 */
	.align	2
	.global	_gte_mvmva_sf0_asm
	.type	_gte_mvmva_sf0_asm, @function
_gte_mvmva_sf0_asm:
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov.l	r14, @-r15

	mov.w	@r5+, r8		/* vx, sign-extended */
	mov.w	@r5+, r9		/* vy */
	mov.w	@r5+, r10		/* vz */

	mov.l	90f, r0
	mov.l	@r0, r13		/* r13 = gte_hot.flag */
	mov	r4, r14
	add	#64, r14		/* MAC1..3 at @(36,r14)/40/44 */
	mov	r4, r5
	add	#32, r5			/* IR1..3  at @(4,r5)/8/12   */
	mova	91f, r0
	mov	r0, r1			/* r1 = flag table */
	mov	#36, r2			/* r2 = MAC displacement */

70:	/* ---- one component ---- */
	mov.l	@r7+, r0		/* t[i] */
	mov	r0, r11
	mov	#12, r3
	shld	r3, r11			/* lo = t << 12 */
	mov	r0, r12
	mov	#-20, r3
	shad	r3, r12			/* hi = t >> 20, arithmetic */

	mov.w	@r6+, r0
	dmuls.l	r0, r8			/* m * vx, full signed 64 */
	sts	macl, r0
	sts	mach, r3
	clrt
	addc	r0, r11
	addc	r3, r12

	mov.w	@r6+, r0
	dmuls.l	r0, r9			/* m * vy */
	sts	macl, r0
	sts	mach, r3
	clrt
	addc	r0, r11
	addc	r3, r12

	mov.w	@r6+, r0
	dmuls.l	r0, r10			/* m * vz */
	sts	macl, r0
	sts	mach, r3
	clrt
	addc	r0, r11
	addc	r3, r12

	/*
	 * A1/A2/A3.  The 64-bit value fits in s32 exactly when the high word
	 * is the sign extension of the low one; when it does not, the sign of
	 * the high word says which way it went.  MAC is the low word either
	 * way, which is what the C's (s32) cast does.
	 */
	mov	r11, r3
	shal	r3			/* T = bit 31 of lo */
	subc	r3, r3			/* r3 = -T: 0 or -1 */
	cmp/eq	r3, r12
	bt	72f
	cmp/pz	r12
	bf	71f
	mov.l	@r1, r0			/* overflowed up */
	bra	72f
	 or	r0, r13
71:	mov.l	@(4,r1), r0		/* overflowed down */
	or	r0, r13

72:	mov	r2, r0
	mov.l	r11, @(r0,r14)		/* MAC */

	/* limB1/2/3: clamp to s16.  exts.w round-trips iff it is in range. */
	exts.w	r11, r3
	cmp/eq	r3, r11
	bt	75f
	mov.l	@(8,r1), r0		/* the clamp raised a flag */
	or	r0, r13
	mov	#1, r3
	shll8	r3
	shll8	r3
	shlr	r3			/* 0x8000 */
	cmp/pz	r11
	bf	73f
	bra	74f
	 add	#-1, r3			/* 0x7fff */
73:	neg	r3, r3			/* -0x8000 */
74:	mov	r3, r11

75:	mov	r2, r0
	add	#-32, r0
	mov.w	r11, @(r0,r5)		/* IR */

	add	#4, r2
	add	#12, r1
	mov	#48, r0
	cmp/eq	r0, r2
	bf	70b

	mov.l	90f, r0
	mov.l	r13, @r0		/* gte_hot.flag */

	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	rts
	 mov.l	@r15+, r8

	.align	2
90:	.long	_gte_hot
	/* per component: A max, A min, limB */
91:	.long	0x40000000, 0x88000000, 0x81000000
	.long	0x20000000, 0x84000000, 0x80800000
	.long	0x10000000, 0x82000000, 0x00400000
	.size	_gte_mvmva_sf0_asm, . - _gte_mvmva_sf0_asm

/*
 * ================================================================
 * THE LIGHTING FAMILY -- NCS/NCT, NCCS/NCCT, NCDS/NCDT, CC, CDP
 * ================================================================
 *
 * From bloom's `fuckGNULightning` branch (`9c6dd3d`), where these are leaves
 * under fgl's no-ABI calling convention.  The arithmetic is taken verbatim;
 * only ILEAF_ENTER and ILEAF_LEAVE are rewritten, because fgl finds the COP2
 * file through GBR and returns GBR in r3, and here the file arrives in r4 and
 * the op word in r5 as ordinary C arguments.
 *
 * These are integer, not FPU: pcsx's gte.c arithmetic done exactly, flags
 * included -- sequential 44-bit overflow per addend, the clamps, the master
 * bit.  That makes them *more* faithful than the C in gte_fpu.c, which takes
 * the float shortcut and hardcodes sf = 1 and lm = 1 because no game was seen
 * to ask for anything else.  These decode both from the op word.
 *
 * Why they exist at all: this family has had no assembly, and it is 11.7% of
 * Crash Bandicoot's GTE commands (NCCT alone 12,262) and 4.4% of Colin
 * McRae's.  On Spyro, GT and F1 it is under 3% and this changes nothing.
 *
 * Register plan (all of it fgl's):
 *   r4   the file                 r2   file + 32: IR0 @0, IR1..3 @4/8/12
 *   r6   file + 64: SZ @0..12, RGB0..2 @16/20/24, MAC0 @32, MAC1..3 @36/40/44
 *   r3   the flags, no master bit (ILEAF_LEAVE adds it and writes FLAG)
 *   r14  -shift (0 or -12)        r13  lm's low bound (0 or -0x8000)
 *   r15  the op word is at @(0,r15) after ILEAF_ENTER
 *
 * The 44-bit accumulator is kept the way pcsx keeps it: as a 64-bit value
 * scaled by 2^20, so a signed overflow of the high word IS the 44-bit
 * overflow, `addv`/`subv` report it, and the final `>> (shift + 20)` is
 * either the high word or a 12-bit funnel shift.
 */
/* ---- the small building blocks ---- */

/* The flag bits, built with shifts so the cold stubs carry no literals. */
.macro FLAG_A_POS n		/* 1 << (31 - n): 30, 29, 28 */
	.if \n == 1
	mov	#0x40, r0
	.elseif \n == 2
	mov	#0x20, r0
	.else
	mov	#0x10, r0
	.endif
	shll16	r0
	shll8	r0
.endm
.macro FLAG_A_NEG n		/* 1 << (28 - n): 27, 26, 25 */
	.if \n == 1
	mov	#8, r0
	.elseif \n == 2
	mov	#4, r0
	.else
	mov	#2, r0
	.endif
	shll16	r0
	shll8	r0
.endm

/* hi:lo (x 2^20) += p << 20, with the 44-bit overflow flags of component n.
 * r1 = p on entry and is destroyed; r0 scratch. */
.macro MACADD n, lo, hi
	mov	r1, r0
	shll16	r0
	shll2	r0
	shll2	r0			/* p << 20 */
	clrt
	addc	r0, \lo
	mov	#-12, r0
	shad	r0, r1			/* p >> 12 */
	movt	r0
	add	r0, r1			/* + carry */
	addv	r1, \hi
	bf	8f
	cmp/pz	\hi
	bt	1f
	FLAG_A_POS \n			/* wrapped negative: overflowed up */
	bra	2f
	 or	r0, r3
1:	FLAG_A_NEG \n
	or	r0, r3
2:
8:
.endm

/* r0 = (hi:lo x 2^20) >> (shift + 20).  r1 scratch. */
.macro EXTR20 lo, hi
	mov	\hi, r0
	tst	r14, r14
	bf	3f			/* shift 12: the high word */
	shll8	r0
	shll2	r0
	shll2	r0
	mov	\lo, r1
	shlr16	r1
	shlr2	r1
	shlr2	r1
	or	r1, r0
3:
.endm

/* r0 = (hi:lo, plain 64-bit) >> shift.  r1 scratch. */
.macro EXTR0 lo, hi
	mov	\lo, r0
	tst	r14, r14
	bt	3f			/* shift 0: the low word */
	shlr8	r0
	shlr2	r0
	shlr2	r0
	mov	\hi, r1
	shll16	r1
	shll2	r1
	shll2	r1
	or	r1, r0
3:
.endm

/* limB\n: clamp reg to [min, 0x7fff], flag bit 25 - n.  r0 scratch. */
.macro LIMB reg, n, min
	mov	#-1, r0
	shlr16	r0
	shlr	r0			/* 0x7fff */
	cmp/gt	r0, \reg
	bt	5f
	cmp/ge	\min, \reg
	bt	7f
	mov	\min, \reg
	bra	6f
	 nop
5:	mov	r0, \reg
6:	mov	#0x40, r0
	shll16	r0
	.if \n == 1
	shll2	r0
	.elseif \n == 2
	shll	r0
	.endif
	or	r0, r3
7:
.endm

/* limC\n on reg >> 4 into byte n-1 of r12, flag bit 22 - n.  reg destroyed. */
.macro LIMC reg, n
	mov	#-4, r0
	shad	r0, \reg
	mov	#-1, r0
	extu.b	r0, r0			/* 255 */
	cmp/hi	r0, \reg
	bf	2f
	cmp/pz	\reg
	bt	1f
	mov	#0, \reg
	bra	3f
	 nop
1:	mov	r0, \reg
3:	.if \n == 1
	mov	#0x20, r0
	.elseif \n == 2
	mov	#0x10, r0
	.else
	mov	#8, r0
	.endif
	shll16	r0
	or	r0, r3
2:	.if \n == 1
	mov	\reg, r12
	.elseif \n == 2
	shll8	\reg
	or	\reg, r12
	.else
	shll16	\reg
	or	\reg, r12
	.endif
.endm

/* r13 = lm's low bound from the op word at @(off, r15).  r0, r1 scratch. */
.macro LMB off
	mov.l	@(\off, r15), r0
	mov	#4, r1
	shll8	r1			/* 1 << 10 */
	tst	r1, r0
	movt	r13
	neg	r13, r13		/* -1 when lm == 0 */
	mov	#1, r0
	shll16	r0
	shlr	r0
	neg	r0, r0			/* -0x8000 */
	and	r0, r13
.endm

/* RGB0 = RGB1, RGB1 = RGB2 */
.macro SHIFT_RGB
	mov.l	@(20, r6), r0
	mov.l	r0, @(16, r6)
	mov.l	@(24, r6), r0
	mov.l	r0, @(20, r6)
.endm

/* The colour word in r12 gets CODE and lands at file + 64 + (r8 & 0xff). */
.macro RGB_OUT
	mov.l	@(24, r4), r1
	mov	#-1, r0
	shll16	r0
	shll8	r0
	and	r0, r1
	or	r1, r12
	mov	r8, r0
	and	#0xff, r0
	mov.l	r12, @(r0, r6)
.endm

/*
 * Prologue and epilogue for the family.  fgl's originals are leaves: entered
 * by `jsr` with GBR holding the state block, the file at GBR + 264, the op
 * word already in a register, and `rts` handing GBR back in r3.  There is no
 * GBR here and the recompiler calls through the C ABI, so these two are the
 * only things that change -- the file arrives in r4 and the op word in r5,
 * exactly what the body wants, so the four instructions fgl spends deriving
 * the base off GBR turn into one `mov`.  Everything between them is fgl's,
 * unaltered, because it was already written against r4/r2/r6.
 */
.macro ILEAF_ENTER
	sts.l	pr, @-r15
	mov.l	r7, @-r15
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov.l	r14, @-r15
	mov.l	r5, @-r15		/* the op word, @(0, r15) */
	mov	r5, r1
	mov	r4, r2
	add	#32, r2
	mov	r4, r6
	add	#64, r6
	mov	#0, r3
	mov	#8, r0
	shll16	r0			/* sf, 1 << 19 */
	tst	r0, r1
	movt	r14
	add	#-1, r14		/* -1 when sf, else 0 */
	shll2	r14
	mov	r14, r0
	add	r0, r14
	add	r0, r14			/* -12 or 0 */
	LMB	0
.endm

.macro ILEAF_LEAVE
	mov.l	98f, r0
	tst	r0, r3
	bt	99f
	mov	#1, r0
	rotr	r0
	or	r0, r3			/* the master bit */
99:	mov	#63, r0
	shll2	r0			/* 252 */
	mov.l	r3, @(r0, r4)		/* FLAG */
	add	#4, r15
	mov.l	@r15+, r14
	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	mov.l	@r15+, r7
	lds.l	@r15+, pr
	rts
	 nop
	.align	2
98:	.long	0x7f87e000
.endm

/*
 * One component of pcsx's runColor2 on the value in r5:
 *   ir  = limB(((FC << 12) - (mac << 20 >> 20... i.e. FC<<12 - mac) >> shift), lm = 0)
 *   mac = (IR0 * ir + (s64)mac) >> shift
 * r7 = IR0, r9 = file + 212 (RFC/GFC/BFC).  r0, r1, r10, r11 scratch.
 */
.macro RC2_CORE n
	mov.l	@(4 * (\n - 1), r9), r11	/* hi = FC */
	mov	r5, r0
	shll16	r0
	shll2	r0
	shll2	r0			/* mac << 20 */
	mov	#0, r1
	clrt
	subc	r0, r1			/* lo = -(mac << 20), T = borrow */
	mov	#-12, r0
	mov	r5, r10
	shad	r0, r10			/* mac >> 12 */
	movt	r0
	add	r0, r10			/* + borrow */
	subv	r10, r11		/* hi -= ; T = 44-bit overflow */
	bf	8f
	cmp/pz	r11
	bt	1f
	FLAG_A_POS \n
	bra	2f
	 or	r0, r3
1:	FLAG_A_NEG \n
	or	r0, r3
2:
8:	mov	r11, r10
	tst	r14, r14
	bf	3f			/* shift 12: the high word */
	shll8	r10
	shll2	r10
	shll2	r10
	shlr16	r1
	shlr2	r1
	shlr2	r1
	or	r1, r10
3:	mov	#1, r1
	shll16	r1
	shlr	r1
	neg	r1, r1			/* -0x8000: this clamp ignores lm */
	LIMB	r10, \n, r1
	mul.l	r7, r10
	sts	macl, r0		/* IR0 * ir */
	mov	r5, r11
	shll	r11
	subc	r11, r11		/* sign of mac */
	mov	r0, r10
	shll	r10
	subc	r10, r10		/* sign of the product */
	clrt
	addc	r0, r5			/* lo */
	addc	r10, r11		/* hi */
	tst	r14, r14
	bt	4f			/* shift 0: the low word */
	shlr8	r5
	shlr2	r5
	shlr2	r5
	shll16	r11
	shll2	r11
	shll2	r11
	or	r11, r5
4:
.endm
	.align	2
gl_zeros:
	.long	0, 0, 0

/*
 * pcsx's gteMAC123f: MAC1..3 = (t << 12 + m * v) >> shift, with the 44-bit
 * overflow flags raised per addend, written to the file.
 *   r5 = the vector (three s16), r1 = its stride (2: V0..2, 4: IR1..3)
 *   r7 = the matrix, nine s16 in a row     r11 = the translation, 3 x s32
 * Clobbers r0, r1, r5, r7, r9, r10, r11, r12, r13.  Preserves r8.
 */
.macro MACROW n
	mov.l	@r11+, r13		/* hi = t: t << 12 in x 2^20 form */
	mov	#0, r12
	mov.w	@r7+, r1
	mul.l	r9, r1
	sts	macl, r1
	MACADD	\n, r12, r13
	mov.w	@r7+, r1
	mul.l	r10, r1
	sts	macl, r1
	MACADD	\n, r12, r13
	mov.w	@r7+, r1
	mul.l	r5, r1
	sts	macl, r1
	MACADD	\n, r12, r13
	EXTR20	r12, r13
	mov.l	r0, @(32 + 4 * \n, r6)
.endm

gl_mac123f:
	mov.w	@r5, r9
	add	r1, r5
	mov.w	@r5, r10
	add	r1, r5
	mov.w	@r5, r5			/* vx r9, vy r10, vz r5 */
	MACROW 1
	MACROW 2
	MACROW 3
	rts
	 nop

/* ---- the lighting family ---- */

/*
 * One component of pcsx's runColor1 after the colour matrix.  r5 = MACn
 * from the file; r8 = dest | 0x100 (ncc: modulate by the colour) | 0x200
 * (ncd: modulate, then the depth cue) | 0x400 (writeback: MAC, IR).
 */
.macro RC1_COMP n
	mov.l	@(32 + 4 * \n, r6), r5
	mov	r8, r0
	shlr8	r0
	tst	#3, r0
	bt	20f			/* NCS: the colour stage is the result */
	mov	r5, r10
	LIMB	r10, \n, r13		/* ir = limB(mac, lm) */
	mov.l	@(24, r4), r0
	.if \n == 2
	shlr8	r0
	.elseif \n == 3
	shlr16	r0
	.endif
	extu.b	r0, r0
	mul.l	r10, r0
	sts	macl, r5
	shll2	r5
	shll2	r5			/* (C * ir) << 4 */
	mov	r8, r0
	shlr8	r0
	tst	#2, r0
	bf	21f
	bra	20f
	 shad	r14, r5			/* NCC: >> shift */
21:	RC2_CORE \n			/* NCD: the depth cue */
20:	mov	r8, r0
	shlr8	r0
	tst	#4, r0
	bt	22f
	mov.l	r5, @(32 + 4 * \n, r6)	/* MACn */
	mov	r5, r10
	LIMB	r10, \n, r13
	mov	r10, r0
	bra	23f
	 mov.w	r0, @(4 * \n, r2)	/* IRn */
22:	mov	r5, r10
	LIMB	r10, \n, r13		/* flags only */
23:	LIMC	r5, \n
.endm

/* CC / CDP enter here: the colour matrix on IR as it stands. */
gl_rc3:
	sts.l	pr, @-r15
	bra	gl_rc1_c
	 nop

/* One vertex of NCS/NCT/NCCS/NCCT/NCDS/NCDT: r5 = the vertex, r1 = 2,
 * r8 = dest | mode bits. */
gl_rc1:
	sts.l	pr, @-r15
	mov	r4, r7
	add	#80, r7
	add	#80, r7			/* the light matrix @160 */
	mov.l	92f, r11		/* no translation */
	bsr	gl_mac123f
	 nop
	bra	93f
	 nop
	.align	2
92:	.long	gl_zeros
93:	LMB	4
	mov.l	@(36, r6), r5
	LIMB	r5, 1, r13
	mov	r5, r0
	mov.w	r0, @(4, r2)
	mov.l	@(40, r6), r5
	LIMB	r5, 2, r13
	mov	r5, r0
	mov.w	r0, @(8, r2)
	mov.l	@(44, r6), r5
	LIMB	r5, 3, r13
	mov	r5, r0
	mov.w	r0, @(12, r2)		/* IR = limB(light), the next vector */
gl_rc1_c:
	mov	r2, r5
	add	#4, r5
	mov	#4, r1
	mov	r4, r7
	add	#96, r7
	add	#96, r7			/* the colour matrix @192 */
	mov	r4, r11
	add	#90, r11
	add	#90, r11		/* the background colour @180 */
	bsr	gl_mac123f
	 nop
	LMB	4
	mov	#106, r9
	shll	r9
	add	r4, r9			/* the far colour @212 */
	mov.w	@r2, r7			/* IR0 */
	mov	#0, r12
	RC1_COMP 1
	RC1_COMP 2
	RC1_COMP 3
	RGB_OUT
	lds.l	@r15+, pr
	rts
	 nop

/* ---- the lighting family, one entry per command ---- */

.macro LIGHT_LEAF name, mode
	.balign	4
	.global	\name
	.type	\name, @function
\name:
	ILEAF_ENTER
	SHIFT_RGB
	mov	r4, r5			/* V0 */
	mov	#2, r1
	mov	#\mode, r8
	shll8	r8
	add	#24, r8			/* RGB2 */
	bsr	gl_rc1
	 nop
	ILEAF_LEAVE
	.size	\name, . - \name
.endm

.macro LIGHT_LEAF_T name, mode
	.balign	4
	.global	\name
	.type	\name, @function
\name:
	ILEAF_ENTER
	mov	r4, r5
	mov	#2, r1
	mov	#\mode - 4, r8
	shll8	r8
	add	#16, r8			/* RGB0, no writeback */
	bsr	gl_rc1
	 nop
	mov	r4, r5
	add	#8, r5
	mov	#2, r1
	mov	#\mode - 4, r8
	shll8	r8
	add	#20, r8			/* RGB1 */
	bsr	gl_rc1
	 nop
	mov	r4, r5
	add	#16, r5
	mov	#2, r1
	mov	#\mode, r8
	shll8	r8
	add	#24, r8			/* RGB2, writeback */
	bsr	gl_rc1
	 nop
	ILEAF_LEAVE
	.size	\name, . - \name
.endm

.macro COLOUR_LEAF name, mode
	.balign	4
	.global	\name
	.type	\name, @function
\name:
	ILEAF_ENTER
	SHIFT_RGB
	mov	#\mode, r8
	shll8	r8
	add	#24, r8
	bsr	gl_rc3
	 nop
	ILEAF_LEAVE
	.size	\name, . - \name
.endm

	LIGHT_LEAF   _gte_ncs_asm,  4		/* 0x400 */
	LIGHT_LEAF_T _gte_nct_asm,  4
	LIGHT_LEAF   _gte_nccs_asm, 5		/* 0x100 | 0x400 */
	LIGHT_LEAF_T _gte_ncct_asm, 5
	LIGHT_LEAF   _gte_ncds_asm, 6		/* 0x200 | 0x400 */
	LIGHT_LEAF_T _gte_ncdt_asm, 6
	COLOUR_LEAF  _gte_cc_asm,   5
	COLOUR_LEAF  _gte_cdp_asm,  6

/* ===================================================================== */
/* NCCT, in float                                                        */
/* ===================================================================== */

/*
 * The same command as _gte_ncct_asm above, done approximately on the FPU.
 * `GTE_FAST` bit 6 chooses between them; both stay in the binary, because the
 * integer one is bit-exact against pcsx's gte.c and is what you want when
 * something looks wrong.
 *
 * Why this command and not the others: Crash Bandicoot issues NCCT 12262
 * times in a bench window, 15% of its GTE commands and **53.5% of its GTE
 * instructions** at 1756.6 SH-4 instructions a call.  Nothing else in the
 * lighting family appears in any bench title except Colin's 6417 NCCT.
 *
 * NCCT is three vertices through two 3x3 matrices:
 *
 *	IR  = clamp(L  . N,        0, 0x7fff)		the light matrix
 *	IR  = clamp(BK + LC . IR,  0, 0x7fff)		the colour matrix
 *	MAC = (RGBC * IR) >> 8    RGB = clamp(MAC >> 4, 0, 0xff)
 *
 * Four things make this cheap, and three of them are only available because
 * it is the *triple*:
 *
 * **Two XMTRX loads, not six.**  `ftrv` takes its matrix from the back bank,
 * so a per-vertex loop would reload it twice a vertex.  Ordering by matrix
 * instead -- all three normals through the light matrix, then all three
 * results through the colour matrix -- costs two loads for the whole command.
 *
 * **Three `ftrv` in flight.**  fv0, fv4 and fv8 hold the three vertices at
 * once and the three transforms are independent, so the 4-cycle F0/F1 lock
 * one `ftrv` takes (table 8.3 #234) is covered by the next one's issue rather
 * than stalling.  fv12 is left for constants.
 *
 * **The background colour is free.**  `ftrv` is 4x4 and the fourth column of
 * the cached colour matrix is already BK (gte_col[2][1], the cv*8+5 entry),
 * with w = 1.0 carried in fr3/fr7/fr11.  Stage two's add happens inside the
 * same instruction as its multiply.
 *
 * **Half the clamp is branchless**, which is the part that usually kills this:
 *
 *	max(0, x) == (x + |x|) / 2
 *
 * `fabs` is LS group with latency 0 (table 8.3 #182), so two of the three
 * instructions dual-issue against the FE stream.  The upper bound at 0x7fff
 * is a not-taken branch; it was left out first, and Crash said no.  See
 * GLF_POS3.
 *
 * The final clamp cannot underflow -- IR is >= 0 by the line above and RGBC is
 * three unsigned bytes -- so it is a single unsigned compare against 255
 * rather than a pair of signed ones.
 *
 * And the RGB FIFO shifts three times over three vertices, so RGB0, RGB1 and
 * RGB2 simply *are* the three results: three stores, no shifting.
 *
 * What is not faithful: FLAG is written as zero rather than accumulated, and
 * the arithmetic is float32 throughout.  Measured against integer gte.c on
 * 409831 real NCCT calls in Crash: no RGB channel off by more than 1, 0.5% of
 * channels off by 1 at all, worst MAC/IR delta 2.  328 SH-4 instructions a
 * call against the integer assembly's 1756.6.
 */

/* limB(x, lm=1): clamp to 0 .. 0x7fff, three components at a time.
 *
 * The low end is branchless -- max(0, x) is (x + |x|) / 2, and `fabs` is an
 * LS instruction with no latency, so two of every three issue alongside the
 * FE stream.  Two scratch registers, not three, because fr13 has to hold
 * 32767.0: the fadds still interleave, which is the point of doing three.
 *
 * The high end is a branch, and it is not optional.  Dropping it was the
 * first thing measured and it cost real colour: on Crash, 8.3% of the RGB
 * channels a lighting command hands the guest came out more than 8 off, worst
 * 66, because the *second* stage saturates routinely -- the background colour
 * alone puts MAC near 0x7fff before the light matrix adds anything.  The
 * branch is not taken on the common path, which is one cycle. */
.macro GLF_POS3 a, b, c
	fmov	\a, fr14
	fmov	\b, fr15
	fabs	fr14
	fabs	fr15
	fadd	fr14, \a		/* 2 * max(0, x) */
	fadd	fr15, \b
	fmov	\c, fr14
	fabs	fr14
	fadd	fr14, \c
	fmul	fr12, \a		/* * 0.5 */
	fmul	fr12, \b
	fmul	fr12, \c
	fcmp/gt	fr13, \a		/* T = x > 32767.0 */
	bf	1f
	fmov	fr13, \a
1:	fcmp/gt	fr13, \b
	bf	2f
	fmov	fr13, \b
2:	fcmp/gt	fr13, \c
	bf	3f
	fmov	fr13, \c
3:
.endm

/* Load XMTRX from a cached 3x3 and its translation column.  Same eight moves
 * gte_mtx_load() makes; inline because this command needs two matrices and the
 * residency check in gte_mtx_use() can never hit for both. */
.macro GLF_XMTRX rot, col
	mov.l	\rot, r1
	mov.l	\col, r2
	fschg
	fmov.d	@r1+, xd0
	fmov.d	@r1+, xd2
	fmov.d	@r1+, xd4
	fmov.d	@r1+, xd6
	fmov.d	@r1+, xd8
	fmov.d	@r1+, xd10
	fmov.d	@r2+, xd12
	fmov.d	@r2+, xd14
	fschg
.endm

/* One s16 out of the register file, into an FP register as a float. */
.macro GLF_LDV off, fr
	mov.w	@(\off, r8), r0
	lds	r0, fpul
	float	fpul, \fr
.endm

/* 0 .. 255.  The value is known non-negative, so one unsigned compare does
 * both ends and the common case falls through. */
.macro GLF_B255 r
	cmp/hi	r10, \r
	bf	1f
	mov	r10, \r
1:
.endm

/* One vertex's colour: MAC = RGBC * IR / 256, the byte is MAC >> 4.
 * `last` writes MAC1-3 and IR1-3, which only the third vertex leaves behind. */
.macro GLF_OUT a, b, c, rgbofs, last
	fmul	fr13, \a
	fmul	fr14, \b
	fmul	fr15, \c
	ftrc	\a, fpul
	sts	fpul, r1
	ftrc	\b, fpul
	sts	fpul, r2
	ftrc	\c, fpul
	sts	fpul, r3
.if \last
	mov.l	r1, @(36, r9)		/* MAC1 */
	mov.l	r2, @(40, r9)		/* MAC2 */
	mov.l	r3, @(44, r9)		/* MAC3 */
	mov	r1, r0
	mov.w	r0, @(4, r13)		/* IR1 = (s16)MAC1, no clamp */
	mov	r2, r0
	mov.w	r0, @(8, r13)
	mov	r3, r0
	mov.w	r0, @(12, r13)
.endif
	shad	r11, r1			/* >> 4 */
	shad	r11, r2
	shad	r11, r3
	GLF_B255 r1
	GLF_B255 r2
	GLF_B255 r3
	shll8	r2
	shll16	r3
	mov	r12, r0			/* CODE << 24 */
	or	r1, r0
	or	r2, r0
	or	r3, r0
	mov.l	r0, @(\rgbofs, r9)
.endm

	.balign	4
	.global	_gte_ncct_fast
	.type	_gte_ncct_fast, @function
_gte_ncct_fast:
	sts.l	pr, @-r15
	mov.l	r8, @-r15
	mov.l	r9, @-r15
	mov.l	r10, @-r15
	mov.l	r11, @-r15
	mov.l	r12, @-r15
	mov.l	r13, @-r15
	mov	r4, r8

	/* Both matrices at the current generation.  gte_mtx_use_slow() checks
	 * the 3x3 before the column, so the column's generation answers for
	 * both.  It also loads XMTRX, which is wasted here -- but only on the
	 * cold path, which is a control-file write. */
	mov.l	40f, r0
	mov.l	@r0, r3			/* psxCP2CtrlGen */
	mov.l	42f, r0
	mov.l	@(16, r0), r1		/* gte_col[LIGHT][NONE].gen */
	cmp/eq	r3, r1
	bt	1f
	mov	r8, r4
	mov	#1, r5			/* GTE_MTX_LIGHT */
	mov	#3, r6			/* GTE_CV_NONE */
	mov	#7, r7			/* key = mx * 4 + cv */
	mov.l	44f, r0
	jsr	@r0
	 nop
	mov.l	40f, r0
	mov.l	@r0, r3
1:	mov.l	43f, r0
	mov.l	@(16, r0), r1		/* gte_col[COLOUR][BK].gen */
	cmp/eq	r3, r1
	bt	2f
	mov	r8, r4
	mov	#2, r5			/* GTE_MTX_COLOUR */
	mov	#1, r6			/* cv 1: CP2C[13], the background */
	mov	#9, r7
	mov.l	44f, r0
	jsr	@r0
	 nop
2:
	mov	r8, r9
	add	#64, r9			/* MAC and RGB are past disp reach */
	mov	#-1, r10
	shlr16	r10
	shlr8	r10			/* 255 */
	mov	#-4, r11
	mov	r8, r13
	add	#32, r13		/* IR1-3 are past mov.w's 30-byte reach */

	mov.l	45f, r0
	fmov.s	@r0, fr12		/* 0.5 */
	add	#8, r0
	fmov.s	@r0, fr13		/* 32767.0, limB's upper end */

	/* ---- stage one: the light matrix on the three normals ---- */
	GLF_XMTRX 41f, 42f

	GLF_LDV	0,  fr0
	GLF_LDV	2,  fr1
	GLF_LDV	4,  fr2
	fldi1	fr3
	GLF_LDV	8,  fr4
	GLF_LDV	10, fr5
	GLF_LDV	12, fr6
	fldi1	fr7
	GLF_LDV	16, fr8
	GLF_LDV	18, fr9
	GLF_LDV	20, fr10
	fldi1	fr11

	ftrv	xmtrx, fv0
	ftrv	xmtrx, fv4
	ftrv	xmtrx, fv8

	GLF_POS3 fr0, fr1, fr2
	GLF_POS3 fr4, fr5, fr6
	GLF_POS3 fr8, fr9, fr10

	/* ---- stage two: the colour matrix, background in column four ---- */
	GLF_XMTRX 46f, 43f

	ftrv	xmtrx, fv0
	ftrv	xmtrx, fv4
	ftrv	xmtrx, fv8

	GLF_POS3 fr0, fr1, fr2
	GLF_POS3 fr4, fr5, fr6
	GLF_POS3 fr8, fr9, fr10

	/* ---- stage three: modulate by RGBC and pack ---- */
	mov.l	@(24, r8), r7		/* RGBC */
	mov.l	45f, r0
	add	#4, r0
	fmov.s	@r0, fr12		/* 1/256 */

	mov	r7, r0
	extu.b	r0, r0
	lds	r0, fpul
	float	fpul, fr13
	mov	r7, r0
	shlr8	r0
	extu.b	r0, r0
	lds	r0, fpul
	float	fpul, fr14
	mov	r7, r0
	shlr16	r0
	extu.b	r0, r0
	lds	r0, fpul
	float	fpul, fr15
	fmul	fr12, fr13		/* R / 256 */
	fmul	fr12, fr14
	fmul	fr12, fr15

	mov.l	47f, r0
	and	r0, r7
	mov	r7, r12			/* CODE << 24 */

	GLF_OUT	fr0,  fr1, fr2,  16, 0	/* RGB0 */
	GLF_OUT	fr4,  fr5, fr6,  20, 0	/* RGB1 */
	GLF_OUT	fr8,  fr9, fr10, 24, 1	/* RGB2, and the MAC/IR left behind */

	/* FLAG.  Not accumulated on this path; say zero rather than stale. */
	mov	r9, r3
	add	#124, r3
	add	#64, r3			/* CP2C[31], 252 bytes in */
	mov	#0, r0
	mov.l	r0, @r3

	/* XMTRX is not what the cache thinks it is any more. */
	mov.l	48f, r0
	mov	#-1, r1
	mov.l	r1, @(16, r0)		/* gte_hot.xmtrx_key */
	mov.l	r1, @(20, r0)		/* gte_hot.xmtrx_mx  */

	mov.l	@r15+, r13
	mov.l	@r15+, r12
	mov.l	@r15+, r11
	mov.l	@r15+, r10
	mov.l	@r15+, r9
	mov.l	@r15+, r8
	lds.l	@r15+, pr
	rts
	 nop

	.balign	4
40:	.long	_psxCP2CtrlGen
41:	.long	_gte_rot + 64		/* gte_rot[GTE_MTX_LIGHT].m       */
42:	.long	_gte_col + 224		/* gte_col[GTE_MTX_LIGHT][3].t    */
43:	.long	_gte_col + 288		/* gte_col[GTE_MTX_COLOUR][1].t   */
44:	.long	_gte_mtx_use_slow
45:	.long	gl_ncct_const
46:	.long	_gte_rot + 128		/* gte_rot[GTE_MTX_COLOUR].m      */
47:	.long	0xff000000
48:	.long	_gte_hot
	.size	_gte_ncct_fast, . - _gte_ncct_fast

	.balign	4
gl_ncct_const:
	.float	0.5
	.float	0.00390625		/* 1 / 256 */
	.float	32767.0
