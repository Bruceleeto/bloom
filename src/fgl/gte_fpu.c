// SPDX-License-Identifier: GPL-2.0-only
/*
 * The GTE's coordinate transform, on the SH-4 FPU.
 *
 * The 3x3 product plus translation is one `ftrv`: the matrix rows carry the
 * >> 12 as a 1/4096 premultiply and the translation rides in the fourth
 * column against a w of 1.0.  The perspective divide is an `fdiv`, which is
 * more accurate than the PS1's table and so a different answer (`gte_divide`).
 * MAC is `ftrc` (truncate) where the reference floors: one apart on negatives
 * with a fraction, shipped as-is.  Limits and flags are integer and exact.
 *
 * XMTRX belongs to this file; `src/pvr.c` uses multiplies instead under the
 * same option, so the two never both own the back bank.
 *
 * Copyright (C) 2026 bloom contributors
 */

#include <stdio.h>
#include <string.h>

#include <libpcsxcore/r3000a.h>
#include <libpcsxcore/gte.h>
#include <libpcsxcore/gte_divider.h>
#include <libpcsxcore/psxinterpreter.h>

/* The assembly path (src/gte_rtp.S): native on the SH-4, and on Linux the
 * same code under the SH-4 interpreter (fgl/gte_sh4.c, -DFGL_GTE_ASM). */
#if defined(__sh__) || defined(FGL_GTE_ASM)
#define GTE_ASM 1
#endif

#include "bloom-config.h"
#include "gte_fpu.h"

/*
 * Every command is its own function: a shared dispatch frame is sized by the
 * largest body and the smallest commands pay all of it.  NCLIP stays inline
 * because it is a third of all GTE calls and small.
 */
#define GTE_CMD static __attribute__((noinline))

/* ------------------------------------------------------------------ */
/* The COP2 file, by the names gte.c uses                              */
/* ------------------------------------------------------------------ */

#define C_R11 (r->CP2C.p[0].sw.l)
#define C_R12 (r->CP2C.p[0].sw.h)
#define C_R13 (r->CP2C.p[1].sw.l)
#define C_R21 (r->CP2C.p[1].sw.h)
#define C_R22 (r->CP2C.p[2].sw.l)
#define C_R23 (r->CP2C.p[2].sw.h)
#define C_R31 (r->CP2C.p[3].sw.l)
#define C_R32 (r->CP2C.p[3].sw.h)
#define C_R33 (r->CP2C.p[4].sw.l)
#define C_TRX (((s32 *)r->CP2C.r)[5])
#define C_TRY (((s32 *)r->CP2C.r)[6])
#define C_TRZ (((s32 *)r->CP2C.r)[7])
#define C_OFX (((s32 *)r->CP2C.r)[24])
#define C_OFY (((s32 *)r->CP2C.r)[25])
#define C_H   (r->CP2C.p[26].w.l)
#define C_DQA (r->CP2C.p[27].sw.l)
#define C_DQB (((s32 *)r->CP2C.r)[28])
#define C_FLAG (r->CP2C.r[31])

#define D_IR0 (r->CP2D.p[8].sw.l)
#define D_IR1 (r->CP2D.p[9].sw.l)
#define D_IR2 (r->CP2D.p[10].sw.l)
#define D_IR3 (r->CP2D.p[11].sw.l)
#define D_SXY0 (r->CP2D.r[12])
#define D_SXY1 (r->CP2D.r[13])
#define D_SXY2 (r->CP2D.r[14])
#define D_SX2 (r->CP2D.p[14].sw.l)
#define D_SY2 (r->CP2D.p[14].sw.h)
#define D_SZ0 (r->CP2D.p[16].w.l)
#define D_SZ1 (r->CP2D.p[17].w.l)
#define D_SZ2 (r->CP2D.p[18].w.l)
#define D_SZ3 (r->CP2D.p[19].w.l)
#define D_MAC0 (((s32 *)r->CP2D.r)[24])
#define D_MAC1 (((s32 *)r->CP2D.r)[25])
#define D_MAC2 (((s32 *)r->CP2D.r)[26])
#define D_MAC3 (((s32 *)r->CP2D.r)[27])

/*
 * LLM and LCM are not named individually: they sit at the rotation matrix's
 * stride (five packed words then three translations, eight words apart), so
 * LLM is matrix 1 with BK as its translation column and LCM matrix 2 with FC.
 */
#define C_RFC (((s32 *)r->CP2C.r)[21])
#define C_GFC (((s32 *)r->CP2C.r)[22])
#define C_BFC (((s32 *)r->CP2C.r)[23])
#define C_ZSF3 (r->CP2C.p[29].sw.l)
#define C_ZSF4 (r->CP2C.p[30].sw.l)

/* The colour FIFO, and the source colour the lighting commands modulate. */
#define D_RGB   (r->CP2D.r[6])
#define D_R     (r->CP2D.p[6].b.l)
#define D_G     (r->CP2D.p[6].b.h)
#define D_B     (r->CP2D.p[6].b.h2)
#define D_CODE  (r->CP2D.p[6].b.h3)
#define D_OTZ   (r->CP2D.p[7].w.l)
#define D_RGB0  (r->CP2D.r[20])
#define D_R0    (r->CP2D.p[20].b.l)
#define D_G0    (r->CP2D.p[20].b.h)
#define D_B0    (r->CP2D.p[20].b.h2)
#define D_RGB1  (r->CP2D.r[21])
#define D_RGB2  (r->CP2D.r[22])
#define D_R2    (r->CP2D.p[22].b.l)
#define D_G2    (r->CP2D.p[22].b.h)
#define D_B2    (r->CP2D.p[22].b.h2)
#define D_CODE2 (r->CP2D.p[22].b.h3)
#define D_SX0 (r->CP2D.p[12].sw.l)
#define D_SY0 (r->CP2D.p[12].sw.h)
#define D_SX1 (r->CP2D.p[13].sw.l)
#define D_SY1 (r->CP2D.p[13].sw.h)

/* The op word's selector fields, by the names the core's macros use. */
#define GTE_SF(op) (((op) >> 19) & 1)
#define GTE_MX(op) (((op) >> 17) & 3)
#define GTE_V(op)  (((op) >> 15) & 3)
#define GTE_CV(op) (((op) >> 13) & 3)
#define GTE_LM(op) (((op) >> 10) & 1)

/* The vertex registers are three contiguous halfwords at the base of the data
 * file - VX0, VY0, VZ0 - which is why the transform takes one pointer. */
#define D_V(n) ((const s16 *)&r->CP2D.r[(n) * 2])

/* ------------------------------------------------------------------ */
/* Flags                                                               */
/* ------------------------------------------------------------------ */

/* The bits are gte.c's.  limB3 sets bit 22 and NOT the master bit 31, while
 * limB1 and limB2 set both.  Games read FLAG. */
/* The state every command reads or writes: 32-byte aligned and under 32
 * bytes, so it is one cache line. */
struct gte_hot {
	u32 flag;		/* accumulated during a command          */
	float ofx, ofy;		/* OFX/OFY in screen units, not 16.16    */
	u32 ofs_gen;		/* psxCP2CtrlGen when ofx/ofy were built */
	int xmtrx_key;		/* (matrix, column) XMTRX holds, -1 for none */
	int xmtrx_mx;		/* which matrix, for the column-only reload   */
	u32 xmtrx_serial;	/* that matrix's serial when it was loaded    */
	u32 xmtrx_col_serial;
} __attribute__((aligned(32)));

struct gte_hot gte_hot = { .xmtrx_key = -1, .xmtrx_mx = -1 };

/* Would the RTPS/RTPT leaf take its XMTRX and offsets as they stand?  For
 * FGL_STATS's hit count; the leaf makes the same test itself. */
int gte_leaf_xmtrx_hit(void)
{
	return gte_hot.xmtrx_key == -2 && gte_hot.xmtrx_serial == psxCP2RtGen;
}
_Static_assert(sizeof(struct gte_hot) == 32, "gte_sh4.c maps 32 bytes");

#define gte_flag	(gte_hot.flag)

static s32 gte_lim(s32 v, s32 max, s32 min, u32 flag)
{
	if (v > max) {
		gte_flag |= flag;
		return max;
	}
	if (v < min) {
		gte_flag |= flag;
		return min;
	}
	return v;
}

/*
 * The 32-bit overflow check.  It raises the flag and does not clamp: the
 * reference's MAC wraps.  The saturating form is the FPU path's below.
 */
static s32 gte_bounds(s64 v, u32 maxflag, u32 minflag)
{
	if (v > 0x7fffffff)
		gte_flag |= maxflag;
	else if (v < -(s64)0x80000000)
		gte_flag |= minflag;

	return (s32)v;
}

/* `lm` selects the lower bound: 0 for the transforms, 1 for lighting. */
static s32 gte_limB1(s32 v, int lm)
{
	return gte_lim(v, 0x7fff, lm ? 0 : -0x8000, (1u << 31) | (1u << 24));
}

static s32 gte_limB2(s32 v, int lm)
{
	return gte_lim(v, 0x7fff, lm ? 0 : -0x8000, (1u << 31) | (1u << 23));
}

static s32 gte_limB3(s32 v, int lm)
{
	return gte_lim(v, 0x7fff, lm ? 0 : -0x8000, (1u << 22));
}

#define limB1(a) gte_limB1((a), 0)
#define limB2(a) gte_limB2((a), 0)
#define limB3(a) gte_limB3((a), 0)
#define limC1(a) gte_lim((a), 0x00ff, 0x0000, (1u << 21))
#define limC2(a) gte_lim((a), 0x00ff, 0x0000, (1u << 20))
#define limC3(a) gte_lim((a), 0x00ff, 0x0000, (1u << 19))
#define limD(a)  gte_lim((a), 0xffff, 0x0000, (1u << 31) | (1u << 18))
#define limG1(a) gte_lim((a), 0x3ff, -0x400, (1u << 31) | (1u << 14))
#define limG2(a) gte_lim((a), 0x3ff, -0x400, (1u << 31) | (1u << 13))
#define limH(a)  gte_lim((a), 0x1000, 0x0000, (1u << 12))
#define F(a)     gte_bounds((a), (1u << 31) | (1u << 16), (1u << 31) | (1u << 15))

static u32 limE(u32 v)
{
	if (v > 0x1ffff) {
		gte_flag |= (1u << 31) | (1u << 17);
		return 0x1ffff;
	}
	return v;
}

/*
 * `ftrc` saturates to these on overflow, which is how the FPU path reports
 * that MAC left the 32-bit range.  A true result of exactly INT_MAX raises
 * the flag spuriously; accepted.
 */
static s32 gte_a(s32 mac, u32 maxflag, u32 minflag)
{
	if (mac == (s32)0x7fffffff)
		gte_flag |= maxflag;
	else if (mac == (s32)0x80000000)
		gte_flag |= minflag;
	return mac;
}

/* From the FPU path, where the value has already saturated. */
#define AF1(a) gte_a((a), (1u << 30), (1u << 31) | (1u << 27))
#define AF2(a) gte_a((a), (1u << 29), (1u << 31) | (1u << 26))
#define AF3(a) gte_a((a), (1u << 28), (1u << 31) | (1u << 25))

/* From 64-bit integer arithmetic, where it has not. */
#define A1(a) gte_bounds((a), (1u << 30), (1u << 31) | (1u << 27))
#define A2(a) gte_bounds((a), (1u << 29), (1u << 31) | (1u << 26))
#define A3(a) gte_bounds((a), (1u << 28), (1u << 31) | (1u << 25))

/* ------------------------------------------------------------------ */
/* The matrix cache                                                    */
/* ------------------------------------------------------------------ */

/*
 * Matrix `mx` is five packed words at `CP2C[mx * 8]`, translation `cv` three
 * words at `CP2C[cv * 8 + 5]`; every matrix stage is one of the nine
 * combinations, so one cache serves all.
 *
 * XMTRX is column-major: result[i] = sum_j xf[i + 4j] * fv[j], and eight
 * `fmov.d` fill xf0..xf15 straight from the array.  8-byte alignment is
 * required by the pair form; 32 keeps each entry in two lines.
 *
 * Validity is one compare against `psxCP2CtrlGen` (gte.c), which counts CTC2
 * writes from both engines.  Conservative: any control write invalidates.
 */
#define GTE_MTX_ROT    0
#define GTE_MTX_LIGHT  1
#define GTE_MTX_COLOUR 2

/* The translation selector for a stage that has none.  The reference spells
 * it as an out-of-range index whose accessors return zero. */
#define GTE_CV_NONE 3

/*
 * Two caches: the 3x3 per matrix, and the translation column per (matrix,
 * selector) pair, so switching TR on and off (MVMVA cv=0/cv=3 on the same
 * matrix) only reloads xf12-xf15.
 */
struct gte_rot_cache {
	float m[12];		/* xf0..xf11: the 3x3, column-major, / 4096 */
	u32 gen;		/* psxCP2CtrlGen when m[] was built         */
	u32 serial;		/* bumped whenever m[] is rebuilt           */
} __attribute__((aligned(32)));

struct gte_col_cache {
	float t[4];		/* xf12..xf15: the column, then 1.0         */
	u32 gen;
	u32 serial;
} __attribute__((aligned(32)));

static struct gte_rot_cache gte_rot[3];
struct gte_col_cache gte_col[3][4];
_Static_assert(sizeof(gte_col) == 384, "gte_sh4.c maps 384 bytes");
static u32 gte_mtx_serial;

#ifdef GTE_CACHE_STATS
static u32 gte_stat[8];
static void gte_stat_tick(void)
{
	if (++gte_stat[0] == 20000) {
		printf("GTE cache: %u uses, rot rebuilds %u, col rebuilds %u, full loads %u col loads %u\n",
		       gte_stat[0], gte_stat[1], gte_stat[3], gte_stat[5], gte_stat[6]);
		memset(gte_stat, 0, sizeof(gte_stat));
	}
}
#define GTE_STAT(n) (gte_stat[n]++)
#else
#define GTE_STAT(n) ((void)0)
#define gte_stat_tick() ((void)0)
#endif

/*
 * Rebuild on a control-file write.  Out of line, cold, and with no compare
 * against the previous words: a generation change is usually a real change.
 * The serial moves every time, so XMTRX reloads after every rebuild.
 */
#ifdef GTE_ASM
void gte_rot_convert(const u32 *src, float *m);
#endif

static void gte_rot_rebuild(const psxCP2Regs *r, int mx)
{
	struct gte_rot_cache *c = &gte_rot[mx];
	const PAIR *p = &r->CP2C.p[mx * 8];

	GTE_STAT(1);
	c->gen = psxCP2CtrlGen;
	c->serial = ++gte_mtx_serial;

#ifdef GTE_ASM
	gte_rot_convert(&r->CP2C.r[mx * 8], c->m);
	return;
#endif

	c->m[0]  = p[0].sw.l * (1.0f / 4096.0f);
	c->m[4]  = p[0].sw.h * (1.0f / 4096.0f);
	c->m[8]  = p[1].sw.l * (1.0f / 4096.0f);
	c->m[1]  = p[1].sw.h * (1.0f / 4096.0f);
	c->m[5]  = p[2].sw.l * (1.0f / 4096.0f);
	c->m[9]  = p[2].sw.h * (1.0f / 4096.0f);
	c->m[2]  = p[3].sw.l * (1.0f / 4096.0f);
	c->m[6]  = p[3].sw.h * (1.0f / 4096.0f);
	c->m[10] = p[4].sw.l * (1.0f / 4096.0f);

	c->m[3] = c->m[7] = c->m[11] = 0.0f;
}

static void gte_col_rebuild(const psxCP2Regs *r, int mx, int cv)
{
	struct gte_col_cache *c = &gte_col[mx][cv];
	int i;

	GTE_STAT(3);
	c->gen = psxCP2CtrlGen;

	if (cv == GTE_CV_NONE) {
		/* No translation: a constant column.  Built at most once - the
		 * serial only moves the first time. */
		if (c->serial)
			return;
		c->t[0] = c->t[1] = c->t[2] = 0.0f;
	} else {
		const u32 *t = &r->CP2C.r[cv * 8 + 5];

		for (i = 0; i < 3; i++)
			c->t[i] = (float)(s32)t[i];
	}

	c->t[3] = 1.0f;
	c->serial = ++gte_mtx_serial;
}

/*
 * Put a matrix and a translation column in XMTRX, skipping whatever is
 * already there.  The hot check is two compares: the column entry is at the
 * current generation (which implies its matrix is too), and XMTRX holds this
 * (matrix, column) pair.  Residency assumes nothing else touches the back
 * bank across arbitrary guest code, interrupts and thread switches; nothing
 * in the tree does while this file is built.
 */

static void gte_mtx_load(const float *m, const float *t);
static void gte_col_load(const float *t);

#define gte_xmtrx_mx		(gte_hot.xmtrx_mx)
#define gte_xmtrx_serial	(gte_hot.xmtrx_serial)

__attribute__((noinline)) void
gte_mtx_use_slow(const psxCP2Regs *r, int mx, int cv, int key)
{
	struct gte_rot_cache *rc = &gte_rot[mx];
	struct gte_col_cache *cc = &gte_col[mx][cv];
	u32 gen = psxCP2CtrlGen;

	if (rc->gen != gen)
		gte_rot_rebuild(r, mx);
	if (cc->gen != gen)
		gte_col_rebuild(r, mx, cv);

	if (gte_hot.xmtrx_mx == mx && gte_hot.xmtrx_serial == rc->serial) {
		if (gte_hot.xmtrx_col_serial != cc->serial) {
			GTE_STAT(6);
			gte_col_load(cc->t);
		}
	} else {
		GTE_STAT(5);
		gte_hot.xmtrx_mx = mx;
		gte_hot.xmtrx_serial = rc->serial;
		gte_mtx_load(rc->m, cc->t);
	}

	gte_hot.xmtrx_col_serial = cc->serial;
	gte_hot.xmtrx_key = key;
}

static inline __attribute__((always_inline)) void
gte_mtx_use(const psxCP2Regs *r, int mx, int cv)
{
	int key = mx * 4 + cv;

	gte_stat_tick();

	if (__builtin_expect(gte_col[mx][cv].gen == psxCP2CtrlGen &&
			     gte_hot.xmtrx_key == key, 1))
		return;

	gte_mtx_use_slow(r, mx, cv, key);
}

/*
 * Drop everything derived from the control file.  The generation counter
 * only sees CTC2; a reset or savestate load writes CP2C directly.
 */
#if defined(GTE_ASM) && defined(GTE_RTP_SELFTEST)
static void gte_rtp_selftest(void);
#endif

void gte_fpu_reset(void)
{
	int i;

#if defined(GTE_ASM) && defined(GTE_RTP_SELFTEST)
	gte_rtp_selftest();
#endif

	for (i = 0; i < 3; i++) {
		gte_rot[i].gen = psxCP2CtrlGen - 1u;
		for (int j = 0; j < 4; j++)
			gte_col[i][j].gen = psxCP2CtrlGen - 1u;
	}

	gte_hot.xmtrx_key = -1;
	gte_xmtrx_mx = -1;

	/* The offsets derive from the control file as well. */
	gte_hot.ofs_gen = psxCP2CtrlGen - 1u;
}

/* ------------------------------------------------------------------ */
/* The transform                                                       */
/* ------------------------------------------------------------------ */

#ifdef __sh__
/*
 * Fill XMTRX from the cached float matrix.  `fschg` switches `fmov` to the
 * pair form (the SH-4 writes a pair as two words in address order, so the
 * array reads into xf0..xf15 as laid out) and is toggled back before anything
 * else runs.  Separate from the transform because RTPT has three vertices
 * and one matrix.
 */
static void gte_mtx_load(const float *m, const float *t)
{
	__asm__ __volatile__(
		"fschg\n\t"
		"fmov.d	@%[m]+, xd0\n\t"
		"fmov.d	@%[m]+, xd2\n\t"
		"fmov.d	@%[m]+, xd4\n\t"
		"fmov.d	@%[m]+, xd6\n\t"
		"fmov.d	@%[m]+, xd8\n\t"
		"fmov.d	@%[m]+, xd10\n\t"
		"fmov.d	@%[t]+, xd12\n\t"
		"fmov.d	@%[t]+, xd14\n\t"
		"fschg"
		: [m] "+r" (m), [t] "+r" (t)
		:
		: "memory");
}

/* The translation column alone: xf12..xf15. */
static void gte_col_load(const float *t)
{
	__asm__ __volatile__(
		"fschg\n\t"
		"fmov.d	@%[t]+, xd12\n\t"
		"fmov.d	@%[t]+, xd14\n\t"
		"fschg"
		: [t] "+r" (t)
		:
		: "memory");
}

/*
 * One vertex: transformed, truncated, stored to MAC, clamped into IR.
 *
 * Three base registers because `mov.l` reaches 60 bytes and `mov.w` 30,
 * while MAC1 sits at 100 and IR1 at 36:
 *
 *   r4  the register file          IR1/2/3 at 36/40/44 (halfwords)
 *   r6  r4 + 64                    MAC1/2/3 at 36/40/44
 *   r2  r4 + 36                    walked forward across the three IR slots
 *
 * The flag word accumulates in r7 and is OR'd into `gte_flag` once.  Each
 * bound branches out to a stub that sets the flag and substitutes the bound.
 * `limB3` sets bit 22 and NOT the master bit.
 */
static void gte_xform_ir(psxCP2Regs *r, const s16 *v)
{
	register u32 flags __asm__("r7");

	__asm__ __volatile__(
		"mov.w	@%[v], r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr0\n\t"
		"mov.w	@(2,%[v]), r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr1\n\t"
		"mov.w	@(4,%[v]), r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr2\n\t"
		"fldi1	fr3\n\t"

		"ftrv	xmtrx, fv0\n\t"

		"mov	%[r], r6\n\t"
		"add	#64, r6\n\t"
		"mov	%[r], r2\n\t"
		"add	#36, r2\n\t"
		"mov	#0, r7\n\t"

		/* MAC1 / IR1 */
		"ftrc	fr0, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(36,r6)\n\t"
		"mov.l	1f, r3\n\t"
		"cmp/gt	r3, r1\n\t"
		"bt	10f\n\t"
		"mov.l	2f, r3\n\t"
		"cmp/ge	r3, r1\n\t"
		"bf	11f\n"
		"12:\t"
		"mov.w	r1, @r2\n\t"
		"add	#4, r2\n\t"

		/* MAC2 / IR2 */
		"ftrc	fr1, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(40,r6)\n\t"
		"mov.l	1f, r3\n\t"
		"cmp/gt	r3, r1\n\t"
		"bt	20f\n\t"
		"mov.l	2f, r3\n\t"
		"cmp/ge	r3, r1\n\t"
		"bf	21f\n"
		"22:\t"
		"mov.w	r1, @r2\n\t"
		"add	#4, r2\n\t"

		/* MAC3 / IR3 */
		"ftrc	fr2, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(44,r6)\n\t"
		"mov.l	1f, r3\n\t"
		"cmp/gt	r3, r1\n\t"
		"bt	30f\n\t"
		"mov.l	2f, r3\n\t"
		"cmp/ge	r3, r1\n\t"
		"bf	31f\n"
		"32:\t"
		"bra	9f\n\t"
		" mov.w	r1, @r2\n\t"

		".align 2\n"
		"1:	.long	32767\n"
		"2:	.long	-32768\n"

		/* The six saturation stubs.  Each substitutes the bound it
		 * failed, ORs its flag word and rejoins. */
		"10:	mov.l	6f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	12b\n\t"
		" mov	r3, r1\n"
		"11:	mov.l	6f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	12b\n\t"
		" mov	r3, r1\n"

		"20:	mov.l	7f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	22b\n\t"
		" mov	r3, r1\n"
		"21:	mov.l	7f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	22b\n\t"
		" mov	r3, r1\n"

		"30:	mov.l	8f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	32b\n\t"
		" mov	r3, r1\n"
		"31:	mov.l	8f, r0\n\t"
		"or	r0, r7\n\t"
		"bra	32b\n\t"
		" mov	r3, r1\n"

		/* The flag words live here: `mov.l` is PC-relative forward only and
		 * reaches 1020 bytes, which the stubs are past.  Nothing falls
		 * through into this. */
		".align 2\n"
		"6:	.long	0x81000000\n"	/* limB1: master | bit 24 */
		"7:	.long	0x80800000\n"	/* limB2: master | bit 23 */
		"8:	.long	0x00400000\n"	/* limB3: bit 22, no master */
		"9:"
		: "=r" (flags)
		: [r] "r" (r), [v] "r" (v)
		: "r0", "r1", "r2", "r3", "r6",
		  "fr0", "fr1", "fr2", "fr3", "fr4",
		  "fpul", "t", "memory");

	gte_flag |= flags;
}

/*
 * One vertex through the resident matrix, truncated, as three s32.  Only
 * fr0-fr3, r0, r1, FPUL and T are touched, all caller-saved: no prologue.
 */
static void gte_xform(const s16 *v, s32 *mac)
{
	__asm__ __volatile__(
		"mov.w	@%[v], r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr0\n\t"
		"mov.w	@(2,%[v]), r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr1\n\t"
		"mov.w	@(4,%[v]), r0\n\t"
		"lds	r0, fpul\n\t"
		"float	fpul, fr2\n\t"
		"fldi1	fr3\n\t"

		"ftrv	xmtrx, fv0\n\t"

		"ftrc	fr0, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(0,%[mac])\n\t"

		"ftrc	fr1, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(4,%[mac])\n\t"

		"ftrc	fr2, fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(8,%[mac])"
		:
		: [v] "r" (v), [mac] "r" (mac)
		: "r0", "r1", "fr0", "fr1", "fr2", "fr3",
		  "fpul", "t", "memory");
}

/* Three floats to three truncated 32-bit results, for the commands whose
 * products are three multiplies rather than a matrix. */
static void gte_floor3(float a, float b, float c, s32 *mac)
{
	__asm__ __volatile__(
		"ftrc	%[a], fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(0,%[mac])\n\t"

		"ftrc	%[b], fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(4,%[mac])\n\t"

		"ftrc	%[c], fpul\n\t"
		"sts	fpul, r1\n\t"
		"mov.l	r1, @(8,%[mac])"
		:
		: [a] "f" (a), [b] "f" (b), [c] "f" (c), [mac] "r" (mac)
		: "r1", "fpul", "memory");
}
#else
/*
 * The host stand-in for the self-test: same arithmetic, same order, same
 * precision.  The resident matrix is a pointer here; on the target it is XMTRX.
 */
#include <math.h>

static float gte_resident[16];

#ifdef GTE_ASM
/* The assembly runs under the SH-4 interpreter (gte_sh4.c), whose back bank
 * is its XMTRX; the C stand-ins here keep reading gte_resident. */
void gte_sh4_mtx_load(const float *m, const float *t);
void gte_sh4_col_load(const float *t);
#endif

static void gte_mtx_load(const float *m, const float *t)
{
	memcpy(gte_resident, m, 12 * sizeof(float));
	memcpy(gte_resident + 12, t, 4 * sizeof(float));
#ifdef GTE_ASM
	gte_sh4_mtx_load(m, t);
#endif
}

static void gte_col_load(const float *t)
{
	memcpy(gte_resident + 12, t, 4 * sizeof(float));
#ifdef GTE_ASM
	gte_sh4_col_load(t);
#endif
}

static void gte_xform(const s16 *v, s32 *mac)
{
	const float *m = gte_resident;
	int i;

	for (i = 0; i < 3; i++) {
#ifdef GTE_ASM
		/* The assembly's `ftrv` runs under the SH-4 interpreter
		 * (fgl/gte_sh4.c), which accumulates in double and rounds
		 * once; sum the same way so the self-test compares the
		 * assembly's plumbing and not two summation orders. */
		float acc = (float)((double)m[i] * v[0] + (double)m[4 + i] * v[1] +
				    (double)m[8 + i] * v[2] + (double)m[12 + i]);
#else
		float acc = m[i] * (float)v[0] + m[4 + i] * (float)v[1] +
			    m[8 + i] * (float)v[2] + m[12 + i];
#endif

		if (acc >= 2147483648.0f)
			mac[i] = (s32)0x7fffffff;
		else if (acc < -2147483648.0f)
			mac[i] = (s32)0x80000000;
		else
			mac[i] = (s32)acc;	/* truncation, as ftrc */
	}
}

/* The host equivalent of the fused routine; see the SH-4 version above. */
static void gte_xform_ir(psxCP2Regs *r, const s16 *v)
{
	s32 mac[3];

	gte_xform(v, mac);

	D_MAC1 = mac[0];
	D_MAC2 = mac[1];
	D_MAC3 = mac[2];

	D_IR1 = limB1(D_MAC1);
	D_IR2 = limB2(D_MAC2);
	D_IR3 = limB3(D_MAC3);
}

static void gte_floor3(float a, float b, float c, s32 *mac)
{
	const float v[3] = { a, b, c };
	int i;

	for (i = 0; i < 3; i++) {
		if (v[i] >= 2147483648.0f)
			mac[i] = (s32)0x7fffffff;
		else if (v[i] < -2147483648.0f)
			mac[i] = (s32)0x80000000;
		else
			mac[i] = (s32)v[i];	/* truncation, as ftrc */
	}
}
#endif

/* The `sf` field as a multiplier: `>> 12` is a premultiply by 1/4096. */
static const float gte_sf_scale[2] = { 1.0f, 1.0f / 4096.0f };

/*
 * The perspective divide: one `fdiv` in place of the reference's table walk
 * (which opens with a count of leading zeros SH-4 does not have).  The guard
 * is the reference's own and stays integer because it picks the path.
 * `n << 16` is exact as a float and the result is under 2^17, so the only
 * rounding is the divide's: more accurate than the hardware, hence different.
 */
static inline __attribute__((always_inline)) u32
gte_divide(u16 n, u16 d)
{
	u32 q;

#ifdef GTE_TABLE_DIV
	q = DIVIDE(n, d);
#else
	if ((u32)n < (u32)d * 2)
		q = (u32)((float)((u32)n << 16) / (float)d + 0.5f);
	else
		q = 0xffffffff;
#endif

	return q;
}

/* ------------------------------------------------------------------ */
/* RTPS                                                               */
/* ------------------------------------------------------------------ */

/*
 * One vertex: transformed, limited, and projected.  The caller says where the
 * results land (RTPS pushes a stack, RTPT writes the slots directly).
 * Returns the quotient, which the depth cue needs from the last vertex.
 */
/* Truncation, not floor: one fidelity level, chosen once for the whole
 * file (see the header). */
static s32 gte_floor_small(float v)
{
	return (s32)v;
}

/*
 * A projected coordinate, saturated the way the reference saturates it: the
 * 16.16 sum is bounded at +-2^31 and then shifted, so the saturated value
 * reaching the screen clamp is +-2^15, and it is still written.
 */
static s32 gte_project(float v)
{
	if (v >= 32768.0f) {
		gte_flag |= (1u << 31) | (1u << 16);
		return 32767;
	}
	if (v < -32768.0f) {
		gte_flag |= (1u << 31) | (1u << 15);
		return -32768;
	}
	return gte_floor_small(v);
}

/* OFX and OFY as screen units rather than 16.16, rebuilt per command. */
/* OFX and OFY as screen units rather than 16.16, rebuilt only when the
 * control file has been written (same counter as the matrix cache). */
#define gte_ofx	(gte_hot.ofx)
#define gte_ofy	(gte_hot.ofy)

static void gte_offsets_rebuild(const psxCP2Regs *r)
{
	gte_hot.ofs_gen = psxCP2CtrlGen;
	gte_hot.ofx = (float)C_OFX * (1.0f / 65536.0f);
	gte_hot.ofy = (float)C_OFY * (1.0f / 65536.0f);
}

static inline __attribute__((always_inline)) void
gte_offsets(const psxCP2Regs *r)
{
	if (gte_hot.ofs_gen != psxCP2CtrlGen)
		gte_offsets_rebuild(r);
}

static inline __attribute__((always_inline)) u32
gte_rtp_one_i(psxCP2Regs *r, int v, u16 *szp, s16 *sxp, s16 *syp)
{
	u32 quotient;
	float qs;

	gte_xform_ir(r, D_V(v));

	/* The `ftrc` saturation flags (bits 30-25) are NOT raised: flag-only,
	 * and no game reaches them. */
	*szp = limD(D_MAC3);

	quotient = limE(gte_divide(C_H, *szp));

	/*
	 * The projection, in float: the quotient is scaled to screen units
	 * first, so `(OFX + IR1 * quotient) >> 16` is one multiply and one add.
	 * `quotient` is under 2^17 and the scale a power of two, so `qs` is
	 * exact.  The saturation flag comes from the unclamped value, at 2^15
	 * (the reference's 2^31 before the shift).
	 */
	qs = (float)quotient * (1.0f / 65536.0f);

	*sxp = limG1(gte_project(gte_ofx + (float)D_IR1 * qs));
	*syp = limG2(gte_project(gte_ofy + (float)D_IR2 * qs));

	return quotient;
}

/*
 * Publish the accumulated flags.  The master bit is NOT recomputed: the
 * reference's constants carry bit 31 individually, and limB3 and the three
 * A-maxima deliberately do not.
 */
static void gte_end(psxCP2Regs *r)
{
	C_FLAG = gte_flag;
}

/* The depth cue.  Both transforms end this way. */
/* RTPT's copy, out of line: three inlined copies would not fit the icache. */
static u32 gte_rtp_one(psxCP2Regs *r, int v, u16 *szp, s16 *sxp, s16 *syp)
{
	return gte_rtp_one_i(r, v, szp, sxp, syp);
}

static inline __attribute__((always_inline)) void
gte_rtp_end_i(psxCP2Regs *r, u32 quotient)
{
	s64 tmp = (s64)C_DQB + (s64)C_DQA * quotient;

	/* The 32-bit overflow flags (bits 16/15) are not raised; MAC0 still takes
	 * the truncating store the reference takes. */
	D_MAC0 = (s32)tmp;
	D_IR0 = limH((s32)(tmp >> 12));

	gte_end(r);
}

#ifdef GTE_ASM
/*
 * On the target both transforms are `src/gte_rtp.S`.  The C versions are the
 * reference the self-test compares against, and the host build.
 */
void gte_rtpt_asm(psxCP2Regs *r, struct gte_hot *hot);
void gte_rtps_asm(psxCP2Regs *r, struct gte_hot *hot);

__attribute__((noinline)) void gte_fpu_rtps(psxCP2Regs *r)
{
	gte_mtx_use(r, GTE_MTX_ROT, 0);
	gte_offsets(r);
	gte_rtps_asm(r, &gte_hot);
}

__attribute__((noinline)) void gte_fpu_rtpt(psxCP2Regs *r)
{
	gte_mtx_use(r, GTE_MTX_ROT, 0);
	gte_offsets(r);
	gte_rtpt_asm(r, &gte_hot);
}

/*
 * The entry points the generated code calls (src/gte_rtp.S) do the cache
 * check themselves and come here only when it fails.
 */
#ifndef GTE_FAST
#define GTE_FAST 31
#endif
void gte_rtps_fast(psxCP2Regs *r);
void gte_rtpt_fast(psxCP2Regs *r);
void gte_mvmva_fast(psxCP2Regs *r, u32 op);
void gte_dpcs_fast(psxCP2Regs *r, u32 op);
void gte_intpl_fast(psxCP2Regs *r, u32 op);

void gte_rtp_slow(psxCP2Regs *r)
{
	gte_mtx_use_slow(r, GTE_MTX_ROT, 0, 0);
	gte_offsets(r);
}

#define gte_fpu_rtps gte_fpu_rtps_c
#define gte_fpu_rtpt gte_fpu_rtpt_c
#endif

__attribute__((noinline)) void gte_fpu_rtps(psxCP2Regs *r)
{
	u32 quotient;

	gte_flag = 0;

	gte_mtx_use(r, GTE_MTX_ROT, 0);

	gte_offsets(r);

	/* Both stacks shift before the new vertex lands, as the reference does. */
	D_SZ0 = D_SZ1;
	D_SZ1 = D_SZ2;
	D_SZ2 = D_SZ3;
	D_SXY0 = D_SXY1;
	D_SXY1 = D_SXY2;

	quotient = gte_rtp_one_i(r, 0, &r->CP2D.p[19].w.l,
				 &r->CP2D.p[14].sw.l, &r->CP2D.p[14].sw.h);

	gte_rtp_end_i(r, quotient);
}

/*
 * RTPT: no stack.  SZ0 takes the old SZ3 once, before anything is written,
 * and each vertex writes its own slot.  MAC, IR and the quotient are left
 * holding the third vertex's values for the depth cue.
 */
__attribute__((noinline)) void gte_fpu_rtpt(psxCP2Regs *r)
{
	u32 quotient = 0;
	int v;

	gte_flag = 0;

	gte_mtx_use(r, GTE_MTX_ROT, 0);
	gte_offsets(r);

	D_SZ0 = D_SZ3;

	for (v = 0; v < 3; v++)
		quotient = gte_rtp_one(r, v, &r->CP2D.p[v + 17].w.l,
				       &r->CP2D.p[v + 12].sw.l,
				       &r->CP2D.p[v + 12].sw.h);

	gte_rtp_end_i(r, quotient);
}

#ifdef GTE_ASM
#undef gte_fpu_rtps
#undef gte_fpu_rtpt

#ifdef GTE_RTP_SELFTEST
/* The assembly against the C it replaced, on random register files.  Runs
 * once at the first reset with -DGTE_RTP_SELFTEST; every register, FLAG and
 * the flag word must match bit for bit. */
static u32 gte_st_seed = 0x2545f491;

static void gte_mvmva(psxCP2Regs *r, u32 op);
static void gte_mvmva_c(psxCP2Regs *r, u32 op);
static void gte_dpcs(psxCP2Regs *r, u32 op);
static void gte_intpl(psxCP2Regs *r, u32 op);

static u32 gte_st_rnd(void)
{
	gte_st_seed = gte_st_seed * 1664525u + 1013904223u;
	return gte_st_seed >> 3;
}

/* Mostly in-game magnitudes, sometimes wild, so every limit fires. */
static s32 gte_st_val(int bits_common, int bits_wild)
{
	u32 r = gte_st_rnd();
	int bits = (r & 7) ? bits_common : bits_wild;
	s32 v = (s32)(gte_st_rnd() & ((1u << bits) - 1));

	return (r & 8) ? -v : v;
}

static void gte_st_fill(psxCP2Regs *r)
{
	int i;

	memset(r, 0, sizeof(*r));

	for (i = 0; i < 6; i++)
		r->CP2D.r[i] = 0;
	for (i = 0; i < 3; i++) {
		r->CP2D.p[i * 2].sw.l = gte_st_val(11, 15);
		r->CP2D.p[i * 2].sw.h = gte_st_val(11, 15);
		r->CP2D.p[i * 2 + 1].sw.l = gte_st_val(11, 15);
	}
	for (i = 12; i < 20; i++)
		r->CP2D.r[i] = gte_st_rnd();

	for (i = 0; i < 5; i++) {
		r->CP2C.p[i].sw.l = gte_st_val(12, 15);
		r->CP2C.p[i].sw.h = gte_st_val(12, 15);
	}
	for (i = 5; i < 8; i++)
		((s32 *)r->CP2C.r)[i] = gte_st_val(16, 31);
	for (i = 8; i < 13; i++) {
		r->CP2C.p[i].sw.l = gte_st_val(12, 15);
		r->CP2C.p[i].sw.h = gte_st_val(12, 15);
	}
	for (i = 16; i < 21; i++) {
		r->CP2C.p[i].sw.l = gte_st_val(12, 15);
		r->CP2C.p[i].sw.h = gte_st_val(12, 15);
	}
	for (i = 13; i < 16; i++)
		((s32 *)r->CP2C.r)[i] = gte_st_val(16, 31);
	for (i = 21; i < 24; i++)
		((s32 *)r->CP2C.r)[i] = gte_st_val(16, 31);
	for (i = 9; i < 12; i++)
		r->CP2D.p[i].sw.l = gte_st_val(11, 15);
	r->CP2D.p[8].sw.l = (gte_st_rnd() & 7) ? (gte_st_rnd() & 0x1fff)
					     : gte_st_val(15, 15);
	r->CP2D.r[6] = gte_st_rnd();
	for (i = 20; i < 23; i++)
		r->CP2D.r[i] = gte_st_rnd();
	((s32 *)r->CP2C.r)[24] = gte_st_val(25, 31);
	((s32 *)r->CP2C.r)[25] = gte_st_val(25, 31);
	r->CP2C.p[26].w.l = (gte_st_rnd() & 7) ? (gte_st_rnd() & 0x7ff)
					      : (gte_st_rnd() & 0xffff);
	r->CP2C.p[27].sw.l = gte_st_val(12, 15);
	r->CP2C.p[29].sw.l = gte_st_val(12, 15);
	r->CP2C.p[30].sw.l = gte_st_val(12, 15);
	((s32 *)r->CP2C.r)[28] = gte_st_val(24, 31);
}

/* 0: RTPS, 1: RTPT, 2: MVMVA with a random op word (sf=1, lm=0, mx<3) */
static int gte_st_run(int which, int n)
{
	static psxCP2Regs a, b;
	static const char *names[] = { "RTPS", "RTPT", "MVMVA", "DPCS", "INTPL",
				       "RTPS leaf", "RTPT leaf" };
	int i, bad = 0;
	u32 op = 0;

	for (i = 0; i < n; i++) {
		gte_st_fill(&a);
		psxCP2CtrlGen++;
		b = a;

		if (which == 2) {
			u32 x = gte_st_rnd();
			op = (1u << 19) | (((x % 3) & 3) << 17) |
			     (((x >> 4) & 3) << 15) | (((x >> 8) & 3) << 13);
		} else if (which >= 3) {
			u32 x = gte_st_rnd();
			op = ((x & 1) << 19) | (((x >> 1) & 1) << 10);
		}

		if (which == 3)
			gte_dpcs(&a, op);
		else if (which == 4)
			gte_intpl(&a, op);
		else if (which == 2)
			gte_mvmva_c(&a, op);
		else if (which == 1 || which == 6)
			gte_fpu_rtpt_c(&a);
		else
			gte_fpu_rtps_c(&a);
		u32 fa = gte_hot.flag;

		if (which == 3)
			gte_dpcs_fast(&b, op);
		else if (which == 4)
			gte_intpl_fast(&b, op);
		else if (which == 2)
			gte_mvmva_fast(&b, op);
		else if (which == 5)
			gte_rtps_leaf_call(&b);
		else if (which == 6)
			gte_rtpt_leaf_call(&b);
		else if (which)
			gte_rtpt_fast(&b);
		else
			gte_rtps_fast(&b);
		/* A leaf writes FLAG into the file and not gte_hot.flag; the
		 * file compare below covers it. */
		u32 fb = which >= 5 ? fa : gte_hot.flag;

		if (memcmp(&a, &b, sizeof(a)) || fa != fb) {
			if (bad < 4) {
				int k;
				int first = -1;
				for (k = 0; k < (int)sizeof(a); k++)
					if (((u8 *)&a)[k] != ((u8 *)&b)[k]) { first = k; break; }
				printf("GTE %s case %d op %08x: flag %08x vs %08x firstbyte %d\n",
				       names[which], i, op, fa, fb, first);
				for (k = 0; k < 64; k++) {
					u32 x = ((u32 *)&a)[k], y = ((u32 *)&b)[k];
					if (x != y)
						printf("  r%d: %08x vs %08x\n", k, x, y);
				}
			}
			bad++;
		}
	}
	return bad;
}

static void gte_nclip_cmd(psxCP2Regs *r);

static void gte_rtp_selftest(void)
{
	static int done;
	int bs, bt, bm, bd, bi;

	if (done)
		return;
	done = 1;

	{
		/* The matrix conversion against the C it replaced. */
		int k, j, bc = 0;
		for (k = 0; k < 2000; k++) {
			union { u32 w[5]; PAIR p[5]; } src;
			float ma[12], mb[12];
			for (j = 0; j < 5; j++)
				src.w[j] = gte_st_rnd() ^ (gte_st_rnd() << 13);
			ma[0]  = src.p[0].sw.l * (1.0f / 4096.0f);
			ma[4]  = src.p[0].sw.h * (1.0f / 4096.0f);
			ma[8]  = src.p[1].sw.l * (1.0f / 4096.0f);
			ma[1]  = src.p[1].sw.h * (1.0f / 4096.0f);
			ma[5]  = src.p[2].sw.l * (1.0f / 4096.0f);
			ma[9]  = src.p[2].sw.h * (1.0f / 4096.0f);
			ma[2]  = src.p[3].sw.l * (1.0f / 4096.0f);
			ma[6]  = src.p[3].sw.h * (1.0f / 4096.0f);
			ma[10] = src.p[4].sw.l * (1.0f / 4096.0f);
			ma[3] = ma[7] = ma[11] = 0.0f;
			gte_rot_convert(src.w, mb);
			if (memcmp(ma, mb, sizeof(ma)))
				bc++;
		}
		printf("GTE selftest: rot_convert %d/2000 bad\n", bc);
	}

	{
		/* The NCLIP leaf against the C, MAC0 and FLAG.  Every eighth
		 * case is built from the extremes so both overflow flags are
		 * exercised, not just the in-range path. */
		static psxCP2Regs a, b;
		int k, bn = 0;
		for (k = 0; k < 4000; k++) {
			gte_st_fill(&a);
			if ((k & 7) == 0) {
				u32 x = gte_st_rnd();
				a.CP2D.r[12] = (x & 1) ? 0x80008000u : 0x7fff7fffu;
				a.CP2D.r[13] = (x & 2) ? 0x7fff8000u : 0x80007fffu;
				a.CP2D.r[14] = (x & 4) ? 0x80007fffu : x;
			}
			b = a;
			gte_nclip_cmd(&a);
			gte_nclip_leaf_call(&b);
			if (a.CP2D.r[24] != b.CP2D.r[24] ||
			    a.CP2C.r[31] != b.CP2C.r[31]) {
				if (bn < 3)
					printf("GTE selftest: NCLIP leaf %08x %08x %08x: C mac0 %08x flag %08x, leaf %08x %08x\n",
					       a.CP2D.r[12], a.CP2D.r[13], a.CP2D.r[14],
					       a.CP2D.r[24], a.CP2C.r[31],
					       b.CP2D.r[24], b.CP2C.r[31]);
				bn++;
			}
		}
		printf("GTE selftest: NCLIP leaf %d/4000 bad\n", bn);
	}

	bs = gte_st_run(0, 4000);
	bt = gte_st_run(1, 4000);
	bm = gte_st_run(2, 4000);
	bd = gte_st_run(3, 4000);
	bi = gte_st_run(4, 4000);
	printf("GTE selftest: RTPS %d/4000 bad, RTPT %d/4000 bad, MVMVA %d/4000 bad, DPCS %d/4000 bad, INTPL %d/4000 bad\n",
	       bs, bt, bm, bd, bi);
	bs = gte_st_run(5, 4000);
	bt = gte_st_run(6, 4000);
	printf("GTE selftest: RTPS leaf %d/4000 bad, RTPT leaf %d/4000 bad\n", bs, bt);

	{
		/* Every integer leaf against pcsx's gte.c, the ground truth,
		 * on the whole file: random sf/lm (MVMVA: every selector),
		 * random registers, the extremes every eighth case. */
		static const u8 fns[] = { 0x12, 0x10, 0x11, 0x28, 0x0c, 0x3d,
			0x3e, 0x29, 0x2a, 0x2d, 0x2e, 0x1e, 0x20, 0x1b, 0x3f,
			0x13, 0x16, 0x1c, 0x14 };
		static psxCP2Regs a, b;
		int f, total = 0;
		char line[256];
		int at = 0;

		for (f = 0; f < (int)sizeof fns; f++) {
			int leaf = gte_fpu_leaf_cmd(fns[f]) >> 1;
			int k, bad = 0;
			for (k = 0; k < 4000; k++) {
				u32 x = gte_st_rnd();
				u32 op = 0x4a000000u | fns[f] | ((x & 1) << 19) |
					 ((x & 2) << 9);
				if (fns[f] == 0x12)
					op |= ((x >> 2) & 0x7f) << 13;
				gte_st_fill(&a);
				if ((k & 7) == 0) {
					int j;
					for (j = 0; j < 3; j++)
						a.CP2D.p[9 + j].sw.l = (x >> j) & 8 ? 0x7fff : -0x8000;
					a.CP2D.p[8].sw.l = (x & 0x40) ? 0x7fff : -0x8000;
					for (j = 21; j < 24; j++)
						((s32 *)a.CP2C.r)[j] = (x >> j) & 1 ? 0x7fffffff : (s32)0x80000000;
					for (j = 5; j < 8; j++)
						((s32 *)a.CP2C.r)[j] = (x >> j) & 1 ? 0x7fffffff : (s32)0x80000000;
					for (j = 13; j < 16; j++)
						((s32 *)a.CP2C.r)[j] = (x >> j) & 1 ? 0x7fffffff : (s32)0x80000000;
				}
				b = a;
				gteDispatch(&a, op);
				gte_leaf_call(leaf, &b, op);
				if (memcmp(&a, &b, sizeof(a))) {
					if (bad < 8) {
						int w;
						printf("GTE selftest: %s leaf op %08x:", gte_fpu_leaf_name(leaf), op);
						for (w = 0; w < 64; w++)
							if (((u32 *)&a)[w] != ((u32 *)&b)[w])
								printf(" r%d %08x/%08x", w, ((u32 *)&a)[w], ((u32 *)&b)[w]);
						printf("\n");
					}
					bad++;
				}
			}
			total += bad;
			at += snprintf(line + at, sizeof line - at, "%s %d ",
				       gte_fpu_leaf_name(leaf), bad);
			if (at > 200) {
				printf("GTE selftest: leaves bad/4000: %s\n", line);
				at = 0;
			}
		}
		printf("GTE selftest: leaves bad/4000: %s\n", line);
		printf("GTE selftest: integer leaves %d bad in total\n", total);
	}
}
#endif
#endif

/* ------------------------------------------------------------------ */
/* The shared stages                                                   */
/* ------------------------------------------------------------------ */

/* The vector a command multiplies.  Selector 3 names IR1..IR3, one halfword
 * every four bytes, so that case is copied into a contiguous triple. */
static const s16 *gte_vector(const psxCP2Regs *r, int v, s16 *scratch)
{
	if (v < 3)
		return D_V(v);

	scratch[0] = D_IR1;
	scratch[1] = D_IR2;
	scratch[2] = D_IR3;

	return scratch;
}

static void gte_ir_from_mac(psxCP2Regs *r, int lm)
{
	D_IR1 = gte_limB1(D_MAC1, lm);
	D_IR2 = gte_limB2(D_MAC2, lm);
	D_IR3 = gte_limB3(D_MAC3, lm);
}

/* The light stage: MAC = (LLM * V) >> 12, IR clamped at zero.  No
 * translation column and no overflow flags (the reference raises none). */
static void gte_light_stage(psxCP2Regs *r, const s16 *v)
{
	s32 mac[3];

	gte_mtx_use(r, GTE_MTX_LIGHT, GTE_CV_NONE);
	gte_xform(v, mac);

	D_MAC1 = mac[0];
	D_MAC2 = mac[1];
	D_MAC3 = mac[2];

	gte_ir_from_mac(r, 1);
}

/*
 * The colour stage: MAC = (BK << 12 + LCM * IR) >> 12, the same `ftrv` with
 * BK as the translation column.  It leaves IR alone: NCT runs three of these
 * and clamps only once, at the end.
 */
static void gte_colour_stage(psxCP2Regs *r)
{
	s16 ir[3];
	s32 mac[3];

	ir[0] = D_IR1;
	ir[1] = D_IR2;
	ir[2] = D_IR3;

	gte_mtx_use(r, GTE_MTX_COLOUR, 1);
	gte_xform(ir, mac);

	D_MAC1 = AF1(mac[0]);
	D_MAC2 = AF2(mac[1]);
	D_MAC3 = AF3(mac[2]);
}

/* MAC modulated by the source colour, which is where the lighting commands
 * differ from each other: NCCS clamps the result, CC does not. */
static void gte_modulate(psxCP2Regs *r)
{
	D_MAC1 = ((s32)D_R * D_IR1) >> 8;
	D_MAC2 = ((s32)D_G * D_IR2) >> 8;
	D_MAC3 = ((s32)D_B * D_IR3) >> 8;
}

/* The far-colour interpolation the depth-cued commands run over the modulated
 * colour.  `limB1` on all three components is the reference's, not a slip. */
static void gte_depth_cue(psxCP2Regs *r)
{
	D_MAC1 = ((((s32)D_R << 4) * D_IR1) +
		  (D_IR0 * limB1(A1((s64)C_RFC - (((s32)D_R * D_IR1) >> 8))))) >> 12;
	D_MAC2 = ((((s32)D_G << 4) * D_IR2) +
		  (D_IR0 * limB2(A2((s64)C_GFC - (((s32)D_G * D_IR2) >> 8))))) >> 12;
	D_MAC3 = ((((s32)D_B << 4) * D_IR3) +
		  (D_IR0 * limB3(A3((s64)C_BFC - (((s32)D_B * D_IR3) >> 8))))) >> 12;
}

/* MAC into the colour FIFO, which every colour command ends with. */
static void gte_mac_to_rgb(psxCP2Regs *r)
{
	D_RGB0 = D_RGB1;
	D_RGB1 = D_RGB2;
	D_CODE2 = D_CODE;
	D_R2 = limC1(D_MAC1 >> 4);
	D_G2 = limC2(D_MAC2 >> 4);
	D_B2 = limC3(D_MAC3 >> 4);
}

/* ------------------------------------------------------------------ */
/* MVMVA                                                               */
/* ------------------------------------------------------------------ */

/*
 * The general matrix product.  Selector index 3 is not a matrix (zeros), and
 * `sf` clear would put the translation in at 2^12 times its value, past what
 * a float mantissa holds exactly; both go down the integer path.
 */
#ifdef GTE_ASM
void gte_mvmva_asm(psxCP2Regs *r, const s16 *v, int stride,
		   struct gte_hot *hot);
static void gte_mvmva_c(psxCP2Regs *r, u32 op);

/*
 * The loop shape (`sf` set, a real matrix, `lm` clear) is `src/gte_rtp.S`.
 * IR as the vector is three halfwords four bytes apart, hence the stride.
 * Everything else is the reference's integer arithmetic below.
 */
GTE_CMD void gte_mvmva(psxCP2Regs *r, u32 op)
{
	int mx = GTE_MX(op);
	int cv = GTE_CV(op);
	int vs = GTE_V(op);

	if (GTE_SF(op) && mx < 3 && !GTE_LM(op)) {
		gte_mtx_use(r, mx, cv);

		if (vs < 3)
			gte_mvmva_asm(r, D_V(vs), 2, &gte_hot);
		else
			gte_mvmva_asm(r, &D_IR1, 4, &gte_hot);
		return;
	}

	gte_mvmva_c(r, op);
}

#define gte_mvmva gte_mvmva_c
#endif

GTE_CMD void gte_mvmva(psxCP2Regs *r, u32 op)
{
	int sf = GTE_SF(op);
	int mx = GTE_MX(op);
	int cv = GTE_CV(op);
	int lm = GTE_LM(op);
	s16 scratch[3];
	const s16 *v = gte_vector(r, GTE_V(op), scratch);

	gte_flag = 0;

	if (sf && mx < 3) {
		s32 mac[3];

		gte_mtx_use(r, mx, cv);
		gte_xform(v, mac);

		D_MAC1 = AF1(mac[0]);
		D_MAC2 = AF2(mac[1]);
		D_MAC3 = AF3(mac[2]);
	} else {
		const PAIR *p = mx < 3 ? &r->CP2C.p[mx * 8] : NULL;
		const s32 *t = cv < 3 ? &((s32 *)r->CP2C.r)[cv * 8 + 5] : NULL;
		int shift = 12 * sf;
		s32 vx = v[0], vy = v[1], vz = v[2];
		s32 m11 = p ? p[0].sw.l : 0, m12 = p ? p[0].sw.h : 0;
		s32 m13 = p ? p[1].sw.l : 0, m21 = p ? p[1].sw.h : 0;
		s32 m22 = p ? p[2].sw.l : 0, m23 = p ? p[2].sw.h : 0;
		s32 m31 = p ? p[3].sw.l : 0, m32 = p ? p[3].sw.h : 0;
		s32 m33 = p ? p[4].sw.l : 0;
		s64 cv1 = t ? t[0] : 0, cv2 = t ? t[1] : 0, cv3 = t ? t[2] : 0;

		D_MAC1 = A1(((cv1 << 12) + (m11 * vx) + (m12 * vy) + (m13 * vz)) >> shift);
		D_MAC2 = A2(((cv2 << 12) + (m21 * vx) + (m22 * vy) + (m23 * vz)) >> shift);
		D_MAC3 = A3(((cv3 << 12) + (m31 * vx) + (m32 * vy) + (m33 * vz)) >> shift);
	}

	gte_ir_from_mac(r, lm);

	gte_end(r);
}

#ifdef GTE_ASM
#undef gte_mvmva

void gte_mvmva_sf0_asm(psxCP2Regs *r, const s16 *v, const s16 *m,
		       const s32 *t);

/* cv == 3 selects no translation column; the asm reads three words either
 * way, so point it at zeros rather than branch inside the loop. */
static const s32 gte_no_translation[3];

void gte_mvmva_slow(psxCP2Regs *r, u32 op)
{
	int mx = GTE_MX(op);
	int cv = GTE_CV(op);

	/* sf == 0 is the shape the FPU path cannot take. */
	if (!GTE_SF(op) && mx < 3 && !GTE_LM(op)) {
		s16 scratch[3];
		const s16 *v = gte_vector(r, GTE_V(op), scratch);

		gte_flag = 0;
		gte_mvmva_sf0_asm(r, v, (const s16 *)&r->CP2C.p[mx * 8],
				  cv < 3 ? &((s32 *)r->CP2C.r)[cv * 8 + 5]
					 : gte_no_translation);
		gte_end(r);
		return;
	}

	gte_mvmva_c(r, op);
}

#ifdef GTE_SHADOW
/*
 * Every call runs the C reference on a copy of the file first, then the
 * assembly on the real one, and prints the first few disagreements.
 */
static int gte_shadow_bad;
static void gte_dpcs(psxCP2Regs *r, u32 op);
static void gte_intpl(psxCP2Regs *r, u32 op);

static void gte_shadow_cmp(const char *name, u32 op, const psxCP2Regs *a,
			   const psxCP2Regs *b, u32 fa, u32 fb)
{
	int k;

	if (!memcmp(a, b, sizeof(*a)) && fa == fb)
		return;
	if (++gte_shadow_bad > 8)
		return;
	printf("GTE shadow %s op %08x: flag %08x vs %08x\n", name, op, fa, fb);
	for (k = 0; k < 64; k++)
		if (((u32 *)a)[k] != ((u32 *)b)[k])
			printf("  r%d: %08x vs %08x\n", k, ((u32 *)a)[k], ((u32 *)b)[k]);
}

#define GTE_SHADOW_FN(name, cfn, afn, ...)				\
static void gte_shadow_##name(psxCP2Regs *r, u32 op)			\
{									\
	static psxCP2Regs copy;						\
	u32 fa, fb;							\
	copy = *r;							\
	cfn(&copy, ##__VA_ARGS__);					\
	fa = gte_hot.flag;						\
	afn(r, ##__VA_ARGS__);						\
	fb = gte_hot.flag;						\
	gte_shadow_cmp(#name, op, &copy, r, fa, fb);			\
}

static void gte_rtps_c_op(psxCP2Regs *r, u32 op) { (void)op; gte_fpu_rtps_c(r); }
static void gte_rtpt_c_op(psxCP2Regs *r, u32 op) { (void)op; gte_fpu_rtpt_c(r); }
static void gte_rtps_a_op(psxCP2Regs *r, u32 op) { (void)op; gte_rtps_fast(r); }
static void gte_rtpt_a_op(psxCP2Regs *r, u32 op) { (void)op; gte_rtpt_fast(r); }
GTE_SHADOW_FN(rtps, gte_rtps_c_op, gte_rtps_a_op, op)
GTE_SHADOW_FN(rtpt, gte_rtpt_c_op, gte_rtpt_a_op, op)
GTE_SHADOW_FN(mvmva, gte_mvmva_c, gte_mvmva_fast, op)
GTE_SHADOW_FN(dpcs, gte_dpcs, gte_dpcs_fast, op)
GTE_SHADOW_FN(intpl, gte_intpl, gte_intpl_fast, op)
#endif
#endif

/* ------------------------------------------------------------------ */
/* The lighting family                                                 */
/* ------------------------------------------------------------------ */

GTE_CMD void gte_ncs(psxCP2Regs *r)
{
	gte_flag = 0;

	gte_light_stage(r, D_V(0));
	gte_colour_stage(r);
	gte_ir_from_mac(r, 1);
	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void gte_nct(psxCP2Regs *r)
{
	int v;

	gte_flag = 0;

	/* No clamp inside the loop: the reference takes IR from the last
	 * vertex's MAC once, after all three have been written. */
	for (v = 0; v < 3; v++) {
		gte_light_stage(r, D_V(v));
		gte_colour_stage(r);
		gte_mac_to_rgb(r);
	}

	gte_ir_from_mac(r, 1);

	gte_end(r);
}

GTE_CMD void gte_nccs(psxCP2Regs *r)
{
	gte_flag = 0;

	gte_light_stage(r, D_V(0));
	gte_colour_stage(r);
	gte_ir_from_mac(r, 1);
	gte_modulate(r);

	D_IR1 = D_MAC1;
	D_IR2 = D_MAC2;
	D_IR3 = D_MAC3;

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void gte_ncct(psxCP2Regs *r)
{
	int v;

	gte_flag = 0;

	for (v = 0; v < 3; v++) {
		gte_light_stage(r, D_V(v));
		gte_colour_stage(r);
		gte_ir_from_mac(r, 1);
		gte_modulate(r);
		gte_mac_to_rgb(r);
	}

	D_IR1 = D_MAC1;
	D_IR2 = D_MAC2;
	D_IR3 = D_MAC3;

	gte_end(r);
}

GTE_CMD void gte_ncds(psxCP2Regs *r)
{
	gte_flag = 0;

	gte_light_stage(r, D_V(0));
	gte_colour_stage(r);
	gte_ir_from_mac(r, 1);
	gte_depth_cue(r);

	gte_ir_from_mac(r, 1);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void gte_ncdt(psxCP2Regs *r)
{
	int v;

	gte_flag = 0;

	for (v = 0; v < 3; v++) {
		gte_light_stage(r, D_V(v));
		gte_colour_stage(r);
		gte_ir_from_mac(r, 1);
		gte_depth_cue(r);
		gte_mac_to_rgb(r);
	}

	gte_ir_from_mac(r, 1);

	gte_end(r);
}

GTE_CMD void gte_cc(psxCP2Regs *r)
{
	gte_flag = 0;

	gte_colour_stage(r);
	gte_ir_from_mac(r, 1);
	gte_modulate(r);

	gte_ir_from_mac(r, 1);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void gte_cdp(psxCP2Regs *r)
{
	gte_flag = 0;

	gte_colour_stage(r);
	gte_ir_from_mac(r, 1);
	gte_depth_cue(r);

	gte_ir_from_mac(r, 1);

	gte_mac_to_rgb(r);

	gte_end(r);
}

/* ------------------------------------------------------------------ */
/* The integer commands                                                */
/* ------------------------------------------------------------------ */

/* Everything below stays on the integer unit, bit-exact with the reference. */

static inline __attribute__((always_inline)) void
gte_nclip(psxCP2Regs *r)
{
	gte_flag = 0;

	D_MAC0 = F((s64)D_SX0 * (D_SY1 - D_SY2) +
		   (s64)D_SX1 * (D_SY2 - D_SY0) +
		   (s64)D_SX2 * (D_SY0 - D_SY1));

	gte_end(r);
}

/* A standalone body for the compile-time dispatch table: the hot copy above
 * is always_inline into the interpreter's dispatch frame and has no address
 * of its own. */
GTE_CMD void gte_nclip_cmd(psxCP2Regs *r)
{
	gte_nclip(r);
}

GTE_CMD void
gte_avsz3(psxCP2Regs *r)
{
	gte_flag = 0;

	D_MAC0 = F((s64)C_ZSF3 * (D_SZ1 + D_SZ2 + D_SZ3));
	D_OTZ = limD(D_MAC0 >> 12);

	gte_end(r);
}

GTE_CMD void
gte_avsz4(psxCP2Regs *r)
{
	gte_flag = 0;

	D_MAC0 = F((s64)C_ZSF4 * (D_SZ0 + D_SZ1 + D_SZ2 + D_SZ3));
	D_OTZ = limD(D_MAC0 >> 12);

	gte_end(r);
}

/*
 * SQR, OP and GPF: three products and an optional `>> 12`.  `sf` decides the
 * unit: at `sf = 1` the shift is the float premultiply and results are under
 * 2^18; at `sf = 0` OP needs 25 exact bits and a single mantissa holds 24, so
 * it stays integer.  None raises an overflow flag.
 */
GTE_CMD void
gte_sqr(psxCP2Regs *r, u32 op)
{
	float s = gte_sf_scale[GTE_SF(op)];
	float i1 = D_IR1, i2 = D_IR2, i3 = D_IR3;
	s32 mac[3];

	gte_flag = 0;

	gte_floor3(i1 * i1 * s, i2 * i2 * s, i3 * i3 * s, mac);

	D_MAC1 = mac[0];
	D_MAC2 = mac[1];
	D_MAC3 = mac[2];

	gte_ir_from_mac(r, GTE_LM(op));

	gte_end(r);
}

/* The cross product of IR with the rotation matrix's diagonal. */
GTE_CMD void
gte_op(psxCP2Regs *r, u32 op)
{
	float s = gte_sf_scale[GTE_SF(op)];
	float d1 = C_R11, d2 = C_R22, d3 = C_R33;
	float i1 = D_IR1, i2 = D_IR2, i3 = D_IR3;
	s32 mac[3];

	gte_flag = 0;

	gte_floor3((d2 * i3 - d3 * i2) * s,
		   (d3 * i1 - d1 * i3) * s,
		   (d1 * i2 - d2 * i1) * s, mac);

	D_MAC1 = mac[0];
	D_MAC2 = mac[1];
	D_MAC3 = mac[2];

	gte_ir_from_mac(r, GTE_LM(op));

	gte_end(r);
}

GTE_CMD void
gte_dcpl(psxCP2Regs *r, u32 op)
{
	int lm = GTE_LM(op);
	s32 rir1 = ((s32)D_R * D_IR1) >> 8;
	s32 gir2 = ((s32)D_G * D_IR2) >> 8;
	s32 bir3 = ((s32)D_B * D_IR3) >> 8;

	gte_flag = 0;

	D_MAC1 = rir1 + ((D_IR0 * limB1(A1((s64)C_RFC - rir1))) >> 12);
	D_MAC2 = gir2 + ((D_IR0 * limB1(A2((s64)C_GFC - gir2))) >> 12);
	D_MAC3 = bir3 + ((D_IR0 * limB1(A3((s64)C_BFC - bir3))) >> 12);

	gte_ir_from_mac(r, lm);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void
gte_gpf(psxCP2Regs *r, u32 op)
{
	float s = gte_sf_scale[GTE_SF(op)];
	float i0 = D_IR0;
	float i1 = D_IR1, i2 = D_IR2, i3 = D_IR3;
	s32 mac[3];

	gte_flag = 0;

	gte_floor3(i0 * i1 * s, i0 * i2 * s, i0 * i3 * s, mac);

	D_MAC1 = mac[0];
	D_MAC2 = mac[1];
	D_MAC3 = mac[2];

	D_IR1 = limB1(D_MAC1);
	D_IR2 = limB2(D_MAC2);
	D_IR3 = limB3(D_MAC3);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void
gte_gpl(psxCP2Regs *r, u32 op)
{
	int shift = 12 * GTE_SF(op);

	gte_flag = 0;

	D_MAC1 = A1((((s64)D_MAC1 << shift) + (D_IR0 * D_IR1)) >> shift);
	D_MAC2 = A2((((s64)D_MAC2 << shift) + (D_IR0 * D_IR2)) >> shift);
	D_MAC3 = A3((((s64)D_MAC3 << shift) + (D_IR0 * D_IR3)) >> shift);

	D_IR1 = limB1(D_MAC1);
	D_IR2 = limB2(D_MAC2);
	D_IR3 = limB3(D_MAC3);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void
gte_dpcs(psxCP2Regs *r, u32 op)
{
	int shift = 12 * GTE_SF(op);

	gte_flag = 0;

	D_MAC1 = (((s32)D_R << 16) +
		  (D_IR0 * limB1(A1(((s64)C_RFC - ((s32)D_R << 4)) << (12 - shift))))) >> 12;
	D_MAC2 = (((s32)D_G << 16) +
		  (D_IR0 * limB2(A2(((s64)C_GFC - ((s32)D_G << 4)) << (12 - shift))))) >> 12;
	D_MAC3 = (((s32)D_B << 16) +
		  (D_IR0 * limB3(A3(((s64)C_BFC - ((s32)D_B << 4)) << (12 - shift))))) >> 12;

	D_IR1 = limB1(D_MAC1);
	D_IR2 = limB2(D_MAC2);
	D_IR3 = limB3(D_MAC3);

	gte_mac_to_rgb(r);

	gte_end(r);
}

GTE_CMD void gte_dpct(psxCP2Regs *r)
{
	int v;

	gte_flag = 0;

	/* Each pass reads RGB0 *after* the previous pass shifted the FIFO, so
	 * the three iterations see three different source colours. */
	for (v = 0; v < 3; v++) {
		D_MAC1 = (((s32)D_R0 << 16) +
			  (D_IR0 * limB1(A1((s64)C_RFC - ((s32)D_R0 << 4))))) >> 12;
		D_MAC2 = (((s32)D_G0 << 16) +
			  (D_IR0 * limB1(A2((s64)C_GFC - ((s32)D_G0 << 4))))) >> 12;
		D_MAC3 = (((s32)D_B0 << 16) +
			  (D_IR0 * limB1(A3((s64)C_BFC - ((s32)D_B0 << 4))))) >> 12;

		gte_mac_to_rgb(r);
	}

	D_IR1 = limB1(D_MAC1);
	D_IR2 = limB2(D_MAC2);
	D_IR3 = limB3(D_MAC3);

	gte_end(r);
}

GTE_CMD void
gte_intpl(psxCP2Regs *r, u32 op)
{
	int shift = 12 * GTE_SF(op);
	int lm = GTE_LM(op);

	gte_flag = 0;

	D_MAC1 = ((D_IR1 << 12) + (D_IR0 * limB1(A1((s64)C_RFC - D_IR1)))) >> shift;
	D_MAC2 = ((D_IR2 << 12) + (D_IR0 * limB2(A2((s64)C_GFC - D_IR2)))) >> shift;
	D_MAC3 = ((D_IR3 << 12) + (D_IR0 * limB3(A3((s64)C_BFC - D_IR3)))) >> shift;

	gte_ir_from_mac(r, lm);

	gte_mac_to_rgb(r);

	gte_end(r);
}

/* ------------------------------------------------------------------ */
/* Dispatch                                                            */
/* ------------------------------------------------------------------ */

/*
 * Every command the GTE has, so the core's gte.c never enters the icache.
 * A command the hardware does not implement does nothing (`gteNULL`).
 */
static inline void gte_dispatch(psxCP2Regs *r, u32 op)
{
	switch (op & 0x3f) {
#ifdef GTE_ASM
	case 0x01: gte_rtps_fast(r); break;
	case 0x10: gte_dpcs_fast(r, op); break;
	case 0x11: gte_intpl_fast(r, op); break;
	case 0x12: gte_mvmva_fast(r, op); break;
	case 0x30: gte_rtpt_fast(r); break;
#else
	case 0x01: gte_fpu_rtps(r); break;
	case 0x06: gte_nclip(r); break;
	case 0x0c: gte_op(r, op); break;
	case 0x10: gte_dpcs(r, op); break;
	case 0x11: gte_intpl(r, op); break;
	case 0x12: gte_mvmva(r, op); break;
#endif
	case 0x13: gte_ncds(r); break;
	case 0x14: gte_cdp(r); break;
	case 0x16: gte_ncdt(r); break;
	case 0x1b: gte_nccs(r); break;
	case 0x1c: gte_cc(r); break;
	case 0x1e: gte_ncs(r); break;
	case 0x20: gte_nct(r); break;
	case 0x28: gte_sqr(r, op); break;
	case 0x29: gte_dcpl(r, op); break;
	case 0x2a: gte_dpct(r); break;
	case 0x2d: gte_avsz3(r); break;
	case 0x2e: gte_avsz4(r); break;
#ifndef GTE_ASM
	case 0x30: gte_fpu_rtpt(r); break;
#endif
	case 0x3d: gte_gpf(r, op); break;
	case 0x3e: gte_gpl(r, op); break;
	case 0x3f: gte_ncct(r); break;
	default: break;
	}
}

/*
 * A GTE command.  `op` is the whole instruction word: the shift, the
 * saturation flag and MVMVA's selectors live in the upper bits.  The hot
 * commands are compares ahead of the 64-way switch and inline into this
 * frame.  GTE stalls are not modelled.
 */
void gte_fpu_cmd(psxCP2Regs *r, u32 op)
{
	u32 fn = op & 0x3f;

	/* Generated code calls here directly, skipping cop2_op (the plugin's
	 * entry), so any GTE instrumentation belongs here. */

	if (fn == 0x01)
		gte_fpu_rtps(r);
	else if (fn == 0x06)
		gte_nclip(r);
	else if (fn == 0x30)
		gte_fpu_rtpt(r);
	else
		gte_dispatch(r, op);


}

/*
 * Compile-time dispatch for the recompiler; NULL means emit nothing.  The
 * caller always passes the op word: an extra argument register is invisible
 * to a body that does not name it.
 */
int gte_fpu_leaf_cmd(u32 op)
{
#ifdef GTE_ASM
	/* | 1: the leaf reads sf/lm (and MVMVA its selectors) from r1. */
	switch (op & 0x3f) {
	case 0x01: return GTE_LEAF_RTPS << 1;
	case 0x06: return GTE_LEAF_NCLIP << 1;
	case 0x0c: return (GTE_LEAF_OP << 1) | 1;
	case 0x10: return (GTE_LEAF_DPCS << 1) | 1;
	case 0x11: return (GTE_LEAF_INTPL << 1) | 1;
	case 0x12: return (GTE_LEAF_MVMVA << 1) | 1;
	case 0x13: return (GTE_LEAF_NCDS << 1) | 1;
	case 0x14: return (GTE_LEAF_CDP << 1) | 1;
	case 0x16: return (GTE_LEAF_NCDT << 1) | 1;
	case 0x1b: return (GTE_LEAF_NCCS << 1) | 1;
	case 0x1c: return (GTE_LEAF_CC << 1) | 1;
	case 0x1e: return (GTE_LEAF_NCS << 1) | 1;
	case 0x20: return (GTE_LEAF_NCT << 1) | 1;
	case 0x28: return (GTE_LEAF_SQR << 1) | 1;
	case 0x29: return (GTE_LEAF_DCPL << 1) | 1;
	case 0x2a: return (GTE_LEAF_DPCT << 1) | 1;
	case 0x2d: return GTE_LEAF_AVSZ3 << 1;
	case 0x2e: return GTE_LEAF_AVSZ4 << 1;
	case 0x30: return GTE_LEAF_RTPT << 1;
	case 0x3d: return (GTE_LEAF_GPF << 1) | 1;
	case 0x3e: return (GTE_LEAF_GPL << 1) | 1;
	case 0x3f: return (GTE_LEAF_NCCT << 1) | 1;
	default: break;
	}
#endif
	(void)op;
	return 0;
}

const char *gte_fpu_leaf_name(int leaf)
{
	static const char *const names[GTE_LEAF_N] = {
		NULL, "nclip", "rtps", "rtpt", "mvmva", "dpcs", "intpl", "sqr",
		"op", "gpf", "gpl", "dcpl", "dpct", "avsz3", "avsz4", "ncs",
		"nct", "nccs", "ncct", "ncds", "ncdt", "cc", "cdp",
	};
	return (leaf > 0 && leaf < GTE_LEAF_N) ? names[leaf] : NULL;
}

void *gte_fpu_resolve(u32 op)
{
	switch (op & 0x3f) {
#ifdef GTE_SHADOW
	case 0x01: return (void *)gte_shadow_rtps;
	case 0x10: return (void *)gte_shadow_dpcs;
	case 0x11: return (void *)gte_shadow_intpl;
	case 0x12: return (void *)gte_shadow_mvmva;
	case 0x30: return (void *)gte_shadow_rtpt;
#elif defined(GTE_ASM)
	case 0x01: return (void *)(GTE_FAST & 1 ? gte_rtps_fast : gte_fpu_rtps);
	case 0x10: return (void *)(GTE_FAST & 2 ? gte_dpcs_fast : gte_dpcs);
	case 0x11: return (void *)(GTE_FAST & 4 ? gte_intpl_fast : gte_intpl);
	case 0x12: return (void *)(GTE_FAST & 8 ? gte_mvmva_fast : gte_mvmva);
	case 0x30: return (void *)(GTE_FAST & 16 ? gte_rtpt_fast : gte_fpu_rtpt);
#else
	case 0x01: return (void *)gte_fpu_rtps;
#endif
	case 0x06: return (void *)gte_nclip_cmd;
	case 0x0c: return (void *)gte_op;
#ifndef GTE_ASM
	case 0x10: return (void *)gte_dpcs;
	case 0x11: return (void *)gte_intpl;
	case 0x12: return (void *)gte_mvmva;
#endif
	case 0x13: return (void *)gte_ncds;
	case 0x14: return (void *)gte_cdp;
	case 0x16: return (void *)gte_ncdt;
	case 0x1b: return (void *)gte_nccs;
	case 0x1c: return (void *)gte_cc;
	case 0x1e: return (void *)gte_ncs;
	case 0x20: return (void *)gte_nct;
	case 0x28: return (void *)gte_sqr;
	case 0x29: return (void *)gte_dcpl;
	case 0x2a: return (void *)gte_dpct;
	case 0x2d: return (void *)gte_avsz3;
	case 0x2e: return (void *)gte_avsz4;
#ifndef GTE_ASM
	case 0x30: return (void *)gte_fpu_rtpt;
#endif
	case 0x3d: return (void *)gte_gpf;
	case 0x3e: return (void *)gte_gpl;
	case 0x3f: return (void *)gte_ncct;
	default: return NULL;
	}
}

/* Whether the recompiler may emit NCLIP inline. */
int gte_fpu_nclip_inline(void)
{
	return 1;
}

/* The shape the core's `psxCP2` table wants: no op word, it is read back
 * out of `psxRegs.code`, which both callers assign before dispatching. */
#ifndef FGL_LINUX
static void gte_fpu_slot(psxCP2Regs *r)
{
	gte_fpu_cmd(r, psxRegs.code);
}
#endif

/* Take the table over, so nothing runs two GTE implementations at once. */
void gte_fpu_install(void)
{
#ifdef FGL_LINUX
	/* No psxCP2[] table here: fgl_gte_dispatch (fgl_run.c) is the one
	 * entry for the dispatcher and the lockstep oracle alike. */
#else
	int i;

	for (i = 0; i < 64; i++)
		psxCP2[i] = gte_fpu_slot;
#endif
}
