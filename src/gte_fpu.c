// SPDX-License-Identifier: GPL-2.0-only
/*
 * The GTE's coordinate transform, on the SH-4 FPU.
 *
 * RTPS is a 3x3 fixed-point matrix product with a translation, a reciprocal,
 * and a stack of saturating limits.  The product is the part worth moving off
 * the integer unit: nine 16x16 multiplies accumulated in 64 bits become one
 * `ftrv`, because the matrix rows carry the >> 12 as a 1/4096 premultiply and
 * the translation rides in the fourth column against a w of 1.0.
 *
 * The perspective divide is an `fdiv` rather than the reference's table-driven
 * reciprocal - see `gte_divide` for what that costs, because it is not free:
 * the table reproduces the PS1's quantisation and a float divide is *more*
 * accurate, which shows up as a different answer.  The limits and the flags
 * after it are integer and are exactly the reference's.
 *
 * **The rounding is not the shift, and the difference is shipped.**  The
 * reference builds MAC with `>> 12` on a signed 64-bit value, which floors;
 * `ftrc` truncates toward zero, one apart on every negative with a fraction.
 * A five-instruction-per-component correction closed that (23% -> 99.8%
 * bit-match, `docs/tests/test_gte.c`, gotcha 40) and was then REMOVED: the
 * bare truncation is visually correct and the correction cost ~15
 * instructions plus FPU latency on every vertex.  If a title ever shows
 * one-unit vertex jitter, this is the first suspect.
 *
 * XMTRX belongs to this file when `WITH_FPU_GTE` is set.  It otherwise holds
 * the PVR's screen transform, which is a diagonal scale being applied with a
 * 4x4 instruction; `src/pvr.c` does that with four multiplies instead when
 * this file is built, and the back bank is free for a product that actually
 * needs it.  The two cannot both own it, which is why they share one option.
 *
 * Copyright (C) 2026 bloom contributors
 */

#include <string.h>

#include <libpcsxcore/r3000a.h>
#include <libpcsxcore/gte.h>
#include <libpcsxcore/gte_divider.h>
#include <libpcsxcore/psxinterpreter.h>

#include "bloom-config.h"
#include "census.h"
#include "gte_fpu.h"
#include "prof.h"
#if WITH_GTE_PROFILE
#include "gteprof.h"
#endif

/*
 * Every command is its own function.
 *
 * They used to be inlined into one dispatch frame, on the reasoning that a
 * removed call boundary is a removed cost.  It is not when the bodies differ
 * this much: the shared frame is sized by the largest of them - seven
 * registers, `pr` and eighty bytes of stack - and the smallest commands pay
 * all of it.  NCLIP is three multiplies and was measured at 237 cycles inside
 * that frame against 82 for the same arithmetic in a function of its own.
 *
 * NCLIP stays inline because it is a third of all GTE calls and small enough
 * to set the dispatch frame rather than pay someone else's.
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
 * The light matrix (`LLM`) and the light colour matrix (`LCM`) are not named
 * individually: they sit at the same stride as the rotation matrix above -
 * five packed words then three translations, eight words apart - and the
 * matrix cache addresses all three that way.  So `LLM` is matrix 1 with the
 * background colour as its translation column at matrix 1's slot, and `LCM` is
 * matrix 2 with the far colour at matrix 2's.
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

/*
 * The bits are gte.c:237-251, including the one that is easy to get wrong:
 * **limB3 sets bit 22 and not the master bit 31**, while limB1 and limB2 set
 * both.  Games read FLAG.
 */
/*
 * The state every command reads or writes, in one place.
 *
 * These were seven file-scope variables in whatever order the linker felt
 * like, so a single RTPS could touch four or five different cache lines to
 * read a flag word, two screen offsets and a generation counter.  RTPS is
 * called 257,286 times in a bench window with the guest streaming vertex data
 * between calls, so none of them stays resident on its own.
 *
 * 32-byte aligned and under 32 bytes, so it is one line.
 */
struct gte_hot {
	u32 flag;		/* accumulated during a command          */
	float ofx, ofy;		/* OFX/OFY in screen units, not 16.16    */
	u32 ofs_gen;		/* psxCP2CtrlGen when ofx/ofy were built */
	u32 mtx_serial;		/* bumped whenever a cached matrix moves */
	int xmtrx_mx;		/* which matrix XMTRX holds, -1 for none */
	u32 xmtrx_serial;
} __attribute__((aligned(32)));

static struct gte_hot gte_hot = { .xmtrx_mx = -1 };

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
 * The 32-bit overflow check.
 *
 * **It raises the flag and does not clamp.**  The reference's `BOUNDS` returns
 * the value it was given and the assignment to a 32-bit MAC truncates it, so
 * an overflowing MAC wraps; clamping here instead would be a different number
 * in the guest's register.  The saturating form belongs to the FPU path below,
 * where `ftrc` has already saturated and only the flag is left to raise.
 */
static s32 gte_bounds(s64 v, u32 maxflag, u32 minflag)
{
	if (v > 0x7fffffff)
		gte_flag |= maxflag;
	else if (v < -(s64)0x80000000)
		gte_flag |= minflag;

	return (s32)v;
}

/*
 * `lm` selects the lower bound: 0 for the transforms, which allow negative
 * IR, and 1 for the lighting stages, which clamp at zero.  The reference
 * spells it `-0x8000 * !l`.
 */
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
 * `ftrc` saturates to these on overflow, which is how the transform reports
 * that MAC left the 32-bit range - the reference raises A1/A2/A3 from the full
 * 64-bit intermediate, which is not available here.  A true result of exactly
 * INT_MAX raises the flag spuriously; the alternative is carrying the
 * pre-clamp magnitude out of the FPU, and this is one guest-visible bit
 * against three instructions per component.
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
 * The register file holds three 3x3 matrices and four translation columns, at
 * a uniform stride: matrix `mx` is five packed words at `CP2C[mx * 8]`, and
 * translation `cv` is three words at `CP2C[cv * 8 + 5]`.  Every matrix stage
 * in the instruction set is one of the nine combinations, so one cache serves
 * all of them - rotation with TR for the transforms, the light matrix with no
 * translation and the light colour matrix with the background colour for the
 * lighting commands, and whatever pair MVMVA's selectors name.
 *
 * XMTRX order is column-major: `ftrv` computes
 * result[i] = sum over j of xf[i + 4j] * fv[j], and eight `fmov.d` fill
 * xf0..xf15 straight from the array.  8-byte alignment is required by the pair
 * form; 32 gets each entry into two cache lines.
 *
 * Rebuilding is nine integer-to-float conversions and twelve multiplies, so it
 * is worth avoiding across the hundreds of vertices a frame transforms with
 * one matrix.
 *
 * **Validity is one compare, not eight.**  It used to re-read the five matrix
 * words and three translation words out of the control file every command and
 * diff them against a cached copy - about twenty instructions that **measured
 * 219 cycles a call**, an IPC of 0.09, because the two lines it touches are
 * evicted by the guest's own vertex traffic between calls.  That was the
 * largest single phase of RTPS, larger than the `ftrv` at 80.
 *
 * `psxCP2CtrlGen` (`libpcsxcore/gte.c`) counts writes to the control file.
 * `CTC2` is the only way guest code reaches those registers and both engines
 * call it, so one counter covers the interpreter as well - which is what an
 * earlier comment here said was impossible and why the comparison existed.
 * The counter is conservative: a write to `H` or `OFX` invalidates all three
 * matrices, and that is fine, because those move per scene and the matrices
 * move per object.
 */
#define GTE_MTX_ROT    0
#define GTE_MTX_LIGHT  1
#define GTE_MTX_COLOUR 2

/* The translation selector for a stage that has none.  The reference spells
 * it as an out-of-range index whose accessors return zero. */
#define GTE_CV_NONE 3

struct gte_mtx_cache {
	float m[16];
	u32 src[5];		/* the packed matrix words, as last seen   */
	u32 tsrc[3];		/* the translation words, as last seen     */
	int tsel;		/* which translation the column holds      */
	int built;
	u32 gen;		/* psxCP2CtrlGen when m[] was built        */
	u32 serial;		/* bumped whenever m[] changes             */
} __attribute__((aligned(32)));

static struct gte_mtx_cache gte_cache[3];
#define gte_mtx_serial	(gte_hot.mtx_serial)

/*
 * Rebuild whatever the control file changed.  Out of line and cold: the check
 * that decides whether to come here is inline, and it holds on all but a
 * couple of thousand of the million-odd matrix uses in a bench window.
 */
static void gte_mtx_rebuild(const psxCP2Regs *r, int mx, int cv)
{
	struct gte_mtx_cache *c = &gte_cache[mx];
	const u32 *src = &r->CP2C.r[mx * 8];
	int i;

	c->gen = psxCP2CtrlGen;

	if (!c->built || memcmp(c->src, src, sizeof(c->src))) {
		const PAIR *p = &r->CP2C.p[mx * 8];

		for (i = 0; i < 5; i++)
			c->src[i] = src[i];

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
		c->m[15] = 1.0f;
		c->built = 1;
		c->serial = ++gte_mtx_serial;
	}

	if (cv == GTE_CV_NONE) {
		if (c->tsel != cv || c->m[12] || c->m[13] || c->m[14]) {
			c->tsel = cv;
			c->m[12] = c->m[13] = c->m[14] = 0.0f;
			c->serial = ++gte_mtx_serial;
		}
	} else {
		const u32 *t = &r->CP2C.r[cv * 8 + 5];

		if (c->tsel != cv || memcmp(c->tsrc, t, sizeof(c->tsrc))) {
			c->tsel = cv;
			for (i = 0; i < 3; i++) {
				c->tsrc[i] = t[i];
				c->m[12 + i] = (float)(s32)t[i];
			}
			c->serial = ++gte_mtx_serial;
		}
	}
}

/* One compare against the control file's write counter, and on the path that
 * matters it is the only memory this touches. */
static inline __attribute__((always_inline)) const float *
gte_mtx_get(const psxCP2Regs *r, int mx, int cv)
{
	struct gte_mtx_cache *c = &gte_cache[mx];

	if (!(c->built && c->gen == psxCP2CtrlGen && c->tsel == cv))
		gte_mtx_rebuild(r, mx, cv);

	return c->m;
}

/*
 * Put a matrix in XMTRX, skipping the load if it is already there.
 *
 * RTPS transforms one vertex per call and always against matrix 0, so without
 * this every call spends eight `fmov.d` and two `fschg` reloading the same
 * sixteen words - 257,286 times in a bench window, measured.  The cache's
 * serial says whether the array changed since XMTRX was filled from it, so a
 * rebuild invalidates residency without anything else having to notice.
 *
 * **Residency assumes nothing else disturbs the back bank**, and that spans
 * arbitrary guest code, interrupts and thread switches rather than the few
 * instructions between a load and its `ftrv`.  Nothing in the tree touches XF
 * while this file is built - `src/pvr.c` uses multiplies instead of an `ftrv`
 * under the same option, and no KOS matrix call is linked - but the failure
 * mode if something does is wrong geometry rather than a crash.
 */

static void gte_mtx_load(const float *m);

#define gte_xmtrx_mx		(gte_hot.xmtrx_mx)
#define gte_xmtrx_serial	(gte_hot.xmtrx_serial)

static inline __attribute__((always_inline)) void
gte_mtx_use(const psxCP2Regs *r, int mx, int cv)
{
	const float *m = gte_mtx_get(r, mx, cv);

	if (gte_xmtrx_mx == mx && gte_xmtrx_serial == gte_cache[mx].serial)
		return;

	gte_xmtrx_mx = mx;
	gte_xmtrx_serial = gte_cache[mx].serial;

	gte_mtx_load(m);
}

/*
 * Drop everything derived from the control file.
 *
 * The generation counter only sees `CTC2`.  A reset or a savestate load writes
 * `CP2C` straight into the register file, which no counter can observe, so the
 * one path that does that says so explicitly.
 */
void gte_fpu_reset(void)
{
	int i;

	for (i = 0; i < 3; i++) {
		gte_cache[i].built = 0;
		gte_cache[i].tsel = -1;
	}

	gte_xmtrx_mx = -1;

	/* The offsets are derived from the control file as well, and a reset
	 * writes it without CTC2.  `ofs_gen` is invalid rather than stale. */
	gte_hot.ofs_gen = psxCP2CtrlGen - 1u;
}

/* ------------------------------------------------------------------ */
/* The transform                                                       */
/* ------------------------------------------------------------------ */

#ifdef __sh__
/*
 * Fill XMTRX from the cached float matrix.
 *
 * `fschg` switches `fmov` to the pair form so this is eight 64-bit moves - the
 * SH-4 writes a pair as two 32-bit words in address order, so the array reads
 * into xf0..xf15 as laid out (KOS `mat_load` relies on the same thing).  It is
 * toggled back before anything else runs.
 *
 * **Separate from the transform because RTPT has three vertices and one
 * matrix.**  Folded together it reloaded XMTRX per vertex - twenty-four moves
 * where eight do, which is about what the transform saves in the first place.
 *
 * Nothing between this and the transforms disturbs the back bank: the limits
 * and the divide are integer, and no other code in the tree touches XF.
 */
static void gte_mtx_load(const float *m)
{
	__asm__ __volatile__(
		"fschg\n\t"
		"fmov.d	@%[m]+, xd0\n\t"
		"fmov.d	@%[m]+, xd2\n\t"
		"fmov.d	@%[m]+, xd4\n\t"
		"fmov.d	@%[m]+, xd6\n\t"
		"fmov.d	@%[m]+, xd8\n\t"
		"fmov.d	@%[m]+, xd10\n\t"
		"fmov.d	@%[m]+, xd12\n\t"
		"fmov.d	@%[m]+, xd14\n\t"
		"fschg"
		: [m] "+r" (m)
		:
		: "memory");
}

/*
 * One vertex: transformed, truncated, stored to MAC, clamped into IR.
 *
 * This is `gte_xform` plus the block that used to follow it in C, fused.  The
 * C version wrote three results to a stack array, read them back, stored them
 * to MAC, read *those* back for the saturation flags, clamped, and stored
 * again - the arithmetic is one `ftrv` and the rest was the register file
 * going to memory and coming back.
 *
 * The layout constants are the COP2 file's, and they are why there are three
 * base registers: `mov.l` reaches 60 bytes from a base and `mov.w` reaches 30,
 * while MAC1 sits at 100 and IR1 at 36.
 *
 *   r4  the register file          IR1/2/3 at 36/40/44 (halfwords)
 *   r6  r4 + 64                    MAC1/2/3 at 36/40/44
 *   r2  r4 + 36                    walked forward across the three IR slots
 *
 * The flag word is accumulated in r7 and OR'd into `gte_flag` once, rather
 * than read-modify-written per limit the way the C helpers did.
 *
 * Saturation is the reference's shape: the in-range path falls straight
 * through and each bound branches out to a stub that sets the flag and
 * substitutes the bound.  `limB3` sets bit 22 and **not** the master bit,
 * which is the asymmetry the whole file keeps warning about.
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

		/*
		 * The flag words live here rather than in the pool above:
		 * `mov.l` is PC-relative forward only and reaches 1020 bytes,
		 * which the stubs are past.  Nothing falls through into this -
		 * the stub above ends in a branch and its delay slot.
		 */
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
 * One vertex through the resident matrix, truncated (`ftrc`, see the header
 * note on rounding), as three s32.
 *
 * Only fr0-fr3, r0, r1, FPUL and T are touched, all caller-saved, so this
 * needs no prologue.
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

/*
 * Three floats to three truncated 32-bit results (`ftrc`, see the header
 * note on rounding), for the commands whose products are three multiplies
 * rather than a matrix: the caller does the arithmetic in C floats and this
 * does the narrowing.
 */
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
 * The host stand-in, so `docs/tests/test_gte.c` exercises everything in this
 * file except the assembly.  Same arithmetic, same order, same precision -
 * single, floored - which is what makes the measured match rate mean anything.
 * The resident matrix is a pointer here; on the target it is XMTRX.
 */
#include <math.h>

static const float *gte_resident;

static void gte_mtx_load(const float *m)
{
	gte_resident = m;
}

static void gte_xform(const s16 *v, s32 *mac)
{
	const float *m = gte_resident;
	int i;

	for (i = 0; i < 3; i++) {
		float acc = m[i] * (float)v[0] + m[4 + i] * (float)v[1] +
			    m[8 + i] * (float)v[2] + m[12 + i];

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

/*
 * The `sf` field as a multiplier, which is how it enters a float pipeline: the
 * reference's `>> 12` is a premultiply by 1/4096, and no shift is a multiply by
 * one.  Indexed by the bit, so the branch on `sf` disappears.
 */
static const float gte_sf_scale[2] = { 1.0f, 1.0f / 4096.0f };

/*
 * The perspective divide.
 *
 * The reference walks a 257-entry table and refines a reciprocal, which opens
 * with a count of leading zeros - and **SH-4 has no such instruction**, so the
 * compiler expands it before the divide even starts.  One `fdiv` replaces the
 * whole thing.
 *
 * The guard is the reference's own: the hardware returns all-ones when the
 * numerator reaches twice the denominator, and `limE` then clamps it.  That
 * comparison stays integer because it decides which path runs, not a value.
 *
 * `n << 16` is exact as a float - shifting only moves the exponent - and the
 * result is under 2^17 by the guard, so the only rounding is the divide's.
 * That makes this *more* accurate than the hardware, which is its own kind of
 * wrong: the table reproduces the PS1's quantisation and this does not.  The
 * residue is measured rather than argued about (`docs/tests/test_gte.c`).
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
 * results land, because that is the only thing RTPS and RTPT disagree about -
 * RTPS pushes a three-deep stack, RTPT writes the three slots directly.
 * Returns the quotient, which the depth cue needs from the last vertex.
 */
/*
 * Truncation, not floor.  The reference's `>>` floors and `ftrc` truncates,
 * one apart on every negative with a fraction; the explicit correction was
 * measured at 23% -> 99.8% bit-match and then dropped anyway, everywhere in
 * this file at once, for the header note's reasons.  One fidelity level,
 * chosen once - do not floor some paths and truncate others.
 */
static s32 gte_floor_small(float v)
{
	return (s32)v;
}

/*
 * A projected coordinate, saturated the way the reference saturates it.
 *
 * The reference bounds the 16.16 sum at +-2^31 and *then* shifts, so the
 * saturated result reaching the screen clamp is +-2^15, not the clamp itself -
 * and it is still written.  Returning early on overflow instead leaves the
 * previous vertex's coordinate in place, which is a 1024-unit error rather
 * than the one-unit kind everything else here produces.
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

/*
 * OFX and OFY as screen units rather than 16.16, rebuilt per command because
 * they do not change across a triangle's vertices.
 */
/*
 * OFX and OFY as screen units rather than 16.16.
 *
 * Rebuilt only when the control file has been written, on the same counter the
 * matrix cache uses: these move per scene and RTPS is called per vertex, so
 * recomputing them every call was two loads, two conversions, two multiplies
 * and two stores for a value that had not changed.
 */
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

	/*
	 * The `ftrc` saturation flags (bits 30-25) are NOT raised: they were
	 * three MAC read-backs and six compares per vertex, firing on nothing
	 * a game reaches - the file's own note said so while still paying for
	 * them.  Flag-only, no coordinate changes.
	 */
	*szp = limD(D_MAC3);

	quotient = limE(gte_divide(C_H, *szp));

	/*
	 * The projection, in float.  The reference forms
	 * `(OFX + IR1 * quotient) >> 16` as a 64-bit product; here the
	 * quotient is scaled to screen units first, so the same value comes
	 * out of one multiply and one add with no 64-bit arithmetic at all.
	 *
	 * `quotient` is under 2^17 and the scale is a power of two, so `qs` is
	 * exact - this keeps the *reference's* precision rather than the
	 * divide's, which matters: a more accurate quotient here would diverge
	 * further, not less.
	 *
	 * The saturation flag still has to come from the unclamped value, and
	 * the reference raises it at 2^31 before the shift - which is 2^15
	 * after it.
	 */
	qs = (float)quotient * (1.0f / 65536.0f);

	*sxp = limG1(gte_project(gte_ofx + (float)D_IR1 * qs));
	*syp = limG2(gte_project(gte_ofy + (float)D_IR2 * qs));

	return quotient;
}

/*
 * Publish the accumulated flags.
 *
 * The master bit is *not* recomputed here.  The reference's flag constants
 * carry bit 31 individually - and two of them deliberately do not, `limB3` and
 * the three A-maxima - so deriving it from a mask instead sets it in cases the
 * reference leaves clear.  It only recomputes on a `CTC2` write to FLAG, which
 * is the core's own path and not this one.
 */
static void gte_end(psxCP2Regs *r)
{
	C_FLAG = gte_flag;
}

/* The depth cue.  Both transforms end this way. */
/*
 * RTPT's copy, out of line.  Three vertices through an inlined body would put
 * three copies of it in an 8 KB direct-mapped instruction cache, and RTPT is
 * an eighth of RTPS's call count.
 */
static u32 gte_rtp_one(psxCP2Regs *r, int v, u16 *szp, s16 *sxp, s16 *syp)
{
	return gte_rtp_one_i(r, v, szp, sxp, syp);
}

static inline __attribute__((always_inline)) void
gte_rtp_end_i(psxCP2Regs *r, u32 quotient)
{
	s64 tmp = (s64)C_DQB + (s64)C_DQA * quotient;

	/* The 32-bit overflow flags (bits 16/15) are not raised - a 64-bit
	 * bounds compare per command whose only output was FLAG bits no game
	 * hits.  MAC0 still takes the truncating store the reference takes. */
	D_MAC0 = (s32)tmp;
	D_IR0 = limH((s32)(tmp >> 12));

	gte_end(r);
}

__attribute__((noinline)) void gte_fpu_rtps(psxCP2Regs *r)
{
	u32 quotient;

	gte_flag = 0;

	gte_mtx_use(r, GTE_MTX_ROT, 0);

	gte_offsets(r);

	/*
	 * Both stacks shift before the new vertex lands, which is what the
	 * reference does either side of computing it - nothing in between
	 * depends on the old values.
	 */
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
 * RTPT - the same transform on all three vertex registers.
 *
 * It is not RTPS three times: there is no stack here.  SZ0 takes the *old* SZ3
 * once, before anything is written, and each vertex then writes its own slot -
 * SZ1, SZ2, SZ3 and SXY0, SXY1, SXY2.  MAC, IR and the quotient are left
 * holding the third vertex's values, which is what the depth cue then uses.
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

/* ------------------------------------------------------------------ */
/* The shared stages                                                   */
/* ------------------------------------------------------------------ */

/*
 * The vector a command multiplies.
 *
 * Selector 3 names IR1..IR3, which are one halfword every four bytes rather
 * than three in a row, so that case is copied into the caller's scratch and
 * everything downstream reads one contiguous triple.
 */
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

/*
 * The light stage: MAC = (LLM * V) >> 12, IR clamped at zero.
 *
 * No translation column and no overflow flags - the reference raises none
 * here, and lets an overflowing MAC wrap.  `ftrc` saturates instead, which is
 * the same trade the transform documents above and reachable only by a normal
 * vector far outside anything a light source is given.
 */
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
 * The colour stage: MAC = (BK << 12 + LCM * IR) >> 12.
 *
 * The background colour is the translation column, exactly as TR is for the
 * transform, so this is the same `ftrv` against a different matrix.
 *
 * **It leaves IR alone**, because NCT is the one command that does not clamp
 * after this stage - it runs three of them and only clamps once, at the end.
 * Folding the clamp in here would raise `limB` flags on its first two
 * iterations that the reference does not.
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
 * The general matrix product, five selector fields wide.
 *
 * Two of the nine (matrix, translation) pairs the selectors can name are not
 * matrices at all - index 3 reads off the end of the file, and the reference
 * substitutes zeros - and `sf` can ask for no shift, which would put the
 * translation in at 2^12 times its value and past what a float mantissa holds
 * exactly.  Both go down the integer path, which is the reference's arithmetic
 * transcribed; it is exact, and neither case is one a game issues in a loop.
 */
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

/*
 * Everything below is three to nine multiplies of 16-bit quantities and stays
 * on the integer unit, where it is bit-exact with the reference.  The FPU is
 * worth an `ftrv` when nine multiplies accumulate into 64 bits; it is not
 * worth a mode switch for three that fit in 32.
 */

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
 * SQR, OP and GPF are three independent products and an optional `>> 12`.
 *
 * **`sf` decides which unit runs them, and it is not a preference.**  At
 * `sf = 1` the `>> 12` is exactly the premultiply a float pipeline wants, the
 * results are bounded by 2^18, and the FPU is free to have them.  At `sf = 0`
 * the reference's answer is an exact 32-bit integer: OP subtracts two products
 * that each reach 2^24, so the result needs 25 bits and a single-precision
 * mantissa holds 24.  Measured, before this split: OP agreed with the
 * reference on 96.66% of cases, all of the loss in the `sf = 0` half.
 *
 * The integer arm is also the cheaper of the two integer forms - `sf = 0` is a
 * bare multiply with no shift at all - so nothing is given up by leaving it
 * there.
 *
 * None of the three raises an overflow flag: the reference assigns MAC
 * directly, with no `A1`/`A2`/`A3`.
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
 * Every command the GTE has, in this file.  The core's `psxCP2` table is not
 * reached, which is the point: one command left behind would keep the core's
 * `gte.c` in the instruction cache, and that working-set cost is the larger
 * half of what moving RTPS to the FPU bought.
 *
 * A function the hardware does not implement does nothing and does not touch
 * FLAG, which is the core's `gteNULL`.
 */
static inline void gte_dispatch(psxCP2Regs *r, u32 op)
{
	switch (op & 0x3f) {
	case 0x01: gte_fpu_rtps(r); break;
	case 0x06: gte_nclip(r); break;
	case 0x0c: gte_op(r, op); break;
	case 0x10: gte_dpcs(r, op); break;
	case 0x11: gte_intpl(r, op); break;
	case 0x12: gte_mvmva(r, op); break;
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
	case 0x30: gte_fpu_rtpt(r); break;
	case 0x3d: gte_gpf(r, op); break;
	case 0x3e: gte_gpl(r, op); break;
	case 0x3f: gte_ncct(r); break;
	default: break;
	}
}

/*
 * A GTE command.  `op` is the whole instruction word, not just the function
 * field: the shift, the saturation flag and MVMVA's three selectors live in
 * the upper bits.
 *
 * The core takes those from `#define gteop (psxRegs.code & 0x1ffffff)` - a
 * parameter passed through a global that nothing at the call site makes
 * visible - so a caller that has the word in hand passes it instead.
 *
 * The three transforms and NCLIP are compares rather than entries in the
 * 64-way switch, which lowers to an indirect branch through a table nothing
 * else touches.  RTPS and NCLIP alone are 92% of all GTE calls in Spyro; RTPT
 * is here because other titles lean on it and a third compare costs the rare
 * commands nothing measurable.  The two hot commands inline into this frame:
 * RTPS used to be reached through four C frames and is 81% of GTE time, so
 * their prologues were stack traffic on the hottest path there is.
 *
 * GTE stalls are not modelled; this is the core's own no-stall path.
 */
void gte_fpu_cmd(psxCP2Regs *r, u32 op)
{
	u32 fn = op & 0x3f;
#if WITH_GTE_PROFILE
	GTEPROF_ENTER;
#endif

	/* Generated code calls here directly, skipping cop2_op (the plugin's
	 * bracketed entry), so the profiler bucket and the census histogram
	 * live here — they are what makes a PROF/CENSUS run's GTE numbers
	 * complete, and they compile to nothing when off. */
	prof_enter(PROF_GTE);
	census_gte(op);

	if (fn == 0x01)
		gte_fpu_rtps(r);
	else if (fn == 0x06)
		gte_nclip(r);
	else if (fn == 0x30)
		gte_fpu_rtpt(r);
	else
		gte_dispatch(r, op);

	prof_leave();

#if WITH_GTE_PROFILE
	GTEPROF_LEAVE(fn);
#endif
}

/*
 * Compile-time dispatch for the recompiler: it knows the command word when
 * it compiles the block, so the body is resolved once, here, and called
 * directly - no runtime dispatch at all.  NULL means the hardware treats
 * the command as a no-op and the block emits nothing for it.
 *
 * The bodies disagree about taking the op word, and the caller always passes
 * it: an extra argument register is invisible to a function that does not
 * name it, which is the same ABI fact `psxCP2[]`'s single-argument entries
 * rely on.
 *
 * Instrumented builds resolve everything to `gte_fpu_cmd` instead, because
 * that is where the PROF bracket and the census histogram live - the
 * measurement contract outranks the dispatch saving it would be measuring.
 */
void *gte_fpu_resolve(u32 op)
{
#if defined(BLOOM_PROF) || defined(BLOOM_CENSUS) || WITH_GTE_PROFILE
	(void)op;
	return (void *)gte_fpu_cmd;
#else
	switch (op & 0x3f) {
	case 0x01: return (void *)gte_fpu_rtps;
	case 0x06: return (void *)gte_nclip_cmd;
	case 0x0c: return (void *)gte_op;
	case 0x10: return (void *)gte_dpcs;
	case 0x11: return (void *)gte_intpl;
	case 0x12: return (void *)gte_mvmva;
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
	case 0x30: return (void *)gte_fpu_rtpt;
	case 0x3d: return (void *)gte_gpf;
	case 0x3e: return (void *)gte_gpl;
	case 0x3f: return (void *)gte_ncct;
	default: return NULL;
	}
#endif
}

/*
 * Whether the recompiler may emit NCLIP inline instead of calling anything -
 * it is a third of all GTE commands and small enough that a call frame
 * costs more than the command.  Refused on instrumented builds for the same
 * reason as above: an inline NCLIP would vanish from the GTE bucket and the
 * census histogram, and the workload counters must stay complete.
 */
int gte_fpu_nclip_inline(void)
{
#if defined(BLOOM_PROF) || defined(BLOOM_CENSUS) || WITH_GTE_PROFILE
	return 0;
#else
	return 1;
#endif
}

/*
 * The shape the core's `psxCP2` table wants: no op word, because every entry
 * in that table reads it back out of `psxRegs.code`.  Both callers - the
 * interpreter and the GTE-in-a-delay-slot path in `psxException` - assign it
 * before dispatching, so one function serves all sixty-four slots.
 */
static void gte_fpu_slot(psxCP2Regs *r)
{
	gte_fpu_cmd(r, psxRegs.code);
}

/*
 * Take the table over, so a core that dispatches through it gets this GTE too.
 * Without it a build could run two implementations at once depending on which
 * path reached the coprocessor.
 */
void gte_fpu_install(void)
{
	int i;

	for (i = 0; i < 64; i++)
		psxCP2[i] = gte_fpu_slot;
}
