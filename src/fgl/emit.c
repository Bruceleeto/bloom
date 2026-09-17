#include <stdio.h>
#include <stdlib.h>
/* The emit pass: IR nodes to SH-4.
 *
 * One linear walk. The allocation pass has already said where every operand
 * lives -- a pool register, or -1 for "still in the state block" -- so there
 * is one code path per node and the out-of-registers case is not a special
 * one, it just reads through r0 like any other state access.
 *
 * The contract this emits under is in fgl.h and is not negotiable here.
 */

#include "fgl.h"
#include "fgl_state.h"
#include "pins.h"
#include "gte.h"

#include <string.h>

/* Word index of the next instruction to be emitted. Displacement fixups are
 * recorded in these units, not bytes, because that is what patching wants. */
static int here(const fgl_emitter *e)
{
	return (int)((e->cg.ptr - e->start) / 2);
}

uint32_t fgl_size(const fgl_emitter *e)
{
	return (uint32_t)(e->cg.ptr - e->start);
}

void fgl_init(fgl_emitter *e, void *buf, uint32_t size, uint32_t base)
{
	memset(e, 0, sizeof(*e));
	e->cg.ptr = (uint8_t *)buf;
	e->cg.end = (uint8_t *)buf + size;
	e->start  = (uint8_t *)buf;
	e->base   = base;
}

void fgl_set_targets(fgl_emitter *e, const fgl_targets *t)
{
	e->tgt = t;
}

/* OR bits into an already-emitted word: how every displacement that was not
 * known at emission time gets filled in. */
static void or_word_at(fgl_emitter *e, int at, uint16_t bits)
{
	uint8_t *p = e->start + 2 * at;
	uint16_t w = (uint16_t)(p[0] | (p[1] << 8));

	w |= bits;
	p[0] = (uint8_t)(w & 0xff);
	p[1] = (uint8_t)(w >> 8);
}

/* Flip bits of an already-emitted word: `bf` <-> `bt` is one bit. */
static void xor_word_at(fgl_emitter *e, int at, uint16_t bits)
{
	uint8_t *p = e->start + 2 * at;
	uint16_t w = (uint16_t)(p[0] | (p[1] << 8));

	w ^= bits;
	p[0] = (uint8_t)(w & 0xff);
	p[1] = (uint8_t)(w >> 8);
}

static int bt_fwd(fgl_emitter *e) { int s = here(e); sh4_emit_bt(&e->cg, 0); return s; }
static int bf_fwd(fgl_emitter *e) { int s = here(e); sh4_emit_bf(&e->cg, 0); return s; }
static int emit_delayed(fgl_emitter *e, uint16_t branch, uint32_t reads);
static void emit_const(fgl_emitter *e, uint32_t v, int rn);
static void emit_svc(fgl_emitter *e, unsigned idx, int rn);
static void slot_barrier(fgl_emitter *e) { e->slot_floor = here(e); }
static void peep_hoist(fgl_emitter *e);
/* `bra` with its slot filled from what preceded it when that is possible;
 * returns the index the `bra` landed on. */
static int bra_fwd(fgl_emitter *e) { return emit_delayed(e, 0xa000u, 0); }

static void patch_fwd8(fgl_emitter *e, int site)
{
	int disp = here(e) - site - 2;

	if (!sh4_disp8_fits(disp))
		e->overflow = 3;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xff));
	slot_barrier(e);
}

/* Patch a `bt`/`bf` at `site` to land at word `at`; 0 if out of reach. */
static int patch_cond8_to(fgl_emitter *e, int site, int at)
{
	int disp = at - site - 2;

	if (!sh4_disp8_fits(disp))
		return 0;
	or_word_at(e, site, (uint16_t)(disp & 0xff));
	return 1;
}

/* FGL_NO_LAYOUT: `f` keeps the fallthrough `bra`, `x` keeps one dispatcher
 * exit per basic block, `t` keeps the T park across the slot, `b` keeps the
 * `bf`+`bra` taken arm, anything else keeps all four.  Bisection only. */
static int layout_off(int which)
{
	static int v = -1;

	if (v < 0) {
		const char *s = getenv("FGL_NO_LAYOUT");

		v = 0;
		if (s) {
			v = 15;
			if (!strcmp(s, "f")) v = 1;
			if (!strcmp(s, "x")) v = 2;
			if (!strcmp(s, "t")) v = 4;     /* keep the T park   */
			if (!strcmp(s, "b")) v = 8;     /* keep bf+bra taken */
		}
	}
	return v & which;
}
static unsigned long layout_fall_n, layout_share_n;
/* Pass 2 only, so these count once. */
static unsigned long layout_bt_n, layout_bt_fall_n, layout_tlive_n;

static void patch_fwd12(fgl_emitter *e, int site)
{
	int disp = here(e) - site - 2;

	if (!sh4_disp12_fits(disp))
		e->overflow = 4;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xfff));
	slot_barrier(e);
}

/* ---------------------------------------------------------------- */
/* Delay-slot filling                                                */
/* ---------------------------------------------------------------- */

/* A LABEL IS A BARRIER.  Both patchers land a branch HERE, so nothing emitted
 * before this point may be lifted past it: a path arriving at the label would
 * skip the lifted instruction on the way in and then execute it in the slot
 * on the way out. */
/* WHAT ONE EMITTED WORD READS AND WRITES, for the filler and nothing else.
 *
 * Only the encodings sh4.h can produce are decoded; anything else is
 * `unknown`, which means "not liftable, and a wall for anything behind it".
 * Registers are bits 0-15.  T, the MAC pair and the M/Q divide bits get bits
 * of their own so a `cmp` cannot slide past a `bt` or a `div1` past its
 * `div0s`.  PC-relative loads are the one legal instruction a delay slot may
 * not hold (they and `mova` raise a slot-illegal exception on the hardware,
 * and the interpreter would silently read the wrong PC); they are decoded so
 * something else can be lifted OVER them, which is the whole point -- every
 * service call ends in two or three of them. */
#define SI_T    (1u << 16)
#define SI_MACL (1u << 17)
#define SI_MACH (1u << 18)
#define SI_MQ   (1u << 19)

enum { SM_NONE, SM_STATE, SM_STACK, SM_GUEST, SM_POOL };

typedef struct {
	uint32_t rd, wr;
	int      mem;           /* SM_*                              */
	int      mem_wr;        /* 1 = store                         */
	int      mem_off;       /* SM_STATE: byte offset, -1 = any   */
	int      legal;         /* may sit in a delay slot           */
	int      unknown;       /* not decoded: a wall               */
} slot_info;

/* r3 is the guest-file base only under FGL_GBASE; otherwise it is an
 * ordinary pool register and a memory operand off it is a guest access. */
#if FGL_GBASE
#define SLOT_GBASE FGL_R_GBASE
#else
#define SLOT_GBASE (-1)
#endif

/* r4 is the high half of the same window under FGL_GBASE2, so a memory
 * operand off it is state traffic too -- and its displacement is relative to
 * word FGL_GBASE2_LO, which the byte offset has to add back or two accesses
 * sixteen words apart look like the same one. */
#if FGL_GBASE2
#define SLOT_GBASE2 FGL_R_GBASE2
#define SLOT_GBASE2_BIAS FGL_GBASE2_LO
#else
#define SLOT_GBASE2 (-1)
#define SLOT_GBASE2_BIAS 0
#endif

static int slot_mem_class(int base, int off_words, int scale, slot_info *si)
{
	if (base == SLOT_GBASE) {
		si->mem = SM_STATE;
		si->mem_off = off_words < 0 ? -1 : off_words * scale;
	} else if (base == SLOT_GBASE2) {
		si->mem = SM_STATE;
		si->mem_off = off_words < 0 ? -1
					    : (off_words + SLOT_GBASE2_BIAS) * scale;
	} else if (base == 15) {
		si->mem = SM_STACK;
	} else {
		si->mem = SM_GUEST;
	}
	return 0;
}

static void slot_decode(uint16_t w, slot_info *si)
{
	unsigned hi = w >> 12, n = (w >> 8) & 15, m = (w >> 4) & 15;
	unsigned lo = w & 15, d8 = w & 0xff;
#define R(x) (1u << (x))

	memset(si, 0, sizeof(*si));
	si->legal = 1;
	si->mem_off = -1;

	switch (hi) {
	case 0x0:
		switch (w & 0xff) {
		case 0x09: return;                              /* nop */
		case 0x08: case 0x18: si->wr = SI_T; return;    /* clrt sett */
		case 0x19: si->wr = SI_T | SI_MQ; return;       /* div0u */
		case 0x28: si->wr = SI_MACL | SI_MACH; return;  /* clrmac */
		case 0x29: si->rd = SI_T; si->wr = R(n); return; /* movt */
		case 0x0a: si->rd = SI_MACH; si->wr = R(n); return;
		case 0x1a: si->rd = SI_MACL; si->wr = R(n); return;
		case 0x07: si->rd = R(n) | R(m); si->wr = SI_MACL; return;
		}
		switch (lo) {
		case 0xc: case 0xd: case 0xe:   /* mov.x @(r0,Rm),Rn */
			si->rd = R(0) | R(m); si->wr = R(n);
			slot_mem_class((int)m, -1, 1, si);
			return;
		case 0x4: case 0x5: case 0x6:   /* mov.x Rm,@(r0,Rn) */
			si->rd = R(0) | R(m) | R(n);
			slot_mem_class((int)n, -1, 1, si);
			si->mem_wr = 1;
			return;
		}
		break;
	case 0x1:                               /* mov.l Rm,@(d,Rn) */
		si->rd = R(m) | R(n);
		slot_mem_class((int)n, (int)lo, 4, si);
		si->mem_wr = 1;
		return;
	case 0x2:
		switch (lo) {
		case 0x0: case 0x1: case 0x2:   /* mov.x Rm,@Rn */
			si->rd = R(m) | R(n);
			slot_mem_class((int)n, (int)n == SLOT_GBASE ? 0 : -1, 1, si);
			si->mem_wr = 1;
			return;
		case 0x4: case 0x5: case 0x6:   /* mov.x Rm,@-Rn */
			si->rd = R(m) | R(n); si->wr = R(n);
			slot_mem_class((int)n, -1, 1, si);
			si->mem_wr = 1;
			return;
		case 0x7: si->rd = R(n) | R(m); si->wr = SI_T | SI_MQ; return;
		case 0x8: case 0xc: si->rd = R(n) | R(m); si->wr = SI_T; return;
		case 0x9: case 0xa: case 0xb: case 0xd:
			si->rd = R(n) | R(m); si->wr = R(n); return;
		}
		break;
	case 0x3:
		switch (lo) {
		case 0x0: case 0x2: case 0x3: case 0x6: case 0x7:
			si->rd = R(n) | R(m); si->wr = SI_T; return;
		case 0x4:                       /* div1 */
			si->rd = R(n) | R(m) | SI_T | SI_MQ;
			si->wr = R(n) | SI_T | SI_MQ; return;
		case 0x5: case 0xd:
			si->rd = R(n) | R(m); si->wr = SI_MACL | SI_MACH; return;
		case 0x8: case 0xc:
			si->rd = R(n) | R(m); si->wr = R(n); return;
		case 0xa: case 0xe:             /* subc addc */
			si->rd = R(n) | R(m) | SI_T; si->wr = R(n) | SI_T; return;
		case 0xb: case 0xf:             /* subv addv */
			si->rd = R(n) | R(m); si->wr = R(n) | SI_T; return;
		}
		break;
	case 0x4:
		switch (w & 0xff) {
		case 0x00: case 0x01: case 0x04: case 0x05:
		case 0x20: case 0x21: case 0x10:
			si->rd = R(n); si->wr = R(n) | SI_T; return;
		case 0x24: case 0x25:
			si->rd = R(n) | SI_T; si->wr = R(n) | SI_T; return;
		case 0x08: case 0x09: case 0x18: case 0x19: case 0x28: case 0x29:
			si->rd = R(n); si->wr = R(n); return;
		case 0x11: case 0x15:
			si->rd = R(n); si->wr = SI_T; return;
		case 0x0a: si->rd = R(n); si->wr = SI_MACH; return;
		case 0x1a: si->rd = R(n); si->wr = SI_MACL; return;
		}
		if (lo == 0xc || lo == 0xd) {   /* shad shld */
			si->rd = R(n) | R(m); si->wr = R(n); return;
		}
		break;
	case 0x5:                               /* mov.l @(d,Rm),Rn */
		si->rd = R(m); si->wr = R(n);
		slot_mem_class((int)m, (int)lo, 4, si);
		return;
	case 0x6:
		switch (lo) {
		case 0x0: case 0x1: case 0x2:   /* mov.x @Rm,Rn */
			si->rd = R(m); si->wr = R(n);
			slot_mem_class((int)m, (int)m == SLOT_GBASE ? 0 : -1, 1, si);
			return;
		case 0x6:                       /* mov.l @Rm+,Rn */
			si->rd = R(m); si->wr = R(n) | R(m);
			slot_mem_class((int)m, -1, 1, si);
			return;
		case 0xa:                       /* negc */
			si->rd = R(m) | SI_T; si->wr = R(n) | SI_T; return;
		case 0x3: case 0x7: case 0x8: case 0x9: case 0xb:
		case 0xc: case 0xd: case 0xe: case 0xf:
			si->rd = R(m); si->wr = R(n); return;
		}
		break;
	case 0x7: si->rd = R(n); si->wr = R(n); return;   /* add #imm */
	case 0x8:
		switch (n) {
		case 0x0: case 0x1:             /* mov.x r0,@(d,Rn) */
			si->rd = R(0) | R(m);
			slot_mem_class((int)m, -1, 1, si);
			si->mem_wr = 1;
			return;
		case 0x4: case 0x5:             /* mov.x @(d,Rm),r0 */
			si->rd = R(m); si->wr = R(0);
			slot_mem_class((int)m, -1, 1, si);
			return;
		case 0x8: si->rd = R(0); si->wr = SI_T; return;
		}
		break;                          /* bt bf bt/s bf/s */
	case 0x9:                               /* mov.w @(d,pc),Rn */
		si->wr = R(n); si->mem = SM_POOL; si->legal = 0; return;
	case 0xc:
		switch (n) {
		case 0x0: case 0x1:             /* mov.b/w r0,@(d,gbr) */
			si->rd = R(0); si->mem = SM_STATE; si->mem_wr = 1;
			si->mem_off = -1; return;
		case 0x2:                       /* mov.l r0,@(d,gbr) */
			si->rd = R(0); si->mem = SM_STATE; si->mem_wr = 1;
			si->mem_off = (int)d8 * 4; return;
		case 0x4: case 0x5:
			si->wr = R(0); si->mem = SM_STATE; si->mem_off = -1;
			return;
		case 0x6:
			si->wr = R(0); si->mem = SM_STATE;
			si->mem_off = (int)d8 * 4; return;
		case 0x7:                       /* mova */
			si->wr = R(0); si->mem = SM_POOL; si->legal = 0; return;
		case 0x8: si->rd = R(0); si->wr = SI_T; return;
		case 0x9: case 0xa: case 0xb:
			si->rd = R(0); si->wr = R(0); return;
		}
		break;
	case 0xd:                               /* mov.l @(d,pc),Rn */
		si->wr = R(n); si->mem = SM_POOL; si->legal = 0; return;
	case 0xe: si->wr = R(n); return;        /* mov #imm,Rn */
	}
	si->unknown = 1;
	si->legal = 0;
#undef R
}

/* May `a` (earlier) and `b` (later) change places? */
static int slot_independent(const slot_info *a, const slot_info *b)
{
	if (a->unknown || b->unknown)
		return 0;
	if ((a->wr & b->rd) || (a->rd & b->wr) || (a->wr & b->wr))
		return 0;
	if (a->mem == SM_NONE || b->mem == SM_NONE)
		return 1;
	if (!a->mem_wr && !b->mem_wr)
		return 1;
	if (a->mem == SM_POOL || b->mem == SM_POOL)
		return 1;                       /* nothing stores to a pool */
	if (a->mem != b->mem)
		return 1;                       /* the state block, the stack
						 * and guest memory are three
						 * different places */
	if (a->mem == SM_STATE && a->mem_off >= 0 && b->mem_off >= 0)
		return a->mem_off != b->mem_off;
	return 0;
}

static uint16_t word_at(const fgl_emitter *e, int at)
{
	const uint8_t *p = e->start + 2 * at;

	return (uint16_t)(p[0] | (p[1] << 8));
}

static void set_word_at(fgl_emitter *e, int at, uint16_t w)
{
	uint8_t *p = e->start + 2 * at;

	p[0] = (uint8_t)(w & 0xff);
	p[1] = (uint8_t)(w >> 8);
}

/* A DELAYED BRANCH WITH ITS SLOT FILLED FROM WHAT CAME BEFORE IT.
 *
 * `branch` is the word to emit and `reads` what it reads -- the target
 * register of a `jmp`/`jsr`, nothing for `bra`.  The filler looks back up to
 * SLOT_DEPTH words for one it can move: legal in a slot, independent of every
 * word between it and the branch, not writing anything the branch reads, and
 * not separated from the branch by a label (`slot_floor`).  It then rotates
 * that word into the slot and returns the index the branch landed on, which
 * is one less than `here` was when it was called.  With nothing to lift it
 * emits the branch and a `nop`, exactly as before.
 *
 * The slot runs BEFORE the transfer on SH-4 and in the interpreter alike, so
 * `A; B` and `B; A-in-slot` are the same program whenever A and B commute --
 * which the dependency test is.  Pool fixups whose sites slide down a word
 * are renumbered; tags travel with their words.
 *
 * The depth is small because everything a service call materialises is two
 * or three PC-relative loads, and past those is the instruction that built
 * the argument -- which is the one worth having. */
#ifndef SLOT_DEPTH
#define SLOT_DEPTH 4
#endif
static int slot_debug;

/* FGL_SLOT_STATS=1: how many delay slots were filled and how many stayed a
 * `nop`, plus why the unfilled ones failed.  The filler is the cheapest win
 * in the emitter -- a lifted instruction costs nothing and a `nop` costs a
 * whole issue slot -- so the fill RATE is the number worth watching, not the
 * code size it happens to save. */
static unsigned long slot_fill, slot_plain;
static unsigned long slot_why_floor;    /* the window was empty: barrier      */
static unsigned long slot_why_wall;     /* hit an undecodable word            */
static unsigned long slot_why_dep;      /* candidates existed, none commuted  */
static unsigned long slot_wall_hi[16];  /* which opcode class walled us       */
static unsigned long slot_xfer[16];     /* what each block ended in           */

/* Emitted BYTES by tag.  Counted in sh4_word (sh4.h) on the tagged pass. */
unsigned long fgl_tag_bytes[FGL_TAG_N];

/* THE SHAPE OF EVERY GUEST ADDRESS, because `mem` is 20% of emitted bytes and
 * the only way to know which collapse pays is to count which shapes occur.
 *
 * The one that matters is SH-4's `mov.l @(disp,Rm),Rn`: a 4-bit displacement,
 * scaled by four, so 0..60 aligned.  Where the base is already pinned and the
 * mask is proved away, that form turns the whole address calculation into
 * nothing at all and the access becomes the two bytes bleem pays.  Where the
 * mask survives, it still eats the `add`.
 *
 *   addr_n      every call
 *   addr_pin    base is in a host register already
 *   addr_mask   the mask survived
 *   addr_d0     displacement zero
 *   addr_disp   displacement 4..60 and word-aligned (the @(disp,Rm) window)
 *   addr_i8     displacement fits `add #imm` but not the window
 *   addr_wide   needs a pool word
 */
static unsigned long addr_n, addr_pin, addr_mask, addr_d0,
		     addr_disp, addr_i8, addr_wide;

/* Pool-requiring constants by destination, because only a constant headed for
 * r0 can come out of a GBR slot instead -- `mov.l @(disp,GBR)` has no register
 * field.  And link sites, to price the six dead bytes each one carries. */
static unsigned long konst_n, konst_r0, link_sites;
static unsigned long peep_hoist_n, peep_fwd_n, peep_cse_n;

/* WHICH CONSTANTS THE LITERAL POOL KEEPS RE-EMITTING.
 *
 * Constants are deduplicated WITHIN a block and not across blocks, so a
 * service address wanted by a thousand blocks costs four bytes a thousand
 * times -- and sits in the instruction stream, where the icache fetches it.
 * A value that repeats is a value that belongs in a GBR slot instead, which
 * is what bleem does (@(536,gbr), @(548,gbr), @(552,gbr)).  This says which
 * ones and how much they cost. */
#define POOL_HASH 8192
static struct { uint32_t v; unsigned long n; } pool_hist[POOL_HASH];
static unsigned long pool_words, pool_distinct;

static void pool_seen(uint32_t v)
{
	unsigned h = (v * 2654435761u) & (POOL_HASH - 1), i;

	pool_words++;
	for (i = 0; i < POOL_HASH; i++, h = (h + 1) & (POOL_HASH - 1)) {
		if (pool_hist[h].n && pool_hist[h].v != v)
			continue;
		if (!pool_hist[h].n) {
			pool_hist[h].v = v;
			pool_distinct++;
		}
		pool_hist[h].n++;
		return;
	}
}

static int pool_cmp(const void *a, const void *b)
{
	unsigned long x = ((const unsigned long *)a)[1];
	unsigned long y = ((const unsigned long *)b)[1];

	return x < y ? 1 : x > y ? -1 : 0;
}

/* THE CURVE THAT DECIDES WHETHER A SHARED TABLE IS WORTH IT.
 *
 * `mov.l @(disp,GBR),R0` reaches 1020 bytes with an 8-bit scaled
 * displacement, and the state block already ends at +716 -- so there are 76
 * words of GBR window left, and a constant living in one of them costs two
 * bytes and NO pool word at all.  That is bleem's shape exactly (`@(536,gbr)`,
 * `@(548,gbr)`, `@(552,gbr)`).
 *
 * Whether 76 slots are worth having is entirely a question of how steep the
 * head of this distribution is, so print the cumulative coverage at the cut
 * points rather than guessing from a top-12 list. */
static void pool_dump(void)
{
	static unsigned long ent[POOL_HASH][2];
	static const int cut[] = { 16, 32, 64, 76, 128, 256 };
	unsigned long cum = 0;
	int n = 0, i, k, c = 0;

	if (!pool_words)
		return;
	for (k = 0; k < POOL_HASH; k++)
		if (pool_hist[k].n) {
			ent[n][0] = pool_hist[k].v;
			ent[n][1] = pool_hist[k].n;
			n++;
		}
	qsort(ent, n, sizeof ent[0], pool_cmp);

	fprintf(stderr, "fgl: pool %lu words (%lu bytes), %lu distinct values\n",
		pool_words, 4 * pool_words, pool_distinct);
	for (i = 0; i < 12 && i < n; i++)
		fprintf(stderr, "fgl:   %08lx  x%-6lu %5.2f%% of pool\n",
			ent[i][0], ent[i][1], 100.0 * ent[i][1] / pool_words);

	fprintf(stderr, "fgl: pool coverage by hottest N distinct:");
	for (i = 0; i < n; i++) {
		cum += ent[i][1];
		while (c < (int)(sizeof cut / sizeof cut[0]) &&
		       i + 1 == cut[c]) {
			fprintf(stderr, " %d=%.1f%%", cut[c],
				100.0 * cum / pool_words);
			c++;
		}
	}
	fprintf(stderr, " all=%lu words\n", pool_words);
}

static void slot_stats_dump(void)
{
	unsigned long n = slot_fill + slot_plain;

	if (!n)
		return;
	fprintf(stderr, "fgl: slots %lu filled %lu nop (%.1f%% filled)"
		" | empty-window %lu wall %lu no-candidate %lu\n",
		slot_fill, slot_plain, 100.0 * slot_fill / n,
		slot_why_floor, slot_why_wall, slot_why_dep);
	{
		unsigned long tb = 0;
		int t;

		for (t = 0; t < FGL_TAG_N; t++)
			tb += fgl_tag_bytes[t];
		if (tb) {
			fprintf(stderr, "fgl: emitted bytes %lu by tag:", tb);
			for (t = 0; t < FGL_TAG_N; t++)
				if (fgl_tag_bytes[t])
					fprintf(stderr, " %s=%.1f%%",
						fgl_tag_name[t],
						100.0 * fgl_tag_bytes[t] / tb);
			fprintf(stderr, "\n");
		}
	}
	/* Both passes decide alike, so a site counts twice: halved here. */
	if (layout_fall_n || layout_share_n)
		fprintf(stderr, "fgl: layout: %lu fallthroughs direct, %lu exits "
			"shared\n", layout_fall_n / 2, layout_share_n / 2);
	if (layout_bt_n || layout_tlive_n)
		fprintf(stderr, "fgl: layout: %lu taken arms a bare bt (%lu with "
			"no fall arm), %lu T kept live\n", layout_bt_n,
			layout_bt_fall_n, layout_tlive_n);
	if (peep_hoist_n || peep_fwd_n || peep_cse_n)
		fprintf(stderr, "fgl: peep sites: loads hoisted %lu, store->load"
			" forwarded %lu, literals reused %lu\n",
			peep_hoist_n / 2, peep_fwd_n / 2, peep_cse_n / 2);
	if (konst_n)
		fprintf(stderr, "fgl: pooled consts %lu, to r0 %lu (%.1f%%)"
			" | link sites %lu\n", konst_n, konst_r0,
			100.0 * konst_r0 / konst_n, link_sites);
	if (addr_n)
		fprintf(stderr, "fgl: addr %lu: pinned %.1f%% masked %.1f%%"
			" | disp 0=%.1f%% 4..60=%.1f%% i8=%.1f%% wide=%.1f%%\n",
			addr_n, 100.0 * addr_pin / addr_n,
			100.0 * addr_mask / addr_n, 100.0 * addr_d0 / addr_n,
			100.0 * addr_disp / addr_n, 100.0 * addr_i8 / addr_n,
			100.0 * addr_wide / addr_n);
	pool_dump();
	fprintf(stderr, "fgl: block terminator: COND=%lu CAPTURE=%lu JUMP=%lu other=%lu none=%lu\n",
		slot_xfer[IR_COND & 15], slot_xfer[IR_CAPTURE & 15],
		slot_xfer[IR_JUMP & 15],
		slot_xfer[IR_STOP & 15], slot_xfer[15]);
	fprintf(stderr, "fgl: wall by opcode class:");
	for (n = 0; n < 16; n++)
		if (slot_wall_hi[n])
			fprintf(stderr, " %lx=%lu", n, slot_wall_hi[n]);
	fprintf(stderr, "\n");
}

static int slot_stats_on(void)
{
	static int on = -1;

	if (on < 0) {
		on = getenv("FGL_SLOT_STATS") != NULL;
		if (on)
			atexit(slot_stats_dump);
	}
	return on;
}

/* A `timeout`-killed run never reaches atexit, and every bench run here ends
 * that way, so the line is also printed as it goes. */
static void slot_stats_tick(void)
{
	unsigned long n = slot_fill + slot_plain;

	if (n % 20000 == 0)
		slot_stats_dump();
}

/* Set once the stubs are in the arena; until then emit_invalidate has nothing
 * to call and falls back to the inline sequence.  FGL_NO_INV_STUB=1 keeps it
 * inline permanently, which is the bisection switch for this change. */
int fgl_inv_stubs_ready;

static int inv_stub_on(void)
{
	static int on = -1;

	if (on < 0)
		on = getenv("FGL_NO_INV_STUB") == NULL;
	return on && fgl_inv_stubs_ready;
}

static int slot_off(void)
{
	static int off = -1;

	if (off < 0) {
		off = getenv("FGL_NO_SLOT") != NULL;
		slot_debug = getenv("FGL_SLOT_DEBUG") != NULL;
	}
	return off;
}

/* ---------------------------------------------------------------- */
/* Peepholes over the emitted words                                  */
/* ---------------------------------------------------------------- */

/* THREE PEEPHOLES, ALL ON THE WORD STREAM, ALL BEHIND ONE SWITCH.
 *
 * The pipeline model (sh4_pipe.c) put 14% of the emitted code's cycles in
 * load-use stalls: a load followed at once by the instruction that needs it
 * costs a bubble the SH-4 cannot fill, and the emitter writes that shape on
 * nearly every node -- `operand` fetches from the state block and the ALU
 * word follows it.  Three shapes stood out, per thousand instructions:
 *
 *   116  a load used by the very next word
 *    31  a guest register loaded back from the state block within a few
 *        words of being stored there, from a register still holding it
 *    13  a pool literal loaded again within eight loads of the last time
 *
 * All three are decided from the words already emitted, with the same
 * decoder and the same dependency test as the delay-slot filler, and under
 * the same barrier: nothing below `slot_floor` is looked at, so no label,
 * data word, link site or delay slot is ever crossed.  A word the decoder
 * does not know is a wall.  Both emission passes see the same words above
 * the floor, so both reach the same decisions and the same size.
 *
 * FGL_NO_PEEP=1 turns all three off, exactly as before they existed; any
 * of the letters h, f, c turns off just the hoist, the store forwarding or
 * the literal reuse, for bisection. */
#ifndef PEEP_DEPTH
#define PEEP_DEPTH 12
#endif

#define PEEP_HOIST 1
#define PEEP_FWD   2
#define PEEP_CSE   4

static int peep_off(int which)
{
	static int off = -1;

	if (off < 0) {
		const char *v = getenv("FGL_NO_PEEP");

		off = 0;
		if (v) {
			if (strchr(v, 'h')) off |= PEEP_HOIST;
			if (strchr(v, 'f')) off |= PEEP_FWD;
			if (strchr(v, 'c')) off |= PEEP_CSE;
			if (!off)
				off = PEEP_HOIST | PEEP_FWD | PEEP_CSE;
		}
	}
	return off & which;
}

/* Exchange adjacent words j and j+1: the words, their tags, and any literal
 * site that names either.  Nothing else records a word index above the
 * floor -- branch sites and labels are all below it. */
static void swap_words(fgl_emitter *e, int j)
{
	uint16_t a = word_at(e, j), b = word_at(e, j + 1);
	int f, h = here(e);

	set_word_at(e, j, b);
	set_word_at(e, j + 1, a);
	if (e->cg.tagp) {
		uint8_t *t = e->cg.tagp - (h - j), x = t[0];

		t[0] = t[1];
		t[1] = x;
	}
	for (f = 0; f < e->n_fix; f++) {
		if (e->fix[f].at == j)
			e->fix[f].at = j + 1;
		else if (e->fix[f].at == j + 1)
			e->fix[f].at = j;
	}
}

/* A LOAD WHOSE RESULT THE NEXT WORD NEEDS IS MOVED UP, past words it
 * commutes with, until two instructions separate it from its consumer --
 * the load latency.  Called at every node boundary over the words since the
 * last call (and one before, so a load ending one node and used by the
 * first word of the next is seen).  A word swapped down that is itself a
 * load is not allowed to land in front of its own consumer, which would
 * only move the stall.  A word swapped down is never looked at again: the
 * walk resumes past the load's ORIGINAL place, so two loads that each
 * want the other's slot cannot trade places for ever. */
static void peep_hoist(fgl_emitter *e)
{
	int hi = here(e), lo, i;

	if (e->peep_at > hi)
		e->peep_at = hi;
	lo = e->peep_at - 1;
	if (lo < e->slot_floor)
		lo = e->slot_floor;
	if (lo < 0)
		lo = 0;
	e->peep_at = hi;
	if (peep_off(PEEP_HOIST) || e->cg.overflow)
		return;

	for (i = lo; i + 1 < hi; i++) {
		slot_info L, C, X;
		int c, gap, want, j, at = i;

		slot_decode(word_at(e, i), &L);
		if (L.unknown || L.mem == SM_NONE || L.mem_wr ||
		    !(L.wr & 0xffffu))
			continue;
		for (c = i + 1; c < hi && c <= i + 2; c++) {
			slot_decode(word_at(e, c), &C);
			if (C.unknown || (C.rd & L.wr))
				break;
		}
		if (c >= hi || c > i + 2 || C.unknown || !(C.rd & L.wr))
			continue;
		gap = c - i - 1;
		for (want = 2 - gap; want > 0; want--) {
			j = at - 1;
			if (j < lo)
				break;
			slot_decode(word_at(e, j), &X);
			if (X.unknown || !slot_independent(&X, &L))
				break;
			/* X lands at `at`; the word after it is at+1. */
			if (X.mem != SM_NONE && !X.mem_wr && at + 1 < hi) {
				slot_decode(word_at(e, at + 1), &C);
				if (C.unknown || (C.rd & X.wr))
					break;
			}
			swap_words(e, j);
			at = j;
			peep_hoist_n++;
		}
	}
}

/* A GUEST REGISTER READ BACK FROM THE STATE BLOCK JUST AFTER BEING STORED
 * THERE is copied from the register it was stored from -- a `mov`, or
 * nothing when it is the same one -- instead of a load.  The words between
 * must all decode, none may write the source, and none may be a store the
 * decoder cannot place (an r0-indexed or sub-word state store could be this
 * slot).  Only longword stores are forwarded: a `mov.b` to the slot is not
 * the slot's value. */
static int peep_forward(fgl_emitter *e, unsigned g, int rn)
{
	int k, lo = here(e) - PEEP_DEPTH, off = (int)GUEST_AT(g) * 4;
	uint32_t clob = 0;

	if (peep_off(PEEP_FWD) || e->cg.overflow)
		return 0;
	if (lo < e->slot_floor)
		lo = e->slot_floor;
	for (k = here(e) - 1; k >= lo; k--) {
		uint16_t w = word_at(e, k);
		slot_info s;
		int src;

		slot_decode(w, &s);
		if (s.unknown)
			return 0;
		if (s.mem == SM_STATE && s.mem_wr) {
			if (s.mem_off < 0)
				return 0;
			if (s.mem_off == off) {
				if ((w >> 12) == 0x1)
					src = (int)((w >> 4) & 15);
				else if ((w & 0xff00u) == 0xc200u)
					src = 0;
				else
					return 0;
				if (clob & (1u << src))
					return 0;
				if (src != rn)
					sh4_emit_mov_reg(&e->cg, src, rn);
				peep_fwd_n++;
				return 1;
			}
		}
		clob |= s.wr;
	}
	return 0;
}

/* A LITERAL LOADED A FEW WORDS AGO INTO A REGISTER NOTHING HAS WRITTEN
 * SINCE is copied from that register instead of loaded again. */
static int peep_const(fgl_emitter *e, uint32_t v, int rn)
{
	int k, lo = here(e) - PEEP_DEPTH;
	uint32_t clob = 0;

	if (peep_off(PEEP_CSE) || e->cg.overflow)
		return 0;
	if (lo < e->slot_floor)
		lo = e->slot_floor;
	for (k = here(e) - 1; k >= lo; k--) {
		uint16_t w = word_at(e, k);
		slot_info s;

		slot_decode(w, &s);
		if (s.unknown)
			return 0;
		if ((w >> 12) == 0xd) {
			int f, dst = (w >> 8) & 15;

			for (f = 0; f < e->n_fix; f++)
				if (e->fix[f].at == k)
					break;
			if (f < e->n_fix && e->fix[f].value == v &&
			    !(clob & (1u << dst))) {
				if (dst != rn)
					sh4_emit_mov_reg(&e->cg, dst, rn);
				peep_cse_n++;
				return 1;
			}
		}
		clob |= s.wr;
	}
	return 0;
}

static int emit_delayed(fgl_emitter *e, uint16_t branch, uint32_t reads)
{
	int b = here(e), k, i, lo;
	int why = 0;                            /* 0 floor, 1 wall, 2 no-candidate */
	slot_info si[SLOT_DEPTH];               /* si[k - lo] is word k */

	/* A lift needs two bytes of room, a `nop` four: the exact-size second
	 * pass must reach the same decision as the first, so room is not a
	 * criterion here.  `sh4_word` flags a full buffer on its own. */
	if (e->cg.overflow)
		goto plain;
	/* FGL_NO_SLOT=1: every slot a `nop`, exactly as before the filler
	 * existed.  FGL_SLOT_DEBUG=1: one line per lift.  Bisection switches. */
	if (slot_off())
		goto plain;

	lo = b - SLOT_DEPTH;
	if (lo < e->slot_floor)
		lo = e->slot_floor;
	if (lo < 0)
		lo = 0;

	for (k = lo; k < b; k++)
		slot_decode(word_at(e, k), &si[k - lo]);

	for (k = b - 1; k >= lo; k--) {
		const slot_info *c = &si[k - lo];
		uint16_t a = word_at(e, k);
		int ok = 1;

		if (c->unknown) {
			why = 1;
			if (slot_stats_on())
				slot_wall_hi[a >> 12]++;
			break;                  /* a wall: nothing behind it */
		}
		if (!c->legal || (c->wr & reads) || a == 0x0009) {
			why = 2;
			continue;               /* stays put; must commute */
		}
		for (i = k + 1; i < b && ok; i++)
			ok = slot_independent(c, &si[i - lo]);
		if (!ok) {
			why = 2;
			continue;
		}

		/* Lift: the words after A move down one, the branch takes
		 * the vacated slot before A, and A becomes the slot. */
		if (slot_debug)
			fprintf(stderr, "slot: pc=%08x pass%d b=%d k=%d a=%04x br=%04x left=%ld\n",
				e->charge_pc, e->cg.tagp ? 2 : 1, b, k, a, branch,
				(long)(e->cg.end - e->cg.ptr));
		{
			uint8_t *tags = e->cg.tagp ? e->cg.tagp - (b - k) : NULL;
			uint8_t atag = tags ? tags[0] : 0, t0 = e->cg.tag;
			int f;

			for (i = k; i < b - 1; i++) {
				set_word_at(e, i, word_at(e, i + 1));
				if (tags)
					tags[i - k] = tags[i - k + 1];
			}
			for (f = 0; f < e->n_fix; f++)
				if (e->fix[f].at > k && e->fix[f].at < b)
					e->fix[f].at--;
			set_word_at(e, b - 1, branch);
			if (tags) {
				tags[b - 1 - k] = t0;
				e->cg.tag = atag;
			}
			sh4_word(&e->cg, a);
			e->cg.tag = t0;
			e->slots_filled++;
			if (slot_stats_on())
				slot_fill++, slot_stats_tick();
			/* A SLOT IS A BARRIER.  The next branch must not lift
			 * this instruction a second time: that would leave a
			 * branch in this branch's delay slot. */
			slot_barrier(e);
			return b - 1;
		}
	}

plain:
	if (slot_stats_on()) {
		slot_plain++;
		if (why == 1)
			slot_why_wall++;
		else if (why == 2)
			slot_why_dep++;
		else
			slot_why_floor++;
		slot_stats_tick();
	}
	sh4_word(&e->cg, branch);
	sh4_emit_nop(&e->cg);
	slot_barrier(e);
	return b;
}

static int emit_jsr_slot(fgl_emitter *e, int rs)
{
	/* A crossing into C rebuilds r14 on return (fgl_run.c, fgl_call_out;
	 * shim.S on the DC), so an interrupt raised in there is a zero
	 * budget from here on.  A region edge after this must test it. */
	e->may_exit = 1;
	return emit_delayed(e, SH4_N(0x400b, rs), 1u << rs);
}

/* ---------------------------------------------------------------- */
/* Regions: local branches as host branches                          */
/* ---------------------------------------------------------------- */

void fgl_region_begin(fgl_emitter *e, const uint32_t *pcs, int n)
{
	int i;

	e->region = 1;
	e->n_lbl = n > FGL_MAX_LABELS ? FGL_MAX_LABELS : n;
	for (i = 0; i < e->n_lbl; i++) {
		e->lbl[i].pc = pcs[i];
		e->lbl[i].at = -1;
	}
	e->n_lref = 0;
}

/* TWO SHAPES THE FIRST PASS DECIDES.
 *
 * Both passes emit the same code, except where the second is told it may
 * emit less; the driver accepts a second pass that is smaller, never one that
 * is larger.  Everything the first pass measured can only get closer in the
 * second, because every difference is a shrink of an even number of words:
 * alignment, pool reach and label reach all keep.
 *
 * FGL_HINT_BT: the taken arm of a conditional whose target is a forward
 * label was `bf skip; bra label; nop; skip:`.  Pass 1 records the `bf` site
 * and the label; if the label landed within a `bt`'s reach, pass 2 flips
 * that `bf` into a `bt` aimed at the label and emits nothing for the arm.
 * When the fallthrough is the very next label and nothing (no pool, no
 * exit) follows, the fallthrough arm is not emitted either: the branch
 * falls into it.
 *
 * FGL_HINT_TLIVE is set by the epilogue itself, in pass 1, after decoding
 * every word between the park and the unpark: none unknown, none writing T
 * means the park was for nothing, and pass 2 leaves T where the compare put
 * it. */
void fgl_region_hints(fgl_emitter *e)
{
	int i;

	if (!e->hint_out)
		return;
	for (i = 0; i < e->n_lbl; i++) {
		int l = e->arm_lbl[i];

		if (e->arm_site[i] <= 0 || l < 0 || e->lbl[l].at < 0)
			continue;
		if (sh4_disp8_fits(e->lbl[l].at - e->arm_site[i] - 2))
			e->hint_out[i] |= FGL_HINT_BT;
	}
}

int fgl_region_end(const fgl_emitter *e)
{
	int i, left = 0;

	for (i = 0; i < e->n_lref; i++)
		left += e->lref[i].lbl >= 0;
	return left;
}

int fgl_region_label_at(const fgl_emitter *e, int i)
{
	return (i >= 0 && i < e->n_lbl) ? e->lbl[i].at : -1;
}

/* FGL_REGION_DBG bisection mask: 1 = every local edge tests the budget,
 * 2 = every basic block keeps its dispatcher exit, 4 = IR_JUMP always loads
 * r2. */

/* BLOOM SHAPE.  `FGL_NO_ARMS=1` drops step 3 of PORT.md (the epilogue arms
 * materialising their own target next to their own link site) so this tree
 * emits the conditional shape bloom still has.  Bisection only. */
static int no_arms(void)
{
	static int v = -1;

	if (v < 0)
		v = getenv("FGL_NO_ARMS") != NULL;
	return v;
}

static int region_dbg(void)
{
	static int v = -1;

	if (v < 0) {
		const char *s = getenv("FGL_REGION_DBG");

		v = s ? (int)strtoul(s, NULL, 0) : 0;
	}
	return v;
}

static int region_find(const fgl_emitter *e, uint32_t pc)
{
	int i;

	if (!e->region)
		return -1;
	for (i = 0; i < e->n_lbl; i++)
		if (e->lbl[i].pc == pc)
			return i;
	return -1;
}

/* Patch one forward `bra` at `site` to land at word `at`. */
static void patch_bra12(fgl_emitter *e, int site, int at)
{
	int disp = at - site - 2;

	if (!sh4_disp12_fits(disp))
		e->overflow = 5;
	else
		or_word_at(e, site, (uint16_t)(disp & 0xfff));
}

/* The basic block starting at `pc` begins HERE.  A label is a barrier for
 * the slot filler (see the note above slot_barrier). */
static void region_define(fgl_emitter *e, uint32_t pc)
{
	int l = region_find(e, pc), i;

	if (l < 0)
		return;
	if (e->lbl[l].at >= 0) {
		e->overflow = 7;                /* defined twice */
		return;
	}
	e->lbl[l].at = here(e);
	e->cur_lbl = l;
	for (i = 0; i < e->n_lref; i++) {
		if (e->lref[i].lbl != l)
			continue;
		if (e->lref[i].cond) {
			if (!patch_cond8_to(e, e->lref[i].site, e->lbl[l].at))
				e->overflow = 3;
		} else {
			patch_bra12(e, e->lref[i].site, e->lbl[l].at);
		}
		e->lref[i].lbl = -1;
	}
	slot_barrier(e);
}

/* AN EDGE TO A LABEL, in place of a link site.
 *
 * Backward (the label is behind us): this closes a loop, so it carries the
 * budget test the link arms carry, and for the same reason -- nothing else
 * between here and the loop's next trip round looks at the timeslice.  A
 * budget miss leaves through the dispatcher with the target in r2, exactly
 * as a link arm's `bf out` does.
 *
 * Forward: a plain `bra`, patched when the label is reached.  No test,
 * unless this basic block crossed into C (`may_exit`): a collection
 * requested in there has to be honoured at the next boundary, which is this
 * one.  That is what the dispatcher and the link arms did for it before. */
/* Returns the budget test's `bf` site, for the CALLER to patch at the
 * dispatcher exit, or -1 when there was no test.  Patching it here would
 * land a taken arm's budget miss on the fallthrough arm's code, which is
 * exactly what happened: Spyro spun for ever in a pad-read timeout loop
 * whose timeout branch was the taken arm. */
/* AN IDLE LOOP IS WAITED OUT, NOT GONE ROUND.
 *
 * lightrec's optimiser (`lightrec_detect_idle`) marks a local backward branch
 * whose loop body writes nothing it reads, does no store, no syscall, no
 * COP0/COP2 and touches only directly-backed memory.  Nothing inside such a
 * loop can change its own exit condition: only an interrupt can.  So taking
 * the branch means the guest is going to sit here until the next event, and
 * running the iterations for real is pure emitted work that buys nothing.
 *
 * Every taken edge already ends in `cmp/pl r14` + `bf` -- the budget test
 * that leaves for the dispatcher.  Clamping the budget to zero just ahead of
 * it turns that existing exit into the idle exit, so this costs three
 * instructions and no new control flow.  The exit PC is the loop top, which
 * the arm has already put in place, so the guest resumes the loop after the
 * event is serviced.
 *
 * The clamp is `min(r14, 0)`, not a plain zero, for the same reason lightrec
 * writes `lti`/`movzr` rather than a store: a budget that is already negative
 * has overrun, and the settle charges `(base - r14) * cpo`.  Zeroing it there
 * would charge less than was spent and run the guest clock slow. */
static void emit_idle_clamp(fgl_emitter *e)
{
	sh4_emit_cmppl(&e->cg, FGL_R_CYCLE);    /* T = budget still positive */
	sh4_emit_bf(&e->cg, 0);                 /* no: leave it alone        */
	sh4_emit_mov_imm(&e->cg, 0, FGL_R_CYCLE);
}

static int emit_local_edge(fgl_emitter *e, uint32_t pc, int idle)
{
	int l = region_find(e, pc);
	int backward = e->lbl[l].at >= 0;
	int out = -1, site;

	if (backward || e->may_exit || (region_dbg() & 1)) {
		emit_const(e, pc, FGL_R_EXIT);
		if (idle)
			emit_idle_clamp(e);
		sh4_emit_cmppl(&e->cg, FGL_R_CYCLE);
		out = bf_fwd(e);
	}

	site = bra_fwd(e);
	if (backward) {
		patch_bra12(e, site, e->lbl[l].at);
	} else if (e->n_lref < FGL_MAX_LREFS) {
		e->lref[e->n_lref].site = site;
		e->lref[e->n_lref].lbl = l;
		e->n_lref++;
	} else {
		e->overflow = 6;
	}
	return out;
}


/* HOW MUCH OF A CONDITIONAL EXIT GETS A LINK.
 *
 * 0  `IR_JUMP` only -- one constant successor, one site.
 * 1  plus IR_COND's TAKEN arm; the fallthrough keeps the dispatcher.
 * 2  both arms.  A conditional block then need never reach the dispatcher.
 * 3  the FALLTHROUGH arm only; the mirror of 1, kept because it is the
 *    control that says whether a fault belongs to an arm or to the pair.
 *
 * Spyro hardware, matched steady rows: 0 is 56.3 ms/f, 1 is 47.9, 2 is 43.5,
 * and every millisecond of that came out of the emitted-code bucket.
 *
 * MODE 2 IS WHY `curr_pc` HAD TO BE PUBLISHED AT ALL.  With one site a block still
 * leaves through the dispatcher on its other arm; with two it stops visiting
 * the dispatcher at all, and everything the dispatcher did on the way in has
 * to be done at the site instead.  Adding a mode here means asking what else
 * that entry path was providing.
 *
 * A BISECT MODE MUST DIFFER FROM THE THING IT BISECTS.  Mode 3 was gated
 * `>= 2` for the second site and `< 2` for the branch choosing between the
 * arms, so it emitted two sites and left that branch unpatched -- mode 2
 * mirrored, and worse.  Three hardware runs were spent proving "the
 * fallthrough is unlinkable" from it, which is false. */
#ifndef FGL_LINK_COND
#define FGL_LINK_COND 2
#endif

/* Whether link sites are EMITTED at all.  `FGL_NO_LINK` in the environment
 * only stops them being patched -- the site is still there and still calls the
 * stub -- which is the matched pair you want when measuring.  This is the
 * coarser switch: build with -DFGL_LINK=0 and the epilogue is what it was
 * before linking existed, which is the pair you want when bisecting a fault. */
#ifndef FGL_LINK
#define FGL_LINK 1
#endif


/* ONE SELF-PATCHING EDGE.  Twelve bytes, four of them the two instructions the
 * patcher overwrites and six of them data it may need; see the long comment
 * in `fgl_emit` for what each patch turns this into.
 *
 * The length is deliberately a multiple of four, so that a second site placed
 * immediately after a first is still longword aligned and its pad costs
 * nothing.  Conditional linking emits two of these back to back. */
/* PUBLISH THE GUEST PC, THE WAY THE DISPATCHER DOES.
 *
 * `_fgl_dispatch_loop` stores r2 into `curr_pc` as its FIRST act, before the
 * budget test and before the table lookup, on every single block entry.  A
 * linked edge does not pass through it, so a linked edge does not publish --
 * and while a chain is running, `curr_pc` names whichever block last went the
 * long way round.
 *
 * ONE SITE HID THIS AND TWO SITES DID NOT.  A conditional with a single site
 * still leaves through the dispatcher on its other arm, so the value goes
 * stale for one edge and is put right again almost immediately.  Give the
 * block a site on BOTH arms and it need never reach the dispatcher again
 * until its timeslice runs out -- `curr_pc` then sits frozen across a whole
 * budget's worth of guest execution, and everything that reads it while a
 * chain is running reads a lie.  That is measured, not argued: with both arms
 * linked the game boots when the stub returns 0 and sends the edge round the
 * dispatcher, and faults when it returns the target and the edge goes direct.
 *
 * Two instructions, because `mov.l @(disp,GBR)` can only name r0 -- and r0 is
 * FGL_R_XFER, which the site below reloads on its very next instruction, so
 * there is nothing to preserve.  Both are `mov`, so the T bit the arm branch
 * above just consumed is not disturbed either.
 *
 * IT SITS OUTSIDE THE PATCHED LONGWORD, deliberately.  The patcher rewrites
 * site+0..3 and nothing else, so putting the publish ahead of the site keeps
 * it on the path for the life of the edge, patched or not.  It is also four
 * bytes, which leaves the site's alignment exactly as it was. */
static void emit_const(fgl_emitter *e, uint32_t v, int rn);

/* THE SAME, BUT FROM A CONSTANT INSTEAD OF FROM r2.
 *
 * `find_block_from_lut` wants ANY address inside the block -- it tests
 * `pc <= addr < pc + nb_ops * 4` -- and the node's own guest address satisfies
 * that by construction, at compile time, whatever the exit register happens to
 * be holding.  r2 does not: an exception node (IR_RFE, and the syscall/break
 * paths) sets it to a guest vector mid-block, so a node emitted after one of
 * those publishes the vector and sends the lookup to the wrong block or to
 * none.  Same two instructions, one pool word, and no dependence on the
 * register protocol at all. */
static void emit_publish_pc(fgl_emitter *e, uint32_t pc)
{
	emit_const(e, pc, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_CURR_PC);
}

/* WHERE `curr_pc` HAS TO BE PUBLISHED, AND WHY IT IS NOT EVERY LINK SITE.
 *
 * It used to be two instructions on every site.  That is 21.4M of them in the
 * Spyro savestate scene, 3.9% of everything executed, and the dispatcher takes
 * the exit PC from r2 -- it never reads `curr_pc` on the ordinary path, and it
 * rewrites `curr_pc` from r2 itself on every trip round the loop.
 *
 * BUT IT IS NOT DEAD, AND THE ONE READER IS THE REASON.
 * `lightrec_rw_generic_cb` (lightrec.c) does
 *
 *      lightrec_find_block_from_lut(state->block_cache, arg >> 16,
 *                                   state->curr_pc)
 *
 * to reach the opcode it is about to rewrite.  That call comes from INSIDE a
 * block, and with linking a chain runs many blocks without the dispatcher, so
 * the last `curr_pc` the dispatcher wrote names a block several hops back --
 * the lookup then finds the wrong block, or none, and exits SEGFAULT.  It is
 * rare because the generic path is the re-optimisation fallback, which is
 * exactly what makes it dangerous: the scene runs cycle-identical with no
 * publish anywhere, so testing does not find it.
 *
 * So the publish moves to the ONE node that can reach that reader, where it
 * joins a full register flush and a call and costs nothing next to them,
 * instead of riding on every block-to-block edge.
 *
 * A CLEAN RUN AFTER DELETING IT OUTRIGHT IS NOT EVIDENCE IT IS DEAD.  The
 * generic path is the re-optimisation fallback and barely fires; removing the
 * publish altogether also runs cycle-identical for a long while. */

static void emit_link_site(fgl_emitter *e, uint16_t slot, uint32_t target)
{
	uint8_t tag0 = e->cg.tag;

	e->cg.tag = FGL_TAG_LINK;
	if (e->cg.tagp && slot_stats_on())
		link_sites++;

	/* THE SITE IS NOT ALIGNED; THE LITERAL IS.  `mov.l @(1,PC),R0` reads
	 * `((site + 4) & ~3) + 4`: site+8 from a 4-aligned site, site+6 from a
	 * 2-aligned one.  So a 2-aligned site needs no pad in front of it and
	 * no pad word inside it -- it is one word shorter, not one longer.
	 * The patcher (fgl_run.c, fgl_link_resolve) derives the literal's
	 * place from the site address the same way. */

	/* EXCEPT ON REAL HARDWARE, WHERE THE SITE MUST BE 4-ALIGNED.
	 *
	 * The patch replaces the site's two instruction words while a block
	 * may be EXECUTING them -- the recompiler thread tears links down
	 * from `fgl_unlink_*` while emitted code runs.  A 4-aligned site is
	 * one longword store and the fetcher cannot catch it half-done.  A
	 * 2-aligned one takes two halfword stores, and the window between
	 * them pairs the old first word with the new second word: the old
	 * `mov.l @(FGL_AT_LINK,gbr),r0` leaves the STUB address in r0 and the
	 * new `jmp @r0` jumps to it without setting PR, so the stub derives
	 * its site from a stale PR and patches whatever that names.
	 *
	 * The interpreter tree can ignore this -- its patch is a memcpy and
	 * nothing executes concurrently -- which is why the layout above was
	 * free to save a word there.  Here it costs one `nop` in half the
	 * sites and buys an atomic patch, which is not a trade.
	 *
	 * Costed against the alternative: keeping 2-aligned sites and only
	 * ever patching them near, which is the one form that fits in a
	 * single aligned store at site+2.  That refuses the far form, and
	 * far is 71% of edges in the measured stream -- so it would leave a
	 * third of all edges paying a C call forever. */
	if (SH4_LINK_SITE_ALIGN && ((e->base + fgl_size(e)) & 2))
		sh4_word(&e->cg, 0x0009u);             /* nop */
	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_LINK);
	/* THE SLOT IS FILLED LIKE ANY OTHER, AND BOTH PATCHES KEEP IT.  The
	 * slot instruction runs before the transfer whatever the site has
	 * become: `jsr`'s slot while unpatched, `jmp`'s slot after the far
	 * patch (same word, same place), and the near patch COPIES it into
	 * the `bra`'s slot at site+2 (fgl_run.c, fgl_link_resolve).  It never
	 * writes r0 -- `emit_delayed` refuses that -- so `mov.l @(1,PC),r0`
	 * ahead of it in the far form is safe.  What can land here is what
	 * precedes the site: the flush store of a single-successor block.
	 *
	 * A conditional's two arms sit behind a branch and a label, so nothing
	 * can be lifted into them; instead the epilogue HOISTS the arm branch
	 * above the last word before it and hands that word to both sites as
	 * `slot`, which executes it once on either path.  Two nops become one
	 * duplicate. */
	if (slot) {
		sh4_emit_jsr(&e->cg, FGL_R_XFER);
		sh4_word(&e->cg, slot);
		e->slots_filled++;
	} else {
		emit_delayed(e, SH4_N(0x400b, FGL_R_XFER), 1u << FGL_R_XFER);
	}

	/* THE LITERAL HOLDS THE TARGET GUEST PC UNTIL THE FAR PATCH REPLACES
	 * IT WITH A HOST ADDRESS.  The stub reads it from here, so a site no
	 * longer needs r2 loaded ahead of it -- which was a pool load on every
	 * linked edge, executed for ever after the edge had been patched into
	 * a `bra` that never looks at r2.  Unlinking writes the guest pc back
	 * (fgl_run.c, fgl_link_restore).  Builds that keep a `bf out` past the
	 * site still load r2, because that path lands in the dispatcher. */
	if ((e->base + fgl_size(e)) & 2)
		sh4_word(&e->cg, 0x0000u);     /* pad, so the literal aligns */
	sh4_word(&e->cg, (uint16_t)(target & 0xffffu));
	sh4_word(&e->cg, (uint16_t)(target >> 16));

	e->cg.tag = tag0;
	slot_barrier(e);                       /* data: nothing lifts past it */
}

/* ---------------------------------------------------------------- */
/* Constants                                                         */
/* ---------------------------------------------------------------- */

/* Two tiers, and the cheap one is not an optimisation so much as the common
 * case: MIPS immediates are small far more often than not, and `mov #imm,Rn`
 * is one word with no pool entry and no load. */
static void emit_const(fgl_emitter *e, uint32_t v, int rn)
{
	int32_t s = (int32_t)v;

	if (s >= -128 && s <= 127) {
		sh4_emit_mov_imm(&e->cg, (int)s, rn);
		return;
	}

	if (peep_const(e, v, rn))
		return;
	if (e->n_fix >= FGL_MAX_LITERALS) {
		e->overflow = 5;
		return;
	}
	if (e->cg.tagp && slot_stats_on()) {
		konst_n++;
		if (rn == FGL_R_XFER)
			konst_r0++;
	}
	e->fix[e->n_fix].value = v;
	e->fix[e->n_fix].at = here(e);
	e->n_fix++;

	/* Displacement 0 for now; the pool pass ORs the real one in. */
	sh4_emit_mov_l_load_pc(&e->cg, 0, rn);
}

/* Place the literal pool and resolve every site that wants one.
 *
 * `mov.l @(disp,PC)` reads from (PC + 4) rounded down to a longword boundary,
 * so the pool must be longword aligned and every displacement computed from
 * the rounded address rather than from the site itself. */
static void emit_pool(fgl_emitter *e)
{
	uint32_t values[FGL_MAX_LITERALS];
	int n_values = 0;
	uint32_t pool_at;
	int i, k;

	if (!e->n_fix)
		return;

	if ((e->base + fgl_size(e)) & 2)
		sh4_emit_nop(&e->cg);           /* align the pool */
	pool_at = e->base + fgl_size(e);

	for (i = 0; i < e->n_fix; i++) {
		for (k = 0; k < n_values; k++)
			if (values[k] == e->fix[i].value)
				break;
		if (k == n_values)
			values[n_values++] = e->fix[i].value;
	}

	e->cg.tag = FGL_TAG_POOL;
	for (k = 0; k < n_values; k++) {
		if (e->cg.tagp && slot_stats_on())
			pool_seen(values[k]);
		sh4_word(&e->cg, (uint16_t)(values[k] & 0xffff));
		sh4_word(&e->cg, (uint16_t)(values[k] >> 16));
	}
	e->pool_bytes += 4u * (uint32_t)n_values;

	for (i = 0; i < e->n_fix; i++) {
		uint32_t site = e->base + 2u * (uint32_t)e->fix[i].at;
		uint32_t ref = (site + 4) & ~3u;
		uint32_t disp;

		for (k = 0; k < n_values; k++)
			if (values[k] == e->fix[i].value)
				break;

		disp = (pool_at + 4u * (uint32_t)k - ref) / 4u;
		if (disp > 255) {
			e->overflow = 2;        /* pool out of reach */
			return;
		}
		or_word_at(e, e->fix[i].at, (uint16_t)disp);
	}

	e->n_fix = 0;
	slot_barrier(e);                        /* data */
}

/* A LITERAL POOL THAT DOES NOT HAVE TO SIT AT THE END.
 *
 * `mov.l @(disp,PC)` reaches 1020 bytes forward. A block of 32 guest
 * instructions that lower to twenty SH-4 each -- which the unaligned four do
 * -- is well past that, so a pool parked after the code is unreachable from
 * the top of the block and every site in it overflows.
 *
 * Rejecting the block is not an option: THERE IS NO FALLBACK PATH, so a block
 * fgl declines is a game that does not run. Instead the pool is flushed mid
 * block, jumped over, and started again. Cost is two instructions per flush,
 * paid only by blocks long enough to need one.
 *
 * The trigger is the OLDEST outstanding site, not the newest: that is the one
 * whose reach runs out first. */
static void emit_pool(fgl_emitter *e);

static void maybe_flush_pool(fgl_emitter *e)
{
	int over;

	if (!e->n_fix)
		return;

	/* TWO TRIGGERS, AND THE SECOND IS NOT A SAFETY MARGIN.
	 *
	 * Distance first: 700 rather than 1020, because the pool itself, and
	 * whatever the next node emits before the flush actually happens, both
	 * sit inside the gap.
	 *
	 * But reach is not the only thing that runs out.  `fix[]` holds one
	 * entry per SITE, not per distinct value, and a service call costs
	 * three sites in five instructions -- so a run of I/O or GTE nodes
	 * fills the table long before it has travelled 700 bytes.  That is
	 * what refused the BIOS block at 0xbfc05460: 36 nodes, no reach
	 * problem, and `emit_const` out of fixup slots.  A full table is a
	 * reason to flush, exactly like a full reach; it was only ever an
	 * overflow because nothing here looked at it.
	 *
	 * The headroom is for the widest single node, since a flush can only
	 * happen BETWEEN nodes -- an unaligned store setting up a shim call is
	 * the worst of them and is nowhere near sixteen. */
	if (2 * (here(e) - e->fix[0].at) < 700 &&
	    e->n_fix < FGL_MAX_LITERALS - 16)
		return;

	over = bra_fwd(e);
	emit_pool(e);
	patch_fwd12(e, over);
	e->pool_flushes++;
}

/* ---------------------------------------------------------------- */
/* State-block traffic                                               */
/* ---------------------------------------------------------------- */

/* Guest register `g` into host register `rn`, and back. Two instructions,
 * not one, whenever `rn` is not r0 -- the GBR displacement form has no other
 * destination. This is the whole reason r0 is reserved. */
/* Guest register `g` into host register `rn`, and back.
 *
 * On the GBR path this is TWO instructions and not one whenever `rn` is not
 * r0, because the GBR displacement form has no other destination -- which is
 * the whole reason r0 is reserved, and 27.9% of everything the recompiler
 * executes.  $0-$15 take the base-register path instead and cost one.  See
 * pins.h. */
static void ld_guest(fgl_emitter *e, unsigned g, int rn)
{
	if (peep_forward(e, g, rn))
		return;
#if FGL_GBASE
	/* One instruction, straight to the destination, and R0 never involved
	 * -- which is the dependency chain as much as the count.  See pins.h. */
	if (GUEST_AT(g) < FGL_GBASE_MAX) {
		sh4_emit_mov_l_load_disp(&e->cg, FGL_R_GBASE, rn,
					 (int)GUEST_AT(g));
		return;
	}
#endif
#if FGL_GBASE2
	/* $16-$31, the other window: $s0-$s7, $t8, $t9, $gp, $fp, $ra and the
	 * two kernel registers.  Same one instruction, same R0 avoided. */
	if (GUEST_AT(g) >= FGL_GBASE2_LO && GUEST_AT(g) < FGL_GBASE2_MAX) {
		sh4_emit_mov_l_load_disp(&e->cg, FGL_R_GBASE2, rn,
					 (int)GUEST_AT(g) - FGL_GBASE2_LO);
		return;
	}
#endif
	sh4_emit_mov_l_load_gbr(&e->cg, (int)GUEST_AT(g));
	if (rn != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, rn);
}

/* WRITING GUEST $ZERO IS A NO-OP, AND A LOAD INTO IT IS NOT.
 *
 * `decode.c` used to drop any load whose destination was $zero, on the
 * reasoning that a node producing nothing produces nothing.  That is true of
 * a register write and false of a memory READ: `lbu $zero,0x1040($v1)` is how
 * a driver drains the SIO receive FIFO -- the value is thrown away and the
 * side effect IS the instruction.  Spyro's memcard code does exactly that,
 * and dropping it hung the transfer and then faulted on a wild pointer.
 *
 * So the node is built like any other and the destination is discarded HERE,
 * in the one place every writeback passes through.  `host_dst` already
 * returns -1 for guest 0, so such a node arrives with `hd < 0` and would
 * otherwise store over the state block's $zero -- which every later read of
 * $zero would then see. */
static void st_guest(fgl_emitter *e, unsigned g, int rn)
{
	if (g == 0)
		return;
#if FGL_GBASE
	if (GUEST_AT(g) < FGL_GBASE_MAX) {
		sh4_emit_mov_l_store_disp(&e->cg, rn, FGL_R_GBASE,
					  (int)GUEST_AT(g));
		return;
	}
#endif
#if FGL_GBASE2
	if (GUEST_AT(g) >= FGL_GBASE2_LO && GUEST_AT(g) < FGL_GBASE2_MAX) {
		sh4_emit_mov_l_store_disp(&e->cg, rn, FGL_R_GBASE2,
					  (int)GUEST_AT(g) - FGL_GBASE2_LO);
		return;
	}
#endif
	if (rn != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, rn, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(g));
}

/* An operand, wherever the allocator left it. `-1` means the state block, and
 * `spare` is where to put it -- which the caller has to have kept free. */
static int operand(fgl_emitter *e, int host, unsigned g, int spare)
{
	if (host >= 0)
		return host;
	ld_guest(e, g, spare);
	return spare;
}

/* ---------------------------------------------------------------- */
/* Addresses                                                         */
/* ---------------------------------------------------------------- */

/* A guest address, `rs + imm`, masked and left in `dst`.
 *
 * THE MASK IS A REGISTER, NOT A CONSTANT. r13 holds it for the life of the
 * block, so every access costs one `and Rm,Rn` instead of materialising
 * 0x1fffffff and reloading it -- which, at one load and store per handful of
 * guest instructions, is the difference the pinning is there to buy.
 *
 * `other` is the emitter's other working register, used only when the
 * displacement is too wide for `add #imm`. The caller picks the pair, because
 * which of r0/r1 is free depends on whether a value is already in flight: a
 * load builds its address in r0, a store builds it in r1 and keeps r0 for the
 * value it is about to write.
 */
static void emit_addr_ex(fgl_emitter *e, const ir_node *p, int dst, int other,
			 int mask)
{
	int32_t s = (int32_t)p->imm;

	if (e->cg.tagp && slot_stats_on()) {
		addr_n++;
		if (p->hs >= 0)                         addr_pin++;
		if (mask && !(p->hint & FGL_H_NO_MASK)) addr_mask++;
		if (s == 0)                             addr_d0++;
		else if (s > 0 && s <= 60 && !(s & 3))  addr_disp++;
		else if (s >= -128 && s <= 127)         addr_i8++;
		else                                    addr_wide++;
	}

	/* A WIDE DISPLACEMENT ON A REGISTER BASE IS TWO INSTRUCTIONS, NOT
	 * THREE: the constant goes straight into `dst` and the base is added
	 * to it, so there is no move of the base and `other` is never touched.
	 * The `add Rm,Rn` that ends it reads only pool registers, which is what
	 * lets the delay-slot filler lift it past the service call's own
	 * constants -- the `mov Rm,Rn; mov.l @(d,PC),R0; add R0,Rn` form ended
	 * on r0 and could never move. */
	if (p->hs >= 0 && p->hs != dst && (s < -128 || s > 127)) {
		emit_const(e, p->imm, dst);
		sh4_emit_add_reg(&e->cg, p->hs, dst);
		s = 0;
	} else if (p->hs >= 0) {
		if (p->hs != dst)
			sh4_emit_mov_reg(&e->cg, p->hs, dst);
	} else {
		ld_guest(e, p->rs, dst);
	}

	if (s) {
		if (s >= -128 && s <= 127) {
			sh4_emit_add_imm(&e->cg, (int)s, dst);
		} else {
			emit_const(e, p->imm, other);
			sh4_emit_add_reg(&e->cg, other, dst);
		}
	}

	/* THE MASK, AND THE ONE PROOF THAT LETS IT GO.
	 *
	 * Every guest address is masked into the window, and it costs one
	 * instruction on every load and every store -- which, at six and
	 * five-and-a-half SH-4 instructions per guest access, is real money on
	 * the two hottest classes there are.
	 *
	 * It comes off only where the optimiser established that the WHOLE
	 * possible range of this address already lands inside one region:
	 * `lightrec_get_constprop_map` requires the minimum and maximum
	 * computed address to agree in their top three bits before it will
	 * name a region at all (constprop.c:745), and NO_MASK is set on top of
	 * that. The raw-word front end never sets it, so the oracle's default
	 * path still masks and this is a strictly-added fast path rather than
	 * a changed one.
	 *
	 * If that proof is ever wrong the failure is a store through a wild
	 * address, which is the worst kind of bug this project can produce --
	 * so it is driven by the flag alone and never by anything inferred
	 * here. */
	if (mask && !(p->hint & FGL_H_NO_MASK)) {
		uint8_t tag0 = e->cg.tag;

		e->cg.tag = FGL_TAG_MASK;
		sh4_emit_and(&e->cg, FGL_R_MASK, dst);
		e->cg.tag = tag0;
	}
}

/* A DIRECT access: the address is dereferenced right here, so it is masked
 * into the window the host map covers. */
static void emit_addr(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	emit_addr_ex(e, p, dst, other, 1);
}

/* A DEVICE access, handed to C -- AND THE MASK MUST NOT BE APPLIED.
 *
 * `lightrec_hw_lb` and its family take the RAW guest address and run their own
 * `kunseg` on it (lightrec.c:1615 onwards), and `kunseg` is not a mask: it
 * tells KSEG1 from everything else by testing `addr >= 0xa0000000`, and that
 * test is exactly what `and 0x1fffffff` destroys.
 *
 * For most addresses masking first is invisible, because the two agree.  It is
 * not invisible for the segment lightrec keys a map on rather than a physical
 * address: the cache control register is reached at 0xfffe0130, `kunseg` makes
 * that 0x5ffe0130, and PSX_MAP_CACHE_CONTROL sits at 0x5ffe0130 (plugin.c:358).
 * Pre-masking gives 0x1ffe0130 instead, no map matches, and the store comes
 * back as a segfault from code that was otherwise perfectly correct.
 *
 * lightrec's own `rec_store_hw_call` does `jit_addi(tmp, rs, imm)` and passes
 * that -- an add and nothing else, which is what this reproduces. */
static void emit_addr_raw(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	emit_addr_ex(e, p, dst, other, 0);
}

/* SELF-MODIFYING CODE: A STORE INTO RAM MUST UNCOMPILE WHAT IT WROTE OVER.
 *
 * The guest writes an instruction, jumps to it, and expects the NEW one to
 * run.  Nothing about the store tells the dispatcher that the block it
 * compiled from those bytes is now a lie, so the store has to say so: it
 * writes NULL into the block table entry covering the word, and the next
 * dispatch through that PC misses, calls C, and compiles the new code.
 *
 * lightrec emitted exactly this inline on every invalidating store
 * (emitter.c:1600), and fgl not emitting it is what failed S7-SMC on the CPU
 * test -- the guest patched a function, called it, and got the code from
 * before the patch.  There is no cheaper place to put it: an emitted store is
 * a `mov.l` with no call and no exit, so C never sees it happen.
 *
 * THE TABLE INDEX IS THE ADDRESS. One entry per aligned guest word, four
 * bytes each, so the byte offset is `addr & 0x1ffffc` -- the mirrors fold
 * into it for free, exactly as `lut_offset` does for the dispatcher, and
 * masking the low bits is right whether or not the address was masked
 * already (FGL_H_NO_MASK leaves a KSEG address in the register and the bits
 * that survive are the same ones).
 *
 * THE RANGE TEST, and why only the unproven class pays for it. FGL_IO_RAM is
 * a proof that the address lands in RAM, so the entry is always the right one
 * to clear. FGL_IO_DIRECT is only a proof that it lands in SOME directly
 * backed region, which may be the scratchpad or the BIOS -- and clearing a
 * table entry for one of those addresses would uncompile an unrelated block.
 * On bloom's map every non-RAM region sits above 0x1f000000 once masked, and
 * RAM with its mirrors is the low 8 MB, so `(addr & mask) >> 24 == 0` decides
 * it in three instructions and no literal.
 *
 * REGISTERS. This runs AFTER the value has been stored, so r0 and the address
 * register are both dead and may be used freely; the address register is left
 * holding zero, which is what gets written to the table. */
static void emit_invalidate(fgl_emitter *e, const ir_node *p, int addr, int tmp)
{
	int skip = -1;
	uint8_t tag_inv = e->cg.tag;

	if (!ir_store_invalidates(p))
		return;

	/* Tagged apart from the rest of FGL_TAG_MEM: this is the one sequence
	 * that is inlined at every store and would collapse to a call. */
	e->cg.tag = FGL_TAG_INV;

	/* ZERO IS "NO SCRATCH", NOT "REGISTER ZERO".
	 *
	 * `sc[]` is unsigned and uses 0 for a scratch the allocation pass
	 * could not spare -- which is how every other node reads it
	 * (`emit_rw`, and the `p->sc[0] ? p->sc[0] : FGL_R_XFER` in IR_STORE
	 * itself).  Testing `< 0` can never fire, so a store in a block with
	 * no register to spare took r0 as its scratch, and r0 is the transfer
	 * register this sequence is already using:
	 *
	 *      and     addr, r0        ; r0 = the byte offset
	 *      mov     r0, r0          ; "save it in tmp" -- tmp IS r0
	 *      mov.l   @(LUT,gbr), r0  ; r0 = the table base, offset gone
	 *      mov.l   addr, @(r0, r0) ; store at twice the table base
	 *
	 * -- a wild store, fired only under register pressure, which is why
	 * it was data-dependent and why comparing registers never saw it. */
	/* THE WHOLE SEQUENCE AS A CALL.  Six bytes instead of sixteen or
	 * twenty-eight; see FGL_AT_INV_RAM.  The stub wants the guest address
	 * in r1, which is where IR_STORE built it -- the unaligned path builds
	 * it in a scratch instead and pays one `mov` for the move.
	 *
	 * No `may_exit`: the stub is emitted code that never reaches C, so the
	 * cycle pair cannot have moved under it and a region edge after this
	 * does not have to re-test the budget.  That is the difference between
	 * this and emit_jsr_slot, and it is the difference between the call
	 * being a saving and being a wash. */
	if (inv_stub_on()) {
		if (addr != FGL_R_T1)
			sh4_emit_mov_reg(&e->cg, addr, FGL_R_T1);
		/* NOTHING FROM THE STORE MAY ENTER THIS DELAY SLOT.
		 *
		 * The obvious candidate is the store itself -- `mov.l r0,@r1`
		 * writes no register, so the filler is happy to lift it, and
		 * in the slot it runs AFTER the load below has replaced r0
		 * with the stub's address.  The guest then has the stub
		 * pointer written into its memory instead of the value.  It
		 * cost a BIOS that jumped to 0x4f422cb0.
		 *
		 * The slot is worth at most two bytes and the fill rate is
		 * 8.9%; the barrier is not worth being clever about. */
		slot_barrier(e);
		sh4_emit_mov_l_load_gbr(&e->cg,
			p->io == FGL_IO_DIRECT ? (int)FGL_AT_INV_DIRECT
					       : (int)FGL_AT_INV_RAM);
		emit_delayed(e, SH4_N(0x400b, FGL_R_XFER),
			     (1u << FGL_R_XFER) | (1u << FGL_R_T1));
		e->cg.tag = tag_inv;
		return;
	}

	if (tmp <= 0) {
		e->overflow = 9;        /* the allocator and ir.h disagree */
		return;
	}

	if (p->io == FGL_IO_DIRECT) {
		sh4_emit_mov_reg(&e->cg, addr, tmp);
		e->cg.tag = FGL_TAG_MASK;
		sh4_emit_and(&e->cg, FGL_R_MASK, tmp);
		e->cg.tag = FGL_TAG_INV;
		sh4_emit_shlr16(&e->cg, tmp);
		sh4_emit_shlr8(&e->cg, tmp);
		sh4_emit_tst(&e->cg, tmp, tmp);   /* T = (it is RAM) */
		skip = bf_fwd(e);
	}

	/* The index mask comes out of the state block, not the literal pool:
	 * the GBR form is the same one instruction and drags no word into the
	 * instruction stream with it.  See FGL_AT_INV_MASK. */
	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_INV_MASK);
	sh4_emit_and(&e->cg, addr, FGL_R_XFER);         /* r0 = byte offset */
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, tmp);
	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_LUT);
	sh4_emit_mov_imm(&e->cg, 0, addr);              /* the NULL to write */
	sh4_emit_mov_l_store_r0(&e->cg, addr, tmp);     /* mov.l Rn,@(r0,tmp) */

	if (skip >= 0)
		patch_fwd8(e, skip);
	e->cg.tag = tag_inv;
}

/* ---------------------------------------------------------------- */
/* Shifts                                                            */
/* ---------------------------------------------------------------- */

/* How many instructions the constant-shift decomposition would cost. SH-4
 * has 16/8/2/1-bit shifts but only in one direction each, so a shift by 23
 * is five instructions and `mov #imm` + `shld` is two. Pick per amount. */
static int shift_steps(unsigned k)
{
	return (int)(k / 16) + (int)((k % 16) / 8) + (int)((k % 8) / 2) + (int)(k % 2);
}

static void shift_const(fgl_emitter *e, int rn, unsigned k, int dir)
{
	unsigned n;

	if (!k)
		return;

	/* SH_RA has no multi-bit form at all, so it is always the dynamic
	 * one past a single bit. */
	if (dir == SH_RA) {
		if (k == 1) {
			sh4_emit_shar(&e->cg, rn);
			return;
		}
		sh4_emit_mov_imm(&e->cg, -(int)k, FGL_R_T1);
		sh4_emit_shad(&e->cg, FGL_R_T1, rn);
		return;
	}

	if (shift_steps(k) > 2) {
		sh4_emit_mov_imm(&e->cg, dir == SH_LL ? (int)k : -(int)k, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, rn);
		return;
	}

	for (n = k / 16; n--;) dir == SH_LL ? sh4_emit_shll16(&e->cg, rn) : sh4_emit_shlr16(&e->cg, rn);
	for (n = (k % 16) / 8; n--;) dir == SH_LL ? sh4_emit_shll8(&e->cg, rn) : sh4_emit_shlr8(&e->cg, rn);
	for (n = (k % 8) / 2; n--;) dir == SH_LL ? sh4_emit_shll2(&e->cg, rn) : sh4_emit_shlr2(&e->cg, rn);
	for (n = k % 2; n--;) dir == SH_LL ? sh4_emit_shll(&e->cg, rn) : sh4_emit_shlr(&e->cg, rn);
}

/* ---------------------------------------------------------------- */
/* ALU                                                               */
/* ---------------------------------------------------------------- */

/* rd = rs <op> rt on a two-operand machine.
 *
 * The destructive form means the destination has to start out holding rs, so
 * the only awkward case is rd already being rt -- there the move would
 * destroy the other operand, and a working register is used instead. */
static void alu_rr(fgl_emitter *e, int sub, int rd, int rs, int rt)
{
	int w = (rd == rt && rd != rs) ? FGL_R_T1 : rd;

	switch (sub) {
	case ALU_SLT:
	case ALU_SLTU:
		/* No move at all: the comparison reads both operands and the
		 * result is one bit out of T. */
		if (sub == ALU_SLT)
			sh4_emit_cmpgt(&e->cg, rs, rt);   /* T = rt > rs */
		else
			sh4_emit_cmphi(&e->cg, rs, rt);
		sh4_emit_movt(&e->cg, rd);
		return;
	default:
		break;
	}

	if (w != rs)
		sh4_emit_mov_reg(&e->cg, rs, w);

	switch (sub) {
	case ALU_ADD: sh4_emit_add_reg(&e->cg, rt, w); break;
	case ALU_SUB: sh4_emit_sub(&e->cg, rt, w); break;
	case ALU_AND: sh4_emit_and(&e->cg, rt, w); break;
	case ALU_OR:  sh4_emit_or(&e->cg, rt, w); break;
	case ALU_XOR: sh4_emit_xor(&e->cg, rt, w); break;
	case ALU_NOR: sh4_emit_or(&e->cg, rt, w); sh4_emit_not(&e->cg, w, w); break;
	default:      e->unsupported = 1; e->unsupported_op = sub; return;
	}

	if (w != rd)
		sh4_emit_mov_reg(&e->cg, w, rd);
}

/* rd = rs <op> imm. ADD has a one-word immediate form; the rest materialise
 * the constant, which is what the R0-only immediate ALU forms cost us. */
static void alu_ri(fgl_emitter *e, int sub, int rd, int rs, uint32_t imm)
{
	int32_t s = (int32_t)imm;

	if (sub == ALU_ADD && s >= -128 && s <= 127) {
		if (rd != rs)
			sh4_emit_mov_reg(&e->cg, rs, rd);
		if (s)
			sh4_emit_add_imm(&e->cg, (int)s, rd);
		return;
	}

	if (sub == ALU_SLT || sub == ALU_SLTU) {
		emit_const(e, imm, FGL_R_T1);
		if (sub == ALU_SLT)
			sh4_emit_cmpgt(&e->cg, rs, FGL_R_T1);
		else
			sh4_emit_cmphi(&e->cg, rs, FGL_R_T1);
		sh4_emit_movt(&e->cg, rd);
		return;
	}

	emit_const(e, imm, FGL_R_T1);
	alu_rr(e, sub, rd, rs, FGL_R_T1);
}

/* ---------------------------------------------------------------- */
/* COP2 register traffic                                             */
/* ---------------------------------------------------------------- */

/* MFC2/CFC2/MTC2/CTC2/LWC2/SWC2 -- the moves only.  The geometry
 * commands themselves are IR_GTE and are not lowered here.
 *
 * `imm` IS ALREADY THE STATE-BLOCK WORD INDEX of the coprocessor
 * register, resolved by the decoder (`cop2_disp`, decode.c:59).  Data
 * and control are one contiguous 64-entry file at cp2d = word 66 and
 * cp2c = word 98 (fgl_state.h), so the highest index any of these can
 * name is 129 -- byte 516, inside `mov.l @(disp,GBR)`'s 1020 -- and no
 * address is ever computed for a coprocessor register.  Nothing below
 * needs to know which half of the file it is touching.
 *
 * LWC2 AND SWC2 KEEP THE MEMORY DISPLACEMENT IN `imm2`, NOT `imm`
 * (decode.c:419, 428), which is the reverse of every other memory
 * node.  `emit_addr` reads `imm`, so it is handed a node with the
 * fields the way round it expects; getting this wrong would address
 * off a coprocessor register number and the fault would look like a
 * bad base register rather than a swapped field. */
static void emit_cop2_addr(fgl_emitter *e, const ir_node *p, int dst, int other)
{
	ir_node q = *p;

	q.imm = p->imm2;
	emit_addr(e, &q, dst, other);
}

/* ---------------------------------------------------------------- */
/* Exceptions                                                        */
/* ---------------------------------------------------------------- */

#define CP0_AT(r) ((int)(FGL_AT_COP0 + (unsigned)(r)))

/* THE GUEST'S GENERAL EXCEPTION VECTOR, BEV=0.
 *
 * `tools/docs/cpuspecifications.md`, "Exception Vectors": General is
 * 80000080h with BEV clear and BFC00180h with it set, and the same section
 * states the PSX uses only the BEV=0 vectors -- the BIOS ROM does not contain
 * the BEV=1 ones at all.  So the constant is emitted unconditionally and the
 * SR bit 22 test is not.  THAT IS THE CONDITION on this sequence: a guest that
 * sets BEV and then traps goes to the wrong place.
 *
 * The 80000040h "COP0 Break" vector in that same table is the COP0 debug
 * breakpoint unit (BPC/BDA), not the BREAK opcode; BREAK raises excode 09h
 * through the general vector like any other exception.  `psxException`
 * (deps/pcsx_rearmed/libpcsxcore/r3000a.c:113) is the runtime we have to agree
 * with and it makes no distinction either. */
#define GUEST_EXC_VECTOR     0x80000080u
#define GUEST_EXC_VECTOR_BEV 0xbfc00180u

/* Replace a field of a COP0 word with three instructions and no third
 * register.
 *
 * `and #imm` and `or #imm` exist only for r0, so the obvious "load, mask with
 * a materialised constant, or in the new bits" wants a register we do not
 * have and a literal we do not want.  Instead: with `a` in r0 and a copy in
 * r1, and `b` a value whose bits outside the field are don't-care,
 *
 *      x = a ^ b ; y = x & field ; z = a ^ y
 *
 * gives z = (a & ~field) | (b & field).  Bits outside the field xor with zero
 * twice; bits inside come out as b.  Two xors and an `and #imm` -- and it is
 * the `and #imm` that lets the caller be sloppy about b's high bits. */
static void merge_low_field(fgl_emitter *e, int field)
{
	sh4_emit_xor(&e->cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_and_imm(&e->cg, field);
	sh4_emit_xor(&e->cg, FGL_R_T1, FGL_R_XFER);
}

/* The mode stack push an exception performs: SR bits 3:0 move up to 5:2, and
 * 1:0 come in zero -- kernel mode, interrupts off.  Bits 5:4 (the OLD pair)
 * are pushed off the end and lost.
 *
 * `cpuspecifications.md`, cop0r12: bit0 IEc / bit1 KUc are current, bit2 IEp /
 * bit3 KUp previous, bit4 IEo / bit5 KUo old.  The runtime writes exactly
 * `SR = (SR & ~0x3f) | ((SR & 0x0f) << 2)` (r3000a.c:139), and
 * `(SR << 2) & 0x3f` is that same value -- the field mask does the `& 0x0f`,
 * so the shift stands alone. */
static void emit_sr_push(fgl_emitter *e)
{
	sh4_emit_mov_l_load_gbr(&e->cg, CP0_AT(COP0_SR));
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, FGL_R_T1);
	sh4_emit_shll2(&e->cg, FGL_R_XFER);
	merge_low_field(e, 0x3f);
	sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_SR));
}

/* CAUSE for a synchronous exception taken outside a delay slot.
 *
 * `CAUSE = (CAUSE & 0x700) | (code << 2)`, which is `psxException` verbatim
 * (r3000a.c:136, with bdt = 0).  Bits 10:8 are the two software interrupt
 * bits and the hardware one and MUST survive -- a trap that cleared them
 * would lose an interrupt the guest has not looked at yet.  Everything else,
 * BD and BT at 31:30 included, is written zero, which is correct here because
 * SYSCALL and BREAK reach this emitter only outside a delay slot: `is_transfer`
 * counts them as terminators (decode.c:654) and the decoder does not call
 * `decode_transfer` for a delay slot.
 *
 * Three instructions rather than a masked literal: shifting the field down to
 * the bottom brings it inside `and #imm`'s reach, and `or #imm` then supplies
 * the ExcCode -- 0x20 for Sys, 0x24 for Bp, both 8-bit. */
static void emit_cause(fgl_emitter *e, unsigned excode)
{
	sh4_emit_mov_l_load_gbr(&e->cg, CP0_AT(COP0_CAUSE));
	sh4_emit_shlr8(&e->cg, FGL_R_XFER);
	sh4_emit_and_imm(&e->cg, 0x07);
	sh4_emit_shll8(&e->cg, FGL_R_XFER);
	sh4_emit_or_imm(&e->cg, (int)(excode << 2));
	sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_CAUSE));
}

/* ---------------------------------------------------------------- */
/* Multiply and divide                                               */
/* ---------------------------------------------------------------- */

/* Where one half of the pair ends up.  A host register means a move and no
 * host register means a store, like any other destination -- but there are
 * two of them and neither is `p->rd`, so this is not the
 * `st_guest(e, p->rd, rd)` tail every other node ends with. */
static void st_pair(fgl_emitter *e, int host, unsigned g, int src)
{
	if (host < 0) {
		st_guest(e, g, src);
		return;
	}
	if (host != src)
		sh4_emit_mov_reg(&e->cg, src, host);
}

/* THE 32-STEP DIVIDE, UNSIGNED, AND THE ONLY PART OF THIS THAT IS HARDWARE.
 *
 * `div0u` clears M, Q and T; then each `rotcl`/`div1` pair shifts one bit of
 * the dividend into the partial remainder and one quotient bit back in at the
 * bottom, the partial remainder's 33rd bit living in Q.  After 32 pairs the
 * dividend register holds the quotient bar its last bit, which the trailing
 * `rotcl` supplies.
 *
 * The divisor must be non-zero.  A divide by zero here does not fault, it
 * produces a number, and MIPS wants a specific different one -- the caller
 * tests for it.
 *
 * `dividend` is destroyed and becomes the quotient, `rem` is written from
 * scratch and becomes the remainder, `divisor` is only read.
 *
 * THE SEQUENCE IS NON-RESTORING, so a step that subtracts too much is not
 * undone -- it is compensated by the next step adding.  After the last step
 * there is no next one, so the remainder is left one divisor low exactly when
 * that last subtraction failed.  T already IS that answer, because it is also
 * the quotient's last bit.  The quotient comes out right either way, which is
 * why the correction is easy to leave out and hard to notice: only code that
 * reads HI ever sees it.  Neither `bt` nor `add` writes T, so the trailing
 * `rotcl` still gets the bit it came for.
 *
 * 69 words, and there is no shorter form: a loop would have to carry T across
 * the back edge, and every loop-control instruction on this machine writes
 * it. */
static void emit_div_core(fgl_emitter *e, int dividend, int divisor, int rem)
{
	int i, exact;

	sh4_emit_mov_imm(&e->cg, 0, rem);
	sh4_emit_div0u(&e->cg);
	for (i = 0; i < 32; i++) {
		sh4_emit_rotcl(&e->cg, dividend);
		sh4_emit_div1(&e->cg, divisor, rem);
	}

	exact = bt_fwd(e);
	sh4_emit_add_reg(&e->cg, divisor, rem);
	patch_fwd8(e, exact);

	sh4_emit_rotcl(&e->cg, dividend);
}

/* |Rn|, in place, with no second register and no constant. */
static void emit_abs(fgl_emitter *e, int rn)
{
	int pos;

	sh4_emit_cmppz(&e->cg, rn);
	pos = bt_fwd(e);
	sh4_emit_neg(&e->cg, rn, rn);
	patch_fwd8(e, pos);
}

/* Negate `rn` when `sign` is negative: the fixup that turns an unsigned
 * result back into a signed one. */
static void emit_neg_if(fgl_emitter *e, int sign, int rn)
{
	int pos;

	sh4_emit_cmppz(&e->cg, sign);
	pos = bt_fwd(e);
	sh4_emit_neg(&e->cg, rn, rn);
	patch_fwd8(e, pos);
}

/* MULT, MULTU, DIV, DIVU.
 *
 * The multiplies are three instructions.  The divides are a hundred, because
 * SH-4 has no divide and the architecture's own sequence is 32 unrolled
 * steps; see NOTES for the service routine that lifts them out of the call
 * site once fgl has somewhere to put one.
 *
 * The dividend has to be somewhere the core may destroy that is not the
 * allocator's, so it is copied to r1.  The remainder needs a third register,
 * which is the scratch alloc.c already grants this node.  The divisor stays
 * where the allocator left it for DIVU -- the core only reads it -- and is
 * copied to r0 for DIV, which negates it.
 *
 * DIVIDE BY ZERO IS NOT A TRAP ON THIS GUEST.  It has defined results, and
 * they are produced here rather than left to whatever the sequence happens to
 * do:
 *
 *      DIVU, d == 0:   LO = 0xffffffff             r3000a.h:363-364
 *                      HI = n
 *      DIV,  d == 0:   LO = n >= 0 ? 0xffffffff : 1
 *                      HI = n                      r3000a.h:348-350
 *
 * The other special case needs no code at all.  `0x80000000 / -1` is defined
 * as LO = 0x80000000, HI = 0 (r3000a.h:351-353), and that is what falls out:
 * |n| is 0x80000000 and |d| is 1, the core divides them to 0x80000000
 * remainder 0, and the quotient's sign `n ^ d` has its top bit clear so
 * nothing is negated.
 *
 * THE TWO SIGN WORDS GO ON THE STACK, NOT IN THE STATE BLOCK'S TEMP WORD.
 * That word is the parking slot for a deferred load (ir.h, THE LOAD SHADOW),
 * and a divide is allowed to BE the shadow instruction sitting between an
 * IR_LOAD and its IR_TEMP_GET -- writing it here would eat the parked value.
 * Both pushes are past the divide-by-zero branch, so every path through this
 * node pops exactly what it pushed.
 */
static void emit_muldiv(fgl_emitter *e, const ir_node *p)
{
	int dividend = FGL_R_T1;
	int rem = p->sc[0];
	int divisor;
	int to_zero, done;
	int a, b;
	/* A half nothing reads is a half nothing computes. The allocator is
	 * told the same thing (alloc.c) and gives the dead half no register,
	 * so there is nothing holding a stale value for the block's writeback
	 * to publish. */
	int no_lo = (p->hint & FGL_H_NO_LO) != 0;
	int no_hi = (p->hint & FGL_H_NO_HI) != 0;
	int check_zero = !(p->hint & FGL_H_NO_DIV_CHK);

	if (p->sub == MD_MULT || p->sub == MD_MULTU) {
		/* `dmuls.l`/`dmulu.l` leave the whole 64-bit product in
		 * MACH:MACL, MACL the low half -- which is LO, MACH being HI.
		 * `mul.l` would save nothing here and leaves MACH undefined,
		 * so it is only usable where HI is dead; the IR does not say
		 * that, so it is not used. */
		a = operand(e, p->hs, p->rs, FGL_R_T1);
		b = operand(e, p->ht, p->rt, FGL_R_XFER);

		/* `mul.l` computes only the low 32 bits and leaves MACH
		 * undefined, so it is usable exactly when the optimiser has
		 * shown nothing reads HI. That is the whole of what NO_HI buys
		 * here, and it is why the comment above says the IR does not
		 * say that -- now it does. */
		if (no_hi)
			sh4_emit_mul_l(&e->cg, a, b);
		else if (p->sub == MD_MULT)
			sh4_emit_dmuls_l(&e->cg, a, b);
		else
			sh4_emit_dmulu_l(&e->cg, a, b);

		/* Both halves sit in MAC until they are read, so the order is
		 * free and LO goes first to match the reference's. */
		if (!no_lo) {
			a = p->hd >= 0 ? p->hd : FGL_R_XFER;
			sh4_emit_sts_macl(&e->cg, a);
			st_pair(e, p->hd, GUEST_LO, a);
		}

		if (!no_hi) {
			a = p->hx >= 0 ? p->hx : FGL_R_XFER;
			sh4_emit_sts_mach(&e->cg, a);
			st_pair(e, p->hx, GUEST_HI, a);
		}
		return;
	}

	if (p->sub != MD_DIV && p->sub != MD_DIVU) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	if (!rem) {
		e->overflow = 6;
		return;
	}

	/* The dividend is taken first: loading it from the state block goes
	 * through r0, which is where the divisor is about to live. */
	a = operand(e, p->hs, p->rs, FGL_R_T1);
	if (a != dividend)
		sh4_emit_mov_reg(&e->cg, a, dividend);

	if (p->sub == MD_DIVU && p->ht >= 0) {
		divisor = p->ht;                /* read-only: no copy needed */
	} else {
		divisor = FGL_R_XFER;
		b = operand(e, p->ht, p->rt, FGL_R_XFER);
		if (b != divisor)
			sh4_emit_mov_reg(&e->cg, b, divisor);
	}

	/* THE DIVIDE-BY-ZERO PATH, AND WHEN IT IS NOT THERE.
	 *
	 * A zero divisor is defined on the R3000A rather than trapping -- the
	 * quotient is -1 or 1 by the sign of the dividend and the remainder is
	 * the dividend -- so the check is part of the instruction, not a
	 * safety net, and it is emitted unless the optimiser proved the
	 * divisor cannot be zero. `LIGHTREC_NO_DIV_CHECK` is that proof, and
	 * nothing here infers it: an inferred one would remove a defined
	 * result and the wrong answer would be a plausible number.
	 *
	 * Note what is NOT skipped with it: the signed path's sign handling
	 * stays exactly as it was. Only the zero test and its branch go. */
	if (check_zero) {
		sh4_emit_tst(&e->cg, divisor, divisor);
		to_zero = bt_fwd(e);
	} else {
		to_zero = 0;
	}

	if (p->sub == MD_DIVU) {
		emit_div_core(e, dividend, divisor, rem);

		if (check_zero) {
			done = bra_fwd(e);
			patch_fwd8(e, to_zero);
			sh4_emit_mov_reg(&e->cg, dividend, rem);   /* HI = n  */
			sh4_emit_mov_imm(&e->cg, -1, dividend);    /* LO = -1 */
			patch_fwd12(e, done);
		}
	} else {
		int nonneg;

		/* The quotient takes the sign of the operands' exclusive or
		 * and the remainder the sign of the dividend -- C's
		 * truncating division, which is what `n / d` and `n % d` mean
		 * at r3000a.h:354-355, not a floor.  Neither sign survives
		 * the core: both operands are made positive, and `rem` is
		 * built by the core itself. */
		sh4_emit_mov_reg(&e->cg, dividend, rem);
		sh4_emit_xor(&e->cg, divisor, rem);             /* n ^ d */
		sh4_emit_mov_l_store_dec(&e->cg, rem, 15);
		sh4_emit_mov_l_store_dec(&e->cg, dividend, 15);

		emit_abs(e, dividend);
		emit_abs(e, divisor);

		emit_div_core(e, dividend, divisor, rem);

		/* r0 holds the divisor, which the core's own correction was
		 * the last reader of, so the sign words come back into it. */
		sh4_emit_mov_l_load_inc(&e->cg, 15, FGL_R_XFER);   /* n     */
		emit_neg_if(e, FGL_R_XFER, rem);
		sh4_emit_mov_l_load_inc(&e->cg, 15, FGL_R_XFER);   /* n ^ d */
		emit_neg_if(e, FGL_R_XFER, dividend);

		if (check_zero) {
			done = bra_fwd(e);

			patch_fwd8(e, to_zero);
			sh4_emit_mov_reg(&e->cg, dividend, rem);   /* HI = n */
			sh4_emit_mov_imm(&e->cg, -1, dividend);
			sh4_emit_cmppz(&e->cg, rem);
			nonneg = bt_fwd(e);
			sh4_emit_mov_imm(&e->cg, 1, dividend);
			patch_fwd8(e, nonneg);
			patch_fwd12(e, done);
		}
	}

	/* The core computes both halves whatever happens -- a 32-step divide
	 * produces its remainder on the way to its quotient -- so a dead half
	 * saves the store, not the work. */
	if (!no_lo)
		st_pair(e, p->hd, GUEST_LO, dividend);
	if (!no_hi)
		st_pair(e, p->hx, GUEST_HI, rem);
}

/* ---------------------------------------------------------------- */
/* The unaligned four                                                */
/* ---------------------------------------------------------------- */

/* LWL/LWR/SWL/SWR, the only guest accesses DEFINED on an unaligned address.
 *
 * All four work the same way: take the address, split it into the aligned
 * word below it and a byte offset, and use the offset to build a shift and a
 * mask. The shift distances are `8 * offset` and its complement `24 - 8n`,
 * which is why both are computed up front.
 *
 * These need a scratch register from the allocator -- the address survives
 * across the whole sequence, so r0 and r1 alone are not enough. It gives one
 * to LOAD_UN and two to STORE_UN (alloc.c:479-495).
 */

/* r0 = 8 * (address & 3), s0 = address & ~3. */
static void emit_unaligned_setup(fgl_emitter *e, const ir_node *p, int s0)
{
	emit_addr(e, p, FGL_R_XFER, FGL_R_T1);          /* r0 = address     */
	sh4_emit_mov_reg(&e->cg, FGL_R_XFER, s0);
	sh4_emit_and_imm(&e->cg, 3);                    /* r0 = offset      */
	sh4_emit_shll2(&e->cg, FGL_R_XFER);
	sh4_emit_shll(&e->cg, FGL_R_XFER);              /* r0 = 8 * offset  */
	sh4_emit_mov_imm(&e->cg, -4, FGL_R_T1);
	sh4_emit_and(&e->cg, FGL_R_T1, s0);             /* s0 = address & ~3 */
}

/* r1 = 24 - r0, the complementary shift distance. */
static void emit_comp_shift(fgl_emitter *e)
{
	sh4_emit_mov_imm(&e->cg, 24, FGL_R_T1);
	sh4_emit_sub(&e->cg, FGL_R_XFER, FGL_R_T1);
}

static void emit_load_un(fgl_emitter *e, const ir_node *p)
{
	int s0 = p->sc[0];
	int d;

	if (!s0) {
		e->overflow = 7;
		return;
	}

	emit_unaligned_setup(e, p, s0);
	sh4_emit_mov_l_load(&e->cg, s0, s0);            /* s0 = the word */

	if (p->sub == UN_LWL) {
		/* value = word << (24 - 8n), mask = 0x00ffffff >> 8n */
		emit_comp_shift(e);
		sh4_emit_shld(&e->cg, FGL_R_T1, s0);
		emit_const(e, 0x00ffffffu, FGL_R_T1);
		sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	} else {
		/* value = word >> 8n, mask = 0xffffff00 << (24 - 8n) */
		sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, s0);
		emit_comp_shift(e);
		sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
		emit_const(e, 0xffffff00u, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	}

	/* The destination keeps the bytes the load does not cover, so it is
	 * read as well as written -- the one node where that is true.
	 *
	 * Deferred, it is read and NOT written: the merge is built in r0 and
	 * parked, so the shadow instruction still sees the old register. */
	if (p->defer) {
		if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, p->hd, FGL_R_XFER);
		else
			ld_guest(e, p->rd, FGL_R_XFER);
		sh4_emit_and(&e->cg, FGL_R_T1, FGL_R_XFER);
		sh4_emit_or(&e->cg, s0, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		return;
	}

	d = p->hd;
	if (d < 0) {
		ld_guest(e, p->rd, FGL_R_XFER);
		d = FGL_R_XFER;
	}
	sh4_emit_and(&e->cg, FGL_R_T1, d);
	sh4_emit_or(&e->cg, s0, d);
	if (p->hd < 0)
		st_guest(e, p->rd, d);
}

static void emit_store_un(fgl_emitter *e, const ir_node *p)
{
	int s0 = p->sc[0], s1 = p->sc[1];
	int v;

	if (!s0 || !s1) {
		e->overflow = 8;
		return;
	}

	/* The value goes to a register before anything else, because from the
	 * address onwards r0 is spoken for. */
	v = operand(e, p->ht, p->rt, FGL_R_XFER);
	sh4_emit_mov_reg(&e->cg, v, s1);

	emit_unaligned_setup(e, p, s0);

	if (p->sub == UN_SWL) {
		/* keep = word & (0xffffff00 << 8n), value >>= 24 - 8n */
		emit_comp_shift(e);
		sh4_emit_neg(&e->cg, FGL_R_T1, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_T1, s1);
		emit_const(e, 0xffffff00u, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	} else {
		/* keep = word & (0x00ffffff >> (24 - 8n)), value <<= 8n */
		sh4_emit_shld(&e->cg, FGL_R_XFER, s1);
		emit_comp_shift(e);
		sh4_emit_neg(&e->cg, FGL_R_T1, FGL_R_T1);
		sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
		emit_const(e, 0x00ffffffu, FGL_R_T1);
		sh4_emit_shld(&e->cg, FGL_R_XFER, FGL_R_T1);
	}

	sh4_emit_mov_l_load(&e->cg, s0, FGL_R_XFER);    /* the word */
	sh4_emit_and(&e->cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_or(&e->cg, s1, FGL_R_XFER);
	sh4_emit_mov_l_store(&e->cg, FGL_R_XFER, s0);

	/* SWL and SWR write inside ONE aligned word, which is exactly what one
	 * table entry covers, so the aligned address this path already built
	 * is the right thing to invalidate and no third scratch is needed --
	 * both of these are dead the instant the merge is stored. */
	emit_invalidate(e, p, s0, s1);
}

/* ---------------------------------------------------------------- */
/* The block                                                         */
/* ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- */
/* Hardware registers                                                */
/* ---------------------------------------------------------------- */

/* WHEN A LOAD OR A STORE IS NOT MEMORY.
 *
 * lightrec's optimiser tags every access with the region it proved, and fgl
 * lowers most of them to the two-instruction masked access that is the whole
 * point of having a region analysis.  `FGL_IO_HW` is the tag that says the
 * proof went the other way: this address is a device register, and reading it
 * has side effects that a `mov.l` does not produce.
 *
 * WHY THIS IS A CALL AND NOT A FAULT.  bloop reaches its device model by
 * leaving the address unmapped and catching the exception, which costs no
 * instructions at the site at all.  Measured on hardware that path is 170-230
 * cycles against 27 for a call, because the cost is in the exception entry and
 * not in anything that strides or caches; see build/docs/CLAUDE.md 3c, which
 * also records what would flip the decision back.
 *
 * WHAT THE CALLEE ALREADY DOES, so that the call site does not do it twice:
 * `lightrec_hw_lb` and its family `kunseg()` the address themselves, and they
 * return a value ALREADY EXTENDED to 32 bits -- sign for lb/lh, zero for
 * lbu/lhu (lightrec.c, the `(u32)(s32)(s8)` and `(u8)` casts).  So there is no
 * extension here, and a call site that added one would sign-extend an already
 * sign-extended byte, which is invisible until the byte has its top bit set.
 *
 * WHAT NEEDS NO FLUSH, which is the reason this path is cheap.  These take
 * their address and their value as ARGUMENTS and hand the result back as a
 * return value; they do not read or write the guest register file.  So the
 * allocator may keep every guest register in a host register across the call
 * and nothing has to be spilled.  That is exactly not true of the generic
 * `lightrec_rw` path, which works on the register file in the state block --
 * see build/docs/fgl_step4_generic_io.md.
 */
/* CHARGE WHAT THE BLOCK HAS ALREADY SPENT, BEFORE C IS ASKED ANYTHING.
 *
 * The epilogue charge is the right total and the wrong moment.  A device
 * register is answered from the guest clock, and a block that reaches a timer
 * read with its own cycles still unspent is answered as though the block had
 * only just started -- pcsx's interpreter has charged every instruction up to
 * and including the load by the time it reads, so the two get different
 * numbers out of the same hardware.
 *
 * The cost up to this node is its distance from the block's first
 * instruction, plus the node's own instruction, which the interpreter has
 * also charged by then.  Whatever is taken here the epilogue no longer owes,
 * so `charged` is subtracted from its total.
 *
 * Two instructions, on device accesses only.  r0 is the scratch the charge
 * needs and this runs before either shim path has built its arguments. */
/* Why a call-out did or did not charge, for FGL_CHARGE_LOG.  A device read
 * answered from a stale clock is invisible in the trace -- it looks like an
 * ordinary value -- so the log has to say which call sites were reached. */
static void fgl_charge_note(const fgl_emitter *e, const ir_node *p,
			    const char *why)
{
	static FILE *log;
	static int tried;

	if (!tried) {
		const char *v = getenv("FGL_CHARGE_LOG");

		tried = 1;
		if (v)
			log = fopen(v, "w");
	}
	if (log) {
		fprintf(log, "%08x %08x op=%u %s\n",
			e->charge_pc, p->pc, p->op, why);
		fflush(log);
	}
}

/* THE CHARGE IS AN INSTRUCTION COUNT, NOT A CYCLE COUNT.
 *
 * `r14` used to hold cycles, and a block's charge was `n_ops * cycles_per_op`
 * -- a runtime product around 1800*n that fits in no immediate, so it was a
 * GBR load of a precomputed table entry plus a subtract.  Two instructions and
 * a state-block read on every block and every mid-block device access:
 * 10.9M instructions in a hundred Spyro vsyncs, 6.6% of everything emitted,
 * against bleem's zero.
 *
 * So r14 counts GUEST INSTRUCTIONS instead.  `n` is at most a block's length,
 * which always fits `add #imm`, and the conversion to cycles happens once in
 * C -- where the budget is handed out and settled -- instead of once per
 * range in emitted code.  See fgl_cycles_in/out in fgl_run.c: the meter is
 * `ceil((target - current) / cycles_per_op)`, which makes the gate's
 * `cmp/pl r14` fire on exactly the instruction it used to.
 *
 * A zero charge now emits nothing at all; it used to load cycle_table[0] and
 * subtract it. */
static void emit_charge_ops(fgl_emitter *e, unsigned n)
{
	while (n > 127u) {
		sh4_emit_add_imm(&e->cg, -127, FGL_R_CYCLE);
		n -= 127u;
	}
	if (n)
		sh4_emit_add_imm(&e->cg, -(int)n, FGL_R_CYCLE);
}

static void emit_charge_upto(fgl_emitter *e, const ir_node *p)
{
	unsigned upto;

	fgl_charge_note(e, p, "seen");

	if (p->pc < e->charge_pc) {
		fgl_charge_note(e, p, "before-entry");
		return;
	}

	upto = ((p->pc - e->charge_pc) >> 2) + 1;

	/* A node further from the entry than any block is long means the two
	 * PCs are not from the same block -- a mapped-segment difference, say.
	 * Charge nothing rather than something wild. */
	if (upto > FGL_MAX_BLOCK_OPS)
		return;
	if (upto <= e->charged)
		return;

	upto -= e->charged;
	e->charged += upto;

	fgl_charge_note(e, p, "charged");

#if FGL_DEADLINE
	/* DEADLINE MODE: GENERATED CODE DOES NOT COUNT.  The mid-block charge
	 * exists so a device sees a clock that has already paid for the guest
	 * instructions ahead of it; with the clock advancing in lumps at the
	 * hook there is nothing here to pay.  `e->charged` is still tracked, so
	 * the epilogue below stays consistent with a non-deadline build. */
	(void)upto;
#else
	emit_charge_ops(e, upto);
#endif
}


static void emit_hw_load(fgl_emitter *e, const ir_node *p)
{
	uint8_t tag0 = e->cg.tag;

	e->cg.tag = FGL_TAG_HWL;
	uint32_t fn;
	int rs = p->sc[0];
	int rd;

	if (!e->tgt || !e->tgt->shim_call || p->sub >= 5 ||
	    !(fn = e->tgt->hw_load[p->sub]) || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	/* THE CHARGE COMES FIRST so that the address computation is the last
	 * thing before the call's constants: its final instruction is then
	 * what the delay-slot filler lifts into the `jsr`'s slot.  The charge
	 * uses r0 and r14 only, and r0 is dead until the address needs it. */
	emit_charge_upto(e, p);

	/* The address is the shim's first argument, so it is built in r1 and
	 * r0 is the spare -- the opposite of the direct path, which keeps r0
	 * for the address it is about to dereference. */
	emit_addr_raw(e, p, FGL_R_T1, FGL_R_XFER);

	emit_svc(e, FGL_SVC_SHIM_CALL, rs);
	emit_svc(e, FGL_SVC_HW_LOAD + (unsigned)p->sub, FGL_R_XFER);
	emit_jsr_slot(e, rs);

	/* The value comes back in r0, and from here this is the tail of an
	 * ordinary load: parked for the shadow if deferred, otherwise written
	 * wherever the allocator put the destination. */
	rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
	if (p->defer) {
		sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
	} else if (p->hd >= 0) {
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, rd);
	} else {
		st_guest(e, p->rd, FGL_R_XFER);
	}
	e->cg.tag = tag0;

}

/* The store, which is the one service with three values to pass and therefore
 * the one that parks a value in the state block on the way.  See shim.h. */
static void emit_hw_store(fgl_emitter *e, const ir_node *p)
{
	uint8_t tag0 = e->cg.tag;

	e->cg.tag = FGL_TAG_HWS;
	uint32_t fn;
	int rs = p->sc[0];
	int rt;

	if (!e->tgt || !e->tgt->shim_call_st || p->sub >= 5 ||
	    !(fn = e->tgt->hw_store[p->sub]) || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	/* The value first and through r0, because `mov.l Rm,@(disp,GBR)` has
	 * no other source register -- and it goes to memory before the address
	 * is built, so the address computation is free to use r0 as its spare
	 * afterwards. */
	/* The charge first, while r0 is still nobody's; see emit_hw_load. */
	emit_charge_upto(e, p);

	rt = operand(e, p->ht, p->rt, FGL_R_XFER);
	if (rt != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, rt, FGL_R_XFER);
	sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_SHIM_ARG);

	emit_addr_raw(e, p, FGL_R_T1, FGL_R_XFER);

	emit_svc(e, FGL_SVC_SHIM_CALL_ST, rs);
	emit_svc(e, FGL_SVC_HW_STORE + (unsigned)p->sub, FGL_R_XFER);
	emit_jsr_slot(e, rs);
	e->cg.tag = tag0;

}

/* The whole access, done by C.
 *
 * Everything about this node is in the state block: C reads the base register
 * from there and writes the destination back to there, which is why the
 * allocation pass flushes around it and why there are no operands to set up.
 * All that is left at the call site is the guest instruction word.
 *
 * The cycle counters are reconciled by the shim, which matters more here than
 * anywhere else: an unproven address can reach a device, a device can raise
 * an interrupt, and an interrupt is delivered by moving `target_cycle` in
 * memory where a register cannot see it. */
/* PUBLISH AND RELOAD, THE TWO HALVES OF THE PINNING CONTRACT AT A C CROSSING.
 *
 * C reads and writes guest registers through the state block and knows nothing
 * about r9-r12, so a node that calls a service has to put the pinned values
 * where C will look and take back whatever C left there.  Which nodes those
 * are is not a judgement call: it is exactly the set the allocator flushes at
 * (alloc.c, the IR_MTC_C/IR_MFC2_C/IR_RFE/IR_RW case), and the flush is what
 * makes an UNCONDITIONAL publish correct here -- after it, every pinned host
 * register is holding its own guest register again, so storing all of them
 * needs no per-node liveness.
 *
 * Two instructions each way per pin, because `@(disp,GBR)` has no destination
 * but r0.  That is why this is affordable only at the crossings: it would be
 * ruinous per block, and per block is exactly what pinning removes.
 *
 * BOTH RUN BEFORE r0 IS LOADED WITH ANYTHING, or after it has been consumed.
 * The publish is the first thing a service node emits and the reload the last,
 * so the callee address and the arguments are never live across either.
 *
 * A CROSSING WITHOUT A PUBLISH DOES NOT CRASH.  It hands C a stale register
 * and the guest drifts somewhere else entirely, so add one to the list here
 * whenever a new service is added; `grep emit_publish_pinned` is the check. */
#if FGL_NUM_PINS > 0
static const struct { uint8_t host, guest; } fgl_pin[FGL_NUM_PINS] = {
	{ FGL_PIN0_HOST, FGL_PIN0_GUEST },
#if FGL_NUM_PINS > 1
	{ FGL_PIN1_HOST, FGL_PIN1_GUEST },
#endif
#if FGL_NUM_PINS > 2
	{ FGL_PIN2_HOST, FGL_PIN2_GUEST },
#endif
#if FGL_NUM_PINS > 3
	{ FGL_PIN3_HOST, FGL_PIN3_GUEST },
#endif
#if FGL_NUM_PINS > 4
	{ FGL_PIN4_HOST, FGL_PIN4_GUEST },
#endif
#if FGL_NUM_PINS > 5
	{ FGL_PIN5_HOST, FGL_PIN5_GUEST },
#endif
};
#endif

/* FGL_NO_PIN_STUB=1 puts both sequences back inline, for bisection. */
int fgl_rw_tramp_ready;
static int rw_tramp_on(void)
{
	static int on = -1;
	if (on < 0) on = getenv("FGL_NO_RW_TRAMP") == NULL;
	return on && fgl_rw_tramp_ready;
}

int fgl_pin_stubs_ready;

static int pin_stub_on(void)
{
	static int on = -1;

	if (on < 0)
		on = getenv("FGL_NO_PIN_STUB") == NULL;
	return on && fgl_pin_stubs_ready;
}

/* The call, which is the same shape for both directions: the routine's
 * address out of the state block and a `jsr`.  Both clobber r0 and PR and
 * nothing else, and r0 is dead at every site -- the inline sequences these
 * replace used it as their transfer register.
 *
 * No `may_exit`, and the barrier for the same reason as the invalidation
 * stub: nothing from the surrounding code may be lifted into a delay slot
 * where it would run after r0 has become the routine's address. */
/* A SERVICE ADDRESS OUT OF THE STATE BLOCK INSTEAD OF THE LITERAL POOL.
 *
 * Two bytes into r0, four into anything else, against six and a pool word
 * either way for `emit_const`.  See FGL_AT_SVC.  Only ever called with an
 * address `fgl_targets` supplied, which is fixed for the life of the process
 * -- a value that could vary has no business in a state slot. */
static void emit_svc(fgl_emitter *e, unsigned idx, int rn)
{
	sh4_emit_mov_l_load_gbr(&e->cg, (int)(FGL_AT_SVC + idx));
	if (rn != FGL_R_XFER)
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, rn);
}

static void emit_pin_call(fgl_emitter *e, int slot)
{
	slot_barrier(e);
	sh4_emit_mov_l_load_gbr(&e->cg, slot);
	emit_delayed(e, SH4_N(0x400b, FGL_R_XFER), 1u << FGL_R_XFER);
}

static void emit_publish_pinned(fgl_emitter *e)
{
#if FGL_NUM_PINS > 0
	unsigned i;

	if (pin_stub_on()) {
		emit_pin_call(e, (int)FGL_AT_PIN_PUB);
		return;
	}
	for (i = 0; i < FGL_NUM_PINS; i++) {
		sh4_emit_mov_reg(&e->cg, fgl_pin[i].host, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)GUEST_AT(fgl_pin[i].guest));
	}
#else
	(void)e;
#endif
}

/* The other half.  Only for a service that RETURNS INTO THE BLOCK -- a node
 * that leaves for good (IR_STOP, IR_EXIT) publishes and does not reload,
 * because the next block's entry is where the pinned registers come from. */
static void emit_reload_pinned(fgl_emitter *e)
{
#if FGL_NUM_PINS > 0
	unsigned i;

	if (pin_stub_on()) {
		emit_pin_call(e, (int)FGL_AT_PIN_REL);
		return;
	}
	for (i = 0; i < FGL_NUM_PINS; i++) {
		sh4_emit_mov_l_load_gbr(&e->cg, (int)GUEST_AT(fgl_pin[i].guest));
		sh4_emit_mov_reg(&e->cg, FGL_R_XFER, fgl_pin[i].host);
	}
#else
	(void)e;
#endif
}

static void emit_rw(fgl_emitter *e, const ir_node *p)
{
	uint8_t tag0 = e->cg.tag;

	e->cg.tag = FGL_TAG_RW;
	int rs = p->sc[0];

	if (!e->tgt || !e->tgt->shim_call || !e->tgt->rw || !rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	/* THE GENERIC PATH NEEDS THE CHARGE MORE THAN THE DIRECT ONE DOES.
	 *
	 * `lightrec_can_hw_direct` refuses the root counters and GPUSTAT, so
	 * the registers whose value is literally derived from the guest clock
	 * are exactly the ones that come through here rather than through
	 * `emit_hw_load`.  Charging only the direct path leaves every timer
	 * read answered from a clock that has not moved since the block
	 * started. */
	emit_charge_upto(e, p);

	if (!rw_tramp_on()) {
		emit_publish_pinned(e);

		/* THE PC THE GENERIC PATH WILL LOOK THIS BLOCK UP BY; see
		 * FGL_PUBLISH_PC above.  BEFORE the operands, because it goes
		 * through r0 and r0 is about to become the first of them. */
		emit_publish_pc(e, p->pc);
	}

	if (rw_tramp_on()) {
		/* THE DATA HAS TO BE 4-ALIGNED: the trampoline reads it with
		 * `mov.l @r1+`.  It begins two words past the call, and a
		 * lift out of `emit_delayed` DOES move it: the lifted word
		 * leaves its place, so everything from there to the branch
		 * comes forward one word and the data with it.  Aligning
		 * before the branch was therefore right only when a lift
		 * followed, and every site had one until the store ahead of
		 * this one stopped emitting its invalidation (the
		 * LIGHTREC_OPT_INV_DMA_ONLY hack): the first site without a
		 * lift put its data on a 2-aligned address and the DC took a
		 * data address error in the trampoline.  So no lift here --
		 * the barrier makes the slot a `nop` -- and the `mov.l` goes
		 * at a 2-aligned address so that the call, the slot and the
		 * data fall at 0, 2 and 0 modulo 4. */
		slot_barrier(e);
		if (!((e->base + fgl_size(e)) & 2))
			sh4_emit_nop(&e->cg);
		sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_RW_TRAMP);
		emit_delayed(e, SH4_N(0x400b, FGL_R_XFER), 1u << FGL_R_XFER);
		sh4_word(&e->cg, (uint16_t)(p->pc & 0xffffu));
		sh4_word(&e->cg, (uint16_t)(p->pc >> 16));
		sh4_word(&e->cg, (uint16_t)(p->imm & 0xffffu));
		sh4_word(&e->cg, (uint16_t)(p->imm >> 16));
		/* The owning block, for the tag (fgl_state.h, FGL_AT_RW_BLOCK). */
		sh4_word(&e->cg, (uint16_t)(e->block_pc & 0xffffu));
		sh4_word(&e->cg, (uint16_t)(e->block_pc >> 16));
		slot_barrier(e);               /* data: nothing lifts past it */
		e->may_exit = 1;
		e->cg.tag = tag0;
		return;
	}

	/* THE SHIM ADDRESS FIRST, because both it and `rw` come through r0 and
	 * only `rw` is allowed to still be there at the `jsr`. */
	emit_svc(e, FGL_SVC_SHIM_CALL, rs);
	emit_svc(e, FGL_SVC_RW, FGL_R_XFER);
	emit_const(e, p->imm, FGL_R_T1);        /* the guest instruction word */

	emit_jsr_slot(e, rs);
	emit_reload_pinned(e);
	e->cg.tag = tag0;

}

/* Does this access reach a device rather than memory?
 *
 * Everything the optimiser could name a plain region for is memory, including
 * FGL_IO_DIRECT_HW -- that tag means the FRONT END certified this particular
 * hardware address as ordinary storage (optimizer.c, `ops.hw_direct`), so the
 * masked direct access is right for it under the same flat-map assumption RAM
 * and BIOS already rely on.
 *
 * FGL_IO_UNKNOWN is deliberately NOT here.  It is not memory either, but it
 * needs the generic `lightrec_rw` protocol rather than this one, and routing
 * it here would silently give it a device access it may not want.  Until that
 * path exists an unknown region is a refusal, which is what having no fallback
 * means. */
static int is_hw(const ir_node *p)
{
	return p->io == FGL_IO_HW;
}

/* ---------------------------------------------------------------- */
/* COP2 commands                                                     */
/* ---------------------------------------------------------------- */

/* THE ONE PLACE A BLOCK CALLS OUT, AND WHAT IT COSTS TO GET THERE.
 *
 * A GTE command is thousands of instructions of fixed-point geometry; there
 * was never a version of this that got inlined.  What there is instead is the
 * cheapest possible way to leave and come back:
 *
 *      mov.l   @(disp,pc), r0          ; the body, resolved right now
 *      mov.l   @(disp,pc), r1          ; the guest command word
 *      mov.l   @(disp,pc), rS          ; fgl_shim_gte
 *      jsr     @rS
 *       nop
 *
 * The body is chosen HERE, at compile time, because the command word is a
 * constant in the block -- so nothing decodes it at run time and nothing
 * compares it against a table on every execution.  This is the same decision
 * lightrec's own SH-4 path made (emitter.c, rec_CP2_gte) and the reason it
 * skipped the generic C wrapper: a COP2 command reads and writes the COP2
 * file in the state block and touches neither the cycle counter nor the
 * guest's general registers.
 *
 * WHY THERE IS NO FLUSH HERE.  The callee reads the COP2 file out of the
 * state block, and the COP2 file is never held in a host register -- IR_MTC2
 * stores straight through to the state block and IR_MFC2 loads straight out
 * of it.  So the file the callee sees is already the current one, and the
 * guest GPRs the allocator is holding in r3-r12 are none of its business.
 * If COP2 registers ever start living in registers, this comment is the thing
 * that stops being true.
 *
 * A command the hardware ignores resolves to nothing, and the right amount of
 * code for it is none -- not a call to an empty function. */
static void emit_gte(fgl_emitter *e, const ir_node *p)
{
	uint32_t body;
	int rs = p->sc[0];

	if (!e->tgt || !e->tgt->gte_body || !e->tgt->shim_gte) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	body = e->tgt->gte_body(e->tgt->user, p->imm);
	if (!body)
		return;

	/* BLEEM'S SHAPE, where a leaf exists: the routine's address in r0 and
	 * a `jsr`, nothing else.  No shim, no saved r2-r7, no cycle
	 * reconciliation -- the leaf keeps the contract itself and never
	 * calls C, so the cycle pair cannot have moved. */
	if (e->tgt->gte_leaf) {
		uint32_t leaf = e->tgt->gte_leaf(e->tgt->user, p->imm);

		if (leaf) {
			emit_svc(e, FGL_SVC_GTE_LEAF + (leaf >> 1), FGL_R_XFER);
			if (leaf & 1u)
				emit_const(e, p->imm, FGL_R_T1);
			emit_jsr_slot(e, FGL_R_XFER);
			return;
		}
	}

	/* The allocator owes this node a scratch register, because `jsr` wants
	 * its target in a general register and r0 and r1 are both carrying
	 * arguments. Without one there is nowhere to put the shim's address. */
	if (!rs) {
		e->unsupported = 1;
		e->unsupported_op = p->op;
		return;
	}

	emit_svc(e, FGL_SVC_SHIM_GTE, rs);
	emit_const(e, body, FGL_R_XFER);
	emit_const(e, p->imm, FGL_R_T1);
	emit_jsr_slot(e, rs);
}

/* THE TWO INVALIDATION STUBS, EMITTED ONCE.
 *
 * Called with the guest address in r1; clobbers r0, r1 and PR, and r1 is
 * already dead at every call site (the inline sequence this replaces zeroed
 * it).  See FGL_AT_INV_RAM in fgl_state.h for why this is a call at all.
 *
 * The r0-only restriction of `mov.l @(disp,GBR),r0` is what shapes the body:
 * the mask and the table base both have to come through r0, so the offset is
 * accumulated in r1 between the two loads rather than the other way round.
 *
 * The RAM stub does not touch T.  The DIRECT one does -- its region test is a
 * `tst` -- and so did the inline sequence it replaces, so no call site loses
 * anything it had.
 *
 * Returns bytes written, or 0 if the buffer was too small.  `ram` and
 * `direct` come back as byte offsets from the start of the buffer.
 */
unsigned fgl_emit_inv_stubs(void *buf, unsigned cap,
			    unsigned *ram, unsigned *direct,
			    unsigned *pub, unsigned *rel, unsigned *rwt)
{
	sh4_codegen cg;
	uint8_t *base = (uint8_t *)buf;
	int skip;

	cg.ptr = base;
	cg.end = base + cap;
	cg.overflow = 0;
	cg.tagp = NULL;
	cg.tag = FGL_TAG_INV;

	/* --- the RAM stub: the address is known to be guest RAM ---------- */
	*ram = 0;
	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_INV_MASK);  /* r0 = mask     */
	sh4_emit_and(&cg, FGL_R_XFER, FGL_R_T1);             /* r1 = offset   */
	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_LUT);       /* r0 = table    */
	sh4_emit_add_reg(&cg, FGL_R_T1, FGL_R_XFER);         /* r0 = &entry   */
	sh4_emit_mov_reg(&cg, FGL_R_XFER, FGL_R_T1);         /* r1 = &entry   */
	sh4_emit_mov_imm(&cg, 0, FGL_R_XFER);                /* r0 = NULL     */
	sh4_emit_rts(&cg);
	sh4_emit_mov_l_store(&cg, FGL_R_XFER, FGL_R_T1);     /* in the slot   */

	/* --- the DIRECT stub: test the region, then fall into the same --- */
	*direct = (unsigned)(cg.ptr - base);
	sh4_emit_mov_reg(&cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_and(&cg, FGL_R_MASK, FGL_R_XFER);           /* into window   */
	sh4_emit_shlr16(&cg, FGL_R_XFER);
	sh4_emit_shlr8(&cg, FGL_R_XFER);
	sh4_emit_tst(&cg, FGL_R_XFER, FGL_R_XFER);           /* T = it is RAM */
	skip = (int)(cg.ptr - base);
	sh4_emit_bf(&cg, 0);                                 /* patched below */

	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_INV_MASK);
	sh4_emit_and(&cg, FGL_R_XFER, FGL_R_T1);
	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_LUT);
	sh4_emit_add_reg(&cg, FGL_R_T1, FGL_R_XFER);
	sh4_emit_mov_reg(&cg, FGL_R_XFER, FGL_R_T1);
	sh4_emit_mov_imm(&cg, 0, FGL_R_XFER);
	sh4_emit_mov_l_store(&cg, FGL_R_XFER, FGL_R_T1);

	/* `bf` lands at PC + 4 + 2*disp, and PC is the branch's own address. */
	{
		int32_t d = (int32_t)((cg.ptr - base) - skip - 4) / 2;
		uint16_t w = (uint16_t)(0x8b00u | (uint8_t)(int8_t)d);

		base[skip]     = (uint8_t)(w & 0xff);
		base[skip + 1] = (uint8_t)(w >> 8);
	}

	sh4_emit_rts(&cg);
	sh4_emit_nop(&cg);

	/* --- publishing the pinned registers, and picking them back up ---
	 *
	 * The same two loops emit_publish_pinned and emit_reload_pinned would
	 * have inlined, written once.  See FGL_AT_PIN_PUB.  The final store
	 * and the final move go in the `rts` delay slot, so each routine is
	 * one instruction shorter than the loop that built it. */
	*pub = (unsigned)(cg.ptr - base);
#if FGL_NUM_PINS > 0
	{
		unsigned i;

		for (i = 0; i < FGL_NUM_PINS; i++) {
			sh4_emit_mov_reg(&cg, fgl_pin[i].host, FGL_R_XFER);
			if (i + 1 == FGL_NUM_PINS)
				sh4_emit_rts(&cg);
			sh4_emit_mov_l_store_gbr(&cg,
					(int)GUEST_AT(fgl_pin[i].guest));
		}
	}
#else
	sh4_emit_rts(&cg);
	sh4_emit_nop(&cg);
#endif

	*rel = (unsigned)(cg.ptr - base);
#if FGL_NUM_PINS > 0
	{
		unsigned i;

		for (i = 0; i < FGL_NUM_PINS; i++) {
			sh4_emit_mov_l_load_gbr(&cg,
					(int)GUEST_AT(fgl_pin[i].guest));
			if (i + 1 == FGL_NUM_PINS)
				sh4_emit_rts(&cg);
			sh4_emit_mov_reg(&cg, FGL_R_XFER, fgl_pin[i].host);
		}
	}
#else
	sh4_emit_rts(&cg);
	sh4_emit_nop(&cg);
#endif

	/* --- THE GENERIC-ACCESS TRAMPOLINE ------------------------------
	 *
	 * Called by `jsr` from the site, so PR names the three words that
	 * follow the delay slot: the guest pc, the guest instruction word
	 * and the pc of the lightrec block the site was compiled from.  Everything else the site used to spell out is the same at
	 * every site and lives here instead -- thirty-four bytes a site
	 * become fifteen.
	 *
	 * THE RETURN ADDRESS HAS TO CROSS A C CALL, and the only registers
	 * that survive one are the callee-saved set, all of which are either
	 * pinned guest registers or r13/r14.  So it rides in a PIN, which is
	 * dead between the publish and the reload precisely because it has
	 * been published.
	 *
	 * NOT r2.  r2 is FGL_R_EXIT, and a block materialises its exit PC
	 * there at the TRANSFER NODE -- which the decoder places ahead of the
	 * delay slot, so a generic access in that slot runs afterwards.  This
	 * trampoline used r2 as a scratch and left a host address in it; with
	 * linking on the site takes its target from its own literal and
	 * almost nothing reads r2, so it survived the whole bench.  With
	 * FGL_LINK=0 every exit reads r2 and the first block faulted.  The
	 * reload stub touches only r0 and the pins, so r1 is safe for the
	 * return address once the shim has returned. */
	*rwt = (unsigned)(cg.ptr - base);
	sh4_emit_sts_pr(&cg, FGL_R_T1);                      /* r1 = &pc     */
	sh4_emit_mov_l_load_inc(&cg, FGL_R_T1, FGL_R_XFER);  /* r0 = pc      */
	sh4_emit_mov_l_store_gbr(&cg, (int)FGL_AT_CURR_PC);
	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_PIN_PUB);
	sh4_emit_jsr(&cg, FGL_R_XFER);
	sh4_emit_nop(&cg);                        /* r8 is not published yet */
	sh4_emit_mov_reg(&cg, FGL_R_T1, fgl_pin[4].host);    /* r8 = &imm    */
	sh4_emit_mov_l_load_inc(&cg, fgl_pin[4].host, FGL_R_T1);  /* r1 = imm  */
	sh4_emit_mov_l_load_inc(&cg, fgl_pin[4].host, FGL_R_XFER); /* r0 = block,
							      r8 = return   */
	sh4_emit_mov_l_store_gbr(&cg, (int)FGL_AT_RW_BLOCK);
	sh4_emit_mov_l_load_gbr(&cg, (int)(FGL_AT_SVC + FGL_SVC_SHIM_CALL));
	sh4_emit_mov_reg(&cg, FGL_R_XFER, fgl_pin[3].host);  /* shim entry   */
	sh4_emit_mov_l_load_gbr(&cg, (int)(FGL_AT_SVC + FGL_SVC_RW));
	sh4_emit_jsr(&cg, fgl_pin[3].host);
	sh4_emit_nop(&cg);
	sh4_emit_mov_reg(&cg, fgl_pin[4].host, FGL_R_T1);    /* r1 = return  */
	sh4_emit_mov_l_load_gbr(&cg, (int)FGL_AT_PIN_REL);
	sh4_emit_jsr(&cg, FGL_R_XFER);
	sh4_emit_nop(&cg);
	sh4_emit_jmp(&cg, FGL_R_T1);
	sh4_emit_nop(&cg);

	if (cg.overflow)
		return 0;
	return (unsigned)(cg.ptr - base);
}

const char *const fgl_tag_name[FGL_TAG_N] = {
	"other", "preload", "spill", "flush", "body",
	"mem", "pool", "cycle", "epilogue", "link", "mask", "pin", "pinmid",
	"hook", "hwl", "hws", "rw", "inv",
};

/* IS THIS FIXUP A PINNED REGISTER LEAVING OR COMING BACK TO ITS OWN HOST?
 *
 * `pins.h` puts the six pins at the TOP of the pool so the allocator spends
 * its lowest-ranked registers first and a short block never disturbs one.  A
 * long block does, and then the boundary has to put the assignment back --
 * which is a load or a store like any other.  This separates that traffic from
 * the rest so the claim can be checked rather than assumed. */
static int fixup_is_pin(const ir_fixup *f)
{
#ifdef ALLOC_PINNED_LIVE
	int slot = (int)f->host - ALLOC_FIRST;

	return slot >= 0 && slot < ALLOC_N &&
	       ir_pin[slot] == (int8_t)f->guest;
#else
	(void)f;
	return 0;
#endif
}

static void emit_fixup(fgl_emitter *e, const ir_fixup *f)
{
	uint8_t tag0 = e->cg.tag;

	if (fixup_is_pin(f))
		e->cg.tag = (tag0 == FGL_TAG_FLUSH) ? FGL_TAG_PIN
						    : FGL_TAG_PINMID;

	if (f->store)
		st_guest(e, f->guest, f->host);
	else
		ld_guest(e, f->guest, f->host);

	e->cg.tag = tag0;
}

static void emit_node(fgl_emitter *e, const ir_node *p)
{
	int rd, rs, rt;

	switch (p->op) {
	case IR_MOVE:
		rs = operand(e, p->hs, p->rs, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
		if (rd != rs)
			sh4_emit_mov_reg(&e->cg, rs, rd);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SET:
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		emit_const(e, p->imm, rd);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_ALU:
		rs = operand(e, p->hs, p->rs, FGL_R_T1);
		/* The second operand cannot also land in r1: `alu_rr` uses it
		 * as the awkward-case working register. A node the allocator
		 * left with two operands in memory is given scratch for it. */
		rt = operand(e, p->ht, p->rt, p->sc[0] ? p->sc[0] : FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : (rs == FGL_R_T1 ? FGL_R_XFER : FGL_R_T1);
		alu_rr(e, p->sub, rd, rs, rt);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_ALU_IMM:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : FGL_R_XFER;
		alu_ri(e, p->sub, rd, rs, p->imm);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SHIFT_IMM:
		rt = operand(e, p->ht, p->rt, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		if (rd != rt)
			sh4_emit_mov_reg(&e->cg, rt, rd);
		shift_const(e, rd, p->imm & 31u, p->sub);
		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_SHIFT_REG:
		/* THE AMOUNT IS TAKEN BEFORE THE VALUE IS MOVED. `sllv rd,rt,rs`
		 * is allowed to name the same guest register for rd and rs, and
		 * moving the value into the destination first would destroy the
		 * count before it was read. Reading the count into r0 up front
		 * makes the aliasing case ordinary.
		 *
		 * The value is fetched first only in the sense of choosing its
		 * register: `operand` may go through r0 to load from the state
		 * block, so the count cannot already be sitting there. */
		rt = operand(e, p->ht, p->rt, FGL_R_T1);

		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		sh4_emit_and_imm(&e->cg, 31);           /* r0 &= 31 */
		/* shad/shld take a signed count, so a right shift is a
		 * negative left one. */
		if (p->sub != SH_LL)
			sh4_emit_neg(&e->cg, FGL_R_XFER, FGL_R_XFER);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		if (rd != rt)
			sh4_emit_mov_reg(&e->cg, rt, rd);

		if (p->sub == SH_RA)
			sh4_emit_shad(&e->cg, FGL_R_XFER, rd);
		else
			sh4_emit_shld(&e->cg, FGL_R_XFER, rd);

		if (p->hd < 0)
			st_guest(e, p->rd, rd);
		break;

	case IR_LOAD:
		if (is_hw(p)) {
			emit_hw_load(e, p);
			break;
		}
		/* The address goes in r0 and r1 is free for a wide
		 * displacement; the value lands wherever the allocator put it,
		 * or in r1 on its way to the state block.
		 *
		 * A DEFERRED load reads memory here and parks the value in the
		 * state block's temp word, writing no guest register -- the
		 * IR_TEMP_GET after the shadow instruction does that. See
		 * `ir.h`. */
		emit_addr(e, p, FGL_R_XFER, FGL_R_T1);
		rd = p->hd >= 0 ? p->hd : FGL_R_T1;
		switch (p->sub) {
		case MEM_B:
			sh4_emit_mov_b_load(&e->cg, FGL_R_XFER, rd);
			break;
		case MEM_BU:
			sh4_emit_mov_b_load(&e->cg, FGL_R_XFER, rd);
			sh4_emit_extu_b(&e->cg, rd, rd);
			break;
		case MEM_H:
			sh4_emit_mov_w_load(&e->cg, FGL_R_XFER, rd);
			break;
		case MEM_HU:
			sh4_emit_mov_w_load(&e->cg, FGL_R_XFER, rd);
			sh4_emit_extu_w(&e->cg, rd, rd);
			break;
		case MEM_W:
			sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, rd);
			break;
		default:
			e->unsupported = 1;
			e->unsupported_op = p->op;
			return;
		}
		if (p->defer) {
			if (rd != FGL_R_XFER)
				sh4_emit_mov_reg(&e->cg, rd, FGL_R_XFER);
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		} else if (p->hd < 0) {
			st_guest(e, p->rd, rd);
		}
		break;

	case IR_TEMP_GET:
		sh4_emit_mov_l_load_gbr(&e->cg, FGL_AT_TEMP_REG);
		if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			st_guest(e, p->rd, FGL_R_XFER);
		break;

	case IR_STORE:
		if (is_hw(p)) {
			emit_hw_store(e, p);
			break;
		}
		/* The other way round: the address is built in r1 first, so
		 * that r0 is still free to carry a value out of the state
		 * block. Doing it in the other order costs a scratch register
		 * on every store whose value is not in a register. */
		emit_addr(e, p, FGL_R_T1, FGL_R_XFER);

		/* DEBUG: catch a store whose address is outside guest RAM and
		 * outside the scratchpad -- a wild store -- at the moment it
		 * happens, while the dispatcher's ring still holds the blocks
		 * that led here.  Four instructions and a branch that is never
		 * taken in a healthy run. */
		if (e->tgt && e->tgt->wild && p->sc[0]) {
			int sk1, sk2;

			emit_const(e, 0x00200000u, FGL_R_XFER);
			sh4_emit_cmphs(&e->cg, FGL_R_XFER, FGL_R_T1);
			sk1 = bf_fwd(e);
			emit_const(e, 0x1f800000u, FGL_R_XFER);
			sh4_emit_cmphs(&e->cg, FGL_R_XFER, FGL_R_T1);
			sk2 = bt_fwd(e);
			/* Hand the offending address to C through the state
			 * block's temp slot: the handler takes no arguments
			 * and the argument registers are in the pool. */
			sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
			sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_TEMP_REG);
			emit_const(e, e->tgt->wild, p->sc[0]);
			emit_jsr_slot(e, p->sc[0]);
			patch_fwd8(e, sk1);
			patch_fwd8(e, sk2);
		}

		rt = operand(e, p->ht, p->rt, FGL_R_XFER);
		switch (p->sub) {
		case MEM_B:
		case MEM_BU:
			sh4_emit_mov_b_store(&e->cg, rt, FGL_R_T1);
			break;
		case MEM_H:
		case MEM_HU:
			sh4_emit_mov_w_store(&e->cg, rt, FGL_R_T1);
			break;
		case MEM_W:
			sh4_emit_mov_l_store(&e->cg, rt, FGL_R_T1);
			break;
		default:
			e->unsupported = 1;
			e->unsupported_op = p->op;
			return;
		}
		emit_invalidate(e, p, FGL_R_T1, p->sc[0]);
		break;

	case IR_LOAD_UN:
		emit_load_un(e, p);
		break;

	case IR_STORE_UN:
		emit_store_un(e, p);
		break;

	case IR_MFC0:
		/* COP0 sits inside GBR's reach like everything else, so a
		 * coprocessor read is the same one instruction an ordinary
		 * state access is -- no address computation, no second file. */
		sh4_emit_mov_l_load_gbr(&e->cg, (int)(FGL_AT_COP0 + (p->imm & 31)));
		if (p->defer)
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		else if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			st_guest(e, p->rd, FGL_R_XFER);
		break;

	case IR_MTC_C:
		/* The same call shape as IR_RW, and flushed the same way --
		 * C reads the source register out of the state block. */
		rs = p->sc[0];
		if (!e->tgt || !e->tgt->shim_call || !e->tgt->mtc || !rs) {
			e->unsupported = 1;
			e->unsupported_op = p->op;
			break;
		}
		emit_charge_upto(e, p);

		emit_publish_pinned(e);
		emit_svc(e, FGL_SVC_SHIM_CALL, rs);
		emit_svc(e, FGL_SVC_MTC, FGL_R_XFER);
		emit_const(e, p->imm, FGL_R_T1);   /* the guest instruction */
		emit_jsr_slot(e, rs);
		break;

	case IR_MTC0:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)(FGL_AT_COP0 + (p->imm & 31)));
		break;

	case IR_STOP:
		/* AN EXCEPTION IS AN ORDINARY BLOCK EXIT.
		 *
		 * Taking one is four state writes and a change of PC, and the
		 * block can do all five itself: EPC, CAUSE and SR into the
		 * state block, and the vector into the exit register, which
		 * the epilogue publishes to `next_pc` like any other
		 * destination.  So no service routine, no call out of
		 * generated code, and nothing in the runtime that has to know
		 * this block ended differently from a `j` -- the dispatcher
		 * compiles the handler at the vector the way it compiles any
		 * other address, and a guest that installed its own handler
		 * there gets it.
		 *
		 * The allocation pass has already flushed every live guest
		 * register (alloc.c, IR_STOP), so the handler's block sees
		 * coherent state.
		 *
		 * EPC first, SR last: the SR read has to see the value from
		 * BEFORE the push, and anything added here later that wants
		 * the old SR (a BEV test, say) finds it still in r1. */
		emit_const(e, p->imm, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, CP0_AT(COP0_EPC));

		emit_cause(e, p->sub);
		emit_sr_push(e);

		/* WHICH VECTOR, AND WHY IT IS TESTED RATHER THAN ASSUMED.
		 *
		 * SR bit 22 (BEV) picks the bootstrap vector at bfc00180h
		 * over the normal one at 80000080h. It is tempting to skip
		 * the test on the grounds that the PSX never sets BEV -- but
		 * the runtime we are keeping does test it (`psxException`,
		 * deps/pcsx_rearmed/libpcsxcore/r3000a.c:135), and a guest
		 * that sets BEV and then traps would otherwise be sent
		 * somewhere the interpreter would not send it. Ten
		 * instructions on a path that runs once per BIOS call is not
		 * where this project's time goes.
		 *
		 * `emit_sr_push` leaves the OLD SR in r1, which is what the
		 * test needs -- the push clears BEV's neighbours but not BEV,
		 * so either copy would do; taking the old one means nothing
		 * here depends on that staying true.
		 *
		 * Bit 22 is brought to bit 31 by two shifts and read with
		 * `cmp/pz`, rather than masked: `tst #imm` reaches only the
		 * low byte. */
		{
			int bev, over;

			sh4_emit_mov_reg(&e->cg, FGL_R_T1, FGL_R_XFER);
			sh4_emit_shll8(&e->cg, FGL_R_XFER);
			sh4_emit_shll(&e->cg, FGL_R_XFER);
			sh4_emit_cmppz(&e->cg, FGL_R_XFER);
			bev = bf_fwd(e);                /* T clear = BEV set */
			emit_const(e, GUEST_EXC_VECTOR, FGL_R_EXIT);
			over = bra_fwd(e);
			patch_fwd8(e, bev);
			emit_const(e, GUEST_EXC_VECTOR_BEV, FGL_R_EXIT);
			patch_fwd12(e, over);
		}
		break;

	case IR_RFE:
		/* THE BIT SHUFFLE IS THE EASY HALF, AND IT WAS THE ONLY HALF.
		 *
		 * Popping the interrupt-enable stack is four instructions and
		 * fgl used to emit them: SR bits 5:2 copied down to 3:0.  What
		 * it did not do is what `lightrec_rfe` (lightrec.c) does with
		 * the result -- write it back THROUGH `lightrec_mtc0`, which
		 * raises `LIGHTREC_EXIT_CHECK_INTERRUPT` when the restored
		 * Status unmasks an interrupt that is already pending.
		 *
		 * RFE is the last instruction of every interrupt handler, so
		 * that is the moment a second pending interrupt becomes
		 * deliverable.  Without the flag the emulator does not learn
		 * of it until the timeslice ends, and every interrupt after
		 * the first arrives late.
		 *
		 * lightrec's own emitter inlines a partial version of this --
		 * it sets the flag for a pending SOFTWARE interrupt only, and
		 * misses the hardware case its interpreter handles.  fgl calls
		 * the interpreter's function rather than copying the emitter's
		 * shortcut, so there is one behaviour and it is the complete
		 * one. */
		rs = p->sc[0];
		if (!e->tgt || !e->tgt->shim_call || !e->tgt->rfe || !rs) {
			e->unsupported = 1;
			e->unsupported_op = p->op;
			break;
		}
		emit_charge_upto(e, p);

		emit_svc(e, FGL_SVC_SHIM_CALL, rs);
		emit_svc(e, FGL_SVC_RFE, FGL_R_XFER);
		emit_jsr_slot(e, rs);
		break;

	case IR_MFC2:
		/* One load and one move.  No sign extension on the way out:
		 * the sixteen-bit registers are narrowed on the way IN
		 * (see IR_MTC2), so what is in the state block is already
		 * the value a read must hand back -- r3000a.h:483-484 reads
		 * `cp2[]` raw for exactly that reason. */
		sh4_emit_mov_l_load_gbr(&e->cg, (int)p->imm);
		if (p->sub == CP2_SX)
			sh4_emit_exts_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		else if (p->sub == CP2_ZX)
			sh4_emit_extu_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		if (p->defer)
			sh4_emit_mov_l_store_gbr(&e->cg, FGL_AT_TEMP_REG);
		else if (p->hd >= 0)
			sh4_emit_mov_reg(&e->cg, FGL_R_XFER, p->hd);
		else
			st_guest(e, p->rd, FGL_R_XFER);
		break;

	case IR_MFC2_C:
		/* The IR_RW call shape again; see IR_MTC_C. */
		rs = p->sc[0];
		if (!e->tgt || !e->tgt->shim_call || !e->tgt->mfc || !rs) {
			e->unsupported = 1;
			e->unsupported_op = p->op;
			break;
		}
		/* `fgl_mfc` WRITES the destination guest register into the
		 * state block, so this one reloads as well as publishes.
		 * `fgl_mtc` above only reads, and `lightrec_rfe` touches cop0
		 * alone -- which is why neither of those reloads and IR_RFE
		 * does not publish at all. */
		emit_charge_upto(e, p);

		emit_publish_pinned(e);
		emit_svc(e, FGL_SVC_SHIM_CALL, rs);
		emit_svc(e, FGL_SVC_MFC, FGL_R_XFER);
		emit_const(e, p->imm, FGL_R_T1);
		emit_jsr_slot(e, rs);
		emit_reload_pinned(e);
		break;

	case IR_MTC2:
		/* `imm2` IS THE HALFWORD FLAG HERE, not a displacement
		 * (decode.c:406) -- the field is used for one thing on this
		 * node and something else entirely on IR_LWC2/IR_SWC2.
		 *
		 * Nine of the sixty-four registers keep only the low
		 * halfword of a write and hand back its sign extension
		 * (decode.c:64-104; nocash GTE section, "Writing 32bit
		 * values to 16bit GTE registers by software does not
		 * trigger saturation").  The decoder has already worked out
		 * WHICH register this is; narrowing it is still the
		 * emitter's job, and it is one `exts.w` because the
		 * register number was known at compile time.  The reference
		 * does the same narrowing on the write side,
		 * `r3k_cop2_in` (r3000a.h:279-286). */
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_XFER)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_XFER);
		if (p->imm2)
			sh4_emit_exts_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)p->imm);

		/* A CONTROL WRITE HAS A SECOND DESTINATION.
		 *
		 * `gte_fpu.c` keeps the control file's derived form -- the
		 * matrices as floats, the projection offsets -- in caches it
		 * validates against `psxCP2CtrlGen`, and nothing else ever
		 * invalidates them.  Storing the register without bumping the
		 * counter leaves the geometry unit transforming with whatever
		 * was loaded before the first command, for the rest of the
		 * run: right at first, then progressively wrong as scenes load
		 * new matrices, and flat once `OFX`/`OFY` are stale.
		 *
		 * lightrec pays this on its CTC2 path too (lightrec.c:614).
		 * Four instructions on an instruction that runs a handful of
		 * times per object, against a cache that saves the matrix
		 * rebuild on every vertex. */
		if (p->sub) {
			if (!e->tgt || !e->tgt->cp2_ctrl_gen) {
				e->unsupported = 1;
				e->unsupported_op = p->op;
				break;
			}
			emit_const(e, e->tgt->cp2_ctrl_gen, FGL_R_XFER);
			sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, FGL_R_T1);
			sh4_emit_add_imm(&e->cg, 1, FGL_R_T1);
			sh4_emit_mov_l_store(&e->cg, FGL_R_T1, FGL_R_XFER);
			/* The RTPS leaf keys its XMTRX/offset cache on the
			 * second counter, bumped only by RT/TR/OFX/OFY writes
			 * (gte.h PSXCP2_RT_REG), so a CTC2 to H, DQA, DQB or
			 * the light matrices no longer forces a matrix rebuild. */
			if (PSXCP2_RT_REG(p->imm - FGL_AT_CP2D - 32u)) {	/* gte.h */
				sh4_emit_add_imm(&e->cg, 4, FGL_R_XFER);
				sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, FGL_R_T1);
				sh4_emit_add_imm(&e->cg, 1, FGL_R_T1);
				sh4_emit_mov_l_store(&e->cg, FGL_R_T1, FGL_R_XFER);
			}
		}
		break;

	case IR_LWC2:
		/* Address in r0 with r1 free for a wide displacement, the
		 * same pairing IR_LOAD uses, and the loaded word goes
		 * straight back out through r0 -- so the whole node holds
		 * nothing live across the state access.
		 *
		 * No narrowing, deliberately: the reference stores the full
		 * word (r3000a.h:518), and matching it is what the oracle
		 * compares against.  See NOTES. */
		emit_cop2_addr(e, p, FGL_R_XFER, FGL_R_T1);
		sh4_emit_mov_l_load(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)p->imm);
		break;

	case IR_SWC2:
		/* Address in r1 first, as in IR_STORE: r0 has to stay free
		 * to carry the coprocessor register out of the state block,
		 * and there is no other register that can. */
		emit_cop2_addr(e, p, FGL_R_T1, FGL_R_XFER);
		sh4_emit_mov_l_load_gbr(&e->cg, (int)p->imm);
		/* Narrowed on the way out, exactly as IR_MFC2 narrows it: a
		 * store of a coprocessor register is a read of it.  See
		 * decode.c on SWC2. */
		if (p->sub == CP2_SX)
			sh4_emit_exts_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		else if (p->sub == CP2_ZX)
			sh4_emit_extu_w(&e->cg, FGL_R_XFER, FGL_R_XFER);
		sh4_emit_mov_l_store(&e->cg, FGL_R_XFER, FGL_R_T1);
		break;

	case IR_MULDIV:
		emit_muldiv(e, p);
		break;

	case IR_GTE:
		emit_gte(e, p);
		break;

	case IR_RW:
		emit_rw(e, p);
		break;

	case IR_JUMP:
		/* Linked and gated at entry: no path from this block's exit
		 * reads r2, the stub takes the target from the site's literal.
		 * See `link_last` and emit_link_site. */
		if (FGL_ENTRY_HOOK && e->link_last)
			break;
		/* A local edge loads r2 itself, and only on the path that
		 * needs it (emit_local_edge). */
		if (e->local_last && !(region_dbg() & 4))
			break;
		emit_const(e, p->imm, FGL_R_EXIT);
		break;

	/* LEAVING FOR C, WHICH IS DONE BY ARITHMETIC AND NOT BY A BRANCH.
	 *
	 * lightrec has no "return to the caller" instruction in a block. What
	 * it has is one budget test in the dispatcher, so a block that wants
	 * out reconciles the absolute counters and sets the delta to zero;
	 * the dispatcher regains control by the ordinary path, finds the
	 * budget spent and returns to C, which reads `exit_flags` to find out
	 * that this was a request rather than a timeout.
	 *
	 * The reconciliation is `current = target = target - delta`, which is
	 * what `rec_exit_early` does (emitter.c) -- the delta being what is
	 * left unspent, so subtracting it from the target is what the machine
	 * actually reached.
	 *
	 * The block still runs its own epilogue after this, which charges for
	 * the instructions and publishes the exit PC. That charge lands on a
	 * delta that is now zero and makes it negative, which is harmless:
	 * the test is `<= 0` and the counters C reads were already written.
	 *
	 * Seven instructions, and every exit code is a power of two below 64,
	 * so the flag is an immediate and costs no literal. */
	case IR_EXIT:
		/* Park the unspent meter and close the gate; C turns it into
		 * cycles.  See FGL_AT_EXIT_METER in fgl_state.h -- this used
		 * to be `current = target = target - r14`, which is only
		 * arithmetic a cycle budget can do. */
		sh4_emit_mov_reg(&e->cg, FGL_R_CYCLE, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_EXIT_METER);
		sh4_emit_mov_imm(&e->cg, 0, FGL_R_CYCLE);

		sh4_emit_mov_imm(&e->cg, (int)p->imm, FGL_R_XFER);
		sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_EXIT_FLAGS);

		/* Where C resumes. The epilogue publishes it like any other
		 * exit, so there is nothing further to do here. */
		emit_const(e, p->imm2, FGL_R_EXIT);
		break;

	case IR_CAPTURE:
		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		if (rs != FGL_R_EXIT)
			sh4_emit_mov_reg(&e->cg, rs, FGL_R_EXIT);
		break;

	case IR_COND: {
		int taken;

		rs = operand(e, p->hs, p->rs, FGL_R_XFER);
		rt = (p->sub == CC_EQ || p->sub == CC_NE)
			   ? operand(e, p->ht, p->rt, p->sc[0] ? p->sc[0] : FGL_R_T1)
			   : 0;

		switch (p->sub) {
		case CC_EQ:  sh4_emit_cmpeq(&e->cg, rt, rs); break;
		case CC_NE:  sh4_emit_cmpeq(&e->cg, rt, rs); break;
		case CC_LEZ: sh4_emit_cmppl(&e->cg, rs); break;  /* T = rs > 0 */
		case CC_GTZ: sh4_emit_cmppl(&e->cg, rs); break;
		case CC_LTZ: sh4_emit_cmppz(&e->cg, rs); break;  /* T = rs >= 0 */
		case CC_GEZ: sh4_emit_cmppz(&e->cg, rs); break;
		default:     e->unsupported = 1; e->unsupported_op = p->op; return;
		}

		/* T now says "the easy sense"; whether that is taken or not
		 * depends on the condition, so the branch is chosen rather
		 * than the comparison inverted. */
		/* FOUR WORDS, NOT SIX.  The fallthrough PC goes into the exit
		 * register unconditionally and the taken PC overwrites it
		 * behind a branch that skips the overwrite.  One conditional
		 * branch, no `bra`, no empty delay slot; the not-taken arm
		 * executes one more load than before and the taken arm one
		 * fewer branch.  The pool loads do not touch T, so the
		 * epilogue's link arm still reads the comparison's T. */
		/* WHEN THE EPILOGUE LINKS THIS BRANCH IT SETS r2 ITSELF, once
		 * per arm, behind the arm branch it has to emit anyway.  That
		 * is bleem's BEQ/BNE shape: one conditional branch, each arm
		 * materialising its own target next to its own link.  The
		 * three words below would be a second branch and two loads
		 * doing the same job, and the join label they end on is what
		 * kept the link sites' delay slots empty. */
		if (e->cond_arms)
			break;

		emit_const(e, p->imm2, FGL_R_EXIT);             /* fallthrough */
		taken = (p->sub == CC_EQ || p->sub == CC_GTZ || p->sub == CC_GEZ)
			      ? bf_fwd(e)       /* T set = taken: skip on clear */
			      : bt_fwd(e);      /* T set = not taken */
		emit_const(e, p->imm, FGL_R_EXIT);              /* taken */
		patch_fwd8(e, taken);
		break;
	}

	default:
		e->unsupported = 1;
		e->unsupported_op = p->op;
		break;
	}
}

uint32_t fgl_emit(fgl_emitter *e, const ir_node *ir, int n, const ir_alloc *a,
		  unsigned n_ops)
{
	uint32_t entry;
	int i, f = 0, need_exit = 1;
	int miss[3], n_miss = 0, seq_exit = 1, k2;

	entry = e->base + fgl_size(e);
	slot_barrier(e);

	/* THE BLOCK'S FIRST GUEST INSTRUCTION, WHICH IS NOT ir[0].
	 *
	 * A constant fold (front.c's `movi_step`) turns a LUI/ORI pair into no
	 * IR at all, so the first node can sit several instructions into the
	 * block.  Measuring the mid-block charge from it undercounts by
	 * exactly those instructions -- which is a device register read
	 * answered too early, and a wrong value handed to the guest.  The
	 * caller sets `charge_pc` to the real entry; ir[0] is the fallback for
	 * callers that do not (the unit tests). */
	if (!e->charge_pc)
		e->charge_pc = n > 0 ? ir[0].pc : 0;
	e->charged = 0;
	e->cond_saved = 0;
	e->may_exit = 0;
	e->local_last = 0;
	region_define(e, e->charge_pc);

	/* THE TRANSFER THE EPILOGUE WILL LINK, FOUND BY A BACKWARD SCAN.
	 *
	 * The decoder puts a transfer AHEAD of its delay slot, so in
	 * `j target; addiu $a0,$a0,1` the IR_JUMP is at n-2 and the slot's
	 * node is last.  Testing only the last node meant a block whose delay
	 * slot decoded to anything real got NO link site and went round the
	 * dispatcher on every execution for ever.  Bloom measured 95% of
	 * dispatcher entries coming from such blocks; here it was 4.24M
	 * dispatcher entries per 300 vsyncs against the DC's 168k, nearly all
	 * of them two-instruction local-branch blocks (`bltz; slot`).
	 *
	 * bleem links every direct branch with no such condition
	 * (recompiler_backend.md, "Chaining is unconditional"), so this is the
	 * shape to match.  A conditional found back here is linkable too, now
	 * that T is parked across the delay slot (FGL_AT_TSAVE).  A capture is
	 * not: its target is a register and there is nothing to patch to, so
	 * stop rather than walk past the transfer. */
	e->xfer_at = -1;
	if (FGL_LINK) {
		for (i = n - 1; i >= 0; i--) {
			if (ir[i].op == IR_JUMP || ir[i].op == IR_COND) {
				e->xfer_at = i;
				break;
			}
			if (ir[i].op == IR_CAPTURE)
				break;
		}
	}

#if FGL_ENTRY_HOOK
	/* THE BLOCK HEADER, AND THE TABLE ENTERS FOUR BYTES IN.  See
	 * FGL_ENTRY_HOOK in fgl.h for the layout and why the entry is at E+4.
	 *
	 * `bt -5` from E+6 lands on E+0: the SH-4 computes a conditional
	 * branch as PC + 4 + disp*2, so E+6 + 4 - 10 = E+0.
	 *
	 * `jsr`, as bleem: r2 no longer holds this block's guest pc on entry
	 * (a linked edge does not load it), so the dispatcher reads it from
	 * the header word at PR-10 instead (fgl_call_out, FGL_SH4_DISPATCH).
	 * The word is written as two halves: the block start need not be
	 * longword-aligned. */
	e->cg.tag = FGL_TAG_HOOK;
	sh4_word(&e->cg, (uint16_t)(e->charge_pc & 0xffff));    /* H+0 */
	sh4_word(&e->cg, (uint16_t)(e->charge_pc >> 16));       /* H+2 */
	sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_DISPATCH);   /* E+0 */
	sh4_emit_jsr(&e->cg, FGL_R_XFER);                        /* E+2 */
	sh4_emit_tst(&e->cg, FGL_R_CYCLE, FGL_R_CYCLE);          /* E+4 */
	sh4_emit_bt(&e->cg, -5);                                 /* E+6 */
	slot_barrier(e);
#endif

	e->cg.tag = FGL_TAG_PRELOAD;
	for (i = 0; i < a->n_preload; i++)
		emit_fixup(e, &a->preload[i]);
	peep_hoist(e);

	for (i = 0; i < n; i++) {
		e->cg.tag = FGL_TAG_SPILL;
		while (f < a->n_fix && a->fix[f].at == i)
			emit_fixup(e, &a->fix[f++]);
		e->cg.tag = FGL_TAG_BODY;
		maybe_flush_pool(e);
		/* THE TARGETS OF A BLOCK-ENDING CONDITIONAL ARE SET IN THE
		 * EPILOGUE'S ARMS, not by the node -- bleem's shape (see
		 * IR_COND and the arm code below).  Only the last node can
		 * be linked, so only the last node defers. */
		e->cond_arms = !no_arms() && FGL_LINK && FGL_LINK_COND == 2 && i == e->xfer_at &&
			       ir[i].op == IR_COND;
		e->link_last = FGL_LINK && i == e->xfer_at && ir[i].op == IR_JUMP;
		e->local_last = e->link_last && region_find(e, ir[i].imm) >= 0;
		emit_node(e, &ir[i]);

		/* PARK T ACROSS THE DELAY SLOT (bloom).  The link arm below is
		 * chosen by the T bit IR_COND's comparison left standing, and
		 * that only survives while IR_COND is the last node.  Two
		 * instructions here, two at the link point, in conditional
		 * blocks with a real delay slot only.  r0 is per-node scratch
		 * and nothing carries a value in it across a node boundary. */
		if (FGL_LINK && FGL_LINK_COND && i == e->xfer_at &&
		    ir[i].op == IR_COND && i != n - 1) {
			if (e->hint_in && !layout_off(4) &&
			    (e->hint_in[e->cur_lbl] & FGL_HINT_TLIVE)) {
				/* Pass 1 read every word of the slot: T
				 * survives it (fgl_region_hints). */
				e->cond_saved = 2;
				layout_tlive_n++;
			} else {
				sh4_emit_movt(&e->cg, FGL_R_XFER);
				sh4_emit_mov_l_store_gbr(&e->cg, (int)FGL_AT_TSAVE);
				e->cond_saved = 1;
				if (e->hint_out)
					e->tsave_at[e->cur_lbl] = here(e);
			}
		}
		peep_hoist(e);
	}

	e->cg.tag = FGL_TAG_FLUSH;
	while (f < a->n_fix)
		emit_fixup(e, &a->fix[f++]);
	peep_hoist(e);

	/* `cond_arms` is per node and the slot's node ran after the transfer,
	 * clearing it.  The epilogue's arms need the transfer's value. */
	e->cond_arms = !no_arms() && FGL_LINK && FGL_LINK_COND == 2 && e->xfer_at >= 0 &&
		       ir[e->xfer_at].op == IR_COND;
	e->link_last = FGL_LINK && e->xfer_at >= 0 && ir[e->xfer_at].op == IR_JUMP;

	/* THE EPILOGUE: CHARGE FOR THE BLOCK, AND LEAVE THROUGH THE DISPATCHER.
	 *
	 * Five instructions.  The block does not return -- it jumps to the
	 * address in the state block, carrying the guest PC it is leaving for
	 * in r2, which is where the exit register has held it all along.
	 *
	 * IT USED TO BE `rts`, AND THAT WAS ONE INSTRUCTION CHEAPER-LOOKING
	 * AND STRICTLY WORSE.  Returning means PR holds the block's return
	 * address for the block's entire life, so PR silently joins the
	 * register contract and every service routine called from inside a
	 * block has to save and restore it -- while fgl.h says a service may
	 * clobber "r0 and r1 and nothing else" and does not mention PR.  The
	 * `rts` form also filled its delay slot usefully, with the store of
	 * the exit PC to `next_pc`, so it looked like it was getting that
	 * publication for free.  It was not free; it was unnecessary.  The
	 * dispatcher reads r2.
	 *
	 * So: same five instructions, PR is nobody's, and `next_pc` is written
	 * only on the paths where C actually reads it.  The `nop` is forced --
	 * `mov.l @(disp,GBR),Rn` exists only for R0, so the dispatch address
	 * has to land in r0, and an instruction in a `jmp`'s delay slot may
	 * not modify the jump's target register.
	 *
	 * The last thing this buys is the one that matters later: a jump
	 * through a slot can be PATCHED into a direct branch to the block that
	 * follows it.  An `rts` can never be linked to anything.
	 *
	 * A block that consumed no guest instructions charges nothing, which
	 * is entry 0 of the table and still a correct load.
	 *
	 * A BLOCK CAN BE LONGER THAN THE TABLE.  `IR_MAX_NODES` bounds the IR,
	 * not the guest instructions behind it: a constant fold (front.c's
	 * `movi_step`) turns a LUI/ORI pair into no nodes at all, and BIOS
	 * init code is largely made of those.  Blocks of fifty guest ops
	 * against a forty-node budget are ordinary.  This used to CLAMP, which
	 * silently charged 33 ops for all of them -- the block at `bfc001f0`
	 * is 63 ops, and fgl billed it 57 cycles against the interpreter's
	 * 110.  A guest clock that runs slow is invisible to a register
	 * comparison until an interrupt lands on a different instruction.
	 *
	 * So charge in whole table-loads and then the remainder.  Two extra
	 * instructions per 33 ops, on the epilogue of a long block only. */
	/* A DELIBERATE ERROR IN THE GUEST CLOCK, TO SEE IF THE FAULT MOVES.
	 *
	 * Two theories for the Spyro fault: fgl emits a wrong value, or fgl
	 * charges cycles differently from the interpreter and an interrupt
	 * lands on a different instruction.  They are told apart by nudging
	 * the clock and nothing else -- if the crash address moves, the bug is
	 * timing; if it sits at exactly the same address, the value is wrong
	 * and cycles are innocent.
	 *
	 * Biasing the table index costs no instructions and no bytes: the
	 * displacement changes, the encoding does not.  So the two builds have
	 * identical layout and the comparison is not confounded by it. */
	e->cg.tag = FGL_TAG_CYCLE;
	n_ops += FGL_CYCLE_BIAS;

	/* Mid-block device accesses have already taken part of this. */
	n_ops = n_ops > e->charged ? n_ops - e->charged : 0;

#if FGL_DEADLINE
	/* THE WHOLE POINT OF DEADLINE MODE, AND THE ONLY THING IT DELETES FROM
	 * A BLOCK.  Two instructions on every epilogue, 30.2M executions in the
	 * Spyro savestate scene -- exactly 2.00 per block, because every block
	 * pays the load and the subtract whatever its length.
	 *
	 * r14 survives as the GATE: `cmp/pl` at the link sites is unchanged and
	 * still the only thing standing between a linked chain and forever.
	 * What changes is who moves it.  A block used to; now the hook does,
	 * in lumps. */
	(void)n_ops;
#else
	emit_charge_ops(e, (unsigned)n_ops);
#endif
	e->cg.tag = FGL_TAG_EPILOGUE;

	/* THE LINK, WHEN THE SUCCESSORS WERE KNOWN AT COMPILE TIME.
	 *
	 * Two nodes put compile-time addresses in the exit register.  `IR_JUMP`
	 * has one successor; `IR_COND` has TWO, both constants -- the taken
	 * target and the fallthrough -- because a MIPS conditional branch has
	 * a sixteen-bit signed displacement and nothing else.  `IR_CAPTURE`
	 * puts a guest register there, which is an indirect branch and cannot
	 * be linked to anything.
	 *
	 * Neither can be branched to here, because the successor is usually
	 * not compiled yet.  Instead each edge is a call to `fgl_link_stub`,
	 * which REWRITES ITS OWN CALL SITE the first time that successor
	 * exists -- see fgl_lightrec.c.  From the second execution the edge is
	 * two instructions and no memory reference, in place of the three
	 * below plus thirteen in the dispatcher.
	 *
	 * FINDING THE TRANSFER IS A BACKWARD SCAN, NOT `ir[n - 1]`.  The
	 * decoder places a transfer's node AHEAD of its delay slot (decode.c,
	 * `ir_decode`, and the comment at the top of ir.h says why), so in
	 * `j target; addiu $a0,$a0,1` the IR_JUMP is at n-2 and the slot's
	 * node is last.  Testing the last node therefore linked only the
	 * transfers whose delay slot decoded to no nodes at all -- a `nop`, or
	 * a pair the constant folder ate.  That is most of what the first
	 * census counted, and it is why so few edges appeared.
	 *
	 * THE BUDGET TEST HAS TO BE HERE, and this is the only reason it is.
	 * It used to live entirely in the dispatcher, which every exit passed
	 * through.  A linked edge does not pass through the dispatcher, so a
	 * guest loop of two linked blocks would run for ever without ever
	 * checking its timeslice.  `cmp/pl` and its branch are what buy that
	 * back, and they are paid only by blocks that can actually link.  One
	 * test covers both arms of a conditional. */
	{
		/* `t` is the transfer the backward scan in the node loop found
		 * (`xfer_at`), which is the last node only when the delay slot
		 * decoded to nothing.  Scanning back was once tried here and
		 * Spyro faulted during BIOS boot; that version did not park T,
		 * so a slot node writing T swapped the two arms.  It does now. */
		const ir_node *t = (e->xfer_at >= 0) ? &ir[e->xfer_at] : NULL;

		/* FGL_SLOT_STATS also counts what each block ends in.  A block
		 * that ends in IR_CAPTURE cannot be linked at all -- its
		 * successor is a register -- so this histogram is the ceiling
		 * on what linking could ever cover. */
		if (slot_stats_on())
			slot_xfer[t ? (unsigned)t->op & 15u : 15u]++;

		if (FGL_LINK && t && (t->op == IR_JUMP ||
			  (FGL_LINK_COND && t->op == IR_COND))) {
			int fall = -1, out1 = -1, out2 = -1, direct = 0;
			uint16_t hoist = 0;

			need_exit = 0;
			seq_exit = 0;

			/* THE ARM IS CHOSEN BY THE T BIT, NOT BY RE-ASKING.
			 *
			 * A site patches itself into a branch to ONE address,
			 * so the two arms of a conditional cannot share one.
			 * The first version of this picked between them by
			 * re-materialising the taken PC into r0 and comparing
			 * it against the exit register -- the answer rather
			 * than the question -- because T "obviously" could not
			 * have survived.  It cost a literal, a compare and a
			 * branch, and it faulted.
			 *
			 * T DOES SURVIVE, and nothing between here and the
			 * comparison can disturb it: IR_COND's own arms are
			 * `mov`/`mov.l`/`bra`, the register writeback flush is
			 * `st_guest` and `ld_guest` which are also only
			 * `mov`/`mov.l`, and the cycle charge above is
			 * `mov.l` and `sub` -- SUB does not write T on SH-4,
			 * only SUBC and SUBV do.  So the compare's result is
			 * still standing, and this is bloop's shape: one
			 * conditional branch off the original comparison and a
			 * link on each side of it.
			 *
			 * The sense is recomputed from the condition code the
			 * same way IR_COND computed it, rather than carried in
			 * the emitter, so the two cannot drift apart silently
			 * -- they are the same expression. */
			if (FGL_LINK_COND && t->op == IR_COND) {
				int t_means_taken = (t->sub == CC_EQ ||
						     t->sub == CC_GTZ ||
						     t->sub == CC_GEZ);

				/* THE WORD BOTH ARMS' SLOTS WILL SHARE.  The
				 * last word emitted so far (a flush store,
				 * usually) moves from here to the delay slot
				 * of each link site, so it runs once on either
				 * path, after the arm branch and after the
				 * site's `mov.l @(LINK,gbr),r0`.  It therefore
				 * must not touch T or r0 and must commute with
				 * that load; `slot_decode` says whether it
				 * does.  Mode 2 only: mode 1 and 3 have one
				 * site and one slot, and `emit_delayed` fills
				 * that on its own.
				 *
				 * ENTRY-HOOK BUILDS ONLY.  Without the hook each
				 * arm has `cmp/pl; bf out` between the split
				 * and its site, and a budget miss takes `out`
				 * PAST the site -- and past the slot, so the
				 * hoisted store would never run on that path.
				 * Found the hard way: plain deadline faulted
				 * with a stale guest register.  With the hook
				 * every path through an arm runs its site. */
				if (FGL_ENTRY_HOOK && FGL_LINK_COND == 2 &&
				    !e->cg.overflow && !slot_off() &&
				    here(e) - 1 >= e->slot_floor) {
					slot_info x, l;
					uint16_t w = word_at(e, here(e) - 1);

					slot_decode(w, &x);
					slot_decode((uint16_t)(0xc600u | FGL_AT_LINK), &l);
					if (x.legal && !x.unknown && w != 0x0009 &&
					    !((x.rd | x.wr) & SI_T) &&
					    !((x.rd | x.wr) & ((1u << FGL_R_XFER) |
							       (1u << FGL_R_EXIT))) &&
					    slot_independent(&x, &l)) {
						hoist = w;
						e->cg.ptr -= 2;
						if (e->cg.tagp)
							e->cg.tagp--;
					}
				}

				/* UNPARK T, when the delay slot emitted
				 * anything.  `cmp/pl` and not `tst`: movt left
				 * 1 or 0, and `T = (r0 > 0)` reproduces the
				 * original bit, while `tst r0,r0` would invert
				 * it and silently swap the two arms.  After the
				 * hoist above, which refuses words touching T
				 * or r0 -- these do both. */
				if (e->cond_saved == 1) {
					if (e->hint_out &&
					    e->tsave_at[e->cur_lbl] > 0) {
						int k, live = 1;
						slot_info x;

						for (k = e->tsave_at[e->cur_lbl];
						     k < here(e); k++) {
							slot_decode(word_at(e, k), &x);
							if (x.unknown || (x.wr & SI_T))
								live = 0;
						}
						if (live)
							e->hint_out[e->cur_lbl] |=
								FGL_HINT_TLIVE;
					}
					sh4_emit_mov_l_load_gbr(&e->cg,
							(int)FGL_AT_TSAVE);
					sh4_emit_cmppl(&e->cg, FGL_R_XFER);
					slot_barrier(e);
				} else if (e->cond_saved == 2) {
					slot_barrier(e);
				}

				/* Which arm walks away from the site.  Mode 3
				 * is the mirror of 1 and 2: the single site
				 * below belongs to the fallthrough instead. */
				if (FGL_LINK_COND == 3)
					fall = t_means_taken ? bt_fwd(e)
							     : bf_fwd(e);
				else
					fall = t_means_taken ? bf_fwd(e)
							     : bt_fwd(e);
			}

			/* THE BUDGET TEST HAS TO BE HERE, and this is the only
			 * reason it is.  It used to live entirely in the
			 * dispatcher, which every exit passed through.  A
			 * linked edge does not pass through the dispatcher, so
			 * a guest loop of two linked blocks would run for ever
			 * without checking its timeslice.  Each arm pays its
			 * own copy: `cmp/pl` writes T, so one test shared
			 * ahead of the arm branch would destroy the very bit
			 * the arm branch reads. */
			/* THIS ARM'S TARGET, which IR_COND left to us (see
			 * `cond_arms`).  Before the budget test: a budget
			 * miss leaves through the dispatcher with r2 as the
			 * exit PC.  A pool load does not touch T. */
			/* THE ARM IS A HOST BRANCH WHEN ITS TARGET IS IN THE
			 * REGION (fgl.h, "REGION").  Bleem's shape for a
			 * block-internal branch, and lightning's: no site, no
			 * stub, no dispatcher. */
			/* THE IDLE FLAG IS ON THE TAKEN EDGE ONLY.  lightrec
			 * sets it on a backward branch, so `imm` is the loop
			 * top and `imm2` is the way out; clamping the exit
			 * arm would cut short a slice that is doing work. */
			{
				int l = region_find(e, t->imm);

				if (l >= 0 && fall >= 0 && FGL_LINK_COND == 2 &&
				    e->lbl[l].at < 0 && !e->may_exit &&
				    !(t->hint & FGL_H_IDLE) &&
				    !(region_dbg() & 1) && !layout_off(8) &&
				    e->n_lref < FGL_MAX_LREFS) {
					if (e->hint_out) {
						e->arm_site[e->cur_lbl] = fall;
						e->arm_lbl[e->cur_lbl] = l;
					}
					if (e->hint_in &&
					    (e->hint_in[e->cur_lbl] & FGL_HINT_BT)) {
						/* The `bf skip` becomes `bt
						 * label` (fgl_region_hints). */
						xor_word_at(e, fall, 0x0200);
						e->lref[e->n_lref].site = fall;
						e->lref[e->n_lref].lbl = l;
						e->lref[e->n_lref].cond = 1;
						e->n_lref++;
						slot_barrier(e);
						layout_bt_n++;
						direct = 1;
						fall = -1;
						out1 = -1;
						goto taken_done;
					}
				}
				if (l >= 0) {
					out1 = emit_local_edge(e, t->imm,
							t->hint & FGL_H_IDLE);
					goto taken_done;
				}
			}

			if (t->op == IR_COND && e->cond_arms && !FGL_ENTRY_HOOK)
				emit_const(e, t->imm, FGL_R_EXIT);       /* taken */

			if (t->hint & FGL_H_IDLE)
				emit_idle_clamp(e);

#if FGL_ENTRY_HOOK
			/* THE TEST MOVED TO THE TARGET.  Every block now
			 * begins with its own `tst r14,r14`, so a linked edge
			 * is checked when it ARRIVES rather than before it
			 * leaves -- which is the same check, once, instead of
			 * once per arm here. */
			out1 = -1;
#else
			sh4_emit_cmppl(&e->cg, FGL_R_CYCLE);
			out1 = bf_fwd(e);
#endif

			/* SIX BYTES OF DATA PER SITE, BECAUSE `bra` ALMOST
			 * NEVER REACHES.
			 *
			 * SH-4's unconditional branch has a twelve-bit word
			 * displacement, so +/-4 KB.  bloop's blocks are
			 * bump-allocated out of one 1 MB buffer and its census
			 * puts out-of-range edges at 0.1%; lightrec's arena is
			 * a tlsf heap over megabytes and a block is compiled
			 * long before its successor, so the two land wherever
			 * there was room.  Measured here: 97% of edges could
			 * not be reached by a `bra`.
			 *
			 * An edge that cannot be patched is not merely
			 * unimproved, it is WORSE than no linking at all -- it
			 * pays the stub and a whole C call on every execution
			 * instead of going round the dispatcher.  So the far
			 * case has to patch too, and what it patches to needs
			 * an arbitrary 32-bit address:
			 *
			 *      site+0  mov.l @(1,pc), r0   <- reads site+8
			 *      site+2  jmp   @r0
			 *      site+4  nop                 <- same delay slot
			 *      site+6  .word 0             <- pad, target aligns
			 *      site+8  .long <host address>
			 *
			 * Three instructions and one load in place of
			 * eighteen.  The near case still patches to `bra`+`nop`
			 * and leaves these six bytes dead; nothing executes
			 * them either way, because the `bf` above jumps over
			 * the whole site. */
			emit_link_site(e, hoist, t->imm);      /* taken, or the only one */
taken_done:

			if ((fall >= 0 || direct) && FGL_LINK_COND == 2) {
				/* THE FALLTHROUGH IS THE NEXT BASIC BLOCK: the
				 * arm branch lands on it directly instead of on
				 * a `bra` to it.  Only when the local edge would
				 * have been a bare `bra` (forward, no budget
				 * test), and only for the label emitted next,
				 * so the reach is the taken arm and nothing
				 * else -- the same span `out1` already crosses. */
				{
					int l2 = region_find(e, t->imm2);

					if (direct) {
						/* Nothing between here and
						 * the next label: no arm at
						 * all, the `bt` above falls
						 * into it.  A pending pool
						 * or a dispatcher exit would
						 * sit in the way. */
						if (l2 >= 0 && e->lbl[l2].at < 0 &&
						    l2 == e->cur_lbl + 1 &&
						    !e->n_fix && !need_exit &&
						    !(region_dbg() & 3)) {
							slot_barrier(e);
							layout_bt_fall_n++;
							out2 = -1;
							goto fall_done;
						}
					} else if (l2 >= 0 && e->lbl[l2].at < 0 &&
					    l2 == e->cur_lbl + 1 && !e->may_exit &&
					    !(region_dbg() & 1) && !layout_off(1) &&
					    e->n_lref < FGL_MAX_LREFS) {
						e->lref[e->n_lref].site = fall;
						e->lref[e->n_lref].lbl = l2;
						e->lref[e->n_lref].cond = 1;
						e->n_lref++;
						layout_fall_n++;
						slot_barrier(e);
						out2 = -1;
						goto fall_done;
					}
				}
				if (!direct)
					patch_fwd8(e, fall);
				if (region_find(e, t->imm2) >= 0) {
					out2 = emit_local_edge(e, t->imm2, 0);
					goto fall_done;
				}
				if (e->cond_arms && !FGL_ENTRY_HOOK)
					emit_const(e, t->imm2, FGL_R_EXIT); /* fallthrough */
#if !FGL_ENTRY_HOOK
				sh4_emit_cmppl(&e->cg, FGL_R_CYCLE);
#endif
#if FGL_ENTRY_HOOK
				out2 = -1;
#else
				out2 = bf_fwd(e);
#endif
				emit_link_site(e, hoist, t->imm2);     /* the fallthrough */
fall_done:
				;
			}

			if (out1 >= 0) {
				miss[n_miss++] = out1;
				need_exit = 1;
			}
			if (out2 >= 0) {
				miss[n_miss++] = out2;
				need_exit = 1;
			}

			/* Mode 1: the fallthrough arm was never given a site,
			 * so its branch lands here, at the dispatcher, exactly
			 * as it did before linking existed. */
			if (fall >= 0 && FGL_LINK_COND != 2) {
				miss[n_miss++] = fall;
				need_exit = 1;
			}
		}
	}

	/* THE DISPATCHER EXIT, when any path still reaches it.  A basic block
	 * whose every edge is a forward `bra` into its own region has none;
	 * the three words would be dead. */
	/* ONE DISPATCHER EXIT PER BLOCK, when the `bf`s can reach it.
	 *
	 * An exit reached only by budget-miss branches is three words that
	 * every basic block used to carry.  Its `jmp` slot is always a `nop`
	 * -- the branch target is a barrier, so nothing is lifted into it --
	 * which is what makes it the same three words in every basic block
	 * and safe to share.  An exit the code falls into (an unlinkable
	 * transfer) may carry a lifted flush store in that slot and belongs
	 * to its own basic block; it is never the shared one. */
	if (need_exit || (region_dbg() & 2)) {
		int shared = 0, k;

		if (!seq_exit && n_miss && e->exit_at1 && !layout_off(2) &&
		    !(region_dbg() & 2)) {
			int at = e->exit_at1 - 1;

			shared = 1;
			for (k = 0; k < n_miss; k++)
				if (!sh4_disp8_fits(at - miss[k] - 2))
					shared = 0;
			if (shared) {
				for (k = 0; k < n_miss; k++)
					patch_cond8_to(e, miss[k], at);
				layout_share_n++;
				slot_barrier(e);
			}
		}
		if (!shared) {
			int at;

			for (k = 0; k < n_miss; k++)
				patch_fwd8(e, miss[k]);
			at = here(e);
			e->cg.tag = FGL_TAG_EPILOGUE;
			sh4_emit_mov_l_load_gbr(&e->cg, (int)FGL_AT_DISPATCH);
			/* The slot: the last flush store when it went through
			 * r3, or whatever else commutes with the load above.
			 * `nop` when nothing does. */
			emit_delayed(e, SH4_N(0x402b, FGL_R_XFER),
				     1u << FGL_R_XFER);
			if (!seq_exit && n_miss && !(region_dbg() & 2))
				e->exit_at1 = at + 1;
		}
	} else {
		for (k2 = 0; k2 < n_miss; k2++)
			patch_fwd8(e, miss[k2]);
	}

	emit_pool(e);

	if (e->cg.overflow)
		e->overflow = 1;

	return (e->overflow || e->unsupported) ? 0 : entry;
}

uint32_t fgl_emit_block(fgl_emitter *e, const uint32_t *words, uint32_t pc)
{
	ir_node ir[IR_MAX_NODES];
	ir_alloc a;
	int n = ir_decode(words, pc, ir, IR_MAX_NODES);

	if (n <= 0)
		return 0;
	ir_allocate(ir, n, &a);
	return fgl_emit(e, ir, n, &a, (unsigned)ir_block_length(words));
}
