/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * CROSS-BLOCK LINKING ON THE TARGET.
 *
 * Lifted out of fgl_lightrec.c so that the two trees can share that file: on a
 * workstation the "link" is an address in the SH-4 runtime's fiction of the
 * address space and lives with the rest of that fiction (fgl_run.c in the
 * proving ground); here it is a `bra` written into real host code.
 *
 * Everything below is the self-patching link and the undoing of it.  Read the
 * comment on fgl_link_resolve first; the unlink half exists because a patched
 * branch is baked into another block with no indirection left in front of it,
 * so clearing a code-table slot does not reach it.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lightrec-private.h"
#include "blockcache.h"
#include "fgl.h"
#include "fgl_state.h"
#include "fgl_backend.h"

/* ---------------------------------------------------------------- */
/* CROSS-BLOCK LINKING: THE DISPATCH THAT DELETES ITSELF             */
/* ---------------------------------------------------------------- */

/* A BLOCK EXIT IS EIGHTEEN INSTRUCTIONS AND ONLY TWO OF THEM ARE THE JUMP.
 *
 * Leaving a block costs the epilogue -- load the cycle constant, `sub` it off
 * r14, load the dispatch address, `jmp`, `nop` -- and then the dispatcher:
 * publish `curr_pc`, test the budget, ten instructions of branchless block
 * table arithmetic, load the slot, test it, and an INDIRECT jump.  All of that
 * to reach an address that, for a branch with a constant target, was known
 * when the block was compiled.
 *
 * The reason it is not simply branched to is that at compile time the
 * successor usually does not exist yet.  So the exit is emitted as a call to
 * `fgl_link_stub`, and the stub asks this function where the successor went;
 * once there is an answer, the CALL SITE IS REWRITTEN INTO A `bra` and the
 * question is never asked again.  The steady-state edge is `bra` + delay slot,
 * with no memory reference and no indirect branch for the pipeline to stall on
 * -- and the epilogue that is no longer emitted is footprint the icache does
 * not have to carry, which is the measured lever on this machine.
 *
 * WHAT IT COSTS, AND IT IS NOT FREE.  A patched link is a branch baked into
 * another block's code with no indirection left in front of it, so the block
 * table can no longer redirect it.  Selective invalidation therefore stops
 * working, and the only invalidation left is to put every link back.  bloop
 * pays exactly the same price and says so (`bloop/src/blocks.h`): it is a
 * consequence of making the common case free, not an oversight.
 *
 * Hence `fgl_unlink_all`, and hence the rule its callers obey: it runs BEFORE
 * anything frees code or clears a table slot, while the sites are still ours.
 */

/* THE CALL SITE, WHICH IS THREE INSTRUCTIONS AND ONE LONGWORD.
 *
 *      site+0  mov.l @(FGL_AT_LINK,gbr), r0
 *      site+2  jsr   @r0
 *      site+4  <slot>                  <- delay slot; PR is site+6
 *
 * The patch replaces site+0..3 -- the load and the `jsr` -- with a branch and
 * its delay slot.  The `<slot>` at site+4 is NOT a nop any more: the emitter
 * fills it, usually with a register flush store, so the near patch has to copy
 * it down into the `bra`'s own slot at site+2 rather than write a nop there.
 *
 * THE PATCH IS ONE ALIGNED LONGWORD STORE, and it has to be.  A block may be
 * executing this site at the instant it is written: the recompiler thread
 * tears links down from `fgl_unlink_*` while emitted code runs.  An aligned
 * store cannot be caught half-done by the instruction fetcher; two halfword
 * stores can, and the window pairs the old first word with the new second
 * word -- `mov.l @(FGL_AT_LINK,gbr),r0` leaving the STUB address in r0,
 * followed by `jmp @r0`, which does not set PR, so the stub derives its site
 * from a stale PR and patches whatever that names.  That was a segfault in
 * emitted code at a garbage address, thirteen thousand cycles after boot.
 *
 * So the emitter pads every site to four bytes (SH4_LINK_SITE_ALIGN in
 * fgl.h), one `nop` in half the sites, and this file may assume it. */
#define FGL_LINK_SITE_W0  ((uint16_t)(0xc600u | FGL_AT_LINK))   /* mov.l @(d,gbr),r0 */
#define FGL_LINK_SITE_W1  ((uint16_t)0x400bu)                   /* jsr @r0           */

/* Little-endian: the halfword at the lower address is the low half. */
#define FGL_PACK(lo, hi)  (((uint32_t)(uint16_t)(hi) << 16) | (uint16_t)(lo))

#define FGL_LINK_SITE_WORD FGL_PACK(FGL_LINK_SITE_W0, FGL_LINK_SITE_W1)

/* Every site this run has patched, so that they can be put back.
 *
 * A fixed array and not a list: it is walked in full or not at all, it is
 * never searched, and an allocation on the compile path is a failure mode
 * this does not need.
 *
 * RUNNING OUT IS WORSE THAN IT LOOKS.  A site that cannot be recorded cannot
 * be patched, and an unpatched site pays the stub AND a C call on every single
 * execution -- strictly worse than never having linked it.  It is not a
 * graceful degradation, it is a cliff.  So this wants to be comfortably above
 * the live edge count, and `fgl_link_patched + fgl_link_range` is the number
 * to check it against: the last steady-state census was ~1160.
 *
 * It was briefly 16384, raised in the same change that added conditional
 * linking, on the guess that the edge count would triple.  Two variables at
 * once and the build faulted; this went back first because it was the guess.
 *
 * AND THE GUESS WAS RIGHT, JUST UNMEASURED AT THE TIME.  4096 is what the
 * census counted before conditional linking and before the backward scan; now
 * every direct branch gets a site and the live set runs well past it.  What
 * kept the old table on the cliff was not only its size but the global sweep
 * on every free, which emptied and refilled it: 253,424 links torn down in a
 * 600-vsync run against 1,156 once teardown became block-scoped.  With the
 * sweep gone the table holds the live set instead of a churn of dead ones.
 *
 * NOT 65536.  Two uint32_t arrays of that length is 512 KiB of bss, and with
 * it Spyro died at `sbrk` during boot -- "Unable to allocate memory", then
 * "Unable to recompile block".  The link table competes with the code heap for
 * the same 16 MiB, so it is bounded by what is left over.  16384 is 128 KiB.
 *
 * `full` in the links report is the refusal count and is the number to watch:
 * if it is not zero, this is too small and the emulator is on the cliff. */
#define FGL_MAX_LINKS 16384
static uint32_t fgl_link_site[FGL_MAX_LINKS];

/* WHICH TABLE SLOT EACH SITE BRANCHES AT, so that a teardown can be about the
 * code that actually died.
 *
 * A site is stored with `lut_offset()` of its successor rather than the guest
 * PC, because that is the unit the invalidation paths work in: they clear
 * `((len + 3) / 4)` slots starting at `lut_offset(addr)`, so a site is stale
 * exactly when its index falls in that window.  Comparing indices makes the
 * two agree by construction instead of by both getting the segment masking
 * right separately. */
static uint32_t fgl_link_slot[FGL_MAX_LINKS];

/* The guest PC each site branches to, kept so that a teardown can put it back
 * in the site's literal -- see `fgl_link_restore`. */
static uint32_t fgl_link_pc[FGL_MAX_LINKS];
static unsigned fgl_n_links;

/* WHERE A SITE KEEPS ITS LITERAL, AND WHY IT IS NOT ALWAYS site+8.
 *
 * `mov.l @(1,PC),R0` reads `((site + 4) & ~3) + 4`: site+8 from a 4-aligned
 * site, site+6 from a 2-aligned one.  The emitter lays the data out to match
 * and pads only when it has to (emit.c, emit_link_site), so a 2-aligned site
 * is one word SHORTER, not one longer.
 *
 * THIS IS THE FUNCTION THAT WAS MISSING.  The patcher this file was extracted
 * from predates that emitter and assumed every site was 4-aligned, refusing
 * `site & 3` outright -- silently, with no counter on the path.  Sites are
 * emitted at both alignments, so roughly every other edge in the program was
 * refused and paid a C call through the stub on every single execution,
 * forever.  That is worse than not linking at all, and it is what "on par with
 * base lightrec" was measuring. */
static uint32_t fgl_link_lit(uint32_t site)
{
	return site + ((site & 2) ? 6u : 8u);
}

/* A SITE MAY BE 2-ALIGNED, so its two instruction words are read as halfwords.
 * A longword access to a two-mod-four address is a data address error on SH-4
 * -- an x86 host tolerates it, which is why the tree this came from could use
 * a plain longword and this cannot. */
static uint32_t fgl_site_rd(uint32_t site)
{
	const volatile uint16_t *p = (const volatile uint16_t *)(uintptr_t)site;

	return FGL_PACK(p[0], p[1]);
}

static void fgl_site_inv(struct lightrec_state *state, uint32_t site)
{
	if (state->ops.code_inv)
		state->ops.code_inv((void *)(uintptr_t)site, 4);
}

/* HOW MANY LINKS POINT INTO EACH PAGE OF THE TABLE.
 *
 * An invalidation clears two slots and then asked "is any of my 1400 links
 * one of these two?" by walking all 1400.  Spyro does that 213 times a frame
 * and the answer is no every single time: 358k entries visited a frame, 118 ms
 * of it, which was the whole of the `rw` bucket and most of the frame.
 *
 * A count per 1024 slots turns the common answer into one load.  The walk is
 * still there for when a page really does hold links -- this only skips the
 * sweeps that were always going to find nothing.  Exact, not a filter: the
 * count is incremented where a site enters the table and decremented where one
 * leaves, so a zero means zero. */
#define FGL_LINK_PAGE_SHIFT 10
#define FGL_LINK_PAGES ((CODE_LUT_SIZE >> FGL_LINK_PAGE_SHIFT) + 1u)
static uint16_t fgl_link_pgcnt[FGL_LINK_PAGES];

static int fgl_links_in_range(u32 first, u32 last)
{
	u32 p = first >> FGL_LINK_PAGE_SHIFT;
	u32 e = (last - 1u) >> FGL_LINK_PAGE_SHIFT;

	if (e >= FGL_LINK_PAGES)
		e = FGL_LINK_PAGES - 1u;
	for (; p <= e; p++)
		if (fgl_link_pgcnt[p])
			return 1;
	return 0;
}

/* Edges refused because the table was full.  Nonzero means FGL_MAX_LINKS is
 * too small and the emulator is on the cliff described above. */
unsigned fgl_link_full;

/* DIAGNOSTIC: how the edges came out, printed by nothing yet but readable
 * from a debugger and cheap to keep. */
unsigned fgl_link_patched, fgl_link_uncompiled, fgl_link_range, fgl_link_undone;
unsigned fgl_link_calls, fgl_link_bad_site;

/* `calls` IS THE ONE THAT SAYS WHETHER THIS IS WORKING.  A patched edge never
 * comes back here, so in a machine that is linking well the call count goes
 * quiet while the frame counter keeps moving; a call count that tracks the
 * frame rate means the links are being torn down as fast as they are made.
 *
 * Nothing prints them -- they are read from a debugger, or by a printf added
 * for one run.  A per-second report was here and was noise once the mechanism
 * was known to work. */

unsigned fgl_unlink_calls[LIGHTREC_UNLINK_N];
unsigned fgl_unlink_links[LIGHTREC_UNLINK_N];

static void fgl_link_restore(struct lightrec_state *state, uint32_t site,
			     uint32_t pc)
{
	uint32_t lit = fgl_link_lit(site);

	/* One aligned store: the fetcher cannot see this half-done, which is
	 * what SH4_LINK_SITE_ALIGN exists to guarantee (emit.c). */
	*(volatile uint32_t *)(uintptr_t)site = FGL_LINK_SITE_WORD;
	fgl_site_inv(state, site);

	/* THE LITERAL LAST, and not first as the x86 tree does it.  A far
	 * patch left a host address there; writing the guest pc in before the
	 * site is restored leaves a window where the live far patch loads a
	 * GUEST address and jumps to it.  Once the site is a site again
	 * nothing reads the literal until the stub runs. */
	*(volatile uint32_t *)(uintptr_t)lit = pc;
	fgl_site_inv(state, lit);
}

void fgl_unlink_all(struct lightrec_state *state, unsigned why)
{
	unsigned i;

	fgl_unlink_calls[why]++;
	fgl_unlink_links[why] += fgl_n_links;

	if (!fgl_n_links)
		return;

	for (i = 0; i < fgl_n_links; i++)
		fgl_link_restore(state, fgl_link_site[i], fgl_link_pc[i]);

	fgl_link_undone += fgl_n_links;
	fgl_n_links = 0;
	memset(fgl_link_pgcnt, 0, sizeof fgl_link_pgcnt);
}

/* THE SAME, FOR ONE WINDOW OF THE TABLE -- WHICH IS ALMOST ALWAYS WHAT WAS
 * MEANT.
 *
 * `lightrec_invalidate` clears `n` slots and nothing else, and then called
 * `fgl_unlink_all`, which put back every patched site in the program.  A four
 * byte guest store threw away the whole link graph.  Measured on Spyro: 7992
 * invalidations destroyed 251516 links, 99.7% of all the teardown in the run,
 * while the other five callers together managed 688.
 *
 * A site is stale when the slot it branches at is one of the cleared ones, and
 * no other site is affected, so only that window is torn down and the rest of
 * the table is compacted down over the holes.  Order within the table means
 * nothing -- it is walked whole or not at all -- so compacting is just a
 * second index.
 *
 * FREEING CODE IS THE OTHER CASE, and it is `fgl_unlink_block` below.  This
 * one restores sites, which means writing to them, so it is only sound while
 * every site in the table is still live memory. */
void fgl_unlink_range(struct lightrec_state *state, u32 first, u32 n,
		      unsigned why)
{
	unsigned i, keep = 0, hit = 0;
	u32 last = first + n;

	fgl_unlink_calls[why]++;

	if (!n || !fgl_links_in_range(first, last))
		return;

	for (i = 0; i < fgl_n_links; i++) {
		if (fgl_link_slot[i] >= first && fgl_link_slot[i] < last) {
			fgl_link_restore(state, fgl_link_site[i],
					 fgl_link_pc[i]);
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		fgl_link_site[keep] = fgl_link_site[i];
		fgl_link_slot[keep] = fgl_link_slot[i];
		fgl_link_pc[keep] = fgl_link_pc[i];
		keep++;
	}

	fgl_n_links = keep;
	fgl_unlink_links[why] += hit;
	fgl_link_undone += hit;
}

/* ONE BLOCK'S CODE IS ABOUT TO BE HANDED BACK TO THE ARENA.
 *
 * Two different populations die with it, and they need opposite treatment:
 *
 *   - Sites POINTING AT it.  Their target slot is inside [first, first + n),
 *     they live in some other block that is still perfectly good memory, and
 *     they must be rewritten back into a call to the stub.  Same as a range
 *     unlink.
 *
 *   - Sites LIVING IN it, anywhere in [code, code + code_size).  These must be
 *     forgotten WITHOUT being touched.  Restoring one means storing into
 *     memory that is being freed this instant, and a later teardown that still
 *     had it in the table would store into whatever the arena handed out next.
 *     That is the hazard the old global sweep avoided by running before every
 *     free and taking the entire table with it.
 *
 * A site can be in both sets -- a block that branches to itself -- and the
 * living-in test wins, because not writing is always safe and writing into
 * freed memory never is.
 *
 * MUST RUN BEFORE THE FREE.  The first population is rewritten here and the
 * addresses have to still be ours. */
void fgl_unlink_block(struct lightrec_state *state, u32 first, u32 n,
		      void *fn, unsigned code_size, unsigned why)
{
	unsigned i, keep = 0, hit = 0;
	uintptr_t code = (uintptr_t)fn;
	u32 last = first + n;

	fgl_unlink_calls[why]++;

	for (i = 0; i < fgl_n_links; i++) {
		uintptr_t site = (uintptr_t)fgl_link_site[i];

		/* Living in the dying code: drop it, do not write it. */
		if (code_size && site >= code && site < code + code_size) {
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		/* Pointing at the dying code: put it back. */
		if (fgl_link_slot[i] >= first && fgl_link_slot[i] < last) {
			fgl_link_restore(state, fgl_link_site[i],
					 fgl_link_pc[i]);
			fgl_link_pgcnt[fgl_link_slot[i]
					>> FGL_LINK_PAGE_SHIFT]--;
			hit++;
			continue;
		}

		fgl_link_site[keep] = fgl_link_site[i];
		fgl_link_slot[keep] = fgl_link_slot[i];
		fgl_link_pc[keep] = fgl_link_pc[i];
		keep++;
	}

	fgl_n_links = keep;
	fgl_unlink_links[why] += hit;
	fgl_link_undone += hit;
}

/* Called by `fgl_link_stub` with the guest PC the block is leaving for and the
 * address of the call site that asked.  Returns the host address to enter, or
 * 0 to mean "go round the dispatcher this time" -- which is what happens while
 * the successor is not compiled yet, and is why the stub is a call and not a
 * one-shot resolver.
 *
 * The answer is only PATCHED IN when it is one a `bra` can reach.  SH-4's
 * unconditional branch has a twelve-bit word displacement, so +/-4 KB; the
 * code arena is megabytes, so out-of-range successors are ordinary and they
 * keep the call.  They are not a bug and not worth a second mechanism: the
 * call is what the edge cost before linking existed.
 *
 * IT REFUSES TO LINK TO THE COMPILE SENTINEL.  Under the threaded compiler a
 * table slot can hold `get_next_block` to mean "seen, not compiled yet"
 * (lightrec.c, `lightrec_precompile_block`).  That is a legal thing for the
 * DISPATCHER to jump to, because the dispatcher asks again next time -- but
 * baking it into a `bra` would freeze the edge on the sentinel for ever and
 * the real block would never be reached. */
static u32 fgl_link_resolve_inner(struct lightrec_state *state, u32 target,
				  u32 site);

/* Reached from emitted code through `fgl_link_stub`, so it is another door
 * out.  Wrapped rather than bracketed inline because it has several returns. */
u32 fgl_link_resolve(struct lightrec_state *state, u32 target, u32 site)
{
	u32 r;

#if defined(FGL_NO_PATCH) && FGL_NO_PATCH
	/* SITES EMITTED, NEVER PATCHED.  The measured pair for "what does
	 * linking actually buy": every code shape is byte-identical to the
	 * linked build, but each edge goes round the stub and this call for
	 * ever instead of becoming a `bra` or a `jmp`.  Unlike -DFGL_LINK=0,
	 * which deletes the sites and takes several long-untested epilogue
	 * paths with it, this touches nothing the tested build does not
	 * already do. */
	(void)site;
	return (u32)(uintptr_t)lut_read(state, lut_offset(target));
#else
	r = fgl_link_resolve_inner(state, target, site);

	return r;
#endif
}

static u32 fgl_link_resolve_inner(struct lightrec_state *state, u32 target,
				  u32 site)
{
	void *slot = lut_read(state, lut_offset(target));
	int32_t d;

	fgl_link_calls++;

	if (!slot) {
		fgl_link_uncompiled++;
		return 0;
	}

	/* A TABLE SLOT NEED NOT BE CODE FGL EMITTED.  lightrec parks four
	 * sentinels in slots -- `get_next_block` for "seen, not compiled yet",
	 * `memset_func` for a recognised memset loop, and the interpreter and
	 * delay-slot doors -- and a `bra` baked at one would freeze the edge on
	 * it forever, because the dispatcher's "ask again next time" is what
	 * resolves it.  Only the compile sentinel was being caught here. */
	if (slot == (void *)(uintptr_t)state->memset_func ||
	    slot == (void *)(uintptr_t)state->interpreter_func ||
	    slot == (void *)(uintptr_t)state->ds_check_func) {
		fgl_link_uncompiled++;
		return (u32)(uintptr_t)slot;
	}

	if (slot == (void *)(uintptr_t)state->get_next_block) {
		/* THE SENTINEL IS AN ENTRY POINT, AND IT READS `curr_pc`.
		 *
		 * The stub `jmp`s straight to whatever comes back, so returning
		 * this sends the edge into `_fgl_dispatch_compile` -- which
		 * takes the guest PC out of the state block, because the
		 * dispatcher's own route there has already destroyed r2 working
		 * out the table index.  The loop publishes on its way in; a
		 * linked edge never passes through the loop, so without this
		 * the compile would run on whichever block last went the long
		 * way round and enter the wrong one, silently.
		 *
		 * Here rather than in the stub because this is the only branch
		 * that needs it: every other return is either 0, which goes
		 * round the loop, or a real block entry, which is entered with
		 * r2 and needs nothing. */
		state->curr_pc = target;
		fgl_link_uncompiled++;
		return (u32)(uintptr_t)slot;
	}

	if (fgl_n_links >= FGL_MAX_LINKS) {
		fgl_link_full++;
		return (u32)(uintptr_t)slot;
	}

	/* SH4_LINK_SITE_ALIGN (emit.c) pads every site to four bytes so the
	 * patch below is one aligned store.  If one ever arrives unaligned the
	 * emitter and this file have drifted apart again -- say so rather than
	 * refusing in silence, which is how the old 4-aligned-only patcher hid
	 * the fact that it was refusing half the edges in the program. */
	if (site & 3) {
		if (fgl_link_bad_site++ < 20)
			fprintf(stderr, "fgl: link site %08x is not "
				"4-aligned\n", (unsigned)site);
		return (u32)(uintptr_t)slot;
	}

	/* THE SITE HAS TO STILL BE A SITE BEFORE ANYTHING IS WRITTEN TO IT.
	 *
	 * `site` is not passed in from anywhere trustworthy -- the stub derives
	 * it from PR, as `PR - 6`, which is only the call site if the emitter
	 * laid the site out exactly as the stub assumes.  If those two ever
	 * disagree, the two stores below write a `bra` and a `nop` into
	 * WHATEVER `site` happens to name.  The code arena and bloom's own
	 * .text are both writable, so the likely outcome is not a fault here
	 * but silently corrupted C, faulting somewhere unrelated much later --
	 * which is exactly the shape of the boot fault that prompted this.
	 *
	 * An unpatched site is two known halfwords and nothing else can be
	 * mistaken for them, so checking is one load and one compare.  Refusing
	 * costs the edge and nothing more; the dispatcher still runs.
	 *
	 * An ALREADY-patched site reaching here would also fail this test, and
	 * that is equally a bug: a site is recorded when it is patched and only
	 * an unpatched one can call the stub. */
	if (fgl_site_rd(site) != FGL_LINK_SITE_WORD) {
		/* Capped: if this fires at all it is likely to fire on every
		 * edge, and a flood over dc-load is slower than the fault. */
		if (fgl_link_bad_site++ < 20)
			fprintf(stderr,
				"fgl: link site %08x is not a link site: %08x\n",
				(unsigned)site, (unsigned)fgl_site_rd(site));
		return (u32)(uintptr_t)slot;
	}

	/* `sh4_branch_disp12` counts from the branch's own address, which is
	 * site+0 -- the load is what the `bra` replaces, not what it follows. */
	d = sh4_branch_disp12(site, (uintptr_t)slot);

	if (sh4_disp12_fits(d)) {
		/* NEAR: the `bra` lands on site+0 so its delay slot is site+2,
		 * where the `jsr` was -- NOT site+4, where the emitter's slot
		 * instruction is.  So that instruction is COPIED down into it;
		 * site+4 is dead from here on, like the literal bytes.
		 *
		 * Writing a `nop` there instead, which is what this did, drops
		 * the instruction: it was a `nop` before the emitter started
		 * filling slots and is usually a register flush store now
		 * (emit.c, emit_link_site), so the patch silently skipped a
		 * store the block had already accounted for. */
		uint16_t dslot = *(volatile uint16_t *)(uintptr_t)(site + 4);

		*(volatile uint32_t *)(uintptr_t)site =
			FGL_PACK(SH4_D12(0xa000, d), dslot);
		fgl_link_patched++;
	} else {
		/* FAR: the address goes in the dead bytes the emitter left
		 * behind the site, and the site becomes a PC-relative load and
		 * an indirect jump.  `mov.l @(disp,PC),R0` reads
		 * `(PC & ~3) + disp * 4` with PC = site+4, so disp 1 names
		 * site+8 from a 4-aligned site and site+6 from a 2-aligned
		 * one -- `fgl_link_lit`, which is also where the emitter put
		 * the data.
		 *
		 * THE LITERAL IS WRITTEN FIRST.  Until the instruction pair
		 * below lands, the literal is data nothing reads; after it
		 * lands it is the jump's target.  The other order gives the
		 * fetcher a jump through whatever was there. */
		uint32_t lit = fgl_link_lit(site);

		*(volatile uint32_t *)(uintptr_t)lit =
			(uint32_t)(uintptr_t)slot;
		fgl_site_inv(state, lit);

		*(volatile uint32_t *)(uintptr_t)site =
			FGL_PACK(0xd001u /* mov.l @(1,pc),r0 */,
				 0x402bu /* jmp @r0 */);
		fgl_link_range++;
	}

	fgl_site_inv(state, site);

	fgl_link_slot[fgl_n_links] = lut_offset(target);
	fgl_link_pc[fgl_n_links] = target;
	fgl_link_site[fgl_n_links++] = site;
	fgl_link_pgcnt[lut_offset(target) >> FGL_LINK_PAGE_SHIFT]++;

	return (u32)(uintptr_t)slot;
}
