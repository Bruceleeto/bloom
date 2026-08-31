/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Guest-op attribution: which host bytes came from which MIPS instruction.
 *
 * WHY THIS EXISTS.  We can measure bloom's emitted code by byte pattern -- how
 * many reg-reg moves, how many memory refs -- but not by guest opcode.  Nothing
 * says "a MIPS LW costs bloom 14 host instructions and bleem 6".  Four straight
 * changes were mispriced because a byte-pattern bucket turned out to be several
 * unrelated things sharing an encoding (`build/docs/attribution_spec.md`).
 *
 * HOW IT MEASURES WITHOUT DISTURBING.  Host addresses do not exist while an op
 * is being lowered: lightrec builds lightning IR and lightning assigns
 * addresses at `jit_emit()`.  So each op gets a `jit_note` marker, and the
 * markers are resolved to addresses afterwards with `jit_address()`.
 *
 * A NOTE IS THE ONLY MARKER THAT IS FREE.  In `jit_sh.c` a `jit_code_note`
 * records the pc and nothing else; a `jit_code_label` flushes the instruction
 * buffer and resets FPU mode, which CHANGES THE CODE BEING MEASURED.  For an
 * instrument that is the difference between a measurement and a fiction.
 *
 * THE KNOWN INACCURACY, stated up front: the SH-4 backend buffers instructions
 * to fill delay slots, so an instruction can move across a note into a
 * preceding delay slot.  Attribution is therefore exact per block and
 * approximate at op boundaries where a transfer sits.  It cannot be otherwise
 * without pinning the delay slot, which would change the code.
 *
 * OUTPUT.  There is no filesystem on the target, so the table lives in memory
 * with its own header and is dumped like everything else:
 *
 *     ./testcast -jitdump-region <addr printed at boot>:<size> \
 *                -jitdump-no-entries -jitdump-out attr bloom.elf
 *
 * Build with -DBLOOM_ATTR=1.  It is a measurement build and does not ship:
 * the buffer alone is megabytes.
 */

#ifndef __LIGHTREC_ATTR_H__
#define __LIGHTREC_ATTR_H__

#ifndef BLOOM_ATTR
#  define BLOOM_ATTR 0
#endif

/* Deliberately NOT lightrec-private.h: that header carries the compile state,
 * which has to embed the pending list declared below, and including it here
 * would close the loop.  Forward declarations are enough. */
#include <lightning.h>

#include "lightrec.h"

struct block;
struct lightrec_cstate;

/* Records, in the order they were compiled.  `host_start` of the NEXT record
 * is this one's end -- which is why every block emits a terminator whose
 * `guest_pc` is 0, so the last op of a block has an end too. */
/* Two guest_pc values are sentinels, not addresses: 0 ends a block, and
 * LIGHTREC_ATTR_EPILOGUE separates the last guest op from the block's own
 * scaffolding -- the out-of-line LUT entry stubs, the local-branch patching,
 * the link stub and the return.  Without it that scaffolding is charged to
 * whichever op happened to be last, which is always the branch: `jr` measured
 * 54.2 host instructions a site, and almost none of that is the jump. */
#define LIGHTREC_ATTR_EPILOGUE 0xffffffffu

struct lightrec_attr_rec {
	u32 guest_pc;		/* MIPS PC, 0 = block end, ~0 = epilogue     */
	u32 guest_opcode;	/* the raw MIPS word                         */
	u32 host_start;		/* first host byte of this op                */
	u32 live_mask;		/* bit N: SH-4 rN holds a live value here    */
	u32 block_pc;		/* the block this belongs to                 */
};

/* THE OPCODE IS CARRIED, NOT LOOKED UP.  It could be recovered offline by
 * decoding guest RAM at `guest_pc`, which would save four bytes a record -- and
 * would mean every reading of this table depended on having the matching RAM
 * dump from the same run.  That is the failure that has already cost this
 * project a day: `build/bloom_jd.bin` and `build/prof1.prof` are from different
 * runs and every row computed from the pair was wrong.  Self-contained wins. */

#define LIGHTREC_ATTR_MAGIC   "SH4ATTR1"
#define LIGHTREC_ATTR_VERSION 2

/* THE CODE REGION IS CARRIED, for the same reason the opcode is.  A reader has
 * to know how much live code exists to say what FRACTION of it this table
 * covers -- a 34% share of an unstated 8% of the buffer is a confident wrong
 * number, and that is the failure this whole exercise exists to end.  The
 * region cannot be a flag: it moved from 0x8cd00000 to 0x8dd00000 the moment
 * testcast went from 16 to 32 MiB of RAM, and a stale literal in a script says
 * nothing when it goes wrong. */
struct lightrec_attr_hdr {
	char magic[8];
	u32 version;
	u32 nb_records;		/* records actually written                  */
	u32 rec_size;		/* sizeof(struct lightrec_attr_rec)          */
	u32 dropped;		/* records lost to a full buffer, 0 is clean */
	u32 unresolved;		/* markers lightning never gave an address   */
	u32 hdr_size;		/* bytes before the first record             */
	u32 code_base;		/* the JIT code buffer this describes        */
	u32 code_size;
	u32 pad[2];		/* header is 48 bytes                        */
};

#if BLOOM_ATTR

/* Per block, at most this many pending markers.  A block that exceeds it stops
 * recording and counts the loss rather than growing: a measurement that
 * silently reallocates is a measurement of a different program. */
#define LIGHTREC_ATTR_MAX_OPS 1024

struct lightrec_attr_pending {
	u32 guest_pc;
	u32 guest_opcode;
	u32 live_mask;
	jit_node_t *note;
};

/* Called once, from lightrec_init: prints the buffer address to dump. */
void lightrec_attr_init(void);

/* The JIT code buffer, recorded so a reader can state coverage.  Separate from
 * init() because lightrec_init() reads the map after it. */
void lightrec_attr_set_code_region(const void *base, unsigned int size);

/* Marker for one guest op, placed before it is lowered. */
void lightrec_attr_mark(struct lightrec_cstate *state,
			const struct block *block, u16 offset);

/* Marker for the start of the block's own scaffolding. */
void lightrec_attr_mark_epilogue(struct lightrec_cstate *state,
				 const struct block *block);

/* Block boundaries: reset the pending list, then resolve it once the code has
 * an address. */
void lightrec_attr_block_start(struct lightrec_cstate *state);
void lightrec_attr_block_end(struct lightrec_cstate *state,
			     const struct block *block, jit_state_t *_jit,
			     void *fn, unsigned int size);

#else

static inline void lightrec_attr_init(void) {}
static inline void lightrec_attr_set_code_region(const void *base,
						 unsigned int size) {}
static inline void lightrec_attr_mark(struct lightrec_cstate *state,
				      const struct block *block, u16 offset) {}
static inline void lightrec_attr_mark_epilogue(struct lightrec_cstate *state,
					       const struct block *block) {}
static inline void lightrec_attr_block_start(struct lightrec_cstate *state) {}
static inline void lightrec_attr_block_end(struct lightrec_cstate *state,
					   const struct block *block,
					   jit_state_t *_jit,
					   void *fn, unsigned int size) {}

#endif /* BLOOM_ATTR */

#endif /* __LIGHTREC_ATTR_H__ */
