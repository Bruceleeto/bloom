/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Direct block linking.
 *
 * A block whose exit target is known at compile time ends in a jump through
 * a literal word in its own code (mov.l @(pc),r; jmp @r). That word holds
 * the target block's entry address when the target is compiled, or the
 * dispatcher's eob_wrapper otherwise. Links mirror the code LUT: every
 * write to the LUT for a target offset rewrites the words of the edges that
 * point at it, so a linked edge is never taken into invalidated code.
 *
 * The word lives in the literal pool, which SH-4 reads through the data
 * path, so linking and unlinking is a plain 32-bit store - no icache flush.
 */

#ifndef __LIGHTREC_LINKS_H__
#define __LIGHTREC_LINKS_H__

#include "lightrec-private.h"

/* Per-edge placeholder value emitted into the literal pool. Both halves
 * decode as "mac.l @Rm+,@R15+", which lightning never emits, so a scan of
 * the finished block for the word is unambiguous. Up to 256 exits per
 * block. */
#define LINK_MAGIC(i) (0x0F0F0F0Fu | ((u32)((i) & 0xf0) << 16) | ((u32)((i) & 0x0f) << 4))

struct lightrec_link {
	u32 offset;			/* LUT offset of the target */
	u32 *word;			/* literal in the owner's code */
	struct lightrec_link *next_target;	/* hash chain by offset */
	struct lightrec_link *next_owner;	/* the owner's list */
};

struct lightrec_links * lightrec_links_init(struct lightrec_state *state);
void lightrec_links_destroy(struct lightrec_state *state);

/* Resolve the pending exits of a freshly emitted block: find each magic
 * word in [function, function + block->code_size), register the edge, and
 * write the word's initial value. The list of links goes into block->links.
 * `function` is passed explicitly so this runs before block->function is
 * published: the threaded compiler's main thread enters new code as soon
 * as it sees the pointer (recompiler.c, run_first_pass). */
int lightrec_links_register_block(struct lightrec_state *state,
				  struct block *block, void *function,
				  const struct lightrec_pending_link *pending,
				  unsigned int nb_pending);

/* Drop a list of links (the owner's code is going away). Does not touch
 * the words. */
void lightrec_links_free_list(struct lightrec_state *state,
			      struct lightrec_link *list);

/* LUT mirror hooks. */
void lightrec_links_lut_changed(struct lightrec_state *state,
				u32 offset, void *ptr);
void lightrec_links_lut_cleared(struct lightrec_state *state,
				u32 offset, u32 count);
void lightrec_links_lut_cleared_all(struct lightrec_state *state);

#endif /* __LIGHTREC_LINKS_H__ */
