/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Guest-op attribution.  See attr.h for why this exists and what it cannot
 * see; this file is only the bookkeeping.
 */

#include "attr.h"

#if BLOOM_ATTR

#include "blockcache.h"
#include "regcache.h"
#include "lightrec-private.h"

#include <lightning.h>
#include <stdio.h>
#include <string.h>

#ifndef BLOOM_ATTR_MB
#  define BLOOM_ATTR_MB 2
#endif

#define ATTR_BYTES  ((size_t)BLOOM_ATTR_MB * 1024 * 1024)
#define ATTR_MAX    ((ATTR_BYTES - sizeof(struct lightrec_attr_hdr)) \
		     / sizeof(struct lightrec_attr_rec))

/* One static buffer, so its address is fixed for the whole run and can be
 * named on a `-jitdump-region` command line.  Aligned to a cache line for no
 * reason beyond keeping the header off a shared line. */
static union {
	struct {
		struct lightrec_attr_hdr hdr;
		struct lightrec_attr_rec rec[ATTR_MAX];
	};
	char raw[ATTR_BYTES];
} attr_buf __attribute__((aligned(32)));

void lightrec_attr_init(void)
{
	memset(&attr_buf, 0, sizeof attr_buf);
	memcpy(attr_buf.hdr.magic, LIGHTREC_ATTR_MAGIC, 8);
	attr_buf.hdr.version = LIGHTREC_ATTR_VERSION;
	attr_buf.hdr.rec_size = sizeof(struct lightrec_attr_rec);
	attr_buf.hdr.hdr_size = sizeof(struct lightrec_attr_hdr);

	/* The address is what the dump command needs, and it is not knowable
	 * from outside the binary, so it is printed rather than documented --
	 * a documented address goes stale the first time the link order
	 * changes and then silently dumps something else. */
	printf("ATTR buffer at 0x%08x, %u bytes, capacity %u records\n"
	       "  ./testcast -jitdump-region 0x%08x:0x%x "
	       "-jitdump-no-entries -jitdump-out attr bloom.elf\n",
	       (unsigned)(uintptr_t)&attr_buf, (unsigned)ATTR_BYTES,
	       (unsigned)ATTR_MAX,
	       (unsigned)(uintptr_t)&attr_buf, (unsigned)ATTR_BYTES);
}

void lightrec_attr_set_code_region(const void *base, unsigned int size)
{
	attr_buf.hdr.code_base = (u32)(uintptr_t)base;
	attr_buf.hdr.code_size = size;
}

void lightrec_attr_block_start(struct lightrec_cstate *state)
{
	state->attr_nb = 0;
}

void lightrec_attr_mark(struct lightrec_cstate *state,
			const struct block *block, u16 offset)
{
	struct lightrec_attr_pending *p;
	jit_state_t *_jit = block->_jit;

	if (state->attr_nb >= LIGHTREC_ATTR_MAX_OPS) {
		attr_buf.hdr.dropped++;
		return;
	}

	p = &state->attr_pending[state->attr_nb++];
	p->guest_pc = block->pc + ((u32)offset << 2);
	p->guest_opcode = block->opcode_list[offset].c.opcode;
	p->live_mask = lightrec_regcache_live_mask(state->reg_cache);

	/* A note, never a label: a label flushes the instruction buffer and
	 * resets FPU mode, which would change the code being measured. */
	p->note = jit_note(NULL, 0);
}

void lightrec_attr_mark_epilogue(struct lightrec_cstate *state,
				 const struct block *block)
{
	struct lightrec_attr_pending *p;
	jit_state_t *_jit = block->_jit;

	if (state->attr_nb >= LIGHTREC_ATTR_MAX_OPS) {
		attr_buf.hdr.dropped++;
		return;
	}

	p = &state->attr_pending[state->attr_nb++];
	p->guest_pc = LIGHTREC_ATTR_EPILOGUE;
	p->guest_opcode = 0;
	p->live_mask = lightrec_regcache_live_mask(state->reg_cache);
	p->note = jit_note(NULL, 0);
}

void lightrec_attr_block_end(struct lightrec_cstate *state,
			     const struct block *block, jit_state_t *_jit,
			     void *fn, unsigned int size)
{
	unsigned int i;

	if (!state->attr_nb)
		return;

	for (i = 0; i < state->attr_nb; i++) {
		struct lightrec_attr_pending *p = &state->attr_pending[i];
		struct lightrec_attr_rec *r;

		if (attr_buf.hdr.nb_records >= ATTR_MAX) {
			attr_buf.hdr.dropped++;
			continue;
		}

		/* A NOTE CAN COME BACK WITH NO ADDRESS.  `jit_address()` reads
		 * `node->u.w`, which the emit loop assigns when it walks the
		 * node; a node lightning dropped as unreachable is never
		 * walked and keeps its initial zero.  Recording that as a host
		 * address makes the NEXT op's range start at 0 and run to
		 * wherever this one landed -- one such record produced a
		 * single `sw` row of 1,181,256,056 host instructions on the
		 * first run, which is 3.6 million per site and obvious only
		 * because it is absurd.  Count them and drop them. */
		if (!jit_address(p->note)) {
			attr_buf.hdr.unresolved++;
			continue;
		}

		r = &attr_buf.rec[attr_buf.hdr.nb_records++];
		r->guest_pc = p->guest_pc;
		r->guest_opcode = p->guest_opcode;
		r->host_start = (u32)(uintptr_t)jit_address(p->note);
		r->live_mask = p->live_mask;
		r->block_pc = block->pc;
	}

	/* THE TERMINATOR IS NOT OPTIONAL.  An op's host range ends where the
	 * next one starts, so without a record marking the end of the block
	 * the last op of every block has no end -- and the last op of a block
	 * is the branch, which is exactly the row the delay-slot arguments
	 * keep turning on. */
	if (attr_buf.hdr.nb_records < ATTR_MAX) {
		struct lightrec_attr_rec *r =
			&attr_buf.rec[attr_buf.hdr.nb_records++];

		r->guest_pc = 0;
		r->guest_opcode = 0;
		r->host_start = (u32)(uintptr_t)fn + size;
		r->live_mask = 0;
		r->block_pc = block->pc;
	} else {
		attr_buf.hdr.dropped++;
	}

	state->attr_nb = 0;
}

#endif /* BLOOM_ATTR */
