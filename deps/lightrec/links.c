// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Direct block linking - see links.h.
 */

#include "debug.h"
#include "lightrec-config.h"
#include "lightrec-private.h"
#include "links.h"
#include "memmanager.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if ENABLE_THREADED_COMPILER
#include <pthread.h>
#endif

#define LINKS_HASH_BITS	12
#define LINKS_HASH_SIZE	(1u << LINKS_HASH_BITS)

struct lightrec_links {
#if ENABLE_THREADED_COMPILER
	pthread_mutex_t mutex;
#endif
	struct lightrec_link *heads[LINKS_HASH_SIZE];
	unsigned int nb_links;
};

static inline unsigned int links_hash(u32 offset)
{
	return (offset ^ (offset >> LINKS_HASH_BITS)) & (LINKS_HASH_SIZE - 1);
}

static inline void links_lock(struct lightrec_links *links)
{
#if ENABLE_THREADED_COMPILER
	pthread_mutex_lock(&links->mutex);
#endif
}

static inline void links_unlock(struct lightrec_links *links)
{
#if ENABLE_THREADED_COMPILER
	pthread_mutex_unlock(&links->mutex);
#endif
}

/* What an edge jumps to for a given LUT value: the code if there is some,
 * otherwise the dispatcher entry that looks the target up the slow way.
 * The preprocessed-not-compiled marker counts as "no code". */
/* SH-4: `mov.l lit,rN; jmp @rN; nop` at every linked exit is a literal
 * load and an indirect jump for a target that, three times out of four,
 * is within the 4 KiB a `bra` reaches (77% of Rayman's resolved links).
 * Find the pair once, then keep it as `nop; bra target; nop` whenever
 * the target is in range. Anything unexpected in the shape leaves the
 * link on the far form, which is always correct. */
#if defined(__sh__)
#define SH_NOP		0x0009
#define SH_BRA(disp)	(0xa000 | ((disp) & 0xfff))

static void link_locate(const struct block *block, struct lightrec_link *link)
{
	const u16 *code = (const u16 *)block->function;
	unsigned int i, j, nb = block->code_size / sizeof(u16);
	uintptr_t lit = (uintptr_t)link->word;
	u16 op, rn;

	link->movl = NULL;
	link->jmp = NULL;

	for (i = 0; i < nb; i++) {
		op = code[i];
		if ((op & 0xf000) != 0xd000)
			continue;
		/* mov.l @(disp,pc),rN: pc is the opcode's address + 4, &~3 */
		if ((((uintptr_t)&code[i] + 4) & ~(uintptr_t)3) + (op & 0xff) * 4 != lit)
			continue;
		rn = (op >> 8) & 0xf;
		for (j = i + 1; j < nb && j < i + 16; j++) {
			if (code[j] == (0x402b | (rn << 8)) && j + 1 < nb
			    && code[j + 1] == SH_NOP) {
				link->movl = (u16 *)&code[i];
				link->jmp = (u16 *)&code[j];
				link->movl_word = op;
				link->jmp_word = code[j];
				return;
			}
		}
	}
}

static void link_set_shape(struct lightrec_state *state,
			   struct lightrec_link *link, u32 value, bool code)
{
	long disp;

	if (!link->jmp)
		return;

	disp = (((long)value - (long)(uintptr_t)link->jmp) >> 1) - 2;

	if (code && disp >= -2048 && disp <= 2047) {
		*link->movl = SH_NOP;
		*link->jmp = SH_BRA(disp);
		link->patched = true;
	} else if (link->patched) {
		*link->movl = link->movl_word;
		*link->jmp = link->jmp_word;
		link->patched = false;
	} else {
		return;
	}

	if (state->ops.code_inv)
		state->ops.code_inv(link->movl,
				    (uintptr_t)link->jmp + 2 - (uintptr_t)link->movl);
}
#else
static inline void link_locate(const struct block *block, struct lightrec_link *link) {}
static inline void link_set_shape(struct lightrec_state *state,
				  struct lightrec_link *link, u32 value, bool code) {}
#endif

static inline u32 link_value(struct lightrec_state *state, void *ptr)
{
	if (ptr && ptr != (void *)state->get_next_block)
		return (u32)(uintptr_t)ptr;

	return (u32)(uintptr_t)state->eob_wrapper_func;
}

static void link_resolve(struct lightrec_state *state,
			 struct lightrec_link *link, void *ptr)
{
	bool code = ptr && ptr != (void *)state->get_next_block;
	u32 value = link_value(state, ptr);

	*link->word = value;
	link_set_shape(state, link, value, code);
}

struct lightrec_links * lightrec_links_init(struct lightrec_state *state)
{
	struct lightrec_links *links;

	links = lightrec_calloc(state, MEM_FOR_LIGHTREC, sizeof(*links));
	if (!links)
		return NULL;

#if ENABLE_THREADED_COMPILER
	if (pthread_mutex_init(&links->mutex, NULL)) {
		lightrec_free(state, MEM_FOR_LIGHTREC, sizeof(*links), links);
		return NULL;
	}
#endif

	return links;
}

void lightrec_links_destroy(struct lightrec_state *state)
{
	struct lightrec_links *links = state->links;

	if (!links)
		return;

#if ENABLE_THREADED_COMPILER
	pthread_mutex_destroy(&links->mutex);
#endif
	lightrec_free(state, MEM_FOR_LIGHTREC, sizeof(*links), links);
	state->links = NULL;
}

static void links_remove_from_chain(struct lightrec_links *links,
				    struct lightrec_link *link)
{
	struct lightrec_link **pp = &links->heads[links_hash(link->offset)];

	for (; *pp; pp = &(*pp)->next_target) {
		if (*pp == link) {
			*pp = link->next_target;
			links->nb_links--;
			return;
		}
	}
}

void lightrec_links_free_list(struct lightrec_state *state,
			      struct lightrec_link *list)
{
	struct lightrec_links *links = state->links;
	struct lightrec_link *link, *next;

	if (!list)
		return;

	links_lock(links);

	for (link = list; link; link = next) {
		next = link->next_owner;
		links_remove_from_chain(links, link);
		lightrec_free(state, MEM_FOR_LIGHTREC, sizeof(*link), link);
	}

	links_unlock(links);
}

static u32 * find_magic_word(const struct block *block, u32 magic)
{
	const u32 *code = (const u32 *)block->function;
	unsigned int i, nb = block->code_size / sizeof(u32);
	u32 *found = NULL;

	for (i = 0; i < nb; i++) {
		if (code[i] != magic)
			continue;

		if (found) {
			pr_err("Block "PC_FMT": link word %08" PRIx32 " is ambiguous\n",
			       block->pc, magic);
			return NULL;
		}

		found = (u32 *)&code[i];
	}

	if (!found)
		pr_err("Block "PC_FMT": link word %08" PRIx32 " not found\n",
		       block->pc, magic);

	return found;
}

int lightrec_links_register_block(struct lightrec_state *state,
				  struct block *block,
				  const struct lightrec_pending_link *pending,
				  unsigned int nb_pending)
{
	struct lightrec_links *links = state->links;
	struct lightrec_link *link, *list = NULL;
	unsigned int i, h;
	u32 *word;

	/* The block's previous code generation gave its links away before
	 * we got here. */
	block->links = NULL;

	if (!nb_pending)
		return 0;

	for (i = 0; i < nb_pending; i++) {
		word = find_magic_word(block, pending[i].magic);
		if (!word) {
			lightrec_links_free_list(state, list);
			return -EINVAL;
		}

		link = lightrec_malloc(state, MEM_FOR_LIGHTREC, sizeof(*link));
		if (!link) {
			lightrec_links_free_list(state, list);
			return -ENOMEM;
		}

		link->offset = pending[i].offset;
		link->word = word;
		link->patched = false;
		link_locate(block, link);
		link->next_owner = list;
		list = link;

		links_lock(links);

		/* Resolve now, under the lock, so a LUT change racing with
		 * us is ordered against this write. */
		link_resolve(state, link, lut_read(state, link->offset));

		h = links_hash(link->offset);
		link->next_target = links->heads[h];
		links->heads[h] = link;
		links->nb_links++;

		links_unlock(links);
	}

	block->links = list;
	return 0;
}

void lightrec_links_lut_changed(struct lightrec_state *state,
				u32 offset, void *ptr)
{
	struct lightrec_links *links = state->links;
	struct lightrec_link *link;

	if (!links)
		return;

	links_lock(links);

	for (link = links->heads[links_hash(offset)]; link;
	     link = link->next_target) {
		if (link->offset == offset)
			link_resolve(state, link, ptr);
	}

	links_unlock(links);
}

void lightrec_links_lut_cleared(struct lightrec_state *state,
				u32 offset, u32 count)
{
	struct lightrec_links *links = state->links;
	struct lightrec_link *link;
	unsigned int h;
	u32 i;

	if (!links)
		return;

	/* The "no links" test must be under the lock: the compiler thread
	 * reads the LUT and inserts its link in one critical section, and a
	 * DMA invalidation that slips between an unlocked check here and
	 * that insertion would leave a live link into overwritten code. */
	links_lock(links);

	if (!links->nb_links) {
		links_unlock(links);
		return;
	}

	if (count <= 256) {
		for (i = 0; i < count; i++) {
			for (link = links->heads[links_hash(offset + i)]; link;
			     link = link->next_target) {
				if (link->offset == offset + i)
					link_resolve(state, link, NULL);
			}
		}
	} else {
		for (h = 0; h < LINKS_HASH_SIZE; h++) {
			for (link = links->heads[h]; link;
			     link = link->next_target) {
				if (link->offset - offset < count)
					link_resolve(state, link, NULL);
			}
		}
	}

	links_unlock(links);
}

void lightrec_links_lut_cleared_all(struct lightrec_state *state)
{
	lightrec_links_lut_cleared(state, 0, CODE_LUT_SIZE);
}
