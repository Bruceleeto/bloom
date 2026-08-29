// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Direct block linking - see links.h.
 */

#include "debug.h"
#include "lightrec-config.h"
#include "lightrec-private.h"
#include "links.h"
#include "memmanager.h"
#include "regcache.h"

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
static inline u32 link_value(struct lightrec_state *state, void *ptr)
{
	/* Real code starts with the pinned-register load stub; a link
	 * carries the pins in their registers and enters behind it. The
	 * dispatcher fallback writes them back first (regcache.c). */
	if (ptr && ptr != (void *)state->get_next_block)
		return (u32)(uintptr_t)ptr + LIGHTREC_PIN_STUB_BYTES;

	return (u32)(uintptr_t)state->eob_wrapper_pins_func;
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

static u32 * find_magic_word(const struct block *block, const void *function,
			     u32 magic)
{
	const u32 *code = (const u32 *)function;
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
				  struct block *block, void *function,
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
		word = find_magic_word(block, function, pending[i].magic);
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
		link->next_owner = list;
		list = link;

		links_lock(links);

		/* Resolve now, under the lock, so a LUT change racing with
		 * us is ordered against this write. */
		*word = link_value(state, lut_read(state, link->offset));

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
	u32 value;

	if (!links)
		return;

	links_lock(links);

	value = link_value(state, ptr);

	for (link = links->heads[links_hash(offset)]; link;
	     link = link->next_target) {
		if (link->offset == offset)
			*link->word = value;
	}

	links_unlock(links);
}

void lightrec_links_lut_cleared(struct lightrec_state *state,
				u32 offset, u32 count)
{
	struct lightrec_links *links = state->links;
	struct lightrec_link *link;
	unsigned int h;
	u32 value, i;

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

	value = link_value(state, NULL);

	if (count <= 256) {
		for (i = 0; i < count; i++) {
			for (link = links->heads[links_hash(offset + i)]; link;
			     link = link->next_target) {
				if (link->offset == offset + i)
					*link->word = value;
			}
		}
	} else {
		for (h = 0; h < LINKS_HASH_SIZE; h++) {
			for (link = links->heads[h]; link;
			     link = link->next_target) {
				if (link->offset - offset < count)
					*link->word = value;
			}
		}
	}

	links_unlock(links);
}

void lightrec_links_lut_cleared_all(struct lightrec_state *state)
{
	lightrec_links_lut_cleared(state, 0, CODE_LUT_SIZE);
}
