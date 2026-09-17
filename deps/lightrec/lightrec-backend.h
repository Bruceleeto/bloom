/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * The seam between lightrec's portable core and whatever turns a MIPS opcode
 * list into host code.
 *
 * lightrec was already layered: blockcache.c, optimizer.c, interpreter.c,
 * recompiler.c and memmanager.c contain no reference to the code generator at
 * all, and GNU Lightning's own use lives in emitter.c and regcache.c.  What
 * was missing is this header: about four hundred lines of Lightning had leaked
 * into lightrec.c -- the dispatcher, the code emitter, the cycle-counter
 * helpers -- with no way to supply a different one short of forking the file.
 *
 * A backend owns exactly three things:
 *
 *   - whatever it needs to enter emitted code (Lightning generates a
 *     dispatcher block; a hand-written backend may have one in assembly),
 *   - turning `block->opcode_list` into host code,
 *   - running the guest.
 *
 * Everything else -- the block cache, the optimiser, the LUT, the reaper,
 * dead-block detection -- stays in the core and is shared.
 *
 * Selection is by name at run time rather than by #ifdef, so that one binary
 * can be asked for either backend and the two compared without a rebuild.
 * Nothing on this path is hot: `compile` runs once per block and `execute`
 * about twice per frame, so the indirect call costs nothing measurable.
 */

#ifndef __LIGHTREC_BACKEND_H__
#define __LIGHTREC_BACKEND_H__

#include "lightrec.h"

struct block;
struct lightrec_cstate;
struct lightrec_state;

/* Backend capability bits.  The optimiser has a couple of transforms that a
 * backend has to be able to consume, and one binary can hold two backends, so
 * this cannot be an #ifdef. */

/* The LUI/ORI constant fold tags a LOAD and parks the LUI's immediate, leaving
 * the load to materialise the whole constant.  A backend that does not honour
 * that contract for loads sets this and the pass skips them. */
#define LIGHTREC_BACKEND_NO_LUI_LOAD_FOLD	(1u << 0)

struct lightrec_backend {
	const char *name;
	unsigned int flags;

	/* Alignment the code arena must give a block, or 0 for none.  fgl
	 * asks for 32 so that every block starts an SH-4 icache line. */
	unsigned int code_align;

	/* Process-wide setup, done once before the first state exists and
	 * undone after the last one is gone.  Lightning needs it for its own
	 * global tables; a backend that needs nothing leaves these null. */
	int  (*global_init)(char *argv0);
	void (*global_fini)(void);

	/* Per-state setup, called from lightrec_init() once the block cache,
	 * recompiler and reaper are up, and undone from lightrec_destroy().
	 * Returns 0 or a negative errno. */
	int  (*init)(struct lightrec_state *state);
	void (*destroy)(struct lightrec_state *state);

	/* `cycles_per_op` has changed (the frontend's overclock knob).  A
	 * backend that precomputes anything from it rebuilds here.  Lightning
	 * reads the field itself and leaves this null; fgl keeps a table of
	 * `n * cycles_per_op` in its state words, and a table left at the
	 * init-time value of 2 charges the guest clock 1/1024 of what it
	 * should -- which is a guest that never reaches its next interrupt. */
	void (*cycles_changed)(struct lightrec_state *state);

	/* THE CYCLE PAIR WAS MOVED FROM C, UNDER A BLOCK THAT IS STILL RUNNING.
	 *
	 * `set_exit_flags`, `reset_cycle_count` and `set_target_cycle_count`
	 * are how the frontend moves the pair, and all three can be reached
	 * from a device access in the middle of a block.  A backend that hands
	 * emitted code a budget DERIVED from the pair -- rather than the pair
	 * itself -- has to rebuild it here, or the block runs on past a
	 * deadline that has already moved.  Lightning gives emitted code the
	 * difference and leaves this null. */
	void (*cycles_moved)(struct lightrec_state *state);

	/* Turn block->opcode_list into host code.
	 *
	 * On success: block->function and block->code_size are set, and
	 * cstate->targets[i].host holds the host address of each branch
	 * target the block registered -- the core writes those into the code
	 * LUT and must not have to know how the backend names a label.
	 *
	 * Returns 0 or a negative errno.  The backend is responsible for
	 * leaving block->function untouched on failure.
	 */
	int  (*compile)(struct lightrec_cstate *cstate, struct block *block);

	/* Free whatever `compile` hung off the block.  `reap_block` is the
	 * same thing deferred through the reaper, which hands back the opaque
	 * pointer the backend stored rather than the block itself -- by then
	 * the block may be gone. */
	void (*free_block)(struct lightrec_state *state, struct block *block);
	void (*reap_block)(struct lightrec_state *state, void *priv);

	/* Run the guest from `pc` until `target_cycle`, and return the PC it
	 * stopped at.  The core has already set exit_flags to NORMAL. */
	u32  (*execute)(struct lightrec_state *state, u32 pc, u32 target_cycle);

	/* THE SELF-PATCHING LINK, UNDONE BEFORE THE TABLE MOVES.
	 *
	 * A backend may patch a block's exit into a direct branch to its
	 * successor.  Such a branch is baked into host code with no
	 * indirection left in front of it, so clearing a code-LUT slot does
	 * not reach it: every link has to go back BEFORE anything frees code
	 * or blanks a slot.  A backend whose exits always go through the LUT
	 * leaves all three null.
	 *
	 *   unlink_range  the `nb_ops` slots from byte `offset` are about to
	 *                 be cleared.
	 *   unlink_block  as above, and this block's code is about to be
	 *                 freed -- `fn`/`size` name the code itself, because
	 *                 links INTO it have to go as well as links out.
	 *   unlink_all    every slot at once; every link is stale.
	 */
	void (*unlink_range)(struct lightrec_state *state, u32 offset,
			     unsigned int nb_ops, unsigned int why);
	void (*unlink_block)(struct lightrec_state *state, u32 offset,
			     unsigned int nb_ops, void *fn, unsigned int size,
			     unsigned int why);
	void (*unlink_all)(struct lightrec_state *state, unsigned int why);
};

/* Why the links are going down.  A backend counts these; the core only
 * reports them.  `INV` is the one that matters -- it used to be answered with
 * unlink_all, so a four-byte guest store threw away the whole link graph. */
enum lightrec_unlink_reason {
	LIGHTREC_UNLINK_INV_MAP,	/* lightrec_invalidate_map: store/DMA */
	LIGHTREC_UNLINK_SMC,		/* an opcode was tagged, block recompiles */
	LIGHTREC_UNLINK_FREE,		/* code is being freed */
	LIGHTREC_UNLINK_INV,		/* lightrec_invalidate */
	LIGHTREC_UNLINK_INV_ALL,	/* lightrec_invalidate_all */
	LIGHTREC_UNLINK_LUT,		/* remove_from_code_lut */
	LIGHTREC_UNLINK_N
};

/* The backends compiled into this build.  A backend that is not compiled in
 * is not declared, so asking for it is a link error rather than a run-time
 * surprise -- which is what the CMake option decides. */
#if !defined(LIGHTREC_NO_LIGHTNING)
extern const struct lightrec_backend lightrec_backend_lightning;
#endif
#if defined(LIGHTREC_WITH_FGL)
extern const struct lightrec_backend lightrec_backend_fgl;
#endif

/*
 * Choose the backend the next lightrec_init() will use.
 *
 * Deliberately not a parameter to lightrec_init(): every caller of that
 * function in pcsx_rearmed and bloom would have to change, and the point of
 * this exercise is a port that does not ripple.  Call it before lightrec_init()
 * or not at all.
 *
 * `name` is matched against lightrec_backend::name.  NULL or an unknown name
 * selects the default, which is $LIGHTREC_BACKEND if that names a compiled-in
 * backend and otherwise the first one compiled in.  Returns the backend that
 * will actually be used, so a caller can report it.
 */
const struct lightrec_backend * lightrec_select_backend(const char *name);

/* The backend this state is running, for reporting. */
const struct lightrec_backend * lightrec_get_backend(const struct lightrec_state *state);

#endif /* __LIGHTREC_BACKEND_H__ */
