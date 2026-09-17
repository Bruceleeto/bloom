/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* 
 * The boundary between lightrec and fgl: what the backend calls to compile,
 * what the assembly calls back into C, and the dispatch.S entry points kept
 * in the state block as plain addresses.
 *
 */

#ifndef FGL_BACKEND_H
#define FGL_BACKEND_H

#include "lightrec-private.h"
#include "fgl_state.h"

/* The backend's words inside struct lightrec_state::backend[].  Layout asserted against fgl_state.h. */
struct fgl_words {
	u32 cycle_table[FGL_CYCLE_ENTRIES];
	u32 dispatch;
	u32 lut_base;
	u32 addr_mask;
	u32 shim_arg;
	u32 link;
	u32 tsave;
	s32 exit_meter;
	s32 meter_base;
	s32 meter_in;
	u32 inv_mask;
	u32 inv_ram;
	u32 inv_direct;
	u32 pin_pub;
	u32 pin_rel;
	u32 rw_tramp;
	u32 rw_block;
	u32 svc[FGL_SVC_N];
};

static inline struct fgl_words *
fgl_state_words(struct lightrec_state *state)
{
	return (struct fgl_words *)state->backend;
}

/* Compile one block from lightrec's optimised list; returns the entry point or NULL.
 * `*why`: -ENOMEM = arena full (transient, caller may flush and retry);
 * -EINVAL = fgl declined this block (permanent; retrying loops).  `why` may be NULL.
 */
void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why);

/* The budget is a count of guest instructions, not cycles, so restoring the pair is fgl's arithmetic. */
void fgl_cycles_settle(struct lightrec_state *state, s32 delta);

/* C moved the cycle pair under a running block (device access reschedules the
 * slice), so the parked budget is stale.  Wired to the backend's `cycles_moved` hook.
 */
void fgl_meter_refresh(struct lightrec_state *state);

/* The dispatcher: hand-written SH-4 (dispatch.S).  `fgl_dispatch` is the way in from C; the rest are jump targets. */
u32 fgl_dispatch(struct lightrec_state *state, u32 pc, void *first_block,
		 s32 cycle_delta);

void fgl_dispatch_loop(void);
void fgl_link_stub(void);
void fgl_dispatch_compile(void);
void fgl_dispatch_memset(void);
void fgl_dispatch_interpreter(void);
void fgl_dispatch_ds_check(void);

/* Arena addresses as emitted code sees them: real on the Dreamcast, resolved by the SH-4 interpreter elsewhere. */
u32 fgl_run_dispatch_addr(void);
u32 fgl_run_link_addr(void);
u32 fgl_run_lut_addr(void);

/* The self-patching link: a constant-successor edge rewrites its own call site
 * into a branch once the successor exists.  `fgl_unlink_all` must run before
 * anything frees code or clears a table slot; `why` is enum lightrec_unlink_reason.
 */
void fgl_unlink_all(struct lightrec_state *state, unsigned why);
void fgl_unlink_block(struct lightrec_state *state, u32 first, u32 n,
		      void *fn, unsigned size, unsigned why);
void fgl_unlink_range(struct lightrec_state *state, u32 first, u32 n,
		      unsigned why);

extern unsigned fgl_unlink_calls[LIGHTREC_UNLINK_N];
extern unsigned fgl_unlink_links[LIGHTREC_UNLINK_N];
extern unsigned fgl_free_code_n, fgl_recompile_n, fgl_outdated_n, fgl_compile_n;

/* What the assembly calls back into. */
void *fgl_get_next_block(struct lightrec_state *state, u32 pc);
u32 fgl_memset(struct lightrec_state *state);
u32 fgl_emulate_block(struct lightrec_state *state, struct block *block,
		      u32 pc);
u32 fgl_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg);

/* An access whose region the optimiser could not prove, done in C.  Reached through `fgl_shim_call`. */
/* Direct device access for HW-tagged plain loads and stores, called bare from generated code. */
u32 lightrec_hw_lb(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lbu(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lh(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lhu(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lw(u32 addr, struct lightrec_state *state);
void lightrec_hw_sb(u32 addr, u32 val, struct lightrec_state *state);
void lightrec_hw_sh(u32 addr, u32 val, struct lightrec_state *state);
void lightrec_hw_sw(u32 addr, u32 val, struct lightrec_state *state);

void fgl_rw(u32 opcode, struct lightrec_state *state);
void fgl_mtc(u32 opcode, struct lightrec_state *state);
void fgl_mfc(u32 opcode, struct lightrec_state *state);
void fgl_rfe(u32 unused, struct lightrec_state *state);

/* The arena address of something the emitter put there, as generated code sees it. */
u32 fgl_run_addr_of(void *fn);

/* The out-of-line store-invalidation stubs (emit.c); offsets come back into the caller's buffer. */
unsigned fgl_emit_inv_stubs(void *buf, unsigned cap,
			    unsigned *ram, unsigned *direct,
			    unsigned *pub, unsigned *rel, unsigned *rwt);
extern int fgl_inv_stubs_ready;
extern int fgl_pin_stubs_ready;
extern int fgl_rw_tramp_ready;

#endif /* FGL_BACKEND_H */
