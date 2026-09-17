/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * The service table on the real machine.  `fgl_lightrec.c` is shared with the
 * Linux proving ground, where none of these symbols exist, so it calls
 * `fgl_run_fill_targets` and each tree supplies its own.  The research
 * helpers at the bottom answer the proving ground's unconditional call sites
 * with nothing; `fgl_tags_at` returning NULL is a supported answer.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lightrec-private.h"

#include "fgl.h"
#include "fgl_backend.h"
#include "shim.h"

/* Declared rather than included: `gte_fpu.h` pulls in libpcsxcore's headers,
 * which this translation unit has no other need of.  lightrec's own emitter
 * declared it the same way and for the same reason. */
extern void *gte_fpu_resolve(u32 op);
extern int gte_fpu_leaf_cmd(u32 op);
extern uint32_t psxCP2Gen[2];

/* The leaf routines (gte_rtp.S).  Declared as data: their addresses go into
 * emitted code, and they are not C functions -- they take the register file
 * from GBR and keep every register but r0, r1, PR, MAC and T. */
extern char gte_nclip_leaf[], gte_rtps_leaf[], gte_rtpt_leaf[],
	    gte_mvmva_leaf[], gte_dpcs_leaf[], gte_intpl_leaf[],
	    gte_sqr_leaf[], gte_op_leaf[], gte_gpf_leaf[], gte_gpl_leaf[],
	    gte_dcpl_leaf[], gte_dpct_leaf[], gte_avsz3_leaf[],
	    gte_avsz4_leaf[], gte_ncs_leaf[], gte_nct_leaf[],
	    gte_nccs_leaf[], gte_ncct_leaf[], gte_ncds_leaf[],
	    gte_ncdt_leaf[], gte_cc_leaf[], gte_cdp_leaf[];

/* The C half of the seam, defined in fgl_lightrec.c. */
void fgl_rw(u32 opcode, struct lightrec_state *state);
void fgl_mtc(u32 opcode, struct lightrec_state *state);
void fgl_mfc(u32 opcode, struct lightrec_state *state);
void fgl_rfe(u32 unused, struct lightrec_state *state);
void fgl_wild_store(void);

/* The COP2 body for one command; NULL means the hardware ignores it and the
 * emitter emits nothing. */
static u32 fgl_gte_body(void *user, u32 op)
{
	(void)user;
	return (u32)(uintptr_t)gte_fpu_resolve(op);
}

/* The leaves in GTE_LEAF_* order (gte_fpu.h); index 0 is "no leaf" (shim).
 * The table is copied into the GBR slots at FGL_SVC_GTE_LEAF once, so an
 * emitted COP2 call reads its target from there in one instruction. */
static char *const fgl_gte_leaves[FGL_GTE_LEAF_N] = {
	NULL,            gte_nclip_leaf,  gte_rtps_leaf,
	gte_rtpt_leaf,   gte_mvmva_leaf,  gte_dpcs_leaf,
	gte_intpl_leaf,  gte_sqr_leaf,    gte_op_leaf,
	gte_gpf_leaf,    gte_gpl_leaf,    gte_dcpl_leaf,
	gte_dpct_leaf,   gte_avsz3_leaf,  gte_avsz4_leaf,
	gte_ncs_leaf,    gte_nct_leaf,    gte_nccs_leaf,
	gte_ncct_leaf,   gte_ncds_leaf,   gte_ncdt_leaf,
	gte_cc_leaf,     gte_cdp_leaf,
};

/* The leaf for one command as a slot index: `gte_fpu_leaf_cmd` already
 * returns (index << 1) | argflag, so this only refuses commands without one. */
static u32 fgl_gte_leaf(void *user, u32 op)
{
	int cmd = gte_fpu_leaf_cmd(op);
	unsigned idx = (unsigned)cmd >> 1;

	(void)user;
	if (!cmd || idx >= FGL_GTE_LEAF_N || !fgl_gte_leaves[idx])
		return 0;

	return (u32)cmd;
}

/* Filled at run time: a function address is not a constant expression. */
void fgl_run_fill_targets(fgl_targets *t)
{
	t->shim_call_st = (u32)(uintptr_t)fgl_shim_call_st;
	t->shim_gte     = (u32)(uintptr_t)fgl_shim_gte;
	t->shim_divu    = (u32)(uintptr_t)fgl_shim_divu;
	t->shim_div     = (u32)(uintptr_t)fgl_shim_div;

	t->hw_load[MEM_B]  = (u32)(uintptr_t)lightrec_hw_lb;
	t->hw_load[MEM_BU] = (u32)(uintptr_t)lightrec_hw_lbu;
	t->hw_load[MEM_H]  = (u32)(uintptr_t)lightrec_hw_lh;
	t->hw_load[MEM_HU] = (u32)(uintptr_t)lightrec_hw_lhu;
	t->hw_load[MEM_W]  = (u32)(uintptr_t)lightrec_hw_lw;

	t->hw_store[MEM_B] = (u32)(uintptr_t)lightrec_hw_sb;
	t->hw_store[MEM_H] = (u32)(uintptr_t)lightrec_hw_sh;
	t->hw_store[MEM_W] = (u32)(uintptr_t)lightrec_hw_sw;

	t->rw       = (u32)(uintptr_t)fgl_rw;
	t->mtc      = (u32)(uintptr_t)fgl_mtc;
	t->mfc      = (u32)(uintptr_t)fgl_mfc;
	t->rfe      = (u32)(uintptr_t)fgl_rfe;
	t->cp2_ctrl_gen = (u32)(uintptr_t)&psxCP2Gen[0];
	t->wild     = (u32)(uintptr_t)fgl_wild_store;
	t->gte_body = fgl_gte_body;
	t->gte_leaf = fgl_gte_leaf;
	t->user     = NULL;

	/* The addresses themselves, for the GBR slots. */
	{
		unsigned li;
		for (li = 0; li < FGL_GTE_LEAF_N; li++)
			t->gte_leaf_tab[li] = (u32)(uintptr_t)fgl_gte_leaves[li];
	}

	/* Last: nothing may observe a half-filled table. */
	t->shim_call = (u32)(uintptr_t)fgl_shim_call;
}

/* ------------------------------------------------------------------ */
/* The research instruments, not built into the image                  */
/* ------------------------------------------------------------------ */

uint8_t *fgl_tags_at(void *code)
{
	(void)code;
	return NULL;
}

void fgl_ops_at(void *code, unsigned n_ops)
{
	(void)code;
	(void)n_ops;
}

void fgl_static_census(const void *code, unsigned size, unsigned n_ops)
{
	(void)code;
	(void)size;
	(void)n_ops;
}

/* Both only gate an FGL_REGION_DBG experiment; no environment here. */
unsigned fgl_block_access_classes(const struct block *block)
{
	(void)block;
	return 0;
}

bool fgl_block_touches_device(const struct block *block)
{
	(void)block;
	return false;
}
