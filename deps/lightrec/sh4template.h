/* SPDX-License-Identifier: LGPL-2.1-or-later */

#ifndef __LIGHTREC_SH4TEMPLATE_H__
#define __LIGHTREC_SH4TEMPLATE_H__

#include "disassembler.h"
#include "sh4asm.h"

/*
 * The fixed whole-run register contract.  r13 and r14 are reserved from the
 * first template even though dispatch and the deadline gate land later.
 */
#define LIGHTREC_SH4T_XFER	SH4ASM_R0
#define LIGHTREC_SH4T_SCRATCH	SH4ASM_R3
#define LIGHTREC_SH4T_POOL_FIRST	SH4ASM_R1
#define LIGHTREC_SH4T_POOL_LAST	SH4ASM_R12
#define LIGHTREC_SH4T_ADDR_MASK	SH4ASM_R13
#define LIGHTREC_SH4T_DEADLINE	SH4ASM_R14
#define LIGHTREC_SH4T_STACK	SH4ASM_R15
#define LIGHTREC_SH4T_NR_PINS	6

struct lightrec_sh4t {
	struct sh4asm as;
	uint16_t movi_high[32];
	unsigned int gpr_offset;
};

void lightrec_sh4t_init(struct lightrec_sh4t *t, void *buffer, size_t size,
			uintptr_t base, unsigned int gpr_offset);

/* Fixed pin lookup; -1 means the guest register is memory-backed. */
int lightrec_sh4t_guest_host(unsigned int guest_reg);

void lightrec_sh4t_emit_pin_loads(struct lightrec_sh4t *t);
void lightrec_sh4t_emit_pin_stores(struct lightrec_sh4t *t);

/* Returns true when op belongs to the phase-one ALU-immediate bucket. */
bool lightrec_sh4t_emit_alu_imm(struct lightrec_sh4t *t,
				const struct opcode *op);
bool lightrec_sh4t_emit_alu_reg(struct lightrec_sh4t *t,
				const struct opcode *op);
bool lightrec_sh4t_emit_meta_alu(struct lightrec_sh4t *t,
				 const struct opcode *op);

/* Direct-map integer load (LB/LBU/LH/LHU/LW).  host_offset and mask come from
 * the opcode's IO mode; use_mask is false when the address is provably KUSEG. */
bool lightrec_sh4t_emit_load(struct lightrec_sh4t *t, const struct opcode *op,
			     uintptr_t host_offset, bool use_mask, uint32_t mask);

/* Direct-map integer store (SB/SH/SW) to RAM or the scratchpad.  The caller
 * must have confirmed no code-LUT invalidation is required for this store. */
bool lightrec_sh4t_emit_store(struct lightrec_sh4t *t, const struct opcode *op,
			      uintptr_t host_offset, bool use_mask, uint32_t mask);

/* Fallback memory access via a C helper (any mode).  opcode is the raw guest
 * instruction; c_func is the GBR-shimmed lightrec_rw_cb entry. */
void lightrec_sh4t_emit_mem_call(struct lightrec_sh4t *t, uint32_t opcode,
				 uintptr_t c_func);

/* mult/div/MFHI/MFLO/MTHI/MTLO via a C helper.  flags carries NO_HI/NO_LO. */
void lightrec_sh4t_emit_muldiv_call(struct lightrec_sh4t *t, uint32_t opcode,
				    uint32_t flags, uintptr_t c_func);

/* Whole-block gate.  Accepts bodies of implemented ops (plus NOPs) with forward
 * conditional branches (mid-block or terminating) and external unconditional
 * terminators.  A false result means the whole block stays on lightning. */
bool lightrec_sh4t_can_emit_block(const struct opcode *ops,
				 unsigned int nr_ops, uint32_t pc,
				 bool inv_dma_only);

/* Emit the comparison for a MIPS branch condition; returns true if "taken"
 * means the SH-4 T bit is set (use bt), false if it means T clear (use bf). */
bool lightrec_sh4t_emit_condition(struct lightrec_sh4t *t,
				  const struct opcode *op);

/* Branch classification and target resolution, shared by the gate and the
 * compiler so both agree on control-flow shape. */
bool lightrec_sh4t_is_uncond(const struct opcode *op);
bool lightrec_sh4t_is_cond_branch(const struct opcode *op);
bool lightrec_sh4t_is_jump_reg(const struct opcode *op);
uint32_t lightrec_sh4t_branch_target(const struct opcode *op, uint32_t pc,
				     unsigned int i, bool no_ds);

/* JAL / static call: link (link_reg 0 = none) then static exit to target. */
void lightrec_sh4t_emit_call(struct lightrec_sh4t *t, unsigned int link_reg,
			     uint32_t return_pc, unsigned int cycles,
			     uint32_t target_pc, uint32_t target_lut,
			     unsigned int fast_eob_offset);

/* JR / JALR: optional link, then register-indirect jump via eob_wrapper_func. */
void lightrec_sh4t_emit_jump_reg(struct lightrec_sh4t *t, unsigned int rs_guest,
				 unsigned int link_reg, uint32_t return_pc,
				 unsigned int cycles, unsigned int eob_offset);

/* Shared dispatcher-boundary exit: charge cycles in r1, hand next_pc to r4,
 * its LUT entry to r5, and jump through state->fast_eob via GBR. */
void lightrec_sh4t_emit_dispatch_exit(struct lightrec_sh4t *t,
				      unsigned int cycles, uint32_t next_pc,
				      uint32_t lut_entry,
				      unsigned int fast_eob_offset);

#endif /* __LIGHTREC_SH4TEMPLATE_H__ */
