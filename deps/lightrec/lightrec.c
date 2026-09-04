// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Copyright (C) 2014-2021 Paul Cercueil <paul@crapouillou.net>
 */

#include "blockcache.h"
#include "debug.h"
#include "disassembler.h"
#include "interpreter.h"
#include "lightrec-config.h"
#include "lightrec-private.h"
#include "lightrec.h"
#include "memmanager.h"
#include "reaper.h"
#include "recompiler.h"
#include "optimizer.h"
#include "tlsf/tlsf.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#if ENABLE_THREADED_COMPILER
#include <stdatomic.h>
#endif
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*
 * Bumped on every write to a COP2 control register, wherever the write comes
 * from.  Defined by the core's GTE (libpcsxcore/gte.c); declared here because
 * this library does not include the core's headers.
 */
extern uint32_t psxCP2CtrlGen;

static struct block * lightrec_precompile_block(struct lightrec_state *state,
						u32 pc);
static bool lightrec_block_is_fully_tagged(const struct block *block);

static void lightrec_mtc2(struct lightrec_state *state, u8 reg, u32 data);
static u32 lightrec_mfc2(struct lightrec_state *state, u8 reg);

static void lightrec_reap_block(struct lightrec_state *state, void *data);

static void lightrec_default_sb(struct lightrec_state *state, u32 opcode,
				void *host, u32 addr, u32 data)
{
	*(u8 *)host = (u8)data;

	if (!(state->opt_flags & LIGHTREC_OPT_INV_DMA_ONLY))
		lightrec_invalidate(state, addr, 1);
}

static void lightrec_default_sh(struct lightrec_state *state, u32 opcode,
				void *host, u32 addr, u32 data)
{
	*(u16 *)host = HTOLE16((u16)data);

	if (!(state->opt_flags & LIGHTREC_OPT_INV_DMA_ONLY))
		lightrec_invalidate(state, addr, 2);
}

static void lightrec_default_sw(struct lightrec_state *state, u32 opcode,
				void *host, u32 addr, u32 data)
{
	*(u32 *)host = HTOLE32(data);

	if (!(state->opt_flags & LIGHTREC_OPT_INV_DMA_ONLY))
		lightrec_invalidate(state, addr, 4);
}

static u8 lightrec_default_lb(struct lightrec_state *state,
			      u32 opcode, void *host, u32 addr)
{
	return *(u8 *)host;
}

static u16 lightrec_default_lh(struct lightrec_state *state,
			       u32 opcode, void *host, u32 addr)
{
	return LE16TOH(*(u16 *)host);
}

static u32 lightrec_default_lw(struct lightrec_state *state,
			       u32 opcode, void *host, u32 addr)
{
	return LE32TOH(*(u32 *)host);
}

static u32 lightrec_default_lwu(struct lightrec_state *state,
				u32 opcode, void *host, u32 addr)
{
	u32 val;

	memcpy(&val, host, 4);

	return LE32TOH(val);
}

static void lightrec_default_swu(struct lightrec_state *state, u32 opcode,
				 void *host, u32 addr, u32 data)
{
	data = HTOLE32(data);

	memcpy(host, &data, 4);

	if (!(state->opt_flags & LIGHTREC_OPT_INV_DMA_ONLY))
		lightrec_invalidate(state, addr & ~0x3, 8);
}

static const struct lightrec_mem_map_ops lightrec_default_ops = {
	.sb = lightrec_default_sb,
	.sh = lightrec_default_sh,
	.sw = lightrec_default_sw,
	.lb = lightrec_default_lb,
	.lh = lightrec_default_lh,
	.lw = lightrec_default_lw,
	.lwu = lightrec_default_lwu,
	.swu = lightrec_default_swu,
};

static void __segfault_cb(struct lightrec_state *state, u32 addr,
			  const struct block *block)
{
	lightrec_set_exit_flags(state, LIGHTREC_EXIT_SEGFAULT);
	pr_err("Segmentation fault in recompiled code: invalid "
	       "load/store at address "PC_FMT"\n", addr);
	if (block)
		pr_err("Was executing block "PC_FMT"\n", block->pc);
}

static void lightrec_swl(struct lightrec_state *state,
			 const struct lightrec_mem_map_ops *ops,
			 u32 opcode, void *host, u32 addr, u32 data)
{
	unsigned int shift = addr & 0x3;
	unsigned int mask = shift < 3 ? GENMASK(31, (shift + 1) * 8) : 0;
	u32 old_data;

	/* Align to 32 bits */
	addr &= ~3;
	host = (void *)((uintptr_t)host & ~3);

	old_data = ops->lw(state, opcode, host, addr);

	data = (data >> ((3 - shift) * 8)) | (old_data & mask);

	ops->sw(state, opcode, host, addr, data);
}

static void lightrec_swr(struct lightrec_state *state,
			 const struct lightrec_mem_map_ops *ops,
			 u32 opcode, void *host, u32 addr, u32 data)
{
	unsigned int shift = addr & 0x3;
	unsigned int mask = (1 << (shift * 8)) - 1;
	u32 old_data;

	/* Align to 32 bits */
	addr &= ~3;
	host = (void *)((uintptr_t)host & ~3);

	old_data = ops->lw(state, opcode, host, addr);

	data = (data << (shift * 8)) | (old_data & mask);

	ops->sw(state, opcode, host, addr, data);
}

static void lightrec_swc2(struct lightrec_state *state, union code op,
			  const struct lightrec_mem_map_ops *ops,
			  void *host, u32 addr)
{
	u32 data = lightrec_mfc2(state, op.i.rt);

	ops->sw(state, op.opcode, host, addr, data);
}

static u32 lightrec_lwl(struct lightrec_state *state,
			const struct lightrec_mem_map_ops *ops,
			u32 opcode, void *host, u32 addr, u32 data)
{
	unsigned int shift = addr & 0x3;
	unsigned int mask = (1 << (24 - shift * 8)) - 1;
	u32 old_data;

	/* Align to 32 bits */
	addr &= ~3;
	host = (void *)((uintptr_t)host & ~3);

	old_data = ops->lw(state, opcode, host, addr);

	return (data & mask) | (old_data << (24 - shift * 8));
}

static u32 lightrec_lwr(struct lightrec_state *state,
			const struct lightrec_mem_map_ops *ops,
			u32 opcode, void *host, u32 addr, u32 data)
{
	unsigned int shift = addr & 0x3;
	unsigned int mask = shift ? GENMASK(31, 32 - shift * 8) : 0;
	u32 old_data;

	/* Align to 32 bits */
	addr &= ~3;
	host = (void *)((uintptr_t)host & ~3);

	old_data = ops->lw(state, opcode, host, addr);

	return (data & mask) | (old_data >> (shift * 8));
}

static void lightrec_lwc2(struct lightrec_state *state, union code op,
			  const struct lightrec_mem_map_ops *ops,
			  void *host, u32 addr)
{
	u32 data = ops->lw(state, op.opcode, host, addr);

	lightrec_mtc2(state, op.i.rt, data);
}

static void lightrec_invalidate_map(struct lightrec_state *state,
		const struct lightrec_mem_map *map, u32 addr, u32 len)
{
	if (map == &state->maps[PSX_MAP_KERNEL_USER_RAM]) {
		memset(lut_address(state, lut_offset(addr)), 0,
		       ((len + 3) / 4) * lut_elm_size(state));
	}
}

static enum psx_map
lightrec_get_map_idx(struct lightrec_state *state, u32 kaddr)
{
	const struct lightrec_mem_map *map;
	unsigned int i;

	for (i = 0; i < state->nb_maps; i++) {
		map = &state->maps[i];

		if (kaddr >= map->pc && kaddr < map->pc + map->length)
			return (enum psx_map) i;
	}

	return PSX_MAP_UNKNOWN;
}

const struct lightrec_mem_map *
lightrec_get_map(struct lightrec_state *state, void **host, u32 kaddr)
{
	const struct lightrec_mem_map *map;
	enum psx_map idx;
	u32 addr;

	idx = lightrec_get_map_idx(state, kaddr);
	if (idx == PSX_MAP_UNKNOWN)
		return NULL;

	map = &state->maps[idx];
	addr = kaddr - map->pc;

	while (map->mirror_of)
		map = map->mirror_of;

	if (host)
		*host = map->address + addr;

	return map;
}

u32 lightrec_rw(struct lightrec_state *state, union code op, u32 base,
		u32 data, u32 *flags, struct block *block, u16 offset)
{
	const struct lightrec_mem_map *map;
	const struct lightrec_mem_map_ops *ops;
	u32 opcode = op.opcode;
	bool was_tagged = true;
	u16 old_flags;
	u32 addr;
	void *host;

	addr = kunseg(base + (s16) op.i.imm);

	map = lightrec_get_map(state, &host, addr);
	if (!map) {
		__segfault_cb(state, addr, block);
		return 0;
	}

	if (flags)
		was_tagged = LIGHTREC_FLAGS_GET_IO_MODE(*flags);

	if (likely(!map->ops)) {
		if (flags && !LIGHTREC_FLAGS_GET_IO_MODE(*flags)) {
			/* Force parallel port accesses as HW accesses, because
			 * the direct-I/O emitters can't differenciate it. */
			if (unlikely(map == &state->maps[PSX_MAP_PARALLEL_PORT]))
				*flags |= LIGHTREC_IO_MODE(LIGHTREC_IO_HW);
			/* If the base register is 0x0, be extra suspicious.
			 * Some games (e.g. Sled Storm) actually do segmentation
			 * faults by using uninitialized pointers, which are
			 * later initialized to point to hardware registers. */
			else if (op.i.rs && base == 0x0)
				*flags |= LIGHTREC_IO_MODE(LIGHTREC_IO_HW);
			else
				*flags |= LIGHTREC_IO_MODE(LIGHTREC_IO_DIRECT);
		}

		ops = &lightrec_default_ops;
	} else if (flags &&
		   LIGHTREC_FLAGS_GET_IO_MODE(*flags) == LIGHTREC_IO_DIRECT_HW) {
		ops = &lightrec_default_ops;
	} else {
		if (flags && !LIGHTREC_FLAGS_GET_IO_MODE(*flags))
			*flags |= LIGHTREC_IO_MODE(LIGHTREC_IO_HW);

		ops = map->ops;
	}

	if (!was_tagged && likely(!block_has_flag(block, BLOCK_NEVER_COMPILE))) {
		old_flags = block_set_flags(block, BLOCK_SHOULD_RECOMPILE);

		if (!(old_flags & BLOCK_SHOULD_RECOMPILE)) {
			pr_debug("Opcode of block at "PC_FMT" has been tagged"
				 " - flag for recompilation\n", block->pc);

			if (ENABLE_THREADED_COMPILER)
				lightrec_recompiler_add(state->rec, block);
			else
				lut_write(state, lut_offset(block->pc), NULL);
		}
	}

	switch (op.i.op) {
	case OP_SB:
		ops->sb(state, opcode, host, addr, data);
		return 0;
	case OP_SH:
		ops->sh(state, opcode, host, addr, data);
		return 0;
	case OP_SWL:
		lightrec_swl(state, ops, opcode, host, addr, data);
		return 0;
	case OP_SWR:
		lightrec_swr(state, ops, opcode, host, addr, data);
		return 0;
	case OP_SW:
		ops->sw(state, opcode, host, addr, data);
		return 0;
	case OP_SWC2:
		lightrec_swc2(state, op, ops, host, addr);
		return 0;
	case OP_LB:
		return (s32) (s8) ops->lb(state, opcode, host, addr);
	case OP_LBU:
		return ops->lb(state, opcode, host, addr);
	case OP_LH:
		return (s32) (s16) ops->lh(state, opcode, host, addr);
	case OP_LHU:
		return ops->lh(state, opcode, host, addr);
	case OP_LWC2:
		lightrec_lwc2(state, op, ops, host, addr);
		return 0;
	case OP_LWL:
		return lightrec_lwl(state, ops, opcode, host, addr, data);
	case OP_LWR:
		return lightrec_lwr(state, ops, opcode, host, addr, data);
	case OP_META_LWU:
		return ops->lwu(state, opcode, host, addr);
	case OP_META_SWU:
		ops->swu(state, opcode, host, addr, data);
		return 0;
	case OP_LW:
	default:
		return ops->lw(state, opcode, host, addr);
	}
}

static void lightrec_rw_helper(struct lightrec_state *state,
			       union code op, u32 *flags,
			       struct block *block, u16 offset)
{
	u32 ret = lightrec_rw(state, op, state->regs.gpr[op.i.rs],
			      state->regs.gpr[op.i.rt], flags, block, offset);

	switch (op.i.op) {
	case OP_LB:
	case OP_LBU:
	case OP_LH:
	case OP_LHU:
	case OP_LWL:
	case OP_LWR:
	case OP_LW:
	case OP_META_LWU:
		if (OPT_HANDLE_LOAD_DELAYS && unlikely(!state->in_delay_slot_n)) {
			state->temp_reg = ret;
			state->in_delay_slot_n = 0xff;
		} else if (op.i.rt) {
			state->regs.gpr[op.i.rt] = ret;
		}
		fallthrough;
	default:
		break;
	}
}

static void lightrec_rw_cb(struct lightrec_state *state, u32 arg)
{
	lightrec_rw_helper(state, (union code) arg, NULL, NULL, 0);
}

static void lightrec_rw_generic_cb(struct lightrec_state *state, u32 arg)
{
	struct block *block;
	struct opcode *op;
	u16 offset = (u16)arg;

	block = lightrec_find_block_from_lut(state->block_cache,
					     arg >> 16, state->curr_pc);
	if (unlikely(!block)) {
		pr_err("rw_generic: No block found in LUT for "PC_FMT" offset 0x%"PRIx16"\n",
			 state->curr_pc, offset);
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_SEGFAULT);
		return;
	}

	op = &block->opcode_list[offset];
	lightrec_rw_helper(state, op->c, &op->flags, block, offset);
}

static u32 clamp_s32(s32 val, s32 min, s32 max)
{
	return val < min ? min : val > max ? max : val;
}

static u16 load_u16(u32 *ptr)
{
	return ((struct u16x2 *) ptr)->l;
}

static void store_u16(u32 *ptr, u16 value)
{
	((struct u16x2 *) ptr)->l = value;
}

static u32 lightrec_mfc2(struct lightrec_state *state, u8 reg)
{
	s16 gteir1, gteir2, gteir3;

	switch (reg) {
	case 1:
	case 3:
	case 5:
	case 8:
	case 9:
	case 10:
	case 11:
		return (s32)(s16) load_u16(&state->regs.cp2d[reg]);
	case 7:
	case 16:
	case 17:
	case 18:
	case 19:
		return load_u16(&state->regs.cp2d[reg]);
	case 28:
	case 29:
		gteir1 = (s16) load_u16(&state->regs.cp2d[9]);
		gteir2 = (s16) load_u16(&state->regs.cp2d[10]);
		gteir3 = (s16) load_u16(&state->regs.cp2d[11]);

		return clamp_s32(gteir1 >> 7, 0, 0x1f) << 0 |
			clamp_s32(gteir2 >> 7, 0, 0x1f) << 5 |
			clamp_s32(gteir3 >> 7, 0, 0x1f) << 10;
	case 15:
		reg = 14;
		fallthrough;
	default:
		return state->regs.cp2d[reg];
	}
}

u32 lightrec_mfc(struct lightrec_state *state, union code op)
{
	u32 val;

	if (op.i.op == OP_CP0)
		return state->regs.cp0[op.r.rd];

	if (op.i.op == OP_SWC2) {
		val = lightrec_mfc2(state, op.i.rt);
	} else if (op.r.rs == OP_CP2_BASIC_MFC2)
		val = lightrec_mfc2(state, op.r.rd);
	else {
		val = state->regs.cp2c[op.r.rd];

		switch (op.r.rd) {
		case 4:
		case 12:
		case 20:
		case 26:
		case 27:
		case 29:
		case 30:
			val = (u32)(s16)val;
			fallthrough;
		default:
			break;
		}
	}

	if (state->ops.cop2_notify)
		(*state->ops.cop2_notify)(state, op.opcode, val);

	return val;
}

static void lightrec_mfc_cb(struct lightrec_state *state, union code op)
{
	u32 rt = lightrec_mfc(state, op);

	if (op.i.op == OP_SWC2)
		state->temp_reg = rt;
	else if (op.r.rt)
		state->regs.gpr[op.r.rt] = rt;
}

static void lightrec_mtc0(struct lightrec_state *state, u8 reg, u32 data)
{
	u32 status, oldstatus, cause;

	switch (reg) {
	case 1:
	case 4:
	case 8:
	case 14:
	case 15:
		/* Those registers are read-only */
		return;
	default:
		break;
	}

	if (reg == 12) {
		status = state->regs.cp0[12];
		oldstatus = status;

		if (status & ~data & BIT(16)) {
			state->ops.enable_ram(state, true);
			lightrec_invalidate_all(state);
		} else if (~status & data & BIT(16)) {
			state->ops.enable_ram(state, false);
		}
	}

	if (reg == 13) {
		state->regs.cp0[13] &= ~0x300;
		state->regs.cp0[13] |= data & 0x300;
	} else {
		state->regs.cp0[reg] = data;
	}

	if (reg == 12 || reg == 13) {
		cause = state->regs.cp0[13];
		status = state->regs.cp0[12];

		/* Handle software interrupts */
		if ((!!(status & cause & 0x300)) & status)
			lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);

		/* Handle hardware interrupts */
		if (reg == 12 && !(~status & 0x401) && (~oldstatus & 0x401))
			lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);
	}
}

static u32 count_leading_bits(s32 data)
{
#ifdef __has_builtin
#if __has_builtin(__builtin_clrsb)
	return 1 + __builtin_clrsb(data);
#endif
#endif
	data ^= data >> 31;
	return data ? clz32(data) : 32;
}

static void lightrec_mtc2(struct lightrec_state *state, u8 reg, u32 data)
{
	switch (reg) {
	case 15:
		state->regs.cp2d[12] = state->regs.cp2d[13];
		state->regs.cp2d[13] = state->regs.cp2d[14];
		state->regs.cp2d[14] = data;
		break;
	case 28:
		state->regs.cp2d[9] = (data << 7) & 0xf80;
		state->regs.cp2d[10] = (data << 2) & 0xf80;
		state->regs.cp2d[11] = (data >> 3) & 0xf80;
		break;
	case 31:
		return;
	case 30:
		state->regs.cp2d[31] = count_leading_bits((s32) data);
		fallthrough;
	default:
		state->regs.cp2d[reg] = data;
		break;
	}
}

static void lightrec_ctc2(struct lightrec_state *state, u8 reg, u32 data)
{
	psxCP2CtrlGen++;

	switch (reg) {
	case 4:
	case 12:
	case 20:
	case 26:
	case 27:
	case 29:
	case 30:
		store_u16(&state->regs.cp2c[reg], data);
		break;
	case 31:
		data = (data & 0x7ffff000) | !!(data & 0x7f87e000) << 31;
		fallthrough;
	default:
		state->regs.cp2c[reg] = data;
		break;
	}
}

void lightrec_mtc(struct lightrec_state *state, union code op, u8 reg, u32 data)
{
	if (op.i.op == OP_CP0) {
		lightrec_mtc0(state, reg, data);
	} else {
		if (op.i.op == OP_LWC2 || op.r.rs != OP_CP2_BASIC_CTC2)
			lightrec_mtc2(state, reg, data);
		else
			lightrec_ctc2(state, reg, data);

		if (state->ops.cop2_notify)
			(*state->ops.cop2_notify)(state, op.opcode, data);
	}
}

static void lightrec_mtc_cb(struct lightrec_state *state, u32 arg)
{
	union code op = (union code) arg;
	u32 data;
	u8 reg;

	if (op.i.op == OP_LWC2) {
		data = state->temp_reg;
		reg = op.i.rt;
	} else {
		data = state->regs.gpr[op.r.rt];
		reg = op.r.rd;
	}

	lightrec_mtc(state, op, reg, data);
}

void lightrec_rfe(struct lightrec_state *state)
{
	u32 status;

	/* Read CP0 Status register (r12) */
	status = state->regs.cp0[12];

	/* Switch the bits */
	status = ((status & 0x3c) >> 2) | (status & ~0xf);

	/* Write it back */
	lightrec_mtc0(state, 12, status);
}

void lightrec_cp(struct lightrec_state *state, union code op)
{
	if (op.i.op == OP_CP0) {
		pr_err("Invalid CP opcode to coprocessor #0\n");
		return;
	}

	(*state->ops.cop2_op)(state, op.opcode);
}

static void lightrec_cp_cb(struct lightrec_state *state, u32 arg)
{
	lightrec_cp(state, (union code) arg);
}

static struct block * lightrec_get_block(struct lightrec_state *state, u32 pc)
{
	struct block *block = lightrec_find_block(state->block_cache, pc);
	u8 old_flags;

	if (block && lightrec_block_is_outdated(state, block)) {
		pr_debug("Block at "PC_FMT" is outdated!\n", block->pc);

		old_flags = block_set_flags(block, BLOCK_IS_DEAD);
		if (!(old_flags & BLOCK_IS_DEAD)) {
			/* Make sure the recompiler isn't processing the block
			 * we'll destroy */
			if (ENABLE_THREADED_COMPILER)
				lightrec_recompiler_remove(state->rec, block);

			remove_from_code_lut(state->block_cache, block);

			if (ENABLE_THREADED_COMPILER) {
				lightrec_reaper_add(state->reaper,
						    lightrec_reap_block, block);
			} else {
				lightrec_unregister_block(state->block_cache, block);
				lightrec_free_block(state, block);
			}
		}

		block = NULL;
	}

	if (!block) {
		block = lightrec_precompile_block(state, pc);
		if (!block) {
			pr_err("Unable to recompile block at "PC_FMT"\n", pc);
			lightrec_set_exit_flags(state, LIGHTREC_EXIT_SEGFAULT);
			return NULL;
		}

		lightrec_register_block(state->block_cache, block);
	}

	return block;
}

/* THE ONE C FUNCTION THE DISPATCHER CALLS ON ITS HOT PATH.
 *
 * `dispatch.S` reaches this through a patchable literal slot when the code
 * table has no entry for a PC. Its contract is exactly what the assembly
 * assumes: return the block's entry point or NULL, and leave the PC actually
 * arrived at in `state->curr_pc` -- which the dispatcher re-reads rather than
 * trusting its own exit register, because this may have run the interpreter
 * and moved it. */
void * fgl_get_next_block(struct lightrec_state *state, u32 pc)
{
	struct block *block;
	{ static unsigned c; if (c < 40) { c++;
	  fprintf(stderr, "gnb %u pc=%08x cyc=%u/%u flags=%x\n", c, pc,
		  state->current_cycle, state->target_cycle, state->exit_flags); } }
	bool should_recompile;
	void *func;
	int err;

	do {
		func = lut_read(state, lut_offset(pc));
		if (func && func != state->get_next_block)
			break;

		block = lightrec_get_block(state, pc);

		if (unlikely(!block))
			break;

		if (OPT_REPLACE_MEMSET &&
		    block_has_flag(block, BLOCK_IS_MEMSET)) {
			func = state->memset_func;
			break;
		}

		should_recompile = block_has_flag(block, BLOCK_SHOULD_RECOMPILE) &&
			!block_has_flag(block, BLOCK_NEVER_COMPILE) &&
			!block_has_flag(block, BLOCK_IS_DEAD);

		if (unlikely(should_recompile)) {
			pr_debug("Block at "PC_FMT" should recompile\n", pc);

			if (ENABLE_THREADED_COMPILER) {
				lightrec_recompiler_add(state->rec, block);
			} else {
				err = lightrec_compile_block(state->cstate, block);
				if (err == -ENOMEM) {
					state->exit_flags = LIGHTREC_EXIT_NOMEM;
					return NULL;
				}
			}
		}

		if (ENABLE_THREADED_COMPILER && likely(!should_recompile))
			func = lightrec_recompiler_run_first_pass(state, block, &pc);
		else
			func = block->function;

		if (likely(func))
			break;

		if (unlikely(block_has_flag(block, BLOCK_NEVER_COMPILE))) {
			pc = lightrec_emulate_block(state, block, pc);

		} else if (!ENABLE_THREADED_COMPILER) {
			/* Block wasn't compiled yet - run the interpreter */
			if (block_has_flag(block, BLOCK_FULLY_TAGGED))
				pr_debug("Block fully tagged, skipping first pass\n");
			else if (ENABLE_FIRST_PASS && likely(!should_recompile))
				pc = lightrec_emulate_block(state, block, pc);

			/* Then compile it using the profiled data */
			err = lightrec_compile_block(state->cstate, block);
			if (err == -ENOMEM) {
				state->exit_flags = LIGHTREC_EXIT_NOMEM;
				return NULL;
			}
		} else if (unlikely(block_has_flag(block, BLOCK_IS_DEAD))) {
			/*
			 * If the block is dead but has never been compiled,
			 * then its function pointer is NULL and we cannot
			 * execute the block. In that case, reap all the dead
			 * blocks now, and in the next loop we will create a
			 * new block.
			 */
			lightrec_reaper_reap(state->reaper);
		} else {
			lightrec_recompiler_add(state->rec, block);
		}
	} while (state->exit_flags == LIGHTREC_EXIT_NORMAL
		 && state->current_cycle < state->target_cycle);

	state->curr_pc = pc;
	return func;
}

void * lightrec_alloc_code(struct lightrec_state *state, size_t size)
{
	void *code;

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_lock(state);

	code = tlsf_malloc(state->tlsf, size);

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_unlock(state);

	return code;
}

static void lightrec_realloc_code(struct lightrec_state *state,
				  void *ptr, size_t size)
{
	/* NOTE: 'size' MUST be smaller than the size specified during
	 * the allocation. */

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_lock(state);

	tlsf_realloc(state->tlsf, ptr, size);

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_unlock(state);
}

void lightrec_free_code(struct lightrec_state *state, void *ptr)
{
	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_lock(state);

	tlsf_free(state->tlsf, ptr);

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_unlock(state);
}




static u32 lightrec_memset(struct lightrec_state *state)
{
	u32 kunseg_pc = kunseg(state->regs.gpr[4]);
	void *host;
	const struct lightrec_mem_map *map = lightrec_get_map(state, &host, kunseg_pc);
	u32 length = state->regs.gpr[5] * 4;

	if (!map) {
		pr_err("Unable to find memory map for memset target address "PC_FMT"\n",
		       kunseg_pc);
		return 0;
	}

	pr_debug("Calling host memset, "PC_FMT" (host address 0x%"PRIxPTR") for %"PRIu32" bytes\n",
		 kunseg_pc, (uintptr_t)host, length);
	memset(host, 0, length);

	if (!(state->opt_flags & LIGHTREC_OPT_INV_DMA_ONLY))
		lightrec_invalidate_map(state, map, kunseg_pc, length);

	/* Rough estimation of the number of cycles consumed */
	return 8 + 5 * (length  + 3 / 4);
}

static u32 lightrec_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg)
{
	struct block *block;
	union code first_op;

	first_op = lightrec_read_opcode(state, pc);

	if (likely(!opcode_reads_register(first_op, reg))) {
		state->regs.gpr[reg] = state->temp_reg;
	} else {
		block = lightrec_get_block(state, pc);
		if (unlikely(!block)) {
			pr_err("Unable to get block at "PC_FMT"\n", pc);
			lightrec_set_exit_flags(state, LIGHTREC_EXIT_SEGFAULT);
			pc = 0;
		} else {
			pc = lightrec_handle_load_delay(state, block, pc, reg);
		}
	}

	return pc;
}





union code lightrec_read_opcode(struct lightrec_state *state, u32 pc)
{
	void *host = NULL;

	lightrec_get_map(state, &host, kunseg(pc));

	const u32 *code = (u32 *)host;
	return (union code) LE32TOH(*code);
}

unsigned int lightrec_cycles_of_opcode(const struct lightrec_state *state,
				       union code code)
{
	return state->cycles_per_op;
}

void lightrec_free_opcode_list(struct lightrec_state *state, struct opcode *ops)
{
	struct opcode_list *list = container_of(ops, struct opcode_list, ops);

	lightrec_free(state, MEM_FOR_IR,
		      sizeof(*list) + list->nb_ops * sizeof(struct opcode),
		      list);
}

static unsigned int lightrec_get_mips_block_len(const u32 *src)
{
	unsigned int i;
	union code c;

	for (i = 1; ; i++) {
		c.opcode = LE32TOH(*src++);

		if (is_syscall(c))
			return i;

		if (c.i.op == OP_META_BIOS)
			return i;

		if (is_unconditional_jump(c))
			return i + 1;
	}
}

static struct opcode * lightrec_disassemble(struct lightrec_state *state,
					    const u32 *src, unsigned int *len)
{
	struct opcode_list *list;
	unsigned int i, length;

	length = lightrec_get_mips_block_len(src);

	list = lightrec_malloc(state, MEM_FOR_IR,
			       sizeof(*list) + sizeof(struct opcode) * length);
	if (!list) {
		pr_err("Unable to allocate memory\n");
		return NULL;
	}

	list->nb_ops = (u16) length;

	for (i = 0; i < length; i++) {
		list->ops[i].opcode = LE32TOH(src[i]);
		list->ops[i].flags = 0;
	}

	*len = length * sizeof(u32);

	return list->ops;
}

static struct block * lightrec_precompile_block(struct lightrec_state *state,
						u32 pc)
{
	struct opcode *list;
	struct block *block;
	void *host, *addr;
	const struct lightrec_mem_map *map = lightrec_get_map(state, &host, kunseg(pc));
	const u32 *code = (u32 *) host;
	unsigned int length;
	bool fully_tagged;
	u8 block_flags = 0;

	if (!map)
		return NULL;

	block = lightrec_malloc(state, MEM_FOR_IR, sizeof(*block));
	if (!block) {
		pr_err("Unable to recompile block: Out of memory\n");
		return NULL;
	}

	list = lightrec_disassemble(state, code, &length);
	if (!list) {
		lightrec_free(state, MEM_FOR_IR, sizeof(*block), block);
		return NULL;
	}

	block->pc = pc;
	block->function = NULL;
	block->opcode_list = list;
	block->code = code;
	block->next = NULL;
	block->flags = 0;
	block->code_size = 0;
	block->precompile_date = state->current_cycle;
	block->nb_ops = length / sizeof(u32);

	lightrec_optimize(state, block);

	length = block->nb_ops * sizeof(u32);

	lightrec_register(MEM_FOR_MIPS_CODE, length);

	if (ENABLE_DISASSEMBLER) {
		pr_debug("Disassembled block at "PC_FMT"\n", block->pc);
		lightrec_print_disassembly(block, code);
	}

	pr_debug("Block size: %hu opcodes\n", block->nb_ops);

	fully_tagged = lightrec_block_is_fully_tagged(block);
	if (fully_tagged)
		block_flags |= BLOCK_FULLY_TAGGED;

	if (block_flags)
		block_set_flags(block, block_flags);

	block->hash = lightrec_calculate_block_hash(block);

	if (OPT_REPLACE_MEMSET && block_has_flag(block, BLOCK_IS_MEMSET))
		addr = state->memset_func;
	else
		addr = state->get_next_block;
	lut_write(state, lut_offset(pc), addr);

	pr_debug("Blocks created: %u\n", ++state->nb_precompile);

	return block;
}

static bool lightrec_block_is_fully_tagged(const struct block *block)
{
	const struct opcode *op;
	unsigned int i;

	for (i = 0; i < block->nb_ops; i++) {
		op = &block->opcode_list[i];

		/* If we have one branch that must be emulated, we cannot trash
		 * the opcode list. */
		if (should_emulate(op))
			return false;

		/* Check all loads/stores of the opcode list and mark the
		 * block as fully compiled if they all have been tagged. */
		switch (op->c.i.op) {
		case OP_LB:
		case OP_LH:
		case OP_LWL:
		case OP_LW:
		case OP_LBU:
		case OP_LHU:
		case OP_LWR:
		case OP_SB:
		case OP_SH:
		case OP_SWL:
		case OP_SW:
		case OP_SWR:
		case OP_LWC2:
		case OP_SWC2:
		case OP_META_LWU:
		case OP_META_SWU:
			if (!LIGHTREC_FLAGS_GET_IO_MODE(op->flags))
				return false;
			fallthrough;
		default:
			continue;
		}
	}

	return true;
}

static void lightrec_reap_block(struct lightrec_state *state, void *data)
{
	struct block *block = data;

	pr_debug("Reap dead block at "PC_FMT"\n", block->pc);
	lightrec_unregister_block(state->block_cache, block);
	lightrec_free_block(state, block);
}

static void lightrec_free_function(struct lightrec_state *state, void *fn)
{
	if (ENABLE_CODE_BUFFER && state->tlsf) {
		pr_debug("Freeing code block at 0x%" PRIxPTR "\n", (uintptr_t) fn);
		lightrec_free_code(state, fn);
	}
}

static void lightrec_reap_function(struct lightrec_state *state, void *data)
{
	lightrec_free_function(state, data);
}

static void lightrec_reap_opcode_list(struct lightrec_state *state, void *data)
{
	lightrec_free_opcode_list(state, data);
}

int lightrec_compile_block(struct lightrec_cstate *cstate,
			   struct block *block)
{
	struct lightrec_state *state = cstate->state;
	bool fully_tagged = false;
	void *old_fn, *new_fn;
	size_t old_code_size;
	unsigned int i;
	u8 old_flags;
	int err;

	fully_tagged = lightrec_block_is_fully_tagged(block);
	if (fully_tagged)
		block_set_flags(block, BLOCK_FULLY_TAGGED);

	old_fn = block->function;
	old_code_size = block->code_size;


	/* WHERE THE WHOLE OF GNU LIGHTNING USED TO BE.
	 *
	 * What stood here was a compilation context, a register-cache reset, a
	 * prologue, a per-opcode loop into `emitter.c`, a local-branch patch
	 * pass and a final `jit_emit()`.  fgl replaces all of it with one call:
	 * it owns the lowering, the register allocation, the emission and the
	 * placement of the code in the arena, and it takes lightrec's already
	 * optimised opcode list as its input so that none of the optimiser's
	 * proofs are thrown away and re-derived.
	 *
	 * `cstate->targets[]` and `cstate->local_branches[]` are left empty.
	 * They existed so that a branch INSIDE a block could be patched to an
	 * address inside the same block and published in the code table; fgl
	 * ends a block at the first control transfer instead, so every branch
	 * leaves through the dispatcher and every target becomes a block of its
	 * own.  Correct, and slower than it needs to be -- see the note in
	 * fgl_compile_block.
	 */
	new_fn = fgl_compile_block(cstate, block, &block->code_size, &err);
	if (!new_fn) {
		if (err == -ENOMEM) {
			if (!ENABLE_THREADED_COMPILER)
				pr_err("Code arena full compiling block at "
				       PC_FMT"\n", block->pc);
			return -ENOMEM;
		}

		/* fgl cannot lower this block.  A HOLE TO FILL, and the flag
		 * is not a fallback path -- it is what stops the emulator from
		 * spending the rest of its life recompiling the same refusal.
		 * -ENOMEM here means "arena full" to the caller, which flushes
		 * the whole block cache and asks again, gets the same refusal,
		 * and flushes again.  Interpreting the block instead keeps the
		 * machine alive long enough for the message above to be read
		 * and the hole to be filled. */
		pr_err("fgl cannot lower the block at "PC_FMT" -- interpreting "
		       "it; this is a hole in fgl, not a design\n", block->pc);
		block_set_flags(block, BLOCK_NEVER_COMPILE);
		return -EINVAL;
	}

	/* Pause the reaper, because lightrec_reset_lut_offset() may try to set
	 * the old block->function pointer to the code LUT. */
	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_pause(state->reaper);

	block->function = new_fn;
	block_clear_flags(block, BLOCK_SHOULD_RECOMPILE);

	/* Add compiled function to the LUT */
	lut_write(state, lut_offset(block->pc), block->function);

	/* The pass that walked `cstate->targets[]` -- marking blocks covered by
	 * this one as dead and publishing each internal branch target into the
	 * code table -- is gone with the targets themselves. Nothing here can
	 * cover another block any more, because a block now ends at its first
	 * control transfer. */

	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_continue(state->reaper);

	if (fully_tagged)
		old_flags = block_set_flags(block, BLOCK_NO_OPCODE_LIST);

	if (fully_tagged && !(old_flags & BLOCK_NO_OPCODE_LIST)) {
		pr_debug("Block "PC_FMT" is fully tagged"
			 " - free opcode list\n", block->pc);

		if (ENABLE_THREADED_COMPILER) {
			lightrec_reaper_add(state->reaper,
					    lightrec_reap_opcode_list,
					    block->opcode_list);
		} else {
			lightrec_free_opcode_list(state, block->opcode_list);
		}
	}

	if (old_fn) {
		/* Only the code is reaped now. There is no compilation context
		 * to destroy: fgl's emitter lives on the stack for the length
		 * of one call and owns nothing that outlives it. */
		pr_debug("Block "X32_FMT" recompiled, reaping old code.\n",
			 block->pc);

		if (ENABLE_THREADED_COMPILER)
			lightrec_reaper_add(state->reaper,
					    lightrec_reap_function, old_fn);
		else
			lightrec_free_function(state, old_fn);

		lightrec_unregister(MEM_FOR_CODE, old_code_size);
	}

	pr_debug("Blocks compiled: %u\n", ++state->nb_compile);

	return 0;
}

static void lightrec_print_info(struct lightrec_state *state)
{
	if ((state->current_cycle & ~0xfffffff) != state->old_cycle_counter) {
		pr_info("Lightrec RAM usage: IR %u KiB, CODE %u KiB, "
			"MIPS %u KiB, TOTAL %u KiB, avg. IPI %f\n",
			lightrec_get_mem_usage(MEM_FOR_IR) / 1024,
			lightrec_get_mem_usage(MEM_FOR_CODE) / 1024,
			lightrec_get_mem_usage(MEM_FOR_MIPS_CODE) / 1024,
			lightrec_get_total_mem_usage() / 1024,
		       lightrec_get_average_ipi());
		state->old_cycle_counter = state->current_cycle & ~0xfffffff;
	}
}

/* WHAT dispatch.S CALLS, AND WHY THESE WRAPPERS EXIST AT ALL.
 *
 * `lightrec_memset` and `lightrec_check_load_delay` are static in this file,
 * so the assembly cannot name them however it is written. `lightrec_emulate_block`
 * is not, and is wrapped anyway so that all three slots in dispatch.S are
 * filled from one place and by one convention.
 *
 * The cycle contract differs between them and it is the dispatcher, not these,
 * that implements it: the two that spend cycles inside C are entered and left
 * through `.Lsync_out`/`.Lsync_in`, and memset is not, because it reports its
 * cost as a return value and touches neither counter. */
u32 fgl_memset(struct lightrec_state *state)
{
	return lightrec_memset(state);
}

u32 fgl_emulate_block(struct lightrec_state *state, struct block *block, u32 pc)
{
	return lightrec_emulate_block(state, block, pc);
}

u32 fgl_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg)
{
	return lightrec_check_load_delay(state, pc, reg);
}

u32 lightrec_execute(struct lightrec_state *state, u32 pc, u32 target_cycle)
{
	void *block_trace;
	s32 cycles_delta;

	state->exit_flags = LIGHTREC_EXIT_NORMAL;

	/* Handle the cycle counter overflowing */
	if (unlikely(target_cycle < state->current_cycle))
		target_cycle = UINT_MAX;

	state->target_cycle = target_cycle;
	state->curr_pc = pc;

	block_trace = fgl_get_next_block(state, pc);
	if (block_trace) {
		cycles_delta = state->target_cycle - state->current_cycle;

		/* Straight into the hand-written dispatcher. Its signature is
		 * the one lightrec's generated dispatcher had, third argument
		 * included -- which fgl ignores, looking the first block up
		 * through the table like any other. */
		cycles_delta = fgl_dispatch(state, state->curr_pc,
					    block_trace, cycles_delta);

		state->current_cycle = state->target_cycle - cycles_delta;
	}

	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_reap(state->reaper);

	if (LOG_LEVEL >= INFO_L)
		lightrec_print_info(state);

	return state->curr_pc;
}

u32 lightrec_run_interpreter(struct lightrec_state *state, u32 pc,
			     u32 target_cycle)
{
	struct block *block;

	state->exit_flags = LIGHTREC_EXIT_NORMAL;
	state->target_cycle = target_cycle;

	do {
		block = lightrec_get_block(state, pc);
		if (!block)
			break;

		pc = lightrec_emulate_block(state, block, pc);

		if (ENABLE_THREADED_COMPILER)
			lightrec_reaper_reap(state->reaper);
	} while (state->current_cycle < state->target_cycle);

	if (LOG_LEVEL >= INFO_L)
		lightrec_print_info(state);

	return pc;
}

void lightrec_free_block(struct lightrec_state *state, struct block *block)
{
	u8 old_flags;

	lightrec_unregister(MEM_FOR_MIPS_CODE, block->nb_ops * sizeof(u32));
	old_flags = block_set_flags(block, BLOCK_NO_OPCODE_LIST);

	if (!(old_flags & BLOCK_NO_OPCODE_LIST))
		lightrec_free_opcode_list(state, block->opcode_list);
	if (block->function) {
		lightrec_free_function(state, block->function);
		lightrec_unregister(MEM_FOR_CODE, block->code_size);
	}
	lightrec_free(state, MEM_FOR_IR, sizeof(*block), block);
}

struct lightrec_cstate * lightrec_create_cstate(struct lightrec_state *state)
{
	struct lightrec_cstate *cstate;

	cstate = lightrec_malloc(state, MEM_FOR_LIGHTREC, sizeof(*cstate));
	if (!cstate)
		return NULL;

	cstate->state = state;

	return cstate;
}

void lightrec_free_cstate(struct lightrec_cstate *cstate)
{
	lightrec_free(cstate->state, MEM_FOR_LIGHTREC, sizeof(*cstate), cstate);
}

/* Fill in the products a block's cycle charge is looked up from.
 *
 * A block charges `nb_ops * cycles_per_op`.  `cycles_per_op` is a runtime
 * value -- the frontend changes it when the guest's clock scaling changes --
 * so the charge cannot be an immediate, and the obvious lowering puts a
 * literal in every block's constant pool.  Precomputing the products instead
 * makes the charge a GBR-relative load whose displacement IS the instruction
 * count, costing two instructions and no pool word (src/fgl/fgl_state.h).
 *
 * Called from everywhere `cycles_per_op` is written, and nowhere else. */
static void lightrec_fill_cycle_table(struct lightrec_state *state)
{
	unsigned int i;

	for (i = 0; i < LIGHTREC_CYCLE_ENTRIES; i++)
		state->cycle_table[i] = i * state->cycles_per_op;
}

struct lightrec_state * lightrec_init(char *argv0,
				      const struct lightrec_mem_map *maps,
				      size_t nb,
				      const struct lightrec_ops *ops)
{
	const struct lightrec_mem_map *codebuf_map = &maps[PSX_MAP_CODE_BUFFER];
	const struct lightrec_mem_map *map;
	struct lightrec_state *state;
	uintptr_t addr;
	void *tlsf = NULL;
	bool with_32bit_lut = false;
	size_t lut_size;

	/* Sanity-check ops */
	if (!ops || !ops->cop2_op || !ops->enable_ram) {
		pr_err("Missing callbacks in lightrec_ops structure\n");
		return NULL;
	}

	if (ops->cop2_notify)
		pr_debug("Optional cop2_notify callback in lightrec_ops\n");
	else
		pr_debug("No optional cop2_notify callback in lightrec_ops\n");

	if (ENABLE_CODE_BUFFER && nb > PSX_MAP_CODE_BUFFER
	    && codebuf_map->address) {
		tlsf = tlsf_create_with_pool(codebuf_map->address,
					     codebuf_map->length);
		if (!tlsf) {
			pr_err("Unable to initialize code buffer\n");
			return NULL;
		}

		if (__WORDSIZE == 64) {
			addr = (uintptr_t) codebuf_map->address + codebuf_map->length - 1;
			with_32bit_lut = addr == (u32) addr;
		}
	}

	if (with_32bit_lut)
		lut_size = CODE_LUT_SIZE * 4;
	else
		lut_size = CODE_LUT_SIZE * sizeof(void *);

	state = calloc(1, sizeof(*state) + lut_size);
	if (!state)
		goto err_free_tlsf;

	lightrec_register(MEM_FOR_LIGHTREC, sizeof(*state) + lut_size);

	state->tlsf = tlsf;
	state->with_32bit_lut = with_32bit_lut;
	state->in_delay_slot_n = 0xff;
	state->cycles_per_op = 2;
	lightrec_fill_cycle_table(state);

	state->block_cache = lightrec_blockcache_init(state);
	if (!state->block_cache)
		goto err_free_state;

	if (ENABLE_THREADED_COMPILER) {
		state->rec = lightrec_recompiler_init(state);
		if (!state->rec)
			goto err_free_block_cache;

		state->reaper = lightrec_reaper_init(state);
		if (!state->reaper)
			goto err_free_recompiler;
	} else {
		state->cstate = lightrec_create_cstate(state);
		if (!state->cstate)
			goto err_free_block_cache;
	}

	state->nb_maps = nb;
	state->maps = maps;

	memcpy(&state->ops, ops, sizeof(*ops));

	/* THE DISPATCHER AND THE SERVICE TRAMPOLINE ARE NOT GENERATED ANY MORE.
	 *
	 * lightrec built both at run time, as Lightning IR, because they had to
	 * agree with whatever registers its register allocator had picked on
	 * this host. fgl's register contract is fixed and written down
	 * (src/fgl/fgl.h), so both are ordinary hand-written assembly compiled
	 * into the image, and what used to be a code generator is now a symbol.
	 *
	 * `c_wrappers[]` is gone with them: fgl reaches C by materialising the
	 * callee's address as a literal and calling a fixed shim, so there is
	 * no table to index and no selector to pack. */
	state->memset_func      = fgl_dispatch_memset;
	state->get_next_block   = fgl_dispatch_compile;
	state->interpreter_func = fgl_dispatch_interpreter;
	state->ds_check_func    = fgl_dispatch_ds_check;

	/* What generated code reads out of the state block. All three are
	 * inside GBR's 1020-byte reach; fgl_lightrec.c asserts it. */
	state->dispatch  = (u32)(uintptr_t)fgl_dispatch_loop;
	state->lut_base  = (u32)(uintptr_t)state->code_lut;
	state->addr_mask = 0x1fffffff;

	map = &maps[PSX_MAP_BIOS];
	state->offset_bios = (uintptr_t)map->address - map->pc;

	map = &maps[PSX_MAP_SCRATCH_PAD];
	state->offset_scratch = (uintptr_t)map->address - map->pc;

	map = &maps[PSX_MAP_HW_REGISTERS];
	state->offset_io = (uintptr_t)map->address - map->pc;

	map = &maps[PSX_MAP_KERNEL_USER_RAM];
	state->offset_ram = (uintptr_t)map->address - map->pc;

	if (maps[PSX_MAP_MIRROR1].address == map->address + 0x200000 &&
	    maps[PSX_MAP_MIRROR2].address == map->address + 0x400000 &&
	    maps[PSX_MAP_MIRROR3].address == map->address + 0x600000)
		state->mirrors_mapped = true;

	if (state->offset_bios == 0 &&
	    state->offset_scratch == 0 &&
	    state->offset_ram == 0 &&
	    state->offset_io == 0 &&
	    state->mirrors_mapped) {
		pr_info("Memory map is perfect. Emitted code will be best.\n");
	} else {
		pr_info("Memory map is sub-par. Emitted code will be slow.\n");
	}

	if (state->with_32bit_lut)
		pr_info("Using 32-bit LUT\n");

	return state;

err_free_reaper:
	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_destroy(state->reaper);
err_free_recompiler:
	if (ENABLE_THREADED_COMPILER)
		lightrec_free_recompiler(state->rec);
	else
		lightrec_free_cstate(state->cstate);
err_free_block_cache:
	lightrec_free_block_cache(state->block_cache);
err_free_state:
	lightrec_unregister(MEM_FOR_LIGHTREC, sizeof(*state) +
			    lut_elm_size(state) * CODE_LUT_SIZE);
	free(state);
err_free_tlsf:
	if (ENABLE_CODE_BUFFER && tlsf)
		tlsf_destroy(tlsf);
	return NULL;
}

void lightrec_destroy(struct lightrec_state *state)
{
	/* Force a print info on destroy*/
	state->current_cycle = ~state->current_cycle;
	lightrec_print_info(state);

	lightrec_free_block_cache(state->block_cache);

	if (ENABLE_THREADED_COMPILER) {
		lightrec_free_recompiler(state->rec);
		lightrec_reaper_destroy(state->reaper);
	} else {
		lightrec_free_cstate(state->cstate);
	}

	if (ENABLE_CODE_BUFFER && state->tlsf)
		tlsf_destroy(state->tlsf);

	lightrec_unregister(MEM_FOR_LIGHTREC, sizeof(*state) +
			    lut_elm_size(state) * CODE_LUT_SIZE);
	free(state);
}

/* An IO_HW tag records where the op's FIRST access landed (or that its
 * base register was zero) - later executions may point anywhere, and the
 * BIOS really does write the exception vectors through ops tagged HW.
 * Only the I/O window takes the fast path; everything else goes through
 * the full map dispatch in lightrec_rw. */
static inline const struct lightrec_mem_map_ops *
hw_shim_ops(struct lightrec_state *state, u32 kaddr)
{
	const struct lightrec_mem_map *map = &state->maps[PSX_MAP_HW_REGISTERS];

	if (likely(kaddr - map->pc < map->length))
		return map->ops;

	return NULL;
}

static u32 hw_shim_slow(struct lightrec_state *state, u32 op, u32 addr,
			u32 data)
{
	return lightrec_rw(state, (union code){ .i.op = op }, addr, data,
			   NULL, NULL, 0);
}

u32 lightrec_hw_lb(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u32)(s32)(s8)ops->lb(state, 0, NULL, kaddr);
	return hw_shim_slow(state, OP_LB, addr, 0);
}

u32 lightrec_hw_lbu(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u8)ops->lb(state, 0, NULL, kaddr);
	return hw_shim_slow(state, OP_LBU, addr, 0);
}

u32 lightrec_hw_lh(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u32)(s32)(s16)ops->lh(state, 0, NULL, kaddr);
	return hw_shim_slow(state, OP_LH, addr, 0);
}

u32 lightrec_hw_lhu(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return (u16)ops->lh(state, 0, NULL, kaddr);
	return hw_shim_slow(state, OP_LHU, addr, 0);
}

u32 lightrec_hw_lw(u32 addr, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		return ops->lw(state, 0, NULL, kaddr);
	return hw_shim_slow(state, OP_LW, addr, 0);
}

void lightrec_hw_sb(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sb(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SB, addr, val);
}

void lightrec_hw_sh(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sh(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SH, addr, val);
}

void lightrec_hw_sw(u32 addr, u32 val, struct lightrec_state *state)
{
	u32 kaddr = kunseg(addr);
	const struct lightrec_mem_map_ops *ops = hw_shim_ops(state, kaddr);

	if (likely(ops))
		ops->sw(state, 0, NULL, kaddr, val);
	else
		hw_shim_slow(state, OP_SW, addr, val);
}

void lightrec_invalidate(struct lightrec_state *state, u32 addr, u32 len)
{
	u32 kaddr = kunseg(addr & ~0x3);
	enum psx_map idx = lightrec_get_map_idx(state, kaddr);

	switch (idx) {
	case PSX_MAP_MIRROR1:
	case PSX_MAP_MIRROR2:
	case PSX_MAP_MIRROR3:
		/* Handle mirrors */
		kaddr &= RAM_SIZE - 1;
		fallthrough;
	case PSX_MAP_KERNEL_USER_RAM:
		break;
	default:
		return;
	}

	memset(lut_address(state, lut_offset(kaddr)), 0,
	       ((len + 3) / 4) * lut_elm_size(state));
}

void lightrec_invalidate_all(struct lightrec_state *state)
{
	memset(state->code_lut, 0, lut_elm_size(state) * CODE_LUT_SIZE);
}

void lightrec_set_unsafe_opt_flags(struct lightrec_state *state, u32 flags)
{
	if ((flags ^ state->opt_flags) & LIGHTREC_OPT_INV_DMA_ONLY)
		lightrec_invalidate_all(state);

	state->opt_flags = flags;
}

void lightrec_set_exit_flags(struct lightrec_state *state, u32 flags)
{
	if (flags != LIGHTREC_EXIT_NORMAL) {
		state->exit_flags |= flags;
		state->target_cycle = state->current_cycle;
	}
}

u32 lightrec_exit_flags(struct lightrec_state *state)
{
	return state->exit_flags;
}

u32 lightrec_current_cycle_count(const struct lightrec_state *state)
{
	return state->current_cycle;
}

void lightrec_reset_cycle_count(struct lightrec_state *state, u32 cycles)
{
	state->current_cycle = cycles;

	if (state->target_cycle < cycles)
		state->target_cycle = cycles;
}

void lightrec_set_target_cycle_count(struct lightrec_state *state, u32 cycles)
{
	if (state->exit_flags == LIGHTREC_EXIT_NORMAL) {
		if (cycles < state->current_cycle)
			cycles = state->current_cycle;

		state->target_cycle = cycles;
	}
}

struct lightrec_registers * lightrec_get_registers(struct lightrec_state *state)
{
	return &state->regs;
}

void lightrec_set_cycles_per_opcode(struct lightrec_state *state, u32 cycles)
{
	if (state->cycles_per_op == cycles)
		return;

	state->cycles_per_op = cycles;
	lightrec_fill_cycle_table(state);

	if (ENABLE_THREADED_COMPILER) {
		lightrec_recompiler_pause(state->rec);
		lightrec_reaper_reap(state->reaper);
	}

	lightrec_invalidate_all(state);
	lightrec_free_all_blocks(state->block_cache);

	if (ENABLE_THREADED_COMPILER)
		lightrec_recompiler_unpause(state->rec);
}
