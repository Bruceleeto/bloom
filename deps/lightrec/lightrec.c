// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Copyright (C) 2014-2021 Paul Cercueil <paul@crapouillou.net>
 */

#if !defined(LIGHTREC_NO_LIGHTNING)
#include "arch.h"
#endif
#include "blockcache.h"
#include "debug.h"
#include "disassembler.h"
#if !defined(LIGHTREC_NO_LIGHTNING)
#include "emitter.h"
#endif
#include "interpreter.h"
#include "lightrec-config.h"
#include "lightrec-private.h"
#if !defined(LIGHTREC_NO_LIGHTNING)
#include "lightning-wrapper.h"
#endif
#include "lightrec.h"
#include "memmanager.h"
#include "reaper.h"
#include "recompiler.h"
#if !defined(LIGHTREC_NO_LIGHTNING)
#include "regcache.h"
#endif
#include "optimizer.h"
#include "tlsf/tlsf.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#if ENABLE_THREADED_COMPILER
#include <stdatomic.h>
#endif
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Bumped on every write to a COP2 control register, wherever the write comes
 * from.  Defined by the core's GTE (libpcsxcore/gte.c); declared here because
 * this library does not include the core's headers.
 */
extern uint32_t psxCP2Gen[2];
#define psxCP2CtrlGen psxCP2Gen[0]

#include "prof.h"

#ifdef LIGHTREC_SH4_INTERP
#include "../../libpcsxcore/sh4/sh4_glue.h"
#endif

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
		/* UNLINK FIRST, WHILE THE SITES ARE STILL OURS.  A patched
		 * link is a branch baked into another block with nothing left
		 * to redirect it, so clearing the slot alone leaves the stale
		 * code reachable.  Only the window this is about: tearing
		 * down every link in the program on a four-byte guest store
		 * threw away the whole graph. */
		if (state->backend_ops->unlink_range)
			state->backend_ops->unlink_range(state,
					lut_offset(addr), (len + 3) / 4,
					LIGHTREC_UNLINK_INV_MAP);

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
			else {
				/* Same reason as remove_from_code_lut: a
				 * patched link does not go through the slot,
				 * so clearing the slot alone would leave the
				 * stale code reachable. */
				if (state->backend_ops->unlink_range)
					state->backend_ops->unlink_range(state,
						lut_offset(block->pc),
						block->nb_ops,
						LIGHTREC_UNLINK_SMC);
				lut_write(state, lut_offset(block->pc), NULL);
			}
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

#if !defined(LIGHTREC_NO_LIGHTNING)
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

/* The two callbacks below, not lightrec_rw() itself: these are what emitted
 * code calls, and lightrec_rw() has a return in every arm of a large switch. */

static void lightrec_rw_cb(struct lightrec_state *state, u32 arg)
{
	prof_enter(PROF_IO);
	lightrec_rw_helper(state, (union code) arg, NULL, NULL, 0);
	prof_leave();
}

static void lightrec_rw_generic_cb(struct lightrec_state *state, u32 arg)
{
	struct block *block;
	struct opcode *op;
	u16 offset = (u16)arg;
	union code c;

	prof_enter(PROF_IO);

	block = lightrec_find_block_from_lut(state->block_cache,
					     arg >> 16, state->curr_pc);
	if (unlikely(!block)) {
		pr_err("rw_generic: No block found in LUT for "PC_FMT" offset 0x%"PRIx16"\n",
			 state->curr_pc, offset);
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_SEGFAULT);
		prof_leave();
		return;
	}

	op = &block->opcode_list[offset];
	c = op->c;

	if (op_flag_movi(op->flags))
		c.i.imm = 0;

	lightrec_rw_helper(state, c, &op->flags, block, offset);
	prof_leave();
}
#endif

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

#if !defined(LIGHTREC_NO_LIGHTNING)
static void lightrec_mfc_cb(struct lightrec_state *state, union code op)
{
	u32 rt = lightrec_mfc(state, op);

	if (op.i.op == OP_SWC2)
		state->temp_reg = rt;
	else if (op.r.rt)
		state->regs.gpr[op.r.rt] = rt;
}
#endif

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

#if !defined(LIGHTREC_NO_LIGHTNING)
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
#endif

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

#if !defined(LIGHTREC_NO_LIGHTNING)
static void lightrec_cp_cb(struct lightrec_state *state, u32 arg)
{
	lightrec_cp(state, (union code) arg);
}
#endif

/* WHAT A STORE INTO RAM ACTUALLY COSTS, split three ways.
 *
 * bleem emits bare stores and invalidates only on DMA into RAM; we carry an
 * invalidation check in every emitted store AND tear blocks down when one
 * lands on compiled code.  The claim that this is a real cost has never been
 * priced, and it splits into three very different bills:
 *
 *   lr_inv_calls   invalidations reaching RAM at all.  The work here is a
 *                  LUT memset plus an unlink sweep -- paid every time.
 *   lr_inv_live    of those, the ones that cleared a slot that was NOT
 *                  already null, i.e. the ones that actually killed a
 *                  compiled entry point.  If this stays near zero the whole
 *                  mechanism is overhead and nothing else.
 *   lr_blk_dead    blocks found outdated in lightrec_get_block and torn
 *                  down.  This is the rebuild storm, and it is the only one
 *                  of the three that costs a recompile.
 *
 * Against lr_blocks_compiled these say how much of the recompiler's share of
 * the profile is first-time compilation and how much is churn. */
unsigned long lr_inv_calls, lr_inv_live, lr_blk_dead;

static struct block * lightrec_get_block(struct lightrec_state *state, u32 pc)
{
	struct block *block = lightrec_find_block(state->block_cache, pc);
	u8 old_flags;

	if (block && lightrec_block_is_outdated(state, block)) {
		pr_debug("Block at "PC_FMT" is outdated!\n", block->pc);

		old_flags = block_set_flags(block, BLOCK_IS_DEAD);
		if (!(old_flags & BLOCK_IS_DEAD)) {
			lr_blk_dead++;
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

/* THE COST OF LEAVING EMITTED CODE, counted the same way in both trees so the
 * two can be diffed at the same guest cycle.
 *
 * Every call here is a block transition the code generator did NOT resolve
 * itself: the dispatcher tore down, C ran the lookup, the dispatcher was
 * rebuilt.  With cross-block linking working, most transitions never reach
 * this function. */
unsigned long lr_gnb_calls, lr_gnb_lut_hit, lr_interp_blocks, lr_blocks_compiled;


void * lightrec_get_next_block_func(struct lightrec_state *state, u32 pc)
{
	struct block *block;
	bool should_recompile;
	void *func;
	int err;
	bool first = true;

	lr_gnb_calls++;

	do {
		func = lut_read(state, lut_offset(pc));
		if (func && func != state->get_next_block) {
			if (first)
				lr_gnb_lut_hit++;
			break;
		}
		first = false;

		/* Decode and optimise. This is the miss path: a hit broke out
		 * of the loop above without reaching here. */
		prof_enter(PROF_COMPILE);
		block = lightrec_get_block(state, pc);
		prof_leave();

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
				prof_enter(PROF_COMPILE);
				err = lightrec_compile_block(state->cstate, block);
				prof_leave();
				if (err == -ENOMEM) {
					/* ONLY out of memory gets out.  A
					 * backend may also refuse a block it
					 * cannot lower, permanently, and it
					 * flags the block so; answering NOMEM
					 * to that flushes the whole cache and
					 * asks again, forever, at the same
					 * PC. */
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
			lr_interp_blocks++;
			pc = lightrec_emulate_block(state, block, pc);

		} else if (!ENABLE_THREADED_COMPILER) {
			/* Block wasn't compiled yet - run the interpreter */
			if (block_has_flag(block, BLOCK_FULLY_TAGGED))
				pr_debug("Block fully tagged, skipping first pass\n");
			else if (ENABLE_FIRST_PASS && likely(!should_recompile))
				pc = lightrec_emulate_block(state, block, pc);

			/* Then compile it using the profiled data */
			prof_enter(PROF_COMPILE);
			err = lightrec_compile_block(state->cstate, block);
			prof_leave();
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

	/* A backend may need its blocks to start on a cache line -- fgl models
	 * the Dreamcast's icache lines and wants 32. */
	if (state->backend_ops->code_align)
		code = tlsf_memalign(state->tlsf, state->backend_ops->code_align,
				     size);
	else
		code = tlsf_malloc(state->tlsf, size);

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_unlock(state);

	return code;
}

#if !defined(LIGHTREC_NO_LIGHTNING)
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
#endif

void lightrec_free_code(struct lightrec_state *state, void *ptr)
{
	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_lock(state);

	tlsf_free(state->tlsf, ptr);

	if (ENABLE_THREADED_COMPILER)
		lightrec_code_alloc_unlock(state);
}

#if !defined(LIGHTREC_NO_LIGHTNING)
static char lightning_code_data[0x80000];

static void * lightrec_emit_code(struct lightrec_state *state,
				 const struct block *block,
				 jit_state_t *_jit, unsigned int *size)
{
	bool has_code_buffer = ENABLE_CODE_BUFFER && state->tlsf;
	jit_word_t code_size, new_code_size;
	void *code;

	jit_realize();

	if (ENABLE_DISASSEMBLER)
		jit_set_data(lightning_code_data, sizeof(lightning_code_data), 0);
	else
		jit_set_data(NULL, 0, JIT_DISABLE_DATA | JIT_DISABLE_NOTE);

	if (has_code_buffer) {
		jit_get_code(&code_size);

#ifdef __i386__
		/* Lightning's code size estimation routine is buggy on x86 and
		 * will return a value that's too small. */
		code_size *= 2;
#endif

		code = lightrec_alloc_code(state, (size_t) code_size);

		if (!code) {
			if (ENABLE_THREADED_COMPILER) {
				/* If we're using the threaded compiler, return
				 * an allocation error here. The threaded
				 * compiler will then empty its job queue and
				 * request a code flush using the reaper. */
				return NULL;
			}

			/* Remove outdated blocks, and try again */
			lightrec_remove_outdated_blocks(state->block_cache, block);

			pr_debug("Re-try to alloc %zu bytes...\n", code_size);

			code = lightrec_alloc_code(state, code_size);
			if (!code) {
				pr_err("Could not alloc even after removing old blocks!\n");
				return NULL;
			}
		}

		jit_set_code(code, code_size);
	}

	code = jit_emit();
	if (!code) {
		if (has_code_buffer)
			lightrec_free_code(state, code);

		return NULL;
	}

	jit_get_code(&new_code_size);
	lightrec_register(MEM_FOR_CODE, new_code_size);

	if (has_code_buffer) {
		lightrec_realloc_code(state, code, (size_t) new_code_size);

		pr_debug("Creating code block at address 0x%" PRIxPTR ", "
			 "code size: %" PRIuPTR " new: %" PRIuPTR "\n",
			 (uintptr_t) code, code_size, new_code_size);
	}

	*size = (unsigned int) new_code_size;

	if (state->ops.code_inv)
		state->ops.code_inv(code, new_code_size);

	return code;
}
#endif /* !LIGHTREC_NO_LIGHTNING */

u32 lightrec_memset(struct lightrec_state *state)
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

u32 lightrec_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg)
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

#if !defined(LIGHTREC_NO_LIGHTNING)
static void update_cycle_counter_before_c(jit_state_t *_jit)
{
	/* update state->current_cycle */
	jit_ldxi_i(JIT_R2, LIGHTREC_REG_STATE, lightrec_offset(target_cycle));
	jit_subr(JIT_R1, JIT_R2, LIGHTREC_REG_CYCLE);
	jit_stxi_i(lightrec_offset(current_cycle), LIGHTREC_REG_STATE, JIT_R1);
}

static void update_cycle_counter_after_c(jit_state_t *_jit)
{
	/* Recalc the delta */
	jit_ldxi_i(JIT_R1, LIGHTREC_REG_STATE, lightrec_offset(current_cycle));
	jit_ldxi_i(JIT_R2, LIGHTREC_REG_STATE, lightrec_offset(target_cycle));
	jit_subr(LIGHTREC_REG_CYCLE, JIT_R2, JIT_R1);
}

static void sync_next_pc(jit_state_t *_jit)
{
	if (lightrec_store_next_pc()) {
		jit_ldxi_ui(JIT_V0, LIGHTREC_REG_STATE,
			    lightrec_offset(next_pc));
	}
}

static struct block * generate_dispatcher(struct lightrec_state *state)
{
	struct block *block;
	jit_state_t *_jit;
	jit_node_t *to_end, *to_loop, *to_slow_path, *loop, *loop2,
		   *addr, *addr2, *addr3, *addr4, *addr5, *addr6,
		   *c_wrapper;
	unsigned int i;
	int sp;

	block = lightrec_malloc(state, MEM_FOR_IR, sizeof(*block));
	if (!block)
		goto err_no_mem;

	_jit = jit_new_state();
	if (!_jit)
		goto err_free_block;

	jit_name("dispatcher");
	jit_note(__FILE__, __LINE__);

	jit_prolog();
	jit_frame(256);

	jit_getarg(LIGHTREC_REG_STATE, jit_arg());
	jit_getarg(JIT_V0, jit_arg());
	jit_getarg(JIT_V1, jit_arg());
	jit_getarg_i(LIGHTREC_REG_CYCLE, jit_arg());

	/* Force all callee-saved registers to be pushed on the stack */
	for (i = 0; i < NUM_REGS; i++)
		jit_movr(JIT_V(i + FIRST_REG), JIT_V(i + FIRST_REG));

	to_loop = jit_jmpi();

	/* The block will jump here, with the number of cycles remaining in
	 * LIGHTREC_REG_CYCLE */
	addr2 = jit_indirect();

	sync_next_pc(_jit);

	loop2 = jit_label();

	/* Convert next PC to KUNSEG and avoid mirrors */
	jit_andi(JIT_V1, JIT_V0, RAM_SIZE - 1);
	jit_andi(JIT_R2, JIT_V0, BIOS_SIZE - 1);
	jit_andi(JIT_R1, JIT_V0, BIT(28));
	jit_addi(JIT_R2, JIT_R2, RAM_SIZE);
	jit_movnr(JIT_V1, JIT_R2, JIT_R1);

	/* If possible, use the code LUT */
	if (!lut_is_32bit(state))
		jit_lshi(JIT_V1, JIT_V1, 1);
	jit_add_state(JIT_V1, JIT_V1);
	jit_addi(JIT_V1, JIT_V1, lightrec_offset(code_lut));

	/* The block will jump here if it already knows the code LUT entry */
	addr6 = jit_indirect();

	if (lut_is_32bit(state))
		jit_ldr_ui(JIT_V1, JIT_V1);
	else
		jit_ldr(JIT_V1, JIT_V1);

	/* Jump to end if state->target_cycle < state->current_cycle */
	to_end = jit_blei(LIGHTREC_REG_CYCLE, 0);

	/* Store back the current PC to the lightrec_state structure */
	jit_stxi_i(lightrec_offset(curr_pc), LIGHTREC_REG_STATE, JIT_V0);

	/* If we get NULL, jump to the slow path */
	to_slow_path = jit_beqi(JIT_V1, 0);

	jit_patch(to_loop);
	loop = jit_label();

	if (!arch_has_fast_mask())
		jit_movi(JIT_R1, 0x1fffffff);

	/* Call the block's code */
	jit_jmpr(JIT_V1);

	jit_patch(to_slow_path);

	/* The code LUT will be set to this address when the block at the target
	 * PC has been preprocessed but not yet compiled by the threaded
	 * recompiler */
	addr = jit_indirect();

	/* Slow path: call C function lightrec_get_next_block_func() */

	if (ENABLE_FIRST_PASS || OPT_DETECT_IMPOSSIBLE_BRANCHES) {
		/* We may call the interpreter - update state->current_cycle */
		update_cycle_counter_before_c(_jit);
	}

	jit_prepare();
	jit_pushargr(LIGHTREC_REG_STATE);
	jit_pushargr(JIT_V0);

	/* Save the cycles register if needed */
	if (!(ENABLE_FIRST_PASS || OPT_DETECT_IMPOSSIBLE_BRANCHES))
		jit_movr(JIT_V0, LIGHTREC_REG_CYCLE);

	/* Get the next block */
	jit_finishi(&lightrec_get_next_block_func);
	jit_retval(JIT_V1);

	if (ENABLE_FIRST_PASS || OPT_DETECT_IMPOSSIBLE_BRANCHES) {
		/* The interpreter may have updated state->current_cycle and
		 * state->target_cycle - recalc the delta */
		update_cycle_counter_after_c(_jit);
	} else {
		jit_movr(LIGHTREC_REG_CYCLE, JIT_V0);
	}

	/* Reset JIT_V0 to the next PC */
	jit_ldxi_ui(JIT_V0, LIGHTREC_REG_STATE, lightrec_offset(curr_pc));

	/* If we get non-NULL, loop */
	jit_patch_at(jit_bnei(JIT_V1, 0), loop);

	/* When exiting, the recompiled code will jump to that address */
	jit_note(__FILE__, __LINE__);
	jit_patch(to_end);

	/* Store back the current PC to the lightrec_state structure */
	jit_stxi_i(lightrec_offset(curr_pc), LIGHTREC_REG_STATE, JIT_V0);

	jit_retr(LIGHTREC_REG_CYCLE);

	if (OPT_REPLACE_MEMSET) {
		/* Blocks will jump here when they need to call
		 * lightrec_memset() */
		addr3 = jit_indirect();

		jit_movr(JIT_V1, LIGHTREC_REG_CYCLE);

		jit_prepare();
		jit_pushargr(LIGHTREC_REG_STATE);

		jit_finishi(lightrec_memset);
		jit_retval(LIGHTREC_REG_CYCLE);

		jit_ldxi_ui(JIT_V0, LIGHTREC_REG_STATE, lightrec_offset(regs.gpr[31]));

		jit_subr(LIGHTREC_REG_CYCLE, JIT_V1, LIGHTREC_REG_CYCLE);

		jit_patch_at(jit_b(), loop2);
	}

	if (OPT_DETECT_IMPOSSIBLE_BRANCHES) {
		/* Blocks will jump here when they reach a branch that should
		 * be executed with the interpreter, passing the branch's PC
		 * in JIT_V0 and the address of the block in JIT_V1. */
		addr4 = jit_indirect();

		sync_next_pc(_jit);
		update_cycle_counter_before_c(_jit);

		jit_prepare();
		jit_pushargr(LIGHTREC_REG_STATE);
		jit_pushargr(JIT_V1);
		jit_pushargr(JIT_V0);
		jit_finishi(lightrec_emulate_block);

		jit_retval(JIT_V0);

		update_cycle_counter_after_c(_jit);

		jit_patch_at(jit_b(), loop2);

	}

	if (OPT_HANDLE_LOAD_DELAYS) {
		/* Blocks will jump here when they reach a branch with a load
		 * opcode in its delay slot. The delay slot has already been
		 * executed; the load value is in (state->temp_reg), and the
		 * register number is in JIT_V1.
		 * Jump to a C function which will evaluate the branch target's
		 * first opcode, to make sure that it does not read the register
		 * in question; and if it does, handle it accordingly. */
		addr5 = jit_indirect();

		sync_next_pc(_jit);
		update_cycle_counter_before_c(_jit);

		jit_prepare();
		jit_pushargr(LIGHTREC_REG_STATE);
		jit_pushargr(JIT_V0);
		jit_pushargr(JIT_V1);
		jit_finishi(lightrec_check_load_delay);

		jit_retval(JIT_V0);

		update_cycle_counter_after_c(_jit);

		jit_patch_at(jit_b(), loop2);
	}

	jit_epilog();

	/* Wrapper entry point */
	c_wrapper = jit_indirect();
	jit_prolog();

	sp = jit_allocai(NUM_TEMPS * sizeof(void *));

	/* Save all temporaries on stack */
	for (i = 0; i < NUM_TEMPS; i++)
		jit_stxi(sp + i * sizeof(void *), JIT_FP, JIT_R(i + FIRST_TEMP));

	jit_getarg(JIT_R1, jit_arg());
	jit_getarg(JIT_R2, jit_arg());

	jit_prepare();
	jit_pushargr(LIGHTREC_REG_STATE);
	jit_pushargr(JIT_R2);

	jit_ldxi_ui(JIT_R2, LIGHTREC_REG_STATE, lightrec_offset(target_cycle));

	/* state->current_cycle = state->target_cycle - delta; */
	jit_subr(LIGHTREC_REG_CYCLE, JIT_R2, LIGHTREC_REG_CYCLE);
	jit_stxi_i(lightrec_offset(current_cycle), LIGHTREC_REG_STATE, LIGHTREC_REG_CYCLE);

	/* Call the wrapper function */
	jit_finishr(JIT_R1);

	/* delta = state->target_cycle - state->current_cycle */;
	jit_ldxi_ui(LIGHTREC_REG_CYCLE, LIGHTREC_REG_STATE, lightrec_offset(current_cycle));
	jit_ldxi_ui(JIT_R1, LIGHTREC_REG_STATE, lightrec_offset(target_cycle));
	jit_subr(LIGHTREC_REG_CYCLE, JIT_R1, LIGHTREC_REG_CYCLE);

	/* Restore temporaries from stack */
	for (i = 0; i < NUM_TEMPS; i++)
		jit_ldxi(JIT_R(i + FIRST_TEMP), JIT_FP, sp + i * sizeof(void *));

	jit_ret();
	jit_epilog();

	block->_jit = _jit;
	block->opcode_list = NULL;
	block->flags = BLOCK_NO_OPCODE_LIST;
	block->nb_ops = 0;

	block->function = lightrec_emit_code(state, block, _jit,
					     &block->code_size);
	if (!block->function)
		goto err_free_jit;

	state->c_wrapper = jit_address(c_wrapper);

	state->eob_wrapper_func = jit_address(addr2);
	if (OPT_DETECT_IMPOSSIBLE_BRANCHES)
		state->interpreter_func = jit_address(addr4);
	if (OPT_HANDLE_LOAD_DELAYS)
		state->ds_check_func = jit_address(addr5);
	if (OPT_REPLACE_MEMSET)
		state->memset_func = jit_address(addr3);
	state->fast_eob = jit_address(addr6);
	state->get_next_block = jit_address(addr);

#ifdef LIGHTREC_SH4_INTERP
	/* Name the emitted stubs so the SH-4 PC histogram can say whether a
	 * hot address is guest code or lightrec's own plumbing. */
	sh4_glue_note_stub("get_next_block", state->get_next_block);
	sh4_glue_note_stub("eob_wrapper", state->eob_wrapper_func);
	sh4_glue_note_stub("interpreter", state->interpreter_func);
	sh4_glue_note_stub("ds_check", state->ds_check_func);
	sh4_glue_note_stub("memset", state->memset_func);
	sh4_glue_note_stub("dispatcher", block->function);
#endif

	if (ENABLE_DISASSEMBLER) {
		pr_debug("Dispatcher block:\n");
		jit_disassemble();
	}

	/* We're done! */
	jit_clear_state();
	return block;

err_free_jit:
	jit_destroy_state();
err_free_block:
	lightrec_free(state, MEM_FOR_IR, sizeof(*block), block);
err_no_mem:
	pr_err("Unable to compile dispatcher: Out of memory\n");
	return NULL;
}
#endif /* !LIGHTREC_NO_LIGHTNING */

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
#if !defined(LIGHTREC_NO_LIGHTNING)
	block->_jit = NULL;
#endif /* !LIGHTREC_NO_LIGHTNING */
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

#if !defined(LIGHTREC_NO_LIGHTNING)
static void lightrec_reap_jit(struct lightrec_state *state, void *data)
{
	_jit_destroy_state(data);
}
#endif /* !LIGHTREC_NO_LIGHTNING */

static void lightrec_free_function(struct lightrec_state *state, void *fn)
{
	if (ENABLE_CODE_BUFFER && state->tlsf) {
		pr_debug("Freeing code block at 0x%" PRIxPTR "\n", (uintptr_t) fn);
		lightrec_free_code(state, fn);
	}
}

static void lightrec_reap_function(struct lightrec_state *state, void *data)
{
	/* The code is about to go and the reaper no longer knows which block
	 * it belonged to, so there is nothing left to unlink selectively. */
	if (state->backend_ops->unlink_all)
		state->backend_ops->unlink_all(state, LIGHTREC_UNLINK_FREE);

	lightrec_free_function(state, data);
}

static void lightrec_reap_opcode_list(struct lightrec_state *state, void *data)
{
	lightrec_free_opcode_list(state, data);
}

#if !defined(LIGHTREC_NO_LIGHTNING)
/*
 * The Lightning backend's half of block compilation: opcode list in, host code
 * out.  Everything the core does around it -- dead-block detection, the LUT,
 * the reaper, the fully-tagged opcode-list free -- knows nothing about a code
 * generator and stayed behind in lightrec_compile_block().
 *
 * On success block->function and block->code_size are set and every
 * cstate->targets[i].host holds the address the core writes into the LUT.
 * The old Lightning context is reaped here rather than by the caller: it is
 * this backend's private allocation and the caller cannot type it.
 */
static int lightning_compile(struct lightrec_cstate *cstate,
			     struct block *block)
{
	struct lightrec_state *state = cstate->state;
	jit_state_t *_jit, *oldjit;
	jit_node_t *start_of_block;
	bool skip_next = false;
	struct opcode *elm;
	unsigned int i, j;
	void *new_fn;

	_jit = jit_new_state();
	if (!_jit)
		return -ENOMEM;

	oldjit = block->_jit;
	block->_jit = _jit;

	lightrec_regcache_reset(cstate->reg_cache);

	if (OPT_PRELOAD_PC && (block->flags & BLOCK_PRELOAD_PC))
		lightrec_preload_pc(cstate->reg_cache, _jit);

	if (!arch_has_fast_mask())
		lightrec_preload_imm(cstate->reg_cache, _jit, JIT_R1, 0x1fffffff);

	cstate->cycles = 0;
	cstate->nb_local_branches = 0;
	cstate->nb_targets = 0;
	cstate->no_load_delay = false;

	jit_prolog();
	jit_tramp(256);

	start_of_block = jit_label();

	for (i = 0; i < block->nb_ops; i++) {
		elm = &block->opcode_list[i];

		if (skip_next) {
			skip_next = false;
			continue;
		}

		if (should_emulate(elm)) {
			pr_debug("Branch at offset 0x%x will be emulated\n",
				 i << 2);

			lightrec_emit_jump_to_interpreter(cstate, block, i);
			skip_next = !op_flag_no_ds(elm->flags);
		} else {
			lightrec_rec_opcode(cstate, block, i);
			skip_next = !op_flag_no_ds(elm->flags) && has_delay_slot(elm->c);
#if _WIN32
			/* FIXME: GNU Lightning on Windows seems to use our
			 * mapped registers as temporaries. Until the actual bug
			 * is found and fixed, unconditionally mark our
			 * registers as live here. */
			lightrec_regcache_mark_live(cstate->reg_cache, _jit);
#endif
		}

		cstate->cycles += lightrec_cycles_of_opcode(state, elm->c);
	}

	for (i = 0; i < cstate->nb_local_branches; i++) {
		struct lightrec_branch *branch = &cstate->local_branches[i];

		pr_debug("Patch local branch to offset 0x%"PRIx32"\n",
			 branch->target << 2);

		if (branch->target == 0) {
			jit_patch_at(branch->branch, start_of_block);
			continue;
		}

		for (j = 0; j < cstate->nb_targets; j++) {
			if (cstate->targets[j].offset == branch->target) {
				jit_patch_at(branch->branch,
					     cstate->targets[j].label);
				break;
			}
		}

		if (j == cstate->nb_targets)
			pr_err("Unable to find branch target\n");
	}

	jit_ret();
	jit_epilog();

	new_fn = lightrec_emit_code(state, block, _jit, &block->code_size);
	if (!new_fn) {
		if (!ENABLE_THREADED_COMPILER)
			pr_err("Unable to compile block!\n");
		block->_jit = oldjit;
		jit_clear_state();
		_jit_destroy_state(_jit);
		return -ENOMEM;
	}

	block->function = new_fn;

	/* Resolve every target to a host address while the Lightning context
	 * is still alive.  After jit_clear_state() the labels are gone, and
	 * the core reads these long after that. */
	for (i = 0; i < cstate->nb_targets; i++)
		cstate->targets[i].host = jit_address(cstate->targets[i].label);

	if (ENABLE_DISASSEMBLER) {
		pr_debug("Compiling block at "PC_FMT"\n", block->pc);
		jit_disassemble();
	}

	jit_clear_state();

	if (oldjit) {
		pr_debug("Block "X32_FMT" recompiled, reaping old jit context.\n",
			 block->pc);

		if (ENABLE_THREADED_COMPILER)
			lightrec_reaper_add(state->reaper,
					    lightrec_reap_jit, oldjit);
		else
			_jit_destroy_state(oldjit);
	}

	return 0;
}
#endif /* !LIGHTREC_NO_LIGHTNING */

int lightrec_compile_block(struct lightrec_cstate *cstate,
			   struct block *block)
{
	struct block *dead_blocks[ARRAY_SIZE(cstate->targets)];
	u32 was_dead[ARRAY_SIZE(cstate->targets) / 8];
	struct lightrec_state *state = cstate->state;
	struct lightrec_branch_target *target;
	bool fully_tagged = false;
	struct block *block2;
	void *old_fn;
	size_t old_code_size;
	unsigned int i;
	u8 old_flags;
	u32 offset;
	int ret;

	fully_tagged = lightrec_block_is_fully_tagged(block);
	if (fully_tagged)
		block_set_flags(block, BLOCK_FULLY_TAGGED);

	if (OPT_DETECT_IDLE && !block_has_flag(block, BLOCK_NO_OPCODE_LIST))
		lightrec_detect_idle(block);

	old_fn = block->function;
	old_code_size = block->code_size;

	ret = state->backend_ops->compile(cstate, block);
	if (ret)
		return ret;

	lr_blocks_compiled++;

	/* Pause the reaper, because lightrec_reset_lut_offset() may try to set
	 * the old block->function pointer to the code LUT. */
	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_pause(state->reaper);

	block_clear_flags(block, BLOCK_SHOULD_RECOMPILE);

	/*
	 * GTE_DUMP=<file>: write the emitted code of the first block that
	 * holds a COP2 command, so the call sequence rec_CP2_gte wraps around
	 * the GTE can be counted.  Under SH4=1 that is SH-4; read it back with
	 *
	 *     sh-elf-objdump -b binary -m sh4 -D <file>
	 */
	if (block->opcode_list) {
		static int gte_dumped;
		const char *gd = getenv("GTE_DUMP");

		for (i = 0; gd && gte_dumped < 16 && i < block->nb_ops; i++) {
			union code c = block->opcode_list[i].c;
			char path[512];
			FILE *f;

			if (c.i.op != OP_CP2 || !(c.opcode & BIT(25)))
				continue;

			snprintf(path, sizeof(path), "%s.%02d", gd,
				 gte_dumped);
			f = fopen(path, "wb");
			if (f) {
				fwrite(block->function, 1, block->code_size, f);
				fclose(f);
				printf("GTE_DUMP: %s pc=0x%08x ops=%u code=%u"
				       " cop2=0x%08x at op %u\n", path,
				       block->pc, block->nb_ops,
				       block->code_size, c.opcode, i);
			}
			gte_dumped++;
		}
	}

	/* Add compiled function to the LUT */
	lut_write(state, lut_offset(block->pc), block->function);

	/* Detect old blocks that have been covered by the new one */
	for (i = 0; ENABLE_THREADED_COMPILER && i < cstate->nb_targets; i++) {
		target = &cstate->targets[i];

		if (!target->offset)
			continue;

		offset = block->pc + target->offset * sizeof(u32);

		block2 = lightrec_find_block(state->block_cache, offset);
		if (block2) {
			/* No need to check if block2 is compilable - it must
			 * be, otherwise block wouldn't be compilable either */

			/* Set the "block dead" flag to prevent the dynarec from
			 * recompiling this block */
			old_flags = block_set_flags(block2, BLOCK_IS_DEAD);

			if (old_flags & BLOCK_IS_DEAD)
				was_dead[i / 32] |= BIT(i % 32);
			else
				was_dead[i / 32] &= ~BIT(i % 32);
		}

		dead_blocks[i] = block2;

		/* If block2 was pending for compilation, cancel it.
		 * If it's being compiled right now, wait until it finishes. */
		if (block2)
			lightrec_recompiler_remove(state->rec, block2);
	}

	for (i = 0; i < cstate->nb_targets; i++) {
		target = &cstate->targets[i];

		if (!target->offset)
			continue;

		/* We know from now on that block2 (if present) isn't going to
		 * be compiled. We can override the LUT entry with our new
		 * block's entry point. */
		offset = lut_offset(block->pc) + target->offset;
		lut_write(state, offset, target->host);

		if (ENABLE_THREADED_COMPILER) {
			block2 = dead_blocks[i];
		} else {
			offset = block->pc + target->offset * sizeof(u32);
			block2 = lightrec_find_block(state->block_cache, offset);
		}
		if (block2) {
			pr_debug("Reap block "X32_FMT" as it's covered by block "
				 X32_FMT"\n", block2->pc, block->pc);

			/* Finally, reap the block. */
			if (!ENABLE_THREADED_COMPILER) {
				lightrec_unregister_block(state->block_cache, block2);
				lightrec_free_block(state, block2);
			} else if (!(was_dead[i / 32] & BIT(i % 32))) {
				lightrec_reaper_add(state->reaper,
						    lightrec_reap_block,
						    block2);
			}
		}
	}

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
		if (ENABLE_THREADED_COMPILER) {
			lightrec_reaper_add(state->reaper,
					    lightrec_reap_function, old_fn);
		} else {
			if (state->backend_ops->unlink_block)
				state->backend_ops->unlink_block(state,
						lut_offset(block->pc),
						block->nb_ops, old_fn,
						old_code_size,
						LIGHTREC_UNLINK_FREE);
			lightrec_free_function(state, old_fn);
		}

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

#if !defined(LIGHTREC_NO_LIGHTNING)
static u32 lightning_execute(struct lightrec_state *state, u32 pc,
			     u32 target_cycle)
{
	s32 (*func)(struct lightrec_state *, u32, void *, s32) = (void *)state->dispatcher->function;
	void *block_trace;
	s32 cycles_delta;

	/* Handle the cycle counter overflowing */
	if (unlikely(target_cycle < state->current_cycle))
		target_cycle = UINT_MAX;

	state->target_cycle = target_cycle;
	state->curr_pc = pc;

	block_trace = lightrec_get_next_block_func(state, pc);
	if (block_trace) {
		cycles_delta = state->target_cycle - state->current_cycle;

#ifdef LIGHTREC_SH4_INTERP
		/* The one place lightrec enters emitted code.  With the SH-4
		 * backend the emitted code cannot run on this host, so it is
		 * interpreted instead. */
		cycles_delta = sh4_glue_dispatch((void *)func, state,
						 state->curr_pc, block_trace,
						 cycles_delta);
#else
		cycles_delta = (*func)(state, state->curr_pc,
				       block_trace, cycles_delta);
#endif

		state->current_cycle = state->target_cycle - cycles_delta;
	}

	return state->curr_pc;
}
#endif

/*
 * The portable entry point.  Everything here is true of any backend: clear the
 * exit flags, run, then let the reaper catch up and report.  How the guest
 * actually runs is the one line in the middle.
 */
u32 lightrec_execute(struct lightrec_state *state, u32 pc, u32 target_cycle)
{
	state->exit_flags = LIGHTREC_EXIT_NORMAL;

	pc = state->backend_ops->execute(state, pc, target_cycle);

	if (ENABLE_THREADED_COMPILER)
		lightrec_reaper_reap(state->reaper);

	if (LOG_LEVEL >= INFO_L)
		lightrec_print_info(state);

	return pc;
}

/* THIS TREE'S LOCKSTEP, NOT A BACKEND'S.  The fgl build has its own
 * (libpcsxcore/fgl/fgl_trace.c) under the same public name, because the
 * two verify different machines; only one of them is ever linked. */
#if !defined(LIGHTREC_NO_LIGHTNING)
/* ------------------------------------------------------------ lockstep
 *
 * Ported from rearmed's FGL_LOCKSTEP.  Every block is run twice: once down
 * the compiled path (which under SH4=1 is the SH-4 interpreter), then rewound
 * and run again in lightrec's own interpreter, and the two are diffed.
 *
 * PCSX_LOCKSTEP=1 compares the registers and the exit PC, which is cheap
 * enough to leave on.  =2 adds all of guest RAM and the scratchpad, which is
 * three 2 MiB copies per storing block, and is for when the registers stay
 * clean and the machine diverges anyway.
 *
 * The whole of RAM, not a window: bloom's original compared a fixed 4 KiB and
 * reported nothing while the machine diverged in the first frame, because the
 * wrong store was outside the window.  A verifier that can miss the bug it is
 * looking for is worse than none, because its silence gets believed.
 *
 * Note what a divergence means here.  This tree exists to reproduce bloom, and
 * where bloom is wrong it should be wrong the same way -- so a disagreement
 * with the interpreter is the thing you were looking for, not automatically a
 * bug to fix.
 */
u32 lightrec_lockstep_on;

/* optimizer.c has this test and keeps it static; it is six cases and copying
 * them beats exporting one for a debug mode. */
static bool ls_block_stores(const struct block *block)
{
	u16 k;

	if (block_has_flag((struct block *)block, BLOCK_NO_OPCODE_LIST) ||
	    !block->opcode_list)
		return true;   /* unknown: compare, rather than miss it */

	for (k = 0; k < block->nb_ops; k++) {
		switch (block->opcode_list[k].c.i.op) {
		case OP_SB:
		case OP_SH:
		case OP_SW:
		case OP_SWL:
		case OP_SWR:
		case OP_SWC2:
			return true;
		default:
			break;
		}
	}

	return false;
}

/* A DEVICE READ CANNOT BE DONE TWICE.
 *
 * A status register, a timer, a FIFO: the second read returns something the
 * first one did not, so the two runs disagree over the machine rather than
 * over the compiler.  The optimiser has already tagged every access with the
 * region it proved, so the blocks to leave alone are exactly the ones carrying
 * an access it could not prove to be memory. */
static bool ls_block_touches_device(const struct block *block)
{
	u16 k;

	if (block_has_flag((struct block *)block, BLOCK_NO_OPCODE_LIST) ||
	    !block->opcode_list)
		return true;

	for (k = 0; k < block->nb_ops; k++) {
		const struct opcode *op = &block->opcode_list[k];
		u8 io;

		if (!opcode_is_io(op->c))
			continue;

		io = LIGHTREC_FLAGS_GET_IO_MODE(op->flags);
		if (io == LIGHTREC_IO_HW || io == LIGHTREC_IO_DIRECT_HW ||
		    io == LIGHTREC_IO_UNKNOWN)
			return true;
	}

	return false;
}

static void ls_report(const struct block *block)
{
	u16 k;

	fprintf(stderr, "LS block %08x %u ops\n", block->pc, block->nb_ops);

	if (block_has_flag((struct block *)block, BLOCK_NO_OPCODE_LIST) ||
	    !block->opcode_list)
		return;

	for (k = 0; k < block->nb_ops; k++)
		fprintf(stderr, "LS  op %2u  %08x flags=%08x\n", k,
			block->opcode_list[k].c.opcode,
			block->opcode_list[k].flags);
}

static u32 lightrec_lockstep_one(struct lightrec_state *state, u32 pc)
{
	static struct lightrec_registers before, after;
	static u8 *ram_pre, *ram_post;
	static u8 sp_pre[1024], sp_post[1024];
	static int reported;
	const struct lightrec_mem_map *ram =
		&state->maps[PSX_MAP_KERNEL_USER_RAM];
	const struct lightrec_mem_map *scratch =
		&state->maps[PSX_MAP_SCRATCH_PAD];
	struct block *block;
	u32 pc_jit, pc_int, cyc0, cyc_jit;
	bool stores, cmp_mem;
	unsigned i;

	/* One report is the whole story; after a divergence the two machines
	 * are different machines and every later block disagrees. */
	if (reported)
		return lightrec_execute(state, pc, state->current_cycle);

	if (!ram_pre) {
		ram_pre = malloc(ram->length);
		ram_post = malloc(ram->length);
		if (!ram_pre || !ram_post)
			return lightrec_execute(state, pc, state->current_cycle);
	}

	block = lightrec_find_block(state->block_cache, pc);

	/* A STORING BLOCK CANNOT BE REPLAYED OVER ITS OWN FOOTPRINT.
	 *
	 * The rewind puts the registers and the cycle count back, but if
	 * memory is left as the compiled run left it, a block that stores to
	 * something it also loads reads its own output the second time round
	 * and takes a different branch.  That is not a divergence, it is the
	 * harness marking its own homework -- it cost a calibration pass to
	 * find, on a BIOS block whose `sw $v0,4($a0)` fed its own
	 * `lw $a1,4($a0)`.
	 *
	 * So memory is saved and restored whenever the block stores at all.
	 * The level only decides whether RAM is also *compared*: level 1 is
	 * registers and exit PC, and pays nothing for blocks that cannot
	 * store; level 2 pays three 2 MiB copies on the ones that can. */
	stores = block && ls_block_stores(block);
	cmp_mem = lightrec_lockstep_on > 1 && stores;

	/* Level 1 does not copy RAM, so it cannot replay a storing block
	 * honestly.  Skip those rather than report a fiction. */
	if (stores && !cmp_mem)
		return lightrec_execute(state, pc, state->current_cycle);

	cyc0 = state->current_cycle;
	before = state->regs;
	if (cmp_mem) {
		memcpy(ram_pre, ram->address, ram->length);
		memcpy(sp_pre, scratch->address, 1024);
	}

	/* A target equal to the current cycle is a budget of zero, which the
	 * dispatcher runs exactly one block on. */
	pc_jit = lightrec_execute(state, pc, cyc0);

	/* A block can also stop inside itself: a backward or exiting edge
	 * tests the budget, and the budget here is zero.  Run on from where it
	 * stopped until it leaves, so both sides are compared at the exit. */
	while (block && pc_jit - block->pc < 4u * block->nb_ops &&
	       !state->exit_flags)
		pc_jit = lightrec_execute(state, pc_jit, state->current_cycle);

	after = state->regs;
	cyc_jit = state->current_cycle;

	if (block && !state->exit_flags && !ls_block_touches_device(block)) {
		if (cmp_mem) {
			memcpy(ram_post, ram->address, ram->length);
			memcpy(ram->address, ram_pre, ram->length);
			memcpy(sp_post, scratch->address, 1024);
			memcpy(scratch->address, sp_pre, 1024);
		}
		state->regs = before;
		state->current_cycle = cyc0;

		pc_int = lightrec_emulate_block(state, block, pc);

		/* The interpreter stops at every SYNC-flagged op to settle its
		 * cycle count and hands back a pc still inside the block.  Not
		 * a divergence: run on until it leaves or lands where the JIT
		 * did. */
		while (pc_int != pc_jit &&
		       pc_int - block->pc < 4u * block->nb_ops &&
		       !state->exit_flags)
			pc_int = lightrec_emulate_block(state, block, pc_int);

		if (pc_int != pc_jit) {
			reported = 1;
			fprintf(stderr, "\nLS %08x: exit jit=%08x int=%08x\n",
				pc, pc_jit, pc_int);
		}
		for (i = 0; i < 34 && !reported; i++) {
			if (state->regs.gpr[i] == after.gpr[i])
				continue;
			reported = 1;
			fprintf(stderr, "\nLS %08x: $%u jit=%08x int=%08x\n",
				pc, i, after.gpr[i], state->regs.gpr[i]);
		}
		if (cmp_mem && !reported) {
			const u32 *f = (const u32 *)ram_post;
			const u32 *t = (const u32 *)ram->address;

			for (i = 0; i < ram->length / 4; i++) {
				if (f[i] == t[i])
					continue;
				reported = 1;
				fprintf(stderr, "\nLS %08x: ram %08x "
					"jit=%08x int=%08x\n", pc, i * 4,
					f[i], t[i]);
				break;
			}
		}
		if (cmp_mem && !reported) {
			const u32 *f = (const u32 *)sp_post;
			const u32 *t = (const u32 *)scratch->address;

			for (i = 0; i < 256; i++) {
				if (f[i] == t[i])
					continue;
				reported = 1;
				fprintf(stderr, "\nLS %08x: scratch %08x "
					"jit=%08x int=%08x\n", pc, i * 4,
					f[i], t[i]);
				break;
			}
		}
		if (cmp_mem) {
			memcpy(ram->address, ram_post, ram->length);
			memcpy(scratch->address, sp_post, 1024);
		}
		if (reported) {
			/* Which suspect: a real mismatch, or the harness
			 * letting the zero-budget run cross into the next
			 * block while the interpreter replays only this one?
			 * If pc_jit is outside [block->pc, +4*nb_ops) the two
			 * sides did not run the same amount of machine. */
			fprintf(stderr, "LS  jit exit %08x %s block "
				"[%08x,%08x)  cycles jit=%u int=%u\n",
				pc_jit,
				pc_jit - block->pc < 4u * block->nb_ops ?
					"inside" : "OUTSIDE",
				block->pc, block->pc + 4u * block->nb_ops,
				cyc_jit - cyc0, state->current_cycle - cyc0);
			ls_report(block);
		}
	}

	/* The compiled path is the one that counts: put its results back. */
	state->regs = after;
	state->current_cycle = cyc_jit;
	return pc_jit;
}

u32 lightrec_lockstep(struct lightrec_state *state, u32 pc, u32 target_cycle)
{
	state->exit_flags = LIGHTREC_EXIT_NORMAL;

	if (unlikely(target_cycle < state->current_cycle))
		target_cycle = UINT_MAX;

	do {
		pc = lightrec_lockstep_one(state, pc);
	} while (!state->exit_flags && state->current_cycle < target_cycle);

	return pc;
}
#endif /* !LIGHTREC_NO_LIGHTNING */

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
	if (state->backend_ops->free_block)
		state->backend_ops->free_block(state, block);
	if (block->function) {
		if (state->backend_ops->unlink_block)
			state->backend_ops->unlink_block(state,
					lut_offset(block->pc), block->nb_ops,
					block->function, block->code_size,
					LIGHTREC_UNLINK_FREE);
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

#if !defined(LIGHTREC_NO_LIGHTNING)
	cstate->reg_cache = lightrec_regcache_init(state);
	if (!cstate->reg_cache) {
		lightrec_free(state, MEM_FOR_LIGHTREC, sizeof(*cstate), cstate);
		return NULL;
	}
#endif

	cstate->state = state;

	return cstate;
}

void lightrec_free_cstate(struct lightrec_cstate *cstate)
{
#if !defined(LIGHTREC_NO_LIGHTNING)
	lightrec_free_regcache(cstate->reg_cache);
#endif
	lightrec_free(cstate->state, MEM_FOR_LIGHTREC, sizeof(*cstate), cstate);
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
	const struct lightrec_backend *state_backend;

	/* Whatever lightrec_select_backend() was last told, or the default.
	 * Resolved here rather than taken as a parameter so that every
	 * existing caller of lightrec_init() keeps compiling unchanged. */
	state_backend = lightrec_select_backend(NULL);

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

	if (state_backend->global_init && state_backend->global_init(argv0)) {
		pr_err("Unable to initialise the %s backend\n",
		       state_backend->name);
		if (ENABLE_CODE_BUFFER && tlsf)
			tlsf_destroy(tlsf);
		return NULL;
	}

	/* The operand cache is direct mapped, so the low bits of an address
	 * pick the set. The scratchpad is touched on every block entry and so
	 * is the state block: keep them out of each other's sets. */
	{
		uintptr_t sp = (uintptr_t)maps[PSX_MAP_SCRATCH_PAD].address;
		size_t splen = maps[PSX_MAP_SCRATCH_PAD].length;
		size_t slack = splen + 32;
		char *base = calloc(1, sizeof(*state) + lut_size + slack);
		uintptr_t p;

		if (!base)
			goto err_finish_jit;

		p = ((uintptr_t)base + 31) & ~(uintptr_t)31;
		if ((p & (OCACHE_SIZE - 1)) - (sp & (OCACHE_SIZE - 1)) < splen)
			p += splen;

		state = (struct lightrec_state *)p;
		state->alloc_base = base;
		state->alloc_slack = slack;
	}

	lightrec_register(MEM_FOR_LIGHTREC,
			  sizeof(*state) + lut_size + state->alloc_slack);

	state->tlsf = tlsf;
	state->with_32bit_lut = with_32bit_lut;
	state->in_delay_slot_n = 0xff;
	state->cycles_per_op = 2;

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

	state->backend_ops = state_backend;
	if (state->backend_ops->init && state->backend_ops->init(state))
		goto err_free_reaper;

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
			    lut_elm_size(state) * CODE_LUT_SIZE
			    + state->alloc_slack);
	free(state->alloc_base);
err_finish_jit:
	if (state_backend->global_fini)
		state_backend->global_fini();
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
	if (state->backend_ops->destroy)
		state->backend_ops->destroy(state);

	if (ENABLE_THREADED_COMPILER) {
		lightrec_free_recompiler(state->rec);
		lightrec_reaper_destroy(state->reaper);
	} else {
		lightrec_free_cstate(state->cstate);
	}

	if (state->backend_ops->global_fini)
		state->backend_ops->global_fini();
	if (ENABLE_CODE_BUFFER && state->tlsf)
		tlsf_destroy(state->tlsf);

	lightrec_unregister(MEM_FOR_LIGHTREC, sizeof(*state) +
			    lut_elm_size(state) * CODE_LUT_SIZE
			    + state->alloc_slack);
	free(state->alloc_base);
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

	/* Unlink first: see lightrec_invalidate_map.  Only the slots this is
	 * about -- the window below and no more. */
	if (state->backend_ops->unlink_range)
		state->backend_ops->unlink_range(state, lut_offset(kaddr),
						 (len + 3) / 4,
						 LIGHTREC_UNLINK_INV);

	lr_inv_calls++;
	{
		/* Was anything actually there?  A null slot means this store
		 * paid the whole check to clear nothing. */
		u32 n = (len + 3) / 4, i;
		void *p = lut_address(state, lut_offset(kaddr));

		for (i = 0; i < n; i++) {
			if (lut_elm_size(state) == 4
			    ? ((const u32 *)p)[i] != 0
			    : ((const u16 *)p)[i] != 0) {
				lr_inv_live++;
				break;
			}
		}
	}

	memset(lut_address(state, lut_offset(kaddr)), 0,
	       ((len + 3) / 4) * lut_elm_size(state));
}

void lightrec_invalidate_all(struct lightrec_state *state)
{
	/* Every slot at once, so every link is certainly stale. */
	if (state->backend_ops->unlink_all)
		state->backend_ops->unlink_all(state, LIGHTREC_UNLINK_INV_ALL);

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

	if (state->backend_ops->cycles_moved)
		state->backend_ops->cycles_moved(state);
	}
}

u32 lightrec_exit_flags(struct lightrec_state *state)
{
	return state->exit_flags;
}

u32 lightrec_get_curr_pc(struct lightrec_state *state)
{
	return state->curr_pc;
}

void lightrec_set_curr_pc(struct lightrec_state *state, u32 pc)
{
	state->curr_pc = pc;
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

	if (state->backend_ops->cycles_moved)
		state->backend_ops->cycles_moved(state);
}

void lightrec_set_target_cycle_count(struct lightrec_state *state, u32 cycles)
{
	if (state->exit_flags == LIGHTREC_EXIT_NORMAL) {
		if (cycles < state->current_cycle)
			cycles = state->current_cycle;

		state->target_cycle = cycles;

	if (state->backend_ops->cycles_moved)
		state->backend_ops->cycles_moved(state);
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

	if (state->backend_ops->cycles_changed)
		state->backend_ops->cycles_changed(state);

	if (ENABLE_THREADED_COMPILER) {
		lightrec_recompiler_pause(state->rec);
		lightrec_reaper_reap(state->reaper);
	}

	lightrec_invalidate_all(state);
	lightrec_free_all_blocks(state->block_cache);

	if (ENABLE_THREADED_COMPILER)
		lightrec_recompiler_unpause(state->rec);
}

#if !defined(LIGHTREC_NO_LIGHTNING)
/*
 * ===========================================================================
 * THE GNU LIGHTNING BACKEND
 * ===========================================================================
 *
 * Everything above that mentions a jit_ type belongs here conceptually; it is
 * still in this file only because moving four hundred lines and moving the
 * seam at the same time makes a regression impossible to attribute.  The
 * seam is what matters and it is below.  A build that does not want Lightning
 * compiles this file with -DLIGHTREC_NO_LIGHTNING and gets a lightrec.c with
 * no code generator in it at all -- which is exactly what a second backend
 * needed, and what previously required forking the file.
 */

static int lightning_global_init(char *argv0)
{
	init_jit_with_debug(argv0, stdout);
	return 0;
}

static void lightning_global_fini(void)
{
	finish_jit();
}

static int lightning_init(struct lightrec_state *state)
{
	state->dispatcher = generate_dispatcher(state);
	if (!state->dispatcher)
		return -ENOMEM;

	/* The wrapper table is how emitted code calls back into C.  It is
	 * indexed by an immediate baked into the emitted call, so it is the
	 * emitter's ABI and not the core's. */
	state->c_wrappers[C_WRAPPER_RW] = lightrec_rw_cb;
	state->c_wrappers[C_WRAPPER_RW_GENERIC] = lightrec_rw_generic_cb;
	state->c_wrappers[C_WRAPPER_MFC] = lightrec_mfc_cb;
	state->c_wrappers[C_WRAPPER_MTC] = lightrec_mtc_cb;
	state->c_wrappers[C_WRAPPER_CP] = lightrec_cp_cb;

	return 0;
}

static void lightning_destroy(struct lightrec_state *state)
{
	lightrec_free_block(state, state->dispatcher);
}

static void lightning_free_block(struct lightrec_state *state,
				 struct block *block)
{
	if (block->_jit) {
		_jit_destroy_state(block->_jit);
		block->_jit = NULL;
	}
}

static void lightning_reap_block(struct lightrec_state *state, void *priv)
{
	_jit_destroy_state(priv);
}

const struct lightrec_backend lightrec_backend_lightning = {
	.name		= "lightning",
	.global_init	= lightning_global_init,
	.global_fini	= lightning_global_fini,
	.init		= lightning_init,
	.destroy	= lightning_destroy,
	.compile	= lightning_compile,
	.free_block	= lightning_free_block,
	.reap_block	= lightning_reap_block,
	.execute	= lightning_execute,
};
#endif /* !LIGHTREC_NO_LIGHTNING */

/*
 * ===========================================================================
 * BACKEND SELECTION
 * ===========================================================================
 */

static const struct lightrec_backend * const lightrec_backends[] = {
#if !defined(LIGHTREC_NO_LIGHTNING)
	&lightrec_backend_lightning,
#endif
#if defined(LIGHTREC_WITH_FGL)
	&lightrec_backend_fgl,
#endif
};

static const struct lightrec_backend *lightrec_backend_selected;

const struct lightrec_backend * lightrec_select_backend(const char *name)
{
	unsigned int i;

	if (!name) {
		/* A second call with no name means "whatever was chosen".
		 * The first such call is lightrec_init()'s, and it is also
		 * where $LIGHTREC_BACKEND gets its chance. */
		if (lightrec_backend_selected)
			return lightrec_backend_selected;

		name = getenv("LIGHTREC_BACKEND");
	}

	for (i = 0; name && i < ARRAY_SIZE(lightrec_backends); i++) {
		if (!strcmp(lightrec_backends[i]->name, name)) {
			lightrec_backend_selected = lightrec_backends[i];
			return lightrec_backend_selected;
		}
	}

	if (name)
		pr_err("No such lightrec backend '%s', using '%s'\n",
		       name, lightrec_backends[0]->name);

	lightrec_backend_selected = lightrec_backends[0];
	return lightrec_backend_selected;
}

const struct lightrec_backend *
lightrec_get_backend(const struct lightrec_state *state)
{
	return state->backend_ops;
}
