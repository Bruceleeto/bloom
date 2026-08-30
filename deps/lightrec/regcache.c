// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Copyright (C) 2014-2021 Paul Cercueil <paul@crapouillou.net>
 */

#include "debug.h"
#include "memmanager.h"
#include "lightning-wrapper.h"
#include "regcache.h"

#include <stdbool.h>
#include <stddef.h>

#define REG_PC (offsetof(struct lightrec_state, curr_pc) / sizeof(u32))

enum reg_priority {
	REG_IS_TEMP,
	REG_IS_TEMP_VALUE,
	REG_IS_ZERO,
	REG_IS_LOADED,
	REG_IS_DIRTY,

	REG_NB_PRIORITIES,
};

struct native_register {
	bool used, output, extend, extended,
	     zero_extend, zero_extended, locked;
	s16 emulated_register;
	intptr_t value;
	enum reg_priority prio;
};

struct regcache {
	struct lightrec_state *state;
	struct native_register lightrec_regs[NUM_REGS + NUM_TEMPS];

};

/*
 * PINNED REGISTERS.  Three guest registers live in three fixed callee-saved
 * host registers in every block (bleem's model, bloop's alloc.h): the
 * mapping is a compile-time constant, so a local branch inside a block
 * hands them over with no store/reload, and every C crossing inside a
 * block finds them where the ABI preserves them.  The dispatcher hands
 * over in r4/r5 (LIGHTREC_REG_PC/AUX), so r8-r12 are all free for it.
 *
 * THE CONTRACT.  Inside JIT land a pinned register is always in its host
 * register and always counts as dirty: it may carry a value the state
 * block has not seen, from any number of blocks back.  Consequences:
 *   - a block starts with every pin loaded + dirty (no code: a linked
 *     edge brings them in the registers; the code-LUT entry is a fixed
 *     stub of NUM_PINNED loads in front of the block, see
 *     lightrec_regcache_entry_loads, and a link enters right after it);
 *   - local branches and targets move nothing for them;
 *   - a linked exit stores nothing for them;
 *   - every exit that leaves JIT land (dispatcher, interpreter, ds_check,
 *     the cycle-exhausted fallback of a link) stores the dirty ones;
 *   - C that writes a pinned guest register in memory (the RW wrapper for
 *     loads, MFC) is followed by an "unload": prio == REG_IS_TEMP with the
 *     guest number kept, "reload before the next read".  A linked exit
 *     reloads those first, so the registers always carry the truth across
 *     an edge.  C that only reads gets a clean (store) first, as before.
 * A pinned slot is never handed to another guest register.  The set is a
 * global constant (v0 v1 a0 = 55% of Rayman's guest register references).
 */
#define PIN_FIRST_SLOT 0
#define NUM_PINNED LIGHTREC_NUM_PINNED
/* Ordered by Rayman's register reference counts (v0 32%, v1 13, a0 10,
 * a1 8, at 3.4); every one sits below offset 60 in regs.gpr so its entry
 * load is a single 2-byte `mov.l @(disp,Rm),Rn`, which the fixed-size
 * stub depends on (s0/sp would not be). */
#if NUM_PINNED
/* Ordered by how often the guest register is touched, so any prefix of this
 * table is the right set for that pin count and NUM_PINNED can be swept. */
static const u8 pin_guest_all[6] = { 2, 3, 4, 5, 1, 6 };
#define pin_guest pin_guest_all
_Static_assert(NUM_PINNED <= 6, "pin_guest_all is too short");
#else
static const u8 pin_guest[1] = { 0 };   /* NUM_PINNED 0: never indexed */
#endif
_Static_assert(PIN_FIRST_SLOT + NUM_PINNED <= NUM_VREGS, "pins exceed V pool");
_Static_assert(NUM_PINNED == LIGHTREC_NUM_PINNED, "regcache.h disagrees");

static inline int pin_slot_of_guest(u16 reg)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++)
		if (pin_guest[i] == reg)
			return PIN_FIRST_SLOT + i;

	return -1;
}

static inline bool nreg_is_pinned(const struct regcache *cache,
				  const struct native_register *nreg)
{
	unsigned int idx = (unsigned int)(nreg - cache->lightrec_regs);

	return idx >= PIN_FIRST_SLOT && idx < PIN_FIRST_SLOT + NUM_PINNED;
}

bool lightrec_reg_is_pinned(u16 reg)
{
	return pin_slot_of_guest(reg) >= 0;
}

static void lightrec_pins_reset(struct regcache *cache)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		struct native_register *nreg =
			&cache->lightrec_regs[PIN_FIRST_SLOT + i];

		nreg->emulated_register = pin_guest[i];
		nreg->prio = REG_IS_TEMP;
	}
}

/* The canonical in-block state: every pin in its register, dirty. */
static void lightrec_pins_canonical(struct regcache *cache)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		struct native_register *nreg =
			&cache->lightrec_regs[PIN_FIRST_SLOT + i];

		nreg->extended = true;
		nreg->zero_extended = false;
		nreg->prio = REG_IS_DIRTY;
	}
}

/* Reload the pins whose memory copy is newer (unloaded after a C write),
 * so the registers carry the truth: before a local edge or a linked
 * exit. */
static void lightrec_pins_reload_stale(struct regcache *cache,
				       jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		nreg = &cache->lightrec_regs[PIN_FIRST_SLOT + i];

		if (nreg->prio < REG_IS_LOADED) {
			jit_ldxi_i(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i),
				   LIGHTREC_REG_STATE,
				   lightrec_offset(regs.gpr) + (pin_guest[i] << 2));
			nreg->extended = true;
			nreg->zero_extended = false;
			nreg->prio = REG_IS_DIRTY;
		}
	}
}

static const char * mips_regs[] = {
	"zero",
	"at",
	"v0", "v1",
	"a0", "a1", "a2", "a3",
	"t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
	"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
	"t8", "t9",
	"k0", "k1",
	"gp", "sp", "fp", "ra",
	"lo", "hi",
};

/* Forward declaration(s) */
static void clean_reg(jit_state_t *_jit,
		      struct native_register *nreg, u8 jit_reg, bool clean);

const char * lightrec_reg_name(u8 reg)
{
	return mips_regs[reg];
}

static inline bool lightrec_reg_is_zero(u8 jit_reg)
{
#if defined(__mips__) || defined(__alpha__) || defined(__riscv)
	if (jit_reg == _ZERO)
		return true;
#endif
	return false;
}

static inline s8 lightrec_get_hardwired_reg(u16 reg)
{
#if defined(__mips__) || defined(__alpha__) || defined(__riscv)
	if (reg == 0)
		return _ZERO;
#endif
	return -1;
}

static inline u8 lightrec_reg_number(const struct regcache *cache,
		const struct native_register *nreg)
{
	return (u8) (((uintptr_t) nreg - (uintptr_t) cache->lightrec_regs)
			/ sizeof(*nreg));
}

/* Slot layout: [0, NUM_VREGS) = LIGHTREC_REG_PC.., [NUM_VREGS, NUM_REGS) = r4..r7,
 * [NUM_REGS, +NUM_TEMPS) = JIT_R(FIRST_TEMP).. */
static inline u8 slot_to_jit(unsigned int idx)
{
	if (idx < NUM_VREGS)
		return JIT_V(FIRST_REG + idx);
#if NUM_ARGREGS
	if (idx < NUM_REGS)
		return _R4 + (idx - NUM_VREGS);
#endif
	return JIT_R(FIRST_TEMP + idx - NUM_REGS);
}

static inline unsigned int jit_to_slot(u8 reg)
{
	if (reg >= JIT_V(FIRST_REG))
		return reg - JIT_V(FIRST_REG);
#if NUM_ARGREGS
	if (reg >= _R4)
		return NUM_VREGS + reg - _R4;
#endif
	return NUM_REGS + reg - JIT_R(FIRST_TEMP);
}

static inline u8 lightrec_reg_to_lightning(const struct regcache *cache,
		const struct native_register *nreg)
{
	return slot_to_jit(lightrec_reg_number(cache, nreg));
}

static inline struct native_register * lightning_reg_to_lightrec(
		struct regcache *cache, u8 reg)
{
	return &cache->lightrec_regs[jit_to_slot(reg)];
}

static inline bool slot_is_argreg(unsigned int idx)
{
	return idx >= NUM_VREGS && idx < NUM_REGS;
}

u8 lightrec_get_reg_in_flags(struct regcache *cache, u8 jit_reg)
{
	struct native_register *reg;
	u8 flags = 0;

	if (lightrec_reg_is_zero(jit_reg))
		return REG_EXT | REG_ZEXT;

	reg = lightning_reg_to_lightrec(cache, jit_reg);
	if (reg->extended)
		flags |= REG_EXT;
	if (reg->zero_extended)
		flags |= REG_ZEXT;

	return flags;
}

void lightrec_set_reg_out_flags(struct regcache *cache, u8 jit_reg, u8 flags)
{
	struct native_register *reg;

	if (!lightrec_reg_is_zero(jit_reg)) {
		reg = lightning_reg_to_lightrec(cache, jit_reg);
		reg->extend = flags & REG_EXT;
		reg->zero_extend = flags & REG_ZEXT;
	}
}

static struct native_register * alloc_temp(struct regcache *cache)
{
	struct native_register *elm, *nreg = NULL;
	enum reg_priority best = REG_NB_PRIORITIES;
	unsigned int i;

	/* We search the register list in reverse order. As temporaries are
	 * meant to be used only in the emitter functions, they can be mapped to
	 * caller-saved registers, as they won't have to be saved back to
	 * memory. */
	for (i = ARRAY_SIZE(cache->lightrec_regs); i; i--) {
		elm = &cache->lightrec_regs[i - 1];

		if (nreg_is_pinned(cache, elm))
			continue;

		if (!elm->used && !elm->locked && elm->prio < best) {
			nreg = elm;
			best = elm->prio;

			if (best == REG_IS_TEMP)
				break;
		}
	}

	return nreg;
}

static struct native_register * find_mapped_reg(struct regcache *cache,
						u16 reg, bool out)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++) {
		struct native_register *nreg = &cache->lightrec_regs[i];
		if ((nreg->prio >= REG_IS_ZERO) &&
		    nreg->emulated_register == reg &&
		    (!out || !nreg->locked))
			return nreg;
	}

	return NULL;
}

static struct native_register * alloc_in_out(struct regcache *cache,
					     u16 reg, bool out)
{
	struct native_register *elm, *nreg = NULL;
	enum reg_priority best = REG_NB_PRIORITIES;
	unsigned int i;

	int slot = pin_slot_of_guest(reg);

	/* A pinned register has exactly one home, loaded or not. */
	if (slot >= 0)
		return &cache->lightrec_regs[slot];

	/* Try to find if the register is already mapped somewhere */
	nreg = find_mapped_reg(cache, reg, out);
	if (nreg)
		return nreg;

	nreg = NULL;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++) {
		elm = &cache->lightrec_regs[i];

		if (nreg_is_pinned(cache, elm))
			continue;

		if (!elm->used && !elm->locked && elm->prio < best) {
			nreg = elm;
			best = elm->prio;

			if (best == REG_IS_TEMP)
				break;
		}
	}

	return nreg;
}

static void lightrec_discard_nreg(struct native_register *nreg)
{
	nreg->extended = false;
	nreg->zero_extended = false;
	nreg->output = false;
	nreg->used = false;
	nreg->locked = false;
	nreg->emulated_register = -1;
	nreg->prio = 0;
}

/* Same for a pinned slot: the value is gone (or the state block now holds
 * a newer one), the home stays. */
static void lightrec_discard_pinned(struct native_register *nreg)
{
	s16 guest = nreg->emulated_register;

	lightrec_discard_nreg(nreg);
	nreg->emulated_register = guest;
}

static void lightrec_unload_nreg(struct regcache *cache, jit_state_t *_jit,
		struct native_register *nreg, u8 jit_reg)
{
	clean_reg(_jit, nreg, jit_reg, false);

	if (nreg_is_pinned(cache, nreg))
		lightrec_discard_pinned(nreg);
	else
		lightrec_discard_nreg(nreg);
}

void lightrec_unload_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg)
{
	if (lightrec_reg_is_zero(jit_reg))
		return;

	lightrec_unload_nreg(cache, _jit,
			lightning_reg_to_lightrec(cache, jit_reg), jit_reg);
}

u8 lightrec_alloc_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg)
{
	struct native_register *reg;

	if (lightrec_reg_is_zero(jit_reg))
		return jit_reg;

	reg = lightning_reg_to_lightrec(cache, jit_reg);
	if (nreg_is_pinned(cache, reg))
		pr_err("lightrec_alloc_reg() on a pinned host register!\n");
	lightrec_unload_nreg(cache, _jit, reg, jit_reg);

	reg->used = true;
	reg->prio = REG_IS_LOADED;
	return jit_reg;
}

static u8 lightrec_alloc_real_temp(struct regcache *cache, jit_state_t *_jit)
{
	u8 jit_reg;
	struct native_register *nreg = alloc_temp(cache);
	if (!nreg) {
		/* No free register, no dirty register to free. */
		pr_err("No more registers! Abandon ship!\n");
		return 0;
	}

	jit_reg = lightrec_reg_to_lightning(cache, nreg);
	lightrec_unload_nreg(cache, _jit, nreg, jit_reg);

	nreg->prio = REG_IS_TEMP;
	nreg->used = true;
	return jit_reg;
}

u8 lightrec_alloc_reg_temp(struct regcache *cache, jit_state_t *_jit)
{
	return lightrec_alloc_real_temp(cache, _jit);
}

s8 lightrec_get_reg_with_value(struct regcache *cache, intptr_t value)
{
	struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg->prio == REG_IS_TEMP_VALUE && nreg->value == value) {
			nreg->used = true;
			return lightrec_reg_to_lightning(cache, nreg);
		}
	}

	return -1;
}

void lightrec_temp_set_value(struct regcache *cache, u8 jit_reg, intptr_t value)
{
	struct native_register *nreg;

	nreg = lightning_reg_to_lightrec(cache, jit_reg);

	nreg->prio = REG_IS_TEMP_VALUE;
	nreg->value = value;
}

u8 lightrec_alloc_reg_temp_with_value(struct regcache *cache,
				      jit_state_t *_jit, intptr_t value)
{
	s8 reg;

	reg = lightrec_get_reg_with_value(cache, value);
	if (reg < 0) {
		reg = lightrec_alloc_real_temp(cache, _jit);
		jit_movi((u8)reg, value);
		lightrec_temp_set_value(cache, (u8)reg, value);
	}

	return (u8)reg;
}

u8 lightrec_alloc_reg_out(struct regcache *cache, jit_state_t *_jit,
			  u16 reg, u8 flags)
{
	struct native_register *nreg;
	u8 jit_reg;
	s8 hw_reg;

	hw_reg = lightrec_get_hardwired_reg(reg);
	if (hw_reg >= 0)
		return (u8) hw_reg;

	nreg = alloc_in_out(cache, reg, true);
	if (!nreg) {
		/* No free register, no dirty register to free. */
		pr_err("No more registers! Abandon ship!\n");
		return 0;
	}

	jit_reg = lightrec_reg_to_lightning(cache, nreg);

	/* If we get a dirty register that doesn't correspond to the one
	 * we're requesting, store back the old value */
	if (nreg->emulated_register != reg)
		lightrec_unload_nreg(cache, _jit, nreg, jit_reg);

	nreg->used = true;
	nreg->output = true;
	nreg->emulated_register = reg;
	nreg->extend = flags & REG_EXT;
	nreg->zero_extend = flags & REG_ZEXT;
	nreg->prio = reg ? REG_IS_LOADED : REG_IS_ZERO;
	return jit_reg;
}

u8 lightrec_alloc_reg_in(struct regcache *cache, jit_state_t *_jit,
			 u16 reg, u8 flags)
{
	struct native_register *nreg;
	u8 jit_reg;
	bool reg_changed;
	s8 hw_reg;

	hw_reg = lightrec_get_hardwired_reg(reg);
	if (hw_reg >= 0)
		return (u8) hw_reg;

	nreg = alloc_in_out(cache, reg, false);
	if (!nreg) {
		/* No free register, no dirty register to free. */
		pr_err("No more registers! Abandon ship!\n");
		return 0;
	}

	jit_reg = lightrec_reg_to_lightning(cache, nreg);

	/* If we get a dirty register that doesn't correspond to the one
	 * we're requesting, store back the old value */
	reg_changed = nreg->emulated_register != reg;
	if (reg_changed)
		lightrec_unload_nreg(cache, _jit, nreg, jit_reg);

	if (nreg->prio < REG_IS_LOADED && reg != 0) {
		s16 offset = lightrec_offset(regs.gpr) + (reg << 2);

		nreg->zero_extended = flags & REG_ZEXT;
		nreg->extended = !nreg->zero_extended;

		/* Load previous value from register cache */
		if (nreg->zero_extended)
			jit_ldxi_ui(jit_reg, LIGHTREC_REG_STATE, offset);
		else
			jit_ldxi_i(jit_reg, LIGHTREC_REG_STATE, offset);

		nreg->prio = REG_IS_LOADED;
	}

	/* Clear register r0 before use */
	if (reg == 0 && nreg->prio != REG_IS_ZERO) {
		jit_movi(jit_reg, 0);
		nreg->extended = true;
		nreg->zero_extended = true;
		nreg->prio = REG_IS_ZERO;
	}

	nreg->used = true;
	nreg->output = false;
	nreg->emulated_register = reg;

	if ((flags & REG_EXT) && !nreg->extended &&
	    (!nreg->zero_extended || !(flags & REG_ZEXT))) {
		nreg->extended = true;
		nreg->zero_extended = false;
		jit_extr_i(jit_reg, jit_reg);
	} else if (!(flags & REG_EXT) && (flags & REG_ZEXT) &&
		   !nreg->zero_extended) {
		nreg->zero_extended = true;
		nreg->extended = false;
		jit_extr_ui(jit_reg, jit_reg);
	}

	return jit_reg;
}

void lightrec_remap_reg(struct regcache *cache, jit_state_t *_jit,
			u8 jit_reg, u16 reg_out, bool discard)
{
	struct native_register *nreg;

	lightrec_discard_reg_if_loaded(cache, reg_out);

	nreg = lightning_reg_to_lightrec(cache, jit_reg);
	clean_reg(_jit, nreg, jit_reg, !discard);

	nreg->output = true;
	nreg->emulated_register = reg_out;
	nreg->extend = nreg->extended;
	nreg->zero_extend = nreg->zero_extended;
}

static bool reg_pc_is_mapped(struct regcache *cache)
{
	struct native_register *nreg = lightning_reg_to_lightrec(cache, LIGHTREC_REG_PC);

	return nreg->prio == REG_IS_LOADED && nreg->emulated_register == REG_PC;
}

void lightrec_load_imm(struct regcache *cache,
		       jit_state_t *_jit, u8 jit_reg, u32 pc, u32 imm)
{
	s32 delta = imm - pc;

	if (!reg_pc_is_mapped(cache) || !can_sign_extend(delta, 16))
		jit_movi(jit_reg, imm);
	else if (jit_reg != LIGHTREC_REG_PC || delta)
		jit_addi(jit_reg, LIGHTREC_REG_PC, delta);
}

void lightrec_load_next_pc_imm(struct regcache *cache,
			       jit_state_t *_jit, u32 pc, u32 imm)
{
	struct native_register *nreg = lightning_reg_to_lightrec(cache, LIGHTREC_REG_PC);
	u8 reg = LIGHTREC_REG_PC;

	if (lightrec_store_next_pc())
		reg = lightrec_alloc_reg_temp(cache, _jit);

	if (reg_pc_is_mapped(cache)) {
		/* LIGHTREC_REG_PC contains next PC - so we can overwrite it */
		lightrec_load_imm(cache, _jit, reg, pc, imm);
	} else {
		/* LIGHTREC_REG_PC contains something else - invalidate it */
		if (reg == LIGHTREC_REG_PC)
		      lightrec_unload_reg(cache, _jit, LIGHTREC_REG_PC);

		jit_movi(reg, imm);
	}

	if (lightrec_store_next_pc()) {
		jit_stxi_i(lightrec_offset(next_pc), LIGHTREC_REG_STATE, reg);
		lightrec_free_reg(cache, reg);
	} else {
		nreg->prio = REG_IS_LOADED;
		nreg->emulated_register = -1;
		nreg->locked = true;
	}
}

void lightrec_load_next_pc(struct regcache *cache, jit_state_t *_jit, u8 reg)
{
	struct native_register *nreg_v0, *nreg;
	u16 offset;
	u8 jit_reg;

	if (lightrec_store_next_pc()) {
		jit_reg = lightrec_alloc_reg_in(cache, _jit, reg, 0);
		offset = lightrec_offset(next_pc);
		jit_stxi_i(offset, LIGHTREC_REG_STATE, jit_reg);
		lightrec_free_reg(cache, jit_reg);

		return;
	}

	/* Invalidate LIGHTREC_REG_PC if it is not mapped to 'reg' */
	nreg_v0 = lightning_reg_to_lightrec(cache, LIGHTREC_REG_PC);
	if (nreg_v0->prio >= REG_IS_LOADED && nreg_v0->emulated_register != reg)
		lightrec_unload_nreg(cache, _jit, nreg_v0, LIGHTREC_REG_PC);

	nreg = find_mapped_reg(cache, reg, false);
	if (!nreg) {
		/* Not mapped - load the value from the register cache */

		offset = lightrec_offset(regs.gpr) + (reg << 2);
		jit_ldxi_ui(LIGHTREC_REG_PC, LIGHTREC_REG_STATE, offset);

		nreg_v0->prio = REG_IS_LOADED;
		nreg_v0->emulated_register = reg;

	} else if (nreg == nreg_v0) {
		/* The target register 'reg' is mapped to LIGHTREC_REG_PC */

		if (!nreg->zero_extended)
			jit_extr_ui(LIGHTREC_REG_PC, LIGHTREC_REG_PC);

	} else {
		/* The target register 'reg' is mapped elsewhere. In that case,
		 * move the register's value to LIGHTREC_REG_PC and re-map it in the
		 * register cache. We can then safely discard the original
		 * mapped register (even if it was dirty). */

		jit_reg = lightrec_reg_to_lightning(cache, nreg);
		if (nreg->zero_extended)
			jit_movr(LIGHTREC_REG_PC, jit_reg);
		else
			jit_extr_ui(LIGHTREC_REG_PC, jit_reg);

		*nreg_v0 = *nreg;
		if (nreg_is_pinned(cache, nreg))
			lightrec_discard_pinned(nreg);
		else
			lightrec_discard_nreg(nreg);
	}

	if (lightrec_store_next_pc()) {
		jit_stxi_i(lightrec_offset(next_pc),
			   LIGHTREC_REG_STATE, LIGHTREC_REG_PC);
	} else {
		lightrec_clean_reg(cache, _jit, LIGHTREC_REG_PC);

		nreg_v0->zero_extended = true;
		nreg_v0->locked = true;
	}
}

static void free_reg(struct native_register *nreg)
{
	/* Set output registers as dirty */
	if (nreg->used && nreg->output && nreg->emulated_register > 0)
		nreg->prio = REG_IS_DIRTY;
	if (nreg->output) {
		nreg->extended = nreg->extend;
		nreg->zero_extended = nreg->zero_extend;
	}
	nreg->used = false;
}

void lightrec_free_reg(struct regcache *cache, u8 jit_reg)
{
	if (!lightrec_reg_is_zero(jit_reg))
		free_reg(lightning_reg_to_lightrec(cache, jit_reg));
}

void lightrec_free_regs(struct regcache *cache)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++)
		free_reg(&cache->lightrec_regs[i]);
}

static void clean_reg(jit_state_t *_jit,
		struct native_register *nreg, u8 jit_reg, bool clean)
{
	/* If we get a dirty register, store back the old value */
	if (nreg->prio == REG_IS_DIRTY) {
		s16 offset = lightrec_offset(regs.gpr)
			+ (nreg->emulated_register << 2);

		jit_stxi_i(offset, LIGHTREC_REG_STATE, jit_reg);

		if (clean) {
			if (nreg->emulated_register == 0)
				nreg->prio = REG_IS_ZERO;
			else
				nreg->prio = REG_IS_LOADED;
		}
	}
}

static void lightrec_regs_live(struct regcache *cache, jit_state_t *_jit);

static void clean_regs(struct regcache *cache, jit_state_t *_jit, bool clean)
{
	unsigned int i;

	for (i = 0; i < NUM_REGS + NUM_TEMPS; i++)
		clean_reg(_jit, &cache->lightrec_regs[i], slot_to_jit(i), clean);

	lightrec_regs_live(cache, _jit);
}

void lightrec_storeback_regs(struct regcache *cache, jit_state_t *_jit)
{
	clean_regs(cache, _jit, false);
}

void lightrec_clean_regs(struct regcache *cache, jit_state_t *_jit)
{
	clean_regs(cache, _jit, true);
}

bool lightrec_has_dirty_regs(struct regcache *cache)
{
	struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < NUM_REGS + NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg_is_pinned(cache, nreg)) {
			if (nreg->prio == REG_IS_TEMP)
				return true;	/* needs a reload on the edge */
		} else if (nreg->prio == REG_IS_DIRTY) {
			return true;
		}
	}

	return false;
}

/* Tell lightning the pinned host registers are live: its own scratch
 * allocation only sees the reads it can reach, and a pinned value's next
 * read is usually behind a patched branch or in the next block. */
static void lightrec_regs_live(struct regcache *cache, jit_state_t *_jit)
{
	const struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++)
		jit_live(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i));

	/* r4-r7 slots are plain gpr class to lightning: say so while they
	 * hold something, or it takes them as scratch. */
	for (i = NUM_VREGS; i < NUM_REGS; i++) {
		nreg = &cache->lightrec_regs[i];
		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_live(slot_to_jit(i));
	}
}

void lightrec_regcache_local_edge(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	lightrec_pins_reload_stale(cache, _jit);
	lightrec_regs_live(cache, _jit);

	for (i = 0; i < NUM_REGS + NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg_is_pinned(cache, nreg))
			continue;

		clean_reg(_jit, nreg, slot_to_jit(i), true);
	}
}

/* Store the unpinned dirty registers (an exit that may take a direct
 * link: the pins stay in their registers on the linked path). */
void lightrec_regcache_clean_unpinned(struct regcache *cache,
				      jit_state_t *_jit)
{
	lightrec_regcache_local_edge(cache, _jit);
}

/* Store the dirty pins: an exit that leaves JIT land. */
void lightrec_regcache_store_pins(struct regcache *cache, jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		clean_reg(_jit, &cache->lightrec_regs[PIN_FIRST_SLOT + i],
			  JIT_V(FIRST_REG + PIN_FIRST_SLOT + i), true);
	}
}

/* A local branch target: the fall-through path settles like any edge, and
 * the cache is reset to the canonical state every incoming path carries. */
void lightrec_regcache_sync_target(struct regcache *cache, jit_state_t *_jit)
{
	lightrec_regcache_local_edge(cache, _jit);
	lightrec_regcache_reset(cache);
	lightrec_pins_canonical(cache);
}

/* The code-LUT entry stub: NUM_PINNED loads, exactly LIGHTREC_PIN_STUB_BYTES
 * of code, so a direct link can enter right behind it with the pins already
 * in their registers. One instruction per pin, or two when the state is in
 * GBR - see LIGHTREC_PIN_STUB_BYTES. */
void lightrec_regcache_entry_loads(struct regcache *cache, jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		jit_ldxi_i(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i),
			   LIGHTREC_REG_STATE,
			   lightrec_offset(regs.gpr) + (pin_guest[i] << 2));
	}
}

/* Same stores, for the dispatcher's trampolines (no cache state). */
void lightrec_regcache_pin_stores_raw(jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		jit_stxi_i(lightrec_offset(regs.gpr) + (pin_guest[i] << 2),
			   LIGHTREC_REG_STATE,
			   JIT_V(FIRST_REG + PIN_FIRST_SLOT + i));
	}
}

/* Same loads, for the dispatcher (no cache state). Used where C may have
 * rewritten the guest register file underneath us - the interpreter, which
 * get_next_block_func() can reach - rather than at every block entry. */
void lightrec_regcache_pin_loads_raw(jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		jit_ldxi_i(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i),
			   LIGHTREC_REG_STATE,
			   lightrec_offset(regs.gpr) + (pin_guest[i] << 2));
	}
}

void lightrec_regcache_pin_block(struct regcache *cache, jit_state_t *_jit)
{
	lightrec_pins_canonical(cache);
}

void lightrec_clean_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg)
{
	struct native_register *reg;

	if (!lightrec_reg_is_zero(jit_reg)) {
		reg = lightning_reg_to_lightrec(cache, jit_reg);
		clean_reg(_jit, reg, jit_reg, true);
	}
}

bool lightrec_reg_is_loaded(struct regcache *cache, u16 reg)
{
	return !!find_mapped_reg(cache, reg, false);
}

void lightrec_clean_reg_if_loaded(struct regcache *cache, jit_state_t *_jit,
				  u16 reg, bool unload)
{
	struct native_register *nreg;
	u8 jit_reg;

	nreg = find_mapped_reg(cache, reg, false);
	if (nreg) {
		jit_reg = lightrec_reg_to_lightning(cache, nreg);

		if (unload)
			lightrec_unload_nreg(cache, _jit, nreg, jit_reg);
		else
			clean_reg(_jit, nreg, jit_reg, true);
	}
}

void lightrec_discard_reg_if_loaded(struct regcache *cache, u16 reg)
{
	struct native_register *nreg;

	nreg = find_mapped_reg(cache, reg, false);
	if (!nreg)
		return;

	if (nreg_is_pinned(cache, nreg))
		lightrec_discard_pinned(nreg);
	else
		lightrec_discard_nreg(nreg);
}

struct native_register * lightrec_regcache_enter_branch(struct regcache *cache)
{
	struct native_register *backup;

	backup = lightrec_malloc(cache->state, MEM_FOR_LIGHTREC,
				 sizeof(cache->lightrec_regs));
	memcpy(backup, &cache->lightrec_regs, sizeof(cache->lightrec_regs));

	return backup;
}

void lightrec_regcache_leave_branch(struct regcache *cache,
			struct native_register *regs)
{
	memcpy(&cache->lightrec_regs, regs, sizeof(cache->lightrec_regs));
	lightrec_free(cache->state, MEM_FOR_LIGHTREC,
		      sizeof(cache->lightrec_regs), regs);
}

void lightrec_regcache_reset(struct regcache *cache)
{
	memset(&cache->lightrec_regs, 0, sizeof(cache->lightrec_regs));
	lightrec_pins_reset(cache);
}

void lightrec_preload_pc(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;

	/* The block's PC is loaded in LIGHTREC_REG_PC at the start of the block */
	nreg = lightning_reg_to_lightrec(cache, LIGHTREC_REG_PC);
	nreg->emulated_register = REG_PC;
	nreg->prio = REG_IS_LOADED;
	nreg->zero_extended = true;

	jit_live(LIGHTREC_REG_PC);
}

void lightrec_preload_imm(struct regcache *cache, jit_state_t *_jit,
			  u8 jit_reg, u32 imm)
{
	struct native_register *nreg;

	nreg = lightning_reg_to_lightrec(cache, jit_reg);
	nreg->prio = REG_IS_TEMP_VALUE;
	nreg->value = imm;

	jit_live(jit_reg);
}

struct regcache * lightrec_regcache_init(struct lightrec_state *state)
{
	struct regcache *cache;

	cache = lightrec_calloc(state, MEM_FOR_LIGHTREC, sizeof(*cache));
	if (!cache)
		return NULL;

	cache->state = state;

	return cache;
}

void lightrec_free_regcache(struct regcache *cache)
{
	return lightrec_free(cache->state, MEM_FOR_LIGHTREC,
			     sizeof(*cache), cache);
}

/*
 * Save and restore the caller-saved temporaries around a direct call to C.
 *
 * Marking registers live does not preserve them: jit_live() constrains
 * lightning's own allocator and emits nothing.  What preserves the temporaries
 * on the normal path is the C wrapper block, which spills all of them to
 * state->wrapper_regs on the way in and reloads them on the way out
 * (lightrec.c, generate_wrapper).  A call that skips the wrapper has to do the
 * same, or whatever guest register was living in a temporary is destroyed by
 * the callee.
 *
 * The wrapper saves every temporary because it cannot know which are in use.
 * Here the register cache is in front of us, so only the live ones are spilled.
 */
void lightrec_save_temps(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	/*
	 * LIGHTREC_REG_CYCLE is JIT_R0 - a temporary, and not one of the
	 * NUM_TEMPS this loop covers.  The wrapper block preserves it by
	 * converting the delta it holds into state->current_cycle before the
	 * call and rebuilding it after; a call that does not need the cycle
	 * count to be readable during it can just put the register somewhere.
	 */
	jit_stxi(lightrec_offset(wrapper_cycle), LIGHTREC_REG_STATE,
		 LIGHTREC_REG_CYCLE);

	for (i = 0; i < NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP) {
			jit_stxi(lightrec_offset(wrapper_regs[i]),
				 LIGHTREC_REG_STATE, JIT_R(FIRST_TEMP + i));
		}
	}

	lightrec_save_argregs(cache, _jit);
}

/* r4-r7 are argument registers: a C call destroys them. Spill the slots
 * that hold something; the C wrapper block does not know about them. */
void lightrec_save_argregs(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	for (i = NUM_VREGS; i < NUM_REGS; i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg->used || nreg->prio > REG_IS_TEMP) {
			jit_stxi(lightrec_offset(wrapper_temps_r4[i - NUM_VREGS]),
				 LIGHTREC_REG_STATE, slot_to_jit(i));
		}
	}
}

void lightrec_restore_argregs(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	for (i = NUM_VREGS; i < NUM_REGS; i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg->used || nreg->prio > REG_IS_TEMP) {
			jit_ldxi(slot_to_jit(i), LIGHTREC_REG_STATE,
				 lightrec_offset(wrapper_temps_r4[i - NUM_VREGS]));
		}
	}
}

void lightrec_restore_temps(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

	jit_ldxi(LIGHTREC_REG_CYCLE, LIGHTREC_REG_STATE,
		 lightrec_offset(wrapper_cycle));

	for (i = 0; i < NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP) {
			jit_ldxi(JIT_R(FIRST_TEMP + i), LIGHTREC_REG_STATE,
				 lightrec_offset(wrapper_regs[i]));
		}
	}

	lightrec_restore_argregs(cache, _jit);
}

void lightrec_regcache_mark_live(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;
	unsigned int i;

#ifdef _WIN32
	/* FIXME: GNU Lightning on Windows seems to use our mapped registers as
	 * temporaries. Until the actual bug is found and fixed, unconditionally
	 * mark our registers as live here. */
	for (i = 0; i < NUM_REGS; i++) {
		nreg = &cache->lightrec_regs[i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_live(JIT_V(FIRST_REG + i));
	}
#endif

	for (i = 0; i < NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_live(JIT_R(FIRST_TEMP + i));
	}

	jit_live(LIGHTREC_REG_STATE);
	jit_live(LIGHTREC_REG_CYCLE);
	lightrec_regs_live(cache, _jit);
}
