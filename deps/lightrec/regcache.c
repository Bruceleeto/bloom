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
	struct native_register lightrec_regs[NUM_POOL + NUM_TEMPS];
};

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

#if NUM_PINNED
/* bleem's pin set: v0 v1 a0 a1 at sp. */
const u8 lightrec_pin_guest[NUM_PINNED] = { 2, 3, 4, 5, 1, 29 };
_Static_assert(PIN_FIRST_SLOT + NUM_PINNED <= NUM_REGS,
	       "pins do not fit in the callee-saved pool");
#else
const u8 lightrec_pin_guest[1] = { 0 };
#endif

_Bool lightrec_guest_is_pinned(u16 reg)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++)
		if (lightrec_pin_guest[i] == reg)
			return true;

	return false;
}

static inline bool slot_is_pinned(unsigned int slot)
{
	return NUM_PINNED && slot >= PIN_FIRST_SLOT
		&& slot < PIN_FIRST_SLOT + NUM_PINNED;
}

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

static inline u8 lightrec_reg_to_lightning(const struct regcache *cache,
		const struct native_register *nreg)
{
	u8 offset = lightrec_reg_number(cache, nreg);

	if (offset < NUM_REGS)
		return JIT_V(FIRST_REG + offset);
#if NUM_ARGREGS
	if (offset < NUM_POOL)
		return FIRST_ARGREG + (offset - NUM_REGS);
#endif
	return JIT_R(FIRST_TEMP + offset - NUM_POOL);
}

static inline struct native_register * lightning_reg_to_lightrec(
		struct regcache *cache, u8 reg)
{
#if NUM_ARGREGS
	/* Must come first: on SH-4 the argument registers sit between the
	 * temporaries and the callee-saved ones, so neither test below
	 * identifies them. */
	if (reg >= FIRST_ARGREG && reg < FIRST_ARGREG + NUM_ARGREGS)
		return &cache->lightrec_regs[NUM_REGS + reg - FIRST_ARGREG];
#endif
	if ((JIT_V0 > JIT_R0 && reg >= JIT_V0) ||
			(JIT_V0 < JIT_R0 && reg < JIT_R0)) {
		if (JIT_V1 > JIT_V0)
			return &cache->lightrec_regs[reg - JIT_V(FIRST_REG)];
		else
			return &cache->lightrec_regs[JIT_V(FIRST_REG) - reg];
	} else {
		if (JIT_R1 > JIT_R0)
			return &cache->lightrec_regs[NUM_POOL + reg - JIT_R(FIRST_TEMP)];
		else
			return &cache->lightrec_regs[NUM_POOL + JIT_R(FIRST_TEMP) - reg];
	}
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

static struct native_register * alloc_temp(struct regcache *cache, bool no_arg)
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

		if (slot_is_pinned(i - 1))
			continue;

		/* Only a caller that must survive a call setup excludes the
		 * argument registers - see alloc_temp_no_arg(). */
		if (no_arg && NUM_ARGREGS
		    && i - 1 >= NUM_REGS && i - 1 < NUM_POOL)
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

	/* Try to find if the register is already mapped somewhere */
	nreg = find_mapped_reg(cache, reg, out);
	if (nreg)
		return nreg;

	nreg = NULL;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++) {
		elm = &cache->lightrec_regs[i];

		/* A pin owns its host register for the life of the run;
		 * find_mapped_reg() above is the only way into one. */
		if (slot_is_pinned(i))
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

/* Drop everything the allocator knows about this host register.
 *
 * A pinned slot is the exception: it is the permanent home of one guest
 * register for the whole run, so the binding must survive. Unbinding it -
 * OPT_EARLY_UNLOAD does this after a guest register's last use in a block -
 * hands the pin's host register to the general pool, and the next block's
 * lightrec_regcache_reset() then re-asserts "this slot holds guest rN, no
 * load needed" over whatever the pool left there. The block reads garbage
 * for rN, and when rN is v0 the garbage becomes the target of a jr, which
 * surfaces as "Unable to recompile block at PC 0xffffffff".
 *
 * For a pin, keep the binding and drop only the transient flags. The caller
 * has already stored back a dirty value if it needed to (see
 * lightrec_unload_nreg), so REG_IS_LOADED is the correct resting state. */
static void lightrec_discard_nreg(struct regcache *cache,
				  struct native_register *nreg)
{
	nreg->extended = false;
	nreg->zero_extended = false;
	nreg->output = false;
	nreg->used = false;
	nreg->locked = false;

	if (slot_is_pinned(lightrec_reg_number(cache, nreg))) {
		nreg->prio = REG_IS_LOADED;
		return;
	}

	nreg->emulated_register = -1;
	nreg->prio = 0;
}

static void lightrec_unload_nreg(struct regcache *cache, jit_state_t *_jit,
		struct native_register *nreg, u8 jit_reg)
{
	clean_reg(_jit, nreg, jit_reg, false);
	lightrec_discard_nreg(cache, nreg);
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
	lightrec_unload_nreg(cache, _jit, reg, jit_reg);

	reg->used = true;
	reg->prio = REG_IS_LOADED;
	return jit_reg;
}

u8 lightrec_peek_reg_in(struct regcache *cache, u16 reg)
{
	struct native_register *nreg = find_mapped_reg(cache, reg, false);

	return nreg ? lightrec_reg_to_lightning(cache, nreg) : 0;
}

_Bool lightrec_reg_is_pinned_host(u8 jit_reg)
{
	return NUM_PINNED
		&& jit_reg >= JIT_V(FIRST_REG + PIN_FIRST_SLOT)
		&& jit_reg < JIT_V(FIRST_REG + PIN_FIRST_SLOT + NUM_PINNED);
}

_Bool lightrec_reg_is_argreg(u8 jit_reg)
{
	return NUM_ARGREGS && jit_reg >= FIRST_ARGREG
		&& jit_reg < FIRST_ARGREG + NUM_ARGREGS;
}

u8 lightrec_alloc_reg_temp_no_arg(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg = alloc_temp(cache, true);
	u8 jit_reg;

	if (!nreg) {
		pr_err("No more registers! Abandon ship!\n");
		return 0;
	}

	jit_reg = lightrec_reg_to_lightning(cache, nreg);
	lightrec_unload_nreg(cache, _jit, nreg, jit_reg);

	nreg->prio = REG_IS_TEMP;
	nreg->used = true;
	return jit_reg;
}

/* A register to hold a C call target across the argument setup.
 *
 * It has to survive from the load of the target address to the call itself,
 * with jit_prepare() and the outgoing arguments in between, and it cannot be
 * a live temporary. When the pins take the whole callee-saved pool nothing in
 * the ordinary pool fits.
 *
 * The last argument register does, though. Call sites run
 * lightrec_save_argregs() first, so by this point every argument register has
 * been spilled and holds nothing; and the wrapper call passes a single
 * argument, which lightning places in the first of them. The last one is
 * therefore free from the spill until the call, which is exactly the window
 * that is needed.
 *
 * Borrowing a pin's register was tried instead, on the reasoning that pins are
 * clean here and reloaded afterwards. It does not work: the pin's host
 * register is reserved from lightning and still mapped in the cache, the load
 * of the target address does not survive, and the call jumps to whatever guest
 * value was in it. tools/lrtest -c catches it as a fault at the block
 * containing the call, at an address equal to the pinned guest register.
 *
 * Must only be called after lightrec_save_argregs().
 */
u8 lightrec_alloc_call_target(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg = alloc_temp(cache, true);

	if (nreg) {
		u8 jit_reg = lightrec_reg_to_lightning(cache, nreg);

		lightrec_unload_nreg(cache, _jit, nreg, jit_reg);
		nreg->prio = REG_IS_TEMP;
		nreg->used = true;
		return jit_reg;
	}

#if NUM_ARGREGS
	return FIRST_ARGREG + NUM_ARGREGS - 1;
#else
	pr_err("No more registers! Abandon ship!\n");
	return 0;
#endif
}

/* True when the register came from the borrowed-argument-register path above,
 * and so was never allocated from the pool and must not be freed back into
 * it. */
_Bool lightrec_call_target_is_pin(u8 jit_reg)
{
	return NUM_ARGREGS && jit_reg == FIRST_ARGREG + NUM_ARGREGS - 1;
}

u8 lightrec_alloc_reg_temp(struct regcache *cache, jit_state_t *_jit)
{
	u8 jit_reg;
	struct native_register *nreg = alloc_temp(cache, false);
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
		reg = lightrec_alloc_reg_temp(cache, _jit);
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
	struct native_register *nreg = lightning_reg_to_lightrec(cache, JIT_V0);

	return nreg->prio == REG_IS_LOADED && nreg->emulated_register == REG_PC;
}

void lightrec_load_imm(struct regcache *cache,
		       jit_state_t *_jit, u8 jit_reg, u32 pc, u32 imm)
{
	s32 delta = imm - pc;

	if (!reg_pc_is_mapped(cache) || !can_sign_extend(delta, 16))
		jit_movi(jit_reg, imm);
	else if (jit_reg != JIT_V0 || delta)
		jit_addi(jit_reg, JIT_V0, delta);
}

void lightrec_load_next_pc_imm(struct regcache *cache,
			       jit_state_t *_jit, u32 pc, u32 imm)
{
	struct native_register *nreg = lightning_reg_to_lightrec(cache, JIT_V0);
	u8 reg = JIT_V0;

	if (lightrec_store_next_pc())
		reg = lightrec_alloc_reg_temp(cache, _jit);

	if (reg_pc_is_mapped(cache)) {
		/* JIT_V0 contains next PC - so we can overwrite it */
		lightrec_load_imm(cache, _jit, reg, pc, imm);
	} else {
		/* JIT_V0 contains something else - invalidate it */
		if (reg == JIT_V0)
		      lightrec_unload_reg(cache, _jit, JIT_V0);

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

	/* Invalidate JIT_V0 if it is not mapped to 'reg' */
	nreg_v0 = lightning_reg_to_lightrec(cache, JIT_V0);
	if (nreg_v0->prio >= REG_IS_LOADED && nreg_v0->emulated_register != reg)
		lightrec_unload_nreg(cache, _jit, nreg_v0, JIT_V0);

	nreg = find_mapped_reg(cache, reg, false);
	if (!nreg) {
		/* Not mapped - load the value from the register cache */

		offset = lightrec_offset(regs.gpr) + (reg << 2);
		jit_ldxi_ui(JIT_V0, LIGHTREC_REG_STATE, offset);

		nreg_v0->prio = REG_IS_LOADED;
		nreg_v0->emulated_register = reg;

	} else if (nreg == nreg_v0) {
		/* The target register 'reg' is mapped to JIT_V0 */

		if (!nreg->zero_extended)
			jit_extr_ui(JIT_V0, JIT_V0);

	} else {
		/* The target register 'reg' is mapped elsewhere. In that case,
		 * move the register's value to JIT_V0 and re-map it in the
		 * register cache. We can then safely discard the original
		 * mapped register (even if it was dirty). */

		jit_reg = lightrec_reg_to_lightning(cache, nreg);
		if (nreg->zero_extended)
			jit_movr(JIT_V0, jit_reg);
		else
			jit_extr_ui(JIT_V0, jit_reg);

		if (slot_is_pinned(lightrec_reg_number(cache, nreg))) {
			/* The pin keeps its mapping; JIT_V0 just carries a
			 * copy of the value, mapped to nothing. Moving the
			 * mapping would leave two slots claiming the same
			 * guest register. */
			nreg_v0->emulated_register = -1;
			nreg_v0->prio = REG_IS_LOADED;
		} else {
			*nreg_v0 = *nreg;
			lightrec_discard_nreg(cache, nreg);
		}
	}

	if (lightrec_store_next_pc()) {
		jit_stxi_i(lightrec_offset(next_pc),
			   LIGHTREC_REG_STATE, JIT_V0);
	} else {
		lightrec_clean_reg(cache, _jit, JIT_V0);

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

static void clean_regs(struct regcache *cache, jit_state_t *_jit, bool clean)
{
	unsigned int i;

	for (i = 0; i < NUM_REGS; i++) {
		clean_reg(_jit, &cache->lightrec_regs[i],
			  JIT_V(FIRST_REG + i), clean);
	}
#if NUM_ARGREGS
	for (i = 0; i < NUM_ARGREGS; i++) {
		clean_reg(_jit, &cache->lightrec_regs[NUM_REGS + i],
			  FIRST_ARGREG + i, clean);
	}
#endif
	for (i = 0; i < NUM_TEMPS; i++) {
		clean_reg(_jit, &cache->lightrec_regs[i + NUM_POOL],
				JIT_R(FIRST_TEMP + i), clean);
	}
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
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cache->lightrec_regs); i++)
		if (cache->lightrec_regs[i].prio == REG_IS_DIRTY)
			return true;

	return false;
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

	lightrec_discard_nreg(cache, nreg);
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
	unsigned int i;

	memset(&cache->lightrec_regs, 0, sizeof(cache->lightrec_regs));

	/* Pins are not reset with the rest: their host register still holds
	 * the guest value at this point at run time, so the next block must
	 * not emit a load for it. This is the whole win - it removes the
	 * per-boundary reload that a per-block allocator pays ~144k times a
	 * frame. */
	for (i = 0; i < NUM_PINNED; i++) {
		struct native_register *nreg =
			&cache->lightrec_regs[PIN_FIRST_SLOT + i];

		nreg->emulated_register = lightrec_pin_guest[i];
		nreg->prio = REG_IS_LOADED;
	}
}

void lightrec_preload_pc(struct regcache *cache, jit_state_t *_jit)
{
	struct native_register *nreg;

	/* The block's PC is loaded in JIT_V0 at the start of the block */
	nreg = lightning_reg_to_lightrec(cache, JIT_V0);
	nreg->emulated_register = REG_PC;
	nreg->prio = REG_IS_LOADED;
	nreg->zero_extended = true;

	jit_live(JIT_V0);
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

#if NUM_ARGREGS
	/* Not conditional on _WIN32 like the block above: lightning really
	 * does use r4-r7 for outgoing arguments and as scratch, so a guest
	 * value parked in one has to be declared live every time. */
	for (i = 0; i < NUM_ARGREGS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_live(FIRST_ARGREG + i);
	}
#endif

	for (i = 0; i < NUM_TEMPS; i++) {
		nreg = &cache->lightrec_regs[NUM_POOL + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_live(JIT_R(FIRST_TEMP + i));
	}

	jit_live(LIGHTREC_REG_STATE);
	jit_live(LIGHTREC_REG_CYCLE);
}

void lightrec_clean_pins(struct regcache *cache, jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		clean_reg(_jit, &cache->lightrec_regs[PIN_FIRST_SLOT + i],
			  JIT_V(FIRST_REG + PIN_FIRST_SLOT + i), true);
	}
}

void lightrec_emit_pin_tripwire(struct regcache *cache, jit_state_t *_jit,
				u32 pc)
{
#if OPT_PIN_TRIPWIRE
	jit_node_t *ok;
	u8 tmp, tmp2;
	unsigned int i;

	if (!NUM_PINNED)
		return;

	tmp = lightrec_alloc_reg_temp(cache, _jit);
	tmp2 = lightrec_alloc_reg_temp(cache, _jit);

	for (i = 0; i < NUM_PINNED; i++) {
		jit_ldxi_i(tmp, LIGHTREC_REG_STATE,
			   lightrec_offset(regs.gpr)
			   + (lightrec_pin_guest[i] << 2));

		ok = jit_beqr(tmp, JIT_V(FIRST_REG + PIN_FIRST_SLOT + i));

		jit_movi(tmp2, pc);
		jit_stxi_i(lightrec_offset(dbg_pin_pc), LIGHTREC_REG_STATE, tmp2);
		jit_movi(tmp2, i);
		jit_stxi_i(lightrec_offset(dbg_pin_idx), LIGHTREC_REG_STATE, tmp2);

		jit_patch(ok);
	}

	lightrec_free_reg(cache, tmp);
	lightrec_free_reg(cache, tmp2);
#endif
}

void lightrec_regcache_load_pins(jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		jit_ldxi_i(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i),
			   LIGHTREC_REG_STATE,
			   lightrec_offset(regs.gpr)
			   + (lightrec_pin_guest[i] << 2));
	}
}

void lightrec_regcache_store_pins(jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++) {
		jit_stxi_i(lightrec_offset(regs.gpr)
			   + (lightrec_pin_guest[i] << 2),
			   LIGHTREC_REG_STATE,
			   JIT_V(FIRST_REG + PIN_FIRST_SLOT + i));
	}
}

void lightrec_regcache_reserve_pins(jit_state_t *_jit)
{
	unsigned int i;

	for (i = 0; i < NUM_PINNED; i++)
		jit_reserve_reg(JIT_V(FIRST_REG + PIN_FIRST_SLOT + i));
}

void lightrec_regcache_reserve(jit_state_t *_jit)
{
	lightrec_regcache_reserve_pins(_jit);

#if NUM_ARGREGS
	unsigned int i;

	/* lightning picks the lowest-numbered free register for its own
	 * scratch, which reaches r4-r7 as soon as r1-r3 are busy - and r1-r3
	 * always are here (cycle counter plus two temporaries). Without this
	 * the allocator silently clobbers guest values. */
	for (i = 0; i < NUM_ARGREGS; i++)
		jit_reserve_reg(FIRST_ARGREG + i);
#endif
}

void lightrec_save_argregs(struct regcache *cache, jit_state_t *_jit)
{
#if NUM_ARGREGS
	const struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < NUM_ARGREGS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_stxi(lightrec_offset(wrapper_regs[NUM_TEMPS + i]),
				 LIGHTREC_REG_STATE, FIRST_ARGREG + i);
	}
#endif
}

void lightrec_restore_argregs(struct regcache *cache, jit_state_t *_jit)
{
#if NUM_ARGREGS
	const struct native_register *nreg;
	unsigned int i;

	for (i = 0; i < NUM_ARGREGS; i++) {
		nreg = &cache->lightrec_regs[NUM_REGS + i];

		if (nreg->used || nreg->prio > REG_IS_TEMP)
			jit_ldxi(FIRST_ARGREG + i, LIGHTREC_REG_STATE,
				 lightrec_offset(wrapper_regs[NUM_TEMPS + i]));
	}
#endif
}
