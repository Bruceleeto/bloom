/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2014-2021 Paul Cercueil <paul@crapouillou.net>
 */

#ifndef __REGCACHE_H__
#define __REGCACHE_H__

#include "lightning-wrapper.h"
#include "lightrec-config.h"

#ifndef OPT_PIN_TRIPWIRE
#  define OPT_PIN_TRIPWIRE 0
#endif

#if defined(__sh__) && OPT_SH4_USE_GBR
#  define NUM_REGS JIT_V_NUM
#  define LIGHTREC_REG_STATE _GBR
#else
#  define NUM_REGS (JIT_V_NUM - 1)
#  define LIGHTREC_REG_STATE (JIT_V(JIT_V_NUM - 1))
#endif

/* SH-4 r4-r7 are the caller-saved argument registers. lightrec can map guest
 * registers into them, which brings the pool to bleem's ten (r4-r7 plus the
 * six callee-saved r8-r13); anything live in them has to be spilled around a
 * call into C, see lightrec_save_argregs(). They are never handed out as
 * emitter temporaries, so the register holding a call target can never be one
 * of them. */
#if defined(__sh__) && OPT_SH4_USE_GBR
#  define NUM_ARGREGS 4
#  define FIRST_ARGREG _R4
#else
#  define NUM_ARGREGS 0
#endif
#define NUM_POOL (NUM_REGS + NUM_ARGREGS)

/* Static whole-game pinned guest registers (bleem's model).
 *
 * Each pin permanently owns one callee-saved host register: guest pin_guest[i]
 * lives in JIT_V(PIN_FIRST_SLOT + i) for the entire run, not just one block.
 * The SH-4 ABI makes this cheap - r8-r13 are callee-saved, so every C function
 * lightrec calls preserves them.
 *
 * Six pins, bleem's set: v0 v1 a0 a1 at sp. That is the whole callee-saved
 * pool, so the unpinned pool is the four argument registers r4-r7 - which is
 * also bleem's shape, ten registers with six spoken for.
 *
 * Four, not bleem's six, and starting at slot 2. Slots 0 and 1 (JIT_V0 and
 * JIT_V1) are the dispatcher's own working registers - it carries the next PC
 * and the block address in them, and its `loop` label loads the pins and then
 * does jit_jmpr(JIT_V1), so pinning slot 1 overwrites the jump target with a
 * guest register and jumps to it. Reaching bleem's six therefore requires
 * moving the dispatcher off JIT_V0/JIT_V1 first; that same change is what
 * lets the per-block-entry pin reload below be dropped. See pins_parked.md. */
#if defined(__sh__) && OPT_SH4_USE_GBR
#  define NUM_PINNED 6
#  define PIN_FIRST_SLOT 0
#else
#  define NUM_PINNED 0
#  define PIN_FIRST_SLOT 0
#endif


#if defined(__powerpc__)
#  define NUM_TEMPS JIT_R_NUM
/* JIT_R0 is callee-saved on PowerPC, we have to use something else */
#  define LIGHTREC_REG_CYCLE _R10
#  define FIRST_TEMP 0
#elif defined(__sh__) && OPT_SH4_USE_GBR
/* r1 = cycle counter, r2 = lightrec's only temporary. r0 and r3 are left to
 * lightning: r0 is its hard scratch (every GBR-relative and indexed access
 * goes through it) and r3 is the general-purpose scratch its fallback paths
 * ask for via jit_get_reg(). That leaves r4-r13 - bleem's ten - for the
 * pool. */
#  define NUM_TEMPS 1
#  define LIGHTREC_REG_CYCLE JIT_R0
#  define FIRST_TEMP 1
#else
#  define NUM_TEMPS (JIT_R_NUM - 1)
#  define LIGHTREC_REG_CYCLE JIT_R0
#  define FIRST_TEMP 1
#endif

#include "lightrec-private.h"

#define FIRST_REG 0

/* Flags for lightrec_alloc_reg_in / lightrec_alloc_reg_out. */
#define REG_EXT		BIT(0) /* register is sign-extended */
#define REG_ZEXT	BIT(1) /* register is zero-extended */

struct register_value {
	_Bool known;
	u32 value;
};

struct native_register;
struct regcache;

u8 lightrec_alloc_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg);
u8 lightrec_alloc_reg_temp(struct regcache *cache, jit_state_t *_jit);
/* A temporary that is guaranteed not to be an argument register - for values
 * that have to survive the outgoing arguments of a call being set up. */
u8 lightrec_alloc_reg_temp_no_arg(struct regcache *cache, jit_state_t *_jit);
/* A register for a C call target, valid only between lightrec_clean_pins()
 * and lightrec_regcache_load_pins() - see the definition. */
u8 lightrec_alloc_call_target(struct regcache *cache, jit_state_t *_jit);
_Bool lightrec_call_target_is_pin(u8 jit_reg);
_Bool lightrec_reg_is_argreg(u8 jit_reg);
/* True if this host register is the permanent home of a pinned guest
 * register. Such a register must never be renamed to a different guest
 * register - it would silently steal the pin's home. */
_Bool lightrec_reg_is_pinned_host(u8 jit_reg);
/* Host register currently mapped to this guest register, or 0 if unmapped. */
u8 lightrec_peek_reg_in(struct regcache *cache, u16 reg);
/* True if this *guest* register has a permanent host home. */
_Bool lightrec_guest_is_pinned(u16 reg);
u8 lightrec_alloc_reg_out(struct regcache *cache, jit_state_t *_jit,
			  u16 reg, u8 flags);
u8 lightrec_alloc_reg_in(struct regcache *cache, jit_state_t *_jit,
			 u16 reg, u8 flags);

void lightrec_remap_reg(struct regcache *cache, jit_state_t *_jit,
			u8 jit_reg, u16 reg_out, _Bool discard);

void lightrec_load_imm(struct regcache *cache,
		       jit_state_t *_jit, u8 jit_reg, u32 pc, u32 imm);
void lightrec_load_next_pc(struct regcache *cache, jit_state_t *_jit, u8 reg);
void lightrec_load_next_pc_imm(struct regcache *cache,
			       jit_state_t *_jit, u32 pc, u32 imm);

s8 lightrec_get_reg_with_value(struct regcache *cache, intptr_t value);
void lightrec_temp_set_value(struct regcache *cache, u8 jit_reg, intptr_t value);
u8 lightrec_alloc_reg_temp_with_value(struct regcache *cache,
				      jit_state_t *_jit, intptr_t value);

u8 lightrec_get_reg_in_flags(struct regcache *cache, u8 jit_reg);
void lightrec_set_reg_out_flags(struct regcache *cache, u8 jit_reg, u8 flags);

void lightrec_regcache_reset(struct regcache *cache);
void lightrec_preload_pc(struct regcache *cache, jit_state_t *_jit);
void lightrec_preload_imm(struct regcache *cache, jit_state_t *_jit,
			  u8 jit_reg, u32 imm);

void lightrec_free_reg(struct regcache *cache, u8 jit_reg);
void lightrec_free_regs(struct regcache *cache);
void lightrec_clean_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg);
void lightrec_clean_regs(struct regcache *cache, jit_state_t *_jit);
void lightrec_unload_reg(struct regcache *cache, jit_state_t *_jit, u8 jit_reg);
void lightrec_storeback_regs(struct regcache *cache, jit_state_t *_jit);

/* Spill / reload guest values held in the caller-saved argument registers
 * around a call into C. No-ops where NUM_ARGREGS is 0. */
void lightrec_save_argregs(struct regcache *cache, jit_state_t *_jit);
void lightrec_restore_argregs(struct regcache *cache, jit_state_t *_jit);

extern const u8 lightrec_pin_guest[];

/* Emitted once per lightrec_execute, in the dispatcher's prologue/exit. */
void lightrec_regcache_load_pins(jit_state_t *_jit);
void lightrec_regcache_store_pins(jit_state_t *_jit);
/* Flush dirty pins to memory before a call into C, which reads guest
 * registers out of struct lightrec_registers - and may write them. */
void lightrec_clean_pins(struct regcache *cache, jit_state_t *_jit);
/* Debug: at block entry, verify every pin still matches its memory home.
 * Records the first divergence in state->dbg_pin_{pc,idx}. */
void lightrec_emit_pin_tripwire(struct regcache *cache, jit_state_t *_jit,
				u32 pc);

/* Tell lightning which registers lightrec owns, so its own scratch allocator
 * cannot hand them out. Must be called once per jit_state, before emitting. */
void lightrec_regcache_reserve(jit_state_t *_jit);

/* Pins only. The dispatcher needs this too: it loads the pins on entry and
 * stores them back on exit, so lightning must not use those registers as
 * scratch in between - it would write its scratch into the guest's gpr. */
void lightrec_regcache_reserve_pins(jit_state_t *_jit);
_Bool lightrec_has_dirty_regs(struct regcache *cache);

_Bool lightrec_reg_is_loaded(struct regcache *cache, u16 reg);
void lightrec_clean_reg_if_loaded(struct regcache *cache, jit_state_t *_jit,
				  u16 reg, _Bool unload);
void lightrec_discard_reg_if_loaded(struct regcache *cache, u16 reg);

u8 lightrec_alloc_reg_in_address(struct regcache *cache,
		jit_state_t *_jit, u16 reg, s16 offset);

struct native_register * lightrec_regcache_enter_branch(struct regcache *cache);
void lightrec_regcache_leave_branch(struct regcache *cache,
			struct native_register *regs);

struct regcache * lightrec_regcache_init(struct lightrec_state *state);
void lightrec_free_regcache(struct regcache *cache);

__cnst const char * lightrec_reg_name(u8 reg);

void lightrec_regcache_mark_live(struct regcache *cache, jit_state_t *_jit);

#endif /* __REGCACHE_H__ */
