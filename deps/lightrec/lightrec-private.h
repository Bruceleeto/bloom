/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2016-2021 Paul Cercueil <paul@crapouillou.net>
 */

#ifndef __LIGHTREC_PRIVATE_H__
#define __LIGHTREC_PRIVATE_H__

#include "lightrec-config.h"
#include "disassembler.h"
#include "lightrec.h"

#if ENABLE_THREADED_COMPILER
#include <stdatomic.h>
#endif

#ifdef _MSC_BUILD
#include <immintrin.h>
#endif

#include <inttypes.h>
#include <stdint.h>

#define X32_FMT "0x%08"PRIx32
#define PC_FMT "PC "X32_FMT

#define ARRAY_SIZE(x) (sizeof(x) ? sizeof(x) / sizeof((x)[0]) : 0)

/* WHAT GNU LIGHTNING USED TO SUPPLY.
 *
 * `__WORDSIZE` is a glibc symbol, out of <bits/wordsize.h>, and it reached
 * this header only because <lightning.h> pulled it in. Nothing includes that
 * any more and KOS is newlib, which does not define it -- so the three
 * remaining uses would silently be errors on the one target that matters.
 * GCC knows the answer without being told. */
#ifndef __WORDSIZE
#define __WORDSIZE (__SIZEOF_POINTER__ * 8)
#endif

#define GENMASK(h, l) \
	(((uintptr_t)-1 << (l)) & ((uintptr_t)-1 >> (__WORDSIZE - 1 - (h))))

#ifdef __GNUC__
#	define likely(x)       __builtin_expect(!!(x),1)
#	define unlikely(x)     __builtin_expect(!!(x),0)
#else
#	define likely(x)       (x)
#	define unlikely(x)     (x)
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#	define LE32TOH(x)	__builtin_bswap32(x)
#	define HTOLE32(x)	__builtin_bswap32(x)
#	define LE16TOH(x)	__builtin_bswap16(x)
#	define HTOLE16(x)	__builtin_bswap16(x)
#else
#	define LE32TOH(x)	(x)
#	define HTOLE32(x)	(x)
#	define LE16TOH(x)	(x)
#	define HTOLE16(x)	(x)
#endif

#if HAS_DEFAULT_ELM
#define SET_DEFAULT_ELM(table, value) [0 ... ARRAY_SIZE(table) - 1] = value
#else
#define SET_DEFAULT_ELM(table, value) [0] = NULL
#endif

#if __has_attribute(__fallthrough__)
#	define fallthrough	__attribute__((__fallthrough__))
#else
#	define fallthrough	do {} while (0)  /* fallthrough */
#endif

#define container_of(ptr, type, member) \
	((type *)((void *)(ptr) - offsetof(type, member)))

#ifdef _MSC_BUILD
#	define popcount32(x)	__popcnt(x)
#	define clz32(x)		_lzcnt_u32(x)
#	define ctz32(x)		_tzcnt_u32(x)
#else
#	define popcount32(x)	__builtin_popcount(x)
#	define clz32(x)		__builtin_clz(x)
#	define ctz32(x)		__builtin_ctz(x)
#endif

/* Flags for (struct block *)->flags */
#define BLOCK_NEVER_COMPILE	BIT(0)
#define BLOCK_SHOULD_RECOMPILE	BIT(1)
#define BLOCK_FULLY_TAGGED	BIT(2)
#define BLOCK_IS_DEAD		BIT(3)
#define BLOCK_IS_MEMSET		BIT(4)
#define BLOCK_NO_OPCODE_LIST	BIT(5)
#define BLOCK_PRELOAD_PC	BIT(6)

#define RAM_SIZE	0x200000
#define BIOS_SIZE	0x80000

#define CODE_LUT_SIZE	((RAM_SIZE + BIOS_SIZE) >> 2)

#define REG_LO 32
#define REG_HI 33
#define REG_TEMP (offsetof(struct lightrec_state, temp_reg) / sizeof(u32))

/* One precomputed product per possible block length: cycle_table[k] is
 * k * cycles_per_op.  A block's charge is then a load whose displacement is
 * its instruction count.  Sized for the longest block fgl will emit plus the
 * inclusive end -- kept in step with FGL_CYCLE_ENTRIES. */
#define LIGHTREC_CYCLE_ENTRIES 34

struct blockcache;
struct recompiler;
struct opcode;
struct reaper;

struct u16x2 {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	u16 h, l;
#else
	u16 l, h;
#endif
};

struct block {
	struct opcode *opcode_list;
	void (*function)(void);
	const u32 *code;
	struct block *next;
	u32 pc;
	u32 hash;
	u32 precompile_date;
	unsigned int code_size;
	u16 nb_ops;
#if ENABLE_THREADED_COMPILER
	_Atomic u8 flags;
#else
	u8 flags;
#endif
};

/* KEPT, AND EMPTY, AND THAT IS THE POINT.
 *
 * These described a branch inside a block waiting to be patched to a label
 * inside the same block. fgl ends a block at its first control transfer, so no
 * branch is ever internal and nothing fills either array -- but the arrays
 * still sit in `struct lightrec_cstate` and the compile path still resets the
 * counts, so that restoring intra-block branches later is a change to fgl and
 * not a change to lightrec's bookkeeping. `label` and `branch` were Lightning
 * node pointers; a code address is what fgl would put there. */
struct lightrec_branch {
	void *branch;
	u32 target;
};

struct lightrec_branch_target {
	void *label;
	u32 offset;
};

struct lightrec_cstate {
	struct lightrec_state *state;

	struct lightrec_branch local_branches[512];
	struct lightrec_branch_target targets[512];
	u16 movi_temp[32];
	unsigned int nb_local_branches;
	unsigned int nb_targets;
	unsigned int cycles;


	_Bool no_load_delay;
};

/* THE ORDER OF THE FIRST FIELDS IS THE CODE GENERATOR'S ABI, NOT A STYLE
 * CHOICE.  GBR points at this struct and generated code reaches every one of
 * them with a single `mov.l @(disp,GBR),r0`, whose displacement is a compile
 * time constant baked into the instruction word.  Those constants are
 * declared in src/fgl/fgl_state.h and checked against this struct by the
 * static assertions in src/fgl/fgl_lightrec.c.  Moving a field here without
 * moving it there does not fail to build and does not crash: generated code
 * reads the neighbouring field and the machine desynchronises quietly.
 *
 * `dispatch` is the host address a finished block jumps to.  It is a u32 and
 * not a `void *` on purpose: every field above it is four bytes wide, so the
 * struct has the same layout on this machine and on the workstation that
 * compiles the layout assertions, and the assertions therefore mean something.
 *
 * `cycle_table` is the reason a block charges its cycles in two instructions
 * with no literal pool entry -- see fgl_state.h.  It has to sit immediately
 * behind next_pc so that its displacement does not depend on anything below
 * it, and the three counters follow it for the same reason. */
struct lightrec_state {
	struct lightrec_registers regs;		/* +0   */
	u32 temp_reg;				/* +520 */
	u32 curr_pc;				/* +524 */
	u32 next_pc;				/* +528 */
	u32 cycle_table[LIGHTREC_CYCLE_ENTRIES];/* +532 */
	u32 current_cycle;			/* +668 */
	u32 target_cycle;			/* +672 */
	u32 exit_flags;				/* +676 */
	u32 dispatch;				/* +680 */
	u32 lut_base;				/* +684 */
	u32 addr_mask;				/* +688 */
	u32 shim_arg;				/* +692 */
	u8 in_delay_slot_n;
	u32 old_cycle_counter;
	u32 cycles_per_op;
	struct blockcache *block_cache;
	struct recompiler *rec;
	struct lightrec_cstate *cstate;
	struct reaper *reaper;
	void *tlsf;
	void (*interpreter_func)(void);
	void (*ds_check_func)(void);
	void (*memset_func)(void);
	void (*get_next_block)(void);
	struct lightrec_ops ops;
	unsigned int nb_precompile;
	unsigned int nb_compile;
	unsigned int nb_maps;
	const struct lightrec_mem_map *maps;
	uintptr_t offset_ram, offset_bios, offset_scratch, offset_io;
	u32 opt_flags;
	_Bool with_32bit_lut;
	_Bool mirrors_mapped;
	void *code_lut[];
};

#define lightrec_offset(ptr) \
	offsetof(struct lightrec_state, ptr)

u32 lightrec_rw(struct lightrec_state *state, union code op, u32 addr,
		u32 data, u32 *flags, struct block *block, u16 offset);

/* Direct-call device access for HW-tagged plain loads/stores: called bare
 * from generated code with the cycle contract emitted inline - no wrapper
 * block, no lightrec_rw, no lightrec_get_map. Loads return the value
 * already sign/zero-extended. */
u32 lightrec_hw_lb(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lbu(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lh(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lhu(u32 addr, struct lightrec_state *state);
u32 lightrec_hw_lw(u32 addr, struct lightrec_state *state);
void lightrec_hw_sb(u32 addr, u32 val, struct lightrec_state *state);
void lightrec_hw_sh(u32 addr, u32 val, struct lightrec_state *state);
void lightrec_hw_sw(u32 addr, u32 val, struct lightrec_state *state);

void lightrec_free_block(struct lightrec_state *state, struct block *block);

void remove_from_code_lut(struct blockcache *cache, struct block *block);

const struct lightrec_mem_map *
lightrec_get_map(struct lightrec_state *state, void **host, u32 kaddr);

static inline u32 kunseg(u32 addr)
{
	if (unlikely(addr >= 0xa0000000))
		return addr - 0xa0000000;
	else
		return addr &~ 0x80000000;
}

static inline u32 lut_offset(u32 pc)
{
	if (pc & BIT(28))
		return ((pc & (BIOS_SIZE - 1)) + RAM_SIZE) >> 2; // BIOS
	else
		return (pc & (RAM_SIZE - 1)) >> 2; // RAM
}

static inline _Bool is_big_endian(void)
{
	return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
}

static inline _Bool lut_is_32bit(const struct lightrec_state *state)
{
	return __WORDSIZE == 32 ||
		(ENABLE_CODE_BUFFER && state->with_32bit_lut);
}

static inline size_t lut_elm_size(const struct lightrec_state *state)
{
	return lut_is_32bit(state) ? 4 : sizeof(void *);
}

static inline void ** lut_address(struct lightrec_state *state, u32 offset)
{
	if (lut_is_32bit(state))
		return (void **) ((uintptr_t) state->code_lut + offset * 4);
	else
		return &state->code_lut[offset];
}

static inline void * lut_read(struct lightrec_state *state, u32 offset)
{
	void **lut_entry = lut_address(state, offset);

	if (lut_is_32bit(state))
		return (void *)(uintptr_t) *(u32 *) lut_entry;
	else
		return *lut_entry;
}

static inline void lut_write(struct lightrec_state *state, u32 offset, void *ptr)
{
	void **lut_entry = lut_address(state, offset);

	if (lut_is_32bit(state))
		*(u32 *) lut_entry = (u32)(uintptr_t) ptr;
	else
		*lut_entry = ptr;
}

static inline u32 get_ds_pc(const struct block *block, u16 offset, s16 imm)
{
	u16 flags = block->opcode_list[offset].flags;

	offset += op_flag_no_ds(flags);

	return block->pc + ((offset + imm) << 2);
}

static inline u32 get_branch_pc(const struct block *block, u16 offset, s16 imm)
{
	u16 flags = block->opcode_list[offset].flags;

	offset -= op_flag_no_ds(flags);

	return block->pc + ((offset + imm) << 2);
}

void lightrec_mtc(struct lightrec_state *state, union code op, u8 reg, u32 data);
u32 lightrec_mfc(struct lightrec_state *state, union code op);
void lightrec_rfe(struct lightrec_state *state);
void lightrec_cp(struct lightrec_state *state, union code op);

struct lightrec_cstate * lightrec_create_cstate(struct lightrec_state *state);
void lightrec_free_cstate(struct lightrec_cstate *cstate);

union code lightrec_read_opcode(struct lightrec_state *state, u32 pc);

int lightrec_compile_block(struct lightrec_cstate *cstate, struct block *block);
void lightrec_free_opcode_list(struct lightrec_state *state,
			       struct opcode *list);

unsigned int lightrec_cycles_of_opcode(const struct lightrec_state *state,
				       union code code);

static inline u8 get_mult_div_lo(union code c)
{
	return (OPT_FLAG_MULT_DIV && c.r.rd) ? c.r.rd : REG_LO;
}

static inline u8 get_mult_div_hi(union code c)
{
	return (OPT_FLAG_MULT_DIV && c.r.imm) ? c.r.imm : REG_HI;
}

static inline s16 s16_max(s16 a, s16 b)
{
	return a > b ? a : b;
}

static inline _Bool block_has_flag(struct block *block, u8 flag)
{
#if ENABLE_THREADED_COMPILER
	return atomic_load_explicit(&block->flags, memory_order_relaxed) & flag;
#else
	return block->flags & flag;
#endif
}

static inline u8 block_set_flags(struct block *block, u8 mask)
{
#if ENABLE_THREADED_COMPILER
	return atomic_fetch_or_explicit(&block->flags, mask,
					memory_order_relaxed);
#else
	u8 flags = block->flags;

	block->flags |= mask;

	return flags;
#endif
}

static inline u8 block_clear_flags(struct block *block, u8 mask)
{
#if ENABLE_THREADED_COMPILER
	return atomic_fetch_and_explicit(&block->flags, ~mask,
					 memory_order_relaxed);
#else
	u8 flags = block->flags;

	block->flags &= ~mask;

	return flags;
#endif
}

static inline _Bool can_sign_extend(s32 value, u8 order)
{
      return ((u32)(value >> (order - 1)) + 1) < 2;
}

static inline _Bool can_zero_extend(u32 value, u8 order)
{
      return (value >> order) == 0;
}

static inline _Bool is_low_mask(u32 imm)
{
	return imm & 1 ? popcount32(imm + 1) <= 1 : 0;
}

static inline _Bool is_high_mask(u32 imm)
{
	return imm ? popcount32(imm + BIT(ctz32(imm))) == 0 : 0;
}

static inline const struct opcode *
get_delay_slot(const struct opcode *list, u16 i)
{
	return op_flag_no_ds(list[i].flags) ? &list[i - 1] : &list[i + 1];
}

/* Whether a block hands its next PC over through memory rather than in a
 * register.  Under GNU Lightning this was a register-pressure question and
 * the answer varied by architecture.  fgl does not have the choice and does
 * not want it: the block's `rts` needs a delay slot filled, and the store of
 * next_pc fills it, so publishing the PC costs nothing and the dispatcher
 * reads it from the state block.  Always. */
static inline _Bool lightrec_store_next_pc(void)
{
	return 1;
}

/* ------------------------------------------------------------------ */
/* fgl                                                                 */
/* ------------------------------------------------------------------ */

/* THE WHOLE OF THE BOUNDARY, IN ONE PLACE.
 *
 * Above this line is lightrec as it always was, minus a code generator. Below
 * it is everything the two halves say to each other: what lightrec calls to
 * compile a block, what fgl's hand-written assembly calls back into C, and the
 * three entry points in `dispatch.S` that are stored in this struct as plain
 * addresses because they are no longer generated.
 */

/* Compile one block. Takes lightrec's already-optimised opcode list and
 * returns the block's entry point, or NULL if fgl met something it cannot
 * lower -- which is a hole to fill and never a case to route around, there
 * being no second code generator to fall back to.
 *
 * On failure `*why` says WHICH failure, because the two are opposites and
 * the caller's response to one is a loop under the other:
 *
 *   -ENOMEM   the code arena is full.  Transient.  The caller may flush the
 *             block cache and try again, and that is what lightrec does.
 *   -EINVAL   fgl declined to lower this block.  Permanent, for these exact
 *             opcodes: retrying compiles the same block and refuses again,
 *             and answering -ENOMEM here flushes the entire cache on every
 *             attempt, forever, at the same PC.  That is not a theory; it is
 *             what the first fgl boot did.
 *
 * `why` may be NULL if the caller does not care. */
void *fgl_compile_block(struct lightrec_cstate *cstate, struct block *block,
			unsigned int *code_size, int *why);

/* The code arena, shared with fgl because it places its own blocks. */
void *lightrec_alloc_code(struct lightrec_state *state, size_t size);
void lightrec_free_code(struct lightrec_state *state, void *ptr);

/* The dispatcher: hand-written SH-4 (src/fgl/dispatch.S), not generated.
 * `fgl_dispatch` is the way in from C and has the signature lightrec's
 * generated dispatcher had; the rest are addresses stored in the state block
 * or in the code table, and are entered by a jump with no return address. */
u32 fgl_dispatch(struct lightrec_state *state, u32 pc, void *first_block,
		 s32 cycle_delta);
void fgl_dispatch_loop(void);
void fgl_dispatch_compile(void);
void fgl_dispatch_memset(void);
void fgl_dispatch_interpreter(void);
void fgl_dispatch_ds_check(void);

/* What that assembly calls back into. The last three exist because lightrec's
 * own functions are static; see the comment on them in lightrec.c. */
void *fgl_get_next_block(struct lightrec_state *state, u32 pc);
u32 fgl_memset(struct lightrec_state *state);
u32 fgl_emulate_block(struct lightrec_state *state, struct block *block, u32 pc);
u32 fgl_check_load_delay(struct lightrec_state *state, u32 pc, u8 reg);

/* An access whose region the optimiser could not prove, performed entirely in
 * C against the state block. Reached from generated code through
 * `fgl_shim_call`, so it wears that shim's two-argument shape. */
void fgl_rw(u32 opcode, struct lightrec_state *state);
void fgl_mtc(u32 opcode, struct lightrec_state *state);

#endif /* __LIGHTREC_PRIVATE_H__ */
