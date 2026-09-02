/* SPDX-License-Identifier: LGPL-3.0-or-later */

#include "sh4asm.h"

#include <string.h>

void sh4asm_init(struct sh4asm *as, void *buffer, size_t size,
		 uintptr_t base)
{
	memset(as, 0, sizeof(*as));
	as->start = buffer;
	as->cursor = buffer;
	as->end = as->cursor + size;
	as->base = base;
}

size_t sh4asm_size(const struct sh4asm *as)
{
	return (size_t)(as->cursor - as->start);
}

bool sh4asm_ok(const struct sh4asm *as)
{
	return !as->failed;
}

static void sh4asm_record_literal(struct sh4asm *as, uint32_t value,
				  unsigned int reg)
{
	struct sh4asm_literal *literal;

	if (as->nr_literals == SH4ASM_MAX_LITERALS) {
		as->failed = true;
		return;
	}

	literal = &as->literals[as->nr_literals++];
	literal->value = value;
	literal->insn_offset = sh4asm_size(as);
	sh4asm_emit_ld_l_pc(as, 0, reg);
}

void sh4asm_emit_load_imm32(struct sh4asm *as, unsigned int reg,
			    uint32_t value)
{
	int32_t signed_value = (int32_t)value;

	/* These are lightning's cheap constant tiers, before its pool path. */
	if (signed_value >= -128 && signed_value <= 127) {
		sh4asm_emit_mov_imm(as, signed_value, reg);
	} else if (!(signed_value & 1) &&
		   signed_value >= -256 && signed_value <= 255) {
		sh4asm_emit_mov_imm(as, signed_value >> 1, reg);
		sh4asm_emit_shll(as, reg);
	} else if (!(signed_value & 3) &&
		   signed_value >= -512 && signed_value <= 511) {
		sh4asm_emit_mov_imm(as, signed_value >> 2, reg);
		sh4asm_emit_shll2(as, reg);
	} else if (!(signed_value & 0xff) &&
		   signed_value >= -32768 && signed_value <= 32767) {
		sh4asm_emit_mov_imm(as, signed_value >> 8, reg);
		sh4asm_emit_shll8(as, reg);
	} else if (!(signed_value & 0xffff) &&
		   signed_value >= -8388608 && signed_value <= 8388607) {
		sh4asm_emit_mov_imm(as, signed_value >> 16, reg);
		sh4asm_emit_shll16(as, reg);
	} else {
		sh4asm_record_literal(as, value, reg);
	}
}

size_t sh4asm_emit_cond_branch_fwd(struct sh4asm *as, bool bt)
{
	size_t off = sh4asm_size(as);

	if (bt)
		sh4asm_emit_bt(as, 0);
	else
		sh4asm_emit_bf(as, 0);

	return off;
}

static uint16_t sh4asm_read_word(const uint8_t *ptr)
{
	return (uint16_t)(ptr[0] | ((uint16_t)ptr[1] << 8));
}

static void sh4asm_write_word(uint8_t *ptr, uint16_t value)
{
	ptr[0] = (uint8_t)value;
	ptr[1] = (uint8_t)(value >> 8);
}

static int sh4asm_literal_index(const uint32_t *values, unsigned int nr_values,
				uint32_t value)
{
	unsigned int i;

	for (i = 0; i < nr_values; i++)
		if (values[i] == value)
			return (int)i;

	return -1;
}

void sh4asm_patch_cond_branch(struct sh4asm *as, size_t branch_off)
{
	long target = (long)sh4asm_size(as);
	long disp = (target - (long)branch_off - 4) / 2;
	uint8_t *insn = as->start + branch_off;
	uint16_t word;

	/* bt/bf carry a signed 8-bit word displacement from PC+4. */
	if (disp < -128 || disp > 127) {
		as->failed = true;
		return;
	}

	word = sh4asm_read_word(insn);
	sh4asm_write_word(insn, (uint16_t)((word & 0xff00) | ((uint16_t)disp & 0xff)));
}

bool sh4asm_finalize(struct sh4asm *as)
{
	uint32_t values[SH4ASM_MAX_LITERALS];
	unsigned int nr_values = 0;
	uintptr_t pool_address;
	unsigned int i;

	if (as->finalized)
		return !as->failed;

	if (as->nr_literals && ((as->base + sh4asm_size(as)) & 2))
		sh4asm_emit_nop(as);

	pool_address = as->base + sh4asm_size(as);
	for (i = 0; i < as->nr_literals; i++) {
		uint32_t value = as->literals[i].value;

		if (sh4asm_literal_index(values, nr_values, value) < 0)
			values[nr_values++] = value;
	}

	for (i = 0; i < nr_values; i++) {
		sh4asm_emit_word(as, (uint16_t)values[i]);
		sh4asm_emit_word(as, (uint16_t)(values[i] >> 16));
	}

	for (i = 0; i < as->nr_literals && !as->failed; i++) {
		const struct sh4asm_literal *literal = &as->literals[i];
		uintptr_t insn_address = as->base + literal->insn_offset;
		uintptr_t reference = (insn_address + 4) & ~(uintptr_t)3;
		int index = sh4asm_literal_index(values, nr_values,
						 literal->value);
		uintptr_t value_address = pool_address + (uintptr_t)index * 4;
		uintptr_t displacement;
		uint8_t *insn;
		uint16_t word;

		if (value_address < reference) {
			as->failed = true;
			break;
		}

		displacement = (value_address - reference) >> 2;
		if (displacement > 0xff) {
			as->failed = true;
			break;
		}

		insn = as->start + literal->insn_offset;
		word = sh4asm_read_word(insn);
		sh4asm_write_word(insn,
				  (uint16_t)(word | (uint16_t)displacement));
	}

	as->finalized = true;
	return !as->failed;
}
