/* SPDX-License-Identifier: LGPL-2.1-or-later */

#include "sh4asm.h"
#include "sh4template.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static uint16_t read_word(const uint8_t *buffer, size_t index)
{
	return (uint16_t)(buffer[index * 2] |
			((uint16_t)buffer[index * 2 + 1] << 8));
}

static void fail(const char *test, size_t index, uint16_t expected,
		 uint16_t actual)
{
	fprintf(stderr, "%s: word %zu: expected %04x, got %04x\n",
		test, index, expected, actual);
	exit(EXIT_FAILURE);
}

static void require(const char *test, bool condition, const char *message)
{
	if (!condition) {
		fprintf(stderr, "%s: %s\n", test, message);
		exit(EXIT_FAILURE);
	}
}

static void expect_words(const char *test, const uint8_t *buffer,
			 const uint16_t *expected, size_t nr_words)
{
	size_t i;

	for (i = 0; i < nr_words; i++) {
		uint16_t actual = read_word(buffer, i);

		if (actual != expected[i])
			fail(test, i, expected[i], actual);
	}
}

static struct opcode alu_imm(unsigned int code, unsigned int rs,
			     unsigned int rt, uint16_t imm, uint32_t flags)
{
	struct opcode op;

	memset(&op, 0, sizeof(op));
	op.i.op = code;
	op.i.rs = rs;
	op.i.rt = rt;
	op.i.imm = imm;
	op.flags = flags;
	return op;
}

static struct opcode alu_reg(unsigned int code, unsigned int rs,
			     unsigned int rt, unsigned int rd)
{
	struct opcode op;

	memset(&op, 0, sizeof(op));
	op.i.op = OP_SPECIAL;
	op.r.op = code;
	op.r.rs = rs;
	op.r.rt = rt;
	op.r.rd = rd;
	return op;
}

static void test_encoder_words(void)
{
	static const uint16_t expected[] = {
		0x6c33, /* mov r3,r12 */
		0x387c, /* add r7,r8 */
		0xc611, /* mov.l @(17*4,gbr),r0 */
		0x727f, /* add #127,r2 */
		0x30c6, /* cmp/hi r12,r0 */
		0xf9fd, /* ftrv xmtrx,fv8 */
	};
	uint8_t buffer[sizeof(expected)];
	struct sh4asm as;

	sh4asm_init(&as, buffer, sizeof(buffer), 0x1000);
	sh4asm_emit_mov(&as, SH4ASM_R3, SH4ASM_R12);
	sh4asm_emit_add(&as, SH4ASM_R7, SH4ASM_R8);
	sh4asm_emit_ld_l_gbr(&as, 17);
	sh4asm_emit_add_imm(&as, 127, SH4ASM_R2);
	sh4asm_emit_cmphi(&as, SH4ASM_R12, SH4ASM_R0);
	sh4asm_emit_ftrv(&as, 2);

	require(__func__, sh4asm_ok(&as), "assembler failed");
	require(__func__, sh4asm_size(&as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_literal_pool(void)
{
	static const uint16_t expected[] = {
		0xd200, 0xd300, 0x5678, 0x1234,
	};
	uint8_t buffer[32];
	struct sh4asm as;

	sh4asm_init(&as, buffer, sizeof(buffer), 0x1000);
	sh4asm_emit_load_imm32(&as, SH4ASM_R2, 0x12345678);
	sh4asm_emit_load_imm32(&as, SH4ASM_R3, 0x12345678);
	require(__func__, sh4asm_finalize(&as), "finalization failed");
	require(__func__, sh4asm_size(&as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_addiu_pinned(void)
{
	static const uint16_t expected[] = { 0x7705 };
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode op = alu_imm(OP_ADDIU, 2, 2, 5, 0);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require(__func__, lightrec_sh4t_emit_alu_imm(&t, &op), "not handled");
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_andi_pinned(void)
{
	static const uint16_t expected[] = { 0x697c };
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode op = alu_imm(OP_ANDI, 2, 4, 0xff, 0);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	lightrec_sh4t_emit_alu_imm(&t, &op);
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_sltiu_literal(void)
{
	static const uint16_t expected[] = {
		0xd001, 0x30c6, 0x0b29, 0x0009, 0x7fff, 0x0000,
	};
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode op = alu_imm(OP_SLTIU, 6, 1, 0x7fff, 0);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	lightrec_sh4t_emit_alu_imm(&t, &op);
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_addiu_memory_backed(void)
{
	static const uint16_t expected[] = {
		0xc60a, /* gpr[9] -> r0 (gpr array starts at byte 4) */
		0x6303, /* r0 -> scratch r3 */
		0x73ff, /* add #-1,r3 */
		0x6033, /* r3 -> r0 */
		0xc209, /* r0 -> gpr[8] */
	};
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode op = alu_imm(OP_ADDIU, 9, 8, 0xffff, 0);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	lightrec_sh4t_emit_alu_imm(&t, &op);
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_dispatch_exit(void)
{
	static const uint16_t expected[] = {
		0x71fb, /* cycles -= 5 */
		0xe420, /* next PC -> r4 */
		0xe540, /* LUT entry -> r5 */
		0xc640, /* state->fast_eob -> r0 */
		0x402b, /* jmp @r0 */
		0x0009, /* delay-slot nop */
	};
	uint8_t buffer[32];
	struct lightrec_sh4t t;

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	lightrec_sh4t_emit_dispatch_exit(&t, 5, 0x20, 0x40, 0x100);
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_alu_reg_aliases(void)
{
	static const uint16_t add_expected[] = {
		0x379c, /* rd(v0)=rt(v0): add rs(a0),r7 */
	};
	static const uint16_t sub_expected[] = {
		0x6393, /* preserve rt(v0): rs(a0) -> scratch r3 */
		0x3378, /* r3 -= r7 */
		0x6733, /* scratch -> rd(v0) */
	};
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode add = alu_reg(OP_SPECIAL_ADDU, 4, 2, 2);
	struct opcode sub = alu_reg(OP_SPECIAL_SUBU, 4, 2, 2);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require(__func__, lightrec_sh4t_emit_alu_reg(&t, &add), "ADD rejected");
	require(__func__, sh4asm_finalize(&t.as), "ADD finalization failed");
	expect_words(__func__, buffer, add_expected, ARRAY_SIZE(add_expected));

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require(__func__, lightrec_sh4t_emit_alu_reg(&t, &sub), "SUB rejected");
	require(__func__, sh4asm_finalize(&t.as), "SUB finalization failed");
	expect_words(__func__, buffer, sub_expected, ARRAY_SIZE(sub_expected));
}

static void emit_one_reg(struct lightrec_sh4t *t, const struct opcode *op)
{
	require("shift", lightrec_sh4t_emit_alu_reg(t, op), "op rejected");
	require("shift", sh4asm_finalize(&t->as), "finalization failed");
}

static void test_shifts(void)
{
	uint8_t buffer[64];
	struct lightrec_sh4t t;

	/* SLL v0, a0, 3: pinned dest r7, pinned src r9, constant amount. */
	static const uint16_t sll_c[] = { 0x6793, 0xe003, 0x470d };
	struct opcode sll = alu_reg(OP_SPECIAL_SLL, 0, 4, 2);
	sll.r.imm = 3;
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	emit_one_reg(&t, &sll);
	expect_words("sll_const", buffer, sll_c, ARRAY_SIZE(sll_c));

	/* SRA t4, a1, 1: memory-backed dest via r3, pinned src r10, arith. */
	static const uint16_t sra_c[] = { 0x63a3, 0xe0ff, 0x430c, 0x6033, 0xc20d };
	struct opcode sra = alu_reg(OP_SPECIAL_SRA, 0, 5, 12);
	sra.r.imm = 1;
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	emit_one_reg(&t, &sra);
	expect_words("sra_const", buffer, sra_c, ARRAY_SIZE(sra_c));

	/* SLLV v0, v1, t0: pinned dest/value, memory-backed count masked to 5b. */
	static const uint16_t sllv_v[] = { 0x6783, 0xc609, 0xc91f, 0x470d };
	struct opcode sllv = alu_reg(OP_SPECIAL_SLLV, 8, 3, 2);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	emit_one_reg(&t, &sllv);
	expect_words("sllv_var", buffer, sllv_v, ARRAY_SIZE(sllv_v));

	/* SLLV a0, v1, a0: rd==rs alias (both pin r9) - count stashed in r3. */
	static const uint16_t sllv_alias[] = {
		0x6093, 0xc91f, 0x6303, 0x6983, 0x493d,
	};
	struct opcode alias = alu_reg(OP_SPECIAL_SLLV, 4, 3, 4);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	emit_one_reg(&t, &alias);
	expect_words("sllv_alias", buffer, sllv_alias, ARRAY_SIZE(sllv_alias));
}

static void test_loads(void)
{
	uint8_t buffer[64];
	struct lightrec_sh4t t;

	/* LW v0, 0(a0): pinned dest r7, pinned base r9, no mask, base 0. */
	static const uint16_t lw[] = { 0x6793, 0x6772 };
	struct opcode op_lw = alu_imm(OP_LW, 4, 2, 0, 0);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require("lw", lightrec_sh4t_emit_load(&t, &op_lw, 0, false, 0), "rejected");
	require("lw", sh4asm_finalize(&t.as), "finalization failed");
	expect_words("lw", buffer, lw, ARRAY_SIZE(lw));

	/* LBU v0, 0(a0): mov.b sign-loads, extu.b zero-extends. */
	static const uint16_t lbu[] = { 0x6793, 0x6770, 0x677c };
	struct opcode op_lbu = alu_imm(OP_LBU, 4, 2, 0, 0);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require("lbu", lightrec_sh4t_emit_load(&t, &op_lbu, 0, false, 0), "rejected");
	require("lbu", sh4asm_finalize(&t.as), "finalization failed");
	expect_words("lbu", buffer, lbu, ARRAY_SIZE(lbu));

	/* LH t0, 0(a0): memory-backed dest via r3, signed mov.w, store to gbr. */
	static const uint16_t lh[] = { 0x6393, 0x6331, 0x6033, 0xc209 };
	struct opcode op_lh = alu_imm(OP_LH, 4, 8, 0, 0);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require("lh", lightrec_sh4t_emit_load(&t, &op_lh, 0, false, 0), "rejected");
	require("lh", sh4asm_finalize(&t.as), "finalization failed");
	expect_words("lh", buffer, lh, ARRAY_SIZE(lh));
}

static void test_stores(void)
{
	uint8_t buffer[64];
	struct lightrec_sh4t t;

	/* SW v0, 0(a0): addr built in r3 from a0(r9); value is the pin r7. */
	static const uint16_t sw[] = { 0x6393, 0x2372 };
	struct opcode op_sw = alu_imm(OP_SW, 4, 2, 0, 0);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require("sw", lightrec_sh4t_emit_store(&t, &op_sw, 0, false, 0), "rejected");
	require("sw", sh4asm_finalize(&t.as), "finalization failed");
	expect_words("sw", buffer, sw, ARRAY_SIZE(sw));

	/* SB t0, 0(a0): memory-backed value loaded to r0, low byte stored. */
	static const uint16_t sb[] = { 0x6393, 0xc609, 0x2300 };
	struct opcode op_sb = alu_imm(OP_SB, 4, 8, 0, 0);
	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	require("sb", lightrec_sh4t_emit_store(&t, &op_sb, 0, false, 0), "rejected");
	require("sb", sh4asm_finalize(&t.as), "finalization failed");
	expect_words("sb", buffer, sb, ARRAY_SIZE(sb));
}

static void test_whole_block_gate(void)
{
	struct opcode supported[5] = {
		alu_imm(OP_LUI, 0, 2, 0x1234, LIGHTREC_MOVI),
		alu_imm(OP_ORI, 2, 2, 0x5678, LIGHTREC_MOVI),
		alu_reg(OP_SPECIAL_ADDU, 2, 4, 2),
		{ .opcode = 0 },
		{ .i = { .op = OP_J, .imm = 0x800 }, .flags = LIGHTREC_NO_DS },
	};
	struct opcode mixed[ARRAY_SIZE(supported)];

	require(__func__,
		lightrec_sh4t_can_emit_block(supported,
			ARRAY_SIZE(supported), 0x1000, false),
		"supported block rejected");

	memcpy(mixed, supported, sizeof(mixed));
	mixed[1] = alu_imm(OP_LWL, 2, 2, 0, 0);	/* unaligned load: unsupported */
	require(__func__,
		!lightrec_sh4t_can_emit_block(mixed,
			ARRAY_SIZE(mixed), 0x1000, false),
		"mixed block did not fall back");

	/* Conditional branch: BNE t0,zero with a safe delay slot, taken target
	 * external (pc+48), fall-through off the block end. */
	struct opcode cond[3] = {
		alu_imm(OP_ADDIU, 8, 8, 1, 0),		/* t0 += 1        */
		alu_imm(OP_BNE, 8, 0, 10, 0),		/* bne t0,zero,+  */
		alu_reg(OP_SPECIAL_ADDU, 2, 8, 2),	/* ds: v0 += t0   */
	};
	require(__func__,
		lightrec_sh4t_can_emit_block(cond, ARRAY_SIZE(cond),
			0x1000, false),
		"conditional block rejected");

	/* A delay slot that clobbers the compared register must fall back. */
	cond[2] = alu_imm(OP_ADDIU, 8, 8, 1, 0);	/* ds writes t0   */
	require(__func__,
		!lightrec_sh4t_can_emit_block(cond, ARRAY_SIZE(cond),
			0x1000, false),
		"delay slot clobbering branch input did not fall back");

	/* Backward branch (loop) must fall back for now. */
	struct opcode loop[3] = {
		alu_imm(OP_ADDIU, 8, 8, 1, 0),
		alu_imm(OP_BNE, 8, 0, (uint16_t)-2, 0),	/* back to op 0   */
		{ .opcode = 0 },
	};
	require(__func__,
		!lightrec_sh4t_can_emit_block(loop, ARRAY_SIZE(loop),
			0x1000, false),
		"backward branch did not fall back");
}

static void test_fused_lui_ori(void)
{
	static const uint16_t expected[] = {
		0xd700, 0x0009, 0x5678, 0x1234,
	};
	uint8_t buffer[32];
	struct lightrec_sh4t t;
	struct opcode lui = alu_imm(OP_LUI, 0, 2, 0x1234, LIGHTREC_MOVI);
	struct opcode ori = alu_imm(OP_ORI, 2, 2, 0x5678, LIGHTREC_MOVI);

	lightrec_sh4t_init(&t, buffer, sizeof(buffer), 0x1000, 4);
	lightrec_sh4t_emit_alu_imm(&t, &lui);
	lightrec_sh4t_emit_alu_imm(&t, &ori);
	require(__func__, sh4asm_finalize(&t.as), "finalization failed");
	require(__func__, sh4asm_size(&t.as) == sizeof(expected), "wrong size");
	expect_words(__func__, buffer, expected, ARRAY_SIZE(expected));
}

static void test_overflow(void)
{
	uint8_t buffer[1];
	struct sh4asm as;

	sh4asm_init(&as, buffer, sizeof(buffer), 0);
	sh4asm_emit_nop(&as);
	require(__func__, !sh4asm_ok(&as), "overflow was not reported");
	require(__func__, !sh4asm_finalize(&as), "overflow finalized successfully");
}

int main(void)
{
	test_encoder_words();
	test_literal_pool();
	test_addiu_pinned();
	test_andi_pinned();
	test_sltiu_literal();
	test_addiu_memory_backed();
	test_dispatch_exit();
	test_alu_reg_aliases();
	test_shifts();
	test_loads();
	test_stores();
	test_whole_block_gate();
	test_fused_lui_ori();
	test_overflow();
	puts("sh4asm: all tests passed");
	return EXIT_SUCCESS;
}
