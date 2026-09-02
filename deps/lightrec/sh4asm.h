/* SPDX-License-Identifier: LGPL-3.0-or-later */
/*
 * Flat SH-4 instruction encoder for lightrec's template backend.
 *
 * The instruction words are salvaged from GNU lightning's SH backend.  This
 * layer deliberately contains none of lightning's IR, allocation, or
 * scheduling machinery: every sh4asm_emit_* function appends exactly one
 * 16-bit target-endian instruction word.
 */

#ifndef __LIGHTREC_SH4ASM_H__
#define __LIGHTREC_SH4ASM_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SH4ASM_MAX_LITERALS 64

enum sh4asm_reg {
	SH4ASM_R0,
	SH4ASM_R1,
	SH4ASM_R2,
	SH4ASM_R3,
	SH4ASM_R4,
	SH4ASM_R5,
	SH4ASM_R6,
	SH4ASM_R7,
	SH4ASM_R8,
	SH4ASM_R9,
	SH4ASM_R10,
	SH4ASM_R11,
	SH4ASM_R12,
	SH4ASM_R13,
	SH4ASM_R14,
	SH4ASM_R15,
};

struct sh4asm_literal {
	uint32_t value;
	size_t insn_offset;
};

struct sh4asm {
	uint8_t *start;
	uint8_t *cursor;
	uint8_t *end;
	uintptr_t base;
	struct sh4asm_literal literals[SH4ASM_MAX_LITERALS];
	unsigned int nr_literals;
	bool failed;
	bool finalized;
};

void sh4asm_init(struct sh4asm *as, void *buffer, size_t size,
		 uintptr_t base);
size_t sh4asm_size(const struct sh4asm *as);
bool sh4asm_ok(const struct sh4asm *as);

/* Multi-word policy helpers.  The encoders below remain one-word functions. */
void sh4asm_emit_load_imm32(struct sh4asm *as, unsigned int reg,
			    uint32_t value);
bool sh4asm_finalize(struct sh4asm *as);

/* Forward conditional branch: emit a bt (bt==true) or bf with a placeholder
 * displacement and return its byte offset; sh4asm_patch_cond_branch later sets
 * the displacement to the current cursor. Only forward (target > branch). */
size_t sh4asm_emit_cond_branch_fwd(struct sh4asm *as, bool bt);
void sh4asm_patch_cond_branch(struct sh4asm *as, size_t branch_off);

static inline void sh4asm_emit_word(struct sh4asm *as, uint16_t word)
{
	if (as->finalized || (size_t)(as->end - as->cursor) < sizeof(word)) {
		as->failed = true;
		return;
	}

	/* Generated SH-4 code is little-endian even in host-side tests. */
	as->cursor[0] = (uint8_t)word;
	as->cursor[1] = (uint8_t)(word >> 8);
	as->cursor += sizeof(word);
}

#define SH4ASM_N(op, n) \
	((uint16_t)((op) | (((n) & 0xf) << 8)))
#define SH4ASM_NM(op, n, m) \
	((uint16_t)((op) | (((n) & 0xf) << 8) | (((m) & 0xf) << 4)))
#define SH4ASM_NMD(op, n, m, d) \
	((uint16_t)((op) | (((n) & 0xf) << 8) | (((m) & 0xf) << 4) | ((d) & 0xf)))
#define SH4ASM_MD(op, m, d) \
	((uint16_t)((op) | (((m) & 0xf) << 4) | ((d) & 0xf)))
#define SH4ASM_D8(op, d) \
	((uint16_t)((op) | ((d) & 0xff)))
#define SH4ASM_ND8(op, n, d) \
	((uint16_t)((op) | (((n) & 0xf) << 8) | ((d) & 0xff)))
#define SH4ASM_D12(op, d) \
	((uint16_t)((op) | ((d) & 0xfff)))

/* No operands. */
static inline void sh4asm_emit_clrt(struct sh4asm *as)
{
	sh4asm_emit_word(as, 0x0008);
}

static inline void sh4asm_emit_nop(struct sh4asm *as)
{
	sh4asm_emit_word(as, 0x0009);
}

static inline void sh4asm_emit_rts(struct sh4asm *as)
{
	sh4asm_emit_word(as, 0x000b);
}

static inline void sh4asm_emit_sett(struct sh4asm *as)
{
	sh4asm_emit_word(as, 0x0018);
}

static inline void sh4asm_emit_div0u(struct sh4asm *as)
{
	sh4asm_emit_word(as, 0x0019);
}

/* Single-register integer and control instructions. */
static inline void sh4asm_emit_movt(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x0029, n));
}

static inline void sh4asm_emit_cmppz(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4011, n));
}

static inline void sh4asm_emit_cmppl(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4015, n));
}

static inline void sh4asm_emit_dt(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4010, n));
}

static inline void sh4asm_emit_shll(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4000, n));
}

static inline void sh4asm_emit_shlr(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4001, n));
}

static inline void sh4asm_emit_rotcl(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4024, n));
}

static inline void sh4asm_emit_rotcr(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4025, n));
}

static inline void sh4asm_emit_shll2(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4008, n));
}

static inline void sh4asm_emit_shlr2(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4009, n));
}

static inline void sh4asm_emit_shll8(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4018, n));
}

static inline void sh4asm_emit_shlr8(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4019, n));
}

static inline void sh4asm_emit_shll16(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4028, n));
}

static inline void sh4asm_emit_shlr16(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x4029, n));
}

static inline void sh4asm_emit_jmp(struct sh4asm *as, unsigned int m)
{
	sh4asm_emit_word(as, SH4ASM_N(0x402b, m));
}

static inline void sh4asm_emit_jsr(struct sh4asm *as, unsigned int m)
{
	sh4asm_emit_word(as, SH4ASM_N(0x400b, m));
}

static inline void sh4asm_emit_ldc_gbr(struct sh4asm *as, unsigned int m)
{
	sh4asm_emit_word(as, SH4ASM_N(0x401e, m));
}

static inline void sh4asm_emit_stc_gbr(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x0012, n));
}

static inline void sh4asm_emit_lds_pr(struct sh4asm *as, unsigned int m)
{
	sh4asm_emit_word(as, SH4ASM_N(0x402a, m));
}

static inline void sh4asm_emit_sts_pr(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_N(0x002a, n));
}

/* Two-register integer instructions. Source m comes before destination n. */
static inline void sh4asm_emit_mov(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x6003, n, m));
}

static inline void sh4asm_emit_add(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x300c, n, m));
}

static inline void sh4asm_emit_addc(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x300e, n, m));
}

static inline void sh4asm_emit_sub(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3008, n, m));
}

static inline void sh4asm_emit_subc(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x300a, n, m));
}

static inline void sh4asm_emit_and(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2009, n, m));
}

static inline void sh4asm_emit_or(struct sh4asm *as, unsigned int m,
			  unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x200b, n, m));
}

static inline void sh4asm_emit_xor(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x200a, n, m));
}

static inline void sh4asm_emit_tst(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2008, n, m));
}

static inline void sh4asm_emit_not(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x6007, n, m));
}

static inline void sh4asm_emit_neg(struct sh4asm *as, unsigned int m,
			   unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x600b, n, m));
}

static inline void sh4asm_emit_exts_b(struct sh4asm *as, unsigned int m,
			      unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x600e, n, m));
}

static inline void sh4asm_emit_exts_w(struct sh4asm *as, unsigned int m,
			      unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x600f, n, m));
}

static inline void sh4asm_emit_extu_b(struct sh4asm *as, unsigned int m,
			      unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x600c, n, m));
}

static inline void sh4asm_emit_extu_w(struct sh4asm *as, unsigned int m,
			      unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x600d, n, m));
}

static inline void sh4asm_emit_cmpeq(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3000, n, m));
}

static inline void sh4asm_emit_cmphs(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3002, n, m));
}

static inline void sh4asm_emit_cmpge(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3003, n, m));
}

static inline void sh4asm_emit_cmphi(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3006, n, m));
}

static inline void sh4asm_emit_cmpgt(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3007, n, m));
}

static inline void sh4asm_emit_div1(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3004, n, m));
}

static inline void sh4asm_emit_div0s(struct sh4asm *as, unsigned int m,
			     unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2007, n, m));
}

static inline void sh4asm_emit_dmuls_l(struct sh4asm *as, unsigned int m,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x300d, n, m));
}

static inline void sh4asm_emit_dmulu_l(struct sh4asm *as, unsigned int m,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x3005, n, m));
}

static inline void sh4asm_emit_shad(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x400c, n, m));
}

static inline void sh4asm_emit_shld(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x400d, n, m));
}

/* Register-indirect memory. */
static inline void sh4asm_emit_ld_b(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x6000, n, m));
}

static inline void sh4asm_emit_ld_w(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x6001, n, m));
}

static inline void sh4asm_emit_ld_l(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x6002, n, m));
}

static inline void sh4asm_emit_st_b(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2000, n, m));
}

static inline void sh4asm_emit_st_w(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2001, n, m));
}

static inline void sh4asm_emit_st_l(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0x2002, n, m));
}

/* State traffic. GBR displacement arguments are scaled, as in the ISA. */
static inline void sh4asm_emit_ld_b_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc400, d));
}

static inline void sh4asm_emit_ld_w_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc500, d));
}

static inline void sh4asm_emit_ld_l_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc600, d));
}

static inline void sh4asm_emit_st_b_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc000, d));
}

static inline void sh4asm_emit_st_w_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc100, d));
}

static inline void sh4asm_emit_st_l_gbr(struct sh4asm *as, unsigned int d)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc200, d));
}

/* PC-relative literal loads. */
static inline void sh4asm_emit_ld_w_pc(struct sh4asm *as, unsigned int d,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_ND8(0x9000, n, d));
}

static inline void sh4asm_emit_ld_l_pc(struct sh4asm *as, unsigned int d,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_ND8(0xd000, n, d));
}

/* Immediates. */
static inline void sh4asm_emit_mov_imm(struct sh4asm *as, int imm,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_ND8(0xe000, n, imm));
}

static inline void sh4asm_emit_add_imm(struct sh4asm *as, int imm,
			       unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_ND8(0x7000, n, imm));
}

static inline void sh4asm_emit_cmpeq_imm_r0(struct sh4asm *as, int imm)
{
	sh4asm_emit_word(as, SH4ASM_D8(0x8800, imm));
}

static inline void sh4asm_emit_and_imm_r0(struct sh4asm *as,
				  unsigned int imm)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xc900, imm));
}

static inline void sh4asm_emit_or_imm_r0(struct sh4asm *as,
				 unsigned int imm)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xcb00, imm));
}

static inline void sh4asm_emit_xor_imm_r0(struct sh4asm *as,
				  unsigned int imm)
{
	sh4asm_emit_word(as, SH4ASM_D8(0xca00, imm));
}

/* Branch displacements are in instruction words relative to PC + 4. */
static inline void sh4asm_emit_bra(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D12(0xa000, disp));
}

static inline void sh4asm_emit_bsr(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D12(0xb000, disp));
}

static inline void sh4asm_emit_bt(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D8(0x8900, disp));
}

static inline void sh4asm_emit_bf(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D8(0x8b00, disp));
}

static inline void sh4asm_emit_bt_s(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D8(0x8d00, disp));
}

static inline void sh4asm_emit_bf_s(struct sh4asm *as, int disp)
{
	sh4asm_emit_word(as, SH4ASM_D8(0x8f00, disp));
}

/* FPU forms needed by the later GTE bucket. */
static inline void sh4asm_emit_fadd(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0xf000, n, m));
}

static inline void sh4asm_emit_fmul(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0xf002, n, m));
}

static inline void sh4asm_emit_fmac(struct sh4asm *as, unsigned int m,
			    unsigned int n)
{
	sh4asm_emit_word(as, SH4ASM_NM(0xf00e, n, m));
}

static inline void sh4asm_emit_ftrv(struct sh4asm *as, unsigned int n)
{
	sh4asm_emit_word(as, (uint16_t)(0xf1fd | ((n & 3) << 10)));
}

#endif /* __LIGHTREC_SH4ASM_H__ */
