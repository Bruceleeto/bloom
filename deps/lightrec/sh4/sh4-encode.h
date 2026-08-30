/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * SH-4 instruction encoders.
 *
 * The encodings are taken from GNU Lightning's SH-4 backend (jit_sh-cpu.c,
 * Paul Cercueil, LGPL), which is known-good and has been carrying bloom's
 * codegen. Only the encoding tables are reused: the node graph, the register
 * allocator and the two-pass emitter are not, because lightrec needs to own
 * register allocation to pin guest registers for the whole run, and lightning
 * reserves r0 and r3 for itself - two of the sixteen registers, one of which
 * is the one the sixth pin needs.
 *
 * Everything here writes one 16-bit instruction through a struct sh4_emit and
 * has no other state, so the encoders can be tested in isolation.
 */

#ifndef __SH4_ENCODE_H__
#define __SH4_ENCODE_H__

#include <stdint.h>

/* SH-4 general registers. r0 is special in a large part of the instruction
 * set - it is the only register many displacement and indexed forms accept -
 * so it is named rather than numbered at the call sites that require it. */
#define SH4_R0	0
#define SH4_R15	15

struct sh4_emit {
	uint16_t *pc;		/* next instruction to write */
	uint16_t *start;	/* first instruction of the buffer */
	uint16_t *end;		/* one past the last writable instruction */
	int overflow;		/* set once a write was refused */
};

static inline void sh4_emit_init(struct sh4_emit *e, void *buf, size_t len)
{
	e->start = e->pc = buf;
	e->end = (uint16_t *)((char *)buf + (len & ~1u));
	e->overflow = 0;
}

/* Byte offset of the next instruction from the start of the buffer. */
static inline unsigned int sh4_emit_offset(const struct sh4_emit *e)
{
	return (unsigned int)((char *)e->pc - (char *)e->start);
}

/* A refused write sets the overflow flag rather than running off the end;
 * callers check it once when the block is finished instead of at every
 * instruction. */
static inline void sh4_ii(struct sh4_emit *e, uint16_t insn)
{
	if (e->pc >= e->end) {
		e->overflow = 1;
		return;
	}
	*e->pc++ = insn;
}

/* The four SH-4 instruction shapes. Fields are masked here so that a caller
 * passing an out-of-range displacement corrupts its own operand rather than
 * the opcode of the instruction. */
static inline void sh4_cni(struct sh4_emit *e, unsigned c, unsigned n, unsigned i)
{
	sh4_ii(e, (uint16_t)(((c & 0xf) << 12) | ((n & 0xf) << 8) | (i & 0xff)));
}

static inline void sh4_cnmd(struct sh4_emit *e, unsigned c, unsigned n,
			    unsigned m, unsigned d)
{
	sh4_ii(e, (uint16_t)(((c & 0xf) << 12) | ((n & 0xf) << 8) |
			     ((m & 0xf) << 4) | (d & 0xf)));
}

static inline void sh4_cmd(struct sh4_emit *e, unsigned c, unsigned m, unsigned d)
{
	sh4_ii(e, (uint16_t)(((c & 0xf) << 12) | ((m & 0xf) << 8) | (d & 0xff)));
}

static inline void sh4_cd(struct sh4_emit *e, unsigned c, unsigned d)
{
	sh4_ii(e, (uint16_t)(((c & 0xf) << 12) | (d & 0xfff)));
}

/* --- 0x0 --- */
#define SH4_STRB(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0x4)	/* mov.b Rm,@(R0,Rn) */
#define SH4_STRW(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0x5)
#define SH4_STRL(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0x6)
#define SH4_MULL(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0x7)	/* mul.l -> MACL */
#define SH4_LDRB(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0xc)	/* mov.b @(R0,Rm),Rn */
#define SH4_LDRW(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0xd)
#define SH4_LDRL(e,rn,rm)	sh4_cnmd(e, 0x0, rn, rm, 0xe)
#define SH4_BSRF(e,rn)		sh4_cni(e, 0x0, rn, 0x03)
#define SH4_STCGBR(e,rn)	sh4_cni(e, 0x0, rn, 0x12)
#define SH4_STSMACH(e,rn)	sh4_cni(e, 0x0, rn, 0x0a)
#define SH4_STSMACL(e,rn)	sh4_cni(e, 0x0, rn, 0x1a)
#define SH4_BRAF(e,rn)		sh4_cni(e, 0x0, rn, 0x23)
#define SH4_MOVT(e,rn)		sh4_cni(e, 0x0, rn, 0x29)
#define SH4_STSPR(e,rn)		sh4_cni(e, 0x0, rn, 0x2a)
#define SH4_RTS(e)		sh4_ii(e, 0x000b)
#define SH4_NOP(e)		sh4_ii(e, 0x0009)
#define SH4_CLRT(e)		sh4_ii(e, 0x0008)
#define SH4_SETT(e)		sh4_ii(e, 0x0018)
#define SH4_CLRMAC(e)		sh4_ii(e, 0x0028)

/* --- 0x1: mov.l Rm,@(disp*4,Rn) --- */
#define SH4_STDL(e,rn,rm,d)	sh4_cnmd(e, 0x1, rn, rm, d)

/* --- 0x2 --- */
#define SH4_STB(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x0)	/* mov.b Rm,@Rn */
#define SH4_STW(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x1)
#define SH4_STL(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x2)
#define SH4_STBU(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x4)	/* mov.b Rm,@-Rn */
#define SH4_STWU(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x5)
#define SH4_STLU(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x6)
#define SH4_DIV0S(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x7)
#define SH4_TST(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x8)
#define SH4_AND(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0x9)
#define SH4_XOR(e,rn,rm)	sh4_cnmd(e, 0x2, rn, rm, 0xa)
#define SH4_OR(e,rn,rm)		sh4_cnmd(e, 0x2, rn, rm, 0xb)

/* --- 0x3 --- */
#define SH4_CMPEQ(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x0)
#define SH4_CMPHS(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x2)	/* unsigned >= */
#define SH4_CMPGE(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x3)	/* signed >= */
#define SH4_DIV1(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x4)
#define SH4_DMULU(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x5)
#define SH4_CMPHI(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x6)	/* unsigned > */
#define SH4_CMPGT(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x7)	/* signed > */
#define SH4_SUB(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0x8)
#define SH4_SUBC(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xa)
#define SH4_SUBV(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xb)
#define SH4_ADD(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xc)
#define SH4_DMULS(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xd)
#define SH4_ADDC(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xe)
#define SH4_ADDV(e,rn,rm)	sh4_cnmd(e, 0x3, rn, rm, 0xf)

/* --- 0x4 --- */
#define SH4_SHLL(e,rn)		sh4_cni(e, 0x4, rn, 0x00)
#define SH4_SHLR(e,rn)		sh4_cni(e, 0x4, rn, 0x01)
#define SH4_ROTL(e,rn)		sh4_cni(e, 0x4, rn, 0x04)
#define SH4_ROTR(e,rn)		sh4_cni(e, 0x4, rn, 0x05)
#define SH4_SHLL2(e,rn)		sh4_cni(e, 0x4, rn, 0x08)
#define SH4_SHLR2(e,rn)		sh4_cni(e, 0x4, rn, 0x09)
#define SH4_JSR(e,rn)		sh4_cni(e, 0x4, rn, 0x0b)
#define SH4_DT(e,rn)		sh4_cni(e, 0x4, rn, 0x10)
#define SH4_CMPPZ(e,rn)		sh4_cni(e, 0x4, rn, 0x11)	/* >= 0 */
#define SH4_CMPPL(e,rn)		sh4_cni(e, 0x4, rn, 0x15)	/* > 0 */
#define SH4_SHLL8(e,rn)		sh4_cni(e, 0x4, rn, 0x18)
#define SH4_SHLR8(e,rn)		sh4_cni(e, 0x4, rn, 0x19)
#define SH4_LDCGBR(e,rm)	sh4_cni(e, 0x4, rm, 0x1e)
#define SH4_SHAL(e,rn)		sh4_cni(e, 0x4, rn, 0x20)
#define SH4_SHAR(e,rn)		sh4_cni(e, 0x4, rn, 0x21)
#define SH4_ROTCL(e,rn)		sh4_cni(e, 0x4, rn, 0x24)
#define SH4_ROTCR(e,rn)		sh4_cni(e, 0x4, rn, 0x25)
#define SH4_SHLL16(e,rn)	sh4_cni(e, 0x4, rn, 0x28)
#define SH4_SHLR16(e,rn)	sh4_cni(e, 0x4, rn, 0x29)
#define SH4_LDSPR(e,rn)		sh4_cni(e, 0x4, rn, 0x2a)
#define SH4_JMP(e,rn)		sh4_cni(e, 0x4, rn, 0x2b)
#define SH4_LDSMACL(e,rn)	sh4_cni(e, 0x4, rn, 0x1a)
#define SH4_SHAD(e,rn,rm)	sh4_cnmd(e, 0x4, rn, rm, 0xc)	/* arithmetic, dynamic */
#define SH4_SHLD(e,rn,rm)	sh4_cnmd(e, 0x4, rn, rm, 0xd)	/* logical, dynamic */

/* --- 0x5: mov.l @(disp*4,Rm),Rn --- */
#define SH4_LDDL(e,rn,rm,d)	sh4_cnmd(e, 0x5, rn, rm, d)

/* --- 0x6 --- */
#define SH4_LDB(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x0)	/* mov.b @Rm,Rn */
#define SH4_LDW(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x1)
#define SH4_LDL(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x2)
#define SH4_MOV(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x3)
#define SH4_LDBU(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x4)	/* mov.b @Rm+,Rn */
#define SH4_LDWU(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x5)
#define SH4_LDLU(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x6)
#define SH4_NOT(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x7)
#define SH4_SWAPB(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x8)
#define SH4_SWAPW(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0x9)
#define SH4_NEGC(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xa)
#define SH4_NEG(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xb)
#define SH4_EXTUB(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xc)
#define SH4_EXTUW(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xd)
#define SH4_EXTSB(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xe)
#define SH4_EXTSW(e,rn,rm)	sh4_cnmd(e, 0x6, rn, rm, 0xf)

/* --- 0x7: add #imm,Rn (imm is sign-extended) --- */
#define SH4_ADDI(e,rn,i)	sh4_cni(e, 0x7, rn, i)

/* --- 0x8 --- */
#define SH4_LDDB(e,rm,d)	sh4_cnmd(e, 0x8, 0x4, rm, d)	/* mov.b @(d,Rm),R0 */
#define SH4_LDDW(e,rm,d)	sh4_cnmd(e, 0x8, 0x5, rm, d)
#define SH4_STDB(e,rn,d)	sh4_cnmd(e, 0x8, 0x0, rn, d)	/* mov.b R0,@(d,Rn) */
#define SH4_STDW(e,rn,d)	sh4_cnmd(e, 0x8, 0x1, rn, d)
#define SH4_CMPEQI(e,i)		sh4_cni(e, 0x8, 0x8, i)		/* cmp/eq #imm,R0 */
#define SH4_BT(e,d)		sh4_cni(e, 0x8, 0x9, d)
#define SH4_BF(e,d)		sh4_cni(e, 0x8, 0xb, d)
#define SH4_BTS(e,d)		sh4_cni(e, 0x8, 0xd, d)		/* with delay slot */
#define SH4_BFS(e,d)		sh4_cni(e, 0x8, 0xf, d)

/* --- 0x9: mov.w @(disp*2,PC),Rn --- */
#define SH4_LDPW(e,rn,d)	sh4_cni(e, 0x9, rn, d)

#define SH4_BRA(e,d)		sh4_cd(e, 0xa, d)
#define SH4_BSR(e,d)		sh4_cd(e, 0xb, d)

/* --- 0xc: GBR-relative, R0 only --- */
#define SH4_GBRSTB(e,d)		sh4_cni(e, 0xc, 0x0, d)
#define SH4_GBRSTW(e,d)		sh4_cni(e, 0xc, 0x1, d)
#define SH4_GBRSTL(e,d)		sh4_cni(e, 0xc, 0x2, d)
#define SH4_GBRLDB(e,d)		sh4_cni(e, 0xc, 0x4, d)
#define SH4_GBRLDW(e,d)		sh4_cni(e, 0xc, 0x5, d)
#define SH4_GBRLDL(e,d)		sh4_cni(e, 0xc, 0x6, d)
#define SH4_MOVA(e,d)		sh4_cni(e, 0xc, 0x7, d)
#define SH4_TSTI(e,i)		sh4_cni(e, 0xc, 0x8, i)
#define SH4_ANDI(e,i)		sh4_cni(e, 0xc, 0x9, i)
#define SH4_XORI(e,i)		sh4_cni(e, 0xc, 0xa, i)
#define SH4_ORI(e,i)		sh4_cni(e, 0xc, 0xb, i)

/* --- 0xd: mov.l @(disp*4,PC),Rn --- */
#define SH4_LDPL(e,rn,d)	sh4_cni(e, 0xd, rn, d)

/* --- 0xe: mov #imm,Rn (imm is sign-extended) --- */
#define SH4_MOVI(e,rn,i)	sh4_cni(e, 0xe, rn, i)

#endif /* __SH4_ENCODE_H__ */
