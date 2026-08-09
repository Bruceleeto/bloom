/*
 * Copyright (C) 2020  Free Software Foundation, Inc.
 *
 * This file is part of GNU lightning.
 *
 * GNU lightning is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU lightning is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * Authors:
 *	Paul Cercueil
 */

#ifndef _jit_sh_h
#define _jit_sh_h

#define JIT_HASH_CONSTS		0
#define JIT_NUM_OPERANDS	2

/* SH-4 register-model fork, off by default.
 *
 * Stock exposes six V registers, r8-r13.  With the fork the four C
 * argument registers r4-r7 join the V pool as well, and a spill/reload
 * shim around calli/callr keeps their contents alive across C calls, which
 * the SH-4 ABI does not.  Together that gives a client ten V registers
 * instead of six.
 *
 * Enable with -DJIT_SH_FORK=1 (cmake -DSH_FORK=ON).  It must be defined
 * for every translation unit, not just lightning's: JIT_SH_WIDE_POOL
 * changes JIT_V_NUM, and a client that sizes anything off JIT_V_NUM will
 * miscompile silently if it sees a different value than the backend does.
 * Hence a global definition and a default here, never a per-target one.
 *
 * JIT_SH_WIDE_POOL (this file) and JIT_SH_SHIM (jit_sh-cpu.c) both derive
 * from it and are individually overridable, but only 0/0 and 1/1 are valid
 * configurations — the wide pool without the shim hands out registers the
 * next C call destroys. */
#ifndef JIT_SH_FORK
#define JIT_SH_FORK		0
#endif

#ifndef JIT_SH_WIDE_POOL
#define JIT_SH_WIDE_POOL	JIT_SH_FORK
#endif

typedef enum {
#define jit_r(i)		(JIT_R0 + (i))
#define jit_r_num()		3
#if JIT_SH_WIDE_POOL
/* Deliberately NOT contiguous: V0..V4 are the callee-saved r8-r12, V5..V8
 * are the C argument registers r4-r7, and V9 is r13.
 *
 * The argument registers come LAST for two reasons.  A client that uses
 * fewer than five V registers then gets exactly stock's placement and never
 * touches the shim at all; and the registers that cost a spill/reload at
 * every C call are only handed out once the free ones are used up.
 *
 * The ordering is also load-bearing for correctness with lightrec, which
 * uses JIT_V0 and JIT_V1 by name for the guest PC and the branch target
 * while they are simultaneously its guest register slots 0 and 1.  Putting
 * the argument registers first made those two aliases of the first two
 * marshalling targets, and the shim then destroyed control-flow data.
 *
 * jit_hppa.h has a non-contiguous jit_v() too, so nothing inside lightning
 * assumes otherwise — but a client that inverts the mapping by arithmetic
 * will need fixing, as lightrec's regcache did. */
#define jit_v(i)		((i) < 5 ? _R8 + (i) : \
				 ((i) < 9 ? _R4 + (i) - 5 : _R13))
#define jit_v_num()		10
#else
#define jit_v(i)		(JIT_V0 + (i))
#define jit_v_num()		6
#endif
#define jit_f(i)		(JIT_F0 - (i) * 2)
#ifdef __SH_FPU_ANY__
#    define jit_f_num()		8
#else
#    define jit_f_num()		0
#endif
	_R0,

	/* caller-saved temporary registers */
#define JIT_R0			_R1
#define JIT_R1			_R2
#define JIT_R2			_R3
	_R1,	_R2,	_R3,

	/* C argument registers, exposed as the TOP of the V pool (V5..V8):
	 * preserved across C calls by the spill/reload shim around
	 * calli/callr, not by the callee. */
#if JIT_SH_WIDE_POOL
#define JIT_V5			_R4
#define JIT_V6			_R5
#define JIT_V7			_R6
#define JIT_V8			_R7
#endif
	_R4,	_R5,	_R6,	_R7,

	/* callee-saved registers */
#if JIT_SH_WIDE_POOL
#define JIT_V0			_R8
#define JIT_V1			_R9
#define JIT_V2			_R10
#define JIT_V3			_R11
#define JIT_V4			_R12
#define JIT_V9			_R13
#else
#define JIT_V0			_R8
#define JIT_V1			_R9
#define JIT_V2			_R10
#define JIT_V3			_R11
#define JIT_V4			_R12
#define JIT_V5			_R13
#endif
	_R8,	_R9,	_R10,	_R11,	_R12,	_R13,

#define JIT_FP			_R14
	_R14,
	_R15,

	_GBR,

	/* floating-point registers */
#define JIT_F0			_F14
#define JIT_F1			_F12
#define JIT_F2			_F10
#define JIT_F3			_F8
#define JIT_F4			_F6
#define JIT_F5			_F4
#define JIT_F6			_F2
#define JIT_F7			_F0
	_F0,	_F1,	_F2,	_F3,	_F4,	_F5,	_F6,	_F7,
	_F8,	_F9,	_F10,	_F11,	_F12,	_F13,	_F14,	_F15,

	/* Banked floating-point registers */
	_XF0,	_XF1,	_XF2,	_XF3,	_XF4,	_XF5,	_XF6,	_XF7,
	_XF8,	_XF9,	_XF10,	_XF11,	_XF12,	_XF13,	_XF14,	_XF15,

#define JIT_NOREG		_NOREG
	_NOREG,
} jit_reg_t;

#endif /* _jit_sh_h */
