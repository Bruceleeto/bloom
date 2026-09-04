/* fgl — the SH-4 code generator.
 *
 * Guest instruction in, SH-4 out. It replaces GNU Lightning and lightrec's
 * emitter both, and the reason it can be smaller than either is that it sees
 * the guest instruction directly instead of a portable virtual RISC's idea of
 * it.
 *
 * THE REGISTER CONTRACT. Generated code does not obey the SH-4 C ABI and must
 * never be made to:
 *
 *   r0     the transfer register. Every state-block access goes through it,
 *          because `mov.l @(disp,GBR),Rn` EXISTS ONLY FOR R0. Nothing may
 *          hold a live value in r0 across a state access.
 *   r1     the emitter's second working register.
 *   r2     the exit register: the guest PC the block leaves for. Written
 *          before the delay slot runs, read by the epilogue -- so nothing
 *          between those two points may touch it, including anything the
 *          block calls.
 *   r3-r12 the allocator's pool. A node needing a third working register is
 *          given one here by the allocation pass, in `sc[]`.
 *   r13    the guest address mask. Held permanently, never written.
 *   r15    the stack.
 *   GBR    the state block.
 *
 * A SERVICE ROUTINE CALLED FROM INSIDE A BLOCK MAY CLOBBER r0 AND r1 AND
 * NOTHING ELSE. r2 carries the exit PC and r3-r12 carry guest values not yet
 * written back. A compiled C function is not callable from inside a block at
 * all: it knows nothing about GBR or r13.
 */

#ifndef FGL_H
#define FGL_H

#include <stdint.h>

#include "sh4.h"
#include "ir.h"
#include "alloc.h"

#define FGL_R_XFER  0
#define FGL_R_T1    1
#define FGL_R_EXIT  2
#define FGL_R_MASK  13

#define FGL_MAX_LITERALS 64

typedef struct {
	sh4_codegen cg;
	uint8_t    *start;      /* first word of the buffer          */
	uint32_t    base;       /* address that word will execute at */

	/* Sites waiting on a literal, and the values they want. The pool is
	 * placed after the code, and `mov.l @(disp,PC)` reaches 1020 bytes
	 * forward -- which a 32-instruction block cannot exceed. */
	struct { uint32_t value; int at; } fix[FGL_MAX_LITERALS];
	int n_fix;

	int overflow;           /* buffer or pool exhausted */

	/* An IR node fgl does not lower yet. Distinct from `overflow`: that
	 * is a block too big for its buffer, this is a hole in the emitter,
	 * and the two want completely different responses. */
	int unsupported;
	int unsupported_op;

	/* What the literal pool weighed, in bytes. It is DATA sitting inside
	 * the block and is never executed, so an instruction census that
	 * counted it would overstate the block by whatever its constants
	 * happen to be. */
	uint32_t pool_bytes;

	/* How many times the pool had to be flushed mid-block. Zero for
	 * anything short; a block that needs several is one whose lowering is
	 * materialising far too many constants. */
	int pool_flushes;
} fgl_emitter;

void     fgl_init(fgl_emitter *e, void *buf, uint32_t size, uint32_t base);
uint32_t fgl_size(const fgl_emitter *e);

/* Emit one already-decoded, already-allocated block. Returns its entry
 * address, or 0 if the emitter overflowed or met something it cannot lower. */
uint32_t fgl_emit(fgl_emitter *e, const ir_node *ir, int n, const ir_alloc *a);

/* Decode, allocate and emit in one call: `words` is a window of guest
 * instructions (see ir.h -- it must hold IR_MAX_INSNS + 1 of them) and `pc`
 * the guest address of the first. Returns the entry address, or 0. */
uint32_t fgl_emit_block(fgl_emitter *e, const uint32_t *words, uint32_t pc);

#endif /* FGL_H */
