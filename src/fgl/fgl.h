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
 *          before the delay slot runs and carried out of the block into the
 *          dispatcher, which reads it -- so nothing from the write to the end
 *          of the block may touch it, including anything the block calls.
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
 *
 * PR IS NOT IN THIS CONTRACT, AND THAT IS DELIBERATE. A block does not
 * return -- it ends by jumping through FGL_AT_DISPATCH -- so nothing in a
 * block owns PR and a service may use it freely for its own call. The
 * alternative, an `rts` epilogue, costs the same five instructions and
 * silently makes PR live for the whole of every block, which would put a save
 * and a restore around every service call for no gain. See the note on
 * FGL_AT_DISPATCH in fgl_state.h.
 *
 * WHAT A C-ABI CALL ACTUALLY COSTS, WHICH IS LESS THAN IT LOOKS. r8-r15 are
 * callee-saved on SH-4, so r8-r12 of the guest pool, the mask in r13 and the
 * cycle delta in r14 all survive a compiled C function on their own. Only
 * r2-r7 need saving, and to the STACK rather than to the state block: a
 * GBR-relative spill of a register that is not r0 costs two instructions each
 * way instead of one, and a fixed shim has no access to the allocator's model
 * of which registers are dirty. GBR itself is the SH-4 TLS pointer and is NOT
 * callee-saved -- it survives only because nothing in the linked image uses
 * TLS, which is a property to assert at link time rather than to assume.
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

/* THE CYCLE DELTA, AND WHY IT IS r14.
 *
 * lightrec keeps `target_cycle - current_cycle` as a signed quantity in a
 * register for the whole of a run, not per block: every block subtracts what
 * it cost, and the DISPATCHER, not the block, tests whether it has gone
 * negative. So a block never branches on it and never reloads it -- it
 * subtracts once and leaves.
 *
 * r14 is the only general register the contract had not already spoken for.
 * It is the frame pointer in the SH-4 C ABI, which costs nothing here: it is
 * callee-saved, so the dispatcher saves it once on the way in from C and
 * restores it on the way out, and no compiled C runs inside a block to want
 * it. Two instructions per `lightrec_execute`, not per block.
 *
 * A service routine may clobber r0 and r1 AND NOTHING ELSE, so it may not
 * touch this either. The oracle has always checked r14 unchanged across a
 * block; it now checks the charge instead, which is the same property with a
 * known answer. */
#define FGL_R_CYCLE 14

#define FGL_MAX_LITERALS 64

/* WHERE THE SERVICES ARE, AND WHY THE EMITTER IS TOLD RATHER THAN LINKED.
 *
 * A block reaches a service by materialising its address as a literal and
 * `jsr`-ing through it (see shim.h).  On the Dreamcast those addresses are
 * ordinary link-time symbols, so the obvious emitter takes them straight from
 * `&fgl_shim_gte`.  It cannot: emit.c is also compiled by the host oracle and
 * by test_reloc, where none of those symbols exist and the SH-4 they emit is
 * run by an interpreter rather than a linker.  Referencing them directly
 * would make the newest and least proven paths the only ones the oracle is
 * structurally unable to see.
 *
 * So the addresses arrive as data.  The emulator fills this in from the real
 * symbols; a host test fills it in with stubs it can point wherever it likes,
 * and exercises the identical call site.
 *
 * A zero address is not a default to fall back from -- there is no fallback
 * path in fgl -- it is "this service was not supplied", and a node that needs
 * it sets `unsupported` and says so.
 */
typedef struct {
	uint32_t shim_call;     /* r0 = callee, r1 = arg   -> f(arg, state) */
	uint32_t shim_gte;      /* r0 = callee, r1 = op    -> f(&cp2d, op)  */
	uint32_t shim_divu;     /* r1 / r0 -> r0 quotient, r1 remainder     */
	uint32_t shim_div;
	uint32_t shim_call_st;  /* r0 = callee, r1 = addr, shim_arg = value */

	/* The hardware-register accessors, indexed by the IR's MEM_* width so
	 * that the width the decoder already resolved is the index and there
	 * is no second switch.  `lightrec_hw_lb` and friends take the guest
	 * address and the state block and RETURN THE VALUE ALREADY EXTENDED to
	 * 32 bits -- signed for lb/lh, zero for lbu/lhu -- so a call site must
	 * not extend it again.  The stores take a third argument, which is why
	 * they go through `shim_call_st`.
	 *
	 * A width with no accessor is a zero, and a zero is a refusal: stores
	 * have no unsigned forms and never ask for one. */
	uint32_t hw_load[5];    /* MEM_B, MEM_BU, MEM_H, MEM_HU, MEM_W */
	uint32_t hw_store[5];   /* MEM_B,       , MEM_H,       , MEM_W */

	/* The whole-access bridge, for a region the optimiser could not prove.
	 * `f(the guest instruction word, the state block)`, which is
	 * fgl_shim_call's shape exactly -- it reads the base register out of
	 * the state block and writes the destination back to it. */
	uint32_t rw;

	/* The Status/Cause write, same call shape as `rw`: `f(the guest
	 * instruction word, the state block)`.  See ir.h on IR_MTC_C. */
	uint32_t mtc;

	/* The C body that runs one COP2 command, given the guest instruction
	 * word.  Resolved AT COMPILE TIME -- the command is a constant in the
	 * block, so there is no runtime dispatch and nothing decodes it twice.
	 * Returns 0 for a command the hardware ignores, for which the right
	 * amount of code is none.  `user` is passed back untouched so a host
	 * test can hang its own table off it. */
	uint32_t (*gte_body)(void *user, uint32_t op);
	void     *user;
} fgl_targets;

typedef struct {
	sh4_codegen cg;
	uint8_t    *start;      /* first word of the buffer          */
	uint32_t    base;       /* address that word will execute at */

	/* Sites waiting on a literal, and the values they want. The pool is
	 * placed after the code, and `mov.l @(disp,PC)` reaches 1020 bytes
	 * forward -- which a 32-instruction block cannot exceed. */
	struct { uint32_t value; int at; } fix[FGL_MAX_LITERALS];
	int n_fix;

	/* Service addresses, or NULL.  See fgl_targets: NULL and a zero entry
	 * mean the same thing, which is that a node needing that service
	 * cannot be emitted. */
	const fgl_targets *tgt;

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

/* Supply the service addresses.  Separate from `fgl_init` so a harness with
 * no services keeps compiling and simply cannot emit the nodes that need
 * them.  `t` is borrowed, not copied. */
void     fgl_set_targets(fgl_emitter *e, const fgl_targets *t);
uint32_t fgl_size(const fgl_emitter *e);

/* Emit one already-decoded, already-allocated block. Returns its entry
 * address, or 0 if the emitter overflowed or met something it cannot lower. */
/* Emit one block. `n_ops` is the number of GUEST INSTRUCTIONS it covers, delay
 * slot included -- not the node count, which folding and transfer expansion
 * both move. It is what the block charges the cycle counter for, so getting it
 * wrong desynchronises the machine without producing a single wrong register:
 * `fgl_front` reports it as `info.n_ops`, and the raw-word path takes it from
 * `ir_block_length`. */
uint32_t fgl_emit(fgl_emitter *e, const ir_node *ir, int n, const ir_alloc *a,
		  unsigned n_ops);

/* Decode, allocate and emit in one call: `words` is a window of guest
 * instructions (see ir.h -- it must hold IR_MAX_INSNS + 1 of them) and `pc`
 * the guest address of the first. Returns the entry address, or 0. */
uint32_t fgl_emit_block(fgl_emitter *e, const uint32_t *words, uint32_t pc);

#endif /* FGL_H */
