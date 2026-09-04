/* The decode pass, from the inside.
 *
 * `ir_decode()` is the whole of what the oracle needs: guest words in, IR out.
 * The emulator needs one thing more.  There the block does not arrive as raw
 * words -- lightrec has already run its optimiser over it, folded constants,
 * marked delay slots and labelled every load and store with the memory region
 * it hits -- so the *loop* over the block belongs to `front.c`, which walks
 * lightrec's `struct opcode_list` instead of a word array.
 *
 * What must NOT be duplicated is the per-instruction lowering.  That mapping
 * is what 100,000 oracle blocks validated; a second copy of it in the front
 * end would be a second thing to be wrong, and the oracle would never see it.
 * So the mapping stays here, exactly as tested, and both loops call into it.
 *
 * Nothing outside decode.c and front.c has any business including this.
 */

#ifndef DECODE_INT_H
#define DECODE_INT_H

#include <stdint.h>
#include "ir.h"

/* Where the pass is putting nodes.  `n` is the count so far, `max` the size of
 * the caller's array; every constructor returns 0 rather than overrun it, and
 * every caller of a constructor tests for that. */
typedef struct {
        ir_node *out;
        int      n;
        int      max;

        /* AN OPCODE THE DECODER HAS NO CASE FOR, WHICH IS NOT AN ERROR.
         *
         * The raw-word path skips one and says nothing: the oracle's
         * reference interpreter skips it too, so the two agree and there is
         * nothing to report.
         *
         * In the emulator it is the opposite of nothing. That is how an HLE
         * BIOS call is DELIVERED -- the frontend marks an opcode unknown so
         * the block will stop and hand it to C -- so an emitter that silently
         * drops it does not run the BIOS at all, and does it quietly. The
         * front end turns this into an IR_EXIT; see front.c.
         *
         * Set, never cleared, by whoever is filling the context. */
        int      unknown;
} ir_ctx;

/* Append a node, zeroed but for its opcode and guest PC. 0 if the array is
 * full. */
ir_node *ir_node_new(ir_ctx *c, int op, uint32_t pc);

/* rd = v, folded to nothing when rd is $zero. Exposed because a transfer's
 * link is a set and the front end forms some of those itself. */
void ir_emit_set(ir_ctx *c, uint32_t pc, unsigned rd, uint32_t v);

/* One non-transfer guest instruction. Folds $zero reads and writes; may append
 * no nodes at all, and appends more than one only where the guest instruction
 * genuinely is more than one operation. */
void ir_decode_op(ir_ctx *c, uint32_t insn, uint32_t pc);

/* Is this an MTC0 that C has to perform?  See ir.h on IR_MTC_C. */
int ir_mtc_needs_c(uint32_t insn);

/* Is this instruction a control transfer -- a branch, a jump, or a trap? */
int ir_is_transfer(uint32_t insn);

/* The part of a transfer that has to read guest state as it stood before the
 * delay slot: the comparison, the target register, the link. Returns non-zero
 * if a delay slot follows (SYSCALL and BREAK have none). */
int ir_decode_transfer(ir_ctx *c, uint32_t insn, uint32_t pc);

/* THE LOAD SHADOW, WHICH IS A PROPERTY OF THE LOOP AND NOT OF ONE OPCODE.
 *
 * A load's register write lands one instruction late, so the pass carries a
 * pending load across each step: `ir_shadow_pending` says whether the
 * instruction just lowered was a single load that can be moved, returning the
 * node index to remember or -1, and `ir_shadow_fix` deals with the next
 * instruction -- rotating the load past it, or, where that instruction writes
 * state the load reads, splitting the load instead so the memory access keeps
 * its place.  ir.h's comment on `defer` has the reasoning.
 *
 * Both front ends carry this, because both see instructions one at a time and
 * neither can see the hazard from inside a single opcode's lowering. */
int ir_shadow_pending(const ir_ctx *c, uint32_t insn, int mark);
int ir_shadow_fix(ir_ctx *c, int pend, uint32_t load, uint32_t insn, int mark);

#endif /* DECODE_INT_H */
