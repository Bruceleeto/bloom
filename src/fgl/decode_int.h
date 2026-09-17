/* Decode internals shared by decode.c (raw words) and front.c (lightrec's list).
 * The per-instruction lowering lives in one place; only the loops differ.
 */

#ifndef DECODE_INT_H
#define DECODE_INT_H

#include <stdint.h>
#include "ir.h"

/* Where the pass is putting nodes; every constructor returns 0 rather than overrun `max`. */
typedef struct {
        ir_node *out;
        int      n;
        int      max;

        /* An opcode with no case.  The raw-word path just skips it; the emulator turns
         * it into an IR_EXIT, since that is how an HLE BIOS call is delivered (front.c).
         */
        int      unknown;
} ir_ctx;

/* Append a node, zeroed but for its opcode and guest PC.  0 if the array is full. */
ir_node *ir_node_new(ir_ctx *c, int op, uint32_t pc);

/* rd = v, folded to nothing when rd is $zero. */
void ir_emit_set(ir_ctx *c, uint32_t pc, unsigned rd, uint32_t v);

/* One non-transfer guest instruction.  May append zero or several nodes. */
void ir_decode_op(ir_ctx *c, uint32_t insn, uint32_t pc);

/* Is this an MTC0 that C has to perform?  See ir.h on IR_MTC_C. */
int ir_mtc_needs_c(uint32_t insn);

/* Is this instruction a control transfer -- a branch, a jump, or a trap? */
int ir_is_transfer(uint32_t insn);

/* The part of a transfer that reads guest state before the delay slot.  Non-zero if a slot follows. */
int ir_decode_transfer(ir_ctx *c, uint32_t insn, uint32_t pc);

/* The load shadow: a load's register write lands one instruction late.
 * `ir_shadow_pending` notes a movable load, `ir_shadow_fix` rotates it past the
 * next instruction or splits it when that instruction writes what the load reads.  See ir.h `defer`.
 */
int ir_shadow_pending(const ir_ctx *c, uint32_t insn, int mark);
int ir_shadow_fix(ir_ctx *c, int pend, uint32_t load, uint32_t insn, int mark);
int ir_slot_holds_shadow(uint32_t slot, uint32_t load,
                         uint32_t branch);

#endif /* DECODE_INT_H */
