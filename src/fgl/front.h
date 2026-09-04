/* The emulator's front end: lightrec's optimised block in, fgl IR out.
 *
 * `ir_decode()` reads raw guest words and is what the oracle drives.  In the
 * emulator the block has already been through lightrec's optimiser, and that
 * pass knows things no amount of looking at the words can recover: which
 * memory region a load reaches, that a divisor cannot be zero, that a delay
 * slot has already been moved.  Throwing that away and re-deriving from the
 * raw words would cost the mask on every access and the check on every
 * division, which is most of what this project exists to remove.
 *
 * So the two front ends share their per-instruction lowering -- the mapping
 * the oracle validated -- and differ only in the loop and in what the loop
 * knows.  See decode_int.h.
 */

#ifndef FGL_FRONT_H
#define FGL_FRONT_H

#include <stdint.h>
#include "ir.h"

struct opcode;

typedef struct {
        unsigned n_ops;         /* guest instructions this fgl block covers */
        unsigned unsupported;   /* opcodes the front end could not lower    */
        uint32_t unsupported_pc;
        uint32_t unsupported_op;
        int      ended_early;   /* stopped before the list did, and why:    */
        uint32_t stop_reason;   /* one of FGL_STOP_*                        */
} fgl_front_info;

enum {
        FGL_STOP_END,           /* ran out of opcode list                   */
        FGL_STOP_TRANSFER,      /* a control transfer ended the block       */
        FGL_STOP_LOCAL,         /* a branch inside the block; see front.c   */
        FGL_STOP_FULL,          /* the IR array filled                      */
        FGL_STOP_UNSUPPORTED,   /* an opcode with no lowering               */
        FGL_STOP_UNKNOWN_OP     /* an opcode C must execute -- an HLE call  */
};

/* lightrec's exit flag for "this opcode is not mine", which is how the
 * emulator's frontend receives an HLE BIOS call. Spelled out here rather than
 * pulling lightrec.h into the IR: it is one number and it is part of the
 * contract between a block and its dispatcher. */
#define FGL_EXIT_UNKNOWN_OP (1u << 5)

/* Lower one fgl block from lightrec's list.
 *
 * `ops` is the block's opcode array and `nb` its length; `pc` is the guest
 * address of ops[0].  Lowering starts at ops[0] and stops at the first control
 * transfer, so ONE CALL DOES NOT NECESSARILY CONSUME THE WHOLE LIST -- a
 * lightrec block can hold several basic blocks.  `info->n_ops` says how far it
 * got; call again from there for the next one.
 *
 * Returns the node count.
 */
int fgl_front(const struct opcode *ops, unsigned nb, uint32_t pc,
              ir_node *out, int max, fgl_front_info *info);

#endif /* FGL_FRONT_H */
