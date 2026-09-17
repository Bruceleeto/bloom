/* The emulator's front end: lightrec's optimised block in, fgl IR out.
 * The per-instruction lowering is shared with ir_decode() (decode_int.h);
 * this loop differs only in walking lightrec's opcode list and using its facts.
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

/* lightrec's exit flag for "this opcode is not mine": how an HLE BIOS call arrives. */
#define FGL_EXIT_UNKNOWN_OP (1u << 5)

/* Lower one fgl block from lightrec's list.  Stops at the first control transfer,
 * so one call need not consume the whole list: `info->n_ops` says how far it got.
 * Returns the node count.
 */
int fgl_front(const struct opcode *ops, unsigned nb, uint32_t pc,
              ir_node *out, int max, fgl_front_info *info);

#endif /* FGL_FRONT_H */
