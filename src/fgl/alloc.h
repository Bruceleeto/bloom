/* The allocation pass: guest registers into host registers, one linear walk.
 *
 * It sits between decode and emit and rewrites nothing in the IR except three
 * bytes per node — where each of that node's operands ended up.  A negative
 * answer means "still in the state block", so the emitter's existing memory
 * path is the fallback for every node, and a block that runs out of registers
 * degrades instead of failing.
 *
 * TEN REGISTERS, `r3`-`r12`, LRU BY PERMUTATION TABLE.  Ranks 0-9 are packed a
 * nibble each into five bytes; touching the register at rank k is five table
 * lookups, and the victim is the one at rank 0.  Because the ranks are always
 * a permutation there is exactly one of those — no free-list, no "is anything
 * free?" test, no fallback path.  The tables are built once at init.
 *
 * Two kinds of memory traffic come out of the pass, and both are lists of
 * (guest, host) pairs the emitter turns into single instructions:
 *
 *   preloads   a register read before it is written, loaded once at block
 *              entry.  Only for host registers nothing else has used yet —
 *              once a host register has held something, a load into it has to
 *              wait until that something is dead.
 *   fixups     everything else, each charged to the node it must be emitted
 *              in front of: a writeback when a value is evicted or the block
 *              ends, a reload when an evicted register is read again.
 *
 * A node that needs more working registers than `r0` and `r1` is given scratch
 * out of the pool — the least recently used ones, which are not touched, so
 * they stay first in line to be taken again.
 */

#ifndef ALLOC_H
#define ALLOC_H

#include <stdint.h>

#include "ir.h"

#define ALLOC_FIRST 3           /* r3 */
#define ALLOC_N     10          /* r3-r12 */

/* SIX GUEST REGISTERS LIVE IN HOST REGISTERS ACROSS BLOCK BOUNDARIES, indexed
 * here by pool slot: -1 for a register that arrives holding nothing.
 *
 * The mapping is a compile-time constant, which is the whole point — two
 * blocks join with a bare `bra` and no state handshake, because the callee
 * side already knows what it is being handed.
 *
 * IT IS A PREFERENCE, NOT A RESERVATION.  The four unpinned registers rank
 * lowest, so they are handed out before any pinned one is disturbed, but a
 * block that wants more may evict a pinned register like any other — and the
 * flush at every exit puts the assignment back.  Nothing is reserved and no
 * allocation can fail. */
extern const int8_t ir_pin[ALLOC_N];

/* One memory reference the emitter has to make on the allocator's behalf. */
typedef struct {
        uint8_t at;             /* emit before this node; == n at block end */
        uint8_t guest;          /* guest register number, i.e. GBR index    */
        uint8_t host;           /* host register number, 3-12               */
        uint8_t store;          /* 1: host -> state.  0: state -> host      */
} ir_fixup;

/* Worst case per node: three operands and two scratch registers, each evicting
 * a dirty value and each operand then needing a load.  Plus, when the block
 * ends, one writeback per pool register and one reload per pinned one. */
#define ALLOC_MAX_FIXUPS (IR_MAX_NODES * 8 + 2 * ALLOC_N)

typedef struct {
        ir_fixup preload[ALLOC_N];
        int      n_preload;
        ir_fixup fix[ALLOC_MAX_FIXUPS];
        int      n_fix;         /* in emission order: sorted by `at` */
} ir_alloc;

/* Allocate registers for `n` nodes, filling in each node's `ad`/`as`/`at` and
 * `sc[]`, and `out` with the traffic that needs emitting. */
void ir_allocate(ir_node *ir, int n, ir_alloc *out);

#endif /* ALLOC_H */
