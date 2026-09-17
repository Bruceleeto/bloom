/* The allocation pass: guest registers into host registers, one linear walk.
 * Writes only the per-node operand locations; a negative answer means "still in
 * the state block" and the emitter's memory path is the fallback.
 * r3-r12, LRU by a nibble-packed permutation table: the victim is rank 0.
 * Traffic comes out as (guest, host) pairs: preloads at block entry, fixups
 * charged to the node they precede (writeback on eviction/exit, reload on reuse).
 */

#ifndef ALLOC_H
#define ALLOC_H

#include <stdint.h>

#include "ir.h"
#include "pins.h"

/* r3 is the guest-file base under FGL_GBASE, so the pool starts at r4.  See pins.h. */
/* Overridable so a smaller pool can be priced; pins are placed by PIN_SLOT(host). */
#ifndef ALLOC_FIRST
#if FGL_GBASE
#define ALLOC_FIRST 4           /* r4 */
#define ALLOC_N     9           /* r4-r12 */
#else
#define ALLOC_FIRST 3           /* r3 */
#define ALLOC_N     10          /* r3-r12 */
#endif
#endif

/* A pool slot that is not the allocator's, or -1.  FGL_GBASE2 keeps the high
 * guest-file base in r9 (GTE leaves only protect r7-r14); the slot is never handed out.
 */
#if FGL_GBASE2
#define ALLOC_SKIP (FGL_R_GBASE2 - ALLOC_FIRST)
#else
#define ALLOC_SKIP (-1)
#endif

/* Guest registers that live in host registers across block boundaries, by pool
 * slot; -1 = arrives empty.  A preference, not a reservation: a block may still
 * evict a pinned register, and the exit flush puts the assignment back.
 */
extern const int8_t ir_pin[ALLOC_N];

/* One memory reference the emitter has to make on the allocator's behalf. */
typedef struct {
        uint8_t at;             /* emit before this node; == n at block end */
        uint8_t guest;          /* guest register number, i.e. GBR index    */
        uint8_t host;           /* host register number, 3-12               */
        uint8_t store;          /* 1: host -> state.  0: state -> host      */
} ir_fixup;

/* Worst case per node: three operands and two scratch, each evicting and reloading; plus the block-end flush. */
#define ALLOC_MAX_FIXUPS (IR_MAX_NODES * 8 + 2 * ALLOC_N)

typedef struct {
        ir_fixup preload[ALLOC_N];
        int      n_preload;
        ir_fixup fix[ALLOC_MAX_FIXUPS];
        int      n_fix;         /* in emission order: sorted by `at` */
} ir_alloc;

/* Allocate registers for `n` nodes, filling each node's `ad`/`as`/`at`/`sc[]` and `out` with the traffic. */
void ir_allocate(ir_node *ir, int n, ir_alloc *out);

/* Region pins: `extra[h]` is a guest register for free slot h, or -1.  `first` = the region's entry range. */
void ir_pin_set_region(const int8_t *extra);
void ir_allocate_region(ir_node *ir, int n, ir_alloc *out, int first);
extern int8_t  ir_pin_cur[ALLOC_N];
extern uint8_t ir_pin_region[ALLOC_N];

#endif /* ALLOC_H */
