/* The allocation pass.  See alloc.h for the shape; this is the walk.
 *
 * One forward pass over the nodes.  For each node the sources are taken first
 * and the destinations after, so that a destination needing a register never
 * evicts a source it is about to read: taking a register makes it most
 * recently used, and the victim is always the least.
 *
 * Nothing here looks ahead.  A value is written back when its register is
 * wanted by something else, or when the block ends, and never speculatively.
 */

#include "alloc.h"
#include "fgl_state.h"

/* THE POOL HAS TO OUTNUMBER WHAT ONE NODE CAN WANT AT ONCE, or a node's own
 * allocation evicts one of its own operands — the emitter still holds the host
 * register number the pass gave it, and by the time the code runs something
 * else is living there.  The worst node is a division: four guest registers
 * (two sources, LO and HI) and one scratch for the service pointer.
 *
 * Nothing enforces this at run time and nothing can: the allocation is already
 * baked into the node by then.  Ten leaves plenty of room, and the failure at
 * three is a real one — the oracle finds it in seconds. */
_Static_assert(ALLOC_N >= 5, "a node can want four registers and a scratch");

/* The pinned assignment.  Slot 0 is r3.
 *
 * The four that hold nothing are the low-ranked ones, so they are spent first
 * and a short block never disturbs a pinned register at all.  They are also
 * the three the block-boundary services use as working storage, which is only
 * safe because nothing is live in them when a block begins or ends. */
/* PINNING IS OFF, AND THAT IS A DECISION RATHER THAN AN OVERSIGHT.
 *
 * A pinned guest register is one a block assumes it was HANDED, in a host
 * register, by whoever branched to it -- no preload at entry, no writeback at
 * exit. It is worth real instructions, and it is also a contract between two
 * blocks that were compiled separately.
 *
 * Two reasons it stays off for now:
 *
 *   - Nothing can check it yet. The oracle runs one block, cold, and reads
 *     the registers out; a cross-block register handshake is invisible to it
 *     by construction. Turning this on would make the oracle report failures
 *     for correct code and, worse, hide real ones behind the noise.
 *   - lightrec enters blocks through a code LUT from arbitrary predecessors,
 *     including the interpreter and the exception path. Every one of those
 *     entries has to honour the same assignment or the block reads garbage.
 *     That is the dispatcher's problem, and the dispatcher does not exist.
 *
 * The mechanism is intact: put the guest register numbers back and the
 * allocator uses them again. Revisit once blocks link directly, and expect to
 * need a hardware check rather than an oracle one. */
const int8_t ir_pin[ALLOC_N] = {
        -1, -1, -1, -1,         /* r3-r6  */
        -1, -1, -1, -1,         /* r7-r10 */
        -1, -1                  /* r11-r12 */
};

/* ---------------------------------------------------------------- */
/* LRU                                                               */
/* ---------------------------------------------------------------- */

/* Ten ranks, one nibble each, two to a byte — so five bytes hold the whole
 * order.  Touching the register at rank k makes it rank 9 and drops everything
 * above k by one; that transform is a function of the nibble alone, so a table
 * per k does two registers at a time and the whole update is five loads. */
#define RANK_BYTES ((ALLOC_N + 1) / 2)

static uint8_t lru_tab[ALLOC_N][256];
static int     lru_built;

static void lru_build(void)
{
        int k, b;

        for (k = 0; k < ALLOC_N; k++)
                for (b = 0; b < 256; b++) {
                        int lo = b & 15, hi = (b >> 4) & 15;

                        lo = (lo == k) ? ALLOC_N - 1 : (lo > k ? lo - 1 : lo);
                        hi = (hi == k) ? ALLOC_N - 1 : (hi > k ? hi - 1 : hi);
                        lru_tab[k][b] = (uint8_t)((hi << 4) | lo);
                }
        lru_built = 1;
}

typedef struct {
        int8_t    owner[ALLOC_N];       /* guest register held, -1 for none  */
        uint8_t   dirty[ALLOC_N];
        uint8_t   used[ALLOC_N];        /* has ever held anything            */
        uint8_t   last[ALLOC_N];        /* node of the last reference        */
        uint8_t   wrote[ALLOC_N];       /* ...and whether it was a write     */
        int8_t    where[GUEST_HI + 1];  /* pool index per guest register     */
        uint8_t   rank[RANK_BYTES];
        int       node;                 /* the node being allocated for      */
        ir_alloc *a;
} allocator;

static int rank_of(const allocator *s, int h)
{
        return (s->rank[h >> 1] >> ((h & 1) * 4)) & 15;
}

static void touch(allocator *s, int h)
{
        int k = rank_of(s, h);
        int b;

        for (b = 0; b < RANK_BYTES; b++)
                s->rank[b] = lru_tab[k][s->rank[b]];
}

/* The ranks are a permutation, so exactly one register has rank 0 and there is
 * no case where nothing comes back. */
static int victim(const allocator *s)
{
        int h;

        for (h = 0; h < ALLOC_N; h++)
                if (rank_of(s, h) == 0)
                        return h;
        return 0;
}

/* ---------------------------------------------------------------- */
/* Traffic                                                           */
/* ---------------------------------------------------------------- */

static void fixup_at(allocator *s, int at, int guest, int h, int store)
{
        ir_fixup *f;

        if (s->a->n_fix >= ALLOC_MAX_FIXUPS)
                return;
        f = &s->a->fix[s->a->n_fix++];
        f->at    = (uint8_t)at;
        f->guest = (uint8_t)guest;
        f->host  = (uint8_t)(ALLOC_FIRST + h);
        f->store = (uint8_t)store;
}

static void fixup(allocator *s, int guest, int h, int store)
{
        fixup_at(s, s->node, guest, h, store);
}

/* A writeback goes to the node where the value was last referenced, not to the
 * node that evicted it.  The register is untouched in between, so the store is
 * valid anywhere in that span, and putting it at the near end keeps it away
 * from the code that wanted the register.
 *
 * ONE CASE CANNOT GO THERE.  Writebacks are emitted in front of their node's
 * code, so if the last reference was a *write*, the register does not hold the
 * value yet — the store would save what the node is about to overwrite.  Those
 * are charged to the node after it, which is the same position one instruction
 * later.
 */
static void charge_writeback(allocator *s, int h)
{
        int at = s->last[h] + (s->wrote[h] ? 1 : 0);

        if (at > s->node)
                at = s->node;
        fixup_at(s, at, s->owner[h], h, 1);
}

static void evict(allocator *s, int h)
{
        if (s->owner[h] < 0)
                return;

        if (s->dirty[h])
                charge_writeback(s, h);
        s->where[s->owner[h]] = -1;
        s->owner[h] = -1;
        s->dirty[h] = 0;
}

/* A guest register into the pool.  `want_value` is whether its current value
 * matters — a pure destination does not need loading, which is the whole
 * saving on a register written before it is read. */
static int take(allocator *s, unsigned g, int want_value)
{
        int h;

        if (s->where[g] >= 0) {
                h = s->where[g];
                s->last[h] = (uint8_t)s->node;
                s->wrote[h] = 0;
                touch(s, h);
                return h;
        }

        h = victim(s);
        evict(s, h);

        if (want_value) {
                /* A register nothing has used yet can be loaded at block
                 * entry, where the load is furthest from its use.  Once it has
                 * held a value, the load has to wait until that value is
                 * dead. */
                if (!s->used[h] && s->a->n_preload < ALLOC_N) {
                        ir_fixup *f = &s->a->preload[s->a->n_preload++];

                        f->at    = 0;
                        f->guest = (uint8_t)g;
                        f->host  = (uint8_t)(ALLOC_FIRST + h);
                        f->store = 0;
                } else {
                        fixup(s, (int)g, h, 0);
                }
        }

        s->owner[h] = (int8_t)g;
        s->used[h]  = 1;
        s->where[g] = (int8_t)h;
        s->last[h]  = (uint8_t)s->node;
        s->wrote[h] = 0;
        touch(s, h);
        return h;
}

/* Guest register 0 is never allocated: it reads zero and discards writes, so
 * a register holding it would hold nothing. */
static int8_t host_src(allocator *s, unsigned g)
{
        if (g == 0)
                return -1;
        return (int8_t)(ALLOC_FIRST + take(s, g, 1));
}

static int8_t host_dst(allocator *s, unsigned g)
{
        int h;

        if (g == 0)
                return -1;
        h = take(s, g, 0);
        s->dirty[h] = 1;
        s->wrote[h] = 1;
        return (int8_t)(ALLOC_FIRST + h);
}

/* Read and written both — the unaligned loads merge into their destination. */
static int8_t host_mod(allocator *s, unsigned g)
{
        int h;

        if (g == 0)
                return -1;
        h = take(s, g, 1);
        s->dirty[h] = 1;
        s->wrote[h] = 1;
        return (int8_t)(ALLOC_FIRST + h);
}

/* Working registers for a node that needs more than `r0` and `r1`.  Taken from
 * the least recently used end and NOT touched, so they stay first in line: a
 * scratch register holds nothing once the node is done, and the next node that
 * wants one should get the same one back. */
static void scratch(allocator *s, ir_node *p, int k)
{
        int r, got = 0;

        for (r = 0; r < ALLOC_N && got < k; r++) {
                int h, i;
                int taken = 0;

                for (h = 0; h < ALLOC_N; h++)
                        if (rank_of(s, h) == r)
                                break;
                if (h == ALLOC_N)
                        continue;

                /* Not one of this node's own operands, and not one already
                 * handed to it. */
                if ((p->hd >= 0 && p->hd == ALLOC_FIRST + h) ||
                    (p->hs >= 0 && p->hs == ALLOC_FIRST + h) ||
                    (p->ht >= 0 && p->ht == ALLOC_FIRST + h) ||
                    (p->hx >= 0 && p->hx == ALLOC_FIRST + h))
                        continue;
                for (i = 0; i < got; i++)
                        if (p->sc[i] == ALLOC_FIRST + h)
                                taken = 1;
                if (taken)
                        continue;

                evict(s, h);
                s->used[h] = 1;
                p->sc[got++] = (uint8_t)(ALLOC_FIRST + h);
        }
}

/* Everything the next block is entitled to assume.
 *
 * TWO PASSES, AND THE ORDER IS LOAD BEARING.  Every dirty value goes back
 * before any register is reloaded, because a pinned register's home may be the
 * only place some other guest register's value exists — reload it first and
 * that value is gone.
 *
 * Then the pinned assignment is put back where a block moved it, which is what
 * makes a bare `bra` into the next block legal: the join has no handshake, so
 * the state at every exit has to be the state at every entry.
 */
static void flush(allocator *s)
{
        int h;
#ifdef ALLOC_PINNED_LIVE
        /* Which pinned registers loop 1 declines to store.  Those are the ones
         * whose only current copy is the host register once this flush is
         * over, so the walk below has to put the dirty flag straight back --
         * `flush` also runs MID-BLOCK, and a register left clean here is one
         * the eviction path will drop. Same reasoning as the entry state in
         * `ir_allocate`. Fixing only the entry state and not this one leaves
         * the same hole for any block that flushes part-way through. */
        uint8_t live[ALLOC_N];
#endif

        for (h = 0; h < ALLOC_N; h++) {
                /* THE PINNED REGISTERS ARE LIVE IN HOST REGISTERS, and the
                 * state block's copy of one is allowed to be stale.
                 *
                 * The reference does not store a pinned register that still
                 * holds its own guest register: the next block assumes `PIN`
                 * and finds the value already there (`asm/regalloc.md`, the
                 * flush's loop 1 clearing `C[host]` when `A[host] ==
                 * B[host]`). We used to store it, on every block.
                 *
                 * Worth **-9.1% on Spyro** (3.75 -> 3.41 on the weighted
                 * census), against the 1.4% it was written off at when priced
                 * on a spin loop -- `gotchas.md` 305, and the reason `CENSUS=1`
                 * is not the same instrument as `WEIGHTS=`.
                 *
                 * WHAT MAKES IT CORRECT is in `emit.c`: `ld_guest_live` for a
                 * stub that wants a pinned register, and `emit_publish_pinned`
                 * at every crossing into C, because gcc knows nothing about
                 * the pinning. The reference needs neither -- its handlers are
                 * asm and read r7/r8/r11 directly (`asm/bios_hle.md` 5.5).
                 * ADD A CROSSING INTO C AND IT NEEDS THE PUBLISH:
                 * `grep emit_publish_pinned` is the check (296). */
#ifdef ALLOC_PINNED_LIVE
                live[h] = (uint8_t)(ir_pin[h] >= 0 &&
                                    s->owner[h] == ir_pin[h]);
                if (live[h])
                        s->dirty[h] = 0;
#endif
                if (s->owner[h] >= 0 && s->dirty[h]) {
                        fixup(s, s->owner[h], h, 1);
                        s->dirty[h] = 0;
                        s->wrote[h] = 0;
                        s->last[h] = (uint8_t)s->node;
                }
        }

        for (h = 0; h < ALLOC_N; h++)
                if (ir_pin[h] >= 0 && s->owner[h] != ir_pin[h])
                        fixup(s, ir_pin[h], h, 0);

        /* The entry state again, so a flush in the middle of a block leaves
         * the walk describing what the code actually holds. */
        for (h = 0; h < ALLOC_N; h++) {
                if (s->owner[h] >= 0)
                        s->where[s->owner[h]] = -1;
                s->owner[h] = ir_pin[h];
#ifdef ALLOC_PINNED_LIVE
                /* Unstored above, so still the only copy. A register that was
                 * NOT in its home was reloaded from the state block by the
                 * loop before this one, and is genuinely clean. */
                s->dirty[h] = live[h];
#endif
                s->last[h] = (uint8_t)s->node;
                s->wrote[h] = 0;
        }
        for (h = 0; h < ALLOC_N; h++)
                if (ir_pin[h] >= 0)
                        s->where[ir_pin[h]] = (int8_t)h;
}

/* ---------------------------------------------------------------- */
/* The walk                                                          */
/* ---------------------------------------------------------------- */

void ir_allocate(ir_node *ir, int n, ir_alloc *out)
{
        allocator s;
        int i;

        if (!lru_built)
                lru_build();

        out->n_preload = 0;
        out->n_fix = 0;

        /* A pinned register arrives holding its guest register, and it counts
         * as used: a preload into it would overwrite a value that is already
         * the right one.
         *
         * WHETHER IT ALSO AGREES WITH THE STATE BLOCK IS EXACTLY WHAT
         * `ALLOC_PINNED_LIVE` CHANGES, and getting this wrong loses a guest
         * register across two blocks. Without the flag the previous block's
         * flush stored it, so it is clean. WITH the flag that flush
         * deliberately skipped the store — so it arrives DIRTY: the host
         * register is the only current copy.
         *
         * Mark it clean anyway and the eviction path believes there is
         * nothing to save, drops the value, and the flush's reload pulls the
         * stale copy back out of memory. It needs one block to write a pinned
         * register and the NEXT to evict that host register, so no host suite
         * sees it — real blocks never spill (gotchas 33/34), the oracle's
         * blocks are independent, and cross-block state is the first item in
         * CLAUDE.md's list of what nothing covers. Spyro found it in one run
         * (`gotchas.md` 308).
         *
         * Dirty here costs nothing when the register is never evicted: the
         * flush's loop 1 clears the flag again because the register still
         * holds its own guest register, and no store is emitted. */
        for (i = 0; i < ALLOC_N; i++) {
                s.owner[i] = ir_pin[i];
#ifdef ALLOC_PINNED_LIVE
                s.dirty[i] = ir_pin[i] >= 0;
#else
                s.dirty[i] = 0;
#endif
                s.used[i] = ir_pin[i] >= 0;
                s.last[i] = 0;
                s.wrote[i] = 0;
        }
        for (i = 0; i <= GUEST_HI; i++)
                s.where[i] = -1;
        for (i = 0; i < ALLOC_N; i++)
                if (ir_pin[i] >= 0)
                        s.where[ir_pin[i]] = (int8_t)i;
        for (i = 0; i < RANK_BYTES; i++)
                s.rank[i] = (uint8_t)((2 * i + 1) << 4 | (2 * i));
        s.a = out;

        for (i = 0; i < n; i++) {
                ir_node *p = &ir[i];

                s.node = i;

                switch (p->op) {
                case IR_MOVE:
                        p->hs = host_src(&s, p->rs);
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_SET:
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_MFC0:
                        /* A coprocessor read carries the same shadow as a
                         * memory load, so it defers the same way: parked
                         * value, no destination claimed here. */
                        p->hd = p->defer ? -1 : host_dst(&s, p->rd);
                        break;

                case IR_ALU:
                case IR_SHIFT_REG:
                        p->hs = host_src(&s, p->rs);
                        p->ht = host_src(&s, p->rt);
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_ALU_IMM:
                        p->hs = host_src(&s, p->rs);
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_LOAD:
                        p->hs = host_src(&s, p->rs);
                        /* A deferred load parks its value in the state block
                         * and writes no register here -- the IR_TEMP_GET
                         * after the shadow instruction does that. Claiming a
                         * destination would tell the allocator rd is live
                         * from here, and the shadow instruction would then
                         * read the new value instead of the old one. */
                        p->hd = p->defer ? -1 : host_dst(&s, p->rd);
                        break;

                case IR_TEMP_GET:
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_SHIFT_IMM:
                        p->ht = host_src(&s, p->rt);
                        p->hd = host_dst(&s, p->rd);
                        break;

                case IR_MULDIV:
                        p->hs = host_src(&s, p->rs);
                        p->ht = host_src(&s, p->rt);
                        p->hd = host_dst(&s, GUEST_LO);
                        p->hx = host_dst(&s, GUEST_HI);
                        /* Division is a service call, and the pointer to it
                         * has to be somewhere neither operand is. */
                        if (p->sub == MD_DIV || p->sub == MD_DIVU)
                                scratch(&s, p, 1);
                        break;

                case IR_STORE:
                case IR_STORE_UN:
                        p->hs = host_src(&s, p->rs);
                        p->ht = host_src(&s, p->rt);
                        /* The aligned address and the word read back from it
                         * are both live across the merge. */
                        if (p->op == IR_STORE_UN)
                                scratch(&s, p, 2);
                        break;

                case IR_LOAD_UN:
                        p->hs = host_src(&s, p->rs);
                        /* Read-modify-write normally; when deferred it only
                         * READS rd -- the merge is parked and IR_TEMP_GET
                         * writes the register afterwards. */
                        p->hd = p->defer ? host_src(&s, p->rd)
                                         : host_mod(&s, p->rd);
                        scratch(&s, p, 1);
                        break;

                case IR_COND:
                        p->hs = host_src(&s, p->rs);
                        if (p->sub == CC_EQ || p->sub == CC_NE)
                                p->ht = host_src(&s, p->rt);
                        break;

                case IR_CAPTURE:
                case IR_MTC0:
                case IR_MTC2:
                case IR_LWC2:
                case IR_SWC2:
                        p->hs = host_src(&s, p->rs);
                        break;

                case IR_MFC2:
                        /* Deferred, it claims no destination -- the same
                         * shadow an ordinary load has. */
                        p->hd = p->defer ? -1 : host_dst(&s, p->rd);
                        break;

                case IR_STOP:
                        /* It leaves the block through a service routine, so
                         * everything the guest can see has to be in the state
                         * block before it goes. */
                        flush(&s);
                        break;

                default:                /* IR_JUMP, IR_RFE — no operands */
                        break;
                }
        }

        s.node = n;
        flush(&s);

        /* Charging by last use means the list comes out unordered — an
         * eviction late in the block can name an early node.  The emitter
         * walks it once alongside the nodes, so it is put back in order here,
         * stably: two entries on the same node keep the order they were made
         * in, which is what makes a writeback precede the reload that took
         * its register. */
        for (i = 1; i < out->n_fix; i++) {
                ir_fixup f = out->fix[i];
                int j = i;

                while (j > 0 && out->fix[j - 1].at > f.at) {
                        out->fix[j] = out->fix[j - 1];
                        j--;
                }
                out->fix[j] = f;
        }
}
