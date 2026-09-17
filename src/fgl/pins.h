/* The pinned register assignment, shared by the C and the .S sources.
 *
 * A pinned guest register arrives in a host register and leaves the same way:
 * no load at entry, no store at exit.  Generated code reaching generated code
 * is free (the dispatcher touches only r0-r2).  Anything that reaches C must
 * PUBLISH the pins into the state block first and RELOAD them after; a missed
 * one hands C a stale register.  FGL_NUM_PINS 0 turns pinning off: the A/B
 * that isolates a pinning bug from an emitter bug. */

#ifndef FGL_PINS_H
#define FGL_PINS_H

/* The guest-file base register: `mov.l @(disp,Rm),Rn` reaches 16 words, so
 * one register pointing at the state block makes $0-$15 a single instruction
 * to any destination instead of a GBR load plus a `mov` off R0.  Its value
 * is the state block pointer, so a C callee that clobbers r3 (caller-saved)
 * is undone with `stc gbr, r3`, never the stack.  FGL_GBASE 0 is the A/B. */
#ifndef FGL_GBASE
#define FGL_GBASE 1
#endif

#if FGL_GBASE
#define FGL_R_GBASE     3
#define FGL_GBASE_MAX   16      /* guest registers reachable: $0..$15 */
#endif

/* The second window: base at word 16 for $16-$31.  Its register is r9, which
 * was PIN5 ($at), so FGL_NUM_PINS drops to five.  Not r4: the GTE leaf owns
 * r0-r6 and destroys it.  r9 is a hole in the allocator's pool (ALLOC_SKIP),
 * not a shorter range.  FGL_GBASE2 0 is the A/B. */
#ifndef FGL_GBASE2
#if FGL_GBASE
#define FGL_GBASE2 1
#else
#define FGL_GBASE2 0            /* pointless without the first window */
#endif
#endif

#if FGL_GBASE2 && !FGL_GBASE
#error "FGL_GBASE2 needs FGL_GBASE: the low window is the cheaper half"
#endif

#if FGL_GBASE2
#define FGL_R_GBASE2    9       /* PIN5's old host; see above on why not r4 */
#define FGL_GBASE2_LO   16      /* guest registers reachable: $16..$31 */
#define FGL_GBASE2_MAX  32
#endif

/* How many of the four below are live.  0 disables pinning entirely.
 * FGL_GBASE2 spends PIN5 ($at, r9) on the second guest-file base -- see
 * above -- so the default is five whenever it is on. */
#ifndef FGL_NUM_PINS
#if FGL_GBASE2
#define FGL_NUM_PINS 5
#else
#define FGL_NUM_PINS 6
#endif
#endif

/* The sixth pin and the second base want the same register, so there is no
 * ordering that gives both. */
#if FGL_GBASE2 && FGL_NUM_PINS > 5
#error "FGL_GBASE2 takes r9, which is PIN5's host: FGL_NUM_PINS must be <= 5"
#endif

#if FGL_NUM_PINS > 6
#error "FGL_NUM_PINS is larger than the table below"
#endif

/* host register, guest register -- bloop's assignment, register for register.
 * Host must be in ALLOC_FIRST..ALLOC_FIRST+9.  Dropping FGL_NUM_PINS below six
 * drops from the END of this list, so the order is bloop's priority: the ones
 * most worth keeping are first. */
#define FGL_PIN0_HOST   10
#define FGL_PIN0_GUEST  29      /* $sp */
#define FGL_PIN1_HOST   11
#define FGL_PIN1_GUEST   2      /* $v0 */
#define FGL_PIN2_HOST   12
#define FGL_PIN2_GUEST   3      /* $v1 */
#define FGL_PIN3_HOST    7
#define FGL_PIN3_GUEST   4      /* $a0 */
#define FGL_PIN4_HOST    8
#define FGL_PIN4_GUEST   5      /* $a1 */
#define FGL_PIN5_HOST    9
#define FGL_PIN5_GUEST   1      /* $at */

/* The allocator's per-slot view of the same table.  ALLOC_PINNED_LIVE makes
 * the flush treat a pinned register as live in its host register rather than
 * as something to store; the two must move together (alloc.c). */
#if FGL_NUM_PINS > 0
#define ALLOC_PINNED_LIVE 1
#endif

#endif /* FGL_PINS_H */
