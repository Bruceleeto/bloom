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
#include "fgl_state.h"

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

/* THE DEADLINE SCHEDULER, OFF BY DEFAULT.  bloop's scheme: generated code
 * stops counting cycles and the guest clock advances in lumps at a gated hook.
 *
 * r14 keeps its name and its gate -- `cmp/pl FGL_R_CYCLE` at every link site
 * is untouched, because that test is still the only thing between a linked
 * chain and running for ever.  What deadline mode removes is the two
 * instructions per block epilogue that MOVED it, and the mid-block charge that
 * paid devices in advance.  Something else has to move r14 instead, and on the
 * Dreamcast that is TMU2 writing the pinned register; here it is a step
 * counter in the dispatcher, which is deterministic and therefore comparable.
 *
 * Build with -DFGL_DEADLINE=1.  Everything downstream -- the shims' cycle
 * round-trip, IR_EXIT's reconciliation, `fgl_service_events` -- is unchanged
 * and does not know which mode it is in. */
#ifndef FGL_DEADLINE
#define FGL_DEADLINE 0
#endif

/* BLEEM'S BLOCK HEADER, off by default and only meaningful with FGL_DEADLINE.
 *
 * bleem does not test the budget at the places a block LEAVES; it tests it
 * where a block is ENTERED, and the block table points four bytes into the
 * block so the test is what every entry lands on
 * (`asm/dynarec_services.md` 4.9, `asm/recompiler_backend.md` "Block layout"):
 *
 *      H+0  .long guest_pc           the header word (bleem: H+0 too)
 *      E+0  mov.l @(disp,gbr),r0     the hook            (E = H+4)
 *      E+2  jsr   @r0                PR = E+6, so the header is at PR-10
 *      E+4  tst   r14,r14            <- THE ENTRY POINT, and the delay slot
 *      E+6  bt    -5                 -> E+0, taken when r14 == 0
 *      E+8  body
 *
 * The header word is why the hook can be a `jsr`: the dispatcher derives
 * the block's guest pc from PR instead of needing it in r2, so a linked
 * edge never loads r2 (the target lives in the link site's literal) and
 * the hook still knows where the slice stopped.
 *
 * Two register-only instructions on the common path and no memory reference,
 * against our epilogue's `cmp/pl` + `bf` per link ARM.  It needs r14 to be a
 * flag rather than a countdown, which is what deadline mode makes it: the
 * charge clamps at the target, so r14 reaches exactly zero and never passes
 * it.  With blocks still counting, r14 steps over zero and the gate is missed.
 */
#ifndef FGL_ENTRY_HOOK
#define FGL_ENTRY_HOOK 0
#endif

#if FGL_ENTRY_HOOK && !FGL_DEADLINE
#error "FGL_ENTRY_HOOK needs FGL_DEADLINE: r14 must be a flag, not a countdown"
#endif

/* THE LAZY CLOCK, off by default and only meaningful with FGL_DEADLINE.
 *
 * With it, nothing charges `psxRegs.cycle` at a crossing into C.  Generated
 * code retires SH-4 instructions and a meter counts them (here the
 * interpreter's step count; on the Dreamcast PRFC1); the guest clock is
 * computed from that meter when -- and only when -- something asks what time
 * it is.  Devices ask through `psx_clock_now()` (r3000a.h), which charges the
 * work accrued so far, clamped at `next_interupt` so a read can never see time
 * past an event that has not fired; the hook asks through `psx_clock_settle()`,
 * unclamped, and then drains what is due.  An MMIO access that never asks costs
 * the clock nothing, which is the whole point: the per-crossing charge is the
 * tax that made bloom's deadline build slower than its counting build. */
#ifndef FGL_LAZY_CLOCK
#define FGL_LAZY_CLOCK 0
#endif

#if FGL_LAZY_CLOCK && !FGL_DEADLINE
#error "FGL_LAZY_CLOCK needs FGL_DEADLINE: blocks must not count"
#endif

/* LINK SITES ARE 4-ALIGNED, so that patching one is a single longword store.
 *
 * The patch rewrites a site's two instruction words while a block may be
 * executing them -- the recompiler thread tears links down while emitted code
 * runs.  A 4-aligned site is one aligned store and the fetcher cannot catch it
 * half-done; a 2-aligned site takes two halfword stores and the window between
 * them pairs the old first word with the new second word, which on SH-4 jumps
 * to the stub with a stale PR.  One `nop` in half the sites is the price, and
 * it is the same price in both trees so their census streams stay comparable.
 *
 * 0 restores the shorter, unaligned layout.  Only safe where nothing executes
 * emitted code while it is being patched. */
#ifndef SH4_LINK_SITE_ALIGN
#define SH4_LINK_SITE_ALIGN 1
#endif

/* How far into a block its table entry points. */
#if FGL_ENTRY_HOOK
#define FGL_ENTRY_OFF 8u
#else
#define FGL_ENTRY_OFF 0u
#endif

#define FGL_MAX_LITERALS 64
#define FGL_MAX_LABELS 32
#define FGL_MAX_LREFS  64

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

	/* The computed COP2 reads, same call shape again.  See ir.h on
	 * IR_MFC2_C. */
	uint32_t mfc;

	/* Returning from an exception.  `f(ignored, the state block)`, the
	 * same shape again.  See ir.h on IR_RFE. */
	uint32_t rfe;

	/* THE ADDRESS OF `psxCP2CtrlGen`, and why a code generator has to
	 * know about it.
	 *
	 * `gte_fpu.c` does not read the COP2 control file on every command.
	 * It caches what it derives from it -- the three 3x3 matrices in
	 * float form, and the projection offsets -- and decides whether a
	 * cache is still good by comparing a generation counter
	 * (`gte_fpu.c:342,391,472,502,976`).  `lightrec_ctc2` bumps that
	 * counter on every control write (lightrec.c:614), which is the only
	 * thing that ever invalidates them.
	 *
	 * A back end that writes the control register and not the counter
	 * therefore transforms every vertex for the rest of the run with the
	 * matrix and the projection offsets that happened to be loaded before
	 * the first GTE command.  See ir.h on IR_MTC2. */
	uint32_t cp2_ctrl_gen;

	/* DEBUG: called when an emitted store computes a wild address. */
	uint32_t wild;

	/* The C body that runs one COP2 command, given the guest instruction
	 * word.  Resolved AT COMPILE TIME -- the command is a constant in the
	 * block, so there is no runtime dispatch and nothing decodes it twice.
	 * Returns 0 for a command the hardware ignores, for which the right
	 * amount of code is none.  `user` is passed back untouched so a host
	 * test can hang its own table off it. */
	uint32_t (*gte_body)(void *user, uint32_t op);
	/* The LEAF for a command, or 0 for "through the shim".  Returns the
	 * gte_fpu.h GTE_LEAF_* index shifted left one, with bit 0 set when
	 * the routine also wants the op word in r1 (bleem's argument flag);
	 * NCLIP does not.  May be NULL.
	 *
	 * The INDEX and not the address, because the address lives in a GBR
	 * slot -- FGL_SVC_GTE_LEAF -- so the site is `mov.l @(disp,gbr),r0;
	 * jsr @r0` and drags no pool word.  `gte_leaf_tab` is what gets
	 * copied into those slots at init; entry i is leaf i's address, and
	 * entry 0 is never used. */
	uint32_t (*gte_leaf)(void *user, uint32_t op);
	uint32_t gte_leaf_tab[FGL_GTE_LEAF_N];
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

	/* --- MID-BLOCK CYCLE CHARGE ---
	 *
	 * A device register is answered from the guest clock, so the clock has
	 * to be current when the block reaches one.  The epilogue charge is
	 * too late: a timer read taken with the block's cycles still unspent
	 * is answered as though no time had passed since the block started,
	 * and the guest gets a number that is simply wrong -- not a rounding
	 * difference, a different value.
	 *
	 * BLOOM DOES NOT HAVE THIS, and that is the one place the two trees
	 * deliberately differ: bloom has no interpreter to disagree with, and
	 * this tree's whole job is agreeing with one.  Do not drop it when
	 * syncing from bloom again.
	 *
	 * `charge_pc` is the first guest instruction of the block, so the cost
	 * up to any node is the distance from it.  `charged` is what has
	 * already been taken off mid-block, which the epilogue then owes less
	 * of. */
	uint32_t charge_pc;
	unsigned charged;

#define FGL_MAX_BLOCK_OPS 128u

	/* How many times the pool had to be flushed mid-block. Zero for
	 * anything short; a block that needs several is one whose lowering is
	 * materialising far too many constants. */
	int pool_flushes;

	/* --- DELAY-SLOT FILLING ---
	 *
	 * `slot_floor` is the first word index the filler may lift from: no
	 * instruction at or above it is separated from the branch by a label,
	 * data, or anything the filler cannot decode.  Every forward-branch
	 * patch, every pool, every link site raises it to `here`.  See
	 * `emit_delayed` in emit.c. */
	int slot_floor;
	int cond_arms;          /* IR_COND: leave r2 to the epilogue's arms  */
	int link_last;          /* IR_JUMP is the last node: it will be linked */
	int cond_saved;         /* T parked in FGL_AT_TSAVE across the slot  */
	int xfer_at;            /* index of the transfer the epilogue links, or -1 */
	int slots_filled;
	/* First word the peephole pass has not looked at yet (emit.c,
	 * peep_hoist).  Advanced at every node boundary. */
	int peep_at;

	/* --- REGION: SEVERAL BASIC BLOCKS IN ONE CODE BLOCK ---
	 *
	 * lightrec's block is a whole loop body; fgl used to compile only its
	 * first basic block and let every internal branch leave through the
	 * dispatcher (or a link site) to a block of its own.  A region is the
	 * lightrec block compiled as ONE code block: every basic block in it
	 * is a label, and a transfer whose target is a label becomes a host
	 * `bra` to it instead of an exit.  See fgl_compile_block.
	 *
	 * `lbl[]` is set before emission (`fgl_region_begin`); each `at` is
	 * filled when `fgl_emit` reaches that basic block.  A forward edge is
	 * a `bra` recorded in `lref[]` and patched when its label is defined;
	 * a backward edge is patched on the spot and carries the budget test,
	 * because it closes a loop.  `may_exit` says the current basic block
	 * crossed into C, after which r14 may have been zeroed to request a
	 * collection -- a forward edge then tests the budget too. */
	int region;
	struct { uint32_t pc; int at; } lbl[FGL_MAX_LABELS];
	int n_lbl;
	/* `cond`: the site is a `bt`/`bf` (8-bit displacement) rather than a
	 * `bra`; a conditional's fallthrough arm branching straight to the
	 * next basic block (emit.c, the epilogue). */
	struct { int site; int lbl; int cond; } lref[FGL_MAX_LREFS];
	int n_lref;
	int cur_lbl;            /* the label defined last                    */
	/* 1 + the word index of this block's dispatcher exit when one exists
	 * that is reached only by budget-miss branches, 0 otherwise.  Later
	 * basic blocks aim their `bf` at it instead of emitting their own. */
	int exit_at1;
	int local_last;         /* the epilogue's IR_JUMP targets a label   */
	int may_exit;
	/* WHAT THE FIRST PASS MEASURED FOR THE SECOND (emit.c, "TWO SHAPES
	 * THE FIRST PASS DECIDES").  One byte of FGL_HINT_* per label, indexed
	 * like `lbl[]`.  Pass 1 writes `hint_out` and reads nothing; pass 2
	 * reads `hint_in` and writes nothing.  Both NULL outside a region. */
	uint8_t *hint_out;
	const uint8_t *hint_in;
	int arm_site[FGL_MAX_LABELS];   /* pass 1: the taken arm's `bf` site   */
	int arm_lbl[FGL_MAX_LABELS];    /* pass 1: the label that arm goes to  */
	int tsave_at[FGL_MAX_LABELS];   /* pass 1: word after the T park       */
	uint32_t block_pc;      /* the lightrec block this code is compiled from */
} fgl_emitter;

#define FGL_HINT_BT    1  /* taken arm: a `bt` reaches the label      */
#define FGL_HINT_TLIVE 2  /* nothing across the slot writes T          */

/* After pass 1: the label-distance hints into e->hint_out. */
void fgl_region_hints(fgl_emitter *e);

/* Declare the labels a region will define, in emission order.  Call after
 * fgl_init on every pass.  Then one fgl_emit per basic block; then
 * fgl_region_end returns the number of edges left unresolved (0 = good). */
void fgl_region_begin(fgl_emitter *e, const uint32_t *pcs, int n);
int  fgl_region_end(const fgl_emitter *e);
/* The word offset of label `i` after emission, for the profiler's ops map. */
int  fgl_region_label_at(const fgl_emitter *e, int i);

/* EMITTER SITES, for the provenance tagmap in `sh4_codegen`.
 *
 * The split exists to answer one question: half of every instruction fgl
 * emits is a load or a store, and nobody knows how much of that is block-entry
 * preloads, mid-block spills, or the block-exit flush.  Those are the three
 * separate ids below; everything else is here so they can be read against a
 * whole. */
enum {
	FGL_TAG_OTHER = 0,
	FGL_TAG_PRELOAD,        /* a->preload[]: guest regs loaded at entry   */
	FGL_TAG_SPILL,          /* a->fix[] mid-block: the allocator evicted  */
	FGL_TAG_FLUSH,          /* a->fix[] after the last node: the exit     */
	FGL_TAG_BODY,           /* the guest instruction's own lowering       */
	FGL_TAG_MEM,            /* guest load/store: address calc and access  */
	FGL_TAG_POOL,           /* the literal pool, and loads from it        */
	FGL_TAG_CYCLE,          /* the cycle charge                           */
	FGL_TAG_EPILOGUE,       /* publish pc, dispatch jump                  */
	FGL_TAG_LINK,           /* link sites and their data                  */
	FGL_TAG_MASK,           /* `and r13,addr`: the guest address mask     */
	FGL_TAG_PIN,            /* pin fixup at the block-end flush           */
	FGL_TAG_PINMID,         /* pin fixup at a MID-BLOCK flush             */
	FGL_TAG_HOOK,           /* bleem's entry hook: tst r14,r14 / bt       */
	FGL_TAG_HWL,           /* device load shim                          */
	FGL_TAG_HWS,           /* device store shim                         */
	FGL_TAG_RW,            /* the generic unproven-address fallback     */
	FGL_TAG_INV,            /* the store's code-invalidation sequence     */
	FGL_TAG_N
};

extern const char *const fgl_tag_name[FGL_TAG_N];

/* Where a block's tag bytes live, or NULL when not profiling. */
uint8_t *fgl_tags_at(void *code);
void fgl_ops_at(void *code, unsigned n_ops);

void     fgl_init(fgl_emitter *e, void *buf, uint32_t size, uint32_t base);

/* Supply the service addresses.  Separate from `fgl_init` so a harness with
 * no services keeps compiling and simply cannot emit the nodes that need
 * them.  `t` is borrowed, not copied. */
void     fgl_set_targets(fgl_emitter *e, const fgl_targets *t);
uint32_t fgl_size(const fgl_emitter *e);

/* Emit one already-decoded, already-allocated block. Returns its entry
 * address, or 0 if the emitter overflowed or met something it cannot lower. */
/* DUMB MODE: THE EMITTER WITH ITS TWO CLEVER PARTS SWITCHED OFF.
 *
 * fgl does two things beyond turning one guest instruction into SH-4: it
 * keeps guest registers in host registers across a run of instructions, and
 * it compiles a run of instructions as one block.  Both are where the hard
 * bugs live -- eviction, writeback, spill, delay-slot placement, block
 * boundaries -- and neither is needed for a correct machine, only a fast one.
 *
 * These two switches remove them independently, so a fault can be bisected to
 * one half of the emitter in two runs rather than argued about:
 *
 *   FGL_DUMB_REGS=1     no register allocation.  Every operand is loaded from
 *                       the state block and every result stored straight back
 *                       to it, exactly as the emitter already does when the
 *                       allocator runs out -- so this is the existing memory
 *                       path taken always, not a new one.
 *   FGL_DUMB_BLOCKS=1   one guest instruction per block, plus the delay slot
 *                       when that instruction is a transfer (splitting those
 *                       two would change what the machine does, not just how
 *                       fast it does it).
 *
 * Both are slow -- slower than the C interpreter, most likely -- and neither
 * is a configuration to ship.  They are instruments.  What they do NOT touch
 * is cycle accounting or interrupt delivery, so a fault that survives both is
 * in an instruction template or in the runtime, not in block formation.
 */
#ifndef FGL_DUMB_REGS
#define FGL_DUMB_REGS 0
#endif
#ifndef FGL_DUMB_BLOCKS
#define FGL_DUMB_BLOCKS 0
#endif

/* Cycles added to every block's charge.  Zero is the real clock; a
 * non-zero value is the A/B described at the charge site in emit.c. */
#ifndef FGL_CYCLE_BIAS
#define FGL_CYCLE_BIAS 0
#endif

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
