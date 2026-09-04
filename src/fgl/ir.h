/* The IR the three passes hand between them.
 *
 * One node per operation, flat array, no graph.  A block is decoded into it in
 * one linear pass and emitted from it in another; nothing walks it backwards
 * and nothing rewrites its shape.
 *
 * TWO THINGS ABOUT THE ORDER ARE NOT OBVIOUS.
 *
 * A delay slot appears in the array BEFORE the transfer it belongs to, because
 * that is the order a sequential emitter needs: the slot's code runs first on
 * the machine, so it is emitted first.
 *
 * And a transfer is split.  Everything that has to read guest state as it
 * stood *before* the delay slot — a branch's comparison, a jump register's
 * target, a link's return address — is its own node placed ahead of the slot.
 * What is left needs no guest state at all: the final PC is already in a host
 * register, and the block epilogue publishes it.  So there is no "branch" node
 * in this IR, only the nodes that compute where the block goes.
 *
 * Guest register numbering is the state block's: 0-31 GPRs, 32 LO, 33 HI, so a
 * register number is already its GBR displacement.
 */

#ifndef IR_H
#define IR_H

#include <stdint.h>

enum {
        IR_MOVE,        /* rd = rs                                          */
        IR_SET,         /* rd = imm                                         */
        IR_ALU,         /* rd = rs op rt                                    */
        IR_ALU_IMM,     /* rd = rs op imm  (imm already widened to 32 bits) */
        IR_SHIFT_IMM,   /* rd = rt shift imm                                */
        IR_SHIFT_REG,   /* rd = rt shift (rs & 31)                          */
        IR_MULDIV,      /* LO,HI = rs op rt                                 */
        IR_LOAD,        /* rd = mem[rs + imm]                               */
        IR_STORE,       /* mem[rs + imm] = rt                               */
        IR_LOAD_UN,     /* rd = merge(rd, mem[rs + imm])   LWL / LWR        */
        IR_STORE_UN,    /* mem[rs + imm] = merge(mem, rt)  SWL / SWR        */

        /* The three that decide where the block goes.  Each leaves the next
         * guest PC in the exit register; all of them sit ahead of the delay
         * slot, so they see guest state as the transfer saw it. */
        IR_COND,        /* exit = <test rs,rt> ? imm : imm2                 */
        IR_CAPTURE,     /* exit = rs                                        */
        IR_JUMP,        /* exit = imm                                       */

        IR_MFC0,        /* rd = cop0[imm]  */
        IR_MTC0,        /* cop0[imm] = rs  */

        /* MTC0 TO STATUS OR CAUSE, WHICH C HAS TO PERFORM.  `imm` is the
         * guest instruction word.
         *
         * Bit 16 of Status isolates the data cache, and while it is set the
         * PSX's stores go to the cache instead of to RAM.  The BIOS flushes
         * its cache exactly that way -- `mtc0 t1,SR` with t1 = 0x10000, then
         * four kilobytes of `sw zero`, then `mtc0 zero,SR` -- so a code
         * generator that treats the write as an ordinary state store lets
         * those four kilobytes land in guest RAM and erases the kernel's own
         * A0/B0/C0 vector stubs.  The emulator models the isolation by saving
         * and restoring that memory (`lightrec_enable_ram`, plugin.c), and it
         * only learns to do so if the write reaches `lightrec_mtc0`.
         *
         * lightrec makes the same call for the same reason and inlines the
         * write only where it can prove the block is running from RAM through
         * kuseg or kseg0 (emitter.c, `block_uses_icache`), which BIOS code
         * never is.  fgl does not carry that proof yet, so it calls C for
         * every Status and Cause write; they occur in kernel and interrupt
         * code and not in any loop that matters.
         *
         * C reads the source register out of the state block and can set exit
         * flags, so the allocation pass flushes around it exactly as for
         * IR_RW, AND THE BLOCK ENDS HERE -- a newly unmasked interrupt has to
         * be delivered before the next instruction runs. */
        IR_MTC_C,
        IR_RFE,         /* pop the interrupt-enable stack */

        /* COP2, the geometry unit.  `imm` is the state-block displacement of
         * the coprocessor register, already resolved by the decoder — data and
         * control are one contiguous file, so there is no second opcode and
         * nothing downstream has to know which half it is in. */
        IR_MFC2,        /* rd = cop2[imm]              */
        IR_MTC2,        /* cop2[imm] = rs              */
        IR_LWC2,        /* cop2[imm] = mem[rs + imm2]  */
        IR_SWC2,        /* mem[rs + imm2] = cop2[imm]  */
        IR_GTE,         /* run command `imm` -- the whole guest word */

        /* The other half of a deferred load: rd = the parked value. See
         * `defer` below. */
        IR_TEMP_GET,

        /* A MEMORY ACCESS C HAS TO PERFORM, `imm` being the guest
         * instruction word.
         *
         * lightrec's optimiser proves a region for most accesses and fgl
         * lowers those to two instructions.  When it cannot prove one, the
         * access may reach anything -- RAM, a device, an unmapped hole -- and
         * the only thing that knows which is lightrec's map dispatch.  So the
         * whole access goes to C, address arithmetic included.
         *
         * A SEPARATE NODE RATHER THAN A FLAG ON IR_LOAD, for a reason that is
         * about the oracle rather than about taste.  `FGL_IO_UNKNOWN` is zero,
         * and zero is also what the raw-word decoder leaves on every node it
         * builds, because it has no optimiser to learn a region from.  A test
         * for "unknown region" would therefore fire on every access the oracle
         * has ever compared, and turn its twenty thousand blocks into C calls
         * without failing anything.  Only `front.c` builds this node, so the
         * decision is made where the information is and the oracle cannot be
         * quietly redefined by it.
         *
         * C reads the base register and writes the destination IN THE STATE
         * BLOCK, so every guest register the allocator is holding must be
         * written back before this node and reloaded after it.  That is what
         * `flush` in alloc.c does, and why this node claims no operands. */
        IR_RW,

        IR_STOP,        /* SYSCALL / BREAK — sub is the exception code */

        /* LEAVE FOR C, AND SAY WHY.  `imm` is lightrec's exit flag, `imm2` the
         * guest PC to resume at.
         *
         * Not an error path.  The emulator delivers its HLE BIOS calls by
         * marking an opcode unknown and letting the block exit with
         * LIGHTREC_EXIT_UNKNOWN_OP, so a code generator with no way to express
         * this cannot run the BIOS at all -- the block simply fails to build.
         *
         * The mechanism is lightrec's and it is indirect: there is no return
         * instruction here. The block reconciles the absolute cycle counters,
         * ZEROES THE DELTA so the dispatcher's one budget test fires the
         * moment it regains control, and leaves normally. */
        IR_EXIT
};

enum { ALU_ADD, ALU_SUB, ALU_AND, ALU_OR, ALU_XOR, ALU_NOR, ALU_SLT, ALU_SLTU };
enum { SH_LL, SH_RL, SH_RA };
enum { MD_MULT, MD_MULTU, MD_DIV, MD_DIVU };
enum { MEM_B, MEM_BU, MEM_H, MEM_HU, MEM_W };
enum { UN_LWL, UN_LWR, UN_SWL, UN_SWR };
enum { CC_EQ, CC_NE, CC_LEZ, CC_GTZ, CC_LTZ, CC_GEZ };

typedef struct {
        uint8_t  op;
        uint8_t  sub;
        uint8_t  rd, rs, rt;
        uint32_t imm;
        uint32_t imm2;          /* IR_COND's not-taken PC */
        uint32_t pc;            /* the guest instruction this came from */

        /* Filled in by the allocation pass, and by nothing else.  A host
         * register number for an operand that is in one, or -1 for an operand
         * still in the state block — so a node emitted before the allocator
         * has run and a node the allocator ran out of registers for take the
         * same path. */
        int8_t   hd, hs, ht;
        int8_t   hx;            /* IR_MULDIV writes two: hd is LO, hx is HI */
        uint8_t  sc[2];         /* scratch the node may clobber, 0 for none */

        /* THE LOAD SHADOW, WHERE ROTATION CANNOT REACH IT.
         *
         * A load's register write is delayed by one instruction; its MEMORY
         * READ IS NOT. nocash's spec is explicit -- "every load reads through
         * to memory and halts the CPU until the data arrives", and only "the
         * target register isn't updated until the next opcode has completed"
         * (`tools/docs/cpuspecifications.md`, Caution - Load Delay).
         *
         * Moving the whole load node after the shadow instruction models the
         * register half and breaks the memory half, which is wrong exactly
         * when the shadow instruction is a store to the address just read:
         *
         *      lw  r11, 0x1f0(r29)
         *      sb  r11, 0x1f0(r29)     <- reads old r11, writes that word
         *
         * The load must read memory BEFORE that store and write r11 AFTER it.
         * So instead of rotating, the load is split: `defer` says "read
         * memory, park the value, write nothing", and an IR_TEMP_GET placed
         * after the shadow instruction does the register write. The allocator
         * then sees the truth -- the load writes nothing, the shadow
         * instruction reads the old rd, and the register is written later --
         * so it preloads and evicts correctly with no special case.
         *
         * The parking slot is the state block's temp word, not a register:
         * a scratch register would have to stay live across a node, which the
         * allocator does not model, and this pair is rare enough that two
         * extra instructions cost nothing. */
        uint8_t  defer;

        /* WHAT THE FRONT END KNEW AND THE EMITTER CANNOT WORK OUT.
         *
         * Zero on every node the raw-word decoder builds, and zero is the
         * conservative answer everywhere it is read -- "assume nothing, emit
         * the general form".  Only `front.c` sets these, from what lightrec's
         * optimiser proved about the block, and the emitter is entitled to
         * treat a nonzero value as fact.
         *
         * `io` is the memory region a load or store reaches (FGL_IO_*); `hint`
         * carries the rest of the per-node facts as FGL_H_* bits. */
        uint8_t  io;
        uint8_t  hint;
} ir_node;

/* Memory regions, as the optimiser labels them. UNKNOWN is not a region but
 * an admission, and it is what the raw-word decoder produces for everything. */
enum { FGL_IO_UNKNOWN, FGL_IO_DIRECT, FGL_IO_HW, FGL_IO_RAM,
       FGL_IO_BIOS, FGL_IO_SCRATCH, FGL_IO_DIRECT_HW };

/* Per-node facts that are not a region. */
enum {
        FGL_H_NO_MASK   = 1u << 0,   /* address is already in range         */
        FGL_H_NO_LO     = 1u << 1,   /* mul/div: nothing reads LO           */
        FGL_H_NO_HI     = 1u << 2,   /* mul/div: nothing reads HI           */
        FGL_H_NO_DIV_CHK = 1u << 3,  /* divisor is provably nonzero         */
        FGL_H_ALIGN     = 1u << 4,   /* LWL/LWR/SWL/SWR byte offset known;
                                      * the offset itself is in `sub`'s
                                      * upper bits -- see front.c */
        FGL_H_NO_INV    = 1u << 5    /* the optimiser proved this store cannot
                                      * land on code, so it need not clear the
                                      * block table -- see emit_invalidate */
};

/* DOES THIS STORE HAVE TO CLEAR THE BLOCK TABLE?
 *
 * Only a store that can land in RAM can land on code, and only if the
 * optimiser did not already prove otherwise.  The allocation pass and the
 * emitter have to agree exactly -- one handing out a scratch register the
 * other does not use is merely wasteful, the other way round is a store
 * through register -1 -- so the question is asked in one place.
 *
 * FGL_IO_SCRATCH and the two device classes are excluded because nothing
 * compiled ever lives there, and FGL_IO_UNKNOWN never reaches the emitter as
 * a store: front.c turns it into IR_RW and C invalidates for itself.  That is
 * also why the oracle, whose raw front end leaves every node UNKNOWN, emits
 * none of this and measures the same IPI it always did. */
static inline int ir_store_invalidates(const ir_node *p)
{
        if (p->hint & FGL_H_NO_INV)
                return 0;
        return p->io == FGL_IO_RAM || p->io == FGL_IO_DIRECT;
}

/* A block is at most 32 guest instructions, and a transfer expands to at most
 * three nodes, so this has headroom over anything the decoder can produce. */
#define IR_MAX_INSNS 32
#define IR_MAX_NODES 40

/* Decode one basic block: guest words from `words`, first instruction at guest
 * address `pc`.  Stops after the first control transfer and its delay slot.
 *
 * `words` MUST HOLD IR_MAX_INSNS + 1 WORDS.  A transfer in the last slot has
 * its delay slot in the one after, and the decoder reads it -- so a block can
 * cover 33 guest instructions even though it decodes 32.  In the emulator the
 * window is guest RAM and the extra word is simply the next instruction; a
 * harness that hands over exactly 32 reads off the end of its own array.
 * Returns the node count, which is not the instruction count — folding removes
 * nodes and a transfer adds them. */
int ir_decode(const uint32_t *words, uint32_t pc, ir_node *out, int max);

/* THE ONE LOAD-SHADOW CASE THE DECODER CANNOT COVER, MADE COUNTABLE.
 *
 * A load in a branch delay slot has its shadow in the *target* block, so the
 * reordering that models every other shadow cannot reach it — plan.md §4.5,
 * §3.4 row 40.  It is declined on cost, and the risk of declining it is not
 * that it fires often but that it fires SILENTLY: a wrong value, no signal,
 * and a week spent looking somewhere else.
 *
 * These two make it visible from the compile pass, which runs once per block
 * and can afford to look.  The count has to be taken from the LOAD's side, not
 * the shadow's: a block is compiled once and entered from any number of
 * predecessors, so "was a load left pending by whoever jumped here" is not a
 * question the target block can answer.  Ending in a delay-slot load is a
 * static property of one block, and where the terminator is direct its
 * successors are known right there.
 *
 * `ir_delay_slot_load` returns the guest register such a block leaves in
 * flight, or 0.  `ir_reads` says whether an instruction reads a register, so
 * the caller can peek at a successor's first word and decide. */
unsigned ir_delay_slot_load(const uint32_t *words);
int ir_reads(uint32_t insn, unsigned r);

/* How many guest instructions the block starting at `words` covers, its
 * terminator and delay slot included.  Not derivable from the node count,
 * because folding removes nodes and a transfer adds them — and a differential
 * test against an interpreter needs to know exactly how far one block gets. */
int ir_block_length(const uint32_t *words);

#endif /* IR_H */
