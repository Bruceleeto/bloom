/* Lowering lightrec's optimised block into fgl IR.
 *
 * The loop lives here; the per-instruction mapping does not.  Every ordinary
 * guest instruction is handed to `ir_decode_op()` -- the same function the
 * oracle drove through 100,000 random blocks -- so this file cannot get ADDU
 * wrong.  What it can get wrong is everything around it, and that is exactly
 * what is written out at length below.
 *
 * THREE THINGS ARE DIFFERENT HERE FROM THE RAW-WORD PATH.
 *
 * The list is not all MIPS.  The optimiser rewrites instructions it can prove
 * things about into meta opcodes of its own -- a move, a sign extension, a
 * complement -- which have no encoding in the guest ISA and must be lowered
 * from `struct opcode_m`'s fields rather than from a word.
 *
 * The list is not one basic block.  With OPT_LOCAL_BRANCHES a lightrec block
 * runs past a branch whose target is inside the same block.  fgl's IR has one
 * exit, so one call lowers one basic block and reports how far it got.
 *
 * And the list carries proof.  A load labelled LIGHTREC_IO_RAM has had its
 * region established by constant propagation; a division flagged
 * LIGHTREC_NO_DIV_CHECK has a divisor the optimiser showed to be nonzero.
 * Those facts ride onto the node in `io` and `hint`, where the emitter can
 * drop the mask or the check.  Nothing is assumed: a fact absent is a fact not
 * used, and the node keeps the conservative zero the raw-word decoder gives.
 */

#include <stddef.h>

#include "ir.h"
#include "decode_int.h"
#include "fgl_state.h"
#include "front.h"

#include "disassembler.h"

/* The region enum is lightrec's, renumbered nowhere: fgl's FGL_IO_* are the
 * same values so the mapping is an assignment rather than a switch.  If that
 * ever stops being true this stops compiling, which is the point. */
typedef char fgl_io_enum_matches[
        (FGL_IO_UNKNOWN    == LIGHTREC_IO_UNKNOWN &&
         FGL_IO_DIRECT     == LIGHTREC_IO_DIRECT &&
         FGL_IO_HW         == LIGHTREC_IO_HW &&
         FGL_IO_RAM        == LIGHTREC_IO_RAM &&
         FGL_IO_BIOS       == LIGHTREC_IO_BIOS &&
         FGL_IO_SCRATCH    == LIGHTREC_IO_SCRATCH &&
         FGL_IO_DIRECT_HW  == LIGHTREC_IO_DIRECT_HW) ? 1 : -1];

/* ------------------------------------------------------------------ */
/* Carrying the optimiser's proofs onto the nodes                      */
/* ------------------------------------------------------------------ */

/* One guest instruction can become several nodes -- a load and its deferred
 * write, a link and its jump -- so the flags are applied to the range the
 * instruction produced rather than to "the node", and each node takes only the
 * facts that mean something for its own opcode. */
/* Can fgl lower this access itself, or does the whole thing go to C?
 *
 * Anything with a proven region is fgl's: the direct regions become a masked
 * access, and FGL_IO_HW becomes a call to the accessor for its width.  The two
 * that are not:
 *
 *   - an UNPROVEN region.  The address may reach RAM, a device or nothing at
 *     all, and only lightrec's map dispatch knows which.
 *   - a device access whose SHAPE has no accessor.  `lightrec_hw_lb` and its
 *     family cover the five plain load widths and the three plain store
 *     widths; LWL/LWR/SWL/SWR, LWC2/SWC2 and the meta forms have none, which
 *     is the same conclusion lightrec reaches when `rec_load_hw_call` returns
 *     false and falls through to its generic path.
 */
static int access_is_ours(const ir_node *p, unsigned io)
{
        if (io == FGL_IO_UNKNOWN)
                return 0;
        if (io != FGL_IO_HW)
                return 1;               /* a region, and a direct access */
        return p->op == IR_LOAD || p->op == IR_STORE;
}

/* Returns 0 if it met an access it cannot express, which stops the block. */
static int apply_flags(ir_ctx *c, int from, uint32_t flags, uint32_t insn)
{
        unsigned io = LIGHTREC_FLAGS_GET_IO_MODE(flags);
        int i;

        for (i = from; i < c->n; i++) {
                ir_node *p = &c->out[i];

                switch (p->op) {
                case IR_LOAD: case IR_STORE:
                case IR_LOAD_UN: case IR_STORE_UN:
                case IR_LWC2: case IR_SWC2:
                        p->io = (uint8_t)io;
                        if (op_flag_no_mask(flags))
                                p->hint |= FGL_H_NO_MASK;

                        if (access_is_ours(p, io))
                                break;

                        /* A DEFERRED LOAD CANNOT GO THIS WAY, and refusing is
                         * the only honest answer.  C decides for itself
                         * whether to write the destination register or park
                         * the value, from `state->in_delay_slot_n` -- which is
                         * lightrec's own load-delay machinery and not fgl's.
                         * The IR_TEMP_GET waiting after the shadow would then
                         * collect from a slot that may never have been
                         * written.  Two mechanisms for one thing, and picking
                         * either at random here is how a register goes stale
                         * once in a while rather than every time. */
                        if (p->defer)
                                return 0;

                        p->op = IR_RW;
                        p->imm = insn;  /* the whole guest word; see ir.h */
                        p->imm2 = 0;
                        p->rd = p->rs = p->rt = 0;
                        break;

                case IR_MULDIV:
                        if (OPT_FLAG_MULT_DIV) {
                                if (flags & LIGHTREC_NO_LO)
                                        p->hint |= FGL_H_NO_LO;
                                if (flags & LIGHTREC_NO_HI)
                                        p->hint |= FGL_H_NO_HI;
                                if (flags & LIGHTREC_NO_DIV_CHECK)
                                        p->hint |= FGL_H_NO_DIV_CHK;
                        }
                        break;

                default:
                        break;
                }
        }
        return 1;
}

/* ------------------------------------------------------------------ */
/* The opcodes that are not MIPS                                       */
/* ------------------------------------------------------------------ */

/* A sign extension, lowered as a pair of shifts rather than as an IR opcode of
 * its own.
 *
 * SH-4 has exts.b and exts.w and this is two instructions where one would do,
 * which is a real cost on a hot path.  It is still the right first move: an
 * IR_EXT node would be the one opcode in the IR that NO MIPS INSTRUCTION CAN
 * PRODUCE, so the oracle -- which generates guest code and compares -- could
 * not reach it, and it would go to hardware never having been executed.  The
 * shifts are two operations the oracle has already validated a great many
 * times.  When the oracle learns to generate meta opcodes directly this
 * becomes a one-node lowering; until then the cost buys coverage.
 */
static void lower_ext(ir_ctx *c, uint32_t pc, unsigned rd, unsigned rs,
                      unsigned bits)
{
        unsigned shift = 32u - bits;
        ir_node *p;

        if (rd == 0)
                return;

        p = ir_node_new(c, IR_SHIFT_IMM, pc);
        if (!p)
                return;
        p->sub = SH_LL;
        p->rd = (uint8_t)rd;
        p->rt = (uint8_t)rs;
        p->imm = shift;

        p = ir_node_new(c, IR_SHIFT_IMM, pc);
        if (!p)
                return;
        p->sub = SH_RA;
        p->rd = (uint8_t)rd;
        p->rt = (uint8_t)rd;
        p->imm = shift;
}

/* A shift into LO or HI, folded the way the shared lowering folds one: a zero
 * shift is a move, and a shift of $zero is a zero. */
static void shift_into(ir_ctx *c, uint32_t pc, unsigned dst, unsigned rs,
                       unsigned sub, unsigned amount)
{
        ir_node *p;

        if (rs == 0) {
                ir_emit_set(c, pc, dst, 0);
                return;
        }
        if (amount == 0) {
                p = ir_node_new(c, IR_MOVE, pc);
                if (p) {
                        p->rd = (uint8_t)dst;
                        p->rs = (uint8_t)rs;
                }
                return;
        }
        p = ir_node_new(c, IR_SHIFT_IMM, pc);
        if (p) {
                p->sub = (uint8_t)sub;
                p->rd = (uint8_t)dst;
                p->rt = (uint8_t)rs;
                p->imm = amount;
        }
}

/* MULT or MULTU by a power of two, which the optimiser proved and rewrote.
 *
 * It is still a 64-bit product -- {HI:LO} = rs << k -- and NOT a shift
 * instruction: k lives in `c.r.op`, the field that is the sub-opcode selector
 * for every other SPECIAL encoding, which is why this is decoded here by hand
 * rather than fed to the shared lowering.
 *
 * LO is the low word of that shift and HI the bits it pushed out, filled
 * according to the sign of the operation.  Both are ordinary shifts of an
 * ordinary register, so this inherits the shift lowering's validation; what it
 * does not inherit is this decode, which is what the tests below pin.
 */
static void lower_mult2(ir_ctx *c, const struct opcode *op, uint32_t pc)
{
        unsigned k = op->r.op;                  /* NOT a sub-opcode */
        unsigned rs = op->i.rs;
        int is_signed = (op->opcode >> 26) == OP_META_MULT2;
        uint32_t flags = op->flags;

        if (!(OPT_FLAG_MULT_DIV && (flags & LIGHTREC_NO_LO))) {
                if (k >= 32)
                        ir_emit_set(c, pc, GUEST_LO, 0);
                else
                        shift_into(c, pc, GUEST_LO, rs, SH_LL, k);
        }

        if (OPT_FLAG_MULT_DIV && (flags & LIGHTREC_NO_HI))
                return;

        if (k >= 32)
                shift_into(c, pc, GUEST_HI, rs, SH_LL, k - 32);
        else if (k == 0)
                /* Nothing was pushed out of the low word, so the high word is
                 * the sign of rs -- and for an unsigned multiply there is no
                 * sign, so it is zero. Not a shift by zero, which would be a
                 * move and would put the whole value in HI. */
                if (is_signed)
                        shift_into(c, pc, GUEST_HI, rs, SH_RA, 31);
                else
                        ir_emit_set(c, pc, GUEST_HI, 0);
        else
                shift_into(c, pc, GUEST_HI, rs,
                           is_signed ? SH_RA : SH_RL, 32 - k);
}

/* The unaligned word the optimiser fused back into the pair it came from.
 *
 * LWL and LWR at addresses three apart are one unaligned 32-bit access, and
 * lightrec rewrites the pair into a single meta opcode whose immediate is the
 * LOW address (optimizer.c:1053) with the other half zeroed out of the list.
 *
 * fgl un-fuses it.  That is deliberately giving something up: the fused form
 * is one access where this is two, and the unaligned lowering is the most
 * expensive thing fgl emits.  But the pair is code the oracle has run millions
 * of times, and a fused lowering is a new one nothing has tested -- and the
 * fusion buys nothing until the emitter has an unaligned-word path worth
 * calling, which is a separate piece of work with its own validation. Doing it
 * in the wrong order would put untested code on the hottest path in the
 * emitter.
 */
static void lower_fused_unaligned(ir_ctx *c, const struct opcode *op,
                                  uint32_t pc, int store)
{
        uint32_t lo = (uint32_t)(int32_t)(int16_t)op->i.imm;
        unsigned rs = op->i.rs, rt = op->i.rt;
        uint32_t word;

        /* Rebuilt as guest instructions and handed to the shared lowering,
         * rather than as nodes: the pair is exactly what the decoder saw
         * before the optimiser fused it, and re-deriving the nodes here would
         * be a second copy of a lowering with a great deal of detail in it. */
        word = ((store ? 0x2eu : 0x26u) << 26) | (rs << 21) | (rt << 16) |
               (lo & 0xffffu);
        ir_decode_op(c, word, pc);              /* SWR / LWR, low address  */

        word = ((store ? 0x2au : 0x22u) << 26) | (rs << 21) | (rt << 16) |
               ((lo + 3u) & 0xffffu);
        ir_decode_op(c, word, pc);              /* SWL / LWL, high address */
}

/* Returns non-zero if the opcode was lowered. */
static int lower_meta(ir_ctx *c, const struct opcode *op, uint32_t pc)
{
        ir_node *p;

        switch (op->m.op) {
        case OP_META_MOV:
                if (op->m.rd == 0)
                        return 1;
                if (op->m.rs == 0) {
                        ir_emit_set(c, pc, op->m.rd, 0);
                        return 1;
                }
                p = ir_node_new(c, IR_MOVE, pc);
                if (p) {
                        p->rd = (uint8_t)op->m.rd;
                        p->rs = (uint8_t)op->m.rs;
                }
                return 1;

        case OP_META_EXTC:
                lower_ext(c, pc, op->m.rd, op->m.rs, 8);
                return 1;

        case OP_META_EXTS:
                lower_ext(c, pc, op->m.rd, op->m.rs, 16);
                return 1;

        /* ~rs is NOR(rs, $zero), which the ALU already does, so the complement
         * needs no opcode of its own and inherits the ALU's validation. */
        case OP_META_COM:
                if (op->m.rd == 0)
                        return 1;
                p = ir_node_new(c, IR_ALU, pc);
                if (p) {
                        p->sub = ALU_NOR;
                        p->rd = (uint8_t)op->m.rd;
                        p->rs = (uint8_t)op->m.rs;
                        p->rt = 0;
                }
                return 1;

        default:
                return 0;
        }
}

/* ------------------------------------------------------------------ */
/* LUI paired with ORI or ADDIU: one constant, not two operations      */
/* ------------------------------------------------------------------ */

/* The optimiser marks both halves of a `lui rt,hi` / `ori rt,rt,lo` pair with
 * LIGHTREC_MOVI, and the pair is a single 32-bit constant.  Lowering them
 * separately is not wrong -- it is two ordinary MIPS instructions with an
 * ordinary result -- so the fold is a saving rather than a requirement, and if
 * it is ever in doubt the honest move is to drop it.
 *
 * It is safe to do here because the pair cannot straddle an fgl block.
 * `find_next_reader` (optimizer.c:265) walks forward from the LUI and stops at
 * a write to the register, at a SYNC, at a branch whose slot has been switched
 * and at a delay slot -- so the partner is at worst the delay slot of the
 * transfer that ends this block, which this loop still lowers.  Nothing
 * between the two touches the register either, which is what makes emitting
 * one node in the partner's place equivalent.
 *
 * The pending half is still flushed on every exit, because "cannot happen"
 * and "produces no code if it does" should not be the same sentence. */
typedef struct {
        uint16_t hi[32];
        uint32_t live;          /* one bit per guest register */
} movi_state;

static void movi_flush(ir_ctx *c, movi_state *m, uint32_t pc)
{
        unsigned r;

        for (r = 1; r < 32; r++)
                if (m->live & (1u << r))
                        ir_emit_set(c, pc, r, (uint32_t)m->hi[r] << 16);
        m->live = 0;
}

/* Returns non-zero if the opcode was consumed by the fold. */
static int movi_step(ir_ctx *c, movi_state *m, const struct opcode *op,
                     uint32_t pc)
{
        unsigned major = op->opcode >> 26;
        unsigned rt = op->i.rt;
        uint32_t hi;

        if (!(op->flags & LIGHTREC_MOVI))
                return 0;

        if (major == OP_LUI) {
                if (rt == 0)
                        return 1;               /* into $zero: nothing at all */
                m->hi[rt] = (uint16_t)op->i.imm;
                m->live |= 1u << rt;
                return 1;
        }

        if (major != OP_ORI && major != OP_ADDI && major != OP_ADDIU)
                return 0;

        /* The partner without its LUI. The optimiser does not produce this,
         * but a block entered part-way would, and the instruction still has
         * its own perfectly good meaning. */
        if (rt == 0 || !(m->live & (1u << rt)))
                return 0;

        hi = (uint32_t)m->hi[rt] << 16;
        m->live &= ~(1u << rt);

        ir_emit_set(c, pc, rt,
                    major == OP_ORI ? (hi | (op->i.imm & 0xffffu))
                                    : hi + (uint32_t)(int32_t)(int16_t)op->i.imm);
        return 1;
}

/* ------------------------------------------------------------------ */
/* The loop                                                            */
/* ------------------------------------------------------------------ */

/* WHAT THE HAZARD ANALYSIS IS ALLOWED TO SEE.
 *
 * `ir_shadow_pending` and `ir_shadow_fix` answer questions about a guest
 * instruction -- does it read this register, does it write state a pending
 * load reads -- by decoding a MIPS word.  A meta opcode is not one.  Handing
 * `OP_META_MULT2` to them would have them read a shift amount as a sub-opcode
 * and answer confidently from nonsense, which is the same failure mode as the
 * load-shadow bug that motivated `defer`: no crash, no signal, a wrong
 * register some frames later.
 *
 * So every meta opcode is given a real MIPS instruction with THE SAME
 * REGISTER BEHAVIOUR to be analysed as -- not lowered as, only analysed as.
 * The substitute has to read what the meta opcode reads and write what it
 * writes; nothing else about it is used.
 */
static uint32_t hazard_word(const struct opcode *op)
{
        unsigned major = op->opcode >> 26;

        switch (major) {
        case OP_META:
                /* All four meta forms read rs and write rd: `addu rd,rs,$zero`
                 * does exactly that. */
                return (op->m.rs << 21) | (op->m.rd << 11) | 0x21u;

        case OP_META_MULT2:
        case OP_META_MULTU2:
                /* Reads rs and rt, writes neither: a MULT. */
                return (op->i.rs << 21) | (op->i.rt << 16) | 0x18u;

        case OP_META_LWU:
                /* Reads rs, writes rt, and is a load -- so it can leave a
                 * shadow, exactly as the LWR it was fused from. */
                return (0x26u << 26) | (op->i.rs << 21) | (op->i.rt << 16) |
                       (op->i.imm & 0xffffu);

        case OP_META_SWU:
                return (0x2eu << 26) | (op->i.rs << 21) | (op->i.rt << 16) |
                       (op->i.imm & 0xffffu);

        default:
                return op->opcode;
        }
}

/* One step of the pass's load-shadow bookkeeping: settle the pending load
 * against the instruction just lowered, then decide whether that instruction
 * leaves a load pending itself. */
static int shadow_step(ir_ctx *c, int pend, uint32_t *pend_insn,
                       uint32_t insn, int mark)
{
        int settled = ir_shadow_fix(c, pend, *pend_insn, insn, mark);

        *pend_insn = insn;
        return settled ? -1 : ir_shadow_pending(c, insn, mark);
}

static void note_unsupported(fgl_front_info *info, const struct opcode *op,
                             uint32_t pc)
{
        info->unsupported++;
        if (info->unsupported == 1) {
                info->unsupported_pc = pc;
                info->unsupported_op = op->opcode;
        }
}

int fgl_front(const struct opcode *ops, unsigned nb, uint32_t pc,
              ir_node *out, int max, fgl_front_info *info)
{
        ir_ctx c;
        unsigned i;
        int pend = -1;                  /* node index of a load in shadow */
        uint32_t pend_insn = 0;
        movi_state movi;

        c.out = out;
        c.n = 0;
        c.max = max;
        c.unknown = 0;

        info->n_ops = 0;
        info->unsupported = 0;
        info->unsupported_pc = 0;
        info->unsupported_op = 0;
        info->ended_early = 0;
        info->stop_reason = FGL_STOP_END;
        movi.live = 0;

        for (i = 0; i < nb; i++) {
                const struct opcode *op = &ops[i];
                uint32_t at = pc + 4u * i;
                uint32_t word = op->opcode;
                unsigned major = word >> 26;
                /* What the hazard analysis is shown; the same word for
                 * everything that really is a MIPS instruction. */
                uint32_t hword = hazard_word(op);
                int mark = c.n;

                if (c.n + 4 > max) {
                        movi_flush(&c, &movi, at);
                        info->stop_reason = FGL_STOP_FULL;
                        info->ended_early = 1;
                        break;
                }

                /* The optimiser's own opcodes first: they occupy encodings
                 * that mean something else in the guest ISA, so testing for
                 * them before decoding is not an optimisation but a
                 * correctness requirement. */
                if (major == OP_META) {
                        /* A meta opcode is never a load, so it cannot start a
                         * shadow -- but it can BE one, and rotating a load
                         * past a move that reads it would be wrong.  It goes
                         * through the same hazard step as anything else. */
                        if (!lower_meta(&c, op, at)) {
                                note_unsupported(info, op, at);
                                info->stop_reason = FGL_STOP_UNSUPPORTED;
                                info->ended_early = 1;
                                break;
                        }
                        if (!apply_flags(&c, mark, op->flags, op->opcode)) {
                                note_unsupported(info, op, at);
                                info->stop_reason = FGL_STOP_UNSUPPORTED;
                                info->ended_early = 1;
                                break;
                        }
                        pend = shadow_step(&c, pend, &pend_insn, hword, mark);
                        info->n_ops = i + 1;
                        continue;
                }

                if (major == OP_META_MULT2 || major == OP_META_MULTU2) {
                        lower_mult2(&c, op, at);
                        if (!apply_flags(&c, mark, op->flags, op->opcode)) {
                                note_unsupported(info, op, at);
                                info->stop_reason = FGL_STOP_UNSUPPORTED;
                                info->ended_early = 1;
                                break;
                        }
                        pend = shadow_step(&c, pend, &pend_insn, hword, mark);
                        info->n_ops = i + 1;
                        continue;
                }

                if (major == OP_META_LWU || major == OP_META_SWU) {
                        lower_fused_unaligned(&c, op, at,
                                              major == OP_META_SWU);
                        if (!apply_flags(&c, mark, op->flags, op->opcode)) {
                                note_unsupported(info, op, at);
                                info->stop_reason = FGL_STOP_UNSUPPORTED;
                                info->ended_early = 1;
                                break;
                        }
                        pend = shadow_step(&c, pend, &pend_insn, hword, mark);
                        info->n_ops = i + 1;
                        continue;
                }

                /* OP_META_BIOS is tested for in this tree but never produced
                 * by any pass in it. If that ever changes it stops the block
                 * loudly instead of being lowered as whatever 0x3b happens to
                 * decode as. */
                if (major == OP_META_BIOS) {
                        movi_flush(&c, &movi, at);
                        note_unsupported(info, op, at);
                        info->stop_reason = FGL_STOP_UNSUPPORTED;
                        info->ended_early = 1;
                        break;
                }

                if (!ir_is_transfer(word)) {
                        if (movi_step(&c, &movi, op, at)) {
                                /* A FOLDED HALF IS STILL AN INSTRUCTION.
                                 *
                                 * The load shadow counts instructions, not
                                 * nodes, and a folded LUI emits no nodes at
                                 * all.  Skipping the hazard step here would
                                 * leave a pending load to settle against the
                                 * instruction after this one, delaying its
                                 * register write by two instead of one --
                                 * silently, and only when a load happens to
                                 * sit in front of a constant. So the step runs
                                 * either way; with no nodes added it settles
                                 * the load exactly where it already is, which
                                 * is what one instruction of delay means. */
                                pend = shadow_step(&c, pend, &pend_insn,
                                                   hword, mark);
                                info->n_ops = i + 1;
                                continue;
                        }
                        c.unknown = 0;
                        ir_decode_op(&c, word, at);

                        /* AN OPCODE WITH NO CASE IS A BIOS CALL, NOT A GAP.
                         *
                         * The emulator delivers HLE by marking an opcode
                         * unknown and letting the block stop at it, so the
                         * right lowering is not "skip" -- which is what the
                         * raw-word path does and is correct there -- but
                         * "leave for C and say why". Dropping it instead
                         * would run the game with no BIOS and no error.
                         *
                         * It ends the block: C resumes AT this instruction,
                         * not after it, because C is what executes it. */
                        if (c.unknown) {
                                ir_node *x;

                                c.n = mark;     /* drop any partial lowering */
                                movi_flush(&c, &movi, at);
                                x = ir_node_new(&c, IR_EXIT, at);
                                if (x) {
                                        x->imm = FGL_EXIT_UNKNOWN_OP;
                                        x->imm2 = at;
                                }
                                info->n_ops = i;
                                info->stop_reason = FGL_STOP_UNKNOWN_OP;
                                info->ended_early = 1;
                                return c.n;
                        }

                        if (!apply_flags(&c, mark, op->flags, op->opcode)) {
                                note_unsupported(info, op, at);
                                info->stop_reason = FGL_STOP_UNSUPPORTED;
                                info->ended_early = 1;
                                break;
                        }
                        pend = shadow_step(&c, pend, &pend_insn, hword, mark);
                        info->n_ops = i + 1;
                        continue;
                }

                /* The block ends here, so anything the fold is still holding
                 * has to become code. */
                movi_flush(&c, &movi, at);

                /* A branch whose target is inside this same lightrec block.
                 * fgl's IR has one exit, so the block ends here and the exit
                 * PC sends the dispatcher to a block starting at the target.
                 * That is correct and it is slow -- the whole point of a local
                 * branch is not to pay a dispatch -- so it is the first thing
                 * to revisit once blocks can chain. */
                if (op_flag_local_branch(op->flags)) {
                        if (ir_decode_transfer(&c, word, at)) {
                                ir_shadow_fix(&c, pend, pend_insn, hword, mark);
                                if (!op_flag_no_ds(op->flags) && i + 1 < nb)
                                        ir_decode_op(&c, ops[i + 1].opcode,
                                                     at + 4);
                        } else {
                                ir_shadow_fix(&c, pend, pend_insn, hword, mark);
                        }
                        info->n_ops = i + 2;
                        info->stop_reason = FGL_STOP_LOCAL;
                        info->ended_early = 1;
                        return c.n;
                }

                if (ir_decode_transfer(&c, word, at)) {
                        /* A load shadowed by the transfer settles BETWEEN the
                         * transfer's nodes and the delay slot: the transfer
                         * reads pre-slot state, and the load's write lands
                         * after the transfer has read it. */
                        ir_shadow_fix(&c, pend, pend_insn, hword, mark);

                        /* LIGHTREC_NO_DS says the slot has already been moved
                         * ahead of the branch by the optimiser, so it has
                         * been lowered already and there is nothing after the
                         * transfer to lower. */
                        if (!op_flag_no_ds(op->flags) && i + 1 < nb) {
                                int dmark = c.n;

                                ir_decode_op(&c, ops[i + 1].opcode, at + 4);
                                if (!apply_flags(&c, dmark, ops[i + 1].flags,
                                                 ops[i + 1].opcode)) {
                                        note_unsupported(info, &ops[i + 1], at + 4);
                                        info->stop_reason = FGL_STOP_UNSUPPORTED;
                                        info->ended_early = 1;
                                        return c.n;
                                }
                                info->n_ops = i + 2;
                        } else {
                                info->n_ops = i + 1;
                        }
                } else {
                        ir_shadow_fix(&c, pend, pend_insn, hword, mark);
                        info->n_ops = i + 1;    /* SYSCALL / BREAK: no slot */
                }

                info->stop_reason = FGL_STOP_TRANSFER;
                info->ended_early = (info->n_ops < nb);
                return c.n;
        }

        /* Fell off the end of the list without a transfer. The next block
         * starts at the instruction after the last one lowered. */
        {
                movi_flush(&c, &movi, pc + 4u * info->n_ops);
        }
        {
                ir_node *p = ir_node_new(&c, IR_JUMP, pc);

                if (p)
                        p->imm = pc + 4u * info->n_ops;
        }
        return c.n;
}
