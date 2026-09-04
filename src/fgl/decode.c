/* The decode pass: guest words in, IR nodes out, one linear pass.
 *
 * No basic-block analysis, no profiling, no second look at anything already
 * decoded.  A word is fetched, its opcode selects a case, a node is filled in.
 *
 * CONSTANT FOLDING HAPPENS HERE, not in a later pass, and it is free because
 * every fold is a test the handler was going to make anyway.  Guest register 0
 * reads as zero and discards writes, so a surprising share of real code is
 * `addu rd,rs,$zero` and friends — a move — or writes a result nobody keeps.
 * Both vanish before the emitter runs.
 *
 * The block ends at the first control transfer plus its delay slot.  What that
 * costs is a block boundary every few instructions; what it buys is that every
 * boundary has exactly one entry state, which is what makes the writeback
 * reconciliation at the end of a block valid at all.
 */

#include <stddef.h>

#include "ir.h"
#include "fgl_state.h"

typedef struct {
        ir_node *out;
        int      n;
        int      max;
} ctx;

static ir_node *node(ctx *c, int op, uint32_t pc)
{
        ir_node *p;

        if (c->n >= c->max)
                return 0;
        p = &c->out[c->n++];
        p->op = (uint8_t)op;
        p->sub = 0;
        p->rd = p->rs = p->rt = 0;
        p->imm = p->imm2 = 0;
        p->pc = pc;
        /* Unallocated until the allocation pass says otherwise, which is what
         * makes that pass optional. */
        p->hd = p->hs = p->ht = p->hx = -1;
        p->sc[0] = p->sc[1] = 0;
        p->defer = 0;
        return p;
}

/* Writes to guest register 0 are discarded by the machine, so anything that
 * produces one produces nothing.  Every constructor below tests `rd == 0` and
 * returns, which is what keeps the emitter from needing to know about $zero. */

/* A COP2 register's displacement in the state block.
 *
 * Data and control are one contiguous 64-entry file at the reference's own
 * offsets, so the number the decoder resolves here is the number every GBR
 * displacement in its disassembly refers to.  In state-block units, which are
 * longwords, because that is what the load and store instructions take. */
static uint32_t cop2_disp(unsigned reg, int control)
{
        return FGL_AT_CP2D + (control ? 32u : 0u) + (reg & 31u);
}

/* SIX OF THE SIXTY-FOUR ARE SIXTEEN BITS WIDE, AND KEEPING THEM AS THIRTY-TWO
 * IS NOT A HARMLESS SIMPLIFICATION.
 *
 * `IR0`-`IR3` and the four scalar control registers hold a signed halfword on
 * the real unit: a write keeps the low sixteen bits and a read hands back the
 * sign extension of them.  Storing the guest's whole word instead is invisible
 * until something reads one back and looks at bit 31 — and the target title's
 * display-list interpreter opens with exactly that, `cfc2` of `ZSF4` followed
 * by `srl 31`, using the register as a one-bit flag it wrote earlier.  Keeping
 * 32 bits makes that bit permanently zero and sends the routine down the wrong
 * arm before it has emitted a single primitive.
 *
 * The extension is done on the way IN, which is the cheaper half: the register
 * number is known at compile time, so it costs one `exts.w` on a write that
 * happens rarely, and every read stays a bare load.
 *
 * NINE, NOT SIX.  Each 3x3 matrix packs into five words — four holding two
 * halfwords and a fifth holding the ninth on its own — so `RT33`, `LLM33` and
 * `LCM33` are 16-bit registers that were missed when the list was first
 * written, and the same reasoning that applies to `ZSF4` applies to them.
 *
 * MEASURED, not deduced: the target title leaves `RT33` reading `0x005703F1`
 * here against `0x000003F1` on a working PlayStation. The top half is `0x57`,
 * which is the title's own `ZSF4` — it packs a flag above the matrix entry and
 * the machine drops it. The matrix load reads the low halfword either way, so
 * nothing about the transform changed; what changes is what a `cfc2` of `RT33`
 * hands back, and this title is already known to read a control register and
 * test its sign bit. */
static int cop2_is_half(unsigned reg, int control)
{
        if (control)
                return reg == 4u  ||    /* RT33  — the odd one out of nine */
                       reg == 12u ||    /* LLM33 */
                       reg == 20u ||    /* LCM33 */
                       reg == 26u ||    /* H     */
                       reg == 27u ||    /* DQA   */
                       reg == 29u ||    /* ZSF3  */
                       reg == 30u;      /* ZSF4  */
        return reg >= 8u && reg <= 11u; /* IR0-IR3 */
}

static void emit_move(ctx *c, uint32_t pc, unsigned rd, unsigned rs)
{
        ir_node *p;

        if (rd == 0 || rd == rs)
                return;
        p = node(c, IR_MOVE, pc);
        if (!p)
                return;
        p->rd = (uint8_t)rd;
        p->rs = (uint8_t)rs;
}

static void emit_set(ctx *c, uint32_t pc, unsigned rd, uint32_t v)
{
        ir_node *p;

        if (rd == 0)
                return;
        p = node(c, IR_SET, pc);
        if (!p)
                return;
        p->rd = (uint8_t)rd;
        p->imm = v;
}

/* rd = rs <alu> rt, with the identities that fall out of a zero operand. */
static void emit_alu(ctx *c, uint32_t pc, unsigned sub,
                     unsigned rd, unsigned rs, unsigned rt)
{
        ir_node *p;

        if (rd == 0)
                return;

        if (rs == 0 && rt == 0) {
                /* Every one of these is a constant with both inputs zero. */
                emit_set(c, pc, rd, sub == ALU_NOR ? 0xffffffffu : 0);
                return;
        }
        if (rt == 0 && (sub == ALU_ADD || sub == ALU_SUB ||
                        sub == ALU_OR || sub == ALU_XOR)) {
                emit_move(c, pc, rd, rs);
                return;
        }
        if (rs == 0 && (sub == ALU_ADD || sub == ALU_OR || sub == ALU_XOR)) {
                emit_move(c, pc, rd, rt);
                return;
        }
        if (sub == ALU_AND && (rs == 0 || rt == 0)) {
                emit_set(c, pc, rd, 0);
                return;
        }

        p = node(c, IR_ALU, pc);
        if (!p)
                return;
        p->sub = (uint8_t)sub;
        p->rd = (uint8_t)rd;
        p->rs = (uint8_t)rs;
        p->rt = (uint8_t)rt;
}

/* rd = rs <alu> imm.  `imm` arrives already widened the way the guest
 * instruction widens it, so nothing downstream needs to know which. */
static void emit_alu_imm(ctx *c, uint32_t pc, unsigned sub,
                         unsigned rd, unsigned rs, uint32_t imm)
{
        ir_node *p;

        if (rd == 0)
                return;

        if (rs == 0) {
                uint32_t v;

                switch (sub) {
                case ALU_ADD: case ALU_OR: case ALU_XOR: v = imm; break;
                case ALU_AND: v = 0; break;
                case ALU_SLT: v = (int32_t)0 < (int32_t)imm; break;
                default:      v = 0u < imm; break;      /* ALU_SLTU */
                }
                emit_set(c, pc, rd, v);
                return;
        }
        if (imm == 0 && (sub == ALU_ADD || sub == ALU_OR || sub == ALU_XOR)) {
                emit_move(c, pc, rd, rs);
                return;
        }
        if (imm == 0 && (sub == ALU_AND || sub == ALU_SLTU)) {
                emit_set(c, pc, rd, 0);
                return;
        }

        p = node(c, IR_ALU_IMM, pc);
        if (!p)
                return;
        p->sub = (uint8_t)sub;
        p->rd = (uint8_t)rd;
        p->rs = (uint8_t)rs;
        p->imm = imm;
}

/* An ordinary instruction — anything that is not a control transfer.  Called
 * for the delay slot too, which is why it is a function rather than inline in
 * the loop. */
static void decode_op(ctx *c, uint32_t insn, uint32_t pc)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;
        unsigned rt = (insn >> 16) & 31;
        unsigned rd = (insn >> 11) & 31;
        unsigned sa = (insn >> 6) & 31;
        unsigned fn = insn & 63;
        uint32_t imm = insn & 0xffff;
        uint32_t simm = (uint32_t)(int32_t)(int16_t)imm;
        ir_node *p;

        switch (op) {
        case 0x00:
                switch (fn) {
                case 0x00: case 0x02: case 0x03:                /* SLL/SRL/SRA */
                        if (rd == 0)
                                return;
                        if (sa == 0 || rt == 0) {
                                /* A zero shift is a move; a zero source is a
                                 * zero result whatever the direction. */
                                if (rt == 0)
                                        emit_set(c, pc, rd, 0);
                                else
                                        emit_move(c, pc, rd, rt);
                                return;
                        }
                        p = node(c, IR_SHIFT_IMM, pc);
                        if (!p)
                                return;
                        p->sub = (uint8_t)(fn == 0x00 ? SH_LL :
                                           fn == 0x02 ? SH_RL : SH_RA);
                        p->rd = (uint8_t)rd;
                        p->rt = (uint8_t)rt;
                        p->imm = sa;
                        return;

                case 0x04: case 0x06: case 0x07:            /* SLLV/SRLV/SRAV */
                        if (rd == 0)
                                return;
                        if (rs == 0) {
                                emit_move(c, pc, rd, rt);
                                return;
                        }
                        p = node(c, IR_SHIFT_REG, pc);
                        if (!p)
                                return;
                        p->sub = (uint8_t)(fn == 0x04 ? SH_LL :
                                           fn == 0x06 ? SH_RL : SH_RA);
                        p->rd = (uint8_t)rd;
                        p->rs = (uint8_t)rs;
                        p->rt = (uint8_t)rt;
                        return;

                case 0x10: emit_move(c, pc, rd, 33); return;    /* MFHI */
                case 0x11: emit_move(c, pc, 33, rs); return;    /* MTHI */
                case 0x12: emit_move(c, pc, rd, 32); return;    /* MFLO */
                case 0x13: emit_move(c, pc, 32, rs); return;    /* MTLO */

                case 0x18: case 0x19: case 0x1a: case 0x1b:
                        p = node(c, IR_MULDIV, pc);
                        if (!p)
                                return;
                        p->sub = (uint8_t)(fn - 0x18);
                        p->rs = (uint8_t)rs;
                        p->rt = (uint8_t)rt;
                        return;

                /* ADD and ADDU are one case: no overflow trap.  Same for SUB.
                 * The reference agrees, deliberately. */
                case 0x20: case 0x21: emit_alu(c, pc, ALU_ADD, rd, rs, rt); return;
                case 0x22: case 0x23: emit_alu(c, pc, ALU_SUB, rd, rs, rt); return;
                case 0x24: emit_alu(c, pc, ALU_AND, rd, rs, rt); return;
                case 0x25: emit_alu(c, pc, ALU_OR, rd, rs, rt); return;
                case 0x26: emit_alu(c, pc, ALU_XOR, rd, rs, rt); return;
                case 0x27: emit_alu(c, pc, ALU_NOR, rd, rs, rt); return;
                case 0x2a: emit_alu(c, pc, ALU_SLT, rd, rs, rt); return;
                case 0x2b: emit_alu(c, pc, ALU_SLTU, rd, rs, rt); return;

                default: return;                        /* skipped, not trapped */
                }

        case 0x08: case 0x09: emit_alu_imm(c, pc, ALU_ADD, rt, rs, simm); return;
        case 0x0a: emit_alu_imm(c, pc, ALU_SLT, rt, rs, simm); return;
        case 0x0b: emit_alu_imm(c, pc, ALU_SLTU, rt, rs, simm); return;
        case 0x0c: emit_alu_imm(c, pc, ALU_AND, rt, rs, imm); return;
        case 0x0d: emit_alu_imm(c, pc, ALU_OR, rt, rs, imm); return;
        case 0x0e: emit_alu_imm(c, pc, ALU_XOR, rt, rs, imm); return;
        case 0x0f: emit_set(c, pc, rt, imm << 16); return;      /* LUI */

        case 0x20: case 0x21: case 0x23: case 0x24: case 0x25:
                if (rt == 0)
                        return;
                p = node(c, IR_LOAD, pc);
                if (!p)
                        return;
                p->sub = (uint8_t)(op == 0x20 ? MEM_B :
                                   op == 0x21 ? MEM_H :
                                   op == 0x24 ? MEM_BU :
                                   op == 0x25 ? MEM_HU : MEM_W);
                p->rd = (uint8_t)rt;
                p->rs = (uint8_t)rs;
                p->imm = simm;
                return;

        case 0x28: case 0x29: case 0x2b:
                p = node(c, IR_STORE, pc);
                if (!p)
                        return;
                p->sub = (uint8_t)(op == 0x28 ? MEM_B :
                                   op == 0x29 ? MEM_H : MEM_W);
                p->rs = (uint8_t)rs;
                p->rt = (uint8_t)rt;
                p->imm = simm;
                return;

        case 0x22: case 0x26:                                   /* LWL / LWR */
                if (rt == 0)
                        return;
                p = node(c, IR_LOAD_UN, pc);
                if (!p)
                        return;
                p->sub = (uint8_t)(op == 0x22 ? UN_LWL : UN_LWR);
                p->rd = (uint8_t)rt;
                p->rs = (uint8_t)rs;
                p->imm = simm;
                return;

        case 0x2a: case 0x2e:                                   /* SWL / SWR */
                p = node(c, IR_STORE_UN, pc);
                if (!p)
                        return;
                p->sub = (uint8_t)(op == 0x2a ? UN_SWL : UN_SWR);
                p->rs = (uint8_t)rs;
                p->rt = (uint8_t)rt;
                p->imm = simm;
                return;

        case 0x10: {    /* COP0 */
                /* Only three COP0 registers are live, and the emitter decides
                 * which — the decoder's job is the instruction form, not the
                 * state block's layout. */
                if (rs == 0x00) {                       /* MFC0 rt,rd */
                        if (rt == 0)
                                return;
                        p = node(c, IR_MFC0, pc);
                        if (!p)
                                return;
                        p->rd = (uint8_t)rt;
                        p->imm = rd;
                        return;
                }
                if (rs == 0x04) {                       /* MTC0 rt,rd */
                        p = node(c, IR_MTC0, pc);
                        if (!p)
                                return;
                        p->rs = (uint8_t)rt;
                        p->imm = rd;
                        return;
                }
                if (rs == 0x10 && fn == 0x10) {         /* RFE */
                        node(c, IR_RFE, pc);
                        return;
                }
                return;
        }

        case 0x12: {    /* COP2 — the geometry unit */
                /* Bit 25 splits the whole coprocessor space in two: set means
                 * a command, clear means a register move, and the move's kind
                 * is the `rs` field.  Same split the reference makes. */
                if (insn & (1u << 25)) {
                        p = node(c, IR_GTE, pc);
                        if (p)
                                p->imm = insn & 0x1ffffffu;
                        return;
                }
                switch (rs) {
                case 0x00:                              /* MFC2 rt,rd */
                case 0x02:                              /* CFC2 rt,rd */
                        if (rt == 0)
                                return;
                        p = node(c, IR_MFC2, pc);
                        if (!p)
                                return;
                        p->rd = (uint8_t)rt;
                        p->imm = cop2_disp(rd, rs == 0x02);
                        return;
                case 0x04:                              /* MTC2 rt,rd */
                case 0x06:                              /* CTC2 rt,rd */
                        p = node(c, IR_MTC2, pc);
                        if (!p)
                                return;
                        p->rs = (uint8_t)rt;
                        p->imm = cop2_disp(rd, rs == 0x06);
                        p->imm2 = (uint32_t)cop2_is_half(rd, rs == 0x06);
                        return;
                default:
                        return;
                }
        }

        case 0x32:                                      /* LWC2 rt,imm(rs) */
                p = node(c, IR_LWC2, pc);
                if (!p)
                        return;
                p->rs = (uint8_t)rs;
                p->imm = cop2_disp(rt, 0);
                p->imm2 = simm;
                return;

        case 0x3a:                                      /* SWC2 rt,imm(rs) */
                p = node(c, IR_SWC2, pc);
                if (!p)
                        return;
                p->rs = (uint8_t)rs;
                p->imm = cop2_disp(rt, 0);
                p->imm2 = simm;
                return;

        default: return;                                /* skipped, not trapped */
        }
}

/* THE LOAD SHADOW, AND WHY IT NEEDS NOTHING AT RUNTIME.
 *
 * A guest load's destination does not hold the loaded value until one
 * instruction later — a one-instruction shadow with no interlock, so the
 * instruction immediately after a load reads the register's previous contents.
 * Compiled code fills the shadow with something independent, which is exactly
 * why this costs nothing on real code; hand-written code exploits it.
 *
 * The whole hazard fits in the decoder's own window: the load, and the one
 * instruction after it.  Where the second reads what the first writes, the two
 * nodes are simply emitted in the other order.  The load still happens before
 * the shadow as far as memory is concerned; it just writes its register after,
 * and that is the entire observable difference.  So there is no in-flight field
 * on the node, no pending slot in the state block, and no cost at all on the
 * overwhelming majority of loads, whose shadow is independent.
 *
 * THREE THINGS ARE DELIBERATELY NOT COVERED.
 *
 * A load in a branch delay slot.  Its shadow is the first instruction of the
 * TARGET block, which the decoder cannot see and which two successors may
 * disagree about.  Carrying the value across a block boundary is runtime state
 * paid for on every load; the case is not worth it.  plan.md §4.5.
 *
 * A shadow that writes the load's own destination.  Both orders are wrong in
 * some detail and the architecture's own answer is not agreed; the reordering
 * is declined and the existing behaviour stands.
 *
 * A shadow that writes the load's base register — the load would then address
 * off the new value.  Declined for the same reason: undoing it needs somewhere
 * to keep the old base, and there is no scratch guest register.
 */
static int mips_writes(uint32_t insn, unsigned r)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;
        unsigned rt = (insn >> 16) & 31;
        unsigned rd = (insn >> 11) & 31;
        unsigned fn = insn & 63;

        if (r == 0)
                return 0;               /* writes to $zero are discarded */

        switch (op) {
        case 0x00:
                switch (fn) {
                case 0x00: case 0x02: case 0x03:        /* SLL  SRL  SRA   */
                case 0x04: case 0x06: case 0x07:        /* SLLV SRLV SRAV  */
                case 0x09:                              /* JALR            */
                case 0x10: case 0x12:                   /* MFHI MFLO       */
                case 0x20: case 0x21: case 0x22: case 0x23:
                case 0x24: case 0x25: case 0x26: case 0x27:
                case 0x2a: case 0x2b:                   /* the ALU         */
                        return rd == r;
                default:
                        return 0;
                }
        case 0x01:                                      /* BLTZAL / BGEZAL */
                return (rt == 0x10 || rt == 0x11) && r == 31;
        case 0x03:                                      /* JAL             */
                return r == 31;
        case 0x08: case 0x09: case 0x0a: case 0x0b:
        case 0x0c: case 0x0d: case 0x0e: case 0x0f:     /* imm ALU, LUI    */
        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26:                /* the loads       */
                return rt == r;
        case 0x10:                                      /* MFC0            */
                return rs == 0x00 && rt == r;
        case 0x12:                                      /* MFC2 / CFC2     */
                return !(insn & (1u << 25)) &&
                       (rs == 0x00 || rs == 0x02) && rt == r;
        default:
                return 0;
        }
}

int ir_reads(uint32_t insn, unsigned r)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;
        unsigned rt = (insn >> 16) & 31;
        unsigned fn = insn & 63;

        if (r == 0)
                return 0;               /* $zero is a constant, not a value */

        switch (op) {
        case 0x00:
                switch (fn) {
                case 0x00: case 0x02: case 0x03:        /* SLL SRL SRA     */
                        return rt == r;
                case 0x08:                              /* JR              */
                case 0x11: case 0x13:                   /* MTHI MTLO       */
                        return rs == r;
                case 0x09:                              /* JALR            */
                        return rs == r;
                case 0x04: case 0x06: case 0x07:        /* SLLV SRLV SRAV  */
                case 0x18: case 0x19: case 0x1a: case 0x1b:
                case 0x20: case 0x21: case 0x22: case 0x23:
                case 0x24: case 0x25: case 0x26: case 0x27:
                case 0x2a: case 0x2b:
                        return rs == r || rt == r;
                default:
                        return 0;
                }
        case 0x01: case 0x06: case 0x07:                /* REGIMM BLEZ BGTZ */
                return rs == r;
        case 0x04: case 0x05:                           /* BEQ BNE          */
                return rs == r || rt == r;
        case 0x08: case 0x09: case 0x0a: case 0x0b:
        case 0x0c: case 0x0d: case 0x0e:                /* imm ALU          */
                return rs == r;
        case 0x10:                                      /* MTC0             */
                return rs == 0x04 && rt == r;
        case 0x12:                                      /* MTC2 / CTC2      */
                return !(insn & (1u << 25)) &&
                       (rs == 0x04 || rs == 0x06) && rt == r;
        case 0x20: case 0x21: case 0x23: case 0x24: case 0x25:
        case 0x32: case 0x3a:                           /* loads, LWC2/SWC2 */
                return rs == r;
        case 0x22: case 0x26:                           /* LWL / LWR        */
                return rs == r || rt == r;
        case 0x28: case 0x29: case 0x2b:
        case 0x2a: case 0x2e:                           /* the stores       */
                return rs == r || rt == r;
        default:
                return 0;
        }
}

/* Does this instruction write state that a load in front of it might read?
 *
 * Not just memory. The rotation below moves a load's READ past this
 * instruction's WRITE, so anything the load could have read matters -- and a
 * coprocessor read is a load too:
 *
 *      mfc0 r22, cop0r20
 *      mtc0 r22, cop0r20       <- reads old r22, writes the register just read
 *
 * Rotating puts the mtc0 first and the mfc0 then reads what it wrote. Memory
 * and the two coprocessor files are the whole of what a load reads, so they
 * are the whole of this list. */
static int mips_writes_state(uint32_t insn)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;

        if (op == 0x28 || op == 0x29 || op == 0x2a ||           /* SB SH SWL */
            op == 0x2b || op == 0x2e ||                         /* SW SWR    */
            op == 0x3a)                                         /* SWC2      */
                return 1;

        if (op == 0x10)                                         /* MTC0      */
                return rs == 0x04;
        if (op == 0x12)                                         /* MTC2 CTC2 */
                return !(insn & (1u << 25)) && (rs == 0x04 || rs == 0x06);

        return op == 0x32;                                      /* LWC2      */
}

/* Did `insn` decode into exactly one load node, sitting at index `mark`?  Only
 * then is there a single node to move, and only loads have a shadow. */
static int shadow_pending(const ctx *c, uint32_t insn, int mark)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;

        /* A COPROCESSOR READ HAS THE SAME SHADOW AS A MEMORY LOAD. "When
         * reading from a coprocessor register, the next opcode cannot use the
         * destination register as operand (much the same as the Load Delays
         * that occur when reading from memory)" -- tools/docs/
         * cpuspecifications.md, Caution - Load Delay, COP section. Both MFC0
         * and MFC2/CFC2 decode to a single node, so both can be deferred the
         * same way an ordinary load is. */
        if (op == 0x10 && rs == 0x00)
                return c->n == mark + 1 ? mark : -1;
        if (op == 0x12 && !(insn & (1u << 25)) && (rs == 0x00 || rs == 0x02))
                return c->n == mark + 1 ? mark : -1;

        if (op != 0x20 && op != 0x21 && op != 0x22 &&
            op != 0x23 && op != 0x24 && op != 0x25 && op != 0x26)
                return -1;
        if (c->n != mark + 1)
                return -1;
        return mark;
}

/* The shadow instruction has just been decoded into `[mark, c->n)`.  If it
 * reads what the pending load writes, move the load node after it.  Returns
 * non-zero if it did, because the array order and the decode order have then
 * parted company and the caller must stop tracking. */
static int shadow_fix(ctx *c, int pend, uint32_t load, uint32_t insn, int mark)
{
        unsigned rd = (load >> 16) & 31;        /* the load's destination */
        unsigned rb = (load >> 21) & 31;        /* and its base           */
        ir_node hold;
        int i;

        if (pend < 0 || c->n <= mark)
                return 0;
        if (!ir_reads(insn, rd))
                return 0;
        if (mips_writes(insn, rd) || mips_writes(insn, rb))
                return 0;

        /* A STORE IN THE SHADOW CANNOT BE HANDLED BY MOVING THE LOAD.
         *
         * The rotation below models the register delay by running the load
         * last -- which also runs its memory read last, and the hardware does
         * not: the read happens at the load and only the register write is
         * delayed (`ir.h`, `defer`). If the shadow instruction writes memory,
         * it may write the very thing the load is about to read, so the load
         * is split in place instead: read now, write the register after. */
        if (mips_writes_state(insn)) {
                ir_node *ld = &c->out[pend];
                ir_node *get;
                unsigned dest = ld->rd;

                ld->defer = 1;
                get = node(c, IR_TEMP_GET, ld->pc);
                if (!get)
                        return 0;
                get->rd = (uint8_t)dest;
                return 1;
        }

        hold = c->out[pend];
        for (i = pend; i < c->n - 1; i++)
                c->out[i] = c->out[i + 1];
        c->out[c->n - 1] = hold;
        return 1;
}

/* Is this a control transfer?  The list the block terminates on. */
static int is_transfer(uint32_t insn)
{
        unsigned op = insn >> 26, fn = insn & 63;

        if (op == 0)
                return fn == 0x08 || fn == 0x09 ||       /* JR, JALR         */
                       fn == 0x0c || fn == 0x0d;         /* SYSCALL, BREAK   */
        return op == 0x01 ||                             /* REGIMM branches  */
               op == 0x02 || op == 0x03 ||               /* J, JAL           */
               (op >= 0x04 && op <= 0x07);               /* BEQ .. BGTZ      */
}

/* The transfer, and only the part of it that has to see pre-delay-slot state.
 * Returns non-zero if a delay slot follows. */
static int decode_transfer(ctx *c, uint32_t insn, uint32_t pc)
{
        unsigned op = insn >> 26;
        unsigned rs = (insn >> 21) & 31;
        unsigned rt = (insn >> 16) & 31;
        unsigned rd = (insn >> 11) & 31;
        unsigned fn = insn & 63;
        uint32_t simm = (uint32_t)(int32_t)(int16_t)(insn & 0xffff);
        uint32_t target = pc + 4 + (simm << 2);
        uint32_t fall = pc + 8;
        ir_node *p;

        if (op == 0) {
                if (fn == 0x0c || fn == 0x0d) {         /* SYSCALL / BREAK */
                        p = node(c, IR_STOP, pc);
                        if (p) {
                                /* R3000A exception codes: Sys is 8, Bp is 9.
                                 * The service has to tell them apart, and only
                                 * one of them looks at $a0. */
                                p->sub = (uint8_t)(fn == 0x0c ? 8 : 9);
                                p->imm = pc;
                        }
                        return 0;                       /* no delay slot runs */
                }
                /* JALR links before the target is read, so a JALR through r31
                 * jumps to the link value.  The reference does the same; this
                 * is where the two have to agree. */
                if (fn == 0x09)
                        emit_set(c, pc, rd, fall);
                p = node(c, IR_CAPTURE, pc);
                if (p)
                        p->rs = (uint8_t)rs;
                return 1;
        }

        if (op == 0x02 || op == 0x03) {                 /* J / JAL */
                if (op == 0x03)
                        emit_set(c, pc, 31, fall);
                p = node(c, IR_JUMP, pc);
                if (p)
                        p->imm = (pc & 0xf0000000u) | ((insn & 0x03ffffffu) << 2);
                return 1;
        }

        if (op == 0x01) {                               /* REGIMM */
                unsigned cc;

                /* The link forms link unconditionally, before the test. */
                if (rt == 0x10 || rt == 0x11)
                        emit_set(c, pc, 31, fall);

                if (rt == 0x00 || rt == 0x10)
                        cc = CC_LTZ;
                else if (rt == 0x01 || rt == 0x11)
                        cc = CC_GEZ;
                else {
                        /* Not a branch at all: never taken, slot still runs. */
                        p = node(c, IR_JUMP, pc);
                        if (p)
                                p->imm = fall;
                        return 1;
                }

                p = node(c, IR_COND, pc);
                if (p) {
                        p->sub = (uint8_t)cc;
                        p->rs = (uint8_t)rs;
                        p->imm = target;
                        p->imm2 = fall;
                }
                return 1;
        }

        p = node(c, IR_COND, pc);
        if (p) {
                p->sub = (uint8_t)(op == 0x04 ? CC_EQ :
                                   op == 0x05 ? CC_NE :
                                   op == 0x06 ? CC_LEZ : CC_GTZ);
                p->rs = (uint8_t)rs;
                p->rt = (uint8_t)(op <= 0x05 ? rt : 0);
                p->imm = target;
                p->imm2 = fall;
        }
        return 1;
}

int ir_block_length(const uint32_t *words)
{
        int i;

        for (i = 0; i < IR_MAX_INSNS; i++)
                if (is_transfer(words[i])) {
                        unsigned fn = words[i] & 63;

                        /* SYSCALL and BREAK have no delay slot: the block
                         * stops at the instruction itself. */
                        if ((words[i] >> 26) == 0 && (fn == 0x0c || fn == 0x0d))
                                return i + 1;
                        return i + 2;
                }
        return IR_MAX_INSNS;
}

unsigned ir_delay_slot_load(const uint32_t *words)
{
        int i;

        for (i = 0; i < IR_MAX_INSNS; i++) {
                uint32_t slot;
                unsigned op, fn;

                if (!is_transfer(words[i]))
                        continue;

                fn = words[i] & 63;
                if ((words[i] >> 26) == 0 && (fn == 0x0c || fn == 0x0d))
                        return 0;               /* no delay slot runs */

                slot = words[i + 1];
                op = slot >> 26;
                if (op != 0x20 && op != 0x21 && op != 0x22 &&
                    op != 0x23 && op != 0x24 && op != 0x25 && op != 0x26)
                        return 0;
                return (slot >> 16) & 31;       /* 0 discards, so 0 is "none" */
        }
        return 0;                               /* ran out before a transfer */
}

int ir_decode(const uint32_t *words, uint32_t pc, ir_node *out, int max)
{
        ctx c;
        int i;
        int pend = -1;                  /* node index of a load in shadow */
        uint32_t pend_insn = 0;

        c.out = out;
        c.n = 0;
        c.max = max;

        for (i = 0; i < IR_MAX_INSNS; i++) {
                uint32_t insn = words[i];
                uint32_t at = pc + 4u * (uint32_t)i;
                int mark = c.n;

                if (!is_transfer(insn)) {
                        decode_op(&c, insn, at);
                        /* A rotation leaves this instruction's own node behind
                         * the load's, so its index is no longer `mark` and a
                         * second rotation on top of it would be wrong.  One
                         * hazard per pair is the whole of what real code has
                         * anyway. */
                        pend = shadow_fix(&c, pend, pend_insn, insn, mark)
                                       ? -1
                                       : shadow_pending(&c, insn, mark);
                        pend_insn = insn;
                        continue;
                }

                /* The transfer's own nodes sit ahead of the delay slot, so a
                 * load shadowed by the transfer has to be placed between the
                 * two — which is where this call is. */
                if (decode_transfer(&c, insn, at)) {
                        shadow_fix(&c, pend, pend_insn, insn, mark);
                        decode_op(&c, words[i + 1], at + 4);
                } else {
                        shadow_fix(&c, pend, pend_insn, insn, mark);
                }
                return c.n;
        }

        /* Ran out of block before finding a transfer.  The next block starts
         * at the following instruction; nothing else is different. */
        {
                ir_node *p = node(&c, IR_JUMP, pc);

                if (p)
                        p->imm = pc + 4u * IR_MAX_INSNS;
        }
        return c.n;
}
