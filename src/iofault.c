/* See iofault.h.  The design and the verified facts behind every constant
 * here (register mapping, wrapper cycle contract, exception codes) are in
 * plan.md under "MMU-FAULT I/O CLASSIFICATION".
 *
 * Shape: the DTLB miss handler must not call device code — device paths
 * take locks, which are illegal in exception context.  So the handler only
 * decodes the faulting access, stashes it, points the interrupted pc at
 * iofault_stub and returns.  The stub, running in the interrupted (thread)
 * context, saves r0-r12/pr/mach/macl in a stack frame, calls
 * iofault_dispatch(), restores, and jumps to the resume address.  The
 * dispatch writes a load's result directly into the frame slot of the
 * destination register, so the stub's restore doubles as the writeback.
 *
 * The wrapper cycle contract this mirrors (lightrec.c, C wrapper emission):
 *   in:  state->current_cycle = state->target_cycle - r1
 *   out: r1 = state->target_cycle - state->current_cycle
 * r1 is lightning's JIT_R0, the down-counting scaled cycle counter; the
 * dispatch applies the contract to the frame's r1 slot.
 *
 * FPU state is not saved: generated code keeps nothing live in the scalar
 * bank across C calls and the GTE state lives in XMTRX/the back bank —
 * the same contract the direct GTE calls already rely on.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <arch/arch.h>
#include <arch/irq.h>

#include <libpcsxcore/lightrec/mem.h>

#include "iofault.h"

#define TEA (*(volatile uint32_t *)0xff00000c)

#define IO_BASE		0x1f801000u
#define IO_SIZE		0x2000u

#define IOFAULT_NO_DEST	0xffffffffu

/* From the lightrec plugin (plugin.c) */
struct lightrec_state;
struct lightrec_state *bloom_lightrec_state(void);
uint32_t bloom_iofault_read(uint32_t mem, unsigned int size);
void bloom_iofault_write(uint32_t mem, uint32_t val, unsigned int size);

/* lightrec public API */
uint32_t lightrec_current_cycle_count(const struct lightrec_state *state);
uint32_t lightrec_target_cycle_count(const struct lightrec_state *state);
void lightrec_reset_cycle_count(struct lightrec_state *state, uint32_t cycles);
_Bool lightrec_backpatch_io(struct lightrec_state *state, uint32_t host_pc);

static uint32_t iofault_reads, iofault_writes, iofault_strays;
static uint32_t iofault_patched;

/* Register frame the stub builds on the interrupted stack; layout must
 * match the stub's push order exactly (r1 shallowest, r0 deepest). */
struct iofault_frame {
	uint32_t r1_12[12];		/* r1..r12 */
	uint32_t macl, mach, pr, r0;
};

/* Access stashed by the handler for the stub's dispatch.  Single-flight:
 * nothing between handler return and stub completion touches the guest
 * window, so a second fault cannot preempt it. */
static struct {
	uint32_t addr;
	uint32_t val;			/* store value */
	uint32_t size;			/* bytes: 1, 2 or 4 */
	uint32_t dest;			/* load dest 0..12, or IOFAULT_NO_DEST */
	uint32_t fault_pc;		/* host address of the faulting insn */
} iofault_pending;

/* Referenced only from the stub's literal pool, which LTO can't see —
 * both must be pinned as used + externally visible or the link fails. */
__attribute__((used, externally_visible))
uint32_t iofault_resume_pc;

extern void iofault_stub(void);

void iofault_dispatch(struct iofault_frame *frame);

__attribute__((used, externally_visible))
void iofault_dispatch(struct iofault_frame *frame)
{
	struct lightrec_state *state = bloom_lightrec_state();
	uint32_t val;

	lightrec_reset_cycle_count(state,
			lightrec_target_cycle_count(state) - frame->r1_12[0]);

	if (iofault_pending.dest == IOFAULT_NO_DEST) {
		bloom_iofault_write(iofault_pending.addr, iofault_pending.val,
				    iofault_pending.size);
		iofault_writes++;
	} else {
		val = bloom_iofault_read(iofault_pending.addr,
					 iofault_pending.size);

		/* mov.b and mov.w sign-extend into the register */
		if (iofault_pending.size == 1)
			val = (uint32_t)(int32_t)(int8_t)val;
		else if (iofault_pending.size == 2)
			val = (uint32_t)(int32_t)(int16_t)val;

		if (iofault_pending.dest == 0)
			frame->r0 = val;
		else
			frame->r1_12[iofault_pending.dest - 1] = val;
		iofault_reads++;
	}

	frame->r1_12[0] = lightrec_target_cycle_count(state)
		- lightrec_current_cycle_count(state);

	/* This site always hits I/O when it faults; re-tag its opcode so the
	 * block recompiles with the C-wrapper access and stops faulting. */
	if (lightrec_backpatch_io(state, iofault_pending.fault_pc))
		iofault_patched++;
}

/* The stub the handler resumes into.  Runs on the interrupted thread's
 * stack (r15 is always a valid stack pointer in generated code).  The
 * final jmp uses the resume address in r0 while the delay slot pops the
 * real r0 — the jump target is latched when jmp executes, so the delay
 * slot rewriting r0 is fine. */
__asm__(
"	.text\n"
"	.align	2\n"
"	.globl	_iofault_stub\n"
"_iofault_stub:\n"
"	mov.l	r0,@-r15\n"
"	sts.l	pr,@-r15\n"
"	sts.l	mach,@-r15\n"
"	sts.l	macl,@-r15\n"
"	mov.l	r12,@-r15\n"
"	mov.l	r11,@-r15\n"
"	mov.l	r10,@-r15\n"
"	mov.l	r9,@-r15\n"
"	mov.l	r8,@-r15\n"
"	mov.l	r7,@-r15\n"
"	mov.l	r6,@-r15\n"
"	mov.l	r5,@-r15\n"
"	mov.l	r4,@-r15\n"
"	mov.l	r3,@-r15\n"
"	mov.l	r2,@-r15\n"
"	mov.l	r1,@-r15\n"
"	mov.l	1f,r0\n"
"	jsr	@r0\n"
"	 mov	r15,r4\n"
"	mov.l	2f,r0\n"
"	mov.l	@r0,r0\n"
"	mov.l	@r15+,r1\n"
"	mov.l	@r15+,r2\n"
"	mov.l	@r15+,r3\n"
"	mov.l	@r15+,r4\n"
"	mov.l	@r15+,r5\n"
"	mov.l	@r15+,r6\n"
"	mov.l	@r15+,r7\n"
"	mov.l	@r15+,r8\n"
"	mov.l	@r15+,r9\n"
"	mov.l	@r15+,r10\n"
"	mov.l	@r15+,r11\n"
"	mov.l	@r15+,r12\n"
"	lds.l	@r15+,macl\n"
"	lds.l	@r15+,mach\n"
"	lds.l	@r15+,pr\n"
"	jmp	@r0\n"
"	 mov.l	@r15+,r0\n"
"	.align	2\n"
"1:	.long	_iofault_dispatch\n"
"2:	.long	_iofault_resume_pc\n"
);

static void iofault_die(irq_context_t *ctx, uint32_t addr, const char *why)
{
	printf("IOFAULT: unhandled fault: %s\n"
	       "IOFAULT: pc 0x%08lx  addr 0x%08lx  insn 0x%04x\n",
	       why, (unsigned long)ctx->pc, (unsigned long)addr,
	       ctx->pc ? *(uint16_t *)ctx->pc : 0);
	fflush(stdout);
	arch_abort();
}

/* True for every SH-4 opcode that has a delay slot.  If the faulting access
 * sits in one, SPC points at the branch itself and skip-by-2 would be wrong;
 * lightning does not emit guest accesses in delay slots, so this existing is
 * a bug worth dying loudly on. */
static int insn_has_delay_slot(uint16_t op)
{
	if ((op >> 12) == 0xa || (op >> 12) == 0xb)	/* bra, bsr */
		return 1;
	if ((op & 0xfd00) == 0x8d00)			/* bt/s, bf/s */
		return 1;
	if ((op & 0xf0ff) == 0x402b || (op & 0xf0ff) == 0x400b) /* jmp, jsr */
		return 1;
	if (op == 0x000b || op == 0x002b)		/* rts, rte */
		return 1;
	return 0;
}

static void iofault_dtlb(irq_t code, irq_context_t *ctx, void *data)
{
	uint32_t addr = TEA;
	unsigned int size, reg;
	bool is_io;
	uint16_t op;

	(void)data;

	if (ctx->pc - (uint32_t)code_buffer >= CODE_BUFFER_SIZE)
		iofault_die(ctx, addr, "fault from outside generated code");

	op = *(uint16_t *)ctx->pc;

	if (insn_has_delay_slot(op))
		iofault_die(ctx, addr, "guest access in a delay slot");

	/* Anything outside the I/O window is a stray guest access — the
	 * cache-control register at 0x1ffe0130, or plain game bugs.  The old
	 * generic wrapper tolerated those, so this path must too: writes are
	 * dropped, reads return all-ones. */
	is_io = addr - IO_BASE < IO_SIZE;
	if (!is_io && iofault_strays++ < 4) {
		printf("iofault: stray %s at 0x%08lx from pc 0x%08lx\n",
		       code == EXC_DTLB_MISS_READ ? "read" : "write",
		       (unsigned long)addr, (unsigned long)ctx->pc);
	}

	if (code == EXC_DTLB_MISS_READ) {
		/* mov.b/w/l @Rm,Rn (6nm0/1/2); mov.l @(disp,Rm),Rn (5nmd);
		 * mov.b/w @(disp,Rm),R0 (84md/85md);
		 * mov.b/w/l @(R0,Rm),Rn (0nmC/D/E) */
		switch (op >> 12) {
		case 0x6:
			size = op & 0x3;
			reg = (op >> 8) & 0xf;
			break;
		case 0x5:
			size = 2;
			reg = (op >> 8) & 0xf;
			break;
		case 0x8:
			size = (op >> 8) & 0xf;
			if (size != 0x4 && size != 0x5)
				iofault_die(ctx, addr, "unknown load opcode");
			size -= 0x4;
			reg = 0;
			break;
		case 0x0:
			size = (op & 0xf) - 0xc;
			if (size > 2)
				iofault_die(ctx, addr, "unknown load opcode");
			reg = (op >> 8) & 0xf;
			break;
		default:
			iofault_die(ctx, addr, "unknown load opcode");
			return;
		}

		if (!is_io) {
			ctx->r[reg] = 0xffffffff;
			ctx->pc += 2;
			return;
		}

		/* r1 is the cycle channel, r13+ are not in the stub frame */
		if (reg == 1 || reg >= 13)
			iofault_die(ctx, addr, "load into a reserved register");

		iofault_pending.dest = reg;
	} else {
		/* mov.b/w/l Rm,@Rn (2nm0/1/2); mov.l Rm,@(disp,Rn) (1nmd);
		 * mov.b/w R0,@(disp,Rn) (80nd/81nd);
		 * mov.b/w/l Rm,@(R0,Rn) (0nm4/5/6) */
		switch (op >> 12) {
		case 0x2:
			size = op & 0x3;
			reg = (op >> 4) & 0xf;
			break;
		case 0x1:
			size = 2;
			reg = (op >> 4) & 0xf;
			break;
		case 0x8:
			size = (op >> 8) & 0xf;
			if (size > 1)
				iofault_die(ctx, addr, "unknown store opcode");
			reg = 0;
			break;
		case 0x0:
			size = (op & 0xf) - 0x4;
			if (size > 2)
				iofault_die(ctx, addr, "unknown store opcode");
			reg = (op >> 4) & 0xf;
			break;
		default:
			iofault_die(ctx, addr, "unknown store opcode");
			return;
		}

		if (!is_io) {
			ctx->pc += 2;
			return;
		}

		iofault_pending.dest = IOFAULT_NO_DEST;
		iofault_pending.val = ctx->r[reg];
	}

	/* Hand the access to the stub, in the interrupted context */
	iofault_pending.addr = addr;
	iofault_pending.size = 1u << size;
	iofault_pending.fault_pc = ctx->pc;
	iofault_resume_pc = ctx->pc + 2;
	ctx->pc = (uint32_t)iofault_stub;
}

void iofault_init(void)
{
	irq_set_handler(EXC_DTLB_MISS_READ, iofault_dtlb, NULL);
	irq_set_handler(EXC_DTLB_MISS_WRITE, iofault_dtlb, NULL);

	printf("iofault: DTLB miss handlers armed, I/O window 0x%08x +0x%x\n",
	       IO_BASE, IO_SIZE);
}

void iofault_report(void)
{
	if (iofault_reads || iofault_writes || iofault_strays)
		printf("iofault: %lu reads, %lu writes serviced by fault"
		       " (%lu strays, %lu sites backpatched)\n",
		       (unsigned long)iofault_reads,
		       (unsigned long)iofault_writes,
		       (unsigned long)iofault_strays,
		       (unsigned long)iofault_patched);
}
