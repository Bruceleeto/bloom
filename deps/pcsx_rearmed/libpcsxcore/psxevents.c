#include <stddef.h>
#include <stdio.h>
#include "r3000a.h"
#include "cdrom.h"
#include "psxdma.h"
#include "mdec.h"
#include "psxevents.h"
#include "deadline.h"
#include "perf.h"

//#define evprintf printf
#define evprintf(...)

static psxRegisters *cp0TOpsxRegs(psxCP0Regs *cp0)
{
#ifndef LIGHTREC
	return (void *)((char *)cp0 - offsetof(psxRegisters, CP0));
#else
	// lightrec has it's own cp0
	return &psxRegs;
#endif
}

/* WHAT IS PENDING, AND HOW LATE.  One line a second under the deadline: the
 * event mask, and each armed event's due cycle as a signed distance from now.
 * A positive number is an event still to come; a negative one is an event that
 * is overdue and has not been drained, which is what a lost event looks like
 * from the outside. */
u32 fgl_last_mmio;
int fgl_dump_pending;

/* THE LOST-IRQ TRACE.  I_STAT bit 2 goes clear while the CD still holds an
 * unacknowledged INT3, so the BIOS polls for a bit that will never be set
 * again.  Which of the three writes lands out of order is not guessable from
 * a once-a-second snapshot, so keep the last few of them: setIrq raising the
 * bit, the guest acking the CD register, and the guest acking I_STAT.  The
 * ring is dumped at the stuck point, so it shows the ordering that led there
 * rather than the steady state after. */
#define FGL_TRACE_N 24
static struct { u32 cycle, a, b, c; char tag; } fgl_trace_ring[FGL_TRACE_N];
static u32 fgl_trace_pos;

void fgl_trace3(char tag, u32 a, u32 b, u32 c)
{
	u32 i = fgl_trace_pos++ & (FGL_TRACE_N - 1);

	fgl_trace_ring[i].cycle = psxRegs.cycle;
	fgl_trace_ring[i].tag = tag;
	fgl_trace_ring[i].a = a;
	fgl_trace_ring[i].b = b;
	fgl_trace_ring[i].c = c;
}

void fgl_trace(char tag, u32 a, u32 b)
{
	fgl_trace3(tag, a, b, 0);
}

static void fgl_trace_dump(void)
{
	u32 n = fgl_trace_pos < FGL_TRACE_N ? fgl_trace_pos : FGL_TRACE_N;
	u32 i;

	for (i = 0; i < n; i++) {
		u32 k = (fgl_trace_pos - n + i) & (FGL_TRACE_N - 1);
		printf("  tr %c %u %08x %08x %08x\n", fgl_trace_ring[k].tag,
		       fgl_trace_ring[k].cycle, fgl_trace_ring[k].a,
		       fgl_trace_ring[k].b, fgl_trace_ring[k].c);
	}
}

void fgl_dump_events(void)
{
	const psxRegisters *regs = &psxRegs;
	u32 i, irqs = regs->interrupt;

	{
		u32 istat, imask, pend;
		fgl_cdr_state(&istat, &imask, &pend);
		printf("cd: irqstat %02x irqmask %02x irq1pending %02x |"
		       " I_STAT %04x I_MASK %04x SR %08x\n",
		       istat, imask, pend, psxHu32(0x1070) & 0xffff,
		       psxHu32(0x1074) & 0xffff, psxRegs.CP0.n.SR);
	}

	printf("ev: pc %08x mmio %08x mask %08x cycle %u next %+d |",
	       regs->pc, fgl_last_mmio, irqs, regs->cycle,
	       (int)(regs->next_interupt - regs->cycle));

	for (i = 0; irqs != 0; i++, irqs >>= 1) {
		if (!(irqs & 1))
			continue;
		printf(" %u:%+d", i, (int)(regs->event_cycles[i] - regs->cycle));
	}
	printf("\n");
	fgl_trace_dump();
}

u32 schedule_timeslice(psxRegisters *regs)
{
	u32 i, c = regs->cycle;
	u32 irqs = regs->interrupt;
	s32 min, dif;

	min = PSXCLK;
	for (i = 0; irqs != 0; i++, irqs >>= 1) {
		if (!(irqs & 1))
			continue;
		dif = regs->event_cycles[i] - c;
		//evprintf("  ev %d\n", dif);
		/* An event due NOW has dif == 0, and the original test wanted
		 * dif strictly positive -- so it was skipped, nothing else was
		 * pending, and the slice got sized to a whole PSXCLK with the
		 * event still sitting there.  A counting clock never lands on
		 * that case; a deadline clock arrives late by construction and
		 * hits it constantly. */
		if (FGL_DEADLINE && dif <= 0) {
			min = 0;
			break;
		}
		if (0 < dif && dif < min)
			min = dif;
	}
	regs->next_interupt = c + min;
	return regs->next_interupt;
}

static void irqNoOp() {
}

typedef void (irq_func)();

static irq_func * const irq_funcs[] = {
	[PSXINT_SIO]	= sioInterrupt,
	[PSXINT_CDR]	= cdrInterrupt,
	[PSXINT_CDREAD]	= cdrPlayReadInterrupt,
	[PSXINT_GPUDMA]	= gpuInterrupt,
	[PSXINT_MDECOUTDMA] = mdec1Interrupt,
	[PSXINT_SPUDMA]	= spuInterrupt,
	[PSXINT_MDECINDMA] = mdec0Interrupt,
	[PSXINT_GPUOTCDMA] = gpuotcInterrupt,
	[PSXINT_CDRDMA] = cdrDmaInterrupt,
	[PSXINT_NEWDRC_CHECK] = irqNoOp,
	[PSXINT_CDRLID] = cdrLidSeekInterrupt,
	[PSXINT_IRQ10] = irq10Interrupt,
	[PSXINT_SPU_UPDATE] = spuUpdate,
	[PSXINT_SPU_IRQ] = spuDelayedIrq,
	[PSXINT_RCNT] = psxRcntUpdate,
};

static int events_due(const psxRegisters *regs)
{
	u32 irq, irq_bits, cycle = regs->cycle;

	for (irq = 0, irq_bits = regs->interrupt; irq_bits != 0; irq++, irq_bits >>= 1) {
		if (!(irq_bits & 1))
			continue;
		if ((s32)(cycle - regs->event_cycles[irq]) >= 0)
			return 1;
	}
	return 0;
}

/* Only the handlers.  irq_test()'s Cause and exception work must not be
 * repeated per pass -- doing so raises the guest's exception over and over. */
static void fire_due_events(psxRegisters *regs)
{
	u32 irq, irq_bits, cycle = regs->cycle;

	for (irq = 0, irq_bits = regs->interrupt; irq_bits != 0; irq++, irq_bits >>= 1) {
		if (!(irq_bits & 1))
			continue;
		if ((s32)(cycle - regs->event_cycles[irq]) >= 0) {
			// note: irq_funcs() also modify regs->interrupt
			regs->interrupt &= ~(1u << irq);
			irq_funcs[irq]();
		}
	}
}

/* THE DRAIN AT A CROSSING, AND WHY IT REPLACES HOLDING THE CLOCK BACK.
 *
 * A device model must never see a clock that has already run past an event
 * that has not fired -- cdrom.c does arithmetic against pending event cycles
 * and none of it was written for `now` to be on the far side.  There are only
 * two ways to honour that.  The old one was to CLAMP the charge to the room
 * before the next event and carry the rest in a pool, which is where owed,
 * OWED_MAX, OVERSHOOT and the dropped-time accounting all came from: the pool
 * grows whenever the guest's own schedule leaves smaller gaps than real time
 * fills, and then the excess has to be thrown away.
 *
 * This is the other way, and it is what bleem does: let the clock be where it
 * really is and FIRE the events that are behind it, before the handler runs.
 * Nothing is owed because nothing is held back, so the pool and every constant
 * attached to it go away.
 *
 * NOT irq_test: that ends in psxException, which moves the guest PC.  Here the
 * guest is mid-block at an MMIO access and the PC must not move.  Cause is
 * still updated, and that is the entire point -- it is what makes
 * has_interrupt() true on the way back in, so the slice ends at the next block
 * boundary and the exception is delivered there.  Which is where bleem
 * delivers it too.
 *
 * The loop is gen_interupt's: a handler can reschedule itself for now
 * (psxRcntSet does whenever a counter ran late), and schedule_timeslice only
 * reports that by handing back next_interupt == cycle. */
void events_run_due(psxCP0Regs *cp0)
{
	psxRegisters *regs = cp0TOpsxRegs(cp0);
	int guard = 1024;

	do {
		fire_due_events(regs);
	} while (schedule_timeslice(regs) == regs->cycle && !regs->stop
		 && --guard > 0);

	cp0->n.Cause &= ~0x400;
	if (psxHu32(0x1070) & psxHu32(0x1074))
		cp0->n.Cause |= 0x400;
}

void irq_test(psxCP0Regs *cp0)
{
	psxRegisters *regs = cp0TOpsxRegs(cp0);
	u32 cycle = regs->cycle;
	u32 irq, irq_bits;

	for (irq = 0, irq_bits = regs->interrupt; irq_bits != 0; irq++, irq_bits >>= 1) {
		if (!(irq_bits & 1))
			continue;
		if ((s32)(cycle - regs->event_cycles[irq]) >= 0) {
			// note: irq_funcs() also modify regs->interrupt
			regs->interrupt &= ~(1u << irq);
			{
				/* Split `evt` by source: this loop is the
				 * whole of it, and which device is costing
				 * the milliseconds is not guessable. */
				PERF_BEGIN_AT(t0);
				irq_funcs[irq]();
				PERF_END_EVT(t0, irq < PERF_EVT_N ? irq
							  : PERF_EVT_N - 1);
			}
		}
	}

	cp0->n.Cause &= ~0x400;
	if (psxHu32(0x1070) & psxHu32(0x1074))
		cp0->n.Cause |= 0x400;
	if (((cp0->n.Cause | 1) & cp0->n.SR & 0x401) == 0x401)
		psxException(0, 0, cp0);
}

void gen_interupt(psxCP0Regs *cp0)
{
	psxRegisters *regs = cp0TOpsxRegs(cp0);

	evprintf("  +ge %08x, %u->%u (%d)\n", regs->pc, regs->cycle,
		regs->next_interupt, regs->next_interupt - regs->cycle);

	/* The deadline advances regs->cycle in steps that can land past
	 * next_interupt, so several events may be due at once and servicing one
	 * can leave another overdue.  schedule_timeslice() only looks at events
	 * strictly in the future, so anything still overdue here would be
	 * pushed a whole PSXCLK away.  Terminates because each pass either
	 * services an event, rearming it at least a period ahead, or clears it. */
	if (FGL_DEADLINE) {
		int guard = 1024;

		while (events_due(regs) && --guard > 0)
			fire_due_events(regs);
	}

	/* irq_test can itself reschedule an event for now -- psxRcntSet does it
	 * whenever a counter update ran late.  The drain above cannot see that
	 * one, so keep going until the slice this sizes is actually non-empty. */
	if (FGL_DEADLINE) {
		do {
			irq_test(cp0);
		} while (schedule_timeslice(regs) == regs->cycle && !regs->stop);
	} else {
		irq_test(cp0);
		schedule_timeslice(regs);
	}

	evprintf("  -ge %08x, %u->%u (%d)\n", regs->pc, regs->cycle,
		regs->next_interupt, regs->next_interupt - regs->cycle);
}

void events_restore(void)
{
	int i;
	for (i = 0; i < PSXINT_COUNT; i++)
		psxRegs.event_cycles[i] = psxRegs.intCycle[i].sCycle + psxRegs.intCycle[i].cycle;

	psxRegs.event_cycles[PSXINT_RCNT] = psxRegs.psxNextsCounter + psxRegs.psxNextCounter;
	psxRegs.interrupt |=  1 << PSXINT_RCNT;
	psxRegs.interrupt &= (1 << PSXINT_COUNT) - 1;
}
