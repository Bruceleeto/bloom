#ifndef __PSXEVENTS_H__
#define __PSXEVENTS_H__

#include "psxcommon.h"

enum {
	PSXINT_SIO = 0,      // sioInterrupt
	PSXINT_CDR,          // cdrInterrupt
	PSXINT_CDREAD,       // cdrPlayReadInterrupt
	PSXINT_GPUDMA,       // gpuInterrupt
	PSXINT_MDECOUTDMA,   // mdec1Interrupt
	PSXINT_SPUDMA,       // spuInterrupt
	PSXINT_SPU_IRQ,      // spuDelayedIrq
	PSXINT_MDECINDMA,    // mdec0Interrupt
	PSXINT_GPUOTCDMA,    // gpuotcInterrupt
	PSXINT_CDRDMA,       // cdrDmaInterrupt
	PSXINT_NEWDRC_CHECK, // (none)
	PSXINT_RCNT,         // psxRcntUpdate
	PSXINT_CDRLID,       // cdrLidSeekInterrupt
	PSXINT_IRQ10,        // irq10Interrupt
	PSXINT_SPU_UPDATE,   // spuUpdate
	PSXINT_COUNT
};

#define set_event_raw_abs(e, abs) { \
	u32 abs_ = abs; \
	s32 di_ = psxRegs.next_interupt - abs_; \
	psxRegs.event_cycles[e] = abs_; \
	if (di_ > 0) { \
		/*printf("%u: next_interupt %u -> %u\n", psxRegs.cycle, psxRegs.next_interupt, abs_);*/ \
		psxRegs.next_interupt = abs_; \
	} \
}

#define set_event(e, c) do { \
	psxRegs.interrupt |= (1 << (e)); \
	psxRegs.intCycle[e].cycle = c; \
	psxRegs.intCycle[e].sCycle = psxRegs.cycle; \
	set_event_raw_abs(e, psxRegs.cycle + (c)) \
} while (0)

union psxCP0Regs_;
struct psxRegisters;

u32  schedule_timeslice(struct psxRegisters *regs);
void fgl_dump_events(void);
void fgl_cdr_state(u32 *irqstat, u32 *irqmask, u32 *irq1pending);
/* tags: 'S' cd setIrq, 'A' guest acks the cd register, 'I' guest writes I_STAT,
 * 'B' guest requests sector data (BFRD), 'D' DMA3 moves it into guest RAM */
void fgl_trace(char tag, u32 a, u32 b);
void fgl_trace3(char tag, u32 a, u32 b, u32 c);
extern u32 fgl_last_mmio;   /* address of the most recent MMIO crossing */
extern int fgl_dump_pending;/* set once a second, consumed at the next hook */
void irq_test(union psxCP0Regs_ *cp0);
/* Fire everything already due, update Cause, do NOT deliver.  For an MMIO
 * crossing, where the guest PC must not move.  See psxevents.c. */
void events_run_due(union psxCP0Regs_ *cp0);
void gen_interupt(union psxCP0Regs_ *cp0);
void events_restore(void);

#endif // __PSXEVENTS_H__
