/* THE DEADLINE SCHEDULER.
 *
 * Generated code contains no cycle arithmetic at all.  r14 is a gate, not a
 * countdown: non-zero means keep running, zero means go to the dispatcher.
 * Nothing inside emitted code ever moves it.  Two things close it -- the
 * dispatcher's own test when a slice is asked to end, and TMU2 firing.
 *
 * That leaves the guest's clock with nothing to advance it, so the charge is
 * made at the crossings instead, and what it charges is WORK: the number of
 * SH-4 instruction slots issued since the last crossing, read from PRFC1.  One
 * read per crossing, and everything since the previous one is charged -- the
 * guest's translated code and the C the crossing itself runs, which is
 * proportional to guest work and so falls inside K.  Only host work with no
 * guest cause is excluded, by an explicit pause/resume pair.  Charging elapsed
 * time instead would weld the guest's clock to the host's speed, and no
 * constant can undo that -- which is why openbios' CD wait, a spin loop that
 * touches no hardware, is the test case that kills every wall-clock variant.
 *
 * The charge is never clamped to the remaining budget.  A clamp looks harmless
 * and turns the charge straight back into a time quantity in exactly the case
 * that matters: when the guest did more work than the slice had room for.
 */

#include <stdio.h>

#include <arch/irq.h>
#include <dc/perfctr.h>
#include <arch/timer.h>

#include "deadline.h"

unsigned int fgl_deadline_arms, fgl_deadline_fires, fgl_deadline_in_code;
unsigned int fgl_deadline_cycles, fgl_deadline_insns;
unsigned int fgl_dl_crossings, fgl_dl_redundant;
unsigned int fgl_dl_owed_hi;

/* THE WORK CLOCK.  PRFC1 in instructions-issued mode.  KOS uses PRFC0 for its
 * own timing and leaves PRFC1 alone.  See src/dlcheck.c, which must pass on
 * hardware before any of this is worth believing.
 *
 * Read as a single 32-bit load of the counter's low word, not through
 * perf_cntr_count().  That helper reads the high word, the low word and the
 * high word again to catch a carry, so it is three uncached peripheral-bus
 * accesses; this path runs on every guest MMIO access -- 435,000 a second in
 * the BIOS -- and six P4 round trips per access was measurably the whole cost
 * of the deadline.  The low word alone is sound because only differences are
 * ever used and a difference never approaches 2^32: at 200 MHz the low word
 * takes twenty-one seconds to wrap, and the gap between two samples is a few
 * hundred.  Unsigned subtraction carries correctly across the wrap. */
#define PMCTR1_LOW	(*(volatile uint32_t *)0xff100010)

static uint32_t dl_mark;		/* counter at the last sample */
static int dl_excluded;			/* nesting depth of pause/resume */
static uint32_t dl_insns;		/* work not yet charged */
static uint32_t dl_frac;		/* 1/1024ths carried between charges */

/* TMU2, the alarm.  DL_NUM/DL_SHIFT converts guest cycles to ticks of the
 * 12468720 Hz peripheral clock: 12468720/33868800 = 3016/8192. */
#define TIMER_BASE	0xffd80000
#define TSTR		0x04
#define TCOR2		0x20
#define TCNT2		0x24
#define TCR2		0x28

/* NOT timer_stop().  KOS's timer_stop() calls timer_disable_ints() first, so
 * it does not mean "clear the start bit", it means "stop the channel and mask
 * it at the INTC".  Using it here re-masked TMU2 on every arm, which is why
 * the alarm counted down, set UNF, and never once raised an interrupt.  All
 * that is wanted is the start bit. */
#define DL_TIMER_HALT()	(*(volatile uint8_t *)(TIMER_BASE + TSTR) &= ~0x04)

#define DL_NUM		3016u
#define DL_SHIFT	13

static volatile uint32_t *dl_cur, *dl_tgt;
static uint32_t dl_code_lo, dl_code_hi;
static volatile int dl_fired;

/* ONE SAMPLE PER CROSSING, not two.
 *
 * The charge is taken where the guest leaves generated code, and everything
 * since the previous crossing is charged: the guest's own translated code and
 * the C the crossing itself runs.  That C is not free, but it is PROPORTIONAL
 * to guest work -- an MMIO wrapper runs because the guest touched hardware, a
 * compile runs because the guest reached new code -- so a fitted K absorbs it,
 * and paying a second counter read to subtract it out cost more than the thing
 * being subtracted.
 *
 * Pause and resume survive for the host work that is NOT proportional: waiting
 * on the scanout, blocking on audio.  Time spent there has no guest cause, and
 * charging it would let a slow frame inflate the guest's clock. */
static void dl_accum(void)
{
	uint32_t now = PMCTR1_LOW;

	if (!dl_excluded)
		dl_insns += now - dl_mark;

	dl_mark = now;
}

/* A DEPTH, NOT A FLAG.  The hook is bracketed, and things that run inside it
 * -- a console print, the compiler -- are bracketed too, so these nest.  With
 * a flag the inner resume reopens the interval and charges the rest of the
 * hook to the guest. */
void fgl_deadline_pause(void)
{
	fgl_dl_crossings++;
	dl_accum();
	dl_excluded++;
}

/* THE CHEAP HALF OF A PAUSE.  `pause` reads PRFC1 to close the interval before
 * it stops counting.  At an MMIO crossing that read has already happened --
 * `lightrec_tansition_to_pcsx` charges (deadline) or settles (fit) on the way
 * in, and both go through dl_accum, so dl_mark is current -- and a second read
 * of the same value costs a peripheral access to learn nothing.
 *
 * Only correct immediately after a charge or a settle.  Anywhere else, pause. */
void fgl_deadline_exclude_begin(void)
{
	fgl_dl_crossings++;
	dl_excluded++;
}

void fgl_deadline_resume(void)
{
	if (!dl_excluded) {
		fgl_dl_redundant++;
		return;
	}

	if (--dl_excluded == 0)
		dl_mark = PMCTR1_LOW;
}

/* THE FIT.  In a counter build the guest clock is right and PRFC1 is counting
 * the same work, so their ratio over a second IS K.  In a deadline build the
 * same line reports what K would have to be for this build's own clock to be
 * self-consistent, which is only a sanity check, not a fit. */
void fgl_deadline_report(unsigned int guest_cycle)
{
	static unsigned int last_cycle, last_insns, sec;
	static unsigned int k_lo = ~0u, k_hi, k_sum, k_n;
	unsigned int dc, di, k;

	if (!FGL_DL_MEASURE)
		return;

	/* Drain whatever is open so the second's instructions are all counted
	 * even in a build that never calls the charge. */
	if (!FGL_DEADLINE)
		(void)fgl_deadline_settle();

	if (++sec < 60)
		return;
	sec = 0;

	dc = guest_cycle - last_cycle;
	di = fgl_deadline_insns - last_insns;
	last_cycle = guest_cycle;
	last_insns = fgl_deadline_insns;

	if (!di)
		return;

	/* K'S SPREAD, not just its value.  One number cannot tell you whether
	 * the fit is real; a K that walks with scene complexity is host work
	 * with no guest cause leaking into the meter, and the spread is what
	 * makes that announce itself instead of hiding. */
	k = (unsigned)(((uint64_t)dc << 10) / di);
	if (k < k_lo)
		k_lo = k;
	if (k > k_hi)
		k_hi = k;
	k_sum += k;
	k_n++;

	/* A fit build has to say what K is -- that is the whole point of it.
	 * A deadline build's line is only a sanity check, so it goes behind
	 * the trace knob with the rest of the chatter. */
	if (FGL_DL_FIT || FGL_DL_TRACE)
		printf("dl: %u cyc %u insn -> K %u/1024"
		       " [lo %u hi %u avg %u n %u]"
		       " | arms %u fires %u incode %u cross %u owedhi %u\n",
		       dc, di, k, k_lo, k_hi, k_sum / k_n, k_n,
		       fgl_deadline_arms, fgl_deadline_fires,
		       fgl_deadline_in_code, fgl_dl_crossings,
		       fgl_dl_owed_hi);

	fgl_deadline_arms = fgl_deadline_fires = fgl_deadline_in_code = 0;
	fgl_dl_owed_hi = 0;
	fgl_dl_crossings = 0;
}

/* THE CHARGE, IN TWO FORMS.
 *
 * A device read must never see a clock that has already run past an event that
 * has not fired.  ReARMed's device models do arithmetic against pending events
 * -- cdrom.c computes from the next event cycle minus now -- and none of it was
 * written for now to be on the far side.  A CD status poll landing in that
 * window hangs a load screen forever and looks exactly like a lost event.  So
 * the charge taken AT A CROSSING is clamped to the room left before the next
 * event, and whatever will not fit stays owed.
 *
 * The charge taken AT THE HOOK is not clamped.  That is where the drain runs,
 * and the drain has to see how overdue every event really is.  Nothing is ever
 * lost either way: the two share one pool.
 */
static uint32_t dl_owed;

static void dl_convert(void)
{
	uint64_t prod;

	dl_accum();

	/* One multiply and one shift over the whole accumulated count, so the
	 * 1/1024ths are carried rather than truncated per crossing. */
	prod = (uint64_t)dl_insns * FGL_DL_K + dl_frac;
	fgl_deadline_insns += dl_insns;
	dl_insns = 0;

	dl_owed += (uint32_t)(prod >> 10);
	dl_frac = (uint32_t)(prod & 1023u);

	/* If this only ever grows, the bound is starving the guest clock. */
	if (dl_owed > fgl_dl_owed_hi)
		fgl_dl_owed_hi = dl_owed;
}

uint32_t fgl_deadline_charge_upto(int32_t room)
{
	uint32_t take;

	dl_convert();

	if (room <= 0)
		return 0;

	take = (uint32_t)room < dl_owed ? (uint32_t)room : dl_owed;
	dl_owed -= take;
	fgl_deadline_cycles += take;

	return take;
}

uint32_t fgl_deadline_settle(void)
{
	uint32_t take;

	dl_convert();

	take = dl_owed;
	dl_owed = 0;
	fgl_deadline_cycles += take;

	return take;
}

/* THE ALARM.  One-shot: TCOR would reload and fire again. */
static void fgl_deadline_underflow(irq_t code, irq_context_t *ctx, void *data)
{
	uint32_t pc;

	(void)code;
	(void)data;

	DL_TIMER_HALT();
	timer_clear(TMU2);

	fgl_deadline_fires++;
	dl_fired = 1;

	if (!dl_cur)
		return;

	pc = CONTEXT_PC(*ctx);

	if (pc >= dl_code_lo && pc < dl_code_hi) {
		/* In generated code, where r14 IS the gate.  Zero it and the
		 * next test the block reaches sends it to the dispatcher. */
		fgl_deadline_in_code++;
		ctx->r[14] = 0;
		return;
	}

	/* In C, where r14 belongs to whatever C was running.  The gate lives
	 * in memory until the next entry, so close it there instead. */
	*dl_tgt = *dl_cur;
}

void fgl_deadline_init(volatile uint32_t *current, volatile uint32_t *target,
		       void *code_lo, uint32_t code_size)
{
	dl_cur = current;
	dl_tgt = target;
	dl_code_lo = (uint32_t)code_lo;
	dl_code_hi = dl_code_lo + code_size;

	if (!FGL_DL_MEASURE)
		return;

	perf_cntr_start(PRFC1, PMCR_INSTRUCTION_ISSUED_MODE, PMCR_COUNT_CPU_CYCLES);
	dl_mark = PMCTR1_LOW;
	dl_excluded = 0;

	irq_set_handler(EXC_TMU2_TUNI2, fgl_deadline_underflow, NULL);

	/* THE HANDLER IS NOT THE ALARM.  Setting UNIE in TCR2 makes the timer
	 * want to interrupt; the INTC still has to be told TMU2 is allowed to.
	 * Its priority defaults to masked, so without this the underflow sets
	 * TCR2's UNF flag and raises nothing at all -- the gate then has only
	 * MMIO crossings to close it, and openbios' CD wait, which is a spin
	 * loop with no MMIO in it, hangs forever. */
	timer_enable_ints(TMU2);

	DL_TIMER_HALT();
	timer_clear(TMU2);
}

/* Armed once per slice, for that slice's guest budget.  Not re-armed at every
 * C crossing: the alarm bounds the slice, not the interval between crossings. */
void fgl_deadline_arm(int32_t cycles)
{
	uint32_t ticks;

	DL_TIMER_HALT();
	timer_clear(TMU2);
	dl_fired = 0;
	fgl_deadline_arms++;

	if (cycles <= 0)
		return;

	ticks = (uint32_t)(((uint64_t)(uint32_t)cycles * DL_NUM) >> DL_SHIFT);
	if (!ticks)
		ticks = 1;

	*(volatile uint32_t *)(TIMER_BASE + TCOR2) = 0xffffffffu;
	*(volatile uint32_t *)(TIMER_BASE + TCNT2) = ticks;
	*(volatile uint16_t *)(TIMER_BASE + TCR2)  = 0x0020;	/* PCK/4, UNIE */
	*(volatile uint8_t  *)(TIMER_BASE + TSTR) |= 0x04;
}

void fgl_deadline_disarm(void)
{
	DL_TIMER_HALT();
	timer_clear(TMU2);
}

int fgl_deadline_fired(void)
{
	return dl_fired;
}
