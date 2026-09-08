/* THE DEADLINE SCHEDULER.
 *
 * Generated code contains no cycle arithmetic at all.  r14 is a gate, not a
 * countdown: non-zero means keep running, zero means go to the dispatcher.
 * Nothing inside emitted code ever moves it.  Two things close it -- the
 * dispatcher's own test when a slice is asked to end, and TMU2 firing.
 *
 * That leaves the guest's clock with nothing to advance it, so the charge is
 * made at the crossings instead, and what it charges is TIME: ticks of TMU1
 * since the last crossing, scaled to guest cycles.
 *
 * ONE CLOCK, WHICH IS THE ENTIRE POINT OF THIS FILE.
 *
 * It used to be two.  The charge came from PRFC1, counting SH-4 issue slots,
 * scaled by a fitted constant K; the alarm was a timer.  Every failure this
 * design has had came from those two rulers disagreeing:
 *
 *   - K is not a property of the machine.  Host work per guest instruction
 *     differs between a tight BIOS poll and ordinary game code, so no single
 *     constant served both, and the one that was fitted moved with the emitter
 *     and the scene.
 *   - The alarm converted the slice budget at REAL PS1 SPEED while the charge
 *     ran at whatever the host managed.  It therefore expired after a fraction
 *     of the slice, the hook found nothing due and re-armed, and a slice took
 *     about twenty dispatcher round trips to finish instead of one.
 *
 * bloop and bleem never had either problem, because neither has a second
 * ruler: guest time is host time and the countdowns are held in host ticks.
 * That is what this is now.  TMU1 both measures the interval and, through the
 * same fixed ratio, sizes the TMU2 arm -- so the alarm cannot fire early, and
 * there is nothing left to fit.
 *
 * WHAT IS GIVEN UP, honestly: determinism.  A fixed run no longer costs a fixed
 * number of guest cycles, so two runs of the same binary do not line up and
 * there is no RAM-hash comparison against a counter build.  And a slow scene
 * shows the guest real time passing -- the game sees several vblanks per
 * rendered frame.  Both are what bloop lives with.
 *
 * FGL_DL_STRETCH is the handle for the second one.  See deadline.h: it is not
 * K wearing a different hat, because it only has to be generous rather than
 * correct.
 *
 * The charge is never clamped and nothing is ever owed.  A clamp keeps the
 * clock behind an event that has not fired; the answer is to fire the event
 * instead -- events_run_due() in psxevents.c, called at the crossing.  That is
 * what deleted the owed pool, OWED_MAX and the dropped time along with it.
 */

#include <stdio.h>

#include <arch/irq.h>
#include <arch/timer.h>

#include "deadline.h"
#include "bench.h"

unsigned int fgl_deadline_arms, fgl_deadline_fires, fgl_deadline_in_code;
unsigned int fgl_deadline_cycles, fgl_deadline_ticks;
unsigned int fgl_dl_crossings, fgl_dl_redundant;

/* THE CLOCK.  TMU1, KOS's uptime counter.
 *
 * KOS's own contract for it (arch/timer.h): "used as a free-running counter and
 * is only ever read by KOS, so an application may share it, but must not reload
 * or stop it."  The single write pair is in timer_uptime_enable, guarded by
 * `if(!timer_running(TMU1))`, once at boot; timer_shutdown deliberately skips
 * it.  So reading it is free and nothing moves it under us.
 *
 * IT COUNTS DOWN, and it is never reloaded, so elapsed is (previous - current)
 * and unsigned arithmetic carries that correctly across the wrap at zero.  A
 * 32-bit read of a peripheral register, no call, no helper.
 *
 * TMU2 below is the alarm and runs off the same prescaler, PCK/4, so a tick
 * here and a tick there are the same tick.  That identity is what makes the
 * alarm exact rather than estimated. */
#define TIMER_BASE	0xffd80000
#define TCNT1		0x18
#define DL_NOW		(*(volatile uint32_t *)(TIMER_BASE + TCNT1))

static uint32_t dl_mark;		/* the counter at the last sample */
static int dl_excluded;			/* nesting depth of pause/resume */
static uint32_t dl_elapsed;		/* ticks not yet converted */
static uint32_t dl_frac;		/* 1/4096ths carried between charges */

/* TMU2, THE ALARM, on the same tick as the clock above.
 *
 * 12468720 host ticks a second against 33868800 guest cycles: 3016/8192
 * exactly.  DL_TICK converts guest cycles to ticks for the arm, DL_CYCLE
 * converts ticks to guest cycles for the charge, and they are the SAME ratio
 * read in opposite directions.  That is why the alarm lands where the charge
 * says it will -- there is no second opinion to be wrong.
 *
 * FGL_DL_STRETCH multiplies the arm and divides the charge, which is what makes
 * a guest cycle longer in real terms without any interval in the emulator
 * needing to know. */
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

#define DL_FIX		12

/* ticks = cycles * 3016 * STRETCH / 8192, in 1/4096ths */
#define DL_TICK		(uint32_t)(((uint64_t)3016u * FGL_DL_STRETCH16 << DL_FIX) \
			           / (16u * 8192u))

/* cycles = ticks * 8192 / (3016 * STRETCH), in 1/4096ths.  Truncation is a few
 * thousandths of a percent and the remainder is carried in dl_frac anyway. */
#define DL_CYCLE	(uint32_t)(((uint64_t)(8192u * 16u) << DL_FIX) \
			           / (3016u * FGL_DL_STRETCH16))

static volatile uint32_t *dl_cur, *dl_tgt;
static uint32_t dl_code_lo, dl_code_hi;
static volatile int dl_fired;


/* ONE SAMPLE PER CROSSING, not two.
 *
 * The charge is taken where the guest leaves generated code, and everything
 * since the previous crossing is charged: the guest's own translated code and
 * the C the crossing itself runs.  That C is not free, but it is time the guest
 * caused -- an MMIO wrapper runs because the guest touched hardware, a compile
 * runs because the guest reached new code -- and under a wall clock it is time
 * the guest genuinely spent.
 *
 * Pause and resume survive for the host time that has NO guest cause: waiting
 * on the scanout, blocking on audio.  Charging that would let a slow frame
 * inflate the guest's clock for something the guest did not ask for, and under
 * a wall clock that is the one leak that still matters. */
static void dl_accum(void)
{
	uint32_t now = DL_NOW;

	/* Counting down, so this order is not a typo. */
	if (!dl_excluded)
		dl_elapsed += dl_mark - now;

	dl_mark = now;
}

/* A DEPTH, NOT A FLAG.  The hook is bracketed, and things that run inside it
 * -- a console print, the compiler -- are bracketed too, so these nest.  With
 * a flag the inner resume reopens the interval and charges the rest of the
 * hook to the guest. */
void fgl_deadline_pause(void)
{
	fgl_dl_crossings++;

	/* CHARGE EVERYTHING: the clock never stops, so there is nothing to
	 * close, nothing to reopen, AND NOTHING TO SAMPLE.
	 *
	 * dl_accum used to run here.  It was pure waste.  With CHARGE_ALL,
	 * dl_excluded is never incremented -- this function returns before the
	 * increment and so does exclude_begin -- so dl_accum is always
	 *
	 *	dl_elapsed += dl_mark - now;  dl_mark = now;
	 *
	 * which telescopes: between two settles, calling it a hundred times
	 * and calling it zero times leave dl_elapsed with the identical value,
	 * because fgl_deadline_settle() calls dl_accum() itself before reading
	 * it.  Every crossing was therefore paying a load from TCNT1 -- a
	 * peripheral register on the external bus, one of the slowest loads the
	 * SH-4 issues -- for a result that arithmetic discards.  At `cross`
	 * ~30,000 per report interval against an alarm that fires ~230 times,
	 * that is two orders of magnitude more clock sampling than bleem does:
	 * it reads TCNT0 exactly once per collection and never at a block or
	 * call boundary (dynarec_services.md 4.5).
	 *
	 * The old behaviour is not worth a toggle -- it is not a different
	 * policy, it is the same policy computed the expensive way. */
	if (FGL_DL_CHARGE_ALL)
		return;

	dl_accum();
	dl_excluded++;
}

/* THE CHEAP HALF OF A PAUSE.  In a non-CHARGE_ALL build `pause` reads TMU1 to
 * close the interval before it stops counting.  At an MMIO crossing that read has already happened --
 * `lightrec_tansition_to_pcsx` charges (deadline) or settles (fit) on the way
 * in, and both go through dl_accum, so dl_mark is current -- and a second read
 * of the same value costs a peripheral access to learn nothing.
 *
 * Only correct immediately after a charge or a settle.  Anywhere else, pause. */
void fgl_deadline_exclude_begin(void)
{
	fgl_dl_crossings++;

	if (FGL_DL_CHARGE_ALL)
		return;

	dl_excluded++;
}

void fgl_deadline_resume(void)
{
	/* Not "redundant" -- there was deliberately nothing to resume. */
	if (FGL_DL_CHARGE_ALL)
		return;

	if (!dl_excluded) {
		fgl_dl_redundant++;
		return;
	}

	if (--dl_excluded == 0)
		dl_mark = DL_NOW;
}

/* THE REPORT.  There is no K to fit any more, so this prints the raw pair --
 * guest cycles and host ticks over the same window -- and, since the alarm
 * turns out to end only a twentieth of slices, what ends the other nineteen.
 * Deriving a ratio from the pair would be circular; see the printf. */
void fgl_deadline_report(unsigned int guest_cycle)
{
	static unsigned int last_cycle, last_ticks, last_wall, sec;
	unsigned int dc, dt, dw, deliv, chg;
	uint32_t wall = DL_NOW;

	if (!FGL_DL_MEASURE)
		return;

	/* Drain whatever is open so the second's time is all counted even in a
	 * build that never calls the charge. */
	if (!FGL_DEADLINE)
		(void)fgl_deadline_settle();


	if (++sec < 60)
		return;
	sec = 0;

	dc = guest_cycle - last_cycle;
	dt = fgl_deadline_ticks - last_ticks;
	dw = last_wall - wall;		/* counts down, so this order */
	last_cycle = guest_cycle;
	last_ticks = fgl_deadline_ticks;
	last_wall = wall;

	if (!dc || !dw)
		return;

	/* THE DELIVERED STRETCH, WHICH IS NOT THE SET STRETCH.
	 *
	 * dt is CHARGED ticks -- what dl_accum let through -- and the cycle
	 * count is derived from it, so dt/dc is circular and can only return
	 * FGL_DL_STRETCH16.  dw is every tick TMU1 counted in the window,
	 * charged or excluded, read straight off the counter and touched by
	 * nothing else.  That one is not circular.
	 *
	 *   deliv = real seconds per guest second, in 1/16ths
	 *         = 16 * (dw / 12468720) / (dc / 33868800)
	 *         = 16 * dw * 8192 / (dc * 3016)
	 *
	 * If deliv comes back equal to the set stretch, the guest is being
	 * charged for all of its real time and a stretch sweep means what it
	 * says.  If deliv is larger, the excluded regions are eating the
	 * difference: the guest is getting more real time per guest second
	 * than the setting asks for, the setting is not the thing throttling
	 * the run, and sweeping it proves nothing until that is understood.
	 *
	 * chg is the same fact from the other end -- charged ticks as a
	 * per-mille of real ticks.  1000 means nothing is excluded. */
	deliv = (unsigned int)(((uint64_t)dw * 8192u * 16u) / ((uint64_t)dc * 3016u));
	chg   = (unsigned int)(((uint64_t)dt * 1000u) / dw);

	/* NO `slow` RATIO HERE.  There was one and it was circular: the stretch
	 * is applied inside the conversion, so ticks-per-guest-cycle can only
	 * ever come back as exactly the stretch.  It measured the constant it
	 * was derived from.  The honest numbers are the raw pair above and the
	 * slice enders below. */
	printf("dl: %u cyc %u tick %u wall | arms %u fires %u incode %u cross %u"
	       " stretch %u/16 deliv %u/16 charged %u/1000 catchup %u coal %u"
	       " | end step %u room %u irq %u alarm %u\n",
	       dc, dt, dw,
	       fgl_deadline_arms, fgl_deadline_fires,
	       fgl_deadline_in_code, fgl_dl_crossings,
	       (unsigned)FGL_DL_STRETCH16, deliv, chg,
	       fgl_rcnt_catchup, fgl_rcnt_coalesced,
	       fgl_dl_end_step, fgl_dl_end_room, fgl_dl_end_irq,
	       fgl_dl_end_alarm);

	fgl_dl_end_step = fgl_dl_end_room = 0;
	fgl_dl_end_irq = fgl_dl_end_alarm = 0;
	fgl_deadline_arms = fgl_deadline_fires = fgl_deadline_in_code = 0;
	fgl_dl_crossings = 0;
	fgl_rcnt_catchup = fgl_rcnt_coalesced = 0;
}

/* THE CHARGE.  ONE FORM, AND NOTHING IS OWED.
 *
 * There used to be two.  A device model must never see a clock that has already
 * run past an event that has not fired -- cdrom.c does arithmetic against
 * pending event cycles and none of it was written for `now` to be on the far
 * side -- so the charge taken at a crossing was CLAMPED to the room before the
 * next event and the remainder stayed in a pool.
 *
 * The pool was the mistake.  Real time accrues at whatever rate it likes while
 * the guest's own schedule decides how fast it can be paid out, and root
 * counters firing every couple of thousand cycles leave very small gaps.  So it
 * grew without bound, and the only way to shrink it was to throw guest time
 * away -- OWED_MAX, `dropped`, and a guest that ran in bursts of starvation.
 *
 * The right answer is not to hold the clock back but to fire what is behind it:
 * events_run_due() at the crossing, before any handler looks at the clock.  See
 * psxevents.c.  With that in place the charge is unconditional -- put the clock
 * where the counter says it is -- and OWED_MAX, OVERSHOOT and charge_upto are
 * all gone with the pool.
 */
uint32_t fgl_deadline_settle(void)
{
	uint64_t prod;
	uint32_t take;

	dl_accum();

	/* One multiply and one shift over the whole accumulated count, so the
	 * 1/4096ths are carried rather than truncated per crossing. */
	prod = (uint64_t)dl_elapsed * DL_CYCLE + dl_frac;
	fgl_deadline_ticks += dl_elapsed;
	dl_elapsed = 0;

	take = (uint32_t)(prod >> DL_FIX);
	dl_frac = (uint32_t)(prod & ((1u << DL_FIX) - 1u));

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

	/* NO NOT-DUE-YET CHECK ANY MORE.  There used to be one: the alarm was
	 * armed in real-PS1 time against a charge that ran at host speed, so it
	 * expired early and the handler had to ask pcsx whether the fire meant
	 * anything.  One clock removes the question -- the same counter at the
	 * same rate arms the alarm and pays the charge, so an alarm that has
	 * underflowed IS due, by construction. */
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

	/* Nothing to start: TMU1 has been free-running since KOS booted and is
	 * ours to read.  Just take the first mark. */
	dl_mark = DL_NOW;
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
 * C crossing: the alarm bounds the slice, not the interval between crossings.
 *
 * Nothing is measured here now.  DL_TICK is the exact inverse of the ratio the
 * charge uses, on the same counter, so this is arithmetic and not an estimate. */
void fgl_deadline_arm(int32_t cycles)
{
	uint32_t ticks;

	DL_TIMER_HALT();
	timer_clear(TMU2);
	dl_fired = 0;
	fgl_deadline_arms++;

	if (cycles <= 0)
		return;

	ticks = (uint32_t)(((uint64_t)(uint32_t)cycles * DL_TICK) >> DL_FIX);
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
