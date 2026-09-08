#ifndef BLOOM_DEADLINE_H
#define BLOOM_DEADLINE_H

#include <stdint.h>

/* THE CD/EVENT TRACE.  The ring, the once-a-second stuck-point dump and the
 * cdrom "irq miss" chatter.  It found the lost-IRQ race; leave it off unless
 * something is stuck again.  At 0 every call site compiles away. */
#ifndef FGL_DL_TRACE
#define FGL_DL_TRACE 0
#endif

#ifndef FGL_DEADLINE
#define FGL_DEADLINE 0
#endif

/* THE SANITY BUILD.  There is nothing left to fit -- see FGL_DL_STRETCH16 --
 * but the sampling and the report are still worth having in a counter build,
 * where they say how much real time a correct guest second actually took.
 * That ratio is the stretch, measured rather than argued about. */
#ifndef FGL_DL_FIT
#define FGL_DL_FIT 0
#endif

#define FGL_DL_MEASURE (FGL_DEADLINE || FGL_DL_FIT)

/* THE STRETCH, WHICH IS NOT K.
 *
 * The guest's clock is host time, so a host slower than a PS1 hands the guest
 * fewer instructions per guest second than the machine it is pretending to be.
 * Far enough below 1:1 and the guest's own timeouts start firing before the
 * work behind them is done -- a pad transfer that lands late reads as a pad
 * that is not there, and a display list gets submitted half-built.
 *
 * The stretch is the answer: lengthen a guest cycle by this factor and every
 * guest-facing interval lengthens with it, because they are all already
 * expressed in guest cycles -- the root counters, the CD's sector period, the
 * display reload.  The guest still sees its own 60 Hz and its own 150 sectors
 * a second, and gets this many times more work done between them.
 *
 * AND IT IS THE ONLY DIAL LEFT.  Frame rate is linear in it, because the game
 * renders one frame per guest vblank and the guest clock is real time over the
 * stretch: 8.75M guest cycles a real second at 4, 19.8M at 1.7, and the frame
 * rates measured at those two (6.5 and 14.3) are in exactly that ratio.  The
 * emulator's throughput does not change; only how much guest time it buys.
 *
 * So the value wants to be the SMALLEST one that still renders, not a generous
 * one.  Too large is not safety, it is frame rate given away.
 */
/* IN SIXTEENTHS, so the useful range is reachable.  The break-even is not an
 * integer: this host delivers about 25.8M guest instructions a real second
 * against a PS1's 33.87M -- 0.76x -- so a stretch of 1.32 is exactly 1:1 and
 * anything below it starves the guest.  K=296 was effectively 1.7 and rendered
 * correctly; 4 rendered correctly too and cost 2.2x the frame rate for margin
 * it did not need.  32/16 = 2.0 is 1.5x break-even. */
#ifndef FGL_DL_STRETCH16
#define FGL_DL_STRETCH16 32
#endif

/* CHARGE EVERYTHING, WHICH IS WHAT THE MACHINE DOES.
 *
 * A real PS1's CPU clock does not stop while the GPU draws.  The R3000 keeps
 * counting through the whole frame, the root counters keep counting with it,
 * and the game's own timing is written against that.  bleem charged the same
 * way: one clock, everything on it, rendering included.
 *
 * bloom did not.  `pause`/`resume` bracketed the renderer, the CD and the
 * vblank wait out of the guest clock on the argument that they are host time
 * with no guest cause.  The argument is wrong on its own terms -- the guest
 * DID cause the drawing, it submitted the display list -- and it costs three
 * ways:
 *
 *   1. The stretch stops meaning what it says.  Measured at stretch 27/16 the
 *      guest was being handed 35/16, because only 758/1000 of real time was
 *      ever charged.  Every number read off the set value was a third out.
 *   2. The excluded fraction is not even constant -- 739 to 955 across one
 *      run -- so the delivered rate moves with the scene.
 *   3. Every bracket is a TMU1 read, and TCNT1 is a peripheral register on the
 *      external bus.  Those reads are what cost the deadline build 9 ms a
 *      frame the first time round.
 *
 * With this set, `charged` reads ~1000/1000 and `deliv` equals the stretch.
 *
 * THE CONVERSION, because the dial's meaning changes: a stretch of S here does
 * what S * 758/1000 did before.  To reproduce a run made at 22 under the old
 * exclusion, set about 29. */
#ifndef FGL_DL_CHARGE_ALL
#define FGL_DL_CHARGE_ALL 1
#endif
/* THE COALESCE BOUND.  How many base-counter periods psxRcntUpdate will render
 * before it gives up and discards the rest.  Each pass renders a frame, so a
 * lumpy charge -- a savestate load, a long compile -- could otherwise render a
 * hundred frames in one go.
 *
 * Past the bound the overshoot is DROPPED, not carried: the guest asked for
 * repeat vblanks it can no longer use, and handing them over one at a time is
 * the burst of starvation the owed ceiling used to cause from the other end.
 * This is what bleem does -- store the reload, discard the overshoot -- and it
 * is why dropping guest TIME is no longer needed anywhere.
 *
 * (What stood here was FGL_DL_OVERSHOOT and FGL_DL_OWED_MAX.  Both existed only
 * to make a held-back clock survivable.  The clock is no longer held back; see
 * events_run_due in psxevents.c.) */
#ifndef FGL_RCNT_CATCHUP_MAX
#define FGL_RCNT_CATCHUP_MAX 8
#endif

/* THE PAD, THE WAY BLEEM DOES IT.
 *
 * ReARMed schedules a PSXINT_SIO event 535 guest cycles after every pad byte
 * and raises IRQ7 when it fires.  Under a clock that is real time that event
 * lands whenever the slice happens to end, and a nine-byte handshake that slips
 * one byte hands the game an analog axis where the button word belongs -- bit 3
 * of which is Start.  That is Spyro opening its own pause menu while you walk.
 *
 * Bleem has no such event.  The response is prepared at the JOY_DATA write and
 * IRQ7 is raised at the JOY_STAT read, once RX is ready.  Raising it at the
 * WRITE does not work: the guest acks the previous IRQ through JOY_CTRL after
 * writing, which wipes it, and the pad times out.  Raising it at the status
 * read cannot be acked away, because the ack always precedes the look. */
#ifndef FGL_SIO_BLEEM
#define FGL_SIO_BLEEM 1
#endif

extern unsigned int fgl_rcnt_catchup, fgl_rcnt_coalesced;
/* Why slices ended, counted in lightrec_tansition_from_pcsx: block_stepping,
 * no room left before next_interupt, a pending guest interrupt, or the alarm. */
extern unsigned int fgl_dl_end_step, fgl_dl_end_room, fgl_dl_end_irq,
		    fgl_dl_end_alarm;

void fgl_deadline_init(volatile uint32_t *current, volatile uint32_t *target,
		       void *code_lo, uint32_t code_size);

/* The two crossings.  `pause` closes the in-code interval on the way out to C,
 * `resume` opens a new one on the way back in.  Everything between them is
 * host work and is never charged to the guest. */
void fgl_deadline_pause(void);
void fgl_deadline_resume(void);

/* `pause` without the counter read, for the one place the mark is known to be
 * current already: an MMIO crossing, straight after the charge. */
void fgl_deadline_exclude_begin(void);

/* Guest cycles for the real time since the last call.  Consumes it, never
 * clamps, never carries: see the pool that used to be here, in deadline.c. */
uint32_t fgl_deadline_settle(void);

void fgl_deadline_arm(int32_t cycles);
void fgl_deadline_disarm(void);
int  fgl_deadline_fired(void);

extern unsigned int fgl_deadline_arms, fgl_deadline_fires, fgl_deadline_in_code;
extern unsigned int fgl_deadline_cycles, fgl_deadline_ticks;
extern unsigned int fgl_dl_crossings, fgl_dl_redundant;

/* Prints the fit and the alarm counts once a second.  Call it wherever a frame
 * ends; it does nothing unless FGL_DL_MEASURE. */
void fgl_deadline_report(unsigned int guest_cycle);

#endif
