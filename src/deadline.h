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

/* FITTING K.  The deadline build cannot fit its own constant -- its guest clock
 * is the thing under test.  The counter build can: it charges the guest
 * correctly, per guest opcode, while PRFC1 measures exactly the same SH-4 work
 * the deadline build would measure.  So K is simply
 *
 *      cycles the counter build charged / instructions PRFC1 counted
 *
 * over the same interval, and one run of a counter build with FGL_DL_FIT=1
 * prints it.  Sampling is compiled in; only the charge is left to the
 * deadline. */
#ifndef FGL_DL_FIT
#define FGL_DL_FIT 0
#endif

#define FGL_DL_MEASURE (FGL_DEADLINE || FGL_DL_FIT)

/* K: guest cycles per SH-4 instruction executed in generated code, in 1/1024
 * fixed point.  Fitted once against the counter build's guest instructions per
 * frame; it is the one number in the design and it is not derived. */
#ifndef FGL_DL_K
#define FGL_DL_K 1024
#endif

/* THE OVERSHOOT BOUND.  The gate only closes at a block boundary or an MMIO
 * crossing, so the settle at the hook can land arbitrarily far past
 * next_interupt -- and it does whenever anything on the host runs long enough
 * to be worth thousands of guest cycles: a console print, a compile on the
 * worker thread, a cache miss storm.  A lump like that lands between two
 * adjacent guest instructions, and a device whose model assumes those two
 * instructions are microseconds apart races against itself.  The CD losing
 * I_STAT bit 2 between the guest's CD ack and its I_STAT ack is exactly that.
 *
 * So the hook charges to next_interupt plus this and carries the rest.  Events
 * then run at most this far late, and the backlog drains over the following
 * hooks instead of in one step. */
#ifndef FGL_DL_OVERSHOOT
#define FGL_DL_OVERSHOOT 128
#endif

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

/* Guest cycles owed for the in-code work since the last call.  Consumes it. */
uint32_t fgl_deadline_charge_upto(int32_t room);
uint32_t fgl_deadline_settle(void);

void fgl_deadline_arm(int32_t cycles);
void fgl_deadline_disarm(void);
int  fgl_deadline_fired(void);

extern unsigned int fgl_deadline_arms, fgl_deadline_fires, fgl_deadline_in_code;
extern unsigned int fgl_deadline_cycles, fgl_deadline_insns;
extern unsigned int fgl_dl_crossings, fgl_dl_redundant;
extern unsigned int fgl_dl_owed_hi;

/* Prints the fit and the alarm counts once a second.  Call it wherever a frame
 * ends; it does nothing unless FGL_DL_MEASURE. */
void fgl_deadline_report(unsigned int guest_cycle);

#endif
