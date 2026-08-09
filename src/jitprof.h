/* PC-sampling profiler for the emitted code buffer.
 *
 * Answers one question: inside the ~85% of wall time the bench reports as
 * "in guest code", is the time spread evenly across emitted instructions, or
 * concentrated somewhere nameable?  Even spread means codegen quality and
 * only a backend rewrite helps.  Concentration means there is still something
 * to fix in place.
 *
 * Samples the interrupted PC from a TMU2 underflow interrupt.  TMU2 is free
 * (KOS 4fdde8b2 moved the gettime functions to TMU1), and the handler only
 * reads ctx->pc and bumps a counter, so it does not allocate, print, or
 * touch anything the emulator owns.
 */

#ifndef BLOOM_JITPROF_H
#define BLOOM_JITPROF_H

#ifdef JITPROF

void jitprof_start(void);
void jitprof_report(void);

#else

static inline void jitprof_start(void) { }
static inline void jitprof_report(void) { }

#endif

#endif /* BLOOM_JITPROF_H */
