/* WHAT EMITTED CODE COSTS, AND NOTHING ELSE'S.
 *
 * A counter pair run across a window but PAUSED wherever the machine is not
 * executing emitted code.  The window runs three times with a different pair
 * loaded each lap -- misses, then the cycles those misses froze, then elapsed
 * cycles and instructions issued as the denominator -- so one run prices the
 * recompiler's own memory behaviour and its own CPI.  See emitmiss.c.
 *
 * The pause points are the doors out of emitted code: `fgl_service_events`
 * and the four shims (hw, rw, cop, svc).  What is left is the JIT's own
 * traffic, which is the number a code-layout change has to move.
 *
 * PAUSE, NOT SAMPLE.  `perf_cntr_stop` retains the count and `perf_cntr_resume`
 * carries on, so a door costs two register writes instead of four 48-bit reads
 * and an accumulate.  That matters: the hw door alone runs ~1270 times a frame.
 *
 * NOT `perf_cntr_clear` ANYWHERE BUT THE ARM.  KOS's clear asserts PMCR_CLR and
 * never drops it, and resume only ORs in PMCR_RUN -- so a clear followed by a
 * resume leaves the counter running with CLR still asserted and reading near
 * zero.  Clear once, then `perf_cntr_start` (which writes the whole config),
 * and use stop/resume from then on. */

#ifndef BLOOM_EMITMISS_H
#define BLOOM_EMITMISS_H

#include <stdint.h>

/* Dispatcher entries, bumped by four instructions at `.Lrun` in dispatch.S.
 * DISPATCHER ENTRIES ONLY -- a patched link is a `bra` straight to a block's
 * entry point and never passes this label, so this is not the number of blocks
 * executed.  With linking working it counts the exits that could not be
 * patched: returns, computed jumps, and edges whose target was not compiled
 * when the site asked. */
extern uint32_t fgl_blocks_run;

#if defined(BLOOM_EMITMISS) && defined(__sh__)

#ifndef EMITMISS_VSYNCS
#define EMITMISS_VSYNCS 120
#endif

/* Presented frames to let go by before the window opens.  The first few
 * hundred are level load: the compiler thread is still working, fps is 9.6
 * against a steady 13.7, and its misses land on whichever thread the counters
 * happen to be running for.  Measuring a warm JIT means starting after it. */
#ifndef EMITMISS_SKIP
#define EMITMISS_SKIP 120
#endif

void bloom_emitmiss_vsync(void);
void bloom_emitmiss_resume(void);       /* entering emitted code */
void bloom_emitmiss_pause(void);        /* leaving it: shim or service */

#define EMITMISS_VSYNC()        bloom_emitmiss_vsync()
#define EMITMISS_ENTER()        bloom_emitmiss_resume()
#define EMITMISS_LEAVE()        bloom_emitmiss_pause()
#define EMITMISS_OUT_BEGIN()    bloom_emitmiss_pause()
#define EMITMISS_OUT_END()      bloom_emitmiss_resume()

#else

#define EMITMISS_VSYNC()        do { } while (0)
#define EMITMISS_ENTER()        do { } while (0)
#define EMITMISS_LEAVE()        do { } while (0)
#define EMITMISS_OUT_BEGIN()    do { } while (0)
#define EMITMISS_OUT_END()      do { } while (0)

#endif

#endif /* BLOOM_EMITMISS_H */
