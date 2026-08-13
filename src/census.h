/* THE DENOMINATORS.
 *
 * A bucket in percent cannot be compared against another emulator: two runs
 * drawing different scenes disagree at the same percentage, and bloom and
 * bloop draw very different scenes for the same game (bloop's display list is
 * untextured colour polys through one context; src/pvr.c does texture pages,
 * CLUTs, VQ, PT/TR splitting and a second pass for PS1 semi-transparency).
 *
 * So every bucket in prof.h gets a count beside it, and the counts are the
 * ones bloop already prints, spelled the same way, so the two reports can be
 * set side by side and divided:
 *
 *   RENDER ms / binds     against bloop's ~6 us per bind
 *   GTE ms / GTE commands against bloop's ~1.13 us per RTPS
 *
 * These are plain increments on extern arrays — no function call, no timer
 * read — so this is far cheaper than prof.h and can be left on in runs where
 * the bucket profiler cannot be.  It is still not free and still off by
 * default: configure with -DCENSUS=ON.
 *
 * CROSSINGS ARE COUNTS, NOT TIME.  The C-crossing counters here (cop2, rw,
 * hw, and entries to lightrec_execute) exist to be divided into prof.h's ms:
 * a count plus a cost says whether to attack how often we cross or what a
 * crossing costs, and neither number answers that alone.
 */

#ifndef BLOOM_CENSUS_H
#define BLOOM_CENSUS_H

#include <stdint.h>

typedef enum {
	CENSUS_FRAMES,		/* guest display flips */
	CENSUS_VERTICES,	/* vertices submitted to the PVR */
	CENSUS_RECORDS,		/* primitives submitted */
	CENSUS_BINDS,		/* PVR context switches — bloop's "binds" */
	CENSUS_TEX_LOOKUPS,
	CENSUS_TEX_MISSES,
	CENSUS_TEX_EVICTIONS,
	CENSUS_GTE_CMDS,	/* calls into cop2_op */
	CENSUS_RW_GENERIC,	/* lightrec_rw_generic_cb: unclassified access */
	CENSUS_HW_READ,		/* plugin hw read wrappers */
	CENSUS_HW_WRITE,	/* plugin hw write wrappers */
	CENSUS_EXEC_ENTRIES,	/* entries to lightrec_execute — dispatcher round trips */
	CENSUS_COMPILES,	/* blocks compiled inline on the emu thread */
	CENSUS_INVALIDATES,	/* calls to the DMA clear path */
	CENSUS_CD_SECTORS,

	/* Guest cycles the GPUSTATUS poll-skip added without retiring an
	 * instruction (plugin.c hw_read_word, GPUSTATUS_POLLING_THRESHOLD).
	 * bloom's k/s is psxRegs.cycle / 1.75 — it counts cycles and calls
	 * them instructions — so every cycle here is k/s the run did not
	 * earn.  Subtract before quoting a throughput number against an
	 * emulator that counts retired instructions. */
	CENSUS_SKIPPED_CYCLES,

	CENSUS_N
} census_id;

const char *census_name(census_id id);

#ifdef BLOOM_CENSUS

extern uint32_t census_ctr[CENSUS_N];
extern uint32_t census_gte_func[64];	/* GTE commands by function */
extern uint32_t census_gp0_op[256];	/* GP0 commands by opcode */
extern uint32_t census_irq_fired[32];	/* irq_test dispatches by PSXINT_* */
extern uint32_t census_hw_reg[1024];	/* hw wrapper accesses by register */

static inline void census_bump(census_id id)
{
	census_ctr[id]++;
}

static inline void census_add(census_id id, uint32_t n)
{
	census_ctr[id] += n;
}

static inline void census_gte(uint32_t func)
{
	census_gte_func[func & 0x3f]++;
	census_ctr[CENSUS_GTE_CMDS]++;
}

static inline void census_gp0(uint32_t op)
{
	census_gp0_op[op & 0xff]++;
}

static inline void census_irq(unsigned int irq)
{
	census_irq_fired[irq & 0x1f]++;
}

/* One 4 KiB page of I/O registers, word-granular — enough to name which
 * register the hw wrappers are being hammered with. */
static inline void census_hw_access(uint32_t mem)
{
	census_hw_reg[(mem >> 2) & 0x3ff]++;
}

void census_reset(void);
void census_report(uint32_t wall_ms, uint32_t frames);

#else

#define census_bump(id)			((void)0)
#define census_add(id, n)		((void)0)
#define census_gte(func)		((void)0)
#define census_gp0(op)			((void)0)
#define census_irq(irq)			((void)0)
#define census_hw_access(mem)		((void)0)
#define census_reset()			((void)0)
#define census_report(ms, f)		((void)0)

#endif /* BLOOM_CENSUS */

#endif /* BLOOM_CENSUS_H */
