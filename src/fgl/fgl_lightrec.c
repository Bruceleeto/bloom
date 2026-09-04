/* Where fgl and lightrec have to agree, and the proof that they do.
 *
 * fgl's emitter bakes state-block displacements into instruction words at
 * compile time.  It gets them from `fgl_state.h`, which is a list of integer
 * literals -- it has to be, because the host test rig compiles fgl without
 * lightrec's headers in scope, and that is the whole reason the oracle can
 * run on a workstation.  So the two descriptions of the same memory live in
 * two files and nothing but this translation unit makes them one.
 *
 * THAT IS THE FAILURE THIS FILE EXISTS TO PREVENT, AND IT IS A QUIET ONE.
 * A field inserted into `struct lightrec_state` above `current_cycle` does
 * not break a build and does not fault.  Generated code goes on issuing
 * `mov.l @(167*4,GBR),r0`, that displacement now names `target_cycle`
 * instead, and the emulator runs -- wrongly, in a way that surfaces as a game
 * hanging some minutes later on hardware, with nothing pointing here.
 *
 * So every displacement fgl can emit is checked against `offsetof` below.  A
 * mismatch is a compile error naming the field.
 */

#include <stddef.h>

#include "lightrec-private.h"

#include "fgl_state.h"

#define FGL_ASSERT(cond, name) \
	typedef char fgl_layout_##name[(cond) ? 1 : -1]

/* The register file.  A guest register number IS its displacement: this is
 * what lets the writeback flush emit a store from the register number with no
 * translation table, and what makes LO and HI ordinary registers rather than
 * a special case (lightrec.h:120 gives gpr[] 34 entries for exactly that). */
FGL_ASSERT(offsetof(struct lightrec_state, regs.gpr) == 0, gpr_base);
FGL_ASSERT(GUEST_AT(0) * 4 == offsetof(struct lightrec_state, regs.gpr[0]),
	   gpr_scale);
FGL_ASSERT(GUEST_LO * 4 == offsetof(struct lightrec_state, regs.gpr[32]),
	   guest_lo);
FGL_ASSERT(GUEST_HI * 4 == offsetof(struct lightrec_state, regs.gpr[33]),
	   guest_hi);
FGL_ASSERT(GUEST_LO == REG_LO && GUEST_HI == REG_HI, lo_hi_agree);

/* The coprocessor files. */
FGL_ASSERT(FGL_AT_COP0 * 4 == offsetof(struct lightrec_state, regs.cp0),
	   cop0_base);
FGL_ASSERT(FGL_AT_CP2D * 4 == offsetof(struct lightrec_state, regs.cp2d),
	   cp2d_base);
FGL_ASSERT(FGL_AT_CP2C * 4 == offsetof(struct lightrec_state, regs.cp2c),
	   cp2c_base);

/* The scratch word and the two PCs. */
FGL_ASSERT(FGL_AT_TEMP_REG * 4 == offsetof(struct lightrec_state, temp_reg),
	   temp_reg);
FGL_ASSERT(FGL_AT_TEMP_REG == REG_TEMP, temp_reg_agrees);
FGL_ASSERT(FGL_AT_CURR_PC * 4 == offsetof(struct lightrec_state, curr_pc),
	   curr_pc);
FGL_ASSERT(FGL_AT_NEXT_PC * 4 == offsetof(struct lightrec_state, next_pc),
	   next_pc);

/* The cycle table, whose displacement is an instruction count. */
FGL_ASSERT(FGL_AT_CYCLES * 4 == offsetof(struct lightrec_state, cycle_table),
	   cycle_table);
FGL_ASSERT(FGL_CYCLE_ENTRIES == LIGHTREC_CYCLE_ENTRIES, cycle_entries);

/* The counters and the flag, which only the exit paths touch. */
FGL_ASSERT(FGL_AT_CURRENT_CYCLE * 4
	   == offsetof(struct lightrec_state, current_cycle), current_cycle);
FGL_ASSERT(FGL_AT_TARGET_CYCLE * 4
	   == offsetof(struct lightrec_state, target_cycle), target_cycle);
FGL_ASSERT(FGL_AT_EXIT_FLAGS * 4
	   == offsetof(struct lightrec_state, exit_flags), exit_flags);

/* Where a finished block jumps.  Read by every block that ever runs, so a
 * wrong displacement here is not a subtle desynchronisation -- it is a jump to
 * whatever `exit_flags` happened to contain. */
FGL_ASSERT(FGL_AT_DISPATCH * 4
	   == offsetof(struct lightrec_state, dispatch), dispatch);
FGL_ASSERT(FGL_AT_LUT * 4
	   == offsetof(struct lightrec_state, lut_base), lut_base);
FGL_ASSERT(FGL_AT_ADDR_MASK * 4
	   == offsetof(struct lightrec_state, addr_mask), addr_mask);

/* THE REACH ITSELF.  `mov.l @(disp,GBR),r0` has an eight-bit displacement
 * scaled by four, so the last word generated code can name is at +1020.
 * Everything asserted above is inside that; this says so once, rather than
 * leaving it to be rediscovered when a field is appended. */
FGL_ASSERT(FGL_STATE_WORDS <= 256, gbr_reach);

/* And the COP0 registers fgl actually keeps. */
FGL_ASSERT(COP0_SR == 12 && COP0_CAUSE == 13 && COP0_EPC == 14, cop0_regs);
