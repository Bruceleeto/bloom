/* SH7750 hardware performance counters, per BENCH window, split by area.
 * PRFC0 counts elapsed cycles always; PRFC1 rotates through every event
 * one window at a time, so one long hardware run prices everything. */
#ifndef BLOOM_PERF_H
#define BLOOM_PERF_H

enum perf_area {
	PERF_REST = 0,	/* JIT, GTE, pcsx core, everything unbracketed */
	PERF_GPU,	/* GP0 decode + renderer: dma chain, writeDataMem, updateLace */
	PERF_FLIP,	/* vout flip: scene finish, PVR wait, BENCH print */
	PERF_CD,	/* cdrom reads */
	PERF_GP1,	/* GPUwriteStatus (GP1 register writes) */
	PERF_JIT,	/* lightrec_execute: generated code, dispatcher, C wrappers */
	PERF_HW,	/* psxHwRead/Write: hardware register C path */
	PERF_SPU,	/* SPU_async: mixer */
	PERF_NB_AREAS,
};

enum perf_area perf_area_switch(enum perf_area area);
void perf_report(unsigned int frames, unsigned int ms);

/* The PERF_HW bracket costs 12 uncached PMCTR reads per guest HW register
 * access (thousands per frame), often wrapping ~3 instructions of work —
 * and the GPUSTAT poll-skipper calls psxHwRead32 inside its fast-forward
 * loop. Off by default: hw time folds into the caller's bucket (jit).
 * Rebuild with WITH_PERF_HW=1 only for a run that needs the hw column. */
#ifndef WITH_PERF_HW
#define WITH_PERF_HW 0
#endif

#if WITH_PERF_HW
#define perf_hw_enter()  perf_area_switch(PERF_HW)
#define perf_hw_exit(pa) perf_area_switch(pa)
#else
#define perf_hw_enter()  PERF_REST
#define perf_hw_exit(pa) ((void)(pa))
#endif

#endif
