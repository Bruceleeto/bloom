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

#endif
