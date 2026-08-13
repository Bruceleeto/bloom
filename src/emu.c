// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bloom!
 *
 * Copyright (C) 2024 Paul Cercueil <paul@crapouillou.net>
 */


#include <kos.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include <libpcsxcore/misc.h>
#include <libpcsxcore/plugins.h>
#include <libpcsxcore/psxcommon.h>
#include <libpcsxcore/psxmem.h>
#include <libpcsxcore/r3000a.h>
#include <libpcsxcore/sio.h>
#include <psemu_plugin_defs.h>

#include <arch/gdb.h>
#include <dc/cdrom.h>
#include <dc/video.h>

#include <sys/stat.h>

#include "bloom-config.h"
#include "census.h"
#include "emu.h"
#include "jitprof.h"
#include "prof.h"
#include "pvr.h"

int fs_fat_init(void);
void fs_fat_shutdown(void);

static bool is_exe;

extern uint32_t _arch_mem_top;

bool started;

void SysPrintf(const char *fmt, ...) {
	va_list list;

	va_start(list, fmt);
	vfprintf(stdout, fmt, list);
	va_end(list);
}

void SysMessage(const char *fmt, ...) {
	va_list list;
	char msg[512];
	int ret;

	va_start(list, fmt);
	ret = vsnprintf(msg, sizeof(msg), fmt, list);
	va_end(list);

	if (ret < sizeof(msg) && msg[ret - 1] == '\n')
		msg[ret - 1] = 0;

	SysPrintf("%s\n", msg);
}

static void init_config(void)
{
	memset(&Config, 0, sizeof(Config));

	Config.PsxAuto = 1;
	Config.cycle_multiplier = CYCLE_MULT_DEFAULT;
	Config.GpuListWalking = -1;
	Config.FractionalFramerate = -1;

	strcpy(Config.Mcd1, WITH_MCD1_PATH);
	strcpy(Config.Mcd2, WITH_MCD2_PATH);

	strcpy(Config.PluginsDir, "plugins");
	strcpy(Config.Gpu, "builtin_gpu");
	strcpy(Config.Spu, "builtin_spu");
}

static unsigned int screenshot_num;

static void emu_screenshot(uint8_t port, uint32_t)
{
	maple_device_t *dev;
	cont_state_t *state;
	char buf[1024];

	dev = maple_enum_dev(port, 0);
	state = maple_dev_status(dev);

	if (state->start) {
		snprintf(buf, sizeof(buf), "/pc/screenshot%03u.ppm",
			 ++screenshot_num);
		vid_screen_shot(buf);
	}
}

static void emu_exit(uint8_t, uint32_t)
{
	psxRegs.stop = 1;
}

bool emu_check_cd(const char *path)
{
	SetIsoFile(path);

	ReloadCdromPlugin();

	if (OpenPlugins() < 0) {
		fprintf(stderr, "Could not open plugins\n");
		return false;
	}

	is_exe = !!strstr(path, ".exe");

	if (!is_exe && CheckCdrom() != 0) {
		ClosePlugins();
		return false;
	}

	return true;
}

#define BENCH_EXE_WINDOW_MS 5000
#define BENCH_CD_WINDOW_MS 60000
#define PSX_CLOCK_HZ 33868800

/* A frame budget stalls forever if the guest stops producing frames, so the
 * wall-clock window stays on as a backstop.  Generous, because the whole point
 * of the frame budget is that a slower emulator is allowed to take longer. */
#define BENCH_FRAME_CAP_MS 300000

/* Wall time spent inside generated code, counted in the lightrec plugin */
extern uint64_t bench_exec_us;

/* Guest display flips since boot, from the vout flip callback */
extern unsigned int bloom_frame_count;

/* Dump the hot blocks' emitted SH-4.  Lives in the lightrec plugin, which
 * owns lightrec_state and the address list — see lightrec_dump_hot(). */
void lightrec_dump_hot(void);

static unsigned int bench_window_ms;
static unsigned int bench_frames;

static void *bench_stop_thd(void *arg)
{
	uint64_t t0, t, t_prev, exec, exec_prev, exec0, insns;
	uint32_t cycle, cycle0, cycle_prev, cycles, ms, pc;
	uint32_t frames, frames0, frames_prev, frames_done;

	/* Don't count the BIOS boot: arm once the PC reaches the game EXE
	 * (RAM above the kernel's 64K, below the shell at 0x30000). */
	do {
		thd_sleep(100);
		pc = psxRegs.pc & 0x1fffffff;
	} while (!psxRegs.stop && (pc < 0x10000 || pc >= 0x30000));

	if (psxRegs.stop)
		return NULL;

	printf("BENCH: armed at pc %08lx, budget %u frames%s\n",
	       (unsigned long)psxRegs.pc, bench_frames,
	       bench_frames ? "" : " (wall-clock window)");

	/* Sample over exactly the window the numbers are taken from. */
	jitprof_start();
	prof_start();
	prof_calibrate();
	census_reset();

	t0 = t_prev = timer_ms_gettime64();
	cycle0 = cycle_prev = psxRegs.cycle;
	exec0 = exec_prev = bench_exec_us;
	frames0 = frames_prev = bloom_frame_count;

	while (!psxRegs.stop) {
		thd_sleep(1000);

		t = timer_ms_gettime64();
		cycle = psxRegs.cycle;
		exec = bench_exec_us;
		frames = bloom_frame_count;

		ms = t - t_prev;
		cycles = cycle - cycle_prev;
		insns = (uint64_t)cycles * 100 / Config.cycle_multiplier;

		printf("BENCH: %llu k/s, %lu.%lu fps, %u%% realtime,"
		       " %u%% in guest code\n",
		       (unsigned long long)(insns / ms),
		       (unsigned long)((uint64_t)(frames - frames_prev) * 10000 / ms / 10),
		       (unsigned long)((uint64_t)(frames - frames_prev) * 10000 / ms % 10),
		       (unsigned int)((uint64_t)cycles * 100
				      / ((uint64_t)ms * (PSX_CLOCK_HZ / 1000))),
		       (unsigned int)((exec - exec_prev) / (ms * 10)));

		t_prev = t;
		cycle_prev = cycle;
		exec_prev = exec;
		frames_prev = frames;

		if (bench_frames) {
			if (frames - frames0 >= bench_frames
			    || t - t0 >= BENCH_FRAME_CAP_MS)
				break;
		} else if (t - t0 >= bench_window_ms) {
			break;
		}
	}

	ms = timer_ms_gettime64() - t0;
	if (!ms)
		ms = 1;
	insns = (uint64_t)(psxRegs.cycle - cycle0) * 100 / Config.cycle_multiplier;
	frames_done = bloom_frame_count - frames0;

	printf("BENCH: total %llu guest instructions in %lu ms"
	       " -> %llu k/s, %lu frames (%lu.%lu fps), %u%% in guest code\n",
	       (unsigned long long)insns, (unsigned long)ms,
	       (unsigned long long)(insns / ms),
	       (unsigned long)frames_done,
	       (unsigned long)((uint64_t)frames_done * 10000 / ms / 10),
	       (unsigned long)((uint64_t)frames_done * 10000 / ms % 10),
	       (unsigned int)((bench_exec_us - exec0) / (ms * 10)));

	if (bench_frames && frames_done < bench_frames)
		printf("BENCH: SHORT — %lu of %u frames before the %u ms cap;"
		       " this run is not comparable\n",
		       (unsigned long)frames_done, bench_frames,
		       BENCH_FRAME_CAP_MS);

	fflush(stdout);

	prof_report(ms, frames_done, insns);
	census_report(ms, frames_done);
	jitprof_report();

	if (WITH_BLOCKDUMP)
		lightrec_dump_hot();

	psxRegs.stop = 1;

	return NULL;
}

/* Copy of the default params, but with FSAA enabled */
static pvr_init_params_t pvr_init_params_fsaa = {
	.opb_sizes = {
		PVR_BINSIZE_16,
		PVR_BINSIZE_0,
		HARDWARE_ACCELERATED ? PVR_BINSIZE_16 : PVR_BINSIZE_0,
		(HARDWARE_ACCELERATED && WITH_CLIPPING) ? PVR_BINSIZE_8 : PVR_BINSIZE_0,
		HARDWARE_ACCELERATED ? PVR_BINSIZE_16 : PVR_BINSIZE_0,
	},
	.vertex_buf_size = 768 * 1024,
	.fsaa_enabled = WITH_FSAA,
	.opb_overflow_count = 3,
};

int main(int argc, char **argv)
{
	enum vid_display_mode_generic video_mode;
	const char *bench_path = NULL;
	bool should_exit;
	bool bench;
	file_t fd;

	if (WITH_GDB)
		gdb_init();

	if (WITH_IDE || WITH_SDCARD)
		fs_fat_init();

	if (WITH_IDE)
		ide_init();
	if (WITH_SDCARD)
		sdcard_init();

	input_init();

	init_config();

	if (WITH_CACHED_STDIO)
		setvbuf(stdout, NULL, _IOFBF, 0);

	if (EmuInit() == -1) {
		fprintf(stderr, "Could not initialize PCSX core\n");
		return 1;
	}

	if (LoadPlugins() < 0) {
		fprintf(stderr, "Could not load plugins\n");
		return 1;
	}

	plugin_call_rearmed_cbs();

	cont_btn_callback(0, CONT_RESET_BUTTONS, emu_exit);
	cont_btn_callback(0, CONT_START | CONT_DPAD_UP, emu_screenshot);

	do {
		started = false;

		bench = false;

		if (WITH_BENCH) {
			static const char * const bench_paths[] = {
				"/ide/bloom/game.bin",
				"/rd/prog.exe",
			};
			unsigned int i;

			for (i = 0; !bench && i < 2; i++) {
				fd = fs_open(bench_paths[i], O_RDONLY);
				if (fd != -1) {
					fs_close(fd);
					bench_path = bench_paths[i];
					bench = true;
				}
			}

			if (bench) {
				bool exe = !!strstr(bench_path, ".exe");

				bench_window_ms = exe ? BENCH_EXE_WINDOW_MS
						      : BENCH_CD_WINDOW_MS;

				/* The .exe fixture never flips, so a frame
				 * budget would never retire on it. */
				bench_frames = exe ? 0 : WITH_BENCH_FRAMES;
			}
		}

		if (bench || WITH_GAME_PATH[0]) {
			emu_check_cd(bench ? bench_path : WITH_GAME_PATH);
			ClosePlugins();
		} else {
			vid_set_mode(DM_640x480, PM_RGB888P);
			pvr_init_defaults();

			should_exit = runMenu();
			ClosePlugins();
			pvr_shutdown();

			if (should_exit)
				break;
		}

		if (WITH_480P)
			video_mode = DM_640x480;
		else
			video_mode = DM_320x240;

		if (WITH_24BPP)
			vid_set_mode(video_mode, PM_RGB888P); /* 24-bit */
		else
			vid_set_mode(video_mode, PM_RGB565); /* 16-bit */

		/* Re-init PVR without translucent polygon autosort, and optional FSAA */
		pvr_init(&pvr_init_params_fsaa);

		pvr_set_vertical_scale(1.0f);

		PVR_SET(PVR_OBJECT_CLIP, 0.00001f);

		if (HARDWARE_ACCELERATED)
			pvr_renderer_init();

		started = true;
		OpenPlugins();

		EmuReset();

		if (UsingIso() && !!strncmp(GetIsoFile(), "/cd", sizeof("/cd") - 1))
			cdrom_spin_down();

		if (is_exe)
			Load(GetIsoFile());
		else
			LoadCdrom();

		mcd_fs_init();

		psxRegs.stop = 0;

		if (bench) {
			/* The profiler's brackets all run on this thread;
			 * the watcher below only arms and reports. */
			prof_claim();

			thd_create(1, bench_stop_thd, NULL);
		}

		while (!psxRegs.stop)
			psxCpu->Execute(&psxRegs);

		ClosePlugins();

		if (HARDWARE_ACCELERATED)
			pvr_renderer_shutdown();

		pvr_shutdown();
		mcd_fs_shutdown();
	} while (!bench && !WITH_GAME_PATH[0]);

	printf("Exit...\n");
	EmuShutdown();
	ReleasePlugins();

	input_shutdown();

	if (WITH_SDCARD)
		sdcard_shutdown();
	if (WITH_IDE)
		ide_shutdown();
	if (WITH_IDE || WITH_SDCARD)
		fs_fat_shutdown();

	return 0;
}

mode_t umask(mode_t mask) {
	return mask;
}

int chmod(const char *pathname, mode_t mode)
{
	return 0;
}

void lightrec_code_inv(void *ptr, uint32_t len)
{
	icache_sync_range((uintptr_t)ptr, len);
}

static void copy_bios(void)
{
	if (WITH_EMBEDDED_BIOS_PATH)
		memcpy((uint8_t *)(_arch_mem_top + 0x10000), _bss_start, 0x80000);
}
KOS_INIT_EARLY(copy_bios);

void psxMemReset()
{
	bool success = false;
	file_t fd;

	if (WITH_BIOS_PATH[0]) {
		fd = fs_open(WITH_BIOS_PATH, O_RDONLY);

		if (fd != -1) {
			success = load_bios(fd);
			fs_close(fd);
		}
	}

	Config.HLE = !success && !WITH_EMBEDDED_BIOS_PATH;
	Config.SlowBoot = 1;
}
