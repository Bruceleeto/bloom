/*
 * Guest machine state dump.
 *
 * Writes PSX RAM, BIOS, scratchpad, hardware registers and the guest register
 * file to /pc/, so tools/lrtest can replay the exact scene the profiler
 * measured instead of booting the BIOS.
 *
 * That matters because the two are not the same code. The differential
 * harness has only ever run BIOS boot: a few dozen blocks of setup that no
 * game spends time in. Everything measured about emitted code so far - the
 * nop share, host instructions per guest opcode - describes those blocks, not
 * Spyro's. Feeding it a real scene turns those numbers into something worth
 * acting on, and puts the game's actual blocks in front of the interpreter
 * oracle for the first time.
 *
 * The dump is a snapshot, not a savestate: it is enough to re-execute from,
 * not to resume the emulator from.
 */

#include "bloom-config.h"

#include "guestdump.h"

#if WITH_PROF

#include <stdio.h>
#include <string.h>

#include <libpcsxcore/psxmem.h>
#include <libpcsxcore/r3000a.h>

#define PSX_RAM_SIZE		0x200000
#define PSX_BIOS_SIZE		0x80000
#define PSX_SCRATCH_SIZE	0x400
#define PSX_HW_SIZE		0x8000
#define PSX_PPORT_SIZE		0x10000

/* dcload's host filesystem does not survive a multi-megabyte fwrite: the call
 * reports success and the file on the PC ends up a fraction of the size. Write
 * in chunks and check each one, then verify the final size against what was
 * asked for - a silently truncated dump would be loaded as a valid scene with
 * most of RAM zeroed, which is far worse than a failure. */
#define GUEST_DUMP_CHUNK	(16 * 1024)

static int write_blob(FILE *f, const void *p, unsigned int len, const char *what)
{
	const char *src = p;
	unsigned int done = 0;

	if (!p || !len)
		return 0;

	while (done < len) {
		unsigned int n = len - done;

		if (n > GUEST_DUMP_CHUNK)
			n = GUEST_DUMP_CHUNK;

		if (fwrite(src + done, 1, n, f) != n) {
			printf("guestdump: short write on %s at %u/%u\n",
			       what, done, len);
			return -1;
		}
		done += n;
	}
	return 0;
}

void guest_dump(const char *tag)
{
	struct guest_dump_hdr h;
	char path[64];
	FILE *f;
	unsigned int i;

	snprintf(path, sizeof(path), "/pc/bloom_guest_%s.bin", tag ? tag : "run");

	f = fopen(path, "wb");
	if (!f) {
		printf("guestdump: cannot open %s (is /pc mapped? "
		       "dc-tool-ip -m <dir>)\n", path);
		return;
	}

	memset(&h, 0, sizeof(h));
	memcpy(h.magic, GUEST_DUMP_MAGIC, sizeof(h.magic));
	h.hdr_size = (unsigned int)sizeof(h);
	h.version = 1;
	h.pc = psxRegs.pc;
	h.cycle = psxRegs.cycle;
	h.ram_size = PSX_RAM_SIZE;
	h.bios_size = PSX_BIOS_SIZE;
	h.scratch_size = PSX_SCRATCH_SIZE;
	h.hw_size = PSX_HW_SIZE;
	h.pport_size = psxP ? PSX_PPORT_SIZE : 0;

	/* psxRegs is the emulator's copy and is in step with lightrec's here:
	 * the plugin syncs them around every exit from generated code, and
	 * this runs from the frame loop, not from inside a block. */
	for (i = 0; i < 34; i++)
		h.gpr[i] = psxRegs.GPR.r[i];
	for (i = 0; i < 32; i++) {
		h.cp0[i] = psxRegs.CP0.r[i];
		h.cp2d[i] = psxRegs.CP2D.r[i];
		h.cp2c[i] = psxRegs.CP2C.r[i];
	}

	if (fwrite(&h, 1, sizeof(h), f) != sizeof(h)) {
		printf("guestdump: short write on header\n");
		fclose(f);
		return;
	}

	if (write_blob(f, psxM, h.ram_size, "ram") ||
	    write_blob(f, psxR, h.bios_size, "bios") ||
	    write_blob(f, psxH, h.scratch_size, "scratch") ||
	    write_blob(f, psxH + 0x1000, h.hw_size, "hwregs") ||
	    write_blob(f, psxP, h.pport_size, "pport")) {
		fclose(f);
		return;
	}

	{
		long expect = (long)sizeof(h) + h.ram_size + h.bios_size +
			      h.scratch_size + h.hw_size + h.pport_size;
		long actual = ftell(f);

		fclose(f);

		if (actual != expect) {
			printf("guestdump: %s is %ld bytes, expected %ld - "
			       "dump is unusable\n", path, actual, expect);
			return;
		}

		printf("guestdump: pc=%08x cycle=%u -> %s (%ld KiB)\n",
		       h.pc, h.cycle, path, actual >> 10);
	}
}

#endif /* WITH_PROF */
