// SPDX-License-Identifier: GPL-2.0-only
/*
 * PSX memory map configuration and MMU setup
 *
 * Copyright (C) 2024 Paul Cercueil <paul@crapouillou.net>
 */

#include <arch/cache.h>
#include <dc/cache.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <kos.h>

#include <libpcsxcore/psxmem.h>
#include <libpcsxcore/lightrec/mem.h>

#define OFFSET 0x0

extern u32 _arch_mem_top;

uintptr_t arch_stack_16m = 0x8cd60000 - CODE_BUFFER_SIZE;
uintptr_t arch_stack_32m = 0x8dd60000 - CODE_BUFFER_SIZE;

static void *do_memset(void *dst, int c, size_t len)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
	return memset(dst, c, len);
#pragma GCC diagnostic pop
}

int lightrec_init_mmap(void)
{
	unsigned int i;
	int err;

	mmu_init_basic();

#if BLOOM_OIX
	/* Operand cache in two halves by address bit 25: the lightrec state
	 * lives on the upper one (STATE_ALIAS in CMakeLists.txt), guest RAM,
	 * literal pools and everything else on the lower. */
	arch_dcache_purge_all();
	dcache_toggle_ocindex(true);
	printf("OIX enabled\n");
#endif

	/* Verify that the stack has been moved down */
	assert((_arch_mem_top & 0xfffff) == 0x60000);

	psxH = (s8 *)_arch_mem_top;
	psxR = (s8 *)(_arch_mem_top + 0x10000);
	psxP = (s8 *)(_arch_mem_top + 0x90000);
	psxM = (s8 *)(_arch_mem_top + 0xa0000);
	code_buffer = (void *)(_arch_mem_top + 0x2a0000);

	/* Create the PSX memory map using 18 pages:
	 * - two 1 MiB pages per RAM mirror, for a total of eight pages;
	 * - eight 64 KiB pages for the BIOS;
	 * - one 64 KiB page for the parallel port;
	 * - one 64 KiB page for the scratchpad and I/O area.
	 */

	for (i = 0; i < 4; i++) {
		/* Map first 1 MiB page of RAM mirror */
		err = mmu_page_map_static(OFFSET + 0x200000 * i, (uintptr_t)psxM,
					  PAGE_SIZE_1M, MMU_KERNEL_RDWR,
					  !UNCACHED_GUEST_RAM);
		if (err)
			goto handle_err;

		/* Map second 1 MiB page of RAM mirror */
		err = mmu_page_map_static(OFFSET + 0x200000 * i + 0x100000,
					  (uintptr_t)psxM + 0x100000,
					  PAGE_SIZE_1M, MMU_KERNEL_RDWR,
					  !UNCACHED_GUEST_RAM);
		if (err)
			goto handle_err;
	}

	printf("RAM mapped\n");

	/* Map the scratchpad only, one 1 KiB page.  The I/O window at
	 * 0x1f801000+ is deliberately NOT mapped: generated code compiles
	 * every unproven load/store to a plain masked access, and a real
	 * I/O touch takes a DTLB miss serviced by src/iofault.s.  C code
	 * reaches the backing store through the psxH host pointer. */
	err = mmu_page_map_static(OFFSET + 0x1f800000, (uintptr_t)psxH,
				  PAGE_SIZE_1K, MMU_KERNEL_RDWR, true);
	if (err)
		goto handle_err;

	printf("Scratchpad mapped, IO window faults\n");

	/* Map parallel port using one 64 KiB page */
	err = mmu_page_map_static(OFFSET + 0x1f000000, (uintptr_t)psxP,
				  PAGE_SIZE_64K, MMU_KERNEL_RDWR, true);
	if (err)
		goto handle_err;

	printf("Parallel port mapped\n");

	/* Map BIOS using eight 64 KiB pages */
	for (i = 0; i < 8; i++) {
		err = mmu_page_map_static(OFFSET + 0x1fc00000 + i * 0x10000,
					  (uintptr_t)psxR + i * 0x10000,
					  PAGE_SIZE_64K, MMU_KERNEL_RDONLY, true);
		if (err)
			goto handle_err;
	}

	printf("BIOS mapped\n");

	psxM = (void *)OFFSET;
	psxP = (void *)(OFFSET + 0x1f000000);
	/* psxH stays the HOST pointer: C reads and writes HW registers
	 * through it while the guest-visible window faults.  Only the first
	 * 1 KiB (the scratchpad) is aliased at 0x1f800000 for the guest;
	 * a C write to that 1 KiB goes through the host synonym, so any
	 * C path that bulk-writes scratchpad must purge those lines. */
	psxR = (void *)(OFFSET + 0x1fc00000);

	/* Clear pages */
	do_memset(psxM, 0x0, 0x200000);
	do_memset(psxH, 0x0, 0x10000);
	do_memset(psxP, 0xff, 0x10000);

	{
		extern void bloom_iofault_install(void);
		bloom_iofault_install();
	}

	printf("Memory-map succeeded.\n"
	       "RAM: 0x%x BIOS: 0x%x SCRATCH: 0x%x CODE: 0x%x\n",
	       (unsigned int)psxM, (unsigned int)psxR, (unsigned int)psxH,
	       (unsigned int)code_buffer);

	return 0;

handle_err:
	printf("Unable to memory-map PSX memories\n");
	lightrec_free_mmap();
	return err;
}

void lightrec_free_mmap(void)
{
	mmu_shutdown();
}

bool load_bios(int fd)
{
	return fs_read(fd, (s8 *)(_arch_mem_top + 0x10000), 0x80000) == 0x80000;
}
