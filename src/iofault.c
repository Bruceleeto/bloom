// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fault-driven guest I/O - installer.
 *
 * KOS's exception "vector table" is a code block at VBR; the TLB-miss slot
 * at VBR+0x400 is six bytes (nop; bra _irq_save_regs; mov #2,r4) followed
 * by half a kilobyte of zero padding.  We overwrite the slot with a far
 * jump to bloom_iofault_entry (src/iofault.s) and recover the original
 * branch target so non-I/O misses still chain into KOS untouched.
 */

#include <arch/cache.h>
#include <arch/irq.h>
#include <kos.h>
#include <stdint.h>
#include <stdio.h>

extern void bloom_iofault_entry(void);
extern uint32_t bloom_iofault_chain_target;

void bloom_iofault_install(void)
{
	uint32_t vbr;
	uint16_t *slot;
	int32_t disp;

	__asm__ __volatile__("stc vbr, %0" : "=r"(vbr));
	slot = (uint16_t *)(vbr + 0x400);

	/* Expect KOS's "nop; bra _irq_save_regs; mov #2,r4". */
	if ((slot[1] & 0xf000) != 0xa000) {
		printf("iofault: unexpected VBR+0x400 contents %04x %04x, not installed\n",
		       slot[0], slot[1]);
		return;
	}

	disp = ((int32_t)(slot[1] << 20)) >> 20;
	bloom_iofault_chain_target =
		(uint32_t)&slot[1] + 4 + (uint32_t)(disp * 2);

	/* mov.l @(1,pc),r0 ; jmp @r0 ; nop ; nop ; .long entry */
	slot[0] = 0xd001;
	slot[1] = 0x402b;
	slot[2] = 0x0009;
	slot[3] = 0x0009;
	*(uint32_t *)(vbr + 0x408) = (uint32_t)&bloom_iofault_entry;

	dcache_flush_range(vbr + 0x400, 32);
	icache_flush_range(vbr + 0x400, 32);

	printf("iofault: owned VBR+0x400, chain %08lx\n",
	       (unsigned long)bloom_iofault_chain_target);
}
