/* MMU-fault I/O classification — see plan.md.
 *
 * The PS1 I/O window (0x1f801000+) has no virtual mapping; a direct access
 * from generated code takes a DTLB miss and is serviced here, so emitted
 * code doesn't need a RAM-vs-I/O check on every load/store. */

#ifndef BLOOM_IOFAULT_H
#define BLOOM_IOFAULT_H

void iofault_init(void);
void iofault_report(void);

#endif
