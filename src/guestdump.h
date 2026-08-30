/* Guest machine state dump. See guestdump.c. */
#ifndef BLOOM_GUESTDUMP_H
#define BLOOM_GUESTDUMP_H

#include "bloom-config.h"

/* On-disk layout, little-endian throughout. Blobs follow the header in the
 * order the size fields appear. */
#define GUEST_DUMP_MAGIC	"BLOOMGD1"

struct guest_dump_hdr {
	char		magic[8];
	/* Size of this header as the writer laid it out. The reader lives in
	 * tools/lrtest, built by a different toolchain against its own copy of
	 * this struct, so a layout difference would otherwise be read as
	 * register values - a confusing wrong answer instead of an error. */
	unsigned int	hdr_size;
	unsigned int	version;
	unsigned int	pc;
	unsigned int	cycle;
	unsigned int	ram_size;
	unsigned int	bios_size;
	unsigned int	scratch_size;
	unsigned int	hw_size;
	unsigned int	pport_size;
	unsigned int	reserved;
	unsigned int	gpr[34];
	unsigned int	cp0[32];
	unsigned int	cp2d[32];
	unsigned int	cp2c[32];
};

#if WITH_PROF
void guest_dump(const char *tag);
#else
#define guest_dump(tag)		do { (void)(tag); } while (0)
#endif

#endif /* BLOOM_GUESTDUMP_H */
