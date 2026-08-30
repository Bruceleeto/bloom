/* PC sampling profiler. See prof.c. */
#ifndef BLOOM_PROF_H
#define BLOOM_PROF_H

#include <stdint.h>

#include "bloom-config.h"

#if WITH_PROF

void prof_init(void);
void prof_shutdown(void);

/* Call once per BENCH window. After WITH_PROF_WINDOWS of them the histogram
 * is written out and sampling stops, so a profile lands whether or not the
 * run is exited cleanly. */
void prof_window(void);

/* Write the histogram out and reset it. `tag` is used in the filename so
 * several runs can be told apart. */
void prof_dump(const char *tag);

#else

#define prof_init()		do { } while (0)
#define prof_window()		do { } while (0)
#define prof_shutdown()		do { } while (0)
#define prof_dump(tag)		do { (void)(tag); } while (0)

#endif

#endif /* BLOOM_PROF_H */
