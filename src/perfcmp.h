/* The five-event head-to-head against the GNU lightning tree.  See
 * src/perfcmp.c and build/docs/issue_not_cache.md. */
#ifndef BLOOM_PERFCMP_H
#define BLOOM_PERFCMP_H

#ifndef BLOOM_PERFCMP
#define BLOOM_PERFCMP 0
#endif

#if BLOOM_PERFCMP
void perfcmp_report(unsigned frames);
#else
#define perfcmp_report(frames) ((void)(frames))
#endif

#endif
