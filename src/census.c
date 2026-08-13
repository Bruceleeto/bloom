/* See census.h. */

#include "census.h"
#include "prof.h"

static const char * const census_names[CENSUS_N] = {
	[CENSUS_FRAMES]		= "frames",
	[CENSUS_VERTICES]	= "vertices",
	[CENSUS_RECORDS]	= "records",
	[CENSUS_BINDS]		= "binds",
	[CENSUS_TEX_LOOKUPS]	= "tex lookups",
	[CENSUS_TEX_MISSES]	= "tex misses",
	[CENSUS_TEX_EVICTIONS]	= "tex evictions",
	[CENSUS_GTE_CMDS]	= "gte cmds",
	[CENSUS_RW_GENERIC]	= "rw generic",
	[CENSUS_HW_READ]	= "hw reads",
	[CENSUS_HW_WRITE]	= "hw writes",
	[CENSUS_EXEC_ENTRIES]	= "exec entries",
	[CENSUS_COMPILES]	= "compiles",
	[CENSUS_INVALIDATES]	= "invalidates",
	[CENSUS_CD_SECTORS]	= "cd sectors",
	[CENSUS_SKIPPED_CYCLES]	= "skipped cycles",
};

const char *census_name(census_id id)
{
	if ((unsigned int)id >= CENSUS_N)
		return "?";

	return census_names[id];
}

#ifdef BLOOM_CENSUS

#include <stdio.h>
#include <string.h>

uint32_t census_ctr[CENSUS_N];
uint32_t census_gte_func[64];
uint32_t census_gp0_op[256];

void census_reset(void)
{
	memset(census_ctr, 0, sizeof(census_ctr));
	memset(census_gte_func, 0, sizeof(census_gte_func));
	memset(census_gp0_op, 0, sizeof(census_gp0_op));
}

/* The per-unit costs, which are the only figures that survive being compared
 * against an emulator drawing a different scene.  Printed as nanoseconds so
 * the small ones do not truncate to zero; bloop quotes microseconds. */
static void census_cost(const char *what, prof_bucket b, census_id id)
{
	uint64_t ns = prof_ns(b);
	uint32_t n = census_ctr[id];

	if (!n || !ns)
		return;

	printf("CENSUS: %-14s %10lu ns each  (%s %lu ms / %s %lu)\n",
	       what, (unsigned long)(ns / n),
	       prof_name(b), (unsigned long)(ns / 1000000u),
	       census_name(id), (unsigned long)n);
}

/* Same, for a bucket whose denominator is two counters added together. */
static void census_cost2(const char *what, prof_bucket b,
			 census_id a, census_id c)
{
	uint64_t ns = prof_ns(b);
	uint32_t n = census_ctr[a] + census_ctr[c];

	if (!n || !ns)
		return;

	printf("CENSUS: %-14s %10lu ns each  (%s %lu ms / %lu accesses)\n",
	       what, (unsigned long)(ns / n),
	       prof_name(b), (unsigned long)(ns / 1000000u),
	       (unsigned long)n);
}

void census_report(uint32_t wall_ms, uint32_t frames)
{
	unsigned int i;
	int first;

	printf("CENSUS: over %lu ms, %lu frames\n",
	       (unsigned long)wall_ms, (unsigned long)frames);

	for (i = 0; i < CENSUS_N; i++) {
		if (!census_ctr[i])
			continue;

		printf("CENSUS: %-14s %10lu  %8lu /frame\n",
		       census_name(i), (unsigned long)census_ctr[i],
		       (unsigned long)(frames ? census_ctr[i] / frames : 0));
	}

	for (i = 0, first = 1; i < 64; i++) {
		if (!census_gte_func[i])
			continue;

		if (first) {
			printf("CENSUS: GTE by function:");
			first = 0;
		}

		printf(" %02x=%lu", i, (unsigned long)census_gte_func[i]);
	}
	if (!first)
		printf("\n");

	for (i = 0, first = 1; i < 256; i++) {
		if (!census_gp0_op[i])
			continue;

		if (first) {
			printf("CENSUS: GP0 by opcode:");
			first = 0;
		}

		printf(" %02x=%lu", i, (unsigned long)census_gp0_op[i]);
	}
	if (!first)
		printf("\n");

	/* The rows the comparison actually turns on. */
	census_cost("per bind", PROF_RENDER, CENSUS_BINDS);
	census_cost("per GTE cmd", PROF_GTE, CENSUS_GTE_CMDS);
	census_cost("per exec entry", PROF_GUEST, CENSUS_EXEC_ENTRIES);
	census_cost("per compile", PROF_COMPILE, CENSUS_COMPILES);

	/* IO and MEMCLASS each cover reads and writes together, so the
	 * denominator has to be both — dividing the whole bucket by writes
	 * alone overstated the cost by 11x in the first run. */
	census_cost2("per hw access", PROF_IO,
		     CENSUS_HW_READ, CENSUS_HW_WRITE);
	census_cost2("per crossing", PROF_MEMCLASS,
		     CENSUS_HW_READ, CENSUS_HW_WRITE);

	fflush(stdout);
}

#endif /* BLOOM_CENSUS */
