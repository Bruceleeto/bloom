#ifndef BLOOM_DLCHECK_H
#define BLOOM_DLCHECK_H

/* Returns nonzero if PRFC1 counts instructions issued linearly.  See
 * src/dlcheck.c: the deadline may not use the counter until this passes. */
int fgl_dl_check_prfc1(void);

#endif
