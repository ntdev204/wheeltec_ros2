












#ifndef NAV2_AMCL__PORTABLE_UTILS_HPP_
#define NAV2_AMCL__PORTABLE_UTILS_HPP_

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_DRAND48


static double drand48(void)
{
  return ((double)rand()) / RAND_MAX;
}

static void srand48(long int seedval)
{
  srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif
