#ifndef __HUNGARIAN_METHOD__
#define __HUNGARIAN_METHOD__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

int* matching(const unsigned int * const & C, const int nrow, const int ncol, const int max_cost, int * p_result);

#endif
