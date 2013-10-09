#ifndef __MORPHOLOGY__
#define __MORPHOLOGY__

#include "peopledetection.h"

void _imerode(unsigned char * const & map, 
              const int nrows, const int ncols,
              int sze_h = 3, int sze_v = 0);

void _imdilate(unsigned char * const & map, 
               const int nrows, const int ncols,
               int sze_h = 3, int sze_v = 0);

#endif
