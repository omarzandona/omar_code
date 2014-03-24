#ifndef __BLOB_DETECTION__
#define __BLOB_DETECTION__

#include <stdlib.h>
#include "directives.h"

#ifdef USE_NEW_DETECTION

#ifdef USE_BINNING_WITH_CHECK
const unsigned char OUT_OF_RANGE_OR_STEREO_FAILURE = 0;  ///< disparity 0 corresponds to stereo failure in general and this is very frequent with out-of-range (this depends on how FPGA works).
const unsigned char UNIFORM_ZONE_OR_DISP_1 = 16;  ///< disparity 1 corresponds to uniform surfaces or the real disparity 1 (this depends on how FPGA works).
/*!
  When stereo failure corresponds to high correlation but multiple results means that probably there 
  is a uniform surface. This failure is differentiated from general failure to avoid problem in out-of-range managing
  due to uniform background. disparity 1 could still represent the real
  disparity 1 but in normal working consitions this disparity should not appear (it corresponds to 
  underfloor objects.
*/
#endif

#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */

/* Peaks data */
typedef struct _PeakProps {
  int x, y, z, a, 
#ifdef USE_NEW_DETECTION2
    wx, wy;
#else
    w; 
#endif
} tPeakProps;

const int binning = 3; // if changed, the kernel dimension and values should be changed coherently

void
compute_binned_nrow_ncols(
  const int nrows, const int ncols, 
  const int binning,
  const int border_x, const int border_y,
  int & bnrows, int & bncols);

tPeakProps* peak_detection(
  unsigned char * const & bmap,
  const int bnrows,
  const int bncols,
  int & num_peaks);

void image_binning(
  const unsigned char * const & map, 
  const int nrows, const int ncols, 
  const int binning,
  const int border_x, const int border_y,
  unsigned char * const & bmap, 
  unsigned char * const & i_bp_mask,
  int & bnrows, int & bncols);

void
peak_unbinning(
  tPeakProps* const & peaks,
  const int num_peaks, 
  const int binning, 
  const int border_x,
  const int border_y);


#endif
#endif
