#include "directives.h"

#ifdef USE_NEW_DETECTION

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "blob_detection.h"
#include "default_parms.h"
#include "morphology.h"

#include <limits.h>

#define USE_PEAKS_PRUNING

#define FROM_DISP_TO_SHOULDER 5 // notice that shoulder width divided by baseline (about 40cm / 5cm = 8) gives the scale factor that multiplied by the disparity gives the pixel width
                                // here disparities are multiplied by 16 and there is a binning of 3 so we have to divide by 16*3 obtaining the following relation: length_px = disp*8/(16*3) ~ disp/6

#if !defined(PERFORMANCE_TEST) && !defined(SAVE_RESULT) && defined(SHOW_RESULT)
//#define SHOW_ORIG_IMAGE
#define SHOW_BINNED_IMAGE
#define SHOW_UNBINNED_IMAGE
#define SHOW_DILATED_IMAGE
#define SHOW_ERODED_IMAGE
#define SHOW_PEAKS_IMAGE
#define SHOW_CLUSTERED_PEAKS_IMAGE
using namespace std;
#endif

//#define USE_PEAK_MIN_DIST

//#define USE_REPLICATE_IN_CONV_H
//#define USE_REPLICATE_IN_CONV_V

/* Blob detection specific parameters */
const int   min_disp = 3*16-1;
const float th_fact = 0.7f;
const int   person_head_width = 21; //a person head cannot be more than 40 pixel along x in the 160x120 map

#ifdef USE_PEAK_MIN_DIST
const int   peak_min_dist_orig = 6; // min_dist among peaks in the 160x120 image
#endif
const int   strel_sze_orig_h = 15; ///< closing on the 160x120 map, if 0 no closing is performed
const int   strel_sze_orig_v = 9; ///< closing on the 160x120 map, if 0 no closing is performed
const int   strel_sze2_orig_h = 15; ///< opening on the 160x120 map, if 0 no opening is performed
const int   strel_sze2_orig_v = 9; ///< opening on the 160x120 map, if 0 no opening is performed

const int   min_area_orig = 12*12; // min area of a blob in the 160x120 map, if 0 not check is performed
const int   min_w_orig = 12; // min width of a blob in the 160x120 map, if 0 not check is performed
const int   min_h_orig = 12; // min height of a blob in the 160x120 map, if 0 not check is performed

/* binned parameters */
const int   strel_sze_h = strel_sze_orig_h/binning + (strel_sze_orig_h%binning > 0); ///< closing on the binned image
const int   strel_sze_v = strel_sze_orig_v/binning + (strel_sze_orig_v%binning > 0); ///< closing on the binned image
const int   strel_sze2_h = strel_sze2_orig_h/binning + (strel_sze2_orig_h%binning > 0); ///< opening on the binned image
const int   strel_sze2_v = strel_sze2_orig_v/binning + (strel_sze2_orig_v%binning > 0); ///< opening on the binned image
const int   min_area = min_area_orig/(binning*binning); // min area of a blob in the binned image
const int   min_w = min_w_orig/binning; // min width of a blob in the binned image
const int   min_h = min_h_orig/binning; // min height of a blob in the binned image
const int   BINNED_DIM = (NX-2*BORDER_X+binning)*(NY-2*BORDER_Y+binning)/(binning*binning); ///< binned dimension obtained using round up

#ifdef USE_PEAK_MIN_DIST
const int   peak_min_dist = max(1,peak_min_dist_orig/binning); // min_dist among peaks in the binned image
#endif

/* Kernel dependent parameters */
#define KERNEL_X_DIM 13 //odd number
#define KERNEL_Y_DIM 13 //11 // odd number
//#define SUM_Z 53 // sum of all the gaussian coefficients of the 2D mask (not the two 1D masks but the originale 2D mask)
#define NORM_FACT 256 // if 1024 then a simple gaussian filter is performed: it should be 1) less then 1024, 2) a power of two, and 3) such that (SUM_Z*255*1024^2)/(NORM_FACT^2) can be represented by an unsigned int

unsigned int kernel_x[KERNEL_X_DIM]; //= {139, 255, 421, 621, 820, 969, 1024, 969, 820, 621, 421, 255, 139};
unsigned int kernel_y[KERNEL_Y_DIM]; //= {139, 255, 421, 621, 820, 969, 1024, 969, 820, 621, 421, 255, 139};
//const unsigned int threshold = (unsigned int)(th_fact*min_disp*SUM_Z);

/* Support data */
const int MIN_PEAKS_DIST = 2;
const int MAX_NUM_PEAKS = ((((NX-2*BORDER_X)/binning)/(MIN_PEAKS_DIST+1))*(((NY-2*BORDER_Y)/binning)/(MIN_PEAKS_DIST+1)))/2;


void
compute_binned_nrow_ncols(const int nrows, const int ncols, 
               const int binning,
               const int border_x, const int border_y,
               int & bnrows, int & bncols)
{
  int central_num_rows = nrows-2*border_y;
  int central_num_cols = ncols-2*border_x;
  bnrows = central_num_rows/binning + (central_num_rows%binning>0);
  bncols = central_num_cols/binning + (central_num_cols%binning>0);
  assert(bnrows*bncols <= BINNED_DIM);
}


void image_binning(const unsigned char * const & map, 
  const int nrows, const int ncols, 
  const int binning,
  const int border_x, const int border_y,
  unsigned char * const & bmap,
  unsigned char * const & i_bp_mask,
  int & bnrows, int & bncols)
{
  assert(border_x>binning);
  assert(border_y>binning);

  compute_binned_nrow_ncols(nrows, ncols, binning, border_x, border_y, bnrows, bncols);

  unsigned char** conv_ptr = (unsigned char**) malloc(binning*sizeof(unsigned char*)); // puntatori di riga

  unsigned char* ptr_bmap = (unsigned char*)bmap;
  unsigned int sum_square;
  int last_col;

  memset(i_bp_mask, 0, bnrows*bncols);
  unsigned char* ptr_bg_mask = (unsigned char*)i_bp_mask;
  for (int r=border_y; r<border_y+bnrows*binning; r+=binning) 
  {
    assert(r < nrows-binning+1);

    conv_ptr[0] = (unsigned char*)(map + r*ncols + border_x);
    for (int i=1; i<binning; ++i)
      conv_ptr[i] = conv_ptr[i-1]+ncols;

    for (int c=border_x; c<border_x+bncols*binning; c+=binning, ++ptr_bmap, ++ptr_bg_mask)
    {
      assert (c < ncols-binning+1);

      sum_square = 0;
      last_col = min(ncols, c+binning);
      int num_pixels = 0;
      int num_zeros = 0;
      for (int conv_idx=0; conv_idx<binning; ++conv_idx)
      {
        for (int c2=c; c2<last_col; ++c2, ++conv_ptr[conv_idx])
        {
          unsigned char elem = *conv_ptr[conv_idx];
#ifdef USE_BINNING_WITH_CHECK
          if (elem != OUT_OF_RANGE_OR_STEREO_FAILURE && elem != UNIFORM_ZONE_OR_DISP_1)
#endif
          {
            sum_square += elem;
            ++num_pixels;
          }
          num_zeros += (elem == OUT_OF_RANGE_OR_STEREO_FAILURE);
        }
      }
#ifdef USE_BINNING_WITH_CHECK
      if (num_pixels == 0)
        *ptr_bmap = UNIFORM_ZONE_OR_DISP_1;
      else
#endif
        *ptr_bmap = sum_square/num_pixels;

      if (num_zeros >= binning)
        *ptr_bg_mask = 255;
    }
  }

  free(conv_ptr);

#ifdef SHOW_BINNED_IMAGE
  {
    IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
    IplImage* tmp_resized = cvCreateImage(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
    cvSetData(tmp, bmap, bncols);
    cvResize(tmp, tmp_resized, CV_INTER_NN);
    cvShowImage("binned", tmp);
    cvShowImage("res_binned", tmp_resized);
    //cvWaitKey(0);
    cvReleaseImageHeader(&tmp);
    cvReleaseImage(&tmp_resized);
  }
#endif
}


void _unbinning(unsigned char * const & bmap,
                const int bnrows, const int bncols,
                const int binning,
                const int border_x, const int border_y,
                unsigned char * const & map, 
                const int nrows, const int ncols)
{
  for (int r=border_y; r<border_y+bnrows*binning; r+=binning)
  {
    assert (r < nrows-binning+1);

    int offset_bmap = ((r-border_y)/binning)*bncols;
    int offset_map = r*ncols;
    for (int c=border_x; c<border_x+bncols*binning; c+=binning)
    {
      assert (c < ncols-binning+1);

      int index_bmap = offset_bmap+(c-border_x)/binning;
      for (int k=0; k<binning; ++k)
      {
        int offset_r = k*ncols;
        for (int i=0; i<binning; ++i)
          map[offset_map+offset_r+c+i] = bmap[index_bmap];
      }
    }
  }
}


void
peak_unbinning(
  tPeakProps* const & peaks,
  const int num_peaks, 
  const int binning, 
  const int border_x,
  const int border_y)
{
  for (int i=0; i<num_peaks; ++i)
  {
    peaks[i].x *= binning;
    peaks[i].x += border_x;
    peaks[i].y *= binning;
    peaks[i].y += border_y;
    peaks[i].a *= (binning*binning);
#ifdef USE_NEW_DETECTION2
    peaks[i].wx *= binning;
    peaks[i].wy *= binning;
#else
    peaks[i].w = (int)sqrt((float)peaks[i].a); // sqrtf maybe better
#endif
  }
}


//// convolve horizontally
//void _convH(const unsigned char * const & map, const int nrows, const int ncols, 
//            unsigned int* const & C)
//{
//  const unsigned char* ptr_map = map;
//  const unsigned int* ptr_k;
//  unsigned int* ptr_C = C;
//
//  int radius = KERNEL_X_DIM / 2;
//  for (int r=0; r<nrows; ++r)
//  {
//    // left part
//    ptr_map = map+r*ncols;
//    for (int c=0; c<radius; ++c, ++ptr_C, ++ptr_map)
//    {
//#ifdef USE_REPLICATE_IN_CONV
//      unsigned int sum__ = 0;
//#endif
//      *ptr_C = 0;
//      ptr_k = (unsigned int*)&(kernel_x[radius-c]);
//      for (int k=-c; k<=radius; ++k, ++ptr_k)
//      {
//#ifdef USE_REPLICATE_IN_CONV
//        unsigned int tmpval = *(ptr_map+k) * *ptr_k;
//        *ptr_C += tmpval;
//        if (k > c)
//          sum__ += tmpval;
//#else
//        *ptr_C += *(ptr_map+k) * *ptr_k;
//#endif
//        assert( (ptr_map+k) < map+nrows*ncols );
//      }
//#ifdef USE_REPLICATE_IN_CONV
//      *ptr_C = (*ptr_C+sum__)/NORM_FACT;
//#else
//      *ptr_C /= NORM_FACT;
//#endif
//    }
//
//    // central part
//    ptr_map = map+r*ncols+radius;
//    for (int c=radius; c<ncols-radius; ++c, ++ptr_C, ++ptr_map)
//    {
//      *ptr_C = 0;
//      ptr_k = (unsigned int*)kernel_x;
//      for (int k=-radius; k<=radius; ++k, ++ptr_k)
//      {
//        *ptr_C += *(ptr_map+k) * *ptr_k;
//        assert( (ptr_map+k) < map+nrows*ncols );
//      }
//      *ptr_C /= NORM_FACT;
//    }
//
//    // right part
//    ptr_map = map+r*ncols+ncols-radius;
//    for (int c=ncols-radius; c<ncols; ++c, ++ptr_C, ++ptr_map)
//    {
//#ifdef USE_REPLICATE_IN_CONV
//      unsigned int sum__ = 0;
//#endif
//      *ptr_C = 0;
//      ptr_k = (unsigned int*)kernel_x;
//      int last_k = ncols-1-c;
//      for (int k=-radius; k<=last_k; ++k, ++ptr_k)
//      {
//#ifdef USE_REPLICATE_IN_CONV
//        unsigned int tmpval = *(ptr_map+k) * *ptr_k;
//        *ptr_C += tmpval;
//        if (k < -last_k)
//          sum__+= tmpval;
//#else
//        *ptr_C += *(ptr_map+k) * *ptr_k;
//#endif
//        assert( (ptr_map+k) < map+nrows*ncols );
//      }
//#ifdef USE_REPLICATE_IN_CONV
//      *ptr_C = (*ptr_C+sum__)/NORM_FACT;
//#else
//      *ptr_C /= NORM_FACT;
//#endif
//    }
//
//    assert(ptr_map <= map+nrows*ncols);
//    assert(ptr_C <= C+nrows*ncols);
//  }
//}
//
//
//// convolve vertically
//void _convV(const unsigned int* const & C1, const int nrows, const int ncols, 
//            unsigned int* const & C2)
//{
//  const unsigned int* ptr_C1 = C1;
//  const unsigned int* tmp;
//  const unsigned int* ptr_k;
//  unsigned int* ptr_C2 = C2;
//
//  // upper part
//  int radius = KERNEL_Y_DIM/2;
//  for (int r=0; r<radius; ++r)
//  {
//    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2)
//    {
//#ifdef USE_REPLICATE_IN_CONV
//      unsigned int sum__ = 0;
//#endif
//      *ptr_C2 = 0; 
//      tmp = ptr_C1+ncols*(-r); 
//
//      assert(tmp >= C1);
//      assert(tmp < C1+nrows*ncols);
//
//      ptr_k = (unsigned int*)&(kernel_y[radius-r]);
//      for (int k=-r; k<=radius; ++k, ++ptr_k, tmp+=ncols)
//      {
//#ifdef USE_REPLICATE_IN_CONV
//        unsigned int tmpval = *tmp * *ptr_k;
//        *ptr_C2 += tmpval; 
//        if (k > r)
//          sum__ += tmpval;
//#else
//        *ptr_C2 += (*tmp) * (*ptr_k); 
//#endif
//        assert(tmp < C1+nrows*ncols);
//      }
//#ifdef USE_REPLICATE_IN_CONV
//      *ptr_C2 = (*ptr_C2+sum__)/NORM_FACT;
//#else
//      *ptr_C2 /= NORM_FACT;
//#endif
//    }
//  }
//
//  // central part
//  for (int r=radius; r<nrows-radius; ++r)
//  {
//    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2) 
//    {
//      *ptr_C2 = 0;
//      tmp = ptr_C1+ncols*(-radius);
//
//      assert(tmp >= C1);
//      assert(tmp < C1+nrows*ncols);
//
//      ptr_k = (unsigned int*)kernel_y;
//      for (int k=-radius; k<=radius; ++k, ++ptr_k, tmp+=ncols)
//      {
//        *ptr_C2 += (*tmp) * (*ptr_k); 
//        assert(tmp < C1+nrows*ncols);
//      }
//
//      *ptr_C2 /= NORM_FACT; 
//    }
//  }
//
//  // lower part
//  for (int r=nrows-radius; r<nrows; ++r)
//  {
//    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2)
//    {
//#ifdef USE_REPLICATE_IN_CONV
//      unsigned int sum__ = 0;
//#endif
//      *ptr_C2 = 0;
//      tmp = ptr_C1+ncols*(-radius);
//
//      assert(tmp >= C1);
//      assert(tmp < C1+nrows*ncols);
//
//      ptr_k = (unsigned int*)kernel_y;
//      int last_k = nrows-1-r;
//      for (int k=-radius; k<=last_k; ++k, ++ptr_k, tmp+=ncols)
//      {
//#ifdef USE_REPLICATE_IN_CONV
//        unsigned int tmpval = *tmp * *ptr_k;
//        *ptr_C2 += tmpval; 
//        if (k < -last_k)
//          sum__+= tmpval;
//#else
//        *ptr_C2 += (*tmp) * (*ptr_k); 
//#endif
//        assert(tmp < C1+nrows*ncols);
//      }
//#ifdef USE_REPLICATE_IN_CONV
//      *ptr_C2 = (*ptr_C2+sum__)/NORM_FACT;
//#else
//      *ptr_C2 /= NORM_FACT;
//#endif
//    }
//
//    assert(ptr_C1 <= C1+nrows*ncols);
//    assert(ptr_C2 <= C2+nrows*ncols);
//
//  }
//}


// convolve horizontally
void _convH(
  const unsigned char * const & map,
  const int nrows,
  const int ncols,
  unsigned int* const & C)
{
  const unsigned char* ptr_map;
  const unsigned int* ptr_k;
  unsigned int* ptr_C;

  int radius = KERNEL_X_DIM / 2;

#ifdef USE_REPLICATE_IN_CONV_H
  static int sum_weights = 0;
  if (sum_weights == 0)
  {
    for (int i=0; i<KERNEL_X_DIM; ++i)
      sum_weights += kernel_x[i];
  }
#endif

  for (int r=0; r<nrows; ++r)
  {
    // left part
    ptr_map = map+r*ncols;
    ptr_C = C+r*ncols;
    for (int c=0; c<radius; ++c, ++ptr_C, ++ptr_map)
    {
#ifdef USE_REPLICATE_IN_CONV_H
      unsigned int sum__ = 0;
#endif
      *ptr_C = 0;
      ptr_k = (unsigned int*)&(kernel_x[radius-c]);
      for (int k=-c; k<=radius; ++k, ++ptr_k)
      {
#ifdef USE_REPLICATE_IN_CONV_H
        unsigned int tmpval = *(ptr_map+k) * *ptr_k;
        *ptr_C += tmpval;
        sum__ += *ptr_k;
        //if (k > c)
        //  sum__ += tmpval;
#else
        *ptr_C += *(ptr_map+k) * *ptr_k;
#endif
        assert( (ptr_map+k) < map+nrows*ncols );
      }
#ifdef USE_REPLICATE_IN_CONV_H
      //*ptr_C = (*ptr_C+sum__)/NORM_FACT;
      *ptr_C = ((*ptr_C /sum__) * sum_weights)/ NORM_FACT;
#else
      *ptr_C /= NORM_FACT;
#endif
    }

    // central part
    ptr_map = map+r*ncols+radius;
    ptr_C = C+r*ncols+radius;
    for (int c=radius; c<ncols-radius; ++c, ++ptr_C, ++ptr_map)
    {
      *ptr_C = 0;
      ptr_k = (unsigned int*)kernel_x;
      for (int k=-radius; k<=radius; ++k, ++ptr_k)
      {
        *ptr_C += *(ptr_map+k) * *ptr_k;
        assert( (ptr_map+k) < map+nrows*ncols );
      }
      *ptr_C /= NORM_FACT;
    }

    // right part
    ptr_map = map+r*ncols+ncols-radius;
    ptr_C = C+r*ncols+ncols-radius;
    for (int c=ncols-radius; c<ncols; ++c, ++ptr_C, ++ptr_map)
    {
#ifdef USE_REPLICATE_IN_CONV_H
      unsigned int sum__ = 0;
#endif
      *ptr_C = 0;
      ptr_k = (unsigned int*)kernel_x;
      int last_k = ncols-1-c;
      for (int k=-radius; k<=last_k; ++k, ++ptr_k)
      {
#ifdef USE_REPLICATE_IN_CONV_H
        unsigned int tmpval = *(ptr_map+k) * *ptr_k;
        *ptr_C += tmpval;
        sum__ += *ptr_k;
        //if (k < -last_k)
        //  sum__+= tmpval;
#else
        *ptr_C += *(ptr_map+k) * *ptr_k;
#endif
        assert( (ptr_map+k) < map+nrows*ncols );
      }
#ifdef USE_REPLICATE_IN_CONV_H
      //*ptr_C = (*ptr_C+sum__)/NORM_FACT;
      *ptr_C = ((*ptr_C/sum__) * sum_weights) / NORM_FACT;
#else
      *ptr_C /= NORM_FACT;
#endif
    }

    assert(ptr_map <= map+nrows*ncols);
    assert(ptr_C <= C+nrows*ncols);
  }
}


// convolve vertically
void _convV(const unsigned int* const & C1, const int nrows, const int ncols, 
            unsigned int* const & C2)
{
  const unsigned int* ptr_C1 = C1;
  const unsigned int* tmp;
  const unsigned int* ptr_k;
  unsigned int* ptr_C2 = C2;

  int radius = KERNEL_Y_DIM/2;

#ifdef USE_REPLICATE_IN_CONV_V
  static int sum_weights = 0;
  if (sum_weights == 0)
  {
    for (int i=0; i<KERNEL_Y_DIM; ++i)
      sum_weights += kernel_y[i];
  }
#endif

  ptr_C1 = C1;
  ptr_C2 = C2;
  for (int r=0; r<radius; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2)
    {
#ifdef USE_REPLICATE_IN_CONV_V
      unsigned int sum__ = 0;
#endif
      *ptr_C2 = 0; 
      tmp = ptr_C1+ncols*(-r); 

      assert(tmp >= C1);
      assert(tmp < C1+nrows*ncols);

      ptr_k = (unsigned int*)&(kernel_y[radius-r]);
      for (int k=-r; k<=radius; ++k, ++ptr_k, tmp+=ncols)
      {
#ifdef USE_REPLICATE_IN_CONV_V
        unsigned int tmpval = *tmp * *ptr_k;
        *ptr_C2 += tmpval; 
        sum__ += *ptr_k;
        //if (k > r)
        //  sum__ += tmpval;
#else
        *ptr_C2 += (*tmp) * (*ptr_k); 
#endif
        assert(tmp < C1+nrows*ncols);
      }
#ifdef USE_REPLICATE_IN_CONV_V
      //*ptr_C2 = (*ptr_C2+sum__)/NORM_FACT;
      *ptr_C2 = ((*ptr_C2/sum__) * sum_weights) /NORM_FACT;
#else
      *ptr_C2 /= NORM_FACT;
#endif
    }
  }

  // central part
  ptr_C1 = C1+radius*ncols;
  ptr_C2 = C2+radius*ncols;
  for (int r=radius; r<nrows-radius; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2) 
    {
      *ptr_C2 = 0;
      tmp = ptr_C1+ncols*(-radius);

      assert(tmp >= C1);
      assert(tmp < C1+nrows*ncols);

      ptr_k = (unsigned int*)kernel_y;
      for (int k=-radius; k<=radius; ++k, ++ptr_k, tmp+=ncols)
      {
        *ptr_C2 += (*tmp) * (*ptr_k); 
        assert(tmp < C1+nrows*ncols);
      }

      *ptr_C2 /= NORM_FACT; 
    }
  }

  // lower part
  ptr_C1 = C1+(nrows-radius)*ncols;
  ptr_C2 = C2+(nrows-radius)*ncols;
  for (int r=nrows-radius; r<nrows; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_C1, ++ptr_C2)
    {
#ifdef USE_REPLICATE_IN_CONV_V
      unsigned int sum__ = 0;
#endif
      *ptr_C2 = 0;
      tmp = ptr_C1+ncols*(-radius);

      assert(tmp >= C1);
      assert(tmp < C1+nrows*ncols);

      ptr_k = (unsigned int*)kernel_y;
      int last_k = nrows-1-r;
      for (int k=-radius; k<=last_k; ++k, ++ptr_k, tmp+=ncols)
      {
#ifdef USE_REPLICATE_IN_CONV_V
        unsigned int tmpval = *tmp * *ptr_k;
        *ptr_C2 += tmpval; 
        sum__ += *ptr_k;
        //if (k < -last_k)
        //  sum__+= tmpval;
#else
        *ptr_C2 += (*tmp) * (*ptr_k); 
#endif
        assert(tmp < C1+nrows*ncols);
      }
#ifdef USE_REPLICATE_IN_CONV_V
      //*ptr_C2 = (*ptr_C2+sum__)/NORM_FACT;
      *ptr_C2 = ((*ptr_C2/sum__) * sum_weights) / NORM_FACT;
#else
      *ptr_C2 /= NORM_FACT;
#endif
    }

    assert(ptr_C1 <= C1+nrows*ncols);
    assert(ptr_C2 <= C2+nrows*ncols);

  }
}


void _peak_amplification(const unsigned char * const & bmap, const int & bnrows, const int & bncols,
                         unsigned int** C = NULL)
{
  static unsigned int C1[BINNED_DIM]; 
  static unsigned int C2[BINNED_DIM]; 

  assert(bnrows*bncols <= BINNED_DIM);

  _convH(bmap, bnrows, bncols, C1);
  _convV(C1, bnrows, bncols, C2);

  if (C != NULL)
    *C = C2;
}


void
_find_min_max(const unsigned int* const & C, const int nrows, const int ncols, 
              unsigned int & min_c, unsigned int & max_c)
{
  max_c = 0;
  min_c = UINT_MAX;
  for (int r=0; r<nrows; ++r)
  {
    int offset = r*ncols;
    for (int c=0; c<ncols; ++c)
    {
      unsigned int index = offset+c;
      if (C[index] > max_c)
        max_c = C[index];
      if (C[index] < min_c)
        min_c = C[index];
    }
  }
}


void
_normalize_ldg(unsigned int* const & C, 
               const int nrows, const int ncols,
               const unsigned int min_c, const unsigned int max_c,
               unsigned char * const & map)
{
  assert((max_c-min_c) != 0);
  for (int r=0; r<nrows; ++r)
  {
    int offset = r*ncols;
    for (int c=0; c<ncols; ++c)
    {
      int index = offset+c;
      C[index] = (255*(C[index]-min_c))/(max_c-min_c);
      map[index] = (unsigned char) C[index];
    }
  }
}


void _findMaxH(const unsigned int * const & map, const int nrows, const int ncols,
               const int sze, unsigned int * const & mapH)
{
  assert(sze > 0);
  int radius = sze/2;

  const unsigned int* ptr_map = map;
  unsigned int* ptr_mapH = mapH;

  for (int r=0; r<nrows; ++r)
  {
    for (int c=0; c<radius; ++c, ++ptr_map, ++ptr_mapH)
    {
      *ptr_mapH = *(ptr_map-c); // <- map(r, 0)
      for (int k=-c+1; k<=radius; ++k)
      {
        if (*(ptr_map+k) > *ptr_mapH)
          *ptr_mapH = *(ptr_map+k); // <- map(r, k)
      }
    }

    for (int c=radius; c<ncols-radius; ++c, ++ptr_map, ++ptr_mapH)
    {
      *ptr_mapH = *(ptr_map-radius);
      for (int k=-radius+1; k<=radius; ++k)
      {
        if ( *(ptr_map+k) > *ptr_mapH )
          *ptr_mapH = *(ptr_map+k);
      }
    }

    for (int c=ncols-radius; c<ncols; ++c, ++ptr_map, ++ptr_mapH)
    {
      *ptr_mapH = *(ptr_map-radius);
      for (int k=-radius+1; k<=(ncols-1-c); ++k)
      {
        if (*(ptr_map+k) > *ptr_mapH )
          *ptr_mapH = *(ptr_map+k);
      }
    }
  }
}


void _findMaxV(const unsigned int* const & map, const int nrows, const int ncols,
               const int sze, unsigned int* const & mapV)
{
  assert(sze > 0);
  int radius = sze/2;

  const unsigned int* ptr_map = map;
  const unsigned int* tmp;
  unsigned int* ptr_mapV = mapV;

  for (int r=0; r<radius; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_map, ++ptr_mapV)
    {
      *ptr_mapV = *(ptr_map-r*ncols); // <- map(0, c)
      tmp = ptr_map+(-r+1)*ncols; // tmp -> map(0+1, c)
      for (int k=-r+1; k<=radius; ++k)
      {
        if ( *tmp > *ptr_mapV )
          *ptr_mapV = *(tmp);
        tmp+=ncols; // tmp -> map(r+k, c)
      }
    }
  }

  for (int r=radius; r<nrows-radius; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_map, ++ptr_mapV)
    {
      tmp = ptr_map-radius*ncols; // tmp -> map(r-radius, c)
      *ptr_mapV = *tmp;
      tmp+=ncols;
      for (int k=-radius+1; k<=radius; ++k)
      {
        if ( *tmp > *ptr_mapV )
          *ptr_mapV = *(tmp);
        tmp+=ncols; // tmp -> map(r+k, c)
      }
    }
  }

  for (int r=nrows-radius; r<nrows; ++r)
  {
    for (int c=0; c<ncols; ++c, ++ptr_map, ++ptr_mapV)
    {
      tmp = ptr_map-radius*ncols; // tmp -> map(r-radius, c)
      *ptr_mapV = *tmp;
      tmp+=ncols; // tmp -> map(r-radius+1, c)
      for (int k=-radius+1; k<=(nrows-1-r); ++k)
      {
        if ( *tmp > *ptr_mapV )
          *ptr_mapV = *tmp;
        tmp+=ncols; // tmp -> map(r+k, c)
      }
    }
  }
}


bool
_no_maxima_around(const bool* const & selected_maxima, const int row, const int col, const int ncols, const int nrows, const int ray)
{
  bool no_maxima = true;
  int first_row = max(0,row-ray);
  int last_row = min(nrows-1,row+ray);
  int first_col = max(0,col-ray); 
  int last_col = min(ncols-1,col+ray);
  for (int r=first_row; r<=last_row; ++r)
  {
    const bool* sel_max_ptr = &selected_maxima[r*ncols+first_col];
    for (int c=first_col; c<=last_col && no_maxima; ++c, ++sel_max_ptr)
      no_maxima = no_maxima && !(*sel_max_ptr);
  }
  return no_maxima;
}


tPeakProps*
_peak_detection(const unsigned char* const & map, const unsigned int* const & C, 
                const int nrows, const int ncols,
                const float sum,
                int & num_peaks)
{
  static unsigned int local_maxima[BINNED_DIM];
  assert(nrows*ncols <= BINNED_DIM);
  {
    static unsigned int maxH[BINNED_DIM]; 
    int search_ray_x = max(1,KERNEL_X_DIM/4);
    int search_ray_y = max(1,KERNEL_Y_DIM/4);
    _findMaxH(C, nrows, ncols, 2*search_ray_x+1, maxH);
    _findMaxV(maxH, nrows, ncols, 2*search_ray_y+1, local_maxima);
  }

  unsigned char* map_ptr = (unsigned char*)map;
  unsigned int* C_ptr = (unsigned int*) C;
  unsigned int* local_maxima_ptr = (unsigned int*) local_maxima;

  static tPeakProps peaks[MAX_NUM_PEAKS];
  static bool selected_maxima[BINNED_DIM];
  memset(selected_maxima, 0, BINNED_DIM*sizeof(bool));
  bool* sel_max_ptr = selected_maxima;

  static const int coeff = (1024/NORM_FACT)*(1024/NORM_FACT);
  static const float coeff2 = th_fact*coeff; // minimizzo operazioni float precalcolando il piu' possibile
  const int threshold_min = (int)(sum*min_disp*coeff2);
  const int scaled_sum = (int)(sum*coeff);
  int peaks_idx = 0;
  for (int r=0; (r<nrows && peaks_idx<MAX_NUM_PEAKS); ++r)
  {
    for (int c=0; (c<ncols && peaks_idx<MAX_NUM_PEAKS); ++c, ++map_ptr, ++C_ptr, ++local_maxima_ptr, ++sel_max_ptr)
    {
      if (*map_ptr>=min_disp && *C_ptr>=threshold_min &&
         ((*C_ptr/scaled_sum) < *map_ptr-4 || *map_ptr >= min_disp+32))  // solo se la disparità è bassa (=> testa piccola => contenuta nel kernel) il centro della 
                                                                         // finestra deve essere massimo rispetto al vicinato (solo se la disparità è alta ha senso
                                                                         // avere zone uniformi nel kernel a causa di una testa molto grande); questo controllo evita
                                                                         // grandi quantità di picchi sullo sfondo che poi causa grosso carico CPU per PCN.
      {
        if (*C_ptr == *local_maxima_ptr)
        {
          if (_no_maxima_around(selected_maxima, r, c, ncols, nrows, MIN_PEAKS_DIST))
          {
            *sel_max_ptr = true;
            peaks[peaks_idx].x = c;
            peaks[peaks_idx].y = r;
            peaks[peaks_idx].z = *map_ptr;
            peaks[peaks_idx].a = -1;
#ifdef USE_NEW_DETECTION2
            peaks[peaks_idx].wx = -1;
            peaks[peaks_idx].wy = -1;
#else
            peaks[peaks_idx].w = -1;
#endif
            peaks_idx++;
          }
        }
      }
    }
  }
  num_peaks = peaks_idx;

#ifdef SHOW_PEAKS_IMAGE
  {
    IplImage* tmp = cvCreateImageHeader(cvSize(ncols, nrows), IPL_DEPTH_8U, 1);
    IplImage* tmp_rgb = cvCreateImage(cvSize(ncols, nrows), IPL_DEPTH_8U, 3);
    cvSetData(tmp, (void*)map, ncols);
    cvCvtColor(tmp,tmp_rgb, CV_GRAY2RGB);
    for (int i=0; i<num_peaks; ++i)
      cvCircle(tmp_rgb, cvPoint(peaks[i].x, peaks[i].y), 2, cvScalar(0, 0, 255));
    cvShowImage("peaks", tmp_rgb);
    cvReleaseImageHeader(&tmp);
    cvReleaseImage(&tmp_rgb);
  }
#endif

  return peaks;
}


void 
_init_prox_mat(unsigned int* const & prox_mat, 
               const tPeakProps* const & peaks, const int num_peaks)
{
  unsigned int* ptr_prox_mat = prox_mat;
  tPeakProps* ptr_peaks1 = (tPeakProps*) peaks;
  for (int i=0; i<num_peaks; ++i, ++ptr_peaks1)
  {
    ptr_prox_mat += (i+1);
    tPeakProps* ptr_peaks2 = (ptr_peaks1+1);
    for (int j=i+1; j<num_peaks; ++j, ++ptr_peaks2, ++ptr_prox_mat)
      *ptr_prox_mat = abs(ptr_peaks1->x-ptr_peaks2->x) + abs(ptr_peaks1->y-ptr_peaks2->y); //abs(peaks[i].x-peaks[j].x)+abs(peaks[i].y-peaks[j].y);
  }
}


void
_find_min_prox_mat(const unsigned int* const & prox_mat, const int num_peaks,
                   unsigned int & min_dist, int & min_dist_i, int & min_dist_j)
{
  int max_val = INT_MAX;
  min_dist = max_val;
  min_dist_i = -1;
  min_dist_j = -1;

  unsigned int* ptr_prox_mat = (unsigned int*) prox_mat;
  for (int i=0; i<num_peaks; ++i)
  {
    ptr_prox_mat += (i+1);
    for (int j=i+1; j<num_peaks; ++j, ++ptr_prox_mat)
      if (*ptr_prox_mat<min_dist)
      {
        min_dist = *ptr_prox_mat;
        min_dist_i = i;
        min_dist_j = j;
      }
  }
}


void
_peaks_clustering(tPeakProps* const & peaks, int & num_peaks, const unsigned int cutoff)
{
  static unsigned int prox_mat[MAX_NUM_PEAKS*MAX_NUM_PEAKS];
  static int clusters[MAX_NUM_PEAKS];
  static int clusters_num_elem[MAX_NUM_PEAKS];
  static unsigned int sum_x[MAX_NUM_PEAKS];
  static unsigned int sum_y[MAX_NUM_PEAKS];
  static unsigned int sum_z[MAX_NUM_PEAKS];

  _init_prox_mat(prox_mat, peaks, num_peaks);

  int max_val = INT_MAX;
  unsigned int min_dist = max_val;
  int min_dist_i = -1;
  int min_dist_j = -1;
  _find_min_prox_mat(prox_mat, num_peaks, min_dist, min_dist_i, min_dist_j);

  int num_clusters = 0;
  memset(clusters, 0, sizeof(int)*num_peaks);
  while (min_dist < cutoff)
  {
    int min_idx = min(min_dist_i, min_dist_j);
    int max_idx = max(min_dist_i, min_dist_j);
    if (clusters[min_idx] == 0 && clusters[max_idx] == 0)
    {
      num_clusters++;
      clusters[min_idx] = num_clusters;
      clusters[max_idx] = num_clusters;
    }
    else
    {
      if (clusters[min_idx] == 0)
        clusters[min_idx] = clusters[max_idx];
      else
        clusters[max_idx] = clusters[min_idx];
    }

    for (int i=0; i<=min_idx-1; ++i)
    {
      unsigned int offset = i*num_peaks;
      unsigned int min_val = min(prox_mat[offset+min_idx], prox_mat[offset+max_idx]);
      prox_mat[offset+min_idx] = min_val;
      prox_mat[offset+max_idx] = max_val;
    }
    for (int j=min_idx+1; j<=max_idx-1; ++j)
    {
      unsigned int offset1 = min_idx*num_peaks+j;
      unsigned int offset2 = j*num_peaks+max_idx;
      unsigned int min_val = min(prox_mat[offset1], prox_mat[offset2]);
      prox_mat[offset1] = min_val;
      prox_mat[offset2] = max_val;
    }
    for (int j=max_idx+1; j<num_peaks; ++j)
    {
      unsigned int offset1 = min_idx*num_peaks+j;
      unsigned int offset2 = max_idx*num_peaks+j;
      unsigned int min_val = min(prox_mat[offset1], prox_mat[offset2]);
      prox_mat[offset1] = min_val;
      prox_mat[offset2] = max_val;
    }
    prox_mat[min_idx*num_peaks+max_idx] = max_val;

    _find_min_prox_mat(prox_mat, num_peaks, min_dist, min_dist_i, min_dist_j);
  }

  for (int i=0; i<num_peaks; ++i)
  {
    if (clusters[i] == 0)
    {
      num_clusters++;
      clusters[i] = num_clusters;
    }
  }

  memset(sum_x, 0, sizeof(unsigned int)*num_clusters);
  memset(sum_y, 0, sizeof(unsigned int)*num_clusters);
  memset(sum_z, 0, sizeof(unsigned int)*num_clusters);
  memset(clusters_num_elem, 0, sizeof(int)*num_clusters);
  for (int i=0; i<num_peaks; ++i)
  {
    unsigned int idx = clusters[i]-1;
    sum_x[idx] += 1024*peaks[i].x;
    sum_y[idx] += 1024*peaks[i].y;
    sum_z[idx] += 1024*peaks[i].z;
    clusters_num_elem[idx]++;
  }

  num_peaks = num_clusters;
  for (int i=0; i<num_clusters; ++i)
  {
    assert(clusters_num_elem[i] > 0);
    peaks[i].x = (sum_x[i]/clusters_num_elem[i])/1024;
    peaks[i].y = (sum_y[i]/clusters_num_elem[i])/1024;
    peaks[i].z = (int)(sum_z[i]/clusters_num_elem[i])/1024; //no more processing so round can be done
  }
}


#ifdef USE_NEW_DETECTION2
void
_peaks_area_and_width_computation(tPeakProps* const & peaks, const int num_peaks, 
                                  const unsigned char* const & map, const int nrows, const int ncols)
{
  int radiusX = KERNEL_X_DIM/2;
  int radiusY = KERNEL_Y_DIM/2;
  //const int FACT = 2048;

  unsigned long maximum_base = (KERNEL_X_DIM*KERNEL_Y_DIM); //*FACT);
  for (int i=0; i<num_peaks; ++i)
  {
    tPeakProps* peak = &(peaks[i]);
    
    unsigned long sum = 0;
    unsigned long peak_base = 0;
    unsigned int peak_w = 0;
    unsigned int peak_h = 0;

    unsigned int local_peak_h[KERNEL_X_DIM];
    for (int c=0; c<KERNEL_X_DIM; ++c)
      local_peak_h[c]=0;

    int first_r = max(0,peak->y-radiusY);
    int last_r  = min(nrows-1,peak->y+radiusY);

    int first_c = max(0,peak->x-radiusX);
    int last_c  = min(ncols-1,peak->x+radiusX);

    for (int r=first_r; r<=last_r; ++r)
    {
      unsigned int local_peak_w = 0;
      for (int c=first_c; c<=last_c; ++c)
      {
        unsigned char elem = map[r*ncols+c];
        sum += elem;
        ++peak_base;
        if (elem >= peak->z/2)
        {
          local_peak_w++;
          local_peak_h[c-first_c]++;
        }
      }
      if (local_peak_w > peak_w)
        peak_w = local_peak_w;
    }

    for (int c=0; c<KERNEL_X_DIM; ++c)
      if (local_peak_h[c] > peak_h)
        peak_h = local_peak_h[c];

    assert(peak->z > 0);
    peak->a = (((maximum_base*sum)/peak_base)/peak->z); ///FACT;
    peak->wx = peak_w;
    peak->wy = peak_h;
  }
}

void
_peaks_pruning(tPeakProps* & peaks, int & num_peaks, const int min_area, const int min_w, const int min_h)
{
  static tPeakProps pruned_peaks[MAX_NUM_PEAKS];

  assert(min_area >= 16);
  assert(min_w >= 4);
  assert(min_h >= 4);

  int pruned_peaks_idx = 0;
  for (int i=0; i<num_peaks; ++i)
  {
    tPeakProps* peak = &(peaks[i]);
    float dim_ratio = (peak->wx/(float)peak->wy);
    
    int max_w_from_z = peak->z/FROM_DISP_TO_SHOULDER;    
    if (peak->a >= min_area &&
        peak->wx >= min_w && peak->wy >= min_h &&
        (peak->wx <= max_w_from_z && peak->wy <= max_w_from_z) &&
        dim_ratio >= 0.25f && dim_ratio <= 4.0f)
    {
      memcpy(&(pruned_peaks[pruned_peaks_idx]), peak, sizeof(tPeakProps));
      ++pruned_peaks_idx;
    }
  }

  /*{
    tPeakProps* peak = &(peaks[i]);
    float dim_ratio = (peak->wx/(float)peak->wy);
    
    int max_w_from_z = peak->z/FROM_DISP_TO_SHOULDER; 
    int min_w_from_z = max(min_w_orig, peak->z/FROM_DISP_TO_HEAD);
    
    const float ray = min_w_from_z/2.0f;
    int min_area_from_z = max(min_area, (int)(ray*ray*3.14f));

    if (peak->a >= min_area_from_z && 
        (peak->wx >= min_w_from_z || peak->wy >= min_w_from_z) && 
        (peak->wx >= min_w && peak->wy >= min_w) &&
        (peak->wx <= max_w_from_z && peak->wy <= max_w_from_z) &&
        dim_ratio >= 0.25f && dim_ratio <= 4.0f)
    {
      memcpy(&(pruned_peaks[pruned_peaks_idx]), peak, sizeof(tPeakProps));
      ++pruned_peaks_idx;
    }
  }*/

  num_peaks = pruned_peaks_idx;
  peaks = pruned_peaks;
}
#else
void
_peaks_area_and_width_computation(tPeakProps* const & peaks, const int num_peaks, 
                                  const unsigned char* const & map, const int nrows, const int ncols)
{
  int radiusX = KERNEL_X_DIM/2;
  int radiusY = KERNEL_Y_DIM/2;

  unsigned int maximum_base = (KERNEL_X_DIM*KERNEL_Y_DIM*1024);
  for (int i=0; i<num_peaks; ++i)
  {
    tPeakProps* peak = &(peaks[i]);
    unsigned int sum = 0;
    unsigned int peak_base = 0;
    for (int r=max(0,peak->y-radiusY); r<=min(nrows-1,peak->y+radiusY); ++r)
    {
      for (int c=max(0,peak->x-radiusX); c<=min(ncols-1,peak->x+radiusX); ++c)
      {
        sum += map[r*ncols+c];
        ++peak_base;
      }
    }
    peak->a = (((maximum_base/peak_base)*sum)/peak->z)/1024;
  }
}

void
_peaks_pruning(tPeakProps* & peaks, int & num_peaks, const int min_area)
{
  static tPeakProps pruned_peaks[MAX_NUM_PEAKS];

  assert(min_area > 9);

  //int radiusX = KERNEL_X_DIM/2;
  //int radiusY = KERNEL_Y_DIM/2;
  int pruned_peaks_idx = 0;
  for (int i=0; i<num_peaks; ++i)
  {
    tPeakProps* peak = &(peaks[i]);
    if (peak->a > min_area)
    {
      memcpy(&(pruned_peaks[pruned_peaks_idx]), peak, sizeof(tPeakProps));
      ++pruned_peaks_idx;
    }
  }

  num_peaks = pruned_peaks_idx;
  peaks = pruned_peaks;
}
#endif


float
peak_detection_init()
{
  static bool first_time = true;
  static float sum;
  if (first_time)
  {
    printf("peak_detection_init(): inizializzazione dati peak_detection.\n");

    assert(KERNEL_X_DIM%2 == 1);
    assert(KERNEL_Y_DIM%2 == 1);

    first_time = false;
    
    const float sigma = 4.246f;
    const float var = sigma*sigma;
    int offset_x = KERNEL_X_DIM/2;
    int offset_y = KERNEL_Y_DIM/2;

    sum = 0;
    for (int i=-offset_x; i<=offset_x; ++i)
      for (int j=-offset_y; j<=offset_y; ++j)
        sum += exp(-(i*i+j*j)/var);

    for (int i=-offset_x; i<=offset_x; ++i)
      kernel_x[i+offset_x] = (unsigned int) (1024*exp(-i*i/var));

    for (int i=-offset_y; i<=offset_y; ++i)
      kernel_y[i+offset_y] = (unsigned int) (1024*exp(-i*i/var));
  }

  return sum;
}


//tPeakProps*
//peak_detection(const unsigned char * const & map, const int nrows, const int ncols, int & num_peaks)
tPeakProps*
peak_detection(unsigned char * const & bmap, const int bnrows, const int bncols, int & num_peaks)
{
#ifdef SHOW_ORIG_IMAGE
  {
    IplImage* tmp = cvCreateImageHeader(cvSize(ncols, nrows), IPL_DEPTH_8U, 1);
    cvSetData(tmp, (unsigned char*)map, ncols);
    cvShowImage("orig", tmp);
    cvReleaseImageHeader(&tmp);
  }
#endif

  // close
  if (strel_sze_h > 0)
  {
    _imdilate(bmap, bnrows, bncols, strel_sze_h, strel_sze_v);
#ifdef SHOW_DILATED_IMAGE
    {
      IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
      IplImage* tmp_resized = cvCreateImage(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
      cvSetData(tmp, bmap, bncols);
      cvResize(tmp, tmp_resized, CV_INTER_NN);
      cvShowImage("dilated", tmp_resized);
      //cvShowImage("dilated", tmp);
      cvReleaseImageHeader(&tmp);
      cvReleaseImage(&tmp_resized);
    }
#endif

    _imerode(bmap, bnrows, bncols, strel_sze_h, strel_sze_v);
#ifdef SHOW_ERODED_IMAGE
    {
      IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
      IplImage* tmp_resized = cvCreateImage(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
      cvSetData(tmp, bmap, bncols);
      cvResize(tmp, tmp_resized, CV_INTER_NN);
      cvShowImage("erored", tmp_resized);
      //cvShowImage("erored", tmp);
      //cvWaitKey(0);
      cvReleaseImageHeader(&tmp);
      cvReleaseImage(&tmp_resized);
    }
#endif
  }

  // open
  if (strel_sze2_h > 0)
  {
    _imerode(bmap, bnrows, bncols, strel_sze2_h, strel_sze2_v);
#ifdef SHOW_ERODED_IMAGE
    {
      IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
      IplImage* tmp_resized = cvCreateImage(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
      cvSetData(tmp, bmap, bncols);
      cvResize(tmp, tmp_resized, CV_INTER_NN);
      cvShowImage("erored2", tmp_resized);
      //cvShowImage("erored2", tmp);
      //cvWaitKey(0);
      cvReleaseImageHeader(&tmp);
      cvReleaseImage(&tmp_resized);
    }
#endif

    _imdilate(bmap, bnrows, bncols, strel_sze2_h, strel_sze2_v);
#ifdef SHOW_DILATED_IMAGE
    {
      IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
      IplImage* tmp_resized = cvCreateImage(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
      cvSetData(tmp, bmap, bncols);
      cvResize(tmp, tmp_resized, 0);
      cvShowImage("dilated2", tmp_resized);
      cvReleaseImageHeader(&tmp);
      cvReleaseImage(&tmp_resized);
    }
#endif
  }

  // kernel and threshold initialization
  float sum = peak_detection_init();

  // amplification
  unsigned int* C;
  _peak_amplification(bmap, bnrows, bncols, &C);

  // detection
  tPeakProps* peaks = _peak_detection(bmap, C, bnrows, bncols, sum, num_peaks);

  // clustering
  _peaks_clustering(peaks, num_peaks, person_head_width/binning);

  // compute other decriptor parameters
  _peaks_area_and_width_computation(peaks, num_peaks, bmap, bnrows, bncols);

  // prune spurius peaks
#ifdef USE_PEAKS_PRUNING
  if (min_area > 0)
#ifdef USE_NEW_DETECTION2
    _peaks_pruning(peaks, num_peaks, min_area, min_w, min_h);
#else
    _peaks_pruning(peaks, num_peaks, min_area);
#endif
#endif

#ifdef SHOW_CLUSTERED_PEAKS_IMAGE
  {
    IplImage* tmp = cvCreateImageHeader(cvSize(bncols, bnrows), IPL_DEPTH_8U, 1);
    IplImage* tmp_rgb = cvCreateImage(cvSize(bncols, bnrows), IPL_DEPTH_8U, 3);
    cvSetData(tmp, bmap, bncols);
    cvCvtColor(tmp, tmp_rgb, CV_GRAY2RGB);
    for (int i=0; i<num_peaks; ++i)
      cvCircle(tmp_rgb, cvPoint(peaks[i].x, peaks[i].y), 2, cvScalar(0, 255, 0));
    cvShowImage("clustered_peaks", tmp_rgb);
    cvReleaseImageHeader(&tmp);
    cvReleaseImage(&tmp_rgb);
  }
#endif



#if !defined(PERFORMANCE_TEST) && defined SHOW_UNBINNED_IMAGE
    unsigned int min_c, max_c;
  _find_min_max(C, bnrows, bncols, min_c, max_c);
  max_c += max_c/15;
  if ((max_c-min_c) != 0)
    _normalize_ldg(C, bnrows, bncols, min_c, max_c, bmap);
  //printf("min_c %d, max_c %d\n", min_c, max_c);

  int dim = binning*bncols*binning*bnrows;
  unsigned char* map2 = (unsigned char*) malloc (dim);
  memset(map2, 0, dim);
  _unbinning(bmap, bnrows, bncols, binning, 0, 0, map2, binning*bnrows, binning*bncols);
  IplImage* tmp = cvCreateImageHeader(cvSize(binning*bncols, binning*bnrows), IPL_DEPTH_8U, 1);
  cvSetData(tmp, map2, binning*bncols);
  cvShowImage("unbinned", tmp);
  cvReleaseImageHeader(&tmp);

  free(map2);
#endif

  return peaks;
}


//#define NORMALIZE_CONV
//// convolve horizontally
//void _convH(unsigned char * const & map, const int nrows, const int ncols, 
//            unsigned int* const & C)
//{
//#ifdef NORMALIZE_CONV
//  unsigned int sum_kx = 0;
//  for (int i=0; i<KERNEL_X_DIM; ++i)
//    sum_kx += kernel_x[i];
//#endif
//
//  int radius = KERNEL_X_DIM / 2;
//  for (int r=0; r<nrows; ++r)
//  {
//    int offset = r*ncols;
//    for (int c=0; c<ncols; ++c)
//    {
//#ifdef NORMALIZE_CONV
//      int sum_kx_adjusted = sum_kx;
//#endif
//      int index = offset+c;
//      C[index] = 0;
//      for (int k=-radius; k<=radius; ++k)
//      {
//        int c2 = c+k;
//        if (c2>=0 && c2<ncols) {
//          int kernel_idx = k+radius;
//          C[index] += map[index+k]*kernel_x[kernel_idx];
//        }
//#ifdef NORMALIZE_CONV
//        else {
//          int kernel_idx = k+radius;
//          sum_kx_adjusted -= kernel_x[kernel_idx];
//        }
//#endif
//      }
//#ifdef NORMALIZE_CONV
//      C[index] = C[index]/sum_kx_adjusted;
//#else
//      C[index] /= 1024;
//#endif
//    }
//  }
//}


//// convolve vertically
//void _convV(unsigned int* const & C1, const int nrows, const int ncols, 
//            unsigned int* const & C2)
//{
//#ifdef NORMALIZE_CONV
//  unsigned int sum_ky = 0;
//  for (int i=0; i<KERNEL_Y_DIM; ++i)
//    sum_ky += kernel_y[i];
//#endif
//
//  int radius = KERNEL_Y_DIM/2;
//  for (int r=0; r<nrows; ++r)
//  {
//    int offset = r*ncols;
//    for (int c=0; c<ncols; ++c)
//    {
//      int index = offset+c;
//#ifdef NORMALIZE_CONV
//    int sum_ky_adjusted = sum_ky;
//#endif
//      C2[index] = 0;
//      for (int k=-radius; k<=radius; ++k)
//      {
//        int kernel_idx = k+radius;
//        int r2 = r+k;
//        if (r2>=0 && r2<nrows)
//        {
//          int index2 = index+ncols*k;
//          C2[index] += C1[index2]*kernel_y[kernel_idx];
//        }
//#ifdef NORMALIZE_CONV
//        else
//          sum_ky_adjusted -= kernel_y[kernel_idx];
//#endif
//      }
//#ifdef NORMALIZE_CONV
//      C2[index] = C2[index]/sum_ky_adjusted;
//#else
//      C2[index] /= 1024;
//#endif
//    }
//  }
//}

#endif
