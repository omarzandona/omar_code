#include "morphology.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>


#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */


bool _first_gt_second(const unsigned char & first, const unsigned char & second)
{
  return (first > second);
}


bool _first_lt_second(const unsigned char & first, const unsigned char & second)
{
  return (first < second);
}


typedef bool (* func_test) (const unsigned char &, const unsigned char &);


// convolve horizontally
void _immorphH(const unsigned char * const & map, const int nrows, const int ncols,
               const int sze_orig, unsigned char * const & mapH)
{
  assert (map != mapH);
  const func_test _test_passed = (sze_orig > 0) ? _first_gt_second : _first_lt_second;
  int sze = abs(sze_orig);
  int radius = sze/2;

  assert(sze >= 1);
  if (sze == 1)
    memcpy(mapH, map, ncols*nrows*sizeof(unsigned char));
  else
  {
    const unsigned char* ptr_map = map;
    unsigned char* ptr_mapH = mapH;

    for (int r=0; r<nrows; ++r)
    {
      ptr_map = map+r*ncols;

      for (int c=0; c<ncols; ++c, ++ptr_map, ++ptr_mapH)
      {
        int min_c_radius = min(c, radius);

        *ptr_mapH = *(ptr_map - min_c_radius);
        assert((ptr_map - min_c_radius) >= map);
        
        for (int k = 1-min_c_radius; k <= min(ncols-1-c, radius); ++k)
        {
          if (_test_passed(*(ptr_map+k), *ptr_mapH))
            *ptr_mapH = *(ptr_map+k);
          assert( (ptr_map+k) < (map + nrows*ncols) );
        }
      }

      assert( ptr_map <= map+nrows*ncols );
      assert( ptr_mapH <= mapH+nrows*ncols );
    }
  }
}


// convolve vertically
void _immorphV(const unsigned char * const & map, const int nrows, const int ncols,
               const int sze_orig, unsigned char * const & mapV)
{
  assert (map != mapV);
  const func_test _test_passed = (sze_orig > 0) ? _first_gt_second : _first_lt_second;
  int sze = abs(sze_orig);
  int radius = sze/2;

  assert(sze >= 1);
  if (sze == 1)
    memcpy(mapV, map, ncols*nrows*sizeof(unsigned char));
  else
  {
    const unsigned char* ptr_map = map;
    const unsigned char* tmp;
    unsigned char* ptr_mapV = mapV;

    for (int r=0; r<nrows; ++r)
    {
      for (int c=0; c<ncols; ++c, ++ptr_map, ++ptr_mapV)
      {
        int min_r_radius = min(r, radius);

        tmp = ptr_map-min_r_radius*ncols;
        *ptr_mapV = *tmp;
        tmp += ncols;

        for (int k = 1-min_r_radius; k <= min(nrows-1-r, radius); ++k)
        {
          if (_test_passed(*tmp, *ptr_mapV))
            *ptr_mapV = *(tmp);
          assert(tmp < map+nrows*ncols);
          tmp += ncols; // tmp -> map(r+k, c)
        }
      }
    }
  }
}


void _immorph(unsigned char * const & map, 
              const int nrows, const int ncols,
              int sze_h, int sze_v)
{
  assert ( (sze_h%2) != 0);
  assert(sze_h != 0);

  assert ( (sze_v == 0) || ((sze_v%2) != 0) );
  assert(sze_h != 0);

  if (sze_v == 0)
    sze_v = sze_h;

  static unsigned char tmp[NN]; // support data statically allocated

  _immorphH(map, nrows, ncols, sze_h, tmp);
  _immorphV(tmp, nrows, ncols, sze_v, map);
}


void _imdilate(unsigned char * const & map, 
               const int nrows, const int ncols,
               int sze_h, int sze_v)
{
  assert(sze_h > 0);
  assert(sze_v >= 0);
  _immorph(map, nrows, ncols, sze_h, sze_v);
}


void _imerode(unsigned char * const & map, 
              const int nrows, const int ncols,
              int sze_h, int sze_v)
{
  assert(sze_h > 0);
  assert(sze_v >= 0);
  _immorph(map, nrows, ncols, -sze_h, -sze_v);
}


