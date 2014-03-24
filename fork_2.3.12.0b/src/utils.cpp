#include "utils.h"

void colorizeDepth( const IplImage* const & gray, IplImage* const & rgb)
{
  double maxDisp= 255;
  float S=0.8f;
  float V=1.f;

  for (int y = 0; y < gray->height; ++y)
  {
    unsigned char* gray_ptr = (unsigned char*) &(gray->imageData[y*gray->widthStep]);
    unsigned char* rgb_ptr = (unsigned char*) &(rgb->imageData[y*rgb->widthStep]);
    for (int x = 0; x < gray->width; ++x, ++gray_ptr, rgb_ptr+=3)
    {
      uchar d = *gray_ptr;
      unsigned int H = 255 - (((uchar)maxDisp - d) * 255)/ (uchar)maxDisp;    
      unsigned int hi = (H/60) % 6;

      float f = H/60.f - H/60;
      float p = V * (1 - S);
      float q = V * (1 - f * S);
      float t = V * (1 - (1 - f) * S);

      CvPoint3D32f res;

      if( hi == 0 ) //R = V,  G = t,  B = p
        res = cvPoint3D32f( p, t, V );
      if( hi == 1 ) // R = q, G = V,  B = p
        res = cvPoint3D32f( p, V, q );
      if( hi == 2 ) // R = p, G = V,  B = t
        res = cvPoint3D32f( t, V, p );
      if( hi == 3 ) // R = p, G = q,  B = V
        res = cvPoint3D32f( V, q, p );
      if( hi == 4 ) // R = t, G = p,  B = V
        res = cvPoint3D32f( V, p, t );
      if( hi == 5 ) // R = V, G = p,  B = q
        res = cvPoint3D32f( q, p, V );

      uchar b = (uchar)(std::max(0.f, std::min (res.x, 1.f)) * 255.f);
      uchar g = (uchar)(std::max(0.f, std::min (res.y, 1.f)) * 255.f);
      uchar r = (uchar)(std::max(0.f, std::min (res.z, 1.f)) * 255.f);

      rgb_ptr[0] = b;
      rgb_ptr[1] = g;
      rgb_ptr[2] = r;
    }
  }
}