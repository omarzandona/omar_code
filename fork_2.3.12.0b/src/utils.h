#ifndef UTILS_H_
#define UTILS_H_

#include "directives.h"

#ifdef OPENCV_1
#  include <cv.h>
#  include <cxcore.h>
#  include <highgui.h>
#else
#  include "opencv2/highgui/highgui.hpp"
#  include "opencv2/features2d/features2d.hpp"
#  include "opencv2/contrib/contrib.hpp"
#  include "opencv2/legacy/compat.hpp"
#  include "opencv2/imgproc/imgproc.hpp"
#  include "opencv2/imgproc/imgproc_c.h"
#  include "opencv2/imgproc/types_c.h"
#  include "opencv2/calib3d/calib3d.hpp"
#endif

void colorizeDepth( const IplImage* const & gray, IplImage* const & rgb);

#endif