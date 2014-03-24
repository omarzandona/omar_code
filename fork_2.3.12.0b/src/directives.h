#ifndef __DIRECTIVES__
#define __DIRECTIVES__

#ifndef PCN_VERSION
//#define PCN_VERSION // uncommented only when used in the imgserver (notice that makefile now creates PCN_VERSION from command line using the -D option)
#endif

#define SEQ_AT_54FPS // commented with old sequences acquired at 27fps

#ifndef PCN_VERSION
//#  define CHANGE_PATH
#  ifndef BATCH_TEST  // if one define this flag as a compiler option we can avoid redefinition and warnign messages
//#  define BATCH_TEST  // follows directives used only when you want test all video of one daset7
#  endif
#endif

#define USE_NEW_TRACKING
#ifdef USE_NEW_TRACKING
#define USE_SOLVE_CONFLICTS  // gestisci il conflitto tra inhi e inlo su uno stesso blob (vince costo minimo o piu' vicino)
#define USE_CONSISTENCY_CHECK  // gestisce conflitti tra blob nello storico (che possono esserci a causa delle vite)
#define USE_HANDLE_OUT_OF_RANGE  // abilita la gestione dell'out-of-range
#  ifdef USE_HANDLE_OUT_OF_RANGE
#  define USE_BINNING_IN_BLACK_PIXEL_COUNTING  // il conteggio dei pixel neri non viene fatto scandendo tutti i pixel ma a salti di 2 o 3 a seconda della dimensione del blob virtuale
#  define CHECK_FALSE_COUNTS  // evita più di TOT conteggi in TOT secondi (per tenere sotto controllo eventuale rumore non gestito)
#  define USE_STATIC_BLOB_CHECK  // usata per controllare la staticità del blob
#    ifndef BATCH_TEST
//#    define COPY_VIRTUAL_BLOB  // usata per scopi di debug e visualizzazione (da disabilitare quando funziona normalmente sul PCN)
#    endif
#  endif
#  ifdef PCN_VERSION
#  define DISABLE_BACKGROUND_CHECK 
#  else
#  define DISABLE_BACKGROUND_CHECK // per NUOVE_SEQUENZE 3-4-8 (vedi main.c)
#  endif
#endif

#define USE_NEW_DETECTION
#ifdef USE_NEW_DETECTION
#define USE_NEW_DETECTION2
//#define COMPUTE_FEET_COORDS  // to use feet coordinates in the cost computation and in other parts
#define USE_BINNING_WITH_CHECK  // in order to try filtering out stereo failures during binning
#endif

// follows define that are specific for emulator
#ifndef PCN_VERSION  // follows directives used only by the offline version
#define USE_RAW_DATA  // this flag is automatically forced in those experiments where digital input are embedded in RAW_DATA
//#define _DEBUG 
#define READ_INPUT  // this flag is automatically undefined in those experiments where digital input are embedded in RAW_DATA
//#define SUBTRACT_BG
//#define PERFORMANCE_TEST
#  ifndef PERFORMANCE_TEST
//#  define LOAD_PARAMS
//#  define SAVE_RESULT
#  ifndef BATCH_TEST
#  define SHOW_RESULT
#  endif
#  ifdef USE_NEW_TRACKING
//#    define VERBOSE
#  endif
#  endif
#endif

// follows define that are specific for the PCN
#ifdef PCN_VERSION
//#define eVS_TIME_EVAL // to have an estimate of the processing time in /tmp/time_file.txt
//#define FRAME_RATE_COMPUTATION // to compute in the fps.txt file the processed frame rate
#endif

#if (defined(PCN_VERSION) || defined(READ_INPUT)) && defined(USE_NEW_DETECTION2)
//#define USE_NEW_STRATEGIES
#endif

#ifdef SAVE_RESULT
#define JPEG
#endif

#ifdef SHOW_RESULT
//#define OPENCV_1
#  ifdef OPENCV_1
#    include <cv.h>
#    include <cxcore.h>
#    include <highgui.h>
#  else
#    include "opencv2/highgui/highgui.hpp"
#    include "opencv2/features2d/features2d.hpp"
#    include "opencv2/contrib/contrib.hpp"
#    include "opencv2/legacy/compat.hpp"
#    include "opencv2/imgproc/imgproc.hpp"
#    include "opencv2/imgproc/imgproc_c.h"
#    include "opencv2/imgproc/types_c.h"
#    include "opencv2/calib3d/calib3d.hpp"
#  endif
#endif

#endif
