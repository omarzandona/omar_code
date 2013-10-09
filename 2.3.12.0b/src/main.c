#include "directives.h"
#ifndef BATCH_TEST

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "imgserver.h"
#include "blob_detection.h"

#ifdef USE_NEW_TRACKING
#include "blob_tracking.h"
#else
#include "peopletrack.h"
#endif

#include "default_parms.h"
#include "peopledetection.h"
#include "utils.h"

#ifdef USE_RAW_DATA
#include "RawData.h"
#endif

#ifdef USE_HANDLE_OUT_OF_RANGE
#include "OutOfRangeManager.h"
#endif

//#define OPENCV_1
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

#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */

const int LR_BORDER = 340;  // 500 con zoom_fact = 1, 420 con zoom_fact = 2, 340 con zoom_fact = 3
const int MSG_BORDER = 510;
const int line_h = (MSG_BORDER-10)/20;
const int img_border = 20;
const int zoom_fact = 3;
CvSize img_shown_sz = cvSize(zoom_fact*NX, zoom_fact*NY);

extern unsigned long people_count_input;
extern unsigned long people_count_output;
extern void record_counters(unsigned long peoplein, unsigned long peopleout);

//#ifdef LOAD_PARAMS
//#define PM_FILENAME   "bologna_parameters.txt"
//#endif

#define FILE_NAME "pcn1001_video"


//#define ARABIA_SAUDITA
#ifdef ARABIA_SAUDITA
#define STR_NUM "5"
#endif

#define ROMA
//#define ROMA_27_FPS
#define ROMA_54_FPS

#ifdef ROMA_27_FPS
#define PATH_SEQ "sequenze_27fps"
#endif

#ifdef ROMA_54_FPS
#define PATH_SEQ "sequenze"
#endif



//#define PORTA_ANTERIORE  //Da a 1-3 per deq 27fps, 1-3 per seq 54fps
#define PORTA_CENTRALE  //Da a 1-3 per deq 27fps, 1-5 per seq 54fps
//#define PORTA_POSTERIORE  //Da a 1-3 per deq 27fps, 1-5 per seq 54fps

#ifdef PORTA_ANTERIORE
#define PATH_PORTA "porta_anteriore"
#endif
#ifdef PORTA_CENTRALE
#define PATH_PORTA "porta_centrale"
#endif
#ifdef PORTA_POSTERIORE
#define PATH_PORTA "porta_posteriore"
#endif

#define STR_NUM "3"



// variabili per la gestione del segnale porta aperta-chiusa
extern bool mem_door; 
extern bool ev_door_close;
extern bool ev_door_open;
extern bool ev_door_open_rec;
extern unsigned char frame_fermo;
extern unsigned char frame_cnt_door;
//extern tDoorStatus door_status;

// variabli che definiscono la no-tracking zone
extern unsigned char limitSx; 
extern unsigned char limitDx;
extern unsigned char limitSx_riga_start;
extern unsigned char limitDx_riga_start;
extern unsigned char limitSx_riga_end;
extern unsigned char limitDx_riga_end;
extern unsigned char limit_line_Up;
extern unsigned char limit_line_Down;

// altre variabili di setup
unsigned short soglia_porta_main;
extern int num_pers;
extern int xt;
extern int inst_height;
extern unsigned char door_size;
unsigned char people_dir = 1;  // questa è definita in imgserver.cpp e passato a detectAndTrack()

// necessario per alcune sequenze per non partire da inizio sequenza ma piu' avanti
int first_frame = 1;

// variabile usata per le nuove strategie che nel imgserver.cpp equivale a verificare se la input_function0 o la input_function1 e' enable_function()
// quindi nella SetDefaultValues() delle sequenze in cui vi è il segnale porta da usare, questa flag deve essere posta a true
#ifdef USE_NEW_STRATEGIES
bool is_door_signal_managed=false;
#endif


void
_old_sequence_check()
{
#ifdef SEQ_AT_54FPS
  printf("SEQ_AT_54FPS must be NOT defined in directives.h!\n");
  printf("Premi enter per chiudere...");
  getchar();
  exit(-1);
#endif
#if defined(USE_HANDLE_OUT_OF_RANGE) && !defined(DISABLE_BACKGROUND_CHECK)
  printf("DISABLE_BACKGROUND_CHECK must be defined in directives.h!\n");
  printf("Premi enter per chiudere...");
  getchar();
  exit(-1);
#endif
}


void
_new_sequence_check(bool is_optical_image_embedded = false)
{
  if (is_optical_image_embedded)
  {
#ifdef SEQ_AT_54FPS
    printf("SEQ_AT_54FPS must NOT be defined in directives.h!\n");
    printf("Premi enter per chiudere...");
    getchar();
    exit(-1);
#endif
  }
  else
  {
#ifndef SEQ_AT_54FPS
    printf("SEQ_AT_54FPS must be defined in directives.h!\n");
    printf("Premi enter per chiudere...");
    getchar();
    exit(-1);
#endif
  }

#ifndef USE_NEW_TRACKING
  printf("USE_NEW_TRACKING must be defined in directives.h!\n");
  printf("Premi enter per chiudere...");
  getchar();
  exit(-1);
#endif

#ifndef USE_HANDLE_OUT_OF_RANGE
  printf("USE_HANDLE_OUT_OF_RANGE must be defined in directives.h!\n");
  printf("Premi enter per chiudere...");
  getchar();
  exit(-1);
#endif
}

#ifdef ROMA
//#undef READ_INPUT
#ifdef CHANGE_PATH
//#define PATH "C:\\Documents and Settings\\zandonà\\Desktop\\blob_detection\\nuovo_setup_eVS"
#else
#define PATH "C:\\Documents and Settings\\eVS\\Documenti\\PCN\\Problemi\\Roma\\"PATH_SEQ"\\"PATH_PORTA
#endif
#ifndef USE_RAW_DATA
#define USE_RAW_DATA
#include "RawData.h"
#endif
const int img_presence_flag = RD_IMG_BEFORE_INPUT;
#ifdef SUBTRACT_BG
#define BACKGROUND_FN PATH"\\"FILE_NAME""STR_NUM".bkg"
#endif
#define RAW_FN				PATH"\\"FILE_NAME""STR_NUM".raw"
#ifdef SAVE_RESULT
#  ifdef USE_NEW_DETECTION
#    ifdef USE_NEW_TRACKING
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_newd_newt"
#    else
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_newd_oldt"
#    endif
#  else
#    ifdef USE_NEW_TRACKING
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_oldd_newt"
#    else
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_oldd_oldt"
#    endif
#  endif
#endif
void SetDefaultValues()
{
#ifdef ROMA_54_FPS
    _new_sequence_check();
#else
    _new_sequence_check(true);
#endif
#ifdef PORTA_ANTERIORE
  limitSx=110; 
  limitDx=150;
  limitSx_riga_start=0;
  limitDx_riga_start=0;
  limitSx_riga_end=119;
  limitDx_riga_end=119;
  limit_line_Up=9;
  limit_line_Down=110;
  soglia_porta_main=72;
  inst_height=200;
  door_size = 60;
  people_dir = 0;
  #ifdef USE_HANDLE_OUT_OF_RANGE
  OutOfRangeManager::getInstance().SetEnableStateOutOfRange(false); // Gestione out-of-range
  #endif
#endif
#ifdef PORTA_CENTRALE
  limitSx=14; 
  limitDx=145;
  limitSx_riga_start=0;
  limitDx_riga_start=0;
  limitSx_riga_end=119;
  limitDx_riga_end=119;
  limit_line_Up=9;
  limit_line_Down=110;
  soglia_porta_main= 68;
  inst_height= 200;
  door_size = 120;
  people_dir = 0;
  #ifdef USE_HANDLE_OUT_OF_RANGE
  OutOfRangeManager::getInstance().SetEnableStateOutOfRange(false); // Gestione out-of-range
  #endif
#endif
#ifdef PORTA_POSTERIORE
  limitSx=14; 
  limitDx=145;
  limitSx_riga_start=0;
  limitDx_riga_start=0;
  limitSx_riga_end=119;
  limitDx_riga_end=119;
  limit_line_Up=9;
  limit_line_Down=110;
  soglia_porta_main= 68;
  inst_height= 200;
  door_size = 120;
  people_dir = 0;
  #ifdef USE_HANDLE_OUT_OF_RANGE
  OutOfRangeManager::getInstance().SetEnableStateOutOfRange(true); // Gestione out-of-range
  #endif
#endif

}
#endif




#ifdef ARABIA_SAUDITA
#undef READ_INPUT
#ifdef CHANGE_PATH
#define PATH "C:\\Documents and Settings\\zandonà\\Desktop\\blob_detection\\nuovo_setup_eVS"
#else
#define PATH "C:\\Documents and Settings\\eVS\\Documenti\\PCN\\Problemi\\Arabia_Saudita\\sequenze"
#endif
#ifndef USE_RAW_DATA
#define USE_RAW_DATA
#include "RawData.h"
#endif
const int img_presence_flag = RD_IMG_AFTER_INPUT;
#ifdef SUBTRACT_BG
#define BACKGROUND_FN PATH"\\"FILE_NAME""STR_NUM".bkg"
#endif
#define RAW_FN				PATH"\\"FILE_NAME""STR_NUM".raw"
#ifdef SAVE_RESULT
#  ifdef USE_NEW_DETECTION
#    ifdef USE_NEW_TRACKING
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_newd_newt"
#    else
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_newd_oldt"
#    endif
#  else
#    ifdef USE_NEW_TRACKING
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_oldd_newt"
#    else
#    define SAVE_PREFIX   PATH"\\"FILE_NAME""STR_NUM"_oldd_oldt"
#    endif
#  endif
#endif
void SetDefaultValues()
{
  _new_sequence_check();
  limitSx=15; 
  limitDx=144;
  limitSx_riga_start=0;
  limitDx_riga_start=0;
  limitSx_riga_end=119;
  limitDx_riga_end=119;
  limit_line_Up=5;
  limit_line_Down=105;
  soglia_porta_main=45;
  inst_height=216;
  door_size = 130;
  people_dir = 1;

#ifdef USE_HANDLE_OUT_OF_RANGE
  OutOfRangeManager::getInstance().SetEnableStateOutOfRange(true); // Gestione out-of-range
  if (strcmp(STR_NUM, "1") == 0)
    first_frame = 1900;
#endif
}
#endif
//! Valori di default dei parametri relativi alle porte seriali, no tracking zone e motion detection (vedi \ref tabella_parms)
unsigned short default_values[256] = {
    OUTTIME0,
    OUTTIME1,
    INPUT0,  
    INPUT1,  
    SLED,
    DIR,
    THRESHOLD,
    SERIAL_ID,
    0, //SERIAL_BR,
    0, //SERIAL_DB,
    SERIAL_PR,
    SERIAL_SB,
    DETECT_AREA,
    OFF,
    INST_HEIGHT,
    SERIAL_MS,
    SERIAL_SID,
    0, //SERIAL_SBR,
    0, //SERIAL_SDB,
    SERIAL_SPR,
    SERIAL_SSB,
    TIMEBKG,
    STATIC_TH,
    SLAVE_ID,
    SX_DX,
    INST_DIST,
    WG_CHECK,
    SYS_NUMBER,
    SYS_NUMBER_INDEX,
    SXLIMIT,
    DXLIMIT,
    SXLIMIT_RIGA_START,
    DXLIMIT_RIGA_START,
    SXLIMIT_RIGA_END,
    DXLIMIT_RIGA_END,
    OFF,
    DIAGNOSTIC_EN,
    MOVE_DET_COL0,
    MOVE_DET_COL1,
    MOVE_DET_ROW0,
    MOVE_DET_ROW1,
    MOVE_DET_ALFAREG,
    0, //MOVE_DET_THR,
    MOVE_DET_EN,
    DOOR_STAIRS_EN,
    UP_LINE_LIMIT,
    DOWN_LINE_LIMIT,
    DIS_AUTOBKG,   //20090506 Lisbona
    DOOR_SIZE      //20100419 eVS
};

char parm_names[256][PARM_STR_LEN] = {
    "outtime0",
    "outtime1",
    "input0",
    "input1",
    "sled",
    "dir",
    "threshold",
    "serial_id",
    "serial_br",
    "serial_db",
    "serial_pr",
    "serial_sb",
    "detect_area",
    "autoled",
    "inst_height",
    "serial_ms",
    "serial_sid",
    "serial_sbr",
    "serial_sdb",
    "serial_spr",
    "serial_ssb",
    "timebkg",
    "staticth",
    "slave_id",
    "sx_dx",
    "inst_dist",
    "wg_check",
    "sys_number",
    "sys_number_index",
    "sxlimit",
    "dxlimit",
    "sxlimit_riga_start",
    "dxlimit_riga_start",
    "sxlimit_riga_end",
    "dxlimit_riga_end" ,
    "cond_diff_1p",
    "diagnostic_en",
    "move_det_col0",
    "move_det_col1",
    "move_det_row0",
    "move_det_row1",
    "move_det_alfareg",
    "move_det_thr",
    "move_det_en",
    "door_stairs_en",
    "up_line_limit",
    "down_line_limit",
    "dis_autobkg",     //20090506 Lisbona
    "door_size"        //20100419 eVS
};

unsigned short parm_values[256];

unsigned short calib_parm_values[256] = {
    MAP_MINTH,
    MAP_THUNI,
    MAP_DISP,
    DAC_VREF,
    DAC_VPREC,
    DAC_VGAP,
    DAC_VREF,
    DAC_VPREC,
    DAC_VGAP,
    THBKG,
    STEPS_225[0],
    STEPS_225[1],
    STEPS_225[2],
    STEPS_225[3],
    STEPS_225[4],
    STEPS_225[5],
    STEPS_225[6],
    STEPS_225[7],
    STEPS_225[8],
    STEPS_225[9],
    STEPS_225[10],
    STEPS_225[11],
    STEPS_225[12],
    STEPS_225[13],
    STEPS_225[14],
    STEPS_225[15],
    WIN_SIZE,
    STEPS_240[0],
    STEPS_240[1],
    STEPS_240[2],
    STEPS_240[3],
    STEPS_240[4],
    STEPS_240[5],
    STEPS_240[6],
    STEPS_240[7],
    STEPS_240[8],
    STEPS_240[9],
    STEPS_240[10],
    STEPS_240[11],
    STEPS_240[12],
    STEPS_240[13],
    STEPS_240[14],
    STEPS_240[15]
};


#if defined(USE_NEW_DETECTION)
bool g_use_new_algorithm = true;
#else
bool g_use_new_algorithm = false;
#endif

unsigned char move_det_en = 0; //!< Flag relativo all'abilitazione del motion detection da interfaccia win_client.

int write_parms(char *name,unsigned short value);
int set_parms(char *name,unsigned short value);
unsigned short get_parms(char *name);
#ifdef LOAD_PARAMS
void load_parms();
#endif

int count_enabled = 1;	

unsigned char send_enable=0; //!< E' pari a 1 se sono in widegate e sono il master (inizializzata in load_parms())

#ifdef READ_INPUT
extern void enable_counting(unsigned long value);
#define INPUT_VAL ((use_0) ? (input0) : (input1))
#endif

void load_background(); // Carico la scena del background
//void record_counters(unsigned long &peoplein,
//                     unsigned long &peopleout);//Scrittura dei messaggi di log
void save_records();// Salvo in un txt la sequenza


void _draw_limits(IplImage* const & gray_img)
{
  if (limit_line_Down < NY)
    cvRectangle(gray_img, 
                cvPoint(0, limit_line_Down), cvPoint(gray_img->width-1, gray_img->height-1), 
                cvScalar(128,128,128), CV_FILLED);
  if (limit_line_Up > 0)
    cvRectangle(gray_img, 
                cvPoint(0, 0), cvPoint(gray_img->width-1, limit_line_Up), 
                cvScalar(128,128,128), CV_FILLED);
  if (limitSx > 0)
    cvRectangle(gray_img,
                cvPoint(limitSx_riga_start,0), cvPoint(limitSx,limitSx_riga_end), 
                cvScalar(128,128,128), CV_FILLED);
  if (limitDx < NX)
    cvRectangle(gray_img, 
                cvPoint(limitDx,limitDx_riga_end), cvPoint(gray_img->width-1,0), 
                cvScalar(128,128,128), CV_FILLED);
}


void
_draw(tPersonTracked* const & tracked_pers, const bool is_from_high, IplImage* const & img_msg, const int i)
{
  CvFont font;
  double hScale=0.5;
  double vScale=1.0;
  int    lineWidth=1;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth, CV_AA);
  char str[255];
  unsigned char soglia_porta_peopledetection = GetDoor();

#ifdef COMPUTE_FEET_COORDS
  static float conv_fact_mm2px = 15.0f/200.0f;
#endif

  if (tracked_pers->trac)
  {
    cvDrawCircle(img_msg, cvPoint(zoom_fact*tracked_pers->x+LR_BORDER, zoom_fact*tracked_pers->y+img_border), zoom_fact*tracked_pers->h/4, cvScalar(0,255,100), 1, CV_AA);
    cvDrawCircle(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER), (int)(zoom_fact*tracked_pers->y+img_border)), (int)(zoom_fact*tracked_pers->h*3.0/16.0), cvScalar(0,255,0), 1, CV_AA);
    cvDrawCircle(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER), (int)(zoom_fact*tracked_pers->y+img_border)), (int)(zoom_fact*(21*8)/4), cvScalar(255,255,0), 2, CV_AA);

#if defined(USE_HANDLE_OUT_OF_RANGE)
    if (OutOfRangeManager::getInstance().IsOutOfRange() && tracked_pers->h > 120)
    {
      int ray1 = OutOfRangeManager::getInstance().GetRay()*binning;
      //int ray2 = ray1 + 10;
      int cent_r = OutOfRangeManager::getInstance().GetRow()*binning + BORDER_Y;
      int cent_c = OutOfRangeManager::getInstance().GetCol()*binning + BORDER_X;
      assert(cent_r > 0 && cent_c > 0 && ray1 > 0);
      cvDrawCircle(img_msg, cvPoint((int)(zoom_fact*cent_c+LR_BORDER), (int)(zoom_fact*cent_r+img_border)), (int)(zoom_fact*ray1), cvScalar(200,200,0), 1, CV_AA);
      //cvDrawCircle(img_msg, cvPoint((int)(zoom_fact*cent_c+LR_BORDER), (int)(zoom_fact*cent_r+img_border)), (int)(zoom_fact*ray2), cvScalar(200,200,0), 1, CV_AA);
    }
    else
#endif
      cvDrawCircle(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER), (int)(zoom_fact*tracked_pers->y+img_border)), (int)(zoom_fact*tracked_pers->h*8.0/16.0), cvScalar(200,200,0), 1, CV_AA);

#if defined(USE_HANDLE_OUT_OF_RANGE) && defined(COMPUTE_FEET_COORDS)
    if (tracked_pers->black_pixels_num > 0)
    {
      int new_x, new_y;
      int new_z_mm = tracked_pers->z_mm;
      if (tracked_pers->h <= 120)
      {
        int pers_height = inst_height*10 - tracked_pers->z_mm;
        new_z_mm += pers_height/8;
      }
      _from_xyz_mm_to_xy_px(tracked_pers->x_mm, tracked_pers->y_mm, new_z_mm, new_x, new_y);
      cvDrawCircle(
        img_msg, 
        cvPoint(zoom_fact*new_x+LR_BORDER, zoom_fact*new_y+img_border), 
        zoom_fact*(((22*16 + FROM_DISP_TO_HEAD_RAY/2)/FROM_DISP_TO_HEAD_RAY)), 
        cvScalar(0,0,255), 
        1, 
        CV_AA);
    }
#endif

    cvDrawEllipse(img_msg, cvPoint(zoom_fact*tracked_pers->x+LR_BORDER, zoom_fact*tracked_pers->y+img_border), cvSize(zoom_fact*tracked_pers->wx, zoom_fact*tracked_pers->wy), 0, 0, 360, cvScalar(255,0,255), 1, CV_AA);
#if defined(COMPUTE_FEET_COORDS) && defined(USE_NEW_TRACKING)
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER-10), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER+10), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border)), cvScalar(255,0,255));
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border-10)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border+10)), cvScalar(255,0,255));
#endif
  }
  else
  {
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER-10), (int)(zoom_fact*tracked_pers->y+img_border)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER+10), (int)(zoom_fact*tracked_pers->y+img_border)), cvScalar(0,0,255));
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER), (int)(zoom_fact*tracked_pers->y+img_border-10)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x+LR_BORDER), (int)(zoom_fact*tracked_pers->y+img_border+10)), cvScalar(0,0,255));
#if defined(COMPUTE_FEET_COORDS) && defined(USE_NEW_TRACKING)
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER-10), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER+10), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border)), cvScalar(0,0,200));
    cvDrawLine(img_msg, cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border-10)), 
                        cvPoint((int)(zoom_fact*tracked_pers->x_mm*conv_fact_mm2px+img_shown_sz.width/2+LR_BORDER), (int)(zoom_fact*tracked_pers->y_mm*conv_fact_mm2px+zoom_fact*soglia_porta_peopledetection+img_border+10)), cvScalar(0,0,200));
#endif
  }


#  ifdef _DEBUG
#    ifdef USE_NEW_TRACKING
  sprintf(str, "%c%lu", (is_from_high) ? 'H' : 'L', tracked_pers->ID);
#    else
  sprintf(str, "%c%d", (is_from_high) ? 'H' : 'L', i);
#    endif
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale/1.5f, vScale/1.5f, 0, lineWidth, CV_AA);
  cvPutText(img_msg, str, cvPoint(zoom_fact*tracked_pers->x+1+LR_BORDER, zoom_fact*tracked_pers->y-1+img_border), &font,  (tracked_pers->trac) ? cvScalar(0,255,100) : cvScalar(0,255,0));
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth, CV_AA);
#  endif
#  if defined(_DEBUG) && defined(USE_NEW_TRACKING)
  //sprintf(str, "%c%lu: %c%c (%d,%d),dy=%d,(fx,fy)=(%d,%d),h=%d,mh=%d,nfr=%d,match=%d,cost=%lu,lives=%d", 
  sprintf(str, "%c%lu: %c (%d,%d),dy=%d,(fx,fy)=(%d,%d),h=%d,mh=%d,nfr=%d,lives=%d", 
    (is_from_high) ? 'H' : 'L',
    tracked_pers->ID, 
    tracked_pers->trac ? 'T' : '-', 
    // tracked_pers->cont ? 'C' : '-', 
    tracked_pers->x, tracked_pers->y, 
    tracked_pers->delta_y, tracked_pers->first_x, tracked_pers->first_y, 
    tracked_pers->h, tracked_pers->max_h, 
    tracked_pers->num_frames,  
    tracked_pers->life);
  int delta = (is_from_high) ? 0 : MSG_BORDER/2;
  cvPutText(img_msg, str, cvPoint(5, img_shown_sz.height+5+i*line_h+2*img_border+delta), &font, cvScalar(255,255,255));
#  endif
}


#ifdef READ_INPUT
void _print_input(IplImage* const & img_msg, IplImage* const & img, const int door, const bool use_0, const char input0, const char input1)
{
  if (INPUT_VAL)
    cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border), cvPoint(zoom_fact*img->width-1+LR_BORDER, zoom_fact*door+img_border), cvScalar(0,255,0), 3);
  else
    cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border), cvPoint(zoom_fact*img->width-1+LR_BORDER, zoom_fact*door+img_border), cvScalar(0,0,255), 1);
}
#endif

void
_manage_key_press(
  const int min_pause_ms,
  const int pause_step,
  const int init_pause,
  int & pause_ms,
  bool & manual_mode,
  bool & forward,
  unsigned long & i,
  bool & show,
  bool & exit,
  int & show_interval)
{
  char c;
  if (!manual_mode)
  {
    i++;
    c = cvWaitKey(pause_ms);
  }
  else
  {
    do
    {
      c = cvWaitKey(pause_ms);
    } while (c < 0 && c != 'z' && c != 'Z' && c != 'x' && c != 'X' && c != 27 && c != 32);
  }

  if ( c >= 0 ) 
  {
    if (c == 27)
    {
      printf("Frame no. %d\n", i+1);
      exit = true;
    }
    else
    {
      if (!manual_mode && (c == 'a' || c == 'A' || c == 'd' || c == 'D' || c == 's' || c == 'S'))
      {
        if (c == 'a' || c == 'A')
          pause_ms += pause_step;
        else if (c == 'd' || c == 'D')
        {
          if (pause_ms >= pause_step)
            pause_ms -= pause_step;
          pause_ms = max(pause_ms, 5);
        }
        else if (c == 's' || c == 'S')
          show = !show;
      }
      else if (manual_mode && (c=='z' || c=='Z' || c=='x' || c=='X'))
      {
        // garantisco refresh delle immagini
        if (show)
        {
          SYSTEMTIME start, stop;
          GetSystemTime(&start);
          unsigned long elapsed = 0;
          while (cvWaitKey(pause_ms) != -1 || elapsed < 10)
          {
            GetSystemTime(&stop);
            elapsed += ((stop.wHour-start.wHour)*60*60*1000 + (stop.wMinute-start.wMinute)*60*1000 + (stop.wSecond-start.wSecond)*1000 + (stop.wMilliseconds-start.wMilliseconds));
          }
        }

        if (c=='z' || c=='Z')
        {
          forward = false;
          if (i > 0)
          {
            i--;
            printf("Frame no. %d\n", i+1);
          }
        }
        else
        {
          forward = true;
          i++;
          printf("Frame no. %d\n", i+1);
        }
      }
      else if (c == 32)
      {
        manual_mode = !manual_mode;
        pause_ms = init_pause;
        if (!manual_mode)
          forward = true;
        else
        {
          show = true;
          printf("Frame no. %d\n", i+1);
        }
      }
    }
  }

  if (pause_ms < init_pause)
    show_interval = 2;
  else
    show_interval = 1;
}


int
main (int argc, char *argv[])
{
  // le seguenti righe sono state commentate in quanto le assegnazioni fatte corrispondono alle inizializzazioni di default
  //mem_door = false; 
  //ev_door_close = false;
  //ev_door_open = false; // se frame fermo arriva a 200 me lo setta a false. Analogo per ev_door_close
  //ev_door_open_rec = false;
  //num_pers = NUM_PERS_SING;
  //xt=160;

  unsigned long num_frames;

  setbuf(stdout, NULL); // from now on printf is unbuffered

#ifdef USE_RAW_DATA
#  ifdef PERFORMANCE_TEST
  bool info_memory = true;  // to be set true if one want to load all the sequence in memory
#  else
  bool info_memory = false;  // to be set true if one want to load all the sequence in memory
#  endif
  RawData raw_data(RAW_FN,info_memory,img_presence_flag,true,0,-1);
  assert(raw_data.isRawDataInitialized());
  num_frames = raw_data.getNumFrames();
#else
  CvCapture* avifile = cvCaptureFromFile( AVI_FN );
  num_frames = (unsigned long) cvGetCaptureProperty( avifile, CV_CAP_PROP_FRAME_COUNT );
  cvSetCaptureProperty(avifile, CV_CAP_PROP_POS_AVI_RATIO, 0.0);
#endif

  printf("Numero frames: %d\n", num_frames);

#ifdef SAVE_RESULT
  bool save = true;
#else
  bool save = false;
#endif

#ifdef SHOW_RESULT
  bool show = true;
#else
  bool show = false;
#endif

  int show_interval = 1;
  if (show)
  {
    cvNamedWindow("Frame", 1);
    cvNamedWindow("DSP", 1);
    IplImage* tmp = cvCreateImage(cvSize(NX,NY), IPL_DEPTH_8U, 1);
    cvShowImage("Frame", tmp);
    cvShowImage("DSP", tmp);
#ifdef USE_RAW_DATA
    if (img_presence_flag != RD_ONLY_DSP_AT_54FPS && img_presence_flag != RD_IMG_NOT_PRESENT)
    {
      cvNamedWindow("IMG", 1);
      cvShowImage("IMG", tmp);
    }
#endif
    cvReleaseImage(&tmp);
  }

  bool exit = false;

  // Inizializzo il numero totale di persone in entrata e in uscita
  //unsigned long peoplein = 0;
  //unsigned long peopleout = 0;
  unsigned long people[2] = {0, 0};

#ifdef USE_RAW_DATA
  first_frame = raw_data.getFirstFrameIdxInFile()+1;
#else
  first_frame = 1;
#endif

  SetDefaultValues(); // setto i campi di default del files (questa va dopo le assegnazioni a first_frame e prima del calcolo di num_frames)

//#ifdef USE_HANDLE_OUT_OF_RANGE
//  OutOfRangeManager::getInstance().ResetBG();
//#endif

  memcpy(parm_values, default_values, 256);

  // the following instruction must be placed after SetDefaultValues
  set_parms("dir", people_dir);
  set_parms("threshold", soglia_porta_main);
  set_parms("sxlimit", limitSx); 
  set_parms("dxlimit", limitDx);
  set_parms("sxlimit_riga_start", limitSx_riga_start);
  set_parms("dxlimit_riga_start", limitDx_riga_start);
  set_parms("sxlimit_riga_end", limitSx_riga_end);
  set_parms("dxlimit_riga_end", limitDx_riga_end);
  set_parms("up_line_limit", limit_line_Up);
  set_parms("down_line_limit", limit_line_Down);
  set_parms("inst_height", inst_height);
  set_parms("door_size", door_size);
  SetDoor(soglia_porta_main);

#ifdef USE_RAW_DATA
  num_frames = raw_data.getNumFrames()-first_frame+1;
#else
  num_frames = num_frames-first_frame+1;
#endif

  printf("Numero frames from first_frame: %d\n", num_frames);

#ifdef SAVE_RESULT
  char prefix[255];// prefix DOMANDA

  sprintf(prefix, "%s"
#  ifdef SUBTRACT_BG
    "_BGS"
#  endif
#  ifdef READ_INPUT
    "_DS"
#    ifdef USE_NEW_STRATEGIES
    "N"
#    else
    "O"
#    endif
#  endif
    , SAVE_PREFIX);

  char fn[255];
  sprintf(fn, "%s.avi", prefix);

  CvVideoWriter* avi = cvCreateVideoWriter(fn, -1, 30, cvSize(NX+2*LR_BORDER, NY+MSG_BORDER), 1);
  if (avi == NULL)
  {
    printf("Error: cannot create AVI file.\n");
    return -1;
  }
#endif //SAVE_RESULT

  load_background(); // carico il background

#ifdef LOAD_PARAMS
  load_parms();
#endif

#ifdef PERFORMANCE_TEST
  SYSTEMTIME start, stop;
  SYSTEMTIME start2, stop2;
  volatile unsigned long elapsed = 0;
  volatile unsigned long elapsed_max = 0;
  //GetSystemTime(&start);
#else
  FILE *fp_cnt = fopen("pcn1001_video6_algo.cnt", "w");
#endif

#ifdef READ_INPUT
  char prev_in0 = 0;
  char prev_in1 = 0;
  bool use_0 = true;
#ifndef USE_RAW_DATA
  FILE *fp_in0 = fopen(IN0, "rb");
  assert(fp_in0 != NULL);
  FILE *fp_in1 = fopen(IN1, "rb");
  assert(fp_in1 != NULL);
#  endif
#  ifdef SAVE_RESULT
  unsigned long countin_prev_open = 0;
  unsigned long countout_prev_open = 0;  
  int* delta_in = (int*)malloc((first_frame+num_frames-1)*sizeof(int));
  int* delta_out = (int*)malloc((first_frame+num_frames-1)*sizeof(int));
  int delta_idx = 0;
  int inertia = 0;
#  endif
#endif

  const int min_pause_ms = 10;
  const int pause_step = 15;
  const int init_pause = pause_step+min_pause_ms;  // under this value show_interval has to be increased by 1
  int pause_ms = init_pause;
  bool manual_mode = false;
  bool forward = true;

  // precedente numero di persone in entrata e in uscita
  unsigned long prev_people[2] = {0, 0};

  IplImage* img_msg = cvCreateImage(cvSize(img_shown_sz.width+2*LR_BORDER, img_shown_sz.height+MSG_BORDER) , IPL_DEPTH_8U, 3);
  IplImage* img = cvCreateImage(cvSize(NX, NY) , IPL_DEPTH_8U, 3);

  IplImage* tmp = NULL;
  IplImage* rgb = NULL;
  IplImage* tmp2 = NULL;
  if (show)
  {
    tmp = cvCreateImageHeader(cvSize(NX,NY), IPL_DEPTH_8U, 1);
    rgb = cvCreateImage(cvSize(NX,NY), IPL_DEPTH_8U, 3);
    tmp2 = cvCreateImageHeader(cvSize(NX,NY), IPL_DEPTH_8U, 1);
  }

  unsigned long i = first_frame-1;
  unsigned long last = i+num_frames-1;

#ifdef PERFORMANCE_TEST
  GetSystemTime(&start2);
#endif
  
  //peak_detection_init();  // this should be placed before main loop in the PCN as done here

  while (i<=last && !exit)
  {
    unsigned char* dsp = NULL;
    unsigned char* opt = NULL;

#ifdef USE_RAW_DATA
    //Ottieni la mappa di disparita'
    dsp = raw_data.getDisparityMap(i);
    opt = raw_data.getImage(i);

    //Crea l'immagine	
    IplImage* gray_img = cvCreateImageHeader(cvSize(NX,NY),IPL_DEPTH_8U, 1);
    cvSetImageData(gray_img, dsp, NX);
#else
    cvFillImage(img_msg, 0);
    const IplImage* img_orig = cvQueryFrame( avifile );
    IplImage* gray_img = cvCreateImage(cvGetSize(img_orig), IPL_DEPTH_8U, 1);
    cvCvtColor(img_orig, gray_img, CV_RGB2GRAY);
    dsp = (unsigned char*)gray_img->imageData;
#endif

#ifdef READ_INPUT
    char input0, input1;
#  ifdef USE_RAW_DATA
    input0 = raw_data.getInput0(i);
    input1 = raw_data.getInput1(i);
#  else
    fseek(fp_in0, i*sizeof(char), SEEK_SET);
    fread(&input0, sizeof(char), 1, fp_in0);
    fseek(fp_in1, i*sizeof(char), SEEK_SET);
    fread(&input1, sizeof(char), 1, fp_in1);
#  endif
    enable_counting( INPUT_VAL );
#  ifdef SAVE_RESULT
    if (inertia > 0)
    {
      inertia--;
      if (inertia == 0 || i == first_frame+num_frames-2)
      {
        delta_in[delta_idx] = peoplein-countin_prev_open;
        delta_out[delta_idx] = peopleout-countout_prev_open;
        countin_prev_open = peoplein;
        countout_prev_open = peopleout;
        delta_idx++;
      }
    }
    else
#  endif
    {
      if (prev_in0 != input0 || prev_in1 != input1)
      {
        if (input0 != 0 || input1 != 0)
        { // door opened
#ifdef SHOW_RESULT
          manual_mode = true;
#endif
        }
#  ifdef SAVE_RESULT
        else 
        { // door closed
          inertia = 20;
        }
#  endif
      }
    }

    prev_in0 = input0;
    prev_in1 = input1;
#else
    enable_counting( 1 );
#endif

    if (show)
    {
      cvSetData(tmp, dsp, NX);
      colorizeDepth(tmp, rgb);
      cvShowImage("DSP", rgb);
    }

#ifdef PERFORMANCE_TEST
    GetSystemTime(&start);
#endif


  //  unsigned char *ptrE_map = dsp;
  //for ( int i= 0; i < NN;++i,++ptrE_map)
  //{
  //  if (*ptrE_map == 16)
  //        *ptrE_map = 0;
  //}

    detectAndTrack(
      dsp,
      people[0],
      people[1],
      count_enabled,  // inizializzata a 1 e, se uso il segnale porta, modificata dalla enable_counting
      get_parms("threshold"),  // nel PCN qui viene passato il valore nel vettore dei parametri con get_parms("threshold") che è allineato a quello su file
      get_parms("dir"),  // nel PCN qui viene passato il valore nel vettore dei parametri con get_parms("dir") che è allineato a quello su file
      move_det_en
#ifdef USE_NEW_TRACKING
      , (min(limit_line_Down, NY-BORDER_Y)-max(limit_line_Up, BORDER_Y)+1)/4);
#else
      );
#endif

#ifdef PERFORMANCE_TEST
     GetSystemTime(&stop);
     volatile unsigned long delta_ms = ((stop.wHour-start.wHour)*60*60*1000 + (stop.wMinute-start.wMinute)*60*1000 + (stop.wSecond-start.wSecond)*1000 + (stop.wMilliseconds-start.wMilliseconds));
     elapsed += delta_ms;
     if (delta_ms > elapsed_max)
       elapsed_max = delta_ms;
#endif

    record_counters(people[people_dir], people[1-people_dir]);

    if (prev_people[0] != people[0] || prev_people[1] != people[1])
    {
#ifndef PERFORMANCE_TEST
      fprintf(fp_cnt, "%6d IN=%6ld OUT=%6ld\n", i, people[1], people[0]);
#endif

      if (!forward)
      {
        int diff_in = people[0] - prev_people[0];
        int diff_out = people[1] - prev_people[1];

        long diff_input = (diff_in+diff_out);
        if (diff_input <= people_count_input)
          people_count_input -= diff_input;
        else
          people_count_input = 0;

        long diff_output = (diff_out+diff_in);
        if (diff_output <= people_count_output)
          people_count_output -= diff_output;
        else
          people_count_output = 0;

        people[0] = people_count_input;
        people[1] = people_count_output;
      }

      prev_people[0] = people[0];
      prev_people[1] = people[1];
    }

#ifndef PERFORMANCE_TEST
#  if !defined(READ_INPUT) || defined(USE_NEW_STRATEGIES)
    if ((show && (i%show_interval==0)) || save)
#  else
    if ((show && (i%show_interval)==0 && (input0 != 0 || input1 != 0)) || save)
#  endif
    {
      if (opt)
      {
        cvSetData(tmp2, opt, NX);
        cvShowImage("IMG", tmp2);
      }

      _draw_limits(gray_img);

      cvCvtColor(gray_img, img, CV_GRAY2RGB);
      //colorizeDepth(gray_img, img);
      
      cvFillImage(img_msg, 0);
      cvSetImageROI(img_msg, cvRect(LR_BORDER, img_border, img_shown_sz.width, img_shown_sz.height));
      cvResize(img, img_msg);
      cvResetImageROI(img_msg);

      unsigned char soglia_porta_peopledetection = GetDoor();
#  ifdef READ_INPUT
      _print_input(img_msg, img, soglia_porta_peopledetection, use_0, input0, input1); //door, use_0, input0, input1);
#  else
      cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*soglia_porta_peopledetection+img_border), cvPoint(img_shown_sz.width-1+LR_BORDER, zoom_fact*soglia_porta_peopledetection+img_border), cvScalar(0,255,255), 1);
#  endif

#  ifdef USE_NEW_TRACKING
      {
        unsigned short door = soglia_porta_peopledetection;
        cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border-zoom_fact*DELTA_DOOR_TH), cvPoint(img_shown_sz.width-1+LR_BORDER, img_border+zoom_fact*(door-DELTA_DOOR_TH)), cvScalar(0,128,255), 1);
        cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border+zoom_fact*DELTA_DOOR_TH), cvPoint(img_shown_sz.width-1+LR_BORDER, img_border+zoom_fact*(door+DELTA_DOOR_TH)), cvScalar(0,128,255), 1);
#ifdef MIN_DIST_NEW_BLOB_FROM_DOOR_TH
        cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border-zoom_fact*MIN_DIST_NEW_BLOB_FROM_DOOR_TH), cvPoint(img_shown_sz.width-1+LR_BORDER, img_border+zoom_fact*(door-MIN_DIST_NEW_BLOB_FROM_DOOR_TH)), cvScalar(0,128,128), 1);
        cvLine(img_msg, cvPoint(0+LR_BORDER, zoom_fact*door+img_border+zoom_fact*MIN_DIST_NEW_BLOB_FROM_DOOR_TH), cvPoint(img_shown_sz.width-1+LR_BORDER, img_border+zoom_fact*(door+MIN_DIST_NEW_BLOB_FROM_DOOR_TH)), cvScalar(0,128,128), 1);
#endif
      }
#  endif

/*#ifdef USE_NEW_TRACKING
      int num_pers;
      tPersonTracked** inhi = get_inhi(num_pers); //!<Lista Persone traccate in entrata
      tPersonTracked** inlo = get_inlo(num_pers); //!<Lista persone traccate in uscita
#else*/
      // uso liste definite in peopletrack.cpp
      extern tPersonTracked** inhi;
      extern tPersonTracked** inlo;
//#endif

      {        
        int inhi_num = 0;
        int inlo_num = 0;
        for (int i=0; i<NUM_PERS_SING; i++)
        {
          if (inhi)
          {
            if (inhi[i])
            {
              inhi_num++;
              _draw(inhi[i], true, img_msg, i);
            }
          }

          if (inlo)
          {
            if (inlo[i])
            {
              inlo_num++;
              _draw(inlo[i], false, img_msg, i);
            }
          }
        }

        CvFont font;
        double hScale=0.5;
        double vScale=1.0;
        int    lineWidth=1;
        char str[255];

        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth, CV_AA);
        sprintf(str, "%d", people[0]);
        cvPutText(img_msg, str, cvPoint(0+LR_BORDER-35, img_shown_sz.height-5+img_border), &font, cvScalar(255,255,255));
        sprintf(str, "%d", people[1]);
        cvPutText(img_msg, str, cvPoint(0+LR_BORDER-35, line_h+5+img_border), &font, cvScalar(255,255,255));

        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale/1.5f, vScale/1.5f, 0, lineWidth, CV_AA);
        sprintf(str, "%d", inlo_num);
        cvPutText(img_msg, str, cvPoint(img_shown_sz.width+25+LR_BORDER, img_shown_sz.height-5+img_border), &font, cvScalar(255,255,255));
        sprintf(str, "%d", inhi_num);
        cvPutText(img_msg, str, cvPoint(img_shown_sz.width+25+LR_BORDER, line_h+5+img_border), &font, cvScalar(255,255,255));
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth, CV_AA);
      }

      if (show)
      {
        cvShowImage( "Frame", img_msg );
      }

#  ifdef SAVE_RESULT
      if (save)
      {
#    ifdef JPEG
        int p[3];
        p[0] = CV_IMWRITE_JPEG_QUALITY;
        p[1] = 95;
        p[2] = 0;

        char fn[255];
        sprintf(fn, "%s%06d.jpg", SAVE_PREFIX, i+first_frame);
        cvSaveImage(fn, img, p);
#    else
        cvWriteFrame(avi, img_msg);
#    endif
      }
#  endif
      
    }
#endif

#ifdef NDEBUG
    if ((i-first_frame+1)%100==0)
      printf("%.2f%%\tIN = %4d\tOUT = %4d\n", (float)(100*i)/(float)(first_frame+num_frames-1), people[people_dir], people[1-people_dir]);
#else
    if (i%100==0)
      printf("%.2f%%\tIN = %4d\tOUT = %4d\n", (float)(100*i)/(float)(first_frame+num_frames-1), people[people_dir], people[1-people_dir]);
#endif

#ifdef USE_RAW_DATA
    cvReleaseImageHeader(&gray_img);
    free(dsp);
#else 
    cvReleaseImage(&gray_img);
#endif

#ifdef SHOW_RESULT
    _manage_key_press(min_pause_ms, pause_step, init_pause, pause_ms, manual_mode, forward, i, show, exit, show_interval);
#else
    i++;
#endif

    if (opt)
      free(opt);
  }

#ifdef PERFORMANCE_TEST
  GetSystemTime(&stop2);
#endif

#if defined(READ_INPUT) && defined(SAVE_RESULT)
  {
    char fn[255];
    sprintf(fn, "%s_delta_in.txt", prefix);

    FILE *in = fopen(fn,"w");
    fprintf(in, "ID\tDELTA\n");
    for (int i=0; i<delta_idx; ++i)
    {
      fprintf(in, "%d\t%d\n", i, delta_in[i]);
    }
    fclose(in);  
    free(delta_in);

    sprintf(fn, "%s_delta_out.txt", prefix);
    FILE *out = fopen(fn,"w");
    fprintf(out, "ID\tDELTA\n");
    for (int i=0; i<delta_idx; ++i)
    {
      fprintf(out, "%d\t%d\n", i, delta_out[i]);
    }
    fclose(out);  
    free(delta_out);
  }
#endif

  cvReleaseImage(&img);
  cvReleaseImage(&img_msg);

#ifndef PERFORMANCE_TEST
  fclose(fp_cnt);
#endif

  printf("100.00%%\tIN = %4d\tOUT = %4d\n", people[people_dir], people[1-people_dir]);

#ifdef PERFORMANCE_TEST
  printf("time per frame (from sum of partial durations) %f ms\n", (float)elapsed/(float)(num_frames));
  volatile unsigned long delta_tot = ((stop2.wHour-start2.wHour)*60*60*1000 + (stop2.wMinute-start2.wMinute)*60*1000 + (stop2.wSecond-start2.wSecond)*1000 + (stop2.wMilliseconds-start2.wMilliseconds));
  printf("time per frame (from total duration) %f ms\n", (float)delta_tot/(float)(num_frames));
  printf("max time per frame %ld ms\n", elapsed_max);
  cvWaitKey(0);
#endif

  if (show)
  {
    cvDestroyAllWindows();
    cvReleaseImageHeader(&tmp);
    cvReleaseImage(&rgb);
    cvReleaseImageHeader(&tmp2);
  }

#ifdef SAVE_RESULT
  cvReleaseVideoWriter(&avi);
#endif

#ifdef READ_INPUT
#  ifndef USE_RAW_DATA
  fclose(fp_in0);
  fclose(fp_in1);
#  endif
#endif

  save_records();
#ifndef USE_RAW_DATA
  cvReleaseCapture(&avifile);
#endif 

//#ifdef _DEBUG
  printf("Premi enter per chiudere...");
  getchar();
//#endif

  return 0;
}

#ifdef _MSC_VER
#include <windows.h>
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
  return main (__argc, __argv);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

int pcn_status = 0;
int old_pcn_status = 0;
int diagnostic_en = 0;

char records[MAX_RECORDS][128];  //!< Buffer storing log messages (reporting counting results) periodically written on file.
unsigned long records_idx = 0;   //!< Indice del buffer dei messaggi di log #records.
unsigned long counter_in = 0;    //!< Contatore delle persone entrate.
unsigned long counter_out = 0;   //!< Contatore delle persone uscite.
//unsigned long door_in;
//unsigned long door_out;
int error_pcn_status_code = 0;
//unsigned long out0 = 0; //!< optocoupled out0 queue. ???
//unsigned long out1 = 0; //!< optocoupled out1 queue. ???
int  record_enabled = 1;			//enabling in/out counters recording

extern unsigned char total_sys_number;
extern unsigned char current_sys_number;

extern unsigned char Bkgvec[NN];
extern int svec[NN];
//extern unsigned char vm_bkg;

void load_background()
{
    // loading scene background image
#ifdef SUBTRACT_BG
    FILE *in;
    if((in = fopen(BACKGROUND_FN,"rb")))
    {
        fread(Bkgvec,sizeof(Bkgvec),1,in);
        fread(svec,sizeof(svec),1,in);
        //fread(&vm_bkg,sizeof(vm_bkg),1,in);
        fclose(in);
    }
#else
  memset(Bkgvec, 0, sizeof(Bkgvec));
  memset(svec, 0, sizeof(svec));
  //vm_bkg = 0;
#endif
}


void save_records()
{
  if(records_idx)
  {
      FILE *recordfd;
      if((recordfd = fopen("records.txt","a+")))// saving last records to file
      {
          fseek(recordfd,0L,SEEK_END);
          for(unsigned long i=0;i<records_idx;++i)
              fprintf(recordfd,"%s",records[i]);          
          fclose(recordfd);
          records_idx = 0; 
      }
  }
}


#ifdef LOAD_PARAMS
void load_parms(void)
{
    FILE *parms;
    char name[PARM_STR_LEN];
    char svalue[PARM_STR_LEN];
    unsigned short value;
    int ret;

    parms = fopen(PM_FILENAME,"r"); 
    if (parms == NULL)
      exit(-10);
    else
    {
        do
        {
            ret = fscanf(parms,"%s ",name);
            if((get_parms(name) != 0xFFFF) && ret != EOF)	// checking if the parameter is valid
            {
                fscanf(parms,"%s",svalue);
                value = strtol(svalue,NULL,16);
                
                set_parms(name,value);				// saving parameters in memory
                write_parms(name,value);			// writing parameters on devices
            }
        }
        while(ret != EOF); 
    }  
    
    fclose(parms);
}
#endif
#endif  // Fine 
