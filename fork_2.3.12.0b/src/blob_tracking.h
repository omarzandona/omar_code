#ifndef __BLOB_TRACKING__
#define __BLOB_TRACKING__

#include "directives.h"
#ifdef USE_NEW_TRACKING

#include "peopledetection.h"

#define DELTA_DOOR_TH 9  // > 3 since the detection has an error of 3 pixels because of binning

typedef struct
{
  bool cont;              //!< cont=true allora la persona &egrave; contatabile
  bool trac;              //!< trac=true allora la persona &egrave; stata inseguita dal tracker
  int life;               //!< numero di vite
  unsigned char wx;       //!< larghezza del centroide
  unsigned char wy;       //!< altezza del centroide

  int num_frames;
  int delta_y;
  int max_h;

  unsigned char h;        //!< altezza (disparit&agrave;) della testa rilevata
  int x;                  //!< colonna del centroide
  int y;                  //!< riga del centroide
  unsigned char first_y;  //!< memorizza la riga in cui compare la persona per la prima volta
  unsigned char first_x;  //!< memorizza la colonna in cui compare la persona per la prima volta

#if defined(_DEBUG) || defined(VERBOSE)
  unsigned long ID;
#endif

#ifdef VERBOSE
  unsigned long first_frame;
#endif
} tPersonTracked;


void track(const int *people, 
           const int person, 
           unsigned char *disparityMap,
           const unsigned char *dimpers,
           const unsigned char *hpers,
           unsigned long &trackin,
           unsigned long &trackout, 
           const int & diff_cond_1p,
           const unsigned char & direction_inout,
           const unsigned short & door_threshold,
           const unsigned char & current_sys_number,
           const unsigned char & total_sys_number,
           const unsigned char & door_stairs_en,
           const unsigned char & move_det_en,
           const bool & count_true_false,
           int &num_pers,
           int &xt,
           const int &min_y_gap);

void initpeople(unsigned long pi,unsigned long po,
                unsigned char & total_sys_number, int & num_pers);

void deinitpeople(const int & num_pers);

void clearpeople(unsigned long &trackin,unsigned long &trackout, 
                 const unsigned short & door_threshold, const unsigned char & move_det_en,
                 const bool & count_true_false, const int & num_pers, const int & min_y_gap);

void CloseDoor(unsigned long & trackin,unsigned long & trackout,
               const unsigned char & direction, const unsigned short & door_threshold, 
               const unsigned char & move_det_en,const bool & count_true_false,
               const int & num_pers, const int & min_y_gap);

#endif
#endif
