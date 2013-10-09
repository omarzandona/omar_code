/*!
\file peopletrack.h

\brief File header di peopletrack.cpp contenente le strutture dati, le costanti e
le funzioni necessarie al tracking delle persone.

Contiene la definizione della struttura dati tPersonTracked, alcune define
specifiche per il tracking e i prototipi delle funzioni.

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include "directives.h"
#ifndef USE_NEW_TRACKING

#ifndef __PEOPLETRACK__
#define __PEOPLETRACK__

// eVS 20100421
#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */

/*!
\def YT
\brief ???
*/
#define YT 120

/*!
\def SEARCH
\brief ???
*/
#define SEARCH 20

/*!
\def LIFE
\brief ???
*/
#define LIFE 3 // 20111202 eVS, restored 3 instead of 4

/*!
\def PAS_MIN
\brief Numero passi minimo per essere contati.

Numero di passi minimo che una persona deve aver fatto da quando e' 
entrata a quando e' uscita per poter essere contata. 

Un passo corrisponde ad una spostamento inter-fotogramma 
di almeno 3 pixel.
*/
#define PAS_MIN 4

/*!
  \struct tPersonTracked
  \brief Struttura dati contenente le informazioni delle persone inseguite.
  
  Si tratta di una struttura simile alla struttura tPersonDetected in quanto condivide i dati che
  descrivono le propriet&agrave; geometriche del blob (centroide, dimensioni e 
  altezza). Le due strutture si differenziano per altri campi che sono specifici
  per la funzione di detezione (nel caso di tPersonDetected) e per la funzione di inseguimento
  (nel caso di #tPersonTracked).
  
  \see tPersonDetected
*/
typedef struct
{
	bool cont;				//!< cont=true allora la persona &egrave; contatabile
	bool trac;				//!< trac=true allora la persona &egrave; stata inseguita dal tracker
	unsigned char life;		//!< numero di vite
	unsigned char wx;		//!< larghezza del centroide
	unsigned char wy;		//!< altezza del centroide
	int passi;				//!< numero di passi fatti dalla persona nella scena
	unsigned char h;		//!< altezza (disparit&agrave;) della testa rilevata
	int x;					//!< colonna del centroide
	unsigned char y;		//!< riga del centroide
	unsigned char first_y;  //!< memorizza la riga in cui compare la persona per la prima volta
} tPersonTracked;


/**********************************************************/
unsigned char ReadDoor();
void CloseDoor(unsigned long & trackin,unsigned long & trackout,
               const unsigned char & direction, const unsigned short & door_threshold, 
               const unsigned char & move_det_en,const bool & count_true_false,
               const int & num_pers);
//void SetDoor(unsigned char soglia);
void SetPassi(const unsigned char & direction, const int & num_pers);
void clearpeople(unsigned long &trackin, unsigned long &trackout, 
                 const unsigned short & door_threshold, const unsigned char & move_det_en,
                 const bool & count_true_false, const int & num_pers);
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
           const unsigned char & door_kind,
           const unsigned char & current_sys_number,
           const unsigned char & total_sys_number,
           const unsigned char & door_stairs_en,
           const unsigned char & move_det_en,
           const bool & count_true_false,
           int &num_pers,
           int &xt);

void initpeople(unsigned long pi,unsigned long po, unsigned char & total_sys_number, int & num_pers);

void deinitpeople(const int & num_pers);
/**********************************************************/

#endif

#endif
