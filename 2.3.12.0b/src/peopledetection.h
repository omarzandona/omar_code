#ifndef __PEOPLEDETECTION__
#define __PEOPLEDETECTION__
/*!
\file peopledetection.h
\brief File header di peopledetection.cpp contenente le strutture dati, le costanti e le 
funzioni necessarie alla detection delle persone.

Contiene la definizione delle seguenti strutture dati: ogg_static, model e per.

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include "directives.h"

#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */

const int MIN_M = 3*16+1;   /*!< \brief Altezza minima delle proiezioni (valori tra 30 e 50) ??? 

Minimum value for the disparity map (0..255) in order to be considered a candidate for people detection. */

#define NX 160     //!< Numero di colonne dell'immagine sottocampionata
#define NY 120     //!< Numero di righe dell'immagine sottocampionata
#define NN (NX*NY) //!< Numero totale di pixel dell'immagine sottocampionata (#NX*#NY=160*120)
#define time_bkg   //!< Using this define the feature about the time background update ("Time bkg" in the win_client) is activated and compiled.

// 20100512 eVS
#define NUM_PERS_SING 10  //!< maximum number of person detectable for each sensor

/*!
\struct model
\brief Potenziale blob ottenuto mediante l'analisi della proiezione sull'asse X (piuttosto che sull'asse Y).
*/
typedef struct 
{
        unsigned char mean; ///< valor medio
        unsigned char dev; ///< deviazione standard
        unsigned char wid; ///< larghezza del blob (larghezza del blob orizzontale piuttosto che altezza del blob verticale, a seconda che sia stato analizzata la proiezione della mappa di disparit&agrave; sull'asse X o sull'asse Y)
        unsigned char pos; ///< centro del blob (colonna piuttosto che riga)
}model;

#ifdef time_bkg
/*!
\struct ogg_static
\brief Contiene lo sfondo statico, la deviazione standard dello sfondo e l'istante di acquisizione.
*/
typedef struct
{
  unsigned char BkgTmpStatic[NN]; ///< Sfondo statico.
  int svec[NN]; ///< Deviazione standard dello sfondo statico.
  unsigned char min; ///< Data e ora di acquisizione dello sfondo espressa in minuti.
}ogg_static;
#endif


/*!
\struct tPersonDetected
\brief Struttura dati contenente le informazioni relative ai potenziali blob trovati nel frame corrente.
\see tPersonTracked
*/
typedef struct 
{
        int real_x;///<X reale calcolata per evitare che nel caso di due o piu PCN collegati in serie una persona venga contata due volte
        bool fusa;///<la persona &egrave; stata fusa con un altra vicina o sovrapposta ???
        unsigned char x;///<colonna del centroide
        unsigned char y;///<riga del centroide
        unsigned char wx;///<larghezza del centroide
        unsigned char wy;///<altezza del centroide
        unsigned char h;///<altezza della testa trovata in termini di disparit&agrave a 7bit;
        unsigned char sys;///<numero corrente del sistema che ha rilevato questa persona/testa
        unsigned char sincro;///<...
        unsigned char stato_canc_da;///<...
        unsigned char ho_canc_su_sys;///<...
        int h_se_cancellato;///<...
}tPersonDetected;

/*********************************************************/
#ifndef USE_NEW_TRACKING
extern void SetPassi(const unsigned char & direction, const int & num_pers); //, unsigned char wideg
#endif
/*extern void CloseDoor(unsigned long & trackin,unsigned long & trackout,
                      const unsigned char & direction, const unsigned short & door_threshold, 
                      const unsigned char & move_det_en, const bool & count_true_false,
                      const int & num_pers);
extern unsigned char ReadDoor();
extern void GetCnt(unsigned long& in, unsigned long& out);
*/

void SetDoor(unsigned char soglia);
unsigned char GetDoor();

void detectAndTrack(unsigned char *disparityMap,
              unsigned long &peoplein,
              unsigned long &peopleout, 
              const int &enabled, 
              const unsigned short &door,
              const unsigned char &direction,
              const unsigned char &move_det_en
#ifdef USE_NEW_TRACKING
              , const int &min_y_gap);
#else
              );
#endif

//void initpeople(unsigned long pi,unsigned long po);
//void deinitpeople(const int num_pers);
void SetBkgThreshold(unsigned char soglia);
void InitPers(tPersonDetected* persone);
void CalculateReal_x(tPersonDetected* persone);
void Merge_and_Filter_people(tPersonDetected* persone, int &num_pers);
void WritePersDetected(tPersonDetected* persone, const int num_pers_det);

void initpeople(unsigned long pi,unsigned long po, unsigned char & total_sys_number, int & num_pers);
void deinitpeople(const int & num_pers);

#ifdef debug_
void WritePersDebug(unsigned char* vect);
void WriteRealXDebug(unsigned char* vect);
#endif

#ifdef time_bkg
unsigned char GetMin();
void InitStaticObj();
void CheckStatic();
void FindStaticObj();
void SetStaticTh(int soglia);
void SetMinBkgTh(unsigned char soglia);
#endif
/***************************************************************/

#endif
