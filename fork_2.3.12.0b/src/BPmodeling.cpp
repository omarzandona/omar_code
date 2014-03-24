/*!
\file BPmodeling.cpp

\brief Contiene l'implementazione dell'algoritmo per la modellazione dei pixel neri che si presentano nella scena.

Questa classe permette di modellare la presenza dei pixel neri presenti all'interno della scena.
L'idea di fondo &egrave; quella di non voler considerare i pixel neri dati da rumore (o fallimenti stereo in generale) nel calcolo dei pixel neri
nella gestione della situazione di Out-Of-Range.
Per far ci&ograve; utilizziamo la classe implementata in questo file.
La classe mette a disposizione dei metodi che permetteno di imparare quali pixel neri sono dovuti
appunto a rumore e quindi si presentano in maniera costante/permanente nella scena.
Questo permette quindi di modellare la situazione meno frequente di Out-Of_Range in maniera tale 
da garantire l'attivazione del blob virtuale solo in caso di effettiva situazione anomala
consentendo anche di eliminare la presenza di blob virtuali dovuti a rumore che potrebbero portare a problemi di conteggio
Prova per vedere la modifica
\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <assert.h>

#include "BPmodeling.h"
#include "directives.h"


#ifdef SHOW_RESULT
#define SHOW_BP
#endif

/*!
Costruttore di default della classe BPModeling: Inizializza i campi necessaria per la modellazione della maschera
*/
BPmodeling::BPmodeling()
{
  border_x = -1;
  border_y = -1;
  width = 0;
  height = 0;
  dim = 0;
  BP = NULL;
  num_mask_pixel = 0;
}


/*!
Costruttore della classe BPModeling: Inizializza con i parametri in input i campi per la modellazione della maschera
*/

BPmodeling::BPmodeling(
  const int i_w,  ///< [in]  image width
  const int i_h,  ///< [in]  image height
  const int i_bx,  ///< [in] horizontal border
  const int i_by)  ///< [in] vertical border
{
  border_x = i_bx;
  border_y = i_by;
  width = i_w;
  height = i_h;
  dim = 2*(width-2*border_x)*(height-2*border_y)*sizeof(unsigned short);
  num_mask_pixel = 0;
  BP = (unsigned short*) malloc(dim);

  Reset();
}

/*!
Distruttore di default
*/
BPmodeling::~BPmodeling()
{
  free(BP);
}
/*!
Reset del modello: Resetta il modello dei BP azzerandone i due contatori per ogni frame
*/

void
BPmodeling::Reset()
{
  num_mask_pixel = 0;
  memset(BP, 0, dim);
}

/*
GetMask()  Restituisce l'immagine che rappresenta la modellazione dei pixel neri.
In particolare verr&agrave; creata una immagine delle dimensioni passate come parametro, in cui
i pixel di coordinate analoghe a quelli rilevati dal modello come pixel neri sono di colore bianco (255)
altrimenti restano di colore nero (0).
Questo modalit&agrave; risulta utile in fase di debug e ricordiamo che non vengono alterati i pixel nel del bkg ne del frame corrente
quindi questa funzione ci mostra solamente una comoda rappresentazione dei pixel neri.

Il criterio di decisione che indica se il pixel &egrave; effettivamente nero si basa sul fatto che il numero di frame del suo contatore
per il colore nero superi di quattro volte il contatore dei frame in cui il suo colore non era nero.
Inoltre viene anche garantito il fatto che il primo contatore abbia raggiunto i 54 frame.

\param o_BPbw   [in|out] Immagine che rappresenta la presenza o meno di pixel neri
\param i_width  [in]     Larghezza immagine
\param i_height [in]     Alteza dell'immagine
*/

void
BPmodeling::GetMask(unsigned char* const & o_BPbw, const int i_width, const int i_height)
{
  assert(i_width == width);
  assert(i_height == height);
  
  num_mask_pixel = 0;
  memset(o_BPbw, 0, (width*height));
  for (int r=border_y; r<height-border_y; ++r)
  {
    int index_r1 = (r * width);
    int index_r2 = ((r-border_y) * (width-2*border_x));
    unsigned short* ptr_row_BP = &(BP[2*index_r2]);
    for (int c=border_x; c<width-border_x; ++c, ptr_row_BP+=2)
    {
      if (ptr_row_BP[0] > ptr_row_BP[1]/4 && ptr_row_BP[0] > 54)
      {
        o_BPbw[index_r1 + c] = 255;
        ++num_mask_pixel;
      }
      else
        o_BPbw[index_r1 + c] = 0;
    }
  }
}

/*!
UpdateModel() Aggiorna i contatori per ogni pixel scandendo la mappa.
Questo viene fatto solamente quando le porte sono aperte e quando non vi &egrave; situazione di Out-Of-Range per garantire
al modello di imparare il background dinamico in situazioni non ottimali.

\param i_bp_mask  [in] Maschera che contiene i due contatori per ogni frame
\param i_width    [in] Larghezza della maschera
\param i_height   [in] Altezza della maschera

*/
void
BPmodeling::UpdateModel(const unsigned char* const & i_bp_mask, const int i_width, const int i_height)
{
  assert(i_width == width);
  assert(i_height == height);

  const unsigned short MAX_VAL = 3700;  // 7/4*N=54*60*2=6480 => N=3700 cioe' 2min di solo nero (o di solo "altro" cioé di livelli diversi da zero)

#ifdef SHOW_BP
  unsigned char* BPbw = (unsigned char*) malloc(width*height);
#endif

  for (int r=border_y; r<height-border_y; ++r)
  {
    const unsigned char* ptr_row_DSP = &(i_bp_mask[r * width + border_x]);

    int index_r = ((r-border_y) * (width-2*border_x));
    unsigned short* ptr_row_BP = &(BP[2*index_r]);
    for (int c=border_x; c<width-border_x; ++c, ++ptr_row_DSP, ptr_row_BP+=2)
    {
      unsigned short value = (*ptr_row_DSP == 0);
      if (ptr_row_BP[value] < MAX_VAL)
        ptr_row_BP[value]++;
      else
      {
        // se il contatore da incrementare &egrave; saturo decremento entrambi i contatori
        //ptr_row_BP[value]--;  // essendo saturo questo contatore lo posso sicuramente decrementare
        if (ptr_row_BP[1-value] > 0)  // questo contatore potrebbe essere 0 e quindi controllo
          ptr_row_BP[1-value]--;
      }
    }
  }

#ifdef SHOW_BP
  //cvNamedWindow("BP", 1);
  //int w = width-2*border_x;
  //int h = height-2*border_y;
  //IplImage* tmp_virtual_blob = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 1);
  //cvSetData(tmp_virtual_blob, BP, w);
  //cvShowImage("BP", tmp_virtual_blob);
  //cvReleaseImageHeader(&tmp_virtual_blob);

  GetMask(BPbw, width, height);

  cvNamedWindow("BPbw", 1);
  //tmp_virtual_blob = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 1);
  IplImage* tmp_virtual_blob = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
  //cvSetData(tmp_virtual_blob, BPbw, w);
  cvSetData(tmp_virtual_blob, BPbw, width);
  cvShowImage("BPbw", tmp_virtual_blob);
  cvReleaseImageHeader(&tmp_virtual_blob);

  free(BPbw);
#endif
}

/*!
GetBP() Ritorna il modello corrente dei pixel neri

\param o_BPwidth  [out]  Larghezza mappa
\param o_BPheight [out]  Altezza mappa

*/
const unsigned short*
BPmodeling::GetBP(int & o_BPwidth, int & o_BPheight)
{
  return (const unsigned short*) BP;
}
