/*!
\file OutOfRangeManager.h
\brief Contiene l'header per la definizione della classe OutOfRangeManager

In questo file si definiscono le funzioni  implementate nel file OutOfRangeManager.cpp

\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#ifndef OUTOFRANGEMANAGER_H_
#define OUTOFRANGEMANAGER_H_

#include "directives.h"

#ifdef USE_HANDLE_OUT_OF_RANGE

#include "default_parms.h"
#include "peopledetection.h"
#include "blob_tracking.h"
#include "blob_detection.h"


#ifndef NOMINMAX
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#endif  /* NOMINMAX */

/*!
\brief Classe singleton per la gestione dell'out-of-range
*/
class OutOfRangeManager
{
public:
  static OutOfRangeManager & getInstance()
  {
    static OutOfRangeManager obj;
    return obj;
  }

  void SetEnableStateOutOfRange(const bool state);  ///< Abilita/Disabilita la gestione dello situazione di out-of-range.
  void HandleOutOfRange(tPersonDetected* const & persone,
    const int num_pers,
    unsigned char* const & disparityMap,
    const unsigned char* const & BP_map,
    const unsigned char* const & BP_BG,
    const bool motion,
    const int door_threshold,
    const int num_mask_pixel);  ///< Verifica e gestione dello stato di out-of-range.
  int GetRay();  ///< Ritorna il raggio del blob virtuale (-1 se non c'e' out-of-range)
  int GetRow();  ///< Ritorna la riga del blob virtuale (-1 se non c'e' out-of-range)
  int GetCol();  ///< Ritorna la colonna del blob virtuale (-1 se non c'e' out-of-range)
  bool IsOutOfRange();  ///< Ritorna true se c'e' out-of-range
  bool IsOutOfRangeEnabled();  ///< Ritorna true se la gestione dell'out-of-range e' attiva
  bool CheckBkgOk(unsigned char* const & map);  // Funzione chiamata dal widegate per controllare lo sfondo
  int CountBlackPixels(unsigned char* const & disparityMap);  ///< Conta numero pixels neri (usata in abbinamento a isBackgroundCheckInProgress()).
  bool isBackgroundCheckInProgress(const int black_pixel_cnt,
    bool & is_background_ok,
    const bool reinit = false);  ///< Controllo sul background per verificare se e' possibile usare HandleOutOfRange().
private:
  OutOfRangeManager();  ///< Costruttore della classe OutOfManager.
  ~OutOfRangeManager(){};  ///< Distruttore.

  // seguono costanti interne
  static const int NUM_BLACK_PIXEL_TO_DISABLE = 1500;  ///< Indica il numero minimo di pixel neri per NON abilitare la gestione OOR 
  static const int NUM_BLACK_PIXEL_ON = 600/(binning*binning);  ///< Indica il numero minimo di pixel a zero per decidere se abilitare HandleOutOfRange().
  static const int NUM_BLACK_PIXEL_OFF = (2*NUM_BLACK_PIXEL_ON)/3;  ///< Indica il numero minimo di pixel a zero per decidere se abilitare HandleOutOfRange().
  static const int MAX_BLACK_PIXELS_NUM_TO_BE_REALIABLE = (2*NUM_BLACK_PIXEL_TO_DISABLE)/3;  ///< Indica il numero massimo di pixel a zero per decidere se disabilitare HandleOutOfRange(). Il 25% del valore usato dall'interruttore a ON.
  static const int VIRTUAL_BLOB_RAY = (90+binning/2)/binning;  ///< Raggio del blob virtuale.
  static const int VIRTUAL_BLOB_BORDER_DIM = (20+binning/2)/binning; ///< Questo bordo deve essere maggiore dei kernel usati per le dilatazioni (ovvero strel_sze_orig_h, strel_sze_orig_v, strel_sze2_orig_h, strel_sze2_orig_v definiti in blob_detection.cpp) altrimenti la dilatazione chiude il buco appositamente creato.
  static const int DISP_VALUE_FOR_VIRTUAL_BLOB_BORDER = 20;  ///< Valore usato per i bordi scuri del blob virtuale in caso di out-of-range (deve essere diverso da #OUT_OF_RANGE_OR_STEREO_FAILURE e da #UNIFORM_ZONE_OR_DISP_1 definiti in blob_detection.h e minore di #MIN_M).
  static const int MAX_RAY = VIRTUAL_BLOB_RAY+VIRTUAL_BLOB_BORDER_DIM;  ///< Raggio massimo del blob virtuale comprensivo del bordo scuro aggiunto per aiutare la detection nei dintorni del blob virtuale e allo stesso tempo permette la detezione di picchi spuri dovuti a spalle o rumore non ben coperti dal blob viruale (vedi _clear_blobs_around_virtual_blob()).
  static const int MIN_RAY = MAX_RAY-18/binning;  ///< Raggio minimo del blob virtuale.
  static const int DIM_KERNEL_COLORS = MAX_RAY + 1; ///< Dimensione del vettore dei colori del blob virtuale. Il +1 serve per il colore del centro del blob.
  static const int VIRTUAL_BLOB_BOX = (MAX_RAY * 2 + 1);   ///< Dimensione del lato della maschera che contiene il blob virtuale.

#ifdef USE_STATIC_BLOB_CHECK
  static const int MAX_NUM_FRAME_FOR_STATIC_BLOB = 4*54;  ///< Massimo numero di frame in cui il blob virtuale è statico
#endif
  // seguono membri
  bool m_enable_handle_out_of_range;  ///< Flag per abilitare o meno la gestione dell'out-of-range
  bool m_is_out_of_range;  ///< Flag di stato di out-of-range (true se in out-of-range).
  bool m_is_from_high;  ///< Flag che indica la direzione del blob ( proviene o meno dall'alto)
#ifdef USE_STATIC_BLOB_CHECK
  bool m_blob_static;  ///< Flag per indicare una situazione di staticit&agrave del blob virtuale dovuto a rumore
#endif
  int m_num_black_pixels,  ///< Se in out-of-range contiene il numero di pixel in out-of-range altrimenti -1
    m_num_DSP,  ///< Se in out-of-range contiene il numero di pixel con una disparita' elevata nell'intorno del centroide altrimenti -1
    m_cent_r,  ///< Se in out-of-range contiene la riga del centroide dei pixel in out-of-range altrimenti -1
    m_cent_c,  ///< Se in out-of-range contiene la colonna del centroide dei pixel in out-of-range altrimenti -1
    m_ray;  ///< Se in out-of-range contiene la dimensione del blob virtuale altrimenti -1

  // le variabili che seguono sono inizializzate nel costruttore e contengono dati usati dal manager
  unsigned int kernel_color_weights[DIM_KERNEL_COLORS];  ///< Vettore di dimensione #DIM_KERNEL_COLORS che contiene i colori da assegnare al blob in base alla distanza dal centro.
  unsigned char virtual_blob[VIRTUAL_BLOB_BOX * VIRTUAL_BLOB_BOX];  ///< Maschera del blob virtuale.
  int orig_virtual_blob_rows[NY/binning];  ///< Vettore che contiene gli offset di riga del virtual blob originale in base alla riga nella mappa di disparità in cui inserire il virtual blob riscalato.

  int orig_virtual_blob_col_deltas[VIRTUAL_BLOB_BOX];  ///< Vettore degli offset di colonna per che specificano, per ogni riga del virtual blob, il delta del primo pixel valido rispetto alla colonna centrale.
  int scaled_delta_fc[VIRTUAL_BLOB_BOX];   ///< Vettore degli offset di sinistra aggiornati per il blob virtuale scalato.
  int scaled_delta_lc[VIRTUAL_BLOB_BOX];   ///< Vettore degli offset di destra aggiornati per il blob virtuale scalato.

  // Don't forget to declare these two. You want to make sure they
  // are unaccessable otherwise you may accidently get copies of
  // your singleton appearing.
  OutOfRangeManager(OutOfRangeManager const&);  // Don't Implement
  void operator=(OutOfRangeManager const&);  // Don't implement

  void CopyVirtualBlob(unsigned char* const & disparityMap,const unsigned char* const & BP_map);  ///< Copia il blob virtuale nella mappa
  void CreateVirtualBlob();  ///< Crea la maschera che contiene il blob virtuale
  int ComputeRay();  ///< Calcola il raggio del blob virtuale data la posizione del centroide nella mappa.
  bool CheckOutOfRangeAndUpdate(const int & num_black_pixels,
    const int & num_DSP,
    const int & cent_r,
    const int & cent_c,
    const int & door_threshold,
    const int & num_mask_pixel);  ///< Verifica se vi &egrave; o meno situazione di Out-Of-Range

};

#endif
#endif
