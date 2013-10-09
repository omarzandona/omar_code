/*!
\file OutOfRangeManager.cpp

\brief Contiene l'implementazione dell'algoritmo per la gestione del out-of-range.

Per i dettagli vedi la descrizione della funzione HandleOutOfRange() che viene abilitata solo in 
presenza di un determinato set-up.

\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include "OutOfRangeManager.h"

#ifdef USE_HANDLE_OUT_OF_RANGE

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "blob_tracking.h"
#include "default_parms.h"
#include "peopledetection.h"
#define USE_ADAPTIVE_RAY
#ifdef SHOW_RESULT
// #define SHOW_VIRTUAL_BLOB
#endif

int g_nrows, g_ncols, g_border_x, g_border_y;
static const float DIM_BINNED_IMG = 1610.0f; ///< Dimensione dell'immagine binnata


extern unsigned char handle_oor; // 20130715 eVS, manage OOR after system reboot
extern void print_log(const char *format, ...);

// dichiarazione funzioni locali
void _create_adaptive_ray(int* const & ray_value_vec,
                          // const int ray_value_vec_dim,
                          const int min_ray,
                          const int max_ray);  ///< Crea un vettore che conterr&agrave; le dimensioni del raggio del blob virtuale rispetto al centroide

/*void _check_region(int first_indx,
int last_indx,
unsigned char* disparityMap,
const int max_disp_value); */  ///< Controlla le regioni sopra e sotto il blob viruale per eliminare rumore

/*void _check_side_region(int min_row_indx,
int max_row_indx,
int min_col_indx,
int max_col_indx,
int* const & delta_vec,
unsigned char* disparityMap,
int* const & offset_vec,
unsigned int centroid_col,
const int max_disp_value);*/  ///< Controlla la regione destra e sinistra del blob virtuale per eliminare rumore

void _solve_conflict_between_blob(tPersonTracked** p_inhi,
                                  tPersonTracked** p_inlo,
                                  int indx_max_inhi,
                                  int indx_max_inlo,
                                  bool & isFromHigh,
                                  int & indx_max);  ///< Determina il blob candidato per trasformarlo in quello virtuale

void _find_max_num_black_pixels(tPersonDetected* persone,
                                const int num_pers,
                                const unsigned char* const & map,
                                const unsigned char* const & BP_map,
                                const unsigned char* const & BP_BG,
                                const int & door_threshold,
                                const int & prev_ray,
                                int & num_black_pixels,
                                int & cent_r,
                                int & cent_c,
                                int & num_DSP);


/*!
Conta i pixel neri (valore nella mappa uguale a 0) presenti nel frame
e calcola le coordinate del centroide di cui si tiene conto la storia nel 
caso di out-of-range. 

\param disparityMap [in|out] Mappa di disparit&agrave;
\see isBackgroundCheckInProgress()
*/
int
OutOfRangeManager::CountBlackPixels(unsigned char* const & disparityMap)
{
  // Inizializzo i limiti della finestra per il conteggio dei pixel neri
  int black_pixel_cnt = 0; // Contatore dei pixel neri
  unsigned char*disvec;  // Puntatore mappa di disparit&agrave;

  // Determinazione del numero di pixel neri
  for (int r = BORDER_Y; r <= NY-BORDER_Y-1; ++r)
  {
    disvec = &(disparityMap[r*NX + BORDER_X]); // Mi posizione sulla riga r-esima della mappa di disparit&agrave;
    for (int c = BORDER_X; c <= NX-BORDER_X-1; ++c, ++disvec)
      if (*disvec == 0) // Se il valore della mappa di disparit&agrave; &egrave; 0 allora aumento il contatore
        ++black_pixel_cnt;
  }

  return black_pixel_cnt;
}


/*! 
\brief Controllo sul background per verificare se e' possibile usare HandleOutOfRange().

Controllo del background da fare prima di usare HandleOutOfRange().
Il suo uso e' del tipo:

bool is_backgroung_reliable;
int num_processed_frames = 0;
do {
black_pixel_cnt = CountBlackPixels(dispMap);
num_processed_frames++;
} while (isBackgroundCheckInProgress(black_pixel_cnt, is_background_reliable, num_processed_frames == 1);
*/
bool 
OutOfRangeManager::isBackgroundCheckInProgress(
  const int black_pixel_cnt,  ///< [in] Numero di pixel neri di ogni frame
  bool & is_background_ok,    ///< [in|out] Flag che indica che il background &egrave; buono
  const bool reinit)          ///< [in] Flag che indica che il controllo del background deve essere rifatto e quindi le variabili re-inizializzate
{
  static bool first_time = true;  // E' la prima volta che viene effettuato il controllo
  static bool is_BG_reliable;  // Flag che indica se il background non e' corrotto
  static bool is_check_finished;  // Flag che indica che il controllo e' finito
  static int count_frames;  // Numero di frame
  static int count_bad_frames;  // Numero di frame corrotti
  static int last_black_pixel_cnt;  // Numero di frame corrotti

  if (reinit || first_time)  // Re inizializzo in caso di nuovo controllo
  {
    first_time = false;
    is_BG_reliable = true;
    is_check_finished = false;
    count_frames = 0;
    count_bad_frames = 0;
    last_black_pixel_cnt = 0;
  }

  if (!is_check_finished)  // if the check is not finished yet...
  {
    // Se il numero dei pixel neri &egrave; maggiore della met&agrave; del totale sono in presenza di un background nero
    // gestisco la situazione disabilitando la gestione dell'out-of-range
    const int check_interval = 80;  // se questo valore viene cambiato, dovrebbe cambiare la Sleep nel winclient in attesa del risultato di "check_bg"
    const int min_bad_frames_num = check_interval / 4;  // se il 25% dei frame sono fuori soglia disabilito la gestione dell'out-of-range

    if (count_frames >= check_interval || !is_BG_reliable)
    {
      // check is finished when the BG is discovered to be not realiable or when it is realiable for check_interval frames
      is_check_finished = true;
      print_log("is_BG_reliable = %s = (%d >= %d || %d >= %d)\n", 
        (is_BG_reliable) ? "true" : "false", 
        count_bad_frames,
        min_bad_frames_num,
        last_black_pixel_cnt,
        OutOfRangeManager::NUM_BLACK_PIXEL_TO_DISABLE);
    }
    else  // if (count_frames < check_interval && is_BG_reliable)
    {
      last_black_pixel_cnt = black_pixel_cnt;
      const int min_bad_pixels_num = MAX_BLACK_PIXELS_NUM_TO_BE_REALIABLE;
      if (black_pixel_cnt >= min_bad_pixels_num)
      {
#ifdef _DEBUG
        printf("black_pixel_cnt >= min_bad_pixels_num\n");
#endif
        count_bad_frames++;
        if (count_bad_frames >= min_bad_frames_num || black_pixel_cnt >= NUM_BLACK_PIXEL_TO_DISABLE)
        {
          is_BG_reliable = false;
        }
      }
      count_frames++;  // this increment guarantees that sooner or later count_frames will be greater than check_interval when is_BG_reliable is still true
    }
  }

  is_background_ok = is_BG_reliable;

  return !is_check_finished;
}

#ifdef USE_HANDLE_OUT_OF_RANGE
static int weight[NY];  ///< pesi usati in _count_black_pixels_around() e inizializzati con init_out_of_range_centroid_weights()
static int weight_dsp[256];  ///< pesi usati in _count_black_pixels_around() e inizializzati con init_out_of_range_centroid_weights()

/*!
\brief Inizializza i pesi (variabili globali #weights e #weights_DSP) usati nel calcolo del centroide.

Questa funzione dovrebbe essere lanciata dal costruttore di OutOfRangeManager in modo
che i calcoli per il riempimento dei vettori non facciano perdere frame o causino
problemi di sincronia con l'FPGA. Ma per sicurezza anche la _count_black_pixels_around()
chiama questa funzione che, grazie alla flag statica first_time si limitera' a fare un 
solo if.
*/
int
init_out_of_range_centroid_weights()
{
  const char MIN_DISP_FOR_OUT_OF_RANGE_CHECK = 6;  ///< Minima disparita' oltre la quale effettuare il check di out-of-range.
  const char MAX_DISP = 15;  ///< Massima disparita' che compare che posso ottenere dalla mappa dividendo per 16

  // la map ha disparita' multiple di 16
  const int min_dsp = MIN_DISP_FOR_OUT_OF_RANGE_CHECK*16;
  const int max_dsp_in_map = MAX_DISP*16;
  static bool first_time = true;
  if (first_time)
  {
    first_time = false;

    printf("init_out_of_range_centroid_weights(): inizializzazione pesi.\n");

    // inizializza i vettori dei pesi allocati staticamente
    const int delta_r_max = (g_nrows-2*g_border_y)/2;
    for (int r=0; r<g_nrows; ++r)
      weight[r] = 768 + (256*abs(r-g_nrows/2)) / delta_r_max;

    int m = (1024*512)/(max_dsp_in_map-min_dsp);
    int q = 1024*512 - m*max_dsp_in_map;
    for (int dsp=min_dsp; dsp<=max_dsp_in_map; ++dsp)
      weight_dsp[dsp] = 512 + (m*dsp + q)/1024;

    // per sicurezza copio valore weight_dsp[min_dsp] in tutti gli elementi sotto a min_dsp
    for (int dsp=0; dsp<min_dsp; ++dsp)
      weight_dsp[dsp] = weight_dsp[min_dsp];

    // per sicurezza copio valore weight_dsp[max_dsp_in_map] in tutti gli elementi sopra a max_dsp_in_map
    for (int dsp=max_dsp_in_map; dsp<256; ++dsp)
      weight_dsp[dsp] = weight_dsp[max_dsp_in_map];
  }

  return min_dsp;
}


/*!
\brief Effettua il conteggio dei pixel neri attorno al blob passato come parametro

La funzione scandisce la mapp&agrave; e calcola il numero di pixel neri, pesando il conteggio
e il calcolo delle coordinate del cenrtroide dei pixel neri, in base alla posizione rispetto
al centroide. Avremmo cosi un peso maggiore nelle vicinanze del centroide del blob e inferiore
man mano che ci allontaniamo da quest'ultimo.
Se il numero di pixel neri &egrave; minore di un terzo dell'area in cui si effettua la ricerca,
mi calcolo anche il centroide dovute alle disparit&agrave; "interessanti" e calcolo il centroide come media tra il centroide dei pixel neri
e il centroide dovuto alle disparit&agrave;

\return Il numero di pixel neri
*/
int
_count_black_pixels_around(
                           const tPersonDetected & p,            ///< [in|out] Persona(Blob) da considerare
                           const int border_x,                   ///< [in] Bordo dei pixel neri nelle colonne
                           const int border_y,                   ///< [in] Bordo dei pixel neri nelle righe
                           const unsigned char* const & map,     ///< [in] Mappa di disparit&agrave;
                           const unsigned char* const & BP_map,  ///< [in] Mappa che contiene i contatori per ogni frame
                           const unsigned char* const & BP_BG,   ///< [in] Maschera dei pixel a zero (pari a 255 se 0)
                           const int nrows,                      ///< [in] Numero dei righe
                           const int ncols,                      ///< [in] Numero di colonne
                           const int & prev_ray,                 ///< [in] Raggio precedente
                           int & cent_r,                         ///< [out] Coordinata X del centroide
                           int & cent_c,                         ///< [out] Coordinata Y del centroide
                           int & num_DSP)                        ///< [out] Numero di disparit&agrave; rilevanti del candidato blob virtuale
{
  const int FROM_DISP_TO_HEAD_RAY = FROM_DISP_TO_HEAD*2;
  const int min_out_of_range_dsp = 21;  // TODO qui ci andrebbe l'elemento 15 del vettore dei dati di calibrazione
  const int max_dist_orig = (16*min_out_of_range_dsp + FROM_DISP_TO_HEAD_RAY/2) / FROM_DISP_TO_HEAD_RAY;  // i pixel out-of-range sono principalmente sulla testa
  int max_dist = max_dist_orig/binning;

  if (prev_ray > 0)
    max_dist = max(max_dist, prev_ray);

  const int min_dsp = init_out_of_range_centroid_weights();
  const int max_dist2 = max_dist*max_dist;
  //const int max_area = (max_dist2 * 314)/100;  // area da considerare

  // mi sposto verso il basso di 30cm per tenere conto (in modo approssimativo) della prospettiva
  // di una quantità proporzionale all'altezza della persona
  int new_x = (p.x-BORDER_X)/binning;
  int new_y = (p.y-BORDER_Y)/binning;

  int first_r = max(border_y, new_y-max_dist);
  int last_r = min(nrows-border_y-1, new_y+max_dist);

  int first_c = max(border_x, new_x-max_dist);
  int last_c = min(ncols-border_x-1, new_x+max_dist);

  // calcolo pixel neri e relativo centroide pesando rispetto ai bordi in alto e in basso
  int cent_r_BP = 0;
  int cent_c_BP = 0;

  int num_BP = 0;
  int sum_weight_BP = 0;

  //const int delta_r_max = (g_nrows-2*g_border_y)/2;
  for (int r = first_r; r <= last_r; ++r)
  {
    const unsigned char* ptr_BP_map = &(BP_map[r*ncols+first_c]);
    const unsigned char* ptr_BP_BG = &(BP_BG[r*ncols+first_c]);
    const int diffy = (p.y-BORDER_Y)/binning-r;
    const int weight_r = weight[r];
    for (int c = first_c; c <= last_c ; ++c, ++ptr_BP_map, ++ptr_BP_BG)
    {
      if (*ptr_BP_map == 255)
      {
        if (*ptr_BP_BG == 0)
        {
          int diffx = (p.x-BORDER_X)/binning-c;
          int dist2 = diffx*diffx + diffy*diffy;

          if (dist2 <= max_dist2)
          {         
#ifdef _DEBUG
            const int delta_r_max = (g_nrows-2*g_border_y)/2;
            int w = 768 + (256*abs(r-g_nrows/2)) / delta_r_max;
            assert(w == weight[r]);
#endif
            cent_r_BP += weight_r * r;
            cent_c_BP += weight_r * c;
            sum_weight_BP += weight_r;

            num_BP++;
          }
        }
      }
    }
  }

  if (num_BP > 0)
  {
    if (sum_weight_BP > 0)
    {
      cent_r_BP /= sum_weight_BP;
      cent_c_BP /= sum_weight_BP;
    }
    else
    {
      //print_log("_count_black_pixels_around(): (sum_weight_BP > 0) failed.\n");
      cent_r_BP = -1;
      cent_c_BP = -1;
      num_BP = 0;
    }
  }

#if defined(_DEBUG) && defined(SHOW_RESULT)
  printf("_count_black_pixels_around = (c1=%d, r1=%d, BP=%d", 
    cent_c_BP, cent_r_BP,
    num_BP);
#endif

  //if (num_BP >= max_area/2)
  //{
  //  cent_r = cent_r_BP;
  //  cent_c = cent_c_BP;
  //}
  //else if (num_BP > 0) // TODO qui andrebbe la soglia usata in HandleOutOfRange per andare in OFF (cosi' faccio i calcoli solo quando servono)
  {
    // calcolo numero disparita' "interessanti" e relativo centroide pesando rispetto alla disparita'
    int cent_r_DSP = 0;
    int cent_c_DSP = 0;
    num_DSP = 0;
    int sum_weight_DSP = 0;
    for (int r = first_r; r <= last_r; ++r)
    {
      const unsigned char* ptr = &(map[r*ncols+first_c]);
      const int diffy = (p.y-BORDER_Y)/binning-r;
      for (int c = first_c; c <= last_c ; ++c, ++ptr)
      {
        if (*ptr >= min_dsp)
        {
          int diffx = (p.x-BORDER_X)/binning-c;
          int dist2 = diffx*diffx + diffy*diffy;

          if (dist2 <= max_dist2)
          {
#ifdef _DEBUG
            const int max_dsp_in_map = 15*16;  // la map ha disparità multiple di 16
            assert(*ptr <= max_dsp_in_map);
            int m = (1024*512)/(max_dsp_in_map-min_dsp);
            int q = 1024*512 - m*max_dsp_in_map;
            int w = 512 + (m*(*ptr) + q)/1024;
            assert(w == weight_dsp[*ptr]);
#endif
            cent_r_DSP += weight_dsp[*ptr]*r;
            cent_c_DSP += weight_dsp[*ptr]*c;
            sum_weight_DSP += weight_dsp[*ptr];

            num_DSP++;
          }
        }
      }
    }

    if (num_DSP > 0)
    {
      if (sum_weight_DSP > 0)
      {
        cent_r_DSP /= sum_weight_DSP;
        cent_c_DSP /= sum_weight_DSP;
      }
      else
      {
        //print_log("_count_black_pixels_around(): (sum_weight_DSP > 0) failed.\n");
        cent_r_DSP = -1;
        cent_c_DSP = -1;
        num_DSP = 0;
      }
    }

    // calcolo centroide come media tra il centroide dei pixel neri e il centroide dovuto alle disparita' "interessanti"
    int tot = num_BP+num_DSP;
    if (tot > 0)
    {
      cent_r = (num_DSP*cent_r_DSP + num_BP*cent_r_BP)/tot;
      cent_c = (num_DSP*cent_c_DSP + num_BP*cent_c_BP)/tot;
    }
    else
    {
      //print_log("_count_black_pixels_around(): (tot > 0) failed.\n");
      cent_r = -1;
      cent_c = -1;
      num_BP = 0;
    }

#if defined(_DEBUG) && defined(SHOW_RESULT)
    printf(", c2=%d, r2=%d, nDSP=%d", 
      cent_c_DSP, cent_r_DSP,
      num_DSP);
#endif
  }

#if defined(_DEBUG) && defined(SHOW_RESULT)
  printf(")\n");
#endif

  return num_BP;
}


/*!
\brief Per ogni persona della lista passata si contano i pixel neri vicini a essa se il blob si trova nelle condizioni
adatte per valutare l'out-of-range.

Il conteggio viene effettuato dalla funzione #_count_black_pixels_around().
In base al risultato del conteggio si effettuano queste operazioni:
A)- Se non trovo nessun pixel nero ma avevo trovato qualcosa in precedenza abbasso 
il numero di black pixel nello storico mediando con zero lasciando lo stesso centroide.
Se torno a zero re-inizializzo
B)- Se invece ci sono dei pixel in out-of-range aggiorno lo storico mediando con i valori calcolato

In caso in cui il blob non si trovi nella condizioni adatte per valutare l'out-of-range re-inizializzo
i campi dello storico interessati.
*/
void
_find_max_num_black_pixels(
                           tPersonDetected* persone,             ///< [in] Lista da aggiornare
                           const int num_pers,                   ///< [in] Numero di elementi della lista
                           const unsigned char* const & map,     ///< [in] Mappa di disparit&agrave;
                           const unsigned char* const & BP_map,  ///< [in] Mappa che contiene i contatori di colore per ogni pixel
                           const unsigned char* const & BP_BG,   ///< [in] Maschera dei pixel a zero (pari a 255 se 0)
                           const int & door_threshold,           ///< [in|out] Soglia porta
                           const int & prev_ray,                 ///< [in] Raggio al frame precedente
                           int & num_black_pixels,               ///< [in] Numero di pixel neri
                           int & cent_r,                         ///< [in] Coordinata Y del centroide
                           int & cent_c,                         ///< [in] Coordinata X del centroide
                           int & num_DSP)                        ///< [in] Numero di disparit&agrave; rilevanti
{
  assert(num_pers > 0);

  bool is_virtual_blob_found = false;
  num_black_pixels = 0;
  cent_r = -1;
  cent_c = -1;
  for (int i=0; i<num_pers && !is_virtual_blob_found; ++i)
  {
    if (persone[i].h > 0)
    {
      const char MIN_DISP_FOR_OUT_OF_RANGE_CHECK = 10;  ///< Minima disparita' oltre la quale effettuare il check di out-of-range.
      const int min_h = MIN_DISP_FOR_OUT_OF_RANGE_CHECK*8; // la h nel repository e' a 7 bit cioe' da 0 a 127 quindi moltiplico per 8 e non per 16
      if (persone[i].h >= min_h)
      {
        // se sono un blob nelle condizioni adatte per valutare l'out-of-range allora...
        int current_cent_r, current_cent_c, current_num_DSP;
        int current_bp = _count_black_pixels_around(persone[i], g_border_x, g_border_y, map, BP_map, BP_BG, g_nrows, g_ncols, prev_ray, current_cent_r, current_cent_c, current_num_DSP);

        if (persone[i].h > 120)
        {
          num_black_pixels = current_bp;
          num_DSP = current_num_DSP;
          cent_r = current_cent_r;
          cent_c = current_cent_c;
          is_virtual_blob_found = true;
        }
        else
        {
          if (current_bp > num_black_pixels) // se non trovo nulla
          {
            num_black_pixels = current_bp;
            num_DSP = current_num_DSP;
            cent_r = current_cent_r;
            cent_c = current_cent_c;
          }
        }
      }
    }
  }
}
#endif


/*! 
Viene innanzitutto gestito il caso in cui il blob &egrave; statico a causa di rumore o per altre cause.
In questo caso disabilito la gestione dell'out-of-range per 10 secondi.
In caso contrario e ee la variabile #m_enable_handle_out_of_range &egrave true, la gestione dell'out-of-range &egrave; abilitata.
Si cerca il blob canditato per renderlo virtuale mediante la funzione #_find_max_num_black_pixels()e viene valutata la presenza o meno di una situazione di out-of-range grazie alla funzione #CheckOutOfRangeAndUpdate().
In caso affermativo, si procede con la copia del blob virtuale nella mappa tramite la funzione #CopyVirtualBlob().
Durante la gestione viene chiamata la funzione #ComputeRay() per determinare la dimensione del raggio del blob.
*/

void
OutOfRangeManager::HandleOutOfRange(
                                    tPersonDetected* const & persone,     ///< [in|out] Lista delle persone detected al passo precedente
                                    const int num_pers,                   ///< [in] Dimensione della lista delle persone detected
                                    unsigned char* const & disparityMap,  ///< [in|out] Mappa di disparit&agrave;
                                    const unsigned char* const & BP_map,  ///< [in] Maschera dei pixel a zero (pari a 255 se 0) del frame corrente
                                    const unsigned char* const & BP_BG,   ///< [in] Maschera dei pixel a zero (pari a 255 se 0) nel BG
                                    const bool motion,                    ///< [in] True se c'&egrave; movimento o se motion detection disdabilitato nella scena false altrimenti
                                    const int door_threshold,             ///< [in] Posizione della porta
                                    const int num_mask_pixel)             ///< [in] Numero di black pixels dato dal modellazione di quest'ultimo da parte della classe #BPMOdeling
{
#ifdef USE_STATIC_BLOB_CHECK
  // Gestione della staticita' del blob virtuale
  static int num_frame_to_wait = 0;  // contatore del numero di frame da aspettare per riattivare la gestione dell'OOR
  static const int FRAME_TO_WAIT_TO_RE_ENABLE_OOR_MANAGER = 10*54;  // 30 *54 = 1620 frame == 30 secondi
  if (m_blob_static &&
    num_frame_to_wait < FRAME_TO_WAIT_TO_RE_ENABLE_OOR_MANAGER)
  {
    ++num_frame_to_wait; // se sono qui il frame &egrave; statico e la gestione e' disattivata: incremento il contatore
  }
  else
  {
    if(num_frame_to_wait >= FRAME_TO_WAIT_TO_RE_ENABLE_OOR_MANAGER)  // Ho raggiunto il numero di frame richiesto per riabilitare l'OOR
    {
      m_blob_static = false;
      SetEnableStateOutOfRange(true);
      num_frame_to_wait = 0;
    }
  }
#endif

 

  if (m_enable_handle_out_of_range && num_pers > 0)
  {
    // In caso di due blob proveniente da direzioni diverse si tiene il blob con il maggior numero di pixel neri
    bool correct_background;

#ifndef DISABLE_BACKGROUND_CHECK
    isBackgroundCheckInProgress(black_pixel_cnt, correct_background);
#else
    correct_background = true;
#endif

    if (correct_background)
    {
      // Controllo che ci sia movimento nella scena
      static int no_motion_nframes = 0;
      const int max_nframes = 27;
      if (!motion)
      {
        if (no_motion_nframes < max_nframes)
          no_motion_nframes++;
      }
      else
        no_motion_nframes = 0;

      if (no_motion_nframes < max_nframes)
      {
        int num_black_pixels, cent_r, cent_c, num_DSP;

        // Aggiorno il repository
        _find_max_num_black_pixels(persone,
          num_pers,
          disparityMap,
          BP_map,
          BP_BG,
          door_threshold,
          m_ray,
          num_black_pixels,
          cent_r,
          cent_c,
          num_DSP);

        // Se sono in out-of-range gestisco la situazione creando un blob virtuale
        if (CheckOutOfRangeAndUpdate(num_black_pixels, num_DSP, cent_r, cent_c, door_threshold,num_mask_pixel))
        {
          // Copio il blob virtuale all'interno della mappa
          CopyVirtualBlob(disparityMap,BP_map);
        }
      }
    }
  }
  else
  {
    m_is_out_of_range = false;
    m_num_black_pixels = 0;
    m_num_DSP = 0;
    m_cent_r = -1;
    m_cent_c = -1;
    m_ray = -1;
  }
}


/*!
Crea la maschera contenente il blob virtuale. Inoltre calcola i vettori di offset per
gestire la corretta copia del blob virtuale nella mappa.
*/
void OutOfRangeManager::CreateVirtualBlob()
{
  static bool first_time = true;  // Permette di creare il blob virtuale una volta sola. Sara' mantenuta poi in memoria

  assert(DISP_VALUE_FOR_VIRTUAL_BLOB_BORDER != OUT_OF_RANGE_OR_STEREO_FAILURE && 
    DISP_VALUE_FOR_VIRTUAL_BLOB_BORDER != UNIFORM_ZONE_OR_DISP_1 &&  // altrimenti i bordi scuri appositamente aggiunti vengono poi eliminati dal procedimento corrispondente alla direttiva USE_BINNING_WITH_CHECK
    DISP_VALUE_FOR_VIRTUAL_BLOB_BORDER < MIN_M);  // altrimenti viene coinvolto nel calcolo dei picchi

  if (first_time)
  {
    printf("CreateVirtualBlob(): inizializzazione blob virtuale.\n");

    // Creo il vettore dei colori del blob tramite una gaussiana
    first_time = false;
    const float sigma = 120.0f/binning;
    const float var = sigma*sigma;

    for (int i = 0; i < VIRTUAL_BLOB_RAY; ++i)
      kernel_color_weights[i] = (unsigned int) (255*exp(-(i*i)/var));
    for (int i = VIRTUAL_BLOB_RAY; i < DIM_KERNEL_COLORS; ++i)
      kernel_color_weights[i] = DISP_VALUE_FOR_VIRTUAL_BLOB_BORDER;

    // Creo il blob virtuale di base con raggio OutOfRangeManager::MAX_RAY
    unsigned char* ptr_virtual_blob = virtual_blob;
    memset(virtual_blob, 0, sizeof(virtual_blob));
    for (int r = 0; r < VIRTUAL_BLOB_BOX; ++r)
    {
      bool first_found = false;  // Serve per determinare l'offset
      for (int c = 0; c < VIRTUAL_BLOB_BOX; ++c, ++ptr_virtual_blob)
      {
        unsigned int tmp_i = (r - MAX_RAY)*(r - MAX_RAY);
        unsigned int tmp_j = (c - MAX_RAY)*(c - MAX_RAY);
        unsigned int dist = tmp_i + tmp_j;  // distanza dal centroide..il valore indicizza il colore nel kernel
        if (dist <= MAX_RAY*MAX_RAY)
        {
          if (!first_found)  // salvo l'offset
          {
            first_found = true;
            orig_virtual_blob_col_deltas[r] = c - MAX_RAY;
          }
          int indx = max(0, min(DIM_KERNEL_COLORS-1, (int)(sqrt(float(dist)) + 0.5f)));
          *ptr_virtual_blob = kernel_color_weights[indx];  // Assegno il colore al blob virtuale
        }
      }
    }
  }

#ifdef SHOW_VIRTUAL_BLOB
  cvNamedWindow("Virtual_blob", 1);
  IplImage* tmp_virtual_blob = cvCreateImage(cvSize(VIRTUAL_BLOB_BOX, VIRTUAL_BLOB_BOX), IPL_DEPTH_8U, 1);
  cvSetData(tmp_virtual_blob, virtual_blob, VIRTUAL_BLOB_BOX);
  cvShowImage("Virtual_blob", tmp_virtual_blob);
  cvReleaseImageHeader(&tmp_virtual_blob);
#endif
}


/*!
Controlla se si verifica la presenza di una situazione di out-of-range.
In particolare se il numero di pixel neri #num_black_pixels &egrave; maggiore della soglia #NUM_BLACK_PIXEL_ON (di cui se ne considera una percentuale in
base al numero di pixel neri modellati dall'oggetto BPmodeling).
Successivamente viene valutata la possibilit&agrave; che il blob virtuale sia statico considerando il movimento del suo centroide e il numero di pixel neri trovati.
In caso affermativo incremento il contatore statico ad ogni frame.
Se il contatore dei frame supera la soglia MAX_NUM_FRAME_FOR_STATIC_BLOB setto la soglia #m_blob_static a TRUE che disabiliter&agrave la gestione dell OOR.

Se invece il numero dei pixel neri e sopra la soglia, il blob non &egrave; statico e il bkg &egrave; corretto e il centroide calcolato non
risiede nella zona vicina alla soglia della porta allora viene gestito l'OOR inserendo i nuovi campi nel caso in cui
scatta l'OOR . 
Se ho un numero di pixel neri maggiori della soglia e ero già in una situazione di OOR aggiornando 
i campi dell'ogetto pesando i nuovi valori che lo caratterizzano 3 volte in pi&ugrave; rispetto alla storia dei valori precedente.

Infine se la soglia dei pixel neri &egrave; minore della soglia #NUM_BLACK_PIXEL_OFF ed ero in OOR re-inizializzo tutti i contatori altrimenti mantengo attivo il blob virtuale per garantire
l'uscita di scena ( In sostanza si tratta di un'isteresi che ci permette di evitare una situazione di continua attivazione/spegnimento del blob virtuale)

\return true se c&egrave; una situazione di out-of-range, false altrimenti
*/
bool
OutOfRangeManager::CheckOutOfRangeAndUpdate(
  const int & num_black_pixels,  ///< [in] Numero di black pixel della mappa di disparit&agrave;
  const int & num_DSP,           ///< [in] Numero di disparit&agrave; rilevanti
  const int & cent_r,           ///< [in] Coordinata Y del centroide
  const int & cent_c,           ///< [in] Coordinata X del centroide
  const int & door_threshold,    ///< [in] Valore della soglia della porta
  const int & num_mask_pixel)    ///< [in] Numero di pixel neri della maschera
{
  static bool just_turned_on = false;  // flag che indica la prima attivazione di gestione OOR
#ifdef USE_STATIC_BLOB_CHECK
  static int num_frame_blob_static = 0;  // contatore del numero di frame in cui il blob si presume statico
#endif 

  assert(num_mask_pixel <= DIM_BINNED_IMG);  // controllo paranoico
  float perc_mask_bp = (float)(1-(num_mask_pixel/DIM_BINNED_IMG));  // percentuale di pixel neri nella maschera calcolata dal modello dei BP
  int current_threshold_on = (int)(NUM_BLACK_PIXEL_ON * perc_mask_bp);  // soglia corrente, varia in base alla perc_mask_bp

#ifdef _DEBUG
 /* printf(" num_mask_pixel: %d \n",num_mask_pixel);
  printf(" perc_mask_bp : %d \n",int(perc_mask_bp*100));
  printf(" th_current: %d \n",current_threshold_on);*/
#endif

  // Controllo se sono sopra la soglia che mi indica con certezza che sono in una situazione di out-of-range
  if (num_black_pixels > current_threshold_on && int(perc_mask_bp*100 > 50))
  {

    // Ulteriore controllo per gestire il caso in cui il blob &egrave; fermo anche se c&egrave; movimento nella scena : rumore statico:
    // Il centroide di muove solo in un intorno limitato (+-1) e il numero di black pixel &egrave; costante a meno di una certa percentuale
#ifdef USE_STATIC_BLOB_CHECK

    if ( (cent_c <= (m_cent_c + 1) && cent_c >= (m_cent_c - 1)) &&
      (cent_r <= (m_cent_r + 1) && cent_r >= (m_cent_r - 1))
      && ((num_black_pixels <= 120*m_num_black_pixels/100) && (num_black_pixels >= 80*m_num_black_pixels/100))
      && m_is_out_of_range)
    {
      ++num_frame_blob_static;  // in caso di blob statico aumento il contatore
# ifdef _DEBUG
     /* printf("Blob statico da : %d, frame!! \n", num_frame_blob_static);
      printf("Diff colonna: %d, Diff riga: %d\n", abs(cent_c - m_cent_c),abs(cent_r - m_cent_r));
      printf("Diff numero black pixel: %d\n", abs(m_num_black_pixels - num_black_pixels));*/
# endif
    }
    else
      num_frame_blob_static = 0;  // azzero il contatore. TO-DO: Si potrebbe solo diminuire il contatore di un TOT oppure usare la stessa tecnica del modello BP

    if (num_frame_blob_static <= MAX_NUM_FRAME_FOR_STATIC_BLOB) // se il blob non &egrave; statico gestisco l'OOR
    {
#endif

      if (!m_is_out_of_range)
      {
        if (((m_cent_r == -1 || m_cent_r < door_threshold-DELTA_DOOR_TH/binning) && cent_r < door_threshold-DELTA_DOOR_TH/binning) || 
          ((m_cent_r == -1 || m_cent_r > door_threshold+DELTA_DOOR_TH/binning) && cent_r > door_threshold+DELTA_DOOR_TH/binning))
        {
#ifdef _DEBUG
          printf("OutOfRangeManaging ON\n");
#endif
          just_turned_on = true;
          m_is_out_of_range = true;  // Setto il flag della situazione di out-of-range a true
          m_is_from_high = (cent_r < door_threshold-DELTA_DOOR_TH/binning);
          m_num_black_pixels = num_black_pixels;
          m_num_DSP = num_DSP;
          m_cent_r = cent_r;
          m_cent_c = cent_c;
          m_ray = ComputeRay();  // questa riga va dopo le precedenti
        }
      }
#ifdef USE_STATIC_BLOB_CHECK
    }
    else
    {
      //Sono qui perch&egrave; ho rilevato una situazione di staticità del blob...
      SetEnableStateOutOfRange(false);
      num_frame_blob_static = 0;
      m_blob_static = true;  // Setto a true la flag che mi indica condizioni di staticit&agrave; del blob virtuale
    }
#endif
  }
  else
  {
    if (m_is_out_of_range)  // sono in out-of-range
    {
      if (m_num_black_pixels < (int)(NUM_BLACK_PIXEL_OFF * perc_mask_bp) ||  // se la soglia &egrave; sotto a soglia di OFF
        (m_is_from_high && m_cent_r >= 0 && m_cent_r > door_threshold + DELTA_DOOR_TH/binning && m_num_black_pixels < current_threshold_on) ||  // se la soglia &egrave; sotto a soglia di ON e il blob virtuale &egrave; passato da sopra a sotto
        (!m_is_from_high && m_cent_r >= 0 && m_cent_r < door_threshold - DELTA_DOOR_TH/binning && m_num_black_pixels < current_threshold_on))  // se la soglia &egrave; sotto a soglia di ON e il blob virtuale &egrave; passato da sotto a sopra
      {
        m_is_out_of_range = false;  // Non sono piu' nella situazione di out-of-range
        m_num_black_pixels = 0;
        m_num_DSP = 0;
        m_cent_r = -1;
        m_cent_c = -1;
        m_ray = -1;
#ifdef USE_STATIC_BLOB_CHECK
        num_frame_blob_static = 0;
#endif
#ifdef _DEBUG
        printf("OutOfRangeManaging OFF\n");
#endif
      }
    }
  }

  // Aggiorno i valori
  if (m_is_out_of_range && !just_turned_on && !m_blob_static)
  {
    m_num_black_pixels = (m_num_black_pixels+3*num_black_pixels)/4;
    m_num_DSP = (m_num_DSP+3*num_DSP)/4;
    m_cent_r = (m_cent_r+3*cent_r)/4;
    m_cent_c = (m_cent_c+3*cent_c)/4;
    m_ray = ComputeRay();  // questa riga va dopo le precedenti
  }

  just_turned_on = false;

  return m_is_out_of_range;
}


/*!
Disegna nella mappa di disparit&agrave; un blob virtuale per gestire la situazione out-of-range.

\param disparityMap [in|out] Mappa di disparit&agrave;
\param BP_Map       [in] Maschera dei pixel neri

*/
void
OutOfRangeManager::CopyVirtualBlob(unsigned char* const & disparityMap , const unsigned char* const & BP_Map)
{
  // controllo integrita' dei valori usati
  const int ray = m_ray;
  const int cent_r = m_cent_r;
  const int cent_c = m_cent_c;
  assert(cent_r >= g_border_y && cent_c >= g_border_x && m_ray > 0);

  // Calcolo il raggio del blob virtuale in base alla posizione del centroide nella mappa
  const int max_real_col = g_ncols-g_border_x-1;
  const int max_real_row = g_nrows-g_border_y-1;

  // Calcolo le righe di partenza e di arrivo per la copia del blob
  const int min_row = max(cent_r - ray, g_border_y);
  const int max_row = min(cent_r + ray, max_real_row);

  // Calcolo gli offset
  for (int r = min_row; r <= max_row; ++r)
  {
    const int scaled_row_delta = (r - cent_r);  // differenza sulle righe del blob di raggio ray che voglio disegnare
    const int orig_virtual_blob_delta = (OutOfRangeManager::MAX_RAY * scaled_row_delta) / ray;
    orig_virtual_blob_rows[r] = OutOfRangeManager::MAX_RAY + orig_virtual_blob_delta;  // scalo la differenza in modo da accedere al blob originale
    orig_virtual_blob_rows[r] = max(0, min(OutOfRangeManager::VIRTUAL_BLOB_BOX-1, orig_virtual_blob_rows[r]));  // controllo necessario a causa di possibili errori di arrotondamenti
    const int & virtual_blob_row = orig_virtual_blob_rows[r];  // creo alias per agevolare scrittura

    // Procediamo con lo scalare il orig_virtual_blob_col_deltas in modo coerente al nuovo raggio ray
    int scaled_delta = (OutOfRangeManager::orig_virtual_blob_col_deltas[virtual_blob_row] * ray) / OutOfRangeManager::MAX_RAY;

    // Offset di sinistra
    int local_delta_fc = scaled_delta;  // si noti che OutOfRangeManager::orig_virtual_blob_col_deltas sono negativi
    int c_left = cent_c + local_delta_fc;
    if (c_left < g_border_x)  // Controllo che gli offset siano corretti e in caso contrario saturo il loro valore al minimo disponibile
    {
      int delta_x = (c_left - g_border_x);
      local_delta_fc = local_delta_fc - delta_x;
    }
    scaled_delta_fc[virtual_blob_row] = local_delta_fc;  // Aggiorno l'offset di sinistra

    // Offset di destra
    int local_delta_lc = -scaled_delta;  // il meno e' perche' OutOfRangeManager::orig_virtual_blob_col_deltas sono negativi
    int c_right = cent_c + local_delta_lc;
    if (c_right > max_real_col)  // Controllo che gli offset siano corretti e in caso contrario saturo il loro valore al massimo disponibile
    {
      int delta_x = (c_right - (g_ncols - g_border_x) +1);
      local_delta_lc = local_delta_lc - delta_x;
    }
    scaled_delta_lc[virtual_blob_row] = local_delta_lc;  // Aggiorno l'offset di destra

    // Verifico che gli indici siano corretti
    assert(cent_c+scaled_delta_fc[virtual_blob_row] >= 0 && cent_c+scaled_delta_fc[virtual_blob_row] < g_ncols);
    assert(orig_virtual_blob_rows[r] >=0 && orig_virtual_blob_rows[r] <= OutOfRangeManager::VIRTUAL_BLOB_BOX-1);
  }

  //{
  //  // Controllo la regione sovrastante il blob per eliminare rumore
  //_check_region(g_border_y,
  //  min_row,
  //  disparityMap,
  //  OutOfRangeManager::MAX_DISP_VALUE);

  // Controllo la regione a sinistra del blob per eliminare rumore
  //_check_side_region(min_row,
  //  max_row,
  //  g_border_x,
  //  cent_c,
  //  scaled_delta_fc,
  //  disparityMap,
  //  orig_virtual_blob_rows,
  //  cent_c,
  //  OutOfRangeManager::MAX_DISP_VALUE);
  //}

  // Copio il blob virtuale che avra' una dimensione in base alla posizione del centroide
  int min_dsp = init_out_of_range_centroid_weights();
  for (int r = min_row; r <= max_row; ++r)
  {
    const int & virtual_blob_row = orig_virtual_blob_rows[r];  // l'offset di riga e' stato gia' controllato essere tra 0 e VIRTUAL_BLOB_BOX-1
    int row_offset = virtual_blob_row * OutOfRangeManager::VIRTUAL_BLOB_BOX;  // offset in base a riga per accesso al virtual blob originale
    int disp_col = cent_c + scaled_delta_fc[virtual_blob_row];  // colonna nella mappa di disparità dove copiare il virtual blob riscalato (questo offset e' gia' stato controllato creando scaled_delta_fc)
    unsigned char* disp_map_ptr = &disparityMap[r * g_ncols + disp_col];
    const unsigned char* BP_Map_ptr = &BP_Map[r * g_ncols + disp_col];

    // il codice che segue e' troppo pesante sul PCN
    for (int delta_c = scaled_delta_fc[virtual_blob_row]; delta_c <= scaled_delta_lc[virtual_blob_row]; ++delta_c, ++disp_map_ptr, ++BP_Map_ptr)
    {
      int virtual_blob_c = OutOfRangeManager::MAX_RAY + (delta_c * OutOfRangeManager::MAX_RAY) / ray;
      virtual_blob_c = max(0, min(OutOfRangeManager::VIRTUAL_BLOB_BOX-1, virtual_blob_c));  // mi assicuro che l'offset di colonna sia tra 0 e VIRTUAL_BLOB_BOX-1 (causa arrotondamenti potrebbe non essere cosi')
      const int idx = row_offset + virtual_blob_c;
      assert(idx >= 0 && idx < OutOfRangeManager::VIRTUAL_BLOB_BOX*OutOfRangeManager::VIRTUAL_BLOB_BOX);  // paranoid check
      int w = (int) (*disp_map_ptr >= min_dsp || *BP_Map_ptr != 0);
      *disp_map_ptr = w*OutOfRangeManager::virtual_blob[idx] + (1-w)*(*disp_map_ptr);
    }
  }

  //{
  //  // Controllo la regione a destra del blob per eliminare rumore
  //_check_side_region(min_row,
  //  max_row,
  //  cent_c,
  //  max_real_col,
  //  scaled_delta_lc,
  //  disparityMap,
  //  orig_virtual_blob_rows,
  //  cent_c,
  //  OutOfRangeManager::MAX_DISP_VALUE);

  // Controllo la regione sottostante il blob  per eliminare rumore
  //_check_region(max_row + 1,
  //  max_real_row,
  //  disparityMap,
  //    OutOfRangeManager::MAX_DISP_VALUE);
  //}
}


/*
Controlla la regione sopra e sotto del blob virtuale per eliminare rumore

\param first_indx   [in]     Indice della prima riga da considerare
\param last_indx    [in]     Indice dell'ultima riga da considerare
\param disparityMap [in|out] Mappa di disparit&agrave;
*/
//void _check_region(int first_indx, int last_indx, unsigned char* disparityMap, const int max_disp_value)
//{
//  unsigned char* disvec;
//  for (int i = first_indx; i < last_indx; ++i)
//  {
//    disvec = &(disparityMap[i * g_ncols]);  // Parto dalla prima colonna di ogni riga
//    for (int j = 0; j < g_ncols; ++j, ++disvec)
//      if (*disvec > max_disp_value)  // Gestisco il caso dei blob piccoli che possono nascere in out-of-range
//        *disvec = max_disp_value;
//  }
//}


/*
Controlla le regioni destra e sinistra del blob
eliminando picchi spuri o rumore
*/
//void 
//_check_side_region(
//  const int min_row_indx,       ///< [in]     Indice della prima riga
//  const int max_row_indx,       ///< [in]     Indice dell'ultima riga da considerare
//  const int min_col_indx,       ///< [in]     Indice della prima riga
//  const int max_col_indx,       ///< [in]     Indice dell'ultima riga da considerare
//  int* const & delta_vec,       ///< [in]     Vettore dei OutOfRangeManager::orig_virtual_blob_col_deltas per il limite del cerchio
//  unsigned char* disparityMap,  ///< [in|out] Mappa di disparit&agrave;
//  int* const & offset_vec,      ///< [in]     Vettore degli offset delle righe
//  unsigned int centroid_col,
//  const int max_disp_value)    ///< [in]     Colonna del centroide
//{
//  unsigned char* disvec;
//
//  for (int r = min_row_indx; r <= max_row_indx; ++r)
//  {
//     //Calcolo l'indice dell'offset
//    int indx_offset = offset_vec[r];
//    disvec = &(disparityMap[r * g_ncols + min_col_indx]);  // Mi posiziono sulla mappa
//    for (int c = min_col_indx; c <= max_col_indx; ++c, ++disvec)
//    {
//      int d = (int) c - centroid_col;
//      if (abs(d)  > abs(delta_vec[indx_offset]))
//      {
//        if (*disvec > max_disp_value)
//          *disvec = max_disp_value;
//      }
//    }
//  }
//}


/*!
Crea un vettore che contiene i valori del raggio del blob che cambier&agrave; a seconda della posizione nella mappa
sar&agrave; Ad esempio  nella parte centrale sar&agrave; pi&ugrave; grande rispetto ai lati.
*/
void
_create_adaptive_ray(
                     int* const & ray_value_vec,   ///< [out] Vettore di dimensione #ray_value_vec_dim che assegna un valore che indica il raggio del blob virtuale in base alla colonna del centroide
                     //const int ray_value_vec_dim,  ///< [in] Dimensione del vettore #ray_value_vec
                     const int min_ray,            ///< [in] Raggio minimo del blob virtuale
                     const int max_ray)            ///< [in] Raggio massimo del blob virtuale
{
  float m = (float) (max_ray - min_ray) / (g_ncols / 2 - g_border_x);  // coefficente angolare della retta
  int q = (int) (min_ray - (g_border_x * m));  // intercetta

  for (int i = 0; i < g_border_x; ++i)
    ray_value_vec[i] = min_ray;
  for (int i = g_ncols-g_border_x; i < g_ncols; ++i)
    ray_value_vec[i] = min_ray;
  for (int i = g_border_x; i < (g_ncols+1)/2; ++i)  // si noti che (g_ncols+1)/2 e' adatta sia con g_ncols pari che dispari
  {
    int val = (int) (i * m) + q;
    val = max(min_ray, min(max_ray, val));  // verifico di rimanere nei limiti prestabiliti (a causa di approssimazioni potrebbe non essere cosi')
    ray_value_vec[i] = val;
    ray_value_vec[g_ncols-i-1] = val;
  }
}


/*!
Abilita la capacit&agrave; di gestire o meno l'out-of-range
*/
void
OutOfRangeManager::SetEnableStateOutOfRange(const bool state)  ///< [in] Booleano da assegnare alla variabile statica #m_enable_handle_out_of_range
{
  m_enable_handle_out_of_range = state;
  m_is_out_of_range = false;
  print_log("SetEnableStateOutOfRange (%s)\n", (state) ? "On" : "Off");
}


/*!
Calcola e restituisce il raggio del blob virtuale in base alla posizione del suo centroide nella mappa.
Se la funzione &egrave; richiamata al'interno della funzione #CopyVirtualBlob() allora verr&agrave; considerato
il raggio totale mentre nel caso venga chiamata all'interno della funzione #_clear_repository
verr&agrave; considerato il 114% del raggio usato per la copia del blob in modo da poter eliminare i blob di rumore appena all'esterno
del blob virtuale (circa 10 pixel in pi&ugrave;).

\return Raggio del blob virtuale
*/
int
OutOfRangeManager::ComputeRay()
{
  //static int ray_value_vec[NX] = {};  // Vettore di dimensione #g_ncols che contiene i valori del raggio del blob virtuale adattativo
  //static bool first_time = true;

  //// Se e' la prima volta che entro creo il vettore contenente la dimensione dei raggi del blob in base alla posizione
  //if (first_time)
  //{
  //  printf("ComputeRay(): inizializzazione dati raggio blob virtuale.\n");
  //  first_time = false;
  //  _create_adaptive_ray(ray_value_vec, OutOfRangeManager::MIN_RAY, OutOfRangeManager::MAX_RAY);
  //}

  //int ray = -1;  // Raggio finale
  //if (m_cent_r > 0 && m_cent_c > 0 && m_num_black_pixels > 0)
  //{
  //  // Fattore di normalizzazione
  //  int fact_r = g_nrows/2;
  //  int fact_c = g_ncols/2;

  //  // Differenza tra le coordinate del centroide e il centro della mappa
  //  int delta_r = abs(fact_r - m_cent_r);
  //  int delta_c = abs(fact_c - m_cent_c);

  //  int weight_c, weight_r;  // Peso da assegnare ai due raggi
  //  int ray_c, ray_r;  // Valore del raggio dovuto alla colonne e alla righe
  //  int ind_r = (m_cent_r*4)/3;  // Indice riscalato per le righe

  //  // Calcolo la dimensione dei raggi rispettivamente per le colonne e per le righe
  //  ray_c = ray_value_vec[m_cent_c];
  //  ray_r = ray_value_vec[ind_r];

  //  // Calcolo il raggio mediante una media pesata gestendo il caso in cui i OutOfRangeManager::orig_virtual_blob_col_deltas sono entrambi nulli.
  //  if(delta_c == 0 && delta_r == 0)
  //  {
  //    ray = ray_c;
  //  }
  //  else
  //  {
  //    weight_c = (2048*delta_c + fact_c/2) / fact_c;
  //    weight_r = (2048*delta_r + fact_r/2) / fact_r;
  //    assert(weight_c > 0 || weight_r > 0);
  //    ray = (weight_c * ray_c + weight_r * ray_r) / (weight_c + weight_r);
  //  }

  //  // riscalo il raggio calcolato in base al numero di pixel neri
  //  static const int num_black_pixels_at_10cm = (6500-NUM_BLACK_PIXEL_ON)/(binning*binning);
  //  int fact = min(512, (512*(m_num_black_pixels-NUM_BLACK_PIXEL_ON) + num_black_pixels_at_10cm/2) / num_black_pixels_at_10cm);
  //  ray = ((512 + fact) * ray) / 1024;
  //}

  int ray = (140*(int)(sqrtf((float)((100*(m_num_black_pixels + m_num_DSP))/314))))/100;

  assert(ray > 0);

  return ray;
}


int
OutOfRangeManager::GetRay()
{
  return m_ray;
}


int
OutOfRangeManager::GetRow()
{
  return m_cent_r;
}


int
OutOfRangeManager::GetCol()
{
  return m_cent_c;
}


bool
OutOfRangeManager::IsOutOfRange()
{
  return m_is_out_of_range;
}


bool
OutOfRangeManager::IsOutOfRangeEnabled()
{
  return m_enable_handle_out_of_range;
}


/*! 
Costruttore della classe OutOfRangeManager. Si inizializza il campo m_enable_handle_out_of_range
e si crea il blob virtuale che verr&agrave; tenuto in memoria.
*/
OutOfRangeManager::OutOfRangeManager()
{
  compute_binned_nrow_ncols(NY, NX, binning, BORDER_X, BORDER_Y, g_nrows, g_ncols);
  g_border_x = g_border_y = 0;

  if (handle_oor == 0)
    m_enable_handle_out_of_range = false;
  else
    m_enable_handle_out_of_range = true;


  m_is_out_of_range = false;
  m_num_black_pixels = 0;
  m_num_DSP = 0;
  m_cent_r = m_cent_c = m_ray = -1;
#ifdef USE_STATIC_BLOB_CHECK
  m_blob_static = false;
#endif
  CreateVirtualBlob();
  //ComputeRay();  // just to create static data (see code of ComputeRay())
  init_out_of_range_centroid_weights();  // just to create static data (see code of find_max_num_black_pixels())
}
#endif
