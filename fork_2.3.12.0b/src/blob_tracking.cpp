#include "directives.h"
// prova di modifica
#ifdef USE_NEW_TRACKING
#include "blob_tracking.h"
#include "hungarian_method.h"
#include "default_parms.h"

#ifdef USE_HANDLE_OUT_OF_RANGE
#include "OutOfRangeManager.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <limits.h>

//#define VERBOSE



#define BG_Y_MAX_DELTA 7 // > 6 beacause of binning

const int life_fact = 2;
#if defined(PCN_VERSION) || defined(SEQ_AT_54FPS)
const int LIFE=life_fact*18;  ///< numero massimo di vite di un blob
#else
const int LIFE=life_fact*12;  ///< numero massimo di vite di un blob
#endif

const int INIT_LIFE = LIFE/2;  ///< numero di vite iniziali di un blob
const int LIFE_INC = 1;  ///< incremento del numero di vite quando il tracking ha successo
const int LIFE_DEC = 2;  ///< decremento del numero di vite quando un tracking fallisce

#ifdef VERBOSE
static unsigned long num_frame = 0;
#endif

// 20120120 eVS, added 
#define MAX_NUM_SLAVES 4 //!< Maximum number of slaves connected to a master in widegate

#define MAX_COST (INT_MAX-1)

#ifdef PCN_VERSION
//#define MAX_BLOB_DIST_SQUARED 324 // 18^2
#define MIN_NUM_FRAMES 10
#else
//#define MAX_BLOB_DIST_SQUARED 1225 // 35^2
#define MIN_NUM_FRAMES 7
#endif

#define MAX_H_DIFF_MIN  15 // (> 8, i.e. one level of disparity -notice that h is in 0..127 because of a division by 2) minimum for the maximum difference between disparities of the blobs 
//#define MAX_MH_DIFF_MIN 20 // (> 8, i.e. one level of disparity -notice that h is in 0..127 because of a division by 2) minimum for the maximum difference between disparities of the blobs 

//#define MAX_H_DIFF_PERC 30 // maximum difference of the disparity value of the blobs (percentage wrt the value in the repository)
#define MIN_H_DIFF_PERC 20 // maximum difference of the disparity value of the blobs (percentage wrt the value in the repository)

//#define MAX_WX_DIFF_MIN  7  // (> 6, because of binning) minimum for the maximum difference between widths of the blobs
////#define MAX_WX_DIFF_PERC 25 // maximum difference of the width of the blobs (percentage wrt the value in the repository)
//#define MIN_WX_DIFF_PERC 15 // maximum difference of the width of the blobs (percentage wrt the value in the repository)
//
//#define MAX_WY_DIFF_MIN  7  // (> 6, because of binning) minimum for the maximum difference between heights of the blobs
////#define MAX_WY_DIFF_PERC 25 // maximum difference of the heights of the blobs (percentage wrt the value in the repository)
//#define MIN_WY_DIFF_PERC 15 // maximum difference of the heights of the blobs (percentage wrt the value in the repository)

// 20130515 eVS added
#define CLOSE_DOOR_MOV_TH 4 ///< minimo numero di passi per poter essere contato nell'evento porta chiusa

#ifndef NOMINMAX
  #ifndef max
    #define max(a,b) (((a) > (b)) ? (a) : (b))
  #endif
  #ifndef min
    #define min(a,b) (((a) < (b)) ? (a) : (b))
  #endif
#endif  /* NOMINMAX */

tPersonTracked ** inhi = NULL;
tPersonTracked ** inlo = NULL;


/*!
\var people_count_input
\brief Contatore delle persone entrate (dalla zona alta della regione monitorata se la direzione &egrave; settata a zero).
*/
unsigned long people_count_input;

/*!
\var people_count_output
\brief Contatore delle persone uscite (dalla zona alta della regione monitorata se la direzione &egrave; settata a zero).
*/
unsigned long people_count_output;

void draw_cross_on_map(tPersonTracked *person_data, unsigned char *map);
int find_a_free_element_in_repo(tPersonTracked ** repo, const int person, const int num_pers);


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_robust_num_of_life(const tPersonTracked* const & rep_data_i)
{
  // voglio che il blob sia statisticamente stato traccato nel (100*FILE_DEC/(LIFE_INC+LIFE_DEC))% di frame 
  // e che al piu' due tracking in piu' rispetto a tale statistica siano stati persi
  return (rep_data_i->life >= LIFE-2*LIFE_DEC);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_enoug_tall(const tPersonTracked* const & rep_data_i)
{
  return (rep_data_i->max_h > (MIN_M-16));  // remove a disparity just to be sure we do not remove something interesting
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
has_moved_enough(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int min_y_gap)
{
  int sign = (is_from_high) ? 1 : -1;
  return (sign*rep_data_i->delta_y >= DELTA_DOOR_TH+DELTA_DOOR_TH) && (sign*rep_data_i->delta_y >= min_y_gap);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline int
first_valid_y(const bool is_from_high, const int door_threshold)
{
  int sign = (is_from_high) ? 1 : -1;
  return (door_threshold-sign*DELTA_DOOR_TH);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_appeared_before_threshold(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  int sign = (is_from_high) ? 1 : -1;
  return (sign*rep_data_i->first_y < sign*first_valid_y(is_from_high, door_threshold));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_in_the_exit_zone(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  int sign = (is_from_high) ? 1 : -1;
  return (sign*rep_data_i->y > sign*(door_threshold+sign*DELTA_DOOR_TH));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_in_the_enter_zone(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  int sign = (is_from_high) ? 1 : -1;
  return (sign*rep_data_i->y < sign*(door_threshold-sign*DELTA_DOOR_TH));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_in_the_critical_zone(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  return (!is_in_the_exit_zone(rep_data_i, is_from_high, door_threshold) && !is_in_the_enter_zone(rep_data_i, is_from_high, door_threshold));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_tracking_rate_enough(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  // voglio aver traccato per almeno il 66% dei frame (si noti che gli ultimi LIFE/LIFE_DEC fallimenti sono dovuti alla'uscita di scena e quindi non vanno considerati)
  if (is_in_the_enter_zone(rep_data_i, is_from_high, door_threshold) || is_in_the_critical_zone(rep_data_i, is_from_high, door_threshold))
    return true;  // (rep_data_i->num_frames/max(1,(rep_data_i->num_failures)) >= 2);
  else  // se sono nella exit zone il blob potrebbe essere in via di uscita e quindi ci possono essere i failure dovuti alla sparizione quindi...
    return true;  // (rep_data_i->num_frames/max(1,(rep_data_i->num_failures-LIFE/LIFE_DEC)) >= 2);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_enoug_persistent(const tPersonTracked* const & rep_data_i, const bool is_from_high, const int door_threshold)
{
  return (rep_data_i->num_frames >= MIN_NUM_FRAMES &&
          is_tracking_rate_enough(rep_data_i, is_from_high, door_threshold));  // voglio aver traccato per almeno il 66% dei frame (si noti che gli ultimi LIFE/LIFE_DEC fallimenti sono dovuti alla'uscita di scena e quindi non vanno considerati)
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_blob_new(const tPersonTracked* const & rep_data_i)
{
  return (rep_data_i->num_frames == 1 && 
          rep_data_i->life == INIT_LIFE && 
          rep_data_i->delta_y == 0 &&
          rep_data_i->trac);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool
is_blob_appeared_only_once(const tPersonTracked* const & rep_data_i)
{
  return (rep_data_i->num_frames == 1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// _can_person_be_counted
inline static bool
_can_person_be_counted(
  const tPersonTracked* const & rep_data_i, 
  const bool is_from_high, 
  const unsigned short door_threshold, 
  const int min_y_gap)
{
  const bool enough_tall = is_enoug_tall(rep_data_i);
  const bool enough_persistent = is_enoug_persistent(rep_data_i, is_from_high, door_threshold);
  const bool enough_movement = has_moved_enough(rep_data_i, is_from_high, min_y_gap);
  const bool is_first_y_valid = is_appeared_before_threshold(rep_data_i, is_from_high, door_threshold);
  const bool is_current_y_valid = is_in_the_exit_zone(rep_data_i, is_from_high, door_threshold);

  bool ret = enough_tall && // mi assicuro di aver visto la persona con una altezza minima
             enough_persistent && // mi assicuro di aver inseguito la persona in almeno TOT frame o per almeno TOT passi
             enough_movement && // movimento minimo per essere contati
             //rep_data_i->cont == true && // la persona è entrata molto prima (MIN_DIST_NEW_BLOB_FROM_DOOR_TH) della soglia (altrimenti non sarebbe nella lista) e l'ha sorpassata un po' dopo (DELTA_DOOR_TH) e non è tornata indietro
             is_first_y_valid && // ridondante? dovrebbe essere sempre vero in quanto inserisco i blob solo se la first_y e' valida (ma quando uso il segnale porta?)
             is_current_y_valid; // ridondante? dovrebbe essere incapsulata in cont

  //if (rep_data_i->cont && (!is_first_y_valid || !is_current_y_valid))
  //{
  //  printf("%c%ld countable? %s\n"
  //         "    enough_persistent = %s\n"
  //         "    enough_movement = %s\n"
  //         //"    count = %s\n"
  //         "    is_first_y_valid = %s\n"
  //         "    is_current_y_valid = %s\n",
  //         is_from_high ? 'H' : 'L',
  //         rep_data_i->ID,
  //         ret ? "TRUE" : "false",
  //         enough_persistent ? "TRUE" : "false",
  //         enough_movement ? "TRUE" : "false",
  //         //rep_data_i->cont ? "TRUE" : "false",
  //         is_first_y_valid ? "TRUE" : "false",
  //         is_current_y_valid ? "TRUE" : "false");
  //  getchar();
  //}

  return ret;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// _pers_to_be_counted
inline static bool
_pers_to_be_counted(const tPersonTracked* const & rep_data_i, 
                   const bool is_from_high, 
                   const unsigned short door_threshold, 
                   const int min_y_gap,
                   const bool is_closure = false)
{
  bool ret;
  if (is_closure)  // se sono in chiusura guardo solo movimento minimo e numero frame
  {
    const bool enough_persistent = is_enoug_persistent(rep_data_i, is_from_high, door_threshold);
    ret = enough_persistent;
  }
  else  // altrimenti guardo tutto
  {
     ret = _can_person_be_counted(rep_data_i, is_from_high, door_threshold, min_y_gap); 
  }
  return ret;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// _count_people
inline static void 
_count_people(tPersonTracked** const & repo, const bool is_inhi, 
              const int num_pers, const unsigned short door_threshold, 
              const int min_y_gap, const unsigned char move_det_en, const bool count_true_false,
              unsigned long & counter, const int & count_enabled)
{
    for(int i=0;i<num_pers;++i)
    {
      if(repo[i]!=NULL)
      {
        if(repo[i]->trac==false)
        {
          repo[i]->life = max(0, repo[i]->life-LIFE_DEC); //se la persona nello storico non e' stata traccata diminuisci di due le sue vite
//          repo[i]->num_failures++;

          if(repo[i]->life<=0)
          {
#ifdef debug
            printf("contato repo h=%d x=%d y=%d life=%d\n",repo[i]->h,repo[i]->x,repo[i]->y,repo[i]->life);
#endif 
            // incremento il contatore delle persone entrate solo se
            // il motion detection e' disabilitato oppure se e' abilitato e c'e' "movimento" a meno di 200 frame di 
            // potenziale inattivita' (vedi loops.cpp funzione main_loop)
            if (_pers_to_be_counted(repo[i], is_inhi, door_threshold, min_y_gap))
            {
              if ((move_det_en == 0 || count_true_false == true) && count_enabled)
                counter++;
#ifdef _DEBUG
              if (!count_enabled)
                printf("_count_people(): conteggio non fatto perché porte chiuse.\n");
#endif
#ifndef VERBOSE
            }
#else
              printf("-> repo:\n");
              printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, repo[i]->ID);
            }
            else
            {
              printf("-> repo:\n");
              printf("--> blob_rep[%d]->ID = %ld [removed]\n\n", i, repo[i]->ID);
            }
#endif

            // if life is zero we have to remove the blob from the repository
            delete repo[i];
            repo[i]=NULL;
          }
        }
      }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// _manage_conflict
#ifdef USE_CONSISTENCY_CHECK

inline static bool 
_are_persons_in_conflict(tPersonTracked* const & p1, tPersonTracked* const & p2)
{
  int diff_x_px = p1->x - p2->x;
  int diff_y_px = p1->y - p2->y;
  int dist2_px = diff_x_px*diff_x_px + diff_y_px*diff_y_px;
  int min_dist_px = (min(p1->max_h, p2->max_h))/FROM_DISP_TO_HEAD_RAY; // this is around half head width

#ifdef COMPUTE_FEET_COORDS
  int diff_x_mm = p1->x_mm - p2->x_mm;
  int diff_y_mm = p1->y_mm - p2->y_mm;
  int dist2_mm = diff_x_mm*diff_x_mm + diff_y_mm*diff_y_mm;
  int min_dist_mm = MIN_FEET_DIST; // 50cm should be the minimum distance between persons
  return (dist2_mm <= min_dist_mm*min_dist_mm || dist2_px <= min_dist_px*min_dist_px);
#else
  return (dist2_px < min_dist_px*min_dist_px);
#endif
}


inline static void
_prepare_person_to_be_removed_or_counted(tPersonTracked* const & p, bool & p_still_exists)
{
  p->life = 0;
  p->trac = false;
  p_still_exists = false;
}


#if defined(_DEBUG) || defined(VERBOSE)
inline static void
_print_person_str(tPersonTracked* const & p, const bool is_from_high)
{
  printf("%c%lu: %c (%d,%d),dy=%d,(fx,fy)=(%d,%d),h=%d,mh=%d,nfr=%d,cost=%lu,lives=%d\n", 
    (is_from_high) ? 'H' : 'L',
    p->ID, 
    p->trac ? 'T' : '-', 
    p->x, p->y, 
    p->delta_y, p->first_x, p->first_y, 
    p->h, p->max_h, 
    p->num_frames, 
    p->life);
}
#endif


inline static void
_manage_conflict(tPersonTracked* const & p1, tPersonTracked* const & p2,
                 const bool is_p1_from_hi, const bool is_p2_from_hi,
                 bool & p1_still_exists, bool & p2_still_exists,
                 const unsigned short door_threshold, const int min_y_gap)
{
  if (_are_persons_in_conflict(p1, p2)) // se due blob sono troppo vicini significa che uno dei due è da eliminare dallo storico
  {
#if defined(_DEBUG)
    char p1_desc = (is_p1_from_hi ? 'H' : 'L');
    char p2_desc = (is_p2_from_hi ? 'H' : 'L');
#endif
    bool can_p1_be_counted = _can_person_be_counted(p1, is_p1_from_hi, door_threshold, min_y_gap); // contabile ma le vite non sono ancora a zero
    bool can_p2_be_counted = _can_person_be_counted(p2, is_p2_from_hi, door_threshold, min_y_gap); // contabile ma le vite non sono ancora a zero

#if defined(_DEBUG) && defined(SHOW_RESULT)
    printf("\tManage conflict: %c%lu - %c%lu \n", p1_desc, p1->ID, p2_desc, p2->ID);
#endif

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (is_p1_from_hi != is_p2_from_hi)  // se i blob provengono da liste diverse
    {
      if (can_p1_be_counted || can_p2_be_counted)  // se uno dei due e' contabile allora...
      {
        if (can_p1_be_counted) 
        {
          if (!is_blob_appeared_only_once(p2))
          {
#if defined(_DEBUG) && defined(SHOW_RESULT)
            printf("\t%c%lu e' da contare! Faccio in modo che venga contato!\n", p1_desc, p1->ID);
#endif
            _prepare_person_to_be_removed_or_counted(p1, p1_still_exists);
          }
          else
          {
#if defined(_DEBUG) && defined(SHOW_RESULT)
            printf("\t%c%lu e' da contare ma %c%lu e' nuovo! Elimino %c%lu!\n", p1_desc, p1->ID, p2_desc, p2->ID, p2_desc, p2->ID);
#endif
            _prepare_person_to_be_removed_or_counted(p2, p2_still_exists);
          }
        }

        if (can_p2_be_counted)
        {
          if (!is_blob_appeared_only_once(p1))
          {
#if defined(_DEBUG) && defined(SHOW_RESULT)
            printf("%\t%c%lu e' da contare! Faccio in modo che venga contato!\n", p2_desc, p2->ID);
#endif
            _prepare_person_to_be_removed_or_counted(p2, p2_still_exists);
          }
          else
          {
#if defined(_DEBUG) && defined(SHOW_RESULT)
            printf("\t%c%lu e' da contare ma %c%lu e' nuovo! Elimino %c%lu!\n", p2_desc, p2->ID, p1_desc, p1->ID, p1_desc, p1->ID);
#endif
            _prepare_person_to_be_removed_or_counted(p1, p1_still_exists);
          }
        }
      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (is_p1_from_hi == is_p2_from_hi)  // se sono nella stessa lista 
    {
      if (is_in_the_enter_zone(p1, is_p1_from_hi, door_threshold) &&  // se sia p1 ...
        is_in_the_enter_zone(p2, is_p2_from_hi, door_threshold))  // che p2 sono nella zona di entrata allora tengo il blob piu' recente
      {
        if (is_blob_new(p1) && !p2->trac) // se p1 e' nuovo e p2 non e' stato traccato suppongo che sia qualcouno che sta entrando e tengo il piu' recente
        {
#if defined(_DEBUG) && defined(SHOW_RESULT)
          printf("\t%c%lu e' nuovo e %c%lu non e' stato traccato: elimino %c%lu!\n", 
            p1_desc, p1->ID, p2_desc, p2->ID, p2_desc, p2->ID);
#endif
          _prepare_person_to_be_removed_or_counted(p2, p2_still_exists);
        }
        if (is_blob_new(p2) && !p1->trac) // se p2 e' nuovo e p1 non e' stato traccato suppongo che sia qualcouno che sta entrando e tengo il piu' recente
        {
#if defined(_DEBUG) && defined(SHOW_RESULT)
          printf("\t%c%lu e' nuovo e %c%lu non e' stato traccato: elimino %c%lu!\n", 
            p2_desc, p2->ID, p1_desc, p1->ID, p1_desc, p1->ID);
#endif
          _prepare_person_to_be_removed_or_counted(p1, p1_still_exists);
        }
      }
    }

    if (p1_still_exists && p2_still_exists)  // se il conflitto non e' stato ancora risolto
    {      
      // faccio altri controlli indipendentemente dalla lista di appartenenza

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (//is_in_the_enter_zone(p1, is_p1_from_hi, door_threshold) &&  // se p1 e' appena entrato e...
        (is_in_the_critical_zone(p2, is_p2_from_hi, door_threshold) || is_in_the_exit_zone(p2, is_p2_from_hi, door_threshold))&&  // p2 e' nella zona critica e...
        is_tracking_rate_enough(p2, is_p2_from_hi, door_threshold) &&  // p2 lo vedo bene (>= 66%) e da un po' di frame e...
        is_blob_appeared_only_once(p1) )  // p1 e' nuovo allora tengo solo p2
      {
#if defined(_DEBUG) && defined(SHOW_RESULT)
        printf("\t%c%lu e' avanti e %c%lu e' apparso solo una volta: elimino %c%lu!\n", 
          p2_desc, p2->ID, p1_desc, p1->ID, p1_desc, p1->ID);
#endif
        _prepare_person_to_be_removed_or_counted(p1, p1_still_exists);
      }
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      else if (//is_in_the_enter_zone(p2, is_p2_from_hi, door_threshold) &&  // se p2 e' appena entrato e...
        (is_in_the_critical_zone(p1, is_p1_from_hi, door_threshold) || is_in_the_exit_zone(p1, is_p1_from_hi, door_threshold)) &&  // p1 e' nella zona critica e...
        is_tracking_rate_enough(p1, is_p1_from_hi, door_threshold) &&  // p1 lo vedo da un po' di frame e...
        is_blob_appeared_only_once(p2) )  // p2 e' nuovo allora tengo solo p1
      {
#if defined(_DEBUG) && defined(SHOW_RESULT)
        printf("\t%c%lu e' avanti e %c%lu e' apparso solo una volta: elimino %c%lu!\n", 
          p1_desc, p1->ID, p2_desc, p2->ID, p2_desc, p2->ID);
#endif
        _prepare_person_to_be_removed_or_counted(p2, p2_still_exists);
      }
    }

#if defined(_DEBUG)
    if (p1_still_exists && p2_still_exists)
    {
      printf("\tOps %c%c!\n", p1_desc, p2_desc);
    }
#endif
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// _history_consistency_check
inline static void 
_history_consistency_check(tPersonTracked** const & inhi, tPersonTracked** const & inlo, const int num_pers, 
                           const unsigned short door_threshold, const int min_y_gap)
{
  static bool pH_still_exists[MAX_NUM_PERS];
  static bool pL_still_exists[MAX_NUM_PERS];

  for (int j=0; j<num_pers; ++j)
    pH_still_exists[j] = pL_still_exists[j] = true;

  // verifico conflitti tra gli elementi della lista dall'alto e gli elementi della lista dal basso
  for(int i=0;i<num_pers;++i)
  {
    tPersonTracked* & pH = inhi[i];
    if (pH != NULL && pH_still_exists[i])
    {
      int j=0;
      while (j<num_pers && pH_still_exists[i])
      {
        tPersonTracked* & pL = inlo[j];
        if (pL != NULL && pL_still_exists[j])
        {
          if (_are_persons_in_conflict(pH, pL)) // se due blob sono troppo vicini significa che uno dei due è da eliminare dallo storico
             _manage_conflict(pH, pL, true, false, pH_still_exists[i], pL_still_exists[j], door_threshold, min_y_gap);
        }
        ++j;
      }
    }
  }

  // verifico conflitti tra gli elementi della lista dall'alto
  for(int i=0;i<num_pers;++i)
  {
    tPersonTracked* & pH1 = inhi[i];
    if (pH1 != NULL && pH_still_exists[i])
    {
      int j=0;
      while (j<num_pers && pH_still_exists[i])
      {
        tPersonTracked* & pH2 = inhi[j];
        if (pH2 != NULL && 
            pH_still_exists[j] && // si noti che pH2 fa parte di inhi quindi si usa pH_still_exists
            i != j) // sarebbe errato confrontare pH1 con se stesso (porterebbe alla sua eliminazione)
        {
          if (_are_persons_in_conflict(pH1, pH2)) // se due blob sono troppo vicini significa che uno dei due è da eliminare dallo storico
            //_manage_conflict_HH(pH1, pH2, pH_still_exists[i], pH_still_exists[j], door_threshold, min_y_gap);
            _manage_conflict(pH1, pH2, true, true, pH_still_exists[i], pH_still_exists[j], door_threshold, min_y_gap);
        }
        ++j;
      }
    }
  }

  // verifico conflitti tra gli elementi della lista dal basso
  for(int i=0;i<num_pers;++i)
  {
    tPersonTracked* & pL1 = inlo[i];
    if (pL1 != NULL && pL_still_exists[i])
    {
      int j=0;
      while (j<num_pers && pL_still_exists[i])
      {
        tPersonTracked* & pL2 = inlo[j];
        if (pL2 != NULL && 
            pL_still_exists[j] && // 
            i != j) // sarebbe errato confrontare pL1 con se stesso (porterebbe alla sua eliminazione)
        {
          if (_are_persons_in_conflict(pL1, pL2)) // se due blob sono troppo vicini significa che uno dei due è da eliminare dallo storico
            _manage_conflict(pL1, pL2, false, false, pL_still_exists[i], pL_still_exists[j], door_threshold, min_y_gap);
        }
        ++j;
      }
    }
  }
}
#endif


/*!
  \brief  Elimina nei repository i blob delle lista passata come parametro con centroide ad una distanza inferiore
  al raggio del blob virtuale rispetto al centroide del blob virtuale stesso.
*/
void
_clear_repository(
  tPersonTracked** const & blob_list,      ///< [in] Lista dei blob
  const int dim_blob_list,         ///< [in] Dimensione della lista
  const int centr_c_virtual,       ///< [in] Coordinata X del centroide del blob virtuale
  const int centr_r_virtual,       ///< [in] Coordinata Y del centroide del blob virtuale
  const int ray_virtual)           ///< [in] Raggio di ricerca per l'eliminazione dei blob di rumore
{
  const int ray2 = (ray_virtual)*(ray_virtual);

  for (int i = 0; i < dim_blob_list; ++i)
  {
    if (blob_list[i])
    {
      if ((blob_list[i]->h < 120 || abs(blob_list[i]->x-centr_c_virtual) > 10 || abs(blob_list[i]->y-centr_r_virtual) > 10) &&  // non sono il virtual blob
          blob_list[i]->num_frames < 5)  // TODO <5? qui forse sarebbe meglio == 1 visto che ora _clear_repository() lo facciamo ad ogni frame
      {
        const int temp_c = (centr_c_virtual - blob_list[i]->x)*(centr_c_virtual - blob_list[i]->x);
        const int temp_r = (centr_r_virtual - blob_list[i]->y)*(centr_r_virtual - blob_list[i]->y);
        const int dist = temp_c + temp_r;

        if (dist <= ray2)
        {
#ifdef _DEBUG
          //printf("\n Elimino il blob id: %u, perche' troppo vicino a quello virtuale!\n", blob_list[i]->ID);
#endif
          delete blob_list[i];
          blob_list[i]=NULL;
        }
      }  // fine if blob_list[i]->ID != id_virtual_ray)
    }  // fine if blob_list[i]
  }  // fine scansione lista
}


/*!
  Elimino i blob di rumore vicini al blob virtuale.
*/
void
_clear_blobs_around_virtual_blob(
  tPersonTracked** const & inhi,  ///< [in|out] Lista delle persone provenienti dall'alto
  const int inhi_length,          ///< [in] Dimensione della lista inhi
  tPersonTracked** const & inlo,  ///< [in|out] Lista delle persone provenienti dal basso
  const int inlo_length,          ///< [in] Dimensione della lista inlo
  const int centr_r_virtual,
  const int centr_c_virtual,
  const int ray_virtual)          ///< [in] Raggio di ricerca per l'eliminazione dei blob di rumore
{
  assert(inhi != NULL && inlo != NULL);

#ifdef _DEBUG
  //printf("\n Controllo dei repository....!\n");
#endif

  // Controllo la lista inlo
  _clear_repository(inlo, inlo_length, centr_c_virtual, centr_r_virtual, ray_virtual);

  // Controllo la lista inhi
  _clear_repository(inhi, inhi_length, centr_c_virtual, centr_r_virtual, ray_virtual);
}


static bool
pers_to_be_counted(const tPersonTracked* const & rep_data_i, 
                   const bool is_from_high, 
                   const unsigned short door_threshold, 
                   const int min_y_gap,
                   const bool is_closure = false)
{
  const bool enough_persistent = rep_data_i->num_frames >= MIN_NUM_FRAMES;

  bool ret = enough_persistent && // mi assicuro di aver inseguito la persona in almeno TOT frame o per almeno TOT passi
             (is_closure || 
             ((rep_data_i->cont==true) && // la persona è entrata prima della soglia e l'ha sorpassata
               rep_data_i->life<=0)); // prima di contare attendo che siano finite le vite

  if (is_from_high)
  {
    ret = ( ret && ((is_closure && (rep_data_i->delta_y >= 1+CLOSE_DOOR_MOV_TH*2) )|| 
            ((rep_data_i->delta_y >= 1+DELTA_DOOR_TH*2) &&
             (rep_data_i->y > door_threshold+DELTA_DOOR_TH) && // l'ultima y deve essere oltre soglia
             (rep_data_i->delta_y >= min_y_gap) && // deve aver percorso un certo delta_y
             (rep_data_i->first_y < door_threshold-DELTA_DOOR_TH)))
           ); // la prima y deve essere sotto soglia
  }
  else
  {
    ret = ( ret && 
            ((is_closure && (-(rep_data_i->delta_y) >= 1+CLOSE_DOOR_MOV_TH*2) )|| 
            ((-(rep_data_i->delta_y) >= 1+DELTA_DOOR_TH*2) && 
             (rep_data_i->y < door_threshold-DELTA_DOOR_TH) &&
             (-(rep_data_i->delta_y) >= min_y_gap) &&
             (rep_data_i->first_y >= door_threshold+DELTA_DOOR_TH)))
           );
  }

  return ret;
}


/*! 
\brief Inizializza 2 passi quando si apre la porta.

Serve per contare le persone al limite della immagine che altrimenti non farebbero un numero sufficiente di passi.

\param direction se la direzione &egrave; settata ad 1 allora una persona viene considerata come entrata se 
                attraversa la zona monitorata dall'alto verso il basso
\param num_pers total number of possible tracked person                
*/
void SetPassi(const unsigned char & direction, const int & num_pers)
{
#ifdef debug_
    printf("set passi\n");
    int count_p=0;
    int count_p_hi=0;
    printf("SET PASSI \n");
    for(int i=0;i<num_pers;++i)
    {
        if(inlo!=NULL)
            if(inlo[i]!=NULL) 
            {
                if(inlo[i]->h>0)
                {
                    count_p++;
                }
            }
        if(inhi!=NULL)
            if(inhi[i]!=NULL) 
            { 
                if(inhi[i]->h>0)
                {
                    count_p_hi++;
                }
            }
    }
    printf("ci sono inlo=%d persone inhi=%d persone nelle strutture, la direzione e' %d\n",count_p,count_p_hi,direction);
    count_p=0;
#endif
    if(direction==0)
    {
        for(int i=0;i<num_pers;++i)
        {
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                    inlo[i]->num_frames=MIN_NUM_FRAMES; // per garantire il conteggio
                    inlo[i]->delta_y=-DELTA_DOOR_TH-1;
                    // if(wideg!=0 )inlo[i]->life=100; //90
#ifdef debug_
                    printf("persona inlo indice i=%d h=%d x=%d y=%d passi=%d\n",i,inlo[i]->h,inlo[i]->x,inlo[i]->y,inlo[i]->passi);
                    count_p++;
#endif
                }
        }
    }
    else
    {	
        for(int i=0;i<num_pers;++i)
        {
            if(inhi!=NULL)
                if(inhi[i]!=NULL)
                {
                    inhi[i]->num_frames=MIN_NUM_FRAMES; // per garantire il conteggio
                    inhi[i]->delta_y=DELTA_DOOR_TH+1;
                    //   if(wideg!=0 ) inhi[i]->life=100; //90
#ifdef debug_            
                    printf("persona inhi indice i=%d h=%d x=%d y=%d passi=%d\n",i,inhi[i]->h,inhi[i]->x,inhi[i]->y,inhi[i]->passi);
                    count_p++;
#endif
                }
        }
    }
#ifdef debug_
    printf("ho settato i passi a %d persone\n",count_p);
#endif
    return;
}


/*! 
\brief Cancella le liste inhi e inlo.

Se una persona non viene pi&ugrave; rilevata allora vengono diminuite il suo numero di vite, 
la persona viene contata se esce dalla altra parte della scena, se ha fatto almeno 
PAS_MIN passi e se ha finito le vite. Altrimenti se la persona non ha pi&ugrave; vite viene eliminata.

\param trackin  numero di passeggeri entrati
\param trackout numero di passeggeri usciti
*/
void clearpeople(unsigned long &trackin,unsigned long &trackout, 
                 const unsigned short & door_threshold, const unsigned char & move_det_en,
                 const bool & count_true_false, const int & num_pers, 
                 const int & min_y_gap)
{
    for(int i=0;i<num_pers;++i)
    {
        if(inhi!=NULL)
        {
            if(inhi[i]!=NULL)
            {
                inhi[i]->life = max(0, inhi[i]->life-LIFE_DEC); //se la persona nello storico non e' stata traccata diminuisci di due le sue vite

                //la persona viene contata se esce dall'altra parte della scena
                //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                if(pers_to_be_counted(inhi[i], true, door_threshold, min_y_gap))
                {
                    if(move_det_en == 0 || count_true_false == true) 
                      people_count_input++;

#ifdef VERBOSE
                    printf("-> inhi:\n");
                    printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inhi[i]->ID);
#endif
                    delete inhi[i];
                    inhi[i]=NULL;
                }
                //se non ha piu' vite eliminala
                else if(inhi[i]->life<=0)
                {
#ifdef VERBOSE
                    printf("-> inhi:\n");
                    printf("--> blob_rep[%d]->ID = %ld [removed]\n\n", i, inhi[i]->ID);
#endif
                    delete inhi[i];
                    inhi[i]=NULL;
                }
            }
        }
        if(inlo!=NULL)
        {
            if(inlo[i]!=NULL)
            {
                inlo[i]->life = max(0, inlo[i]->life-LIFE_DEC); //se la persona nello storico non e' stata traccata diminuisci di due le sue vite

                if(pers_to_be_counted(inlo[i], false, door_threshold, min_y_gap))
                {
#ifdef debug_
                    printf("Clearpeople: contato inlo h=%d x=%d y=%d life=%d\n",inlo[i]->h,inlo[i]->x,inlo[i]->y,inlo[i]->life);
#endif
                    if(move_det_en == 0 || count_true_false == true) 
                      people_count_output++;
#ifdef VERBOSE
                    printf("-> inlo:\n");
                    printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inlo[i]->ID);
#endif
                    //delete [] inlo[i]; 
                    delete inlo[i]; // 20120221 bugfix
                    inlo[i]=NULL;
                }
                else if(inlo[i]->life<=0)
                {
#ifdef VERBOSE
                    printf("-> inlo:\n");
                    printf("--> blob_rep[%d]->ID = %ld [removed]\n\n", i, inlo[i]->ID);
#endif
                    //delete [] inlo[i];
                    delete inlo[i]; // 20120221 bugfix
                    inlo[i]=NULL;
                }
            }
        }
    }
    trackin = people_count_input;
    trackout = people_count_output;
    return; // a cosa serve un return se sono gia alla fine ???
}


/*! 
\brief Allocazione delle liste di strutture dati di tipo tPersonTracked (inhi e inlo) in base
       a configurazione (se normale #NUM_PERS_SING, se widegate allora #NUM_PERS_SING*#total_sys_number) e inizializzazione
       dei contatori usati nel tracking #people_count_input e #people_count_output.
\param pi numero di passeggeri entrati
\param po numero di passeggeri usciti
\param total_sys_number total number of PCN used (if in widegate this is greater than one otherwise it is one)
\param num_pers total number of possible tracked person
*/
void initpeople(unsigned long pi,unsigned long po, unsigned char & total_sys_number, int & num_pers)
{
    people_count_input=pi;
    people_count_output=po;
    
    if(total_sys_number==0) total_sys_number=1;
    
    if(inhi==NULL)
    {
        inhi = new tPersonTracked* [NUM_PERS_SING*total_sys_number];
        num_pers=NUM_PERS_SING*total_sys_number;
        for(int i=0;i<num_pers;i++)  inhi[i]=NULL;
    }
    if(inlo==NULL)
    {
        inlo = new tPersonTracked* [NUM_PERS_SING*total_sys_number];
        num_pers=NUM_PERS_SING*total_sys_number;
        for(int i=0;i<num_pers;i++)  inlo[i]=NULL;
    }
    if(total_sys_number!=num_pers/NUM_PERS_SING) // 20100623 eVS it was "num_pers/10"
    {
        delete [] inhi;
        delete [] inlo;
        inhi = new tPersonTracked* [NUM_PERS_SING*total_sys_number];
        inlo = new tPersonTracked* [NUM_PERS_SING*total_sys_number];
        num_pers=NUM_PERS_SING*total_sys_number;
        for(int i=0;i<num_pers;i++)
        {
            inhi[i]=NULL;
            inlo[i]=NULL;
        }
    }
    if(inhi==NULL) inhi = new tPersonTracked* [num_pers];
    if(inlo==NULL) inlo = new tPersonTracked* [num_pers];

    for(int i=0;i<num_pers;i++)
    {
        inhi[i]=NULL;
        inlo[i]=NULL;
    }
}


/*! 
\brief Deallocazione delle liste di strutture dati di tipo tPersonTracked (inhi e inlo).
*/
void deinitpeople(const int & num_pers)
{
#ifdef debug_
    printf("deinit people\n");
#endif
    for(int i=0;i<num_pers;++i)
    {
        if(inhi!=NULL)
            if(inhi[i]!=NULL) 
            {
                delete inhi[i];
                inhi[i]=NULL;	
            }
            if(inlo!=NULL)
                if(inlo[i]!=NULL) 
                {
                    delete inlo[i];
                    inlo[i]=NULL;
                }
    }
    if(inhi!=NULL) delete [] inhi;
    inhi=NULL;
    if(inlo!=NULL) delete [] inlo;
    inlo=NULL;
}


/*! 
\brief Gestione dell'evento porta chiusa.

La persona viene contata se esce dalla altra parte della scena e se ha fatto almeno PAS_MIN passi e se ha finito le vite.
          
*  \param trackin Numero delle persone contate in ingresso
*  \param trackout Numero delle persone contate in uscita
*  \param direction direzione del percorso: se vale 0 allora l'evento di ingresso di una 
                    persona &egrave; considerato dall'alto verso il basso
*/
void CloseDoor(unsigned long & trackin,unsigned long & trackout,
               const unsigned char & direction, const unsigned short & door_threshold, 
               const unsigned char & move_det_en,const bool & count_true_false,
               const int & num_pers, const int & min_y_gap)
{
    for(int i=0;i<num_pers;++i)
    {
#ifdef USE_NEW_STRATEGIES
        if(direction==0)
        {
            if(inhi!=NULL)
                if(inhi[i]!=NULL)
                {
                     // count person as entered if it is the case
                    if (inhi[i]->delta_y > BG_Y_MAX_DELTA && 
                        inhi[i]->num_frames >= MIN_NUM_FRAMES)
                    {
                      if(move_det_en == 0 || count_true_false == true)
                        people_count_input++;

                      // move the person from inhi to inlo in this way they will be able to be counted on exit
                      int n = find_a_free_element_in_repo(inlo, 0, num_pers);
                      if (n>=0 && n<num_pers)
                      {
                        inlo[n] = inhi[i];

                        inlo[n]->first_y = NY; //door_threshold+DELTA_DOOR_TH+1;
                        inlo[n]->first_x = inlo[n]->x;
                        inlo[n]->cont = true;
                        inlo[n]->delta_y = (inlo[n]->y-inlo[n]->first_y);
                      }
#ifdef _DEBUG
                      else
                        printf("Negative idx at 747!\n");
#endif
                    }
                    else                  
                      delete inhi[i];

                    inhi[i]=NULL;
                  }
        }
        else // direction==1
        {
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                     // count person as entered if it is the case
                    if (-inlo[i]->delta_y > BG_Y_MAX_DELTA && 
                         inlo[i]->num_frames >= MIN_NUM_FRAMES)
                    {
                      if(move_det_en == 0 || count_true_false == true) 
                        people_count_output++;
                      
                      // move the person from inlo to inhi in this way they will be able to be counted on exit
                      int n = find_a_free_element_in_repo(inhi, 0, num_pers);
                      if (n>=0 && n<num_pers)
                      {
                        inhi[n] = inlo[i];

                        inhi[n]->first_y = 0; //door_threshold-DELTA_DOOR_TH-1;
                        inhi[n]->first_x = inhi[n]->x;
                        inhi[n]->cont = true;
                        inhi[n]->delta_y = (inhi[n]->y-inhi[n]->first_y);
                      }
#ifdef _DEBUG
                      else
                        printf("Negative idx at 788!\n");
#endif
                    }
                    else                  
                      delete inlo[i];

                    inlo[i]=NULL;
                  }
        }
    }
#else
        if(direction==0)
        {
            if(inhi!=NULL)
                if(inhi[i]!=NULL)
                {
                    //la persona viene contata se esce dall'altra parte della scena
                    //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                    if(pers_to_be_counted(inhi[i], true, door_threshold, min_y_gap, true) 
                       )
                    {
                        // if(move_det_en == 0 || count_true_false == true)  //20130515 eVS. After Heathrow problem remove this condition in order to enable count in door closed event
                          people_count_input++;
#ifdef VERBOSE
                        printf("-> inhi:\n");
                        printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inhi[i]->ID);
#endif
                        delete inhi[i];
                        inhi[i]=NULL;
                    }
                }
        }
        else // direction==1
        {
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                    //la persona viene contata se esce dall'altra parte della scena
                    //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                    if(pers_to_be_counted(inlo[i], false, door_threshold, min_y_gap, true)
                      )
                    {
                      //  if(move_det_en == 0 || count_true_false == true)  //20130515 eVS.
                          people_count_output++;
#ifdef VERBOSE
                        printf("-> inlo:\n");
                        printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inlo[i]->ID);
#endif
                        delete inlo[i];
                        inlo[i]=NULL;
                    }
                }
        }
    }
#endif

    trackin=people_count_input;
    trackout=people_count_output;

#ifndef USE_NEW_STRATEGIES
    deinitpeople(num_pers);
#endif
}


void
compute_cost_mat(tPersonTracked** blob_rep, const int blob_rep_len,
                 const int* people, const unsigned char* hpers, const unsigned char* dimpers, const int person,
                 bool *blob_rep_active, int &blob_rep_active_num, bool *person_active, int &person_active_num, const int xt,
                 unsigned int* const & cost_mat)
{
  const int fvlen = 5;

  // init output data
  for (int i=0; i<blob_rep_len*person; ++i)
    cost_mat[i] = MAX_COST;
  for (int i=0; i<blob_rep_len; ++i)
    blob_rep_active[i] = false;
  for (int i=0; i<person; ++i)
    person_active[i] = false;

  blob_rep_active_num = 0; // number of blobs in blob_rep that have a possible matching with a detected person

  // check for possible correspondeces, i.e., two blobs at distance less than 30 pixels, and
  // compute their cost
  for(int i=0;i<blob_rep_len;++i) //per tutte le persone nello storico
  {
    if (blob_rep[i] != NULL)
    {
      int  feature_vect_in[fvlen]  = {blob_rep[i]->x, blob_rep[i]->y, blob_rep[i]->h, blob_rep[i]->wx, blob_rep[i]->wy};
      int  offset = i*person;
      int  current_num_possible_match = 0; // possible matches between blob_rep[i] and a detected person

      for(int p=0;p<person;++p) //per ogni persona trovata in questo frame
      {
        if (hpers[p] > 0) // se la persona e' nella tracking-zone
        {
          // compute squared euclidean distance
          int xy[2] = {people[p]%xt, people[p]/xt};
          unsigned int eucl_dist2 = 0;
          for (int j=0; j<2; ++j)
          {
            int diff =(xy[j] - feature_vect_in[j]);
            eucl_dist2 += diff*diff;
          }


          //int max_h_diff = max(MAX_H_DIFF_MIN, (feature_vect_in[2]*MAX_H_DIFF_PERC)/100);
          //int max_h_diff = 4*blob_rep[i]->std_h+MAX_H_DIFF_MIN;
          //int diff_x_perc = abs(feature_vect_in[0] - NX/2)/3;
          float lrbdist = ((feature_vect_in[0] < NX/2) ? feature_vect_in[0] : NX-feature_vect_in[0])-12;
          float tbbdist = abs(feature_vect_in[1] - NY/2);
          int diff_x_perc = (int)(40.0f*exp(-(lrbdist*lrbdist/400.0f+tbbdist*tbbdist/800.0f)));
          //if (diff_x_perc > 25)
          //  printf("Ops\n");

          int max_h_diff  = max(MAX_H_DIFF_MIN, (blob_rep[i]->h*(diff_x_perc+MIN_H_DIFF_PERC))/100);
          int max_mh_diff = max(MAX_H_DIFF_MIN, (blob_rep[i]->max_h*(diff_x_perc+MIN_H_DIFF_PERC))/100);

          int diff_h = abs(hpers[p]-feature_vect_in[2]);
          int diff_mh = abs(blob_rep[i]->max_h-hpers[p]); 

          ////int max_wx_diff = max(MAX_WX_DIFF_MIN, ((2*feature_vect_in[3]+1)*MAX_WX_DIFF_PERC)/100);
          //int max_wx_diff = (blob_rep[i]->is_bg) ? MAX_WX_DIFF_MIN : max(MAX_WX_DIFF_MIN, ((2*feature_vect_in[3]+1)*(diff_x_perc+MIN_WX_DIFF_PERC))/100);
          //int diff_wx = 2*abs(dimpers[2*p]-feature_vect_in[3]);

          ////int max_wy_diff = max(MAX_WY_DIFF_MIN, ((2*feature_vect_in[4]+1)*MAX_WY_DIFF_PERC)/100);
          //int max_wy_diff = (blob_rep[i]->is_bg) ? MAX_WY_DIFF_MIN : max(MAX_WY_DIFF_MIN, ((2*feature_vect_in[4]+1)*(diff_x_perc+MIN_WY_DIFF_PERC))/100);
          //int diff_wy = 2*abs(dimpers[2*p+1]-feature_vect_in[4]);

          unsigned int max_dist2 = ((blob_rep[i]->max_h*2+FROM_DISP_TO_HEAD/2) / FROM_DISP_TO_HEAD); // this is a round instead of truncate
          max_dist2 *= max_dist2;

          // check if correspondence is possible
          if(//eucl_dist2 <= MAX_BLOB_DIST_SQUARED &&  // se e' vicina a quella nello storico
             eucl_dist2 <= max_dist2 &&  // se e' vicina a quella nello storico
             diff_h <= max_h_diff && // se e' alta in modo simile a quella nello storico
             //(blob_rep[i]->max_h-hpers[p]) <= (blob_rep[i]->max_h*MAX_H_DIFF_PERC/100) && // voglio evitare di traccare i piedi
             diff_mh <= max_mh_diff) // && // voglio evitare di traccare i piedi
             //diff_wx <= max_wx_diff && // se la sua testa e' larga in modo simile a quella nello storico
             //diff_wy <= max_wy_diff) // se la sua testa e' alta in modo simile a quella nello storico
          {
            person_active[p] = true;
            current_num_possible_match++;

            // compute final cost
            //int dist = eucl_dist2 + diff_h*diff_h + diff_wx*diff_wx + diff_wy*diff_wy + (blob_rep[i]->is_bg ? eucl_dist2/10 : 0);
            unsigned int dist = eucl_dist2 + diff_h*diff_h; // + (blob_rep[i]->is_bg ? eucl_dist2/20 : 0);

#ifdef _DEBUG
            assert(dist <= INT_MAX); //aggiunti check per evitare overflow nel calcolo del costo
#endif

            if (dist > MAX_COST) //aggiunto check per evitare overflow nel calcolo del costo
              dist = MAX_COST-1;

            cost_mat[offset + p] = dist;
          }
        }
      }

      if (current_num_possible_match > 0)
      {
        blob_rep_active[i] = true;
        blob_rep_active_num++;
      }
    }
  }

  person_active_num = 0;
  for (int p=0; p<person; ++p)
    if (person_active[p])
      person_active_num++;
}


void
find_best_match(const unsigned int* const & cost_mat, const int rows, const int cols, 
                const bool* rep_active, const int num_rep_active, // cost matrix rows
                const bool* pers_active, const int num_pers_active, // cost matrix cols
                int* const & best_match)
{
  // init result
  for (int r=0; r<rows; ++r)
    best_match[r] = -1;

  if (num_pers_active > 0 && num_rep_active > 0)
  {

    static unsigned int cost_mat_sub[NUM_PERS_SING*(MAX_NUM_SLAVES+1)*NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    static unsigned int cost_mat_r[NUM_PERS_SING*(MAX_NUM_SLAVES+1)*NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    static unsigned int cost_mat_c[NUM_PERS_SING*(MAX_NUM_SLAVES+1)*NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    
    int subnrow = 0; 
    int subncol = 0;

    // hungarian implementation wants a square matrix
    // if the matrix is not square we can just pad it with a
    // added row or column filled with 0
    int nelem = max(num_rep_active, num_pers_active);
    for (int i=0; i<nelem*nelem; ++i)
      cost_mat_sub[i] = MAX_COST;
    if (num_rep_active != num_pers_active)
    {
      // if not square, pad with zeros where needed
      if (num_rep_active > num_pers_active) // subrows > subcols
      {
        // pad columns
        for (int r=0; r<nelem; ++r)
          for (int c=num_pers_active; c<nelem; ++c)
            cost_mat_sub[r*nelem+c] = 0;
      }
      else // subrows < subcols
      {
        // pad rows
        for (int r=num_rep_active; r<nelem; ++r)
          for (int c=0; c<nelem; ++c)
            cost_mat_sub[r*nelem+c] = 0;
      }
    }

    for (int r=0; r<rows; ++r) // iterate on the blobs in the repository
    {
      if (rep_active[r])
      {
        int offset = r*cols;
        int suboffset = subnrow*nelem;

        subncol = 0;
        for (int c=0; c<cols; ++c)
        {
          if (pers_active[c])
          {
            cost_mat_sub[suboffset+subncol] = cost_mat[offset+c];
            cost_mat_r[suboffset+subncol] = r;
            cost_mat_c[suboffset+subncol] = c;
            subncol++;
          }
        }
        subnrow++;
      }
    }

    assert(subncol == num_pers_active);
    assert(subnrow == num_rep_active);

    static int M[NUM_PERS_SING*(MAX_NUM_SLAVES+1)*NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    matching(cost_mat_sub, nelem, nelem, MAX_COST, M);

    // fill with the matching results
    for (int r=0; r<num_rep_active; ++r)
    {
      int offset = r*nelem;
      for (int c=0; c<num_pers_active; ++c)
      {
        int idx_orig = cost_mat_r[offset+c]*cols + cost_mat_c[offset+c];

        if ( (M[offset+c] == 1) && (cost_mat[idx_orig] != MAX_COST) )
        {
          best_match[cost_mat_r[offset+c]] = cost_mat_c[offset+c];
        }
      }
    }
  }
}


void
update_rep_data(tPersonTracked** blob_rep, int* best_match, const int num_pers, bool* non_trovate, 
                const int* people, const unsigned char* hpers, const unsigned char* dimpers, const int xt, bool is_from_high)
{
  assert(num_pers > 0);

  for (int i=0; i<num_pers; ++i)
  {
    if (best_match[i] >= 0)
    {
      const int p = best_match[i];
      non_trovate[p] = false;
      blob_rep[i]->trac=true;

      blob_rep[i]->life=min(blob_rep[i]->life+1, LIFE);

      int px = (people[p]%xt);
      int py = (people[p]/xt);

      int prev_px = blob_rep[i]->x;
      int prev_py = blob_rep[i]->y;

      int diffx, diffy;

      diffx = prev_px - px;
      diffy = prev_py - py;

      //int diff_h = blob_rep[i]->h - hpers[p];
      if (blob_rep[i]->h > 120)
      {
        blob_rep[i]->x  = px;
        blob_rep[i]->y  = py;
      }
      else
      {
        blob_rep[i]->x  = (blob_rep[i]->x  + 3*px)/4;
        blob_rep[i]->y  = (blob_rep[i]->y  + 3*py)/4;
      }

      {
        const int fact = 3;
        blob_rep[i]->h  = (blob_rep[i]->h + fact*hpers[p])/(fact+1);
        
        if (blob_rep[i]->h > blob_rep[i]->max_h)
          blob_rep[i]->max_h = (blob_rep[i]->max_h + fact*blob_rep[i]->h)/(fact+1);

        blob_rep[i]->wx = (blob_rep[i]->wx + fact*dimpers[2*p])/(fact+1);
        blob_rep[i]->wy = (blob_rep[i]->wy + fact*dimpers[2*p+1])/(fact+1);
      }

      blob_rep[i]->delta_y += (blob_rep[i]->y-prev_py); //blob_rep[i]->first_y);

      blob_rep[i]->num_frames++;
    }
  }
}


void
solve_conflicts(tPersonTracked** inhi, tPersonTracked**inlo,
                int* best_match_hi, int* best_match_lo, const int num_pers,
                unsigned int* cost_mat_hi, unsigned int* cost_mat_lo,
                const int* people, const unsigned char* hpers, const unsigned char* dimpers, const int person, const int xt)
{
  for (int hi_idx=0; hi_idx<num_pers; ++hi_idx)
  {
    if (best_match_hi[hi_idx] >= 0)
    {
      for (int lo_idx=0; lo_idx<num_pers; ++lo_idx)
      {
        if (best_match_lo[lo_idx] >= 0)
        {
          if (best_match_hi[hi_idx] == best_match_lo[lo_idx])
          {
            int c = best_match_hi[hi_idx]; // == best_match_lo[lo_idx]
            int cost_lo = cost_mat_lo[lo_idx*person + c];
            int cost_hi = cost_mat_hi[hi_idx*person + c];

            assert(cost_lo != MAX_COST);
            assert(cost_hi != MAX_COST);

            if (cost_hi < (75*cost_lo)/100)
            {
              best_match_lo[lo_idx] = -1;
            }
            else if (cost_lo < (75*cost_hi)/100)
            {
              best_match_hi[hi_idx] = -1;
            }
            else if (abs(inhi[hi_idx]->delta_y) > 2*abs(inlo[lo_idx]->delta_y) && inhi[hi_idx]->num_frames > MIN_NUM_FRAMES)
            {
              best_match_lo[lo_idx] = -1;
            }
            else if (abs(inlo[lo_idx]->delta_y) > 2*abs(inhi[hi_idx]->delta_y) && inlo[lo_idx]->num_frames > MIN_NUM_FRAMES)
            {
              best_match_hi[hi_idx] = -1;
            }
            else
            {
              const int p = best_match_hi[hi_idx]; // == best_match_lo[lo_idx]
              int px = (people[p]%xt);
              int py = (people[p]/xt);

              int diffx_hi = inhi[hi_idx]->x - px;
              int diffy_hi = inhi[hi_idx]->y - py;
              int dist_hi = diffx_hi*diffx_hi + diffy_hi*diffy_hi;

              int diffx_lo = inlo[lo_idx]->x - px;
              int diffy_lo = inlo[lo_idx]->y - py;
              int dist_lo = diffx_lo*diffx_lo + diffy_lo*diffy_lo;

              if (dist_hi > dist_lo)
                best_match_hi[hi_idx] = -1;
              else if (dist_hi < dist_lo)
                best_match_lo[lo_idx] = -1;
              else
              {
                if (abs(diffy_hi) >= abs(diffy_lo))
                  best_match_hi[hi_idx] = -1;
                else 
                  best_match_lo[lo_idx] = -1;
              }
            }
          }
        }
      }
    }
  }
}


int find_a_free_element_in_repo(tPersonTracked ** repo, const int person, const int num_pers)
{
  int n=-1;
  if (repo != NULL)
  {
    n=person;
    if(n<num_pers)
    {
      while(repo[n]!=NULL) //trovare la prima libera prima in avanti e poi...
      {
        n++;
        if(n>=num_pers)
        {
          n=-1;
          break;
        }
      }
    }

    if(n>=num_pers) //... indietro garantisce nella maggior parte dei casi di assegnare
    {               //alla persona presente da + tempo nel tracciato (="+ vecchia") =>LIFO
      n=0;
      while(repo[n]!=NULL)
      {
        n++;
        if (n>=num_pers) 
        {
          n=-1;
          break;
        }
      }
    }
  }
  return n;
}


/*! 
\brief Algoritmo di tracking delle persone.

Tracking delle persone che attraversano la zona monitorata e conteggio dei passeggeri
entrati ed usciti secondo il seguente schema 
(per ulteriori dettagli vedere i relativi frammenti del codice sorgente sotto riportati): 
- calcolo delle distanze minime tra ciascuna persona presente nelle liste #inhi e #inlo 
con tutte le persone trovate nel frame corrente, 
- controllo possibile persona entrata dalla zona bassa/alta dell'area e confronto con posizione 
(#inhi->wx,#inhi->wy e #inlo->wx,#inlo->wy) e altezza precedenti (#inhi->h e #inlo->h) 
per capire se si tratta della stessa persona (blob matching) per 
aggiornare la posizione (#inhi->x,#inhi->y e #inlo->x,#inlo->y), l'altezza e le dimensioni dei blob relativo, oltre al numero 
di vite ad essa associate (se ha compiuto un certo numero di passi all'interno 
della scena), 
- inserimento delle persone trovate nella lista #inhi e #inlo (se non sono gia state inserite in precedenza), 
- analisi dello stato della persona (se &egrave; in tracking oppure no):
se &egrave; entrata dal basso piuttosto che dall'alto e se ha superato la soglia porta 
orizzontale (di default #door_threshold si trova a met&agrave; immagine) ed eventuale conteggio della 
persona entrata/uscita.

*  \param people coordinate del centroide (see "people_coor" used at the end of detectAndTrack())
*  \param person numero di persone trovate nel frame corrente
*  \param disparityMap mappa di disparit&agrave; letta dall'FPGA (di 16 livelli riscalati da 0 a 255)
*  \param dimpers dimensione delle persone trovate (see "dimpers" used at the end of detectAndTrack())
*  \param hpers altezza delle persone trovate (see "hpers" used at the end of detectAndTrack())
*  \param trackin numero delle persone entrate
*  \param trackout numero delle persone uscite
*  \param diff_cond_1p se vale 1 allora tutto ci&ograve; che &egrave; presente nella scena viene considerato un unico blob
*  \param direction_inout se vale 0 allora il senso di marcia &egrave; dall'alto verso il basso, ovvero il contenuto di #people_dir nonche' il parametro identificato dalla stringa "dir" (\ref tabella_parms).
*  \param current_sys_number used in widegate to identify each PCN in the chain
*  \param total_sys_number used in widegate for the total number of PCN connected in the chain
*  \param door_stairs_en 
*  \param move_det_en move detection on/off
*  \param count_true_false counting enabled/disabled
*  \param num_pers total number of possible counted persons (in widegate this is the single maxima multiplied by the number of sensors)
*  \param min_y_gap minimun delta y that a blob has to move in order to be counted
*/
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
           const int &min_y_gap)
{
#ifdef VERBOSE
    num_frame++;
#endif
    
    /////////////////////////////////////////////////////////////////////////
    // inizializzazione strutture dati per il tracking

    // se e' la prima volta devo allocare e inizializzare inlo e inhi
    if(total_sys_number<2) 
    { 
        xt=160;
        num_pers=NUM_PERS_SING;
    }
    else 
    {
        xt=160*total_sys_number;
        num_pers=NUM_PERS_SING*total_sys_number;
    }

    if(inhi==NULL)
    {
        inhi = new tPersonTracked* [num_pers];
        for(int i=0;i<num_pers;++i)  inhi[i]=NULL;
    }
    if(inlo==NULL)
    {
        inlo = new tPersonTracked* [num_pers];
        for(int i=0;i<num_pers;++i)  inlo[i]=NULL;
    }

#ifdef debug_
    for(int i=0;i<person;++i)
    {
        int px=people[i]%xt;
        int py=people[i]/xt;
        int h=hpers[i];
        int wx=dimpers[2*i+0]; //20110930 eVS, bugfix of indexes
        int wy=dimpers[2*i+1];
        if(px+wx>=NX) printf("x=%d y=%d wx=%d wy=%d h=%d \n",px,py,wx,wy,h);
    }
    int libere_lo=0;
    int libere_hi=0;
    printf("Inizio peopletrack\n");
#endif

    //suppongo nessuna persona nel tracciato
    for(int i=0;i<num_pers;++i)
    {
        if(inhi[i]!=NULL) inhi[i]->trac=false;
        if(inlo[i]!=NULL) inlo[i]->trac=false;
    }

    bool * non_trovate = new bool [person];
    for (int r=0;r<person; r++) 
      non_trovate[r]=true;

    // fine inizializzazione
    /////////////////////////////////////////////////////////////////////////

#ifdef debug_
    for(int k=0;k<num_pers;k++)
    {
        if(inlo[k]!=NULL)
        {
            if(inlo[k]->x + inlo[k]->wx>xt)
            { 
                printf("Prima check presenza: k=%d x=%d wx=%d\n",k,inlo[k]->x,inlo[k]->wx);
            } 
        }
        if(inhi[k]!=NULL)
        {
            if(inhi[k]->x + inhi[k]->wx>xt)
            { 
                printf("Prima check presenza: k=%d x=%d wx=%d\n",k,inhi[k]->x,inhi[k]->wx);
                // inhi[k]->x=80;
            } 
        }
    }
#endif

    // static allocation at maximum size
    static unsigned int  cost_mat_hi[NUM_PERS_SING*(MAX_NUM_SLAVES+1) * NUM_PERS_SING*(MAX_NUM_SLAVES+1)]; 
    static int  best_match_hi[NUM_PERS_SING*(MAX_NUM_SLAVES+1)]; // 0..person<=NUM_PERS_SING*(MAX_NUM_SLAVES+1)
    static bool inhi_active[NUM_PERS_SING*(MAX_NUM_SLAVES+1)];   // 0..num_pers<=NUM_PERS_SING*(MAX_NUM_SLAVES+1)
    static bool pers_active_hi[NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    int inhi_active_num, pers_active_hi_num;

    compute_cost_mat(inhi, num_pers,
      people, hpers, dimpers, person,
      inhi_active, inhi_active_num, 
      pers_active_hi, pers_active_hi_num, 
      xt,
      cost_mat_hi);

    //if (inhi_active_num > 2 && pers_active_hi_num > 2 && inhi_active_num != pers_active_hi_num)
    //  printf("Stop here!\n");

    find_best_match(cost_mat_hi, num_pers, person, 
      inhi_active, inhi_active_num, 
      pers_active_hi, pers_active_hi_num, 
      best_match_hi);

    // static allocation at maximum size
    static unsigned int  cost_mat_lo[NUM_PERS_SING*(MAX_NUM_SLAVES+1) * NUM_PERS_SING*(MAX_NUM_SLAVES+1)]; 
    static int  best_match_lo[NUM_PERS_SING*(MAX_NUM_SLAVES+1)]; // 0..person<=NUM_PERS_SING*(MAX_NUM_SLAVES+1)
    static bool inlo_active[NUM_PERS_SING*(MAX_NUM_SLAVES+1)]; // 0..num_pers<=NUM_PERS_SING*(MAX_NUM_SLAVES+1)
    static bool pers_active_lo[NUM_PERS_SING*(MAX_NUM_SLAVES+1)];
    int inlo_active_num, pers_active_lo_num;

    compute_cost_mat(inlo, num_pers,
      people, hpers, dimpers, person,
      inlo_active, inlo_active_num,
      pers_active_lo, pers_active_lo_num, xt,
      cost_mat_lo);

    //if (inlo_active_num > 2 && pers_active_lo_num > 2 && inlo_active_num != pers_active_lo_num)
    //  printf("Stop here!\n");

    find_best_match(cost_mat_lo, num_pers, person, 
      inlo_active, inlo_active_num, 
      pers_active_lo, pers_active_lo_num, 
      best_match_lo);

    if (inhi_active_num && inlo_active_num) // if there were possible matching in both inhi and inlo then...
    {
      // verify possible conflicts in the matching, i.e., a person in inhi matches the same one matched by a person in inlo
      solve_conflicts(inhi, inlo,
        best_match_hi, best_match_lo, num_pers, 
        cost_mat_hi, cost_mat_lo, 
        people, hpers, dimpers, person, xt);
    }

    if (inhi_active_num) // if someone in the inhi list could match then update repository on the matches
    {
#ifdef VERBOSE
      printf("-> inhi:\n");
#endif
      update_rep_data(inhi, best_match_hi, num_pers, 
        non_trovate, people, hpers, dimpers, xt, true);
    }

    if (inlo_active_num) // if someone in the inlo list could match then update repository on the matches
    {
#ifdef VERBOSE
      printf("-> inlo:\n");
#endif
      update_rep_data(inlo, best_match_lo, num_pers, 
        non_trovate, people, hpers, dimpers, xt, false);
    }


#ifdef debug
    for(int k=0;k<num_pers;k++)
    {
      if(inlo[k]!=NULL)
      {
        if(inlo[k]->x + inlo[k]->wx>xt)
        { 
          printf("Dopo check presenza: k=%d x=%d wx=%d\n",k,inlo[k]->x,inlo[k]->wx);
          inlo[k]->x=80;
        } 
      }
      if(inhi[k]!=NULL)
      {
        if(inhi[k]->x + inhi[k]->wx>xt)
        { 
          printf("Dopo check presenza: k=%d x=%d wx=%d\n",k,inhi[k]->x,inhi[k]->wx);
          inhi[k]->x=80;
        } 
      }
    }
#endif


    //se nel fotogramma corrente ci sono persone "nuove" (cioe' che non sono state abbinate a nessuna
    //persona presente in inhi e inlo) allora le aggiunge a inhi o a inlo a seconda di dove sono 
    //localizzate (sopra o sotto soglia)
    for (int t=0; t<person;t++)
    {
      if(non_trovate[t] //)
         && hpers[t] > 0) // 20111006 eVS bugfix per NTZ
      {
        int px=people[t]%xt;
        int py=people[t]/xt;

        // inserisco le persone entrate dalla parte alta nella lista delle persone da traccare
        // (si tratta delle persone che entrano sull'autobus se la direction e' 0, mentre son 
        // quelle che escono se direction e' 1)
        if(py<door_threshold-DELTA_DOOR_TH)
        {
          int n = find_a_free_element_in_repo(inhi, person, num_pers);

          //se alla fine ha trovato un indice libero inferiore a NUM_PERS
          if(n>=0 && n<num_pers)//controllo di puntare ad una zona corretta
          {

            inhi[n]=new tPersonTracked;
            inhi[n]->cont=false;
            inhi[n]->trac=true;

#if defined(VERBOSE) || defined(_DEBUG)
            static unsigned long blob_counter = 0;
            inhi[n]->ID = blob_counter;
            blob_counter++;
#endif

#ifdef VERBOSE
            inhi[n]->first_frame=num_frame;
#endif

            inhi[n]->wx=dimpers[2*t]; // 20110930 eVS, bug fix on indexes
            inhi[n]->wy=dimpers[2*t+1];
            inhi[n]->h=hpers[t];

            inhi[n]->num_frames=1;
            inhi[n]->life=LIFE/2;
            inhi[n]->delta_y=0;
            inhi[n]->max_h=hpers[t];

            inhi[n]->x=px;
            inhi[n]->y=py;

            inhi[n]->first_x=px;
            if (door_threshold < NY) // se NON sono in fase di apertura o chiusura porta
              inhi[n]->first_y=py;
            else
              inhi[n]->first_y=0;

            //    printf("Inserita nuova inhi, y nascita %d\n",inhi[n]->first_y);
#ifdef debug
            if(inhi[n]->x>xt) printf("Nuove pers: x=%d n=%d \n",inhi[n]->x,n);
            if(inhi[n]->y>YT) printf("Nuove pers: y=%d n=%d \n",inhi[n]->y,n);
#endif
          }
        }

        // inserisco le persone entrate dalla parte bassa nella lista delle persone da traccare
        // (si tratta delle persone che escono dall'autobus)
        if(py>door_threshold+DELTA_DOOR_TH)
        {
          int n = find_a_free_element_in_repo(inlo, person, num_pers);

          //se alla fine ha trovato un indice libero inferiore a NUM_PERS
          if(n>=0 && n<num_pers)
          {

            inlo[n]=new tPersonTracked;
            inlo[n]->cont=false;
            inlo[n]->trac=true;

#if defined(VERBOSE) || defined(_DEBUG)
            static unsigned long blob_counter = 0;
            inlo[n]->ID = blob_counter;
            blob_counter++;
#endif

#ifdef VERBOSE
            inlo[n]->first_frame=num_frame;
#endif

            inlo[n]->num_frames=1;
            inlo[n]->life=LIFE/2;
            inlo[n]->delta_y=0;
            inlo[n]->max_h=hpers[t];

            inlo[n]->h=hpers[t];

            inlo[n]->wx=dimpers[2*t]; // 20110930 eVS, bug fix on indexes
            inlo[n]->wy=dimpers[2*t+1];
            inlo[n]->x=px;
            inlo[n]->y=py;

            inlo[n]->first_x=px;
            if (door_threshold > 0) // se NON sono in fase di apertura o chiusura porta
              inlo[n]->first_y=py;
            else
              inlo[n]->first_y=NY;
            //      printf("Inserita nuova inlo, y nascita %d\n",inlo[n]->first_y);
#ifdef debug
            if(inlo[n]->x>xt) printf("Nuove pers: x=%d n=%d \n",inlo[n]->x,n);
            if(inlo[n]->y>YT) printf("Nuove pers: y=%d n=%d \n",inlo[n]->y,n);
#endif
          }
        }
      }
    }


#ifdef debug
    for(int k=0;k<num_pers;k++)
    {
      if(inlo[k]!=NULL)
      {
        if(inlo[k]->x + inlo[k]->wx>xt)
        { 
          printf("Dopo ins nuove: k=%d x=%d wx=%d\n",k,inlo[k]->x,inlo[k]->wx);
          // inlo[k]->x=80;
        } 
        if(inlo[k]->y>YT) printf("Dopo ins nuove: y=%d k=%d \n",inlo[k]->y,k);
      }
      if(inhi[k]!=NULL)
      {
        if(inhi[k]->x + inhi[k]->wx>xt)
        { 
          printf("Dopo ins nuove: k=%d x=%d wx=%d\n",k,inhi[k]->x,inhi[k]->wx);
          // inhi[k]->x=80;
        } 
        if(inhi[k]->y>YT) printf("Dopo ins nuove: y=%d k=%d \n",inhi[k]->y,k);
      }
    }
#endif

#ifdef USE_CONSISTENCY_CHECK
    // verifico che non ci siano blob nello storico che siano nella stessa posizione
    _history_consistency_check(inhi, inlo, num_pers, door_threshold, min_y_gap);
#endif

    // determina quali persone diventano candidate per il conteggio
    // cioe' se sono traccate e se hanno sorpassato la soglia
    for (int l=num_pers-1;l>=0;l--)
    {
      if(inhi[l]!=NULL)
      {
        if(inhi[l]->cont==false && inhi[l]->trac==true && inhi[l]->y > door_threshold+DELTA_DOOR_TH)
          inhi[l]->cont=true;
      }
    }

    for (int l=num_pers-1;l>=0;l--)
    {
      if(inlo[l]!=NULL)
      {
        if(inlo[l]->cont==false && inlo[l]->trac==true && inlo[l]->y < door_threshold-DELTA_DOOR_TH)
          inlo[l]->cont=true;
      }
    }


    // conteggio delle persone
    for(int i=0;i<num_pers;++i)
    {
      if(inhi[i]!=NULL)
      {
        if(inhi[i]->trac==false)
        {
          inhi[i]->life = max(0, inhi[i]->life-LIFE_DEC); //se la persona nello storico non e' stata traccata diminuisci di due le sue vite

          if(inhi[i]->life<=0)
          {
#ifdef debug
            printf("contato inhi h=%d x=%d y=%d life=%d\n",inhi[i]->h,inhi[i]->x,inhi[i]->y,inhi[i]->life);
#endif 
            // incremento il contatore delle persone entrate solo se
            // il motion detection e' disabilitato oppure se e' abilitato e c'e' "movimento" a meno di 200 frame di 
            // potenziale inattivita' (vedi loops.cpp funzione main_loop)
            if (pers_to_be_counted(inhi[i], true, door_threshold, min_y_gap))
            {
              if(move_det_en == 0 || count_true_false == true)
                people_count_input++;
#ifndef VERBOSE
            }
#else
              printf("-> inhi:\n");
              printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inhi[i]->ID);
            }
            else
            {
              printf("-> inhi:\n");
              printf("--> blob_rep[%d]->ID = %ld [removed]\n\n", i, inhi[i]->ID);
            }
#endif

            // if life is zero we have to remove the blob from the repository
            delete inhi[i];
            inhi[i]=NULL;
          }
        }
      }
    }

    for(int i=0;i<num_pers;++i)
    {
      if(inlo[i]!=NULL)
      {
        if(inlo[i]->trac==false)
        {
          inlo[i]->life = max(0, inlo[i]->life-LIFE_DEC); //se la persona nello storico non e' stata traccata diminuisci di due le sue vite

          if(inlo[i]->life<=0)
          {
#ifdef debug
            printf("contato inlo h=%d x=%d y=%d life=%d\n",inlo[i]->h,inlo[i]->x,inlo[i]->y,inlo[i]->life);
#endif
            if(pers_to_be_counted(inlo[i], false, door_threshold, min_y_gap))
            {
              if(move_det_en == 0 || count_true_false == true) 
                people_count_output++;
#ifndef VERBOSE
            }
#else
              printf("-> inlo:\n");
              printf("--> blob_rep[%d]->ID = %ld [counted]\n\n", i, inlo[i]->ID);
            }
            else
            {
              printf("-> inlo:\n");
              printf("--> blob_rep[%d]->ID = %ld [removed]\n\n", i, inlo[i]->ID);
            }
#endif

            delete inlo[i];
            inlo[i]=NULL;
          }
        }
      }
    }

    //disegna le persone
    if(total_sys_number<2)
    {
        for(int i=num_pers-1;i>=0;--i)
            if(inhi[i]!=NULL)
                draw_cross_on_map(inhi[i], disparityMap);

        for(int i=num_pers-1;i>=0;--i)
            if(inlo[i]!=NULL)
                draw_cross_on_map(inlo[i], disparityMap);
    }

#ifdef USE_HANDLE_OUT_OF_RANGE
    // Se ho una situazione di OOR recupero le coordinate e il raggio e pulisco i picchi spuri presenti fuori dal raggio
    if (OutOfRangeManager::getInstance().IsOutOfRange())
    {
      int cent_r = OutOfRangeManager::getInstance().GetRow()*binning+BORDER_Y;
      int cent_c = OutOfRangeManager::getInstance().GetCol()*binning+BORDER_X;
      int ray = OutOfRangeManager::getInstance().GetRay()*binning;
      _clear_blobs_around_virtual_blob(inhi, num_pers, inlo, num_pers, cent_r, cent_c, ray);
    }
#endif

    trackin = people_count_input;
    trackout = people_count_output;

    delete [] non_trovate;

    return;

    /*!
    <b>Calcolo delle distanze minime</b>
    \code
    // Calcolo delle distanze minime scandendo tutti i blob contenuti nelle liste "inhi" e "inlo"
    // e confronto con i blob trovati dall'algoritmo di people detection nel frame corrente.
    for(int i=0;i<NUM_PERS_SING*current_sys_number;++i)
    {
        ind_pershi=21; /// 21 va bene solo se il numero totale di sistemi collegati in serie e pari a 2 ???
                        /// qual'e il numero massimo di sistemi che possono essere collegati in wide-gate ???
        ind_perslo=21; /// 21 va bene solo se il numero totale di sistemi collegati in serie e pari a 2 ???
        distmin_inlo=2000; /// perche 2000 ???
        distmin_inhi=2000; /// perche 2000 ???

        for(int p=0;p<person;p++) //per ogni persona trovata in questo frame
        {
            if(non_trovate[p]) // se non e' gia' stata assegnata
            {
                //coordinate persone al frame corrente
                int px=people[p]%xt;
                int py=people[p]/xt;

                int dist_tmp;

                //calcola la distanza min delle inlo
                if(inlo[i]!=NULL)
                {
                    int ux=inlo[i]->x;
                    int uy=inlo[i]->y;

                    dist_tmp=(px-ux)*(px-ux)+(py-uy)*(py-uy);
                    if(dist_tmp<distmin_inlo)
                    {
                        distmin_inlo=dist_tmp;
                        ind_perslo=p;
                    }
                }
                //...
    \endcode

        
    <b>BLOB MATCHING</b>
    \code
    // Dopo che l'algoritmo ha trovato tutte le distanze minime tra vecchi blob e nuovi blob, 
    // viene selezionato il nuovo blob piu' vicino che soddisfi la condizione di minima differenza di altezza.
    // Con il ciclo esterno scandisco prima tutte le ultime posizioni delle persone entrate dall'alto 
    // (e poi quelle entrate dal basso) per confrontarle con la posizione attuale della persona 
    // i-esima del frame corrente nel caso che ci sia un elemento della lista inhi molto vicina 
    // alla persona i-esima del frame corrente ed allo stesso tempo un elemento della lista inlo molto vicina 
    // alla persona i-esima del frame corrente allora i due indici (ind_pershi e ind_perslo) sono coincidenti 
    // perche corrispondono alla stessa persona i-esima del frame corrente quindi per discriminare uno dei due 
    // casi vado a vedere qual'e la distanza minima piu piccola tra il blob attuale
    // e i due potenziali blob associati ad esso (quello contenuto dentro la lista inhi e inlo).
    if ((ind_pershi==ind_perslo && distmin_inlo<distmin_inhi) || ind_pershi!=ind_perslo)
    {
        int life=inlo[i]->life;

        // xke incremento di due unita' il numero di vite se ci sono + PCN collegati in serie ???
        if(total_sys_number>1) life+=2;

        // distmin_inlo e' il quadrato della distanza
        if (distmin_inlo<(400+abs(LIFE-life)*50))
            //se la distanza e' abbastanza piccola
        {
            int diff_height;
        
            // xke se ce' 1 solo PCN la diff_height e' 14 anziche' 30 ??? che significato hanno questi due numeri ???
            if(total_sys_number<2) diff_height=14;
            else diff_height=30;

            //se la differenza di altezza fra le due pers e' contenuta
            if(inlo[i]->h-hpers[ind_perslo]<diff_height 
                || ((diff_cond_1p==1) && (inlo[i]->h/2<hpers[ind_perslo])) )
            {
                non_trovate[ind_perslo]=false;
                if(distmin_inlo>9)//se ha fatto almeno 3px ha fatto un passo
                {
                    inlo[i]->passi++;

                    // setto la nuova posizione del centroide
                    inlo[i]->x=people[ind_perslo]%xt;
                    inlo[i]->y=people[ind_perslo]/xt;
                
                    if(inlo[i]->passi>3) inlo[i]->life=LIFE+1; 

                    //ricorda l'altezza massima
                    if (hpers[ind_perslo]> inlo[i]->h)
                    {
                        inlo[i]->h=hpers[ind_perslo]; 
                    }
                
                    inlo[i]->trac=true;
                
                    // setto le nuove dimensioni del centroide
                    //inlo[i]->wx=dimpers[ind_perslo]; // 20110930 eVS, bug fix on indexes
                    //inlo[i]->wy=dimpers[ind_perslo+1];
                    inlo[i]->wx=dimpers[2*ind_perslo]; // 20110930 eVS, bug fix on indexes
                    inlo[i]->wy=dimpers[2*ind_perslo+1];
                
                    //...

    \endcode
            

    <b>Aggiunta nuovi blob alle liste inhi e inlo</b>
    \code
    // Se ci sono dei nuovi blob che prima non erano presenti
    // l'algoritmo di tracking non ha trovato alcuna corrispondenza tra vecchi blob e nuovi blob
    // percio' aggiunge i nuovi blob alle due liste inhi e inlo.
    for (int t=0; t<person;t++)
    {
        if(non_trovate[t])
        {
            int px=people[t]%xt;
            int py=people[t]/xt;

            // inserisco le persone entrate dalla parte alta nella lista delle persone da traccare
            // (si tratta delle persone che entrano sull'autobus)
            if(py<=door_threshold)
            {
                int n=person;
                if(n<num_pers)
                    while(inhi[n]!=NULL)
                    {
                        n++;
                        if(n>=num_pers)	break;
                    }
                    if(n>=num_pers)
                    {
                        n=0;
                        while(inhi[n]!=NULL)
                        {
                            n++;
                            if (n>=num_pers) break;
                        }       
                    }        
                    if(n<num_pers)
                    {
                        inhi[n]=new tPersonTracked;
                        inhi[n]->cont=false;
                        inhi[n]->trac=true;
                        inhi[n]->passi=0;
                        //inhi[n]->wx=dimpers[t]; // 20110930 eVS, bug fix on indexes
                        //inhi[n]->wy=dimpers[t+1];
                        inhi[n]->wx=dimpers[2*t]; // 20110930 eVS, bug fix on indexes
                        inhi[n]->wy=dimpers[2*t+1];
                        inhi[n]->h=hpers[t];
                        inhi[n]->life=LIFE;
                        inhi[n]->x=px;
                        inhi[n]->y=py;
                        //...
                        
            if(py>door_threshold)
            {
                //...                        
    \endcode


    <b>Viene stabilito quali persone andranno contate</b>
    \code
    // a seconda che la persona entrata dall'alto/basso abbia superato o meno la soglia della porta
    // e se lo stato della persona e' in modalita' inseguimento e non in modalita' conteggio.
    for (int l=num_pers-1;l>=0;l--)
    {
        if(inhi[l]!=NULL)
        {
            if(inhi[l]->cont==false && inhi[l]->trac==true)
            {
                if(inhi[l]->y > door_threshold)
                {
                    inhi[l]->cont=true;
                }
            }
        }
        //...    
    \endcode


    <b>Conteggio delle persone</b>
    \code
    // Se la persona non si trova piu in stato di tracking allora
    // diminuisco il numero di vite ad essa associate, inoltre se la persona ha superato la soglia porta,
    // se si trova in stato di conteggio, se ha fatto un numero minimo di passi e se il numero di vite e'
    // zero allora la persona viene contata come entrata (nel caso inhi) oppure uscita (nel caso inlo).
    for(int i=0;i<num_pers;++i)
    {
        if(inhi[i]!=NULL)
        {
            if(inhi[i]->trac==false)
            {
                inhi[i]->life--;
                //...    
                if((inhi[i]->y > door_threshold)&&(inhi[i]->cont==true)&& pas>PAS_MIN && inhi[i]->life==0)
                //...
    \endcode

                                                
    \code
    // incremento il contatore delle persone entrate solo se
    // il motion detection e' disabilitato oppure se e' abilitato
    // e c'e' "movimento" a meno di 200 frame di 
    // potenziale inattivita' (vedi loops.cpp funzione main_loop)
    if(move_det_en == 0) people_count_input++;
    else if(count_true_false == true)  people_count_input++;
    \endcode


    <b>Disegno delle croci sulle teste rilevate e traccate</b>
    \code
    // Viene tracciata una croce sulla mappa di disparita filtrata
    // solo se la persona i-esima ha fatto almeno 3 passi
    // e se la persona stessa si trova in stato di tracking.
    if(total_sys_number<2)
    {
        for(int i=num_pers-1;i>=0;--i)
        {
            if(inhi[i]!=NULL)
            {
                if(inhi[i]->passi>2 && inhi[i]->trac==true)
                {
                    int pos_x=inhi[i]->x;
                    int pos_y=inhi[i]->y;
                    
                    for(int x=-inhi[i]->wx; x<inhi[i]->wx; x++)
                        disparityMap[NX*pos_y+pos_x+x]=0xFF;
                        
                    for(int y=-inhi[i]->wy; y<inhi[i]->wy; y++)
                        disparityMap[(NX*(pos_y+y))+pos_x]=0xFF;
                }
            }
            //...    
    \endcode
    */
    
}


/*! 
\brief Draw a cross on the disparity map where the person appears.
*/
void draw_cross_on_map(tPersonTracked *person_data, unsigned char *map)
{
    //if(person_data->num_frames>MIN_NUM_FRAMES && person_data->trac==true)
    if(person_data->trac==true)
    {
      if (person_data->x >= 0 && person_data->x < NX && person_data->y >= 0 && person_data->y < NY)
      {
        int pos_x=person_data->x;
        int pos_y=person_data->y;

        for(int x=max(0,pos_x-person_data->wx); x<min(NX-1,pos_x+person_data->wx); x++)
            map[NX*pos_y+x]=0xFF;

        for(int y=max(0,pos_y-person_data->wy); y<min(NY-1,pos_y+person_data->wy); y++)
            map[(NX*(y))+pos_x]=0xFF;
      }
    }
}
#endif
