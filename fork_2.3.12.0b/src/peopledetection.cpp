/*!
\file peopledetection.cpp
\brief Contiene l'implementazione dell'algoritmo di detection delle persone.

Per i dettagli vedi la descrizione dettagliata della funzione detectAndTrack().

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <limits.h>

#include "directives.h"
#include "peopledetection.h"
#include "BPmodeling.h"
#include "OutOfRangeManager.h"

#ifndef NOMINMAX
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#endif  /* NOMINMAX */

#ifdef USE_NEW_TRACKING
#include "blob_tracking.h"
#else
#include "peopletrack.h"
#endif

#include "default_parms.h"


// 20100507 eVS measure performances
// If this define is not removed, then the file "/tmp/time_file.txt" will
// be created reporting the mean time needed by detectAndTrack processing
#ifdef eVS_TIME_EVAL
#include <time.h>
#endif

// 20111202 eVS
#ifdef USE_NEW_DETECTION
#include "blob_detection.h"
#endif

// variables shared between main and the detectAndTrack routine in order to substitute
// the original detection algorithm with the results obtained in MATLAB
#if !defined(PCN_VERSION) && (defined(USE_NEW_ALGORITHM_MATLAB) || defined(COMPARE))
extern int g_x[10], g_y[10], g_wx[10], g_wy[10], g_h[10];
extern int g_current_num_pers;
#endif

// parametri per people detection 
#define MAX_DEV 30   //!< massima deviazione standard (valori tra 20 e 40)
#define MIN_VAL 16   //!< soglia sulla disparit&agrave; usata in detection per analizzare le proiezione

#define SPIKE 30 /*!< \brief altezza minima degli spikes (valori tra 30 e 50)

Soglia utilizzata nel processo di rimozione del rumore impulsivo dalle proiezioni #xproj[] e #yproj[][]
della mappa di disparit&agrave; filtrata: se l'elemento i-esimo del vettore proiezione supera di SPIKE il valor medio 
dei quattro precedenti allora l'elemento stesso viene considerato rumore e perci&ograve; viene sostituito con il valor medio. */

#define BORDI_PROJ_X 19 /*!< \brief Soglia utilizzata nel processo di proiezione della mappa di disparit&agrave; filtrata sull'asse X, ovvero #xproj[]: 
questa soglia corrisponde al bordo esterno dell'immagine che non viene considerato. */

#define BORDI_PROJ_MOD 28 /*!< \brief Soglia utilizzata nel processo di decisione della modalit&agrave; una persona piuttosto che pi&ugrave; persone 
(analizzando la proiezione della mappa di disparit&agrave; sull'asse X): 
questa soglia corrisponde al bordo esterno dell'immagine che non viene considerato. 
E' maggiore rispetto al bordo utilizzato prima (#BORDI_PROJ_X) cos&igrave; vengono considerate solo le teste intere. */

#define PIUPERS_X 32 /*!< \brief Soglia utilizzata nel processo di decisione della modalit&agrave; una persona piuttosto che pi&ugrave; persone: 
se il numero di occorrenze, lungo l'asse X, che supera il valore di 24 disparit&agrave; (su 128) &egrave; maggiore di 20 e 
se la disparit&agrave; massima di ciascuna colonna (una ogni 3 colonne) supera il valore #MIN_VAL, 
allora incremento un indicatore che se a sua volta supera PIUPERS_X allora molto probabilmente nella scena sono 
presenti pi&ugrave; persone. */

#define PIUPERS_Y 27 /*!< \brief Soglia utilizzata nel processo di decisione della modalit&agrave; una persona piuttosto che pi&ugrave; persone: 
se lungo l'asse Y la disparit&agrave; massima di ciascuna riga (una ogni 3 righe) supera il valore #MIN_VAL, 
allora incremento un indicatore che se a sua volta supera PIUPERS_Y allora molto probabilmente nella scena sono 
presenti pi&ugrave; persone. */

#define NUM_MAX 20 //!< Maximum number of peaks detectable in the x-axis projection.

//#define INIZ_X 8   //!< ??? DEFINITA MA NON UTILIZZATA ???
//#define INIZ_Y 8   //!< ??? DEFINITA MA NON UTILIZZATA ???

#define FRAME_CLEAN 220  //!< Number of clean frames to acquire and to be averaged to obtain and save a new background.

#define MAX_DIFF_BKG 1920 //!< Serve per verificare che l'auto background non sia troppo diverso da quello attuale (meno del 10% dei pixel diversi). Prima era 4800, cioe' il 25%.

//#define debug_
#define time_bkg

#ifdef time_bkg
#include "time.h"
#endif

#if defined(USE_NEW_TRACKING) && !defined(PCN_VERSION)
bool reset = false;
#endif

/*******************************************************************************/
extern unsigned long people_count_input;
extern unsigned long people_count_output;
extern void record_counters(const unsigned long i_people_in, const unsigned long i_people_out);

#ifndef USE_NEW_DETECTION
void maxsearch_y(unsigned char fun_mode, int nump, int q);
void maxsearch_x(unsigned char fun_mode, int t, unsigned char xproj[NX]);
#endif

void drawtrack(unsigned char *img);
#ifdef PCN_VERSION
extern void SaveBkg();
#endif

unsigned char Bkgvec[NN]; //!< Vettore di elementi che contiene lo sfondo.
//unsigned char Pasvec[NN]; //< Vettore di elementi che contiene la mappa di disparit&agrave; elaborata.
//unsigned char min_m; //!< ???

/*!
\var disparityMapOriginal
\brief Vettore di elementi che contiene la mappa di disparit&agrave; originale prima del processing fatto in detectAndTrack().
*/
unsigned char disparityMapOriginal[NN];

/*!
\var Bkgvectmp
\brief Buffer temporaneo che contiene lo sfondo caricato e poi usato per l'auto background.

Questa variabile viene inizializzata la prima volta quando viene acquisito il background o caricato da file.
Poi viene aggiornata quando non c'e' nessuno sotto al sensore (pp==0, i.e. nessuno e' stato trovato dalla 
detection), le immagini sono simili (i valori di grigio medio, i.e. (abs(#vm_img-#vm_bkg) < #soglia_bkg)) 
e, inoltre, il nuovo background e quello precedente non sono troppo diversi (differiscono per meno 
di #MAX_DIFF_BKG pixels).
*/
unsigned char Bkgvectmp[NN];

/*
\var person
\brief DECLEARED BUT NOT USED ???
*/
/*
\var total
\brief DECLEARED BUT NOT USED ???
*/
/*
\var go
\brief DECLEARED BUT NOT USED ???
*/
//unsigned int person, total, go;

/*
\var t
\brief Numero di modelX trovati (moved inside the function detectAndTrack())
usata per contare i modelX
*/
/*
\var q
\brief Numero modelY trovati (moved inside the function detectAndTrack())
usata per contare i modelY dato un certo modelX
*/
/*
\var nump (moved inside the function detectAndTrack())
\brief ??? viene usato solo come indice per scandire i 
massimi sull'asse delle x e quindi la dichiarazione e' 
stata spostata dove c'e' il for che la usa.
*/
//int q;
//int t, q;

/*
\var aggiobkg
\brief ??? DEFINITA MA NON UTILIZZATA ???
*/
//bool aggiobkg=false;

/*
\var var
\brief Era usata per salvare o meno la varianza ma in realtà la varianza
viene sempre salvata e quindi la variabile è stata rimossa
*/
//bool var=true;

/*
\var save
\brief ??? DEFINITA MA NON UTILIZZATA ???
*/
//bool save=true;

/*
\var th
\brief ??? DEFINITA MA NON UTILIZZATA ???
*/
/*
\var dith
\brief ??? DEFINITA MA NON UTILIZZATA ??? 
*/
/*
\var bloth
\brief ??? DEFINITA MA NON UTILIZZATA ???
*/
/*
\var minsum
\brief ??? DEFINITA MA NON UTILIZZATA ???
*/
//int th, dith, bloth, minsum;


/*!
\var svec
\brief Vettore delle deviazioni standard associate allo sfondo caricato da file.
*/
int svec[NN];

/*!
\var svectmp
\brief Vettore temporaneo delle deviazioni standard associate allo sfondo caricato da file.
*/
int svectmp[NN];


/*!
\var numFrameClean
\brief Used by "time bkg"
*/
unsigned numFrameClean=0;

/*!
\var data_wide_gate
\brief Data collected by each sensor from its slaves in wideconfiguration.

This variable is not used normally but only in wideconfiguration. It is a sort of 
table collecting data from slaves. Each sensor (master included) collect data in
a local #persdata variable. Each slave receives data from the previous slaves,
adds its own data, and then sends the new colleted data to its master. This
process is done by the "persdetwidegate" command via serial port.

The dimension of this table is 54 multipied by the number of sensors in
wideconfiguration.

An explanation of the number 54 is given in the comment of #persdata.
*/
unsigned char *data_wide_gate=NULL;

/*!
\var limitSx
\brief Indice della colonna della "no tracking zone" di sinistra
*/
unsigned char limitSx=0; 

/*!
\var limitDx
\brief Indice della colonna della "no tracking zone" di destra
*/
unsigned char limitDx=160;

/*!
\var limitSx_riga_start
\brief Indice della riga di inizio della "no tracking zone" di sinistra
*/
unsigned char limitSx_riga_start=0;

/*!
\var limitDx_riga_start
\brief Indice della riga di inizio della "no tracking zone" di destra
*/
unsigned char limitDx_riga_start=120;

/*!
\var limitSx_riga_end
\brief Indice della riga di fine della "no tracking zone" di sinistra
*/
unsigned char limitSx_riga_end=0;

/*!
\var limitDx_riga_end
\brief Indice della riga di inizio della "no tracking zone" di destra
*/
unsigned char limitDx_riga_end=120;

/*!
\var limit_line_Up
\brief Altezza della "no tracking zone" in alto
*/
unsigned char limit_line_Up=0;

/*!
\var limit_line_Down
\brief Altezza della "no tracking zone" in basso
*/
unsigned char limit_line_Down=120;

/*!
\var diff_cond_1p
\brief Condizione "One person difficult condition" sul tab Advanced dell'interfaccia win_client (frame Single Way Tracking).
*/
int diff_cond_1p=0;

/*!
\var autobkg_disable
\brief Used to force the automatic background update to be disabled.
\date February 2009
*/
unsigned char autobkg_disable = DIS_AUTOBKG;    //20090506 Lisbona

unsigned char door_size = DOOR_SIZE;            //20100419 eVS


/*!
\var persdata
\brief Data collected by the single sensor in wideconfiguration and sent to the master.

The 54 bytes are used as the following: 4 bytes for header (0xFF, 0xFF, system number, 
and syncro info), and 5 bytes (h,x,y,wx,wy) for each detected person (at most 10 persons 
so at most 50 bytes).

The datapers of each sensor has to be added to the #data_wide_gate variable
and sent to the master by the serial_port command "persdetwidegate".
*/
unsigned char persdata[54];

/*!
\var count_sincro
\brief Used in wideconfiguration to syncronize data from all the sensors.
*/
/*!
\var framecounter
\brief Incremented at each new image acquisition and used to scan some activities every N acquisitions.

For example, when connected with the win_client application, this variable is exploited to 
send data not for all the acquisitions but only sometimes: in tracking mode every 2
acquisitions, in those modalities where three images have to be sent every 6 acquisitions,
in those modalities where two images have to be sent every 4 acquisitions.
*/
unsigned char count_sincro=0;
unsigned long int framecounter=0;

/*!
\var inst_height
\brief Sensors installation height used in wideconfiguration (the default value is #INST_HEIGHT centimeters).
*/
int inst_height = INST_HEIGHT;

/*!
\var inst_dist
\brief Distance between consecutive sensors in wideconfiguration (the default value is #INST_DIST centimeters).
*/
int inst_dist = INST_DIST;

/*!
\var handle_oor
\brief Set 1 when OOR handle is active, 0 otherwise.
*/
unsigned char handle_oor = HANDLE_OOR; // 20130715 eVS

static const int NUMBER_OF_FRAMES_BEFORE_OOR_CKECK = 5*54;

/*!
\var people
\brief Contiene i contatori delle persone entrate ed uscite ???
*/
unsigned long people[2];

/*!
\var det_area
\brief Assume valore 0 o 1 in base, rispettivamente, a setup 25/30 e 31/40.

Modificato da win_client con comando "detect_area" (vedi commands.cpp) e corrisponde al parametro con stringa "detect_area" nei parametri (\ref tabella_parms).
Si noti che 0 corrisponde ad una altezza di installazione di 225cm mentre 1 corrisponde ad una altezza di installazione di 240cm.

Si noti che ai due setup corrispondono due diversi mapping tra i 16 livelli di disparita' memorizzabili e il range di valori
reali ottenuti con la calibrazione e, proprio per questo, fanno parte dei parametri di calibrazione (\ref tabella_calib).

Questi due mapping sono memorizzati in calibration.txt dove sono caratterizzati dalle stringhe steps225_0..steps225_F e steps240_0..steps240_F.
In mancanza del file di calibrazione i mapping vengono inizializzati usando le variabili globali #STEPS_225 e #STEPS_240.
*/
unsigned char det_area;

/*!
\var Tab_Altezza_Disparita225
\brief Mapping between disparities and the height from the floor when the sensor is installed at 225cm.
*/
/*!
\var Tab_Altezza_Disparita240
\brief Mapping between disparities and the height from the floor when the sensor is installed at 240cm.
*/
const int Tab_Altezza_Disparita225[16] = {225-225,225-225,225-225,225-225,225-125,225-105,225-90,225-80,225-70,225-60,225-55,225-50,225-42,225-35,225-32,225-30};
const int Tab_Altezza_Disparita240[16] = {240-240,240-240,240-240,240-240,240-150,240-125,240-105,240-90,240-80,240-70,240-60,240-55,240-50,240-42,240-35,240-32};


#ifdef time_bkg
unsigned char minuti_th=0; //!< Specifies the time interval to be waited before update the background using 
//int frame_clean_th;        //< ???
int static_th;             //!< Soglia statica impostata da interfaccia win_client per aggiornamento automatico del background.
/*!<
Numero di pixel che servono per discriminare la situazione 
di aggiornamento del background statico, nel caso che ci sia un numero di pixel inferiore alla soglia, 
altrimenti re-inizializzo il background statico con la InitStaticObj(), nel caso che il numero di pixel differenti 
sia maggiore della soglia.
*/
//int framecounter_bkg=0;  //??? DEFINITA MA NON UTILIZZATA ???
ogg_static BkgStatic;      //!< E' una struttura dati che contiene lo sfondo statico, la deviazione standard dello sfondo e l'istante di acquisizione.
#endif

bool ev_door_open=false;     //!< Evento di porta appena aperta.
bool ev_door_close=false;    //!< Evento di porta appena chiusa.
bool ev_door_open_rec=false; //!< Viene utilizzata per decidere se scrivere un messaggio di log in corrispondenza dell'evento di apertura porta.
bool ev_door_close_rec=false;//!< Viene utilizzata per decidere se scrivere un messaggio di log in corrispondenza dell'evento di chiusura porta.
bool mem_door = false; //!< Per la gestione del segnale porta aperta/chiusa, contiene il valore dello stato attuale

//unsigned long door_in;  // 20100702 eVS moved in io.cpp where used
//unsigned long door_out;  // 20100702 eVS moved in io.cpp where used

unsigned char soglia_bkg; //!< Soglia usata per decidere se aggiornare o meno il background in modo automatico
/*!< Soglia sulla differenza tra il livello di grigio medio attuale #vm_img e quello delle immagini usate in acquisizione del background #vm_bkg.
Questa soglia viene usata per decidere se aggiornare o meno il background in modo automatico dopo aver gia' verificato che nessuna
persona sia stata trovata nella scena (ovvero la detection non trova nessuno).*/
bool write_records=false; //!< ???

unsigned char frame_cnt_door=0; //!< Pu&ograve; assumere i valori 0, 1, 2 e 3 corrispondenti a 4 stati.
/*!<
Non si &egrave; certi del significato dei quattro valori ma sembra che corrispondano a:<br>
- 0 = appena finito di gestire la porta aperta nel caso non wide-gate e sono pronto alla prossima apertura;<br>
- 1 = ho finito la gestione della chiusura della porta e sono quindi pronto appena aperto le porte 
in wide-gate (analogo a 0 solo che siamo in wigegate);<br>
- 2 = ho inserito le persone gi&agrave; in scena nelle liste e posso rimettere la soglia al suo valore;<br>
- 3 = ho finito di gestire l'apertura della porta ma sono in widegate.<br>

Viene usata nella gestione dell'evento porta aperta all'inizio dell'algoritmo di detection.
*/

unsigned char frame_fermo=0; //!< Sembra servire per gestire l'eventuale problema che la flag ev_door_open non venga messa a false. 
/*!<
Infatti se frame_fermo arriva a 200 viene forzato l'assegnamento ev_door_open = false. Analogo per ev_door_close. 
Viene usata nella gestione dell'evento porta aperta/porta chiusa all'inizio dell'algoritmo di detection.
*/



/*!
\var modelx
\brief Vettore di 160 elementi che contiene l'elenco dei possibili blob rilevati in orizzontale.
*/
model modelx[NX];

/*!
\var modely
\brief Vettore di 120 elementi che contiene l'elenco dei possibili blob rilevati in verticale.
*/
model modely[NY];

/*!
\var max_x
\brief Vettore di NUM_MAX elementi che contiene l'elenco dei massimi locali rilevati sulla proiezione dell'asse X.
*/
model max_x[NUM_MAX];

/*!
\var max_y
\brief Vettore di NUM_MAX elementi che contiene l'elenco dei massimi locali rilevati sulla proiezione dell'asse Y per ogni massimo locale rilevato e filtrato sull'asse X.
*/
model max_y[NUM_MAX][NUM_MAX];

/*!
\var vm_img
\brief Valor medio dell'immagine di sinistra o di destra a seconda della variabile "sensor" locale al main_loop che viene cambiata ogni 40 frame.

In altre parole, ogni 40 frame viene cambiato il sensore rispetto al quale calcolare la media.
Inoltre, ogni volta che ho una nuova media (dell'immagine di sinistra o di destra) la medio con la 
sua storia dando peso 3 alla storia e peso 1 al nuovo valore.
*/
unsigned char vm_img = 128;

/*!
\var vm_bkg
\brief Valor medio (di una?) delle immagini durante l'acquisizione dello sfondo
*/
unsigned char vm_bkg;

/*!
\var mov_dect_0_7l
\brief Parte bassa (8 bits meno significativi) del parametro di motion detection (relativo al frame sinistro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
/*!
\var mov_dect_8_15l
\brief Parte intermedia (dall'ottavo bit al quindicesimo bit) del parametro di motion detection (relativo al frame sinistro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
/*!
\var mov_dect_15_23l
\brief Parte alta (8 bits pi&ugrave; significativi) del parametro di motion detection (relativo al frame sinistro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
unsigned char mov_dect_0_7l;
unsigned char mov_dect_8_15l;
unsigned char mov_dect_15_23l;

/*!
\var mov_dect_0_7r
\brief Parte bassa (8 bits meno significativi) del parametro di motion detection (relativo al frame destro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
/*!
\var mov_dect_8_15r
\brief Parte intermedia (dall'ottavo bit al quindicesimo bit) del parametro di motion detection (relativo al frame destro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
/*!
\var mov_dect_15_23r
\brief Parte alta (8 bits pi&ugrave; significativi) del parametro di motion detection (relativo al frame destro) 
estratto dalla get_images() dal blocco di #NX*#NY*4=160*120*4=320x240 bytes.
*/
unsigned char mov_dect_0_7r;
unsigned char mov_dect_8_15r;
unsigned char mov_dect_15_23r;

/*!
\var num_pers
\brief Contiene il numero massimo di persone per la detection.

Di default viene settata a #NUM_PERS_SING, ovvero non possono essere trovate pi&ugrave; di #NUM_PERS_SING persone nella scena).
Si noti che in base alla configurazione del PCN (standalone o widegate) il numero massimo di persone
cambia in in base: se standalone e' pari al default #NUM_PERS_SING ma se in widegate allora
ogni sensore puo' vedere al piu' #NUM_PERS_SING persone e quindi il numero massimo di 
persone persone individuabili e' #NUM_PERS_SING*#total_sys_number.

Questa variabile e' usata in initpeople() per l'allocazione della memoria delle strutture 
dati #inhi e #inlo usate per il tracking e in detectAndTrack() per le strutture dati usate per 
la detection (dimpers, hpers, and people_coor).

*/
int num_pers = NUM_PERS_SING;

int xt=160;

/*!
\var total_sys_number
\brief Used in widegate mode: total number of systems chained to supervise a wide gate.
*/
unsigned char total_sys_number=1;

/*!
\var current_sys_number
\brief Used in widegate mode: index of the current system in the chain of system when in widegate.
*/
unsigned char current_sys_number=1;

/*!
\var count_true_false
\brief Flag relativo alla potenziale inattivit&agrave; per 200 frame consecutivi (in questo caso vale false).
*/
bool count_true_false=true;

//unsigned char move_det_en; //!< Flag relativo all'abilitazione del motion detection da interfaccia win_client.

unsigned char door_stairs_en; //!< Corresponding to the option "Stairs Upgrade Enabled" in the tab "Advanced (2/2)" of the win_client application.

/*!
\var soglia_porta
\brief Posizione della soglia (intesa come riga sulla mappa di disparit&agrave;) oltre la quale un passeggero viene contato

Il passeggero viene contato come input o come output a seconda della direzionalita' della porta e del suo moto (entrato 
da sopra e uscito sotto o vicersa). Questa variabile viene opportunamente settata con SetDoor() prima della chiamata alla track()
in detectAndTrack(). Inoltre questa soglia puo' essere cambiata da win_client con il comando "threshold" (vedi commands.cpp).
Si noti che in widegate, solo il master fa' il tracking e quindi solo per il master viene usata questa soglia. E' per questo
motivo che in serial_port.cpp il comando "threshold" serve solo per gli slave.
*/
unsigned short soglia_porta; 

/*!
\var yproj
\brief Vettore di elementi che per ogni massimo locale dell'asse X, 
contiene il valore massimo di disparit&agrave; relativamente alla riga j-esima 
(considerando cio&egrave; un segmento orizzontale prelevato dall'elemento max_x[i]).
*/
unsigned char yproj[NUM_MAX][NY];

/*!
\var xproj
\brief Vettore di elementi che contiene per ogni colonna il valore massimo di disparit&agrave;.
*/
unsigned char xproj[NX];

/*!
\var yprojtot
\brief Vettore di elementi che contiene per ogni riga il massimo locale della proiezione sull'asse X.
*/
unsigned char yprojtot[NY];


/*!
\brief Setta la soglia della porta al valore "soglia", per poter discriminare 
il caso di una persona entrata piuttosto che uscita.
*/
void SetDoor(unsigned char soglia)
{
  soglia_porta=soglia;
}


/*!
\brief Restituisce la soglia usata internamente a peopledetection.cpp.
*/
unsigned char GetDoor()
{
  return (unsigned char)soglia_porta;
}


/*!
\brief Imposta la soglia sul livello di grigio medio usata per decidere se aggiornare o meno il background in modo automatico.

Viene cambiato il valore di #soglia_bkg usata per verificare se (abs(#vm_img-#vm_bkg) < #soglia_bkg) prima di aggiornare
il background in automatico, ovvero si verifica che quanto ripreso ora non differisca troppo da quanto ripreso durante
l'acquisizione.
*/
void SetBkgThreshold(unsigned char soglia)
{
  soglia_bkg=soglia;
  return;
}
#ifdef time_bkg


/*!
\brief Torna data e ora espresse in minuti.
*/
unsigned char GetMin()
{
  time_t timer;
  unsigned char minuti;
  struct tm *tblock;
  /* gets time of day */
  timer = time(NULL);
  /* converts date/time to a structure */
  tblock = localtime(&timer);
  minuti = tblock->tm_min;
  return minuti;
}


/*!
\brief Controlla se il background usato per la sottrazione dello sfondo pu&ograve; essere aggiornato.

L'aggiornamento viene eseguito nel caso in cui venga passato il seguente controllo: va a vedere
quanto tempo &egrave; passato da quando &egrave; stato acquisito l'ultimo 
background statico e se sono passati un numero di minuti pari ad almeno #minuti_th 
allora aggiorna il background (media e deviazione standard) usato per fare la 
sottrazione del background in detectAndTrack().

L'aggiornamento viene fatto sia in memoria, aggiornando le variabili
#Bkgvec (#Bkgvectmp) e svec, sia su disco usando SaveBkg(). 

Prima di concludere, deve re-inizializzare la struttura dati #BkgStatic con InitStaticObj() per prepararla
ad essere elaborata per i prossimi #minuti_th minuti.
*/
#ifdef PCN_VERSION
void CheckStatic()
{
  //controlla se una persona statica puo' essere cancellata
  unsigned char  minuti=GetMin();
  unsigned char min_ferma=BkgStatic.min;
  int pixel_validi=0;
  // printf("min attuali %d, min ferma %d, min soglia %d\n",minuti, min_ferma, min_soglia);
  bool aggiorna=false;
  if(min_ferma>minuti)  //tengo conto dell'eventuale cambio di ora
  {
    if((60-min_ferma + minuti)>=minuti_th)
    {
      aggiorna = true;
    }
  }
  else
  {
    if((minuti-min_ferma)>=minuti_th)
    {
      aggiorna= true;
    }
  }
  if(aggiorna)
  {
    for(int i=17912;i>=1288;i--)
    {
      if(BkgStatic.BkgTmpStatic[i]>0) pixel_validi++;
    }
    deinitpeople(num_pers); //eventuali oggetti che hanno passato la soglia non devono esser contati
    for(int h=NN-1;h>=0;h--) svec[h]= BkgStatic.svec[h];//la setto d'ufficio ad un livello poi si autoaggiorna da sola
    memcpy(Bkgvec,BkgStatic.BkgTmpStatic,NN);
    memcpy(Bkgvectmp,BkgStatic.BkgTmpStatic,NN);//sincronizza l'altro aggio di bkg
    SaveBkg();
#ifdef debug_
    printf("Bkg aggiornato per tempo minuti=%d min_ferma=%d minuti_th=%d\n",minuti,min_ferma,minuti_th);
#endif
    InitStaticObj();
  }
}


/*!
\brief Inizializza le strutture dati per il mantenimento del background.

Lo sfondo statico (#BkgStatic.%BkgTmpStatic) viene inizializzato con la mappa di
disparit&agrave; acquisita e la deviazione standard dei singoli pixel (#BkgStatic.%svec) 
viene posta a zero. Inoltre, memorizzo una informazione su data e ora di 
inizializzazione (#BkgStatic.%min).
*/
void InitStaticObj()
{
  for(int i=NN-1;i>=0;i--)
  {
    BkgStatic.BkgTmpStatic[i]=disparityMapOriginal[i];
    BkgStatic.svec[i]=0;
  }
  BkgStatic.min=GetMin();
#ifdef debug_
  printf("Inizializzazione completa, minuti %d \n",BkgStatic.min);
#endif
}


/*!
\brief Aggiorna il background statico.

Per aggiornare il background statico, si osservano le differenze tra il 
background statico attualmente memorizzato in #BkgStatic.%BkgTmpStatic
e la mappa di disparit&agrave; attuale. Nel caso in cui il
numero di pixel diversi sia inferiore alla soglia #static_th allora
aggiorno il background statico altrimenti re-inizializzo il background 
statico con la InitStaticObj(). Due pixel sono
considerati diversi se sono tali che la loro differenza in modulo
&egrave; maggiore di 16.

Quindi, l'idea &egrave; che il background statico viene aggiornato 
con la mappa di disparit&agrave; attuale solo quando essa &egrave; simile al 
background statico attualmente in memoria. 

L'aggiornamento dall'istante i all'istante i+1 avviene come media 
pesata nel seguente modo:
\verbatim
BkgTmpStatic(i+1) = (15*BkgTmpStatic(i) + disparityMapOriginal(i))/16
svec(i+1) = sqrt[15*svec(i)^2 + (BkgTmpStatic(i) - disparityMapOriginal(i))^2]
\endverbatim

L'aggiornamento &egrave; poi seguito da una fase di controllo. La funzione
CheckStatic() &egrave; legata alle continue re-inizializzazioni del background
statico fatte con InitStaticObj(). Infatti, queste re-inizializzazioni
vengono fatte quando la mappa attuale &egrave; diversa dal background 
statico memorizzato in BkgTmpStatic. Si noti che ad ogni re-inizializzazione
viene aggiornato il campo min inserendovi l'istante attuale.
Pertanto, nel caso in cui si continuino ad acquisire mappe che assomigliano
al background statico si ha che il campo min rimane invariato mentre invece 
#BkgStatic.%BkgTmpStatic e #BkgStatic.%svec vengono aggiornati con le formule viste sopra. 

Chiarito questo, diventa semplice capire cosa fa CheckStatic(): verifica da 
quanto tempo le mappe di disparit&agrave; acquisite sono simili al background
confrontando l'istante attuale con quello memorizzato nel campo min e se &egrave; 
passato abbastanza tempo (un numero di minuti maggiore o uguale a #minuti_th) 
allora aggiorna il background usato per la sottrazione (sia in memoria che su 
disco) con quello in #BkgStatic.%BkgTmpStatic e #BkgStatic.%svec.
*/
void FindStaticObj() //int *people,int person
{
  //rivedere puntatori
  unsigned char *bkgvec,*disvec;
  int count=0;
  int r2;
  if(BkgStatic.min==255)
  {
#ifdef debug_
    printf("inizializzo struttura bkg static\n");
#endif
    InitStaticObj();
    return;
  }
  for(int i=17912;i>=1288;i--)
  {
    if(abs(BkgStatic.BkgTmpStatic[i]-disparityMapOriginal[i])>16) count++;
  }
  if (count < static_th) 
  {

    for(int y2=0;y2<NY;y2++)for(int x2=0;x2<NX;x2++)
    {
      r2=y2*NX+x2;
      disvec = &disparityMapOriginal[r2];
      bkgvec = &BkgStatic.BkgTmpStatic[r2];
      if(*disvec > 0 || BkgStatic.svec[r2]>0)
      {
        *bkgvec=(15*(*bkgvec)+(*disvec)) >> 4;
        BkgStatic.svec[r2]=((int)sqrt((float)((15*BkgStatic.svec[r2]*BkgStatic.svec[r2]+(*disvec-*bkgvec)*(*disvec-*bkgvec)) >> 4)));
      }
    } 
    CheckStatic();
  }
  else
  {
    InitStaticObj();
  }

}
#endif


/*!
\brief ???
*/
void SetStaticTh(int soglia)
{
  BkgStatic.min=255; //struct not initialized
  static_th=soglia;
  return;
}



/*!
\brief ???
*/
void SetMinBkgTh(unsigned char soglia)
{
  minuti_th=soglia;
  return;
}


#endif

/*! 
\brief Image erode.

\param [in,out] disparityMap disparity map
*/
void map_erode(unsigned char *disparityMap)
{
  unsigned char *disvec,*pasvec;
  int n;    
  unsigned char cen_col = 0;
  unsigned char first_col = 0;

  static unsigned char Pasvec[NN];

  memcpy(Pasvec, disparityMap, NN);

  for(int y=NY-9;y>7;y--)
  {
    for(int x=NX-9;x>7;x--)
    {
      disvec = &disparityMap[NX*y+x];
      pasvec = &Pasvec[NX*y+x];

      //n=0;
      //if(y==NY-9) // 20100510 eVS bug???
      if (x==NX-9) //se sono all'inizio di una riga carico tutta la maschera
      {
        n = first_col = cen_col = 0; // 20100510 eVS bug???

        // prima colonna
        if(Pasvec[NX*(y-1)+x-1]>0)
        {
          n++;
          first_col++;
        }
        if(Pasvec[NX*y+x-1]>0)
        {
          n++;
          first_col++;
        }
        if(Pasvec[NX*(y+1)+x-1]>0)
        {
          n++;
          first_col++;
        }

        //colonna centrale
        if(Pasvec[NX*(y-1)+x]>0)
        {
          n++;
          cen_col++;
        }
        if((*pasvec)>0)
        {
          n++;
          cen_col++;
        }
        if(Pasvec[NX*(y+1)+x]>0)
        {
          n++;
          cen_col++;
        }

        //ultima colonna non serve salvarla
        if(Pasvec[NX*(y-1)+x+1]>0)  n++;
        if(Pasvec[NX*y+x+1]>0)      n++;
        if(Pasvec[NX*(y+1)+x+1]>0)  n++;
      }
      else //se non sono all'inizio di una riga
      {
        n         = cen_col + first_col; //aggiornamento numero validi nelle colonne dopo lo spostamento
        cen_col   = first_col;
        first_col = 0;

        if(Pasvec[NX*(y+1)+x-1] > 0)
        {
          n++;
          first_col++;
        }
        if(Pasvec[NX*y+x-1] > 0)
        {
          n++;
          first_col++;
        }
        if(Pasvec[NX*(y-1)+x-1] > 0)
        {
          n++;
          first_col++;
        }
      }
      if(n<=4) 
        *disvec = 0;
    }
  }
}


/*! 
\brief Image dilate.

\param[in,out] disparityMap disparity map
*/
void map_dilate(unsigned char *disparityMap)
{

#define DILATE_HEIGHT 2

  unsigned char *disvec, *pasvec;

  int n;
  int ss;
  int sum_first_col=0;
  int sum_cen_col=0;

  unsigned char cen_col = 0;
  unsigned char first_col = 0;

  static unsigned char Pasvec[NN];
  unsigned char elem;
  unsigned char threshold = (3*(DILATE_HEIGHT*2+1))/2;

  memcpy(Pasvec,disparityMap,NN);

  for(int y=NY-9;y>7;y--)
  {
    for(int x=NX-9;x>7;x--)
    {
      disvec = &disparityMap[NX*y+x];
      pasvec = &Pasvec[NX*y+x];
      if(x==NX-9) //se sono all'inizio di una riga carico tutta la maschera
      {
        sum_first_col=0;
        sum_cen_col=0;
        first_col=0;
        cen_col=0;
        ss=0;
        n=0;
        //for(int j=-1;j<=1;j++) 
        for(int j=-DILATE_HEIGHT;j<=DILATE_HEIGHT;j++) 
        {
          for(int i=-1;i<=1;i++)
          {
            elem = Pasvec[NX*(y+j)+x+i];
            if(elem > 0)
            {
              n++; // elementi > 0 nella finestrella 3x3 
              ss += elem; // sommo elementi > 0 nella finestrella 
              if(i==0) {
                sum_cen_col += elem; //Pasvec[NX*(y+j)+x+i]; // sommo elementi colonna centrale > 0
                cen_col++; // conto elementi della colonna centrale > 0
              }
              else if(i==-1) {
                sum_first_col += elem; //Pasvec[NX*(y+j)+x+i]; // sommo elementi prima colonna
                first_col++;
              }
            }
          }
        }
      }
      else //se non sono all'inizio di una riga
      {
        n = cen_col + first_col;
        ss = sum_first_col + sum_cen_col;

        cen_col = first_col;
        first_col = 0;
        sum_cen_col = sum_first_col;
        sum_first_col = 0;

        // analizzo nuova first_col
        for (int j=-DILATE_HEIGHT; j<=DILATE_HEIGHT; j++)
        {
          if(Pasvec[NX*(y+j)+x-1]>0)
          {
            n++;
            first_col++;
            sum_first_col += Pasvec[NX*(y+j)+x-1];
          }
        }

        ss+=sum_first_col;
      }

      // verifico se è il caso di riempire
      if(n>threshold)
        *disvec = ss/n;
    }
  }
}


#ifdef CHECK_FALSE_COUNTS
/*! 
Se ci sono nuovi conteggi si controlla la differenza tra il numero di frame
in cui essi sono avvenuti e il numero di frame del conteggio che si vuole sovrascrivere ( il buffer &egrave;
circolare). In caso affermativo si decrementa il numero di conteggi, si sovrascrive  il numero di
frame in cui &egrave; avvenuto il falso conteggio per aggiornare il buffer e si incrementa l'indice dello stesso.
La soglia di decisione &egrave; #NUM_ONE_TENTH_OF_SECONDS*#NUM_FRAME_PER_SEC che rappresenta il numero di frame al secondo.
*/
void _check_false_counts_one_dir(
                                 unsigned long & people,                  ///< [in|out] Conteggio corrente
                                 const unsigned long & prev_people,       ///< [in|out] Conteggio al frame precedente
                                 int & indx,                              ///< [in|out] Indice del buffer
                                 unsigned long* const & buffer_cnt,       ///< [in|out] Buffer circolare che memorizza il numero di frame del conteggio
                                 const int DIM_BUFFER_CNT,                ///< [in]     Dimensione del buffer circolare
                                 const unsigned long & number_of_frames)  ///< [in|out] Indice del frame corrente
{
  // Nuovi conteggi
  unsigned long delta_cnt = (people >= prev_people) ? (people - prev_people) : (prev_people - people);  // check needed for offline version where video can be played backward and counters can be decreased instead of be increased

  for (unsigned int i = 0; i < delta_cnt; ++i)
  {
    if (buffer_cnt[indx] != 0)  // Garantisce l'inserimento iniziale
    {
#if defined(PCN_VERSION) || defined(SEQ_AT_54FPS)
      const int NUM_FRAME_PER_SEC = 54;
#else
      const int NUM_FRAME_PER_SEC = 27;  // Numero di frame per secondo
#endif
      const int NUM_ONE_HUNDREDTH_OF_SECONDS = 63;  // Numero di decimi di secondo rispetto al quale fare il controllo dei falsi conteggi (non piu' di #DIM_BUFFER_CNT ogni NUM_ONE_TENTH_OF_SECONDS). Finestra temporale su cui effettuare il CheckFalseCounts()

      if ((number_of_frames - buffer_cnt[indx]) < (NUM_ONE_HUNDREDTH_OF_SECONDS * NUM_FRAME_PER_SEC)/100)
        --people;   // Ho avuto troppi conteggi in poco tempo: scarto l'ultimo conteggio
      else
      {
        // non devo scartare conteggi quindi inserisco il conteggio valido nel buffer
        buffer_cnt[indx] = number_of_frames;  // Salvo il numero del frame corrente nel buffer
        indx = (indx + 1) % DIM_BUFFER_CNT;  // incremento l'indice del buffer
      }
    }
    else
    {
      // non ho ancora riempito il buffer di conteggi quindi inserisco il conteggio nel buffer
      buffer_cnt[indx] = number_of_frames;  // Salvo il numero del frame corrente nel buffer
      indx = (indx + 1) % DIM_BUFFER_CNT;  // incremento l'indice del buffer
    }
  }

#ifdef _DEBUG
  // Stampa di prova
  /*for ( int i=0; i<DIM_BUFFER_CNT;++i)
  printf(" \n Tempo : %lu  \n",buffer_cnt[i]);
  */
#endif
}


/*! 
Effettua il controllo dei falsi conteggi basandosi sulla scansione di due buffer circolari
(uno per gli IN e uno per gli OUT) di dimensione #DIM_BUFFER_CNT che tengono traccia degli ultimi #DIM_BUFFER_CNT conteggi .
In particolare i buffer memorizzano il numero del frame in cui &egrave; stato effettuato
il conteggio. La funziona _check_false_count chiamata per entrambe i buffer provveder&agrave; al controllo dei conteggi.
*/
void
_check_false_counts(
                    unsigned long & peopleout,       ///< [in|out] Conteggio delle persone in uscita
                    unsigned long & peoplein,        ///< [in|out] Conteggio delle persone in entrata
                    unsigned long & prev_peopleout,  ///< [in|out] Conteggio delle persone in uscita al frame precedente
                    unsigned long & prev_peoplein,   ///< [in|out] Conteggio delle persone in entrata al frame precedente
                    const unsigned char & real_buf_sze)      ///< [in] Dimensione del buffer per la gestione dei falsi conteggi
{
  // Dichiarazione delle variabili statiche
  static const int DIM_BUFFER_CNT = 5;  // Dimensione dei buffer circolari contenenti il numero del frame in cui sono stati effettuati i conteggi.
  // In altre parole si tratta del numero massimi di conteggi in IN o in OUT nel periodo di tempo dato da NUM_ONE_TENTH_OF_SECONDS.
  static unsigned long buffer_cnt_in[DIM_BUFFER_CNT] = {};  // Buffer dei conteggi in entrata
  static unsigned long buffer_cnt_out[DIM_BUFFER_CNT] = {};  // Buffer dei conteggi in uscita
  static int indx_out = 0;  // Indice corrente del buffer in uscita
  static int indx_in = 0;  // Indice corrente del buffer in entrata
  static unsigned long number_of_frames = 0;  // Indice del frame corrente

  assert(real_buf_sze <= DIM_BUFFER_CNT);

  // printf("in=%ld; out=%ld; prev_in=%ld; prev_out=%ld\n", peoplein, peopleout, prev_peoplein, prev_peopleout);

  // Controllo conteggi in IN
  _check_false_counts_one_dir(peoplein, prev_peoplein, indx_in, buffer_cnt_in, real_buf_sze, number_of_frames);

  // Controllo conteggi in OUT
  _check_false_counts_one_dir(peopleout, prev_peopleout, indx_out, buffer_cnt_out, real_buf_sze, number_of_frames);

  ++number_of_frames;
}


static unsigned long prev_out = ULONG_MAX;
static unsigned long prev_in = ULONG_MAX;
void _set_prev_counters(const unsigned char & total_sys_number, const unsigned long & in, const unsigned long & out)
{
  if (total_sys_number < 2)  // se in widegate non uso la strategia di rimozione dei falsi conteggi
  {
    prev_in = in;
    prev_out = out;
  }
}
void _check_new_counters(const unsigned char & total_sys_number, unsigned long & io_new_in, unsigned long & io_new_out, const unsigned char & i_door_size)
{
  if (total_sys_number < 2)  // se in widegate non uso la strategia di rimozione dei falsi conteggi
  {
    assert(prev_out != ULONG_MAX && prev_in != ULONG_MAX); // i contatori precedenti devono essere stati inizializzati

    // Controllo per i falsi conteggi
    unsigned char real_buf_sze = (i_door_size+22)/45;  // 65 eVS modified 30092013
    _check_false_counts(io_new_out, io_new_in, prev_out, prev_in, real_buf_sze);
  }
}

#endif
/*! 
\brief Algoritmo di detection delle persone.

L'algoritmo di people detection rileva le persone che si trovano nell'area monitorata secondo il seguente schema 
(per ulteriori dettagli vedere i relativi frammenti del codice sorgente sotto riportati): 
- gestione evento porta aperta/chiusa (CloseDoor()), 
- sottrazione dello sfondo dalla mappa di disparit&agrave;, 
- filtraggio morfologico della mappa senza sfondo (erosione + erosione + espansione), 
- proiezione della mappa filtrata sull'asse X (per ogni colonna viene presa 
la massima disparit&agrave; e assegnata all'emento i-esimo del vettore #xproj), 
- eliminazione dei bordi per non considerare le teste incomplete, 
- rimozione degli spikes dalla proiezione sull'asse X (confrontando ciascun elemento 
del vettore #xproj con il valor medio dei 4 precedenti, per eventualmente sostituirlo), 
- eventuale (se viene attivata la modalit&agrave; "one person difficult condition") proiezione 
globale della mappa sull'asse Y (con rimozione degli spikes, ricerca massimi locali nella #yprojtot
e soppressione dei massimi sovrapposti), 
- calcolo media, deviazione standard, larghezza e centro dei potenziali blob orizzontali (#modelx), 
- ricerca dei massimi locali (#max_x costruita da maxsearch_x()) 
e soppressione dei massimi sovrapposti o troppo vicini (sempre analizzando la #xproj), 
- proiezione dei massimi locali dell'asse X (#max_x) sull'asse Y (ottenendo il vettore #max_y), 
- eliminazione degli spikes, ricerca dei massimi locali (#max_y costruita dalla maxsearch_y()) 
nella #yproj[n] (n=0,..,NumTotMassimiSullAsseX) e soppressione dei massimi sovrapposti. 
- calcolo media, deviazione standard, larghezza e centro dei potenziali blob verticali (#modely), 
- rilevamento delle teste in 2D mediante calcolo coordinate del centroide (nel caso in cui 
"one person difficult condition" sia stato attivato ovvero tutto ci&ograve; che &egrave; presente 
nella scena venga considerato un unico blob) piuttosto che analizzare la densit&agrave; dei 
punti validi contenuti nei blob ricavati dall'intersezione tra massimi filtrati sull'asse X (#max_x)
e vettore dei massimi filtrati sull'asse Y (#max_y), pi&ugrave; inserimento della persona trovata in 
un apposita struttura dati di tipo tPersonDetected (e relativo tracciamento della croce sulla 
mappa di disparit&agrave;).
- chiamata all'algoritmo di tracking track() delle persone trovate nel frame corrente rispetto alle 
persone trovate nel frame precedente. Il tracking mantiene due liste contenenti elementi di tipo 
tPersonTracked: una per le persone che entrano dall'alto e una per quelle che entrano dal basso.

\param disparityMap mappa di disparit&agrave; (vedi #Frame_DSP)
\param peoplein contatore delle persone entrate
\param peopleout contatore delle persone uscite
\param enabled specifica se il conteggio e' abilitato o meno (vedi #count_enabled)
\param door soglia porta usata per considerare una persona entrata piuttosto che uscita, ovvero il parametro identificato dalla stringa "threshold" (\ref tabella_parms).
\param direction se vale 0 allora il senso di marcia &egrave; dall'alto verso il basso, ovvero il contenuto di #people_dir nonche' il parametro identificato dalla stringa "dir" (\ref tabella_parms).
\param move_det_en se vale 1 significa che il motion detection &egrave; abilitato altrimenti &egrave; disabilitato.
\param min_y_gap usato come soglia per lo spostamento minimo lungo le y di un blob affinch&egrave; possa essere contato (esso dipende dalla no-tracking-zone)
*/
void detectAndTrack(unsigned char *disparityMap,
                    unsigned long & peoplein,
                    unsigned long & peopleout, 
                    const int & enabled, 
                    const unsigned short & door,
                    const unsigned char & direction,
                    const unsigned char & move_det_en
#ifdef USE_NEW_TRACKING
                    , const int & min_y_gap)
#else
                    )
#endif
{
  //static unsigned char *bkgvec,*disvec; //,*pasvec;
  /*!
  \var persone
  \brief Lista di blob costruita dall'algoritmo di people detection e indirettamente passata all'algoritmo di people tracking 
  (nel senso che le informazioni in esso contenute vengono prima suddivise in tre vettori: 
  people_coor contiene le coordinate delle persone rilevate nel frame corrente, 
  dimpers contiene le dimensioni dei potenziali blob ed infine hpers contiene le altezze delle teste rilevate 
  in termini di disparit&agrave;).
  */
#ifdef USE_HANDLE_OUT_OF_RANGE
  static tPersonDetected *prev_persone = new tPersonDetected[num_pers];  // essendo static ad ogni chiamata di detectAndTrack() in persone ho la lista delle detection precedenti
  static int prev_pp=0;  // essendo static ad ogni chiamata di detectAndTrack() in pp ho il numero di detection precedenti
#endif

#ifdef COMPARE
  static tPersonDetected *persone_matlab=NULL;
#endif
  //printf("inizio track num_pers=%d\n",num_pers);

  if(enabled==0 && ev_door_close==false) 
  {
    peoplein = people_count_input;
    peopleout = people_count_output;
    return;
  }
#ifdef CHECK_FALSE_COUNTS
  _set_prev_counters(total_sys_number, people_count_input, people_count_output);
#endif
  if(ev_door_close) //evento di porta chiusa
  {
    frame_fermo++;
#ifdef debug_
    if(frame_fermo%10==0) printf("frame fermo\n");
#endif
    if(frame_fermo==200)
    {
      ev_door_close=false; //assicuro un'uscita se ci fossero problemi
    }
    if(total_sys_number<2 || total_sys_number==current_sys_number)
    {
      // 20100604 eVS se sono solo o se sono l'ultimo del widegate
      // cioe' se sono quello che fa il tracking e il conteggio
#ifndef USE_NEW_STRATEGIES
      if(direction==0) SetDoor(0); // soglia_porta = soglia = 0
      else SetDoor(NY); // soglia_porta = soglia = NY
#endif
      CloseDoor(peoplein,peopleout,
        direction,soglia_porta,
        move_det_en,count_true_false,
        num_pers
#ifdef USE_NEW_TRACKING
        , min_y_gap);
#else
        );
#endif

#ifndef USE_NEW_STRATEGIES
      SetDoor(door);
#endif
      if(total_sys_number<2 || total_sys_number==current_sys_number) // eVS 20130628 
        ev_door_close_rec=true;  // l'evento porta chiusa e' stato gestito 
      
      if(direction==0) record_counters(peoplein,peopleout);
      else record_counters(peopleout,peoplein);
      
      if(total_sys_number<2 || total_sys_number==current_sys_number)
      ev_door_close_rec=false;
      
      if(total_sys_number<2) ev_door_close=false;
      else frame_cnt_door=1;
    }
#ifndef USE_NEW_STRATEGIES
#ifdef CHECK_FALSE_COUNTS
    _check_new_counters(total_sys_number, people_count_input, people_count_output, door_size);
#endif
#  ifdef USE_HANDLE_OUT_OF_RANGE
    prev_pp = 0;
#  endif
    return;
#endif
  }

  if(ev_door_open)
  { 
    frame_fermo++;

    if(frame_fermo==200) 
    { //assicuro un'uscita se ci fossero problemi
      ev_door_open=false;
    }

    if(total_sys_number<2 || total_sys_number==current_sys_number) //se wg spento o sono l'ultimo
    {
      if(frame_cnt_door==2)
      { 
        int count_pers=0;
        for(int i=4;i<50;i+=5)
        {
          if(persdata[i]>0) 
          {
            count_pers++;
          }
        }
#ifndef USE_NEW_STRATEGIES
        //  if(total_sys_number<2) frame_cnt_door=0;
        if(direction==0) SetDoor(0); //se c'e' stato l'evento di porta aperta
        else SetDoor(NY);
#endif
      }
      else if(frame_cnt_door<2)
      { 
        if(total_sys_number<2) frame_cnt_door++; //se sono in singoloe
        else //se sono in doppio
        {    //aspetto arrivino tutti i dati con evento attivo
          unsigned char count_event=1;
          for(int r=1; r<54*total_sys_number;r+=54) //conto quanti sistemi stanno eseguendo l'evento 
          {  
            if(data_wide_gate==NULL)
            {
#ifdef debug_
              printf("\ndata_wide_gate non ancora inizializzata!\n\n");
#endif
              return;
            }
            if(data_wide_gate[r]==252) 
            {
              count_event++;
            }
          }

          if(total_sys_number==count_event) frame_cnt_door++;//se tutti i sistemi lo stanno gestendo incremento
        }
        peoplein = people_count_input;
        peopleout = people_count_output;
#ifndef USE_NEW_STRATEGIES
        if(direction==0) SetDoor(0); //se c'e' stato l'evento di porta aperta
        else SetDoor(NY);
        deinitpeople(num_pers);
#ifdef CHECK_FALSE_COUNTS
        _check_new_counters(total_sys_number, people_count_input, people_count_output, door_size);
#endif
        return;
#endif
      }
    } 
  }

  // 20100507 eVS measure performances
#ifdef eVS_TIME_EVAL
  clock_t start, finish;
  static int num_init = 0;

  static clock_t first_time = 0;
  static clock_t prev = 0;

  static unsigned int time_counter = 0;

  static double det_2_det_time = 0;
  static double elapsed_time = 0;

  start = clock();

  if (time_counter > 24000 || (prev != 0 && prev > start)) {
    first_time     = 0;
    prev           = 0;
    time_counter   = 0;
    det_2_det_time = 0;
    elapsed_time   = 0;

    num_init++;
  }

  if (first_time == 0)
    first_time = start;
  if (prev != 0)
    det_2_det_time += ((double)(start - prev)/(double)CLOCKS_PER_SEC);
#endif

  ////////////////////////////////////////////////////////////////////////////
  //// THRESHOLDING AND BACKGROUND SUBTRACTION

  /*!
  <H1>DETTAGLI IMPLEMENTATIVI</H1>

  <B>Sottrazione dello sfondo dalla mappa di disparit&agrave;</B>

  Prima di tutto viene escluso un bordo di 8 pixels
  per ogni pixel dalla mappa di disparita': se il suo valore
  e' minore di una soglia (MIN_M) allora viene settato a zero
  altrimenti se la differenza tra mappa e sfondo e' minore
  del triplo della deviazione standard allora 
  il pixel i-esimo della mappa viene spento

  \code
  memcpy(disparityMapOriginal,disparityMap,NN);

  for(int i=NN-(NX*7+167+1);i>=NX*7+167;--i)
  {
  disvec = &disparityMap[i];
  bkgvec = &Bkgvec[i];

  if(*disvec > MIN_M)
  {
  if(*disvec <= *bkgvec) *disvec = 0;
  else if( (*disvec - *bkgvec) < 3*svec[i]) *disvec = 0;
  }
  else *disvec = 0;
  }
  \endcode
  */

  memcpy(disparityMapOriginal,disparityMap,NN);  // for InitStaticObj

#ifdef USE_NEW_DETECTION
  // binning
  static unsigned char bmap[NN]; 
  static unsigned char BP_map[NN];
  static unsigned char BP_BG[NN];
  static unsigned char bmap_original[NN];
  static int counter_frames_before_oor_check = 0;
  int bnrows, bncols;
  image_binning(disparityMap, NY, NX, binning, BORDER_X, BORDER_Y, bmap, BP_map, bnrows, bncols);
  memcpy(bmap_original,bmap,NN);  // for InitStaticObj
  // update black pixels model
  static BPmodeling bp_model(bncols, bnrows, 0, 0);
#  ifdef USE_HANDLE_OUT_OF_RANGE
  if (ev_door_open && OutOfRangeManager::getInstance().IsOutOfRangeEnabled())
  {
    printf("Imparo il bkg !\n");
    counter_frames_before_oor_check = 0;
    bp_model.Reset();
  }
  if (mem_door && !OutOfRangeManager::getInstance().IsOutOfRange() && OutOfRangeManager::getInstance().IsOutOfRangeEnabled())
  {
#if defined(USE_NEW_TRACKING) && !defined(PCN_VERSION)
    if (reset)
    {
      reset = false;
      bp_model.Reset();
    }
#endif
    bp_model.UpdateModel(BP_map, bncols, bnrows);  // Aggiorno il modello dei BP solo se le porte sono aperte e non sono in ORR
    counter_frames_before_oor_check++;
  }

  // controllo out-of-range
  bp_model.GetMask(BP_BG, bncols, bnrows);  // Recupero la maschera creata dal modello e in caso di persone detectate gestico la situazione di OOR
  if (prev_pp != 0 && counter_frames_before_oor_check >= NUMBER_OF_FRAMES_BEFORE_OOR_CKECK)
  { 
    OutOfRangeManager::getInstance().HandleOutOfRange(prev_persone,
      prev_pp,
      bmap,
      BP_map,
      BP_BG,
      (move_det_en == 0 || count_true_false),
      (door-BORDER_Y+binning/2)/binning,
      bp_model.num_mask_pixel);
    if(counter_frames_before_oor_check == NUMBER_OF_FRAMES_BEFORE_OOR_CKECK)
      printf("inizio gestione OOR !\n");
  }
#  endif




  // sottrazione del background
    unsigned char* ptr_map;
  for (int r=BORDER_Y; r< NY-BORDER_Y; ++r)
  {
     ptr_map = &disparityMapOriginal[r*NX+BORDER_X];
    unsigned char* ptr_bkgvec = &Bkgvec[r*NX+BORDER_X];
    int* ptr_svec = &svec[r*NX+BORDER_X];
    for (int c=BORDER_X; c<NX - BORDER_X; ++c, ++ptr_map, ++ptr_bkgvec, ++ptr_svec)
    {
      if ((*ptr_map <= MIN_M || (*ptr_map - *ptr_bkgvec) < 3*(*ptr_svec)) && *ptr_map != UNIFORM_ZONE_OR_DISP_1)
        *ptr_map = 0;
    }
  }

  unsigned char* ptr_bmap = bmap;
  for (int r=BORDER_Y; r< NY-BORDER_Y; r+=binning)
  {
    unsigned char* ptr_bkgvec = &Bkgvec[r*NX+BORDER_X];
    int* ptr_svec = &svec[r*NX+BORDER_X];
    for (int c=BORDER_X; c<NX-BORDER_X; c+=binning, ++ptr_bmap, ptr_bkgvec+=binning, ptr_svec+=binning)
    {
      if ((*ptr_bmap <= MIN_M || (*ptr_bmap - *ptr_bkgvec) < 3*(*ptr_svec)) && *ptr_bmap != UNIFORM_ZONE_OR_DISP_1)
        *ptr_bmap = 0;
    }
  }


#endif
  // Re-inizializzo le persone
  static tPersonDetected *persone = new tPersonDetected[num_pers];
  int pp=0;
  InitPers(persone);

#ifdef COMPARE
  if(persone_matlab!=NULL) delete [] persone_matlab;
  persone_matlab=new tPersonDetected[num_pers];
  InitPers(persone_matlab);
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  //if (g_use_new_algorithm) // selezione algoritmo (1 nuovo, 0 originale)
#ifdef USE_NEW_DETECTION 
  { // inizio new_algorithm
    
    int num_peaks;
    tPeakProps* peaks = peak_detection(bmap, bnrows, bncols, num_peaks);
    memcpy(disparityMap,disparityMapOriginal,NN);
    peak_unbinning(peaks, num_peaks, binning, BORDER_X, BORDER_Y);
#if defined(USE_NEW_ALGORITHM_MATLAB)
    pp = g_current_num_pers;
#else
    assert(num_peaks<=NUM_PERS_SING);
    pp = min(NUM_PERS_SING, num_peaks);
#endif
    for (int i=0; i<pp; ++i)
    {
#if defined(USE_NEW_ALGORITHM_MATLAB)
      persone[i].x = g_x[i];
      persone[i].y = g_y[i];
      persone[i].wx = g_wx[i]/2;
      persone[i].wy = g_wy[i]/2;
      persone[i].h = g_h[i]/2;
      persone[i].sys = current_sys_number;
#else
      persone[i].x = peaks[i].x;
      persone[i].y = peaks[i].y;
#ifdef USE_NEW_DETECTION2
      persone[i].wx = (peaks[i].wx+1)/2;
      persone[i].wy = (peaks[i].wy+1)/2;
#else
      persone[i].wx = (peaks[i].w+1)/2;
      persone[i].wy = (peaks[i].w+1)/2;
#endif
      persone[i].h = (peaks[i].z+1)/2;
      persone[i].sys = current_sys_number;
#ifdef COMPARE
      persone_matlab[i].x = g_x[i];
      persone_matlab[i].y = g_y[i];
      persone_matlab[i].wx = g_wx[i]/2;
      persone_matlab[i].wy = g_wy[i]/2;
      persone_matlab[i].h = g_h[i]/2;
      persone_matlab[i].sys = current_sys_number;

      int dx = persone[i].x - persone_matlab[i].x;
      int dy = persone[i].y - persone_matlab[i].y;
      int dh = persone[i].h - persone_matlab[i].h;
      int dw = persone[i].wx - persone_matlab[i].wx;

      //printf("diff %4d = #%2d,\tdx %3d,\tdy %3d,\tdh %3d,\tdw %3d\n", num_peaks-g_current_num_pers, i, dx, dy, dh, dw);
      //Sleep(50);
#endif
#endif
    }
  } 

  // fine new_algorithm
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

#else

  { // inizio algoritmo originale

    static unsigned char min_m; 
    static unsigned char *bkgvec,*disvec; //,*pasvec;

    // sottrazione del background
    for(int i=NN-(NX*7+167+1);i>=NX*7+167;--i)
    {
      disvec = &disparityMap[i];
      bkgvec = &Bkgvec[i];

      if(*disvec > MIN_M)
      {
        if(*disvec <= *bkgvec) *disvec = 0;
        else if( (*disvec - *bkgvec) < 3*svec[i]) *disvec = 0;
      }
      else *disvec = 0;
    }

    ////////////////////////////////////////////////////////////////////////////
    // DISPARITY MAP FILTERING

    /*!
    <B>Filtraggi morfologici</B>

    Doppia erosione della mappa di disparita' senza sfondo.

    Espansione della mappa di disparita' precedentemente erosa.
    */

    map_erode(disparityMap);
    map_erode(disparityMap);
    map_dilate(disparityMap);

    //{
    //  IplImage* tmp = cvCreateImageHeader(cvSize(NX, NY), IPL_DEPTH_8U, 1);
    //  cvSetData(tmp, disparityMap, NX);
    //  cvShowImage("map", tmp);
    //  //cvWaitKey(0);
    //  cvReleaseImageHeader(&tmp);
    //}

    //-------     PROIEZIONE SULLE X      --------------------------------------//
    /*!
    <b>Proiezione della mappa di disparit&agrave; filtrata sull'asse X</b>

    La mappa di disparita filtrata viene riscalata su 7 bits
    poi viene presa la massima disparita per ogni colonna (xproj)
    e viene memorizzato il numero di pixel che superano una certa disparita (num_occ)
    \code
    for(int x=NX-1;x>=0;x--) 
    {
    for(int y=BORDI_PROJ_X;y<NY-BORDI_PROJ_X;y++) 
    {
    val7bit=disparityMap[NX*y+x]>>1;     
    if(xproj[x]<val7bit) xproj[x]=val7bit;
    if(val7bit> 24) num_occ[x]++;
    }
    }
    \endcode*/

    //   if(ev_door_open) printf("inizio detezione\n");
    memset(xproj,0,sizeof(xproj)); //NX); 20100512 eVS

    for(int nn=NUM_MAX-1;nn>=0;nn--)
      for(int y=NY-1;y>=0;y--) 
        yproj[nn][y]=0;

    //inizializza strutture
    int val7bit;
    int spi, spi2, j;

    // numero di elementi (per ogni colonna) il cui valore "val7bit" supera le 24 disparita' (su una mappa di 7 bits)
    unsigned char num_occ[NX];
    memset(num_occ,0,sizeof(num_occ)); //NX); 20100512 eVS

    // projection of the disparity map on X axis
    for(int x=NX-1;x>=0;x--) 
    {
      // I hide the border because the head introduce an error
      for(int y=BORDI_PROJ_X;y<NY-BORDI_PROJ_X;y++) 
      {
        val7bit=disparityMap[NX*y+x]>>1;     
        if(xproj[x]<val7bit) 
          xproj[x]=val7bit; 
        if(val7bit> 24) 
          num_occ[x]++;
      }
    }

    /*!
    <b>Rimozione degli spikes dalla proiezione X</b>

    Se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
    supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
    da rimuovere perche' considerato rumore impulsivo (spike).

    \code
    for(int x=4;x<NX-5;x++)
    {
    spi=(xproj[x-1]+xproj[x-2]+xproj[x-3]+xproj[x-4])>>2;
    if(xproj[x]-spi>SPIKE)
    {
    j=0;
    spi2=0;

    while(xproj[x+j]-spi>SPIKE) j++;

    if(j>0) 
    {
    for(int e=0;e<j;e++) spi2+=xproj[x+e];
    spi2=spi2/j;
    if(j<(spi2>>3))
    {
    for(int e=0;e<j;e++)
    xproj[x+e]=spi;
    }
    }
    }
    }
    \endcode

    Si ripete quanto appena visto con i 4 valori successivi (dal fondo, cioe' NX, verso l'inizio, cioe' 0).
    */

    for(int x=4;x<NX-5;x++)
      // removal of spikes from the X axis projection
    {
      spi=(xproj[x-1]+xproj[x-2]+xproj[x-3]+xproj[x-4])>>2;
      if(xproj[x]-spi>SPIKE) // da sinistra verso destra
      {
        j=0;
        spi2=0;

        // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
        // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
        // da rimuovere perche' considerato rumore impulsivo (spike)
        while(xproj[x+j]-spi>SPIKE) j++;

        if(j>0) 
        {
          for(int e=0;e<j;e++) spi2+=xproj[x+e];
          spi2=spi2/j;
          if(j<(spi2>>3))
          {
            for(int e=0;e<j;e++)
              xproj[x+e]=spi;
          }
        }
      }
      spi=(xproj[NX-x+1]+xproj[NX-x+2]+xproj[NX-x+3]+xproj[NX-x+4])>>2;
      if(xproj[NX-x]-spi>SPIKE) // da destra verso sinistra
      {
        j=0;
        spi2=0;

        // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
        // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
        // da rimuovere perche' considerato rumore impulsivo (spike)
        while(xproj[NX-x-j]-spi>SPIKE) j++;

        if(j>0)
        {
          for(int e=0;e<j;e++) spi2+=xproj[NX-x-e];
          spi2=spi2/j;
          if(j<(spi2>>3))
          {
            for(int e=0;e<j;e++)
              xproj[NX-x-e]=spi;
          }
        }
      }
    }

    /*!
    <b>Calcolo soglia ???</b>
    */    
    int medXproj=0;
    int numElMed=0;
    unsigned char valtmp;
    for (int l=NX-1;l>=0;l--)
    {
      valtmp=xproj[l];
      if (valtmp)
      {
        medXproj+=valtmp;
        numElMed++;
      }
    }
    if(numElMed!=0)
    {
      medXproj/=numElMed;
      if (medXproj<=38) min_m=32; // cosa significa 32 ???
      else min_m=40;  // cosa significa 40 ???
    }
    else min_m=40; // cosa significa 40 ???
    //fine calcolo soglia


    //------------------ PROIEZIONE GLOBALE Y -------------------//

    /*!
    <b>Se "diff_cond_1p" attivo allora fai proiezione globale della mappa sull'asse Y</b>

    Se diff_cond_1p==1 allora viene calcolata anche la proiezione globale sull'asse Y
    con successiva eliminazione degli spikes (come per il caso dell'asse X).
    */

    if(diff_cond_1p==1)
    {
      int validi_riga;
      memset(yprojtot,0,sizeof(yprojtot));
      for(int y=0;y<NY;y++)
      {
        validi_riga=0;
        for(int x=0;x<NX;x++)
        {
          val7bit=disparityMap[NX*y+x]>>1;
          if(yprojtot[y]<val7bit) yprojtot[y]=val7bit;
          if(val7bit>24) validi_riga++;
        }
        if(validi_riga<12) yprojtot[y]=0;
      }
      //rimozione spikes
      for(int y=4;y<NY-5;y++)
      {
        spi=(yprojtot[y-1]+yprojtot[y-2]+yprojtot[y-3]+yprojtot[y-4])>>2;
        if(yprojtot[y]-spi>SPIKE)
        {
          j=0;
          spi2=0;

          // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
          // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
          // da rimuovere perche' considerato rumore impulsivo (spike)
          while(yprojtot[y+j]-spi>SPIKE) j++;

          if(j>0)
          {
            for(int e=0;e<j;e++) spi2+=yprojtot[y+e];
            spi2=spi2/j;
            if(j<(spi2>>3))
            {
              for(int e=0;e<j;e++)
                yprojtot[y+e]=spi;
            }
          }
        }
        spi=(yprojtot[NY-y+1]+yprojtot[NY-y+2]+yprojtot[NY-y+3]+yprojtot[NY-y+4])>>2;
        if(yprojtot[NY-y]-spi>SPIKE)
        {
          j=0;
          spi2=0;

          // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
          // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
          // da rimuovere perche' considerato rumore impulsivo (spike)
          while(yprojtot[NY-y-j]-spi>SPIKE) j++;

          if(j>0)
          {
            for(int e=0;e<j;e++) spi2+=yprojtot[NY-y-e];
            spi2=spi2/j;
            if(j<(spi2>>3))
            {
              for(int e=0;e<j;e++)
                yprojtot[NY-y-e]=spi;
            }
          }
        }
      }

      int count_val=0;
      //conto i validi della proiezione
      for(int i=0;i<NY;++i)
        if(yprojtot[i]>32) count_val++;

      /*   if(framecounter%1000==0) 
      {
      printf("count_val=%d min_m=%d\n",count_val,min_m);
      printf("PROIEZIONE:\n");
      for(int i=0;i<NY;++i) printf("%d ",yprojtot[i]);
      printf("\n"); 
      }*/

      if(count_val<13) 
        //memset(yprojtot,0,NN); // perche' NN e non NY???
        memset(yprojtot,0,sizeof(yprojtot)); // 20100512 eVS 
    } // fine if(diff_cond_1p==1)


    //-----------------    MEDIA E DEVIAZIONE STANDARD   -----------------------------//
    int t=0;//numero di modelX trovati
    unsigned char b, bb; //dimensioni modelx corrente
    int summa, summab;
    int meanval, stdev, meanvalb;
    unsigned char i; //fine del modelx corrente
    bool piu_pers;
    unsigned char valpro,xpro2[NX];

    /*!
    <b>Valutazione se una persona o pi&ugrave; persone sulle X</b>

    Assegnazione della modalita' 1 persona piuttosto che piu persone
    in base al numero di valori validi trovati sulla proiezione dell'asse X

    \code
    valpro=0;
    for(int k=NX-1;k>0;k-=3)
    if(xpro2[k]>MIN_VAL&& num_occ[k]>20) 
    valpro++;

    if (valpro>PIUPERS_X)    
    piu_pers=true;
    else
    piu_pers=false;
    \endcode
    */

    //calcolo istogramma zona centrale per avere info sul numero possibile di persone dentro
    memset(xpro2,0,sizeof(xpro2)); //NX); 20100512 eVS
    for(int x=NX-1;x>=0;x--)       //  proiezione della mappa di disparita' sull'asse delle X
    {
      for(int y=BORDI_PROJ_MOD;y<NY-BORDI_PROJ_MOD;y++)
      {
        val7bit=disparityMap[NX*y+x]>>1;     //da aggiungere
        if(xpro2[x]<val7bit) xpro2[x]=val7bit;
      }
    }

    valpro=0;
    for(int k=NX-1;k>0;k-=3) // perche' 3 pixel alla volta piuttosto che 1 alla volta???
    {
      if(xpro2[k]>MIN_VAL &&  num_occ[k]>20) valpro++;
    }

    //assegnazione della modalita' in base al numero di valori validi trovati sulle proiezioni
    if (valpro>PIUPERS_X)    
    {
      piu_pers=true;
    }
    else
    {
      piu_pers=false;
    }

    /*!
    <b>Calcolo media e deviazione standard dei potenziali picchi lungo la proiezione sull'asse delle X</b>

    Calcolo media, deviazione standard, larghezza e posizione del centro dei potenziali blob
    considerando solo la proiezione sull'asse X

    \code
    for(int x=0;x<NX;x+=2) // perche' 2 pixel alla volta piuttosto che 1 alla volta???
    {
    if(xproj[x]>=min_m)
    {
    b=xproj[x]>>1; //  modello usato: rettangolo con base = altezza / 2
    summa=0;
    meanval=0;
    i=x;

    while(i<(b+x) && i<NX)
    {
    summa+=xproj[i];
    ++i;
    }

    if(i>x) meanval=summa/(i-x);

    if(meanval>=min_m)
    { //selezione + accurata del centro della persona
    bb=meanval>>1; 
    summab=0;
    ...    
    \endcode
    */

    for(int x=0;x<NX;x+=2) // perche' 2 pixel alla volta piuttosto che 1 alla volta???
    {
      if(xproj[x]>=min_m)
      {
        b=xproj[x]>>1; //  modello usato: rettangolo con base = altezza / 2
        summa=0;
        meanval=0;
        i=x;

        while(i<(b+x) && i<NX)
        {
          summa+=xproj[i];
          ++i;
        }

        if(i>x) meanval=summa/(i-x);

        if(meanval>=min_m)
        {
          //selezione + accurata del centro della persona
          bb=meanval>>1;
          summab=0;

          for(int l=x;l<=(bb+x);l++)
          {
            if(l<NX)summab+=xproj[l];
            else if(l>=NX)
            {
              bb=l-x;
              break;
            }
          }
          meanvalb=summab/bb; //  media

          if(abs(bb-(meanvalb>>1))<=(bb>>3))
          {
            int sumdif=0;
            for(int n=x;n<(bb+x);n++)
            {
              sumdif+=(meanvalb-xproj[n])*(meanvalb-xproj[n]);
            }
            stdev=(int)sqrt((float)sumdif/bb); //  deviazione standard

            if(stdev<MAX_DEV)
            {
              modelx[t].mean=meanvalb;
              modelx[t].dev=stdev;
              modelx[t].wid=bb>>1;
              modelx[t].pos=x+(bb>>1);
              t++;
            }
          }
        }
      }
    }


    //
    //------------------- MASSIMI LOCALI -------------------
    //
    /*!
    <b>Ricerca dei massimi locali sulla proiezione X</b>
    \code
    for(int i=NUM_MAX-1;i>=0;--i)
    max_x[i].mean=0;

    for(int p=0;p<t;p++)
    if(modelx[p].mean>0)
    maxsearch_x(piu_pers, t);
    \endcode*/

    for(int i=NUM_MAX-1;i>=0;--i)
    {
      max_x[i].mean=0;
    }

    for(int p=0;p<t;p++)
    {
      if(modelx[p].mean>0) //  ricerca dei massimi locali
      {
        maxsearch_x(piu_pers, t, xproj);
      }
    }

    /*!
    <b>Eliminazione dei massimi sovrapposti o troppo vicini sulla proiezione X</b>
    \code
    for (int w=0;w< NUM_MAX;w++)
    {
    if(max_x[w].mean>0)
    {
    for (int ww=0;ww< NUM_MAX;ww++)
    if(max_x[ww].mean>0)
    {
    if(ww!=w)
    {
    if(abs(max_x[ww].pos-max_x[w].pos)<=2*max_x[w].wid)                             
    { // controllo sulle teste (max locali)
    if(max_x[ww].mean<=max_x[w].mean) 
    max_x[ww].mean=0;
    else 
    if(max_x[ww].mean>max_x[w].mean) // questa condizione non e' inutile ???
    max_x[w].mean=0; 
    }
    else if(abs(max_x[ww].pos-max_x[w].pos)<(3*(max_x[w].wid+max_x[ww].wid))/2)                            
    { // controllo teste/spalle
    if(max_x[ww].mean<max_x[w].mean)
    if(abs(max_x[ww].mean-max_x[w].mean)>=(max_x[w].mean>>2))
    max_x[ww].mean=0;  // perche' viene azzerato max_x[ww].mean e non max_x[w].mean ???
    }
    }
    }
    }      
    }
    \endcode*/

    //eliminazione massimi sovrapposti o troppo vicini
    for (int w=0;w< NUM_MAX;w++)
    {
      if(max_x[w].mean>0)
      {
        for (int ww=0;ww< NUM_MAX;ww++)
        {
          if(max_x[ww].mean>0)
          {
            if(ww!=w)
            {
              if(abs(max_x[ww].pos-max_x[w].pos)<=2*max_x[w].wid) 
                //  controllo sulle teste (max locali)
              {
                if(max_x[ww].mean<=max_x[w].mean) max_x[ww].mean=0;
                else if(max_x[ww].mean>max_x[w].mean) max_x[w].mean=0;
                // questa condizione " if(max_x[ww].mean>max_x[w].mean) " non e' inutile ???
              }
              else if(abs(max_x[ww].pos-max_x[w].pos)<(3*(max_x[w].wid+max_x[ww].wid))/2)    //  controllo teste/spalle
              {
                if(max_x[ww].mean<max_x[w].mean)
                  if(abs(max_x[ww].mean-max_x[w].mean)>=(max_x[w].mean>>2))
                    max_x[ww].mean=0; // perche' viene azzerato max_x[ww].mean e non max_x[w].mean ???
              }
            }
          }
        }
      }      
    }

    /*!
    <b>Controllo modalit&agrave; pi&ugrave; persone</b>

    Se nel modo a piu' persone ne ha trovate solo una guardo se ce ne sta un'altra
    \code
    if(piu_pers==true )
    {
    ...    

    for(int p=0;p<NUM_MAX;p++)
    {
    if(max_x[p].mean>0)
    {
    num_max++;
    ind_max=p;
    }
    }

    if(num_max==1) //se ho trovato solo un max eseguo un controllo + accurato
    {
    ...    
    \endcode*/

    // se nel modo a piu' persone ne ha trovate solo una guardo se ce ne sta un'altra
    if(piu_pers==true )
    {
      unsigned char num_max=0;
      unsigned char ind_max=0;
      unsigned char ind_new=0;
      int sommaf=0;
      int meanvf=0;
      int stdevf;
      unsigned char x1,x2;

      for(int p=0;p<NUM_MAX;p++)
      {
        if(max_x[p].mean>0)
        {
          num_max++;
          ind_max=p;
        }
      }

      if(num_max==1) //se ho trovato solo un max eseguo un controllo + accurato
      {
        if (ind_max==0) ind_new=1;
        else ind_new=0;

        unsigned char trovati=0;
        unsigned char pos1=max_x[ind_max].pos-max_x[ind_max].wid;
        unsigned char pos2=max_x[ind_max].pos+max_x[ind_max].wid;
        unsigned char new_pos,dimf;
        x2=pos1;
        x1=pos2;
        //conto quanti valori validi sono presenti a sx del massimo
        while(pos1>0)
        {
          if (xproj[pos1]>MIN_VAL) trovati++;
          if (xproj[pos1]==0) break;
          pos1--;
        }
        //se ci sta un'altra persona
        if(trovati>max_x[ind_max].wid*2)
        {
          //controllo se ci sono i criteri per cui puo essere
          //una persona
          new_pos=(pos1+x2)>>1;
          x2=new_pos+(xproj[new_pos]>>2);
          pos1=new_pos-(xproj[new_pos]>>2);
          sommaf=0;
          for(int l=pos1;l<x2;l++)
          {
            sommaf+=xproj[l];
          }
          dimf=x2-pos1;
          if(dimf>0)
          {
            meanvf=sommaf/dimf; //  media
            sommaf=0;

            int sumdif=0;
            for(int n=pos1;n<x2;n++)
            {
              sumdif+=(meanvf-xproj[n])*(meanvf-xproj[n]);
            }
            stdevf=(int)sqrt((float)sumdif/dimf); //  deviazione standard
            if(stdevf<MAX_DEV)
            {
              max_x[ind_new].mean=meanvf;
              max_x[ind_new].dev=stdevf;
              max_x[ind_new].wid=dimf>>1;
              max_x[ind_new].pos=new_pos;
              t++;
            }
          }
        }
        /*  else //altrimenti lo cerca dalla'ltra parte
        {     */
        trovati=0;
        //conto quanti valori validi sono presenti a dx del massimo
        while(pos2<NX)
        {
          if (xproj[pos2]>MIN_VAL) trovati++;
          if (xproj[pos2]==0) break;
          pos2++;
        }
        if(trovati>max_x[ind_max].wid*2+3)
        {
          new_pos=(pos2+x1)>>1;
          x1=new_pos-(xproj[new_pos]>>2);
          pos2=new_pos+(xproj[new_pos]>>2);
          sommaf=0;
          for(int l=x1;l<pos2;l++)
          {
            sommaf+=xproj[l];
          }
          dimf=pos2-x1;
          if(dimf>0)
          {
            meanvf=sommaf/dimf;                                                    //  media
            sommaf=0;
            if(abs(dimf-(meanvf>>1))<=(dimf>>3))
            {
              int sumdif = 0;
              for(int n=x1;n<pos2;n++)
              {
                sumdif+=(meanvf-xproj[n])*(meanvf-xproj[n]);
              }
              stdevf=(int)sqrt((float)sumdif/dimf);                                            //  deviazione standard
              if(stdevf<MAX_DEV)
              {
                max_x[ind_new].mean=meanvf;
                max_x[ind_new].dev=stdevf;
                max_x[ind_new].wid=dimf>>1;
                max_x[ind_new].pos=new_pos;
                t++;
              }
            }
          }
        }
        //}//fine else
      }//fine num_max==1
    }


    //----------------------------------------------     MULTIPLE PROIEZIONI SULLE Y     --------------------------------------------------//
    /*!
    <b>Proiezioni multiple dei massimi dell'asse X sull'asse delle Y</b>

    Ogni massimo locale trovato (e filtrato dai massimi sovrapposti o troppo vicini)
    sull'asse X viene proiettato sull'asse Y, seguendo poi la stessa regola 
    della massima disparita lungo ogni segmento orizzontale di immagine
    e come per il caso asse X vengono eliminati gli spikes
    \code
    for(nump=0;nump<NUM_MAX;nump++)
    {
    if(max_x[nump].mean>0)
    {
    ...    
    \endcode

    <b>Selezione candidati: calcolo media e dev. std. dei picchi nelle proiezioni Y, trovo massimi e sopprimo massimi locali</b>

    Per ogni massimo locale trovato sull'asse Y (relativamente a ciascun massimo locale dell'asse X)
    ciene calcolata la modalita' 1pers/piu pers come nel caso precedente
    media, dev.std., larghezza e posizione dei potenziali blob (sull'asse Y)
    ricerca dei massimi locali e soppressione dei massimi sovrapposti o troppi vicini
    (sempre analizzando le varie proiezioni sull'asse Y per ciascun massimo dell'asse X)
    ed infine eventuale controllo piu accurato della yproj[nump] nel caso che
    sia stata impostata la modalita' piu persone e l'algoritmo abbia trovato una sola persona.
    */


    //nump=0; ??? inutile e rimosso
    //if(ev_door_open) printf("proiez y\n");
    for(int nn=0;nn<NUM_MAX;nn++)
      for(int i=0;i<NUM_MAX;++i)
        max_y[nn][i].mean=0;

    unsigned char a, aa;//dimensione modely corrente
    int summa2, summa2a;
    int meanval2, stdev2, meanval2a;
    unsigned char l,numzeri;
    int countval;
    int tmp2;
    int nump; // ??? mossa qui (prima era globale)

    for(nump=0;nump<NUM_MAX;nump++)
    {
      if(max_x[nump].mean>0) // proiezione dei massimi delle X sull'asse delle Y
      {
        for(int y=NY-1;y>=0;y--)
        {
          countval=0;
          for(int x=max_x[nump].pos-max_x[nump].wid;x<max_x[nump].pos+max_x[nump].wid;x++)
          {
            tmp2=disparityMap[NX*y+x];
            if(tmp2>MIN_VAL) countval++;//conto quanti valori validi ci sono sulla riga
          }                //serve per eliminare influenza seconda testa

          if (countval>max_x[nump].wid/2)   //se ne ha trovati almeno 1/4 di intervallo
          {
            for(int x=max_x[nump].pos-max_x[nump].wid;x<max_x[nump].pos+max_x[nump].wid;x++)
            {
              val7bit=disparityMap[NX*y+x]>>1;
              if(yproj[nump][y]<val7bit) yproj[nump][y]=val7bit;
            }
          }
        }

        for(int y=4;y<NY-5;y++) // y<NY-5 oppure y<=NY-5 oppure y<NY-4 ???
        {
          spi=(yproj[nump][y-1]+yproj[nump][y-2]+yproj[nump][y-3]+yproj[nump][y-4])>>2;
          if(yproj[nump][y]-spi>SPIKE)
          {
            j=0;
            spi2=0;

            // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
            // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
            // da rimuovere perche considerato rumore impulsivo (spike)
            while(yproj[nump][y+j]-spi>SPIKE) j++;

            if(j>0)
            {
              for(int e=0;e<j;e++) spi2+=yproj[nump][y+e];
              spi2=spi2/j;
              if(j<(spi2>>3))
              {
                for(int e=0;e<j;e++)
                  yproj[nump][y+e]=spi;
              }
            }
          }
          spi=(yproj[nump][NY-y+1]+yproj[nump][NY-y+2]+yproj[nump][NY-y+3]+yproj[nump][NY-y+4])>>2;
          if(yproj[nump][NY-y]-spi>SPIKE)
          {
            j=0;
            spi2=0;

            // se lo scarto tra la proiezione e il valor medio dei 4 valori precedenti
            // supera la soglia SPIKE, allora viene incrementato di uno il contatore degli elementi
            // da rimuovere perche considerato rumore impulsivo (spike)
            while(yproj[nump][NY-y-j]-spi>SPIKE) j++;

            if(j>0)
            {
              for(int e=0;e<j;e++) spi2+=yproj[nump][NY-y-e];
              spi2=spi2/j;
              if(j<(spi2>>3))
              {
                for(int e=0;e<j;e++)
                  yproj[nump][NY-y-e]=spi;
              }
            }
          }
        }

        //--------    MEDIA E DEVIAZIONE STANDARD   -----------------------------//

        int q=0; //numero modely trovati
        //check a grandi linee per capire quante persone ci sono nell'immagine
        valpro=0;
        for(int k=NY-14;k>14;k-=3)
        {
          if(yproj[nump][k]>MIN_VAL) valpro++;
        }

        if (valpro>PIUPERS_Y)
        {
          piu_pers=true;
        }
        else
        {

          piu_pers=false;
        }


        for(int y=0;y<NY;y+=2)
        {
          if(yproj[nump][y]>=min_m)
          {
            a=yproj[nump][y]>>1;
            summa2=0;
            meanval2=0;
            l=y;

            while(l<(a+y) && l<NY)
            {
              summa2+=yproj[nump][l];
              l++;
            }
            if(l>y) meanval2=summa2/(l-y);

            if(meanval2>=min_m){
              aa=meanval2>>1;
              summa2a=0;
              numzeri=0;
              for(int ll=y;ll<=(aa+y);ll++)
              {
                if(yproj[nump][ll]==0) numzeri++;
                if(ll<NY)summa2a+=yproj[nump][ll];
                else if(ll>=NY)
                {
                  aa=ll-y;
                  break;
                }
              }
              //condizione divisione garantita dal bordo nero
              if(numzeri<7 && piu_pers==false) meanval2a=summa2a/(aa-numzeri);
              else meanval2a=summa2a/aa;
              if ( ((abs(aa-(max_x[nump].wid*2))<=10) && piu_pers==false) || (abs(aa-(meanval2a>>1))<=(aa>>3)) && piu_pers==true){

                int sumdif2=0;
                for(int s=y;s<=(aa+y);s++)
                {
                  sumdif2+=(meanval2a-yproj[nump][s])*(meanval2a-yproj[nump][s]);
                }
                stdev2=(int)sqrt((float)sumdif2/aa);


                if(stdev2<MAX_DEV)
                {
                  modely[q].mean=meanval2a;
                  modely[q].dev=stdev2;
                  modely[q].wid=aa>>1;
                  modely[q].pos=y+(aa>>1);
                  q++;
                }
              }
            }
          }
        }


        //-----------------------------------------MASSIMI LOCALI---------------------------------------------------------//
        for(int p=0;p<q;p++)
          if(modely[p].mean>0)
          {
            maxsearch_y(piu_pers, nump, q);
          }

          // soppressione dei massimi locali
          for(int w=0;w< NUM_MAX;w++)
          {
            if (max_y[nump][w].mean>0)
            {
              for (int ww=0;ww< NUM_MAX;ww++)
              {
                if(max_y[nump][ww].mean>0)
                {
                  if(ww!=w)
                  {
                    if(abs(max_y[nump][ww].pos-max_y[nump][w].pos)<=2*max_y[nump][w].wid)
                    {
                      if(max_y[nump][ww].mean<=max_y[nump][w].mean) max_y[nump][ww].mean=0;
                      else if(max_y[nump][ww].mean>max_y[nump][w].mean) max_y[nump][w].mean=0;
                    }
                    else if(abs(max_y[nump][ww].pos-max_y[nump][w].pos)<(3*(max_y[nump][w].wid+max_y[nump][ww].wid))>>1)
                    {
                      if(max_y[nump][ww].mean<max_y[nump][w].mean)
                        if(abs(max_y[nump][ww].mean-max_y[nump][w].mean)>=(max_y[nump][w].mean/4)) //>>2
                          max_y[nump][ww].mean=0;
                    }
                  }
                }
              }
            }
          }

          //se nel modo a piu persone ne ha trovate solo una guardo se ce ne sta un'altra
          if(piu_pers==true )
          {
            unsigned char num_max=0;
            unsigned char ind_max=0;
            unsigned char ind_new=0;
            int sommaf=0;
            int meanvf=0;
            int stdevf;
            unsigned char y1,y2;
            for(int p=0;p<NUM_MAX;p++)
            {
              if(max_y[nump][p].mean>0)
              {
                num_max++;
                ind_max=p;
              }
            }
            if(num_max==1)
            {
              if (ind_max==0) ind_new=1;   //forse non serve
              else ind_new=0;

              unsigned char trovati=0;
              unsigned char pos1=max_y[nump][ind_max].pos-max_y[nump][ind_max].wid;
              unsigned char pos2=max_y[nump][ind_max].pos+max_y[nump][ind_max].wid;
              unsigned char new_pos,dimf;
              y2=pos1;
              y1=pos2;
              while(pos1>0)
              {
                if (yproj[nump][pos1]>MIN_VAL) trovati++;
                if (yproj[nump][pos1]==0) break;
                pos1--;
              }
              if(trovati>max_y[nump][ind_max].wid*2)
              {
                new_pos=(pos1+y2)/2;
                y2=new_pos+yproj[nump][new_pos]/4;
                pos1=new_pos-yproj[nump][new_pos]/4;
                sommaf=0;
                for(int l=pos1;l<y2;l++)
                {
                  sommaf+=yproj[nump][l];
                }
                dimf=y2-pos1;
                if(dimf>0)
                {
                  meanvf=sommaf/dimf;                                                    //  media
                  sommaf=0;
                  if(abs(dimf-(meanvf>>1))<=(dimf>>3))
                  {
                    int sumdif=0;
                    for(int n=pos1;n<y2;n++)
                    {
                      sumdif+=(meanvf-yproj[nump][n])*(meanvf-yproj[nump][n]);
                    }
                    stdevf=(int)sqrt((float)sumdif/dimf);          //  deviazione standard
                    if(stdevf<MAX_DEV)
                    {
                      max_y[nump][ind_new].mean=meanvf;
                      max_y[nump][ind_new].dev=stdevf;
                      max_y[nump][ind_new].wid=dimf>>1;
                      max_y[nump][ind_new].pos=new_pos;
                      q++;
                    }
                  }
                }
              }
              else //altrimenti lo cerca dall'altra parte
              {
                trovati=0;
                while(pos2<NY)
                {
                  if (yproj[nump][pos2]>MIN_VAL) trovati++;
                  if (yproj[nump][pos2]==0) break;
                  pos2++;
                }
                if(trovati>max_y[nump][ind_max].wid*2)
                {
                  new_pos=(pos2+y1)/2;
                  y1=new_pos-yproj[nump][new_pos]/4;
                  pos2=new_pos+yproj[nump][new_pos]/4;
                  sommaf=0;
                  for(int l=y1;l<pos2;l++)
                  {
                    sommaf+=yproj[nump][l];
                  }
                  dimf=pos2-y1;
                  if(dimf>0)   meanvf=sommaf/dimf;                                                    //  media
                  sommaf=0;                                
                  if(abs(dimf-(meanvf>>1))<=(dimf>>3))
                  {
                    int sumdif=0;
                    for(int n=y1;n<pos2;n++)
                    {
                      sumdif+=(meanvf-yproj[nump][n])*(meanvf-yproj[nump][n]);
                    }
                    stdevf=(int)sqrt((float)sumdif/dimf);                                            //  deviazione standard
                    if(stdevf<MAX_DEV)
                    {
                      max_y[nump][ind_new].mean=meanvf;
                      max_y[nump][ind_new].dev=stdevf;
                      max_y[nump][ind_new].wid=dimf>>1;
                      max_y[nump][ind_new].pos=new_pos;
                      q++;
                    }
                  }
                }
              }//fine else
            }//fine num_max==1
          }


          for(int i=NY-1;i>=0;--i)
            modely[i].mean=0;

      }
    }


    //----------------------------------------------------------------------------
    //
    //                          RILEVAMENTO DELLE TESTE IN 2D 
    //
    //----------------------------------------------------------------------------
    /*!
    <b>Rilevamento delle teste in 2D dati i candidati ottenuti dall'analisi dei massimi nelle due proiezioni su X e Y</b>

    Se l'opzione "difficult condition one person" e' stata selezionata sull'interfaccia win_client
    allora tutti i pixel non neri presenti sulla mappa di disparita senza sfondo e filtrata,
    vengono considerati come appartenenti ad un unico grande blob
    percio' il centroide viene calcolato come il centro di massa di tutti i pixel.

    Calcolo del centro di massa sull'asse X
    \code
    for(int c=0;c<NX;c++) 
    {
    massa_tot_x+=xproj[c]; // somma dei pesi
    massa_pesata_x+=c*xproj[c];
    ...
    }
    \endcode

    Calcolo del centro di massa sull'asse Y
    \code    
    for(int r=0;r<NY;r++)
    {
    massa_tot_y+=yprojtot[r];  // somma dei pesi
    massa_pesata_y+=r*yprojtot[r];
    ...
    }               
    \endcode

    Se invece l'opzione "difficult condition one person" non e' stata selezionata, allora
    per ogni massimo locale trovato sull'asse X, vengono scanditi tutti i relativi massimi locali
    calcolati sull'asse Y e vengono eseguiti una serie di calcoli per capire se il potenziale blob
    contiene un certo numero minimo di pixel non neri (quindi se ha una certa densita minima)
    se si allora il potenziale blob viene aggiunto alla lista delle persone trovate nel frame corrente.

    Eventuale eliminazione delle false terze persone ???
    \code
    for(int pe=0;pe<pp;pe++)      
    {
    for(int ue=0;ue<pp;ue++)
    {
    if((pe!=ue)&&(persone[pe].y==persone[ue].y))
    for(int ze=0;ze<pp;ze++)
    {
    if((ze!=pe)&&(ze!=ue)&&(abs(persone[pe].y-persone[ze].y)<3*persone[ze].wy)
    &&(abs(persone[pe].x-persone[ze].x)<persone[ze].wx))
    persone[pe].h=0;
    }
    }
    }    
    \endcode

    Se c'e' un solo sistema collegato allora la lista viene passata cosi com'e' all'algoritmo di tracking,
    altrimenti viene ulteriormente filtrata per evitare che una persona venga contata piu di una volta
    (il filtraggio avviene mediante il calcolo della x real e tramite la funzione Merge_and_Filter_people).
    */

    int mean_v;
    int hei;
    int som;

    if(diff_cond_1p==0)
    {
      //allocazione delle persone in base al numero di sistemi
      for(int g=0; g<NUM_PERS_SING;g++)
      {
        // perche vengono considerati solo i primi 10 elementi di max_x ???
        if(max_x[g].mean>0)
        {
          for(int h=0;h<NUM_PERS_SING;h++)
            // perche vengono considerati solo i primi 10 elementi di max_y ???
            if(max_y[g][h].mean>0)
              //  controllo della similarita' tra le altezze dei rettangoli corrispondenti
            {
              mean_v = (max_y[g][h].mean+max_x[g].mean)>>1;

              // perche se ci sono piu persone viene considerata l'altezza media /2
              // e invece se c'e solo una persona viene considerata l'altezza media /4 ???
              if(piu_pers==false)
                hei=mean_v>>2;
              else
                hei=mean_v>>1;

              if((abs(max_y[g][h].mean-max_x[g].mean)<=hei)  )
                //  verifica della densita' di punti nella mappa di disparita'
              {
                som=0;
                for(int j=-max_x[g].wid;j<=max_x[g].wid;j++) 
                  for(int i=-max_y[g][h].wid;i<=max_y[g][h].wid;++i)
                  {
                    if(disparityMap[((max_y[g][h].pos+i)*NX)+max_x[g].pos+j]>mean_v-hei)
                      som++;
                  }
                  //  inserimento nelle strutture "persone"
                  if(som>(max_y[g][h].wid*max_x[g].wid))
                  {
                    if(pp<10)
                    {
                      persone[pp].x=max_x[g].pos;
                      persone[pp].y=max_y[g][h].pos;
                      persone[pp].wx=max_x[g].wid;
                      persone[pp].wy=max_y[g][h].wid;
                      persone[pp].h=mean_v;
                      persone[pp].sys=current_sys_number;
                      persone[pp].real_x=0;
                      persone[pp].sincro=count_sincro;
                      persone[pp].stato_canc_da=0;
                      persone[pp].ho_canc_su_sys=0;
                      persone[pp].h_se_cancellato=0;
                      pp++;
                    }
#ifdef debug_ 
                    else printf("Tracking Problem pp>10!!!\n");
#endif
                  }
              }
            }
        }
      }

      for(int pe=0;pe<pp;pe++)      //  eliminazione dei fantasmi ??? (falsa terza persona)
      {
        for(int ue=0;ue<pp;ue++)
        {
          if((pe!=ue)&&(persone[pe].y==persone[ue].y))
            for(int ze=0;ze<pp;ze++)
            {
              if((ze!=pe)&&(ze!=ue)&&(abs(persone[pe].y-persone[ze].y)<3*persone[ze].wy)
                &&(abs(persone[pe].x-persone[ze].x)<persone[ze].wx))
                persone[pe].h=0;
            }
        }
      }
    } // fine if(diff_cond_1p==1)
    else
    {
      int validi_x=0;
      int validi_y=0;
      int val_x_tmp=0;
      int val_x_max=0; //contiene il numero massimo di valori >0 contigui (senza buchi)
      int val_y_tmp=0;
      int val_y_max=0; //contiene il numero massimo di valori >0 contigui (senza buchi)
      int temp_wx,temp_wy;

      // calcolo del centro di massa sull'asse X
      unsigned long massa_tot_x = 0;
      unsigned long massa_pesata_x = 0;
      for(int c=0;c<NX;c++) // rinominato r in c!!!
      {
        massa_tot_x+=xproj[c]; // somma dei pesi
        massa_pesata_x+=c*xproj[c];
        if(xproj[c]>0)
        {
          validi_x++;
          val_x_tmp++;
        }
        else
        {
          if(val_x_tmp>val_x_max) val_x_max=val_x_tmp;
          val_x_tmp=0;
        }
      }

      // calcolo del centro di massa sull'asse Y
      unsigned long massa_tot_y = 0;
      unsigned long massa_pesata_y = 0;
      for(int r=0;r<NY;r++)
      {
        massa_tot_y+=yprojtot[r];  // somma dei pesi
        massa_pesata_y+=r*yprojtot[r];
        if(yprojtot[r]>0)
        {
          validi_y++;
          val_y_tmp++;
        }
        else
        {
          if(val_y_tmp>val_y_max) val_y_max=val_y_tmp;
          val_y_tmp=0;
        }
      }

      if(massa_tot_y>0 && massa_tot_x>0)
      {
        int h_media_x=massa_tot_x/validi_x;
        int h_media_y=massa_tot_y/validi_y;
        mean_v=(h_media_x+h_media_y)>>1;
        if(h_media_x>12 && h_media_y>12 && val_x_max>12 && val_y_max>12)
        {
          persone[0].h=mean_v;
          persone[0].x=massa_pesata_x/massa_tot_x;
          persone[0].y=massa_pesata_y/massa_tot_y; //somma_y/max_pres;
          temp_wx=h_media_x/2;
          if(temp_wx>20) temp_wx=20;

          //controllo per evitare di sforare nello scrivere la croce
          if(persone[0].x+temp_wx>=NX)
            temp_wx=NX-persone[0].x-4;
          if(persone[0].x-temp_wx<=0)
            temp_wx=persone[0].x-4;

          persone[0].wx=temp_wx;

          temp_wy=2*h_media_y/5;
          if(temp_wy>20) temp_wy=20;

          //controllo per evitare di sforare nello scrivere la croce
          if(persone[0].y+temp_wy>=NY)
            temp_wy=NY-persone[pp].y-4;
          if(persone[0].y-temp_wy<=0)
            temp_wy=persone[0].y-4;

          persone[0].wy=temp_wy;
          pp++;
        }
        //controllo densita minima
        int densita=0;
        int area=4*persone[0].wx * persone[0].wy;
        for(int w=persone[0].x-persone[0].wx;w<persone[0].x+persone[0].wx;w++)
          for(int h=persone[0].y-persone[0].wy;h<persone[0].wy+persone[0].y;h++)
            if(disparityMap[h*NX+w]>min_m)  densita++;

        if(densita<area/3) persone[0].h=0;
      }
    }


    //-------------inserimento persone per il tracciamento-----------------//
    /*!
    <b>Se in widegate e sono pcn master allora fondo liste persone</b>

    Se ci sono piu' sistemi e se sono sull'ultimo
    allora prima costruisco la lista di persone
    trovate nel frame corrente e poi la filtro
    \code
    if(total_sys_number > 1 && total_sys_number==current_sys_number) 
    {
    ...
    CalculateReal_x(); // calcolo della x reale per evitare che due o piu' sistemi collegati in serie contino due volte la stessa persona        
    Merge_and_Filter_people(pp); // unione e filtraggio della lista di persona trovate nel frame corrente        
    ...
    }
    \endcode

    <b>La lista persone viene suddivisa in people_coor, dimpers e hpers</b>

    people_coor e' un vettore di variabili di tipo per che contiene le coordinate 
    del centroide delle persone trovate nel frame corrente.

    persone invece e' una lista a struttura dati di tipo per che contiene le stesse 
    informazioni di people_coor piu' altre informazioni relative alla persona 
    trovata nel frame corrente.

    hpers e' un vettore che contiene le informazioni relative all'altezza delle persone 

    dimpers e' un vettore che contiene le informazioni relative alla larghezza e 
    altezza delle teste trovate nel frame corrente.
    trovate nel frame corrente, ovvero la disparita'.

    \code
    unsigned char dimpers[num_pers*2]; //contiene le dimensioni delle persone trovate
    unsigned char hpers[num_pers];     //contiene l'altezza delle persone trovate
    int people_coor[num_pers];         //contiene coordinate persone trovate

    people_coor[i] = riga*160*total_sys_number+colonna
    if(total_sys_number==0) 
    people_coor[pe]=(xt*persone[pe].y)+persone[pe].x; 
    else 
    people_coor[pe]=(xt*persone[pe].y)+persone[pe].real_x;

    hpers[pe]=persone[pe].h;

    //dimpers[pe+inddim]=persone[pe].wx; // 20110930 eVS, bug fix on indexes
    //dimpers[pe+inddim+1]=persone[pe].wy;
    //inddim++;
    dimpers[2*pe+0]=persone[pe].wx;  // 20110930 eVS, bug fix on indexes
    dimpers[2*pe+1]=persone[pe].wy;
    \endcode*/

  }

  // fine algoritmo originale
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

  unsigned char* dimpers = new unsigned char[num_pers*2]; //contiene le dimensioni delle persone trovate
  unsigned char* hpers = new unsigned char[num_pers]; //contiene l'altezza delle persone trovate
  int* people_coor = new int[num_pers];

  //inserimento di tutte le persone della daisy chain nella struttura
  //if(ev_door_open) printf("inserimento persone altri sistemi\n");

  // 20110930 eVS, moved here the common initialization
  for (int r=num_pers-1;r>=0;r--) 
  {
    hpers[r]=0;
    people_coor[r]=0;
    dimpers[2*r+0]=0;
    dimpers[2*r+1]=0;
  }

  // 20100603 eVS attention!!! the last sensor in the chain is the one that
  // has to check for false double counting using the real_x value
  if(total_sys_number > 1 && total_sys_number==current_sys_number) 
    //se sono lo slave485 e ho ricevuto dati validi Calcolo le realx e aggiungo le pers
  {//NB: il main e' gia bloccato prima della chiamata della tracking
    //int inddim=0;
    //for (int r=2*num_pers-1;r>=0;r--)  // moved before if
    //{
    //    dimpers[r]=0;
    //    if(r<num_pers)
    //    {
    //        hpers[r]=0;
    //        people_coor[r]=0;
    //    }
    //}

    int pers=0;
    int pers_aggiunte=0;
    unsigned char system_number;
    unsigned char sincro;
    for(int sys=0;sys<(total_sys_number-1);sys++)
    {
      if(data_wide_gate==NULL)
      {
#ifdef debug_
        printf("\ndata_wide_gate non ancora inizializzata!\n\n");
#endif
        return;
      }
      system_number=data_wide_gate[sys*54+2];

      sincro=data_wide_gate[sys*54+3];
      pers=0;
      pers_aggiunte=0;
      while((pers<NUM_PERS_SING)&&(data_wide_gate[sys*54+4+pers*5]!=0))
      {
        persone[pp].h=data_wide_gate[sys*54+4+pers*5];
        persone[pp].x=data_wide_gate[sys*54+5+pers*5];
        persone[pp].y=data_wide_gate[sys*54+6+pers*5];
        persone[pp].wx=data_wide_gate[sys*54+7+pers*5];
        persone[pp].wy=data_wide_gate[sys*54+8+pers*5];
        persone[pp].real_x=0;
        persone[pp].sys=system_number;
        persone[pp].sincro=sincro;
        persone[pp].stato_canc_da=0;
        persone[pp].ho_canc_su_sys=0;
        persone[pp].h_se_cancellato=0;
        pp++;
        pers_aggiunte++;
        pers++;
      }
    }

    CalculateReal_x(persone);
    Merge_and_Filter_people(persone, pp);

    if(pp>=num_pers)
    {
#ifdef debug_
      printf("ERROR pp=%d \n",pp);
#endif
      pp=num_pers;
    }

    for(int pe=0;pe<pp;pe++)
    {
      if(persone[pe].h>0)
      {
        // people_coor e' un vettore di variabili di tipo int che contiene le coordinate 
        // del centroide delle persone trovate nel frame corrente. 
        // people_coor[i] = riga*xt+colonna
        // xt=160 se gate normale oppure xt=160*total_sys_number se in widegate.

        hpers[pe]=persone[pe].h;
        if(total_sys_number==0) 
          people_coor[pe]=(xt*persone[pe].y)+persone[pe].x; 
        else 
          people_coor[pe]=(xt*persone[pe].y)+persone[pe].real_x;

        // dimpers e' un vettore che contiene le informazioni relative alla larghezza e 
        // altezza delle teste trovate nel frame corrente.
        // hpers e' un vettore che contiene le informazioni relative all'altezza delle persone 
        // trovate nel frame corrente, ovvero la disparita'.

        dimpers[2*pe+0]=persone[pe].wx; // 20110930 eVS, bug fix on indexes
        dimpers[2*pe+1]=persone[pe].wy;
      }
      //inddim++; //20110929 eVS, moved here
    }
  }
  else //metto le persone nelle strutture senza calcolare real_x e fusione
  {
    //for (int r=2*num_pers-1;r>=0;r--)  // moved before if
    //{
    //    dimpers[r]=0;
    //    if(r<num_pers)
    //    {
    //        hpers[r]=0;
    //        people_coor[r]=0;
    //    }
    //}

    if(pp>=num_pers)
    {
#ifdef debug_
      printf("ERROR pp=%d \n",pp);
#endif
      pp=num_pers;
    }
    for(int pe=0;pe<pp;pe++)
    {
#ifdef debug_
      printf("limit UP %d,  limit DOWN %d\n",limit_line_Up,limit_line_Down);
#endif 
      if(persone[pe].h>0) {
        if(!((persone[pe].y>=limitSx_riga_start) && (persone[pe].y<=limitSx_riga_end) && (persone[pe].x<=limitSx)))
          if(!((persone[pe].y>=limitDx_riga_start) && (persone[pe].y<=limitDx_riga_end) && (persone[pe].x>=limitDx)))
            if(!((persone[pe].y<=limit_line_Up) || (persone[pe].y>=limit_line_Down)))
            {
              // se la persona si trova fuori dalla "no tracking zone"
              hpers[pe]=persone[pe].h;
              people_coor[pe]=(xt*persone[pe].y)+persone[pe].x;
              dimpers[2*pe+0]=persone[pe].wx;
              dimpers[2*pe+1]=persone[pe].wy;
            }
      }
      //inddim++; //20110929 eVS, moved here
    }
  }

  /*!
  <b>Se sono in widegate ma NON sono il master</b>
  \code
  if(total_sys_number>1 && total_sys_number!=current_sys_number) 
  WritePersDetected(persone, pp);
  \endcode
  */
  if(total_sys_number>1 && total_sys_number!=current_sys_number) 
    WritePersDetected(persone, pp);

  /*!
  <b>Se sono state rilevate delle persone e se sono il master (o l'unico) lancio il tracking</b>
  \code
  if(pp!=0)
  {
  if(total_sys_number<2 || total_sys_number==current_sys_number)
  {
  track(people_coor, pp, disparityMap, dimpers,hpers,peoplein,peopleout, diff_cond_1p);
  }
  }
  \endcode
  */
  if(pp!=0)
  {
    if(total_sys_number<2 || total_sys_number==current_sys_number) // ???IL MASTER 485 non la esegue??? Come? La esegue solo il master!!!
    {
      //persone passate alla track
#ifdef debug_
      memset(persdebug,0,320);
      persdebug[0]=num_pers;
      int ind_vect=1;
      for(int i=0;i<pp;++i)
      {
        persdebug[ind_vect]=hpers[i];
        persdebug[ind_vect+1]=0; //x
        persdebug[ind_vect+2]=people_coor[i]%xt; //x
        persdebug[ind_vect+3]=people_coor[i]/xt; //y
        persdebug[ind_vect+4]=dimpers[2*i+0];//wx //20110930 eVS, bugfix of indexes
        persdebug[ind_vect+5]=dimpers[2*i+1];//wy
        ind_vect+=6;
      }
#endif
      //fine debug
      /*       if(framecounter%300==0)
      {
      for(int r=0; r<num_pers;r++)
      {
      if(persone[r].h!=0)
      printf("chiamata track: persone[%d].h=%d .x=%d .real_x=%d .y=%d .wx=%d .wy=%d xt=%d\n",r,persone[r].h,persone[r].x,persone[r].real_x,persone[r].y,persone[r].wx,persone[r].wy,xt);
      }
      printf("pp=%d persdebug:",pp);
      for(int h=0;h<40;h++)
      printf(" %d",persdebug[h]);
      printf("\n");
      } */

      //       if(ev_door_open) printf("chiamata track total_sys_number=%d current_sys_number=%d\n",total_sys_number,current_sys_number);

      track(people_coor, pp, 
        disparityMap, 
        dimpers, hpers, 
        peoplein, peopleout, 
        diff_cond_1p, 
        direction, soglia_porta,
#ifndef USE_NEW_TRACKING  // 20130411 eVS, for 2.3.10.7 door_kind is not anymore used in track() but for the old algorithm we need to pass a value
        0,  // two way door
#endif
        current_sys_number, total_sys_number,
        door_stairs_en, move_det_en, count_true_false,
        num_pers,xt
#ifdef USE_NEW_TRACKING
        , min_y_gap);
#else
        );
#endif

    }
    numFrameClean=0;

  }
  /*!
  <b>Se non sono state rilevate persone e non ho disabilitato il background automatico faccio aggiornamento del background</b>
  \code
  //20090506 Lisbona (aggiunta flag di disabilitazione)
  //if(pp==0) 
  if(pp==0)
  {
  if (abs(vm_img-vm_bkg)<soglia_bkg  && autobkg_disable==0)
  {
  ... aggiorna background ogni 8 fotogrammi in cui non sono ...
  ... presenti persone, in cui il valor di grigio medio e' simile ...
  ... a quello che c'era durante l'acquisizione del background e ...
  ... se l'autobackground e' abilitato, i.e. autobkg_disable==0
  }
  ...
  }
  \endcode
  */
  //20090506 Lisbona
  //else if(pp==0) 
  //else if((pp==0) && autobkg_disable==0) // 20100512 eVS, "autobkg_disable==0" moved in the next if
  else if(pp==0) 
    //20090506 fine modifiche Lisbona
  {
    //if(abs(vm_img-vm_bkg)<soglia_bkg) // 20100512 eVS, "autobkg_disable==0" moved here
    // 20100517 eVS commentato codice per background automatico per evitare ogni dubbio
    /*if(abs(vm_img-vm_bkg)<soglia_bkg  && autobkg_disable==0) 
    {
    // se e' la prima volta inizializzo la varianza a 0
    if(numFrameClean==0)
    {
    //for(int h=NN-1;h>=0;h--) svectmp[h]=0;
    for(int h=NN-1;h>=0;h--) svectmp[h]=INITIAL_STD_BKG;
    }
    int r2;

    // ogni 8 fotogrammi aggiorno il background
    //if(var && numFrameClean%8==0 && numFrameClean!=0) // eVS tolta la variabile var
    if(numFrameClean%8==0 && numFrameClean!=0)
    {
    for(int y2=0;y2<NY;y2++)for(int x2=0;x2<NX;x2++)
    {
    r2=y2*NX+x2;
    disvec = &disparityMapOriginal[r2];
    bkgvec = &Bkgvectmp[r2];
    if(*disvec > 0 || svectmp[NX*y2+x2]>0)
    {
    if(x2 < BORDER_X || x2 >= NX-BORDER_X || y2 < BORDER_Y || y2 >= NY-BORDER_Y)
    *disvec=0;
    *bkgvec=(15*(*bkgvec)+(*disvec)) >> 4; // 20100416 eVS, aumentata inerzia
    // *bkgvec=((int)(31*((int)(*bkgvec))+((int)(*disvec))) >> 5);
    svectmp[r2]=((int)sqrt((15*svectmp[r2]*svectmp[r2]+(*disvec-*bkgvec)*(*disvec-*bkgvec)) >> 4));
    //svectmp[r2]=((int)sqrt((31*svectmp[r2]*svectmp[r2]+((int)(*disvec)-(int)(*bkgvec))*((int)(*disvec)-(int)(*bkgvec))) >> 5));
    }
    } 
    }
    numFrameClean++;

    // dopo FRAME_CLEAN frames "puliti" e consecutivi (si noti la reinizializzazione alla fine del precedente ramo if pp!=0)
    // se il nuovo background (Bkgvectmp) e' "simile" a quello usato ora (Bkgvec), aggiorna il background con quello nuovo
    if(numFrameClean == FRAME_CLEAN)
    {
    // verifico la similarita' tra nuovo e vecchio background
    int count1=0;
    for(int i=NN-1;i>=0;--i) //per evitare problema di copertura campo di vista
    {
    if(abs(Bkgvec[i]-Bkgvectmp[i])>16) count1++;
    }

    // se sono simili allora aggiorno
    if(count1<MAX_DIFF_BKG)
    {
    //if(var) 
    //{
    for(int h=NN-1;h>=0;h--) 
    svec[h]=svectmp[h];
    //}
    memcpy(Bkgvec,Bkgvectmp,NN);
    // eVS come da nota è stato tolto l'if e di conseguenza anche la variabile var
    SaveBkg(); //NOTA:togliere if
    }
    //memcpy(Bkgvectmp,disparityMapOriginal,NN); //??? perche' re-inizializza con la mappa corrente? Non ha senso!!! // 20100416 eVS
    numFrameClean=0;
    }
    }*/ 

    clearpeople(peoplein, peopleout, soglia_porta, move_det_en, count_true_false, num_pers
#ifdef USE_NEW_TRACKING
      , min_y_gap);
#else
      );
#endif

    if(total_sys_number>1 && current_sys_number!=total_sys_number)
    {
      peoplein = people[0];
      peopleout = people[1];
    }
  }

  delete [] dimpers; 
  delete [] hpers; 
  delete [] people_coor; 

#ifdef USE_HANDLE_OUT_OF_RANGE
  tPersonDetected* tmp = prev_persone;
  prev_persone = persone;
  persone = tmp;
  prev_pp = pp;
#endif

  /*!
  <b>Euristiche per gestire apertura della porta</b>
  \code
  if(ev_door_open==true && frame_cnt_door==2) //se c'e' stato l'evento di porta aperta  
  {			  //risistemo la soglia e cancello l'evento
  if(total_sys_number<2 || total_sys_number==current_sys_number)
  {
  SetPassi(direction, num_pers);
  SetDoor(door);
  if(direction==0) 
  record_counters(peoplein,peopleout);
  else 
  record_counters(peopleout,peoplein);
  if(total_sys_number<2) 
  ev_door_open=false;
  frame_cnt_door=3;
  if(total_sys_number<2) frame_cnt_door=0;
  }
  }
  \endcode
  */
  if(ev_door_open==true && frame_cnt_door==2) //se c'e' stato l'evento di porta aperta  
  {			  //risistemo la soglia e cancello l'evento
    if(total_sys_number<2 || total_sys_number==current_sys_number)
    {
#ifndef USE_NEW_STRATEGIES
#  ifndef USE_NEW_TRACKING
      SetPassi(direction, num_pers);
#  endif
      SetDoor(door);
#endif
      if(direction==0) record_counters(peoplein,peopleout);
      else record_counters(peopleout,peoplein);
      if(total_sys_number<2) ev_door_open=false;
      frame_cnt_door=3;
      if(total_sys_number<2) frame_cnt_door=0;
    }
    //if(frame_cnt_open==2)frame_cnt_open=3; ???
  }

  // 20100507 eVS measure performances
#ifdef eVS_TIME_EVAL
  {
    finish = clock();
    time_counter++;

    //elapsed_time = max(elapsed_time, ((double)(finish - start)/(double)CLOCKS_PER_SEC));
    elapsed_time += ((double)(finish - start)/(double)CLOCKS_PER_SEC);

    if ((time_counter % 70) == 0)
    {
      FILE *time_file;
      if((time_file = fopen("/tmp/time_file.txt","w")))
      {
        fprintf(time_file, "num. init: %d \t time_counter: %d\n", num_init, time_counter);
        fprintf(time_file,    "processing time: %f sec.\n",elapsed_time/(double)time_counter);
        double total_time = ((double)(finish - first_time)/(double)CLOCKS_PER_SEC);            
        fprintf(time_file,    "processing time: %f sec.\n",total_time/(double)time_counter);
        if (time_counter > 1)
          fprintf(time_file,"acq. period:     %f sec.\n",det_2_det_time/(double)(time_counter-1));
        fclose(time_file);
      }
    }

    // per evitare di avere tempi forviati dal salvataggio del file
    // aggiorno prev dopo il salvataggio
    prev = clock();
  }
#endif
#ifdef CHECK_FALSE_COUNTS
  _check_new_counters(total_sys_number, people_count_input, people_count_output, door_size);
#endif
  return;
}

#ifndef USE_NEW_DETECTION
/*! 
\brief Ricerca i massimi locali all'interno della proiezione globale sull'asse X.

Analizzando il vettore modelx (che contiene i potenziali blob dal punto di vista dell'asse X) 
riesco ad estrarre i massimi locali dell'asse X e metterli dentro al vettore max_x.

\param fun_mode Is the boolean variable: if fun_mode = true then there are more people else there is one person.
\param t  Number of modelX found
*/
void maxsearch_x(unsigned char fun_mode, int t, unsigned char xproj[NX])
{
  unsigned char c=0;
  int mmean=20;
  int uguali=0;
  unsigned char int_max,validi,maxtmp;
  int pmin,pmax,pmed;
  pmin=0;
  pmax=0;
  pmed=0;
  maxtmp=0;
  validi=0;
  int_max=0;

  //trovo il centro della proiezione
  //nota: questa parte si puo' portar fuori e cambiare il prototipo di maxsearch
  for(int l=NX-1;l>=0;l--)
  {
    if(xproj[l]>24)  validi++;
    else
    {
      if(validi>int_max)
      {
        pmax=maxtmp;
        pmin=l;
      }
      validi=0;
      maxtmp=l;
    }
  }

  pmed=(pmin+pmax)>>1;

  if(fun_mode==true) //se sono presenti + persone
  {
    for(int e=0;e<t;e++)
    {
      // domanda: se sia dentro l'if che dentro l'else c'e' lo stesso codice (a meno di un =) che senso ha l'if???
      if(modelx[e].pos >= pmed) //se sono piu a dx del centro proiez
      {
        if(modelx[e].mean>=mmean) //prendo il massimo piu a dx
        {
          mmean=modelx[e].mean;
          c=e;
        }
      }
      else //se sono piu a sx del centro proiez
      {
        if(modelx[e].mean>mmean) //prendo il massimo piu a sx
        {
          mmean=modelx[e].mean;
          c=e;
        }
      }
    }

    if(modelx[c].pos> pmed + pmed/4) // perche c'e' un termine pmed/4 ???
    {
      int old_c=c;
      while(c<old_c+10 && c<NX-1) //permetto al massimo uno shift di 10 pixel
      {
        if(modelx[c+1].mean==mmean-1) c++;
        else break;
      }
    }
    else if(modelx[c].pos < 3*(pmed-pmin)/4 + pmin)
    {
      int old_c=c;
      while(c>old_c-10 && c>0) //permetto al massimo uno shift di 10 pixel
      {
        if(modelx[c-1].mean==mmean-1) c--;
        else break;
      }
    }
  }
  else
  {
    for(int e=0;e<t;e++)
    {
      if(modelx[e].mean>=mmean)
      {
        if(modelx[e].mean==mmean) uguali++;//conto quanti ne trovo di uguali
        else uguali=0; //per assegnare il centrale
        mmean=modelx[e].mean;
        c=e;
      }
    }
  }

  c=c-uguali/2; // perche uguali/2 ???
  int m=0;
  while(max_x[m].mean!=0)m++;

  max_x[m]=modelx[c];

  int cpiu=c+1;
  int cmen=c-1;

  while(abs(modelx[cpiu].pos-modelx[c].pos)<modelx[c].wid+modelx[cpiu].wid)
  {
    if(cpiu<t)cpiu++;
    else break;
  }
  while(abs(modelx[c].pos-modelx[cmen].pos)<modelx[c].wid+modelx[cmen].wid)
  {
    if(cmen>=0)cmen--;
    else break;
  }
  for(int p=cmen+1;p<cpiu;p++)
  {
    modelx[p].mean=0;
  }
}



/*! 
\brief Ricerca i massimi locali all'interno delle proiezioni multiple sull'asse Y per un certo massimo locale sull'asse X.

Analizzando il vettore modely (che contiene i potenziali blob dal punto di vista dell'asse Y) 
riesco ad estrarre i massimi locali (per ogni massimo sull'asse X) e metterli dentro al vettore max_y.

\param fun_mode NON VIENE USATA ???
\param nump Indice del massimo locale sull'asse delle X.
\param q Number of modelY found

*/
void maxsearch_y(unsigned char fun_mode, int nump, int q)
{
  unsigned char c=0;
  int mmean=20;
  int uguali=0;
  for(int p=0;p<q;p++)
  {
    if(modely[p].mean>=mmean)
    {
      if(modely[p].mean==mmean) uguali++;//conto quanti ne trovo di uguali
      else uguali=0; //per assegnare il centrale
      mmean=modely[p].mean;
      c=p;
    }
  }
  c=c-uguali/2;
  int m=0;
  while(max_y[nump][m].mean!=0)m++;

  max_y[nump][m]=modely[c];

  int cpiu=c+1;
  int cmen=c-1;

  while(abs(modely[cpiu].pos-modely[c].pos)<(modely[c].wid+modely[cpiu].wid))
  {
    if(cpiu<q)cpiu++;
    else break;
  }
  while(abs(modely[c].pos-modely[cmen].pos)<(modely[c].wid+modely[cmen].wid))
  {
    if(cmen>=0)cmen--;
    else break;
  }
  for(int p=cmen+1;p<cpiu;p++)
  {
    modely[p].mean=0;
  }
}
#endif


void InitPers(tPersonDetected* persone)
{
  for(int pp=num_pers-1; pp>=0;pp--)
  {
    persone[pp].x=0;
    persone[pp].y=0;
    persone[pp].wx=0;
    persone[pp].wy=0;
    persone[pp].h=0;
    persone[pp].fusa=false;
    if(pp<10)
    {
      max_x[pp].mean=0;
      max_x[pp].dev=0;
      max_x[pp].wid=0;
      max_x[pp].pos=0;
    }
  }
}


/*!
\brief Compute the real X coordinate on the floor in wideconfiguration only.

This information is only used in wideconfiguration and it is used by
Merge_and_Filter_people() to understand if the same person is seen from
two different sensor in order to avoid to count that person twice.
*/
void CalculateReal_x(tPersonDetected* persone)
{
  if(inst_height==0 || inst_dist==0)
  {
    return;
  }

  //  printf("calculate real_x \n");
  static const int NUM_DISP = 16; // numero di disparita'
  //static int Tab_Altezza_Disparita225[NUM_DISP] = {225-225/*???*/,225-225/*???*/,225-225,225-225,225-125,225-105,225-90,225-80,225-70,225-60,225-55,225-50,225-42,225-35,225-32/*???*/,225-30/*???*/};
  //static int Tab_Altezza_Disparita240[NUM_DISP] = {240-240/*???*/,240-240/*???*/,240-240,240-240,240-150,240-125,240-105,240-90,240-80,240-70,240-60,240-55,240-50,240-42,240-35/*???*/,240-32/*???*/};
  static int Tab_Altezza_Disparita[NUM_DISP];  // Tabella che collega le altezze alla disparita' in base all'altezza d'installazione
  static bool first_time = true;  // per assicurami che entra la prima  volta nel ciclo
  static int prev_inst_height = inst_height;  // installazione al ciclo precedente                            
  static const int Tab_Distanze225[NUM_DISP] = {225,225,225,225,125,105,90,80,70,60,55,50,42,35,32,30};  // tabelle delle distanze a 225 cm
  static const int Tab_Distanze240[NUM_DISP] = {240,240,240,240,150,125,105,90,80,70,60,55,50,42,35,32};  // tabelle delle  distanze a 240 cm

  //int correction_height; // 20130522 eVS bugfix
  if ( first_time || prev_inst_height != inst_height)
  {
    first_time = false;
    if(det_area==0)
    {
      //correction_height=(inst_height >= 225) ? (inst_height-225) : 0; // 20130522 eVS bugfix
      //for(int i=0;i<16;i++) // 20130522 eVS bugfix
      //  Tab_Altezza_Disparita[i]=Tab_Altezza_Disparita225[i]+correction_height;// 20130522 eVS bugfix
      Tab_Altezza_Disparita[NUM_DISP - 1] = (inst_height > 225) ? (inst_height - Tab_Distanze225[NUM_DISP - 1]) : (225 - Tab_Distanze225[NUM_DISP - 1]); 

      for ( int i = NUM_DISP - 2; i >= 0; --i)
        Tab_Altezza_Disparita[i] = inst_height - Tab_Distanze225[i];

    }
    else
      if(det_area==1)
      {
        //correction_height=inst_height-240;
        //for(int i=0;i<16;i++)
        //Tab_Altezza_Disparita[i]=Tab_Altezza_Disparita240[i]+correction_height;// 
        // 20130522 eVS modified
        for(int i= NUM_DISP - 1; i>= 0; --i)
          Tab_Altezza_Disparita[i]= inst_height - Tab_Distanze240[i];
      }

      else
      {
        return;
      }
  }

  prev_inst_height = inst_height;  // salvo la precedente altezza installazione per decidere se fare o meno il calcolo dei vettori

  int dx=0;
  int x;
  int Lenght_at_floor_level = int(1.399*inst_height);

  for(int i=0;i<num_pers;i++)
  {
    if(persone[i].h !=0)
    {

      // nella formula che segue persone[i].h è la disparità a 7 bit e deve essere moltiplicata 
      // per 2 (è perché durante la fase di proiezione vengono calcolati valori a 7bit dividendo 
      // per 2, si cerchi "val7bit")
      float index_fl = float((persone[i].h*2)/16.0 + 0.5);
      int index = int(index_fl);
      if(index>15)
        index=0;   // 20130522 eVS ????

      dx = ((NX/2-persone[i].x)*Tab_Altezza_Disparita[index])/inst_height;
      x = persone[i].x+(NX*inst_dist/Lenght_at_floor_level)*(total_sys_number-persone[i].sys)+dx;
      if(x>160*total_sys_number || x<0) 
      {
        x=0;
        persone[i].h=0;
      }
      persone[i].real_x=x;
    }
  }
}


/*!
\brief Used in widegate in order to avoid multiple counts of people in the shared field of view.
\param persone Array of all the tracked people.
\param num_pers Length of the array.
*/
void Merge_and_Filter_people(tPersonDetected* persone, int &num_pers)
{
  int window_x,window_y;
  //static const int NUM_DISP = 16; // numero di disparita'
  //static int Tab_Altezza_Disparita[NUM_DISP];  // Tabella che collega le altezze alla disparita' in base all'altezza d'installazione

  // parametri utilizzati a 225 normalizzati rispetto Install_Height
  // Spiegazione dei valori di default_value e offset
  // Questi valori sono il risultato di un'approfondita analisi relativa al dimensionamento delle soglie sottostanti in base all'altezza di installazione e
  // alla distanza d'installazione tra due dispositivi, in quanto le soglie erano state trimmate per la seguente configurazione inst_height: 225cm e inst_dist: 60cm 
  // Per l'altezza si moltiplica per il valore (inst_height / 255) mentre si aggiunge alla soglia il valore di offset ( vedi formula) per considerare l'a distanza di installazione
  float fact = 1.399f *225.0f;
  int default_value = int((NX*60)/fact); // e' 30 di default 
  int offset =   default_value - int((NX*inst_dist)/fact);  // valore - 30;
  float fact_height = float(inst_height/225.0f);
  // soglie di base
  static int th_1_default = int(NX/3);
  static int th_2_default = 85;
  static int th_3_default = 30;
  // soglie in base a cambiamenti di altezza e distanza dei dispositivi in installazione
  int th_1 = int((th_1_default + offset) * fact_height);
  int th_2 = int((th_2_default + offset) * fact_height);
  int th_3 = int((th_3_default + offset) * fact_height);
  static int th_h = 120;

  window_x = 8; //6-8 grandezza dell'intorno su cui considerare la coordinata x
  int fact_offset = 4; // fattore per aumentare l'intorno da considerare
  for(int i=0;i<num_pers;i++)
  {
    if(persone[i].h !=0)
    {
      window_y = persone[i].wy*2;
      for(int u=0;u<num_pers;u++)
      {
        if((persone[u].h !=0)&&(persone[u].sys < (persone[i].sys))) // per guardare solo i PCN precendenti CONDIZIONE 2
          if(persone[u].x < th_1)  // Apparso per almeno  1/3 del frame
            ///Minore di 4/5 e ??) oppure ( altezza per camera pos meglio?) oppure (altezza minore di quella dell camera dopo) oppure (altezza uguale e area minore) 
          {
            if(((persone[u].h <= 4*persone[i].h/5) && (persone[u].x +NX- persone[i].x>th_2))  ||
              (persone[u].h <th_h)  ||
              ((persone[u].h <= persone[i].h) && (persone[u].x +NX- persone[i].x<=th_2)) ||
              ((persone[u].h == persone[i].h) && ((persone[u].wy*persone[u].wx)<(persone[i].wy*persone[i].wx)))) //<-new
              if(persone[u].h <= persone[i].h)
                if((persone[u].real_x<(persone[i].real_x+3*window_x))&&(persone[u].real_x>(persone[i].real_x-window_x)))
                  if((persone[u].y<(persone[i].y+window_y))&&(persone[u].y>(persone[i].y-window_y)))
                    //if(persone[u].ho_canc_su_sys==0)    // 20130517 eVS DO NOT ADD THIS CODE : sembra che porti a conteggi doppi
                    {
                      //printf("\n Ho fuso la persona del sistema %d\n",persone[u].sys);
                      persone[u].h_se_cancellato=persone[u].h;
                      persone[u].h=0;
                      persone[u].stato_canc_da=persone[i].sys;
                      persone[i].ho_canc_su_sys=persone[u].sys;
                      persone[i].x=(persone[i].x+persone[u].x)/2;
                      persone[i].y=(persone[i].y+persone[u].y)/2;
                      persone[i].real_x=(persone[i].real_x+persone[u].real_x)/2;
                    }
          }
      }
    }
  }

  for(int i=0;i<num_pers;i++)
  {
    if(persone[i].h !=0)
    {
      window_y=persone[i].wy*2;
      for(int u=0;u<num_pers;u++)
      {
        if((persone[u].h !=0) && (persone[u].sys > persone[i].sys))  // per guardare solo i PCN successivi
          if(persone[u].x > (NX-th_1))
            if(((persone[u].h <= 4*persone[i].h/5) && (NX-persone[u].x + persone[i].x>th_2))  ||
              (persone[u].h<th_h)  ||
              ((persone[u].h <= persone[i].h) &&  (NX-persone[u].x + persone[i].x<=th_2)) ||
              ((persone[u].h <= persone[i].h) && ((persone[u].wy*persone[u].wx)<(persone[i].wy*persone[i].wx))))
              if(persone[u].h <= persone[i].h) 
                if((persone[u].real_x<(persone[i].real_x+window_x))&&(persone[u].real_x>(persone[i].real_x-3*window_x)))
                  if((persone[u].y<(persone[i].y+window_y))&&(persone[u].y>(persone[i].y-window_y)))
                    //if(persone[u].ho_canc_su_sys==0)  // 20130517 eVS DO NOT ADD THIS CODE : sembra che porti a conteggi doppi
                    {
                      //printf("\n persone[u].ho_canc_su_sys %d\n",(persone[u].ho_canc_su_sys!=0) ? persone[u].ho_canc_su_sys : 0);
                      persone[u].h_se_cancellato=persone[u].h;
                      persone[u].h=0;
                      //printf("\n Ho fuso la persona del sistema %d\n",persone[u].sys);
                      persone[u].stato_canc_da=persone[i].sys;
                      persone[i].ho_canc_su_sys=persone[u].sys;
                      persone[i].x=(persone[i].x+persone[u].x)/2;
                      persone[i].y=(persone[i].y+persone[u].y)/2;
                      persone[i].real_x=(persone[i].real_x+persone[u].real_x)/2;
                    }
      }
    }
  }

  // Eliminazione delle croci sull'area di sovrapposizione, vicino al bordo inferiore e superiore.
  // Se fossero da tenere dovrei vedere qualcosa anche con l'altra telecamera e quindi sarebbero fuse.
  // Quello che tolgo sono persone che ci sono, sono nell'area che sto'valutando e non vengono da fusione
  // precedente (MA COSA VUOL DIRE ???)
  for(int i=0;i<num_pers;i++)
  {
    if(persone[i].h !=0 && (persone[i].y < 25 || persone[i].y> NY-25) && (persone[i].sys!=1))
    {  //							perche 25 ??? cosa significa ???
      if((persone[i].x>NX-th_3))
      {
        persone[i].h = 0;
      }
    }
    if(persone[i].h !=0 && (persone[i].y < 25 || persone[i].y> NY-25) &&(persone[i].sys!=current_sys_number))
    {  //							perche 25 ??? cosa significa ???
      if(persone[i].x<th_3)
      {
        persone[i].h = 0;
      }
    }
  }

}



/*!
\brief Once people are detected (at most 10 otherwise error arises) they are copied in a vector persdata
then used in widegate to be sent to the master PCN (see "persdetwidegate" in serial_port.cpp).
*/
void WritePersDetected(tPersonDetected* persone, const int num_pers_det)
{
  memset(persdata,0,sizeof(persdata));
  if(num_pers_det>10) 
  {
    printf("Error in WritePersDet num_pers troppo grande! %d\n",num_pers_det);
    return;
  }
  int ind_data=4;
  persdata[0]=250;
  if(ev_door_close==false) persdata[1]=250;
  else 
  {
    persdata[1]=251;
  }
  if(ev_door_open==false) persdata[1]=250;
  else
  {
    persdata[1]=252;
  }
  persdata[2]=current_sys_number;
  persdata[3]=count_sincro;
  for(int i=0;i<num_pers_det;i++)
  {
    if(persone[i].h!=0)
    {
      persdata[ind_data]=persone[i].h;
      persdata[ind_data+1]=persone[i].x;
      persdata[ind_data+2]=persone[i].y;
      persdata[ind_data+3]=persone[i].wx;
      persdata[ind_data+4]=persone[i].wy;
      ind_data+=5;
    }
  }

}


