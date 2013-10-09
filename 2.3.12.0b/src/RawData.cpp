/*!
\file RawData.cpp

\brief Contiene l'implementazione  della funzioni della classe RawData.

In questo file &egrave; contenuta l'implementazione delle funzioni definite nel file RawData.h:
1)- Lettura del file RAW
2)- Inizializzazione membri della classe
Definisco il costruttore che gestisce la possibilita' di leggere tutto il file oppure solo una sotto-sequenza.
Viene lasciata la possibilita' di scegliere se bufferizzare tutto in memoria (leggendo tutto il file nel costruttore)
non rendendo più necessario accedere al file oppure se non leggere nulla e accedere opportunamente al file
per ottenere le informazioni richieste dai vari metodi.

\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#include "RawData.h"

const int DIM_COLUNMS = 160;   //!< Numero di colonne del frame 
const int DIM_ROWS = 120;     //!< Numero di righe del frame 
const int DIM_FRAME = (DIM_ROWS*DIM_COLUNMS); //!< Numero totale di pixel dell'immagine sottocampionata (#NX*#NY=160*120) 

long _getOrigNumFrames(const char *i_file_src, const int i_file_elem_sz);



/*!
Costruttore RawData vuoto.
*/
RawData::RawData()
{
// Inizializzo i campi a valori significativi per indicare un oggetto vuoto
  disparityMap = NULL;
  image = NULL;
  input0 = NULL;
  input1 = NULL;
  k_file_name = NULL;

  num_frames  = UINT_MAX;
  last_frame  = UINT_MAX;
  first_frame = UINT_MAX;

  is_door_status_present = true;
  img_presence_flag = INT_MAX;
}
/*!
Costruttore di RawData che riceve come parametri: il nome del file RAW da caricare,
l'informazione sull'utilizzo della memoria, flag che dice se l'immagine ottica e' presente,
flag che si se i digital input sono presenti, l'indice del primo e dell'ultimo frame da leggere.

Si puo' leggere sia una sottosequenza del video sia tutto il filmato.

Se il valore #i_is_buffering_active &egrave; positivo allora il costrutture richiama la funzione privata #setDataRaw() che
procede con la lettura del file RAW che viene trasferito in memoria. 

\param *i_file_src		          [in] Nome del file RAW da leggere
\param i_is_buffering_active	  [in] Flag per la possibilit&agrave; di salvare in memoria tutto il file (true) o solo un frame (false)
\param i_img_presence_flag	    [in] Flag per specificare informazioni sul formato del file (se e' presente o meno l'immagine ottica e dove si trova nel file)
\param i_is_door_status_present	[in] Flag per il door status
\param i_f_frame			          [in] Indice del primo frame da leggere
\param i_l_frame			          [in] Indice dell'ultimo frame da leggere (se pari a UINT_MAX allora devo leggere fino all'ultimo fotogramma)
*/

RawData::RawData(const char *i_file_src, bool i_is_buffering_active, int i_img_presence_flag, bool i_is_door_status_present, unsigned int i_f_frame, unsigned int i_l_frame)
{	
  // Inizializza i campi della classe
  disparityMap = NULL;
  image = NULL;
  input0 = NULL;
  input1 = NULL;
  k_file_name = NULL;

  num_frames  = 0;
  last_frame  = 0;
  first_frame = 0;

  is_door_status_present = i_is_door_status_present;
  img_presence_flag = i_img_presence_flag;

  long n_orig_frames = _getOrigNumFrames(i_file_src, getFileElemSz());
  if (n_orig_frames > 0) // se il file esiste ed ha almeno un elemento
  {
    assert(i_f_frame >= 0); //!< L'indice del primo frame deve essere maggiore di 0.
    assert(i_l_frame >= 0); //!< L'indice dell'ultimo frame deve essere maggiore di 0.
    assert(i_f_frame <= i_l_frame); //!< L'indice del primo frame deve essere minore stretto dell'indice dell'ultimo frame.
    assert(i_l_frame <= n_orig_frames-1 || i_l_frame == UINT_MAX);//!< L'indice dell'ultimo frame deve essere minore o uguale dell'indice del numero del frame.

    if (i_l_frame == UINT_MAX)
      i_l_frame = n_orig_frames-1;

    if (i_l_frame <= n_orig_frames-1)
    {
      int sz = strlen(i_file_src)+1; // attenzione all'ultimo carattere \0
      k_file_name = (char*) malloc (sz); // alloca memoria per k_file_name
      memcpy(k_file_name, i_file_src, sz); // computazionalmente piu veloce

      num_frames	=	i_l_frame-i_f_frame+1;
      last_frame	= i_l_frame;
      first_frame = i_f_frame;

      // Se memory_info vale true allora voglio leggere tutto il file e tenerlo in memoria
      if (i_is_buffering_active)
        setDataRaw();
    }
  }
}


/*!
\return Nome del file RAW che stiamo leggendo.
*/
const char* RawData::getFileName()
{ 
  return k_file_name;
}

/*!
\return Numero di frame considerati dall'oggetto.
*/

unsigned int RawData::getNumFrames()
{ 
  return num_frames;
}

/*!
\return Indice del primo frame considerato nel file RAW.
*/
unsigned int RawData::getFirstFrameIdxInFile()
{ 
  return first_frame;
}

/*!
\return Indice dell'ultimo frame considerato nel file RAW.
*/
unsigned int RawData::getLastFrameIdxInFile()
{ 
  return last_frame;
}


/*!
\param i_index_frame [in] Indice del frame in corrispondenza del quale si vuole ottenere il digital input idx.
\param idx           [in] Specifica a quale digital input si e' interessati (0 o 1).
\return Valore digital input[idx] al frame i_index_frame.
*/
unsigned char RawData::getInput(const int i_index_frame, const int idx)
{
  assert(i_index_frame <= num_frames-1); // Se il valore di i_index_frame e minore dell'indice dell'ultimo frame da caricare procedo
  assert(i_index_frame >= 0);// e il valore di i_index_frame e maggiore o uguale a 0 procedo
  assert(idx == 0 || idx == 1);

  if ((input0 && idx == 0) || (input1 && idx == 1))
    return ((idx == 0) ? input0[i_index_frame] : input1[i_index_frame]); 
  else
  {
    if (is_door_status_present)
    {
      FILE * fp = fopen (k_file_name, "rb" );

      unsigned char out;
      fseek (fp, ((first_frame+i_index_frame)*getFileElemSz()) + DIM_FRAME + (img_presence_flag == RD_IMG_BEFORE_INPUT ? DIM_FRAME : 0) + idx, SEEK_SET);
      fread(&out,1,1,fp);
      fclose (fp); 

      return out; 
    }
    else
      return 1;
  }
}


/*!
\param i_index_frame [in] Indice del frame in corrispondenza del quale si vuole ottenere il digital input0.
\return Valore digital input0 al frame i_index_frame
*/
unsigned char RawData::getInput0(int i_index_frame)
{ 
  return getInput(i_index_frame, 0);
}


/*!
\param i_index_frame [in] Indice del frame in corrispondenza del quale si vuole ottenere il digital input1.
\return Valore digital input1 al frame i_index_frame
*/
unsigned char RawData::getInput1(int i_index_frame)
{ 
  return getInput(i_index_frame, 1);
}


/*!
\param i_index_frame [in] Rappresenta l'indice frame da restituire
\return Frame della mappa di disparit&agrave;  di indice i_index_frame
*/
unsigned char *RawData::getDisparityMap(int i_index_frame)
{  
  assert(i_index_frame <= num_frames-1); // Se il valore di i_index_frame e minore dell'indice dell'ultimo frame da caricare procedo
  assert(i_index_frame >= 0);// e il valore di i_index_frame e maggiore o uguale a 0 procedo

  unsigned char* o_frame = (unsigned char*) malloc(DIM_FRAME*sizeof(unsigned char));//!<Contiene la mappa di disparit&agrave da restituire 
  //unsigned char* digital_input =  (unsigned char*) malloc(sizeof(unsigned char));

  // Controllo il valore di memory info
  if(disparityMap) // verifico se i dati sono stati letto nel costruttore
  {
    memcpy(o_frame, disparityMap[i_index_frame], DIM_FRAME); // Restituisco direttamente il frame di indice i_index_frame
  }
  else
  { 
    FILE * fp = fopen (k_file_name, "rb" );

    // creo un vettore temporaneo per il frame da restituire 
    fseek (fp, (first_frame+i_index_frame)*getFileElemSz(), SEEK_SET);
    fread(o_frame,1,DIM_FRAME,fp);//leggo  il frame cercato

    fclose (fp); //Chiudo il file
  }

  return o_frame;
}

/*!
\param i_index_frame	[in]  Rappresenta l'indice frame da restituire
\return Frame della mappa di disparit&agrave;  di indice i_index_frame
*/
unsigned char* RawData::getImage(int i_index_frame)
{  
  assert(i_index_frame <= num_frames-1); // Se il valore di i_index_frame e minore dell'indice dell'ultimo frame da caricare procedo
  assert(i_index_frame >= 0);// e il valore di i_index_frame e maggiore o uguale a 0 procedo

  if (img_presence_flag == RD_IMG_NOT_PRESENT || img_presence_flag == RD_ONLY_DSP_AT_54FPS)
    return NULL;
  else
  {
    unsigned char* img_frame = (unsigned char*) malloc(DIM_FRAME*sizeof(unsigned char));//!<Contiene la mappa di disparit&agrave da restituire 

    if(image) // verifico se i dati sono stati letto nel costruttore
    {
      memcpy(img_frame, image[i_index_frame], DIM_FRAME); // Restituisco direttamente il frame di indice i_index_frame
    }
    else
    { 
      FILE * fp = fopen (k_file_name, "rb" );

      fseek (fp, ((first_frame+i_index_frame)*getFileElemSz()) + DIM_FRAME + (img_presence_flag == RD_IMG_BEFORE_INPUT ? 0 : (is_door_status_present ? 2 : 0)), SEEK_SET);
      fread(img_frame,1,DIM_FRAME,fp);//leggo  il frame cercato

      fclose (fp); //Chiudo il file    
    }

    return img_frame;
  }
}

/*!
Nel caso il costruttore fosse stato eseguito con buffering attivo, il distruttore
dovra' deallocate opportunamente i dati in memoria.
*/
RawData::~RawData()
{
  if (k_file_name)
    free(k_file_name);

  if (disparityMap)
  {
    for (int i=0; i<num_frames; ++i) // Elimino la mappa di disparit&agrave
      if (disparityMap[i])
        free(disparityMap[i]);  
    free(disparityMap);
  }

  if (image)
  {
    for (int i=0; i<num_frames; ++i) // Elimino l'immagine ottica
      if (image[i])
        free(image[i]);  
    free(image);
  }

  if (input0)
    free(input0);

  if (input1)
    free(input1);
}


void
_read_file_elem(FILE* fp, const int idx, const int file_elem_sz, const int img_presence_flag, const bool is_door_status_present, unsigned char* const & disparityMap, unsigned char* const & image, unsigned char & input0, unsigned char & input1)
{
  fseek (fp, idx*file_elem_sz, SEEK_SET);
  fread(disparityMap, 1, DIM_FRAME, fp);

  if (img_presence_flag == RD_IMG_BEFORE_INPUT)
    fread(image, 1, DIM_FRAME, fp);

  if (is_door_status_present)
  {
    fread(&(input0), 1, 1, fp);
    fread(&(input1), 1, 1, fp);
  }

  if (img_presence_flag == RD_IMG_AFTER_INPUT || img_presence_flag == RD_ONLY_DSP_AT_54FPS)
    fread(image, 1, DIM_FRAME, fp);
}


/*!
Questa funzione effettua la lettura del file RAW passato precedentemente come parametro al costruttore.
Legge il file  per ogni un numero di byte corrispondente alla dimensione del frame 
e salva l'intera mappa di disparit&agrave; contenuta in memoria.
Il metodo &egrave; richiamato solamente nel caso in cui il campo memory_input &egrave; true. 
Questo vuol dire che si &egrave; deciso di allocare la memoria necessaria per mantenere tutto il file
e leggere tutta la mappa di disparit&agrave;.
*/
int RawData::setDataRaw()
{
  int ret = 0;

  FILE* fp = fopen (k_file_name, "rb" );

  // Leggo il file
  if (fp == NULL) 
  {
    printf("ERRORE: File non trovato\n"); 
    ret = RD_ERR_NO_FILE_FOUND;
  }
  else
  {
    // TODO
    printf ("Testare allocazione e deallocazione in quanto sembra esserci bug nel free!\n");

    // Alloco la memoria necessaria:
    //per la mappa di disparit&agrave;
    disparityMap = (unsigned char**) malloc(num_frames*sizeof(unsigned char*));// per ogni frame della la mappa di disparit&agrave;
    for (int i=0; i<num_frames; ++i) // Per la dimensione del frame
      disparityMap[i] = (unsigned char*) malloc(DIM_FRAME*sizeof(unsigned char));

    // per l'immagine ottica
    if (img_presence_flag != RD_IMG_NOT_PRESENT)
    {
      image = (unsigned char**) malloc(num_frames*sizeof(unsigned char*));// per ogni frame della la mappa di disparit&agrave;
      for (int i=0; i<num_frames; ++i) // Per la dimensione del frame
        image[i] = (unsigned char*) malloc(DIM_FRAME*sizeof(unsigned char));
    }

    //per il digital_input
    if (is_door_status_present)
    {
      input0 = (unsigned char*) malloc(num_frames*sizeof(unsigned char));// per digital input0
      input1 = (unsigned char*) malloc(num_frames*sizeof(unsigned char));// per digital input1
    }

    unsigned int idx = first_frame;
    unsigned int internal_idx = 0;

    // Mi posizione sul file
    while (idx <= last_frame) 
    {
      _read_file_elem(fp, idx, getFileElemSz(), img_presence_flag, is_door_status_present, disparityMap[internal_idx], image[internal_idx], input0[internal_idx], input1[internal_idx]);

      ++idx;
      ++internal_idx;
    }

    fclose (fp); // Chiudo il file
  }

  return ret;
}

/*! 
In base alla lunghezza del file RAW da leggere e della dimensione del frame conto il numero di frame della
sequenza. 
Questa informazione mi permette di verificare la bonta dell'input nel costruttore che riceve in input gli
indici della sottosequenza da analizzare.

\param i_file_src [in] Vettore che contiene il nome del file RAW da leggere.
\param i_file_elem_sz [in] Intero che indica la dimensioni del frame.
*/
long _getOrigNumFrames(const char *i_file_src, const int i_file_elem_sz)
{
  long ret = 0;

  // apre il file in lettura e controlla che il file esista
  FILE* fp = fopen (i_file_src, "rb" );

  if (fp == NULL) {
    printf("ERRORE: File non trovato\n"); 
    ret = RD_ERR_NO_FILE_FOUND;
  }
  else
  {
    // Mi posizione all'inizio del file
    fseek (fp, 0 , SEEK_END);
    unsigned long length_file = ftell (fp);// ottengo la lunghezza del file

    // Calcolo l'ultimo frame se non c'e l'immagine ottica moltiplicare per 2!
    ret = length_file/(i_file_elem_sz);

    fclose(fp);// chiudo il file
  }

  return ret;
}


/*! 
\return Numero di colonne del frame

*/
int RawData::getNumCols()
{
  return DIM_COLUNMS;
}


/*! 
\return Numero di righe del frame
*/
int RawData::getNumRows()
{
  return DIM_ROWS;
}


/*! 
\return Dimensione elemento del file.
*/
int RawData::getFileElemSz()
{
  int ret = (DIM_FRAME + ((img_presence_flag==RD_IMG_AFTER_INPUT || img_presence_flag==RD_IMG_BEFORE_INPUT) ? DIM_FRAME : 0) + 2*(is_door_status_present ? 1 : 0));
  return ret;
}


/*! 
\return true se esiste il digital input false altrimenti.
*/
bool RawData::isInputPresent()
{
  return (input0 != NULL);
}


/*! 
\return true se esiste l'immagine ottica false altrimenti.
*/
bool RawData::isImagePresent()
{
  return (image != NULL);
}


/*! 
\return true se il costruttore ha avuto successo.
*/
bool RawData::isRawDataInitialized()
{
  return (k_file_name!=NULL);
}