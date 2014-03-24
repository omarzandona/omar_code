/*!
\file RawData.h
\brief Contiene l'header per la definizione della classe RawData

In questo file definiamo i campi della nostra classe e la definizione delle funzioni
poi implementate nel file RawData.cpp

\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#ifndef __RAW_DATA__
#define __RAW_DATA__

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

enum RD_ERR_CODES {
  RD_OK = 0,
  RD_ERR_NO_FILE_FOUND = -1
};

enum RD_IMG_PLACE {
  RD_IMG_NOT_PRESENT = 0, //!< Used when the read sequence contains only disparity maps
  RD_IMG_BEFORE_INPUT = 1, //!< Used when the read sequence contains disparity maps and the left image placed BEFORE digital input
  RD_IMG_AFTER_INPUT = 2, //!< Used when the read sequence contains disparity maps and the left image placed AFTER digital input
  RD_ONLY_DSP_AT_54FPS = 3 //!< Used when the read sequence contains only disparity maps and digital inputs (more precisely there are images but they have no meaning)
};

/*!
  \brief Definisce la struttura necessaria per la lettura da un file RAW.

  Gestisco i vari campi di cui ho bisogno per la gestione del file RAW.
*/
class RawData 
{
public:
  RawData();//!< Costruttore di default. Setto i parametri per la creazione di un oggetto RawData vuoto.
  RawData(const char *i_file_src, bool i_is_buffering_active = false, int i_img_presence_flag = RD_IMG_AFTER_INPUT, bool i_is_door_status_present = true, unsigned int i_f_frame = 0, unsigned int i_l_frame = UINT_MAX); //!< Costruttore. 
  ~RawData(); //!< Distuttore.
  unsigned int getNumFrames(); //!<  Restituisco il numero di frame considerati dall'oggetto ovvero (#getLastFrameIdxInFile()-#getFirstFrameIdxInFile())+1.
  unsigned int getFirstFrameIdxInFile(); //!<  Restituisco l'indice che il primo frame dell'oggetto ha nel file RAW da cui proviene.
  unsigned int getLastFrameIdxInFile(); //!<  Restituisco l'indice che l'ultimo frame dell'oggetto ha nel file RAW da cui proviene.
  const char* getFileName(); //!< Restituisco il nome del file.
  unsigned char *getDisparityMap(int i_index_frame); //!<  Restituisco la mappa di disparit&agrave; di indice i_index_frame da 0 a #getNumFrames()-1.
  unsigned char *getImage(int i_index_frame); //!< Restituisco l'immagine ottica del frame di indice i_index_frame da 0 a #getNumFrames()-1.
  unsigned char getInput0(int i_index_frame); //!< Restituisco digital input0 di indice i_index_frame da 0 a #getNumFrames()-1.
  unsigned char getInput1(int i_index_frame); //!< Restituisco digital input1 di indice i_index_frame da 0 a #getNumFrames()-1.
  unsigned char getInput(const int i_index_frame, const int idx); //!< Restituisco digital input0 (o input1 a seconda di idx uguale a 0 o 1) di indice i_index_frame da 0 a #getNumFrames()-1.
  int getNumCols(); //!< Restituisce il numero di colonne dei frame.
  int getNumRows(); //!< Restituisce il numero di righe dei frame.
  bool isInputPresent(); //!< Restituisce true se il digital input era presente nel file.
  bool isImagePresent(); //!< Restituisce true se l'immagine ottica era presente nel file.
  bool isRawDataInitialized(); //!< Restituisce true se il costruttore e' andato a buon fine.

private:
  char *k_file_name;//!< Nome del file RAW da leggere.
  bool is_door_status_present;//!< Questa flag e' a true quando ogni elemento nel file contiene anche i valori del digital input.
  int img_presence_flag;//!< Se questa flag e' #RD_IMG_NOT_PRESENT allora ogni elemento nel file ha solo la mappa di disparita' e digital input, altrimenti se uguale a #RD_IMG_BEFORE_INPUT allora l'immagine ottica e' tra la mappa e il digital input se uguale a #RD_IMG_AFTER_INPUT o a #RD_ONLY_DSP_AT_54FPS allora e' dopo ai digital input.
  unsigned int num_frames;//!< Numero frame di interesse cioe' #last_frame-#first_frame+1.
  unsigned int first_frame;//!< Indice del primo frame ( Default 1).
  unsigned int last_frame;//!< Indice dell'ultimo frame ( Default -1 verr&agrave; calcolato in base alla lunghezza del file).

  unsigned char **disparityMap; //!< Se memory_info a true vettore per le Mappe di disparit&agrave; altrimenti NULL.
  unsigned char **image; //!< Se memory_info a true vettore per le immagini ottiche altrimenti NULL.
  unsigned char *input0;//!< Se memory_info a true contiene le informazioni sul digital input0 altrimenti NULL.
  unsigned char *input1;//!< Se memory_info a true contiene le informazioni sul digital input1 altrimenti NULL.

  int setDataRaw(); //!< Inizializzo tutti i campi e creo la Mappa di disparit&agrave; se memory_info vale true.
  int getFileElemSz(); //!< Restituisce la dimensione in byte di un elemento del file che dipende da #is_door_status_present e da #img_presence_flag.
};
#endif