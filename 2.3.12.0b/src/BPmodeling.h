/*!
\file BPmodeling.h
\brief Contiene l'header per la definizione della classe BPmodeling

In questo file si definiscono le funzioni  implementate nel file BPmodeling.cpp

\author Omar Zandon&agrave;, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/


#ifndef BPMODELING_H
#define BPMODELING_H

class BPmodeling  // black pixels modeling
{
private:
  unsigned short *BP;  ///< Maschera dei pixel neri
  int height;  ///< Dimensione Y della maschera
  int width;  ///< Dimensione X della maschera
  int dim;  ///< Dimensione della maschera
  int border_x;  ///< Bordo delle X ( Si usa la maschera binnata di conseguenza questi valori sono settati a 0)
  int border_y;  ///< Bordo delle Y


public:
  int num_mask_pixel;  ///< Numero di pixel neri trovati dal modello per ogni frame

  BPmodeling();  ///< Costruttore di default.
  BPmodeling(const int i_w, const int i_h, const int i_bx, const int i_by);  ///< Costruttore della classe BPmodeling.
  ~BPmodeling();  ///< Distruttore.
 
  void Reset();  ///> resetta il modello dei pixel neri
  void UpdateModel(const unsigned char* const & i_bp_mask, const int i_width, const int i_height);  ///< Aggiorna il modello
  void GetMask(unsigned char* const & o_BPbw, const int i_width, const int i_height);   //<Restituisce il modello finale
  const unsigned short* GetBP(int & o_BPwidth, int & o_BPheight);  //Restituisce la maschera che contiene i contatori di ogni pixel




};

#endif
