/*!
\file peopletrack.cpp
\brief Contiene l'implementazione dell'algoritmo di tracking delle persone.

Per i dettagli si veda la descrizione dettagliata della funzione track().

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/
#include "directives.h"
#ifndef USE_NEW_TRACKING

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "peopletrack.h"

#include "peopledetection.h"

//#define debug_

#ifdef PCN_VERSION
#define DIST_MAX 18 //!< Maximum displacement of a person between consecutive frames
#else
#define DIST_MAX 27 //!< Maximum displacement of a person between consecutive frames
#endif

#define DIST_INC 2  //!< Displacement increment when tracking looses a person for a frame

/*!
\var inhi
\brief List of the tracked persons entered in the field of view from the top of the image.

    If the "dir" parameter in #parm_values is set to 0 (incoming people go from the top towards the 
    bottom of the image), the persons in this list are potentially ENTERING the monitored gate).

    Notice that, 
    - since all the detectable persons (at most #NUM_PERS_SING) could appear from the top
      of the image (it is unprobable but it could be) the length of this list can be at most
      equal to #NUM_PERS_SING
    - more precisely, the maximum length of this list depends on how the PCN works: alone or in
    widegate. If it works alone, the maximum length of this list is equal to #NUM_PERS_SING
      whereas if it works with others the dimension will be #NUM_PERS_SING multiplyed by the number
    of system working together, i.e. #total_sys_number.
    
    This consideration is used to properly allocate memory for this data.
    
    \see inlo
*/
tPersonTracked ** inhi = NULL;

/*!
\var inlo
\brief List of the tracked persons entered in the field of view from the bottom of the image.
       
This data structure is very similar to #inhi but in this case if the "dir" parameter in #parm_values 
is set to 0 (incoming people go from the top towards the bottom of the image), the
persons in this list are potentially EXITING the monitored gate).

\see inhi
*/
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

/*!
\var soglia_porta
\brief Posizione della soglia (intesa come riga sulla mappa di disparit&agrave;) oltre la quale un passeggero viene contato

Il passeggero viene contato come input o come output a seconda della direzionalita' della porta e del suo moto (entrato 
da sopra e uscito sotto o vicersa). Questa variabile viene opportunamente settata con SetDoor() prima della chiamata alla track()
in detectAndTrack(). Inoltre questa soglia puo' essere cambiata da win_client con il comando "threshold" (vedi commands.cpp).
Si noti che in widegate, solo il master fa' il tracking e quindi solo per il master viene usata questa soglia. E' per questo
motivo che in serial_port.cpp il comando "threshold" serve solo per gli slave.
*/
//unsigned short soglia_porta; 


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
//int num_pers = NUM_PERS_SING;

//int xt=160;

/*!
\var total_sys_number
\brief Used in widegate mode: total number of systems chained to supervise a wide gate.
*/
//unsigned char total_sys_number=1;

/*!
\var current_sys_number
\brief Used in widegate mode: index of the current system in the chain of system when in widegate.
*/
//unsigned char current_sys_number=1;

/*!
\var count_true_false
\brief Flag relativo alla potenziale inattivit&agrave; per 200 frame consecutivi (in questo caso vale false).
*/
//bool count_true_false=true;

//unsigned char move_det_en; //!< Flag relativo all'abilitazione del motion detection da interfaccia win_client.

//unsigned char door_stairs_en; //!< Corresponding to the option "Stairs Upgrade Enabled" in the tab "Advanced (2/2)" of the win_client application.



/*! 
\brief Inizializza 2 passi quando si apre la porta.

Serve per contare le persone al limite della immagine che altrimenti non farebbero un numero sufficiente di passi.

\param direction se la direzione &egrave; settata ad 1 allora una persona viene considerata come entrata se 
                attraversa la zona monitorata dall'alto verso il basso
*/
void SetPassi(const unsigned char & direction, const int & num_pers)
{
#ifdef debug_
    printf("set passi\n");
    int count_p=0;
    int count_p_hi=0;
    printf("SET PASSI \n");
    for(int i=0;i<num_pers;i++)
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
        for(int i=0;i<num_pers;i++)
        {
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                    inlo[i]->passi=3;
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
        for(int i=0;i<num_pers;i++)
        {
            if(inhi!=NULL)
                if(inhi[i]!=NULL)
                {
                    inhi[i]->passi=3;
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
\brief ??? Definita ma non utilizzata ???
*/
//void GetCnt(unsigned long& inp, unsigned long& outp)
//{
//    inp=people_count_input;
//    outp=people_count_output;
//}


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
               const int & num_pers)
{
    int pas_min=PAS_MIN;

    for(int i=0;i<num_pers;i++)
    {
        if(direction==0)
        {
            if(inhi!=NULL)
                if(inhi[i]!=NULL)
                {
                    int pas=inhi[i]->passi;
                    //la persona viene contata se esce dall'altra parte della scena
                    //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                    if((inhi[i]->y > door_threshold) && pas>=pas_min)
                    {
                        delete inhi[i];
                        inhi[i]=NULL;
                        if(move_det_en == 0)
                            people_count_input++;
                        else if(count_true_false == true)
                                people_count_input++;	
                    }
                }
        }
        else
        {
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                    int pas=inlo[i]->passi;
                    //la persona viene contata se esce dall'altra parte della scena
                    //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                    if((inlo[i]->y <= door_threshold) && pas>=pas_min)
                    {
                        delete inlo[i];
                        inlo[i]=NULL;
                        if(move_det_en == 0) people_count_output++;
                        else if(count_true_false == true)  people_count_output++;
                    }
                }
        }
    }
    trackin=people_count_input;
    trackout=people_count_output;
    deinitpeople(num_pers);
}


/*! 
\brief ???
*/
//unsigned char ReadDoor()
//{
//    return soglia_porta;
//}

/*!
\brief Setta la soglia della porta al valore "soglia", per poter discriminare 
        il caso di una persona entrata piuttosto che uscita.
*/
//void SetDoor(unsigned char soglia)
//{
//    soglia_porta=soglia;
//}


/*! 
\brief Allocazione delle liste di strutture dati di tipo tPersonTracked (inhi e inlo) in base
       a configurazione (se normale #NUM_PERS_SING, se widegate allora #NUM_PERS_SING*#total_sys_number) e inizializzazione
       dei contatori usati nel tracking #people_count_input e #people_count_output.
\param pi numero di passeggeri entrati
\param po numero di passeggeri usciti
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
    for(int i=0;i<num_pers;i++)
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
\brief Aggiorna le liste inhi e inlo.

Se una persona non viene pi&ugrave; rilevata allora vengono diminuite il suo numero di vite, 
la persona viene contata se esce dalla altra parte della scena, se ha fatto almeno 
#PAS_MIN passi e se ha finito le vite. Altrimenti se la persona non ha pi&ugrave; vite viene eliminata.

\param trackin  numero di passeggeri entrati
\param trackout numero di passeggeri usciti
*/
void clearpeople(unsigned long &trackin, unsigned long &trackout, 
                 const unsigned short & door_threshold, const unsigned char & move_det_en,
                 const bool & count_true_false, const int & num_pers)
{
    for(int i=0;i<num_pers;i++)
    {
        if(inhi!=NULL)
            if(inhi[i]!=NULL)
            {
                inhi[i]->life--; //se la persona non c'e piu' diminuisci la sua vita
                int pas=inhi[i]->passi;
                //la persona viene contata se esce dall'altra parte della scena
                //se ha fatto almeno PAS_MIN passi e se ha finito le vite
                if((inhi[i]->y > door_threshold)&&(inhi[i]->cont==true) && pas>PAS_MIN && inhi[i]->life==0)
                {
                    delete inhi[i];
                    inhi[i]=NULL;

                    if(move_det_en == 0) people_count_input++;
                    else if(count_true_false == true)  people_count_input++;
                }
                //se non ha piu' vite eliminala
                else if(inhi[i]->life==0)
                {
                    delete inhi[i];
                    inhi[i]=NULL;
                }
            }
            if(inlo!=NULL)
                if(inlo[i]!=NULL)
                {
                    inlo[i]->life--; //se la persona non c'e piu' diminuisci la sua vita
                    int pas=inlo[i]->passi;

                    if((inlo[i]->y < door_threshold)&&(inlo[i]->cont==true)&& pas>4 && inlo[i]->life==0)
                    {
#ifdef debug_
                        printf("Clearpeople: contato inlo h=%d x=%d y=%d life=%d\n",inlo[i]->h,inlo[i]->x,inlo[i]->y,inlo[i]->life);
#endif
                        delete [] inlo[i];
                        inlo[i]=NULL;
                        if(move_det_en == 0) people_count_output++;
                        else if(count_true_false == true)  people_count_output++;
                    }
                    else if(inlo[i]->life==0)
                    {
                        delete [] inlo[i];
                        inlo[i]=NULL;
                    }

                }
    }
    trackin = people_count_input;
    trackout = people_count_output;
    return; // a cosa serve un return se sono gia alla fine ???
}


/*! 
\brief Draw a cross on the disparity map where the person appears.
*/
void draw_cross_on_map(tPersonTracked *person_data, unsigned char *map)
{
    if(person_data->passi>2 && person_data->trac==true)
    {
        int pos_x=person_data->x;
        int pos_y=person_data->y;
            
        for(int x=max(0,pos_x-person_data->wx); x<min(NX-1,pos_x+person_data->wx); x++)
            map[NX*pos_y+x]=0xFF;

        for(int y=max(0,pos_y-person_data->wy); y<min(NY-1,pos_y+person_data->wy); y++)
            map[(NX*(y))+pos_x]=0xFF;
    }
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
orizzontale ed eventuale conteggio della 
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
*  \param door_kind specifica il tipo di porta (o ingresso, o uscita, o ingresso/uscita)
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
           const unsigned char & door_kind,
           const unsigned char & current_sys_number,
           const unsigned char & total_sys_number,
           const unsigned char & door_stairs_en,
           const unsigned char & move_det_en,
           const bool & count_true_false,
           int &num_pers,
           int &xt)
{
    /////////////////////////////////////////////////////////////////////////
    // inizializzazione strutture dati per il tracking
    
    // se e' la prima volta devo allocare e inizializzare inlo e inhi
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

#ifdef debug_
    //int ind=0; // 20111202 eVS, bugfix on indeces
    for(int i=0;i<person;i++)
    {
        int px=people[i]%xt;
        int py=people[i]/xt;
        int h=hpers[i];
        /*int wx=dimpers[i+ind]; 
        int wy=dimpers[i+ind+1];*/
        // 20111202 eVS, bugfix on indeces
        int wx=dimpers[2*i]; 
        int wy=dimpers[2*i+1];
        //ind++;
        if(px+wx>=NX) printf("x=%d y=%d wx=%d wy=%d h=%d \n",px,py,wx,wy,h);
    }
    int libere_lo=0;
    int libere_hi=0;
    printf("Inizio peopletrack\n");
#endif

    //suppongo nessuna persona nel tracciato
    for(int i=0;i<num_pers;i++)
    {
        if(inhi[i]!=NULL) inhi[i]->trac=false;
        if(inlo[i]!=NULL) inlo[i]->trac=false;
    }

    bool * non_trovate = new bool [person];
    for (int r=0;r<person; r++) non_trovate[r]=true;

    // fine inizializzazione
    /////////////////////////////////////////////////////////////////////////

    //contengono l'indice delle persone + vicine trovate
    unsigned char ind_perslo,ind_pershi;

    int distmin_inlo;
    int distmin_inhi;

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

    // per ogni persona che ho negli storici del tracking inlo e inhi trova
    // la persona piu' vicina nel frame corrente, quindi verifica che anche le altre 
    // proprieta' (larghezza, altezza e disparita') siano compatibili
    for(int i=0;i<NUM_PERS_SING*current_sys_number;i++) //per tutte le persone possibili
    {
        //calcola le distanze minime delle persone trovate nella mappa corrente
        //rispetto a quelle "traccate" e memorizzate in inhi e inlo
        ind_pershi=21; //indice della persona piu' vicina a quella descritta da inhi[i]
        ind_perslo=21; //indice della persona piu' vicina a quella descritta da inlo[i]
        distmin_inlo=2000; // distanza minima per poter avere un matching, questo valore potrebbe essere 
                           // anche infinito tanto poi vi e' un controllo sulla distanza minima 
                           // affinche' il candidato sia valido
        distmin_inhi=2000; // vedi commento precedente
        
        for(int p=0;p<person;p++) //per ogni persona trovata in questo frame
        {
            //if(non_trovate[p]) // se non e' gia' stata assegnata
            if(non_trovate[p] //)
               && hpers[p] > 0) // 20111202 eVS bugfix per No Tracking Zone
            {
                int px=people[p]%xt;//coordinate persone al frame corrente
                int py=people[p]/xt;
#ifdef debug_
                if(px>=xt) printf("px=%d XT=%d people[p]=%d\n",px,xt,people[p]);
                if(py>=YT) printf("py=%d YT=%d people[p]=%d\n",py,YT,people[p]);
#endif
                //unsigned char ux, uy;
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
                //calcola la distanza min delle inhi
                if (inhi[i]!=NULL)
                {
                    int ux=inhi[i]->x;
                    int uy=inhi[i]->y;

                    dist_tmp=(px-ux)*(px-ux)+(py-uy)*(py-uy);
                    if(dist_tmp<distmin_inhi)
                    {
                        distmin_inhi=dist_tmp;
                        ind_pershi=p;
                    }
                }
            }
        }

        if (ind_perslo!=21)                           
        {        
            // Se ind_perslo!=21 significa che l'algoritmo ha trovato per inlo[i] il potenziale candidato 
            // per il matching (cioe' quello piu' vicino nel frame corrente).
            
            // Verifico un eventuale conflitto nel matching, ossia se sia per inhi[i] che per inlo[i]
            // e' stato trovato lo stesso candidato, i.e. (ind_pershi==ind_perslo). Se non vi e' conflitto,
            // i.e. (ind_pershi!=ind_perslo), posso procedere. In caso di conflitto invece, i.e. (ind_pershi==ind_perslo),
            // bisogna discriminare se vince inhi[i] o inlo[i]. Per vedere se inlo[i] e' il vincente  basta
            // verificare che (distmin_inlo<distmin_inhi).
            if ((ind_pershi==ind_perslo && distmin_inlo<distmin_inhi) || ind_pershi!=ind_perslo)
            {
                // verificata la bonta del candidato indicato da "ind_perslo" per la persona descritta in inlo[i]
                // andiamo a valutare meglio l'accoppiamento
                int life=inlo[i]->life;

                // xke incremento di due unita' il numero di vite se ci sono + PCN collegati in serie ???
                if(total_sys_number>1) life+=2;

                //20100514 eVS
                static int dist_max;
                dist_max = (DIST_MAX+abs(LIFE-life)*DIST_INC);
                dist_max = dist_max*dist_max; 
                if (distmin_inlo<dist_max) //(400+abs(LIFE-life)*50)) 
                //se la distanza e' abbastanza piccola (si noti che distmin_inlo e' il quadrato della distanza) allora...
                {
                    // si noti che distanza e' considerata accettabile se minore di sqrt(400)=20pixels inoltre nel 
                    // caso in cui inlo[i]->life sia stato decrementato (blob senza matching per LIFE-life volte) 
                    // rabbonisco la soglia aumentandola di una quantita' proporzionale alle vite perdute 
                    // sqrt(400 + abs(LIFE-life)*50)
                    // Si noti che si tratta di un rabbonimento di pochi pixel in quanto, nel caso 
                    // peggiore (life=1) si ottiene sqrt(400+2*50)=22.4
                    int diff_height;
                    
                    // xke se ce' 1 solo PCN la diff_height e' 14 anziche' 30 ??? che significato hanno questi due numeri ???
                    if(total_sys_number<2) diff_height=14;
                    else diff_height=30;

                    //se la differenza di altezza fra le due pers e' contenuta
                    if( (inlo[i]->h - hpers[ind_perslo] < diff_height) ||
                        ((diff_cond_1p==1) && (inlo[i]->h/2<hpers[ind_perslo])) )
                    {
                        non_trovate[ind_perslo]=false;

                        //20111202 eVS, se ha fatto almeno 2px ha fatto un passo 
                        //PS. il nuovo algoritmo è talmente + stabile che, se le persone si muovono 
                        //lentamente, questa variabile non si inctementa e poi non vengono contate
                        if(distmin_inlo>4)
                        //if(distmin_inlo>9)//se ha fatto almeno 3px ha fatto un passo
                        {
                            inlo[i]->passi++;
                        }
                       
                        //inlo[i]->x=people[ind_perslo]%xt;
                        //inlo[i]->y=people[ind_perslo]/xt;
                        inlo[i]->x=(inlo[i]->x + 3*(people[ind_perslo]%xt))/4; // eVS 20100419 smoothing
                        inlo[i]->y=(inlo[i]->y + 3*(people[ind_perslo]/xt))/4;
#ifdef debug
                        if(inlo[i]->x>xt) printf("x=%d ind_pers=%d i=%d\n",inlo[i]->x,ind_perslo,i);
                        if(inlo[i]->y>YT) printf("y=%d ind_pers=%d i=%d\n",inlo[i]->y,ind_perslo,i);
#endif
                        //inlo[i]->wx=dimpers[ind_perslo];
                        //inlo[i]->wy=dimpers[ind_perslo+1];
                        /*inlo[i]->wx=(inlo[i]->wx + 3*dimpers[ind_perslo])/4; // eVS 20100419 smoothing
                        inlo[i]->wy=(inlo[i]->wy + 3*dimpers[ind_perslo+1])/4;*/
                        inlo[i]->wx=(inlo[i]->wx + 3*dimpers[2*ind_perslo])/4; // 20110930 eVS, bugfix of indexes
                        inlo[i]->wy=(inlo[i]->wy + 3*dimpers[2*ind_perslo+1])/4;
                        

                        //if(inlo[i]->passi>3) inlo[i]->life=LIFE+1;  // eVS 20100421 messo PAS_MIN come per inhi[]
                        //if(inlo[i]->passi>PAS_MIN) inlo[i]->life=LIFE+1; // eVS 20100518 check on the direction
                        if(inlo[i]->passi>PAS_MIN) {
                            if ((door_kind == 0) ||
                                (direction_inout == 0 && door_kind == 2) ||
                                (direction_inout == 1 && door_kind == 1)) 
                                inlo[i]->life=LIFE+1;
                            else
                                inlo[i]->life=min(LIFE+1,inlo[i]->life+1);
                        }
                        
                        if (hpers[ind_perslo]> inlo[i]->h) 
                            //inlo[i]->h = hpers[ind_perslo]; //ricorda l'altezza massima
                            inlo[i]->h = (inlo[i]->h + hpers[ind_perslo])/2; //ricorda l'altezza massima
                        //{
                        //    int tmp;
                        //    if (hpers[ind_perslo] > inlo[i]->h) 
                        //        tmp = (inlo[i]->h + 3*hpers[ind_perslo]);
                        //    else
                        //        tmp = (3*inlo[i]->h + hpers[ind_perslo]);
                        //    inlo[i]->h = tmp/4; // eVS 20100419 smoothing
                        //}
                            
                        inlo[i]->trac=true;
                    }
                }
            }
        }

        if (ind_pershi!=21)
        {
            if ((ind_pershi==ind_perslo && distmin_inlo>distmin_inhi) || ind_pershi!=ind_perslo)
                //se ha trovato una pers entrata dall'alto e la distanza e' minore
            {
                int life=inhi[i]->life;

                // xke incremento di due unita' il numero di vite se ci sono + PCN collegati in serie ???
                if(total_sys_number>1) life+=2;

                //20100423 eVS
                static int dist_max;
                dist_max = (DIST_MAX+abs(LIFE-life)*DIST_INC);
                dist_max = dist_max*dist_max; 
                // distmin_inhi e' il quadrato della distanza
                if (distmin_inhi < dist_max)//(400+ abs(LIFE-life)*50))  //se la distanza e' abbastanza piccola
                {
                    int diff_height;
                    
                    // xke se ce' 1 solo PCN la diff_height e' 14 anziche' 30 ??? che significato hanno questi due numeri ???
                    if(total_sys_number<2) diff_height=14;
                    else diff_height=30;

                    if( (inhi[i]->h - hpers[ind_pershi]<diff_height) || 
                        ((diff_cond_1p==1) && (inhi[i]->h/2<hpers[ind_pershi])) )
                    //se la differenza di altezza fra le due pers e' contenuta
                    {
                        non_trovate[ind_pershi]=false;
                        
                        //20111202 eVS, se ha fatto almeno 2px ha fatto un passo 
                        //PS. il nuovo algoritmo è talmente + stabile che, se le persone si muovono 
                        //lentamente, questa variabile non si inctementa e poi non vengono contate
                        if(distmin_inhi>4)
                        //if(distmin_inhi>9)//se ha fatto almeno 3px ha fatto un passo
                        {
                            inhi[i]->passi++;
                        }
                        
                        //inhi[i]->x=people[ind_pershi]%xt;
                        //inhi[i]->y=people[ind_pershi]/xt;
                        inhi[i]->x=(inhi[i]->x + 3*(people[ind_pershi]%xt))/4; // eVS 20100419 smoothing
                        inhi[i]->y=(inhi[i]->y + 3*(people[ind_pershi]/xt))/4;
#ifdef debug
                        if(inhi[i]->x>xt) printf("x=%d ind_pers=%d \n",inhi[i]->x,ind_pershi);
                        if(inhi[i]->y>YT) printf("y=%d ind_pers=%d \n",inhi[i]->y,ind_pershi);
#endif                             
                        //if(inhi[i]->passi>PAS_MIN) inhi[i]->life=LIFE+1; // eVS 20100518 check on the direction
                        if(inhi[i]->passi>PAS_MIN) {
                            if ((door_kind == 0) ||
                                (direction_inout == 0 && door_kind == 1) ||
                                (direction_inout == 1 && door_kind == 2)) 
                                inhi[i]->life=LIFE+1;
                            else
                                inhi[i]->life=min(LIFE+1,inhi[i]->life+1);
                        }
                        
                        if (hpers[ind_pershi] > inhi[i]->h) 
                            //inhi[i]->h=hpers[ind_pershi]; //ricorda l'altezza massima
                            inhi[i]->h=(inhi[i]->h+hpers[ind_pershi])/2; //ricorda l'altezza massima
                        //{
                        //    int tmp;
                        //    if (hpers[ind_pershi] > inhi[i]->h) 
                        //        tmp = (inhi[i]->h + 3*hpers[ind_pershi]);
                        //    else
                        //        tmp = (3*inhi[i]->h + hpers[ind_pershi]);
                        //    inhi[i]->h = tmp/4; // eVS 20100419 smoothing
                        //}
                                
                        inhi[i]->trac=true;
                            
                        //inhi[i]->wx=dimpers[ind_pershi];
                        //inhi[i]->wy=dimpers[ind_pershi+1];
                        /*inhi[i]->wx=(inhi[i]->wx + 3*dimpers[ind_pershi])/4; // eVS 20100419 smoothing
                        inhi[i]->wy=(inhi[i]->wy + 3*dimpers[ind_pershi+1])/4;*/
                        inhi[i]->wx=(inhi[i]->wx + 3*dimpers[2*ind_pershi])/4; // eVS 20111202 bugfix of indexes
                        inhi[i]->wy=(inhi[i]->wy + 3*dimpers[2*ind_pershi+1])/4;
                        
                    }
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
        //if(non_trovate[t])
        if(non_trovate[t] //)
           && hpers[t] > 0) // 20111202 eVS bugfix per No Tracking Zone
        {
            int px=people[t]%xt;
            int py=people[t]/xt;

            // inserisco le persone entrate dalla parte alta nella lista delle persone da traccare
            // (si tratta delle persone che entrano sull'autobus se la direction e' 0, mentre son 
            // quelle che escono se direction e' 1)
            if(py<=door_threshold)
            {
                int n=person;
                if(n<num_pers)
                    while(inhi[n]!=NULL) //trovare la prima libera prima in avanti e poi
                    {
                        n++;
                        if(n>=num_pers)	break;
                    }
                    if(n>=num_pers) //indietro garantisce nella maggior parte dei casi di assegnare
                    {//alla persona presente da + tempo nel tracciato (="+ vecchia") =>LIFO
                        n=0;
                        while(inhi[n]!=NULL)
                        {
                            n++;
                            if (n>=num_pers) break;
                        }       
                    }        
                    //se alla fine ha trovato un indice libero inferiore a NUM_PERS
                    if(n<num_pers)//controllo di puntare ad una zona corretta
                    {
                        inhi[n]=new tPersonTracked;
                        inhi[n]->cont=false;
                        inhi[n]->trac=true;
                        inhi[n]->passi=0;
                        /*inhi[n]->wx=dimpers[t];
                        inhi[n]->wy=dimpers[t+1];*/
                        inhi[n]->wx=dimpers[2*t]; // 20111202 eVS, bug fix on indexes
                        inhi[n]->wy=dimpers[2*t+1];
                        
                        inhi[n]->h=hpers[t];
                        
                        //inhi[n]->life=LIFE; // eVS 20100419
                        /*if (door_kind == 0) // eVS 20100518 replaced with the code after comments
                            inhi[n]->life=LIFE;
                        else
                            if (direction_inout == 0) // people coming in from top
                                if (door_kind == 1) // one way in
                                    inhi[n]->life=LIFE;
                                else
                                    inhi[n]->life=LIFE/2;
                            else // people coming in from the bottom
                                if (door_kind == 1) // one way in
                                    inhi[n]->life=LIFE/2;
                                else
                                    inhi[n]->life=LIFE;*/
                        if ((door_kind == 0) ||
                            (direction_inout == 0 && door_kind == 1) ||
                            (direction_inout == 1 && door_kind == 2)) 
                            inhi[n]->life=LIFE;
                        else
                            inhi[n]->life=LIFE/2;
                            
                        inhi[n]->x=px;
                        inhi[n]->y=py;
                        inhi[n]->first_y=py;
                        //    printf("Inserita nuova inhi, y nascita %d\n",inhi[n]->first_y);
#ifdef debug
                        if(inhi[n]->x>xt) printf("Nuove pers: x=%d n=%d \n",inhi[n]->x,n);
                        if(inhi[n]->y>YT) printf("Nuove pers: y=%d n=%d \n",inhi[n]->y,n);
#endif
                    }
            }
            
            // inserisco le persone entrate dalla parte bassa nella lista delle persone da traccare
            // (si tratta delle persone che escono dall'autobus)
            if(py>door_threshold)
            {
                int n=person;
                if(n<num_pers) 
                    while(inlo[n]!=NULL)
                    {
                        n++;
                        if(n>=num_pers)	break;
                    }

                    if(n>=num_pers)
                    {
                        n=0;
                        while(inlo[n]!=NULL)
                        {
                            n++;
                            if (n>=num_pers) break;
                        }        
                    }
                    //se alla fine ha trovato un indice libero inferiore a NUM_PERS
                    if(n<num_pers)
                    {
                        inlo[n]=new tPersonTracked;
                        inlo[n]->cont=false;
                        inlo[n]->trac=true;
                        inlo[n]->passi=0;
                        
                        //inlo[n]->life=LIFE; // eVS 20100419
                        /*if (door_kind == 0) // eVS 20100518 replaced with the code after comments
                            inlo[n]->life=LIFE;
                        else
                            if (direction_inout == 0) // people coming in from top
                                if (door_kind == 2) // one way out
                                    inlo[n]->life=LIFE;
                                else
                                    inlo[n]->life=LIFE/2;
                            else // people coming in from the bottom
                                if (door_kind == 2) // one way out
                                    inlo[n]->life=LIFE/2;
                                else
                                    inlo[n]->life=LIFE;*/
                        if ((door_kind == 0) ||
                            (direction_inout == 0 && door_kind == 2) ||
                            (direction_inout == 1 && door_kind == 1)) 
                            inlo[n]->life=LIFE;
                        else
                            inlo[n]->life=LIFE/2;
                        
                        inlo[n]->h=hpers[t];
                        /*inlo[n]->wx=dimpers[t];
                        inlo[n]->wy=dimpers[t+1];*/
                        inlo[n]->wx=dimpers[2*t]; // 20111202 eVS, bug fix on indexes
                        inlo[n]->wy=dimpers[2*t+1];
                        
                        inlo[n]->x=px;
                        inlo[n]->y=py;
                        inlo[n]->first_y=py;
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


    // determina quali persone diventano candidate per il conteggio
    // cioe' se sono traccate e se hanno sorpassato la soglia
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
        if(inlo[l]!=NULL)
        {
            if(inlo[l]->cont==false && inlo[l]->trac==true)
            {
                // int pas=inlo[l]->passi;
                if(inlo[l]->y < door_threshold)
                {
                    inlo[l]->cont=true;
                }
            }
        }
    }



    // conteggio delle persone
    for(int i=0;i<num_pers;i++)
    {
        if(inhi[i]!=NULL)
        {
            if(inhi[i]->trac==false)
            {
                inhi[i]->life--; //se la persona nello storico non e' stata traccata diminuisci di uno le sue vite
                
                int pas=inhi[i]->passi;
                
                /*   if(inhi[i]->life==0)
                {
                printf("door_stairs_en %d\n",door_stairs_en);
                printf("inhi[i]->y %d\n",inhi[i]->y);
                printf("door_threshold %d\n",door_threshold);
                printf("inhi[i]->cont %d\n",inhi[i]->cont);
                printf("pas > PAS_MIN %d > %d\n",pas,PAS_MIN);
                printf("inhi[i]->life %d\n",inhi[i]->life);
                printf("inhi[i]->first_y-inhi[i]->y  %d-%d = %d \n",inhi[i]->y,inhi[i]->first_y, inhi[i]->first_y-inhi[i]->y);
                }*/

                if((door_stairs_en==1)
                    &&(inhi[i]->y < door_threshold)
                    &&(inhi[i]->cont==false)
                    &&(pas>PAS_MIN)
                    &&(inhi[i]->life==0)
                    &&(inhi[i]->first_y-inhi[i]->y>30))
                {
                    people_count_output++;  
                    //  printf("contato (door_stairs_en==1) OUT, y in-out %d %d\n",inhi[i]->first_y,inhi[i]->y);  
                    delete inhi[i];
                    inhi[i]=NULL;
                }
                else if((inhi[i]->y > door_threshold)
                         &&(inhi[i]->cont==true)
                         &&(pas>PAS_MIN)
                         &&(inhi[i]->life==0))
                {
#ifdef debug
                    printf("contato inhi h=%d x=%d y=%d life=%d\n",inhi[i]->h,inhi[i]->x,inhi[i]->y,inhi[i]->life);
#endif 
                    delete inhi[i];
                    inhi[i]=NULL;
                    
                    // incremento il contatore delle persone entrate solo se
                    // il motion detection e' disabilitato oppure se e' abilitato e c'e' "movimento" a meno di 200 frame di 
                    // potenziale inattivita' (vedi loops.cpp funzione main_loop)
                    if(move_det_en == 0) people_count_input++;
                    else if(count_true_false == true)  people_count_input++;
                }
                else if(inhi[i]->life==0)
                {
                    delete inhi[i];
                    inhi[i]=NULL;
                }
            }
        }

        if(inlo[i]!=NULL)
        {
            if(inlo[i]->trac==false)
            {
                inlo[i]->life--; //se la persona non c'e' piu' diminuisci la sua vita
                int pas=inlo[i]->passi;
                
                /*     if(inlo[i]->life==0)
                {
                printf("door_stairs_en %d\n",door_stairs_en);
                printf("inlo[i]->y %d\n",inlo[i]->y);
                printf("door_threshold %d\n",door_threshold);
                printf("inlo[i]->cont %d\n",inlo[i]->cont);
                printf("pas > PAS_MIN %d > %d\n",pas,PAS_MIN);
                printf("inlo[i]->life %d\n",inlo[i]->life);
                printf("inlo[i]->y-inlo[i]->first_y  %d-%d = %d \n",inlo[i]->y,inlo[i]->first_y, inlo[i]->y-inlo[i]->first_y);
                }*/
                if((door_stairs_en==1)
                    &&(inlo[i]->y > door_threshold)
                    &&(inlo[i]->cont==false)
                    &&(pas>PAS_MIN)
                    &&(inlo[i]->life==0)
                    &&(inlo[i]->y-inlo[i]->first_y>30))
                {
                    people_count_input++;  
                    //  printf("contato (door_stairs_en==1) IN, y in-out %d %d\n",inlo[i]->first_y,inlo[i]->y);  
                    delete inlo[i];
                    inlo[i]=NULL;
                }
                else if((inlo[i]->y < door_threshold)
                         &&(inlo[i]->cont==true)
                         &&(pas>PAS_MIN)
                         &&(inlo[i]->life==0))
                {
#ifdef debug
                    printf("contato inlo h=%d x=%d y=%d life=%d\n",inlo[i]->h,inlo[i]->x,inlo[i]->y,inlo[i]->life);
#endif
                    delete inlo[i];
                    inlo[i]=NULL;
                    if(move_det_en == 0) people_count_output++;
                    else if(count_true_false == true)  people_count_output++; 
                }
                else if(inlo[i]->life==0)
                {
                    delete inlo[i];
                    inlo[i]=NULL;
                }
            }
        }
    }


    //disegna le persone
    if(total_sys_number<2)
    {
        for(int i=num_pers-1;i>=0;i--)
        {
            if(inhi[i]!=NULL)
            {
#ifdef debug
                if(inhi[i]->x+inhi[i]->wx>=NX) 
                {
                    printf("Disegno:x + wx troppo grande x=%d wx=%d ind=%d \n",int(inhi[i]->x),int(inhi[i]->wx),i);
                    break;
                }
#endif
                draw_cross_on_map(inhi[i], disparityMap);
                /*if(inhi[i]->passi>2 && inhi[i]->trac==true)
                {
                    int pos_x=inhi[i]->x;
                    int pos_y=inhi[i]->y;
                                        
                    // eVS
                    //for(int x=-inhi[i]->wx; x<inhi[i]->wx; x++)
                    //    disparityMap[NX*pos_y+pos_x+x]=0xFF;
                    for(int x=max(0,pos_x-inhi[i]->wx); x<min(NX-1,pos_x+inhi[i]->wx); x++)
                        disparityMap[NX*pos_y+x]=0xFF;
                        
                    // eVS
                    //for(int y=-inhi[i]->wy; y<inhi[i]->wy; y++)
                    //    disparityMap[(NX*(pos_y+y))+pos_x]=0xFF;
                    for(int y=max(0,pos_y-inhi[i]->wy); y<min(NY-1,pos_y+inhi[i]->wy); y++)
                        disparityMap[(NX*(y))+pos_x]=0xFF;
                }*/
            }
            if(inlo[i]!=NULL)
            {
#ifdef debug
                if(inlo[i]->x+inlo[i]->wx>=NX) 
                {
                    printf("Disegno:x + wx troppo grande x=%d wx=%d ind=%d \n",int(inlo[i]->x),int(inlo[i]->wx),i);
                    break;
                }
#endif
                draw_cross_on_map(inlo[i], disparityMap);
                /*if(inlo[i]->passi>2 && inlo[i]->trac==true)
                {
                    int pos_x=inlo[i]->x;draw_cross_on_map(inlo[i]);
                    int pos_y=inlo[i]->y;
                    
                    // eVS
                    //for(int x=-inlo[i]->wx; x<inlo[i]->wx; x++)
                    //    disparityMap[NX*pos_y+pos_x+x]=0xFF;
                    for(int x=max(0,pos_x-inlo[i]->wx); x<min(NX-1,pos_x+inlo[i]->wx); x++)
                        disparityMap[NX*pos_y+x]=0xFF;
                        
                    // eVS
                    //for(int y=-inlo[i]->wy; y<inlo[i]->wy; y++)
                    //    disparityMap[(NX*(pos_y+y))+pos_x]=0xFF;
                    for(int y=max(0,pos_y-inlo[i]->wy); y<min(NY-1,pos_y+inlo[i]->wy); y++)
                        disparityMap[(NX*(y))+pos_x]=0xFF;
                }*/
            }
        }
    }
    
    trackin = people_count_input;
    trackout = people_count_output;

    delete [] non_trovate;     

    return;
    
    
    /*!
    <b>Calcolo delle distanze minime</b>
    \code
    // Calcolo delle distanze minime scandendo tutti i blob contenuti nelle liste "inhi" e "inlo"
    // e confronto con i blob trovati dall'algoritmo di people detection nel frame corrente.
    for(int i=0;i<NUM_PERS_SING*current_sys_number;i++)
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
                    inlo[i]->wx=dimpers[2*ind_perslo];
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
                        inhi[n]->wx=dimpers[2*t];
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
    for(int i=0;i<num_pers;i++)
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
        for(int i=num_pers-1;i>=0;i--)
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

#endif
