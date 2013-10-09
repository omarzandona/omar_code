/*!
\file imgserver.cpp
\brief Sorgente principale dell'imgserver: contiene le inizializzazioni di variabili, 
       la creazione dei thread ed, infine, il ciclo in cui il server resta in ascolto del client.

Si tratta del file principale del progetto dove risiede il main all'interno del quale viene fatto quanto segue: 
- parsing degli eventuali argomenti (--dir o -version, il primo permette di
  specificare una cartella diversa da quella di default in cui andare a
  leggere e scrivere i vari file mentre il secondo visualizza a video
  la versione dello image server); 
- inizializzazione di alcune delle innumerevoli variabili 
  globali (vedi var_init()); 
- inizializzazione dei device di input e 
  output (vedi init_io()); 
- caricamento dei parametri di default per i device ed altre variabili 
  globali (vedi load_parms() e calib_load_parms()); 
- caricamento della mappa di disparita' del background da file; 
- inizializzazione strutture dati per il tracking (vedi initpeople());
- aggiunta al file di log del messaggio di avvio dello imgserver 
  con verifica di riempimento del file: se i file di log hanno un numero
  di righe che supera le MAX_LOG_LINES allora viene creato un nuovo 
  file di log numerato ciclicamente da 0 a 9.
- vengono attivati i vari thread corrispondenti alle seguenti
  funzioni implementate in loops.cpp: main_loop(), watchdog_loop(), 
  record_loop(), ser_loopttyS0(), ser_loopttyS1(), input_loop0() e 
  input_loop1();
- inizializza la comunicazione via socket tramite un canale TCP di 
  comunicazione listener usato per rimanere in ascolto degli eventuali 
  messaggi del client. 
  Inoltre, viene creato un ulteriore canale UDP di comunicazione datagram
  utilizzato in ping_loop() ma per il quale non viene lanciato il comando
  listen() e non vengono specificate opzioni particolari con setsockopt(). 
- crea il thread corrispondente al ping_loop();
- infine, il main() e' concluso da un ciclo infinito con il quale lo
  image server resta in ascolto di eventuali comandi inviati dal
  win_client tramite la RecvString() e se li riceve li passa alla
  Communication() affinche' ne venga fatto il parsing e venga presa
  l'opportuna azione.

Per ulteriori dettagli vedere la funzione main().

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)

\mainpage
    Il passenger counter &egrave; un sistema di visione embedded utilizzato per contare le persone
    che attraversano un varco (ad esempio i passeggeri di un autobus che salgono e scendono).
    Il sistema &egrave; suddiviso in tre parti fondamentali: hardware, firmware e software.    
    
    Con hardware si intende l'insieme dei dispositivi che costituiscono il passenger
    counter ad eccezione dell'FPGA che insieme all'imgserver formano il firmware. 
    Quindi, in questa categoria ricadono il processore, la memoria, i bus, le memorie
    flash, i sensori CMOS, ecc...

    In FPGA sono implementati vari algoritmi di image processing come calcolo della mappa 
    di disparit&agrave;, il calcolo dei parametri di motion detection e il valor medio
    delle due immagini acquisite.

    La parte software si suddivide in parte client (<b>win_client/rs485_gui</b>) e parte 
    server (<b>imgserver</b>). Dal momento che il software del lato server fa comunque parte del 
    sensore PCN, sar&agrave; considerato parte del firmware.
    
    Questo documento ipertestuale descrive il codice del software del lato server (imgserver) che 
    incorpora la logica di alto livello necessaria per il conteggio 
    delle persone, ovvero sottrazione dello sfondo dalla mappa di disparit&agrave; e 
    successivo filtraggio, detection e tracking delle teste. Il software del lato client 
    consta di due applicazioni <b>win_client</b> e <b>rs485_gui</b>. Riassumendo, win_client &egrave; 
    un'interfaccia utente che permette di interagire con un sensore PCN
    mediante connessione USB al fine di aggiornare il software del server, di 
    visualizzare le immagini sorgenti, quelle rettificate, la mappa di disparit&agrave; 
    originale, quella filtrata e il risultato del tracking delle teste. Il software
    rs485_gui &egrave; sempre una interfaccia grafica che permette di interagire col sensore 
    PCN ma per via seriale. Dati i limiti intrinseci di questo tipo di collegamento, 
    questo client ha funzionalit&agrave; pi&ugrave; limitate.
    
    Esiste anche una versione di <b>win_client di debug</b> con cui &egrave; possibile acquisire delle sequenze
    al fine di acquisire un data set da usare come test-bed durante lo sviluppo di modifiche 
    e/o evoluzioni del software. Per questa versione esistono due comandi specifici "recimg" 
    e "recimgdsp" (vedi commands.cpp) che permettono al client di chiedere al server i valori attuali dei contatori
    in e out ed inoltre o le immagini+mappa di disparit&agrave; ("recimg") o solo la mappa di 
    disparit&agrave; ("recimgdsp"). Affinch&egrave; i contatori siano perfettamente allineati con le
    immagini passate, anche gli algoritmi di detezione e tracking sono eseguiti dal server 
    solo in concomitanza delle richieste del client.

    Per quanto riguarda il lato server qui descritto, possiamo riassumerne
    le caratteristiche andando ad illustrare brevemente il contenuto dei vari
    file:
    
    - imgserver.cpp: contiene il main() dell'applicazione dove vengono effettuate
      tutte le inizializzazioni delle variabili globali, dei device e dei necessari
      canali di comunicazione per poter interagire con il client o con il software 
      concentratore (molti dei valori di default usati per le varie inizializzazioni
      sono definiti in default_parms.h); quindi vengono creati i vari thread corrispondenti alle funzioni
      di tipo loop in loops.cpp; infine vi &egrave; un ciclo infinito (a meno di 
      errori che provocano la terminazione dell'applicazione) che gestisce la 
      comunicazione con il client via socket per mezzo delle funzioni contenute 
      in socket.cpp e in commands.cpp. I parametri sono memorizzati in due strutture dati
      descritte rispettivamente in \ref tabella_parms e in \ref tabella_calib.
    - loops.cpp: contiene il codice relativo ai vari thread lanciati nel main()
      tra i quali vi &egrave; il main_loop() che si occupa di acquisire i dati
      dall'FPGA (vedi images_fpga.cpp) e di elaborarli per il detection (vedi peopledetection.cpp) e il 
      tracking (vedi peopletrack.cpp) delle persone.
    - socket.cpp: insieme di funzioni per la gestione della comunicazione via socket.
    - commands.cpp: contiene un'unica monolitica funzione che gestisce lo scambio
      di messaggi tra client (<b>win_client</b>) e server via socket: interpreta il pacchetto arrivato dal client, esegue
      quanto richiesto e risponde con un opportuno feedback.
    - serial_port.cpp: insieme di funzioni per la gestione della comunicazione tra client (<b>rs485_gui</b>) e 
      imgserver via seriale nonche' tra i vari sensori nel caso di configurazione "widegate"
      usata per monitorare varchi piu' larghi di 120cm (60cm in piu' per ogni ulteriore sensore
      fino a un massimo di 5).
    - io.cpp: insieme di funzioni per la gestione dei device e per la lettura e scrittura
      dei parametri su file.
    - calib_io.cpp: insieme di funzioni per la lettura e scrittura su file dei
      parametri di calibrazione che possono essere cambiati via client sciegliendo
      una delle due configurazioni 25/30 o 31/40.
    - pcn1001.h: contiene tutta una serie di define tra cui i valori
      da usare per settare il registro FPGA che specifica cosa l'FPGA restituisce nel 
      buffer prelevato poi nel main_loop() e alcuni settaggi per configurare l'acquisizione
      dei sensori CMOS.
    - makefile: direttive per il compilatore.

    <b>Note per la consultazione</b><br>
    Per meglio consultare questa documentazione si tenga presente che:
    - Si puo' sfruttare il motore di ricerca in alto a destra.
    - I commenti dei singoli sottoprogrammi sono completati da due voci "Riferimenti/References" e "Referenziato 
      da/Referenced by": la prima corrisponde a un elenco di tutte le variabili, funzioni e costanti usate 
      all'interno della funzione in esame; la seconda corrisponde a un elenco di altri sottoprogrammi
      che richiamano al loro interno il sottoprogramma in esame. Data la grande quantita' di
      variabili globali e la scarsa modularita' del codice, questi due elenchi risultano
      molto comodi per poter, per esempio, "ricostruire la storia" di una variabile o capire l'uso di
      un particolare sottoprogramma in base a dove viene chiamato.
    - Alle volte i commenti contengono tre punti interrogativi "???" per indicare
      quei punti ancora poco chiari.
    - Si noti, inoltre, che la struttura del progetto software non segue le direttive
      standard che prevedono coppie di file .c e .h al fine di modulare il codice
      bensi' imgserver.cpp include tutti gli altri file cpp ottenendo qualcosa di equivalente 
      ad un unico file cpp.
    - Se ci sono dubbi si consiglia di consultare anche le technotes che completano la 
      documentazione al codice.
    
\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)

\note Per info e dettagli contattare Andrea Colombari (andrea.colombari@evsys.net)
*/

#include "imgserver.h"

extern int num_pers;

unsigned char Frame[NN << 2]; //!< Blocco dati trasferito mediante quick capture technology (#NN*4=#NX*#NY*4=160*120*4=320*240 bytes).
unsigned char Frame_DX[NN];   //!< Immagine destra a 256 livelli di grigio (#NX*#NY=160*120).
unsigned char Frame_SX[NN];   //!< Immagine sinistra a 256 livelli di grigio (#NX*#NY=160*120).
unsigned char Frame_DSP[NN];  //!< Mappa di disparit&agrave; #NX*#NY=160*120 (16 livelli di disparit&agrave; distribuiti su 256 livelli di grigio).

//unsigned char minuti_log=0; //!< ???

char FPN_DX[NN]; //!< Buffer contenente il Fixed Pattern Noise del sensore destro (#NN=#NX*#NY=160*120).
char FPN_SX[NN]; //!< Buffer contenente il Fixed Pattern Noise del sensore sinistro (#NN=#NX*#NY=160*120).

unsigned short Frame10_DX[NN]; //!< Immagine destra a toni di grigio a 10 bit di dimensione #NX*#NY=160*120.
unsigned short Frame10_SX[NN]; //!< Immagine sinistra a toni di grigio a 10 bit di dimensione #NX*#NY=160*120.

short ODCbuf[NN*8]; //!< Buffer contenente le informazioni (look-up-table) per la correzione della distorsione causata dalle ottiche (dimensione totale del buffer = #NX*#NY*8=160*120*8 bytes).

//char records[MAX_RECORDS][64]; //!< Buffer storing log messages (reporting counting results) periodically written on file.
char records[MAX_RECORDS][128]; //!< Buffer storing log messages (reporting counting results) periodically written on file.
/*!<
	Records (log lines reporting counting results) are not immediately written on a file but they are first stored in a
	buffer of strings (one for each line of at most 64 characters) named #records which is periodically (every 
	#RECORD_INTERVAL seconds) got empty and saved on a file with name #rd_filename.
	
	Therefore, this buffer has to be big enough to contain all the records. Supposing that people run at 5m/sec and
	that the detection area is 100cm*120cm the upper bound #MAX_PEOPLE_PER_SEC can be fixed at 10 persons per second. 
	This number is multiplied by the #RECORD_INTERVAL to obtain the maximum dimension for the buffer #MAX_RECORDS. 
	If (but it should not be) the buffer get full then the last element is re-written.
	
	A file of records can be at maximum of #MAX_REC_LINES lines of logs. When a file get greater than that
	another file is created with a new suffix. This suffix starts from 0 and goes to #MAX_REC_FILES-1 and 
	then starts from 0 again and the old file is rewritten.
*/

char working_dir[128] = WORK_DIR; //!< Cartella di lavoro (#WORK_DIR).

char bg_filename[128]; //!< Percorso del file che contiene lo sfondo (#WORK_DIR + #BG_FILENAME).
char rd_filename[128]; //!< Percorso del file che contiene i messaggi di log (#WORK_DIR + #RD_FILENAME).
char pm_filename[128]; //!< Percorso del file che contiene i parametri della porta seriale, no tracking zone e motion detection (#WORK_DIR + #PM_FILENAME).
char ca_filename[128]; //!< Percorso del file che contiene i parametri di calibrazione (#WORK_DIR + #CA_FILENAME).
char cr_filename[128]; //!< Percorso dei file che contiene i contatori delle persone entrate/uscite (#WORK_DIR + #CR_FILENAME).

int pxa_qcp; //!< File descriptor della porta di tipo quick capture della intel. 
/*!<
Viene utilizzata mediante la chiamata ioctl() per l'interfacciamento con l'FPGA, 
ad esempio settaggio strutture dati video_picture e video_window, 
inizializzazione cattura video e settaggio della modalit&agrave; di acquisizione (ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)).
*/

int imagesize;                 //!< Size of the Quick Capture Interface buffer which is cpoied in #Frame (#NN*4=#NX*#NY*4).
struct video_picture vid_pict; //!< Questa variabile &egrave; una struttura dati di tipo "video_picture" (Video4Linux).
struct video_window vid_win;   //!< Questa variabile &egrave; una struttura dati di tipo "video_window" (Video4Linux).
pxa_i2c_t i2cstruct;           //!< Variabile usata per la comunicazione via I2C.
char addresses[MAX_CONN][16];  //!< Lista degli indirizzi relativi alle connessioni accettate dal server mediante la funzione accept().

struct sockaddr_in myaddr;          //!< Indirizzo lato server associato alla socket di tipo TCP per la comunicazione client/server (mediante connessione usb).

// 20111014 eVS, myaddr_data declaration moved in the ping_pong loop code
//struct sockaddr_in myaddr_data;     //!< Indirizzo lato server associato alla socket di tipo UDP per la comunicazione client/server (mediante connessione usb).
struct sockaddr_in remoteaddr;      //!< Indirizzo lato client associato alla socket di tipo TCP per la comunicazione client/server (mediante connessione usb).

struct sockaddr_in remoteaddr_data; //!< Indirizzo lato client associato alla socket di tipo UDP per la comunicazione client/server (mediante connessione usb).
int fdmax;      //!< Numero massimo di file descriptor da associare alla socket TCP per la comunicazione client/server.
int listener;   //!< Nuovo file descriptor restituito dalla funzione accept (listener=socket(AF_INET,SOCK_STREAM,0)) quando il server accetta la nuova connessione da parte di un client (mediante socket di tipo TCP).
int sockfd;     //!< File descriptor associato alla socket di tipo TCP per la comunicazione tra win_client ed imgserver. Viene restituito dalla funzione accept: newfd = accept(listener, (struct sockaddr *)&remoteaddr,&addrlen)).
bool connected; //!< Flag che indica se un client &egrave; connesso oppure no al server.

// 20111014 eVS, datagram declaration moved in the ping_pong loop code
//int datagram;   //!< File descriptor associato alla socket di tipo UDP (datagram = socket(AF_INET, SOCK_DGRAM, 0)).
/*
  In questo caso i pacchetti di dati (datagram o datagramma) hanno una lunghezza massima prefissata, 
  indirizzati singolarmente. Non esiste una connessione e la trasmissione &egrave; effettuata
  in maniera non affidabile.
*/


//int level; //??? DEFINITA MA NON USATA ???
//unsigned short door;	// ??? DEFINITA MA NON USATA ??? 

int record_enabled; //!< Salvataggio messaggi di log abilitato.
int thread_status;  //!< Stato del thread associato al ciclo principale di esecuzione.
int count_enabled;  //!< Person counting enabled (via I/O).
int images_enabled; //!< Abilitazione dell'invio delle immagini dall'imgserver al win_client.
bool autoled;       //!< Modalita' automatica di funzionamento dei LED.
bool led_status;    //!< Stato degli illuminatori.
char sys_ver[256];  //!< Versione del sistema operativo.
float fw_ver;       //!< Versione del bitstream (FPGA).
bool autogain;      //!< Modalità automatica gain      20130927 eVS added to manage gain Vref

unsigned char fpn_counter;   //!< Relative to the FPN calibration application (to obtain FPN win_client_calib needs more than one image and this counter helps).

unsigned long records_idx;   //!< Indice del buffer dei messaggi di log #records.
// 20111012 eVS, added records_saving_in_progress
bool records_saving_in_progress = false;   //!< A true se il salvataggio dei records è in corso (in tal caso la "rdsv" e "rdsave" aspetteranno la fine del salvataggio prima di procedere al trasferimento del log
unsigned long counter_in;    //!< Contatore delle persone entrate INDIPENDENTE dalla direzione di marcia (ovvero come se la direzione fosse 0).
unsigned long counter_out;   //!< Contatore delle persone uscite INDIPENDENTE dalla direzione di marcia (ovvero come se la direzione fosse 0).

unsigned char people_dir; //!< Vale zero se la direzione di ingresso &egrave; dall'alto verso il basso, uno viceversa.

unsigned long out0; //!< optocoupled out0 queue for incoming people. 
/*!< 
    The default value 0 (given at startup by #var_init) but during counting this
    value is related to the number of the counted persons and is used to
    communicate counting data to the collector via digital 
    output (see #write_output).
    
    \see record_counters()   
*/
unsigned long out1; //!< optocoupled out1 queue for outgoing people.
/*!< 
    \see out0 
*/

unsigned short acq_mode; //!< Specifica se l'acquisizione &egrave; o meno abilitata e, nel caso in cui sia abilitata, dice in quale modalit&agrave; avviene l'acquisizione.
/*!<
Inizializzata in init_io().

Le due informazioni, su accensione/spegnimento dell'acquisizione e su quale modalit&agrave;
&egrave; in uso, sono inserite rispettivamente nella parte alta e bassa dei sedici bit
della variabile. Quindi per vedere se l'acquisizione &egrave; attiva basta fare
il seguente and bit a bit 
\code
(acq_mode & 0x0100)
\endcode
se la parte alta &egrave; a 1 questo and restituir&agrave; 1
altrimento dar&agrave; 0.

Per estrarre la parte bassa, invece, baster&agrave; fare il seguente and bit a bit
\code
(acq_mode & 0x00FF)
\endcode

Le modalit&agrave; di acquisizione specificata dalla parte
bassa pu&ograve; assumere i valori specificati dalle seguenti define: <BR>
#MUX_MODE_10_NOFPN_DX <BR>
#MUX_MODE_10_NOFPN_SX <BR>
#MUX_MODE_8_NOFPN <BR>
#MUX_MODE_8_FPN <BR>
#MUX_MODE_8_FPN_ODC <BR>
#MUX_MODE_8_FPN_ODC_SOBEL <BR>
#MUX_MODE_8_FPN_ODC_DISP <BR>
#MUX_MODE_8_FPN_ODC_DISP_SOBEL <BR>
#MUX_MODE_8_FPN_ODC_MEDIAN_DISP <BR>
#MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL <BR>
#MUX_MODE_8_FPN_ODC_DISP_SOBEL_SX <BR>
#MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL_SX

Altro modo per capire la modalit&agrave; a partire dalla variabile acq_mode e dalle define 
appena elencate &egrave; data dall'uso di un confronto del tipo 
\code
(acq_mode != (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100))
\endcode
*/

unsigned short start_stop; //!< Modified in the command "disconnect" in commands.cpp but not used in other places.
unsigned short dac_sensor; //!< Initialized in var_init() but not used in other places.
unsigned char *input_test; //!< see "testin" command in both commands.cpp and serial_port.cpp
unsigned char input_test0; //!< used by "testin" command in commands.cpp and updated by input_loop0() in loops.cpp
//!< \warning Usage of this variable seems to be deprecated instead use the proper register in #regs
unsigned char input_test1; //!< used by "testin" command in commands.cpp and updated by input_loop1() in loops.cpp
//!< \warning Usage of this variable seems to be deprecated instead use the proper register in #regs

bool wd_check; //!< watchdog flag.

unsigned char slave_id=0;    //!< Used in widegate mode by each sensor in the chain to replay (SNP_reply()) a message (coming from the previous in the chain) to the next sensor (this value is always 0 and should correspond to the 24th element of the params vector).
//unsigned char count_debug=0; //!< ??? 20100520 commented

extern unsigned char total_sys_number; 
extern unsigned char current_sys_number;

// 20100521 eVS aggiornati commenti
unsigned char current_sys_number_tmp=0; //!< Used together #total_sys_number_tmp for the wideconfiguration initialization (after receiving wideconfiguration activation till end_chain command arrives).
/*!<
Vedi commands.cpp ("wideconfiguration") e serial.cpp ("wideconfiguration" e "end_chain").
*/
unsigned char total_sys_number_tmp=0;   //!< Used together #current_sys_number_tmp for the wideconfiguration initialization (after receiving wideconfiguration activation till end_chain command arrives).
/*!<
Vedi commands.cpp ("wideconfiguration") e serial.cpp ("wideconfiguration" e "end_chain").
*/

extern unsigned char count_sincro;      //!< Per la sincronizzazione di piu' PCN in wide-gate.
extern unsigned char *data_wide_gate;
unsigned char send_enable=0;  //!< Se sono in widegate e sono il master uso send_enable per regolare l'accesso alla seriale (send_enable deve essere zero per gli slave).

int pcin0;    //!< File descriptor associato alla prima porta seriale di input.
int pcin1;    //!< File descriptor associato alla seconda porta seriale di input.
int pcout0;   //!< File descriptor associato alla prima porta seriale di output.
int pcout1;   //!< File descriptor associato alla seconda porta seriale di output.
int watchdog; //!< File descriptor associato al dispositivo "/dev/watchdog".

//FILE *in,*out; // sostituite variabili globali con variabili locali dove serviva
//FILE *platelist; //??? DICHIARATA MA NON UTILIZZATA ???.

struct timeval start0; //!< Per controllare timeout0
struct timeval stop0;  //!< Per controllare timeout0
struct timeval start1; //!< Per controllare timeout1
struct timeval stop1;  //!< Per controllare timeout1

//int debug; //??? DICHIARATA MA NON UTILIZZATA ???.



/********************* wide-gate variables **************************/
#define time_bkg //!< Per abilitare la compilazione del codice relativo all'aggiornamento automatico del background.

unsigned long people_rec[2];  //!< Vettore contenente i due contatori delle persone entrare/uscite che devono essere inseriti nel messaggio di log.

extern unsigned long people[2]; 
extern unsigned char det_area;
extern int inst_height;       ///< Altezza d'installazione ( in cm)
extern int inst_dist;         ///< Distanza tra due dispositivi in modalita widegate ( in cm)

unsigned char flag_serial;    //!< Flag associato alla comunicazione seriale di tipo master-slave.
unsigned char flag_wg=1;      //!< Flag che indica se &egrave; presente la configurazione wide-gate, cio&egrave; se &egrave; presente almeno uno slave.
unsigned char flag_wg_count=0;//!< Slave presence flag.
unsigned char wg_check;       //!< Abilita/disabilita il controllo della presenza della configurazione wide-gate (DA VERIFICARE) ???

unsigned char * SNPimg;

extern unsigned char frame_cnt_door;
extern unsigned char frame_fermo;



/********************* record.txt variables **************************/

int recordfd;           //!< File descriptor del file contenente i messaggi di log.
char record_fn[256];    //!< Nome del file contenente i messaggi di log.
unsigned long record_id;//!< Indice del file contenente i messaggi di log.



/************************** threads **********************************/

pthread_t serloopttyS0;   //!< Posix thread associato alla funzione ser_loopttyS0() per la gestione della porta seriale numero 0.
pthread_t serloopttyS1;   //!< Posix thread associato alla funzione ser_loopttyS1() per la gestione della porta seriale numero 1.
pthread_mutex_t rdlock;   //!< Semaforo di tipo posix utilizzato per poter accedere in modo esclusivo alla risorsa #records[].
pthread_mutex_t mainlock; //!< Semaforo di tipo posix utilizzato per poter accedere in modo esclusivo ad una variabile globale da thread concorrenti.
pthread_mutex_t acq_mode_lock; // 20100517 eVS

// (reset_counters function and "serial" command)
pthread_t ploop;
pthread_t mloop; //!< Posix thread associato alla funzione main_loop() che contiene il ciclo principale di esecuzione.
pthread_t tloop;
pthread_t rdloop;

pthread_t inloop0;
pthread_t inloop1;

pthread_t wdloop; //!< Posix thread associato al ciclo di controllo "watchdog".




/**************** serial port variables **********************************/

// 20100520 eVS comments added for both ttyS0 and ttyS1
int ttyS0; //!< File descriptor corresponding to the serial port 0 (set to -1 if inactive).
/*!< 
This port is used by a PCN to communicate with its master (which can be the
rs485_gui or another PCN in wideconfiguration that precedes this one in 
the chain or the collector)
*/
int ttyS1; //!< File descriptor corresponding to the serial port 1 (set to -1 if inactive).
/*!< 
This port is used by a PCN to communicate with its slave (which can only be
another PCN in wideconfiguration that follows this one in the chain)
*/


//struct termios newtio; 20100521 eVS commented because not used
// 20100521 eVS added comments for framecounter and test_serial
extern unsigned long int framecounter; //!< Used to count acquired frames.
/*!<
Initialized to zero and then incremented by one at each new acquisition.
It is used to start some processing every N frame just using the % 
operator, e.g., if (framecounter % N == 0).
*/
unsigned char test_serial; //!< Used from the win_client (via socket) to test the serial port at the slave side (via #ttyS1)



/*******************  parameters *****************************************/

/*!
\var parm_names
\brief Nome dei parametri relativi alle porte seriali, no tracking zone e motion detection (\ref tabella_parms).
*/
char parm_names[256][PARM_STR_LEN] = {
    "outtime0",
    "outtime1",
    "input0",
    "input1",
    "sled",
    "dir",
    "threshold",
    "serial_id",
    "serial_br",
    "serial_db",
    "serial_pr",
    "serial_sb",
    "detect_area",
    "autoled",
    "inst_height",
    "serial_ms",
    "serial_sid",
    "serial_sbr",
    "serial_sdb",
    "serial_spr",
    "serial_ssb",
    "timebkg",
    "staticth",
    "slave_id",
    "sx_dx",
    "inst_dist",
    "wg_check",
    "sys_number",
    "sys_number_index",
    "sxlimit",
    "dxlimit",
    "sxlimit_riga_start",
    "dxlimit_riga_start",
    "sxlimit_riga_end",
    "dxlimit_riga_end" ,
    "cond_diff_1p",
    "diagnostic_en",
    "move_det_col0",
    "move_det_col1",
    "move_det_row0",
    "move_det_row1",
    "move_det_alfareg",
    "move_det_thr",
    "move_det_en",
    "door_stairs_en",
    "up_line_limit",
    "down_line_limit",
    "dis_autobkg",     // 20090506 Lisbona
    "door_size",       // 20130411 eVS, door_size instead of door_kind (for 2.3.10.7)
    "handle_oor",       // 20130715 eVS added to manage OOR situation after reboot
    "auto_gain"         // 20130927 eVS added to manage gain Vref
};

/*!
\var calib_parm_names
\brief Nomi dei parametri di calibrazione (\ref tabella_calib).
*/
char calib_parm_names[256][PARM_STR_LEN] = {
    "map_minth",
    "map_thuni",
    "map_disp",
    "dac_vref1",
    "dac_vprec1",
    "dac_vgap1",
    "dac_vref2",
    "dac_vprec2",
    "dac_vgap2",
    "threshBkg",
    "step225_0",
    "step225_1",
    "step225_2",
    "step225_3",
    "step225_4",
    "step225_5",
    "step225_6",
    "step225_7",
    "step225_8",
    "step225_9",
    "step225_A",
    "step225_B",
    "step225_C",
    "step225_D",
    "step225_E",
    "step225_F",
    "winsz",
    "step240_0",
    "step240_1",
    "step240_2",
    "step240_3",
    "step240_4",
    "step240_5",
    "step240_6",
    "step240_7",
    "step240_8",
    "step240_9",
    "step240_A",
    "step240_B",
    "step240_C",
    "step240_D",
    "step240_E",
    "step240_F"
};

/*!
\var parm_values
\brief Valori dei parametri relativi alle porte seriali, no tracking zone e motion detection (vedi \ref tabella_parms).
*/
unsigned short parm_values[256];

//! Valori di default dei parametri relativi alle porte seriali, no tracking zone e motion detection (vedi \ref tabella_parms)
unsigned short default_values[256] = {
    OUTTIME0,
    OUTTIME1,
    INPUT0,  
    INPUT1,  
    SLED,
    DIR,
    THRESHOLD,
    SERIAL_ID,
    SERIAL_BR,
    SERIAL_DB,
    SERIAL_PR,
    SERIAL_SB,
    DETECT_AREA,
    OFF,
    INST_HEIGHT,
    SERIAL_MS,
    SERIAL_SID,
    SERIAL_SBR,
    SERIAL_SDB,
    SERIAL_SPR,
    SERIAL_SSB,
    TIMEBKG,
    STATIC_TH,
    SLAVE_ID,
    SX_DX,
    INST_DIST,
    WG_CHECK,
    SYS_NUMBER,
    SYS_NUMBER_INDEX,
    SXLIMIT,
    DXLIMIT,
    SXLIMIT_RIGA_START,
    DXLIMIT_RIGA_START,
    SXLIMIT_RIGA_END,
    DXLIMIT_RIGA_END,
    OFF,
    DIAGNOSTIC_EN,
    MOVE_DET_COL0,
    MOVE_DET_COL1,
    MOVE_DET_ROW0,
    MOVE_DET_ROW1,
    MOVE_DET_ALFAREG,
    MOVE_DET_THR,
    MOVE_DET_EN,
    DOOR_STAIRS_EN,
    UP_LINE_LIMIT,
    DOWN_LINE_LIMIT,
    DIS_AUTOBKG,   //20090506 Lisbona
    DOOR_SIZE,      //20130411 eVS, DOOR_SIZE instead of DOOR_KIND (for 2.3.10.7)
    HANDLE_OOR, // 20130715 eVS, in order to manage OOR correctly after reboot
    AUTO_GAIN   // 20130927 eVS added to manage gain Vref
    };



/*!
\var calib_parm_values
\brief Valori dei parametri di calibrazione (\ref tabella_calib).
*/
unsigned short calib_parm_values[256];


/*!
\var calib_default_values
\brief Valori di default dei parametri di calibrazione (\ref tabella_calib).
*/
unsigned short calib_default_values[256] = {
    MAP_MINTH,
    MAP_THUNI,
    MAP_DISP,
    DAC_VREF,
    DAC_VPREC,
    DAC_VGAP,
    DAC_VREF,
    DAC_VPREC,
    DAC_VGAP,
    THBKG,
    STEPS_225[0],
    STEPS_225[1],
    STEPS_225[2],
    STEPS_225[3],
    STEPS_225[4],
    STEPS_225[5],
    STEPS_225[6],
    STEPS_225[7],
    STEPS_225[8],
    STEPS_225[9],
    STEPS_225[10],
    STEPS_225[11],
    STEPS_225[12],
    STEPS_225[13],
    STEPS_225[14],
    STEPS_225[15],
    WIN_SIZE,
    STEPS_240[0],
    STEPS_240[1],
    STEPS_240[2],
    STEPS_240[3],
    STEPS_240[4],
    STEPS_240[5],
    STEPS_240[6],
    STEPS_240[7],
    STEPS_240[8],
    STEPS_240[9],
    STEPS_240[10],
    STEPS_240[11],
    STEPS_240[12],
    STEPS_240[13],
    STEPS_240[14],
    STEPS_240[15]
};

/****************************   gpios strucures   ************************/
/*!
\struct reg_info
\brief Struttura dati contenente informazioni relative alle porte di input/output di tipo geneal purpose.
*/
struct reg_info {
    char *name; //!< nome registro di memoria
    unsigned int addr; //!< indirizzo registro di memoria
    int shift; //!< traslazione espressa in bits
    unsigned int mask; //!< maschera
    char type; //!< tipo
    char *desc; //!< descrizione
};

/*!
\var static struct reg_info regs[]
\brief Informazioni dei registri di tipo general purpose.

Dopo Lisbona sono stati aggiunti gli ultimi due record "GPLR2_095" e "GPLR3_096" che sono
stati utilizzati al posto delle variabili globali #input_test0 e #input_test1
per verificare lo stato dei digital input.

\date February 2009 (after Lisbon)
*/
static struct reg_info regs[] = {
    //GPIO_100 RS485 RTS pin
    { "GPLR3_100", 0x40E00100,  4, 0x00000001, 'd', "GPIO 100 level" },
    { "GPSR3_100", 0x40E00118,  4, 0x00000001, 'd', "GPIO 100 set" },
    { "GPCR3_100", 0x40E00124,  4, 0x00000001, 'd', "GPIO 100 clear" },
    { "GPDR3_100", 0x40E0010C,  4, 0x00000001, 'd', "GPIO 100 i/o direction (1=output)" },
    { "FFLSR_TEMT", 0x40100014,  6, 0x00000001, 'd', "FFUART LSR transmitter empty" },
    { "BTLSR_TEMT", 0x40200014,  6, 0x00000001, 'd', "BTUART LSR transmitter empty" },
    { "STLSR_TEMT", 0x40700014,  6, 0x00000001, 'd', "STUART LSR transmitter empty" },
    { "IER_RTOIE", 0x40700004,  7, 0x00000001, 'd', "IER Receiver Timeout Interrupt Enable" },
    { "IIR_TOD", 0x40700008,  3, 0x00000001, 'd', "IER Receiver Timeout Interrupt Enable" },
    { "FOR_BC", 0x40700024,  0, 0x0000003F, 'd', "IER Receiver Timeout Interrupt Enable" },
    { "GPLR3_091", 0x40E00008,  27, 0x00000001, 'd', "GPOUT0 level" },
    { "GPSR3_091", 0x40E00020,  27, 0x00000001, 'd', "GPOUT0 set" },
    { "GPCR3_091", 0x40E0002C,  27, 0x00000001, 'd', "GPOUT0 clear" },
    { "GPLR3_090", 0x40E00008,  26, 0x00000001, 'd', "GPOUT1 level" },
    { "GPSR3_090", 0x40E00020,  26, 0x00000001, 'd', "GPOUT1 set" },
    { "GPCR3_090", 0x40E0002C,  26, 0x00000001, 'd', "GPOUT1 clear" },
    { "GPLR2_095", 0x40E00008,  31, 0x00000001, 'd', "GPIN1 level" },  //20090506 Lisbona
    { "GPLR3_096", 0x40E00100,   0, 0x00000001, 'd', "GPIN0 level" }   //20090506 Lisbona
};



/******************   tracking extern variables   ************************/
extern unsigned char Bkgvec[NN];
//extern unsigned char Pasvec[NN];
extern int svec[NN];
extern unsigned char Bkgvectmp[NN];
extern bool ev_door_open;
extern bool ev_door_close;
extern bool ev_door_open_rec;
extern bool ev_door_close_rec;
extern bool mem_door;
//extern unsigned long door_in;  // 20100702 eVS moved in io.cpp where used
//extern unsigned long door_out; // 20100702 eVS moved in io.cpp where used
extern unsigned char vm_img;
extern unsigned char vm_bkg;
extern unsigned char soglia_bkg;

extern unsigned char persdata[54];

extern int static_th;

extern unsigned char minuti_th;
extern unsigned char limitSx;
extern unsigned char limitDx;
extern unsigned char limitSx_riga_start;
extern unsigned char limitDx_riga_start;
extern unsigned char limitSx_riga_end;
extern unsigned char limitDx_riga_end;
extern unsigned char draw_detected;
extern unsigned char limit_line_Up;
extern unsigned char limit_line_Down;

extern int diff_cond_1p;
unsigned char auto_gain; // 20130927 eVS, manage gainn vref
//20090506 Lisbona
extern unsigned char autobkg_disable;

//20100419 eVS
extern unsigned char door_size;  // 20130411 eVS, door_size instead of door_kind (for 2.3.10.7)
extern unsigned char handle_oor; // 20130715 eVS, manage OOR after system reboot
int count_check_counter;

extern unsigned char mov_dect_0_7l;
extern unsigned char mov_dect_8_15l;
extern unsigned char mov_dect_15_23l;
extern unsigned char mov_dect_0_7r;
extern unsigned char mov_dect_8_15r;
extern unsigned char mov_dect_15_23r;

extern bool count_true_false;

int mov_det_left; //!< Parametro di motion detection relativo al frame sinistro.
/*!<
Viene calcolato nel main_loop() concatenando i tre byte estratti da get_images() 
e dividendo per 100:<BR>
((((#mov_dect_15_23l & 0xff) << 16) | ((#mov_dect_8_15l & 0xff) << 8) | (#mov_dect_0_7l & 0xff))/100)
*/

int mov_det_right; //!< Parametro di motion detection relativo al frame destro.
/*!<
Viene calcolato nel main_loop() concatenando i tre byte estratti da get_images() 
e dividendo per 100:<BR>
((((#mov_dect_15_23r & 0xff) << 16) | ((#mov_dect_8_15r & 0xff) << 8) | (#mov_dect_0_7r & 0xff))/100)
*/

int default_vref_left; //!< Valore di default della vref di sinistra letta da #calib_parm_values mediante la calib_get_parms().
int default_vref_right;//!< Valore di default della vref di destra letta da #calib_parm_values mediante la calib_get_parms().
int current_vref_left; //!< Valore di corrente della vref di sinistra modificata dalla funzione main_loop() (in loops.cpp).
int current_vref_right;//!< Valore di corrente della vref di destra modificata dalla funzione main_loop() (in loops.cpp).

//extern unsigned char move_det_en;
unsigned char move_det_en;

// 20111207 eVS, changed data type avoiding redundancy (now error_pcn_status_code is useless because it is encoded inside the pcn_status itself)
//bool pcn_status; //!< Stato corrente del PCN: 1->ok, 0->riscontrati problemi di diagnostica.
//bool old_pcn_status; //!< Used by record_counter() to understand a diagnostic error message has to be written in the log.
unsigned char pcn_status; //!< Stato corrente del PCN: 0->ok, N->riscontrati problemi di diagnostica.
unsigned char old_pcn_status; //!< Used by record_counter() to understand which diagnostic error message has to be written in the log.
//unsigned char error_pcn_status_code; //!< Contain the diagnosis error code to be written in the log file (see record_counter() and main_loop() code).
/*!<
  From the least significant bit to the most significant one, the meaning of each bit is:
  - average gray level of the right image too dark (obscured right camera)
  - average gray level of the left image too dark (obscured left camera)
  - average gray levels of the two camera are contrasting (high difference)
  - the vref of the two cameras are contrasting (high difference): this can happen when auto-vref is running
  The other bits have no meaning.
  
  If an error occurs it is written in the log file (see record_counters() and main_loop() code).
*/
unsigned char diagnostic_en; //!< Global variable corresponding to the parameter "diagnostic_en" in #parm_values

int move_det_thr; //!< Soglia di motion detection per capire se c'&egrave; stato del movimento nella scena confrontando questo parametro con i valori di #mov_det_left e #mov_det_right.

extern unsigned char door_stairs_en;


/*!
\brief Funzione pricipale dell'imgserver.

Dopo una serie di settaggi iniziali (var_init() per le variabili e init_io() per i dispositivi e 
canali di comunicazione I/O) e dopo aver caricato dei parametri da file mediante la load_parms() e la calib_load_parms(), 
la funzione main contiene un ciclo in cui l'imgserver resta in ascolto di eventuali comandi inviati 
dal win_client (mediante socket TCP). 

Inoltre viene attivato il ciclo principale di elaborazione main_loop() mediante 
la chiamata a mainloop_enable() (in loops.cpp): Per ogni mappa di disparit&agrave; (calcolata 
direttamente dall'FPGA) viene eseguita una serie di pre-filtraggi (sottrazione dello sfondo
ed operazioni morfologiche), dopo di che viene chiamato l'algoritmo di people 
detection, che raffina ulteriormente la mappa di disparit&agrave; senza sfondo (vedi la
funzione detectAndTrack() in peopledetection.cpp), e successivamente l'algoritmo di
tracking (vedi la funzione track() in peopletrack.cpp) che esegue il blob matching, a
partire dal risultato dell'algoritmo precedente, disegnando le croci sulle teste delle 
persone che si muovono all'interno della zona monitorata al fine di contare i passeggeri
che entrano e che ed escono. 

Segue, quindi, il ciclo principale di esecuzione in cui l'imgserver resta in ascolto 
di eventuali comandi inviati dal win_client.

All'uscita del ciclo viene eseguita la deallocazione delle risorse (deinitpeople()), la cancellazione dei thread 
e la chiusura dei dispositivi (deinit_io()).
*/
int main(int argc, char ** argv)
{
    fd_set master;   			//master file descriptor list
    fd_set read_fds; 			//temp file descriptor list
    int newfd;
    char buf[MAX_STR_LENGTH];
    int nbytes;
    int yes=1;
    socklen_t addrlen;
    int i;
    int c;
    static struct option long_options[] =
    {
        {"version",no_argument,0,'v'},
        {"dir",required_argument,0,'d'},
        {0, 0, 0, 0}
    };

    int option_index = 0;

    /*! \code
    // Esegue il parsing della stringa di comando (per esempio: imgserver --version oppure imgserver -v)
    c = getopt_long(argc, argv, "v:d",long_options, &option_index);
    \endcode */

    c = getopt_long(argc, argv, "v:d",long_options, &option_index);
    
    switch (c)
    {
    case 'v' :
        printf("System: %s. Daemon version: %s\n",SYSTEM,VERSION);
        return 0;
    case 'd' :
        if(optarg)
        {
            sprintf(working_dir,"%s/",optarg);
        }
        break;
    }   

    // check if ip was changed                              
    {
      char filename[255];
      sprintf(filename, "%sip.txt", working_dir);
      FILE *ip = fopen(filename,"r");
      if (ip)
      {           
        // change ip       
        char command[255];
        fgets(command, 255, ip);
        fclose(ip);            
        system(command);
        print_log("%s\n", command);
               
        // remove ip file
        sprintf(command, "rm -f %s", filename);
        system(command);        
      }
    }
        
    /*! 
    <b>Inizializzazioni variabili e dispositivi</b>
    \code
    // settaggio variabili globali
    var_init();	
    			
    input_function0 = do_nothing;
    input_function1 = do_nothing;
    
    // inizializzazione dispositivi
    if(init_io() < 0) exit(0);		
    \endcode 
    
    <b>Caricamento parametri</b>    
    \code 
    // caricamento dei parametri della porta seriale, no tracking zone e motion detection
    load_parms();	
    		
    // caricamento dei parametri di calibrazione
    calib_load_parms();			
    \endcode 
    */
        
    var_init();				// setting global variables

    input_function0 = do_nothing;
    input_function1 = do_nothing;

    if(init_io() < 0) exit(0);		// initializing  devices

    load_parms();			// loading saved parameters
    calib_load_parms();			// loading saved calibration parameters
   
    gettimeofday(&start0,NULL);
    gettimeofday(&start1,NULL);

    memset(ODCbuf,0,sizeof(ODCbuf));

    /*! \code
    // get calibration vref value
    default_vref_left = calib_get_parms("dac_vref2");
    default_vref_right = calib_get_parms("dac_vref1");
    current_vref_left = default_vref_left;
    current_vref_right = default_vref_right;
    \endcode */

    // get calibration vref value
    default_vref_left = calib_get_parms("dac_vref2");
    default_vref_right = calib_get_parms("dac_vref1");
    current_vref_left = default_vref_left;
    current_vref_right = default_vref_right;

    /*!
    <b>Caricamento dello sfondo della scena da file ascii</b>
    \code
    if((in = fopen(bg_filename,"rb")))
    {
        // Bkgvec contiene lo sfondo (o meglio una media di n immagini di sfondo)
        fread(Bkgvec,sizeof(Bkgvec),1,in);
        // svec contiene la deviazione standard di ciascun pixel dal valor medio dello sfondo
        fread(svec,sizeof(svec),1,in);
        // vm_bkg rappresenta il valor medio dello sfondo
        fread(&vm_bkg,sizeof(vm_bkg),1,in);
        ...
    \endcode 
    */

    // loading scene background image
    FILE *in;
    if((in = fopen(bg_filename,"rb")))
    {
        fread(Bkgvec,sizeof(Bkgvec),1,in);
        fread(svec,sizeof(svec),1,in);
        fread(&vm_bkg,sizeof(vm_bkg),1,in);
        fclose(in);
    }

    // copio il background caricato nella variabile usata per l'auto background
    memcpy(Bkgvectmp,Bkgvec,NN);

    //---------------people tracking init------------------//
    unsigned long people[2];
    imagesize = NN << 2;
    people[people_dir] = counter_in;  // la load_counters carica i valori in counter_in e counter_out
    people[1-people_dir] = counter_out;  // la load_counters carica i valori in counter_in e counter_out
    //initpeople(people[0],people[1]);
    initpeople(people[0],people[1], total_sys_number, num_pers);

    //------------salvataggio log riavvio-------------------//
    time_t curtime_reb;
    struct tm *loctime_reb;
    char reb_str[255];
    char comando[200];
    FILE *record_reb;
    unsigned long lines;

    curtime_reb = time (NULL);
    loctime_reb = localtime (&curtime_reb);
    sprintf(reb_str,"Boot\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n",
        loctime_reb->tm_mday,loctime_reb->tm_mon+1,1900+loctime_reb->tm_year,
        loctime_reb->tm_hour,loctime_reb->tm_min,loctime_reb->tm_sec,
        counter_in, counter_out);
    record_reb = fopen("/var/neuricam/syslog.txt","a+");
    fseek(record_reb,0L,SEEK_END);
    lines = ftell(record_reb)/MED_LINE_LEN;

    if(lines >= MAX_LOG_LINES)
    {
        fclose(record_reb);	// the records file is full.
        sprintf(comando,"rm /var/neuricam/syslog.txt");
        system(comando);	// removing the oldest file
        record_reb = fopen("/var/neuricam/syslog.txt","a+");
    }

    fprintf(record_reb,"%s",reb_str);   
    fclose(record_reb);
    
    // 20110914 eVS - Add boot string in records<N>.txt
    { 
        FILE* recordfd = fopen(record_fn,"a+");
        if (recordfd)
        {
            fseek(recordfd,0L,SEEK_END);
            fprintf(recordfd, "%s", reb_str);
            
            /*sprintf(reb_str,"Diagnostic enabled: %d\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n", diagnostic_en,
                loctime_reb->tm_mday,loctime_reb->tm_mon+1,1900+loctime_reb->tm_year,
                loctime_reb->tm_hour,loctime_reb->tm_min,loctime_reb->tm_sec,
                counter_in, counter_out);                
            fprintf(recordfd, "%s", reb_str);*/
            
            fclose(recordfd);
        }
    }

    ////////////////////
    // 20091120 eVS    
    //{    
    /*    int r0  = get_parms("move_det_row0");
        int r1  = get_parms("move_det_row1");
        int c0  = get_parms("move_det_col0");
        int c1  = get_parms("move_det_col1");
        int thr = get_parms("move_det_thr");
        //sprintf(reb_str, "Init: md_row0=%d\t md_row1=%d\t md_col0=%d\t md_col1=%d\t md_thr=%d\n", r0, r1, c0, c1, thr);
    */
    //}
    //fprintf(record_reb,"%s",reb_str);
    // 20091120 eVS    
    ////////////////////
    //print_log("Init:\n\t md_r0=%d\t md_r1=%d\t md_c0=%d\t md_c1=%d\t md_thr=%d\n", r0, r1, c0, c1, thr);


    //20090506 Lisbona  20111010 eVS moved after widegate initialization
    //if((input_function0 == enable_counting && get_gpio("GPLR3_096")==0) || (input_function1 == enable_counting && get_gpio("GPLR2_095")==0)) 
    //  count_enabled=0; //stopping the counting 

    sockfd = 0;
    FD_ZERO(&master); //// This function initializes the file descriptor set to contain no file descriptors.
    FD_ZERO(&read_fds);

    /*! 
    <B>Funzione SOCKET (di tipo TCP)</B>
    \code
    // funzione che permette di creare una socket per la comunicazione client/server
    // Provvede un canale di trasmissione dati bidirezionale, sequenziale e affidabile. 
    // Opera su una connessione con un altro socket. I dati vengono ricevuti e trasmessi 
    // come un flusso continuo di byte (da cui il nome stream).
    listener = socket(AF_INET, SOCK_STREAM, 0);
    \endcode */
    
    if ((listener = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        print_log("Exit (socket)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    /*! 
    <B>Funzione SETSOCKOPT</B>
    \code
    // funzione che permette di settare le opzioni di una socket
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes,sizeof(int));
    \endcode */
    
    if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes,sizeof(int)) == -1)
    {
        perror("setsockopt");
        print_log("Exit (setsockopt)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = INADDR_ANY;
    myaddr.sin_port = htons(PORT);
    memset(&(myaddr.sin_zero), '\0', 8);

    /*! 
    <B>Funzione BIND</B>
    \code
    // Con bind si puo' assegnare un indirizzo IP specifico ad un socket, 
    // purche' questo appartenga ad una interfaccia della macchina. 
    // Per un client TCP questo diventera' l'indirizzo sorgente usato 
    // per i tutti i pacchetti inviati sul socket, mentre per un server TCP 
    // questo restringera' l'accesso al socket solo alle connessioni 
    // che arrivano verso tale indirizzo.
    // Normalmente un client non specifica mai l'indirizzo di un socket, 
    // ed il kernel sceglie l'indirizzo di origine quando viene effettuata 
    // la connessione, sulla base dell'interfaccia usata per trasmettere i pacchetti, 
    // (che dipendera' dalle regole di instradamento usate per raggiungere il server). 
    // Se un server non specifica il suo indirizzo locale il kernel usera' 
    // come indirizzo di origine l'indirizzo di destinazione specificato dal SYN del client.
    bind(listener, (struct sockaddr *)&myaddr, sizeof(myaddr));
    \endcode */
    
    if (bind(listener, (struct sockaddr *)&myaddr, sizeof(myaddr)) == -1)
    {
        perror("bind");
        print_log("Exit (bind)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    /*! 
    <B>Funzione LISTEN</B>
    \code
    // La funzione listen serve ad usare un socket in modalita' passiva, cioe', come dice il nome, 
    // per metterlo in ascolto di eventuali connessioni; in sostanza l'effetto della funzione e' di 
    // portare il socket dallo stato CLOSED a quello LISTEN. In genere si chiama la funzione in un 
    // server dopo le chiamate a socket e bind e prima della chiamata ad accept.
    listen(listener, MAX_CONN);
    \endcode */
    
    if (listen(listener, MAX_CONN) == -1)
    {
        perror("listen");
        print_log("Exit (listen)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    FD_SET(listener, &master); //// This function adds a file descriptor to a file descriptor set.
    fdmax = listener;

    /*! 
    <b>Socket di tipo UDP</b>
    \code
    // Viene usato per trasmettere pacchetti di dati (datagram) di lunghezza massima prefissata, 
    // indirizzati singolarmente. Non esiste una connessione e la trasmissione e' effettuata
    // in maniera non affidabile.
    datagram = socket(AF_INET, SOCK_DGRAM, 0);
    \endcode */
    
    // 20111014 eVS, datagram creation moved in the ping_pong loop code
    // datagram socket
    /*if ((datagram = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        perror("socket");
        exit(1);
    }

    myaddr_data.sin_family = AF_INET;
    myaddr_data.sin_addr.s_addr = INADDR_ANY;
    myaddr_data.sin_port = htons(PORT);

    memset(&(myaddr_data.sin_zero), '\0', 8);*/

    /*! \code
    bind(datagram, (struct sockaddr *)&myaddr_data, sizeof(myaddr_data);
    \endcode */
    
    // 20111014 eVS, datagram bind moved in the ping_pong loop code
    /*if (bind(datagram, (struct sockaddr *)&myaddr_data, sizeof(myaddr_data)) == -1)
    {
        perror("bind");
        exit(1);
    }*/
    
    /*! 
    <B>Lancio dei threads e inizializzazione semafori</B>
    \code
    mainloop_enable(1); //starting the acquisition thread
    pthread_mutex_init(&mainlock,NULL);
    pthread_create (&wdloop, NULL, watchdog_loop, NULL);
    pthread_create(&rdloop, NULL, record_loop, NULL);
    pthread_mutex_init(&rdlock,NULL);
    pthread_create (&inloop0, NULL, input_loop0, NULL);
    pthread_create (&inloop1, NULL, input_loop1, NULL);
    \endcode */

    pthread_mutex_init(&acq_mode_lock,NULL); // 20100518 eVS
    pthread_mutex_init(&mainlock,NULL);
    pthread_mutex_init(&rdlock,NULL);
      
    // 20100603 eVS moved here
    if (total_sys_number > 1) {
        count_sincro=0;  // initialize the clock-syncro
        
        current_sys_number_tmp = current_sys_number;
        total_sys_number_tmp = total_sys_number;
        
        if(current_sys_number_tmp!=total_sys_number_tmp)
        {
            save_parms("serial_sbr",B230400); //B921600
            write_parms("serial_sbr",B230400);

            save_parms("serial_sdb",CS8);
            write_parms("serial_sdb",CS8);

            save_parms("serial_spr",0);
            write_parms("serial_spr",0);

            save_parms("serial_ssb",0);
            write_parms("serial_ssb",0);
        }

        // 20111010 eVS bugfix, at boot the saved configuration about input0 has already been rightly loaded
        // we just commented the following part
        
        /*if(current_sys_number_tmp!=1)
        {
            write_parms("input0",3);
            save_parms("input0",3);
        }
        else if (current_sys_number_tmp==1)
        {   
            if (get_parms("input0") != 0 && get_parms("input0") != 2)
            {
              write_parms("input0",0);
              save_parms("input0",0);
            }
        }
    
        write_parms("input1",3);
        save_parms("input1",3);*/

        write_parms("outtime0",4); //disable FPGA in optoO
        save_parms("outtime0",4);
        write_parms("outtime1",4); //disable FPGA in optoO
        save_parms("outtime1",4);

        // 20111010 eVS by default in widegate the system starts counting
        count_enabled=1;
        set_gpio("GPSR3_090",1);	//enable slave people counting
                  
        if(current_sys_number_tmp>1)
        {
            write_parms("serial_br",B230400);
            save_parms("serial_br",B230400);

            write_parms("serial_db",CS8);
            save_parms("serial_db",CS8);

            write_parms("serial_pr",0);
            save_parms("serial_pr",0);

            write_parms("serial_sb",0);
            save_parms("serial_sb",0);
        }

        if(current_sys_number_tmp==total_sys_number_tmp)
        {
            write_parms("outtime1",200);
            save_parms("outtime1",200);
            
            // 20111010 eVS bugfix, at boot the saved configuration about input0 has already been rightly loaded
            // we just commented the following part
            
            /*write_parms("input0",3);
            save_parms("input0",3);
            
            //initpeople(0,0);
            //initpeople(people[0],people[1]);*/
        }

        if(current_sys_number_tmp==1)
        {
            write_parms("outtime0",200);
            save_parms("outtime0",200);
            
            // 20111010 eVS bugfix, at boot the saved configuration about input0 has already been rightly loaded
            // we just commented the following part
            
            /*write_parms("input1",3);
            save_parms("input1",3);

            usleep(500000);
            unsigned char value=get_parms("autoled");
            SNP_Send(slave_id,"autoled",(char *)&value,sizeof(value),ttyS1);
            usleep(50000);
            value=get_parms("threshold");
            SNP_Send(slave_id,"threshold",(char *)&value,sizeof(value),ttyS1);
            usleep(50000);
            value=get_parms("dir");
            SNP_Send(slave_id,"dir",(char *)&value,sizeof(value),ttyS1);
            usleep(50000);
            value=get_parms("detect_area");
            SNP_Send(slave_id,"detect_area",(char *)&value,sizeof(value),ttyS1);  
            */
        }
        
        // 20111010 eVS added conters syncronization between master and the last slave (the one actually counting)
        if(current_sys_number_tmp==1)
        {
          unsigned long counters[2];
          counters[0] = counter_in;
          counters[1] = counter_out;         
          SNP_Send(get_parms("slave_id"), "gcounters", (char *)counters,sizeof(counters),ttyS0);
          usleep(TIMEOUT_485); //wait answer
        }
                
        if(current_sys_number_tmp==1) 
            send_enable=1;  // boot: se sono il master uso send_enable per evitare conflitti su seriale
        else 
            send_enable=0;  // boot: tutti gli slave non usano send_enable e quindi la pongono a zero

        framecounter=0;
    }    
    
    //20090506 Lisbona 20111010 eVS moved here from above because this code has to be placed after widegate initialization
    if((input_function0 == enable_counting && get_gpio("GPLR3_096")==0) || (input_function1 == enable_counting && get_gpio("GPLR2_095")==0)) 
    {
      //count_enabled=0; //stopping the counting  // 20101010 eVS commented because now there enable_counting(0)
     
      // 20101010 eVS added check for widegate as done in the input_loop
      if(total_sys_number>0 && current_sys_number!=total_sys_number) //se non sono l'ultimo
      {
        //faccio il ponte del segnale porta
        set_gpio("GPCR3_090",1);
      }
      enable_counting(0); // 20101010 eVS added this emulation of the enable_counting input_function
    } else {

      /*if(total_sys_number>0 && current_sys_number!=total_sys_number) //se non sono l'ultimo
      {//faccio il ponte del segnale porta
          if((input_function0 == enable_counting && get_gpio("GPLR3_096")!=0) ||
             (input_function1 == enable_counting && get_gpio("GPLR2_095")!=0) ||
             (input_function0 != enable_counting && input_function1 != enable_counting) 
              set_gpio("GPSR3_090",1);
          else
              set_gpio("GPCR3_090",1);
      }*/

      // 20101010 eVS added check for widegate as done in the input_loop
      if(total_sys_number>0 && current_sys_number!=total_sys_number) //se non sono l'ultimo
      {//faccio il ponte del segnale porta
        set_gpio("GPSR3_090",1);
      }
      enable_counting(1); // 20101010 eVS added this emulation of the enable_counting input_function
      ev_door_open_rec = true; // 20101104 eVS added to properly start counting
    }
      
    mainloop_enable(1); //starting the acquisition and processing thread
       
    pthread_create (&wdloop, NULL, watchdog_loop, NULL); //watchdog loop   
    pthread_create (&rdloop, NULL, record_loop, NULL);    
    pthread_create (&inloop0, NULL, input_loop0, NULL);
    pthread_create (&inloop1, NULL, input_loop1, NULL);
    
    //-----------------------------------------------------//

    /*! \code
    pthread_create (&ploop, NULL, ping_loop, NULL);
    \endcode */
    
    pthread_create (&ploop, NULL, ping_loop, NULL); 

    // 20100623 eVS moved above before threads creation
    // 20100603 eVS moved here
    /*if (total_sys_number > 1) {
        count_sincro=0;  // initialize the clock-syncro
        
        current_sys_number_tmp = current_sys_number;
        total_sys_number_tmp = total_sys_number;
        
        wide_gate_serial_parms_set();                
    }*/
    

    // ciclo principale di esecuzione in cui l'imgserver resta in ascolto di eventuali comandi inviati dal win_client
    for(;;)
    {              
        read_fds = master;
        if (select(fdmax+1, &read_fds, NULL, NULL,NULL) == -1)
        {
            perror("select");
            print_log("Exit (select)\n"); // 20120119 eVS, added to verify if exit
            exit(1);
        }

        for(i = 0; i <= fdmax; i++)
        {
            if (FD_ISSET(i, &read_fds)) 
            // This function returns a value for the file descriptor in the file descriptor set.
            // Returns a non-zero value if the file descriptor is set in the file descriptor 
            // set pointed to by fdset; otherwise returns 0.
            {

/*! 
<b>Funzione ACCEPT</b>
\code
//new connection
if (i == listener && !connected)
{
    addrlen = sizeof(remoteaddr);
    // La funzione accept e' chiamata da un server per gestire la connessione una volta 
    // che sia stato completato il three way handshake, la funzione restituisce un nuovo 
    // socket descriptor su cui si potra' operare per effettuare la comunicazione. 
    // Se non ci sono connessioni completate il processo viene messo in attesa.
    if ((newfd = accept(listener, (struct sockaddr *)&remoteaddr,&addrlen)) == -1)
    {
        ...
\endcode */
              
              if (i == listener && !connected)		//new connection
              {
                  addrlen = sizeof(remoteaddr);
                  if ((newfd = accept(listener, (struct sockaddr *)&remoteaddr,&addrlen)) == -1)
                  {
                      perror("accept");
                  }
                  else
                  {
                      FD_SET(newfd, &master); // add to master set
                      if (newfd > fdmax)
                      {
                          fdmax = newfd;
                      }
                      strcpy(addresses[newfd],inet_ntoa(remoteaddr.sin_addr));
                      printf("selectserver: new connection from %s on "
                          "socket %d\n", addresses[newfd], newfd);
                      sockfd = newfd;

/*! \code
// imgserver will send images to remoteaddr_data (see main_loop)
remoteaddr_data.sin_family = AF_INET;
remoteaddr_data.sin_addr.s_addr = remoteaddr.sin_addr.s_addr;
remoteaddr_data.sin_port = htons(UDP_PORT);
memset(&(remoteaddr_data.sin_zero), '\0', 8);
connected = true;
\endcode */

                      // imgserver will send images to remoteaddr_data (see main_loop)
                      remoteaddr_data.sin_family = AF_INET;
                      remoteaddr_data.sin_addr.s_addr = remoteaddr.sin_addr.s_addr;
                      remoteaddr_data.sin_port = htons(UDP_PORT);
                      memset(&(remoteaddr_data.sin_zero), '\0', 8);
                    
                      connected = true;
                  }
              }
              else if(i != listener && connected)
              {
/*! 
<b>Configurazione della modalit&agrave; di acquisizione</b>
\code
//old connection
else if(i != listener && connected)
{
if((nbytes = RecvString(i, buf,sizeof(buf))) <= 0)
{
    ...
    // questa funzione rimuove un file descriptor dal set di fd
    FD_CLR(i, &master);

    // configurazione della modalita' di acquisizione
    if(acq_mode != (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100))
    {
        acq_mode = MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100;
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
    }                        
    ...

    //starting the acquisition thread (if stopped)
    mainloop_enable(1);
    connected = false;  
    ...                      
\endcode */
                  
                  if((nbytes = RecvString(i, buf,sizeof(buf))) <= 0)
                  {
                      printf("imgserver: host %s has closed the connection\n", addresses[i]);

                      if (nbytes < 0) perror("recv");

                      images_enabled = 0;
                      sockfd = 0;
                      close(i);

                      // This function removes a file descriptor from the file descriptor set.
                      FD_CLR(i, &master);

                      //configuring the tracking process
                      if(acq_mode != (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100))
                      {
                          acq_mode = MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100;
                          i2cstruct.reg_addr =  MUX_MODE;
                          i2cstruct.reg_value = acq_mode & 0x00FF;
                          ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
                      }

                      if(total_sys_number>1 && current_sys_number==1)
                      {
                          send_enable=1;
                      }

                      //starting the acquisition thread (if stopped)
                      mainloop_enable(1);
                      connected = false;
                  } 
                  else 
/* \code
// Se la connessione è valida e se è stato ricevuto un comando valido dal win_client 
// allora l'imgserver esegue il comando ricevuto mediante la funzione Communication 
// definita nel sorgente commands.cpp
Communication(i,buf)
\endcode */

                  // Se la connessione è valida e se è stato ricevuto un comando valido dal win_client 
                  // allora l'imgserver esegue il comando ricevuto mediante la funzione Communication 
                  // definita nel sorgente commands.cpp
                  {
                      if(Communication(i,buf) < 0)
                         printf("imgserver: error executing %s command!\n",buf);
                  }
              }
            } 
        } // end internal for
              
    } // end of for(;;)

    /*! 
    <b>Cancellazione thread e chiusura dispositivi</b>
    \code
    deinitpeople(num_pers);
    pthread_mutex_destroy(&rdlock);
    pthread_mutex_destroy(&mainlock);
    pthread_cancel(ploop);
    pthread_cancel(rdloop);
    pthread_cancel(inloop0);
    pthread_cancel(mloop);
    deinit_io();
    \endcode */

    //deinitpeople();
    //pthread_mutex_destroy(&rdlock);   20100518 eVS moved after cancelling threads!!!
    //pthread_mutex_destroy(&mainlock);
    pthread_cancel(ploop);
    pthread_cancel(rdloop);
    pthread_cancel(inloop0);
    pthread_cancel(inloop1); // 20100518 eVS added
    pthread_cancel(mloop);
    deinitpeople(num_pers); // 20100518 eVS added
    deinit_io();
    
    pthread_mutex_destroy(&rdlock);
    pthread_mutex_destroy(&mainlock);
    pthread_mutex_destroy(&acq_mode_lock);
    
    return 0;
}



/*!
\brief Inizializzazione variabili.

Creazione del percorso del file contenente lo sfondo (#bg_filename), 
creazione del percorso del file contenente i messaggi di log (#rd_filename), 
creazione del percorso del file contenente i parametri di calibrazione (#ca_filename), 
creazione del percorso del file contenente i parametri della porta seriale, no tracking zone e motion detection (#pm_filename), 
creazione del percorso del file contenente i contatori (#cr_filename), 
resetta i contatori delle persone entrate/uscite (#counter_in/#counter_out), 
abilita il conteggio delle persone (#count_enabled), 
settaggio stato porte seriali (#ttyS0=-1 e #ttyS1=-1), 
inizializzazione thread (#thread_status=#MAINLOOP_STOP), 
inizializzazione numero totale di sistemi connessi in serie (#total_sys_number=0), 
inizializzazione indice PCN corrente (#current_sys_number=0), 
disabilita l'invio delle immagini al win_client (#images_enabled=0), 
illuminatori settati in modalit&agrave; manuale (#autoled=false), 
settaggio dello stato iniziale del people counter (#pcn_status=true), 
inizializzazione altri parametri.
*/
void var_init()
{
    sprintf(bg_filename,"%s%s",working_dir,BG_FILENAME); // create background file path
    sprintf(rd_filename,"%s%s",working_dir,RD_FILENAME); // create records file path
    sprintf(pm_filename,"%s%s",working_dir,PM_FILENAME); // create parameters file path
    sprintf(ca_filename,"%s%s",working_dir,CA_FILENAME); // create calibration parameters file path
    sprintf(cr_filename,"%s%s",working_dir,CR_FILENAME); // create counters file path

    connected = false;			//client connected
    counter_in = 0;			//in counter
    counter_out = 0;			//out counter
    people_dir = 0;			//incoming people go down, outgoing go up
    out0 = 0;				//optocoupled out0 queue
    out1 = 0;				//optocoupled out1 queue
    records_idx = 0;			//"records" buffer index
    records_saving_in_progress = false;
    record_enabled = 1;			//enabling in/out counters recording
    count_enabled = 1;			//enabling counting process (via opto I/O and RS485)
    ttyS0 = -1;			//serial port loop status = not active
    ttyS1 = -1;			//serial port loop status = not active
    serloopttyS0 = 0;			//serial port thread init
    serloopttyS1 = 0;			//serial port thread init
    dac_sensor = DAC_SENS;		//enabling changes on both CMOS sensors
    images_enabled = 0;			//not sending images

    /*! \code
    //mainloop is not running also mainloop is running for peopledetection and peopletracking
    thread_status = MAINLOOP_STOP;
    \endcode */

    //mainloop is not running also mainloop is running for peopledetection and peopletracking
    thread_status = MAINLOOP_STOP;

    wd_check = 0;			// watchdog check
    autoled = false;
    autogain = true; //eVS added 20130927
    led_status = false;

    total_sys_number = 0; // total systems number connected for each door
    current_sys_number = 0; // selected system index (0 none system selected???)

    data_wide_gate=NULL;
    pcn_status = 0; //true;					// current pcn status 1->ok, 0->diagnostic found problem
    old_pcn_status = 0; //true;
}



/*!
\brief Inizializzazione periferiche.

Inizializzazione dell'interfaccia quick capture (#pxa_qcp), 
settaggio struttura dati video_picture (#vid_pict), 
settaggio struttura dati video_window (#vid_win), 
settaggio struttura dati i2cstruct, 
configurazione modalit&agrave; di acquisizione mediante la chiamata ioctl(), 
apertura canali di comunicazione di input/output (#pcin0, #pcin1, #pcout0 e #pcout1).

<B>Le porte di input di tipo general purpose (GPI1/GPI2)</B> possono essere usate in quattro modi differenti: <BR>
Do nothing: il sistema ignora ogni segnale ricevuto sulla linea di input.<BR>
Test: permette all'operatore di testare le due linee di ingresso.<BR>
Reset counters: setta a zero i contatori degli ingressi/uscite quando viene rilevato un fronte di salita. 
In configurazione wide-gate solo la porta GPI2 pu&ograve; essere settata come reset.<BR>
Enable/Disable counting: quando la porta GPI1/GPI2 riceve un fronte di salita il PCN inizia il processo di conteggio. 
Quando la porta GPI1/GPI2 riceve un fronte di discesa il PCN blocca il processo di conteggio. 
Nella configurazione wide-gate solo il GPI1 pu&ograve; essere settato come abilitato/disabilitato. 
Per default il processo di conteggio &egrave; attivo.

<B>Le porte di output di tipo general purpose (GPO1/GPO2)</B>: le due porte di output hanno lo scopo di reagire 
quando una persona viene contata. Per default la porta GPO1 &egrave; associata alle persone entranti, mentre la porta 
GPO2 &egrave; associata alle persone uscenti. Quando un PCN trova una persona, uno dei due output (in base alla direzione 
della persona) modifica il suo stato in aperto per un periodo di "GPO-Open-Time" millisecondi. 
*/

/*
If two people walk in the same direction under the PCN-1001, the first signal will be immediately 
sent to the appropriate output, the second will be queued for GPOOT for 2 milliseconds. 
In the Wide-Gate configuration the GPO1 used will be the one of the first PCN-1001. 
The GPO2 will be the one of the last PCN-1001. In any case, both  the GPO-Open-Time have to be set 
connecting the first PCN-1001.
*/
int init_io()
{
    int arg;

    /*! \code
    // inizializzazione dell'interfaccia quick capture
    pxa_qcp = open("/dev/video",O_RDWR);
    \endcode */
    
    pxa_qcp = 0;
    pxa_qcp = open("/dev/video",O_RDWR);
    if (pxa_qcp < 0)
    {
        printf("Error: cannot open /dev/video.\n");
        return -1;
    }

    vid_pict.palette = CAMERA_IMAGE_FORMAT_RAW8;

    /*! \code
    // settaggio della struttura dati video_picture
    ioctl(pxa_qcp,VIDIOCSPICT,&vid_pict);
    \endcode */

    ioctl(pxa_qcp,VIDIOCSPICT,&vid_pict);

    vid_win.width = NX << 1; 
    vid_win.height = NY << 1;

    /*! \code
    // settaggio della struttura dati video_window
    ioctl(pxa_qcp,VIDIOCSWIN,&vid_win);
    \endcode */

    ioctl(pxa_qcp,VIDIOCSWIN,&vid_win);

    /*! \code
    // inizializzazione cattura video
    arg = VIDEO_START;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);
    \endcode */

    arg = VIDEO_START;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);

    // -------------- fpga i2c address --------------------------
    i2cstruct.adapter_nr = 0;
    i2cstruct.slave_addr = 0x20;

    /*! \code
    // configurazione modalita' di acquisizione
    acq_mode = MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100;
    i2cstruct.reg_addr =  MUX_MODE;
    i2cstruct.reg_value = acq_mode & 0x00FF;
    ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
    \endcode */

    if(acq_mode != (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100))
    {
        acq_mode = MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100;
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
    }

    /*! \code
    // viene aperto il primo dispositivo di input seriale
    pcin0 = open("/dev/pcin0",O_RDWR);
    // il primo dispositivo viene triggerato sul fronte di salita
    ioctl(pcin0,PCIOCSIN,RISING);
    \endcode */

    // ------------- optocoupled I/Os ---------------------------
    // viene aperto il primo dispositivo di input seriale
    pcin0 = open("/dev/pcin0",O_RDWR);
    if (pcin0 < 0)
    {
        printf("Error: cannot open /dev/pcin0.\n");
        return -1;
    }
    // il primo dispositivo viene triggerato sul fronte di salita
    ioctl(pcin0,PCIOCSIN,RISING);	// triggered on rising edges

    // the second input device is opened
    pcin1 = open("/dev/pcin1",O_RDWR);
    if (pcin1 < 0)
    {
        printf("Error: cannot open /dev/pcin1.\n");
        return -1;
    }
    // the second input device is triggered onto signal rise    
    ioctl(pcin1,PCIOCSIN,RISING);	// triggered on rising edges
        
    // the first output device is opened
    pcout0 = open("/dev/pcout0",O_RDWR);
    if (pcout0 < 0)
    {
        printf("Error: cannot open /dev/pcout0.\n");
        return -1;
    }
    // the second output device is opened
    pcout1 = open("/dev/pcout1",O_RDWR);
    if (pcout1 < 0)
    {
        printf("Error: cannot open /dev/pcout1.\n");
        return -1;
    }
    
    /*! \code
    // viene aperto il dispositivo wathdog
    watchdog = open("/dev/watchdog",O_WRONLY);
    if (watchdog < 0)
    {
        //...
    }
    else
    {
        int timeout = 30;	// 30 seconds
        ioctl(watchdog, WDIOC_SETTIMEOUT, &timeout);
        write(watchdog, "\0", 1);
    }
    \endcode */
    
    // open the wathdog device
    watchdog = open("/dev/watchdog",O_WRONLY);
    if (watchdog < 0)
    {
        printf("Warning: cannot open /dev/watchdog.\n");
        printf("Warning: watchdog disabled.\n");
    }
    else
    {
        int timeout = 30;	// 30 seconds
        ioctl(watchdog, WDIOC_SETTIMEOUT, &timeout);
        write(watchdog, "\0", 1);
    }

    // ----------- loading the newest records file ------------------
    record_id = 0;
    sprintf(record_fn,"%s%ld.txt",rd_filename,record_id);

    while(file_exists(record_fn))
    {
        sprintf(record_fn,"%s%ld.txt",rd_filename,++record_id);
    }

    if(record_id > 0) sprintf(record_fn,"%s%ld.txt",rd_filename,--record_id);

    return 0;
}


/*!
\brief Blocco della cattura video e chiusura delle porte di input/output, del pxa_qcp e watchdog.
*/
void deinit_io()
{
    int arg;

    arg = VIDEO_STOP;

    // stop video capture
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);

    /*! \code
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);
    \endcode */

    close(pcout0);
    close(pcout1);
    close(pcin0);
    close(pcin1);
    close(pxa_qcp);
    close(watchdog);

    return;
}


/*!
\brief Lettura della versione del sistema operativo dal file "/etc/system.info".
\param version versione del sistema operativo
*/
int sys_version(char *version)
{
    int ret = -1;
    char string[256];
    FILE *fd;

    if((fd = fopen(OS_FILENAME,"r")) == NULL) return -1;
    else
    {
        do
        {
            ret = fscanf(fd,"%s",string);
            if(!strcmp(string,"Software:"))
            {
                ret = fscanf(fd,"%s",string);
                sprintf(version,"%s",string);
                fclose(fd);
                return 0;
            }
        }
        while(ret != EOF);
        fclose(fd);
    }
    return -1;
}


/*!
\brief Lettura della versione del kernel dal file "/proc/version".
\param version versione del kernel
*/
int ker_version(char *version)
{
    float ret = -1;
    char string[256];
    unsigned char i,j;
    FILE *fd;

    if((fd = fopen(KL_FILENAME,"r")) == NULL) return -1;
    else
    {
        do
        {
            ret = fscanf(fd,"%s",string);
            i = 0;
            while(string[i])
            {
                if(!strncasecmp(&string[i],"pcn1001-",8))
                {
                    j=0;
                    while(string[i+8+j] && (string[i+8+j] != '-'))
                    {
                        version[j] = string[i+8+j];
                        j++;
                    }
                    version[j] = 0;
                    fclose(fd);
                    return 0;
                }
                i++;
            }
        }
        while(ret != EOF);
        fclose(fd);
    }
    return -1;
}


/*!
\brief Lettura della versione del bitstream nell'FPGA.
\return Numero della versione del bitstream nell'FPGA.
*/
float fw_version()
{
    /*!
    \code
    i2cstruct.reg_value = 0;
    i2cstruct.reg_addr =  FW_VERSION;
    ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
    \endcode
    */
    i2cstruct.reg_value = 0;
    i2cstruct.reg_addr =  FW_VERSION;
    ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
    return ((i2cstruct.reg_value & 0xF0) >> 4) + ((float)(i2cstruct.reg_value & 0x0F)/10);
}



/*!
\brief Checking file existence.
\param path
\return fopen result
*/
int file_exists(char *path)
{
    int ret = 0;
    FILE *fd;

    if((fd = fopen(path,"r")) != NULL)
    {
        fclose(fd);
        ret = 1;
    }

    return ret;
}


// 20100521 eVS created a common function
void prepare_for_wideconfiguration()
{
    write_parms("sxlimit",SXLIMIT);
    write_parms("dxlimit",DXLIMIT);
    write_parms("sxlimit_riga_start",SXLIMIT_RIGA_START);
    write_parms("dxlimit_riga_start",DXLIMIT_RIGA_START);
    write_parms("sxlimit_riga_end",SXLIMIT_RIGA_END);
    write_parms("dxlimit_riga_end",DXLIMIT_RIGA_END);
    write_parms("up_line_limit",UP_LINE_LIMIT);
    write_parms("down_line_limit",DOWN_LINE_LIMIT);
    save_parms("sxlimit",SXLIMIT);
    save_parms("dxlimit",DXLIMIT);
    save_parms("sxlimit_riga_start",SXLIMIT_RIGA_START);
    save_parms("dxlimit_riga_start",DXLIMIT_RIGA_START);
    save_parms("sxlimit_riga_end",SXLIMIT_RIGA_END);
    save_parms("dxlimit_riga_end",DXLIMIT_RIGA_END);
    save_parms("up_line_limit",UP_LINE_LIMIT);
    save_parms("down_line_limit",DOWN_LINE_LIMIT);

    write_parms("cond_diff_1p",OFF);
    save_parms("cond_diff_1p",OFF);

    write_parms("move_det_en",OFF);
    save_parms("move_det_en",OFF);
    
    ///////////////////
    // 20091120 eVS        
    check_mov_det_parms();
    // 20091120 eVS        
    ////////////////////
}




/*!
\page tabella_calib Tabella riassuntiva parametri di calibrazione

Tabella che permette di capire l'associazione esistente tra indice del vettore dei 
parametri di calibrazione, stringa con il nome del parametro e il valore di default.
\htmlinclude tab_cal.htm
*/


/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


/*!
\page tabella_parms Tabella riassuntiva parametri

Tabella che permette di capire l'associazione esistente tra indice del vettore dei parametri,
stringa con il nome del parametro e il valore di default.
\htmlinclude tab_par.htm
*/



#include "loops.cpp"
#include "commands.cpp"
#include "socket.cpp"
#include "images_fpga.cpp"
#include "io.cpp"
#include "calib_io.cpp"
#include "serial_port.cpp"
