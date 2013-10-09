/*!
    \file default_parms.h
    \brief File header contenente i valori di default di vari parametri.

    Definizione delle costanti relative alle porte seriali, no tracking zone, 
    motion detection, nomi file di configurazione e valori di default delle varie soglie.

    Many of the default parameters specified in this header file are inserted 
    in an array called #default_values[] and the mapping between array locations 
    and parameters is given in this table \ref tabella_parms.

    All the values in #default_values[] are used only if no corresponding
    value is found the parameters file specified by #PM_FILENAME.

    \author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

#ifndef __DEFAULT_PARMS__
#define __DEFAULT_PARMS__

#define VERSION "2.3.12.0b"  // Versione eVS 20130927 BETA

#define SYSTEM  "PCN-1001"	// system name

/*! \def MAX_CONN
\brief Numero massimo di connessioni. */
#define MAX_CONN 256			// max connections

#define MAX_STR_LENGTH 		256     //!< Lunghezza massima della stringa corrispondente ai possibili comandi spedibili con SendString() e ricevibili con RecvString().

#define PARM_STR_LEN		32      //!< Lunghezza massima dei nomi dei parametri (vedi #parm_names)

#define REC_LINE_LEN		34	//!< Lunghezza di una riga del file dei record dei conteggi.

#define MAX_REC_LINES		30000	//!< Numero massimo di righe memorizzate in un file di log.

#define MAX_REC_FILES		10	//!< Numero massimo di file differenti contenenti i messaggi di log.

#define RECORD_INTERVAL	    60  //!< Intervallo di tempo ogni cui viene salvato un file di log.

#define WD_INTERVAL		10	//!< Period of the watchdog check in seconds.

#define MAX_PEOPLE_PER_SEC	10  	//!< Used as the upper limit for the number of the persons that can pass in a second through the surveilled gate.

#define THRESHOLD	60
//!< Initial/Default value for the door threshold variable #soglia_porta.
/*!<
    This value has to be between #MIN_THRESHOLD and #MAX_THRESHOLD.
*/

#define MIN_THRESHOLD		35	
//!< min door threshold position

#define MAX_THRESHOLD		84	
//!< max door threshold position

#define MAX_RECORDS	MAX_PEOPLE_PER_SEC*RECORD_INTERVAL //!< Maximum number of records collectable during the #RECORD_INTERVAL.

/*! \def BORDER_X
\brief Quando nella get_images() viene letta la mappa di disparit&agrave; dal blocco dati 320*240, 
i pixel delle prime 8 e delle ultime 8 colonne vengono settati a zero.*/
/*! \def BORDER_Y
\brief Quando nella get_images() viene letta la mappa di disparit&agrave; dal blocco dati 320*240, 
i pixel delle prime 8 e delle ultime 8 righe vengono settati a zero.*/
#define BORDER_X 12
#define BORDER_Y 8

/*! \def MAX_DATA_LEN
\brief Used by the CRC computation algorithm.

Max length in bytes of the data part in a SNP protocol packet.
*/
/*! \def MAX_PACKET_LEN
\brief Used by the CRC computation algorithm.

Max length in bytes of a packet used by the SNP protocol.
*/
/*! \def MAX_CRC_DIGITS
\brief Used by the CRC computation algorithm.

Length of the CRC table.
*/
/*! \def BYTE_WIDTH
\brief Used by the CRC computation algorithm.

Number of bits of a byte.
*/
/*! \def CRC_WIDTH
\brief Used by the CRC computation algorithm.

Number of bits the CRC code.
*/
/*! \def CRC_MASK
\brief Used by the CRC computation algorithm.

To select the interesting bits of the computed CRC table index.
*/
#define MAX_DATA_LEN		3072
#define MAX_PACKET_LEN		5+1+1+1+1+1+2+MAX_DATA_LEN+2+1
#define MAX_CRC_DIGITS       	256
#define BYTE_WIDTH            	8
#define CRC_WIDTH             	16
#define CRC_MASK              	0xFF

// eVS 20110916
#define MAX_CMD_LEN     12
#define MAX_CHUNK_LEN   (MAX_DATA_LEN-MAX_CMD_LEN-1)

/*! \def MAP_SIZE
\brief ??? */
/*! \def MAP_MASK
\brief ??? */
#define MAP_SIZE 4096			// gpios map size
#define MAP_MASK ( MAP_SIZE - 1 )	// gpios map mask

/*! \def WORK_DIR
\brief Cartella corrente di lavoro. */
#define WORK_DIR	"/var/neuricam/"

/*! \def BG_FILENAME
\brief Nome del file ascii in cui viene salvato lo sfondo acquisito */
/*! \def RD_FILENAME
\brief Nome del file ascii in cui vengono salvati i messaggi di log */
/*! \def PM_FILENAME
\brief Nome del file ascii in cui vengono salvati i parametri di motion detection, parametri delle porte seriali, 
parametri della "no tracking zone", ecc..*/
/*! \def CA_FILENAME
\brief Nome del file ascii in cui vengono salvati i parametri di calibrazione*/
/*! \def CR_FILENAME
\brief Nome del file ascii in cui vengono salvati i contatori delle persone entrate/uscite */
/*! \def OS_FILENAME
\brief Nome del file ascii da cui viene letta la versione del sistema operativo embedded */
/*! \def KL_FILENAME
\brief Nome del file ascii da cui viene letta la versione del kernel */
#define BG_FILENAME 	"background"
#define RD_FILENAME 	"records"
#define PM_FILENAME 	"parameters.txt"
#define CA_FILENAME 	"calibration.txt"
#define CR_FILENAME 	"counters.txt"
#define OS_FILENAME 	"/etc/system.info"
#define KL_FILENAME 	"/proc/version"

/*! \def SERIAL_DEV0
\brief Percorso del file system embedded associato alla porta seriale numero 0 */
/*! \def SERIAL_DEV1
\brief Percorso del file system embedded associato alla porta seriale numero 1 */
#define SERIAL_DEV0 	"/dev/ttyS0"
#define SERIAL_DEV1 	"/dev/ttyS1"

/*! \def MESSAGE_L
\brief ??? */
/*! \def RISING
\brief ??? */
/*! \def FALLING
\brief ??? */
/*! \def RISING_FALLING
\brief ??? */
#define MESSAGE_L	5		// length of a message coming from RS485 port
#define RISING		1
#define FALLING		2
#define RISING_FALLING	3

/*! \def MAINLOOP_STOP
\brief Valore di stop del ciclo principale di elaborazione. */
#define MAINLOOP_STOP	0

/*! \def MAINLOOP_START
\brief Valore di start del ciclo principale di elaborazione. */
#define MAINLOOP_START	1

/*! \def PORT
\brief Valore della porta associata alla socket per la comunicazione client/server mediante protocollo TPC. */
#define PORT 		5400	// socket port

/*! \def UDP_PORT
\brief Valore della porta associata alla socket per la comunicazione client/server mediante protocollo UDP. */
#define UDP_PORT	5402	// RTP port

/*! \def THR_VM_DIAGN
\brief Minimum mean value used for lens obfuscation (or sensor fault) detection in diagnostic (see check_pcn_status())*/
/*! \def THR_VM_DIAGN_DIFF
\brief Maximum accepted difference between left and right mean values used for lens obfuscation (or sensor fault) detection in diagnostic (see check_pcn_status()) */
//#define THR_VM_DIAGN 70 // 20101027 eVS now this value works at 8 bit instead of 10 (just divided by 4)
//#define THR_VM_DIAGN_DIFF 30 // 20101027 eVS now this value works at 8 bit instead of 10 (just divided by 4)

/*******************  default parameters *************************/

/*! \def TIMEBKG
\brief Tempo di attesa (in secondi) per l'aggiornamento dello sfondo. */
#define TIMEBKG 0   //time wait in time background update

/*! \def STATIC_TH
\brief Numero massimo di pixel oltre i quali lo sfondo viene considerato cambiato. */
#define STATIC_TH 2500 //max number of pixel that can be modified in order to I consider the backgound however not changed

/*! \def MAP_MINTH
\brief ??? */
/*! \def MAP_THUNI
\brief ??? */
#define MAP_MINTH	0x3D
#define MAP_THUNI	0x45

/*! \def MAP_DISP
\brief ??? */
/*! \def SLED
\brief ??? */
#define MAP_DISP	0xF0
#define SLED		0x00

/*! \def DIR
\brief Direzione di default: le persone che entrano dalla parte alta ed escono dalla parte bassa 
       dell'immagine, sono considerate entrate nell'area monitorata. */
#define DIR		0x00	// incoming people go down, outgoing go up

/*! \def SERIAL_BR
\brief Transfert rate della porta seriale numero 0 pari a 115200 bits per secondo. */
/*! \def SERIAL_DB
\brief Valore di default dei bits della parte dati da settare sulla porta seriale primaria*/
/*! \def SERIAL_PR
\brief Valore di default del bit di parit&agrave; da settare sulla porta seriale primaria*/
/*! \def SERIAL_SB
\brief Valore di default del bit di stop da settare sulla porta seriale primaria*/
/*! \def SERIAL_ID
\brief Valore di default del bit di indirizzo da settare sulla porta seriale primaria*/
/*! \def SERIAL_MS
\brief Valore di default del bit di abilitazione delle funzionalit&agrave; del PCN master, da settare sulla porta seriale primaria*/
#define SERIAL_BR	 B115200	// 115200 bits per second (ttyS0)
#define SERIAL_DB	 CS8	// 8 bits (ttyS0)
#define SERIAL_PR	 0	// no parity (ttyS0)
#define SERIAL_SB	 0	// 1 stop bit (ttyS0)
#define SERIAL_ID  0x02	// serial ID
#define SERIAL_MS	 0x00	// 1=Enable Master functionalities

/*! \def SERIAL_SBR
\brief Transfert rate della porta seriale numero 1 pari a 115200 bits per secondo. */
/*! \def SERIAL_SDB
\brief Valore di default dei bits della parte dati da settare sulla porta seriale secondaria*/
/*! \def SERIAL_SPR
\brief Valore di default del bit di parit&agrave; da settare sulla porta seriale secondaria*/
/*! \def SERIAL_SSB
\brief Valore di default del bit di stop da settare sulla porta seriale secondaria*/
/*! \def SERIAL_SID
\brief Valore di default del bit di indirizzo da settare sulla porta seriale secondaria*/
#define SERIAL_SBR	B115200	// 115200 bits per second (ttyS1)
#define SERIAL_SDB	CS8	// 8 bits (ttyS1)
#define SERIAL_SPR	0	// no parity (ttyS1)
#define SERIAL_SSB	0	// 1 stop bit (ttyS1)
#define SERIAL_SID	0x03	// second serial ID (if attached)

/*! \def DAC_SENS
\brief ??? */
/*! \def DAC_VREF
\brief ??? */
/*! \def DAC_VPREC
\brief ??? */
/*! \def DAC_VGAP
\brief ??? */
#define DAC_SENS	0x03
#define DAC_VREF	1300	// 1300 mv for both sensors
#define DAC_VPREC	2000	// 2000 mv for both sensors
#define DAC_VGAP	1650	// 1650 mv for both sensors
#define DELTA_VM 50
#define DELTA_VREF 400
#define STEP_VREF 10

/*! \def INPUT0
\brief ??? */
/*! \def INPUT1
\brief ??? */
#define INPUT0		0
#define INPUT1		0

/*! \def OUTTIME0
\brief ??? */
/*! \def OUTTIME1
\brief ??? */
#define OUTTIME0	200
#define OUTTIME1	200

/*! \def NUM_STEP
\brief ??? */
#define NUM_STEP	16

/* \def AUTOLED
\brief Di default e' impostata la modalita' manuale per la variazione della luminosita' dei LED. */
//#define AUTOLED		0

/* \def AUTO_GAIN
\brief Di default e' impostata la modalita' automatica per la variazione del guadagno della Vref. */
#define AUTO_GAIN		1

/*! \def THBKG
\brief ??? */
#define THBKG		0x0A //light intensity threshold to save bkg

/*! \def DETECT_AREA
\brief Distanza (in centimetri) tra il sensore e l'area monitorata. */
#define DETECT_AREA	0 // the distance between the sensor and the detection area (def = 25cm)

/*! \def MIN_AUTOLED
\brief Soglia minima per il controllo automatico dei LED: se il valor medio delle due immagini (#sx_vm_img e #dx_vm_img) 
sono entrambi minore a MIN_AUTOLED e l'algoritmo di autoled e' abilitato allora verranno accesi/potenziati i led
fino a quando non si otterra' un valor medio abbastanza alto in entrambe le immagini (vedi autoled_management())*/
//#define MIN_AUTOLED 	100 //80 //20111207 moved inside the proper function

/*! \var STEPS_225
\brief Mapping delle disparit&agrave; reali sui 16 livelli possibili nel caso in cui si installi il sensore a 225cm.

Vettore contenente i 16 livelli di disparit&agrave; mappati su di un range reale da 1 a 19 nel caso di distanza minima delle teste di 25/30cm.
In realt&agrave; questi valori vengono usati SOLO nel caso in cui non sia presente il file calibration.txt.

\warning Values used for STEPS_225 and STEPS_240 seem to be inverted!
*/
//prendendo spunto dai file di calibrazione trovati sui pcn valori piu' sensati sembrano:
const unsigned char STEPS_225[16]={0,1,2,3,4,5,6,7,8,9,10,12,14,16,19,22};
//al posto di:
//const unsigned char STEPS_225[16]={0,1,2,3,4,5,6,7,8,9,10,11,13,15,17,19};

/*!\var STEPS_240
\brief Mapping delle disparit&agrave; reali sui 16 livelli possibili nel caso in cui si installi il sensore a 240cm.

Vettore contenente i 16 livelli di disparit&agrave; mappati su di un range relae da 0 a 21 nel caso di distanza minima delle teste di 31/40 cm.
In realt&agrave; questi valori vengono usati SOLO nel caso in cui non sia presente il file calibration.txt.

\warning Values used for STEPS_225 and STEPS_240 seem to be inverted!
*/
//prendendo spunto dai file di calibrazione trovati sui pcn valori piu' sensati sembrano:
const unsigned char STEPS_240[16]={0,0,1,2,3,4,5,6,7,8,9,10,12,14,17,20};
//al posto di:
//const unsigned char STEPS_240[16]={0,0,1,2,3,4,5,6,7,9,11,13,15,17,19,21};

/*! \def WIN_SIZE
\brief Command the FPGA to filter using a certain size of the window.  */
/*! \def TIMEOUT_WG
\brief Specifies the timeout for the watch dog */
/*! \def WG_CHECK
\brief ??? */
/*! \def SLAVE_ID
\brief ??? */
/*! \def SX_DX
\brief ??? */
#define WIN_SIZE	0x06 //window size (15+WIN_SIZE)*9
#define TIMEOUT_WG      200     //200*180frames=5min ??? non 10min?
#define WG_CHECK        1     	//check slave presence in wg mode active
#define SLAVE_ID	0x02    //slave ID
#define SX_DX 		1       //power connector is external the two sensor (see the manual)

/*! \def INST_HEIGHT
\brief Altezza vera di installazione, necessaria per riproiezione a terra. */
#define INST_HEIGHT	225     //altezza vera di installazione, necessaria per riproiezione a terra

/*! \def INST_DIST
\brief Distanza fra i due sensori (in cm). */
#define INST_DIST       60      //cm di distanza fra i due sensori

/*! \def NX_WG
\brief ??? */
/*! \def NY_WG
\brief ??? */
#define NX_WG 		40
#define NY_WG 		22

/*! \def TIMEOUT_485
\brief ??? */
/*! \def M485
\brief ??? */
/*! \def S485
\brief ??? */
/*! \def SYS_NUMBER
\brief ??? */
/*! \def SYS_NUMBER_INDEX
\brief ??? */
/*! \def MED_LINE_LEN
\brief ??? */
/*! \def MAX_LOG_LINES
\brief ??? */
#define TIMEOUT_485 1500000
#define M485 1
#define S485 2
#define SYS_NUMBER 0
#define SYS_NUMBER_INDEX 0
#define MED_LINE_LEN	28	// median log lines length
#define MAX_LOG_LINES	100	// max log lines number

/*! \def SXLIMIT
\brief Valore di default dell'indice della colonna della "no tracking zone" di sinistra. */
#define SXLIMIT 0

/*! \def DXLIMIT
\brief Valore di default dell'indice della colonna della "no tracking zone" di destra. */
#define DXLIMIT 160

/*! \def SXLIMIT_RIGA_START
\brief Valore di default della riga di inizio della "no tracking zone" di sinistra. */
#define SXLIMIT_RIGA_START 0

/*! \def DXLIMIT_RIGA_START
\brief Valore di default della riga di inizio della "no tracking zone" di destra. */
#define DXLIMIT_RIGA_START 0

/*! \def SXLIMIT_RIGA_END
\brief Valore di default della riga di fine della "no tracking zone" di sinistra. */
#define SXLIMIT_RIGA_END 120

/*! \def DXLIMIT_RIGA_END
\brief Valore di default della riga di fine della "no tracking zone" di destra. */
#define DXLIMIT_RIGA_END 120

/*! \def OFF
\brief ??? */
/*! \def ON
\brief ??? */
/*! \def DIAGNOSTIC_EN
\brief ??? */
#define OFF 0
#define ON 1
#define DIAGNOSTIC_EN 1

#define MAX_NOISE_PER_PIXEL 2 
//!< Used by the automatic threshold computation for motion detection.
/*!<
    The automatic threshold is computed just multiplying the number of pixel in
    the motion detection zone by this value representing the maximum noise
    that can corrupt a pixel (because of electronic noise, lighting changes,
    neon flickering, ecc...).
    
    At the moment this value is fixed at compile-time and cannot be changed.
*/

////////////////////
// 20091120 eVS
// - Aggiunta MOVE_DET_HH e modificate di conseguenza MOVE_DET_ROW0 e MOVE_DET_ROW1
// - Aggiunta COMP_MOVE_DET_THR e modificata di conseguenza MOVE_DET_THR

//20091112 Bug nei valori dell'area del MD che erano invertiti

#define MOVE_DET_HH   10
//!< Half height of the motion detection zone.

#define MOVE_DET_ROW0 THRESHOLD-MOVE_DET_HH  //50 
//!< Default/Initial value for the upper limit of the motion detection zone (#THRESHOLD-#MOVE_DET_HH).

#define MOVE_DET_ROW1 THRESHOLD+MOVE_DET_HH  //110 
//!< Default/Initial value for the lower limit of the motion detection zone (#THRESHOLD+#MOVE_DET_HH).

#define MOVE_DET_COL0 5  //50 
//!< Default/Initial value for the left limit of the motion detection zone.

#define MOVE_DET_COL1 154 //70  
//!< Default/Initial value for the right limit of the motion detection zone.

#define MOVE_DET_ALFAREG 7 /* equivale a 128 (vedi documentazione fpga) */
//!< Definisce il "learning rate" del motion detection.
/*!<
    Cambiando questo valore si cambia il coefficiente alfa usato nella
    formula di aggiornamento del "background" usata dall'algoritmo di motion detection.
*/

#define COMP_MOVE_DET_THR(r0,r1,c0,c1) ((((r1-r0+1)*(c1-c0+1)*MAX_NOISE_PER_PIXEL)/100)+200)
//!< Computes the motion detection threshold.
/*!<
    The threshold has to be greater than the noise (i.e. at most (MD_area*#MAX_NOISE_PER_PIXEL))
    plus a certain amount empirically estimated equal to 200.
*/

 
#define MOVE_DET_THR COMP_MOVE_DET_THR(-MOVE_DET_HH,MOVE_DET_HH,MOVE_DET_COL0,MOVE_DET_COL1) //80
//!< Default/Initial value for the motion detection threshold.
/*!< 
    Questa soglia puo' essere cambiato anche tramite win_client ma il valore contenuto
    in questa define e' quello di default che viene inserito nel vettore #default_values[] (vedi \ref tabella_parms).
*/

#define MOVE_DET_EN 1 // 20091125 now move detection is ON by default
//!< Valore di default dello stato on/off dell'algoritmo di motion detection.
/*!< 
    L'attivazione/disattivazione dell'algoritmo di motion detection puo' essere 
    fatta anche da win_client ma il valore associato a questa define e' quello di
    default.    

    The used motion detection zone is obtained by intersecting the default 
    motion detection zone (given by #MOVE_DET_ROW0, #MOVE_DET_ROW1, #MOVE_DET_COL0, 
    #MOVE_DET_COL1) with the area where tracking is active (i.e. the complement of the
    no-tracking zone).
    
    The motion detection zone has to be centered around the door threshold
    whose default value is #THRESHOLD. So the starting/ending rows for motion detection
    are obtained by subtracting/adding #MOVE_DET_HH to #THRESHOLD.
*/

// 20091120 eVS
////////////////////


#define DOOR_STAIRS_EN 0 //!< Default value for the option "Stairs Upgrade Enabled" in the tab "Advanced (2/2)" of the win_client application.

#define UP_LINE_LIMIT 0     //!< No-tracking zone up line default value.
#define DOWN_LINE_LIMIT 120 //!< No-tracking zone down line default value. 

/*! \def DIS_AUTOBKG
\brief Default value for the option "Disable Automatic Background"
\date February 2009 (after Lisbon)
*/
#define DIS_AUTOBKG 0   //20090506 Lisbona
#define DOOR_SIZE   120   //20130411 eVS, DOOR_SIZE instead of DOOR_KIND (for 2.3.10.7)

//eVS 20130715
#define HANDLE_OOR 0 // To handle the OOR situation after reboot

// eVS 20100419
#define INITIAL_STD_BKG 10 //!< Serve per inizializzare la deviazione standard del background al posto dello zero

const int FROM_DISP_TO_HEAD = 4; // notice that length_px = disp * (length_cm/baseline) = disp*k, since the head is 
                                 // supposed to be around 20cm and the baseline is 5 than follows that k=20/5=4
                                 // now since disp is multiply by 16 we have to divide it by 16 before this operation
                                 // follows that head_px = (disp/16)*k = disp*k/16 = disp*4/16 = disp/4
                                 // more precisely, we should use the number idx = (disp/16) as an index in the 
                                 // calib_parm_values vector which maps those values in the correct disparity,
                                 // i.e., length_px = calib_parm_values[disp/16]*(length_cm/baseline)
const int FROM_DISP_TO_HEAD_RAY = FROM_DISP_TO_HEAD*2;

const float FROM_DISP_TO_SHOULDER = 1.33f; // shoulder_px = (disp/16)*(60cm/5cm) = (disp/16)*12 = disp*(3/4) = disp/(4/3)
const float FROM_DISP_TO_SHOULDER_RAY = FROM_DISP_TO_SHOULDER*2.0f; 

// 20130220 eVS, moved here both NUM_PERS_SING and MAX_NUM_SLAVES
#define NUM_PERS_SING 10  //!< maximum number of person detectable for each sensor
const int MAX_NUM_SLAVES = 5;  //!< Maximum number of slaves connected to a master in widegate

// 20130220 eVS, added
const int MAX_NUM_PERS = NUM_PERS_SING*(MAX_NUM_SLAVES+1);  //!< Maximum number of slaves connected to a master in widegate

#endif
