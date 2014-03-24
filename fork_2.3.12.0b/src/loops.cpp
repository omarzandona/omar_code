/*!
\file loops.cpp
\brief Implementazione delle funzioni corrispondenti ai vari thread lanciati nel main().

I vari thread corrispondono a delle funzioni di tipo loop ovvero contenenti 
un ciclo che continua a ripetere le stesse azioni. I cicli implementati sono:
- ciclo principale (main_loop()) in cui avviene la lettura del blocco dati #NX*#NY*4=160*120*4=320*240 dall'FPGA
e la chiamata agli algoritmi di detection e tracking delle persone (detectAndTrack())
- cicli di gestione delle porte seriali: input_loop0(), input_loop1(), ser_loopttyS0() e ser_loopttyS1()
- il ciclo di ping ping_loop()
- il ciclo relativo al watchdog watchdog_loop()

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
        
*/

//extern bool count_true_false;
//extern pthread_mutex_t acq_mode_lock; // 20100517 eVS

#include "directives.h"

//////////////////////
// 20091123 eVS
//#define MEASUREMENTS
#define MAX_MEASURES_PER_FILE 2500

//#define LAST_MOV_DET_LENGTH 10
//int last_mov_det_left[LAST_MOV_DET_LENGTH];
//int last_mov_det_right[LAST_MOV_DET_LENGTH];

// median algorithm implementation follows
/*
    typedef int pixelvalue ;
    #define PIX_SWAP(a,b) { register pixelvalue t=(a);(a)=(b);(b)=t; }
    pixelvalue kth_smallest(pixelvalue a[], int n, int k)
    {
        register int i,j,l,m ;
        register pixelvalue x ;

        l=0 ; m=n-1 ;
        while (l<m) {
            x=a[k] ;
            i=l ;
            j=m ;
            do {
                while (a[i]<x) i++ ;
                while (x<a[j]) j-- ;
                if (i<=j) {
                    PIX_SWAP(a[i],a[j]) ;
                    i++ ; j-- ;
                }
            } while (i<=j) ;
            if (j<k) l=i ;
            if (k<i) m=j ;
        }
        return a[k] ;
    }
    #undef PIX_SWAP
    #define median_WIRTH(a,n) kth_smallest(a,n,(((n)&1)?((n)/2):(((n)/2)-1)))
*/
// 20091123 eVS
//////////////////////

extern unsigned char Frame_DX[NN];   //!< Immagine destra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_SX[NN];   //!< Immagine sinistra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_DSP[NN];  //!< Mappa di disparit&agrave; #NX*#NY=160*120 (16 livelli di disparit&agrave; distribuiti su 256 livelli di grigio).

inline void autoled_management(const unsigned int i_sx_vm_img, const unsigned int i_dx_vm_img);
inline bool check_pcn_status(const unsigned int i_sx_vm_img, const unsigned int i_dx_vm_img, const bool i_autoled, unsigned char& o_error_code);
extern unsigned char auto_gain;

void diagnostic_log(void)
{
    if(pcn_status != old_pcn_status && diagnostic_en==1) 
    {
      time_t curtime;
      struct tm *loctime;
  
      curtime = time (NULL);
      loctime = localtime (&curtime);

      pthread_mutex_lock(&rdlock);
      //if(pcn_status==0)
      if(pcn_status) // 20111207 eVS, if !=0 means that an error occurred
      {
          sprintf(records[records_idx],"Diagnostic error=%02d\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n", pcn_status,
              loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year,
              loctime->tm_hour,loctime->tm_min,loctime->tm_sec, counter_in, counter_out); 
      }
      else
      {
          sprintf(records[records_idx],"Diagnostic ok\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n",
              loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year,
              loctime->tm_hour,loctime->tm_min,loctime->tm_sec, counter_in, counter_out);
      }    
      
      old_pcn_status = pcn_status;

      if(records_idx < (MAX_RECORDS-1)) records_idx++;
      pthread_mutex_unlock(&rdlock);
    }
}

/*!
\brief Ciclo principale di elaborazione.

Questa funzione contiene il ciclo principale di lettura del blocco dati #NX*#NY*4=160*120*4=320*240 (#Frame) dall'FPGA, 
la funzione di deinterlacciamento del buffer (mediante la funzione get_images() contenuta in images_fpga.cpp) 
nelle immagini sinistra (#Frame_SX), destra (#Frame_DX), mappa di disparit&agrave; (#Frame_DSP), 
parametri di motion detection e valor medio (calcolati dall'FPGA). 
Inoltre in base al valor medio della mappa di disparit&agrave; viene aggiornato il valore corrente della VREF 
per migliorare la luminosit&agrave; dell'immagine. 
Poi viene eseguito il controllo automatico dei led: 
accensione/spegnimento in base al valor medio della mappa di disparit&agrave;. 
Dopo lettura della immagini, deinterlacciamento e settaggio vref, viene chiamato l'algoritmo di people detection 
(funzione detectAndTrack(), contenuta in peopledetection.cpp) delle persone che si trovano all'interno dell'area monitorata. 
Diagnostica: in base al valor medio dell'immagine #vm_img destra (o sinistra) e in base al valore corrente della VREF 
di destra #current_vref_right (o di sinistra #current_vref_left) rispetto al valore di default #default_vref_right 
(o #default_vref_left), a meno di una costante, viene aggiornato il valore corrente della vref destra (o sinistra). 
Diagnostica: in base al valor medio destro e sinistro e in base al valore corrente della VREF 
aggiorno un contatore che mi servir&agrave; per modificare lo stato del sistema da diagnostica 
a corretto funzionamento.
Se il sistema &egrave; connesso ed abilitato 
a ricevere immagini (a seconda del valore della #connected e della #images_enabled) 
allora procede con l'acquisizione dei frames da telecamere. 
Inoltre il server invia al client le immagini cos&igrave; ottenute 
(mediante la sendto() sulla socket UDP inizialmente aperta dal server stesso).
*/
void*main_loop(void*)
{
    unsigned long ID,header;
    unsigned long people[2];
    //struct timeval start,stop; //not used ???
    //unsigned long time_udp; //not used ???
    int value;
    static unsigned char sensor = 0;
    static unsigned char sx_vm_img = 128; //512; // 20101014 eVS bugfix
    static unsigned char dx_vm_img = 128; //512; // 20101014 eVS bugfix
    socklen_t addr_len;
    int imgfd;

    imgfd = socket(AF_INET,SOCK_DGRAM,0);
    
    // questa funzione viene usata per le operazioni di manipolazione e di controllo 
    // delle varie proprieta' e caratteristiche di un file descriptor
    // usando F_SETFL la funzione imposta il file status flag al valore specificato da arg
    fcntl(imgfd,F_SETFL,O_NONBLOCK);
    
    addr_len = sizeof(struct sockaddr);

    /*!
    <b>Lettura mappa di disparit&agrave; e tracking delle persone</b>
    \code
    while(thread_status != MAINLOOP_STOP)
    { 
        // read buffer (160*120*4=320*240) from FPGA
        read(pxa_qcp,Frame,imagesize);

        // Deinterlacciamento del blocco dati 160*120*4=320*240 precedentemente letto in left img, right img, 
        // disparity map, left or right mean value and motion detection output.
        get_images(Frame,0);

        //...

        // tracking mode
        if(acq_mode & 0x0100)	
        {
            //...
            
            // main_loop call the people detection algorithm
            detectAndTrack(Frame_DSP,people[0],people[1],count_enabled,get_parms("threshold"),get_parms("dir"));

            //...
    \endcode
    */

    //////////////////////
    // 20091123 eVS
    // - Added the declaration of the new flag FPGA_boot_done
    // - and added initialization of the variable count_check_counter
    // - moreover, added the initialization of the count_true_false flag
    // otherwise it has no value before FPGA_boot_done gets true
    // - added log file for move detection measurements saving
    bool FPGA_boot_done = false;
    count_check_counter = 0;
    count_true_false = false;

#ifdef MEASUREMENTS
    FILE *measurements_file;
    {
        char command[255];
        // creating the debug log file if it does not exist
        sprintf(command,"/bin/touch /var/neuricam/measurements.txt");	
        system(command);
    }

    measurements_file = fopen("/var/neuricam/measurements.txt","w");
#endif
    // 20091123 eVS
    //////////////////////
    
    
#ifdef FRAME_RATE_COMPUTATION
    //////////////////////
    // 20101028 eVS, to evaluate framerate
    struct timeval fr_start, fr_stop; 
    gettimeofday(&fr_start,NULL);
    unsigned long int prev_framecounter = 0;
    // 20101028 eVS
    //////////////////////
#endif
    
#ifdef FRAME_RATE_COMPUTATION
    int num_fr_not_processed = 0;
#endif

#ifdef ENSURE_ONE_BY_ONE_PROCESSING
    int num_cons_eq = 0;
    unsigned char Frame_DSP_prev[NN];
    unsigned char Frame_SX_prev[NN];
    memset(Frame_DSP_prev,0,sizeof(Frame_DSP_prev)); 
    memset(Frame_SX_prev,0,sizeof(Frame_SX_prev)); 
#endif    
    
    framecounter = 0; // 20101028 eVS added
    while(thread_status != MAINLOOP_STOP)
    { 
        pthread_mutex_lock(&acq_mode_lock); // 20100517 eVS

        read(pxa_qcp,Frame,imagesize); // read buffer (160*120*4=320*240) from FPGA

        get_images(Frame,0); // decomposizione del buffer 160*120*4=320*240 precedentemente letto (left img, right img, disparity map, left or right mean value and motion detection output)

        // ************ for moving detection **************************************
        mov_det_left = ((((mov_dect_15_23l & 0xff) << 16) | ((mov_dect_8_15l & 0xff) << 8) | (mov_dect_0_7l & 0xff))/100); 
        mov_det_right = ((((mov_dect_15_23r & 0xff) << 16) | ((mov_dect_8_15r & 0xff) << 8) | (mov_dect_0_7r & 0xff))/100); 

        //////////////////////
        // 20091123 eVS
        // Noticed corrupted move detection data before the 450th frame
        // so the flag boot_done has been added to wait
        if (framecounter > 450)
            FPGA_boot_done = true;

        // 20091123 Bug fixed and added new condition on FPGA_boot_done 
        //if((mov_det_left > move_det_thr) | (mov_det_right > move_det_thr)
        if (FPGA_boot_done) 
        {
          // add a measurement to the move detection values file
#ifdef MEASUREMENTS
          if (move_det_en)
            fprintf(measurements_file,"%d\n",mov_det_left);
#endif            
          //move_det_thr = 200; // threshold fixed just to be sure that is it set as I want
          if(((mov_det_left > move_det_thr) || (mov_det_right > move_det_thr)) && (FPGA_boot_done)) 
        // 20091123 eVS
        //////////////////////
            {
                //////////////////////
                // 20091123 eVS
                /*if (count_check_counter == 0)
                    //print_log("Motion on:\n\t framecounter=%d\t mov_det_left=%d\t mov_det_right=%d\n", framecounter, mov_det_left, mov_det_right);
                if (framecounter%40 == 0)
                    //print_log("Motion on:\n\t framecounter=%d\t mov_det_left=%d\t mov_det_right=%d\n", framecounter, mov_det_left, mov_det_right);*/
                // 20091123 eVS
                //////////////////////

                count_check_counter=200;  // 200 if for inertia
            } else {
                count_check_counter--;
            
                //////////////////////
                // 20091123 eVS
                /*if (count_check_counter == 0)
                    //print_log("Motion off:\n\t framecounter=%d\t mov_det_left=%d\t mov_det_right=%d\n", framecounter, mov_det_left, mov_det_right);*/
                // 20091123 eVS
                //////////////////////

                if(count_check_counter<0) count_check_counter=0;
            }
        

            if(count_check_counter > 0) {
                count_true_false = true;
            } else {
                count_true_false = false;
            }
        }
        // ************************************************************************
      if(auto_gain)
        {
          if(framecounter%2 ==0 && framecounter%40 !=0 && framecounter%38 !=0  && framecounter%42 !=0) // per essere lontani da cambio sensore per vm
          {
            // in base al valor medio della mappa viene aggiornato il valore corrente 
            // della VREF per migliorare la luminosita' dell'immagine
            if((vm_img*4)<512-DELTA_VM)
            {
                if((current_vref_left+STEP_VREF)<(default_vref_left +DELTA_VREF))
                {
                    if((sensor & 0x01) == 1)
                    {
                        current_vref_left = current_vref_left+STEP_VREF;
                        if(current_vref_left > default_vref_left +DELTA_VREF)  
                        {
                            current_vref_left = default_vref_left +DELTA_VREF;
                        }
                        calib_write_parms("dac_vref2",current_vref_left);
                    }
                }
                if((current_vref_right+STEP_VREF)<(default_vref_right +DELTA_VREF))
                {
                    if((sensor & 0x01) == 0)
                    {
                        current_vref_right = current_vref_right+STEP_VREF;
                        if(current_vref_right > default_vref_right +DELTA_VREF)  
                        {
                            current_vref_right = default_vref_right +DELTA_VREF;
                        }
                        calib_write_parms("dac_vref1",current_vref_right);
                    }        
                }
            } 
            if((vm_img*4)>512+DELTA_VM)
            {
                if((current_vref_left-STEP_VREF)>(default_vref_left -DELTA_VREF))
                {
                    if((sensor & 0x01) == 1)
                    {
                        current_vref_left = current_vref_left-STEP_VREF;
                        if(current_vref_left < default_vref_left -DELTA_VREF)  current_vref_left = default_vref_left -DELTA_VREF;
                        calib_write_parms("dac_vref2",current_vref_left);
                    }
                }
                if((current_vref_right-STEP_VREF)>(default_vref_right -DELTA_VREF))
                {
                    if((sensor & 0x01) == 0)
                    {
                        current_vref_right = current_vref_right-STEP_VREF;
                        if(current_vref_right < default_vref_right -DELTA_VREF)  current_vref_right = default_vref_right -DELTA_VREF;
                        calib_write_parms("dac_vref1",current_vref_right);
                    } 
                }
            }
          }
        }
        unsigned long counter_in_to_be_sent; // 20111011 eVS, added in order to avoid usage of counter_in in Send
        unsigned long counter_out_to_be_sent; // 20111011 eVS, added in order to avoid usage of counter_out in Send
        bool count_true_false_to_be_sent; // 20111011 eVS, added in order to avoid usage of counter_out in Send

#ifdef ENSURE_ONE_BY_ONE_PROCESSING
        // 20120508 eVS process only new frame (different wrt the previous one)
        int cmp_res = memcmp(Frame_DSP_prev, Frame_DSP, sizeof(Frame_DSP_prev));
        if (cmp_res == 0) // se sono uguali verifico per scrupolo anche l'immagine SX
          cmp_res = memcmp(Frame_SX_prev, Frame_SX, sizeof(Frame_SX_prev));
        
        memcpy(Frame_DSP_prev, Frame_DSP, sizeof(Frame_DSP_prev));
        memcpy(Frame_SX_prev, Frame_SX, sizeof(Frame_SX_prev));
        
        if (cmp_res == 0)
        {
          num_cons_eq++;
          if (num_cons_eq > 2)
          {
            // non possono esserci più di 3 frame uguali consecutivi quindi elaboro
            cmp_res = 1;
            num_cons_eq = 0;
          }
#ifdef FRAME_RATE_COMPUTATION
          else
            num_fr_not_processed++;
#endif
        }
        else
          num_cons_eq = 0;
        
        if((acq_mode & 0x0100) && (cmp_res != 0))	//tracking
#else
        if(acq_mode & 0x0100)	//tracking
#endif
        {
            pthread_mutex_lock(&mainlock); 

            //detectAndTrack(Frame_DSP,people[0],people[1],count_enabled,get_parms("threshold"),get_parms("dir"));
            detectAndTrack(
              Frame_DSP,
              people[0], people[1],
              count_enabled,
              get_parms("threshold"),
              get_parms("dir"),
              move_det_en
#ifdef USE_NEW_TRACKING
              , (limit_line_Down-limit_line_Up+1)/4);
#else
              );
#endif
            
            people_rec[0] = people[0];
            people_rec[1] = people[1];
            
            // 20100521 eVS if we are in wideconfiguration and this sensor
            // is the last in the chain
            if(total_sys_number>1 && current_sys_number==total_sys_number) //se sono l'ultimo
            {
                if(ev_door_open==true && frame_cnt_door==3) //open door procedure has 4 states
                {
                    unsigned long counters[2];
                    counters[0] = people[0];
                    counters[1] = people[1];
                    int error=1;
                    flag_serial=0;
                    SNP_Send(get_parms("serial_sid"),"open_door",(char *)counters,sizeof(counters),ttyS0);
                    while(flag_serial==0 && error<100)
                    {
                        error++;
                        if(error%4==0) 
                            SNP_Send(get_parms("serial_sid"),"open_door",(char *)counters,sizeof(counters),ttyS0);
                        
                        //pthread_mutex_unlock(&mainlock); // 20130429 eVS, DO NOT ADD THIS LINE
                        usleep(100); //wait 1/10000 of sec and check another time
                        //pthread_mutex_lock(&mainlock);  // 20130429 eVS, DO NOT ADD THIS LINE
                        
                        // 20100521 eVS removed because this code was not reachable
                        // ma se nella condizione del while c'e' error<100 come fara' ad essere soddisfatta la condizione sotto ???
                        /*if(error==1000)
                        {//after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                            ev_door_open=false;
                        }*/
                    }
                    ev_door_open=false;      
                }

                if(ev_door_close==true && frame_cnt_door==1) //open door procedure has 2 states
                {
                    unsigned long counters[2];
                    counters[0] = people[0];
                    counters[1] = people[1];
                    ev_door_close=false;
                    SNP_Send(get_parms("serial_sid"),"close_door",(char *)counters,sizeof(counters),ttyS0); //close door event
                }
            }
            
            // 20100521 eVS if the time background and counting are enabled
            // every 60 frames try to update the background
            if(minuti_th!=0 && (framecounter%60==0) && count_enabled==1)
                FindStaticObj(); 

            // 20100521 eVs if in wideconfiguration, i.e., (total_sys_number>1), and
            // this sensor is the last of the chain, i.e., (current_sys_number==total_sys_number)
            // then this system has to generate the syncro clock
            if((current_sys_number==total_sys_number) && (total_sys_number>1))
            {// se sono sistema N di N allora genero clock di sincro
                value=framecounter%2;
                if(value==1)
                {
                    set_gpio("GPSR3_091",1); // 20100521 eVS "GPOUT0 set"
                    //count_sincro++;
                }
                else
                {
                    set_gpio("GPCR3_091",1); // 20100521 eVS "GPOUT0 clear" 
                    //count_sincro++;
                }
                // 20100521 eVS moved here avoiding redundancy
                count_sincro++;
            }

            // 20100521 eVs if in wideconfiguration, i.e., (total_sys_number>1), 
            // and this sensor is the master, i.e., (current_sys_number==1),
            // and (count_sincro==255) then reset the syncro-counter of the
            // slaves
            if((current_sys_number==1)&&(total_sys_number>1)&&(count_sincro==255))
            {
                unsigned char val=0;
                SNP_Send(slave_id,"set_sincro_counter",(char *)&val,sizeof(val),ttyS1);
            }
           
            if(send_enable)
            {                
                if(total_sys_number<2) 
                {
                    send_enable=0;
                }
                else
                {   
                    // 20100603 eVS if we are here means that we are
                    // in wideconfiguration
                    
                    if(data_wide_gate==NULL)
                    {
                        // 20100521 eVS added because if break exits from the while
                        // loop, the muteses have to be unlock
                        pthread_mutex_unlock(&mainlock); // 20100521 eVS added
                        pthread_mutex_unlock(&acq_mode_lock); // 20100521 eVS added
                        
                        print_log("data_wide_gate == NULL\n"); // 20111013 eVS, added to verify problems

                        break; // ???
                    }

                    memcpy(data_wide_gate,persdata,sizeof(persdata));

                    people[people_dir]=counter_in; //slave perform the counting so the 2 counters is provided via rs485
                    people[1-people_dir]=counter_out; 
                    
                    if(framecounter%180!=0)
                        // 20100525 eVS at each new frame each ask data to its slave
                        SNP_Send(slave_id,"persdetwidegate",(char *)data_wide_gate,54*sizeof(unsigned char),ttyS1);
                    // 20100524 eVS inverted condition in the if in order to get code more readable
                    // and moved else before
                    //if(framecounter%220==0) 
                    else {
                    //if(framecounter%180==0) {
                        // 20100521 eVS every 220 frames master ask the counters to slave
                        if(flag_wg==0 && wg_check==1)  //wg_check==1 the slave presence check is activated
                        {
                            if(flag_wg_count==TIMEOUT_WG)
                            {
                                //------------scrittura su syslog-------------------//     
                                time_t curtime_wg;
                                struct tm *loctime_wg;
                                char wg_str[255];
                                char comando[200];
                                FILE *record_wg;    
                                unsigned long lines;

                                curtime_wg = time (NULL);
                                loctime_wg = localtime (&curtime_wg);
                                sprintf(wg_str,"wg_off\t%02d/%02d/%04d\t%02d:%02d:%02d\n",
                                    loctime_wg->tm_mday,loctime_wg->tm_mon+1,1900+loctime_wg->tm_year,
                                    loctime_wg->tm_hour,loctime_wg->tm_min,loctime_wg->tm_sec); 
                                record_wg = fopen("/var/neuricam/syslog.txt","a+");
                                fseek(record_wg,0L,SEEK_END);
                                lines = ftell(record_wg)/MED_LINE_LEN;

                                if(lines >= MAX_LOG_LINES)
                                {
                                    fclose(record_wg);	// the records file is full.
                                    sprintf(comando,"rm /var/neuricam/syslog.txt");
                                    system(comando);	// removing the oldest file

                                    record_wg = fopen("/var/neuricam/syslog.txt","a+");
                                }

                                fprintf(record_wg,"%s",wg_str);

                                fclose(record_wg); 
                                //--------------------fine scrittura---------------------------//
                                SNP_Send(slave_id,"restore",NULL,0,ttyS1);
                                save_parms("sys_number",0);
                                save_parms("sys_number_index",0);
                                write_parms("sys_number",0);  //slave not responding => wg off
                                write_parms("sys_number_index",0);
                                wide_gate_serial_parms_reset();
                            }
                            else 
                            {
#ifdef debug_
                                printf("Last device not responding to gcounters %d\n",flag_wg_count);
#endif
                                flag_wg_count++;
                            }
                        }
                        flag_wg=0;
                        
                        // 20111013 eVS, nota: ogni 180 frame (circa 3-4 sec) il master perde 100ms+100ms per ricevere i conteggi
                        // dall'ultimo slave che conta... questo credo possa provocare perdita di conteggi nel lato 
                        // master (persone che passano piuttosto lateralmente sotto al master)
                        //pthread_mutex_unlock(&mainlock); // 20130429 eVS, DO NOT ADD THIS LINE
                        usleep(100000);
                        SNP_Send(slave_id,"gcounters",NULL,0,ttyS1); //every 180(220) frames master ask the counters to slave
                        usleep(100000);
                        //pthread_mutex_lock(&mainlock); // 20130429 eVS, DO NOT ADD THIS LINE

                    }
                    // 20100524 eVS inverted condition in the if in order to get code more readable
                    //else 
                    //    SNP_Send(slave_id,"persdetwidegate",(char *)data_wide_gate,54*sizeof(unsigned char),ttyS1);
                }
            } // fine if(send_enable)
            //framecounter++; 20100517 eVS moved after while finishes

            // 20120317 eVS removed if because in widegate using digital output both first and last PCN in widegate needs to send data            
            //if (total_sys_number < 2 || current_sys_number == total_sys_number || current_sys_number == 1) 
              record_counters(people[people_dir],people[1-people_dir]);
                        
            counter_in_to_be_sent = counter_in;
            counter_out_to_be_sent = counter_out;            
            count_true_false_to_be_sent = count_true_false;
            
            pthread_mutex_unlock(&mainlock); 
            
        } //if(acq_mode & 0x0100)
        else 
        {
        
          //usleep(4000); // just wait a bit for the next new image (since processing time is around 15ms and the acquisition time is around 18ms, this time should be greater than 3ms)
          
          pthread_mutex_lock(&mainlock);         
            counter_in_to_be_sent = counter_in;
            counter_out_to_be_sent = counter_out;            
            count_true_false_to_be_sent = count_true_false;        
          pthread_mutex_unlock(&mainlock); 
          
          // 20101029 eVS, added this to have diagnostic messages in records even if no counting is performed
          diagnostic_log();          
        }
                
        
        framecounter++; // 20100517 eVS moved here
        
#ifdef FRAME_RATE_COMPUTATION
        ////////////////////////////
        // 20101028 eVS, to evaluate framerate
        if (framecounter%400==0) {
            if (framecounter>prev_framecounter) {
                static int frame_rate = 40;

                gettimeofday(&fr_stop,NULL);
                unsigned long int fr_time = (fr_stop.tv_sec-fr_start.tv_sec)*1000 + (fr_stop.tv_usec-fr_start.tv_usec)/1000;

                frame_rate = ((framecounter-prev_framecounter-num_fr_not_processed)*1000)/fr_time;
                num_fr_not_processed = 0;

                FILE *fr_file = fopen("/var/neuricam/fps.txt","w"); // create a new file
                fprintf(fr_file,"fps: %d\n", frame_rate);
                fclose(fr_file);
            }
                
            gettimeofday(&fr_start,NULL);
            prev_framecounter = framecounter;
        } 
        // 20101028 eVS, to evaluate framerate
        ////////////////////////////
#endif

        // *************** for diagnostic, autoled and overexposure control ***************
        //if(framecounter%20 ==0 && framecounter%40 !=0)
        {
            //static unsigned char sx_vm_img = 128; //512; // 20101014 eVS bugfix
            //static unsigned char dx_vm_img = 128; //512; // 20101014 eVS bugfix
            //static unsigned char sensor = 0; // il primo sensore di cui leggero' il valore e' lo 0, i.e., il destro
            static const int SWITCH_SENSOR_INTERVAL = 40; // has to be a multiple of 4

            if(framecounter%SWITCH_SENSOR_INTERVAL == 0) // 0, 40, 80, ecc
            {
                // switch del sensore di cui osservare il valor medio
                i2cstruct.reg_addr =  0x15;
                i2cstruct.reg_value = (sensor & 0x01);   // 0 dx, 1 sx
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
                //sensor++; // mi preparo per il prossimo switch
            }
            else if( (framecounter%SWITCH_SENSOR_INTERVAL) == (SWITCH_SENSOR_INTERVAL/4) ) // 10, 50, 90, ecc
            {
                // automatic control of led light intensity with trigger zone
                if (autoled)
                    autoled_management(sx_vm_img, dx_vm_img);
            } 
            else if( (framecounter%SWITCH_SENSOR_INTERVAL) == (SWITCH_SENSOR_INTERVAL/2) ) // 20, 60, 100, ecc
            {
                // in base al valor medio destro e sinistro e in base al valore corrente della VREF 
                // aggiorno un contatore che mi servira' per modificare lo stato del sistema da diagnostica 
                // a corretto funzionamento
                if((sensor & 0x01)==0)
                    dx_vm_img = (unsigned char)(((int)(dx_vm_img)*3 + (int)vm_img)>>2);
                else
                    sx_vm_img = (unsigned char)(((int)(sx_vm_img)*3 + (int)vm_img)>>2);
                    
                // controlli di diagnostica con un intervallo doppio dello switch del sensore
                // in modo che tra un check e l'altro vengono aggiornati entrambi i valori medi
                // sia del sensor 0 che del sensor 1
                if((sensor & 0x01)==1) {
                    //print_log("vm_sx = %d,\t vm_dx = %d\n", sx_vm_img, dx_vm_img);  // eVS DEBUG
                    check_pcn_status(sx_vm_img, dx_vm_img, autoled, pcn_status); 
                }
                
                // mi preparo per il prossimo switch dei sensori
                sensor++;
                //print_log("sensor = %d \n", sensor);  // eVS DEBUG    
            }
        }
        // ***************************************************************************************

        /*!
        <b>Il server invia le immagini al client per la visualizzazione su interfaccia</b>
        \code
        if(connected && images_enabled)
        // se il win_client è connesso ed abilitato a ricevere immagini 
        // allora si procede con lo spedire la mappa di disparita' origianle o filtrata
        // con o meno i frame a seconda dell'opzione scelta da win_client
        {
            //...
            switch(acq_mode)
            {
                // il server invia al client (mediante socket UDP) solo le immagini destra e sinistra
                // poi il client mediante un'unica recvfrom deinterlaccia 
                // le informazioni contenute in un unico pacchetto
                case MUX_MODE_8_FPN: // 8 bit + Removing Fixed Pattern Noise
                case (MUX_MODE_8_FPN_ODC | 0x1000): // 8 bit + Removal Fixed Pattern Noise + Removal Distorsion
                    header = 0x03;
                    // Le due funzioni principali usate per la trasmissione di dati attraverso i socket UDP 
                    // sono "sendto" e "recvfrom". La necessita' di usare queste funzioni e' dovuta al fatto che 
                    // non esistendo con UDP il concetto di connessione, non si ha neanche a disposizione 
                    // un socket connesso su cui sia possibile usare direttamente read e write avendo gia' 
                    // stabilito (grazie alla chiamata ad accept che lo associa ad una connessione) quali 
                    // sono sorgente e destinazione dei dati.
                    // Per questo motivo nel caso di UDP diventa essenziale utilizzare queste due funzioni, 
                    // che sono comunque utilizzabili in generale per la trasmissione di dati attraverso 
                    // qualunque tipo di socket. Esse hanno la caratteristica di prevedere tre argomenti 
                    // aggiuntivi attraverso i quali e' possibile specificare la destinazione dei dati 
                    // trasmessi o ottenere l'origine dei dati ricevuti.
                    sendto(imgfd,(char *)&header,sizeof(header),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x01;
                    //...
                
                // il server invia al client (mediante socket UDP)
                // le immagini destra e sinistra e la mappa di disparita'
                // poi il client mediante un'unica recvfrom deinterlaccia 
                // le informazioni contenute in un unico pacchetto
                case (MUX_MODE_8_FPN_ODC_DISP | 0x2000): 
                case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x3000):
                    //...
                
                // il server invia al client (mediante socket UDP) solo la mappa di disparita'
                // poi il client mediante un'unica recvfrom deinterlaccia 
                // le informazioni contenute in un unico pacchetto
                case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100):
                case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x4100):
                    //...
        \endcode
        */
       
        if(connected && images_enabled)
        // se il win_client è connesso ed abilitato a ricevere immagini 
        // allora si procede con lo spedire la mappa di disparita' origianle o filtrata
        // con o meno i frame a seconda dell'opzione scelta da win_client
        {
            //gettimeofday(&stop,NULL); 
            //time_udp = (stop.tv_sec-start.tv_sec)*1000000 + stop.tv_usec-start.tv_usec;
            //gettimeofday(&start,NULL);
            
            int sendto_ret = 0; // 20111013 eVS, used for lost connection check
            //static int num_consecutive_failures = 0; // 20111013 eVS, used for lost connection check
                   
            switch(acq_mode)
            {
            case MUX_MODE_8_FPN: 
            case (MUX_MODE_8_FPN_ODC | 0x1000):
                //20100517 eVS aggiunto if
                //if (framecounter%4==0)
                if (framecounter%8==0) // 20120513 eVS, modified to reduce CPU usage
                {
                    //header = 0x03;
                    header = 0x23;
                    sendto_ret = sendto(imgfd,(char *)&header,sizeof(header),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x01;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);  
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_SX,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x02;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                
                    /////////////////////////
                    // 20091124 eVS
                    //sendto(imgfd,Frame_DX,NN,0,(struct sockaddr*)&remoteaddr_data,addr_len);
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_DX,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x20;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    //// 20111011 eVS, modified to avoid memory conflicts
                    ////sendto(imgfd,(char *)&count_true_false,sizeof(count_true_false),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&count_true_false_to_be_sent,sizeof(count_true_false_to_be_sent),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    // 20091124 eVS
                    /////////////////////////
                }
                break;
            case (MUX_MODE_8_FPN_ODC_DISP | 0x2000): 
            case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x3000):
            case (MUX_MODE_8_FPN_ODC_DISP_SOBEL | 0x2000):
            case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL | 0x3000):
                //20100517 eVS aggiunto if
                //if (framecounter%6==0) 
                if (framecounter%12==0) // 20120513 eVS, modified to reduce CPU usage
                {
                    //header = 0x07;
                    header = 0x27;
                    sendto_ret = sendto(imgfd,(char *)&header,sizeof(header),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x01;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);  
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_SX,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    ID = 0x02;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_DX,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    ID = 0x04;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);

                    /////////////////////////
                    // 20091124 eVS
                    //sendto(imgfd,Frame_DSP,NN,0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_DSP,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    ID = 0x20;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    //// 20111011 eVS, modified to avoid memory conflicts
                    ////sendto(imgfd,(char *)&count_true_false,sizeof(count_true_false),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&count_true_false_to_be_sent,sizeof(count_true_false_to_be_sent),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    // 20091124 eVS
                    /////////////////////////
                }
                break;
            case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x0100):
            case (MUX_MODE_8_FPN_ODC_MEDIAN_DISP | 0x4100):
                //20100517 eVS aggiunto if
                //if (framecounter%2==0)
                if (framecounter%6==0) // 20120513 eVS, modified to reduce CPU usage
                {
                    //header = 0x1C;
                    header = 0x3C;
                    
                    sendto_ret = sendto(imgfd,(char *)&header,sizeof(header),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    //num_consecutive_failures+= (sendto_ret < 0) ? 1 : 0;
                    
                    ID = 0x04;
                    if (sendto_ret > 0) 
                    sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);  
                   
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,Frame_DSP,NN,MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                   
                    ID = 0x08;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    
                    //sendto(imgfd,(char *)&counter_in,sizeof(counter_in),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&counter_in_to_be_sent,sizeof(counter_in_to_be_sent),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    
                    ID = 0x10;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    
                    /////////////////////////
                    // 20091124 eVS
                    //sendto(imgfd,(char *)&counter_out,sizeof(counter_out),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    //// 20111011 eVS
                    ////sendto(imgfd,(char *)&counter_out,sizeof(counter_out),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&counter_out_to_be_sent,sizeof(counter_out_to_be_sent),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    
                    ID = 0x20;
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&ID,sizeof(ID),MSG_MORE,(struct sockaddr*)&remoteaddr_data,addr_len);
                    
                    //// 20111011 eVS, modified to avoid memory conflicts                    
                    //sendto(imgfd,(char *)&count_true_false,sizeof(count_true_false),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    if (sendto_ret > 0) sendto_ret = sendto(imgfd,(char *)&count_true_false_to_be_sent,sizeof(count_true_false_to_be_sent),0,(struct sockaddr*)&remoteaddr_data,addr_len); 
                    // 20091124 eVS
                    /////////////////////////
                }
                break;
              }
               
        }
        
        wd_check = 1;		// the main loop loop is alive
        
        //////////////////////
        // 20091123 eVS
#ifdef MEASUREMENTS
        if (framecounter%MAX_MEASURES_PER_FILE==0)
        {
            time_t curtime_wg;
            struct tm *loctime_wg;
            char filename[255];
            char command[200];
                
            fclose(measurements_file);
            
            curtime_wg = time (NULL);
            loctime_wg = localtime (&curtime_wg);
            sprintf(filename,"measurements%02d%02d%04d-%02d%02d%02d.txt",
                loctime_wg->tm_mday,loctime_wg->tm_mon+1,1900+loctime_wg->tm_year,
                loctime_wg->tm_hour,loctime_wg->tm_min,loctime_wg->tm_sec); 

            sprintf(command,"mv /var/neuricam/measurements.txt /var/neuricam/%s", filename);
            system(command);	// rename the oldest file
            
            measurements_file = fopen("/var/neuricam/measurements.txt","w"); // create a new file            
        }
#endif
        // 20091123 eVS
        //////////////////////
        
        pthread_mutex_unlock(&acq_mode_lock); // 20100517 eVS
    }

    //////////////////////
    // 20091123 eVS
#ifdef MEASUREMENTS
    fclose(measurements_file);
    {
        time_t curtime_wg;
        struct tm *loctime_wg;
        char filename[255];
        char command[200];
        
        curtime_wg = time (NULL);
        loctime_wg = localtime (&curtime_wg);
        sprintf(filename,"measurements%02d%02d%04d-%02d%02d%02d.txt",
            loctime_wg->tm_mday,loctime_wg->tm_mon+1,1900+loctime_wg->tm_year,
            loctime_wg->tm_hour,loctime_wg->tm_min,loctime_wg->tm_sec); 

        sprintf(command,"mv /var/neuricam/measurements.txt /var/neuricam/%s", filename);
        system(command);	// rename the oldest file
    }
#endif
    // 20091123 eVS
    //////////////////////
    
    thread_status = MAINLOOP_STOP;

    close(imgfd);

    return (void *) thread_status;
}



/*! 
\brief Thread to record logs and counter values.

This thread every #RECORD_INTERVAL seconds saves the buffer #records[] 
into the file #recordfd named #record_fn. The buffer #records
contains log messages generated by the application run-time.
The filename #record_fn is composed by the prefix #rd_filename 
and by a suffix corresponding to the number in 
#record_id. Such a number is between 0 and #MAX_REC_FILES-1.
When a file exceeds #MAX_REC_LINES a new file is created after 
incrementing #record_id. When #record_id gets greater than #MAX_REC_FILES
it is set to 0 and the old file is deleted.

Moreover, this function saves also the current values of the in and out 
counters into the file named  #cr_filename.
*/
void *record_loop(void *arg)
{
    unsigned int i;
    unsigned long lines;
    unsigned long num_elem; // 20100518 eVS, added
    char command[256];
    FILE *recordfd;
    FILE *counterfd;

    // eVS 20100423 add a control on the open_door flag
    while(1)
    {
        sleep(RECORD_INTERVAL);
        pthread_mutex_lock(&rdlock);
        //if(records_idx)
        //if(records_idx && !count_enabled && ev_door_close==false) // 20100424 eVS added condition on counting
        if(records_idx) // 20100518 eVS removed the previously added condition to get back the previous version
        {
            records_saving_in_progress = true; // 20111012 eVS, added
            
            recordfd = fopen(record_fn,"a+");
            fseek(recordfd,0L,SEEK_END);
            lines = ftell(recordfd)/REC_LINE_LEN;

            // if the file containing the account has already passed the MAX_REC_LINES lines will then create another with
            // the same prefix number but larger (record_id)
            // se il file contenente i conteggi ha gia superato le MAX_REC_LINES righe allora ne creo un'altro con lo
            // stesso prefisso ma numero progressivo maggiore (record_id)
            if(lines >= MAX_REC_LINES)
            {
                fclose(recordfd);	// the records file is full.
                sprintf(record_fn,"%s%ld.txt",rd_filename,++record_id);

                if(record_id >= MAX_REC_FILES)
                {
                    sprintf(command,"rm %s%ld.txt",rd_filename,record_id-MAX_REC_FILES);
                    system(command);	// removing the oldest file
                } 

                recordfd = fopen(record_fn,"a+");
            }
            num_elem=records_idx; // 20100518 eVS, records_idx coulb be changed in other threads when unlocked
            pthread_mutex_unlock(&rdlock); // 20100518 eVS, added
            
            i=0; //for(i=0;i<records_idx;i++) { 20100518 eVS, changed "for" in a "while"
            while (i<num_elem)
            {
                pthread_mutex_lock(&rdlock); // 20100518 eVS, added
                fprintf(recordfd,"%s",records[i]);
                num_elem = records_idx; // 20100518 eVS, records_idx coulb be changed in other threads when unlocked
                pthread_mutex_unlock(&rdlock); // 20100518 eVS, added
                i++;
            }
            
            pthread_mutex_lock(&rdlock); // 20100518 eVS, added
            fclose(recordfd);
            records_idx = 0; 

            records_saving_in_progress = false; // 20111012 eVS, added 
            
            // 20100424 eVS moved out the if in order to save counters everytime
            //-------  saving the two people counters  --------//
            /*if((counterfd = fopen(cr_filename,"w+")))
            {
                fprintf(counterfd,"counter_in %ld\n",counter_in);
                fprintf(counterfd,"counter_out %ld\n",counter_out);
                fclose(counterfd); 
            }*/
            //-------------------------------------------------//
        }       
        
        // 20100424 eVS, moved here!
        //-------  saving the two people counters  --------//
        if((counterfd = fopen(cr_filename,"w+")))
        {
            fprintf(counterfd,"counter_in %ld\n",counter_in);
            fprintf(counterfd,"counter_out %ld\n",counter_out);
            fclose(counterfd); 
        }
        //-------------------------------------------------//
        
        pthread_mutex_unlock(&rdlock);
    }
}


/*!
\brief Thread to monitor optocoupled input 0

This thread continuously checks for the signal from
the optocoupled input 0. Once the signal is taken it passed to
the previously set input_function0(). 

In the case of wide-gate mode, if this PCN is not the last,
the signal is sent to the next PCN.
*/
void *input_loop0(void *arg)
{
    unsigned char value;
    while(1)
    {
        //read(pcin0,&value,sizeof(value));	
        value = get_gpio("GPLR3_096"); // 20120117 eVS, used the new method to read the digital input
        
        input_test0 = value;
        
        //segnale porta
        if(total_sys_number>0 && current_sys_number!=total_sys_number) //se non sono l'ultimo
        {//faccio il ponte del segnale porta
            if(value==1)
                set_gpio("GPSR3_090",1);
            else
                set_gpio("GPCR3_090",1);
        }

        input_function0(value);
        usleep(10); // 20120712 eVS, added to get more CPU to the main thread
    }
}


/*!
\brief Thread to monitor optocoupled input 1

Very similar to #input_loop0() but replacing 0 with 1.
Moreover, this thread handles also a counter named #count_sincro
used to sincronize all the PCN those monitor the same gate in 
wide-gate mode.

From this function one can notice that #current_sys_number
has to be zero when the wide-gate mode is off otherwise
the #count_sincro is incremented unnecessarily.

\see input_loop0()
*/
void *input_loop1(void *arg)
{
    unsigned char value;
    while(1)
    {
        //read(pcin1,&value,sizeof(value));	
        value = get_gpio("GPLR2_095"); // 20120117 eVS, used the new method to read the digital input

        input_test1 = value;

        if(current_sys_number==1) //if I'm the first
            count_sincro++;

        else if((current_sys_number<total_sys_number) && (current_sys_number>1))// if I'm in the middle => I'm a bridge
        {
            if(value==1)
                set_gpio("GPSR3_091",1);
            else
                set_gpio("GPCR3_091",1);

            count_sincro++;
        }
        
        input_function1(value);
        usleep(10); // 20120712 eVS, added to get more CPU to the main thread
    }
}


/*!
\brief Scan loop

Just a ping loop from/to the win_client application used by win_client during the "Scan" phase.
*/
void *ping_loop(void * arg)
{
    socklen_t addr_len;
    struct sockaddr_in remote;	//client address
    char buf[MAX_STR_LENGTH];

    struct sockaddr_in myaddr_data;  // 20111014 eVS, moved here // Indirizzo lato server associato alla socket di tipo UDP per la comunicazione client/server (mediante connessione usb).
    int datagram;   // 20111014 eVS, moved here //File descriptor associato alla socket di tipo UDP (datagram = socket(AF_INET, SOCK_DGRAM, 0)).

    /*
    <b>Socket di tipo UDP</b>
    \code
    // Viene usato per trasmettere pacchetti di dati (datagram) di lunghezza massima prefissata, 
    // indirizzati singolarmente. Non esiste una connessione e la trasmissione e' effettuata
    // in maniera non affidabile.
    datagram = socket(AF_INET, SOCK_DGRAM, 0);
    \endcode */
    
    // datagram socket
    if ((datagram = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        perror("socket");
        exit(1);
    }

    myaddr_data.sin_family = AF_INET;
    myaddr_data.sin_addr.s_addr = INADDR_ANY;
    myaddr_data.sin_port = htons(PORT);

    memset(&(myaddr_data.sin_zero), '\0', 8);

    /*! \code
    bind(datagram, (struct sockaddr *)&myaddr_data, sizeof(myaddr_data);
    \endcode */    
    if (bind(datagram, (struct sockaddr *)&myaddr_data, sizeof(myaddr_data)) == -1)
    {
        perror("bind");
        print_log("Exit! Bind failed!\n");
        exit(1);
    }
    
    
    while(1)
    {       
        addr_len = sizeof(struct sockaddr);
        int ret = recvfrom(datagram,buf,MAX_STR_LENGTH-1,0,(struct sockaddr *)&remote, &addr_len);

        if (ret >= 0) // 20111014 eVS, added check
        {
          pthread_mutex_lock(&acq_mode_lock); // 20111012 eVS, added
          bool is_connected = connected;
          pthread_mutex_unlock(&acq_mode_lock); // 20111012 eVS, added

          //if(!connected) // 20111012 eVS, substituted with a copy mutually accessed
          if(!is_connected) 
              sendto(datagram,"Cam\0",4,0,(struct sockaddr*)&remote,addr_len);
          /*else
              print_log("Connected is true but ping loop received something!\n");*/
        }
        usleep(100); // 20120712 eVS, added to get more CPU to the main thread
    }
}


/*!
\brief Thread #serloopttyS0 waits for messages on the #ttyS0 (from its master 
or rs485_gui or the collector) and when a message is received, it 
answers properly. 

This thread continuously waits for messages on the #ttyS0 using SNP_Recv(). 

After reception, the message is virtually split into a command string and
the argument of that command args. More precisely, the two obtained parameters 
(command and args) passed to SNP_reply() are not two different vectors but two 
different pointers pointing to two different locations in the same 
received buffer.

\code
// split the message in command plus arguments
command = (char *)&buf[8];
args = (char *)&buf[8+(strlen((char *)&buf[8])+1)];
args_size = datalen-(strlen((char *)&buf[8])+1);
\endcode

Since the command string has to be inserted in the buffer with the termination
character, the resulting behavior of command is just like a string containing 
the command string. For example, even in the shown code the behavior of strlen()
is just the one expected.

After splitting the message, the extracted command and argument 
are passed to SNP_reply() in order to react properly to the 
received message.
*/
void *ser_loopttyS0(void *arg)
{
    int datalen;
    unsigned int args_size;
    char *command,*args;
    unsigned char sender,recipient;
    static char buf[MAX_PACKET_LEN];

    while(ttyS0 > 0)
    {  
start_loop_ttyS0:
        //usleep(100); // 20120712 eVS, added to get more CPU to the main thread
        
        memset(buf,0,sizeof(buf)); //MAX_PACKET_LEN); 20100512 eVS
        
        // si tratta di un ciclo che continua finche non riesce a prelevare
        // un pacchetto dalla porta seriale 0
        if((datalen = SNP_Recv(buf,&sender,&recipient,ttyS0)) < 0) {
            //print_log("Not received!\n");
            goto start_loop_ttyS0;
        }
        
        // se riceve una stringa di lunghezza zero allora inserisce in buf il comando "polling"
        if(!datalen) strcpy(&buf[8],"polling");

        /////////////////
        // eVS
        //print_log("%s received\n", (char *)&buf[8]);
        // eVS
        /////////////////

        if(datalen >= 0)
        {
            // split the message in command plus argument
            command = (char *)&buf[8];
            args = (char *)&buf[8+(strlen((char *)&buf[8])+1)];
            args_size = datalen-(strlen((char *)&buf[8])+1);

            //print_log("Received command %s\n", command);
            
            // interpret the command and give the proper answer
            SNP_reply(sender,recipient,command,args,args_size,ttyS0);
        }
    }
    return NULL;
}


/*!
\brief Thread #serloopttyS1 waits for messages on the #ttyS1 (from the 
slave) and when a message is received, it answers properly. 

Exactly the same as ser_loopttyS0() but it works on the #ttyS1 
instead of #ttyS0.

\see ser_loopttyS0().
*/
void *ser_loopttyS1(void *arg)
{
    int datalen;
    unsigned int args_size;
    char *command,*args;
    unsigned char sender,recipient;
    static char buf[MAX_PACKET_LEN];

    while(ttyS1 > 0)
    {  
start_loop_ttyS1:
        //usleep(100); // 20120712 eVS, added to get more CPU to the main thread
        
        memset(buf,0,sizeof(buf)); //MAX_PACKET_LEN); 20100512 eVS
        if((datalen = SNP_Recv(buf,&sender,&recipient,ttyS1)) < 0) {
            //print_log("??? received something ???\n");
            goto start_loop_ttyS1;
        }
        if(!datalen) strcpy(&buf[8],"polling");

        if(datalen >= 0)
        {
            command = (char *)&buf[8];
            args = (char *)&buf[8+(strlen((char *)&buf[8])+1)];
            args_size = datalen-(strlen((char *)&buf[8])+1);

            SNP_reply(sender,recipient,command,args,args_size,ttyS1);
        }
    }
    return NULL;
}



/*! 
\brief Enable/Disable main_loop().

This function is used to enable or disable the main_loop() thread.

If the request is to enable the main_loop(), this function checks if  
the thread is already running or not, if the thread is running then this
function does nothing but if the thread is not running,
i.e. thread_status equals to MAINLOOP_STOP which is less then 
MAINLOOP_START, it is started just creating a thread corresponding
to main_loop().

If the request is to disable the main loop then this function changes 
the thread status and waits the end of the thread and then deletes it.

\param enable TRUE to enable and FALSE to disable the main_loop().
*/
void mainloop_enable(bool enable)
{
/*!
\code
    if(enable)
    {
        if(thread_status < MAINLOOP_START) 
        {
            //starting the acquisition thread
            thread_status = MAINLOOP_START;
            pthread_create (&mloop, NULL, main_loop, NULL);
        }
    }
\endcode
*/
    if(enable)
    {
        // ??? non e' meglio mettere != invece di < altrimenti qui si usa la conoscenza su
        // cosa contengono le due define di START e STOP del main loop
        
        //if(thread_status < MAINLOOP_START) //starting the acquisition thread
        if(thread_status != MAINLOOP_START) //starting the acquisition thread
        {
            //wg_check = 0; // 20100524 eVS added
            thread_status = MAINLOOP_START;
            pthread_create (&mloop, NULL, main_loop, NULL);
        }
    }
    else
    {
        thread_status = MAINLOOP_STOP;  		//stopping the acquisition thread
        pthread_join(mloop,NULL);			//waiting until the thread terminates
        pthread_cancel(mloop);
        //wg_check = 1; // 20100524 eVS added
    }
}


/*!
\brief Watchdog loop.

Thread used to continuously update the watchdog variable wd_check.
This variable is set to 1 at the end of each cycle of the main loop
and here it is set to 0 once checked it is 1.
*/
void *watchdog_loop(void *arg)
{
    while(watchdog > 0)
    {
        sleep(WD_INTERVAL);
        if(thread_status == MAINLOOP_START)
        {
            pthread_mutex_lock(&acq_mode_lock); // 20111014 eVS added
            bool is_main_loop_active = wd_check;
            pthread_mutex_unlock(&acq_mode_lock); // 20111014 eVS added
            
            //if(wd_check) // checking if the mainloop is alive
            if(is_main_loop_active)
            {
                ioctl(watchdog, WDIOC_KEEPALIVE, 0);
                
                pthread_mutex_lock(&acq_mode_lock); // 20111014 eVS added
                wd_check = 0;
                pthread_mutex_unlock(&acq_mode_lock); // 20111014 eVS added
            }
        }
        else ioctl(watchdog, WDIOC_KEEPALIVE, 0);
    }
    return NULL;
}


/*!
\brief Decides the led power on the base of the average gray level of the left and right images.

Experimentally we could notice that the mean value changes in this way:
- at the same light condition and with no people, there is no remarkable difference if led are on or off in good 
conditions (only at most 2 level of gray difference), sensible differences (around 4-5) arise when
light is weak and the mean value goes under 80.
- leds have a short range influence, i.e., their influence at floor level can only be seen when the mean value goes 
under 80-90 (or less) but at short distances (with people head) can already be seen around 100 with even more 
than 10 level of gray between led on and led off.

Therefore:
- the threshold for autoled has been set at 90.
*/
inline void autoled_management(
    const unsigned int i_sx_vm_img, 
    const unsigned int i_dx_vm_img)
{
    #define LED_INERTIA_INTERVAL 3
    #define LED_MAX_VALUE 255
    #define LED_STEP 25
    #define MIN_AUTOLED 90

    static int inertia_on = 0; // must be initialized to 0
    static int inertia_off = 0; // must be initialized to 0
    
    unsigned short current_value = get_parms("sled");
    unsigned short new_value = current_value;
       
    //print_log("vm_sx = %d,\t vm_dx = %d\n", i_sx_vm_img, i_dx_vm_img);
    
    if (i_sx_vm_img<MIN_AUTOLED && i_dx_vm_img<MIN_AUTOLED) // se le immagini sono entrambe scure
    {
        if (current_value < LED_MAX_VALUE) // e i led non sono ancora a massima potenza
        {
            // provo ad accenderli un po' di piu'... con una certa inerzia, non subito
            inertia_on++;
            inertia_off = 0;
            if (inertia_on > LED_INERTIA_INTERVAL) {
                new_value = (current_value+LED_STEP > LED_MAX_VALUE) ? LED_MAX_VALUE : current_value+LED_STEP;
            }
        }
    } 
    else // se le immagini non sono scure 
    { 
        if (current_value > 0) // e i led sono accesi
        {
            // provo ad spegnerli un po'... con una certa inerzia, non subito
            inertia_off++;
            inertia_on = 0;        
            if (inertia_off > LED_INERTIA_INTERVAL) {
                new_value = (current_value-LED_STEP < 0) ? 0 : current_value-LED_STEP;
            }
        }
    }
    
    if (new_value != current_value) 
    {
        set_parms("sled", new_value);
        write_parms("sled", new_value);
        inertia_on = 0;
        inertia_off = 0;
    }
}


/*!
\brief Diagnostic checks related to lens obfuscation or image sensors failure.

Experimentally (using two PCN in various light conditions) we could notice that the mean value 
changes in this way:
- it is around 45-53 when obscured using an occluding object in front of the lenses
- it is around 45-53 also when it is very dark without led
- it is around 66-80 when very dark but with led at maximum power (in this condition there is a lot of noise and possible false counting)
- it is around 100-110 in indoor good conditions (with artificial lights)
- it is around 110-120 in indoor good conditions (with artificial lights) plus
light from windows in a cloudy day
- not tested in outdoor
- when the mean value get lower and lower the fixed noise get more and more visible
and this could compromise the counting
- the two sensors can have different mean values because of a different calibration of the vref and
because the see a slightly different scene, so the threshold on the difference of the two sensor has
to be large enough to cope with this problem. The observed difference (which is not linear on
the mean value but changes because of the logarithmic response of the used sensors) arrived at
most at 10.

Therefore:
- the threshold for total obfuscation THR_VM_DIAGN_MIN or cmos failure has been set at 55
- we added a threshold THR_VM_DIAGN for darkness which has been set at 85 (when very dark and led at maximum power
the image is very noisy and the counting results could be compromised; at least one light somewhere
close to the PCN has to be switched on: complete darkness has to be avoided)
- the threshold on the difference has been set at 20 (this can happen if one lens is dirty or for calibration problems).

The error code returned is:
- 0 if no problem arised
- 1 (bit 0 set to 1) if right sensor is completely occluded or broken (<=THR_VM_DIAGN_MIN)
- 2 (bit 1 set to 1) if left sensor is completely occluded or broken (<=THR_VM_DIAGN_MIN)
- 3 if both sensors are completely occluded or broken
- 4 (bit 2 set to 1) if right sensor is too dark (<=THR_VM_DIAGN)
- 8 (bit 3 set to 1) if left sensor is too dark (<=THR_VM_DIAGN)
- 12 if both sensors are too dark
- 16 (bit 4 set to 1) when the difference between the two images is relevant (maybe one lens is dirty)
- 20, 24, 28 when the difference between the two images is relevant and it is combined with darkness in the sensors
*/
inline bool check_pcn_status(
    const unsigned int i_sx_vm_img, //<! [in] mean value of the left image
    const unsigned int i_dx_vm_img, //<! [in] mean value of the right image
    const bool i_autoled, //<! [in] autoled flag (if true then autoled is enabled)
    unsigned char& o_error_code) //<! [out] contain the error code (0 if ok)
{
    #define ERROR_STATE_INERTIA 10
    #define THR_VM_DIAGN_MIN 55 // 20111209 eVS, added new threshold
    #define THR_VM_DIAGN 85 // 20101029 eVS moved here
    #define THR_VM_DIAGN_DIFF 20 // 20101029 eVS moved here
    
    static unsigned char accumula = 0;
    static unsigned char prev_error_code = 0x00;
    
    unsigned char error_code = 0x00;

    if(i_dx_vm_img <= THR_VM_DIAGN_MIN) // controllo oscuramento/guasto a destra
        error_code |= 0x01; // 0000 0001
    if(i_sx_vm_img <= THR_VM_DIAGN_MIN) // controllo oscuramento/guasto a sinistra
        error_code |= 0x02; // 0000 0010
    if(error_code != 0x00 && error_code != 0x03 && i_sx_vm_img <= THR_VM_DIAGN_MIN+5 && i_dx_vm_img <= THR_VM_DIAGN_MIN+5 )
        error_code |= 0x03; // 0000 0011
    
    if(error_code == 0x00) // se e' 0 significa che non c'e' ne' guasto nei sensori ne' buio totale
    {
        if(i_dx_vm_img <= THR_VM_DIAGN) // controllo se c'e' troppo buio a destra
            error_code |= 0x04; // 0000 0100                   
        if(i_sx_vm_img <= THR_VM_DIAGN) // controllo se c'e' troppo buio a sinistra
            error_code |= 0x08; // 0000 1000
        if(error_code != 0x00 && error_code != 0x0C && i_sx_vm_img <= THR_VM_DIAGN+5 && i_dx_vm_img <= THR_VM_DIAGN+5 )
            error_code |= 0x0C; // 0000 1100
            
        if (abs((int)i_dx_vm_img-(int)i_sx_vm_img) >= THR_VM_DIAGN_DIFF) // verifico differenza di luminosità tra le due immagini
        {
            // le due immagini sono molto dissimili
            error_code |= 0x10; // 0001 0000
        }
        else
        {
            // le immagini hanno medie abbastanza simili
            if (error_code == 0x0C &&  // se le immagini sono entrambe buie (cioe' error code 0000 1100) e...
                i_autoled &&         // l'autoled e' abilitato e...
                get_parms("sled") < LED_MAX_VALUE) // il valore massimo dei led non e' stato ancora raggiunto allora...
            {
                error_code = 0x00; // aspetto di mettere i led al massimo prima di dare errore
            }
        }
    }
              
    if(error_code != prev_error_code)
        accumula++; 
    else
        accumula = 0;
    
    if (accumula < ERROR_STATE_INERTIA)
      o_error_code = prev_error_code;
    else {
      o_error_code = error_code;
      prev_error_code = error_code;
      accumula = 0;
    }
      
    return !o_error_code;
}

