/*!
\file commands.cpp
\brief Ricezione ed esecuzione dei comandi inviati dal client (mediante socket).

Per i dettagli vedere la Communication().

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/


/*! 
\brief Ricezione ed esecuzione dei comandi inviati dal client (mediante socket TCP).

Questa funzione ha come input una stringa proveniente dal 
win_client via socket TCP (vedi socket.cpp) precedentemente ottenuta con la funzione RecvString(). 
Viene effettuato un parsing di tale stringa per comprendere di che comando si tratta; una volta 
noto il comando, rimane in attesa per mezzo di una Recv() di un pacchetto di dati di cui effettua 
un opportuno parsing per poter cos&igrave; andare a modificare lo stato dei device e/o delle variabili 
globali dell'imgserver. Infatti, ogni comando che si trova nella GUI del client (pulsanti, 
toggle, ecc...) corrisponde in linea di massima ad una SendString() di una opportuna stringa che 
indica il comando/azione seguita poi da una Send() con le impostazioni da usare per poter agire 
sullo image server o sui device.

\param fd file descriptor relativo al socket per la comunicazione win_client/imgserver.
\param buffer comando in formato stringa
\return zero se l'esecuzione del comando ha avuto successo.
*/
#include "directives.h"
#include "OutOfRangeManager.h"
#include "record_utils.h"
#include <assert.h>

extern unsigned char Frame_DX[NN];   //!< Immagine destra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_SX[NN];   //!< Immagine sinistra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_DSP[NN];  //!< Mappa di disparit&agrave; #NX*#NY=160*120 (16 livelli di disparit&agrave; distribuiti su 256 livelli di grigio).


////////////////////////////////////////////////////////////////////////////////////////////////
void _enable_out_of_range_handle( bool state)
{
  // stop mainloop before enable OOR  
  mainloop_enable(0);
  OutOfRangeManager::getInstance().SetEnableStateOutOfRange(state);
  // restart mainloop      
  mainloop_enable(1); 

}
bool
_is_background_reliable_for_out_of_range_handle()
{
  // stop mainloop before background checking 
  mainloop_enable(0); 
  
  // be sure that FPGA inserts the disparity map inside the quick capture interface buffer
  i2cstruct.reg_addr =  MUX_MODE;
  i2cstruct.reg_value = MUX_MODE_8_FPN_ODC_MEDIAN_DISP;
  ioctl(pxa_qcp, VIDIOCSI2C, &i2cstruct);
  for (int i=0; i<5; ++i)
    read(pxa_qcp, Frame, imagesize);	// the first image is dirty (just read more than once to be sure)

  // verify background  
  bool is_backgroung_reliable;
  int num_unrealiable_pixels = 0; 
  int num_processed_frames = 0;
  do {
    read(pxa_qcp, Frame, imagesize); 
    get_images(Frame,0);    
    num_unrealiable_pixels = OutOfRangeManager::getInstance().CountBlackPixels(Frame_DSP);
    num_processed_frames++;
  } while (OutOfRangeManager::getInstance().isBackgroundCheckInProgress(num_unrealiable_pixels, is_backgroung_reliable, num_processed_frames == 1));

  printf("BackgroundCheck (%s, %d)\n", (is_backgroung_reliable) ? "OK" : "ko", num_unrealiable_pixels);

  // restart mainloop      
  mainloop_enable(1); 
  
  return (is_backgroung_reliable);
}

void
_send_disp_4bit(int i_fd, unsigned char* const & i_disp_map, 
                const unsigned long i_cnt[2], 
                unsigned char i_input_test0, unsigned char i_input_test1)
{
  assert(NN % 2 == 0);
  static unsigned char buf[NN/2 + 2*sizeof(unsigned long) + 2]; // buffer for the 4bit image + counters + digital input
  unsigned char* ptrd = i_disp_map;
  unsigned char* ptrb = buf;
  
  for (int i=0; i<NN-1; i+=2, ptrd+=2, ptrb++)
  {
    unsigned char elem1 = *ptrd;
    unsigned char elem2 = *(ptrd+1);
    *ptrb = ((elem1 & 0xF0) | ((elem2 >> 4) & 0x0F));
  }
  
  for (int j=0; j<2; ++j)
  {
    unsigned char* ptrcnt = (unsigned char*)&i_cnt[j];
    for (unsigned int i=0; i<sizeof(i_cnt[j]); ++i, ++ptrb, ++ptrcnt)
      *ptrb = *ptrcnt;
  }
    
  *ptrb++ = i_input_test0;
  *ptrb = i_input_test1;
    
  Send(i_fd,(unsigned char *)buf, sizeof(buf));    
}


//////////////////////////////////////////////////////////////////////////////////////////////////
int Communication(int fd,char *buffer)
{
    char mystr[MAX_STR_LENGTH];
    unsigned char pic_width;
    unsigned char pic_height;
    int ret;

    /*! \code
    // ogni timebkg minuti viene automaticamente aggiornato lo sfondo
    if(strcmp(buffer,"timebkg")==0)
    \endcode */

    if(strcmp(buffer,"timebkg")==0)
    {
        unsigned char value;
        Recv(fd,(int *)&value,sizeof(value));
        if(value>59) value=59;
        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;
        if(total_sys_number>1 && current_sys_number==1)
        {
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
        }
        ret=write_parms(buffer,(unsigned short)value);
        if(ret==0) 
            ret=save_parms(buffer,(unsigned short)value);
        send_enable=send_old;
        return ret;
    }

    /*! \code
    // soglia statica affinche' il nuovo sfondo venga considerato come un cambiamento di scena
    if(strcmp(buffer,"staticth")==0)
    \endcode */

    //threshold value of acceptable scene background change
    if(strcmp(buffer,"staticth")==0)
    {
        int value;
        Recv(fd,(int *)&value,sizeof(value)); 
        if(value<0) value=0;
        if(value>19200) value=19200;

        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;

        if(total_sys_number>1 && current_sys_number==1)
        {
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
        }
        ret=write_parms(buffer,(unsigned short)value);
        if(ret==0) 
            ret=save_parms(buffer,(unsigned short)value);
        send_enable=send_old;
        return ret;
    }   

    if(strcmp(buffer,"win")==0)		// selects images size (unused)
    {     
        Recv(fd,(unsigned char *)&pic_width,sizeof(pic_width));
        Recv(fd,(unsigned char *)&pic_height,sizeof(pic_height));

        vid_win.width = pic_width;
        vid_win.height = pic_height;
        imagesize = vid_win.width*vid_win.height;

        ioctl(pxa_qcp,VIDIOCSWIN,&vid_win);
        return 0;
    } 

    /*!
    \code
    // Acquisizione immagini: quando il PCN riceve il comando di start seguito dal valore 1, 
    // inizia ad inviare le immagini e il valore dei conteggi di ingresso/uscita mediante il protocollo UDP. 
    // Dopo il comando start, il client deve aprire una socket UDP e creare un thread che legge 
    // i dati che arrivano sulla porta 5402. Il sistema ferma l'invio dei dati al client quando 
    // riceve il comando start seguito dal valore 0.
    if(strcmp(buffer,"start")==0)
    {   
        unsigned char value;

        Recv(fd,(char *)&value,sizeof(value)); 
        if(value)
        {
            // abilita invio immagini
            images_enabled = 1; 
            mainloop_enable(1);
        }
        else
        {
            // disabilita invio immagini
            images_enabled = 0;
            mainloop_enable(0);
        }
        return 0; 
    }
    \endcode
    */
    if(strcmp(buffer,"start")==0)		// main loop starts sending images
    {   
        unsigned char value;

        Recv(fd,(char *)&value,sizeof(value)); 
        if(value)
        {
            images_enabled = 1; //enabling sending images
            mainloop_enable(1);
        }
        else
        {
            images_enabled = 0;
            mainloop_enable(0);  //stopping the acquisition thread
        }
        return 0; 
    }

    /*! \code
    // restituisce la versione corrente dell'imgserver
    if(strcmp(buffer,"version")==0)
    \endcode */
    if(strcmp(buffer,"version")==0)		// returns imgserver version
    {   
        SendString(fd,VERSION);
        return 0;
    }

    /*! \code
    // restituisce la versione corrente del sistema operativo
    if(strcmp(buffer,"sys_version")==0)
    \endcode */
    if(strcmp(buffer,"sys_version")==0)		// returns APC OS version
    {   
        char ver[32];
        sys_version(ver);

        SendString(fd,ver);
        return 0; 
    }

    /*! \code
    // restituisce la versione corrente del bitstream (FPGA)
    if(strcmp(buffer,"fw_version")==0)
    \endcode */
    if(strcmp(buffer,"fw_version")==0)		// returns FPGA bitstream version
    {   
        char ver[32];
        sprintf(ver,"%2.1f",fw_version());
        SendString(fd,ver);
        return 0; 
    }

    /*! \code
    // restituisce la versione corrente del kernel
    if(strcmp(buffer,"ker_version")==0)
    \endcode */
    if(strcmp(buffer,"ker_version")==0)		// returns the kernel version
    {   
        char ver[32];
        ker_version(ver);

        SendString(fd,ver);
        return 0; 
    }
   
    /*! \code
    // restituisce il file relativo ai parametri per la correzione della distorsione
    if(strcmp(buffer,"odc")==0)
    \endcode */
    if(strcmp(buffer,"odc")==0)			// receive ODC parameters file
    {  
        unsigned char wrsel;
        i2cstruct.reg_addr =  FL_WRSEL;
        ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
        wrsel=i2cstruct.reg_value;
        Recv(fd,(char *)ODCbuf,sizeof(short)*NN*8);
        if(wrsel==0x02)  
        {
            mainloop_enable(0);	// stopping mainloop before saving the background
            fpn_counter = 0;
            save_fpn();
            usleep(300000);
            mainloop_enable(1);		// restarting the mainloop
            usleep(300000);
            images_enabled = 1;
            mainloop_enable(1);		// restarting the mainloop
            memset(ODCbuf,0,sizeof(ODCbuf));  // resetting ODC parameters buffer
        }
        return 0;
    }

    /*! \code
    // reset dei contatori delle persone entrate/uscite dalla zona monitorata
    if(strcmp(buffer,"reset")==0)		
    \endcode */
    if(strcmp(buffer,"reset")==0)		
    {
        reset_counters(0);
        return 0;
    }

    /*! \code
    // restituisce i due contatori (persone entrate e uscite)
    if(strcmp(buffer,"gcounters")==0)		
    \endcode */
    if(strcmp(buffer,"gcounters")==0)		
    {
        unsigned char send_old=send_enable;
        send_enable=0;
        if(total_sys_number>1 && current_sys_number==1)
        {
            SNP_Send(slave_id,buffer,NULL,0,ttyS1);
            usleep(TIMEOUT_485); //wait answer
        }

        Send(fd,(char *)&counter_in,sizeof(counter_in));
        Send(fd,(char *)&counter_out,sizeof(counter_out));

        send_enable=send_old;      

        return 0;
    }

    /*! \code
    // Ripristino della configurazione di fabbrica
    // per le luci, optocoupled input functions, tempo di apertura della GPO ed RS485,
    // direzione di input/output, soglia porta e altezza di installazione.
    // Dopo questa operazione e' consigliabile risincronizzare il client con il server
    // usando il comando gparms.
    
    // codice lato client
    SendString(sock, "restore");

    // codice lato server
    if(strcmp(buffer,"restore")==0)		
    \endcode */
    if(strcmp(buffer,"restore")==0)		
    {  
        //char command[32];
        unsigned char send_old;
               
        // 20100520 eVS azzero flag send_enable momentaneamente per 
        // "evitare conflitti" (usare un mutex sarebbe meglio!!!) nel 
        // main_loop (cerca "send_enable" in loops.cpp e troverai un if)
        pthread_mutex_lock(&mainlock); // 20100524 eVS added
        send_old = send_enable;
        //send_enable=0;
        pthread_mutex_unlock(&mainlock); // 20100524 eVS added

        // 20100524 eVS used common function to avoid redundancy
        restore_factory_settings(buffer, true);
        /*        
        if(total_sys_number>1 && current_sys_number==1)
        {
            SNP_Send(slave_id,buffer,NULL,0,ttyS1);
            usleep(TIMEOUT_485);
        }

        load_default_parms();      
        
        pthread_mutex_lock(&mainlock);
        count_enabled=1;
        pthread_mutex_unlock(&mainlock);
        
        sprintf(command,"rm -rf %s",pm_filename);
        system(command);  				// deleting parameters file
        */
        
        // 20100520 eVS ripristino valore della flag send_enable
        pthread_mutex_lock(&mainlock); //20100524 eVS added
        send_enable=send_old;
        pthread_mutex_unlock(&mainlock); //20100524 eVS added
            
        return 0;
    }

    /*! \code
    // Cancellazione della memoria flash dell'FPGA
    if(strcmp(buffer,"erase")==0)		
    \endcode */
    if(strcmp(buffer,"erase")==0)		
    {          
        erase_flash();
        return 0;
    }

    /*!
    \code
    // Aggiornamento dello sfondo: per ottenere una mappa di disparita' pulita (senza oggetti sullo sfondo) 
    // e' necessario eseguire una rimozione dello sfondo dalla scena acquisita. Questa operazione richiede 
    // tre passi e quindi tre comandi differenti: sbackI, sbackS and sbackE. 
    // Il primo inizializza la procedura, il secondo acquisisce un nuovo sfondo per aggiornare quello vecchio 
    // (in base a dei pesi) e il terzo comando salva il nuovo sfondo sulla memoria non volatile.
    if(strncasecmp(buffer,"sback",5)==0)
    {         
        // aggiornamento dello sfondo mediante l'acquisizione
        // di 200 immagini mediate con un peso di 1/15 con lo sfondo vecchio
        // ...
        // il main_loop viene disabilitato
        // ...
        // aggiornamento dello sfondo..
        // ...
        // il main_loop viene abilitato
        // ...
    \endcode */

    if(strncasecmp(buffer,"sback",5)==0)	// scene background procedure
    {     
        int x,y;
        int a,b;
        static unsigned char counter;
        unsigned char *bkgvec,*disvec;  
        unsigned char send_old=send_enable;
        send_enable=0;

        switch(buffer[5])
        {
        case 'I':
            if(total_sys_number>1 && current_sys_number==1) 
            {
                flag_serial=0;

                /*! \code
                // il PCN i-esimo comunica al suo slave il comando di aggiornamento dello sfondo
                SNP_Send(slave_id,buffer,NULL,0,ttyS1);
                \endcode */

                // il PCN i-esimo comunica al suo slave il comando di aggiornamento dello sfondo
                //if I'am the master and there is a slave  
                SNP_Send(slave_id,buffer,NULL,0,ttyS1);
            }
            mainloop_enable(0);	// stopping mainloop before saving the background
            
            i2cstruct.reg_addr =  MUX_MODE;
            i2cstruct.reg_value = MUX_MODE_8_FPN_ODC_MEDIAN_DISP;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

            read(pxa_qcp,Frame,imagesize);	// the first image is dirty
            
            //memset(svec,0,sizeof(int)*NN); // eVS 20100419 meglio aumentare il valore iniziale
            for(int h=NN-1;h>=0;h--) 
                svec[h]=INITIAL_STD_BKG;
            memset(Bkgvec,0,sizeof(Bkgvec)); //NN); 20100512 eVS            
            counter = 0;
            
            if(total_sys_number>1 && current_sys_number==1)  
            {//master wait until all slaves complete the same step
                unsigned char error=0;
                
                // 20100521 eVS stop if a feedback is received or after 10 seconds
                // 20100526 eVS feedback is received by means of the command "slavereplay"
                // in serial_port.cpp
                
                //while(flag_serial==0)
                while(flag_serial==0 && error<100)
                {
                    error++;
                    usleep(100000); //wait 1/10 of sec and check another time
                    /*if(error==100) 
                    {//after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                        write_parms("sys_number",0);
                        write_parms("sys_number_index",0);
                        wide_gate_serial_parms_reset();
                        save_parms("sys_number",0);
                        save_parms("sys_number_index",0);
                        break;
                    }*/
                }
                // 20100521 eVS moved here from inside the while
                if(error==100) 
                {//after 10 sec exit, because probably there are connection problems (ex:cable not plugged)
                    write_parms("sys_number",0);
                    write_parms("sys_number_index",0);
                    wide_gate_serial_parms_reset();
                    save_parms("sys_number",0);
                    save_parms("sys_number_index",0);
                }
            } 

            break;
        case 'S':
            if(total_sys_number>1 && current_sys_number==1) //if I'm first in the chain
            {
                flag_serial=0;
                SNP_Send(slave_id,buffer,NULL,0,ttyS1); //if I'am the master and there is a slave  
            }
            a = 15;
            b = 16;
            read(pxa_qcp,Frame,imagesize); 

            get_images(Frame,0);    
            for(y=0;y<NY;y++)for(x=0;x<NX;x++)
            {
                disvec = &Frame_DSP[NX*y+x];
                bkgvec = &Bkgvec[NX*y+x];
                if(*disvec > 0)
                {
                    if(x < BORDER_X || x >= NX-BORDER_X || y < BORDER_Y || y >= NY-BORDER_Y)
                        *disvec=0;
                    *bkgvec=((a*(*bkgvec))+((b-a)*(*disvec))) >> 4;
                    svec[NX*y+x]=((int)sqrt((a*svec[NX*y+x]*svec[NX*y+x]+(b-a)*(*disvec-*bkgvec)*(*disvec-*bkgvec)) >> 4));
                    
                    //bool use_value = counter<32;
                    //if (!use_value)
                    //  use_value = (*disvec >= (int)(*bkgvec)-3*svec[NX*y+x] && *disvec <= (int)(*bkgvec)+3*svec[NX*y+x]);
                    
                    //if (use_value)
                    //{
                    //  *bkgvec=((a*(*bkgvec))+((b-a)*(*disvec))) >> 4;
                    //  svec[NX*y+x]=((int)sqrt((a*svec[NX*y+x]*svec[NX*y+x]+(b-a)*(*disvec-*bkgvec)*(*disvec-*bkgvec)) >> 4));
                    //}                        
                }
            }
            counter++;
            if(total_sys_number>1 && current_sys_number==1)  //master wait until the slave complete the same step
            {
                unsigned char error=0;
                // 20100521 eVS stop if a feedback is received or 10 seconds are passed
                //while(counter!=flag_serial)
                while(counter!=flag_serial && error < 100)
                {
                    error++;
                    usleep(100000); //wait 1/10 of sec and check another time
                    /*if(error==100) //after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                    {           
                        write_parms("sys_number",0);
                        write_parms("sys_number_index",0);
                        wide_gate_serial_parms_reset();
                        save_parms("sys_number",0);
                        save_parms("sys_number_index",0);
                        break; 
                    }*/
                }
                // 20100521 eVS moved here from inside the while loop
                if(error==100) //after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                {           
                    write_parms("sys_number",0);
                    write_parms("sys_number_index",0);
                    wide_gate_serial_parms_reset();
                    save_parms("sys_number",0);
                    save_parms("sys_number_index",0);
                }
            }  
            Send(sockfd,(char*)&counter,sizeof(counter)); 

            break;

        case 'E':
            if(total_sys_number>1 && current_sys_number==1) //If I'm a master and there is a slave
            {
                flag_serial=0;
                SNP_Send(slave_id,buffer,NULL,0,ttyS1);   
            }	  
            FILE *out = fopen(bg_filename,"wb");
            fwrite(Bkgvec,sizeof(Bkgvec),1,out);
            fwrite(svec,sizeof(svec),1,out);
            fwrite(&vm_img,sizeof(vm_img),1,out); //save the mean value of image in background file
            vm_bkg=vm_img; 
            fclose(out);    		
            memcpy(Bkgvectmp,Bkgvec,NN);

            i2cstruct.reg_addr =  MUX_MODE;
            i2cstruct.reg_value = acq_mode & 0x00FF;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
            if(total_sys_number>1 && current_sys_number==1)  //master wait until the slave complete the same step
            {
                unsigned char error=0;
                // 20100521 eVS stop if a feedback is received or 10 seconds are passed
                //while(flag_serial==0)
                while(flag_serial==0 && error<100)
                {
                    error++;
                    usleep(100000); //wait 1/10 of sec and check another time
                    /*if(error==100) //after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                    {           
                        write_parms("sys_number",0);
                        write_parms("sys_number_index",0);
                        wide_gate_serial_parms_reset();
                        save_parms("sys_number",0);
                        save_parms("sys_number_index",0);
                        break; 
                    }*/
                }
                // 20100521 moved here from inside the while loop
                if(error==100) //after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
                {           
                    write_parms("sys_number",0);
                    write_parms("sys_number_index",0);
                    wide_gate_serial_parms_reset();
                    save_parms("sys_number",0);
                    save_parms("sys_number_index",0);
                }
            }

            mainloop_enable(1);		// restarting the mainloop
            break;

        }
        send_enable=send_old;
        return 0;
    }

    /*! \code
    // Salvataggio del Fixed Pattern Noise sulla memoria flash dell'FPGA
    if(strcmp(buffer,"fpn")==0)
    \endcode */

    if(strcmp(buffer,"fpn")==0)		// save FPN correction on FPGA flash
    { 
        unsigned char wrsel;

        i2cstruct.reg_addr =  FL_WRSEL;
        ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
        wrsel=i2cstruct.reg_value;

        int i;
        fpn_counter = 0;
        mainloop_enable(0);	// stopping mainloop before saving the background

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_SX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        for(i=0;i<10;i++)  
            read(pxa_qcp,Frame,imagesize);

        get_images(Frame,0);
        int tmp=0;
        for(int ind=0;ind<NX*NY;ind++)
        {
            tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
            Frame10_SX[ind] = tmp;
        }
        //   get_10bit_image(Frame10_SX,Frame);

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_DX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        for(i=0;i<10;i++)  
            read(pxa_qcp,Frame,imagesize);


        get_images(Frame,0);
        tmp=0;
        for(int ind=0;ind<NX*NY;ind++)
        {
            tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
            Frame10_DX[ind] = tmp;
        }

        //   get_10bit_image(Frame10_DX,Frame);

        if((wrsel & 0x01) && (wrsel & 0x02))  erase_flash();  	// FPN && ODC

        background(FPN_SX,Frame10_SX);
        background(FPN_DX,Frame10_DX);

        ret = save_fpn();

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        usleep(300000);
        mainloop_enable(1);		// restarting the mainloop
        usleep(300000);

        memset(ODCbuf,0,sizeof(ODCbuf));  // resetting ODC parameters buffer
        return ret;
    }

    if(strcmp(buffer,"downfpn")==0)
    {
        char FPN[2*NN];
        int i;

        images_enabled = 0;
        mainloop_enable(0);	// stopping mainloop before saving the background

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_SX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        for(i=0;i<20;i++)  
            read(pxa_qcp,Frame,imagesize); 

        //   get_10bit_image(Frame10_SX,Frame); 
        get_images(Frame,0);
        int tmp=0;
        for(int ind=0;ind<NX*NY;ind++)
        {
            tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
            Frame10_SX[ind] = tmp;
        }

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_DX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct); 
        for(i=0;i<10;i++)  
            read(pxa_qcp,Frame,imagesize);

        get_images(Frame,0);
        tmp=0;
        for(int ind=0;ind<NX*NY;ind++)
        {
            tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
            Frame10_DX[ind] = tmp;
        }
        //    get_10bit_image(Frame10_DX,Frame);
        background(FPN_SX,Frame10_SX);
        background(FPN_DX,Frame10_DX);

        for(int i=0;i<NN;i++)
        {
            FPN[i]=FPN_SX[i];
            FPN[i+NN]=FPN_DX[i];
        }

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        Send(fd,(char *)FPN,2*NN*sizeof(char));		
        usleep(300000);
        mainloop_enable(1);		// restarting the mainloop
        usleep(300000);
        images_enabled = 1;

        return 0;
    }

    if(strcmp(buffer,"upfpn")==0)
    {
        unsigned char val=0x01;
        char FPN[2*NN];
        int ret;
        images_enabled = 0;
        mainloop_enable(0);
        i2cstruct.reg_addr =  FL_WRSEL;
        i2cstruct.reg_value = val;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        Recv(fd,(char *)FPN,sizeof(char)*NN*2);
        fpn_counter=0;
        for(int i=0;i<NN;i++)
        {
            FPN_SX[i]=FPN[i];
            FPN_DX[i]=FPN[i+NN];
        }
        ret = save_fpn();

        usleep(300000);
        mainloop_enable(1);		// restarting the mainloop
        usleep(300000);
        images_enabled = 1;
        return ret;
    }

    /*! \code
    // Settaggio della modalita' di acquisizione
    if(strcmp(buffer,"smode")==0)
    \endcode */

    if(strcmp(buffer,"smode")==0)		// sets acquisition mode
    {
        unsigned short loc_acq_mode;
        Recv(fd,(char *)&loc_acq_mode,sizeof(acq_mode)); // 20120711 eVS, moved outside lock/unlock

        pthread_mutex_lock(&acq_mode_lock); // 20100517 eVS
        acq_mode = loc_acq_mode; // 20120711 eVS, added instead of Recv which was moved before lock
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        pthread_mutex_unlock(&acq_mode_lock); // 20100517 eVS

        return 0;
    }

    /*! 
    \code
    // Lettura dei parametri da PCN: prima di acquisire le immagini da telecamere il client 
    // deve capire in che stato si trova il PCN, percio' la prima cosa da fare dopo aver stabilito la connessione 
    // e' sincronizzarsi con il server. I primi parametri di sincronizzazione sono la modalita' di acquisizione, 
    // data e ora.
    if(strcmp(buffer,"gmode")==0) // returns acquisition mode
    \endcode */

    if(strcmp(buffer,"gmode")==0)		// returns acquisition mode
    {
        Send(fd,(char *)&acq_mode,sizeof(acq_mode));
        return 0;
    }

    if(strcmp(buffer,"gparms")==0)		// returns parameters
    {
        Send(fd,(char *)parm_values,sizeof(parm_values));
        return 0;
    }
    if(strcmp(buffer,"gcalibparms")==0)		// returns parameters
    {
        Send(fd,(char *)calib_parm_values,sizeof(calib_parm_values));
        return 0;
    }  

    /*! \code
    // Chiusura della connessione client/server mediante socket
    if(strcmp(buffer,"disconnect")==0)
    \endcode */

    if(strcmp(buffer,"disconnect")==0)		// stops the socket connection
    {          
        start_stop &= ~0x0002;
        strcpy(mystr,"disconnect\0");
        Send(fd,mystr,strlen(mystr)+1);
        return 0;
    }

    // 20120119 eVS, added reboot command
    if(strcmp(buffer,"reboot")==0)		// stops the socket connection
    {          
      pthread_mutex_lock(&rdlock);
      
      // 20111012 eVS, added the following check to avoid conflicts with the log saving procedure                                    
      while (records_saving_in_progress)
      {
        pthread_mutex_unlock(&rdlock);
        usleep(100000);
        pthread_mutex_lock(&rdlock);
      }
      
      // save latest record data
      if(records_idx)
      {
          FILE *recordfd,*counterfd;

          //-------  saving the two people counters  --------//
          if((counterfd = fopen(cr_filename,"w+")))
          {
              fprintf(counterfd,"counter_in %ld\n",counter_in);
              fprintf(counterfd,"counter_out %ld\n",counter_out);
              fclose(counterfd); 
          }
          //-------------------------------------------------//
          if((recordfd = fopen(record_fn,"a+")))// saving last records to file
          {
              fseek(recordfd,0L,SEEK_END);
              for(unsigned int i=0;i<records_idx;i++)
                  fprintf(recordfd,"%s",records[i]);          
              fclose(recordfd);
              records_idx = 0; 
          }
          //print_log("%s records moved in file\n", buffer);
      }
      pthread_mutex_unlock(&rdlock);
      
      system("reboot");
      return 0;
    }
    
    /*! \code
    // restituisce la data e l'ora di sistema
    if(strncasecmp(buffer,"gdatetime",9)==0)
    \endcode */

    if(strncasecmp(buffer,"gdatetime",9)==0)	// returns system date and time
    {
        char systime[16];
        char sysdate[16];
        time_t curtime;
        struct tm *loctime;

        curtime = time (NULL);
        loctime = localtime (&curtime);
        memset(sysdate,0,sizeof(sysdate));    
        memset(systime,0,sizeof(systime)); 
        sprintf(sysdate,"%02d/%02d/%04d",loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year); 
        sprintf(systime,"%02d.%02d",loctime->tm_hour,loctime->tm_min); 
        SendString(fd,sysdate);
        SendString(fd,systime);

        return 0;
    }

    /*! \code
    // setta la data e l'ora di sistema
    if(strncasecmp(buffer,"sdatetime",9)==0)
    \endcode */

    if(strncasecmp(buffer,"sdatetime",9)==0)	// sets system date and time
    {
        char datetime[16];
        char command[32];
        RecvString(fd, datetime,sizeof(datetime));
        strcpy(command,"date ");
        strcat(command,datetime);

        unsigned char send_old=send_enable;
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&datetime,sizeof(datetime),ttyS1);
        }

        system(command);
        system("hwclock -w");
        send_enable=send_old;

        return 0;
    }

    /*! \code
    // per trasferire via RS485 l'update dell'imgserver
    if(strncasecmp(buffer,"updateI",7)==0)
    \endcode */
    //if(strncasecmp(buffer,"updateI",7)==0)	// update system software modules
    // 20100526 eVS reduce redundancy
    if(strncasecmp(buffer,"update",6)==0)	// update system software modules
    {
        int size,ret;
        unsigned char *buf;
        char command[64];
        Recv(fd,(char *)&size,sizeof(size));
        buf = new unsigned char [size];
        ret = Recv(fd,(char *)buf,size);

        //if(ret != size) // 20100526 eVS 
        if(ret != size || (buffer[6] != 'I' && buffer[6] != 'F'))
        {
            ret = -1;
            Send(fd,(char *)&ret,sizeof(ret));
        }
        else
        {
            FILE *out; //= fopen("/tmp/imgserver.new","wb"); 20100526
            if (buffer[6] == 'I') // 20100526 eVS added
                out = fopen("/tmp/imgserver.new","wb"); 
            else // 20100526 eVS added
                out = fopen("/tmp/pcn1001.bin","wb");
                
            if(out < 0) 
                ret = -1;
            else
            { 
                fwrite(buf,size,1,out);
                fclose(out);  

                if (buffer[6] == 'I') { // 20100526 eVS added
                    system("chmod 755 /tmp/imgserver.new");  
                    sprintf(command,"cp /tmp/imgserver.new %s",working_dir); 
                } else // 20100526 eVS added
                    sprintf(command,"cp /tmp/pcn1001.bin %s",working_dir); 
                    
                system(command);

                ret = 0;
            }
            Send(fd,(char *)&ret,sizeof(ret));
        }	
        delete [] buf;
        return ret; //0; bugfix eVS
    }

    
    /*! \code
    // per trasferire via RS485 l'update del bitsream
    if(strncasecmp(buffer,"updateF",7)==0)
    \endcode */
    // 20100526 eVS reduce redundancy
    /*if(strncasecmp(buffer,"updateF",7)==0)	// update the FPGA bitstream
    {
        int size,ret;
        unsigned char *buf;
        char command[64];
        Recv(fd,(char *)&size,sizeof(size));
        buf = new unsigned char [size];
        ret = Recv(fd,(char *)buf,size);

        if(ret != size)
        {
            ret = -1;
            Send(fd,(char *)&ret,sizeof(ret));
        }
        else
        {
            FILE *out = fopen("/tmp/pcn1001.bin","wb");
            if(out < 0) 
                ret = -1;
            else
            {
                fwrite(buf,size,1,out);
                fclose(out);
                
                ///////////////////////
                // 20091207 eVS
                
                sprintf(command,"cp /tmp/pcn1001.bin %s",working_dir); 
                system(command);
                //mainloop_enable(0);
                //sprintf(command,"flash_eraseall /dev/mtd6");
                //system(command);
                //sprintf(command,"cat pcn1001.bin > /dev/mtdblock6");
                //system(command);
                //sprintf(command,"rm pcn1001.bin");
                //system(command);
                //mainloop_enable(1);

                // 20091207 eVS
                ///////////////////////
                
                ret = 0;
            }
            Send(fd,(char *)&ret,sizeof(ret));
        }
        delete [] buf;
        return ret; //0; bugfix eVS
    }*/

    /*!
    \code
    // Scaricamento dei contatori: ogni 60 secondi il server salva la data, l'ora e la direzione 
    // di ogni passeggero sulla memoria non volatile. 
    // L'archivio puo' essere scaricato via ftp oppure utilizzando un apposito comando: rdsave.
    // Invece il comando rddelete permette all'utente di cancellare il file dal file system del PCN.
    if(strcmp(buffer,"rdsave")==0) 
    {
        // downloads records.txt
        //...
    \endcode
    */
    if(strcmp(buffer,"rdsave")==0)		// downloads records.txt
    {
        int fsize,total_size;
        unsigned int i;	
        unsigned char *buf,*ptr;
        unsigned long start_id;
        char filename[256];
        FILE *recordfd,*counterfd;
        
        pthread_mutex_lock(&rdlock);
        
        // 20111012 eVS, added the following check to avoid conflicts with the log saving procedure
        while (records_saving_in_progress)
        {
          pthread_mutex_unlock(&rdlock);
          usleep(50000);
          pthread_mutex_lock(&rdlock);
        }

        if(records_idx)
        {
            //-------  saving the two people counters  --------//
            if((counterfd = fopen(cr_filename,"w+")))
            {
                fprintf(counterfd,"counter_in %ld\n",counter_in);
                fprintf(counterfd,"counter_out %ld\n",counter_out);
                fclose(counterfd); 
            }
            //-------------------------------------------------//
            if((recordfd = fopen(record_fn,"a+")))// saving last records to file
            {
                fseek(recordfd,0L,SEEK_END);
                for(i=0;i<records_idx;i++)
                    fprintf(recordfd,"%s",records[i]);          
                fclose(recordfd);
                records_idx = 0; 
            }
        }
        total_size = fsize = 0;
        start_id = (record_id+1) > MAX_REC_FILES ? (record_id+1)-MAX_REC_FILES : 0; //first file index
        for(i=start_id;i<(record_id+1);i++)	// computing the total size
        {
            sprintf(filename,"%s%d.txt",rd_filename,i);
            if((recordfd = fopen(filename,"a+")))
            { 
                fseek(recordfd,0L,SEEK_END);
                total_size += ftell(recordfd);
                fclose(recordfd);
            }
        }
        Send(fd,(char *)&total_size,sizeof(total_size));  //sending the total size

        buf = new unsigned char [total_size];
        ptr = buf;
        for(i=start_id;i<(record_id+1);i++)		// copying data to a buffer
        {
            sprintf(filename,"%s%d.txt",rd_filename,i);
            if((recordfd = fopen(filename,"a+")))
            { 
                fseek(recordfd,0L,SEEK_END);
                fsize = ftell(recordfd);
                fseek(recordfd,0L,SEEK_SET);
                if((ptr+fsize) > (buf + total_size))
                {
                    fclose(recordfd);
                    break;
                }
                fread(ptr,1,fsize,recordfd);
                ptr += fsize;
                fclose(recordfd);
            }
        }
        Send(fd,(char *)buf,total_size);			// sending the buffer
        delete [] buf;
        pthread_mutex_unlock(&rdlock);

        return 0;
    }

    if(strcmp(buffer,"rddelete")==0)		// deletes records.txt from filesystem
    {
        unsigned int i;	
        unsigned long start_id;
        char command[256];

        if(total_sys_number>1 && current_sys_number==1) //If I'm a master and there is a slave
            SNP_Send(slave_id,buffer,NULL,0,ttyS1);

        pthread_mutex_lock(&rdlock);
        start_id = (record_id+1) > MAX_REC_FILES ? (record_id+1)-MAX_REC_FILES : 0;//first file index
        for(i=start_id;i<(record_id+1);i++)
        {
            sprintf(command,"rm -rf %s%d.txt",rd_filename,i);
            system(command);
        }
        record_id = 0;
        sprintf(record_fn,"%s%ld.txt",rd_filename,record_id);

        pthread_mutex_unlock(&rdlock);
        return 0;
    }

    /*! \code
    // Regolazione dell'intensita' degli illuminatori all'infrarosso (vicino infrarosso)

    // codice lato client
    unsigned char val = 127; // 0..255
    SendString(sock, "sled");
    Send(sock, &val, sizeof(val));

    // codice lato server
    if(strcmp(buffer,"sled")==0)
    \endcode */

    if(strcmp(buffer,"sled")==0)		// sets light intensity
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        if (value >= LED_MAX_VALUE)  // 20101025 eVS added check
            value = LED_MAX_VALUE;
            
        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;
        if(total_sys_number>1 && current_sys_number==1)
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

        send_enable=send_old; 
        if(write_parms(buffer,(unsigned short)value) < 0) 
        {
            //send_enable=send_old; 
            return -1;  
        }
        ret=save_parms(buffer,(unsigned short)value);
        //send_enable=send_old;
        return ret;
    }

    /*! \code
    // Attivazione modalita' automatica di regolazione degli illuminatori all'infrarosso

    // codice lato client
    unsigned char val = 1;
    SendString(sock, "autoled");
    Send(sock, &val, sizeof(val));

    // codice lato server
    if(strcmp(buffer,"autoled")==0)
    \endcode */

    if(strcmp(buffer,"autoled")==0)
    {
        unsigned char val;
        unsigned char send_old=send_enable;
        int ret;
        Recv(fd,&val,sizeof(val));
        send_enable=0;

        if(total_sys_number>1 && current_sys_number==1) //If I'm a master and there is a slave
            SNP_Send(slave_id,buffer,(char *)&val,sizeof(val),ttyS1);

        ret=write_parms(buffer,(unsigned short)val);  
        if(ret==0) ret=save_parms(buffer,(unsigned short)val);
        if(ret==0) ret=write_parms("sled",0); //initial condition for feedback control
        if(ret==0) ret=save_parms("sled",0);
        send_enable=send_old;

        return ret;
    }
    if(strcmp(buffer,"auto_gain")==0)
    {
        unsigned char val;
        unsigned char send_old=send_enable;
        int ret;
        Recv(fd,&val,sizeof(val));
        send_enable=0;
        
        if(total_sys_number>1 && current_sys_number==1) //If I'm a master and there is a slave
            SNP_Send(slave_id,buffer,(char *)&val,sizeof(val),ttyS1);

        
        //auto_gain = (val) ? 1 : 0;
        printf(" Auto_gain set to = %d \n",auto_gain);
        ret=write_parms(buffer,(unsigned short)val);  
        if(ret==0) ret=save_parms(buffer,(unsigned short)val);
        
        send_enable=send_old;

        return ret;
    }
    if(strcmp(buffer,"dir")==0)		// sets the incoming/outgoing people direction
    {
        unsigned char value;    	
        unsigned char send_old=send_enable;
        int ret = 0; // 20100425 eVS 
        Recv(fd,(char *)&value,sizeof(value)); 	
	
        // 20100425 eVS, check if the sent direction is different from the current one
        unsigned char current_dir = get_parms("dir");
        if (current_dir != value)
        {
            send_enable=0;
            if(total_sys_number>1)
            {
                if(current_sys_number!=1) 
                  ret = -1; //in wg this command is reserved by master
                else //If I'm a master and there is a slave
                {
                    //SNP_Send(slave_id,"dir",(char *)&value,sizeof(value),ttyS1);
                    SNP_Send(slave_id,"reset",(char *)&value,sizeof(value),ttyS1);  
                } 
            }
            
            if(ret==0)  ret=write_parms(buffer,(unsigned short)value); 
            if(ret==0)  ret=save_parms(buffer,(unsigned short)value);
            if(total_sys_number<2) // ???
                reset_counters(0);
            send_enable = send_old;     
        }
        return ret; 
    }

    // 20120117 eVS, added command for the new socket_gui application
    // enables/disables person-counting    
    if(strcmp(buffer,"enable_pc")==0)		// enables/disables person-counting
    {
        unsigned char value;
        
        Recv(fd,(char *)&value,sizeof(value)); 	
        
        send_enable=0;
        if(total_sys_number>1)
        {
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
        }

        enable_counting(value); //bisogna lasciare questa per gestire logica porta
        
        if (value)
          SendString(fd, "counting started");
        else
          SendString(fd, "counting stopped");

        return 0;
    }
    
    // 20120119 eVS, added command for the new socket_gui application
    if(strcmp(buffer,"gdoorstatus")==0)		// get door status
    {
        pthread_mutex_lock(&mainlock);  // 20111011 eVS, added 
        
        unsigned char value;
        if(!mem_door) value=0;
        else value=1;
        
        pthread_mutex_unlock(&mainlock); // 20111011 eVS, added
        
        Send(fd,(char *)&value,sizeof(value));
        return 0;
    }
    
    //********************************************************************
    /*! \code
    // used to set the PCN ip address
    if(strcmp(buffer,"address")==0)
        ...
    \endcode */
    if(strcmp(buffer,"address")==0)  	// sets the system ip address
    { 
        char args[16];
        int args_size = sizeof(args)-1;

        RecvString(fd, args, args_size);        
        args[15] = '\0';
        
        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)args,args_size,ttyS1);

            send_enable=send_old;
        }

        char str[255];
        sprintf(str, "address %s will be used after reboot ", args);
        SendString(fd, str);
        
        char command[32];
        strcpy(command,"/sbin/netconfig address ");
        strcat(command,args);

        {
          char filename[255];
          sprintf(filename, "%sip.txt", working_dir);
          FILE *ip = fopen(filename,"w");
          fprintf(ip, command);
          fclose(ip);  
        }

        return 0;
    }

    /*! \code
    // setta l'ID per la comunicazione seriale
    if(strcmp(buffer,"serial_id")==0)
    \endcode */

    if(strcmp(buffer,"serial_id")==0)		// sets ID for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 
        if(total_sys_number>1 && current_sys_number!=1) return -1; //in wg this command is reserved to master
        return save_parms(buffer,value);
    }

    if(strcmp(buffer,"serial_br")==0)		// sets baud rate for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 
        if(total_sys_number>1 && current_sys_number!=1) return -1; //in wg this command is reserved to master
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }

    if(strcmp(buffer,"serial_db")==0)		// sets data bits for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1 && current_sys_number!=1) return -1; //in wg this command is reserved to master
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }

    if(strcmp(buffer,"serial_pr")==0)		// sets parity for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 
        if(total_sys_number>1 && current_sys_number!=1) return -1; //in wg this command is reserved to master
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_sb")==0)		// sets stop bits for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1 && current_sys_number!=1) return -1; //in wg this command is reserved to master
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_sid")==0)		// sets ttyS1 ID 
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1) return -1; //in wg this command is not permitted
        return save_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_sbr")==0)		// sets baud rate for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1) return -1; // in wg this command is not permitted
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_sdb")==0)		// sets data bits for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 
        if(total_sys_number>1) return -1; //in wg this command is not permitted
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_spr")==0)		// sets parity for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1) return -1; //in wg this command is not permitted
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strcmp(buffer,"serial_ssb")==0)		// sets stop bits for serial communication
    {
        unsigned short value;
        Recv(fd,(char *)&value,sizeof(value)); 

        if(total_sys_number>1) return -1; //in wg this command is not permitted
        save_parms(buffer,value);
        return write_parms(buffer,value);
    }
    if(strncasecmp(buffer,"dac",3)==0)		// sets sensors dac values
    {
        unsigned short value;
        int ret=0;
        Recv(fd,&value,sizeof(value));
        calib_write_parms(buffer,value);
        ret=calib_save_parms(buffer,value);
        return ret;
    }
    if(strncasecmp(buffer,"map",3)==0)		// sets disparity map parameters	
    {
        unsigned char value;

        Recv(fd,&value,sizeof(value));
        if(calib_write_parms(buffer,(unsigned short)value) < 0) return -1;  
        return calib_save_parms(buffer,(unsigned short)value);
    }
    if(strncasecmp(buffer,"input",5)==0)	// sets optocoupled input function
    {    
        unsigned short value;      
        unsigned char send_old=send_enable;

        Recv(fd,(char *) &value,sizeof(value));
        send_enable=0;     
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number) //if I'm a master
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
                if(buffer[5]=='0')
                {
                    if(value==0 || value==2)
                    {
                        write_parms(buffer,value);
                        send_enable=send_old;
                        return save_parms(buffer,value); 
                    }
                    else
                    {
                        send_enable=send_old; 
                        return -1;  
                    }
                }
                else if(buffer[5]=='1') 
                    SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
                send_enable=send_old;
                return 0;
            }
            else 
            {
                if(buffer[5]=='0') return -1; //in wg input0 is reserved to reset counters
                if(value==2) return -1; //slave cannot have the door signal;
                if(write_parms(buffer,value) < 0) return -1;
                return save_parms(buffer,value);
            }
        }
        else //if daisy chain disable
        {
            send_enable=send_old;
            if(write_parms(buffer,value) < 0) return -1;  

            return save_parms(buffer,value); 
        }
        send_enable=send_old;
        return -1;
    }
    if(strncasecmp(buffer,"outtime",7)==0)  	//optocoupled output test
    {
        unsigned short value;
        int ret;
        unsigned char send_old=send_enable;

        Recv(fd,(char *) &value,sizeof(value));
        if(total_sys_number<2)
        {
            if(write_parms(buffer,value) < 0) return -1;
            return save_parms(buffer,value);       
        }
        send_enable=0;
        if(total_sys_number>1 && total_sys_number!=current_sys_number && buffer[7]=='1') 
        {
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            send_enable=send_old;
            return 0;
        }               
        value = (value >> 2) << 2;	// outtime has to be a multiple of 4

        if((current_sys_number==1)&&(buffer[7]=='0'))
        {
            ret=write_parms(buffer,value);
            if(ret==0) ret=save_parms(buffer,value); 
            send_enable=send_old;
            return ret;
        } 
        send_enable=send_old;
        return -1;           
    }
    if(strncasecmp(buffer,"testin",6)==0)  	//optocoupled input test
    {
        int ret = 0;
        //char value;
        unsigned char *input_test = NULL;
        pthread_mutex_lock(&mainlock); // 20100424 eVS, removed comment
        // 20120117 eVS, restored old method after modification in input_loop0 e input_loop1
        if(strlen(buffer) == 7) 
        {
            input_test = (atoi(&buffer[6]) == 0) ? &input_test0 : &input_test1;
        }
        else ret = -1;
        /*if (strlen(buffer) == 7 && buffer[6]=='0')
            value = get_gpio("GPLR3_096");
        else if (strlen(buffer) == 7 && buffer[6]=='1')
            value = get_gpio("GPLR2_095");
        else
            ret = -1;*/
        
        if (!ret)
            Send(fd,input_test,sizeof(char));
            //Send(fd,&value,sizeof(char));
        pthread_mutex_unlock(&mainlock); // 20100424 eVS, removed comment
        return ret;
    }

    /*!
    \code
    // Settaggio soglia porta mediante il comando threshold
    // durante il processo di tracking, i contatori di input/output sono incrementati
    // solo se una persona supera la linea di separazione della zona alta/bassa 
    // e poi esce dall'area monitorata
    // Per default la soglia porta viene settata a 60 (su 120 righe dell'immagine sottocampionata).
    // La posizione puo' essere variata all'interno di un range tra 30 e 89.
    if(strcmp(buffer,"threshold")==0)
    {
        //...
    \endcode
    */
    if(strcmp(buffer,"threshold")==0)
    {
        unsigned char value;
        unsigned char send_old=send_enable;
        int ret;
        Recv(fd,(char *)&value,sizeof(value));

        value = value < MIN_THRESHOLD ? MIN_THRESHOLD : value;
        value = value > MAX_THRESHOLD ? MAX_THRESHOLD : value;

        send_enable=0;
        if(total_sys_number>1)
        { 
            if(current_sys_number==1)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            else 
            {
                send_enable=send_old; 
                return -1;  
            }
        }
       
        if(write_parms(buffer,(unsigned short)value) < 0) return -1;
        ret=save_parms(buffer,(unsigned short)value);
        send_enable=send_old;
        
        ////////////////////
        // 20091120 eVS
        // - Changing the door threshold has to affect the move detection zone
        //   more precisely the starting and ending rows
        //return ret;
        return check_mov_det_parms();
        // 20091120 eVS
        ////////////////////
    }
    if(strcmp(buffer,"calibimg")==0)
    {
        mainloop_enable(0);	// stopping mainloop before saving the background
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_8_FPN;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        read(pxa_qcp,Frame,imagesize);	// the first image is dirty
        read(pxa_qcp,Frame,imagesize);
        get_images(Frame,0);
        Send(fd,(char *)Frame_DX,NN*sizeof(char));
        Send(fd,(char *)Frame_SX,NN*sizeof(char));
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        mainloop_enable(1);		// restarting the mainloop
        return 0;
    }

    /*!
    \code
    // Configurazione della distanza: la distanza tra il sensore e il bordo superiore
    // della regione di monitoraggio puo' essere compresa tra 25 e 40 centimetri.
    
    // codice lato client per fare la richiesta all'imgserver
    SendString(sock, "detect_area");
    Send(sock, &val, sizeof(val));
    // val = 0 for a distance between 25 and 30 cm.
    // val = 1 for a distance between 31 and 40 cm.

    // codice lato server per eseguire la richiesta del client
    if(strcmp(buffer,"detect_area")==0)
    {
        //...
    \endcode
    */
    if(strcmp(buffer,"detect_area")==0)
    {
        char passo;
        char stepstring[10];
        unsigned short value;
        short ret = 0;
        unsigned short old_height;
        Recv(fd,(char *)&value,sizeof(value));
        if(value != 0 && value != 1)
        {
            ret = -1;
            Send(fd,(char *)&ret, sizeof(ret));
            return ret;
        }
        old_height=get_parms("detect_area");
        if(old_height != value)
        {//write the fpga steps only if it is necessary

            unsigned char send_old=send_enable;
            int ret;
            send_enable=0;
            if(total_sys_number>1)
            { 
                if(current_sys_number==1)
                    SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
                else 
                {
                    send_enable=send_old;
                    return -1;
                }
            }

            if(value == 0)
                sprintf(stepstring,"step225_");
            else if(value == 1) 
                sprintf(stepstring,"step240_");
            else 
            {
                send_enable=send_old;
                return -1;
            }
            for(int i=0;i<NUM_STEP;i++)
            {
                if(i<=9) passo=0x30+i; // ex: i=10 => passo='A' => stepA
                else passo=0x37+i;
                stepstring[8]=passo; stepstring[9]='\0';

                unsigned char valore_passo=calib_get_parms(stepstring);
                if(calib_write_parms(stepstring,valore_passo) < 0)
                {
                    ret = -1;
                    Send(fd,(char *)&ret, sizeof(ret)); 
                    send_enable=send_old;
                    return ret; 
                }	
            }
            if(save_parms("detect_area",value) < 0) 
            {
                ret = -1;
                Send(fd,(char *)&ret, sizeof(ret)); 
                send_enable=send_old;
                return ret; 
            }	   
            write_parms("detect_area",value);
            send_enable=send_old;
        } 
        ret = 0;
        Send(fd,(char *)&ret, sizeof(ret)); 
        return 0;
    }
    if(strcmp(buffer,"threshBkg")==0)//winsize
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value)); 
        soglia_bkg=value;
        if(calib_write_parms(buffer,(unsigned short)value) < 0) return -1;  
        return calib_save_parms(buffer,(unsigned short)value);
    }
    if(strcmp(buffer,"raddr")==0)
    {
        unsigned char addr,val;
        Recv(fd,(char *)&addr,sizeof(addr)); 
        i2cstruct.reg_addr =  addr;
        if(ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct)) val=0xFE; 
        else val=i2cstruct.reg_value;
        Send(sockfd,&val,sizeof(val));
    }
    if(strcmp(buffer,"waddr")==0)
    {
        unsigned char addr,val,val2;
        Recv(fd,(char *)&addr,sizeof(addr)); 
        Recv(fd,(char *)&val,sizeof(val)); 
        i2cstruct.reg_addr =  addr;
        i2cstruct.reg_value = (unsigned char)val;

        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct))  val2=144;
        else val2=157;
        Send(sockfd,&val2,sizeof(val2));
        return 0;
    }    
    if(strcmp(buffer,"wr_fpga")==0)
    {
        unsigned char addr,val;
        Recv(fd,(char *)&addr,sizeof(addr));
        Recv(fd,(char *)&val,sizeof(val));

        i2cstruct.reg_addr =  addr;
        i2cstruct.reg_value = val;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);     
        return 0;
    }
    if(strcmp(buffer,"rd_fpga")==0)
    {
        unsigned char addr,val;
        Recv(fd,(char *)&addr,sizeof(addr));

        i2cstruct.reg_addr =  addr;
        ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
        val=i2cstruct.reg_value;     

        Send(fd,(unsigned short*)&val, sizeof(val));
        return 0;
    }
    if(strcmp(buffer,"waddrsign")==0)
    {
        unsigned char addr,val2;
        char val;
        Recv(fd,(char *)&addr,sizeof(addr)); 
        Recv(fd,(char *)&val,sizeof(val)); 
        i2cstruct.reg_addr =  addr;
        i2cstruct.reg_value = (unsigned char)val;

        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) val2=0;
        else val2=255;

        Send(sockfd,&val2,sizeof(val2));
        return 0;
    }    
    if(strcmp(buffer,"meanv")==0)
    {
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_SX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

        for(int i=0; i<10;i++)
            read(pxa_qcp,Frame,imagesize);

        get_10bit_image(Frame10_SX,Frame); 

        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = MUX_MODE_10_NOFPN_DX;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct); 

        for(int i=0; i<10;i++)
            read(pxa_qcp,Frame,imagesize);

        get_10bit_image(Frame10_DX,Frame);

        unsigned long averageSX, averageDX;
        averageSX=0;
        averageDX=0;
        unsigned short average;

        for (int i=NN-1;i>=0;i--)
        {
            averageDX+=Frame10_DX[i] & 0x3FF;
            averageSX+=Frame10_SX[i] & 0x3FF;
        }
        averageDX/=NN;
        averageSX/=NN;
        average=(averageDX+averageSX)/2;
        Send(fd,(unsigned short*)&average, sizeof(average));      
        return 0;
    }
    if(strcmp(buffer,"setwrsel")==0)
    {
        unsigned char val;
        Recv(fd,&val,sizeof(val));
        i2cstruct.reg_addr =  FL_WRSEL;
        i2cstruct.reg_value = val;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(buffer,"steps225")==0)
    {
        unsigned char value[NUM_STEP]; 
        char string[10]="step225_";
        Recv(fd,(char *)&value,sizeof(value));
        char passo;
        int valore;
        for(int i=0;i<NUM_STEP;i++)
        {
            if(i<=9) passo=0x30+i; //make the char correspond to Step Number ex: i=10 => passo='A' => stepA
            else passo=0x37+i;
            string[8]=passo; string[9]='\0';
            if(calib_write_parms(string,(unsigned short)value[i]) < 0)  return -1;
            valore=int(value[i]);
            if(calib_save_parms(string,(unsigned short)value[i])<0)    return -1;
        }

        valore=0;
        Send(fd,(unsigned short*)&valore, sizeof(valore));
        return 0;
    }
    if(strcmp(buffer,"steps240")==0)
    {
        unsigned char value[NUM_STEP]; 
        char string[10]="step240_";

        Recv(fd,(char *)&value,sizeof(value)); 
        char passo;
        int valore;
        for(int i=0;i<NUM_STEP;i++)
        {
            if(i<=9) passo=0x30+i; //make the character correspond to Step Number ex: i=10 => passo='A' => stepA
            else passo=0x37+i;
            string[8]=passo; string[9]='\0';
            if(calib_write_parms(string,(unsigned short)value[i]) < 0)   return -1;
            valore=int(value[i]);
            if(calib_save_parms(string,(unsigned short)value[i])<0)      return -1;
        }

        valore=0;
        Send(fd,(unsigned short*)&valore, sizeof(valore));
        return 0;    
    }
    if(strcmp(buffer,"wsz")==0)//winsize
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value)); 
        if(calib_write_parms("winsz",(unsigned short)value) < 0) return -1;  
        return calib_save_parms("winsz",(unsigned short)value);
    }

    if(strcmp(buffer,"wg_check")==0)		// enable/disable the check of slave presence
    {  
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value)); 
        save_parms("wg_check",value);
        write_parms("wg_check",value);
        return 0;
    }
    if(strcmp(buffer,"inst_height")==0)		// sets installation height
    {
        int value;

        Recv(fd,(char *)&value,sizeof(value));

        if(value<100 || value>350) return -1;
        
        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number==1) //If I'm a master and there is a slave
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            else 
            {
                send_enable=send_old; 
                return -1;  //in wg mode this command is permitted only from the Master
            }
        }

        ret=write_parms("inst_height",(unsigned short)value); 
        if(ret==0) save_parms("inst_height",(unsigned short)value);
        send_enable=send_old;
        return ret;
    }
    if(strcmp(buffer,"inst_dist")==0)		// sets installation distance
    {
        int value;

        Recv(fd,(char *)&value,sizeof(value));

        if(value<60 || value>100) return -1;
        unsigned char send_old=send_enable;
        
        int ret;
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number==1) //If I'm a master and there is a slave
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            else
            {
             send_enable=send_old;
             ret = -1; //in wg mode this command is permitted only from the Master
            }
        }
        ret=write_parms(buffer,(unsigned short)value); 
        if(ret==0) save_parms(buffer,(unsigned short)value);
        send_enable=send_old;
        return ret;
    }

    if(strcmp(buffer,"saveimg")==0)
    {
        pthread_mutex_lock(&rdlock);
        read(pxa_qcp,Frame,imagesize);
        get_images(Frame,0); 
        int door_th=get_parms("threshold");
        for(int i=0;i<NX;i++) Frame_SX[NX*door_th+i]=255;
        Send(fd,(char *)Frame_DX,NN*sizeof(char));
        Send(fd,(char *)Frame_SX,NN*sizeof(char));
        Send(fd,(char *)Frame_DSP,NN*sizeof(char));
        Send(fd,(char *)Bkgvec,NN*sizeof(char));

        pthread_mutex_unlock(&rdlock);
        return 0; 
    }
    if(strcmp(buffer,"blockmain")==0)
    {
        mainloop_enable(0);	
        return 0;
    }

    if(strcmp(buffer,"enablemain")==0)
    {
        mainloop_enable(1);	
        return 0;
    }

    /*!
    \code
    // Used by the win_client to ask for the mean gray level value one of the 
    // two images. This value is referred to the left or the right image on the
    // basis of a partcular register in the FPGA (see FPGA documentation).
    if(strcmp(buffer,"get_vm")==0)
    {
        //...
    \endcode
    */
    if(strcmp(buffer,"get_vm")==0)
    {
        Send(fd,(unsigned char*)&vm_img, sizeof(vm_img));
        return 0;
    }

    if(strcmp(buffer,"wideconfiguration")==0)		// sets the systems number in wide-gate configuration
    {
        // 20100521 eVS ***THIS COMMAND HAS TO BE RECEIVED BY THE MASTER ONLY***
        // and olny in case of wideconfiguration
        
        // 20100520 eVS if the widegate configuration is activated then (a) the
        // no traking zone has to be reset, (b) the doff_cond_1p has to be 
        // disabled, and (c) also motion detection has to be disabled
                
        mainloop_enable(0);	// stopping mainloop before saving the background
	
        // 20100521 eVS created a common function to avoid redundancy
        prepare_for_wideconfiguration();
        
        /*write_parms("sxlimit",SXLIMIT);
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

        ////////////////////
        // 20091120 eVS
        // 20100520 eVS since no tracking zone has been changed, also the
        // motion detection area should be checked
        check_mov_det_parms();
        // 20091120 eVS
        ////////////////////
        */
        
        unsigned char next_sys_number;
        unsigned char param[2];
        
        current_sys_number_tmp=1; // this has to be the master of the widegate so the number 1
        next_sys_number=2; // and the next sensor (its slave) has to be the number 2        
        count_sincro=0;  // initialize the clock-syncro
        // wide_status=0;
        
        // 20100520 eVS the win_client send to the master the total number
        // of systems connected in the chain, so this value has to be received
        Recv(fd,(char *)&total_sys_number_tmp,sizeof(total_sys_number_tmp));
        
        // 20100520 eVS a check follows: if the received value in "total_sys_number_tmp"
        // is 0 means that the wideconfiguration has been disabled by the win_client
        // so the value is adjusted to be 1. So, now the value should be 1 for a single
        // sensor or greater than 1 for a wideconfiguration
        if(total_sys_number_tmp==0) total_sys_number_tmp=1;        
        
        // 20100520 eVS, if both total_sys_number and total_sys_number_tmp 
        // are both greater than 1, means that win_client asked this system
        // to enable wideconfiguration (total_sys_number_tmp!=1) but this 
        // system is already in wideconfiguration (total_sys_number>1) and
        // so this situation is unexpected -> send win_client an error 
        // and return -1
        if(total_sys_number>1 && total_sys_number_tmp!=1)
        {
            flag_serial=2;
            Send(fd,(char *)&flag_serial, sizeof(flag_serial));
            return -1;        
        }
        
        // 20100520 eVS, if both total_sys_number and total_sys_number_tmp 
        // are equal to 1, means that win_client asked this system to disable
        // wideconfiguration but it was already disabled -> error -> return
        if(total_sys_number==total_sys_number_tmp)
        {
            // entra qui solo se total_sys_number e total_sys_number_tmp sono = 1 ???
            flag_serial=1;
            Send(fd,(char *)&flag_serial, sizeof(flag_serial));
            return 0;
        }
        
        // 20100521 eVS, arrived here means that the win_client request
        // is resonable and we have to try to configure (enable/disable
        // wideconfiguration) the sensors chain (all the slaves)
        // ...
        
        // 20100521 eVS, before start the slaves configuration we have to 
        // guarantee that the main_loop() doen not perform wideconfiguration
        // stuff (why no mutex is used here???)
        send_enable=0;

        if(total_sys_number_tmp<2)  //spengo il widegate
        {
            // 20100520 eVS, if total_sys_number_tmp < 2 wideconfiguration 
            // has to be disabled, in order to do that for all the slaves
            // I have to send them the values [0, 0] in param[2]
            total_sys_number_tmp=0;
            current_sys_number_tmp=0;
            next_sys_number=0;
        }
        
        // 20100521 eVS, send configuration to the slave
        param[0]= total_sys_number_tmp;
        param[1]= next_sys_number;
        SNP_Send(slave_id,"wideconfiguration",(char *)param,sizeof(param),ttyS1);

        // 20100521 eVS, ***THE READER SHOULD NOW CHECK*** the 
        // "wideconfiguration" command in serial_port.cpp
        // to better understand what happens in the slaves
        // ...
        
        // 20100521 eVS
        // now the configuration packets are "floating" through
        // the sensors chain and this sensor has to wait the answer 
        // of its slave, i.e., the "end_chain" commands has to be received
        // and the flag #flag_serial set to 1...
        
        // 20100524 eVS, as just said, wait a feedback from the slaves: if 
        // a feedback arrives ("end_chain" is received), send a confirmation 
        // to the win_client otherwise set to 0 the wide gate configuration 
        // params and send a failure to the win_client (the "flag_serial" sent
        // to the win_client will be 0 in case of failure or 1 in case of 
        // confirmation)
        unsigned char error=0;
        flag_serial=0;
        // 20100521 eVS stops if a feedback is received or 10 seconds are passed
        //while(flag_serial==0)
        while(flag_serial==0 && error<100) 
        {
            error++;
            usleep(100000); //wait 1/10 of sec and check another time
            /*if(error==100) //after 10 sec exit, because probabily there are a connections problem (ex:cable not plugged)
            {
                write_parms("sys_number",0);
                write_parms("sys_number_index",0);
                wide_gate_serial_parms_reset();
                save_parms("sys_number",0);
                save_parms("sys_number_index",0);

                break;
            }*/
        }
        
        // 20100521 eVS moved here and added condition in the while guard
        if(flag_serial==0) //after 10 sec exit, because probably there is a connections problem (ex: cable not plugged)
        {
            write_parms("sys_number",0);
            write_parms("sys_number_index",0);
            wide_gate_serial_parms_reset();
            save_parms("sys_number",0);
            save_parms("sys_number_index",0);
        } else {
            // 20100525 eVS added "else" in order to save parameters of the wideconfiguration
            if(total_sys_number_tmp>1) //activating
            {
                if(write_parms("sys_number",(unsigned char)total_sys_number_tmp) < 0)
                    return -1;
                if(write_parms("sys_number_index",(unsigned char)current_sys_number_tmp) < 0)
                    return -1;
                wide_gate_serial_parms_set(); 
                if(save_parms("sys_number",(unsigned char)total_sys_number_tmp) < 0)
                    return -1;
                if(save_parms("sys_number_index",(unsigned char)current_sys_number_tmp) < 0)
                    return -1;
            }
            else //disactivating
            {
                wide_gate_serial_parms_reset();
                if(write_parms("sys_number",(unsigned char)total_sys_number_tmp) < 0)
                    return -1;
                if(write_parms("sys_number_index",(unsigned char)current_sys_number_tmp) < 0)
                    return -1;
                if(save_parms("sys_number",(unsigned char)total_sys_number_tmp) < 0)
                    return -1;
                if(save_parms("sys_number_index",(unsigned char)current_sys_number_tmp) < 0)
                    return -1;
            }
        }
        Send(fd,(char *)&flag_serial, sizeof(flag_serial));
        return 0;
    }

    if(strcmp(buffer,"set_opto_full_control")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        if(value==0)
        {
            write_parms("outtime0",200);
            save_parms("outtime0",200);
            write_parms("outtime1",200);
            save_parms("outtime1",200);
        }
        else if(value==1)
        {
            write_parms("outtime0",4); //disable FPGA in optoO
            save_parms("outtime0",4);
            write_parms("outtime1",4); //disable FPGA in opto1
            save_parms("outtime1",4);
        }
        return 0;
    }

    if(strcmp(buffer,"set_opto")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        if(value==0)
        {
            set_gpio("GPSR3_091",1);
            set_gpio("GPSR3_090",1);
        }
        else if(value==1)
        {
            set_gpio("GPCR3_091",1);
            set_gpio("GPCR3_090",1);
        }
        return 0;
    }

    if(strcmp(buffer,"test_serial_port")==0)
    {
        unsigned char error=0;
        test_serial=0;
        SNP_Send(2,"test_serial_port",NULL,0,ttyS1);
        // 20100521 eVS wait a feedback or stop after 2 secods
        //while(test_serial==0)
        while(test_serial==0 && error<20)
        {
            error++;
            usleep(100000); //wait 1/10 of sec and check another time
            /*if(error==20) //after 2 sec exit, because probabily there are a connections problem (ex:cable not plugged)
            {
                break;
            }*/
        }
        Send(fd,(char *)&test_serial, sizeof(test_serial));
        return 0;
    }
   
    /*!
    \code
    // Needed by the special version of win_client to be used for 
    // recording video sequences. This command was introduced with the 2.3.11 version
    // and change the way how the recording is managed allowing a full-framerate recording.
    // See the record_utils.* files for more details.
    // The client has to "start_rec", then use a thread loop sending "recimgdsp" 
    // requests, and then use "stop_rec" to terminate the recording
    if(strcmp(buffer,"start_rec")==0)
    {
        //...
    \endcode
    */
    /*!
    \code
    // See "start_rec" for more details.
    if(strcmp(buffer,"stop_rec")==0)
    {
        //...
    \endcode
    */
    if(strcmp(buffer,"start_rec")==0 || strcmp(buffer,"start_rec_dsp")==0)
    {
      mainloop_enable(0);
      
      // be sure that FPGA inserts the disparity map inside the quick capture interface buffer
      i2cstruct.reg_addr =  MUX_MODE;
      i2cstruct.reg_value = MUX_MODE_8_FPN_ODC_MEDIAN_DISP;
      ioctl(pxa_qcp, VIDIOCSI2C, &i2cstruct);
      for (int i=0; i<5; ++i)
        read(pxa_qcp, Frame, imagesize);	// the first image is dirty (just read more than once to be sure)
      
      int num_grab_per_packet = ru_start_record(pxa_qcp, fd, strcmp(buffer,"start_rec_dsp")==0);
      
      // here a Send can be done without problems because the win_client has not yet spawn the acquisition thread      
      Send(fd,(char *)&num_grab_per_packet, sizeof(num_grab_per_packet));
      
      return 0;
    }
    
    
    if(strcmp(buffer,"stop_rec")==0)
    {
      ru_stop_record();      
      // do not send anything otherwise this send can be confused (by the win_client) with the last pending send of the record_utils thread
      mainloop_enable(1);
      return 0;
    }
    
    /*!
    \code
    // Needed by the special version of win_client to be used for 
    // recording video sequences. This command is similar to "recimg"
    // the unique difference is that with "recimgdsp" only the disparity 
    // map is sent to the client. In both cases, the current values of
    // the in and out counters are also sent.
    // From version 2.3.10.8 this command can be combined with "start_rec" 
    // and "stop_rec" in order to have a full-framerate recording.
    if(strcmp(buffer,"recimgdsp")==0)
    {
        //...
    \endcode
    */
    /*!
    \code
    // Needed by the special version of win_client to be used for 
    // recording video sequences. This command is similar to "recimgdsp"
    // the unique difference is that with "recimg" not only the disparity 
    // map is sent to the client but also the two images (left and right). 
    // In both cases, the current values of the in and out counters are
    // also sent.
    if(strcmp(buffer,"recimg")==0)
    {
        //...
    \endcode
    */
    if(strcmp(buffer,"recimg")==0 || strcmp(buffer,"recimgdsp")==0)
    {
        if (strcmp(buffer,"recimgdsp")==0)
        {
          if (ru_reply_data() != 0)
            printf("recimgdsp: request received before start or after stop\n"); 
        }
        else // if(strcmp(buffer,"recimg")==0)
        {
          printf("recimg: read and send data.\n"); 
          
          read(pxa_qcp,Frame,imagesize);
          get_images(Frame,0); 
        
          /*if(acq_mode & 0x0100)	//tracking
          {
              pthread_mutex_lock(&mainlock);
              //detectAndTrack(Frame_DSP,people_rec[0],people_rec[1],count_enabled,get_parms("threshold"),get_parms("dir"));
              detectAndTrack(
                Frame_DSP,
                people_rec[0],people_rec[1],
                count_enabled,
                get_parms("threshold"),
                get_parms("dir"),
                move_det_en
#ifdef USE_NEW_TRACKING
                , (limit_line_Down-limit_line_Up+1)/4);
#else
                );
#endif

              record_counters(people_rec[people_dir],people_rec[1-people_dir]);
              pthread_mutex_unlock(&mainlock);
          }*/
        
          
          {
            Send(fd,(char *)Frame_DX,NN*sizeof(char));
            Send(fd,(char *)Frame_SX,NN*sizeof(char));
            Send(fd,(char *)Frame_DSP,NN*sizeof(char));    
            Send(fd,(unsigned long*)&people_rec[0], sizeof(people_rec[0]));
            Send(fd,(unsigned long*)&people_rec[1], sizeof(people_rec[1]));
            unsigned char testin_val = input_test0;
            Send(fd,(unsigned char*)&testin_val,sizeof(testin_val));
            testin_val = input_test1;
            Send(fd,(unsigned char*)&testin_val,sizeof(testin_val));
            
            printf("recimg: all data sent.\n"); 
          }
        }
        
        return 0;
    }


    if(strcmp(buffer,"limit_track")==0)
    {
        unsigned char lim_sx;
        unsigned char lim_dx;
        unsigned char lim_sx_riga_start;
        unsigned char lim_dx_riga_start;
        unsigned char lim_sx_riga_end;
        unsigned char lim_dx_riga_end;
        unsigned char lim_up_line;
        unsigned char lim_down_line;

        Recv(fd,(char *)&lim_sx,sizeof(lim_sx));
        Recv(fd,(char *)&lim_dx,sizeof(lim_dx));
        Recv(fd,(char *)&lim_sx_riga_start,sizeof(lim_sx_riga_start));
        Recv(fd,(char *)&lim_dx_riga_start,sizeof(lim_dx_riga_start));
        Recv(fd,(char *)&lim_sx_riga_end,sizeof(lim_sx_riga_end));
        Recv(fd,(char *)&lim_dx_riga_end,sizeof(lim_dx_riga_end));
        Recv(fd,(char *)&lim_up_line,sizeof(lim_up_line));
        Recv(fd,(char *)&lim_down_line,sizeof(lim_down_line));
        // debug OMAR 
       /*  printf("lim_sx: %d \n",lim_sx);
        printf("lim_dx: %d \n",lim_dx);
        printf("lim_sx_riga_start: %d \n",lim_sx_riga_start);
        printf("lim_sx_riga_end: %d \n",lim_sx_riga_end);
        printf("lim_dx_riga_start: %d \n",lim_dx_riga_start);
        printf("lim_dx_riga_end: %d \n",lim_dx_riga_end);
        printf("lim_up_line: %d \n",lim_up_line);
        printf("lim_down_line: %d \n",lim_down_line); */
        // fine debug OMAR
        if(lim_sx>71 || lim_dx<=91 || lim_dx>160) // if(lim_sx>70 || lim_dx<=90 || lim_dx>159)
        {
            printf("Limit out of range limitsx=%d, limitdx=%d\n",lim_sx,lim_dx);
            return -1;
        }
        if(write_parms("sxlimit",(unsigned short)lim_sx) < 0)
            return -1;
        if(save_parms("sxlimit",(unsigned short)lim_sx)< 0)
            return -1;
        if(write_parms("dxlimit",(unsigned short)lim_dx) < 0)
            return -1;
        if(save_parms("dxlimit",(unsigned short)lim_dx)< 0)
            return -1;

        if(write_parms("sxlimit_riga_start",(unsigned short)lim_sx_riga_start) < 0)
            return -1;
        if(save_parms("sxlimit_riga_start",(unsigned short)lim_sx_riga_start)< 0)
            return -1;
        if(write_parms("dxlimit_riga_start",(unsigned short)lim_dx_riga_start) < 0)
            return -1;
        if(save_parms("dxlimit_riga_start",(unsigned short)lim_dx_riga_start)< 0)
            return -1;

        if(write_parms("sxlimit_riga_end",(unsigned short)lim_sx_riga_end) < 0)
            return -1;
        if(save_parms("sxlimit_riga_end",(unsigned short)lim_sx_riga_end)< 0)
            return -1;
        if(write_parms("dxlimit_riga_end",(unsigned short)lim_dx_riga_end) < 0)
            return -1;
        if(save_parms("dxlimit_riga_end",(unsigned short)lim_dx_riga_end)< 0)
            return -1;

        if(write_parms("up_line_limit",(unsigned short)lim_up_line) < 0)
            return -1;
        if(save_parms("up_line_limit",(unsigned short)lim_up_line)< 0)
            return -1;
        if(write_parms("down_line_limit",(unsigned short)lim_down_line) < 0)
            return -1;
        
        ////////////////////
        // 20091119 eVS
        // - Modifications in the no-tracking zone can imply changes in 
        //   the move detection zone.
        
        //return save_parms("down_line_limit",(unsigned short)lim_down_line);
        if (save_parms("down_line_limit",(unsigned short)lim_down_line) < 0)
            return -1;
        return check_mov_det_parms();
        // 20091119 eVS
        ////////////////////
    }
    if(strcmp(buffer,"cond_diff_1p")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("cond_diff_1p",(unsigned short)value);
        save_parms("cond_diff_1p",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"useBGsub")==0) // 20120426 eVS
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        pthread_mutex_lock(&acq_mode_lock); 
        if (value == 0)
        {
          memset(Bkgvec,0,sizeof(Bkgvec));
          memset(svec,0,sizeof(svec));
          //vm_bkg = 0;
        }
        else
        {
          FILE *in;
          if((in = fopen(bg_filename,"rb")))
          {
              fread(Bkgvec,sizeof(Bkgvec),1,in);
              fread(svec,sizeof(svec),1,in);
              fread(&vm_bkg,sizeof(vm_bkg),1,in);
              fclose(in);
          }
        }
        
        // copio il background caricato nella variabile usata per l'auto background
        memcpy(Bkgvectmp,Bkgvec,NN);
        pthread_mutex_unlock(&acq_mode_lock); 
        
        return 0;
    }
    if(strcmp(buffer,"removeBG")==0)  // 20120426 eVS
    {
      pthread_mutex_lock(&acq_mode_lock); 
      memset(Bkgvec,0,sizeof(Bkgvec));
      memset(svec,0,sizeof(svec));
      //vm_bkg = 0;    
      pthread_mutex_unlock(&acq_mode_lock); 
       
      // copio il background azzerato nella variabile usata per l'auto background
      //memcpy(Bkgvectmp,Bkgvec,NN);

      pthread_mutex_lock(&rdlock); 
      char command[255];       
      sprintf(command,"rm -rf %s",bg_filename);
      system(command); // deleting background file to avoid to load it at next boot
      
      // 20130122 eVS, instead of only remove background file we save an empty 
      // background in order to be compatible with the recording procedure 
      // which uses ftp to save the background
      FILE *out;
      if((out = fopen(bg_filename,"wb")))
      {
        fwrite(Bkgvec,sizeof(Bkgvec),1,out);
        fwrite(svec,sizeof(svec),1,out);
        fwrite(&vm_bkg,sizeof(vm_bkg),1,out);
        fclose(out);
      }
      pthread_mutex_unlock(&rdlock);
      return 0;      
    }
    if(strcmp(buffer,"isBGempty")==0)  // 20130121 eVS
    {
      bool isBGempty = true;
      int i = 0;
      
      pthread_mutex_lock(&acq_mode_lock); 
      while (isBGempty && i<NN)
      {
        isBGempty = (Bkgvec[i] == 0);
        ++i;
      }
      pthread_mutex_unlock(&acq_mode_lock); 
      
      printf("isBGempty: %s\n", (isBGempty) ? "true" : "false");
      Send(fd,(char *)&isBGempty, sizeof(isBGempty));
      return 0;
    }
     
     //20130802 eVS : to  CheckBG  without activation of OOR manager
    if(strcmp(buffer,"check_bg")==0)
    {
      
      bool is_bg_reliable = _is_background_reliable_for_out_of_range_handle();     
      pthread_mutex_lock(&acq_mode_lock); 
      pthread_mutex_unlock(&acq_mode_lock); 
      Send(fd,(char *)&is_bg_reliable, sizeof(is_bg_reliable));
      return 0;
    }
    // 20130802 eVS : to Enable Handle OOR 
    if(strcmp(buffer,"enable_handle_oor")==0)
    {
      unsigned char value;
      Recv(fd,(char *)&value,sizeof(value));
      bool state = (value == 1) ? true : false;
      _enable_out_of_range_handle(state);
      return 0;  
    }
    
    if(strcmp(buffer,"eye_conn_check")==0)
    {

      int i;
      int andbitbitleft,orbitbitleft,andbitbitright,orbitbitright;
      int pixel_sx,pixel_dx;
      int pixel_sx_tmp1,pixel_sx_tmp2,pixel_dx_tmp1,pixel_dx_tmp2;
      int resSx,resDx;
      andbitbitleft = 1023;
      orbitbitleft = 0;
      andbitbitright = 1023;
      orbitbitright = 0;
      resSx = 0;
      resDx = 0;

      images_enabled = 0;
      mainloop_enable(0);	// stopping mainloop before saving the background

      i2cstruct.reg_addr =  MUX_MODE;
      i2cstruct.reg_value = MUX_MODE_10_NOFPN_SX;
      ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

      for(i=0;i<20;i++)  
        read(pxa_qcp,Frame,imagesize); 
      get_images(Frame,0);
      int tmp=0;
      for(int ind=0;ind<NX*NY;ind++)
         {
         tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
         Frame10_SX[ind] = tmp;
         }

      i2cstruct.reg_addr =  MUX_MODE;
      i2cstruct.reg_value = MUX_MODE_10_NOFPN_DX;
      ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct); 
      for(i=0;i<10;i++)  
        read(pxa_qcp,Frame,imagesize);

      get_images(Frame,0);
      tmp=0;
      for(int ind=0;ind<NX*NY;ind++)
         {
         tmp = ((Frame_SX[ind] << 8) & 0x300) | (Frame_DX[ind] & 0xff) & 0x3ff;
         Frame10_DX[ind] = tmp;
         }

      i2cstruct.reg_addr =  MUX_MODE;
      i2cstruct.reg_value = acq_mode & 0x00FF;
      ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

      for(i=0;i<NX*NY;i++)
         {
         andbitbitleft &= Frame10_SX[i];
         orbitbitleft |= Frame10_SX[i];
         }
      for(i=0;i<NX*NY;i++)
         {
         andbitbitright &= Frame10_DX[i];
         orbitbitright |= Frame10_DX[i];
         }
      for(i=0;i<NX*NY;i++)
         {
         pixel_sx=Frame10_SX[i];
         pixel_dx=Frame10_DX[i];
      
         pixel_sx_tmp1 = pixel_sx & 0x001; 
         pixel_sx_tmp2 = pixel_sx & 0x002;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x001;
         pixel_sx_tmp1 = pixel_sx & 0x002; 
         pixel_sx_tmp2 = pixel_sx & 0x004;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x002;
         pixel_sx_tmp1 = pixel_sx & 0x004; 
         pixel_sx_tmp2 = pixel_sx & 0x008;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x004;
         pixel_sx_tmp1 = pixel_sx & 0x008; 
         pixel_sx_tmp2 = pixel_sx & 0x010;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x008;
         pixel_sx_tmp1 = pixel_sx & 0x010; 
         pixel_sx_tmp2 = pixel_sx & 0x020;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x010;
         pixel_sx_tmp1 = pixel_sx & 0x020; 
         pixel_sx_tmp2 = pixel_sx & 0x040;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x020;
         pixel_sx_tmp1 = pixel_sx & 0x040; 
         pixel_sx_tmp2 = pixel_sx & 0x080;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x040;
         pixel_sx_tmp1 = pixel_sx & 0x080; 
         pixel_sx_tmp2 = pixel_sx & 0x100;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x080;
         pixel_sx_tmp1 = pixel_sx & 0x100; 
         pixel_sx_tmp2 = pixel_sx & 0x200;  
         if(pixel_sx_tmp1  == pixel_sx_tmp2)
           resSx |= 0x100;

         pixel_dx_tmp1 = pixel_dx & 0x001; 
         pixel_dx_tmp2 = pixel_dx & 0x002;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x001;
         pixel_dx_tmp1 = pixel_dx & 0x002; 
         pixel_dx_tmp2 = pixel_dx & 0x004;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x002;
         pixel_dx_tmp1 = pixel_dx & 0x004; 
         pixel_dx_tmp2 = pixel_dx & 0x008;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x004;
         pixel_dx_tmp1 = pixel_dx & 0x008; 
         pixel_dx_tmp2 = pixel_dx & 0x010;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x008;
         pixel_dx_tmp1 = pixel_dx & 0x010; 
         pixel_dx_tmp2 = pixel_dx & 0x020;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x010;
         pixel_dx_tmp1 = pixel_dx & 0x020; 
         pixel_dx_tmp2 = pixel_dx & 0x040;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x020;
         pixel_dx_tmp1 = pixel_dx & 0x040; 
         pixel_dx_tmp2 = pixel_dx & 0x080;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x040;
         pixel_dx_tmp1 = pixel_dx & 0x080; 
         pixel_dx_tmp2 = pixel_dx & 0x100;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x080;
         pixel_dx_tmp1 = pixel_dx & 0x100; 
         pixel_dx_tmp2 = pixel_dx & 0x200;  
         if(pixel_dx_tmp1  == pixel_dx_tmp2)
           resDx |= 0x100;

         }
      resSx |= 0x200;  // Metto ad 1 anche questo bit. I confronti tra bit sono (10-1 = 9)
      resDx |= 0x200;   

      Send(fd,(int *)&andbitbitleft, sizeof(andbitbitleft));	
      Send(fd,(int *)&orbitbitleft, sizeof(orbitbitleft));	
      Send(fd,(int *)&resSx, sizeof(resSx));
      Send(fd,(int *)&andbitbitright, sizeof(andbitbitright));	
      Send(fd,(int *)&orbitbitright, sizeof(orbitbitright));	
      Send(fd,(int *)&resDx, sizeof(resDx));
      usleep(300000);
      mainloop_enable(1);		// restarting the mainloop
      usleep(300000);
      images_enabled = 1;

      return 0;
    }
    if(strcmp(buffer,"diagnostic_en")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        
        if (diagnostic_en != value) // 20111207 eVs, added check and re-init of the status variables
          pcn_status = old_pcn_status = 0;
          
        write_parms("diagnostic_en",(unsigned short)value);
        save_parms("diagnostic_en",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"pcn1001_status")==0)
    { 
        unsigned char current_status;
        unsigned char error_pcn_status_code; // 20111207 eVS, added the now missing variable
        
        pthread_mutex_lock(&mainlock);      
        if(diagnostic_en) 
        {
            current_status = (pcn_status == 0) ? 1 : 0;
            error_pcn_status_code=pcn_status; //0; 20111207 eVS, now pcn_status containts the error code
        }
        else
        {
            current_status=1;
            error_pcn_status_code=0;
        }
        pthread_mutex_unlock(&mainlock);
        
        Send(fd,(unsigned char*)&current_status, sizeof(current_status));
        Send(fd,(unsigned char*)&error_pcn_status_code, sizeof(error_pcn_status_code));
        return 0;
    }
    if(strcmp(buffer,"move_det_col0")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_col0",(unsigned short)value);
        save_parms("move_det_col0",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_col1")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_col1",(unsigned short)value);
        save_parms("move_det_col1",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_row0")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_row0",(unsigned short)value);
        save_parms("move_det_row0",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_row1")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_row1",(unsigned short)value);
        save_parms("move_det_row1",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_alfareg")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_alfareg",(unsigned short)value);
        save_parms("move_det_alfareg",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_en")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("move_det_en",(unsigned short)value);
        save_parms("move_det_en",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_thr")==0)
    {
        int value;
        Recv(fd,(int *)&value,sizeof(value)); 
        write_parms("move_det_thr",(unsigned short)value);
        save_parms("move_det_thr",(unsigned short)value);
        return 0;
    }
    if(strcmp(buffer,"move_det_val")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        Send(fd,(char *)&mov_det_left, sizeof(mov_det_left));
        Send(fd,(char *)&mov_det_right, sizeof(mov_det_right));
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(buffer,"move_det_status")==0)
    {
        unsigned char current_status = 0;    
        pthread_mutex_lock(&mainlock);
        // eVS, perchè c'è un OR bit a bit ??? anzichè un OR classico ||
        //if(count_true_false == true | move_det_en == false) 
        if(count_true_false == true || move_det_en == false) 
            current_status=1;     
        pthread_mutex_unlock(&mainlock);
        Send(fd,(char *)&current_status, sizeof(current_status));
        return 0;
    }
    if(strcmp(buffer,"door_stairs_en")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
        write_parms("door_stairs_en",(unsigned short)value);
        save_parms("door_stairs_en",(unsigned short)value);
        return 0;
    }
    /*!
    \code
    // Command to change the value of the option "Disable automatic background"
    //
    // 20090506 Lisbona
    if(strcmp(buffer,"dis_autobkg")==0)
    {
        //...
    \endcode
    */
    //20090506 Lisbona
    if(strcmp(buffer,"dis_autobkg")==0)
    {
      unsigned char value;
      Recv(fd,(char *)&value,sizeof(value));
      write_parms("dis_autobkg",(unsigned short)value);
      save_parms("dis_autobkg",(unsigned short)value);
      return 0;    
    }
    /*!
    \code
    // Command to change the value of the "Door kind" (in, out, in/out)
    //
    // 20130411 eVS, door_size instead of door_kind (for 2.3.10.7)
    if(strcmp(buffer,"door_size")==0)
    {
        //...
    \endcode
    */
    // 20130411 eVS, door_size instead of door_kind (for 2.3.10.7)
    if(strcmp(buffer,"door_size")==0)
    {
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
      
        if(total_sys_number>1)
        {
            //if(current_sys_number==1) //If I'm a master and there is a slave
            //    SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            //else 
                return -1; //in wg mode this command is permitted only from the Master
        }

        write_parms("door_size",(unsigned short)value);
        save_parms("door_size",(unsigned short)value);
        return 0;    
    }
    
    /*!
    \code
    // Command to change the value of the handle_oor (0 or 1)
    // Avaible only version 2.3.11.2
    if(strcmp(buffer,"handle_oor")==0)
    {
        //...
    \endcode
    */
    // eVS added 20130716
    if(strcmp(buffer,"handle_oor")==0)
    {
        
        unsigned char value;
        Recv(fd,(char *)&value,sizeof(value));
      
        if(total_sys_number>1)
        {
            //if(current_sys_number==1) //If I'm a master and there is a slave
            //    SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            //else 
                return -1; //in wg mode this command is permitted only from the Master
        }

        write_parms("handle_oor",(unsigned short)value);
        save_parms("handle_oor",(unsigned short)value);
        return 0;    
    }
    
    return -1;
}

