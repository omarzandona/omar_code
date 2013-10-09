/*!
\file images_fpga.cpp
\brief Interfacciamento tra imgserver ed FPGA, per la lettura delle immagini e altri parametri.

Contiene le funzioni per prelevare le immagini sorgenti a toni di grigio, 
mappa di disparit&agrave;, parametri di motion detection, valor medio delle immagini (get_images()), 
calcolo valore medio dello sfondo (background()), salvataggio del fixed pattern noise (save_fpn()) 
e reset della memoria flash (erase_flash()).

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/


extern unsigned char Frame_DX[NN];   //!< Immagine destra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_SX[NN];   //!< Immagine sinistra a 256 livelli di grigio (#NX*#NY=160*120).
extern unsigned char Frame_DSP[NN];  //!< Mappa di disparit&agrave; #NX*#NY=160*120 (16 livelli di disparit&agrave; distribuiti su 256 livelli di grigio).

/*!
\brief Estrazione e deinterlacciamento del blocco dati #NX*#NY*4=160*120*4=320*240 bytes.

In particolare viene estratto dal buffer #NX*#NY*4=160*120*4=320*240 le immagini rettificate destra (#Frame_DX) e sinistra (#Frame_SX), 
la mappa di disparit&agrave; a 16 livelli (#Frame_DSP), 
i parametri di motion detection (#mov_dect_15_23l, #mov_dect_8_15l, #mov_dect_0_7l, #mov_dect_15_23r, 
#mov_dect_8_15r e #mov_dect_0_7r), 
il valor medio di una delle due immagini (#vm_img).

\param orig buffer #NX*#NY*4=160*120*4=320*240 letto dall'FPGA.
\param img NON USATO (???)
*/
void get_images(unsigned char *orig,int img)
{
    unsigned long *ptr;
    int i,j,step;

    ptr = (unsigned long *) orig; // puntatore ad una zona di memoria a 32bits

    for(j=0;j<NY;j++)
    {
        step = j*NX;
        for(i=0;i<(NX >> 1);i++)
        { 
            // vengono prelevati i 32 bits appartenenti alla finestra 2x2 del blocco 160*120*4=320*240
            // che contiene il primo pixel di ciascuna immagine (left and right) piu' mappa di disparita'
            // piu' le informazioni relative al valor medio e al motion detection

    /*!
    \code
    // Il blocco dati 160*120*4=320*240 si suddivide 160*120 finestre piu' piccole da 2x2 pixels:
    // il primo pixel (posizione top-left) si riferisce all'immagine sinistra
    // il secondo pixel (posizione top-right) si riferisce all'immagine sinistra
    // e il terzo pixel (posizione bottom-left) si riferisce alla mappa di disparita'.
    // Se la modalita' tracking e' attiva allora il terzo pixel si riferisce al valor medio
    // ed ai parametri di motion detection.

    vm_img = (*(ptr+(NX>>1)) & 0x0000FF00)>>8;	
    mov_dect_15_23l = (*(ptr+(NX>>1)) & 0xFF000000) >> 24;
    mov_dect_8_15l = (*(ptr+1+(NX>>1)) & 0x0000FF00)>>8;
    mov_dect_0_7l = (*(ptr+1+(NX>>1)) & 0xFF000000) >> 24;
    mov_dect_15_23r = (*(ptr+2+(NX>>1)) & 0x0000FF00)>>8;
    mov_dect_8_15r = (*(ptr+2+(NX>>1)) & 0xFF000000) >> 24;
    mov_dect_0_7r = (*(ptr+3+(NX>>1)) & 0x0000FF00)>>8;

    if(!(acq_mode & 0x0100)) //no tracking mode
    {
        Frame_DX[step] = *ptr & 0x000000FF;
        Frame_SX[step] = (*ptr & 0x0000FF00) >> 8;
    }
    
    if((i<<1) < BORDER_X || (i<<1) >= NX-BORDER_X || j < BORDER_Y || j >= NY-BORDER_Y) 
        Frame_DSP[step]=0;
    else
        Frame_DSP[step] = (*(ptr+(NX>>1)) & 0x0000000F) << 4;            
    \endcode
    */
            
            // mean value: viene prelevato il byte relativo al pixel della 2°riga 1°colonna del buffer 160*120*4=320*240
            if(i==0&&j==0) vm_img = (*(ptr+(NX>>1)) & 0x0000FF00)>>8;	
            if(i==0&&j==0) mov_dect_15_23l = (*(ptr+(NX>>1)) & 0xFF000000) >> 24;
            if(i==0&&j==0) mov_dect_8_15l = (*(ptr+1+(NX>>1)) & 0x0000FF00)>>8;
            if(i==0&&j==0) mov_dect_0_7l = (*(ptr+1+(NX>>1)) & 0xFF000000) >> 24;
            if(i==0&&j==0) mov_dect_15_23r = (*(ptr+2+(NX>>1)) & 0x0000FF00)>>8;
            if(i==0&&j==0) mov_dect_8_15r = (*(ptr+2+(NX>>1)) & 0xFF000000) >> 24;
            if(i==0&&j==0) mov_dect_0_7r = (*(ptr+3+(NX>>1)) & 0x0000FF00)>>8;

            //  average_img_val
            if(!(acq_mode & 0x0100))	//no tracking
            {
                // viene prelevato il pixel "step" (sia left che right)
                Frame_DX[step]  = *ptr & 0x000000FF; // The byte is taken on the pixels of the first row second column buffer 160*120*4=320*240. //viene prelevato il byte relativo al pixel della 1°riga 2°colonna del buffer 160*120*4=320*240
                Frame_SX[step]  = (*ptr & 0x0000FF00) >> 8; // The byte is taken on the pixels of the first row first column of the buffer 160*120*4=320*240. //viene prelevato il byte relativo al pixel della 1°riga 1°colonna del buffer 160*120*4=320*240
            }

            // se non sono sui bordi allora viene prelevato il pixel "step" della disparity map
            if((i<<1) < BORDER_X || (i<<1) >= NX-BORDER_X || j < BORDER_Y || j >= NY-BORDER_Y) 
                Frame_DSP[step]=0;
            else
                Frame_DSP[step] = (*(ptr+(NX>>1)) & 0x0000000F) << 4;

            step++;

            if(!(acq_mode & 0x0100))	//no tracking
            {
                // viene prelevato il pixel "step+1" (sia left che right)
                Frame_DX[step]  = (*ptr & 0x00FF0000) >> 16; 
                Frame_SX[step]  = (*ptr & 0xFF000000) >> 24;
            }

            // se non sono sui bordi allora viene prelevato il pixel "step+1" della disparity map
            if((i<<1) < BORDER_X || (i<<1) >= NX-BORDER_X || j < BORDER_Y || j >= NY-BORDER_Y) 
                Frame_DSP[step]=0;
            else
                Frame_DSP[step] = (*(ptr+(NX>>1)) & 0x000F0000) >> 12;

            step++; 

            ptr++;
        } 

        // salta una riga corrispondente ad un blocco di 320 bytes ovvero NX/2*4
        ptr += (NX >> 1);		// 2-rows step
    }
}



/*!
\brief Usata in Commands.cpp in corrispondenza del comando "meanv".
*/
void get_10bit_image(unsigned short *dest,unsigned char *src)
{
    unsigned long *ptr;
    int i,j,step;

    ptr = (unsigned long *) src;

    for(j=0;j<NY;j++)
    {
        step = j*NX;
        for(i=0;i<(NX >> 1);i++)
        {  
            dest[step]  = *ptr & 0x000000FF; 
            dest[step]  |= (*ptr & 0x0000FF00);
            step += 1; 
            dest[step]  = (*ptr & 0x00FF0000) >> 16; 
            dest[step]  |= (*ptr & 0xFF000000) >> 16;
            step += 1;
            ptr++;
        } 
        ptr += (NX >> 1);
    }

}


/*
NOT USED SO COMMENTED
int get_8bit_image(unsigned char *dest,unsigned short *src)
{
    unsigned long average;
    unsigned short inf;
    int tmp;
    int i;

    average = 0;
    for(i=0;i<NN;i++)
        average += (src[i] & 0x3FF);

    average /= NN;

    inf = (average-127)>0 ? (average-127) : 0;
    for(i=0;i<NN;i++)
    {
        tmp = src[i] - inf;

        if(tmp >=0 && tmp <= 255) dest[i] = (unsigned char) tmp;
        if(tmp < 0)    dest[i] = 0;
        if(tmp > 255)  dest[i] = 255;  
    }

    return average;
}
*/

/*!
\brief Calcola il valor medio dello sfondo contenuto in src, inoltre calcola l'FPN.

Viene calcolato lo scarto che c'&egrave; tra le due immagini 
acquisite puntando il sensore su di uno sfondo bianco e il valor medio dello sfondo. 
Poi l'FPN viene successivamente salvato in appositi registri dell'FPGA dalla funzione save_fpn().
*/
int background(char *dest,unsigned short *src)
{
    unsigned long average;
    int tmp;
    int i;

    average = 0;
    for(i=0;i<NN;i++)
        average += (src[i] & 0x3FF);

    average /= NN;

    for(i=0;i<NN;i++)
    {
        tmp = average - src[i];

        dest[i] = (char) tmp;
        if(tmp < -128) dest[i] = (char)-128;
        if(tmp > 127)  dest[i] = (char)127;  
    }

    return average;
}


/*!
\brief Cancellazione della memoria flash dell'FPGA.

Prima di cancellare il contenuto della memoria flash dell'FPGA fermo l'acquisizione delle immagini, 
poi resetto la flash e invio un risultato al win_client mediante socket TCP. Infine riattivo l'acquisizione 
delle immagini. Start e stop acquisizione viene effettuata mediante chiamata ioctl(), passando il comando VIDIOCCAPTURE.
*/
int erase_flash()
{
    unsigned char addr;
    int arg;

    arg = VIDEO_STOP;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);
    usleep(33000);

    /* flash reset */
    addr = FL_RST;
    arg = 1;
    i2cstruct.reg_addr =  addr;
    i2cstruct.reg_value = arg;

    if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) 
    {
        unsigned char err=0xFF;
        Send(sockfd,(char*)&err,sizeof(err));
        return -1;  
    }
    else
    {
        usleep(150000);
        Send(sockfd,(bool*)&fpn_counter,sizeof(fpn_counter));
        printf("Flash erased!\n"); 
    }

    arg = VIDEO_START;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);

    return 0;
}


/*!
\brief Salvataggio delle informazioni relative all'FPN e all'ODC.

Mediante la chiamata di sistema ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct) 
vengono memorizzate (all'interno di appositi registri dell'FPGA: #FL_LOBYTE e #FL_HIBYTE) le informazioni relative 
al pattern fisso di rumore da eliminare dalle immagini sorgenti 
e parametri relativi alla correzione della distorsione introdotta dall'ottica.
*/
int save_fpn()
{
    unsigned char row;
    unsigned long index;
    unsigned char tmp;
    int i,j,arg;
    unsigned char wrsel;

    i2cstruct.reg_addr =  FL_WRSEL;

    ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
    wrsel=i2cstruct.reg_value;  

    arg = VIDEO_STOP;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);
    usleep(33000);

    /* writing parameters on flash */
    row = 0;
    index = 0;
    fpn_counter = 0;
    for(j=0;j<NY+16;j++)	/*ODC parameters go from line 16 to line 135*/
    {
        i2cstruct.reg_addr =  FL_SETROW;
        i2cstruct.reg_value = j;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        for(i=0;i<NX;i++)
        {      

            if(wrsel & 0x01) //only FPN
            {

                /*  FPN low byte */
                i2cstruct.reg_addr =  FL_LOBYTE;
                i2cstruct.reg_value = (j < NY) ? FPN_DX[j*NX+i]:0;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /* FPN high byte */
                i2cstruct.reg_addr =  FL_HIBYTE;
                i2cstruct.reg_value = (j < NY) ? FPN_SX[j*NX+i]:0;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

            }

            if(wrsel & 0x02) //only ODC
            {  
                if(j>=16) index = ((j-16)*NX+i) << 3;

                /*  ODC low byte 1*/
                tmp = 0;
                if(j>=16)
                    tmp |= (ODCbuf[index] & 0x3f) | ((ODCbuf[index+1] & 0x03) << 6);

                i2cstruct.reg_addr =  FL_LOBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /* ODC high byte 1*/
                tmp = 0;
                if(j>=16)
                    tmp |= ((ODCbuf[index+1] & 0x3c) >> 2) | ((ODCbuf[index+2] & 0x0f) << 4);

                i2cstruct.reg_addr =  FL_HIBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /*  ODC low byte 2*/
                tmp = 0;
                if(j>=16)
                    tmp |= ((ODCbuf[index+2] & 0x30) >> 4) | ((ODCbuf[index+3] & 0x3f) << 2);

                i2cstruct.reg_addr =  FL_LOBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /* ODC high byte 2*/
                tmp = 0;
                if(j>=16)
                    tmp |= ((ODCbuf[index+4] & 0x1f)) | ((ODCbuf[index+5] & 0x07) << 5);

                i2cstruct.reg_addr =  FL_HIBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /*  ODC low byte 3*/
                tmp = 0;
                if(j>=16)
                    tmp |= ((ODCbuf[index+5] & 0x18) >> 3) | ((ODCbuf[index+6] & 0x1f) << 2);

                i2cstruct.reg_addr =  FL_LOBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

                /* ODC high byte 3*/
                tmp = 0;
                if(j>=16)
                    tmp |= ((ODCbuf[index+7] & 0x1f));

                i2cstruct.reg_addr =  FL_HIBYTE;
                i2cstruct.reg_value = tmp;
                ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

            }


        }
        fpn_counter++;
        Send(sockfd,(bool*)&fpn_counter,sizeof(fpn_counter));
    }

    if (wrsel & 0x01) printf("FPN  saved!\n");
    if (wrsel & 0x02) printf("ODC  saved!\n"); 

    arg = VIDEO_START;
    ioctl(pxa_qcp, VIDIOCCAPTURE, arg);

    return 0;
}

