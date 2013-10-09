/*!
\file serial_port.cpp
\brief Serial port comunication routines. 
OMAR
The comunication via serial port can be tested by using the application RS485_gui
(if you have no serial port you can install the RS-485 emulator via USB). 
The comunication is based on a Serial Network Protocol (SNP) described in the
chapter 4 of the PCN-1001 User Manual.

This protocol allows to receive and send messages via serial port by the routines
SNP_Recv() and SNP_Send(). To properly build a packet for this protocol, SNP_Send() 
uses BuildSNPCommand(). To read/write bytes from/to the serial port at low level, SNP_Recv()
and SNP_Send() use read_port() and write_port().

Once a message is received it is interpreted by SNP_reply() which parses the data
to catch the sent command in order to reply properly.

The imgserver uses two threads to continuously check the two serial ports: 
ser_loopttyS0() and ser_loopttyS1().

<B>SNP packet form:</B>

\verbatim
Name            Length in bytes             Contents 
PreAmble            5                       a sequence of 5 bytes equal to 0xFF
                                            in modo da tempo per cambiare la direzione di trasmissione
StartCharacter      1                       SOH (Start Of Header) equals to 0x01 
Source              1                       sender
Destination         1                       recipient
TotalPacketNumber   1                       total number of packets
PacketNumber        1                       number of the current packet (starting from 1 and not 0)
DataLength          2                       data length (the LSB - Least Significative Byte - comes first)
Data                0 to MAX_DATA_LEN       data
CRC16               2                       CRC16 (x16 x15 x2 x0, the LSB - Least Significative Byte - comes first)
Postamble           1                       0xFF.
\endverbatim 


<B>List of commands</B>

\verbatim 
"ker_version" ask for the version of the kernel
"sys_version" ask for the version of the OS
"fw_version"  ask for the version of the FPGA bitstream
"version"     ask for the version of the imgserver 
"enable_pc"   enable/disable people counting
"gcounters"   ask for the value of the counters
"reset"       reset counters
"address"     set the IP address
"sdatetime"   set date and time
"input0/1"    set the optocoupled input function (do_nothing(), enable_counting(), reset_counter())
"outtime0/1"  set the GPO open time
"sled"        set the led intensity
"gdatetime"   read date and time
"polling"     just for polling purposes
\endverbatim 

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

extern unsigned char total_sys_number;
extern int num_pers;

enum UPDATE_ERR_CODES {
    UEC_CMD_NOT_EXISTING = -2000,
    UEC_CMD_INCOHERENT = -1000,
    UEC_GENERIC_ERR = -3,
    UEC_FAILED_MALLOC = -4,
    UEC_PACKET_UNEXPECTED = -5,
    UEC_FAILED_FWRITE = -7,
    UEC_FAILED_FOPEN = -8,
    UEC_CMD_UNEXPECTED = -9
};

/*! \var CrcTable
\brief CRC table
*/
static const unsigned short CrcTable[MAX_CRC_DIGITS] =
{
    0,32773,32783,   10,32795,   30,   20,32785,32819,   54,
    60,32825,   40,32813,32807,   34,32867,  102,  108,32873,
    120,32893,32887,  114,   80,32853,32863,   90,32843,   78,
    68,32833,32963,  198,  204,32969,  216,32989,32983,  210,
    240,33013,33023,  250,33003,  238,  228,32993,  160,32933,
    32943,  170,32955,  190,  180,32945,32915,  150,  156,32921,
    136,32909,32903,  130,33155,  390,  396,33161,  408,33181,
    33175,  402,  432,33205,33215,  442,33195,  430,  420,33185,
    480,33253,33263,  490,33275,  510,  500,33265,33235,  470,
    476,33241,  456,33229,33223,  450,  320,33093,33103,  330,
    33115,  350,  340,33105,33139,  374,  380,33145,  360,33133,
    33127,  354,33059,  294,  300,33065,  312,33085,33079,  306,
    272,33045,33055,  282,33035,  270,  260,33025,33539,  774,
    780,33545,  792,33565,33559,  786,  816,33589,33599,  826,
    33579,  814,  804,33569,  864,33637,33647,  874,33659,  894,
    884,33649,33619,  854,  860,33625,  840,33613,33607,  834,
    960,33733,33743,  970,33755,  990,  980,33745,33779, 1014,
    1020,33785, 1000,33773,33767,  994,33699,  934,  940,33705,
    952,33725,33719,  946,  912,33685,33695,  922,33675,  910,
    900,33665,  640,33413,33423,  650,33435,  670,  660,33425,
    33459,  694,  700,33465,  680,33453,33447,  674,33507,  742,
    748,33513,  760,33533,33527,  754,  720,33493,33503,  730,
    33483,  718,  708,33473,33347,  582,  588,33353,  600,33373,
    33367,  594,  624,33397,33407,  634,33387,  622,  612,33377,
    544,33317,33327,  554,33339,  574,  564,33329,33299,  534,
    540,33305,  520,33293,33287,  514
};

extern pthread_mutex_t acq_mode_lock; // 20100517 eVS

/*! 
*  \brief Reset della porta seriale indicata con fd.
*  \param fd file descriptor della porta seriale
*/
void reset_serial(int fd)
{
    /*! \code
    struct termios temptio;
    // clear struct for new port settings
    bzero(&temptio, sizeof(temptio));
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&temptio);
    \endcode */
    struct termios temptio;
    // clear struct for new port settings
    bzero(&temptio, sizeof(temptio));
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&temptio);
}


/*! 
*  \brief Configurazione della porta seriale.
*  \param fd file descriptor della porta seriale
*  \param baudrate data rate dei dati trasmessi su porta seriale
*  \param bytesize dimensione dei pacchetti trasmessi lungo la porta seriale
*  \param parity bit di parit&agrave;
*  \param stopbits indica se si tratta della porta seriale numero zero piuttosto che della numero uno
*  \return Risultato della funzione tcsetattr()
*/
int set_serial(int fd,unsigned long baudrate,unsigned long bytesize,
               unsigned long parity, unsigned long stopbits)
{
    int ret = 0;
    struct termios options;

    bzero(&options, sizeof(options)); 	/* clear struct for new port settings */

    ret = tcflush(fd, TCIOFLUSH);		
    ret = fcntl(fd, F_SETFL, 0);		

    options.c_cflag |= (parity | bytesize | CLOCAL | CREAD | CRTSCTS);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    options.c_oflag &= ~OPOST;
    
    cfsetispeed(&options,baudrate);
    cfsetospeed(&options,baudrate);

    options.c_cc[VTIME]    = 0;     
    options.c_cc[VMIN]     = 1;

    tcflush(fd, TCIOFLUSH);

/*
    The tcsetattr() function sets the parameters associated with the terminal referred to by the open file descriptor 
    fildes (an open file descriptor associated with a terminal) from the termios structure referenced by termios_p as 
    follows:<BR>
         * If optional_actions is TCSANOW, the change will occur immediately.<BR>
         * If optional_actions is TCSADRAIN, the change will occur after all output written to fildes is transmitted. 
            This function should be used when changing parameters that affect output.<BR>
         * If optional_actions is TCSAFLUSH, the change will occur after all output written to fildes is transmitted, 
            and all input so far received but not read will be discarded before the change is made.<BR>
    <BR>
    If the output baud rate stored in the termios structure pointed to by termios_p is the zero baud rate, B0, 
    the modem control lines will no longer be asserted. Normally, this will disconnect the line.
    If the input baud rate stored in the termios structure pointed to by termios_p is 0, the input baud rate 
    given to the hardware will be the same as the output baud rate stored in the termios structure. 
    
    \code
    ret = tcsetattr(fd,TCSANOW,&options);    
    \endcode
*/    
    ret = tcsetattr(fd,TCSANOW,&options);

    return ret;
}



/*! 
\brief Abilita la porta seriale 0/1.

Apre il canale di comunicazione seriale #ttyS0 e #ttyS1, 
setta i parametri della porta seriale selezionata mediante la set_serial() 
ed infine crea i thread associati rispettivamente alle callback ser_loopttyS0() e ser_loopttyS1()
per la gestione delle due porte seriali.

\param value
*/
int enable_port(unsigned short value)
{   
    int status;
    if(value==0)
    {
        if(ttyS0 > 0)
        {
            tcflush(ttyS0, TCIOFLUSH);
            shutdown(ttyS0,2);
            ttyS0 = -1;
        }

        ttyS0 = open(SERIAL_DEV0,O_RDWR | O_NOCTTY | O_NDELAY);
        if (ttyS0 < 0) 
        {
            printf("Error: cannot open %s.\n",SERIAL_DEV0);
            return -1;
        }

        if(set_serial(ttyS0,get_parms("serial_br"),get_parms("serial_db"),
            get_parms("serial_pr"),get_parms("serial_sb")) < 0)
        {
            printf("Error: unable to configure %s.\n",SERIAL_DEV0);
            return -1;
        }

        ioctl(ttyS0,TIOCMGET,&status);
        status &= ~TIOCM_RTS;
        ioctl(ttyS0,TIOCMSET,&status);			// disabling transmission on RS485 (RS485 RTS)
        // ------------------------------------------------------- 
        if(serloopttyS0 > 0) pthread_cancel(serloopttyS0); 

        /*!
        \code
        // se si sta abilitando la porta seriale 0 (value == 0) allora viene creato il thread serloopttyS1
        pthread_create (&serloopttyS0, NULL, ser_loopttyS0, NULL);
        \endcode
        */
        pthread_create (&serloopttyS0, NULL, ser_loopttyS0, NULL);  // creating the reading thread 
    }
    else if(value==1)
    {
        if(ttyS1 > 0)
        {
            tcflush(ttyS1, TCIOFLUSH);
            shutdown(ttyS1,2);
            ttyS1 = -1;
        }

        ttyS1 = open(SERIAL_DEV1,O_RDWR | O_NOCTTY | O_NDELAY); 
        if (ttyS1 < 0) 
        {
            printf("Error: cannot open %s.\n",SERIAL_DEV1);
            return -1;
        }

        if(set_serial(ttyS1,get_parms("serial_sbr"),get_parms("serial_sdb"),
            get_parms("serial_spr"),get_parms("serial_ssb")) < 0)
        {
            printf("Error: unable to configure %s.\n",SERIAL_DEV1);
            return -1;
        }

        ioctl(ttyS1,TIOCMGET,&status);
        status |= TIOCM_RTS;
        ioctl(ttyS1,TIOCMSET,&status);			// enabling transmission on RS485 (RS485 RTS)

        // ------------------------------------------------------- 
        if(serloopttyS1 > 0) pthread_cancel(serloopttyS1); 

        /*!
        \code
        // se si sta abilitando la porta seriale 1 (value == 1) allora viene creato il thread serloopttyS1
        pthread_create (&serloopttyS1, NULL, ser_loopttyS1, NULL);
        \endcode
        */
        pthread_create (&serloopttyS1, NULL, ser_loopttyS1, NULL);  // creating the reading thread

    }
    return 0;

}


void restore_factory_settings(char *buffer, bool bridge)
{
    char command[32];
        
    // 20100524 eVS se sono in widegate, i.e., (total_sys_number>1),
    // e sono il master, i.e., (current_sys_number==1), allora
    // mando un messaggio allo slave affinche anche lui faccia 
    // restore
    if(bridge && current_sys_number<total_sys_number && current_sys_number!=total_sys_number)
    {
        SNP_Send(slave_id,buffer,NULL,0,ttyS1);
        usleep(TIMEOUT_485);
    }
    
    // 20100520 eVS, rimuovo file dei parametri per garantire
    // il restore al prossimo riavvio        
    sprintf(command,"rm -rf %s",pm_filename); // 20100524 eVS moved here from the end
    system(command);  // deleting parameters file
    
    // 20100524 eVS quindi ricarico i parametri di default
    load_default_parms();
    save_current_parms(); // 20100524 eVS added new function
    
    // 20100524 eVS modifico flag count_enabled ???
    pthread_mutex_lock(&mainlock);
    count_enabled=1;
    pthread_mutex_unlock(&mainlock);
    
    // 20100520 eVS, rimuovo file dei parametri per garantire
    // il restore al prossimo riavvio        
    //sprintf(command,"rm -rf %s",pm_filename);
    //system(command);  				// deleting parameters file
}


/*! 
\brief Reply via SNP (Serial Network Protocol).

Once a message is received on the serial port using SNP_Recv() in the 
thread ser_loopttyS0() or ser_loopttyS1(), the message has to be properly 
interpreted in order to give a correct answer to the request. 

This function supposes that the sender has built data by concatenating
a valid command string (including the termination character) followed 
by its arguments. The termination character has to be included in order to
allow string comparison via strcmp() between buffer and the various 
command strings (see serial_port.cpp detailed description for a list of 
commands).

So after reception, the message data is virtually split (see ser_loopttyS0() for 
details) into a command string and the argument of that command. 
After splitting the message, the extracted command and argument are
passed to this function.

This function interprets/parses the command string in order to properly react
both changing the current status of the application parameters using 
write_parms() and save_parms() and sending a proper answer using SNP_Send().

In the case of wide-gate configuration, this function checks if this PCN has 
to forward the received command to the next PCN. The code for this check can be
summarized as follows:

\code
if(total_sys_number>1)
{
    if(current_sys_number<total_sys_number) // if I am a slave in the middle forward message
    {
        SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
    }
    if(current_sys_number==1) //if I am the master answer the requesting application
    {
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
    }
}
else // if I am working alone 
    if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0); // answer the requesting application
\endcode

See comments relative to #current_sys_number and #total_sys_number for further details.

Follows a list of the commands available just taking commented part from the code.
- "timebkg": imposta il valore di temporizzazione per l'aggiornamento periodico dello sfondo ("Timebkg" del win_client)
- "staticth": imposta il valore della soglia statica per l'aggiornamento periodico dello sfondo ("Timebkg" del win_client)
- "version": risponde con la versione dell'imgserver 
- "ker_version": risponde con la versione del kernel 
- "sys_version": risponde con la versione del sistema operativo 
- "fw_version": risponde con la versione del bitstream (FPGA)
- "enable_pc": enables/disables person-counting    
- "reset": reset dei contatori 
- "smode": settaggio modalita' di acquisizione
- "rddelete": deletes records.txt from filesystem
- "sbackI"/"sbackS"/"sbackE": scene background procedure
- "slavereply": this command is used in wideconfiguration by the last 
  slave to confirm the reception of the command "sback*" (where * can
  be I, S, E).
- "restore": restore default parameters
- "gdatetime": gets system date and time
- "sdatetime": sets system date and time
- "sled": sets light intensity
- "input0"/"input1": sets optocoupled input function
- "outtime": optocoupled output test
- "address": used to set the PCN ip address
- "polling": polling tra concentratore e master
- "autoled": enable/disable automatic light intensity
- "auto_gain": enable/disable automatic gain of vref 20130927 eVS added
- "inst_height": to set the real installation height 
- "persdetwidegate": used in widegate by each PCN in the chain to send data about detection 
  and tracking to the last PCN in the chain which will perform the final
  counting
- "counters_wg": used in widegate by the last PCN in the chain to send data about counting
- "wideconfiguration": this command is sent from a client application (rs485_gui or collector) 
  or from a PCN in wideconfiguration to its slave till the last 
  PCN in the chain is reached. The last PCN willconfirm with the
  "end_chain" command which will be passed from a PCN to the previous
  in the chain till the master is reached
- "end_chain": used in widegate: the slave of this PCN has sent the acknowledge
  that the last PCN in the chain of the wideconfiguration has 
  been reached and configured by the "wideconfiguration" command
- "set_sincro_counter": used in widegate to syncronize counting data
- "cond_diff_1p": per settare il valore dell'opzione "difficult 
   condition one person" mediante porta seriale
- "diagnostic_en": abilita e disabilita la diagnostica mediante porta seriale
- "testin": testa i digital input (0 e 1 a seconda del settimo carattere
  della stringa in buffer)
- "updI"/"updF": per il trasferimento dell'update di imgserver e/o bitstream (a seconda 
  del quarto carattere della stringa in buffer)

\param[in] dest destination
\param[in] recipient sender
\param[in] buffer command
\param[in] args argument
\param[in] args_size argument size in bytes
\param[in] ttyS serial port to be used
\return 0 if ok, -1 if error
*/
int SNP_reply(unsigned char dest,unsigned char recipient,char *buffer,char *args,unsigned int args_size, int ttyS)
{
    int ret = -1;
    bool reply;
    
    if(recipient == 0xFF) reply = false;	// broadcast message, do not reply!!
    else reply = true;

    //********************************************************************
    /*! \code
    // inoltro del valore di temporizzazione per l'aggiornamento dello sfondo
    // mediante porta seriale
    if(strcmp(buffer,"timebkg")==0)
    \endcode */
    if(strcmp(buffer,"timebkg")==0) 
    {
        unsigned char value;
        value = get_arg(args,args_size,0,sizeof(value));
        if(write_parms(buffer,value) < 0) return -1;

        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;

        /*! \code
        // se ci sono piu PCN collegati in serie
        // allora nel caso del PCN numero current_sys_number (current_sys_number=2,..,N-1)
        // viene inoltrato il comando al PCN slave (ovvero il numero current_sys_number+1)
        // mediante la funzione SNP_Send, 
        // invece nel caso che il PCN corrente sia il master (current_sys_number=1)
        // piuttosto che un solo PCN (total_sys_number=1)
        // il comando contenuto in buffer viene inviato
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        \endcode
        */

        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        ret=save_parms(buffer,(unsigned short)value);
        send_enable=send_old;    
        return ret;
    }
    //********************************************************************
    /*! \code
    // inoltro del valore della soglia statica mediante porta seriale
    if(strcmp(buffer,"staticth")==0) 
    \endcode */
    if(strcmp(buffer,"staticth")==0) 
    {
        int value;
        value = get_arg(args,args_size,0,sizeof(value));
        if(write_parms(buffer,value) < 0) return -1;

        unsigned char send_old=send_enable;
        int ret;
        send_enable=0;

        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                // il primo PCN della catena e' il master ???
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        ret=save_parms(buffer,(unsigned short)value);
        send_enable=send_old;
        return ret;
    }
    //********************************************************************
    /*! \code
    // inoltro della versione dell'imgserver mediante porta seriale
    if(strcmp(buffer,"version")==0)
    \endcode */
    if(strcmp(buffer,"version")==0)		// returns imgserver version
    {
        if(reply) SNP_Send(dest,buffer,VERSION,strlen(VERSION)+1,ttyS0);
        return 0;
    }
    //********************************************************************
    /*! \code
    // inoltro della versione del kernel mediante porta seriale
    if(strcmp(buffer,"ker_version")==0)
    \endcode */
    if(strcmp(buffer,"ker_version")==0)		// returns APC kernel version
    {
        char ver[32];
        ker_version(ver);
        if(reply) SNP_Send(dest,buffer,ver,strlen(ver)+1,ttyS0);
        return 0;
    }
    //********************************************************************
    /*! \code
    // inoltro della versione del sistema operativo mediante porta seriale
    if(strcmp(buffer,"sys_version")==0)
    \endcode */
    if(strcmp(buffer,"sys_version")==0)		// returns APC OS version
    {
        char ver[32];
        sys_version(ver);
        if(reply) SNP_Send(dest,buffer,ver,strlen(ver)+1,ttyS0);
        return 0;
    }
    //********************************************************************
    /*! \code
    // inoltro della versione del bitstream (FPGA) mediante porta seriale
    if(strcmp(buffer,"fw_version")==0)
    \endcode */
    if(strcmp(buffer,"fw_version")==0)		// returns FPGA bitstream version
    {
        char ver[32];
        sprintf(ver,"%2.1f",fw_version());
        if(reply) SNP_Send(dest,buffer,ver,strlen(ver)+1,ttyS0);
        return 0;
    }
    //********************************************************************
    // enables/disables person-counting    
    if(strcmp(buffer,"enable_pc")==0)		// enables/disables person-counting
    {
        unsigned char value;
        value = get_arg(args,args_size,0,sizeof(value));
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        enable_counting(value); //bisogna lasciare questa per gestire logica porta
        return 0;
    }
    //********************************************************************
    // reset dei contatori 
    if(strcmp(buffer,"reset")==0)			// resets the two counters
    {
        unsigned char send_old=send_enable;
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,NULL,0,ttyS1);
            if(current_sys_number==1 && reply!=0)
            {
                SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        reset_counters(0);

        if (total_sys_number==current_sys_number 
            && total_sys_number>1) // 20110706 eVS added &&
        {
            unsigned long counters[2];
            counters[0] = counter_in;
            counters[1] = counter_out;
            //if(reply) // 20110706 eVS added if
                SNP_Send(dest,"counters_wg",(char *)counters,sizeof(counters),ttyS0); // ???
        }
        send_enable=send_old;
        return 0;
    }
    
    //********************************************************************
    // returns the two counters
    if(strcmp(buffer,"gcounters")==0)		// returns the two counters
    { 
        unsigned long counters[2];
        unsigned char send_old=send_enable;

        pthread_mutex_lock(&mainlock); // 20100424 eVS added
        counters[0] = counter_in;
        counters[1] = counter_out;
        pthread_mutex_unlock(&mainlock);  // 20100424 eVS added

        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                // 20100521 eVS if we are in wideconfiguration ("current_sys_number<total_sys_number" can 
                // only happen in wideconfiguration) then we have to "pass" the message to 
                // its slave
                // ...
                SNP_Send(slave_id,buffer,NULL,0,ttyS1); //bridge
                
                if(current_sys_number==1 && reply!=0) 
                    // ... 
                    // and if this sensor is the master, i.e., current_sys_number==1, and there is no broadcast
                    // activated, i.e., reply == true, it has to reply to the 
                    // collector/rs485_gui
                    SNP_Send(dest,buffer,(char *)counters,sizeof(counters),ttyS0); 
            }
            if(current_sys_number==total_sys_number)
            {
                // 20100521 eVS if this is the last sensor in the chain of a
                // wideconfiguration or if this is a standalone sensor
                // send counter to its master
                SNP_Send(dest,"counters_wg",(char *)counters,sizeof(counters),ttyS0);
            }
        }
        else 
            if(reply) 
                SNP_Send(dest,buffer,(char *)counters,sizeof(counters),ttyS0);
        send_enable=send_old;
        return 0;
    }
    
    //********************************************************************
    /*! \code
    // settaggio modalita' di acquisizione
    if(strcmp(buffer,"smode")==0)
    {
        acq_mode = get_arg(args,args_size,0,sizeof(acq_mode));
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    \endcode */
    if(strcmp(buffer,"smode")==0)		// sets acquisition mode
    {
        pthread_mutex_lock(&acq_mode_lock); // 20100517 eVS
        acq_mode = get_arg(args,args_size,0,sizeof(acq_mode)); // 20120711 eVS, moved inside lock/unlock
        i2cstruct.reg_addr =  MUX_MODE;
        i2cstruct.reg_value = acq_mode & 0x00FF;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        pthread_mutex_unlock(&acq_mode_lock); // 20100517 eVS
        
        return 0;
    }

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
    
    if(strncmp(buffer,"rdsv",4)==0)		// downloads records.txt via serial port
    {
        static int total_size = 0;
        static int num_of_packets = 0;
        static char *buf = NULL, *current_ptr = NULL;
        static int packet_idx = -1;

        int re_init = 0;
        int ret = -1;
               
        //print_log("%s entered\n", buffer);
        
        //////////////////////////////////////////////////////////
        {
            switch (buffer[4]) {
                case 'S': { // start
                    // allocate file buffer
                    if (buf) {
                        delete [] buf;
                        buf = NULL;
                        //print_log("%s old buffer found and de-allocated\n", buffer);
                    }
            

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
        
                    // fill buffer to be sent
                    {
                        unsigned long start_id;
                        int fsize;
                        char filename[256];
                        FILE *recordfd;

                        total_size = fsize = 0;
                        start_id = (record_id+1) > MAX_REC_FILES ? (record_id+1)-MAX_REC_FILES : 0; //first file index
                        for(unsigned int i=start_id;i<(record_id+1);i++)	// computing the total size
                        {
                            sprintf(filename,"%s%d.txt",rd_filename,i);
                            if((recordfd = fopen(filename,"a+")))
                            { 
                                fseek(recordfd,0L,SEEK_END);
                                total_size += ftell(recordfd);
                                fclose(recordfd);
                            }
                        }
        
                        buf = new char [total_size];
                        char *ptr = buf;
                        for(unsigned int i=start_id;i<(record_id+1);i++)		// copying data to a buffer
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
                        //print_log("%s records in files moved in local buf\n", buffer);

                    }

                    pthread_mutex_unlock(&rdlock);

                    current_ptr = buf;
            
                    if (buf) {                               
                        // everything was fine so set the return value as 0
                        // and send the records dimension
                        packet_idx = 0;
                                                
                        num_of_packets = total_size/MAX_CHUNK_LEN;                        
                        if (num_of_packets*MAX_CHUNK_LEN != total_size)
                            num_of_packets++;
                            
                        buffer[4]='D';
                        buffer[5]='\0';
                        SNP_Send(dest, buffer, (char *)&total_size, sizeof(total_size), ttyS0);
                        
                        ret = 0;
                        //print_log("%s Sent records dimension = %d (num_of_packets = %d, packet_idx = %d)\n", buffer, total_size, num_of_packets, packet_idx);
                    }
                    break;
                } 
                
                case 'N': { // next
                
                  int req_packet = get_arg(args,args_size,0,sizeof(req_packet));
                  
                  char *ptr = buf + req_packet*MAX_CHUNK_LEN;
                  int bytes_remaining = total_size-req_packet*MAX_CHUNK_LEN;
                  int packet_size = (bytes_remaining > MAX_CHUNK_LEN) ? MAX_CHUNK_LEN : bytes_remaining;
                  char tmp[64];

                  //print_log("%s Requested packet %d (size %d)\n", buffer, req_packet, packet_size);
                  
                  if (req_packet == num_of_packets) 
                  {
                    buffer[4]='E';
                    buffer[5]='\0';
                    SNP_Send(dest, buffer, (char*)&packet_idx, sizeof(packet_idx),ttyS0);
                    //print_log("%s Sent End (req_packet = %d)\n", buffer, req_packet);
                    ret = 0;
                  } 
                  else if (req_packet == packet_idx || req_packet == packet_idx-1) 
                  {
                    if (packet_idx < num_of_packets)
                    {
                      buffer[5] = '\0';
                      sprintf(tmp, "%s%04d", buffer, req_packet);
                      tmp[9]='\0';
                      SNP_Send(dest, tmp, ptr, packet_size, ttyS0);
                      //print_log("%s Sent packet %d (size %d)\n", tmp, packet_idx, packet_size);

                      if (req_packet == packet_idx)
                        packet_idx++;
                        
                      ret = 0;
                    } 
                  } 
                  break;
                }            
                        
                case 'E': { // problem
                    //print_log("%s End received (packet_idx %d)\n", buffer, packet_idx);

                    re_init = 1;
                    ret = 0;
                    break;
                }
                
                case 'P': { // problem
                    //Send the application an 'A'cknowledgement of the received error notification
                    buffer[4]='A'; 
                    buffer[5]='\0';
                    SNP_Send(dest,buffer,(char *)&packet_idx,sizeof(packet_idx),ttyS0);
                    //print_log("%s Sent acknoledgement (packet_idx %d)\n", buffer, packet_idx);

                    re_init = 1;
                    ret = 0;
                    break;
                }
            }
        }
        
        // reply on the basis of the received packet
        if (ret != 0) {
            //Inform the application that a 'P'roblem occurred at packet expected_packet_idx
            buffer[4]='P'; 
            buffer[5]='\0';
            SNP_Send(dest,buffer,(char *)&packet_idx,sizeof(packet_idx),ttyS0);
            //print_log("%s problem arised (packet_idx %d)\n", buffer, packet_idx);
            re_init = 1;
        } 
                
        if (re_init) 
        {    
            //Re-initialize static variables and performs a check on the file
            if (buf != NULL)            
                free(buf);
                
            buf = NULL;            
            current_ptr = NULL; 
            total_size = -1;
            packet_idx = -1;
            //last_replay[0] = '\0';
            
            //print_log("%s re-initialized! (packet_idx %d)\n", buffer, packet_idx);
        }
                    
        // store the last command before change it for reply purposes
        //strcpy(last_replay, buffer);
        
        return ret;
        //////////////////////////////////////////////////////////
    } // rdsave


    //********************************************************************
    // deletes records.txt from filesystem
    if(strcmp(buffer,"rddelete")==0)		// deletes records.txt from filesystem
    {
        unsigned int i;
        unsigned long start_id;
        char command[256];
        unsigned char send_old=send_enable;
        send_enable=0;

        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,NULL,0,ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        send_enable=send_old;
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
    //********************************************************************
    // 20100526 eVS this command is used in wideconfiguration by the last 
    // slave to confirm the reception of the command "sback*" (where * can
    // be I, S, E.
    if(strcmp(buffer,"slavereply")==0)	// scene background procedure
    {
        if(total_sys_number>1 && current_sys_number==1) //if I'am the master
        {
            flag_serial = get_arg(args,args_size,0,sizeof(flag_serial));
        }
        else
        {
            unsigned char var;
            SNP_Send(get_parms("serial_sid"),buffer,args,args_size,ttyS0);
            var = get_arg(args,args_size,0,sizeof(var));
        }
        return 0;
    }
    //*****************************************************************************
    // scene background procedure
    if(strncasecmp(buffer,"sback",5)==0)	// scene background procedure
    {
        int x,y;
        int a,b;
        unsigned char *bkgvec,*disvec;

        switch(buffer[5])
        {
        case 'I':

            if(total_sys_number>1 && current_sys_number!=total_sys_number) //se non sono l'ultimo
                SNP_Send(get_parms("slave_id"),buffer,NULL,0,ttyS1); // 20100520 eVS ??? why does it take the value from the #parm_values vector and not using the global variable slave_id? Even because the default value in the parm_values is 0x02 whereas the value in slave_id is 0.

            mainloop_enable(0);	// stopping mainloop before saving the background

            i2cstruct.reg_addr =  MUX_MODE;
            i2cstruct.reg_value = MUX_MODE_8_FPN_ODC_MEDIAN_DISP;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

            read(pxa_qcp,Frame,imagesize);	// the first image is dirty

            //memset(svec,0,sizeof(int)*NN); // eVS 20100419
            for(int h=NN-1;h>=0;h--) svec[h]=INITIAL_STD_BKG;
            memset(Bkgvec,0,sizeof(Bkgvec)); //NN); 20100512 eVS
            
            if(total_sys_number>1 && current_sys_number==total_sys_number) //se sono l'ultimo
            {
                flag_serial = 1;
                SNP_Send(dest,"slavereply",(char *)&flag_serial,sizeof(flag_serial),ttyS0);
            }
            flag_serial = 0;
            break;
        case 'S':
            a = 15;
            b = 16;

            if(total_sys_number>1 && current_sys_number!=total_sys_number) //se non sono l'ultimo
                SNP_Send(get_parms("slave_id"),buffer,NULL,0,ttyS1); // 20100520 eVS ??? why does it take the value from the #parm_values vector and not using the global variable slave_id? Even because the default value in the parm_values is 0x02 whereas the value in slave_id is 0.

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
                }
            }
            if(total_sys_number>1 && current_sys_number==total_sys_number) //se sono l'ultimo
            {
                flag_serial++;
                SNP_Send(dest,"slavereply",(char *)&flag_serial,sizeof(flag_serial),ttyS0);
            }
            break;
        case 'E':

            if(total_sys_number>1 && current_sys_number!=total_sys_number) //se non sono l'ultimo
                SNP_Send(get_parms("slave_id"),buffer,NULL,0,ttyS1);  // 20100520 eVS ??? why does it take the value from the #parm_values vector and not using the global variable slave_id? Even because the default value in the parm_values is 0x02 whereas the value in slave_id is 0.

            FILE *out = fopen(bg_filename,"wb");
            if(!out)
                printf("SNP: unable to open file %s\n",bg_filename);
            else
            {
                fwrite(Bkgvec,sizeof(Bkgvec),1,out);
                fwrite(svec,sizeof(svec),1,out);
                fwrite(&vm_img,sizeof(vm_img),1,out); //save the meanvalue of Image in the background file
                vm_bkg=vm_img;
                fclose(out);
            }

            if(total_sys_number>1 && current_sys_number==total_sys_number) //se sono l'ultimo
            {
                flag_serial=1;
                SNP_Send(dest,"slavereply",(char *)&flag_serial,sizeof(flag_serial),ttyS0);
            }

            memcpy(Bkgvectmp,Bkgvec,NN);
            i2cstruct.reg_addr =  MUX_MODE;
            i2cstruct.reg_value = acq_mode & 0x00FF;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);

            mainloop_enable(1);		// restarting the mainloop

            break;
        default: printf("sback not recognized!\n");
        }
        return 0;
    }
    //*******************************************************************
    // restore default parameters
    if(strcmp(buffer,"restore")==0)		// restore default parameters
    {

        pthread_mutex_lock(&mainlock);
        send_enable=0;
        pthread_mutex_unlock(&mainlock);
        
        // 20100524 eVS created a function for "restore" to be used to avoid
        // redundancy
        restore_factory_settings(buffer, true);
        
        /*char command[32];
                        
        if(current_sys_number<total_sys_number && current_sys_number!=total_sys_number)
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
        return 0;
    }
    //********************************************************************
    // gets system date and time
    if(strcmp(buffer,"gdatetime")==0)	// gets system date and time
    { 
        char systime[16];
        char sysdate[16];
        char data[33];
        time_t curtime;
        struct tm *loctime;

        curtime = time (NULL);
        loctime = localtime (&curtime);
        memset(sysdate,0,sizeof(sysdate));
        memset(systime,0,sizeof(systime));
        sprintf(sysdate,"%02d/%02d/%04d",loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year);
        sprintf(systime,"%02d.%02d",loctime->tm_hour,loctime->tm_min);
        sprintf(data,"%s %s",sysdate,systime);

        if(reply) 
            SNP_Send(dest,buffer,(char *)data,sizeof(data),ttyS0);

        return 0;
    }
    //***********************************************************************************************//
    /*! \code
    // sets system date and time
    if(strcmp(buffer,"sdatetime")==0)
    \endcode
    */
    if(strcmp(buffer,"sdatetime")==0)	// sets system date and time
    {
        char command[32];

        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char*)args,args_size,ttyS1);
            }
            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,"sdatetime",NULL,0,ttyS0);
        }
        else 
            if(reply) 
                SNP_Send(dest,buffer,NULL,0,ttyS0);    

        strcpy(command,"date ");
        strcat(command,args);
        system(command);
        system("hwclock -w");

        return 0;
    }
    //***************************************************************************
    /*  if(strcmp(buffer,"dir")==0)		// set count direction
    {
    unsigned char value;
    unsigned char send_old=send_enable;

    value = get_arg(args,args_size,0,sizeof(value));
    send_enable=0;
    if(total_sys_number>1 && current_sys_number<total_sys_number)
        SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
    int  ret;
    ret=write_parms(buffer,(unsigned short)value); 
    if(ret==0)  ret=save_parms(buffer,(unsigned short)value);
    reset_counters(0);
    send_enable=send_old;

    return ret;
    }*/


    //********************************************************************
    // sets light intensity
    if(strcmp(buffer,"sled")==0)		// sets light intensity
    {
        unsigned char value;
        unsigned char send_old=send_enable;

        value = get_arg(args,args_size,0,sizeof(value));
        if (value >= LED_MAX_VALUE) // 20101025 eVS added check
            value = LED_MAX_VALUE;
        if(write_parms(buffer,(unsigned short)value) < 0) return -1;
        send_enable=0;
        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
            }
            if(current_sys_number==1)
            {
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
        }
        else if(reply) 
        {
            SNP_Send(dest,buffer,NULL,0,ttyS0);
        }
        send_enable=send_old;

        return save_parms(buffer,(unsigned short)value);

    }
    //********************************************************************
    // sets optocoupled input function
    if(strncasecmp(buffer,"input",5)==0)	// sets optocoupled input function
    {
        unsigned short value;

        value = get_arg(args,args_size,0,sizeof(value));

        if(total_sys_number>1)
        {
            if(current_sys_number==1)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
                if(write_parms(buffer,value) < 0) return -1;
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
                return save_parms(buffer,value);
            }
            if(current_sys_number<total_sys_number)
            {
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
                if(buffer[5]=='0')
                {
                    if(value==0 || value==2)
                    {
                        if(write_parms(buffer,value) < 0) return -1;
                        return save_parms(buffer,value);
                    }      
                    else return -1;
                }
                return 0;
            }
            else if(current_sys_number==total_sys_number)
            {
                if(buffer[5]=='0' && value!=2) return -1;
                if(write_parms(buffer,value) < 0) return -1;
                return save_parms(buffer,value);
            }
        }
        else 
        {
            if(write_parms(buffer,value) < 0) return -1;
            if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            return save_parms(buffer,value);
        }
    }
    //********************************************************************
    //optocoupled output test
    if(strncasecmp(buffer,"outtime",7)==0)  	//optocoupled output test
    {
        unsigned short value;

        value = get_arg(args,args_size,0,sizeof(value));
        value = (value >> 2) << 2;	// outtime has to be a multiple of 4

        if(total_sys_number>1)
        {
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1)
            {
                if(buffer[7]=='0')
                {
                    if(write_parms(buffer,value) < 0) return -1;
                    save_parms(buffer,value);
                }
                if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            }
            if(current_sys_number==total_sys_number)
            {
                if(buffer[7]=='1')
                {
                    if(write_parms(buffer,value) < 0) return -1;
                    save_parms(buffer,value);
                }
            }
        }
        else 
        {
            if(write_parms(buffer,value) < 0) return -1;
            if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
            return save_parms(buffer,value);
        }
    }
    //********************************************************************
    /*! \code
    // used to set the PCN ip address
    if(strcmp(buffer,"address")==0)
        ...
    \endcode */
    if(strcmp(buffer,"address")==0)  	// sets the system ip address
    { 
        char command[32];

        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)args,args_size,ttyS1);

            if(current_sys_number==1 && reply!=0)  
                SNP_Send(dest,buffer,NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        strcpy(command,"/sbin/netconfig address ");
        strcat(command,args);
        system(command); 

        return 0;
    }
    //********************************************************************
    /*! \code
    // polling tra concentratore e master
    if(strcmp(buffer,"polling")==0)  	
    {
        // viene usato per la comunicazione tra PCN(master) e concentratore
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        // NULL,0 perche' il comando polling non prevede argomenti
        return 0;
    }
    \endcode */
    if(strcmp(buffer,"polling")==0)  	// polling from AVL (software concentratore) to master
    {
        // viene usato per la comunicazione tra PCN(master) e concentratore
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0); // NULL,0 perche' il comando polling non prevede argomenti
        return 0;
    }
    //********************************************************************
    //enable/disable automatic light intensity
    if(strcmp(buffer,"autoled")==0)
    {
        unsigned char value;

        value = get_arg(args,args_size,0,sizeof(value));

        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;

            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,"autoled",NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        if(write_parms(buffer,value) < 0) return -1;
        if(save_parms(buffer,value)<0) return -1;
        if(write_parms("sled",0)<0) return -1; //initial condition in the feedback control
        return save_parms("sled",0);
    }
    //enable/disable automatic gain of vref 20130927 eVS added
    if(strcmp(buffer,"auto_gain")==0)
    {
        unsigned char value;

        value = get_arg(args,args_size,0,sizeof(value));

        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;

            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,"auto_gain",NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        if(write_parms(buffer,value) < 0) return -1;
        
        return save_parms(buffer,value);
       
        
    }
    //********************************************************************
    //used in widegate to set the exact installation height in order 
    //to compute the real x used to avoid multiple counting of a
    //person seen from two adjacent PCN
    if(strcmp(buffer,"inst_height")==0)
    {
        unsigned char value;
        value = get_arg(args,args_size,0,sizeof(value));
        
        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;

            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,buffer,NULL,0,ttyS0);

            send_enable=send_old;
        }
        else 
          if(reply)
             SNP_Send(dest,buffer,NULL,0,ttyS0);
              
        if(write_parms(buffer,value) < 0) return -1;
        return save_parms("inst_height",value);
    }
    //********************************************************************
    
     //used in widegate by each PCN in the chain
    if(strcmp(buffer,"inst_dist")==0)
    { 
        int value;
        value = get_arg(args,args_size,0,sizeof(value));

        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,buffer,NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        if(write_parms(buffer,value) < 0) return -1;
        return save_parms(buffer,value);
    }
    //********************************************************************
    //used in widegate by each PCN in the chain to send data about detection 
    //and tracking to the last PCN in the chain which will perform the final
    //counting
    if(strcmp(buffer,"persdetwidegate")==0)  	// detected person data from master
    {    
        unsigned int temp=54*(current_sys_number-1);

        if(data_wide_gate==NULL)
        {
            return -1;
        }
        if(args_size!=temp)
        {
            return -1;
        }

        // 20100524 eVS received data from the previous slaves
        memcpy(data_wide_gate,args,args_size);
        // 20100524 eVS add the data of this sensor to the global data structure
        memcpy(&data_wide_gate[54*(current_sys_number-1)],persdata,sizeof(persdata));

        // 20100524 eVS if this sensor is not the master, farward data 
        // to its master (next slave or sooner or later the real master)
        if(total_sys_number>1 && current_sys_number<total_sys_number)
        {
            SNP_Send(slave_id,"persdetwidegate",(char *)data_wide_gate,54*current_sys_number*sizeof(unsigned char),ttyS1);
        }

        return 0;
    }
    /********************************************************************/
    //used in widegate by the last PCN in the chain to send data about counting
    //to the master
    if(strcmp(buffer,"counters_wg")==0)		// returns the two counters last slave to master
    {
        unsigned long counter0, counter1;

        if(total_sys_number>1 && current_sys_number!=1)
            // 20100521 eVS if we are in wideconfiguration, i.e., (total_sys_number>1),
            // and this sensor is not the master, i.e., (current_sys_number!=1)
            // then this sensor has to "pass" the acquired data to its master
            SNP_Send(get_parms("serial_sid"),buffer,(char *)args,args_size,ttyS0);

        pthread_mutex_lock(&mainlock); // 20100603 eVS moved here
        {
          flag_wg_count=0;
          flag_wg=1;
          counter0 = get_arg(args,args_size,0,sizeof(counter0));
          counter1 = get_arg(args,args_size,1,sizeof(counter1));
          people[0]=counter0;
          people[1]=counter1;
          //pthread_mutex_lock(&mainlock); //20100603 eVS moved up        
          record_counters(people[people_dir],people[1-people_dir]);
        }  
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    /********************************************************************/
    if(strcmp(buffer,"sdoorstatus")==0)		// set door status
    {
        unsigned char value;
        value = get_arg(args,args_size,0,sizeof(value));

        pthread_mutex_lock(&mainlock);  // 20111011 eVS, added
        
        if(value==0) mem_door=false;
        else mem_door=true;
        
        pthread_mutex_unlock(&mainlock);  // 20111011 eVS, added
        
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        return 0;
    }
    if(strcmp(buffer,"gdoorstatus")==0)		// get door status
    {
        pthread_mutex_lock(&mainlock);  // 20111011 eVS, added 
        unsigned char value;
        if(!mem_door) value=0;
        else value=1;
        
        pthread_mutex_unlock(&mainlock); // 20111011 eVS, added
        
        SNP_Send(dest,buffer,(char *)&value,sizeof(value),ttyS0);
        return 0;
    }
    if(strcmp(buffer,"close_door")==0)		// close door event
    {
        if(current_sys_number<total_sys_number)
        {
            unsigned long counters[2];

            if(current_sys_number!=1) SNP_Send(get_parms("serial_sid"),buffer,(char *)args,args_size,ttyS0);
            counters[0] = get_arg(args,args_size,0,sizeof(counters[0]));
            counters[1] = get_arg(args,args_size,1,sizeof(counters[1]));
            
            pthread_mutex_lock(&mainlock);
            
            people[0]=counters[0];
            people[1]=counters[1];
            ev_door_close_rec=true;  // close door event is finished
            record_counters(people[people_dir],people[1-people_dir]);
            ev_door_close_rec=false;
            ev_door_close=false;
            count_enabled=0;
            
            pthread_mutex_unlock(&mainlock);
        }
        return 0;
    } 
    /********************************************************************/
    if(strcmp(buffer,"door_ok")==0)	
    {
        flag_serial=1;
        return 0;
    }
    //***************************************************************************************
    if(strcmp(buffer,"open_door")==0)
    {
        if(current_sys_number<total_sys_number)
        {
            unsigned long counters[2];
            SNP_Send(slave_id,"door_ok",NULL,0,ttyS1);
            flag_serial=0;
            if(current_sys_number!=1) SNP_Send(get_parms("serial_sid"),buffer,(char *)args,args_size,ttyS0);   

            counters[0] = get_arg(args,args_size,0,sizeof(counters[0]));
            counters[1] = get_arg(args,args_size,1,sizeof(counters[1]));
            
            pthread_mutex_lock(&mainlock); // 20100424 eVS, removed comment
            
            people[0]=counters[0];
            people[1]=counters[1];
            ev_door_open_rec = true;
            record_counters(people[people_dir],people[1-people_dir]);
            ev_door_open=false;
            
            pthread_mutex_unlock(&mainlock); // 20100424 eVS, removed comment
            
            if(current_sys_number!=1)
            {
                int error=0;
                while(flag_serial==0 && error<100)
                {
                    error++;
                    if(error%10==0) SNP_Send(get_parms("serial_sid"),"open_door",(char *)counters,sizeof(counters),ttyS0);
                    usleep(100); //wait 0.1 milliseconds and check another time
                }
            }
        }

        return 0;
    }
    //********************************************************************
    if(strcmp(buffer,"threshold")==0)		// only for slave
    {  
        unsigned char value;
        value = get_arg(args,args_size,0,sizeof(value));
        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,buffer,NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        value = value < MIN_THRESHOLD ? MIN_THRESHOLD : value;
        value = value > MAX_THRESHOLD ? MAX_THRESHOLD : value;

        if(write_parms(buffer,(unsigned short)value) < 0) return -1;
        
        ////////////////////
        // 20091120 eVS        
        //return save_parms(buffer,(unsigned short)value);
        if (save_parms(buffer,(unsigned short)value) < 0) return -1;
        return check_mov_det_parms();
        // 20091120 eVS        
        ////////////////////
    }
    //********************************************************************
    if(strcmp(buffer,"detect_area")==0)  // detect area distance from the sensor
    {
        char passo;
        char stepstring[10];
        unsigned short value;
        value = get_arg(args,args_size,0,sizeof(value));

        if(value == 0)
            sprintf(stepstring,"step225_");
        else if(value == 1)
            sprintf(stepstring,"step240_");
        else return -1;

        if(total_sys_number>1)
        {
            unsigned char send_old=send_enable;
            send_enable=0;
            if(current_sys_number<total_sys_number)
                SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);

            if(current_sys_number==1 && reply!=0)
                SNP_Send(dest,buffer,NULL,0,ttyS0);

            send_enable=send_old;
        }
        else if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);

        for(int i=0;i<NUM_STEP;i++)
        {
            if(i<=9) passo=0x30+i; // ex: i=10 => step='A' => stepA
            else passo=0x37+i;
            stepstring[8]=passo; stepstring[9]='\0';

            unsigned char valore_passo=calib_get_parms(stepstring);
            calib_write_parms(stepstring,valore_passo);
        }

        save_parms("detect_area",value);
        write_parms("detect_area",value);
        return 0;
    }
    /********************************************************************/
    // This command is sent from a client application (rs485_gui or collector) 
    // or from a PCN in wideconfiguration to its slave till the last 
    // PCN in the chain is reached. The last PCN willconfirm with the
    // "end_chain" command which will be passed from a PCN to the previous
    // in the chain till the master is reached
    if(strcmp(buffer,"wideconfiguration")==0)
    {
        // 20100521 eVS ***THIS COMMAND HAS TO BE RECEIVED BY A SLAVE***
        // in the case of wideconfiguration
        
        unsigned char totalnr, index;
        unsigned char param[2];
       
        mainloop_enable(0);	// 20110705 eVS, stopping mainloop
	
        // 20100521 eVS read command parameters (total number of 
        // systems in widegate and the index to be given to this 
        // sensor)
        totalnr = get_arg(args,args_size,0,sizeof(totalnr));
        index = get_arg(args,args_size,1,sizeof(index));
        
        // 20100521 eVS store temporally the passed total_system_number
        // and current_sys_number waiting for the "end_chain" command
        current_sys_number_tmp=index;
        total_sys_number_tmp=totalnr;
                
        // 20100521 eVS created a common function
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
        
        ////////////////////
        // 20091120 eVS        
        check_mov_det_parms();
        // 20091120 eVS        
        ////////////////////
        */
       
        // 20100521 eVS when the master in a wideconfiguration receives
        // the order to disable the wideconfiguration, it sends to the slave
        // via ttyS1 a "wideconfiguration" command with two zeros as parameters.
        if(totalnr==0 && index==0) //disactivating
        {
            if(current_sys_number==total_sys_number)
            {//I'am the last of the chain
            
                unsigned char argomento=0;
                SNP_Send(get_parms("serial_sid"),"end_chain",(char *)&argomento,sizeof(argomento),ttyS0);
                
                mainloop_enable(1);	// 20110803 eVS, enabling again the mainloop
                
                wide_gate_serial_parms_reset();

                if(write_parms("sys_number",(unsigned char)totalnr) < 0)
                    return -1;
                if(write_parms("sys_number_index",(unsigned char)index) < 0)
                    return -1;

                if(save_parms("sys_number",(unsigned char)totalnr)<0)
                    return -1;
                if(save_parms("sys_number_index",(unsigned char)index)<0)
                    return -1;
            }
            else //I am in the middle of the chain
            {
                SNP_Send(slave_id,buffer,(char *)args,args_size,ttyS1);
            }
            //return 0; 20100521 eVS removed this return and added "else"
        } else { // 20100521 eVS added this else in order to have good structured code
            
            //activation
            if(totalnr!=index)
            {
                // 20100521 eVS if I am NOT the last slave in wideconfiguration
                // send the correct parameters (the total number of system and a 
                // new index obtained just adding one to the current indenx) to
                // my slave
                param[0]=totalnr;
                param[1]=index+1;
                SNP_Send(2,buffer,(char *)param,sizeof(param),ttyS1);  
            }
            // 20100521 eVS removed redundancy
            //else if(totalnr==index) //I'm activating the chain and I am the last device
            else //I'm activating the chain and I am the last device
            {
                // 20100521 eVS if I AM the last slave in wideconfiguration
                // I have to advise the master that the chain has been finished
                // just sending the command "end_chain" to my master which
                // will send this command to its master and so on till the 
                // primary master of the wideconfiguration is reached
                unsigned char argomento=1;
                SNP_Send(get_parms("serial_sid"),"end_chain",(char *)&argomento,sizeof(argomento),ttyS0);
                
                // 20100521 eVS before start with the wideconfiguration this 
                // system has to be reset and the total_sys_number (parameter "sys_number"
                // in parm_values) and the current_sys_number (parameter "sys_number_index"
                // in parm_values) have to be properly set with the temporary values 
                // current_sys_number_tmp and total_sys_number_tmp
                count_enabled=0;
                reset_counters(0);
                if(write_parms("sys_number",(unsigned char)current_sys_number_tmp) < 0)
                {
                    count_enabled=1;
                    return -1;
                }
                if(write_parms("sys_number_index",(unsigned char)total_sys_number_tmp) < 0)
                {
                    count_enabled=1;
                    return -1;
                }
                wide_gate_serial_parms_set();

                if(save_parms("sys_number",(unsigned char)current_sys_number_tmp)<0)
                {
                    count_enabled=1;
                    return -1;
                }
                if(save_parms("sys_number_index",(unsigned char)total_sys_number_tmp)<0)
                {
                    count_enabled=1;
                    return -1;
                }
                count_enabled=1;
            }
            //return 0; 20100521 eVS removed and added else
        }
        return 0;
    } 
    //********************************************************************
    // used in widegate: the slave of this PCN has sent the acknowledge
    // that the last PCN in the chain of the wideconfiguration has 
    // been reached and configured by the "wideconfiguration" command
    if(strcmp(buffer,"end_chain")==0)
    {
        // 20100521 eVS the slave of this PCN has sent the acknowledge
        // that the last sensor in the chain of the wideconfiguration has 
        // been reached and configured
        unsigned char test_index, test_total;
        unsigned char value=get_arg(args,args_size,0,sizeof(value));
        
        //pthread_mutex_lock(&mainlock); // 20100521 eVS added
        if(value ==0) {
            // 20100521 eVS if the argument is zero means that the 
            // configuration sent was "disable wideconfiguration"
            // in this case this system was already configured in
            // widegate as so its global variables current_sys_number
            // and total_sys_number have to have been properly set:
            // we use these values for a test
            test_index = current_sys_number;
            test_total = total_sys_number;
        } else {
            // 20100521 eVS otherwise the command was "enable 
            // wideconfiguration". In this case current_sys_number_tmp
            // and total_sys_number_tmp should contain the new values
            // to be set
            test_index = current_sys_number_tmp;
            test_total = total_sys_number_tmp;
        }
        //pthread_mutex_unlock(&mainlock); // 20100521 eVS added
        
        if(test_index==1) //if I am the first
        { 
            // 20100521 eVS if I am the first (master) of the wideconfiguration
            // I have to stop waiting the feedback (see "wideconfiguration" in 
            // commands.cpp) and reset counters. Notice that, to stop waiting 
            // I have to change the value of the "flag_serial" variable to 1.
            flag_serial=1;
            reset_counters(0);
        }
        else if(test_index<test_total) //if I am in the middle
        {
            // 20100521 eVS if I am in the middle I have to sent "end_chain" 
            // command to my master. The goal is to inform the master of the 
            // wideconfiguration that all the chain received the command
            SNP_Send(get_parms("serial_sid"),buffer,(char *)args,args_size,ttyS0);      
        }
        else 
            // 20010521 eVS this case is not valid so return error
            return -1;

        // 20100521 eVS therefore I have to set the configuration
        // on the basis of the activate/disactivate command received
        if(value==1) //activating
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
        else if(value==0) //disactivating
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
        mainloop_enable(1);	// 20110705 eVS restart mainloop
        return 0;
    }

    if(strcmp(buffer,"test_serial_port")==0)
    {
        test_serial=1;
        return 0;
    }

    //**************************************************
    // used in widegate to syncronize counting data
    if(strcmp(buffer,"set_sincro_counter")==0)
    {
        unsigned char value = get_arg(args,args_size,0,sizeof(value));

        count_sincro=value;
        if(current_sys_number<total_sys_number)
        {
            SNP_Send(slave_id,buffer,(char *)&value,sizeof(value),ttyS1);
        }
        return 0;
    }
    //*****************************************************************/
    /*! \code
    // inoltro ad uno slave del valore dell'opzione "difficult condition one person" mediante porta seriale
    if(strcmp(buffer,"cond_diff_1p")==0)
    {
        unsigned char value = get_arg(args,args_size,0,sizeof(value));
        write_parms("cond_diff_1p",(unsigned short)value);
        save_parms("cond_diff_1p",(unsigned short)value);
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        return 0;
    }
    \endcode */
    if(strcmp(buffer,"cond_diff_1p")==0)
    {
        unsigned char value = get_arg(args,args_size,0,sizeof(value));
        write_parms("cond_diff_1p",(unsigned short)value);
        save_parms("cond_diff_1p",(unsigned short)value);
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        return 0;
    }
    //*****************************************************************/
    /*! \code
    // abilita e disabilita la diagnostica mediante porta seriale
    if(strcmp(buffer,"diagnostic_en")==0)
        ...
    \endcode */
    if(strcmp(buffer,"diagnostic_en")==0)
    {
        unsigned char value = get_arg(args,args_size,0,sizeof(value));
        write_parms("diagnostic_en",(unsigned short)value);
        save_parms("diagnostic_en",(unsigned short)value);
        if(reply) SNP_Send(dest,buffer,NULL,0,ttyS0);
        return 0;
    }

    //*****************************************************************/
    if(strcmp(buffer,"pcn1001_status")==0)
    { 
        unsigned char current_error_code; // 20110706 eVS added
        
        pthread_mutex_lock(&acq_mode_lock); // 20101028 eVS

            current_error_code=0; 
        if(diagnostic_en)
            current_error_code=pcn_status; // 20111207 eVS, changed accordingly to the new pcn_status type and meaning
        
        pthread_mutex_unlock(&acq_mode_lock); // 20101028 eVS
        
        if(reply) SNP_Send(dest,buffer,(char *)&current_error_code,sizeof(current_error_code),ttyS0);
        
        return 0;
    }

    //*****************************************************************/
    /*! \code
    // testa i digital input (0 e 1 a seconda del settimo carattere
    // della stringa in buffer)
    if(strcmp(buffer,"testin")==0)
        ...
    \endcode */
    if(strncasecmp(buffer,"testin",6)==0)  	//optocoupled input test
    {
        int ret = 0;
        //char value;
        unsigned char *input_test = NULL;
        //pthread_mutex_lock(&rdlock); // 20100424 eVS, removed comment
        pthread_mutex_lock(&mainlock);
        /*if (strlen(buffer) == 7 && buffer[6]=='0')
            value = get_gpio("GPLR3_096");
        else if (strlen(buffer) == 7 && buffer[6]=='1')
            value = get_gpio("GPLR2_095");
        else
            ret = -1;*/
        // 20120117 eVS, restored old method after modification in input_loop0 e input_loop1
        if(strlen(buffer) == 7) {
            input_test = (atoi(&buffer[6]) == 0) ? &input_test0 : &input_test1;
        } else 
            ret = -1;
        //pthread_mutex_unlock(&rdlock); // 20100424 eVS, removed comment
        pthread_mutex_unlock(&mainlock);
        if (reply && !ret)        
            SNP_Send(dest,buffer,(char *)input_test,sizeof(char),ttyS0);
            //SNP_Send(dest,buffer,(char *)&value,sizeof(value),ttyS0);
        return ret;
    }
    
    /*
    20100121 eVS - now the original approach "testin" works
    
    if(strncasecmp(buffer,"get_gpio",8)==0)   //optocoupled input test 2
    {         
        char value;
        int ret = 0;
 
        pthread_mutex_lock(&mainlock);
        
        if (buffer[8]=='0')
            value = get_gpio("GPLR3_096");
        else if (buffer[8]=='1')
            value = get_gpio("GPLR2_095");
        else
            ret = -1;
            
        pthread_mutex_unlock(&mainlock);

        if(reply && !ret)  
            SNP_Send(dest,buffer,(char *)&value,sizeof(value),ttyS0);

        return ret;
    }*/

    //*****************************************************************/
    /*! \code
    // per il trasferimento dell'update di imgserver e/o bitstream (a seconda 
    // del quarto carattere della stringa in buffer)
    if(strcmp(buffer,"upd")==0)
        ...
    \endcode */
    if(strncasecmp(buffer,"upd",3)==0)	// update system software modules
    {
        static char *file_buffer = NULL, *current_ptr = NULL;
        static int filesize = -1;
        static int expected_packet_idx = -3;
        static char last_replay[20] = "";
        
        int ret = -1;
        int re_init = 0;
        char mycommand[64];
        FILE *fp = NULL;
        
        if (buffer[3] != 'I' && buffer[3] != 'F') {
            expected_packet_idx = UEC_CMD_NOT_EXISTING;
        } /*else {
            if (strlen(last_replay) > 0)
                if (last_replay[3] != buffer[3])
                    expected_packet_idx = UEC_CMD_INCOHERENT;
        }*/
        
        if (expected_packet_idx != UEC_CMD_INCOHERENT && 
            expected_packet_idx != UEC_CMD_NOT_EXISTING) 
        {        
            switch (buffer[4]) {
                case 'S': {
                    // allocate file buffer
                    if (file_buffer) {
                        free(file_buffer);
                        //print_log("%s old buffer found and de-allocated\n", buffer);
                    }
            
                    filesize = get_arg(args, args_size, 0, sizeof(filesize));
                    file_buffer = (char *) malloc(filesize*sizeof(char));
                    current_ptr = file_buffer;
            
                    if (file_buffer) {                               
                        // everything was fine so set the return value as 0
                        // and ask for the first packet
                        expected_packet_idx = 0;                        
                        ret = 0;
                        //print_log("%s file created (size %d)\n", buffer, filesize);
                    } else {
                        expected_packet_idx = UEC_FAILED_MALLOC;
                    }
                    break;
                } 
                case 'N': {
                    if (file_buffer && expected_packet_idx >= 0)
                    {        
                        int received_packet_idx = atoi(&buffer[5]);
                
                        if (expected_packet_idx == received_packet_idx) {               
                            memcpy(current_ptr, args, args_size);
                            current_ptr += args_size;                    
                            ret = 0;                                
                            expected_packet_idx++;                    
                        } else {
                            // check if the application has sent again the alreay received packet
                            if (last_replay[4] == 'N' && expected_packet_idx-1 == received_packet_idx) {
                                ret = 0;
                            } else {
                                if (last_replay[4] != 'N')
                                    expected_packet_idx = UEC_CMD_UNEXPECTED;                                
                                else
                                    expected_packet_idx = UEC_PACKET_UNEXPECTED;
                            }
                        }
                    } else {
                        expected_packet_idx = UEC_CMD_UNEXPECTED;
                    }
                    break;
                } 
                case 'E': {
                    if (file_buffer && (current_ptr-file_buffer)==filesize) {
                        //if packets were all correctly received then...
                    
                        //create the file and move it in the working dir if everything works fine
                        char tmp_filename[50];
                        if (buffer[3] == 'I')
                            sprintf(tmp_filename, "/tmp/imgserver.new");
                        else
                            sprintf(tmp_filename, "/tmp/pcn1001.bin");
                            
                        fp = fopen(tmp_filename,"wb");
                        if (fp) {
                            if (fwrite(file_buffer, filesize, 1, fp)) {
                                //close the file and move it into the working dir
                                fclose(fp);
                                fp = NULL;

                                if (buffer[3] == 'I')
                                    sprintf(mycommand,"chmod 755 %s", tmp_filename);
                                else
                                    sprintf(mycommand,"chmod 644 %s", tmp_filename);
                                system(mycommand);
                                
                                sprintf(mycommand,"mv %s %s", tmp_filename, working_dir); 
                                system(mycommand);
                    
                                //-2 means "end reveived"
                                expected_packet_idx = -2;
                                // re-initialize static variables
                                ret = 0;
                                re_init = 1;
                            } else {
                                expected_packet_idx = UEC_FAILED_FWRITE;
                            }
                        } else {
                            expected_packet_idx = UEC_FAILED_FOPEN;
                        }
                    } else {
                        if (last_replay[4] == 'E') {
                            // 'E'nd was already sent or an error occured
                            ret = 0;
                            expected_packet_idx = -2;
                        } else {
                            expected_packet_idx = UEC_CMD_UNEXPECTED;
                        }
                    }
                    break;
                }
                case 'P': {
                    //Send the application an 'A'cknowledgement of the received error notification
                    expected_packet_idx = -1;
                    re_init = 1;
                    ret = 0;
                    break;
                }
            }
        }
         
        // reply on the basis of the received packet
        if (ret != 0) {
            //Inform the application that a 'P'roblem occurred at packet expected_packet_idx
            buffer[4]='P'; 
            buffer[5]='\0';
            SNP_Send(dest,buffer,(char *)&expected_packet_idx,sizeof(expected_packet_idx),ttyS0);
            //print_log("%s %d: problem arised\n", buffer, expected_packet_idx);
            re_init = 1;
        } else {
            if (expected_packet_idx == -1) {
                //Inform the application that a the sent 'P'roblem notification was received
                buffer[4]='A'; 
                buffer[5]='\0';
                SNP_Send(dest,buffer,(char *)&expected_packet_idx,sizeof(expected_packet_idx),ttyS0);
                //print_log("%s %d: send acknowledge\n", buffer, expected_packet_idx);
            } else if (expected_packet_idx == -2) { //num_of_packets) {
                //Inform the application that the last packet was correcly received
                buffer[4]='E'; 
                buffer[5]='\0';
                SNP_Send(dest,buffer,(char *)&expected_packet_idx,sizeof(expected_packet_idx),ttyS0);
                //print_log("%s %d: send end\n", buffer, expected_packet_idx);
            } else {
                //Ask next packet 
                buffer[4]='N';
                buffer[5]='\0';
                SNP_Send(dest,buffer,(char *)&expected_packet_idx,sizeof(expected_packet_idx),ttyS0);
                //print_log("%s %d: ask next\n", buffer, expected_packet_idx);
            }
        }
        
        if (re_init) 
        {    
            //Re-initialize static variables and performs a check on the file
            if (fp != NULL)
                fclose(fp);
            if (file_buffer != NULL)
                free(file_buffer);
                
            file_buffer = NULL;            
            current_ptr = NULL; 
            filesize = -1;
            expected_packet_idx = -3;
            last_replay[0] = '\0';
            
            //print_log("%s %d: re-initialized!\n", buffer, expected_packet_idx);
        }
                    
        // store the last command before change it for reply purposes
        strcpy(last_replay, buffer);
        
        return ret;
    }
        
    return ret;
}




/*! 
\brief Invio di un messaggio di tipo SNP (Serial Network Protocol) mediante porta seriale.

  Creazione ed invio ad un PCN slave di un messaggio di tipo SNP sulla porta seriale mediante la funzione write_port(), 
  la quale a sua volta si appoggia sulla funzione di scrittura non formattata a basso livello (write) di un certo numero di bytes.

  \param dest destinatario
  \param command comando
  \param arg dati da trasmettere
  \param arg_size numero di byte dell'argomento
  \param ttyS file descriptor della porta seriale su cui trasmettere
  \return numero byte letti o un valore negativo se ci sono stati errori
*/
int SNP_Send(unsigned char dest,char *command,char *arg,int arg_size,int ttyS)
{
    int len;
    int ret;
    int status;
    short cmd_len;
    short total_len;
    char *tmp_buf;
    char buffer[MAX_PACKET_LEN];
    unsigned int tmout;

    memset(buffer,0,sizeof(buffer)); //MAX_PACKET_LEN); 20100512 eVS
    cmd_len = strlen(command)+1;
    total_len = arg_size+cmd_len;

    if(total_len > MAX_DATA_LEN) 
        return -1;
        
    tmp_buf = new char [total_len];
    strcpy(tmp_buf,command);
    memcpy(tmp_buf+cmd_len,arg,arg_size);

    /*!
    \code
    // mediante la funzione BuildSNPCommand viene costruito 
    // il pacchetto da inviare ad un'altro dispositivo collegato in serie
    if(ttyS==ttyS0)
    {
        if(!strcmp(command,"polling")) len = BuildSNPCommand(buffer,get_parms("serial_id"),dest,0,0,0,NULL);
        else len = BuildSNPCommand(buffer,get_parms("serial_id"),dest,1,1,total_len,tmp_buf);
    }
    else
    {
        if(!strcmp(command,"polling")) len = BuildSNPCommand(buffer,get_parms("serial_sid"),dest,0,0,0,NULL);
        else len = BuildSNPCommand(buffer,get_parms("serial_sid"),dest,1,1,total_len,tmp_buf);
    }
    \endcode
    */

    if(ttyS==ttyS0)
    {
        if(!strcmp(command,"polling")) len = BuildSNPCommand(buffer,get_parms("serial_id"),dest,0,0,0,NULL);
        else len = BuildSNPCommand(buffer,get_parms("serial_id"),dest,1,1,total_len,tmp_buf);
    }
    else
    {
        if(!strcmp(command,"polling")) len = BuildSNPCommand(buffer,get_parms("serial_sid"),dest,0,0,0,NULL);
        else len = BuildSNPCommand(buffer,get_parms("serial_sid"),dest,1,1,total_len,tmp_buf);
    }

    ioctl(ttyS,TIOCMGET,&status);
    status |= TIOCM_RTS;
    ioctl(ttyS,TIOCMSET,&status);			// enabling transmission on RS485 (RS485 RTS)

    /*!
    \code
    // scrittura del buffer su porta seriale
    ret = write_port(ttyS,buffer,len);
    \endcode
    */
    ret = write_port(ttyS,buffer,len);

    if (strncmp(command, "rdsv", 4) == 0) // 20110922 eVS added to avoid timeout when records are transferred
      tmout = 5000;
    else
      tmout = 1000;

    if(ttyS==ttyS0)  while (--tmout && !get_gpio("FFLSR_TEMT"));		// waiting for the FFUART transmitter empty
    else if(ttyS==ttyS1)  while (--tmout && !get_gpio("BTLSR_TEMT"));		// waiting for the BTUART transmitter empty

    ioctl(ttyS,TIOCMGET,&status);
    status &= ~TIOCM_RTS;
    ioctl(ttyS,TIOCMSET,&status);			// disabling transmission on RS485 (RS485 RTS)

    delete [] tmp_buf;
    return ret;
}


/*! 
\brief Receive an SNP (Serial Network Protocol) messagge from a serial port.

  From the serial port specified by tty, a message is received and used to
  fill the three output parameters. This function is based on read_port()
  to read byte from the serial port.

  \param[out] buf received message (similar to the one obtained with BuildSNPCommand() even if the preamble is compressed in only one 0xFF instead of five)
  \param[out] sender 
  \param[out] recipient
  \param[in] ttyS specifies serial port
  \return Size of the data part in the SNP message or -1 if an error occurred.
*/
int SNP_Recv(char *buf,unsigned char *sender,unsigned char *recipient,int ttyS)
{
    char tmp;
    char tail[3];
    short crc,crctest;
    unsigned short datalen;

    // discard preamble made of 0xFF and the SOH corresponding to 0x01
    int count = 0; // 20111212 eVS, added to manage a proper searching for the preamble
    bool soh_found = false; // 20111212 eVS, added to manage a proper searching for the preamble
    
    buf[0] = buf[1] = 0;
    do
    {
        if (soh_found) // 20111212 eVS, added to manage a proper searching for the preamble
            count = 0; // 20111212 eVS, if soh_found but the program is still in the loop then it means that we have to restart looking for the preamble
          
        if(read_port(ttyS,&tmp,1) < 0) return -1;
        switch(tmp)
        {

        case 0xFF: 
            buf[0] = tmp;
            soh_found = false; // 20111212 eVS, added to manage a proper searching for the preamble
            count++; // 20111212 eVS, added to manage a proper searching for the preamble
            break;
        case 0x01: 
            buf[1] = tmp; 
            soh_found = true; // 20111212 eVS, added to manage a proper searching for the preamble
            break;
        default: 
            count = 0; // 20111212 eVS, added to manage a proper searching for the preamble
            soh_found = false; // 20111212 eVS, added to manage a proper searching for the preamble
            buf[0] = buf[1] = 0;
        }
    }
    //while(buf[0] == 0 || buf[1] == 0);
    // 20111212 eVS, modified to manage a proper searching for the preamble
    while(buf[0] != 0xFF || buf[1] != 0x01 || !soh_found || count<5); 

    // read other 6 bytes corresponding respectively to:
    // sender, recipient, 
    // tot. number of pockets, pocket no., 
    // data length LSB, data length HSB
    if(read_port(ttyS,buf+2,6) < 0) {
        //print_log("??? received something wrong\n");
        return -1;
    }
    // check if I am the recipient
    int my_id;
    if (ttyS==ttyS1) 
        my_id=get_parms("serial_sid");
    else 
      /*if (ttyS==ttyS1) 
        my_id=get_parms("serial_sid");
      else */
        my_id=get_parms("serial_id");

    if((buf[3] != my_id) && (buf[3] != 0xFF))  {
        //print_log("??? received something not for me\n");
        return -1; //it's not my address
    }


    // assign outputs
    *sender = buf[2];
    *recipient = buf[3];
    
    // read data length and check if it is less than or equal to the limit
    datalen = buf[6] | (buf[7] <<8);
    if(datalen > MAX_DATA_LEN) {
        //print_log("??? received something too long\n");
        return -1;
    }

    // read data from the tty
    if(read_port(ttyS,buf+8,datalen) < 0) 
        return -1;
    
    // read the last part of the message
    if(read_port(ttyS,tail,sizeof(tail)) < 0) 
        return -1;

    // read crc
    crc = tail[0] | (tail[1] << 8);
    
    // crc check
    CalcCrc16Block(&buf[2],6+datalen,&crctest);
    if(crc != crctest) {
        //print_log("??? received something crc failed\n");
        return -1;  // crc test failed!
    }

    return datalen;

}


/*! 
\brief Build an SNP (Serial Network Protocol) message.

  An SNP message has the following form:
  - (byte idx = meaning)
  - 0..4 = preamble (five bytes equals to 0xFF)
  - 5 = SOH (one byte equal to 0x01)
  - 6 = sender (one byte)
  - 7 = recipient (one byte)
  - 8 = total number of packets (one byte)
  - 9 = number of this packet, i.e. it is a value in the range 1..total number of packets (one byte)
  - 10, 11 = data length (two bytes: the LSB - Least Significative Byte - comes first)
  - 12..(12+data length-1) = dati (tanti byte quanti specificati dalla lunghezza dati)
  - (12+data length) and the next byte = CRC (two byte: the LSB - Least Significative Byte - comes first)
  - (12+data length+2) = postamble (one byte equals to 0xFF)
  
  Notice that data length has to be less than MAX_PACKET_LEN. 

  \param[out] buf The resulting SNP message to be sent via serial port.
  \param[in] src sender
  \param[in] dest recipient
  \param[in] tpn total number of packets
  \param[in] pn packet number
  \param[in] datalen message length
  \param[in] data message
  \return Number of bytes of the SNP message.
*/
int BuildSNPCommand(char *buf,unsigned char src,unsigned char dest,char tpn,char pn,short datalen,char *data)
{
    short crc;
    int index = 0;
    int data_start;

    buf[index++] = 0xFF;                //Preample
    buf[index++] = 0xFF;                //Preample
    buf[index++] = 0xFF;                //Preample
    buf[index++] = 0xFF;                //Preample
    buf[index++] = 0xFF;                //Preample

    buf[index++] = 0x01;                //SOH
    data_start = index;

    buf[index++] = src;                 //source address
    buf[index++] = dest;                //destination address
    buf[index++] = tpn;                 //Total Packet Number
    buf[index++] = pn;                  //Number of this packet
    buf[index++] = datalen & 0x00FF;    //Data Length LSB
    buf[index++] = datalen >> 8;        //Data Length MSB

    if(data) memcpy(&buf[index],data,datalen); //Data copied in buf

    index += datalen;
    CalcCrc16Block(&buf[data_start],6+datalen,&crc);
    
    buf[index++] = crc & 0x00FF;        //CRC LSB
    buf[index++] = crc >> 8;            //CRC MSB
    buf[index++] = 0xFF;                //Postamble

    return index;
}



/*! 
*  \brief Calcolo del CRC.

Cyclic Redundancy Check (ovvero controllo a ridondanza ciclica): &egrave; 
un metodo per il calcolo di somme di controllo. 
Prevede la generazione di una stringa di bit di controllo che viene 
normalmente trasmessa assieme ai dati e il calcolo &egrave; basato sull'aritmetica modulare.

*  \param pBlock pointer to start of block
*  \param Number number of bytes i block
*  \param pCrc will be updated with CRC16
*/
void CalcCrc16Block(char *pBlock,		// Pointer to start of block
                    unsigned short Number, 		// Number of bytes i block
                    short *pCrc)		// Will be updated with CRC16
{
    *pCrc = -1;
    while (Number)
    {
        *pCrc = CrcTable[((*pCrc >> (CRC_WIDTH - BYTE_WIDTH)) ^ *pBlock++) &
            CRC_MASK] ^ (*pCrc << BYTE_WIDTH);
        Number--;
    }
}




/*! 
  \brief Retrieve a specific argument from an arguments array.
  
  This function is used by SNP_reply() in order to get a specific
  argument from the input vector of all the arguments.
  
  Since command have different arguments that can be of different 
  type (int, char, short, ...), this function allows to retrieve
  an argument on the basis of its type and position inside the
  argument buffer.
  
  \param[in] arg argument
  \param[in] arg_size number of bytes of the argument
  \param[in] pos position index of the desired argument inside arg
  \param[in] size size of each element in arg
  \return the extracted value from arg
*/
unsigned long get_arg(char *arg,int arg_size,int pos,int size)
{
    int i;
    char *ptr;
    unsigned long ret;

    if((pos+1)*size > arg_size) return 0;
    else ptr = arg + pos*size;

    ret = 0; 
    for(i=0;i<size;i++)
    { 
        ret |= ptr[i] << i*8;
    }
    return ret;
}



/*!
*  \brief Lettura del buffer dalla porta seriale.

La funzione &egrave; basata sulla lettura non formatta a basso livello (read) 
per leggere una serie di bytes sulla porta seriale associata al file descriptor fd.

*  \param fd file descriptor
*  \param buf data buffer
*  \param len buffer size
*  \return number of bytes read
*/
int read_port(int fd,char *buf,int len)
{
    int sum;
    char *buffer;
    int bytesleft;
    int bytes;

/*! \code
    while(sum < len)
    {
        bytes = read(fd,buffer+sum,bytesleft);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
    } 
\endcode */

    sum = 0;
    buffer = (char *)buf;
    bytesleft = len;
    while(sum < len)
    {
        bytes = read(fd,buffer+sum,bytesleft);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
    } 
    return sum;
}



/*!
*  \brief Scrittura del buffer sulla porta seriale.

La funzione &egrave; basata sulla scrittura non formatta a basso livello (write) 
per scrivere una serie di bytes sulla porta seriale associata al file descriptor fd.

*  \param fd file descriptor
*  \param buf data buffer
*  \param len buffer size
*  \return numero di bytes scritti
*/
int write_port(int fd,char *buf,int len)
{
    int sum;
    char *buffer;
    int bytesleft;
    int bytes;

/*! \code
    while(sum < len)
    {
        bytes = write(fd,buffer+sum,bytesleft);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
    }  
\endcode */

    sum = 0;
    buffer = (char *)buf;
    bytesleft = len;
    while(sum < len)
    {
        bytes = write(fd,buffer+sum,bytesleft);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
    }  
    return sum;
}



// 20100521 eVS aggiornati commenti
/*!
\brief Set dei parametri della porta seriale nel caso wide-gate.

In base al PCN attualmente selezionato, la funzione setta i parametri
della porta seriale. Usata per i comandi relativi alla configurazione
widegate in serial.cpp (che sono "wideconfiguraiton" e "end_chain")
e in commands.cpp (cioe' il "wideconfiguraiton" spedito da 
win_client).
*/
void wide_gate_serial_parms_set()
{
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
    if(current_sys_number_tmp!=1)
    {
        write_parms("input0",3);
        save_parms("input0",3);
    }
    else if (current_sys_number_tmp==1)
    {
        write_parms("input0",0);
        save_parms("input0",0);
    }
    
    write_parms("input1",3);
    save_parms("input1",3);

    write_parms("outtime0",4); //disable FPGA in optoO
    save_parms("outtime0",4);
    write_parms("outtime1",4); //disable FPGA in optoO
    save_parms("outtime1",4);

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
        write_parms("input0",3);
        save_parms("input0",3);
        initpeople(0,0,total_sys_number,num_pers);
    }

    if(current_sys_number_tmp==1)
    {
        write_parms("outtime0",200);
        save_parms("outtime0",200);
        write_parms("input1",3);
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
        value=get_parms("auto_gain");
        SNP_Send(slave_id,"auto_gain",(char *)&value,sizeof(value),ttyS1);        
    }

    if(current_sys_number_tmp==1) send_enable=1;
    else send_enable=0;

    pthread_mutex_lock(&mainlock);
    framecounter=0;
    pthread_mutex_unlock(&mainlock);

    //if(send_enable==0)   count_debug=0; 20100520 commented

    //return;
}



/*!
\brief Reset dei parametri nel caso wide-gate.

In base al PCN attualmente selezionato, la funzione resetta i parametri della porta seriale
e inizializza le liste di PERSONE inhi e inlo.
*/
void wide_gate_serial_parms_reset()
{
    send_enable=0; // ???
    //if(send_enable==0)   count_debug=0; 20100520 commented

    if(current_sys_number!=total_sys_number)
    {
        save_parms("serial_sbr",SERIAL_SBR);
        write_parms("serial_sbr",SERIAL_SBR);

        save_parms("serial_sdb",SERIAL_SDB);
        write_parms("serial_sdb",SERIAL_SDB);

        save_parms("serial_spr",SERIAL_SPR);
        write_parms("serial_spr",SERIAL_SPR);

        save_parms("serial_ssb",SERIAL_SSB);
        write_parms("serial_ssb",SERIAL_SSB);
    }

    if(current_sys_number!=1)
    {
        save_parms("serial_br",SERIAL_BR);
        write_parms("serial_br",SERIAL_BR);

        save_parms("serial_db",SERIAL_DB);
        write_parms("serial_db",SERIAL_DB);

        save_parms("serial_pr",SERIAL_PR);
        write_parms("serial_pr",SERIAL_PR);

        save_parms("serial_sb",SERIAL_SB);
        write_parms("serial_sb",SERIAL_SB);
    }

    write_parms("input0",INPUT0); //optoInput=do nothing
    write_parms("input1",INPUT1);
    save_parms("input0",INPUT0);
    save_parms("input1",INPUT1);

    write_parms("outtime0",OUTTIME0); //enable FPGA in optoO
    save_parms("outtime0",OUTTIME0);
    write_parms("outtime1",OUTTIME1);
    save_parms("outtime1",OUTTIME1);

    initpeople(0,0,total_sys_number,num_pers);
    
    return;
}

