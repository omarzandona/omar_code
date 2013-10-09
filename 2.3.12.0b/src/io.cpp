/*!
\file io.cpp
\brief Lettura e scrittura dei parametri e per la configurazione dei vari dispositivi.
\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdarg.h> //for va_list in print_log

#include "directives.h"
#include "default_parms.h"
#include "directives.h"
#include "peopledetection.h"


// 20100702 eVS door_in e door_out are used only here in this file, so they were moved here
unsigned long door_in;  //!< Used to locally store the number of incoming people.
unsigned long door_out; //!< Used to locally store the number of outgoing people.

extern unsigned char Bkgvec[NN];
extern int svec[NN];
extern unsigned char vm_bkg;
extern int num_pers;
extern unsigned char total_sys_number;
extern bool ev_door_open;
extern bool ev_door_close;
extern bool ev_door_open_rec;
extern bool ev_door_close_rec;
extern bool mem_door;
extern unsigned char frame_cnt_door;
extern unsigned char frame_fermo;
extern unsigned char send_enable;
extern unsigned short parm_values[256];
extern unsigned short default_values[256];
extern char parm_names[256][PARM_STR_LEN];
extern char records[MAX_RECORDS][128];
extern unsigned long records_idx;
extern unsigned long counter_in;
extern unsigned long counter_out;
extern int record_enabled;
extern unsigned char total_sys_number;
extern unsigned char current_sys_number;
extern int count_enabled;
extern int inst_height;
extern int inst_dist;
extern unsigned char handle_oor;
extern unsigned char people_dir;
extern unsigned char limitSx; 
extern unsigned char limitDx;
extern unsigned char limitSx_riga_start;
extern unsigned char limitDx_riga_start;
extern unsigned char limitSx_riga_end;
extern unsigned char limitDx_riga_end;
extern unsigned char limit_line_Up;
extern unsigned char limit_line_Down;
extern unsigned long people[2];

extern void print_log(const char *format, ...);

int load_default_parms(void);
unsigned short get_parms(char *name);
int set_parms(char *name,unsigned short value);
int write_parms(char *name,unsigned short value);

#ifndef PCN_VERSION
void pthread_mutex_lock(void*) {};
void pthread_mutex_unlock(void*) {};
void* mainlock = NULL;
void* rdlock = NULL;
#endif


/*! 
\brief Caricamento parametri.

Caricamento dei valori di default dei parametri dalla variabile #default_values, 
impostare le variabili globali e scrivere i parametri sui vari dispositivi con la funzione
write_parms(), caricamento dei contatori (#counter_in e #counter_out) da file ASCII e 
abilitazione delle porte seriali mediante la enable_port().
*/
#ifdef PCN_VERSION
void load_parms(void)
#else
void load_parms(char* pm_filename)
#endif
{
    FILE *parms;
    char name[PARM_STR_LEN];
    char svalue[PARM_STR_LEN];
    unsigned short value;
    int ret;

    load_default_parms();

#ifdef PCN_VERSION
    load_counters();					//loading input/output counters
#endif
    
    if((parms = fopen(pm_filename,"r")) == NULL) return; 
    else
    {
        do
        {
            ret = fscanf(parms,"%s ",name);
            if((get_parms(name) != 0xFFFF) && ret != EOF)	// checking if the parameter is valid
            {
                fscanf(parms,"%s",svalue);
                value = strtol(svalue,NULL,16);
                
                set_parms(name,value);				// saving parameters in memory
                write_parms(name,value);			// writing parameters on devices
            }
        }
        while(ret != EOF); 

        // 20130308 eVS, moved here to be independent on the parameter loading order
        if(current_sys_number==1 && total_sys_number>1)
        {
            send_enable=1;  // load_parms: in widegate il master usa send_enable per regolare l'accesso alla seriale (tutti gli slave hanno send_enable a zero)
        }
    }
    
#ifdef PCN_VERSION
    enable_port(1);	
    enable_port(0);	
#endif
    
    fclose(parms);
}


/*! 
\brief Caricamento contatori da file ascii (#cr_filename).
*/
#ifdef PCN_VERSION
void load_counters(void)
{
    FILE *counterfd;
    char name[16];

    if((counterfd = fopen(cr_filename,"r")) == NULL) return; 
    else
    {
        fscanf(counterfd,"%s%ld",name,&counter_in);
        fscanf(counterfd,"%s%ld",name,&counter_out);
    }
    fclose(counterfd);
}
#endif


/*! 
\brief Inizializzazione dei parametri con quelli di default in #default_values.

Caricamento dei parametri di default da #default_values dentro la variabile #parm_values 
e impostazione dei dispositivi mediante la funzione write_parms().
*/
int load_default_parms(void)
{
    int i;
    int ret = 0;

    send_enable=0;

    /*! \code
    memcpy(parm_values,default_values,sizeof(default_values));
    \endcode */
      
    // 20100525 eVS lock/unlock 
    //pthread_mutex_lock(&mainlock);    
    memcpy(parm_values,default_values,sizeof(default_values));
    //pthread_mutex_unlock(&mainlock);
        
    for(i=0;(strlen(parm_names[i]) && (i < 256));i++)
    {
        printf("load default %s %d\n",parm_names[i],parm_values[i]);
        if(write_parms(parm_names[i],parm_values[i]) < 0)
        {
            printf("Error loading default parameter: %s\n",parm_names[i]);
            ret = -1;
        }
    }
    return ret;
}


/*! 
\brief Salavataggio di tutti i parametri correnti su file.

Scandisce tutti i parametri e usa la save_parms() per ognuno al fine di 
aggiungere ogni singolo parametro al file.
*/
#ifdef PCN_VERSION
int save_current_parms(void) // 20100524 eVS added new function
{
    int i;
    int ret = 0;


    for(i=0;(strlen(parm_names[i]) && (i < 256));i++)
    {
        if(save_parms(parm_names[i],parm_values[i]) < 0)
        {
            printf("Error saving current parameter: %s\n",parm_names[i]);
            ret = -1;
        }
    }
    return ret;
}
#endif


/*! 
\brief Salvataggio di uno specifico parametro su file ascii (#pm_filename) e nel vettore #parm_values.

I parametri da salvare sono quelli in #parm_values i cui nomi sono in #parm_names.

\param name nome del parametro
\param value valore del parametro
\return La funzione ritorna zero se il salvataggio &egrave; andato a buon fine.
*/
#ifdef PCN_VERSION
int save_parms(char *name,unsigned short value)
{
    FILE *out;
    char sname[PARM_STR_LEN];
    char string[PARM_STR_LEN];
    char command[32];
    char ch;
    int i;
    long pos,len;
    unsigned char send_old=send_enable;
    send_enable=0;

    if(set_parms(name,value) < 0) return -1;	//wrong parameter

    sprintf(command,"/bin/touch %s",pm_filename);	// creating the parameters file if it does not esist
   
    system(command);

    out = fopen(pm_filename,"r+");
    
    if(!out) 
        {
            printf("Unable to open the file: %s\n",pm_filename); 
            send_enable=send_old; // 20111104 eVS added
            return -1;
        }
    else
    {
        while(!feof(out))
        { 
            pos = ftell(out);
            fscanf(out,"%s ",sname);

            if(!strcmp(name,sname))
            {
                // if the parameter's name is found then the value is modified according to the input
                // and then function quits
                fseek(out,pos,SEEK_SET);
                
                sprintf(string,"%s 0x%x",sname,value);
                len = strlen(string);
                
                for(i=len;i<PARM_STR_LEN-1;i++) 
                    string[i] = ' ';  //deleting unused chars
                    
                string[PARM_STR_LEN-1] = '\0';
                fputs(string,out);
                fputc('\n',out);
                fclose(out);
                
                return 0;  // quit
            }
            do ch = fgetc(out);			//going to the end of the line
            while(ch != '\n' && !feof(out)) ;
        }
    }   

    // if the parameter's name is not found in the file then it is added at 
    // the end of the file with the value in input
    sprintf(string,"%s 0x%x",name,value);
    len = strlen(string);
    for(i=len;i<PARM_STR_LEN-1;i++) 
        string[i] = ' ';	//deleting unused chars
    string[PARM_STR_LEN-1] = '\0';
    fputs(string,out);
    fputc('\n',out);

    fclose(out);
    send_enable=send_old;

    return 0;
}
#endif


/*! 
\brief Lettura del valore di uno specifico parametro da #parm_values.

Scandisce #parm_names alla ricerca di name, arrivato all'elemento i-esimo che 
corrispondente a name, restituisce l'elemanto i-esimo di #parm_values.

Legge il valore #parm_values[i] del parametro specificato nel vettore #parm_names.
Per la mappatura tra elementi del vettore e stringhe si veda la tabella \ref tabella_parms.

\param name nome del parametro di cui si vuol leggere il valore
\return valore del parametro
*/
unsigned short get_parms(char *name)
{
    int i;

    for(i=0;(strlen(parm_names[i]) && (i < 256));i++)
    {
        if(!strcmp(parm_names[i],name))
            return parm_values[i];
    }
    return 0xFFFF;
}


/*! 
\brief Settaggio di uno specifico parametro in memoria.

Scrive il valore "value" in #parm_values[i], se il nome del parametro "name" corrisponde a #parm_names[i].

\param name nome del parametro da settare
\param value valore del parametro da settare
\return 0 or -1
*/
int set_parms(char *name,unsigned short value)
{
    int i;

    for(i=0;(parm_names[i] && (i < 256));i++)
    {
        if(!strcmp(parm_names[i],name))
        {
            parm_values[i] = value;
            return 0;
        }
    }
    return -1;
}


/*! 
\brief Modifica lo stato del sistema (memoria e/o dispositivi) in base al nuovo valore "value" da dare
al parametro corrispondente a "name".

Usa set_parms() per cambiare lo stato in memoria della variabile #parm_values e, a seconda del
parametro, va anche ad agire su specifiche variabili globali e/o su specifici dispositivi 
per impostarli in modo coerente ai corrispondenti parametri in #parm_values.

Si noti che molti parametri non hanno nessuna relazione con dei dispositivi hardware ma
sono solo in relazione a delle variabili in memoria che devono essere mantenute coerenti con 
i nuovi valori dei parametri.

\param name
\param value
\return 0 or -1
*/
int write_parms(char *name,unsigned short value)
{
#ifdef debug_
    printf("Write parms %s=%d\n",name,value);
#endif

#ifdef PCN_VERSION
    if(strncasecmp(name,"outtime",7)==0)  //uptime 0 and 1
    {
        //since fpga computes outtime as outtime = ((value/4)+1)*4
        //instead of outtime = ((value+1)/4)*4 it is necessary to set
        //outtime as "value-4". Otherwise value, let to say, = 8 will
        //produce a GPO open time of 12 ms. This is why the minimum
        //time is 8 ms

        unsigned char output;
        int *pcout;

        if(strlen(name) == 8) output = atoi(&name[7]);
        else return -1;

        if(value<4) value=4; //set FPGAouttime=0 
        if(value>=1024) return -1;

        if(output == 0) set_parms("outtime0",value);
        else set_parms("outtime1",value);

        pcout = (output == 0) ? &pcout0 : &pcout1;
        ioctl(*pcout, PCIOCSINT, value-4); 
        if(total_sys_number<2) ioctl(*pcout,PCIOCSOUT);

        return 0;
    }
    if(strncasecmp(name,"input",5)==0)
    {
        if(get_parms(name)==2) count_enabled=1; //garantisco conteggi attivi

        if(strlen(name) == 6) set_inputs(atoi(&name[5]),value);
        else return -1;

        return 0;
    }
    
    /*! \code
    if(strcmp(name,"sled")==0)
    {
        char command[64];        
        // enabling/disabling potentiometer
        i2cstruct.reg_addr =  SET_LED;
        i2cstruct.reg_value = value ? 1 : 0;		
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;
        ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
        // led in protection mode
        if((i2cstruct.reg_value & 0x02))
        {
            i2cstruct.reg_value = 0;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
            return -1;
        }
        //...
    }
    \endcode */

    if(strcmp(name,"sled")==0)
    {
        char command[64];
        // enabling/disabling potentiometer
        i2cstruct.reg_addr =  SET_LED;
        i2cstruct.reg_value = value ? 1 : 0;		
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;
        
        ioctl(pxa_qcp,VIDIOCGI2C,&i2cstruct);
        // led in protection mode
        if((i2cstruct.reg_value & 0x02))			
        {
            i2cstruct.reg_value = 0;
            ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
            return -1;
        }

        sprintf(command,"echo %d > /sys/bus/i2c/drivers/ad5245/0-002c/led_power",(unsigned char)value);
        system(command);
        led_status = value > 0 ? true : false;
        return 0;
    }

    /*! \code
    if(strcmp(name,"wg_check")==0)
    {  
        // wide gate setting
        pthread_mutex_lock(&mainlock);
        wg_check=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    \endcode */
    if(strcmp(name,"wg_check")==0)
    {  
        pthread_mutex_lock(&mainlock);
        wg_check=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"slave_id")==0)
    {
        pthread_mutex_lock(&mainlock);
        slave_id=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"serial_ms")==0)
    {  
        return 0;
    }
    if(strcmp(name,"detect_area")==0)
    {  
        pthread_mutex_lock(&mainlock);
        det_area=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#endif
    if(strcmp(name,"inst_height")==0)
    { 
        pthread_mutex_lock(&mainlock);
        inst_height=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"inst_dist")==0)
    { 
        pthread_mutex_lock(&mainlock);
        inst_dist=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#ifdef PCN_VERSION

    if(strcmp(name,"sx_dx")==0)
    { 
        return 0;
    }
    if(strcmp(name,"timebkg")==0)
    {  
        pthread_mutex_lock(&mainlock);
        SetMinBkgTh(value);
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"staticth")==0)
    {  
        pthread_mutex_lock(&mainlock);
        SetStaticTh(int(value));
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#endif
    if(strcmp(name,"dir")==0)
    {
        people_dir = value & 0x01;
        return 0;
    }
#ifdef PCN_VERSION
    if(strcmp(name,"serial_id")==0)
    {
        return 0;  	  
    }
    if(strcmp(name,"serial_sb")==0)
    {
        set_serial(ttyS0,get_parms("serial_br"),get_parms("serial_db"),
        get_parms("serial_pr"),get_parms("serial_sb"));
        return 0;  	  
    }
    if(strncasecmp(name,"serial_s",8)==0)
    {
        set_serial(ttyS1,get_parms("serial_sbr"),get_parms("serial_sdb"),
        get_parms("serial_spr"),get_parms("serial_ssb"));
        return 0; 	       
    }
    if(strncasecmp(name,"serial",6)==0)
    {
        set_serial(ttyS0,get_parms("serial_br"),get_parms("serial_db"),
        get_parms("serial_pr"),get_parms("serial_sb"));
        return 0;  	       
    }
#endif
    if(strcmp(name,"threshold")==0)
    {  
        pthread_mutex_lock(&mainlock);
        SetDoor((unsigned char)value);
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#ifdef PCN_VERSION    
    if(strcmp(name,"autoled")==0)
    {  
        pthread_mutex_lock(&mainlock);
        autoled = value ? true : false;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"auto_gain")==0) // eVS added 20130927 to manage gain of vref
    {  
        pthread_mutex_lock(&mainlock);
        auto_gain = value ? true : false;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"threshBkg")==0)
    {  
        pthread_mutex_lock(&mainlock);
        SetBkgThreshold((unsigned char)value);
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"sys_number")==0)
    {
        if(value<=0) value=1;
        pthread_mutex_lock(&mainlock);
        // TODO ??? mettere controllo (value != total_sys_number) con eventuale deinitpeople/initpeople ???
        total_sys_number = value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"sys_number_index")==0)
    {
        if(value==0) value=1;
        pthread_mutex_lock(&mainlock);
        current_sys_number = value;
        //pthread_mutex_unlock(&mainlock); 20100513 eVS, moved at the end of this if
        if(current_sys_number!=0)
        {
            if(data_wide_gate!=NULL)
                delete [] data_wide_gate;
            int dim=54*current_sys_number;
            data_wide_gate = new unsigned char [dim]; 
            memset(data_wide_gate,0, dim*sizeof(char));
        }
        else send_enable=0; //altrimenti con restore il loop continua ad inviare
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#endif
    /*! \code
    if(strcmp(name,"sxlimit")==0)
    {
        limitSx=value;
        return 0;
    }
    \endcode */
    if(strcmp(name,"sxlimit")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitSx=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    
    if(strcmp(name,"dxlimit")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitDx=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(name,"sxlimit_riga_start")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitSx_riga_start=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(name,"dxlimit_riga_start")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitDx_riga_start=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(name,"sxlimit_riga_end")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitSx_riga_end=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(name,"dxlimit_riga_end")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limitDx_riga_end=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
#ifdef PCN_VERSION
    if(strcmp(name,"cond_diff_1p")==0)
    {
        pthread_mutex_lock(&mainlock);
        diff_cond_1p = value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"diagnostic_en")==0)
    {  
        pthread_mutex_lock(&mainlock);
        diagnostic_en=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#endif

    /*! \code    
    // motion detection parameter setting
    if(strcmp(name,"move_det_col0")==0)
    {
        i2cstruct.reg_addr =  0x50;
        i2cstruct.reg_value = value;   
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    \endcode */
    
#ifdef PCN_VERSION
    if(strcmp(name,"move_det_col0")==0)
    {
        i2cstruct.reg_addr =  0x50;
        i2cstruct.reg_value = value;   
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(name,"move_det_col1")==0)
    {
        i2cstruct.reg_addr =  0x51;
        i2cstruct.reg_value = value;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(name,"move_det_row0")==0)
    {
        i2cstruct.reg_addr =  0x52;
        i2cstruct.reg_value = value;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(name,"move_det_row1")==0)
    {
        i2cstruct.reg_addr =  0x53;
        i2cstruct.reg_value = value;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(name,"move_det_alfareg")==0)
    {
        i2cstruct.reg_addr =  0x54;
        i2cstruct.reg_value = value;
        ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct);
        return 0;
    }
    if(strcmp(name,"move_det_thr")==0)
    {  
        pthread_mutex_lock(&mainlock);
        move_det_thr=value;  // is better use 4*value ?
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"move_det_en")==0)
    {  
        pthread_mutex_lock(&mainlock);
        move_det_en=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    if(strcmp(name,"door_stairs_en")==0)
    {  
        pthread_mutex_lock(&mainlock);
        door_stairs_en=value;
        pthread_mutex_unlock(&mainlock);    
        return 0;
    }
#endif
    if(strcmp(name,"up_line_limit")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limit_line_Up=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
    if(strcmp(name,"down_line_limit")==0)
    {
        pthread_mutex_lock(&mainlock); // 20100517 eVS
        limit_line_Down=value;
        pthread_mutex_unlock(&mainlock); // 20100517 eVS
        return 0;
    }
#ifdef PCN_VERSION
    //20090506 Lisbona
    if(strcmp(name,"dis_autobkg")==0)
    {    
        pthread_mutex_lock(&mainlock);
        autobkg_disable=value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
    //20130411 eVS, door_size instead of door_kind (for 2.3.10.7)
    if(strcmp(name,"door_size")==0)
    {    
        pthread_mutex_lock(&mainlock);
        door_size=(unsigned char)value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
        //20130711 eVS, handle_oor 
    if(strcmp(name,"handle_oor")==0)
    {    
        pthread_mutex_lock(&mainlock);
        //printf("Write parms %s = %d\n",name,value); // DEBUG_OMAR
        handle_oor=(unsigned char)value;
        pthread_mutex_unlock(&mainlock);
        return 0;
    }
#endif

    return -1;
}


/*!
    \brief Scrittura dei messaggi di log e su digital output.

          La funzione scrive i messaggi di log in un vettore di stringhe #records[#records_idx].
    Ciascuna stringa termina con i conteggi attuali i_people_in e i_people_out presi in ingresso 
    dalla funzione stessa.
          
    Questo vettore viene periodicamente travasato su file dal thread record_loop().
   
    Inoltre, scrive sul digital output un segnale per comunicare il numero di persone entrate e uscite
    memorizzati rispettivamente in #out0 e #out1. Ad ogni chiamata di record_counters se 
    #out0 (o #out1) e' diverso da zero viene chiamata la write_output() la quale effettua 
    una scrittura sul digital output e decrementata il valore di #out0 (o out#1).
    
    Si noti che record_counters viene chiamata alla fine di ogni ciclo del #main_loop per garantire
    che i conteggi vengano aggiornati sia su disco che su digital output.
          
    \param i_people_in numero totale persone entrate cioe' people[people_dir] (vedi passaggi dei parametri)
    \param i_people_out numero totale persone uscite cioe' people[1-people_dir] (vedi passaggi dei parametri)
*/
void record_counters(const unsigned long i_people_in, const unsigned long i_people_out)
{
    time_t curtime;
    struct tm *loctime;

        curtime = time (NULL);
        loctime = localtime (&curtime);

#ifdef PCN_VERSION
    diagnostic_log(); // 20111207 eVS, added a new function to avoid redundancy (search "diagnostic_log" in the project to see where it is used)
#endif

    if(ev_door_open && ev_door_open_rec)
    {
        //curtime = time (NULL);
        //loctime = localtime (&curtime);
        door_in = i_people_in;
        door_out = i_people_out;	

        pthread_mutex_lock(&rdlock);

        sprintf(records[records_idx],"Start\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n",
            loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year,
            loctime->tm_hour,loctime->tm_min,loctime->tm_sec,counter_in,counter_out); 
        if(records_idx < (MAX_RECORDS-1)) records_idx++;

        ev_door_open_rec = false;
        pthread_mutex_unlock(&rdlock);	
    }

    if(counter_in != i_people_in || counter_out != i_people_out)
    {
      // 20130313 eVS, se in widegate e se i contatori non sono cambiati rispetto al
      // momento di apertura della porta, veniva fatto un return che usciva da record_counters()
      // questo però impedisce la periodica scrittura del treno di umpulsi sui digital output
      // che viene fatto in fondo a questa funzione. Pertanto si al posto di uscire si evita
      // di eseguire il contenuto di questo if ma senza precludere l'esecuzione del resto della
      // funzione record_counters (in particolare il pezzo finale che genera il treno di impulsi)
      //if((total_sys_number>1) && i_people_in==door_in && i_people_out==door_out) return;
      // TODO ??? verificare che questa modifica non comprometta il fuznionamento in widegate ???
      if (total_sys_number<=1 || i_people_in!=door_in || i_people_out!=door_out)
      {        
        if(counter_in>i_people_in) counter_in = i_people_in;
        if(counter_out>i_people_out) counter_out = i_people_out;

#ifdef PCN_VERSION
        out0 += (i_people_in - counter_in);
        out1 += (i_people_out - counter_out);
#endif

        counter_in = i_people_in; 
        counter_out = i_people_out;

        if(record_enabled)
        {
            //curtime = time (NULL);
            //loctime = localtime (&curtime);

            pthread_mutex_lock(&rdlock);	// see records_loop
            sprintf(records[records_idx],"Count\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n",
                loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year,
                loctime->tm_hour,loctime->tm_min,loctime->tm_sec,counter_in,counter_out); 

            if(records_idx < (MAX_RECORDS-1)) records_idx++;
            pthread_mutex_unlock(&rdlock);
        }
      }
    }

    if(ev_door_close_rec)
    {
        unsigned long inp, outp;

        //curtime = time (NULL);
        //loctime = localtime (&curtime);
        inp = i_people_in-door_in;;
        outp = i_people_out-door_out;

        pthread_mutex_lock(&rdlock);
        sprintf(records[records_idx],"Stop\t%02d/%02d/%04d\t%02d:%02d:%02d\t%06ld\t%06ld\n",
            loctime->tm_mday,loctime->tm_mon+1,1900+loctime->tm_year,
            loctime->tm_hour,loctime->tm_min,loctime->tm_sec,inp,outp); 
        if(records_idx < (MAX_RECORDS-1)) records_idx++;

//#ifndef USE_NEW_STRATEGIES
        if(total_sys_number>1 && current_sys_number!=total_sys_number) 
            initpeople(people[0], people[1], total_sys_number, num_pers);
//#endif

        ev_door_close_rec=false;
        pthread_mutex_unlock(&rdlock);	
    }	

#ifdef PCN_VERSION
    if (total_sys_number<2)
    {
        write_output();
    }
    else if(current_sys_number==total_sys_number) //in wg mode Master=>out0 last slave=>out1 
    {
        out0=0;
        write_output(); 
    }
    else if(current_sys_number==1)
    {
        out1=0;
        write_output(); 
    }
    else
    {
        out0=0;
        out1=0;
    }
#endif
}


/*!
  \brief Scrive su digital output il risultato dei conteggi.
  
  \see record_counters()
*/
#ifdef PCN_VERSION
void write_output(void)
{
    unsigned long time0,time1;
    unsigned short outtime;

    if(out0)
    {
        gettimeofday(&stop0,NULL); 
        time0 = (stop0.tv_sec-start0.tv_sec)*1000000 + stop0.tv_usec-start0.tv_usec;

        outtime = get_parms("outtime0");

        if(time0 > (unsigned long)(2*outtime)*1000)
        {
            if(!ioctl(pcout0,PCIOCSOUT)) 
            {
                out0--;
                if(out0 < 0) out0 = 0;
                gettimeofday(&start0,NULL);
            }
        }
    }
    if(out1)
    {
        gettimeofday(&stop1,NULL); 
        time1 = (stop1.tv_sec-start1.tv_sec)*1000000 + stop1.tv_usec-start1.tv_usec;

        outtime = get_parms("outtime1");

        if(time1 > (unsigned long)(2*outtime)*1000)
        {
            if(!ioctl(pcout1,PCIOCSOUT)) 
            {
                out1--;
                if(out1 < 0) out1 = 0;
                gettimeofday(&start1,NULL);
            }
        }
    }
}
#endif


/*! 
*  \brief Configura la porta di input mediante la funzione ioctl().
*  \param input 
*  \param value 
*/
#ifdef PCN_VERSION
int set_inputs(unsigned char input,unsigned char value)
{
    void (**input_function)(unsigned long);
    int *pcin;

    input_function = (input == 0) ? &input_function0 : &input_function1;
    pcin = (input == 0) ? &pcin0 : &pcin1;

    switch(value)
    {
    case 0:
        *input_function = do_nothing;
        break;
    case 1:
        ioctl(*pcin,PCIOCSIN,RISING);
        *input_function = reset_counters;
        break;
    case 2:
        ioctl(*pcin,PCIOCSIN,RISING_FALLING);
        *input_function = enable_counting;
        break;
    case 3:
        ioctl(*pcin,PCIOCSIN,RISING_FALLING);
        *input_function = do_nothing;
        break;
    default:
        *input_function = do_nothing;
        break;
    }
    return 0;
}
#endif


/*! 
*  \brief This function do nothing.
* 
*  The input parameters is needed just for function header compability reasons (see reset_counter() and enable_counting()).
*
*  \param value
*/
void do_nothing(unsigned long value)
{
    return;
}


#ifdef PCN_VERSION
void reset_counters(unsigned long value)
{
    pthread_mutex_lock(&mainlock);
    
    unsigned long people[2]; 
    unsigned long value_in,value_out;
    char command[256];
    unsigned char send_old=1;
    if(total_sys_number>1 && current_sys_number==1)
    {
        send_old=send_enable;
        send_enable=0;
        SNP_Send(slave_id,"reset",NULL,0,ttyS1);
    }

    value_in = value & 0x0000FFFF;
    value_out = (value & 0xFFFF0000) >> 16;

    deinitpeople(num_pers);
    people[people_dir] = value_in;
    people[1-people_dir] = value_out;
    
    //initpeople(people[0],people[1]);	
    initpeople(people[0],people[1], total_sys_number, num_pers);


    counter_in = value_in;
    counter_out = value_out;
    out0 = 0;
    out1 = 0;

    door_in = value_in;
    door_out = value_out;

    if(file_exists(cr_filename))
    {
      sprintf(command,"rm %s",cr_filename);
      system(command);
    }
    if(total_sys_number>1 && current_sys_number==1)
      send_enable=send_old;

    pthread_mutex_unlock(&mainlock);
}
#endif


void enable_counting(unsigned long value)
{
    pthread_mutex_lock(&mainlock);
    
    count_enabled =(int)value;
    if(total_sys_number>1 && current_sys_number==1) 
    { 
        send_enable=(unsigned char)value;
    }
    if (value)  //door open
    {
        if (!mem_door) 
        {
            frame_cnt_door=0;
            frame_fermo=0;
            ev_door_open=true;
            if(total_sys_number<2 || total_sys_number==current_sys_number) ev_door_open_rec = true;
            else ev_door_open_rec = false;
        }
        mem_door=true;
    }
    else  //door close
    {
        if(mem_door)
        {
            frame_cnt_door=0;
            ev_door_close=true;
        }
        mem_door=false;
    } 
    pthread_mutex_unlock(&mainlock);
}



/*!
    \brief Salvataggio sfondo su file binario (#bg_filename).
    
    Viene salvato lo sfondo (#Bkgvec), la deviazione standard (#svec) e il valor medio dello sfondo acquisito (#vm_bkg).
*/
#ifdef PCN_VERSION
void SaveBkg()
{
    FILE *out = fopen(bg_filename,"wb");
    fwrite(Bkgvec,sizeof(Bkgvec),1,out);
    fwrite(svec,sizeof(svec),1,out);
    fwrite(&vm_bkg,sizeof(vm_bkg),1,out);
    fclose(out);  
}



/*! 
*  \brief Legge un valore da uno specifico registro di memoria mediante la funzione get_mem().
*  \param name nome registro
*  \return (data >> regs[i].shift) & regs[i].mask
*/
int get_gpio(char *name) // GPIO = general purpose input/output
{
    int i,data;
    int n=sizeof(regs)/sizeof(struct reg_info);

    for (i=0; i<n; i++) 
    {
        if (strstr(regs[i].name, name))
        {
            data = get_mem(regs[i].addr);      
            return (data >> regs[i].shift) & regs[i].mask;
        }
    }
    return -1;
}
#endif



/*! 
*  \brief Modifica il contenuto di uno specifico registro di memoria mediante la funzione put_mem().
*  \param name nome del registro
*  \param val valore da scrivere nel registro specificato
*/
#ifdef PCN_VERSION
int set_gpio(char *name,unsigned int val) // GPIO = general purpose input/output
{
    int i;
    unsigned int mem;
    int found=0;
    int count=0;
    int n=sizeof(regs)/sizeof(struct reg_info);

    for (i=0; i<n; i++) 
    {
        if (strcmp(regs[i].name, name)==0) 
        {
            found = i;
            count++;
        }
    }
    if (count!=1) 
    {
        print_log("Exit (set_gpio)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    mem = get_mem(regs[found].addr);
    mem &= ~(regs[found].mask << regs[found].shift);
    val &= regs[found].mask;
    mem |= val << regs[found].shift;
    put_mem(regs[found].addr, mem);  

    return 0; 
}
#endif


/*!
*  \brief Legge un valore da uno specifico indirizzo di memoria.

Internamente utilizza le chiamate di sistema map() e munmap() che servono rispettivamente a 
stabilire una mappatura tra lo spazio degli indirizzi di un processo ed un file, mentre la seconda serve per 
rimuovere il mapping a partire dall'indirizzo passato come primo parametro per un lunghezza map + MAP_SIZE.

*  \param addr indirizzo di memoria
*  \return value valore letto dall'indirizzo di memoria specificato
*/
#ifdef PCN_VERSION
int get_mem(unsigned int addr)
{
    void *map, *regaddr;
    int fd;
    unsigned int val;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd<0) 
    {
        perror("open(\"/dev/mem\")");
        return -1;
    }

    // stabilisce una mappatura tra lo spazio degli indirizzi di un processo ed un file
    map = mmap(0, 
        MAP_SIZE, 
        PROT_READ | PROT_WRITE, 
        MAP_SHARED, 
        fd, 
        addr & ~MAP_MASK 
        );
        
    if (map < 0 ) 
    {
        perror("mmap()");
        print_log("Exit (get_mem)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    regaddr = (unsigned char *)map + (addr & MAP_MASK);

    val = *(unsigned int *) regaddr;

    // rimuove il mapping a partire dall'indirizzo passato come primo parametro per un lunghezza map + MAP_SIZE
    munmap(map,MAP_SIZE);
    
    close(fd);

    return val;
}


/*! 
*  \brief Modifica il contenuto di uno specifico registro di memoria.

Internamente utilizza le chiamate di sistema map() e munmap() che servono rispettivamente a 
stabilire una mappatura tra lo spazio degli indirizzi di un processo ed un file, mentre la seconda serve per 
rimuovere il mapping a partire dall'indirizzo passato come primo parametro per un lunghezza map + MAP_SIZE.

*  \param addr indirizzo di memoria
*  \param val valore da scrivere nell'indirizzo di memoria specificato
*/
int put_mem(unsigned int addr, unsigned int val)
{
    void *map, *regaddr;
    static int fd = -1;


    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) 
    {
        perror("open(\"/dev/mem\")");
        return -1;
    }

    // stabilisce una mappatura tra lo spazio degli indirizzi di un processo ed un file
    map = mmap(0, 
        MAP_SIZE, 
        PROT_READ | PROT_WRITE, 
        MAP_SHARED, 
        fd, 
        addr & ~MAP_MASK 
        );
        
    if (map == (void*)-1 ) {
        perror("mmap()");
        print_log("Exit (put_mem)\n"); // 20111013 eVS, added to verify if exit
        exit(1);
    }

    regaddr = (unsigned char *)map + (addr & MAP_MASK);

    *(unsigned int *) regaddr = val;
    
    // rimuove il mapping a partire dall'indirizzo passato come primo parametro per un lunghezza map + MAP_SIZE
    munmap(map,MAP_SIZE);
    
    close(fd);

    return 0;
}
#endif


////////////////////
// 20091120 eVS
#define MAX_DEBUG_LOG_LINES 10000
#ifdef PCN_VERSION    
static pthread_mutex_t print_log_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

// TODO use print_log only for debug purposes or in case of exit events because it seems to cause problems
void print_log(const char *format, ...)
//!< Used for debugging in order to write messages in the debuglog.txt file
{
    #ifdef PCN_VERSION
  if (pthread_mutex_trylock(&print_log_mutex) == 0)  // just to avoid to block processing because of logging
  {
#endif
    
    char msg[255];
    va_list args;

    /*...*/
                                                                                                                        
    va_start (args, format);
    vsprintf (msg, format, args);
    va_end (args);

    /*...*/   

    // write the time on the log
    time_t curtime_reb;
    struct tm *loctime_reb;
    char reb_str[255];
  
    curtime_reb = time (NULL);
    loctime_reb = localtime (&curtime_reb);
    sprintf(reb_str,"%02d/%02d/%04d %02d:%02d:%02d\t",
        loctime_reb->tm_mday,loctime_reb->tm_mon+1,1900+loctime_reb->tm_year,
        loctime_reb->tm_hour,loctime_reb->tm_min,loctime_reb->tm_sec);
    //fprintf(record_reb,"%s",reb_str);
    strcat(reb_str, msg);

    printf("print_log(): %s", reb_str);
      
#ifdef PCN_VERSION
    if (pthread_mutex_trylock(&rdlock) == 0)  // just to avoid to block processing because of logging
    {
      FILE *record_reb;
      char command[200];

      // creating the debug log file if it does not exist
      sprintf(command,"/bin/touch /var/neuricam/debuglog.txt");	
      system(command);
      record_reb = fopen("/var/neuricam/debuglog.txt","a+");

      fseek(record_reb,0L,SEEK_END);

      unsigned long lines;
      lines = ftell(record_reb)/MED_LINE_LEN;
      if(lines >= MAX_DEBUG_LOG_LINES)
      {
          fclose(record_reb);	// the records file is full.
          sprintf(command,"mv /var/neuricam/debuglog.txt /var/neuricam/debuglog.old");
          system(command);	// rename the oldest file
          record_reb = fopen("/var/neuricam/debuglog.txt","a+"); // create a new file
      }

      // write the message on the log
      fprintf(record_reb,"%s",reb_str);
      fclose(record_reb);

      pthread_mutex_unlock(&rdlock);
    }
    
    pthread_mutex_unlock(&print_log_mutex);
  }
#endif
}
// 20091120 eVS    
////////////////////



////////////////////
// 20091120 eVS
// - Changing the door threshold has to affect the move detection zone
//   more precisely the starting and ending rows
#ifdef PCN_VERSION
int check_mov_det_parms()
{ 
    //move detection parameters have to be updated coherently with
    //the new door threshold
    int row0, row1, col0, col1, thr;
    int door_threshold = get_parms("threshold");
    
    // no-tracking zone parameters influence the move detection zone
    // so they are read from the parameters vector
    int lim_up_line   = get_parms("up_line_limit");
    int lim_down_line = get_parms("down_line_limit");
    int lim_sx            = get_parms("sxlimit");
    int lim_sx_riga_start = get_parms("sxlimit_riga_start");
    int lim_sx_riga_end   = get_parms("sxlimit_riga_end");
    int lim_dx            = get_parms("dxlimit");
    int lim_dx_riga_start = get_parms("dxlimit_riga_start");
    int lim_dx_riga_end   = get_parms("dxlimit_riga_end");
    
    // starting and ending row of the move detection zone have to be
    // set around the actual door threshold but inside the tracking zone        
    row0 = (lim_up_line   >= door_threshold-MOVE_DET_HH) ? lim_up_line   : door_threshold-MOVE_DET_HH;
    row1 = (lim_down_line <= door_threshold+MOVE_DET_HH) ? lim_down_line : door_threshold+MOVE_DET_HH;
   
    // starting and ending columns have to be modified on the basis of the 
    // new values of the no-tracking zone and the default values
    if ((lim_sx_riga_start <= row0 && lim_sx_riga_end <= row0) ||
        (lim_sx_riga_start >= row1 && lim_sx_riga_end >= row1))
        // if the no-tracking zone on the left is below or over the 
        // move detection zone (i.e. no intersection between them) 
        // then col0 is set with the default value
        col0 = MOVE_DET_COL0;
    else
        // otherwise the move detection zone has to be intersected
        // with the no-tracking zone
        col0 = (lim_sx >= MOVE_DET_COL0) ? lim_sx : MOVE_DET_COL0;
    if ((lim_dx_riga_start <= row0 && lim_dx_riga_end <= row0) ||
        (lim_dx_riga_start >= row1 && lim_dx_riga_end >= row1))
        // if the no-tracking zone on the right is below or over the
        // move detection zone (i.e. no intersection between them) 
        // then col1 is set with the default value
        col1 = MOVE_DET_COL1;
    else
        // otherwise the move detection zone has to be intersected
        // with the no-tracking zone
        col1 = (lim_dx <= MOVE_DET_COL1) ? lim_dx : MOVE_DET_COL1;

    // the move detection threshold is recomputed to take effect of the new values
    thr = COMP_MOVE_DET_THR(row0,row1,col0,col1);
    
    //print_log("Changed MD:\n\t md_r0=%d\t md_r1=%d\t md_c0=%d\t md_c1=%d\t md_thr=%d\n", row0, row1, col0, col1, thr);

    if (write_parms("move_det_row0",(unsigned short)row0) < 0)
        return -1;
    if (save_parms ("move_det_row0",(unsigned short)row0) < 0)
        return -1;

    if (write_parms("move_det_row1",(unsigned short)row1) < 0)
        return -1;
    if (save_parms ("move_det_row1",(unsigned short)row1) < 0)
        return -1;
          
    if (write_parms("move_det_col0",(unsigned short)col0) < 0)
        return -1;
    if (save_parms ("move_det_col0",(unsigned short)col0) < 0)
        return -1;

    if (write_parms("move_det_col1",(unsigned short)col1) < 0)
        return -1;
    if (save_parms ("move_det_col1",(unsigned short)col1) < 0)
        return -1;

    if (write_parms("move_det_thr",(unsigned short)thr) < 0)
        return -1;
    if (save_parms("move_det_thr",(unsigned short)thr) < 0)
        return -1;
                
    return 0;   
}
#endif
// 20091120 eVS
////////////////////

