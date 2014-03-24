/*!
\file calib_io.cpp
\brief Lettura/scrittura dei parametri relativi alla calibrazione.

Gestione di lettura e scrittura dei parametri di calibrazione dei sensori 
e per l'FPGA nel file calibration.txt (calib_save_parms()) e/o nei 
registri SET_comp00 (0x30), ..., SET_comp15 (0x3F) dell'FPGA relativi 
alle sedici disparita' nelle due modalita' 25/30 piuttosto che 31/40
settabili da win_client. 

Inoltre, qui e' gestita la configurazione anche di altri parametri
dell'FPGA come: la dimensione della finestra di correlazione tramite
il registro SET_win (0x40); le varie soglie e i parametri usati per il 
calcolo della mappa di disparita' tramite i registri SET_DISP (0x14), 
SET_MinTH (0x12) e SET_TH&UNI (0x13); il settaggio delle impostazioni dei 
sensori tramite i registri SET_Vref (0x01),  SET_Vprec (0x02) e 
SET_Vgap (0x04). 

Infine, settare la soglia soglia_bkg usata per vedere se due pixel 
corrispondenti  in due mappe di disparita' (una il background e
l'altra una mappa acquisita) possono essere considerati simili: se la
loro differenza in modulo e' inferiore a soglia_bkg.

Tutti i parametri appena citati sono riassunti in \ref tabella_calib.

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/



/*!
    \brief Set calibration parameters from file if it exists otherwise set default calibration parameters
*/
void calib_load_parms(void)
{
    FILE *parms;
    char name[PARM_STR_LEN];
    char svalue[PARM_STR_LEN];
    unsigned short value;
    int ret;
    det_area=get_parms("detect_area");

    calib_load_default_parms();
    if((parms = fopen(ca_filename,"r")) == NULL) return;
    else
    {
        do                                                               {
            ret = fscanf(parms,"%s ",name);
            if((calib_get_parms(name) != 0xFFFF) && ret != EOF)    // checking if the parameter is valid
            {
                fscanf(parms,"%s",svalue);
                value = strtol(svalue,NULL,16);
                calib_set_parms(name,value);                // saving parameters in memory

                if(strncasecmp(name,"step",4)!=0)
                    calib_write_parms(name,value);
                if((strncasecmp(name,"step225",7)==0 && det_area==0))
                    calib_write_parms(name,value);                // writing parameters on devices
                else if((strncasecmp(name,"step240",7)==0 && det_area==1))
                    calib_write_parms(name,value); 

                usleep(100000);
            }
        }
        while(ret != EOF);
    }  
    fclose(parms);
}


/*!
    \brief Set calibration parameters to the default ones.
*/
int calib_load_default_parms(void)
{
    int i;

    memcpy(calib_parm_values,calib_default_values,sizeof(calib_default_values));
    for(i=0;(strlen(calib_parm_names[i]) && (i < 256));i++)
    {
        if(calib_write_parms(calib_parm_names[i],calib_parm_values[i]) < 0)
            return -1;
    }
    return 0;
}


/*!
    \brief Save all the parameters of the calibration on the file #ca_filename.
*/
// saving parameters to a text file
int calib_save_parms(char *name,unsigned short value)
{
    FILE *out;
    char sname[PARM_STR_LEN];
    char string[PARM_STR_LEN];
    char command[32];
    char ch;
    int i;
    long pos,len;

    if(calib_set_parms(name,value) < 0) return -1;	//wrong parameter
    sprintf(command,"/bin/touch %s",ca_filename);	// creating the parameters file if it does not exist
    system(command);

    out = fopen(ca_filename,"r+");
    if(!out) {printf("Unable to open the file: %s\n",ca_filename); return -1;}
    else
    {
        while(!feof(out))
        { 
            pos = ftell(out);
            fscanf(out,"%s ",sname);

            if(!strcmp(name,sname))
            {
                fseek(out,pos,SEEK_SET);
                sprintf(string,"%s 0x%x",sname,value);
                len = strlen(string);
                for(i=len;i<PARM_STR_LEN-1;i++) string[i] = ' ';  //deleting unused chars
                string[PARM_STR_LEN-1] = '\0';
                fputs(string,out);
                fputc('\n',out);
                fclose(out);
                return 0;
            }
            do ch = fgetc(out);			//going to the end of the line
            while(ch != '\n' && !feof(out)) ;
        }
    }   

    sprintf(string,"%s 0x%x",name,value);
    len = strlen(string);
    for(i=len;i<PARM_STR_LEN-1;i++) string[i] = ' ';	//deleting unused chars
    string[PARM_STR_LEN-1] = '\0';
    fputs(string,out);
    fputc('\n',out);   
    fclose(out);

    return 0;
}


/*!
    \brief Restituisce da #calib_parm_values il valore del parametro specificato da name.
    
    Scandisce #calib_parm_names fino a che non trova name.
    Se lo trova restituisce il corrispondente valore in #calib_parm_values
    altrimenti restituisce 0xFFFF.
    
    \param[in] name nome del parametro di cui restituire il valore
*/
unsigned short calib_get_parms(char *name)
{
    int i;

    for(i=0;(strlen(calib_parm_names[i]) && (i < 256));i++)
    {
        if(!strcmp(calib_parm_names[i],name))
            return calib_parm_values[i];
    }
    return 0xFFFF;
}


/*!
    \brief Setta il valore del parametro corrispondente a name.
    
    Scandisce #calib_parm_names fino a che non trova name.
    Se lo trova aggiorna il corrispondente valore in #calib_parm_values
    con il value. Se non lo trova non fa nulla e torna -1.
    
    \param[in] name nome del parametro da settare
    \param[in] value nuovo valore per il parametro specificato
*/
int calib_set_parms(char *name,unsigned short value)
{
    int i;

    for(i=0;(calib_parm_names[i] && (i < 256));i++)
    {
        if(!strcmp(calib_parm_names[i],name))
        {
            calib_parm_values[i] = value;
            return 0;
        }
    }
    return -1;
}


/*!
    \brief Write parameters values in FPGA (expect for "threshBkg" which is stored in RAM).
*/
int calib_write_parms(char *name,unsigned short value)
{
    if(strncasecmp(name,"dac",3)==0)
    {
        unsigned char addr,len,val,sensor;

        if(!strncasecmp(name,"dac_vref",8)) 	{addr = SET_VREF;val = (value*255)/3300;}
        else if(!strncasecmp(name,"dac_vprec",9)) 	{addr = SET_VPREC;val = (value*255)/3300;}
        else if(!strncasecmp(name,"dac_vgap",8)) 	{addr = SET_VGAP;val = (value*255)/3300;}
        else return -2;

        len = strlen(name);
        sensor = (name[len-1] == '1') ? 1 : 2;

        i2cstruct.reg_addr =  SET_DAC;
        i2cstruct.reg_value = sensor;
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;

        i2cstruct.reg_addr =  addr;
        i2cstruct.reg_value = val;
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;
        return 0;
    }
    if(strncasecmp(name,"map",3)==0)
    {
        unsigned char addr,arg;
        arg = (unsigned char) value;

        if(!strcmp(name,"map_minth")) 	addr = SET_MINTH;
        else if(!strcmp(name,"map_thuni")) 	addr = SET_TH_UNI;
        else if(!strcmp(name,"map_disp")) 	addr = SET_DISP;
        else return -2;	

        i2cstruct.reg_addr =  addr;
        i2cstruct.reg_value = arg;
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;

        return 0;
    }

    if(strcmp(name,"threshBkg")==0)
    {  
        pthread_mutex_lock(&mainlock);
        SetBkgThreshold((unsigned char)value);
        pthread_mutex_unlock(&mainlock);
        return 0;
    }

    if(strncasecmp(name,"step225_",8)==0)
    { 
        unsigned char c=name[8]; //indica a quale passo assegno il dato;
        unsigned char addr;
        switch(c)
        {
        case '0': addr = SET_STEP0; break; 
        case '1': addr = SET_STEP1; break;
        case '2': addr = SET_STEP2; break;
        case '3': addr = SET_STEP3; break;
        case '4': addr = SET_STEP4; break;
        case '5': addr = SET_STEP5; break;
        case '6': addr = SET_STEP6; break;
        case '7': addr = SET_STEP7; break;
        case '8': addr = SET_STEP8; break;
        case '9': addr = SET_STEP9; break;
        case 'A': addr = SET_STEPA; break;
        case 'B': addr = SET_STEPB; break;
        case 'C': addr = SET_STEPC; break;
        case 'D': addr = SET_STEPD; break;
        case 'E': addr = SET_STEPE; break;
        case 'F': addr = SET_STEPF; break;
        default: addr=0xFF;	
        } 
        if(addr != 0xFF)
        {
            i2cstruct.reg_addr =  addr;
            i2cstruct.reg_value = (unsigned char)value;
            if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;
            return 0;
        }
        else return -1;

        return 0;
    }

    if(strncasecmp(name,"step240_",8)==0)
    { 
        unsigned char c=name[8]; //indica a quale passo assegno il dato;
        unsigned char addr;

        switch(c)
        {
        case '0': addr = SET_STEP0; break; 
        case '1': addr = SET_STEP1; break;
        case '2': addr = SET_STEP2; break;
        case '3': addr = SET_STEP3; break;
        case '4': addr = SET_STEP4; break;
        case '5': addr = SET_STEP5; break;
        case '6': addr = SET_STEP6; break;
        case '7': addr = SET_STEP7; break;
        case '8': addr = SET_STEP8; break;
        case '9': addr = SET_STEP9; break;
        case 'A': addr = SET_STEPA; break;
        case 'B': addr = SET_STEPB; break;
        case 'C': addr = SET_STEPC; break;
        case 'D': addr = SET_STEPD; break;
        case 'E': addr = SET_STEPE; break;
        case 'F': addr = SET_STEPF; break;
        default: addr=0xFF;	
        }

        if(addr != 0xFF)
        {
            i2cstruct.reg_addr =  addr;
            i2cstruct.reg_value = (unsigned char)value;
            if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct))  return -1;
            return 0;
        }
        else return -1;
        return 0;
    }

    if(strncasecmp(name,"winsz",5)==0)
    {  
        i2cstruct.reg_addr =  SET_WIN;
        i2cstruct.reg_value = (unsigned char)value;
        if(ioctl(pxa_qcp,VIDIOCSI2C,&i2cstruct)) return -1;
        return 0;
    }
    return -1;
}
