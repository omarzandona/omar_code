/*!
\file pcn1001.h
\brief File header principale dell'applicazione server contenente la definizione 
di varie costanti e della struttura dati pxa_i2c_t.

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/

/*
 *  linux/include/asm-arm/arch-pxa/pcn1001.h
 *
 *  Author:	Flavio Suligoi
 *  Created:	Feb 26, 2007
 *  Copyright:	Flavio Suligoi <f.suligoi@eurotech.it>
 *  		Eurotech S.p.A. <info@eurotech.it>
 *
 *  Based on linux/include/asm-arm/arch-pxa/wwpc1100.h
 *
 *  Original copyright message:
 *
 *  Author:	Rodolfo Giometti
 *  Created:	Sep 18, 2006
 *  Copyright:	Rodolfo Giometti, Eurotech S.p.A.
 *
 *  Based on linux/include/asm-arm/arch-pxa/mainstone.h
 *
 *  Original copyright message:
 *
 *  Author:	Nicolas Pitre
 *  Created:	Nov 14, 2002
 *  Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ASM_ARCH_PCN1001_H
#define ASM_ARCH_PCN1001_H


/*
 * --- PCN1001 specific functions --------------------------------------------
 */

/********************* i2c xilinx registers	**************************/

/*! \def SET_DAC
\brief ??? */
/*! \def SET_VREF
\brief ??? */
/*! \def SET_VPREC
\brief ??? */
/*! \def SET_VCM
\brief ??? */
/*! \def SET_VGAP
\brief ??? */
/*! \def RESET_XILINX
\brief ??? */
#define SET_DAC		0x00
#define SET_VREF	0x01
#define SET_VPREC	0x02
#define SET_VCM		0x03
#define SET_VGAP	0x04
#define RESET_XILINX	0x06

/*! \def SET_LED
\brief Indirizzo di memoria dell'FPGA per il settaggio del valore degli illuminatori */
#define SET_LED		0x05

/*! \def SET_FRAMERATE
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def SET_FRAMERATE_30FPS
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def SET_FRAMERATE_60FPS
\brief ??? DEFINITA MA NON USATA ??? */
#define SET_FRAMERATE		0x07
#define SET_FRAMERATE_30FPS	0x00
#define SET_FRAMERATE_60FPS	0x01

/*! \def PCIO_OUT0
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def PCIO_OUT1
\brief ??? DEFINITA MA NON USATA ??? */
#define PCIO_OUT0		0x08
#define PCIO_OUT1		0x09

/*! \def FW_VERSION
\brief Indirizzo di memoria dell'FPGA per recuperare la versione del bitstream presente sul PCN-1001 
(mediante la funzione fw_version())*/
#define FW_VERSION	0x0A

/*! \def ACQ_MODE
\brief ??? DEFINITA MA NON USATA ??? */

/*! \def ACQ_MODE_OFF
\brief ??? DEFINITA MA NON USATA ??? (acquisizione bloccata) */

/*! \def ACQ_MODE_SINGLE
\brief ??? DEFINITA MA NON USATA ??? (acquisizione di un singolo frame) */

/*! \def ACQ_MODE_ON
\brief ??? DEFINITA MA NON USATA ??? (acquisizione continua) */
#define ACQ_MODE		0x10
#define ACQ_MODE_OFF		0x00000004
#define ACQ_MODE_SINGLE 	0x00000001
#define ACQ_MODE_ON		0x00000002


/*! \def MUX_MODE
\brief Indirizzo del registro in cui andare a specificare la modalit&agrave; di acquisizione.

Le varie modalit&agrave; sono specificate con altre define<BR>
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
*/

/*! \def MUX_MODE_10_NOFPN_DX
\brief sensore dx, 10bit, no FPN */
/*! \def MUX_MODE_10_NOFPN_SX
\brief sensore sx, 10bit, no FPN */
/*! \def MUX_MODE_8_NOFPN
\brief sensore dx e sx, 8bit, noFPN */
/*! \def MUX_MODE_8_FPN
\brief sensore dx e sx, 8bit, FPN */
/*! \def MUX_MODE_8_FPN_ODC
\brief sensore dx e sx, 8bit, FPN, ODC */
/*! \def MUX_MODE_8_FPN_ODC_SOBEL
\brief sensore dx e sx, 8bit, FPN, ODC, SOBEL */
/*! \def MUX_MODE_8_FPN_ODC_DISP
\brief DispMap + sensore dx e sx, 8bit, FPN, ODC */
/*! \def MUX_MODE_8_FPN_ODC_DISP_SOBEL
\brief DispMap + sensore dx e sx, 8bit, FPN, ODC, SOBEL */
/*! \def MUX_MODE_8_FPN_ODC_MEDIAN_DISP
\brief DispMap,MEDIAN + 8bit, FPN, ODC */
/*! \def MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL
\brief DispMap,MEDIAN + 8bit, FPN, ODC, SOBEL */
/*! \def MUX_MODE_8_FPN_ODC_DISP_SOBEL_SX
\brief DispMap + sensore sx, 8bit, FPN, ODC + sensore sx, 8bit, FPN, ODC, SOBEL */
/*! \def MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL_SX
\brief DispMap + MEDIAN su DispMap + sensore sx, 8bit, FPN, ODC  +sensore sx, 8bit, FPN, ODC, SOBEL */
#define MUX_MODE				0x11
#define MUX_MODE_10_NOFPN_DX			0x81	
#define MUX_MODE_10_NOFPN_SX			0x82
#define MUX_MODE_8_NOFPN			0x83
#define MUX_MODE_8_FPN				0x84
#define MUX_MODE_8_FPN_ODC			0x88
#define MUX_MODE_8_FPN_ODC_SOBEL		0x90
#define MUX_MODE_8_FPN_ODC_DISP			0x01
#define MUX_MODE_8_FPN_ODC_DISP_SOBEL		0x02
#define MUX_MODE_8_FPN_ODC_MEDIAN_DISP		0x04
#define MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL	0x08
#define MUX_MODE_8_FPN_ODC_DISP_SOBEL_SX 	0x03
#define MUX_MODE_8_FPN_ODC_MEDIAN_DISP_SOBEL_SX 0x09

/*! \def SET_MINTH
\brief ??? */
/*! \def SET_TH_UNI
\brief ??? */
/*! \def SET_DISP
\brief ??? */
/*! \def SET_VM
\brief ??? */
/*! \def SET_Z
\brief ??? */
/*! \def SET_PSH
\brief ??? */
#define SET_MINTH	0x12
#define SET_TH_UNI	0x13
#define SET_DISP	0x14
#define SET_VM		0x15
#define SET_Z		0x16
#define SET_PSH		0x17

/*! \def SET_STEP0
\brief ??? */
/*! \def SET_STEP1
\brief ??? */
/*! \def SET_STEP2
\brief ??? */
/*! \def SET_STEP3
\brief ??? */
/*! \def SET_STEP4
\brief ??? */
/*! \def SET_STEP5
\brief ??? */
/*! \def SET_STEP6
\brief ??? */
/*! \def SET_STEP7
\brief ??? */
/*! \def SET_STEP8
\brief ??? */
/*! \def SET_STEP9
\brief ??? */
/*! \def SET_STEPA
\brief ??? */
/*! \def SET_STEPB
\brief ??? */
/*! \def SET_STEPC
\brief ??? */
/*! \def SET_STEPD
\brief ??? */
/*! \def SET_STEPE
\brief ??? */
/*! \def SET_STEPF
\brief ??? */
#define SET_STEP0	0X30
#define SET_STEP1	0X31
#define SET_STEP2	0X32
#define SET_STEP3	0X33
#define SET_STEP4	0X34
#define SET_STEP5	0X35
#define SET_STEP6	0X36
#define SET_STEP7	0X37
#define SET_STEP8	0X38
#define SET_STEP9	0X39
#define SET_STEPA	0X3A
#define SET_STEPB	0X3B
#define SET_STEPC	0X3C
#define SET_STEPD	0X3D
#define SET_STEPE	0X3E
#define SET_STEPF	0X3F

/*! \def SET_STEPF
\brief ??? */
#define SET_WIN		0X40

/*! \def FL_RST
\brief ??? */
/*! \def FL_SETROW
\brief ??? */
/*! \def FL_WRSEL
\brief ??? */
#define FL_RST		0x20
#define FL_SETROW	0x21
#define FL_WRSEL	0x24	

/*! \def FL_LOBYTE
\brief Apposito registro dell'FPGA in cui viene memorizzata la parte bassa o meno significativa 
del generico elemento della look-up-table riferita all'FPN piuttosto che all'ODC.*/
#define FL_LOBYTE	0x22

/*! \def FL_HIBYTE
\brief Apposito registro dell'FPGA in cui viene memorizzata la parte alta o pi&ugrave; significativa 
del generico elemento della look-up-table riferita all'FPN piuttosto che all'ODC.*/
#define FL_HIBYTE	0x23

/********************** drivers ioctl defs************************/

/*! \def PCIOCSOUT
\brief ??? */
/*! \def PCIOCGINT
\brief ??? */
/*! \def PCIOCSINT
\brief ??? */
#define PCIOCSOUT	_IO('v',50)
#define PCIOCGINT	_IOR('v',51, unsigned short)
#define PCIOCSINT	_IOW('v',52, unsigned short)

/*! \def PCIOCGIN
\brief ??? NOT USED */
#define PCIOCGIN	_IOR('v',53, unsigned char)

/*! \def PCIOCSIN
\brief Comando passato come secondo parametro alla chiamata di sistema ioctl() 
per sincronizzare un dispositivo, ovvero per triggerarlo sul fronte di salita e/o discesa */
#define PCIOCSIN	_IOW('v',54, unsigned char)


/*! \def VIDIOCGI2C
\brief E' un comando non presente nel set standard di video4linux (ma definito internamente) 
che viene utilizzato per leggere dei particolari valori da determinati registri di memoria dell'FPGA, ad esempio 
correzione distorsione ottica, fixed pattern noise, lettura della versione del bitstream.*/
#define VIDIOCGI2C	_IOR('v',50, pxa_i2c_t)

/*! \def VIDIOCSI2C
\brief E' un comando non presente nel set standard di video4linux (ma definito internamente) 
che viene utilizzato per scrivere dei particolari valori in determinati registri di memoria dell'FPGA, ad esempio 
correzione distorsione ottica, fixed pattern noise, reset della memoria flash, abilitazione/disabilitazione 
dei potenziometri, settaggio parametri di motion detection. 
Inoltre viene utilizzato anche per settare la modalit&agrave; di acquisizione dati da FPGA (ovvero il MUX_MODE_).*/
#define VIDIOCSI2C	_IOW('v',51, pxa_i2c_t) 

/*! \def FPGA_DONE
\brief ??? NOT USED ??? */
#define FPGA_DONE	_IOR('v',50, unsigned char)

/*
VIDIOCCAPTURE Arguments
*/

/*! \def VIDEO_START
\brief Corrisponde al terzo parametro di input della chiamata ioctl() per l'interfacciamento con il 
       dispositivo di acquisizione video (sfruttando le API del kernel di linux, video4linux) ed in particolare 
       per abilitare l'acquisizione delle immagini. 
       ioctl(pxa_qcp, VIDIOCCAPTURE, VIDEO_START); */
#define VIDEO_START         0

/*! \def VIDEO_STOP
\brief Corrisponde al terzo parametro di input della chiamata ioctl() per l'interfacciamento con il 
       dispositivo di acquisizione video (sfruttando le API del kernel di linux, video4linux) ed in particolare 
       per terminare l'acquisizione delle immagini. 
       ioctl(pxa_qcp, VIDIOCCAPTURE, VIDEO_STOP); */
#define VIDEO_STOP			-1

/*! \def STILL_IMAGE
\brief ??? DEFINITA MA NON USATA ??? */
#define STILL_IMAGE				1

/*
Image format definition
*/

/*! \def CAMERA_IMAGE_FORMAT_RAW8
\brief Valore per il campo palette della struttura dati video_picture di video4linux. */
#define CAMERA_IMAGE_FORMAT_RAW8                0

/*! \def CAMERA_IMAGE_FORMAT_RAW9
\brief ??? DEFINITA MA NON USATA ??? <BR> Valore per il campo palette della struttura dati video_picture di video4linux. */
/*! \def CAMERA_IMAGE_FORMAT_RAW10
\brief ??? DEFINITA MA NON USATA ??? <BR> Valore per il campo palette della struttura dati video_picture di video4linux. */
/*! \def CAMERA_IMAGE_FORMAT_RGB444
\brief ??? DEFINITA MA NON USATA ??? <BR> Valore per il campo palette della struttura dati video_picture di video4linux. */
/*! \def CAMERA_IMAGE_FORMAT_RGB555
\brief ??? DEFINITA MA NON USATA ??? <BR> Valore per il campo palette della struttura dati video_picture di video4linux. */
/*! \def CAMERA_IMAGE_FORMAT_RGB565
\brief ??? DEFINITA MA NON USATA ??? <BR> Valore per il campo palette della struttura dati video_picture di video4linux. */
#define CAMERA_IMAGE_FORMAT_RAW9                1
#define CAMERA_IMAGE_FORMAT_RAW10               2
#define CAMERA_IMAGE_FORMAT_RGB444              3
#define CAMERA_IMAGE_FORMAT_RGB555              4
#define CAMERA_IMAGE_FORMAT_RGB565              5

/*! \def CAMERA_IMAGE_FORMAT_RGB666_PACKED
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGB666_PLANAR
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGB888_PACKED
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGB888_PLANAR
\brief ??? DEFINITA MA NON USATA ??? */
#define CAMERA_IMAGE_FORMAT_RGB666_PACKED       6
#define CAMERA_IMAGE_FORMAT_RGB666_PLANAR       7
#define CAMERA_IMAGE_FORMAT_RGB888_PACKED       8
#define CAMERA_IMAGE_FORMAT_RGB888_PLANAR       9

/*! \def CAMERA_IMAGE_FORMAT_RGBT555_0
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGBT888_0
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGBT555_1
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_RGBT888_1
\brief ??? DEFINITA MA NON USATA ??? */
#define CAMERA_IMAGE_FORMAT_RGBT555_0          10  //RGB+Transparent bit 0
#define CAMERA_IMAGE_FORMAT_RGBT888_0          11
#define CAMERA_IMAGE_FORMAT_RGBT555_1          12  //RGB+Transparent bit 1
#define CAMERA_IMAGE_FORMAT_RGBT888_1          13
                                                                                                                             
/*! \def CAMERA_IMAGE_FORMAT_YCBCR400
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_YCBCR422_PACKED
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_YCBCR444_PACKED
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def CAMERA_IMAGE_FORMAT_YCBCR444_PLANAR
\brief ??? DEFINITA MA NON USATA ??? */
#define CAMERA_IMAGE_FORMAT_YCBCR400           14
#define CAMERA_IMAGE_FORMAT_YCBCR422_PACKED    15
#define CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR    16
#define CAMERA_IMAGE_FORMAT_YCBCR444_PACKED    17
#define CAMERA_IMAGE_FORMAT_YCBCR444_PLANAR    18

/*
Bpp definition
*/

/*! \def YUV422_BPP
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def RGB565_BPP
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def RGB666_UNPACKED_BPP
\brief ??? DEFINITA MA NON USATA ??? */
/*! \def RGB666_PACKED_BPP
\brief ??? DEFINITA MA NON USATA ??? */
#define YUV422_BPP				16
#define RGB565_BPP				16
#define RGB666_UNPACKED_BPP			32
#define RGB666_PACKED_BPP			24


/*!
\struct pxa_i2c_t
\brief Struttura dati relativa al protocollo di comunicazione i2c.

    i2c sta per IIC cio&egrave; Inter Integrated Circuit che &egrave; un 
    meccanismo pensato per far comunicare diversi circuiti integrati che 
    risiedono sullo stesso circuito stampato.
*/
typedef struct {
	unsigned char adapter_nr; ///< numero adattatore
	unsigned char slave_addr; ///< indirizzo PCN slave
	unsigned char reg_addr; ///< indirizzo registro
	unsigned char reg_value; ///< valore registro
} pxa_i2c_t;

#endif /* ASM_ARCH_PCN1001_H */

