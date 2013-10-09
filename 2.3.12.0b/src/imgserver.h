#ifndef __IMG_SERVER_H__
#define __IMG_SERVER_H__


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <math.h>

#include "directives.h"
#ifdef PCN_VERSION
#include <unistd.h>
#include <netdb.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <termios.h>
#include <linux/watchdog.h>
#include <linux/videodev.h>
#include <getopt.h>
//#include <asm/arch/pcn1001.h>
#endif

#include "default_parms.h"
#include "peopledetection.h"

#ifdef PCN_VERSION
#include "pcn1001.h"
#endif

/******************   initialization functions    ************************/
void var_init();
int init_io();
void deinit_io();
int sys_version(char *version);
int ker_version(char *version);
float fw_version();
int file_exists(char *path);

/****************  eth commands functions ********************************/
int Communication(int fd,char *buffer);

/****************  images_fpga  functions ********************************/
void get_images(unsigned char *orig,int img);
void get_10bit_image(unsigned short *dest,unsigned char *src);
int background(char *dest,unsigned short *src);
int get_8bit_image(unsigned char *dest,unsigned short *src);
int erase_flash();
int save_fpn();

/****************  socket functions **************************************/
int Send(int fd,void *buf,int len);
int Recv(int fd,void *buf,int len);
int SendString(int fd,char *buf);
int RecvString(int fd,char *buf,int maxlen);
int Connect(int *sockfd,char * addr);
void Disconnect(int *sockfd);

/****************  loops functions ***************************************/
void *ping_loop(void *arg);
void *main_loop(void *fd);
void *record_loop(void *arg);
void *input_loop0(void *arg);
void *input_loop1(void *arg);
void *ser_loopttyS0(void *arg);
void *ser_loopttyS1(void *arg);
void *watchdog_loop(void *arg);
void mainloop_enable(bool enable);

/****************  serial_port functions *********************************/
void reset_serial(int fd);
int set_serial(int fd,unsigned long baudrate,unsigned long bitesize,unsigned long parity, unsigned long stopbits);
int enable_port(unsigned short value);
int SNP_reply(unsigned char dest,unsigned char recipient,char *buffer,char *args,unsigned int args_size,int ttyS);
int SNP_Send(unsigned char dest,char *command,char *arg,int arg_size,int ttyS);
int SNP_Recv(char *buf,unsigned char *sender,unsigned char *recipient,int ttyS);
int BuildSNPCommand(char *buf,unsigned char src,unsigned char dest,char tpn,char pn,short datalen,char *data);
void CalcCrc16Block(char *pBlock, unsigned short Number, short *pCrc);
unsigned long get_arg(char *arg,int arg_size,int pos,int size);
int read_port(int fd,char *buf,int len);
int write_port(int fd,char *buf,int len);
void wide_gate_serial_parms_set();
void wide_gate_serial_parms_reset();

/*************************  I/O functions    *****************************/
void load_parms(void);
void load_counters(void);
int load_default_parms(void);
int save_parms(char *name,unsigned short value);
unsigned short get_parms(char *name);
int set_parms(char *name,unsigned short value);
int write_parms(char *name,unsigned short value);
void record_counters(const unsigned long i_people_in, const unsigned long i_people_out);
void write_output(void);
void do_nothing(unsigned long value);
void reset_counters(unsigned long value);
void enable_counting(unsigned long value);
int set_inputs(unsigned char input,unsigned char value);
void (*input_function0)(unsigned long value);
void (*input_function1)(unsigned long value);
int get_mem(unsigned int addr);
int put_mem(unsigned int addr, unsigned int val);
int get_gpio(char *name);
int set_gpio(char *name,unsigned int val);
///////////////////
// 20091120 eVS
void print_log(const char *format, ...);
int check_mov_det_parms(void);

// 20091120 eVS
///////////////////

/*********** sys calibration  I/O functions    ***************************/
void calib_load_parms(void);
int calib_load_default_parms(void);
int calib_save_parms(char *name,unsigned short value);
unsigned short calib_get_parms(char *name);
int calib_set_parms(char *name,unsigned short value);
int calib_write_parms(char *name,unsigned short value);

/*********************** RTP protocol functions **************************/
//int init_rtp(char *address);
//void deinit_rtp();
/*************************************************************************/

void prepare_for_wideconfiguration();
void restore_factory_settings(char *buffer, bool bridge);
int save_current_parms(void);
void diagnostic_log(void);

#endif /* __IMG_SERVER_H__ */
