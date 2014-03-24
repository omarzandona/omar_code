#include <pthread.h> // must be the first include (the added -D_THREAD_SAFE flag in the makefile should be enough but just to be sure...)

#include "record_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <string.h> // memset()
#include <sys/time.h>
#include <limits.h>

#include "peopledetection.h"
#include "directives.h"

//#define _DEBUG_
//#define _COUNT_DURING_RECORD_


////////////////////////////////////////////////////////////////////////////////
// consts
const unsigned int BORDER_X_REC = 9; //9 or 12 could be used
const unsigned int BORDER_Y_REC = 8; //5 or 8 could be used
const unsigned int DSP_SZ = (NX-2*BORDER_X_REC)*(NY-2*BORDER_Y_REC)/2; //NN;

const unsigned int IMG_SZ = 2*DSP_SZ; //NN;

#ifndef _COUNT_DURING_RECORD_
//const int RU_SINGLE_GRAB_DIM = NN/2 + IMG_SZ + 2;
const unsigned int RU_SINGLE_GRAB_DIM = DSP_SZ + IMG_SZ + 2;
#else
//const int RU_SINGLE_GRAB_DIM = NN/2 + IMG_SZ + 2*sizeof(unsigned long) + 2;
const unsigned int RU_SINGLE_GRAB_DIM = DSP_SZ + IMG_SZ + 2*sizeof(unsigned long) + 2;
#endif

const unsigned int RU_NUM_GRAB_PER_PACKET = 13;
const unsigned int RU_BUF_DIM = RU_NUM_GRAB_PER_PACKET * 40; // to be sure that buffer dim is a multiple of RU_PACKET_DIM to simplify the Send() call

const unsigned long RU_MAX_WAIT_TIME = 3000; // milliseconds
const unsigned int RU_ACQ_TIME = 19000; // microseconds


////////////////////////////////////////////////////////////////////////////////
// external references needed to combine digital input with image data
extern unsigned char input_test0;
extern unsigned char input_test1;


////////////////////////////////////////////////////////////////////////////////
// external references needed in case of failure in order to restart the mainloop
//extern bool is_recording;
extern void mainloop_enable(bool enable);


////////////////////////////////////////////////////////////////////////////////
// external reference needed to Send packets on the socket
extern unsigned int Send(int fd,void *buf,int len);


////////////////////////////////////////////////////////////////////////////////
// external references needed to combine counters with image data
#ifdef _COUNT_DURING_RECORD_
extern int count_enabled;
extern unsigned char limit_line_Down;
extern unsigned char limit_line_Up;
extern unsigned char people_dir;
extern unsigned char move_det_en;
extern unsigned short get_parms(char *name);
extern void detectAndTrack( unsigned char *disparityMap,
                            unsigned long &peoplein,
                            unsigned long &peopleout, 
                            const int &enabled, 
                            const unsigned short &door,
                            const unsigned char &direction,
                            const unsigned char &move_det_en
#  ifdef USE_NEW_TRACKING
                            , const int &min_y_gap);
#  else
                            );
#  endif

#endif


////////////////////////////////////////////////////////////////////////////////
// types declaration
const unsigned int RU_GRAB_DATA_DIM = RU_NUM_GRAB_PER_PACKET * RU_SINGLE_GRAB_DIM;
const unsigned int RU_PACKET_DIM = sizeof(unsigned long) + RU_GRAB_DATA_DIM;

typedef struct _SyncData
{
  pthread_t sync_loop_id;
  
  // not shared data
  int  socket;                 // socket file descriptor
  struct timeval last_request_time;    // used to verity connection problems, i.e., no packets are not asked for so long
  unsigned long sent_frame_count;
  unsigned char packet[RU_PACKET_DIM];
  
  // shared data
  bool is_sync_running;        // used to verify is sync_loop is running   
  bool is_sync_stop_requested; // used to ask for sync_loop termination  
  bool is_sending_packet;      // used to verify if a packet is still in sending state
  bool is_send_requested;      // used to verify if a send was requested by the communication thread (command "recimgdsp")
  
  // mutex used for sharing data
  //pthread_mutex_t g_mtx_sync_data;
} tSyncData; // data shared between the process using ru_reply_data() and the 


typedef struct _AcqData
{
  pthread_t acq_loop_id;

  // not shared data
  unsigned long frame_count;
  struct timeval start_time;
  struct timeval stop_time;
  bool is_buffer_full;        // if the buffer becomes full, only black frames will be sent
  int pxa_qcp;                // used to read images  
#ifdef _COUNT_DURING_RECORD_
  unsigned long people_rec[2];
#endif
  
  // shared data
  bool is_acq_running;        // used to verify is acq_loop is running
  bool is_acq_stop_requested; // used to ask for acq_loop termination
  int idx_r, idx_w;           // used to access the buffer for reading and writing
  unsigned char buffer[RU_BUF_DIM][RU_SINGLE_GRAB_DIM]; // used to store acquired data

  // mutex used for sharing data
  //pthread_mutex_t g_mtx_acq_data; 
} tAcqData;


typedef struct _RecState 
{
  tSyncData sd;
  tAcqData  ad;
} tRecState;


////////////////////////////////////////////////////////////////////////////////
// global variables
static bool g_send_only_disp = false;

static tRecState rs;
static pthread_mutex_t g_mtx_sync_data = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_mtx_acq_data = PTHREAD_MUTEX_INITIALIZER;

static bool g_is_recording = false;
static pthread_mutex_t mtx_g_is_recording = PTHREAD_MUTEX_INITIALIZER; 

#ifdef _DEBUG_
static pthread_mutex_t mtx_printf = PTHREAD_MUTEX_INITIALIZER;
#endif


////////////////////////////////////////////////////////////////////////////////
// local routines declaration 
static bool _is_acq_running(tAcqData & sd);
static bool _is_acq_buffer_full(tAcqData & ad);
static bool _is_acq_data_still_available(tAcqData & ad);

static bool _is_acq_stop_requested(tAcqData & sd);

static bool _prepare_and_send_packet(tSyncData & sd, tAcqData & ad);
static void _wait_acq_loop_termination(tAcqData & ad);

static void _acq_loop_start(tAcqData & ad, const int pxa_qcp);
static void _acq_loop_stop(tAcqData & ad, tSyncData & sd);

static bool _is_sync_running(tSyncData & sd);
static bool _wait_send_or_stop_requests(tSyncData & sd, const bool check_stop);
static void _wait_enough_data_or_buffer_full(tAcqData & ad);
//static void _get_disp_map(unsigned char* const & Frame_DSP, unsigned char* const & orig);
static void _get_images(unsigned char* const & Frame_SX, unsigned char* const & Frame_DX, unsigned char* const & Frame_DSP, unsigned char* const & orig);

#ifdef _COUNT_DURING_RECORD_
static void _buffer_insert_elem(unsigned char* const & i_img_sx, unsigned char* const & i_img_dx, unsigned char* const & i_disp_map, unsigned long i_cnt[2], unsigned char i_input_test0, unsigned char i_input_test1, tAcqData & io_ad);
#else
static void _buffer_insert_elem(unsigned char* const & i_img_sx, unsigned char* const & i_img_dx, unsigned char* const & i_disp_map, unsigned char i_input_test0, unsigned char i_input_test1, tAcqData & io_ad);
#endif

static bool _is_sync_sending_packet(tSyncData & sd);
static bool _is_sync_stop_requested(tSyncData & sd);
static void _wait_sync_loop_termination(tSyncData & sd);

static void _sync_loop_start(tSyncData & sd, const int pxa_qcp, const int i_fd);
static void _sync_loop_stop(tSyncData & sd);

void *acq_loop(void *arg);
void *sync_loop(void *arg);


////////////////////////////////////////////////////////////////////////////////
// routines definitions
/*static void
_get_disp_map(unsigned char* const & Frame_DSP, unsigned char* const & orig)
{
  unsigned long* ptr;
  unsigned char* ptrd;
  int i,j;//,step;

  ptr = (unsigned long *) orig; // puntatore ad una zona di memoria a 32bits
  ptrd = Frame_DSP;
  
  for(j=0;j<NY;++j)
  {
    //step = j*NX;
    for(i=0;i<(NX >> 1);++i,++ptr,++ptrd)
    { 
      //Frame_DSP[step] = (*(ptr+(NX>>1)) & 0x0000000F) << 4;
      //step++;
      *ptrd = (*(ptr+(NX>>1)) & 0x0000000F) << 4;
      ++ptrd;

      //Frame_DSP[step] = (*(ptr+(NX>>1)) & 0x000F0000) >> 12; 
      //step++;
      *ptrd = (*(ptr+(NX>>1)) & 0x000F0000) >> 12;
    } 

    // salta una riga corrispondente ad un blocco di 320 bytes ovvero NX/2*4
    ptr += (NX >> 1);		// 2-rows step
  }
}*/


////////////////////////////////////////////////////////////////////////////////
static void
_get_images(unsigned char* const & Frame_SX, unsigned char* const & Frame_DX, 
            unsigned char* const & Frame_DSP, unsigned char* const & orig)
{
  unsigned long* ptr;
  unsigned char* ptrd, * ptrsx, * ptrdx;
  int i,j;

  ptr = (unsigned long *) orig; // puntatore ad una zona di memoria a 32bits
  ptrd = Frame_DSP;
  ptrsx = Frame_SX;
  ptrdx = Frame_DX;
  
  for(j=0;j<NY;++j)
  {
    for(i=0;i<(NX >> 1);++i,++ptr,++ptrd,++ptrsx) //,++ptrdx)
    { 
      *ptrsx = (*ptr & 0x000000FF); // The byte is taken on the pixels of the first row second column buffer 160*120*4=320*240. //viene prelevato il byte relativo al pixel della 1°riga 2°colonna del buffer 160*120*4=320*240
      *ptrdx = (*ptr & 0x0000FF00) >> 8; // The byte is taken on the pixels of the first row first column of the buffer 160*120*4=320*240. //viene prelevato il byte relativo al pixel della 1°riga 1°colonna del buffer 160*120*4=320*240
      *ptrd = (*(ptr+(NX>>1)) & 0x0000000F) << 4;
      
      ++ptrd;
      ++ptrsx;
      ++ptrdx;
           
      *ptrsx = (*ptr & 0x00FF0000) >> 16; 
      *ptrdx = (*ptr & 0xFF000000) >> 24;
      *ptrd = (*(ptr+(NX>>1)) & 0x000F0000) >> 12;
    } 

    // salta una riga corrispondente ad un blocco di 320 bytes ovvero NX/2*4
    ptr += (NX >> 1);		// 2-rows step
  }
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_acq_running(tAcqData & ad)
{ 
  pthread_mutex_lock(&g_mtx_acq_data);
  bool is_running = ad.is_acq_running;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  return is_running;
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_acq_buffer_full(tAcqData & ad)
{
  pthread_mutex_lock(&g_mtx_acq_data);
  bool is_buffer_full = ad.is_buffer_full;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  return is_buffer_full;
}


////////////////////////////////////////////////////////////////////////////////
static unsigned int
__dist_idxs(const int idx_r, const int idx_w)
{
  int dist;
  if (idx_r <= idx_w)
    dist = idx_w-idx_r;
  else
    dist = idx_w+(RU_BUF_DIM-idx_r);    
  return dist;
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_acq_data_still_available(tAcqData & ad)
{
  pthread_mutex_lock(&g_mtx_acq_data);
  int idx_w = ad.idx_w;
  int idx_r = ad.idx_r;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  bool is_data_available = (__dist_idxs(idx_r, idx_w) >= RU_NUM_GRAB_PER_PACKET);  
  return is_data_available;
}


////////////////////////////////////////////////////////////////////////////////
bool
_is_acq_stop_requested(tAcqData & ad)
{
  pthread_mutex_lock(&g_mtx_acq_data);
  bool is_stop_requested = ad.is_acq_stop_requested;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  return is_stop_requested;
}


////////////////////////////////////////////////////////////////////////////////
static bool
_prepare_and_send_packet(tSyncData & sd, tAcqData & ad)
{    
  // send data start
  pthread_mutex_lock(&g_mtx_sync_data);
  sd.is_sending_packet = true;
  pthread_mutex_unlock(&g_mtx_sync_data);

  bool data_finished = false;
  if (_is_acq_buffer_full(ad) && !_is_acq_data_still_available(ad))
  {
    memset((unsigned char*)&(ad.buffer[ad.idx_r][0]), 0, RU_PACKET_DIM);
    data_finished = true;
  }
  else
    sd.sent_frame_count += RU_NUM_GRAB_PER_PACKET;

  // build the packet (idx of the first frame in data + data)
  int sent_frame_count_sz = sizeof(sd.sent_frame_count);
  unsigned char* ptr = &(sd.packet[0]);
  unsigned char* ptr_fr_cnt = (unsigned char*)&(sd.sent_frame_count);
  for (int i=0; i<sent_frame_count_sz; ++i, ++ptr, ++ptr_fr_cnt)
    *ptr = *ptr_fr_cnt;

  bool problem_arised;
  unsigned int data_dim; 
  unsigned char* ptrb = (unsigned char*)&(ad.buffer[ad.idx_r][0]);
  unsigned char* ptrp = &(sd.packet[sent_frame_count_sz]);
  if (!g_send_only_disp)
  {      
    data_dim = RU_GRAB_DATA_DIM;
    memcpy(
      ptrp, 
      ptrb, // use this pointer is possible only if the RU_BUF_DIM is a multiple of NUM_IMG_PER_PACKET otherwise you have to manage the fact that buffer is circular
      data_dim);
    assert(sent_frame_count_sz+data_dim == RU_PACKET_DIM);
  }
  else
  {
    int packet_elem_sz = RU_SINGLE_GRAB_DIM-IMG_SZ;
    
    data_dim = 0; 
    for (unsigned int i=0; i<RU_GRAB_DATA_DIM; i+= RU_SINGLE_GRAB_DIM)
    {
      memcpy(ptrp, ptrb, packet_elem_sz);
      ptrb += RU_SINGLE_GRAB_DIM;
      ptrp += packet_elem_sz;
      data_dim += packet_elem_sz;
    }    
  }  
  
  // Send packet    
  unsigned int packet_dim = sent_frame_count_sz + data_dim; 
  unsigned int bytes = Send(
    sd.socket, 
    sd.packet,
    packet_dim); 
                    
  problem_arised = (bytes < packet_dim);
  
  if (!problem_arised)
  {       
    //if (!_is_acq_buffer_full(ad) || _is_acq_data_still_available(ad))
    if (!data_finished)
    {           
      pthread_mutex_lock(&g_mtx_acq_data);
      ad.idx_r = (ad.idx_r + RU_NUM_GRAB_PER_PACKET) % RU_BUF_DIM;
      pthread_mutex_unlock(&g_mtx_acq_data);
    }
  }
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_prepare_and_send_packet: sent %d bytes\n", packet_dim);
  pthread_mutex_unlock(&mtx_printf);
#endif
  
  // send data finished
  pthread_mutex_lock(&g_mtx_sync_data);
  sd.is_send_requested = false; // send request satisfied so set the flag to false
  sd.is_sending_packet = false;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  return problem_arised;
}


////////////////////////////////////////////////////////////////////////////////
static void
_wait_acq_loop_termination(tAcqData & ad)
{
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_acq_loop_termination: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  while (_is_acq_running(ad))
    usleep(1000);  // wait acq_loop termination
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_acq_loop_termination: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char*
_copy_internal_area(unsigned char* ptri, unsigned char* ptrb, bool is_4_bit)
{
  unsigned char* ptrb_orig = ptrb;  
  
  ptri += (BORDER_Y_REC*NX);
  for (unsigned int r=BORDER_Y_REC; r<NY-BORDER_Y_REC; ++r)
  {
    ptri += BORDER_X_REC;
    if (is_4_bit)
    {
      for (unsigned int c=BORDER_X_REC; c<NX-BORDER_X_REC; c+=2, ptri+=2, ++ptrb)
      {    
        unsigned char elem1 = *ptri;
        unsigned char elem2 = *(ptri+1);
        *ptrb = ((elem1 & 0xF0) | ((elem2 >> 4) & 0x0F));      
      }
    }
    else
    {
      int ncols = NX-2*BORDER_X_REC;
      memcpy(ptrb, ptri, ncols);
      ptrb += ncols;
      ptri += ncols;
    }
    ptri += BORDER_X_REC;
  }
    
  return ptrb_orig + (is_4_bit ? DSP_SZ : IMG_SZ);
}

static void
#ifndef _COUNT_DURING_RECORD_
_buffer_insert_elem(unsigned char* const & i_img_sx, unsigned char* const & i_img_dx, unsigned char* const & i_disp_map, unsigned char i_input_test0, unsigned char i_input_test1, tAcqData & io_ad)
#else
_buffer_insert_elem(unsigned char* const & i_img_sx, unsigned char* const & i_img_dx, unsigned char* const & i_disp_map, unsigned long i_cnt[2], unsigned char i_input_test0, unsigned char i_input_test1, tAcqData & io_ad)
#endif
{ 
  //assert(NN % 2 == 0);
  assert((NX-2*BORDER_X_REC) % 2 == 0);
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_buffer_insert_elem: write image data into buffer\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
  
  if (!io_ad.is_buffer_full)
  {    
    unsigned char* ptrb = &(io_ad.buffer[io_ad.idx_w][0]);

    // copy internal region of the ***disparity map*** into buffer and return the buffer pointer properly updated to point after the copied disparity map part
    ptrb = _copy_internal_area(i_disp_map, ptrb, true); 
     
#ifdef _COUNT_DURING_RECORD_
    for (int j=0; j<2; ++j)
    {
      unsigned char* ptrcnt = (unsigned char*)&(i_cnt[j]);
      for (unsigned int i=0; i<sizeof(i_cnt[j]); ++i, ++ptrb, ++ptrcnt)
        *ptrb = *ptrcnt;
    }
#endif
    
    // insert digital inputs in the buffer and coherently update buffer pointer
    *ptrb++ = i_input_test0;
    *ptrb++ = i_input_test1;
    
    // copy internal region of the ***image*** into buffer and return the buffer pointer properly updated to point after the copied image part
    ptrb = _copy_internal_area(i_img_sx, ptrb, false);

    pthread_mutex_lock(&g_mtx_acq_data);
    int new_idx = (io_ad.idx_w+1) % RU_BUF_DIM;
    if (new_idx != io_ad.idx_r)
      io_ad.idx_w = new_idx;
    else
      io_ad.is_buffer_full = true;
    pthread_mutex_unlock(&g_mtx_acq_data);
  }
}


////////////////////////////////////////////////////////////////////////////////
// thread that continuously acquires images and puts them into the circular buffer
// the shared buffer between the acq_loop and the sync_loop is managed by the
// g_mtx_acq_data mutex
void *acq_loop(void *arg)
{
  static const int imagesize = NN << 2;
  static unsigned char Frame[imagesize];
  static unsigned char Frame_DSP[NN];
  static unsigned char Frame_SX[NN];
  static unsigned char Frame_DX[NN];
  
  tAcqData & ad = rs.ad;

  pthread_mutex_lock(&g_mtx_acq_data);
  ad.is_acq_running = true;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  while (!_is_acq_stop_requested(ad))
  {
#ifdef _DEBUG_    
    pthread_mutex_lock(&mtx_printf);
    printf("acq_loop: read image data\n");
    pthread_mutex_unlock(&mtx_printf);
    usleep(100000);  // for debug purposes
#endif

    read(ad.pxa_qcp,Frame,imagesize);    
    //_get_disp_map(Frame_DSP,Frame);
    _get_images(Frame_SX, Frame_DX, Frame_DSP, Frame);
    
    ad.frame_count++;
    
    //if (ad.frame_count%3==0)    
    if (ad.frame_count%2==0 || g_send_only_disp)
    {
#ifdef _COUNT_DURING_RECORD_
      detectAndTrack(
        Frame_DSP,
        ad.people_rec[0],ad.people_rec[1],
        count_enabled,
        get_parms("threshold"),
        get_parms("dir"),
        move_det_en
#  ifdef USE_NEW_TRACKING
        , (limit_line_Down-limit_line_Up+1)/4);
#  else
        );
#  endif
      _buffer_insert_elem(Frame_SX, Frame_DX, Frame_DSP, ad.people_rec, input_test0, input_test1, ad);
#else
      _buffer_insert_elem(Frame_SX, Frame_DX, Frame_DSP, input_test0, input_test1, ad);   
#endif
    }
  }
  
  pthread_mutex_lock(&g_mtx_acq_data);
  ad.is_acq_running = false;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
static bool
_wait_send_or_stop_requests(tSyncData & sd, const bool check_stop)
{
  bool is_send_requested, is_stop_requested, still_wait;
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_send_or_stop_requests: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  bool problem_arised = false;
  unsigned long elapsed_time = 0;
  do
  {
    pthread_mutex_lock(&g_mtx_sync_data);
    is_send_requested = sd.is_send_requested;
    is_stop_requested = sd.is_sync_stop_requested;
    pthread_mutex_unlock(&g_mtx_sync_data);

    {
      struct timeval now;    
      gettimeofday(&now, NULL);
      elapsed_time = (now.tv_sec - sd.last_request_time.tv_sec)*1000 + (now.tv_usec - sd.last_request_time.tv_usec)/1000;
      problem_arised = (elapsed_time > RU_MAX_WAIT_TIME);
    }
                            
    still_wait = (((check_stop) ? !is_stop_requested : true) && !is_send_requested && !problem_arised);

    if (still_wait)
      usleep(RU_ACQ_TIME);
    
  } while (still_wait);

  gettimeofday(&sd.last_request_time, NULL);
    
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_send_or_stop_requests: leave (elapsed_time=%ld)\n", elapsed_time);
  pthread_mutex_unlock(&mtx_printf);
#endif

  return problem_arised;
}


////////////////////////////////////////////////////////////////////////////////
static void
_wait_enough_data_or_buffer_full(tAcqData & ad)
{
  bool still_wait;
  
  int idx_w; 
  int idx_r = ad.idx_r;

#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_enough_data_or_buffer_full: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  do
  {
    pthread_mutex_lock(&g_mtx_acq_data);
    idx_w = ad.idx_w;
    bool is_buffer_full = ad.is_buffer_full;
    pthread_mutex_unlock(&g_mtx_acq_data);

    still_wait = (__dist_idxs(idx_r, idx_w) <= RU_NUM_GRAB_PER_PACKET && !is_buffer_full);
    
    if (still_wait)    
      usleep(1000);  // wait enough data or buffer full

  } while (still_wait);
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_enough_data_or_buffer_full: exit (idx_r=%d, idx_w=%d)\n", idx_r, idx_w);
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_sync_running(tSyncData & sd)
{ 
  pthread_mutex_lock(&g_mtx_sync_data);
  bool is_running = sd.is_sync_running;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  return is_running;
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_sync_stop_requested(tSyncData & sd)
{ 
  pthread_mutex_lock(&g_mtx_sync_data);
  bool is_stop_requested = sd.is_sync_stop_requested;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  return is_stop_requested;
}


////////////////////////////////////////////////////////////////////////////////
static void
_wait_sync_loop_termination(tSyncData & sd)
{
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_sync_loop_termination: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  while (_is_sync_running(sd))
    usleep(1000);  // wait sync_loop termination
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_wait_sync_loop_termination: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
static bool 
_is_sync_sending_packet(tSyncData & sd)
{
  pthread_mutex_lock(&g_mtx_sync_data);
  bool is_sending_packet = sd.is_sending_packet;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  return is_sending_packet;
}


////////////////////////////////////////////////////////////////////////////////
static void
_acq_loop_start(tAcqData & ad, const int pxa_qcp)
{
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_acq_loop_start: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  // not shared data
  gettimeofday(&ad.start_time, NULL);
  ad.frame_count = 0;
  ad.pxa_qcp = pxa_qcp;
  ad.is_buffer_full = false;
  
#ifdef _COUNT_DURING_RECORD_
  ad.people_rec[0] = 0;
  ad.people_rec[1] = 0;
#endif

  // shared data
  ad.is_acq_running = false;
  ad.is_acq_stop_requested = false;
  ad.idx_r = 0;
  ad.idx_w = 0;
 
  //pthread_mutex_init(&g_mtx_acq_data, NULL);  
  pthread_create (&ad.acq_loop_id, NULL, acq_loop, NULL);
   
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_acq_loop_start: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
static void
_acq_loop_stop(tAcqData & ad, tSyncData & sd)
{
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_acq_loop_stop: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  // ask for termination
  pthread_mutex_lock(&g_mtx_acq_data);
  ad.is_acq_stop_requested = true;
  pthread_mutex_unlock(&g_mtx_acq_data);
  
  // wait threads termination
  _wait_acq_loop_termination(ad);  

  gettimeofday(&ad.stop_time, NULL);
    
  // destroy thread data
  //pthread_mutex_destroy(&g_mtx_acq_data); 
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_acq_loop_stop: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
// thread that 1) receives the reply request made by ru_reply_data() and 2) reads 
// data from the buffer filled by the acq_loop; the shared buffer between the acq_loop
// and the sync_loop is managed by the g_mtx_acq_data mutex whilst the shared data
// between the ru_reply_data() and the sync_loop are managed by the g_mtx_sync_data mutex
void *sync_loop(void *arg)
{
  int pxa_qcp = *((int *)arg);

  tAcqData & ad = rs.ad;
  tSyncData & sd = rs.sd;
  
  // start acquisition thread
  _acq_loop_start(ad, pxa_qcp);    
  
  // tell others that sync_loop is now running
  pthread_mutex_lock(&g_mtx_sync_data);
  sd.is_sync_running = true;
  pthread_mutex_unlock(&g_mtx_sync_data);

  unsigned int problem_code = 0x0000;
  
  // sending loop
  while (!_is_sync_stop_requested(sd) && problem_code == 0x0000)
  {
    // wait a send request or a stop request
    problem_code = (_wait_send_or_stop_requests(sd, true) ? problem_code | 0x000F : problem_code);
    
    if (!_is_sync_stop_requested(sd) && problem_code == 0x0000)
    {
      // wait enough data if still running
      _wait_enough_data_or_buffer_full(ad);
           
      // send data 
      problem_code = (_prepare_and_send_packet(sd, ad) ? problem_code | 0x00F0 : problem_code);
      
#ifdef _DEBUG_    
      pthread_mutex_lock(&mtx_printf);
      printf("sync_loop: packet sent (failed? 0x%0X)\n", problem_code);
      pthread_mutex_unlock(&mtx_printf);
#endif

      usleep(1000);  // wait before check for send or stop requests
    }
  }
   
  // stop acquisition loop  
  _acq_loop_stop(ad, sd);
  
  // tell others that sync_loop is now stopped
  pthread_mutex_lock(&g_mtx_sync_data);
  sd.is_sync_running = false;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  if (problem_code != 0x0000)
  {
    printf("sync_loop: stop because a problem arised (0x%0X)\n", problem_code);

    pthread_mutex_lock(&mtx_g_is_recording);
    g_is_recording = false;
    pthread_mutex_unlock(&mtx_g_is_recording);
    
    //is_recording = false;
    
    mainloop_enable(1);
  } 
  
  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
static void
_sync_loop_start(tSyncData & sd, const int pxa_qcp, const int i_fd)
{
#ifdef _DEBUG_    
  //pthread_mutex_init(&mtx_printf, NULL);
  printf("_sync_loop_start: enter\n");
#endif

  gettimeofday(&sd.last_request_time, NULL);
  sd.sent_frame_count = 0;

  sd.is_sync_running = false;
  sd.is_sync_stop_requested = false;
  sd.is_sending_packet = false;
  sd.is_send_requested = false;
  sd.socket = i_fd;
  
  //pthread_mutex_init(&g_mtx_sync_data, NULL);

  int* p_int = (int*) malloc(sizeof(int));
  *p_int = pxa_qcp;
  
  pthread_create (&sd.sync_loop_id, NULL, sync_loop, p_int);
  
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_sync_loop_start: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
static void
_sync_loop_stop(tSyncData & sd)
{
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_sync_loop_stop: enter\n");
  pthread_mutex_unlock(&mtx_printf);
#endif

  // ask for termination
  pthread_mutex_lock(&g_mtx_sync_data);
  sd.is_sync_stop_requested = true;
  pthread_mutex_unlock(&g_mtx_sync_data);
  
  // wait threads termination
  _wait_sync_loop_termination(sd);  
  
  // destroy thread data
  //pthread_mutex_destroy(&g_mtx_sync_data);
   
#ifdef _DEBUG_    
  pthread_mutex_lock(&mtx_printf);
  printf("_sync_loop_stop: leave\n");
  pthread_mutex_unlock(&mtx_printf);
#endif
}


////////////////////////////////////////////////////////////////////////////////
int ru_start_record(const int i_pxa_qcp, const int i_fd, const bool i_save_only_disp)
{
#ifdef _COUNT_DURING_RECORD_
  int ret = -RU_NUM_GRAB_PER_PACKET;
#else
  int ret = RU_NUM_GRAB_PER_PACKET;
#endif  

  
  pthread_mutex_lock(&mtx_g_is_recording);
  bool already_started = g_is_recording;
  g_send_only_disp = i_save_only_disp;
  pthread_mutex_unlock(&mtx_g_is_recording);
  
  printf("ru_start_record:\n");
  if (!already_started)
  {   
    printf("ru_start_record: starting...\n");

    _sync_loop_start(rs.sd, i_pxa_qcp, i_fd);     
    
    pthread_mutex_lock(&mtx_g_is_recording);
    g_is_recording = true;
    printf("ru_start_record: started!\n");
    pthread_mutex_unlock(&mtx_g_is_recording);
  }   
  else
  {
    printf("ru_start_record: failes because already started!\n");
  }
  
  return ret;
}


////////////////////////////////////////////////////////////////////////////////
float ru_stop_record(void)
{ 
  float fps = -1;
  
  pthread_mutex_lock(&mtx_g_is_recording);
  bool already_started = g_is_recording;
  pthread_mutex_unlock(&mtx_g_is_recording);
  
  if (already_started)
  {
    _sync_loop_stop(rs.sd);
  
    float mtime = ((rs.ad.stop_time.tv_sec - rs.ad.start_time.tv_sec) * 1000 + (rs.ad.stop_time.tv_usec - rs.ad.start_time.tv_usec)/1000.0f) + 0.5f;
    if (mtime > 0)
      fps = 1000.0f * (rs.ad.frame_count / mtime);
      
    printf("ru_stop_record:\n");
    printf("  Start-stop time (seconds): %f\n", mtime/1000.0f);
    printf("  Acquired frames count: %ld\n", rs.ad.frame_count);
    printf("  Acquisition frame rate: %f\n", fps);
    printf("  Sent frame count: %ld\n", rs.sd.sent_frame_count);
    
    pthread_mutex_lock(&mtx_g_is_recording);
    g_is_recording = false;
    pthread_mutex_unlock(&mtx_g_is_recording);
  }
  
  return fps;
}


////////////////////////////////////////////////////////////////////////////////
int ru_reply_data(void)
{
  int ret = -1;
  
  pthread_mutex_lock(&mtx_g_is_recording);
  bool already_started = g_is_recording;
  //printf("ru_reply_data: is acquisition running? %d\n", already_started);
  pthread_mutex_unlock(&mtx_g_is_recording);

  if (already_started)
  {
    ret = 0;
    
    tSyncData & sd = rs.sd;
    
#ifdef _DEBUG_    
    pthread_mutex_lock(&mtx_printf);
    printf("--> ru_reply_data: wait\n");
    pthread_mutex_unlock(&mtx_printf);
#endif

    // wait if another send has to be finished
    while (_is_sync_sending_packet(sd))
      usleep(1000);  // wait that packet is sent

#ifdef _DEBUG_    
    pthread_mutex_lock(&mtx_printf);
    printf("--> ru_reply_data: socket free\n");
    pthread_mutex_unlock(&mtx_printf);
#endif
    
    // request a new image transfer 
    pthread_mutex_lock(&g_mtx_sync_data);
    sd.is_send_requested = true;
    pthread_mutex_unlock(&g_mtx_sync_data);
    
#ifdef _DEBUG_    
    pthread_mutex_lock(&mtx_printf);
    printf("--> ru_reply_data: data request sent\n");
    pthread_mutex_unlock(&mtx_printf);
#endif
  }
  else
  {
#ifdef _DEBUG_    
    pthread_mutex_lock(&mtx_printf);
    printf("--> ru_reply_data: send not asked because not recording\n");
    pthread_mutex_unlock(&mtx_printf);
#endif
  }
   
  return ret;
}

