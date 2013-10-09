#ifndef __RECORD_UTILS__
#define __RECORD_UTILS__

int   ru_start_record(const int i_pxa_qcp, const int i_fd, const bool i_save_only_disp = false);
float ru_stop_record(void);
int   ru_reply_data(void);
//bool  ru_is_recording(void);

#endif
