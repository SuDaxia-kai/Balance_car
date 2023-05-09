#ifndef __REMOTE_CTRL_H
#define __REMOTE_CTRL_H

#include "sys.h"

struct PPM_Receiver {
    uint8_t update_flag;
    uint16_t databuf[10];
};

void ppm_receive_init(void);

extern struct PPM_Receiver ppm_receiver;
extern uint8_t tim_update_flag;
extern uint16_t capture_val;


#endif

