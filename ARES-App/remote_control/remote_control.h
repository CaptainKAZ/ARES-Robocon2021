#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u

#define SBUS_VALUE_MAX ((fp32)671)
#define SBUS_VALUE_OFFSET ((fp32)1024)

extern void SBUS_init(void);
extern void SBUS_disable(void);
extern void SBUS_restart(void);

#endif
