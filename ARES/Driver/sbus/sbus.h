#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u

#define SBUS_VALUE_MAX ((fp32)671)
#define SBUS_VALUE_OFFSET ((fp32)1024)

extern void sbus_init(void);
extern void sbus_disable(void);
extern void sbus_restart(void);

#endif
