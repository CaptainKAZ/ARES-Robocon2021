#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_MAX         ((fp32)671)
#define RC_CH_VALUE_OFFSET      ((fp32)1024)

typedef __packed struct
{
        __packed struct
        {
                fp32 ch[10];
        } rc;

} RC_ctrl_t;

extern void remote_control_init(void);

extern const RC_ctrl_t *get_remote_control_point(void);

#endif
