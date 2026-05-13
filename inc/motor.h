#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>


/* Kontrol modu */
typedef enum {
    CTRL_OFF = 0,
    CTRL_VELOCITY,        /* Sadece hiz döngüsü    */
    CTRL_POSITION,        /* Pos + vel iç içe      */
} CtrlMode_e;

extern volatile CtrlMode_e s_ctrl_mode;
extern volatile int32_t s_target_pos;
extern volatile int32_t s_target_vel; /* count/s */

void MotorControl_Init(void);
void ControlTimer_Init(void);
void Motor_MoveToPosition(int32_t target_count);


#if defined(__cplusplus)
}
#endif
