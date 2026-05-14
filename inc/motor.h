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
	  CTRL_FREE_DUTY,
} CtrlMode_e;

extern volatile CtrlMode_e s_ctrl_mode;
extern volatile int32_t s_target_pos;
extern volatile int32_t s_target_vel; /* count/s */

void MotorControl_Init(void);
void ControlTimer_Init(void);
void Motor_MoveToPosition(int32_t target_count);
void Motor_MoveWithDuty(int32_t duty) ;
void Motor_SetVelocity(int32_t vel_cps);
void Motor_UpdateVelocity(int32_t vel_cps);   /* PID state'i bozmadan target update */
void Motor_Stop(void);
void Motor_EmergencyStop(void);
void Motor_ResetPosition(void);   /* QEI sayacini + hiz olcum gecmisini sifirla */
void Motor_SetMaxVelocity(int32_t vmax_cps);
int32_t Motor_GetMaxVelocity(void);
void  Motor_SetPosKp(float kp);
void  Motor_SetPosKi(float ki);
float Motor_GetPosKp(void);
float Motor_GetPosKi(void);


#if defined(__cplusplus)
}
#endif
