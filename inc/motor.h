#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * motor.h — Geriye dönük uyumluluk katmanı
 *
 * Tüm Motor_* fonksiyonları artık axis.h/axis.c içindeki
 * eksen-agnostik Axis_* fonksiyonlarına inline wrapper'dır.
 * Doğrudan Z ekseni üzerinde çalışır (g_axes[AXIS_Z]).
 *
 * Yeni kod için axis.h API'sini kullanın.
 */
#include "axis.h"

/* 5kHz kontrol zamanlayıcısı başlatma */
void ControlTimer_Init(void);

/* --- Hareket komutları --- */
static inline void   Motor_MoveToPosition(int32_t c)  { Axis_MoveToPosition(&g_axes[AXIS_Z], c); }
static inline void   Motor_MoveWithDuty  (int32_t d)  { Axis_MoveWithDuty  (&g_axes[AXIS_Z], d); }
static inline void   Motor_SetVelocity   (int32_t v)  { Axis_SetVelocity   (&g_axes[AXIS_Z], v); }
static inline void   Motor_UpdateVelocity(int32_t v)  { Axis_UpdateVelocity(&g_axes[AXIS_Z], v); }
static inline void   Motor_Stop          (void)        { Axis_Stop          (&g_axes[AXIS_Z]);    }
static inline void   Motor_EmergencyStop (void)        { Axis_EmergencyStop (&g_axes[AXIS_Z]);    }
static inline void   Motor_ResetPosition (void)        { Axis_ResetPos      (&g_axes[AXIS_Z]);    }

/* --- Pos loop hız sınırı --- */
static inline void    Motor_SetMaxVelocity(int32_t v)  { Axis_SetMaxVelocity(&g_axes[AXIS_Z], v); }
static inline int32_t Motor_GetMaxVelocity(void)       { return Axis_GetMaxVelocity(&g_axes[AXIS_Z]); }

/* --- Canlı PID ayarı --- */
static inline void  Motor_SetPosKp(float kp) { Axis_SetPosKp(&g_axes[AXIS_Z], kp); }
static inline void  Motor_SetPosKi(float ki) { Axis_SetPosKi(&g_axes[AXIS_Z], ki); }
static inline float Motor_GetPosKp(void)     { return Axis_GetPosKp(&g_axes[AXIS_Z]); }
static inline float Motor_GetPosKi(void)     { return Axis_GetPosKi(&g_axes[AXIS_Z]); }

#if defined(__cplusplus)
}
#endif
