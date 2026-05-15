#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * probe.h — G38.2 / G38.3 kenar bulma modülü
 *
 * G38.2 (MEASURE): Temas noktasını mutlak makine koordinatında raporlar,
 *                  encoder sıfırlanmaz.
 * G38.3 (ZERO):    Temas anında Axis_ResetPos() çağrılır; temas noktası
 *                  yeni referans sıfırı olur.
 *
 * Temas tespiti: g_u32FilteredGapVoltage < PROBE_GAP_SHORT_ADC (20)
 * Çağrı yeri:    EADC01_IRQHandler (Ark_Tick() ile aynı bağlam, 5 kHz)
 * İç prescaler:  /5 → 1 kHz etkili çalışma
 */

#include <stdint.h>
#include <stdbool.h>
#include "axis.h"

/*-------------------------------------------------
 * Probe state machine durumları
 *------------------------------------------------*/
typedef enum {
    PROBE_IDLE = 0,
    PROBE_SEEKING,      /* Elektrot iş parçasına doğru ilerliyor */
    PROBE_RETRACTING,   /* Temas bulundu, geri çekiliyor          */
    PROBE_DONE,         /* Geri çekme tamamlandı                  */
    PROBE_ERROR,        /* Temas olmadan hedef aşıldı             */
} ProbeState_e;

/*-------------------------------------------------
 * Çalışma modu
 *------------------------------------------------*/
typedef enum {
    PROBE_MODE_MEASURE = 0, /* G38.2: mutlak koord. raporla, sıfırlama yok */
    PROBE_MODE_ZERO    = 1, /* G38.3: temas anında Axis_ResetPos() çağır   */
} ProbeMode_e;

/*-------------------------------------------------
 * API
 *------------------------------------------------*/
void         Probe_Start(AxisId_e axis, int32_t approach_cps,
                         float target_mm, ProbeMode_e mode);
void         Probe_Tick(void);    /* EADC01_IRQHandler'dan çağrılır */
void         Probe_Reset(void);   /* Durumu PROBE_IDLE'a çek        */

ProbeState_e Probe_GetState(void);
float        Probe_GetContactMm(void); /* Temas anındaki pozisyon (mm)  */
bool         Probe_GetSuccess(void);
AxisId_e     Probe_GetAxis(void);

#if defined(__cplusplus)
}
#endif
