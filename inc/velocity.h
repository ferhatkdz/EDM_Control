#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>

#include "NuMicro.h"

/*-------------------------------------------------
 * Sabitler
 *------------------------------------------------*/
/* Encoder çözünürlüğü — mekanik sisteme göre ayarla */
#define QEI_COUNTS_PER_REV     55000UL   /* 1000 CPR × 4 (quadrature) */

/* QEI sayaç taşma değeri: 32-bit max veya
 * çok hızlı sistemler için sarma önlemi           */
#define QEI_MAX_COUNT          0xFFFFFFFFUL

/* Hız ölçümü için timer periyodu */
#define VEL_SAMPLE_US          200UL    /* 5kHz */


/*-------------------------------------------------
 * Hiz ölçümü: M/T metodu
 * Hem düsük hem yüksek hizda dogru sonuç
 *
 * M metodu:  ?count / ?t   ? yüksek hizda iyi
 * T metodu:  1 / (pulse arasi süre) ? düsük hizda iyi
 * M/T:       her ikisini birlestirir
 *------------------------------------------------*/
typedef struct
{
    int32_t  prev_count;
    int32_t  curr_count;
    float    velocity_cps;   /* count/s */
    float    velocity_rpm;
    float    velocity_mms;   /* mm/s — mekanik konfigürasyona göre */

    /* M/T için timer */
    uint32_t timer_ticks;
    uint32_t prev_timer;

    /* Low-pass filtre */
    float    vel_filtered;
    float    lpf_alpha;      /* 0..1, küçük=daha fazla filtre */

} VelMeasure_t;

/* Mekanik dönüsüm sabitleri — sisteme göre ayarla */
#define LEAD_SCREW_MM_PER_REV  5.0f    /* mm/devir */
#define COUNTS_PER_REV_F       55000.0f

/* count/s ? mm/s */
#define CPS_TO_MMS(cps)  ((cps) * LEAD_SCREW_MM_PER_REV / COUNTS_PER_REV_F)

static inline int32_t QEI_GetSignedCount(QEI_T *qei) {
	return (int32_t)QEI_GET_CNT_VALUE(qei);
}

void VelMeasure_Update(VelMeasure_t *vm, int32_t qei_count);

#if defined(__cplusplus)
}
#endif
