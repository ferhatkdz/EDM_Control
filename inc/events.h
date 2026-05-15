#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * events.h — QPC event tipleri
 *
 * Tüm dinamik event'ler burada tanımlanır.
 * QEvt'yi miras alan struct'lar; QF_NEW ile tahsis edilir.
 */
#include "qpc.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>   /* NAN, isnanf */

/*-------------------------------------------------
 * G-code hareketi
 *
 * NAN → o eksen bu komutta belirtilmedi.
 * M-code ve G-code aynı event'te taşınabilir
 * (örn. M3 + G0 Z-1.0 aynı satırda).
 *------------------------------------------------*/
typedef struct {
    QEvt    super;
    float   x, y, z, w;    /* mm; NAN = belirtilmemiş */
    float   f;              /* mm/dak; 0 = değişmez   */
    uint8_t gcode;          /* 0=G0, 1=G1, 28=G28     */
    uint8_t mcode;          /* 0=yok, 3=M3, 5=M5      */
    bool    has_x, has_y, has_z, has_w;
    bool    is_home;        /* G28 */
} GCodeEvt;

/*-------------------------------------------------
 * PC'ye gönderilecek cevap (MotionAO → GCodeAO)
 *------------------------------------------------*/
typedef struct {
    QEvt    super;
    char    msg[96];
    uint8_t len;
} GCodeRspEvt;

/*-------------------------------------------------
 * Axis CPU'ya hareket komutu (MotionAO → AxisCommAO)
 *------------------------------------------------*/
typedef struct {
    QEvt    super;
    int32_t x_counts;   /* NAN yerine INT32_MIN = belirtilmemiş */
    int32_t y_counts;
    bool    has_x;
    bool    has_y;
} AxisCommCmdEvt;

#if defined(__cplusplus)
}
#endif
