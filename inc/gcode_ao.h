#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * gcode_ao.h — GCodeAO: USB VCOM karakter toplama + G-code dispatch
 *
 * Görevler:
 *   1. USB VCOM'dan gelen karakterleri (UART_RX_SIG) satır tamponuna ekler.
 *   2. '\n' veya '\r' alındığında GCode_Parse() çağırır.
 *   3. Geçerli komut → GCodeEvt(GCODE_CMD_SIG) → MotionAO
 *      '?' sorgusu   → QEvt(GCODE_STATUS_SIG)  → MotionAO
 *      Tanınmayan    → CliAO'ya yönlendirilir (diagnostik)
 *   4. MotionAO'dan GCODE_RSP_SIG alınca PC'ye (USB CDC) yazar.
 *
 * Öncelik: 4 (en yüksek — USB karakterlerini hızlı tüketmek için)
 */

#include "qpc.h"

extern QActive * const AO_GCode;  /* global AO pointer */

void GCode_ctor(void);             /* constructor — main.c'den çağrılır */

#if defined(__cplusplus)
}
#endif
