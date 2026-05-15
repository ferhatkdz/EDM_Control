#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * motion_ao.h — MotionAO: G-code hareket koordinatörü
 *
 * Durum makinesi:
 *   IDLE → XY_MOVING → W_MOVING → Z_ARK → IDLE
 *
 * Geçiş kuralları:
 *   • XY yok → XY_MOVING atlanır
 *   • W  yok → W_MOVING  atlanır
 *   • Z  yok → Z_ARK     atlanır
 *
 * Faz 2'de AxisCommAO stub anında AXIS_XY_DONE_SIG gönderir.
 *
 * Öncelik: 3
 */

#include "qpc.h"

extern QActive * const AO_Motion;  /* global AO pointer */

void Motion_ctor(void);             /* constructor — main.c'den çağrılır */

#if defined(__cplusplus)
}
#endif
