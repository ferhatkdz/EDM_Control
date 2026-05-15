#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * axis_comm_ao.h — AxisCommAO: Axis CPU UART haberleşmesi
 *
 * Faz 2 (stub): AXIS_COMM_CMD_SIG alındığında anında
 *               AXIS_XY_DONE_SIG → MotionAO gönderir.
 *
 * Faz 3: Gerçek UART1 protokolü
 *   Main → Axis: "M X+001234 Y+005678\n"
 *   Axis → Main: "OK\n" → "DONE\n"
 *   Timeout: 30 saniye
 *
 * Öncelik: 2
 */

#include "qpc.h"

extern QActive * const AO_AxisComm;  /* global AO pointer */

void AxisComm_ctor(void);             /* constructor — main.c'den çağrılır */

#if defined(__cplusplus)
}
#endif
