#pragma once

#include "qpc.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define CLI_BUF_SIZE 64

/*..........................................................................*/

/* Led Event class */
typedef struct {
  QEvt super; /* inherit Event */
  uint8_t ch;
} UartEvt;

typedef struct { /* the active object */
  QActive super; /* inherit QActive */
  char buf[CLI_BUF_SIZE];
  uint8_t idx;
} Cli;

/* hierarchical state machine ... */
QState Cli_initial(Cli* const me, QEvt const* const e);
QState Cli_idle(Cli* const me, QEvt const* const e);

void Cli_ctor(void);
extern QActive* const AO_Cli; /* opaque pointer */

#if defined(__cplusplus)
}
#endif
