#pragma once

#include "qpc.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*..........................................................................*/
typedef struct {     		/* the Motor active object */
	QActive super;   		/* inherit QActive */
	QTimeEvt 	te;
} Motor;

/* hierarchical state machine ... */
QState Motor_initial(Motor * const me, QEvt const * const e);
QState Motor_idle(Motor * const me, QEvt const * const e);

void Motor_ctor(void);
extern QActive * const AO_Motor; /* opaque pointer */


#if defined(__cplusplus)
}
#endif

