/*
 * axis_comm_ao.c — AxisCommAO
 *
 * FAZ 2 STUB: AXIS_COMM_CMD_SIG alındığında anında AXIS_XY_DONE_SIG
 * gönderir. Gerçek UART donanımı yoktur.
 *
 * Faz 3'te bu dosya gerçek UART1 protokolü ile değiştirilecek:
 *   IDLE ──AXIS_COMM_CMD_SIG──► CMD_SENT (UART'a yaz, 30s timeout)
 *                                  │ "OK\n"
 *                               WAIT_DONE
 *                                  │ "DONE\n"
 *                               → AXIS_XY_DONE_SIG → MotionAO
 *                               timeout / "ER\n"
 *                               → AXIS_XY_ERROR_SIG → MotionAO
 */

#include "axis_comm_ao.h"
#include "motion_ao.h"
#include "events.h"
#include "bsp.h"

Q_DEFINE_THIS_FILE

/* ------------------------------------------------------------------ */
/*  AxisCommAO struct                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    QActive super;   /* QActive tabanı — İLK üye */
} AxisCommAO;

/* ------------------------------------------------------------------ */
/*  Statik AO örneği + global pointer                                 */
/* ------------------------------------------------------------------ */
static AxisCommAO l_axis_comm;
QActive * const AO_AxisComm = &l_axis_comm.super;

/* ------------------------------------------------------------------ */
/*  İleri bildiriler                                                   */
/* ------------------------------------------------------------------ */
static QState AxisCommAO_initial (AxisCommAO *me, QEvt const *e);
static QState AxisCommAO_idle    (AxisCommAO *me, QEvt const *e);

/* ------------------------------------------------------------------ */
/*  Constructor                                                        */
/* ------------------------------------------------------------------ */
void AxisComm_ctor(void)
{
    QActive_ctor(&l_axis_comm.super, Q_STATE_CAST(AxisCommAO_initial));
}

/* ------------------------------------------------------------------ */
/*  Initial pseudo-state                                               */
/* ------------------------------------------------------------------ */
static QState AxisCommAO_initial(AxisCommAO *me, QEvt const *e)
{
    (void)e;
    return Q_TRAN(&AxisCommAO_idle);
}

/* ------------------------------------------------------------------ */
/*  idle — Faz 2 stub                                                  */
/* ------------------------------------------------------------------ */
static QState AxisCommAO_idle(AxisCommAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case AXIS_COMM_CMD_SIG: {
            /*
             * Faz 2: Axis CPU yok → anında DONE gönder.
             * Faz 3'te: UART1'e "M X.. Y..\n" yaz, "OK\n" + "DONE\n" bekle.
             */
            QEvt *done = Q_NEW(QEvt, AXIS_XY_DONE_SIG);
            QACTIVE_POST(AO_Motion, done, me);
            status = Q_HANDLED();
            break;
        }

        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}
