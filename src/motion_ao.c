/*
 * motion_ao.c — MotionAO: G-code hareket koordinatörü
 *
 * Durum geçişleri:
 *   IDLE ──GCODE_CMD_SIG──► XY_MOVING ──AXIS_XY_DONE_SIG──► W_MOVING
 *        ──GCODE_STATUS_SIG (her durumdan cevapla)
 *
 *   W_MOVING: 20ms QTimeEvt ile Axis_IsAtTarget(W) polling
 *             → true ise Z_ARK'a geç
 *
 *   Z_ARK: 20ms polling Ark_GetState() == ARK_REACHED / ARK_OFF
 *          → IDLE'a dön, "ok\n" gönder
 *
 * Herhangi bir durumdan:
 *   AXIS_XY_ERROR_SIG → acil dur, "error:1\n" gönder, IDLE
 */

#include "motion_ao.h"
#include "axis_comm_ao.h"
#include "gcode_ao.h"
#include "events.h"
#include "gcode.h"
#include "bsp.h"
#include "axis.h"
#include "ark.h"
#include <math.h>   /* isnanf */
#include <string.h>
#include <stdio.h>

Q_DEFINE_THIS_FILE

/* ------------------------------------------------------------------ */
/*  Sabitler                                                           */
/* ------------------------------------------------------------------ */
#define MOTION_TICK_MS      20U      /* polling periyodu (ms)             */
#define MOTION_TICK_TICKS   ((BSP_TICKS_PER_SEC * MOTION_TICK_MS) / 1000U)
#define W_POS_TOL_COUNTS    100      /* W ekseninde "hedefe ulaştı" eşiği  */
#define XY_COUNTS_PER_MM    11000.0f /* Faz 3'te Axis CPU konfigürasyon.  */

/* ------------------------------------------------------------------ */
/*  MotionAO struct                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    QActive   super;        /* QActive tabanı — İLK üye                */
    QTimeEvt  tick_te;      /* 20ms polling timer                      */

    /* Aktif komut — IDLE dışında geçerli */
    GCodeCmd_t cmd;         /* kopyası (GCodeEvt serbest bırakılabilir) */

    /* Durum metin ID'si (status sorgusu için) */
    const char *state_str;
} MotionAO;

/* ------------------------------------------------------------------ */
/*  Statik AO örneği + global pointer                                 */
/* ------------------------------------------------------------------ */
static MotionAO l_motion;
QActive * const AO_Motion = &l_motion.super;

/* ------------------------------------------------------------------ */
/*  İleri bildiriler                                                   */
/* ------------------------------------------------------------------ */
static QState MotionAO_initial   (MotionAO *me, QEvt const *e);
static QState MotionAO_idle      (MotionAO *me, QEvt const *e);
static QState MotionAO_xy_moving (MotionAO *me, QEvt const *e);
static QState MotionAO_w_moving  (MotionAO *me, QEvt const *e);
static QState MotionAO_z_ark     (MotionAO *me, QEvt const *e);

/* ------------------------------------------------------------------ */
/*  Yardımcı: status cevabı gönder                                    */
/* ------------------------------------------------------------------ */
static void send_status(MotionAO *me)
{
    GCodeRspEvt *rsp = Q_NEW(GCodeRspEvt, GCODE_RSP_SIG);

    /* Pozisyonlar counts → mm */
    float z_mm = (float)g_axes[AXIS_Z].hw->get_pos() /
                  (float)g_axes[AXIS_Z].cfg->counts_per_mm;
    float w_mm = (float)g_axes[AXIS_W].hw->get_pos() /
                  (float)g_axes[AXIS_W].cfg->counts_per_mm;

    GCode_FormatStatus(rsp->msg, (uint8_t)sizeof(rsp->msg),
                       me->state_str,
                       0.0f, 0.0f,  /* X, Y — Axis CPU'dan gelecek (Faz 3) */
                       z_mm, w_mm);
    rsp->len = (uint8_t)strlen(rsp->msg);
    QACTIVE_POST(AO_GCode, &rsp->super, me);
}

/* ------------------------------------------------------------------ */
/*  Yardımcı: PC'ye kısa metin gönder                                 */
/* ------------------------------------------------------------------ */
static void send_str(MotionAO *me, const char *s)
{
    GCodeRspEvt *rsp = Q_NEW(GCodeRspEvt, GCODE_RSP_SIG);
    uint8_t len = 0U;
    while (s[len] && len < (sizeof(rsp->msg) - 1U)) {
        rsp->msg[len] = s[len]; len++;
    }
    rsp->msg[len] = '\0';
    rsp->len = len;
    QACTIVE_POST(AO_GCode, &rsp->super, me);
}

/* ------------------------------------------------------------------ */
/*  Yardımcı: bir sonraki fazı başlat veya IDLE'a dön                 */
/* ------------------------------------------------------------------ */

/* Yalnızca W fazını başlat */
static QState start_w_moving(MotionAO *me)
{
    if (me->cmd.has_w) {
        int32_t target = (int32_t)(me->cmd.w *
                          (float)g_axes[AXIS_W].cfg->counts_per_mm);
        Axis_MoveToPosition(&g_axes[AXIS_W], target);
        me->state_str = "Run";
        QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
        return Q_TRAN(&MotionAO_w_moving);
    }
    /* W yok — Z'ye atla */
    return Q_TRAN(&MotionAO_z_ark);
}

/* Z fazını başlat */
static QState start_z_ark(MotionAO *me)
{
    if (me->cmd.has_z) {
        Ark_StartDrill(me->cmd.z);   /* mm cinsinden — ark.h API */
        me->state_str = "Run";
        QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
        return Q_TRAN(&MotionAO_z_ark);
    }
    /* Z yok — bitti */
    send_str(me, "ok\n");
    me->state_str = "Idle";
    return Q_TRAN(&MotionAO_idle);
}

/* ------------------------------------------------------------------ */
/*  Constructor                                                        */
/* ------------------------------------------------------------------ */
void Motion_ctor(void)
{
    MotionAO *me = &l_motion;
    QActive_ctor(&me->super, Q_STATE_CAST(MotionAO_initial));
    QTimeEvt_ctorX(&me->tick_te, &me->super, MOTION_TICK_SIG, 0U);
    me->state_str = "Idle";
}

/* ------------------------------------------------------------------ */
/*  Initial pseudo-state                                               */
/* ------------------------------------------------------------------ */
static QState MotionAO_initial(MotionAO *me, QEvt const *e)
{
    (void)e;
    me->state_str = "Idle";
    return Q_TRAN(&MotionAO_idle);
}

/* ------------------------------------------------------------------ */
/*  idle                                                               */
/* ------------------------------------------------------------------ */
static QState MotionAO_idle(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Idle";
            status = Q_HANDLED();
            break;

        case GCODE_CMD_SIG: {
            GCodeEvt const *ge = (GCodeEvt const *)e;

            /* Komutu kaydet */
            me->cmd.x       = ge->x;
            me->cmd.y       = ge->y;
            me->cmd.z       = ge->z;
            me->cmd.w       = ge->w;
            me->cmd.f       = ge->f;
            me->cmd.gcode   = ge->gcode;
            me->cmd.mcode   = ge->mcode;
            me->cmd.has_x   = ge->has_x;
            me->cmd.has_y   = ge->has_y;
            me->cmd.has_z   = ge->has_z;
            me->cmd.has_w   = ge->has_w;
            me->cmd.is_home = ge->is_home;

            /* M kodu işle */
            if (me->cmd.mcode == 3U) { Ark_Enable(true);  }
            if (me->cmd.mcode == 5U) { Ark_Enable(false); }

            /* G28 home */
            if (me->cmd.is_home) {
                /* TODO Faz 4: home sequence */
                send_str(me, "ok\n");
                status = Q_HANDLED();
                break;
            }

            /* Hiç eksen hareketi yok mu? */
            if (!me->cmd.has_x && !me->cmd.has_y &&
                !me->cmd.has_z && !me->cmd.has_w) {
                send_str(me, "ok\n");
                status = Q_HANDLED();
                break;
            }

            /* X/Y hareketi var mı? */
            if (me->cmd.has_x || me->cmd.has_y) {
                /* AxisCommAO'ya komut gönder */
                AxisCommCmdEvt *ce = Q_NEW(AxisCommCmdEvt, AXIS_COMM_CMD_SIG);
                ce->x_counts = me->cmd.has_x
                    ? (int32_t)(me->cmd.x * XY_COUNTS_PER_MM) : INT32_MIN;
                ce->y_counts = me->cmd.has_y
                    ? (int32_t)(me->cmd.y * XY_COUNTS_PER_MM) : INT32_MIN;
                ce->has_x = me->cmd.has_x;
                ce->has_y = me->cmd.has_y;
                QACTIVE_POST(AO_AxisComm, &ce->super, me);
                me->state_str = "Run";
                status = Q_TRAN(&MotionAO_xy_moving);
            } else {
                /* X/Y yok — W'ya geç */
                status = start_w_moving(me);
            }
            break;
        }

        case GCODE_STATUS_SIG:
            send_status(me);
            status = Q_HANDLED();
            break;

        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}

/* ------------------------------------------------------------------ */
/*  xy_moving                                                          */
/* ------------------------------------------------------------------ */
static QState MotionAO_xy_moving(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Run";
            status = Q_HANDLED();
            break;

        case AXIS_XY_DONE_SIG:
            /* X/Y tamamlandı, W'ya geç */
            status = start_w_moving(me);
            break;

        case AXIS_XY_ERROR_SIG:
            /* Axis CPU hatası veya timeout */
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "error:1\n");
            status = Q_TRAN(&MotionAO_idle);
            break;

        case GCODE_STATUS_SIG:
            send_status(me);
            status = Q_HANDLED();
            break;

        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}

/* ------------------------------------------------------------------ */
/*  w_moving                                                           */
/* ------------------------------------------------------------------ */
static QState MotionAO_w_moving(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Run";
            status = Q_HANDLED();
            break;

        case Q_EXIT_SIG:
            QTimeEvt_disarm(&me->tick_te);
            status = Q_HANDLED();
            break;

        case MOTION_TICK_SIG:
            if (Axis_IsAtTarget(&g_axes[AXIS_W], W_POS_TOL_COUNTS)) {
                status = start_z_ark(me);
            } else {
                status = Q_HANDLED();
            }
            break;

        case AXIS_XY_ERROR_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "error:1\n");
            status = Q_TRAN(&MotionAO_idle);
            break;

        case GCODE_STATUS_SIG:
            send_status(me);
            status = Q_HANDLED();
            break;

        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}

/* ------------------------------------------------------------------ */
/*  z_ark                                                              */
/* ------------------------------------------------------------------ */
static QState MotionAO_z_ark(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Run";
            status = Q_HANDLED();
            break;

        case Q_EXIT_SIG:
            QTimeEvt_disarm(&me->tick_te);
            status = Q_HANDLED();
            break;

        case MOTION_TICK_SIG: {
            ArkState_e ark_st = Ark_GetState();
            if (ark_st == ARK_REACHED || ark_st == ARK_OFF) {
                send_str(me, "ok\n");
                me->state_str = "Idle";
                status = Q_TRAN(&MotionAO_idle);
            } else {
                status = Q_HANDLED();
            }
            break;
        }

        case AXIS_XY_ERROR_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "error:1\n");
            status = Q_TRAN(&MotionAO_idle);
            break;

        case GCODE_STATUS_SIG:
            send_status(me);
            status = Q_HANDLED();
            break;

        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}
