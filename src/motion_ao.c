/*
 * motion_ao.c — MotionAO: G-code hareket koordinatörü
 *
 * Durum geçişleri:
 *   IDLE ──GCODE_CMD_SIG──► XY_MOVING ──AXIS_XY_DONE_SIG──► W_MOVING
 *        ──GCODE_STATUS_SIG   (her durumdan cevapla)
 *        ──GCODE_FEED_HOLD_SIG (her durumdan: dur, IDLE'a dön)
 *
 *   W_MOVING: 20ms QTimeEvt ile Axis_IsAtTarget(W) polling
 *             → true ise Z_MOVING veya Z_ARK'a geç
 *
 *   Z_MOVING: Ark OFF iken direkt pozisyon hareketi (W ile simetrik)
 *             20ms polling Axis_IsAtTarget(Z) → IDLE'a dön, "ok\n"
 *
 *   Z_ARK: Ark aktif (M3) → servo delme modu
 *          20ms polling Ark_GetState() == ARK_REACHED / ARK_OFF
 *          → IDLE'a dön, "ok\n" gönder
 *
 *   PROBING: 20ms polling Probe_GetState() == PROBE_DONE / PROBE_ERROR
 *            → IDLE'a dön, "[PRB:...]\nok\n" veya "error:8\n" gönder
 *
 * Herhangi bir durumdan:
 *   AXIS_XY_ERROR_SIG → acil dur, "error:1\n" gönder, IDLE
 *
 * G0/G1 ayrımı (W ekseni):
 *   G0: Axis_SetMaxVelocity ile varsayılan maks hız
 *   G1 Fxxx: F parametresinden cps hesaplanır, W hareketi başlamadan uygulanır
 */

#include "motion_ao.h"
#include "axis_comm_ao.h"
#include "gcode_ao.h"
#include "events.h"
#include "gcode.h"
#include "bsp.h"
#include "axis.h"
#include "ark.h"
#include "probe.h"
#include <math.h>   /* isnanf */
#include <string.h>
#include <stdio.h>

extern volatile uint32_t g_u32FilteredGapVoltage;
extern volatile uint32_t g_u32FilteredSparkCurrent;

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
static QState MotionAO_z_moving  (MotionAO *me, QEvt const *e);
static QState MotionAO_z_ark     (MotionAO *me, QEvt const *e);
static QState MotionAO_probing   (MotionAO *me, QEvt const *e);

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

    snprintf(rsp->msg, sizeof(rsp->msg),
             "<%.4s|MPos:%.3f,%.3f,%.3f,%.3f|FS:0,0|GV:%u|SC:%u>\n",
             me->state_str ? me->state_str : "Idle",
             0.0, 0.0, (double)z_mm, (double)w_mm,
             (unsigned)g_u32FilteredGapVoltage,
             (unsigned)g_u32FilteredSparkCurrent);
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


/* Z fazını başlat */
static QState start_z_ark(MotionAO *me)
{
    if (me->cmd.has_z) {
        if (Ark_GetState() == ARK_OFF) {
            /* Ark kapalı → W gibi direkt pozisyon hareketi */
            int32_t target = (int32_t)(me->cmd.z *
                              (float)g_axes[AXIS_Z].cfg->counts_per_mm);
            if (me->cmd.gcode == 1U && me->cmd.f > 0.0f) {
                int32_t f_cps = (int32_t)((me->cmd.f / 60.0f) *
                                 (float)g_axes[AXIS_Z].cfg->counts_per_mm);
                Axis_SetMaxVelocity(&g_axes[AXIS_Z], f_cps);
            } else {
                Axis_SetMaxVelocity(&g_axes[AXIS_Z],
                                    (int32_t)g_axes[AXIS_Z].cfg->pos_out_max);
            }
            Axis_MoveToPosition(&g_axes[AXIS_Z], target);
            me->state_str = "Run";
            QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
            return Q_TRAN(&MotionAO_z_moving);
        }
        /* Ark aktif → servo delme modu
         * me->cmd.z mutlak koordinat; Ark_StartDrill göreceli pozitif derinlik bekler */
        float cur_mm = (float)g_axes[AXIS_Z].hw->get_pos() /
                       (float)g_axes[AXIS_Z].cfg->counts_per_mm;
        float depth_mm = cur_mm - me->cmd.z;   /* ör: 0 - (-1.0) = 1.0 mm */
        Ark_StartDrill(depth_mm);
        me->state_str = "Run";
        QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
        return Q_TRAN(&MotionAO_z_ark);
    }
    /* Z yok — bitti */
    send_str(me, "ok\n");
    me->state_str = "Idle";
    return Q_TRAN(&MotionAO_idle);
}

/* Yalnızca W fazını başlat */
static QState start_w_moving(MotionAO *me)
{
    if (me->cmd.has_w) {
        int32_t target = (int32_t)(me->cmd.w *
                          (float)g_axes[AXIS_W].cfg->counts_per_mm);

        /* G0/G1 hız seçimi */
        if (me->cmd.gcode == 1U && me->cmd.f > 0.0f) {
            /* G1: belirtilen besleme hızı (mm/dak → cps) */
            int32_t f_cps = (int32_t)((me->cmd.f / 60.0f) *
                             (float)g_axes[AXIS_W].cfg->counts_per_mm);
            Axis_SetMaxVelocity(&g_axes[AXIS_W], f_cps);
        } else {
            /* G0: varsayılan maksimum hız */
            Axis_SetMaxVelocity(&g_axes[AXIS_W],
                                (int32_t)g_axes[AXIS_W].cfg->pos_out_max);
        }

        Axis_MoveToPosition(&g_axes[AXIS_W], target);
        me->state_str = "Run";
        QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
        return Q_TRAN(&MotionAO_w_moving);
    }
    /* W yok — Z fazını başlat (start_z_ark Ark OFF/ON ayrımını yapar) */
    return start_z_ark(me);
}


/* Probe fazını başlat (G38.2 / G38.3) */
static QState start_probing(MotionAO *me)
{
    AxisId_e    axis;
    float       target_mm;
    ProbeMode_e mode = me->cmd.is_probe_zero ? PROBE_MODE_ZERO : PROBE_MODE_MEASURE;

    if (me->cmd.has_z)      { axis = AXIS_Z; target_mm = me->cmd.z; }
    else if (me->cmd.has_w) { axis = AXIS_W; target_mm = me->cmd.w; }
    else {
        send_str(me, "error:8\n");
        return Q_TRAN(&MotionAO_idle);
    }

    /* Spark açıkken probe yasak */
    if (Ark_GetState() != ARK_OFF) {
        send_str(me, "error:8\n");
        return Q_TRAN(&MotionAO_idle);
    }

    /* F parametresi → cps; varsayılan 100 mm/dak */
    float f   = (me->cmd.f > 0.0f) ? me->cmd.f : 100.0f;
    int32_t cps = (int32_t)((f / 60.0f) *
                             (float)g_axes[axis].cfg->counts_per_mm);
    /* Negatif hedef = iş parçasına doğru hareket */
    if (target_mm < 0.0f) { cps = -cps; }

    Probe_Start(axis, cps, target_mm, mode);
    me->state_str = "Run";
    QTimeEvt_armX(&me->tick_te, MOTION_TICK_TICKS, MOTION_TICK_TICKS);
    return Q_TRAN(&MotionAO_probing);
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
            me->cmd.x            = ge->x;
            me->cmd.y            = ge->y;
            me->cmd.z            = ge->z;
            me->cmd.w            = ge->w;
            me->cmd.f            = ge->f;
            me->cmd.p            = ge->p;
            me->cmd.gcode        = ge->gcode;
            me->cmd.mcode        = ge->mcode;
            me->cmd.has_x        = ge->has_x;
            me->cmd.has_y        = ge->has_y;
            me->cmd.has_z        = ge->has_z;
            me->cmd.has_w        = ge->has_w;
            me->cmd.has_p        = ge->has_p;
            me->cmd.is_home      = ge->is_home;
            me->cmd.is_probe     = ge->is_probe;
            me->cmd.is_probe_zero = ge->is_probe_zero;

            /* M kodu işle */
            if (me->cmd.mcode == 3U) { Ark_Enable(true);  }
            if (me->cmd.mcode == 5U) { Ark_Enable(false); }
            if (me->cmd.mcode == 100U && me->cmd.has_p)
                Ark_SetSparkPwmNormal((uint32_t)me->cmd.p);
            if (me->cmd.mcode == 101U && me->cmd.has_p)
                Ark_SetSparkPwmShort((uint32_t)me->cmd.p);
            if (me->cmd.mcode == 102U && me->cmd.has_p)
                Ark_SetPower((uint8_t)me->cmd.p);

            /* G28 home */
            if (me->cmd.is_home) {
                /* TODO Faz 4: home sequence */
                send_str(me, "ok\n");
                status = Q_HANDLED();
                break;
            }

            /* G38.2 / G38.3 probe */
            if (me->cmd.is_probe) {
                status = start_probing(me);
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

        case GCODE_FEED_HOLD_SIG:
            send_str(me, "ok\n");
            status = Q_HANDLED();
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

        case GCODE_FEED_HOLD_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "ok\n");
            me->state_str = "Idle";
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
            /* G1 ile kısıtlanan hız limitini geri yükle */
            Axis_SetMaxVelocity(&g_axes[AXIS_W],
                                (int32_t)g_axes[AXIS_W].cfg->pos_out_max);
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

        case GCODE_FEED_HOLD_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "ok\n");
            me->state_str = "Idle";
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
/*  z_moving — Ark OFF iken direkt Z pozisyon hareketi (W ile simetrik) */
/* ------------------------------------------------------------------ */
static QState MotionAO_z_moving(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Run";
            status = Q_HANDLED();
            break;

        case Q_EXIT_SIG:
            QTimeEvt_disarm(&me->tick_te);
            Axis_SetMaxVelocity(&g_axes[AXIS_Z],
                                (int32_t)g_axes[AXIS_Z].cfg->pos_out_max);
            status = Q_HANDLED();
            break;

        case MOTION_TICK_SIG:
            if (Axis_IsAtTarget(&g_axes[AXIS_Z], W_POS_TOL_COUNTS)) {
                send_str(me, "ok\n");
                me->state_str = "Idle";
                status = Q_TRAN(&MotionAO_idle);
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

        case GCODE_FEED_HOLD_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            send_str(me, "ok\n");
            me->state_str = "Idle";
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

        case GCODE_FEED_HOLD_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            Ark_Enable(false);
            send_str(me, "ok\n");
            me->state_str = "Idle";
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
/*  probing — G38.2 / G38.3 kenar bulma                               */
/* ------------------------------------------------------------------ */
static QState MotionAO_probing(MotionAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        case Q_ENTRY_SIG:
            me->state_str = "Run";
            status = Q_HANDLED();
            break;

        case Q_EXIT_SIG:
            QTimeEvt_disarm(&me->tick_te);
            Probe_Reset();
            status = Q_HANDLED();
            break;

        case MOTION_TICK_SIG: {
            ProbeState_e ps = Probe_GetState();

            if (ps == PROBE_DONE || ps == PROBE_ERROR) {
                GCodeRspEvt *rsp = Q_NEW(GCodeRspEvt, GCODE_RSP_SIG);
                float cm    = Probe_GetContactMm();
                float z_prb = (Probe_GetAxis() == AXIS_Z) ? cm : 0.0f;
                float w_prb = (Probe_GetAxis() == AXIS_W) ? cm : 0.0f;
                int   flag  = Probe_GetSuccess() ? 1 : 0;

                snprintf(rsp->msg, sizeof(rsp->msg),
                         "[PRB:0.000,0.000,%.3f,%.3f:%d]\n%s",
                         (double)z_prb, (double)w_prb, flag,
                         flag ? "ok\n" : "error:8\n");
                rsp->len = (uint8_t)strlen(rsp->msg);
                QACTIVE_POST(AO_GCode, &rsp->super, me);

                Probe_Reset();
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
            Probe_Reset();
            send_str(me, "error:1\n");
            status = Q_TRAN(&MotionAO_idle);
            break;

        case GCODE_FEED_HOLD_SIG:
            Axis_EmergencyStop(&g_axes[AXIS_Z]);
            Axis_EmergencyStop(&g_axes[AXIS_W]);
            Probe_Reset();
            send_str(me, "ok\n");
            me->state_str = "Idle";
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
