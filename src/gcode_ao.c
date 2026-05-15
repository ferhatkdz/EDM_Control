/*
 * gcode_ao.c — GCodeAO
 *
 * USB VCOM karakterlerini satır tamponunda biriktir, '\n'/'\r' alınca
 * GCode_Parse() ile çöz ve uygun AO'ya ilet.
 *
 *  UART_RX_SIG → karakter biriktir
 *       '\n'   → GCode_Parse()
 *                  ├─ is_status  → GCODE_STATUS_SIG → MotionAO
 *                  ├─ geçerli   → GCODE_CMD_SIG    → MotionAO  (GCodeEvt)
 *                  └─ geçersiz  → "error:20\n" → PC
 *  GCODE_RSP_SIG → GCodeRspEvt.msg → BSP_cli_transmit (USB CDC)
 */

#include "gcode_ao.h"
#include "motion_ao.h"
#include "events.h"
#include "gcode.h"
#include "bsp.h"
#include "cli.h"   /* UartEvt tanımı */

Q_DEFINE_THIS_FILE

/* ------------------------------------------------------------------ */
/*  Sabitler                                                           */
/* ------------------------------------------------------------------ */
#define GCODE_BUF_LEN   96U   /* maksimum satır uzunluğu              */

/* ------------------------------------------------------------------ */
/*  GCodeAO struct                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    QActive super;              /* QActive tabanı — İLK üye olmalı     */

    char    buf[GCODE_BUF_LEN]; /* satır tamponu                       */
    uint8_t buf_len;            /* tampondaki karakter sayısı          */
} GCodeAO;

/* ------------------------------------------------------------------ */
/*  Statik AO örneği + global pointer                                 */
/* ------------------------------------------------------------------ */
static GCodeAO l_gcode;
QActive * const AO_GCode = &l_gcode.super;

/* ------------------------------------------------------------------ */
/*  İleri bildiriler                                                   */
/* ------------------------------------------------------------------ */
static QState GCodeAO_initial (GCodeAO *me, QEvt const *e);
static QState GCodeAO_running (GCodeAO *me, QEvt const *e);

/* ------------------------------------------------------------------ */
/*  Constructor                                                        */
/* ------------------------------------------------------------------ */
void GCode_ctor(void)
{
    GCodeAO *me = &l_gcode;
    QActive_ctor(&me->super, Q_STATE_CAST(GCodeAO_initial));
    me->buf_len = 0U;
}

/* ------------------------------------------------------------------ */
/*  Initial pseudo-state                                               */
/* ------------------------------------------------------------------ */
static QState GCodeAO_initial(GCodeAO *me, QEvt const *e)
{
    (void)e;
    me->buf_len = 0U;
    return Q_TRAN(&GCodeAO_running);
}

/* ------------------------------------------------------------------ */
/*  running state — tek kalıcı durum                                  */
/* ------------------------------------------------------------------ */
static QState GCodeAO_running(GCodeAO *me, QEvt const *e)
{
    QState status;

    switch (e->sig) {

        /* ---------------------------------------------------------- */
        case UART_RX_SIG: {
            /*
             * BSP, USB CDC karakterlerini tek tek UartEvt içinde
             * gönderir. cli.h'da tanımlı UartEvt kullanılır.
             */
            char ch = (char)((UartEvt const *)e)->ch;

            /* Satır sonu → parse */
            if (ch == '\n' || ch == '\r') {
                if (me->buf_len > 0U) {
                    me->buf[me->buf_len] = '\0';

                    /* Önce '?' inline sorgusu var mı? */
                    bool has_status = false;
                    for (uint8_t i = 0U; i < me->buf_len; i++) {
                        if (me->buf[i] == '?') { has_status = true; break; }
                    }

                    if (has_status) {
                        /* Sadece status sinyali gönder */
                        QEvt *pe = Q_NEW(QEvt, GCODE_STATUS_SIG);
                        QACTIVE_POST(AO_Motion, pe, me);
                    }

                    /* Normal parse (? bile olsa G-kodu da olabilir) */
                    GCodeCmd_t cmd;
                    if (GCode_Parse(me->buf, &cmd) && !cmd.is_status) {
                        GCodeEvt *ge = Q_NEW(GCodeEvt, GCODE_CMD_SIG);
                        ge->x       = cmd.x;
                        ge->y       = cmd.y;
                        ge->z       = cmd.z;
                        ge->w       = cmd.w;
                        ge->f       = cmd.f;
                        ge->gcode   = cmd.gcode;
                        ge->mcode   = cmd.mcode;
                        ge->has_x   = cmd.has_x;
                        ge->has_y   = cmd.has_y;
                        ge->has_z   = cmd.has_z;
                        ge->has_w   = cmd.has_w;
                        ge->is_home       = cmd.is_home;
                        ge->is_probe      = cmd.is_probe;
                        ge->is_probe_zero = cmd.is_probe_zero;
                        QACTIVE_POST(AO_Motion, &ge->super, me);
                    } else if (!has_status && me->buf_len > 0U) {
                        /* Tanınmayan komut */
                        BSP_cli_transmit("error:20\n", 9);
                    }

                    me->buf_len = 0U;
                }
            } else {
                /* Tampon dolmadıysa ekle */
                if (me->buf_len < (GCODE_BUF_LEN - 1U)) {
                    me->buf[me->buf_len++] = ch;
                }
                /* Dolu ise karakteri at (overrun koruması) */
            }
            status = Q_HANDLED();
            break;
        }

        /* ---------------------------------------------------------- */
        case GCODE_RSP_SIG: {
            /* MotionAO'dan gelen cevabı USB CDC'ye yaz */
            GCodeRspEvt const *rsp = (GCodeRspEvt const *)e;
            BSP_cli_transmit((char *)rsp->msg, (int)rsp->len);
            status = Q_HANDLED();
            break;
        }

        /* ---------------------------------------------------------- */
        default:
            status = Q_SUPER(&QHsm_top);
            break;
    }

    return status;
}
