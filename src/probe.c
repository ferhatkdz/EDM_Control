/*
 * probe.c — G38.2 / G38.3 kenar bulma (probing) modülü
 *
 * Eksen bağımsız: AXIS_Z veya AXIS_W ile çalışır.
 * Temas tespiti: g_u32FilteredGapVoltage < PROBE_GAP_SHORT_ADC
 * EADC01_IRQHandler'dan 5 kHz'de çağrılır; iç /5 prescaler → 1 kHz.
 * QPC bağımlılığı yok.
 */

#include "probe.h"
#include "axis.h"

/* ------------------------------------------------------------------ */
/*  Sabitler                                                           */
/* ------------------------------------------------------------------ */
#define PROBE_GAP_SHORT_ADC  20U      /* Temas eşiği (ADC count)        */
#define PROBE_RETRACT_MM     1.0f     /* Temas sonrası geri çekme (mm)  */
#define PROBE_TOL_COUNTS     100      /* Hedefe ulaşma toleransı        */
#define PROBE_TICK_PRE       5U       /* ISR prescaler: 5 kHz → 1 kHz  */

/* ------------------------------------------------------------------ */
/*  Extern (bsp.c'de EADC01 ISR günceller)                            */
/* ------------------------------------------------------------------ */
extern volatile uint32_t g_u32FilteredGapVoltage;

/* ------------------------------------------------------------------ */
/*  Modül durumu                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile ProbeState_e state;
    AxisId_e    axis;
    ProbeMode_e mode;
    int32_t     approach_cps;   /* işaretli: negatif = iş parçasına doğru */
    int32_t     target_count;   /* seyahat limiti (encoder counts)         */
    float       contact_mm;     /* temas anındaki pozisyon (mm)            */
    int32_t     retract_target; /* geri çekme hedefi (encoder counts)      */
    bool        success;
    uint32_t    tick_div;
} Probe_t;

static Probe_t s_probe = { .state = PROBE_IDLE };

/* ------------------------------------------------------------------ */
/*  Probe_Start                                                        */
/* ------------------------------------------------------------------ */
void Probe_Start(AxisId_e axis, int32_t approach_cps,
                 float target_mm, ProbeMode_e mode)
{
    if (s_probe.state != PROBE_IDLE) { return; }

    AxisHandle_t *ax = &g_axes[axis];

    /* Önceki feed hold / emergency stop hw->disable() ile PWM'yi
     * maskelemiş olabilir; Axis_UpdateVelocity (Probe_Tick'in kullandığı)
     * hw->enable() çağırmadığı için motor sessizce hareket etmez.
     * Axis_SetVelocity hem hw->enable hem vel PID reset yapar. */
    Axis_SetVelocity(ax, 0);

    s_probe.axis         = axis;
    s_probe.mode         = mode;
    s_probe.approach_cps = approach_cps;
    s_probe.target_count = (int32_t)(target_mm * ax->cfg->counts_per_mm);
    s_probe.contact_mm   = 0.0f;
    s_probe.success      = false;
    s_probe.tick_div     = 0U;
    s_probe.state        = PROBE_SEEKING;
}

/* ------------------------------------------------------------------ */
/*  Probe_Tick — EADC01_IRQHandler'dan çağrılır (5 kHz)               */
/* ------------------------------------------------------------------ */
void Probe_Tick(void)
{
    if (s_probe.state == PROBE_IDLE  ||
        s_probe.state == PROBE_DONE  ||
        s_probe.state == PROBE_ERROR) {
        return;
    }

    /* İç prescaler: 5 kHz → 1 kHz */
    if (++s_probe.tick_div < PROBE_TICK_PRE) { return; }
    s_probe.tick_div = 0U;

    AxisHandle_t *ax = &g_axes[s_probe.axis];
    int32_t  pos = ax->hw->get_pos();
    uint32_t gap = g_u32FilteredGapVoltage;

    /* ---------------------------------- */
    if (s_probe.state == PROBE_SEEKING) {

        /* Seyahat limiti kontrolü */
        bool limit = (s_probe.approach_cps < 0)
                     ? (pos <= s_probe.target_count)
                     : (pos >= s_probe.target_count);

        if (limit) {
            Axis_Stop(ax);
            s_probe.success = false;
            s_probe.state   = PROBE_ERROR;
            return;
        }

        if (gap < PROBE_GAP_SHORT_ADC) {
            /* Temas! */
            Axis_Stop(ax);
            s_probe.success = true;

            int32_t ret_cnt =
                (int32_t)(PROBE_RETRACT_MM * ax->cfg->counts_per_mm);

            if (s_probe.mode == PROBE_MODE_ZERO) {
                /* G38.3: encoder sıfırla → temas = yeni referans noktası */
                s_probe.contact_mm = 0.0f;
                Axis_ResetPos(ax);
                /* Geri çekme: yeni sıfırdan itibaren */
                s_probe.retract_target = (s_probe.approach_cps < 0)
                                         ? +ret_cnt
                                         : -ret_cnt;
            } else {
                /* G38.2: mutlak pozisyon kaydet, sıfırlama yapma */
                s_probe.contact_mm     = (float)pos / ax->cfg->counts_per_mm;
                s_probe.retract_target = (s_probe.approach_cps < 0)
                                         ? pos + ret_cnt
                                         : pos - ret_cnt;
            }

            Axis_MoveToPosition(ax, s_probe.retract_target);
            s_probe.state = PROBE_RETRACTING;
            return;
        }

        /* Temas yok — ilerlemeye devam */
        Axis_UpdateVelocity(ax, s_probe.approach_cps);
        return;
    }

    /* ---------------------------------- */
    if (s_probe.state == PROBE_RETRACTING) {
        if (Axis_IsAtTarget(ax, PROBE_TOL_COUNTS)) {
            s_probe.state = PROBE_DONE;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Getters                                                            */
/* ------------------------------------------------------------------ */
void         Probe_Reset(void)        { s_probe.state = PROBE_IDLE; }
ProbeState_e Probe_GetState(void)     { return s_probe.state; }
float        Probe_GetContactMm(void) { return s_probe.contact_mm; }
bool         Probe_GetSuccess(void)   { return s_probe.success; }
AxisId_e     Probe_GetAxis(void)      { return s_probe.axis; }
