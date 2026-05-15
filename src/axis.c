#include "axis.h"
#include <string.h>

/*-------------------------------------------------
 * Global eksen dizisi
 *------------------------------------------------*/
AxisHandle_t g_axes[AXIS_COUNT];

/*-------------------------------------------------
 * Z ekseni konfigürasyonu — saha testinden tuned
 *
 * Pos loop (1kHz, positional form):
 *   Kp=20, Ki=0 → saf P, overshoot yok, salınım yok.
 *   Encoder entegratör olduğundan steady-state=0.
 *   Ki eklense underdamped salınım çıkar (test doğruladı).
 *   Çıkış: vel_cmd [±100000 cps]
 *
 * Vel loop (5kHz, velocity form):
 *   Kp=0.5, Ki=50 → hızlı tepki, stabil.
 *   Çıkış: duty [±10000]
 *------------------------------------------------*/
static const AxisConfig_t s_z_config = {
    .pos_kp       = 20.0f,
    .pos_ki       = 0.0f,
    .pos_kd       = 0.0f,
    .pos_out_min  = -100000.0f,
    .pos_out_max  =  100000.0f,
    .pos_int_min  = -50000.0f,
    .pos_int_max  =  50000.0f,
    .pos_kff      = 0.0f,
    .pos_d_filter = 0.5f,

    .vel_kp       = 0.5f,
    .vel_ki       = 50.0f,
    .vel_kd       = 0.0f,
    .vel_out_min  = -10000.0f,
    .vel_out_max  =  10000.0f,
    .vel_int_min  = -8000.0f,
    .vel_int_max  =  8000.0f,
    .vel_kff      = 0.0f,
    .vel_d_filter = 0.7f,

    .vel_lpf_alpha            = 0.3f,
    .pwm_freq_hz              = 20000,
    .counts_per_mm            = 11000.0f,   /* 55000 cnt/rev ÷ 5 mm/rev */
    .overcurrent_threshold_adc = 3000,
    .overcurrent_trip_ticks    = 10,        /* 10 tick = 2ms @ 5kHz     */
};

/* W ekseni — başlangıçta Z ile aynı; mekanik test sonrası revize edilir */
static const AxisConfig_t s_w_config = {
    .pos_kp       = 20.0f,
    .pos_ki       = 0.0f,
    .pos_kd       = 0.0f,
    .pos_out_min  = -100000.0f,
    .pos_out_max  =  100000.0f,
    .pos_int_min  = -50000.0f,
    .pos_int_max  =  50000.0f,
    .pos_kff      = 0.0f,
    .pos_d_filter = 0.5f,

    .vel_kp       = 0.5f,
    .vel_ki       = 50.0f,
    .vel_kd       = 0.0f,
    .vel_out_min  = -10000.0f,
    .vel_out_max  =  10000.0f,
    .vel_int_min  = -8000.0f,
    .vel_int_max  =  8000.0f,
    .vel_kff      = 0.0f,
    .vel_d_filter = 0.7f,

    .vel_lpf_alpha            = 0.3f,
    .pwm_freq_hz              = 20000,
    .counts_per_mm            = 11000.0f,
    .overcurrent_threshold_adc = 3000,
    .overcurrent_trip_ticks    = 10,
};

/*-------------------------------------------------
 * Axis_Init
 *------------------------------------------------*/
void Axis_Init(AxisHandle_t *ax, AxisId_e id,
               const AxisHW_t *hw, const AxisConfig_t *cfg)
{
    ax->hw  = hw;
    ax->cfg = cfg;
    ax->id  = id;

    /* Pozisyon PID — positional form, 1kHz */
    PID_Init(&ax->pid_pos,
             cfg->pos_kp, cfg->pos_ki, cfg->pos_kd,
             0.001f,                              /* dt = 1kHz             */
             cfg->pos_out_min, cfg->pos_out_max,
             cfg->pos_int_min, cfg->pos_int_max,
             cfg->pos_kff,
             cfg->pos_d_filter);

    /* Hız PID — velocity form, 5kHz */
    PID_Init(&ax->pid_vel,
             cfg->vel_kp, cfg->vel_ki, cfg->vel_kd,
             0.0002f,                             /* dt = 5kHz             */
             cfg->vel_out_min, cfg->vel_out_max,
             cfg->vel_int_min, cfg->vel_int_max,
             cfg->vel_kff,
             cfg->vel_d_filter);

    /* Hız ölçümü */
    memset(&ax->vel, 0, sizeof(VelMeasure_t));
    ax->vel.lpf_alpha = cfg->vel_lpf_alpha;

    /* Kontrol durumu */
    ax->ctrl_mode        = CTRL_OFF;
    ax->target_pos       = 0;
    ax->target_vel       = 0;
    ax->current_adc      = 0;
    ax->fault_overcurrent = false;
    ax->last_duty        = 0;
    ax->pos_div          = 0;
    ax->overcurrent_count = 0;

    /* Donanım başlat */
    hw->pwm_init(cfg->pwm_freq_hz);
    hw->adc_init();
    hw->qei_init();
    hw->enable();
}

/*-------------------------------------------------
 * AllAxes_Init — QF_onStartup'tan çağrılır
 *------------------------------------------------*/
void AllAxes_Init(void)
{
    Axis_Init(&g_axes[AXIS_Z], AXIS_Z, &g_axis_z_hw, &s_z_config);
    Axis_Init(&g_axes[AXIS_W], AXIS_W, &g_axis_w_hw, &s_w_config);
}

/*-------------------------------------------------
 * AllAxes_ControlTick — TMR2_IRQHandler'dan çağrılır
 *------------------------------------------------*/
void AllAxes_ControlTick(void)
{
    Axis_ControlTick(&g_axes[AXIS_Z]);
    Axis_ControlTick(&g_axes[AXIS_W]);
}

/*-------------------------------------------------
 * Axis_ControlTick — 5kHz, per-axis kontrol döngüsü
 *
 * Sıralama (orijinal motor.c ile aynı):
 *  1. CTRL_FREE_DUTY → hemen dön (vel ölçümü yok)
 *  2. QEI oku + VelMeasure_Update
 *  3. Aşırı akım kontrolü
 *  4. CTRL_OFF → duty=0, dön
 *  5. CTRL_POSITION: pos loop 1kHz (÷5), vel_cmd üret
 *  6. CTRL_VELOCITY: vel_cmd = target_vel
 *  7. Vel PID → duty → set_duty
 *------------------------------------------------*/
void Axis_ControlTick(AxisHandle_t *ax)
{
    /* Açık çevrim modunda PID hesabı yapma */
    if (ax->ctrl_mode == CTRL_FREE_DUTY) {
        return;
    }

    /* QEI oku + hız ölç (CTRL_OFF dahil tüm modlarda) */
    int32_t pos_now = ax->hw->get_pos();
    VelMeasure_Update(&ax->vel, pos_now);

    /* Aşırı akım koruması — sadece motor çalışıyorsa */
    if (ax->ctrl_mode != CTRL_OFF) {
        if (ax->current_adc > ax->cfg->overcurrent_threshold_adc) {
            ax->overcurrent_count++;
            if (ax->overcurrent_count >= ax->cfg->overcurrent_trip_ticks) {
                Axis_EmergencyStop(ax);
                ax->fault_overcurrent = true;
                return;
            }
        } else {
            ax->overcurrent_count = 0;
        }
    }

    /* Kapalı modda duty'yi sıfırla */
    if (ax->ctrl_mode == CTRL_OFF) {
        ax->hw->set_duty(0);
        ax->last_duty = 0;
        return;
    }

    /* Hız komutu üret */
    float vel_cmd = 0.0f;

    if (ax->ctrl_mode == CTRL_POSITION) {
        ax->pos_div++;
        if (ax->pos_div >= 5) {    /* 5kHz ÷ 5 = 1kHz */
            ax->pos_div = 0;

            /* Positional-form: büyük adımda saturasyon sonrası
             * bilgi kaybı olmaz; her tick u = Kp·e + I + FF  */
            float ff = (float)ax->target_vel;
            vel_cmd = PID_Update_Pos(&ax->pid_pos,
                                     (float)ax->target_pos,
                                     (float)pos_now,
                                     ff);
        } else {
            /* 1kHz arası: son pos PID çıkışını koru */
            vel_cmd = ax->pid_pos.output;
        }
    } else {    /* CTRL_VELOCITY */
        vel_cmd = (float)ax->target_vel;
    }

    /* Hız PID → duty */
    float duty_f = PID_Update_Velocity(&ax->pid_vel,
                                       vel_cmd,
                                       ax->vel.velocity_cps,
                                       0.0f);

    int32_t duty  = (int32_t)duty_f;
    ax->last_duty = duty;
    ax->hw->set_duty(duty);
}

/*-------------------------------------------------
 * Hareket komutları
 *------------------------------------------------*/
void Axis_MoveToPosition(AxisHandle_t *ax, int32_t target_count)
{
    /* Önce dur — tutma moduna geç */
    Axis_Stop(ax);

    /* PID birikim temizle */
    PID_Reset(&ax->pid_pos);
    PID_Reset(&ax->pid_vel);
    ax->pid_pos.output = 0.0f;
    ax->pid_vel.output = 0.0f;

    ax->hw->enable();

    ax->target_pos = target_count;
    ax->target_vel = 0;
    ax->ctrl_mode  = CTRL_POSITION;
}

void Axis_MoveWithDuty(AxisHandle_t *ax, int32_t duty)
{
    ax->hw->enable();
    ax->ctrl_mode = CTRL_FREE_DUTY;
    ax->last_duty = duty;
    ax->hw->set_duty(duty);
}

void Axis_SetVelocity(AxisHandle_t *ax, int32_t vel_cps)
{
    ax->hw->enable();

    /* Yeni komutta eski integratör birikimi yeni komutu bozmasın */
    PID_Reset(&ax->pid_vel);
    ax->pid_vel.output = 0.0f;

    ax->target_vel = vel_cps;
    ax->ctrl_mode  = CTRL_VELOCITY;
}

void Axis_UpdateVelocity(AxisHandle_t *ax, int32_t vel_cps)
{
    /* PID state bozmadan sadece target güncelle — ark servo için */
    ax->target_vel = vel_cps;
    if (ax->ctrl_mode != CTRL_VELOCITY) {
        ax->ctrl_mode = CTRL_VELOCITY;
    }
}

void Axis_Stop(AxisHandle_t *ax)
{
    ax->target_vel = 0;
    ax->target_pos = ax->hw->get_pos();    /* mevcut pos = hedef */
    ax->ctrl_mode  = CTRL_POSITION;        /* pozisyonu tut      */
}

void Axis_EmergencyStop(AxisHandle_t *ax)
{
    ax->ctrl_mode = CTRL_OFF;
    PID_Reset(&ax->pid_pos);
    PID_Reset(&ax->pid_vel);
    ax->hw->disable();
}

void Axis_ResetPos(AxisHandle_t *ax)
{
    /* QEI donanım sayacını 0'a çek */
    ax->hw->reset_pos();

    /* VelMeasure geçmişini temizle — aksi halde ilk tick'te
     * büyük delta → sahte hız sıçraması oluşur              */
    ax->vel.prev_count   = 0;
    ax->vel.vel_filtered = 0.0f;
    ax->vel.velocity_cps = 0.0f;

    /* POSITION modunda ani sıçrama olmasın */
    ax->target_pos = 0;
}

/*-------------------------------------------------
 * Pos loop max hız sınırı
 *------------------------------------------------*/
void Axis_SetMaxVelocity(AxisHandle_t *ax, int32_t vmax_cps)
{
    if (vmax_cps < 0)       vmax_cps = -vmax_cps;
    if (vmax_cps > 200000)  vmax_cps = 200000;   /* güvenlik limiti */

    float v = (float)vmax_cps;
    ax->pid_pos.out_max =  v;
    ax->pid_pos.out_min = -v;

    /* Integral sınırı out_max'ın yarısı — Ki>0 durumunda
     * I-term tek başına vmax'ı zorlamasın               */
    ax->pid_pos.int_max =  v * 0.5f;
    ax->pid_pos.int_min = -v * 0.5f;
}

int32_t Axis_GetMaxVelocity(const AxisHandle_t *ax)
{
    return (int32_t)ax->pid_pos.out_max;
}

/*-------------------------------------------------
 * Canlı PID ayarı
 *------------------------------------------------*/
void  Axis_SetPosKp(AxisHandle_t *ax, float kp) { if (kp >= 0.0f) ax->pid_pos.kp = kp; }
void  Axis_SetPosKi(AxisHandle_t *ax, float ki) { if (ki >= 0.0f) ax->pid_pos.ki = ki; }
float Axis_GetPosKp(const AxisHandle_t *ax)     { return ax->pid_pos.kp; }
float Axis_GetPosKi(const AxisHandle_t *ax)     { return ax->pid_pos.ki; }

/*-------------------------------------------------
 * Hata temizle
 *------------------------------------------------*/
void Axis_ClearFault(AxisHandle_t *ax)
{
    ax->fault_overcurrent = false;
    ax->overcurrent_count = 0;
}
