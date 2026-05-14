#include "pid.h"

/*-------------------------------------------------
 * PID Init
 *------------------------------------------------*/
void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float dt,
              float out_min, float out_max,
              float int_min, float int_max,
              float kff,
              float d_filter)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->dt           = dt;
    pid->prev_error   = 0.0f;
    pid->prev_error2  = 0.0f;
    pid->output       = 0.0f;
    pid->out_min      = out_min;
    pid->out_max      = out_max;
    pid->integrator   = 0.0f;
    pid->int_min      = int_min;
    pid->int_max      = int_max;
    pid->kff          = kff;
    pid->d_filter_coef = d_filter;
    pid->d_prev       = 0.0f;
}

void PID_Reset(PID_t *pid)
{
    pid->prev_error  = 0.0f;
    pid->prev_error2 = 0.0f;
    pid->integrator  = 0.0f;
    pid->d_prev      = 0.0f;
    /* output sifirlama: smooth stop i�in YAPMA
     * Motor_Disable() ile birlikte kullan        */
}

/*-------------------------------------------------
 * Velocity Form PID
 * ?u[k] = kp�(e[k]-e[k-1])
 *        + ki�dt�e[k]
 *        + kd/dt�(e[k]-2e[k-1]+e[k-2])
 * u[k]  = u[k-1] + ?u[k]
 *------------------------------------------------*/
float PID_Update_Velocity(PID_t *pid, float setpoint,
                           float measured, float ff_input)
{
    float error = setpoint - measured;

    /*--- P terimi (incremental) ---*/
    float delta_p = pid->kp * (error - pid->prev_error);

    /*--- I terimi ---*/
    float delta_i = pid->ki * pid->dt * error;

    /*--- D terimi (filtered) ---*/
    float d_raw = (error - 2.0f * pid->prev_error + pid->prev_error2)
                   / pid->dt;
    float d_filtered = pid->d_filter_coef * pid->d_prev
                      + (1.0f - pid->d_filter_coef) * d_raw;
    pid->d_prev = d_filtered;
    float delta_d = pid->kd * d_filtered;

    /*--- Feed-forward ---*/
    float delta_ff = pid->kff * ff_input;

    /*--- Toplam delta ---*/
    float delta_u = delta_p + delta_i + delta_d + delta_ff;

    /*--- �ikisi g�ncelle ---*/
    pid->output += delta_u;

		#if 0
    /*--- Anti-windup: back-calculation ---*/
    float output_sat = pid->output;
    if (output_sat > pid->out_max) output_sat = pid->out_max;
    if (output_sat < pid->out_min) output_sat = pid->out_min;

    /* Saturasyon varsa integrat�r� geri al */
    if (output_sat != pid->output)
    {
        pid->output = output_sat;
        /* Integrat�r katkisini azalt */
        pid->integrator += (output_sat - pid->output);
    }
		#else
		if (pid->output > pid->out_max) pid->output = pid->out_max;
		if (pid->output < pid->out_min) pid->output = pid->out_min;
		#endif

    /*--- Hata ge�misini g�ncelle ---*/
    pid->prev_error2 = pid->prev_error;
    pid->prev_error  = error;

    return pid->output;
}

/*-------------------------------------------------
 * Positional form PID
 *   u[k] = Kp·e[k] + Ki·∫e + Kff·ff
 *
 * Velocity form'dan farkı: çıkış her tick'te baştan
 * hesaplanır, geçmiş u'ya bağlı değildir. Saturasyondan
 * sonra hata kaldığı sürece u doymuş olarak kalır;
 * motor hareket etmeye devam eder. Pozisyon döngüleri
 * için doğru tercih.
 *
 * Anti-windup: conditional integration —
 * çıkış doymuşken ve hatanın işareti doymayı derinleştiriyorsa
 * integrator dondurulur.
 *------------------------------------------------*/
float PID_Update_Pos(PID_t *pid, float setpoint,
                     float measured, float ff_input)
{
    float error = setpoint - measured;

    /* P ve FF terimleri */
    float p_term  = pid->kp  * error;
    float ff_term = pid->kff * ff_input;

    /* Saturasyon tahmini (integrator güncellenmeden) */
    float output_pre = p_term + pid->integrator + ff_term;

    /* Conditional integration: doyma yönünde I büyütme */
    int sat_high = (output_pre >= pid->out_max) && (error > 0.0f);
    int sat_low  = (output_pre <= pid->out_min) && (error < 0.0f);
    if (!sat_high && !sat_low)
    {
        pid->integrator += pid->ki * pid->dt * error;
        if (pid->integrator > pid->int_max) pid->integrator = pid->int_max;
        if (pid->integrator < pid->int_min) pid->integrator = pid->int_min;
    }

    /* Toplam ve clamp */
    float output = p_term + pid->integrator + ff_term;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->output     = output;
    pid->prev_error = error;
    return output;
}