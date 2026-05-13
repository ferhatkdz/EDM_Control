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
    /* output sifirlama: smooth stop için YAPMA
     * Motor_Disable() ile birlikte kullan        */
}

/*-------------------------------------------------
 * Velocity Form PID
 * ?u[k] = kp×(e[k]-e[k-1])
 *        + ki×dt×e[k]
 *        + kd/dt×(e[k]-2e[k-1]+e[k-2])
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

    /*--- Çikisi güncelle ---*/
    pid->output += delta_u;

    /*--- Anti-windup: back-calculation ---*/
    float output_sat = pid->output;
    if (output_sat > pid->out_max) output_sat = pid->out_max;
    if (output_sat < pid->out_min) output_sat = pid->out_min;

    /* Saturasyon varsa integratörü geri al */
    if (output_sat != pid->output)
    {
        pid->output = output_sat;
        /* Integratör katkisini azalt */
        pid->integrator += (output_sat - pid->output);
    }

    /*--- Hata geçmisini güncelle ---*/
    pid->prev_error2 = pid->prev_error;
    pid->prev_error  = error;

    return pid->output;
}