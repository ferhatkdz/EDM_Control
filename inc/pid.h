#pragma once

#if defined(__cplusplus)
extern "C" {
#endif



/*-------------------------------------------------
 * PID tip seçimi: position veya velocity formu
 * EDM için velocity form tercih edilir:
 *   - Integrator windup korumasi daha kolay
 *   - Parametre degisimi smooth
 *   - Output zaten ?duty ? toplam duty birikir
 *------------------------------------------------*/
typedef struct
{
    /* Parametreler */
    float kp;
    float ki;
    float kd;
    float dt;           /* Örnekleme süresi [s]   */

    /* Durum */
    float prev_error;
    float prev_error2;  /* velocity form için e[k-2] */
    float output;       /* birikimli çikis          */

    /* Sinirlar */
    float out_min;
    float out_max;

    /* Anti-windup */
    float integrator;
    float int_min;
    float int_max;

    /* Feed-forward */
    float kff;

    /* Filtre (D terimi gürültü azaltma) */
    float d_filter_coef; /* 0=filtre yok, 0.8=güçlü filtre */
    float d_prev;

} PID_t;


void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float dt,
              float out_min, float out_max,
              float int_min, float int_max,
              float kff,
              float d_filter);
							
float PID_Update_Velocity(PID_t *pid, float setpoint,
                           float measured, float ff_input);



void PID_Reset(PID_t *pid);


#if defined(__cplusplus)
}
#endif
