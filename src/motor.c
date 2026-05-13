#include "NuMicro.h"
#include "bsp.h"
#include "motor.h"
#include "pid.h"
#include "velocity.h"

/*-------------------------------------------------
 * Global kontrol nesneleri
 *------------------------------------------------*/
static PID_t      s_pid_pos;     /* Pozisyon döngüsü */
static PID_t      s_pid_vel;     /* Hiz döngüsü      */
static VelMeasure_t s_vel;

/* Hedef pozisyon: count cinsinden */
volatile int32_t s_target_pos = 0;
volatile int32_t s_target_vel = 0; /* count/s */
volatile CtrlMode_e s_ctrl_mode = CTRL_OFF;

/*-------------------------------------------------
 * Sistem Init
 *------------------------------------------------*/
void MotorControl_Init(void)
{

	/* init asis z QEI */
	BSP_AXIS_Z_qei_init();

	/* init axis z PWM */
	BSP_AXIS_Z_pwm_init(20000);
	
		/* init axis z adc */
	BSP_AXIS_Z_adc_Init();

	/*--- Pozisyon PID ---*/
	/* Çikis: hiz komutu [count/s]
	 * 5000 count/s ˜ 1 devir/s @ 4000 CPR          */
	PID_Init(&s_pid_pos,
					 /*kp*/  8.0f,
					 /*ki*/  0.1f,
					 /*kd*/  0.05f,
					 /*dt*/  0.001f,          /* 1kHz */
					 /*out*/ -8000.0f, 8000.0f,
					 /*int*/ -4000.0f, 4000.0f,
					 /*kff*/ 0.0f,
					 /*lpf*/ 0.5f);

	/*--- Hiz PID ---*/
	/* Çikis: duty [-10000..+10000]                  */
	PID_Init(&s_pid_vel,
					 /*kp 0.8f*/  10.0f,
					 /*ki*/  5.0f,
					 /*kd*/  0.002f,
					 /*dt*/  0.0002f,         /* 5kHz */
					 /*out*/ -10000.0f, 10000.0f,
					 /*int*/ -8000.0f,  8000.0f,
					 /*kff*/ 0.0f,
					 /*lpf*/ 0.7f);

	/*--- Hiz ölçer ---*/
	s_vel.prev_count = 0;
	s_vel.vel_filtered = 0.0f;
	s_vel.lpf_alpha  = 0.3f;   /* Encoder gürültüsüne göre ayarla */

	BSP_AXIS_Z_enable();
}

/*-------------------------------------------------
 * TIMER2 ISR — 5kHz Hiz + Iç Döngü
 *------------------------------------------------*/
static uint32_t s_pos_div = 0; /* 5kHz?1kHz bölücü */

/*-------------------------------------------------
 * Hiz ölçüm debug — SADECE bunu test et
 *------------------------------------------------*/
typedef struct
{
    int32_t  prev_count;
    float    vel_raw;       /* ham, filtresiz      */
    float    vel_filtered;
    float    alpha;         /* LPF katsayisi       */
    uint32_t tick;
} VelDebug_t;

static VelDebug_t s_vdbg = {
    .prev_count   = 0,
    .vel_filtered = 0.0f,
    .alpha        = 0.3f,  /* baslangiçta düsük tutuyoruz */
};

void TMR2_IRQHandler(void)
{
	#if 1
    if (TIMER_GetIntFlag(TIMER2))
    {
        /*--- QEI oku ---*/
        int32_t pos_now = QEI_GetSignedCount(QEI1);

        /*--- Hiz ölç ---*/
        VelMeasure_Update(&s_vel, pos_now);

        if (s_ctrl_mode == CTRL_OFF)
        {
            BSP_AXIS_Z_set_duty(0);
            TIMER_ClearIntFlag(TIMER2);
            return;
        }

        /*--- Hiz komutu kaynagi ---*/
        float vel_cmd = 0.0f;

        /*--- Pozisyon döngüsü (1kHz) ---*/
        if (s_ctrl_mode == CTRL_POSITION)
        {
            s_pos_div++;
            if (s_pos_div >= 5) /* 5kHz / 5 = 1kHz */
            {
                s_pos_div = 0;

                float pos_err = (float)(s_target_pos - pos_now);

                /* Feed-forward: target hiz tahminini ekle
                 * (trajectory planner varsa buradan gelir) */
                float ff = (float)s_target_vel;

                vel_cmd = PID_Update_Velocity(&s_pid_pos,
                                              (float)s_target_pos,
                                              (float)pos_now,
                                              ff);
            }
            else
            {
                /* 1kHz arasi: son vel_cmd'yi koru */
                /* s_pid_pos.output degismedi       */
                vel_cmd = s_pid_pos.output;
            }
        }
        else /* CTRL_VELOCITY */
        {
            vel_cmd = (float)s_target_vel;
        }

        /*--- Hiz döngüsü (5kHz) ---*/
        float duty_f = PID_Update_Velocity(&s_pid_vel,
                                            vel_cmd,
                                            s_vel.velocity_cps,
                                            0.0f);

        BSP_AXIS_Z_set_duty((int32_t)duty_f);

        TIMER_ClearIntFlag(TIMER2);
    }
		#else
		
    if (!TIMER_GetIntFlag(TIMER2)) return;
    TIMER_ClearIntFlag(TIMER2);

    static uint32_t tick = 0;
    tick++;

    /* QEI */
    int32_t qei_now = (int32_t)QEI_GET_CNT_VALUE(QEI1);
    static int32_t qei_prev = 0;
    int32_t delta = qei_now - qei_prev;
    qei_prev = qei_now;

    /* Ham hiz */
    float vel_raw = (float)delta / 0.0002f;

    /* Agresif LPF — titresimi öldür */
    static float vel_filt = 0.0f;
    vel_filt = 0.05f * vel_raw
             + 0.95f * vel_filt; /* çok güçlü filtre */

    /* Hedef: 2000 count/s sabit */
    float setpoint = 2000.0f;

    /* Saf P kontrol — velocity form degil, position form */
    float error   = setpoint - vel_filt;
    float duty_f  = error * 0.05f; /* kp=0.05 */

    /* Sinirla */
    //if (duty_f >  3000.0f) duty_f =  3000.0f;
    //if (duty_f < -3000.0f) duty_f = -3000.0f;

    BSP_AXIS_Z_set_duty((int32_t)duty_f);

    /* Rapor */
    if (tick % 100 == 0)
    {
        CDC_SendFmt(
            "P_CTRL,"
            "SP:%.0f,PV:%.0f,RAW:%.0f,"
            "ERR:%.0f,DUTY:%.0f\r\n",
            setpoint, vel_filt, vel_raw,
            error, duty_f);
    }
		#endif
}

/*-------------------------------------------------
 * Timer Init — 5kHz
 *------------------------------------------------*/
void ControlTimer_Init(void)
{
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* 5kHz = 200µs periyot */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 5000);
    TIMER_EnableInt(TIMER2);
    NVIC_SetPriority(TMR2_IRQn, 1);  /* ADC ISR'den düsük öncelik */
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);
}

/*-------------------------------------------------
 * Hareket komutu API
 *------------------------------------------------*/
void Motor_MoveToPosition(int32_t target_count)
{
    s_target_pos  = target_count;
    s_target_vel  = 0;
    s_ctrl_mode   = CTRL_POSITION;
}

void Motor_SetVelocity(int32_t vel_cps)
{
    s_target_vel = vel_cps;
    s_ctrl_mode  = CTRL_VELOCITY;
}

void Motor_Stop(void)
{
    s_target_vel = 0;
    s_target_pos = QEI_GetSignedCount(QEI1); /* mevcut pos = hedef */
    s_ctrl_mode  = CTRL_POSITION;            /* pozisyonu tut      */
}

void Motor_EmergencyStop(void)
{
    s_ctrl_mode = CTRL_OFF;
    PID_Reset(&s_pid_pos);
    PID_Reset(&s_pid_vel);
    BSP_AXIS_Z_disable();
}
