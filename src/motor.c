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
	/* Çıkış: hız komutu [count/s]
	 *
	 * Mantık: pozisyon hatası → vel_cmd. Encoder pozisyonu zaten
	 * integratör olduğundan saf P kontrolü teorik olarak 0 steady-
	 * state verir. Ki ekleyince sisteme fazladan faz gecikmesi girer
	 * ve klasik underdamped (önce hızlı, sonra yavaş sönümlenen)
	 * salınım çıkar — kullanıcı testi bunu doğruladı.
	 *
	 * Kd velocity form'da (e[k]-2e[k-1]+e[k-2])/dt = 2. türev
	 * olarak hesaplanıyor, setpoint adımında patlar; gerçek
	 * sönümleme katkısı yok, sıfırda tutuyoruz.
	 *
	 * Output [-20000..+20000] cps ≈ ±22 rpm (55000 CPR'de) — motorun
	 * gerçekçi max hız aralığına göre sınırlı. Saturasyona girip
	 * yığılmayı azaltır.                                          */
	/* Pos PID tuning (saha testinden):
	 * Kp=20, Ki=0, vmax=100000 → tüm mesafelerde overshootsuz, hızlı,
	 * salınımsız hedefe varış. Yüksek Kp + saturasyon clamp + hızlı vel
	 * loop kombinasyonu doğrusal deselere zonu (~ vmax/Kp count) içinde
	 * pürüzsüz duruş sağlıyor. Bu motor/yük için stiction sorunu yok,
	 * Ki gerekmiyor.                                                  */
	PID_Init(&s_pid_pos,
					 /*kp*/  20.0f,
					 /*ki*/  0.0f,
					 /*kd*/  0.0f,
					 /*dt*/  0.001f,          /* 1kHz */
					 /*out*/ -100000.0f, 100000.0f,  /* default max hız; CLI 'vmax' ile değişir */
					 /*int*/ -50000.0f, 50000.0f,
					 /*kff*/ 0.0f,
					 /*lpf*/ 0.5f);

	/*--- Hiz PID ---*/
	/* �ikis: duty [-10000..+10000]
	 * NOT: Kd=0 — velocity form'da D-term setpoint adımında
	 * (e[k]-2e[k-1]+e[k-2])/dt 2. türev olarak patlıyor.
	 * Önce P+I ile stabil taban kur, gerekirse D'yi ölçüm
	 * üzerinden (error değil) ekleriz.                            */
	PID_Init(&s_pid_vel,
					 /*kp*/  0.5f,  /* was 0.1f */
					 /*ki*/  50.0f, /* was 2.0f */
					 /*kd*/  0.0f,
					 /*dt*/  0.0002f,         /* 5kHz */
					 /*out*/ -10000.0f, 10000.0f,
					 /*int*/ -8000.0f,  8000.0f,
					 /*kff*/ 0.0f,
					 /*lpf*/ 0.7f);

	/*--- Hiz �l�er ---*/
	s_vel.prev_count = 0;
	s_vel.vel_filtered = 0.0f;
	s_vel.lpf_alpha  = 0.3f;   /* Encoder g�r�lt�s�ne g�re ayarla */

	BSP_AXIS_Z_enable();
}

/*-------------------------------------------------
 * TIMER2 ISR � 5kHz Hiz + I� D�ng�
 *------------------------------------------------*/
static uint32_t s_pos_div = 0; /* 5kHz?1kHz b�l�c� */

/*-------------------------------------------------
 * Hiz �l��m debug � SADECE bunu test et
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
    .alpha        = 0.3f,  /* baslangi�ta d�s�k tutuyoruz */
};

volatile int32_t last_speed = 0;

void TMR2_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER2))
    {
			
        if (s_ctrl_mode == CTRL_FREE_DUTY)
        {
            TIMER_ClearIntFlag(TIMER2);
            return;
        }

				
        /*--- QEI oku ---*/
        int32_t pos_now = QEI_GetSignedCount(QEI1);

        /*--- Hiz �l� ---*/
        VelMeasure_Update(&s_vel, pos_now);

			
			
        if (s_ctrl_mode == CTRL_OFF)
        {
            BSP_AXIS_Z_set_duty(0);
            TIMER_ClearIntFlag(TIMER2);
            return;
        }

        /*--- Hiz komutu kaynagi ---*/
        float vel_cmd = 0.0f;

        /*--- Pozisyon dongusu (1kHz) ---*/
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

                /* Positional-form: velocity-form'da büyük adım girişinde
                 * doyma sonrası "information loss" olup motor erken
                 * duruyordu. Pos döngüsünde her tick u = Kp·e + I + FF
                 * şeklinde baştan hesaplanmalı.                        */
                vel_cmd = PID_Update_Pos(&s_pid_pos,
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

        /*--- Hiz dongusu (5kHz) ---*/
				
				last_speed = s_vel.velocity_cps;
				
        float duty_f = PID_Update_Velocity(&s_pid_vel,
                                            vel_cmd,
                                            s_vel.velocity_cps,
                                            0.0f);

				//CDC_SendFmt("duty:%.2f\n\r", duty_f);
				
        BSP_AXIS_Z_set_duty((int32_t)duty_f);

        TIMER_ClearIntFlag(TIMER2);
    }
}

/*-------------------------------------------------
 * Timer Init � 5kHz
 *------------------------------------------------*/
void ControlTimer_Init(void)
{
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* 5kHz = 200us periyot */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 5000);
    TIMER_EnableInt(TIMER2);
    NVIC_SetPriority(TMR2_IRQn, 1);  /* ADC ISR'den d�s�k �ncelik */
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);
}

/*-------------------------------------------------
 * Hareket komutu API
 *------------------------------------------------*/
void Motor_MoveToPosition(int32_t target_count)
{
		Motor_Stop();

	  PID_Reset(&s_pid_pos);
    PID_Reset(&s_pid_vel);

    /* PID_Reset output'u smooth-stop için tutuyor; yeni komutta
     * eski birikim olmaması için explicit sıfırla.              */
    s_pid_pos.output = 0.0f;
    s_pid_vel.output = 0.0f;

		BSP_AXIS_Z_enable();

    s_target_pos  = target_count;
    s_target_vel  = 0;
    s_ctrl_mode   = CTRL_POSITION;
}

/*-------------------------------------------------
 * Pos loop max hız sınırı (out_max) runtime ayarı.
 * vmax_cps: pozitif değer; pos PID çıkışını ±vmax_cps cps
 * aralığında kısıtlar. Bu değer pratikte hareket esnasındaki
 * max motor hızı olur.
 *------------------------------------------------*/
void Motor_SetMaxVelocity(int32_t vmax_cps)
{
    if (vmax_cps < 0)        vmax_cps = -vmax_cps;     /* abs */
    if (vmax_cps > 200000)   vmax_cps = 200000;        /* üst limit (güvenlik) */

    float v = (float)vmax_cps;
    s_pid_pos.out_max =  v;
    s_pid_pos.out_min = -v;
    /* Integral sınırını out_max'ın yarısı yap — Ki>0 durumunda
     * I-term tek başına vmax'ı zorlamasın.                       */
    s_pid_pos.int_max =  v * 0.5f;
    s_pid_pos.int_min = -v * 0.5f;
}

int32_t Motor_GetMaxVelocity(void)
{
    return (int32_t)s_pid_pos.out_max;
}

/*-------------------------------------------------
 * Pos PID Kp / Ki canlı ayarı (CLI'dan tuning için)
 *------------------------------------------------*/
void  Motor_SetPosKp(float kp) { if (kp >= 0.0f) s_pid_pos.kp = kp; }
void  Motor_SetPosKi(float ki) { if (ki >= 0.0f) s_pid_pos.ki = ki; }
float Motor_GetPosKp(void)     { return s_pid_pos.kp; }
float Motor_GetPosKi(void)     { return s_pid_pos.ki; }

void Motor_MoveWithDuty(int32_t duty) {
	  BSP_AXIS_Z_enable();

		s_ctrl_mode   = CTRL_FREE_DUTY;
		BSP_AXIS_Z_set_duty(duty);
}

/*-------------------------------------------------
 * Vel target'i PID state'ini bozmadan güncelle.
 * Sürekli (her tick) hız komutu üreten kontrolörler için
 * (örn. ark servo loop'u) Motor_SetVelocity yerine bunu kullan;
 * vel PID integrator/output korunarak smooth tracking olur.
 *------------------------------------------------*/
void Motor_UpdateVelocity(int32_t vel_cps)
{
    s_target_vel = vel_cps;
    if (s_ctrl_mode != CTRL_VELOCITY)
    {
        s_ctrl_mode = CTRL_VELOCITY;
    }
}

void Motor_SetVelocity(int32_t vel_cps)
{
		BSP_AXIS_Z_enable();

    /* Yeni hız komutunda PID state'i temizle: eski integrator
     * birikimi yeni komutu bozmasın.                            */
    PID_Reset(&s_pid_vel);
    s_pid_vel.output = 0.0f;
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

/*-------------------------------------------------
 * QEI donanim sayacini 0'a cek + hiz olcumunun
 * prev_count gecmisini temizle. Aksi halde reset
 * sonrasi ilk TMR2 tick'inde VelMeasure_Update
 * dev bir delta gorup sahte hiz sicramasi uretir.
 * s_target_pos da 0'a cekilir (POSITION modunda
 * ani sicrama olmasin diye).
 *
 * NOT: EADC ISR (TMR2'den yuksek oncelik) icinden
 * cagrilirsa TMR2'ye gore atomiktir.
 *------------------------------------------------*/
void Motor_ResetPosition(void)
{
    BSP_AXIS_z_reset_pos();
    s_vel.prev_count = 0;
    s_target_pos     = 0;
}
