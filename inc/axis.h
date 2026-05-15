#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "velocity.h"

/*-------------------------------------------------
 * Eksen kimliği
 *------------------------------------------------*/
typedef enum { AXIS_Z = 0, AXIS_W = 1, AXIS_COUNT } AxisId_e;

/*-------------------------------------------------
 * Kontrol modu
 *------------------------------------------------*/
typedef enum {
    CTRL_OFF = 0,
    CTRL_VELOCITY,      /* Sadece hız döngüsü          */
    CTRL_POSITION,      /* Pos + vel iç içe            */
    CTRL_FREE_DUTY,     /* Açık çevrim — PID yok       */
} CtrlMode_e;

/*-------------------------------------------------
 * Donanım soyutlama tablosu (ROM'da saklanır)
 * Her eksen farklı bir tablo gösterir.
 *------------------------------------------------*/
typedef struct {
    void     (*pwm_init)(uint32_t freq_hz);
    void     (*adc_init)(void);
    void     (*qei_init)(void);
    void     (*enable)(void);
    void     (*disable)(void);
    void     (*set_duty)(int32_t duty);
    void     (*reset_pos)(void);
    int32_t  (*get_pos)(void);
} AxisHW_t;

/*-------------------------------------------------
 * Derleme-zamanı PID + mekanik konfigürasyon
 *------------------------------------------------*/
typedef struct {
    /* Pozisyon PID */
    float pos_kp, pos_ki, pos_kd;
    float pos_out_min, pos_out_max;
    float pos_int_min, pos_int_max;
    float pos_kff;
    float pos_d_filter;

    /* Hız PID */
    float vel_kp, vel_ki, vel_kd;
    float vel_out_min, vel_out_max;
    float vel_int_min, vel_int_max;
    float vel_kff;
    float vel_d_filter;

    /* Hız ölçümü */
    float    vel_lpf_alpha;

    /* Donanım */
    uint32_t pwm_freq_hz;
    float    counts_per_mm;

    /* Aşırı akım koruması */
    uint32_t overcurrent_threshold_adc;  /* ADC count cinsinden eşik       */
    uint32_t overcurrent_trip_ticks;     /* Kaç ardışık tick sonra trip     */
} AxisConfig_t;

/*-------------------------------------------------
 * Eksen handle — statik tahsis, malloc yok
 * Sadece ISR ile paylaşılan 3 alan volatile.
 *------------------------------------------------*/
typedef struct {
    const AxisHW_t     *hw;
    const AxisConfig_t *cfg;

    PID_t               pid_pos;
    PID_t               pid_vel;
    VelMeasure_t        vel;

    volatile CtrlMode_e ctrl_mode;
    volatile int32_t    target_pos;
    volatile int32_t    target_vel;
    volatile uint32_t   current_adc;        /* EADC ISR her tick yazar    */
    volatile bool       fault_overcurrent;  /* Aşırı akım hata bayrağı    */
    volatile int32_t    last_duty;          /* Son uygulanan duty         */

    uint32_t            pos_div;            /* 5kHz→1kHz böleç            */
    uint32_t            overcurrent_count;  /* Ardışık aşım sayacı        */
    AxisId_e            id;
} AxisHandle_t;

/*-------------------------------------------------
 * Global eksen dizisi + donanım tabloları
 * (bsp.c'de tanımlanır)
 *------------------------------------------------*/
extern AxisHandle_t   g_axes[AXIS_COUNT];
extern const AxisHW_t g_axis_z_hw;
extern const AxisHW_t g_axis_w_hw;

/*-------------------------------------------------
 * Yaşam döngüsü
 *------------------------------------------------*/
void Axis_Init(AxisHandle_t *ax, AxisId_e id,
               const AxisHW_t *hw, const AxisConfig_t *cfg);

void AllAxes_Init(void);        /* bsp.c QF_onStartup'tan çağrılır    */
void AllAxes_ControlTick(void); /* TMR2_IRQHandler'dan çağrılır        */

/*-------------------------------------------------
 * ISR tick — AllAxes_ControlTick içinden çağrılır
 *------------------------------------------------*/
void Axis_ControlTick(AxisHandle_t *ax);

/*-------------------------------------------------
 * Hareket komutları
 *------------------------------------------------*/
void Axis_MoveToPosition(AxisHandle_t *ax, int32_t target_count);
void Axis_MoveWithDuty  (AxisHandle_t *ax, int32_t duty);
void Axis_SetVelocity   (AxisHandle_t *ax, int32_t vel_cps);

/* PID state'i bozmadan sadece target_vel güncelle.
 * Sürekli hız komutu üreten kontrolörler için (örn. ark servo). */
void Axis_UpdateVelocity(AxisHandle_t *ax, int32_t vel_cps);

void Axis_Stop         (AxisHandle_t *ax); /* Pozisyonu tut             */
void Axis_EmergencyStop(AxisHandle_t *ax); /* PWM kapat, PID sıfırla   */
void Axis_ResetPos     (AxisHandle_t *ax); /* QEI + vel geçmişi sıfırla */

/*-------------------------------------------------
 * Pos loop max hız sınırı
 *------------------------------------------------*/
void    Axis_SetMaxVelocity(AxisHandle_t *ax, int32_t vmax_cps);
int32_t Axis_GetMaxVelocity(const AxisHandle_t *ax);

/*-------------------------------------------------
 * Canlı PID ayarı (CLI tuning)
 *------------------------------------------------*/
void  Axis_SetPosKp(AxisHandle_t *ax, float kp);
void  Axis_SetPosKi(AxisHandle_t *ax, float ki);
float Axis_GetPosKp(const AxisHandle_t *ax);
float Axis_GetPosKi(const AxisHandle_t *ax);

/*-------------------------------------------------
 * Hata yönetimi
 *------------------------------------------------*/
void Axis_ClearFault(AxisHandle_t *ax);

#if defined(__cplusplus)
}
#endif
