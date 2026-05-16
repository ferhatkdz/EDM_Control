#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*-------------------------------------------------
 * ARK (EDM spark) kontrol modülü
 *
 * Sorumluluk:
 *  - Spark PWM ve enerji bankası MOSFET kontrolü
 *  - Gap voltage feedback ile Z-ekseni servo (delme)
 *  - State machine: OFF / IDLE / DRILLING / REACHED
 *
 * Konvansiyon:
 *  - Negatif encoder yönü = aşağı = delme yönü
 *  - mm cinsinden komut, içeride counts'a çevrilir
 *  - Gap voltage düşük (< short threshold) → kısa devre → kaçır
 *  - Gap voltage yüksek (> target) → boşluk büyük → ilerle
 *
 * Bağımlılıklar:
 *  - g_u32FilteredGapVoltage (bsp.c, EADC01 ISR'de güncellenir)
 *  - Motor_UpdateVelocity (motor.c)
 *  - BSP_SPARK_pwm_set (bsp.c)
 *
 * Tick:
 *  - Ark_Tick() EADC01 ISR'den her çağrı, içeride 1 kHz'e böler
 *------------------------------------------------*/

typedef enum {
    ARK_OFF = 0,       /* spark kapalı, motor durduruldu                */
    ARK_IDLE,          /* spark açık, motor idle (hedef yok)            */
    ARK_DRILLING,      /* aktif delme, gap servo motor hızını ayarlıyor */
    ARK_REACHED,       /* hedef Z derinliğine ulaşıldı, pozisyon tut    */
    ARK_FIND_EDGE,     /* touch-off: elektrot iş parçasına yaklaşıyor   */
    ARK_FIND_RETRACT,  /* temas bulundu, pos sıfırlandı, güvenli mesafeye çekiliyor */
} ArkState_e;

/*-------------------------------------------------
 * Lifecycle
 *------------------------------------------------*/
void Ark_Init(void);

/* Servo tick — EADC01 ISR'den çağır (5 kHz);
 * içeride /5 prescaler ile 1 kHz servo hesaplar.   */
void Ark_Tick(void);

/*-------------------------------------------------
 * Kontrol komutları
 *------------------------------------------------*/
void Ark_Enable(bool en);              /* spark PWM on/off            */
void Ark_StartDrill(float depth_mm);   /* o anki pozisyondan rel delme */
void Ark_StopDrill(void);              /* delmeyi bırak, IDLE'a dön    */

/* Touch-off / kenar bulma: spark KAPALI iken elektrotu approach_cps
 * hızıyla aşağı sürer; gap voltage < short eşiği (temas) anında
 * pozisyonu iş parçası yüzeyinde sıfırlar ve safe_mm kadar yukarı
 * güvenli mesafeye çekilir. ARK_OFF/IDLE'dan çağrılabilir.          */
void Ark_StartFindEdge(int32_t approach_cps, float safe_mm);

/*-------------------------------------------------
 * Sorgulama
 *------------------------------------------------*/
ArkState_e  Ark_GetState(void);
const char* Ark_GetStateStr(void);
int32_t     Ark_GetTargetZ(void);     /* target absolute pos (counts) */
int32_t     Ark_GetStartZ(void);
int32_t     Ark_GetCurrentZ(void);

/*-------------------------------------------------
 * Tuning — gap setpoint, servo kazancı, hız sınırları
 *------------------------------------------------*/
void Ark_SetGapTarget(uint32_t adc);    /* hedef gap voltage (ADC counts) */
void Ark_SetGapShort(uint32_t adc);     /* kısa devre eşiği                */
void Ark_SetServoKp(float kp);          /* cps / ADC_count                 */
void Ark_SetVelAdvanceMax(int32_t cps); /* max ilerleme hızı (pozitif)     */
void Ark_SetVelRetractMax(int32_t cps); /* max geri çekme hızı             */
void Ark_SetSparkPwmNormal(uint32_t d); /* normal spark duty (0-100%)      */
void Ark_SetSparkPwmShort (uint32_t d); /* kısa devrede spark duty         */
void Ark_SetPower(uint8_t level);       /* enerji bankası: aktif MOSFET sayısı 0-10 */

/* Peck (gagalama) drilling parametreleri */
void Ark_SetApproachFeed    (int32_t  cps);  /* yaklaşma & toparlanma hızı (M105)   */
void Ark_SetPeckVel         (int32_t  cps);  /* gagalama hızı (M106)                */
void Ark_SetPeckAmplitudeMm (float    mm);   /* gagalama yüksekliği (M107)          */
void Ark_SetSparkThreshold  (uint32_t adc);  /* SC > bu → spark var (M108)          */
void Ark_SetNoSparkTimeoutMs(uint32_t ms);   /* peck recovery zaman aşımı (M109)    */

uint32_t Ark_GetGapTarget(void);
uint32_t Ark_GetGapShort(void);
float    Ark_GetServoKp(void);
int32_t  Ark_GetVelAdvanceMax(void);
int32_t  Ark_GetVelRetractMax(void);
uint32_t Ark_GetSparkPwmNormal(void);
uint32_t Ark_GetSparkPwmShort(void);
uint8_t  Ark_GetPower(void);

/* Peck faz karakteri: 'A'=APPROACH, 'R'=RETRACT, 'D'=ADVANCE */
char     Ark_GetPeckPhaseChar(void);

#if defined(__cplusplus)
}
#endif
