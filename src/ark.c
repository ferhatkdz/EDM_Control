#include "ark.h"
#include "motor.h"
#include "velocity.h"   /* QEI_GetSignedCount, COUNTS_PER_REV_F, LEAD_SCREW_MM_PER_REV */
#include "NuMicro.h"

/*-------------------------------------------------
 * Extern globals (bsp.c'de EADC01 ISR günceller)
 *------------------------------------------------*/
extern volatile uint32_t g_u32FilteredGapVoltage;
extern volatile uint32_t g_u32FilteredSparkCurrent;

/*-------------------------------------------------
 * Spark PWM + enerji bankası MOSFET kontrolü (bsp.c)
 *------------------------------------------------*/
extern void BSP_SPARK_pwm_set(int u32DutyCycle);
extern void BSP_SPARK_mosfet_set(uint8_t id, uint8_t state);
extern void BSP_SPARK_mosfet_apply(void);

/*-------------------------------------------------
 * mm → counts dönüşüm
 *   1 devir = LEAD_SCREW_MM_PER_REV mm = COUNTS_PER_REV_F count
 *------------------------------------------------*/
#define MM_TO_COUNTS(mm_f) \
    ((int32_t)((mm_f) * (COUNTS_PER_REV_F / LEAD_SCREW_MM_PER_REV)))

/*-------------------------------------------------
 * Modül durumu
 *------------------------------------------------*/
typedef struct {
    volatile ArkState_e state;

    int32_t start_z_count;
    int32_t target_z_count;     /* mutlak hedef pos (counts); start - depth */

    /* Tuning */
    uint32_t gap_target_adc;
    uint32_t gap_short_adc;
    float    kp_servo;
    int32_t  vel_advance_max;   /* pozitif değer, ilerleme yönü için negatife çevrilir */
    int32_t  vel_retract_max;

    uint32_t spark_pwm_normal;  /* % */
    uint32_t spark_pwm_short;

    uint8_t  power_level;       /* aktif enerji bankası MOSFET sayısı 0-10 */

    /* Touch-off / kenar bulma */
    int32_t  find_approach_cps; /* yaklaşma hızı (pozitif; aşağı = negatif uygulanır) */
    int32_t  find_safe_count;   /* temas sonrası geri çekme mesafesi (counts, pozitif) */

    /* 5 kHz → 1 kHz prescaler */
    uint32_t tick_div;
} Ark_t;

static Ark_t s_ark = {
    .state            = ARK_OFF,
    .gap_target_adc   = 150,        /* deneysel başlangıç */
    .gap_short_adc    = 20,         /* mevcut kodla uyumlu */
    .kp_servo         = 100.0f,     /* cps / ADC_count */
    .vel_advance_max  = 50000,      /* cps */
    .vel_retract_max  = 80000,
    .spark_pwm_normal = 10,
    .spark_pwm_short  = 2,
    .power_level      = 1,          /* default: 1 MOSFET — şu an sadece MOSFET 1 (80V klemens tarafı) bağlı */
    .find_approach_cps = 0,
    .find_safe_count   = 0,
    .tick_div         = 0,
};

/*-------------------------------------------------
 * Enerji bankası: ilk 'level' adet MOSFET'i aç, geri kalanı kapat.
 * TPIC6C595 shift register'a SPI ile yazılır.
 *------------------------------------------------*/
static void ark_apply_mosfets(uint8_t level)
{
    uint8_t i;
    for (i = 1; i <= 10; i++)
        BSP_SPARK_mosfet_set(i, (i <= level) ? 1 : 0);
    BSP_SPARK_mosfet_apply();
}

/*-------------------------------------------------
 * Init
 *------------------------------------------------*/
void Ark_Init(void)
{
    s_ark.state = ARK_OFF;
    /* Spark donanımı BSP_init içinde init edildi; sadece kapalı tut */
    BSP_SPARK_pwm_set(0);
    ark_apply_mosfets(0);   /* tüm enerji bankası MOSFET'leri kapalı */
}

/*-------------------------------------------------
 * Public komutlar
 *------------------------------------------------*/
void Ark_Enable(bool en)
{
    if (en) {
        /* Enerji bankasını seç + spark PWM aç */
        ark_apply_mosfets(s_ark.power_level);
        BSP_SPARK_pwm_set((int)s_ark.spark_pwm_normal);
        if (s_ark.state == ARK_OFF) s_ark.state = ARK_IDLE;
    } else {
        BSP_SPARK_pwm_set(0);
        ark_apply_mosfets(0);   /* enerji bankasını da kes — güvenlik */
        Motor_UpdateVelocity(0);
        s_ark.state = ARK_OFF;
    }
}

void Ark_StartDrill(float depth_mm)
{
    /* Ark kapalıyken delme başlatma — önce ark on gerekli */
    if (s_ark.state == ARK_OFF) return;

    int32_t cur = QEI_GetSignedCount(QEI1);
    int32_t depth_counts = MM_TO_COUNTS(depth_mm);

    s_ark.start_z_count  = cur;
    s_ark.target_z_count = cur - depth_counts; /* negatif yön = aşağı */

    /* Vel loop'u temiz başlat */
    Motor_SetVelocity(0);
    s_ark.state = ARK_DRILLING;
}

void Ark_StopDrill(void)
{
    Motor_UpdateVelocity(0);
    if (s_ark.state == ARK_DRILLING || s_ark.state == ARK_REACHED)
        s_ark.state = ARK_IDLE;
    else if (s_ark.state == ARK_FIND_EDGE || s_ark.state == ARK_FIND_RETRACT)
        s_ark.state = ARK_OFF;   /* find-edge standalone & spark kapalı */
}

/*-------------------------------------------------
 * Touch-off / kenar bulma başlat.
 *   approach_cps : yaklaşma hızı (pozitif), aşağı yönde uygulanır
 *   safe_mm      : temas sonrası geri çekilecek güvenli mesafe (mm)
 * Spark PWM kapalı tutulur — temas saf gap-voltage divider ile
 * algılanır, kıvılcım enerjisi oluşmaz.
 *------------------------------------------------*/
void Ark_StartFindEdge(int32_t approach_cps, float safe_mm)
{
    if (approach_cps <= 0) return;
    /* Aktif delme sırasında başlatma — önce stop gerekli */
    if (s_ark.state == ARK_DRILLING || s_ark.state == ARK_REACHED) return;

    /* Standalone touch-off: spark PWM kapalı kalsın */
    BSP_SPARK_pwm_set(0);

    s_ark.find_approach_cps = approach_cps;

    int32_t sc = MM_TO_COUNTS(safe_mm);
    if (sc < 0) sc = -sc;               /* her zaman yukarı yön = pozitif */
    s_ark.find_safe_count = sc;

    /* Vel loop'u temiz başlat */
    Motor_SetVelocity(0);
    s_ark.state = ARK_FIND_EDGE;
}

/*-------------------------------------------------
 * Servo Tick — EADC01 ISR'den her çağrılır (5 kHz)
 * İçeride /5 prescaler → 1 kHz servo
 *------------------------------------------------*/
void Ark_Tick(void)
{
    if (++s_ark.tick_div < 5) return;
    s_ark.tick_div = 0;

    uint32_t gap_v = g_u32FilteredGapVoltage;

    /*--- Touch-off (kenar bulma) state machine -------------------
     * Spark kapalı; temas gap voltage'ın short eşiğinin altına
     * düşmesiyle algılanır.                                       */
    if (s_ark.state == ARK_FIND_EDGE) {
        if (gap_v < s_ark.gap_short_adc) {
            /* Temas! Motoru durdur, pozisyonu iş parçası yüzeyinde
             * sıfırla, güvenli mesafeye çekilmeye geç.            */
            Motor_UpdateVelocity(0);
            Motor_ResetPosition();
            s_ark.state = ARK_FIND_RETRACT;
        } else {
            /* aşağı yön = negatif hız */
            Motor_UpdateVelocity(-s_ark.find_approach_cps);
        }
        return;
    }

    if (s_ark.state == ARK_FIND_RETRACT) {
        /* Reset sonrası temas noktası = 0; yukarı = pozitif */
        int32_t p = QEI_GetSignedCount(QEI1);
        if (p >= s_ark.find_safe_count) {
            Motor_UpdateVelocity(0);
            s_ark.state = ARK_OFF;       /* spark kapalı, motor idle */
        } else {
            Motor_UpdateVelocity(+s_ark.vel_retract_max);
        }
        return;
    }

    if (s_ark.state == ARK_OFF || s_ark.state == ARK_IDLE) {
        /* OFF/IDLE'da servo çalışmaz; motor yönetimi başka yerde */
        return;
    }

    int32_t  pos    = QEI_GetSignedCount(QEI1);
    int32_t  vel_cmd;

    if (gap_v < s_ark.gap_short_adc) {
        /* Kısa devre / ark — acil geri çekme + spark duty düşür */
        vel_cmd = s_ark.vel_retract_max;
        BSP_SPARK_pwm_set((int)s_ark.spark_pwm_short);
    } else {
        /* Servo kanunu:
         *   err > 0 (gap büyük)  → vel negatif (aşağı/ilerleme)
         *   err < 0 (gap küçük)  → vel pozitif (yukarı/geri)        */
        float err = (float)gap_v - (float)s_ark.gap_target_adc;
        float v   = -s_ark.kp_servo * err;

        if (v < (float)-s_ark.vel_advance_max) v = (float)-s_ark.vel_advance_max;
        if (v > (float)+s_ark.vel_retract_max) v = (float)+s_ark.vel_retract_max;

        vel_cmd = (int32_t)v;
        BSP_SPARK_pwm_set((int)s_ark.spark_pwm_normal);
    }

    /* Hedef derinliği geçme — drilling'de aşağı yönde clamp */
    if (s_ark.state == ARK_DRILLING) {
        if (pos <= s_ark.target_z_count && vel_cmd < 0) {
            vel_cmd = 0;
            s_ark.state = ARK_REACHED;
        }
    } else { /* ARK_REACHED */
        /* Hedefin üstüne çıktıysa tekrar ilerlemeye izin ver,
         * aksi halde ilerleme yönünü kilitle */
        if (pos <= s_ark.target_z_count && vel_cmd < 0) {
            vel_cmd = 0;
        }
    }

    Motor_UpdateVelocity(vel_cmd);
}

/*-------------------------------------------------
 * Getters
 *------------------------------------------------*/
ArkState_e Ark_GetState(void)    { return s_ark.state; }

const char* Ark_GetStateStr(void) {
    switch (s_ark.state) {
        case ARK_OFF:          return "OFF";
        case ARK_IDLE:         return "IDLE";
        case ARK_DRILLING:     return "DRILLING";
        case ARK_REACHED:      return "REACHED";
        case ARK_FIND_EDGE:    return "FIND_EDGE";
        case ARK_FIND_RETRACT: return "FIND_RETRACT";
    }
    return "?";
}

int32_t Ark_GetTargetZ(void)  { return s_ark.target_z_count; }
int32_t Ark_GetStartZ(void)   { return s_ark.start_z_count; }
int32_t Ark_GetCurrentZ(void) { return QEI_GetSignedCount(QEI1); }

uint32_t Ark_GetGapTarget(void)      { return s_ark.gap_target_adc; }
uint32_t Ark_GetGapShort(void)       { return s_ark.gap_short_adc; }
float    Ark_GetServoKp(void)        { return s_ark.kp_servo; }
int32_t  Ark_GetVelAdvanceMax(void)  { return s_ark.vel_advance_max; }
int32_t  Ark_GetVelRetractMax(void)  { return s_ark.vel_retract_max; }
uint32_t Ark_GetSparkPwmNormal(void) { return s_ark.spark_pwm_normal; }
uint32_t Ark_GetSparkPwmShort(void)  { return s_ark.spark_pwm_short; }
uint8_t  Ark_GetPower(void)          { return s_ark.power_level; }

/*-------------------------------------------------
 * Setters
 *------------------------------------------------*/
void Ark_SetGapTarget(uint32_t v)      { s_ark.gap_target_adc = v; }
void Ark_SetGapShort(uint32_t v)       { s_ark.gap_short_adc  = v; }
void Ark_SetServoKp(float v)           { if (v >= 0.0f) s_ark.kp_servo = v; }
void Ark_SetVelAdvanceMax(int32_t v)   { if (v > 0) s_ark.vel_advance_max = v; }
void Ark_SetVelRetractMax(int32_t v)   { if (v > 0) s_ark.vel_retract_max = v; }
void Ark_SetSparkPwmNormal(uint32_t v)
{
    if (v <= 100) {
        s_ark.spark_pwm_normal = v;
        /* IDLE/DRILLING/REACHED'de hemen donanıma yansıt;
         * FIND_EDGE/FIND_RETRACT'te spark kasıtlı kapalı — dokunma */
        if (s_ark.state == ARK_IDLE ||
            s_ark.state == ARK_DRILLING ||
            s_ark.state == ARK_REACHED)
            BSP_SPARK_pwm_set((int)v);
    }
}

void Ark_SetSparkPwmShort(uint32_t v)  { if (v <= 100) s_ark.spark_pwm_short  = v; }

void Ark_SetPower(uint8_t level)
{
    if (level > 10) level = 10;
    s_ark.power_level = level;
    /* Ark aktifse hemen donanıma yansıt; OFF ise sadece sakla */
    if (s_ark.state != ARK_OFF)
        ark_apply_mosfets(level);
}
