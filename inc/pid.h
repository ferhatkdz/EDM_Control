#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct {
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;
  float output_limit;
} PID_TypeDef;

typedef struct {
  float pos_start;
  float pos_target;
  float v_max;
  float a_max;
  float a_decel;

  float v_peak;     // vmax ya da ucgen profil icin ulasilabilecek tepe hiz
  float d_acc;      // hizlanma mesafesi
  float d_const;    // sabit hiz mesafesi
  float d_dec;      // yavaslama mesafesi

  float sign;       // ileri (+1) / geri (-1)
  int hold_mode;
} TrapProfile;

/* ===================== AUTO TUNE MODULE ===================== */
/* Genel amaçlı, hafif online PID ayarlayıcı.
   - Hız döngüsü: e_v = vref - v_meas, e_dot ≈ (e_v - e_v_prev)/dt
   - Hold (pozisyon) döngüsü: e_p = pos_ref - pos_mm, e_dot ≈ -v_meas (ölçüm türevi)
   - Doygunlukta (|u| == u_limit) ve hata doyum yönünde ise Ki'yi azaltır (anti-windup adaptasyonu).
   - Sık sıfır-kesişim = osilasyon → Kd↑, Kp↓ (yumuşatır)
   - Kalıcı ofset → Ki↑
   - Büyük hata & artan hata → Kp↑ (ama sınırlı)
*/

typedef struct {
  /* EMA metrikleri */
  float ema_e;        // hata ortalaması
  float ema_abs_e;    // |hata| ortalaması
  float ema_edot;     // hata türevi ortalaması

  /* Osilasyon/overshoot takibi */
  float prev_e;
  float peak_abs_e;
  int zero_cross_count;
  float time_since_cross;

  /* Öğrenme oranları ve sınırlar */
  float alpha_kp, alpha_ki, alpha_kd;    // kazanç güncelleme hızları
  float kp_min, kp_max;
  float ki_min, ki_max;
  float kd_min, kd_max;

  /* Eşikler */
  float deadband;       // küçük hata bölgesi (mm veya mm/s)
  float bias_thresh;    // kalıcı ofset eşiği (EMA |e|)
  float osc_thresh;     // osilasyon tespiti için min |e| (overshoot sayılır)
  float settle_band;    // hedefte kabul bandı (|e|<settle_band)

  /* Zamanlama */
  float ema_alpha;       // EMA katsayısı (0.01–0.2 arası)
  float cross_min_dt;    // sıfır-kesişim saymak için min süre (ms sönümlemek için)

  /* İç durum zamanlayıcı */
  float dt_accum;
} AutoTuneState;

void PID_Reset(PID_TypeDef *des_pid, const PID_TypeDef *src_pid);
float PID_Update(PID_TypeDef *pid, float setpoint, float measurement, float dt);
void TrapProfile_Init(TrapProfile *p, float start, float target, float vmax, float amax);
float TrapProfile_GetVelocity(TrapProfile *p, float t);
float TrapProfile_AdjustVmax(float v_min_nominal, float v_max_nominal, float a_max, float distance);

void AutoTune_Init(AutoTuneState *s, float kp_min, float kp_max, float ki_min, float ki_max, float kd_min, float kd_max);
void AutoTune_Step(AutoTuneState *s, PID_TypeDef *pid, float e, float e_dot, float u, float u_limit, float dt);
#if defined(__cplusplus)
}
#endif
