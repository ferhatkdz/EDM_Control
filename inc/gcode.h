#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * gcode.h — Saf GRBL-subset G-code ayrıştırıcı
 *
 * QPC'ye bağımlılık YOK; doğrudan struct üzerinde çalışır.
 * GCodeAO bu fonksiyonları çağırarak GCodeEvt oluşturur.
 *
 * Desteklenen komutlar:
 *   G0  [Xx] [Yy] [Zz] [Ww]         — hızlı hareket (maks hız)
 *   G1  [Xx] [Yy] [Zz] [Ww] [Fff]   — doğrusal hareket (besleme hızı)
 *   G28                               — home
 *   G38.2 [Zz|Ww] [Fff]             — probe: temas bul, mutlak konum raporla
 *   G38.3 [Zz|Ww] [Fff]             — probe: temas bul, encoder sıfırla
 *   M3                                — kıvılcım aç
 *   M5                                — kıvılcım kapat
 *   ?                                 — anlık durum sorgusu
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>   /* NAN, isnanf */

/*-------------------------------------------------
 * Ayrıştırılmış G-code komutu (QEvt taşıyıcısı olmadan)
 * GCodeEvt.super çıkarılmış hali; GCodeAO bunu QEvt'ye sarar.
 *------------------------------------------------*/
typedef struct {
    float   x, y, z, w;   /* mm; NAN = bu komutta belirtilmemiş */
    float   f;             /* mm/dak; 0.0f = değişmez            */
    float   p;             /* P parametresi (M100/M101/M102)     */
    uint8_t gcode;         /* 0=G0, 1=G1, 28=G28                 */
    uint8_t mcode;         /* 0=yok, 3=M3, 5=M5, 100-102=güç    */
    bool    has_x;
    bool    has_y;
    bool    has_z;
    bool    has_w;
    bool    has_p;         /* P parametresi belirtildi mi        */
    bool    is_home;       /* G28 geldi                          */
    bool    is_status;     /* '?' satır içi durum sorgusu        */
    bool    is_probe;      /* G38.2 — ölçüm modu (sıfırlamaz)   */
    bool    is_probe_zero; /* G38.3 — temas anında encoder sıfır */
} GCodeCmd_t;

/*-------------------------------------------------
 * GCode_Parse
 *
 * Tek bir G-code satırını ayrıştırır.
 * line: null-terminated, büyük/küçük harf duyarsız.
 * out : başarıda doldurulur.
 *
 * Döndürür:
 *   true  — geçerli komut ayrıştırıldı (out dolu)
 *   false — boş satır, yorum veya tanınmayan komut
 *
 * Not: is_status==true olduğunda diğer alanlar tanımsız olabilir.
 *------------------------------------------------*/
bool GCode_Parse(const char *line, GCodeCmd_t *out);

/*-------------------------------------------------
 * GCode_FormatStatus
 *
 * GRBL uyumlu durum dizesi oluşturur, buf'a yazar.
 * Örnek çıktı:
 *   <Idle|MPos:0.000,0.000,-1.500,2.200|FS:0,0>
 *
 * state_str : "Idle", "Run", "Home" vb.
 * *_mm      : eksen pozisyonları mm cinsinden
 *------------------------------------------------*/
void GCode_FormatStatus(char    *buf,
                        uint8_t  buflen,
                        const char *state_str,
                        float    x_mm,
                        float    y_mm,
                        float    z_mm,
                        float    w_mm);

#if defined(__cplusplus)
}
#endif
