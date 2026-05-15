/*
 * gcode.c — Saf GRBL-subset G-code ayrıştırıcı
 *
 * QPC'ye, BSP'ye veya donanıma hiçbir bağımlılık içermez.
 * Birim testlerde doğrudan derlenebilir.
 */

#include "gcode.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>  /* strtof */

/* ------------------------------------------------------------------ */
/*  Yardımcı: beyaz boşluk atla, büyük harfe çevir                    */
/* ------------------------------------------------------------------ */
static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') { p++; }
    return p;
}

/* ------------------------------------------------------------------ */
/*  GCode_Parse                                                        */
/* ------------------------------------------------------------------ */
bool GCode_Parse(const char *line, GCodeCmd_t *out)
{
    if (!line || !out) { return false; }

    /* Çıktıyı sıfırla */
    out->x        = NAN;
    out->y        = NAN;
    out->z        = NAN;
    out->w        = NAN;
    out->f        = 0.0f;
    out->gcode    = 0;
    out->mcode    = 0;
    out->has_x    = false;
    out->has_y    = false;
    out->has_z    = false;
    out->has_w    = false;
    out->is_home       = false;
    out->is_status     = false;
    out->is_probe      = false;
    out->is_probe_zero = false;

    const char *p = skip_ws(line);

    /* Boş satır veya yorum (';' veya '(') */
    if (*p == '\0' || *p == ';' || *p == '(') { return false; }
    if (*p == '\r' || *p == '\n')              { return false; }

    /* Satır numarası: "N<sayı>" → atla */
    if (*p == 'N' || *p == 'n') {
        p++;
        while (isdigit((unsigned char)*p)) { p++; }
        p = skip_ws(p);
    }

    bool any_cmd = false;  /* en az bir komut kelimesi bulundu mu */

    /*
     * Karakterleri döngüyle tara.
     * GRBL birden fazla kelimeyi tek satırda destekler: "G0 X1.0 M3 F200"
     */
    while (*p != '\0' && *p != ';' && *p != '(' && *p != '\r' && *p != '\n') {
        p = skip_ws(p);
        if (*p == '\0' || *p == ';' || *p == '(' ||
            *p == '\r' || *p == '\n') { break; }

        char ch = (char)toupper((unsigned char)*p);

        /* Anlık durum sorgusu */
        if (ch == '?') {
            out->is_status = true;
            any_cmd = true;
            p++;
            continue;
        }

        /* Sayısal parametre içeren harfler: G, M, X, Y, Z, W, F */
        if (ch == 'G' || ch == 'M' || ch == 'X' || ch == 'Y' ||
            ch == 'Z' || ch == 'W' || ch == 'F') {
            p++;  /* harfi geç */

            /* Sayıyı oku (strtof ile, işaret dahil) */
            char *endptr = NULL;
            float val = strtof(p, &endptr);
            if (endptr == p) {
                /* Sayı yok — geçersiz; karakteri atla */
                p++;
                continue;
            }
            p = endptr;
            any_cmd = true;

            switch (ch) {
                case 'G': {
                    int gnum = (int)val;
                    out->gcode = (uint8_t)gnum;
                    if (gnum == 28) { out->is_home = true; }
                    if (fabsf(val - 38.2f) < 0.05f) { out->is_probe = true; }
                    if (fabsf(val - 38.3f) < 0.05f) { out->is_probe = true; out->is_probe_zero = true; }
                    break;
                }
                case 'M': {
                    out->mcode = (uint8_t)(int)val;
                    break;
                }
                case 'X': {
                    out->x     = val;
                    out->has_x = true;
                    break;
                }
                case 'Y': {
                    out->y     = val;
                    out->has_y = true;
                    break;
                }
                case 'Z': {
                    out->z     = val;
                    out->has_z = true;
                    break;
                }
                case 'W': {
                    out->w     = val;
                    out->has_w = true;
                    break;
                }
                case 'F': {
                    out->f = val;
                    break;
                }
                default:
                    break;
            }
            continue;
        }

        /* Tanınmayan harf — atla */
        p++;
    }

    return any_cmd;
}

/* ------------------------------------------------------------------ */
/*  GCode_FormatStatus                                                 */
/* ------------------------------------------------------------------ */
void GCode_FormatStatus(char       *buf,
                        uint8_t     buflen,
                        const char *state_str,
                        float       x_mm,
                        float       y_mm,
                        float       z_mm,
                        float       w_mm)
{
    if (!buf || buflen == 0) { return; }

    snprintf(buf, (size_t)buflen,
             "<%.4s|MPos:%.3f,%.3f,%.3f,%.3f|FS:0,0>\n",
             state_str ? state_str : "Idle",
             (double)x_mm, (double)y_mm, (double)z_mm, (double)w_mm);
}
