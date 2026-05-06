#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float measured;
    float previous;
    float filtered;
    float filter;
    float ifilter;
} FILTER_TypeDef;

void FILTER_calculate(float sample_freq, float cutoff_freq, FILTER_TypeDef *value);
void FILTER_update_value(FILTER_TypeDef *value);
void FILTER_reset_value(FILTER_TypeDef *value);

#if defined(__cplusplus)
}
#endif

