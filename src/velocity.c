#include "velocity.h"

/*-------------------------------------------------
 * 5kHz timer ISR içinde çagrilir
 *------------------------------------------------*/
void VelMeasure_Update(VelMeasure_t *vm, int32_t qei_count)
{
    int32_t delta = qei_count - vm->prev_count;
    vm->prev_count = qei_count;

    /* count/s: delta / dt (dt = 200µs = 0.0002s) */
    float vel_raw = (float)delta / (VEL_SAMPLE_US * 1e-6f);

    /* Low-pass filtre: gürültü azalt */
    vm->vel_filtered = vm->lpf_alpha * vel_raw
                     + (1.0f - vm->lpf_alpha) * vm->vel_filtered;

    vm->velocity_cps = vm->vel_filtered;
    vm->velocity_mms = CPS_TO_MMS(vm->vel_filtered);
    vm->velocity_rpm = vm->vel_filtered * 60.0f / COUNTS_PER_REV_F;
}
