#include "NuMicro.h"
#include "bsp.h"
#include "motor.h"

/*-------------------------------------------------
 * TIMER2 ISR — 5kHz Kontrol Döngüsü
 *
 * Tüm eksen mantığı AllAxes_ControlTick() içinde.
 * Z ve W eksenleri sırayla servis edilir (~4µs toplam,
 * 200µs periyotta %2 yük).
 *------------------------------------------------*/
void TMR2_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER2))
    {
        AllAxes_ControlTick();
        TIMER_ClearIntFlag(TIMER2);
    }
}

/*-------------------------------------------------
 * ControlTimer_Init — 5kHz periyodik zamanlayıcı
 *------------------------------------------------*/
void ControlTimer_Init(void)
{
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_EnableModuleClock(TMR2_MODULE);

    /* 5kHz = 200µs periyot */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 5000);
    TIMER_EnableInt(TIMER2);
    NVIC_SetPriority(TMR2_IRQn, 1);  /* EADC ISR'den düşük öncelik */
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);
}
