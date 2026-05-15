#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include "pinnames.h"
#include "bsp_hal.h"
#include "qpc.h"
#include "axis.h"
#include <string.h>
#include <stdio.h>

#define BSP_TICKS_PER_SEC    			1000U

enum Signals {
    DUMMY_SIG = Q_USER_SIG,
    MAX_PUB_SIG,          /* 5 the last published signal */

		TIMEOUT_SIG,

		ZLIMIT1_PRESSED_SIG,
		ZLIMIT1_PRESSED_LONG_SIG,
		ZLIMIT1_RELEASED_SIG,

		ZLIMIT2_PRESSED_SIG,
		ZLIMIT2_PRESSED_LONG_SIG,
		ZLIMIT2_RELEASED_SIG,

		WLIMIT1_PRESSED_SIG,
		WLIMIT1_PRESSED_LONG_SIG,
		WLIMIT1_RELEASED_SIG,

		WLIMIT2_PRESSED_SIG,
		WLIMIT2_PRESSED_LONG_SIG,
		WLIMIT2_RELEASED_SIG,

    UART_RX_SIG,

    /* GCodeAO → MotionAO */
    GCODE_CMD_SIG,        /* ayrıştırılmış hareket komutu — GCodeEvt      */
    GCODE_STATUS_SIG,     /* '?' anlık durum sorgusu                       */

    /* MotionAO → GCodeAO: PC'ye gönderilecek string */
    GCODE_RSP_SIG,        /* GCodeRspEvt                                   */

    /* AxisCommAO ↔ MotionAO */
    AXIS_COMM_CMD_SIG,    /* MotionAO → AxisCommAO: X/Y hareketi başlat   */
    AXIS_XY_DONE_SIG,     /* AxisCommAO → MotionAO: Axis CPU "DONE\n"     */
    AXIS_XY_ERROR_SIG,    /* AxisCommAO → MotionAO: timeout veya "ER\n"   */

    /* MotionAO iç periyodik kontrol */
    MOTION_TICK_SIG,      /* QTimeEvt, 20ms                                */

    /* AxisCommAO: UART1 RX karakteri (Faz 3'te kullanılacak) */
    UART_AXIS_RX_SIG,     /* UartEvt yapısı, UART_RX_SIG ile aynı tip     */

    MAX_SIG               /* the last signal */
};


/* board pin definition */
enum {
	/* inputs */
	pinLIMIT1Z = 0,				//Z_LIM1
	pinLIMIT2Z,						//Z_LIM2
	pinLIMIT1W,						//W_LIM1
	pinLIMIT2W,						//W_LIM2
	/* outputs */
	pinSSR1,							//SSR1
	pinSSR2,							//SSR2
	pinSSR3,							//SSR3
	pinSSR4,							//SSR4
	pinSSR5,							//SSR5
	pinSSR6,							//SSR6
};

/* outs defininiton */
enum {
	SSR1	= 0,
	SSR2,
	SSR3,
	SSR4,
};

typedef struct {
	gpio_t 				*gpio;
	uint8_t 			activePinState;
	KeyStatus_t 	status;
	uint16_t 			sig;
	uint16_t 			tick;
}key_t;


typedef void (*FunctionOutSet)(const void* const me,uint8_t val);
typedef struct {
	void 						*out;
	FunctionOutSet	set;
}out_t;

#define SET_PIN_STATUS(pin,x)						outs[pin].set(&outs[pin],x)

extern uart_t uart_debug;

void debug_usb(char *format, ...);
void CDC_SendFmt(const char *fmt, ...);

void BSP_init(void);
void BSP_pin_set(const void* const me, uint8_t val);
void BSP_cli_puts(char* buf);
void BSP_cli_transmit(char* buf, int length);

/* Z ekseni BSP — doğrudan erişim (eski kod uyumluluğu) */
void BSP_AXIS_z_reset_pos(void);
void BSP_AXIS_Z_qei_init(void);
void BSP_AXIS_Z_pwm_init(uint32_t u32Freq);
void BSP_AXIS_Z_adc_Init(void);
void BSP_AXIS_Z_enable(void);
void BSP_AXIS_Z_disable(void);
void BSP_AXIS_Z_set_duty(int32_t duty);

#ifdef __cplusplus
}
#endif
