#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include "pinnames.h"
#include "bsp_hal.h"
#include "qpc.h"
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

void BSP_init(void);
void BSP_pin_set(const void* const me, uint8_t val);
void BSP_uart_puts(uart_t* uart, char* buf);

#ifdef __cplusplus
}
#endif