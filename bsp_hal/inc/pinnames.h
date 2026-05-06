#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>

/* Defines mode types of GPIO pin */
typedef enum {
	HAL_GPIO_MODE_IN = 0,
	HAL_GPIO_MODE_OUT,
	HAL_GPIO_MODE_AF_IN,
	HAL_GPIO_MODE_AF_OUT,
	HAL_GPIO_MODE_ANALOG
} HalGpioMode;

typedef enum {
	HAL_GPIO_PULL_NONE = 0,
	HAL_GPIO_PULL_UP,
	HAL_GPIO_PULL_DOWN
} HalGpioPull;

typedef enum {
	HAL_GPIO_OTYPE_PP = 0,
	HAL_GPIO_OTYPE_OD,
	HAL_GPIO_OTYPE_NONE,
	HAL_GPIO_OTYPE_QUASI
} HalGpioOuttype;

typedef enum {
	HAL_GPIO_AF0 = 0,
	HAL_GPIO_AF1,
	HAL_GPIO_AF2,
	HAL_GPIO_AF3,
	HAL_GPIO_AF4,
	HAL_GPIO_AF5,
	HAL_GPIO_AF6,
	HAL_GPIO_AF7,
	HAL_GPIO_AFNONE = 0xff
} HalGpioAFType;

typedef enum {
	HAL_UART_DATA5 = 0,
	HAL_UART_DATA6,
	HAL_UART_DATA7,
	HAL_UART_DATA8,
	HAL_UART_DATA9
} HalUartDatabit;

typedef enum {
	HAL_UART_PARITY_NONE = 0,
	HAL_UART_PARITY_EVEN,
	HAL_UART_PARITY_ODD
} HalUartParity;

typedef enum {
	HAL_UART_STOP_1 = 0,
	HAL_UART_STOP_0_5,
	HAL_UART_STOP_2,
	HAL_UART_STOP_1_5
} HalUartStop;

enum {
	UART_IND0 = 0,
	UART_IND1,
	UART_IND2,
	UART_IND3,
	UART_IND4,
	UART_IND5,
	UART_IND6,
	UART_IND7
};

typedef enum {
	HAL_TIMER_OC_MODE_TIMING = 0,
	HAL_TIMER_OC_MODE_ACTIVE,
	HAL_TIMER_OC_MODE_INACTIVE,
	HAL_TIMER_OC_MODE_TOGGLE,
	HAL_TIMER_OC_MODE_LOW,
	HAL_TIMER_OC_MODE_HIGH,
	HAL_TIMER_OC_MODE_PWM0,
	HAL_TIMER_OC_MODE_PWM1
}HalTimerOCMode;

typedef enum {
	HAL_TIMER_IC_POLARITY_RISING = 0,
	HAL_TIMER_IC_POLARITY_FALLING,
	HAL_TIMER_IC_POLARITY_BOTH_EDGE
}HalTimerICPolarity;


typedef enum {
	HAL_TIMER_IC_SELECTION_DIRECTTI = 0,
	HAL_TIMER_IC_SELECTION_INDIRECTTI,
	HAL_TIMER_IC_SELECTION_ITS
}HalTimerICSelection;

typedef enum {
	HAL_TIMER_TRG_INTERNAL0 = 0,
	HAL_TIMER_TRG_INTERNAL1,
	HAL_TIMER_TRG_INTERNAL2,
	HAL_TIMER_TRG_INTERNAL3,
	HAL_TIMER_TRG_EDGE_DETECTOR,
	HAL_TIMER_TRG_TIMER_INPUT0,
	HAL_TIMER_TRG_TIMER_INPUT1,
	HAL_TIMER_TRG_EXTERNAL
}HalTimerInputTriggerSource;

typedef enum {
	HAL_TIMER_SLAVE_MODE_DISABLE = 0,
	HAL_TIMER_ENCODER_MODE0,
	HAL_TIMER_ENCODER_MODE1,
	HAL_TIMER_ENCODER_MODE2,
	HAL_TIMER_SLAVE_MODE_RESTART,
	HAL_TIMER_SLAVE_MODE_PAUSE,
	HAL_TIMER_SLAVE_MODE_EVENT,
	HAL_TIMER_SLAVE_MODE_EXTERNAL0
}HalTimerSlaveMode;

typedef enum {
	HAL_TIMER_INT_UPDATE,
	HAL_TIMER_INT_CH0,
	HAL_TIMER_INT_CH1,
	HAL_TIMER_INT_CH2,
	HAL_TIMER_INT_CH3,
}HalInterrupt;

enum {
	TIMER_IND0 = 0,
	TIMER_IND1,
	TIMER_IND2,
	TIMER_IND3,
	TIMER_IND4,
	TIMER_IND5,
	TIMER_IND6,
	TIMER_IND7,
	TIMER_IND8,
	TIMER_IND9,
	TIMER_IND10,
	TIMER_IND11,
	TIMER_IND12,
	TIMER_IND13,
	TIMER_IND14,
	TIMER_IND15,
	TIMER_IND16,
};

#define PORT_GET(X) (((uint32_t)(X) >> 4) & 0xF)
#define PIN_GET(X)  (((uint32_t)(X) & 0xF))


typedef enum {
	PA0 = 0X00,
	PA1 = 0X01,
	PA2 = 0X02,
	PA3 = 0X03,
	PA4 = 0X04,
	PA5 = 0X05,
	PA6 = 0X06,
	PA7 = 0X07,
	PA8 = 0X08,
	PA9 = 0X09,
	PA10 = 0X0A,
	PA11 = 0X0B,
	PA12 = 0X0C,
	PA13 = 0X0D,
	PA14 = 0X0E,
	PA15 = 0X0F,

	PB0 = 0X10,
	PB1 = 0X11,
	PB2 = 0X12,
	PB3 = 0X13,
	PB4 = 0X14,
	PB5 = 0X15,
	PB6 = 0X16,
	PB7 = 0X17,
	PB8 = 0X18,
	PB9 = 0X19,
	PB10 = 0X1A,
	PB11 = 0X1B,
	PB12 = 0X1C,
	PB13 = 0X1D,
	PB14 = 0X1E,
	PB15 = 0X1F,

	PC0 = 0X20,
	PC1 = 0X21,
	PC2 = 0X22,
	PC3 = 0X23,
	PC4 = 0X24,
	PC5 = 0X25,
	PC6 = 0X26,
	PC7 = 0X27,
	PC8 = 0X28,
	PC9 = 0X29,
	PC10 = 0X2A,
	PC11 = 0X2B,
	PC12 = 0X2C,
	PC13 = 0X2D,
	PC14 = 0X2E,
	PC15 = 0X2F,

	PD0 = 0X30,
	PD1 = 0X31,
	PD2 = 0X32,
	PD3 = 0X33,
	PD4 = 0X34,
	PD5 = 0X35,
	PD6 = 0X36,
	PD7 = 0X37,
	PD8 = 0X38,
	PD9 = 0X39,
	PD10 = 0X3A,
	PD11 = 0X3B,
	PD12 = 0X3C,
	PD13 = 0X3D,
	PD14 = 0X3E,
	PD15 = 0X3F,

	PE0 = 0X40,
	PE1 = 0X41,
	PE2 = 0X42,
	PE3 = 0X43,
	PE4 = 0X44,
	PE5 = 0X45,
	PE6 = 0X46,
	PE7 = 0X47,
	PE8 = 0X48,
	PE9 = 0X49,
	PE10 = 0X4A,
	PE11 = 0X4B,
	PE12 = 0X4C,
	PE13 = 0X4D,
	PE14 = 0X4E,
	PE15 = 0X4F,

	PF0 = 0X50,
	PF1 = 0X51,
	PF2 = 0X52,
	PF3 = 0X53,
	PF4 = 0X54,
	PF5 = 0X55,
	PF6 = 0X56,
	PF7 = 0X57,
	PF8 = 0X58,
	PF9 = 0X59,
	PF10 = 0X5A,
	PF11 = 0X5B,
	PF12 = 0X5C,
	PF13 = 0X5D,
	PF14 = 0X5E,
	PF15 = 0X5F,
	NC = 0xFF
} PinName;

typedef struct {
	PinName 			pin;
	uint32_t 			peripheral;
	uint32_t			channel;
	uint32_t 			function;
} PinMap;


enum {
	PIN_RESET = 0,
	PIN_SET
};

typedef enum {
	KEY_DOWN = 0,
	KEY_UP,
	KEY_PRESSED,
	KEY_RELEASED
}KeyStatus_t;



typedef struct gpio_pin_t gpio_pin_t;


typedef void (*UartDataReceivedIRQhandler)(uint8_t uart_index, uint8_t data);
typedef void (*TimerUpdateIRQHandler)(uint8_t timer_index);
typedef void (*TimerChannelIRQHandler)(uint8_t timer_index, uint8_t channel_index);


typedef struct {
	PinName  				pin;
	HalGpioMode		 	mode;
	HalGpioPull		 	pull;
	HalGpioOuttype	outtype;	
	HalGpioAFType	 	aftype; 
	unsigned int 		mask;
	unsigned int 		addr;
	uint8_t					initial;
}gpio_t;

typedef struct {
	PinName  				rxpin;
	PinName  				txpin;
	PinName  				depin;
	uint32_t				baudrate;
	HalUartDatabit	databits;
	HalUartParity		parity;
	HalUartStop			stopbits;
	uint8_t					index;
	unsigned int 		addr;
}uart_t;

typedef struct {
	uint32_t										prescaler;
	int32_t				  						period;
	uint8_t											index;
	uint32_t										pwm_frequency;
	HalInterrupt 								interrupt;
	unsigned int 								addr;
}timer_t;

typedef struct {
	PinName					pin;
	HalTimerOCMode	mode;
	timer_t					*timer;
	double					duty_mul;
	uint32_t				channel;
}pwm_out_t;

typedef struct {
	PinName					pin;
	int 						pwmModuleIndex;
	int 						pwmChannel;
	int 						timeNSec;
	int 						dmaChannel;
}NUVOTON_pwm_capture_t;

typedef struct {
	gpio_t								gpio;
	HalTimerICPolarity 		polarity;
	timer_t								*timer;
	NUVOTON_pwm_capture_t	NUVOTON_pwm_capture;
}pwm_in_t;


#if defined(__cplusplus)
}
#endif

