#include "bsp_hal.h"

extern uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map);
extern uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map);

static const uint32_t MODES[] = {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT_PP,
	GPIO_MODE_AF_PP,
	GPIO_MODE_AF_PP,
	GPIO_MODE_ANALOG
};

static const uint32_t PULLS[] = {
	GPIO_NOPULL,
	GPIO_PULLUP,
	GPIO_PULLDOWN
};

static const uint32_t OUTTYPES[] = {
	GPIO_MODE_OUTPUT_PP,
	GPIO_MODE_OUTPUT_OD,
	GPIO_MODE_OUTPUT_PP,
	GPIO_MODE_OUTPUT_PP
};


static uint32_t gpio_periph_clock_enable(const PinName pin){
	uint8_t port = PORT_GET(pin);
	switch(port) {
	#if defined(GPIOA)
		case 0:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			return GPIOA_BASE;
	#endif
	#if defined(GPIOB)
		case 1:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			return GPIOB_BASE;
	#endif
	#if defined(GPIOC)
		case 2:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			return GPIOC_BASE;
	#endif
	#if defined(GPIOD)
		case 3:
			__HAL_RCC_GPIOD_CLK_ENABLE();
			return GPIOD_BASE;
	#endif
	#if defined(GPIOE)
		case 4:
			__HAL_RCC_GPIOE_CLK_ENABLE();
			return GPIOE_BASE;
	#endif
	#if defined(GPIOF)
		case 5:
			__HAL_RCC_GPIOF_CLK_ENABLE();
			return GPIOF_BASE;
	#endif
	}
	return 0;
}


void bsp_hal_gpio_init(gpio_t * const gpio,const uint8_t initial_pin_state) {
	
		GPIO_InitTypeDef GPIO_InitStruct;
	
		/* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK */
		gpio->addr = gpio_periph_clock_enable(gpio->pin);
		
		/* GET PIN MASK */
		gpio->mask = 1 << PIN_GET(gpio->pin);
	
		/* SET INITIAL PIN STATE */
		if (initial_pin_state!=2)  bsp_hal_gpio_write(gpio,initial_pin_state);

	
		GPIO_InitStruct.Pin = gpio->mask;
		GPIO_InitStruct.Mode = MODES[gpio->mode];
		if (gpio->mode == HAL_GPIO_MODE_OUT) GPIO_InitStruct.Mode |=  OUTTYPES[gpio->outtype];
		GPIO_InitStruct.Pull = PULLS[gpio->pull];
		HAL_GPIO_Init((GPIO_TypeDef*)(((gpio_t*)gpio)->addr), &GPIO_InitStruct);
}


