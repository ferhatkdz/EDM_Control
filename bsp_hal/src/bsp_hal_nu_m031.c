#include "bsp_hal.h"




extern uint32_t pinmap_merge(uint32_t a, uint32_t b);
extern uint32_t pinmap_peripheral(PinName pin, const PinMap *map);
extern uint32_t pinmap_function(PinName pin, const PinMap *map);
extern uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map);
extern uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map);

static UartDataReceivedIRQhandler uartDataReceivedIRQhandler = 0;

static const uint32_t MODES[] = {
		GPIO_MODE_INPUT,
		GPIO_MODE_OUTPUT,
	  GPIO_MODE_OPEN_DRAIN,
	  GPIO_MODE_QUASI
};

/* m031 no pull control */
static const uint32_t PULLS[] = {
		0,
		0,
		0
};

const uint32_t DATABITS[] = {
		UART_WORD_LEN_5,
		UART_WORD_LEN_6,
		UART_WORD_LEN_7,
		UART_WORD_LEN_8,
		UART_WORD_LEN_8
};

const uint32_t STOPBITS[] = {
		UART_STOP_BIT_1,
		UART_STOP_BIT_1,
		UART_STOP_BIT_2,
		UART_STOP_BIT_1_5
};

const uint32_t PARITY[] = {
		UART_PARITY_NONE,
		UART_PARITY_EVEN,
		UART_PARITY_ODD
};

static uint32_t gpio_periph_clock_enable(const PinName pin){
	uint8_t port = PORT_GET(pin);
	switch(port) {
#if defined(_PA0)
	case 0:
		//CLK_EnableModuleClock(GPA_MODULE);
		return PA_BASE;
#endif
#if defined(_PB0)
	case 1:
		//CLK_EnableModuleClock(GPB_MODULE);
		return PB_BASE;
#endif
#if defined(_PC0)
	case 2:
		//CLK_EnableModuleClock(GPC_MODULE);
		return PC_BASE;
#endif
#if defined(_PD0)
	case 3:
		//CLK_EnableModuleClock(GPD_MODULE);
		return PD_BASE;
#endif
#if defined(_PE0)
	case 4:
		//CLK_EnableModuleClock(GPE_MODULE);
		return PE_BASE;
#endif
#if defined(_PF0)
	case 5:
		//CLK_EnableModuleClock(GPF_MODULE);
		return PF_BASE;
#endif
	}
	return 0;
}

uint32_t uart_irq_number_get(const uint8_t uart_index) 
{
	uint32_t irq_number = 0;
	switch (uart_index) {
#if defined(UART0)
	case UART_IND0:
		irq_number = UART02_IRQn;
		break;
#endif
#if defined(UART1)
	case UART_IND1:
		irq_number = UART13_IRQn;
		break;
#endif
#if defined(UART2)
	case UART_IND2:
		irq_number = UART02_IRQn;
		break;
#endif
#if defined(UART3)
	case UART_IND3:
		irq_number = UART13_IRQn;
		break;
#endif
	default:
		break;
	}
	return irq_number;
}

void bsp_hal_gpio_init(gpio_t * const gpio,const uint8_t initial_pin_state) {

	/* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK */
	gpio->addr = gpio_periph_clock_enable(gpio->pin);

	/* GET PIN MASK */
	gpio->mask = 1 << PIN_GET(gpio->pin);

	/* SET INITIAL PIN STATE */
	if (initial_pin_state != 2)  bsp_hal_gpio_write(gpio,initial_pin_state);

	/* SET MODE */
	if (gpio->outtype == HAL_GPIO_OTYPE_OD) GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, GPIO_MODE_OPEN_DRAIN);
	else if (gpio->outtype == HAL_GPIO_OTYPE_QUASI) GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, GPIO_MODE_QUASI);
	else GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, MODES[gpio->mode]);

	/* SET PULL MODE */
	//GPIO_SetPullCtl((GPIO_T*)gpio->addr, gpio->mask, PULLS[gpio->pull]);
}

void bsp_hal_gpio_init_initial(gpio_t * const gpio) {
	bsp_hal_gpio_init(gpio, gpio->initial);
}

/* PWM OUT PinMap */
const PinMap PinMap_PWM_OUT[] = {
		{.pin = PA5, 	.peripheral = 0, 	.channel = 0, .function = SYS_GPA_MFPL_PA5MFP_PWM0_CH0},
		{.pin = PB5, 	.peripheral = 0, 	.channel = 0, .function = SYS_GPB_MFPL_PB5MFP_PWM0_CH0},
		{.pin = PE7, 	.peripheral = 0, 	.channel = 0, .function = SYS_GPE_MFPL_PE7MFP_PWM0_CH0},
		{.pin = PE8, 	.peripheral = 0, 	.channel = 0, .function = SYS_GPE_MFPH_PE8MFP_PWM0_CH0},
		{.pin = PF5, 	.peripheral = 0, 	.channel = 0, .function = SYS_GPF_MFPL_PF5MFP_PWM0_CH0},

		{.pin = PA4, 	.peripheral = 0, 	.channel = 1, .function = SYS_GPA_MFPL_PA4MFP_PWM0_CH1},
		{.pin = PB4, 	.peripheral = 0, 	.channel = 1, .function = SYS_GPB_MFPL_PB4MFP_PWM0_CH1},
		{.pin = PE6, 	.peripheral = 0, 	.channel = 1, .function = SYS_GPE_MFPL_PE6MFP_PWM0_CH1},
		{.pin = PE9, 	.peripheral = 0, 	.channel = 1, .function = SYS_GPE_MFPH_PE9MFP_PWM0_CH1},
		{.pin = PF4, 	.peripheral = 0, 	.channel = 1, .function = SYS_GPF_MFPL_PF4MFP_PWM0_CH1},

		{.pin = PA3, 	.peripheral = 0, 	.channel = 2, .function = SYS_GPA_MFPL_PA3MFP_PWM0_CH2},
		{.pin = PB3, 	.peripheral = 0, 	.channel = 2, .function = SYS_GPB_MFPL_PB3MFP_PWM0_CH2},
		{.pin = PE5, 	.peripheral = 0, 	.channel = 2, .function = SYS_GPE_MFPL_PE5MFP_PWM0_CH2},
		{.pin = PE10, .peripheral = 0, 	.channel = 2, .function = SYS_GPE_MFPH_PE10MFP_PWM0_CH2},

		{.pin = PA2, 	.peripheral = 0, 	.channel = 3, .function = SYS_GPA_MFPL_PA2MFP_PWM0_CH3},
		{.pin = PB2, 	.peripheral = 0, 	.channel = 3, .function = SYS_GPB_MFPL_PB2MFP_PWM0_CH3},
		{.pin = PE4, 	.peripheral = 0, 	.channel = 3, .function = SYS_GPE_MFPL_PE4MFP_PWM0_CH3},
		{.pin = PE11, .peripheral = 0, 	.channel = 3, .function = SYS_GPE_MFPH_PE11MFP_PWM0_CH3},

		{.pin = PA1, 	.peripheral = 0, 	.channel = 4, .function = SYS_GPA_MFPL_PA1MFP_PWM0_CH4},
		{.pin = PB1, 	.peripheral = 0, 	.channel = 4, .function = SYS_GPB_MFPL_PB1MFP_PWM0_CH4},
		{.pin = PE3, 	.peripheral = 0, 	.channel = 4, .function = SYS_GPE_MFPL_PE3MFP_PWM0_CH4},
		{.pin = PE12, .peripheral = 0, 	.channel = 4, .function = SYS_GPE_MFPH_PE12MFP_PWM0_CH4},
		{.pin = PF14, .peripheral = 0, 	.channel = 4, .function = SYS_GPF_MFPH_PF14MFP_PWM0_CH4},

		{.pin = PA0, 	.peripheral = 0, 	.channel = 5, .function = SYS_GPA_MFPL_PA0MFP_PWM0_CH5},
		{.pin = PB0, 	.peripheral = 0, 	.channel = 5, .function = SYS_GPB_MFPL_PB0MFP_PWM0_CH5},
		{.pin = PD15, .peripheral = 0, 	.channel = 5, .function = SYS_GPD_MFPH_PD15MFP_PWM0_CH5},
		{.pin = PE2, 	.peripheral = 0, 	.channel = 5, .function = SYS_GPE_MFPL_PE2MFP_PWM0_CH5},
		{.pin = PE13, .peripheral = 0, 	.channel = 5, .function = SYS_GPE_MFPH_PE13MFP_PWM0_CH5},

		{.pin = PB15, .peripheral = 1, 	.channel = 0, .function = SYS_GPB_MFPH_PB15MFP_PWM1_CH0},
		{.pin = PC5, 	.peripheral = 1, 	.channel = 0, .function = SYS_GPC_MFPL_PC5MFP_PWM1_CH0},
		{.pin = PC12, .peripheral = 1, 	.channel = 0, .function = SYS_GPC_MFPH_PC12MFP_PWM1_CH0},
		{.pin = PE13, .peripheral = 1, 	.channel = 0, .function = SYS_GPE_MFPH_PE13MFP_PWM1_CH0},

		{.pin = PB14, .peripheral = 1, 	.channel = 1, .function = SYS_GPB_MFPH_PB14MFP_PWM1_CH1},
		{.pin = PC4,  .peripheral = 1, 	.channel = 1, .function = SYS_GPC_MFPL_PC4MFP_PWM1_CH1},
		{.pin = PC8,  .peripheral = 1, 	.channel = 1, .function = SYS_GPC_MFPH_PC8MFP_PWM1_CH1},
		{.pin = PC11, .peripheral = 1, 	.channel = 1, .function = SYS_GPC_MFPH_PC11MFP_PWM1_CH1},

		{.pin = PB13, .peripheral = 1, 	.channel = 2, .function = SYS_GPB_MFPH_PB13MFP_PWM1_CH2},
		{.pin = PC3,  .peripheral = 1, 	.channel = 2, .function = SYS_GPC_MFPL_PC3MFP_PWM1_CH2},
		{.pin = PC7,  .peripheral = 1, 	.channel = 2, .function = SYS_GPC_MFPL_PC7MFP_PWM1_CH2},
		{.pin = PC10, .peripheral = 1, 	.channel = 2, .function = SYS_GPC_MFPH_PC10MFP_PWM1_CH2},

		{.pin = PB12, .peripheral = 1, 	.channel = 3, .function = SYS_GPB_MFPH_PB12MFP_PWM1_CH3},
		{.pin = PC2,  .peripheral = 1, 	.channel = 3, .function = SYS_GPC_MFPL_PC2MFP_PWM1_CH3},
		{.pin = PC6,  .peripheral = 1, 	.channel = 3, .function = SYS_GPC_MFPL_PC6MFP_PWM1_CH3},
		{.pin = PC9,  .peripheral = 1, 	.channel = 3, .function = SYS_GPC_MFPH_PC9MFP_PWM1_CH3},

		{.pin = PA7,  .peripheral = 1, 	.channel = 4, .function = SYS_GPA_MFPL_PA7MFP_PWM1_CH4},
		{.pin = PB1,  .peripheral = 1, 	.channel = 4, .function = SYS_GPB_MFPL_PB1MFP_PWM1_CH4},
		{.pin = PB7,  .peripheral = 1, 	.channel = 4, .function = SYS_GPB_MFPL_PB7MFP_PWM1_CH4},
		{.pin = PC1,  .peripheral = 1, 	.channel = 4, .function = SYS_GPC_MFPL_PC1MFP_PWM1_CH4},

		{.pin = PA6,  .peripheral = 1, 	.channel = 5, .function = SYS_GPA_MFPL_PA6MFP_PWM1_CH5},
		{.pin = PB0,  .peripheral = 1, 	.channel = 5, .function = SYS_GPB_MFPL_PB0MFP_PWM1_CH5},
		{.pin = PB6,  .peripheral = 1, 	.channel = 5, .function = SYS_GPB_MFPL_PB6MFP_PWM1_CH5},
		{.pin = PC0,  .peripheral = 1, 	.channel = 5, .function = SYS_GPC_MFPL_PC0MFP_PWM1_CH5},

		{NC}
};

void bsp_hal_pwm_clk_init(int pwm_module_index){

	/* Unlock protected registers */
	SYS_UnlockReg();

	if (pwm_module_index & PWM0_INDEX) {
		/* Enable PWM module clock */
		CLK_EnableModuleClock(PWM0_MODULE);
		CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

		/* Reset PWM module */
		SYS_ResetModule(PWM0_RST);

	}

	if (pwm_module_index & PWM1_INDEX) {
		/* Enable PWM module clock */
		CLK_EnableModuleClock(PWM1_MODULE);
		CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

		/* Reset PWM module */
		SYS_ResetModule(PWM1_RST);
	}

	/* Update System Core Clock */
	SystemCoreClockUpdate();	

	/* Lock protected registers */
	SYS_LockReg();		
}

void bsp_hal_pwm_out_pin_init(pwm_out_t * const pwm_out) {
	timer_t *timer = pwm_out->timer;


	pwm_out->channel = pinmap_channel(pwm_out->pin,timer->index,(PinMap*)&PinMap_PWM_OUT);

	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Set multi-function pins for PWM x Channel y */
	uint32_t *reg = (uint32_t *)SYS + 12 + (PORT_GET(pwm_out->pin) * 2);
	*reg = (*reg & ~(0xful<<((PIN_GET(pwm_out->pin) % 8) * 4))) | pinmap_function_ex(pwm_out->pin,timer->index, (PinMap*)&PinMap_PWM_OUT);

	/* Lock protected registers */
	SYS_LockReg();

	if (timer->index == 0) {
		/* PWM channel n frequency is 2000Hz, duty 50%, */
		PWM_ConfigOutputChannel(PWM0, pwm_out->channel, timer->pwm_frequency, 0);

		/* Enable output of PWM0 channel n*/
		PWM_EnableOutput(PWM0, 1<<pwm_out->channel); //PWM_CH_0_MASK
	}else if (timer->index == 1) {
		/* PWM1 channel n frequency is 2000Hz, duty 50%, */
		PWM_ConfigOutputChannel(PWM1, pwm_out->channel, timer->pwm_frequency, 0);

		/* Enable output of PWM1 channel n */
		PWM_EnableOutput(PWM1, 1<<pwm_out->channel); //PWM_CH_0_MASK
	}
}

void bsp_hal_pwm_start(pwm_out_t * const pwm_out) {
	timer_t *timer = pwm_out->timer;
	if (timer->index == 0) {
		PWM_Start(PWM0, 1<<pwm_out->channel);
	}else if (timer->index == 1) {
		PWM_Start(PWM1, 1<<pwm_out->channel);
	}
}

void bsp_hal_pwm_out_pin_set_duty(const pwm_out_t* const pwm_out, uint16_t duty) {
	uint32_t u32CycleResolution = 100;
	uint32_t newCMR;
	PWM_T *pwm = (pwm_out->timer->index == 0) ? PWM0 : PWM1;

	if (duty >= u32CycleResolution) {
		newCMR = PWM_GET_CNR(pwm, pwm_out->channel);
	}else {
		newCMR = (duty * (PWM_GET_CNR(pwm, pwm_out->channel) + 1) / u32CycleResolution);
	}

	PWM_SET_CMR(pwm, pwm_out->channel, newCMR);

}



/* USART PinMap */
const PinMap PinMap_UART_TX[] = {
		{.pin =  PA9, .peripheral = UART_IND0, .function = HAL_GPIO_AF1},
		{NC}
};

const PinMap PinMap_UART_RX[] = {
		{.pin = PA10, .peripheral = UART_IND0, .function = HAL_GPIO_AF1},
		{NC}
};



static uint32_t uart_periph_clock_enable(const uint8_t index){
	uint32_t addr;
	uint32_t uart_rst;

	/* Unlock protected registers */
	SYS_UnlockReg();

	switch(index) {
#if defined(UART0)
	case 0:
		addr = UART0_BASE;
		uart_rst = UART0_RST;
		/* Switch UART0 clock source to HIRC */
		CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
		/* Enable UART peripheral clock */
		CLK_EnableModuleClock(UART0_MODULE);
		break;
#endif
#if defined(UART1)
	case 1:
		addr = UART1_BASE;
		uart_rst = UART1_RST;
		/* Switch UART1 clock source to HIRC */
		CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
		/* Enable UART peripheral clock */
		CLK_EnableModuleClock(UART1_MODULE);
		break;
#endif
#if defined(UART2)
	case 2:
		addr = UART2_BASE;
		uart_rst = UART2_RST;
		/* Switch UART2 clock source to HIRC */
		CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
		/* Enable UART peripheral clock */
		CLK_EnableModuleClock(UART2_MODULE);
		break;
#endif
	}

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
	SystemCoreClockUpdate();


	/* Lock protected registers */
	SYS_LockReg();

	/* Reset UART */
	SYS_ResetModule(uart_rst);

	return addr;
}



void bsp_hal_uart_init(uart_t * const uart) {

	uart->addr =  uart_periph_clock_enable(uart->index);

	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Set GPx multi-function pins for UART RXD and TXD */
	switch(uart->rxpin) {
	case PA0:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk))  | (SYS_GPA_MFPL_PA0MFP_UART0_RXD);break;
	case PA2:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk))  | (SYS_GPA_MFPL_PA2MFP_UART1_RXD);break;
	case PA4:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk))  | (SYS_GPA_MFPL_PA4MFP_UART0_RXD);break;
	case PA6:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk))  | (SYS_GPA_MFPL_PA6MFP_UART0_RXD);break;
	case PA8:  SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk))  | (SYS_GPA_MFPH_PA8MFP_UART1_RXD);break;
	case PA15: SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA15MFP_Msk)) | (SYS_GPA_MFPH_PA15MFP_UART0_RXD);break;
	case PC0:  SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk))  | (SYS_GPC_MFPL_PC0MFP_UART2_RXD);break;
	case PC4:  SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC4MFP_Msk))  | (SYS_GPC_MFPL_PC4MFP_UART2_RXD);break;
	case PF1:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF1MFP_Msk))  | (SYS_GPF_MFPL_PF1MFP_UART1_RXD);break;
	case PF2:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF2MFP_Msk))  | (SYS_GPF_MFPL_PF2MFP_UART0_RXD);break;
	case PF5:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF5MFP_Msk))  | (SYS_GPF_MFPL_PF5MFP_UART2_RXD);break;
	case PB0:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk))  | (SYS_GPB_MFPL_PB0MFP_UART2_RXD);break;
	case PB2:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk))  | (SYS_GPB_MFPL_PB2MFP_UART1_RXD);break;
	case PB4:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk))  | (SYS_GPB_MFPL_PB4MFP_UART2_RXD);break;
	case PB6:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB6MFP_Msk))  | (SYS_GPB_MFPL_PB6MFP_UART1_RXD);break;
	case PB12: SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk)) | (SYS_GPB_MFPH_PB12MFP_UART0_RXD);break;
	default:
		break;
	}

	switch(uart->txpin) {
	case PA1:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA1MFP_Msk))  | (SYS_GPA_MFPL_PA1MFP_UART0_TXD);break;
	case PA3:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk))  | (SYS_GPA_MFPL_PA3MFP_UART1_TXD);break;
	case PA5:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA5MFP_Msk))  | (SYS_GPA_MFPL_PA5MFP_UART0_TXD);break;
	case PA7:  SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA7MFP_Msk))  | (SYS_GPA_MFPL_PA7MFP_UART0_TXD);break;
	case PA9:  SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk))  | (SYS_GPA_MFPH_PA9MFP_UART1_TXD);break;
	case PA14: SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA14MFP_Msk)) | (SYS_GPA_MFPH_PA14MFP_UART0_TXD);break;
	case PC1:  SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_Msk))  | (SYS_GPC_MFPL_PC1MFP_UART2_TXD);break;
	case PC5:  SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk))  | (SYS_GPC_MFPL_PC5MFP_UART2_TXD);break;
	case PF0:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF0MFP_Msk))  | (SYS_GPF_MFPL_PF0MFP_UART1_TXD);break;
	case PF3:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF3MFP_Msk))  | (SYS_GPF_MFPL_PF3MFP_UART0_TXD);break;
	case PF4:  SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF4MFP_Msk))  | (SYS_GPF_MFPL_PF4MFP_UART2_TXD);break;
	case PB1:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_Msk))  | (SYS_GPB_MFPL_PB1MFP_UART2_TXD);break;
	case PB3:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB3MFP_Msk))  | (SYS_GPB_MFPL_PB3MFP_UART1_TXD);break;
	case PB5:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk))  | (SYS_GPB_MFPL_PB5MFP_UART2_TXD);break;
	case PB7:  SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB7MFP_Msk))  | (SYS_GPB_MFPL_PB7MFP_UART1_TXD);break;
	case PB13: SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB13MFP_Msk)) | (SYS_GPB_MFPH_PB13MFP_UART0_TXD);break;
	default:
		break;
	}

	/* Lock protected registers */
	SYS_LockReg();

	/* Configure UART */
	UART_OpenEx((UART_T *) uart->addr, uart->baudrate, DATABITS[uart->databits], PARITY[uart->parity], STOPBITS[uart->stopbits]);

}


void bsp_hal_uart_enable_rx_int(uart_t * const uart, const uint8_t priority) {

	IRQn_Type nvic_irq = (IRQn_Type)uart_irq_number_get(uart->index);

	/* set the priority and enable the selected IRQ */
	NVIC_SetPriority(nvic_irq, priority);
	NVIC_EnableIRQ(nvic_irq);

	UART_EnableInt((UART_T *) uart->addr, UART_INTEN_RDAIEN_Msk);

}

void bsp_hal_uart_set_rx_irqhandler(UartDataReceivedIRQhandler irqhandler) {
	if (irqhandler != 0) uartDataReceivedIRQhandler = irqhandler;
}

void bsp_hal_uart_transmit(uart_t * const uart, uint8_t * const data, const uint8_t length) {
	for (int i=0; i<length; i++) {
		UART_WRITE((UART_T *) uart->addr, data[i]);
		while(UART_IS_TX_FULL((UART_T *) uart->addr));
	}

	//while(!UART_GET_TX_EMPTY((UART_T *) uart->addr));
	
	UART_WAIT_TX_EMPTY((UART_T *) uart->addr);
}

void UART02_IRQHandler(void) 
{		
#if defined(UART0)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART0);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND0,data); 
	}
#endif

#if defined(UART2)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART2);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND2,data); 
	}
#endif
}


void UART13_IRQHandler(void) 
{		
#if defined(UART1)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART1);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND1,data); 
	}
#endif

#if defined(UART3)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART3, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART3);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND3,data); 
	}
#endif
}

