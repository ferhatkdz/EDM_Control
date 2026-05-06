#include "bsp_hal.h"

extern uint32_t pinmap_function(PinName pin, const PinMap *map);
extern uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map);
extern uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map);

static UartDataReceivedIRQhandler uartDataReceivedIRQhandler = 0;

static const uint32_t MODES[] = {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
};

static const uint32_t PULLS[] = {
	GPIO_PUSEL_DISABLE,
	GPIO_PUSEL_PULL_UP,
	GPIO_PUSEL_PULL_DOWN
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

uint32_t uart_irq_number_get(const uint8_t uart_index) 
{
	uint32_t irq_number = 0;
	switch (uart_index) {
#if defined(UART0)
	case UART_IND0:
		irq_number = UART0_IRQn;
		break;
#endif
#if defined(UART1)
	case UART_IND1:
		irq_number = UART1_IRQn;
		break;
#endif
#if defined(UART2)
	case UART_IND2:
		irq_number = UART2_IRQn;
		break;
#endif
#if defined(UART3)
	case UART_IND3:
		irq_number = UART3_IRQn;
		break;
#endif
	default:
		break;
	}
	return irq_number;
}

static uint32_t gpio_periph_clock_enable(const PinName pin){
	uint8_t port = PORT_GET(pin);
	switch(port) {
	#if defined(_PA0)
		case 0:
			CLK_EnableModuleClock(GPA_MODULE);
			return GPIOA_BASE;
	#endif
	#if defined(_PB0)
		case 1:
			CLK_EnableModuleClock(GPB_MODULE);
			return GPIOB_BASE;
	#endif
	#if defined(_PC0)
		case 2:
			CLK_EnableModuleClock(GPC_MODULE);
			return GPIOC_BASE;
	#endif
	#if defined(_PD0)
		case 3:
			CLK_EnableModuleClock(GPD_MODULE);
			return GPIOD_BASE;
	#endif
	#if defined(_PE0)
		case 4:
			CLK_EnableModuleClock(GPE_MODULE);
			return GPIOE_BASE;
	#endif
	#if defined(_PF0)
		case 5:
			CLK_EnableModuleClock(GPF_MODULE);
			return GPIOF_BASE;
	#endif
	}
	return 0;
}

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
		//CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
		/* Enable UART peripheral clock */
		CLK_EnableModuleClock(UART1_MODULE);
		break;
#endif
#if defined(UART2)
	case 2:
		addr = UART2_BASE;
		uart_rst = UART2_RST;
		/* Switch UART2 clock source to HIRC */
		//CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
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


void bsp_hal_gpio_init(gpio_t * const gpio,const uint8_t initial_pin_state) {
	
		/* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK */
		gpio->addr = gpio_periph_clock_enable(gpio->pin);
		
		/* GET PIN MASK */
		gpio->mask = 1 << PIN_GET(gpio->pin);
	
		/* SET INITIAL PIN STATE */
		if (initial_pin_state!=2)  bsp_hal_gpio_write(gpio,initial_pin_state);

		/* SET MODE */
		if (gpio->outtype == HAL_GPIO_OTYPE_OD) GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, GPIO_MODE_OPEN_DRAIN);
		else if (gpio->outtype == HAL_GPIO_OTYPE_QUASI) GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, GPIO_MODE_QUASI);
		else GPIO_SetMode((GPIO_T*)gpio->addr, gpio->mask, MODES[gpio->mode]);
	
		/* SET PULL MODE */
		GPIO_SetPullCtl((GPIO_T*)gpio->addr, gpio->mask, PULLS[gpio->pull]);
		
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

	
/* UART PinMap */
const PinMap PinMap_UART[] = {
    {.pin = PA0, 	.function = SYS_GPA_MFPL_PA0MFP_UART0_RXD},
    {.pin = PA1, 	.function = SYS_GPA_MFPL_PA1MFP_UART0_TXD},
		
    {.pin = PA4, 	.function = SYS_GPA_MFPL_PA4MFP_UART0_RXD},
		{.pin = PA5, 	.function = SYS_GPA_MFPL_PA5MFP_UART0_TXD},

    {.pin = PA6, 	.function = SYS_GPA_MFPL_PA6MFP_UART0_RXD},
    {.pin = PA7, 	.function = SYS_GPA_MFPL_PA7MFP_UART0_TXD},
		
    {.pin = PA15, .function = SYS_GPA_MFPH_PA15MFP_UART0_RXD},
    {.pin = PA14, .function = SYS_GPA_MFPH_PA14MFP_UART0_TXD},
		
		{.pin = PB8, 	.function = SYS_GPB_MFPH_PB8MFP_UART0_RXD},
		{.pin = PB9, 	.function = SYS_GPB_MFPH_PB9MFP_UART0_TXD},
		
    {.pin = PB12, .function = SYS_GPB_MFPH_PB12MFP_UART0_RXD},
    {.pin = PB13, .function = SYS_GPB_MFPH_PB13MFP_UART0_TXD},
		
		{.pin = PF2, 	.function = SYS_GPF_MFPL_PF2MFP_UART0_RXD},
		{.pin = PF3, 	.function = SYS_GPF_MFPL_PF3MFP_UART0_TXD},
		
    {.pin = PA2, 	.function = SYS_GPA_MFPL_PA2MFP_UART1_RXD},
		{.pin = PA3, 	.function = SYS_GPA_MFPL_PA3MFP_UART1_TXD},

    {.pin = PA8, 	.function = SYS_GPA_MFPH_PA8MFP_UART1_RXD},
    {.pin = PA9, 	.function = SYS_GPA_MFPH_PA9MFP_UART1_TXD},
    
		{.pin = PB2, 	.function = SYS_GPB_MFPL_PB2MFP_UART1_RXD},
    {.pin = PB3, 	.function = SYS_GPB_MFPL_PB3MFP_UART1_TXD},
    
		{.pin = PB6,  .function = SYS_GPB_MFPL_PB6MFP_UART1_RXD},
    {.pin = PB7,  .function = SYS_GPB_MFPL_PB7MFP_UART1_TXD},
		
		{.pin = PF1, 	.function = SYS_GPF_MFPL_PF1MFP_UART1_RXD},
		{.pin = PF0, 	.function = SYS_GPF_MFPL_PF0MFP_UART1_TXD},
	  
		{.pin = PC0, 	.function = SYS_GPC_MFPL_PC0MFP_UART2_RXD},
	  {.pin = PC1, 	.function = SYS_GPC_MFPL_PC1MFP_UART2_TXD},

    {.pin = PC4, 	.function = SYS_GPC_MFPL_PC4MFP_UART2_RXD},
    {.pin = PC5, 	.function = SYS_GPC_MFPL_PC5MFP_UART2_TXD},

    {.pin = PB0, 	.function = SYS_GPB_MFPL_PB0MFP_UART2_RXD},
    {.pin = PB1, 	.function = SYS_GPB_MFPL_PB1MFP_UART2_TXD},

    {.pin = PB4,  .function = SYS_GPB_MFPL_PB4MFP_UART2_RXD},
    {.pin = PB5,  .function = SYS_GPB_MFPL_PB5MFP_UART2_TXD},

		{.pin = PE15, .function = SYS_GPE_MFPH_PE15MFP_UART2_RXD},
		{.pin = PE14, .function = SYS_GPE_MFPH_PE14MFP_UART2_TXD},

		{.pin = PF5, 	.function = SYS_GPF_MFPL_PF5MFP_UART2_RXD},
		{.pin = PF4, 	.function = SYS_GPF_MFPL_PF4MFP_UART2_TXD},

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
	
		bsp_gpio_set_multi_function(pwm_out->pin, pinmap_function_ex(pwm_out->pin,timer->index, (PinMap*)&PinMap_PWM_OUT));
	
		//uint32_t *reg = (uint32_t *)SYS + 12 + (PORT_GET(pwm_out->pin) * 2) + (PIN_GET(pwm_out->pin) / 8) ;
		//*reg = (*reg & ~(0xful<<((PIN_GET(pwm_out->pin) % 8) * 4))) | pinmap_function_ex(pwm_out->pin,timer->index, (PinMap*)&PinMap_PWM_OUT);
		
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

void bsp_hal_pwm_set_frequency(pwm_out_t * const pwm_out, uint32_t frequency, uint32_t duty) {
		timer_t *timer = pwm_out->timer;
	
		pwm_out->channel = pinmap_channel(pwm_out->pin,timer->index,(PinMap*)&PinMap_PWM_OUT);
	
		
		if (timer->index == 0) {
			/* PWM channel n frequency is 2000Hz, duty 50%, */
			PWM_ConfigOutputChannel(PWM0, pwm_out->channel, frequency, duty);
		
			/* Enable output of PWM0 channel n*/
			PWM_EnableOutput(PWM0, 1<<pwm_out->channel); //PWM_CH_0_MASK
		}else if (timer->index == 1) {
			/* PWM1 channel n frequency is 2000Hz, duty 50%, */
			PWM_ConfigOutputChannel(PWM1, pwm_out->channel, frequency, duty);
		
			/* Enable output of PWM1 channel n */
			PWM_EnableOutput(PWM1, 1<<pwm_out->channel); //PWM_CH_0_MASK
		}
}

void bsp_hal_pwm_in_pin_init(pwm_in_t * const pwm_in) {
	
		PWM_T *PWM = (pwm_in->NUVOTON_pwm_capture.pwmModuleIndex == 0) ? PWM0 : PWM1;
		int channel = pinmap_channel(pwm_in->NUVOTON_pwm_capture.pin,pwm_in->NUVOTON_pwm_capture.pwmModuleIndex,(PinMap*)&PinMap_PWM_OUT);
		pwm_in->NUVOTON_pwm_capture.pwmChannel = channel;
	
		/* Unlock protected registers */
    SYS_UnlockReg();
		/* Set multi-function pins for PWM x Channel y */
	
		bsp_gpio_set_multi_function(pwm_in->NUVOTON_pwm_capture.pin,  pinmap_function_ex(pwm_in->NUVOTON_pwm_capture.pin,pwm_in->NUVOTON_pwm_capture.pwmModuleIndex, (PinMap*)&PinMap_PWM_OUT));
	
		//uint32_t *reg = (uint32_t *)SYS + 12 + (PORT_GET(pwm_in->NUVOTON_pwm_capture.pin) * 2) + (PIN_GET(pwm_in->NUVOTON_pwm_capture.pin) / 8) ;
		//*reg = (*reg & ~(0xful<<((PIN_GET(pwm_in->NUVOTON_pwm_capture.pin) % 8) * 4))) | pinmap_function_ex(pwm_in->NUVOTON_pwm_capture.pin,pwm_in->NUVOTON_pwm_capture.pwmModuleIndex, (PinMap*)&PinMap_PWM_OUT);
		/* Lock protected registers */
    SYS_LockReg();
	
		/* set PWMx channel n capture configuration */
		PWM_ConfigCaptureChannel(PWM, channel, pwm_in->NUVOTON_pwm_capture.timeNSec, 0);
		/* Enable Timer for PWMx channel n */
		PWM_Start(PWM, 1<<channel); //PWM_CH_n_MASK
		/* Enable Capture Function for PWMx channel n */
		PWM_EnableCapture(PWM, 1<<channel); //PWM_CH_n_MASK
		/* Enable falling capture reload */
    PWM->CAPCTL |= (0x1ul << (PWM_CAPCTL_FCRLDEN0_Pos+channel));
		/* clear pending capture interrupts */
		PWM_ClearCaptureIntFlag(PWM, channel, PWM_CAPTURE_INT_RISING_LATCH);
		PWM_ClearCaptureIntFlag(PWM, channel, PWM_CAPTURE_INT_FALLING_LATCH);
}

void bsp_hal_pwm_in_dma_init(pwm_in_t * const pwm_in, int *counter_value) {
		PWM_T *PWM = (pwm_in->NUVOTON_pwm_capture.pwmModuleIndex == 0) ? PWM0 : PWM1;
		int dmaChannel = pwm_in->NUVOTON_pwm_capture.dmaChannel;
		int pwmChannel = pinmap_channel(pwm_in->NUVOTON_pwm_capture.pin,pwm_in->NUVOTON_pwm_capture.pwmModuleIndex,(PinMap*)&PinMap_PWM_OUT);
		int pwmPair = pwmChannel/2;
	
		/* Unlock protected registers */
		SYS_UnlockReg();
		/* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);
		/* Lock protected registers */
    SYS_LockReg();
	
		/*--------------------------------------------------------------------------------------*/
		/* Configure PDMA peripheral mode form PWM to memory                                    */
		/*--------------------------------------------------------------------------------------*/
		/* Open Channel n */
		PDMA_Open(PDMA, (1<<dmaChannel ));
		/* Transfer width is half word(32 bit) and transfer count is 1 */
		PDMA_SetTransferCnt(PDMA, dmaChannel, PDMA_WIDTH_32, 1);
		/* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_u16Count array(increment) */
		PDMA_SetTransferAddr(PDMA, dmaChannel, (uint32_t)&PWM->PDMACAP[pwmPair], PDMA_SAR_FIX, (uint32_t)counter_value, PDMA_DAR_FIX);
			/* Select PDMA request source as PWM RX(PWM1 channel 0 should be PWM1 pair 1) */
		PDMA_SetTransferMode(PDMA, dmaChannel, PDMA_PWM1_P1_RX+pwmPair, FALSE, 0);
		/* Set PDMA as single request type for PWM */
		PDMA_SetBurstType(PDMA, dmaChannel, PDMA_REQ_SINGLE, PDMA_BURST_1);
		/* Enable PDMA for PWM0 channel 2 capture function, and set capture order as falling first, */
		/* And select capture mode as both rising and falling to do PDMA transfer. */
		PWM_EnablePDMA(PWM, pwmChannel, TRUE, PWM_CAPTURE_PDMA_RISING_LATCH);
		/* enable PDMA_INT_TRANS_DONE */
		PDMA_EnableInt(PDMA, dmaChannel, PDMA_INT_TRANS_DONE);
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
	uint32_t u32CycleResolution = 0xff; //255;
	uint32_t newCMR;
	PWM_T *pwm = (pwm_out->timer->index == 0) ? PWM0 : PWM1;
	
	if (duty >= u32CycleResolution) {
		newCMR = PWM_GET_CNR(pwm, pwm_out->channel);
	}else {
		newCMR = (duty * (PWM_GET_CNR(pwm, pwm_out->channel) + 1) / u32CycleResolution);
	}

	PWM_SET_CMR(pwm, pwm_out->channel, newCMR);
	
}

void bsp_hal_pwm_out_pin_set_duty_100(const pwm_out_t* const pwm_out, uint16_t duty) {
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


void bsp_hal_uart_init(uart_t * const uart) {

	
	uart->addr =  uart_periph_clock_enable(uart->index);

	/* Unlock protected registers */
	SYS_UnlockReg();

	
	/* Set GPx multi-function pins for UART RXD */
	bsp_gpio_set_multi_function(uart->rxpin, pinmap_function(uart->rxpin,(PinMap*)&PinMap_UART));
	//uint32_t *reg = (uint32_t *)SYS + 12 + (PORT_GET(uart->rxpin) * 2) + (PIN_GET(uart->rxpin) / 8) ;
	//*reg = (*reg & ~(0xful<<((PIN_GET(uart->rxpin) % 8) * 4))) | pinmap_function(uart->rxpin,(PinMap*)&PinMap_UART);

	/* Set GPx multi-function pins for UART TXD */
	bsp_gpio_set_multi_function(uart->txpin, pinmap_function(uart->txpin,(PinMap*)&PinMap_UART));
	//uint32_t *reg_tx = (uint32_t *)SYS + 12 + (PORT_GET(uart->txpin) * 2) + (PIN_GET(uart->txpin) / 8) ;
	//*reg_tx = (*reg_tx & ~(0xful<<((PIN_GET(uart->txpin) % 8) * 4))) | pinmap_function(uart->txpin,(PinMap*)&PinMap_UART);
	
	#if 0
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
	case PB8:  SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB8MFP_Msk))  | (SYS_GPB_MFPH_PB8MFP_UART0_RXD);break;
	case PB12: SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk)) | (SYS_GPB_MFPH_PB12MFP_UART0_RXD);break;
		
	case PE15: SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE15MFP_Msk)) | (SYS_GPE_MFPH_PE15MFP_UART2_RXD);break;
		
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
	case PB9:  SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB9MFP_Msk))  | (SYS_GPB_MFPH_PB9MFP_UART0_TXD);break;	
	case PB13: SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB13MFP_Msk)) | (SYS_GPB_MFPH_PB13MFP_UART0_TXD);break;
		
	case PE14: SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE14MFP_Msk)) | (SYS_GPE_MFPH_PE14MFP_UART2_TXD);break;

	default:
		break;
	}
	#endif
	
	switch(uart->depin) {
	case PD8:  SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD8MFP_Msk))  | (SYS_GPD_MFPH_PD8MFP_UART2_nRTS);break;
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

void UART0_IRQHandler(void) 
{		
#if defined(UART0)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART0);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND0,data); 
	}
#endif
}

void UART1_IRQHandler(void) 
{		
#if defined(UART1)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART1);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND1,data); 
	}
#endif
}

void UART2_IRQHandler(void) 
{		
#if defined(UART2)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART2);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND2,data); 
	}
#endif
}


void UART3_IRQHandler(void) 
{		
#if defined(UART3)
	/* Rx Ready INT */
	if(UART_GET_INT_FLAG(UART3, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
		uint8_t data =  UART_READ(UART3);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND3,data); 
	}
#endif
}



/* eeprom emulation */


/* Exported constants --------------------------------------------------------*/
/* Page size */



/* Pages 0 and 1 base and end addresses */ 
#define PAGE0_BASE_ADDRESS  (uint32_t)(EEPROM_START_ADDRESS + (uint16_t)0x0000)
#define PAGE0_END_ADDRESS   (uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1))

#define PAGE1_BASE_ADDRESS  (uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE)
#define PAGE1_END_ADDRESS   (uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1))


/* Used Flash pages for EEPROM emulation */
#define PAGE0    					0x00
#define PAGE1    					0x01

/* No valid page define */
#define NO_VALID_PAGE    	0xAB

/* Page status definitions */
#define EMPTY_DATA      	(uint32_t)0xFFFFFFFF      /* PAGE is marked to receive data */
#define RECEIVE_DATA      (uint32_t)0xEEEEEEEE      /* PAGE is marked to receive data */
#define VALID_PAGE       	(uint32_t)0x00000000      /* PAGE containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    (uint8_t)0x00
#define WRITE_IN_VALID_PAGE     (uint8_t)0x01

/* Page full define */
#define PAGE_FULL    (uint8_t)0x80
#define WRITE_ERR    (uint8_t)0x81


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variable used to store variable value in read sequence */
uint16_t DataVar = 0 ;

/* Virtual address defined by the user: 0xFFFF value is prohibited */

extern const uint16_t NumbOfVar; /* Variables' number */
extern const uint16_t VirtAddVarTab[];


/* Private function prototypes -----------------------------------------------*/
extern void    FLASH_PageErase(uint32_t PageAddress);
/* Private functions ---------------------------------------------------------*/
static uint8_t EE_FindValidPage(uint8_t operation);
static uint8_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static uint8_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);

/*******************************************************************************
* Function Name  : EE_Format
* Description    : Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
* Input          : None
* Output         : None
* Return         : Status of the last operation (Flash write or erase) done during
*                  EEPROM formating
*******************************************************************************/
void EE_Format(void)
{
	// Erase page 0
  FMC_Erase(PAGE0_BASE_ADDRESS);

	// Erase page 1
  FMC_Erase(PAGE1_BASE_ADDRESS);
	
	/* set page0 as valid page word1 */
	FMC_Write(PAGE0_BASE_ADDRESS, VALID_PAGE);

	/* set page0 as valid page word2 */
	FMC_Write(PAGE0_BASE_ADDRESS+4, VALID_PAGE);
}


/*******************************************************************************
* Function Name  : EE_Format
* Description    : Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
* Input          : None
* Output         : None
* Return         : Status of the last operation (Flash write or erase) done during
*                  EEPROM formating
*******************************************************************************/
void EE_Erase(uint32_t PageBaseAddress)
{
	/* erase target page */
	FMC_Erase(PageBaseAddress);
}

/*******************************************************************************
* Function Name  : EE_FindValidPage
* Description    : Find valid Page for write or read operation
* Input          : - Operation: operation to achieve on the valid page:
*                      - READ_FROM_VALID_PAGE: read operation from valid page
*                      - WRITE_IN_VALID_PAGE: write operation from valid page
* Output         : None
* Return         : Valid page number (PAGE0 or PAGE1) or NO_VALID_PAGE in case
*                  of no valid page was found
*******************************************************************************/
uint8_t EE_FindValidPage(uint8_t Operation)
{
  uint32_t PageStatus0 = EMPTY_DATA,PageStatus00, PageStatus01;
	uint32_t PageStatus1 = EMPTY_DATA,PageStatus10, PageStatus11;

  /* Get Page0 actual status */
  PageStatus00 = FMC_Read(PAGE0_BASE_ADDRESS);
  PageStatus01 = FMC_Read(PAGE0_BASE_ADDRESS+4);
	if ( (PageStatus00 == VALID_PAGE) &&  (PageStatus01 == VALID_PAGE)) PageStatus0 = VALID_PAGE;
	else if ( (PageStatus00 == VALID_PAGE) &&  (PageStatus01 == EMPTY_DATA)) PageStatus0 = RECEIVE_DATA;
	
  /* Get Page1 actual status */
  PageStatus10 = FMC_Read(PAGE1_BASE_ADDRESS);
  PageStatus11 = FMC_Read(PAGE1_BASE_ADDRESS+4);
	if ( (PageStatus10 == VALID_PAGE) &&  (PageStatus11 == VALID_PAGE)) PageStatus1 = VALID_PAGE;
	else if ( (PageStatus10 == VALID_PAGE) &&  (PageStatus11 == EMPTY_DATA)) PageStatus1 = RECEIVE_DATA;
	

  /* Write or read operation */
  switch (Operation)
  {
    case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
      if (PageStatus1 == VALID_PAGE)
      {
        /* Page0 receiving data */
        if (PageStatus0 == RECEIVE_DATA)
        {
          return PAGE0;   /* Page0 valid */
        }
        else
        {
          return PAGE1;   /* Page1 valid */
        }
      }else if (PageStatus0 == VALID_PAGE)
      {
        /* Page1 receiving data */
        if (PageStatus1 == RECEIVE_DATA)
        {
          return PAGE1;      /* Page1 valid */
        }
        else
        {
          return PAGE0;      /* Page0 valid */
        }
      }
      else
      {
        return NO_VALID_PAGE;   /* No valid Page */
      }

    case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
      if (PageStatus0 == VALID_PAGE)
      {
        return PAGE0;           /* Page0 valid */
      }
      else if (PageStatus1 == VALID_PAGE)
      {
        return PAGE1;           /* Page1 valid */
      }
      else
      {
        return NO_VALID_PAGE ;  /* No valid Page */
      }

    default:
      return PAGE0;       /* Page0 valid */
  }
}

/*******************************************************************************
* Function Name  : EE_VerifyPageFullWriteVariable
* Description    : Verify if active page is full and Writes variable in EEPROM.
* Input          : - VirtAddress: 16 bit virtual address of the variable
*                  - Data: 16 bit data to be written as variable value
* Output         : None
* Return         : - Success or error status:
*                      - FLASH_COMPLETE: on success
*                      - PAGE_FULL: if valid page is full
*                      - NO_VALID_PAGE: if no valid page was found
*                      - Flash error code: on write Flash error
*******************************************************************************/
uint8_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  uint32_t ValidPage = PAGE0;
  uint32_t Address = 0, PageEndAddress = 0;
	uint32_t wdata;

  /* Get valid Page for write operation */
  ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE)
  {
    return  NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));
	Address += 8;  //skip first 64 bits for page status bytes

  /* Get the valid Page end Address */
  PageEndAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from beginning */
  while (Address < PageEndAddress)
  {
    /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
    if (FMC_Read(Address) == 0xFFFFFFFF)
    {
			wdata = VirtAddress;
			wdata<<=16;
			wdata|=Data;
			
      /* Set variable data */
			FMC_Write(Address, wdata);
			
      /* Return program operation status */
      return 0;
    }
    else
    {
      /* Next address location */
      Address = Address + 4;
    }
  }

  /* Return PAGE_FULL in case the valid page is full */
  return PAGE_FULL;
}

/*******************************************************************************
* Function Name  : EE_ReadVariable
* Description    : Returns the last stored variable data, if found, which
*                  correspond to the passed virtual address
* Input          : - VirtAddress: Variable virtual address
*                  - Read_data: Global variable contains the read variable value
* Output         : None
* Return         : - Success or error status:
*                      - 0: if variable was found
*                      - 1: if the variable was not found
*                      - NO_VALID_PAGE: if no valid page was found.
*******************************************************************************/
uint8_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* ReadData)
{
  uint32_t ValidPage;
  uint32_t AddressAndValue; 
  uint32_t Address, PageStartAddress;
	uint8_t ReadStatus = 1;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE)
  {
    return  NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  Address = (uint32_t)((EEPROM_START_ADDRESS - 4) + (uint32_t)((1 + ValidPage) * PAGE_SIZE)); 
		
  /* Check each active page address starting from end */
  while (Address > (PageStartAddress+4))
  {
    /* Get the current location content to be compared with virtual address */
    AddressAndValue =  FMC_Read(Address);

    /* Compare the read address with the virtual address */
    if (((AddressAndValue>>16) & 0xffff) == VirtAddress)
    {
      /* Get content of Address-2 which is variable value */
      *ReadData = AddressAndValue & 0xffff;
      /* In case variable value is read, reset ReadStatus flag */
      ReadStatus = 0;
      break;
    }
    else
    {
      /* Next address location */
      Address = Address - 4;
    }
  }

  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return ReadStatus;
}

/*******************************************************************************
* Function Name  : EE_PageTransfer
* Description    : Transfers last updated variable data from the full Page to
*                  an empty one.
* Input          : - VirtAddress: 16 bit virtual address of the variable
*                  - Data: 16 bit data to be written as variable value
* Output         : None
* Return         : - Success or error status:
*                      - FLASH_COMPLETE: on success,
*                      - PAGE_FULL: if valid page is full
*                      - NO_VALID_PAGE: if no valid page was found
*                      - Flash error code: on write Flash error
*******************************************************************************/
uint8_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
  uint32_t NewPageAddress, OldPageAddress;
  uint16_t VarIdx = 0;
	uint32_t ValidPage = PAGE0;
  uint8_t EepromStatus = 0, ReadStatus = 0;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  if (ValidPage == PAGE1)        /* Page1 valid */
  {
    /* New page address where variable will be moved to */
    NewPageAddress = PAGE0_BASE_ADDRESS;
    /* Old page address where variable will be taken from */
    OldPageAddress = PAGE1_BASE_ADDRESS;

  }
  else if (ValidPage == PAGE0)   /* Page0 valid */
  {
    /* New page address where variable will be moved to */
    NewPageAddress = PAGE1_BASE_ADDRESS;
    /* Old page address where variable will be taken from */
    OldPageAddress = PAGE0_BASE_ADDRESS;
  }
  else
  {
    return NO_VALID_PAGE;       /* No valid Page */
  }

  /* Set the new Page status to RECEIVE_DATA status */
  FMC_Write(NewPageAddress, VALID_PAGE);
	
  /* Write the variable passed as parameter in the new active page */
  EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if (EepromStatus != 0)
  {
    return EepromStatus;
  }

  /* Transfer process: transfer variables from old to the new active page */
  for (VarIdx = 0; VarIdx < NumbOfVar; VarIdx++)
  {
    if (VirtAddVarTab[VarIdx] != VirtAddress)  /* Check each variable except the one passed as parameter */
    {
      /* Read the other last variable updates */
      ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
      /* In case variable corresponding to the virtual address was found */
      if (ReadStatus != 0x01)
      {
        /* Transfer the variable to the new active page */
        EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
        /* If program operation was failed, a Flash error code is returned */
        if (EepromStatus != 0)
        {
          return EepromStatus;
        }
      }
    }
  }

  /* Erase the old Page: Set old Page status to ERASED status */
	EE_Erase(OldPageAddress);

  /* Set new Page status to VALID_PAGE status */
  FMC_Write(NewPageAddress+4, VALID_PAGE);
	
  
  /* Return last operation flash status */
  return 0;
}

/*******************************************************************************
* Function Name  : EE_WriteVariable
* Description    : Writes/updates variable data in EEPROM.
* Input          : - VirtAddress: Variable virtual address
*                  - Data: 16 bit data to be written
* Output         : None
* Return         : - Success or error status:
*                      - FLASH_COMPLETE: on success,
*                      - PAGE_FULL: if valid page is full
*                      - NO_VALID_PAGE: if no valid page was found
*                      - Flash error code: on write Flash error
*******************************************************************************/
uint8_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  uint8_t Status = WRITE_ERR;
	uint16_t ReadData;

	/* Check the variable has same value */
	if (EE_ReadVariable(VirtAddress,&ReadData)==0){
		if (ReadData==Data) {
			return 0;
		}
	}
	
	/* Write the variable virtual address and value in the EEPROM */
	Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
	/* In case the EEPROM active page is full */
	if (Status == PAGE_FULL)
	{
		/* Perform Page transfer */
		Status = EE_PageTransfer(VirtAddress, Data);
	}

  /* Return last operation status */
  return Status;
}


uint8_t EE_ReadVariable32(uint16_t VirtAddressStart, uint32_t* ReadData)
{
	uint8_t Status;
	uint16_t Data;
	
	Status = EE_ReadVariable(VirtAddressStart,&Data);
	if (Status!=0) return Status;
	
	*ReadData = Data;
	*ReadData<<=16;
	
	Status = EE_ReadVariable(VirtAddressStart+1,&Data);
	if (Status!=0) return Status;
	
	*ReadData |= Data;

  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return Status;
}

uint8_t EE_WriteVariable32(uint16_t VirtAddressStart, uint32_t Data)
{
	uint8_t Status = 0;
	
	Status = EE_WriteVariable(VirtAddressStart,Data>>16);
	Status = EE_WriteVariable(VirtAddressStart+1,Data);
	
	/* Return last operation status */
	return Status;
}




