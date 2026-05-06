#include "bsp_hal.h"

extern uint32_t pinmap_merge(uint32_t a, uint32_t b);
extern uint32_t pinmap_peripheral(PinName pin, const PinMap *map);
extern uint32_t pinmap_function(PinName pin, const PinMap *map);
extern uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map);
extern uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map);

static UartDataReceivedIRQhandler uartDataReceivedIRQhandler = 0;
static TimerUpdateIRQHandler timerUpdateIRQHandler = 0;
static TimerChannelIRQHandler timerChannelIRQHandler = 0;

static const uint32_t MODES[] = {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_AF,
	GPIO_MODE_AF,
	GPIO_MODE_ANALOG
};

static const uint32_t PULLS[] = {
	GPIO_PUPD_NONE,
	GPIO_PUPD_PULLUP,
	GPIO_PUPD_PULLDOWN
};

static const uint32_t OUTTYPES[] = {
	GPIO_OTYPE_PP,
	GPIO_OTYPE_OD
};

static const uint32_t AF_TYPES[] = {
	GPIO_AF_0,
	GPIO_AF_1,
	GPIO_AF_2,
	GPIO_AF_3,
	GPIO_AF_4,
	GPIO_AF_5,
	GPIO_AF_6,
	GPIO_AF_7
};

const uint32_t DATABITS[] = {
	USART_WL_8BIT,
	USART_WL_8BIT,
	USART_WL_8BIT,
	USART_WL_8BIT,
	USART_WL_9BIT
};

const uint32_t STOPBITS[] = {
	USART_STB_1BIT,
	USART_STB_0_5BIT,
	USART_STB_2BIT,
	USART_STB_1_5BIT
};

const uint32_t PARITY[] = {
	USART_PM_NONE,
	USART_PM_EVEN,
	USART_PM_ODD
};


const uint32_t TIMER_OC_MODES[] = {
	TIMER_OC_MODE_TIMING,
	TIMER_OC_MODE_ACTIVE,
	TIMER_OC_MODE_INACTIVE,
	TIMER_OC_MODE_TOGGLE,
	TIMER_OC_MODE_LOW,
	TIMER_OC_MODE_HIGH,
	TIMER_OC_MODE_PWM0,
	TIMER_OC_MODE_PWM1
};


const uint32_t TIMER_IC_POLARITIES[] = {
	TIMER_IC_POLARITY_RISING,
	TIMER_IC_POLARITY_FALLING,
	TIMER_IC_POLARITY_BOTH_EDGE
};

const uint32_t TIMER_IC_SELECTION[] = {
	TIMER_IC_SELECTION_DIRECTTI,
	TIMER_IC_SELECTION_INDIRECTTI,
	TIMER_IC_SELECTION_ITS
};

const uint32_t TIMER_IC_TRIGGERS[] = {
	TIMER_SMCFG_TRGSEL_ITI0,
	TIMER_SMCFG_TRGSEL_ITI1,
	TIMER_SMCFG_TRGSEL_ITI2,
	TIMER_SMCFG_TRGSEL_ITI3,
	TIMER_SMCFG_TRGSEL_CI0F_ED,
	TIMER_SMCFG_TRGSEL_CI0FE0,
	TIMER_SMCFG_TRGSEL_CI1FE1,
	TIMER_SMCFG_TRGSEL_ETIFP
};

const uint32_t TIMER_IC_SLAVE_MODES[] = {
	TIMER_SLAVE_MODE_DISABLE,
	TIMER_ENCODER_MODE0,
	TIMER_ENCODER_MODE1,
	TIMER_ENCODER_MODE2,
	TIMER_SLAVE_MODE_RESTART,
	TIMER_SLAVE_MODE_PAUSE,
	TIMER_SLAVE_MODE_EVENT,
	TIMER_SLAVE_MODE_EXTERNAL0
};

const uint32_t INTERRUPTS[] = {
	TIMER_INT_UP,
	TIMER_INT_CH0,
	TIMER_INT_CH1,
	TIMER_INT_CH2,
	TIMER_INT_CH3,
};

const uint32_t INTERRUPT_FLAGS[] = {
	TIMER_INT_FLAG_UP,
	TIMER_INT_FLAG_CH0,
	TIMER_INT_FLAG_CH1,
	TIMER_INT_FLAG_CH2,
	TIMER_INT_FLAG_CH3,
};

static uint32_t gpio_periph_clock_enable(const PinName pin){
	uint8_t port_index = PORT_GET(pin);
	switch(port_index) {
	#if defined(GPIOA)
		case 0:
			rcu_periph_clock_enable(RCU_GPIOA);
			return GPIOA;
	#endif
	#if defined(GPIOB)
		case 1:
			rcu_periph_clock_enable(RCU_GPIOB);
			return GPIOB;
	#endif
	#if defined(GPIOC)
		case 2:
			rcu_periph_clock_enable(RCU_GPIOC);
			return GPIOC;
	#endif
	#if defined(GPIOD)
		case 3:
			rcu_periph_clock_enable(RCU_GPIOD);
			return GPIOD;
	#endif
	#if defined(GPIOE)
		case 4:
			rcu_periph_clock_enable(RCU_GPIOE);
			return GPIOE;
	#endif
	#if defined(GPIOF)
		case 5:
			rcu_periph_clock_enable(RCU_GPIOF);
			return GPIOF;
	#endif
	}
	return 0;
}

static uint32_t uart_periph_clock_enable(const uint8_t index){
	switch(index) {
	#if defined(USART0)
		case 0:
			rcu_periph_clock_enable(RCU_USART0);
			return USART0;
	#endif
	#if defined(USART1)
		case 1:
			rcu_periph_clock_enable(RCU_USART1);
			return USART1;
	#endif
	}
	return 0;
}

static uint32_t timer_periph_clock_enable(const uint8_t index){
	switch(index) {
	#if defined(TIMER0)
		case 0:
			rcu_periph_clock_enable(RCU_TIMER0);
			return TIMER0;
	#endif
	#if defined(TIMER1)
		case 1:
			rcu_periph_clock_enable(RCU_TIMER1);
			return TIMER1;
	#endif
	#if defined(TIMER2)
		case 2:
			rcu_periph_clock_enable(RCU_TIMER2);
			return TIMER2;
	#endif
	#if defined(TIMER5)
		case 5:
			rcu_periph_clock_enable(RCU_TIMER5);
			return TIMER5;
	#endif
	#if defined(TIMER13)
		case 13:
			rcu_periph_clock_enable(RCU_TIMER13);
			return TIMER13;
	#endif
	#if defined(TIMER14)
		case 14:
			rcu_periph_clock_enable(RCU_TIMER14);
			return TIMER14;
	#endif
	#if defined(TIMER15)
		case 15:
			rcu_periph_clock_enable(RCU_TIMER15);
			return TIMER15;
	#endif
	#if defined(TIMER16)
		case 16:
			rcu_periph_clock_enable(RCU_TIMER16);
			return TIMER16;
	#endif
	
	}
	return 0;
}

uint32_t uart_irq_number_get(const uint8_t uart_index) 
{
  uint32_t irq_number = 0;
	
	switch (uart_index) {
			#if defined(USART0)			
			case UART_IND0:
				irq_number = USART0_IRQn;
				break;
			#endif
			#if defined(USART1)			
			case UART_IND1:
				irq_number = USART1_IRQn;
				break;
			#endif
			default:
				break;
	}
	return irq_number;
}

uint32_t timer_irq_number_get(const uint8_t timer_index) 
{
	switch (timer_index) {
	#if defined(TIMER0)
		case 0:
			return TIMER0_BRK_UP_TRG_COM_IRQn;
	#endif
	#if defined(TIMER1)
		case 1:
			return TIMER1_IRQn;
	#endif
	#if defined(TIMER2)
		case 2:
			return TIMER2_IRQn;
	#endif
	#if defined(TIMER5)
		case 5:
			#if defined(GD32F310)
			return TIMER5_DAC_IRQn;
			#elif defined(GD32E23x)
			return TIMER5_IRQn;
			#endif
	#endif
	#if defined(TIMER13)
		case 13:
			return TIMER13_IRQn;
	#endif
	#if defined(TIMER14)
		case 14:
			return TIMER14_IRQn;
	#endif
	#if defined(TIMER15)
		case 15:
			return TIMER15_IRQn;
	#endif
	#if defined(TIMER16)
		case 16:
			return TIMER16_IRQn;
	#endif
	}
	return 0;

}

void bsp_hal_gpio_init(gpio_t * const gpio,const uint8_t initial_pin_state) {
	
		/* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK */
		gpio->addr = gpio_periph_clock_enable(gpio->pin);
		
		/* GET PIN MASK */
		gpio->mask = 1 << PIN_GET(gpio->pin);
	
		/* SET INITIAL PIN STATE */
		if (initial_pin_state!=2) gpio_bit_write(gpio->addr, gpio->mask, initial_pin_state);
	
		/* SET AF MODE*/
		if ((gpio->mode == HAL_GPIO_MODE_AF_IN) || (gpio->mode == HAL_GPIO_MODE_AF_OUT)) {
			gpio_af_set(gpio->addr, AF_TYPES[gpio->aftype], gpio->mask);
		}
		
		/* SET MODE */
		gpio_mode_set(gpio->addr, MODES[gpio->mode], PULLS[gpio->pull], gpio->mask);
		
		/* SET OUTPUT OPTIONS */
		if (gpio->mode == HAL_GPIO_MODE_AF_OUT) {
			gpio_output_options_set(gpio->addr, OUTTYPES[gpio->outtype],GPIO_OSPEED_50MHZ, gpio->mask);
		}	
}


/* USART PinMap */
const PinMap PinMap_UART_TX[] = {
    {.pin =  PA9, .peripheral = UART_IND0, .function = HAL_GPIO_AF1},
		{.pin =  PA2, .peripheral = UART_IND1, .function = HAL_GPIO_AF1},
    {NC}
};

const PinMap PinMap_UART_RX[] = {
    {.pin = PA10, .peripheral = UART_IND0, .function = HAL_GPIO_AF1},
		{.pin = PA3,  .peripheral = UART_IND1, .function = HAL_GPIO_AF1},
    {NC}
};

void bsp_hal_uart_init(uart_t * const uart) {
	gpio_t rxgpio = {uart->rxpin, HAL_GPIO_MODE_AF_IN, 	HAL_GPIO_PULL_UP, 	HAL_GPIO_OTYPE_NONE};
	gpio_t txgpio = {uart->txpin, HAL_GPIO_MODE_AF_OUT, 	HAL_GPIO_PULL_NONE, HAL_GPIO_OTYPE_PP};
	
	/* GET UART INDEX */
	uart->index = pinmap_merge(pinmap_peripheral(rxgpio.pin, PinMap_UART_RX), pinmap_peripheral(txgpio.pin, PinMap_UART_TX));
	
	/* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK FOR UART */
	uart->addr = uart_periph_clock_enable(uart->index);
	
	/* GET AF FUNCTIONS */
	rxgpio.aftype = pinmap_function(rxgpio.pin,PinMap_UART_RX);
	txgpio.aftype = pinmap_function(txgpio.pin,PinMap_UART_TX);
	
	/* CONFIGURE RX TX PINS */
	bsp_hal_gpio_init(&rxgpio,2);
	bsp_hal_gpio_init(&txgpio,2);
	
	/* CONFIGURE USART  */
	usart_deinit(uart->addr);
	usart_word_length_set(uart->addr, DATABITS[uart->databits]);
	usart_stop_bit_set(uart->addr, STOPBITS[uart->stopbits]);
	usart_parity_config(uart->addr, PARITY[uart->parity]);
	usart_baudrate_set(uart->addr, uart->baudrate);
	usart_oversample_config(uart->addr,USART_OVSMOD_16);
	usart_overrun_disable(uart->addr);
	usart_receive_config(uart->addr, USART_RECEIVE_ENABLE);
	usart_transmit_config(uart->addr, USART_TRANSMIT_ENABLE);	
}


void bsp_hal_uart_enable_rx_int(uart_t * const uart, const uint8_t priority) {
	bsp_hal_nvic_irq_enable(uart_irq_number_get(uart->index), priority, 0);
	usart_interrupt_enable(uart->addr, USART_INT_RBNE);
}

void bsp_hal_uart_set_rx_irqhandler(UartDataReceivedIRQhandler irqhandler) {
	if (irqhandler != 0) uartDataReceivedIRQhandler = irqhandler;
}

void bsp_hal_uart_transmit(uart_t * const uart, uint8_t * const data, const uint8_t length) {
	for (int i=0; i<length; i++) {
		usart_data_transmit(uart->addr,data[i]);
		while(RESET == usart_flag_get(uart->addr, USART_FLAG_TBE));	
	}
}

void bsp_hal_timer_init(timer_t * const timer) {
	timer_parameter_struct timer_initpara;

	timer->addr = timer_periph_clock_enable(timer->index);
	
	/* deinit a TIMER */
	timer_deinit(timer->addr);
	/* initialize TIMER init parameter struct */
	timer_struct_para_init(&timer_initpara);
	/* TIMER configuration */
	timer_initpara.prescaler         = timer->prescaler;
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = timer->period;
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_init(timer->addr, &timer_initpara);
}


void bsp_hal_timer_enable_int(timer_t * const timer, uint8_t priority) {
	/* clear interrupt bit */
	timer_interrupt_flag_clear(timer->addr, INTERRUPT_FLAGS[timer->interrupt]);
	
	/* enable the TIMER interrupt */
	timer_interrupt_enable(timer->addr, INTERRUPTS[timer->interrupt]);
	
	/* configure the TIMER interrupt */
	bsp_hal_nvic_irq_enable(timer_irq_number_get(timer->index), priority, 0);
}

void bsp_hal_pwm_in_start(pwm_in_t * const pwm_in) {
	timer_t *timer = pwm_in->timer;
	
	/* clear interrupt bit */
	timer_interrupt_flag_clear(timer->addr, INTERRUPT_FLAGS[timer->interrupt]);
	
	/* enable the TIMER interrupt */
	timer_interrupt_enable(timer->addr, INTERRUPTS[timer->interrupt]);
	
	/* start TIMER */
	timer_enable(timer->addr);
}

void bsp_hal_pwm_in_stop(pwm_in_t * const pwm_in) {
	timer_t *timer = pwm_in->timer;
	
	/* disable the TIMER interrupt */
	timer_interrupt_disable(timer->addr, INTERRUPTS[timer->interrupt]);
	
	/* stop TIMER */
	timer_disable(timer->addr);
}


void bsp_hal_timer_set_update_irqhandler(TimerUpdateIRQHandler irqhandler) {
	if (irqhandler != 0) timerUpdateIRQHandler = irqhandler;
}

void bsp_hal_timer_set_channel_irqhandler(TimerChannelIRQHandler irqhandler) {
	if (irqhandler != 0) timerChannelIRQHandler = irqhandler;
}


/* PWM OUT PinMap */
const PinMap PinMap_PWM_OUT[] = {
    {.pin = PA2, 	.peripheral = TIMER_IND14, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF0},
    
		{.pin = PA3, 	.peripheral = TIMER_IND14, 	.channel = TIMER_CH_1, .function = HAL_GPIO_AF0},
    
		{.pin = PA4, 	.peripheral = TIMER_IND13, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF4},

    {.pin = PA6, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF1},
    {.pin = PA6, 	.peripheral = TIMER_IND15, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF5},

    {.pin = PA7, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_1, .function = HAL_GPIO_AF1},
    {.pin = PA7, 	.peripheral = TIMER_IND13, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF4},
    {.pin = PA7, 	.peripheral = TIMER_IND16, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF5},

    {.pin = PA8, 	.peripheral = TIMER_IND0, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF2},
    
		{.pin = PA9, 	.peripheral = TIMER_IND0, 	.channel = TIMER_CH_1, .function = HAL_GPIO_AF2},
    
		{.pin = PA10, .peripheral = TIMER_IND0, 	.channel = TIMER_CH_2, .function = HAL_GPIO_AF2},
    
		{.pin = PB0, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_2, .function = HAL_GPIO_AF1},
		
		{.pin = PB1, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_3, .function = HAL_GPIO_AF1},
    {.pin = PB1, 	.peripheral = TIMER_IND13, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF0},
		
		{.pin = PB4, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF1},
		
		{.pin = PB5, 	.peripheral = TIMER_IND2, 	.channel = TIMER_CH_1, .function = HAL_GPIO_AF1},
		
		{.pin = PB8, 	.peripheral = TIMER_IND15, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF2},
		
		{.pin = PB9, 	.peripheral = TIMER_IND16, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF2},
		
		{.pin = PB14, .peripheral = TIMER_IND14, 	.channel = TIMER_CH_0, .function = HAL_GPIO_AF1},
		
		{.pin = PB15, .peripheral = TIMER_IND14, 	.channel = TIMER_CH_1, .function = HAL_GPIO_AF1},
		
    {NC}
};


void bsp_hal_pwm_out_pin_init(pwm_out_t * const pwm_out) {
	timer_oc_parameter_struct timer_ocinitpara;
	timer_break_parameter_struct timer_breakpara;
	
	timer_t *timer = pwm_out->timer;
	gpio_t gpio = {pwm_out->pin, HAL_GPIO_MODE_AF_OUT, 	HAL_GPIO_PULL_NONE, 	HAL_GPIO_OTYPE_PP};

	/* GET AF FUNCTIONS */
	gpio.aftype = pinmap_function_ex(gpio.pin,timer->index, (PinMap*)&PinMap_PWM_OUT);
	
	/* CONFIGURE GPIO PIN */
	bsp_hal_gpio_init(&gpio,0);

	
	/* GET TIMER CHANNEL */
	pwm_out->channel = pinmap_channel(gpio.pin,timer->index,(PinMap*)&PinMap_PWM_OUT);
	
	/* pwm timer init if needed */
	if (timer->addr == 0) {
		if (timer->period == -1) timer->period = ((SystemCoreClock / (timer->prescaler+1))/timer->pwm_frequency);
		bsp_hal_timer_init(pwm_out->timer);
	}

	pwm_out->duty_mul = (pwm_out->timer->period+1)/100.0f;
	
	/* initialize TIMER channel output parameter struct */
	timer_channel_output_struct_para_init(&timer_ocinitpara);
	/* configure TIMER channel output function */
	timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
	timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
	timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
	timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;		
	
	timer_channel_output_config(timer->addr, pwm_out->channel, &timer_ocinitpara);
  timer_channel_output_pulse_value_config(timer->addr, pwm_out->channel, 0 * pwm_out->duty_mul);
  timer_channel_output_mode_config(timer->addr, pwm_out->channel, TIMER_OC_MODES[pwm_out->mode]);
  timer_channel_output_shadow_config(timer->addr, pwm_out->channel, TIMER_OC_SHADOW_DISABLE);
	
	
	if ((timer->index == 0) || (timer->index == 14) || (timer->index == 15)) {
		timer_break_struct_para_init(&timer_breakpara);
		 /* automatic output enable, break, dead time and lock configuration*/
		timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
		timer_break_config(timer->addr, &timer_breakpara);
			
		/* primary output function enable */
		timer_primary_output_config(timer->addr, ENABLE);
	}
		
	/* auto-reload preload enable */
  timer_auto_reload_shadow_enable(pwm_out->timer->addr);
		
  /* auto-reload preload enable */
  timer_enable(pwm_out->timer->addr);	
}

void bsp_hal_pwm_start(pwm_out_t * const pwm_out) {
	if (pwm_out->timer->addr == 0) return;
	
	/* auto-reload preload enable */
  timer_auto_reload_shadow_enable(pwm_out->timer->addr);
		
  /* auto-reload preload enable */
  timer_enable(pwm_out->timer->addr);
}

void bsp_hal_pwm_out_pin_set_val(const pwm_out_t* const pwm_out, uint32_t val) {
	if (pwm_out->timer->addr == 0) return;
	
	timer_channel_output_pulse_value_config(pwm_out->timer->addr,  pwm_out->channel,val);
}

void bsp_hal_pwm_out_pin_set_duty(const pwm_out_t* const pwm_out, uint16_t duty) {
	if (pwm_out->timer->addr == 0) return;
	
	timer_channel_output_pulse_value_config(pwm_out->timer->addr,  pwm_out->channel,pwm_out->duty_mul  * duty);
}


void bsp_hal_pwm_in_pin_init(pwm_in_t * const pwm_in) {
	timer_ic_parameter_struct timer_icinitpara;
	
	timer_t *timer = pwm_in->timer;
	
	gpio_t *gpio = &pwm_in->gpio;
	
	gpio->mode = HAL_GPIO_MODE_AF_IN;
	gpio->pull = HAL_GPIO_PULL_NONE;
	
	/* GET AF FUNCTIONS */
	gpio->aftype = pinmap_function_ex(gpio->pin,timer->index, (PinMap*)&PinMap_PWM_OUT);
	
	/* CONFIGURE GPIO PIN */
	bsp_hal_gpio_init(gpio,2);
	
	/* pwm timer init if needed */
	if (timer->addr == 0) {
		if (timer->period == -1) timer->period = ((SystemCoreClock / timer->prescaler)/timer->pwm_frequency)-1;
		bsp_hal_timer_init(pwm_in->timer);
	}
	
	/* initialize TIMER channel input parameter struct */
	timer_channel_input_struct_para_init(&timer_icinitpara);
	/* TIMER CHANNEL PWM input capture configuration */
	timer_icinitpara.icpolarity  = TIMER_IC_POLARITIES[pwm_in->polarity];
	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
	timer_icinitpara.icfilter    = 4;

	timer_input_pwm_capture_config(timer->addr, TIMER_CH_0, &timer_icinitpara);
	
	/* channel 0 interrupt disable */
	timer_interrupt_disable(timer->addr, TIMER_INT_CH0);
	/* clear channel 0 interrupt bit */
	timer_interrupt_flag_clear(timer->addr, TIMER_INT_FLAG_CH0);

	/* TIMER counter disable */
	timer_disable(timer->addr);	
}

uint32_t bsp_hal_pwm_in_capture_value_read(pwm_in_t * const pwm_in) {
	return timer_channel_capture_value_register_read(pwm_in->timer->addr,TIMER_CH_0);
}

#if defined(TIMER13)
void TIMER13_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER13, TIMER_INT_FLAG_UP)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
				/* call timer handler */
				if (timerUpdateIRQHandler != 0) timerUpdateIRQHandler(13);
    }
}
#endif

#if defined(TIMER14)
void TIMER14_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER14, TIMER_INT_FLAG_UP)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER14, TIMER_INT_FLAG_UP);
				/* call timer handler */
				if (timerUpdateIRQHandler != 0) timerUpdateIRQHandler(14);
    }
}
#endif

#if defined(TIMER15)
void TIMER15_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER15, TIMER_INT_FLAG_UP)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
				/* call timer handler */
				if (timerUpdateIRQHandler != 0) timerUpdateIRQHandler(15);
    }
		
		if(SET == timer_interrupt_flag_get(TIMER15, TIMER_INT_FLAG_CH0)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_CH0);
				/* call timer handler */
				if (timerChannelIRQHandler != 0) timerChannelIRQHandler(15,0);
    }		
}
#endif

#if defined(TIMER16)
void TIMER16_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER16, TIMER_INT_FLAG_UP)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER16, TIMER_INT_FLAG_UP);
				/* call timer handler */
				if (timerUpdateIRQHandler != 0) timerUpdateIRQHandler(16);
    }
}
#endif

#if defined(USART0)
void USART0_IRQHandler(void) 
{		
	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
	{
		uint8_t data = usart_data_receive(USART0);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND0,data); 
	}
}
#endif

#if defined(USART1)
void USART1_IRQHandler(void) 
{		
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
	{
		uint8_t data = usart_data_receive(USART1);
		/* call uart handler */
		if (uartDataReceivedIRQhandler != 0) uartDataReceivedIRQhandler(UART_IND1,data); 
	}
}
#endif






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
	/* unlock the flash program/erase controller */
	fmc_unlock();
	
	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	/* erase page 0 */
	fmc_page_erase(PAGE0_BASE_ADDRESS);

	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	/* erase page 1 */
	fmc_page_erase(PAGE1_BASE_ADDRESS);
	
	/* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

	/* set page0 as valid page word1 */
	fmc_word_program(PAGE0_BASE_ADDRESS, VALID_PAGE);
	
	/* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	/* set page0 as valid page word2 */
	fmc_word_program(PAGE0_BASE_ADDRESS+4, VALID_PAGE);
	
	/* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	
  /* lock the main FMC after the operation */
  fmc_lock();
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
	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	/* erase target page */
	fmc_page_erase(PageBaseAddress);
	
	/* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
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
  PageStatus00 = REG32(PAGE0_BASE_ADDRESS);
  PageStatus01 = REG32(PAGE0_BASE_ADDRESS+4);
	if ( (PageStatus00 == VALID_PAGE) &&  (PageStatus01 == VALID_PAGE)) PageStatus0 = VALID_PAGE;
	else if ( (PageStatus00 == VALID_PAGE) &&  (PageStatus01 == EMPTY_DATA)) PageStatus0 = RECEIVE_DATA;
	
  /* Get Page1 actual status */
  PageStatus10 = REG32(PAGE1_BASE_ADDRESS);
  PageStatus11 = REG32(PAGE1_BASE_ADDRESS+4);
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
  fmc_state_enum FlashStatus = FMC_READY;
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
    if ( REG32(Address) == 0xFFFFFFFF)
    {
			wdata = VirtAddress;
			wdata<<=16;
			wdata|=Data;
			
      /* Set variable data */
			FlashStatus = fmc_word_program(Address, wdata);
			fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

      /* Return program operation status */
      return FlashStatus;
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
    AddressAndValue =  REG32(Address);

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
  fmc_state_enum FlashStatus = FMC_READY;
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
  FlashStatus = fmc_word_program(NewPageAddress, VALID_PAGE);
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != FMC_READY)
  {
    return FlashStatus;
  }

  /* Write the variable passed as parameter in the new active page */
  EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if (EepromStatus != FMC_READY)
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
        if (EepromStatus != FMC_READY)
        {
          return EepromStatus;
        }
      }
    }
  }

  /* Erase the old Page: Set old Page status to ERASED status */
	EE_Erase(OldPageAddress);

  /* Set new Page status to VALID_PAGE status */
  FlashStatus = fmc_word_program(NewPageAddress+4, VALID_PAGE);
	
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
  
  /* Return last operation flash status */
  return FlashStatus;
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
			return FMC_READY;
		}
	}
	
	/* unlock flash first */
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
	
	/* Write the variable virtual address and value in the EEPROM */
	Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
	/* In case the EEPROM active page is full */
	if (Status == PAGE_FULL)
	{
		/* Perform Page transfer */
		Status = EE_PageTransfer(VirtAddress, Data);
	}
	/* lock flash finally */
	fmc_lock();
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

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




