#include "bsp_hal.h"

extern uint32_t pinmap_function(PinName pin, const PinMap *map);
extern uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map);
extern uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map);

static UartDataReceivedIRQhandler uartDataReceivedIRQhandler = 0;

static const uint32_t MODES[] = {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
};

static const uint32_t PULLS[] = {GPIO_PUSEL_DISABLE, GPIO_PUSEL_PULL_UP, GPIO_PUSEL_PULL_DOWN};

const uint32_t DATABITS[] = {UART_WORD_LEN_5, UART_WORD_LEN_6, UART_WORD_LEN_7, UART_WORD_LEN_8, UART_WORD_LEN_8};

const uint32_t STOPBITS[] = {UART_STOP_BIT_1, UART_STOP_BIT_1, UART_STOP_BIT_2, UART_STOP_BIT_1_5};

const uint32_t PARITY[] = {UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD};

/* UART PinMap */
const PinMap PinMap_UART[] = {

    {.pin = PB0, .peripheral = 2, .function = SYS_GPB_MFPL_PB0MFP_UART2_RXD},
    {.pin = PB1, .peripheral = 2, .function = SYS_GPB_MFPL_PB1MFP_UART2_TXD},

    {.pin = PB2, .peripheral = 1, .function = SYS_GPB_MFPL_PB2MFP_UART1_RXD},
    {.pin = PB3, .peripheral = 1, .function = SYS_GPB_MFPL_PB3MFP_UART1_TXD},

    {.pin = PB4, .peripheral = 5, .function = SYS_GPB_MFPL_PB4MFP_UART5_RXD},
    {.pin = PB5, .peripheral = 5, .function = SYS_GPB_MFPL_PB5MFP_UART5_TXD},

    {.pin = PB5, .peripheral = 2, .function = SYS_GPB_MFPL_PB5MFP_UART2_TXD},

    {.pin = PB6, .peripheral = 1, .function = SYS_GPB_MFPL_PB6MFP_UART1_RXD},
    {.pin = PB7, .peripheral = 1, .function = SYS_GPB_MFPL_PB7MFP_UART1_TXD},

    {.pin = PB8, .peripheral = 0, .function = SYS_GPB_MFPH_PB8MFP_UART0_RXD},
    {.pin = PB9, .peripheral = 0, .function = SYS_GPB_MFPH_PB9MFP_UART0_TXD},

    {.pin = PB8, .peripheral = 7, .function = SYS_GPB_MFPH_PB8MFP_UART7_RXD},
    {.pin = PB9, .peripheral = 7, .function = SYS_GPB_MFPH_PB9MFP_UART7_TXD},

    {.pin = PB10, .peripheral = 4, .function = SYS_GPB_MFPH_PB10MFP_UART4_RXD},
    {.pin = PB11, .peripheral = 4, .function = SYS_GPB_MFPH_PB11MFP_UART4_TXD},

    {.pin = PB12, .peripheral = 0, .function = SYS_GPB_MFPH_PB12MFP_UART0_RXD},
    {.pin = PB13, .peripheral = 0, .function = SYS_GPB_MFPH_PB13MFP_UART0_TXD},

    {.pin = PB14, .peripheral = 3, .function = SYS_GPB_MFPH_PB14MFP_UART3_RXD},
    {.pin = PB15, .peripheral = 3, .function = SYS_GPB_MFPH_PB15MFP_UART3_TXD},

    {.pin = PA0, .peripheral = 0, .function = SYS_GPA_MFPL_PA0MFP_UART0_RXD},
    {.pin = PA1, .peripheral = 0, .function = SYS_GPA_MFPL_PA1MFP_UART0_TXD},

    {.pin = PA2, .peripheral = 4, .function = SYS_GPA_MFPL_PA2MFP_UART4_RXD},
    {.pin = PA3, .peripheral = 4, .function = SYS_GPA_MFPL_PA3MFP_UART4_TXD},

    {.pin = PA4, .peripheral = 0, .function = SYS_GPA_MFPL_PA4MFP_UART0_RXD},
    {.pin = PA5, .peripheral = 0, .function = SYS_GPA_MFPL_PA5MFP_UART0_TXD},

    {.pin = PA4, .peripheral = 5, .function = SYS_GPA_MFPL_PA4MFP_UART5_RXD},
    {.pin = PA5, .peripheral = 5, .function = SYS_GPA_MFPL_PA5MFP_UART5_TXD},

    {.pin = PA6, .peripheral = 0, .function = SYS_GPA_MFPL_PA6MFP_UART0_RXD},
    {.pin = PA7, .peripheral = 0, .function = SYS_GPA_MFPL_PA7MFP_UART0_TXD},

    {.pin = PA8, .peripheral = 7, .function = SYS_GPA_MFPH_PA8MFP_UART7_RXD},
    {.pin = PA9, .peripheral = 7, .function = SYS_GPA_MFPH_PA9MFP_UART7_TXD},

    {.pin = PA9, .peripheral = 1, .function = SYS_GPA_MFPH_PA9MFP_UART1_TXD},

    {.pin = PA10, .peripheral = 6, .function = SYS_GPA_MFPH_PA10MFP_UART6_RXD},
    {.pin = PA11, .peripheral = 6, .function = SYS_GPA_MFPH_PA11MFP_UART6_TXD},

    {.pin = PA12, .peripheral = 4, .function = SYS_GPA_MFPH_PA12MFP_UART4_TXD},
    {.pin = PA13, .peripheral = 4, .function = SYS_GPA_MFPH_PA13MFP_UART4_RXD},

    {.pin = PA14, .peripheral = 0, .function = SYS_GPA_MFPH_PA14MFP_UART0_TXD},
    {.pin = PA15, .peripheral = 0, .function = SYS_GPA_MFPH_PA15MFP_UART0_RXD},

    {.pin = PF0, .peripheral = 0, .function = SYS_GPF_MFPL_PF0MFP_UART0_TXD},
    {.pin = PF1, .peripheral = 0, .function = SYS_GPF_MFPL_PF1MFP_UART0_RXD},

    {.pin = PF0, .peripheral = 1, .function = SYS_GPF_MFPL_PF0MFP_UART1_TXD},
    {.pin = PF1, .peripheral = 1, .function = SYS_GPF_MFPL_PF1MFP_UART1_RXD},

    {.pin = PF2, .peripheral = 0, .function = SYS_GPF_MFPL_PF2MFP_UART0_RXD},
    {.pin = PF3, .peripheral = 0, .function = SYS_GPF_MFPL_PF3MFP_UART0_TXD},

    {.pin = PF4, .peripheral = 2, .function = SYS_GPF_MFPL_PF4MFP_UART2_TXD},
    {.pin = PF5, .peripheral = 2, .function = SYS_GPF_MFPL_PF5MFP_UART2_RXD},

    {.pin = PF6, .peripheral = 4, .function = SYS_GPF_MFPL_PF6MFP_UART4_RXD},

    {.pin = PC0, .peripheral = 2, .function = SYS_GPC_MFPL_PC0MFP_UART2_RXD},
    {.pin = PC1, .peripheral = 2, .function = SYS_GPC_MFPL_PC1MFP_UART2_TXD},

    {.pin = PC2, .peripheral = 3, .function = SYS_GPC_MFPL_PC2MFP_UART3_RXD},
    {.pin = PC3, .peripheral = 3, .function = SYS_GPC_MFPL_PC3MFP_UART3_TXD},

    {.pin = PC4, .peripheral = 4, .function = SYS_GPC_MFPL_PC4MFP_UART4_RXD},
    {.pin = PC5, .peripheral = 4, .function = SYS_GPC_MFPL_PC5MFP_UART4_TXD},

    {.pin = PC5, .peripheral = 2, .function = SYS_GPC_MFPL_PC5MFP_UART2_TXD},

    {.pin = PC6, .peripheral = 4, .function = SYS_GPC_MFPL_PC6MFP_UART4_RXD},
    {.pin = PC7, .peripheral = 4, .function = SYS_GPC_MFPL_PC7MFP_UART4_TXD},

    {.pin = PC7, .peripheral = 6, .function = SYS_GPC_MFPL_PC7MFP_UART6_TXD},

    {.pin = PD0, .peripheral = 3, .function = SYS_GPD_MFPL_PD0MFP_UART3_RXD},
    {.pin = PD1, .peripheral = 3, .function = SYS_GPD_MFPL_PD1MFP_UART3_TXD},

    {.pin = PD2, .peripheral = 0, .function = SYS_GPD_MFPL_PD2MFP_UART0_RXD},
    {.pin = PD3, .peripheral = 0, .function = SYS_GPD_MFPL_PD3MFP_UART0_TXD},

    {NC}};

uint32_t uart_irq_number_get(const uint8_t uart_index) {
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
#if defined(UART4)
  case UART_IND4:
    irq_number = UART4_IRQn;
    break;
#endif
#if defined(UART5)
  case UART_IND5:
    irq_number = UART5_IRQn;
    break;
#endif
#if defined(UART6)
  case UART_IND6:
    irq_number = UART6_IRQn;
    break;
#endif
#if defined(UART7)
  case UART_IND7:
    irq_number = UART7_IRQn;
    break;
#endif
  default:
    break;
  }
  return irq_number;
}

static uint32_t gpio_periph_clock_enable(const PinName pin) {
  uint8_t port = PORT_GET(pin);
  switch (port) {
#if defined(_PA0)
  case 0:
    return GPIOA_BASE;
#endif
#if defined(_PB0)
  case 1:
    return GPIOB_BASE;
#endif
#if defined(_PC0)
  case 2:
    return GPIOC_BASE;
#endif
#if defined(_PD0)
  case 3:
    return GPIOD_BASE;
#endif
#if defined(_PE0)
  case 4:
    return GPIOE_BASE;
#endif
#if defined(_PF0)
  case 5:
    return GPIOF_BASE;
#endif
#if defined(_PG0)
  case 6:
    return GPIOG_BASE;
#endif
#if defined(_PH0)
  case 7:
    return GPIOH_BASE;
#endif
  }
  return 0;
}

static uint32_t uart_periph_clock_enable(const uint8_t index) {
  uint32_t addr;
  uint32_t uart_rst;

  /* Unlock protected registers */
  SYS_UnlockReg();

  switch (index) {
#if defined(UART0)
  case 0:
    addr     = UART0_BASE;
    uart_rst = UART0_RST;
    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    break;
#endif
#if defined(UART1)
  case 1:
    addr     = UART1_BASE;
    uart_rst = UART1_RST;
    /* Switch UART1 clock source to HIRC */
    // CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    break;
#endif
#if defined(UART2)
  case 2:
    addr     = UART2_BASE;
    uart_rst = UART2_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);
    break;
#endif
#if defined(UART3)
  case 3:
    addr     = UART3_BASE;
    uart_rst = UART3_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART3_MODULE);
    break;
#endif
#if defined(UART4)
  case 4:
    addr     = UART4_BASE;
    uart_rst = UART4_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART4_MODULE);
    break;
#endif
#if defined(UART5)
  case 5:
    addr     = UART5_BASE;
    uart_rst = UART5_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART5_MODULE);
    break;
#endif
#if defined(UART6)
  case 6:
    addr     = UART6_BASE;
    uart_rst = UART6_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART6_MODULE);
    break;
#endif
#if defined(UART7)
  case 7:
    addr     = UART7_BASE;
    uart_rst = UART7_RST;
    /* Switch UART2 clock source to HIRC */
    // CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART7_MODULE);
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

void bsp_hal_gpio_init(gpio_t *const gpio, const uint8_t initial_pin_state) {
  /* GET PERIPHERAL ADDRES & ENABLE RCU_CLOCK */
  gpio->addr = gpio_periph_clock_enable(gpio->pin);

  /* GET PIN MASK */
  gpio->mask = 1 << PIN_GET(gpio->pin);

  /* SET INITIAL PIN STATE */
  if (initial_pin_state != 2)
    bsp_hal_gpio_write(gpio, initial_pin_state);

  /* SET MODE */
  if (gpio->outtype == HAL_GPIO_OTYPE_OD)
    GPIO_SetMode((GPIO_T *)gpio->addr, gpio->mask, GPIO_MODE_OPEN_DRAIN);
  else if (gpio->outtype == HAL_GPIO_OTYPE_QUASI)
    GPIO_SetMode((GPIO_T *)gpio->addr, gpio->mask, GPIO_MODE_QUASI);
  else
    GPIO_SetMode((GPIO_T *)gpio->addr, gpio->mask, MODES[gpio->mode]);

  /* SET PULL MODE */
  GPIO_SetPullCtl((GPIO_T *)gpio->addr, gpio->mask, PULLS[gpio->pull]);
}

void bsp_hal_uart_init(uart_t *const uart) {
  uart->addr = uart_periph_clock_enable(uart->index);

  /* Unlock protected registers */
  SYS_UnlockReg();

  /* Set GPx multi-function pins for UART RXD */
  bsp_gpio_set_multi_function(uart->rxpin, pinmap_function_ex(uart->rxpin, uart->index, (PinMap *)&PinMap_UART));

  /* Set GPx multi-function pins for UART TXD */
  bsp_gpio_set_multi_function(uart->txpin, pinmap_function_ex(uart->txpin, uart->index, (PinMap *)&PinMap_UART));

#if 0
  switch (uart->depin) {
  case PD8:
    SYS->GPD_MFPH = (SYS->GPD_MFPH & ~(SYS_GPD_MFPH_PD8MFP_Msk)) | (SYS_GPD_MFPH_PD8MFP_UART2_nRTS);
    break;
  default:
    break;
  }
#endif

  /* Lock protected registers */
  SYS_LockReg();

  /* Configure UART */
  UART_OpenEx(
      (UART_T *)uart->addr, uart->baudrate, DATABITS[uart->databits], PARITY[uart->parity], STOPBITS[uart->stopbits]);
}

void bsp_hal_uart_enable_rx_int(uart_t *const uart, const uint8_t priority) {
  IRQn_Type nvic_irq = (IRQn_Type)uart_irq_number_get(uart->index);

  /* set the priority and enable the selected IRQ */
  NVIC_SetPriority(nvic_irq, priority);
  NVIC_EnableIRQ(nvic_irq);

  UART_EnableInt((UART_T *)uart->addr, UART_INTEN_RDAIEN_Msk);
}

void bsp_hal_uart_set_rx_irqhandler(UartDataReceivedIRQhandler irqhandler) {
  if (irqhandler != 0)
    uartDataReceivedIRQhandler = irqhandler;
}

void bsp_hal_uart_transmit(uart_t *const uart, uint8_t *const data, const uint8_t length) {
  for (int i = 0; i < length; i++) {
    UART_WRITE((UART_T *)uart->addr, data[i]);
    while (UART_IS_TX_FULL((UART_T *)uart->addr))
      ;
  }
  UART_WAIT_TX_EMPTY((UART_T *)uart->addr);
}



void UART0_IRQHandler(void) {
#if defined(UART0)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART0);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND0, data);
  }
#endif
}

void UART1_IRQHandler(void) {
#if defined(UART1)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART1);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND1, data);
  }
#endif
}

void UART2_IRQHandler(void) {
#if defined(UART2)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART2);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND2, data);
  }
#endif
}

void UART3_IRQHandler(void) {
#if defined(UART3)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART3, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART3);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND3, data);
  }
#endif
}

void UART4_IRQHandler(void) {
#if defined(UART4)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART4, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART4);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND4, data);
  }
#endif
}

void UART5_IRQHandler(void) {
#if defined(UART5)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART5, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART5);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND5, data);
  }
#endif
}

void UART6_IRQHandler(void) {
#if defined(UART6)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART6, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART6);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND6, data);
  }
#endif
}

void UART7_IRQHandler(void) {
#if defined(UART7)
  /* Rx Ready INT */
  if (UART_GET_INT_FLAG(UART7, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)) {
    uint8_t data = UART_READ(UART7);
    /* call uart handler */
    if (uartDataReceivedIRQhandler != 0)
      uartDataReceivedIRQhandler(UART_IND7, data);
  }
#endif
}
