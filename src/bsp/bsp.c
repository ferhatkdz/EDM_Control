#include "bsp.h"
#include "cli.h"
#include "vcom_serial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define CRYSTAL_LESS				1
#define TRIM_INIT           (SYS_BASE+0x10C)
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */
#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */

STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

gpio_t board_pins[] = {
    /* INPUTS */
    GPIO_INPUT_PIN_PN(PA1),     // pinLIMIT1Z
    GPIO_INPUT_PIN_PN(PA2),     // pinLIMIT2Z
    GPIO_INPUT_PIN_PN(PA6),     // pinLIMIT1W
    GPIO_INPUT_PIN_PN(PF3),     // pinLIMIT2W

    /* OUTPUTS */
    GPIO_OUTPUT_PIN_PP(PB0),     // pinSSR1
    GPIO_OUTPUT_PIN_PP(PF6),     // pinSSR2
    GPIO_OUTPUT_PIN_PP(PF4),     // pinSSR3
    GPIO_OUTPUT_PIN_PP(PB1),     // pinSSR4
    GPIO_OUTPUT_PIN_PP(PA11),    // pinSSR5
    GPIO_OUTPUT_PIN_PP(PF5),     // pinSSR6
};


/* uart definition  */
uart_t uart_debug = {.rxpin    = PD2,
                     .txpin    = PD3,
                     .baudrate = 115200,
                     .databits = HAL_UART_DATA8,
                     .parity   = HAL_UART_PARITY_NONE,
                     .stopbits = HAL_UART_STOP_1,
                     .index    = UART_IND0};

/* ISRs used in this project ===============================================*/
volatile uint32_t tickSys = 0;
uint32_t u32TrimInit;

volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;
										 
uint8_t gRxBuf[64] = {0};
uint8_t *gpu8RxBuf = 0;
uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

uint16_t g_mosfet_status = 0;

										 
void SysTick_Handler(void) {
  QK_ISR_ENTRY(); /* inform QK about entering an ISR */

  tickSys++;

  QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); /* process time events for rate 0 */

  QK_ISR_EXIT(); /* inform QK about exiting an ISR */
}


void BSP_pin_set(const void* const me, uint8_t val) {
  const gpio_t* const gpio = (gpio_t* const)((out_t*)me)->out;

  bsp_hal_gpio_write(gpio, val);
}

void BSP_uart_puts(uart_t* uart, char* buf) {
  bsp_hal_uart_transmit(uart, (uint8_t*)buf, strlen(buf));
}


void debug_usb(char *format, ...) {
	char tmp_buf[128];
	int pos = 0;
	int size;
	va_list args;
	
	va_start(args, format);
	while (*format && pos < (sizeof(tmp_buf)-1)) {
			if (*format == '%') {
					format++;
					switch (*format) {
							case 'd': {
									int num = va_arg(args, int);
									pos += snprintf((char*)tmp_buf + pos, sizeof(tmp_buf) - pos, "%d", num);
									break;
							}
							case 'c': {
									char c = va_arg(args, int);
									pos += snprintf((char*)tmp_buf + pos, sizeof(tmp_buf) - pos, "%c", c);
									break;
							}
							case 's': {
									char *str = va_arg(args, char*);
									pos += snprintf((char*)tmp_buf + pos, sizeof(tmp_buf) - pos, "%s", str);
									break;
							}
					}
					format++;
			} else {
					tmp_buf[pos++] = *format++;
			}
	}
	va_end(args);

	tmp_buf[pos] = '\0';
	
	
	__set_PRIMASK(1);	
	for (int i=0; i<pos; i++) {
		comRbuf[comRtail++] = tmp_buf[i];	
		if (comRtail >= RXBUFSIZE) comRtail = 0;
    comRbytes++;		
	}
	__set_PRIMASK(0);
	
	//bsp_hal_uart_transmit(&uart_debug, (uint8_t*)tmp_buf, ++pos);
}


void VCOM_TransferData(void)
{
	int32_t i, i32Len;

	/* VBUS Bağlantı Kontrolü */
	/* Eğer kablo takılı değilse işlem yapma ve durumu sıfırla */
	if (!(USBD->VBUSDET & USBD_VBUSDET_VBUSDET_Msk)) {
			if (comRbytes > 0 || comTbytes > 0) VCOM_ResetState();
			return; 
	}
		
	if (((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1) && (CRYSTAL_LESS))
	{
			/* Start USB trim if it is not enabled. */
			if ((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
			{
					if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
					{
							/* Clear SOF */
							USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

							/* Re-enable crystal-less */
							SYS->HIRCTCTL = 0x1;
							SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk;
					}
			}

			/* Disable USB Trim when error */
			if (SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
			{
					/* Init TRIM */
					M32(TRIM_INIT) = u32TrimInit;

					/* Disable crystal-less */
					SYS->HIRCTCTL = 0;

					/* Clear error flags */
					SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

					/* Clear SOF */
					USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
			}
	}

	/* (tx to usb) Check whether USB is ready for next packet or not*/
	if(gu32TxSize == 0)
	{
			/* Check whether we have new COM Rx data to send to USB or not */
			if(comRbytes)
			{
					i32Len = comRbytes;
					if(i32Len > EP2_MAX_PKT_SIZE)
							i32Len = EP2_MAX_PKT_SIZE;

					for(i = 0; i < i32Len; i++)
					{
							gRxBuf[i] = comRbuf[comRhead++];
							if(comRhead >= RXBUFSIZE)
									comRhead = 0;
					}

					__set_PRIMASK(1);
					comRbytes -= i32Len;
					__set_PRIMASK(0);

					gu32TxSize = i32Len;
					USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
					USBD_SET_PAYLOAD_LEN(EP2, i32Len);
			}
			else
			{
					/* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
						 no more data to send at this moment to note Host the transfer has been done */
					i32Len = USBD_GET_PAYLOAD_LEN(EP2);
					if(i32Len == EP2_MAX_PKT_SIZE)
							USBD_SET_PAYLOAD_LEN(EP2, 0);
			}
	}

	/* (rx from usb) Process the Bulk out data when bulk out data is ready. */
	if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
	{
			for(i = 0; i < gu32RxSize; i++)
			{
					comTbuf[comTtail++] = gpu8RxBuf[i];
					if(comTtail >= TXBUFSIZE)
							comTtail = 0;
			}

			__set_PRIMASK(1);
			comTbytes += gu32RxSize;
			__set_PRIMASK(0);

			gu32RxSize = 0;
			gi8BulkOutReady = 0; /* Clear bulk out ready flag */

			/* Ready to get next BULK out */
			USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
	}

	/* Process the software Tx FIFO */
	if(comTbytes)
	{
			#if 0
			/* Check if Tx is working */
			if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
			{
					/* Send one bytes out */
					UART_WRITE(UART0, comTbuf[comThead++]);
					if(comThead >= TXBUFSIZE)
							comThead = 0;

					comTbytes--;

					/* Enable Tx Empty Interrupt. (Trigger first one) */
					UART0->INTEN |= UART_INTEN_THREIEN_Msk;
			}
			#else
			for (i = 0; i<comTbytes; i++) {
				debug_usb("%c", comTbuf[comThead++]);
				if(comThead >= TXBUFSIZE)
							comThead = 0;
				comTbytes--;
			}
			#endif
	}
}

static void BSP_usb_init(void) {
  /* Unlock protected registers */
  SYS_UnlockReg();
	
	/* M480LD support crystal-less */
	if (((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1) && (CRYSTAL_LESS))
	{
			CLK->PWRCTL |= CLK_PWRCTL_HIRC48MEN_Msk;
			/* HIRC48M stabil olana kadar bekle */
      while((CLK->STATUS & CLK_STATUS_HIRC48MSTB_Msk) == 0);		
			/* Select IP clock source */
			CLK->CLKSEL0 &= ~CLK_CLKSEL0_USBSEL_Msk;
	}
	else
	{
			/* Select IP clock source */
			CLK->CLKSEL0 |= CLK_CLKSEL0_USBSEL_Msk;
			CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(4);
	}
	
	/* Select USBD */
	SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

	/* Enable IP clock */
	CLK_EnableModuleClock(USBD_MODULE);

	/* Pin Fonksiyon Seçimi */
	SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk);
	SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P);	
  
  /* Windows'un cihazın çekildiğini ve tekrar takıldığını anlaması için D+ ve D- hatlarını lojik 0'a çekiyoruz */	
	USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
	USBD_SET_SE0();           // Hattı resetle
	CLK_SysTickDelay(100000); // 100ms bekle
	USBD_CLR_SE0();           // Hattı serbest bırak	

	/* Endpoint configuration */
	VCOM_Init();
	USBD_Start();
	
#if 0
	if (((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x1) && (CRYSTAL_LESS))
	{
			/* Start USB trim */
			USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
			while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);
			SYS->HIRCTCTL = 0x1;
			SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk;
			/* Backup default trim */
			u32TrimInit = M32(TRIM_INIT);
	}
#endif	
	
	/* Lock protected registers */
  SYS_LockReg();	
}

void BSP_SPARK_mosfet_set(uint8_t id, uint8_t state) {
    if (id < 1 || id > 10) return;

    if (state)
        g_mosfet_status |= (1 << (id - 1));
    else
        g_mosfet_status &= ~(1 << (id - 1));
}

void BSP_SPARK_mosfet_apply(void) {
    uint16_t physical_word = 0;

		_PF2 = 0; // Latch Low
	
    // --- TPIC 2 (Kaskad - Uzaktaki) Mapped to Mosfet 1-5 ---
    if (g_mosfet_status & (1 << 0)) physical_word |= (1 << 3); // M1  -> Drain 3
    if (g_mosfet_status & (1 << 1)) physical_word |= (1 << 4); // M2  -> Drain 4
    if (g_mosfet_status & (1 << 2)) physical_word |= (1 << 5); // M3  -> Drain 5
    if (g_mosfet_status & (1 << 3)) physical_word |= (1 << 6); // M4  -> Drain 6
    if (g_mosfet_status & (1 << 4)) physical_word |= (1 << 7); // M5  -> Drain 7

    // --- TPIC 1 (Birinci - Yakındaki) Mapped to Mosfet 6-10 ---
    if (g_mosfet_status & (1 << 5)) physical_word |= (1 << 12);  // M6  -> Drain 4
    if (g_mosfet_status & (1 << 6)) physical_word |= (1 << 13);  // M7  -> Drain 5
    if (g_mosfet_status & (1 << 7)) physical_word |= (1 << 12);  // M8  -> Drain 6
    if (g_mosfet_status & (1 << 8)) physical_word |= (1 << 11);  // M9  -> Drain 7
    if (g_mosfet_status & (1 << 9)) physical_word |= (1 << 8);   // M10 -> Drain 0

    // SPI Gönderim
    SPI_WRITE_TX(SPI1, physical_word);
    while(SPI_IS_BUSY(SPI1));
	
    _PF2 = 1; // Latch High (Veri çıkışlara aktarıldı)
}

void BSP_SPARK_pwm_set(int u32DutyCycle)
{

	// Güvenlik Kontrolü: On time 0 ise spark'ı hemen maskele (Kapat)
	if (u32DutyCycle <= 0)
	{
			//SET PWM OUT TO HIGH */
			EPWM_MASK_OUTPUT(EPWM1, EPWM_CH_4_MASK, EPWM_CH_4_MASK);
			return;
	}
	

	// Normal Çalışma: Maskeyi kaldır ve süreleri güncelle
	EPWM1->MSKEN &= ~(1UL << 4);      // Maskeyi kaldır (PWM sinyali pine gitsin)

	// set new duty 
	EPWM_SET_CMR(EPWM1, 4, u32DutyCycle * (EPWM_GET_CNR(EPWM1, 4) + 1U) / 100U);
}

void BSP_SPARK_mosfet_init(void) {
    SYS_UnlockReg();
    
	  CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, 0);
		CLK_EnableModuleClock(SPI1_MODULE);

	  /* Setup SPI1 multi-function pins */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC6MFP_SPI1_MOSI | SYS_GPC_MFPL_PC7MFP_SPI1_MISO);
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA7MFP_Msk;
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA7MFP_SPI1_CLK);

    /* Latch Pin (PF2) */
    GPIO_SetMode(PF, BIT2, GPIO_MODE_OUTPUT);
    _PF2 = 0;

    /* 16-Bit Master Mod, 2MHz */
    SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 16, 1000000);
    SPI_SET_MSB_FIRST(SPI1);
    
    SYS_LockReg();
}

void BSP_SPARK_pwm_init(void)
{
		SYS_UnlockReg();
	
		CLK_EnableModuleClock(EPWM1_MODULE);  
		CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);
	  

    /* PC1 Pinini EPWM1_CH4 Olarak Ayarla */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_EPWM1_CH4;

		EPWM_ConfigOutputChannel(EPWM1, 4, 20000, 50);

		/* Başlangıçta Spark Kapalı (PWM OUT HIGH)*/
		EPWM_MASK_OUTPUT(EPWM1, EPWM_CH_4_MASK, EPWM_CH_4_MASK);
	
		/* Çıkışı ve Sayıcıyı Başlat */
		EPWM_EnableOutput(EPWM1, EPWM_CH_4_MASK);
		EPWM_Start(EPWM1, EPWM_CH_4_MASK);

		#if 0
    /* 3. EPWM1_CH4 Yapılandırması */
    /* Prescaler ayarı: PCLK / (191 + 1) = 192MHz / 192 = 1MHz (1 tick = 1us) */
    EPWM_SET_PRESCALER(EPWM1, 4, 191); 

    /* Up-count modunda çalış */
    EPWM_SET_ALIGNED_TYPE(EPWM1, BIT4, EPWM_EDGE_ALIGNED);

    /* Sayıcı 0 iken: High (1) yap */
    /* Sayıcı Compare iken: Low (0) yap */
    /* Böylece 0 -> offTime arası 1, offTime -> CNR arası 0 olur (Spark On) */
    EPWM1->WGCTL0 = (EPWM1->WGCTL0 & ~(3UL << 8)) | (1UL << 8); // Zero point: High
    EPWM1->WGCTL1 = (EPWM1->WGCTL1 & ~(3UL << 8)) | (2UL << 8); // Compare Up: Low

    /* Başlangıçta Spark Kapalı */
    EPWM1->MSK |= (1UL << 4);     // CH4 Maske verisi = 1 (High)
    EPWM1->MSKEN |= (1UL << 4);   // CH4 Maskeleme aktif

    /* Çıkışı ve Sayıcıyı Başlat */
    EPWM1->POEN |= (1UL << 4);    // Output Enable
    EPWM1->CNTEN |= (1UL << 4);   // Counter Enable
		#endif
		
		SYS_LockReg();
}

void BSP_init(void) {
  /* configure gpio ports */
  for (int i = 0; i < Q_DIM(board_pins); i++)
    bsp_hal_gpio_init(&board_pins[i], 0);
	
	/* init spark mosfets */
	BSP_SPARK_mosfet_init();
	BSP_SPARK_mosfet_apply();
	
	/* init spark PWM */
	BSP_SPARK_pwm_init();
	
	/* init usb */
	BSP_usb_init();
}

/*..........................................................................*/
void QF_onCleanup(void) {}

/* QF callbacks ============================================================*/
void QF_onStartup(void) {
  /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate */
  SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

/* assing all priority bits for preemption-prio. and none to sub-prio. */
#if defined(GD32F310) || defined(M4)
  NVIC_SetPriorityGrouping(0U);
#endif
  /* set priorities of ALL ISRs used in the system, see NOTE1
   *
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
   * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
   */
  NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI);
  /* ... */

  /* enable IRQs... */
	NVIC_EnableIRQ(USBD_IRQn);

	//BSP_SPARK_mosfet_set(1,1);
	/*
	BSP_SPARK_mosfet_set(2,1);
	BSP_SPARK_mosfet_set(3,1);
	BSP_SPARK_mosfet_set(4,1);
	BSP_SPARK_mosfet_set(5,1);
	BSP_SPARK_mosfet_set(6,1);
	BSP_SPARK_mosfet_set(7,1);
	BSP_SPARK_mosfet_set(8,1);
	BSP_SPARK_mosfet_set(9,1);
	BSP_SPARK_mosfet_set(10,1);
	*/
	
	BSP_SPARK_mosfet_apply();
	
	BSP_SPARK_pwm_set(95);
	
}

/*..........................................................................*/


void QK_onIdle(void) {
	static uint32_t tickLed = 0;
  //QF_INT_DISABLE();

	if (tickSys-tickLed>=10) {
		tickLed = tickSys;
		_PF5 = 1 - _PF5;
	}
	
	
	
	VCOM_TransferData();	
	
	
	
	
  //QF_INT_ENABLE();

#ifdef NDEBUG
  /* Put the CPU and peripherals to the low-power mode.
   * you might need to customize the clock management for your application,
   * see the datasheet for your particular Cortex-M3 MCU.
   */
  __WFI(); /* Wait-For-Interrupt */
#endif
}

/*..........................................................................*/
Q_NORETURN Q_onAssert(char const* const module, int_t const loc) {
  /*
   * NOTE: add here your application-specific error handling
   */
  (void)module;
  (void)loc;

  QS_ASSERTION(module, loc, 10000U); /* report assertion to QS */

  while (1) {
  }

  NVIC_SystemReset();
}

/*****************************************************************************
 * NOTE1:
 * The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
 * ISR priority that is disabled by the QF framework. The value is suitable
 * for the NVIC_SetPriority() CMSIS function.
 *
 * Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
 * with the numerical values of priorities equal or higher than
 * QF_AWARE_ISR_CMSIS_PRI) are allowed to call any QF services. These ISRs
 * are "QF-aware".
 *
 * Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
 * level (i.e., with the numerical values of priorities less than
 * QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
 * Such "QF-unaware" ISRs cannot call any QF services. The only mechanism
 * by which a "QF-unaware" ISR can communicate with the QF framework is by
 * triggering a "QF-aware" ISR, which can post/publish events.
 *
 * NOTE2:
 * One of the LEDs is used to visualize the idle loop activity. The brightness
 * of the LED is proportional to the frequency of invcations of the idle loop.
 * Please note that the LED is toggled with interrupts locked, so no interrupt
 * execution time contributes to the brightness of the User LED.
 */
