#include "bsp.h"
#include "cli.h"
#include "gcode_ao.h"
#include "vcom_serial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "motor.h"
#include "ark.h"
#include "probe.h"
#include "velocity.h"

void debug_usb(char *format, ...);
void BSP_SPARK_mosfet_set(uint8_t id, uint8_t state);
void BSP_SPARK_mosfet_apply(void);
void BSP_SPARK_pwm_set(int u32DutyCycle);
	
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

/* EMA Filtre Katsayısı (Bit Shift)
 * Değer büyüdükçe filtre güçlenir (tepkime yavaşlar, gürültü azalır).
 * 3 -> 1/8  (Orta seviye filtre)
 * 4 -> 1/16 (Güçlü filtre)
 * 5 -> 1/32 (Çok güçlü filtre)
 */
#define EMA_SHIFT_VAL   4  

/* Global olarak filtrelenmiş verileri tutacağımız değişkenler 
 * (Diğer dosyalardan extern ile erişebilirsiniz) */
volatile uint32_t g_u32FilteredGapVoltage = 0;
volatile uint32_t g_u32FilteredSparkCurrent = 0;

/* EMA Akümülatörleri (Hassasiyet kaybını önlemek için yüksek tutulur) */
static uint32_t s_u32GapVolAccumulator = 0;
static uint32_t s_u32SparkCurAccumulator = 0;

								
volatile int32_t pos_now; 

void SysTick_Handler(void) {
  QK_ISR_ENTRY(); /* inform QK about entering an ISR */

  tickSys++;

  QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); /* process time events for rate 0 */

  QK_ISR_EXIT(); /* inform QK about exiting an ISR */
}

void EADC00_IRQHandler(void)
{
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

/* Z motor akımı — EADC Modül 2, CH15, EPWM1TG0 tetiklemeli */
void EADC02_IRQHandler(void)
{
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
	g_axes[AXIS_Z].current_adc = EADC_GET_CONV_DATA(EADC, 2);
}

/* W motor akımı — EADC Modül 3, CH12, EPWM0TG0 tetiklemeli */
void EADC03_IRQHandler(void)
{
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);
	g_axes[AXIS_W].current_adc = EADC_GET_CONV_DATA(EADC, 3);
}

void EADC01_IRQHandler(void)
{
		QK_ISR_ENTRY(); /* inform QK about entering an ISR */

		EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

		g_u32FilteredSparkCurrent = EADC_GET_CONV_DATA(EADC, 0);
		g_u32FilteredGapVoltage   = EADC_GET_CONV_DATA(EADC, 1);

		/* Ark servo loop (state machine + Z-ekseni servo). İçeride
		 * /5 prescaler ile 1 kHz çalışıyor. Ad-hoc hysteresis kodu
		 * ark modülüne taşındı.                                      */
		Ark_Tick();
		Probe_Tick();

	#if 0
    /* 1. Kesme Bayrağını Temizle (Biz INT0 kullanmıştık - BIT0) */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* 2. ADC Ham Verilerini Oku 
       (EADC'in 13. ve 14. örnekleme modülleri) */
    uint32_t u32RawSparkCur = EADC_GET_CONV_DATA(EADC, 13);
    uint32_t u32RawGapVol   = EADC_GET_CONV_DATA(EADC, 14);

    /* --- 3. SPARK AKIMI İÇİN EMA FİLTRESİ --- */
    /* İlk çalışma durumu (Akümülatör boşsa ilk değerle doldur) */
    if (s_u32SparkCurAccumulator == 0) {
        s_u32SparkCurAccumulator = u32RawSparkCur << EMA_SHIFT_VAL;
    }
    /* EMA Matematik Modeli: Akümülatör = Akümülatör + Ham Değer - (Akümülatör / Filtre) */
    s_u32SparkCurAccumulator += u32RawSparkCur - (s_u32SparkCurAccumulator >> EMA_SHIFT_VAL);
    
    /* Gerçek filtrelenmiş değeri elde etmek için akümülatörü geri kaydır */
    g_u32FilteredSparkCurrent = s_u32SparkCurAccumulator >> EMA_SHIFT_VAL;


    /* --- 4. GAP GERİLİMİ İÇİN EMA FİLTRESİ --- */
    if (s_u32GapVolAccumulator == 0) {
        s_u32GapVolAccumulator = u32RawGapVol << EMA_SHIFT_VAL;
    }
    s_u32GapVolAccumulator += u32RawGapVol - (s_u32GapVolAccumulator >> EMA_SHIFT_VAL);
    
    g_u32FilteredGapVoltage = s_u32GapVolAccumulator >> EMA_SHIFT_VAL;

    /* * NOT: Filtrelenmiş GAP ve Akım değerleri hazır. 
     * Motorun Z eksenini (Servo/Step) hareket ettirecek olan PID 
     * fonksiyonunu veya durum makinesini (State Machine) burada 
     * tetikleyebilir veya ana döngüde (main loop) bu global değişkenleri 
     * okuyarak işleyebilirsiniz.
     */
		 #endif
		
		QK_ISR_EXIT(); /* inform QK about exiting an ISR */
}


void BSP_pin_set(const void* const me, uint8_t val) {
  const gpio_t* const gpio = (gpio_t* const)((out_t*)me)->out;

  bsp_hal_gpio_write(gpio, val);
}

void BSP_cli_puts(char* buf) {
	BSP_cli_transmit(buf, strlen(buf));
}

void BSP_cli_transmit(char* buf, int length) {
	__set_PRIMASK(1);	
	for (int i=0; i<length; i++) {
		comRbuf[comRtail++] = buf[i];	
		if (comRtail >= RXBUFSIZE) comRtail = 0;
    comRbytes++;		
	}
	__set_PRIMASK(0);
}


void CDC_SendFmt(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    //CDC_Send(buf);
	
	__set_PRIMASK(1);	
	for (int i=0; i<strlen(buf); i++) {
		comRbuf[comRtail++] = buf[i];	
		if (comRtail >= RXBUFSIZE) comRtail = 0;
    comRbytes++;		
	}
	__set_PRIMASK(0);	
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

void VCOM_ResetState(void) {
	__set_PRIMASK(1);
	comRhead = comRtail = comRbytes = 0;
	comThead = comTtail = comTbytes = 0;
	gu32TxSize = 0;
	gu32RxSize = 0;
	gi8BulkOutReady = 0;
	__set_PRIMASK(0);
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
			#if 0
			for (i = 0; i<comTbytes; i++) {
				debug_usb("%c", comTbuf[comThead++]);
				if(comThead >= TXBUFSIZE)
							comThead = 0;
				comTbytes--;
			}
			#else
			for (i = 0; i<comTbytes; i++) {
				
				UartEvt* e = Q_NEW(UartEvt, UART_RX_SIG);
				e->ch      = comTbuf[comThead++];
				/* USB VCOM karakterleri GCodeAO'ya yönlendir (Faz 2+) */
				QACTIVE_POST(AO_GCode, &e->super, 0);
			
				if(comThead >= TXBUFSIZE)
							comThead = 0;
				comTbytes--;
			}
			#endif
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
	EPWM1->MSKEN &= ~(EPWM_CH_4_MASK);  // Maskeyi kaldır (PWM sinyali pine gitsin)

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

void BSP_SPARK_adc_init(void)
{
		SYS_UnlockReg();
		
		/* ----- Sample Module 0: CH13, EPWM1CH4 tetiklemeli ----- */
    EADC_ConfigSampleModule(EADC,
                            0,                              /* modül indeksi       */
                            EADC_EPWM1TG4_TRIGGER,          /* EPWM1 CH4 tetikle   */
                            13);                            /* ADC kanalı 13       */
		EADC_ConfigSampleModule(EADC,
                            1,                              /* modül indeksi       */
                            EADC_EPWM1TG4_TRIGGER,          /* EPWM1 CH4 tetikle   */
                            14);                            /* ADC kanalı 14       */
				
    /* Modül 1 dönüşüm tamamlandığında ADINT1 oluştur */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT1);   /* ADINT1 <- Module1 */

    /* ADC interrupt'larını etkinleştir */
    EADC_ENABLE_INT(EADC, BIT1);    /* ADINT1 */
		
		SYS_LockReg();
}

void BSP_SPARK_pwm_init(uint32_t u32Frequency)
{
		SYS_UnlockReg();
	
		/* Merkezi Hizalama (Up-Down Count) Moduna Al
       Bu mod, gürültünün en az olduğu ON süresi ortasında ölçüm yapmamızı sağlar. 
			 EPWM_ConfigOutputChannel den önce çağırılmalıdır. */
    EPWM_SET_ALIGNED_TYPE(EPWM1, BIT4, EPWM_CENTER_ALIGNED);

		/* PWM Yapılandırması: u32Frequency KHz, %0 Duty (Başlangıçta kapalı) */
		EPWM_ConfigOutputChannel(EPWM1, 4, u32Frequency, 0);

    /* ADC Tetikleme Özelliği: Sayaç Periyot (Tepe) noktasına ulaştığında tetikle */
    EPWM_EnableADCTrigger(EPWM1, 4, EPWM_TRG_ADC_EVEN_PERIOD); 
	
		/* Başlangıçta Spark Kapalı (PWM OUT HIGH)*/
		EPWM_MASK_OUTPUT(EPWM1, EPWM_CH_4_MASK, EPWM_CH_4_MASK);
	
	
	/*
		EPWM_SET_OUTPUT_LEVEL(epwm, u32ChannelMask, u32ZeroLevel, u32CmpUpLevel, u32PeriodLevel, u32CmpDownLevel)
	        *
	      *   *
	    *       *
	  *           *
		
		  __________
		_|          |_
		
	*/
	
		EPWM_SET_OUTPUT_LEVEL(EPWM1, BIT4,
                          EPWM_OUTPUT_LOW,   		/* sayıcı yukarı, CMP eşleşmesi  */
                          EPWM_OUTPUT_HIGH,    	/* sayıcı aşağı, CMP eşleşmesi  */
                          EPWM_OUTPUT_NOTHING,
                          EPWM_OUTPUT_NOTHING);

		/* Çıkışı ve Sayıcıyı Başlat */
		EPWM_EnableOutput(EPWM1, EPWM_CH_4_MASK);
		EPWM_Start(EPWM1, EPWM_CH_4_MASK);
		
		SYS_LockReg();
}

void BSP_AXIS_Z_adc_Init(void)
{
		SYS_UnlockReg();
		
		/* ----- Sample Module 2: CH15, EPWM1CH0 tetiklemeli ----- */
    EADC_ConfigSampleModule(EADC,
                            2,                              /* modül indeksi       */
                            EADC_EPWM1TG0_TRIGGER,          /* EPWM1 CH0 tetikle   */
                            15);                            /* ADC kanalı 15       */
				
    /* Modül 2 dönüşüm tamamlandığında ADINT2 oluştur */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, BIT2);   /* ADINT2 <- Module2 */

    /* ADC interrupt'larını etkinleştir */
    EADC_ENABLE_INT(EADC, BIT2);    /* ADINT2 */
		
		SYS_LockReg();	
}



#define PCLK1_HZ             96000000UL

/*
 * Dead-time: 500ns @ 96MHz -> 500ns * 96 = 48 clock
 * EPWM_EnableDeadZone() parametresi clock count cinsinden
 */
#define DEAD_TIME_CYCLES  ((500UL * (PCLK1_HZ / 1000000UL)) / 1000UL)    /* 48 */


/* Bootstrap guard: min off-time için
 * CU11=2.2µF, I_q=200µA → 11ms hold
 * 20kHz'de her periyot zaten şarj eder
 * Ama duty=%100 için guard: min 200ns off
 * = DEAD_TIME_CYCLES yeterli                         */
#define BOOTSTRAP_GUARD    (DEAD_TIME_CYCLES + 5UL)

/* Duty ölçeği: -10000..+10000 (0.01% çözünürlük) */
#define DUTY_SCALE         10000

void BSP_AXIS_Z_pwm_init(uint32_t u32Freq) {
	SYS_UnlockReg();
	
	/* Merkezi Hizalama (Up-Down Count) Moduna Al
       Bu mod, gürültünün en az olduğu ON süresi ortasında ölçüm yapmamızı sağlar. 
			 EPWM_ConfigOutputChannel den önce çağırılmalıdır. */
  EPWM_SET_ALIGNED_TYPE(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK, 
															 EPWM_CENTER_ALIGNED);
	
	// Tam Köprü (Full Bridge) için CH0 ve CH2 Complementary (Tamamlayıcı) modu aç
	// EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM1); bu CH0 CH2 ve CH4 ün hepsini açıyor, aşağıdaki manuel CH0 ve CH2 yi açıyor.
	EPWM1->CTL1 |=  (EPWM_CTL1_OUTMODE0_Msk | EPWM_CTL1_OUTMODE2_Msk);

	// EPWM1 CH0 ve CH2 için Center-aligned (Merkez hizalı) mod seçelim (EMI için daha iyidir)
	EPWM_ConfigOutputChannel(EPWM1, 0, u32Freq, 50); // Sol Kol (AH-AL)
	EPWM_ConfigOutputChannel(EPWM1, 2, u32Freq, 50); // Sağ Kol (BH-BL)

		// 2us civarında bir dead-time (MOSFET hızına göre ayarlanabilir)
	EPWM_EnableDeadZone(EPWM1, 0, DEAD_TIME_CYCLES); // CH0-CH1 çifti için
	EPWM_EnableDeadZone(EPWM1, 2, DEAD_TIME_CYCLES); // CH2-CH3 çifti için	
	
	/* CH0'ın sync çıkışı: counter sıfıra eşit olduğunda pulse üret */
	EPWM_ConfigSyncPhase(EPWM1,
											 0,                              /* CH0/1 çifti */
											 EPWM_SYNC_OUT_FROM_COUNT_TO_ZERO, /* zero'da sync */
											 EPWM_PHS_DIR_INCREMENT,
											 0U);

	/* CH2'yi bu sync sinyali ile senkronize et */
	EPWM_ConfigSyncPhase(EPWM1,
											 2,                              /* CH2/3 çifti */
											 EPWM_SYNC_OUT_FROM_SYNCIN_SWSYNC, /* dışarıdan al */
											 EPWM_PHS_DIR_INCREMENT,
											 0U);                            /* phase offset = 0 */

	/* CH2 için phase load enable */
	EPWM_EnableSyncPhase(EPWM1, EPWM_CH_2_MASK);   /* CH2 PHSEN = 1 */
		
		
  /* ADC Tetikleme Özelliği: Sayaç Periyot (Tepe) noktasına ulaştığında tetikle */
  EPWM_EnableADCTrigger(EPWM1, 0, EPWM_TRG_ADC_EVEN_PERIOD); 	

	
  /* Motor_Enable() çağrılana kadar çıkışlar 0 */
  EPWM_MASK_OUTPUT(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK, 
													0UL);

	EPWM_EnableOutput(EPWM1, BIT0 | BIT1 | BIT2 | BIT3);	
	
	SYS_LockReg();
}


void BSP_AXIS_Z_qei_init(void)
{
	
	/* Reset QEI module */
  QEI_Close(QEI1);
	
	/* X4 mode: A ve B her iki kenarı say → max çözünürlük */
	QEI_Open(QEI1,
					 QEI_CTL_X4_FREE_COUNTING_MODE,
					 QEI_MAX_COUNT);
	
	/* Enable QEI noise filter */
	QEI_ENABLE_NOISE_FILTER(QEI1, QEI_CTL_NFCLKSEL_DIV4);

  /* Set QEI counter value to 0 */
  QEI_SET_CNT_VALUE(QEI1, 0);
	
	QEI_Start(QEI1);
}

void BSP_AXIS_Z_enable(void)
{
	/* Önce duty'yi merkeze al (0 hız) */
	EPWM_SET_CMR(EPWM1, 0, 50 * (EPWM_GET_CNR(EPWM1, 0) + 1U) / 100U);
	EPWM_SET_CMR(EPWM1, 2, 50 * (EPWM_GET_CNR(EPWM1, 2) + 1U) / 100U);

	/* Mask kaldır → çıkışlar aktif */
	EPWM1->MSKEN &= ~(EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK); 

	EPWM_EnableOutput(EPWM1,
			EPWM_CH_0_MASK | EPWM_CH_1_MASK |
			EPWM_CH_2_MASK | EPWM_CH_3_MASK);


	EPWM_Start(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK);
}

void BSP_AXIS_Z_disable(void)
{
	/* Motor_Enable() çağrılana kadar çıkışlar 0 */
  EPWM_MASK_OUTPUT(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK, 
													0UL);
}

void BSP_AXIS_z_reset_pos(void) {
	QEI_Stop(QEI1);
	QEI_SET_CNT_VALUE(QEI1, 0);
	QEI_Start(QEI1);
	pos_now = 0;
}


volatile int32_t last_duty = 0;

void BSP_AXIS_Z_set_duty(int32_t duty) {
	
	last_duty = duty;

	/* NOT: Dead-band kompansasyonu (±2000 ofset) zero-crossing'de
	 * süreksizlik yaratıp limit cycle'a yol açıyordu; kaldırıldı.
	 * Statik sürtünme telafisi gerekirse PID integral ile ele alınmalı. */

	/* Sınırla */
	if (duty >  DUTY_SCALE) duty =  DUTY_SCALE;
	if (duty < -DUTY_SCALE) duty = -DUTY_SCALE;

	/* Center-aligned bipolar:
	 * duty=0      → CMR = CNR/2        → net Vmotor=0
	 * duty=+MAX   → CMR → CNR          → net Vmotor=+VM
	 * duty=-MAX   → CMR → 0            → net Vmotor=-VM
	 *
	 * CMR_A = CNR/2 + duty*(CNR/2)/DUTY_SCALE
	 * CMR_B = CNR   - CMR_A  (ters faz, bipolar)
	 *
	 * Bootstrap guard: CMR_A ∈ [GUARD, CNR-GUARD]  */

	int32_t half = (int32_t)(EPWM_GET_CNR(EPWM1, 0) / 2);
	int32_t cmp_a = half + (duty * half / DUTY_SCALE);

	/* Bootstrap guard uygula */
	if (cmp_a < (int32_t)BOOTSTRAP_GUARD)
			cmp_a = (int32_t)BOOTSTRAP_GUARD;
	if (cmp_a > (int32_t)(EPWM_GET_CNR(EPWM1, 0) - BOOTSTRAP_GUARD))
			cmp_a = (int32_t)(EPWM_GET_CNR(EPWM1, 0) - BOOTSTRAP_GUARD);

	int32_t cmp_b = (int32_t)EPWM_GET_CNR(EPWM1, 0) - cmp_a;

	EPWM_SET_CMR(EPWM1, 0, (uint32_t)cmp_a); /* AH master */
	EPWM_SET_CMR(EPWM1, 2, (uint32_t)cmp_b); /* BH master */
}

/* Z get_pos — fonksiyon işaretçisi için (statik, bsp.h'de yok) */
static int32_t BSP_AXIS_Z_get_pos(void)
{
	return QEI_GetSignedCount(QEI1);
}

/*==========================================================
 * W EKSENİ BSP — EPWM0 (PB2-5) + QEI0 (PA3-5)
 * Tüm fonksiyonlar statik; AxisHW_t tablosu üzerinden erişilir.
 *==========================================================*/

static void BSP_AXIS_W_pwm_init(uint32_t u32Freq)
{
	SYS_UnlockReg();

	EPWM_SET_ALIGNED_TYPE(EPWM0,
		EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK,
		EPWM_CENTER_ALIGNED);

	/* Tam köprü: CH0 ve CH2 complementary mod */
	EPWM0->CTL1 |= (EPWM_CTL1_OUTMODE0_Msk | EPWM_CTL1_OUTMODE2_Msk);

	EPWM_ConfigOutputChannel(EPWM0, 0, u32Freq, 50); /* Sol kol (AH-AL) */
	EPWM_ConfigOutputChannel(EPWM0, 2, u32Freq, 50); /* Sağ kol (BH-BL) */

	EPWM_EnableDeadZone(EPWM0, 0, DEAD_TIME_CYCLES);
	EPWM_EnableDeadZone(EPWM0, 2, DEAD_TIME_CYCLES);

	/* CH0 sync çıkışı: sayaç sıfıra eşit olduğunda */
	EPWM_ConfigSyncPhase(EPWM0, 0,
		EPWM_SYNC_OUT_FROM_COUNT_TO_ZERO,
		EPWM_PHS_DIR_INCREMENT, 0U);

	/* CH2'yi sync sinyali ile senkronize et */
	EPWM_ConfigSyncPhase(EPWM0, 2,
		EPWM_SYNC_OUT_FROM_SYNCIN_SWSYNC,
		EPWM_PHS_DIR_INCREMENT, 0U);
	EPWM_EnableSyncPhase(EPWM0, EPWM_CH_2_MASK);

	/* Motor akımı ADC tetiklemesi — sayaç tepe noktasında */
	EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

	/* enable() çağrılana kadar çıkışlar maskeli */
	EPWM_MASK_OUTPUT(EPWM0,
		EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK,
		0UL);

	EPWM_EnableOutput(EPWM0, BIT0 | BIT1 | BIT2 | BIT3);

	SYS_LockReg();
}

static void BSP_AXIS_W_adc_init(void)
{
	SYS_UnlockReg();

	/* Modül 3: CH12, EPWM0 CH0 periyodunda tetikle */
	EADC_ConfigSampleModule(EADC, 3, EADC_EPWM0TG0_TRIGGER, 12);
	EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 3, BIT3);   /* ADINT3 <- Modül3 */
	EADC_ENABLE_INT(EADC, BIT3);

	SYS_LockReg();
}

static void BSP_AXIS_W_qei_init(void)
{
	QEI_Close(QEI0);
	QEI_Open(QEI0, QEI_CTL_X4_FREE_COUNTING_MODE, QEI_MAX_COUNT);
	QEI_ENABLE_NOISE_FILTER(QEI0, QEI_CTL_NFCLKSEL_DIV4);
	QEI_SET_CNT_VALUE(QEI0, 0);
	QEI_Start(QEI0);
}

static void BSP_AXIS_W_enable(void)
{
	EPWM_SET_CMR(EPWM0, 0, 50 * (EPWM_GET_CNR(EPWM0, 0) + 1U) / 100U);
	EPWM_SET_CMR(EPWM0, 2, 50 * (EPWM_GET_CNR(EPWM0, 2) + 1U) / 100U);

	EPWM0->MSKEN &= ~(EPWM_CH_0_MASK | EPWM_CH_1_MASK |
	                   EPWM_CH_2_MASK | EPWM_CH_3_MASK);

	EPWM_EnableOutput(EPWM0,
		EPWM_CH_0_MASK | EPWM_CH_1_MASK |
		EPWM_CH_2_MASK | EPWM_CH_3_MASK);

	EPWM_Start(EPWM0,
		EPWM_CH_0_MASK | EPWM_CH_1_MASK |
		EPWM_CH_2_MASK | EPWM_CH_3_MASK);
}

static void BSP_AXIS_W_disable(void)
{
	EPWM_MASK_OUTPUT(EPWM0,
		EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK,
		0UL);
}

static void BSP_AXIS_W_reset_pos(void)
{
	QEI_Stop(QEI0);
	QEI_SET_CNT_VALUE(QEI0, 0);
	QEI_Start(QEI0);
}

static int32_t BSP_AXIS_W_get_pos(void)
{
	/* W encoder faz yönü test sonrası ayarlanacak;
	 * Z ile aynı negatif sözleşme başlangıç noktası.   */
	return QEI_GetSignedCount(QEI0);
}

static void BSP_AXIS_W_set_duty(int32_t duty)
{
	if (duty >  DUTY_SCALE) duty =  DUTY_SCALE;
	if (duty < -DUTY_SCALE) duty = -DUTY_SCALE;

	int32_t half  = (int32_t)(EPWM_GET_CNR(EPWM0, 0) / 2);
	int32_t cmp_a = half + (duty * half / DUTY_SCALE);

	if (cmp_a < (int32_t)BOOTSTRAP_GUARD)
		cmp_a = (int32_t)BOOTSTRAP_GUARD;
	if (cmp_a > (int32_t)(EPWM_GET_CNR(EPWM0, 0) - BOOTSTRAP_GUARD))
		cmp_a = (int32_t)(EPWM_GET_CNR(EPWM0, 0) - BOOTSTRAP_GUARD);

	int32_t cmp_b = (int32_t)EPWM_GET_CNR(EPWM0, 0) - cmp_a;

	EPWM_SET_CMR(EPWM0, 0, (uint32_t)cmp_a); /* AH master */
	EPWM_SET_CMR(EPWM0, 2, (uint32_t)cmp_b); /* BH master */
}

/*----------------------------------------------------------
 * Donanım tabloları — ROM'da saklanır (const)
 *----------------------------------------------------------*/
const AxisHW_t g_axis_z_hw = {
	.pwm_init  = BSP_AXIS_Z_pwm_init,
	.adc_init  = BSP_AXIS_Z_adc_Init,
	.qei_init  = BSP_AXIS_Z_qei_init,
	.enable    = BSP_AXIS_Z_enable,
	.disable   = BSP_AXIS_Z_disable,
	.set_duty  = BSP_AXIS_Z_set_duty,
	.reset_pos = BSP_AXIS_z_reset_pos,
	.get_pos   = BSP_AXIS_Z_get_pos,
};

const AxisHW_t g_axis_w_hw = {
	.pwm_init  = BSP_AXIS_W_pwm_init,
	.adc_init  = BSP_AXIS_W_adc_init,
	.qei_init  = BSP_AXIS_W_qei_init,
	.enable    = BSP_AXIS_W_enable,
	.disable   = BSP_AXIS_W_disable,
	.set_duty  = BSP_AXIS_W_set_duty,
	.reset_pos = BSP_AXIS_W_reset_pos,
	.get_pos   = BSP_AXIS_W_get_pos,
};

void BSP_init(void) {
  /* configure gpio ports */
  for (int i = 0; i < Q_DIM(board_pins); i++)
    bsp_hal_gpio_init(&board_pins[i], 0);
	
	/* init spark mosfets */
	BSP_SPARK_mosfet_init();
	BSP_SPARK_mosfet_apply();

	/* init spark ADC */
	BSP_SPARK_adc_init();
	
	/* init spark PWM */
	BSP_SPARK_pwm_init(5000);
	
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
    NVIC_EnableIRQ(EADC00_IRQn);
	NVIC_EnableIRQ(EADC01_IRQn);
	NVIC_EnableIRQ(EADC02_IRQn);
	NVIC_EnableIRQ(EADC03_IRQn);
	
	
	NVIC_SetPriority(EADC00_IRQn, QF_AWARE_ISR_CMSIS_PRI+1);
	NVIC_SetPriority(EADC01_IRQn, QF_AWARE_ISR_CMSIS_PRI+1);
	NVIC_SetPriority(EADC02_IRQn, QF_AWARE_ISR_CMSIS_PRI+1);
	NVIC_SetPriority(EADC03_IRQn, QF_AWARE_ISR_CMSIS_PRI+1);

	
	BSP_SPARK_mosfet_set(1,1);
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

	/* Ark modülünü init et — spark KAPALI başlar.
	 * Kullanıcı CLI'dan 'ark on' ile aktif eder.                 */
	Ark_Init();

	/* Tüm eksenleri başlat (Z + W) — MotorControl_Init yerini aldı */
	AllAxes_Init();
	ControlTimer_Init();
}

/*..........................................................................*/

volatile int32_t err = 0;

void QK_onIdle(void) {
	static uint32_t tickLed = 0;
  //QF_INT_DISABLE();

	if (tickSys-tickLed>=10) {
		tickLed = tickSys;
		_PF5 = 1 - _PF5;
	}
	
	
	pos_now = QEI_GetSignedCount(QEI1);

	/* Aşırı akım hata raporlama — ISR flag'i ana döngüde CDC'ye yaz */
	for (int _ai = 0; _ai < AXIS_COUNT; _ai++) {
		if (g_axes[_ai].fault_overcurrent) {
			g_axes[_ai].fault_overcurrent = false;
			BSP_cli_puts("\r\nERROR: Axis ");
			BSP_cli_puts(_ai == AXIS_Z ? "Z" : "W");
			BSP_cli_puts(" overcurrent — motor stopped\r\n");
		}
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
