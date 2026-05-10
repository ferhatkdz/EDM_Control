#include "bsp.h"
#include "cli.h"
#include "vcom_serial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

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

volatile uint32_t g_u32MotorCurrentZ = 0;

void EADC02_IRQHandler(void)
{
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	
	g_u32MotorCurrentZ = EADC_GET_CONV_DATA(EADC, 2);
}


void EADC03_IRQHandler(void)
{
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

volatile int ark_on = 1;
void EADC01_IRQHandler(void)
{
		QK_ISR_ENTRY(); /* inform QK about entering an ISR */
	
		EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);      /* Clear the A/D ADINT0 interrupt flag */
	  
		g_u32FilteredSparkCurrent = EADC_GET_CONV_DATA(EADC, 0);
		g_u32FilteredGapVoltage   = EADC_GET_CONV_DATA(EADC, 1);
	
		if (g_u32FilteredGapVoltage < 20) {
			if (ark_on == 1) {
				//BSP_SPARK_mosfet_set(1,0);
				//BSP_SPARK_mosfet_apply();
				BSP_SPARK_pwm_set(2);
				ark_on = 0;
			}
		}else {
			if (ark_on == 0) {
				//BSP_SPARK_mosfet_set(1,1);
				//BSP_SPARK_mosfet_apply();
				BSP_SPARK_pwm_set(10);
				ark_on = 1;
			}
		}			

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
				QACTIVE_POST(AO_Cli, &e->super, 0);
			
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

void BSP_AXIS_Z_pwm_init(uint32_t u32Freq) {
	SYS_UnlockReg();
	
	/* Merkezi Hizalama (Up-Down Count) Moduna Al
       Bu mod, gürültünün en az olduğu ON süresi ortasında ölçüm yapmamızı sağlar. 
			 EPWM_ConfigOutputChannel den önce çağırılmalıdır. */
  EPWM_SET_ALIGNED_TYPE(EPWM1, BIT0 | BIT1 | BIT2 | BIT3, EPWM_CENTER_ALIGNED);
	
	// EPWM1 CH0 ve CH2 için Center-aligned (Merkez hizalı) mod seçelim (EMI için daha iyidir)
	EPWM_ConfigOutputChannel(EPWM1, 0, u32Freq, 50); // Sol Kol (AH-AL)
	EPWM_ConfigOutputChannel(EPWM1, 2, u32Freq, 50); // Sağ Kol (BH-BL)
	
  /* ADC Tetikleme Özelliği: Sayaç Periyot (Tepe) noktasına ulaştığında tetikle */
  EPWM_EnableADCTrigger(EPWM1, 0, EPWM_TRG_ADC_EVEN_PERIOD); 	

	// Tam Köprü (Full Bridge) için Complementary (Tamamlayıcı) modu aç
	EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM1);
	
	// 2us civarında bir dead-time (MOSFET hızına göre ayarlanabilir)
	EPWM_EnableDeadZone(EPWM1, 0, 200); // CH0-CH1 çifti için
	EPWM_EnableDeadZone(EPWM1, 2, 200); // CH2-CH3 çifti için	
	
	EPWM_EnableOutput(EPWM1, BIT0 | BIT1 | BIT2 | BIT3);	
	
	SYS_LockReg();
}

void BSP_AXIS_Z_set_speed(int32_t speed) {
	// speed: -500 ile 500 arası (0 durma noktası)
	// PWM_PERIOD_VALUE / 2 = %50 duty
	
	uint32_t duty_A = 500 + speed;
	uint32_t duty_B = 500 - speed;

	// Sınırları kontrol et
	if(duty_A > 950) duty_A = 950; // Bootstrap kapasitörü için %100 yapmıyoruz
	if(duty_A < 50)  duty_A = 50;
	
	EPWM_SET_CMR(EPWM1, 0, (duty_A * EPWM1->PERIOD[0]) / 1000);
	EPWM_SET_CMR(EPWM1, 2, ((1000 - duty_A) * EPWM1->PERIOD[2]) / 1000);
	
  // Start
	EPWM_Start(EPWM1, BIT0 | BIT1 | BIT2 | BIT3);

}

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
	
	/* init axis z adc */
	BSP_AXIS_Z_adc_Init();
	
	/* init axis z PWM */
	BSP_AXIS_Z_pwm_init(20000);
	
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
	
	BSP_SPARK_pwm_set(10);
	
	BSP_AXIS_Z_set_speed(500);
	
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
