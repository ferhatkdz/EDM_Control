#include "bsp.h"
#include "cli.h"
#include "qpc.h"

Q_DEFINE_THIS_FILE

static QEvt const *cliQSto[10];          /* Event queue storage for Cli */
static QF_MPOOL_EL(QEvt) smlPoolSto[50]; /* Small pool storage for dynamic events */

void SYS_Init(void) {
  /* Unlock protected registers */
  SYS_UnlockReg();

  /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
  PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

  /* Enable HIRC clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

  /* Waiting for 12MHz clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

  /* Set core clock as PLL_CLOCK from PLL */
  CLK_SetCoreClock(FREQ_192MHZ);

  /* Set both PCLK0 and PCLK1 as HCLK/2 */
  CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

  /* Update System Core Clock */
  /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
  SystemCoreClockUpdate();
	
	// VREF voltaj seviyesini seç 
	SYS->VREFCTL = SYS_VREFCTL_VREF_3_0V;



	/* EADC Saatini Etkinlestir */
	CLK_EnableModuleClock(EADC_MODULE);

	/* EADC Saat Bölücüsünü Ayarla (192MHz / (7+1) = 24MHz) */
	/* EADC hizi 72MHz'i (veya DS'deki siniri) geçmemelidir. 24MHz güvenlidir. */	
	CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

	/* Set PB.12 ~ PB.15 to input mode */
	PB->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk | GPIO_MODE_MODE15_Msk);

	/* PB.12 (CH12) PB.13 (CH13) PB.14 (CH14) ve PB.15 (CH15) Pinlerini Analog Giris Yap */
	SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk | SYS_GPB_MFPH_PB15MFP_Msk)) |
									(SYS_GPB_MFPH_PB12MFP_EADC0_CH12 |SYS_GPB_MFPH_PB13MFP_EADC0_CH13 | SYS_GPB_MFPH_PB14MFP_EADC0_CH14 | SYS_GPB_MFPH_PB15MFP_EADC0_CH15); 

	/* Dijital Girisi Kapat (Analog hassasiyeti için kritik) */
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT12 | BIT13 | BIT14 | BIT15);

	/* ADC Modülünü Aç */
	EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

  /* Clear the A/D ADINTx interrupt flag for safe */
  EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);


	/* EPWM1 Saatini Etkinlestir */
	CLK_EnableModuleClock(EPWM1_MODULE);  
	
	/* select EPWM module clock source as PCLK1 */
	CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

	/* PC1 Pinini EPWM1_CH4 Olarak Ayarla */
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC1MFP_Msk) | SYS_GPC_MFPL_PC1MFP_EPWM1_CH4;

	// PC.1 - PC.5 arasini EPWM1 fonksiyonuna ata
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                (SYS_GPC_MFPL_PC5MFP_EPWM1_CH0 | SYS_GPC_MFPL_PC4MFP_EPWM1_CH1 | SYS_GPC_MFPL_PC3MFP_EPWM1_CH2 | SYS_GPC_MFPL_PC2MFP_EPWM1_CH3 | SYS_GPC_MFPL_PC1MFP_EPWM1_CH4);


	/*--- QEI Clock ---*/
	CLK_EnableModuleClock(QEI1_MODULE);

	/*--- Pin MUX ---*/
	SYS->GPA_MFPH =  (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
									 (SYS_GPA_MFPH_PA8MFP_QEI1_B  | SYS_GPA_MFPH_PA9MFP_QEI1_A  | SYS_GPA_MFPH_PA10MFP_QEI1_INDEX);



  /* Lock protected registers */
  SYS_LockReg();
}

/*..........................................................................*/
int main(void) {
  SYS_Init();

  QF_init(); /* initialize the framework and the underlying RT kernel */

  /* publish-subscribe not used, no call to QF_psInit() */

  /* initialize the event pools... */
  QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

  BSP_init(); /* initialize the Board Support Package */

  /* instantiate and start the active objects... */
  Cli_ctor();
  QACTIVE_START(AO_Cli,           /* AO pointer to start */
                1U,               /* QF-priority/preemption-threshold */
                cliQSto,          /* storage for the AO's queue */
                Q_DIM(cliQSto),   /* length of the queue [entries] */
                (void *)0,        /* stack storage (not used in QK) */
                0U,               /* stack size [bytes] (not used in QK) */
                (QEvt *)0);       /* initial event (or 0) */

  return QF_run();                /* run the QF application */
}
