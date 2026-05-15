#include "bsp.h"
#include "cli.h"
#include "gcode_ao.h"
#include "motion_ao.h"
#include "axis_comm_ao.h"
#include "events.h"
#include "qpc.h"

Q_DEFINE_THIS_FILE

/* ------------------------------------------------------------------ */
/*  Event queue storage — her AO için ayrı                            */
/* ------------------------------------------------------------------ */
static QEvt const *gcodeQSto[32];      /* GCodeAO  (öncelik 4 — en yüksek) */
static QEvt const *motionQSto[16];     /* MotionAO (öncelik 3)              */
static QEvt const *axisCommQSto[16];   /* AxisCommAO (öncelik 2)            */
static QEvt const *cliQSto[10];        /* CliAO (öncelik 1 — en düşük)      */

/* ------------------------------------------------------------------ */
/*  Event pool storage — boyuta göre küçükten büyüğe                  */
/* ------------------------------------------------------------------ */
static QF_MPOOL_EL(QEvt)        smlPoolSto[30];   /* küçük (sadece sig) */
static QF_MPOOL_EL(GCodeRspEvt) rspPoolSto[10];   /* GCodeRspEvt        */
static QF_MPOOL_EL(GCodeEvt)    gcodePoolSto[20]; /* GCodeEvt (en büyük)*/

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
  SystemCoreClockUpdate();

	// VREF voltaj seviyesini seç
	SYS->VREFCTL = SYS_VREFCTL_VREF_3_0V;

	/* EADC Saatini Etkinlestir */
	CLK_EnableModuleClock(EADC_MODULE);

	/* EADC Saat Bölücüsünü Ayarla (192MHz / (7+1) = 24MHz) */
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

	/* PC.1 - PC.5 arasini EPWM1 fonksiyonuna ata */
	SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                (SYS_GPC_MFPL_PC5MFP_EPWM1_CH0 | SYS_GPC_MFPL_PC4MFP_EPWM1_CH1 | SYS_GPC_MFPL_PC3MFP_EPWM1_CH2 | SYS_GPC_MFPL_PC2MFP_EPWM1_CH3 | SYS_GPC_MFPL_PC1MFP_EPWM1_CH4);

	/*--- Z ekseni QEI1 Clock ---*/
	CLK_EnableModuleClock(QEI1_MODULE);

	/*--- Z ekseni QEI1 Pin MUX (PA8-10, GPA_MFPH) ---*/
	SYS->GPA_MFPH = (SYS->GPA_MFPH &
	                  ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
	                 (SYS_GPA_MFPH_PA8MFP_QEI1_B | SYS_GPA_MFPH_PA9MFP_QEI1_A | SYS_GPA_MFPH_PA10MFP_QEI1_INDEX);

	/*--- W ekseni EPWM0 Clock ---*/
	CLK_EnableModuleClock(EPWM0_MODULE);
	CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

	/*--- W ekseni EPWM0 Pin MUX (PB2-5, GPB_MFPL) ---*/
	/* PB2->EPWM0_CH3(BL), PB3->EPWM0_CH2(BH),
	 * PB4->EPWM0_CH1(AL), PB5->EPWM0_CH0(AH)      */
	SYS->GPB_MFPL = (SYS->GPB_MFPL &
	                  ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk |
	                    SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
	                 (SYS_GPB_MFPL_PB2MFP_EPWM0_CH3 | SYS_GPB_MFPL_PB3MFP_EPWM0_CH2 |
	                  SYS_GPB_MFPL_PB4MFP_EPWM0_CH1 | SYS_GPB_MFPL_PB5MFP_EPWM0_CH0);

	/*--- W ekseni QEI0 Clock ---*/
	CLK_EnableModuleClock(QEI0_MODULE);

	/*--- W ekseni QEI0 Pin MUX (PA3-5, GPA_MFPL) ---*/
	/* PA3->QEI0_B, PA4->QEI0_A, PA5->QEI0_INDEX */
	SYS->GPA_MFPL = (SYS->GPA_MFPL &
	                  ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk |
	                    SYS_GPA_MFPL_PA5MFP_Msk)) |
	                 (SYS_GPA_MFPL_PA3MFP_QEI0_B | SYS_GPA_MFPL_PA4MFP_QEI0_A |
	                  SYS_GPA_MFPL_PA5MFP_QEI0_INDEX);

  /* Lock protected registers */
  SYS_LockReg();
}

/*..........................................................................*/
int main(void) {
  SYS_Init();

  QF_init(); /* initialize the framework and the underlying RT kernel */

  /* Pub-sub tablosu: MAX_PUB_SIG kadar sinyal yayınlanabilir */
  static QSubscrList subscrSto[MAX_PUB_SIG];
  QF_psInit(subscrSto, Q_DIM(subscrSto));

  /* Event pool'ları küçükten büyüğe sırayla kaydet */
  QF_poolInit(smlPoolSto,   sizeof(smlPoolSto),   sizeof(smlPoolSto[0]));
  QF_poolInit(rspPoolSto,   sizeof(rspPoolSto),   sizeof(rspPoolSto[0]));
  QF_poolInit(gcodePoolSto, sizeof(gcodePoolSto), sizeof(gcodePoolSto[0]));

  BSP_init(); /* initialize the Board Support Package */

  /* AO'ları oluştur ve başlat — düşük öncelikten yükseğe */
  Cli_ctor();
  QACTIVE_START(AO_Cli,
                1U,
                cliQSto,        Q_DIM(cliQSto),
                (void *)0, 0U,
                (QEvt *)0);

  AxisComm_ctor();
  QACTIVE_START(AO_AxisComm,
                2U,
                axisCommQSto,   Q_DIM(axisCommQSto),
                (void *)0, 0U,
                (QEvt *)0);

  Motion_ctor();
  QACTIVE_START(AO_Motion,
                3U,
                motionQSto,     Q_DIM(motionQSto),
                (void *)0, 0U,
                (QEvt *)0);

  GCode_ctor();
  QACTIVE_START(AO_GCode,
                4U,
                gcodeQSto,      Q_DIM(gcodeQSto),
                (void *)0, 0U,
                (QEvt *)0);

  return QF_run();                /* run the QF application */
}
