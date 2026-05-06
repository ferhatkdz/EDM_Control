#include "bsp.h"
#include "cli.h"
#include "motor.h"
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
