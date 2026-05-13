#include "cli.h"
#include "bsp.h"
#include "qpc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor.h"

Cli Cli_inst; /* the active object */
QActive* const AO_Cli = &Cli_inst.super;

/*..........................................................................*/

#define CLI_HISTORY_MAX 5
char history[CLI_HISTORY_MAX][CLI_BUF_SIZE];
int hist_count = 0;
int hist_index = -1;
int in_history = 0;    // 0 = normal mod, 1 = geçmiş modundayız

static const char* cli_commands[] = {"help", "move", "stop", "status", "reset"};
#define CLI_COMMAND_COUNT (sizeof(cli_commands) / sizeof(cli_commands[0]))

static void SendPrompt(void);
static void ClearLine(void);
static void CLI_SaveHistory(const char* cmd);
static void CLI_HistoryUp(Cli* const me);
static void CLI_HistoryDown(Cli* const me);

static void CLI_ProcessCommand(char* cmd);


void Cli_ctor(void) {
  Cli* const me = &Cli_inst;
  QActive_ctor(&me->super, Q_STATE_CAST(&Cli_initial));
}

/* HSM definition ----------------------------------------------------------*/
QState Cli_initial(Cli* const me, QEvt const* const e) {
  me->idx = 0;
  //SendPrompt();
  return Q_TRAN(&Cli_idle);
}

QState Cli_idle(Cli* const me, QEvt const* const e) {
  typedef enum { ESC_STATE_NONE, ESC_STATE_ESC, ESC_STATE_CSI } esc_state_t;

  static esc_state_t esc_state = ESC_STATE_NONE;
  QState status                = Q_HANDLED();
  switch (e->sig) {
  case UART_RX_SIG: {
    char c = ((UartEvt const*)e)->ch;

    switch (esc_state) {
    case ESC_STATE_NONE:
      if (c == 0x1B) {    // ESC
        esc_state = ESC_STATE_ESC;
        break;
      }

      if (c == '\t') {    // TAB
        int matches            = 0;
        const char* last_match = NULL;

        for (int i = 0; i < CLI_COMMAND_COUNT; i++) {
          if (strncmp(me->buf, cli_commands[i], me->idx) == 0) {
            matches++;
            last_match = cli_commands[i];
          }
        }

        if (matches == 1) {
          // Tek eşleşme → otomatik tamamla
          strcpy(me->buf, last_match);
          me->idx = strlen(me->buf);
          ClearLine();
          BSP_cli_transmit((char*)me->buf, me->idx);
        } else if (matches > 1) {
          // Birden fazla → listeyi göster
          BSP_cli_puts("\r\n");
          for (int i = 0; i < CLI_COMMAND_COUNT; i++) {
            if (strncmp(me->buf, cli_commands[i], me->idx) == 0) {
              BSP_cli_puts((char*)cli_commands[i]);
              BSP_cli_puts("  ");
            }
          }
          SendPrompt();
          BSP_cli_transmit((char*)me->buf, me->idx);
        }
      } else

        // Normal karakter işleme
        if (c == '\r' || c == '\n') {
          me->buf[me->idx] = '\0';

          if (me->idx > 0) {
            CLI_SaveHistory(me->buf);
          }

          CLI_ProcessCommand(me->buf);
          me->idx    = 0;
          in_history = 0;     // geçmiş modundan çık
          hist_index = -1;    // index sıfırla

          SendPrompt();
        } else if ((me->idx < CLI_BUF_SIZE - 1) && (c >= 32 && c <= 127)) {
          if (c == 127) {    // Backspace
            if (me->idx) {
              me->idx--;
              if (me->idx == 0) {
                in_history = 0;
                hist_index = -1;
              }
            } else {
              break;
            }
          } else {
            me->buf[me->idx++] = c;
          }
          BSP_cli_transmit((char*)&c, 1);
        }else if ((me->idx) && (c == 0x08)) { // Backspace
					me->idx--;
					if (me->idx == 0) {
						in_history = 0;
						hist_index = -1;
					}
          BSP_cli_transmit((char*)&c, 1);
        }
      break;

    case ESC_STATE_ESC:
      if (c == '[') {
        esc_state = ESC_STATE_CSI;
      } else {
        esc_state = ESC_STATE_NONE;
      }
      break;

    case ESC_STATE_CSI:
      if (c == 'A') {           // Yukarı ok
        CLI_HistoryUp(me);
      } else if (c == 'B') {    // Aşağı ok
        CLI_HistoryDown(me);
      } else if (c == 'C') {    // Sağ ok
                                // İmleç sağ hareket
      } else if (c == 'D') {    // Sol ok
                                // İmleç sol hareket
      }
      esc_state = ESC_STATE_NONE;
      break;
    }
  } break;

  default: {
    status = Q_SUPER(&QHsm_top);
  } break;
  }

  return status;
}

static void SendPrompt(void) {
  BSP_cli_puts("\r\nM2Drive> ");

	#if 0
	Motor_MoveToPosition(10000);
	
    /* Şu anki CMR değerlerini oku */
    uint32_t cmr0 = EPWM_GET_CMR(EPWM1, 0); /* AH */
    uint32_t cmr2 = EPWM_GET_CMR(EPWM1, 2); /* BH */
    uint32_t cnr  = EPWM_GET_CNR(EPWM1, 0);

    CDC_SendFmt(
        "PWM,CNR:%lu,CMR0:%lu,CMR2:%lu,"
        "DUTY_A:%.1f%%,DUTY_B:%.1f%%\r\n",
        (unsigned long)cnr,
        (unsigned long)cmr0,
        (unsigned long)cmr2,
        100.0f * cmr0 / cnr,
        100.0f * cmr2 / cnr);
	#endif

}

static void ClearLine(void) {
  BSP_cli_puts("\x1B[2K\rM2Drive> ");
}

void CLI_ProcessCommand(char* cmd) {
  char* token;
  char* argv[5];
  int argc = 0;
	char buf[64];

  token = strtok(cmd, " ");

  while (token != NULL && argc < 5) {
    argv[argc++] = token;
    token        = strtok(NULL, " ");
  }

  if (argc == 0)
    return;

  if (!strcmp(argv[0], "help")) {
    BSP_cli_puts("\r\nCommand:\r\nmove stop status reset\r\n");
  } else if (!strcmp(argv[0], "move")) {
    int duty   = atol(argv[1]);
		
		if (duty != 0) {
			sprintf(buf, "\r\nMoving: %s @ %d\r\n", (duty < 0) ? "negative" : "positive", abs(duty));
			BSP_cli_puts(buf);
		}
		//BSP_AXIS_Z_set_duty(duty);
		
		Motor_MoveToPosition(duty);
		
  } else if (!strcmp(argv[0], "reset")) {
		BSP_AXIS_z_reset_pos();
		sprintf(buf, "\r\nReset z pos\r\n");
		BSP_cli_puts(buf);
  } 
	
}

static inline int hist_phys_index(int logical) {
  return logical % CLI_HISTORY_MAX;
}

void CLI_SaveHistory(const char* cmd) {
  if (strlen(cmd) == 0)
    return;
  strncpy(history[hist_count % CLI_HISTORY_MAX], cmd, CLI_BUF_SIZE);
  hist_count++;
}

void CLI_HistoryUp(Cli* const me) {
  if (hist_count == 0)
    return;

  if (!in_history) {
    hist_index = hist_count - 1;    // en son komut
    in_history = 1;
  } else {
    int oldest = (hist_count > CLI_HISTORY_MAX) ? hist_count - CLI_HISTORY_MAX : 0;
    if (hist_index > oldest)
      hist_index--;
  }

  int idx = hist_phys_index(hist_index);
  strncpy(me->buf, history[idx], CLI_BUF_SIZE);
  me->idx = strlen(me->buf);
  ClearLine();
  BSP_cli_transmit((char*)me->buf, me->idx);
}

void CLI_HistoryDown(Cli* const me) {
  if (!in_history)
    return;

  int newest = hist_count - 1;
  if (hist_index < newest)
    hist_index++;
  else {
    in_history = 0;
    me->buf[0] = '\0';
    me->idx    = 0;
    ClearLine();
    return;
  }

  int idx = hist_phys_index(hist_index);
  strncpy(me->buf, history[idx], CLI_BUF_SIZE);
  me->idx = strlen(me->buf);
  ClearLine();
  BSP_cli_transmit((char*)me->buf, me->idx);
}
