#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#include "pinnames.h"

#define GPIO_INPUT_PIN(pin_, pull_) {.pin = pin_, .mode = HAL_GPIO_MODE_IN, .pull = pull_}
#define GPIO_INPUT_PIN_PU(pin_) GPIO_INPUT_PIN(pin_, HAL_GPIO_PULL_UP)
#define GPIO_INPUT_PIN_PD(pin_) GPIO_INPUT_PIN(pin_, HAL_GPIO_PULL_DOWN)
#define GPIO_INPUT_PIN_PN(pin_) GPIO_INPUT_PIN(pin_, HAL_GPIO_PULL_NONE)

#define GPIO_OUTPUT_PIN(pin_, outtype_, initial_) \
  {.pin = pin_, .mode = HAL_GPIO_MODE_OUT, .outtype = outtype_, .initial = initial_}

#define GPIO_OUTPUT_PIN_PP(pin_) GPIO_OUTPUT_PIN(pin_, HAL_GPIO_OTYPE_PP, 0)
#define GPIO_OUTPUT_PIN_OD(pin_) GPIO_OUTPUT_PIN(pin_, HAL_GPIO_OTYPE_OD, 0)
#define GPIO_OUTPUT_PIN_PP_INITIAL(pin_, initial_) GPIO_OUTPUT_PIN(pin_, HAL_GPIO_OTYPE_PP, initial_)

void bsp_hal_gpio_init(gpio_t* const gpio, const uint8_t initial_pin_state);
void bsp_hal_gpio_init_initial(gpio_t* const gpio);

void bsp_hal_uart_init(uart_t* const uart);
void bsp_hal_uart_enable_rx_int(uart_t* const uart, const uint8_t priority);
void bsp_hal_uart_set_rx_irqhandler(UartDataReceivedIRQhandler irqhandler);
void bsp_hal_uart_transmit(uart_t* const uart, uint8_t* const data, const uint8_t length);

void bsp_hal_timer_init(timer_t* const timer);
void bsp_hal_timer_enable_int(timer_t* const timer, uint8_t priority);
void bsp_hal_timer_set_update_irqhandler(TimerUpdateIRQHandler irqhandler);
void bsp_hal_timer_set_channel_irqhandler(TimerChannelIRQHandler irqhandler);

void bsp_hal_pwm_out_pin_init(pwm_out_t* const pwm_out);
void bsp_hal_pwm_start(pwm_out_t* const pwm_out);
void bsp_hal_pwm_out_pin_set_val(const pwm_out_t* const pwm_out, uint32_t val);
void bsp_hal_pwm_out_pin_set_duty(const pwm_out_t* const pwm_out, uint16_t duty);
void bsp_hal_pwm_out_pin_set_duty_100(const pwm_out_t* const pwm_out, uint16_t duty);
void bsp_hal_pwm_set_frequency(pwm_out_t* const pwm_out, uint32_t frequency, uint32_t duty);

void bsp_hal_pwm_in_pin_init(pwm_in_t* const pwm_in);
uint32_t bsp_hal_pwm_in_capture_value_read(pwm_in_t* const pwm_in);
void bsp_hal_pwm_in_start(pwm_in_t* const pwm_in);
void bsp_hal_pwm_in_stop(pwm_in_t* const pwm_in);

void EE_Format(void);
void EE_Erase(uint32_t PageBaseAddress);
uint8_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Read_data);
uint8_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
uint8_t EE_ReadVariable32(uint16_t VirtAddressStart, uint32_t* ReadData);
uint8_t EE_WriteVariable32(uint16_t VirtAddressStart, uint32_t Data);

#if defined(GD32F310) || defined(GD32E23x)
#if defined(GD32F310)
#include "gd32f3x0.h"
#define bsp_hal_nvic_irq_enable(nvic_irq, nvic_irq_priority, nvic_irq_sub_priority) \
  nvic_irq_enable(nvic_irq, nvic_irq_priority, nvic_irq_sub_priority)
#define FLASH_SIZE ((uint32_t)0x4000U)
#define PAGE_SIZE ((uint32_t)0x400U)
#elif defined(GD32E23x)
#include "gd32e23x.h"
#define bsp_hal_nvic_irq_enable(nvic_irq, nvic_irq_priority, nvic_irq_sub_priority) \
  nvic_irq_enable(nvic_irq, nvic_irq_priority)
#endif

#if defined(GD32E230K8)
#define FLASH_SIZE ((uint32_t)0x10000U)
#define PAGE_SIZE ((uint32_t)0x400U)
#endif

#if defined(GD32E230F6)
#define FLASH_SIZE ((uint32_t)0x8000U)
#define PAGE_SIZE ((uint32_t)0x400U)
#endif

//* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS FLASH_BASE + FLASH_SIZE - ((uint32_t)(2 * PAGE_SIZE))

#define bsp_hal_gpio_write(gpio, value) gpio_bit_write(((gpio_t*)gpio)->addr, ((gpio_t*)gpio)->mask, value)
#define bsp_hal_gpio_read(gpio) gpio_input_bit_get(((gpio_t*)gpio)->addr, ((gpio_t*)gpio)->mask)
#define bsp_hal_gpio_toggle(gpio) gpio_bit_toggle(((gpio_t*)gpio)->addr, ((gpio_t*)gpio)->mask)
#define bsp_hal_uart_start(uart) usart_enable(((uart_t*)uart)->addr)
#define bsp_hal_timer_start(timer) timer_enable(((timer_t*)timer)->addr)
#define bsp_hal_timer_stop(timer) timer_disable(((timer_t*)timer)->addr)
static inline void bsp_hal_timer_counter_value_write(timer_t* const timer, uint32_t val) {
  timer_counter_value_config(timer->addr, val);
}

static inline void bsp_hal_timer_autoreload_value_write(timer_t* const timer, uint32_t val) {
  timer_autoreload_value_config(timer->addr, val);
}
#endif

#if defined(M251) || defined(M031) || defined(M480)
#include "NuMicro.h"

#if defined(M031)
	#define GPIO_QUASI_PIN(pin_) {.pin = pin_, .mode = GPIO_MODE_QUASI, .pull = 0}
#endif

#if defined(M251)
#define FLASH_SIZE FMC_APROM_END
#define PAGE_SIZE FMC_FLASH_PAGE_SIZE

//* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS FMC_APROM_BASE + FLASH_SIZE - ((uint32_t)(2 * PAGE_SIZE))

#endif

#define PWM0_INDEX 0X01
#define PWM1_INDEX 0X02

#define bsp_gpio_set_multi_function(pin, function)                                  \
  {                                                                                 \
    uint32_t* reg = (uint32_t*)SYS + 12 + (PORT_GET(pin) * 2) + (PIN_GET(pin) / 8); \
    *reg          = (*reg & ~(0xful << ((PIN_GET(pin) % 8) * 4))) | function;       \
  }

#define bsp_hal_nvic_irq_enable(nvic_irq, nvic_irq_priority, nvic_irq_sub_priority) \
  nvic_irq_enable(nvic_irq, nvic_irq_priority)

#define bsp_hal_gpio_write(gpio, value) \
  GPIO_PIN_DATA(PORT_GET(((gpio_t*)gpio)->pin), PIN_GET(((gpio_t*)gpio)->pin)) = value
#define bsp_hal_gpio_read(gpio) GPIO_PIN_DATA(PORT_GET(((gpio_t*)gpio)->pin), PIN_GET(((gpio_t*)gpio)->pin))
#define bsp_hal_gpio_toggle(gpio)                                                \
  GPIO_PIN_DATA(PORT_GET(((gpio_t*)gpio)->pin), PIN_GET(((gpio_t*)gpio)->pin)) = \
      ~GPIO_PIN_DATA(PORT_GET(((gpio_t*)gpio)->pin), PIN_GET(((gpio_t*)gpio)->pin))

void bsp_hal_pwm_clk_init(int pwm_module_index);

void bsp_hal_pwm_in_dma_init(pwm_in_t* const pwm_in, int* counter_value);

#endif

#if defined(STM32F)

#if defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F0)
#include "stm32f0xx.h"
#endif

#define bsp_hal_gpio_write(gpio, value) \
  HAL_GPIO_WritePin((GPIO_TypeDef*)(((gpio_t*)gpio)->addr), ((gpio_t*)gpio)->mask, value)
#define bsp_hal_gpio_read(gpio) HAL_GPIO_ReadPin((GPIO_TypeDef*)(((gpio_t*)gpio)->addr), ((gpio_t*)gpio)->mask)
#define bsp_hal_gpio_toggle(gpio) HAL_GPIO_TogglePin((GPIO_TypeDef*)(((gpio_t*)gpio)->addr), ((gpio_t*)gpio)->mask)

#endif

#if defined(__cplusplus)
}
#endif
