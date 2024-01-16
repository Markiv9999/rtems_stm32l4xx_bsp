#include "stm32l4r9_module_clk_config.h"

void enable_debug_clock(void) {
  /* enable debug clock output */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // enable AHB2 clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  RCC->CFGR |= 0b0101 << RCC_CFGR_MCOSEL_Pos; // PLL
  // RCC->CFGR |= 0b0010 << RCC_CFGR_MCOSEL_Pos; // MSI

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8_Msk;
  GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED8_Pos;

  // configure clock out gpio MCO
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
  GPIOA->AFR[1] |= 0b00 << GPIO_AFRH_AFSEL8_Msk;

  GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
  GPIOA->MODER |= 0b10 << GPIO_MODER_MODE8_Pos;
}
