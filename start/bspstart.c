#include <bsp.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>

extern uint32_t SystemCoreClock;
uint32_t stm32l4_systick_frequency(void) { return SystemCoreClock; }

void bsp_start(void) {
  // init_main_osc();
  bsp_interrupt_initialize();
}
