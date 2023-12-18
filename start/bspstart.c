#include <bsp.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>

uint32_t stm32l4_systick_frequency(void) { return SystemCoreClock; }

void bsp_start(void) {
  // init_main_osc();

  // stm32f4_gpio_set_config_array(&stm32f4_start_config_gpio[0]);

  // bsp_interrupt_initialize();
}
