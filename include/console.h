
#ifndef LIBBSP_ARM_STM32L4_BSP_CONSOLE
#define LIBBSP_ARM_STM32L4_BSP_CONSOLE
/* this file contains the options that are in the h7 bsp are generated from the
 * options in the build specifications */
#define BSP_CONSOLE_BAUD 9600
/* can you still use the usart for the application even if it is used by printk
 * and getchark */

#include <stm32l4/hal.h>
extern stm32l4_uart_context stm32l4_usart2_instance;
#define STM32L4_PRINTK_INSTANCE stm32l4_usart2_instance

#endif /* LIBBSP_ARM_STM32L4_BSP_CONSOLE */
