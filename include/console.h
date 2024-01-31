/* this file contains the options that are in the h7 bsp are generated from the
 * options in the build specifications */
#define BSP_CONSOLE_BAUD 9600
/* can you still use the usart for the application even if it is used by printk
 * and getchark */
#define STM32L4_PRINTK_INSTANCE stm32h7_usart2_instance
