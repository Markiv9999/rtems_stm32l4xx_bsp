/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2020 embedded brains GmbH & Co. KG
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <bsp.h>
#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include <console.h> //XXX: custom
#include <stm32l4/hal.h>

const stm32l4_uart_config stm32l4_usart2_config = {
    .gpio = {.regs = 0,           // XXX: placed null
             .config = {.Pin = 0, // XXX: placed null
                        .Mode = GPIO_MODE_AF_PP,
                        .Pull = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_LOW,
                        .Alternate = GPIO_AF7_USART2}},
    .irq = USART2_IRQn,
    .device_index = 1};

stm32l4_uart_context stm32l4_usart2_instance = {
    .uart = {.Instance = USART2,
             .Init.BaudRate = BSP_CONSOLE_BAUD,
             .Init.WordLength = UART_WORDLENGTH_8B,
             .Init.StopBits = UART_STOPBITS_1,
             .Init.Parity = UART_PARITY_NONE,
             .Init.Mode = UART_MODE_TX_RX,
             .Init.HwFlowCtl = UART_HWCONTROL_NONE,
             .Init.OverSampling = UART_OVERSAMPLING_16,
             .Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE,
             .Init.ClockPrescaler = UART_PRESCALER_DIV1,
             .AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT},
    .config = &stm32l4_usart2_config};

static void stm32l4_output_char(char c) {
  stm32l4_uart_polled_write(&STM32L4_PRINTK_INSTANCE.device, c);
  // interface in ./stm32l4/hal
}

static void stm32l4_output_char_init(void) {
  UART_HandleTypeDef *uart;

  uart = &STM32L4_PRINTK_INSTANCE.uart;
  (void)HAL_UART_Init(uart);
  (void)HAL_UARTEx_SetTxFifoThreshold(uart, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(uart, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_EnableFifoMode(uart);

  BSP_output_char = stm32l4_output_char;
}

static void stm32l4_output_char_init_early(char c) {
  stm32l4_output_char_init();
  stm32l4_output_char(c);
}

static int stm32l4_poll_char(void) {
  return stm32l4_uart_polled_read(&STM32L4_PRINTK_INSTANCE.device); // interface
}

BSP_output_char_function_type BSP_output_char = stm32l4_output_char_init_early;

BSP_polling_getchar_function_type BSP_poll_char = stm32l4_poll_char;

RTEMS_SYSINIT_ITEM(stm32l4_output_char_init, RTEMS_SYSINIT_BSP_START,
                   RTEMS_SYSINIT_ORDER_LAST_BUT_5);
