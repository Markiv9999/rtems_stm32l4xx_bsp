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

#ifndef LIBBSP_ARM_STM32l4_STM32H7_HAL_H
#define LIBBSP_ARM_STM32l4_STM32H7_HAL_H

#include <stm32l4xx_hal.h>

#include <rtems/termiostypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  GPIO_TypeDef *regs;
  GPIO_InitTypeDef config;
} stm32l4_gpio_config;

typedef struct {
  stm32l4_gpio_config gpio;
  rtems_vector_number irq;
  uint8_t device_index;
} stm32l4_uart_config;

typedef struct {
  UART_HandleTypeDef uart;
  bool transmitting;
  rtems_termios_device_context device;
  const stm32l4_uart_config *config;
} stm32l4_uart_context;

static inline stm32l4_uart_context *
stm32l4_uart_get_context(rtems_termios_device_context *base) {
  return RTEMS_CONTAINER_OF(base, stm32l4_uart_context, device);
}

void stm32l4_uart_polled_write(rtems_termios_device_context *base, char c);

int stm32l4_uart_polled_read(rtems_termios_device_context *base);

#ifdef __cplusplus
}
#endif

#endif /* LIBBSP_ARM_STM32l4_STM32H7_HAL_H */
