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

#include <stm32l4/hal.h>

#include <bsp.h>
#include <bsp/fatal.h>
#include <bsp/irq.h>
#include <rtems/console.h>

#include <console.h> //XXX: custom
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>

extern stm32l4_uart_context stm32l4_usart2_instance;

static stm32l4_uart_context *const stm32l4_uart_instances[] = {
    &stm32l4_usart2_instance};

static bool stm32l4_uart_set_attributes(rtems_termios_device_context *base,
                                        const struct termios *term) {
  stm32l4_uart_context *ctx;
  uint32_t previous_baud;
  uint32_t previous_stop_bits;
  uint32_t previous_parity;
  uint32_t previous_mode;
  HAL_StatusTypeDef status;

  if ((term->c_cflag & CSIZE) != CS8) {
    return false;
  }

  ctx = stm32l4_uart_get_context(base);

  previous_baud = ctx->uart.Init.BaudRate;
  ctx->uart.Init.BaudRate = rtems_termios_baud_to_number(term->c_ospeed);

  previous_stop_bits = ctx->uart.Init.StopBits;
  if ((term->c_cflag & CSTOPB) != 0) {
    ctx->uart.Init.StopBits = UART_STOPBITS_2;
  } else {
    ctx->uart.Init.StopBits = UART_STOPBITS_1;
  }

  previous_parity = ctx->uart.Init.Parity;
  if ((term->c_cflag & PARENB) != 0) {
    if ((term->c_cflag & PARODD) != 0) {
      ctx->uart.Init.Parity = UART_PARITY_ODD;
    } else {
      ctx->uart.Init.Parity = UART_PARITY_EVEN;
    }
  } else {
    ctx->uart.Init.Parity = UART_PARITY_NONE;
  }

  previous_mode = ctx->uart.Init.Mode;
  if ((term->c_cflag & CREAD) != 0) {
    ctx->uart.Init.Mode = UART_MODE_TX_RX;
  } else {
    ctx->uart.Init.Mode = UART_MODE_RX;
  }

  status = UART_SetConfig(&ctx->uart);
  if (status != HAL_OK) {
    ctx->uart.Init.BaudRate = previous_baud;
    ctx->uart.Init.StopBits = previous_stop_bits;
    ctx->uart.Init.Parity = previous_parity;
    ctx->uart.Init.Mode = previous_mode;
    return false;
  }

  return true;
}

static bool stm32l4_uart_first_open(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    struct termios *term,
                                    rtems_libio_open_close_args_t *args) {
  stm32l4_uart_context *ctx;
  UART_HandleTypeDef *uart;
#ifdef BSP_CONSOLE_USE_INTERRUPTS
  rtems_status_code sc;
#endif

  ctx = stm32l4_uart_get_context(base);
  uart = &ctx->uart;

  rtems_termios_set_initial_baud(tty, BSP_CONSOLE_BAUD);

  (void)HAL_UART_Init(uart);
  (void)HAL_UARTEx_SetTxFifoThreshold(uart, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(uart, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_EnableFifoMode(uart);

  stm32l4_uart_set_attributes(base, term);

  return true;
}

static void stm32l4_uart_last_close(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    rtems_libio_open_close_args_t *args) {}

static void stm32l4_uart_write(rtems_termios_device_context *base,
                               const char *buf, size_t len) {
  size_t i;

  for (i = 0; i < len; ++i) {
    stm32l4_uart_polled_write(base, buf[i]);
  }
}

static const rtems_termios_device_handler stm32l4_uart_handler = {
    .first_open = stm32l4_uart_first_open,
    .last_close = stm32l4_uart_last_close,
    .write = stm32l4_uart_write,
    .set_attributes = stm32l4_uart_set_attributes,
    .poll_read = stm32l4_uart_polled_read,
    .mode = TERMIOS_POLLED};

rtems_status_code console_initialize(rtems_device_major_number major,
                                     rtems_device_minor_number minor,
                                     void *arg) {
  size_t i;

  rtems_termios_initialize();

  for (i = 0; i < RTEMS_ARRAY_SIZE(stm32l4_uart_instances); ++i) {
    stm32l4_uart_context *ctx;
    char path[sizeof("/dev/ttySXXX")];

    ctx = stm32l4_uart_instances[i];
    snprintf(path, sizeof(path), "/dev/ttyS%" PRIu8, ctx->config->device_index);

    rtems_termios_device_install(path, &stm32l4_uart_handler, NULL,
                                 &ctx->device);

    if (ctx == &STM32L4_PRINTK_INSTANCE) {
      link(path, CONSOLE_DEVICE_NAME);
    }
  }

  return RTEMS_SUCCESSFUL;
}
