#pragma once
/* this is an EXTREMELY CRUDE and temporary uart driver
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "stm32l4r9xx.h"

// WARNING: Substitute with bsp system headers
#define FREQ 4000000 // CPU frequency
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--)
    asm("nop");
}

#define GPIO(bank) ((GPIO_TypeDef *)(GPIOA_BASE + 0x400U * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin)); // GPIO bank
  int n = PINNO(pin);                      // Pin number
  RCC->AHB1ENR |= BIT(PINBANK(pin));       // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin)); // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t)af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

#ifndef UART_DEBUG
#define UART_DEBUG USART2
#endif

static inline void uart_init(USART_TypeDef *uart, unsigned long baud) {
  uint8_t af = 7;          // Alternate function
  uint16_t rx = 0, tx = 0; // pins

  // enable clock for GPIOD YES! ottimo lavoro, serviva
  RCC->AHB2ENR |= BIT(3);

  // Tochange
  if (uart == UART1)
    RCC->APB2ENR |= BIT(14);
  if (uart == UART2)
    RCC->APB1ENR1 |= BIT(17);
  if (uart == UART3)
    RCC->APB1ENR1 |= BIT(18);

  // Tochange
  // if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
  if (uart == UART2)
    tx = PIN('D', 5), rx = PIN('D', 6);
  // if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);

  // Tochek
  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;           // Disable this UART
  uart->BRR = FREQ / baud; // FREQ is a UART bus frequency
  // uart->CR2 |= BIT(15);                  // sets tx/rx switch
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3); // Set UE, RE, TE
}

static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0)
    spin(1);
}

static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0)
    uart_write_byte(uart, *(uint8_t *)buf++);
}

static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->ISR & BIT(5); // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t)(uart->RDR & 255);
}
static inline uint8_t timer_expired(volatile uint32_t *threshold, uint32_t prd,
                                    uint32_t now) {
  if (now + prd < *threshold)
    *threshold = 0; // Time wrapped? Reset timer
  if (*threshold == 0)
    *threshold = now + prd; // Firt poll? Set expiration
  if (*threshold > now)
    return 0; // Not expired yet, return
  *threshold = (now - *threshold) > prd
                   ? now + prd
                   : *threshold + prd; // Next expiration time
  return 1;                            // Expired, return true
}
