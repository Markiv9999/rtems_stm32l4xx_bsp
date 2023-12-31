#include "stm32l4r9_module_mspi.h"
#include "ext_error_codes.h"
#include "ext_typedefs.h"
#include "stm32l476xx.h"

#include <stdlib.h>

/**
 * --------------------------------------------------------------------------- *
 *       INTERFACE SPECIFIC METHODS
 * --------------------------------------------------------------------------- *
 */

int mspi_init(void) {
  /* this procedure satisfy the following functions:
   * - enabling of clock for peripheral and relative bus
   * - configuring GPIO as required
   *   the function MUST be configured in accordance with the system
   * configuration
   */

  /* Enables the clock for the gpios, sets to alternate function and configures
   * the alternate function */
  mspi_gpio_cfg();
  /* Enables the multi-spi clock, enables the multispi-dma */
  mspi_sys_cfg();
  /* Configure device specific mspi settings, like clock prescaler, device size
   */
  mspi_dev_cfg();

  /* PENDING
   * Currently no checks are performed to assure the successful
   * configuration of the system. This is considered a pending feauture
   */
  return EXIT_UNDEFINED;
}
void mspi_gpio_cfg(void) {
  // configure alternate function for CLK and CS
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
  GPIOB->AFR[1] |= 0b1010 << GPIO_AFRH_AFSEL10_Pos;
  GPIOB->AFR[1] |= 0b1010 << GPIO_AFRH_AFSEL11_Pos;

  GPIOB->MODER &= ~GPIO_MODER_MODE10_Msk;
  GPIOB->MODER &= ~GPIO_MODER_MODE11_Msk;
  GPIOB->MODER |= 0b10 << GPIO_MODER_MODE10_Pos;
  GPIOB->MODER |= 0b10 << GPIO_MODER_MODE11_Pos;

  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10_Msk;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED11_Msk;
  GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED10_Pos;
  GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED11_Pos;

  // gpio to pushpull
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT10_Msk;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT11_Msk;
  // no pull up/down
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11_Msk;

  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
  GPIOA->AFR[0] |= 0b1010 << GPIO_AFRL_AFSEL6_Pos;
  GPIOA->AFR[0] |= 0b1010 << GPIO_AFRL_AFSEL7_Pos;

  GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
  GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
  GPIOB->AFR[0] |= 0b1010 << GPIO_AFRL_AFSEL0_Pos;
  GPIOB->AFR[0] |= 0b1010 << GPIO_AFRL_AFSEL1_Pos;

  GPIOA->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOA->MODER |= 0b10 << GPIO_MODER_MODE6_Pos;
  GPIOA->MODER |= 0b10 << GPIO_MODER_MODE7_Pos;
  GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
  GPIOB->MODER &= ~GPIO_MODER_MODE1_Msk;
  GPIOB->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
  GPIOB->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk;
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk;
  GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED6_Pos;
  GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED7_Pos;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
  GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED0_Pos;
  GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED1_Pos;

  // gpio to pushpull
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT6_Msk;
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT7_Msk;
  // no pull up/down
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk;
  // gpio to pushpull
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
  // no pull up/down
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
}

void mspi_sys_cfg(void) {
  /* Enable the Quad-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
  /* Reset Quad-SPI peripheral */
  RCC->AHB3RSTR |= (RCC_AHB3RSTR_QSPIRST);  /* Reset */
  RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_QSPIRST); /* Release reset */
  /* Enable Quad-SPI DMA */
  QUADSPI->CR |= QUADSPI_CR_DMAEN;
}

// initialization method
void mspi_dev_cfg(void) {
  QUADSPI->DCR |= 27 << QUADSPI_DCR_FSIZE_Pos;
  QUADSPI->CR |= 16 << QUADSPI_CR_PRESCALER_Pos;
}

void mspi_interface_cleanup(void) {
  // cleanup of functional registers
  QUADSPI->CR &= ~(QUADSPI_CR_EN);
  QUADSPI->CCR &=
      ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_IMODE | QUADSPI_CCR_ADMODE |
        QUADSPI_CCR_DMODE | QUADSPI_CCR_ADSIZE | QUADSPI_CCR_INSTRUCTION);

  QUADSPI->DLR &= ~(QUADSPI_DLR_DL);
  QUADSPI->AR &= ~(QUADSPI_AR_ADDRESS_Msk);
  QUADSPI->DR &= ~(QUADSPI_DR_DATA_Msk);
}

u16 mspi_interface_wait_busy(void) {
  while (QUADSPI->SR & QUADSPI_SR_BUSY) {
  };
  // PENDING add timeout
  return EXIT_SUCCESS;
}

// TODO (add interface specifier f.e. OCTOSPI1/OCTOSPI2)
u16 mspi_transfer_dma(struct mspi_cmd (*device_fun_handler)(void *),
                      struct mspi_interface interface, void *argument) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);

  // PENDING save interface registers for check
  mspi_interface_cleanup();
  // set functional mode
  QUADSPI->CCR |= ((cmd.fun_mode << QUADSPI_CCR_FMODE_Pos));
  // set *-MODES (number of transfer lines)
  QUADSPI->CCR |= ((cmd.instr_mode << QUADSPI_CCR_IMODE_Pos) |
                   (cmd.addr_mode << QUADSPI_CCR_ADMODE_Pos) |
                   (cmd.data_mode << QUADSPI_CCR_DMODE_Pos));
  // set length of *-PHASES (if in use)
  if (cmd.addr_mode > 0) {
    QUADSPI->CCR |= (cmd.addr_size << QUADSPI_CCR_ADSIZE_Pos);
  }
  if (cmd.data_mode > 0) {
    QUADSPI->DLR |= (cmd.data_size << QUADSPI_DLR_DL_Pos);
  }
  // enable the peripheral
  QUADSPI->CR |= (QUADSPI_CR_EN);
  // set intruction, address (if in use)
  QUADSPI->CCR |= (cmd.instr_cmd << QUADSPI_CCR_INSTRUCTION_Pos);
  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b01) { // read -- dma pull
      // enable dma channel
    }
  }
  // Set the address if needed by the command
  if (cmd.addr_mode > 0) {
    QUADSPI->AR |=
        (cmd.addr_cmd
         << QUADSPI_AR_ADDRESS_Pos); // for now even plane, beginning of page
  }

  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b00) { // write -- dma push
      // enable dma channel
    }
  }

  // wait for the transaction to complete (+ timeout and abort)
  if (mspi_interface_wait_busy()) {
    QUADSPI->CR &= ~(QUADSPI_CR_EN); // disable the interface in anay case
    return ERROR_MSPI_INTERFACE_STUCK;
  }

  QUADSPI->CR &= ~(QUADSPI_CR_EN);

  return EXIT_SUCCESS;
}

u16 mspi_autopoll_wait(struct mspi_cmd (*device_fun_handler)(void *),
                       struct mspi_interface interface, void *argument,
                       u32 mask, u32 match) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);

  // Set the 'mask', 'match', and 'polling interval' values.
  QUADSPI->PSMKR = mask;
  QUADSPI->PSMAR = match;
  QUADSPI->PIR = 0x10;

  // PENDING save interface registers for check
  mspi_interface_cleanup();

  // set functional mode
  QUADSPI->CCR |= ((cmd.fun_mode << QUADSPI_CCR_FMODE_Pos));
  // set *-MODES (number of transfer lines)
  QUADSPI->CCR |= ((cmd.instr_mode << QUADSPI_CCR_IMODE_Pos) |
                   (cmd.addr_mode << QUADSPI_CCR_ADMODE_Pos) |
                   (cmd.data_mode << QUADSPI_CCR_DMODE_Pos));

  // set length of *-PHASES (if in use)
  if (cmd.addr_mode > 0) {
    QUADSPI->CCR |= (cmd.addr_size << QUADSPI_CCR_ADSIZE_Pos);
  }
  if (cmd.data_mode > 0) {
    QUADSPI->DLR |= (cmd.data_size << QUADSPI_DLR_DL_Pos);
  }
  // enable the peripheral
  QUADSPI->CR |= (QUADSPI_CR_EN);
  // set intruction, address (if in use)
  QUADSPI->CCR |= (cmd.instr_cmd << QUADSPI_CCR_INSTRUCTION_Pos);
  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b01) { // read -- dma pull
      // enable dma channel
    }
  }
  // Set the address if needed by the command
  if (cmd.addr_mode > 0) {
    QUADSPI->AR |=
        (cmd.addr_cmd
         << QUADSPI_AR_ADDRESS_Pos); // for now even plane, beginning of page
  }

  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b00) { // write -- dma push
      // enable dma channel
    }
  }

  // wait for the transaction to complete (+ timeout and abort)
  if (mspi_interface_wait_busy()) {
    QUADSPI->CR &= ~(QUADSPI_CR_EN); // disable the interface in anay case
    return ERROR_MSPI_INTERFACE_STUCK;
  }

  // Wait for a match.
  while (QUADSPI->SR & QUADSPI_SR_BUSY) {
  };
  // Acknowledge the 'status match flag.'
  QUADSPI->FCR |= (QUADSPI_FCR_CSMF);
  // Un-set the data mode and disable auto-polling.
  QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE);
  // Disable the peripheral.
  QUADSPI->CR &= ~(QUADSPI_CR_EN);
  return EXIT_SUCCESS;
}
