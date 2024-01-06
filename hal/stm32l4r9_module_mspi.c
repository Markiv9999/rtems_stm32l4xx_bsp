#include "stm32l4r9_module_mspi.h"
#include "stm32l4r9xx.h"

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
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI1EN;
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI2EN;
  /* Reset Quad-SPI peripheral */
  RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI1RST);  /* Reset */
  RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI2RST);  /* Reset */
  RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI1RST); /* Release reset */
  RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI2RST); /* Release reset */
  /* Enable Quad-SPI DMA */
  OCTOSPI1->CR |= OCTOSPI_CR_DMAEN;
}

// initialization method
void mspi_dev_cfg(void) {
  /* TODO: Check
   * check git diff with original
   */
  OCTOSPI1->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos;
  OCTOSPI1->DCR2 |= 16 << OCTOSPI_DCR2_PRESCALER_Pos;
}

void mspi_interface_cleanup(void) {
  // cleanup of functional registers
  OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);
  OCTOSPI1->CCR &=
      ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_IMODE | OCTOSPI_CCR_ADMODE |
        OCTOSPI_CCR_DMODE | OCTOSPI_CCR_ADSIZE | OCTOSPI_IR_INSTRUCTION);

  OCTOSPI1->DLR &= ~(OCTOSPI_DLR_DL);
  OCTOSPI1->AR &= ~(OCTOSPI_AR_ADDRESS_Msk);
  OCTOSPI1->DR &= ~(OCTOSPI_DR_DATA_Msk);

  // disable mspi push pull dma channels
  DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
  DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

  // clear flags
  // CTOF seems is not defined in the cmsis header
  OCTOSPI1->FCR |= (OCTOSPI_FCR_CSMF);
  OCTOSPI1->FCR |= (OCTOSPI_FCR_CTCF);
  OCTOSPI1->FCR |= (OCTOSPI_FCR_CTEF);
}

u16 mspi_interface_wait_busy(void) {
  while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {
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
  OCTOSPI1->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
  // set *-MODES (number of transfer lines)
  OCTOSPI1->CCR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                    (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                    (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));
  // set length of *-PHASES (if in use)
  if (cmd.addr_mode > 0) {
    OCTOSPI1->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
  }
  if (cmd.data_mode > 0) {
    OCTOSPI1->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
  }
  // enable the peripheral
  OCTOSPI1->CR |= (OCTOSPI_CR_EN);
  // set intruction, address (if in use)
  OCTOSPI1->IR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);
  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b01) { // read -- dma pull
      // enable dma channel
      DMA1_Channel2->CCR |= DMA_CCR_EN;
    }
  }
  // Set the address if needed by the command
  if (cmd.addr_mode > 0) {
    OCTOSPI1->AR |=
        (cmd.addr_cmd
         << OCTOSPI_AR_ADDRESS_Pos); // for now even plane, beginning of page
  }

  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b00) { // write -- dma push
      // enable dma channel
      DMA1_Channel1->CCR |= DMA_CCR_EN;
    }
  }

  // wait for the transaction to complete (+ timeout and abort)
  if (mspi_interface_wait_busy()) {
    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
    return ERROR_MSPI_INTERFACE_STUCK;
  }

  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b00) { // write -- dma push
      // enable dma channel
      DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
    }
    if (cmd.fun_mode == 0b01) { // read  -- dma push
      // enable dma channel
      DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
    }
  }

  OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);

  return EXIT_SUCCESS;
}

u16 mspi_autopoll_wait(struct mspi_cmd (*device_fun_handler)(void *),
                       struct mspi_interface interface, void *argument,
                       u32 mask, u32 match) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);

  // Set the 'mask', 'match', and 'polling interval' values.
  OCTOSPI1->PSMKR = mask;
  OCTOSPI1->PSMAR = match;
  OCTOSPI1->PIR = 0x10;

  // PENDING save interface registers for check
  mspi_interface_cleanup();

  // set functional mode
  OCTOSPI1->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
  // set *-MODES (number of transfer lines)
  OCTOSPI1->IR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                   (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                   (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));

  // set length of *-PHASES (if in use)
  if (cmd.addr_mode > 0) {
    OCTOSPI1->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
  }
  if (cmd.data_mode > 0) {
    OCTOSPI1->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
  }
  // enable the peripheral
  OCTOSPI1->CR |= (OCTOSPI_CR_EN);
  // set intruction, address (if in use)
  OCTOSPI1->CCR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);
  // if read functional mode enable the dma pull channel before sending the
  // address
  if (cmd.data_mode > 0) {
    if (cmd.fun_mode == 0b01) { // read -- dma pull
      // enable dma channel
    }
  }
  // Set the address if needed by the command
  if (cmd.addr_mode > 0) {
    OCTOSPI1->AR |=
        (cmd.addr_cmd
         << OCTOSPI_AR_ADDRESS_Pos); // for now even plane, beginning of page
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
    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
    return ERROR_MSPI_INTERFACE_STUCK;
  }

  // Wait for a match.
  while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {
  };
  // Acknowledge the 'status match flag.'
  OCTOSPI1->FCR |= (OCTOSPI_FCR_CSMF);
  // Un-set the data mode and disable auto-polling.
  OCTOSPI1->CCR &= ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_DMODE);
  // Disable the peripheral.
  OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);
  return EXIT_SUCCESS;
}
