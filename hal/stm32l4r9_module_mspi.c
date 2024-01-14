#include "stm32l4r9_module_mspi.h"
#include "stm32l4r9xx.h"

void mspi_dma_push_init(void); // TODO: move to headers
void mspi_dma_pull_init(void); // TODO: move to headers

extern char dma_push_buffer[];
extern char dma_pull_buffer[];

/**
 * --------------------------------------------------------------------------- *
 *       INTERFACE SPECIFIC METHODS
 * --------------------------------------------------------------------------- *
 */

int mspi_init(struct mspi_interface device) {
  /* this procedure satisfy the following functions:
   * - enabling of clock for peripheral and relative bus
   * - configuring GPIO as required
   *   the function MUST be configured in accordance with the system
   * configuration
   */

  /* Enables the clock for the gpios, sets to alternate function and configures
   * the alternate function */
  mspi_gpio_cfg(device);
  /* Enables the multi-spi clock, enables the multispi-dma */
  mspi_sys_cfg(device);
  /* Configure device specific mspi settings, like clock prescaler, device size
   */
  mspi_dev_cfg(device);

  /* PENDING
   * Currently no checks are performed to assure the successful
   * configuration of the system. This is considered a pending feauture
   */
  return EXIT_UNDEFINED;
}

void mspi_gpio_cfg(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    // TODO: configure also for the second quadspi memory

    // enable clock for the gpio banks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

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
  if (device.interface_select == 0x02) {
    // enable clock for the gpio banks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
    GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL0_Pos;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL1_Pos;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL2_Pos;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL3_Pos;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL4_Pos;
    GPIOG->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL0_Pos;
    GPIOG->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL1_Pos;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL9_Pos;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL10_Pos;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL12_Pos;

    GPIOF->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOF->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOF->MODER &= ~GPIO_MODER_MODE2_Msk;
    GPIOF->MODER &= ~GPIO_MODER_MODE3_Msk;
    GPIOF->MODER &= ~GPIO_MODER_MODE4_Msk;
    GPIOG->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOG->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOG->MODER &= ~GPIO_MODER_MODE9_Msk;
    GPIOG->MODER &= ~GPIO_MODER_MODE10_Msk;
    GPIOG->MODER &= ~GPIO_MODER_MODE12_Msk;
    GPIOB->MODER |= 0b10 << GPIO_MODER_MODE10_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE2_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE3_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE4_Pos;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE9_Pos;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE10_Pos;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE12_Pos;

    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9_Msk;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10_Msk;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_Msk;
    GPIOB->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED10_Pos;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED4_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED9_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED10_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED12_Pos;

    // gpio to pushpull
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT2_Msk;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT3_Msk;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT9_Msk;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT10_Msk;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT12_Msk;

    // no pull up/down
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD12_Msk;
  }
}

void mspi_sys_cfg(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    /* Enable the Octa-SPI interface clock */
    RCC->AHB3ENR |= RCC_AHB3ENR_OSPI1EN;
    /* Reset Octa-SPI peripheral */
    RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI1RST);  /* Reset */
    RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI1RST); /* Release reset */
    /* Enable Octa-SPI DMA */
    OCTOSPI1->CR |= OCTOSPI_CR_DMAEN;
  }
  if (device.interface_select == 0x02) {
    /* Enable the Octa-SPI interface clock */
    RCC->AHB3ENR |= RCC_AHB3ENR_OSPI2EN;
    /* Reset Octa-SPI peripheral */
    RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI2RST);  /* Reset */
    RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI2RST); /* Release reset */
    /* Enable Octa-SPI DMA */
    OCTOSPI2->CR |= OCTOSPI_CR_DMAEN;
  }
}

// initialization method
void mspi_dev_cfg(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    OCTOSPI1->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos;
    OCTOSPI1->DCR2 |= 16 << OCTOSPI_DCR2_PRESCALER_Pos;
  }
  if (device.interface_select == 0x02) {
    OCTOSPI2->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos;
    OCTOSPI2->DCR2 |= 16 << OCTOSPI_DCR2_PRESCALER_Pos;
  }
}

void mspi_interface_cleanup(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
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

  if (device.interface_select == 0x02) {
    // cleanup of functional registers
    OCTOSPI2->CR &= ~(OCTOSPI_CR_EN);
    OCTOSPI2->CCR &=
        ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_IMODE | OCTOSPI_CCR_ADMODE |
          OCTOSPI_CCR_DMODE | OCTOSPI_CCR_ADSIZE | OCTOSPI_IR_INSTRUCTION);

    OCTOSPI2->DLR &= ~(OCTOSPI_DLR_DL);
    OCTOSPI2->AR &= ~(OCTOSPI_AR_ADDRESS_Msk);
    OCTOSPI2->DR &= ~(OCTOSPI_DR_DATA_Msk);

    // disable mspi push pull dma channels
    DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
    DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

    // clear flags
    // CTOF seems is not defined in the cmsis header
    OCTOSPI2->FCR |= (OCTOSPI_FCR_CSMF);
    OCTOSPI2->FCR |= (OCTOSPI_FCR_CTCF);
    OCTOSPI2->FCR |= (OCTOSPI_FCR_CTEF);
  }
}

u16 mspi_interface_wait_busy(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) {
    };
    // PENDING add timeout
    return EXIT_SUCCESS;
  }
  if (device.interface_select == 0x02) {
    while (OCTOSPI2->SR & OCTOSPI_SR_BUSY) {
    };
    // PENDING add timeout
    return EXIT_SUCCESS;
  }
}

// TODO (add interface specifier f.e. OCTOSPI1/OCTOSPI2)
u16 mspi_transfer_dma(struct mspi_interface device,
                      struct mspi_cmd (*device_fun_handler)(void *),
                      void *argument) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);

  // PENDING save interface registers for check
  mspi_interface_cleanup(device);

  if (device.interface_select == 0x01) {
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
    if (cmd.data_mode && cmd.req_dma) {
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

    // if read functional mode enable the dma push channel before sending the
    // address
    if (cmd.data_mode) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        if (cmd.req_dma) {
          // enable dma channel
          DMA1_Channel1->CCR |= DMA_CCR_EN;
        } else {
          // manually set data register content
          OCTOSPI1->DR |= (cmd.data_cmd);
        }
      }
    }

    // wait for the transaction to complete (+ timeout and abort)
    if (mspi_interface_wait_busy(device)) {
      OCTOSPI1->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
      return ERROR_MSPI_INTERFACE_STUCK;
    }

    if (cmd.data_mode && cmd.req_dma) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        // disable dma channel
        DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
      }
      if (cmd.fun_mode == 0b01) { // read  -- dma push
        // disable dma channel
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
      }
    }

    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);

    return EXIT_SUCCESS;
  }
  if (device.interface_select == 0x02) {
    // set functional mode
    OCTOSPI2->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
    // set *-MODES (number of transfer lines)
    OCTOSPI2->CCR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                      (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                      (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));
    // set length of *-PHASES (if in use)
    if (cmd.addr_mode > 0) {
      OCTOSPI2->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
    }
    if (cmd.data_mode > 0) {
      OCTOSPI2->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
    }
    // enable the peripheral
    OCTOSPI2->CR |= (OCTOSPI_CR_EN);
    // set intruction, address (if in use)
    OCTOSPI2->IR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);
    // if read functional mode enable the dma pull channel before sending the
    // address
    if (cmd.data_mode && cmd.req_dma) {
      if (cmd.fun_mode == 0b01) { // read -- dma pull
        // enable dma channel
        DMA1_Channel2->CCR |= DMA_CCR_EN;
      }
    }
    // Set the address if needed by the command
    if (cmd.addr_mode > 0) {
      OCTOSPI2->AR |=
          (cmd.addr_cmd
           << OCTOSPI_AR_ADDRESS_Pos); // for now even plane, beginning of page
    }

    // if read functional mode enable the dma push channel before sending the
    // address
    if (cmd.data_mode) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        if (cmd.req_dma) {
          // enable dma channel
          DMA1_Channel1->CCR |= DMA_CCR_EN;
        } else {
          // manually set data register content
          OCTOSPI2->DR |= (cmd.data_cmd);
        }
      }
    }

    // wait for the transaction to complete (+ timeout and abort)
    if (mspi_interface_wait_busy(device)) {
      OCTOSPI2->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
      return ERROR_MSPI_INTERFACE_STUCK;
    }

    if (cmd.data_mode && cmd.req_dma) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        // disable dma channel
        DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
      }
      if (cmd.fun_mode == 0b01) { // read  -- dma push
        // disable dma channel
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
      }
    }

    OCTOSPI2->CR &= ~(OCTOSPI_CR_EN);

    return EXIT_SUCCESS;
  }
}

u16 mspi_autopoll_wait(struct mspi_interface device,
                       struct mspi_cmd (*device_fun_handler)(void *),
                       void *argument, u32 mask, u32 match) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);

  if (device.interface_select == 0x01) {
    // Set the 'mask', 'match', and 'polling interval' values.
    OCTOSPI1->PSMKR = mask;
    OCTOSPI1->PSMAR = match;
    OCTOSPI1->PIR = 0x10;

    // PENDING save interface registers for check
    mspi_interface_cleanup(device);

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
    if (cmd.data_mode && cmd.req_dma) {
      if (cmd.fun_mode == 0b01) { // read -- dma pull
        // enable dma channel
      }
    }
    // Set the address if needed by the command
    if (cmd.addr_mode > 0) {
      OCTOSPI1->AR |= (cmd.addr_cmd << OCTOSPI_AR_ADDRESS_Pos);
    }

    // if read functional mode enable the dma push channel before sending the
    // address
    if (cmd.data_mode) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        if (cmd.req_dma) {
          // enable dma channel
          DMA1_Channel1->CCR |= DMA_CCR_EN;
        } else {
          // manually set data register content
          OCTOSPI1->DR |= (cmd.data_cmd);
        }
      }
    }

    // wait for the transaction to complete (+ timeout and abort)
    if (mspi_interface_wait_busy(device)) {
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
  if (device.interface_select == 0x02) {
    // Set the 'mask', 'match', and 'polling interval' values.
    OCTOSPI1->PSMKR = mask;
    OCTOSPI1->PSMAR = match;
    OCTOSPI1->PIR = 0x10;

    // PENDING save interface registers for check
    mspi_interface_cleanup(device);

    // set functional mode
    OCTOSPI2->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
    // set *-MODES (number of transfer lines)
    OCTOSPI2->IR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                     (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                     (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));

    // set length of *-PHASES (if in use)
    if (cmd.addr_mode > 0) {
      OCTOSPI2->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
    }
    if (cmd.data_mode > 0) {
      OCTOSPI2->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
    }
    // enable the peripheral
    OCTOSPI2->CR |= (OCTOSPI_CR_EN);
    // set intruction, address (if in use)
    OCTOSPI2->CCR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);
    // if read functional mode enable the dma pull channel before sending the
    // address
    if (cmd.data_mode && cmd.req_dma) {
      if (cmd.fun_mode == 0b01) { // read -- dma pull
        // enable dma channel
      }
    }
    // Set the address if needed by the command
    if (cmd.addr_mode > 0) {
      OCTOSPI2->AR |= (cmd.addr_cmd << OCTOSPI_AR_ADDRESS_Pos);
    }

    // if read functional mode enable the dma push channel before sending the
    // address
    if (cmd.data_mode) {
      if (cmd.fun_mode == 0b00) { // write -- dma push
        if (cmd.req_dma) {
          // enable dma channel
          DMA1_Channel1->CCR |= DMA_CCR_EN;
        } else {
          // manually set data register content
          OCTOSPI2->DR |= (cmd.data_cmd);
        }
      }
    }

    // wait for the transaction to complete (+ timeout and abort)
    if (mspi_interface_wait_busy(device)) {
      OCTOSPI2->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
      return ERROR_MSPI_INTERFACE_STUCK;
    }

    // Wait for a match.
    while (OCTOSPI2->SR & OCTOSPI_SR_BUSY) {
    };
    // Acknowledge the 'status match flag.'
    OCTOSPI2->FCR |= (OCTOSPI_FCR_CSMF);
    // Un-set the data mode and disable auto-polling.
    OCTOSPI2->CCR &= ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_DMODE);
    // Disable the peripheral.
    OCTOSPI2->CR &= ~(OCTOSPI_CR_EN);
    return EXIT_SUCCESS;
  }
}

#define OCTOSPI_DR_OFF 0x050
/*  TODO: protect the dma channels with a define guard */
void mspi_dma_push_init(void) {
  /* Set up DMA1_CH1 "Push" dma channel */

  /* Enable the dma controller 1 peripheral */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Clear interrupt status flags */
  DMA1->IFCR |= DMA_IFCR_CGIF1;

  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel1->CPAR = OCTOSPI2_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel1->CMAR = (uint32_t)&dma_push_buffer;
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel1->CNDTR = 2048 >> 2; // HACK: FIXME (page size)
  /* Configure the CCR register */
  DMA1_Channel1->CCR |=
      /* channel priority */
      0b00 << DMA_CCR_PL_Pos |
      /* channel direction (read from memory) */
      DMA_CCR_DIR |
      /* memory size (32bit)*/
      0b11 << DMA_CCR_MSIZE_Pos |
      /* peripheral size (32bit)*/
      0b11 << DMA_CCR_PSIZE_Pos |
      /* memory increment mode */
      DMA_CCR_MINC;
}

void mspi_dma_pull_init(void) {
  /* Set up DMA1_CH2 "Pull" dma channel */

  /* Enable the dma controller 1 peripheral */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Clear interrupt status flags */
  DMA1->IFCR |= DMA_IFCR_CGIF2;

  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel2->CPAR = OCTOSPI2_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel2->CMAR = (uint32_t)&dma_pull_buffer;
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel2->CNDTR = 2048 >> 2; // HACK: FIXME (page size)
  /* Configure the CCR register */
  DMA1_Channel2->CCR |=
      /* channel priority */
      0b00 << DMA_CCR_PL_Pos |
      /* memory size (32bit)*/
      0b11 << DMA_CCR_MSIZE_Pos |
      /* peripheral size (32bit)*/
      0b11 << DMA_CCR_PSIZE_Pos |
      /* memory increment mode */
      DMA_CCR_MINC;

  /*Configure CCR register:channel direction (read from peripheral) */
  DMA1_Channel2->CCR &= ~(DMA_CCR_DIR);
}
