#include "stm32l4r9_module_mspi.h"
#include "stm32l4r9xx.h"
#include <stdint.h>

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
  mspi_sys_cfg(device); // XXX: interface triggered once here(due to reset)

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

    /* OCTOSPIM_P1_CLK */
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOA->AFR[0] |= 0xA << GPIO_AFRL_AFSEL3_Pos;
    GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk;
    GPIOA->MODER |= 0b10 << GPIO_MODER_MODE3_Pos;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3_Msk;

    /* OCTOSPIM_P1_NCS */
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk; /* OCTOPIM_P1_NCS */
    GPIOA->AFR[0] |= 0xA << GPIO_AFRL_AFSEL2_Pos;
    GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk;
    GPIOA->MODER |= 0b10 << GPIO_MODER_MODE2_Pos;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT2_Msk;
    GPIOA->PUPDR |= 0b01 << GPIO_PUPDR_PUPD2_Pos; // you need a pullup

    /* OCTOSPIM_P1_IO0 */
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOB->AFR[0] |= 0xA << GPIO_AFRL_AFSEL1_Pos;
    GPIOB->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOB->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

    /* OCTOSPIM_P1_IO1 */
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOB->AFR[0] |= 0xA << GPIO_AFRL_AFSEL0_Pos;
    GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOB->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
    GPIOB->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;

    /* OCTOSPIM_P1_IO2 */
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
    GPIOA->AFR[0] |= 0xA << GPIO_AFRL_AFSEL7_Pos;
    GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;
    GPIOA->MODER |= 0b10 << GPIO_MODER_MODE7_Pos;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk;
    GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED7_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT7_Msk;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk;

    /* OCTOSPIM_P1_IO3 */
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
    GPIOA->AFR[0] |= 0xA << GPIO_AFRL_AFSEL6_Pos;
    GPIOA->MODER &= ~GPIO_MODER_MODE6_Msk;
    GPIOA->MODER |= 0b10 << GPIO_MODER_MODE6_Pos;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk;
    GPIOA->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED6_Pos;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6_Msk;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk;

    /* OCTOSPIM_P1_IO4 */
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOC->AFR[0] |= 0xA << GPIO_AFRL_AFSEL1_Pos;
    GPIOC->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOC->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOC->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

    /* OCTOSPIM_P1_IO5 */
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOC->AFR[0] |= 0xA << GPIO_AFRL_AFSEL2_Pos;
    GPIOC->MODER &= ~GPIO_MODER_MODE2_Msk;
    GPIOC->MODER |= 0b10 << GPIO_MODER_MODE2_Pos;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOC->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;

    /* OCTOSPIM_P1_IO6 */
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOC->AFR[0] |= 0xA << GPIO_AFRL_AFSEL3_Pos;
    GPIOC->MODER &= ~GPIO_MODER_MODE3_Msk;
    GPIOC->MODER |= 0b10 << GPIO_MODER_MODE3_Pos;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOC->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT3_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;

    /* OCTOSPIM_P1_IO7 */
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
    GPIOC->AFR[0] |= 0xA << GPIO_AFRL_AFSEL4_Pos;
    GPIOC->MODER &= ~GPIO_MODER_MODE4_Msk;
    GPIOC->MODER |= 0b10 << GPIO_MODER_MODE4_Pos;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
    GPIOC->OSPEEDR |= 0b11 << GPIO_OSPEEDR_OSPEED4_Pos;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;
  }
  if (device.interface_select == 0x02) {
    // enable clock for the gpio banks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

    /* OCTOSPIM_P2_CLK */
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL4_Pos;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE4_Pos;
    GPIOF->MODER &= ~GPIO_MODER_MODE4_Msk;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED4_Pos;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;

    /* OCTOSPIM_P2_NCS */
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL12_Pos;
    GPIOG->MODER &= ~GPIO_MODER_MODE12_Msk;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE12_Pos;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_Msk;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED12_Pos;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT12_Msk;
    GPIOG->PUPDR |= 0b01 << GPIO_PUPDR_PUPD12_Pos; // you need a pullup

    /* OCTOSPIM_P2_IO0 */
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL0_Pos;
    GPIOF->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;

    /* OCTOSPIM_P2_IO1 */
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL1_Pos;
    GPIOF->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

    /* OCTOSPIM_P2_IO2 */
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL2_Pos;
    GPIOF->MODER &= ~GPIO_MODER_MODE2_Msk;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE2_Pos;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT2_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;

    /* OCTOSPIM_P2_IO3 */
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
    GPIOF->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL3_Pos;
    GPIOF->MODER &= ~GPIO_MODER_MODE3_Msk;
    GPIOF->MODER |= 0b10 << GPIO_MODER_MODE3_Pos;
    GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_Msk;
    GPIOF->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT3_Msk;
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;

    /* OCTOSPIM_P2_IO4 */
    GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOG->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL0_Pos;
    GPIOG->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE0_Pos;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;

    /* OCTOSPIM_P2_IO5 */
    GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOG->AFR[0] |= 0x5 << GPIO_AFRL_AFSEL1_Pos;
    GPIOG->MODER &= ~GPIO_MODER_MODE1_Msk;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE1_Pos;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;

    /* OCTOSPIM_P2_IO6 */
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL9_Pos;
    GPIOG->MODER &= ~GPIO_MODER_MODE9_Msk;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE9_Pos;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9_Msk;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED9_Pos;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT9_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;

    /* OCTOSPIM_P2_IO7 */
    GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
    GPIOG->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL10_Pos;
    GPIOG->MODER &= ~GPIO_MODER_MODE10_Msk;
    GPIOG->MODER |= 0b10 << GPIO_MODER_MODE10_Pos;
    GPIOG->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10_Msk;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED10_Pos;
    GPIOG->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED10_Pos;
    GPIOG->OTYPER &= ~GPIO_OTYPER_OT10_Msk;
    GPIOG->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
  }
}

void mspi_sys_cfg(struct mspi_interface device) {
  /* Enable the Octa-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI2EN;
  /* Enable the Octa-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI1EN;

  if (device.interface_select == 0x01) {

    /* Reset Octa-SPI peripheral */
    RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI1RST);  /* Reset */
    RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI1RST); /* Release reset */
    /* Enable Octa-SPI DMA */
    // OCTOSPI1->CR |= OCTOSPI_CR_DMAEN;
  }
  if (device.interface_select == 0x02) {

    /* Reset Octa-SPI peripheral */
    RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI2RST);  /* Reset */
    RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI2RST); /* Release reset */
    /* Enable Octa-SPI DMA */
    // OCTOSPI2->CR |= OCTOSPI_CR_DMAEN;
  }
  /* Enable Octa-SPI Muliplexer */
  RCC->AHB2ENR |= RCC_AHB2ENR_OSPIMEN;
  OCTOSPIM->PCR[0] |= OCTOSPIM_PCR_CLKEN;
  OCTOSPIM->PCR[1] |= OCTOSPIM_PCR_CLKEN;
}

// initialization method
void mspi_dev_cfg(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    OCTOSPI1->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos; // 27
    OCTOSPI1->DCR2 &= ~(OCTOSPI_DCR2_PRESCALER_Msk);
    OCTOSPI1->DCR2 |= 0 << OCTOSPI_DCR2_PRESCALER_Pos;
  }
  if (device.interface_select == 0x02) {
    OCTOSPI2->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos; // 27
    OCTOSPI2->DCR2 &= ~(OCTOSPI_DCR2_PRESCALER_Msk);
    OCTOSPI2->DCR2 |= 0 << OCTOSPI_DCR2_PRESCALER_Pos;
  }
}

void mspi_interface_cleanup(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    // cleanup of functional registers
    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);

    OCTOSPI1->CCR &= ~(OCTOSPI_CR_FMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_IMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_ADMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_DMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_ADSIZE);
    OCTOSPI1->IR &= ~(OCTOSPI_IR_INSTRUCTION);

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

    OCTOSPI2->CCR &= ~(OCTOSPI_CR_FMODE);
    OCTOSPI2->CCR &= ~(OCTOSPI_CCR_IMODE);
    OCTOSPI2->CCR &= ~(OCTOSPI_CCR_ADMODE);
    OCTOSPI2->CCR &= ~(OCTOSPI_CCR_DMODE);
    OCTOSPI2->CCR &= ~(OCTOSPI_CCR_ADSIZE);
    OCTOSPI2->IR &= ~(OCTOSPI_IR_INSTRUCTION);

    OCTOSPI2->DLR &= ~(OCTOSPI_DLR_DL);
    OCTOSPI2->AR &= ~(OCTOSPI_AR_ADDRESS_Msk);
    OCTOSPI2->DR &= ~(OCTOSPI_DR_DATA_Msk);

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
  }
  if (device.interface_select == 0x02) {
    while (OCTOSPI2->SR & OCTOSPI_SR_BUSY) {
    };
    // PENDING add timeout
  }
}

u16 mspi_transfer(
    struct mspi_interface device_context,
    struct mspi_cmd (*device_fun_handler)(void *), // TODO: typedef this one
    void *argument) {

  struct mspi_cmd cmd = {0};
  cmd = device_fun_handler(argument);
  mspi_interface_cleanup(device_context);

  if (device_context.interface_select == 0x01) {
    /* configure for transfer */
    OCTOSPI1->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
    OCTOSPI1->CCR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                      (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                      (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));
    if (cmd.addr_mode > 0) {
      OCTOSPI1->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
    }
    if (cmd.data_mode > 0) {
      OCTOSPI1->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
    }

    /* overrides for auto-polling mode*/
    // Set the 'mask', 'match', and 'polling interval' values.
    // TEST: check if creates issues when not in autopoll mode
    OCTOSPI1->PSMKR = cmd.autopoll_mask;
    OCTOSPI1->PSMAR = cmd.autopoll_match;
    OCTOSPI1->PIR = 0x10;

    // Enable the peripheral
    OCTOSPI1->CR |= (OCTOSPI_CR_EN);

    // Set intruction
    OCTOSPI1->IR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);

    if (cmd.addr_mode > 0) {
      OCTOSPI1->AR |= (cmd.addr_cmd << OCTOSPI_AR_ADDRESS_Pos);
    }
    if (cmd.data_mode) { // write -- dma push / indirect
      // creates deadbee
      if (cmd.data_cmd_embedded == 0) {
        if (cmd.fun_mode == 0b00) {
          if (device_context.use_dma) {
            // configure and enable dma channel
            // mspi_dma_push_init(device_context.data_ptr,
            // device_context.size_tr);
          } else {
            // manually set data register content
            for (uint32_t i = 0; i < ((cmd.data_size >> 2) + 1); i++) {
              OCTOSPI1->DR |= *(device_context.data_ptr + i);
              while ((OCTOSPI1->SR & OCTOSPI_SR_FTF) == 0) {
                /*wait while FTF flag is zero */
              };
              // XXX: Could be superfluous
            }
          }
        }
      }
      // embedded data
      if (cmd.data_cmd_embedded == 1) {
        OCTOSPI1->DR |= cmd.data_cmd;
      }

      if (cmd.fun_mode == 0b01) { // read -- only indirect mode
        if (OCTOSPI1->SR & OCTOSPI_SR_FTF_Msk)
          for (uint32_t i = 0; i < (cmd.data_size + 1); i++) {
            *(device_context.data_ptr + i) = OCTOSPI1->DR;
          }
      }
    }
    // wait for the transaction to complete (+ timeout and abort)
    if (mspi_interface_wait_busy(device_context)) {
      OCTOSPI1->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
      return ERROR_MSPI_INTERFACE_STUCK;
    }

    // Acknowledge the 'status match flag.'
    OCTOSPI1->FCR |= (OCTOSPI_FCR_CSMF);
    // Un-set the data mode and disable auto-polling.
    OCTOSPI1->CCR &= ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_DMODE);

    // disable dma channel
    DMA1_Channel1->CCR &= ~(DMA_CCR_EN);

    // disable interface
    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);

    return EXIT_SUCCESS;
  }
  if (device_context.interface_select == 0x02) {
    /* configure for transfer */
    OCTOSPI2->CR |= ((cmd.fun_mode << OCTOSPI_CR_FMODE_Pos));
    OCTOSPI2->CCR |= ((cmd.instr_mode << OCTOSPI_CCR_IMODE_Pos) |
                      (cmd.addr_mode << OCTOSPI_CCR_ADMODE_Pos) |
                      (cmd.data_mode << OCTOSPI_CCR_DMODE_Pos));
    if (cmd.addr_mode > 0) {
      OCTOSPI2->CCR |= (cmd.addr_size << OCTOSPI_CCR_ADSIZE_Pos);
    }
    if (cmd.data_mode > 0) {
      OCTOSPI2->DLR |= (cmd.data_size << OCTOSPI_DLR_DL_Pos);
    }
    /* overrides for auto-polling mode*/
    // Set the 'mask', 'match', and 'polling interval' values.
    // TEST: check if creates issues when not in autopoll mode
    OCTOSPI2->PSMKR = cmd.autopoll_mask;
    OCTOSPI2->PSMAR = cmd.autopoll_match;
    OCTOSPI2->PIR = 0x10;

    // Enable the peripheral
    OCTOSPI2->CR |= (OCTOSPI_CR_EN);

    // Set intruction
    OCTOSPI2->IR |= (cmd.instr_cmd << OCTOSPI_IR_INSTRUCTION_Pos);
    // Set the address
    if (cmd.addr_mode > 0) {
      OCTOSPI2->AR |= (cmd.addr_cmd << OCTOSPI_AR_ADDRESS_Pos);
    }
    if (cmd.data_mode) { // write -- dma push / indirect
      if (cmd.data_cmd_embedded == 0 && cmd.fun_mode == 0b00) {
        if (device_context.use_dma) {
          // configure and enable dma channel
          // mspi_dma_push_init(device_context.data_ptr,
          // device_context.size_tr);
        } else {
          // manually set data register content
          // static volatile uint32_t testval = {0};
          for (uint32_t i = 0; i < (cmd.data_size >> 2); i++) {
            // NOTE: works with words transfers
            // testval = (u32) * (device_context.data_ptr + i);
            // XXX: the cast was superfluous
            OCTOSPI2->DR |= *(device_context.data_ptr + i);
            // TODO: add some kind of fifo check
            // XXX: maybe ad a bit of spin
          }
        }
      }
      // autopoll -- embedded data
      if (cmd.data_cmd_embedded == 0) {
        OCTOSPI2->DR |= cmd.data_cmd;
      }

      if (cmd.fun_mode == 0b01) { // read -- only indirect mode
        if (OCTOSPI2->SR & OCTOSPI_SR_FTF_Msk)
          for (uint32_t i = 0; i < (device_context.size_tr >> 2); i++) {
            *(device_context.data_ptr + i) = OCTOSPI2->DR;
            // TODO: add some kind of fifo check
          }
      }
    }
    // wait for the transaction to complete for autopoll case
    // to implement: timeout and abort
    //
    if (mspi_interface_wait_busy(device_context)) {
      OCTOSPI2->CR &= ~(OCTOSPI_CR_EN); // disable the interface in anay case
      return ERROR_MSPI_INTERFACE_STUCK;
    }

    // Acknowledge the 'status match flag.'
    OCTOSPI2->FCR |= (OCTOSPI_FCR_CSMF);
    // Un-set the data mode and disable auto-polling.
    OCTOSPI2->CCR &= ~(OCTOSPI_CR_FMODE | OCTOSPI_CCR_DMODE);

    // disable dma channel
    DMA1_Channel1->CCR &= ~(DMA_CCR_EN);

    // disable interface
    OCTOSPI2->CR &= ~(OCTOSPI_CR_EN);

    return EXIT_SUCCESS;
  }
}

#define OCTOSPI_DR_OFF 0x050
void mspi_dma_push_init(u32 *data_ptr, u32 size_tr) {
  /* Set up DMA1_CH1 "Push" dma channel */

  /* Enable the dma controller 1 peripheral */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Clear interrupt status flags */
  DMA1->IFCR |= DMA_IFCR_CGIF1;

  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel1->CPAR = OCTOSPI2_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  // TEST: test if the address is correct
  DMA1_Channel1->CMAR = (uint32_t)data_ptr;
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel1->CNDTR = size_tr >> 2;
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

  /* Activate the dma channel */
  DMA1_Channel1->CCR |= DMA_CCR_EN;
}
