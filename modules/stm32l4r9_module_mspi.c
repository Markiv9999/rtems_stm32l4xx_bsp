#include "stm32l4r9_module_mspi.h"
#include "stm32l4r9_module_mspi_mt29.h"

static uint32_t dma_push_buffer[MT29_PAGE_W_SIZE];
static uint32_t dma_test_buffer[MT29_PAGE_W_SIZE] = {0};

void mspi_dmamux_cfg(void);
/**
 * --------------------------------------------------------------------------- *
 *       INITIALIZATION INTERFACE
 * --------------------------------------------------------------------------- *
 */
u32 mspi_init(void) {
  /* INIT PREHAMBLE */

  /* INIT DEPENDANCIES */
#define CURRENT_IF &mspi_init
  /* sensor related dependancies */
  hwlist_require(&hw_head, &mspi1_dmamux_init, CURRENT_IF);

  hwlist_require(&hw_head, &mspi1_gpio_init, CURRENT_IF);
  hwlist_require(&hw_head, &mspi1_peripheral_init, CURRENT_IF);
  hwlist_require(&hw_head, &mspi_mux_init, CURRENT_IF);
  hwlist_require(&hw_head, &mspi1_dev_cfg, CURRENT_IF);

  /* produce a document that gives a snapshot of the subsytem status at the end
   * of the configuration */
}
/**
 * --------------------------------------------------------------------------- *
 *       INITIALIZATION METHODS
 * --------------------------------------------------------------------------- *
 */
u32 mspi1_gpio_init(void) {
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

u32 mspi2_gpio_init(void) {
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

u32 mspi1_peripheral_init(void) {
  /* Enable the Octa-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI1EN;

  /* Reset Octa-SPI peripheral */
  RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI1RST);  /* Reset */
  RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI1RST); /* Release reset */
}

u32 mspi2_peripheral_init(void) {
  /* Enable the Octa-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_OSPI2EN;

  /* Reset Octa-SPI peripheral */
  RCC->AHB3RSTR |= (RCC_AHB3RSTR_OSPI2RST);  /* Reset */
  RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_OSPI2RST); /* Release reset */
                                             /* Enable Octa-SPI DMA */
}

u32 mspi_mux_init(void) {
  /* Enable Octa-SPI Muliplexer */
  RCC->AHB2ENR |= RCC_AHB2ENR_OSPIMEN;
  OCTOSPIM->PCR[0] |= OCTOSPIM_PCR_CLKEN;
  OCTOSPIM->PCR[1] |= OCTOSPIM_PCR_CLKEN;
}

// initialization method
u32 mspi1_dev_cfg(void) {
  OCTOSPI1->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos; // 27
  OCTOSPI1->DCR2 &= ~(OCTOSPI_DCR2_PRESCALER_Msk);
  OCTOSPI1->DCR2 |= 0 << OCTOSPI_DCR2_PRESCALER_Pos;
}
u32 mspi2_dev_cfg(void) {
  OCTOSPI2->DCR1 |= 27 << OCTOSPI_DCR1_DEVSIZE_Pos; // 27
  OCTOSPI2->DCR2 &= ~(OCTOSPI_DCR2_PRESCALER_Msk);
  OCTOSPI2->DCR2 |= 0 << OCTOSPI_DCR2_PRESCALER_Pos;
}

u32 mspi1_dmamux_init(void) {
  /*
   * Configures the DMAMUX for the push and pull channels
   * */

  /* DEPENDANCIES */
  hwlist_require(&hw_head, &mspi1_dmachannel_push_init, CURRENT_IF);
  hwlist_require(&hw_head, &mspi1_dmachannel_pull_init, CURRENT_IF);

  /*Enable DMAMUX clock if not already enabled */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

  /* DMAMUX OCTOSPI1 to Push dma channel */
  while ((DMAMUX1_Channel0->CCR & 0x7f) != 41UL) {
    DMAMUX1_Channel0->CCR |= 41UL;
  }
  /* DMAMUX OCTOSPI1 to Pull dma channel */
  // TODO:
}
u32 mspi2_dmamux_init(void) {
  /*
   * Configures the DMAMUX for the push and pull channels
   * */

  /* DEPENDANCIES */
  hwlist_require(&hw_head, &mspi2_dmachannel_push_init, CURRENT_IF);
  hwlist_require(&hw_head, &mspi2_dmachannel_pull_init, CURRENT_IF);

  /*Enable DMAMUX clock if not already enabled */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

  /* DMAMUX OCTOSPI2 to Push dma channel */
  // TODO:

  /* DMAMUX OCTOSPI2 to Pull dma channel */
  // TODO:
}

#define OCTOSPI_DR_OFF 0x050
u32 mspi1_dmachannel_push_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Clear interrupt status flags */
  DMA1->IFCR |= DMA_IFCR_CGIF1;

  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel1->CPAR = (u32) & (OCTOSPI1->DR);
  // DMA1_Channel1->CPAR = (u32)&dma_test_buffer[0];
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel1->CMAR = (u32)&dma_push_buffer[0];
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel1->CNDTR = MT29_PAGE_W_SIZE;
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
      DMA_CCR_MINC |
      /* memory circular mode */
      DMA_CCR_CIRC;
}
void mspi1_dmachannel_pull_init(void) {}
void mspi2_dmachannel_push_init(void) {}
void mspi2_dmachannel_pull_init(void) {}

/* --------------------------------------------------------------- */
/* ------ FUNCTIONAL METHODS ------------------------------------- */
/* --------------------------------------------------------------- */

void mspi_interface_cleanup(struct mspi_interface device) {
  if (device.interface_select == 0x01) {
    // cleanup of functional registers
    OCTOSPI1->CR &= ~(OCTOSPI_CR_EN);
    OCTOSPI1->CR &= ~(OCTOSPI_CR_DMAEN);

    OCTOSPI1->CR &= ~(OCTOSPI_CR_FMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_IMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_ADMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_DMODE);
    OCTOSPI1->CCR &= ~(OCTOSPI_CCR_ADSIZE);

    OCTOSPI1->TCR &= ~(OCTOSPI_TCR_DCYC);

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

inline u16 mspi_interface_wait_busy(struct mspi_interface device) {
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

    OCTOSPI1->TCR |= cmd.dummy_size << OCTOSPI_TCR_DCYC_Pos;

    // XXX: REMOVE
    // memcpy(dma_push_buffer, device_context.data_ptr, cmd.data_size);

    /* overrides for auto-polling mode*/
    // Set the 'mask', 'match', and 'polling interval' values.
    OCTOSPI1->PSMKR = cmd.autopoll_mask;
    OCTOSPI1->PSMAR = cmd.autopoll_match;
    OCTOSPI1->PIR = 0x10;

    OCTOSPI1->CR |= 0x4 << OCTOSPI_CR_FTHRES_Pos;
    /*sets dma utilization*/
    OCTOSPI1->CR |= OCTOSPI_CR_DMAEN;
    /*sets automatic polling stop*/
    OCTOSPI1->CR |= OCTOSPI_CR_APMS;
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
            /* enables the dma channel */
            DMA1_Channel1->CCR |= DMA_CCR_EN;
          } else {
            // manually set data register content
            for (uint32_t i = 0; i < ((cmd.data_size >> 2) + 1); i++) {
              OCTOSPI1->DR |= (u32) * (device_context.data_ptr + i);
              while (~(OCTOSPI1->SR) & OCTOSPI_SR_FTF) {
                /*wait while FTF flag is zero */
              };
              // XXX: Could be superfluous? No, otherwhise
              // octospi interface causes bus error
            }
          }
        }
      }
      // embedded data
      if (cmd.data_cmd_embedded == 1) {
        OCTOSPI1->DR |= cmd.data_cmd;
      }

      if (cmd.fun_mode == 0b01) { // read -- only indirect mode
        for (uint32_t i = 0; i < ((cmd.data_size >> 2) + 1); i++) {
          *(device_context.data_ptr + i) =
              OCTOSPI1->DR; // very imprtant that it clears and copies
          // add fifo check
          if (~(OCTOSPI1->SR) & OCTOSPI_SR_TCF_Msk)
            while (~(OCTOSPI1->SR) & OCTOSPI_SR_FTF_Msk) {
              /*wait while FTF flag is zero */
            };
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
  }
}
