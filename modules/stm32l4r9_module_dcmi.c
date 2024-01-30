#include "stm32l4r9_module_dcmi.h"

/* static buffers */
// XXX: this works, but it is half of a raw image
uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_BYTESIZE];

u32 dcmi_init(void) {
  /* PREAMBLE */
  /* initialize and dirty the target buffer for debug purposes
   */
  for (u32 i = 0; i < MAX_DMA_TRS_SIZE; i++) {
    dcmi_dma_buffer[i] = 0xBEEFFEED;
  }
/* INIT DEPENDANCIES */
#define CURRENT_IF &dcmi_init
  /* sensor related dependancies */
  hwlist_require(&hw_head, &ov5640_init, CURRENT_IF);

  /* image acquisition system dependancies */
  hwlist_require(&hw_head, &dcmi_dmachannel_init, CURRENT_IF);
  hwlist_require(&hw_head, &dcmi_dmamux_init, CURRENT_IF);
  hwlist_require(&hw_head, &dcmi_dma_enable, CURRENT_IF);
  hwlist_require(&hw_head, &dcmi_peripheral_init, CURRENT_IF);
}

u32 dcmi_dmachannel_init(void) {
  /*
   * note: probably since the data to be transfered is of variable size
   * due to the jpeg compression,
   * the transfer will probably not complete after a frame and the
   * transfer complete flag will not be asserted
   */

  /* TODO: you can macro the DMA1_Channel3 name,
   * to a name that is functional and can be contained
   * with the others in a configuration file
   */

  /* Enable the dma controller 1 peripheral */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Clear interrupt status flags */
  DMA1->IFCR |= DMA_IFCR_CGIF3;

  /* Set the peripheral register address in the DMA_CPARx register */
  DMA1_Channel3->CPAR = DCMI_BASE + 0x28; // dcmi data register

  DMA1_Channel3->CMAR = (uint32_t)&dcmi_dma_buffer[0];
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel3->CNDTR = MAX_DMA_TRS_SIZE;
  /* Configure the CCR register */
  DMA1_Channel3->CCR |=
      /* channel priority */
      0b00 << DMA_CCR_PL_Pos |
      /* memory size (32bit)*/
      0b11 << DMA_CCR_MSIZE_Pos |
      /* peripheral size (32bit)*/
      0b11 << DMA_CCR_PSIZE_Pos |
      /* dma in circular mode */
      DMA_CCR_CIRC |
      /* memory increment mode */
      DMA_CCR_MINC;
  /* DIR control register is 0, read from peripheral */
  /* Set up DMA1_CH1 "Push" dma channel */
}

#define DMAMUX_DCMI_REQUEST_CODE 90
u32 dcmi_dmamux_init(void) {
  /*Enable DMAMUX clock */

  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

  /* connects dcmi request to a specific dma multiplexer channel */
  DMAMUX1_Channel2->CCR |= DMAMUX_DCMI_REQUEST_CODE;
  /* syncronization identification */
  // DMAMUX1_Channel2->CCR |= 3 << 24; // SYNC_ID [28:24]
  // XXX: not sure if this is needed, or what it does..
}

u32 dcmi_dma_enable(void) {
  /* Enable the dma channel */
  DMA1_Channel3->CCR |= DMA_CCR_EN;
}

u32 dcmi_peripheral_init(void) {
  // XXX: since it is jpeg, how many data lines are being used by the sensor
  // in the 720p configuration? The peripheral needs to be configured
  // accordingly
  /*
   * clocks the dcmi peripheral
   * */
  RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
  /*
   * resets the dcmi peripheral
   * */
  RCC->AHB2RSTR |= RCC_AHB2RSTR_DCMIRST;
  RCC->AHB2RSTR &= ~(RCC_AHB2RSTR_DCMIRST);

  /*
   * Configure the DCMI gpios
   * */
  // enable clock for the gpio banks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;

#define DCMI_H_AF 0x0A
#define DCMI_L_AF 0x04

  /* PA4 - DMCI_HSYNC */
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
  GPIOA->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL4_Pos;
  GPIOA->MODER &= ~GPIO_MODER_MODE4_Msk;
  GPIOA->MODER |= 0b10 << GPIO_MODER_MODE4_Pos;

  /* PD9 - DMCI_PXCLK */
  GPIOD->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
  GPIOD->AFR[1] |= DCMI_H_AF << GPIO_AFRH_AFSEL9_Pos;
  GPIOD->MODER &= ~GPIO_MODER_MODE9_Msk;
  GPIOD->MODER |= 0b10 << GPIO_MODER_MODE9_Pos;

  /* PB7 - DMCI_VSYNC */
  GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
  GPIOB->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL7_Pos;
  GPIOB->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOB->MODER |= 0b10 << GPIO_MODER_MODE7_Pos;

  /* PC6 - DMCI_D0 */
  GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
  GPIOC->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL6_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE6_Pos;

  /* PC7 - DMCI_D1 */
  GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
  GPIOC->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL7_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE7_Pos;

  /* PC8 - DMCI_D2 */
  GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
  GPIOC->AFR[1] |= DCMI_H_AF << GPIO_AFRH_AFSEL8_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE8_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE8_Pos;

  /* PC9 - DMCI_D3 */
  GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
  GPIOC->AFR[1] |= DCMI_L_AF << GPIO_AFRH_AFSEL9_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE9_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE9_Pos;

  /* PC10 - DMCI_D8 */
  GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
  GPIOC->AFR[1] |= DCMI_H_AF << GPIO_AFRH_AFSEL10_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE10_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE10_Pos;

  /* PC12 - DMCI_D9 */
  GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;
  GPIOC->AFR[1] |= DCMI_H_AF << GPIO_AFRH_AFSEL12_Pos;
  GPIOC->MODER &= ~GPIO_MODER_MODE12_Msk;
  GPIOC->MODER |= 0b10 << GPIO_MODER_MODE12_Pos;

  /* PD3 - DMCI_D5 */
  GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
  GPIOD->AFR[0] |= DCMI_L_AF << GPIO_AFRL_AFSEL3_Pos;
  GPIOD->MODER &= ~GPIO_MODER_MODE3_Msk;
  GPIOD->MODER |= 0b10 << GPIO_MODER_MODE3_Pos;

  /* PE4 - DMCI_D4 */
  GPIOE->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
  GPIOE->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL4_Pos;
  GPIOE->MODER &= ~GPIO_MODER_MODE4_Msk;
  GPIOE->MODER |= 0b10 << GPIO_MODER_MODE4_Pos;

  /* PE5 - DMCI_D6 */
  GPIOE->AFR[0] &= ~GPIO_AFRL_AFSEL5_Msk;
  GPIOE->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL5_Pos;
  GPIOE->MODER &= ~GPIO_MODER_MODE5_Msk;
  GPIOE->MODER |= 0b10 << GPIO_MODER_MODE5_Pos;

  /* PE6 - DMCI_D7 */
  GPIOE->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
  GPIOE->AFR[0] |= DCMI_H_AF << GPIO_AFRL_AFSEL6_Pos;
  GPIOE->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOE->MODER |= 0b10 << GPIO_MODER_MODE6_Pos;

  /*
   * configures the dcmi peripheral
   * */

  /* Vsync polarity -> active low */
  // keep reset (0)
  DCMI->CR |= DCMI_CR_VSPOL;
  // XXX: typically removed

  /* Hsync polarity -> active high */
  // DCMI->CR |= DCMI_CR_HSPOL;

  /* PCLK polarity -> active high(rising) */
  DCMI->CR |= DCMI_CR_PCKPOL;

  /* JPEG format -> enabled */
  DCMI->CR |= DCMI_CR_JPEG;

  /* Capture mode -> snapshot */
  DCMI->CR |= DCMI_CR_CM;

  /* Capture enable */
  // XXX: Check if needs to be enabled before or after peripheral enable
  // DCMI->CR |= DCMI_CR_CAPTURE;

  /* enable interface */
  DCMI->CR |= DCMI_CR_ENABLE;
}

u32 *dcmi_get_buffer_ptr(void) { return &dcmi_dma_buffer[0]; }
