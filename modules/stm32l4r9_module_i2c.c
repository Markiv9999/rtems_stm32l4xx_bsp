#include "ext_typedefs.h"
#include "stm32l4r9xx.h"

/*
the sensor control interface uses I2C1
the pins used are PB9 and PB8
PB9 - SDA - AF4
PB8 - SCL - AF4
*/

u16 i2c1_initialize(void) {
  // the i2c peripheral is located on APB1

  // clocking the peripheral
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

  // clock the GPIOB bank
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  // clear gpio mode register
  GPIOB->MODER &= ~(GPIO_MODER_MODE8_0);
  GPIOB->MODER &= ~(GPIO_MODER_MODE8_0);
  GPIOB->MODER &= ~(GPIO_MODER_MODE9_1);
  GPIOB->MODER &= ~(GPIO_MODER_MODE9_1);

  // sets gpio mode to alternate function
  GPIOB->MODER |= GPIO_MODER_MODE8_1;
  GPIOB->MODER |= GPIO_MODER_MODE9_1;

  // sets gpio altenate function
  GPIOB->AFR |= (0x4 << GPIO_AFRH_AFSEL8_Pos) | (0x4 << GPIO_AFRH_AFSEL9_Pos);

  // sets gpio pull up

  // sets gpio to open drain
}
