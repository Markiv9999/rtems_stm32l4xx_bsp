#include "stm32l4r9xx.h"

void clk_cfg(void) {
  // ------------ CLOCK  CONFIGURATION ----------------------------------------
  if ((RCC->CFGR & RCC_CFGR_SWS_Msk) !=
      0b11) { // if current clock is not PLL TODO: make more robust, pll does
              // not mean it is configured

    RCC->CR &= ~(RCC_CR_PLLON_Msk); // easy way to unset the register
    while ((RCC->CR & RCC_CR_PLLRDY_Msk)) {
    }; // easy way to check a specific mask of the register

    FLASH->ACR |= (6 << FLASH_ACR_LATENCY_Pos);
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_Msk | // clear coefficient fields
                      RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLR_Msk);
    // RCC->PLLCFGR |=  ( ( 28 << RCC_PLLCFGR_PLLN_Pos ) |
    RCC->PLLCFGR |= ((40 << RCC_PLLCFGR_PLLN_Pos) | // 4Mhz input multiplied by
                                                    // 40 = 160Mhz, /2 = 80Mhz
                     (0b000 << RCC_PLLCFGR_PLLM_Pos) | // divived by 1
                     (0b00 << RCC_PLLCFGR_PLLR_Pos));  // divided by 4
    // enable HSE
    // RCC->CR |= ( RCC_CR_HSEON );
    // while ( !(RCC->CR & RCC_CR_HSERDY) ) {}; //wait for PLL to be used as
    // clock source

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC_Msk);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI; // select msi as pll clock source

    RCC->CR |= (RCC_CR_PLLON); // easy way yo set the register
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    };

    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as clock source
    // RCC->CFGR   |=  RCC_CFGR_SW_HSE;            //select HSE as clock source
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {
    }; // wait for PLL to be used as clock source
    // while ( ( RCC->CFGR & RCC_CFGR_SWS_Msk ) != RCC_CFGR_SWS_HSE ) {}; //wait
    // for HSE to be used as clock source
    //  TODO: Document the clock configurations used
    //  TODO: If possible move clock initialization even before
  }
}
