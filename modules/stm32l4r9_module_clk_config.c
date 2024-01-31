#include <stm32l4r9_module_clk_config.h>

extern struct Node *hw_head;

/* local */
void SystemClock_Config(void);

/* public */
u32 system_clock_init(void) {
  hwlist_require(&hw_head, &HAL_Init, &system_clock_init);
  /* if the PLL clock source is in use skips configuration */
  if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0b11 << RCC_CFGR_SWS_Pos) {
    SystemClock_Config();
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) !=
      HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

u32 sensor_clock_init(void) {
  hwlist_require(&hw_head, &system_clock_init, &sensor_clock_init);
  /* enable debug clock output */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // enable AHB2 clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  RCC->CFGR |= 0b0101 << RCC_CFGR_MCOSEL_Pos; // PLL
  // RCC->CFGR |= 0b0010 << RCC_CFGR_MCOPRE_Pos; // div 4
  // RCC->CFGR |= 0b0001 << RCC_CFGR_MCOPRE_Pos; // div 2
  // RCC->CFGR |= 0b0000 << RCC_CFGR_MCOPRE_Pos; // div 1
  RCC->CFGR |= 0b0011 << RCC_CFGR_MCOPRE_Pos; // div 8

  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8_Msk;
  GPIOA->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED8_Pos;

  // configure clock out gpio MCO
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
  GPIOA->AFR[1] |= 0b00 << GPIO_AFRH_AFSEL8_Msk;

  GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
  GPIOA->MODER |= 0b10 << GPIO_MODER_MODE8_Pos;
}
