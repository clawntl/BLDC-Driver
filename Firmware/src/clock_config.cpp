//------------------------------------------------------------------------------
//  SystemClock_Config for the custom STM32G431CBU6 board
//
//  Ported 1:1 from the CubeMX-generated Core/Src/main.c that was supplied
//  for this board. The STM32duino Arduino core declares SystemClock_Config()
//  as a __weak symbol and calls it once from its startup code before
//  setup() runs, so this strong definition simply replaces the core's
//  default clock tree with the one this board actually needs.
//
//  Clock tree:
//    HSE (16 MHz, see build_flags -DHSE_VALUE=16000000) --/4--> 4 MHz
//      --x80--> 320 MHz VCO --/2--> SYSCLK = 160 MHz (PLLR)
//    AHB  = SYSCLK /1 = 160 MHz
//    APB1 = HCLK   /1 = 160 MHz  <-- FDCAN kernel clock source (PCLK1)
//    APB2 = HCLK   /1 = 160 MHz
//    HSI48 + CRS   -> used for USB
//
//  IMPORTANT: this requires PWR boost mode (VOS1 boost) because 160 MHz is
//  above the non-boosted VOS1 limit (150 MHz) on the G4. That is configured
//  below exactly as CubeMX generated it.
//------------------------------------------------------------------------------

#include <Arduino.h>

//------------------------------------------------------------------------------
//  Error_Handler() used below is provided by the STM32duino core.
//------------------------------------------------------------------------------

extern "C" void SystemClock_Config (void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  //--- Configure the main internal regulator output voltage (boost mode, needed for 160 MHz)
  HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) ;

  //--- Oscillators: HSE + HSI48, PLL sourced from HSE
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE ;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON ;
  RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON ;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON ;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE ;
  RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4 ;
  RCC_OscInitStruct.PLL.PLLN       = 80 ;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2 ;
  RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2 ;
  RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2 ;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler () ;
  }

  //--- CPU / AHB / APB bus clocks, all /1 -> 160 MHz
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 ;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK ;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1 ;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1 ;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1 ;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler () ;
  }

  //--- HSI48 trimmed against USB SOF via CRS, for USB reliability without a crystal on HSI48
  __HAL_RCC_CRS_CLK_ENABLE () ;
  pInit.Prescaler          = RCC_CRS_SYNC_DIV1 ;
  pInit.Source             = RCC_CRS_SYNC_SOURCE_USB ;
  pInit.Polarity           = RCC_CRS_SYNC_POLARITY_RISING ;
  pInit.ReloadValue        = __HAL_RCC_CRS_RELOADVALUE_CALCULATE (48000000, 1000) ;
  pInit.ErrorLimitValue    = 34 ;
  pInit.HSI48CalibrationValue = 32 ;
  HAL_RCCEx_CRSConfig (&pInit) ;
}

//------------------------------------------------------------------------------
