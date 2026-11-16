#pragma once

//------------------------------------------------------------------------------
//   Settings for custom STM32G431CBU6 board
//------------------------------------------------------------------------------

#define HAS_PROGRAMMABLE_FDCAN_RAM_SECTIONS (false)

//------------------------------------------------------------------------------

#include <stm32g4xx_ll_rcc.h>

//------------------------------------------------------------------------------
//  FDCAN kernel clock source.
//  This matches the Cube-generated HAL_FDCAN_MspInit for this board
//  (Core/Src/stm32g4xx_hal_msp.c : PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1).
//  With the SystemClock_Config ported from Core/Src/main.c (HSE -> PLL, boost mode),
//  SYSCLK = 160 MHz and AHB/APB1 prescalers are both /1, so PCLK1 = 160 MHz.
//------------------------------------------------------------------------------

inline uint32_t fdcanClock (void) {
  if (!__HAL_RCC_FDCAN_IS_CLK_ENABLED ()) { // Is not enabled ?
  //--- Enable CAN clock
    __HAL_RCC_FDCAN_CLK_ENABLE () ;
  //--- Reset CAN peripherals
    __HAL_RCC_FDCAN_FORCE_RESET () ;
    __HAL_RCC_FDCAN_RELEASE_RESET () ;
  //--- Select CAN clock source: PCLK1
    LL_RCC_SetFDCANClockSource (RCC_FDCANCLKSOURCE_PCLK1) ;
  }
  return HAL_RCC_GetPCLK1Freq () ;
}

//------------------------------------------------------------------------------
