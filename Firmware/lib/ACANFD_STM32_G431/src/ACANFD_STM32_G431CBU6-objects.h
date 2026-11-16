#pragma once

//------------------------------------------------------------------------------

#include <ACANFD_STM32_from_cpp.h>

//------------------------------------------------------------------------------
//  Custom board: STM32G431CBU6
//    FDCAN_RX : PB8  (AF9)
//    FDCAN_TX : PB9  (AF9)
//  (matches HAL_FDCAN_MspInit generated for this board — see Core/Src/stm32g4xx_hal_msp.c)
//
//  If your board revision routes FDCAN1 to different pins, add them to the
//  arrays below (first entry is default, override at runtime with
//  ACANFD_STM32_Settings::mTxPin / mRxPin — index into these arrays).
//------------------------------------------------------------------------------
//   FDCAN1
//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan1_tx_pin_array {
  ACANFD_STM32::PinPort (PB_9, 9), // Tx Pin: PB_9, AF9
} ;

//------------------------------------------------------------------------------

const std::initializer_list <ACANFD_STM32::PinPort> fdcan1_rx_pin_array {
  ACANFD_STM32::PinPort (PB_8, 9), // Rx Pin: PB_8, AF9
} ;

//------------------------------------------------------------------------------

ACANFD_STM32 fdcan1 (
  FDCAN1, // CAN Peripheral base address
  SRAMCAN_BASE, // CAN RAM Base Address
  ACANFD_STM32::IRQs (FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn), // interrupts
  fdcan1_tx_pin_array,
  fdcan1_rx_pin_array
) ;

//------------------------------------------------------------------------------

extern "C" void FDCAN1_IT0_IRQHandler (void) ;
extern "C" void FDCAN1_IT1_IRQHandler (void) ;

//------------------------------------------------------------------------------

void FDCAN1_IT0_IRQHandler (void) {
  fdcan1.isr0 () ;
}

//------------------------------------------------------------------------------

void FDCAN1_IT1_IRQHandler (void) {
  fdcan1.isr1 () ;
}

//------------------------------------------------------------------------------
