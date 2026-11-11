#pragma once

// Checks the DFU button (active low) for a press edge and reboots straight
// into the USB DFU bootloader if pressed. Call once per loop() iteration.
void checkDfuButton();

// Reboots straight into the STM32 system memory (USB DFU) bootloader, e.g. so a
// PlatformIO/dfu-util upload can find the device without a manual BOOT0+reset.
// Never returns. If the independent watchdog is running (main firmware once
// FOC has started) this goes via a system reset + checkDfuRequestAtBoot(),
// because the bootloader doesn't reload the watchdog.
void rebootToDFU();

// Must be the FIRST thing called in setup(): completes a DFU request made by
// rebootToDFU() before the reset. Returns normally if there was no request.
void checkDfuRequestAtBoot();

// Use this instead of NVIC_SystemReset() for a normal reboot.
// PB8 is both FDCAN1_RX and BOOT0, and the chip samples BOOT0 as it leaves
// reset. While the CAN transceiver is enabled it drives RX (PB8) high
// whenever the bus is idle - even with nothing connected - overpowering the
// 10k pull-down, so a plain software reset starts the ST DFU bootloader.
// This disables the transceiver, waits (up to 100 ms) for PB8 to read low,
// then resets. Prints a warning if PB8 never went low. Never returns.
// (A watchdog reset can't do this, so it may still land in the bootloader.)
void softwareReset();
