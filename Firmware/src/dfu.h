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
