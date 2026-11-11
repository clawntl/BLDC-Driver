#include "dfu.h"

#include <Arduino.h>
#include <cstring>

#include "stm32g4xx_hal.h" // needed for HAL_DeInit / rebootToDFU
#include <IWatchdog.h>

#include "pins.h"

// Magic left in a TAMP backup register to ask the next boot to go straight to
// the bootloader. Backup registers keep their contents across a system reset.
static constexpr uint32_t DFU_REQUEST_MAGIC = 0xDF00B007;

static void enableBackupRegisterAccess() {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
}

// Direct jump into the STM32 system-memory (USB DFU) bootloader from the
// running firmware. Only safe while the independent watchdog is NOT running:
// the bootloader never reloads it, so the IWDG would reset the chip out of
// DFU mode within one timeout period.
static void jumpToBootloader() {
    __disable_irq();

    // Stop SysTick
    SysTick->CTRL = 0;

    // Deinit HAL (shuts down peripherals & interrupts)
    HAL_RCC_DeInit();
    HAL_DeInit();

    // ---- Disable USB (STM32G431 version) ----

    // Disable USB clock
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USBEN;

    // Reset USB peripheral
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_USBRST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

    // Reset USB pins PA11/PA12 to analog
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Remap system memory to 0x00000000
    SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;

    // System memory bootloader start
    uint32_t bootAddr = 0x1FFF0000;

    // Load stack pointer and reset vector
    uint32_t sp = *(__IO uint32_t*)(bootAddr);
    uint32_t rv = *(__IO uint32_t*)(bootAddr + 4);

    // Set main stack pointer
    __set_MSP(sp);

    // Jump to bootloader
    ((void (*)(void))rv)();
}

void rebootToDFU() {
    if (IWatchdog.isEnabled()) {
        // The IWDG can't be stopped, but a system reset does stop it (it is
        // software-started). Reset, and let checkDfuRequestAtBoot() do the
        // jump before anything restarts the watchdog. The reset also puts all
        // the PWM outputs back into their reset (Hi-Z) state.
        enableBackupRegisterAccess();
        TAMP->BKP0R = DFU_REQUEST_MAGIC;
        NVIC_SystemReset();
    }
    jumpToBootloader();
}

void softwareReset() {
    // Disable the CAN transceiver so it stops driving RX (= BOOT0) high, and
    // take PB8 away from FDCAN so we can read what the pin is actually doing.
    pinMode(CAN_EN, OUTPUT);
    digitalWrite(CAN_EN, LOW);
    pinMode(CAN_RX_BOOT0, INPUT);

    const uint32_t start = millis();
    while (digitalRead(CAN_RX_BOOT0) == HIGH && (millis() - start) < 100) {
        // The watchdog may be running (FOC was started); don't let it fire mid-wait.
        if (IWatchdog.isEnabled()) IWatchdog.reload();
    }

    if (digitalRead(CAN_RX_BOOT0) == HIGH) {
        // Either the transceiver keeps RX driven when disabled, or the BOOT
        // button is being held. Reset anyway; the bootloader will start.
        Serial.println("WARNING: BOOT0 (PB8) still high with the CAN transceiver disabled - "
                       "the board will probably start the DFU bootloader. Press reset to recover.");
    } else {
        Serial.print("BOOT0 low after ");
        Serial.print(millis() - start);
        Serial.println(" ms - resetting");
    }
    if (IWatchdog.isEnabled()) IWatchdog.reload();
    Serial.flush();
    delay(20);

    NVIC_SystemReset();
}

void checkDfuRequestAtBoot() {
    enableBackupRegisterAccess();
    if (TAMP->BKP0R == DFU_REQUEST_MAGIC) {
        TAMP->BKP0R = 0; // one-shot, so the next reset boots normally
        jumpToBootloader();
    }
}

void checkDfuButton() {
    static bool dfuButtonPrevState = HIGH;

    const bool dfuButtonState = digitalRead(DFU_BUTTON);
    if (dfuButtonState == LOW && dfuButtonPrevState == HIGH) {
        Serial.println("Entering DFU mode...");
        rebootToDFU();
    }
    dfuButtonPrevState = dfuButtonState;
}
