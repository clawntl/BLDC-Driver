# Mk3 changes

Planned changes for the next revision. The main goal is to bring cost down.

## Planned

### 1. Switch back to the DRV8316's built-in low-side current sense
- Remove the three inline shunts (R1, R4, R8; 3 mΩ, 1206) and the three INA241A3 amplifiers (U1, U11, U12) along with their decoupling and reference parts.
- Route the DRV8316C's current-sense outputs (SOA/SOB/SOC) to STM32 ADC pins. Set the CSA gain over SPI to suit the motor's current range.
- Firmware: switch from inline to low-side current sensing. The ADC has to sample in sync with the PWM, while the low-side FETs are on. SimpleFOC supports this on the STM32G4.
- This frees space on the bottom side, where the inline sense stage currently sits.

### 2. Cheaper CAN transceiver
- Replace the ATA6561-GBQW-N (U4) with a SIT1051ATK/3 or a similar part.
- Before swapping, check the package and pinout against the current footprint, how the silent/standby pin is wired, and that the part's CAN FD data-rate rating covers the bitrate you'll actually run.

### 3. Use the DRV8316's built-in buck regulator again
- Remove the AP63205 buck (U8), its inductor (L2) and its passives, and make 5 V with the DRV8316's own buck instead, as BLDC-Driver did.
- Mk2 disables the DRV8316 buck (`BUCK_DIS`) and fits the R_BK/C_BK parts that requires. Replace those with the buck inductor and output capacitor.
- Check that the buck's current limit covers the LED, the MCU, the encoder, the CAN transceiver and the optional sensors when they're fitted.

### 4. One WS2812B instead of two
- Remove one of the two WS2812B-2020 LEDs (D3, D5).

## Candidates (to evaluate)

These are other cost or part-count savings found in the Mk2 BOM. None are decided yet.

- **Cheaper encoder:** the AS5047P (U6) is one of the most expensive parts. The MT6835 is out of stock, so the MT6701 is the main alternative. Check its resolution, noise and interface against FOC needs.
- **Drop the INA226 power monitor (U13, R17):** a VIN resistor divider into an ADC pin (as on BLDC-Driver) gives battery voltage for almost nothing. Only keep the INA226 if total current or power is actually used.
- **Assembly cost:** reduce the number of JLCPCB extended parts (each adds a setup fee) by preferring basic-library parts for passives, LEDs and the LDO. The encoder has to stay on the bottom, but moving everything else to the top would shrink the bottom-side placement to a handful of parts.

## Unchanged

- The BME280 (U3) and LSM6DSV16X IMU (IC1) stay on the board as optional parts. They're normally left unpopulated, so they don't add to the usual build cost.

## Mk2 issues to fix

_Add problems found on Rev 01 here._
