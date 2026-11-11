# BLDC-Driver: a compact FOC driver board

BLDC-Driver is an integrated brushless motor driver built around an **STM32G431** microcontroller and a **TI DRV8316** three-phase gate driver with integrated MOSFETs. It runs field-oriented control (FOC) with [SimpleFOC](https://github.com/simplefoc) and takes current (torque) commands over CAN. It suits gimbal motors and small direct-drive wheels.

The board includes an optional **AS5048A** magnetic encoder on a break-away tab. With the tab removed, the board measures just **39 mm × 36 mm**.

![BLDC-Driver Pinout Diagram](Images/BLDC%20Driver%20Pinout%20Diagram.png "Pinout Diagram")

## Features

| | |
|---|---|
| **MCU** | STM32G431CBU6 (Cortex-M4F, 170 MHz max, run at 160 MHz; CORDIC used for fast trig) |
| **Power stage** | DRV8316 with integrated FETs: 3 A continuous / 8 A peak, 6-PWM control over SPI |
| **Supply** | 5–35 V DC. The DRV8316 buck converter generates 5 V, and an XC6206 LDO generates 3.3 V |
| **Current sense** | DRV8316 integrated low-side current-sense amplifiers on all three phases |
| **Position sensor** | AS5048A 14-bit magnetic encoder (SPI1) on a break-away tab, or an external sensor via the SPI1 header |
| **Communication** | CAN via a TJA1051T/3 transceiver (FDCAN capable up to 5 Mbit/s; the firmware uses 1 Mbit/s classic CAN). Two JST-PH connectors allow daisy-chaining |
| **CAN termination** | Optional split 120 Ω termination, set with two jumpers |
| **Other I/O** | USB-C (CDC serial console + DFU flashing), SWD header, SPI3 header, user button, user LED, power LED |
| **Supply sensing** | 100 kΩ / 10 kΩ divider on VIN, read by the MCU (PA3) |
| **Mounting** | 4 × M2 holes, 33 mm × 30 mm spacing |

## Repository layout

| Path | Contents |
|---|---|
| [`Hardware/`](Hardware) | KiCad project for the v0.2 board (schematic, PCB, custom footprints and symbols) |
| [`Hardware/production/`](Hardware/production) | Fabrication outputs: Gerber zip, BOM and pick-and-place files (JLCPCB/LCSC part numbers) |
| [`schematic_v0.2.pdf`](schematic_v0.2.pdf) | PDF export of the schematic |
| [`Firmware/`](Firmware) | PlatformIO firmware: the CAN motor driver and a bench-test build |
| [`Images/`](Images) | Pinout diagram |
| [`docs/`](docs) | Detailed documentation (see below) |

## Documentation

- **[Hardware guide](docs/hardware.md)**: pin mapping, connectors, jumpers, buttons, LEDs and manufacturing notes.
- **[Firmware guide](docs/firmware.md)**: building, flashing, first-time configuration, the serial console, tuning parameters and the bench-test firmware.
- **[Test routines](docs/test-routines.md)**: step-by-step bench testing of a board, motor and encoder with the `motor_test` firmware, with expected readings and troubleshooting.
- **[CAN protocol](docs/can-protocol.md)**: message IDs, frame formats, status and error codes, plus an example host program.

## Quick start

1. **Install [PlatformIO](https://platformio.org/)** (the VS Code extension or the CLI).
2. **Put the board in DFU mode.** Hold **BOOT**, plug in USB-C (or tap **RST**), then release **BOOT**. On a board that's already running the firmware, you can type `dfu` in the serial console instead.
3. **Build and flash** from the `Firmware/` directory:
   ```bash
   cd Firmware
   ```
   ```bash
   pio run -e driver -t upload
   ```
4. **Configure the board** over USB serial. The board boots unconfigured, with CAN disabled:
   ```bash
   pio device monitor
   ```
   Then type:
   ```text
   set side left      # or: set side right  (selects the CAN IDs)
   set invert off     # optional: flip the sign of commands/telemetry for a mirrored motor
   save               # writes to flash and reboots
   ```
5. **Drive it over CAN** at 1 Mbit/s:
   - Send `0x10` to the board's command ID (`0x021` left / `0x022` right) to align and start FOC.
   - Wait for the `[0x10][1.0f]` reply.
   - Stream target currents in amps as two big-endian floats on ID `0x020` at least every 100 ms.

   See [docs/can-protocol.md](docs/can-protocol.md) for the full protocol.

> [!CAUTION]
> Test new motors with the wheels off the ground and a current-limited supply. The `motor_test` bench firmware ([docs/firmware.md](docs/firmware.md#bench-test-firmware-motor_test)) checks the encoder, power stage and current sense step by step before you run closed-loop control.

## Status

This is hardware revision **v0.2**. The firmware's default motor parameters (11 pole pairs, 2 A current limit, 10 V voltage limit) live in [`Firmware/include/motor_config.h`](Firmware/include/motor_config.h). Change them to match your motor.

## License

This project is shared under the terms of the [GNU Affero General Public License Version 3](https://choosealicense.com/licenses/agpl-3.0/) as published by the Free Software Foundation.

The bundled [`ACANFD_STM32_G431`](Firmware/lib/ACANFD_STM32_G431) library is a stripped-down fork of Pierre Molinaro's [ACANFD-STM32](https://github.com/pierremolinaro/acanfd-stm32) and keeps its own MIT license.
