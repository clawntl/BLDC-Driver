# MiniFOC Mk2: a round, motor-mounted FOC driver

MiniFOC is a compact brushless motor driver built around an **STM32G431** microcontroller and a **TI DRV8316C** three-phase driver with integrated MOSFETs. It's designed to bolt straight onto the back of a gimbal motor or small direct-drive wheel, with the encoder IC sitting over the motor's diametric magnet.

It's the successor to [BLDC-Driver](https://github.com/clawntl/BLDC-Driver). The main changes are a round 40 mm form factor, inline phase-current sensing, an on-board IMU, power monitoring and an XT30 power input. Mk2 isn't perfect, and a Mk3 is planned.

| Top | Bottom |
|---|---|
| ![MiniFOC Mk2 top](Images/render_top.png) | ![MiniFOC Mk2 bottom](Images/render_bottom.png) |

## Features

| | |
|---|---|
| **MCU** | STM32G431CBT6 (Cortex-M4F, 170 MHz, LQFP-48) with a 16 MHz crystal |
| **Power stage** | DRV8316C with integrated FETs (up to 8 A peak) |
| **Phase current sense** | Inline 3 mΩ shunts on all three phases with INA241A3 amplifiers (50 V/V, bidirectional, about ±10 A range) |
| **Supply** | 5–35 V through an XT30 connector. An AP63205 buck converter makes 5 V and an XC6206 LDO makes 3.3 V. An LM66200 ideal-diode OR lets the board run from USB with no battery attached |
| **Power monitor** | INA226 on a 3 mΩ shunt (bus voltage, current and power over I²C, address `0x40`) |
| **Position sensor** | AS5047P 14-bit magnetic encoder, centred on the bottom of the board |
| **IMU** | LSM6DSV16X 6-axis IMU |
| **Environment** | BME280 temperature/humidity/pressure sensor (I²C, address `0x76`) |
| **Communication** | CAN FD via an ATA6561 transceiver on a JST-GH 3-pin connector, with solder-jumper 120 Ω termination |
| **Other I/O** | USB-C, SWD pads, SPI3 header (JST-SH 6-pin), I²C header (JST-SH 4-pin), BOOT and RST buttons, two WS2812B-2020 RGB LEDs, status LEDs |
| **Board** | 40 mm diameter, 4 layers, 4 × M2.5 mounting holes on a 24 mm square (34 mm PCD) |

## Repository layout

| Path | Contents |
|---|---|
| [`Hardware/`](Hardware) | KiCad project: schematic, PCB, custom symbol/footprint library and 3D models |
| [`Hardware/production/`](Hardware/production) | Fabrication outputs: Gerber zip, BOM and pick-and-place files (JLCPCB/LCSC part numbers) |
| [`schematic.pdf`](schematic.pdf) | PDF export of the schematic |
| [`Images/`](Images) | Board renders |
| [`Firmware/`](Firmware) | PlatformIO firmware: the CAN motor driver (`driver`) and a bench-test build (`motor_test`) |

## Firmware

The firmware is ported from [BLDC-Driver](https://github.com/engbuildspace/BLDC-Driver/tree/main/Firmware) firmware **1.2.0** (CAN protocol 3) and uses the same CAN protocol, serial console and board settings, so the docs there ([firmware guide](https://github.com/engbuildspace/BLDC-Driver/blob/main/docs/firmware.md), [CAN protocol](https://github.com/engbuildspace/BLDC-Driver/blob/main/docs/can-protocol.md), [test routines](https://github.com/engbuildspace/BLDC-Driver/blob/main/docs/test-routines.md)) still apply. A MiniFOC and a BLDC-Driver can share a bus and run with the same controller firmware. That covers:

- **Node IDs 1-15:** currents in frame `0x010 + (n-1)/2` (NaN = no command for that node), commands on `0x020 + n`, telemetry on `0x100 + n`.
- **Configuration over CAN:** firmware info, settings, node ID, invert, alignment store/clear, save, revert and reboot.
- **Supply voltage** in the status heartbeat (mV).

The differences on Mk2 are:

- **Pins:** new mapping in [`Firmware/include/pins.h`](Firmware/include/pins.h). The user LED is on PB2.
- **Encoder:** AS5047P instead of AS5048A.
- **Current sense:** SimpleFOC `InlineCurrentSense` (3 mΩ shunts, INA241A3 at 50 V/V) instead of the DRV8316's low-side amplifiers.
- **Supply voltage:** read from the INA226 over I²C instead of the V_SENS ADC divider, both at boot (PWM scaling) and every 100 ms for the CAN heartbeat. The INA226 does its own averaging, so there is no extra filter. The `config` console command also shows the live supply voltage and current.
- **DRV8316 buck:** switched off, because 5 V comes from the AP63205.
- **CAN transceiver:** its VIO pin is powered from PC13, so the firmware turns it on at boot.
- **No DFU button GPIO:** BOOT is wired straight to BOOT0/CAN_RX. To enter DFU, hold BOOT and press RST, or type `dfu` in the serial console.

Build and flash from `Firmware/`:

```bash
pio run -e driver -t upload
```

Then configure the board over USB serial (`set node 1-15`, or `set side left|right` for node 1/2, then `save`), the same as on BLDC-Driver. A board saved as left/right by older firmware comes up as node 1/2.

The CAN bench test tools are in [`Firmware/tools/can_test`](Firmware/tools/can_test) (`minifoc_cantest.py`, and a GUI that drives two nodes at once or monitors a bus with the robot's controller on it).

## Known issues

This is hardware revision **Rev 01** (Mk2). Known problems will be listed here and fixed in Mk3. See [MK3.md](MK3.md) for planned Mk3 changes.

## License

This project is shared under the terms of the [GNU Affero General Public License Version 3](LICENSE) as published by the Free Software Foundation.

The bundled [`ACANFD_STM32_G431`](Firmware/lib/ACANFD_STM32_G431) library is a stripped-down fork of Pierre Molinaro's [ACANFD-STM32](https://github.com/pierremolinaro/acanfd-stm32) and keeps its own MIT license.

The CAN test tool bundles ElmüSoft's Candlelight Python classes in [`Firmware/tools/can_test/elmue`](Firmware/tools/can_test/elmue), unmodified, under their own BSD 3-Clause license.
