# Hardware guide (v0.2)

The design files are in [`Hardware/`](../Hardware) (KiCad). A PDF of the schematic is at [`schematic_v0.2.pdf`](../schematic_v0.2.pdf).

## Block overview

```
 VIN 5-35 V ──┬── DRV8316 ── phases A/B/C ──► motor
              │     │  ├─ buck 5 V ──► XC6206 LDO 3.3 V ──► MCU, encoder, CAN
              │     │  └─ CSA outputs SO_A/B/C ──► PA0/PA1/PA2 (ADC)
              │     └─ SPI1 + 6× PWM ◄── STM32G431CBU6 ──► FDCAN1 ──► TJA1051T/3 ──► CAN H/L
              └── 100k/10k divider ──► PA3 (V_SENS)          │
                                       AS5048A (SPI1) ◄───────┤
                                       USB-C (CDC / DFU) ◄────┘
```

## Pin mapping

The source of truth is [`Firmware/include/pins.h`](../Firmware/include/pins.h).

### Power stage (DRV8316)

| Signal | MCU pin | Notes |
|---|---|---|
| Phase A high / low | PA10 / PB15 | TIM1 6-PWM |
| Phase B high / low | PA9 / PB14 | |
| Phase C high / low | PA8 / PB13 | |
| Current sense A / B / C | PA0 / PA1 / PA2 | DRV8316 low-side CSA outputs, ADC1 |
| DRV8316 SPI | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS | SPI1, shared with the encoder |

### Encoder (AS5048A)

| Signal | MCU pin |
|---|---|
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 |
| Encoder CS | PC4 |

Both SPI1 devices share MISO. The firmware drives **both** chip selects high before any SPI traffic, so neither device fights the bus. If you attach an external sensor through the SPI1 header, keep this in mind.

### CAN

| Signal | MCU pin | Notes |
|---|---|---|
| CAN RX | PB8 | This pin is also **BOOT0**. The BOOT button pulls it during reset |
| CAN TX | PB9 | |
| CAN enable | PC10 | Must be driven **high** to enable the TJA1051 transceiver |

### Miscellaneous

| Function | MCU pin | Notes |
|---|---|---|
| Supply sense (V_SENS) | PA3 | VIN → 100 kΩ → PA3 → 10 kΩ → GND, so V_PA3 = VIN / 11 |
| User LED | PC6 | Status LED, see [LED patterns](firmware.md#status-led) |
| User button | PB12 | Active low. The firmware uses it to reboot into DFU |
| Reset | PG10 (NRST) | RST button |
| SWD | PA13 SWDIO, PA14 SWCLK | 2×2 1.27 mm header (3V3, GND, SWDIO, SWCLK) |
| SPI3 header | PB3 SCK, PB4 MISO, PB5 MOSI, PA15 NSS | Spare interface. The firmware doesn't use it |

## Connectors

| Connector | Description |
|---|---|
| Power input | 2-pin, 5–35 V DC and GND |
| Motor phases | A, B, C pads |
| CAN (×2) | JST-PH 2.0 mm 3-pin (CAN L, GND, CAN H), wired in parallel for daisy-chaining |
| USB-C | USB 2.0 full speed: CDC serial console and DFU bootloader |
| SWD | ST-Link programming and debugging |
| AS5048A / SPI1 header | CS, SCK, MISO, MOSI, GND, 3V3. Use it to connect the encoder after breaking off the tab, or to connect another SPI sensor |
| SPI3 header | NSS, SCK, MISO, MOSI, GND |
| Power header | 3V3, GND |

## Jumpers and buttons

- **CAN termination.** Fit **both** jumpers to enable the split 120 Ω termination (2 × 60.4 Ω with a centre-tap capacitor to GND). Enable termination only on the two nodes at the physical ends of the bus.
- **BOOT.** Hold the button while resetting or powering up to start the STM32 ROM USB DFU bootloader.
- **RST.** Resets the MCU.
- **USER (PB12).** In the shipped firmware, a press reboots into DFU.

## Electrical notes

- **Current rating.** The DRV8316 is rated for 8 A peak, but the practical continuous current is about 3 A and depends on cooling. The firmware limits phase current to `MOTOR_CURRENT_LIMIT` (2 A by default).
- **Over-current protection.** The firmware sets the DRV8316 OCP to 16 A with automatic retry after 500 ms.
- **Current-sense gain.** The firmware uses 0.6 V/A. The DRV8316 datasheet's original gain table was wrong: register codes 0–3 actually give 0.15 / 0.3 / 0.6 / 1.2 V/A. As a result, the SimpleFOCDrivers enum `Gain_0V25` really selects **0.6 V/A**.
- **Supply measurement.** VIN is measured once at boot. The ADC uses a long sample time because the divider has a high source impedance and no filter capacitor. If you run the board from USB only, the reading falls outside the 6–30 V sanity window, and the firmware falls back to `MOTOR_VOLTAGE_POWER_SUPPLY` (12 V).
- **Clock.** The board uses a 16 MHz HSE crystal. The MCU runs at 160 MHz, which requires boost mode. USB runs from HSI48 with CRS. See [`clock_config.cpp`](../Firmware/src/clock_config.cpp).

## Manufacturing

[`Hardware/production/`](../Hardware/production) contains files ready for JLCPCB:

- `v0.2_Custom_Motor_Driver_DRV8316_STM32G4.zip`: Gerbers and drill files
- `bom.csv`: bill of materials with LCSC part numbers
- `positions.csv`: pick-and-place (CPL)
- `designators.csv`, `netlist.ipc`

These files were generated with the KiCad *Fabrication Toolkit* plugin (see `fabrication-toolkit-options.json`). The connectors (power terminal, pin headers) may need to be fitted by hand.
