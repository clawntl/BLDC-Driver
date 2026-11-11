# Test routines (`motor_test` firmware)

A step-by-step procedure for testing a BLDC-Driver board together with its motor and encoder, using the `motor_test` bench firmware. It's written so that someone new to the board can follow it, and for each step it lists the commands, the expected readings and motion, and what to look at if a reading is out of range.

Work through the steps **in order**. Each one relies on the ones before it, so if a step fails, fix it before you move on. You can copy the [record sheet](#record-sheet) at the end and fill it in for each board.

> [!NOTE]
> **Reference values.** The expected readings come from a v0.2 board running a 12 V supply with an 11-pole-pair gimbal-style motor (5.3 Ω between motor wires) and the on-board AS5048A. Values that depend on the motor are marked, and [Appendix A](#appendix-a-choosing-test-voltages) explains how to adapt them to a different motor. Readings that depend only on the board, such as current-sense offsets and DRV8316 flags, should match on any board.

For building, flashing and configuring the `driver` firmware, see the [Firmware guide](firmware.md). For pins, connectors and buttons, see the [Hardware guide](hardware.md).

---

## 0. What you need

| Item | Notes |
|---|---|
| BLDC-Driver board | With the motor connected to the A/B/C pads and the encoder fitted, either on its tab or on the SPI1 header |
| Motor | Mounted so its shaft can turn freely. Remove or secure anything attached to it |
| Power supply | 5–35 V DC. A **current-limited bench supply** (limit 1–2 A) is safest for first power-up |
| USB-C cable | A data cable, not a charge-only one |
| Computer | VS Code + PlatformIO, with this repository open |
| Serial monitor | `pio device monitor`, or any serial terminal at 115200 baud |
| Multimeter | Resistance and DC volts |
| Magnet | 6 mm diameter, **diametrically magnetised** NdFeB (e.g. 6 × 2.5 mm or 6 × 3 mm, N35) |
| Marker pen or tape flag | For counting turns of the shaft or of whatever it drives |
| Glue | For fixing the magnet once it's set up (step 5) |

### Safety
- **Keep the motor free to spin.** Make sure anything attached to it (a wheel, arm or gear) can turn without hitting anything or anyone. For a wheel, lift it clear of the ground.
- Open-loop tests (`ol`, `rev`, `hold`) push current through the motor **even when it isn't turning**. Keep to the voltages given here. Every motor-on test stops by itself after 30 s.
- **To stop the motor at any time,** press **Enter** in the serial monitor, or type `x` and press Enter.
- If anything smells hot, buzzes loudly, or the motor or the board becomes hot to the touch, stop and investigate.

---

## 1. Check the motor before connecting

Check the motor first, with the board **unpowered** and the motor wires **disconnected** from the A/B/C pads:

1. Measure the resistance between each pair of motor wires: A–B, B–C and A–C.

| Expected | Notes |
|---|---|
| All three pairs read the same, within ~5% | Reference motor: **5.3 Ω**. The phase resistance R is half the wire-to-wire value, so **2.65 Ω** for the reference motor |

| If… | Likely cause |
|---|---|
| One pair reads open (very high or OL) | A broken motor wire, or a bad crimp or solder joint |
| One pair reads much lower than the others | A shorted winding. Replace the motor |
| The values differ from the reference motor but agree with each other | It's a different motor. Note the value, and work out your test voltages from [Appendix A](#appendix-a-choosing-test-voltages) |
| The readings drift | Poor probe contact. The meter leads alone add about 0.1–0.3 Ω |

Once the readings are good, reconnect the motor.

---

## 2. Flash the test firmware

1. Connect USB. The motor supply can stay off for now.
2. Put the board in **DFU mode** using whichever of these applies:
   - **Firmware already running** (`motor_test` or `driver`): type `dfu` in the serial console. With `driver` running, you can press the **USER** button instead.
   - **Blank or unresponsive board:** hold **BOOT**, tap **RST**, then release **BOOT**.
3. From the `Firmware/` directory, run:
   ```bash
   pio run -e motor_test -t upload
   ```
4. Open the serial monitor at **115200** baud:
   ```bash
   pio device monitor -e motor_test
   ```
5. The monitor shows `BLDC test firmware ready - press Enter to start` every 2 s. **Press Enter.**

| If… | Likely cause |
|---|---|
| Upload says "No DFU capable USB device" | The board isn't in DFU mode. Retry step 2. To check, run `system_profiler SPUSBDataType \| grep -i -A6 DFU` (macOS) or `lsusb` (Linux). On macOS, if the board is listed but the upload still fails, try `brew install libusb` |
| No serial port appears | A charge-only USB cable, or the board is still in DFU mode. Press **RST** |
| A port appears but nothing prints | Wrong baud rate. Press Enter anyway, because the prompt only repeats every 2 s |

After you press Enter, the firmware prints the command list, the supply voltage and an encoder snapshot.

> [!NOTE]
> The firmware accepts CR, LF or CRLF line endings. Type commands in lower case, with any argument after a space, e.g. `v 1.5`.

The saved `driver` settings (side, invert, stored alignment) are kept while `motor_test` is flashed. Flashing over DFU doesn't touch the settings page.

---

## 3. Supply voltage: `vin`

1. Turn the motor supply **on**.
2. Type `vin`.

| Expected | Notes |
|---|---|
| `VIN =` within ~2% of a multimeter reading across the power input | e.g. a reported 11.73 V against a measured 11.83 V is fine |

| If… | Likely cause |
|---|---|
| VIN ≈ 0–1 V | The supply is off or not connected, and only USB is powering the board |
| VIN is off by more than 3% | Tolerance in the divider resistors (R16/R17) or the 3.3 V regulator. Set `VSENS_CALIBRATION` in `Firmware/include/motor_config.h` to *meter ÷ reported*, then reflash |
| `vin` says the reading is "cached" | You've already run `cs`. Type `reset`, reconnect the monitor, press Enter, and try again |

---

## 4. Encoder communication: `s` (no magnet)

Do this step **before** fitting the magnet, or with the magnet removed. It proves that the encoder and its SPI wiring work, and that the warnings you'll rely on in step 5 can be trusted.

Type `s`.

| Expected with **no magnet** |
|---|
| `parityErr=0 cmdInvalid=0 framingErr=0` |
| `compHigh=1`, `AGC=255`, a small `magnitude` (below ~100). `OCF` may read 0 |
| Warning lines saying the field is too weak |

| If… | Likely cause |
|---|---|
| `!! All zeros` | The encoder has no power, MISO is shorted to GND, or the encoder CS line (PC4) isn't reaching the chip |
| `!! All ones` | MISO isn't connected (floating), the CS pin is wrong, or an external encoder isn't plugged in |
| Parity or framing errors | A poor connection, or an external encoder cable that's too long or noisy for SPI |
| It reports the field as OK with no magnet fitted | A stray magnet nearby, such as the motor's own magnets, a tool or a screw |

---

## 5. Fit and centre the magnet: `s`, `n`, `w`

1. Fit the magnet on the end of the motor shaft, centred over the AS5048A chip. The usual gap between magnet and chip is **0.5–2 mm**.
2. Type `s`.

| Expected | Notes |
|---|---|
| `OCF=1 COF=0 compLow=0 compHigh=0` | The field is in range |
| `AGC` somewhere between ~**20 and ~200** | Reference: 50–53. The line ends `OK: field in range, AGC has headroom both ways` |
| `magnitude` ≈ 4500–4800 | |

3. **Centring check.** Turn the shaft by hand to about **8 positions** around one full turn, running `s` at each one, and note the AGC value each time.

| Expected | Notes |
|---|---|
| AGC stays roughly constant around the turn, within ±5–10 | Reference: 50–53 all the way round |

| If… | Likely cause |
|---|---|
| `compHigh=1`, or AGC at or near 255 | The field is too **weak**: the gap is too large, the magnet is too small, or it isn't diametrically magnetised (see below) |
| `compLow=1`, or AGC at or near 0 | The field is too **strong**: the magnet is too close, so increase the gap |
| AGC rises and falls once per turn (e.g. between 45 and 80) | The magnet is **off-centre or tilted**, or the shaft wobbles. Reseat the magnet |
| The angle barely changes as you turn the shaft | The magnet is **axially** magnetised, with its poles on the flat faces. It must be **diametric**, with the poles on the curved sides. To test: an axial disc snaps flat onto steel, while a diametric one prefers to sit on its edge |

4. **Noise check.** Hold the shaft still and type `n`.

| Expected |
|---|
| `spread_counts` of about 10 or less (reference: 7, i.e. 0.15°) |
| `glitches=0`, `errFlags=0`, `rawStuck=0`, followed by `OK` |

| If… | Likely cause |
|---|---|
| A spread above 20 counts | A marginal field (check the AGC), or electrical noise on the encoder wiring |
| `glitches` or `errFlags` above 0 | An SPI signal problem: a loose connector, a long cable or a poor ground |

5. **Tracking check.** Type `w`, then slowly turn the shaft **2–3 full turns one way, then back again**. Press Enter to stop.

| Expected |
|---|
| `total_rad` changes smoothly, by about **6.28 per turn** |
| `rotations` counts up or down once per turn |
| `total_rad` returns to roughly its starting value after you turn it back |
| `glitches=0`, and `maxJump_rad` stays small (below 0.01) |

| If… | Likely cause |
|---|---|
| Jumps or glitches | The magnet is off-centre, the field is marginal, or there's noise on SPI |
| The count drifts after turning back and forth | Missed readings. Run `n` again |

6. **Glue the magnet** once all of the above pass. A magnet held only by its own pull can creep round the shaft, and that shifts the zero angle found in step 11.

---

## 6. DRV8316: `drv`

Type `drv`. This configures the DRV8316 and **enables the power stage** at 0 V, so the motor won't move.

| Expected |
|---|
| `fault=0`, with every `OC_*`, `buck*`, `chargePumpUV`, `otpErr` and `spi*` flag at 0 |
| `pwmMode=0` (6-PWM) |
| `driver.voltage_power_supply` ≈ your VIN, and `driver.voltage_limit=10.00` (or VIN, if that's lower) |
| Earlier in the output: `STM32-DRV: best: TIM1-CH3 TIM1-CH3N TIM1-CH2 TIM1-CH2N TIM1-CH1 TIM1-CH1N` and `Synchronising 1 timers` |

| If… | Likely cause |
|---|---|
| `powerOnReset=1` on the first read only | Normal after power-up, and cleared automatically. Run `drv` again and it should read 0 |
| `spiErr`, `spiAddr`, `spiClock` or `spiParity` = 1 | SPI to the DRV8316 isn't reliable: check the soldering on the DRV8316 or the shared SPI1 lines. **Stop here**, because no later result can be trusted |
| `chargePumpUV=1` or `buckUV=1` | The supply is too low or missing, or there's a fault in the charge-pump or buck components (C5, C6, L1) |
| Any `OC_*` = 1 | Overcurrent tripped: a shorted motor wire, a motor phase shorted to its frame, or a damaged output stage |
| `overTemp=1` | The driver is overheating. Check the cooling and look for a short |
| `voltage_power_supply` ≈ 12.00, with a warning | The VIN measurement failed or was out of range. Go back to step 3 |
| A timer other than TIM1 is chosen | The pin mapping in `Firmware/include/pins.h` is wrong |

---

## 7. Current sense at rest: `cs`

Type `cs`.

| Expected | Notes |
|---|---|
| `offA_V`, `offB_V` and `offC_V` each **1.62–1.68 V** | Half the 3.3 V reference. Reference: 1.651 / 1.661 / 1.657 |
| `gainA = gainB = gainC = 1.667` | 1 ÷ 0.6 V/A, fixed in the firmware |
| `Ia_A`, `Ib_A` and `Ic_A` within **±0.05 A** | Reference: 0.003 / 0.000 / −0.001 |
| Earlier in the output: `STM32-CS: Using ADC: 1` and `Using timer: 1` | |

| If… | Likely cause |
|---|---|
| One offset ≈ 0 V or ≈ 3.3 V | That SOx line or its ADC pin (PA0/PA1/PA2) is open or shorted. Check R12/R14/R15 and C18–C20 |
| All the offsets are far from 1.65 V | The DRV8316 VREF pin (pin 37) isn't at 3.3 V |
| Currents above 0.1 A at rest | The offset calibration was disturbed, either by the motor moving during `cs` or by noise on VREF/3.3 V. Type `reset` and repeat with the shaft still |
| `current_sense.init() failed` | An ADC/timer synchronisation problem, usually caused by a firmware or library change. Report it with the full log |

---

## 8. Current sense under load: `hold`

`hold` locks the motor's field onto phase A. The rotor will **twitch and lock**, then stay still with current flowing through the windings.

1. Choose the test voltage from [Appendix A](#appendix-a-choosing-test-voltages). For the reference motor, type `v 1.5`.
2. Type `hold`. Let it print about 10 lines, then press Enter.

| Expected | Notes |
|---|---|
| `Ia` ≈ (v − 0.3) ÷ R | Reference: +0.44 A at 1.5 V. It's below the simple v ÷ R = 0.57 A because about 0.3 V is lost to switching dead time |
| `Ib` ≈ `Ic` ≈ **−Ia ÷ 2** | Reference: −0.21 / −0.19 |
| `sum` within ±0.06 A | Ideally 0 |
| `SOA_V` ≈ 1.65 + 0.6 × Ia; `SOB_V` and `SOC_V` ≈ 1.65 − 0.3 × Ia | Reference: 1.91 / 1.53 / 1.54 V |
| Readings steady from line to line | |

| If… | Likely cause |
|---|---|
| Ia ≈ 0 and the rotor didn't twitch | Phase A isn't connected, or the supply is off |
| Ia ≈ 0 but the rotor twitched | A fault in the phase A current-sense path (see step 7) |
| Ib or Ic has the wrong sign, or reads ≈ 0 | A fault in that phase's wire or its current-sense channel |
| Much more current than expected (e.g. more than 1.5 × the expected value) | The motor's resistance is lower than you assumed. Recheck step 1 and lower `v` |
| `sum` large (above 0.1 A) | One channel's gain or offset is off. Compare the three `SO*_V` values |

**Optional linearity check.** Run `hold` at three or four voltages, e.g. `hold 1.0`, `hold 2.0` and `hold 2.5`, for a few lines each. Ia should rise by about **1 ÷ R amps per volt** (about 0.38 A/V for the reference motor). If the rise per volt is clearly different, the current-sense gain is off by that factor. Keep each run short, so that the winding doesn't warm up and change its resistance.

---

## 9. Open-loop spin: `ol`

Open loop drives the motor **without using the encoder**. It checks the DRV8316, all six output transistors, the motor wiring and the pole-pair count.

1. Put a mark on the shaft or on whatever it drives. Keep the test voltage from step 8.
2. Type `ol 3`. It runs for 30 s and then stops by itself.

| Expected | Notes |
|---|---|
| The motor turns **continuously**, at about **one turn every 2.1 s**, so **just over 14 turns** in the 30 s | 3 rad/s × 30 s ÷ 2π = 14.3 turns. This holds for any motor if `MOTOR_POLE_PAIRS` is right |
| A slight pulsing or lumpiness you can feel | **Normal** for open loop: it's the motor's cogging, with no feedback to smooth it out |
| `Ia`, `Ib` and `Ic` all swing to the same peak | The phase order cycles through all three |
| `ratio` ≈ ±1.00 | ±0.9 to ±1.1 is fine, and the sign doesn't matter. Only meaningful with the magnet fitted |
| `loop_Hz` ≈ 30,000–34,000 | |

3. Type `ol -3`. It does the same thing in the **opposite** direction.
4. **Note which way a positive command turns the motor**, as seen from a reference side such as the shaft end. A positive open-loop command turns the motor the same way as a positive current command in the `driver` firmware. You need this when you decide on the `set invert` setting (see the [Firmware guide](firmware.md#first-time-setup)): set it so that positive commands move your mechanism the way your controller expects.

| If… | Likely cause |
|---|---|
| It buzzes or vibrates but doesn't turn | A motor phase is open, or `v` is too low for this motor. Try 0.5 V more |
| It turns in jerks, with a hard knock at the same point every turn | One phase is weak or missing. Compare the Ia/Ib/Ic peaks |
| One of Ia/Ib/Ic stays near 0 | That motor wire or its output stage is dead |
| Clearly not ~14 turns (e.g. ~22 or ~9) | `MOTOR_POLE_PAIRS` in `Firmware/include/motor_config.h` is wrong for this motor |
| `ratio` is steady but not ±1 (e.g. ±1.57) | The same: the pole-pair count is wrong |
| `ratio` is erratic or ≈ 0 while the motor turns | An encoder problem. Go back to step 5 |
| The motor gets hot | `v` is too high for this motor. See [Appendix A](#appendix-a-choosing-test-voltages) |

---

## 10. Single revolution: `rev`

1. Line your mark up with a fixed reference point.
2. Type `rev`. The rotor locks for 0.8 s, then turns slowly through one revolution, which takes about 6 s.
3. Type `rev -1`.

| Expected |
|---|
| The motor turns **exactly one full turn** each way and stops on the mark |
| `ratio` ≈ ±1.00, and `estimated_pp` equals your motor's pole-pair count (11 for the reference motor) |
| `direction=` gives the same answer for both runs |

| If… | Likely cause |
|---|---|
| It turns noticeably less than one turn, with jerks | The rotor is slipping. Raise `v` by 0.5 V |
| It turns a clearly wrong amount (e.g. 1.5 turns) | The pole-pair count is wrong (see step 9) |
| It turns once, but `ratio` ≈ 0 | The encoder isn't seeing the rotor: the magnet is loose on the shaft, or step 5 failed |
| `Stopped: aborted by serial input` straight away | Your firmware is older than the fix for this. Reflash `motor_test` |

---

## 11. Encoder alignment: `foc`

`foc` runs SimpleFOC's start-up calibration. The motor will **jolt and turn a little each way**, which is expected.

1. Make sure you've run `cs` (step 7) since the last reset, so that the current-sense check is included.
2. Type `foc` **three times**.

| Expected (every run) | Notes |
|---|---|
| `MOT:PP check: OK!` | The encoder confirms the pole-pair count |
| `MOT:Align current sense.` followed by `MOT:Success: 1` | 1 means the phases and current-sense channels already match |
| `success=1` | |
| The same `sensor_direction` on every run | Either sign is fine, as long as it doesn't change. Reference: −1 (CCW) |
| `zero_electric_angle` within **±0.05 rad** across the three runs | Its value depends on how the magnet was fitted. Reference: 3.864 / 3.873 / 3.877 |

| If… | Likely cause |
|---|---|
| `PP check: fail` | The pole-pair count is wrong, or the encoder isn't reading the rotor (a loose magnet or a marginal field) |
| `sensor_direction` changes between runs | The magnet is slipping, or the alignment voltage is too low to move the rotor reliably. Raise `v`: alignment uses 2 × v, up to a maximum of 3 V |
| `zero_electric_angle` varies by more than 0.1 rad | The magnet is loose or slipping, the field is marginal, or friction is stopping the rotor from settling. Recheck step 5 |
| `MOT:Success: 2`, `3` or `4` | The current-sense channels didn't match the phases (2 = channels swapped, 3 = a sign inverted, 4 = both). SimpleFOC corrected this for the current session, but check the wiring and `pins.h` |
| `MOT:Success: 0`, or `Align error!` | A current-sense channel isn't seeing any current. Go back to step 8 |
| `success=0` | Read the `MOT:` lines above it to see which stage failed |
| With nothing changed, the zero angle differs from an earlier session by more than ~0.3 rad | The magnet is creeping on the shaft. Glue it (step 5.6) |

> [!WARNING]
> If this board uses a stored alignment in the `driver` firmware (`cal store`), any change of magnet, encoder or motor means you must run `cal clear` and then `save` in `driver`. See the [Firmware guide](firmware.md#storing-the-foc-alignment-optional).

---

## 12. Closed-loop spin: `t`

`t` runs full FOC with the encoder, in voltage mode.

1. Type `t 1`. Watch it for a few seconds, then let it time out or press Enter.
2. Type `t -1`.

| Expected | Notes |
|---|---|
| The motor spins **smoothly**, noticeably more smoothly than with `ol` | |
| `sensor_rad_s` is steady, varying by less than ~2% | The speed depends on the motor and the load. Reference, with an unloaded wheel at 12 V: ±13.5 rad/s (≈ 130 rpm) |
| The speed is the same in both directions, within a few percent | This shows the alignment is accurate |
| When `sensor_direction=-1`, the sign of `sensor_rad_s` is opposite to the sign of `t` | Normal: it's the raw encoder reading |

| If… | Likely cause |
|---|---|
| `t 0.5` doesn't move it, or only makes it twitch | **Normal** at low voltage: most of the voltage is lost to dead time, and cogging and friction win. Use `t 1` or more |
| It doesn't move at `t 1` | The load is binding, so turn it by hand to check. Otherwise, run `foc` again |
| It buzzes, runs roughly, or runs in one direction only | Bad alignment. Run `foc` again and check that the zero angle is consistent |
| It runs away fast in one direction, whatever the sign | `sensor_direction` is wrong, which shouldn't happen after a good `foc`. Run `foc` again |
| One direction is much faster than the other (more than 10%) | The zero angle is off. Run `foc` again and check the magnet |
| The speed surges or oscillates | The magnet is loose or the field is marginal. Recheck step 5 |

---

## When you've finished

1. Type `dfu`, then flash the `driver` firmware from the `Firmware/` directory:
   ```bash
   pio run -e driver -t upload
   ```
2. If this is a new board, configure it as described in [Firmware guide: First-time setup](firmware.md#first-time-setup). Use your note from step 9.4 to choose the `invert` setting.
3. Fill in the record sheet and keep it with the board.

---

## Command reference (`motor_test`)

| Command | Motor | What it does |
|---|---|---|
| `help` | — | Lists the commands |
| `info` | — | Reprints the help, VIN and an encoder snapshot |
| `s` | off | Encoder snapshot: angle, AGC, magnitude, field and error flags |
| `n [N]` | off | Noise test: N reads (default 2000) with the shaft still |
| `w` | off | Shows the live angle while you turn the shaft by hand |
| `vin` | off | Measures the supply voltage (only before `cs`) |
| `drv` | enabled, 0 V | Configures the DRV8316 and shows its fault status |
| `v <volts>` | — | Sets the test voltage (default 1.0, maximum 4.0) |
| `cs` | enabled, 0 V | Sets up current sensing; shows offsets and currents at rest |
| `hold [volts]` | **on**, locked | Locks the field on phase A and streams the DC currents |
| `ol <rad/s>` | **on**, spinning | Open-loop spin, without the encoder |
| `rev [n]` | **on**, n turns | Turns n revolutions open loop; checks pole pairs and direction |
| `foc` | **on**, jolts | Runs SimpleFOC encoder (and current-sense) alignment |
| `t <volts>` | **on**, spinning | Closed-loop voltage torque (after `foc`) |
| `x` or Enter | stops | Stops and disables the motor |
| `reset` | — | Resets the MCU (reconnect the monitor afterwards) |
| `dfu` | — | Reboots into the USB bootloader for flashing |

Every motor-on mode stops automatically after 30 s.

---

## Appendix A: choosing test voltages

Pick the test voltage `v` so that the current is about **0.5 A**. With R as the phase resistance (half of the wire-to-wire reading from step 1):

> `v` ≈ 0.5 × R + 0.3  (the 0.3 V allows for dead time)

| Wire-to-wire resistance | R (phase) | Suggested `v` |
|---|---|---|
| 2 Ω | 1.0 Ω | 0.8 V |
| 5.3 Ω | 2.65 Ω | 1.5 V |
| 10 Ω | 5 Ω | 2.8 V |

The current-sense range is about ±2.75 A, so keep test currents well under 2 A. The firmware caps `v` at 4 V.

---

## Appendix B: background

- **AGC:** the AS5048A adjusts its own gain (AGC, 0–255) to suit the magnet's field. A value near 255 means it's working as hard as it can, because the field is weak. A value near 0 means the field is very strong. A mid-range value that barely changes around a turn means a good, centred magnet.
- **Why `t 0.5` stalls:** SimpleFOC's default dead time of 2% costs about 0.24 V per phase at 12 V, which leaves very little torque from a 0.5 V command. The `driver` firmware uses **current** control, which corrects for this automatically.
- **Current-sense gain:** the DRV8316 register code used here (`Gain_0V25` in SimpleFOCDrivers) actually selects **0.6 V/A**. See the [Hardware guide](hardware.md#electrical-notes). The `hold` test confirms the scaling.
- **Zero electrical angle:** where the magnet sits relative to the motor's magnetic poles. It's fixed by how the magnet was fitted, so it only changes if the magnet moves. That's why a stored alignment must be cleared whenever the magnet, encoder or motor is refitted.
- **Open loop vs `driver` direction:** both turn the motor in the direction the electrical angle increases, so a positive `ol` and a positive current command in `driver` turn the motor the same way. `set invert on` in `driver` reverses the sign of both the commands and the reported velocity.

---

## Record sheet

```
Board ID: ______________   Motor: ______________   Date: __________   Tester: ________

1  Motor resistance   A–B ____ Ω   B–C ____ Ω   A–C ____ Ω                              [ ] OK
3  VIN                reported ____ V   meter ____ V                                     [ ] OK
4  s (no magnet)      compHigh=1, no SPI errors                                          [ ] OK
5  s (magnet)         AGC at 8 positions: ___ ___ ___ ___ ___ ___ ___ ___                 [ ] OK
   n                  spread ____ counts   glitches ____                                 [ ] OK
   w                  ~6.28/turn, no glitches                                            [ ] OK
   Magnet glued                                                                          [ ] Done
6  drv                all flags 0, pwmMode=0                                             [ ] OK
7  cs                 offsets A ____ B ____ C ____ V   rest currents < 0.05 A            [ ] OK
8  hold ____ V        Ia ____  Ib ____  Ic ____  sum ____ A                              [ ] OK
9  ol 3 / ol -3       turns in 30 s ____ / ____   positive = CW / CCW from ________      [ ] OK
10 rev / rev -1       one turn each way, est_pp ____                                     [ ] OK
11 foc x3             dir ____  ____  ____   zero angle ______  ______  ______  CS ____  [ ] OK
12 t 1 / t -1         speed ______ / ______ rad/s   smooth Y/N                           [ ] OK
   driver flashed     configured: side ______  invert ______                             [ ] Done
Notes:
```
