#pragma once

// Non-blocking USB serial command console (line based, LF or CR terminated).
// Type `help` for the command list. Call pollSerialConsole() once per loop().
//
//   config                    show active (running) and pending settings + state
//   set side left|right       choose which motor this board drives (CAN IDs)
//   set invert on|off         flip command/telemetry sign for this motor
//   cal store                 keep this boot's FOC alignment (FOC must be running)
//   cal clear                 forget the stored alignment
//   save                      write pending settings to flash, then reboot
//   revert                    discard unsaved changes
//   start                     start FOC from the console (bench use)
//   reboot                    reset the board
//   dfu                       reboot into the USB DFU bootloader
//
// `set`/`cal` only change the pending settings; nothing takes effect until
// `save`, which stops the motor (a flash erase stalls the CPU), writes,
// verifies and reboots.
void pollSerialConsole();

// Prints active/pending settings and runtime state.
void printConfig();
