#pragma once

#include <cstdint>

class Stream;

// Anti-cogging: a table of the current needed to hold the rotor against its
// cogging torque, against the encoder's raw mechanical angle, measured once
// per board and added to the commanded current while the motor runs slowly.
//
// Calibration (FOC running, wheel free to turn): the board turns the rotor
// slowly with its own stiff position loop, one revolution forwards and one
// back, and averages the current that loop needed in each of COG_BINS angle
// bins. The mean of the two directions is the cogging (friction pushes the
// opposite way in each, so it cancels); half their difference is the
// friction, which is reported. The table is zero-mean (a constant offset
// isn't cogging). A "refine" calibration runs with the existing table
// already applied and adds what is left over to it.
//
// The current loop is set to at least COG_CAL_CURRENT_I during a calibration
// (the position loop needs it to respond within a couple of ms) and the
// velocity filter to COG_CAL_VELOCITY_TF; both go back afterwards.
//
// Storage: in the settings flash page, after the board settings
// (COG_TABLE_OFFSET), written by saveBoardSettings(). It belongs to the
// encoder magnet and motor it was measured on: recalibrate after changing
// either. It's only applied while FOC runs with the same sensor direction it
// was measured with.
//
// The table keeps only the repeatable part: harmonics 1..COG_HARMONICS per
// turn plus 264 and 528 (slot/pole cogging); the rest of each bin's value is
// measurement noise. (The GM3506s measured so far cog mostly at 2, 22-26
// and 42-48 per turn, hardly at all at 264.)
//
// Compensation is switched by BoardSettings::cogComp (a tuning setting:
// applies at once, kept by save): off, on, or "slots only", which leaves out
// harmonics 1..COG_LOW_HARMONICS per turn (on e4a7d0 the 2-per-turn part made
// the standing robot wander; it probably depends on the bearing load, which
// differs between lying down and standing). The table is read ahead of the rotor by
// the current loop's response time, (MOTOR_PHASE_RESISTANCE + P) / I, and its effect
// fades out between COG_FADE_START and COG_FADE_END rad/s: above that the
// cogging is too fast for the current loop to follow anyway, and the
// wheel's inertia smooths it out.
// BoardSettings::cogComp / PARAM_COG_COMP values.
constexpr uint8_t COG_MODE_OFF   = 0;
constexpr uint8_t COG_MODE_FULL  = 1;
constexpr uint8_t COG_MODE_SLOTS = 2; // without harmonics 1..COG_LOW_HARMONICS per turn

namespace Cogging {

enum class State : uint8_t {
    Idle     = 0, // no calibration since boot
    Settling = 1, // holding the start position (and between the passes)
    Forward  = 2, // first revolution
    Backward = 3, // second revolution, the other way
    Done     = 4, // the last calibration succeeded (table unsaved until `save`)
    Failed   = 5, // the last calibration failed, see failure()
};

enum class Failure : uint8_t {
    None        = 0,
    NotRunning  = 1, // FOC isn't running / stopped during the calibration
    HostCommand = 2, // the host sent a non-zero current: someone wants to drive
    Stalled     = 3, // the rotor didn't follow (wheel blocked?)
    Aborted     = 4, // stopped by command
    Coverage    = 5, // too many angle bins without data in one of the passes
    NoTable     = 6, // refine without a valid table for this motor
};

// Loads the table from the settings page. Call once at boot, after
// loadBoardSettings().
void begin();

// Starts a calibration (refine: on top of the existing table). FOC must be
// running. kp [A/rad], kd [A/(rad/s)] and speed [rad/s] replace the defaults
// when > 0. Returns false (and sets failure()) if it can't start.
bool startCalibration(bool refine, float kp = 0, float kd = 0, float speed = 0);
void abort(Failure why = Failure::Aborted);
// Forgets the table (until saved: revert() brings the stored one back).
void clear();
// Reloads the stored table, discarding a calibration or clear not yet saved.
void revert();

bool calibrating();

// Called with every current command from the host (amps, before inversion).
// A non-zero command during a calibration aborts it.
void noteHostCurrent(float amps);

// The current to command this FOC iteration (motor frame, amps), given the
// host's. While calibrating: the calibration's position loop (the host's is
// ignored). Otherwise the host's plus the compensation (mode: COG_MODE_*,
// BoardSettings::cogComp). Call
// between motor.loopFOC() and motor.move().
float update(float hostCurrent, uint8_t mode);

// The compensation added by the last update() [A], 0 when off.
float lastCompensation();

// ---- Status -------------------------------------------------------------
State state();
Failure failure();
uint8_t progress();           // 0-100 during a calibration, 100 when done
bool tableValid();
bool tableUnsaved();          // calibrated / cleared since the last load or save
bool tableMatchesMotor();     // valid, and measured with this boot's sensor direction
bool tableRefined();          // the table's last calibration was a refine
float tableAmplitude();       // peak-to-peak [A]
float tableFriction();        // friction measured with it [A]
uint8_t tableCoverage();      // % of bins that had data in both directions
uint8_t tableQuality();       // forward/backward agreement, 0-100 (see printStatus)
float lastMaxTrackingError(); // largest position error in the last calibration [rad]
float lastFilteredNoise();    // rms of the per-bin noise the last calibration left out of the table [A]
float tableScale();           // A per table count (0 = no table)
int8_t tableCount(int bin);   // raw table value of a bin (x tableScale() = A)

// ---- Storage (used by saveBoardSettings) --------------------------------
// Copies the table block into the EEPROM emulation buffer (the settings
// page's RAM copy), or marks it empty when there's no table.
void writeToEepromBuffer();
// Checks the buffer (just filled from flash) holds what was written.
bool verifyEepromBuffer();

// ---- USB serial ---------------------------------------------------------
void printStatus(Stream &out);
// Starts printing the table (and the last calibration's per-direction data)
// as CSV; serviceDump() prints a few lines per call so FOC keeps running.
void startDump();
void serviceDump(Stream &out);

} // namespace Cogging

// Byte offset of the table block in the settings page, and the number of bins.
constexpr uint16_t COG_TABLE_OFFSET = 64;
constexpr uint16_t COG_BINS = 1920; // 0.1875 degrees each, ~7 per cogging period of a 24-slot 22-pole motor
