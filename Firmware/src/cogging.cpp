#include "cogging.h"

#include <Arduino.h>
#include <EEPROM.h> // buffered eeprom_* API (stm32_eeprom.h)
#include <cmath>
#include <cstring>

#include "motor_config.h"
#include "motor_control.h"
#include "board_settings.h"
#include "hw/encoder.h"

namespace Cogging {
namespace {

constexpr float TWO_PI_F = 6.28318530718f;
constexpr float BIN_PER_RAD = COG_BINS / TWO_PI_F;

// ---- Stored table --------------------------------------------------------

constexpr uint32_t COG_MAGIC   = 0x54474F43; // "COGT"
constexpr uint16_t COG_VERSION = 1;

enum HeaderFlag : uint8_t { HDR_REFINED = 1 << 0 };

struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t bins;
    float    scale;       // A per table count
    int8_t   sensorDir;   // SimpleFOC sensor_direction it was measured with
    uint8_t  coverage;    // %
    uint8_t  quality;     // 0-100
    uint8_t  flags;       // HeaderFlag
    float    amplitude;   // peak-to-peak, A
    float    friction;    // A
    uint32_t crc;         // CRC-32 of the header before this field, then the table
};
static_assert(sizeof(Header) == 28, "unexpected padding in the cogging header");
static_assert(COG_TABLE_OFFSET + sizeof(Header) + COG_BINS <= E2END + 1,
              "cogging table does not fit in the settings page");

int8_t table[COG_BINS];
void computeLow(); // (below)
Header hdr;            // describes table[] when valid
bool   valid   = false;
bool   unsaved = false;

uint32_t crcUpdate(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

uint32_t blockCrc(const Header &h, const int8_t *t) {
    uint32_t crc = 0xFFFFFFFFu;
    crc = crcUpdate(crc, reinterpret_cast<const uint8_t *>(&h), offsetof(Header, crc));
    crc = crcUpdate(crc, reinterpret_cast<const uint8_t *>(t), COG_BINS);
    return ~crc;
}

// Table value at a raw mechanical angle [A]: Catmull-Rom between the bin
// centres (much closer to the real curve than straight lines at ~7 bins per
// cogging period).
float lookup(float raw) {
    float f = raw * BIN_PER_RAD - 0.5f;
    if (f < 0) f += COG_BINS;          // raw is 0..2pi, plus at most the lead
    else if (f >= COG_BINS) f -= COG_BINS;
    int i = (int)f;
    if (i < 0) { i = 0; f = 0; }
    const float t = f - (float)i;
    if (i >= COG_BINS) i = 0;
    const float p0 = table[(i + COG_BINS - 1) % COG_BINS];
    const float p1 = table[i];
    const float p2 = table[(i + 1) % COG_BINS];
    const float p3 = table[(i + 2) % COG_BINS];
    const float v = p1 + 0.5f * t * (p2 - p0 + t * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3
                                              + t * (3.0f * (p1 - p2) + p3 - p0)));
    return v * hdr.scale;
}

int binOf(float raw) {
    int b = (int)(raw * BIN_PER_RAD);
    if (b < 0) b = 0;
    if (b >= COG_BINS) b = COG_BINS - 1;
    return b;
}

// ---- Calibration ---------------------------------------------------------

// Per-bin sums of the position loop's current (units of 0.1 mA) and sample
// counts, and the forward pass's means (NO_DATA where a bin got none). After
// the backward pass sum[] holds its means. Both kept after a calibration for
// `cog dump`. (~15 KB of RAM, all of it here.)
int32_t sum[COG_BINS];
uint8_t cnt[COG_BINS];
int16_t fwdMean[COG_BINS];
constexpr int16_t NO_DATA = INT16_MIN;
bool    calData = false; // fwdMean / sum hold a finished calibration

State   st = State::Idle;
Failure fail = Failure::None;
bool    refining = false;
float   kp = COG_CAL_KP, kd = COG_CAL_KD, speed = COG_CAL_SPEED;
float   ref = 0;           // position set point, motor frame [rad]
float   passStart = 0;
float   travelled = 0;     // in the current pass [rad]
int8_t  passDir = 1;
uint32_t phaseStartUs = 0;
uint32_t lastUs = 0;
float   maxErr = 0;
float   savedVelTf = 0;
float   savedI = 0;
bool    overridesOn = false;
float   lastComp = 0;

constexpr float PASS_LENGTH  = TWO_PI_F + 0.15f; // a full turn and a bit, so every bin is crossed
constexpr float SKIP_LENGTH  = 0.03f;            // not recorded at the start of a pass (settling into the sweep)
constexpr uint32_t SETTLE_US = 400'000;

void applyOverrides() {
    savedVelTf = motor.LPF_velocity.Tf;
    savedI     = motor.PID_current_q.I;
    motor.LPF_velocity.Tf = COG_CAL_VELOCITY_TF;
    if (motor.PID_current_q.I < COG_CAL_CURRENT_I) {
        motor.PID_current_q.I = COG_CAL_CURRENT_I;
        motor.PID_current_d.I = COG_CAL_CURRENT_I;
    }
    overridesOn = true;
}

void restoreOverrides() {
    if (!overridesOn) return;
    motor.LPF_velocity.Tf = savedVelTf;
    // The current loop from the settings it was running with (the pending
    // ones; a change made during the calibration is kept too).
    applyCurrentLoopSettings(pendingSettings());
    overridesOn = false;
}

const char *failureName(Failure f) {
    switch (f) {
        case Failure::NotRunning:  return "FOC not running";
        case Failure::HostCommand: return "host sent a current";
        case Failure::Stalled:     return "rotor didn't follow (wheel blocked?)";
        case Failure::Aborted:     return "aborted";
        case Failure::Coverage:    return "angles missed";
        case Failure::NoTable:     return "no table to refine";
        default:                   return "-";
    }
}

void finishFailed(Failure why) {
    restoreOverrides();
    st = State::Failed;
    fail = why;
    Serial.print("Cogging calibration failed: ");
    Serial.println(failureName(why));
}

void clearAccumulators() {
    memset(sum, 0, sizeof(sum));
    memset(cnt, 0, sizeof(cnt));
}

void startPass(int8_t dir) {
    passDir = dir;
    passStart = ref;
    travelled = 0;
    clearAccumulators();
    st = dir > 0 ? State::Forward : State::Backward;
}

// Fills bins without data from their neighbours (circular, linear).
// have(i) says whether bin i has data.
template <typename T, typename Have>
void fillGaps(T *v, Have have) {
    int first = -1;
    for (int i = 0; i < COG_BINS; i++) if (have(i)) { first = i; break; }
    if (first < 0) return;
    int prev = first;
    for (int k = 1; k <= COG_BINS; k++) {
        const int i = (first + k) % COG_BINS;
        if (!have(i)) continue;
        const int gap = (i - prev + COG_BINS) % COG_BINS;
        for (int j = 1; j < (gap ? gap : COG_BINS); j++) {
            const int at = (prev + j) % COG_BINS;
            const float a = (float)v[prev], b = (float)v[i];
            v[at] = (T)lroundf(a + (b - a) * j / (gap ? gap : COG_BINS));
        }
        prev = i;
    }
}

// Harmonics kept in the table (see finishCalibration).
constexpr int COG_NK = COG_HARMONICS + 2;
uint16_t harmonic(int n) { return n < COG_HARMONICS ? n + 1 : (n == COG_HARMONICS ? 264 : 528); }
float coefC[COG_NK], coefS[COG_NK]; // A
float lastNoise = 0;                 // rms of what the harmonics left out [A]

// Table value at bin i from the kept harmonics [A].
float synth(int i) {
    float c1, s1;
    _sincos(TWO_PI_F * i / COG_BINS, &s1, &c1);
    float c = c1, sn = s1, v = 0;
    for (int n = 0; n < COG_HARMONICS; n++) { // k = n + 1
        v += coefC[n] * c + coefS[n] * sn;
        const float nc = c * c1 - sn * s1;
        sn = sn * c1 + c * s1;
        c = nc;
    }
    for (int n = COG_HARMONICS; n < COG_NK; n++) {
        const uint32_t k = harmonic(n);
        float sa, ca;
        _sincos(TWO_PI_F * (float)((k * (uint32_t)i) % COG_BINS) / COG_BINS, &sa, &ca); // exact phase
        v += coefC[n] * ca + coefS[n] * sa;
    }
    return v;
}

void finishCalibration() {
    restoreOverrides();

    // sum[] -> backward means.
    int both = 0;
    for (int i = 0; i < COG_BINS; i++) {
        if (cnt[i]) sum[i] /= (int32_t)cnt[i];
        if (cnt[i] && fwdMean[i] != NO_DATA) both++;
    }
    const uint8_t coverage = (uint8_t)((both * 100 + COG_BINS / 2) / COG_BINS);
    if (both < COG_BINS * 9 / 10) { finishFailed(Failure::Coverage); return; }
    // Bins without data in a pass are filled in from their neighbours.
    fillGaps(sum, [](int i) { return cnt[i] != 0; });
    fillGaps(fwdMean, [](int i) { return fwdMean[i] != NO_DATA; });
    calData = true;

    // Cogging = mean of the directions, friction = half the difference.
    int32_t tf = 0, tb = 0; // sums, 0.1 mA (at most 1920 x 32767: fits)
    for (int i = 0; i < COG_BINS; i++) { tf += fwdMean[i]; tb += sum[i]; }
    const float mf = (float)tf / COG_BINS, mb = (float)tb / COG_BINS;
    const float friction = (mf - mb) * 0.5e-4f;

    // The table keeps only the harmonics (per turn) that are repeatable:
    // 1..COG_HARMONICS, plus the slot/pole cogging frequency of a 24-slot
    // 22-pole motor (264) and its double. What is left in each bin besides
    // those is mostly measurement noise (~20 mA per bin and pass on the
    // GM3506s), which the compensation would otherwise play back as fake
    // cogging. Fourier coefficients of forward, backward and (refining) the
    // old table in one pass per harmonic; the table is rebuilt from them.
    // ~25 ms of number crunching, once, with the rotor standing still.
    const float oldUnits = refining ? hdr.scale * 1e4f : 0.0f; // table count -> 0.1 mA
    float sff = 0, sbb = 0, sfb = 0;
    for (int n = 0; n < COG_NK; n++) {
        const uint32_t k = harmonic(n);
        float dc, ds, c = 1, sn = 0;
        _sincos(TWO_PI_F * k / COG_BINS, &ds, &dc); // (SimpleFOC's, on the CORDIC: no libm trig needed)
        float fr = 0, fi = 0, br = 0, bi = 0, orr = 0, oi = 0;
        for (int i = 0; i < COG_BINS; i++) {
            // Exact phase every 64 bins, so rounding can't build up.
            if ((i & 63) == 0) _sincos(TWO_PI_F * (float)((k * (uint32_t)i) % COG_BINS) / COG_BINS, &sn, &c);
            const float f = fwdMean[i], bb = sum[i], o = table[i] * oldUnits;
            fr += f * c;  fi += f * sn;
            br += bb * c; bi += bb * sn;
            orr += o * c; oi += o * sn;
            const float nc = c * dc - sn * ds;
            sn = c * ds + sn * dc;
            c = nc;
        }
        // Agreement: correlation of the forward and backward curves over
        // these harmonics (refining: of the whole cogging, what was measured
        // plus the table that was applied).
        const float ar = fr + orr, ai = fi + oi, cr = br + orr, ci = bi + oi;
        sff += ar * ar + ai * ai;
        sbb += cr * cr + ci * ci;
        sfb += ar * cr + ai * ci;
        const float toA = 2.0f / COG_BINS * 1e-4f;
        coefC[n] = (0.5f * (fr + br) + orr) * toA;
        coefS[n] = (0.5f * (fi + bi) + oi) * toA;
    }
    const float corr = (sff > 0 && sbb > 0) ? sfb / sqrtf(sff * sbb) : 0.0f;

    // Rebuild: peak first (for the scale) and how much was left out, then
    // quantise. raw() is the unfiltered table, read before table[i] is replaced.
    auto raw = [&](int i) { return 0.5f * (fwdMean[i] + sum[i] - mf - mb) * 1e-4f + table[i] * oldUnits * 1e-4f; };
    float peak = 0, rawMean = 0, noise2 = 0;
    for (int i = 0; i < COG_BINS; i++) rawMean += raw(i);
    rawMean /= COG_BINS;
    for (int i = 0; i < COG_BINS; i++) {
        const float v = synth(i);
        peak = max(peak, fabsf(v));
        const float e = raw(i) - rawMean - v;
        noise2 += e * e;
    }
    lastNoise = sqrtf(noise2 / COG_BINS);
    const float scale = peak > 1e-5f ? peak / 127.0f : 1e-5f;
    int8_t lo = 127, hi = -127;
    for (int i = 0; i < COG_BINS; i++) {
        const long q = lroundf(synth(i) / scale);
        table[i] = (int8_t)constrain(q, -127L, 127L);
        lo = min(lo, table[i]);
        hi = max(hi, table[i]);
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic     = COG_MAGIC;
    hdr.version   = COG_VERSION;
    hdr.bins      = COG_BINS;
    hdr.scale     = scale;
    hdr.sensorDir = (int8_t)motor.sensor_direction;
    hdr.coverage  = coverage;
    hdr.quality   = (uint8_t)constrain(lroundf(corr * 100.0f), 0L, 100L);
    hdr.flags     = refining ? HDR_REFINED : 0;
    hdr.amplitude = (hi - lo) * scale;
    hdr.friction  = friction;
    hdr.crc       = blockCrc(hdr, table);
    valid   = true;
    unsaved = true;
    computeLow();
    st = State::Done;
    fail = Failure::None;

    printStatus(Serial);
    Serial.println("`save` keeps it, `revert` drops it");
}

float calibrationStep() {
    const uint32_t now = micros();
    const float dt = min((now - lastUs) * 1e-6f, 0.01f);
    lastUs = now;

    // Values from the previous move() (one FOC iteration old).
    const float angle = motor.shaft_angle;
    const float vel   = motor.shaft_velocity;
    float vref = 0;

    if (st == State::Settling) {
        if (now - phaseStartUs >= SETTLE_US) {
            // Settling comes before each pass: after the forward pass go back.
            startPass(passDir);
        }
    } else {
        vref = passDir * speed;
        ref += vref * dt;
        travelled = fabsf(ref - passStart);
    }

    const float err = ref - angle;
    if (fabsf(err) > COG_CAL_STALL_RAD) { finishFailed(Failure::Stalled); return 0; }

    const float raw = boardEncoder().getMechanicalAngle();
    const float ff = refining ? lookup(raw) : 0.0f;
    float fb = kp * err + kd * (vref - vel);
    fb = constrain(fb, -COG_CAL_CURRENT_LIMIT, COG_CAL_CURRENT_LIMIT);

    if (st == State::Forward || st == State::Backward) {
        if (travelled > SKIP_LENGTH) {
            maxErr = max(maxErr, fabsf(err));
            const int b = binOf(raw);
            if (cnt[b] < 255) {
                sum[b] += lroundf(fb * 1e4f); // 0.1 mA
                cnt[b]++;
            }
        }
        if (travelled >= PASS_LENGTH) {
            if (st == State::Forward) {
                for (int i = 0; i < COG_BINS; i++) {
                    fwdMean[i] = cnt[i] ? (int16_t)constrain(sum[i] / (int32_t)cnt[i], -32767L, 32767L) : NO_DATA;
                }
                passDir = -1;
                st = State::Settling;
                phaseStartUs = now;
            } else {
                finishCalibration();
                return 0;
            }
        }
    }
    return ff + fb;
}

// The table's harmonics 1..COG_LOW_HARMONICS per turn [A], for the
// "slots only" mode, which leaves them out (computed from the table whenever
// it changes). On e4a7d0 the 2-per-turn part made the standing robot wander:
// it probably depends on the load on the bearings, which is different lying
// down (calibration) and standing.
float lowC[COG_LOW_HARMONICS], lowS[COG_LOW_HARMONICS];

void computeLow() {
    for (int k = 1; k <= COG_LOW_HARMONICS; k++) {
        float c = 0, sn = 0;
        for (int i = 0; i < COG_BINS; i++) {
            float sa, ca;
            // Bin centre, as lookup() interpolates between centres.
            _sincos(TWO_PI_F * (float)((k * (2 * i + 1)) % (2 * COG_BINS)) / (2 * COG_BINS), &sa, &ca);
            c += table[i] * ca;
            sn += table[i] * sa;
        }
        const float toA = valid ? 2.0f / COG_BINS * hdr.scale : 0.0f;
        lowC[k - 1] = c * toA;
        lowS[k - 1] = sn * toA;
    }
}

float lowPart(float raw) {
    float s1, c1;
    _sincos(raw, &s1, &c1);
    float c = c1, sn = s1, v = 0;
    for (int k = 0; k < COG_LOW_HARMONICS; k++) {
        v += lowC[k] * c + lowS[k] * sn;
        const float nc = c * c1 - sn * s1;
        sn = sn * c1 + c * s1;
        c = nc;
    }
    return v;
}

// Compensation for this iteration [A, motor frame]; mode as cogComp.
float compensation(uint8_t mode) {
    if (!tableMatchesMotor()) return 0;
    const float speedAbs = fabsf(motor.shaft_velocity);
    if (speedAbs >= COG_FADE_END) return 0;
    const float fade = speedAbs <= COG_FADE_START ? 1.0f
                     : (COG_FADE_END - speedAbs) / (COG_FADE_END - COG_FADE_START);
    // The raw angle runs with sensor_direction x the shaft angle; read the
    // table where the rotor will be when the current has followed.
    // Lead = the current loop's response time, (R + P) / I.
    const float I = motor.PID_current_q.I;
    const float lead = I > 0 ? min((MOTOR_PHASE_RESISTANCE + motor.PID_current_q.P) / I, 0.01f) : 0.0f;
    const float raw = boardEncoder().getMechanicalAngle()
                    + (float)motor.sensor_direction * motor.shaft_velocity * lead;
    float v = lookup(raw);
    if (mode == COG_MODE_SLOTS) v -= lowPart(raw);
    return fade * v;
}

// ---- Dump ----------------------------------------------------------------
int dumpAt = -1; // -1 = not dumping, -2 = header next

} // namespace

// ---- Public --------------------------------------------------------------

void begin() {
    valid = false;
    unsaved = false;
    eeprom_buffer_fill();
    Header h;
    uint8_t *dst = reinterpret_cast<uint8_t *>(&h);
    for (size_t i = 0; i < sizeof(h); i++) dst[i] = eeprom_buffered_read_byte(COG_TABLE_OFFSET + i);
    if (h.magic != COG_MAGIC || h.version != COG_VERSION || h.bins != COG_BINS) return;
    if (!std::isfinite(h.scale) || h.scale <= 0 || (h.sensorDir != 1 && h.sensorDir != -1)) return;
    // CRC straight from the buffer, so a bad block leaves the table alone.
    uint32_t crc = crcUpdate(0xFFFFFFFFu, reinterpret_cast<const uint8_t *>(&h), offsetof(Header, crc));
    for (size_t i = 0; i < COG_BINS; i++) {
        const uint8_t b = eeprom_buffered_read_byte(COG_TABLE_OFFSET + sizeof(h) + i);
        crc = crcUpdate(crc, &b, 1);
    }
    if (~crc != h.crc) return;
    for (size_t i = 0; i < COG_BINS; i++) table[i] = (int8_t)eeprom_buffered_read_byte(COG_TABLE_OFFSET + sizeof(h) + i);
    hdr = h;
    valid = true;
    computeLow();
}

bool startCalibration(bool refine, float kpArg, float kdArg, float speedArg) {
    if (calibrating()) abort();
    if (!focRunning() || !motorCalibrated()) { st = State::Failed; fail = Failure::NotRunning; return false; }
    if (refine && !tableMatchesMotor())      { st = State::Failed; fail = Failure::NoTable;    return false; }
    kp    = kpArg    > 0 ? kpArg    : COG_CAL_KP;
    kd    = kdArg    > 0 ? kdArg    : COG_CAL_KD;
    speed = speedArg > 0 ? speedArg : COG_CAL_SPEED;
    refining = refine;
    calData = false;
    ref = motor.shaft_angle;
    passDir = 1;
    travelled = 0;
    maxErr = 0;
    lastUs = micros();
    phaseStartUs = lastUs;
    st = State::Settling;
    fail = Failure::None;
    applyOverrides();
    Serial.print("Cogging calibration: kp ");
    Serial.print(kp, 1);
    Serial.print(" kd ");
    Serial.print(kd, 3);
    Serial.print(" speed ");
    Serial.print(speed, 2);
    Serial.print(", ");
    Serial.print((int)(2 * PASS_LENGTH / speed + 1));
    Serial.println(" s; keep the wheel free");
    return true;
}

void abort(Failure why) {
    if (calibrating()) finishFailed(why);
}

void clear() {
    if (calibrating()) abort();
    valid = false;
    unsaved = true;
    memset(table, 0, sizeof(table));
    computeLow();
}

void revert() {
    if (calibrating()) abort();
    begin();
}

bool calibrating() {
    return st == State::Settling || st == State::Forward || st == State::Backward;
}

void noteHostCurrent(float amps) {
    if (calibrating() && fabsf(amps) > COG_HOST_ABORT_A) finishFailed(Failure::HostCommand);
}

float update(float hostCurrent, uint8_t mode) {
    if (calibrating()) {
        if (!focRunning()) { finishFailed(Failure::NotRunning); return 0; }
        lastComp = 0;
        return calibrationStep();
    }
    lastComp = mode ? compensation(mode) : 0.0f;
    return hostCurrent + lastComp;
}

float lastCompensation() { return lastComp; }

State state() { return st; }
Failure failure() { return fail; }

uint8_t progress() {
    switch (st) {
        case State::Settling: return passDir > 0 ? 0 : 50;
        case State::Forward:  return (uint8_t)min(49.0f, 50.0f * travelled / PASS_LENGTH);
        case State::Backward: return (uint8_t)min(99.0f, 50.0f + 50.0f * travelled / PASS_LENGTH);
        case State::Done:     return 100;
        default:              return 0;
    }
}

bool tableValid() { return valid; }
bool tableUnsaved() { return unsaved; }
bool tableMatchesMotor() {
    return valid && motorCalibrated() && hdr.sensorDir == (int8_t)motor.sensor_direction;
}
bool tableRefined() { return valid && (hdr.flags & HDR_REFINED); }
float tableAmplitude() { return valid ? hdr.amplitude : 0.0f; }
float tableFriction() { return valid ? hdr.friction : 0.0f; }
uint8_t tableCoverage() { return valid ? hdr.coverage : 0; }
uint8_t tableQuality() { return valid ? hdr.quality : 0; }
float lastMaxTrackingError() { return maxErr; }
float lastFilteredNoise() { return lastNoise; }
float tableScale() { return valid ? hdr.scale : 0.0f; }
int8_t tableCount(int bin) { return (valid && bin >= 0 && bin < COG_BINS) ? table[bin] : 0; }

void writeToEepromBuffer() {
    Header h;
    if (valid) {
        h = hdr;
    } else {
        memset(&h, 0, sizeof(h)); // magic 0: no table
    }
    const uint8_t *src = reinterpret_cast<const uint8_t *>(&h);
    for (size_t i = 0; i < sizeof(h); i++) eeprom_buffered_write_byte(COG_TABLE_OFFSET + i, src[i]);
    if (valid) {
        for (size_t i = 0; i < COG_BINS; i++)
            eeprom_buffered_write_byte(COG_TABLE_OFFSET + sizeof(h) + i, (uint8_t)table[i]);
    }
}

bool verifyEepromBuffer() {
    Header h;
    uint8_t *dst = reinterpret_cast<uint8_t *>(&h);
    for (size_t i = 0; i < sizeof(h); i++) dst[i] = eeprom_buffered_read_byte(COG_TABLE_OFFSET + i);
    if (!valid) return h.magic != COG_MAGIC;
    if (memcmp(&h, &hdr, sizeof(h)) != 0) return false;
    for (size_t i = 0; i < COG_BINS; i++)
        if ((int8_t)eeprom_buffered_read_byte(COG_TABLE_OFFSET + sizeof(h) + i) != table[i]) return false;
    unsaved = false;
    return true;
}

void printStatus(Stream &out) {
    out.print("Cogging: ");
    if (valid) {
        out.print(hdr.amplitude * 1000.0f, 0);
        out.print(" mA p-p, friction ");
        out.print(hdr.friction * 1000.0f, 1);
        out.print(" mA, coverage ");
        out.print(hdr.coverage);
        out.print("%, agreement ");
        out.print(hdr.quality);
        out.print('%');
        if (lastNoise > 0) { out.print(", noise left out "); out.print(lastNoise * 1000.0f, 1); out.print(" mA"); }
        if (hdr.flags & HDR_REFINED) out.print(", refined");
        if (motorCalibrated() && !tableMatchesMotor()) out.print(" [other sensor direction: not used]");
    } else {
        out.print("no table");
    }
    if (unsaved) out.print(" [unsaved]");
    out.print("; compensation ");
    static const char *const modes[] = { "off", "on", "slots only (low harmonics left out)" };
    out.println(modes[pendingSettings().cogComp <= 2 ? pendingSettings().cogComp : 0]);
    if (st == State::Idle) return;
    out.print("  calibration: ");
    static const char *const names[] = { "", "settling", "forward", "backward", "done", "failed: " };
    out.print(names[(int)st]);
    if (st == State::Failed) out.print(failureName(fail));
    out.print(", ");
    out.print(progress());
    out.print("%, max position error ");
    out.print(maxErr * 1000.0f, 1);
    out.println(" mrad");
}

void startDump() { dumpAt = -2; }

void serviceDump(Stream &out) {
    if (dumpAt == -1) return;
    // A few lines per call, and only while the USB buffer has room, so the
    // FOC loop (and the watchdog) keep running.
    for (int lines = 0; lines < 4 && dumpAt != -1; lines++) {
        if (out.availableForWrite() < 48) return;
        if (dumpAt == -2) {
            out.println(calData ? "bin,deg,table_mA,fwd_mA,bwd_mA" : "bin,deg,table_mA");
            dumpAt = 0;
            continue;
        }
        const int i = dumpAt;
        out.print(i);
        out.print(',');
        out.print((i + 0.5f) * 360.0f / COG_BINS, 3);
        out.print(',');
        out.print(valid ? table[i] * hdr.scale * 1000.0f : 0.0f, 2);
        if (calData) {
            out.print(',');
            out.print(fwdMean[i] * 0.1f, 1);
            out.print(',');
            out.print(sum[i] * 0.1f, 1);
        }
        out.println();
        if (++dumpAt >= COG_BINS) { dumpAt = -1; out.println("# end"); }
    }
}

} // namespace Cogging
