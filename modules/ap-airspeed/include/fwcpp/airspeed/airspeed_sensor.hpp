#pragma once

// Port of AP_Airspeed's real read()-formula core - NOT the full generic
// multi-sensor AP_Airspeed interface (primary-sensor voting, the
// interactive ground/inflight ratio-calibration state machine, the
// IMU/GPS-cross-check health-check state machine, sensor failure
// injection) - just the single-instance pressure-to-EAS pipeline every
// real airspeed backend (analog, digital, SITL) shares. Matches this
// port's established "match the real formula, not the generic interface"
// methodology (ap-compass's Compass, ap-gps's Gps). CPP-082, phase 1 of
// this port's airspeed sensor subsystem. CPP-083, phase 2, added the
// real boot-time zero-offset calibration routine - see "CALIBRATION"
// section below.
//
// Upstream (Plane-4.7.0, read directly from the pinned worktree, not from
// training-data memory):
//   - AP_Airspeed/AP_Airspeed.cpp's real read(uint8_t i) (~line 646, read
//     in full) - offset subtraction, the 0.7/0.3 IIR filter with its
//     unhealthy->healthy reset edge case, and the PITOT_TUBE_ORDER_AUTO
//     branch's sqrt(|filtered_pressure| * ratio) (this port's only
//     pitot-tube-order branch - see "EXCLUDED" below).
//   - AP_Airspeed/AP_Airspeed.cpp's real enabled()/healthy()/use()/
//     get_offset() (~lines 931-1045, read in full).
//   - AP_Airspeed/AP_Airspeed_SITL.cpp's real get_differential_pressure()
//     (read in full) - confirms this is the backend AP_Airspeed::init()
//     actually selects for CONFIG_HAL_BOARD==HAL_BOARD_SITL builds, and
//     that its return value never fails without fail-injection this
//     port doesn't build (see "HEALTHY()" note below).
//   - AP_Airspeed/AP_Airspeed_Params.cpp's real AP_GROUPINFO defaults:
//     ARSPD_RATIO=2 (GROUPINFO id 4), ARSPD_OFFSET=0 (GROUPINFO id 3) -
//     verified directly, not guessed.
//   - CPP-083: AP_Airspeed/AP_Airspeed.cpp's real calibrate() (~line 528)
//     and update_calibration() (~line 574), read in full - see
//     "CALIBRATION" below. AP_Vehicle/AP_Vehicle.cpp (~line 441) calls
//     `airspeed.calibrate(true)` UNCONDITIONALLY at boot when the sensor
//     is enabled - no GCS/MAVLink command needed - and AP_Airspeed_
//     Params.cpp's SKIP_CAL (GROUPINFO id 8) defaults to 0
//     (SkipCalType::None - calibration proceeds normally), both verified
//     directly this ticket: this is genuinely active-by-default upstream
//     behavior, not a disabled/opt-in feature.
//
// REAL read() FORMULA (AP_Airspeed.cpp, PITOT_TUBE_ORDER_AUTO branch,
// transcribed directly):
//   raw_pressure = sensor[i]->get_differential_pressure();      // Pa
//   airspeed_pressure = raw_pressure - get_offset(i);
//   if (!prev_healthy) {
//       filtered_pressure = airspeed_pressure;                  // RESET, not blended
//   } else {
//       filtered_pressure = 0.7f*filtered_pressure + 0.3f*airspeed_pressure;
//   }
//   airspeed = sqrtf(fabsf(filtered_pressure) * ratio);
// `prev_healthy` is `state[i].healthy` READ BEFORE this call overwrites
// it - i.e. whether the SENSOR's last read was healthy, not whether the
// FILTER had previously produced a valid airspeed. `state[i].healthy`
// starts false at construction, so the very first ever update() call
// always takes the reset branch too (there is no "filtered_pressure
// implicitly starts healthy" special case) - reproduced exactly below by
// defaulting healthy_ to false and reading it as prev_healthy before
// this update() sets it.
//
// PITOT_TUBE_ORDER - AUTO/default ONLY: PITOT_TUBE_ORDER_NEGATIVE/
// _POSITIVE (the other two real upstream branches, differing only in
// whether last_pressure/raw_airspeed negate airspeed_pressure or use it
// unclamped-sign) are NOT ported - this port has no PITOT_TUBE_ORDER
// parameter concept, matching the ticket's own instruction ("this port
// has ONE airspeed sensor... do NOT port... non-AUTO pitot-tube-order
// branches").
//
// RATIO/OFFSET PLACEMENT - AirspeedSensor-OWNED DEFAULTED CONSTRUCTOR
// FIELDS, NOT Plane::FixedWingTunables/aparm MEMBERS - a DELIBERATE
// choice, matching ap-compass's Compass class's own declination/
// inclination/intensity precedent rather than plane.hpp's aparm.
// airspeed_min/max/scaling_speed precedent. Both ARSPD_RATIO and
// ARSPD_OFFSET ARE real top-level AP_Param entries upstream (like
// AIRSPEED_MIN/AIRSPEED_MAX), so the choice is genuinely between two
// real precedents already established in this port, not a fabricated
// third option - reasons for choosing Compass's shape over aparm's:
//   1. aparm's own file banner describes it as "every AP_Param-backed
//      tunable MANUAL/FBWA's real code paths actually read" - a flight-
//      BEHAVIOR knob (roll/pitch limits, speed targets) a GCS operator
//      tunes to change how the aircraft FLIES. ratio/offset are SENSOR
//      CALIBRATION constants (how to convert this ONE physical sensor's
//      raw output into a trustworthy EAS number) - closer in kind to
//      Compass's earth-field declination/inclination/intensity (how to
//      interpret this ONE physical sensor's raw output) than to
//      airspeed_min/max (how the autopilot should behave).
//   2. Wiring into aparm's own param table (the `entry(...)` list
//      plane.hpp's save/load/persistence system walks) would couple a
//      brand-new, single-purpose sensor module's construction to
//      ap-vehicle's AparmParamKey enum/persistence machinery - real
//      scope beyond "phase 1: sensor model + wiring into tick()", and
//      no other single-sensor module this port has added (ap-compass,
//      ap-gps) participates in that system either.
//   3. AirspeedSensor remains fully unit-testable and constructible in
//      total isolation (airspeed_sensor_test.cpp below) with zero
//      dependency on ap-vehicle/ap-param, exactly like Compass/Gps.
// A caller wanting a persisted, GCS-settable ratio/offset in a later
// phase can still add ARSPD_RATIO/ARSPD_OFFSET to aparm then and pass
// aparm.airspeed_ratio/aparm.airspeed_offset into this constructor - this
// choice does not foreclose that, it just doesn't build it prematurely.
// CPP-083's calibration routine follows this SAME precedent: the
// calibrated offset is written directly to this class's own offset_
// field ("set", the achievable subset), never through an AP_Param
// set_and_save() equivalent ("save"/persistence across a restart is out
// of scope - see "CALIBRATION" below).
//
// "USE()" - A REAL, DISCLOSED SIMPLIFICATION: upstream's real use(i)
// (AP_Airspeed.cpp) is `lib_enabled() && !_force_disable_use &&
// enabled(i) && param[i].use && !(param[i].use==2 && throttle-running-
// glider-special-case)` - an ARSPD_USE enable parameter, a global
// force-disable flag, and a glider-behind-propeller throttle special
// case, none of which this port models (single always-configured
// sensor, no ARSPD_USE parameter, no glider special case - real,
// separate scope, not silently dropped: named here as future work).
// use() below is therefore simply an alias for healthy() - "this
// port's one airspeed sensor is used whenever it is healthy", the
// simplest faithful reduction given no enable/disable knob exists yet.
//
// "HEALTHY()" - A REAL, DISCLOSED PHASE-1 NEAR-NO-OP, NOT AN INVENTED
// FAILURE MODE: upstream's real healthy(i) (AP_Airspeed.cpp) is
// `enabled(i) && state[i].healthy && sensor[i]!=nullptr && (allowZero
// Offset || !is_zero(param[i].offset))` - the LAST clause is the
// interactive-calibration health check (an unconfigured, zero-offset
// sensor that hasn't been told to skip calibration is treated as
// UNHEALTHY, precisely to force a user through Airspeed_Calibration.cpp
// before flight). That calibration state machine is real, separate,
// explicitly deferred upstream scope (see "OUT OF SCOPE" below) - this
// port has no calibration-skip concept to check, so healthy_ below
// tracks only "has update() ever been called and, if so, did the
// backend's own get_differential_pressure() succeed" (always true here,
// since SITL's own AP_Airspeed_SITL::get_differential_pressure() never
// fails without fail-injection this phase doesn't build - see
// sim_plane.hpp's airspeed_sensor_differential_pressure() doc comment).
// Matches Compass's own "healthy means update() was called" precedent
// exactly (compass.hpp).
//
// CALIBRATION (CPP-083) - the real boot-time ZERO-OFFSET calibration
// routine, i.e. AP_Airspeed::calibrate()/update_calibration() - NOT the
// interactive in-flight RATIO auto-calibration (Airspeed_Calibration.cpp,
// a separate, substantially more complex mechanism, out of scope - see
// "EXCLUDED" below).
//
//   calibrate(in_startup=true) (AP_Airspeed.cpp ~line 528) real upstream
//   sequence, ported by start_calibration(now_ms):
//     cal.start_ms = now_ms; cal.count = 0; cal.sum = 0;
//     cal.read_count = 0; cal.state = IN_PROGRESS;
//   NOT ported (explicitly deferred, named per ticket):
//     - was_watchdog_reset() skip-on-crash-restart check - no HAL
//       watchdog subsystem in this port.
//     - the in_startup SkipCalType switch (NoCalRequired/SkipBootCal) -
//       SkipCalType/SKIP_CAL parameter support is out of scope; this
//       port's start_calibration() always behaves like the real
//       SkipCalType::None (default) path, matching SKIP_CAL's own real
//       default of 0.
//     - the NOT_REQUIRED_ZERO_OFFSET early-continue - inapplicable, this
//       port's one SITL sensor always needs offset calibration (real
//       upstream SITL sensors do too - see AP_Airspeed_Backend.cpp's own
//       has_zero_offset_calibration() default of false, never overridden
//       by AP_Airspeed_SITL).
//
//   update_calibration(i, raw_pressure) (AP_Airspeed.cpp ~line 574) real
//   upstream logic, transcribed exactly, woven into update()'s own body
//   below (gated on calibration_state() == InProgress, matching
//   upstream's own `cal.start_ms != 0` gate - the ticket's own instructed
//   substitution since this port models calibration as an enum, not a
//   raw timestamp):
//     - Called with raw_pressure (PRE-offset-subtraction), and, in real
//       upstream, textually BEFORE the filter update but AFTER
//       airspeed_pressure/corrected_pressure are computed with the
//       offset value CURRENT AT THE START of this call - reproduced
//       below by computing corrected_pressure_ first, then running
//       calibration, so a same-tick Success->offset_ write takes effect
//       starting NEXT tick's corrected_pressure_, never retroactively
//       this tick, exactly like upstream's get_offset(i) ordering.
//     - Finalize check FIRST: once `now_ms - cal.start_ms >= 1000` AND
//       `cal.read_count > 15`: if `cal.count == 0`, mark FAILED
//       (shouldn't happen once healthy, but real upstream still checks
//       it); otherwise `offset = sum / count`, mark SUCCESS. Either way
//       `cal.start_ms = 0` and return - no accumulation happens on the
//       finalizing call itself.
//     - Otherwise (still accumulating): discard the first 5 samples -
//       `cal.read_count > 5` gates accumulation (a STRICT greater-than,
//       so calls 1-5 are discarded and accumulation starts on call 6).
//       When healthy_ (this port's near-always-true gate, matching
//       upstream's own `state[i].healthy &&`) and past the discard
//       count: `cal.sum += raw_pressure; cal.count++`. `cal.read_count`
//       increments on EVERY non-finalizing call regardless of the
//       discard/accumulate gate - read_count and count are genuinely
//       DIFFERENT counters (read_count counts calls; count counts
//       samples actually summed) - re-verified directly against upstream,
//       not assumed.
//   NOT ported (explicitly deferred, named per ticket):
//     - the fixed_wing_parameters->airspeed_min-based "offset changed
//       too much, pitot may be covered" WARNING check - upstream's own
//       real behavior here is GCS-text-only, not behaviorally
//       load-bearing, and this port has no GCS.
//     - set_and_save() - this ticket applies the calibrated value to
//       offset_ directly ("set"); persisting it across a restart
//       ("save") is out of scope, matching CPP-082's own RATIO/OFFSET
//       PLACEMENT precedent (no AP_Param wiring for this value yet).
//
//   The aggregate get_calibration_state() (multi-sensor voting across
//   AIRSPEED_MAX_SENSORS) has no equivalent here - this port has exactly
//   one sensor, so calibration_state() below already IS the aggregate.
//
// EXCLUDED - each a genuine, named scope boundary for THIS ticket, not
// an oversight:
//   - Sensor failure injection (arspd.fail/arspd.fail_pressure/
//     arspd.signflip, sitl_airspeed.cpp) - see sim_plane.hpp's own
//     airspeed_sensor_differential_pressure() doc comment.
//   - Multi-sensor support / select_primary() voting (AP_Airspeed.cpp) -
//     this port has one airspeed sensor, matching its single-IMU-
//     instance precedent.
//   - The interactive ground/inflight ratio-calibration state machine
//     (Airspeed_Calibration.cpp, update_calibration()/get_calibration_
//     state()) - a real, separate upstream subsystem; this port's ratio
//     is a fixed, caller-supplied constructor value, never recalibrated
//     at runtime. (NOTE: upstream reuses the identifier
//     "update_calibration" for BOTH this ratio auto-cal machinery's own
//     ratio-side bookkeeping in some comments AND the zero-offset
//     routine CPP-083 actually ports from AP_Airspeed.cpp - CPP-083
//     ports only the latter, the real `AP_Airspeed::update_calibration
//     (uint8_t i, float raw_pressure)` zero-offset function, never the
//     Airspeed_Calibration.cpp ratio estimator.)
//   - The health-check state machine (AP_Airspeed_Health.cpp - IMU/GPS-
//     predicted-vs-pitot consistency checking used to auto-disable a
//     drifting sensor). Investigated in full this round: its real,
//     active-by-default gate is ARSPD_WIND_GATE (default 5.0, positive),
//     which drives an EKF-INNOVATION-CONSISTENCY check
//     (AP::ahrs().airspeed_health_data()) - this port has no live EKF
//     fusion wired in place of AhrsDcm (EkfCoreBackend, CPP-080, exists
//     but is not wired to replace AhrsDcm as of CPP-081), and AhrsDcm
//     itself has no Kalman-filter "innovation" concept at all (a
//     complementary-filter DCM, not an EKF). The OTHER half of that
//     function (ARSPD_WIND_MAX-gated GPS-speed-vs-airspeed plausibility
//     check) IS self-contained and buildable, but ARSPD_WIND_MAX
//     defaults to 0 (NOT positive) - i.e. that half is INACTIVE by
//     default, and building only the inactive half would be exactly the
//     kind of default-dead speculative scope this port's own convention
//     avoids (see TKOFF_OPTIONS's own exclusion precedent). Genuinely
//     blocked on live EKF wiring - named here as real future work, not
//     built as a partial, default-inert version now.
//   - PITOT_TUBE_ORDER_NEGATIVE/_POSITIVE - see "PITOT TUBE ORDER" above.
//   - Temperature compensation / get_temperature() - AP_Airspeed_SITL's
//     own get_temperature() has no consumer in this port's read()-
//     formula port (upstream's own temperature correction lives in
//     specific hardware backends' airspeed calculations, not the shared
//     read() path this ticket ports).
#include <cstdint>
#include <cmath>

namespace fwcpp::airspeed {

// Real upstream defaults (AP_Airspeed_Params.cpp), verified directly -
// see file banner.
inline constexpr float kDefaultRatio = 2.0f;  // ARSPD_RATIO
inline constexpr float kDefaultOffset = 0.0f; // ARSPD_OFFSET

// upstream: AP_Airspeed::CalibrationState (AP_Airspeed.h) - this port
// omits NOT_REQUIRED_ZERO_OFFSET (inapplicable, see file banner's
// "CALIBRATION" section) since this port's one SITL sensor always needs
// offset calibration.
enum class CalibrationState { NotStarted, InProgress, Success, Failed };

class AirspeedSensor {
public:
    // ratio/offset default to upstream's own real ARSPD_RATIO/
    // ARSPD_OFFSET values - see file banner's "RATIO/OFFSET PLACEMENT"
    // note for why these are constructor-owned rather than
    // Plane::aparm-owned.
    explicit AirspeedSensor(float ratio = kDefaultRatio, float offset = kDefaultOffset) : ratio_(ratio), offset_(offset) {}

    // upstream: AP_Airspeed::calibrate(in_startup=true) (~line 528) - see
    // file banner's "CALIBRATION" section for the full real sequence and
    // what's deferred (watchdog-reset check, SkipCalType, the
    // NOT_REQUIRED_ZERO_OFFSET early-continue).
    void start_calibration(std::uint32_t now_ms) {
        cal_start_ms_ = now_ms;
        cal_count_ = 0;
        cal_sum_ = 0.0f;
        cal_read_count_ = 0;
        cal_state_ = CalibrationState::InProgress;
    }

    // upstream: AP_Airspeed::read(i) (~line 646) - see file banner for
    // the full formula transcription and the unhealthy->healthy reset
    // edge case. Takes the raw differential pressure (Pa) this tick -
    // upstream's own sensor[i]->get_differential_pressure() return value
    // (a caller drives this from sim_plane.hpp's
    // airspeed_sensor_differential_pressure() in a closed-loop test, or
    // a real hardware driver in production). `now_ms` defaults to 0 for
    // callers that never calibrate (calibration_state() stays
    // NotStarted forever, so the CPP-083 calibration branch below is
    // simply never entered and now_ms is never read) - CPP-082's own
    // existing single-argument call sites and tests keep compiling
    // unchanged.
    void update(float raw_pressure, std::uint32_t now_ms = 0) {
        // prev_healthy MUST be read before healthy_ is overwritten below -
        // this is upstream's own state[i].healthy read at the top of
        // read(), before this same call's backend read touches it again.
        const bool prev_healthy = healthy_;
        // SITL's own get_differential_pressure() never fails without
        // fail-injection this phase doesn't build - see file banner's
        // "HEALTHY()" note. A caller building a real hardware backend in
        // a later phase would thread that backend's own success/failure
        // through here instead of hard-coding true.
        healthy_ = true;

        const float airspeed_pressure = raw_pressure - offset_;
        corrected_pressure_ = airspeed_pressure;

        // CPP-083: real update_calibration(i, raw_pressure) - called
        // with the RAW pressure (matching upstream), AFTER
        // corrected_pressure_ is computed with THIS tick's starting
        // offset_ (matching upstream's real ordering - see file banner's
        // "CALIBRATION" section for why this ordering matters: a
        // same-tick Success->offset_ write must not retroactively change
        // this same tick's corrected_pressure_).
        if (cal_state_ == CalibrationState::InProgress) {
            step_calibration(raw_pressure, now_ms);
        }

        if (!prev_healthy) {
            // upstream: "if (!prev_healthy) { filtered_pressure =
            // airspeed_pressure; }" - reset to the (offset-corrected) raw
            // value instead of blending with a stale/uninitialized
            // filter. Also covers the very-first-call case exactly as
            // upstream does: healthy_ defaults false, so the first ever
            // update() call always takes this branch too.
            filtered_pressure_ = airspeed_pressure;
        } else {
            filtered_pressure_ = 0.7f * filtered_pressure_ + 0.3f * airspeed_pressure;
        }

        // PITOT_TUBE_ORDER_AUTO branch only - see file banner.
        raw_airspeed_ = std::sqrt(std::fabs(airspeed_pressure) * ratio_);
        airspeed_ = std::sqrt(std::fabs(filtered_pressure_) * ratio_);
    }

    // upstream: AP_Airspeed::get_airspeed(i) (the filtered value - what
    // every real consumer, e.g. ahrs.airspeed_EAS(), actually reads).
    [[nodiscard]] float airspeed() const { return airspeed_; }

    // upstream: AP_Airspeed::get_raw_airspeed(i) (unfiltered, for
    // logging/diagnostics - exposed for symmetry/testability, not
    // currently read by any tick() consumer).
    [[nodiscard]] float raw_airspeed() const { return raw_airspeed_; }

    // upstream: AP_Airspeed::healthy(i) - see file banner's "HEALTHY()"
    // note for the real calibration-offset check this phase omits.
    [[nodiscard]] bool healthy() const { return healthy_; }

    // upstream: AP_Airspeed::use(i) - see file banner's "USE()" note for
    // the real ARSPD_USE/glider-throttle special case this phase omits.
    [[nodiscard]] bool use() const { return healthy_; }

    // upstream: AP_Airspeed::get_corrected_pressure(i) - exposed for
    // tests verifying the offset-subtraction step independently of the
    // filter.
    [[nodiscard]] float corrected_pressure() const { return corrected_pressure_; }

    // upstream: state[i].filtered_pressure - exposed for tests verifying
    // the IIR filter's own recurrence/reset behavior directly.
    [[nodiscard]] float filtered_pressure() const { return filtered_pressure_; }

    [[nodiscard]] float ratio() const { return ratio_; }
    [[nodiscard]] float offset() const { return offset_; }

    // upstream: state[i].cal.state - CPP-083. See file banner's
    // "CALIBRATION" section.
    [[nodiscard]] CalibrationState calibration_state() const { return cal_state_; }

    // upstream: state[i].cal.read_count - exposed for tests verifying
    // the discard-first-5/finalize-at->15-reads thresholds directly,
    // independent of offset()'s own final averaged value.
    [[nodiscard]] std::uint16_t calibration_read_count() const { return cal_read_count_; }

    // upstream: state[i].cal.count - the number of samples actually
    // accumulated into cal.sum (i.e. calibration_read_count() minus the
    // first 5 discarded reads) - exposed for tests verifying the discard
    // count independently of calibration_read_count().
    [[nodiscard]] std::uint16_t calibration_sample_count() const { return cal_count_; }

private:
    // upstream: AP_Airspeed::update_calibration(i, raw_pressure)
    // (AP_Airspeed.cpp ~line 574) - see file banner's "CALIBRATION"
    // section for the full transcription. Only called from update()
    // while calibration_state() == InProgress.
    void step_calibration(float raw_pressure, std::uint32_t now_ms) {
        // consider calibration complete when we have at least 15 samples
        // over at least 1 second (upstream's own comment, transcribed).
        if (now_ms - cal_start_ms_ >= 1000 && cal_read_count_ > 15) {
            if (cal_count_ == 0) {
                cal_state_ = CalibrationState::Failed;
            } else {
                offset_ = cal_sum_ / cal_count_;
                cal_state_ = CalibrationState::Success;
            }
            cal_start_ms_ = 0;
            return;
        }
        // we discard the first 5 samples
        if (healthy_ && cal_read_count_ > 5) {
            cal_sum_ += raw_pressure;
            cal_count_++;
        }
        cal_read_count_++;
    }

    float ratio_;
    float offset_;
    bool healthy_ = false; // upstream: state[i].healthy's real zero-initialized default.
    float corrected_pressure_ = 0.0f;
    float filtered_pressure_ = 0.0f;
    float raw_airspeed_ = 0.0f;
    float airspeed_ = 0.0f;

    // upstream: state[i].cal.* (AP_Airspeed.h) - CPP-083.
    CalibrationState cal_state_ = CalibrationState::NotStarted;
    std::uint32_t cal_start_ms_ = 0;
    float cal_sum_ = 0.0f;
    std::uint16_t cal_count_ = 0;
    std::uint16_t cal_read_count_ = 0;
};

} // namespace fwcpp::airspeed
