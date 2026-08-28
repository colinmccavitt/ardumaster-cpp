#pragma once

// Port of AP_Airspeed's real read()-formula core - NOT the full generic
// multi-sensor AP_Airspeed interface (primary-sensor voting, the
// interactive ground/inflight ratio-calibration state machine, the
// IMU/GPS-cross-check health-check state machine, sensor failure
// injection) - just the single-instance pressure-to-EAS pipeline every
// real airspeed backend (analog, digital, SITL) shares. Matches this
// port's established "match the real formula, not the generic interface"
// methodology (ap-compass's Compass, ap-gps's Gps). CPP-082, phase 1 of
// this port's airspeed sensor subsystem.
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
//     state()) - a real, separate upstream subsystem; this port's ratio/
//     offset are fixed, caller-supplied constructor values, never
//     recalibrated at runtime.
//   - The health-check state machine (AP_Airspeed_Health.cpp - IMU/GPS-
//     predicted-vs-pitot consistency checking used to auto-disable a
//     drifting sensor) - real, separate, substantial upstream scope, not
//     modeled by healthy() above.
//   - PITOT_TUBE_ORDER_NEGATIVE/_POSITIVE - see "PITOT TUBE ORDER" above.
//   - Temperature compensation / get_temperature() - AP_Airspeed_SITL's
//     own get_temperature() has no consumer in this port's read()-
//     formula port (upstream's own temperature correction lives in
//     specific hardware backends' airspeed calculations, not the shared
//     read() path this ticket ports).
#include <cmath>

namespace fwcpp::airspeed {

// Real upstream defaults (AP_Airspeed_Params.cpp), verified directly -
// see file banner.
inline constexpr float kDefaultRatio = 2.0f;  // ARSPD_RATIO
inline constexpr float kDefaultOffset = 0.0f; // ARSPD_OFFSET

class AirspeedSensor {
public:
    // ratio/offset default to upstream's own real ARSPD_RATIO/
    // ARSPD_OFFSET values - see file banner's "RATIO/OFFSET PLACEMENT"
    // note for why these are constructor-owned rather than
    // Plane::aparm-owned.
    explicit AirspeedSensor(float ratio = kDefaultRatio, float offset = kDefaultOffset) : ratio_(ratio), offset_(offset) {}

    // upstream: AP_Airspeed::read(i) (~line 646) - see file banner for
    // the full formula transcription and the unhealthy->healthy reset
    // edge case. Takes the raw differential pressure (Pa) this tick -
    // upstream's own sensor[i]->get_differential_pressure() return value
    // (a caller drives this from sim_plane.hpp's
    // airspeed_sensor_differential_pressure() in a closed-loop test, or
    // a real hardware driver in production).
    void update(float raw_pressure) {
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

private:
    float ratio_;
    float offset_;
    bool healthy_ = false; // upstream: state[i].healthy's real zero-initialized default.
    float corrected_pressure_ = 0.0f;
    float filtered_pressure_ = 0.0f;
    float raw_airspeed_ = 0.0f;
    float airspeed_ = 0.0f;
};

} // namespace fwcpp::airspeed
