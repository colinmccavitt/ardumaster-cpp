#pragma once

// Port of libraries/AP_Motors/AP_Motors_Thrust_Linearization.{h,cpp}
// (Copter-4.7.0) - CCP-010, the first ticket of the AP_Motors
// OUTPUT-STAGE phase (motors_matrix.hpp's own file banner named this
// phase as deliberately deferred by every CCP-001 through CCP-009
// ticket). Re-verified directly against the pinned upstream tree:
// AP_Motors_Thrust_Linearization.h is 64 real lines, .cpp is 206 real
// lines (including the blank first line and the trailing newline).
//
// WHAT THIS CLASS DOES: ESCs/props do not turn actuator output (PWM,
// expressed here as 0~1) into thrust linearly, and thrust for a given
// actuator output sags as battery voltage sags. `ThrustLinearization`
// hides both non-linearities from everything above it: callers reason in
// thrust (0~1, roughly proportional to vehicle acceleration) and this
// class converts to/from the actuator range the "ESCs" actually see,
// folding in a filtered battery-voltage-derived `lift_max` term and (via
// an explicit parameter - see below) an air-density gain-scheduling term.
//
// SCOPE: non-Heli only, real upstream `#else` branch of
// `#if APM_BUILD_TYPE(APM_BUILD_Heli)` (AP_Motors_Thrust_Linearization.cpp
// lines 20-33). Real non-Heli defaults, ported as ThrustLinParams's own
// default member initializers below: curve_expo=0.65, spin_min=0.15,
// spin_max=0.95, batt_voltage_max=0.0, batt_voltage_min=0.0 (lines
// 28-32). The Heli branch (lines 21-26: curve_expo=0, spin_min=0,
// spin_max=1, both voltage bounds 0 - "defaults to no linearisation to
// not break users existing setups") is EXCLUDED - out of this
// fixed-wing-first, non-Heli port's scope, exactly as CCP-001 through
// CCP-009 already excluded Heli-only AP_MotorsMatrix concerns.
//
// NO AP_Param: no var_info[]/AP_GROUPINFO table. This port has never used
// AP_Param for pure numeric tuning constants like these six upstream
// parameters (THST_EXPO/SPIN_MIN/SPIN_MAX/BAT_IDX/BAT_V_MAX/BAT_V_MIN) -
// ThrustLinParams below is a plain, constructor/field-settable struct,
// matching copter-rust's own COP-006 (`ThrustLinParams` in
// thrust_linearization.rs), which made the identical choice for the
// identical reason on the Rust side. `BAT_IDX` (upstream: which battery
// monitor instance to compensate against) is NOT represented as a field
// here at all - this class never dereferences it (upstream's own
// AP_MotorsMulticopter resolves AP::battery() by that index BEFORE
// calling in here), and battery-index selection is the caller's problem,
// same as COP-006 notes for its own identical omission.
//
// NO SINGLETONS (ADR-0012 decision 6): real upstream
// `update_lift_max_from_batt_voltage()` calls `AP::battery().voltage(...)`/
// `.voltage_resting_estimate(...)` directly (AP_Motors_Thrust_Linearization.cpp
// line 161), and real upstream `get_compensation_gain()` calls
// `AP::ahrs().get_air_density_ratio()` directly (line 200). This port has
// no ap-battery module and no air-density/AHRS accessor (confirmed via
// `ls modules/`), and ADR-0012 forbids taking either as a global anyway.
// Both are explicit function parameters instead:
//   - update_lift_max_from_batt_voltage takes a `BatteryVoltage` (below),
//     mirroring COP-006's own `BatteryState` parameter on the Rust side.
//   - get_compensation_gain takes `air_density_ratio` directly as a
//     float, UNLIKE COP-006 - see "AIR DENSITY: WHERE THIS PORT DIFFERS
//     FROM COP-006" below for why.
//
// BATTERY VOLTAGE PARAMETER SHAPE, AND THE has_option(BATT_RAW_VOLTAGE)
// SIMPLIFICATION: real upstream reads
// `motors.has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE)` TWICE -
// once to choose `AP::battery().voltage(batt_idx)` (raw, instantaneous,
// sag included) over `.voltage_resting_estimate(batt_idx)` (sag removed)
// as the input, once to choose resetting `batt_voltage_filt` outright
// over actually filtering it. This port has no `AP_Motors::MotorOptions`
// concept at all yet, so `BatteryVoltage` carries BOTH real upstream
// quantities (`raw`/`resting_estimate`) as separate fields - matching
// this exact real branch's shape, so a future ticket that adds a real
// options concept can wire it up without changing this struct - but
// `update_lift_max_from_batt_voltage` below only ever reads
// `resting_estimate` and always filters (never resets-to-value), i.e.
// treats every caller as the option being UNSET, upstream's own default
// path. This is the identical simplification COP-006 made on the Rust
// side (see that file's own "No BATT_RAW_VOLTAGE option" section) for
// the identical reason.
//
// UNCLAMPED ZERO-EXPO ASYMMETRY - REAL, RE-VERIFIED DIRECTLY: both
// apply_thrust_curve_and_volt_scaling (.cpp lines 119-133) and
// remove_thrust_curve_and_volt_scaling (lines 137-153) special-case
// is_zero(curve_expo) to avoid a divide-by-zero in the general quadratic
// formula (upstream's own comment: "avoid floating point exception for
// small values"). In BOTH functions, the zero-expo branch returns
// DIRECTLY - `lift_max * thrust * battery_scale` (line 129) /
// `throttle / (lift_max * battery_scale)` (line 147) - with NO
// `constrain_float(..., 0, 1)` clamp, while the non-zero-expo branch a
// few lines below DOES clamp its own result (line 132 / line 152). This
// is a real, easy-to-miss upstream asymmetry (independently re-verified
// by COP-006 on the Rust side too), faithfully reproduced below, NOT
// "corrected" by clamping both branches - see thrust_linearization_test.cpp
// for a dedicated test that would fail if it were.
//
// AIR DENSITY: WHERE THIS PORT DIFFERS FROM COP-006 - COP-006 added a
// real `ap-baro` dependency to `ap-motors` on the Rust side and had
// `get_compensation_gain` take an altitude (AMSL, metres), computing the
// density ratio itself via `ap_baro::air_density_for_alt_amsl`. This
// port's own CCP-010 ticket explicitly calls for the ratio itself as the
// parameter instead ("take the air density ratio as an explicit
// parameter too ... do not silently drop the air-density term entirely"),
// with no mention of pulling in an ap-baro-equivalent dependency here -
// this port's ap-motors module has no such dependency today, and adding
// one is left to whichever future ticket actually wires up a real
// AHRS/baro accessor. `get_compensation_gain(air_density_ratio)` below
// therefore takes the ratio directly, a strictly smaller/simpler
// dependency footprint than COP-006's own choice for the equivalent Rust
// method - stated explicitly here as a deliberate divergence, not an
// oversight.
//
// LowPassFilter API (modules/ap-filter/include/fwcpp/filter/low_pass_filter.hpp,
// read directly before using it): `fwcpp::filter::LowPassFilter<T>`'s
// `apply(sample, dt)` recomputes alpha from `dt` every call, matching
// upstream's own variable-dt `LowPassFilterFloat::apply(sample, dt)` this
// class's real `batt_voltage_filt` member uses (line 176/336... i.e. the
// AP_MOTORS_BATT_VOLT_FILT_HZ-cutoff filter) - NOT
// `LowPassFilterConstDt`, which precomputes alpha from a fixed sample
// rate and has a different `apply(sample)` (no dt) signature. `reset(v)`
// matches upstream's own `batt_voltage_filt.reset(v)` exactly (immediate
// set, no smoothing).
//
// DEFERRED FUTURE PHASES: the remainder of the real AP_Motors output
// stage - output_to_motors/output_armed_stabilizing/
// check_for_failed_motor/thrust_compensation/current-limiting (the C++
// analogue of copter-rust's own separate current_limit.rs work, COP-004)
// - are separate, deliberately deferred future phases of this same
// phase, not part of CCP-010's own scope.

#include <algorithm>

#include <fwcpp/filter/low_pass_filter.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::motors {

// AP_MOTORS_BATT_VOLT_FILT_HZ (AP_Motors_Thrust_Linearization.cpp line 9:
// "battery voltage filtered at 0.5hz").
inline constexpr float kBattVoltFiltHz = 0.5f;

// Thrust_Linearization::var_info's six real tunables, minus BAT_IDX (see
// file banner's "NO AP_Param" section for why it has no field here).
// Defaults are the real non-Heli branch - see file banner's "SCOPE".
struct ThrustLinParams {
    // THST_EXPO: motor thrust curve exponent, 0 for linear, 1 for a full
    // second-order curve. Clamped to [-1, 1] wherever it is read below,
    // never here (matching upstream, which stores the raw AP_Float and
    // clamps at each use site instead).
    float curve_expo = 0.65f;
    // SPIN_MIN: throttle-out ratio which produces the minimum thrust.
    float spin_min = 0.15f;
    // SPIN_MAX: throttle-out ratio which produces the maximum thrust.
    float spin_max = 0.95f;
    // BAT_V_MAX: maximum voltage used to scale lift. 0.0 disables
    // battery-voltage compensation entirely (see
    // update_lift_max_from_batt_voltage's own sanity check below).
    float batt_voltage_max = 0.0f;
    // BAT_V_MIN: minimum voltage used to scale lift.
    float batt_voltage_min = 0.0f;
};

// Real upstream `has_option(AP_Motors::MotorOptions::BATT_RAW_VOLTAGE)`
// branch's two possible voltage sources, both carried explicitly (see
// file banner's "BATTERY VOLTAGE PARAMETER SHAPE" section) rather than
// reading either from a global battery-monitor singleton.
struct BatteryVoltage {
    // Upstream `AP::battery().voltage(batt_idx)` - raw, instantaneous
    // reading, sag included. NOT read by update_lift_max_from_batt_voltage
    // today (this port has no BATT_RAW_VOLTAGE-equivalent option), carried
    // only so this struct's shape does not need to change once one exists.
    float raw = 0.0f;
    // Upstream `AP::battery().voltage_resting_estimate(batt_idx)` -
    // sag-removed estimate. This is the only field
    // update_lift_max_from_batt_voltage actually reads, matching
    // upstream's own default (option-unset) path.
    float resting_estimate = 0.0f;
};

// ThrustLinearization - port of upstream `Thrust_Linearization`. See file
// banner for exactly what upstream behavior this reproduces and what is
// deferred.
class ThrustLinearization {
public:
    // Real upstream constructor (AP_Motors_Thrust_Linearization.cpp
    // lines 90-101, non-Heli branch): lift_max starts at 1.0, the
    // battery-voltage filter's cutoff is set to kBattVoltFiltHz and reset
    // to 1.0 immediately (matching `batt_voltage_filt.reset(1.0)` - a
    // full-voltage, no-compensation-yet starting point, re-verified as
    // 1.0 and not 0.0/unset).
    ThrustLinearization() {
        batt_voltage_filt_.set_cutoff_frequency(kBattVoltFiltHz);
        batt_voltage_filt_.reset(1.0f);
    }

    // LowPassFilter<T> (a DigitalLPF<T> base) has its copy ctor/assign
    // deleted, so this would be implicitly deleted anyway; deleted
    // explicitly to say so, matching this port's own Tecs precedent
    // (fwcpp/tecs/tecs.hpp) for a class holding a LowPassFilterFloat
    // member.
    ThrustLinearization(const ThrustLinearization&) = delete;
    ThrustLinearization& operator=(const ThrustLinearization&) = delete;

    // Upstream `get_lift_max()`.
    [[nodiscard]] float lift_max() const { return lift_max_; }

    // apply_thrust_curve_and_volt_scaling - returns throttle in the range
    // 0~1 (upstream's own comment). See file banner's "UNCLAMPED
    // ZERO-EXPO ASYMMETRY" - the zero-expo branch below is genuinely
    // unclamped, matching real upstream exactly.
    [[nodiscard]] float apply_thrust_curve_and_volt_scaling(const ThrustLinParams& params, float thrust) const {
        float battery_scale = 1.0f;
        if (math::is_positive(batt_voltage_filt_.get())) {
            battery_scale = 1.0f / batt_voltage_filt_.get();
        }
        // Domain -1.0 to 1.0, range -1.0 to 1.0 (upstream's own comment).
        const float curve_expo = math::constrain_value(params.curve_expo, -1.0f, 1.0f);
        if (math::is_zero(curve_expo)) {
            // Zero expo means linear, avoid floating point exception for
            // small values (upstream's own comment) - UNCLAMPED, real
            // upstream asymmetry, see file banner. Do not add a clamp
            // here.
            return lift_max_ * thrust * battery_scale;
        }
        const float throttle_ratio =
            ((curve_expo - 1.0f) +
             math::safe_sqrt((1.0f - curve_expo) * (1.0f - curve_expo) + 4.0f * curve_expo * lift_max_ * thrust)) /
            (2.0f * curve_expo);
        return math::constrain_value(throttle_ratio * battery_scale, 0.0f, 1.0f);
    }

    // Inverse of above (upstream's own comment) - tested upstream with
    // AP_Motors/examples/expo_inverse_test, so transcribed exactly rather
    // than re-derived. Also carries the real zero-expo UNCLAMPED
    // asymmetry, matching apply_thrust_curve_and_volt_scaling's own - see
    // file banner.
    [[nodiscard]] float remove_thrust_curve_and_volt_scaling(const ThrustLinParams& params, float throttle) const {
        float battery_scale = 1.0f;
        if (math::is_positive(batt_voltage_filt_.get())) {
            battery_scale = 1.0f / batt_voltage_filt_.get();
        }
        const float curve_expo = math::constrain_value(params.curve_expo, -1.0f, 1.0f);
        if (math::is_zero(curve_expo)) {
            // UNCLAMPED - matches apply_thrust_curve_and_volt_scaling's
            // own zero-expo branch, real upstream asymmetry (file
            // banner). Do not add a clamp here.
            return throttle / (lift_max_ * battery_scale);
        }
        float thrust = ((throttle / battery_scale) * (2.0f * curve_expo)) - (curve_expo - 1.0f);
        thrust = (thrust * thrust) - ((1.0f - curve_expo) * (1.0f - curve_expo));
        thrust /= 4.0f * curve_expo * lift_max_;
        return math::constrain_value(thrust, 0.0f, 1.0f);
    }

    // Converts desired thrust to linearized actuator output in a range of
    // 0~1 (upstream's own comment).
    [[nodiscard]] float thrust_to_actuator(const ThrustLinParams& params, float thrust_in) const {
        thrust_in = math::constrain_value(thrust_in, 0.0f, 1.0f);
        return params.spin_min + (params.spin_max - params.spin_min) *
                                      apply_thrust_curve_and_volt_scaling(params, thrust_in);
    }

    // Inverse of above (upstream's own comment) - used to calculate
    // equivalent motor throttle level to direct output, used in
    // tailsitter transitions (upstream's own comment).
    [[nodiscard]] float actuator_to_thrust(const ThrustLinParams& params, float actuator) const {
        const float scaled_actuator = (actuator - params.spin_min) / (params.spin_max - params.spin_min);
        return math::constrain_value(remove_thrust_curve_and_volt_scaling(params, scaled_actuator), 0.0f, 1.0f);
    }

    // update_lift_max_from_batt_voltage - used for voltage compensation
    // (upstream's own comment). See file banner's "BATTERY VOLTAGE
    // PARAMETER SHAPE" section: `battery.resting_estimate` is the only
    // field read (this port always takes upstream's default,
    // option-unset path: sag-removed voltage, always filtered - never
    // `battery.raw`, never the reset-to-value branch).
    //
    // Real upstream also permanently raises a misconfigured
    // `batt_voltage_min` in place (`batt_voltage_min.set(MAX(...))`,
    // .cpp line 169) - reproduced here as a real write-back to
    // `params.batt_voltage_min`, hence `ThrustLinParams&` and not
    // `const ThrustLinParams&`.
    void update_lift_max_from_batt_voltage(ThrustLinParams& params, const BatteryVoltage& battery, float dt) {
        const float batt_voltage = battery.resting_estimate;

        // Sanity check batt_voltage_min is not too small. If disabled or
        // misconfigured, exit immediately (upstream's own comment).
        if (params.batt_voltage_max <= 0.0f || params.batt_voltage_min >= params.batt_voltage_max ||
            batt_voltage < 0.25f * params.batt_voltage_min) {
            batt_voltage_filt_.reset(1.0f);
            lift_max_ = 1.0f;
            return;
        }

        params.batt_voltage_min = std::max(params.batt_voltage_min, params.batt_voltage_max * 0.6f);

        // Constrain resting voltage estimate (resting voltage is actual
        // voltage with sag removed based on current draw and resistance -
        // upstream's own comment) into the configured range.
        const float constrained_voltage =
            math::constrain_value(batt_voltage, params.batt_voltage_min, params.batt_voltage_max);

        // Filter at kBattVoltFiltHz (upstream's own comment: "filter at
        // 0.5 Hz"). Always the filtered path - see file banner's
        // has_option(BATT_RAW_VOLTAGE) simplification.
        batt_voltage_filt_.apply(constrained_voltage / params.batt_voltage_max, dt);

        // Calculate lift max (upstream's own comment).
        const float curve_expo = math::constrain_value(params.curve_expo, -1.0f, 1.0f);
        const float filt = batt_voltage_filt_.get();
        lift_max_ = filt * (1.0f - curve_expo) + curve_expo * filt * filt;
    }

    // Return gain scheduling gain based on voltage and air density
    // (upstream's own comment). `air_density_ratio` is an explicit
    // parameter, not read from AHRS - see file banner's "AIR DENSITY:
    // WHERE THIS PORT DIFFERS FROM COP-006".
    [[nodiscard]] float get_compensation_gain(float air_density_ratio) const {
        // Avoid divide by zero (upstream's own comment).
        if (lift_max_ <= 0.0f) {
            return 1.0f;
        }

        float ret = 1.0f / lift_max_;

        // Air density ratio is increasing in density / decreasing in
        // altitude (upstream's own comment). Real upstream gate is
        // strictly `> 0.3 && < 1.5` - re-verified directly, both
        // exclusive, not inclusive.
        if (air_density_ratio > 0.3f && air_density_ratio < 1.5f) {
            ret *= 1.0f / math::constrain_value(air_density_ratio, 0.5f, 1.25f);
        }
        return ret;
    }

private:
    float lift_max_ = 1.0f;                          // maximum lift ratio from battery voltage
    fwcpp::filter::LowPassFilterFloat batt_voltage_filt_; // see constructor: cutoff set there, not here
};

} // namespace fwcpp::motors
