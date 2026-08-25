#pragma once

// Port of ArduPlane's Mode base class (ArduPlane/mode.h, 1075 lines +
// mode.cpp, 414 lines - both read in full) reduced to what ModeManual/
// ModeFBWA actually use, plus ModeManual (mode_manual.cpp, 31 lines, in
// full) and ModeFBWA (mode_fbwa.cpp, 45 lines, in full). CPP-031 "slice
// 1". See plane.hpp (same module) for the vehicle-level state/helpers
// these call into and that file's banner for the module-wide judgment
// calls (no singletons, RC/SRV channel mapping, TECS's reset()-only role,
// ground_mode/reversed_throttle, stick-mixing/ground-steering exclusions).
//
// SHAPE CHOICE: a small virtual-dispatch class hierarchy (update()/run()),
// matching upstream's own Mode/ModeManual/ModeFBWA inheritance as closely
// as ADR-0012 allows. This is a DELIBERATELY DIFFERENT choice from
// ap-fw-control's FwController (composed, not inherited - see fw_
// controller.hpp's own banner): FwController's base never has more than
// one live caller shape to dispatch across, but THIS hierarchy's entire
// purpose is a caller holding one `Mode&` (of unknown concrete type) and
// calling update()/run() on whichever mode is currently active - exactly
// the live polymorphic-dispatch need fw_controller.hpp's banner says
// never existed for it. A future slice adding FBWB/CRUISE/AUTO/etc (see
// the ticket's "slice 2" list) extends this the same way upstream does:
// a new Mode subclass, no change to Mode itself or to tick() below.
//
// Mode::Number/name()/name4()/is_vtol_mode()/is_guided_mode()/
// does_auto_navigation()/does_auto_throttle()/mode_allows_autotuning()/
// allows_throttle_nudging()/use_throttle_limits()/use_battery_
// compensation()/navigate()/update_target_altitude()/pre_arm_checks()/
// enter()/exit() are NOT PORTED - mode-IDENTIFICATION and mode-SWITCHING
// machinery, not stabilization logic, out of scope for a fixed two-mode
// slice with no runtime mode-change support. Each depends on a subsystem
// this port hasn't built (fence/mission/camera/ADSB/arming/battery/
// TECS-driven navigation) or on modes this slice doesn't implement.
//
// Mode::run()'s StickMixing switch (stabilize_stick_mixing_fbw/direct) is
// skipped entirely - see plane.hpp's banner. Every mode in this slice
// behaves as upstream's StickMixing::NONE case (the real default,
// STICK_MIXING param default 0).

#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/srv/srv_channels.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::vehicle {

class Mode {
public:
    explicit Mode(Plane& plane) : plane_(plane) {}
    virtual ~Mode() = default;
    Mode(const Mode&) = delete;
    Mode& operator=(const Mode&) = delete;

    // upstream: Mode::update() (pure virtual) - convert pilot/mode input
    // into nav_roll_cd/nav_pitch_cd targets and/or direct servo output.
    virtual void update(const StabilizeInputs& in) = 0;

    // upstream: Mode::run() (mode.cpp) minus the StickMixing switch (see
    // file banner) - stabilize all three axes. ModeFBWA overrides this to
    // add output_pilot_throttle() after calling the base (matching
    // upstream's ModeFBWA::run(), "Run base class function and then
    // output throttle"); ModeManual overrides it entirely (matching
    // upstream's ModeManual::run(), which does not call Mode::run() at
    // all - MANUAL never stabilizes).
    virtual void run(const StabilizeInputs& in) {
        plane_.stabilize_roll(in);
        plane_.stabilize_pitch(in);
        plane_.stabilize_yaw(in);
    }

    // upstream: Mode::reset_controllers() (mode.cpp) - PUBLIC there (not
    // protected): tick() below calls it directly on the active mode from
    // outside the Mode hierarchy, exactly as upstream's Plane::
    // stabilize() does on its own 2-second-stale check. steer_state reset
    // is skipped: no ground-steering subsystem (see plane.hpp's banner).
    void reset_controllers() {
        plane_.roll_controller.reset_i();
        plane_.pitch_controller.reset_i();
        plane_.yaw_controller.reset_I();
        plane_.tecs.reset();
    }

protected:
    // upstream: Mode::output_pilot_throttle() (mode.cpp) - "Output pilot
    // throttle, this is used in stabilized modes without auto throttle
    // control."
    void output_pilot_throttle() {
        if (plane_.aparm.throttle_passthru_stabilize) {
            plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_throttle_input(true));
            return;
        }
        plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_adjusted_throttle_input(true));
    }

    // upstream: Mode::output_rudder_and_steering() (mode.cpp) - "Helper
    // to output to both k_rudder and k_steering servo functions."
    void output_rudder_and_steering(float val) {
        plane_.srv_channels.set_output_scaled(srv::Function::kRudder, val);
        plane_.srv_channels.set_output_scaled(srv::Function::kSteering, val);
    }

    Plane& plane_;
};

// upstream: ModeManual (mode.h) + mode_manual.cpp, read and reproduced in
// full (31 lines) - no exclusions in this one, it is already this small
// upstream. use_battery_compensation()/use_throttle_limits() overrides
// not ported - see file banner.
class ModeManual : public Mode {
public:
    using Mode::Mode;

    void update(const StabilizeInputs&) override {
        plane_.srv_channels.set_output_scaled(srv::Function::kAileron, plane_.roll_in_expo(false));
        plane_.srv_channels.set_output_scaled(srv::Function::kElevator, plane_.pitch_in_expo(false));
        output_rudder_and_steering(plane_.rudder_in_expo(false));

        const float throttle = plane_.get_throttle_input(true);
        plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, throttle);

        // nav_roll_cd/nav_pitch_cd are set from the AHRS's OWN current
        // attitude here, NOT a demand - matches upstream exactly (kept
        // for logging/consistency with other modes; MANUAL never
        // stabilizes toward these, see run() below - direct stick
        // passthrough only).
        plane_.nav_roll_cd = plane_.roll_sensor_cd();
        plane_.nav_pitch_cd = plane_.pitch_sensor_cd();
    }

    // upstream: ModeManual::run() - "reset_controllers();" only. Does NOT
    // call Mode::run() (no stabilization at all in MANUAL); just resets
    // the rate/TECS controllers so they don't accumulate integrator
    // wind-up while MANUAL is active.
    void run(const StabilizeInputs&) override { reset_controllers(); }
};

// upstream: ModeFBWA (mode.h) + mode_fbwa.cpp, read in full (45 lines).
// EXCLUDED (documented in the ticket, not silently dropped):
//   - The RC-failsafe glide branch (`if (plane.failsafe.rc_failsafe &&
//     plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) { nav_roll_cd = 0;
//     nav_pitch_cd = 0; SRV_Channels::set_output_limit(k_throttle, MIN);
//     }`) - no failsafe subsystem in this port.
//   - The FBWA-taildragger-takeoff aux-switch check (`rc().find_channel_
//     for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER)`) - needs the
//     aux-function-dispatch subsystem CPP-027 explicitly deferred.
// fly_inverted()'s pitch negation IS kept (cheap, self-contained, and
// upstream's own real behavior) even though it is always a no-op in this
// slice's scope - Plane::fly_inverted() always returns false for MANUAL/
// FBWA (see its own doc comment in plane.hpp).
class ModeFBWA : public Mode {
public:
    using Mode::Mode;

    void update(const StabilizeInputs&) override {
        // set nav_roll and nav_pitch using sticks
        plane_.nav_roll_cd =
            static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
        plane_.update_load_factor();

        const float pitch_input = plane_.channel_pitch()->norm_input();
        if (pitch_input > 0.0f) {
            plane_.nav_pitch_cd = static_cast<std::int32_t>(pitch_input * plane_.aparm.pitch_limit_max_deg * 100.0f);
        } else {
            plane_.nav_pitch_cd = static_cast<std::int32_t>(-(pitch_input * plane_.pitch_limit_min * 100.0f));
        }
        plane_.adjust_nav_pitch_throttle();
        plane_.nav_pitch_cd = math::constrain_value(plane_.nav_pitch_cd,
                                                      static_cast<std::int32_t>(plane_.pitch_limit_min * 100.0f),
                                                      static_cast<std::int32_t>(plane_.aparm.pitch_limit_max_deg * 100.0f));
        if (plane_.fly_inverted()) {
            plane_.nav_pitch_cd = -plane_.nav_pitch_cd;
        }
    }

    // upstream: ModeFBWA::run() - "Run base class function and then
    // output throttle." FBWA has manual (pilot-stick) throttle, not
    // auto-throttle (see Mode::does_auto_throttle(), not ported, always
    // false for this slice's two modes).
    void run(const StabilizeInputs& in) override {
        Mode::run(in);
        output_pilot_throttle();
    }
};

// upstream: ModeFBWB (mode.h) + mode_fbwb.cpp, read in full (17 lines) -
// CPP-031 slice 2. See plane.hpp's file banner addendum for the full
// design rationale (altitude reference frame, current-altitude-input vs.
// target-altitude-state split, why Tecs::update_50hz()/
// update_pitch_throttle() are called from update_fbwb_speed_height()
// below rather than mode.hpp's shared tick(), and the FBWB airspeed-
// target surprise).
//
// _enter() IS NOT PORTED/CALLED AUTOMATICALLY - this slice has no mode-
// switching machinery yet (mode.hpp's own file banner already documents
// this exclusion for Mode::enter()/exit() generally). Upstream's real
// _enter() body is just `plane.set_target_altitude_current()` (the
// HAL_SOARING_ENABLED init_cruising() call is excluded - no soaring
// subsystem). A CALLER CONSTRUCTING A ModeFBWB MUST CALL
// plane.set_target_altitude_current(current_altitude_cm) ONCE, EXPLICITLY,
// BEFORE THE FIRST tick()/update() - otherwise target_altitude_cm starts
// at its bare default (0), not the vehicle's actual current altitude, and
// FBWB's very first pitch/throttle demand would target that instead of
// "hold where you are" as upstream's real mode-entry behavior guarantees.
class ModeFBWB : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeFBWB::update() (mode_fbwb.cpp) - "set nav_roll from
    // the roll stick exactly like FBWA, then update_load_factor(), then
    // update_fbwb_speed_height()." Pitch is NOT set here at all (unlike
    // FBWA) - update_fbwb_speed_height() (plane.hpp) computes nav_pitch_cd
    // from TECS's own pitch demand instead, via calc_nav_pitch().
    void update(const StabilizeInputs& in) override {
        plane_.nav_roll_cd =
            static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
        plane_.update_load_factor();
        plane_.update_fbwb_speed_height(in);
    }

    // upstream: ModeFBWB has NO run() override at all - relies entirely on
    // the base Mode::run() (stabilize all three axes, see Mode::run()
    // above). Unlike ModeFBWA, FBWB does NOT call output_pilot_throttle()
    // after stabilizing: calc_throttle() (called from
    // update_fbwb_speed_height() above, i.e. during update() - BEFORE
    // run()) already wrote TECS's computed throttle demand straight to
    // the throttle servo function. Mode::does_auto_throttle() (not
    // ported - see mode.hpp's own banner) is true for FBWB upstream; this
    // "no run() override" shape IS that behavior, expressed structurally
    // instead of via a ported boolean flag.
};

// upstream: the real scheduler task-table sequence (AHRS update ->
// update_control_mode/navigate -> Plane::stabilize() -> Plane::
// set_servos()/output), inferred from Mode::run()'s own body plus
// ModeFBWA::run()'s "run base then output throttle" pattern, per the
// ticket's own instruction. Folds in Plane::ahrs_update()'s roll/pitch-
// limit scaling, Plane::calc_airspeed_errors()'s speed-scaler filter
// update, and Plane::stabilize()'s 2-second-stale reset_controllers()
// check (Attitude.cpp) - the pre-takeoff integrator-zeroing check right
// after it in upstream's stabilize() is excluded (needs barometer/
// relative-altitude/groundspeed, no such subsystem in this port), as is
// the nav_scripting_active()/mode_training special dispatch in stabilize()
// (both always just call mode.run() unconditionally here - matching this
// slice's two modes, neither of which is mode_training or
// scripting-driven).
//
// CPP-031 SLICE 2 (FBWB) NOTE: this function is UNCHANGED by ModeFBWB's
// addition - Tecs::update_50hz()/update_pitch_throttle() are called from
// within Plane::update_fbwb_speed_height() (plane.hpp), reached only via
// ModeFBWB::update() above, not from here. See plane.hpp's file banner
// addendum ("SURPRISING UPSTREAM FINDING #1") for why: upstream itself
// gates both calls on `does_auto_throttle()` (true for FBWB only), and
// calling them unconditionally from this shared tick() would run them for
// MANUAL/FBWA too - wrong per upstream's own real behavior - without
// resurrecting the mode-identification machinery this port deliberately
// left unported.
//
// CPP-031 SLICE 3 NOTE: this function is what actually closes the real gap
// CPP-033 was built for - see plane.hpp's own file banner addendum for the
// full design rationale (StabilizeInputs's four new fields, the
// fly_forward()/accel_healthy()/ins_healthy() Plane methods, and the
// CALL-ORDER NOTE explaining why drift correction runs AFTER, not
// interleaved with, ahrs.update() this tick). Previously this function
// called ONLY `plane.ahrs.update(gyro_sample)` - pure gyro integration,
// with NO drift correction at all, despite CPP-028 slices 2/3 having fully
// ported and unit-tested drift_correction_yaw()/drift_correction_accel().
// Steps 2 and 3b below are what were missing.
inline void tick(Plane& plane, Mode& mode, const ahrs::GyroSample& gyro_sample, const StabilizeInputs& in) {
    // 1. pull RC input (upstream: AP_Vehicle's read_radio() scheduled task)
    plane.rc_channels.read_input(plane.hal.rc_input);

    // 2. GPS update (upstream: AP_GPS::update(), a separate, earlier
    //    scheduled task feeding the AHRS update that follows it - see
    //    ap-gps/gps.hpp's own file banner for what this reproduces from
    //    AP_GPS_SITL). Always called every tick; internally rate-limited to
    //    200ms, exactly like the real backend, so most calls are a no-op.
    plane.gps.update(in.true_velocity_ned, in.now_ms);

    // 3. AHRS update (upstream: Plane::ahrs_update()'s ahrs.update() call)
    plane.ahrs.update(gyro_sample);

    // 3b. Drift correction (upstream: the REST of AP_AHRS_DCM::update() -
    //    drift_correction(delta_t) - which this port's AhrsDcm (CPP-028
    //    slices 2/3) split into accumulate_accel() (every tick, unrated)
    //    plus drift_correction_yaw()/drift_correction_accel() (each
    //    internally gated on a new GPS-fix-time observation - see their own
    //    doc comments in ahrs_dcm.hpp). CompassSample is constructed fresh
    //    here with healthy=false (its own default) EVERY TICK - no compass
    //    hardware in this port yet, a REAL CURRENT LIMITATION, not a
    //    permanent design choice (see plane.hpp's file banner addendum).
    //    This means drift_correction_yaw()'s GPS-course fallback path -
    //    reachable exactly because use_compass() returns false immediately
    //    for an unhealthy compass - is what actually corrects yaw for this
    //    port's vehicle today, once it is moving fast enough for a
    //    meaningful GPS course (kGpsSpeedMinMs, 3 m/s).
    const ahrs::CompassSample compass; // healthy=false (default) - see above
    const ahrs::GpsSample& gps_sample = plane.gps.sample();
    const float wind_speed_ms = in.wind_estimate.xy().length(); // see plane.hpp's file banner addendum
    const float airspeed_tas = in.airspeed_valid ? in.airspeed_eas * in.eas2tas : 0.0f; // matches ap-tecs's own EAS*eas2tas->TAS precedent

    plane.ahrs.accumulate_accel(in.accel_sample, in.dt);
    plane.ahrs.drift_correction_yaw(compass, gps_sample, plane.fly_forward(), in.armed_and_safety_off, in.gps_use_enabled,
                                     wind_speed_ms, in.now_ms);
    plane.ahrs.drift_correction_accel(compass, gps_sample, plane.fly_forward(), in.armed_and_safety_off, in.gps_use_enabled,
                                       in.wind_estimate, airspeed_tas, plane.accel_healthy(), plane.ins_healthy(), in.now_ms);

    // 4. scaled roll/pitch limits from current attitude (upstream: the
    //    rest of Plane::ahrs_update())
    plane.update_flight_limits();

    // 5. speed-scaler low-pass filter (upstream: calc_airspeed_errors(),
    //    normally a separate 10Hz scheduled task - see Plane::
    //    update_speed_scaler()'s own doc comment)
    plane.update_speed_scaler(in.airspeed_valid, in.airspeed_eas, in.armed_and_safety_off, in.dt);

    // 6. mode update + stabilize (upstream: Plane::stabilize())
    if (in.now_ms - plane.last_stabilize_ms > 2000U) {
        mode.reset_controllers();
    }
    plane.last_stabilize_ms = in.now_ms;
    mode.update(in);
    mode.run(in);

    // 7. write computed PWM to hardware (upstream: Plane::set_servos() ->
    //    SRV_Channels::output_ch_all())
    plane.srv_channels.output_ch_all(plane.hal.rc_output);
}

} // namespace fwcpp::vehicle
