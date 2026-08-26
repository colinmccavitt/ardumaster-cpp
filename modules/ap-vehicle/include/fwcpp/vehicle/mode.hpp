#pragma once

// Out-of-line method bodies for the Mode class hierarchy (declared in
// plane.hpp - see that file's own "CPP-031 SLICE 7 ADDENDUM" banner note
// for exactly why the split exists: Plane needs each concrete Mode
// subclass to be a COMPLETE type to hold it by value, while every
// Plane-touching Mode/ModeXXX method body needs PLANE to be a complete
// type - an ordering cycle broken the same way upstream's own mode.h/
// mode.cpp split breaks it, reproduced within this port's existing
// two-header convention instead of adding six new mode_*.cpp-equivalent
// files. This file `#include`s plane.hpp (Plane, complete, plus every
// Mode class declaration) and ONLY defines method bodies + tick() - no
// new class declarations live here anymore as of CPP-031 slice 7.
//
// Every judgment call, upstream citation, and exclusion for each method
// body below is documented on that method's DECLARATION in plane.hpp (the
// Mode class hierarchy section) and in plane.hpp's own SLICE 1/2/4/5/6/7
// file-banner addenda - not repeated here beyond a short pointer, to keep
// a single source of truth per judgment call rather than two copies that
// could drift.
//
// TICK() - see this file's own comment on the function below for the
// CPP-031 slice 4 navigate()-vs-update()/run() ordering decision (SHAPE
// CHOICE unchanged since slice 1: "a single fixed sequence suffices") and
// slice 7's own change (dispatching through `plane.control_mode` instead
// of taking an explicit `Mode&` parameter - the real payoff of this
// slice's set_mode() work: a set_mode() call made mid-tick, e.g. from
// ModeAUTO::navigate()'s own mission-complete transition, takes effect
// starting the FOLLOWING tick() call, not the one still in progress - see
// that comment for why and how this is verified).

#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::vehicle {

// ---------------------------------------------------------------------
// Mode (base class) - out-of-line bodies. Declarations + full judgment-
// call documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void Mode::run(const StabilizeInputs& in) {
    plane_.stabilize_roll(in);
    plane_.stabilize_pitch(in);
    plane_.stabilize_yaw(in);
}

inline void Mode::reset_controllers() {
    plane_.roll_controller.reset_i();
    plane_.pitch_controller.reset_i();
    plane_.yaw_controller.reset_I();
    plane_.tecs.reset();
}

inline void Mode::output_pilot_throttle() {
    if (plane_.aparm.throttle_passthru_stabilize) {
        plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_throttle_input(true));
        return;
    }
    plane_.srv_channels.set_output_scaled(srv::Function::kThrottle, plane_.get_adjusted_throttle_input(true));
}

inline void Mode::output_rudder_and_steering(float val) {
    plane_.srv_channels.set_output_scaled(srv::Function::kRudder, val);
    plane_.srv_channels.set_output_scaled(srv::Function::kSteering, val);
}

// ---------------------------------------------------------------------
// ModeManual - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeManual::update(const StabilizeInputs&) {
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

inline void ModeManual::run(const StabilizeInputs&) { reset_controllers(); }

// ---------------------------------------------------------------------
// ModeFBWA - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeFBWA::update(const StabilizeInputs&) {
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
    plane_.nav_pitch_cd = math::constrain_value(plane_.nav_pitch_cd, static_cast<std::int32_t>(plane_.pitch_limit_min * 100.0f),
                                                  static_cast<std::int32_t>(plane_.aparm.pitch_limit_max_deg * 100.0f));
    if (plane_.fly_inverted()) {
        plane_.nav_pitch_cd = -plane_.nav_pitch_cd;
    }
}

inline void ModeFBWA::run(const StabilizeInputs& in) {
    Mode::run(in);
    output_pilot_throttle();
}

// ---------------------------------------------------------------------
// ModeFBWB - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeFBWB::update(const StabilizeInputs& in) {
    plane_.nav_roll_cd =
        static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
    plane_.update_load_factor();
    plane_.update_fbwb_speed_height(in);
}

// ---------------------------------------------------------------------
// ModeCRUISE - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline void ModeCRUISE::update(const StabilizeInputs& in) {
    if (plane_.channel_roll()->control_in != 0 || plane_.rudder_input() != 0) {
        locked_heading_ = false;
        lock_timer_ms_ = 0;
    }

    if (!locked_heading_) {
        plane_.nav_roll_cd =
            static_cast<std::int32_t>(plane_.channel_roll()->norm_input() * static_cast<float>(plane_.roll_limit_cd));
        plane_.update_load_factor();
    } else {
        plane_.calc_nav_roll(in);
    }
    plane_.update_fbwb_speed_height(in);
}

inline void ModeCRUISE::navigate(const StabilizeInputs& in) {
    const ahrs::GpsSample& gps_sample = plane_.gps.sample();
    const std::int32_t ground_course_cd = static_cast<std::int32_t>(gps_sample.ground_course_deg * 100.0f);
    const bool moving_forwards = std::fabs(math::wrap_PI(
        math::cd_to_rad(static_cast<float>(ground_course_cd)) - plane_.ahrs.yaw)) < static_cast<float>(M_PI_2);

    if (!locked_heading_ && plane_.channel_roll()->control_in == 0 && plane_.rudder_input() == 0 && gps_sample.has_fix &&
        gps_sample.ground_speed_ms >= kGpsGndCrsMinSpd && moving_forwards && lock_timer_ms_ == 0) {
        // user wants to lock the heading - start the timer.
        lock_timer_ms_ = in.now_ms;
    }
    if (lock_timer_ms_ != 0 && (in.now_ms - lock_timer_ms_) > 500U) {
        // lock the heading after 0.5 seconds of zero heading input from
        // the pilot.
        locked_heading_ = true;
        lock_timer_ms_ = 0;
        locked_heading_cd_ = ground_course_cd;
        plane_.prev_WP_loc = plane_.current_loc;
    }
    if (locked_heading_) {
        plane_.next_WP_loc = plane_.prev_WP_loc;
        // always look 1km ahead.
        plane_.next_WP_loc.offset_bearing(static_cast<float>(locked_heading_cd_) * 0.01f,
                                           plane_.prev_WP_loc.get_distance(plane_.current_loc) + 1000.0f);
        const nav::L1Inputs l1_in = plane_.build_l1_inputs(in);
        plane_.nav_controller.update_waypoint(plane_.prev_WP_loc, plane_.next_WP_loc, l1_in);
    }
}

// ---------------------------------------------------------------------
// ModeAUTO - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp (both the ModeAUTO class banner and the CPP-031
// SLICE 7 ADDENDUM's "MISSION-COMPLETE-TO-RTL"/"HOME-BEFORE-AUTO-RTL"
// notes).
// ---------------------------------------------------------------------

inline bool ModeAUTO::enter() {
    // CPP-031 slice 7 - see plane.hpp's own "HOME-BEFORE-AUTO-RTL" note:
    // a real, minimal fallback for a caller that never called
    // plane.set_home() at all - treats "home is still exactly the
    // untouched default Location()" as "never explicitly set", and sets
    // it to wherever THIS mission is starting from. Does NOT override any
    // other value a caller may have already set.
    if (plane_.home.lat == 0 && plane_.home.lng == 0 && plane_.home.alt == 0) {
        plane_.set_home(plane_.current_loc);
    }

    plane_.next_WP_loc = plane_.prev_WP_loc = plane_.current_loc;
    if (plane_.mission.current() != nullptr) {
        plane_.do_nav_wp();
    }
    return true;
}

inline void ModeAUTO::update(const StabilizeInputs& in) {
    plane_.update_auto_speed_height(in);
    plane_.calc_nav_roll(in);
    plane_.calc_nav_pitch();
    plane_.calc_throttle();
}

inline void ModeAUTO::navigate(const StabilizeInputs& in) {
    if (plane_.mission.current() == nullptr) {
        return; // no mission loaded
    }
    const nav::L1Inputs l1_in = plane_.build_l1_inputs(in);
    if (plane_.verify_nav_wp(l1_in)) {
        if (plane_.mission.advance()) {
            plane_.do_nav_wp();
        } else {
            // CPP-031 slice 7 - reached/passed the FINAL waypoint with no
            // more legs to advance to: the real upstream mission-complete
            // trigger (AP_Mission::complete() -> Plane::
            // exit_mission_callback() -> `if (control_mode == &mode_auto)
            // set_mode(mode_rtl, ...)` - see plane.hpp's own "CPP-031
            // SLICE 7 ADDENDUM" "MISSION-COMPLETE-TO-RTL" note for the
            // full trace) reproduced directly at the one place this
            // port's own smaller Mission detects the same condition.
            // Replaces slice 5's own "hold the final leg forever" no-op -
            // the documented gap this slice closes.
            plane_.set_mode(plane_.mode_rtl);
        }
    }
}

// ---------------------------------------------------------------------
// ModeRTL - out-of-line bodies. Declarations + full judgment-call
// documentation: plane.hpp.
// ---------------------------------------------------------------------

inline bool ModeRTL::enter() {
    plane_.prev_WP_loc = plane_.current_loc;
    plane_.do_RTL(plane_.get_RTL_altitude_cm());
    plane_.rtl.done_climb = false;
    return true;
}

// CPP-034 FIX - see plane.hpp's own "UPDATE_AUTO_SPEED_HEIGHT()" note
// (Plane class, just above that method) and calc_throttle()'s own doc
// comment ("This is called by TECS-enabled flight modes"): any mode that
// calls calc_nav_pitch()/calc_throttle() only READS Tecs's last computed
// pitch/throttle demand - something else has to actually DRIVE that
// demand (tecs.update_50hz() + tecs.update_pitch_throttle(), bundled as
// update_auto_speed_height()) earlier the SAME tick, exactly as
// ModeAUTO::update() does above. ModeRTL::update() has called calc_nav_
// pitch()/calc_throttle() since RTL was first added (CPP-031 slice 6) but
// NEVER called update_auto_speed_height() - a genuine port-side gap, not
// an upstream-fidelity choice: upstream's real TECS update runs from a
// separate, mode-independent scheduled task (Plane::update_speed_height(),
// scheduler_tasks[], called every loop regardless of control_mode), so
// upstream never needed a "which mode is responsible for driving TECS"
// convention at all. This port chose instead to drive TECS from inside
// each auto-throttle mode's own update() (see UPDATE_AUTO_SPEED_HEIGHT()/
// UPDATE_FBWB_SPEED_HEIGHT()'s own notes) - a reasonable substitute for
// the missing scheduler, but that self-imposed convention was simply
// never extended to ModeRTL when slice 6 added it.
//
// EFFECT, ROOT-CAUSED VIA A REAL CLOSED-LOOP REPRO (CPP-034 ticket -
// vehicle_test.cpp's own "Closed loop: CRUISE-then-RTL converges toward
// home" test, which FAILED to converge before this fix and converges
// after it, with no other change): without this call, tecs.get_pitch_
// demand()/get_throttle_demand() (read by calc_nav_pitch()/calc_throttle()
// below) stayed FROZEN at whatever the PREVIOUS active mode last computed
// - e.g. ModeCRUISE's own last update_fbwb_speed_height() call, tuned for
// level flight at CRUISE's OWN altitude/speed, not RTL's real RTL_ALTITUDE
// climb target (do_RTL() sets target_altitude_cm correctly, but nothing
// ever fed it to Tecs again once RTL took over). The aircraft therefore
// never actually climbed to home.alt+RTL_ALTITUDE, and flew its entire
// loiter approach on a stale throttle/pitch trim never re-tuned for RTL's
// own speed/energy regime - which combined with L1Control's loiter
// capture-then-circle law (l1_control.hpp) to produce a large,
// non-decaying orbit oscillation (radius swinging roughly 45m-330m,
// never settling) instead of the tight, steady loiter every OTHER passing
// RTL closed-loop test already reaches. RTL-alone and AUTO-then-RTL both
// already converged fine despite this same gap - their own frozen/default
// Tecs demand at the moment RTL took over happened to still be close
// enough to survivable for L1's lateral loop to visibly work - which is
// exactly why this was invisible until the RC-failsafe slice's own agent
// tried CRUISE-then-RTL specifically (see vehicle_test.cpp's own "WHY
// AUTO, NOT CRUISE" note, CPP-031 slice 8) and flagged it for this ticket
// rather than assuming CRUISE was simply unlucky.
inline void ModeRTL::update(const StabilizeInputs& in) {
    plane_.update_auto_speed_height(in);
    plane_.calc_nav_roll(in);
    plane_.calc_nav_pitch();
    plane_.calc_throttle();

    if (plane_.aparm.rtl_climb_min <= 0.0f) {
        return;
    }

    // when RTL first starts, limit bank angle to LEVEL_ROLL_LIMIT
    // until we have climbed by RTL_CLIMB_MIN meters.
    const bool alt_threshold_reached =
        static_cast<float>(plane_.current_loc.alt - plane_.prev_WP_loc.alt) * 0.01f > plane_.aparm.rtl_climb_min;

    if (!plane_.rtl.done_climb && alt_threshold_reached) {
        plane_.prev_WP_loc = plane_.current_loc;
        // setup_alt_slope() - deferred, see plane.hpp's own note;
        // nothing left to do in this port's flat-altitude model.
        plane_.rtl.done_climb = true;
    }
    if (!plane_.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if
        // something goes wrong the plane will eventually turn back
        // and go to RTL instead of going perfectly straight. This
        // also leaves some leeway for fighting wind.
        plane_.roll_limit_cd = std::min(plane_.roll_limit_cd, static_cast<std::int32_t>(plane_.aparm.level_roll_limit_deg * 100.0f));
        plane_.nav_roll_cd = math::constrain_value(plane_.nav_roll_cd, -plane_.roll_limit_cd, plane_.roll_limit_cd);
    }
}

inline void ModeRTL::navigate(const StabilizeInputs& in) {
    const std::uint16_t radius = static_cast<std::uint16_t>(std::fabs(plane_.aparm.rtl_radius));
    if (radius > 0) {
        plane_.loiter.direction = (plane_.aparm.rtl_radius < 0.0f) ? -1 : 1;
    }
    plane_.update_loiter(radius, in);
}

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
// slice's modes, none of which is mode_training or scripting-driven).
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
//
// CPP-031 SLICE 4 NOTE: adds step 5b (plane.update_current_loc()) and the
// mode.navigate(in) call in step 6, both new shared infrastructure this
// slice's ModeCRUISE needs. See plane.hpp's file banner addendum for
// current_loc's own design rationale, and plane.hpp's own "CPP-031 SLICE 4
// ADDENDUM" note (on the Mode class hierarchy section) for the navigate()-
// vs-update()/run() ordering decision and its upstream justification.
// Neither change alters MANUAL/FBWA/FBWB's behavior: update_current_loc()
// only writes plane.current_loc (a field no pre-existing mode reads), and
// navigate() is a no-op on every mode but ModeCRUISE (Mode's own base
// implementation).
//
// CPP-031 SLICE 7 NOTE - THE REAL PAYOFF OF THIS SLICE: `mode` is bound
// ONCE, at the top of this function, from `plane.control_mode` - NOT
// re-read at each of the three dispatch points below. This is a
// deliberate choice, not an oversight: it reproduces upstream's own real
// same-iteration ordering, traced directly rather than assumed. Upstream's
// `control_mode->navigate()` (a separate, slower-rate SCHED_TASK - see
// this file's own "CPP-031 SLICE 4 ADDENDUM" note on the timing
// investigation) runs, WHEN IT FIRES, strictly AFTER that same loop
// iteration's `control_mode->update()`/`run()` (both FAST_TASKs, always
// run first) - so if navigate() triggers a mode switch via set_mode()
// (e.g. ModeAUTO's own mission-complete transition, plane.hpp), that
// iteration's update()/stabilize() have ALREADY run against the OLD mode;
// only the NEXT iteration's update_control_mode()/stabilize() (and
// navigate()) read the new `control_mode`. Binding `Mode& mode =
// *plane.control_mode;` once here and reusing it for navigate()/update()/
// run() reproduces exactly that: this tick()'s own update()/run() finish
// out against whichever mode was active when THIS tick() call began, even
// if navigate() (called first, per SLICE 4's own ordering choice) just
// switched `plane.control_mode` out from under it - the switch is only
// OBSERVABLE, and only dispatched through, starting the FOLLOWING tick()
// call, which re-reads `plane.control_mode` fresh. Verified directly by a
// dedicated test (vehicle_test.cpp) that installs a mode whose navigate()
// calls set_mode() mid-tick and confirms THIS tick's update()/run() still
// ran on the old mode, with the switch only visible on the next tick().
//
// CPP-031 SLICE 8 NOTE: adds step 1b (plane.update_throttle_failsafe()/
// plane.check_short_rc_failsafe()) - see plane.hpp's own "CPP-031 SLICE 8
// ADDENDUM" file banner for the full RC short (throttle) failsafe design.
// Placed immediately after step 1's RC read, matching upstream's own
// adjacent same-rate scheduling of read_radio()/control_failsafe() and
// check_short_rc_failsafe() (Plane.cpp's scheduler_tasks[], priorities 6
// and 9, both 50Hz). Neither call changes any EXISTING mode's behavior
// for a caller that never lets rc_failsafe actually latch (the real,
// in-scope default: THR_FAILSAFE defaults Enabled, but FixedWingTunables::
// fs_action_short/throttle_fs_value/rc_fs_timeout_ms all default to
// upstream's own real values, and every existing test's own
// set_sticks()-driven throttle PWM stays comfortably above
// THR_FS_VALUE's default threshold (950) every tick it runs) - verified
// directly by running every pre-existing test unchanged (vehicle_test.cpp).
inline void tick(Plane& plane, const ahrs::GyroSample& gyro_sample, const StabilizeInputs& in) {
    Mode& mode = *plane.control_mode; // see this function's own "CPP-031 SLICE 7 NOTE" above

    // 1. pull RC input (upstream: AP_Vehicle's read_radio() scheduled task)
    plane.rc_channels.read_input(plane.hal.rc_input);

    // 1b. RC short (throttle) failsafe - CPP-031 slice 8, see plane.hpp's
    //     own "CPP-031 SLICE 8 ADDENDUM" file banner for the full design
    //     (detection debounce, mode-switch on/off events, and the
    //     NEW-FRAME-DETECTION note explaining why this reads
    //     RcChannels::input_update_count() rather than re-checking
    //     RcInput::new_input(), already consumed by vehicle_test.cpp's own
    //     set_sticks() helper in every closed-loop test that calls it
    //     before tick()). Matches upstream's own adjacent, same-rate
    //     scheduling (read_radio() -> control_failsafe() ->
    //     check_short_rc_failsafe(), Plane.cpp's scheduler_tasks[]
    //     priorities 6/9, both 50Hz) - this port's single fixed-sequence
    //     tick() reproduces that ordering directly rather than needing a
    //     separate task-table entry.
    plane.update_throttle_failsafe(in.now_ms);
    plane.check_short_rc_failsafe();

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
    //    doc comments in ahrs_dcm.hpp). CPP-035: plane.compass is now a
    //    real (fixed-earth-field) compass model - see plane.hpp's file
    //    banner "CPP-035 ADDENDUM" and modules/ap-compass/include/fwcpp/
    //    compass/compass.hpp's own file banner for the full design. It is
    //    only updated this tick when the caller's StabilizeInputs::
    //    compass_healthy is true (in.compass_field_bf then holds the
    //    already-body-frame field the caller computed from true attitude -
    //    Compass::update() itself never touches attitude, see compass.hpp's
    //    "WHO COMPUTES..." note) - a caller that never populates these two
    //    fields gets EXACTLY the prior behavior (plane.compass.sample()
    //    stays default-constructed, healthy=false, forever). With a real
    //    compass wired in, drift_correction_yaw()'s use_compass() prefers
    //    it over GPS ground course - see ahrs_dcm.hpp's use_compass() - so
    //    yaw drift can now be corrected even below kGpsSpeedMinMs (3 m/s),
    //    closing the gap CPP-035's own ticket exists to close.
    if (in.compass_healthy) {
        plane.compass.update(in.compass_field_bf, in.now_us);
    }
    const ahrs::CompassSample compass = plane.compass.sample();
    const ahrs::GpsSample& gps_sample = plane.gps.sample();
    const float wind_speed_ms = in.wind_estimate.xy().length(); // see plane.hpp's file banner addendum
    const float airspeed_tas = in.airspeed_valid ? in.airspeed_eas * in.eas2tas : 0.0f; // matches ap-tecs's own EAS*eas2tas->TAS precedent
    // CPP-031 slice 9: armed_and_safety_off is now COMPUTED from the real
    // Plane::armed/RcOutput::safety_state() this slice wires together,
    // not a StabilizeInputs field a caller sets directly - see plane.hpp
    // file banner's "IS_ARMED_AND_SAFETY_OFF() BECOMES COMPUTED" note.
    const bool armed_and_safety_off = plane.is_armed_and_safety_off();

    plane.ahrs.accumulate_accel(in.accel_sample, in.dt);
    plane.ahrs.drift_correction_yaw(compass, gps_sample, plane.fly_forward(), armed_and_safety_off, in.gps_use_enabled,
                                     wind_speed_ms, in.now_ms);
    plane.ahrs.drift_correction_accel(compass, gps_sample, plane.fly_forward(), armed_and_safety_off, in.gps_use_enabled,
                                       in.wind_estimate, airspeed_tas, plane.accel_healthy(), plane.ins_healthy(), in.now_ms);

    // 4. scaled roll/pitch limits from current attitude (upstream: the
    //    rest of Plane::ahrs_update())
    plane.update_flight_limits();

    // 5. speed-scaler low-pass filter (upstream: calc_airspeed_errors(),
    //    normally a separate 10Hz scheduled task - see Plane::
    //    update_speed_scaler()'s own doc comment)
    plane.update_speed_scaler(in.airspeed_valid, in.airspeed_eas, armed_and_safety_off, in.dt);

    // 5b. current position as a Location (upstream: nothing this simple -
    //     see plane.hpp's file banner addendum). Always called, every mode:
    //     cheap, and no pre-existing mode reads plane.current_loc, so this
    //     cannot change MANUAL/FBWA/FBWB's behavior.
    plane.update_current_loc(in.position_ned);

    // 6. navigate + mode update + stabilize (upstream: Plane::navigate() ->
    //    control_mode->navigate(), then Plane::stabilize()). See this
    //    file's own "CPP-031 SLICE 4 ADDENDUM" note (plane.hpp, Mode class
    //    hierarchy section) for why mode.navigate(in) is called HERE -
    //    before update()/run(), not after - despite upstream's real
    //    navigate() being a separate, slower-rate task that (when it does
    //    fire) runs AFTER that same iteration's stabilize(). See this
    //    function's own "CPP-031 SLICE 7 NOTE" above for why `mode` is
    //    bound once, at the top of this function, rather than re-read at
    //    each dispatch point.
    if (in.now_ms - plane.last_stabilize_ms > 2000U) {
        mode.reset_controllers();
    }
    plane.last_stabilize_ms = in.now_ms;
    mode.navigate(in);
    mode.update(in);
    mode.run(in);

    // 7. write computed PWM to hardware (upstream: Plane::set_servos() ->
    //    SRV_Channels::output_ch_all())
    plane.srv_channels.output_ch_all(plane.hal.rc_output);
}

} // namespace fwcpp::vehicle
