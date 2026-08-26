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
// compensation()/update_target_altitude()/pre_arm_checks()/enter()/exit()
// are NOT PORTED - mode-IDENTIFICATION and mode-SWITCHING machinery, not
// stabilization logic, out of scope for a fixed two-mode slice with no
// runtime mode-change support. Each depends on a subsystem this port
// hasn't built (fence/mission/camera/ADSB/arming/battery/TECS-driven
// navigation) or on modes this slice doesn't implement. navigate() WAS in
// this list through CPP-031 slice 3 (no mode needed it yet) - CPP-031
// slice 4 (ModeCRUISE, below) is the first mode that does, so it is now
// ported as a default-no-op virtual hook on Mode itself - see Mode::
// navigate()'s own comment below for why the base class needed a change
// rather than adding this only to ModeCRUISE.
//
// Mode::run()'s StickMixing switch (stabilize_stick_mixing_fbw/direct) is
// skipped entirely - see plane.hpp's banner. Every mode in this slice
// behaves as upstream's StickMixing::NONE case (the real default,
// STICK_MIXING param default 0).
//
// =====================================================================
// CPP-031 SLICE 4 ADDENDUM (ModeCRUISE): see ModeCRUISE's own class banner
// (below) for its full design rationale, and plane.hpp's own file banner
// addendum for current_loc/nav_controller's design. This note covers only
// the ONE shared-infrastructure change this slice makes: tick()'s own
// sequencing.
//
// WHERE navigate() RUNS, RELATIVE TO update()/run() - INVESTIGATED, NOT
// ASSUMED, per the ticket's own instruction. Read upstream directly
// (ArduPlane/Plane.cpp's scheduler_tasks[] table, ArduPlane/navigation.cpp's
// Plane::navigate()) rather than guessing from mode.h's `virtual void
// navigate()` declaration alone:
//   - Plane::navigate() (navigation.cpp) is Mode::navigate()'s REAL caller
//     upstream - NOT Mode::run()/Attitude.cpp's stabilize(). It does its own
//     housekeeping (check_home_alt_change(), waypoint distance/bearing
//     bookkeeping) THEN calls `control_mode->navigate();` (the Mode-level
//     hook) near the end of its own body.
//   - Plane::navigate() is registered as `SCHED_TASK(navigate, 10, 150,
//     36)` - a SEPARATE, independently-10Hz-rate-limited scheduled task,
//     NOT a FAST_TASK. By contrast, `FAST_TASK(ahrs_update)`,
//     `FAST_TASK(update_control_mode)` (Mode::update()'s real caller), and
//     `FAST_TASK(stabilize)` (Mode::run()'s real caller) all run on EVERY
//     loop iteration (typically 50-400Hz, whatever SCHED_LOOP_RATE is
//     configured to) - unconditionally, before ANY of the slower
//     SCHED_TASK-rate-limited tasks (read_radio, the two GPS tasks,
//     navigate, ...) even get a chance to run within that same iteration's
//     remaining time budget. So on an iteration where navigate() actually
//     fires, it runs STRICTLY AFTER that same iteration's update_control_
//     mode()/stabilize() calls - meaning any Mode::navigate() side effect
//     (locking a heading, moving next_WP_loc, calling nav_controller->
//     update_waypoint()) is only visible to the FOLLOWING iteration's
//     calc_nav_roll()/nav_controller->nav_roll_cd() read, not the current
//     one. Two real facts, confirmed by reading the scheduler table and
//     navigation.cpp directly: (1) navigate() runs at 10Hz, far slower than
//     stabilize()'s fast-loop rate; (2) even within an iteration where it
//     does fire, it runs AFTER, not before, that iteration's stabilize().
//
// THIS PORT'S CHOICE: mode.navigate(in) is called ONCE PER TICK, BEFORE
// mode.update()/mode.run() (see tick() below) - i.e. ALWAYS FRESH, not
// stale by up to one whole navigate-task-period the way upstream's real
// timing is. This is a deliberate, DOCUMENTED divergence from upstream's
// literal timing, for three reasons, not an oversight:
//   1. PRECEDENT: this port's tick() (CPP-031 slice 1's own "SHAPE CHOICE"
//      banner note, "a single fixed sequence suffices") already made the
//      exact same call for update_speed_scaler() (calc_airspeed_errors()'s
//      real upstream task is also a separate 10Hz SCHED_TASK) - called
//      every tick, using the real per-tick dt, rather than resurrecting an
//      artificial 10Hz task boundary this port has no scheduler to express.
//      This slice's choice for navigate() is the same call, for the same
//      reason, made once already for the sibling function nearest it in
//      the scheduler table.
//   2. SAFETY: L1Control::update_waypoint() already self-clamps its own
//      internal dt to <=0.1s (l1_control.hpp) regardless of how often it's
//      called, and its crosstrack-integrator reset (`dt > 1.0f`) only helps
//      guard against a large gap, not a small one - calling it every 20-
//      50ms tick instead of every 100ms is strictly WITHIN its own designed
//      operating range, not a novel or riskier calling pattern.
//   3. NO COLD-START GAP: placing navigate() BEFORE update()/run() in the
//      SAME tick means the very first tick after the heading actually locks
//      already has a real, freshly-computed next_WP_loc/nav_controller
//      state for that same tick's calc_nav_roll() to read - avoiding an
//      extra one-tick "stale/uninitialized waypoint" special case that
//      placing it after update()/run() (mimicking upstream's literal
//      one-iteration lag) would otherwise introduce for no benefit, since
//      this port has no scheduler-driven reason to prefer the lag.
// This ordering can only make CRUISE's guidance MORE current than upstream,
// never less - a fresher world model is never a correctness regression for
// a low-bandwidth trim/guidance loop like this one, and no existing
// MANUAL/FBWA/FBWB test observes navigate() at all (its default is a no-op
// on every mode but ModeCRUISE), so this change to the SHARED tick()
// sequence is verified not to alter any of their behavior - see
// vehicle_test.cpp's own unchanged-tests confirmation.
//
// =====================================================================
// CPP-031 SLICE 5 ADDENDUM (ModeAUTO): see plane.hpp's own file banner
// addendum for the full shared-infrastructure design rationale
// (MissionItem/Mission/kMaxMissionItems, the crosstrack state machine,
// the flat-altitude simplification, update_auto_speed_height()'s own
// reason for existing separately from update_fbwb_speed_height(), and the
// full exclusion list). This note covers only ModeAUTO's own mode-level
// shape - no change to tick() itself is needed: navigate() already runs
// before update()/run() for every mode (added for ModeCRUISE, SLICE 4).
//
// upstream (Plane-4.7.0, read in full per the ticket): ArduPlane/
// mode_auto.cpp (202 lines) - ModeAuto::_enter()/update()/navigate(). This
// slice's update()/navigate() cover ONLY the normal NAV_WAYPOINT case -
// upstream's own MAV_CMD_NAV_TAKEOFF/MAV_CMD_NAV_LAND/
// MAV_CMD_NAV_SCRIPT_TIME/quadplane special-case branches in update() are
// skipped entirely (not even a recognized-but-unimplemented dispatch arm -
// MissionItem's vocabulary has no such commands to dispatch on, see
// plane.hpp's exclusion list).
//
// _ENTER() - NOT AUTOMATICALLY CALLED, matching ModeFBWB/ModeCRUISE's own
// precedent (this port has no mode-switching machinery yet). Upstream's
// real _enter() body (after the quadplane/watchdog-resume/soaring
// branches, all excluded - no such subsystems) is: `plane.next_WP_loc =
// plane.prev_WP_loc = plane.current_loc; plane.mission.start_or_resume();`
// - start_or_resume() ultimately reaches set_current_cmd(0) ->
// start_command(cmd) -> do_nav_wp(cmd) for the mission's first NAV_WAYPOINT
// (this slice's vocabulary has no jump/DO_LAND_START resume point to make
// start_or_resume() do anything more interesting than "start at index 0").
// Reproduced directly below as enter(): a caller MUST call
// `plane.mission.load(...)` THEN this method ONCE, before the first
// tick()/update() while ModeAUTO is active.
//
// UPDATE() - the real body for a normal NAV_WAYPOINT command is exactly
// `plane.calc_nav_roll(); plane.calc_nav_pitch(); plane.calc_throttle();`
// (all three already exist, from FBWB/CRUISE) - but see plane.hpp's
// "UPDATE_AUTO_SPEED_HEIGHT()" note for why this port needs one more call
// first (plane.update_auto_speed_height(in)) to actually drive TECS before
// calc_nav_pitch()/calc_throttle() read its demand - upstream's real
// driver (Plane::update_alt()) lives outside ModeAuto::update() entirely,
// in a separate scheduled task this port's tick() has no slot for, so it
// is folded in here instead, exactly as SLICE 2 already folded FBWB's own
// TECS-driving call into ModeFBWB::update().
//
// NAVIGATE() - upstream's real body is `if (AP::ahrs().home_is_set()) {
// plane.mission.update(); }` where AP_Mission::update() is the ~200-line
// state machine this slice deliberately does NOT port (see plane.hpp's
// "SCOPE" note) - dispatching to do_nav_wp()/verify_nav_wp() for whatever
// command is current, advancing on completion, resuming across a mission-
// index jump, etc. THIS slice's navigate() below is the minimal, real
// equivalent for exactly this slice's one command type (NAV_WAYPOINT):
// build one L1Inputs snapshot (same "one caller-visible snapshot per tick"
// precedent as build_tecs_inputs()/build_l1_inputs() elsewhere), call
// verify_nav_wp() (which itself calls nav_controller.update_waypoint() -
// see plane.hpp, matching upstream's real verify_nav_wp() exactly), and on
// a true (reached-or-passed) result call mission.advance() + do_nav_wp()
// to move to the next leg - or, at the final waypoint, do neither (see
// plane.hpp's "MISSION COMPLETE" note). `AP::ahrs().home_is_set()`'s guard
// is dropped (no home/EKF-origin subsystem to have a "not set yet" state -
// this port's current_loc is always valid once tick() has run once, same
// treatment build_l1_inputs()'s own `location_valid = true UNCONDITIONALLY`
// note already documents).
//
// VERIFY_NAV_WP() SKIPPED LINES, EXACT UPSTREAM LOCATIONS (commands_logic.
// cpp Plane::verify_nav_wp, ~line 634, read in full per the ticket): the
// `uint8_t cmd_passby = HIGHBYTE(cmd.p1)` extraction and its
// flex_next_WP_loc offset_bearing() adjustment (pass-by is not part of
// MissionItem's vocabulary at all); the `g.waypoint_max_radius > 0`
// override block (waypoint_max_radius is not a ported tunable); the GCS
// text messages (gcs().send_text(...) - no GCS subsystem). Everything else
// - the crosstrack-vs-current-location nav_controller.update_waypoint()
// choice, the turn_distance()-derived acceptance radius, the plain
// distance check, and the past_interval_finish_line() "flew past it" catch
// - is real, faithfully-ported behavior (see plane.hpp's Plane::
// verify_nav_wp() for the port-side implementation).
//
// DOES_AUTO_NAVIGATION()/DOES_AUTO_THROTTLE()/RUN() - NOT PORTED, same
// "mode-IDENTIFICATION machinery, out of scope" exclusion this file's own
// banner already documents for every mode - ModeAUTO has NO run()
// override, relying entirely on base Mode::run() (stabilize all three
// axes), exactly like ModeFBWB/ModeCRUISE's own "auto-throttle mode relies
// on the base" shape (see their banners) - does_auto_throttle() is true
// for AUTO too, expressed structurally the same way.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/nav/l1_control.hpp>
#include <fwcpp/srv/srv_channels.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace fwcpp::vehicle {

// upstream: ArduPlane/defines.h's GPS_GND_CRS_MIN_SPD (5, m/s) - "used to
// set when initial_direction.heading is captured, deciding to heading lock
// in cruise mode" (upstream's own comment, verified directly against
// defines.h line 13, not assumed).
inline constexpr float kGpsGndCrsMinSpd = 5.0f;

class Mode {
public:
    explicit Mode(Plane& plane) : plane_(plane) {}
    virtual ~Mode() = default;
    Mode(const Mode&) = delete;
    Mode& operator=(const Mode&) = delete;

    // upstream: Mode::update() (pure virtual) - convert pilot/mode input
    // into nav_roll_cd/nav_pitch_cd targets and/or direct servo output.
    virtual void update(const StabilizeInputs& in) = 0;

    // upstream: Mode::navigate() (mode.h) - "virtual void navigate() {
    // return; }" - a default NO-OP hook, overridden only by navigation
    // modes (ModeCruise/ModeAuto/ModeLoiter/etc - mode.h). MANUAL/FBWA/
    // FBWB never override this (none of them do any waypoint/heading-lock
    // bookkeeping), so the base no-op is their entire behavior here,
    // exactly matching upstream. See tick()'s own comment below (CPP-031
    // "slice 4" note) for WHERE this is called from and why, added to the
    // Mode base for the first time in this slice (ModeCRUISE, mode.hpp
    // below) - a change to shared, already-tested infrastructure, so its
    // rationale is documented here rather than only at the one new call
    // site.
    virtual void navigate(const StabilizeInputs&) {}

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

// upstream: ModeCruise (mode.h) + mode_cruise.cpp, read in full (CPP-031
// "slice 4") - the first mode in this port to do real GPS-based
// navigation. In CRUISE, aileron/rudder sticks directly command roll
// (exactly like FBWA) UNTIL the pilot centers both sticks and holds still
// for 0.5 seconds, at which point the current GPS ground course is
// "locked" as a heading to hold and L1Control takes over roll guidance,
// flying a straight line along that locked heading (a virtual waypoint
// projected 1km ahead). FBWB's elevator/altitude/airspeed logic
// (update_fbwb_speed_height()) is reused UNCHANGED - CRUISE only adds the
// heading-lock/navigation layer on top, matching upstream's own
// update()'s final unconditional `plane.update_fbwb_speed_height();` call.
//
// STATE OWNERSHIP - matches upstream EXACTLY, not collapsed for
// convenience: locked_heading_/lock_timer_ms_/locked_heading_cd_ are
// ModeCruise's OWN members upstream (mode.h's ModeCruise class) - this
// mode's private navigation-lock state, never read outside it. prev_WP_loc/
// next_WP_loc, by contrast, are real Plane members upstream (Plane.h) -
// this port adds them to Plane (plane.hpp) for the same reason: they are
// exactly the two fields a future AUTO mode would also need to read/write
// (mission leg endpoints), so keeping them where upstream keeps them (not
// folding them into ModeCRUISE-private state) is what makes that future
// reuse possible without moving anything. See plane.hpp's file banner
// addendum for current_loc/nav_controller's own design rationale.
//
// _enter() IS NOT PORTED/CALLED AUTOMATICALLY - same exclusion ModeFBWB's
// own class banner already documents (this slice has no mode-switching
// machinery). Upstream's real _enter() body: `locked_heading = false;
// lock_timer_ms = 0; plane.set_target_altitude_current();` (the
// HAL_SOARING_ENABLED init_cruising() call is excluded - no soaring
// subsystem, same as ModeFBWB's). The first two assignments are already
// this class's own default member initializers below (a freshly-
// constructed ModeCRUISE starts unlocked with no timer running, with no
// extra call needed) - the ONLY action a caller must take explicitly
// before the first tick()/update(), exactly matching ModeFBWB's own
// precedent, is `plane.set_target_altitude_current(current_altitude_cm)`.
//
// EXCLUDED (documented, not silently dropped):
//   - AP_SCRIPTING_ENABLED's nav_scripting_active() checks (update()'s
//     stick-lock-input guard AND navigate()'s early return) - no scripting
//     subsystem in this port.
//   - HAL_SOARING_ENABLED's soaring_controller.init_cruising() (_enter())
//     - no soaring subsystem, same exclusion as ModeFBWB's.
//   - Any mission/AUTO-mode coupling - get_target_heading_cd() (ported
//     below, trivial) is used upstream by AUTO-mode-adjacent code for
//     logging/reporting only; nothing in this port's scope consumes it.
//   - aparm.rudder_only channel aliasing - same exclusion plane.hpp's
//     banner already documents for an unconfigured vehicle (rudder_only
//     defaults false, so channel_roll is never aliased to the yaw
//     channel).
//
// GENUINE UPSTREAM QUIRK, REPRODUCED FAITHFULLY, NOT FIXED (per this
// port's "port fixes bugs in the port, not upstream" rule) - discovered
// while writing this slice's own unit tests: lock_timer_ms_ == 0 doubles
// as BOTH "the timer is not running" (the sentinel every gating check
// above tests against) AND a legitimately-reachable real timestamp
// (in.now_ms == 0). Upstream has the IDENTICAL collision against
// AP_HAL::millis() (mode.h's own `uint32_t lock_timer_ms;`, mode_cruise.cpp's
// `lock_timer_ms == 0` checks) - immaterial in practice there because
// millis() is only ever 0 in the first millisecond after boot, long before
// a pilot could switch into CRUISE. This port's own vehicle_test.cpp had to
// deliberately start its unit tests' StabilizeInputs::now_ms at a realistic
// nonzero value for the same reason (see vehicle_test.cpp's own "TIMER
// SENTINEL" comment) - noted here as a real, traced-not-invented upstream
// characteristic worth flagging, not a defect introduced by this port.
class ModeCRUISE : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeCruise::update() (mode_cruise.cpp). "Heading becomes
    // unlocked on any aileron or rudder input" - control_in (dead-zone-
    // applied, matching upstream's own get_control_in()) rather than
    // norm_input(), exactly as upstream reads it here.
    void update(const StabilizeInputs& in) override {
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

    // upstream: ModeCruise::navigate() (mode_cruise.cpp) - the real
    // heading-lock state machine, read VERY carefully (per the ticket's own
    // instruction). See tick()'s own comment (below) for WHEN this runs
    // relative to update()/run() in this port.
    //
    // `plane.gps.ground_course_cd()` upstream is `ground_course() * 100`
    // (AP_GPS.h, a plain float-degrees-to-centidegrees scale) - this port's
    // GpsSample only carries ground_course_deg (float degrees, ap-gps's own
    // scope - see gps.hpp's file banner), so ground_course_cd is computed
    // the same way inline below rather than needing a new GpsSample field.
    //
    // `plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D` collapses to
    // `gps_sample.has_fix` - traced directly against GpsSample's own field
    // doc (ahrs_dcm.hpp): has_fix is defined as `status() > AP_GPS::NO_FIX`,
    // and AP_GPS's real status enum has NO_GPS=0 < NO_FIX=1 < GPS_OK_FIX_2D=2
    // < GPS_OK_FIX_3D=3 - so `status() > NO_FIX` (has_fix) and `status() >=
    // GPS_OK_FIX_2D` are the SAME condition (both mean status >= 2), not an
    // approximation of it.
    void navigate(const StabilizeInputs& in) override {
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

    // upstream: ModeCruise::get_target_heading_cd() (mode_cruise.cpp) -
    // trivial accessor, ported for completeness though nothing in this
    // port's scope consumes it (see class banner's EXCLUDED note).
    [[nodiscard]] bool get_target_heading_cd(std::int32_t& target_heading) const {
        target_heading = locked_heading_cd_;
        return locked_heading_;
    }

    // upstream: ModeCruise has NO run() override at all - same "auto-
    // throttle mode relies entirely on base Mode::run()" shape as ModeFBWB
    // (see its own banner) - does_auto_throttle() is true for CRUISE too.

private:
    // upstream: ModeCruise's own private members (mode.h) - see class
    // banner's "STATE OWNERSHIP" note.
    bool locked_heading_ = false;
    std::uint32_t lock_timer_ms_ = 0;
    std::int32_t locked_heading_cd_ = 0;
};

// upstream: ModeAuto (mode.h) + mode_auto.cpp (202 lines, read in full) -
// CPP-031 "slice 5". Flies a fixed-size, in-memory, ordered list of
// waypoint-only MissionItems (plane.hpp's Mission, this port's own
// deliberately smaller equivalent of AP_Mission) sequentially, using the
// SAME L1Control/TECS machinery CRUISE/FBWB already wired in - see this
// file's own "CPP-031 SLICE 5 ADDENDUM" note (above) for the full
// upstream-vs-port mapping, and plane.hpp's file banner addendum for the
// shared-infrastructure design rationale (MissionItem/Mission, the
// crosstrack state machine, the flat-altitude simplification,
// update_auto_speed_height()).
//
// STATE OWNERSHIP - matches CRUISE's own precedent: mission/next_wp_
// crosstrack/crosstrack/next_turn_angle all live on Plane (plane.hpp,
// matching upstream's own Plane.h placement for `mission` and
// `auto_state`), NOT as ModeAUTO-private members - ModeAUTO itself holds
// NO private state at all, unlike ModeCRUISE's locked_heading_/
// lock_timer_ms_/locked_heading_cd_ (there is nothing mode-local to track;
// every piece of AUTO's navigation state is exactly what a future mode
// reading prev_WP_loc/next_WP_loc - e.g. a real RTL - would also need).
//
// EXCLUDED (documented, not silently dropped):
//   - The MAV_CMD_NAV_TAKEOFF/MAV_CMD_NAV_LAND/MAV_CMD_NAV_SCRIPT_TIME/
//     quadplane special-case branches in update() - no such commands in
//     MissionItem's vocabulary (see plane.hpp's exclusion list).
//   - AP_SCRIPTING_ENABLED's nav_scripting_active()/wiggle_servos()/
//     MAV_CMD_NAV_ALTITUDE_WAIT handling in run() - no scripting
//     subsystem, and ModeAUTO has no run() override at all (see below).
//   - does_auto_navigation()/does_auto_throttle()/_pre_arm_checks()/
//     is_landing() - mode-IDENTIFICATION machinery, same exclusion this
//     file's own banner already documents for every mode.
//   - Watchdog mission-resume and HAL_SOARING_ENABLED's init_cruising()
//     (both in _enter()) - no watchdog-persistence or soaring subsystem.
class ModeAUTO : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeAuto::_enter() - see this file's own "CPP-031 SLICE 5
    // ADDENDUM" note for the real body this reproduces. NOT AUTOMATICALLY
    // CALLED (no mode-switching machinery in this port, same exclusion
    // ModeFBWB/ModeCRUISE's own banners document) - a caller MUST call
    // `plane.mission.load(...)` THEN this method ONCE, before the first
    // tick()/update() while ModeAUTO is active.
    void enter() {
        plane_.next_WP_loc = plane_.prev_WP_loc = plane_.current_loc;
        if (plane_.mission.current() != nullptr) {
            plane_.do_nav_wp();
        }
    }

    // upstream: ModeAuto::update() - the normal-NAV_WAYPOINT branch only
    // (see this file's own "CPP-031 SLICE 5 ADDENDUM" note for why
    // update_auto_speed_height() is called first).
    void update(const StabilizeInputs& in) override {
        plane_.update_auto_speed_height(in);
        plane_.calc_nav_roll(in);
        plane_.calc_nav_pitch();
        plane_.calc_throttle();
    }

    // upstream: ModeAuto::navigate() - see this file's own "CPP-031 SLICE
    // 5 ADDENDUM" note for the full upstream-vs-port mapping (this port's
    // do_nav_wp()/verify_nav_wp()/mission.advance() replacing AP_Mission::
    // update()'s much larger state machine for exactly this slice's one
    // command type).
    void navigate(const StabilizeInputs& in) override {
        if (plane_.mission.current() == nullptr) {
            return; // no mission loaded
        }
        const nav::L1Inputs l1_in = plane_.build_l1_inputs(in);
        if (plane_.verify_nav_wp(l1_in)) {
            if (plane_.mission.advance()) {
                plane_.do_nav_wp();
            }
            // else: reached/passed the FINAL waypoint - see plane.hpp's
            // "MISSION COMPLETE" note. Deliberately do NOT call
            // do_nav_wp()/set_next_WP() again: next_WP_loc stays pinned at
            // the last waypoint, and verify_nav_wp()'s own
            // nav_controller.update_waypoint() call above already re-ran
            // this tick, so the vehicle holds course on the final leg
            // indefinitely.
        }
    }

    // upstream: ModeAuto has NO run() override for the normal-flight case
    // (only the MAV_CMD_NAV_ALTITUDE_WAIT special case does, excluded -
    // see class banner) - relies entirely on base Mode::run(), same "auto-
    // throttle mode relies on the base" shape as ModeFBWB/ModeCRUISE.
};

// upstream: ModeRTL (mode.h) + mode_rtl.cpp (169 lines, read in full) -
// CPP-031 "slice 6". Navigates back to a fixed `home` point and loiters
// there - the FIRST mode in this port to use L1Control's loiter support
// (update_loiter()/reached_loiter_target()/loiter_radius(), ported by
// CPP-017 but never called by anything until now) and the first to need
// a persistent `home` concept - see plane.hpp's file banner addendum for
// the full design rationale (home/set_home(), why current_loc.alt is now
// real data, do_RTL()'s rally/terrain/alt-slope exclusions, update_
// loiter()'s single-mode-check simplification, the LoiterState/RtlState
// structs, and every new tunable's real upstream default).
//
// STATE OWNERSHIP - matches AUTO's own precedent exactly (mode.hpp's
// ModeAUTO class banner): ModeRTL itself holds NO private state at all.
// `home`/`loiter`/`rtl` all live on Plane (plane.hpp, matching upstream's
// own Plane.h placement) - `loiter` in particular is exactly the state a
// FUTURE mode (e.g. LOITER, GUIDED) would also need to read/write, the
// same "keep it where a future reuse would find it" reasoning CRUISE's
// own prev_WP_loc/next_WP_loc placement already established.
//
// _ENTER() - NOT AUTOMATICALLY CALLED, same "no mode-switching machinery
// yet" exclusion every prior mode's own banner documents. Upstream's real
// _enter() body (after every HAL_QUADPLANE_ENABLED branch, excluded - no
// quadplane in this port) is exactly `plane.prev_WP_loc = plane.
// current_loc; plane.do_RTL(plane.get_RTL_altitude_cm()); plane.rtl.
// done_climb = false;` - reproduced directly below as enter(). A CALLER
// MUST CALL plane.set_home(...) AT LEAST ONCE, THEN this method ONCE,
// before the first tick()/update() while ModeRTL is active - matching
// ModeFBWB/ModeCRUISE/ModeAUTO's own explicit-setup precedent. Unlike
// ModeAUTO::enter() (which takes no StabilizeInputs - it only touches
// current_loc/mission), this needed no StabilizeInputs parameter either:
// get_RTL_altitude_cm() kept its own real upstream zero-arg signature
// (see plane.hpp's own note) precisely because current_loc.alt is now
// live data, not a dead field needing an explicit substitute.
//
// UPDATE() - the in-scope subset (see file banner "ModeRTL" note in
// plane.hpp for what CLIMB_BEFORE_TURN exclusion means exactly): the
// three calc_nav_*() calls (all pre-existing, from FBWB/CRUISE/AUTO),
// then the REAL RTL_CLIMB_MIN "climb before turning" feature - a genuine,
// small, non-stub port, not a simplification of it. CLIMB_BEFORE_TURN's
// own FlightOptions bitmask branch (`plane.flight_option_enabled(...)`)
// is excluded - no such bitmask subsystem exists in this port (same
// exclusion this port has documented everywhere a FlightOptions check
// appears, e.g. plane.hpp's apply_load_factor_roll_limits()) - so this
// always reaches upstream's own `else if (plane.g2.rtl_climb_min > 0)`
// branch, which is the real, default-relevant path anyway (RTL_CLIMB_MIN
// default is 0 - see plane.hpp - so the whole clamp is a documented no-op
// for an unconfigured vehicle, exactly matching upstream).
//
// NAVIGATE() - the in-scope subset: `uint16_t radius = abs(g.rtl_radius);
// if (radius > 0) loiter.direction = ...; plane.update_loiter(radius);`,
// reproduced directly. EXCLUDED ENTIRELY (per the ticket, no partial
// dispatch stub): the HAL_QUADPLANE_ENABLED VTOL-approach-landing branch
// and switch_QRTL() (no quadplane in this port); the whole `!plane.
// auto_state.checked_for_autoland` autoland/mission-jump block
// (RTL_IMMEDIATE_DO_LAND_START/RTL_THEN_DO_LAND_START/DO_RETURN_PATH_
// START, jump_to_landing_sequence()/jump_to_closest_mission_leg()) - no
// landing subsystem, and this port's own Mission (SLICE 5) has no jump/
// leg-resume machinery to support it even partially.
class ModeRTL : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeRTL::_enter() - see class banner. NOT AUTOMATICALLY
    // CALLED - a caller MUST call plane.set_home(...) first, then this
    // method, before the first tick()/update() while ModeRTL is active.
    void enter() {
        plane_.prev_WP_loc = plane_.current_loc;
        plane_.do_RTL(plane_.get_RTL_altitude_cm());
        plane_.rtl.done_climb = false;
    }

    // upstream: ModeRTL::update() - see class banner for the
    // CLIMB_BEFORE_TURN exclusion.
    void update(const StabilizeInputs& in) override {
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

    // upstream: ModeRTL::navigate() - see class banner for the
    // autoland/mission-jump exclusion.
    void navigate(const StabilizeInputs& in) override {
        const std::uint16_t radius = static_cast<std::uint16_t>(std::fabs(plane_.aparm.rtl_radius));
        if (radius > 0) {
            plane_.loiter.direction = (plane_.aparm.rtl_radius < 0.0f) ? -1 : 1;
        }
        plane_.update_loiter(radius, in);
    }

    // upstream: ModeRTL has NO run() override at all - relies entirely on
    // base Mode::run(), same "auto-throttle mode relies on the base"
    // shape as ModeFBWB/ModeCRUISE/ModeAUTO.
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
//
// CPP-031 SLICE 4 NOTE: adds step 5b (plane.update_current_loc()) and the
// mode.navigate(in) call in step 6, both new shared infrastructure this
// slice's ModeCRUISE needs. See plane.hpp's file banner addendum for
// current_loc's own design rationale, and this file's own "CPP-031 SLICE 4
// ADDENDUM" note (above Mode's class definition) for the navigate()-vs-
// update()/run() ordering decision and its upstream justification. Neither
// change alters MANUAL/FBWA/FBWB's behavior: update_current_loc() only
// writes plane.current_loc (a field no pre-existing mode reads), and
// navigate() is a no-op on every mode but ModeCRUISE (Mode's own base
// implementation).
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

    // 5b. current position as a Location (upstream: nothing this simple -
    //     see plane.hpp's file banner addendum). Always called, every mode:
    //     cheap, and no pre-existing mode reads plane.current_loc, so this
    //     cannot change MANUAL/FBWA/FBWB's behavior.
    plane.update_current_loc(in.position_ned);

    // 6. navigate + mode update + stabilize (upstream: Plane::navigate() ->
    //    control_mode->navigate(), then Plane::stabilize()). See this
    //    file's own "CPP-031 SLICE 4 ADDENDUM" note (above) for why
    //    mode.navigate(in) is called HERE - before update()/run(), not
    //    after - despite upstream's real navigate() being a separate,
    //    slower-rate task that (when it does fire) runs AFTER that same
    //    iteration's stabilize().
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
