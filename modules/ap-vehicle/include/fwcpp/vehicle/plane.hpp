#pragma once

// Port of ArduPlane's Plane vehicle class - JUST the slice CPP-031 needs:
// calc_speed_scaler()/get_speed_scaler(), stabilize_roll()/stabilize_pitch()/
// stabilize_yaw(), update_load_factor()/apply_load_factor_roll_limits(),
// adjust_nav_pitch_throttle(), get_throttle_input()/get_adjusted_throttle_
// input()/throttle_percentage(), and the roll/pitch/rudder "expo" input
// helpers - everything ModeManual/ModeFBWA (mode.hpp, same module) call
// into. CPP-031 "slice 1": the ArduPlane vehicle skeleton, MANUAL + FBWA
// only. Upstream (Plane-4.7.0, read directly from the pinned worktree, not
// from training-data memory):
//   - ArduPlane/mode.h (1075 lines) + mode.cpp (414 lines) - Mode base
//     class (see mode.hpp, same module).
//   - ArduPlane/mode_manual.cpp (31 lines) + mode_fbwa.cpp (45 lines), in
//     full (see mode.hpp).
//   - ArduPlane/Attitude.cpp (764 lines), in full.
//   - ArduPlane/navigation.cpp - calc_airspeed_errors()'s speed-scaler
//     low-pass update line only.
//   - ArduPlane/radio.cpp - roll_in_expo/pitch_in_expo/rudder_in_expo,
//     channel_expo(), rudder_input(), get_throttle_input()'s siblings.
//   - ArduPlane/reverse_thrust.cpp - get_throttle_input()/
//     get_adjusted_throttle_input().
//   - ArduPlane/system.cpp - throttle_percentage().
//   - ArduPlane/control_modes.cpp - fly_inverted().
//   - ArduPlane/Plane.cpp - Plane::ahrs_update()'s roll_limit_cd/
//     pitch_limit_min scaling, and Plane::set_control_channels()/
//     init_rc_in() (radio.cpp) for RC/SRV channel setup.
//   - ArduPlane/Plane.h, ArduPlane/Parameters.cpp, ArduPlane/config.h,
//     ArduPlane/defines.h - grepped (NOT read in full - see the ticket's
//     own instruction) for every field's real type and default value,
//     cited inline on each field below.
//
// SCOPE - EXACTLY TWO FLIGHT MODES, NO MODE-SWITCHING MACHINERY: this is
// deliberately the smallest slice that produces a genuinely flyable
// vehicle, not a stub. Every piece of MANUAL/FBWA's real control-law math
// is ported faithfully; what's excluded is other subsystems (fence,
// mission, camera, ADSB, arming, quadplane, TECS-driven navigation,
// logging, GCS, failsafe, aux-function dispatch, ground steering) that
// this port has not built yet and that upstream's OWN #if-guards or
// dropped call sites already excise for a QUADPLANE-less, sensor-poor
// configuration - see each function's own comment below for the specific
// citation.
//
// NO SINGLETONS, EXPLICIT CONTEXT INSTEAD (ADR-0012), matching every
// module this slice wires together (AhrsDcm, RollController/
// PitchController/YawController, RcChannels/SrvChannels, Tecs):
//   - arming.is_armed_and_safety_off() -> StabilizeInputs::
//     armed_and_safety_off / calc_speed_scaler()'s own explicit parameter
//     (no arming subsystem in this port).
//   - ahrs.airspeed_EAS()/get_EAS2TAS() -> StabilizeInputs::
//     airspeed_valid/airspeed_eas/eas2tas (no airspeed-sensor subsystem;
//     same "explicit optional-or-defaulted parameter" precedent as
//     Tecs/SimPlane already established).
//   - AP::ins()'s bias-corrected lateral accel (yaw's accel_y) ->
//     StabilizeInputs::accel_y - see yaw_controller.hpp's own
//     YawCoordinationInputs::accel_y note; with YawController::Gains's
//     real k_d default of 0.0f (YAW2SRV_DAMP), get_servo_out() early-
//     returns 0 regardless of accel_y for an untuned vehicle, so this
//     stays inert until a caller wires up a real accelerometer AND tunes
//     yaw damping - not a shortcut, upstream's own default behavior.
//
// RC/SRV CHANNEL INDEX MAPPING - NO AP_RCMapper: upstream's channel_roll/
// channel_pitch/channel_throttle/channel_rudder are resolved through
// RC_Channels::get_roll_channel() etc, which consult AP_RCMapper (a
// remapping table this port hasn't ported) but default to the
// conventional RCMAP_ROLL=1/PITCH=2/THROTTLE=3/YAW=4 (1-indexed) mapping
// for an unconfigured vehicle. kChannelRoll/Pitch/Throttle/Rudder below
// hardcode that same default (0-indexed: 0/1/2/3) - correct for the
// default, unremapped configuration this slice targets. g.rudder_only's
// `channel_roll = &rc().get_yaw_channel()` aliasing (Plane::
// set_control_channels(), radio.cpp) is NOT reproduced for the same
// reason - aparm.rudder_only defaults false (RUDDER_ONLY's real upstream
// default), so that aliasing never fires for an unconfigured vehicle
// either. Likewise SRV output channel assignment (kServoAileron=0/
// Elevator=1/Throttle=2/Rudder=3) matches ArduPlane's conventional
// SERVO1..4 default mapping - see configure_channels() below, which also
// reproduces aux_servo_function_setup()'s real STANDARD-configuration
// defaults (SRV_Channel_aux.cpp: k_aileron/k_elevator/k_rudder/k_steering
// -> set_angle(4500); k_throttle -> set_range(100)) and Plane::
// set_control_channels()/init_rc_in()'s RC input side (set_angle(4500)/
// set_range(100), 30-unit dead zone) for the four primary channels.
//
// TECS: INCLUDED, FOR reset_controllers() PARITY ONLY. Mode::
// reset_controllers() (mode.cpp) calls `plane.TECS_controller.reset()`
// unconditionally, regardless of active mode - MANUAL/FBWA never read a
// TECS output (calc_nav_pitch(), TECS-driven, is a navigation-mode-only
// function not reachable from either mode in this slice), so this Tecs
// member exists purely so that call has a real, correctly-constructed
// target rather than being silently dropped. Gains{}/FixedWingParams{}
// are both fully upstream-defaulted (see ap-tecs/tecs.hpp) - never tuned
// or read back by anything in this slice.
//
// GROUND_MODE / REVERSED_THROTTLE: both upstream fields exist
// (Plane.h), but their real value depends on subsystems this port
// doesn't have (ground_mode: "true when disarmed and not flying" - an
// is_flying()/arming state machine; reversed_throttle: an RC aux-switch
// option, no aux-dispatch subsystem per CPP-027's own exclusion). Both
// are plain bool fields here, defaulted false exactly as an unconfigured,
// airborne vehicle would read them, with a caller free to set either
// explicitly once a real arming/aux-dispatch subsystem exists.
//
// STICK MIXING / GROUND STEERING - NOT PORTED (documented, not silently
// dropped): Mode::run()'s StickMixing switch (stabilize_stick_mixing_fbw/
// stabilize_stick_mixing_direct) and stabilize_yaw()'s ground-steering
// branch (calc_nav_yaw_course/calc_nav_yaw_ground, the `ground_steering`
// bool itself) are both secondary input-blending/taxi features, not core
// in-air stabilization - see mode.hpp and stabilize_yaw() below for
// exactly which upstream branch each collapses to instead. A caller may
// leave SrvChannel::kSteering unwired for ground ops in this slice, per
// the ticket's own allowance.
//
// SPEED-SCALER LOW-PASS "1Hz" COMMENT/CODE MISMATCH - REPRODUCED, NOT
// FIXED: upstream's calc_airspeed_errors() (navigation.cpp) comment says
// "low pass filter speed scaler, with 1Hz cutoff, at 10Hz" but the code
// two lines below sets `const float cutoff_Hz = 2.0;` - a genuine
// upstream comment/code mismatch. update_speed_scaler() below uses the
// CODE's real value (2.0f), per this port's "port fixes bugs in the
// port, not upstream" rule - the comment is upstream's own documentation
// bug, not something this port's behavior should silently diverge over.
//
// DECLARATION-ORDER CONSTRAINT: `aparm` MUST be declared before
// roll_controller/pitch_controller/yaw_controller/tecs below (C++ runs
// member initialization in DECLARATION order regardless of the
// constructor's init-list order - same caveat AhrsDcm's own constructor
// banner and Scheduler's file banner both call out). The four controller
// members are constructed from fw_aparm()/tecs_aparm(), which read
// `aparm`'s fields - if aparm were declared later, those reads would see
// not-yet-initialized memory.
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant
// below is an explicit float-suffixed literal, matching upstream's own
// values (verified against Parameters.cpp/config.h/defines.h, not
// invented).
//
// =====================================================================
// CPP-031 SLICE 2 ADDENDUM: ModeFBWB. Upstream (Plane-4.7.0, read
// directly): ArduPlane/mode_fbwb.cpp (17 lines, full) + its ModeFBWB
// class decl (mode.h); ArduPlane/navigation.cpp's update_fbwb_speed_
// height() (~line 402) AND calc_airspeed_errors()'s FBWB/CRUISE airspeed-
// target branch (~line 160-190, previously out of scope - slice 1 only
// ported that function's speed-scaler line); ArduPlane/altitude.cpp's
// set_target_altitude_current()/change_target_altitude()/
// relative_target_altitude_cm()/check_fbwb_altitude(); ArduPlane/
// Attitude.cpp's calc_throttle()/calc_nav_pitch(); ArduPlane/Plane.cpp's
// update_alt() (~line 620-680) and tecs_hgt_afe() (~line 822).
//
// SURPRISING UPSTREAM FINDING #1 - update_pitch_throttle()/update_50hz()
// do NOT live inside update_fbwb_speed_height(): they're called from a
// completely separate function, Plane::update_alt() (Plane.cpp), gated on
// `should_run_tecs = control_mode->does_auto_throttle()` (true for FBWB,
// false for MANUAL/FBWA) - update_fbwb_speed_height() itself only adjusts
// target_altitude.amsl_cm from the elevator stick and then calls
// calc_throttle()/calc_nav_pitch(), which merely READ BACK whatever
// update_alt() most recently computed. This port has no independently-
// rated scheduled-task table (mode.hpp's own "SHAPE CHOICE" banner note -
// "a single fixed sequence suffices for two modes") and Mode::
// does_auto_throttle() was deliberately not ported (mode.hpp's banner:
// mode-IDENTIFICATION machinery, out of scope). Rather than fabricate a
// does_auto_throttle()/mode-identification mechanism just to reproduce an
// artificial task-boundary, this port folds update_alt()'s
// update_50hz()+update_pitch_throttle() calls DIRECTLY into
// update_fbwb_speed_height() below, called once per tick exclusively from
// ModeFBWB::update() (mode.hpp) - which correctly reproduces upstream's
// real does_auto_throttle() GATE (only ModeFBWB ever calls this function,
// exactly matching "true for FBWB, false for MANUAL/FBWA") without
// needing the boolean flag itself. Consequently mode.hpp's tick() is NOT
// modified for this slice - see mode.hpp's own note.
//
// ALTITUDE REFERENCE FRAME - A JUDGMENT CALL: upstream juggles THREE
// altitude frames (AMSL via target_altitude.amsl_cm, home-relative via
// relative_target_altitude_cm()'s `- home.alt`, and terrain-relative,
// excluded - no terrain subsystem). This port has no GPS/baro/home
// concept at all (see below), so there is no meaningful distinction
// between "AMSL" and "home-relative" here - they collapse into ONE frame:
// altitude relative to the vehicle's fixed start point (matching
// SimPlane's own `position` convention - NED, position={0,0,0} at
// construction - see ap-sim/sim_plane.hpp). Concretely: `home.alt` is
// DEFINITIONALLY 0 in this frame, so relative_target_altitude_cm() below
// is a pure passthrough of target_altitude_cm, and check_fbwb_altitude()'s
// `home.alt + min_alt_cm` becomes just `min_alt_cm`.
//
// CURRENT ALTITUDE INPUT vs. TARGET ALTITUDE STATE - the two concepts the
// ticket asked to keep distinct:
//   - StabilizeInputs::current_altitude_m (NEW) - what Tecs needs to know
//     about where the vehicle ACTUALLY is right now, an explicit per-tick
//     caller-supplied INPUT (upstream: _ahrs.get_relative_position_D_home()
//     for Tecs::update_pitch_throttle()'s unconditional height_ read, AND
//     - see below - a stand-in for AP::baro().get_altitude() too). A real
//     caller derives this from whatever altitude source exists (SimPlane's
//     `-position.z` in this slice's own closed-loop test).
//   - Plane::target_altitude_cm (NEW) - upstream's target_altitude.amsl_cm,
//     STATE this class owns and the elevator stick adjusts over time via
//     change_target_altitude()/set_target_altitude_current() below. Only
//     ever set FROM current_altitude_m (on lock-in) or nudged by a
//     climb-rate integration - never itself a live sensor reading.
//
// NO GPS/BARO - REUSING THE BARO-FALLBACK PATH ON PURPOSE: Tecs::
// update_50hz() has two branches - velocity_ned_valid=true (a real
// GPS/EKF velocity reading feeds climb_rate_ directly) or =false (a
// second-order complementary filter derives climb_rate_ from
// baro_altitude_m + accel_ef_z instead). This port has neither a GPS/EKF
// velocity estimate NOR a barometer, so build_tecs_inputs() below takes
// velocity_ned_valid=false and feeds `current_altitude_m` into
// baro_altitude_m too - i.e. current_altitude_m substitutes for BOTH
// upstream sensor reads at once. This was a deliberate choice over
// inventing a THIRD explicit input (e.g. a "climb_rate_ms"): the
// baro-fallback branch is already fully-ported, already-tested Tecs code
// (a real code path every barometer-only, no-GPS ArduPlane vehicle
// actually takes), so reusing it needs no new machinery and stays
// faithful to a real upstream configuration rather than inventing a
// velocity-source configuration this port doesn't have. accel_ef_z
// (upstream: _ahrs.get_accel_ef().z) and accel_body_x (upstream:
// AP::ins().get_accel().x, derived here via dcm_matrix.transposed() *
// ahrs.accel_ef) both read AhrsDcm::accel_ef, which defaults to the zero
// vector until a caller wires a real accelerometer in - the EXACT same
// "inert until wired" precedent slice 1's own banner already established
// for StabilizeInputs::accel_y (see AhrsDcm's own file banner). Not a
// stub: this is upstream's own real, already-implemented fallback
// behavior, just fed a currently-zero (but real, settable) accel input.
//
// FBWB AIRSPEED TARGET - SURPRISING UPSTREAM FINDING #2: a natural guess
// (the ticket's own included) is "a fixed cruise speed, since FBWB has no
// airspeed stick of its own." Reading calc_airspeed_errors() in full
// shows this is WRONG for an unconfigured vehicle: with FLIGHT_OPTIONS at
// its real default (0 - neither CRUISE_TRIM_AIRSPEED nor
// CRUISE_TRIM_THROTTLE set), upstream's actual FBWB/CRUISE branch maps
// the THROTTLE STICK position linearly onto [airspeed_min, airspeed_max]
// (`target_airspeed_cm = (airspeed_max-airspeed_min)*get_throttle_input()
// + airspeed_min*100`) - i.e. in default FBWB, the throttle stick
// commands airspeed (fed to TECS as the speed target), NOT direct engine
// power; TECS's own throttle law then decides actual throttle output to
// hold that speed while also holding the elevator-commanded altitude.
// Ported faithfully below (get_throttle_input(false), matching upstream's
// no-arg default). The CRUISE_TRIM_AIRSPEED/CRUISE_TRIM_THROTTLE branches
// are excluded - no FlightOptions bitmask subsystem in this port, and
// both default off, matching an unconfigured vehicle exactly (same
// exclusion precedent as stick_mixing_enabled()'s fence_stickmixing()
// note above).
//
// SURPRISING UPSTREAM FINDING #3 - Tecs::set_throttle_min() MUST BE
// CALLED EVERY TICK, NOT ONCE: discovered empirically during this slice's
// own closed-loop verification (see below) - without it, the vehicle
// exhibited real reverse-thrust braking oscillation instead of a clean
// climb. Tecs::thrminf_ext_ defaults to -1.0 (full reverse) and DECAYS
// BACK toward -1.0 every single call to update_throttle_limits() (private,
// called from within update_pitch_throttle()) unless re-asserted - see
// tecs.hpp's own set_throttle_min() doc comment, "applicable for one
// control cycle only." Upstream's real per-loop caller is Plane::
// apply_throttle_limits() (servos.cpp), called from set_servos() every
// loop regardless of mode - a materially bigger function than this
// slice's scope (ICEngine/battery-watt-limiter/takeoff/quadplane branches,
// none of which exist in this port). Rather than port all of that just
// for one line, update_fbwb_speed_height() below calls the minimal
// equivalent every tick: `if (!have_reverse_thrust()) tecs.set_throttle_min(0.0f);`
// - keeping Tecs's floor consistent with THIS vehicle's own
// aparm.throttle_min (default 0, no reverse thrust) instead of silently
// leaving it at Tecs's own reverse-thrust-permissive default. This is a
// genuine, in-scope necessity (without it, aparm.throttle_min is
// configured but never actually enforced), not upstream-patching.
//
// EXCLUDED (documented, not silently dropped):
//   - Terrain following (AP_TERRAIN_AVAILABLE) throughout - no terrain
//     subsystem. set_target_altitude_current()'s terrain_alt_cm branch,
//     relative_target_altitude_cm()'s terrain lookahead/rangefinder
//     branch, and check_fbwb_altitude()'s terrain branch are all dropped.
//   - Rangefinder correction (relative_target_altitude_cm()) - no
//     rangefinder subsystem.
//   - Mission altitude offset (relative_target_altitude_cm()'s
//     mission_alt_offset()) - no mission/AUTO subsystem; always 0 outside
//     AUTO anyway, so dropping it changes nothing even for upstream.
//   - Fence min/max altitude (check_fbwb_altitude()'s AP_FENCE_ENABLED
//     block) - no fence subsystem.
//   - Soaring controller hooks (mode_fbwb.cpp's _enter() HAL_SOARING_
//     ENABLED block, and update_fbwb_speed_height()'s HAL_SOARING_ENABLED
//     target-altitude override) - no soaring subsystem.
//   - RTL climb-min target-altitude boost (Plane.cpp's `control_mode ==
//     &mode_rtl` branch inside update_alt()) - no RTL mode in this slice.
//   - ModeFBWB::_enter() itself - this slice has no mode-switching
//     machinery yet (same exclusion mode.hpp's banner already documents
//     for Mode::enter()/exit()). _enter()'s real body is just
//     `plane.set_target_altitude_current()` - a caller constructing a
//     ModeFBWB for this slice's tests/use MUST call
//     plane.set_target_altitude_current(current_altitude_cm) ONCE,
//     EXPLICITLY, before the first tick() - see ModeFBWB's own class
//     banner (mode.hpp) - not silently skipping real initialization.
//
// 100ms RATE LIMIT - REPRODUCED FAITHFULLY, PER THE TICKET'S OWN
// INSTRUCTION: update_fbwb_speed_height()'s elevator-to-climb-rate-to-
// target-altitude integration only runs when `now_us -
// fbwb_last_elev_check_us >= 100000` (matches upstream's real
// `target_altitude.last_elev_check_us` gate, including the dt clamp to
// [0.1, 0.15] seconds) - this needs a real, independently-incrementing
// MICROSECOND clock distinct from StabilizeInputs::now_ms (matches
// ap-tecs's own TecsInputs::now_us/now_ms "two independent clocks, don't
// derive one from the other" precedent) - hence StabilizeInputs::now_us
// (NEW) below. check_fbwb_altitude()/calc_throttle()/calc_nav_pitch(),
// by contrast, are NOT rate-limited - they run every call, exactly
// matching upstream's own code shape (only the target-altitude
// integration sits inside the `if` block).
//
// =====================================================================
// CPP-031 SLICE 3 ADDENDUM: closing a real gap - tick() (mode.hpp) called
// ahrs.update(gyro_sample) and NOTHING else on the AHRS, meaning this
// vehicle's attitude estimate was PURE gyro integration with NO drift
// correction at all, despite CPP-028 slices 2/3 having fully ported
// AhrsDcm::drift_correction_yaw()/drift_correction_accel() and unit-tested
// them in isolation. The gap was that nothing in this port could ever call
// them with real data - there was no GPS module. CPP-033 built that module
// (ap-gps/gps.hpp); this slice wires it in.
//
// NEW Plane MEMBER: `gps::Gps gps` - see its own file banner (ap-gps) for
// what it reproduces from AP_GPS_SITL. Declared after `ahrs` (its natural
// place among owned subsystems); no declaration-order constraint applies
// to it (unlike aparm/the four controllers - see the DECLARATION-ORDER
// CONSTRAINT note above) since nothing in Plane's constructor reads it.
//
// StabilizeInputs GAINS FOUR NEW FIELDS, all defaulted so slice 1/2's
// existing MANUAL/FBWA/FBWB tests keep compiling AND PASSING unchanged
// (verified - see vehicle_test.cpp's own new closed-loop test comment for
// why a default-valued StabilizeInputs produces byte-for-byte the same
// tick() behavior as before this slice for any caller that doesn't
// populate them):
//   - true_velocity_ned (Vector3f, upstream: AP::sitl()->state.speedN/E/D,
//     the same true velocity ap-gps's own file banner already documents as
//     its `update()` parameter) - fed straight through to
//     `gps.update(in.true_velocity_ned, in.now_ms)` every tick. Default
//     zero vector: with true_velocity_ned always (0,0,0), Gps::update()
//     still fires every 200ms (the rate limit doesn't care about the
//     velocity value) but ground_speed_ms stays 0 forever, which sits
//     below kGpsSpeedMinMs (3.0f) - drift_correction_yaw()'s GPS-course
//     branch requires `ground_speed_ms >= kGpsSpeedMinMs` to ever produce
//     a nonzero yaw correction, so a caller leaving this at its default
//     gets a GPS that "has a fix" but never actually corrects anything,
//     exactly matching a stationary vehicle's real behavior (no ground
//     velocity yet, so no valid GPS heading to fuse) rather than an
//     invented no-op.
//   - accel_sample (ahrs::AccelSample, upstream: get_delta_velocity()/
//     _ins.get_accel() - the same struct AhrsDcm::accumulate_accel()
//     already takes, see ahrs_dcm.hpp's own SLICE 3 section) - fed to
//     `ahrs.accumulate_accel(in.accel_sample, in.dt)` every tick, matching
//     upstream's own dual-rate structure (the fast, every-tick half of
//     drift_correction() runs regardless of GPS timing). Default
//     AccelSample{} (delta_velocity_dt=0.0f, accel=zero): accumulate_accel()
//     treats delta_velocity_dt<=0 as "no valid sample this tick" (same
//     convention GyroSample's own dangle_dt already established) and skips
//     the ra_sum_ integration entirely; accel_ef is still written every
//     call (`dcm_matrix * sample.accel`), but with sample.accel at its
//     default zero this evaluates to the zero vector regardless of
//     dcm_matrix's value - IDENTICAL to accel_ef's pre-this-slice value
//     (it was never written by anything before CPP-028 slice 3 landed, and
//     slice 3's own callers that don't call accumulate_accel() leave it at
//     its own zero default - see ahrs_dcm.hpp's "accel_ef IS NOW COMPUTED"
//     note) - so a caller leaving this field at its default sees no change
//     to accel_ef, and therefore none to yaw_gain() or build_tecs_inputs()'s
//     accel_ef_z/accel_body_x reads either.
//   - wind_estimate (Vector3f, upstream: `_wind` - no wind-estimation
//     subsystem in this port, same "no wind estimation subsystem" precedent
//     ahrs_dcm.hpp's own file banner already established for
//     drift_correction_accel()'s wind_estimate parameter) - default zero
//     vector, fed to drift_correction_accel() directly and, via
//     `.xy().length()`, to drift_correction_yaw()'s wind_speed_ms
//     parameter - the SAME single-source-of-truth derivation ahrs_dcm.hpp's
//     banner documents upstream's own use_compass()/drift_correction() both
//     performing from the one `_wind` member.
//   - gps_use_enabled (bool, default true, upstream: AHRS_GPS_USE's real
//     GSCALAR default, `AP_AHRS::GPSUse::Enable` - verified directly
//     against AP_AHRS.cpp's own AP_GROUPINFO table, not assumed) - fed to
//     both drift_correction_yaw()/drift_correction_accel()'s
//     gps_use_enabled parameter and, transitively, have_gps(). Defaulting
//     to true (matching upstream's own real default) rather than false is
//     safe for slice 1/2's existing tests specifically BECAUSE
//     true_velocity_ned defaults to zero (see above) - have_gps() being
//     true doesn't by itself produce any correction without real velocity
//     data, so this is the faithful default, not a hidden behavior change.
//
// `armed` (drift_correction_yaw/accel's own parameter, upstream:
// hal.util->get_soft_armed()) is NOT a new field - StabilizeInputs::
// armed_and_safety_off already exists (slice 1) and is reused directly,
// matching the SAME reuse precedent calc_speed_scaler() already
// established for that exact field - no separate arming subsystem to model
// a finer distinction with.
//
// fly_forward()/accel_healthy()/ins_healthy() ADDED AS Plane METHODS, NOT
// StabilizeInputs FIELDS - matching fly_inverted()'s own precedent
// immediately below (a hardcoded, documented, always-the-same-answer
// method rather than a field nobody varies):
//   - fly_forward() (upstream: AP_AHRS::get_fly_forward(), set every loop
//     by Plane::update_fly_forward() (Plane.cpp), read directly): read in
//     full - with no quadplane (HAL_QUADPLANE_ENABLED), no idle_mode
//     (balloon-lift ballast release, no such subsystem), and no LAND
//     flight-stage (no landing subsystem/flight_stage machinery in this
//     port), update_fly_forward()'s real body falls straight through every
//     conditional to its final, unconditional `ahrs.set_fly_forward(true);`
//     - traced, not assumed. Always true for this port's scope.
//   - accel_healthy()/ins_healthy() (upstream: _ins.get_accel_health(i)/
//     _ins.healthy() - no INS health-monitoring subsystem in this port,
//     same "no subsystem, so this reads the nominal/unconfigured value"
//     precedent as ground_mode/reversed_throttle above) - both always true:
//     this port's IMU input IS the real gyro/accel data a caller already
//     supplies every tick (GyroSample/AccelSample), so "assume healthy" is
//     the honest behavior of a vehicle with no separate health-checking
//     machinery, not an invented shortcut. A future slice adding real
//     health monitoring can override these.
//
// CompassSample: mode.hpp's tick() constructs one with healthy=false EVERY
// TICK, unconditionally - no compass hardware in this port yet. This is
// documented there (not here) as a REAL, CURRENT LIMITATION, not a
// permanent design choice: drift_correction_yaw()'s use_compass() returns
// false immediately whenever compass.healthy is false (its very first
// check), which sends every call down the GPS-course branch instead
// (`else if (fly_forward && have_gps(...))`) - traced this path directly
// in ahrs_dcm.hpp rather than assuming it, confirming a compass-less
// vehicle genuinely CAN get real yaw correction from GPS course alone once
// moving fast enough (>= kGpsSpeedMinMs, 3 m/s).
//
// CALL-ORDER NOTE - a genuine, minor, and unavoidable consequence of this
// port's own class boundaries, not a bug: upstream's real
// AP_AHRS_DCM::update() calls matrix_update() -> normalize() ->
// drift_correction(delta_t) -> check_matrix() -> to_euler() as ONE atomic
// sequence, so drift_correction()'s newly-computed omega_p_/omega_yaw_p_
// feed into check_matrix()/to_euler() THE SAME TICK they're computed. This
// port's AhrsDcm::update() (CPP-028 slice 1) already bundles matrix_update/
// normalize/check_matrix/to_euler into one method with NO seam for a caller
// to insert drift correction in the middle - that seam was never built,
// and un-building it now would mean touching AhrsDcm's own already-tested,
// already-merged update() from a different ticket's slice, which this
// slice deliberately avoids. So tick() (mode.hpp) calls
// `ahrs.update(gyro_sample)` (using the PREVIOUS tick's omega_p_/
// omega_yaw_p_/omega_i_), THEN gps.update()+accumulate_accel()+
// drift_correction_yaw()/drift_correction_accel() (computing THIS tick's
// omega_p_/omega_yaw_p_/omega_i_ for the NEXT tick's matrix_update()) -a
// one-tick lag versus upstream's atomic ordering. Since these correction
// terms are deliberately slow, low-bandwidth trim signals (P-gains around
// 0.2, integrator time constants of seconds), a single 20ms tick's lag is
// immaterial to their effect and does not change any of this slice's own
// convergence tests' outcomes.

// =====================================================================
// CPP-031 SLICE 4 ADDENDUM: ModeCRUISE (mode.hpp, same module - see its own
// class banner for the mode-level design). Upstream (Plane-4.7.0, read
// directly): ArduPlane/mode_cruise.cpp (in full) + its ModeCruise class
// decl (mode.h); ArduPlane/Attitude.cpp's calc_nav_roll() (~line 652,
// trivial); ArduPlane/mode.cpp's Mode::navigate() default and Plane.cpp's
// scheduler_tasks[]/navigation.cpp's Plane::navigate() (the real caller -
// see mode.hpp's own "CPP-031 SLICE 4 ADDENDUM" note for the full tick()-
// ordering investigation); AP_L1_Control (ap-nav, already fully ported and
// unit-tested by CPP-017, unused until now); Location (ap-common, CPP-011,
// likewise unused until now); ArduPlane/defines.h's GPS_GND_CRS_MIN_SPD.
//
// CURRENT_LOC - THE DESIGN QUESTION THIS SLICE HAD TO ANSWER: this port has
// no GPS-derived position estimate anywhere (ap-gps/gps.hpp's own file
// banner: GpsSample deliberately carries NO lat/lon/position field at all -
// nothing needed one before this slice). L1Control needs a real Location
// for its current position, and Location's own arithmetic (get_distance_NE,
// get_bearing_to, offset/offset_bearing) is all flat-tangent-plane, working
// in NORTH/EAST METERS relative to SOME fixed reference point - it never
// actually needs that reference to be a real GPS lat/lon coordinate, only a
// fixed, consistent one. Following Location::offset()'s own doc comment
// (accurate to ~1mm at 100m) and this port's now-repeated "explicit input,
// no invented sensor" precedent (current_altitude_m/true_velocity_ned,
// CPP-031 slices 2/3):
//   - StabilizeInputs::position_ned (NEW, below) - the vehicle's true NED
//     position (north/east meters) relative to ITS OWN fixed start point -
//     matches SimPlane's own `position` field exactly (ap-sim/sim_plane.hpp:
//     NED, {0,0,0} at construction). A real caller derives this from
//     whatever position source exists (SimPlane's own position in this
//     slice's closed-loop test) - same treatment current_altitude_m already
//     received from `-position.z`.
//   - Plane::current_loc (NEW) - a Location computed FRESH EVERY TICK
//     (update_current_loc(), called from tick() below) as `Location{} `
//     (lat=lng=alt=0, i.e. a fixed, arbitrary reference point - "0N,0E,
//     0msl", never read as a real geodetic coordinate anywhere in this
//     port) THEN `.offset(position_ned.x, position_ned.y)`. NO separate
//     `home_loc` member was added to hold that reference: since it never
//     changes and is never touched by anything other than this one
//     function, a fresh default-constructed Location() each call is
//     exactly as correct and one field simpler than storing a
//     never-mutated copy of the same constant - documented here as the
//     judgment call the ticket asked for, not a memory-churn oversight
//     (Location has no dynamic allocation, ADR-0012, this is a handful of
//     int32_t writes).
//   - current_loc.alt is deliberately left at 0 (Location::offset() only
//     ever touches lat/lng) - EVERY consumer this slice has (L1Control's
//     get_distance_NE/get_bearing_to/get_distance, all called on
//     current_loc/prev_WP_loc/next_WP_loc below) is a purely horizontal
//     calculation that never reads Location::alt. Setting it from
//     current_altitude_m would be easy but purposeless - see this port's
//     "no invented values nothing reads" stance (matches
//     smoothed_airspeed's own "nothing in this slice's scope writes it"
//     note, just inverted: here nothing in this slice's scope READS the
//     value, so nothing writes it either).
//   - Since this reference point is FIXED and consistent from tick to tick
//     (never re-anchored), Location's flat-earth approximation error never
//     accumulates or resets - it only depends on distance from the
//     reference, exactly matching offset()'s own "~1mm at 100m" real
//     accuracy characteristic for the sub-few-km distances this slice's
//     CRUISE (1km look-ahead) and its tests ever reach.
//
// NAV_CONTROLLER (L1Control): a `nav::L1Control nav_controller` member,
// constructed with `nav::L1Control::Gains{}` - CHECKED, NOT RE-DERIVED:
// l1_control.hpp's own Gains struct already carries upstream's real
// NAVL1_PERIOD/NAVL1_DAMPING/NAVL1_XTRACK_I/NAVL1_LIM_BANK defaults
// (25.0f/0.75f/0.02f/0.0f - see its own inline citations, ported by
// CPP-017), so `Gains{}` IS the correct default-tuned controller, nothing
// to re-derive here. No declaration-order constraint applies (unlike
// aparm/the four controllers - see the pre-existing "DECLARATION-ORDER
// CONSTRAINT" note above): L1Control's constructor reads only the Gains
// struct passed to it, not any other Plane member.
//
// calc_nav_roll() TAKES StabilizeInputs, UNLIKE UPSTREAM'S ZERO-ARG
// VERSION - a necessary, minor signature divergence, not a behavior
// change: upstream's `nav_controller->nav_roll_cd()` reaches into a stored
// `AP_AHRS&` internally; this port's `L1Control::nav_roll_cd(const
// L1Inputs&)` needs an explicit L1Inputs (ADR-0012, same as every other
// AP_AHRS-reaching call in this port) - see build_l1_inputs() below, which
// calc_nav_roll() calls to build it from the SAME per-tick inputs every
// other stabilize_*()/build_tecs_inputs() function already takes.
//
// build_l1_inputs() - factored out (same "one caller-visible snapshot per
// tick" precedent as build_tecs_inputs() above) because BOTH calc_nav_roll()
// (called from ModeCRUISE::update(), for nav_roll_cd()) and ModeCRUISE::
// navigate() (for update_waypoint()) need an L1Inputs built from the exact
// same AHRS/GPS/current_loc snapshot - upstream achieves this implicitly by
// both reaching into the same live _ahrs/_gps singletons; this port makes
// the snapshot explicit and shared instead of duplicating field-by-field
// construction twice.
//   - location_valid = true UNCONDITIONALLY - upstream's `_ahrs.
//     get_location()` can fail (no absolute-position estimate available);
//     this port's current_loc is ALWAYS a real, freshly-computed value from
//     a caller-supplied position_ned (see above), the same "trust the
//     caller's explicit input, no separate validity flag" treatment
//     current_altitude_m/true_velocity_ned already received - there is no
//     failure mode to model without inventing one, so this is the honest
//     answer for an input that is always populated when supplied, exactly
//     like every other explicit-input field in this struct.
//   - groundspeed_vector = gps.sample().velocity_ned.xy() - upstream's
//     `_ahrs.groundspeed_vector()` is itself EKF/GPS-derived; reusing this
//     port's own Gps module (already wired into AhrsDcm's drift correction,
//     CPP-031 slice 3) for the SAME purpose here is the direct, faithful
//     equivalent, not an approximation - both ultimately trace to the same
//     ground-truth NED velocity a caller supplies via StabilizeInputs::
//     true_velocity_ned.
//   - yaw_sensor_cd = rad_to_cd(ahrs.yaw), unwrapped (may be outside
//     [-18000,18000]) - safe because L1Control's only two internal readers
//     (prevent_indecision(), get_yaw_sensor()) both immediately wrap it via
//     wrap_180_cd()/wrap_180_cd(18000 + ...) before using it (l1_control.hpp)
//     - matches roll_sensor_cd()/pitch_sensor_cd()'s own existing "convert
//     on demand, no caching" precedent immediately above.
//   - target_airspeed = 0.0f - upstream's `_tecs->get_target_airspeed()` is
//     read ONLY by L1Control::loiter_radius() (l1_control.hpp) - NOT by
//     update_waypoint(), the only L1Control entry point this slice's
//     ModeCRUISE ever calls (no loiter in this slice's scope). A real,
//     traced-not-assumed dead value for this slice, not a shortcut.
//
// PREV_WP_LOC/NEXT_WP_LOC ADDED TO Plane, NOT ModeCRUISE - matches upstream
// exactly (both are real Plane.h members there) - see mode.hpp's
// ModeCRUISE class banner "STATE OWNERSHIP" note for why this placement
// matters for a future AUTO mode.
//
// EXCLUDED (documented, not silently dropped) - see mode.hpp's ModeCRUISE
// class banner for the mode-level exclusions (scripting, soaring,
// mission/AUTO coupling, rudder_only aliasing); the one Plane-level
// exclusion is:
//   - Plane::navigate()'s OWN housekeeping (navigation.cpp: check_home_alt_
//     change(), waypoint distance/bearing bookkeeping for GCS reporting/
//     mission logic, the `next_WP_loc.lat==0 && lng==0` / `!have_position`
//     early-return guards) - none of it is needed to reach `control_mode->
//     navigate()` faithfully for THIS slice's single mode (no home/mission/
//     GCS subsystem exists to have alt-changed or report distance-to-
//     waypoint to), and `have_position`/next_WP_loc-zero-check's only
//     purpose is skipping navigate() when there's nothing to navigate to -
//     moot here since mode.navigate(in) is called unconditionally and
//     ModeCRUISE::navigate() itself already only acts once locked_heading_
//     is true.

// =====================================================================
// CPP-031 SLICE 5 ADDENDUM: ModeAUTO (mode.hpp, same module - see its own
// class banner for the mode-level design). Upstream (Plane-4.7.0, read
// directly, in full where the ticket asked): ArduPlane/mode_auto.cpp (202
// lines, full); ArduPlane/commands.cpp's Plane::set_next_WP (~line 10,
// full); ArduPlane/commands_logic.cpp's Plane::do_nav_wp (~line 400,
// trivial) and Plane::verify_nav_wp (~line 634, full); ArduPlane/
// navigation.cpp's Plane::setup_turn_angle (~line 457) and
// Plane::get_wp_radius (commands_logic.cpp ~line 1196); AP_Mission's
// AP_Mission::get_next_ground_course_cd (AP_Mission.cpp ~line 574, needed
// to trace setup_turn_angle's real look-ahead source).
//
// SCOPE - HONEST AND DELIBERATELY SMALLER THAN AP_Mission, NOT A STUB: a
// real, in-memory, ordered, fixed-size list of WAYPOINT-ONLY mission items
// flown sequentially via the L1Control/TECS machinery CRUISE/FBWB already
// wired in - "fly this list of waypoints in order" is a genuine, tested,
// closed-loop capability (see vehicle_test.cpp's own multi-waypoint
// integration test), just not upstream's general-purpose mission-command
// interpreter. See MissionItem/Mission below and ModeAUTO's own class
// banner (mode.hpp) for exactly where the scope line falls and why.
//
// MissionItem / Mission / kMaxMissionItems - THIS PORT'S OWN, SMALLER
// EQUIVALENT of AP_Mission::Mission_Command / AP_Mission (~4200 lines
// upstream, supporting dozens of MAVLink command types, jump, do-commands,
// splines, and a binary EEPROM storage format for MAVLink mission upload/
// download - none of that exists here, see the exclusion list below).
// MissionItem carries exactly `Location loc` + `float acceptance_radius_m`
// (0 meaning "use the default turn_distance()-based radius" - matches
// upstream's own cmd_acceptance_distance==0 fallback in verify_nav_wp,
// just as a plain float instead of a packed LOWBYTE(p1) byte, which
// upstream itself caps at 255m - this port's field has no such packing
// constraint to reproduce). Mission is a fixed-size std::array<MissionItem,
// kMaxMissionItems> (ADR-0012: no dynamic allocation - same precedent as
// RcChannels'/SrvChannels' own fixed channel arrays) with a current-index
// counter, `load()` (TEST/PROGRAMMATIC-ONLY - no MAVLink mission upload/
// EEPROM storage format in this port, see exclusion list), `current()`/
// `peek_next()` accessors, and `advance()`. kMaxMissionItems (32) is THIS
// PORT'S OWN bound (a plain fixed-array size), NOT a smaller version of
// upstream's real limit (however many commands fit in EEPROM storage,
// hardware-dependent, hundreds to thousands) - a different real constraint
// this port doesn't have, not a shrunken copy of it.
//
// SET_NEXT_WP / DO_NAV_WP / VERIFY_NAV_WP / SETUP_TURN_ANGLE - ported
// faithfully for the in-scope subset (see mode.hpp's ModeAUTO class banner
// for do_nav_wp/verify_nav_wp's exact upstream-vs-port call-signature
// difference - both now read `mission` directly rather than taking an
// AP_Mission::Mission_Command parameter, since this port's Plane owns the
// whole mission already). The real crosstrack-state machine
// (next_wp_crosstrack/crosstrack, upstream: auto_state's own two bools),
// the "already past the new waypoint" catch-up
// (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc) ->
// prev_WP_loc = current_loc), and setup_turn_angle()'s real look-ahead are
// ALL reproduced exactly, not simplified - see the two new Plane fields
// below (next_wp_crosstrack/crosstrack default to false, matching
// upstream's own auto_state being a zero-initialized global at boot,
// which is why a mission's FIRST leg never crosstracks - traced, not
// assumed).
//
// SURPRISING UPSTREAM FINDING - setup_turn_angle()'s real "no next leg"
// fallback is 90.0f DEGREES, NOT 0: AP_Mission::get_next_ground_course_cd
// returns its `default_angle` parameter (-1) when there's no next nav
// command, and Plane::setup_turn_angle() checks for exactly that sentinel
// and sets `auto_state.next_turn_angle = 90.0f` explicitly (also matching
// AutoState's own in-class initializer, `float next_turn_angle {90};`) -
// verified directly against both sites, not assumed from the field's name.
// next_turn_angle defaults to 90.0f below for the same reason.
//
// ALTITUDE SLOPE - DEFERRED, A NAMED SIMPLIFICATION: upstream's
// setup_alt_slope()/adjust_altitude_target() (altitude.cpp) linearly
// interpolate the TECS target altitude between prev_WP_loc's and
// next_WP_loc's altitudes AS THE VEHICLE FLIES THE LEG - a real,
// already-useful feature (Location::linearly_interpolate_alt already
// exists, CPP-011, unused until a future slice wires it in here). This
// slice uses a SIMPLER, HONEST substitute instead: set_next_WP() sets
// `target_altitude_cm` to the NEW waypoint's own altitude immediately,
// flat for the whole leg (no slope) - directly reusing the SAME
// target_altitude_cm/relative_target_altitude_cm() FBWB/CRUISE already
// established (see SLICE 2's own "CURRENT ALTITUDE INPUT vs TARGET
// ALTITUDE STATE" note), rather than inventing a second altitude-target
// mechanism. A vehicle in AUTO will therefore step its altitude target at
// each waypoint transition rather than ramping it - a real, documented
// behavior difference from upstream, not a silently dropped feature.
//
// TERRAIN-RELATIVE ALTITUDE - NOT MODELED: no terrain subsystem (same
// exclusion this port has documented everywhere else, e.g. Location's own
// file banner) - fix_terrain_WP() and change_alt_frame(ABSOLUTE) are both
// skipped in set_next_WP(); a MissionItem's Location is assumed supplied
// already in this port's one collapsed altitude frame (SLICE 2's own
// "ALTITUDE REFERENCE FRAME" note - home.alt is definitionally 0), exactly
// like next_WP_loc/prev_WP_loc/current_loc already are everywhere else in
// this vehicle.
//
// GET_WP_RADIUS() / AIRSPEED_CRUISE - two new FixedWingTunables fields:
// waypoint_radius (WP_RADIUS/g.waypoint_radius, upstream default
// WP_RADIUS_DEFAULT=90, config.h) backs get_wp_radius()'s real body
// (quadplane branch excluded - no quadplane in this port, so it collapses
// to a plain field read); airspeed_cruise (AIRSPEED_CRUISE/
// aparm.airspeed_cruise, upstream default 12 m/s, config.h) backs
// update_auto_speed_height()'s airspeed target - see that function's own
// comment for why AUTO needs a DIFFERENT airspeed-target source than
// FBWB/CRUISE's throttle-stick-driven one.
//
// UPDATE_AUTO_SPEED_HEIGHT() - A NEW FUNCTION, NOT A REUSE OF
// update_fbwb_speed_height() - discovered while tracing exactly where
// upstream drives TECS for AUTO. ModeAuto::update()'s real body is JUST
// `calc_nav_roll(); calc_nav_pitch(); calc_throttle();` (mode_auto.cpp) -
// none of those DRIVE Tecs, they only READ its last-computed demand. The
// real driver is Plane::update_alt() (Plane.cpp), a SEPARATE 10Hz
// SCHED_TASK gated on `control_mode->does_auto_throttle()` (true for
// AUTO too) - the SAME real function SLICE 2's own "SURPRISING UPSTREAM
// FINDING #1" already traced for FBWB, and which this port folded into
// update_fbwb_speed_height() there. Reusing update_fbwb_speed_height()
// for AUTO would be WRONG, not just imprecise: its elevator-stick target-
// altitude adjustment and throttle-stick airspeed target are both real
// FBWB/CRUISE-only behaviors upstream itself never runs for AUTO (see
// calc_airspeed_errors()'s own `control_mode == &mode_fbwb || control_mode
// == &mode_cruise` guard, navigation.cpp ~line 161). So this slice adds
// update_auto_speed_height() as AUTO's own minimal equivalent of
// update_alt()'s should_run_tecs branch, called once per tick exclusively
// from ModeAUTO::update() (mode.hpp) BEFORE the three ticket-specified
// calc_*() calls - mirroring update_fbwb_speed_height()'s own established
// shape (fold the real driving call into the one mode that needs it)
// rather than resurrecting the mode-identification machinery
// (does_auto_throttle()) this port deliberately left unported.
//
// MISSION COMPLETE / FINAL-WAYPOINT-HOLD - A NAMED, DELIBERATE DIVERGENCE
// FROM UPSTREAM: reaching the last waypoint upstream advances the mission
// index past the end and (for a real mission) typically transitions to
// RTL - no RTL mode exists in this port. This slice's Mission::advance()
// is simply a no-op (returns false) once at_last() is true, and
// ModeAUTO::navigate() (mode.hpp) does not call do_nav_wp()/set_next_WP()
// again when advance() returns false - next_WP_loc stays pinned at the
// final waypoint, and verify_nav_wp()'s own nav_controller.update_waypoint()
// call keeps re-running toward it every tick, so the vehicle holds course
// on the final leg indefinitely rather than stopping, looping the mission,
// or falling back to some other mode. This is real, chosen behavior
// (documented here, not silently different from "what real AUTO would do
// at mission end" - real AUTO enters RTL) - a reasonable stand-in until a
// future slice adds RTL.
//
// EXCLUDED (documented, not silently dropped) - see mode.hpp's ModeAUTO
// class banner for the mode-level exclusions (takeoff/land/scripting
// special-case update() branches); the Plane/mission-level exclusions are:
//   - Every non-NAV_WAYPOINT command type: TAKEOFF, LAND, LOITER_UNLIM/
//     TURNS/TIME/TO_ALT, RTL, jump, do-commands (including DO_CHANGE_SPEED
//     - see update_auto_speed_height()'s own comment), splines, VTOL -
//     none exist in MissionItem's vocabulary at all, not even as a
//     recognized-but-unimplemented enum value.
//   - MAVLink mission upload/download, EEPROM mission storage format -
//     Mission::load() is a test/programmatic-only entry point.
//   - AP_Mission::MISSION_RUNNING state-machine / GCS mission-state
//     reporting (mission.state() checks, ModeAuto::update()'s own
//     `plane.mission.state() != AP_Mission::MISSION_RUNNING` guard) - this
//     slice's Mission has no external state-reporting consumers yet; a
//     Mission with no items loaded is handled directly (current()/
//     peek_next() return nullptr, ModeAUTO::navigate() checks for that)
//     rather than via a ported state enum.
//   - verify_nav_wp's pass-by distance (cmd_passby/HIGHBYTE(p1)) and
//     waypoint_max_radius override (g.waypoint_max_radius) - see mode.hpp's
//     ModeAUTO class banner for the exact upstream lines skipped.
//   - Watchdog mission-resume (hal.util->was_watchdog_armed()) and
//     HAL_SOARING_ENABLED's init_cruising() (both in ModeAuto::_enter()) -
//     no watchdog-persistence or soaring subsystem in this port.

// =====================================================================
// CPP-031 SLICE 6 ADDENDUM: ModeRTL (mode.hpp, same module - see its own
// class banner for the mode-level design). Upstream (Plane-4.7.0, read
// directly, in full per the ticket): ArduPlane/mode_rtl.cpp (169 lines,
// full) - ModeRTL::_enter()/update()/navigate(); ArduPlane/commands_logic.
// cpp's Plane::do_RTL (~line 337, full) and Plane::calc_best_rally_or_
// home_location (immediately below it, full, including its #else branch);
// ArduPlane/altitude.cpp's Plane::get_RTL_altitude_cm (~line 99, trivial);
// ArduPlane/navigation.cpp's Plane::update_loiter/update_loiter_update_nav
// (~lines 321-395, both in full).
//
// THE FIRST MODE TO USE L1Control's LOITER SUPPORT AND THE FIRST TO NEED
// A PERSISTENT "HOME": CPP-017 ported update_loiter()/reached_loiter_
// target()/loiter_radius() into L1Control (l1_control.hpp) in an earlier
// slice, but nothing in this vehicle skeleton ever called them until now
// - CRUISE/AUTO only ever call update_waypoint(). Likewise, this vehicle
// has never had a "fixed origin the aircraft can navigate back to"
// concept before - current_loc/prev_WP_loc/next_WP_loc are all either
// recomputed fresh every tick (current_loc) or set transiently by
// whatever mode/mission leg is active (prev_WP_loc/next_WP_loc) - see
// SLICE 4's own "CURRENT_LOC" note. `home` (below) is the first Plane
// member that is meant to be set ONCE and never move again.
//
// `home` / `set_home()` - DESIGN: a plain Location field, default-
// constructed (zero - lat=lng=alt=0, the SAME fixed reference point every
// other Location in this vehicle is ultimately anchored to, per SLICE 4's
// own "CURRENT_LOC" note), settable ONLY via the explicit set_home()
// method below - matching this port's now-established "explicit setup,
// no automatic magic" pattern (ModeFBWB/ModeCRUISE/ModeAUTO's own enter()
// methods each requiring one specific setup call before the first tick).
// Upstream's real Plane::home is set automatically by a GPS-lock/arming
// home-setting subsystem this port doesn't have (no AP_AHRS::
// set_home()-equivalent, no "home is set" state machine) - so, exactly
// like ModeFBWB's set_target_altitude_current() requirement, A CALLER
// MUST CALL plane.set_home(...) AT LEAST ONCE before entering ModeRTL for
// the first time; there is no default that would be meaningful (a plane
// whose home was silently left at the shared origin Location() would
// often be correct by coincidence in this port's own tests - the origin
// IS the usual launch point - but that coincidence is not a substitute
// for a real caller making the choice explicitly, matching the ticket's
// own instruction).
//
// CURRENT_LOC.ALT - A REAL GAP THIS SLICE CLOSES, NOT JUST WORKS AROUND:
// SLICE 4's own "CURRENT_LOC" note left current_loc.alt PERMANENTLY at 0,
// reasoning that "every consumer this slice has... is a purely horizontal
// calculation that never reads Location::alt." Verified directly (not
// re-assumed) against location.hpp for THIS slice: get_distance()/
// get_distance_NE()/past_interval_finish_line()/line_path_proportion()
// are ALL purely horizontal (dlat/dlng only, confirmed by reading their
// bodies) - so that reasoning still holds for every PRE-EXISTING
// consumer. But get_RTL_altitude_cm() (below) and ModeRTL::update()'s
// RTL_CLIMB_MIN check (mode.hpp) are the FIRST real consumers of
// current_loc's VERTICAL component in this vehicle's history - reading
// current_loc.alt as upstream's own get_RTL_altitude_cm() literally does
// would silently always return 0 (a real "current altitude" value is the
// entire POINT of that branch - "maintain current altitude" RTL - not a
// harmless simplification), and the RTL_CLIMB_MIN check would NEVER see
// alt_threshold_reached become true, permanently wedging the climb-limit
// clamp on. Rather than inventing a workaround local to RTL (e.g. a new
// StabilizeInputs field duplicating information the vehicle already
// receives), update_current_loc() (below) is extended to ALSO populate
// current_loc.alt from the SAME position_ned already supplied every tick
// - position_ned.z is NED-down, so altitude above the vehicle's own fixed
// start point is -position_ned.z, exactly the relationship every closed-
// loop test already maintains between position_ned and StabilizeInputs::
// current_altitude_m (e.g. `in.position_ned = sim_plane.position; in.
// current_altitude_m = -sim_plane.position.z;`). This is a genuinely
// SAFE, NON-BREAKING extension of already-tested shared infrastructure:
// since every pre-existing consumer (traced above) never reads
// Location::alt at all, none of CRUISE's or AUTO's existing tests can
// observe this change - verified by running them unchanged (see
// vehicle_test.cpp's own confirmation), not merely argued.
//
// GET_RTL_ALTITUDE_CM() - kept at upstream's own real zero-arg signature
// (unlike calc_nav_roll()'s necessary StabilizeInputs-taking divergence,
// SLICE 4's own note) precisely BECAUSE current_loc.alt is now real data,
// not a dead field needing an explicit substitute parameter.
//
// DO_RTL() - EXCLUDED (per the ticket, and verified by reading the #else
// branch directly): calc_best_rally_or_home_location()'s HAL_RALLY_
// ENABLED branch - no rally-point subsystem in this port, and upstream's
// OWN #else branch (no rally points configured) is EXACTLY `Location{
// home.lat, home.lng, rtl_home_alt_amsl_cm, ABSOLUTE}` - i.e. rally
// points collapse to `home` directly even for a real, unmodified upstream
// build with HAL_RALLY_ENABLED off, not a port-specific approximation.
// fix_terrain_WP()/setup_terrain_target_alt() - no terrain subsystem
// (same exclusion throughout this port). set_target_altitude_location()/
// setup_alt_slope() - both replaced by the SAME flat `target_altitude_cm
// = next_WP_loc.alt` assignment set_next_WP() already established (SLICE
// 2's own "ALTITUDE SLOPE - DEFERRED" note) - there is no second altitude
// mechanism to invent here.
//
// SETUP_TURN_ANGLE() REUSE - VERIFIED, NOT ASSUMED, PER THE TICKET'S OWN
// INSTRUCTION: do_RTL() calls the EXISTING setup_turn_angle() (SLICE 5)
// unmodified. Traced through both real scenarios: (1) no mission loaded
// (this slice's own tests) - mission.peek_next() returns nullptr
// (Mission::peek_next(), plane.hpp above: `current_index_+1 < count_`,
// false when count_==0) - so next_turn_angle correctly falls to the real
// 90-degree "no next leg" fallback (SLICE 5's own "SURPRISING UPSTREAM
// FINDING"), exactly matching upstream's real single-leg RTL behavior.
// (2) a mission WAS loaded and mid-flight when RTL is entered - THIS
// PORT'S `mission` object is Plane-level state (SLICE 5's own "STATE
// OWNERSHIP" note) that persists across mode changes exactly like
// upstream's own AP_Mission does (a mission's current-command index is
// not reset just because the active flight mode changed) - so
// setup_turn_angle() would genuinely look ahead to the OLD mission's next
// waypoint, exactly reproducing upstream's real (if slightly surprising)
// behavior rather than a port-specific quirk. Not exercised by this
// slice's own tests (which never load a mission before testing RTL), but
// traced and documented rather than assumed away.
//
// UPDATE_LOITER()/UPDATE_LOITER_UPDATE_NAV() - the ONE real behavioral
// simplification in this slice, PER THE TICKET'S OWN INSTRUCTION:
// upstream's update_loiter_update_nav() gates its "navigate to the loiter
// point like a waypoint" branch on `(loiter.start_time_ms == 0 &&
// (control_mode == &mode_auto || control_mode == &mode_guided) &&
// auto_state.crosstrack && ...)`. ModeRTL is the ONLY caller of
// update_loiter() anywhere in this port's scope (no ModeGUIDED/ModeLOITER
// exist yet), so the multi-mode check collapses to JUST `crosstrack` -
// this port's own `crosstrack` field (SLICE 5), which do_RTL() always
// resets to false on entry (matching upstream's own auto_state.crosstrack
// meaning for a freshly-started leg) and update_loiter_update_nav() never
// itself sets true - so in practice, for THIS slice, the crosstrack-
// waypoint-nav branch never actually fires (crosstrack stays false for
// the entire time ModeRTL is active), and update_loiter() goes straight
// to nav_controller.update_loiter() every call - a real, honest
// consequence of RTL always flying a single, non-crosstracked leg to
// home, not a bug. UPDATE (CPP-031 SLICE 10): ModeLOITER is now a SECOND
// caller of this same shared function - see this file's own "CPP-031
// SLICE 10 ADDENDUM" ("UPDATE_LOITER_UPDATE_NAV()'S 'JUST crosstrack'
// SIMPLIFICATION GETS A SECOND CALLER" note) for why the collapse remains
// safe for it too, for a DIFFERENT structural reason than RTL's own
// (crosstrack-always-false) one - not a claim that RTL is still the only
// caller. The HAL_QUADPLANE_ENABLED branch (the "switching to
// QRTL" direct-waypoint-nav condition) is dropped entirely - no
// quadplane. update_loiter()'s own `auto_state.wp_proportion > 1`
// alternate "reached" condition is also dropped - wp_proportion is an
// AUTO-mission-only concept never ported to this Plane (SLICE 5's own
// Mission exclusion list has no such field), and RTL has no mission leg
// to compute a proportion along; loiter.start_time_ms's OWN "reached
// via reached_loiter_target()" latch (kept, see below) is the real
// condition this slice's one caller can ever satisfy anyway.
//
// LOITER STATE - THIS PORT'S OWN SMALLER EQUIVALENT of upstream's
// Plane::loiter (Plane.h), carrying ONLY the three fields update_loiter()/
// do_RTL()/ModeRTL::navigate() actually read or write in this slice's
// scope: direction, start_time_ms, radius. Upstream's remaining fields
// (old_target_bearing_cd/total_cd/sum_cd/reached_target_alt/unable_to_
// achieve_target_alt/start_lap_alt_cm/next_sum_lap_cd/time_max_ms) all
// belong to LOITER_TURNS/LOITER_TIME/LOITER_TO_ALT mission commands -
// none exist in MissionItem's vocabulary (SLICE 5's own exclusion list),
// so there is nothing in this port that could ever read or write them -
// a different real constraint than upstream's, not a shrunken copy of a
// struct this port doesn't fully use (same "own smaller equivalent"
// precedent as MissionItem/Mission themselves, SLICE 5's own note).
//
// RTL STATE - `rtl.done_climb` is ALREADY EXACTLY upstream's real struct
// (`struct { bool done_climb; } rtl;`, Plane.h) - no reduction needed,
// ported directly.
//
// NEW FixedWingTunables FIELDS - every default cited against Parameters.
// cpp/config.h directly, not invented: loiter_radius (WP_LOITER_RAD /
// aparm.loiter_radius, default LOITER_RADIUS_DEFAULT=60, config.h, m),
// rtl_altitude (RTL_ALTITUDE / g.RTL_altitude, default ALT_HOLD_HOME=100,
// config.h, m), rtl_radius (RTL_RADIUS / g.rtl_radius, default 0,
// Parameters.cpp, m), rtl_climb_min (RTL_CLIMB_MIN / g2.rtl_climb_min,
// default 0, Parameters.cpp, m). level_roll_limit_deg (LEVEL_ROLL_LIMIT,
// default 5) ALREADY EXISTED (SLICE 1) - reused directly by ModeRTL::
// update()'s climb-before-turn clamp (mode.hpp), no new field needed.
//
// EXCLUDED (documented, not silently dropped) - ModeRTL's own class
// banner (mode.hpp) covers the mode-level exclusions (every
// HAL_QUADPLANE_ENABLED branch in _enter()/navigate(), CLIMB_BEFORE_TURN's
// FlightOptions bitmask branch, the entire RTL_IMMEDIATE_DO_LAND_START/
// RTL_THEN_DO_LAND_START/DO_RETURN_PATH_START autoland-mission-jump
// machinery, GCS messaging); the Plane-level exclusions are:
//   - Rally points (calc_best_rally_or_home_location's HAL_RALLY_ENABLED
//     branch) - see "DO_RTL()" note above.
//   - Terrain-relative altitude (fix_terrain_WP, setup_terrain_target_alt)
//     - no terrain subsystem, same exclusion throughout this port.
//   - update_loiter_update_nav()'s HAL_QUADPLANE_ENABLED branch and
//     update_loiter()'s `auto_state.wp_proportion > 1` term - see
//     "UPDATE_LOITER()" note above.

// =====================================================================
// CPP-031 SLICE 7 ADDENDUM: real mode-switching (Plane::set_mode()) - see
// mode.hpp's own file banner for the tick()-signature/dispatch-through-
// control_mode side of this slice, and mode.hpp's ModeAUTO class banner for
// the mission-complete-to-RTL wiring this slice exists to enable. Upstream
// (Plane-4.7.0, read directly, in full per the ticket): ArduPlane/system.cpp's
// Plane::set_mode(Mode&, ModeReason) (~line 252-352); ArduPlane/mode.h's
// Mode::enter()/exit() (public, non-virtual, lines ~80-84) wrapping protected
// virtual Mode::_enter()/_exit() (lines ~186-189, default bodies `return
// true;`/`return;`); ArduPlane/mode.cpp's Mode::enter()/exit() real bodies
// (lines ~16-278) and Mode::reset_controllers() (~line 279); ArduPlane/
// mode_auto.cpp's ModeAuto::_enter()/_exit(); ArduPlane/commands_logic.cpp's
// Plane::exit_mission_callback() (~line 1047, full) - the REAL mission-
// complete-to-RTL trigger (see below).
//
// WHY MODE'S CLASS DECLARATIONS NOW LIVE HERE, NOT IN mode.hpp - A REAL C++
// COMPLETENESS CONSTRAINT, NOT A STYLE CHOICE: the ticket asks for `Plane`
// to OWN all six concrete Mode subclasses as direct, non-pointer, non-
// allocating members (ModeManual mode_manual{*this}; etc., ADR-0012 - no
// heap allocation). A class holding another class BY VALUE needs that other
// class to be a COMPLETE type (full data-member layout known) at the point
// of the enclosing class's own definition. Simultaneously, every non-trivial
// Mode/ModeXXX method body (update()/run()/navigate() overrides, reset_
// controllers(), output_pilot_throttle(), ModeAUTO::enter()/navigate(),
// ModeRTL::enter()/navigate(), ...) calls straight into `plane_.<something>`
// - which needs PLANE to be a complete type at the point those bodies are
// compiled (ordinary, non-template C++ member functions are checked against
// their enclosing translation unit in one pass; there is no template-style
// deferred instantiation to lean on here). Those two requirements point in
// opposite directions and cannot both be satisfied by simply reordering two
// self-contained headers - this is the exact same problem upstream itself
// solves by splitting mode.h (class declarations, ArduPlane's own headers
// never need Plane to be complete just to declare a method) from mode.cpp/
// mode_manual.cpp/mode_fbwa.cpp/etc. (method BODIES, compiled in Plane.h's
// own translation unit where Plane is already complete). This port has no
// .cpp files (a deliberate header-only convention throughout every prior
// slice) - so the SAME declaration/definition split is reproduced within
// this module's existing two-header convention instead of adding six new
// mode_*.cpp-equivalent files: Mode's class hierarchy DECLARATIONS (data
// members + method signatures - everything needed for the compiler to know
// each subclass's size/layout, i.e. "complete enough" for Plane to hold by
// value) now live HERE, just above `class Plane` (this file), forward-
// declaring `class Plane;` for the `Plane& plane_` reference member (a
// reference to an incomplete type is legal - only DEREFERENCING it requires
// completeness, and no Mode method body appears until after Plane is fully
// defined). Every method body that touches `plane_` is declared-only here
// and DEFINED out-of-line in mode.hpp, which now `#include`s ONLY this file
// (plane.hpp no longer includes mode.hpp at all - the cycle is fully
// broken, not merely hidden behind include guards) and is processed AFTER
// Plane is complete, exactly mirroring upstream's own Plane.h-is-already-
// complete-by-the-time-mode.cpp-compiles ordering. Method bodies that touch
// ONLY a mode's own private state (e.g. ModeCRUISE::get_target_heading_cd())
// or nothing at all (Mode's/every mode's default enter()/exit()/navigate())
// need no such split and stay inline in the class declarations below, same
// as before this slice.
//
// ENTER()/EXIT() BECOME REAL, PUBLIC, VIRTUAL METHODS - COLLAPSING
// UPSTREAM'S PUBLIC-WRAPPER/PROTECTED-VIRTUAL SPLIT INTENTIONALLY:
// upstream's Mode::enter() (public, non-virtual) does a large amount of
// bookkeeping (auto_state.inverted_flight/highest_airspeed/vtol_mode reset,
// steer_state/crash_state/guided_state reset, nav_scripting/camera/ADSB/
// systemid/quadplane/terrain/fence hooks, throttle_suppressed, mission-in-
// landing-sequence reset, `plane.prev_WP_loc = plane.current_loc`,
// `plane.loiter.start_time_ms = 0`, `plane.last_mode_change_ms = millis()`)
// BEFORE calling the protected virtual `_enter()` mode-specific hook, then
// more bookkeeping (steerController.reset_I(), control_failsafe(),
// fence.manual_recovery_start()) AFTER it succeeds - read in full
// (mode.cpp). EVERY SINGLE ONE of those bookkeeping fields/subsystems is
// already-documented out of this port's scope (no scripting/camera/ADSB/
// systemid/quadplane/terrain/fence/failsafe/steering/mission-landing-
// sequence subsystem exists - see this file's own banner's repeated
// exclusions, and mode.hpp's own file banner). The two pieces that ARE
// real, in-scope Plane state (`prev_WP_loc = current_loc`, `loiter.
// start_time_ms = 0`) are NOT reproduced as a generic wrapper here either -
// they are ALREADY each mode's own responsibility in this port (ModeAUTO::
// enter()/ModeRTL::enter() already set prev_WP_loc = current_loc
// themselves, matching upstream's own per-mode _enter() bodies doing the
// identical assignment redundantly; MANUAL/FBWA/FBWB/CRUISE never read
// prev_WP_loc/next_WP_loc at all, so a generic reset before their own
// (nonexistent) _enter() would be dead work; loiter.start_time_ms == 0 is
// already ModeRTL's own struct's default/reset state - do_RTL() being
// called fresh from ModeRTL::enter() never touches it, matching upstream's
// own loiter struct not being re-zeroed by do_RTL() either, only by the
// generic wrapper's "new mode means new loiter" comment, which the one
// mode that reads loiter (RTL) always constructs itself fresh here since
// this port has no persistent Plane-lifetime LoiterState reused across
// unrelated modes the way upstream's Plane::loiter is - a real, minor,
// deliberate simplification, not an oversight). So THIS port's Mode::
// enter()/exit() collapse straight to upstream's protected _enter()/_exit()
// virtual hooks, made public and virtual directly - `virtual bool enter()
// { return true; }` / `virtual void exit() {}` - matching MANUAL/FBWA's own
// real "nothing to set up" behavior as the honest default for every mode
// that doesn't override it (MANUAL, FBWA, FBWB, CRUISE all keep this
// default unchanged from before this slice - none of them gained an
// enter() override, since none of upstream's own real _enter() bodies for
// them do anything this port hasn't already excluded elsewhere or already
// requires a caller to do explicitly - see FBWB/CRUISE's own "NOT PORTED/
// CALLED AUTOMATICALLY" class-banner notes, UNCHANGED by this slice: a
// caller invoking set_mode(plane.mode_fbwb)/set_mode(plane.mode_cruise)
// must still call set_target_altitude_current() explicitly afterward,
// exactly like before this slice - wiring their own real _enter() bodies
// into an override is a natural, small future addition, out of scope here,
// which targets specifically the ModeAUTO-mission-complete-to-RTL gap).
// ModeAUTO::enter()/ModeRTL::enter() (already existed as non-virtual
// methods, CPP-031 slices 5/6) simply gain `override` and a `bool` return
// (`return true;` - neither has any real failure condition to port, see
// mode.hpp's own banner for why this is honest, not a shortcut).
// Mode::exit()'s real upstream body is `_exit(); if (control_mode !=
// &mode_autotune) autotune_restore();` - no AUTOTUNE mode in this port, so
// this collapses to just `_exit()`, i.e. exactly the protected virtual
// hook, i.e. exactly what this port's own public virtual exit() already
// is. The ONE upstream mode with a real (non-empty) _exit() body is
// ModeAuto's (mode_auto.cpp: stop a running mission and maybe restart a
// landing sequence) - traced directly, and it depends ENTIRELY on
// subsystems this port has ALREADY excluded (AP_Mission's own MISSION_
// RUNNING state machine - see plane.hpp's own "EXCLUDED" note on Mission -
// and a landing subsystem for restart_landing_sequence()) - so ModeAUTO
// relies on the base class's default no-op exit() too, a real, traced
// exclusion, not an oversight. No mode in this port's six therefore needs
// an exit() override.
//
// SET_MODE() - DROPS THE ModeReason PARAMETER ENTIRELY, NOT MERELY
// DEFAULTS IT: upstream's real set_mode(Mode&, ModeReason) uses `reason`
// for exactly four things, all already-excluded subsystems - AP_Notify
// happy/sad noise gating, logger.Write_Mode() (HAL_LOGGING_ENABLED),
// gcs().send_text()/send_message() (no GCS), and previous_mode/previous_
// mode_reason/control_mode_reason bookkeeping (no consumer anywhere in
// this port reads "why" a mode changed - see mode.hpp's own banner for
// mode-IDENTIFICATION machinery being consistently out of scope). Carrying
// the parameter through with nothing real to do with it would be dead
// plumbing, not fidelity - the ticket's own explicit instruction. The
// real, in-scope CORE LOGIC - already-in-this-mode no-op, tentative swap,
// enter()-may-fail rollback, exit() the old mode on success - is
// reproduced exactly, including the upstream ordering (swap BEFORE calling
// enter(), so a mode's own enter() body can safely assume `plane_.
// control_mode == this` if it ever needed to - none of this port's six do,
// but the ordering is upstream's real, deliberate choice, preserved
// faithfully). HAL_QUADPLANE_ENABLED's VTOL-availability check, AP_FENCE_
// ENABLED's fence-recovery mode-lock, and FLTMODE_GCSBLOCK's GCS-mode-
// change-blocking are all excluded outright (no such subsystems).
//
// CONTROL_MODE'S DEFAULT - upstream: `Mode *control_mode = &mode_
// initializing;` (Plane.h) - ModeInitializing is a real upstream mode (its
// own _enter() waits for the AHRS to settle before anything else can run)
// that this port has never ported and has no equivalent concept for (no
// AHRS-settling/pre-arm gate this port's tick() waits on - every mode has
// always been immediately runnable from tick 1, throughout every prior
// slice). Defaulting to `&mode_manual` instead - the one real mode that
// needs no setup at all (enter() always trivially succeeds, no state to
// initialize) - is the honest equivalent of "a freshly-constructed vehicle
// starts in its simplest, always-safe mode" rather than a null pointer
// (which tick() would then dereference on tick 1 with no caller action
// required) or a fabricated ModeInitializing this port has no use for.
//
// MISSION-COMPLETE-TO-RTL - THE REAL TRIGGER, TRACED, NOT ASSUMED (per the
// ticket's own instruction): grepped ArduPlane/*.cpp for `mode_rtl`/
// `MISSION_COMPLETE` directly rather than guessing. AP_Mission::
// advance_current_nav_cmd() returning false at the end of a mission calls
// AP_Mission::complete() (AP_Mission.cpp ~line 2039), which sets state to
// MISSION_COMPLETE and invokes a stored callback, `_mission_complete_fn()`
// - bound at construction (Plane.h ~line 683) to `Plane::
// exit_mission_callback()` (commands_logic.cpp ~line 1047, read in full):
// `if (control_mode == &mode_auto) { set_mode(mode_rtl, ModeReason::
// MISSION_END); gcs().send_text(...); }` - i.e. THE real mission-complete-
// to-RTL transition is a callback AP_Mission invokes on ITS OWN state
// machine reaching its end, not code living in mode_auto.cpp/ModeAuto
// itself at all (confirming the ticket's own prediction). This port's much
// smaller Mission class (CPP-031 slice 5) has no callback mechanism (no
// consumer needs one anywhere else) - so the equivalent transition is
// placed at the ONE place this port's own mission-complete condition is
// actually detected: ModeAUTO::navigate() (mode.hpp), in the branch where
// `plane_.mission.advance()` returns false (this slice's own direct
// equivalent of advance_current_nav_cmd() returning false) - calling
// `plane_.set_mode(plane_.mode_rtl)` there directly, reproducing exactly
// upstream's real `if (control_mode == &mode_auto) set_mode(mode_rtl,
// ...)` guard implicitly (navigate() only runs at all when ModeAUTO is the
// active mode being dispatched through tick(), so the guard is structural
// rather than a runtime check - matches this port's now-repeated "express
// upstream's mode-identity guard structurally instead of porting a
// mode-IDENTIFICATION boolean" precedent, e.g. ModeFBWB/ModeCRUISE/ModeAUTO/
// ModeRTL's own "no run() override IS does_auto_throttle()==true" shape).
// gcs().send_text() is dropped (no GCS).
//
// HOME-BEFORE-AUTO-RTL - A JUDGMENT CALL, PER THE TICKET'S OWN INVITATION:
// ModeRTL's own class banner (SLICE 6) already established "a caller MUST
// call plane.set_home(...) at least once" as a REQUIRED, EXPLICIT step -
// but that requirement predates any AUTOMATIC transition into RTL: every
// prior ModeRTL test/use constructed RTL directly and called set_home()
// itself first. Now that ModeAUTO can transition into RTL PROGRAMMATICALLY,
// with no human caller in the loop at the moment of transition, a mission
// that never had set_home() called on its Plane would RTL toward whatever
// `home` happens to default to - Location() (lat=lng=alt=0), the shared
// fixed reference point every current_loc/prev_WP_loc/next_WP_loc already
// anchors to (SLICE 4's own "CURRENT_LOC" note). Checked this port's own
// existing AUTO tests (vehicle_test.cpp, both the white-box sequencing test
// and the 3-waypoint closed-loop test) - NEITHER calls set_home() anywhere.
// Upstream's own real home-setting is a GPS-lock/arming subsystem this port
// has never had (SLICE 6's own note) - normally guaranteeing home is ALWAYS
// set (to the vehicle's real takeoff point) long before AUTO could ever be
// entered, since a vehicle can't arm without it. This port has no arming
// subsystem to enforce that guarantee, so ModeAUTO::enter() (mode.hpp) now
// includes a REAL, MINIMAL fallback: if `plane_.home` is still exactly the
// default-constructed Location() (the only state distinguishable as "never
// explicitly set" - a Location() being a genuinely INTENDED home would
// coincide with a vehicle already sitting at the shared origin, which is
// exactly this port's own "the origin IS the usual launch point" reasoning,
// quoted directly from SLICE 6's own set_home() note), set it to the
// vehicle's OWN current_loc at the moment AUTO starts - i.e. treat "no
// caller ever called set_home()" as "home is wherever the mission started
// from", which is precisely what upstream's real GPS-lock-at-boot behavior
// usually amounts to in practice (a vehicle typically starts its first
// mission from very near where it was armed). This does NOT override a
// caller's own explicit set_home() call made anywhere else with any other
// value - the check only fires when home is still at its untouched
// default, honoring ModeRTL's own "explicit setup, no automatic magic"
// contract for every OTHER case. Documented here, not silently added,
// exactly as the ticket asked.

// =====================================================================
// CPP-031 SLICE 8 ADDENDUM: RC short (throttle) failsafe - detection,
// debounce, automatic mode switch, and recovery. Upstream (Plane-4.7.0,
// read directly, in full where the ticket asked): ArduPlane/radio.cpp's
// Plane::rc_throttle_value_ok() (~line 333), Plane::rc_failsafe_active()
// (~line 348), and the calling context around ~line 227-262 (the
// allow_failsafe_bypass/has_had_input guard and the 10-up/3-down debounce
// counter, both inside Plane::control_failsafe(), called from
// read_radio()); ArduPlane/events.cpp's Plane::rc_failsafe_short_on_event()
// (~line 21, full) and Plane::rc_failsafe_short_off_event() (~line 243,
// full); ArduPlane/system.cpp's Plane::check_short_rc_failsafe() (~line
// 411, full); ArduPlane/defines.h's failsafe_action_short enum;
// ArduPlane/Parameters.h's ThrFailsafe enum; ArduPlane/Parameters.cpp's
// THR_FAILSAFE/THR_FS_VALUE/FS_SHORT_ACTN real GSCALAR defaults;
// libraries/RC_Channel/RC_Channel.h's RC_Channels::get_fs_timeout_ms() and
// RC_Channels_VarInfo.h's RC_FS_TIMEOUT default.
//
// NEW ENUMS - ThrFailsafe (THR_FAILSAFE, real default Enabled) and
// FsActionShort (FS_SHORT_ACTN, real default BestGuess/0) - both ported
// verbatim from upstream's own enum, including values this port's scope
// never actively branches on distinctly (ThrFailsafe::EnabledNoFS,
// FsActionShort::Circle/Disabled) so that upstream's real comparisons
// (`!= ThrFailsafe::Enabled`, `!= FsActionShort::Disabled`) stay literal
// and correct rather than collapsed into a smaller enum that would need
// re-deriving those comparisons by hand.
//
// FIXEDWINGTUNABLES GAINS FOUR NEW FIELDS (see the struct below):
// throttle_fs_enabled (THR_FAILSAFE, default Enabled), throttle_fs_value
// (THR_FS_VALUE, default 950), fs_action_short (FS_SHORT_ACTN, default
// BestGuess), rc_fs_timeout_ms (RC_FS_TIMEOUT, default 1.0s parameter but
// stored here PRE-CONVERTED to milliseconds - upstream's real
// get_fs_timeout_ms() is `MAX(_fs_timeout * 1000, 100)`; with the real
// 1.0s default that floor never actually binds (1000 > 100), so this
// field's default (1000) IS the real converted value, not a re-derivation
// - a future caller changing it below the 0.1s floor would need to
// reproduce the MAX() clamp itself, documented here rather than silently
// dropped since no caller in this slice's own tests exercises that edge).
//
// RC_THROTTLE_VALUE_OK() / RC_FAILSAFE_ACTIVE() - ported directly, with
// ONE necessary signature divergence (ADR-0012, matching this port's
// established "explicit clock parameter, no singleton millis()"
// precedent - e.g. every now_ms/now_us parameter throughout this file):
// rc_failsafe_active() takes an explicit `now_ms` rather than reading a
// singleton clock.
//
// FAILSAFESTATE - REDUCED FROM UPSTREAM'S Plane::failsafe: upstream's real
// `failsafe` struct (Plane.h) also carries AFS_last_valid_rc_ms (AP_Frsky_
// Telem/rangefinder-linked advanced-failsafe, no such subsystem),
// gcs_last_seen_ms/gcs_check_byte (GCS heartbeat failsafe, out of scope),
// adsb (no ADSB), and a `failsafe_state state` enum with FOUR values
// (NONE/GCS/SHORT/LONG). This port's FailsafeState keeps exactly
// rc_failsafe/throttle_counter/last_valid_rc_ms (all real, in-scope) plus
// last_seen_input_update_count (NEW, see update_throttle_failsafe()'s own
// "NEW-FRAME DETECTION" note) and short_failsafe_active - a plain bool
// substituting for `state != FAILSAFE_NONE` restricted to the NONE/SHORT
// distinction this slice's check_short_rc_failsafe() actually needs
// (FAILSAFE_GCS/FAILSAFE_LONG don't exist in this port's scope at all -
// see the EXCLUDED list below).
//
// MODE-RESTORATION DESIGN - failsafe_saved_mode / mode_set_by_failsafe /
// set_mode()'s NEW from_failsafe PARAMETER: see rc_failsafe_short_off_
// event()'s own doc comment below for the full design rationale (this is
// the ticket's own explicitly-requested "simpler, equally-correct
// substitute" for upstream's saved_mode_number/ModeReason machinery,
// commit 6db7924/CPP-031 slice 7's own note on why ModeReason was dropped
// entirely rather than merely defaulted).
//
// RC_FAILSAFE_SHORT_ON_EVENT() - SCOPE TRACE, PER THE TICKET'S OWN
// INSTRUCTION TO CHECK WHETHER AUTO/RTL HAVE THEIR OWN REAL HANDLING
// FURTHER IN THE FUNCTION (they do, for AUTO - traced by reading events.cpp
// in full, not assumed): upstream's real switch has three case groups plus
// a "never take action" group.
//   - Group 1 (MANUAL/STABILIZE/ACRO/FLY_BY_WIRE_A/AUTOTUNE/FLY_BY_WIRE_B/
//     CRUISE/TRAINING): this port has 4 of those 8 modes (MANUAL/FBWA/
//     FBWB/CRUISE) - all four apply fs_action_short (FBWA/FBWB/else-
//     circle). The emergency_landing-overrides-to-FBWA branch is dropped
//     (no emergency-landing subsystem).
//   - Group 2 (QSTABILIZE/QLOITER/QHOVER/QAUTOTUNE/QACRO, entirely
//     HAL_QUADPLANE_ENABLED): no quadplane modes exist in this port -
//     dropped outright, not one line of it applicable.
//   - Group 3 (AUTO/AUTOLAND/AVOID_ADSB/GUIDED/LOITER/THERMAL) - THE REAL
//     FINDING: AUTO is a MEMBER OF THIS GROUP, not the "never take action"
//     one - it gets the SAME fs_action_short-driven FBWA/FBWB/circle
//     switch as Group 1, gated additionally on `fs_action_short !=
//     BESTGUESS` (i.e. BESTGUESS/0 means "take no action" for THIS group
//     specifically, unlike Group 1 where BESTGUESS falls into the
//     circle/RTL-substitute else-branch) AND on
//     `!failsafe_in_landing_sequence()`. The landing-sequence guard is
//     dropped (no landing-sequence/mission-in-landing-sequence subsystem
//     anywhere in this port - Mission, plane.hpp above, has no such
//     concept) and treated as always-false (never in a landing sequence),
//     per the ticket's own instruction - so AUTO's real gate collapses to
//     just the fs_action_short check, reproduced directly.
//   - Group 4 (CIRCLE/TAKEOFF/RTL/quadplane QLAND/QRTL/LOITER_ALT_QLAND/
//     INITIALISING) - "these modes never take any short failsafe action
//     and continue": RTL is this port's one mode in this group - a real,
//     traced no-op (RTL is already the safe, autonomous response;
//     upstream doesn't re-trigger anything for a vehicle already flying
//     it), not a gap.
//
// NO CIRCLE MODE - A REAL, NAMED GAP, PER THE TICKET'S OWN INSTRUCTION:
// upstream's real "else" branch (fs_action_short is BESTGUESS, CIRCLE, or
// DISABLED - upstream's own if/else-if/else only special-cases FBWA/FBWB
// explicitly, so ALL THREE of those values fall into the same else branch,
// ported literally rather than "fixed" - this port's own "port fixes bugs
// in the port, not upstream" rule) is `set_mode(mode_circle, ...)` - this
// port has never built a CIRCLE mode (not in this slice's six). RTL is
// substituted instead - the closest "safe, autonomous, doesn't need the
// pilot" mode this port actually has. A future slice that adds a real
// CIRCLE mode should replace this substitution in apply_fs_action_short()
// below, not build around it permanently.
//
// RC_FAILSAFE_SHORT_OFF_EVENT() - see its own doc comment below for the
// mode-restoration design. gcs().send_text() calls throughout both event
// handlers are dropped (no GCS subsystem, matching this port's exclusion
// everywhere else).
//
// CHECK_SHORT_RC_FAILSAFE()'S flight_stage GUARD - TREATED AS ALWAYS TRUE:
// upstream's real guard also requires `flight_stage != AP_FixedWing::
// FlightStage::LAND` - no flight_stage/landing-stage subsystem exists in
// this port (same exclusion this file's own calc_speed_scaler() note
// already established for the TAKEOFF flight-stage clamp) - a vehicle in
// this port is never in a landing stage, so this condition is always true
// here, per the ticket's own instruction.
//
// UPDATE_THROTTLE_FAILSAFE()'S ALLOW_FAILSAFE_BYPASS - SIMPLIFIED, A NAMED
// JUDGMENT CALL: upstream's real compound guard is `(g.throttle_fs_enabled
// != ThrFailsafe::Enabled && !failsafe.rc_failsafe) || (allow_failsafe_
// bypass && !has_had_input)`, where `allow_failsafe_bypass = !arming.
// is_armed() && !is_flying() && (rc().enabled_protocols() != 0)`. This
// port has NO arming or is_flying() subsystem anywhere (a repeated
// exclusion throughout this file - see e.g. the "GROUND_MODE /
// REVERSED_THROTTLE" note) - there is no state to read for either half of
// that conjunction. Following this file's own established precedent for a
// missing subsystem (accel_healthy()/ins_healthy()/ground_mode all
// "assume the nominal, unconfigured value" rather than inventing a state
// machine) - a freshly-constructed, not-yet-armed-or-flying vehicle is
// exactly `!arming.is_armed() && !is_flying()` == true, and this port's RC
// is always "enabled" (no protocol-negotiation concept to be zero) - so
// allow_failsafe_bypass collapses to the constant `true` here, and the
// compound guard collapses to just `!has_had_input` (rc_channels.
// has_valid_input() - this port's own real equivalent of has_had_rc_
// receiver(): ap-rc-channel/rc_channels.hpp's own file banner documents it
// as LITERALLY upstream's _has_ever_seen_rc_input, the same latch has_had_
// rc_receiver() reads; has_had_rc_override() is dropped, no GCS-override
// subsystem, rc_channels.hpp's own exclusion list). This IS a real,
// narrow behavior difference from upstream: this port's bypass fires
// purely on "never yet received a valid frame", with no way to also
// require "and currently disarmed/not-flying" - it could only ever matter
// for a caller that arms/flies a vehicle in this port's own tests without
// EVER feeding it one single valid RC frame, not a real flight scenario
// for a failsafe that exists specifically because a real RC link is
// expected.
//
// UPDATE_THROTTLE_FAILSAFE()'S NEW-FRAME DETECTION - see that method's own
// doc comment below; a necessary departure from re-checking RcInput::
// new_input(), traced directly against this port's own existing test
// harness (vehicle_test.cpp's set_sticks() helper), not assumed.
//
// CONTROL_FAILSAFE()'S PILOT-STICK-TO-TRIM RESET - NOT PORTED, A REAL,
// NAMED GAP: upstream's control_failsafe() ALSO overwrites channel_roll/
// channel_pitch/channel_rudder's radio_in/control_in to trim/zero (and
// channel_throttle's control_in to 0, or half-range for a spooled-up
// quadplane) every tick that rc_failsafe_active() is true (BEFORE the
// guard/debounce block, radio.cpp), so a mode like FBWA that reads stick
// input directly doesn't fly on stale/frozen PWM values while the link is
// down. This is out of scope per the ticket's own explicit 4-item scope
// list (detection/debounce, on/off events, tick() wiring) - not one of
// the four - so it is NOT ported here. A vehicle in FBWA during an active
// RC failsafe in this port's own current state will keep reading whatever
// stale channel_roll()/channel_pitch()/rudder_input() values were last
// received, which upstream would instead have already zeroed/trimmed. A
// real, documented divergence for a future slice, not a silent omission.
//
// EXCLUDED (documented, not silently dropped):
//   - check_long_failsafe()/FAILSAFE_LONG/FAILSAFE_GCS (radio.cpp/
//     system.cpp) - a separate, longer-timeout failsafe tier with its own
//     action table (RTL/continue/parachute/land, gated on GCS heartbeat OR
//     a long RC outage) - this slice is short-RC-failsafe only, per the
//     ticket.
//   - GCS heartbeat failsafe (gcs_heartbeat_fs_enabled, GCS_FAILSAFE_*) -
//     no GCS subsystem anywhere in this port.
//   - Battery failsafe (handle_battery_failsafe) - no battery subsystem.
//   - AP_Notify (AP_Notify::flags.failsafe_radio, AP_Notify::events.
//     user_mode_change/user_mode_change_failed) / GCS messaging (gcs().
//     send_text()) throughout both event handlers and set_mode() - no
//     consumer, matching this port's exclusion everywhere else.
//   - HAL_QUADPLANE_ENABLED's VTOL-availability check inside set_mode()
//     (unchanged from CPP-031 slice 7 - no quadplane modes to guard).
//
// See update_throttle_failsafe()/check_short_rc_failsafe()/
// rc_failsafe_short_on_event()/rc_failsafe_short_off_event()/apply_
// fs_action_short() below (Plane class body) and mode.hpp's own "CPP-031
// SLICE 8 NOTE" (on tick()) for the concrete code.

// =====================================================================
// CPP-031 SLICE 9 ADDENDUM: Plane::arm()/disarm() - the real armed/
// disarmed state transition, and the REAL connection of that state to
// RcOutput's already-built (CPP-025), never-before-called safety-switch
// machinery (force_safety_off()/force_safety_on()/safety_state(),
// fwcpp/hal/rc_output.hpp). Upstream (Plane-4.7.0, read directly):
// ArduPlane/AP_Arming_Plane.cpp (477 lines, read in FULL, per the
// ticket) - AP_Arming_Plane::arm()/disarm()/change_arm_state()/
// update_soft_armed() (~line 284-419), rc_received_if_enabled_check()
// (~line 462, full), and pre_arm_checks()/mandatory_checks()/
// ins_checks()/quadplane_checks()/mission_checks() (~lines 47-233, 420,
// skimmed per the ticket's own instruction to confirm applicability, not
// ported); libraries/AP_Arming/AP_Arming.cpp (2196 lines - NOT read in
// full, a deliberately out-of-scope file per the ticket - just
// AP_Arming::arm()/disarm() themselves, ~line 1866-1995, to find the base
// class's own real core beneath AP_Arming_Plane's override) and
// AP_Arming::is_armed_and_safety_off()/is_armed() (~line 304-313).
//
// SCOPE - NOT A PORT OF AP_Arming (~2200 lines, mostly pre-arm checks for
// subsystems this port doesn't have: battery, compass, EKF/NavFilter
// variance, GPS fix-quality/hdop thresholds, board-voltage, logging,
// fence, parameter-range validation, RC-channel-configuration sanity, INS
// health beyond a single simulated IMU, quadplane/mission checks) - this
// slice ports the real, small, APPLICABLE core: the armed/disarmed state
// transition itself, its real and-relevant side effects, and the ONE
// pre-arm check that genuinely applies given this port's actual
// subsystems (RcChannels/SrvChannels/AhrsDcm/Gps/Tecs/L1Control/Mission).
//
// `Plane::armed` - a plain, real bool this port owns directly (default
// false, matching AP_Arming::armed's own real default - a vehicle boots
// disarmed). NOT AP_Arming's `Method`/full checks-enabled/checks-to-skip/
// running_arming_checks/last_arm_time_us state machine - see "METHOD NOT
// PORTED" below.
//
// RC_RECEIVED_IF_ENABLED_CHECK() - THE ONE APPLICABLE PRE-ARM CHECK,
// VERIFIED BY READING ITS REAL BODY (not assumed from the ticket's own
// guess): upstream's real function is exactly
//   if (rc().enabled_protocols() == 0) return true;
//   if (g.throttle_fs_enabled == ThrFailsafe::Enabled &&
//       !(rc().has_had_rc_receiver() || rc().has_had_rc_override()))
//       { check_failed(...); return false; }
//   return true;
// - genuinely small, and genuinely applicable: this port already has a
// real ThrFailsafe/throttle_fs_enabled field (CPP-031 slice 8, RC
// failsafe) and a real "has this vehicle EVER seen a valid RC frame"
// latch (RcChannels::has_valid_input(), reused directly - the file
// banner's own "PRE-ARM CHECKS" precedent update_throttle_failsafe()
// already established for has_had_rc_receiver()). Two upstream pieces
// are dropped, both real, named simplifications:
//   - `rc().enabled_protocols() == 0` bypass - no RC-protocol-enablement
//     subsystem in this port (no SERIALx_PROTOCOL/RC_PROTOCOLS bitmask),
//     so this port's RC is always "some protocol enabled" (the common,
//     unconfigured-board case) - the bypass never fires for a
//     configuration this port could represent anyway.
//   - `has_had_rc_override()` - no MAVLink RC-override subsystem; only
//     the has_had_rc_receiver() half of upstream's OR has a real
//     equivalent here (has_valid_input()).
// Every OTHER pre_arm_checks()/mandatory_checks() check (skimmed in full,
// per the ticket) needs a subsystem this port doesn't have: airspeed
// sensor health, ROLL_LIMIT/PTCH_LIM/AIRSPEED_MIN parameter-range
// validation, reversed-throttle-vs-failsafe-value cross-check, quadplane
// checks (HAL_QUADPLANE_ENABLED, no quadplane), ADSB avoidance failsafe
// (no ADSB), FlightOptions::CENTER_THROTTLE_TRIM (no FlightOptions
// bitmask), mission-in-landing-sequence (no landing-sequence concept,
// same exclusion the RC-failsafe slice's own banner already established
// for AUTO's short-failsafe handling), and Mode::pre_arm_checks() (mode-
// identification machinery already excluded, see mode.hpp's own banner -
// every mode in this port's `enter()` already returns `true`
// unconditionally, matching upstream's own real no-op behavior for
// MANUAL/FBWA/FBWB/CRUISE/AUTO/RTL, none of which override
// pre_arm_checks() upstream either). ins_checks()'s AHRS::pre_arm_check()
// is likewise skipped - no INS-health-monitoring subsystem (same
// exclusion accel_healthy()/ins_healthy() already established, file
// banner's SLICE 3 addendum).
//
// ARM() - THE REAL CORE OF BOTH AP_Arming_Plane::arm() AND THE BASE
// CLASS'S AP_Arming::arm() IT CALLS, COLLAPSED INTO ONE FUNCTION (no
// Method/do_arming_checks parameters - see "METHOD NOT PORTED" below):
//   - idempotency guard (AP_Arming::arm(): "if (armed) return false;") -
//     real, ported directly.
//   - rc_received_if_enabled_check() (above) - the one applicable
//     pre-arm check; AP_Arming::arm() itself calls mandatory_checks()/
//     pre_arm_checks()+arm_checks(), all of which collapse to just this
//     one check for this port's subsystems (see above).
//   - `armed = true` - AP_Arming::arm()'s own real state transition.
//   - `hal.rc_output.force_safety_off()` - see "SAFETY STATE" below for
//     why this is THIS PORT'S real substitute for a board-level physical
//     safety switch, not upstream's own arm()/disarm() behavior verbatim.
//   - `set_home(current_loc)` - see "HOME ON ARM" below.
// AP_Arming::arm()'s remaining real body (verified by reading it
// directly, not assumed) - last_arm_time_us/_last_arm_method bookkeeping
// (no consumer, see "METHOD NOT PORTED"), Log_Write_Arm()/arming_
// failure() (no logging subsystem), terrain->set_reference_location()
// (no terrain subsystem), fence->auto_enable_fence_on_arming() (no
// fence), update_arm_gpio() (no HAL_ARM_GPIO_PIN), the GyroFFT
// prepare_for_arming() hook (no FFT subsystem), and the "Warning: Arming
// Checks Disabled" GCS message (no GCS, and this port has no checks-
// bypass concept to warn about anyway) - all genuinely inapplicable, not
// silently dropped. AP_Arming_Plane::arm()'s OWN additions beyond the
// base class - delay_arming's rising edge (MODE_AUTOLAND-only, no such
// mode), mode_autoland.arm_check() (no AUTOLAND mode), the RUDDER-method
// takeoff-warning timer (see "RUDDER ARMING NOT PORTED" below), and
// send_arm_disarm_statustext() (no GCS) - are excluded for the same
// reasons.
//
// DISARM() - THE REAL CORE OF BOTH LAYERS, SAME COLLAPSE:
//   - idempotency guard (AP_Arming::disarm(): "if (!armed) return
//     false;") - real, ported directly.
//   - `armed = false` - AP_Arming::disarm()'s own real state transition.
//   - `hal.rc_output.force_safety_on()` - see "SAFETY STATE" below. NOTE:
//     this is this port's OWN substitute for AP_Arming_Plane::disarm()'s
//     real behavior here, which is actually a NO-OP for force_safety_on
//     on a typical board (that call lives in the BASE class,
//     AP_Arming::disarm(), gated on `BOARD_SAFETY_OPTION_SAFETY_ON_
//     DISARM` - a board-config option that defaults OFF, verified by
//     reading AP_Arming.cpp directly) - upstream's REAL default behavior
//     leaves the physical safety switch alone on disarm, relying on the
//     separate `is_armed_and_safety_off()` AND-condition to stop motors
//     instead. This port has no separate physical-safety-switch input to
//     leave alone (see "SAFETY STATE" below) - unconditionally forcing
//     safety on at disarm is the honest, safe substitute given that
//     divergence, not a faithful default-BOARD_SAFETY_OPTION reproduction.
//   - `if (control_mode != &mode_auto) mission.reset();` - AP_Arming_
//     Plane::disarm()'s own real line, ported verbatim (new Mission::
//     reset() method, below).
//   - `throttle_suppressed = control_mode->does_auto_throttle();` -
//     AP_Arming_Plane::disarm()'s own real line, ported verbatim (see
//     "DOES_AUTO_THROTTLE() UN-EXCLUDED" and "THROTTLE_SUPPRESSED" below).
// AP_Arming::disarm()'s remaining real body - the RUDDER-method throttle-
// stick/rudder-arming-type gate (see "RUDDER ARMING NOT PORTED" below),
// Log_Write_Disarm()/check_forced_logging() (no logging), GyroFFT save_
// params_on_disarm() (no FFT), fence->auto_disable_fence_on_disarming()
// (no fence), update_arm_gpio() (no HAL_ARM_GPIO_PIN) - all inapplicable.
// AP_Arming_Plane::disarm()'s OWN additions beyond the base class -
// is_flying()-gated GCS/rudder disarm refusal (see "DISARM-WHILE-FLYING
// GAP" below, a REAL, NAMED divergence, not a silent omission), the no-
// aux-switch-assigned airmode-off branch (HAL_QUADPLANE_ENABLED, no
// quadplane), qautotune.disarmed() (no qautotune), new_airspeed_cm reset
// (no DO_CHANGE_SPEED in this port's MissionItem vocabulary - see file
// banner's SLICE 5 addendum, so nothing ever sets new_airspeed_cm away
// from its unused default in the first place), takeoff_state.initial_
// direction reset (MODE_AUTOLAND-only), and send_arm_disarm_statustext()
// (no GCS) - are excluded for the same reasons.
//
// DISARM-WHILE-FLYING GAP - A REAL SAFETY DIFFERENCE FROM UPSTREAM,
// FLAGGED EXPLICITLY, NOT A SILENT OMISSION: upstream's real
// AP_Arming_Plane::disarm() refuses to disarm at all (`return false`,
// motors keep running) for a GCS- or RUDDER-originated disarm while
// `plane.is_flying()` is true - a real in-flight safety interlock. This
// port has no is_flying() concept anywhere (a repeated, standing
// exclusion throughout this file - see e.g. the "GROUND_MODE /
// REVERSED_THROTTLE" and "UPDATE_THROTTLE_FAILSAFE()'S ALLOW_FAILSAFE_
// BYPASS" notes) and, per this slice's own scope (see "METHOD NOT
// PORTED" below), disarm() has no method/caller-origin to distinguish
// GCS/RUDDER from a trusted internal caller in the first place. The
// honest consequence: THIS PORT'S disarm() SUCCEEDS UNCONDITIONALLY (once
// armed), INCLUDING MID-FLIGHT - a caller can cut RcOutput's safety
// switch (and therefore every servo/throttle output) on a vehicle
// SimPlane's own ground truth shows is genuinely airborne. This matches
// this port's established "explicit caller control, no hidden safety net
// beyond what's actually built" pattern (every mode's own enter()/exit()
// is likewise caller-invoked with no automatic guard) - but it is a REAL
// GAP versus upstream's real in-flight protection, not something to
// treat as equivalent. A future slice adding a real is_flying() concept
// should reintroduce this refusal.
//
// SAFETY STATE - RcOutput'S SafetyState IS THIS PORT'S STAND-IN FOR
// UPSTREAM'S PHYSICAL SAFETY SWITCH, A DELIBERATE AND NAMED DIVERGENCE:
// upstream's real hardware safety switch is a BOARD-LEVEL, PHYSICALLY
// INDEPENDENT component - a human presses a button, a GPIO interrupt
// (AP_BoardConfig's own safety-button handling) flips hal.rcout's
// internal safety_switch_state, and arm()/disarm() THEMSELVES NEVER TOUCH
// IT (verified by reading AP_Arming.cpp's real arm()/disarm() bodies in
// full - the ONLY force_safety_on() call in the entire arm/disarm path is
// the BOARD_SAFETY_OPTION_SAFETY_ON_DISARM opt-in noted above; arm() calls
// it NEVER). `is_armed_and_safety_off()` (AP_Arming.cpp: `is_armed() &&
// hal.util->safety_switch_state() != SAFETY_DISARMED`) is upstream's own
// AND of two INDEPENDENTLY-driven states for exactly this reason. This
// port has no physical-safety-switch hardware/GPIO-interrupt subsystem to
// model that independence with - RcOutput::force_safety_off()/
// force_safety_on() (CPP-025) already exist as REAL, tested,
// never-before-connected machinery with EXACTLY the right shape
// (SafetyState::kArmed/kDisarmed, zeroing every non-exempt output channel
// while disarmed) to serve as this port's substitute - so this slice
// makes arm()/disarm() THEMSELVES drive it directly, i.e. treats "safety
// off" as synonymous with "armed" rather than a separate human action.
// This is LESS SAFE than upstream's real independent-switch model in one
// specific way worth naming: upstream lets an operator arm SOFTWARE
// (motors spin down, ready) while the PHYSICAL switch stays safety-on
// (motors genuinely can't move) as a deliberate two-step safety
// procedure; this port's arm() collapses that to one step. Given this
// port has no physical switch hardware to two-step WITH, collapsing is
// the only honest option available (leaving RcOutput's safety machinery
// permanently disconnected, as it was before this slice, would be
// strictly worse - dead code, not a safety feature) - documented here as
// the real, named simplification it is.
//
// IS_ARMED_AND_SAFETY_OFF() BECOMES COMPUTED, NOT CALLER-SUPPLIED - THE
// DESIGN DECISION THE TICKET ASKED FOR: `StabilizeInputs::
// armed_and_safety_off` (SLICE 1's own field, set directly by every
// caller/test until now) is REMOVED. Plane::is_armed_and_safety_off()
// (below) computes it directly from the two real states this slice just
// wired together - `armed && hal.rc_output.safety_state() ==
// hal::SafetyState::kArmed` - matching AP_Arming::is_armed_and_safety_
// off()'s own real AND-of-two-independent-states shape exactly (see
// "SAFETY STATE" above for why, in THIS port, those two states are always
// driven together rather than independently). tick() (mode.hpp) now calls
// `plane.is_armed_and_safety_off()` at its three read sites (drift_
// correction_yaw()/drift_correction_accel()/update_speed_scaler()) instead
// of reading `in.armed_and_safety_off` - the SAME three sites, unchanged
// otherwise. This was chosen over the alternative (keep the field, have
// callers populate it FROM plane.is_armed_and_safety_off() themselves
// before calling tick()) because a field a caller must remember to
// re-derive every tick is exactly the double-source-of-truth bug class
// this design question exists to close - a compile-time guarantee (the
// field no longer EXISTS to set inconsistently) beats a documentation-only
// convention. Blast radius, traced fully: calc_speed_scaler()/
// update_speed_scaler() keep their OWN explicit `armed_and_safety_off`
// bool PARAMETER unchanged (that is a different, already-explicit-context
// design, ADR-0012, not the field being removed here) - only the
// STRUCT FIELD and its 3 read sites in tick() change. Every pre-existing
// vehicle_test.cpp closed-loop test that set `in.armed_and_safety_off =
// true;` (9 sites) is updated to `plane.armed = true; plane.hal.rc_output.
// force_safety_off();` instead - the two real, lower-level primitives that
// combination used to fake, set directly and unconditionally, EXACTLY
// preserving each test's original intent ("pretend armed-and-safety-off is
// true regardless of any real precondition") rather than routing through
// arm()'s own new rc_received_if_enabled_check() gate, which several of
// those tests would fail (set_sticks() isn't called until inside their own
// per-tick loop, AFTER this line runs) - a real, deliberate choice, not an
// oversight: these are integration tests of MANUAL/FBWA/FBWB/CRUISE/AUTO/
// RTL's OWN stabilization/navigation logic, not of arm()'s pre-arm gate
// (which gets its own dedicated tests, see vehicle_test.cpp's new "Plane::
// arm()/disarm()" section), so bypassing that gate exactly as the old
// fake-field approach implicitly did is the correct, minimal-blast-radius
// choice - verified one by one that each test still exercises the same
// real behavior it always did (see this slice's own report for the full
// per-site confirmation).
//
// HOME ON ARM - A REAL, NAMED SIMPLIFICATION OF update_home()'s REAL
// CONDITIONAL LOGIC: AP_Arming_Plane::arm()'s own body is `if (plane.
// update_home()) { if (plane.ahrs.set_home(plane.current_loc)) {
// plane.update_current_loc(); } }` - i.e. home is only actually reset to
// current_loc when update_home() ALSO returns true. Reading update_home()
// (commands.cpp) in full: it gates on `!hal.util->was_watchdog_armed()`
// (no watchdog subsystem), a baro-drift threshold check
// (`g2.home_reset_threshold`, no barometer in this port), and `ahrs.
// home_is_set() && !ahrs.home_is_locked() && gps.status() >=
// GPS_OK_FIX_3D` (a "home was already set once, by a real GPS-lock event,
// and isn't locked" gate this port's Gps/AhrsDcm have no equivalent
// concept for - no home_is_set()/home_is_locked() state machine anywhere
// in this port). Every one of those gates depends on a subsystem this
// port doesn't have, so there is no faithful "sometimes skip it" version
// to reproduce - this slice's arm() calls `set_home(current_loc)`
// UNCONDITIONALLY on every successful arm instead, the honest simplified
// behavior of "home is wherever the vehicle was when it was last armed",
// reusing SLICE 6's own real set_home() unchanged. Ordering consequence,
// documented not silently assumed: current_loc is only ever fresh
// starting from the first tick() call (update_current_loc(), SLICE 4) -
// a caller calling arm() before ever calling tick() gets
// `set_home(Location())` (the shared fixed reference point), exactly
// matching how a freshly-constructed Plane's current_loc already reads
// before its first tick, not a new gap this slice introduces.
//
// DOES_AUTO_THROTTLE() UN-EXCLUDED, MINIMALLY - A REAL CONSUMER NOW
// EXISTS: SLICE 1's own file banner (Mode class hierarchy section)
// explicitly excluded does_auto_throttle() as "mode-IDENTIFICATION
// machinery, not stabilization logic... no consumer needs it" - true
// until now. AP_Arming_Plane::disarm()'s own real
// `plane.throttle_suppressed = plane.control_mode->does_auto_throttle();`
// line is a genuine, small, in-scope consumer this slice adds - so
// `virtual bool does_auto_throttle() const { return false; }` is added to
// the Mode base class (mode.hpp/plane.hpp Mode-hierarchy section),
// overridden `{ return true; }` in ModeFBWB/ModeCRUISE/ModeAUTO/ModeRTL -
// verified against mode.h's own real per-class overrides (ModeManual/
// ModeFBWA rely on the base class's real `false` default; ModeFBWB/
// ModeCruise/ModeAuto/ModeRTL all override `true`; ModeAuto's real body
// is `return !nav_scripting_active();`, AP_SCRIPTING_ENABLED-gated - no
// scripting subsystem in this port, so it collapses to the unconditional
// `true` its own `#else` fallthrough already is) - a plain one-liner per
// class, matching ModeCruise's own get_target_heading_cd() inline-
// accessor precedent, not a re-opening of the broader mode-identification
// exclusion (is_vtol_mode()/does_auto_navigation()/etc. all stay excluded
// - still no real consumer for any of them).
//
// THROTTLE_SUPPRESSED - THE ASSIGNMENT IS PORTED, THE FULL CONSUMER IS
// NOT, A DELIBERATE AND NAMED DISTINCTION: `Plane::throttle_suppressed`
// (new field, default false) receives AP_Arming_Plane::disarm()'s own
// real one-line assignment faithfully. Upstream's REAL consumer of this
// flag, `Plane::suppress_throttle()` (servos.cpp, ~150 lines), is a large
// takeoff-detection/un-suppression state machine (GPS-groundspeed
// thresholds, altitude-above-home checks, launch-duration timers,
// is_flying(), auto_takeoff_check()) - genuinely out of scope, no
// takeoff/is_flying() subsystem anywhere in this port (same standing
// exclusion as the "DISARM-WHILE-FLYING GAP" above). This slice
// deliberately does NOT wire throttle_suppressed into calc_throttle() or
// update_fbwb_speed_height()/update_auto_speed_height()'s TECS-driving
// gate (upstream's OWN real gate, `should_run_tecs && !throttle_
// suppressed` in update_alt(), Plane.cpp) as a partial substitute for
// suppress_throttle() - a considered rejection, not an oversight: wiring
// the gate WITHOUT the un-suppression state machine that upstream ALWAYS
// pairs it with would leave throttle_suppressed a ONE-WAY LATCH in this
// port (nothing would ever clear it back to false - arm() doesn't,
// upstream's own arm() doesn't either, only suppress_throttle()'s own
// state machine does), permanently freezing TECS's throttle demand after
// the FIRST disarm in any auto-throttle mode, for every future re-arm -
// strictly WORSE than upstream, not an equivalent simplification of it.
// Matches this port's own "ground_mode/reversed_throttle" precedent
// (SLICE 1's file banner) - a real field, correctly written by the one
// real assignment this slice's scope covers, left otherwise inert
// pending a future slice that ports suppress_throttle() itself (or a
// smaller, honest substitute for it) as its own dedicated piece of work.
// Covered by a dedicated unit test (vehicle_test.cpp) confirming the
// FLAG's value transitions correctly (true after disarm in an auto-
// throttle mode, unaffected in MANUAL/FBWA) - not an "invented value
// nothing reads" case (SLICE 4's own standing concern) precisely because
// it DOES have a real, named, documented future reader, just not one
// this slice builds.
//
// METHOD NOT PORTED - AP_Arming::Method (RUDDER/GCS/MAVLINK/AUXSWITCH/
// MOTORTEST/SCRIPTING/... - HOW arming was requested) has no consumer
// anywhere in this port's scope: no logging (Log_Write_Arm/Disarm take
// it purely for reporting), no GCS (send_arm_disarm_statustext doesn't
// read it), and the only BEHAVIORAL branches on it (the RUDDER-specific
// checks in both arm()/disarm(), see "RUDDER ARMING NOT PORTED" below)
// are themselves excluded. A single caller-invoked arm()/disarm() with no
// method-tracking is this slice's honest scope - matching this port's own
// established precedent for every mode's enter()/exit() (also caller-
// invoked, no dispatch-reason tracking).
//
// RUDDER ARMING NOT PORTED - a real, separate upstream mechanism (holding
// the rudder stick to one side for a sustained period arms/disarms,
// AP_Arming::RudderArming, checked inside both AP_Arming::arm()'s method==
// RUDDER branch and disarm()'s throttle-must-be-down/rudder-arming-type
// gate) - this slice's arm()/disarm() are programmatic/test-callable entry
// points, same precedent as every mode's own enter()/exit() being directly
// callable rather than gesture-triggered. A future slice could add rudder-
// gesture detection as its own dedicated piece of work, calling this
// slice's arm()/disarm() as its two real terminal actions once a gesture
// is recognized.
//
// See Plane::armed/is_armed_and_safety_off()/rc_received_if_enabled_
// check()/arm()/disarm() and Mission::reset() below (Plane class body,
// just after `home`/`set_home()`) for the concrete code, and mode.hpp's
// own tick() for the 3 updated read sites.
//
// CPP-035 ADDENDUM: a real compass. mode.hpp's tick() has, since CPP-031
// slice 3, constructed a `const ahrs::CompassSample compass;` fresh every
// tick with healthy=false (its own default) - no compass hardware existed
// in this port at all, so AhrsDcm::drift_correction_yaw()'s use_compass()
// (ahrs_dcm.hpp) always fell through to its GPS-ground-course fallback,
// itself only usable above kGpsSpeedMinMs (3 m/s). A stationary or slow
// vehicle had NO way to correct yaw drift. This slice adds a new `Plane`
// member (`compass::Compass compass`, see modules/ap-compass/include/
// fwcpp/compass/compass.hpp's own file banner for the full design: a
// fixed, cited earth-frame magnetic field - real lat/lon-keyed declination
// is a PERMANENT scope boundary, this port has no geodesy - rotated into
// body frame by whichever caller holds true attitude, never by Compass
// itself) and two new StabilizeInputs fields (compass_field_bf/
// compass_healthy, see their own doc comment just below) that a caller
// populates exactly like true_velocity_ned/accel_sample already are.
// mode.hpp's tick() now calls `plane.compass.update(...)` only when
// compass_healthy is true this tick, and builds drift_correction_yaw()'s/
// drift_correction_accel()'s CompassSample argument from
// `plane.compass.sample()` instead of a hardcoded unhealthy default - see
// mode.hpp's own tick() comment for the exact wiring. No existing test's
// behavior changes: compass_healthy defaults false, so plane.compass.
// sample() stays exactly the same default/unhealthy CompassSample every
// pre-existing caller already got.

// =====================================================================
// CPP-031 SLICE 10 ADDENDUM: ModeLOITER (mode.hpp, same module - see its
// own class banner for the mode-level design). Upstream (Plane-4.7.0,
// read directly, in full per the ticket): ArduPlane/mode_loiter.cpp (161
// lines, full) - ModeLoiter::_enter()/update()/navigate()/
// isHeadingLinedUp()/isHeadingLinedUp_cd()/update_target_altitude();
// ArduPlane/commands_logic.cpp's Plane::do_loiter_at_location (~line 952,
// trivial, full); ArduPlane/navigation.cpp's Plane::loiter_angle_reset
// (~line 6, full).
//
// THE SECOND MODE TO USE L1Control's LOITER SUPPORT - AND THE SMALLEST
// SLICE YET, BY DESIGN: ModeRTL (SLICE 6) already built and exercised
// everything this mode's own enter()/update()/navigate() need
// (update_loiter()/update_loiter_update_nav(), the LoiterState struct,
// update_auto_speed_height()'s "drive Tecs yourself" convention below) -
// this slice's real new work is genuinely small: one trivial Plane method
// (do_loiter_at_location()) and three short ModeLOITER method bodies.
// NO NEW Plane-level state is added at all (unlike SLICE 6's `home`/`rtl`
// or SLICE 5's `mission`) - `loiter`/`next_WP_loc` are both reused
// directly, exactly the "a future mode would also need this" reuse
// ModeRTL's own class banner (mode.hpp) predicted when `loiter` was first
// added.
//
// DO_LOITER_AT_LOCATION() - READ IN FULL, GENUINELY TRIVIAL: `if
// (aparm.loiter_radius < 0) loiter.direction = -1; else loiter.direction
// = 1; next_WP_loc = current_loc;` - two statements. The FIRST statement
// is BYTE-FOR-BYTE IDENTICAL to do_RTL()'s own loiter.direction sign-
// setting block above (verified by reading both upstream functions
// directly, not assumed from similarity) - rather than duplicate it a
// third time in this port, both now call a single new private helper,
// set_loiter_direction_from_loiter_radius() (below, in this slice's own
// "CPP-031 SLICE 10 (ModeLOITER)" section, right after update_loiter()) -
// do_RTL()'s own inline block is replaced with a call to it. The SECOND
// statement (`next_WP_loc = current_loc`) has no shared precedent to
// reuse (do_RTL()'s own next_WP_loc assignment targets `home`, a
// different value) - ported directly as ModeLOITER::enter()'s own second
// line (mode.hpp).
//
// UPDATE_AUTO_SPEED_HEIGHT() - A REAL, DELIBERATE DIVERGENCE FROM
// UPSTREAM'S OWN LITERAL mode_loiter.cpp, NOT AN OVERSIGHT: upstream's
// real ModeLoiter::update() (read in full) is exactly `calc_nav_roll();
// [else-branch] calc_nav_pitch(); calc_throttle();` - it NEVER calls
// anything resembling update_auto_speed_height(), because upstream drives
// TECS from a completely separate, mode-independent scheduled task
// (Plane::update_speed_height(), Plane.cpp's scheduler_tasks[]) that runs
// every loop regardless of which mode is active. This port has no
// scheduler task table (SLICE 1's own "a single fixed sequence suffices"
// choice) - CPP-034's own root-cause writeup (mode.hpp's ModeRTL::
// update() "CPP-034 FIX" note) already generalizes the resulting
// convention explicitly: "any mode that calls calc_nav_pitch()/
// calc_throttle() only READS Tecs's last computed pitch/throttle demand -
// something else has to actually DRIVE that demand... exactly as
// ModeAUTO::update() does" - and CPP-034 was filed specifically because
// ModeRTL::update() was missing that call for three slices (6 through the
// RC-failsafe slice) before anyone noticed the resulting non-convergent
// CRUISE-then-RTL orbit. ModeLOITER::update() (mode.hpp) is a SECOND real
// auto-throttle mode (does_auto_throttle() below returns true) calling
// calc_nav_pitch()/calc_throttle() - skipping update_auto_speed_height()
// here would silently reproduce THE EXACT SAME BUG CLASS a second time,
// this time from the very first commit rather than needing a later fix.
// So it is called here from the start, matching ModeAUTO's/the
// now-fixed ModeRTL's own real pattern - NOT a port-specific enhancement
// beyond upstream, but the correct, scheduler-less equivalent of
// upstream's real always-on TECS task, applied consistently to every
// mode that needs it. VERIFIED DIRECTLY, NOT JUST ARGUED: this slice's
// own closed-loop test (vehicle_test.cpp) was first run WITHOUT this
// call, entering ModeLOITER directly from a freshly-constructed Plane
// (TECS never previously driven) - the aircraft's throttle/pitch demand
// stayed frozen at Tecs's own zero-initialized defaults for the entire
// run, and SimPlane's true altitude fell away without bound instead of
// leveling off, which fed back into airspeed/roll dynamics badly enough
// that the horizontal orbit never settled either. Adding the call fixed
// both altitude hold and the orbit's own convergence, first try - see
// this ticket's own report for the exact before/after numbers.
//
// LOITER_ANGLE_RESET() - EXCLUDED, VERIFIED BY READING ITS REAL BODY, NOT
// ASSUMED INAPPLICABLE: upstream's real function (navigation.cpp, read in
// full) is exactly `loiter.sum_cd = 0; loiter.total_cd = 0; loiter.
// reached_target_alt = false; loiter.unable_to_achieve_target_alt =
// false;` - FOUR fields, none of which this port's own smaller LoiterState
// (ModeRTL's own class banner, "LOITER STATE" note) ever declared in the
// first place (verified directly against the struct above: exactly
// direction/start_time_ms/radius, nothing else) - every upstream reader of
// those four fields is itself out of this port's scope (LOITER_TURNS/
// LOITER_TIME/LOITER_TO_ALT mission commands, never in MissionItem's
// vocabulary - SLICE 5's own exclusion list - and isHeadingLinedUp_cd()'s
// own N-lap acceptance-widening math, itself excluded below). Calling a
// function whose entire body writes fields that don't exist would be
// nonsensical, not merely low-value - skipped entirely, not stubbed.
//
// ISHEADINGLINEDUP()/ISHEADINGLINEDUP_CD() (BOTH OVERLOADS) - EXCLUDED,
// NO CONSUMER ANYWHERE IN THIS PORT'S SCOPE, CONFIRMED BY TRACING BOTH
// REAL UPSTREAM CALL PATHS DIRECTLY RATHER THAN ASSUMING: upstream's ONLY
// callers of isHeadingLinedUp() are Plane::verify_loiter_heading() and
// Plane::verify_loiter_to_alt() (commands_logic.cpp's own mission-command
// verification functions - grepped for both directly), which upstream's
// own AP_Mission dispatches for the LOITER_TO_ALT/LOITER_TIME NAV_LOITER
// mission-command types while flying in AUTO. This port never ported
// ANY of the verify_loiter_*() functions, and ModeAUTO::navigate()
// (mode.hpp, read directly) calls exactly one verification function,
// verify_nav_wp() - nothing resembling verify_loiter_heading/to_alt
// exists anywhere in this port's Mission/ModeAUTO. MissionItem's own
// vocabulary (SLICE 5's exclusion list) has no LOITER_TO_ALT/LOITER_TIME/
// LOITER_TURNS/LOITER_UNLIM command types at all for such a verify
// function to even dispatch from. Both overloads would therefore be
// genuinely dead code - a real function with zero callers - if ported;
// left unbuilt entirely, not stubbed or partially wired.
//
// UPDATE_TARGET_ALTITUDE() OVERRIDE - ALREADY-ESTABLISHED, PRE-EXISTING
// EXCLUSION, NOT A NEW GAP THIS SLICE INTRODUCES: verified directly
// against the Mode base class declaration above - `update_target_
// altitude()` is not a concept this port's Mode hierarchy has AT ALL (no
// virtual method by that name, and no OTHER mode - MANUAL/FBWA/FBWB/
// CRUISE/AUTO/RTL - overrides one either, since none of their own class
// banners mention it). Upstream's real Mode::update_target_altitude()
// (mode.h/mode.cpp) is itself part of the terrain-relative/rangefinder-
// altitude machinery this port has never ported (same "no terrain
// subsystem" exclusion throughout this file) - there is simply nothing to
// override.
//
// UPDATE_LOITER_UPDATE_NAV()'S "JUST crosstrack" SIMPLIFICATION GETS A
// SECOND CALLER - RE-VERIFIED SAFE FOR IT, NOT JUST ASSUMED: SLICE 6's own
// file banner note ("UPDATE_LOITER()") collapsed upstream's real gate
// (`loiter.start_time_ms == 0 && (control_mode == &mode_auto ||
// control_mode == &mode_guided) && auto_state.crosstrack && distance > 3
// * radius`) down to just `crosstrack`, reasoning ModeRTL was this port's
// ONLY caller of update_loiter() and do_RTL() always resets crosstrack to
// false anyway. ModeLOITER (this slice) is now a SECOND caller, and
// upstream's own real per-mode half of that gate (`control_mode ==
// &mode_auto || control_mode == &mode_guided`) would ALSO exclude
// mode_loiter (LOITER is neither AUTO nor GUIDED upstream either) - so in
// principle a caller that switched directly from a mid-crosstrack AUTO
// leg straight into LOITER (bypassing do_RTL()'s own crosstrack=false
// reset entirely, since ModeLOITER's own do_loiter_at_location() never
// touches `crosstrack`) could make this port's simplified `crosstrack`-
// only check diverge from upstream's real per-mode-gated behavior for the
// FIRST time. Traced through directly rather than left as a theoretical
// worry: do_loiter_at_location() sets `next_WP_loc = current_loc` -
// i.e. the loiter center coincides EXACTLY with the aircraft's own
// position at the moment of entry, so `current_loc.get_distance(next_WP_
// loc)` is 0m at that instant regardless of crosstrack's value, and stays
// within roughly one loiter radius of 0 for as long as the resulting
// orbit is stable (this slice's own closed-loop test confirms the orbit
// settles near kLoiterRadiusDefault, i.e. nowhere close to `3 * radius`)
// - so the distance half of the compound condition alone keeps this
// branch from ever actually firing for ModeLOITER, in every realistic
// invocation this port can currently produce, independent of whatever
// crosstrack happens to be. Unlike ModeRTL (where `home` can be
// arbitrarily far from `current_loc`, so only crosstrack being false
// protects it), ModeLOITER is protected by its own geometry instead - a
// different real reason for the same practical outcome, not a
// coincidence being relied on twice. Still, this is a narrowing of margin
// worth naming plainly: a FUTURE mode/command that sets next_WP_loc to
// somewhere OTHER than current_loc before dispatching through this same
// shared update_loiter_update_nav() (a mission LOITER_UNLIM command, say)
// would not have this same structural protection, and should tighten the
// check back to a real `control_mode == &mode_auto` comparison (this
// port's own closest equivalent of upstream's real gate - no ModeGUIDED
// exists to compare against either) rather than inherit the "just
// crosstrack" shortcut by default.
//
// RC-FAILSAFE CLASSIFICATION - A REAL, NAMED, NOT-YET-WIRED GAP THIS
// SLICE SURFACES BUT DELIBERATELY DOES NOT CLOSE (OUT OF THIS TICKET'S
// OWN SCOPE): upstream's real rc_failsafe_short_on_event() (events.cpp,
// see file banner's "CPP-031 SLICE 8 ADDENDUM") groups LOITER together
// with AUTO/GUIDED/AUTOLAND/AVOID_ADSB/THERMAL - i.e. LOITER gets the
// SAME FBWA/FBWB/circle-substitute failsafe action AUTO does (gated on
// fs_action_short != BESTGUESS), NOT the CIRCLE/TAKEOFF/RTL "never take
// any action" group `rc_failsafe_short_on_event()`'s own `else` branch
// (plane.hpp, below) currently assumes is exhaustively "control_mode ==
// &mode_rtl". Before this slice, that assumption was airtight - mode_
// loiter did not exist, so nothing could ever prove it wrong. As of this
// slice, `plane.mode_loiter` is a real, settable-via-set_mode() Plane
// member for the FIRST time - a caller that manually did `plane.set_mode
// (plane.mode_loiter)` and then triggered a short RC failsafe would
// silently fall into that `else` branch and get RTL's real "never take
// any action" treatment instead of upstream's real AUTO-like action. No
// code shipped by this ticket can actually reach that path (this slice
// wires no RC-failsafe/mission-command/aux-switch target into
// mode_loiter at all - see this file's own "WHAT'S NEXT" pointer just
// below - so mode_loiter's ONLY callers, this ticket's own tests, drive
// it directly via ModeLOITER's own methods or set_mode(), never through
// a failsafe event), so no EXISTING test's behavior changes - but the gap
// is now real and reachable in principle, not merely hypothetical, and is
// flagged with an inline pointer comment at rc_failsafe_short_on_event()
// itself (below) rather than silently left for a future reader to
// rediscover.
//
// WHAT'S NEXT (not this ticket's scope, flagged per its own closing
// question): with a real, working ModeLOITER now built and tested, the
// natural follow-ups are (1) the RC-failsafe classification fix just
// described, (2) wiring LOITER as a real RC-mode-switch-channel target
// (no aux-function-dispatch/mode-switch-channel subsystem decodes stick
// positions into mode changes anywhere in this port yet - set_mode() itself
// remains programmatic-only, same standing gap SLICE 7's own file banner
// already named), and (3) a LOITER_UNLIM-equivalent NAV command in
// MissionItem's vocabulary so ModeAUTO could dispatch into a loiter
// programmatically mid-mission - none of these are needed for THIS
// slice's own scope (a standalone mode that loiters wherever it was
// entered), and none of this slice's own code depends on any of them.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/compass/compass.hpp>
#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/fw_control/pitch_controller.hpp>
#include <fwcpp/fw_control/roll_controller.hpp>
#include <fwcpp/fw_control/yaw_controller.hpp>
#include <fwcpp/gps/gps.hpp>
#include <fwcpp/hal/hal_context.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/nav/l1_control.hpp>
#include <fwcpp/rc/rc_channels.hpp>
#include <fwcpp/srv/srv_channels.hpp>
#include <fwcpp/tecs/tecs.hpp>

namespace fwcpp::vehicle {

// upstream: ArduPlane/defines.h's MIN_AIRSPEED_MIN (5, m/s) - floor used
// by calc_speed_scaler() below.
inline constexpr float kMinAirspeedMin = 5.0f;

// upstream: ArduPlane/config.h's AP_PLANE_TRIM_THROTTLE_DEFAULT (45) -
// calc_speed_scaler()'s own hardcoded-on-purpose fallback constant (see
// upstream's own comment there: "we use a fixed value here as changing
// the trim throttle value is often done at runtime"), deliberately NOT
// the same field as aparm.throttle_cruise below even though they share
// the same numeric default.
inline constexpr float kTrimThrottleDefault = 45.0f;

// upstream: ArduPlane/defines.h's SERVO_MAX (4500.0, centidegrees = 45
// degrees) - the expo helpers' and SRV angle-channel setup's shared scale.
inline constexpr float kServoMax = 4500.0f;

// upstream default RCMAP_ROLL/PITCH/THROTTLE/YAW (1-indexed 1/2/3/4) -
// see file banner's "RC/SRV CHANNEL INDEX MAPPING" note.
inline constexpr std::uint8_t kChannelRoll = 0;
inline constexpr std::uint8_t kChannelPitch = 1;
inline constexpr std::uint8_t kChannelThrottle = 2;
inline constexpr std::uint8_t kChannelRudder = 3;

// upstream default SERVO1..4 = aileron/elevator/throttle/rudder - see
// file banner.
inline constexpr std::uint8_t kServoAileron = 0;
inline constexpr std::uint8_t kServoElevator = 1;
inline constexpr std::uint8_t kServoThrottle = 2;
inline constexpr std::uint8_t kServoRudder = 3;

// upstream: ArduPlane/config.h's LOITER_RADIUS_DEFAULT (60, meters) -
// Plane::update_loiter()'s own "radius<=1 -> use the general loiter
// radius" fallback, itself only reached when BOTH the passed-in radius
// AND aparm.loiter_radius are <=1 - see update_loiter() below.
inline constexpr float kLoiterRadiusDefault = 60.0f;

// upstream: ArduPlane/Parameters.h's Plane::ThrFailsafe (nested enum
// class) - THR_FAILSAFE. See file banner's "CPP-031 SLICE 8 ADDENDUM".
enum class ThrFailsafe : std::uint8_t {
    Disabled = 0,
    Enabled = 1,
    EnabledNoFS = 2,
};

// upstream: ArduPlane/defines.h's failsafe_action_short - FS_SHORT_ACTN.
// See file banner's "CPP-031 SLICE 8 ADDENDUM".
enum class FsActionShort : std::uint8_t {
    BestGuess = 0, // CIRCLE/no-change(if already AUTO|GUIDED|LOITER) - see file banner
    Circle = 1,
    Fbwa = 2,
    Disabled = 3,
    Fbwb = 4,
};

// Every AP_Param-backed tunable MANUAL/FBWA's real code paths (as scoped
// by the ticket) actually read, as a plain aggregate defaulted to
// upstream's real GSCALAR/ASCALAR/config.h value - same established
// precedent as AcPid::Gains/L1Control::Gains/RollController::Gains
// throughout this port. Every default is cited against Parameters.cpp/
// config.h/defines.h by real parameter name, not invented.
struct FixedWingTunables {
    float roll_limit_deg = 45.0f;       // ROLL_LIMIT_DEG / aparm.roll_limit
    float level_roll_limit_deg = 5.0f;  // LEVEL_ROLL_LIMIT / g.level_roll_limit
    float pitch_limit_max_deg = 20.0f;  // PTCH_LIM_MAX_DEG / aparm.pitch_limit_max
    float pitch_limit_min_deg = -25.0f; // PTCH_LIM_MIN_DEG / aparm.pitch_limit_min (unscaled base value)
    float airspeed_min = 9.0f;          // AIRSPEED_MIN / aparm.airspeed_min, upstream default AIRSPEED_FBW_MIN
    float airspeed_max = 22.0f;         // AIRSPEED_MAX / aparm.airspeed_max, upstream default AIRSPEED_FBW_MAX
    float airspeed_stall = 0.0f;        // AIRSPEED_STALL / aparm.airspeed_stall
    bool stall_prevention = true;       // STALL_PREVENTION / aparm.stall_prevention
    float scaling_speed = 15.0f;        // SCALING_SPEED / g.scaling_speed
    float stab_pitch_down = 2.0f;       // STAB_PITCH_DOWN / g.stab_pitch_down
    bool throttle_passthru_stabilize = false; // THR_PASS_STAB / g.throttle_passthru_stabilize
    float throttle_cruise = 45.0f;      // TRIM_THROTTLE / aparm.throttle_cruise
    float throttle_min = 0.0f;          // THR_MIN / aparm.throttle_min (have_reverse_thrust())
    float pitch_trim_deg = 0.0f;        // PTCH_TRIM_DEG / g.pitch_trim
    float kff_throttle_to_pitch = 0.0f; // KFF_THR2PTCH / g.kff_throttle_to_pitch
    float kff_rudder_mix = 0.5f;        // KFF_RDDRMIX / g.kff_rudder_mix, upstream default RUDDER_MIX
    bool rudder_only = false;           // RUDDER_ONLY / g.rudder_only
    std::int8_t man_expo_roll = 0;      // MAN_EXPO_ROLL / g2.man_expo_roll
    std::int8_t man_expo_pitch = 0;     // MAN_EXPO_PITCH / g2.man_expo_pitch
    std::int8_t man_expo_rudder = 0;    // MAN_EXPO_RUDDER / g2.man_expo_rudder

    // --- CPP-031 slice 2 (FBWB) additions - see file banner addendum ---
    float flybywire_climb_rate = 2.0f;  // FBWB_CLIMB_RATE / g.flybywire_climb_rate, m/s
    float cruise_alt_floor = 0.0f;      // CRUISE_ALT_FLOOR / g.cruise_alt_floor, m (config.h's real CRUISE_ALT_FLOOR default)
    bool flybywire_elev_reverse = false; // FBWB_ELEV_REV / g.flybywire_elev_reverse

    // --- CPP-031 slice 5 (ModeAUTO) additions - see file banner addendum ---
    float waypoint_radius = 90.0f; // WP_RADIUS / g.waypoint_radius, upstream default WP_RADIUS_DEFAULT (config.h), m
    float airspeed_cruise = 12.0f; // AIRSPEED_CRUISE / aparm.airspeed_cruise, upstream default AIRSPEED_CRUISE (config.h), m/s

    // --- CPP-031 slice 6 (ModeRTL) additions - see file banner addendum ---
    float loiter_radius = 60.0f; // WP_LOITER_RAD / aparm.loiter_radius, upstream default LOITER_RADIUS_DEFAULT (config.h), m
    float rtl_altitude = 100.0f; // RTL_ALTITUDE / g.RTL_altitude, upstream default ALT_HOLD_HOME (config.h), m
    float rtl_radius = 0.0f;     // RTL_RADIUS / g.rtl_radius, upstream default 0 (Parameters.cpp), m
    float rtl_climb_min = 0.0f;  // RTL_CLIMB_MIN / g2.rtl_climb_min, upstream default 0 (Parameters.cpp), m

    // --- CPP-031 slice 8 (RC short failsafe) additions - see file banner addendum ---
    ThrFailsafe throttle_fs_enabled = ThrFailsafe::Enabled;   // THR_FAILSAFE / g.throttle_fs_enabled, Parameters.cpp default int(ThrFailsafe::Enabled)
    std::int16_t throttle_fs_value = 950;                     // THR_FS_VALUE / g.throttle_fs_value, Parameters.cpp default 950
    FsActionShort fs_action_short = FsActionShort::BestGuess; // FS_SHORT_ACTN / g.fs_action_short, Parameters.cpp default FS_ACTION_SHORT_BESTGUESS (0)
    std::uint32_t rc_fs_timeout_ms = 1000;                    // RC_FS_TIMEOUT / RC_Channels_VarInfo.h's _fs_timeout, default 1.0s -> RC_Channels::get_fs_timeout_ms()'s real MAX(_fs_timeout*1000, 100), ms - see file banner
};

// Explicit per-tick sensor/environment inputs stabilize_roll()/
// stabilize_pitch()/stabilize_yaw()/calc_speed_scaler() need beyond what
// AhrsDcm/SrvChannels already hold - see file banner's "NO SINGLETONS"
// note.
struct StabilizeInputs {
    bool airspeed_valid = false;       // upstream: ahrs.airspeed_EAS()'s return value
    float airspeed_eas = 0.0f;         // m/s EAS, meaningful only if airspeed_valid
    float eas2tas = 1.0f;              // upstream: ahrs.get_EAS2TAS()
    float accel_y = 0.0f;              // m/s^2, bias-corrected body-frame lateral accel - see file banner
    // armed_and_safety_off REMOVED (CPP-031 slice 9, see file banner's
    // "IS_ARMED_AND_SAFETY_OFF() BECOMES COMPUTED" note) - upstream:
    // arming.is_armed_and_safety_off(), now Plane::is_armed_and_safety_off(),
    // computed from the real Plane::armed/RcOutput::safety_state() this
    // slice wires together, not a field a caller sets directly.
    float dt = 0.0f;                   // seconds since the previous tick
    std::uint32_t now_ms = 0;

    // --- CPP-031 slice 2 (FBWB) additions - see file banner addendum's
    // "CURRENT ALTITUDE INPUT vs. TARGET ALTITUDE STATE" and "100ms RATE
    // LIMIT" notes ---
    float current_altitude_m = 0.0f; // current TRUE altitude above the vehicle's fixed start point (NOT AMSL/home - see file banner)
    std::uint64_t now_us = 0;        // upstream: AP_HAL::micros64() - independent microsecond clock, see file banner

    // --- CPP-031 slice 3 additions - see file banner addendum for each
    // field's full provenance/default-safety rationale ---
    math::Vector3f true_velocity_ned; // upstream: AP::sitl()->state.speedN/E/D - fed to gps.update() every tick
    ahrs::AccelSample accel_sample;   // upstream: get_delta_velocity()/_ins.get_accel() - fed to ahrs.accumulate_accel() every tick
    math::Vector3f wind_estimate;     // upstream: _wind - no wind-estimation subsystem, see file banner
    bool gps_use_enabled = true;      // upstream: AHRS_GPS_USE's real default (GPSUse::Enable) - see file banner

    // --- CPP-031 slice 4 addition - see file banner addendum's
    // "CURRENT_LOC" note ---
    math::Vector3f position_ned; // upstream: nothing this simple (no GPS/EKF position estimate in this port) -
                                  // true NED position (north/east meters) relative to the vehicle's OWN fixed
                                  // start point, matching SimPlane's own `position` convention. Default zero
                                  // vector: with position_ned always (0,0,0), current_loc stays pinned to the
                                  // fixed reference point every tick - harmless for MANUAL/FBWA/FBWB (nothing
                                  // reads current_loc), and a caller not populating this simply never gets a
                                  // usable CRUISE heading-lock geometry (prev_WP_loc/next_WP_loc/current_loc all
                                  // coincide), not a silent wrong answer.

    // --- CPP-035 additions - see modules/ap-compass/include/fwcpp/
    // compass/compass.hpp's own file banner ("WHO COMPUTES THE TRUE
    // BODY-FRAME FIELD" note) for the full design rationale. ---
    //
    // tick() (mode.hpp) cannot supply Compass::update() with true attitude
    // in production - a real compass sensor doesn't need it either (it
    // directly senses the true field; SITL's own Aircraft physics class
    // already did that rotation before AP_Compass_SITL ever sees the
    // result - see compass.hpp's file banner). So, exactly like
    // true_velocity_ned/accel_sample above, the CALLER (a production
    // hardware driver, or a test/SITL-integration harness holding true
    // attitude) supplies the already-body-frame field directly.
    //
    // compass_healthy DEFAULTS FALSE, DELIBERATELY, SO EVERY EXISTING
    // CALLER KEEPS TODAY'S EXACT BEHAVIOR UNCHANGED: mode.hpp's tick()
    // only calls plane.compass.update() when compass_healthy is true THIS
    // TICK; a caller that never touches these two fields never triggers
    // that call, so plane.compass.sample() stays default-constructed
    // (healthy=false, field/declination_rad/last_update_usec all zero)
    // forever - bit-for-bit the same CompassSample every existing test
    // already exercises (mode.hpp used to construct one fresh, unhealthy,
    // every tick; now it reads one that starts unhealthy and STAYS
    // unhealthy under the same "no caller ever asked for a compass"
    // condition). A default/zero compass_field_bf on its own is
    // deliberately NOT treated as "healthy with a zero reading" - a
    // zero-magnitude field is not a real compass reading (it would let
    // calculate_heading()'s first-call atan2(-0,0) silently seed a
    // meaningless heading) - so compass_healthy is a SEPARATE, explicit
    // opt-in, not inferred from compass_field_bf being nonzero.
    math::Vector3f compass_field_bf; // pre-rotated body-frame magnetic field, milliGauss - meaningful only if
                                      // compass_healthy is true this tick.
    bool compass_healthy = false;    // whether compass_field_bf holds a real reading this tick - see above.
};

// upstream: this port's own bound on mission length, NOT upstream's - see
// file banner's "MissionItem / Mission / kMaxMissionItems" note. Upstream's
// real limit is however many commands fit in EEPROM storage (hardware-
// dependent, hundreds to thousands) - a different real constraint this
// port doesn't have, not a shrunken copy of it. 32 comfortably covers this
// slice's own tests and any realistic hand-authored test mission.
inline constexpr std::size_t kMaxMissionItems = 32;

// upstream: AP_Mission::Mission_Command, reduced to exactly what this
// slice's vocabulary supports - see file banner. Deliberately NOT tagged
// with a command "type" at all: every MissionItem IS a NAV_WAYPOINT: there
// is no other kind in this slice (TAKEOFF/LAND/LOITER*/RTL/jump/do-
// commands/splines/VTOL are all excluded - see file banner's exclusion
// list).
struct MissionItem {
    Location loc;
    // 0 means "use the default turn_distance()-based radius" - matches
    // upstream's own cmd_acceptance_distance==0 fallback (commands_logic.cpp
    // Plane::verify_nav_wp) exactly, as a plain float rather than a packed
    // LOWBYTE(p1) byte (which upstream itself caps at 255m - this field has
    // no such packing constraint to reproduce).
    float acceptance_radius_m = 0.0f;
};

// upstream: AP_Mission (libraries/AP_Mission) - THIS PORT'S DELIBERATELY
// SMALLER EQUIVALENT, not a port of it - see file banner's "SCOPE" note.
// A fixed-size (ADR-0012: no dynamic allocation), in-memory, ordered list
// of MissionItems, flown sequentially via current()/peek_next()/advance().
class Mission {
public:
    // Sets the mission list, replacing any previous one, and resets to the
    // first item. TEST/PROGRAMMATIC-ONLY entry point - no MAVLink mission
    // upload or EEPROM storage format in this port (see file banner).
    // Returns false (leaving any existing mission untouched) if
    // items.size() exceeds kMaxMissionItems.
    bool load(std::span<const MissionItem> items) {
        if (items.size() > kMaxMissionItems) {
            return false;
        }
        count_ = items.size();
        for (std::size_t i = 0; i < count_; ++i) {
            items_[i] = items[i];
        }
        current_index_ = 0;
        return true;
    }

    [[nodiscard]] std::size_t size() const { return count_; }
    [[nodiscard]] bool empty() const { return count_ == 0; }

    // upstream: AP_Mission::get_current_nav_cmd() - the waypoint currently
    // being flown toward. nullptr if no mission is loaded.
    [[nodiscard]] const MissionItem* current() const {
        return current_index_ < count_ ? &items_[current_index_] : nullptr;
    }

    // upstream: AP_Mission::get_next_nav_cmd(_nav_cmd.index+1), as used by
    // get_next_ground_course_cd() - the waypoint AFTER the current one,
    // read only by Plane::setup_turn_angle()'s look-ahead. nullptr if the
    // current item is the last one (or no mission is loaded).
    [[nodiscard]] const MissionItem* peek_next() const {
        return (current_index_ + 1 < count_) ? &items_[current_index_ + 1] : nullptr;
    }

    // True once current() is the LAST item in the mission (or the mission
    // is empty) - see plane.hpp file banner's "MISSION COMPLETE" note.
    [[nodiscard]] bool at_last() const { return current_index_ + 1 >= count_; }

    // Moves to the next item. No-op, returns false, if already at_last()
    // (or empty) - see file banner's "MISSION COMPLETE" note for what a
    // caller does with that false return (nothing - holds at the current,
    // final item).
    bool advance() {
        if (at_last()) {
            return false;
        }
        ++current_index_;
        return true;
    }

    // upstream: AP_Mission::reset() (AP_Mission.cpp) - "reset mission to
    // the first command." CPP-031 slice 9 (Plane::disarm(), see plane.hpp
    // file banner's "CPP-031 SLICE 9 ADDENDUM"). Upstream's real reset()
    // also clears nav_cmd_loaded/do_cmd_loaded/do_cmd_all_done/in_landing_
    // sequence/in_return_path flags, jump-tracking, and waypoint history -
    // none of that state exists in THIS port's Mission (see this class's
    // own "SCOPE" note: no do-commands, no landing-sequence concept, no
    // jump/loop commands, no crosstrack-history to reset here - the
    // crosstrack state that DOES exist, Plane::next_wp_crosstrack/
    // crosstrack, is not part of Mission itself, matching upstream's own
    // placement in `auto_state`, not `AP_Mission`) - so this port's
    // reset() is exactly the one piece of upstream's real reset() that
    // has a real equivalent here: go back to the first item.
    void reset() { current_index_ = 0; }

private:
    std::array<MissionItem, kMaxMissionItems> items_{};
    std::size_t count_ = 0;
    std::size_t current_index_ = 0;
};

namespace detail {

// upstream: static file-local channel_expo() (ArduPlane/radio.cpp) and
// AP_Math's expo_curve() (AP_Math.cpp) - used by Plane::roll_in_expo()/
// pitch_in_expo()/rudder_in_expo() below. Not added to ap-math: a single
// self-contained one-liner with exactly one caller family, matching this
// port's precedent of keeping small, single-consumer helpers local rather
// than growing a shared module for them (e.g. kGravityMss's per-module
// copies).
[[nodiscard]] inline float expo_curve(float alpha, float x) { return (1.0f - alpha) * x + alpha * x * x * x; }

[[nodiscard]] inline float channel_expo(rc::RcChannel* chan, std::int8_t expo, bool use_dz) {
    if (chan == nullptr) {
        return 0.0f;
    }
    // upstream: `use_dz? chan->get_control_in() : chan->get_control_in_zero_dz();`
    const float rin = use_dz ? static_cast<float>(chan->control_in) : chan->get_control_in_zero_dz();
    return kServoMax * expo_curve(math::constrain_value(static_cast<float>(expo) * 0.01f, 0.0f, 1.0f), rin / kServoMax);
}

} // namespace detail

// upstream: ArduPlane/defines.h's GPS_GND_CRS_MIN_SPD (5, m/s) - "used to
// set when initial_direction.heading is captured, deciding to heading lock
// in cruise mode" (upstream's own comment, verified directly against
// defines.h line 13, not assumed). Used by ModeCRUISE::navigate() (body:
// mode.hpp).
inline constexpr float kGpsGndCrsMinSpd = 5.0f;

// Forward declaration only - see this file's own "CPP-031 SLICE 7 ADDENDUM"
// banner note for why the Mode class hierarchy below is declared HERE,
// before Plane (Plane needs each concrete Mode subclass to be a COMPLETE
// type to hold it by value), while every Mode/ModeXXX method body that
// touches `plane_` is declared-only below and DEFINED out-of-line in
// mode.hpp, once Plane is complete (mode.hpp `#include`s this file, not the
// other way around - the cycle is fully broken, not hidden).
class Plane;

// =====================================================================
// Mode class hierarchy (CPP-031 slices 1/2/4/5/6/7) - port of ArduPlane's
// Mode base class (ArduPlane/mode.h, 1075 lines + mode.cpp, 414 lines, both
// read in full) and ModeManual/ModeFBWA/ModeFBWB/ModeCruise/ModeAuto/
// ModeRTL, reduced to what this port's six modes actually use. See this
// file's own SLICE 1/2/4/5/6/7 addenda above for the full upstream-vs-port
// design rationale for each mode's own real logic (defined out-of-line in
// mode.hpp - see the SLICE 7 addendum above for why); this section carries
// each class's DATA MEMBERS and METHOD SIGNATURES plus every class-level
// judgment-call comment, matching mode.hpp's own original banner text
// (relocated here, not rewritten, for the class-declaration split this
// slice's SLICE 7 addendum documents).
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
// never existed for it.
//
// Mode::Number/name()/name4()/is_vtol_mode()/is_guided_mode()/
// does_auto_navigation()/does_auto_throttle()/mode_allows_autotuning()/
// allows_throttle_nudging()/use_throttle_limits()/use_battery_
// compensation()/update_target_altitude()/pre_arm_checks() are NOT PORTED -
// mode-IDENTIFICATION machinery, not stabilization logic, out of scope (no
// fence/mission-command-vocabulary/camera/ADSB/arming/battery/aux-dispatch
// subsystem needs any of them - see this file's own repeated exclusions).
// enter()/exit() ARE now ported (CPP-031 slice 7 - see this file's own
// SLICE 7 ADDENDUM above for the full upstream-vs-port mapping and why
// they collapse straight to upstream's protected _enter()/_exit() virtual
// hooks, made public and virtual directly).
//
// Mode::run()'s StickMixing switch (stabilize_stick_mixing_fbw/direct) is
// skipped entirely - see plane.hpp's banner. Every mode in this slice
// behaves as upstream's StickMixing::NONE case (the real default,
// STICK_MIXING param default 0).
class Mode {
public:
    explicit Mode(Plane& plane) : plane_(plane) {}
    virtual ~Mode() = default;
    Mode(const Mode&) = delete;
    Mode& operator=(const Mode&) = delete;

    // CPP-031 slice 7 - see this file's own SLICE 7 ADDENDUM for the full
    // upstream-vs-port mapping. Default success/no-op, matching MANUAL/
    // FBWA's real "nothing to set up" upstream behavior - no mode in this
    // port's scope has a real pre-arm-check-style failure condition to
    // port, so `return true;` here is honest, not a shortcut (verified by
    // a dedicated rollback test using a test-only failing mode,
    // vehicle_test.cpp).
    virtual bool enter() { return true; }
    virtual void exit() {}

    // upstream: Mode::update() (pure virtual) - convert pilot/mode input
    // into nav_roll_cd/nav_pitch_cd targets and/or direct servo output.
    virtual void update(const StabilizeInputs& in) = 0;

    // upstream: Mode::navigate() (mode.h) - "virtual void navigate() {
    // return; }" - a default NO-OP hook, overridden only by navigation
    // modes (ModeCruise/ModeAuto/ModeLoiter/etc - mode.h). MANUAL/FBWA/
    // FBWB never override this (none of them do any waypoint/heading-lock
    // bookkeeping), so the base no-op is their entire behavior here,
    // exactly matching upstream. See mode.hpp's tick() comment for WHERE
    // this is called from and why.
    virtual void navigate(const StabilizeInputs&) {}

    // upstream: Mode::run() (mode.cpp) minus the StickMixing switch (see
    // file banner) - stabilize all three axes. ModeFBWA overrides this to
    // add output_pilot_throttle() after calling the base (matching
    // upstream's ModeFBWA::run(), "Run base class function and then
    // output throttle"); ModeManual overrides it entirely (matching
    // upstream's ModeManual::run(), which does not call Mode::run() at
    // all - MANUAL never stabilizes). Body: mode.hpp (touches plane_ -
    // see this file's own SLICE 7 ADDENDUM for why it is declared, not
    // defined, here).
    virtual void run(const StabilizeInputs& in);

    // upstream: Mode::reset_controllers() (mode.cpp) - PUBLIC there (not
    // protected): mode.hpp's tick() calls it directly on the active mode
    // from outside the Mode hierarchy, exactly as upstream's Plane::
    // stabilize() does on its own 2-second-stale check. steer_state reset
    // is skipped: no ground-steering subsystem (see plane.hpp's banner).
    // Body: mode.hpp.
    void reset_controllers();

    // upstream: Mode::does_auto_throttle() (mode.h) - CPP-031 slice 9, see
    // plane.hpp file banner's "DOES_AUTO_THROTTLE() UN-EXCLUDED" note for
    // why this one piece of mode-identification machinery is now real
    // (Plane::disarm()'s own genuine, in-scope consumer). Default false,
    // matching ModeManual/ModeFBWA's real upstream behavior (both rely on
    // this base default, neither overrides it).
    [[nodiscard]] virtual bool does_auto_throttle() const { return false; }

protected:
    // upstream: Mode::output_pilot_throttle() (mode.cpp) - "Output pilot
    // throttle, this is used in stabilized modes without auto throttle
    // control." Body: mode.hpp.
    void output_pilot_throttle();

    // upstream: Mode::output_rudder_and_steering() (mode.cpp) - "Helper
    // to output to both k_rudder and k_steering servo functions." Body:
    // mode.hpp.
    void output_rudder_and_steering(float val);

    Plane& plane_;
};

// upstream: ModeManual (mode.h) + mode_manual.cpp, read and reproduced in
// full (31 lines) - no exclusions in this one, it is already this small
// upstream. use_battery_compensation()/use_throttle_limits() overrides
// not ported - see file banner. No enter()/exit() override - upstream's
// real _enter() body for ModeManual has nothing this port hasn't already
// excluded elsewhere (see this file's SLICE 7 ADDENDUM), so it relies on
// the base class's default `enter() { return true; }`.
class ModeManual : public Mode {
public:
    using Mode::Mode;

    // Body: mode.hpp (touches plane_).
    void update(const StabilizeInputs&) override;

    // upstream: ModeManual::run() - "reset_controllers();" only. Does NOT
    // call Mode::run() (no stabilization at all in MANUAL); just resets
    // the rate/TECS controllers so they don't accumulate integrator
    // wind-up while MANUAL is active. Body: mode.hpp.
    void run(const StabilizeInputs&) override;
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
// FBWA (see its own doc comment in plane.hpp). No enter()/exit() override
// - same "nothing left to port" reasoning as ModeManual's own note above.
class ModeFBWA : public Mode {
public:
    using Mode::Mode;

    // Body: mode.hpp (touches plane_).
    void update(const StabilizeInputs&) override;

    // upstream: ModeFBWA::run() - "Run base class function and then
    // output throttle." FBWA has manual (pilot-stick) throttle, not
    // auto-throttle (see Mode::does_auto_throttle(), not ported, always
    // false for this slice's two modes). Body: mode.hpp.
    void run(const StabilizeInputs& in) override;
};

// upstream: ModeFBWB (mode.h) + mode_fbwb.cpp, read in full (17 lines) -
// CPP-031 slice 2. See plane.hpp's file banner addendum for the full
// design rationale (altitude reference frame, current-altitude-input vs.
// target-altitude-state split, why Tecs::update_50hz()/
// update_pitch_throttle() are called from update_fbwb_speed_height()
// below rather than mode.hpp's shared tick(), and the FBWB airspeed-
// target surprise).
//
// _enter() IS NOT PORTED/CALLED AUTOMATICALLY, EVEN AFTER CPP-031 SLICE 7 -
// this mode has NO enter() override, relying on the base class's default
// `enter() { return true; }`. Upstream's real _enter() body is just
// `plane.set_target_altitude_current()` (the HAL_SOARING_ENABLED
// init_cruising() call is excluded - no soaring subsystem). A CALLER
// CONSTRUCTING/set_mode()-ING INTO A ModeFBWB MUST STILL CALL
// plane.set_target_altitude_current(current_altitude_cm) EXPLICITLY,
// BEFORE THE FIRST tick()/update() - see this file's own SLICE 7 ADDENDUM
// ("HOME-BEFORE-AUTO-RTL" note's sibling reasoning) for why wiring this
// into a real enter() override is a natural future addition, out of scope
// for this slice (which targets specifically the ModeAUTO-mission-
// complete-to-RTL gap) - otherwise target_altitude_cm starts at its bare
// default (0), not the vehicle's actual current altitude, and FBWB's very
// first pitch/throttle demand would target that instead of "hold where
// you are" as upstream's real mode-entry behavior guarantees.
class ModeFBWB : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeFBWB::update() (mode_fbwb.cpp) - "set nav_roll from
    // the roll stick exactly like FBWA, then update_load_factor(), then
    // update_fbwb_speed_height()." Pitch is NOT set here at all (unlike
    // FBWA) - update_fbwb_speed_height() (plane.hpp) computes nav_pitch_cd
    // from TECS's own pitch demand instead, via calc_nav_pitch(). Body:
    // mode.hpp.
    void update(const StabilizeInputs& in) override;

    // upstream: ModeFBWB has NO run() override at all - relies entirely on
    // the base Mode::run() (stabilize all three axes, see Mode::run()
    // above). Unlike ModeFBWA, FBWB does NOT call output_pilot_throttle()
    // after stabilizing: calc_throttle() (called from
    // update_fbwb_speed_height() above, i.e. during update() - BEFORE
    // run()) already wrote TECS's computed throttle demand straight to
    // the throttle servo function. This "no run() override" shape is a
    // structural expression of does_auto_throttle()==true for FBWB; the
    // override just below makes that boolean itself real too (CPP-031
    // slice 9 - see plane.hpp file banner's "DOES_AUTO_THROTTLE()
    // UN-EXCLUDED" note), now that Plane::disarm() is a real consumer.
    [[nodiscard]] bool does_auto_throttle() const override { return true; }
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
// _enter() IS NOT PORTED/CALLED AUTOMATICALLY, EVEN AFTER CPP-031 SLICE 7 -
// same "nothing left to port beyond what a caller must still do
// explicitly" reasoning as ModeFBWB's own note above. Upstream's real
// _enter() body: `locked_heading = false; lock_timer_ms = 0; plane.
// set_target_altitude_current();` (the HAL_SOARING_ENABLED init_cruising()
// call is excluded - no soaring subsystem, same as ModeFBWB's). The first
// two assignments are already this class's own default member
// initializers below (a freshly-constructed ModeCRUISE starts unlocked
// with no timer running, with no extra call needed) - the ONLY action a
// caller must take explicitly before the first tick()/update(), exactly
// matching ModeFBWB's own precedent, is
// `plane.set_target_altitude_current(current_altitude_cm)`.
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
    // norm_input(), exactly as upstream reads it here. Body: mode.hpp.
    void update(const StabilizeInputs& in) override;

    // upstream: ModeCruise::navigate() (mode_cruise.cpp) - the real
    // heading-lock state machine, read VERY carefully (per the ticket's own
    // instruction). See mode.hpp's tick() comment for WHEN this runs
    // relative to update()/run() in this port. Body: mode.hpp.
    void navigate(const StabilizeInputs& in) override;

    // upstream: ModeCruise::get_target_heading_cd() (mode_cruise.cpp) -
    // trivial accessor, ported for completeness though nothing in this
    // port's scope consumes it (see class banner's EXCLUDED note). Touches
    // only this mode's own private state - stays inline (no Plane
    // dependency, so no SLICE 7 declaration/definition split needed).
    [[nodiscard]] bool get_target_heading_cd(std::int32_t& target_heading) const {
        target_heading = locked_heading_cd_;
        return locked_heading_;
    }

    // upstream: ModeCruise has NO run() override at all - same "auto-
    // throttle mode relies entirely on base Mode::run()" shape as ModeFBWB
    // (see its own banner). does_auto_throttle() (CPP-031 slice 9 - see
    // plane.hpp file banner's "DOES_AUTO_THROTTLE() UN-EXCLUDED" note) is
    // real, ported, true for CRUISE too, matching upstream's own override.
    [[nodiscard]] bool does_auto_throttle() const override { return true; }

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
// file's own "CPP-031 SLICE 5 ADDENDUM" note for the full upstream-vs-port
// mapping, and plane.hpp's file banner addendum for the shared-
// infrastructure design rationale (MissionItem/Mission, the crosstrack
// state machine, the flat-altitude simplification, update_auto_speed_
// height()).
//
// STATE OWNERSHIP - matches CRUISE's own precedent: mission/next_wp_
// crosstrack/crosstrack/next_turn_angle all live on Plane (plane.hpp,
// matching upstream's own Plane.h placement for `mission` and
// `auto_state`), NOT as ModeAUTO-private members - ModeAUTO itself holds
// NO private state at all, unlike ModeCRUISE's locked_heading_/
// lock_timer_ms_/locked_heading_cd_ (there is nothing mode-local to track;
// every piece of AUTO's navigation state is exactly what a future mode
// reading prev_WP_loc/next_WP_loc - e.g. RTL, CPP-031 slice 6 - would also
// need).
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
//     (both in upstream's own _enter()) - no watchdog-persistence or
//     soaring subsystem.
//   - CPP-031 SLICE 7: upstream's real _exit() (mode_auto.cpp) stops a
//     running mission and maybe restarts a landing sequence - BOTH depend
//     entirely on subsystems already excluded (AP_Mission's own MISSION_
//     RUNNING state machine, a landing subsystem - see plane.hpp's own
//     Mission "EXCLUDED" note) - so ModeAUTO relies on the base class's
//     default no-op exit(), a real, traced exclusion, not an oversight.
class ModeAUTO : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeAuto::_enter() - see this file's own "CPP-031 SLICE 5
    // ADDENDUM" note for the real body this reproduces, and the "CPP-031
    // SLICE 7 ADDENDUM" (plane.hpp) "HOME-BEFORE-AUTO-RTL" note for the
    // real, minimal home-fallback this slice adds on top. CPP-031 slice 7
    // makes this a real virtual override (`bool`, `override`) - callable
    // automatically via Plane::set_mode(plane.mode_auto) now, in addition
    // to the direct manual-call precedent every prior slice already used
    // (`plane.mission.load(...)` THEN this method ONCE, before the first
    // tick()/update() while ModeAUTO is active - unchanged). Body:
    // mode.hpp.
    bool enter() override;

    // upstream: ModeAuto::update() - the normal-NAV_WAYPOINT branch only
    // (see this file's own "CPP-031 SLICE 5 ADDENDUM" note for why
    // update_auto_speed_height() is called first). Body: mode.hpp.
    void update(const StabilizeInputs& in) override;

    // upstream: ModeAuto::navigate() - see this file's own "CPP-031 SLICE
    // 5 ADDENDUM" note for the full upstream-vs-port mapping (this port's
    // do_nav_wp()/verify_nav_wp()/mission.advance() replacing AP_Mission::
    // update()'s much larger state machine for exactly this slice's one
    // command type), and the "CPP-031 SLICE 7 ADDENDUM" (plane.hpp) for
    // the real mission-complete-to-RTL transition this slice adds on the
    // `mission.advance()` false branch. Body: mode.hpp.
    void navigate(const StabilizeInputs& in) override;

    // upstream: ModeAuto has NO run() override for the normal-flight case
    // (only the MAV_CMD_NAV_ALTITUDE_WAIT special case does, excluded -
    // see class banner) - relies entirely on base Mode::run(), same "auto-
    // throttle mode relies on the base" shape as ModeFBWB/ModeCRUISE.

    // upstream: ModeAuto::does_auto_throttle() (mode_auto.cpp) - real body
    // is `#if AP_SCRIPTING_ENABLED return !nav_scripting_active(); #endif
    // return true;` - no scripting subsystem in this port (same exclusion
    // throughout this class's own banner), so this collapses to the
    // unconditional `true` its own fallthrough already is. CPP-031 slice 9
    // - see plane.hpp file banner's "DOES_AUTO_THROTTLE() UN-EXCLUDED" note.
    [[nodiscard]] bool does_auto_throttle() const override { return true; }
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
// STATE OWNERSHIP - matches AUTO's own precedent exactly (ModeAUTO class
// banner above): ModeRTL itself holds NO private state at all. `home`/
// `loiter`/`rtl` all live on Plane (plane.hpp, matching upstream's own
// Plane.h placement) - `loiter` in particular is exactly the state a
// FUTURE mode (e.g. LOITER, GUIDED) would also need to read/write, the
// same "keep it where a future reuse would find it" reasoning CRUISE's
// own prev_WP_loc/next_WP_loc placement already established.
//
// _ENTER() - upstream's real _enter() body (after every HAL_QUADPLANE_
// ENABLED branch, excluded - no quadplane in this port) is exactly
// `plane.prev_WP_loc = plane.current_loc; plane.do_RTL(plane.
// get_RTL_altitude_cm()); plane.rtl.done_climb = false;` - reproduced
// directly (body: mode.hpp). CPP-031 slice 7 makes this a real virtual
// override (`bool`, `override`) - callable automatically via Plane::
// set_mode(plane.mode_rtl) now (in particular, from ModeAUTO::navigate()'s
// own mission-complete transition - see plane.hpp's "CPP-031 SLICE 7
// ADDENDUM"), in addition to the direct manual-call precedent every prior
// slice already used. A CALLER MUST STILL CALL plane.set_home(...) AT
// LEAST ONCE before entering ModeRTL for the first time - unchanged by
// this slice (see plane.hpp's own "HOME-BEFORE-AUTO-RTL" note for the one
// real, minimal exception this slice adds: ModeAUTO::enter()'s own
// fallback, which only ever fires when set_home() was never called at
// all). Unlike ModeAUTO::enter() (which takes no StabilizeInputs - it only
// touches current_loc/mission), this needed no StabilizeInputs parameter
// either: get_RTL_altitude_cm() kept its own real upstream zero-arg
// signature (see plane.hpp's own note) precisely because current_loc.alt
// is now live data, not a dead field needing an explicit substitute.
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
//
// CPP-031 SLICE 7: upstream has no real _exit() override for ModeRTL
// (mode.h has no `void _exit() override;` for it - verified directly) -
// relies on the base class's default no-op exit(), unchanged by this
// slice.
class ModeRTL : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeRTL::_enter() - see class banner. Body: mode.hpp.
    bool enter() override;

    // upstream: ModeRTL::update() - see class banner for the
    // CLIMB_BEFORE_TURN exclusion. Body: mode.hpp.
    void update(const StabilizeInputs& in) override;

    // upstream: ModeRTL::navigate() - see class banner for the
    // autoland/mission-jump exclusion. Body: mode.hpp.
    void navigate(const StabilizeInputs& in) override;

    // upstream: ModeRTL has NO run() override at all - relies entirely on
    // base Mode::run(), same "auto-throttle mode relies on the base"
    // shape as ModeFBWB/ModeCRUISE/ModeAUTO.

    // upstream: ModeRTL::does_auto_throttle() (mode.h) - `{ return true; }`,
    // ported directly. CPP-031 slice 9 - see plane.hpp file banner's
    // "DOES_AUTO_THROTTLE() UN-EXCLUDED" note.
    [[nodiscard]] bool does_auto_throttle() const override { return true; }
};

// upstream: ModeLoiter (mode.h) + mode_loiter.cpp (161 lines, read in
// full) - CPP-031 "slice 10". Loiters wherever it was entered - the
// SECOND mode to use L1Control's loiter support (update_loiter()/
// reached_loiter_target()/loiter_radius(), see ModeRTL's own class banner
// above for why real new work here is small: everything this mode's own
// enter()/update()/navigate() need already exists from RTL). See this
// file's own "CPP-031 SLICE 10 ADDENDUM" (file banner, above the
// `#include` block) for the full upstream-vs-port mapping and every
// judgment call - not repeated here beyond a short pointer per item,
// matching this port's own established per-class-banner convention.
//
// STATE OWNERSHIP - same precedent as ModeRTL/ModeAUTO's own class
// banners: ModeLOITER itself holds NO private state. `loiter` (the exact
// same Plane-level LoiterState ModeRTL already uses) and `next_WP_loc`
// are both reused directly - NO new Plane-level state this slice needs
// (unlike ModeRTL, which needed `home`/`rtl`; unlike ModeAUTO, which
// needed `mission`).
//
// _ENTER() - upstream's real _enter() body, after the two EXCLUDED pieces
// below, is exactly `plane.do_loiter_at_location();` (body: mode.hpp),
// always returns true - no real failure condition in this port's scope,
// matching every other mode's own enter() (Mode base class's own doc
// comment above). EXCLUDED, both already-established exclusions
// elsewhere in this port, not new ones this slice introduces - see file
// banner's "CPP-031 SLICE 10 ADDENDUM" for the full trace of each:
//   - `plane.setup_terrain_target_alt(...)` - no terrain subsystem.
//   - the `stick_mixing_enabled() && flight_option_enabled(
//     ENABLE_LOITER_ALT_CONTROL)`-gated `set_target_altitude_current()`
//     call - both stick-mixing-as-an-altitude-controller and the
//     FlightOptions bitmask are already-excluded subsystems. Consequence,
//     same caller-responsibility precedent ModeFBWB's own class banner
//     already established ("_enter() IS NOT PORTED" note): this port's
//     ModeLOITER::enter() NEVER sets target_altitude_cm - A CALLER MUST
//     CALL plane.set_target_altitude_current(current_altitude_cm)
//     EXPLICITLY BEFORE THE FIRST tick()/update() for a stable altitude
//     hold, otherwise TECS targets whatever target_altitude_cm was last
//     left at (0, for a Plane that has never held an altitude-target mode
//     before).
//   - `loiter_angle_reset()` - its real body only resets four fields
//     (sum_cd/total_cd/reached_target_alt/unable_to_achieve_target_alt)
//     this port's own smaller LoiterState never declared in the first
//     place - see file banner.
//
// UPDATE() - calc_nav_roll() (unconditional), then - since stick_mixing_
// enabled()/ENABLE_LOITER_ALT_CONTROL are both excluded (see _ENTER()
// above) - ALWAYS takes upstream's own `else` branch: calc_nav_pitch() +
// calc_throttle() (both pre-existing since FBWB/AUTO/RTL). AP_SCRIPTING_
// ENABLED's "reset altitude while a trick runs" branch is excluded - no
// scripting subsystem. ALSO calls plane.update_auto_speed_height(in)
// FIRST, a real, deliberate divergence from upstream's own literal
// mode_loiter.cpp (which never calls it) - see file banner's "CPP-031
// SLICE 10 ADDENDUM" ("UPDATE_AUTO_SPEED_HEIGHT()" note) for the full
// CPP-034-precedented reasoning and the direct before/after verification.
//
// NAVIGATE() - the ENABLE_LOITER_ALT_CONTROL next_WP_loc-altitude-update
// branch and the AP_SCRIPTING_ENABLED early-return are both excluded
// (same reasons as UPDATE()/_ENTER() above), leaving upstream's own real
// final line: `plane.update_loiter(0);` - "Zero indicates to use
// WP_LOITER_RAD" (upstream's own comment) - verified against update_
// loiter()'s own real `radius <= 1` fallback (plane.hpp, ModeRTL's own
// slice) to confirm 0 genuinely reaches it, not a port-specific
// reinterpretation of "0".
//
// EXCLUDED ENTIRELY, OUT OF THIS PORT'S SCOPE (per the ticket, not a
// partial-dispatch stub) - see file banner's "CPP-031 SLICE 10 ADDENDUM"
// for the full trace of each:
//   - isHeadingLinedUp()/isHeadingLinedUp_cd() (both overloads) - no
//     consumer anywhere in this port's scope (mission-exit-from-loiter
//     heading check, needs verify_loiter_heading()/verify_loiter_to_alt(),
//     neither ever ported, and no LOITER_TO_ALT/LOITER_TIME command type
//     in MissionItem's vocabulary to dispatch them from).
//   - update_target_altitude() override - Mode::update_target_altitude()
//     is not a concept this port's Mode base class has at all - an
//     already-established, pre-existing exclusion, not a new gap.
class ModeLOITER : public Mode {
public:
    using Mode::Mode;

    // upstream: ModeLoiter::_enter() - see class banner. Body: mode.hpp.
    bool enter() override;

    // upstream: ModeLoiter::update() - see class banner for the
    // stick-mixing/ENABLE_LOITER_ALT_CONTROL exclusion and the
    // update_auto_speed_height() divergence. Body: mode.hpp.
    void update(const StabilizeInputs& in) override;

    // upstream: ModeLoiter::navigate() - see class banner for the
    // ENABLE_LOITER_ALT_CONTROL/scripting exclusions. Body: mode.hpp.
    void navigate(const StabilizeInputs& in) override;

    // upstream: ModeLoiter has NO run() override at all - relies entirely
    // on base Mode::run(), same "auto-throttle mode relies on the base"
    // shape as ModeFBWB/ModeCRUISE/ModeAUTO/ModeRTL.

    // upstream: ModeLoiter::does_auto_throttle() (mode.h) - `{ return
    // true; }`, ported directly - same real override every other
    // auto-throttle mode in this port's scope has.
    [[nodiscard]] bool does_auto_throttle() const override { return true; }
};

class Plane {
public:
    // loop_rate_hz feeds HalContext's bundled Scheduler only - this
    // slice's tick() (mode.hpp) does not use task-table dispatch (a
    // single fixed sequence suffices for two modes), so the value is not
    // otherwise read yet. 50 matches ArduPlane's own default SCHED_LOOP_RATE.
    explicit Plane(std::uint16_t loop_rate_hz = 50) : hal(loop_rate_hz), roll_controller(fw_control::RollController::Gains{}, fw_aparm()), pitch_controller(fw_control::PitchController::Gains{}, fw_aparm()), yaw_controller(fw_control::YawController::Gains{}, fw_aparm()), tecs(tecs::Tecs::Gains{}, tecs_aparm()) {
        configure_channels();
    }

    Plane(const Plane&) = delete;
    Plane& operator=(const Plane&) = delete;

    // --- owned subsystems (ADR-0012: explicit context, no singletons) ---
    hal::HalContext hal;
    rc::RcChannels rc_channels;
    srv::SrvChannels srv_channels;
    ahrs::AhrsDcm ahrs;
    gps::Gps gps; // CPP-031 slice 3 (see file banner addendum) - CPP-033's minimal SITL GPS backend.
    // CPP-035 (see modules/ap-compass's own file banner) - fixed-earth-field
    // compass model, default-constructed to a real, cited earth field (see
    // compass.hpp's own "FIXED EARTH-FIELD DEFAULT" note). tick() (mode.hpp)
    // only calls compass.update() when the caller's StabilizeInputs::
    // compass_healthy is true this tick - see that field's own doc comment
    // below for why.
    compass::Compass compass;

    // MUST precede roll_controller/pitch_controller/yaw_controller/tecs -
    // see file banner's "DECLARATION-ORDER CONSTRAINT" note.
    FixedWingTunables aparm;

    fw_control::RollController roll_controller;
    fw_control::PitchController pitch_controller;
    fw_control::YawController yaw_controller;
    tecs::Tecs tecs;

    // CPP-031 slice 4 (see file banner addendum) - Gains{} already carries
    // upstream's real NAVL1_* defaults (l1_control.hpp, CPP-017). No
    // declaration-order constraint (unlike the four controllers above):
    // L1Control's constructor reads only the Gains passed to it.
    nav::L1Control nav_controller{nav::L1Control::Gains{}};

    // --- navigation/attitude-demand state - upstream: Plane.h members ---
    std::int32_t nav_roll_cd = 0;     // upstream: Plane::nav_roll_cd, set by the active mode
    std::int32_t nav_pitch_cd = 0;    // upstream: Plane::nav_pitch_cd, set by the active mode
    std::int32_t roll_limit_cd = 4500; // upstream: Plane::roll_limit_cd, recomputed each tick by update_flight_limits()
    float pitch_limit_min = -25.0f;   // upstream: Plane::pitch_limit_min, recomputed each tick by update_flight_limits()
    float aerodynamic_load_factor = 1.0f; // upstream: Plane::aerodynamic_load_factor
    float smoothed_airspeed = 0.0f;   // upstream: Plane::smoothed_airspeed - see file banner (no airspeed sensor); a
                                       // caller with a real sensor should update this the same way upstream's
                                       // calc_airspeed_errors() does (0.8/0.2 exponential blend), not ported here
                                       // since nothing in this slice's scope writes it (apply_load_factor_roll_limits()
                                       // only reads it).
    float surface_speed_scaler = 1.0f; // upstream: Plane::surface_speed_scaler - low-pass-filtered scaler, see get_speed_scaler()

    // CPP-031 slice 4 (see file banner addendum's "CURRENT_LOC" note) -
    // recomputed fresh every tick by update_current_loc() below from
    // StabilizeInputs::position_ned. Default-constructed Location() (all
    // zero) until the first tick.
    Location current_loc;

    // upstream: Plane::prev_WP_loc/next_WP_loc (Plane.h) - see file
    // banner's "PREV_WP_LOC/NEXT_WP_LOC" note and mode.hpp's ModeCRUISE
    // class banner "STATE OWNERSHIP" note for why these live on Plane, not
    // ModeCRUISE. Written only by ModeCRUISE::navigate() (mode.hpp) once a
    // heading is locked; default-constructed (zero) otherwise.
    Location prev_WP_loc;
    Location next_WP_loc;

    // CPP-031 slice 5 (ModeAUTO, see file banner addendum) - upstream:
    // Plane::mission (AP_Mission) and auto_state's next_wp_crosstrack/
    // crosstrack/next_turn_angle fields (Plane.h). Defaults match
    // upstream's own real defaults exactly: next_wp_crosstrack/crosstrack
    // default false because AP_FixedWing::AutoState is a zero-initialized
    // global at boot (traced, not assumed - see file banner's "SET_NEXT_WP"
    // note for why this matters: a mission's first leg never crosstracks);
    // next_turn_angle defaults to 90.0f, matching AutoState's own in-class
    // initializer AND setup_turn_angle()'s real "no next leg" fallback
    // (see file banner's "SURPRISING UPSTREAM FINDING").
    Mission mission;
    bool next_wp_crosstrack = false;
    bool crosstrack = false;
    float next_turn_angle = 90.0f;

    // CPP-031 slice 6 (ModeRTL, see file banner addendum's "home /
    // set_home()" note) - upstream: Plane::home, normally set by a GPS-
    // lock/arming home-setting subsystem this port doesn't have. Default-
    // constructed (zero - the shared fixed reference point, see SLICE 4's
    // own "CURRENT_LOC" note) until a caller calls set_home() explicitly -
    // REQUIRED at least once before ModeRTL is first entered.
    Location home;

    // upstream: Plane::home is set via a real GPS-lock/arming subsystem
    // this port doesn't have (see file banner) - this is the explicit
    // substitute, matching this port's "explicit setup, no automatic
    // magic" pattern.
    void set_home(const Location& loc) { home = loc; }

    // =================================================================
    // CPP-031 SLICE 9 (Plane::arm()/disarm()) - see file banner addendum
    // "CPP-031 SLICE 9 ADDENDUM" for the full upstream-vs-port design
    // rationale, every excluded AP_Arming/AP_Arming_Plane check, and the
    // real, named safety divergences (SAFETY STATE, DISARM-WHILE-FLYING
    // GAP) - not repeated here beyond a short pointer per judgment call.
    // =================================================================

    // upstream: AP_Arming::armed (private there; AP_Arming::is_armed()
    // exposes it) - default false, a vehicle boots disarmed.
    bool armed = false;

    // upstream: Plane::throttle_suppressed (Plane.h) - see file banner's
    // "THROTTLE_SUPPRESSED" note for why this is a real field with only
    // ONE real writer (disarm(), below) and no wired consumer yet.
    bool throttle_suppressed = false;

    // upstream: AP_Arming::is_armed_and_safety_off() (AP_Arming.cpp:
    // `is_armed() && hal.util->safety_switch_state() != SAFETY_DISARMED`).
    // See file banner's "SAFETY STATE" and "IS_ARMED_AND_SAFETY_OFF()
    // BECOMES COMPUTED" notes for why RcOutput::safety_state() is this
    // port's real stand-in for the physical safety switch, and why this
    // is now a computed method rather than a caller-supplied
    // StabilizeInputs field. AP_Arming::is_armed()'s own `|| arming_
    // required() == Required::NO` OR-term is dropped - no ARMING_REQUIRE
    // parameter/subsystem in this port (a vehicle that never needs
    // arming at all), so `is_armed()` collapses to plain `armed` here.
    [[nodiscard]] bool is_armed_and_safety_off() const {
        return armed && hal.rc_output.safety_state() == hal::SafetyState::kArmed;
    }

    // upstream: AP_Arming_Plane::rc_received_if_enabled_check()
    // (AP_Arming_Plane.cpp) - see file banner's "RC_RECEIVED_IF_ENABLED_
    // CHECK()" note for the real body this reproduces and the two
    // upstream pieces (enabled_protocols()==0 bypass, has_had_rc_
    // override()) dropped as named simplifications.
    [[nodiscard]] bool rc_received_if_enabled_check() const {
        return aparm.throttle_fs_enabled != ThrFailsafe::Enabled || rc_channels.has_valid_input();
    }

    // upstream: AP_Arming_Plane::arm() calling AP_Arming::arm() - see file
    // banner's "ARM()" note for the full mapping of what belongs in this
    // port's core versus what's excluded (Method/do_arming_checks,
    // logging, terrain/fence hooks, GPIO, rudder-arming timers). Returns
    // false (armed left false) if already armed or the one applicable
    // pre-arm check fails.
    bool arm() {
        if (armed) {
            return false;
        }
        if (!rc_received_if_enabled_check()) {
            return false;
        }
        armed = true;
        hal.rc_output.force_safety_off(); // see file banner's "SAFETY STATE" note
        set_home(current_loc);            // see file banner's "HOME ON ARM" note
        return true;
    }

    // upstream: AP_Arming_Plane::disarm() calling AP_Arming::disarm() -
    // see file banner's "DISARM()" note for the full mapping, and
    // "DISARM-WHILE-FLYING GAP" for the real, named safety difference
    // this port has versus upstream's is_flying()-gated refusal (NOT
    // reproduced here - no is_flying() concept in this port, so disarm()
    // always succeeds once armed, even mid-flight). Returns false (no
    // state change) if already disarmed.
    bool disarm() {
        if (!armed) {
            return false;
        }
        armed = false;
        hal.rc_output.force_safety_on(); // see file banner's "SAFETY STATE" note
        if (control_mode != &mode_auto) {
            mission.reset();
        }
        throttle_suppressed = control_mode->does_auto_throttle(); // see file banner's "THROTTLE_SUPPRESSED" note
        return true;
    }

    // CPP-031 slice 6 (ModeRTL, see file banner addendum's "LOITER STATE"
    // note) - upstream: Plane::loiter (Plane.h), reduced to exactly the
    // three fields this slice's update_loiter()/do_RTL()/ModeRTL::
    // navigate() read or write.
    struct LoiterState {
        std::int8_t direction = 1;       // upstream: loiter.direction - 1 clockwise, -1 counter-clockwise
        std::uint32_t start_time_ms = 0; // upstream: loiter.start_time_ms - 0 means "not yet reached the loiter point"
        float radius = 0.0f;             // upstream: loiter.radius - current value of loiter radius in metres used by the controller
    };
    LoiterState loiter;

    // upstream: Plane::rtl (Plane.h) - `struct { bool done_climb; } rtl;`,
    // already exactly this one field, ported directly.
    struct RtlState {
        bool done_climb = false;
    };
    RtlState rtl;

    // See file banner's "GROUND_MODE / REVERSED_THROTTLE" note.
    bool ground_mode = false;
    bool reversed_throttle = false;

    std::uint32_t last_stabilize_ms = 0; // upstream: Plane::last_stabilize_ms (Plane::stabilize()'s 2s-stale check)

    // upstream: Plane::set_control_channels() (radio.cpp) + init_rc_in() -
    // configures the four primary RC input channels' type/range/dead-zone
    // and the four primary servo output channels' function/angle-or-range,
    // both to ArduPlane's real STANDARD-configuration defaults. See file
    // banner's "RC/SRV CHANNEL INDEX MAPPING" note.
    void configure_channels() {
        for (std::uint8_t ch : {kChannelRoll, kChannelPitch, kChannelRudder}) {
            rc::RcChannel* c = rc_channels.channel(ch);
            c->type_in = rc::ControlType::kAngle;
            c->high_in = static_cast<std::int16_t>(kServoMax);
            c->dead_zone = 30;
        }
        rc::RcChannel* thr = rc_channels.channel(kChannelThrottle);
        thr->type_in = rc::ControlType::kRange;
        thr->high_in = 100;
        thr->dead_zone = 30;

        srv_channels.set_default_function(kServoAileron, srv::Function::kAileron);
        srv_channels.channels[kServoAileron].set_angle(static_cast<std::int16_t>(kServoMax));
        srv_channels.set_default_function(kServoElevator, srv::Function::kElevator);
        srv_channels.channels[kServoElevator].set_angle(static_cast<std::int16_t>(kServoMax));
        srv_channels.set_default_function(kServoThrottle, srv::Function::kThrottle);
        srv_channels.channels[kServoThrottle].set_range(100);
        srv_channels.set_default_function(kServoRudder, srv::Function::kRudder);
        srv_channels.channels[kServoRudder].set_angle(static_cast<std::int16_t>(kServoMax));
    }

    [[nodiscard]] rc::RcChannel* channel_roll() { return rc_channels.channel(kChannelRoll); }
    [[nodiscard]] rc::RcChannel* channel_pitch() { return rc_channels.channel(kChannelPitch); }
    [[nodiscard]] rc::RcChannel* channel_throttle() { return rc_channels.channel(kChannelThrottle); }
    [[nodiscard]] rc::RcChannel* channel_rudder() { return rc_channels.channel(kChannelRudder); }

    // upstream: ahrs.roll_sensor/ahrs.pitch_sensor (centidegrees, int32_t)
    // - this port's AhrsDcm keeps roll/pitch as float radians only, so
    // these convert on demand rather than caching a redundant int copy.
    [[nodiscard]] std::int32_t roll_sensor_cd() const { return static_cast<std::int32_t>(math::degrees(ahrs.roll) * 100.0f); }
    [[nodiscard]] std::int32_t pitch_sensor_cd() const { return static_cast<std::int32_t>(math::degrees(ahrs.pitch) * 100.0f); }

    // upstream: Plane::calc_speed_scaler() (Attitude.cpp). `armed_and_
    // safety_off` replaces `arming.is_armed_and_safety_off()` - see file
    // banner. The `!ahrs.using_airspeed_sensor() && SURPRESS_TKOFF_SCALING
    // && flight_stage==TAKEOFF` tail clamp is dropped: needs a
    // FlightOptions bitmask (not ported - no flight-options subsystem)
    // and a takeoff flight-stage concept (this port's flight_stage is
    // always treated as NORMAL per the ticket's own instruction) - both
    // conditions are false/not-applicable for an unconfigured, non-
    // takeoff vehicle, so upstream's own guard never fires here either.
    // HAL_QUADPLANE_ENABLED branches are excluded throughout (no
    // quadplane in this port). `auto_state.highest_airspeed`'s update is
    // dropped too - it is write-only in this slice's scope (its only
    // upstream reader is the just-excluded SURPRESS_TKOFF_SCALING branch).
    [[nodiscard]] float calc_speed_scaler(bool airspeed_valid, float airspeed_eas, bool armed_and_safety_off) const {
        float speed_scaler;
        if (airspeed_valid) {
            const float airspeed_min = std::max(aparm.airspeed_min, kMinAirspeedMin);
            const float scale_min = std::min(0.5f, aparm.scaling_speed / (2.0f * aparm.airspeed_max));
            const float scale_max = std::max(2.0f, aparm.scaling_speed / (0.7f * airspeed_min));
            if (airspeed_eas > 0.0001f) {
                speed_scaler = aparm.scaling_speed / airspeed_eas;
            } else {
                speed_scaler = scale_max;
            }
            speed_scaler = math::constrain_value(speed_scaler, scale_min, scale_max);
        } else if (armed_and_safety_off) {
            const float throttle_out = std::max(srv_channels.get_output_scaled(srv::Function::kThrottle), 1.0f);
            speed_scaler = std::sqrt(kTrimThrottleDefault / throttle_out);
            speed_scaler = math::constrain_value(speed_scaler, 0.6f, 1.67f);
        } else {
            speed_scaler = 1.0f;
        }
        return speed_scaler;
    }

    // upstream: Plane::calc_airspeed_errors()'s speed-scaler low-pass
    // update (navigation.cpp). See file banner's "SPEED-SCALER LOW-PASS"
    // note for why kCutoffHz is 2.0f (the code's real value, not the
    // nearby comment's "1Hz" claim). Upstream calls this from a fixed
    // 10Hz scheduled task; this takes the real dt instead of assuming
    // 10Hz, which keeps calc_lowpass_alpha_dt's cutoff-vs-dt relationship
    // correct at whatever rate a caller actually invokes tick() at.
    void update_speed_scaler(bool airspeed_valid, float airspeed_eas, bool armed_and_safety_off, float dt) {
        const float speed_scaler = calc_speed_scaler(airspeed_valid, airspeed_eas, armed_and_safety_off);
        constexpr float kCutoffHz = 2.0f;
        surface_speed_scaler += math::calc_lowpass_alpha_dt(dt, kCutoffHz) * (speed_scaler - surface_speed_scaler);
    }

    // upstream: Plane::get_speed_scaler() (Plane.h, inline: `return
    // surface_speed_scaler;`).
    [[nodiscard]] float get_speed_scaler() const { return surface_speed_scaler; }

    // upstream: Plane::ahrs_update()'s roll/pitch limit scaling
    // (Plane.cpp), the part of that function actually relevant to this
    // slice (the arming/logging/gyro-summing lines around it are out of
    // scope). `rotate_limits` is unconditionally true here - upstream's
    // only exception is `quadplane.tailsitter.active()`, unreachable with
    // no quadplane in this port. ahrs.cos_pitch()/cos_roll() (AP_AHRS's
    // own cached cosines) are replaced with std::cos() of AhrsDcm's
    // roll/pitch fields directly - numerically identical, AhrsDcm simply
    // doesn't cache the cosine the way upstream's AP_AHRS does.
    void update_flight_limits() {
        roll_limit_cd = static_cast<std::int32_t>(aparm.roll_limit_deg * 100.0f);
        pitch_limit_min = aparm.pitch_limit_min_deg;
        roll_limit_cd = static_cast<std::int32_t>(static_cast<float>(roll_limit_cd) * std::cos(ahrs.pitch));
        pitch_limit_min *= std::fabs(std::cos(ahrs.roll));
    }

    // upstream: Plane::stabilize_roll()/stabilize_roll_get_roll_out()
    // (Attitude.cpp). fly_inverted()'s nav_roll_cd wrap adjustment is
    // skipped - fly_inverted() is always false for MANUAL/FBWA (see its
    // own doc comment below), so upstream's `if (fly_inverted()) { ... }`
    // block is dead code in this slice's scope. The mode_stabilize
    // disable_integrator check and the HAL_QUADPLANE_ENABLED/
    // AP_PLANE_SYSTEMID_ENABLED branches are dropped: mode_stabilize
    // doesn't exist in this port and neither does a quadplane/systemid
    // subsystem.
    void stabilize_roll(const StabilizeInputs& in) {
        const float scaler = get_speed_scaler();
        fw_control::RateLoopInputs rin;
        rin.measured_rate = ahrs.omega.x;
        // roll's own no-sensor fallback (0) - see roll_controller.hpp /
        // AP_RollController::get_airspeed().
        rin.airspeed = in.airspeed_valid ? in.airspeed_eas : 0.0f;
        rin.eas2tas = in.eas2tas;
        rin.dt = in.dt;
        rin.now_ms = in.now_ms;
        const float roll_out =
            roll_controller.get_servo_out(nav_roll_cd - roll_sensor_cd(), scaler, /*disable_integrator=*/false, ground_mode, rin);
        srv_channels.set_output_scaled(srv::Function::kAileron, roll_out);
    }

    // upstream: Plane::stabilize_pitch()/stabilize_pitch_get_pitch_out()
    // (Attitude.cpp). takeoff_tail_hold() and the LANDING_FLARE/
    // FORCE_FLARE_ATTITUDE branch are skipped - no takeoff/landing
    // subsystem in this port. mode_stabilize disable_integrator check and
    // HAL_QUADPLANE_ENABLED/AP_PLANE_SYSTEMID_ENABLED branches dropped,
    // same reasoning as stabilize_roll() above.
    void stabilize_pitch(const StabilizeInputs& in) {
        const float scaler = get_speed_scaler();
        fw_control::PitchInputs pin;
        pin.measured_rate = ahrs.omega.y;
        // pitch's own no-sensor fallback (average of min/max) - see
        // pitch_controller.hpp / AP_PitchController::get_airspeed().
        pin.airspeed = in.airspeed_valid ? in.airspeed_eas : 0.5f * (aparm.airspeed_min + aparm.airspeed_max);
        pin.eas2tas = in.eas2tas;
        pin.dt = in.dt;
        pin.now_ms = in.now_ms;
        pin.bank_angle_rad = ahrs.roll;
        pin.pitch_rad = ahrs.pitch;

        const std::int32_t demanded_pitch = nav_pitch_cd + static_cast<std::int32_t>(aparm.pitch_trim_deg * 100.0f) +
            static_cast<std::int32_t>(srv_channels.get_output_scaled(srv::Function::kThrottle) * aparm.kff_throttle_to_pitch);

        const float pitch_out =
            pitch_controller.get_servo_out(demanded_pitch - pitch_sensor_cd(), scaler, /*disable_integrator=*/false, ground_mode, pin);
        srv_channels.set_output_scaled(srv::Function::kElevator, pitch_out);
    }

    // upstream: Plane::stabilize_yaw()/calc_nav_yaw_coordinated()
    // (Attitude.cpp). Ground steering (calc_nav_yaw_course/calc_nav_yaw_
    // ground, and the `ground_steering` bool that selects between them
    // and the coordinated-turn path) is entirely excluded - see file
    // banner. This always takes upstream's own non-ground-steering
    // branch (`if (!ground_steering) { both k_rudder and k_steering get
    // rudder_output; }`), which is exactly upstream's real in-air
    // behavior for a vehicle with roll stick deflected or above
    // GROUND_STEER_ALT. calc_nav_yaw_coordinated()'s guided-mode and
    // AUTOTUNE-yaw-rate branches are dropped too (no guided/autotune mode
    // in this slice) - always upstream's own final `else` branch
    // (get_servo_out() + aileron-roll-mix + rudder_in).
    void stabilize_yaw(const StabilizeInputs& in) {
        const float scaler = get_speed_scaler();
        fw_control::YawCoordinationInputs yin;
        yin.bank_angle_rad = ahrs.roll;
        yin.gyro_z = ahrs.omega.z;
        yin.accel_y = in.accel_y;
        yin.airspeed_valid = in.airspeed_valid;
        yin.airspeed_eas = in.airspeed_eas;
        yin.now_ms = in.now_ms;

        std::int32_t commanded_rudder = yaw_controller.get_servo_out(scaler, /*disable_integrator=*/false, yin);
        commanded_rudder += static_cast<std::int32_t>(srv_channels.get_output_scaled(srv::Function::kAileron) * aparm.kff_rudder_mix);
        commanded_rudder += rudder_input();
        // using_rate_controller is always false in this slice's scope
        // (no autotune) - upstream unconditionally resets the rate PID
        // whenever the (never-taken-here) rate-controller branch wasn't used.
        yaw_controller.reset_rate_PID();

        const float rudder_output = static_cast<float>(
            math::constrain_value(commanded_rudder, static_cast<std::int32_t>(-4500), static_cast<std::int32_t>(4500)));
        srv_channels.set_output_scaled(srv::Function::kRudder, rudder_output);
        srv_channels.set_output_scaled(srv::Function::kSteering, rudder_output);
    }

    // upstream: Plane::update_load_factor() (Attitude.cpp).
    void update_load_factor() {
        float demanded_roll = std::fabs(static_cast<float>(nav_roll_cd) * 0.01f);
        if (demanded_roll > 85.0f) {
            demanded_roll = 85.0f;
        }
        aerodynamic_load_factor = 1.0f / std::cos(math::radians(demanded_roll));
        apply_load_factor_roll_limits();
    }

    // upstream: Plane::apply_load_factor_roll_limits() (Attitude.cpp).
    // The HAL_QUADPLANE_ENABLED transition-limit branch and the
    // tailsitter-active early return are dropped (no quadplane in this
    // port). `enforce_full_roll_limit` needs FlightOptions::ENABLE_FULL_
    // AERO_LF_ROLL_LIMITS (no flight-options bitmask subsystem) &&
    // ahrs.using_airspeed_sensor() (no airspeed-sensor subsystem) - both
    // default-off/unavailable, so this is always false, matching an
    // unconfigured vehicle's real behavior exactly (not an
    // approximation).
    void apply_load_factor_roll_limits() {
        if (!aparm.stall_prevention) {
            return;
        }
        if (fly_inverted()) {
            return;
        }
        const float stall_airspeed_1g = aparm.airspeed_stall > 0.0f ? aparm.airspeed_stall : aparm.airspeed_min;
        const float denom = std::max(stall_airspeed_1g, 1.0f);
        const float ratio = smoothed_airspeed / denom;
        const float max_load_factor = ratio * ratio;

        constexpr bool kEnforceFullRollLimit = false;
        const float level_roll_limit_deg = aparm.level_roll_limit_deg;
        float lf_roll_limit_deg = aparm.roll_limit_deg;
        if (max_load_factor <= 1.0f) {
            lf_roll_limit_deg = kEnforceFullRollLimit ? level_roll_limit_deg : 25.0f;
        } else if (max_load_factor < aerodynamic_load_factor) {
            lf_roll_limit_deg = math::degrees(std::acos(1.0f / max_load_factor));
            if (!kEnforceFullRollLimit && lf_roll_limit_deg < 25.0f) {
                lf_roll_limit_deg = 25.0f;
            }
            if (lf_roll_limit_deg < level_roll_limit_deg) {
                lf_roll_limit_deg = level_roll_limit_deg;
            }
        }

        const std::int32_t lf_roll_limit_cd = static_cast<std::int32_t>(lf_roll_limit_deg * 100.0f);
        nav_roll_cd = math::constrain_value(nav_roll_cd, -lf_roll_limit_cd, lf_roll_limit_cd);
        roll_limit_cd = std::min(roll_limit_cd, lf_roll_limit_cd);
    }

    // =====================================================================
    // CPP-031 SLICE 4 (ModeCRUISE) - see file banner addendum for the full
    // design rationale (current_loc/nav_controller, calc_nav_roll()'s
    // StabilizeInputs parameter, build_l1_inputs()'s factoring).
    // =====================================================================

    // upstream: nothing this simple exists - see file banner's
    // "CURRENT_LOC" note. Called once per tick from tick() (mode.hpp),
    // unconditionally (cheap, and no pre-existing mode reads current_loc).
    void update_current_loc(const math::Vector3f& position_ned) {
        current_loc = Location();
        current_loc.offset(position_ned.x, position_ned.y);
        // CPP-031 slice 6 (ModeRTL) extension - see file banner addendum's
        // "CURRENT_LOC.ALT" note: current_loc.alt was left permanently at
        // 0 through SLICE 4/5 because nothing in this vehicle's history
        // read it (get_distance/get_distance_NE/past_interval_finish_line
        // are all purely horizontal, verified directly against
        // location.hpp). RTL's get_RTL_altitude_cm()/RTL_CLIMB_MIN check
        // are the first real consumers of current_loc's vertical
        // component, so it is now populated too - position_ned.z is
        // NED-down, so altitude above the vehicle's own fixed start point
        // is -position_ned.z, the SAME relationship every closed-loop
        // test already maintains between position_ned and StabilizeInputs
        // ::current_altitude_m. Safe and non-breaking: no pre-existing
        // consumer of current_loc ever reads Location::alt (traced
        // above), so this cannot change CRUISE's or AUTO's behavior.
        current_loc.set_alt_m(-position_ned.z, Location::AltFrame::ABSOLUTE);
    }

    // Everything L1Control needs from this Plane's own AHRS/GPS/current_loc
    // for one update - see file banner's "build_l1_inputs()" note for why
    // this is factored out and shared between calc_nav_roll() and
    // ModeCRUISE::navigate() (mode.hpp).
    [[nodiscard]] nav::L1Inputs build_l1_inputs(const StabilizeInputs& in) const {
        nav::L1Inputs l1_in;
        l1_in.current_loc = current_loc;
        l1_in.location_valid = true; // see file banner's "build_l1_inputs()" note
        l1_in.groundspeed_vector = gps.sample().velocity_ned.xy();
        l1_in.yaw_rad = ahrs.yaw;
        l1_in.yaw_sensor_cd = static_cast<std::int32_t>(math::rad_to_cd(ahrs.yaw));
        l1_in.pitch_rad = ahrs.pitch;
        l1_in.eas2tas = in.eas2tas;
        l1_in.target_airspeed = 0.0f; // not read by update_waypoint() - see file banner
        l1_in.now_us = static_cast<std::uint32_t>(in.now_us);
        l1_in.now_ms = in.now_ms;
        return l1_in;
    }

    // upstream: Plane::calc_nav_roll() (Attitude.cpp): "nav_roll_cd =
    // constrain(nav_controller->nav_roll_cd(), -roll_limit_cd,
    // roll_limit_cd); update_load_factor();" - takes an explicit
    // StabilizeInputs, unlike upstream's zero-arg version - see file
    // banner's own note for why (ADR-0012, no singleton AP_AHRS to reach
    // into for L1Inputs).
    void calc_nav_roll(const StabilizeInputs& in) {
        const nav::L1Inputs l1_in = build_l1_inputs(in);
        const std::int32_t commanded_roll = nav_controller.nav_roll_cd(l1_in);
        nav_roll_cd = math::constrain_value(commanded_roll, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
    }

    // upstream: Plane::adjust_nav_pitch_throttle() (Attitude.cpp).
    // `flight_stage != VTOL` is always true (no quadplane in this port).
    void adjust_nav_pitch_throttle() {
        const std::int8_t throttle = throttle_percentage();
        if (throttle >= 0 && static_cast<float>(throttle) < aparm.throttle_cruise) {
            const float p = (aparm.throttle_cruise - static_cast<float>(throttle)) / aparm.throttle_cruise;
            nav_pitch_cd -= static_cast<std::int32_t>(aparm.stab_pitch_down * 100.0f * p);
        }
    }

    // upstream: Plane::throttle_percentage() (system.cpp).
    // HAL_QUADPLANE_ENABLED branch dropped (no quadplane in this port).
    [[nodiscard]] std::int8_t throttle_percentage() const {
        const float throttle = srv_channels.get_output_scaled(srv::Function::kThrottle);
        if (!have_reverse_thrust()) {
            return static_cast<std::int8_t>(math::constrain_value(throttle, 0.0f, 100.0f));
        }
        return static_cast<std::int8_t>(math::constrain_value(throttle, -100.0f, 100.0f));
    }

    // upstream: Plane::have_reverse_thrust() (reverse_thrust.cpp).
    [[nodiscard]] bool have_reverse_thrust() const { return aparm.throttle_min < 0.0f; }

    // upstream: Plane::fly_inverted() (control_modes.cpp). Always false
    // for this port's two modes: upstream's first branch is `if
    // (control_mode == &mode_manual) return false;`; its remaining
    // branches depend on `inverted_flight` (settable only via an
    // aux-switch RC option - no aux-dispatch subsystem, CPP-027's own
    // exclusion) and `control_mode == &mode_auto` (no AUTO mode in this
    // slice) - neither is reachable here, so FBWA falls through to the
    // exact same `return false;` MANUAL takes explicitly.
    [[nodiscard]] bool fly_inverted() const { return false; }

    // upstream: Plane::update_fly_forward() (Plane.cpp), read via
    // AP_AHRS::get_fly_forward() - see file banner addendum's "fly_forward()
    // /accel_healthy()/ins_healthy() ADDED AS Plane METHODS" note. Always
    // true for this port's scope: no quadplane, no idle_mode (balloon
    // lift), no LAND flight-stage subsystem, so update_fly_forward()'s real
    // body falls through every conditional to its final unconditional
    // `ahrs.set_fly_forward(true)`.
    [[nodiscard]] bool fly_forward() const { return true; }

    // upstream: AP_InertialSensor::get_accel_health(i) - see file banner
    // addendum. No INS health-monitoring subsystem in this port; this
    // vehicle's accel input IS the real data a caller supplies every tick,
    // so "assume healthy" is this port's honest current behavior, not an
    // invented shortcut.
    [[nodiscard]] bool accel_healthy() const { return true; }

    // upstream: AP_InertialSensor::healthy() - see file banner addendum;
    // same "no health-monitoring subsystem, assume healthy" treatment as
    // accel_healthy() above.
    [[nodiscard]] bool ins_healthy() const { return true; }

    // upstream: Plane::stick_mixing_enabled() (Attitude.cpp), reduced to
    // this slice's scope. fence_stickmixing() (AP_FENCE_ENABLED) is not
    // ported (no fence subsystem) - contributes true, same as upstream
    // with no fence configured. failsafe.rc_failsafe is always false (no
    // failsafe subsystem), so the FBWA-glide suppression branch never
    // fires. does_auto_throttle()/does_auto_navigation() are both false
    // for every mode in this slice (MANUAL, FBWA), so this always reaches
    // upstream's own "non-auto mode: always stick mix" final branch.
    [[nodiscard]] bool stick_mixing_enabled() { return rc_channels.has_valid_input(); }

    // upstream: Plane::rudder_input() (radio.cpp). FlightOptions::
    // DIRECT_RUDDER_ONLY is not ported (no flight-options bitmask
    // subsystem) - defaults disabled, matching an unconfigured vehicle.
    [[nodiscard]] std::int16_t rudder_input() {
        if (aparm.rudder_only) {
            return 0;
        }
        if (!stick_mixing_enabled()) {
            return 0;
        }
        return channel_rudder()->control_in;
    }

    // upstream: Plane::get_throttle_input() (reverse_thrust.cpp).
    [[nodiscard]] float get_throttle_input(bool no_deadzone) {
        if (!rc_channels.has_valid_input()) {
            return 0.0f;
        }
        float ret = no_deadzone ? channel_throttle()->get_control_in_zero_dz() : static_cast<float>(channel_throttle()->control_in);
        if (reversed_throttle) {
            ret = -ret;
        }
        return ret;
    }

    // upstream: Plane::get_adjusted_throttle_input() (reverse_thrust.cpp).
    // Collapses to get_throttle_input(): the CENTER_THROTTLE_TRIM flight
    // option it depends on needs a FlightOptions bitmask this port hasn't
    // ported, and defaults disabled - exactly upstream's own early-return
    // branch (`if ((get_type() != RANGE) || !CENTER_THROTTLE_TRIM) return
    // get_throttle_input(...);`) for an unconfigured vehicle.
    [[nodiscard]] float get_adjusted_throttle_input(bool no_deadzone) { return get_throttle_input(no_deadzone); }

    [[nodiscard]] float roll_in_expo(bool use_dz) { return detail::channel_expo(channel_roll(), aparm.man_expo_roll, use_dz); }
    [[nodiscard]] float pitch_in_expo(bool use_dz) { return detail::channel_expo(channel_pitch(), aparm.man_expo_pitch, use_dz); }
    [[nodiscard]] float rudder_in_expo(bool use_dz) { return detail::channel_expo(channel_rudder(), aparm.man_expo_rudder, use_dz); }

    // =====================================================================
    // CPP-031 SLICE 2 (FBWB) - see file banner addendum for the full design
    // rationale (altitude reference frame, current-altitude-input vs.
    // target-altitude-state split, TECS scheduling, airspeed target).
    // =====================================================================

    // upstream: Plane::target_altitude (Plane.h) - only the non-terrain
    // fields are in scope (see file banner). amsl_cm is reframed as
    // "relative to the vehicle's fixed start point" (see file banner's
    // "ALTITUDE REFERENCE FRAME" note), not literally AMSL.
    std::int32_t target_altitude_cm = 0;       // upstream: target_altitude.amsl_cm
    std::uint64_t fbwb_last_elev_check_us = 0; // upstream: target_altitude.last_elev_check_us
    float fbwb_last_elevator_input = 0.0f;     // upstream: target_altitude.last_elevator_input

    // upstream: Plane::set_target_altitude_current() (altitude.cpp).
    // reset_offset_altitude() (slope-offset reset) is dropped - no
    // altitude-slope/offset state in this port (no mission subsystem to
    // slope between waypoints), so there is nothing to reset. Not called
    // automatically on mode entry - see file banner's "_enter()" note.
    void set_target_altitude_current(std::int32_t current_altitude_cm) { target_altitude_cm = current_altitude_cm; }

    // upstream: Plane::change_target_altitude() (altitude.cpp).
    void change_target_altitude(std::int32_t change_cm) { target_altitude_cm += change_cm; }

    // upstream: Plane::relative_target_altitude_cm() (altitude.cpp), non-
    // terrain/non-rangefinder/non-mission-offset part only (see file
    // banner). A pure passthrough: `target_altitude_cm - home.alt`
    // collapses to just target_altitude_cm since home.alt is
    // definitionally 0 in this port's altitude frame (file banner).
    [[nodiscard]] std::int32_t relative_target_altitude_cm() const { return target_altitude_cm; }

    // upstream: Plane::check_fbwb_altitude() (altitude.cpp). With no
    // AP_FENCE_ENABLED in this port, only the cruise_alt_floor branch is
    // real scope (see file banner) - and since aparm.cruise_alt_floor's
    // real upstream default is 0 (config.h's CRUISE_ALT_FLOOR, verified
    // directly), this is a documented no-op for an unconfigured vehicle,
    // exactly as it is upstream. `home.alt + min_alt_cm` collapses to
    // just min_alt_cm (file banner).
    void check_fbwb_altitude() {
        if (aparm.cruise_alt_floor > 0.0f) {
            const std::int32_t min_alt_cm = static_cast<std::int32_t>(aparm.cruise_alt_floor * 100.0f);
            target_altitude_cm = std::max(target_altitude_cm, min_alt_cm);
        }
    }

    // upstream: Plane::calc_throttle() (Attitude.cpp) - "This is called by
    // TECS-enabled flight modes." The `aparm.throttle_cruise <= 1` zero-
    // throttle escape hatch is upstream's own real behavior (a mission
    // wanting the engine off), ported verbatim.
    void calc_throttle() {
        if (aparm.throttle_cruise <= 1.0f) {
            srv_channels.set_output_scaled(srv::Function::kThrottle, 0.0f);
            return;
        }
        srv_channels.set_output_scaled(srv::Function::kThrottle, tecs.get_throttle_demand());
    }

    // upstream: Plane::calc_nav_pitch() (Attitude.cpp).
    void calc_nav_pitch() {
        const std::int32_t commanded_pitch = tecs.get_pitch_demand();
        nav_pitch_cd = math::constrain_value(commanded_pitch, static_cast<std::int32_t>(pitch_limit_min * 100.0f),
                                              static_cast<std::int32_t>(aparm.pitch_limit_max_deg * 100.0f));
    }

    // upstream: everything TecsInputs needs from the AHRS for one tick
    // (get_relative_position_D_home/get_rotation_body_to_ned/get_EAS2TAS/
    // using_airspeed_sensor/airspeed_EAS/get_pitch_rad, plus
    // AP::ins().get_accel()/AP::baro().get_altitude()) - see file banner's
    // "NO GPS/BARO" note for why velocity_ned_valid is always false here
    // and current_altitude_m substitutes for both the position and baro
    // reads. Factored out so both update_50hz() and update_pitch_
    // throttle() (called from update_fbwb_speed_height() below) build the
    // exact same inputs for one tick, matching upstream's own single-tick
    // consistency (both calls happen back-to-back inside update_alt()).
    [[nodiscard]] tecs::TecsInputs build_tecs_inputs(const StabilizeInputs& in) const {
        tecs::TecsInputs t;
        t.relative_position_d_home_m = -in.current_altitude_m;
        t.velocity_ned_valid = false;
        t.baro_altitude_m = in.current_altitude_m;
        t.accel_ef_z = ahrs.accel_ef.z;
        t.rotation_body_to_ned = ahrs.dcm_matrix;
        t.accel_body_x = (ahrs.dcm_matrix.transposed() * ahrs.accel_ef).x;
        t.eas2tas = in.eas2tas;
        t.using_airspeed_sensor = in.airspeed_valid;
        t.airspeed_eas_valid = in.airspeed_valid;
        t.airspeed_eas = in.airspeed_eas;
        t.roll_rad = ahrs.roll;
        t.pitch_rad = ahrs.pitch;
        t.now_us = in.now_us;
        t.now_ms = in.now_ms;
        return t;
    }

    // upstream: Plane::update_fbwb_speed_height() (navigation.cpp), PLUS
    // (see file banner's "SURPRISING UPSTREAM FINDING #1") Plane::
    // update_alt()'s TECS-driving update_50hz()/update_pitch_throttle()
    // calls, PLUS calc_airspeed_errors()'s FBWB airspeed-target branch
    // (see file banner's "SURPRISING UPSTREAM FINDING #2"). Called once
    // per tick, exclusively from ModeFBWB::update() (mode.hpp).
    void update_fbwb_speed_height(const StabilizeInputs& in) {
        const tecs::TecsInputs tecs_in = build_tecs_inputs(in);
        tecs.update_50hz(tecs_in);

        if (in.now_us - fbwb_last_elev_check_us >= 100000ULL) {
            // we don't run this on every loop - see file banner's "100ms
            // RATE LIMIT" note.
            float dt = static_cast<float>(in.now_us - fbwb_last_elev_check_us) * 1.0e-6f;
            dt = math::constrain_value(dt, 0.1f, 0.15f);
            fbwb_last_elev_check_us = in.now_us;

            float elevator_input = static_cast<float>(channel_pitch()->control_in) * (1.0f / 4500.0f);
            if (aparm.flybywire_elev_reverse) {
                elevator_input = -elevator_input;
            }

            const bool input_stop_climb = !(elevator_input > 0.0f) && fbwb_last_elevator_input > 0.0f;
            const bool input_stop_descent = !(elevator_input < 0.0f) && fbwb_last_elevator_input < 0.0f;
            if (input_stop_climb || input_stop_descent) {
                // user elevator input reached or passed zero - lock in the
                // current altitude.
                set_target_altitude_current(static_cast<std::int32_t>(in.current_altitude_m * 100.0f));
            }

            float climb_rate = aparm.flybywire_climb_rate * elevator_input;
            climb_rate = math::constrain_value(climb_rate, -tecs.get_max_sinkrate(), tecs.get_max_climbrate());

            const std::int32_t alt_change_cm = static_cast<std::int32_t>(climb_rate * dt * 100.0f);
            change_target_altitude(alt_change_cm);

            fbwb_last_elevator_input = elevator_input;
        }

        check_fbwb_altitude();

        // FBW_B/cruise airspeed target - see file banner's "SURPRISING
        // UPSTREAM FINDING #2": the throttle stick (not a fixed cruise
        // speed) maps linearly onto [airspeed_min, airspeed_max].
        const float throttle_input_pct = get_throttle_input(false);
        const std::int32_t eas_dem_cm = static_cast<std::int32_t>(
            (aparm.airspeed_max - aparm.airspeed_min) * throttle_input_pct + aparm.airspeed_min * 100.0f);

        // upstream: the one always-called piece of Plane::
        // apply_throttle_limits() (servos.cpp) this slice actually needs -
        // keeping Tecs's own internal throttle floor (set_throttle_min(),
        // "applicable for one control cycle only" per tecs.hpp) consistent
        // with THIS vehicle's aparm.throttle_min every tick. Without this,
        // Tecs::thrminf_ext_ decays back toward ITS OWN default of -1.0
        // (full reverse) every cycle regardless of whether this vehicle is
        // actually configured for reverse thrust (have_reverse_thrust()).
        // apply_throttle_limits() itself is NOT ported in full - its
        // ICEngine/battery-watt-limiter/takeoff/quadplane branches all
        // depend on subsystems this port doesn't have, making it a
        // materially bigger function than this slice's scope - but
        // leaving Tecs's floor silently inconsistent with this port's own
        // have_reverse_thrust() would be a real, avoidable gap, not a
        // faithful "not yet built" exclusion.
        if (!have_reverse_thrust()) {
            tecs.set_throttle_min(0.0f);
        }

        tecs.update_pitch_throttle(relative_target_altitude_cm(), eas_dem_cm, in.current_altitude_m, aerodynamic_load_factor,
                                    tecs_in);

        calc_throttle();
        calc_nav_pitch();
    }

    // =====================================================================
    // CPP-031 SLICE 5 (ModeAUTO) - see file banner addendum for the full
    // design rationale (MissionItem/Mission, the crosstrack state machine,
    // the flat-altitude simplification, update_auto_speed_height()'s own
    // reason for existing separately from update_fbwb_speed_height()).
    // =====================================================================

    // upstream: Plane::get_wp_radius() (commands_logic.cpp) - the
    // HAL_QUADPLANE_ENABLED branch is dropped (no quadplane in this port),
    // so this collapses to upstream's own non-quadplane fallback: a plain
    // tunable read.
    [[nodiscard]] float get_wp_radius() const { return aparm.waypoint_radius; }

    // upstream: Plane::setup_turn_angle() (navigation.cpp), including
    // AP_Mission::get_next_ground_course_cd()'s real look-ahead (traced
    // directly, not assumed - see file banner's "SURPRISING UPSTREAM
    // FINDING"). Called only from set_next_WP() below, after prev_WP_loc/
    // next_WP_loc have already been updated for the leg being started.
    void setup_turn_angle() {
        const MissionItem* next_item = mission.peek_next();
        if (next_item == nullptr) {
            // upstream: get_next_ground_course_cd(-1)'s sentinel return
            // (no next nav command) - the real fallback is 90 degrees, not
            // 0 (see file banner).
            next_turn_angle = 90.0f;
            return;
        }
        const std::int32_t next_ground_course_cd = next_WP_loc.get_bearing_to(next_item->loc);
        const std::int32_t ground_course_cd = prev_WP_loc.get_bearing_to(next_WP_loc);
        next_turn_angle = static_cast<float>(math::wrap_180_cd(next_ground_course_cd - ground_course_cd)) * 0.01f;
    }

    // upstream: Plane::set_next_WP() (commands.cpp), read in full. EXCLUDED
    // (see file banner): the `next_WP_loc.lat==0 && lng==0` "loiter on the
    // spot" convenience (no loiter commands in this slice's vocabulary),
    // fix_terrain_WP()/change_alt_frame() (no terrain subsystem, and a
    // MissionItem's Location is assumed already in this port's one
    // collapsed altitude frame), loiter_angle_reset() (loiter-specific),
    // and setup_alt_slope()/adjust_altitude_target() (replaced with the
    // flat `target_altitude_cm = next_WP_loc.alt` assignment below - see
    // file banner's "ALTITUDE SLOPE - DEFERRED" note). Everything else -
    // the crosstrack-state machine and the "already past the waypoint"
    // catch-up - is reproduced exactly.
    void set_next_WP(const Location& loc) {
        if (next_wp_crosstrack) {
            // copy the current WP into the OldWP slot
            prev_WP_loc = next_WP_loc;
            crosstrack = true;
        } else {
            // we should not try to cross-track for this waypoint
            prev_WP_loc = current_loc;
            // use cross-track for the next waypoint
            next_wp_crosstrack = true;
            crosstrack = false;
        }

        next_WP_loc = loc;

        // are we already past the waypoint? This happens when we jump
        // waypoints, and it can cause us to skip a waypoint. If we are
        // past the waypoint when we start on a leg, then use the current
        // location as the previous waypoint, to prevent immediately
        // considering the waypoint complete.
        if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            prev_WP_loc = current_loc;
        }

        // Flat per-waypoint target altitude - see file banner's "ALTITUDE
        // SLOPE - DEFERRED" note. Reuses the SAME target_altitude_cm SLICE
        // 2 (FBWB) established, rather than inventing a second mechanism.
        target_altitude_cm = next_WP_loc.alt;

        setup_turn_angle();
    }

    // upstream: Plane::do_nav_wp() (commands_logic.cpp) - "set_next_WP(cmd.
    // content.location);" DELIBERATE SIGNATURE DIFFERENCE from upstream:
    // no AP_Mission::Mission_Command parameter - this port's Plane already
    // owns the whole `mission`, so do_nav_wp() reads mission.current()
    // itself rather than requiring a caller to resolve and pass it. A
    // no-op if no mission is loaded (defensive - every real caller in this
    // slice checks mission.current() first, see ModeAUTO, mode.hpp).
    void do_nav_wp() {
        const MissionItem* item = mission.current();
        if (item == nullptr) {
            return;
        }
        set_next_WP(item->loc);
    }

    // upstream: Plane::verify_nav_wp() (commands_logic.cpp), read in full -
    // the in-scope subset only, per the ticket's own instruction. Takes an
    // explicit L1Inputs (ADR-0012, same treatment calc_nav_roll() already
    // gets) built by the caller from build_l1_inputs() - see ModeAUTO's own
    // navigate() (mode.hpp). EXCLUDED (see file banner and mode.hpp's
    // ModeAUTO class banner for the exact upstream lines skipped): the
    // pass-by distance (cmd_passby/HIGHBYTE(p1)) and its flex_next_WP_loc
    // offset-along-track machinery, and the waypoint_max_radius override
    // (g.waypoint_max_radius) - both entirely absent, not merely
    // defaulted-off. steer_state.hold_course_cd's reset is dropped (no
    // ground-steering subsystem, same exclusion plane.hpp's SLICE 1 banner
    // already documents).
    [[nodiscard]] bool verify_nav_wp(const nav::L1Inputs& l1_in) {
        const MissionItem* item = mission.current();
        if (item == nullptr) {
            return false;
        }

        if (crosstrack) {
            nav_controller.update_waypoint(prev_WP_loc, next_WP_loc, l1_in);
        } else {
            nav_controller.update_waypoint(current_loc, next_WP_loc, l1_in);
        }

        // upstream: `cmd_acceptance_distance > 0 ? cmd_acceptance_distance
        // : (cmd_passby == 0 ? nav_controller->turn_distance(get_wp_radius(),
        // auto_state.next_turn_angle) : 0)` - cmd_passby is always 0 in
        // this slice's vocabulary (no pass-by field on MissionItem), so
        // this collapses to the two real remaining cases.
        const float acceptance_distance_m = item->acceptance_radius_m > 0.0f
            ? item->acceptance_radius_m
            : nav_controller.turn_distance(get_wp_radius(), next_turn_angle, l1_in);

        const float wp_dist = current_loc.get_distance(next_WP_loc);
        if (wp_dist <= acceptance_distance_m) {
            return true;
        }

        // have we flown past the waypoint?
        if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            return true;
        }

        return false;
    }

    // upstream: the "should_run_tecs" branch of Plane::update_alt()
    // (Plane.cpp) PLUS calc_airspeed_errors()'s AUTO airspeed-target branch
    // (mode_auto_target_airspeed_cm(), navigation.cpp) - see file banner's
    // "UPDATE_AUTO_SPEED_HEIGHT()" note for why this is its OWN function
    // rather than a reuse of update_fbwb_speed_height() above (that
    // function's elevator-stick/throttle-stick logic is real FBWB/CRUISE-
    // only behavior upstream itself never runs for AUTO). Called once per
    // tick from ModeAUTO::update() (mode.hpp) BEFORE calc_nav_pitch()/
    // calc_throttle() read TECS's demand - and, as of CPP-034 (see mode.
    // hpp's own "CPP-034 FIX" note on ModeRTL::update()), from ModeRTL::
    // update() too, for the exact same reason: RTL is an auto-throttle
    // mode just like AUTO (both call calc_nav_pitch()/calc_throttle()),
    // so it needs the exact same "drive TECS before reading its demand"
    // treatment - a call ModeRTL::update() was missing from CPP-031 slice
    // 6 (when RTL was added) until CPP-034 fixed it. Not "exclusively"
    // ModeAUTO's anymore; kept as ONE shared function rather than forking
    // an RTL-specific copy since RTL's own needs (drive TECS toward
    // aparm.airspeed_cruise at the current target_altitude_cm) are
    // identical to AUTO's - RTL has no mission leg of its own to want a
    // different airspeed target from.
    //
    // EXCLUDED (documented, not silently dropped): barometer.update()/
    // sink-rate low-pass/parachute (no such subsystems); update_flight_
    // stage()/LAND flight-stage distance-beyond-land-wp (no landing
    // subsystem); the RTL climb-min boost - upstream's update_alt() bumps
    // the TECS target altitude during an RTL climb-out; this port's
    // RTL_CLIMB_MIN equivalent (ModeRTL::update(), mode.hpp) only limits
    // roll angle until the climb threshold is reached, it does not touch
    // the TECS altitude target - still a real, standing exclusion as of
    // CPP-034, just no longer because RTL doesn't exist;
    // DO_CHANGE_SPEED's new_airspeed_cm override (no do-commands in this
    // slice's MissionItem vocabulary, so mode_auto_target_airspeed_cm()'s
    // real body always falls to its own "fallover to normal airspeed"
    // branch: aparm.airspeed_cruise); groundspeed-undershoot airspeed
    // nudging and throttle-nudging (both need subsystems - groundspeed-
    // undershoot tracking, aux-function dispatch - this port doesn't
    // have). The airspeed_stall/airspeed_min lower-bound clamp on the
    // target IS kept below (not excluded) - it needs nothing this port
    // lacks.
    void update_auto_speed_height(const StabilizeInputs& in) {
        const tecs::TecsInputs tecs_in = build_tecs_inputs(in);
        tecs.update_50hz(tecs_in);

        // upstream: Plane::mode_auto_target_airspeed_cm()'s real fallback
        // (no DO_CHANGE_SPEED override in this slice - see above), then
        // calc_airspeed_errors()'s own airspeed_lower_bound clamp.
        const float airspeed_lower_bound = aparm.airspeed_stall > 0.0f ? aparm.airspeed_stall : aparm.airspeed_min;
        const float airspeed_target = math::constrain_value(aparm.airspeed_cruise, airspeed_lower_bound, aparm.airspeed_max);
        const std::int32_t eas_dem_cm = static_cast<std::int32_t>(airspeed_target * 100.0f);

        // Same real per-loop necessity SLICE 2's own "SURPRISING UPSTREAM
        // FINDING #3" documents - Tecs's throttle floor decays back toward
        // reverse-permissive every cycle unless re-asserted.
        if (!have_reverse_thrust()) {
            tecs.set_throttle_min(0.0f);
        }

        tecs.update_pitch_throttle(relative_target_altitude_cm(), eas_dem_cm, in.current_altitude_m, aerodynamic_load_factor,
                                    tecs_in);
    }

    // =====================================================================
    // CPP-031 SLICE 6 (ModeRTL) - see file banner addendum for the full
    // design rationale (home/set_home(), current_loc.alt now being real
    // data, do_RTL()'s rally/terrain/alt-slope exclusions, update_loiter()
    // 's single-mode-check simplification, the LoiterState/RtlState
    // structs).
    // =====================================================================

    // upstream: Plane::get_RTL_altitude_cm() (altitude.cpp, ~line 99),
    // read in full - a direct, faithful port at upstream's own real
    // zero-arg signature (current_loc.alt is now real data, not a dead
    // field needing an explicit substitute parameter - see file banner).
    [[nodiscard]] std::int32_t get_RTL_altitude_cm() const {
        if (aparm.rtl_altitude < 0.0f) {
            return current_loc.alt;
        }
        return static_cast<std::int32_t>(aparm.rtl_altitude * 100.0f) + home.alt;
    }

    // upstream: Plane::do_RTL() (commands_logic.cpp), read in full - the
    // in-scope subset (see file banner's "DO_RTL()" note for the rally/
    // terrain/alt-slope exclusions).
    void do_RTL(std::int32_t rtl_altitude_amsl_cm) {
        next_wp_crosstrack = false;
        crosstrack = false;
        prev_WP_loc = current_loc;

        // upstream: calc_best_rally_or_home_location() - collapses to its
        // own real #else (no-rally-points) branch, `Location{home.lat,
        // home.lng, rtl_home_alt_amsl_cm, ABSOLUTE}` - see file banner.
        next_WP_loc = Location(home.lat, home.lng, rtl_altitude_amsl_cm, Location::AltFrame::ABSOLUTE);

        // fix_terrain_WP()/setup_terrain_target_alt()/set_target_altitude_
        // location() - all terrain-relative/altitude-slope machinery this
        // port doesn't have; the flat equivalent set_next_WP() already
        // established (SLICE 2's own "ALTITUDE SLOPE" note) is reproduced
        // directly here instead.
        target_altitude_cm = next_WP_loc.alt;

        // CPP-031 SLICE 10: this exact three-line sign-setting block is
        // byte-for-byte duplicated by upstream's own Plane::
        // do_loiter_at_location() (commands_logic.cpp) - factored into a
        // shared helper (below, this file's own "CPP-031 SLICE 10
        // (ModeLOITER)" section, right after update_loiter()) rather than
        // duplicated a third time in this port. See that section's own doc
        // comment and the file banner's "CPP-031 SLICE 10 ADDENDUM" note
        // ("DO_LOITER_AT_LOCATION()") for the full reasoning.
        set_loiter_direction_from_loiter_radius();

        // setup_alt_slope() - deferred, see SLICE 2's note; nothing left
        // to do here (target_altitude_cm was just set above).
        setup_turn_angle();
    }

    // upstream: Plane::update_loiter_update_nav() (navigation.cpp,
    // "method intended to be used by update_loiter"), read in full - see
    // file banner's "UPDATE_LOITER()" note for the multi-mode-check ->
    // `crosstrack` simplification and the HAL_QUADPLANE_ENABLED exclusion.
    void update_loiter_update_nav(std::uint16_t radius, const StabilizeInputs& in) {
        const nav::L1Inputs l1_in = build_l1_inputs(in);
        if (loiter.start_time_ms == 0 && crosstrack
            && current_loc.get_distance(next_WP_loc) > 3.0f * nav_controller.loiter_radius(static_cast<float>(radius), l1_in)) {
            // never reached the loiter point yet, and crosstracking a
            // real leg toward it (moot in this slice's own tests -
            // do_RTL() always resets crosstrack=false - but reproduced
            // faithfully for a future caller that DOES crosstrack into
            // RTL) - navigate to it like a normal waypoint.
            nav_controller.update_waypoint(prev_WP_loc, next_WP_loc, l1_in);
            return;
        }
        nav_controller.update_loiter(next_WP_loc, static_cast<float>(radius), loiter.direction, l1_in);
    }

    // upstream: Plane::update_loiter() (navigation.cpp), read in full.
    // `auto_state.wp_proportion > 1`'s alternate "reached" condition is
    // dropped - see file banner's "UPDATE_LOITER()" note (no such field
    // in this port, and RTL has no mission leg to compute a proportion
    // along). GCS/quadplane-guided-mode branches on the "just reached"
    // edge are dropped too - no GCS/quadplane subsystem.
    void update_loiter(std::uint16_t radius, const StabilizeInputs& in) {
        if (radius <= 1) {
            // if radius is <=1 then use the general loiter radius. if
            // it's small, use the real upstream default.
            radius = (std::fabs(aparm.loiter_radius) <= 1.0f) ? static_cast<std::uint16_t>(kLoiterRadiusDefault)
                                                                : static_cast<std::uint16_t>(std::fabs(aparm.loiter_radius));
            if (next_WP_loc.loiter_ccw == 1) {
                loiter.direction = -1;
            } else {
                loiter.direction = (aparm.loiter_radius < 0.0f) ? -1 : 1;
            }
        }

        // the radius actually being used by the controller is required by
        // other functions.
        loiter.radius = static_cast<float>(radius);

        update_loiter_update_nav(radius, in);

        if (loiter.start_time_ms == 0 && nav_controller.reached_loiter_target()) {
            // we've reached the target, start the timer.
            loiter.start_time_ms = in.now_ms;
        }
    }

    // =====================================================================
    // CPP-031 SLICE 10 (ModeLOITER) - see file banner addendum for the full
    // design rationale (do_loiter_at_location()'s reuse of do_RTL()'s own
    // sign-setting logic, the update_auto_speed_height() divergence from
    // upstream's literal mode_loiter.cpp, loiter_angle_reset()/
    // isHeadingLinedUp()/update_target_altitude()'s exclusions, and the
    // update_loiter_update_nav() second-caller safety re-verification).
    // =====================================================================

    // upstream: the identical `if (aparm.loiter_radius < 0) loiter.
    // direction = -1; else loiter.direction = 1;` block, duplicated
    // verbatim by BOTH Plane::do_RTL() (commands_logic.cpp, ~line 337) and
    // Plane::do_loiter_at_location() (commands_logic.cpp, ~line 952) -
    // verified by reading both directly. Factored into one shared helper
    // here rather than duplicated a third time in this port - do_RTL()
    // (above) now calls this too, matching build_l1_inputs()'s own
    // factoring precedent (SLICE 4's own file banner note).
    void set_loiter_direction_from_loiter_radius() { loiter.direction = (aparm.loiter_radius < 0.0f) ? -1 : 1; }

    // upstream: Plane::do_loiter_at_location() (commands_logic.cpp, ~line
    // 952, read in full) - genuinely trivial: set loiter.direction from
    // aparm.loiter_radius's sign (see set_loiter_direction_from_loiter_
    // radius() just above), then start a loiter centered on wherever the
    // aircraft currently is. ModeLOITER::enter()'s (mode.hpp) sole caller.
    void do_loiter_at_location() {
        set_loiter_direction_from_loiter_radius();
        next_WP_loc = current_loc;
    }

    // =====================================================================
    // CPP-031 SLICE 7 (real mode-switching) - see file banner addendum for
    // the full design rationale (why Mode's class hierarchy is declared
    // above rather than in mode.hpp, enter()/exit() becoming real virtual
    // methods, set_mode()'s ModeReason-dropping, control_mode's default,
    // the mission-complete-to-RTL trigger, and the home-before-auto-RTL
    // fallback).
    // =====================================================================

    // upstream: Plane::mode_manual/mode_fbwa/mode_fbwb/mode_cruise/
    // mode_auto/mode_rtl (Plane.h) - six real, typed, non-allocating
    // members (ADR-0012: no unique_ptr<Mode>, no heap allocation), each
    // constructed with `*this`. Safe construction order: Mode's own
    // constructor (`explicit Mode(Plane& plane) : plane_(plane) {}`,
    // above) does nothing but store the reference - verified directly, no
    // Mode/ModeXXX constructor in this port reads ANY Plane state at
    // construction time (all six use the inherited `using Mode::Mode;`
    // with no constructor body of their own) - so constructing these six
    // members here, referencing `*this` mid-Plane-construction, is exactly
    // as safe as the ticket's own description of the standard pattern.
    // Declared LAST among Plane's data members (after every subsystem a
    // future mode might reference) purely for readability - declaration
    // order has NO correctness bearing here since nothing reads through
    // the reference until well after full construction.
    ModeManual mode_manual{*this};
    ModeFBWA mode_fbwa{*this};
    ModeFBWB mode_fbwb{*this};
    ModeCRUISE mode_cruise{*this};
    ModeAUTO mode_auto{*this};
    ModeRTL mode_rtl{*this};
    // CPP-031 SLICE 10 - see file banner's own "CPP-031 SLICE 10 ADDENDUM"
    // ("RC-FAILSAFE CLASSIFICATION" note) for a real, named, not-yet-wired
    // gap this addition surfaces: rc_failsafe_short_on_event() (below)
    // does not yet classify this mode.
    ModeLOITER mode_loiter{*this};

    // upstream: Plane::control_mode (Plane.h), `Mode *control_mode =
    // &mode_initializing;` - see file banner's "CONTROL_MODE'S DEFAULT"
    // note for why this port defaults to `&mode_manual` instead (no
    // ModeInitializing/AHRS-settling-gate concept in this port - every
    // mode has always been immediately runnable from tick 1). A raw,
    // non-owning pointer into one of the six members directly above -
    // legal here since `mode_manual` is already fully constructed by the
    // time this default member initializer runs (declared immediately
    // after it, and default member initializers execute in declaration
    // order). mode.hpp's tick() dispatches through this pointer every
    // call (see mode.hpp's own tick() comment) - a caller changes the
    // active mode by calling set_mode() below, never by writing this
    // pointer directly (set_mode() is what makes the switch OBSERVABLE
    // and SAFE - enter()-fails rollback, old-mode exit() - a direct write
    // here would skip all of that).
    Mode* control_mode = &mode_manual;

    // CPP-031 slice 8 (RC short failsafe) - see this file's own "CPP-031
    // SLICE 8 ADDENDUM" for the full design. FailsafeState is this port's
    // reduced equivalent of upstream's Plane::failsafe struct (Plane.h) -
    // only the fields the real, in-scope RC-short-failsafe logic below
    // actually reads or writes. failsafe_saved_mode/mode_set_by_failsafe
    // are this port's own substitute for upstream's saved_mode_number/
    // ModeReason - see rc_failsafe_short_off_event()'s own doc comment
    // below for the full design rationale.
    struct FailsafeState {
        bool rc_failsafe = false;                       // upstream: failsafe.rc_failsafe
        bool short_failsafe_active = false;              // upstream: failsafe.state != FAILSAFE_NONE, reduced to the NONE/SHORT distinction this slice needs
        std::uint8_t throttle_counter = 0;               // upstream: failsafe.throttle_counter
        std::uint32_t last_valid_rc_ms = 0;              // upstream: failsafe.last_valid_rc_ms
        std::uint32_t last_seen_input_update_count = 0;  // NEW - see update_throttle_failsafe()'s own "NEW-FRAME DETECTION" note
    };
    FailsafeState failsafe;

    // CPP-031 slice 8 - see rc_failsafe_short_off_event()'s own doc
    // comment below for the full mode-restoration design these two fields
    // (plus set_mode()'s new from_failsafe parameter) implement together.
    Mode* failsafe_saved_mode = nullptr;
    bool mode_set_by_failsafe = false;

    // upstream: Plane::set_mode(Mode& new_mode, const ModeReason reason)
    // (system.cpp, ~line 252-352) - see file banner's "SET_MODE()" note
    // for why the ModeReason parameter is dropped entirely (no GCS/
    // logging/notify/previous-mode-reason consumer in this port's scope),
    // and the HAL_QUADPLANE_ENABLED/AP_FENCE_ENABLED/FLTMODE_GCSBLOCK
    // exclusions. The real, in-scope core logic is reproduced exactly,
    // including upstream's own ordering (tentative swap BEFORE calling
    // enter(), roll back to the old mode if it returns false, exit() the
    // old mode only on success).
    //
    // CPP-031 SLICE 8: gained ONE new, DEFAULTED parameter, from_failsafe
    // (every pre-existing call site is therefore source- and behavior-
    // unchanged) - the minimal substitute for upstream's ModeReason this
    // slice needs. See this file's own "CPP-031 SLICE 8 ADDENDUM" and
    // rc_failsafe_short_off_event()'s doc comment for the full design.
    // `mode_set_by_failsafe` is stamped tentatively at the SAME point (and
    // rolled back together, on a failed enter()) that upstream tentatively
    // stamps `control_mode_reason = reason` - the exact same ordering,
    // just one bool instead of a whole enum.
    bool set_mode(Mode& new_mode, bool from_failsafe = false) {
        if (control_mode == &new_mode) {
            // upstream: "don't switch modes if we are already in the
            // correct mode" - AP_Notify's happy-noise-on-repeat-request
            // gating is dropped (no AP_Notify subsystem). Note this means
            // mode_set_by_failsafe is NOT updated on this early-return
            // path either - matches upstream's own control_mode_reason
            // not being updated here (system.cpp), a real, minor, shared
            // subtlety: if the failsafe's target mode already happens to
            // be the active one, that mode's "ownership" (failsafe vs
            // deliberate) doesn't change.
            return true;
        }

        Mode& old_mode = *control_mode;
        const bool old_mode_set_by_failsafe = mode_set_by_failsafe; // CPP-031 slice 8 - rolled back together with control_mode below, matching upstream's own control_mode_reason/previous_mode_reason rollback (system.cpp)

        // upstream: "update control_mode assuming success".
        control_mode = &new_mode;
        mode_set_by_failsafe = from_failsafe;

        if (!new_mode.enter()) {
            // upstream: "we failed entering new mode, roll back to old".
            // Not exercised by any of this port's six real modes today
            // (none has a real failure condition - see Mode::enter()'s own
            // doc comment above) but real, live code, not a stub -
            // verified by a dedicated rollback test using a test-only
            // failing mode (vehicle_test.cpp).
            control_mode = &old_mode;
            mode_set_by_failsafe = old_mode_set_by_failsafe;
            return false;
        }

        // upstream: "exit previous mode".
        old_mode.exit();
        return true;
    }

    // upstream: Plane::rc_throttle_value_ok() (radio.cpp ~line 333). See
    // file banner's "CPP-031 SLICE 8 ADDENDUM".
    [[nodiscard]] bool rc_throttle_value_ok() const {
        if (aparm.throttle_fs_enabled == ThrFailsafe::Disabled) {
            return true;
        }
        const rc::RcChannel* thr = rc_channels.channel(kChannelThrottle);
        if (thr->reversed) {
            return thr->radio_in < aparm.throttle_fs_value;
        }
        return thr->radio_in > aparm.throttle_fs_value;
    }

    // upstream: Plane::rc_failsafe_active() (radio.cpp ~line 348). ONE
    // necessary signature divergence: an explicit `now_ms` parameter
    // rather than reading a singleton clock (ADR-0012, matching this
    // port's now-ms-parameter convention throughout - e.g. every other
    // `now_ms`-taking method in this file).
    [[nodiscard]] bool rc_failsafe_active(std::uint32_t now_ms) const {
        if (!rc_throttle_value_ok()) {
            return true;
        }
        if (now_ms - failsafe.last_valid_rc_ms > aparm.rc_fs_timeout_ms) {
            // we haven't had a valid RC frame for rc_fs_timeout_ms.
            return true;
        }
        return false;
    }

    // upstream: the guard + 10-up/3-down debounce block inside Plane::
    // control_failsafe() (radio.cpp ~lines 227-262), called every tick
    // from read_radio(), PLUS read_radio()'s own last_valid_rc_ms update
    // (radio.cpp ~lines 128-132) - folded into one method here rather
    // than kept as two, since both are always called together every tick
    // (mode.hpp's tick()). See file banner's "CONTROL_FAILSAFE()'S
    // PILOT-STICK-TO-TRIM RESET" note for what's deliberately NOT ported
    // from control_failsafe() (out of scope per the ticket), and
    // "UPDATE_THROTTLE_FAILSAFE()'S ALLOW_FAILSAFE_BYPASS" for why the
    // guard below is `!has_had_input` alone rather than upstream's full
    // `allow_failsafe_bypass && !has_had_input` compound.
    //
    // NEW-FRAME DETECTION - A NECESSARY DEPARTURE FROM RE-CHECKING
    // RcInput::new_input(), TRACED DIRECTLY AGAINST THIS PORT'S OWN TEST
    // HARNESS, NOT ASSUMED: upstream's real gate for updating
    // last_valid_rc_ms is "did read_input() see a genuinely new frame
    // THIS call" (RC_Channels::read_input()'s own bool return). tick()
    // (mode.hpp) already calls `rc_channels.read_input(...)` as its own
    // step 1 - but vehicle_test.cpp's OWN set_sticks() helper (used by
    // nearly every existing test, predating this slice) ALREADY calls
    // `rc_channels.read_input()` directly, itself, before tick() ever
    // runs. RcInput::new_input() is a genuine single-shot flag ("true
    // exactly once, however many times someone asks" - hal/rc_input.hpp's
    // own doc comment) - so by the time tick()'s OWN read_input() call
    // runs, the flag is ALREADY consumed and it always returns false, NOT
    // because no frame arrived, but because set_sticks() already looked.
    // Gating last_valid_rc_ms's refresh on tick()'s own read_input()
    // return value would therefore report "stale" on EVERY tick of EVERY
    // existing closed-loop test that uses set_sticks() (nearly all of
    // them) - tripping this slice's own failsafe within ~200ms of
    // simulated time and silently breaking every one of them (discovered
    // by tracing set_sticks()'s real body, not assumed). The fix touches
    // neither set_sticks() nor any existing test (both forbidden) -
    // instead RcChannels gained ONE new, purely-additive, non-consuming
    // counter, input_update_count() (rc_channels.hpp, see its own file
    // banner addendum), incremented every time read_input() processes a
    // real new frame REGARDLESS OF WHICH CALLER triggered it. Comparing it
    // against `failsafe.last_seen_input_update_count` (this Plane's own
    // last-checked snapshot) correctly reports "yes, a new frame arrived
    // since I last checked" even when THIS call didn't personally consume
    // RcInput's flag - verified directly by this slice's own dedicated
    // staleness test (vehicle_test.cpp), which confirms the timeout path
    // fires correctly when set_sticks() is simply never called for over a
    // second of simulated ticks (the counter then genuinely never
    // advances).
    void update_throttle_failsafe(std::uint32_t now_ms) {
        if (rc_channels.input_update_count() != failsafe.last_seen_input_update_count) {
            failsafe.last_seen_input_update_count = rc_channels.input_update_count();
            if (rc_throttle_value_ok()) {
                failsafe.last_valid_rc_ms = now_ms;
            }
        }

        const bool has_had_input = rc_channels.has_valid_input();
        if ((aparm.throttle_fs_enabled != ThrFailsafe::Enabled && !failsafe.rc_failsafe) || !has_had_input) {
            // If throttle fs not enabled and not in failsafe, or we've
            // never yet seen a valid RC frame - see file banner's
            // "ALLOW_FAILSAFE_BYPASS" note for why this port's guard is
            // simpler than upstream's real compound condition.
            return;
        }

        if (rc_failsafe_active(now_ms)) {
            // we detect a failsafe from radio - throttle has dropped
            // below the mark (or the last valid frame is stale).
            failsafe.throttle_counter++;
            if (failsafe.throttle_counter == 10) {
                failsafe.rc_failsafe = true;
            }
            if (failsafe.throttle_counter > 10) {
                failsafe.throttle_counter = 10;
            }
        } else if (failsafe.throttle_counter > 0) {
            // we are no longer in failsafe condition, but we need to
            // recover quickly - note the asymmetric 3 cap vs the 10 cap
            // above, read exactly from upstream, not assumed symmetric.
            failsafe.throttle_counter--;
            if (failsafe.throttle_counter > 3) {
                failsafe.throttle_counter = 3;
            }
            if (failsafe.throttle_counter == 0) {
                failsafe.rc_failsafe = false;
            }
        }
    }

    // upstream: Plane::check_short_rc_failsafe() (system.cpp ~line 411,
    // full) - reacts to failsafe.rc_failsafe's transitions (set by
    // update_throttle_failsafe() above) by calling the on/off event
    // handlers. `flight_stage != AP_FixedWing::FlightStage::LAND` is
    // dropped - see file banner's own note - always true here.
    void check_short_rc_failsafe() {
        if (aparm.fs_action_short != FsActionShort::Disabled && !failsafe.short_failsafe_active) {
            if (failsafe.rc_failsafe) {
                rc_failsafe_short_on_event();
            }
        }

        if (failsafe.short_failsafe_active) {
            if (!failsafe.rc_failsafe || aparm.fs_action_short == FsActionShort::Disabled) {
                rc_failsafe_short_off_event();
            }
        }
    }

    // upstream: the identical if/else-if/else body upstream repeats
    // verbatim across the MANUAL-group and AUTO-group case blocks of
    // rc_failsafe_short_on_event() (a third, quadplane-only repetition is
    // excluded, no quadplane) - factored into one helper since this port
    // has no reason to duplicate it. mode_circle substituted with
    // mode_rtl - see file banner's "NO CIRCLE MODE" note: a real, named
    // gap, not silently papered over.
    void apply_fs_action_short() {
        if (aparm.fs_action_short == FsActionShort::Fbwa) {
            set_mode(mode_fbwa, /*from_failsafe=*/true);
        } else if (aparm.fs_action_short == FsActionShort::Fbwb) {
            set_mode(mode_fbwb, /*from_failsafe=*/true);
        } else {
            // upstream: `set_mode(mode_circle, ...)` - BESTGUESS(0),
            // CIRCLE(1), and DISABLED(3) all fall here (upstream's real
            // if/else-if/else only special-cases FBWA/FBWB explicitly) -
            // ported literally, not "fixed", per this port's own "port
            // fixes bugs in the port, not upstream" rule. See file
            // banner's "NO CIRCLE MODE" note for the RTL substitution.
            set_mode(mode_rtl, /*from_failsafe=*/true);
        }
    }

    // upstream: Plane::rc_failsafe_short_on_event() (events.cpp ~line 21,
    // read in full). See file banner's "CPP-031 SLICE 8 ADDENDUM" for the
    // full switch-case-by-switch-case trace of which of this port's six
    // modes fall into which upstream case group, the CIRCLE->RTL
    // substitution, and AUTO's own real (traced, not assumed)
    // fs_action_short handling.
    void rc_failsafe_short_on_event() {
        failsafe.short_failsafe_active = true; // upstream: failsafe.state = FAILSAFE_SHORT
        failsafe_saved_mode = control_mode;    // upstream: failsafe.saved_mode_number = control_mode->mode_number(), unconditionally - see file banner

        if (control_mode == &mode_manual || control_mode == &mode_fbwa || control_mode == &mode_fbwb ||
            control_mode == &mode_cruise) {
            // upstream's MANUAL/STABILIZE/ACRO/FLY_BY_WIRE_A/AUTOTUNE/
            // FLY_BY_WIRE_B/CRUISE/TRAINING case group - the four of
            // those eight upstream modes this port has. The
            // emergency_landing-overrides-to-FBWA branch is dropped (no
            // emergency-landing subsystem, file banner).
            apply_fs_action_short();
        } else if (control_mode == &mode_auto) {
            // upstream's AUTO/AUTOLAND/AVOID_ADSB/GUIDED/LOITER/THERMAL
            // case group - AUTO is the only one of those six this port
            // has. failsafe_in_landing_sequence() is dropped (no
            // landing-sequence subsystem - always false, i.e. never skip,
            // per the ticket's own instruction), so upstream's real `if
            // (g.fs_action_short != FS_ACTION_SHORT_BESTGUESS)` gate is
            // reproduced unguarded by anything else - AUTO is NOT one of
            // the "never take short failsafe action" modes (that group is
            // CIRCLE/TAKEOFF/RTL/quadplane-LAND-modes/INITIALISING only);
            // it gets the SAME FBWA/FBWB/circle action as the group above,
            // just additionally gated on fs_action_short != BESTGUESS.
            if (aparm.fs_action_short != FsActionShort::BestGuess) {
                apply_fs_action_short();
            }
        }
        // else: control_mode == &mode_rtl - upstream's CIRCLE/TAKEOFF/
        // RTL/(quadplane QLAND/QRTL/LOITER_ALT_QLAND)/INITIALISING case:
        // "these modes never take any short failsafe action and
        // continue" - a real, traced no-op (RTL is already the safe,
        // autonomous, no-pilot-needed response), not a gap.
        //
        // CPP-031 SLICE 10 - NOT YET EXHAUSTIVE, A REAL NAMED GAP: this
        // `else` implicitly assumes "not manual-group, not AUTO" means
        // RTL - true when this function was written (only six modes
        // existed), but `mode_loiter` is now also a real, settable-via-
        // set_mode() Plane member and would silently fall in here too,
        // getting RTL's "never act" treatment instead of upstream's real
        // AUTO-like action (upstream groups LOITER with AUTO/GUIDED, not
        // CIRCLE/RTL - see file banner's "CPP-031 SLICE 10 ADDENDUM",
        // "RC-FAILSAFE CLASSIFICATION" note for the full trace). No
        // shipped code path can reach this today - nothing wires a
        // failsafe/aux-switch/mission-command target into mode_loiter yet
        // - but it is a real, reachable-in-principle gap now, not merely
        // hypothetical, flagged here rather than left for a future reader
        // to rediscover.
    }

    // upstream: Plane::rc_failsafe_short_off_event() (events.cpp ~line
    // 243, full).
    //
    // MODE-RESTORATION DESIGN - THIS PORT'S SUBSTITUTE FOR
    // saved_mode_number/ModeReason: upstream restores by `saved_mode_
    // number` (a Mode::Number enum) via set_mode_by_number(), gated on
    // `control_mode_reason == ModeReason::RADIO_FAILSAFE` (i.e. don't
    // restore over a DELIBERATE later mode change made while failsafe was
    // active). This port has no number-indexed mode lookup and no
    // ModeReason tracking at all (commit 6db7924, CPP-031 slice 7,
    // deliberately dropped the whole enum - no consumer anywhere in this
    // port's scope ever read "why" a mode changed, until now).
    // Resurrecting the whole enum for ONE boolean question ("was the
    // CURRENT mode chosen by us, or by something else since?") would be
    // exactly the dead plumbing that slice's own note argued against.
    // Instead:
    //   - `Mode* failsafe_saved_mode` (above) - a direct pointer to
    //     whichever mode was active the moment the failsafe triggered,
    //     substituting for saved_mode_number's enum-plus-lookup-table
    //     indirection (this port's set_mode() already takes a `Mode&`
    //     directly, so a pointer needs no separate lookup step at all).
    //   - `bool mode_set_by_failsafe` (above) + `set_mode(Mode&, bool
    //     from_failsafe = false)`'s new (defaulted, so every pre-existing
    //     call site is unchanged) second parameter - the minimal
    //     substitute for `control_mode_reason == ModeReason::
    //     RADIO_FAILSAFE`. set_mode() stamps `mode_set_by_failsafe =
    //     from_failsafe` at the SAME point (and with the SAME
    //     tentative-before-enter()/rolled-back-on-failure ordering)
    //     upstream stamps `control_mode_reason = reason` - see set_mode()
    //     itself above. rc_failsafe_short_on_event() (via apply_fs_
    //     action_short()) calls set_mode() with from_failsafe=true; EVERY
    //     OTHER caller (a pilot's/autopilot's own deliberate set_mode()
    //     call, e.g. ModeAUTO's mission-complete-to-RTL transition,
    //     mode.hpp) uses the default `false` - so if one of those fires
    //     WHILE this failsafe is still active, mode_set_by_failsafe
    //     correctly flips back to false, and the check below correctly
    //     refuses to stomp that deliberate later choice once the RC link
    //     eventually recovers - the exact behavior the ticket asked for,
    //     verified by a dedicated test (vehicle_test.cpp).
    //   - Restoring via a plain `set_mode(*failsafe_saved_mode)` call
    //     (from_failsafe defaulting false) is deliberate: the restored
    //     mode is no longer "owned" by the failsafe machinery once
    //     restored, so a SECOND failsafe triggering later correctly
    //     saves/restores fresh rather than seeing a stale
    //     mode_set_by_failsafe=true left over from the restoration
    //     itself.
    void rc_failsafe_short_off_event() {
        failsafe.short_failsafe_active = false; // upstream: failsafe.state = FAILSAFE_NONE
        if (mode_set_by_failsafe && failsafe_saved_mode != nullptr) {
            // upstream: `if (control_mode_reason == ModeReason::
            // RADIO_FAILSAFE) { set_mode_by_number(failsafe.saved_mode_
            // number, ModeReason::RADIO_FAILSAFE_RECOVERY); }`
            set_mode(*failsafe_saved_mode);
        }
    }

private:
    // Used only during the member-init list above - see file banner's
    // "DECLARATION-ORDER CONSTRAINT" note for why this is safe.
    [[nodiscard]] fw_control::FwAparm fw_aparm() const {
        fw_control::FwAparm a;
        a.airspeed_min = aparm.airspeed_min;
        a.airspeed_max = aparm.airspeed_max;
        a.roll_limit_deg = aparm.roll_limit_deg;
        return a;
    }

    [[nodiscard]] tecs::Tecs::FixedWingParams tecs_aparm() const {
        tecs::Tecs::FixedWingParams a;
        a.airspeed_min = aparm.airspeed_min;
        a.airspeed_max = aparm.airspeed_max;
        a.airspeed_stall = aparm.airspeed_stall;
        a.stall_prevention = aparm.stall_prevention;
        a.throttle_cruise = aparm.throttle_cruise;
        a.pitch_limit_max = aparm.pitch_limit_max_deg;
        a.pitch_limit_min = aparm.pitch_limit_min_deg;
        return a;
    }
};

} // namespace fwcpp::vehicle
