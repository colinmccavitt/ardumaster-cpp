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

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/fw_control/pitch_controller.hpp>
#include <fwcpp/fw_control/roll_controller.hpp>
#include <fwcpp/fw_control/yaw_controller.hpp>
#include <fwcpp/gps/gps.hpp>
#include <fwcpp/hal/hal_context.hpp>
#include <fwcpp/math/scalar.hpp>
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
    bool armed_and_safety_off = false; // upstream: arming.is_armed_and_safety_off()
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

    // MUST precede roll_controller/pitch_controller/yaw_controller/tecs -
    // see file banner's "DECLARATION-ORDER CONSTRAINT" note.
    FixedWingTunables aparm;

    fw_control::RollController roll_controller;
    fw_control::PitchController pitch_controller;
    fw_control::YawController yaw_controller;
    tecs::Tecs tecs;

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
